// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"
#include "stage_profiler.h"   // v2 Phase 0: measurement_build timing
#include "cuda/cuda_representative_ivox.h"
#include "ivox/representative_ivox.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <sstream>

PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
std::vector<int> time_seq;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
std::vector<V3D> pbody_list;
std::vector<PointVector> Nearest_Points; 
std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
bool   point_selected_surf[100000] = {0};
std::vector<M3D> crossmat_list;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
esekfom::esekf<state_input, 24, input_ikfom> kf_input;
esekfom::esekf<state_output, 30, input_ikfom> kf_output;
input_ikfom input_in;
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;  
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
double G_m_s2 = 9.81;

namespace {

enum class ActiveMapBackend {
	ORIGINAL_CPU,
	REPRESENTATIVE_CPU,
	REPRESENTATIVE_CUDA,
};

ActiveMapBackend active_map_backend = ActiveMapBackend::ORIGINAL_CPU;
std::unique_ptr<batchlio::RepresentativeIvox> representative_cpu_map;
std::unique_ptr<batchlio::CudaRepresentativeIvox> representative_cuda_map;
std::vector<batchlio::Point3f> representative_insert_buffer;
std::vector<batchlio::Point3f> cuda_query_buffer;
std::vector<batchlio::NeighborSet> cuda_result_buffer;
std::string map_backend_status = "original_cpu";
bool cuda_failure_reported = false;
std::uint64_t cuda_verified_queries = 0;
std::uint64_t cuda_query_mismatches = 0;
std::uint64_t cuda_gpu_query_batches = 0;
std::uint64_t cuda_gpu_query_points = 0;
std::uint64_t cuda_cpu_query_batches = 0;
std::uint64_t cuda_cpu_query_points = 0;
std::atomic<std::uint64_t> preexpanded_verified_queries{0};
std::atomic<std::uint64_t> preexpanded_query_mismatches{0};

batchlio::RepresentativeNearbyType ParseNearbyType(int value)
{
	switch (value)
	{
		case 0: return batchlio::RepresentativeNearbyType::CENTER;
		case 6: return batchlio::RepresentativeNearbyType::NEARBY6;
		case 18: return batchlio::RepresentativeNearbyType::NEARBY18;
		case 26: return batchlio::RepresentativeNearbyType::NEARBY26;
		default: throw std::invalid_argument("representative_nearby_type must be 0, 6, 18, or 26");
	}
}

batchlio::RepresentativeIvoxConfig MakeRepresentativeConfig()
{
	if (representative_map_capacity <= 0)
	{
		throw std::invalid_argument("representative_map_capacity must be positive");
	}
	batchlio::RepresentativeIvoxConfig config;
	config.resolution = representative_map_resolution;
	config.max_points_per_voxel = representative_max_points_per_voxel;
	config.nearby_type = ParseNearbyType(representative_nearby_type);
	config.capacity = static_cast<std::size_t>(representative_map_capacity);
	config.max_range = static_cast<float>(representative_max_range);
	config.preexpand_neighborhoods = representative_preexpand_neighborhoods;
	batchlio::RepresentativeIvox::ValidateConfig(config);
	return config;
}

void NeighborSetToPointVector(const batchlio::NeighborSet& source, PointVector& destination)
{
	destination.clear();
	destination.reserve(batchlio::kRepresentativeIvoxMaxNeighbors);
	for (int i = 0; i < source.count; ++i)
	{
		PointType point;
		point.x = source.points[i].x;
		point.y = source.points[i].y;
		point.z = source.points[i].z;
		destination.emplace_back(point);
	}
}

void QueryRepresentativeCpu(const PointType& query, PointVector& neighbors)
{
	const batchlio::Point3f input{query.x, query.y, query.z};
	batchlio::NeighborSet result = representative_cpu_map->Query(input);
	if (representative_verify_preexpanded &&
		representative_cpu_map->config().preexpand_neighborhoods)
	{
		const batchlio::NeighborSet direct = representative_cpu_map->QueryDirect(input);
		bool equal = result.count == direct.count;
		for (int i = 0; equal && i < result.count; ++i)
		{
			equal = result.points[i].x == direct.points[i].x &&
				result.points[i].y == direct.points[i].y &&
				result.points[i].z == direct.points[i].z &&
				result.squared_distances[i] == direct.squared_distances[i];
		}
		preexpanded_verified_queries.fetch_add(1U, std::memory_order_relaxed);
		if (!equal)
		{
			const std::uint64_t mismatch =
				preexpanded_query_mismatches.fetch_add(1U, std::memory_order_relaxed) + 1U;
			if (mismatch <= 5U)
			{
				#pragma omp critical(batch_lio_preexpanded_verify_log)
				std::cerr << "[batch-LIO PREEXPAND VERIFY] mismatch=" << mismatch
						  << " cached_count=" << result.count
						  << " direct_count=" << direct.count << std::endl;
			}
			// Verification mode prioritizes estimator safety while preserving a
			// visible mismatch counter for diagnosis.
			result = direct;
		}
	}
	NeighborSetToPointVector(result, neighbors);
}

void QueryActiveCpuBackend(const PointType& query, PointVector& neighbors)
{
	if (active_map_backend == ActiveMapBackend::ORIGINAL_CPU || representative_cpu_map == nullptr)
	{
		ivox_->GetClosestPoint(query, neighbors, NUM_MATCH_POINTS);
		return;
	}
	QueryRepresentativeCpu(query, neighbors);
}

bool PrepareCudaQueriesForCurrentBatch()
{
	if (active_map_backend != ActiveMapBackend::REPRESENTATIVE_CUDA ||
		representative_cuda_map == nullptr || !representative_cuda_map->IsReady())
	{
		return false;
	}

	const int count = time_seq[k];
	if (count < std::max(1, cuda_min_batch_points))
	{
		++cuda_cpu_query_batches;
		cuda_cpu_query_points += static_cast<std::uint64_t>(count);
		return false;
	}
	cuda_query_buffer.resize(static_cast<std::size_t>(count));
	cuda_result_buffer.resize(static_cast<std::size_t>(count));
	// A typical 1 ms window has only ~8 points. Creating an OpenMP team for
	// these tiny copy/transform loops costs much more than the work itself.
	#pragma omp parallel for if(count >= 64) schedule(static)
	for (int j = 0; j < count; ++j)
	{
		PointType& point_body = feats_down_body->points[idx + j + 1];
		PointType& point_world = feats_down_world->points[idx + j + 1];
		pointBodyToWorld(&point_body, &point_world);
		cuda_query_buffer[static_cast<std::size_t>(j)] =
			batchlio::Point3f{point_world.x, point_world.y, point_world.z};
	}

	if (!representative_cuda_map->Query(cuda_query_buffer.data(), cuda_query_buffer.size(),
									 cuda_result_buffer.data()))
	{
		if (!cuda_failure_reported)
		{
			std::cerr << "[batch-LIO CUDA] query failed; falling back to representative_cpu: "
					  << representative_cuda_map->LastError() << std::endl;
			cuda_failure_reported = true;
		}
		active_map_backend = ActiveMapBackend::REPRESENTATIVE_CPU;
		map_backend_status = "CUDA query failed (" + representative_cuda_map->LastError() +
			"); using representative_cpu";
		++cuda_cpu_query_batches;
		cuda_cpu_query_points += static_cast<std::uint64_t>(count);
		return false;
	}
	++cuda_gpu_query_batches;
	cuda_gpu_query_points += static_cast<std::uint64_t>(count);

	if (cuda_verify_queries)
	{
		for (int j = 0; j < count; ++j)
		{
			const batchlio::NeighborSet cpu_result =
				representative_cpu_map->Query(cuda_query_buffer[static_cast<std::size_t>(j)]);
			const batchlio::NeighborSet& gpu_result = cuda_result_buffer[static_cast<std::size_t>(j)];
			bool equal = cpu_result.count == gpu_result.count;
			for (int rank = 0; equal && rank < cpu_result.count; ++rank)
			{
				const batchlio::Point3f& lhs = cpu_result.points[rank];
				const batchlio::Point3f& rhs = gpu_result.points[rank];
				equal = lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
			}
			++cuda_verified_queries;
			if (!equal)
			{
				++cuda_query_mismatches;
				if (cuda_query_mismatches <= 5)
				{
					std::cerr << "[batch-LIO CUDA VERIFY] mismatch query=" << cuda_verified_queries
							  << " cpu_count=" << cpu_result.count
							  << " gpu_count=" << gpu_result.count << std::endl;
				}
			}
		}
		if (cuda_verified_queries % 100000U < static_cast<std::uint64_t>(count))
		{
			std::cout << "[batch-LIO CUDA VERIFY] queries=" << cuda_verified_queries
					  << " mismatches=" << cuda_query_mismatches << std::endl;
		}
	}

	#pragma omp parallel for if(count >= 64) schedule(static)
	for (int j = 0; j < count; ++j)
	{
		NeighborSetToPointVector(cuda_result_buffer[static_cast<std::size_t>(j)],
							 Nearest_Points[idx + j + 1]);
	}
	return true;
}

}  // namespace

bool InitializeMeasurementMapBackend()
{
	representative_cpu_map.reset();
	representative_cuda_map.reset();
	cuda_failure_reported = false;
	cuda_verified_queries = 0;
	cuda_query_mismatches = 0;
	cuda_gpu_query_batches = 0;
	cuda_gpu_query_points = 0;
	cuda_cpu_query_batches = 0;
	cuda_cpu_query_points = 0;
	preexpanded_verified_queries.store(0U, std::memory_order_relaxed);
	preexpanded_query_mismatches.store(0U, std::memory_order_relaxed);
	active_map_backend = ActiveMapBackend::ORIGINAL_CPU;
	map_backend_status = "original_cpu";

	if (map_backend == "original_cpu" || map_backend == "original")
	{
		return true;
	}
	if (map_backend != "representative_cpu" && map_backend != "representative_cuda")
	{
		map_backend_status = "invalid map_backend='" + map_backend + "'; using original_cpu";
		return false;
	}

	try
	{
		const batchlio::RepresentativeIvoxConfig config = MakeRepresentativeConfig();
		representative_cpu_map.reset(new batchlio::RepresentativeIvox(config));
		active_map_backend = ActiveMapBackend::REPRESENTATIVE_CPU;
		map_backend_status = "representative_cpu";
		if (map_backend == "representative_cuda")
		{
			representative_cuda_map.reset(new batchlio::CudaRepresentativeIvox(config));
			if (representative_cuda_map->IsReady())
			{
				if (cuda_persistent_queries)
				{
					if (cuda_persistent_max_batch_points <= 0 ||
						!representative_cuda_map->StartPersistentQueryService(
							static_cast<std::size_t>(cuda_persistent_max_batch_points)))
					{
						map_backend_status = "persistent CUDA query service unavailable (" +
							representative_cuda_map->LastError() + "); using representative_cpu";
						active_map_backend = ActiveMapBackend::REPRESENTATIVE_CPU;
						return false;
					}
				}
				active_map_backend = ActiveMapBackend::REPRESENTATIVE_CUDA;
				map_backend_status = cuda_persistent_queries ?
					"representative_cuda(persistent)" : "representative_cuda";
				return true;
			}
			map_backend_status = "representative_cuda unavailable (" +
				representative_cuda_map->LastError() + "); using representative_cpu";
			return false;
		}
		return true;
	}
	catch (const std::exception& error)
	{
		representative_cpu_map.reset();
		representative_cuda_map.reset();
		active_map_backend = ActiveMapBackend::ORIGINAL_CPU;
		map_backend_status = std::string("representative map configuration error (") +
			error.what() + "); using original_cpu";
		return false;
	}
}

void ResetMeasurementMapBackend()
{
	if (representative_cpu_map != nullptr) representative_cpu_map->Reset();
	if (representative_cuda_map != nullptr && representative_cuda_map->IsReady() &&
		!representative_cuda_map->Reset())
	{
		active_map_backend = ActiveMapBackend::REPRESENTATIVE_CPU;
		map_backend_status = "CUDA reset failed (" + representative_cuda_map->LastError() +
			"); using representative_cpu";
	}
	cuda_failure_reported = false;
	cuda_verified_queries = 0;
	cuda_query_mismatches = 0;
	cuda_gpu_query_batches = 0;
	cuda_gpu_query_points = 0;
	cuda_cpu_query_batches = 0;
	cuda_cpu_query_points = 0;
	preexpanded_verified_queries.store(0U, std::memory_order_relaxed);
	preexpanded_query_mismatches.store(0U, std::memory_order_relaxed);
}

void AddPointsToMapBackends(const PointVector& points)
{
	// Keep the original map hot even in representative modes. It is the safe
	// runtime fallback if CUDA reports an error during a long run.
	ivox_->AddPoints(points);
	if (representative_cpu_map == nullptr || points.empty()) return;

	representative_insert_buffer.resize(points.size());
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		representative_insert_buffer[i] = batchlio::Point3f{points[i].x, points[i].y, points[i].z};
	}
	representative_cpu_map->AddPoints(representative_insert_buffer.data(),
									 representative_insert_buffer.size());

	if (active_map_backend == ActiveMapBackend::REPRESENTATIVE_CUDA &&
		representative_cuda_map != nullptr &&
		!representative_cuda_map->AddPoints(representative_insert_buffer.data(),
										 representative_insert_buffer.size()))
	{
		active_map_backend = ActiveMapBackend::REPRESENTATIVE_CPU;
		map_backend_status = "CUDA insertion failed (" + representative_cuda_map->LastError() +
			"); using representative_cpu";
		if (!cuda_failure_reported)
		{
			std::cerr << "[batch-LIO CUDA] " << map_backend_status << std::endl;
			cuda_failure_reported = true;
		}
	}
}

std::string MeasurementMapBackendDescription()
{
	std::ostringstream description;
	description << map_backend_status;
	if (representative_cpu_map != nullptr)
	{
		description << " resolution=" << representative_map_resolution
					<< " K=" << representative_max_points_per_voxel
					<< " nearby=" << representative_nearby_type
					<< " capacity=" << representative_map_capacity
					<< " preexpanded="
					<< (representative_preexpand_neighborhoods ? "true" : "false");
		if (representative_verify_preexpanded)
		{
			description << " verify_preexpanded=true";
		}
	}
	if (active_map_backend == ActiveMapBackend::REPRESENTATIVE_CUDA)
	{
		description << " cuda_min_batch_points=" << cuda_min_batch_points;
		description << " persistent="
					<< (representative_cuda_map->PersistentQueryServiceReady() ? "true" : "false");
		if (cuda_verify_queries)
		{
			description << " verify_queries=true";
		}
	}
	return description.str();
}

std::string MeasurementMapBackendRuntimeStats()
{
	std::ostringstream description;
	description << "backend=" << map_backend_status
				<< " gpu_query_batches=" << cuda_gpu_query_batches
				<< " gpu_query_points=" << cuda_gpu_query_points
				<< " cpu_query_batches=" << cuda_cpu_query_batches
				<< " cpu_query_points=" << cuda_cpu_query_points;
	if (representative_cpu_map != nullptr)
	{
		description << " cpu_voxels=" << representative_cpu_map->NumVoxels()
					<< " cached_query_voxels="
					<< representative_cpu_map->NumCachedQueryVoxels()
					<< " cached_voxel_refs="
					<< representative_cpu_map->CachedVoxelReferences()
					<< " cache_payload_mib="
					<< (representative_cpu_map->EstimatedNeighborhoodCachePayloadBytes() /
						(1024.0 * 1024.0));
	}
	if (representative_verify_preexpanded)
	{
		description << " preexpanded_verified_queries="
					<< preexpanded_verified_queries.load(std::memory_order_relaxed)
					<< " preexpanded_mismatches="
					<< preexpanded_query_mismatches.load(std::memory_order_relaxed);
	}
	if (representative_cuda_map != nullptr)
	{
		const batchlio::CudaRepresentativeIvoxStats stats = representative_cuda_map->Stats();
		description << " gpu_voxels=" << stats.voxel_count
					<< " gpu_accepted_updates=" << stats.accepted_points
					<< " gpu_rejected_updates=" << stats.rejected_points;
	}
	if (cuda_verify_queries)
	{
		description << " verified_queries=" << cuda_verified_queries
					<< " mismatches=" << cuda_query_mismatches;
	}
	return description.str();
}

Eigen::Matrix<double, 24, 24> process_noise_cov_input()
{
	Eigen::Matrix<double, 24, 24> cov;
	cov.setZero();
	cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
	cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
	cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	// MTK::get_cov<process_noise_input>::type cov = MTK::get_cov<process_noise_input>::type::Zero();
	// MTK::setDiagonal<process_noise_input, vect3, 0>(cov, &process_noise_input::ng, gyr_cov_input);// 0.03
	// MTK::setDiagonal<process_noise_input, vect3, 3>(cov, &process_noise_input::na, acc_cov_input); // *dt 0.01 0.01 * dt * dt 0.05
	// MTK::setDiagonal<process_noise_input, vect3, 6>(cov, &process_noise_input::nbg, b_gyr_cov); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	// MTK::setDiagonal<process_noise_input, vect3, 9>(cov, &process_noise_input::nba, b_acc_cov);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

Eigen::Matrix<double, 30, 30> process_noise_cov_output()
{
	Eigen::Matrix<double, 30, 30> cov;
	cov.setZero();
	cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
	cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
	cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
	cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	return cov;
}

Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	vect3 a_inertial = s.rot * (in.acc-s.ba); // .normalized()
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) = omega[i]; 
		res(i + 12) = a_inertial[i] + s.gravity[i]; 
	}
	return res;
}

Eigen::Matrix<double, 30, 1> get_f_output(state_output &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
	vect3 a_inertial = s.rot * s.acc; // .normalized()
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) = s.omg[i]; 
		res(i + 12) = a_inertial[i] + s.gravity[i]; 
	}
	return res;
}

Eigen::Matrix<double, 24, 24> df_dx_input(state_input &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	cov.template block<3, 3>(12, 3) = -s.rot*MTK::hat(acc_); // .normalized().toRotationMatrix()
	cov.template block<3, 3>(12, 18) = -s.rot; //.normalized().toRotationMatrix();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity(); // grav_matrix; 
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 
	return cov;
}

Eigen::Matrix<double, 30, 30> df_dx_output(state_output &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(12, 3) = -s.rot*MTK::hat(s.acc); // .normalized().toRotationMatrix()
	cov.template block<3, 3>(12, 18) = s.rot; //.normalized().toRotationMatrix();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity(); // grav_matrix; 
	cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity(); 
	return cov;
}

void h_model_input(state_input &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	bool match_in_map = false;
	normvec->resize(time_seq[k]);
	int effect_num_k = 0;
	const bool cuda_queries_ready = PrepareCudaQueriesForCurrentBatch();
	// batch-LIO: parallelize the per-point KNN + plane-fit pass. Each iteration writes only
	// j-indexed slots (point_selected_surf, normvec, Nearest_Points, feats_down_world), so there
	// are no cross-iteration write conflicts; pabcd is made loop-private (was shared above) and
	// effect_num_k is reduced. The Jacobian-assembly pass below stays serial (sequential row m).
	// Original iVox lookups are heavy enough to benefit from parallelism even in
	// tiny batches. Representative lookups are much cheaper, so avoid waking an
	// OpenMP team for their typical ~8-point windows.
	#pragma omp parallel for if(batch_omp && (active_map_backend == ActiveMapBackend::ORIGINAL_CPU || time_seq[k] >= 64)) schedule(dynamic) reduction(+:effect_num_k)
	for (int j = 0; j < time_seq[k]; j++)
	{
		VF(4) pabcd;
		pabcd.setZero();
		PointType &point_body_j  = feats_down_body->points[idx+j+1];
		PointType &point_world_j = feats_down_world->points[idx+j+1];
		if (!cuda_queries_ready) pointBodyToWorld(&point_body_j, &point_world_j);
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;
		{
			auto &points_near = Nearest_Points[idx+j+1];
			if (!cuda_queries_ready) QueryActiveCpuBackend(point_world_j, points_near);
			if ((points_near.size() < NUM_MATCH_POINTS)) // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) // 5)
			{
				point_selected_surf[idx+j+1] = false;
			}
			else
			{
				point_selected_surf[idx+j+1] = false;
				if (esti_plane(pabcd, points_near, plane_thr)) //(planeValid)
				{
					float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
					// V3D norm_vec;
					// M3D Rpf, pf;
					// pf = crossmat_list[idx+j+1];
					// // pf << SKEW_SYM_MATRX(p_body);
					// Rpf = s.rot * pf;
					// norm_vec << pabcd(0), pabcd(1), pabcd(2);
					// double noise_state = norm_vec.transpose() * (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec + sqrt(p_norm) * 0.001;
					// // if (p_norm > match_s * pd2 * pd2)
					// double epsilon = pd2 / sqrt(noise_state);
					// // cout << "check epsilon:" << epsilon << endl;
					// double weight = 1.0; // epsilon / sqrt(epsilon * epsilon+1);
					// if (epsilon > 1.0) 
					// {
					// 	weight = sqrt(2 * epsilon - 1) / epsilon;
					// 	pabcd(0) = weight * pabcd(0);
					// 	pabcd(1) = weight * pabcd(1);
					// 	pabcd(2) = weight * pabcd(2);
					// 	pabcd(3) = weight * pabcd(3);
					// }
					if (p_norm > match_s * pd2 * pd2)
					{
						point_selected_surf[idx+j+1] = true;
						normvec->points[j].x = pabcd(0);
						normvec->points[j].y = pabcd(1);
						normvec->points[j].z = pabcd(2);
						normvec->points[j].intensity = pabcd(3);
						effect_num_k ++;
					}
				}  
			}
		}
	}
	if (effect_num_k == 0) 
	{
		ekfom_data.valid = false;
		return;
	}
	ekfom_data.M_Noise = laser_point_cov;
	ekfom_data.h_x.resize(effect_num_k, 12);
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
	ekfom_data.z.resize(effect_num_k);
	int m = 0;
	
	for (int j = 0; j < time_seq[k]; j++)
	{
		// ekfom_data.converge = false;
		if(point_selected_surf[idx+j+1])
		{
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
			
			if (extrinsic_est_en)
			{
				V3D p_body = pbody_list[idx+j+1];
				M3D p_crossmat, p_imu_crossmat;
				p_crossmat << SKEW_SYM_MATRX(p_body);
				V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
				p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
				V3D C(s.rot.transpose() * norm_vec);
				V3D A(p_imu_crossmat * C);
				V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
			}
			else
			{   
				M3D point_crossmat = crossmat_list[idx+j+1];
				V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
				V3D A(point_crossmat * C);
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			}
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;
			
			m++;
		}
	}
	effct_feat_num += effect_num_k;
}

void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	batchlio::FrameAccumScope _hmodel_scope(profiling_enable, batchlio::g_frame_acc.measurement_build_ms);
	bool match_in_map = false;
	normvec->resize(time_seq[k]);
	int effect_num_k = 0;
	const bool cuda_queries_ready = PrepareCudaQueriesForCurrentBatch();
	// batch-LIO: parallelize the per-point KNN + plane-fit pass. Each iteration writes only
	// j-indexed slots (point_selected_surf, normvec, Nearest_Points, feats_down_world), so there
	// are no cross-iteration write conflicts; pabcd is made loop-private (was shared above) and
	// effect_num_k is reduced. The Jacobian-assembly pass below stays serial (sequential row m).
	#pragma omp parallel for if(batch_omp && (active_map_backend == ActiveMapBackend::ORIGINAL_CPU || time_seq[k] >= 64)) schedule(dynamic) reduction(+:effect_num_k)
	for (int j = 0; j < time_seq[k]; j++)
	{
		VF(4) pabcd;
		pabcd.setZero();
		PointType &point_body_j  = feats_down_body->points[idx+j+1];
		PointType &point_world_j = feats_down_world->points[idx+j+1];
		if (!cuda_queries_ready) pointBodyToWorld(&point_body_j, &point_world_j);
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;
		{
			auto &points_near = Nearest_Points[idx+j+1];
			if (!cuda_queries_ready) QueryActiveCpuBackend(point_world_j, points_near);
			
			if ((points_near.size() < NUM_MATCH_POINTS)) // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
			{
				point_selected_surf[idx+j+1] = false;
			}
			else
			{
				point_selected_surf[idx+j+1] = false;
				if (esti_plane(pabcd, points_near, plane_thr)) //(planeValid)
				{
					float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
					// V3D norm_vec;
					// M3D Rpf, pf;
					// pf = crossmat_list[idx+j+1];
					// // pf << SKEW_SYM_MATRX(p_body);
					// Rpf = s.rot * pf;
					// norm_vec << pabcd(0), pabcd(1), pabcd(2);
					// double noise_state = norm_vec.transpose() * (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec + sqrt(p_norm) * 0.001;
					// // if (p_norm > match_s * pd2 * pd2)
					// double epsilon = pd2 / sqrt(noise_state);
					// double weight = 1.0; // epsilon / sqrt(epsilon * epsilon+1);
					// if (epsilon > 1.0) 
					// {
					// 	weight = sqrt(2 * epsilon - 1) / epsilon;
					// 	pabcd(0) = weight * pabcd(0);
					// 	pabcd(1) = weight * pabcd(1);
					// 	pabcd(2) = weight * pabcd(2);
					// 	pabcd(3) = weight * pabcd(3);
					// }
					if (p_norm > match_s * pd2 * pd2)
					{
						// point_selected_surf[i] = true;
						point_selected_surf[idx+j+1] = true;
						normvec->points[j].x = pabcd(0);
						normvec->points[j].y = pabcd(1);
						normvec->points[j].z = pabcd(2);
						normvec->points[j].intensity = pabcd(3);
						effect_num_k ++;
					}
				}  
			}
		}
	}
	if (effect_num_k == 0) 
	{
		ekfom_data.valid = false;
		return;
	}
	ekfom_data.M_Noise = laser_point_cov;
	ekfom_data.h_x.resize(effect_num_k, 12);
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
	ekfom_data.z.resize(effect_num_k);
	int m = 0;
	for (int j = 0; j < time_seq[k]; j++)
	{
		// ekfom_data.converge = false;
		if(point_selected_surf[idx+j+1])
		{
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
			if (extrinsic_est_en)
			{
				V3D p_body = pbody_list[idx+j+1];
				M3D p_crossmat, p_imu_crossmat;
				p_crossmat << SKEW_SYM_MATRX(p_body);
				V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
				p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
				V3D C(s.rot.transpose() * norm_vec);
				V3D A(p_imu_crossmat * C);
				V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
			}
			else
			{   
				M3D point_crossmat = crossmat_list[idx+j+1];
				V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
				V3D A(point_crossmat * C);
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			}
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;
			
			m++;
		}
	}
	effct_feat_num += effect_num_k;
}

void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data)
{
    std::memset(ekfom_data.satu_check, false, 6);
	ekfom_data.z_IMU.block<3,1>(0, 0) = angvel_avr - s.omg - s.bg;
	ekfom_data.z_IMU.block<3,1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
    ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov, imu_meas_acc_cov, imu_meas_acc_cov;
	if(check_satu)
	{
		if(fabs(angvel_avr(0)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[0] = true; 
			ekfom_data.z_IMU(0) = 0.0;
		}
		
		if(fabs(angvel_avr(1)) >= 0.99 * satu_gyro) 
		{
			ekfom_data.satu_check[1] = true;
			ekfom_data.z_IMU(1) = 0.0;
		}
		
		if(fabs(angvel_avr(2)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[2] = true;
			ekfom_data.z_IMU(2) = 0.0;
		}
		
		if(fabs(acc_avr(0)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[3] = true;
			ekfom_data.z_IMU(3) = 0.0;
		}

		if(fabs(acc_avr(1)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[4] = true;
			ekfom_data.z_IMU(4) = 0.0;
		}

		if(fabs(acc_avr(2)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[5] = true;
			ekfom_data.z_IMU(5) = 0.0;
		}
	}
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{    
    V3D p_body(pi->x, pi->y, pi->z);
    
    V3D p_global;
	if (extrinsic_est_en)
	{	
		if (!use_imu_as_input)
		{
			p_global = kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) + kf_output.x_.pos;
		}
		else
		{
			p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) + kf_input.x_.pos;
		}
	}
	else
	{
		if (!use_imu_as_input)
		{
			p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_output.x_.pos; // .normalized()
		}
		else
		{
			p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_input.x_.pos; // .normalized()
		}
	}

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
