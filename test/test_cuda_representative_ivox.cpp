#include "cuda/cuda_representative_ivox.h"
#include "ivox/representative_ivox.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

using batchlio::CudaRepresentativeIvox;
using batchlio::NeighborSet;
using batchlio::Point3f;
using batchlio::RepresentativeIvox;
using batchlio::RepresentativeIvoxConfig;
using batchlio::RepresentativeNearbyType;

bool SameResult(const NeighborSet& cpu, const NeighborSet& gpu, std::string& reason) {
    if (cpu.count != gpu.count) {
        reason = "neighbor count differs";
        return false;
    }
    for (int i = 0; i < cpu.count; ++i) {
        const Point3f& lhs = cpu.points[i];
        const Point3f& rhs = gpu.points[i];
        const float coordinate_error =
            std::max({std::abs(lhs.x - rhs.x), std::abs(lhs.y - rhs.y), std::abs(lhs.z - rhs.z)});
        if (coordinate_error > 1e-6f ||
            std::abs(cpu.squared_distances[i] - gpu.squared_distances[i]) > 1e-4f) {
            reason = "neighbor value differs at rank " + std::to_string(i);
            return false;
        }
    }
    return true;
}

template <typename Function>
double MeanMicroseconds(int repetitions, Function&& function) {
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < repetitions; ++i) function();
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::micro>(end - begin).count() /
           static_cast<double>(repetitions);
}

}  // namespace

int main() {
    std::string runtime_reason;
    if (!CudaRepresentativeIvox::RuntimeAvailable(&runtime_reason)) {
        std::cout << "SKIP: " << runtime_reason << '\n';
        return 0;
    }

    RepresentativeIvoxConfig config;
    config.resolution = 0.5f;
    config.max_points_per_voxel = 4;
    config.nearby_type = RepresentativeNearbyType::NEARBY18;
    config.capacity = 65536;
    config.max_range = 5.0f;
    // The ROS representative CUDA backend keeps this exact cached CPU mirror
    // for small batches and runtime fallback; exercise that production config.
    config.preexpand_neighborhoods = true;

    RepresentativeIvox cpu(config);
    CudaRepresentativeIvox gpu(config);
    if (!gpu.IsReady()) {
        std::cerr << "CUDA map initialization failed: " << gpu.LastError() << '\n';
        return 1;
    }

    std::mt19937 random(0xBA7C110U);
    std::uniform_real_distribution<float> map_distribution(-20.0f, 20.0f);
    std::uniform_real_distribution<float> query_distribution(-18.0f, 18.0f);
    std::vector<Point3f> map_points(20000);
    for (Point3f& point : map_points) {
        point = Point3f{map_distribution(random), map_distribution(random), map_distribution(random)};
    }
    // Explicit signed-coordinate and duplicate probes.
    map_points.push_back(Point3f{-0.01f, -0.01f, -0.01f});
    map_points.push_back(Point3f{-0.01f, -0.01f, -0.01f});
    map_points.push_back(Point3f{std::numeric_limits<float>::max(), 0.0f, 0.0f});

    const std::size_t split = map_points.size() / 2U;
    cpu.AddPoints(map_points.data(), split);
    cpu.AddPoints(map_points.data() + split, map_points.size() - split);
    if (!gpu.AddPoints(map_points.data(), split) ||
        !gpu.AddPoints(map_points.data() + split, map_points.size() - split)) {
        std::cerr << "CUDA map insertion failed: " << gpu.LastError() << '\n';
        return 1;
    }

    const auto stats = gpu.Stats();
    if (stats.voxel_count != cpu.NumVoxels()) {
        std::cerr << "voxel count differs: CPU=" << cpu.NumVoxels()
                  << " GPU=" << stats.voxel_count << '\n';
        return 1;
    }

    std::vector<Point3f> queries(4096);
    for (Point3f& point : queries) {
        point = Point3f{query_distribution(random), query_distribution(random), query_distribution(random)};
    }
    queries[0] = Point3f{-0.01f, -0.01f, -0.01f};
    queries[1] = Point3f{std::numeric_limits<float>::max(), 0.0f, 0.0f};

    std::vector<NeighborSet> cpu_results(queries.size());
    std::vector<NeighborSet> gpu_results(queries.size());
    cpu.Query(queries.data(), queries.size(), cpu_results.data());
    if (!gpu.Query(queries.data(), queries.size(), gpu_results.data())) {
        std::cerr << "CUDA query failed: " << gpu.LastError() << '\n';
        return 1;
    }
    for (std::size_t i = 0; i < queries.size(); ++i) {
        std::string reason;
        if (!SameResult(cpu_results[i], gpu_results[i], reason)) {
            std::cerr << "CPU/CUDA mismatch at query " << i << ": " << reason << '\n';
            return 1;
        }
    }

    std::cout << "CPU/CUDA consistency: " << queries.size() << " queries passed\n";
    std::cout << "map voxels=" << stats.voxel_count
              << " accepted_updates=" << stats.accepted_points
              << " rejected=" << stats.rejected_points << '\n';
    std::cout << "batch,CPU_us,CUDA_e2e_us,CUDA_kernel_us\n";

    for (const std::size_t batch : {8U, 16U, 32U, 64U, 128U, 512U, 768U,
                                    1024U, 1536U, 2048U, 4096U}) {
        const int repetitions = batch <= 128U ? 200 : 50;
        std::vector<NeighborSet> scratch(batch);
        for (int warmup = 0; warmup < 5; ++warmup) {
            gpu.Query(queries.data(), batch, scratch.data());
        }
        const double cpu_us = MeanMicroseconds(repetitions, [&] {
            cpu.Query(queries.data(), batch, scratch.data());
        });
        const double gpu_us = MeanMicroseconds(repetitions, [&] {
            if (!gpu.Query(queries.data(), batch, scratch.data())) {
                std::cerr << "CUDA benchmark query failed: " << gpu.LastError() << '\n';
                std::exit(2);
            }
        });
        std::cout << batch << ',' << std::fixed << std::setprecision(3)
                  << cpu_us << ',' << gpu_us << ',' << gpu.Stats().last_query_kernel_ms * 1000.0f << '\n';
    }

    if (!gpu.StartPersistentQueryService(2048)) {
        std::cerr << "persistent CUDA query service failed: " << gpu.LastError() << '\n';
        return 1;
    }
    if (!gpu.Query(queries.data(), 2048, gpu_results.data())) {
        std::cerr << "persistent CUDA query failed: " << gpu.LastError() << '\n';
        return 1;
    }
    for (std::size_t i = 0; i < 2048; ++i) {
        std::string reason;
        if (!SameResult(cpu_results[i], gpu_results[i], reason)) {
            std::cerr << "persistent CPU/CUDA mismatch at query " << i << ": " << reason << '\n';
            return 1;
        }
    }

    std::cout << "persistent_batch,CUDA_e2e_us\n";
    for (const std::size_t batch : {8U, 16U, 32U, 64U, 128U, 512U, 768U,
                                    1024U, 1536U, 2048U}) {
        const int repetitions = batch <= 128U ? 200 : 50;
        std::vector<NeighborSet> scratch(batch);
        const double gpu_us = MeanMicroseconds(repetitions, [&] {
            if (!gpu.Query(queries.data(), batch, scratch.data())) {
                std::cerr << "persistent CUDA benchmark query failed: " << gpu.LastError() << '\n';
                std::exit(2);
            }
        });
        std::cout << batch << ',' << std::fixed << std::setprecision(3) << gpu_us << '\n';
    }

    // Force the ordinary query buffers to grow while the persistent kernel is
    // active, then confirm that the service was resumed after the large batch.
    std::vector<Point3f> oversized_queries(8192);
    for (Point3f& point : oversized_queries) {
        point = Point3f{query_distribution(random), query_distribution(random), query_distribution(random)};
    }
    std::vector<NeighborSet> oversized_cpu(oversized_queries.size());
    std::vector<NeighborSet> oversized_gpu(oversized_queries.size());
    cpu.Query(oversized_queries.data(), oversized_queries.size(), oversized_cpu.data());
    if (!gpu.Query(oversized_queries.data(), oversized_queries.size(), oversized_gpu.data()) ||
        !gpu.PersistentQueryServiceReady()) {
        std::cerr << "oversized CUDA query with persistent service failed: " << gpu.LastError() << '\n';
        return 1;
    }
    for (std::size_t i = 0; i < oversized_queries.size(); ++i) {
        std::string reason;
        if (!SameResult(oversized_cpu[i], oversized_gpu[i], reason)) {
            std::cerr << "oversized CPU/CUDA mismatch at query " << i << ": " << reason << '\n';
            return 1;
        }
    }

    // Exercise the lifecycle used by Batch-LIO: the map is updated and reset
    // while the persistent query service is enabled. This guards against a
    // cudaMalloc/persistent-kernel synchronization deadlock.
    std::vector<Point3f> live_updates(24000);
    for (Point3f& point : live_updates) {
        point = Point3f{map_distribution(random), map_distribution(random), map_distribution(random)};
    }
    cpu.AddPoints(live_updates.data(), live_updates.size());
    if (!gpu.AddPoints(live_updates.data(), live_updates.size()) ||
        !gpu.PersistentQueryServiceReady()) {
        std::cerr << "CUDA insertion with persistent service failed: " << gpu.LastError() << '\n';
        return 1;
    }
    cpu.Query(queries.data(), 2048, cpu_results.data());
    if (!gpu.Query(queries.data(), 2048, gpu_results.data())) {
        std::cerr << "persistent CUDA query after insertion failed: " << gpu.LastError() << '\n';
        return 1;
    }
    for (std::size_t i = 0; i < 2048; ++i) {
        std::string reason;
        if (!SameResult(cpu_results[i], gpu_results[i], reason)) {
            std::cerr << "post-insertion CPU/CUDA mismatch at query " << i << ": " << reason << '\n';
            return 1;
        }
    }

    cpu.Reset();
    if (!gpu.Reset() || !gpu.PersistentQueryServiceReady() || gpu.Stats().voxel_count != 0U) {
        std::cerr << "CUDA reset failed: " << gpu.LastError() << '\n';
        return 1;
    }
    gpu.StopPersistentQueryService();
    return 0;
}
