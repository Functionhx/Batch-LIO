#include "parameters.h"

namespace {
// Declare with dynamic_typing so a YAML int value for a double param (or an int array for a
// double array, e.g. extrinsic_R: [1,0,0,...]) does NOT throw InvalidParameterTypeException;
// then coerce the stored value into the C++ type. The has_parameter guard makes loads
// idempotent (lidar_meas_cov is loaded twice in the upstream loader).
inline void declare_dyn(rclcpp::Node::SharedPtr n, const std::string &name,
                        const rclcpp::ParameterValue &default_val)
{
    if (!n->has_parameter(name)) {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        n->declare_parameter(name, default_val, desc);
    }
}
inline void fetch(rclcpp::Node::SharedPtr n, const std::string &name, double &var)
{
    rclcpp::Parameter p = n->get_parameter(name);
    var = (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) ? (double)p.as_int() : p.as_double();
}
inline void fetch(rclcpp::Node::SharedPtr n, const std::string &name, float &var)
{
    double v; fetch(n, name, v); var = static_cast<float>(v);
}
inline void fetch(rclcpp::Node::SharedPtr n, const std::string &name, bool &var)
{
    var = n->get_parameter(name).as_bool();
}
inline void fetch(rclcpp::Node::SharedPtr n, const std::string &name, int &var)
{
    rclcpp::Parameter p = n->get_parameter(name);
    var = (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) ? (int)p.as_double() : (int)p.as_int();
}
inline void fetch(rclcpp::Node::SharedPtr n, const std::string &name, std::string &var)
{
    var = n->get_parameter(name).as_string();
}
inline void fetch(rclcpp::Node::SharedPtr n, const std::string &name, std::vector<double> &var)
{
    rclcpp::Parameter p = n->get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        auto a = p.as_integer_array();
        var.assign(a.begin(), a.end());
    } else {
        auto a = p.as_double_array();
        var.assign(a.begin(), a.end());
    }
}
template <typename T>
void get_param(rclcpp::Node::SharedPtr n, const std::string &name, T &var, const T &default_val)
{
    declare_dyn(n, name, rclcpp::ParameterValue(default_val));
    fetch(n, name, var);
}
}  // namespace

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
state_input state_in;
state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false, cut_frame = false;
bool use_imu_as_input = false, space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro, cut_frame_time_interval = 0.1;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000; 
float  DET_RANGE = 450;
bool   imu_en = true;
double imu_time_inte = 0.005;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
double time_diff_lidar_to_imu = 0.0;

double lidar_time_inte = 0.1, first_imu_time = 0.0;
int cut_frame_num = 1, orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
bool cut_frame_init = false; // true;
bool   batch_omp = false;     // batch-LIO: OpenMP on per-point KNN+plane-fit loop
double batch_dt  = 0.001;     // batch-LIO: batch window length [s]; <=0 => point-wise (~Point-LIO)
bool   batch_deskew = true;   // batch-LIO: intra-window de-skew on/off (ablation toggle)

bool   profiling_enable = false;          // v2 Phase 0
int    profiling_report_interval = 100;   // v2 Phase 0

std::string map_backend = "original_cpu";
float representative_map_resolution = 0.5f;
int representative_max_points_per_voxel = 4;
int representative_map_capacity = 262144;
int representative_nearby_type = 18;
double representative_max_range = 5.0;
int cuda_min_batch_points = 512;
bool cuda_verify_queries = false;
bool cuda_persistent_queries = false;
int cuda_persistent_max_batch_points = 2048;

MeasureGroup Measures;

ofstream fout_out, fout_imu_pbp;

void readParameters(rclcpp::Node::SharedPtr nh)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  get_param<bool>(nh, "prop_at_freq_of_imu", prop_at_freq_of_imu, true);
  get_param<bool>(nh, "use_imu_as_input", use_imu_as_input, false);
  get_param<bool>(nh, "check_satu", check_satu, true);
  get_param<int>(nh, "init_map_size", init_map_size, 100);
  get_param<bool>(nh, "space_down_sample", space_down_sample, true);
  get_param<bool>(nh, "batch_omp", batch_omp, false);
  get_param<double>(nh, "batch_dt", batch_dt, 0.001);
  get_param<bool>(nh, "batch_deskew", batch_deskew, true);
  get_param<bool>(nh, "profiling_enable", profiling_enable, false);
  get_param<int>(nh, "profiling_report_interval", profiling_report_interval, 100);
  get_param<std::string>(nh, "map_backend", map_backend, "original_cpu");
  get_param<int>(nh, "representative_max_points_per_voxel", representative_max_points_per_voxel, 4);
  get_param<int>(nh, "representative_map_capacity", representative_map_capacity, 262144);
  get_param<int>(nh, "representative_nearby_type", representative_nearby_type, 18);
  get_param<double>(nh, "representative_max_range", representative_max_range, 5.0);
  get_param<int>(nh, "cuda_min_batch_points", cuda_min_batch_points, 512);
  get_param<bool>(nh, "cuda_verify_queries", cuda_verify_queries, false);
  get_param<bool>(nh, "cuda_persistent_queries", cuda_persistent_queries, false);
  get_param<int>(nh, "cuda_persistent_max_batch_points", cuda_persistent_max_batch_points, 2048);
  get_param<double>(nh, "mapping.satu_acc", satu_acc, 3.0);
  get_param<double>(nh, "mapping.satu_gyro", satu_gyro, 35.0);
  get_param<double>(nh, "mapping.acc_norm", acc_norm, 1.0);
  get_param<float>(nh, "mapping.plane_thr", plane_thr, 0.05f);
  get_param<int>(nh, "point_filter_num", p_pre->point_filter_num, 2);
  get_param<std::string>(nh, "common.lid_topic", lid_topic, "/livox/lidar");
  get_param<std::string>(nh, "common.imu_topic", imu_topic, "/livox/imu");
  get_param<bool>(nh, "common.con_frame", con_frame, false);
  get_param<int>(nh, "common.con_frame_num", con_frame_num, 1);
  get_param<bool>(nh, "common.cut_frame", cut_frame, false);
  get_param<double>(nh, "common.cut_frame_time_interval", cut_frame_time_interval, 0.1);
  get_param<double>(nh, "common.time_diff_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
  get_param<double>(nh, "filter_size_surf", filter_size_surf_min, 0.5);
  get_param<double>(nh, "filter_size_map", filter_size_map_min, 0.5);
  get_param<float>(nh, "mapping.det_range", DET_RANGE, 300.f);
  get_param<double>(nh, "mapping.fov_degree", fov_deg, 180);
  get_param<bool>(nh, "mapping.imu_en", imu_en, true);
  get_param<bool>(nh, "mapping.extrinsic_est_en", extrinsic_est_en, true);
  get_param<double>(nh, "mapping.imu_time_inte", imu_time_inte, 0.005);
  get_param<double>(nh, "mapping.lidar_meas_cov", laser_point_cov, 0.1);
  get_param<double>(nh, "mapping.acc_cov_input", acc_cov_input, 0.1);
  get_param<double>(nh, "mapping.vel_cov", vel_cov, 20);
  get_param<double>(nh, "mapping.gyr_cov_input", gyr_cov_input, 0.1);
  get_param<double>(nh, "mapping.gyr_cov_output", gyr_cov_output, 0.1);
  get_param<double>(nh, "mapping.acc_cov_output", acc_cov_output, 0.1);
  get_param<double>(nh, "mapping.b_gyr_cov", b_gyr_cov, 0.0001);
  get_param<double>(nh, "mapping.b_acc_cov", b_acc_cov, 0.0001);
  get_param<double>(nh, "mapping.imu_meas_acc_cov", imu_meas_acc_cov, 0.1);
  get_param<double>(nh, "mapping.imu_meas_omg_cov", imu_meas_omg_cov, 0.1);
  get_param<double>(nh, "preprocess.blind", p_pre->blind, 1.0);
  get_param<int>(nh, "preprocess.lidar_type", lidar_type, 1);
  get_param<int>(nh, "preprocess.scan_line", p_pre->N_SCANS, 16);
  get_param<int>(nh, "preprocess.scan_rate", p_pre->SCAN_RATE, 10);
  get_param<int>(nh, "preprocess.timestamp_unit", p_pre->time_unit, 1);
  get_param<double>(nh, "mapping.match_s", match_s, 81);
  get_param<std::vector<double>>(nh, "mapping.gravity", gravity, std::vector<double>());
  get_param<std::vector<double>>(nh, "mapping.gravity_init", gravity_init, std::vector<double>());
  get_param<std::vector<double>>(nh, "mapping.extrinsic_T", extrinT, std::vector<double>());
  get_param<std::vector<double>>(nh, "mapping.extrinsic_R", extrinR, std::vector<double>());
  get_param<bool>(nh, "odometry.publish_odometry_without_downsample", publish_odometry_without_downsample, false);
  get_param<bool>(nh, "publish.path_en", path_en, true);
  get_param<bool>(nh, "publish.scan_publish_en", scan_pub_en, true);
  get_param<bool>(nh, "publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
  get_param<bool>(nh, "runtime_pos_log_enable", runtime_pos_log, false);
  get_param<bool>(nh, "pcd_save.pcd_save_en", pcd_save_en, false);
  get_param<int>(nh, "pcd_save.interval", pcd_save_interval, -1);

  get_param<double>(nh, "mapping.lidar_time_inte", lidar_time_inte, 0.1);

  get_param<float>(nh, "mapping.ivox_grid_resolution", ivox_options_.resolution_, 0.2);
  get_param<float>(nh, "mapping.representative_map_resolution", representative_map_resolution, 0.5f);
  get_param<int>(nh, "ivox_nearby_type", ivox_nearby_type, 18);
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    // LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot) 
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file()
{

    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

}

void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    P_init = MD(24, 24)::Identity() * 0.1;
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}
