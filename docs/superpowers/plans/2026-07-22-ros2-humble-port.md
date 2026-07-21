# batch-LIO ROS2 Humble Port — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the ROS1 Noetic (catkin) `batch_lio` package to a native ROS2 Humble (ament_cmake) package that builds with `colcon`, runs on converted Avia bags, publishes odometry/TF/clouds/path, passes a deskew gtest + a smoke launch_test, and reproduces the documented A/B speedup + deskew numbers.

**Architecture:** Thin faithful port — keep `laserMapping.cpp`'s `main()` poll-loop structure, replacing each ROS1 API call 1:1 with its ROS2 equivalent. The full `livox_ros_driver2` package (vendored SDK2) is a colcon sibling that exports `livox_ros_driver2/msg/CustomMsg`. Single-threaded executor + `spin_some()` inside the 500 Hz loop preserves Point-LIO's serialization semantics.

**Tech Stack:** ROS2 Humble, ament_cmake, rclcpp, PCL 1.12, Eigen 3.4, OpenMP, tf2_ros, pcl_conversions, livox_ros_driver2, gtest (`ament_add_gtest`), launch_testing.

## Global Constraints

- Environment: **native `/opt/ros/humble`** (`source /opt/ros/humble/setup.bash` before every build/run). 32 cores, PCL 1.12, Eigen 3.4.
- Branch: **`ros2-humble`** (already created). All work commits here; `main` stays the pristine ROS1 baseline.
- Commit author: `Functionhx <functionhx@gmail.com>`. **Never add `Co-Authored-By`.**
- **No algorithm changes** — EKF/iVox/deskew math untouched. Only ROS API + build system change.
- Custom message `LocalSensorExternalTrigger.msg` and optional `matplotlib`/Python viz are **dropped** (YAGNI).
- ROS1→ROS2 mechanical replacements applied repo-wide (see "Mechanical Patterns" below).

## Mechanical Patterns (apply throughout, do not re-list per task)

These are global, deterministic replacements. Apply them everywhere they appear:

| Find | Replace |
|---|---|
| `sensor_msgs::Imu` | `sensor_msgs::msg::Imu` |
| `sensor_msgs::PointCloud2` | `sensor_msgs::msg::PointCloud2` |
| `nav_msgs::Odometry` | `nav_msgs::msg::Odometry` |
| `nav_msgs::Path` | `nav_msgs::msg::Path` |
| `geometry_msgs::PoseStamped` | `geometry_msgs::msg::PoseStamped` |
| `sensor_msgs::Imu::Ptr` | `sensor_msgs::msg::Imu::SharedPtr` |
| `sensor_msgs::Imu::ConstPtr` | `sensor_msgs::msg::Imu::ConstSharedPtr` |
| `sensor_msgs::PointCloud2::ConstPtr` | `sensor_msgs::msg::PointCloud2::ConstSharedPtr` |
| `livox_ros_driver::CustomMsg::ConstPtr` | `livox_ros_driver2::msg::CustomMsg::ConstSharedPtr` |
| `livox_ros_driver::CustomMsg` | `livox_ros_driver2::msg::CustomMsg` |
| `X.header.stamp.toSec()` | `stamp_to_sec(X.header.stamp)` |
| `ros::Time().fromSec(Y)` | `stamp_from_sec(Y)` |
| `ros::Time::now()` | `nh->now()` |
| `ROS_WARN(...)` | `RCLCPP_WARN(rclcpp::get_logger("batch_lio"), ...)` |
| `ROS_INFO(...)` | `RCLCPP_INFO(rclcpp::get_logger("batch_lio"), ...)` |
| `ROS_ERROR(...)` | `RCLCPP_ERROR(rclcpp::get_logger("batch_lio"), ...)` |

The `stamp_to_sec` / `stamp_from_sec` helpers are defined in Task 3 (common_lib.h). Field access on messages (`msg->points[i].x`, `imu_next.angular_velocity.x`, `msg->header.frame_id`, etc.) is **unchanged** — only the C++ type names and time accessors change.

---

## File Structure

**Created:**
- `ros2_ws/src/livox_ros_driver2` → symlink to `/home/as/vllm/Nav/pb2025_sentry_nav/livox_ros_driver2`
- `launch/mapping_avia.launch.py` (+ `mapping_horizon.launch.py`, `mapping_ouster64.launch.py`, `mapping_velody16.launch.py`, `avia_batch.launch.py`)
- `test/test_smoke.py` (launch_test)

**Rewritten:**
- `CMakeLists.txt` (catkin → ament)
- `package.xml` (catkin → ament format 3)

**Modified (ROS1→ROS2 API):**
- `include/common_lib.h` — includes, `MeasureGroup::imu` type, add time helpers
- `src/parameters.h`, `src/parameters.cpp` — `readParameters(rclcpp::Node::SharedPtr)` + param helper
- `src/preprocess.h`, `src/preprocess.cpp` — livox type, drop dead `pub_*`/`pub_func`
- `src/li_initialization.h`, `src/li_initialization.cpp` — IMU types, callback signatures
- `src/IMU_Processing.h`, `src/IMU_Processing.cpp` — includes, logging
- `src/laserMapping.cpp` — node, publishers, tf2, main loop (the core port)
- `test/test_deskew.cpp` — wrap as `ament_add_gtest`
- `scripts/run_lio.sh`, `scripts/ablations.sh` — ROS2 commands

**Deleted:**
- `scripts/avia_batch.launch` (replaced by `avia_batch.launch.py`)
- `msg/LocalSensorExternalTrigger.msg` + the `msg/` dir (unused)

---

## Task 1: Colcon workspace + build livox_ros_driver2

**Files:**
- Create: `~/batch_lio_ws/src/` (workspace root outside the repo)
- Symlink: `~/batch_lio_ws/src/livox_ros_driver2` → `/home/as/vllm/Nav/pb2025_sentry_nav/livox_ros_driver2`
- Symlink: `~/batch_lio_ws/src/batch_lio` → `/home/as/vllm/Batch-LIO`

**Interfaces:** Produces a built `livox_ros_driver2` exposing `livox_ros_driver2/msg/CustomMsg`; workspace `install/setup.bash` to source.

- [ ] **Step 1: Create the workspace and symlink the two packages**

```bash
mkdir -p ~/batch_lio_ws/src
ln -sfn /home/as/vllm/Nav/pb2025_sentry_nav/livox_ros_driver2 ~/batch_lio_ws/src/livox_ros_driver2
ln -sfn /home/as/vllm/Batch-LIO ~/batch_lio_ws/src/batch_lio
ls -l ~/batch_lio_ws/src   # both symlinks present
```

- [ ] **Step 2: Build livox_ros_driver2 alone first (it must succeed before batch_lio can depend on it)**

```bash
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws
colcon build --packages-select livox_ros_driver2 --symlink-install
```
Expected: build **finishes**. The driver node may emit a warning about the SDK path but must link against the vendored `Livox-SDK2/lib/amd64/liblivox_lidar_sdk_shared.so`. If it fails on the SDK, confirm `~/batch_lio_ws/src/livox_ros_driver2/Livox-SDK2/lib/amd64/liblivox_lidar_sdk_shared.so` exists (it does).

- [ ] **Step 3: Verify the message type was generated**

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 interface show livox_ros_driver2/msg/CustomMsg
```
Expected: prints the CustomMsg field list (`header`, `timebase`, `point_num`, `lidar_id`, `rsvd`, `points` of `CustomPoint[]`). If this works, the dependency is available.

- [ ] **Step 4: Commit the workspace pointer**

The symlinks live outside the repo, so there is nothing to commit in the repo yet. Record the workspace path in `README.md` under a new "Build (ROS2 Humble)" section header (filled in fully in Task 13). For now, leave a one-line placeholder is NOT allowed — instead defer README edits to Task 13 and just move on.

---

## Task 2: Rewrite build system (CMakeLists.txt + package.xml)

**Files:**
- Modify: `CMakeLists.txt` (full rewrite)
- Modify: `package.xml` (full rewrite)

**Interfaces:** Produces two ament targets — `batchlio_mapping` (the node) and the `test_deskew` gtest — with `livox_ros_driver2` as a build/run dependency.

- [ ] **Step 1: Rewrite `CMakeLists.txt`**

Replace the entire file with:

```cmake
cmake_minimum_required(VERSION 3.8)
project(batch_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ---- OpenMP (preserved from ROS1: auto thread count by core count) ----
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number: ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()
find_package(OpenMP REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  src
)

link_directories(${PCL_LIBRARY_DIRS})

# ---- the odometry node ----
add_executable(batchlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp)
target_link_libraries(batchlio_mapping
  ${PCL_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_target_dependencies(batchlio_mapping
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros pcl_conversions livox_ros_driver2)
target_include_directories(batchlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})  # harmless if empty

install(TARGETS batchlio_mapping
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})

ament_package()

# ---- deskew unit test (standalone Eigen; ROS2 gtest target added in Task 10) ----
# (gtest wiring is added in Task 10 once the test file is gtest-converted.)
```

- [ ] **Step 2: Rewrite `package.xml`**

Replace the entire file with:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>batch_lio</name>
  <version>0.0.0</version>
  <description>batch-LIO: batch-wise Point-LIO with intra-window deskew (ROS2 Humble port).</description>
  <maintainer email="dev@livoxtech.com">batch-lio</maintainer>
  <license>BSD</license>
  <author email="hdj65822@connect.hku.hk">Dongjiao He (Point-LIO)</author>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>pcl_conversions</depend>
  <depend>livox_ros_driver2</depend>
  <depend>libpcl-all-dev</depend>
  <depend>libeigen3-dev</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>launch_ros</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>ros2bag</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Do NOT build yet** — source files still reference ROS1 APIs. Building is deferred to after Task 9. Commit the build files now so the diff is reviewable.

```bash
git add CMakeLists.txt package.xml
git commit -m "build: ament_cmake CMakeLists + package.xml for ROS2 Humble"
```

---

## Task 3: Port `include/common_lib.h`

**Files:**
- Modify: `include/common_lib.h` (lines 8-11 includes; line 124 `MeasureGroup::imu` type; add time helpers)

**Interfaces:** Produces `stamp_to_sec()` and `stamp_from_sec()` used by every other file; produces `MeasureGroup::imu` as `deque<sensor_msgs::msg::Imu::ConstSharedPtr>`.

- [ ] **Step 1: Replace the ROS includes block (lines 8-11)**

Replace:
```cpp
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
```
with:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>
```
(`nav_msgs` and `tf` were included here but never used inside common_lib.h; they are included where actually used in laserMapping.cpp.)

- [ ] **Step 2: Change `MeasureGroup::imu` type (line 124)**

Replace:
```cpp
    deque<sensor_msgs::Imu::ConstPtr> imu;
```
with:
```cpp
    deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
```

- [ ] **Step 3: Add the time helpers immediately after the includes block (after line 12 `using namespace Eigen;`)**

Insert:
```cpp
// ---- ROS2 time helpers (replace ros::Time().fromSec / .toSec) ----
inline double stamp_to_sec(const builtin_interfaces::msg::Time &stamp)
{
    return rclcpp::Time(stamp).seconds();
}
inline builtin_interfaces::msg::Time stamp_from_sec(double sec)
{
    return rclcpp::Time(static_cast<int64_t>(sec * 1e9));
}
```

- [ ] **Step 4: Commit**

```bash
git add include/common_lib.h
git commit -m "port: common_lib.h ROS2 includes + MeasureGroup type + time helpers"
```

---

## Task 4: Port `src/parameters.h` + `src/parameters.cpp`

**Files:**
- Modify: `src/parameters.h` (includes block lines 4-25; line 82 signature)
- Modify: `src/parameters.cpp` (lines 50-128 `readParameters` body)

**Interfaces:** Produces `void readParameters(rclcpp::Node::SharedPtr n)` reading all params via a `get_param<T>` helper. Namespaced names use `.` separator (ROS1 `"mapping/satu_acc"` → `"mapping.satu_acc"`).

- [ ] **Step 1: Fix includes in `src/parameters.h`**

Replace lines 4-25 (the include block) with:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"
#include "IMU_Processing.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex>
#include <omp.h>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ivox/ivox3d.h>
#include <condition_variable>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl/common/transforms.h>
#include <geometry_msgs/msg/vector3.hpp>
```
(Removed `<ros/ros.h>` and `<Python.h>` — matplotlib dropped.)

- [ ] **Step 2: Change the `readParameters` declaration (line 82)**

Replace:
```cpp
void readParameters(ros::NodeHandle &n);
```
with:
```cpp
void readParameters(rclcpp::Node::SharedPtr n);
```

- [ ] **Step 3: Rewrite `readParameters` in `src/parameters.cpp`**

Add a helper at the top of `src/parameters.cpp` (after the includes, before the globals), then convert each `n.param<T>(...)` call. Replace the function signature and body. At the top of the file (after `#include "parameters.h"`) insert:

```cpp
namespace {
// declare (with default) then get; ROS2 param names use '.' as the hierarchy separator.
template <typename T>
void get_param(rclcpp::Node::SharedPtr n, const std::string &name, T &var, const T &default_val)
{
    n->declare_parameter<T>(name, default_val);
    var = n->get_parameter(name).get_value<T>();
}
}  // namespace
```

Then change the signature `void readParameters(ros::NodeHandle &nh)` → `void readParameters(rclcpp::Node::SharedPtr nh)`, and convert each call. The mechanical rule: `nh.param<T>("a/b", var, def)` → `get_param<T>(nh, "a.b", var, def)` (slash→dot); top-level names keep their key. Concretely replace these lines (the first one shown verbatim, rest follow the same rule):

```cpp
  get_param<bool>(nh, "prop_at_freq_of_imu", prop_at_freq_of_imu, true);
  get_param<bool>(nh, "use_imu_as_input", use_imu_as_input, false);
  get_param<bool>(nh, "check_satu", check_satu, true);
  get_param<int>(nh, "init_map_size", init_map_size, 100);
  get_param<bool>(nh, "space_down_sample", space_down_sample, true);
  get_param<bool>(nh, "batch_omp", batch_omp, false);
  get_param<double>(nh, "batch_dt", batch_dt, 0.001);
  get_param<bool>(nh, "batch_deskew", batch_deskew, true);
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
  get_param<int>(nh, "ivox_nearby_type", ivox_nearby_type, 18);
```
Leave the `if (ivox_nearby_type == 0) {...}` block and the trailing `p_imu->gravity_ << VEC_FROM_ARRAY(gravity);` unchanged.

- [ ] **Step 4: Commit**

```bash
git add src/parameters.h src/parameters.cpp
git commit -m "port: readParameters(rclcpp::Node) with declare+get helper"
```

---

## Task 5: Port `src/preprocess.h` + `src/preprocess.cpp`

**Files:**
- Modify: `src/preprocess.h` (lines 1-4 includes; remove dead members line 126; remove `pub_func` decl line 135)
- Modify: `src/preprocess.cpp` (all `livox_ros_driver::CustomMsg` type refs; remove dead `pub_func` body line ~875)

**Interfaces:** Produces `Preprocess::process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr&, ...)` etc. Field access (`msg->points[i].x/.reflectivity/.offset_time/.line/.tag`) unchanged.

- [ ] **Step 1: Fix includes in `src/preprocess.h`**

Replace lines 1-4:
```cpp
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
```
with:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
```

- [ ] **Step 2: Remove the dead publisher members (line 126)**

Delete the line:
```cpp
  ros::Publisher pub_full, pub_surf, pub_corn;
```
(Confirmed dead — only referenced in commented-out calls.)

- [ ] **Step 3: Remove the dead `pub_func` declaration (line 135)**

Delete the line:
```cpp
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
```

- [ ] **Step 4: In `src/preprocess.cpp`, replace the type references**

Apply the mechanical pattern repo-wide to this file:
- `livox_ros_driver::CustomMsg::ConstPtr` → `livox_ros_driver2::msg::CustomMsg::ConstSharedPtr` (in the 4 function signatures: `process(CustomMsg)`, `process_cut_frame_livox`, `avia_handler`, and the `CustomMsg` overloads)
- `X.header.stamp.toSec()` → `stamp_to_sec(X.header.stamp)` (lines 131, 144, 268, 286, 300, 316 region, 374) — `stamp_to_sec` comes from common_lib.h which preprocess.cpp gets via parameters.h→preprocess.h→...→common_lib.h. Confirm `preprocess.cpp` transitively includes common_lib.h (it does: preprocess.h includes pcl only; but preprocess.cpp includes "preprocess.h" — does it get common_lib? No!). So **add `#include "common_lib.h"` at the top of `src/preprocess.cpp`** (after `#include "preprocess.h"`) so `stamp_to_sec` is visible. Field access on `msg->points[i]` and `msg->header` is otherwise unchanged.

- [ ] **Step 5: Remove the dead `pub_func` body in `src/preprocess.cpp` (~line 875)**

Delete the entire `void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct) { ... }` definition (and any commented `pub_func(...)` calls at lines 399-400).

- [ ] **Step 6: Commit**

```bash
git add src/preprocess.h src/preprocess.cpp
git commit -m "port: preprocess to livox_ros_driver2::msg; drop dead pub_func/members"
```

---

## Task 6: Port `src/li_initialization.h` + `src/li_initialization.cpp`

**Files:**
- Modify: `src/li_initialization.h` (lines 20, 25, 31-33 types/signatures)
- Modify: `src/li_initialization.cpp` (lines 11, 23, 25, 104, 109, 119, 183-210 callbacks)

**Interfaces:** Produces `livox_pcl_cbk`, `standard_pcl_cbk`, `imu_cbk` with ROS2 `::ConstSharedPtr` signatures bound by `create_subscription` in Task 9; `imu_deque`/`imu_last`/`imu_next` use `sensor_msgs::msg::Imu`.

- [ ] **Step 1: Fix types/declarations in `src/li_initialization.h`**

Replace line 20:
```cpp
extern std::deque<sensor_msgs::Imu::Ptr> imu_deque;
```
with:
```cpp
extern std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
```

Replace line 25:
```cpp
extern sensor_msgs::Imu imu_last, imu_next;
```
with:
```cpp
extern sensor_msgs::msg::Imu imu_last, imu_next;
```

Replace lines 31-33 (callback declarations):
```cpp
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg);
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);
```

- [ ] **Step 2: Fix globals in `src/li_initialization.cpp`**

Replace line 11:
```cpp
sensor_msgs::msg::Imu imu_last, imu_next;
```
Replace line 23:
```cpp
std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
```

- [ ] **Step 3: Fix `standard_pcl_cbk` signature (line 25)**

```cpp
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
```
and within it replace `ROS_ERROR(...)` → `RCLCPP_ERROR(rclcpp::get_logger("batch_lio"), ...)` (lines 32), and `msg->header.stamp.toSec()` → `stamp_to_sec(msg->header.stamp)` (lines 30, 40, 95). `common_lib.h` is included via `li_initialization.h` so `stamp_to_sec` is visible.

- [ ] **Step 4: Fix `livox_pcl_cbk` (line 104)**

```cpp
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg)
```
Replace `ROS_ERROR` → `RCLCPP_ERROR(...)` (line 111); replace `msg->header.stamp.toSec()` → `stamp_to_sec(msg->header.stamp)` (lines 109, 119, 174).

- [ ] **Step 5: Fix `imu_cbk` (lines 183-210)**

```cpp
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    msg->header.stamp = stamp_from_sec(stamp_to_sec(msg->header.stamp) - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);
    double timestamp = stamp_to_sec(msg->header.stamp);
    ...
```
(Replace the `ros::Time().fromSec(msg->header.stamp.toSec() - ...)` line exactly as shown; replace `ROS_ERROR` → `RCLCPP_ERROR(...)` at line 197.) Leave the rest of `imu_cbk` and all of `sync_packages` unchanged (it only dereferences `imu_deque.front()->header.stamp` via the helper — update those `.toSec()` calls at lines 310, 319, 331, 341 to `stamp_to_sec(...)`).

- [ ] **Step 6: Commit**

```bash
git add src/li_initialization.h src/li_initialization.cpp
git commit -m "port: li_initialization callbacks + IMU types to ROS2"
```

---

## Task 7: Port `src/IMU_Processing.h` + `src/IMU_Processing.cpp`

**Files:**
- Modify: `src/IMU_Processing.h` (includes lines 8, 16, 18-19)
- Modify: `src/IMU_Processing.cpp` (`ROS_WARN`/`ROS_INFO` lines 30, 74, 119)

**Interfaces:** Unchanged algorithm; just ROS2 logging + includes.

- [ ] **Step 1: Fix includes in `src/IMU_Processing.h`**

Replace line 8 `#include <ros/ros.h>` with `#include <rclcpp/rclcpp.hpp>`. Replace line 16 `#include <nav_msgs/Odometry.h>` with `#include <nav_msgs/msg/odometry.hpp>`. Replace lines 18-19:
```cpp
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
```
with:
```cpp
#include <tf2_ros/transform_broadcaster.h>
```
(`eigen_conversions`/`tf2_eigen` is not used in IMU_Processing; `tf2_ros` keeps parity for the broadcaster type though it's unused here — harmless. If lint complains, drop it.)

- [ ] **Step 2: Fix logging in `src/IMU_Processing.cpp`**

Replace line 30 `ROS_WARN("Reset ImuProcess");` → `RCLCPP_WARN(rclcpp::get_logger("batch_lio"), "Reset ImuProcess");`. Replace lines 74 and 119 `ROS_INFO("IMU Initializing: %.1f %%", ...)` → `RCLCPP_INFO(rclcpp::get_logger("batch_lio"), "IMU Initializing: %.1f %%", ...);`.

- [ ] **Step 3: Commit**

```bash
git add src/IMU_Processing.h src/IMU_Processing.cpp
git commit -m "port: IMU_Processing includes + ROS2 logging"
```

---

## Task 8: Port `src/laserMapping.cpp` (core)

**Files:**
- Modify: `src/laserMapping.cpp` (includes, globals, publish functions, tf2, main loop)

**Interfaces:** Produces the `batchlio_mapping` node. Consumes `readParameters`, the callbacks, and `stamp_to_sec/stamp_from_sec`.

This is the largest task; do it in numbered sub-steps and commit once at the end.

- [ ] **Step 1: Fix the include block (lines 1-18)**

Replace lines 1-18 with:
```cpp
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "li_initialization.h"
#include <malloc.h>
#include <omp.h>
#include "deskew.h"
```
(Dropped `tf/transform_datatypes.h`, `tf/transform_broadcaster.h`, commented `ros/console.h` and `matplotlibcpp.h`.)

- [ ] **Step 2: Fix global message-typed objects (lines 47-49)**

```cpp
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;
```

Add a global tf2 broadcaster pointer near the other globals (after line 35):
```cpp
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
```

- [ ] **Step 3: Fix `SigHandle` (line 54)**

`ROS_WARN("catch sig %d", sig);` → `RCLCPP_WARN(rclcpp::get_logger("batch_lio"), "catch sig %d", sig);`

- [ ] **Step 4: Fix the publish functions (lines 154-316)**

For each of `publish_init_map`, `publish_frame_world`, `publish_frame_body`, `publish_odometry`, `publish_path`:
- Signature: `const ros::Publisher & pub` → `const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub` (and the odom/path variants: `rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr`, `rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr`).
- `sensor_msgs::PointCloud2 laserCloudmsg;` → `sensor_msgs::msg::PointCloud2 laserCloudmsg;`
- `laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);` → `laserCloudmsg.header.stamp = stamp_from_sec(lidar_end_time);`
- `pubLaserCloudFullRes.publish(laserCloudmsg);` → `pubLaserCloudFullRes->publish(laserCloudmsg);` (the parameter name, e.g. `pubLaserCloudFullRes`).

- [ ] **Step 5: Rewrite the tf2 block in `publish_odometry` (lines 289-300)**

Replace the `static tf::TransformBroadcaster br; ... br.sendTransform(tf::StampedTransform(...));` block with:
```cpp
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.header.frame_id = "camera_init";
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_broadcaster->sendTransform(trans);
```

- [ ] **Step 6: Fix the two remaining stamps in `publish_odometry` (lines 279, 283)**

`odomAftMapped.header.stamp = ros::Time().fromSec(time_current);` → `stamp_from_sec(time_current)`; likewise `lidar_end_time`.

- [ ] **Step 7: Rewrite `main()` setup (lines 318-336)**

Replace lines 318-324:
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    readParameters(nh);
```
with:
```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    readParameters(nh);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
```

Fix the path header stamp (line 334): `path.header.stamp = ros::Time().fromSec(lidar_end_time);` → `stamp_from_sec(lidar_end_time);`.

- [ ] **Step 8: Rewrite subscribers + publishers (lines 381-400)**

Replace lines 381-400 with:
```cpp
    /*** ROS2 subscribe / publish initialization ***/
    auto qos = rclcpp::SensorDataQoS();
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_std;
    if (p_pre->lidar_type == AVIA)
        sub_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, qos, livox_pcl_cbk);
    else
        sub_std = nh->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, qos, standard_pcl_cbk);
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, imu_cbk);

    auto pubLaserCloudFullRes      = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
    auto pubLaserCloudFullRes_body = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 1000);
    auto pubLaserCloudMap          = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 1000);
    auto pubOdomAftMapped          = nh->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 1000);
    auto pubPath                   = nh->create_publisher<nav_msgs::msg::Path>("/path", 1000);
```
(Keep the `(void)` cast / unused-var guards if the compiler warns about `sub_livox`/`sub_std` — both are retained for the executor's lifetime via the node; to be safe, after creating them, add `(void)sub_livox; (void)sub_std;`.)

- [ ] **Step 9: Rewrite the main loop control (lines 402-408, 1089-1091)**

Replace line 403 `ros::Rate loop_rate(500);` → `rclcpp::Rate loop_rate(500);`. Replace line 404 `bool status = ros::ok();` → `bool status = rclcpp::ok();`. Replace line 408 `ros::spinOnce();` → `executor.spin_some();`. Replace line 413 and 731 `ROS_WARN(...)` → `RCLCPP_WARN(rclcpp::get_logger("batch_lio"), ...);`. Replace line 1089 `status = ros::ok();` → `status = rclcpp::ok();`.

After the loop (after line 1091 `loop_rate.sleep();` and before the PCD save block at 1092), add:
```cpp
    rclcpp::shutdown();
```

- [ ] **Step 10: Fix every `imu_next.header.stamp.toSec()` / `imu_last.header.stamp.toSec()` in the main loop**

Apply `X.header.stamp.toSec()` → `stamp_to_sec(X.header.stamp)` to all occurrences in `main()` (lines 444, 457, 615, 632, 633, 649, 657, 659, 662, 666, 681, 776, 782, 803, 854, 873, 880, 882, 886, 889, 966, 972, 994, 1004, 1008). These are pure mechanical replacements; the surrounding logic is unchanged.

- [ ] **Step 11: Commit**

```bash
git add src/laserMapping.cpp
git commit -m "port: laserMapping node to rclcpp + tf2 + ROS2 pub/sub"
```

---

## Task 9: First clean build

**Files:** none (verification only)

**Interfaces:** Confirms the whole package compiles under ament/colcon.

- [ ] **Step 1: Build the package**

```bash
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws
colcon build --packages-select batch_lio --symlink-install 2>&1 | tee /tmp/build1.log
```
Expected: `Finished <<< batch_lio`. If there are errors, fix them in place (the most likely are: a missed `.toSec()`/type rename, a missing `#include`, or a param name typo). Do not move on until it builds clean.

- [ ] **Step 2: Confirm the node executable exists**

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 pkg executables batch_lio
```
Expected: `batch_lio batchlio_mapping`.

- [ ] **Step 3: No commit** (build artifacts are out-of-tree). Re-commit only if source fixes were needed.

---

## Task 10: Deskew gtest

**Files:**
- Modify: `test/test_deskew.cpp` (wrap as gtest)
- Modify: `CMakeLists.txt` (add gtest target)

**Interfaces:** `colcon test --packages-select batch_lio` runs the deskew test green.

- [ ] **Step 1: Convert `test/test_deskew.cpp` to gtest**

Replace the file with:
```cpp
// Standalone unit test for batch-LIO intra-window de-skew (paper eq 3.44-3.47).
#include <gtest/gtest.h>
#include "../src/deskew.h"
#include <Eigen/Dense>
#include <cmath>
using namespace Eigen;

static bool close(const Vector3d& a, const Vector3d& b, double e = 1e-9) {
  return (a - b).norm() < e;
}

TEST(Deskew, IdentityAtZeroDt) {
  const Matrix3d I = Matrix3d::Identity();
  EXPECT_TRUE(close(deskew_point(Vector3d(1, 2, 3), 0.0,
                                 Vector3d(0.1, 0, 0), Vector3d(1, 0, 0), I),
                    Vector3d(1, 2, 3)));
}

TEST(Deskew, ZeroMotionIsIdentity) {
  const Matrix3d I = Matrix3d::Identity();
  EXPECT_TRUE(close(deskew_point(Vector3d(1, 2, 3), -0.001,
                                 Vector3d(0, 0, 0), Vector3d(0, 0, 0), I),
                    Vector3d(1, 2, 3)));
}

TEST(Deskew, PureTranslation) {
  const Matrix3d I = Matrix3d::Identity();
  EXPECT_TRUE(close(deskew_point(Vector3d(0, 0, 0), -0.01,
                                 Vector3d(0, 0, 0), Vector3d(2, 0, 0), I),
                    Vector3d(-0.02, 0, 0)));
}

TEST(Deskew, PureRotationAboutZ) {
  const Matrix3d I = Matrix3d::Identity();
  const double wz = 1.0, dt = -0.5, th = wz * dt;
  Vector3d got = deskew_point(Vector3d(1, 0, 0), dt,
                              Vector3d(0, 0, wz), Vector3d(0, 0, 0), I);
  Vector3d exp(std::cos(th), std::sin(th), 0);
  EXPECT_TRUE(close(got, exp, 1e-9));
}

TEST(Deskew, TranslationRotatesWorldVelIntoBody) {
  Matrix3d Rz; Rz << 0, -1, 0,  1, 0, 0,  0, 0, 1;   // +90deg about z
  double dt = -0.1;
  Vector3d got = deskew_point(Vector3d(0, 0, 0), dt,
                              Vector3d(0, 0, 0), Vector3d(1, 0, 0), Rz);
  Vector3d expv = Rz.transpose() * Vector3d(1, 0, 0) * dt;
  EXPECT_TRUE(close(got, expv));
}
```

- [ ] **Step 2: Add the gtest target to `CMakeLists.txt`**

Append before `ament_package()`:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_deskew test/test_deskew.cpp)
  target_include_directories(test_deskew PRIVATE ${EIGEN3_INCLUDE_DIR} src)
  target_link_libraries(test_deskew Eigen3::Eigen)
endif()
```

- [ ] **Step 3: Build + run the test**

```bash
cd ~/batch_lio_ws
colcon build --packages-select batch_lio --symlink-install
colcon test --packages-select batch_lio --event-handlers console_direct+
colcon test-result --all
```
Expected: `test_deskew` PASS (5 tests).

- [ ] **Step 4: Commit**

```bash
git add test/test_deskew.cpp CMakeLists.txt
git commit -m "test: deskew unit test as ament_add_gtest"
```

---

## Task 11: Launch files + config wrap

**Files:**
- Create: `launch/mapping_avia.launch.py` (+ horizon/ouster64/velody16 + `avia_batch.launch.py`)
- Modify: `config/avia.yaml` (+ horizon/ouster64/velody16)
- Delete: `scripts/avia_batch.launch`, `msg/LocalSensorExternalTrigger.msg`

**Interfaces:** `ros2 launch batch_lio mapping_avia.launch.py` launches the node with `avia.yaml` params + optional rviz2.

- [ ] **Step 1: Wrap `config/avia.yaml` in the ROS2 param-file envelope**

Prepend these two lines to the file and indent every existing line by 2 spaces:
```yaml
/**:
  ros__parameters:
    common:
        lid_topic: "/livox/lidar"
        imu_topic: "/livox/imu"
        # ... (all existing content, each line indented +2)
```
The existing nesting (`common:`, `preprocess:`, `mapping:`, `odometry:`, `publish:`, `pcd_save:`) stays nested under `ros__parameters`. Repeat identically for `config/horizon.yaml`, `config/ouster64.yaml`, `config/velody16.yaml`.

- [ ] **Step 2: Create `launch/mapping_avia.launch.py`**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('batch_lio'), 'config', 'avia.yaml')
    default_rviz = os.path.join(
        get_package_share_directory('batch_lio'), 'rviz_cfg', 'loam_livox.rviz')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    cfg_arg = DeclareLaunchArgument('config', default_value=default_cfg)

    mapping = Node(
        package='batch_lio',
        executable='batchlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, cfg_arg, mapping, rviz])
```
Create `mapping_horizon.launch.py`, `mapping_ouster64.launch.py`, `mapping_velody16.launch.py` identically except the `default_cfg` filename (`horizon.yaml`, `ouster64.yaml`, `velody16.yaml`).

- [ ] **Step 3: Create `launch/avia_batch.launch.py` (headless A/B harness entry, parity of `scripts/avia_batch.launch`)**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('batch_lio'), 'config', 'avia.yaml')

    def arg(name, default):
        return DeclareLaunchArgument(name, default_value=str(default))

    return LaunchDescription([
        arg('batch_dt', 0.001),
        arg('batch_omp', 'false'),
        arg('batch_deskew', 'true'),
        arg('pub_hifreq', 'false'),
        arg('use_imu_as_input', 'false'),
        arg('runtime_pos_log_enable', 'true'),
        Node(
            package='batch_lio',
            executable='batchlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                cfg,
                {
                    'use_imu_as_input': LaunchConfiguration('use_imu_as_input'),
                    'prop_at_freq_of_imu': 'true',
                    'check_satu': 'true',
                    'init_map_size': 10,
                    'point_filter_num': 1,
                    'space_down_sample': 'true',
                    'filter_size_surf': 0.5,
                    'filter_size_map': 0.4,
                    'ivox_nearby_type': 6,
                    'runtime_pos_log_enable': LaunchConfiguration('runtime_pos_log_enable'),
                    'batch_dt': LaunchConfiguration('batch_dt'),
                    'batch_omp': LaunchConfiguration('batch_omp'),
                    'batch_deskew': LaunchConfiguration('batch_deskew'),
                    'odometry.publish_odometry_without_downsample': LaunchConfiguration('pub_hifreq'),
                },
            ],
        ),
    ])
```
Note: launch-arg strings like `'true'`/`'false'` are coerced to the declared param types by `declare_parameter`. If type-coercion errors appear at launch, switch the offending arg to a typed `DeclareLaunchArgument` with `default_value='0.001'` and pass `value_type=...` — verify empirically in Task 12.

- [ ] **Step 4: Delete the obsolete ROS1 files**

```bash
git rm scripts/avia_batch.launch msg/LocalSensorExternalTrigger.msg
```

- [ ] **Step 5: Rebuild + smoke-check the node loads params**

```bash
cd ~/batch_lio_ws
colcon build --packages-select batch_lio --symlink-install
source install/setup.bash
timeout 5 ros2 launch batch_lio mapping_avia.launch.py rviz:=false
```
Expected: node starts, prints `[batch-LIO] batch_dt=...` and `lidar_type: 1`, no "undeclared parameter" errors, then exits on timeout. (It won't get data yet — that's Task 12.)

- [ ] **Step 6: Commit**

```bash
git add launch config
git commit -m "feat: ROS2 python launch + wrapped config yaml; drop ROS1 launch/msg"
```

---

## Task 12: Convert smoke bag + node smoke run

**Files:**
- Create: `~/batch_lio_ws/bags/quick-shack.mcap` (converted; out of repo)

**Interfaces:** A ROS2 bag the node can consume to produce odometry.

- [ ] **Step 1: Install the converter**

```bash
python3 -m pip install --user rosbags
```

- [ ] **Step 2: Convert the smallest ROS1 Avia bag to ROS2 mcap**

```bash
mkdir -p ~/batch_lio_ws/bags
rosbags-convert \
  --src /home/as/vllm/Nav/batch-lio/bags/2020-09-16-quick-shack.bag \
  --dst ~/batch_lio_ws/bags/quick-shack \
  --dst-type livox_ros_driver/CustomMsg=livox_ros_driver2/msg/CustomMsg
```
Expected: writes `~/batch_lio_ws/bags/quick-shack/metadata.yaml` + `quick-shack_0.mcap`. If `rosbags-convert` rejects the custom-type remap, fall back: write a tiny Python script using the `rosbags` library's `Writer` + a registered `livox_ros_driver2/msg/CustomMsg` typemap to re-emit the bag (the `rosbags` docs "Type stores" section covers this). Verify topics:

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 bag info ~/batch_lio_ws/bags/quick-shack
```
Expected: shows `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`) and `/livox/imu` (`sensor_msgs/msg/Imu`).

- [ ] **Step 3: Run the node against the bag and confirm odometry**

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 launch batch_lio mapping_avia.launch.py rviz:=false &
LAUNCH_PID=$!
sleep 3
ros2 bag play ~/batch_lio_ws/bags/quick-shack
ros2 topic hz /aft_mapped_to_init   # in another shell, or: ros2 topic echo /aft_mapped_to_init --once
kill $LAUNCH_PID
```
Expected: `/aft_mapped_to_init` publishes `nav_msgs/msg/Odometry` at a non-trivial rate; `Log/pos_log.txt` is written (if `runtime_pos_log_enable` is on). The trajectory should be non-degenerate (non-zero length). If the node crashes on init or never publishes, debug via the node log (most likely a QoS mismatch — switch `SensorDataQoS()` to `rclcpp::QoS(rclcpp::KeepLast(200000))` in Task 8 Step 8).

- [ ] **Step 4: No commit** (bag is out of repo). Record the bag path in `scripts/README` notes during Task 13.

---

## Task 13: Smoke launch_test + harness port + A/B repro

**Files:**
- Create: `test/test_smoke.py`
- Modify: `CMakeLists.txt` (register launch_test), `scripts/run_lio.sh`, `scripts/ablations.sh`
- Modify: `README.md` (ROS2 build/run section)

**Interfaces:** `colcon test` runs both gtest and the smoke launch_test green; `scripts/ablations.sh` reproduces the speedup + deskew numbers in ROS2.

- [ ] **Step 1: Write `test/test_smoke.py` (launch_test)**

```python
import os
import sys
import unittest
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

BAG = os.path.expanduser('~/batch_lio_ws/bags/quick-shack')


def generate_test_description():
    cfg = os.path.join(get_package_share_directory('batch_lio'), 'config', 'avia.yaml')
    mapping = launch_ros.actions.Node(
        package='batch_lio', executable='batchlio_mapping', name='laserMapping',
        parameters=[cfg], output='screen')
    return launch.LaunchDescription([
        mapping,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapping': mapping}


class TestOdomPublishes(unittest.TestCase):
    def test_publishes_odometry(self, mapping, proc_output):
        rclpy.init()
        node = Node('test_listener')
        got = []
        node.create_subscription(Odometry, '/aft_mapped_to_init',
                                 lambda m: got.append(m), 10)
        import threading, time
        # play the bag in a thread
        def play():
            os.system(f'ros2 bag play {BAG} --rate 2.0 >/dev/null 2>&1')
        t = threading.Thread(target=play, daemon=True)
        t.start()
        end = time.time() + 45.0
        while time.time() < end and not got:
            rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        rclpy.shutdown()
        self.assertGreater(len(got), 0, 'no odometry published within 45 s')
```

- [ ] **Step 2: Register the launch_test in `CMakeLists.txt`**

In the `if(BUILD_TESTING)` block, add:
```cmake
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smoke.py)
```

- [ ] **Step 3: Port `scripts/run_lio.sh` to ROS2**

Replace the ROS1 body with ROS2 equivalents (no roscore; `ros2 launch`, `ros2 node list`, `ros2 bag record/play`). Key substitutions:
- `source /opt/ros/noetic/setup.bash` → `source /opt/ros/humble/setup.bash`
- `source /root/batch-lio/catkin_ws/devel/setup.bash` → `source ~/batch_lio_ws/install/setup.bash`
- remove the `roscore` / `until rostopic list` block entirely
- `roslaunch "$LAUNCH" $LAUNCH_ARGS` → `ros2 launch batch_lio "$(basename "$LAUNCH")" $LAUNCH_ARGS` (LAUNCH now a `.launch.py` in the package share; pass just the basename)
- `rosnode list | grep -q laserMapping` → `ros2 node list | grep -q /laserMapping`
- `rosbag record -O "$OUTDIR/odom.bag" /aft_mapped_to_init __name:=odomrec` → `ros2 bag record -o "$OUTDIR/odom" /aft_mapped_to_init`
- `rosbag play -r "$RATE" "$BAG"` → `ros2 bag play -r "$RATE" "$BAG"`
- shutdown: `pkill -f batchlio_mapping`

- [ ] **Step 4: Port `scripts/ablations.sh`**

Update `B=/home/as/vllm/Batch-LIO`, point bags at `~/batch_lio_ws/bags/` (converted mcap dirs), `RL=$B/scripts/run_lio.sh`, `LB=avia_batch.launch.py`. ROS2 launch args keep the `key:=value` syntax. Replace the baseline (`$BL` = Point-LIO ROS1) comparisons with in-ROS2 point-wise runs (`batch_dt:=0.0` = Point-LIO behavior) so A/B is self-contained:
```bash
$RL $LB $QS $B/run/out/bw_pointwise_hifreq 1.0 $B "batch_dt:=0.0 pub_hifreq:=true batch_omp:=true"
$RL $LB $QS $B/run/out/bw_1ms_hifreq       1.0 $B "batch_dt:=0.001 pub_hifreq:=true batch_omp:=true"
# ... sweep and aggressive rounds, same pattern, true/false instead of 1/0
```

- [ ] **Step 5: Run the full test suite**

```bash
cd ~/batch_lio_ws
colcon build --packages-select batch_lio --symlink-install
colcon test --packages-select batch_lio --event-handlers console_direct+
colcon test-result --all
```
Expected: `test_deskew` PASS + `test_smoke` PASS.

- [ ] **Step 6: Convert the A/B bags and run the ablations**

```bash
rosbags-convert --src /home/as/vllm/Nav/batch-lio/bags/HKU_MB_2020-09-20-13-34-51.bag --dst ~/batch_lio_ws/bags/HKU_MB --dst-type livox_ros_driver/CustomMsg=livox_ros_driver2/msg/CustomMsg
rosbags-convert --src /home/as/vllm/Nav/batch-lio/bags/outdoor_run_100Hz_2020-12-27-17-12-19.bag --dst ~/batch_lio_ws/bags/outdoor_run --dst-type livox_ros_driver/CustomMsg=livox_ros_driver2/msg/CustomMsg
bash scripts/ablations.sh
python3 scripts/compare_traj.py batch_1ms run/out/agg_1ms_deskew/pos_log.txt run/out/agg_1ms_deskew/node.log run/out/agg_baseline/pos_log.txt
```
Expected: the batch (`batch_dt=0.001`) average total per-frame time is **2.3–3.6× lower** than point-wise (`batch_dt=0.0`), and the deskew-on run has **~8× lower** start→end drift than deskew-off on the aggressive bag — matching `docs/RESULTS.md`.

- [ ] **Step 7: Update `README.md` with a "Build (ROS2 Humble)" section**

Add a section after the existing Docker/Noetic section documenting: workspace setup (Task 1 symlinks), `colcon build`, the bag-conversion command, `ros2 launch batch_lio mapping_avia.launch.py`, and the test commands. Keep the ROS1 section as the baseline reference.

- [ ] **Step 8: Commit**

```bash
git add test/test_smoke.py CMakeLists.txt scripts/run_lio.sh scripts/ablations.sh README.md
git commit -m "test+eval: smoke launch_test, ROS2 A/B harness, reproduce speedup+deskew"
```

---

## Self-Review (run after writing this plan)

**Spec coverage:** Every locked decision maps to a task — native env (Global Constraint + every build step), full livox_ros_driver2 (Task 1), thin node port (Tasks 3-8), build system (Task 2), launch/config (Task 11), gtest (Task 10), launch_test (Task 13), A/B repro (Task 13 Step 6), drop dead msg + matplotlib (Task 11 + Task 4). The four success criteria map to Task 9 (build), Task 12 (run+odom), Task 13 Step 5 (colcon test green), Task 13 Step 6 (A/B repro).

**Placeholder scan:** No TBD/TODO. The rosbags fallback (Task 12 Step 2) and QoS fallback (Task 12 Step 3) name concrete fallback actions, not placeholders.

**Type consistency:** `readParameters(rclcpp::Node::SharedPtr)` declared (Task 4) and called (Task 8). `stamp_to_sec`/`stamp_from_sec` defined (Task 3) and used (Tasks 5-8). `livox_ros_driver2::msg::CustomMsg::ConstSharedPtr` consistent across preprocess (Task 5), li_initialization (Task 6), laserMapping (Task 8). `tf_broadcaster` declared + initialized (Task 8 Step 2 + Step 7). `test_deskew` target name consistent (Task 10 + 13).
