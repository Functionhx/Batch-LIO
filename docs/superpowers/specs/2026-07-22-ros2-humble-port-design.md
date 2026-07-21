# Design: batch-LIO ROS2 Humble port

**Date:** 2026-07-22
**Status:** Approved (brainstormed 2026-07-22)
**Branch:** `ros2-humble` (off `main`; `main` stays the pristine ROS1 Noetic A/B baseline)

## Goal

Port the existing ROS1 Noetic (catkin) `batch_lio` package to a native ROS2 Humble
(ament_cmake) package, preserving its batch-wise Point-LIO behavior, and validate it
end-to-end on the same Avia bags used for the original A/B study — including reproducing
the documented speedup and deskew-correctness numbers. All development and testing runs
on the host's native `/opt/ros/humble`.

## Non-goals

- Wheel-speed / innovation #2 (Point-LIWO) — out of scope, unchanged from the ROS1 repo.
- Live Livox sensor capture — the node only consumes `CustomMsg` from bag replay. The full
  `livox_ros_driver2` is built only to obtain the message type and to keep the path to live
  capture open later; no hardware is exercised in this port.
- Algorithm changes — the EKF / iVox / deskew math is untouched.

## Decisions (locked)

| Decision | Choice | Rationale |
|---|---|---|
| Environment | Native `/opt/ros/humble` | Already installed with all packages; 32 cores; bags local; fastest loop |
| Livox message | Build full `livox_ros_driver2` in workspace | SDK2 ships prebuilt (`lib/amd64/liblivox_lidar_sdk_shared.so`); exports `livox_ros_driver2/msg/CustomMsg`; fields are a superset of the ROS1 fields the code reads |
| Scope | Full: build + run + RViz + A/B repro + gtest + launch_test | User-chosen maximum thoroughness |
| Node structure | Thin faithful port (keep `main()` poll loop) | Smallest diff, exact control flow, A/B-comparable; standard FAST-LIO ROS2 port pattern |
| Dead code | Drop `LocalSensorExternalTrigger.msg` (unused) + optional `matplotlib` viz | YAGNI; leaner build, no Python build dep |

## Architecture

### Workspace layout (colcon)
```
ros2_ws/src/
  livox_ros_driver2/      # from Nav/pb2025_sentry_nav/livox_ros_driver2 (vendored SDK2)
  batch_lio/              # the ported package (this repo on branch ros2-humble)
```
`livox_ros_driver2` is a sibling package; `batch_lio` `<depend>`s on it. Build order is
resolved automatically by ament from the dependency declaration.

### Node port (approach A — thin faithful port)
`laserMapping.cpp` keeps its structure: `main()` creates the node, loads params, creates
subscriptions/publishers, then runs the existing `while(ok)` 500 Hz poll loop. Each ROS1 API
call is replaced 1:1 with its ROS2 equivalent:

| ROS1 (catkin) | ROS2 Humble (ament) |
|---|---|
| `ros::init` / `NodeHandle("~")` / `AsyncSpinner` / `ros::spinOnce()` / `ros::Rate` / `ros::ok()` | `rclcpp::init` / `rclcpp::Node` / `SingleThreadedExecutor::spin_some()` / `rclcpp::Rate` / `rclcpp::ok()` |
| `nh.param<T>(name,var,def)` (~50 calls) | helper `get_param<T>(node,name,var,def)` = `declare_parameter`+`get_parameter`; namespaced `"mapping/satu_acc"` → `"mapping.satu_acc"` |
| `ros::Publisher` + `advertise` (cloud-world, cloud-body, cloud-map, odom, path) | `create_publisher` |
| `ros::Subscriber` + `subscribe` (LiDAR + IMU) | `create_subscription`; callbacks take `::SharedPtr` |
| `livox_ros_driver::CustomMsg::ConstPtr` | `livox_ros_driver2::msg::CustomMsg::SharedPtr` (field access identical) |
| `tf::TransformBroadcaster` + `tf::StampedTransform` | `tf2_ros::TransformBroadcaster` + `geometry_msgs::msg::TransformStamped` |
| `ros::Time().fromSec(x)` / `.toSec()` / `::now()` | `rclcpp::Time(x)` / `.seconds()` / `node->now()` |
| `pcl::toROSMsg` / `fromROSMsg` | `pcl_conversions::toROSMsg` / `fromROSMsg` |
| `ROS_WARN/INFO(...)` | `RCLCPP_WARN/INFO(logger, ...)` |

Threading: a single `SingleThreadedExecutor`; `spin_some()` inside the loop. This preserves
Point-LIO's deliberate single-threaded poll semantics (callbacks push into the existing
mutex-guarded deques; fusion runs in the loop body). No `MultiThreadedExecutor`.

### Build system
- `CMakeLists.txt` → ament: `find_package(rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros
  tf2_eigen pcl_conversions livox_ros_driver2 REQUIRED)`, `find_package(PCL Eigen3 OpenMP
  REQUIRED)`, `ament_target_dependencies`, `install(TARGETS batchlio_mapping DESTINATION
  lib/${PROJECT_NAME})`, `install(DIRECTORY launch config rviz_cfg)`. OpenMP thread-count
  logic (`MP_EN` / `MP_PROC_NUM`) preserved.
- `package.xml` → format 3, `<buildtool_depend>ament_cmake</buildtool_depend>`,
  `<depend>livox_ros_driver2</depend>`, `<test_depend>ament_cmake_gtest launch_ros
  launch_testing_ament_cmake</test_depend>`.

### Launch + config
- 5 XML `.launch` → 5 Python `*.launch.py` (`mapping_avia.launch.py`, `mapping_horizon.launch.py`,
  `mapping_ouster64.launch.py`, `mapping_velody16.launch.py`, plus a headless
  `avia_batch.launch.py` parity of `scripts/avia_batch.launch`). Launch args expose
  `batch_dt` / `batch_omp` / `batch_deskew` / `pub_hifreq`. Optional RViz via `rviz` arg.
- 4 config YAMLs wrapped in the ROS2 param-file envelope `/**: {ros__parameters: …}`. The
  existing nesting (`common:`, `preprocess:`, `mapping:`, `odometry:`, `publish:`, `pcd_save:`)
  maps 1:1 onto ROS2 param hierarchy, so this is a mechanical 2-indent wrap.

### Test data + harness (for A/B repro)
- `pip install rosbags`; convert ROS1 `.bag` → ROS2 `.mcap` with type remap
  `livox_ros_driver/CustomMsg → livox_ros_driver2/msg/CustomMsg`; topics `/livox/lidar`,
  `/livox/imu` preserved.
- Smoke-test on the 218 MB `2020-09-16-quick-shack.bag`; A/B on `HKU_MB_…bag` (1.2 GB) and
  `outdoor_run_100Hz_…bag` (2.0 GB) — the sequences backing `docs/RESULTS.md`.
- Port `scripts/{run_lio.sh, compare_traj.py, ablations.sh}` to `ros2 bag play` / `ros2 launch`
  / `ros2 topic` (no `roscore`). The odometry topic `/aft_mapped_to_init` keeps its name, so
  `compare_traj.py`'s trajectory/`pos_log.txt` parsing is essentially unchanged.

## Testing — `colcon test` green

- **gtest**: `test/test_deskew.cpp` (standalone Eigen, no ROS) wired up with
  `ament_add_gtest`; asserts the eq-3.44–3.47 intra-window deskew transform. Test body unchanged.
- **launch_test**: `test/test_smoke.py` launches the node and plays the converted smoke bag for
  a bounded window; asserts `/aft_mapped_to_init` publishes odometry + TF within a timeout.
- **A/B verification**: re-run `ablations.sh` in ROS2; confirm the 2.3–3.6× per-frame speedup
  and the 8×-lower deskew drift hold within tolerance vs the documented ROS1 numbers.

## Success criteria

1. `colcon build` clean for `{livox_ros_driver2, batch_lio}` on native Humble.
2. `ros2 launch batch_lio mapping_avia.launch.py` runs on the converted smoke bag; odometry,
   TF, point clouds, and path publish; RViz displays them.
3. Deskew gtest **and** smoke launch_test pass; `colcon test --packages-select batch_lio` green.
4. A/B reproduction in ROS2 matches the documented ROS1 numbers within tolerance (speedup band
   and deskew drift ratio).

## Risks

- **rosbags type remap for `CustomMsg`** is the most failure-prone test step; if the converter
  cannot remap the custom type directly, fall back to recording a short ROS2 bag from a small
  custom publisher (re-emit a slice of a ROS1 bag's points), or vendor a rosbags typemap.
- **ROS2 `PointCloud2`/`PCL` field handling**: Point-LIO carries per-point time in the PCL
  `curvature` field and builds `PointCloud2` via `pcl::toROSMsg` — both are ROS-agnostic, so no
  semantic change is expected, but the smoke launch_test is the guard.
- **Param-name mismatches** between `readParameters` and the wrapped YAML are caught by the node
  failing to load a param (ament warns on undeclared params); verified at first launch.
