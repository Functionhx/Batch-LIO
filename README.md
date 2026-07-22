<div align="center">

# batch‑LIO

**A batch‑wise extension of [Point‑LIO](https://github.com/hku-mars/Point-LIO):**
per‑~1 ms batch EKF update with in‑batch motion de‑skew —
**higher‑bandwidth, lower‑compute LiDAR‑inertial odometry.**

🌐 **English** · [中文](README.zh-CN.md)

![ROS2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![C++](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=c%2B%2B&logoColor=white)
![Build](https://img.shields.io/badge/colcon%20test-passing-brightgreen)
![License](https://img.shields.io/badge/license-BSD-blue)

</div>

---

batch‑LIO reproduces **innovation #1** of the USTC undergraduate thesis
*《高带宽轮式激光惯性里程计》(Point‑LIWO — Batch‑based Direct Point LiDAR‑IMU‑Wheeled‑speed Odometry)*
by 张昊鹏. It is built directly on HKU‑MARS **Point‑LIO** and kept A/B‑comparable with it
(CPU‑only; wheel‑speed / innovation #2 is out of scope).

## Table of contents

| | Section |
|---|---|
| 1 | [What it changes vs Point‑LIO](#what-it-changes-vs-pointlio) |
| 2 | [Results](#results) |
| 3 | [Build & Run (ROS 2 Humble)](#build--run-ros-2-humble) |
| 4 | [Test](#test) |
| 5 | [Repository layout](#repository-layout) |
| 6 | [Legacy: ROS 1 Noetic](#legacy-ros-1-noetic) |
| 7 | [Attribution & license](#attribution--license) |

---

## What it changes vs Point‑LIO

Point‑LIO updates the EKF **point‑wise** (one update per distinct point timestamp) and already
row‑stacks the measurement Jacobian over a group of same‑timestamp points. batch‑LIO changes two
things and adds OpenMP:

1. **1 ms time‑window grouping** — points are grouped into fixed ~1 ms windows instead of by
   identical timestamp (`time_compressing_batch`, `include/common_lib.h`).
2. **In‑batch de‑skew** — because a window now spans many timestamps, each point is
   motion‑compensated to the window's reference (last‑point) time using the EKF state's
   angular/linear velocity, then all valid residuals are row‑stacked into a **single EKF update
   per window** (`src/deskew.h`, applied in `src/laserMapping.cpp`). This implements thesis
   eq. 3.44–3.47:
3. **OpenMP** on the per‑point KNN + plane‑fit loop (`src/Estimator.cpp`). OpenMP only pays off
   **after** batching enlarges the groups — on Point‑LIO's tiny per‑timestamp groups it is slower.

```
Δtⱼ = tⱼ − t_last                 (≤ 0 within a window)
Rⱼ  = Exp(ω · Δtⱼ)                (ω = state body angular velocity)
Tⱼ  = R_Iᵀ · v · Δtⱼ              (v = state world linear velocity; R_I = state rotation)
p'ⱼ = Rⱼ · pⱼ + Tⱼ
```

All changes are gated by ROS params (defaults make it equivalent to Point‑LIO when `batch_dt ≤ 0`).

| param | default | meaning |
|-------|---------|---------|
| `batch_dt` | `0.001` | batch window length in **seconds** (`≤ 0` ⇒ point‑wise = Point‑LIO) |
| `batch_omp` | `false` | OpenMP on the KNN+plane‑fit loop |
| `batch_deskew` | `true` | in‑window de‑skew on/off (ablation toggle) |

---

## Results

A/B vs pristine Point‑LIO, same bag and parameters, CPU‑only on a 32‑core x86_64 machine.
Full numbers in [`docs/RESULTS.md`](docs/RESULTS.md) (ROS 1) and [`docs/RESULTS_ROS2.md`](docs/RESULTS_ROS2.md)
(re‑run on the ROS 2 port).

### Per‑frame compute: 2.3–4.7× faster

![speedup](docs/figures/fig1_speedup.png)

| bag | point‑wise (ms) | batch 1 ms (ms) | speedup |
|-----|-----------------|-----------------|---------|
| quick‑shack (ROS 2) | 12.42 | 3.51 | **3.5×** |
| outdoor_run (ROS 2) | 2.56 | 0.54 | **4.7×** |

batch‑LIO matches the baseline trajectory closely and closes the `outdoor_run` loop *better* than
baseline (0.020 m vs 0.073 m).

### OpenMP only helps after batching ("batch enables parallelism")
![omp](docs/figures/fig2_omp_causality.png)

### batch_dt sweep: 1–2 ms is the sweet spot
![sweep](docs/figures/fig4_batchdt_sweep.png)

### High output bandwidth (publish per window)
| mode | stable odom rate |
|------|------------------|
| Point‑LIO frame‑rate odom | ~10 Hz |
| **batch‑LIO 1 ms (publish per window)** | **~913 Hz** |

---

## Build & Run (ROS 2 Humble)

Native ROS 2 Humble (`/opt/ros/humble`). The package depends on `livox_ros_driver2`
(its vendored SDK2 ships a prebuilt `liblivox_lidar_sdk_shared.so`, so no separate SDK build).

### 1 · Workspace + build

```bash
# livox_ros_driver2 as a sibling package; batch_lio symlinked from this repo
mkdir -p ~/batch_lio_ws/src
ln -sfn /path/to/livox_ros_driver2  ~/batch_lio_ws/src/livox_ros_driver2
ln -sfn /path/to/Batch-LIO         ~/batch_lio_ws/src/batch_lio
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws && colcon build --symlink-install
```

### 2 · Convert a ROS 1 Livox Avia bag → ROS 2 (mcap)

```bash
python3 -m pip install --user rosbags pyyaml
python3 scripts/convert_bag.py  your_avia.bag  ~/batch_lio_ws/bags/your_avia
# renames livox_ros_driver/CustomMsg -> livox_ros_driver2/msg/CustomMsg (byte-identical wire format)
```

### 3 · Run

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 launch batch_lio mapping_avia.launch.py            # rviz2 on; rviz:=false for headless
ros2 bag play ~/batch_lio_ws/bags/your_avia             # in another shell
ros2 topic echo /aft_mapped_to_init                     # odometry
```

Works on standard Livox Avia bags (`/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`,
`/livox/imu` = `sensor_msgs/msg/Imu`), e.g. the HKU‑MARS / FAST‑LIO Avia sequences.
Odometry on `/aft_mapped_to_init`; per‑frame stage timings as `[ mapping ]:` lines;
trajectory logged to `Log/pos_log.txt` when `runtime_pos_log_enable` is set.

---

## Test

```bash
cd ~/batch_lio_ws
colcon test --packages-select batch_lio          # deskew gtest (5) + smoke launch_test (1)
colcon test-result --all                         # 8 tests, 0 failures
```

| test | what it checks |
|---|---|
| `test_deskew` (gtest) | the eq. 3.44–3.47 de‑skew transform (pure translation, pure rotation, body‑frame velocity) |
| `test_smoke.py` (launch_test) | launches the node, plays a converted bag, asserts odometry is published |

A/B harness (speedup + de‑skew ablation):
```bash
bash scripts/ablations.sh
python3 scripts/compare_traj.py LABEL run/out/<run>/pos_log.txt run/out/<run>/node.log [baseline]
```

---

## Repository layout

```
src/            modified Point‑LIO sources
  deskew.h          NEW: in‑batch de‑skew (eq 3.44‑3.47), header‑only + unit‑tested
  laserMapping.cpp  batch grouping + per‑window de‑skew before the EKF update + OMP control
  Estimator.cpp     OpenMP on the KNN + plane‑fit loop
  parameters.*      batch_dt / batch_omp / batch_deskew params (type‑tolerant loader)
include/common_lib.h  time_compressing_batch (1 ms windows) + ROS 2 time helpers
launch/*.py     ROS 2 python launch (mapping_{avia,horizon,ouster64,velody16}, avia_batch)
config/*.yaml   ROS 2 params files (wrapped in /**: {ros__parameters:})
scripts/        convert_bag.py (ROS1→ROS2), run_lio.sh, ablations.sh, compare_traj.py
test/           test_deskew.cpp (gtest), test_smoke.py (launch_test)
docs/           RESULTS.md, RESULTS_ROS2.md, superpowers/{specs,plans}/, figures/
```

---

## Legacy: ROS 1 Noetic

The original ROS 1 Noetic (catkin) version is preserved on the `ros1-noetic` git tag. The ROS 2
port was a thin faithful port — same algorithm, same params; the only changes are the ROS plumbing
(ament, rclcpp, tf2, livox_ros_driver2). See the design spec and implementation plan under
`docs/superpowers/`.

```bash
git checkout ros1-noetic        # inspect the original ROS 1 version
```

---

## Attribution & license

- Built on **[Point‑LIO](https://github.com/hku-mars/Point-LIO)** (HKU‑MARS); please cite
  Point‑LIO and FAST‑LIO. The batch‑update idea reproduced here is from the USTC undergraduate
  thesis *《高带宽轮式激光惯性里程计》(Point‑LIWO)* by 张昊鹏.
- De‑skew follows the FAST‑LIO / sr_lio motion‑compensation convention; the map uses an
  iVox‑style hashed‑voxel structure.
- License follows the upstream Point‑LIO / LOAM / Livox license — see [`LICENSE`](LICENSE).
  This is a research reproduction.
