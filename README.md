<div align="center">

# batch‑LIO

**A batch‑wise extension of [Point‑LIO](https://github.com/hku-mars/Point-LIO):**
per‑~1 ms batch EKF update with in‑batch motion de‑skew —
**higher‑bandwidth, lower‑compute LiDAR‑inertial odometry.**

**[Point‑LIO](https://github.com/hku-mars/Point-LIO) 的批量扩展:**
每 ~1 ms 一个 batch 做 EKF 更新,并在 batch 内做运动去畸变 ——
**更高带宽、更低算力的激光‑惯性里程计。**

![ROS2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![C++](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=c%2B%2B&logoColor=white)
![Build](https://img.shields.io/badge/colcon%20test-passing-brightgreen)
![License](https://img.shields.io/badge/license-BSD-blue)

</div>

---

> **EN** · batch‑LIO reproduces **innovation #1** of the USTC undergraduate thesis
> *《高带宽轮式激光惯性里程计》(Point‑LIWO — Batch‑based Direct Point LiDAR‑IMU‑Wheeled‑speed Odometry)*
> by 张昊鹏. It is built directly on HKU‑MARS **Point‑LIO** and kept A/B‑comparable with it
> (CPU‑only; wheel‑speed / innovation #2 is out of scope).
>
> **中文** · batch‑LIO 复现张昊鹏本科毕业设计 *《高带宽轮式激光惯性里程计》(Point‑LIWO)* 的**创新点一**。
> 直接基于港大 Mars Lab 的 **Point‑LIO**,与其保持 A/B 可比(纯 CPU;轮速 / 创新点二不在范围内)。

## Table of contents · 目录

| | Section · 章节 |
|---|---|
| 1 | [What it changes vs Point‑LIO · 相比 Point‑LIO 的改动](#what-it-changes-vs-pointlio--相比-pointlio-的改动) |
| 2 | [Results · 结果](#results--结果) |
| 3 | [Build & Run (ROS 2 Humble) · 构建与运行](#build--run-ros-2-humble--构建与运行) |
| 4 | [Test · 测试](#test--测试) |
| 5 | [Repository layout · 目录结构](#repository-layout--目录结构) |
| 6 | [Legacy: ROS 1 Noetic · 旧版](#legacy-ros-1-noetic--旧版) |
| 7 | [Attribution & license · 致谢与许可](#attribution--license--致谢与许可) |

---

## What it changes vs Point‑LIO · 相比 Point‑LIO 的改动

**EN** — Point‑LIO updates the EKF **point‑wise** (one update per distinct point timestamp) and
already row‑stacks the measurement Jacobian over a group of same‑timestamp points. batch‑LIO
changes two things and adds OpenMP:

1. **1 ms time‑window grouping** — points are grouped into fixed ~1 ms windows instead of by
   identical timestamp (`time_compressing_batch`, `include/common_lib.h`).
2. **In‑batch de‑skew** — because a window now spans many timestamps, each point is
   motion‑compensated to the window's reference (last‑point) time using the EKF state's
   angular/linear velocity, then all valid residuals are row‑stacked into a **single EKF update
   per window** (`src/deskew.h`, applied in `src/laserMapping.cpp`). This implements thesis
   eq. 3.44–3.47:
3. **OpenMP** on the per‑point KNN + plane‑fit loop (`src/Estimator.cpp`). OpenMP only pays off
   **after** batching enlarges the groups — on Point‑LIO's tiny per‑timestamp groups it is slower.

**中文** —— Point‑LIO 按**逐点**更新 EKF(每个不同点时间戳一次更新),并在同一时间戳的点组上
行堆叠量测雅可比。batch‑LIO 改了两处并加入 OpenMP:

1. **1ms 时间窗分组** —— 把点按固定 ~1ms 窗口分组,而非按相同时间戳(`time_compressing_batch`,
   `include/common_lib.h`)。
2. **batch 内去畸变** —— 因为一个窗口跨越多个时间戳,每个点用 EKF 状态的角速度/线速度
   运动补偿到窗口参考(末点)时刻,然后所有有效残差行堆叠成**每个窗口一次 EKF 更新**
   (`src/deskew.h`,在 `src/laserMapping.cpp` 中调用)。对应毕业设计公式 3.44–3.47:

```
Δtⱼ = tⱼ − t_last                 (≤ 0 within a window / 窗口内 ≤ 0)
Rⱼ  = Exp(ω · Δtⱼ)                (ω = state body angular velocity / 体坐标系角速度)
Tⱼ  = R_Iᵀ · v · Δtⱼ              (v = state world linear velocity; R_I = state rotation)
p'ⱼ = Rⱼ · pⱼ + Tⱼ
```

All changes are gated by ROS params (defaults make it equivalent to Point‑LIO when `batch_dt ≤ 0`).
所有改动都由 ROS 参数控制(`batch_dt ≤ 0` 时退化为 Point‑LIO)。

| param | default | meaning · 含义 |
|-------|---------|---------|
| `batch_dt` | `0.001` | batch window length in **seconds** (`≤ 0` ⇒ point‑wise = Point‑LIO) · batch 窗口长度(**秒**) |
| `batch_omp` | `false` | OpenMP on the KNN+plane‑fit loop · KNN+平面拟合循环的 OpenMP |
| `batch_deskew` | `true` | in‑window de‑skew on/off (ablation toggle) · 窗口内去畸变开关(消融用) |

---

## Results · 结果

**EN** — A/B vs pristine Point‑LIO, same bag and parameters, CPU‑only on a 32‑core x86_64 machine.
Full numbers in [`docs/RESULTS.md`](docs/RESULTS.md) (ROS 1) and [`docs/RESULTS_ROS2.md`](docs/RESULTS_ROS2.md)
(re‑run on the ROS 2 port).

**中文** —— 与原版 Point‑LIO 的 A/B,相同 bag 与参数,32 核 x86_64 纯 CPU。
完整数据见 [`docs/RESULTS.md`](docs/RESULTS.md)(ROS 1)与 [`docs/RESULTS_ROS2.md`](docs/RESULTS_ROS2.md)(ROS 2 移植复测)。

### Per‑frame compute: 2.3–4.7× faster · 每帧算力:快 2.3–4.7×

![speedup](docs/figures/fig1_speedup.png)

| bag | point‑wise (ms) | batch 1 ms (ms) | speedup · 加速比 |
|-----|-----------------|-----------------|------------------|
| quick‑shack (ROS 2) | 12.42 | 3.51 | **3.5×** |
| outdoor_run (ROS 2) | 2.56 | 0.54 | **4.7×** |

batch‑LIO matches the baseline trajectory closely and closes the `outdoor_run` loop *better* than
baseline (0.020 m vs 0.073 m). · batch‑LIO 轨迹与基线高度吻合,且 `outdoor_run` 闭环误差比基线*更小*。

### OpenMP only helps after batching · OpenMP 只在 batch 后才有效
![omp](docs/figures/fig2_omp_causality.png)

### batch_dt sweep: 1–2 ms is the sweet spot · batch_dt 扫描:1–2ms 为甜点区
![sweep](docs/figures/fig4_batchdt_sweep.png)

### High output bandwidth (publish per window) · 高输出带宽(每窗口发布)
| mode · 模式 | stable odom rate · 稳定里程计频率 |
|------|------------------|
| Point‑LIO frame‑rate odom | ~10 Hz |
| **batch‑LIO 1 ms (publish per window)** | **~913 Hz** |

---

## Build & Run (ROS 2 Humble) · 构建与运行

**EN** — Native ROS 2 Humble (`/opt/ros/humble`). The package depends on `livox_ros_driver2`
(its vendored SDK2 ships a prebuilt `liblivox_lidar_sdk_shared.so`, so no separate SDK build).

**中文** —— 原生 ROS 2 Humble(`/opt/ros/humble`)。依赖 `livox_ros_driver2`
(其内置 SDK2 已附带预编译 `liblivox_lidar_sdk_shared.so`,无需单独编译 SDK)。

### 1 · Workspace + build · 工作空间与编译

```bash
# livox_ros_driver2 as a sibling package; batch_lio symlinked from this repo
mkdir -p ~/batch_lio_ws/src
ln -sfn /path/to/livox_ros_driver2  ~/batch_lio_ws/src/livox_ros_driver2
ln -sfn /path/to/Batch-LIO         ~/batch_lio_ws/src/batch_lio
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws && colcon build --symlink-install
```

### 2 · Convert a ROS 1 Livox Avia bag → ROS 2 (mcap) · 转换 ROS 1 bag

```bash
python3 -m pip install --user rosbags pyyaml
python3 scripts/convert_bag.py  your_avia.bag  ~/batch_lio_ws/bags/your_avia
# renames livox_ros_driver/CustomMsg -> livox_ros_driver2/msg/CustomMsg (byte-identical wire format)
```

### 3 · Run · 运行

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 launch batch_lio mapping_avia.launch.py            # rviz2 on; rviz:=false for headless
ros2 bag play ~/batch_lio_ws/bags/your_avia             # in another shell / 另开终端
ros2 topic echo /aft_mapped_to_init                     # odometry / 里程计
```

Works on standard Livox Avia bags (`/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`,
`/livox/imu` = `sensor_msgs/msg/Imu`), e.g. the HKU‑MARS / FAST‑LIO Avia sequences.
Odometry on `/aft_mapped_to_init`; per‑frame stage timings as `[ mapping ]:` lines;
trajectory logged to `Log/pos_log.txt` when `runtime_pos_log_enable` is set.

---

## Test · 测试

```bash
cd ~/batch_lio_ws
colcon test --packages-select batch_lio          # deskew gtest (5) + smoke launch_test (1)
colcon test-result --all                         # 8 tests, 0 failures
```

| test · 测试 | what it checks · 检查内容 |
|---|---|
| `test_deskew` (gtest) | the eq. 3.44–3.47 de‑skew transform (pure translation, pure rotation, body‑frame velocity) · 去畸变变换(纯平移、纯旋转、体速度) |
| `test_smoke.py` (launch_test) | launches the node, plays a converted bag, asserts odometry is published · 启动节点+回放 bag,断言有里程计输出 |

A/B harness (speedup + de‑skew ablation): · A/B 测试框架(加速比 + 去畸变消融):
```bash
bash scripts/ablations.sh
python3 scripts/compare_traj.py LABEL run/out/<run>/pos_log.txt run/out/<run>/node.log [baseline]
```

---

## Repository layout · 目录结构

```
src/            modified Point‑LIO sources / 改动的 Point‑LIO 源码
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

## Legacy: ROS 1 Noetic · 旧版

**EN** — The original ROS 1 Noetic (catkin) version is preserved on the `ros1-noetic` git tag.
The ROS 2 port was a thin faithful port — same algorithm, same params; the only changes are the
ROS plumbing (ament, rclcpp, tf2, livox_ros_driver2). See the design spec and implementation plan
under `docs/superpowers/`.

**中文** —— 原始的 ROS 1 Noetic (catkin) 版本保留在 git 标签 `ros1-noetic` 上。
ROS 2 移植为"薄而忠实"的移植 —— 算法与参数完全一致,只改 ROS 接口
(ament、rclcpp、tf2、livox_ros_driver2)。设计文档与实施计划见 `docs/superpowers/`。

```bash
git checkout ros1-noetic        # inspect the original ROS 1 version / 查看原始 ROS 1 版本
```

---

## Attribution & license · 致谢与许可

- Built on **[Point‑LIO](https://github.com/hku-mars/Point-LIO)** (HKU‑MARS); please cite
  Point‑LIO and FAST‑LIO. The batch‑update idea reproduced here is from the USTC undergraduate
  thesis *《高带宽轮式激光惯性里程计》(Point‑LIWO)* by 张昊鹏.
- De‑skew follows the FAST‑LIO / sr_lio motion‑compensation convention; the map uses an
  iVox‑style hashed‑voxel structure.
- License follows the upstream Point‑LIO / LOAM / Livox license — see [`LICENSE`](LICENSE).
  This is a research reproduction.

- 基于港大 Mars Lab 的 **[Point‑LIO](https://github.com/hku-mars/Point-LIO)**;请引用 Point‑LIO
  与 FAST‑LIO。此处复现的批量更新思想来自张昊鹏本科毕业设计 *《高带宽轮式激光惯性里程计》(Point‑LIWO)*。
- 去畸变遵循 FAST‑LIO / sr_lio 的运动补偿约定;地图采用 iVox 风格的哈希体素结构。
- 许可证沿用上游 Point‑LIO / LOAM / Livox 许可证 —— 见 [`LICENSE`](LICENSE)。本仓库为研究复现。
