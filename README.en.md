<div align="center">

<!-- ═══════════════════ ASSET SLOT 1 / HERO ═══════════════════
     Drop assets/hero.png here (1280×360 suggested; supply a dark and a
     light variant to follow the reader's GitHub theme):

     <picture>
       <source media="(prefers-color-scheme: dark)"  srcset="assets/hero-dark.png">
       <source media="(prefers-color-scheme: light)" srcset="assets/hero-light.png">
       <img src="assets/hero-light.png" alt="Batch-LIO" width="820">
     </picture>
     ════════════════════════════════════════════════════════════ -->

# Batch‑LIO

### LiDAR‑inertial odometry that collapses per‑point updates into per‑window updates

**A batch‑wise extension of [Point‑LIO](https://github.com/hku-mars/Point-LIO)** — points are
grouped into configurable sub‑millisecond time windows,<br>
motion‑de‑skewed in‑window, then row‑stacked into **one** EKF update per window.

**Bounded resources · lower per‑frame compute · the original Point‑LIO control path always preserved**

<br>

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](#build--run)
[![C++](https://img.shields.io/badge/C%2B%2B-14-00599C?style=flat-square&logo=cplusplus&logoColor=white)](CMakeLists.txt)
[![Tests](https://img.shields.io/badge/colcon_test-52_passed-3fb950?style=flat-square&logo=github-actions&logoColor=white)](#test)
[![Benchmark](https://img.shields.io/badge/ground_truth-N%3D3_all_gates_passed-3fb950?style=flat-square)](docs/CPU_BENCHMARK_PROTOCOL.md)
[![License](https://img.shields.io/badge/license-MIT-0969da?style=flat-square)](LICENSE)

[中文](README.md) · **English**

<br>

<!-- ═══════════════════ ASSET SLOT 2 / DEMO ═══════════════════
     Drop assets/demo.gif here (≤ 8 MB, 960px wide, mapping or RViz capture).
     <img src="assets/demo.gif" alt="Batch-LIO mapping" width="820">
     ════════════════════════════════════════════════════════════ -->

</div>

---

## Headline results

Relative to the Point‑LIO control **in the same binary** (`batch_dt = 0`), across three
ground‑truth datasets, formal N=3 medians:

<table>
<tr>
<th align="left" width="26%">Profile</th>
<th align="center" width="16%">HILTI&nbsp;2021</th>
<th align="center" width="16%">HILTI&nbsp;2022</th>
<th align="center" width="16%">TIERS</th>
<th align="center" width="26%">Map semantics</th>
</tr>
<tr>
<td><b>Batch‑LIO serial</b><br><sub>0.2 ms · stable default</sub></td>
<td align="center"><b>−12.2%</b><br><sub>CPU −11.0%</sub></td>
<td align="center"><b>−5.5%</b><br><sub>CPU −3.1%</sub></td>
<td align="center"><b>−9.4%</b><br><sub>CPU −4.8%</sub></td>
<td align="center">original iVox<br><sub>strictly A/B‑comparable</sub></td>
</tr>
<tr>
<td><b>representative CPU</b><br><sub>1 ms · opt‑in only</sub></td>
<td align="center"><b>−72.4%</b><br><sub>CPU −63.6%</sub></td>
<td align="center"><b>−71.7%</b><br><sub>CPU −46.1%</sub></td>
<td align="center"><b>−79.6%</b><br><sub>CPU −68.0%</sub></td>
<td align="center">⚠️ different<br><sub>not valid for strict A/B</sub></td>
</tr>
</table>

<sub>Figures are **per‑frame latency** reductions. Both profiles passed **every** ATE, RPE,
kernel‑HWM RSS, frame‑count (≥ 95% of control), deterministic‑input and process‑health gate on all
three datasets; each dataset × profile produced one identical trajectory SHA across its three
repeats. Protocol in [`docs/CPU_BENCHMARK_PROTOCOL.md`](docs/CPU_BENCHMARK_PROTOCOL.md), raw
summary in [`eval/benchmarks/20260810_cpu_final_v3_n3/`](eval/benchmarks/20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md).</sub>

> [!IMPORTANT]
> **The win is compute, not bandwidth.** Batching **reduces** the EKF update count
> (windows ≪ points), so the raw max publish rate goes **down**, not up. Any "×N bandwidth"
> claim is wrong.

---

## Contents

[Headline results](#headline-results) ·
[About](#about) ·
[What it changes vs Point‑LIO](#what-it-changes-vs-pointlio) ·
[Full results](#full-results) ·
[Build & run](#build--run) ·
[Test](#test) ·
[Repository layout](#repository-layout) ·
[Future work](#future-work) ·
[Legacy ROS 1](#legacy-ros-1-noetic) ·
[Attribution & license](#attribution--license)

---

## About

Batch‑LIO reproduces **innovation #1** of the USTC undergraduate thesis
*《高带宽轮式激光惯性里程计》(Point‑LIWO — Batch‑based Direct Point LiDAR‑IMU‑Wheeled‑speed Odometry)*
by 张昊鹏. It is built directly on HKU‑MARS **Point‑LIO** and kept A/B‑comparable with it
(CPU‑only; wheel‑speed / innovation #2 is out of scope).

There is exactly one design constraint: **the original Point‑LIO path must be preserved bit for
bit.** Set `batch_dt ≤ 0` and the code walks the identical point‑wise branch — that is both the
ablation control and the denominator behind every performance claim here.

---

## What it changes vs Point‑LIO

<!-- ═══════════════════ ASSET SLOT 3 / PIPELINE ═══════════════════
     Drop assets/pipeline.svg here — the data flow:
       cloud → time-window grouping → in-window de-skew → row-stack → one IEKF update → iVox
     SVG stays crisp in both themes; supply 2× resolution for raster.
     <img src="assets/pipeline.svg" alt="Batch-LIO data flow" width="100%">
     ════════════════════════════════════════════════════════════════ -->

Point‑LIO updates the EKF **per point** (one update per distinct point timestamp), row‑stacking
measurement Jacobians over points sharing a timestamp. Batch‑LIO changes two things and adds OpenMP:

**1 · Configurable sub‑millisecond grouping**
Points are grouped into fixed time windows instead of by identical timestamp
(`time_compressing_batch`, `include/common_lib.h`). The ground‑truth‑validated original‑map CPU
default is **0.2 ms**; the opt‑in representative‑map profile is pinned to 1 ms.

**2 · In‑batch de‑skew**
Because a window spans several timestamps, each point is motion‑compensated to the window
reference (last‑point) time using the EKF state's angular/linear velocity, then all valid residual
rows are stacked into **one EKF update per window** (`src/deskew.h`, called from
`src/laserMapping.cpp`). This matches thesis equations 3.44–3.47:

```
Δtⱼ = tⱼ − t_last                 (≤ 0 within the window)
Rⱼ  = Exp(ω · Δtⱼ)                (ω = state body angular velocity)
Tⱼ  = R_Iᵀ · v · Δtⱼ              (v = state world linear velocity; R_I = state rotation)
p'ⱼ = Rⱼ · pⱼ + Tⱼ
```

**3 · OpenMP** over the per‑point KNN + plane‑fit loop (`src/Estimator.cpp`).
OpenMP only pays off once batching has **enlarged** the point groups — on Point‑LIO's naturally
tiny same‑timestamp groups it is slower.

<details>
<summary><b>All ROS parameters</b> (everything is param-gated; <code>batch_dt ≤ 0</code> reproduces Point‑LIO)</summary>

<br>

| Parameter | Default | Meaning |
|------|--------|------|
| `batch_dt` | `0.0002` | validated original‑map CPU batch window in **seconds** (`≤ 0` ⇒ point‑wise = Point‑LIO) |
| `batch_omp` | `false` | OpenMP over the KNN + plane‑fit loop |
| `batch_omp_threads` | `2` | OpenMP threads when enabled (`≤ 0` uses the runtime default, capped at 16) |
| `batch_omp_min_points` | `8` | keep smaller measurement batches serial to avoid team wake‑up overhead |
| `batch_deskew` | `true` | in‑window de‑skew on/off (ablation toggle) |
| `common.lidar_qos_depth` | `32` | bounded LiDAR backlog; avoids drops from the SensorDataQoS depth‑5 default |
| `common.imu_qos_depth` | `512` | bounded backlog for high‑rate IMU samples |
| `common.sensor_qos_reliable` | `false` | hardware‑safe best‑effort default; the offline benchmark enables reliable delivery explicitly |

</details>

---

## Full results

Full numbers in [`docs/RESULTS.md`](docs/RESULTS.md) (ROS 1) and
[`docs/RESULTS_ROS2.md`](docs/RESULTS_ROS2.md) (re‑run on the ROS 2 port).

> [!NOTE]
> The historical quick‑shack / outdoor_run results below **do not use ground truth**. "Accuracy"
> there means **agreement with the Point‑LIO baseline** (mean \|Δpos\|) plus **loop‑closure drift**
> on return‑loop bags — not error against a true trajectory.

### Per‑frame compute: 2.3–4.7× faster

![speedup](docs/figures/fig1_speedup.png)

| bag | point‑wise (ms) | batch 1 ms (ms) | speedup |
|-----|-----------------|-----------------|---------|
| quick‑shack (ROS 2) | 12.42 | 3.51 | **3.5×** |
| outdoor_run (ROS 2) | 2.56 | 0.54 | **4.7×** |

Batch‑LIO matches the baseline trajectory closely and closes the `outdoor_run` loop *better* than
baseline (0.020 m vs 0.073 m).

### OpenMP only helps after batching ("batch enables parallelism")

![omp](docs/figures/fig2_omp_causality.png)

### Historical no‑truth batch_dt sweep: 1–2 ms was the speed/agreement sweet spot

![sweep](docs/figures/fig4_batchdt_sweep.png)

### Odometry rate depends on publish policy

| scheme | publish policy | stable odom rate |
|--------|----------------|------------------|
| Point‑LIO | per frame (default) | ~10 Hz |
| Point‑LIO | per point | ~6.7 kHz (noisy) |
| Batch‑LIO 1 ms | per window | ~913 Hz |

Point‑LIO isn't limited to 10 Hz either — its per‑point mode reaches ~6.7 kHz. An honest
comparison has to state the publish policy.

---

## Build & run

Native ROS 2 Humble (`/opt/ros/humble`). The package depends on `livox_ros_driver2`
(its vendored SDK2 ships a prebuilt `liblivox_lidar_sdk_shared.so`, so no separate SDK build).
For Jazzy, use the build‑and‑test‑gated [`docker/jazzy.Dockerfile`](docker/jazzy.Dockerfile);
complete Humble and Jazzy image commands are in [`docker/README.md`](docker/README.md).

**1 · Workspace + build**

```bash
# livox_ros_driver2 as a sibling package; batch_lio symlinked from this repo
mkdir -p ~/batch_lio_ws/src
ln -sfn /path/to/livox_ros_driver2  ~/batch_lio_ws/src/livox_ros_driver2
ln -sfn /path/to/Batch-LIO          ~/batch_lio_ws/src/batch_lio
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws && colcon build --symlink-install
```

**2 · Convert a ROS 1 Livox Avia bag → ROS 2 (mcap)**

```bash
python3 -m pip install --user rosbags pyyaml
python3 scripts/convert_bag.py  your_avia.bag  ~/batch_lio_ws/bags/your_avia
# renames livox_ros_driver/CustomMsg to livox_ros_driver2/msg/CustomMsg (wire format is byte-identical)
```

**3 · Run**

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 launch batch_lio mapping_avia.launch.py     # rviz2 on by default; rviz:=false for headless
ros2 bag play ~/batch_lio_ws/bags/your_avia      # another shell
ros2 topic echo /aft_mapped_to_init              # odometry
```

<details>
<summary><b>Optional: CPU-only representative-map profile</b> (different map semantics)</summary>

<br>

For a strict Point‑LIO comparison, **keep using** `mapping_avia.launch.py`. Only when different
map semantics are acceptable:

```bash
ros2 launch batch_lio mapping_avia_cpu_fast.launch.py rviz:=false
```

</details>

Works with standard Livox Avia bags (`/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`,
`/livox/imu` = `sensor_msgs/msg/Imu`), e.g. the HKU‑MARS / FAST‑LIO Avia sequences.
Odometry is published on `/aft_mapped_to_init`; per‑frame stage timings print as `[ mapping ]:`
lines; trajectories go to `Log/pos_log.txt` when `runtime_pos_log_enable` is set.

---

## Test

```bash
cd ~/batch_lio_ws
colcon test --packages-select batch_lio
colcon test-result --all --verbose               # 12 CTest targets, 52 tests, 0 failures
```

| Test | What it checks |
|---|---|
| 10 gtest targets | de‑skew, joint information update, params/preprocess, gravity, plane fit, original/representative iVox, robust measurement, bounded profiler |
| `test_smoke.py` (launch_test) | starts the node, replays a ROS 2 bag, asserts ≥ 5 odometry messages and a clean exit |
| `test_benchmark_scripts.py` (pytest) | kernel‑HWM accounting, legacy resource‑log compatibility, machine‑readable IMU init sequence |

A/B harness (speedup + de‑skew ablation):

```bash
bash scripts/ablations.sh
python3 scripts/compare_traj.py LABEL run/out/<run>/pos_log.txt run/out/<run>/node.log [baseline]
```

---

## Repository layout

```
src/                   modified Point‑LIO sources
  deskew.h             new: in‑batch de‑skew (eq. 3.44‑3.47), header‑only + unit‑tested
  laserMapping.cpp     batch grouping + per‑window de‑skew before the EKF update + OMP thread control
  Estimator.cpp        OpenMP over the KNN + plane‑fit loop
  parameters.*         batch_dt / batch_omp / batch_deskew params (type‑tolerant loader)
include/common_lib.h   configurable time_compressing_batch windows + ROS 2 time helpers
launch/*.py            ROS 2 python launch (mapping_{avia,horizon,ouster64,velody16}, avia_batch)
config/*.yaml          ROS 2 parameter files (wrapped in /**: {ros__parameters:})
scripts/               bag conversion, single/suite benchmark runners, ground-truth eval and summary
test/                  10 gtest groups + end-to-end ROS 2 bag launch_test + benchmark-script pytest
docs/                  CPU_BENCHMARK_PROTOCOL.md, RESULTS*.md, superpowers/{specs,plans}/, figures/
```

---

## Future work

> [!NOTE]
> **Stable scope:** the default is the ground‑truth‑regressed CPU‑only Batch‑LIO path, with the
> original Point‑LIO control preserved. The optional representative CPU map changes map semantics
> and must be enabled explicitly. GPU/Jetson is not the default direction.

| Direction | Status | Tracking |
|---|---|---|
| Small Point‑LIO‑inspired CPU/map optimization | Complete and verified; original path retained | [#5](https://github.com/Functionhx/Batch-LIO/issues/5) |
| x86 CUDA acceleration (optional build) | Numerically verified; no end‑to‑end win, not promoted | [#6](https://github.com/Functionhx/Batch-LIO/issues/6) |
| Robocentric representative CPU map | Verified opt‑in profile; not a FR/FAR‑LIO reproduction | [#7](https://github.com/Functionhx/Batch-LIO/issues/7) |
| Jetson CUDA deployment | Retired by CPU‑only decision; no device performance claim | [#8](https://github.com/Functionhx/Batch-LIO/issues/8) |
| ROS 2 Jazzy and Docker validation | Complete | [#4](https://github.com/Functionhx/Batch-LIO/issues/4) |

[Roadmap #9](https://github.com/Functionhx/Batch-LIO/issues/9) ·
[Issue completion record](docs/ISSUE_COMPLETION.md) ·
[Detailed roadmap](docs/FUTURE_WORK.md) ·
[中文路线](docs/FUTURE_WORK.zh-CN.md)

---

## Legacy: ROS 1 Noetic

The original ROS 1 Noetic (catkin) version is preserved at the git tag `ros1-noetic`. The ROS 2
port is "thin and faithful" — identical algorithm and parameters, only the ROS interface changed
(ament, rclcpp, tf2, livox_ros_driver2). Design docs and implementation plans are in
`docs/superpowers/`.

```bash
git checkout ros1-noetic        # inspect the original ROS 1 version
```

---

## Attribution & license

- Built on HKU‑MARS **[Point‑LIO](https://github.com/hku-mars/Point-LIO)**; please cite Point‑LIO
  and FAST‑LIO. The batch‑update idea reproduced here comes from the USTC undergraduate thesis
  *《高带宽轮式激光惯性里程计》(Point‑LIWO)* by 张昊鹏.
- De‑skew follows the FAST‑LIO / sr_lio motion‑compensation convention; the map is an iVox‑style
  hashed voxel structure.
- **MIT** licensed (see [`LICENSE`](LICENSE)); portions derived from Point‑LIO / LOAM / Livox keep
  their BSD‑3 notices. This repository is a research reproduction.

<div align="center">
<br>
<sub><b>Batch‑LIO</b> · research reproduction · <a href="README.md">中文</a></sub>
</div>
