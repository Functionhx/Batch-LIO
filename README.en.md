<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/assets/hero-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/assets/hero-light.svg">
  <img src="docs/assets/hero-light.svg" alt="Batch-LIO: one EKF update per 1 ms window" width="100%">
</picture>

<br>

# Up to 4.7× less compute. Virtually the same trajectory.

**Batch‑LIO** changes how often Point‑LIO updates the filter:<br>
instead of updating point by point, it updates once per **millisecond** batch.

<br>

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)](#-up-and-running-in-three-minutes)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](docker/README.md)
[![colcon test](https://img.shields.io/badge/tests-passing-3fb950?style=for-the-badge)](#-engineered-to-be-trusted)
[![License: MIT](https://img.shields.io/badge/license-MIT-0969da?style=for-the-badge)](LICENSE)

<a href="README.md">中文</a> · <b>English</b>

</div>

<br>

<table>
<tr>
<td align="center" width="33%">
<h1>4.7×</h1>
<b>less compute per frame</b><br>
<sub>at most; 100 Hz high‑dynamic sequence</sub>
</td>
<td align="center" width="33%">
<h1>0.03 %</h1>
<b>deviation from baseline</b><br>
<sub>103 m building traverse, 3.1 cm mean</sub>
</td>
<td align="center" width="33%">
<h1>100 %</h1>
<b>falls back to the original</b><br>
<sub><code>batch_dt = 0</code> is bit‑exact Point‑LIO</sub>
</td>
</tr>
</table>

---

## One idea: change the update granularity

Point‑LIO showed that updating **point by point** lets LiDAR‑inertial odometry keep up with the
most aggressive motion. The price is thousands of tiny filter updates in every frame.

Batch‑LIO starts from a simple question: **how far can a robot move in one millisecond?**

Our answer: not far, so the motion inside that millisecond can be **compensated first** and the
update done **once**. Each point is de‑skewed to the end of its window, all residuals are stacked
into a single EKF update, and with fewer updates and larger batches, multi‑core parallelism finally
pays off.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/assets/pipeline-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/assets/pipeline-light.svg">
  <img src="docs/assets/pipeline-light.svg" alt="Batch-LIO pipeline" width="100%">
</picture>

<table>
<tr>
<td width="33%" valign="top">

#### ⏱ Millisecond windows
Points are grouped by time, not by identical timestamp.
The window length is tunable; a sweep shows 1–2 ms works best.

</td>
<td width="33%" valign="top">

#### 🌀 In‑window de‑skew
Every point is compensated to the window end using the filter's own angular and linear velocity.
The transform is checked twice: by unit tests and by an amplification experiment.

</td>
<td width="33%" valign="top">

#### ⚡ Batching enables parallelism
Multithreading makes point‑wise mode **slower**. After batching, it makes the pipeline
**2× faster**. The speedup comes from the architecture, not from tuning.

</td>
</tr>
</table>

---

## 📈 Performance

<p align="center">
  <img src="docs/figures/fig1_speedup.png" width="48%" alt="Per-frame compute">
  &nbsp;
  <img src="docs/figures/fig2_omp_causality.png" width="48%" alt="Batch enables parallelism">
</p>

| Sequence | Scene | Point‑LIO | **Batch‑LIO** | Gain |
|---|---|---:|---:|:---:|
| **outdoor_run** | 100 Hz high‑dynamic outdoor loop | 2.56 ms | **0.54 ms** | **4.7×** |
| **HKU_MB** | 260 s building traverse, 103 m | 16.21 ms | **4.56 ms** | **3.6×** |
| **quick‑shack** | handheld indoor loop | 12.42 ms | **3.51 ms** | **3.5×** |

<sub>Average compute per frame; lower is better. CPU only, 32‑core x86_64. Full data: [ROS 1](docs/RESULTS.md) · [ROS 2](docs/RESULTS_ROS2.md).</sub>

<details>
<summary><b>Trajectory agreement, window sweep and ablations</b></summary>

<br>

**Trajectory agreement**

| Sequence | Metric | Point‑LIO | Batch‑LIO |
|---|---|---:|---:|
| HKU_MB (103 m) | mean deviation from baseline | — | 0.031 m |
| outdoor_run (loop) | start‑to‑end closure | 0.073 m | **0.020 m**¹ |
| quick‑shack (loop) | start‑to‑end closure | 0.072 m | **0.053 m** |

<sub>¹ ROS 1 result. On the ROS 2 rerun the same sequence gave 0.085 m (0.079 m with de‑skew off), so the closure advantage did not reproduce consistently; see [`RESULTS_ROS2.md`](docs/RESULTS_ROS2.md).</sub>

**Window length**: from 0.5 to 2 ms the trajectory matches the baseline at 1.7–2.8× the speed; from 5 ms up the constant‑velocity assumption breaks down and drift grows noticeably.

<img src="docs/figures/fig4_batchdt_sweep.png" width="60%" alt="batch_dt sweep">

**De‑skew amplification**: with the window stretched to 20 ms, drift is 7.59 m with de‑skew off and 0.51 m with it on.

**Odometry rate**: publishing per window gives about 913 Hz. Point‑LIO publishes at about 10 Hz per frame, or about 6.7 kHz per point.

</details>

---

## 🚀 Up and running in three minutes

```bash
# Build: livox_ros_driver2 and Batch-LIO as sibling packages in one workspace
mkdir -p ~/batch_lio_ws/src && cd ~/batch_lio_ws/src
ln -sfn /path/to/livox_ros_driver2 livox_ros_driver2
ln -sfn /path/to/Batch-LIO         batch_lio
source /opt/ros/humble/setup.bash && cd .. && colcon build --symlink-install

# Run with multithreading on: the configuration used in the performance table
source install/setup.bash
ros2 run batch_lio batchlio_mapping --ros-args \
  --params-file $(ros2 pkg prefix batch_lio)/share/batch_lio/config/avia.yaml \
  -p batch_omp:=true
ros2 bag play <your_avia_bag>          # in a second terminal
```

Odometry is published on `/aft_mapped_to_init`. Supports Livox Avia / Horizon, Ouster‑64 and Velodyne‑16.
ROS 1 bags convert in one step with [`scripts/convert_bag.py`](scripts/convert_bag.py). To launch with RViz: `ros2 launch batch_lio mapping_avia.launch.py`.

**Three parameters are all you need:**

| Parameter | Default | |
|---|---|---|
| `batch_dt` | `0.001` | window length in seconds; `0` gives the original Point‑LIO |
| `batch_omp` | `false` | multithreaded point matching |
| `batch_deskew` | `true` | in‑window motion de‑skew |

---

## 🛡 Engineered to be trusted

- **Switch back to the original at any time**: at `batch_dt = 0` the trajectory is **bit‑exact** with Point‑LIO, so every speedup can be checked against the original;
- **Multithreading doesn't change results**: OpenMP on or off gives bit‑identical trajectories;
- **Tests**: 5 gtests on the de‑skew transform, plus an end‑to‑end test that plays a real bag and checks that odometry comes out;
- **Across ROS versions**: native ROS 2 Humble, a [Docker image](docker/README.md) for Jazzy, and the ROS 1 version archived at the `ros1-noetic` tag;
- **Reproducible**: the A/B, sweep and ablation scripts are all in [`scripts/`](scripts/).

---

## 🗺 What's next

| | |
|:-:|---|
| 🤖 | **Edge deployment**: bring‑up on NVIDIA Jetson hardware, with full latency, power and thermal reports |
| 🎯 | **Ground‑truth evaluation**: ATE / RPE on public datasets with ground truth |
| ⚙️ | **More speed**: adopt Small Point‑LIO ideas on the CPU and explore a GPU‑resident map |

---

<details>
<summary><b>About the numbers</b></summary>

<br>

- The baseline is Point‑LIO on the same bags with the same parameters; on ROS 2 the baseline is `batch_dt = 0` in the same binary.
- These sequences have no ground truth: "deviation" means agreement with the Point‑LIO trajectory, and "closure" means the start‑to‑end distance on loop sequences.
- The test machine is a 32‑core x86_64; other platforms need to be measured again.
- The gain is compute efficiency, not odometry bandwidth: batching reduces the number of updates.
- This project reproduces innovation #1 of Point‑LIWO and does not include wheel odometry.

</details>

## Acknowledgements

Batch‑LIO is built on **[Point‑LIO](https://github.com/hku-mars/Point-LIO)** from HKU MARS Lab.
The batch‑update idea comes from the USTC undergraduate thesis *《高带宽轮式激光惯性里程计》*
(Point‑LIWO) by 张昊鹏. De‑skew follows the FAST‑LIO / sr_lio motion‑compensation convention.
If you use this project, please cite Point‑LIO and FAST‑LIO.

Licensed under [MIT](LICENSE); parts derived from Point‑LIO, LOAM and Livox keep their BSD‑3 notices.

<div align="center">
<br>
<sub>If Batch‑LIO helps you, a ⭐ is always appreciated.</sub>
</div>
