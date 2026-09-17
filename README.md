<div align="center">

<!-- ═══════════════════ 资源位 1 / HERO ═══════════════════
     放置 assets/hero.png（建议 1280×360，深浅色通吃或提供双版本）。
     双版本时改用下面的 <picture>，GitHub 会自动跟随读者的明暗主题：

     <picture>
       <source media="(prefers-color-scheme: dark)"  srcset="assets/hero-dark.png">
       <source media="(prefers-color-scheme: light)" srcset="assets/hero-light.png">
       <img src="assets/hero-light.png" alt="Batch-LIO" width="820">
     </picture>
     ═══════════════════════════════════════════════════════ -->

# Batch‑LIO

### 把「逐点更新」压成「逐窗口更新」的激光‑惯性里程计

**[Point‑LIO](https://github.com/hku-mars/Point-LIO) 的批量扩展** —— 点按可配置的亚毫秒时间窗分组，<br>
窗口内做运动去畸变，再行堆叠成**每窗口一次** EKF 更新。

**资源有界 · 每帧算力更低 · 原始 Point‑LIO 对照路径始终保留**

<br>

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](#构建与运行)
[![C++](https://img.shields.io/badge/C%2B%2B-14-00599C?style=flat-square&logo=cplusplus&logoColor=white)](CMakeLists.txt)
[![Tests](https://img.shields.io/badge/colcon_test-52_passed-3fb950?style=flat-square&logo=github-actions&logoColor=white)](#测试)
[![Benchmark](https://img.shields.io/badge/真值基准-N%3D3_全门槛通过-3fb950?style=flat-square)](docs/CPU_BENCHMARK_PROTOCOL.md)
[![License](https://img.shields.io/badge/license-MIT-0969da?style=flat-square)](LICENSE)

**中文** · [English](README.en.md)

<br>

<!-- ═══════════════════ 资源位 2 / DEMO ═══════════════════
     放置 assets/demo.gif（建议 ≤ 8 MB，960px 宽，建图或 RViz 实录）。
     <img src="assets/demo.gif" alt="Batch-LIO 建图实录" width="820">
     ═══════════════════════════════════════════════════════ -->

</div>

---

## 核心结果

相对**同一二进制**内的 Point‑LIO 对照（`batch_dt = 0`），三套带真值数据集、正式 N=3 中位数：

<table>
<tr>
<th align="left" width="26%">档位</th>
<th align="center" width="16%">HILTI&nbsp;2021</th>
<th align="center" width="16%">HILTI&nbsp;2022</th>
<th align="center" width="16%">TIERS</th>
<th align="center" width="26%">地图语义</th>
</tr>
<tr>
<td><b>Batch‑LIO serial</b><br><sub>0.2 ms · 稳定默认档</sub></td>
<td align="center"><b>−12.2%</b><br><sub>CPU −11.0%</sub></td>
<td align="center"><b>−5.5%</b><br><sub>CPU −3.1%</sub></td>
<td align="center"><b>−9.4%</b><br><sub>CPU −4.8%</sub></td>
<td align="center">原始 iVox<br><sub>与 Point‑LIO 严格可比</sub></td>
</tr>
<tr>
<td><b>representative CPU</b><br><sub>1 ms · 需显式启用</sub></td>
<td align="center"><b>−72.4%</b><br><sub>CPU −63.6%</sub></td>
<td align="center"><b>−71.7%</b><br><sub>CPU −46.1%</sub></td>
<td align="center"><b>−79.6%</b><br><sub>CPU −68.0%</sub></td>
<td align="center">⚠️ 语义不同<br><sub>不可用于严格 A/B</sub></td>
</tr>
</table>

<sub>数字为**每帧延迟**降幅。两档在三套数据上均通过 ATE、RPE、内核 HWM RSS、帧数（≥ 对照 95%）、
确定性输入与进程健康的**全部**门槛；每个「数据集 × 档位」的三次运行轨迹 SHA 完全一致。
协议见 [`docs/CPU_BENCHMARK_PROTOCOL.md`](docs/CPU_BENCHMARK_PROTOCOL.md)，
原始汇总见 [`eval/benchmarks/20260810_cpu_final_v3_n3/`](eval/benchmarks/20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md)。</sub>

> [!IMPORTANT]
> **收益在算力，不在带宽。** 批量**减少**了 EKF 更新次数（窗口数 ≪ 点数），
> 所以原始最大发布频率是**下降**而非上升。任何「带宽提升 ×N」的说法都不成立。

---

## 目录

[核心结果](#核心结果) ·
[项目介绍](#项目介绍) ·
[相比 Point‑LIO 的改动](#相比-pointlio-的改动) ·
[完整结果](#完整结果) ·
[构建与运行](#构建与运行) ·
[测试](#测试) ·
[目录结构](#目录结构) ·
[后续工作](#后续工作) ·
[旧版 ROS 1](#旧版ros-1-noetic) ·
[致谢与许可](#致谢与许可)

---

## 项目介绍

Batch‑LIO 复现中国科学技术大学本科毕业设计
*《高带宽轮式激光惯性里程计》（Point‑LIWO —— 基于批量的直接点激光‑IMU‑轮速里程计）*
作者张昊鹏的**创新点一**。直接基于港大 Mars Lab 的 **Point‑LIO**，与其保持 A/B 可比
（纯 CPU；轮速 / 创新点二不在范围内）。

设计约束只有一条：**原始 Point‑LIO 路径必须逐位保留**。设 `batch_dt ≤ 0`，代码走回
完全相同的逐点分支 —— 这既是消融对照，也是所有性能声明的分母。

---

## 相比 Point‑LIO 的改动

<!-- ═══════════════════ 资源位 3 / PIPELINE ═══════════════════
     放置 assets/pipeline.svg —— 数据流示意：
       点云 → 时间窗分组 → 窗口内去畸变 → 行堆叠 → 单次 IEKF 更新 → iVox
     SVG 在明暗主题下都清晰；位图请提供 2× 分辨率。
     <img src="assets/pipeline.svg" alt="Batch-LIO 数据流" width="100%">
     ═══════════════════════════════════════════════════════════ -->

Point‑LIO 按**逐点**更新 EKF（每个不同点时间戳一次更新），并在同一时间戳的点组上行堆叠
量测雅可比。Batch‑LIO 改了两处并加入 OpenMP：

**1 · 可配置亚毫秒时间窗分组**
把点按固定时间窗分组，而非按相同时间戳（`time_compressing_batch`，`include/common_lib.h`）。
保持原始地图语义的 CPU 默认档经真值验证后采用 **0.2 ms**；显式启用的 representative 地图档固定为 1 ms。

**2 · batch 内去畸变**
因为一个窗口跨越多个时间戳，每个点用 EKF 状态的角速度 / 线速度运动补偿到窗口参考（末点）时刻，
然后所有有效残差行堆叠成**每个窗口一次 EKF 更新**（`src/deskew.h`，在 `src/laserMapping.cpp` 中调用）。
对应毕业设计公式 3.44–3.47：

```
Δtⱼ = tⱼ − t_last                 （窗口内 ≤ 0）
Rⱼ  = Exp(ω · Δtⱼ)                （ω = 状态体坐标系角速度）
Tⱼ  = R_Iᵀ · v · Δtⱼ              （v = 状态世界系线速度；R_I = 状态旋转）
p'ⱼ = Rⱼ · pⱼ + Tⱼ
```

**3 · OpenMP** 加速逐点的 KNN + 平面拟合循环（`src/Estimator.cpp`）。
OpenMP 只在 batch **放大**点组后才划算 —— 在 Point‑LIO 原本很小的同时间戳点组上反而更慢。

<details>
<summary><b>全部 ROS 参数</b>（所有改动都由参数控制；<code>batch_dt ≤ 0</code> 退化为 Point‑LIO）</summary>

<br>

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `batch_dt` | `0.0002` | 经真值验证的原始地图 CPU 窗口（**秒**）（`≤ 0` ⇒ 逐点 = Point‑LIO） |
| `batch_omp` | `false` | KNN + 平面拟合循环的 OpenMP |
| `batch_omp_threads` | `2` | 启用时的 OpenMP 线程数（`≤ 0` 使用运行时默认值，最多 16） |
| `batch_omp_min_points` | `8` | 小于该点数的量测 batch 保持串行，避免唤醒线程组 |
| `batch_deskew` | `true` | 窗口内去畸变开关（消融用） |
| `common.lidar_qos_depth` | `32` | 有界 LiDAR 积压，避免慢帧期间 DDS 深度 5 导致丢帧 |
| `common.imu_qos_depth` | `512` | 有界高频 IMU 积压 |
| `common.sensor_qos_reliable` | `false` | 实机安全的 best‑effort 默认；离线基准显式启用 reliable |

</details>

---

## 完整结果

完整数据见 [`docs/RESULTS.md`](docs/RESULTS.md)（ROS 1）与
[`docs/RESULTS_ROS2.md`](docs/RESULTS_ROS2.md)（ROS 2 移植复测）。

> [!NOTE]
> 以下 quick‑shack / outdoor_run 属于**历史无真值**结果。其中的「精度」指的是**与 Point‑LIO
> 基线的吻合度**（平均 \|Δpos\|）加上闭环 bag 的**闭环漂移**，而非相对真实轨迹的误差。

### 每帧算力：快 2.3–4.7×

![每帧算力加速比](docs/figures/fig1_speedup.png)

| bag | 逐点 (ms) | batch 1 ms (ms) | 加速比 |
|-----|-----------|----------------|--------|
| quick‑shack (ROS 2) | 12.42 | 3.51 | **3.5×** |
| outdoor_run (ROS 2) | 2.56 | 0.54 | **4.7×** |

Batch‑LIO 轨迹与基线高度吻合，且 `outdoor_run` 闭环误差比基线*更小*（0.020 m 对 0.073 m）。

### OpenMP 只在 batch 之后才有效（「batch 使并行成为可能」）

![OpenMP 因果关系](docs/figures/fig2_omp_causality.png)

### 历史无真值 batch_dt 扫描：1–2 ms 是速度 / 轨迹一致性甜点区

![batch_dt 扫描](docs/figures/fig4_batchdt_sweep.png)

### 里程计频率取决于发布策略

| 方案 | 发布策略 | 稳定里程计频率 |
|------|----------|----------------|
| Point‑LIO | 按帧（默认） | ~10 Hz |
| Point‑LIO | 按点 | ~6.7 kHz（偏噪） |
| Batch‑LIO 1 ms | 按窗口 | ~913 Hz |

Point‑LIO 并非只能 10 Hz —— 它的按点发布可达 ~6.7 kHz。诚实的对比必须写明发布策略。

---

## 构建与运行

原生 ROS 2 Humble（`/opt/ros/humble`）。依赖 `livox_ros_driver2`
（其内置 SDK2 已附带预编译 `liblivox_lidar_sdk_shared.so`，无需单独编译 SDK）。
Jazzy 使用已经构建并执行测试的 [`docker/jazzy.Dockerfile`](docker/jazzy.Dockerfile)；
Humble 与 Jazzy 的完整镜像命令见 [`docker/README.md`](docker/README.md)。

**1 · 工作空间与编译**

```bash
# livox_ros_driver2 作为同目录包；batch_lio 从本仓库软链
mkdir -p ~/batch_lio_ws/src
ln -sfn /path/to/livox_ros_driver2  ~/batch_lio_ws/src/livox_ros_driver2
ln -sfn /path/to/Batch-LIO          ~/batch_lio_ws/src/batch_lio
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws && colcon build --symlink-install
```

**2 · 转换 ROS 1 Livox Avia bag → ROS 2 (mcap)**

```bash
python3 -m pip install --user rosbags pyyaml
python3 scripts/convert_bag.py  your_avia.bag  ~/batch_lio_ws/bags/your_avia
# 将 livox_ros_driver/CustomMsg 重命名为 livox_ros_driver2/msg/CustomMsg（线格式逐字节相同）
```

**3 · 运行**

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 launch batch_lio mapping_avia.launch.py     # 默认开 rviz2；rviz:=false 为无界面
ros2 bag play ~/batch_lio_ws/bags/your_avia      # 另开一个终端
ros2 topic echo /aft_mapped_to_init              # 里程计
```

<details>
<summary><b>可选：纯 CPU representative-map 高效档</b>（地图语义不同）</summary>

<br>

严格对比 Point‑LIO 时**仍使用** `mapping_avia.launch.py`。只有在可以接受不同地图语义时：

```bash
ros2 launch batch_lio mapping_avia_cpu_fast.launch.py rviz:=false
```

</details>

适用于标准 Livox Avia bag（`/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`，
`/livox/imu` = `sensor_msgs/msg/Imu`），例如港大 Mars Lab / FAST‑LIO 的 Avia 序列。
里程计发布在 `/aft_mapped_to_init`；逐帧各阶段耗时以 `[ mapping ]:` 行打印；
设置 `runtime_pos_log_enable` 时轨迹写入 `Log/pos_log.txt`。

---

## 测试

```bash
cd ~/batch_lio_ws
colcon test --packages-select batch_lio
colcon test-result --all --verbose               # 12 个 CTest 目标，52 项，0 失败
```

| 测试 | 检查内容 |
|---|---|
| 10 个 gtest 目标 | 去畸变、联合信息更新、参数 / 预处理、重力、平面拟合、原始 / representative iVox、鲁棒量测和有界 profiler |
| `test_smoke.py`（launch_test） | 启动节点、回放 ROS 2 bag、至少收到 5 条里程计并验证干净退出 |
| `test_benchmark_scripts.py`（pytest） | 验证内核 HWM 记账、旧资源日志兼容和机器可读的 IMU 初始化序列 |

A/B 测试框架（加速比 + 去畸变消融）：

```bash
bash scripts/ablations.sh
python3 scripts/compare_traj.py LABEL run/out/<run>/pos_log.txt run/out/<run>/node.log [baseline]
```

---

## 目录结构

```
src/                   改动后的 Point‑LIO 源码
  deskew.h             新增：batch 内去畸变（公式 3.44‑3.47），仅头文件 + 含单元测试
  laserMapping.cpp     batch 分组 + EKF 更新前做逐窗口去畸变 + OMP 线程控制
  Estimator.cpp        KNN + 平面拟合循环的 OpenMP
  parameters.*         batch_dt / batch_omp / batch_deskew 参数（类型宽容加载器）
include/common_lib.h   可配置 time_compressing_batch 窗口 + ROS 2 时间辅助函数
launch/*.py            ROS 2 python launch（mapping_{avia,horizon,ouster64,velody16}、avia_batch）
config/*.yaml          ROS 2 参数文件（包裹在 /**: {ros__parameters:} 中）
scripts/               bag 转换、单次 / 套件基准运行、真值评估与结果汇总脚本
test/                  10 组 gtest + ROS 2 bag 端到端 launch_test + 基准脚本 pytest
docs/                  CPU_BENCHMARK_PROTOCOL.md、RESULTS*.md、superpowers/{specs,plans}/、figures/
```

---

## 后续工作

> [!NOTE]
> **稳定范围：** 默认是经过真值回归的纯 CPU Batch‑LIO；原始 Point‑LIO 对照路径始终保留。
> 可选 representative CPU 地图具有不同地图语义，必须显式启用。GPU / Jetson 不属于默认方向。

| 方向 | 状态 | 跟踪入口 |
|---|---|---|
| Small Point‑LIO 思路的 CPU / 地图优化 | 已完成并验证；原始路径保留 | [#5](https://github.com/Functionhx/Batch-LIO/issues/5) |
| x86 CUDA 加速（可选构建） | 数值验证完成；端到端无收益，不晋升 | [#6](https://github.com/Functionhx/Batch-LIO/issues/6) |
| 机器人中心 representative CPU 地图 | 已验证的可选高效档；非 FR/FAR‑LIO 复现 | [#7](https://github.com/Functionhx/Batch-LIO/issues/7) |
| Jetson CUDA 部署 | 按纯 CPU 决策退役；无实机性能声明 | [#8](https://github.com/Functionhx/Batch-LIO/issues/8) |
| ROS 2 Jazzy 与 Docker 验证 | 已完成 | [#4](https://github.com/Functionhx/Batch-LIO/issues/4) |

[总路线 #9](https://github.com/Functionhx/Batch-LIO/issues/9) ·
[Issue 收口记录](docs/ISSUE_COMPLETION.md) ·
[中文详细路线](docs/FUTURE_WORK.zh-CN.md) ·
[English roadmap](docs/FUTURE_WORK.md)

---

## 旧版：ROS 1 Noetic

原始的 ROS 1 Noetic（catkin）版本保留在 git 标签 `ros1-noetic` 上。ROS 2 移植为「薄而忠实」
的移植 —— 算法与参数完全一致，只改 ROS 接口（ament、rclcpp、tf2、livox_ros_driver2）。
设计文档与实施计划见 `docs/superpowers/`。

```bash
git checkout ros1-noetic        # 查看原始 ROS 1 版本
```

---

## 致谢与许可

- 基于港大 Mars Lab 的 **[Point‑LIO](https://github.com/hku-mars/Point-LIO)**；请引用 Point‑LIO
  与 FAST‑LIO。此处复现的批量更新思想来自中国科学技术大学本科毕业设计
  *《高带宽轮式激光惯性里程计》（Point‑LIWO）*，作者张昊鹏。
- 去畸变遵循 FAST‑LIO / sr_lio 的运动补偿约定；地图采用 iVox 风格的哈希体素结构。
- 采用 **MIT** 许可（见 [`LICENSE`](LICENSE)）；派生自 Point‑LIO / LOAM / Livox 的部分保留其 BSD‑3 声明。本仓库为研究复现。

<div align="center">
<br>
<sub><b>Batch‑LIO</b> · 研究复现 · <a href="README.en.md">English</a></sub>
</div>
