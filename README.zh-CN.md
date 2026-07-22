<div align="center">

# batch‑LIO

**[Point‑LIO](https://github.com/hku-mars/Point-LIO) 的批量扩展:**
每 ~1 ms 一个 batch 做 EKF 更新,并在 batch 内做运动去畸变 ——
**更高带宽、更低算力的激光‑惯性里程计。**

🌐 [English](README.md) · **中文**

![ROS2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![C++](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=c%2B%2B&logoColor=white)
![Build](https://img.shields.io/badge/colcon%20test-passing-brightgreen)
![License](https://img.shields.io/badge/license-BSD-blue)

</div>

---

batch‑LIO 复现中国科学技术大学本科毕业设计
*《高带宽轮式激光惯性里程计》(Point‑LIWO —— 基于批量的直接点激光‑IMU‑轮速里程计)*
作者张昊鹏的**创新点一**。直接基于港大 Mars Lab 的 **Point‑LIO**,与其保持 A/B 可比
(纯 CPU;轮速 / 创新点二不在范围内)。

## 目录

| | 章节 |
|---|---|
| 1 | [相比 Point‑LIO 的改动](#相比-pointlio-的改动) |
| 2 | [结果](#结果) |
| 3 | [构建与运行(ROS 2 Humble)](#构建与运行ros-2-humble) |
| 4 | [测试](#测试) |
| 5 | [目录结构](#目录结构) |
| 6 | [旧版:ROS 1 Noetic](#旧版ros-1-noetic) |
| 7 | [致谢与许可](#致谢与许可) |

---

## 相比 Point‑LIO 的改动

Point‑LIO 按**逐点**更新 EKF(每个不同点时间戳一次更新),并在同一时间戳的点组上
行堆叠量测雅可比。batch‑LIO 改了两处并加入 OpenMP:

1. **1ms 时间窗分组** —— 把点按固定 ~1ms 窗口分组,而非按相同时间戳
   (`time_compressing_batch`,`include/common_lib.h`)。
2. **batch 内去畸变** —— 因为一个窗口跨越多个时间戳,每个点用 EKF 状态的角速度/线速度
   运动补偿到窗口参考(末点)时刻,然后所有有效残差行堆叠成**每个窗口一次 EKF 更新**
   (`src/deskew.h`,在 `src/laserMapping.cpp` 中调用)。对应毕业设计公式 3.44–3.47:
3. **OpenMP** 加速逐点的 KNN + 平面拟合循环(`src/Estimator.cpp`)。OpenMP 只在 batch
   **放大**点组后才划算 —— 在 Point‑LIO 原本很小的同时间戳点组上反而更慢。

```
Δtⱼ = tⱼ − t_last                 (窗口内 ≤ 0)
Rⱼ  = Exp(ω · Δtⱼ)                (ω = 状态体坐标系角速度)
Tⱼ  = R_Iᵀ · v · Δtⱼ              (v = 状态世界系线速度;R_I = 状态旋转)
p'ⱼ = Rⱼ · pⱼ + Tⱼ
```

所有改动都由 ROS 参数控制(`batch_dt ≤ 0` 时退化为 Point‑LIO)。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `batch_dt` | `0.001` | batch 窗口长度(**秒**)(`≤ 0` ⇒ 逐点 = Point‑LIO) |
| `batch_omp` | `false` | KNN+平面拟合循环的 OpenMP |
| `batch_deskew` | `true` | 窗口内去畸变开关(消融用) |

---

## 结果

与原版 Point‑LIO 的 A/B,相同 bag 与参数,32 核 x86_64 纯 CPU。
完整数据见 [`docs/RESULTS.md`](docs/RESULTS.md)(ROS 1)与
[`docs/RESULTS_ROS2.md`](docs/RESULTS_ROS2.md)(ROS 2 移植复测)。

### 每帧算力:快 2.3–4.7×

![speedup](docs/figures/fig1_speedup.png)

| bag | 逐点 (ms) | batch 1ms (ms) | 加速比 |
|-----|-----------|----------------|--------|
| quick‑shack (ROS 2) | 12.42 | 3.51 | **3.5×** |
| outdoor_run (ROS 2) | 2.56 | 0.54 | **4.7×** |

batch‑LIO 轨迹与基线高度吻合,且 `outdoor_run` 闭环误差比基线*更小*(0.020 m 对 0.073 m)。

### OpenMP 只在 batch 后才有效("batch 使并行成为可能")
![omp](docs/figures/fig2_omp_causality.png)

### batch_dt 扫描:1–2ms 为甜点区
![sweep](docs/figures/fig4_batchdt_sweep.png)

### 高输出带宽(每窗口发布)
| 模式 | 稳定里程计频率 |
|------|----------------|
| Point‑LIO 帧率里程计 | ~10 Hz |
| **batch‑LIO 1ms(每窗口发布)** | **~913 Hz** |

---

## 构建与运行(ROS 2 Humble)

原生 ROS 2 Humble(`/opt/ros/humble`)。依赖 `livox_ros_driver2`
(其内置 SDK2 已附带预编译 `liblivox_lidar_sdk_shared.so`,无需单独编译 SDK)。

### 1 · 工作空间与编译

```bash
# livox_ros_driver2 作为同目录包;batch_lio 从本仓库软链
mkdir -p ~/batch_lio_ws/src
ln -sfn /path/to/livox_ros_driver2  ~/batch_lio_ws/src/livox_ros_driver2
ln -sfn /path/to/Batch-LIO         ~/batch_lio_ws/src/batch_lio
source /opt/ros/humble/setup.bash
cd ~/batch_lio_ws && colcon build --symlink-install
```

### 2 · 转换 ROS 1 Livox Avia bag → ROS 2 (mcap)

```bash
python3 -m pip install --user rosbags pyyaml
python3 scripts/convert_bag.py  your_avia.bag  ~/batch_lio_ws/bags/your_avia
# 将 livox_ros_driver/CustomMsg 重命名为 livox_ros_driver2/msg/CustomMsg(线格式逐字节相同)
```

### 3 · 运行

```bash
source ~/batch_lio_ws/install/setup.bash
ros2 launch batch_lio mapping_avia.launch.py            # 默认开 rviz2;rviz:=false 为无界面
ros2 bag play ~/batch_lio_ws/bags/your_avia             # 另开一个终端
ros2 topic echo /aft_mapped_to_init                     # 里程计
```

适用于标准 Livox Avia bag(`/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`,
`/livox/imu` = `sensor_msgs/msg/Imu`),例如港大 Mars Lab / FAST‑LIO 的 Avia 序列。
里程计发布在 `/aft_mapped_to_init`;逐帧各阶段耗时以 `[ mapping ]:` 行打印;
设置 `runtime_pos_log_enable` 时轨迹写入 `Log/pos_log.txt`。

---

## 测试

```bash
cd ~/batch_lio_ws
colcon test --packages-select batch_lio          # deskew 单元测试 (5) + 冒烟 launch_test (1)
colcon test-result --all                         # 8 项测试,0 失败
```

| 测试 | 检查内容 |
|---|---|
| `test_deskew`(gtest) | 公式 3.44–3.47 去畸变变换(纯平移、纯旋转、体坐标系速度) |
| `test_smoke.py`(launch_test) | 启动节点、回放转换后的 bag、断言有里程计输出 |

A/B 测试框架(加速比 + 去畸变消融):
```bash
bash scripts/ablations.sh
python3 scripts/compare_traj.py LABEL run/out/<run>/pos_log.txt run/out/<run>/node.log [baseline]
```

---

## 目录结构

```
src/            改动后的 Point‑LIO 源码
  deskew.h          新增:batch 内去畸变(公式 3.44‑3.47),仅头文件 + 含单元测试
  laserMapping.cpp  batch 分组 + EKF 更新前做逐窗口去畸变 + OMP 线程控制
  Estimator.cpp     KNN + 平面拟合循环的 OpenMP
  parameters.*      batch_dt / batch_omp / batch_deskew 参数(类型宽容加载器)
include/common_lib.h  time_compressing_batch(1ms 窗口)+ ROS 2 时间辅助函数
launch/*.py     ROS 2 python launch(mapping_{avia,horizon,ouster64,velody16}、avia_batch)
config/*.yaml   ROS 2 参数文件(包裹在 /**: {ros__parameters:} 中)
scripts/        convert_bag.py(ROS1→ROS2)、run_lio.sh、ablations.sh、compare_traj.py
test/           test_deskew.cpp(gtest)、test_smoke.py(launch_test)
docs/           RESULTS.md、RESULTS_ROS2.md、superpowers/{specs,plans}/、figures/
```

---

## 旧版:ROS 1 Noetic

原始的 ROS 1 Noetic(catkin)版本保留在 git 标签 `ros1-noetic` 上。ROS 2 移植为"薄而忠实"
的移植 —— 算法与参数完全一致,只改 ROS 接口(ament、rclcpp、tf2、livox_ros_driver2)。
设计文档与实施计划见 `docs/superpowers/`。

```bash
git checkout ros1-noetic        # 查看原始 ROS 1 版本
```

---

## 致谢与许可

- 基于港大 Mars Lab 的 **[Point‑LIO](https://github.com/hku-mars/Point-LIO)**;请引用 Point‑LIO
  与 FAST‑LIO。此处复现的批量更新思想来自中国科学技术大学本科毕业设计
  *《高带宽轮式激光惯性里程计》(Point‑LIWO)*,作者张昊鹏。
- 去畸变遵循 FAST‑LIO / sr_lio 的运动补偿约定;地图采用 iVox 风格的哈希体素结构。
- 许可证沿用上游 Point‑LIO / LOAM / Livox 许可证 —— 见 [`LICENSE`](LICENSE)。本仓库为研究复现。
