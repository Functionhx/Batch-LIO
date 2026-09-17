# Point-LIO / Batch-LIO CPU 基准协议

这套协议用于后续每一次 CPU 优化的准入测试。当前正式 N=3 结果见
[`20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md`](../eval/benchmarks/20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md)，
机器可读结果见同目录下的 `cpu_benchmark_summary.json`。

## 对比对象

- **Point-LIO control**：共享 ROS2 可执行文件中的逐点路径，`batch_dt=0`、OpenMP 关闭。
  它和 Batch-LIO 使用完全相同的编译器、ROS2 接口、预处理、地图后端及发布选项，专门隔离
  分批本身的影响。这是主要性能基线。
- **Batch-LIO serial**：`batch_dt=0.2 ms`、deskew 开启、OpenMP 关闭。它是经过 discovery
  握手与真值 N=3 确认的原始地图低 CPU 默认档。
- **Batch-LIO OMP2**：`batch_dt=1 ms`、deskew 开启、OpenMP 2 线程。它是低延迟档，不以最低
  总 CPU 时间为目标。

上游 ROS1 Point-LIO 是另一层兼容性检查，不用于主要速度比值：跨 ROS1/ROS2 进程的中间件、
消息转换及编译差异会污染算法加速比，而且上游版本不能原样覆盖这里的所有雷达类型。

## 固定数据集

| 数据集 | 雷达 | 真值 | 作用 |
|---|---|---|---|
| HILTI 2021 UZH tracking area run 2 | Livox | Vicon/官方 6-DoF | 中等点数、精确室内真值 |
| HILTI 2022 exp14 basement 2 | Hesai Pandar XT-32 | HILTI Oxford/官方 6-DoF | 大旋转外参、deskew 回归测试 |
| TIERS OutdoorRoad cut 1 | Ouster 128 | GNSS pose 转本地 ENU | 高点数、户外 CPU 压力测试 |

HILTI 和 TIERS 的真值精度不同，因此只在**同一序列的 A/B 之间**比较误差变化，不横向比较
不同数据集的绝对 ATE。UrbanNav 暂不进入精度门槛：其 Tokyo bag 是 Velodyne 原始 packet，
且官方未提供 LiDAR 标定与传感器外参；在解码和外参来源解决前只可作为吞吐测试。

## 指标与准入线

精度使用时间关联后的一次 SE(3) 刚体对齐，不允许尺度缩放。

- ATE translation RMSE（m）及 rotation RMSE（deg）；
- 1 s RPE translation RMSE（m）及 rotation RMSE（deg）；
- 匹配姿态数、匹配时长和真值路径长度；
- `frame_total` mean / p95 / p99（ms）；
- 进程 interval CPU mean / p95，其中 100% 表示占满一个逻辑核；
- peak RSS（必须来自内核 `VmHWM`）、帧数、平均参与量测点数、NaN 和播放退出状态。

候选默认档必须满足：ATE 平移退化不超过 5%，1 s RPE 平移退化不超过 10%，RSS 增长不超过
10%，匹配覆盖率至少 90%，无 NaN，完整播放正常退出。低 CPU 档还要求 CPU mean 和帧延迟
都不高于 Point-LIO control。所有正式档还必须有成功的 paused/resume 握手，并且同一数据集
各次运行及 Point-LIO 对照的 IMU 初始化进度序列完全一致。OMP2 仍受精度门槛约束，但其
CPU 增长作为显式代价报告。

## 正式运行规则

1. Release 构建，`original_cpu` 地图后端；关闭 PCD、scan、path 和 odometry bag 录制。
2. bag 以 1.0 倍实时速率播放，节点与播放器使用独立 ROS domain，避免其他 ROS 作业串流。
   离线回放通过 `config/benchmark_playback_qos.yaml` 强制 LiDAR/IMU 使用 reliable，并等待
   publisher ACK；播放器建立 publishers 后还要等待 1 秒 DDS discovery，再发送第一条消息，
   防止 volatile publisher 在匹配前丢失开头的 IMU。实机配置仍默认 best-effort，避免与只
   提供 SensorDataQoS 的驱动不兼容。
3. 每个配置完整运行三次，运行次序轮换以减弱升温和顺序偏差，最终报告三次中位数及范围。
4. 测试期间不并行跑其他 LIO；保存 commit、dirty 状态、配置和可执行文件 SHA-256。
   runner 在每轮前还要求 `/proc/stat` 连续两次显示全机至少 90% CPU idle；默认最多等待
   30 分钟，否则本轮失败，不能在后台重负载下生成正式性能数字。
5. 首轮 `20260809` 只有一次完整运行，是方向性基线；任何“默认配置晋级”必须补齐三次重复。
   Ouster 的 `ring` 字段按消息定义读取为 `uint16`；日志出现字段类型警告或候选帧数不足时
   整次运行判无效，不能把少处理数据算作加速。

完整套件（约 31 分钟数据回放，另加初始化与收尾）：

```bash
python3 scripts/run_cpu_benchmark_suite.py \
  --output-root eval/benchmarks/$(date +%Y%m%d_%H%M%S) \
  --repeats 3
```

只复测低 CPU 档（Point control + serial，约 21 分钟数据回放）：

```bash
python3 scripts/run_cpu_benchmark_suite.py \
  --output-root eval/benchmarks/serial_$(date +%Y%m%d_%H%M%S) \
  --repeats 3 \
  --profile point_lio \
  --profile batch_lio_serial
```

生成汇总：

```bash
python3 scripts/aggregate_cpu_benchmarks.py \
  eval/benchmarks/<run>/cpu_benchmark_index.json \
  --json eval/benchmarks/<run>/cpu_benchmark_summary.json \
  --markdown eval/benchmarks/<run>/CPU_BENCHMARK_SUMMARY.md
```

## Jetson 补充规则

Jetson 上必须额外记录型号、JetPack、`nvpmodel` 模式、在线 CPU 核、EMC/GPU/CPU 频率、温度和
`tegrastats` 功耗；每组运行前保持相同热状态。默认优先验证 serial 档。只有在单帧 deadline
无法满足且功耗/温度预算允许时才启用 OMP2，不能用更高总 CPU 时间换来的低 wall latency
宣称为“省算力”。
