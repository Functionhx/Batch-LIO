# Batch-LIO Issue 收口记录（2026-08-10）

本文把 GitHub #4–#9 的目标、实现、验证证据和最终产品决策固定在一起。这里的“收口”
包括正向实现，也包括经过实测后不晋升的负结论；没有 Jetson 实机数据时不会伪造实机完成。

## 总结

| Issue | 结论 | 可复核证据 |
|---|---|---|
| #4 ROS 2 Jazzy | 完成 | `docker/jazzy.Dockerfile` 在 Ubuntu 24.04 / Jazzy 内构建并执行全部 CPU 测试；固定 Livox 驱动 commit；真实 quick-shack rosbag2 兼容性也已验证 |
| #5 CPU / Small Point-LIO 思路 | 完成 | 原始 CPU 路径保留；动态工作区、固定尺寸信息累加、事件驱动调度、修正且有界的 iVox；三套真值 N=3 报告 |
| #6 x86 CUDA | 技术验证完成，产品方向退役 | 可选 CUDA 构建、4,096 查询数值一致、容量/回退、batch 分流、Compute Sanitizer 0 错误；完整流水线没有稳定端到端收益 |
| #7 FR/FAR-LIO 启发实验 | CPU representative 档完成；未冒充完整复现 | 机器人中心裁剪、严格容量、代表点、自适应/鲁棒开关、长期缓存测试、三套真值回归；稀疏 GICP 因改变量测语义未合入 |
| #8 Jetson | 按用户的纯 CPU 决策退役 | 没有目标设备，所以不作功耗、温度、降频或 TensorRT 并发声明；若将来重新授权并提供设备，应作为新任务开启 |
| #9 总路线 | 已由本记录拆解收口 | #4–#8 均有实现证据或明确的退役理由 |

## CPU 正式验收

固定回归集为 HILTI 2021 UZH run 2、HILTI 2022 exp14 和 TIERS OutdoorRoad Ouster。
正式结果见
[`20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md`](../eval/benchmarks/20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md)。

验收规则同时约束：

- 每个 profile 至少 3 次，按中位数汇总；
- ATE 平移退化不超过 5%，1 秒 RPE 平移退化不超过 10%；
- 峰值 RSS 退化不超过 10%；
- 真值匹配比例至少 90%，候选处理帧数至少为 Point-LIO 对照的 95%；
- rosbag 播放正常退出，日志中没有 NaN。

最后一条帧数规则很重要：初次快速回归发现 ROS `SensorDataQoS` 的默认深度 5 会让慢路径
丢样本。LiDAR/IMU 历史已经拆为有界的 32/512 深度；离线回放显式使用 reliable QoS，
播放器以 paused 状态启动，等 LiDAR/IMU 两个 endpoint 都匹配并稳定后才 resume。runner
还要求每轮前全机连续两次至少 90% idle；RSS 使用内核 `VmHWM`，而非可能漏掉瞬时峰值的
轮询样本。修复后三套数据各路径帧数一致，不再把“少做工作”或后台争用计为加速。

保持原始地图语义时使用 `batch_lio_serial`（0.2 ms）。初次 0.5 ms 选择被 rosbag2
volatile publisher 在 DDS discovery 完成前丢失 3–4 个开头 IMU 的问题污染；正式协议改为
paused player + LiDAR/IMU endpoint 握手 + resume 后，Point-LIO 与 0.2 ms 各三次轨迹 SHA
完全一致。正式 N=3 中位数如下；RSS 均为 `VmHWM`，正值表示增加：

| 数据集 | serial 延迟 | serial CPU | serial RSS | serial ATE | serial RPE | 帧数 |
|---|---:|---:|---:|---:|---:|---:|
| HILTI 2021 | -12.16% | -11.00% | -0.23% | -6.96% | -9.05% | 889 / 889 |
| HILTI 2022 | -5.54% | -3.11% | +1.41% | -1.48% | -3.62% | 737 / 737 |
| TIERS Ouster | -9.44% | -4.77% | +1.81% | +0.64% | -0.38% | 443 / 443 |

三套数据上 serial 都通过全部门槛；每个 profile 的三次轨迹 SHA 完全一致，播放退出码为 0，
NaN 和序列化警告均为 0。0.5/0.4/0.3/0.25/0.15/0.05 ms 的完整输入结果未通过 ATE
门槛，因此没有被包装成成功结果。`batch_lio_representative` 是显式可选的低 CPU
档：地图有严格容量和机器人中心裁剪，但语义与原始 iVox 不同。预展开邻域缓存在 TIERS
快速消融中使 RSS 超过门槛，推荐档因此关闭该缓存；直接邻域查询仍保留显著收益。其三套
数据的延迟分别下降 72.36%、71.75%、79.60%，CPU 分别下降 63.56%、46.15%、68.01%，
且 ATE/RPE/RSS 门槛全部通过。

## 正确性与资源安全

- Humble CPU Release：12 个 CTest 目标、52 项测试、0 失败、0 跳过，包含真实 ROS 2 bag
  里程计测试；smoke 使用专用 ROS domain，避免与宿主机其他 DDS participant 串扰；
- Jazzy CPU 镜像 `batch-lio:jazzy-final-v3-20260810`：Ubuntu 24.04 / ROS 2 Jazzy 内
  12/12 CTest 目标通过；挂载真实 quick-shack 后 52 项底层测试 0 失败、0 跳过，Livox
  rosbag2 正常产生里程计并干净退出（`test-result --all` 连同 12 条 wrapper 共 64 项）；
- ASan + UBSan：10 个 C++ 测试全部通过；
- CUDA 原型验证快照：12 个 CTest 目标、59 项测试、0 失败；Compute Sanitizer 0 错误；
- representative iVox：1,000 步机器人中心裁剪/重建测试，检查容量有界和无悬空缓存；
- profiler：最多保留 65,536 个确定性 reservoir 样本，长期运行不再无限增长；
- Path、publisher history、原始/representative iVox 容量均有界；PCD 保存默认关闭；
- 参数加载在启动时检查数组长度、旋转矩阵、有限值、正值、枚举、容量和互斥模式；
- 空点云、非法 ring/time、非有限 IMU、退化平面、时间回退与空 IMU 均有显式处理。

## CUDA 与 Jetson 的边界

RTX 4070 Ti SUPER / CUDA 12.4 的独立查询微基准显示，约 512 点后 CUDA 查询才开始胜过
CPU；典型 Batch-LIO 约 1 ms 小 batch 经常低于该阈值。完整 LIO 还要承担同步、传输和
CPU 小矩阵求解，因此端到端测试没有形成稳定收益。CUDA 保持 `OFF` 默认，仅作为可复核
原型；纯 CPU 构建没有 CUDA 依赖。

桌面 GPU 结果不能外推到 Jetson。#8 没有满足实机验收条件，正确处理是按已确定的纯 CPU
范围退役，而不是写出不存在的 JetPack、功耗或温度数字。

## GitHub 状态说明

本次工作只修改本地代码、测试、容器和证据文件，没有得到“关闭远端 Issue”的明确授权，
所以不会擅自修改 GitHub Issue 状态。维护者可在审阅本记录和正式 N=3 报告后关闭 #4–#9；
#6/#8 的关闭理由应写为“验证后不晋升/范围退役”，而不是“GPU/Jetson 性能目标已实现”。
