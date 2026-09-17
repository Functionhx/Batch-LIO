# Batch-LIO 后续工作

> 总路线 Issue：[#9 — Batch-LIO future work](https://github.com/Functionhx/Batch-LIO/issues/9)
>
> English version: [FUTURE_WORK.md](FUTURE_WORK.md)

## 项目边界

默认分支代表已经验证的纯 CPU **Batch-LIO 基线**：原始地图 CPU 默认使用 0.2 ms 点云
分组（显式启用的 representative 地图固定为 1 ms）、batch 内运动
去畸变、每个窗口一次联合 EKF 更新，以及有界 CPU 地图/工作区。原始 Point-LIO 对照路径
保留；改变地图语义的 representative CPU 档必须显式启用。

| 方向 | 状态 | 跟踪入口 |
|---|---|---|
| ROS 2 Jazzy 与 Docker 验证 | 已完成 | [#4](https://github.com/Functionhx/Batch-LIO/issues/4) |
| Small Point-LIO 思路的 CPU / 地图优化 | 已完成并验证 | [#5](https://github.com/Functionhx/Batch-LIO/issues/5) |
| x86 CUDA 加速 | 验证完成，端到端无收益，退役 | [#6](https://github.com/Functionhx/Batch-LIO/issues/6) |
| FR-LIO / FAR-LIO 启发的 CPU 地图与匹配 | representative 档已验证；其余不晋升 | [#7](https://github.com/Functionhx/Batch-LIO/issues/7) |
| Jetson CUDA 部署 | 按纯 CPU 决策退役；无实机结论 | [#8](https://github.com/Functionhx/Batch-LIO/issues/8) |

## 状态定义

- **已评估**：做过分析或基准测试；负结果也会如实保留。
- **原型**：开发分支上已有实现，但接口和实验结论还不稳定。
- **开发中**：仍在变化，并且至少缺少一项验证门槛。
- **已规划**：范围已经明确，但尚未在目标平台完成实现或测试。
- **稳定**：已经合入 `main`，有文档、测试和可复现实验支撑。
- **退役**：技术结论或范围决策已经明确，不再作为产品方向；代码原型可为研究保留。

## 1. Small Point-LIO 思路的优化

Batch-LIO 与 Small Point-LIO 解决的是不同层次的开销：前者通过 batch 减少滤波更新次数，
后者的地图、内存和数据布局思路可以降低每次更新内部的成本。二者结合时必须保留
Batch-LIO 的联合窗口更新语义，不能为了套用优化而重新拆回逐点更新。

研究范围包括：

- 优化前先做分阶段 profiling；
- 带精确参考路径的有界、缓存友好地图结构；
- 可复用量测工作区与固定尺寸信息量累加；
- 默认关闭的 native CPU、LTO 与 fast-math 实验；
- 每个改动都能独立开关和消融。

这些工作已经加入分阶段有界 profiler、动态量测工作区、修正后的原始 iVox 容量/LRU、
事件驱动 ROS 主循环、分离的有界传感器 QoS，以及真值回归门槛。正式证据见
[`CPU benchmark summary`](../eval/benchmarks/20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md)，
开发蓝图保留在内部开发分支。

## 2. x86 CUDA 加速

目标架构是 CPU / GPU 异构协作：

```text
CPU：ROS 调度 -> IMU 传播 -> batch 控制 -> 小尺寸 EKF 求解
GPU：常驻体素地图 -> 点关联 -> 残差/Jacobian -> HtH/Htz 归约
```

约 1 ms 的窗口通常太小，无法天然覆盖单次 GPU 启动与同步成本。因此原型把 CPU/GPU
分流阈值、kernel 融合、同步次数和固定尺寸结果回传作为核心设计约束。CUDA 必须保持
可选，纯 CPU 构建不能依赖 CUDA 工具链。

原型已经完成 CPU/CUDA 数值一致性、容量/回退、不同 batch 大小分流和 Compute Sanitizer
验证；约 512 点后孤立查询 kernel 才开始有优势。完整 LIO 的同步、传输和小 batch 成本
抵消了收益，因此不晋升为默认或 Jetson 路线。原型与负结果保留在内部开发分支，
便于复核而不是重复投入。

## 3. FR-LIO / FAR-LIO 思路

这两项工作是研究参考，不能直接作为 Batch-LIO 功能标签：

- FR-LIO 启发的是机器人中心体素组织，以及把反复邻域组织成本前移到地图维护阶段；
- FAR-LIO 启发的是 GPU 常驻地图、鲁棒 / 自适应匹配，以及保留 CPU 小尺寸滤波求解。

最终保留的可选档实现了机器人中心地图裁剪、严格容量、代表点密度、自适应阈值、Cauchy
鲁棒权重与可独立开关；长期缓存/裁剪测试和三套真值回归均保留。预展开邻域缓存因 TIERS
内存门槛而在推荐档关闭。稀疏 GICP 会改变当前点到平面的量测语义，无法再做严格
Point-LIO A/B，因此评估后未加入。该档**不是**完整复现 FR-LIO 或 FAR-LIO。

## 4. Jetson 部署

x86 CUDA 结果不能直接外推到 Jetson。本轮已经明确选择纯 CPU 路线，因此没有创建
`Jetson-cuda` 分支，也不会用桌面 GPU 数据冒充 Jetson 结论。

实机评估至少包括持续运行的 P50/P95/P99 延迟、deadline miss、功耗、温度、降频、内存，
以及 LIO 与 TensorRT 感知任务的资源竞争。未来若重新获得真实设备和新的范围授权，应以
新任务重新开启；当前 #8 的正确结论是“退役且无性能声明”，不是伪造完成的实机验收。

## 5. 合入稳定项目的门槛

Future Work 只有满足以下条件才进入 `main`：

1. 明确保留原版 Batch-LIO 对照路径；
2. 有聚焦测试，并能通过构建或运行参数独立开关；
3. 同时报告正确性、轨迹质量和性能；
4. 报告 P50/P95/P99，而不是只有平均值；
5. 记录硬件、软件、数据集、参数与 commit ID；
6. 容量耗尽、错误与回退行为可见且经过测试；
7. 不对没有实际测试的硬件或数据集作结论。

README 负责让访客一眼看到路线；GitHub Issues 负责记录可执行任务、设计决策、实验数据
和完成标准。
