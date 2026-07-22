# Batch-LIO v2 开发计划

> **目标**：在保留 Batch-LIO 核心设计——“约 1 ms 时间窗分组 + batch 内去畸变 + 每窗口一次联合 EKF 更新”——的前提下，参考 Small Point-LIO 的工程优化思路，降低地图查询、内存分配、量测构建和矩阵求解开销，并通过严格消融区分各项优化的独立贡献。
>
> **项目仓库**：https://github.com/Functionhx/Batch-LIO  
> **参考项目**：https://github.com/Yancey2023/small_point_lio  
> **思想来源**：中国科学技术大学张昊鹏本科毕业设计 Point-LIWO 的批量更新部分  
> **计划版本**：v1.0  
> **建议目标版本**：Batch-LIO v2.0

---

## 1. 背景

当前 Batch-LIO 已经完成：

- 基于 Point-LIO 的约 1 ms 时间窗分组；
- batch 内点云运动补偿；
- 每个 batch 一次联合 EKF 更新；
- KNN 与局部平面拟合的可选 OpenMP 并行；
- ROS 2 Humble 原生移植；
- ROS 1 数据包到 ROS 2 MCAP 的转换脚本；
- Point-LIO 式逐点模式与 Batch 模式的 A/B 测试；
- 去畸变单元测试和 ROS 2 冒烟测试。

现有实验说明，减少大量小规模 EKF 更新能够显著降低每帧计算耗时。但 Batch 化以后，耗时占比可能重新集中到以下模块：

1. iVox 地图查询；
2. KNN 候选点筛选；
3. 平面拟合与残差构建；
4. 动态矩阵分配与行堆叠；
5. batch 内临时容器和缓存的反复申请；
6. OpenMP 调度与线程同步。

Small Point-LIO 的主要优化方向是：

- 重写或优化点云预处理；
- 改进 iVox 哈希和近邻搜索；
- 使用更紧凑的哈希表；
- 减少动态矩阵与动态内存；
- 使用固定尺寸 Eigen 矩阵；
- 优化编译选项与缓存局部性。

两条路线并不冲突：

> **Batch-LIO 减少更新次数；Small Point-LIO 降低每次点处理和每次更新的成本。**

Batch-LIO v2 的重点，是在不破坏联合 batch 更新的前提下，吸收其中真正正交、可验证的工程优化。

---

## 2. 总体目标

### 2.1 核心目标

1. 保留 Batch-LIO 的时间窗口更新机制；
2. 降低 iVox/KNN 查询耗时；
3. 降低量测构建过程中的动态内存分配；
4. 避免构造大尺寸动态 Jacobian；
5. 在相同硬件、数据和参数下建立可重复的性能基准；
6. 保证优化后轨迹相对当前 Batch-LIO 不出现明显数值偏移；
7. 保留所有优化的独立开关，支持完整消融；
8. 为后续实车、嵌入式平台和带真值数据集测试预留接口。

### 2.2 成功标准

Batch-LIO v2 至少满足：

- `batch_dt <= 0` 仍可运行 Point-LIO 式逐点路径；
- 原始地图后端与优化地图后端可切换；
- 原始 stacked Jacobian 求解与 information accumulator 求解可切换；
- OpenMP 可独立开关；
- 所有已有测试继续通过；
- 新增地图查询、信息累加和数值一致性测试；
- 对同一 bag 重复运行时结果稳定；
- 性能数据由脚本自动生成，而不是手工抄录；
- 所有结论都能追溯到配置文件、日志和 commit SHA。

### 2.3 非目标

本阶段暂不包含：

- 不直接复刻 Small Point-LIO 的完整代码；
- 不立即替换 IKFoM 或重写整个 ESKF；
- 不把 batch 拆回逐点 `update_point()`；
- 不默认启用 `-ffast-math`；
- 不宣称在 Jetson、Orin 或真实机器人上获得同样加速；
- 不宣称“精度更高”，除非未来补充可靠真值；
- 不在本阶段实现完整 Point-LIWO 的轮速融合；
- 不在未完成消融前合并多个大改动。

---

## 3. 设计原则

### 3.1 保留 Batch-LIO 的核心语义

一个 batch 中包含多个不同采样时刻的点。处理顺序必须保持：

```text
状态传播到窗口参考时刻
        ↓
窗口内去畸变
        ↓
逐点地图关联与残差构建
        ↓
整个窗口联合量测更新
```

不能为了使用固定尺寸矩阵，把 batch 重新拆成多个逐点 EKF 更新，否则会失去 Batch-LIO 的主要设计。

### 3.2 所有优化必须可关闭

任何新增优化都需要参数或编译选项：

```yaml
map_backend: original
batch_solver: stacked
batch_omp: false
native_optimization: false
fast_math: false
```

这样可以快速退回基线并完成消融。

### 3.3 一次只改变一个主要变量

推荐顺序：

1. 先补 profiling；
2. 再优化 iVox；
3. 再优化内存；
4. 再实现信息量累加；
5. 最后重新评估 OpenMP；
6. 只有 profiling 证明滤波器仍是主要瓶颈时，才考虑重写滤波器。

### 3.4 性能结论必须绑定环境

每组测试必须记录：

- CPU 型号；
- 核心与线程数量；
- 操作系统；
- ROS 版本；
- 编译器及版本；
- 编译类型；
- OpenMP 线程数；
- bag 名称；
- bag 播放倍率；
- 参数文件；
- commit SHA；
- 是否开启可视化、日志和 PCD 发布。

---

## 4. 目标架构

建议将 v2 拆成以下模块：

```text
Batch-LIO
├── frontend
│   ├── point_preprocess
│   ├── timestamp_preserving_voxel_filter
│   └── batch_grouper
├── motion
│   └── intra_batch_deskew
├── map
│   ├── original_ivox
│   ├── optimized_ivox
│   └── nearest_neighbor_query
├── measurement
│   ├── plane_fit
│   ├── residual_builder
│   ├── stacked_workspace
│   └── information_accumulator
├── filter
│   └── existing_point_lio_ikfom
├── profiling
│   ├── stage_timer
│   └── statistics_reporter
└── benchmark
    ├── configs
    ├── runner
    ├── trajectory_compare
    └── report_generator
```

建议先通过接口抽象替换点，而不是大规模重写：

```cpp
class MapBackend {
public:
    virtual ~MapBackend() = default;

    virtual void AddPoints(const PointCloudXYZI& points) = 0;

    virtual int GetClosestPoints(
        const PointType& query,
        PointVector& neighbors,
        int max_neighbors) const = 0;
};
```

实际实现时若虚函数影响热点性能，可使用：

- 模板策略；
- `std::variant`；
- 编译期选择；
- 两套独立节点；
- 仅在外层选择后缓存具体实现指针。

---

## 5. 开发阶段

# Phase 0：冻结基线并补充性能剖析

## 5.1 目标

在修改算法前，建立可信的当前性能基线，回答：

- 目前时间主要花在哪里？
- Batch 后 EKF 求解还占多少？
- KNN、平面拟合、去畸变分别占多少？
- OpenMP 的真实收益来自哪里？
- 不同 bag 的瓶颈是否相同？

## 5.2 建议分支

```text
perf/baseline-profiling
```

## 5.3 需要记录的阶段

至少拆分：

```text
preprocess
batch_group
state_propagation
deskew
knn_search
plane_fit
measurement_build
ekf_linear_solve
map_insert
publish
frame_total
```

同时记录：

- 每帧点数；
- 每帧 batch 数；
- 每个 batch 平均点数；
- 有效残差数；
- KNN 候选点数；
- 地图体素数；
- OpenMP 线程数；
- 最大、均值、P50、P90、P95、P99 耗时。

## 5.4 实现建议

不要在热点路径中频繁打印。使用内存内统计器，每隔固定帧数汇总：

```cpp
struct StageStats {
    double total_ms = 0.0;
    double max_ms = 0.0;
    std::uint64_t count = 0;

    void Add(double ms);
    double Mean() const;
};
```

日志最终输出为机器可读格式，例如：

```text
[PROFILE] frame=1000 stage=knn_search mean_ms=1.842 p95_ms=2.114
```

或者直接写 CSV/JSON。

## 5.5 验收标准

- Profiling 关闭时，对总耗时影响小于 2%；
- 运行脚本能自动生成完整统计；
- `batch_dt=0` 和 `batch_dt=0.001` 都有基线结果；
- 至少在 `quick-shack`、`outdoor_run` 和一个较长序列上测试；
- 基线结果存入 `docs/results/v2_baseline/`。

---

# Phase 1：优化 iVox 与 KNN 查询

## 6.1 目标

参考 Small Point-LIO 的地图后端优化思路，降低：

- 哈希查找成本；
- 候选点收集成本；
- 不必要的距离计算；
- 临时容器分配；
- 缓存未命中。

## 6.2 建议分支

```text
feature/optimized-ivox
```

## 6.3 子任务 A：延迟距离计算

当前若对所有候选点先计算距离，再排序/筛选，可改为：

1. 先收集候选点；
2. 只有候选数超过 `k` 时才计算距离；
3. 使用 `std::nth_element` 或固定容量 top-k；
4. 避免对全部候选点完整排序。

目标伪代码：

```cpp
neighbors.clear();

for (const auto& voxel_key : neighbor_voxels) {
    const auto* voxel = map.Find(voxel_key);
    if (voxel == nullptr) {
        continue;
    }

    voxel->AppendPoints(neighbors);
}

if (neighbors.size() > max_neighbors) {
    SelectNearestK(query, neighbors, max_neighbors);
}
```

## 6.4 子任务 B：更紧凑的哈希表

新增可选依赖：

```text
ankerl::unordered_dense
```

保留原后端：

```yaml
map_backend: original_ivox
# map_backend: dense_ivox
```

需要检查：

- 插入速度；
- 查询速度；
- 内存占用；
- rehash 行为；
- 遍历稳定性；
- 多线程只读安全性；
- 坐标负数与大范围地图。

## 6.5 子任务 C：体素键优化

设计明确范围的 `VoxelKey`：

```cpp
struct VoxelKey {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;

    bool operator==(const VoxelKey&) const = default;
};
```

提供：

- 安全哈希版本；
- 可选打包版本；
- 边界测试；
- 负坐标测试；
- 溢出检测。

不要直接依赖未说明的整数范围假设。

## 6.6 子任务 D：复用查询缓冲

每个线程维护自己的 scratch buffer：

```cpp
struct QueryWorkspace {
    std::vector<PointType> candidates;
    std::vector<float> squared_distances;
};
```

在初始化时 `reserve()`，之后只 `clear()`，避免循环内反复申请。

OpenMP 下使用：

```cpp
std::vector<QueryWorkspace> thread_workspaces;
```

或 `thread_local`，但需要注意 ROS 2 组件重载和生命周期问题。

## 6.7 验收标准

- 原始与优化地图后端输出的邻居数量满足一致性要求；
- 同一查询点的 top-k 距离差异在设定浮点容差内；
- 地图后端有独立单元测试；
- `map_backend` 可运行时切换；
- 单独开启优化地图后端时，不改变 batch/滤波器代码；
- 至少在两个 bag 上降低 `knn_search` 平均耗时；
- 不因哈希优化引入偶发 NaN、地图丢失或查询失败。

---

# Phase 2：减少动态内存和临时对象

## 7.1 目标

减少以下开销：

- 每个 batch 动态 `resize()`；
- `Eigen::MatrixXd` 重复构造；
- `std::vector` 扩容；
- 临时点云复制；
- deskew 前后重复创建点对象；
- OpenMP 循环内共享容器竞争。

## 7.2 建议分支

```text
perf/reusable-workspaces
```

## 7.3 BatchWorkspace

新增可复用工作区：

```cpp
struct BatchWorkspace {
    Eigen::Matrix<double, Eigen::Dynamic, 12, Eigen::RowMajor> H;
    Eigen::VectorXd residual;
    std::vector<PointType> deskewed_points;
    std::vector<std::uint8_t> valid_flags;

    void Reserve(std::size_t max_points);
    void Reset(std::size_t point_count);
};
```

原则：

- 启动后按历史最大 batch 大小增长；
- 常规帧内不释放；
- 只使用 `topRows(valid_count)`；
- 避免先 push 再拷贝；
- 使用 `noalias()` 减少 Eigen 临时对象。

## 7.4 点数据原地或旁路存储

需要评估：

### 方案 A：原地修改副本

优点：

- 逻辑简单；
- 兼容现有接口。

缺点：

- 仍需要 batch 点副本。

### 方案 B：结构化旁路缓存

分别存储：

```text
deskewed_xyz
original_index
timestamp
normal
residual
jacobian_terms
```

优点：

- 更有利于连续内存和 SIMD；
- 可避免大型 PCL 点结构在热点循环中搬运。

缺点：

- 改动较大。

Phase 2 优先使用方案 A；方案 B 作为后续优化。

## 7.5 验收标准

- 与 Phase 1 相同配置下轨迹差异在容差范围内；
- 运行过程中 batch 工作区容量趋于稳定；
- 可通过统计或分配器工具观察到热点路径分配次数下降；
- `measurement_build` 或 `frame_total` 有可测改善；
- 不引入跨线程数据竞争。

---

# Phase 3：固定尺寸信息量累加求解器

## 8.1 目标

不再显式构造：

\[
H \in \mathbb{R}^{m \times n}, \qquad r \in \mathbb{R}^{m}
\]

而是对 batch 内点残差直接累加固定尺寸正规方程/信息量：

\[
A = \sum_j H_j^\top R_j^{-1} H_j
\]

\[
b = \sum_j H_j^\top R_j^{-1} r_j
\]

其中：

\[
A \in \mathbb{R}^{n \times n}, \qquad b \in \mathbb{R}^{n}
\]

## 8.2 关键约束

- 一个 batch 仍然只进行一次联合更新；
- 不能调用逐点 EKF update；
- 每轮迭代必须在同一线性化状态下重新累加；
- 权重、噪声和鲁棒核必须与当前 stacked 路径一致；
- 先做严格数值等价验证，再做性能测试。

## 8.3 建议分支

```text
feature/information-accumulator
```

## 8.4 配置接口

```yaml
batch_solver: stacked
# batch_solver: information_accumulator
```

## 8.5 基础实现

```cpp
using MeasurementJacobian = Eigen::Matrix<double, 1, 12>;
using InformationMatrix = Eigen::Matrix<double, 12, 12>;
using InformationVector = Eigen::Matrix<double, 12, 1>;

InformationMatrix A = InformationMatrix::Zero();
InformationVector b = InformationVector::Zero();

for (const auto& measurement : measurements) {
    const MeasurementJacobian& H = measurement.H;
    const double r = measurement.residual;
    const double w = measurement.inverse_variance;

    A.noalias() += w * H.transpose() * H;
    b.noalias() += w * H.transpose() * r;
}
```

OpenMP 下不要直接让多个线程写同一个 `A` 和 `b`。建议每线程维护局部累加器：

```cpp
struct LocalAccumulator {
    InformationMatrix A = InformationMatrix::Zero();
    InformationVector b = InformationVector::Zero();
    std::size_t valid_count = 0;
};
```

并行结束后串行归并。

## 8.6 数值验证

需要新增测试：

1. 随机生成多个合法 `H_j` 和 `r_j`；
2. 计算 stacked 路径结果；
3. 计算 accumulator 路径结果；
4. 比较：
   - \(H^\top R^{-1}H\)；
   - \(H^\top R^{-1}r\)；
   - 状态增量；
   - 更新后协方差；
5. 测试不同点数：
   - 0；
   - 1；
   - 5；
   - 100；
   - 1000；
6. 测试病态与退化平面；
7. 测试不同浮点累加顺序。

## 8.7 精度与稳定性注意事项

信息量累加会改变浮点求和顺序，特别是在 OpenMP 下。建议：

- 默认使用 `double`；
- 固定线程归并顺序；
- 对角项检查；
- 求解前检查有限值；
- 必要时使用 pairwise summation；
- 不要默认使用 `-ffast-math`；
- 记录条件数或 LDLT 分解状态。

## 8.8 验收标准

- 单线程下 accumulator 与 stacked 的单次更新结果高度一致；
- 在完整 bag 上轨迹差异小于预设阈值；
- 所有旧测试通过；
- 新增 accumulator 单元测试；
- 大 batch 下动态内存使用明显下降；
- `measurement_build + ekf_linear_solve` 总耗时下降；
- 对退化场景有明确失败处理，不静默输出 NaN。

---

# Phase 4：保留时间戳的快速降采样

## 9.1 目标

参考 Small Point-LIO 的快速预处理思路，降低点云过滤和体素降采样开销，同时保留 Batch-LIO 必需的真实采样时间。

## 9.2 关键约束

普通体素质心没有严格的真实采样时刻，因此不适合直接用于 batch 分组和去畸变。

每个体素应保留一个真实输入点及其原始：

- `offset_time`；
- XYZ；
- reflectivity；
- tag；
- line/ring。

## 9.3 候选策略

### 策略 A：距离体素中心最近的真实点

优点：

- 保留真实时间戳；
- 空间代表性较好。

### 策略 B：最早采样点

优点：

- 时间语义明确。

缺点：

- 空间代表性未必最好。

### 策略 C：最后采样点

优点：

- 更靠近 batch 参考时刻，可能减小 deskew 幅度。

缺点：

- 可能引入扫描方向偏置。

默认优先评估策略 A。

## 9.4 配置接口

```yaml
preprocess_backend: original
# preprocess_backend: timestamp_voxel

timestamp_voxel_policy: nearest_center
voxel_leaf_size: 0.5
```

## 9.5 验收标准

- 输出每个点均来自真实输入点；
- 时间戳严格保留；
- 同一输入下输出确定；
- 不破坏 batch 时间排序；
- 点数、耗时和轨迹变化都有记录；
- 该阶段单独消融，不与地图后端混在同一 commit 中。

---

# Phase 5：重新评估 OpenMP

## 10.1 背景

当前 OpenMP 的收益来自 batch 放大点组后，KNN 和平面拟合具有足够工作量。

当地图查询、内存分配和量测累加被优化后：

- 每个点的工作量可能下降；
- 原线程数可能不再最优；
- 并行调度成本占比可能重新上升；
- 多线程信息量累加可能改变数值顺序。

因此必须重新测试，而不是默认沿用当前线程配置。

## 10.2 建议分支

```text
perf/openmp-retune
```

## 10.3 扫描矩阵

```text
batch_dt:
  0.0005
  0.001
  0.002
  0.005

OMP threads:
  1
  2
  4
  8
  16
  32

map_backend:
  original
  optimized

batch_solver:
  stacked
  information_accumulator
```

根据机器核数裁剪线程列表。

## 10.4 记录指标

- 平均帧耗时；
- P95/P99；
- CPU 总利用率；
- 每阶段耗时；
- 线程扩展效率；
- 轨迹重复性；
- 内存；
- 是否掉帧；
- 是否出现调度抖动。

## 10.5 验收标准

- 给出默认线程数选择原则；
- 不把 32 核桌面机最优值硬编码为所有平台默认；
- 允许 `batch_omp_threads: 0` 表示由运行时决定；
- README 中明确 OpenMP 收益依赖硬件和数据密度。

---

# Phase 6：统一消融实验

## 11.1 核心 2×2 实验

至少完成：

| 编号 | 更新方式 | 地图后端 |
|---|---|---|
| A | Point-wise | Original iVox |
| B | Point-wise | Optimized iVox |
| C | Batch 1 ms | Original iVox |
| D | Batch 1 ms | Optimized iVox |

该实验用于分离：

- Batch 的贡献：A → C；
- 地图优化的贡献：A → B；
- 二者叠加的贡献：B → D 或 C → D；
- 交互效应：D 是否等于简单加速比乘积。

## 11.2 求解器实验

在 C、D 上进一步测试：

| 编号 | Batch | 地图 | 求解器 |
|---|---|---|---|
| C1 | 1 ms | Original | Stacked |
| C2 | 1 ms | Original | Accumulator |
| D1 | 1 ms | Optimized | Stacked |
| D2 | 1 ms | Optimized | Accumulator |

## 11.3 OpenMP 实验

对最优单线程配置重新测试多线程，不与早期 Point-LIO 结果混用。

## 11.4 测试数据

当前可先使用公开数据：

- quick-shack；
- outdoor_run；
- HKU_MB 或其他较长公开序列；
- 至少一个点较密集序列；
- 至少一个运动较激进序列。

后续扩展：

- 带真值公开数据集；
- MID360 自采数据；
- RoboMaster 实车；
- Jetson Orin/NX。

## 11.5 精度表述

在没有可靠真值时，只报告：

- 与基线对齐后的平均位置差；
- 最大位置差；
- 轨迹长度；
- 返回起点序列的首尾距离；
- 是否发散；
- 是否出现 NaN；
- 多次运行重复性。

禁止仅凭这些指标写：

- “精度更高”；
- “精度完全持平”；
- “可替代真值评估”。

建议表述：

> 在当前公开 rosbag 和参数下，优化版本与基线轨迹接近，未观察到明显数值退化；该结果不替代带真值的 ATE/RPE 评估。

---

## 12. Benchmark 自动化

### 12.1 目标目录

```text
benchmark/
├── configs/
│   ├── point_original_stacked.yaml
│   ├── point_optimized_stacked.yaml
│   ├── batch_original_stacked.yaml
│   ├── batch_optimized_stacked.yaml
│   └── batch_optimized_accumulator.yaml
├── run_matrix.py
├── parse_profile.py
├── compare_trajectory.py
├── generate_report.py
└── results/
```

### 12.2 每次运行输出

```text
results/<date>/<bag>/<config>/
├── metadata.json
├── params.yaml
├── node.log
├── profile.csv
├── trajectory.txt
├── summary.json
└── plots/
```

`metadata.json` 至少包含：

```json
{
  "commit": "abcdef0",
  "cpu": "AMD Ryzen ...",
  "compiler": "gcc ...",
  "ros": "humble",
  "build_type": "Release",
  "omp_threads": 8,
  "bag": "quick-shack",
  "playback_rate": 1.0
}
```

### 12.3 可重复性

每个关键配置至少运行 3 次，报告：

- 均值；
- 标准差；
- 最小/最大；
- P95/P99；
- 三次轨迹是否一致。

---

## 13. 参数设计

建议新增：

```yaml
/**:
  ros__parameters:
    # Batch
    batch_dt: 0.001
    batch_deskew: true

    # Parallelism
    batch_omp: false
    batch_omp_threads: 0

    # Map
    map_backend: original_ivox
    ivox_hash_backend: std_unordered_map
    ivox_points_per_voxel: 5
    ivox_lazy_distance: false

    # Measurement solver
    batch_solver: stacked

    # Preprocess
    preprocess_backend: original
    timestamp_voxel_policy: nearest_center

    # Profiling
    profiling_enable: false
    profiling_output: ""
    profiling_report_interval: 100
```

参数加载仍需保持 ROS 2 数值类型宽容，避免 YAML 中 `1` 与 `1.0` 导致节点启动失败。

---

## 14. 测试计划

### 14.1 单元测试

新增：

```text
test_voxel_key.cpp
test_dense_ivox.cpp
test_neighbor_equivalence.cpp
test_information_accumulator.cpp
test_timestamp_voxel_filter.cpp
test_workspace_reuse.cpp
```

保留：

```text
test_deskew.cpp
test_smoke.py
```

### 14.2 关键测试内容

#### 地图

- 正负坐标；
- 大坐标；
- 重复插入；
- 空地图；
- 边界体素；
- top-k 一致性；
- 多点同体素；
- 地图删除/滑窗逻辑。

#### 信息量累加

- 0 个量测；
- 1 个量测；
- 多个量测；
- 随机矩阵；
- 退化矩阵；
- 极小/极大权重；
- 单线程与多线程归并；
- 与 stacked 路径一致性。

#### 时间戳降采样

- 时间戳未丢失；
- 输出点来自输入；
- 时间顺序；
- 同体素多时刻点；
- 边界条件。

#### 端到端

- 节点启动；
- 播放短 bag；
- 发布 odometry；
- 不出现 NaN；
- 两种地图后端均可运行；
- 两种求解器均可运行。

---

## 15. 分支与提交策略

建议每个阶段独立分支，避免大爆炸式提交：

```text
perf/baseline-profiling
feature/optimized-ivox
perf/reusable-workspaces
feature/information-accumulator
feature/timestamp-voxel-filter
perf/openmp-retune
docs/v2-benchmark-report
```

推荐提交粒度：

```text
perf: add per-stage runtime profiler
test: add voxel key boundary tests
feat: add optional unordered_dense iVox backend
perf: reuse per-thread KNN workspaces
feat: add fixed-size information accumulator
test: compare stacked and accumulated normal equations
bench: add reproducible A/B matrix runner
docs: document Batch-LIO v2 ablations
```

每个 PR 必须说明：

- 改了什么；
- 为什么改；
- 默认行为是否改变；
- 如何关闭；
- 单元测试；
- 性能变化；
- 轨迹变化；
- 已知风险。

---

## 16. 兼容性与回退

任何阶段合并后必须至少保留：

```text
原 Point-LIO 式逐点路径
当前 Batch-LIO v1 路径
Batch-LIO v2 优化路径
```

建议提供预设配置：

```text
config/presets/point_lio_compatible.yaml
config/presets/batch_lio_v1.yaml
config/presets/batch_lio_v2_safe.yaml
config/presets/batch_lio_v2_fast.yaml
```

其中：

- `v2_safe` 不启用 fast-math；
- `v2_fast` 可启用本机指令集，但明确不保证跨 CPU；
- 默认主配置优先稳定性，不追求最高跑分。

---

## 17. 编译优化

### 17.1 可选项

```cmake
option(BATCH_LIO_NATIVE_OPT "Enable -march=native" OFF)
option(BATCH_LIO_FAST_MATH "Enable aggressive fast-math" OFF)
option(BATCH_LIO_LTO "Enable link-time optimization" OFF)
```

### 17.2 原则

- `-march=native` 不默认开启；
- `-ffast-math` 不默认开启；
- 开启任何激进浮点优化后必须重新跑完整轨迹；
- 发布二进制时不得假设用户 CPU 与编译机器相同；
- Benchmark 报告必须注明编译选项。

---

## 18. 风险与应对

### 风险 1：地图优化改变邻居顺序

影响：

- 平面拟合结果可能有微小变化；
- 浮点差异可能被迭代滤波放大。

应对：

- 比较邻居集合而不仅是顺序；
- 固定距离相同时的 tie-break；
- 记录容差而不是要求 bit-exact；
- 保留原始后端。

### 风险 2：信息量累加改变浮点求和顺序

影响：

- 多线程运行结果出现轻微不确定性；
- 极端退化场景下数值差异放大。

应对：

- 每线程局部累加；
- 固定归并顺序；
- 全部使用 double；
- 检查 LDLT/LLT 状态；
- 必要时使用更稳定的求和方式。

### 风险 3：优化后 OpenMP 反而变慢

原因：

- 单点任务更轻；
- 调度开销占比上升。

应对：

- 重新扫描线程数；
- 对小 batch 自动退回单线程；
- 根据有效点数设置并行阈值。

### 风险 4：过度追求性能破坏可复现性

应对：

- 所有优化可关闭；
- 每阶段独立 commit；
- 自动化基准记录 commit 和配置；
- 不把多个主要优化塞进一个 PR。

### 风险 5：与 Small Point-LIO 的许可证和署名问题

应对：

- 优先参考公开思想后自行实现；
- 若直接复制或改写代码，检查原文件许可证；
- 保留对应版权和许可证声明；
- README 与文章明确致谢 Small Point-LIO；
- 不把他人实现描述为自己的原创算法。

---

## 19. 文档计划

新增或更新：

```text
docs/
├── PLAN_V2.md
├── ARCHITECTURE_V2.md
├── PROFILING.md
├── BENCHMARK_PROTOCOL.md
├── RESULTS_V2.md
└── SMALL_POINT_LIO_COMPARISON.md
```

`RESULTS_V2.md` 必须区分：

- 算法思想来源；
- 工程实现来源；
- 自己新增的工作；
- 公开数据测试；
- 当前不能得出的结论。

---

## 20. 版本里程碑

### v2.0-alpha：可剖析

- 完成 Phase 0；
- 输出统一 profiling；
- 冻结 v1 基线。

### v2.0-beta1：优化地图

- 完成 Phase 1；
- 可切换地图后端；
- 补齐地图测试。

### v2.0-beta2：低分配版本

- 完成 Phase 2；
- 工作区复用；
- 热点分配显著减少。

### v2.0-rc1：信息量累加

- 完成 Phase 3；
- stacked 与 accumulator 双路径；
- 完整数值一致性测试。

### v2.0-rc2：预处理与线程重调

- 完成 Phase 4、Phase 5；
- 给出推荐默认配置。

### v2.0：公开发布

- 完成统一消融；
- 生成结果报告；
- README 更新；
- ROS 2 测试全绿；
- 发布 tag 与 release notes。

---

## 21. 推荐的最小可行版本

如果开发资源有限，优先完成以下四项：

1. **Phase 0：性能剖析**
2. **Phase 1：优化 iVox**
3. **Phase 2：工作区复用**
4. **Phase 3：固定尺寸信息量累加**

即：

\[
\boxed{
\text{1 ms batching}
+
\text{in-batch deskew}
+
\text{optimized iVox}
+
\text{fixed-size information accumulation}
}
\]

这四项能够形成清晰的 Batch-LIO v2 技术主线：

> 在时间维度减少更新次数，在空间维度降低地图查询成本，在滤波计算维度避免动态大矩阵，并通过统一消融评估三者的独立和组合收益。

---

## 22. 发布文章时的推荐表述

建议写：

> Batch-LIO v2 在保留 1 ms 批量更新与批内运动补偿的基础上，参考 Small Point-LIO 对地图查询、内存组织和固定尺寸矩阵的工程优化思路，进一步优化了 iVox 后端和联合量测计算。两条路线分别减少“更新次数”和“单次点处理成本”，本文通过统一硬件、数据和基线进行消融，而不直接比较两个项目各自公布的加速倍数。

不要写：

> Batch-LIO 融合 Small Point-LIO 后比所有 Point-LIO 版本更快。

除非未来完成统一平台上的直接横向测试。

---

## 23. 第一周执行清单

### Day 1

- [ ] 创建 `perf/baseline-profiling`
- [ ] 添加 stage timer
- [ ] 记录每帧点数、batch 数和有效残差数

### Day 2

- [ ] 统一 benchmark 目录
- [ ] 固定 quick-shack、outdoor_run 测试命令
- [ ] 记录 commit、CPU、编译器和参数

### Day 3

- [ ] 跑 `batch_dt=0`
- [ ] 跑 `batch_dt=0.001`
- [ ] 跑 OMP on/off
- [ ] 确认当前主要瓶颈

### Day 4

- [ ] 实现 KNN 延迟距离计算
- [ ] 增加邻居等价单元测试

### Day 5

- [ ] 增加查询 workspace 复用
- [ ] 对比分配次数与 KNN 耗时

### Day 6

- [ ] 接入可选 `unordered_dense`
- [ ] 完成负坐标和大坐标测试

### Day 7

- [ ] 输出第一版 `RESULTS_V2.md`
- [ ] 决定是否进入信息量累加阶段
- [ ] 不满足验收标准的优化不合并

---

## 24. 最终交付物

Batch-LIO v2 完成时应包含：

- 可运行源码；
- 原始与优化地图后端；
- stacked 与 accumulator 求解器；
- 完整参数开关；
- 自动化 benchmark；
- profiling CSV/JSON；
- 单元测试与冒烟测试；
- 统一 A/B 消融；
- 公开数据实验报告；
- 局限说明；
- 许可证和致谢；
- RoboMaster 论坛技术文章；
- GitHub Release 与可复现实验说明。

---

## 25. 最终检查清单

### 功能

- [ ] Point-wise 模式可运行
- [ ] Batch 模式可运行
- [ ] 去畸变可开关
- [ ] 地图后端可切换
- [ ] 求解器可切换
- [ ] OpenMP 可开关
- [ ] ROS 2 bag 可正常播放
- [ ] Odometry 正常发布

### 测试

- [ ] Deskew 测试通过
- [ ] Map backend 测试通过
- [ ] Neighbor equivalence 测试通过
- [ ] Accumulator 测试通过
- [ ] Timestamp voxel 测试通过
- [ ] Smoke test 通过
- [ ] 完整 bag 无 NaN/发散

### 性能

- [ ] Profiling 有完整阶段数据
- [ ] 至少三个 bag
- [ ] 关键配置至少重复三次
- [ ] 报告均值与 P95/P99
- [ ] 记录 CPU/编译器/线程数
- [ ] 不混用不同基线的性能数字

### 文档

- [ ] README 更新
- [ ] PLAN_V2.md 完整
- [ ] RESULTS_V2.md 完整
- [ ] 局限写清楚
- [ ] Point-LIWO 致谢
- [ ] Point-LIO 致谢
- [ ] Small Point-LIO 致谢
- [ ] 许可证检查完成

---

> **一句话收尾**：Batch-LIO v2 不应成为“把 Small Point-LIO 代码搬过来的版本”，而应成为一个可以清楚回答以下问题的实验平台：减少更新次数、优化地图查询、避免动态大矩阵，这三件事分别贡献了多少，它们能否稳定叠加，以及代价是什么。
