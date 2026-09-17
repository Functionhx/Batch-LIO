# Batch-LIO Future Work

> Tracking issue: [#9 — Batch-LIO future work](https://github.com/Functionhx/Batch-LIO/issues/9)
>
> Chinese version: [FUTURE_WORK.zh-CN.md](FUTURE_WORK.zh-CN.md)

## Project boundary

The default branch documents the verified CPU-only **Batch-LIO baseline**: a 0.2 ms original-map
CPU default (with the opt-in representative map pinned to 1 ms),
in-batch motion de-skew, one joint EKF update per window, and bounded CPU maps/workspaces. The
original Point-LIO control remains available; the representative CPU profile changes map semantics
and must be enabled explicitly.

| Track | Status | Tracking |
|---|---|---|
| ROS 2 Jazzy and Docker validation | Complete | [#4](https://github.com/Functionhx/Batch-LIO/issues/4) |
| Small Point-LIO-inspired CPU/map optimization | Complete and verified | [#5](https://github.com/Functionhx/Batch-LIO/issues/5) |
| x86 CUDA acceleration | Validation complete; no end-to-end win; retired | [#6](https://github.com/Functionhx/Batch-LIO/issues/6) |
| FR-LIO / FAR-LIO-inspired CPU mapping | Representative profile verified; other ideas not promoted | [#7](https://github.com/Functionhx/Batch-LIO/issues/7) |
| Jetson CUDA deployment | Retired by CPU-only decision; no device result | [#8](https://github.com/Functionhx/Batch-LIO/issues/8) |

## Status vocabulary

- **Evaluated**: investigated and benchmarked; a negative result is still a useful result.
- **Prototype**: an implementation exists on a development branch, but its interface and results
  are not stable.
- **Work in progress**: actively changing and still missing one or more validation gates.
- **Planned**: scoped, but not yet implemented or tested on the target platform.
- **Stable**: merged to `main`, documented, tested, and supported by reproducible evidence.
- **Retired**: the technical result or scope decision is settled and it is no longer a product
  direction; a research prototype may remain for reproducibility.

## 1. Small Point-LIO-inspired optimization

Batch-LIO and Small Point-LIO attack different costs. Batching reduces the number of filter
updates; map, allocation, and data-layout work can reduce the cost inside each update. The useful
combination must retain Batch-LIO semantics instead of splitting a window back into point-wise
updates.

The research direction includes:

- stage-level profiling before optimization;
- bounded and cache-friendly map representations with an exact reference path;
- reusable measurement workspaces and fixed-size information accumulation;
- opt-in native CPU, LTO, and fast-math experiments;
- independent ablations for every change.

This work now includes a bounded stage profiler, dynamic measurement workspaces, corrected original
iVox capacity/LRU semantics, an event-driven ROS loop, separate bounded sensor QoS histories, and
ground-truth regression gates. Formal evidence is in the
[`CPU benchmark summary`](../eval/benchmarks/20260810_cpu_final_v3_n3/CPU_BENCHMARK_SUMMARY.md); the
development blueprint is kept in the internal working branches.

## 2. x86 CUDA acceleration

The intended architecture is hybrid:

```text
CPU: ROS scheduling -> IMU propagation -> batch control -> small EKF solve
GPU: resident voxel map -> point association -> residual/Jacobian -> HtH/Htz reduction
```

A roughly 1 ms window is commonly too small to justify an individual GPU launch. The prototype
therefore treats CPU/GPU crossover, kernel fusion, synchronization, and fixed-size result transfer
as first-class design constraints. CUDA stays opt-in, and a CPU-only build must continue to work
without a CUDA toolkit.

The prototype passed CPU/CUDA numerical consistency, capacity/fallback, batch-crossover, and
Compute Sanitizer checks; isolated queries begin to win around 512 points. Synchronization,
transfer, and small-batch costs remove that gain in the complete LIO pipeline, so CUDA is not
promoted as a default or Jetson direction. The prototype and negative result remain in
the internal working branches for reproducibility.

## 3. FR-LIO / FAR-LIO-inspired work

These systems are research references, not drop-in labels for Batch-LIO:

- FR-LIO motivates robocentric voxel organization and moving repeated neighborhood work toward
  map maintenance.
- FAR-LIO motivates GPU-resident mapping, robust/adaptive matching, and keeping a small filter
  solve on the CPU.

The retained opt-in profile implements robocentric pruning, strict capacity, representative-point
density, adaptive thresholds, and Cauchy weighting behind independent switches. Long-run cache and
pruning tests plus three ground-truth regressions are retained. Pre-expanded neighborhoods are off
in the recommended profile because they failed the TIERS memory gate. Sparse GICP was evaluated but
not added because it changes the point-to-plane measurement semantics and prevents strict Point-LIO
A/B comparison. This profile is **not** a full reproduction of FR-LIO or FAR-LIO.

## 4. Jetson deployment

Desktop CUDA results do not predict Jetson behavior. This work selected a CPU-only direction, so no
`Jetson-cuda` branch was created and desktop results are not presented as Jetson evidence.

Required evaluation includes sustained P50/P95/P99 latency, deadline misses, power, temperature,
clock throttling, memory, and LIO-versus-TensorRT contention. A future effort would require real
hardware and new scope; the correct #8 outcome today is “retired with no performance claim,” not a
fabricated device acceptance result.

## 5. Promotion to the stable project

Future work can move toward `main` only when it:

1. preserves an explicit original Batch-LIO control path;
2. has focused tests and independent build/runtime switches;
3. reports correctness and trajectory quality as well as speed;
4. reports P50/P95/P99 rather than only a mean;
5. records hardware, software, dataset, parameters, and commit ID;
6. exposes capacity, failure, and fallback behavior;
7. avoids claims for hardware or datasets that were not tested.

README is the high-visibility index. The linked GitHub issues are the executable roadmap and the
place for design decisions, benchmark evidence, and completion criteria.
