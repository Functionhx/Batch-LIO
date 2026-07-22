# Batch-LIO v2 — Phase 0 baseline profile (quick-shack)

**Bag:** `quick-shack` (ROS2 mcap, converted from the ROS1 bag via `scripts/convert_bag.py`).
**Machine:** 32-core x86_64, native `/opt/ros/humble`, Release build, single-threaded EKF unless noted.
**Branch:** `perf/baseline-profiling`. **Profiling gate:** `profiling_enable:=true` (no-op when off).

Raw `[PROFILE]` lines (final report of each run): `batch_1ms.profile.txt`, `batch_1ms_omp.profile.txt`, `pointwise.profile.txt`.

## Where the time goes (mean ms/frame)

| config | frame_total | iterated_update | preprocess↓+batch | map_insert | publish |
|---|---:|---:|---:|---:|---:|
| point-wise `batch_dt=0`, OMP off | 10.43 | **10.13 (97%)** | 0.26 | 0.018 | 0.018 |
| batch 1 ms, OMP off | 9.96 | **9.67 (97%)** | 0.23 | 0.024 | 0.022 |
| batch 1 ms, OMP on (16 threads) | 3.55 | **3.24 (91%)** | 0.25 | 0.028 | 0.023 |

Per-frame counts (batch 1 ms): ~734 points, ~93 batches, ~560 valid residuals.

## Conclusions (Phase 0)

1. **`iterated_update` is the bottleneck in every config (~91–97% of frame time).** Preprocess,
   map insert, and publish are all <3% combined and are **not** worth optimizing first.
2. **The batch speedup is an OpenMP effect.** Single-threaded, batch (9.96 ms) ≈ point-wise
   (10.43 ms). Batching enlarges the per-window point groups so OpenMP pays off: OMP cuts
   `iterated_update` 9.67 → 3.24 ms (~3×). This reproduces the "batch enables parallelism" finding.
3. **Next profiling step:** split `iterated_update` into `knn_search` / `plane_fit` /
   `measurement_build` / `ekf_linear_solve` (instrument `h_model_output` in `Estimator.cpp` + the
   IKFoM solve). This tells us how much is the KNN+plane-fit (what OMP parallelizes) vs the serial
   EKF linear solve — and whether Phase 1 (iVox/KNN) or Phase 3 (information accumulator) is the
   higher-leverage optimization.

## Notes / caveats

- No ground truth; these are timing-only profiles. Trajectory accuracy is tracked separately via the
  A/B harness (`scripts/compare_traj.py`).
- `profiling_enable` defaults to `false`; when off the profiler is a no-op (acceptance: <2% overhead).
- These numbers are environment-bound (CPU/ROS/build/bag/commit) — see `docs/PLAN_V2.md` §3.4.
