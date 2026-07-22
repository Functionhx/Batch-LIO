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
3. **Fine split of `iterated_update`** (below): `measurement_build` (the `h_model_output` call =
   parallel KNN + plane-fit + serial Jacobian assembly) is **~94% of the single-threaded update**;
   the serial remainder (IMU propagation + IKFoM linear solve + pre-fill + `pointBodyToWorld`) is
   only ~0.5 ms (~5%); deskew is negligible (~0.02 ms).

## Fine split of `iterated_update` (mean ms/frame)

| stage | OMP off | OMP on (16 thr) | OMP-sensitive? |
|---|---:|---:|---|
| frame_total | 9.56 | 3.84 | yes |
| iterated_update | 9.25 | 3.51 | yes |
| → `measurement_build` (h_model_output: KNN+plane+Jacobian) | **8.74 (94%)** | **2.94** | **yes (~3×)** |
| → `deskew` | 0.024 | 0.031 | no (negligible) |
| → `serial_remainder` (IMU prop + EKF solve + overhead) | 0.49 (~5%) | 0.54 | **no** |

Raw: `batch_1ms_fine.profile.txt`, `batch_1ms_omp_fine.profile.txt`.

## Reprioritization (informs the plan §3.3 / §21)

- **Phase 1 (optimize iVox/KNN) is the high-leverage target** — `measurement_build` is ~94% of the
  update and the only stage OMP speeds up, so its dominant cost is the per-point KNN
  (`ivox_->GetClosestPoint`) + plane-fit. That is exactly Phase 1's scope.
- **Phase 3 (information accumulator) has a low ceiling here** — it targets the stacked-Jacobian
  solve + dynamic matrix, which live in `serial_remainder` ≈ 0.5 ms (~5%). Replacing them saves at
  most ~0.5 ms/frame on this bag. Worth keeping as an option but **not** the MVP priority the plan's
  §21 implies.
- Caveat: `measurement_build` also contains a *serial* Jacobian-assembly pass after the OMP loop;
  the OMP speedup of the whole stage (~3×) implies that serial pass is a minority of it, but
  separating KNN vs plane-fit vs Jacobian-assembly precisely needs a single-threaded micro-bench
  (per-thread timing under OMP would overcount wall time).

## Notes / caveats

- No ground truth; these are timing-only profiles. Trajectory accuracy is tracked separately via the
  A/B harness (`scripts/compare_traj.py`).
- `profiling_enable` defaults to `false`; when off the profiler is a no-op (acceptance: <2% overhead).
- These numbers are environment-bound (CPU/ROS/build/bag/commit) — see `docs/PLAN_V2.md` §3.4.

## Update: clean baseline + Phase 1 experiments (re-measured after the Jazzy sub-agent finished)

**Methodology fix:** the Phase-0 numbers above were captured while a heavy Docker build (the Jazzy
port sub-agent) ran concurrently, inflating means and especially the tail (OMP-on `max` was 77 ms).
Re-measured on a quiet machine (raw: `clean_off.profile.txt`, `clean_on.profile.txt`):

| stage (batch 1 ms) | OMP off | OMP on (16 thr) |
|---|---:|---:|
| frame_total | 9.92 | 3.55 |
| measurement_build | 9.06 | 2.63 (p50 2.72, max 5.1) |
| serial_remainder | 0.52 | 0.57 |

**Phase 1 experiments (both ruled out as throughput wins on quick-shack):**
- *Reusable thread-local scratch buffer* (sub-task D): ST `measurement_build` 9.06 → ~8.7 (~4 %,
  within noise); OMP p50 flat. Reverted.
- *`unordered_dense` hashmap* (sub-task B): clean A/B **2.63 vs 2.63 — no measurable gain**. The
  apparent earlier improvement was just the Jazzy-load confound disappearing. Reverted.

**Conclusion:** the iVox hashmap lookup and the candidate allocation are **not** the ST bottleneck.
`measurement_build`'s single-thread cost (9.06 ms) is dominated by the per-point KNN distance
computation (`KNNPointByCondition` in `ivox3d_node.hpp`); OMP already delivers the win
(9.06 → 2.63 ms, ~3.4×). Going faster on the ST path would require optimizing
`KNNPointByCondition` itself (candidate pruning / SIMD / a smarter NN structure) — a deeper,
higher-risk change. **Net: with OMP on, Batch-LIO is already ~3.55 ms/frame; the easy Phase-1
levers (hashmap / alloc) add nothing here, and Phase 3 (accumulator) is already ruled out by the
~0.5 ms `serial_remainder`.** This re-orders `PLAN_V2`'s MVP (§21): the remaining candidate lever
is the KNN distance computation, not the data structures around it.

## Small Point-LIO ideas — R&D summary (branch `perf/native-fast-math`)

Tested the transferable Small Point-LIO runtime techniques on top of Batch-LIO's existing
batch + OMP code (quick-shack, clean machine):

| idea | ST `measurement_build` | OMP-on p50 | verdict |
|---|---|---|---|
| `unordered_dense` hashmap (sub-task B) | 9.06 → 9.06 | 2.72 → 2.72 | no gain |
| reusable thread-local scratch (sub-task D) | 9.06 → ~8.7 | flat | marginal (noise) |
| `-march=native -ffast-math` (toolchain, §17) | 9.06 → 8.64 (~5 %) | 2.72 → 2.73 | ~5 % ST, OMP-flat; **trajectory safe** |

The native/fast-math/lto CMake options (`BATCH_LIO_NATIVE_OPT`, `BATCH_LIO_FAST_MATH`,
`BATCH_LIO_LTO`) are added **OFF by default** — available and reproducible. `-ffast-math` does not
diverge the filter (22.4 m trajectory, 0.07 m drift on quick-shack).

**Why the ideas don't stack:** Batch-LIO already OMP-parallelizes the per-point KNN + plane-fit
(which *is* its 3.5× over point-wise) — the same win Small Point-LIO gets a different way (a faster
per-point backend). The residual ST cost is raw per-point KNN distance computation
(`KNNPointByCondition`) that `-O3` + OMP already handle; hashmap / alloc / fast-math barely touch
it. So Small Point-LIO's runtime micro-opts and Batch-LIO's batch+OMP are **alternative routes to
similar speed, not additive**. The only remaining runtime lever is rewriting
`KNNPointByCondition` itself; IKFoM→lean-ESKF (another Small Point-LIO idea) would help compile
time / code clarity, not runtime (the EKF solve is ~0.5 ms).
