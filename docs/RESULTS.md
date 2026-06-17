# batch-LIO vs Point-LIO — A/B Results

**Date:** 2026-06-17 · **Machine:** 32-core x86_64, CPU-only, Docker `osrf/ros:noetic-desktop-full` · **Data:** HKU-MARS Avia bags (`/livox/lidar` CustomMsg + `/livox/imu`).

**What batch-LIO changes vs Point-LIO** (innovation #1, Point-LIWO §3): group LiDAR points into 1 ms windows instead of per-timestamp; de-skew each point to the window's reference time using the EKF state's ω/v (eq 3.44–3.47); row-stack residuals of all valid points → one EKF update per window. Plus optional OpenMP on the per-point KNN+plane-fit loop. State model identical to Point-LIO (`use_imu_as_input=0`). No wheel-speed (innovation #2 out of scope).

Reproducibility: each config = `run/run_lio.sh <launch> <bag> <out> <rate> <logsrc> "<args>"`. Metrics via `run/compare_traj.py`. Per-frame timing = the node's `[ mapping ]` running averages (wall-clock, rate-independent). Trajectory diff aligned per-frame vs the Point-LIO baseline.

---

## 1. quick-shack (50 s, 491 frames, gentle handheld), playback rate 1.0

| Config | total ms | icp ms | propag ms | start→end drift | traj len | mean \|Δpos\| vs base |
|--------|---------:|-------:|----------:|----------------:|---------:|----------------------:|
| **Point-LIO baseline** (point-wise) | 7.69 | 6.21 | 0.17 | 0.0715 m | 11.84 m | — |
| batch 1 ms, OMP off | 6.72 | 5.89 | 0.12 | 0.0528 m | 11.53 m | 0.0591 m |
| **batch 1 ms, OMP on** | **3.35** | 2.44 | 0.13 | 0.0528 m | 11.53 m | 0.0591 m |
| _OMP on point-wise (Task 1)_ | _10.43_ | — | — | — | — | bit-identical |
| batch 20 ms, deskew OFF | 6.27 | 5.49 | 0.11 | 7.59 m ⚠ | 21.6 m | 2.99 m |
| batch 20 ms, deskew ON | 6.96 | 6.17 | 0.11 | 0.51 m | 13.2 m | 0.36 m |

**Takeaways (quick-shack):**
- **Efficiency:** batch 1 ms + OMP is **2.3× faster** than baseline (7.69 → 3.35 ms/frame). Batching *alone* is ~13% faster (fewer EKF updates: ~100 windows/frame vs many tiny per-timestamp groups).
- **OMP only pays off after batching.** On Point-LIO's tiny per-timestamp groups, OMP is *slower* (10.43 vs 7.69 ms — fork/join overhead). At 1 ms the groups are large enough that OMP gives ~2× on top. This is exactly the paper's "batch enables parallelism" claim.
- **Accuracy:** batch 1 ms start→end drift (0.053 m) is *lower* than baseline (0.072 m); trajectory length matches (11.5 vs 11.8 m). The 0.059 m mean per-frame diff vs baseline is just two valid LIO estimates differing (no ground truth on this bag), not degradation.
- **De-skew is correct (decisive probe):** amplifying the window to 20 ms, de-skew OFF diverges (7.59 m drift, 21.6 m garbage trajectory) while de-skew ON stays sane (0.51 m drift) — an ~8× drift reduction. This confirms the transform's sign and frame (`Tⱼ = R_Iᵀ·v·Δtⱼ`, resolving the paper-frame Open Question). At 1 ms de-skew is a near-no-op because 1 ms of gentle motion is sub-mm — expected, not a bug.

---

## 2. HKU_MB main building (260 s, 2602 frames processed, traverse), playback rate 2.0

| Config | total ms | icp ms | propag ms | traj len | mean \|Δpos\| vs base | max \|Δpos\| |
|--------|---------:|-------:|----------:|---------:|----------------------:|-------------:|
| **Point-LIO baseline** | 16.21 | 13.07 | 0.31 | 103.0 m | — | — |
| batch 1 ms, OMP off | 12.26 | 11.03 | 0.12 | 105.9 m | 0.031 m | 0.203 m |
| **batch 1 ms, OMP on** | **4.56** | 3.14 | 0.13 | 105.9 m | 0.031 m | 0.203 m |

All three processed 2602/2602 frames (no drops at rate 2.0) and emitted 2602 odom msgs; no NaN/divergence.

**Takeaways (HKU_MB):**
- **Efficiency:** batch 1 ms + OMP is **3.6× faster** than baseline (16.21 → 4.56 ms/frame) — an even bigger win than quick-shack, because this denser scene puts more points in each 1 ms window, so OMP parallelizes more effectively. Batching alone is ~24% faster (16.21 → 12.26).
- **Accuracy:** batch-LIO matches the baseline trajectory to **mean 0.031 m over a 103 m path (~0.03%)**, max 0.20 m — essentially identical estimation at a fraction of the compute.
- Note: HKU_MB is a *traverse* (ends ~63.6 m from start), not a loop, so "start→end drift" is the true displacement (all configs agree at 63.64 m), not an error metric. With no ground truth, trajectory-consistency-vs-baseline is the accuracy measure here.

**Headline:** batch-LIO reproduces Point-LIWO innovation #1 — **2.3–3.6× lower compute per frame** at **equal-or-better accuracy**, with the speedup growing on denser scenes, exactly as the "batch enables parallelism + fewer updates" thesis predicts.

---

## 3. Validation chain (incremental, each committed)

1. **Clone == baseline** (batch_dt=0): bit-exact trajectory (mean Δpos = 0.000000).
2. **OMP numerically faithful**: OMP on/off give bit-identical trajectories.
3. **De-skew unit-tested**: zero-dt, zero-motion, pure-translation, pure-rotation, world-v-into-body cases all pass.
4. **Grouping degenerate path** reproduces Point-LIO exactly.
5. **De-skew correctness** proven via the 20 ms amplification probe (8× drift reduction).

Git history in `Batch-LIO/`: `ce1841f` scaffold → `51ce9b4` OMP → `d671aeb` deskew+test → `a28c699` 1ms grouping → `859c060` deskew integration.
