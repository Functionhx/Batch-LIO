# batch-LIO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert HKU-MARS Point-LIO from a point-wise (per-timestamp) EKF update into a **per-~1 ms batch** update — group points into 1 ms windows, motion-compensate (de-skew) each point to the window's reference time using the state's ω/v (paper eq 3.44–3.47), row-stack the residuals of all valid points, and run a single EKF update per window. Reproduce Point-LIWO innovation #1 (batch-LIO), CPU-only, A/B against pristine Point-LIO.

**Architecture:** Point-LIO already (a) groups points via `time_compressing` → `time_seq` and (b) builds a row-stacked measurement Jacobian over an entire group for **one** EKF update per group (`h_model_output`/`h_model_input` assemble `effct_feat_num` rows). batch-LIO therefore needs only two algorithmic changes layered on a copy of Point-LIO: **(1)** change the grouping granularity from "identical timestamp" to "fixed 1 ms time window"; **(2)** because a 1 ms window now spans multiple distinct point timestamps, de-skew each point to the window's last-point time (3.44–3.47) before KNN/plane matching. A separate, orthogonal change adds **OpenMP** to the per-point KNN+plane-fit loop, whose speedup grows with group size (i.e. it pays off only once batching enlarges groups).

**Tech Stack:** C++14, ROS1 Noetic, Eigen, PCL 1.10, glog, OpenMP, IKFoM (manifold EKF, header-only, vendored in Point-LIO), iVox map. Build via `catkin_make` in `~/vllm/batch-lio/catkin_ws`, run headless in the Docker container `batch-lio` via `run/run_lio.sh`.

## Global Constraints

- **CPU-only.** No `--gpus`. No wheel-speed (innovation #2) — LiDAR+IMU only.
- **Keep `Point-LIO/` pristine.** All edits go in `Batch-LIO/` (new catkin package `batch_lio`, node `batchlio_mapping`). A/B = two binaries, same bag, same params.
- **State model:** `use_imu_as_input=0` (the `kf_output` branch). Paper eq 3.17 state = `[G_R_I, G_P_I, G_v_I, b_g, b_a, g, ω, a]`, dim 24 — identical to Point-LIO's output model. `ω` = state `kf_output.x_.omg`, `v` = `kf_output.x_.vel`. (The `kf_input` branch is out of scope; leave it compiling but unmodified.)
- **Batch window:** default `batch_dt = 0.001` s (1 ms), exposed as a ROS param so it can be swept and so window→0 reduces to ≈ Point-LIO (a differential correctness check).
- **Run env:** all builds/runs inside container `batch-lio`; `source /opt/ros/noetic/setup.bash && source /root/batch-lio/catkin_ws/devel/setup.bash` in every `docker exec`.
- **Primary eval bag:** `bags/HKU_MB_2020-09-20-13-34-51.bag`. Dev/smoke bag: `bags/2020-09-16-quick-shack.bag` (50 s).
- **Baseline reference numbers (Point-LIO, quick-shack, single-thread):** ~7.69 ms/frame total, ICP ~6.21 ms, propag ~0.17 ms, 487 odom msgs, no divergence.

## Open Question (resolve in Task 4, flag to user)

Paper eq 3.46 writes `Tⱼ = v·Δtⱼ` and calls θ/a/T the IMU's motion **in the world frame**, yet eq 3.47 applies `Rⱼ, Tⱼ` directly to the body-frame point `pⱼ`. A literal world-frame `Tⱼ` is dimensionally inconsistent with a body-frame `pⱼ`. Deriving the exact same-frame transform from "express the point measured at `tⱼ` as if measured at `t_Last`" under constant velocity gives:

```
I_p'_j = Exp(ω·Δt_j) · I_p_j  +  G_R_I(t_Last)^T · v · Δt_j
```

i.e. `Rⱼ = Exp(ω·Δtⱼ)` (body-frame, ω is `I_ω`) and `Tⱼ = R_I^T · v · Δtⱼ` (world velocity rotated into body). We implement the **derived** form (which matches FAST-LIO `UndistortPcl` and sr_lio `distortFrameByImu`), treating the paper's `v·Δtⱼ` as shorthand. Task 4 validates by the window→0 differential test; if tracking degrades we revisit the frame.

---

## File Structure

```
~/vllm/batch-lio/
├─ Batch-LIO/                      # NEW: copy of Point-LIO, package renamed batch_lio
│  ├─ package.xml                  # <name>batch_lio</name>
│  ├─ CMakeLists.txt               # project(batch_lio); executable batchlio_mapping; OpenMP REQUIRED
│  ├─ config/avia.yaml             # + batch_dt, batch_omp params
│  ├─ launch/                      # (unused at runtime; we drive via run/ launch files)
│  ├─ include/
│  │  ├─ common_lib.h              # + time_compressing_batch()  (new fn; old one kept)
│  │  └─ ... (ivox, IKFoM, etc. unchanged)
│  ├─ src/
│  │  ├─ laserMapping.cpp          # main loop: batch grouping + per-window de-skew call
│  │  ├─ Estimator.cpp             # h_model_output: OpenMP on pass-1; (reads de-skewed buffers)
│  │  ├─ Estimator.h
│  │  ├─ deskew.h                  # NEW: deskew_point() (3.44–3.47), header-only, unit-testable
│  │  └─ ...
│  └─ test/
│     └─ test_deskew.cpp           # NEW: standalone unit test for deskew_point()
├─ catkin_ws/src/Batch-LIO -> ../../Batch-LIO   # NEW symlink
├─ run/
│  ├─ avia_baseline.launch         # exists (point_lio)
│  ├─ avia_batch.launch            # NEW (batch_lio, batch_dt + batch_omp params, rviz off)
│  ├─ run_lio.sh                   # exists
│  └─ out/                         # run outputs
└─ docs/plans/2026-06-17-batch-lio-implementation.md   # this file
```

---

## Task 0: Scaffold `batch_lio` package as a verified clone of Point-LIO

**Files:**
- Create: `Batch-LIO/**` (copy of `Point-LIO/**`)
- Modify: `Batch-LIO/package.xml` (name), `Batch-LIO/CMakeLists.txt` (project + exe name)
- Create: `catkin_ws/src/Batch-LIO` (symlink → `../../Batch-LIO`)
- Create: `run/avia_batch.launch`

**Interfaces:**
- Produces: catkin package `batch_lio`, node executable `batchlio_mapping`, launch `run/avia_batch.launch`. Identical algorithm to Point-LIO at this stage.

- [ ] **Step 1: Copy the tree (in container, preserves nothing container-specific).**
```bash
docker exec batch-lio bash -lc '
cp -a /root/batch-lio/Point-LIO/. /root/batch-lio/Batch-LIO/
rm -rf /root/batch-lio/Batch-LIO/Log/*.txt
ls /root/batch-lio/Batch-LIO/src'
```
Expected: `Estimator.cpp Estimator.h IMU_Processing.cpp ... laserMapping.cpp ...`

- [ ] **Step 2: Rename the package.** Edit `Batch-LIO/package.xml`: `<name>point_lio</name>` → `<name>batch_lio</name>`.

- [ ] **Step 3: Rename project + executable in `Batch-LIO/CMakeLists.txt`.**
  - `project(point_lio)` → `project(batch_lio)`
  - `add_executable(pointlio_mapping ...)` → `add_executable(batchlio_mapping ...)`
  - update the matching `target_link_libraries(pointlio_mapping ...)` → `batchlio_mapping`
  - change `find_package(OpenMP QUIET)` → `find_package(OpenMP REQUIRED)` (we rely on it).

- [ ] **Step 4: Add the symlink and a batch launch file.**
```bash
ln -sfn ../../Batch-LIO /home/as/vllm/batch-lio/catkin_ws/src/Batch-LIO
```
Create `run/avia_batch.launch` — identical to `run/avia_baseline.launch` but `pkg="batch_lio"`, `type="batchlio_mapping"`, `$(find batch_lio)/config/avia.yaml`, plus two new params (defaults make it behave like Point-LIO until later tasks read them):
```xml
<param name="batch_dt"  type="double" value="0.001" />
<param name="batch_omp" type="bool"   value="0" />
```

- [ ] **Step 5: Build.**
```bash
docker exec batch-lio bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/batch-lio/catkin_ws && catkin_make -j8 2>&1 | tail -5'
```
Expected: build succeeds; `devel/lib/batch_lio/batchlio_mapping` exists.

- [ ] **Step 6: Smoke-run the clone (must match baseline).**
```bash
docker exec batch-lio bash -lc '/root/batch-lio/run/run_lio.sh /root/batch-lio/run/avia_batch.launch /root/batch-lio/bags/2020-09-16-quick-shack.bag /root/batch-lio/run/out/batch_clone_quickshack 1.0 /root/batch-lio/Batch-LIO'
```
Expected: ~487 odom msgs, no NaN, `[ mapping ]` avg total ≈ baseline (~7–8 ms). This proves the clone is behavior-identical before any algorithm change.

- [ ] **Step 7: Commit.** (Batch-LIO has its own git; init if needed.)
```bash
cd /home/as/vllm/batch-lio/Batch-LIO && git init -q 2>/dev/null; git add -A && git commit -q -m "chore: scaffold batch_lio as verified Point-LIO clone

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 1 (Phase A): OpenMP on the per-point KNN + plane-fit loop

**Files:**
- Modify: `Batch-LIO/src/Estimator.cpp` (`h_model_output` pass-1 loop, ~lines 225–280; mirror in `h_model_input` ~119–172 for completeness)
- Modify: `Batch-LIO/src/laserMapping.cpp` (read `batch_omp` param, set OMP thread count)

**Interfaces:**
- Consumes: `batch_omp` (bool), runtime OMP thread count.
- Produces: identical numerical result to Task 0, just faster on large groups. Pass-2 (Jacobian row assembly with sequential index `m`) stays serial.

**Parallelization safety (from code analysis):** in pass-1 each iteration `j` writes only `point_selected_surf[idx+j+1]`, `Nearest_Points[idx+j+1]`, and `normvec->points[j]` — all indexed by `j`, no cross-iteration conflict. `effect_num_k` is a per-group counter → `reduction(+:effect_num_k)`. State reads (`s.rot`, extrinsics) are read-only. No `break`. iVox `GetClosestPoint` is const/read-only and thread-safe for concurrent reads. Pass-2 keeps the running row index `m` and must remain serial (it's cheap).

- [ ] **Step 1: Read `batch_omp` and cap threads (laserMapping.cpp, near other `nh.param` calls).**
```cpp
bool batch_omp = false;
nh.param<bool>("batch_omp", batch_omp, false);
if (batch_omp) omp_set_num_threads(std::min(omp_get_max_threads(), 16));
else           omp_set_num_threads(1);
```
(`#include <omp.h>` if not already present.)

- [ ] **Step 2: Add the pragma to `h_model_output` pass-1 loop.** Immediately before `for (int j = 0; j < time_seq[k]; j++)` (the KNN/plane-fit pass), add:
```cpp
#pragma omp parallel for schedule(dynamic) reduction(+:effect_num_k)
```
Ensure every variable declared *inside* the loop body (`pabcd`, `p_body`, `pd2`, `p_norm`, the local plane temporaries) stays declared inside the loop (so it is implicitly private). Do **not** add the pragma to pass-2.

- [ ] **Step 3: Build.**
```bash
docker exec batch-lio bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/batch-lio/catkin_ws && catkin_make -j8 2>&1 | tail -3'
```
Expected: BUILD OK.

- [ ] **Step 4: A/B-of-A run — OMP off vs on, same bag.**
```bash
docker exec batch-lio bash -lc '
B=/root/batch-lio
$B/run/run_lio.sh $B/run/avia_batch.launch $B/bags/2020-09-16-quick-shack.bag $B/run/out/A_omp_off 1.0 $B/Batch-LIO
'
# then edit avia_batch.launch batch_omp=1 (or pass a second launch) and rerun into out/A_omp_on
```
Expected: identical odom count (~487) and near-identical trajectory; `[ mapping ]` avg-total compared. **Document the result honestly** — on tiny same-timestamp groups OMP may show little/no gain (fork-join overhead); the real win appears after Task 3. This is an expected, informative outcome, not a failure.

- [ ] **Step 5: Commit.**
```bash
cd /home/as/vllm/batch-lio/Batch-LIO && git add -A && git commit -q -m "feat: OpenMP on KNN+plane-fit loop (batch_omp param)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2 (Phase B-1): Unit-test + implement the de-skew transform (3.44–3.47)

**Files:**
- Create: `Batch-LIO/src/deskew.h` (header-only `deskew_point`)
- Create: `Batch-LIO/test/test_deskew.cpp`
- Modify: `Batch-LIO/CMakeLists.txt` (add a non-ROS test executable)

**Interfaces:**
- Produces:
```cpp
// deskew.h
// Compensate a body-frame point sampled at t_j to the window reference time t_Last.
// dt = t_j - t_Last  (<= 0 within a window)
// omg = I_w (body-frame angular velocity, state.omg)
// vel = G_v (world-frame velocity, state.vel)
// R_I = G_R_I at t_Last (state.rot) — used to rotate world velocity into body frame
// Returns I_p' (body-frame, expressed at t_Last).
Eigen::Vector3d deskew_point(const Eigen::Vector3d& p_body, double dt,
                             const Eigen::Vector3d& omg, const Eigen::Vector3d& vel,
                             const Eigen::Matrix3d& R_I);
```

- [ ] **Step 1: Write the failing test (`test/test_deskew.cpp`).**
```cpp
#include "../src/deskew.h"
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace Eigen;

static bool close(const Vector3d&a,const Vector3d&b,double e=1e-9){return (a-b).norm()<e;}

int main(){
  Matrix3d I = Matrix3d::Identity();
  // 1) zero dt -> identity
  assert(close(deskew_point({1,2,3}, 0.0, {0.1,0,0}, {1,0,0}, I), {1,2,3}));
  // 2) zero motion -> identity
  assert(close(deskew_point({1,2,3}, -0.001, {0,0,0}, {0,0,0}, I), {1,2,3}));
  // 3) pure translation, R_I = identity: p' = p + v*dt
  assert(close(deskew_point({0,0,0}, -0.01, {0,0,0}, {2,0,0}, I), {-0.02,0,0}));
  // 4) pure rotation about z by theta = wz*dt
  {
    double wz=1.0, dt=-0.5; double th=wz*dt;
    Vector3d got = deskew_point({1,0,0}, dt, {0,0,wz}, {0,0,0}, I);
    Vector3d exp(std::cos(th), std::sin(th), 0);
    assert(close(got, exp, 1e-9));
  }
  std::printf("ALL DESKEW TESTS PASSED\n");
  return 0;
}
```

- [ ] **Step 2: Add test target to CMakeLists.txt and verify it fails to build (no deskew.h yet).**
```cmake
add_executable(test_deskew test/test_deskew.cpp)
target_link_libraries(test_deskew ${EIGEN3_LIBRARIES})
```
```bash
docker exec batch-lio bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/batch-lio/catkin_ws && catkin_make -j8 2>&1 | grep -E "deskew.h|error" | head'
```
Expected: FAIL — `deskew.h: No such file or directory`.

- [ ] **Step 3: Implement `deskew.h` (derived same-frame form; see Open Question).**
```cpp
#pragma once
#include <Eigen/Dense>
#include <cmath>

inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w){
  double th = w.norm();
  Eigen::Matrix3d K; K << 0,-w.z(),w.y(), w.z(),0,-w.x(), -w.y(),w.x(),0;
  if (th < 1e-12) return Eigen::Matrix3d::Identity() + K;       // 1st-order
  Eigen::Matrix3d Khat = K/th;
  return Eigen::Matrix3d::Identity() + std::sin(th)*Khat + (1-std::cos(th))*Khat*Khat;
}

inline Eigen::Vector3d deskew_point(const Eigen::Vector3d& p_body, double dt,
                                    const Eigen::Vector3d& omg, const Eigen::Vector3d& vel,
                                    const Eigen::Matrix3d& R_I){
  Eigen::Matrix3d Rj = so3Exp(omg*dt);            // 3.45-3.46: Exp(w*dt)
  Eigen::Vector3d Tj = R_I.transpose()*vel*dt;    // world v rotated into body (see Open Question)
  return Rj*p_body + Tj;                           // 3.47
}
```

- [ ] **Step 4: Build + run the test to pass.**
```bash
docker exec batch-lio bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/batch-lio/catkin_ws && catkin_make -j8 2>&1 | tail -2 && ./devel/lib/batch_lio/test_deskew'
```
Expected: `ALL DESKEW TESTS PASSED`.

- [ ] **Step 5: Commit.**
```bash
cd /home/as/vllm/batch-lio/Batch-LIO && git add -A && git commit -q -m "feat: deskew_point (3.44-3.47) + unit test

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3 (Phase B-2): Re-group points into 1 ms windows

**Files:**
- Modify: `Batch-LIO/include/common_lib.h` (add `time_compressing_batch`; keep original `time_compressing`)
- Modify: `Batch-LIO/src/laserMapping.cpp` (read `batch_dt`; call the batch grouping)

**Interfaces:**
- Consumes: `feats_down_body` with per-point `.curvature` = ms since scan start (sorted ascending); `batch_dt` (seconds).
- Produces: `time_seq` where `time_seq[k]` = count of points whose curvature falls in window `k` (`floor(curvature_ms / (batch_dt*1000))`). With `batch_dt`→0 it degenerates to one-point-per-distinct-timestamp ≈ Point-LIO grouping.

- [ ] **Step 1: Add `time_compressing_batch` to common_lib.h.**
```cpp
// Group points into fixed time windows of win_ms milliseconds (curvature is ms since scan start).
// time_seq[k] = number of consecutive points in window k. Points assumed sorted by curvature.
template<typename T>
std::vector<int> time_compressing_batch(const PointCloudXYZI::Ptr &pc, double win_ms)
{
  int n = pc->points.size();
  std::vector<int> time_seq;
  if (n == 0) return time_seq;
  if (win_ms <= 0) return time_compressing<T>(pc);     // degenerate -> original behavior
  time_seq.reserve(n);
  int count = 1;
  long cur_win = (long)std::floor(pc->points[0].curvature / win_ms);
  for (int i = 1; i < n; i++){
    long w = (long)std::floor(pc->points[i].curvature / win_ms);
    if (w == cur_win) { count++; }
    else { time_seq.emplace_back(count); count = 1; cur_win = w; }
  }
  time_seq.emplace_back(count);
  return time_seq;
}
```

- [ ] **Step 2: Read `batch_dt` in laserMapping.cpp (near other params).**
```cpp
double batch_dt = 0.001;
nh.param<double>("batch_dt", batch_dt, 0.001);
```

- [ ] **Step 3: Swap the grouping call.** Find `time_seq = time_compressing<int>(feats_down_body);` (laserMapping.cpp ~496) and replace with:
```cpp
time_seq = time_compressing_batch<int>(feats_down_body, batch_dt * 1000.0);  // ms window
```

- [ ] **Step 4: Build.** Expected BUILD OK.

- [ ] **Step 5: Differential correctness — tiny window must ≈ baseline.** Run with `batch_dt=0.00005` (0.05 ms ≈ finer than timestamp granularity → groups ≈ Point-LIO). NOTE: at this step we have **not** yet added de-skew, so a *large* window would mis-track; the tiny-window run isolates the grouping change.
```bash
docker exec batch-lio bash -lc '... run with a launch whose batch_dt=0.00005 into out/B2_tinywin ...'
```
Expected: odom ~487, trajectory ≈ baseline (grouping alone, no skew within tiny windows).

- [ ] **Step 6: Document the un-deskewed 1 ms behavior (expected to degrade).** Run with `batch_dt=0.001` into `out/B2_1ms_noskew`. Expect *worse* tracking / mild drift vs baseline — this is the motivation for Task 4 and confirms windows now span multiple timestamps.

- [ ] **Step 7: Commit.**
```bash
cd /home/as/vllm/batch-lio/Batch-LIO && git add -A && git commit -q -m "feat: 1ms time-window grouping (time_compressing_batch, batch_dt)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4 (Phase B-3): Apply intra-window de-skew before the EKF update

**Files:**
- Modify: `Batch-LIO/src/laserMapping.cpp` (`kf_output` main loop, ~lines 597–735) — after the state is propagated to the window reference time and before `update_iterated_dyn_share_modified()`, de-skew every point in the window and refresh the buffers the Jacobian/world-transform read.
- Modify: `Batch-LIO/src/Estimator.h`/`.cpp` only if buffer access needs it (prefer not).
- Uses: `Batch-LIO/src/deskew.h`.

**Interfaces:**
- Consumes: `deskew_point(...)`; window points `feats_down_body->points[idx+1 .. idx+time_seq[k]]`; reference time = curvature of the window's **last** point; state `kf_output.x_.omg`, `.vel`, `.rot`.
- Produces: `feats_down_body`, `pbody_list`, `crossmat_list` entries for the window overwritten with de-skewed body points, so the existing `h_model_output` (KNN, world-transform, Jacobian) operates on motion-compensated points with **zero changes to Estimator.cpp's math**.

**Why here:** `h_model_output` reads `feats_down_body->points[idx+j+1]` (→ world transform → KNN), `pbody_list[idx+j+1]` and `crossmat_list[idx+j+1]` (→ Jacobian). De-skewing all three in the main loop, right after propagation, keeps the measurement model untouched and matches the paper (one propagation to `t_Last`, then update).

- [ ] **Step 1: Include deskew.h in laserMapping.cpp.** `#include "deskew.h"`.

- [ ] **Step 2: Insert the de-skew block.** In the `kf_output` branch, *after* `kf_output.predict(dt, ...)` advances the state to `time_current` (the window's last-point time) and *before* `if (!kf_output.update_iterated_dyn_share_modified())`:
```cpp
if (batch_dt > 0.0 && time_seq[k] > 1) {
  const int last = idx + time_seq[k];                   // last point of window k
  const double t_last_ms = feats_down_body->points[last].curvature;
  const Eigen::Vector3d omg = kf_output.x_.omg;         // I_w
  const Eigen::Vector3d vel = kf_output.x_.vel;         // G_v
  const Eigen::Matrix3d R_I = kf_output.x_.rot.toRotationMatrix();
  for (int j = 1; j <= time_seq[k]; j++) {
    PointType &pb = feats_down_body->points[idx + j];
    const double dt_j = (pb.curvature - t_last_ms) / 1000.0;   // <= 0
    Eigen::Vector3d p(pb.x, pb.y, pb.z);
    Eigen::Vector3d pd = deskew_point(p, dt_j, omg, vel, R_I);
    pb.x = pd.x(); pb.y = pd.y(); pb.z = pd.z();
    pbody_list[idx + j] = pd;
    crossmat_list[idx + j] << SKEW_SYM_MATRX(pd);
  }
}
```
(Confirm the exact indexing convention against the live loop — Point-LIO uses `idx+j+1` with `j` from 0; the snippet above uses `idx+j` with `j` from 1. Match whichever the file uses so points `idx+1..idx+time_seq[k]` are covered exactly once. Verify `x_.omg`/`x_.vel`/`x_.rot` member names against `Estimator.h`.)

- [ ] **Step 3: Build.** Expected BUILD OK.

- [ ] **Step 4: Smoke-run at 1 ms (the real configuration).**
```bash
docker exec batch-lio bash -lc '... run avia_batch.launch (batch_dt=0.001, batch_omp=0) into out/B3_1ms_deskew ...'
```
Expected: tracks cleanly, no NaN, far fewer EKF updates than baseline (fewer `[ mapping ]` lines per frame is not the metric — odom count per frame may drop since updates are per-window), trajectory close to baseline. **If it drifts/diverges → de-skew sign or frame bug** (revisit Open Question: try `Tj = vel*dt` without `R_I^T`, or flip `dt` sign).

- [ ] **Step 5: Differential check — window→0 reduces to baseline.** Run `batch_dt=0.00005` into `out/B3_tinywin`; trajectory must match baseline within tight tolerance (de-skew is ~identity for sub-timestamp windows). This guards against a de-skew bug that only cancels at 1 ms by luck.

- [ ] **Step 6: Commit.**
```bash
cd /home/as/vllm/batch-lio/Batch-LIO && git add -A && git commit -q -m "feat: intra-window de-skew before batch EKF update (3.44-3.47)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 5: Trajectory-comparison tooling + A/B evaluation matrix

**Files:**
- Create: `run/compare_traj.py` (read two `pos_log.txt`, align by time, report start–end drift, max/mean position diff, per-stage timing from node.log)
- Create: `run/avia_batch_1ms_omp.launch`, `run/avia_batch_1ms.launch` variants (batch_dt/batch_omp combos)

**Interfaces:**
- Consumes: `out/*/pos_log.txt` (cols: t, rot(3), pos(3), …) and `out/*/node.log` (`[ mapping ]` timing lines).
- Produces: a printed comparison table for the four configs.

- [ ] **Step 1: Write `run/compare_traj.py`** — parse pos_log columns `t,roll,pitch,yaw,px,py,pz,...`; compute start→end Euclidean drift (return-to-origin error where the bag loops), and parse the last `[ mapping ]` line of each node.log for `ave total`, `icp`, `propogate`. No placeholders — full parser.

- [ ] **Step 2: Run the A/B matrix on quick-shack (dev) — 4 configs.**
  1. Point-LIO baseline (`out/eval_qs_pointlio`)
  2. batch_lio, batch_dt=0.001, omp off (`out/eval_qs_1ms`)
  3. batch_lio, batch_dt=0.001, omp on (`out/eval_qs_1ms_omp`)
  4. batch_lio, batch_dt=0.00005 (≈point-wise) sanity (`out/eval_qs_tiny`)
```bash
# each via run_lio.sh with the matching launch + logsrc (Point-LIO vs Batch-LIO)
```

- [ ] **Step 3: Run the same matrix on HKU_MB (primary eval bag).** Longer; expect the batch+OMP config to show the clearest compute-time reduction and ~maintained accuracy. Also `rostopic hz /aft_mapped_to_init` during a run to check output-rate behavior, and (optional) start–end drift since HKU_MB returns near origin.

- [ ] **Step 4: Produce the results table** (efficiency: avg total / icp / propag ms, odom throughput; consistency: start-end drift, mean trajectory diff vs baseline). Save to `docs/results/2026-06-17-batch-lio-ab.md`.

- [ ] **Step 5: Commit eval tooling + results.**

---

## Self-Review

**Spec coverage:**
- Innovation #1 batch update → Tasks 3 (grouping) + 4 (de-skew) + existing Point-LIO row-stacking (3.48/3.58–3.66) → ✅. OpenMP → Task 1 ✅. A/B (Point-LIO vs batch off/on OMP) → Task 5 ✅. Per-stage timing/throughput/drift metrics → Task 5 ✅. Keep Point-LIO pristine → Task 0 (separate package) ✅. CPU-only, no wheel speed → Global Constraints ✅. Read sr_lio + paper first → done (informs Tasks 2,4). Eval bag HKU_MB → Task 5 ✅.
- **Not covered (deliberately, out of scope / optional in user plan):** Mean Map Entropy / CloudCompare wall-thickness, evo ATE/RPE with GT (these bags have no GT), kf_input branch. Flag to user if they want these added.

**Placeholder scan:** de-skew code, time_compressing_batch, OMP pragma, unit test, param reads — all shown in full. `compare_traj.py` is specified as "full parser, no placeholders" to be written in Task 5 Step 1 (not yet inlined — acceptable as it's pure analysis tooling; will be complete when written).

**Type consistency:** `deskew_point` signature identical in Task 2 interface, deskew.h, and Task 4 call site. State members `kf_output.x_.omg/.vel/.rot` flagged for verification against `Estimator.h` in Task 4 Step 2. `time_compressing_batch<int>` matches the original `time_compressing<int>` call convention.

**Risk register:**
1. De-skew frame (Open Question) — mitigated by Task 4 differential test + explicit fallback.
2. OpenMP gain coupled to group size — set expectations in Task 1; real measurement in Task 5.
3. `SKEW_SYM_MATRX` macro availability in laserMapping.cpp scope — verify include; it's used in the existing buffer-init loop so it's in scope.
4. Overwriting `feats_down_body` affects map insertion — intended (de-skewed points are the correct ones to map).
