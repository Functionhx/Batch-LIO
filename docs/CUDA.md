# CUDA representative-iVox backend (`x86-cuda`)

This branch contains an end-to-end, selectable measurement-map backend. The
original Batch-LIO path remains the default and CUDA remains an opt-in build.

## Implemented backends

- `original_cpu`: the existing Batch-LIO iVox implementation (default).
- `representative_cpu`: a bounded iVox retaining 1--8 real points nearest each
  voxel center, inspired by Small Point-LIO.
- `representative_cuda`: the same deterministic map and top-5 query on CUDA,
  with automatic CPU handling for batches below `cuda_min_batch_points`.

The follow-up `fralio-cuda` branch also provides an exact, FR-LIO-inspired
insertion-time neighborhood index for the representative CPU mirror. See
[`FRALIO_CUDA.md`](FRALIO_CUDA.md) for design, verification, and A/B results.

Representative maps use signed 32-bit voxel coordinates, reject non-finite or
out-of-range points, support 0/6/18/26-neighbor lookup, and return a stable
distance/coordinate ordering. The CUDA implementation uses an open-addressed
device hash table and is checked against the CPU reference.

The original map and a CPU representative mirror stay updated in CUDA mode. If
CUDA initialization, insertion, reset, or query fails, odometry continues on
the CPU representative backend without restarting the estimator.

## x86 scheduling policy

The default CUDA crossover is 512 query points. A normal 1 ms Batch-LIO window
contains only about eight points, so those queries remain on CPU; kernel launch,
copy, and synchronization cost more than the work itself.

For representative backends, the per-point OpenMP loop uses the same granularity
idea: batches below 64 points run serially. This avoids repeatedly waking a
16-thread team for roughly eight cheap lookups. The original iVox path keeps its
existing tiny-batch parallelism because each original lookup is much heavier.

`cuda_persistent_queries` is off by default on x86. Its mapped-memory polling
kernel reduces launch latency for tiny requests, but writing many individual
results through PCIe is slower than a normal device buffer plus bulk copy. It is
kept as an explicit experimental path for Jetson, where CPU and GPU share
physical memory.

Map updates and resets temporarily stop the persistent kernel and relaunch it
afterward. This avoids a `cudaMalloc`/persistent-kernel synchronization deadlock
and creates a clear visibility boundary for updated map data.

## Build

The workstation's `/usr/local/cuda` currently selects CUDA 11.8, which is
incompatible with its GCC 12 host compiler. Use CUDA 12.8 explicitly:

```bash
cd /home/as/batch_lio_ws
colcon build --packages-select batch_lio --symlink-install --cmake-args \
  -DBUILD_TESTING=ON \
  -DBATCH_LIO_ENABLE_CUDA=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc \
  -DBATCH_LIO_CUDA_ARCHITECTURES=89
```

Without `BATCH_LIO_ENABLE_CUDA=ON`, the project links a CPU-only stub and does
not require a CUDA toolkit.

## Runtime parameters

Example hybrid CUDA run:

```bash
ros2 run batch_lio batchlio_mapping --ros-args \
  --params-file /path/to/avia.yaml \
  -p map_backend:=representative_cuda \
  -p mapping.representative_map_resolution:=0.5 \
  -p representative_max_points_per_voxel:=4 \
  -p representative_nearby_type:=18 \
  -p representative_map_capacity:=262144 \
  -p representative_max_range:=5.0 \
  -p representative_preexpand_neighborhoods:=true \
  -p cuda_min_batch_points:=512
```

Parameters:

| parameter | default | meaning |
|---|---:|---|
| `map_backend` | `original_cpu` | `original_cpu`, `representative_cpu`, or `representative_cuda` |
| `mapping.representative_map_resolution` | `0.5` | voxel size in metres |
| `representative_max_points_per_voxel` | `4` | retained real points, range 1--8 |
| `representative_map_capacity` | `262144` | maximum occupied voxels |
| `representative_nearby_type` | `18` | center/face/edge/corner search: 0, 6, 18, or 26 |
| `representative_max_range` | `5.0` | maximum accepted neighbor distance in metres |
| `representative_preexpand_neighborhoods` | `true` | cache neighbor voxel pointers during insertion (FR-LIO-inspired CPU path) |
| `representative_verify_preexpanded` | `false` | compare cached/direct CPU results exactly; diagnostic overhead |
| `cuda_min_batch_points` | `512` | smaller batches query the CPU mirror |
| `cuda_verify_queries` | `false` | compare every GPU result with the CPU reference |
| `cuda_persistent_queries` | `false` | use mapped-memory persistent query service |
| `cuda_persistent_max_batch_points` | `2048` | mapped query/result buffer capacity |

At shutdown, the node prints GPU/CPU query batch and point counts plus CUDA map
statistics, making crossover tuning observable rather than implicit.

## Local benchmark

RTX 4070 Ti SUPER, CUDA 12.8, compute capability 8.9, 19,621 occupied voxels,
K=4, nearby=18. Timings include input/output copies and synchronization:

This table is the historical `x86-cuda` baseline with the direct CPU neighbor
lookup. The `fralio-cuda` insertion-time CPU cache changes the CPU column and
therefore requires a fresh crossover measurement on an otherwise idle target;
see `FRALIO_CUDA.md` for its isolated and ROS A/B results.

| query points | CPU (us) | normal CUDA (us) | persistent CUDA (us) |
|---:|---:|---:|---:|
| 8 | 0.79 | 41.86 | 33.79 |
| 128 | 16.64 | 67.41 | 68.21 |
| 512 | 87.79 | 77.96 | 251.92 |
| 1024 | 189.39 | 82.22 | 506.13 |
| 2048 | 433.55 | 97.27 | 1008.80 |
| 4096 | 948.15 | 115.82 | n/a |

The representative CPU backend is the important win for the current 1 ms
workload. On the `quick-shack` bag at 4x playback:

| backend/policy | frames | frame mean (ms) | measurement mean (ms) |
|---|---:|---:|---:|
| original CPU | 479 | 3.58 | 2.67 |
| representative CUDA hybrid, 1 ms/K=4 | 480 | 1.59 | 0.73 |
| forced persistent CUDA for every tiny batch | 480 | 13.39 | 12.20 |

The forced result is an intentional stress test, not the default policy. Hybrid
mode avoids that regression. Changing to a representative map also changes the
measurement set, so trajectory quality must be validated on each target dataset
instead of treating timing alone as success.

In a matched 1x playback (483 frames and identical IMU start), K=4 hybrid output
was compared with the original backend at all 483 exact odometry timestamps:
mean position delta 0.072 m, P95 0.217 m, maximum 0.453 m. The representative
backend's start-to-end drift was 0.095 m versus 0.060 m for the original. This is
promising but is not a replacement for ground-truth ATE/RPE on a larger suite.
K=8 did not improve every delta metric, so K=4 remains the neutral default and
K=8 is an explicit quality/speed tuning option.

To exercise the automatic CUDA branch in the ROS node, a 0.1 s batch run sent
414 batches / 328,571 points to CUDA and kept 66 smaller batches / 24,314 points
on CPU. With `cuda_verify_queries=true`, all 328,571 integrated GPU queries
matched the incrementally updated CPU mirror exactly (zero mismatches).

## Verification

Standalone CPU/CUDA tests and benchmark:

```bash
BATCH_LIO_CUDA_ROOT=/usr/local/cuda-12.8 \
BATCH_LIO_CUDA_ARCH=sm_89 \
scripts/test_cuda_backend.sh
```

Memory/race checking:

```bash
BATCH_LIO_CUDA_ROOT=/usr/local/cuda-12.8 \
BATCH_LIO_CUDA_ARCH=sm_89 \
BATCH_LIO_COMPUTE_SANITIZER=1 \
scripts/test_cuda_backend.sh
```

The test covers deterministic CPU/CUDA equality, signed and invalid voxel
coordinates, incremental insertion, persistent queries, insertion while the
persistent service is running, reset, and buffer growth.

## Jetson branch handoff

Create `Jetson-cuda` from this branch when hardware is available. Keep CUDA
architecture/toolkit selection target-specific (for example, Orin uses a
different architecture from this workstation), first reproduce CPU/CUDA
equality, then retune `cuda_min_batch_points` and the persistent path under the
chosen Jetson power mode. Do not copy the x86 crossover numbers into Jetson
configuration: its shared-memory behavior and CPU/GPU balance are different.
