# FR-LIO / FAR-LIO follow-up (`fralio-cuda`)

This branch starts at the verified `x86-cuda` baseline (`38925e4`) and tests
ideas from both similarly named systems mentioned in `GPU.md`:

- **FR-LIO (2023)** uses RC-Vox, a fixed-size robocentric voxel structure, and
  moves neighborhood organization from repeated KNN lookups toward map update.
- **FAR-LIO (2026)** uses a CUDA voxel hash map, sparsity-aware GICP, adaptive
  density/thresholds, and an EKF backend. Its public repository currently has
  only a README and license, so there is no implementation to transplant.

The first safe improvement therefore adopts the FR-LIO data-organization idea
without replacing Batch-LIO's measurement model. It complements the existing
FAR-LIO-like GPU-resident hash map; it is not presented as a reimplementation
of either paper.

Primary references:

- FR-LIO paper: <https://arxiv.org/abs/2302.04031>
- FAR-LIO paper: <https://arxiv.org/abs/2606.26010>
- FAR-LIO repository: <https://github.com/TUMFTM/FAR-LIO>

## Implemented: compact pre-expanded CPU neighborhoods

The representative CPU map normally hashes up to 1/7/19/27 voxel keys for
every query. With `representative_preexpand_neighborhoods=true`, each occupied
query voxel instead stores pointers to the occupied home voxels visible from
it. A query then performs one hash lookup and traverses those pointers.

Important details:

- Representative points remain stored only in their authoritative home voxel.
- Replacing a representative updates what every cached pointer observes; no
  candidate point is copied or allowed to go stale.
- Cache entries are created only for occupied query voxels. Queries in empty
  voxels use the original direct lookup, avoiding the up-to-27x key expansion
  of a fully materialized neighborhood map.
- Signed-coordinate overflow checks and deterministic top-5 ordering are the
  same on both paths.
- Because the cache contains pointers into its own `unordered_map`, the map is
  intentionally non-copyable and non-movable.
- The ROS representative backends enable the cache by default on this branch.
  `original_cpu` remains the global default and is unchanged.

This primarily accelerates the small batches that correctly stay on CPU on an
x86 host. In `representative_cuda` mode, the cached CPU mirror is also the
small-batch/failure fallback. The CUDA hash query itself still probes adjacent
voxel keys and is unchanged in this experiment.

## Runtime controls and observability

```text
representative_preexpand_neighborhoods: true
representative_verify_preexpanded: false
```

Verification mode evaluates both cached and direct lookup for every CPU query,
compares count, coordinates, and squared distances exactly, and falls back to
the direct result if any mismatch is detected. It is a diagnostic mode, not a
benchmark mode.

The shutdown line reports:

```text
cpu_voxels
cached_query_voxels
cached_voxel_refs
cache_payload_mib
preexpanded_verified_queries
preexpanded_mismatches
```

`cache_payload_mib` counts hash buckets, key/vector payloads, and allocated
pointer capacity. Allocator and hash-node bookkeeping are not included, so it
is a useful lower-bound estimate rather than process RSS.

## Standalone benchmark

Run:

```bash
scripts/benchmark_fr_lio_cache.sh
```

Workstation result (120,000 synthetic LiDAR-surface points, 37,816 occupied
voxels, resolution 0.5 m, K=4, nearby=18):

| batch | direct lookup (us) | compact pre-expanded (us) | speedup |
|---:|---:|---:|---:|
| 8 | 1.851 | 0.897 | 2.06x |
| 128 | 38.587 | 16.877 | 2.29x |
| 512 | 348.052 | 120.564 | 2.89x |
| 4096 | 3294.056 | 1639.157 | 2.01x |

Initial insertion increased from 8.91 ms to 37.90 ms. The compact cache used
37,816 keys, 325,436 voxel references, and 4.88 MiB of reported payload. An
earlier fully expanded prototype used 147,859 keys and 14.03 MiB, so it was
discarded despite slightly faster isolated lookups.

## `quick-shack` end-to-end A/B

The final compact design was measured in an interleaved direct/cache/cache/
direct run at 4x playback. All runs processed 480 profiled frames. Values below
are the mean of the two runs for each mode; publishing was disabled equally.

| mode | measurement mean (ms) | measurement P95 (ms) | map insert mean (ms) | frame mean (ms) | frame P95 (ms) |
|---|---:|---:|---:|---:|---:|
| direct representative lookup | 0.7222 | 1.2461 | 0.0265 | 1.4427 | 2.2901 |
| compact pre-expanded lookup | **0.5300** | **0.8748** | 0.0916 | **1.3298** | **2.0076** |
| change | **-26.6%** | **-29.8%** | +0.0651 ms | **-7.8%** | **-12.3%** |

The integrated map had about 24.6k occupied/cached query voxels, 268k cached
voxel references, and 4.23 MiB of reported cache payload.

In a separate diagnostic run, all **352,338** real estimator queries matched
the original direct path exactly (**zero mismatches**). Repeated offline runs
still show the existing initialization/timing variability, so these results do
not claim an ATE/RPE improvement. Ground-truth trajectory evaluation remains
required before changing the measurement model.

## What should come next from FAR-LIO

The next large gain is not another standalone KNN tweak. It is a fused CUDA
measurement path:

```text
transform/query -> plane or sparse GICP residual -> Jacobian -> HtH/Htz reduction
```

Only the fixed-size information terms should return to the CPU. That removes
the current neighbor-result transfer and CPU plane/Jacobian loop, and is the
path most likely to lower the x86/Jetson CUDA crossover below 512 points.

Before that path can be enabled by default, it needs:

1. a numerically verified GPU plane/GICP solver against the current Eigen QR;
2. an information-form EKF interface with CPU/GPU equivalence tests;
3. GPU map eviction or a robocentric sliding window (the current representative
   map rejects new voxels after reaching capacity);
4. ground-truth ATE/RPE plus P50/P95/P99 latency tests;
5. Jetson Nsight, power-mode, thermal, and concurrent TensorRT measurements.

The FAR-LIO repository does not yet supply source for these pieces. Implementing
its GICP/adaptive-density front end immediately would change estimator behavior
and make this branch impossible to attribute. The compact exact cache is kept
as the independently measurable first step.
