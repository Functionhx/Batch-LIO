#include "cuda/cuda_representative_ivox.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstring>
#include <cstdint>
#include <limits>
#include <sstream>
#include <thread>
#include <type_traits>
#include <utility>

namespace batchlio {
namespace {

constexpr int kEmpty = 0;
constexpr int kOccupied = 2;
constexpr int kQueryThreads = 128;
constexpr int kPersistentControlRequest = 0;
constexpr int kPersistentControlCompletion = 1;
constexpr int kPersistentControlCount = 2;
constexpr int kPersistentControlStop = 3;
constexpr int kPersistentControlWords = 4;

static_assert(std::is_trivially_copyable<Point3f>::value, "Point3f must be CUDA-copyable");
static_assert(std::is_trivially_copyable<NeighborSet>::value, "NeighborSet must be CUDA-copyable");

struct DeviceTable {
    int4* keys = nullptr;                 // xyz = signed voxel key, w = occupancy
    unsigned int* counts = nullptr;       // representative count for each slot
    Point3f* points = nullptr;            // [slot * max_points_per_voxel + point]
    unsigned long long* counters = nullptr;  // voxels, accepted points, rejected points
    std::size_t slot_count = 0;
    std::size_t voxel_capacity = 0;
    int max_points_per_voxel = 0;
    int nearby_type = 0;
    float resolution = 0.0f;
    float inv_resolution = 0.0f;
    float max_squared_range = 0.0f;
};

__host__ std::size_t NextPowerOfTwo(std::size_t value) {
    if (value <= 1) return 1;
    --value;
    for (std::size_t shift = 1; shift < sizeof(std::size_t) * CHAR_BIT; shift <<= 1U) {
        value |= value >> shift;
    }
    return value + 1;
}

__host__ __device__ inline std::uint64_t Mix64(std::uint64_t value) {
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31U;
    return value;
}

__device__ inline std::size_t HashKey(int x, int y, int z, std::size_t mask) {
    std::uint64_t value = static_cast<std::uint32_t>(x);
    value = Mix64(value ^ (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) << 1U));
    value = Mix64(value ^ (static_cast<std::uint64_t>(static_cast<std::uint32_t>(z)) << 32U));
    return static_cast<std::size_t>(value) & mask;
}

__device__ inline bool SamePointDevice(const Point3f& lhs, const Point3f& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

__device__ inline bool PointLessDevice(const Point3f& lhs, const Point3f& rhs) {
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    if (lhs.y != rhs.y) return lhs.y < rhs.y;
    return lhs.z < rhs.z;
}

__device__ inline float SquaredDistanceDevice(const Point3f& lhs, const Point3f& rhs) {
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    const float dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

__device__ inline bool PositionToKeyDevice(const Point3f& point, float inv_resolution,
                                           int& x, int& y, int& z) {
    if (!isfinite(point.x) || !isfinite(point.y) || !isfinite(point.z)) return false;
    const float sx = floorf(point.x * inv_resolution);
    const float sy = floorf(point.y * inv_resolution);
    const float sz = floorf(point.z * inv_resolution);
    constexpr float kIntMinAsFloat = -2147483648.0f;
    constexpr float kIntMaxExclusiveAsFloat = 2147483648.0f;
    if (sx < kIntMinAsFloat || sx >= kIntMaxExclusiveAsFloat ||
        sy < kIntMinAsFloat || sy >= kIntMaxExclusiveAsFloat ||
        sz < kIntMinAsFloat || sz >= kIntMaxExclusiveAsFloat) {
        return false;
    }
    x = static_cast<int>(sx);
    y = static_cast<int>(sy);
    z = static_cast<int>(sz);
    return true;
}

__device__ inline float RepresentativeScoreDevice(int key_x, int key_y, int key_z,
                                                   float resolution, const Point3f& point) {
    const Point3f center{(static_cast<float>(key_x) + 0.5f) * resolution,
                         (static_cast<float>(key_y) + 0.5f) * resolution,
                         (static_cast<float>(key_z) + 0.5f) * resolution};
    return SquaredDistanceDevice(point, center);
}

__device__ inline bool RepresentativeLessDevice(int key_x, int key_y, int key_z,
                                                float resolution,
                                                const Point3f& lhs, const Point3f& rhs) {
    const float lhs_score = RepresentativeScoreDevice(key_x, key_y, key_z, resolution, lhs);
    const float rhs_score = RepresentativeScoreDevice(key_x, key_y, key_z, resolution, rhs);
    if (lhs_score != rhs_score) return lhs_score < rhs_score;
    return PointLessDevice(lhs, rhs);
}

__global__ void InsertPointsKernel(DeviceTable table, const Point3f* input, std::size_t count) {
    // Map insertion is <1% of the current frame time. Processing it in one
    // deterministic device thread makes the retained representative set exactly
    // reproducible and avoids lock-order effects; KNN remains fully parallel.
    if (blockIdx.x != 0 || threadIdx.x != 0) return;

    const std::size_t mask = table.slot_count - 1U;
    for (std::size_t input_index = 0; input_index < count; ++input_index) {
        const Point3f point = input[input_index];
        int key_x = 0;
        int key_y = 0;
        int key_z = 0;
        if (!PositionToKeyDevice(point, table.inv_resolution, key_x, key_y, key_z)) {
            ++table.counters[2];
            continue;
        }

        std::size_t slot = HashKey(key_x, key_y, key_z, mask);
        bool handled = false;
        for (std::size_t probe = 0; probe < table.slot_count; ++probe) {
            int4& key = table.keys[slot];
            if (key.w == kEmpty) {
                if (table.counters[0] >= table.voxel_capacity) {
                    ++table.counters[2];
                    handled = true;
                    break;
                }
                key.x = key_x;
                key.y = key_y;
                key.z = key_z;
                key.w = kOccupied;
                table.counts[slot] = 1U;
                table.points[slot * static_cast<std::size_t>(table.max_points_per_voxel)] = point;
                ++table.counters[0];
                ++table.counters[1];
                handled = true;
                break;
            }

            if (key.w == kOccupied && key.x == key_x && key.y == key_y && key.z == key_z) {
                unsigned int voxel_count = table.counts[slot];
                Point3f* representatives =
                    table.points + slot * static_cast<std::size_t>(table.max_points_per_voxel);
                bool duplicate = false;
                for (unsigned int j = 0; j < voxel_count; ++j) {
                    if (SamePointDevice(representatives[j], point)) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate) {
                    handled = true;
                    break;
                }

                if (voxel_count < static_cast<unsigned int>(table.max_points_per_voxel)) {
                    representatives[voxel_count] = point;
                    table.counts[slot] = voxel_count + 1U;
                    ++table.counters[1];
                    handled = true;
                    break;
                }

                unsigned int worst = 0;
                for (unsigned int j = 1; j < voxel_count; ++j) {
                    if (RepresentativeLessDevice(key_x, key_y, key_z, table.resolution,
                                                 representatives[worst], representatives[j])) {
                        worst = j;
                    }
                }
                if (RepresentativeLessDevice(key_x, key_y, key_z, table.resolution,
                                             point, representatives[worst])) {
                    representatives[worst] = point;
                    ++table.counters[1];
                }
                handled = true;
                break;
            }
            slot = (slot + 1U) & mask;
        }
        if (!handled) ++table.counters[2];
    }
}

__device__ inline bool IncludeNeighborOffset(int nearby_type, int dx, int dy, int dz) {
    const int manhattan = abs(dx) + abs(dy) + abs(dz);
    return nearby_type == 26 ||
           (nearby_type == 18 && manhattan <= 2) ||
           (nearby_type == 6 && manhattan <= 1) ||
           (nearby_type == 0 && manhattan == 0);
}

__device__ inline bool CandidateLess(float lhs_distance, const Point3f& lhs,
                                     float rhs_distance, const Point3f& rhs) {
    if (lhs_distance != rhs_distance) return lhs_distance < rhs_distance;
    return PointLessDevice(lhs, rhs);
}

__device__ inline void InsertCandidate(NeighborSet& result, const Point3f& point, float distance) {
    int insert_at = result.count;
    if (result.count == kRepresentativeIvoxMaxNeighbors) {
        const int last = kRepresentativeIvoxMaxNeighbors - 1;
        if (!CandidateLess(distance, point, result.squared_distances[last], result.points[last])) return;
        insert_at = last;
    } else {
        ++result.count;
    }

    while (insert_at > 0 &&
           CandidateLess(distance, point, result.squared_distances[insert_at - 1],
                         result.points[insert_at - 1])) {
        if (insert_at < kRepresentativeIvoxMaxNeighbors) {
            result.points[insert_at] = result.points[insert_at - 1];
            result.squared_distances[insert_at] = result.squared_distances[insert_at - 1];
        }
        --insert_at;
    }
    result.points[insert_at] = point;
    result.squared_distances[insert_at] = distance;
}

__device__ inline void QueryOne(DeviceTable table, const Point3f& query, NeighborSet& result) {
    result.count = 0;
    for (int i = 0; i < kRepresentativeIvoxMaxNeighbors; ++i) {
        result.points[i].x = 0.0f;
        result.points[i].y = 0.0f;
        result.points[i].z = 0.0f;
        result.squared_distances[i] = 0.0f;
    }

    int center_x = 0;
    int center_y = 0;
    int center_z = 0;
    if (!PositionToKeyDevice(query, table.inv_resolution, center_x, center_y, center_z)) return;

    const std::size_t mask = table.slot_count - 1U;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (!IncludeNeighborOffset(table.nearby_type, dx, dy, dz)) continue;
                const long long candidate_x = static_cast<long long>(center_x) + dx;
                const long long candidate_y = static_cast<long long>(center_y) + dy;
                const long long candidate_z = static_cast<long long>(center_z) + dz;
                if (candidate_x < INT_MIN || candidate_x > INT_MAX ||
                    candidate_y < INT_MIN || candidate_y > INT_MAX ||
                    candidate_z < INT_MIN || candidate_z > INT_MAX) {
                    continue;
                }
                const int key_x = static_cast<int>(candidate_x);
                const int key_y = static_cast<int>(candidate_y);
                const int key_z = static_cast<int>(candidate_z);
                std::size_t slot = HashKey(key_x, key_y, key_z, mask);

                for (std::size_t probe = 0; probe < table.slot_count; ++probe) {
                    const int4 key = table.keys[slot];
                    if (key.w == kEmpty) break;
                    if (key.w == kOccupied && key.x == key_x && key.y == key_y && key.z == key_z) {
                        const unsigned int voxel_count = table.counts[slot];
                        const Point3f* representatives =
                            table.points + slot * static_cast<std::size_t>(table.max_points_per_voxel);
                        for (unsigned int j = 0; j < voxel_count; ++j) {
                            const float distance = SquaredDistanceDevice(representatives[j], query);
                            if (distance < table.max_squared_range) {
                                InsertCandidate(result, representatives[j], distance);
                            }
                        }
                        break;
                    }
                    slot = (slot + 1U) & mask;
                }
            }
        }
    }
}

__global__ void QueryKernel(DeviceTable table, const Point3f* queries,
                            std::size_t count, NeighborSet* results) {
    const std::size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (query_index >= count) return;
    QueryOne(table, queries[query_index], results[query_index]);
}

__global__ void PersistentQueryKernel(DeviceTable table, const Point3f* queries,
                                      NeighborSet* results, volatile int* control) {
    __shared__ int request_sequence;
    __shared__ int query_count;
    __shared__ int should_stop;
    int completed_sequence = 0;

    while (true) {
        if (threadIdx.x == 0) {
            int observed_request = control[kPersistentControlRequest];
            while (observed_request == completed_sequence &&
                   control[kPersistentControlStop] == 0) {
#if __CUDA_ARCH__ >= 700
                __nanosleep(1000U);
#endif
                observed_request = control[kPersistentControlRequest];
            }
            request_sequence = observed_request;
            query_count = control[kPersistentControlCount];
            should_stop = control[kPersistentControlStop];
        }
        __syncthreads();
        if (should_stop != 0) return;

        for (int query_index = static_cast<int>(threadIdx.x);
             query_index < query_count;
             query_index += static_cast<int>(blockDim.x)) {
            QueryOne(table, queries[query_index], results[query_index]);
        }
        __syncthreads();
        if (threadIdx.x == 0) {
            __threadfence_system();
            control[kPersistentControlCompletion] = request_sequence;
            completed_sequence = request_sequence;
        }
        __syncthreads();
    }
}

}  // namespace

class CudaRepresentativeIvox::Impl {
public:
    explicit Impl(RepresentativeIvoxConfig value) : config(std::move(value)) {
        RepresentativeIvox::ValidateConfig(config);
        Initialize();
    }

    ~Impl() { Release(); }

    bool Check(cudaError_t status, const char* operation) {
        if (status == cudaSuccess) return true;
        std::ostringstream stream_message;
        stream_message << operation << ": " << cudaGetErrorString(status);
        error = stream_message.str();
        ready = false;
        return false;
    }

    void Initialize() {
        std::string runtime_reason;
        if (!CudaRepresentativeIvox::RuntimeAvailable(&runtime_reason)) {
            error = runtime_reason;
            return;
        }

        if (config.capacity > std::numeric_limits<std::size_t>::max() / 2U) {
            error = "CUDA representative iVox capacity is too large";
            return;
        }
        table.slot_count = NextPowerOfTwo(std::max<std::size_t>(16U, config.capacity * 2U));
        table.voxel_capacity = config.capacity;
        table.max_points_per_voxel = config.max_points_per_voxel;
        table.nearby_type = static_cast<int>(config.nearby_type);
        table.resolution = config.resolution;
        table.inv_resolution = 1.0f / config.resolution;
        table.max_squared_range = config.max_range * config.max_range;

        if (!Check(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), "cudaStreamCreateWithFlags") ||
            !Check(cudaEventCreate(&start_event), "cudaEventCreate(start)") ||
            !Check(cudaEventCreate(&stop_event), "cudaEventCreate(stop)") ||
            !Check(cudaMalloc(reinterpret_cast<void**>(&table.keys), table.slot_count * sizeof(int4)),
                   "cudaMalloc(keys)") ||
            !Check(cudaMalloc(reinterpret_cast<void**>(&table.counts),
                              table.slot_count * sizeof(unsigned int)), "cudaMalloc(counts)") ||
            !Check(cudaMalloc(reinterpret_cast<void**>(&table.points),
                              table.slot_count * static_cast<std::size_t>(table.max_points_per_voxel) *
                                  sizeof(Point3f)), "cudaMalloc(points)") ||
            !Check(cudaMalloc(reinterpret_cast<void**>(&table.counters),
                              3U * sizeof(unsigned long long)), "cudaMalloc(counters)")) {
            Release();
            return;
        }
        ready = true;
        if (!Reset()) Release();
    }

    bool Reset() {
        if (!ready) return false;
        const bool restart_persistent = persistent_ready;
        if (restart_persistent && !SuspendPersistentQueryKernel()) return false;
        if (!Check(cudaMemsetAsync(table.keys, 0, table.slot_count * sizeof(int4), stream),
                   "cudaMemsetAsync(keys)") ||
            !Check(cudaMemsetAsync(table.counts, 0, table.slot_count * sizeof(unsigned int), stream),
                   "cudaMemsetAsync(counts)") ||
            !Check(cudaMemsetAsync(table.counters, 0, 3U * sizeof(unsigned long long), stream),
                   "cudaMemsetAsync(counters)") ||
            !Check(cudaStreamSynchronize(stream), "cudaStreamSynchronize(reset)")) {
            return false;
        }
        if (restart_persistent && !ResumePersistentQueryKernel()) return false;
        last_insert_kernel_ms = 0.0f;
        last_query_kernel_ms = 0.0f;
        error.clear();
        return true;
    }

    bool EnsureInsertCapacity(std::size_t count) {
        if (count <= insert_capacity) return true;
        Point3f* replacement = nullptr;
        if (!Check(cudaMalloc(reinterpret_cast<void**>(&replacement), count * sizeof(Point3f)),
                   "cudaMalloc(insert buffer)")) return false;
        if (insert_buffer != nullptr) cudaFree(insert_buffer);
        insert_buffer = replacement;
        insert_capacity = count;
        return true;
    }

    bool EnsureQueryCapacity(std::size_t count) {
        if (count <= query_capacity) return true;
        Point3f* replacement_queries = nullptr;
        NeighborSet* replacement_results = nullptr;
        if (!Check(cudaMalloc(reinterpret_cast<void**>(&replacement_queries), count * sizeof(Point3f)),
                   "cudaMalloc(query buffer)")) return false;
        if (!Check(cudaMalloc(reinterpret_cast<void**>(&replacement_results), count * sizeof(NeighborSet)),
                   "cudaMalloc(result buffer)")) {
            cudaFree(replacement_queries);
            return false;
        }
        if (query_buffer != nullptr) cudaFree(query_buffer);
        if (result_buffer != nullptr) cudaFree(result_buffer);
        query_buffer = replacement_queries;
        result_buffer = replacement_results;
        query_capacity = count;
        return true;
    }

    bool AddPoints(const Point3f* points, std::size_t count) {
        if (!ready) return false;
        if (count == 0) return true;
        if (points == nullptr) {
            error = "CudaRepresentativeIvox::AddPoints received a null pointer";
            return false;
        }
        // cudaMalloc/cudaFree are synchronizing operations, and map writes also
        // need a kernel boundary before the persistent reader can safely observe
        // them. Suspend the polling kernel for the comparatively rare map update.
        const bool restart_persistent = persistent_ready;
        if (restart_persistent && !SuspendPersistentQueryKernel()) return false;
        if (!EnsureInsertCapacity(count) ||
            !Check(cudaMemcpyAsync(insert_buffer, points, count * sizeof(Point3f),
                                   cudaMemcpyHostToDevice, stream), "cudaMemcpyAsync(insert input)") ||
            !Check(cudaEventRecord(start_event, stream), "cudaEventRecord(insert start)")) {
            return false;
        }

        InsertPointsKernel<<<1, 1, 0, stream>>>(table, insert_buffer, count);
        if (!Check(cudaGetLastError(), "InsertPointsKernel launch") ||
            !Check(cudaEventRecord(stop_event, stream), "cudaEventRecord(insert stop)") ||
            !Check(cudaEventSynchronize(stop_event), "cudaEventSynchronize(insert)")) {
            return false;
        }
        if (!Check(cudaEventElapsedTime(&last_insert_kernel_ms, start_event, stop_event),
                   "cudaEventElapsedTime(insert)")) return false;
        if (restart_persistent && !ResumePersistentQueryKernel()) return false;
        error.clear();
        return true;
    }

    bool Query(const Point3f* queries, std::size_t count, NeighborSet* results) {
        if (!ready) return false;
        if (count == 0) return true;
        if (queries == nullptr || results == nullptr) {
            error = "CudaRepresentativeIvox::Query received a null pointer";
            return false;
        }
        if (persistent_ready && count <= persistent_capacity) {
            return QueryPersistent(queries, count, results);
        }

        // A batch larger than the mapped persistent buffers uses the ordinary
        // stream. Suspend first so a capacity growth cannot synchronize against
        // an intentionally non-terminating kernel.
        const bool restart_persistent = persistent_ready;
        if (restart_persistent && !SuspendPersistentQueryKernel()) return false;
        const auto wall_start = std::chrono::steady_clock::now();
        if (!EnsureQueryCapacity(count) ||
            !Check(cudaMemcpyAsync(query_buffer, queries, count * sizeof(Point3f),
                                   cudaMemcpyHostToDevice, stream), "cudaMemcpyAsync(query input)") ||
            !Check(cudaEventRecord(start_event, stream), "cudaEventRecord(query start)")) {
            return false;
        }

        const unsigned int blocks = static_cast<unsigned int>((count + kQueryThreads - 1U) / kQueryThreads);
        QueryKernel<<<blocks, kQueryThreads, 0, stream>>>(table, query_buffer, count, result_buffer);
        if (!Check(cudaGetLastError(), "QueryKernel launch") ||
            !Check(cudaEventRecord(stop_event, stream), "cudaEventRecord(query stop)") ||
            !Check(cudaMemcpyAsync(results, result_buffer, count * sizeof(NeighborSet),
                                   cudaMemcpyDeviceToHost, stream), "cudaMemcpyAsync(query output)") ||
            !Check(cudaStreamSynchronize(stream), "cudaStreamSynchronize(query)")) {
            return false;
        }
        if (!Check(cudaEventElapsedTime(&last_query_kernel_ms, start_event, stop_event),
                   "cudaEventElapsedTime(query)")) return false;
        if (restart_persistent && !ResumePersistentQueryKernel()) return false;
        last_query_end_to_end_ms = static_cast<float>(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start).count());
        error.clear();
        return true;
    }

    bool StartPersistentQueryService(std::size_t max_query_points) {
        if (!ready) return false;
        if (max_query_points == 0 || max_query_points > static_cast<std::size_t>(INT_MAX)) {
            error = "persistent CUDA query capacity must be in [1, INT_MAX]";
            return false;
        }
        StopPersistentQueryService();

        int device = 0;
        cudaDeviceProp properties{};
        if (!Check(cudaGetDevice(&device), "cudaGetDevice") ||
            !Check(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties")) {
            return false;
        }
        if (!properties.canMapHostMemory) {
            error = "CUDA device cannot map host memory for the persistent query service";
            return false;
        }

        if (!Check(cudaHostAlloc(reinterpret_cast<void**>(&persistent_control_host),
                                 kPersistentControlWords * sizeof(int), cudaHostAllocMapped),
                   "cudaHostAlloc(persistent control)") ||
            !Check(cudaHostAlloc(reinterpret_cast<void**>(&persistent_queries_host),
                                 max_query_points * sizeof(Point3f), cudaHostAllocMapped),
                   "cudaHostAlloc(persistent queries)") ||
            !Check(cudaHostAlloc(reinterpret_cast<void**>(&persistent_results_host),
                                 max_query_points * sizeof(NeighborSet), cudaHostAllocMapped),
                   "cudaHostAlloc(persistent results)") ||
            !Check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&persistent_control_device),
                                            persistent_control_host, 0),
                   "cudaHostGetDevicePointer(persistent control)") ||
            !Check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&persistent_queries_device),
                                            persistent_queries_host, 0),
                   "cudaHostGetDevicePointer(persistent queries)") ||
            !Check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&persistent_results_device),
                                            persistent_results_host, 0),
                   "cudaHostGetDevicePointer(persistent results)") ||
            !Check(cudaStreamCreateWithFlags(&persistent_stream, cudaStreamNonBlocking),
                   "cudaStreamCreateWithFlags(persistent)")) {
            StopPersistentQueryService();
            return false;
        }

        std::memset(persistent_control_host, 0, kPersistentControlWords * sizeof(int));
        persistent_capacity = max_query_points;
        persistent_sequence = 0;
        if (!ResumePersistentQueryKernel()) {
            StopPersistentQueryService();
            return false;
        }
        error.clear();
        return true;
    }

    bool ResumePersistentQueryKernel() {
        if (persistent_stream == nullptr || persistent_control_host == nullptr ||
            persistent_queries_device == nullptr || persistent_results_device == nullptr) {
            error = "persistent CUDA query service buffers are not initialized";
            return false;
        }
        std::memset(persistent_control_host, 0, kPersistentControlWords * sizeof(int));
        persistent_sequence = 0;
        PersistentQueryKernel<<<1, kQueryThreads, 0, persistent_stream>>>(
            table, persistent_queries_device, persistent_results_device,
            persistent_control_device);
        if (!Check(cudaGetLastError(), "PersistentQueryKernel launch")) {
            return false;
        }
        persistent_ready = true;
        return true;
    }

    bool SuspendPersistentQueryKernel() {
        if (!persistent_ready) return true;
        __atomic_store_n(&persistent_control_host[kPersistentControlStop], 1, __ATOMIC_RELEASE);
        if (!Check(cudaStreamSynchronize(persistent_stream),
                   "cudaStreamSynchronize(persistent suspend)")) {
            return false;
        }
        persistent_ready = false;
        return true;
    }

    bool QueryPersistent(const Point3f* queries, std::size_t count, NeighborSet* results) {
        const auto wall_start = std::chrono::steady_clock::now();
        std::memcpy(persistent_queries_host, queries, count * sizeof(Point3f));
        __atomic_store_n(&persistent_control_host[kPersistentControlCount],
                         static_cast<int>(count), __ATOMIC_RELAXED);
        const int next_sequence = ++persistent_sequence;
        __atomic_store_n(&persistent_control_host[kPersistentControlRequest],
                         next_sequence, __ATOMIC_RELEASE);

        const auto deadline = wall_start + std::chrono::seconds(1);
        while (__atomic_load_n(&persistent_control_host[kPersistentControlCompletion],
                               __ATOMIC_ACQUIRE) != next_sequence) {
            if (std::chrono::steady_clock::now() >= deadline) {
                error = "persistent CUDA query timed out after 1 second";
                return false;
            }
            std::this_thread::yield();
        }
        std::memcpy(results, persistent_results_host, count * sizeof(NeighborSet));
        last_query_kernel_ms = 0.0f;
        last_query_end_to_end_ms = static_cast<float>(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start).count());
        error.clear();
        return true;
    }

    void StopPersistentQueryService() {
        if (persistent_ready) SuspendPersistentQueryKernel();
        if (persistent_stream != nullptr) cudaStreamDestroy(persistent_stream);
        if (persistent_control_host != nullptr) cudaFreeHost(persistent_control_host);
        if (persistent_queries_host != nullptr) cudaFreeHost(persistent_queries_host);
        if (persistent_results_host != nullptr) cudaFreeHost(persistent_results_host);
        persistent_stream = nullptr;
        persistent_control_host = nullptr;
        persistent_queries_host = nullptr;
        persistent_results_host = nullptr;
        persistent_control_device = nullptr;
        persistent_queries_device = nullptr;
        persistent_results_device = nullptr;
        persistent_capacity = 0;
        persistent_sequence = 0;
        persistent_ready = false;
    }

    CudaRepresentativeIvoxStats Stats() const {
        CudaRepresentativeIvoxStats result;
        result.last_insert_kernel_ms = last_insert_kernel_ms;
        result.last_query_kernel_ms = last_query_kernel_ms;
        result.last_query_end_to_end_ms = last_query_end_to_end_ms;
        result.persistent_query_service = persistent_ready;
        if (!ready || table.counters == nullptr) return result;
        unsigned long long counters[3] = {0, 0, 0};
        if (cudaMemcpyAsync(counters, table.counters, sizeof(counters),
                            cudaMemcpyDeviceToHost, stream) == cudaSuccess &&
            cudaStreamSynchronize(stream) == cudaSuccess) {
            result.voxel_count = counters[0];
            result.accepted_points = counters[1];
            result.rejected_points = counters[2];
        }
        return result;
    }

    void Release() {
        StopPersistentQueryService();
        if (insert_buffer != nullptr) cudaFree(insert_buffer);
        if (query_buffer != nullptr) cudaFree(query_buffer);
        if (result_buffer != nullptr) cudaFree(result_buffer);
        if (table.keys != nullptr) cudaFree(table.keys);
        if (table.counts != nullptr) cudaFree(table.counts);
        if (table.points != nullptr) cudaFree(table.points);
        if (table.counters != nullptr) cudaFree(table.counters);
        if (start_event != nullptr) cudaEventDestroy(start_event);
        if (stop_event != nullptr) cudaEventDestroy(stop_event);
        if (stream != nullptr) cudaStreamDestroy(stream);
        insert_buffer = nullptr;
        query_buffer = nullptr;
        result_buffer = nullptr;
        table.keys = nullptr;
        table.counts = nullptr;
        table.points = nullptr;
        table.counters = nullptr;
        start_event = nullptr;
        stop_event = nullptr;
        stream = nullptr;
        ready = false;
    }

    RepresentativeIvoxConfig config;
    DeviceTable table;
    cudaStream_t stream = nullptr;
    cudaEvent_t start_event = nullptr;
    cudaEvent_t stop_event = nullptr;
    Point3f* insert_buffer = nullptr;
    Point3f* query_buffer = nullptr;
    NeighborSet* result_buffer = nullptr;
    std::size_t insert_capacity = 0;
    std::size_t query_capacity = 0;
    float last_insert_kernel_ms = 0.0f;
    float last_query_kernel_ms = 0.0f;
    float last_query_end_to_end_ms = 0.0f;
    cudaStream_t persistent_stream = nullptr;
    int* persistent_control_host = nullptr;
    Point3f* persistent_queries_host = nullptr;
    NeighborSet* persistent_results_host = nullptr;
    int* persistent_control_device = nullptr;
    Point3f* persistent_queries_device = nullptr;
    NeighborSet* persistent_results_device = nullptr;
    std::size_t persistent_capacity = 0;
    int persistent_sequence = 0;
    bool persistent_ready = false;
    bool ready = false;
    std::string error;
};

CudaRepresentativeIvox::CudaRepresentativeIvox(RepresentativeIvoxConfig config)
    : impl_(new Impl(std::move(config))) {}

CudaRepresentativeIvox::~CudaRepresentativeIvox() = default;
CudaRepresentativeIvox::CudaRepresentativeIvox(CudaRepresentativeIvox&&) noexcept = default;
CudaRepresentativeIvox& CudaRepresentativeIvox::operator=(CudaRepresentativeIvox&&) noexcept = default;

bool CudaRepresentativeIvox::IsCompiled() { return true; }

bool CudaRepresentativeIvox::RuntimeAvailable(std::string* reason) {
    int count = 0;
    const cudaError_t status = cudaGetDeviceCount(&count);
    if (status != cudaSuccess) {
        if (reason != nullptr) {
            *reason = std::string("cudaGetDeviceCount: ") + cudaGetErrorString(status);
        }
        // Clear the sticky runtime error so a later diagnostic remains meaningful.
        cudaGetLastError();
        return false;
    }
    if (count <= 0) {
        if (reason != nullptr) *reason = "no CUDA device is visible";
        return false;
    }
    if (reason != nullptr) reason->clear();
    return true;
}

bool CudaRepresentativeIvox::IsReady() const { return impl_ != nullptr && impl_->ready; }
const std::string& CudaRepresentativeIvox::LastError() const { return impl_->error; }
const RepresentativeIvoxConfig& CudaRepresentativeIvox::config() const { return impl_->config; }
bool CudaRepresentativeIvox::Reset() { return impl_->Reset(); }
bool CudaRepresentativeIvox::AddPoints(const Point3f* points, std::size_t count) {
    return impl_->AddPoints(points, count);
}
bool CudaRepresentativeIvox::Query(const Point3f* queries, std::size_t count, NeighborSet* results) {
    return impl_->Query(queries, count, results);
}
bool CudaRepresentativeIvox::StartPersistentQueryService(std::size_t max_query_points) {
    return impl_->StartPersistentQueryService(max_query_points);
}
void CudaRepresentativeIvox::StopPersistentQueryService() {
    impl_->StopPersistentQueryService();
}
bool CudaRepresentativeIvox::PersistentQueryServiceReady() const {
    return impl_->persistent_ready;
}
CudaRepresentativeIvoxStats CudaRepresentativeIvox::Stats() const { return impl_->Stats(); }

}  // namespace batchlio
