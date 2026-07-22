#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace batchlio {

// CUDA-facing data types intentionally have no Eigen/PCL members. This keeps the
// map backend independently testable and makes host/device copies explicit.
struct Point3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

constexpr int kRepresentativeIvoxMaxNeighbors = 5;
constexpr int kRepresentativeIvoxMaxPointsPerVoxel = 8;

struct NeighborSet {
    int count = 0;
    Point3f points[kRepresentativeIvoxMaxNeighbors]{};
    float squared_distances[kRepresentativeIvoxMaxNeighbors]{};
};

enum class RepresentativeNearbyType : int {
    CENTER = 0,
    NEARBY6 = 6,
    NEARBY18 = 18,
    NEARBY26 = 26,
};

struct RepresentativeIvoxConfig {
    float resolution = 0.5f;
    int max_points_per_voxel = 4;
    RepresentativeNearbyType nearby_type = RepresentativeNearbyType::NEARBY18;
    std::size_t capacity = 262144;
    float max_range = 5.0f;
    // FR-LIO/RC-Vox-inspired CPU optimization. Occupied query voxels keep their
    // visible home-voxel pointers, replacing up to 27 hash probes with one hash
    // probe plus pointer traversal. Queries in empty voxels use the direct path.
    // The pointed-to representative vectors remain authoritative, so enabling
    // this cache does not change the retained points or KNN result.
    bool preexpand_neighborhoods = false;
};

struct VoxelKey {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

    bool operator==(const VoxelKey& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        // Signed coordinates are intentionally converted to their two's-complement
        // bit patterns before mixing. Unlike Small Point-LIO's uint16 packing this
        // does not alias coordinates every 65536 voxels.
        std::uint64_t h = 0x9e3779b97f4a7c15ULL;
        h ^= mix(static_cast<std::uint32_t>(key.x) + 0x9e3779b9U);
        h ^= rotate_left(mix(static_cast<std::uint32_t>(key.y) + 0x85ebca6bU), 21);
        h ^= rotate_left(mix(static_cast<std::uint32_t>(key.z) + 0xc2b2ae35U), 42);
        return static_cast<std::size_t>(mix64(h));
    }

private:
    static std::uint64_t rotate_left(std::uint64_t value, unsigned int shift) {
        return (value << shift) | (value >> (64U - shift));
    }

    static std::uint64_t mix(std::uint64_t value) {
        value ^= value >> 16U;
        value *= 0x7feb352dULL;
        value ^= value >> 15U;
        value *= 0x846ca68bULL;
        value ^= value >> 16U;
        return value;
    }

    static std::uint64_t mix64(std::uint64_t value) {
        value ^= value >> 30U;
        value *= 0xbf58476d1ce4e5b9ULL;
        value ^= value >> 27U;
        value *= 0x94d049bb133111ebULL;
        value ^= value >> 31U;
        return value;
    }
};

inline bool IsFinite(const Point3f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

inline float SquaredDistance(const Point3f& lhs, const Point3f& rhs) {
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    const float dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

class RepresentativeIvox {
public:
    explicit RepresentativeIvox(RepresentativeIvoxConfig config = {}) : config_(config) {
        ValidateConfig(config_);
        inv_resolution_ = 1.0f / config_.resolution;
        voxels_.reserve(std::min<std::size_t>(config_.capacity, 65536));
        if (config_.preexpand_neighborhoods) {
            query_neighborhoods_.reserve(std::min<std::size_t>(config_.capacity, 65536));
        }
    }

    // Cached neighborhoods contain pointers into voxels_. The C++ standard
    // keeps unordered_map element pointers valid across rehashes, but copying or
    // moving this object would make the self-referential cache invalid.
    RepresentativeIvox(const RepresentativeIvox&) = delete;
    RepresentativeIvox& operator=(const RepresentativeIvox&) = delete;
    RepresentativeIvox(RepresentativeIvox&&) = delete;
    RepresentativeIvox& operator=(RepresentativeIvox&&) = delete;

    void Reset() {
        query_neighborhoods_.clear();
        voxels_.clear();
        rejected_insertions_ = 0;
        cached_voxel_references_ = 0;
    }

    bool AddPoint(const Point3f& point) {
        if (!IsFinite(point)) {
            ++rejected_insertions_;
            return false;
        }

        VoxelKey key;
        if (!PositionToKey(point, key)) {
            ++rejected_insertions_;
            return false;
        }
        auto iter = voxels_.find(key);
        bool inserted_voxel = false;
        if (iter == voxels_.end()) {
            if (voxels_.size() >= config_.capacity) {
                ++rejected_insertions_;
                return false;
            }
            Voxel voxel;
            voxel.points.reserve(static_cast<std::size_t>(config_.max_points_per_voxel));
            iter = voxels_.emplace(key, std::move(voxel)).first;
            inserted_voxel = true;
        }

        auto& points = iter->second.points;
        if (std::find_if(points.begin(), points.end(), [&](const Point3f& existing) {
                return SamePoint(existing, point);
            }) != points.end()) {
            return false;
        }

        points.push_back(point);
        SortRepresentatives(key, points);
        if (points.size() > static_cast<std::size_t>(config_.max_points_per_voxel)) {
            points.resize(static_cast<std::size_t>(config_.max_points_per_voxel));
        }
        const bool retained =
            std::find_if(points.begin(), points.end(), [&](const Point3f& existing) {
                return SamePoint(existing, point);
            }) != points.end();
        if (inserted_voxel && retained) {
            RegisterVoxelWithQueryNeighborhoods(key, &iter->second);
        }
        return retained;
    }

    std::size_t AddPoints(const Point3f* points, std::size_t count) {
        if (points == nullptr && count != 0) {
            throw std::invalid_argument("RepresentativeIvox::AddPoints received a null pointer");
        }
        std::size_t accepted = 0;
        for (std::size_t i = 0; i < count; ++i) {
            accepted += AddPoint(points[i]) ? 1U : 0U;
        }
        return accepted;
    }

    NeighborSet Query(const Point3f& query) const {
        return QueryImpl(query, config_.preexpand_neighborhoods);
    }

    // Diagnostic path used to prove that the insertion-time neighborhood index
    // preserves the original hash-probe result on real estimator workloads.
    NeighborSet QueryDirect(const Point3f& query) const {
        return QueryImpl(query, false);
    }

    void Query(const Point3f* queries, std::size_t count, NeighborSet* results) const {
        if ((queries == nullptr || results == nullptr) && count != 0) {
            throw std::invalid_argument("RepresentativeIvox::Query received a null pointer");
        }
        for (std::size_t i = 0; i < count; ++i) {
            results[i] = Query(queries[i]);
        }
    }

    std::size_t NumVoxels() const { return voxels_.size(); }
    std::size_t RejectedInsertions() const { return rejected_insertions_; }
    std::size_t NumCachedQueryVoxels() const { return query_neighborhoods_.size(); }
    std::size_t CachedVoxelReferences() const { return cached_voxel_references_; }
    std::size_t EstimatedNeighborhoodCachePayloadBytes() const {
        if (query_neighborhoods_.empty()) return 0U;
        std::size_t bytes = query_neighborhoods_.bucket_count() * sizeof(void*);
        bytes += query_neighborhoods_.size() *
                 (sizeof(VoxelKey) + sizeof(CachedNeighborhood));
        for (const auto& entry : query_neighborhoods_) {
            bytes += entry.second.voxels.capacity() * sizeof(const Voxel*);
        }
        return bytes;
    }
    const RepresentativeIvoxConfig& config() const { return config_; }

    static void ValidateConfig(const RepresentativeIvoxConfig& config) {
        if (!(config.resolution > 0.0f) || !std::isfinite(config.resolution)) {
            throw std::invalid_argument("RepresentativeIvox resolution must be finite and positive");
        }
        if (config.max_points_per_voxel < 1 ||
            config.max_points_per_voxel > kRepresentativeIvoxMaxPointsPerVoxel) {
            throw std::invalid_argument("RepresentativeIvox max_points_per_voxel must be in [1, 8]");
        }
        if (config.capacity == 0) {
            throw std::invalid_argument("RepresentativeIvox capacity must be positive");
        }
        if (!(config.max_range > 0.0f) || !std::isfinite(config.max_range)) {
            throw std::invalid_argument("RepresentativeIvox max_range must be finite and positive");
        }
        const int nearby = static_cast<int>(config.nearby_type);
        if (nearby != 0 && nearby != 6 && nearby != 18 && nearby != 26) {
            throw std::invalid_argument("RepresentativeIvox nearby_type must be 0, 6, 18, or 26");
        }
    }

private:
    struct Voxel {
        std::vector<Point3f> points;
    };

    struct CachedNeighborhood {
        std::vector<const Voxel*> voxels;
    };

    NeighborSet QueryImpl(const Point3f& query, bool use_preexpanded_neighborhoods) const {
        NeighborSet result;
        if (!IsFinite(query)) {
            return result;
        }

        VoxelKey center;
        if (!PositionToKey(query, center)) {
            return result;
        }
        const float max_squared_range = config_.max_range * config_.max_range;
        const auto inspect_voxel = [&](const Voxel& voxel) {
            for (const Point3f& point : voxel.points) {
                const float distance = SquaredDistance(point, query);
                if (distance < max_squared_range) {
                    InsertNeighbor(result, point, distance);
                }
            }
        };

        if (use_preexpanded_neighborhoods) {
            const auto cached = query_neighborhoods_.find(center);
            if (cached != query_neighborhoods_.end()) {
                for (const Voxel* voxel : cached->second.voxels) {
                    inspect_voxel(*voxel);
                }
                return result;
            }
        }

        VisitNeighborOffsets([&](int dx, int dy, int dz) {
            VoxelKey key;
            if (!OffsetKey(center, dx, dy, dz, key)) return;
            const auto iter = voxels_.find(key);
            if (iter != voxels_.end()) inspect_voxel(iter->second);
        });
        return result;
    }

    static bool SamePoint(const Point3f& lhs, const Point3f& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    static bool PointLess(const Point3f& lhs, const Point3f& rhs) {
        if (lhs.x != rhs.x) return lhs.x < rhs.x;
        if (lhs.y != rhs.y) return lhs.y < rhs.y;
        return lhs.z < rhs.z;
    }

    static bool CandidateLess(float lhs_distance, const Point3f& lhs,
                              float rhs_distance, const Point3f& rhs) {
        if (lhs_distance != rhs_distance) return lhs_distance < rhs_distance;
        return PointLess(lhs, rhs);
    }

    static void InsertNeighbor(NeighborSet& result, const Point3f& point, float distance) {
        int insert_at = result.count;
        if (result.count == kRepresentativeIvoxMaxNeighbors) {
            const int last = kRepresentativeIvoxMaxNeighbors - 1;
            if (!CandidateLess(distance, point, result.squared_distances[last],
                               result.points[last])) {
                return;
            }
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

    bool PositionToKey(const Point3f& point, VoxelKey& key) const {
        std::int32_t x = 0;
        std::int32_t y = 0;
        std::int32_t z = 0;
        if (!FloorToInt(point.x * inv_resolution_, x) ||
            !FloorToInt(point.y * inv_resolution_, y) ||
            !FloorToInt(point.z * inv_resolution_, z)) {
            return false;
        }
        key = VoxelKey{x, y, z};
        return true;
    }

    static bool FloorToInt(float value, std::int32_t& result) {
        const double floored = std::floor(static_cast<double>(value));
        if (floored < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
            floored > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
            return false;
        }
        result = static_cast<std::int32_t>(floored);
        return true;
    }

    static bool OffsetKey(const VoxelKey& key, int dx, int dy, int dz, VoxelKey& result) {
        const std::int64_t x = static_cast<std::int64_t>(key.x) + dx;
        const std::int64_t y = static_cast<std::int64_t>(key.y) + dy;
        const std::int64_t z = static_cast<std::int64_t>(key.z) + dz;
        if (x < std::numeric_limits<std::int32_t>::min() ||
            x > std::numeric_limits<std::int32_t>::max() ||
            y < std::numeric_limits<std::int32_t>::min() ||
            y > std::numeric_limits<std::int32_t>::max() ||
            z < std::numeric_limits<std::int32_t>::min() ||
            z > std::numeric_limits<std::int32_t>::max()) {
            return false;
        }
        result = VoxelKey{static_cast<std::int32_t>(x),
                          static_cast<std::int32_t>(y),
                          static_cast<std::int32_t>(z)};
        return true;
    }

    void RegisterVoxelWithQueryNeighborhoods(const VoxelKey& home_key, const Voxel* voxel) {
        if (!config_.preexpand_neighborhoods) return;

        // Existing occupied query voxels that can see the new home voxel only
        // need one appended pointer.
        VisitNeighborOffsets([&](int dx, int dy, int dz) {
            VoxelKey query_key;
            if (!OffsetKey(home_key, -dx, -dy, -dz, query_key)) return;
            const auto cached = query_neighborhoods_.find(query_key);
            if (cached != query_neighborhoods_.end()) {
                cached->second.voxels.push_back(voxel);
                ++cached_voxel_references_;
            }
        });

        // A cache entry is created only when its query voxel itself becomes
        // occupied. Empty query voxels fall back to the direct 0/6/18/26 probe,
        // avoiding the up-to-27x key expansion of a fully materialized index.
        CachedNeighborhood neighborhood;
        VisitNeighborOffsets([&](int dx, int dy, int dz) {
            VoxelKey visible_home_key;
            if (!OffsetKey(home_key, dx, dy, dz, visible_home_key)) return;
            const auto visible = voxels_.find(visible_home_key);
            if (visible != voxels_.end()) neighborhood.voxels.push_back(&visible->second);
        });
        cached_voxel_references_ += neighborhood.voxels.size();
        query_neighborhoods_.emplace(home_key, std::move(neighborhood));
    }

    float RepresentativeScore(const VoxelKey& key, const Point3f& point) const {
        const Point3f center{(static_cast<float>(key.x) + 0.5f) * config_.resolution,
                             (static_cast<float>(key.y) + 0.5f) * config_.resolution,
                             (static_cast<float>(key.z) + 0.5f) * config_.resolution};
        return SquaredDistance(point, center);
    }

    void SortRepresentatives(const VoxelKey& key, std::vector<Point3f>& points) const {
        std::sort(points.begin(), points.end(), [&](const Point3f& lhs, const Point3f& rhs) {
            const float lhs_score = RepresentativeScore(key, lhs);
            const float rhs_score = RepresentativeScore(key, rhs);
            if (lhs_score != rhs_score) {
                return lhs_score < rhs_score;
            }
            return PointLess(lhs, rhs);
        });
    }

    template <typename Visitor>
    void VisitNeighborOffsets(Visitor&& visitor) const {
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
                    const bool include =
                        config_.nearby_type == RepresentativeNearbyType::NEARBY26 ||
                        (config_.nearby_type == RepresentativeNearbyType::NEARBY18 && manhattan <= 2) ||
                        (config_.nearby_type == RepresentativeNearbyType::NEARBY6 && manhattan <= 1) ||
                        (config_.nearby_type == RepresentativeNearbyType::CENTER && manhattan == 0);
                    if (include) {
                        visitor(dx, dy, dz);
                    }
                }
            }
        }
    }

    RepresentativeIvoxConfig config_;
    float inv_resolution_ = 1.0f;
    std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;
    std::unordered_map<VoxelKey, CachedNeighborhood, VoxelKeyHash> query_neighborhoods_;
    std::size_t rejected_insertions_ = 0;
    std::size_t cached_voxel_references_ = 0;
};

}  // namespace batchlio
