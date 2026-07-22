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
    }

    void Reset() {
        voxels_.clear();
        rejected_insertions_ = 0;
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
        if (iter == voxels_.end()) {
            if (voxels_.size() >= config_.capacity) {
                ++rejected_insertions_;
                return false;
            }
            Voxel voxel;
            voxel.points.reserve(static_cast<std::size_t>(config_.max_points_per_voxel));
            iter = voxels_.emplace(key, std::move(voxel)).first;
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
        return std::find_if(points.begin(), points.end(), [&](const Point3f& existing) {
                   return SamePoint(existing, point);
               }) != points.end();
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
        NeighborSet result;
        if (!IsFinite(query)) {
            return result;
        }

        VoxelKey center;
        if (!PositionToKey(query, center)) {
            return result;
        }
        const float max_squared_range = config_.max_range * config_.max_range;
        VisitNeighborOffsets([&](int dx, int dy, int dz) {
            const std::int64_t key_x = static_cast<std::int64_t>(center.x) + dx;
            const std::int64_t key_y = static_cast<std::int64_t>(center.y) + dy;
            const std::int64_t key_z = static_cast<std::int64_t>(center.z) + dz;
            if (key_x < std::numeric_limits<std::int32_t>::min() ||
                key_x > std::numeric_limits<std::int32_t>::max() ||
                key_y < std::numeric_limits<std::int32_t>::min() ||
                key_y > std::numeric_limits<std::int32_t>::max() ||
                key_z < std::numeric_limits<std::int32_t>::min() ||
                key_z > std::numeric_limits<std::int32_t>::max()) {
                return;
            }
            const VoxelKey key{static_cast<std::int32_t>(key_x),
                               static_cast<std::int32_t>(key_y),
                               static_cast<std::int32_t>(key_z)};
            const auto iter = voxels_.find(key);
            if (iter == voxels_.end()) {
                return;
            }
            for (const Point3f& point : iter->second.points) {
                const float distance = SquaredDistance(point, query);
                if (distance < max_squared_range) {
                    InsertNeighbor(result, point, distance);
                }
            }
        });
        return result;
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
    std::size_t rejected_insertions_ = 0;
};

}  // namespace batchlio
