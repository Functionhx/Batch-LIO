#include <gtest/gtest.h>

#include "ivox/representative_ivox.h"

#include <array>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

using batchlio::NeighborSet;
using batchlio::Point3f;
using batchlio::RepresentativeIvox;
using batchlio::RepresentativeIvoxConfig;
using batchlio::RepresentativeNearbyType;

void ExpectSameNeighbors(const NeighborSet& direct, const NeighborSet& cached) {
    ASSERT_EQ(cached.count, direct.count);
    for (int i = 0; i < direct.count; ++i) {
        EXPECT_FLOAT_EQ(cached.points[i].x, direct.points[i].x);
        EXPECT_FLOAT_EQ(cached.points[i].y, direct.points[i].y);
        EXPECT_FLOAT_EQ(cached.points[i].z, direct.points[i].z);
        EXPECT_FLOAT_EQ(cached.squared_distances[i], direct.squared_distances[i]);
    }
}

TEST(RepresentativeIvox, KeepsCenterNearestRealPoints) {
    RepresentativeIvoxConfig config;
    config.resolution = 1.0f;
    config.max_points_per_voxel = 2;
    config.nearby_type = RepresentativeNearbyType::CENTER;
    RepresentativeIvox map(config);

    const std::array<Point3f, 4> input{{
        {0.01f, 0.01f, 0.01f},
        {0.49f, 0.49f, 0.49f},
        {0.60f, 0.50f, 0.50f},
        {0.45f, 0.50f, 0.50f},
    }};
    map.AddPoints(input.data(), input.size());

    const NeighborSet result = map.Query(Point3f{0.5f, 0.5f, 0.5f});
    ASSERT_EQ(result.count, 2);
    EXPECT_FLOAT_EQ(result.points[0].x, 0.49f);
    EXPECT_FLOAT_EQ(result.points[1].x, 0.45f);
    EXPECT_EQ(map.NumVoxels(), 1U);
}

TEST(RepresentativeIvox, SignedKeysDoNotAlias) {
    RepresentativeIvoxConfig config;
    config.resolution = 1.0f;
    config.max_points_per_voxel = 1;
    config.nearby_type = RepresentativeNearbyType::CENTER;
    RepresentativeIvox map(config);

    EXPECT_TRUE(map.AddPoint(Point3f{-0.25f, 0.0f, 0.0f}));
    EXPECT_TRUE(map.AddPoint(Point3f{65535.25f, 0.0f, 0.0f}));
    EXPECT_EQ(map.NumVoxels(), 2U);

    const NeighborSet negative = map.Query(Point3f{-0.25f, 0.0f, 0.0f});
    const NeighborSet positive = map.Query(Point3f{65535.25f, 0.0f, 0.0f});
    ASSERT_EQ(negative.count, 1);
    ASSERT_EQ(positive.count, 1);
    EXPECT_FLOAT_EQ(negative.points[0].x, -0.25f);
    EXPECT_FLOAT_EQ(positive.points[0].x, 65535.25f);
}

TEST(RepresentativeIvox, NearbySixIncludesAxesButNotEdges) {
    RepresentativeIvoxConfig config;
    config.resolution = 1.0f;
    config.max_points_per_voxel = 1;
    config.nearby_type = RepresentativeNearbyType::NEARBY6;
    config.max_range = 10.0f;
    RepresentativeIvox map(config);

    EXPECT_TRUE(map.AddPoint(Point3f{1.1f, 0.1f, 0.1f}));
    EXPECT_TRUE(map.AddPoint(Point3f{1.1f, 1.1f, 0.1f}));
    const NeighborSet result = map.Query(Point3f{0.1f, 0.1f, 0.1f});
    ASSERT_EQ(result.count, 1);
    EXPECT_FLOAT_EQ(result.points[0].y, 0.1f);
}

TEST(RepresentativeIvox, PreexpandedCacheFallsBackForAnEmptyQueryVoxel) {
    RepresentativeIvoxConfig config;
    config.resolution = 1.0f;
    config.max_points_per_voxel = 1;
    config.nearby_type = RepresentativeNearbyType::NEARBY6;
    config.max_range = 10.0f;
    config.preexpand_neighborhoods = true;
    RepresentativeIvox map(config);

    EXPECT_TRUE(map.AddPoint(Point3f{1.1f, 0.1f, 0.1f}));
    EXPECT_EQ(map.NumCachedQueryVoxels(), 1U);
    const NeighborSet result = map.Query(Point3f{0.1f, 0.1f, 0.1f});
    ASSERT_EQ(result.count, 1);
    EXPECT_FLOAT_EQ(result.points[0].x, 1.1f);
    ExpectSameNeighbors(map.QueryDirect(Point3f{0.1f, 0.1f, 0.1f}), result);
}

TEST(RepresentativeIvox, CapacityAndInvalidPointsAreReported) {
    RepresentativeIvoxConfig config;
    config.resolution = 1.0f;
    config.capacity = 1;
    RepresentativeIvox map(config);

    EXPECT_TRUE(map.AddPoint(Point3f{0.1f, 0.1f, 0.1f}));
    EXPECT_FALSE(map.AddPoint(Point3f{2.1f, 0.1f, 0.1f}));
    EXPECT_FALSE(map.AddPoint(Point3f{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}));
    EXPECT_FALSE(map.AddPoint(Point3f{std::numeric_limits<float>::max(), 0.0f, 0.0f}));
    EXPECT_EQ(map.Query(Point3f{std::numeric_limits<float>::max(), 0.0f, 0.0f}).count, 0);
    EXPECT_EQ(map.RejectedInsertions(), 3U);
}

TEST(RepresentativeIvox, RejectsInvalidConfiguration) {
    RepresentativeIvoxConfig config;
    config.max_points_per_voxel = 9;
    EXPECT_THROW(RepresentativeIvox map(config), std::invalid_argument);
}

TEST(RepresentativeIvox, PreexpandedNeighborhoodsMatchDirectLookupIncrementally) {
    std::mt19937 random(0xF2A110U);
    std::uniform_real_distribution<float> map_distribution(-12.0f, 12.0f);
    std::uniform_real_distribution<float> query_distribution(-11.0f, 11.0f);

    for (const RepresentativeNearbyType nearby : {
             RepresentativeNearbyType::CENTER,
             RepresentativeNearbyType::NEARBY6,
             RepresentativeNearbyType::NEARBY18,
             RepresentativeNearbyType::NEARBY26}) {
        RepresentativeIvoxConfig direct_config;
        direct_config.resolution = 0.5f;
        direct_config.max_points_per_voxel = 4;
        direct_config.nearby_type = nearby;
        direct_config.capacity = 32768;
        direct_config.max_range = 5.0f;

        RepresentativeIvoxConfig cached_config = direct_config;
        cached_config.preexpand_neighborhoods = true;
        RepresentativeIvox direct(direct_config);
        RepresentativeIvox cached(cached_config);

        std::vector<Point3f> points(6000);
        for (Point3f& point : points) {
            point = Point3f{map_distribution(random), map_distribution(random),
                            map_distribution(random)};
        }
        // Repeated updates in one voxel exercise representative replacement;
        // cached neighborhoods must observe the authoritative vector in place.
        for (int i = 0; i < 40; ++i) {
            const float coordinate = 0.01f + static_cast<float>(i) * 0.01f;
            points.push_back(Point3f{coordinate, coordinate, coordinate});
        }

        const std::size_t split = points.size() / 2U;
        const std::array<std::pair<std::size_t, std::size_t>, 2> phases{{
            {0U, split}, {split, points.size()}}};
        for (const auto& phase : phases) {
            const std::size_t count = phase.second - phase.first;
            EXPECT_EQ(cached.AddPoints(points.data() + phase.first, count),
                      direct.AddPoints(points.data() + phase.first, count));
            EXPECT_EQ(cached.NumVoxels(), direct.NumVoxels());

            for (int i = 0; i < 2000; ++i) {
                const Point3f query{query_distribution(random), query_distribution(random),
                                    query_distribution(random)};
                ExpectSameNeighbors(direct.Query(query), cached.Query(query));
            }
        }

        EXPECT_GT(cached.NumCachedQueryVoxels(), 0U);
        EXPECT_GE(cached.CachedVoxelReferences(), cached.NumVoxels());
        direct.Reset();
        cached.Reset();
        EXPECT_EQ(cached.NumCachedQueryVoxels(), 0U);
        EXPECT_EQ(cached.CachedVoxelReferences(), 0U);
        ExpectSameNeighbors(direct.Query(Point3f{}), cached.Query(Point3f{}));
    }
}

}  // namespace
