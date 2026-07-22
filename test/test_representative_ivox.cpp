#include <gtest/gtest.h>

#include "ivox/representative_ivox.h"

#include <array>
#include <limits>
#include <stdexcept>

namespace {

using batchlio::NeighborSet;
using batchlio::Point3f;
using batchlio::RepresentativeIvox;
using batchlio::RepresentativeIvoxConfig;
using batchlio::RepresentativeNearbyType;

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

}  // namespace
