#include "ivox/representative_ivox.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

using batchlio::NeighborSet;
using batchlio::Point3f;
using batchlio::RepresentativeIvox;
using batchlio::RepresentativeIvoxConfig;
using batchlio::RepresentativeNearbyType;

template <typename Function>
double MeanMicroseconds(int repetitions, Function&& function) {
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < repetitions; ++i) function();
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::micro>(end - begin).count() /
           static_cast<double>(repetitions);
}

bool SameResult(const NeighborSet& lhs, const NeighborSet& rhs) {
    if (lhs.count != rhs.count) return false;
    for (int i = 0; i < lhs.count; ++i) {
        if (lhs.points[i].x != rhs.points[i].x ||
            lhs.points[i].y != rhs.points[i].y ||
            lhs.points[i].z != rhs.points[i].z ||
            lhs.squared_distances[i] != rhs.squared_distances[i]) {
            return false;
        }
    }
    return true;
}

std::vector<Point3f> MakeSurfacePoints(std::size_t count, std::mt19937& random) {
    std::uniform_real_distribution<float> xy(-50.0f, 50.0f);
    std::normal_distribution<float> noise(0.0f, 0.03f);
    std::vector<Point3f> points;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float x = xy(random);
        const float y = xy(random);
        // Mix a ground-like surface and two facades to resemble local LiDAR maps.
        switch (i % 3U) {
            case 0:
                points.push_back(Point3f{x, y, 0.4f * std::sin(x * 0.08f) + noise(random)});
                break;
            case 1:
                points.push_back(Point3f{x, -20.0f + noise(random), 0.08f * y});
                break;
            default:
                points.push_back(Point3f{25.0f + noise(random), x, 0.08f * y});
                break;
        }
    }
    return points;
}

}  // namespace

int main(int argc, char** argv) {
    const std::size_t map_point_count =
        argc > 1 ? static_cast<std::size_t>(std::strtoull(argv[1], nullptr, 10)) : 120000U;
    std::mt19937 random(0xF2A110U);
    std::vector<Point3f> map_points = MakeSurfacePoints(map_point_count, random);
    std::vector<Point3f> queries = MakeSurfacePoints(4096U, random);

    RepresentativeIvoxConfig direct_config;
    direct_config.resolution = 0.5f;
    direct_config.max_points_per_voxel = 4;
    direct_config.nearby_type = RepresentativeNearbyType::NEARBY18;
    direct_config.capacity = 262144;
    direct_config.max_range = 5.0f;
    RepresentativeIvoxConfig cached_config = direct_config;
    cached_config.preexpand_neighborhoods = true;

    RepresentativeIvox direct(direct_config);
    RepresentativeIvox cached(cached_config);
    const double direct_insert_us = MeanMicroseconds(1, [&] {
        direct.AddPoints(map_points.data(), map_points.size());
    });
    const double cached_insert_us = MeanMicroseconds(1, [&] {
        cached.AddPoints(map_points.data(), map_points.size());
    });

    std::vector<NeighborSet> direct_results(queries.size());
    std::vector<NeighborSet> cached_results(queries.size());
    direct.Query(queries.data(), queries.size(), direct_results.data());
    cached.Query(queries.data(), queries.size(), cached_results.data());
    for (std::size_t i = 0; i < queries.size(); ++i) {
        if (!SameResult(direct_results[i], cached_results[i])) {
            std::cerr << "preexpanded result mismatch at query " << i << '\n';
            return 1;
        }
    }

    std::cout << "map_points=" << map_points.size()
              << " voxels=" << direct.NumVoxels()
              << " cached_query_voxels=" << cached.NumCachedQueryVoxels()
              << " cached_voxel_refs=" << cached.CachedVoxelReferences()
              << " cache_payload_mib="
              << cached.EstimatedNeighborhoodCachePayloadBytes() / (1024.0 * 1024.0) << '\n';
    std::cout << "insert_direct_ms=" << direct_insert_us / 1000.0
              << " insert_preexpanded_ms=" << cached_insert_us / 1000.0 << '\n';
    std::cout << "batch,direct_us,preexpanded_us,speedup\n";

    std::uint64_t checksum = 0;
    for (const std::size_t batch : {8U, 16U, 32U, 64U, 128U, 512U, 1024U, 2048U, 4096U}) {
        const int repetitions = batch <= 128U ? 1000 : 200;
        for (int warmup = 0; warmup < 10; ++warmup) {
            direct.Query(queries.data(), batch, direct_results.data());
            cached.Query(queries.data(), batch, cached_results.data());
        }
        const double direct_us = MeanMicroseconds(repetitions, [&] {
            direct.Query(queries.data(), batch, direct_results.data());
            checksum += static_cast<std::uint64_t>(direct_results[0].count);
        });
        const double cached_us = MeanMicroseconds(repetitions, [&] {
            cached.Query(queries.data(), batch, cached_results.data());
            checksum += static_cast<std::uint64_t>(cached_results[0].count);
        });
        std::cout << batch << ',' << std::fixed << std::setprecision(3)
                  << direct_us << ',' << cached_us << ',' << direct_us / cached_us << '\n';
    }
    std::cerr << "checksum=" << checksum << '\n';
    return 0;
}
