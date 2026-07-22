#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "ivox/representative_ivox.h"

namespace batchlio {

struct CudaRepresentativeIvoxStats {
    std::uint64_t voxel_count = 0;
    std::uint64_t accepted_points = 0;
    std::uint64_t rejected_points = 0;
    float last_insert_kernel_ms = 0.0f;
    float last_query_kernel_ms = 0.0f;
    float last_query_end_to_end_ms = 0.0f;
    bool persistent_query_service = false;
};

// Optional CUDA implementation of the bounded representative map. Its public
// interface remains available in CPU-only builds; those builds link a stub that
// reports IsCompiled() == false rather than making CUDA a hard dependency.
class CudaRepresentativeIvox {
public:
    explicit CudaRepresentativeIvox(RepresentativeIvoxConfig config);
    ~CudaRepresentativeIvox();

    CudaRepresentativeIvox(const CudaRepresentativeIvox&) = delete;
    CudaRepresentativeIvox& operator=(const CudaRepresentativeIvox&) = delete;
    CudaRepresentativeIvox(CudaRepresentativeIvox&&) noexcept;
    CudaRepresentativeIvox& operator=(CudaRepresentativeIvox&&) noexcept;

    static bool IsCompiled();
    static bool RuntimeAvailable(std::string* reason = nullptr);

    bool IsReady() const;
    const std::string& LastError() const;
    const RepresentativeIvoxConfig& config() const;

    bool Reset();
    bool AddPoints(const Point3f* points, std::size_t count);
    bool Query(const Point3f* queries, std::size_t count, NeighborSet* results);
    bool StartPersistentQueryService(std::size_t max_query_points);
    void StopPersistentQueryService();
    bool PersistentQueryServiceReady() const;
    CudaRepresentativeIvoxStats Stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace batchlio
