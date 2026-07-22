#include "cuda/cuda_representative_ivox.h"

#include <utility>

namespace batchlio {

class CudaRepresentativeIvox::Impl {
public:
    explicit Impl(RepresentativeIvoxConfig value) : config(std::move(value)) {
        RepresentativeIvox::ValidateConfig(config);
    }

    RepresentativeIvoxConfig config;
    std::string error = "Batch-LIO was built without BATCH_LIO_ENABLE_CUDA";
};

CudaRepresentativeIvox::CudaRepresentativeIvox(RepresentativeIvoxConfig config)
    : impl_(new Impl(std::move(config))) {}

CudaRepresentativeIvox::~CudaRepresentativeIvox() = default;
CudaRepresentativeIvox::CudaRepresentativeIvox(CudaRepresentativeIvox&&) noexcept = default;
CudaRepresentativeIvox& CudaRepresentativeIvox::operator=(CudaRepresentativeIvox&&) noexcept = default;

bool CudaRepresentativeIvox::IsCompiled() { return false; }

bool CudaRepresentativeIvox::RuntimeAvailable(std::string* reason) {
    if (reason != nullptr) {
        *reason = "Batch-LIO was built without BATCH_LIO_ENABLE_CUDA";
    }
    return false;
}

bool CudaRepresentativeIvox::IsReady() const { return false; }
const std::string& CudaRepresentativeIvox::LastError() const { return impl_->error; }
const RepresentativeIvoxConfig& CudaRepresentativeIvox::config() const { return impl_->config; }
bool CudaRepresentativeIvox::Reset() { return false; }
bool CudaRepresentativeIvox::AddPoints(const Point3f*, std::size_t) { return false; }
bool CudaRepresentativeIvox::Query(const Point3f*, std::size_t, NeighborSet*) { return false; }
bool CudaRepresentativeIvox::StartPersistentQueryService(std::size_t) { return false; }
void CudaRepresentativeIvox::StopPersistentQueryService() {}
bool CudaRepresentativeIvox::PersistentQueryServiceReady() const { return false; }
CudaRepresentativeIvoxStats CudaRepresentativeIvox::Stats() const { return {}; }

}  // namespace batchlio
