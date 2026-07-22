// Batch-LIO v2 — Phase 0 stage profiler (definitions).
#include "stage_profiler.h"

#include <cstdio>

namespace batchlio {

StageProfiler g_profiler;

void StageProfiler::report(const char* tag) const {
    if (stages.empty()) return;
    std::printf("[PROFILE] tag=%s frames=%llu omp_threads=%d avg_pts=%.1f avg_batches=%.1f avg_residuals=%.1f\n",
                tag,
                static_cast<unsigned long long>(frames), omp_threads,
                frames ? points_total / static_cast<double>(frames) : 0.0,
                frames ? batches_total / static_cast<double>(frames) : 0.0,
                frames ? residuals_total / static_cast<double>(frames) : 0.0);
    for (const auto& [name, st] : stages) {
        std::printf("[PROFILE] tag=%s stage=%s count=%llu mean_ms=%.4f p50_ms=%.4f p90_ms=%.4f p95_ms=%.4f p99_ms=%.4f max_ms=%.4f\n",
                    tag, name.c_str(), static_cast<unsigned long long>(st.count),
                    st.mean(), st.percentile(0.50), st.percentile(0.90),
                    st.percentile(0.95), st.percentile(0.99), st.max_ms);
    }
    std::fflush(stdout);
}

}  // namespace batchlio
