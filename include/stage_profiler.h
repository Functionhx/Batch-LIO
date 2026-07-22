// Batch-LIO v2 — Phase 0 stage profiler.
// Lightweight in-memory per-stage timing (mean / P50 / P90 / P95 / P99 / max) plus
// per-frame counters (points, batches, valid residuals). Gated by the `profiling_enable`
// ROS param so it is a no-op in normal runs. Output is machine-readable [PROFILE] lines.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <omp.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace batchlio {

struct StageStats {
    std::vector<double> samples;  // ms per call
    double total_ms = 0.0;
    double max_ms = 0.0;
    std::uint64_t count = 0;

    inline void add(double ms) {
        samples.push_back(ms);
        total_ms += ms;
        if (ms > max_ms) max_ms = ms;
        ++count;
    }
    inline double mean() const { return count ? total_ms / static_cast<double>(count) : 0.0; }
    inline double percentile(double p) const {
        if (samples.empty()) return 0.0;
        std::vector<double> s = samples;
        std::sort(s.begin(), s.end());
        size_t idx = static_cast<size_t>(p * static_cast<double>(s.size() - 1));
        if (idx >= s.size()) idx = s.size() - 1;
        return s[idx];
    }
};

class StageProfiler {
public:
    std::unordered_map<std::string, StageStats> stages;
    std::uint64_t frames = 0;
    double points_total = 0;
    double batches_total = 0;
    double residuals_total = 0;
    int omp_threads = 1;

    inline void add(const std::string& name, double ms) { stages[name].add(ms); }
    inline void count_frame(int n_points, int n_batches, int n_residuals) {
        ++frames;
        points_total += n_points;
        batches_total += n_batches;
        residuals_total += n_residuals;
    }
    inline void reset() {
        stages.clear();
        frames = 0; points_total = 0; batches_total = 0; residuals_total = 0;
    }
    void report(const char* tag) const;
};

// RAII scoped timer. Use: { ScopedStage s(g_profiler, "knn_search"); ... }
struct ScopedStage {
    StageProfiler& p;
    std::string name;
    double t0;
    ScopedStage(StageProfiler& p, const std::string& name) : p(p), name(name), t0(omp_get_wtime()) {}
    ~ScopedStage() { p.add(name, (omp_get_wtime() - t0) * 1e3); }
};

extern StageProfiler g_profiler;

// Per-frame accumulator for stages reached deep inside the EKF update (h_model_output) or
// per-window (deskew). Reset at frame start; read at frame end.
struct FrameAccumulator {
    double measurement_build_ms = 0.0;  // h_model_output wall (parallel KNN+plane-fit + serial Jacobian)
    double deskew_ms = 0.0;             // intra-window de-skew wall
    inline void reset() { measurement_build_ms = 0.0; deskew_ms = 0.0; }
};
extern FrameAccumulator g_frame_acc;

// RAII: adds elapsed wall-ms to `target` on scope exit (only if active). Handles early returns.
struct FrameAccumScope {
    bool active;
    double& target;
    double t0;
    FrameAccumScope(bool active, double& target) : active(active), target(target), t0(omp_get_wtime()) {}
    ~FrameAccumScope() { if (active) target += (omp_get_wtime() - t0) * 1e3; }
};

}  // namespace batchlio
