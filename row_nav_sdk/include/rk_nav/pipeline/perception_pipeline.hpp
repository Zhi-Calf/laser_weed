#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "rk_nav/core/result.hpp"
#include "rk_nav/core/status.hpp"
#include "rk_nav/geometry/bev_projector.hpp"
#include "rk_nav/geometry/centerline_fitter.hpp"
#include "rk_nav/inference/segmenter.hpp"
#include "rk_nav/perception/row_extractor.hpp"
#include "rk_nav/perception/row_temporal_tracker.hpp"

namespace rk_nav {

struct PerceptionPipelineConfig {
    float minimum_navigation_confidence{0.35F};
};

struct PerceptionOutput {
    NavigationObservation navigation;
    cv::Mat bev_mask;
    std::optional<RowExtraction> rows;
    std::optional<CenterlineFit> centerline;
};

class PerceptionPipeline {
public:
    PerceptionPipeline(BevProjector projector, RowExtractor row_extractor,
                       CenterlineFitter centerline_fitter,
                       PerceptionPipelineConfig config = {});

    Result<PerceptionOutput> process(const SegmentationResult& segmentation,
                                     double upstream_latency_ms = 0.0,
                                     const BevProjector* frame_projector = nullptr) const;

private:
    BevProjector projector_;
    RowExtractor row_extractor_;
    CenterlineFitter centerline_fitter_;
    PerceptionPipelineConfig config_;
};

// Stateful wrapper; serial calls per instance. Stateless pipeline remains usable for images/A-B tests.
class TrackedPerceptionPipeline {
public:
    TrackedPerceptionPipeline(PerceptionPipeline pipeline, TemporalTrackerConfig config = {})
        : pipeline_(std::move(pipeline)), tracker_(config) {}
    // Optional absolute steady-clock deadline. Expired work never commits tracker state.
    Result<PerceptionOutput> process(const SegmentationResult& segmentation,
        double upstream_latency_ms = 0.0, std::uint64_t deadline_ns = 0,
        const BevProjector* frame_projector = nullptr);
    void resetTracking() { tracker_.reset(); }
private:
    PerceptionPipeline pipeline_;
    RowTemporalTracker tracker_;
};

}  // namespace rk_nav
