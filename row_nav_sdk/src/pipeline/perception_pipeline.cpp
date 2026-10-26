#include "rk_nav/pipeline/perception_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace rk_nav {
namespace {

float segmentationConfidence(const SegmentationResult& segmentation) {
    float confidence = 0.0F;
    for (const auto& instance : segmentation.instances) {
        confidence = std::max(confidence, instance.confidence);
    }
    return std::clamp(confidence, 0.0F, 1.0F);
}

}  // namespace

PerceptionPipeline::PerceptionPipeline(BevProjector projector, RowExtractor row_extractor,
                                       CenterlineFitter centerline_fitter,
                                       PerceptionPipelineConfig config)
    : projector_(std::move(projector)),
      row_extractor_(std::move(row_extractor)),
      centerline_fitter_(std::move(centerline_fitter)),
      config_(std::move(config)) {}

Result<PerceptionOutput> PerceptionPipeline::process(const SegmentationResult& segmentation,
                                                     double upstream_latency_ms,
                                                     const BevProjector* frame_projector) const {
    if (!std::isfinite(upstream_latency_ms) || upstream_latency_ms < 0.0) {
        return Status::invalidArgument("upstream latency must be finite and non-negative");
    }
    if (!std::isfinite(config_.minimum_navigation_confidence) ||
        config_.minimum_navigation_confidence < 0.0F ||
        config_.minimum_navigation_confidence > 1.0F) {
        return Status::invalidArgument("minimum navigation confidence must be within [0,1]");
    }

    const auto started = std::chrono::steady_clock::now();
    PerceptionOutput output;
    output.navigation.sequence = segmentation.sequence;
    output.navigation.capture_time_ns = segmentation.capture_time_ns;
    output.navigation.lookahead_m = centerline_fitter_.config().lookahead_m;
    output.navigation.status = TrackingStatus::kNoRow;

    const auto finish = [&](PerceptionOutput value) -> Result<PerceptionOutput> {
        const auto elapsed = std::chrono::steady_clock::now() - started;
        value.navigation.latency_ms = upstream_latency_ms +
            std::chrono::duration<double, std::milli>(elapsed).count();
        return value;
    };

    if (segmentation.combined_mask.empty() || segmentation.instances.empty()) {
        return finish(std::move(output));
    }
    const auto& projector = frame_projector ? *frame_projector : projector_;
    const auto projected = projector.projectMask(segmentation.combined_mask);
    if (!projected.isOk()) return projected.status();
    output.bev_mask = projected.value();

    const auto extracted = row_extractor_.extractBetweenRows(output.bev_mask, projector);
    if (!extracted.isOk()) {
        if (extracted.status().code() == StatusCode::kUnavailable) {
            return finish(std::move(output));
        }
        return extracted.status();
    }
    output.rows = extracted.value();
    if (!output.rows->navigation_pair_valid) return finish(std::move(output));

    const auto fitted = centerline_fitter_.fit(output.rows->centerline_ground_points);
    if (!fitted.isOk()) {
        if (fitted.status().code() == StatusCode::kUnavailable) {
            return finish(std::move(output));
        }
        return fitted.status();
    }
    output.centerline = fitted.value();

    const float confidence = std::clamp(segmentationConfidence(segmentation) *
                                            output.rows->confidence * output.centerline->confidence,
                                        0.0F, 1.0F);
    output.navigation.lateral_error_m = output.centerline->lateral_error_m;
    output.navigation.heading_error_rad = output.centerline->heading_error_rad;
    output.navigation.confidence = confidence;
    output.navigation.status = confidence >= config_.minimum_navigation_confidence
                                   ? TrackingStatus::kValid
                                   : TrackingStatus::kLowConfidence;
    return finish(std::move(output));
}

}  // namespace rk_nav
