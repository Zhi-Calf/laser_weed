#include "rk_nav/pipeline/perception_pipeline.hpp"
#include <chrono>
#include <limits>

namespace rk_nav {
Result<PerceptionOutput> TrackedPerceptionPipeline::process(const SegmentationResult& segmentation,
    double upstream_latency_ms,std::uint64_t deadline_ns,const BevProjector* frame_projector) {
    const auto valid=tracker_.validateInput(segmentation.capture_time_ns,segmentation.sequence);
    if (!valid.isOk()) return valid;
    const auto started=std::chrono::steady_clock::now();
    auto out=pipeline_.process(segmentation,upstream_latency_ms,frame_projector);
    if (!out.isOk()) return out.status();
    const auto expired = [&]() {
        const auto now=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        return deadline_ns && now>deadline_ns;
    };
    const auto mark_stale = [&]() {
        auto& n=out.value().navigation; n.status=TrackingStatus::kStaleFrame; n.confidence=0;
        n.lateral_error_m=n.heading_error_rad=std::numeric_limits<double>::quiet_NaN();
        n.tracking={}; // No fresh temporal update was committed.
    };
    if (expired()) { mark_stale(); return out; }
    auto candidate=tracker_;
    const auto updated=candidate.update(out.value());
    if (!updated.isOk()) return updated;
    if (expired()) mark_stale(); else tracker_=std::move(candidate);
    out.value().navigation.latency_ms=upstream_latency_ms+
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    return out;
}
} // namespace rk_nav
