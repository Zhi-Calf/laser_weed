#pragma once
#include "support/row_scenes.hpp"
#include "rk_nav/inference/segmenter.hpp"
#include "rk_nav/pipeline/perception_pipeline.hpp"

namespace rk_nav::test {
struct RowSequenceFrame {
    SegmentationResult segmentation;
    double truth_lateral{0.04}, truth_heading{std::atan(.025)};
    bool observed{true};
};
inline const std::vector<std::string>& rowSequenceNames() {
    static const std::vector<std::string> names={"jitter","drift","impulse","short_occlusion","long_occlusion","sustained_jump","step"};
    return names;
}
inline RowSequenceFrame sequenceFrame(const std::string& name,std::uint64_t frame) {
    RowSequenceFrame out;
    RowScene scene;
    if (name=="jitter") {
        scene.center+=(frame%2 ? .015 : -.015);
        scene.slope+=(frame%2 ? .002 : -.002);
    }
    if (name=="drift") { scene.center+=.002*double(frame); scene.slope+=.0003*double(frame); }
    if ((name=="impulse" && frame==25) || (name=="sustained_jump" && frame>=25)) scene.center+=.22;
    if (name=="step" && frame>=20) scene.center+=.04;
    out.truth_lateral=(name=="jitter" || name=="impulse") ? .04 : scene.center;
    out.truth_heading=name=="jitter" ? std::atan(.025) : std::atan(scene.slope);
    out.observed=!((name=="short_occlusion" && frame>=25 && frame<=26) ||
                   (name=="long_occlusion" && frame>=20 && frame<=28));
    out.segmentation.sequence=frame;
    out.segmentation.capture_time_ns=1000000000LL+static_cast<std::int64_t>(frame)*100000000LL;
    out.segmentation.combined_mask=sceneMask(scene);
    if (!out.observed) out.segmentation.combined_mask.setTo(0);
    out.segmentation.instances.push_back({{},0,1.F,{}});
    return out;
}
inline PerceptionPipeline sequencePipeline() {
    return {sceneProjector(),RowExtractor(sceneConfig({},RowExtractionMethod::kAssociatedRows)),CenterlineFitter{}};
}
} // namespace rk_nav::test
