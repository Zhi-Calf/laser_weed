#pragma once
#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "rk_nav/perception/row_extractor.hpp"

namespace rk_nav::test {
struct RowScene {
    std::string name;
    double spacing{0.8}, center{0.04}, slope{0.025};
    bool short_gap{false}, weeds{false}, bridge{false}, missing_left{false};
    bool spacing_prior{false};
    bool expected_valid{true};
};
inline BevProjector sceneProjector() {
    BevConfig c;
    c.output_size = {640,640}; c.vehicle_origin_u=320; c.vehicle_origin_v=639;
    c.meters_per_pixel=.01;
    return BevProjector(c);
}
inline RowExtractorConfig sceneConfig(const RowScene& scene, RowExtractionMethod method) {
    RowExtractorConfig c;
    c.max_forward_m=5.8; c.method=method;
    if (scene.spacing_prior) {
        c.multi_row.expected_spacing_m=scene.spacing;
        c.multi_row.spacing_tolerance_m=.08;
    }
    return c;
}
inline cv::Mat sceneMask(const RowScene& scene) {
    const auto projector = sceneProjector();
    cv::Mat mask = cv::Mat::zeros(projector.config().output_size,CV_8UC1);
    for (int v=39; v<=609; ++v) {
        const double x=(639-v)*.01;
        for (double multiplier : {-1.5,-.5,.5,1.5}) {
            if (multiplier==.5 && (scene.missing_left || (scene.short_gap && x>2.0 && x<2.4))) continue;
            const double y=scene.slope*x+scene.center+multiplier*scene.spacing;
            const auto pixel=projector.groundToBev({x,y});
            cv::circle(mask,pixel,2,cv::Scalar(255),cv::FILLED);
        }
        if (scene.weeds && x>1.5 && x<2.2)
            cv::circle(mask,projector.groundToBev({x,scene.slope*x+scene.center+.12}),2,cv::Scalar(255),cv::FILLED);
        if (scene.bridge && x>3.0 && x<3.25)
            cv::line(mask,projector.groundToBev({x,scene.slope*x+scene.center-scene.spacing*.5}),
                projector.groundToBev({x,scene.slope*x+scene.center+scene.spacing*.5}),cv::Scalar(255),1);
    }
    return mask;
}
inline std::vector<RowScene> rowScenes() {
    std::vector<RowScene> out;
    for (const double spacing : {.3,.4,.8,1.2,1.4}) {
        const std::string suffix=std::to_string(spacing);
        RowScene clean; clean.name="clean_"+suffix; clean.spacing=spacing;
        out.push_back(clean);
        auto gap=clean; gap.name="gap_"+suffix; gap.short_gap=true; out.push_back(gap);
        auto bridge=clean; bridge.name="bridge_"+suffix; bridge.bridge=true; out.push_back(bridge);
        auto offset=clean; offset.name="offset_"+suffix; offset.center=.3*spacing; out.push_back(offset);
    }
    RowScene weeds; weeds.name="short_weeds"; weeds.weeds=true; out.push_back(weeds);
    RowScene missing; missing.name="missing_left_known_spacing"; missing.spacing=.4;
    missing.missing_left=true; missing.spacing_prior=true; missing.expected_valid=false; out.push_back(missing);
    return out;
}
} // namespace rk_nav::test
