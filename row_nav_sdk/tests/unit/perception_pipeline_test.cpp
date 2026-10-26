#include "test_framework.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

#include "rk_nav/pipeline/perception_pipeline.hpp"

namespace {

rk_nav::PerceptionPipeline makePipeline(float minimum_confidence = 0.35F) {
    rk_nav::BevConfig bev;
    bev.output_size = {400, 600};
    bev.meters_per_pixel = 0.01;
    bev.vehicle_origin_u = 200.0;
    bev.vehicle_origin_v = 599.0;
    rk_nav::RowExtractorConfig rows;
    rows.min_forward_m = 0.5;
    rows.max_forward_m = 5.0;
    rows.min_centerline_points = 20;
    rk_nav::CenterlineFitConfig fit;
    fit.ransac_threshold_m = 0.03;
    fit.min_inliers = 20;
    fit.lookahead_m = 2.0;
    rk_nav::PerceptionPipelineConfig pipeline;
    pipeline.minimum_navigation_confidence = minimum_confidence;
    return {rk_nav::BevProjector(bev), rk_nav::RowExtractor(rows),
            rk_nav::CenterlineFitter(fit), pipeline};
}

rk_nav::SegmentationResult makeRows(float confidence) {
    rk_nav::SegmentationResult segmentation;
    segmentation.sequence = 7;
    segmentation.capture_time_ns = 123456789;
    segmentation.combined_mask = cv::Mat::zeros(600, 400, CV_8UC1);
    for (int v = 80; v <= 570; ++v) {
        const double forward = (599.0 - static_cast<double>(v)) * 0.01;
        const double center = 0.03 * forward - 0.06;
        const int first = static_cast<int>(std::lround(200.0 - (center + 0.4) / 0.01));
        const int second = static_cast<int>(std::lround(200.0 - (center - 0.4) / 0.01));
        cv::circle(segmentation.combined_mask, {first, v}, 3, cv::Scalar(255), cv::FILLED);
        cv::circle(segmentation.combined_mask, {second, v}, 3, cv::Scalar(255), cv::FILLED);
    }
    rk_nav::SegmentationInstance instance;
    instance.confidence = confidence;
    segmentation.instances.push_back(instance);
    return segmentation;
}

}  // namespace

TEST_CASE("perception pipeline converts segmentation into navigation observation") {
    auto pipeline = makePipeline();
    const auto result = pipeline.process(makeRows(0.95F), 12.5);
    REQUIRE(result.isOk());
    REQUIRE(result.value().navigation.status == rk_nav::TrackingStatus::kValid);
    REQUIRE(result.value().navigation.capture_time_ns == 123456789);
    REQUIRE(result.value().navigation.sequence == 7);
    REQUIRE(std::abs(result.value().navigation.lateral_error_m + 0.06) < 0.02);
    REQUIRE(std::abs(result.value().navigation.heading_error_rad - std::atan(0.03)) < 0.01);
    REQUIRE(result.value().navigation.lookahead_m == 2.0);
    REQUIRE(result.value().navigation.confidence > 0.7F);
    REQUIRE(result.value().navigation.latency_ms >= 12.5);
    REQUIRE(result.value().rows.has_value());
    REQUIRE(result.value().centerline.has_value());
}

TEST_CASE("perception pipeline reports no row without treating it as a program error") {
    auto pipeline = makePipeline();
    rk_nav::SegmentationResult segmentation;
    segmentation.capture_time_ns = 42;
    const auto result = pipeline.process(segmentation);
    REQUIRE(result.isOk());
    REQUIRE(result.value().navigation.status == rk_nav::TrackingStatus::kNoRow);
    REQUIRE(result.value().navigation.capture_time_ns == 42);
    REQUIRE(!result.value().rows.has_value());
}

TEST_CASE("perception pipeline preserves low confidence state") {
    auto pipeline = makePipeline(0.8F);
    const auto result = pipeline.process(makeRows(0.5F));
    REQUIRE(result.isOk());
    REQUIRE(result.value().navigation.status == rk_nav::TrackingStatus::kLowConfidence);
    REQUIRE(result.value().centerline.has_value());
}
