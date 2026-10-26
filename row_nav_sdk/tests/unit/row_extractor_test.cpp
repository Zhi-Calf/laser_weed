#include "test_framework.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

#include "rk_nav/geometry/centerline_fitter.hpp"
#include "rk_nav/perception/row_extractor.hpp"

TEST_CASE("row extractor builds centerline between variable crop rows") {
    rk_nav::BevConfig bev_config;
    bev_config.output_size = {400, 600};
    bev_config.meters_per_pixel = 0.01;
    bev_config.vehicle_origin_u = 200.0;
    bev_config.vehicle_origin_v = 599.0;
    rk_nav::BevProjector projector(bev_config);

    cv::Mat mask = cv::Mat::zeros(bev_config.output_size, CV_8UC1);
    // Two slightly angled crop rows, 0.8 m apart. Their midpoint is y = 0.04*x + 0.03 m.
    for (int v = 99; v <= 559; ++v) {
        const double forward = (bev_config.vehicle_origin_v - v) * bev_config.meters_per_pixel;
        const double center_lateral = 0.04 * forward + 0.03;
        const double left_lateral = center_lateral + 0.4;
        const double right_lateral = center_lateral - 0.4;
        const int left_u = static_cast<int>(std::lround(bev_config.vehicle_origin_u - left_lateral / 0.01));
        const int right_u = static_cast<int>(std::lround(bev_config.vehicle_origin_u - right_lateral / 0.01));
        cv::circle(mask, {left_u, v}, 3, cv::Scalar(255), cv::FILLED);
        cv::circle(mask, {right_u, v}, 3, cv::Scalar(255), cv::FILLED);
    }
    cv::rectangle(mask, cv::Rect(10, 200, 12, 80), cv::Scalar(255), cv::FILLED);

    rk_nav::RowExtractorConfig extract_config;
    extract_config.min_forward_m = 0.5;
    extract_config.max_forward_m = 5.0;
    extract_config.min_centerline_points = 20;
    rk_nav::RowExtractor extractor(extract_config);
    const auto rows = extractor.extractBetweenRows(mask, projector);
    REQUIRE(rows.isOk());
    REQUIRE(std::abs(rows.value().median_row_spacing_m - 0.8) < 0.03);
    REQUIRE(rows.value().confidence > 0.9F);

    rk_nav::CenterlineFitConfig fit_config;
    fit_config.ransac_threshold_m = 0.03;
    fit_config.min_inliers = 20;
    rk_nav::CenterlineFitter fitter(fit_config);
    const auto fit = fitter.fit(rows.value().centerline_ground_points);
    REQUIRE(fit.isOk());
    REQUIRE(std::abs(fit.value().slope - 0.04) < 0.01);
    REQUIRE(std::abs(fit.value().lateral_error_m - 0.03) < 0.02);
}

TEST_CASE("row extractor rejects row pairs outside configured spacing") {
    rk_nav::BevConfig bev_config;
    bev_config.output_size = {200, 300};
    bev_config.meters_per_pixel = 0.01;
    bev_config.vehicle_origin_u = 100.0;
    bev_config.vehicle_origin_v = 299.0;
    rk_nav::BevProjector projector(bev_config);
    cv::Mat mask = cv::Mat::zeros(bev_config.output_size, CV_8UC1);
    cv::line(mask, {10, 0}, {10, 299}, cv::Scalar(255), 5);
    cv::line(mask, {190, 0}, {190, 299}, cv::Scalar(255), 5);

    rk_nav::RowExtractorConfig config;
    config.min_forward_m = 0.1;
    config.max_forward_m = 2.5;
    config.min_centerline_points = 4;
    rk_nav::RowExtractor extractor(config);
    const auto result = extractor.extractBetweenRows(mask, projector);
    REQUIRE(result.isOk());
    REQUIRE(!result.value().navigation_pair_valid);
    REQUIRE(result.value().centerline_ground_points.empty());
}
