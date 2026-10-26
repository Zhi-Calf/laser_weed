#include "test_framework.hpp"

#include <cmath>
#include <vector>

#include "rk_nav/geometry/centerline_fitter.hpp"

TEST_CASE("centerline RANSAC recovers navigation errors with outliers") {
    std::vector<cv::Point2d> points;
    for (int index = 0; index < 30; ++index) {
        const double x = static_cast<double>(index) * 0.15;
        const double noise = static_cast<double>((index % 3) - 1) * 0.003;
        points.emplace_back(x, 0.2 * x + 0.1 + noise);
    }
    points.emplace_back(0.5, -1.2);
    points.emplace_back(2.0, 1.4);
    points.emplace_back(4.0, -0.8);

    rk_nav::CenterlineFitConfig config;
    config.ransac_threshold_m = 0.03;
    config.min_inliers = 20;
    config.lookahead_m = 2.0;
    rk_nav::CenterlineFitter fitter(config);
    const auto result = fitter.fit(points);

    REQUIRE(result.isOk());
    REQUIRE(std::abs(result.value().slope - 0.2) < 0.005);
    REQUIRE(std::abs(result.value().lateral_error_m - 0.1) < 0.01);
    REQUIRE(std::abs(result.value().heading_error_rad - std::atan(0.2)) < 0.005);
    REQUIRE(std::abs(result.value().lookahead_offset_m - 0.5) < 0.02);
    REQUIRE(result.value().inlier_count == 30);
    REQUIRE(result.value().refinement_steps >= 1);
    REQUIRE(result.value().refinement_steps <= config.local_optimization_iterations);
    REQUIRE(result.value().confidence > 0.8F);
}

TEST_CASE("centerline local optimization rejects an invalid iteration budget") {
    rk_nav::CenterlineFitConfig config;
    config.local_optimization_iterations = 0;
    rk_nav::CenterlineFitter fitter(config);
    const auto result = fitter.fit({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
                                    {4.0, 0.0}, {5.0, 0.0}, {6.0, 0.0}, {7.0, 0.0}});
    REQUIRE(!result.isOk());
    REQUIRE(result.status().code() == rk_nav::StatusCode::kInvalidArgument);
}

TEST_CASE("centerline fit rejects insufficient observations") {
    rk_nav::CenterlineFitConfig config;
    config.min_inliers = 4;
    rk_nav::CenterlineFitter fitter(config);
    const auto result = fitter.fit({{0.0, 0.0}, {1.0, 0.0}});
    REQUIRE(!result.isOk());
    REQUIRE(result.status().code() == rk_nav::StatusCode::kUnavailable);
}
