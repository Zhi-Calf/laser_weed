#include "test_framework.hpp"

#include <cmath>
#include <filesystem>
#include <vector>

#include "rk_nav/geometry/bev_calibration.hpp"

namespace {

rk_nav::BevGroundControlPoint point(double u, double v, bool fit) {
    // Synthetic identity image-to-BEV mapping for the default 640x800, 1 cm/pixel layout.
    return {{u, v}, {(799.0 - v) * 0.01, (320.0 - u) * 0.01}, fit};
}

std::vector<rk_nav::BevGroundControlPoint> syntheticPoints() {
    return {
        point(100, 700, true), point(540, 700, true),
        point(150, 550, true), point(490, 550, true),
        point(200, 400, true), point(440, 400, true),
        point(240, 250, true), point(400, 250, true),
        point(320, 650, false), point(320, 500, false),
        point(280, 350, false), point(360, 350, false),
    };
}

}  // namespace

TEST_CASE("BEV calibration accepts independent synthetic check points") {
    rk_nav::BevConfig layout;
    const auto result = rk_nav::calibrateBev(syntheticPoints(), layout);
    REQUIRE(result.isOk());
    REQUIRE(result.value().accepted);
    REQUIRE(result.value().fit_inliers == 8);
    REQUIRE(result.value().check_metrics.count == 4);
    REQUIRE(result.value().check_metrics.rmse_m < 1e-8);
    REQUIRE(cv::norm(cv::Mat(result.value().config.image_to_bev), cv::Mat::eye(3, 3, CV_64F)) < 1e-8);

    const auto path = std::filesystem::temp_directory_path() / "rk_nav_bev_accepted.yaml";
    REQUIRE(rk_nav::saveBevCalibration(path, result.value()).isOk());
    const auto loaded = rk_nav::loadBevConfig(path);
    REQUIRE(loaded.isOk());
    REQUIRE(cv::norm(cv::Mat(loaded.value().image_to_bev),
                     cv::Mat(result.value().config.image_to_bev)) < 1e-8);
    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST_CASE("BEV calibration stays invalid without independent check points") {
    auto points = syntheticPoints();
    for (auto& item : points) item.fit = true;
    const auto result = rk_nav::calibrateBev(points, rk_nav::BevConfig{});
    REQUIRE(result.isOk());
    REQUIRE(!result.value().accepted);

    const auto path = std::filesystem::temp_directory_path() / "rk_nav_bev_candidate.yaml";
    REQUIRE(rk_nav::saveBevCalibration(path, result.value()).isOk());
    REQUIRE(!rk_nav::loadBevConfig(path).isOk());
    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST_CASE("BEV calibration rejects inaccurate independent check points") {
    auto points = syntheticPoints();
    points.back().ground_m.y -= 0.50;
    const auto result = rk_nav::calibrateBev(points, rk_nav::BevConfig{});
    REQUIRE(result.isOk());
    REQUIRE(!result.value().accepted);
    REQUIRE(result.value().check_metrics.rmse_m > 0.05);
}

TEST_CASE("BEV calibration rejects fewer than six fit points") {
    auto points = syntheticPoints();
    points.resize(5);
    const auto result = rk_nav::calibrateBev(points, rk_nav::BevConfig{});
    REQUIRE(!result.isOk());
    REQUIRE(result.status().code() == rk_nav::StatusCode::kInvalidArgument);
}

TEST_CASE("BEV point CSV loads fit and check roles") {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "fixtures" / "bev_points_synthetic.csv";
    const auto points = rk_nav::loadBevGroundControlPointsCsv(path);
    REQUIRE(points.isOk());
    REQUIRE(points.value().size() == 12);
    REQUIRE(points.value()[0].fit);
    REQUIRE(!points.value().back().fit);
}

TEST_CASE("BEV point CSV round trips without changing coordinates") {
    const auto source_path = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "fixtures" / "bev_points_synthetic.csv";
    const auto source = rk_nav::loadBevGroundControlPointsCsv(source_path);
    REQUIRE(source.isOk());
    const auto output_path = std::filesystem::temp_directory_path() /
        "rk_nav_bev_points_roundtrip.csv";
    REQUIRE(rk_nav::saveBevGroundControlPointsCsv(output_path, source.value()).isOk());
    const auto loaded = rk_nav::loadBevGroundControlPointsCsv(output_path);
    std::error_code error;
    std::filesystem::remove(output_path, error);
    REQUIRE(loaded.isOk());
    REQUIRE(loaded.value().size() == source.value().size());
    REQUIRE(cv::norm(loaded.value()[3].image_pixel - source.value()[3].image_pixel) < 1e-9);
    REQUIRE(cv::norm(loaded.value()[9].ground_m - source.value()[9].ground_m) < 1e-9);
}
