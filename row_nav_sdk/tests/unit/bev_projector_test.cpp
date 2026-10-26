#include "test_framework.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>

#include <opencv2/imgproc.hpp>

#include "rk_nav/geometry/bev_projector.hpp"

TEST_CASE("BEV converts pixels to vehicle ground coordinates") {
    rk_nav::BevConfig config;
    config.output_size = {200, 300};
    config.meters_per_pixel = 0.02;
    config.vehicle_origin_u = 100.0;
    config.vehicle_origin_v = 299.0;
    rk_nav::BevProjector projector(config);

    const auto ground = projector.imageToGround({75.0, 199.0});
    REQUIRE(ground.isOk());
    REQUIRE(std::abs(ground.value().x - 2.0) < 1e-9);
    REQUIRE(std::abs(ground.value().y - 0.5) < 1e-9);

    const auto pixel = projector.groundToBev(ground.value());
    REQUIRE(std::abs(pixel.x - 75.0) < 1e-9);
    REQUIRE(std::abs(pixel.y - 199.0) < 1e-9);
}

TEST_CASE("BEV mask projection preserves labels with identity homography") {
    rk_nav::BevConfig config;
    config.output_size = {32, 24};
    rk_nav::BevProjector projector(config);
    cv::Mat mask = cv::Mat::zeros(24, 32, CV_8UC1);
    cv::rectangle(mask, cv::Rect(8, 6, 10, 7), cv::Scalar(255), cv::FILLED);

    const auto projected = projector.projectMask(mask);
    REQUIRE(projected.isOk());
    REQUIRE(cv::countNonZero(projected.value() != mask) == 0);
}

TEST_CASE("BEV rejects singular homography") {
    rk_nav::BevConfig config;
    config.image_to_bev = cv::Matx33d::zeros();
    rk_nav::BevProjector projector(config);
    REQUIRE(!projector.validate().isOk());
}

TEST_CASE("BEV loader rejects an unmeasured placeholder configuration") {
    const auto path = std::filesystem::temp_directory_path() / "rk_nav_bev_placeholder.yaml";
    std::ofstream(path) << "%YAML:1.0\n---\nschema_version: 1\ncalibration_valid: 0\n"
                          "output_width: 200\noutput_height: 300\nmeters_per_pixel: 0.01\n"
                          "vehicle_origin_u: 100.\nvehicle_origin_v: 299.\n"
                          "image_to_bev: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
                          "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]\n";
    const auto result = rk_nav::loadBevConfig(path);
    std::error_code error;
    std::filesystem::remove(path, error);
    REQUIRE(!result.isOk());
    REQUIRE(result.status().code() == rk_nav::StatusCode::kInvalidArgument);
}

TEST_CASE("BEV inspection loader can open a rejected candidate") {
    const auto path = std::filesystem::temp_directory_path() /
        "rk_nav_bev_inspection_candidate.yaml";
    std::ofstream(path) << "%YAML:1.0\n---\nschema_version: 1\ncalibration_valid: 0\n"
                          "output_width: 200\noutput_height: 300\nmeters_per_pixel: 0.01\n"
                          "vehicle_origin_u: 100.\nvehicle_origin_v: 299.\n"
                          "image_to_bev: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
                          "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]\n";
    const auto result = rk_nav::loadBevConfigForInspection(path);
    std::error_code error;
    std::filesystem::remove(path, error);
    REQUIRE(result.isOk());
}
