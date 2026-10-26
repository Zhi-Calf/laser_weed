#include "test_framework.hpp"

#include <cmath>
#include <filesystem>

#include "rk_nav/calibration/camera_calibration.hpp"

namespace {

rk_nav::CameraCalibration validCalibration() {
    rk_nav::CameraCalibration calibration;
    calibration.camera_name = "test-camera";
    calibration.image_size = {1920, 1080};
    calibration.camera_matrix = {900.0, 0.0, 960.0, 0.0, 910.0, 540.0, 0.0, 0.0, 1.0};
    calibration.distortion_coefficients = {-0.2, 0.05, 0.0, 0.0, 0.0};
    calibration.reprojection_rms_px = 0.4;
    return calibration;
}

}  // namespace

TEST_CASE("camera calibration round trips through YAML") {
    const auto path = std::filesystem::temp_directory_path() / "rk_nav_camera_calibration.yaml";
    const auto source = validCalibration();
    REQUIRE(rk_nav::saveCameraCalibration(path, source).isOk());
    const auto loaded = rk_nav::loadCameraCalibration(path);
    std::error_code error;
    std::filesystem::remove(path, error);
    REQUIRE(loaded.isOk());
    REQUIRE(loaded.value().camera_name == source.camera_name);
    REQUIRE(loaded.value().image_size == source.image_size);
    REQUIRE(std::abs(loaded.value().camera_matrix(0, 0) - 900.0) < 1e-9);
    REQUIRE(loaded.value().distortion_coefficients.size() == 5);
}

TEST_CASE("camera calibration rejects a different runtime resolution") {
    const auto calibration = validCalibration();
    const auto status = calibration.validateFor({1280, 720});
    REQUIRE(!status.isOk());
    REQUIRE(status.code() == rk_nav::StatusCode::kInvalidArgument);
}

TEST_CASE("camera calibration rejects principal point outside the image") {
    auto calibration = validCalibration();
    calibration.camera_matrix(0, 2) = 2000.0;
    REQUIRE(!calibration.validateFor(calibration.image_size).isOk());
}

TEST_CASE("zero distortion reports zero undistort displacement") {
    auto calibration = validCalibration();
    calibration.distortion_coefficients.assign(5, 0.0);
    const auto diagnostics = calibration.undistortDiagnostics();
    REQUIRE(diagnostics.isOk());
    REQUIRE(diagnostics.value().mean_displacement_px < 1e-6);
    REQUIRE(diagnostics.value().p95_displacement_px < 1e-6);
    REQUIRE(diagnostics.value().max_displacement_px < 1e-6);
    REQUIRE(diagnostics.value().invalid_border_percent < 1e-9);
}

TEST_CASE("nonzero distortion reports stronger edge displacement") {
    const auto diagnostics = validCalibration().undistortDiagnostics();
    REQUIRE(diagnostics.isOk());
    REQUIRE(diagnostics.value().edge_p95_displacement_px > 1.0);
    REQUIRE(diagnostics.value().max_displacement_px >=
            diagnostics.value().edge_p95_displacement_px);
}
