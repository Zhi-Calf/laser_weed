#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rk_nav/core/status.hpp"

namespace rk_nav {

struct UndistortDiagnostics {
    double mean_displacement_px{0.0};
    double p95_displacement_px{0.0};
    double max_displacement_px{0.0};
    double edge_p95_displacement_px{0.0};
    double invalid_border_percent{0.0};
};

struct CameraCalibration {
    int schema_version{1};
    std::string camera_name;
    cv::Size image_size;
    cv::Matx33d camera_matrix{cv::Matx33d::eye()};
    std::vector<double> distortion_coefficients;
    double reprojection_rms_px{0.0};

    [[nodiscard]] Status validateFor(const cv::Size& frame_size) const;
    Result<cv::Mat> undistort(const cv::Mat& image) const;
    Result<UndistortDiagnostics> undistortDiagnostics() const;
};

Result<CameraCalibration> loadCameraCalibration(const std::filesystem::path& path);
Status saveCameraCalibration(const std::filesystem::path& path,
                             const CameraCalibration& calibration);

}  // namespace rk_nav
