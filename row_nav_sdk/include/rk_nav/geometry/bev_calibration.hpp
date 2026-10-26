#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include <opencv2/core.hpp>

#include "rk_nav/core/status.hpp"
#include "rk_nav/geometry/bev_projector.hpp"

namespace rk_nav {

struct BevGroundControlPoint {
    cv::Point2d image_pixel;
    cv::Point2d ground_m;  // x: forward, y: left
    bool fit{true};
};

struct BevErrorMetrics {
    std::size_t count{0};
    double rmse_pixels{0.0};
    double max_pixels{0.0};
    double rmse_m{0.0};
    double max_m{0.0};
};

struct BevCalibrationResult {
    BevConfig config;
    BevErrorMetrics fit_metrics;
    BevErrorMetrics check_metrics;
    std::size_t fit_inliers{0};
    double max_check_rmse_m{0.05};
    bool accepted{false};
};

Result<BevCalibrationResult> calibrateBev(
    const std::vector<BevGroundControlPoint>& points,
    const BevConfig& output_layout,
    double max_check_rmse_m = 0.05,
    double ransac_threshold_pixels = 3.0);

Result<std::vector<BevGroundControlPoint>> loadBevGroundControlPointsCsv(
    const std::filesystem::path& path);
Status saveBevGroundControlPointsCsv(
    const std::filesystem::path& path,
    const std::vector<BevGroundControlPoint>& points);

Status saveBevCalibration(const std::filesystem::path& path,
                          const BevCalibrationResult& calibration);

}  // namespace rk_nav
