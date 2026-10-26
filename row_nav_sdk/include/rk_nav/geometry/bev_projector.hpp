#pragma once

#include <filesystem>
#include <optional>

#include <opencv2/core.hpp>

#include "rk_nav/core/status.hpp"

namespace rk_nav {

struct BevConfig {
    cv::Matx33d image_to_bev{cv::Matx33d::eye()};
    cv::Size output_size{640, 800};
    double meters_per_pixel{0.01};
    double vehicle_origin_u{320.0};
    double vehicle_origin_v{799.0};
    // Dynamic plane only: camera optical depth z = ax_vehicle+by_vehicle+c.
    // Excludes projected ground behind/too close to the camera.
    std::optional<cv::Vec3d> camera_depth_from_ground;
};

class BevProjector {
public:
    explicit BevProjector(BevConfig config);

    [[nodiscard]] Status validate() const;
    Result<cv::Mat> projectMask(const cv::Mat& image_mask) const;
    Result<cv::Point2d> imageToGround(const cv::Point2d& image_point) const;
    [[nodiscard]] cv::Point2d bevToGround(const cv::Point2d& bev_point) const;
    [[nodiscard]] cv::Point2d groundToBev(const cv::Point2d& ground_point) const;
    [[nodiscard]] const BevConfig& config() const;

private:
    BevConfig config_;
};

Result<BevConfig> loadBevConfig(const std::filesystem::path& path);
// Inspection tools may load rejected candidates to visualize why they failed.
// Navigation code must use loadBevConfig(), which enforces calibration_valid: 1.
Result<BevConfig> loadBevConfigForInspection(const std::filesystem::path& path);

}  // namespace rk_nav
