#pragma once
#include "rk_nav/geometry/bev_projector.hpp"

namespace rk_nav {
// Optical-camera -> ROS vehicle rigid transform: p_vehicle = R*p_camera + t.
// The locally flat ground has n_vehicle.p_vehicle + origin_height_m = 0.
// Height is the measured vertical distance of the vehicle origin above ground,
// NOT the camera height. Suspension/heave and sloping terrain are not estimated.
struct DynamicBevConfig {
    bool enabled{false};
    cv::Matx33d camera_to_vehicle_rotation{cv::Matx33d::eye()};
    cv::Vec3d camera_position_vehicle_m{0, 0, 0};
    double origin_height_m{0};
    double reference_roll_rad{0}, reference_pitch_rad{0};
    double max_tilt_rad{0.35}, max_attitude_age_ms{50};
    double max_reference_error_m{0.05};
};

class DynamicBevProjector {
public:
    DynamicBevProjector(BevConfig bev, cv::Matx33d intrinsics, DynamicBevConfig config);
    [[nodiscard]] Status validate() const;
    Result<BevProjector> at(double roll_rad, double pitch_rad) const;
    [[nodiscard]] const DynamicBevConfig& config() const { return config_; }
private:
    [[nodiscard]] Status validateParameters() const;
    BevConfig bev_;
    cv::Matx33d intrinsics_;
    DynamicBevConfig config_;
};
Result<DynamicBevConfig> loadDynamicBevConfig(const std::filesystem::path& path);
} // namespace rk_nav
