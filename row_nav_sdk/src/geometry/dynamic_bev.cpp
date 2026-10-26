#include "rk_nav/geometry/dynamic_bev.hpp"
#include <cmath>
#include <utility>

namespace rk_nav {
DynamicBevProjector::DynamicBevProjector(BevConfig bev, cv::Matx33d k, DynamicBevConfig config)
    : bev_(std::move(bev)), intrinsics_(k), config_(config) {}

Result<BevProjector> DynamicBevProjector::at(double roll, double pitch) const {
    const auto parameters = validateParameters();
    if (!parameters.isOk()) return parameters;
    if (!config_.enabled || !std::isfinite(roll) || !std::isfinite(pitch) ||
        std::abs(roll) > config_.max_tilt_rad || std::abs(pitch) > config_.max_tilt_rad)
        return Status::invalidArgument("dynamic BEV requires enabled, finite, in-envelope attitude");
    // R_ground_vehicle = Ry(pitch)*Rx(roll). Yaw cancels for a horizontal plane.
    const cv::Vec3d n(-std::sin(pitch), std::cos(pitch)*std::sin(roll),
                      std::cos(pitch)*std::cos(roll));
    if (n[2] < 0.5 || n.dot(config_.camera_position_vehicle_m) + config_.origin_height_m <= 0.05)
        return Status::invalidArgument("camera must remain above the ground plane");
    const auto& t = config_.camera_position_vehicle_m;
    const cv::Matx33d plane(1, 0, -t[0], 0, 1, -t[1],
        -n[0]/n[2], -n[1]/n[2], -config_.origin_height_m/n[2]-t[2]);
    const auto camera = config_.camera_to_vehicle_rotation.t() * plane;
    const auto image = intrinsics_ * camera;
    const double det = cv::determinant(cv::Mat(image));
    if (!cv::checkRange(cv::Mat(image)) || !std::isfinite(det) || std::abs(det) < 1e-9)
        return Status::invalidArgument("dynamic ground projection is singular/non-finite");
    const double s = bev_.meters_per_pixel;
    const cv::Matx33d ground_to_bev(0, -1/s, bev_.vehicle_origin_u,
        -1/s, 0, bev_.vehicle_origin_v, 0, 0, 1);
    auto bev = bev_;
    bev.image_to_bev = ground_to_bev * image.inv();
    bev.camera_depth_from_ground = cv::Vec3d(camera(2,0), camera(2,1), camera(2,2));
    BevProjector projector(bev);
    auto valid = projector.validate();
    if (!valid.isOk()) return valid;
    return projector;
}

Status DynamicBevProjector::validateParameters() const {
    const auto base_valid = BevProjector(bev_).validate();
    if (!base_valid.isOk()) return base_valid;
    const auto& r = config_.camera_to_vehicle_rotation;
    if (!config_.enabled || !cv::checkRange(cv::Mat(r)) ||
        !cv::checkRange(cv::Mat(config_.camera_position_vehicle_m)) ||
        !cv::checkRange(cv::Mat(intrinsics_)) || intrinsics_(0,0) <= 0 || intrinsics_(1,1) <= 0 ||
        std::abs(intrinsics_(2,0)) > 1e-12 || std::abs(intrinsics_(2,1)) > 1e-12 ||
        std::abs(intrinsics_(2,2)-1) > 1e-12 ||
        cv::norm(cv::Mat(r.t()*r - cv::Matx33d::eye())) > 1e-6 ||
        std::abs(cv::determinant(cv::Mat(r))-1) > 1e-6 ||
        !std::isfinite(config_.origin_height_m) || config_.origin_height_m < 0 || config_.origin_height_m > 10 ||
        !std::isfinite(config_.max_tilt_rad) || config_.max_tilt_rad <= 0 || config_.max_tilt_rad > 0.6 ||
        !std::isfinite(config_.max_attitude_age_ms) || config_.max_attitude_age_ms <= 0 || config_.max_attitude_age_ms > 1000 ||
        !std::isfinite(config_.max_reference_error_m) || config_.max_reference_error_m <= 0 || config_.max_reference_error_m > 0.2)
        return Status::invalidArgument("invalid dynamic BEV intrinsics/extrinsics/height/limits");
    return Status::ok();
}

Status DynamicBevProjector::validate() const {
    const auto parameters = validateParameters();
    if (!parameters.isOk()) return parameters;
    auto reference = at(config_.reference_roll_rad, config_.reference_pitch_rad);
    if (!reference.isOk()) return reference.status();
    // Check the physical model against the accepted measured static homography.
    // Comparing in meters also catches wrong transform direction, height, and units.
    const auto inverse = reference.value().config().image_to_bev.inv();
    BevProjector measured(bev_);
    for (double x : {1.0, 3.0, 5.0}) for (double y : {-0.5, 0.0, 0.5}) {
        const auto pixel = measured.groundToBev({x,y});
        const auto q = inverse * cv::Vec3d(pixel.x,pixel.y,1);
        if (!cv::checkRange(cv::Mat(q)) || std::abs(q[2]) < 1e-9)
            return Status::invalidArgument("reference ground check projects onto horizon");
        auto actual = measured.imageToGround({q[0]/q[2],q[1]/q[2]});
        if (!actual.isOk() || cv::norm(actual.value()-cv::Point2d(x,y)) > config_.max_reference_error_m)
            return Status::invalidArgument("dynamic extrinsics disagree with measured BEV at reference attitude");
    }
    return Status::ok();
}

Result<DynamicBevConfig> loadDynamicBevConfig(const std::filesystem::path& path) {
    cv::FileStorage storage(path.string(), cv::FileStorage::READ);
    if (!storage.isOpened()) return Status::notFound("cannot open dynamic BEV configuration");
    DynamicBevConfig config;
    const auto node = storage["dynamic_bev"];
    if (node.empty()) return config;
    int enabled = 0, accepted = 0;
    node["enabled"] >> enabled;
    if (enabled != 0 && enabled != 1) return Status::invalidArgument("dynamic_bev.enabled must be 0 or 1");
    config.enabled = enabled == 1;
    if (!config.enabled) return config;
    node["calibration_valid"] >> accepted;
    for (const auto* key : {"camera_to_vehicle_rotation", "camera_position_vehicle_m", "origin_height_m",
                           "reference_roll_rad", "reference_pitch_rad"})
        if (node[key].empty()) return Status::invalidArgument("dynamic BEV requires explicit measured extrinsics, height and reference attitude");
    if (accepted != 1) return Status::invalidArgument("dynamic BEV calibration_valid must be 1");
    cv::Mat r, t;
    node["camera_to_vehicle_rotation"] >> r;
    node["camera_position_vehicle_m"] >> t;
    if (r.rows != 3 || r.cols != 3 || r.channels() != 1 || t.rows != 3 || t.cols != 1 || t.channels() != 1)
        return Status::invalidArgument("dynamic BEV requires 3x3 rotation and 3x1 translation");
    r.convertTo(r,CV_64F); t.convertTo(t,CV_64F);
    for (int i=0;i<3;++i) {
        config.camera_position_vehicle_m[i]=t.at<double>(i,0);
        for (int j=0;j<3;++j) config.camera_to_vehicle_rotation(i,j)=r.at<double>(i,j);
    }
    node["origin_height_m"] >> config.origin_height_m;
    node["reference_roll_rad"] >> config.reference_roll_rad;
    node["reference_pitch_rad"] >> config.reference_pitch_rad;
    if (!node["max_tilt_rad"].empty()) node["max_tilt_rad"] >> config.max_tilt_rad;
    if (!node["max_attitude_age_ms"].empty()) node["max_attitude_age_ms"] >> config.max_attitude_age_ms;
    if (!node["max_reference_error_m"].empty()) node["max_reference_error_m"] >> config.max_reference_error_m;
    return config;
}
} // namespace rk_nav
