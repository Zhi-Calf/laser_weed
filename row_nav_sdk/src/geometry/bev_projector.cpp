#include "rk_nav/geometry/bev_projector.hpp"

#include <cmath>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rk_nav {

BevProjector::BevProjector(BevConfig config) : config_(std::move(config)) {}

Status BevProjector::validate() const {
    if (config_.output_size.width <= 0 || config_.output_size.height <= 0) {
        return Status::invalidArgument("BEV output size must be greater than zero");
    }
    if (!std::isfinite(config_.meters_per_pixel) || config_.meters_per_pixel <= 0.0) {
        return Status::invalidArgument("BEV meters_per_pixel must be greater than zero");
    }
    if (!std::isfinite(config_.vehicle_origin_u) || !std::isfinite(config_.vehicle_origin_v) ||
        (config_.camera_depth_from_ground && !cv::checkRange(cv::Mat(*config_.camera_depth_from_ground))))
        return Status::invalidArgument("BEV origin/depth must be finite");
    if (!std::isfinite(cv::determinant(cv::Mat(config_.image_to_bev))) ||
        std::abs(cv::determinant(cv::Mat(config_.image_to_bev))) < 1e-12) {
        return Status::invalidArgument("image_to_bev homography is singular");
    }
    return Status::ok();
}

Result<cv::Mat> BevProjector::projectMask(const cv::Mat& image_mask) const {
    const auto status = validate();
    if (!status.isOk()) return status;
    if (image_mask.empty() || image_mask.channels() != 1) {
        return Status::invalidArgument("BEV input mask must be a non-empty single-channel image");
    }
    cv::Mat output;
    cv::warpPerspective(image_mask, output, cv::Mat(config_.image_to_bev), config_.output_size,
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    if (config_.camera_depth_from_ground) {
        cv::Mat invalid(output.size(), CV_8U);
        const auto& d = *config_.camera_depth_from_ground;
        for (int v=0;v<output.rows;++v) {
            auto* row=invalid.ptr<uint8_t>(v);
            const double x=(config_.vehicle_origin_v-v)*config_.meters_per_pixel;
            for (int u=0;u<output.cols;++u) {
                const double y=(config_.vehicle_origin_u-u)*config_.meters_per_pixel;
                row[u]=d[0]*x+d[1]*y+d[2] <= 0.05 ? 255 : 0;
            }
        }
        output.setTo(0,invalid);
    }
    return output;
}

Result<cv::Point2d> BevProjector::imageToGround(const cv::Point2d& image_point) const {
    const auto status = validate();
    if (!status.isOk()) return status;
    const auto q = config_.image_to_bev * cv::Vec3d(image_point.x,image_point.y,1);
    if (!cv::checkRange(cv::Mat(q)) || std::abs(q[2]) < 1e-12) {
        return Status::unavailable("image point cannot be projected to the ground plane");
    }
    const auto ground = bevToGround({q[0]/q[2],q[1]/q[2]});
    if (config_.camera_depth_from_ground) {
        const auto& d = *config_.camera_depth_from_ground;
        if (d[0]*ground.x+d[1]*ground.y+d[2] <= 0.05)
            return Status::unavailable("ground point lies behind/too close to camera");
    }
    return ground;
}

cv::Point2d BevProjector::bevToGround(const cv::Point2d& point) const {
    return {(config_.vehicle_origin_v - point.y) * config_.meters_per_pixel,
            (config_.vehicle_origin_u - point.x) * config_.meters_per_pixel};
}

cv::Point2d BevProjector::groundToBev(const cv::Point2d& point) const {
    return {config_.vehicle_origin_u - point.y / config_.meters_per_pixel,
            config_.vehicle_origin_v - point.x / config_.meters_per_pixel};
}

const BevConfig& BevProjector::config() const { return config_; }

namespace {

Result<BevConfig> loadBevConfigImpl(const std::filesystem::path& path, bool require_valid) {
    cv::FileStorage storage(path.string(), cv::FileStorage::READ);
    if (!storage.isOpened()) return Status::notFound("cannot open BEV configuration: " + path.string());
    int schema_version = 0;
    int calibration_valid = 0;
    BevConfig config;
    cv::Mat homography;
    storage["schema_version"] >> schema_version;
    storage["calibration_valid"] >> calibration_valid;
    storage["output_width"] >> config.output_size.width;
    storage["output_height"] >> config.output_size.height;
    storage["meters_per_pixel"] >> config.meters_per_pixel;
    storage["vehicle_origin_u"] >> config.vehicle_origin_u;
    storage["vehicle_origin_v"] >> config.vehicle_origin_v;
    storage["image_to_bev"] >> homography;
    if (schema_version != 1) return Status::invalidArgument("unsupported BEV schema version");
    if (require_valid && calibration_valid != 1) {
        return Status::invalidArgument("BEV configuration is not marked as measured and valid");
    }
    if (homography.rows != 3 || homography.cols != 3 || homography.channels() != 1) {
        return Status::invalidArgument("image_to_bev must be a 3x3 matrix");
    }
    homography.convertTo(homography, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            config.image_to_bev(row, column) = homography.at<double>(row, column);
        }
    }
    BevProjector projector(config);
    const auto status = projector.validate();
    if (!status.isOk()) return status;
    return config;
}

}  // namespace

Result<BevConfig> loadBevConfig(const std::filesystem::path& path) {
    return loadBevConfigImpl(path, true);
}

Result<BevConfig> loadBevConfigForInspection(const std::filesystem::path& path) {
    return loadBevConfigImpl(path, false);
}

}  // namespace rk_nav
