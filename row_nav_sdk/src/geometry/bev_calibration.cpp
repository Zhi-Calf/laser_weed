#include "rk_nav/geometry/bev_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include <opencv2/calib3d.hpp>

namespace rk_nav {
namespace {

bool finitePoint(const cv::Point2d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(trim(field));
    return fields;
}

BevErrorMetrics measureErrors(const std::vector<BevGroundControlPoint>& points,
                              bool fit_role,
                              const BevProjector& projector) {
    BevErrorMetrics metrics;
    double squared_pixels = 0.0;
    double squared_meters = 0.0;
    for (const auto& point : points) {
        if (point.fit != fit_role) continue;
        const auto projected_ground = projector.imageToGround(point.image_pixel);
        if (!projected_ground.isOk()) continue;
        const auto expected_bev = projector.groundToBev(point.ground_m);
        std::vector<cv::Point2d> source{point.image_pixel};
        std::vector<cv::Point2d> projected_bev;
        cv::perspectiveTransform(source, projected_bev,
                                 cv::Mat(projector.config().image_to_bev));
        const double error_pixels = cv::norm(projected_bev[0] - expected_bev);
        const double error_meters = cv::norm(projected_ground.value() - point.ground_m);
        squared_pixels += error_pixels * error_pixels;
        squared_meters += error_meters * error_meters;
        metrics.max_pixels = std::max(metrics.max_pixels, error_pixels);
        metrics.max_m = std::max(metrics.max_m, error_meters);
        ++metrics.count;
    }
    if (metrics.count > 0) {
        const double count = static_cast<double>(metrics.count);
        metrics.rmse_pixels = std::sqrt(squared_pixels / count);
        metrics.rmse_m = std::sqrt(squared_meters / count);
    }
    return metrics;
}

}  // namespace

Result<std::vector<BevGroundControlPoint>> loadBevGroundControlPointsCsv(
    const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) return Status::notFound("cannot open point CSV: " + path.string());
    std::vector<BevGroundControlPoint> points;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto fields = splitCsv(line);
        if (!fields.empty() && fields[0] == "image_u") continue;
        if (fields.size() != 5) {
            return Status::invalidArgument(
                "point CSV line " + std::to_string(line_number) + " must contain five fields");
        }
        try {
            BevGroundControlPoint point;
            std::size_t parsed = 0;
            point.image_pixel.x = std::stod(fields[0], &parsed);
            if (parsed != fields[0].size()) throw std::invalid_argument("image_u");
            point.image_pixel.y = std::stod(fields[1], &parsed);
            if (parsed != fields[1].size()) throw std::invalid_argument("image_v");
            point.ground_m.x = std::stod(fields[2], &parsed);
            if (parsed != fields[2].size()) throw std::invalid_argument("forward_m");
            point.ground_m.y = std::stod(fields[3], &parsed);
            if (parsed != fields[3].size()) throw std::invalid_argument("left_m");
            if (fields[4] == "fit") point.fit = true;
            else if (fields[4] == "check") point.fit = false;
            else throw std::invalid_argument("role");
            if (!finitePoint(point.image_pixel) || !finitePoint(point.ground_m)) {
                throw std::invalid_argument("non-finite point");
            }
            points.push_back(point);
        } catch (...) {
            return Status::invalidArgument(
                "invalid value in point CSV line " + std::to_string(line_number));
        }
    }
    if (points.empty()) return Status::invalidArgument("point CSV contains no control points");
    return points;
}

Status saveBevGroundControlPointsCsv(
    const std::filesystem::path& path,
    const std::vector<BevGroundControlPoint>& points) {
    if (points.empty()) return Status::invalidArgument("cannot save an empty BEV point set");
    std::ofstream output(path);
    if (!output) return Status::unavailable("cannot write point CSV: " + path.string());
    output << "image_u,image_v,forward_m,left_m,role\n" << std::fixed
           << std::setprecision(6);
    for (const auto& point : points) {
        if (!finitePoint(point.image_pixel) || !finitePoint(point.ground_m)) {
            return Status::invalidArgument("cannot save a non-finite BEV control point");
        }
        output << point.image_pixel.x << ',' << point.image_pixel.y << ','
               << point.ground_m.x << ',' << point.ground_m.y << ','
               << (point.fit ? "fit" : "check") << '\n';
    }
    if (!output) return Status::unavailable("failed while writing point CSV: " + path.string());
    return Status::ok();
}

Result<BevCalibrationResult> calibrateBev(
    const std::vector<BevGroundControlPoint>& points,
    const BevConfig& output_layout,
    double max_check_rmse_m,
    double ransac_threshold_pixels) {
    if (!std::isfinite(max_check_rmse_m) || max_check_rmse_m <= 0.0) {
        return Status::invalidArgument("maximum check RMSE must be greater than zero");
    }
    if (!std::isfinite(ransac_threshold_pixels) || ransac_threshold_pixels <= 0.0) {
        return Status::invalidArgument("RANSAC threshold must be greater than zero");
    }
    BevProjector layout_validator(output_layout);
    const auto layout_status = layout_validator.validate();
    if (!layout_status.isOk()) return layout_status;

    std::vector<cv::Point2d> image_fit;
    std::vector<cv::Point2d> bev_fit;
    std::size_t check_count = 0;
    image_fit.reserve(points.size());
    bev_fit.reserve(points.size());
    for (const auto& point : points) {
        if (!finitePoint(point.image_pixel) || !finitePoint(point.ground_m)) {
            return Status::invalidArgument("BEV control points must be finite");
        }
        if (point.fit) {
            image_fit.push_back(point.image_pixel);
            bev_fit.push_back(layout_validator.groundToBev(point.ground_m));
        } else {
            ++check_count;
        }
    }
    if (image_fit.size() < 6) {
        return Status::invalidArgument("BEV calibration requires at least six fit points");
    }

    cv::Mat inlier_mask;
    cv::Mat homography = cv::findHomography(image_fit, bev_fit, cv::RANSAC,
                                            ransac_threshold_pixels, inlier_mask);
    if (homography.empty()) {
        return Status::unavailable("cannot estimate BEV homography from the supplied points");
    }
    homography.convertTo(homography, CV_64F);

    BevCalibrationResult result;
    result.config = output_layout;
    result.max_check_rmse_m = max_check_rmse_m;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.config.image_to_bev(row, column) = homography.at<double>(row, column);
        }
    }
    if (inlier_mask.empty()) result.fit_inliers = image_fit.size();
    else result.fit_inliers = static_cast<std::size_t>(cv::countNonZero(inlier_mask));

    BevProjector projector(result.config);
    const auto projector_status = projector.validate();
    if (!projector_status.isOk()) return projector_status;
    result.fit_metrics = measureErrors(points, true, projector);
    result.check_metrics = measureErrors(points, false, projector);

    const double inlier_ratio = static_cast<double>(result.fit_inliers) /
        static_cast<double>(image_fit.size());
    result.accepted = result.fit_metrics.count == image_fit.size() && check_count >= 4 &&
        result.check_metrics.count == check_count &&
        inlier_ratio >= 0.75 && result.check_metrics.rmse_m <= max_check_rmse_m &&
        result.check_metrics.max_m <= 2.0 * max_check_rmse_m;
    return result;
}

Status saveBevCalibration(const std::filesystem::path& path,
                          const BevCalibrationResult& calibration) {
    cv::FileStorage storage(path.string(), cv::FileStorage::WRITE);
    if (!storage.isOpened()) {
        return Status::unavailable("cannot write BEV configuration: " + path.string());
    }
    storage << "schema_version" << 1;
    storage << "calibration_valid" << (calibration.accepted ? 1 : 0);
    storage << "output_width" << calibration.config.output_size.width;
    storage << "output_height" << calibration.config.output_size.height;
    storage << "meters_per_pixel" << calibration.config.meters_per_pixel;
    storage << "vehicle_origin_u" << calibration.config.vehicle_origin_u;
    storage << "vehicle_origin_v" << calibration.config.vehicle_origin_v;
    storage << "fit_point_count" << static_cast<int>(calibration.fit_metrics.count);
    storage << "fit_inlier_count" << static_cast<int>(calibration.fit_inliers);
    storage << "check_point_count" << static_cast<int>(calibration.check_metrics.count);
    storage << "fit_rmse_m" << calibration.fit_metrics.rmse_m;
    storage << "fit_max_error_m" << calibration.fit_metrics.max_m;
    storage << "check_rmse_m" << calibration.check_metrics.rmse_m;
    storage << "check_max_error_m" << calibration.check_metrics.max_m;
    storage << "max_check_rmse_m" << calibration.max_check_rmse_m;
    storage << "image_to_bev" << cv::Mat(calibration.config.image_to_bev);
    storage.release();
    return Status::ok();
}

}  // namespace rk_nav
