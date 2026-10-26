#include "rk_nav/calibration/camera_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace rk_nav {
namespace {

bool finiteMatrix(const cv::Matx33d& matrix) {
    for (const double value : matrix.val) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

cv::Mat distortionMat(const std::vector<double>& coefficients) {
    return cv::Mat(coefficients, true).reshape(1, 1);
}

double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    const auto index = static_cast<std::size_t>(
        std::floor(fraction * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index),
                     values.end());
    return values[index];
}

}  // namespace

Status CameraCalibration::validateFor(const cv::Size& frame_size) const {
    if (schema_version != 1) return Status::invalidArgument("unsupported camera calibration schema version");
    if (image_size.width <= 0 || image_size.height <= 0) {
        return Status::invalidArgument("calibration image size must be greater than zero");
    }
    if (frame_size.width > 0 && frame_size.height > 0 && frame_size != image_size) {
        return Status::invalidArgument("frame resolution does not match camera calibration");
    }
    if (!finiteMatrix(camera_matrix) || camera_matrix(0, 0) <= 0.0 || camera_matrix(1, 1) <= 0.0 ||
        std::abs(camera_matrix(2, 2) - 1.0) > 1e-9) {
        return Status::invalidArgument("camera matrix is invalid");
    }
    if (camera_matrix(0, 2) < 0.0 || camera_matrix(0, 2) >= image_size.width ||
        camera_matrix(1, 2) < 0.0 || camera_matrix(1, 2) >= image_size.height) {
        return Status::invalidArgument("camera principal point lies outside the calibrated image");
    }
    if (distortion_coefficients.size() < 4) {
        return Status::invalidArgument("at least four distortion coefficients are required");
    }
    for (const double coefficient : distortion_coefficients) {
        if (!std::isfinite(coefficient)) return Status::invalidArgument("distortion coefficients must be finite");
    }
    if (!std::isfinite(reprojection_rms_px) || reprojection_rms_px < 0.0) {
        return Status::invalidArgument("reprojection RMS must be finite and non-negative");
    }
    return Status::ok();
}

Result<cv::Mat> CameraCalibration::undistort(const cv::Mat& image) const {
    if (image.empty()) return Status::invalidArgument("cannot undistort an empty image");
    const auto status = validateFor(image.size());
    if (!status.isOk()) return status;
    cv::Mat output;
    cv::undistort(image, output, cv::Mat(camera_matrix), distortionMat(distortion_coefficients));
    return output;
}

Result<UndistortDiagnostics> CameraCalibration::undistortDiagnostics() const {
    const auto status = validateFor(image_size);
    if (!status.isOk()) return status;
    cv::Mat map_x;
    cv::Mat map_y;
    cv::initUndistortRectifyMap(cv::Mat(camera_matrix),
                                distortionMat(distortion_coefficients), cv::Mat(),
                                cv::Mat(camera_matrix), image_size, CV_32FC1, map_x, map_y);
    std::vector<double> displacement;
    std::vector<double> edge_displacement;
    displacement.reserve(static_cast<std::size_t>(image_size.area()));
    edge_displacement.reserve(static_cast<std::size_t>(image_size.area() / 2));
    double total = 0.0;
    double maximum = 0.0;
    std::size_t invalid = 0;
    const int edge_x = image_size.width / 5;
    const int edge_y = image_size.height / 5;
    for (int y = 0; y < image_size.height; ++y) {
        for (int x = 0; x < image_size.width; ++x) {
            const double source_x = static_cast<double>(map_x.at<float>(y, x));
            const double source_y = static_cast<double>(map_y.at<float>(y, x));
            const double dx = source_x - static_cast<double>(x);
            const double dy = source_y - static_cast<double>(y);
            const double distance = std::hypot(dx, dy);
            displacement.push_back(distance);
            total += distance;
            maximum = std::max(maximum, distance);
            if (x < edge_x || x >= image_size.width - edge_x ||
                y < edge_y || y >= image_size.height - edge_y) {
                edge_displacement.push_back(distance);
            }
            constexpr double kMapTolerancePixels = 1e-3;
            if (source_x < -kMapTolerancePixels ||
                source_x > static_cast<double>(image_size.width - 1) + kMapTolerancePixels ||
                source_y < -kMapTolerancePixels ||
                source_y > static_cast<double>(image_size.height - 1) + kMapTolerancePixels) {
                ++invalid;
            }
        }
    }
    UndistortDiagnostics diagnostics;
    diagnostics.mean_displacement_px = total / static_cast<double>(displacement.size());
    diagnostics.p95_displacement_px = percentile(displacement, 0.95);
    diagnostics.max_displacement_px = maximum;
    diagnostics.edge_p95_displacement_px = percentile(edge_displacement, 0.95);
    diagnostics.invalid_border_percent = 100.0 * static_cast<double>(invalid) /
        static_cast<double>(displacement.size());
    return diagnostics;
}

Result<CameraCalibration> loadCameraCalibration(const std::filesystem::path& path) {
    cv::FileStorage storage(path.string(), cv::FileStorage::READ);
    if (!storage.isOpened()) return Status::notFound("cannot open camera calibration: " + path.string());

    CameraCalibration calibration;
    storage["schema_version"] >> calibration.schema_version;
    storage["camera_name"] >> calibration.camera_name;
    storage["image_width"] >> calibration.image_size.width;
    storage["image_height"] >> calibration.image_size.height;
    storage["reprojection_rms_px"] >> calibration.reprojection_rms_px;

    cv::Mat camera_matrix;
    cv::Mat distortion;
    storage["camera_matrix"] >> camera_matrix;
    storage["distortion_coefficients"] >> distortion;
    if (camera_matrix.rows != 3 || camera_matrix.cols != 3 || camera_matrix.channels() != 1) {
        return Status::invalidArgument("camera_matrix must be 3x3");
    }
    camera_matrix.convertTo(camera_matrix, CV_64F);
    calibration.camera_matrix = camera_matrix;
    if (distortion.empty() || distortion.channels() != 1) {
        return Status::invalidArgument("distortion_coefficients are missing");
    }
    distortion.convertTo(distortion, CV_64F);
    calibration.distortion_coefficients.assign(
        distortion.ptr<double>(), distortion.ptr<double>() + distortion.total());

    const auto status = calibration.validateFor(calibration.image_size);
    if (!status.isOk()) return status;
    return calibration;
}

Status saveCameraCalibration(const std::filesystem::path& path,
                             const CameraCalibration& calibration) {
    const auto status = calibration.validateFor(calibration.image_size);
    if (!status.isOk()) return status;
    cv::FileStorage storage(path.string(), cv::FileStorage::WRITE);
    if (!storage.isOpened()) return Status::unavailable("cannot write camera calibration: " + path.string());
    storage << "schema_version" << calibration.schema_version;
    storage << "camera_name" << calibration.camera_name;
    storage << "image_width" << calibration.image_size.width;
    storage << "image_height" << calibration.image_size.height;
    storage << "camera_matrix" << cv::Mat(calibration.camera_matrix);
    storage << "distortion_coefficients" << distortionMat(calibration.distortion_coefficients);
    storage << "reprojection_rms_px" << calibration.reprojection_rms_px;
    return Status::ok();
}

}  // namespace rk_nav
