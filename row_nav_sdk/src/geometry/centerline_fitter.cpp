#include "rk_nav/geometry/centerline_fitter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

namespace rk_nav {
namespace {

bool finitePoint(const cv::Point2d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool lineThrough(const cv::Point2d& a, const cv::Point2d& b, double& slope, double& intercept) {
    const double delta_x = b.x - a.x;
    if (std::abs(delta_x) < 1e-9) return false;
    slope = (b.y - a.y) / delta_x;
    intercept = a.y - slope * a.x;
    return std::isfinite(slope) && std::isfinite(intercept);
}

double residual(const cv::Point2d& point, double slope, double intercept) {
    return std::abs(point.y - slope * point.x - intercept) / std::sqrt(slope * slope + 1.0);
}

bool leastSquares(const std::vector<cv::Point2d>& points, const std::vector<std::size_t>& indices,
                  double& slope, double& intercept) {
    if (indices.size() < 2) return false;
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto index : indices) {
        sum_x += points[index].x;
        sum_y += points[index].y;
    }
    const double mean_x = sum_x / static_cast<double>(indices.size());
    const double mean_y = sum_y / static_cast<double>(indices.size());
    double numerator = 0.0;
    double denominator = 0.0;
    for (const auto index : indices) {
        const double dx = points[index].x - mean_x;
        numerator += dx * (points[index].y - mean_y);
        denominator += dx * dx;
    }
    if (denominator < 1e-12) return false;
    slope = numerator / denominator;
    intercept = mean_y - slope * mean_x;
    return std::isfinite(slope) && std::isfinite(intercept);
}

std::vector<std::size_t> classifyInliers(const std::vector<cv::Point2d>& points,
                                         double slope, double intercept,
                                         double threshold, double* squared_error = nullptr) {
    std::vector<std::size_t> inliers;
    double error = 0.0;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const double distance = residual(points[index], slope, intercept);
        if (distance <= threshold) {
            inliers.push_back(index);
            error += distance * distance;
        }
    }
    if (squared_error != nullptr) *squared_error = error;
    return inliers;
}

}  // namespace

CenterlineFitter::CenterlineFitter(CenterlineFitConfig config) : config_(std::move(config)) {}

Result<CenterlineFit> CenterlineFitter::fit(const std::vector<cv::Point2d>& input_points) const {
    if (!std::isfinite(config_.ransac_threshold_m) || config_.ransac_threshold_m <= 0.0) {
        return Status::invalidArgument("RANSAC threshold must be greater than zero");
    }
    if (config_.ransac_iterations == 0 || config_.local_optimization_iterations == 0 ||
        config_.min_inliers < 2) {
        return Status::invalidArgument("RANSAC, local optimization and min_inliers are invalid");
    }

    std::vector<cv::Point2d> points;
    points.reserve(input_points.size());
    std::copy_if(input_points.begin(), input_points.end(), std::back_inserter(points), finitePoint);
    if (points.size() < config_.min_inliers) {
        return Status::unavailable("not enough finite centerline points");
    }

    std::mt19937 generator(0x3588U);
    std::uniform_int_distribution<std::size_t> distribution(0, points.size() - 1);
    std::vector<std::size_t> best_inliers;
    double best_error = std::numeric_limits<double>::infinity();
    for (std::uint64_t iteration = 0; iteration < config_.ransac_iterations; ++iteration) {
        const auto first = distribution(generator);
        auto second = distribution(generator);
        if (first == second) second = (second + 1) % points.size();
        double slope = 0.0;
        double intercept = 0.0;
        if (!lineThrough(points[first], points[second], slope, intercept)) continue;

        double error = 0.0;
        auto inliers = classifyInliers(points, slope, intercept,
                                       config_.ransac_threshold_m, &error);
        if (inliers.size() > best_inliers.size() ||
            (inliers.size() == best_inliers.size() && error < best_error)) {
            best_inliers = std::move(inliers);
            best_error = error;
        }
    }
    if (best_inliers.size() < config_.min_inliers) {
        return Status::unavailable("centerline RANSAC did not find enough inliers");
    }

    double slope = 0.0;
    double intercept = 0.0;
    std::uint64_t refinement_steps = 0;
    for (; refinement_steps < config_.local_optimization_iterations; ++refinement_steps) {
        if (!leastSquares(points, best_inliers, slope, intercept)) {
            return Status::unavailable("centerline local fit is degenerate");
        }
        auto refined = classifyInliers(points, slope, intercept, config_.ransac_threshold_m);
        if (refined.size() < config_.min_inliers) {
            return Status::unavailable("centerline local optimization removed too many inliers");
        }
        if (refined == best_inliers) {
            ++refinement_steps;
            break;
        }
        best_inliers = std::move(refined);
    }
    if (!leastSquares(points, best_inliers, slope, intercept)) {
        return Status::unavailable("centerline least-squares fit is degenerate");
    }
    double squared_error = 0.0;
    for (const auto index : best_inliers) {
        const double distance = residual(points[index], slope, intercept);
        squared_error += distance * distance;
    }

    CenterlineFit result;
    result.slope = slope;
    result.intercept_m = intercept;
    result.heading_error_rad = std::atan(slope);
    result.lateral_error_m = intercept;
    result.lookahead_offset_m = slope * config_.lookahead_m + intercept;
    result.rms_residual_m = std::sqrt(squared_error / static_cast<double>(best_inliers.size()));
    result.inlier_count = static_cast<std::uint64_t>(best_inliers.size());
    result.point_count = static_cast<std::uint64_t>(points.size());
    result.refinement_steps = refinement_steps;
    const double inlier_ratio = static_cast<double>(best_inliers.size()) / static_cast<double>(points.size());
    const double residual_score = std::max(0.0, 1.0 - result.rms_residual_m / config_.ransac_threshold_m);
    result.confidence = static_cast<float>(std::clamp(inlier_ratio * residual_score, 0.0, 1.0));
    return result;
}

const CenterlineFitConfig& CenterlineFitter::config() const { return config_; }

}  // namespace rk_nav
