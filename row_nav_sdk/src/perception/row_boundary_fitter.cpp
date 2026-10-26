#include "rk_nav/perception/multi_row.hpp"
#include "rk_nav/geometry/centerline_fitter.hpp"
#include <algorithm>
#include <cmath>

namespace rk_nav {
std::vector<FittedRow> RowBoundaryFitter::fit(const std::vector<AssociatedRow>& rows,
                                            std::uint64_t min_points) const {
    std::vector<FittedRow> fitted;
    CenterlineFitConfig fc;
    fc.min_inliers = min_points; fc.ransac_threshold_m = config_.residual_threshold_m;
    const CenterlineFitter fitter(fc);
    for (const auto& row : rows) {
        if (row.points.size() < min_points ||
            row.points.back().x - row.points.front().x < config_.minimum_span_m) continue;
        const auto line = fitter.fit(row.points);
        if (!line.isOk()) continue;
        FittedRow out;
        out.id = row.id; out.slope = line.value().slope; out.intercept_m = line.value().intercept_m;
        double error = 0;
        for (std::size_t i = 0; i < row.points.size(); ++i) {
            const auto& p = row.points[i];
            const double d = std::abs(p.y - out.slope * p.x - out.intercept_m) / std::hypot(1.0, out.slope);
            if (d <= config_.residual_threshold_m) {
                out.inliers.push_back(p); out.bands.push_back(row.bands[i]); error += d * d;
            } else out.rejected.push_back(p);
        }
        if (out.inliers.size() < min_points || out.inliers.back().x - out.inliers.front().x < config_.minimum_span_m) continue;
        out.rms_m = std::sqrt(error / static_cast<double>(out.inliers.size()));
        fitted.push_back(std::move(out));
    }
    return fitted;
}

Result<JointRowFit> RowBoundaryFitter::fitPair(const FittedRow& left, const FittedRow& right) const {
    if (left.inliers.size() < 2 || right.inliers.size() < 2)
        return Status::unavailable("joint fit needs two observed boundaries");
    cv::Vec4d beta(left.slope, left.intercept_m, right.slope, right.intercept_m);
    const double span = std::min(left.inliers.back().x, right.inliers.back().x) -
                        std::max(left.inliers.front().x, right.inliers.front().x);
    if (span <= 0) return Status::unavailable("joint fit has no shared span");
    const double lambda = config_.parallel_soft_weight *
        static_cast<double>(left.inliers.size() + right.inliers.size()) * span * span / 12.0;
    // Four unknowns, soft slope consistency, and Huber IRLS. The two slopes remain independent.
    for (int iteration = 0; iteration < 8; ++iteration) {
        cv::Matx44d normal = cv::Matx44d::zeros();
        cv::Vec4d rhs(0, 0, 0, 0);
        for (int side = 0; side < 2; ++side) {
            const int offset = side * 2;
            const auto& points = side == 0 ? left.inliers : right.inliers;
            for (const auto& p : points) {
                const double residual = std::abs(p.y - beta[offset] * p.x - beta[offset+1]);
                const double weight = residual <= config_.residual_threshold_m ? 1.0 : config_.residual_threshold_m / residual;
                normal(offset,offset) += weight*p.x*p.x; normal(offset,offset+1) += weight*p.x;
                normal(offset+1,offset) += weight*p.x; normal(offset+1,offset+1) += weight;
                rhs[offset] += weight*p.x*p.y; rhs[offset+1] += weight*p.y;
            }
        }
        normal(0,0) += lambda; normal(2,2) += lambda;
        normal(0,2) -= lambda; normal(2,0) -= lambda;
        cv::Vec4d updated;
        if (!cv::solve(normal, rhs, updated, cv::DECOMP_CHOLESKY))
            return Status::unavailable("joint row fit is degenerate");
        const double change = cv::norm(updated-beta);
        beta = updated;
        if (change < 1e-8) break;
    }
    double error = 0;
    for (int side = 0; side < 2; ++side) for (const auto& p : side == 0 ? left.inliers : right.inliers) {
        const double d = (p.y - beta[2*side]*p.x - beta[2*side+1]) / std::hypot(1.0,beta[2*side]);
        error += d*d;
    }
    const double rms = std::sqrt(error / static_cast<double>(left.inliers.size()+right.inliers.size()));
    if (!std::isfinite(rms) || rms > config_.residual_threshold_m)
        return Status::unavailable("joint fit residual too large");
    return JointRowFit{beta[0], beta[1], beta[2], beta[3], rms};
}
} // namespace rk_nav
