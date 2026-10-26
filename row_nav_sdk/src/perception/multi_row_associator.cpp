#include "rk_nav/perception/multi_row.hpp"
#include <algorithm>
#include <cmath>
#include <tuple>

namespace rk_nav {
namespace {
double predict(const AssociatedRow& row, double x) {
    // Recent points keep association local; robust fitting follows separately.
    const auto begin = row.points.size() > 5 ? row.points.size() - 5 : 0;
    const auto& last = row.points.back();
    double numerator = 0, denominator = 0;
    for (auto i = begin; i < row.points.size(); ++i) {
        const double dx = row.points[i].x - last.x;
        numerator += dx * (row.points[i].y - last.y);
        denominator += dx * dx;
    }
    const double slope = denominator > 1e-12 ? numerator / denominator : 0.0;
    return last.y + slope * (x - last.x);
}
}
std::vector<AssociatedRow> MultiRowAssociator::associate(const std::vector<RowBand>& bands) const {
    std::vector<AssociatedRow> rows;
    for (const auto& band : bands) {
        std::vector<std::tuple<double, std::size_t, std::size_t>> edges;
        for (std::size_t r = 0; r < rows.size(); ++r) {
            const double dx = band.forward_m - rows[r].points.back().x;
            if (dx <= 0 || dx > config_.maximum_gap_m) continue;
            for (std::size_t p = 0; p < band.points.size(); ++p) {
                const double distance = std::abs(band.points[p].y - predict(rows[r], band.forward_m));
                if (distance <= config_.association_gate_m) edges.emplace_back(distance, r, p);
            }
        }
        std::sort(edges.begin(), edges.end());
        std::vector<bool> used_rows(rows.size(), false), used_points(band.points.size(), false);
        std::vector<std::pair<double,double>> ordering;
        for (const auto& edge : edges) {
            const auto r = std::get<1>(edge), p = std::get<2>(edge);
            if (used_rows[r] || used_points[p]) continue;
            const double previous_y = rows[r].points.back().y, current_y = band.points[p].y;
            const bool crosses = std::any_of(ordering.begin(), ordering.end(), [&](const auto& match) {
                return (previous_y-match.first)*(current_y-match.second) <= 0;
            });
            if (crosses) continue;
            ordering.emplace_back(previous_y,current_y);
            rows[r].points.push_back(band.points[p]); rows[r].bands.push_back(band.index);
            used_rows[r] = used_points[p] = true;
        }
        for (std::size_t p = 0; p < band.points.size(); ++p) if (!used_points[p]) {
            AssociatedRow row;
            row.id = rows.size(); row.points.push_back(band.points[p]); row.bands.push_back(band.index);
            rows.push_back(std::move(row));
        }
    }
    return rows;
}

Result<cv::Vec2i> NavigationRowSelector::select(const std::vector<FittedRow>& rows,
    double min_spacing_m, double max_spacing_m, const MultiRowConfig& config, double near_m) const {
    int left = -1, right = -1;
    // Select the adjacent pair at vehicle x=0 once for the entire frame.
    for (std::size_t i = 0; i < rows.size(); ++i) {
        // Avoid extrapolating a distant fragment all the way to the vehicle.
        if (rows[i].inliers.empty() || rows[i].inliers.front().x > near_m + config.maximum_gap_m) continue;
        const double y = rows[i].intercept_m;
        if (y > 0 && (left < 0 || y < rows[static_cast<std::size_t>(left)].intercept_m)) left = static_cast<int>(i);
        if (y < 0 && (right < 0 || y > rows[static_cast<std::size_t>(right)].intercept_m)) right = static_cast<int>(i);
    }
    if (left < 0 || right < 0) return Status::unavailable("no observed row pair brackets vehicle");
    const auto& l = rows[static_cast<std::size_t>(left)];
    const auto& r = rows[static_cast<std::size_t>(right)];
    if (std::abs(std::atan(l.slope) - std::atan(r.slope)) > config.maximum_heading_difference_rad)
        return Status::unavailable("adjacent row directions disagree");
    const double near_x = std::max(l.inliers.front().x, r.inliers.front().x);
    const double far_x = std::min(l.inliers.back().x, r.inliers.back().x);
    if (far_x - near_x < config.minimum_span_m)
        return Status::unavailable("adjacent rows have insufficient shared span");
    for (const double x : {near_x, far_x}) {
        const double spacing = (l.slope - r.slope) * x + l.intercept_m - r.intercept_m;
        if (spacing < min_spacing_m || spacing > max_spacing_m ||
            (config.expected_spacing_m > 0 && std::abs(spacing - config.expected_spacing_m) > config.spacing_tolerance_m))
            return Status::unavailable("adjacent row spacing invalid; do not skip to an outer row");
    }
    return cv::Vec2i(left, right);
}
} // namespace rk_nav
