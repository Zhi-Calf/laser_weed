#include "rk_nav/perception/multi_row.hpp"
#include <algorithm>
#include <cmath>

namespace rk_nav {
std::vector<RowBand> RowCandidateExtractor::extract(const cv::Mat& mask,
    const BevProjector& projector, double near_m, double far_m, int step,
    int half_height, double fill_ratio, int min_width, double max_width_m) const {
    const auto& bev = projector.config();
    const int near_v = std::clamp(static_cast<int>(std::lround(
        bev.vehicle_origin_v - near_m / bev.meters_per_pixel)), 0, mask.rows - 1);
    const int far_v = std::clamp(static_cast<int>(std::lround(
        bev.vehicle_origin_v - far_m / bev.meters_per_pixel)), 0, mask.rows - 1);
    std::vector<RowBand> result;
    for (int v = near_v; v >= far_v; v -= step) {
        RowBand band;
        band.index = result.size();
        band.forward_m = projector.bevToGround({0.0, static_cast<double>(v)}).x;
        const int top = std::max(0, v - half_height), bottom = std::min(mask.rows - 1, v + half_height);
        int start = -1;
        for (int u = 0; u <= mask.cols; ++u) {
            int filled = 0;
            if (u < mask.cols) for (int row = top; row <= bottom; ++row)
                if (mask.at<std::uint8_t>(row, u)) ++filled;
            const bool active = u < mask.cols && double(filled) / (bottom - top + 1) >= fill_ratio;
            if (active && start < 0) start = u;
            if (!active && start >= 0) {
                const int width = u - start;
                if (width >= min_width && width * bev.meters_per_pixel <= max_width_m)
                    band.points.push_back(projector.bevToGround({0.5 * (start + u - 1), double(v)}));
                start = -1;
            }
        }
        result.push_back(std::move(band));
    }
    return result;
}
} // namespace rk_nav
