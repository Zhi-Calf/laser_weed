#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core/types.hpp>

#include "rk_nav/core/status.hpp"

namespace rk_nav {

struct CenterlineFitConfig {
    double ransac_threshold_m{0.05};
    std::uint64_t ransac_iterations{100};
    std::uint64_t local_optimization_iterations{5};
    std::uint64_t min_inliers{8};
    double lookahead_m{2.0};
};

struct CenterlineFit {
    // Vehicle coordinates: x points forward, y points left. The fitted line is y = slope*x + intercept.
    double slope{0.0};
    double intercept_m{0.0};
    double heading_error_rad{0.0};
    double lateral_error_m{0.0};
    double lookahead_offset_m{0.0};
    double rms_residual_m{0.0};
    float confidence{0.0F};
    std::uint64_t inlier_count{0};
    std::uint64_t point_count{0};
    std::uint64_t refinement_steps{0};
};

class CenterlineFitter {
public:
    explicit CenterlineFitter(CenterlineFitConfig config = {});
    Result<CenterlineFit> fit(const std::vector<cv::Point2d>& ground_points) const;
    [[nodiscard]] const CenterlineFitConfig& config() const;

private:
    CenterlineFitConfig config_;
};

}  // namespace rk_nav
