#pragma once

#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include "rk_nav/core/status.hpp"
#include "rk_nav/geometry/bev_projector.hpp"

namespace rk_nav {

struct MultiRowConfig {
    double association_gate_m{0.08};
    double maximum_gap_m{0.65};
    double minimum_span_m{1.0};
    double maximum_run_width_m{0.18};
    double residual_threshold_m{0.05};
    double maximum_heading_difference_rad{0.15};
    double parallel_soft_weight{0.1};
    // Zero means no spacing prior. Set this from the actual planting pattern.
    double expected_spacing_m{0.0};
    double spacing_tolerance_m{0.1};
};

struct RowBand {
    std::uint64_t index{0};
    double forward_m{0.0};
    std::vector<cv::Point2d> points;
};

struct AssociatedRow {
    std::uint64_t id{0}; // Frame-local only; not a persistent track ID.
    std::vector<cv::Point2d> points;
    std::vector<std::uint64_t> bands;
};

struct FittedRow {
    std::uint64_t id{0};
    std::uint64_t track_id{0}; // Zero if temporal tracking did not accept this observation.
    double slope{0.0};
    double intercept_m{0.0};
    double rms_m{0.0};
    std::vector<cv::Point2d> inliers;
    std::vector<std::uint64_t> bands;
    std::vector<cv::Point2d> rejected;
};

struct JointRowFit {
    double left_slope{0.0}, left_intercept_m{0.0};
    double right_slope{0.0}, right_intercept_m{0.0};
    double rms_m{0.0};
};

class RowCandidateExtractor {
public:
    std::vector<RowBand> extract(const cv::Mat& mask, const BevProjector& projector,
        double near_m, double far_m, int step, int half_height, double fill_ratio,
        int min_width, double max_width_m) const;
};

class MultiRowAssociator {
public:
    explicit MultiRowAssociator(MultiRowConfig config = {}) : config_(config) {}
    std::vector<AssociatedRow> associate(const std::vector<RowBand>& bands) const;
private:
    MultiRowConfig config_;
};

class RowBoundaryFitter {
public:
    explicit RowBoundaryFitter(MultiRowConfig config = {}) : config_(config) {}
    std::vector<FittedRow> fit(const std::vector<AssociatedRow>& rows, std::uint64_t min_points) const;
    Result<JointRowFit> fitPair(const FittedRow& left, const FittedRow& right) const;
private:
    MultiRowConfig config_;
};

class NavigationRowSelector {
public:
    // Return indices into rows; left is positive vehicle y, right negative.
    Result<cv::Vec2i> select(const std::vector<FittedRow>& rows, double min_spacing_m,
                            double max_spacing_m, const MultiRowConfig& config, double near_m) const;
};

} // namespace rk_nav
