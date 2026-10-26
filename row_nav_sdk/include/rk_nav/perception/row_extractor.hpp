#pragma once

#include <cstdint>
#include <vector>
#include <string>

#include <opencv2/core.hpp>

#include "rk_nav/core/status.hpp"
#include "rk_nav/geometry/bev_projector.hpp"
#include "rk_nav/perception/multi_row.hpp"

namespace rk_nav {

enum class RowExtractionMethod { kNearestBandBaseline, kAssociatedRows };

struct RowExtractorConfig {
    double min_forward_m{0.4};
    double max_forward_m{6.0};
    double min_row_spacing_m{0.25};
    double max_row_spacing_m{1.5};
    int sample_step_pixels{12};
    int band_half_height_pixels{4};
    int min_run_width_pixels{2};
    double min_column_fill_ratio{0.25};
    std::uint64_t min_centerline_points{8};
    RowExtractionMethod method{RowExtractionMethod::kAssociatedRows};
    MultiRowConfig multi_row;
};

struct RowExtraction {
    std::vector<cv::Point2d> centerline_ground_points;
    double median_row_spacing_m{0.0};
    float confidence{0.0F};
    std::uint64_t sampled_bands{0};
    std::uint64_t accepted_bands{0};
    RowExtractionMethod method{RowExtractionMethod::kNearestBandBaseline};
    std::vector<FittedRow> fitted_rows;
    std::vector<cv::Point2d> candidate_points;
    std::vector<cv::Point2d> rejected_points;
    int selected_left{-1}, selected_right{-1}; // Indices into fitted_rows.
    bool navigation_pair_valid{false};
    JointRowFit joint_fit;
    std::string rejection_reason;
};

class RowExtractor {
public:
    explicit RowExtractor(RowExtractorConfig config = {});
    // Associated mode retains diagnostics on no-pair; check navigation_pair_valid.
    Result<RowExtraction> extractBetweenRows(const cv::Mat& bev_mask,
                                             const BevProjector& projector) const;
    [[nodiscard]] const RowExtractorConfig& config() const;
    [[nodiscard]] Status validateConfig() const;

private:
    Result<RowExtraction> extractAssociated(const cv::Mat& mask, const BevProjector& projector) const;
    RowExtractorConfig config_;
};

// Optional row_extractor block in the existing BEV YAML; absent block uses defaults.
Result<RowExtractorConfig> loadRowExtractorConfig(const std::filesystem::path& path);

}  // namespace rk_nav
