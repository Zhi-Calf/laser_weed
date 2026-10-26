#include "rk_nav/perception/row_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace rk_nav {
namespace {

struct PixelRun {
    double center{0.0};
    int width{0};
};

std::vector<PixelRun> findRuns(const cv::Mat& mask, int center_v, int half_height,
                               double fill_ratio, int minimum_width) {
    const int top = std::max(0, center_v - half_height);
    const int bottom = std::min(mask.rows - 1, center_v + half_height);
    const int band_height = bottom - top + 1;
    std::vector<bool> active(static_cast<std::size_t>(mask.cols), false);
    for (int column = 0; column < mask.cols; ++column) {
        int filled = 0;
        for (int row = top; row <= bottom; ++row) {
            if (mask.at<std::uint8_t>(row, column) != 0) ++filled;
        }
        active[static_cast<std::size_t>(column)] =
            static_cast<double>(filled) / static_cast<double>(band_height) >= fill_ratio;
    }

    std::vector<PixelRun> runs;
    int start = -1;
    for (int column = 0; column <= mask.cols; ++column) {
        const bool enabled = column < mask.cols && active[static_cast<std::size_t>(column)];
        if (enabled && start < 0) start = column;
        if (!enabled && start >= 0) {
            const int width = column - start;
            if (width >= minimum_width) {
                runs.push_back({0.5 * static_cast<double>(start + column - 1), width});
            }
            start = -1;
        }
    }
    return runs;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    double result = *middle;
    if (values.size() % 2 == 0) {
        const auto lower = std::max_element(values.begin(), middle);
        result = 0.5 * (result + *lower);
    }
    return result;
}

}  // namespace

RowExtractor::RowExtractor(RowExtractorConfig config) : config_(std::move(config)) {}

Status RowExtractor::validateConfig() const {
    const auto& m = config_.multi_row;
    if (!std::isfinite(config_.min_forward_m) || !std::isfinite(config_.max_forward_m) ||
        !std::isfinite(config_.min_row_spacing_m) || !std::isfinite(config_.max_row_spacing_m) ||
        !std::isfinite(config_.min_column_fill_ratio) ||
        config_.min_forward_m < 0.0 || config_.max_forward_m <= config_.min_forward_m ||
        config_.min_row_spacing_m <= 0.0 ||
        config_.max_row_spacing_m <= config_.min_row_spacing_m ||
        config_.sample_step_pixels <= 0 || config_.band_half_height_pixels < 0 ||
        config_.min_run_width_pixels <= 0 || config_.min_column_fill_ratio <= 0.0 ||
        config_.min_column_fill_ratio > 1.0 || config_.min_centerline_points < 2) {
        return Status::invalidArgument("row extractor configuration is invalid");
    }
    if (config_.method == RowExtractionMethod::kAssociatedRows) {
        if (!std::isfinite(m.association_gate_m) || m.association_gate_m <= 0 ||
            !std::isfinite(m.maximum_gap_m) || m.maximum_gap_m <= 0 ||
            !std::isfinite(m.minimum_span_m) || m.minimum_span_m <= 0 ||
            !std::isfinite(m.maximum_run_width_m) || m.maximum_run_width_m <= 0 ||
            !std::isfinite(m.residual_threshold_m) || m.residual_threshold_m <= 0 ||
            !std::isfinite(m.maximum_heading_difference_rad) || m.maximum_heading_difference_rad <= 0 ||
            !std::isfinite(m.parallel_soft_weight) || m.parallel_soft_weight < 0 ||
            !std::isfinite(m.expected_spacing_m) || m.expected_spacing_m < 0 ||
            !std::isfinite(m.spacing_tolerance_m) || m.spacing_tolerance_m <= 0)
            return Status::invalidArgument("multi-row configuration is invalid");
        return Status::ok();
    }
    if (config_.method != RowExtractionMethod::kNearestBandBaseline)
        return Status::invalidArgument("unknown row extraction method");
    return Status::ok();
}

Result<RowExtractorConfig> loadRowExtractorConfig(const std::filesystem::path& path) {
    try {
        cv::FileStorage storage(path.string(),cv::FileStorage::READ);
        if (!storage.isOpened()) return Status::notFound("cannot open row extraction configuration");
        RowExtractorConfig c;
        const auto node=storage["row_extractor"];
        if (!node.empty()) {
            if (!node.isMap()) return Status::invalidArgument("row_extractor must be a mapping");
            if (!node["method"].empty()) {
                std::string method; node["method"] >> method;
                if (method=="associated") c.method=RowExtractionMethod::kAssociatedRows;
                else if (method=="nearest_band") c.method=RowExtractionMethod::kNearestBandBaseline;
                else return Status::invalidArgument("unknown row extraction method");
            }
            const auto read = [&](const char* key,double& value) {
                if (!node[key].empty()) {
                    if (!node[key].isReal() && !node[key].isInt()) throw std::invalid_argument("row parameter must be numeric");
                    node[key] >> value;
                }
            };
            read("min_forward_m",c.min_forward_m); read("max_forward_m",c.max_forward_m);
            read("min_row_spacing_m",c.min_row_spacing_m); read("max_row_spacing_m",c.max_row_spacing_m);
            read("association_gate_m",c.multi_row.association_gate_m);
            read("maximum_gap_m",c.multi_row.maximum_gap_m);
            read("minimum_span_m",c.multi_row.minimum_span_m);
            read("maximum_run_width_m",c.multi_row.maximum_run_width_m);
            read("residual_threshold_m",c.multi_row.residual_threshold_m);
            read("maximum_heading_difference_rad",c.multi_row.maximum_heading_difference_rad);
            read("parallel_soft_weight",c.multi_row.parallel_soft_weight);
            read("expected_spacing_m",c.multi_row.expected_spacing_m);
            read("spacing_tolerance_m",c.multi_row.spacing_tolerance_m);
        }
        const auto valid=RowExtractor(c).validateConfig();
        if (!valid.isOk()) return valid;
        return c;
    } catch (const std::exception& e) { return Status::invalidArgument(e.what()); }
}

Result<RowExtraction> RowExtractor::extractBetweenRows(const cv::Mat& input_mask,
                                                       const BevProjector& projector) const {
    if (input_mask.empty() || input_mask.type() != CV_8UC1) {
        return Status::invalidArgument("row extractor expects a non-empty CV_8UC1 BEV mask");
    }
    if (input_mask.size() != projector.config().output_size) {
        return Status::invalidArgument("BEV mask size does not match projector output size");
    }
    const auto valid=validateConfig();
    if (!valid.isOk()) return valid;
    if (!projector.validate().isOk()) return projector.validate();
    if (!std::isfinite(projector.config().vehicle_origin_u) || !std::isfinite(projector.config().vehicle_origin_v))
        return Status::invalidArgument("non-finite vehicle origin");
    if (config_.method==RowExtractionMethod::kAssociatedRows) return extractAssociated(input_mask,projector);

    cv::Mat mask;
    cv::threshold(input_mask, mask, 0, 255, cv::THRESH_BINARY);
    RowExtraction result;
    std::vector<double> spacings;
    const auto& bev = projector.config();
    const int near_v = std::clamp(
        static_cast<int>(std::lround(bev.vehicle_origin_v - config_.min_forward_m / bev.meters_per_pixel)),
        0, mask.rows - 1);
    const int far_v = std::clamp(
        static_cast<int>(std::lround(bev.vehicle_origin_v - config_.max_forward_m / bev.meters_per_pixel)),
        0, mask.rows - 1);

    for (int v = near_v; v >= far_v; v -= config_.sample_step_pixels) {
        ++result.sampled_bands;
        const auto runs = findRuns(mask, v, config_.band_half_height_pixels,
                                   config_.min_column_fill_ratio, config_.min_run_width_pixels);
        const PixelRun* image_left = nullptr;
        const PixelRun* image_right = nullptr;
        for (const auto& run : runs) {
            if (run.center < bev.vehicle_origin_u &&
                (image_left == nullptr || run.center > image_left->center)) {
                image_left = &run;
            } else if (run.center > bev.vehicle_origin_u &&
                       (image_right == nullptr || run.center < image_right->center)) {
                image_right = &run;
            }
        }
        if (image_left == nullptr || image_right == nullptr) continue;

        const double spacing_m = (image_right->center - image_left->center) * bev.meters_per_pixel;
        if (spacing_m < config_.min_row_spacing_m || spacing_m > config_.max_row_spacing_m) continue;
        const double center_u = 0.5 * (image_left->center + image_right->center);
        result.centerline_ground_points.push_back(projector.bevToGround({center_u, static_cast<double>(v)}));
        spacings.push_back(spacing_m);
        ++result.accepted_bands;
    }

    if (result.centerline_ground_points.size() < config_.min_centerline_points) {
        return Status::unavailable("not enough BEV bands contain a valid pair of crop rows");
    }
    result.median_row_spacing_m = median(spacings);
    result.navigation_pair_valid = true;
    result.confidence = result.sampled_bands == 0
                            ? 0.0F
                            : static_cast<float>(static_cast<double>(result.accepted_bands) /
                                                 static_cast<double>(result.sampled_bands));
    return result;
}

Result<RowExtraction> RowExtractor::extractAssociated(const cv::Mat& mask, const BevProjector& projector) const {
    const auto bands = RowCandidateExtractor{}.extract(mask, projector,
        config_.min_forward_m, config_.max_forward_m, config_.sample_step_pixels,
        config_.band_half_height_pixels, config_.min_column_fill_ratio,
        config_.min_run_width_pixels, config_.multi_row.maximum_run_width_m);
    const auto associated = MultiRowAssociator(config_.multi_row).associate(bands);
    const RowBoundaryFitter fitter(config_.multi_row);
    RowExtraction out;
    out.method = RowExtractionMethod::kAssociatedRows;
    out.sampled_bands = bands.size();
    for (const auto& band : bands) for (const auto& p : band.points) out.candidate_points.push_back(p);
    out.fitted_rows = fitter.fit(associated, config_.min_centerline_points);
    // Expose candidates rejected by association support or robust row fitting.
    for (const auto& row : associated) {
        const auto found = std::find_if(out.fitted_rows.begin(), out.fitted_rows.end(),
            [&](const FittedRow& fit) { return fit.id == row.id; });
        const auto& rejected = found == out.fitted_rows.end() ? row.points : found->rejected;
        out.rejected_points.insert(out.rejected_points.end(), rejected.begin(), rejected.end());
    }
    const auto selected = NavigationRowSelector{}.select(out.fitted_rows,
        config_.min_row_spacing_m, config_.max_row_spacing_m, config_.multi_row, config_.min_forward_m);
    if (!selected.isOk()) { out.rejection_reason = selected.status().message(); return out; }
    out.selected_left = selected.value()[0]; out.selected_right = selected.value()[1];
    const auto& left = out.fitted_rows[static_cast<std::size_t>(out.selected_left)];
    const auto& right = out.fitted_rows[static_cast<std::size_t>(out.selected_right)];
    const auto joint = fitter.fitPair(left, right);
    if (!joint.isOk()) { out.rejection_reason = joint.status().message(); return out; }
    out.joint_fit = joint.value();
    std::vector<double> spacings;
    for (std::size_t l = 0, r = 0; l < left.bands.size() && r < right.bands.size();) {
        if (left.bands[l] < right.bands[r]) { ++l; continue; }
        if (right.bands[r] < left.bands[l]) { ++r; continue; }
        const double x = left.inliers[l].x;
        const double yl = out.joint_fit.left_slope*x + out.joint_fit.left_intercept_m;
        const double yr = out.joint_fit.right_slope*x + out.joint_fit.right_intercept_m;
        const double spacing = yl-yr;
        if (spacing < config_.min_row_spacing_m || spacing > config_.max_row_spacing_m ||
            (config_.multi_row.expected_spacing_m > 0 &&
             std::abs(spacing-config_.multi_row.expected_spacing_m) > config_.multi_row.spacing_tolerance_m))
        {
            out.centerline_ground_points.clear(); out.rejection_reason = "joint row spacing invalid";
            return out;
        }
        spacings.push_back(spacing);
        out.centerline_ground_points.emplace_back(x, 0.5*(yl+yr));
        ++l; ++r;
    }
    if (out.centerline_ground_points.size() < config_.min_centerline_points ||
        out.centerline_ground_points.back().x-out.centerline_ground_points.front().x < config_.multi_row.minimum_span_m) {
        out.centerline_ground_points.clear(); out.rejection_reason = "insufficient common observed support"; return out;
    }
    out.navigation_pair_valid = true;
    out.accepted_bands = out.centerline_ground_points.size();
    out.median_row_spacing_m = median(spacings);
    // Actual common observed support and boundary residual, never inferred missing bands.
    const double residual_score = std::max(0.0, 1.0-out.joint_fit.rms_m/config_.multi_row.residual_threshold_m);
    out.confidence = static_cast<float>(residual_score*double(out.accepted_bands)/double(out.sampled_bands));
    return out;
}

const RowExtractorConfig& RowExtractor::config() const { return config_; }

}  // namespace rk_nav
