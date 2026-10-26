#include "rk_nav/io/navigation_output.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rk_nav {

const char* trackingStatusName(TrackingStatus status) {
    switch (status) {
        case TrackingStatus::kValid: return "valid";
        case TrackingStatus::kLowConfidence: return "low_confidence";
        case TrackingStatus::kNoRow: return "no_row";
        case TrackingStatus::kInputError: return "input_error";
        case TrackingStatus::kInferenceError: return "inference_error";
        case TrackingStatus::kStaleFrame: return "stale_frame";
    }
    return "unknown";
}

Result<cv::Mat> renderImageOverlay(const cv::Mat& image,
                                   const SegmentationResult& segmentation,
                                   const NavigationObservation& navigation) {
    if (image.empty()) return Status::invalidArgument("cannot visualize an empty image");
    if (segmentation.combined_mask.empty() || segmentation.combined_mask.size() != image.size()) {
        return Status::invalidArgument("segmentation mask size does not match image");
    }
    cv::Mat color;
    if (image.channels() == 3) color = image.clone();
    else cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
    cv::Mat tint = color.clone();
    tint.setTo(cv::Scalar(40, 190, 40), segmentation.combined_mask);
    cv::addWeighted(tint, 0.4, color, 0.6, 0.0, color);

    std::ostringstream label;
    label << trackingStatusName(navigation.status) << "  e=" << std::fixed << std::setprecision(3)
          << navigation.lateral_error_m << "m  heading="
          << navigation.heading_error_rad * 180.0 / CV_PI << "deg  conf="
          << std::setprecision(2) << navigation.confidence;
    cv::rectangle(color, cv::Rect(0, 0, color.cols, 42), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(color, label.str(), {12, 29}, cv::FONT_HERSHEY_SIMPLEX, 0.72,
                navigation.status == TrackingStatus::kValid ? cv::Scalar(60, 230, 60)
                                                            : cv::Scalar(40, 180, 255),
                2, cv::LINE_AA);
    return color;
}

Result<cv::Mat> renderBevOverlay(const PerceptionOutput& output,
                                 const BevProjector& projector) {
    if (output.bev_mask.empty()) return Status::invalidArgument("cannot visualize an empty BEV mask");
    cv::Mat color;
    cv::cvtColor(output.bev_mask, color, cv::COLOR_GRAY2BGR);
    if (output.rows.has_value()) {
        const auto& rows = *output.rows;
        for (const auto& row : rows.fitted_rows) {
            const bool selected = (rows.selected_left >= 0 && static_cast<std::size_t>(rows.selected_left) < rows.fitted_rows.size() &&
                &row == &rows.fitted_rows[static_cast<std::size_t>(rows.selected_left)]) ||
                (rows.selected_right >= 0 && static_cast<std::size_t>(rows.selected_right) < rows.fitted_rows.size() &&
                &row == &rows.fitted_rows[static_cast<std::size_t>(rows.selected_right)]);
            const auto tint = selected ? cv::Scalar(255, 200, 0) : cv::Scalar(180, 80, 180);
            for (const auto& p : row.inliers) cv::circle(color, projector.groundToBev(p), 2, tint, cv::FILLED);
            if (row.inliers.size() >= 2) {
                const double a = row.inliers.front().x, b = row.inliers.back().x;
                cv::line(color, projector.groundToBev({a,row.slope*a+row.intercept_m}),
                    projector.groundToBev({b,row.slope*b+row.intercept_m}), tint, 1, cv::LINE_AA);
                const auto label=std::to_string(row.id)+(row.track_id ? "/T"+std::to_string(row.track_id) : "");
                cv::putText(color, label, projector.groundToBev(row.inliers.front()),
                    cv::FONT_HERSHEY_SIMPLEX, .4, tint, 1);
            }
        }
        for (const auto& p : rows.rejected_points)
            cv::drawMarker(color, projector.groundToBev(p), cv::Scalar(0,0,255), cv::MARKER_TILTED_CROSS, 6, 1);
        for (const auto& point : output.rows->centerline_ground_points) {
            cv::circle(color, projector.groundToBev(point), 2,
                       cv::Scalar(0, 180, 255), cv::FILLED, cv::LINE_AA);
        }
    }
    if (output.centerline.has_value() && output.rows.has_value() && !output.rows->centerline_ground_points.empty()) {
        const auto& points = output.rows->centerline_ground_points;
        const auto limits = std::minmax_element(points.begin(), points.end(),
            [](const cv::Point2d& a, const cv::Point2d& b) { return a.x < b.x; });
        const auto fitted = [&](double forward) {
            return cv::Point2d(forward, output.centerline->slope * forward + output.centerline->intercept_m);
        };
        cv::line(color, projector.groundToBev(fitted(limits.first->x)),
                 projector.groundToBev(fitted(limits.second->x)),
                 cv::Scalar(60, 255, 60), 3, cv::LINE_AA);
    }
    cv::circle(color, projector.groundToBev({0.0, 0.0}), 7,
               cv::Scalar(255, 80, 40), cv::FILLED, cv::LINE_AA);
    if (output.navigation.tracking.state!=RowTrackingState::kDisabled) {
        const auto& nav=output.navigation;
        cv::putText(color,rowTrackingStateName(nav.tracking.state),{8,22},cv::FONT_HERSHEY_SIMPLEX,.6,
            cv::Scalar(0,180,255),1,cv::LINE_AA);
        if (nav.status==TrackingStatus::kValid && output.rows && !output.rows->centerline_ground_points.empty()) {
            const auto& points=output.rows->centerline_ground_points;
            const auto projected=[&](double x) { return projector.groundToBev({x,nav.lateral_error_m+std::tan(nav.heading_error_rad)*x}); };
            cv::line(color,projected(points.front().x),projected(points.back().x),cv::Scalar(255,100,0),2,cv::LINE_AA);
        }
    }
    return color;
}

Status writeNavigationJson(const std::filesystem::path& path,
                           const NavigationObservation& navigation,
                           const std::optional<RowExtraction>& rows,
                           const std::optional<CenterlineFit>& centerline) {
    std::ofstream stream(path);
    if (!stream) return Status::unavailable("cannot write navigation JSON: " + path.string());
    const auto serialized = serializeNavigationJson(navigation, rows, centerline, true);
    if (!serialized.isOk()) return serialized.status();
    stream << serialized.value() << '\n';
    if (!stream) return Status::unavailable("failed while writing navigation JSON: " + path.string());
    return Status::ok();
}

Result<std::string> serializeNavigationJson(
    const NavigationObservation& navigation,
    const std::optional<RowExtraction>& rows,
    const std::optional<CenterlineFit>& centerline,
    bool pretty) {
    const char* newline = pretty ? "\n" : "";
    const char* indent = pretty ? "  " : "";
    const char* separator = pretty ? ": " : ":";
    const auto number=[](double value) {
        if (!std::isfinite(value)) return std::string("null");
        std::ostringstream s; s<<std::fixed<<std::setprecision(6)<<value; return s.str();
    };
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "{" << newline
           << indent << "\"sequence\"" << separator << navigation.sequence << "," << newline
           << indent << "\"capture_time_ns\"" << separator << navigation.capture_time_ns << "," << newline
           << indent << "\"status\"" << separator << "\"" << trackingStatusName(navigation.status) << "\"," << newline
           << indent << "\"lateral_error_m\"" << separator << number(navigation.lateral_error_m) << "," << newline
           << indent << "\"heading_error_rad\"" << separator << number(navigation.heading_error_rad) << "," << newline
           << indent << "\"lookahead_m\"" << separator << number(navigation.lookahead_m) << "," << newline
           << indent << "\"confidence\"" << separator << number(navigation.confidence) << "," << newline
           << indent << "\"latency_ms\"" << separator << number(navigation.latency_ms) << "," << newline
           << indent << "\"accepted_bands\"" << separator << (rows ? rows->accepted_bands : 0) << "," << newline
           << indent << "\"median_row_spacing_m\"" << separator << (rows ? rows->median_row_spacing_m : 0.0) << "," << newline
           << indent << "\"fit_inliers\"" << separator << (centerline ? centerline->inlier_count : 0) << "," << newline
           << indent << "\"fit_refinement_steps\"" << separator << (centerline ? centerline->refinement_steps : 0) << "," << newline
           << indent << "\"fit_rms_residual_m\"" << separator << (centerline ? centerline->rms_residual_m : 0.0);
    if (rows) {
        const auto points = [&](const std::vector<cv::Point2d>& values) {
            stream << '[';
            for (std::size_t i=0; i<values.size(); ++i) {
                if (i) stream << ',';
                stream << '[' << values[i].x << ',' << values[i].y << ']';
            }
            stream << ']';
        };
        stream << "," << newline << indent << "\"row_method\"" << separator << '"'
               << (rows->method == RowExtractionMethod::kAssociatedRows ? "associated" : "nearest_band") << '"'
               << "," << newline << indent << "\"pair_valid\"" << separator << (rows->navigation_pair_valid ? "true" : "false")
               << "," << newline << indent << "\"selected_left\"" << separator << rows->selected_left
               << "," << newline << indent << "\"selected_right\"" << separator << rows->selected_right
               << "," << newline << indent << "\"rejection_reason\"" << separator << std::quoted(rows->rejection_reason)
               << "," << newline << indent << "\"joint_rms_m\"" << separator << rows->joint_fit.rms_m
               << "," << newline << indent << "\"joint_boundaries\"" << separator << '['
               << rows->joint_fit.left_slope << ',' << rows->joint_fit.left_intercept_m << ','
               << rows->joint_fit.right_slope << ',' << rows->joint_fit.right_intercept_m << ']'
               << "," << newline << indent << "\"candidate_points\"" << separator;
        points(rows->candidate_points);
        stream << "," << newline << indent << "\"rejected_points\"" << separator;
        points(rows->rejected_points);
        stream << "," << newline << indent << "\"centerline_points\"" << separator;
        points(rows->centerline_ground_points);
        stream << "," << newline << indent << "\"rows\"" << separator << '[';
        for (std::size_t i=0; i<rows->fitted_rows.size(); ++i) {
            if (i) stream << ',';
            const auto& row = rows->fitted_rows[i];
            stream << "{\"id\":" << row.id << ",\"track_id\":" << row.track_id << ",\"slope\":" << row.slope
                   << ",\"intercept_m\":" << row.intercept_m << ",\"rms_m\":" << row.rms_m << ",\"points\":";
            points(row.inliers); stream << '}';
        }
        stream << ']';
    }
    if (navigation.tracking.state!=RowTrackingState::kDisabled) {
        const auto& t=navigation.tracking;
        stream<<','<<newline<<indent<<"\"tracking\""<<separator
            <<"{\"state\":"<<std::quoted(rowTrackingStateName(t.state))
            <<",\"raw_valid\":"<<(t.raw_valid?"true":"false")
            <<",\"raw_lateral_m\":"<<number(t.raw_lateral_m)<<",\"raw_heading_rad\":"<<number(t.raw_heading_rad)
            <<",\"prediction_available\":"<<(t.prediction_available?"true":"false")
            <<",\"predicted_lateral_m\":"<<(t.prediction_available ? number(t.predicted_lateral_m) : "null")
            <<",\"predicted_heading_rad\":"<<(t.prediction_available ? number(t.predicted_heading_rad) : "null")
            <<",\"reset_after_gap\":"<<(t.reset_after_gap?"true":"false")
            <<",\"left_id\":"<<t.left_id<<",\"right_id\":"<<t.right_id<<",\"tracks\":[";
        for (std::size_t i=0;i<t.tracks.size();++i) {
            if (i) stream<<',';
            const auto& track=t.tracks[i];
            stream<<"{\"id\":"<<track.id<<",\"lateral_m\":"<<number(track.lateral_m)
                <<",\"heading_rad\":"<<number(track.heading_rad)<<",\"missed_frames\":"<<track.missed_frames
                <<",\"confirmed\":"<<(track.confirmed?"true":"false")
                <<",\"observed\":"<<(track.observed?"true":"false")<<'}';
        }
        stream<<"]}";
    }
    stream << newline << "}";
    return stream.str();
}

}  // namespace rk_nav
