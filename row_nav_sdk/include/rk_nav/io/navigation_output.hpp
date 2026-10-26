#pragma once

#include <filesystem>
#include <string>

#include <opencv2/core.hpp>

#include "rk_nav/core/status.hpp"
#include "rk_nav/inference/segmenter.hpp"
#include "rk_nav/pipeline/perception_pipeline.hpp"

namespace rk_nav {

[[nodiscard]] const char* trackingStatusName(TrackingStatus status);
Result<cv::Mat> renderImageOverlay(const cv::Mat& image,
                                   const SegmentationResult& segmentation,
                                   const NavigationObservation& navigation);
Result<cv::Mat> renderBevOverlay(const PerceptionOutput& output,
                                 const BevProjector& projector);
Status writeNavigationJson(const std::filesystem::path& path,
                           const NavigationObservation& navigation,
                           const std::optional<RowExtraction>& rows,
                           const std::optional<CenterlineFit>& centerline);
Result<std::string> serializeNavigationJson(
    const NavigationObservation& navigation,
    const std::optional<RowExtraction>& rows,
    const std::optional<CenterlineFit>& centerline,
    bool pretty = true);

}  // namespace rk_nav
