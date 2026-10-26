#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "rk_nav/core/frame.hpp"
#include "rk_nav/core/result.hpp"
#include "rk_nav/inference/segmenter.hpp"

namespace rk_nav {

struct LetterboxResult {
    cv::Mat image;
    float scale{1.0F};
    int left{0};
    int top{0};
    int resized_width{0};
    int resized_height{0};
};

LetterboxResult letterboxImage(const cv::Mat& source, int width, int height);

Result<SegmentationResult> decodeRockchipYoloV8Seg(
    const std::vector<cv::Mat>& outputs,
    const Frame& frame,
    const ModelConfig& config,
    const LetterboxResult& transform,
    double inference_ms);

}  // namespace rk_nav
