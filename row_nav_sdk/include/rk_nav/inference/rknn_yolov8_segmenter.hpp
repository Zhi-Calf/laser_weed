#pragma once

#include <memory>
#include <vector>

#include "rk_nav/inference/segmenter.hpp"
#include "rk_nav/inference/yolov8_seg_postprocess.hpp"

namespace rk_nav {

struct RknnPreparedInput {
    Frame frame;
    LetterboxResult transform;
    cv::Mat rgb;
    InferenceTiming timing;
};

struct RknnRawOutput {
    Frame frame;
    LetterboxResult transform;
    std::vector<cv::Mat> outputs;
    InferenceTiming timing;
};

class RknnYoloV8Segmenter final : public Segmenter {
public:
    RknnYoloV8Segmenter();
    ~RknnYoloV8Segmenter() override;
    RknnYoloV8Segmenter(const RknnYoloV8Segmenter&) = delete;
    RknnYoloV8Segmenter& operator=(const RknnYoloV8Segmenter&) = delete;

    Status load(const ModelConfig& config) override;
    Result<SegmentationResult> infer(const Frame& frame) override;
    Result<RknnPreparedInput> preprocess(const Frame& frame) const;
    Result<RknnRawOutput> execute(RknnPreparedInput input);
    Result<SegmentationResult> postprocess(RknnRawOutput output) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rk_nav
