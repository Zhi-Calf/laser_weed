#pragma once

#include <memory>
#include "rk_nav/inference/segmenter.hpp"

namespace rk_nav {

class OnnxYoloV8Segmenter final : public Segmenter {
public:
    OnnxYoloV8Segmenter();
    ~OnnxYoloV8Segmenter() override;
    OnnxYoloV8Segmenter(const OnnxYoloV8Segmenter&) = delete;
    OnnxYoloV8Segmenter& operator=(const OnnxYoloV8Segmenter&) = delete;

    Status load(const ModelConfig& config) override;
    Result<SegmentationResult> infer(const Frame& frame) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rk_nav
