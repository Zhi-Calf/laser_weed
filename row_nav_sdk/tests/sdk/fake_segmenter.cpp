// Only the neural-network backend is replaced: real remap and navigation run.
#include "rk_nav/inference/onnx_yolov8_segmenter.hpp"
#include <opencv2/core.hpp>
#include <stdexcept>
#include <chrono>
#include <thread>
namespace rk_nav {
class OnnxYoloV8Segmenter::Impl {};
OnnxYoloV8Segmenter::OnnxYoloV8Segmenter() : impl_(std::make_unique<Impl>()) {}
OnnxYoloV8Segmenter::~OnnxYoloV8Segmenter() = default;
Status OnnxYoloV8Segmenter::load(const ModelConfig& c) {
    return c.path == "test-only" ? Status::ok() : Status::notFound("test model missing");
}
Result<SegmentationResult> OnnxYoloV8Segmenter::infer(const Frame& f) {
    if (f.sequence == 99) throw std::runtime_error("test inference exception");
    if (f.sequence == 96) return Status::unavailable("test inference failure");
    if (f.sequence == 97) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    SegmentationResult s;
    s.sequence = f.sequence; s.capture_time_ns = f.capture_time_ns;
    cv::extractChannel(f.image, s.combined_mask, 0);
    s.instances.push_back({{}, 0, f.sequence == 98 ? .01F : 1.F, {}});
    return s;
}
}
