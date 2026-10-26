#include "rk_nav/inference/onnx_yolov8_segmenter.hpp"

#include <chrono>
#include <filesystem>
#include <vector>

#include <opencv2/dnn.hpp>

#include "rk_nav/inference/yolov8_seg_postprocess.hpp"

namespace rk_nav {

class OnnxYoloV8Segmenter::Impl {
public:
    ModelConfig config;
    cv::dnn::Net network;
    bool loaded{false};
};

OnnxYoloV8Segmenter::OnnxYoloV8Segmenter() : impl_(std::make_unique<Impl>()) {}
OnnxYoloV8Segmenter::~OnnxYoloV8Segmenter() = default;

Status OnnxYoloV8Segmenter::load(const ModelConfig& config) {
    if (config.path.empty()) return Status::invalidArgument("model path is empty");
    if (!std::filesystem::exists(config.path)) {
        return Status::notFound("model not found: " + config.path.string());
    }
    if (config.input_width <= 0 || config.input_height <= 0) {
        return Status::invalidArgument("model input size must be positive");
    }
    try {
        impl_->network = cv::dnn::readNetFromONNX(config.path.string());
        if (impl_->network.empty()) {
            return Status::unavailable("OpenCV returned an empty ONNX network");
        }
        impl_->network.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        impl_->network.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        impl_->config = config;
        impl_->loaded = true;
        return Status::ok();
    } catch (const cv::Exception& error) {
        return Status::unavailable("cannot load ONNX model: " + std::string(error.what()));
    }
}

Result<SegmentationResult> OnnxYoloV8Segmenter::infer(const Frame& frame) {
    if (!impl_->loaded) return Status::unavailable("segmenter is not loaded");
    if (frame.image.empty()) return Status::invalidArgument("input frame is empty");

    const auto started = std::chrono::steady_clock::now();
    try {
        const auto transformed = letterboxImage(
            frame.image, impl_->config.input_width, impl_->config.input_height);
        cv::Mat blob = cv::dnn::blobFromImage(
            transformed.image, 1.0 / 255.0,
            cv::Size(impl_->config.input_width, impl_->config.input_height),
            cv::Scalar(), true, false, CV_32F);
        impl_->network.setInput(blob);
        const auto names = impl_->network.getUnconnectedOutLayersNames();
        std::vector<cv::Mat> outputs;
        impl_->network.forward(outputs, names);
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        return decodeRockchipYoloV8Seg(
            outputs, frame, impl_->config, transformed, elapsed_ms);
    } catch (const cv::Exception& error) {
        return Status::internal("ONNX inference failed: " + std::string(error.what()));
    }
}

}  // namespace rk_nav
