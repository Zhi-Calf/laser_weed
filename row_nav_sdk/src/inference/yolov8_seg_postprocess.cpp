#include "rk_nav/inference/yolov8_seg_postprocess.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

// Output layout and DFL/mask reconstruction follow the Apache-2.0 Rockchip
// rknn_model_zoo YOLOv8-seg example at commit bad6c7334531becaf90a561988519b7bec34d0ab.

namespace rk_nav {
namespace {

struct Candidate {
    cv::Rect2d box;
    float score{0.0F};
    std::array<float, 32> coefficients{};
};

float tensorValue(const cv::Mat& tensor, int channel, int y, int x) {
    const int height = tensor.size[2];
    const int width = tensor.size[3];
    return tensor.ptr<float>()[((channel * height) + y) * width + x];
}

float dflDistance(const cv::Mat& boxes, int side, int y, int x) {
    constexpr int bins = 16;
    float maximum = -std::numeric_limits<float>::infinity();
    for (int bin = 0; bin < bins; ++bin) {
        maximum = std::max(maximum, tensorValue(boxes, side * bins + bin, y, x));
    }
    float sum = 0.0F;
    float weighted = 0.0F;
    for (int bin = 0; bin < bins; ++bin) {
        const float value = std::exp(tensorValue(boxes, side * bins + bin, y, x) - maximum);
        sum += value;
        weighted += value * static_cast<float>(bin);
    }
    return weighted / sum;
}

bool isTensor(const cv::Mat& tensor, int channels, int spatial) {
    return tensor.dims == 4 && tensor.size[0] == 1 && tensor.size[1] == channels &&
           tensor.size[2] == spatial && tensor.size[3] == spatial && tensor.type() == CV_32F;
}

cv::Rect toOriginalBox(const cv::Rect2d& box, const LetterboxResult& transform,
                       const cv::Size& original) {
    const double x1 = std::clamp((box.x - transform.left) / transform.scale, 0.0,
                                 static_cast<double>(original.width - 1));
    const double y1 = std::clamp((box.y - transform.top) / transform.scale, 0.0,
                                 static_cast<double>(original.height - 1));
    const double x2 = std::clamp((box.x + box.width - transform.left) / transform.scale, 0.0,
                                 static_cast<double>(original.width));
    const double y2 = std::clamp((box.y + box.height - transform.top) / transform.scale, 0.0,
                                 static_cast<double>(original.height));
    return cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                    std::max(0, static_cast<int>(x2 - x1)),
                    std::max(0, static_cast<int>(y2 - y1)));
}

}  // namespace

LetterboxResult letterboxImage(const cv::Mat& source, int width, int height) {
    LetterboxResult result;
    result.scale = std::min(static_cast<float>(width) / static_cast<float>(source.cols),
                            static_cast<float>(height) / static_cast<float>(source.rows));
    result.resized_width = static_cast<int>(std::round(static_cast<float>(source.cols) * result.scale));
    result.resized_height = static_cast<int>(std::round(static_cast<float>(source.rows) * result.scale));
    result.left = (width - result.resized_width) / 2;
    result.top = (height - result.resized_height) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(result.resized_width, result.resized_height));
    cv::copyMakeBorder(resized, result.image, result.top, height - result.resized_height - result.top,
                       result.left, width - result.resized_width - result.left,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return result;
}

Result<SegmentationResult> decodeRockchipYoloV8Seg(
    const std::vector<cv::Mat>& outputs, const Frame& frame, const ModelConfig& config,
    const LetterboxResult& transform, double inference_ms) {
    if (outputs.size() != 10 && outputs.size() != 13) {
        return Status::invalidArgument("Rockchip decoder expects 10 or 13 outputs, got " +
                                       std::to_string(outputs.size()));
    }
    if (config.max_detections <= 0) {
        return Status::invalidArgument("max_detections must be greater than zero");
    }

    const cv::Mat* proto = nullptr;
    std::vector<Candidate> candidates;
    if (outputs.size() == 13 && isTensor(outputs[12], 32, 160)) {
        proto = &outputs[12];
    } else {
        for (const auto& tensor : outputs) {
            if (isTensor(tensor, 32, 160)) proto = &tensor;
        }
    }
    if (proto == nullptr) {
        return Status::invalidArgument("model output does not contain [1,32,160,160] proto");
    }

    int branch = 0;
    for (const int spatial : {80, 40, 20}) {
        const cv::Mat* boxes = nullptr;
        const cv::Mat* scores = nullptr;
        const cv::Mat* coefficients = nullptr;
        if (outputs.size() == 13) {
            const std::size_t offset = static_cast<std::size_t>(branch * 4);
            boxes = &outputs[offset];
            scores = &outputs[offset + 1];
            coefficients = &outputs[offset + 3];
        } else {
            for (const auto& tensor : outputs) {
                if (isTensor(tensor, 64, spatial)) boxes = &tensor;
                else if (isTensor(tensor, 1, spatial)) scores = &tensor;
                else if (isTensor(tensor, 32, spatial)) coefficients = &tensor;
            }
        }
        if (boxes == nullptr || scores == nullptr || coefficients == nullptr ||
            !isTensor(*boxes, 64, spatial) || !isTensor(*scores, 1, spatial) ||
            !isTensor(*coefficients, 32, spatial)) {
            return Status::invalidArgument("incomplete output branch at spatial size " +
                                           std::to_string(spatial));
        }
        const float stride_x = static_cast<float>(config.input_width) / static_cast<float>(spatial);
        const float stride_y = static_cast<float>(config.input_height) / static_cast<float>(spatial);
        for (int y = 0; y < spatial; ++y) {
            for (int x = 0; x < spatial; ++x) {
                const float score = tensorValue(*scores, 0, y, x);
                if (score < config.confidence_threshold) continue;
                const float center_x = (static_cast<float>(x) + 0.5F) * stride_x;
                const float center_y = (static_cast<float>(y) + 0.5F) * stride_y;
                const float left = dflDistance(*boxes, 0, y, x) * stride_x;
                const float top = dflDistance(*boxes, 1, y, x) * stride_y;
                const float right = dflDistance(*boxes, 2, y, x) * stride_x;
                const float bottom = dflDistance(*boxes, 3, y, x) * stride_y;
                Candidate candidate;
                candidate.box = cv::Rect2d(center_x - left, center_y - top,
                                           left + right, top + bottom);
                candidate.score = score;
                for (int channel = 0; channel < 32; ++channel) {
                    candidate.coefficients[static_cast<std::size_t>(channel)] =
                        tensorValue(*coefficients, channel, y, x);
                }
                candidates.push_back(candidate);
            }
        }
        ++branch;
    }

    std::vector<cv::Rect> nms_boxes;
    std::vector<float> nms_scores;
    for (const auto& candidate : candidates) {
        nms_boxes.emplace_back(static_cast<int>(candidate.box.x), static_cast<int>(candidate.box.y),
                               static_cast<int>(candidate.box.width), static_cast<int>(candidate.box.height));
        nms_scores.push_back(candidate.score);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(nms_boxes, nms_scores, config.confidence_threshold,
                      config.nms_threshold, keep, 1.0F, config.max_detections);

    SegmentationResult result;
    result.sequence = frame.sequence;
    result.capture_time_ns = frame.capture_time_ns;
    result.combined_mask = cv::Mat::zeros(frame.image.size(), CV_8UC1);
    const int proto_height = proto->size[2];
    const int proto_width = proto->size[3];
    cv::Mat combined_proto = cv::Mat::zeros(proto_height, proto_width, CV_8UC1);
    for (const int index : keep) {
        const auto& candidate = candidates[static_cast<std::size_t>(index)];
        cv::Mat logits(proto_height, proto_width, CV_32F, cv::Scalar(0));
        for (int channel = 0; channel < 32; ++channel) {
            const float coefficient = candidate.coefficients[static_cast<std::size_t>(channel)];
            const float* plane = proto->ptr<float>() + channel * proto_height * proto_width;
            float* output = logits.ptr<float>();
            for (int pixel = 0; pixel < proto_height * proto_width; ++pixel) {
                output[pixel] += coefficient * plane[pixel];
            }
        }
        SegmentationInstance instance;
        instance.box = toOriginalBox(candidate.box, transform, frame.image.size());
        instance.class_id = 0;
        instance.confidence = candidate.score;
        if (config.materialize_instance_masks) {
            cv::Mat model_mask;
            cv::resize(logits, model_mask, cv::Size(config.input_width, config.input_height),
                       0, 0, cv::INTER_LINEAR);
            cv::exp(-model_mask, model_mask);
            model_mask = 1.0 / (1.0 + model_mask);
            cv::Mat binary = model_mask > config.mask_threshold;
            const cv::Rect model_bounds(0, 0, config.input_width, config.input_height);
            const cv::Rect clipped = nms_boxes[static_cast<std::size_t>(index)] & model_bounds;
            cv::Mat cropped = cv::Mat::zeros(binary.size(), CV_8UC1);
            if (clipped.area() > 0) binary(clipped).copyTo(cropped(clipped));
            const cv::Rect content(transform.left, transform.top,
                                   transform.resized_width, transform.resized_height);
            cv::resize(cropped(content), instance.mask, frame.image.size(),
                       0, 0, cv::INTER_NEAREST);
            cv::bitwise_or(result.combined_mask, instance.mask, result.combined_mask);
        } else {
            cv::Mat probability;
            cv::exp(-logits, probability);
            probability = 1.0 / (1.0 + probability);
            cv::Mat binary = probability > config.mask_threshold;
            const double scale_x = static_cast<double>(proto_width) /
                                   static_cast<double>(config.input_width);
            const double scale_y = static_cast<double>(proto_height) /
                                   static_cast<double>(config.input_height);
            const auto& box = candidate.box;
            const int proto_left = static_cast<int>(std::floor(box.x * scale_x));
            const int proto_top = static_cast<int>(std::floor(box.y * scale_y));
            const int proto_right = static_cast<int>(std::ceil((box.x + box.width) * scale_x));
            const int proto_bottom = static_cast<int>(std::ceil((box.y + box.height) * scale_y));
            cv::Rect proto_box(proto_left, proto_top,
                               proto_right - proto_left, proto_bottom - proto_top);
            proto_box &= cv::Rect(0, 0, proto_width, proto_height);
            if (proto_box.area() > 0) {
                cv::bitwise_or(combined_proto(proto_box), binary(proto_box),
                               combined_proto(proto_box));
            }
        }
        result.instances.push_back(std::move(instance));
    }
    if (!config.materialize_instance_masks && !keep.empty()) {
        const double proto_scale_x = static_cast<double>(proto_width) /
                                     static_cast<double>(config.input_width);
        const double proto_scale_y = static_cast<double>(proto_height) /
                                     static_cast<double>(config.input_height);
        const int left = std::clamp(static_cast<int>(std::floor(
            static_cast<double>(transform.left) * proto_scale_x)), 0, proto_width - 1);
        const int top = std::clamp(static_cast<int>(std::floor(
            static_cast<double>(transform.top) * proto_scale_y)), 0, proto_height - 1);
        const int right = std::clamp(static_cast<int>(std::ceil(
            static_cast<double>(transform.left + transform.resized_width) * proto_scale_x)),
            left + 1, proto_width);
        const int bottom = std::clamp(static_cast<int>(std::ceil(
            static_cast<double>(transform.top + transform.resized_height) * proto_scale_y)),
            top + 1, proto_height);
        cv::resize(combined_proto(cv::Rect(left, top, right - left, bottom - top)),
                   result.combined_mask, frame.image.size(), 0, 0, cv::INTER_NEAREST);
    }
    result.inference_ms = inference_ms;
    return result;
}

}  // namespace rk_nav
