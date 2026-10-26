#pragma once

#include <filesystem>
#include <vector>
#include <opencv2/core/mat.hpp>
#include "rk_nav/core/frame.hpp"
#include "rk_nav/core/status.hpp"

namespace rk_nav {

struct ModelConfig {
    std::filesystem::path path;
    int input_width{640};
    int input_height{640};
    float confidence_threshold{0.25F};
    float nms_threshold{0.70F};
    float mask_threshold{0.5F};
    int max_detections{100};
    bool materialize_instance_masks{true};
};

struct SegmentationInstance {
    cv::Rect box;
    int class_id{0};
    float confidence{0.0F};
    cv::Mat mask;
};

struct InferenceTiming {
    double preprocess_ms{0.0};
    double input_setup_ms{0.0};
    double npu_run_ms{0.0};
    double output_copy_ms{0.0};
    double postprocess_ms{0.0};
};

struct SegmentationResult {
    std::vector<SegmentationInstance> instances;
    cv::Mat combined_mask;
    std::uint64_t sequence{0};
    std::int64_t capture_time_ns{0};
    double inference_ms{0.0};
    InferenceTiming timing;
};

class Segmenter {
public:
    virtual ~Segmenter() = default;
    virtual Status load(const ModelConfig& config) = 0;
    virtual Result<SegmentationResult> infer(const Frame& frame) = 0;
};

}  // namespace rk_nav
