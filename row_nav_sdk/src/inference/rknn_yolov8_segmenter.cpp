#include "rk_nav/inference/rknn_yolov8_segmenter.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rknn_api.h>

#include "rk_nav/inference/yolov8_seg_postprocess.hpp"

namespace rk_nav {
namespace {

Result<std::vector<unsigned char>> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream) return Status::notFound("cannot open RKNN model: " + path.string());
    const auto length = stream.tellg();
    if (length <= 0) return Status::invalidArgument("RKNN model is empty: " + path.string());
    std::vector<unsigned char> bytes(static_cast<std::size_t>(length));
    stream.seekg(0, std::ios::beg);
    if (!stream.read(reinterpret_cast<char*>(bytes.data()), length)) {
        return Status::internal("cannot read RKNN model: " + path.string());
    }
    return bytes;
}

Status rknnFailure(const char* operation, int code) {
    return Status::internal(std::string(operation) + " failed, RKNN code " + std::to_string(code));
}

Result<cv::Mat> outputAsNchw(const rknn_output& output, const rknn_tensor_attr& attr,
                             bool requested_float) {
    if (attr.n_dims != 4) {
        return Status::invalidArgument("RKNN output " + std::to_string(attr.index) +
                                       " is not four-dimensional");
    }
    if (output.buf == nullptr) return Status::internal("RKNN returned a null output buffer");

    const auto value_at = [&](std::size_t index) -> float {
        if (requested_float || attr.type == RKNN_TENSOR_FLOAT32) {
            return static_cast<const float*>(output.buf)[index];
        }
        if (attr.type == RKNN_TENSOR_INT8) {
            const auto value = static_cast<const std::int8_t*>(output.buf)[index];
            return static_cast<float>(static_cast<int>(value) - attr.zp) * attr.scale;
        }
        const auto value = static_cast<const std::uint8_t*>(output.buf)[index];
        return static_cast<float>(static_cast<int>(value) - attr.zp) * attr.scale;
    };

    if (attr.fmt == RKNN_TENSOR_NHWC) {
        const int sizes[4] = {static_cast<int>(attr.dims[0]), static_cast<int>(attr.dims[3]),
                              static_cast<int>(attr.dims[1]), static_cast<int>(attr.dims[2])};
        cv::Mat nchw(4, sizes, CV_32F);
        float* destination = nchw.ptr<float>();
        const int height = sizes[2];
        const int width = sizes[3];
        const int channels = sizes[1];
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                for (int channel = 0; channel < channels; ++channel) {
                    destination[(channel * height + y) * width + x] =
                        value_at(static_cast<std::size_t>((y * width + x) * channels + channel));
                }
            }
        }
        return nchw;
    }

    const int sizes[4] = {static_cast<int>(attr.dims[0]), static_cast<int>(attr.dims[1]),
                          static_cast<int>(attr.dims[2]), static_cast<int>(attr.dims[3])};
    cv::Mat nchw(4, sizes, CV_32F);
    if (requested_float || attr.type == RKNN_TENSOR_FLOAT32) {
        std::memcpy(nchw.ptr<float>(), output.buf, attr.n_elems * sizeof(float));
    } else {
        float* destination = nchw.ptr<float>();
        for (std::size_t index = 0; index < attr.n_elems; ++index) {
            destination[index] = value_at(index);
        }
    }
    return nchw;
}

}  // namespace

class RknnYoloV8Segmenter::Impl {
public:
    ~Impl() { reset(); }

    void reset() {
        if (context != 0) rknn_destroy(context);
        context = 0;
        input_attrs.clear();
        output_attrs.clear();
        loaded = false;
    }

    ModelConfig config;
    rknn_context context{0};
    std::vector<rknn_tensor_attr> input_attrs;
    std::vector<rknn_tensor_attr> output_attrs;
    bool loaded{false};
};

RknnYoloV8Segmenter::RknnYoloV8Segmenter() : impl_(std::make_unique<Impl>()) {}
RknnYoloV8Segmenter::~RknnYoloV8Segmenter() = default;

Status RknnYoloV8Segmenter::load(const ModelConfig& config) {
    if (config.path.empty()) return Status::invalidArgument("model path is empty");
    if (!std::filesystem::exists(config.path)) {
        return Status::notFound("model not found: " + config.path.string());
    }
    impl_->reset();
    auto model = readBinaryFile(config.path);
    if (!model.isOk()) return model.status();

    int code = rknn_init(&impl_->context, model.value().data(),
                         static_cast<uint32_t>(model.value().size()), 0, nullptr);
    if (code != RKNN_SUCC) {
        impl_->reset();
        return rknnFailure("rknn_init", code);
    }

    rknn_input_output_num count{};
    code = rknn_query(impl_->context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count));
    if (code != RKNN_SUCC) {
        impl_->reset();
        return rknnFailure("RKNN_QUERY_IN_OUT_NUM", code);
    }
    if (count.n_input != 1 || (count.n_output != 10 && count.n_output != 13)) {
        impl_->reset();
        return Status::invalidArgument("expected 1 RKNN input and 10 or 13 outputs, got " +
                                       std::to_string(count.n_input) + " and " +
                                       std::to_string(count.n_output));
    }

    impl_->input_attrs.resize(count.n_input);
    impl_->output_attrs.resize(count.n_output);
    for (uint32_t index = 0; index < count.n_input; ++index) {
        auto& attr = impl_->input_attrs[index];
        attr.index = index;
        code = rknn_query(impl_->context, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (code != RKNN_SUCC) {
            impl_->reset();
            return rknnFailure("RKNN_QUERY_INPUT_ATTR", code);
        }
    }
    for (uint32_t index = 0; index < count.n_output; ++index) {
        auto& attr = impl_->output_attrs[index];
        attr.index = index;
        code = rknn_query(impl_->context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        if (code != RKNN_SUCC) {
            impl_->reset();
            return rknnFailure("RKNN_QUERY_OUTPUT_ATTR", code);
        }
    }

    const auto& input = impl_->input_attrs.front();
    const int model_height = input.fmt == RKNN_TENSOR_NCHW
        ? static_cast<int>(input.dims[2]) : static_cast<int>(input.dims[1]);
    const int model_width = input.fmt == RKNN_TENSOR_NCHW
        ? static_cast<int>(input.dims[3]) : static_cast<int>(input.dims[2]);
    if (model_width != config.input_width || model_height != config.input_height) {
        impl_->reset();
        return Status::invalidArgument("configured input size does not match RKNN model: " +
                                       std::to_string(model_width) + "x" +
                                       std::to_string(model_height));
    }
    impl_->config = config;
    impl_->loaded = true;
    return Status::ok();
}

Result<SegmentationResult> RknnYoloV8Segmenter::infer(const Frame& frame) {
    auto prepared = preprocess(frame);
    if (!prepared.isOk()) return prepared.status();
    auto raw = execute(std::move(prepared.value()));
    if (!raw.isOk()) return raw.status();
    return postprocess(std::move(raw.value()));
}

Result<RknnPreparedInput> RknnYoloV8Segmenter::preprocess(const Frame& frame) const {
    if (!impl_->loaded) return Status::unavailable("segmenter is not loaded");
    if (frame.image.empty()) return Status::invalidArgument("input frame is empty");

    const auto started = std::chrono::steady_clock::now();
    RknnPreparedInput prepared;
    prepared.frame = frame;
    prepared.transform = letterboxImage(
        frame.image, impl_->config.input_width, impl_->config.input_height);
    cv::cvtColor(prepared.transform.image, prepared.rgb, cv::COLOR_BGR2RGB);
    if (!prepared.rgb.isContinuous()) prepared.rgb = prepared.rgb.clone();
    prepared.timing.preprocess_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return prepared;
}

Result<RknnRawOutput> RknnYoloV8Segmenter::execute(RknnPreparedInput prepared) {
    if (!impl_->loaded) return Status::unavailable("segmenter is not loaded");

    rknn_input input{};
    input.index = 0;
    input.buf = prepared.rgb.data;
    input.size = static_cast<uint32_t>(prepared.rgb.total() * prepared.rgb.elemSize());
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.pass_through = 0;
    auto stage_started = std::chrono::steady_clock::now();
    int code = rknn_inputs_set(impl_->context, 1, &input);
    if (code != RKNN_SUCC) return rknnFailure("rknn_inputs_set", code);
    prepared.timing.input_setup_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - stage_started).count();
    stage_started = std::chrono::steady_clock::now();
    code = rknn_run(impl_->context, nullptr);
    if (code != RKNN_SUCC) return rknnFailure("rknn_run", code);
    prepared.timing.npu_run_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - stage_started).count();

    stage_started = std::chrono::steady_clock::now();
    std::vector<rknn_output> raw_outputs(impl_->output_attrs.size());
    std::vector<bool> requested_float(raw_outputs.size(), true);
    for (std::size_t index = 0; index < raw_outputs.size(); ++index) {
        raw_outputs[index].index = static_cast<uint32_t>(index);
        const auto type = impl_->output_attrs[index].type;
        requested_float[index] = type != RKNN_TENSOR_INT8 && type != RKNN_TENSOR_UINT8;
        // Quantized output is dequantized while copying into the decoder's NCHW buffer.
        raw_outputs[index].want_float = requested_float[index] ? 1 : 0;
    }
    code = rknn_outputs_get(impl_->context, static_cast<uint32_t>(raw_outputs.size()),
                            raw_outputs.data(), nullptr);
    if (code != RKNN_SUCC) return rknnFailure("rknn_outputs_get", code);

    std::vector<cv::Mat> outputs;
    outputs.reserve(raw_outputs.size());
    Status conversion_status = Status::ok();
    for (std::size_t index = 0; index < raw_outputs.size(); ++index) {
        auto converted = outputAsNchw(raw_outputs[index], impl_->output_attrs[index],
                                      requested_float[index]);
        if (!converted.isOk()) {
            conversion_status = converted.status();
            break;
        }
        outputs.push_back(std::move(converted.value()));
    }
    rknn_outputs_release(impl_->context, static_cast<uint32_t>(raw_outputs.size()),
                         raw_outputs.data());
    if (!conversion_status.isOk()) return conversion_status;
    prepared.timing.output_copy_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - stage_started).count();

    RknnRawOutput raw;
    raw.frame = std::move(prepared.frame);
    raw.transform = std::move(prepared.transform);
    raw.outputs = std::move(outputs);
    raw.timing = prepared.timing;
    return raw;
}

Result<SegmentationResult> RknnYoloV8Segmenter::postprocess(RknnRawOutput raw) const {
    if (!impl_->loaded) return Status::unavailable("segmenter is not loaded");
    const auto started = std::chrono::steady_clock::now();
    auto decoded = decodeRockchipYoloV8Seg(
        raw.outputs, raw.frame, impl_->config, raw.transform, 0.0);
    if (!decoded.isOk()) return decoded.status();
    raw.timing.postprocess_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    decoded.value().timing = raw.timing;
    decoded.value().inference_ms = raw.timing.preprocess_ms + raw.timing.input_setup_ms +
        raw.timing.npu_run_ms + raw.timing.output_copy_ms + raw.timing.postprocess_ms;
    return decoded;
}

}  // namespace rk_nav
