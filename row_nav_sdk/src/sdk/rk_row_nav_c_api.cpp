#include "rk_row_nav.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "rk_nav/calibration/camera_calibration.hpp"
#include "rk_nav/geometry/dynamic_bev.hpp"
#include "rk_nav/inference/onnx_yolov8_segmenter.hpp"
#include "rk_nav/pipeline/perception_pipeline.hpp"
#ifdef RK_NAV_WITH_RKNN
#include "rk_nav/inference/rknn_yolov8_segmenter.hpp"
#endif

struct rk_row_nav_context {
    std::unique_ptr<rk_nav::Segmenter> segmenter;
    std::unique_ptr<rk_nav::TrackedPerceptionPipeline> pipeline;
    std::unique_ptr<rk_nav::DynamicBevProjector> dynamic_bev;
    rk_nav::CameraCalibration calibration;
    cv::Mat map_x, map_y;
    double max_age_ms = 0;
};

namespace {
thread_local char last_error[1024] = {};
int32_t fail(int32_t code, const char* message) noexcept {
    std::snprintf(last_error, sizeof(last_error), "%s", message);
    return code;
}
template<class T> bool compatible(const T* value) noexcept {
    return value && value->struct_size >= sizeof(T) && value->abi_version == RK_ROW_NAV_ABI_VERSION;
}
bool pathPresent(const char* path) { return path && *path; }
bool probability(float value) { return std::isfinite(value) && value >= 0 && value <= 1; }
void invalidate(rk_row_nav_result_t& result) {
    result.lateral_error_m = result.heading_error_rad = result.lookahead_m =
        std::numeric_limits<double>::quiet_NaN();
    result.confidence = 0;
}
}

extern "C" {
void rk_row_nav_config_init(rk_row_nav_config_t* c) {
    if (!c) return;
    *c = {};
    c->struct_size = sizeof(*c); c->abi_version = RK_ROW_NAV_ABI_VERSION;
    c->backend = RK_ROW_NAV_ONNX; c->model_width = c->model_height = 640;
    c->detection_confidence = .25F; c->minimum_navigation_confidence = .35F;
    c->max_frame_age_ms = 250;
}
void rk_row_nav_frame_init(rk_row_nav_frame_t* f) {
    if (!f) return;
    *f = {}; f->struct_size = sizeof(*f); f->abi_version = RK_ROW_NAV_ABI_VERSION;
    f->pixel_format = RK_ROW_NAV_BGR888;
}
void rk_row_nav_result_init(rk_row_nav_result_t* r) {
    if (!r) return;
    *r = {}; r->struct_size = sizeof(*r); r->abi_version = RK_ROW_NAV_ABI_VERSION;
    invalidate(*r);
}
void rk_row_nav_attitude_init(rk_row_nav_attitude_t* a) {
    if (!a) return;
    *a = {}; a->struct_size = sizeof(*a); a->abi_version = RK_ROW_NAV_ABI_VERSION;
}
const char* rk_row_nav_version(void) { return "0.2.0"; }
uint64_t rk_row_nav_now_ns(void) {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
const char* rk_row_nav_get_last_error(void) { return last_error; }

int32_t rk_row_nav_create(const rk_row_nav_config_t* c, rk_row_nav_handle_t* handle) {
    last_error[0] = '\0';
    if (!handle) return fail(RK_ROW_NAV_INVALID_ARGUMENT, "output handle is null");
    *handle = nullptr;
    try {
        if (!compatible(c) || !pathPresent(c->model_path) || !pathPresent(c->intrinsic_path) ||
            !pathPresent(c->bev_path) || c->model_width == 0 || c->model_width > 8192 ||
            c->model_height == 0 || c->model_height > 8192 ||
            !probability(c->detection_confidence) || !probability(c->minimum_navigation_confidence) ||
            !std::isfinite(c->max_frame_age_ms) || c->max_frame_age_ms <= 0)
            return fail(RK_ROW_NAV_INVALID_ARGUMENT, "invalid config, struct size or ABI version");
        auto ctx = std::make_unique<rk_row_nav_context>();
        if (c->backend == RK_ROW_NAV_ONNX) ctx->segmenter = std::make_unique<rk_nav::OnnxYoloV8Segmenter>();
        else if (c->backend == RK_ROW_NAV_RKNN) {
#ifdef RK_NAV_WITH_RKNN
            ctx->segmenter = std::make_unique<rk_nav::RknnYoloV8Segmenter>();
#else
            return fail(RK_ROW_NAV_UNSUPPORTED, "SDK built without RKNN support");
#endif
        } else return fail(RK_ROW_NAV_UNSUPPORTED, "unknown backend");
        // SDK requires explicitly accepted calibration, including intrinsic files.
        try {
            cv::FileStorage intrinsics(c->intrinsic_path, cv::FileStorage::READ);
            int accepted = 0;
            if (intrinsics.isOpened()) intrinsics["calibration_valid"] >> accepted;
            if (accepted != 1) return fail(RK_ROW_NAV_CALIBRATION_ERROR, "intrinsics require calibration_valid: 1");
            auto calibration = rk_nav::loadCameraCalibration(c->intrinsic_path);
            auto bev = rk_nav::loadBevConfig(c->bev_path);
            auto rows = rk_nav::loadRowExtractorConfig(c->bev_path);
            auto tracker = rk_nav::loadTemporalTrackerConfig(c->bev_path);
            auto dynamic = rk_nav::loadDynamicBevConfig(c->bev_path);
            if (!calibration.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, calibration.status().message().c_str());
            if (!bev.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, bev.status().message().c_str());
            if (!rows.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, rows.status().message().c_str());
            if (!tracker.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, tracker.status().message().c_str());
            if (!dynamic.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, dynamic.status().message().c_str());
            if (tracker.value().enabled && rows.value().method!=rk_nav::RowExtractionMethod::kAssociatedRows)
                return fail(RK_ROW_NAV_CALIBRATION_ERROR,"temporal tracking requires associated row extraction");
            ctx->calibration = calibration.value();
            if (ctx->calibration.image_size.width > 32768 || ctx->calibration.image_size.height > 32768 ||
                !std::isfinite(bev.value().vehicle_origin_u) || !std::isfinite(bev.value().vehicle_origin_v))
                return fail(RK_ROW_NAV_CALIBRATION_ERROR, "invalid calibration size or vehicle origin");
            auto valid = ctx->calibration.validateFor(ctx->calibration.image_size);
            if (!valid.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, valid.message().c_str());
            if (dynamic.value().enabled) {
                ctx->dynamic_bev = std::make_unique<rk_nav::DynamicBevProjector>(
                    bev.value(), ctx->calibration.camera_matrix, dynamic.value());
                const auto accepted = ctx->dynamic_bev->validate();
                if (!accepted.isOk()) return fail(RK_ROW_NAV_CALIBRATION_ERROR, accepted.message().c_str());
            }
            ctx->pipeline = std::make_unique<rk_nav::TrackedPerceptionPipeline>(
                rk_nav::PerceptionPipeline(rk_nav::BevProjector(bev.value()), rk_nav::RowExtractor(rows.value()),
                    rk_nav::CenterlineFitter{},rk_nav::PerceptionPipelineConfig{c->minimum_navigation_confidence}),tracker.value());
            cv::initUndistortRectifyMap(cv::Mat(ctx->calibration.camera_matrix),
                cv::Mat(ctx->calibration.distortion_coefficients, true), cv::Mat(),
                cv::Mat(ctx->calibration.camera_matrix), ctx->calibration.image_size,
                CV_32FC1, ctx->map_x, ctx->map_y);
        } catch (const std::exception& e) { return fail(RK_ROW_NAV_CALIBRATION_ERROR, e.what()); }
        rk_nav::ModelConfig model;
        model.path = c->model_path; model.input_width = static_cast<int>(c->model_width);
        model.input_height = static_cast<int>(c->model_height);
        model.confidence_threshold = c->detection_confidence; model.max_detections = 16;
        model.materialize_instance_masks = false;
        try {
            auto loaded = ctx->segmenter->load(model);
            if (!loaded.isOk()) return fail(RK_ROW_NAV_MODEL_ERROR, loaded.message().c_str());
        } catch (const std::exception& e) { return fail(RK_ROW_NAV_MODEL_ERROR, e.what()); }
        ctx->max_age_ms = c->max_frame_age_ms;
        *handle = ctx.release();
        return RK_ROW_NAV_OK;
    } catch (const std::exception& e) { return fail(RK_ROW_NAV_INTERNAL_ERROR, e.what()); }
    catch (...) { return fail(RK_ROW_NAV_INTERNAL_ERROR, "unknown create exception"); }
}

static int32_t processImpl(rk_row_nav_handle_t h, const rk_row_nav_frame_t* f,
                          const rk_row_nav_attitude_t* a, rk_row_nav_result_t* r) {
    last_error[0] = '\0';
    if (!compatible(r)) return fail(RK_ROW_NAV_INVALID_ARGUMENT, "invalid result struct size or ABI version");
    const uint32_t result_size = r->struct_size;
    rk_row_nav_result_init(r); r->struct_size = result_size;
    const auto start = rk_row_nav_now_ns();
    try {
        if (!h || !compatible(f) || !f->data || !f->width || !f->height ||
            f->width > 32768 || f->height > 32768 ||
            f->stride_bytes < uint64_t(f->width) * 3 ||
            f->data_size_bytes < uint64_t(f->stride_bytes) * f->height ||
            !f->capture_time_ns || f->capture_time_ns > start)
            return fail(RK_ROW_NAV_INVALID_ARGUMENT, "invalid frame buffer, stride, size, handle or monotonic timestamp");
        if (f->pixel_format != RK_ROW_NAV_BGR888 && f->pixel_format != RK_ROW_NAV_RGB888)
            return fail(RK_ROW_NAV_UNSUPPORTED, "only BGR888 and RGB888 supported");
        r->capture_time_ns = f->capture_time_ns; r->frame_id = f->frame_id;
        std::optional<rk_nav::BevProjector> frame_projector;
        if (h->dynamic_bev) {
            if (!compatible(a) || a->valid != 1 || a->reserved != 0 || !a->sample_time_ns ||
                a->sample_time_ns > f->capture_time_ns ||
                double(f->capture_time_ns-a->sample_time_ns)/1e6 > h->dynamic_bev->config().max_attitude_age_ms)
                return fail(RK_ROW_NAV_INVALID_ARGUMENT, "dynamic BEV requires causal fresh valid attitude");
            auto projector = h->dynamic_bev->at(a->roll_rad, a->pitch_rad);
            if (!projector.isOk()) return fail(RK_ROW_NAV_INVALID_ARGUMENT, projector.status().message().c_str());
            frame_projector = projector.value();
        } else if (a) {
            return fail(RK_ROW_NAV_UNSUPPORTED, "attitude supplied but dynamic BEV not enabled in calibration");
        }
        auto valid = h->calibration.validateFor(cv::Size(static_cast<int>(f->width), static_cast<int>(f->height)));
        if (!valid.isOk()) return fail(RK_ROW_NAV_INVALID_ARGUMENT, valid.message().c_str());
        auto finish = [&]() {
            const auto end = rk_row_nav_now_ns();
            r->processing_ms = double(end - start) / 1e6;
            r->frame_age_ms = double(end - f->capture_time_ns) / 1e6;
            if (r->frame_age_ms > h->max_age_ms) r->observation_status = RK_ROW_NAV_OBSERVATION_STALE;
            if (r->observation_status != RK_ROW_NAV_OBSERVATION_VALID) invalidate(*r);
            return int32_t(RK_ROW_NAV_OK);
        };
        if (double(start - f->capture_time_ns) / 1e6 > h->max_age_ms) return finish();
        cv::Mat input(static_cast<int>(f->height), static_cast<int>(f->width), CV_8UC3,
                      const_cast<uint8_t*>(f->data), f->stride_bytes);
        cv::Mat bgr;
        if (f->pixel_format == RK_ROW_NAV_RGB888) cv::cvtColor(input, bgr, cv::COLOR_RGB2BGR);
        else bgr = input;
        cv::Mat corrected;
        cv::remap(bgr, corrected, h->map_x, h->map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        auto segmented = h->segmenter->infer({corrected, static_cast<int64_t>(f->capture_time_ns), f->frame_id});
        if (!segmented.isOk()) return fail(RK_ROW_NAV_INFERENCE_ERROR, segmented.status().message().c_str());
        const long double deadline=static_cast<long double>(f->capture_time_ns)+static_cast<long double>(h->max_age_ms)*1e6L;
        const auto deadline_ns=deadline>=static_cast<long double>(std::numeric_limits<std::uint64_t>::max()) ?
            std::numeric_limits<std::uint64_t>::max() : static_cast<std::uint64_t>(deadline);
        auto output = h->pipeline->process(segmented.value(),0.0,deadline_ns,
            frame_projector ? &*frame_projector : nullptr);
        if (!output.isOk()) return fail(output.status().code()==rk_nav::StatusCode::kInvalidArgument ?
            RK_ROW_NAV_INVALID_ARGUMENT : RK_ROW_NAV_INTERNAL_ERROR,output.status().message().c_str());
        const auto& nav = output.value().navigation;
        if (nav.status == rk_nav::TrackingStatus::kValid &&
            (!std::isfinite(nav.lateral_error_m) || !std::isfinite(nav.heading_error_rad) ||
             !std::isfinite(nav.lookahead_m) || !std::isfinite(nav.confidence)))
            return fail(RK_ROW_NAV_INTERNAL_ERROR, "non-finite navigation output");
        if (nav.status == rk_nav::TrackingStatus::kValid) r->observation_status = RK_ROW_NAV_OBSERVATION_VALID;
        else if (nav.status == rk_nav::TrackingStatus::kLowConfidence) r->observation_status = RK_ROW_NAV_OBSERVATION_LOW_CONFIDENCE;
        else r->observation_status = RK_ROW_NAV_OBSERVATION_NO_ROW;
        r->lateral_error_m = nav.lateral_error_m; r->heading_error_rad = nav.heading_error_rad;
        r->lookahead_m = nav.lookahead_m; r->confidence = nav.confidence;
        return finish();
    } catch (const std::exception& e) { return fail(RK_ROW_NAV_INTERNAL_ERROR, e.what()); }
    catch (...) { return fail(RK_ROW_NAV_INTERNAL_ERROR, "unknown process exception"); }
}
int32_t rk_row_nav_dynamic_bev_enabled(rk_row_nav_handle_t h) {
    if (!h) return fail(RK_ROW_NAV_INVALID_ARGUMENT, "null handle");
    return h->dynamic_bev ? 1 : 0;
}
int32_t rk_row_nav_process(rk_row_nav_handle_t h, const rk_row_nav_frame_t* f, rk_row_nav_result_t* r) {
    return processImpl(h,f,nullptr,r);
}
int32_t rk_row_nav_process_with_attitude(rk_row_nav_handle_t h, const rk_row_nav_frame_t* f,
    const rk_row_nav_attitude_t* a, rk_row_nav_result_t* r) {
    // NULL must not accidentally select the static legacy API.
    if (!a) {
        if (compatible(r)) {const auto size=r->struct_size; rk_row_nav_result_init(r); r->struct_size=size;}
        return fail(RK_ROW_NAV_INVALID_ARGUMENT,"attitude is null");
    }
    return processImpl(h,f,a,r);
}
void rk_row_nav_destroy(rk_row_nav_handle_t h) { try { delete h; } catch (...) {} }
}
