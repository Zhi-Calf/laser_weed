#ifndef RK_ROW_NAV_H
#define RK_ROW_NAV_H
#include <stdint.h>

#if defined(_WIN32)
# if defined(RK_ROW_NAV_BUILD)
#  define RK_ROW_NAV_API __declspec(dllexport)
# else
#  define RK_ROW_NAV_API __declspec(dllimport)
# endif
#else
# define RK_ROW_NAV_API __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define RK_ROW_NAV_ABI_VERSION 1u
#define RK_ROW_NAV_HAS_DYNAMIC_BEV 1
typedef struct rk_row_nav_context* rk_row_nav_handle_t;
/* Fixed-width fields, never C++ types or compiler-dependent enum storage. */
enum {
    RK_ROW_NAV_OK = 0,
    RK_ROW_NAV_INVALID_ARGUMENT = -1,
    RK_ROW_NAV_UNSUPPORTED = -2,
    RK_ROW_NAV_CALIBRATION_ERROR = -3,
    RK_ROW_NAV_MODEL_ERROR = -4,
    RK_ROW_NAV_INFERENCE_ERROR = -5,
    RK_ROW_NAV_INTERNAL_ERROR = -6
};
enum { RK_ROW_NAV_ONNX = 1, RK_ROW_NAV_RKNN = 2 };
enum { RK_ROW_NAV_BGR888 = 1, RK_ROW_NAV_RGB888 = 2 };
enum {
    RK_ROW_NAV_OBSERVATION_INVALID = 0,
    RK_ROW_NAV_OBSERVATION_VALID = 1,
    RK_ROW_NAV_OBSERVATION_LOW_CONFIDENCE = 2,
    RK_ROW_NAV_OBSERVATION_NO_ROW = 3,
    RK_ROW_NAV_OBSERVATION_STALE = 4
};

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    const char* model_path;
    const char* intrinsic_path;
    const char* bev_path;
    uint32_t backend;
    uint32_t model_width;
    uint32_t model_height;
    float detection_confidence;
    float minimum_navigation_confidence;
    double max_frame_age_ms;
} rk_row_nav_config_t;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    const uint8_t* data;
    uint64_t data_size_bytes;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    uint32_t pixel_format;
    uint64_t capture_time_ns;
    uint64_t frame_id;
} rk_row_nav_frame_t;

/* Additive API in 0.2; all existing v1 structures/symbols remain unchanged.
   Vehicle ROS axes; sample_time_ns uses the SAME monotonic clock as the image.
   valid must equal 1. No extrapolation or fallback on invalid/missing attitude. */
typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t sample_time_ns;
    double roll_rad;
    double pitch_rad;
    uint32_t valid;
    uint32_t reserved;
} rk_row_nav_attitude_t;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t capture_time_ns;
    uint64_t frame_id;
    uint32_t observation_status;
    uint32_t reserved;
    /* x forward, y left; line y = slope*x + lateral_error_m.
       Positive lateral = path to vehicle's left, heading = atan(slope). */
    double lateral_error_m;
    double heading_error_rad;
    double lookahead_m;
    double confidence;
    double processing_ms;
    double frame_age_ms;
} rk_row_nav_result_t;

/* Call init on owned current-version structures before use.
   API v1 accepts struct_size >= sizeof(v1), preserving unknown tail bytes. */
RK_ROW_NAV_API void rk_row_nav_config_init(rk_row_nav_config_t* config);
RK_ROW_NAV_API void rk_row_nav_frame_init(rk_row_nav_frame_t* frame);
RK_ROW_NAV_API void rk_row_nav_result_init(rk_row_nav_result_t* result);
RK_ROW_NAV_API void rk_row_nav_attitude_init(rk_row_nav_attitude_t* attitude);
RK_ROW_NAV_API const char* rk_row_nav_version(void);
/* Use this same clock for capture timestamps; on Linux CLOCK_MONOTONIC. */
RK_ROW_NAV_API uint64_t rk_row_nav_now_ns(void);
/* Failure sets *handle = NULL. Paths are borrowed for create only. */
RK_ROW_NAV_API int32_t rk_row_nav_create(const rk_row_nav_config_t* config,
                                      rk_row_nav_handle_t* handle);
/* Synchronous, serial calls per handle. No image pointer retained on return.
   All non-VALID observations have zero confidence and NaN navigation values.
   A successful call can still return NO_ROW, LOW_CONFIDENCE or STALE. */
RK_ROW_NAV_API int32_t rk_row_nav_process(rk_row_nav_handle_t handle,
    const rk_row_nav_frame_t* frame, rk_row_nav_result_t* result);
/* Returns 0/1, or INVALID_ARGUMENT for NULL. Immutable for handle lifetime. */
RK_ROW_NAV_API int32_t rk_row_nav_dynamic_bev_enabled(rk_row_nav_handle_t handle);
/* Requires accepted dynamic_bev configuration in BEV YAML. For static handles,
   use rk_row_nav_process; for dynamic handles old process rejects (no fallback).
   Pose must be causal and within max_attitude_age_ms of the image capture. */
RK_ROW_NAV_API int32_t rk_row_nav_process_with_attitude(rk_row_nav_handle_t handle,
    const rk_row_nav_frame_t* frame, const rk_row_nav_attitude_t* attitude,
    rk_row_nav_result_t* result);
/* Thread-local error from the last create/process on this thread; valid until
   the next create/process. Also works when create failed before a handle exists. */
RK_ROW_NAV_API const char* rk_row_nav_get_last_error(void);
/* NULL is allowed. Caller serializes destroy with all uses; never reuse handle. */
RK_ROW_NAV_API void rk_row_nav_destroy(rk_row_nav_handle_t handle);

#ifdef __cplusplus
}
#endif
#endif
