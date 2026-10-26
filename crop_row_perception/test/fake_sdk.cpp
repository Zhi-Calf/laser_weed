// Test-only backend. Pixel[0] selects failure modes; no model is loaded.
#include <rk_row_nav.h>
#include <chrono>
#include <atomic>
#include <limits>
#include <thread>
#include <cstring>
#include <cmath>
struct rk_row_nav_context {bool dynamic{false};};
static std::atomic<bool> busy{false};
extern "C" {
bool fake_sdk_busy() {return busy.load();}
void rk_row_nav_config_init(rk_row_nav_config_t * c) {
  *c = {}; c->struct_size = sizeof(*c); c->abi_version = RK_ROW_NAV_ABI_VERSION;
}
void rk_row_nav_frame_init(rk_row_nav_frame_t * f) {
  *f = {}; f->struct_size = sizeof(*f); f->abi_version = RK_ROW_NAV_ABI_VERSION;
}
void rk_row_nav_result_init(rk_row_nav_result_t * r) {
  *r = {}; r->struct_size = sizeof(*r); r->abi_version = RK_ROW_NAV_ABI_VERSION;
  r->lateral_error_m = r->heading_error_rad = r->lookahead_m =
    std::numeric_limits<double>::quiet_NaN();
}
void rk_row_nav_attitude_init(rk_row_nav_attitude_t * a) {
  *a = {}; a->struct_size = sizeof(*a); a->abi_version = RK_ROW_NAV_ABI_VERSION;
}
const char * rk_row_nav_version() {return "test-only";}
const char * rk_row_nav_get_last_error() {return "injected SDK error";}
uint64_t rk_row_nav_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
int32_t rk_row_nav_create(const rk_row_nav_config_t * c, rk_row_nav_handle_t * h) {
  *h = nullptr;
  if (!c->model_path || c->model_path[0] == '!') {return RK_ROW_NAV_MODEL_ERROR;}
  *h = new rk_row_nav_context;
  (*h)->dynamic = c->bev_path && std::strcmp(c->bev_path, "dynamic-fake") == 0;
  return RK_ROW_NAV_OK;
}
void rk_row_nav_destroy(rk_row_nav_handle_t h) {delete h;}
static int32_t fakeProcess(const rk_row_nav_frame_t * f,
  rk_row_nav_result_t * r)
{
  rk_row_nav_result_init(r);
  r->capture_time_ns = f->capture_time_ns; r->frame_id = f->frame_id;
  if (f->data[0] == 1) {return RK_ROW_NAV_INFERENCE_ERROR;}
  if (f->data[0] == 2) {r->observation_status = RK_ROW_NAV_OBSERVATION_NO_ROW; return 0;}
  if (f->data[0] == 7) {r->observation_status = RK_ROW_NAV_OBSERVATION_LOW_CONFIDENCE; return 0;}
  if (f->data[0] == 6) {r->observation_status = 999; return 0;}
  if (f->data[0] == 4) {
    busy = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    busy = false;
  }
  r->observation_status = RK_ROW_NAV_OBSERVATION_VALID;
  r->lateral_error_m = 0.12; r->heading_error_rad = -0.03;
  r->lookahead_m = 2; r->confidence = 0.9;
  if (f->data[0] == 3) {r->lateral_error_m = std::numeric_limits<double>::quiet_NaN();}
  if (f->data[0] == 5) {++r->frame_id;}
  return 0;
}
int32_t rk_row_nav_dynamic_bev_enabled(rk_row_nav_handle_t h) {
  return h ? (h->dynamic ? 1 : 0) : RK_ROW_NAV_INVALID_ARGUMENT;
}
int32_t rk_row_nav_process(rk_row_nav_handle_t h,const rk_row_nav_frame_t * f,rk_row_nav_result_t * r) {
  if (h && h->dynamic) {rk_row_nav_result_init(r); return RK_ROW_NAV_INVALID_ARGUMENT;}
  return fakeProcess(f,r);
}
int32_t rk_row_nav_process_with_attitude(rk_row_nav_handle_t h,const rk_row_nav_frame_t * f,
  const rk_row_nav_attitude_t * a,rk_row_nav_result_t * r)
{
  rk_row_nav_result_init(r);
  if (!h || !h->dynamic || !a || !a->valid || !a->sample_time_ns ||
    a->sample_time_ns > f->capture_time_ns || f->capture_time_ns-a->sample_time_ns > 50000000 ||
    !std::isfinite(a->roll_rad) || !std::isfinite(a->pitch_rad))
  {return RK_ROW_NAV_INVALID_ARGUMENT;}
  const auto code = fakeProcess(f,r);
  if (code == 0 && r->observation_status == RK_ROW_NAV_OBSERVATION_VALID) {
    // ROS plumbing test only, NOT a physical compensation model.
    r->lateral_error_m += a->roll_rad; r->heading_error_rad += a->pitch_rad;
  }
  return code;
}
}
