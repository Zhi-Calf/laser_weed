#ifndef CROP_ROW_PERCEPTION__CORE_HPP_
#define CROP_ROW_PERCEPTION__CORE_HPP_

#include <rk_row_nav.h>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace crop_row_perception
{
static_assert(RK_ROW_NAV_ABI_VERSION == 1, "Review adapter before upgrading SDK ABI");
static_assert(RK_ROW_NAV_HAS_DYNAMIC_BEV == 1, "RowNav >= 0.2 required");
// ROS capture stamps may only be mapped when producer and consumer share the
// same ROS clock. A clock discontinuity latches a fault until node restart.
class FrameClockMapper
{
public:
  std::optional<uint64_t> map(
    int64_t capture_ros_ns, int64_t now_ros_ns, uint64_t now_mono_ns,
    double max_age_ms)
  {
    if (faulted_) {return std::nullopt;}
    if (!initialized_) {
      ros_origin_ = now_ros_ns;
      mono_origin_ = now_mono_ns;
      initialized_ = true;
    }
    const long double drift =
      (static_cast<long double>(now_ros_ns) - ros_origin_) -
      (static_cast<long double>(now_mono_ns) - mono_origin_);
    if (std::fabs(drift) > 100000000.0L) {
      faulted_ = true;
      return std::nullopt;
    }
    const long double age = static_cast<long double>(now_ros_ns) - capture_ros_ns;
    if (capture_ros_ns <= 0 || age < 0 || age > max_age_ms * 1e6L ||
      age >= now_mono_ns)
    {
      return std::nullopt;
    }
    return now_mono_ns - static_cast<uint64_t>(age);
  }
private:
  bool initialized_{false}, faulted_{false};
  int64_t ros_origin_{0};
  uint64_t mono_origin_{0};
};

// One in-flight frame plus one replaceable pending frame. Own image storage in T.
template<class T> class LatestFrameQueue
{
public:
  bool put(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return false;}
    if (pending_) {++dropped_;}
    pending_ = std::move(value);
    cv_.notify_one();
    return true;
  }
  std::optional<T> take() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] {return closed_ || pending_.has_value();});
    if (closed_) {return std::nullopt;}
    auto result = std::move(pending_);
    pending_.reset();
    return result;
  }
  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    pending_.reset();
    cv_.notify_all();
  }
  uint64_t dropped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }
private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> pending_;
  bool closed_{false};
  uint64_t dropped_{0};
};

class RowNavSdk
{
public:
  explicit RowNavSdk(const rk_row_nav_config_t & config) {
    if (rk_row_nav_create(&config, &handle_) != RK_ROW_NAV_OK) {
      throw std::runtime_error(std::string("RowNav create failed: ") +
        rk_row_nav_get_last_error());
    }
  }
  ~RowNavSdk() {rk_row_nav_destroy(handle_);}
  RowNavSdk(const RowNavSdk &) = delete;
  RowNavSdk & operator=(const RowNavSdk &) = delete;
  int32_t process(const rk_row_nav_frame_t & frame, rk_row_nav_result_t & result) {
    rk_row_nav_result_init(&result);
    return rk_row_nav_process(handle_, &frame, &result);
  }
  bool dynamicBevEnabled() const {return rk_row_nav_dynamic_bev_enabled(handle_) == 1;}
  int32_t process(const rk_row_nav_frame_t & frame, const rk_row_nav_attitude_t & attitude,
    rk_row_nav_result_t & result)
  {
    rk_row_nav_result_init(&result);
    return rk_row_nav_process_with_attitude(handle_, &frame, &attitude, &result);
  }
private:
  rk_row_nav_handle_t handle_{nullptr};
};

inline bool usable(const rk_row_nav_result_t & r) {
  return r.observation_status == RK_ROW_NAV_OBSERVATION_VALID &&
    std::isfinite(r.lateral_error_m) && std::isfinite(r.heading_error_rad) &&
    std::isfinite(r.lookahead_m) && r.lookahead_m > 0 &&
    std::isfinite(r.confidence) && r.confidence >= 0 && r.confidence <= 1 &&
    std::isfinite(r.processing_ms) && r.processing_ms >= 0 &&
    std::isfinite(r.frame_age_ms) && r.frame_age_ms >= 0;
}
}  // namespace crop_row_perception
#endif
