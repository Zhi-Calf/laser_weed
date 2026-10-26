#ifndef CROP_ROW_PERCEPTION__ATTITUDE_GATE_HPP_
#define CROP_ROW_PERCEPTION__ATTITUDE_GATE_HPP_

#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>

namespace crop_row_perception
{
// Shared pose validity/time/envelope checks for static and dynamic BEV modes.
// This class itself performs no estimation or geometry correction.
// Samples are already in vehicle ROS axes.
struct AttitudeGateConfig {
  double reference_roll_rad{0}, reference_pitch_rad{0};
  double max_delta_rad{0.035}, max_sample_age_ms{50};
};
struct AttitudeSample {
  int64_t stamp_ns{0};
  double roll_rad{0}, pitch_rad{0};
  bool valid{false};
};
struct AttitudeDecision {
  bool usable{false};
  double age_ms{0}, roll_delta_rad{0}, pitch_delta_rad{0};
  int64_t sample_stamp_ns{0};
  double roll_rad{0}, pitch_rad{0};
  std::string reason;
};

class AttitudeGate
{
public:
  explicit AttitudeGate(AttitudeGateConfig config) : config_(config) {
    if (!std::isfinite(config.reference_roll_rad) ||
      !std::isfinite(config.reference_pitch_rad) ||
      std::abs(config.reference_roll_rad) > 3.141592653589793 ||
      std::abs(config.reference_pitch_rad) >= 1.5707963267948966 ||
      !std::isfinite(config.max_delta_rad) || config.max_delta_rad <= 0 ||
      config.max_delta_rad > 0.175 || !std::isfinite(config.max_sample_age_ms) ||
      config.max_sample_age_ms <= 0 || config.max_sample_age_ms > 1000)
    {
      throw std::invalid_argument("invalid static-BEV attitude gate config");
    }
  }

  // Reject non-increasing/future source stamps; a backward source clock latches
  // a fault until restart, rather than silently reusing pre-reset samples.
  void add(AttitudeSample sample, int64_t now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sample.stamp_ns <= 0 || sample.stamp_ns > now_ns) {
      fault_ = true; return;
    }
    if (!samples_.empty() && sample.stamp_ns <= samples_.back().stamp_ns) {
      fault_ = true; return;
    }
    if (!std::isfinite(sample.roll_rad) || !std::isfinite(sample.pitch_rad) ||
      std::abs(sample.roll_rad) > 3.141592653589793 ||
      std::abs(sample.pitch_rad) >= 1.5707963267948966)
    {
      sample.valid = false;
    }
    samples_.push_back(sample);
    while (samples_.size() > 128) {samples_.pop_front();}
  }

  AttitudeDecision at(int64_t image_stamp_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    AttitudeDecision out;
    if (fault_) {out.reason = "attitude clock/order fault; restart required"; return out;}
    // Causal matching: never borrow a pose measured AFTER this image, and never
    // skip a newer invalid sample to select an older apparently valid sample.
    for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
      if (it->stamp_ns > image_stamp_ns) {continue;}
      out.age_ms = static_cast<double>(image_stamp_ns - it->stamp_ns) / 1e6;
      out.sample_stamp_ns = it->stamp_ns;
      out.roll_rad = it->roll_rad; out.pitch_rad = it->pitch_rad;
      if (out.age_ms > config_.max_sample_age_ms) {
        out.reason = "attitude sample too old for image"; return out;
      }
      if (!it->valid) {out.reason = "attitude pose invalid/frame mismatch"; return out;}
      const auto delta = [](double a, double b) {
          return std::atan2(std::sin(a - b), std::cos(a - b));
        };
      out.roll_delta_rad = delta(it->roll_rad, config_.reference_roll_rad);
      out.pitch_delta_rad = delta(it->pitch_rad, config_.reference_pitch_rad);
      if (std::abs(out.roll_delta_rad) > config_.max_delta_rad ||
        std::abs(out.pitch_delta_rad) > config_.max_delta_rad)
      {
        out.reason = "attitude exceeds static-BEV calibration envelope"; return out;
      }
      out.usable = true;
      out.reason = "attitude checked; static BEV, no dynamic compensation";
      return out;
    }
    out.reason = "no causal attitude sample for image";
    return out;
  }
private:
  AttitudeGateConfig config_;
  mutable std::mutex mutex_;
  std::deque<AttitudeSample> samples_;
  bool fault_{false};
};
}  // namespace crop_row_perception
#endif
