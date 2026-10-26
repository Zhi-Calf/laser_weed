#include "sim_vehicle/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim_vehicle {
namespace {

constexpr double kWrapPeriod = 6.28318530717958647692;
constexpr double kHalfWrapPeriod = kWrapPeriod * 0.5;
constexpr double kMinGeometryM = 1e-3;
constexpr double kZeroSpeedMps = 1e-6;
constexpr double kZeroAngleRad = 1e-12;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

std::pair<double, double> ComputeWheelVelocity(double speed_m_s,
                                               double yaw_rate_rad_s,
                                               double x_m, double y_m) {
  return {speed_m_s - yaw_rate_rad_s * y_m, yaw_rate_rad_s * x_m};
}

}  // namespace

// 将角度归一化到 [-pi, pi)。
double WrapAngle(double angle) {
  double wrapped = std::fmod(angle + kHalfWrapPeriod, kWrapPeriod);
  if (wrapped < 0.0) {
    wrapped += kWrapPeriod;
  }
  return wrapped - kHalfWrapPeriod;
}

// 按转向模式返回四轮相对底盘参考点的位置。
std::array<WheelPosition, kWheelCount> ComputeWheelPositions(
  int steer_mode, double wheelbase_m, double wheel_track_m) {
  wheelbase_m = std::max(wheelbase_m, kMinGeometryM);
  wheel_track_m = std::max(wheel_track_m, kMinGeometryM);
  const double half_track_m = wheel_track_m * 0.5;
  const bool rear_axle_reference = steer_mode == 0;
  const double front_x_m =
    rear_axle_reference ? wheelbase_m : wheelbase_m * 0.5;
  const double rear_x_m = rear_axle_reference ? 0.0 : -wheelbase_m * 0.5;

  return {{
    {front_x_m, half_track_m},
    {front_x_m, -half_track_m},
    {rear_x_m, half_track_m},
    {rear_x_m, -half_track_m},
  }};
}

// 返回以车体几何中心为原点的四轮位置。
std::array<WheelPosition, kWheelCount> ComputeCenteredWheelPositions(
  double wheelbase_m, double wheel_track_m) {
  wheelbase_m = std::max(wheelbase_m, kMinGeometryM);
  wheel_track_m = std::max(wheel_track_m, kMinGeometryM);
  const double half_wheelbase = wheelbase_m * 0.5;
  const double half_track = wheel_track_m * 0.5;
  return {{
    {half_wheelbase, half_track},
    {half_wheelbase, -half_track},
    {-half_wheelbase, half_track},
    {-half_wheelbase, -half_track},
  }};
}

// 由统一速度与等效转角反解四轮转向角和轮速。
FourWheelState ComputeFourWheelState(double speed_m_s, double steer_angle_rad,
                                     int steer_mode, double wheelbase_m,
                                     double wheel_track_m,
                                     double wheel_radius_m,
                                     double max_steer_rad) {
  FourWheelState state;
  if (steer_mode != 0 && steer_mode != 1) {
    return state;
  }

  if (!std::isfinite(speed_m_s)) {
    speed_m_s = 0.0;
  }
  if (!std::isfinite(steer_angle_rad)) {
    steer_angle_rad = 0.0;
  }
  speed_m_s = std::max(speed_m_s, 0.0);
  max_steer_rad = std::max(max_steer_rad, kMinGeometryM);
  steer_angle_rad = Clamp(steer_angle_rad, -max_steer_rad, max_steer_rad);
  wheelbase_m = std::max(wheelbase_m, kMinGeometryM);
  wheel_track_m = std::max(wheel_track_m, kMinGeometryM);
  wheel_radius_m = std::max(wheel_radius_m, kMinGeometryM);

  state.effective_wheelbase_m =
    steer_mode == 0 ? wheelbase_m : wheelbase_m * 0.5;
  state.yaw_rate_rad_s =
    std::abs(steer_angle_rad) < kZeroAngleRad
      ? 0.0
      : speed_m_s * std::tan(steer_angle_rad) / state.effective_wheelbase_m;

  const auto positions =
    ComputeWheelPositions(steer_mode, wheelbase_m, wheel_track_m);
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    const auto [velocity_x_mps, velocity_y_mps] =
      ComputeWheelVelocity(speed_m_s, state.yaw_rate_rad_s,
                           positions[index].x_m, positions[index].y_m);
    const bool rear_wheel =
      index == static_cast<std::size_t>(WheelIndex::kRearLeft) ||
      index == static_cast<std::size_t>(WheelIndex::kRearRight);

    if (steer_mode == 0 && rear_wheel) {
      state.steer_rad[index] = 0.0;
      state.wheel_rad_s[index] = velocity_x_mps / wheel_radius_m;
      continue;
    }
    if (std::abs(velocity_x_mps) < kZeroSpeedMps &&
        std::abs(velocity_y_mps) < kZeroSpeedMps) {
      state.steer_rad[index] = 0.0;
      state.wheel_rad_s[index] = 0.0;
      continue;
    }

    state.steer_rad[index] = Clamp(std::atan2(velocity_y_mps, velocity_x_mps),
                                   -max_steer_rad, max_steer_rad);
    state.wheel_rad_s[index] =
      std::hypot(velocity_x_mps, velocity_y_mps) / wheel_radius_m;
  }
  return state;
}

}  // namespace sim_vehicle
