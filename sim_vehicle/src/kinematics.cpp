#include "sim_vehicle/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim_vehicle {
namespace {

// 将数值限制到闭区间 [lower, upper]。
// 输入:
//   value : 待限制的数值
//   lower : 下界
//   upper : 上界，调用方必须保证 lower <= upper
// 返回:
//   位于闭区间内的数值
double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

// 根据参考点纵向速度和横摆角速度计算轮心处的车体系平面速度。
// 输入:
//   speed_m_s     : 参考点沿车体 x 轴的速度 (m/s)
//   yaw_rate_rad_s: 绕车体 z 轴的横摆角速度 (rad/s, 左转为正)
//   x_m, y_m      : 轮心相对参考点的位置 (m)
// 返回:
//   (vx, vy)      : 轮心速度在车体系下的 x/y 分量 (m/s)
// 约束:
//   假设参考点无横向速度; 不包含轮胎侧滑和横滚/俯仰运动。
std::pair<double, double> ComputeWheelVelocity(double speed_m_s, double yaw_rate_rad_s, double x_m, double y_m) {
  return {speed_m_s - yaw_rate_rad_s * y_m, yaw_rate_rad_s * x_m};
}

}   // namespace

// 将角度归一化到 [-pi, pi]。
// 输入:
//   angle : 待归一化的角度 (rad)
// 返回:
//   归一化后的角度 (rad)
double WrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 按转向模式返回四轮相对底盘参考点的位置。
// 输入:
//   steer_mode    : 0 = 后轴参考, 其它 = 车体中心参考
//   wheelbase_m   : 前后轴距 (m)
//   wheel_track_m : 左右轮中心距 (m)
// 返回:
//   顺序由 WheelIndex 定义的四轮位置 (m)
std::array<WheelPosition, 4> ComputeWheelPositions(int steer_mode, double wheelbase_m, double wheel_track_m) {
  wheelbase_m                    = std::max(wheelbase_m, 1e-3);
  wheel_track_m                  = std::max(wheel_track_m, 1e-3);
  const double half_track_m      = wheel_track_m * 0.5;
  const bool rear_axle_reference = steer_mode == 0;
  const double front_x_m         = rear_axle_reference ? wheelbase_m : wheelbase_m * 0.5;
  const double rear_x_m          = rear_axle_reference ? 0.0 : -wheelbase_m * 0.5;

  return {{
    {front_x_m, half_track_m},
    {front_x_m, -half_track_m},
    {rear_x_m, half_track_m},
    {rear_x_m, -half_track_m},
  }};
}

// 返回以车体几何中心为原点的四轮位置。
// 输入:
//   wheelbase_m   : 前后轴距 (m)
//   wheel_track_m : 左右轮中心距 (m)
// 返回:
//   顺序由 WheelIndex 定义的四轮位置 (m)
std::array<WheelPosition, 4> ComputeCenteredWheelPositions(double wheelbase_m, double wheel_track_m) {
  wheelbase_m                 = std::max(wheelbase_m, 1e-3);
  wheel_track_m               = std::max(wheel_track_m, 1e-3);
  const double half_wheelbase = wheelbase_m * 0.5;
  const double half_track     = wheel_track_m * 0.5;
  return {{
    {half_wheelbase, half_track},
    {half_wheelbase, -half_track},
    {-half_wheelbase, half_track},
    {-half_wheelbase, -half_track},
  }};
}

// 由统一速度与等效转角反解四轮转向角和轮速。
// 输入:
//   speed_m_s       : 参考点纵向速度 (m/s)
//   steer_angle_rad : 等效转向角 (rad)
//   steer_mode      : 0 = 单阿克曼, 1 = 对称阿克曼
//   wheelbase_m     : 前后轴距 (m)
//   wheel_track_m   : 左右轮中心距 (m)
//   wheel_radius_m  : 车轮半径 (m)
//   max_steer_rad   : 单轮最大转角 (rad)
// 返回:
//   四轮转角、轮速、横摆角速度和等效轴距
// 失败:
//   不支持的 steer_mode 返回全零状态。
FourWheelState ComputeFourWheelState(double speed_m_s, double steer_angle_rad, int steer_mode, double wheelbase_m, double wheel_track_m, double wheel_radius_m, double max_steer_rad) {
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
  speed_m_s       = std::max(speed_m_s, 0.0);
  max_steer_rad   = std::max(max_steer_rad, 1e-3);
  steer_angle_rad = Clamp(steer_angle_rad, -max_steer_rad, max_steer_rad);
  wheelbase_m     = std::max(wheelbase_m, 1e-3);
  wheel_track_m   = std::max(wheel_track_m, 1e-3);
  wheel_radius_m  = std::max(wheel_radius_m, 1e-3);

  state.effective_wheelbase_m = steer_mode == 0 ? wheelbase_m : wheelbase_m * 0.5;
  state.yaw_rate_rad_s        = std::abs(steer_angle_rad) < 1e-12 ? 0.0 : speed_m_s * std::tan(steer_angle_rad) / state.effective_wheelbase_m;

  const auto positions = ComputeWheelPositions(steer_mode, wheelbase_m, wheel_track_m);
  for (std::size_t index = 0; index < 4; ++index) {
    const auto [velocity_x_mps, velocity_y_mps] = ComputeWheelVelocity(speed_m_s, state.yaw_rate_rad_s, positions[index].x_m, positions[index].y_m);
    const bool rear_wheel                       = index == static_cast<std::size_t>(WheelIndex::kRearLeft) || index == static_cast<std::size_t>(WheelIndex::kRearRight);

    if (steer_mode == 0 && rear_wheel) {
      state.steer_rad[index]   = 0.0;
      state.wheel_rad_s[index] = velocity_x_mps / wheel_radius_m;
      continue;
    }
    if (std::abs(velocity_x_mps) < 1e-6 && std::abs(velocity_y_mps) < 1e-6) {
      state.steer_rad[index]   = 0.0;
      state.wheel_rad_s[index] = 0.0;
      continue;
    }

    state.steer_rad[index]   = Clamp(std::atan2(velocity_y_mps, velocity_x_mps), -max_steer_rad, max_steer_rad);
    state.wheel_rad_s[index] = std::hypot(velocity_x_mps, velocity_y_mps) / wheel_radius_m;
  }
  return state;
}

}   // namespace sim_vehicle
