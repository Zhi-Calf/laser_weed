#include "path_tracker/lqr_input.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

namespace path_tracker {
namespace {

// 将角度归一化到 [-pi, pi]。
// 输入:
//   angle : 待归一化角度 (rad)
// 返回:
//   归一化后的角度 (rad)
double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}   // namespace

// 配置路径和视觉输入计算。
// 输入:
//   config : 轴距和位姿参考点配置
void LQRInputCalculator::Configure(const LQRInputConfig& config) {
  config_ = config;
  SPDLOG_INFO(
    "LQRInputCalculator 配置: wheelbase={:.3f}m "
    "pose_at_center={}",
    config_.wheelbase,
    config_.pose_at_vehicle_center);
}

// 根据控制参考状态和目标点计算路径跟踪输入。
// 输入:
//   control_state : 当前模式的控制参考状态
//   target        : 控制点对应的目标路径点
// 返回:
//   横向误差、航向误差和目标曲率；输入非法时 success 为 false
LQRInputResult LQRInputCalculator::CalculatePath(const RobotState& control_state, const TrajectoryPoint& target) const {
  LQRInputResult result;
  if (
    !std::isfinite(control_state.x) || !std::isfinite(control_state.y) || !std::isfinite(control_state.heading) || !std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.heading) ||
    !std::isfinite(target.kappa)) {
    return result;
  }

  const double dx      = control_state.x - target.x;
  const double dy      = control_state.y - target.y;
  result.lateral_error = dy * std::cos(target.heading) - dx * std::sin(target.heading);
  result.heading_error = NormalizeAngle(control_state.heading - target.heading);
  result.target_kappa  = target.kappa;
  result.success       = std::isfinite(result.lateral_error) && std::isfinite(result.heading_error);
  return result;
}

// 根据视觉直线参考计算路径跟踪输入。
// 输入:
//   reference : 车体参考点处的视觉直线参数
// 返回:
//   转换后的横向误差、航向误差和目标曲率
LQRInputResult LQRInputCalculator::CalculateLine(const LineReference& reference) const {
  LQRInputResult result;
  if (!std::isfinite(reference.lateral_error_m) || !std::isfinite(reference.heading_error_rad) || !std::isfinite(reference.curvature) || std::abs(std::cos(reference.heading_error_rad)) < 1e-9) {
    return result;
  }

  const double control_offset    = config_.pose_at_vehicle_center ? config_.wheelbase * 0.5 : 0.0;
  const double slope             = std::tan(reference.heading_error_rad);
  const double path_y_at_control = reference.lateral_error_m - slope * control_offset;
  result.lateral_error           = -path_y_at_control;
  result.heading_error           = -reference.heading_error_rad;
  result.target_kappa            = reference.curvature;
  result.success                 = std::isfinite(result.lateral_error) && std::isfinite(result.heading_error);
  return result;
}

}   // namespace path_tracker
