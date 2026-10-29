#pragma once

#include "path_tracker/global_path_locator.hpp"

namespace path_tracker {

// 视觉中心线在车体参考点处的解析直线参数。
struct LineReference {
  double lateral_error_m   = 0.0;   // x=0 截距，路径在左为正
  double heading_error_rad = 0.0;   // 路径相对车体航向，向左为正
  double curvature         = 0.0;   // 路径曲率 (1/m)，左转为正
};

struct LQRInputResult {
  bool success         = false;   // 输入计算是否成功
  double lateral_error = 0.0;     // 车辆相对路径横向误差，左正 (m)
  double heading_error = 0.0;     // 车辆航向减路径航向 (rad)
  double target_kappa  = 0.0;     // 目标路径曲率 (1/m)
};

class LQRInputCalculator {
public:
  LQRInputResult CalculatePath(const RobotState& control_state, const TrajectoryPoint& target) const;
  LQRInputResult CalculateLine(const LineReference& reference) const;
};

}   // namespace path_tracker
