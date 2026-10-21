// Copyright 2026. 点云避停门控的纯逻辑判定实现。
#include "path_tracker/obstacle_gate.hpp"

namespace path_tracker
{

// 根据避停状态的新鲜度、感知有效性和节点请求
// 决定是否允许运动。
ObstacleGateDecision EvaluateObstacleGate(
  bool required,
  bool has_status,
  double status_age_s,
  double timeout_s,
  bool sensor_valid,
  bool stop_required)
{
  if (!required) {
    return ObstacleGateDecision::kAllow;
  }
  if (!has_status || status_age_s > timeout_s) {
    return ObstacleGateDecision::kUnavailable;
  }
  if (!sensor_valid) {
    return ObstacleGateDecision::kSensorInvalid;
  }
  if (stop_required) {
    return ObstacleGateDecision::kStopRequired;
  }
  return ObstacleGateDecision::kAllow;
}

}  // namespace path_tracker
