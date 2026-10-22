// 版权所有 2026。点云避停门控的纯逻辑判定。
#ifndef PATH_TRACKER__OBSTACLE_GATE_HPP_
#define PATH_TRACKER__OBSTACLE_GATE_HPP_

namespace path_tracker
{

enum class ObstacleGateDecision
{
  kAllow,
  kUnavailable,
  kSensorInvalid,
  kStopRequired,
};

ObstacleGateDecision EvaluateObstacleGate(
  bool required,
  bool has_status,
  double status_age_s,
  double timeout_s,
  bool sensor_valid,
  bool stop_required);

}  // 命名空间 path_tracker

#endif  // PATH_TRACKER__OBSTACLE_GATE_HPP_
