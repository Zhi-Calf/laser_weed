#pragma once

#include <vector>

#include "path_tracker/discrete_points_math.hpp"

namespace path_tracker {

// 当前路径点窗口对应的工作段类型。
enum class PathWorkMode {
  kWork,
  kTurn,
};

// 车辆在 ENU 平面中的运动状态。
struct RobotState {
  double x        = 0.0;   // ENU 坐标系 x 坐标 (m)
  double y        = 0.0;   // ENU 坐标系 y 坐标 (m)
  double heading  = 0.0;   // 车辆航向角 (rad)
  double linear_v = 0.0;   // 实测前向速度 (m/s)
};

// 全局路径在车体附近的最近点搜索结果。
struct PathMatchResult {
  bool success                   = false;                 // 搜索是否成功
  TrajectoryPoint nearest_point;                          // 距离车辆最近的路径点
  TrajectoryPoint target_point;                           // 按工作模式选择的目标点
  int point_index                = -1;                    // 目标路径点索引
  int nearest_index              = -1;                    // 最近路径点索引
  double nearest_distance_m      = 0.0;                   // 车辆到最近路径点的距离 (m)
  bool at_path_end               = false;                 // 是否命中路径终点
  PathWorkMode work_mode         = PathWorkMode::kWork;   // 当前工作模式
  RobotState control_state;                               // 当前模式的控制参考状态
};

struct GlobalPathLocatorConfig {
  int forward_point_window       = 8;      // 最近点前向搜索点数
  double turn_mode_enter_m       = 0.6;    // 进入 TURN 模式前视距离 (m)
  double turn_mode_exit_m        = 0.6;    // 退出 TURN 模式保持距离 (m)
  double wheelbase_m             = 1.0;    // 车辆物理轴距 (m)
  double goal_reached_distance_m = 1.0;    // 距路径终点的到达阈值 (m)
  bool pose_at_vehicle_center    = true;   // 位姿参考点是否为几何中心
};

// 在离散全局路径上执行单调前向的最近点搜索。
class GlobalPathLocator {
public:
  GlobalPathLocator() = default;
  void Configure(const GlobalPathLocatorConfig& config);
  void SetPath(const std::vector<TrajectoryPoint>& path);
  void Reset();
  PathMatchResult Project(const RobotState& robot_state);
  int path_point_count() const;

private:
  int ProjectNear(double x, double y, const TrajectoryPoint& nearest_point) const;
  int FindNearestPoint(int first_index, int last_index, double x, double y) const;
  bool UpdateWorkMode(int nearest_index);

private:
  GlobalPathLocatorConfig config_;                 // 最近点搜索和 TURN 窗口配置
  int point_index_        = 0;                     // 几何中心单调进度索引，局部搜索不得回写
  PathWorkMode work_mode_ = PathWorkMode::kWork;   // 保留跨周期滞回的工作模式
  std::vector<TrajectoryPoint> path_;              // 已计算几何属性的全局路径
};

}   // namespace path_tracker
