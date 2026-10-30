#include "path_tracker/global_path_locator.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_tracker {
namespace {

// 计算查询点到离散路径点的平方距离。
// 输入:
//   point : 离散路径点
//   x, y  : 查询点在 ENU 平面中的位置 (m)
// 返回:
//   两点之间的平方距离 (m²)
double SquaredDistance(const TrajectoryPoint& point, double x, double y) {
  const double dx = x - point.x;
  const double dy = y - point.y;
  return dx * dx + dy * dy;
}

struct TurnSearchResult {
  bool found        = false;                                     // 是否找到 TURN
  int index         = -1;                                        // TURN 点索引
  double distance_m = std::numeric_limits<double>::infinity();   // TURN 弧长距离
};

// 从最近点向前搜索给定弧长内的第一个 TURN。
// 输入:
//   path          : 全局路径
//   nearest_index : 几何中心最近点索引
//   max_distance_m : 最大前向搜索距离 (m)
// 返回:
//   第一个 TURN 的搜索信息；未找到时 found 为 false
TurnSearchResult FindTurnAhead(const std::vector<TrajectoryPoint>& path, int nearest_index, double max_distance_m) {
  TurnSearchResult result;
  const double nearest_s = path[nearest_index].s;
  const int path_size    = static_cast<int>(path.size());
  for (int index = nearest_index; index < path_size; ++index) {
    const double distance_m = path[index].s - nearest_s;
    if (!std::isfinite(distance_m) || distance_m > max_distance_m) {
      break;
    }
    if (path[index].waypoint_type == 1) {
      result.found      = true;
      result.index      = index;
      result.distance_m = std::max(0.0, distance_m);
      break;
    }
  }
  return result;
}

// 从最近点向后搜索给定弧长内的最近 TURN。
// 输入:
//   path          : 全局路径
//   nearest_index : 几何中心最近点索引
//   max_distance_m : 最大后向搜索距离 (m)
// 返回:
//   最近 TURN 的搜索信息；未找到时 found 为 false
TurnSearchResult FindTurnBehind(const std::vector<TrajectoryPoint>& path, int nearest_index, double max_distance_m) {
  TurnSearchResult result;
  const double nearest_s = path[nearest_index].s;
  int index              = nearest_index + 1;
  while (index > 0) {
    --index;
    const double distance_m = nearest_s - path[index].s;
    if (!std::isfinite(distance_m) || distance_m > max_distance_m) {
      break;
    }
    if (path[index].waypoint_type == 1) {
      result.found      = true;
      result.index      = index;
      result.distance_m = std::max(0.0, distance_m);
      break;
    }
  }
  return result;
}

}   // namespace

// 配置并归一化搜索窗口、TURN 模式和终点阈值。
// 输入:
//   config : 定位器完整配置
// 副作用:
//   保存归一化配置、打印日志并重置搜索进度。
void GlobalPathLocator::Configure(const GlobalPathLocatorConfig& config) {
  config_                         = config;
  config_.forward_point_window    = std::max(config_.forward_point_window, 1);
  config_.turn_mode_enter_m       = std::isfinite(config_.turn_mode_enter_m) ? std::max(config_.turn_mode_enter_m, 0.0) : 0.6;
  config_.turn_mode_exit_m        = std::isfinite(config_.turn_mode_exit_m) ? std::max(config_.turn_mode_exit_m, 0.0) : 0.6;
  config_.wheelbase_m             = std::isfinite(config_.wheelbase_m) && config_.wheelbase_m > 0.0 ? config_.wheelbase_m : 1.0;
  config_.goal_reached_distance_m = std::isfinite(config_.goal_reached_distance_m) ? std::max(config_.goal_reached_distance_m, 0.0) : 1.0;
  SPDLOG_INFO("GlobalPathLocator 搜索配置: forward_point_window={}", config_.forward_point_window);
  SPDLOG_INFO("GlobalPathLocator 模式配置: enter={:.2f}m exit={:.2f}m", config_.turn_mode_enter_m, config_.turn_mode_exit_m);
  SPDLOG_INFO(
    "GlobalPathLocator 几何配置: wheelbase={:.3f}m "
    "goal_reached={:.2f}m pose_at_center={}",
    config_.wheelbase_m,
    config_.goal_reached_distance_m,
    config_.pose_at_vehicle_center);
  Reset();
}

// 替换全局路径并重置最近点进度。
// 输入:
//   path : 已包含航向、曲率、弧长和路径类型的离散轨迹
// 副作用:
//   覆盖当前路径并清除搜索进度。
void GlobalPathLocator::SetPath(const std::vector<TrajectoryPoint>& path) {
  path_ = path;
  Reset();
}

// 清除单调推进状态，不删除路径。
// 副作用:
//   最近点索引归零，工作模式恢复为 WORK。
void GlobalPathLocator::Reset() {
  point_index_ = 0;
  work_mode_   = PathWorkMode::kWork;
}

// 在上一命中点之后有限前向窗口内搜索最近离散路径点。
// 输入:
//   robot_state : 车体位姿和速度
// 返回:
//   成功标志和最近点匹配；失败时返回默认结果
// 副作用:
//   仅用几何中心最近点更新 point_index_ 和跨周期工作模式。
PathMatchResult GlobalPathLocator::Project(const RobotState& robot_state) {
  const int path_size = static_cast<int>(path_.size());
  if (!std::isfinite(robot_state.x) || !std::isfinite(robot_state.y) || !std::isfinite(robot_state.heading) || path_size < 2 || point_index_ < 0 || point_index_ >= path_size) {
    return PathMatchResult{};
  }
  const double goal_distance_squared = SquaredDistance(path_.back(), robot_state.x, robot_state.y);
  const int search_end               = std::min(path_size - 1, point_index_ + config_.forward_point_window);
  const int nearest_point_index      = FindNearestPoint(point_index_, search_end, robot_state.x, robot_state.y);
  if (nearest_point_index < 0) {
    return PathMatchResult{};
  }

  PathMatchResult result;
  result.success            = true;
  result.nearest_point      = path_[nearest_point_index];
  result.target_point       = path_[nearest_point_index];
  result.point_index        = nearest_point_index;
  result.nearest_index      = nearest_point_index;
  result.nearest_distance_m = std::sqrt(
      SquaredDistance(result.nearest_point, robot_state.x, robot_state.y));
  result.at_path_end        = nearest_point_index + 1 == path_size || goal_distance_squared <= config_.goal_reached_distance_m * config_.goal_reached_distance_m;
  result.work_mode          = UpdateWorkMode(result.nearest_index) ? PathWorkMode::kTurn : PathWorkMode::kWork;
  point_index_              = result.nearest_index;
  result.control_state      = robot_state;

  if (result.work_mode == PathWorkMode::kWork) {
    const double rear_axle_offset_m = config_.pose_at_vehicle_center ? config_.wheelbase_m * 0.5 : 0.0;
    result.control_state.x -= rear_axle_offset_m * std::cos(robot_state.heading);
    result.control_state.y -= rear_axle_offset_m * std::sin(robot_state.heading);
    const int rear_axle_index = ProjectNear(result.control_state.x, result.control_state.y, result.nearest_point);
    if (rear_axle_index < 0) {
      result.success = false;
      return result;
    }
    result.target_point = path_[rear_axle_index];
    result.point_index  = rear_axle_index;
  }
  return result;
}

// 在以锚点最近点为中心的局部弧长窗口内搜索控制点。
// 输入:
//   x, y         : 待匹配控制点在 ENU 平面中的位置 (m)
//   nearest_point : 几何中心最近点，用于限定路径分支和搜索窗口
// 返回:
//   局部最近路径点索引；失败时返回 -1
// 副作用:
//   无；不得修改全局单调进度 point_index_
int GlobalPathLocator::ProjectNear(double x, double y, const TrajectoryPoint& nearest_point) const {
  const double path_start_s   = path_.front().s;
  const double path_end_s     = path_.back().s;
  const double search_start_s = std::max(path_start_s, nearest_point.s - config_.wheelbase_m);
  const double search_end_s   = std::min(path_end_s, nearest_point.s + config_.wheelbase_m);
  if (!std::isfinite(search_start_s) || !std::isfinite(search_end_s) || search_start_s > search_end_s) {
    return -1;
  }

  const auto first = std::lower_bound(path_.begin(), path_.end(), search_start_s, [](const TrajectoryPoint& point, double s) {
    return point.s < s;
  });
  const auto last  = std::upper_bound(path_.begin(), path_.end(), search_end_s, [](double s, const TrajectoryPoint& point) {
    return s < point.s;
  });
  if (first == last) {
    return -1;
  }
  const int first_index = static_cast<int>(first - path_.begin());
  const int last_index  = static_cast<int>(last - path_.begin() - 1);
  return FindNearestPoint(first_index, last_index, x, y);
}

// 在闭合索引范围内搜索距离查询点最近的离散路径点。
// 输入:
//   first_index : 搜索起点索引
//   last_index  : 搜索终点索引
//   x, y        : 查询点在 ENU 平面中的位置 (m)
// 返回:
//   最近点索引；索引无效或未找到时返回 -1
int GlobalPathLocator::FindNearestPoint(int first_index, int last_index, double x, double y) const {
  const int path_size = static_cast<int>(path_.size());
  if (first_index < 0 || first_index > last_index || last_index >= path_size) {
    return -1;
  }

  int best_index               = first_index;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  bool found                   = false;
  for (int index = first_index; index <= last_index; ++index) {
    const double distance_squared = SquaredDistance(path_[index], x, y);
    if (!std::isfinite(distance_squared)) {
      continue;
    }
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_index            = index;
      found                 = true;
    }
  }
  if (!found) {
    return -1;
  }
  return best_index;
}

// 根据进入/保持窗口更新工作模式。
// 输入:
//   nearest_index : 几何中心最近点索引
// 返回:
//   true = 更新后处于 TURN 模式；false = WORK 模式
// 副作用:
//   更新跨周期工作模式状态。
bool GlobalPathLocator::UpdateWorkMode(int nearest_index) {
  const bool nearest_is_turn    = path_[nearest_index].waypoint_type == 1;
  const TurnSearchResult ahead  = FindTurnAhead(path_, nearest_index, config_.turn_mode_enter_m);
  const TurnSearchResult behind = FindTurnBehind(path_, nearest_index, config_.turn_mode_exit_m);
  const bool enter_turn         = nearest_is_turn || (ahead.found && ahead.distance_m <= config_.turn_mode_enter_m);
  const bool keep_turn          = nearest_is_turn || (ahead.found && ahead.distance_m <= config_.turn_mode_enter_m) || (behind.found && behind.distance_m <= config_.turn_mode_exit_m);

  if (work_mode_ == PathWorkMode::kWork && enter_turn) {
    work_mode_ = PathWorkMode::kTurn;
  } else if (work_mode_ == PathWorkMode::kTurn && !keep_turn) {
    work_mode_ = PathWorkMode::kWork;
  }
  return work_mode_ == PathWorkMode::kTurn;
}

// 返回当前全局路径点数。
// 返回:
//   已载入路径的离散点数
int GlobalPathLocator::path_point_count() const {
  return static_cast<int>(path_.size());
}

}   // namespace path_tracker
