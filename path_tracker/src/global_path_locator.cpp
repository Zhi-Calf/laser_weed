#include "path_tracker/global_path_locator.hpp"

#include <algorithm>
#include <cmath>

namespace path_tracker {
namespace {

// 将角度归一化到 [-pi, pi]。
double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

// 配置每拍允许向前搜索的路径线段数量。
// 输入:
//   forward_segment_window : 至少为 1 的线段数
void GlobalPathLocator::Configure(std::size_t forward_segment_window) {
  forward_segment_window_ = std::max<std::size_t>(forward_segment_window, 1U);
  Reset();
}

// 替换全局路径并预计算前后 TURN 弧长。
// 输入:
//   path : 已包含航向、曲率、弧长和路径类型的离散轨迹
// 输出:
//   path_ 和 TURN 索引表；空路径或单点路径只保留点数，Project 将失败
void GlobalPathLocator::SetPath(const std::vector<TrajectoryPoint>& path) {
  path_ = path;
  next_turn_s_.assign(path_.size() + 1,
                      std::numeric_limits<double>::infinity());
  previous_turn_s_.assign(path_.size() + 1,
                          -std::numeric_limits<double>::infinity());
  for (std::size_t index = path_.size(); index > 0; --index) {
    const std::size_t point_index = index - 1;
    next_turn_s_[point_index] = next_turn_s_[point_index + 1];
    if (path_[point_index].waypoint_type == 1) {
      next_turn_s_[point_index] = path_[point_index].s;
    }
  }
  for (std::size_t index = 0; index < path_.size(); ++index) {
    previous_turn_s_[index] =
        index == 0 ? -std::numeric_limits<double>::infinity()
                   : previous_turn_s_[index - 1];
    if (path_[index].waypoint_type == 1) {
      previous_turn_s_[index] = path_[index].s;
    }
  }
  Reset();
}

// 清除单调推进状态，不删除路径。
void GlobalPathLocator::Reset() {
  segment_index_ = 0;
  target_s_ = 0.0;
}

// 将车体中心投影到上一线段之后有限前向窗口中的最近折线段。
// 输入:
//   x, y : 车体中心在 ENU 平面中的位置 (m)
// 返回:
//   true = 找到有效投影；false = 路径不足、退化或输出指针为空
// 输出:
//   match : 连续投影点、线段索引、误差距离和 TURN 弧长；失败时清空
bool GlobalPathLocator::Project(double x, double y, PathMatch* match) {
  if (match == nullptr) {
    return false;
  }
  *match = PathMatch{};
  if (!std::isfinite(x) || !std::isfinite(y) || path_.size() < 2 ||
      segment_index_ + 1 >= path_.size()) {
    return false;
  }

  const std::size_t last_segment = path_.size() - 2;
  const std::size_t search_end =
      std::min(last_segment,
               segment_index_ + forward_segment_window_ - 1);
  std::size_t best_index = segment_index_;
  double best_fraction = 0.0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = segment_index_; index <= search_end; ++index) {
    const TrajectoryPoint& start = path_[index];
    const TrajectoryPoint& end = path_[index + 1];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (!std::isfinite(length_squared) || length_squared < 1e-12) {
      continue;
    }
    double fraction =
        ((x - start.x) * dx + (y - start.y) * dy) / length_squared;
    fraction = std::clamp(fraction, 0.0, 1.0);
    const double segment_length = end.s - start.s;
    if (segment_length > 1e-12 && target_s_ > start.s) {
      const double minimum_fraction =
          (target_s_ - start.s) / segment_length;
      fraction = std::max(fraction, std::min(minimum_fraction, 1.0));
    }
    const double projection_x = start.x + fraction * dx;
    const double projection_y = start.y + fraction * dy;
    const double error_x = x - projection_x;
    const double error_y = y - projection_y;
    const double distance_squared =
        error_x * error_x + error_y * error_y;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_index = index;
      best_fraction = fraction;
    }
  }
  if (!std::isfinite(best_distance_squared)) {
    return false;
  }

  const TrajectoryPoint& start = path_[best_index];
  const TrajectoryPoint& end = path_[best_index + 1];
  segment_index_ = best_index;
  match->valid = true;
  match->segment_index = best_index;
  match->segment_fraction = best_fraction;
  match->distance_m = std::sqrt(best_distance_squared);
  match->target_point.x =
      start.x + best_fraction * (end.x - start.x);
  match->target_point.y =
      start.y + best_fraction * (end.y - start.y);
  match->target_point.heading = NormalizeAngle(
      start.heading +
      best_fraction * NormalizeAngle(end.heading - start.heading));
  match->target_point.kappa =
      start.kappa + best_fraction * (end.kappa - start.kappa);
  match->target_point.dkappa =
      start.dkappa + best_fraction * (end.dkappa - start.dkappa);
  match->target_point.s =
      start.s + best_fraction * (end.s - start.s);
  target_s_ = std::max(target_s_, match->target_point.s);
  match->target_point.waypoint_type =
      best_fraction < 0.5 ? start.waypoint_type : end.waypoint_type;

  const double next_turn_s = next_turn_s_[best_index];
  if (std::isfinite(next_turn_s)) {
    match->distance_to_next_turn_m =
        std::max(0.0, next_turn_s - match->target_point.s);
  }
  const double previous_turn_s = previous_turn_s_[best_index];
  if (std::isfinite(previous_turn_s)) {
    match->distance_from_previous_turn_m =
        std::max(0.0, match->target_point.s - previous_turn_s);
  }
  match->at_path_end =
      best_index == last_segment && best_fraction >= 1.0 - 1e-9;
  return true;
}

// 返回当前全局路径点数。
std::size_t GlobalPathLocator::path_point_count() const {
  return path_.size();
}

}  // namespace path_tracker
