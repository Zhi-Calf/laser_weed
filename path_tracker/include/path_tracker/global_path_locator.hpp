#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "path_tracker/discrete_points_math.hpp"

namespace path_tracker {

// 全局折线在车体附近的连续投影结果。
struct PathMatch {
  bool valid = false;                                                               // 投影是否成功
  TrajectoryPoint target_point;                                                     // 插值后的连续目标点
  std::size_t segment_index            = 0;                                         // 命中线段起点索引
  double segment_fraction              = 0.0;                                       // 命中线段内比例 [0, 1]
  double distance_m                    = 0.0;                                       // 车体中心到投影点距离 (m)
  double distance_to_next_turn_m       = std::numeric_limits<double>::infinity();   // 到下一 TURN 弧长 (m)
  double distance_from_previous_turn_m = std::numeric_limits<double>::infinity();   // 距上一 TURN 弧长 (m)
  bool at_path_end                     = false;                                     // 是否已投影到路径终点
};

// 在离散全局路径上执行单调前向的连续线段投影。
class GlobalPathLocator {
public:
  GlobalPathLocator() = default;

  void Configure(std::size_t forward_segment_window);
  void SetPath(const std::vector<TrajectoryPoint>& path);
  void Reset();
  bool Project(double x, double y, PathMatch* match);
  std::size_t path_point_count() const;

private:
  std::size_t forward_segment_window_ = 20;    // 每拍前向搜索的线段数
  std::size_t segment_index_          = 0;     // 上一拍命中的线段索引
  double target_s_                    = 0.0;   // 已接受的最大路径进度 (m)
  std::vector<TrajectoryPoint> path_;          // 已计算几何属性的全局路径
  std::vector<double> next_turn_s_;            // 各顶点之后最近 TURN 的弧长
  std::vector<double> previous_turn_s_;        // 各顶点之前最近 TURN 的弧长
};

}  // namespace path_tracker
