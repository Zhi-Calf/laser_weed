// 版权所有 2026。点云 ROI 过滤与停车保持逻辑。
#ifndef VELOCITY_PLANNER__OBSTACLE_FILTER_HPP_
#define VELOCITY_PLANNER__OBSTACLE_FILTER_HPP_

#include <cstddef>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace velocity_planner {

struct RoiConfig {
  double forward_min_m{0.0};            // ROI 前向最小距离（m）
  double forward_max_m{2.0};            // ROI 前向最大距离（m）
  double half_width_m{0.9};             // ROI 左右半宽（m）
  double min_height_m{0.1};             // ROI 最低高度（m）
  double max_height_m{1.5};             // ROI 最高高度（m）
  std::size_t min_obstacle_points{3};   // 触发停车所需的最少点数
};

struct RoiResult {
  bool cloud_valid{false};           // 点云字段和坐标是否可用于判定
  bool stop_required{false};         // ROI 内点数是否达到停车阈值
  std::size_t obstacle_points{0};    // ROI 内的有效障碍点数
  float nearest_distance_m{-1.0F};   // 最近障碍的水平距离
};

class RoiFilter {
public:
  explicit RoiFilter(const RoiConfig& config);

  RoiResult Evaluate(const sensor_msgs::msg::PointCloud2& cloud) const;

private:
  RoiConfig config_;   // ROI 参数
};

class StopHoldFilter {
public:
  explicit StopHoldFilter(double hold_time_s);

  bool Update(bool obstacle_detected, double now_s);

private:
  double hold_time_s_;   // 障碍消失后的最短停车保持时间
  double last_hit_s_;    // 最近一次命中障碍的时间
  bool has_hit_;         // 是否已经记录过障碍命中
};

}   // namespace velocity_planner

#endif   // VELOCITY_PLANNER__OBSTACLE_FILTER_HPP_
