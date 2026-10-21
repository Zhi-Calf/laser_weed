// Copyright 2026. 点云 ROI 过滤与停车保持逻辑实现。
#include "velocity_planner/obstacle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>

namespace velocity_planner {

namespace {

// 检查点云是否包含指定名称的 FLOAT32 字段。
bool HasFloatField(const sensor_msgs::msg::PointCloud2& cloud, const std::string& field_name) {
  for (const auto& field : cloud.fields) {
    if (field.name == field_name) {
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    }
  }
  return false;
}

}   // namespace

// 保存 ROI 参数。
RoiFilter::RoiFilter(const RoiConfig& config) : config_(config) {}

// 遍历点云并统计前向矩形 ROI 内的有效障碍点。
RoiResult RoiFilter::Evaluate(const sensor_msgs::msg::PointCloud2& cloud) const {
  RoiResult result;
  if (!HasFloatField(cloud, "x") || !HasFloatField(cloud, "y") || !HasFloatField(cloud, "z")) {
    return result;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  double nearest_distance   = -1.0;
  std::size_t finite_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double x = static_cast<double>(*iter_x);
    const double y = static_cast<double>(*iter_y);
    const double z = static_cast<double>(*iter_z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    ++finite_points;
    if (x < config_.forward_min_m || x > config_.forward_max_m || std::fabs(y) > config_.half_width_m || z < config_.min_height_m || z > config_.max_height_m) {
      continue;
    }

    ++result.obstacle_points;
    const double distance = std::hypot(x, y);
    if (nearest_distance < 0.0 || distance < nearest_distance) {
      nearest_distance = distance;
    }
  }

  if (finite_points == 0) {
    return result;
  }

  result.cloud_valid = true;
  if (nearest_distance >= 0.0) {
    result.nearest_distance_m = static_cast<float>(nearest_distance);
  }
  result.stop_required = result.obstacle_points >= config_.min_obstacle_points;
  return result;
}

// 保存障碍消失后的停车保持时间。
StopHoldFilter::StopHoldFilter(double hold_time_s) : hold_time_s_(std::max(hold_time_s, 0.0)), last_hit_s_(0.0), has_hit_(false) {}

// 更新障碍命中状态，
// 并在障碍消失后保持输出停车一段时间。
bool StopHoldFilter::Update(bool obstacle_detected, double now_s) {
  if (obstacle_detected) {
    has_hit_    = true;
    last_hit_s_ = now_s;
    return true;
  }
  if (!has_hit_) {
    return false;
  }

  // 时间回退时重新锚定，
  // 避免仿真时钟重置造成无限期停车。
  if (now_s < last_hit_s_) {
    last_hit_s_ = now_s;
  }
  const bool keep_stopping = (now_s - last_hit_s_) < hold_time_s_;
  if (!keep_stopping) {
    has_hit_ = false;
  }
  return keep_stopping;
}

}   // namespace velocity_planner
