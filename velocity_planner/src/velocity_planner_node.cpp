// Copyright 2026. 点云前向矩形避停状态节点实现。
#include "velocity_planner/velocity_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace velocity_planner
{

namespace
{

constexpr const char * kNodeName = "velocity_planner";  // 节点名
constexpr int kWarnIntervalMs = 3000;                  // 告警限流周期

}  // namespace

// 初始化 TF、参数、话题和定时器。
VelocityPlannerNode::VelocityPlannerNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  LoadParameters();
  SetupInterfaces();

  RCLCPP_INFO(
    get_logger(),
    "点云避停节点已启动: 点云=%s 目标坐标系=%s 状态=%s "
    "ROI前向=[%.2f,%.2f]m 半宽=%.2f m 高度=[%.2f,%.2f]m",
    input_cloud_topic_.c_str(),
    target_frame_.c_str(),
    output_status_topic_.c_str(),
    roi_config_.forward_min_m,
    roi_config_.forward_max_m,
    roi_config_.half_width_m,
    roi_config_.min_height_m,
    roi_config_.max_height_m);
}

// 声明并校验点云避停参数。
void VelocityPlannerNode::LoadParameters()
{
  input_cloud_topic_ = declare_parameter<std::string>(
    "input_cloud_topic", "/rslidar_points");
  output_status_topic_ = declare_parameter<std::string>(
    "output_status_topic", "/velocity_planner/status");
  target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
  cloud_timeout_s_ = declare_parameter<double>("cloud_timeout_s", 0.5);
  tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.1);
  stop_hold_s_ = declare_parameter<double>("stop_hold_s", 0.3);

  roi_config_.forward_min_m =
    declare_parameter<double>("roi_forward_min_m", 0.0);
  roi_config_.forward_max_m =
    declare_parameter<double>("roi_forward_max_m", 2.0);
  roi_config_.half_width_m =
    declare_parameter<double>("roi_half_width_m", 0.9);
  roi_config_.min_height_m =
    declare_parameter<double>("roi_min_height_m", 0.1);
  roi_config_.max_height_m =
    declare_parameter<double>("roi_max_height_m", 1.5);
  const int64_t min_points_raw =
    declare_parameter<int64_t>("min_obstacle_points", 3);

  if (input_cloud_topic_.empty()) {
    input_cloud_topic_ = "/rslidar_points";
  }
  if (output_status_topic_.empty()) {
    output_status_topic_ = "/velocity_planner/status";
  }
  if (target_frame_.empty()) {
    target_frame_ = "base_link";
  }

  publish_hz_ = std::clamp(publish_hz_, 1.0, 50.0);
  cloud_timeout_s_ = std::max(cloud_timeout_s_, 0.1);
  tf_timeout_s_ = std::max(tf_timeout_s_, 0.01);
  stop_hold_s_ = std::max(stop_hold_s_, 0.0);
  roi_config_.half_width_m = std::max(roi_config_.half_width_m, 0.01);
  if (roi_config_.forward_max_m <= roi_config_.forward_min_m) {
    RCLCPP_WARN(
      get_logger(),
      "ROI 前向范围无效，"
      "已将 forward_max_m 调整为 forward_min_m + 0.1");
    roi_config_.forward_max_m = roi_config_.forward_min_m + 0.1;
  }
  if (roi_config_.max_height_m <= roi_config_.min_height_m) {
    RCLCPP_WARN(
      get_logger(),
      "ROI 高度范围无效，"
      "已将 max_height_m 调整为 min_height_m + 0.1");
    roi_config_.max_height_m = roi_config_.min_height_m + 0.1;
  }
  roi_config_.min_obstacle_points =
    static_cast<std::size_t>(std::max<int64_t>(min_points_raw, 1));

  roi_filter_ = std::make_unique<RoiFilter>(roi_config_);
  stop_hold_ = std::make_unique<StopHoldFilter>(stop_hold_s_);
}

// 创建点云订阅、状态发布和固定频率定时器。
void VelocityPlannerNode::SetupInterfaces()
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
      CloudCallback(cloud);
    });

  status_pub_ = create_publisher<weeder_msgs::msg::ObstacleStatus>(
    output_status_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_hz_));
  timer_ = create_wall_timer(period, [this]() {PublishStatus();});
}

// 保存最新点云，实际处理在固定频率定时器中进行。
void VelocityPlannerNode::CloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_cloud_ = cloud;
  has_pending_cloud_ = true;
}

// 将点云转换到目标坐标系并执行 ROI 判定。
void VelocityPlannerNode::ProcessCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const rclcpp::Time & now)
{
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  try {
    tf_buffer_.transform(
      cloud,
      transformed_cloud,
      target_frame_,
      tf2::durationFromSec(tf_timeout_s_));
  } catch (const tf2::TransformException & exception) {
    cached_sensor_valid_ = false;
    cached_stop_required_ = true;
    cached_obstacle_points_ = 0;
    cached_nearest_distance_m_ = -1.0F;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      kWarnIntervalMs,
      "点云 TF 转换失败 (%s -> %s): %s，避停按失效停车",
      cloud.header.frame_id.c_str(),
      target_frame_.c_str(),
      exception.what());
    return;
  }

  const RoiResult result = roi_filter_->Evaluate(transformed_cloud);
  const bool hold_stop = stop_hold_->Update(
    result.stop_required, now.seconds());
  cached_sensor_valid_ = result.cloud_valid;
  cached_stop_required_ =
    !result.cloud_valid || result.stop_required || hold_stop;
  cached_obstacle_points_ = result.obstacle_points;
  cached_nearest_distance_m_ = result.nearest_distance_m;
  has_cloud_result_ = true;
  last_cloud_result_time_ = now;

  if (!result.cloud_valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      kWarnIntervalMs,
      "点云缺少 x/y/z FLOAT32 字段，避停按失效停车");
  }
}

// 发布固定频率的避停状态，
// 并在点云失效时输出停车请求。
void VelocityPlannerNode::PublishStatus()
{
  weeder_msgs::msg::ObstacleStatus status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = this->now();
    if (has_pending_cloud_ && pending_cloud_) {
      ProcessCloud(*pending_cloud_, now);
      pending_cloud_.reset();
      has_pending_cloud_ = false;
    }

    bool sensor_valid = cached_sensor_valid_;
    bool stop_required = cached_stop_required_;
    std::size_t obstacle_points = cached_obstacle_points_;
    float nearest_distance_m = cached_nearest_distance_m_;

    if (!has_cloud_result_ ||
      (now - last_cloud_result_time_).seconds() > cloud_timeout_s_)
    {
      sensor_valid = false;
      stop_required = true;
      obstacle_points = 0;
      nearest_distance_m = -1.0F;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        kWarnIntervalMs,
        "点云超过 %.2fs 未产生有效避停结果，输出停车请求",
        cloud_timeout_s_);
    }

    status.header.stamp = now;
    status.header.frame_id = target_frame_;
    status.sensor_valid = sensor_valid;
    status.stop_required = stop_required;
    status.obstacle_points = static_cast<uint32_t>(obstacle_points);
    status.nearest_distance_m = nearest_distance_m;
  }

  status_pub_->publish(status);
}

}  // namespace velocity_planner
