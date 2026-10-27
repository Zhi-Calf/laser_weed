// 版权所有 2026。点云前向矩形避停状态节点实现。
#include "velocity_planner/velocity_planner_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "velocity_planner/logging.hpp"

namespace velocity_planner {

namespace {

constexpr const char* kNodeName = "velocity_planner";   // 节点名
constexpr double kPublishHz     = 10.0;                 // 状态固定发布频率

}   // namespace

// 初始化参数、话题和定时器。
VelocityPlannerNode::VelocityPlannerNode(const rclcpp::NodeOptions& options) : Node(kNodeName, options) {
  LoadLogParameters();

  LogConfig log_config;
  log_config.logger_name         = "velocity_planner";
  log_config.log_file_name       = "velocity_planner.log";
  log_config.log_dir             = log_dir_;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb_) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count_);
  log_config.enable_console      = log_enable_console_;
  log_config.log_level           = log_level_;
  try {
    ConfigureLogging(log_config);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& error) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", error.what());
  }

  LoadParameters();
  SetupInterfaces();
}

// 声明并读取 spdlog 滚动日志参数。
void VelocityPlannerNode::LoadLogParameters() {
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 50);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

// 声明并校验点云避停参数。
void VelocityPlannerNode::LoadParameters() {
  declare_parameter("input_cloud_topic", "/rslidar_points");
  declare_parameter("output_status_topic", "/velocity_planner/status");
  declare_parameter("point_cloud_timeout_s", 0.5);
  declare_parameter("stop_hold_s", 0.3);
  declare_parameter("roi_forward_min_m", 0.0);
  declare_parameter("roi_forward_max_m", 2.0);
  declare_parameter("roi_half_width_m", 0.9);
  declare_parameter("roi_min_height_m", 0.1);
  declare_parameter("roi_max_height_m", 1.5);
  declare_parameter("min_obstacle_points", 3);

  input_cloud_topic_           = get_parameter("input_cloud_topic").as_string();
  output_status_topic_         = get_parameter("output_status_topic").as_string();
  point_cloud_timeout_s_       = get_parameter("point_cloud_timeout_s").as_double();
  stop_hold_s_                 = get_parameter("stop_hold_s").as_double();
  roi_config_.forward_min_m    = get_parameter("roi_forward_min_m").as_double();
  roi_config_.forward_max_m    = get_parameter("roi_forward_max_m").as_double();
  roi_config_.half_width_m     = get_parameter("roi_half_width_m").as_double();
  roi_config_.min_height_m     = get_parameter("roi_min_height_m").as_double();
  roi_config_.max_height_m     = get_parameter("roi_max_height_m").as_double();
  const int64_t min_points_raw = get_parameter("min_obstacle_points").as_int();

  if (input_cloud_topic_.empty()) {
    input_cloud_topic_ = "/rslidar_points";
  }
  if (output_status_topic_.empty()) {
    output_status_topic_ = "/velocity_planner/status";
  }
  if (!std::isfinite(point_cloud_timeout_s_) || point_cloud_timeout_s_ <= 0.0) {
    SPDLOG_WARN("point_cloud_timeout_s={} 非法，回退为 0.5s", point_cloud_timeout_s_);
    point_cloud_timeout_s_ = 0.5;
  }
  stop_hold_s_             = std::max(stop_hold_s_, 0.0);
  roi_config_.half_width_m = std::max(roi_config_.half_width_m, 0.01);
  if (roi_config_.forward_max_m <= roi_config_.forward_min_m) {
    SPDLOG_WARN("ROI 前向范围无效，已将 forward_max_m 调整为"
                " forward_min_m + 0.1");
    roi_config_.forward_max_m = roi_config_.forward_min_m + 0.1;
  }
  if (roi_config_.max_height_m <= roi_config_.min_height_m) {
    SPDLOG_WARN("ROI 高度范围无效，已将 max_height_m 调整为"
                " min_height_m + 0.1");
    roi_config_.max_height_m = roi_config_.min_height_m + 0.1;
  }
  roi_config_.min_obstacle_points = static_cast<std::size_t>(std::max<int64_t>(min_points_raw, 1));

  // ---- 打印生效值 ----
  SPDLOG_INFO("========== 点云避停参数加载 ==========");
  SPDLOG_INFO("日志目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}, 级别: {}", log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "开启" : "关闭", log_level_);
  SPDLOG_INFO("点云话题: {}", input_cloud_topic_);
  SPDLOG_INFO("状态话题: {}", output_status_topic_);
  SPDLOG_INFO("状态发布频率: {:.1f} Hz (固定)", kPublishHz);
  SPDLOG_INFO("点云心跳超时: {:.2f}s", point_cloud_timeout_s_);
  SPDLOG_INFO("障碍消失后停车保持: {:.2f}s", stop_hold_s_);
  SPDLOG_INFO("ROI 前向: [{:.2f}, {:.2f}] m, 左右半宽: {:.2f} m", roi_config_.forward_min_m, roi_config_.forward_max_m, roi_config_.half_width_m);
  SPDLOG_INFO("ROI 高度: [{:.2f}, {:.2f}] m, 最少障碍点数: {}", roi_config_.min_height_m, roi_config_.max_height_m, roi_config_.min_obstacle_points);
  SPDLOG_INFO("=======================================");

  roi_filter_ = std::make_unique<RoiFilter>(roi_config_);
  stop_hold_  = std::make_unique<StopHoldFilter>(stop_hold_s_);
}

// 创建点云订阅、状态发布和固定频率定时器。
void VelocityPlannerNode::SetupInterfaces() {
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    CloudCallback(cloud);
  });

  status_pub_ = create_publisher<weeder_msgs::msg::ObstacleStatus>(output_status_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / kPublishHz));
  timer_            = create_wall_timer(period, [this]() {
    PublishStatus();
  });
}

// 保存最新点云，实际处理在固定频率定时器中进行。
void VelocityPlannerNode::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  pending_cloud_           = cloud;
  has_pending_cloud_       = true;
  last_cloud_receive_time_ = std::chrono::steady_clock::now();
}

// 发布固定频率的避停状态，
void VelocityPlannerNode::PublishStatus() {
  weeder_msgs::msg::ObstacleStatus status;
  const auto now = this->now();
  if (has_pending_cloud_ && pending_cloud_) {
    const RoiResult result     = roi_filter_->Evaluate(*pending_cloud_);
    const bool hold_stop       = stop_hold_->Update(result.stop_required, now.seconds());
    cached_sensor_valid_       = result.cloud_valid;
    cached_stop_required_      = !result.cloud_valid || result.stop_required || hold_stop;
    cached_obstacle_points_    = result.obstacle_points;
    cached_nearest_distance_m_ = result.nearest_distance_m;
    has_cloud_result_          = true;

    SPDLOG_INFO(
      "ROI 结果: cloud_valid={}, stop_required={}, obstacle_points={}, "
      "nearest_distance_m={:.3f}, hold_stop={}",
      result.cloud_valid,
      result.stop_required,
      result.obstacle_points,
      result.nearest_distance_m,
      hold_stop);

    pending_cloud_.reset();
    has_pending_cloud_ = false;
  }

  bool sensor_valid              = cached_sensor_valid_;
  bool stop_required             = cached_stop_required_;
  std::size_t obstacle_points    = cached_obstacle_points_;
  float nearest_distance_m       = cached_nearest_distance_m_;
  const auto point_cloud_age     = std::chrono::steady_clock::now() - last_cloud_receive_time_;
  const bool point_cloud_timeout = has_cloud_result_ && point_cloud_age >= std::chrono::duration<double>(point_cloud_timeout_s_);
  if (!has_cloud_result_ || point_cloud_timeout) {
    sensor_valid       = false;
    stop_required      = true;
    obstacle_points    = 0;
    nearest_distance_m = -1.0F;
    if (point_cloud_timeout) {
      SPDLOG_WARN("点云超过 {:.2f}s 未更新，输出停车请求", point_cloud_timeout_s_);
    } else {
      SPDLOG_WARN("尚未收到有效点云，输出停车请求");
    }
  }

  status.header.stamp       = now;
  status.header.frame_id    = cloud_frame_id_;
  status.sensor_valid       = sensor_valid;
  status.stop_required      = stop_required;
  status.obstacle_points    = static_cast<uint32_t>(obstacle_points);
  status.nearest_distance_m = nearest_distance_m;

  status_pub_->publish(status);
}

}   // namespace velocity_planner
