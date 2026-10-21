// Copyright 2026. 点云前向矩形避停状态节点。
#ifndef VELOCITY_PLANNER__VELOCITY_PLANNER_NODE_HPP_
#define VELOCITY_PLANNER__VELOCITY_PLANNER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <weeder_msgs/msg/obstacle_status.hpp>

#include "velocity_planner/obstacle_filter.hpp"

namespace velocity_planner {

class VelocityPlannerNode : public rclcpp::Node {
public:
  explicit VelocityPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void LoadLogParameters();
  void LoadParameters();
  void SetupInterfaces();
  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void PublishStatus();

  std::string input_cloud_topic_;     // 输入点云话题
  std::string output_status_topic_;   // 输出的避停状态话题
  double point_cloud_timeout_s_{0.5};   // 点云心跳超时 (s)
  double stop_hold_s_{0.3};           // 障碍消失后的停车保持时间
  RoiConfig roi_config_;              // ROI 参数

  std::string log_dir_{"~/.ros/logs"};   // spdlog 日志目录
  int log_file_size_mb_{50};             // 单个日志文件大小上限 (MB)
  int log_file_count_{10};               // 滚动保留的日志文件数量
  bool log_enable_console_{true};        // 是否同时输出到终端
  std::string log_level_{"info"};        // 日志级别

  std::unique_ptr<RoiFilter> roi_filter_;       // ROI 过滤器
  std::unique_ptr<StopHoldFilter> stop_hold_;   // 停车保持过滤器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ObstacleStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2::SharedPtr pending_cloud_;
  bool has_pending_cloud_{false};            // 是否存在待处理点云
  bool has_cloud_result_{false};             // 是否已经处理过点云
  bool cached_sensor_valid_{false};          // 最近点云结果是否有效
  bool cached_stop_required_{true};          // 最近点云是否要求停车
  std::size_t cached_obstacle_points_{0};    // 最近 ROI 障碍点数
  float cached_nearest_distance_m_{-1.0F};   // 最近障碍距离
  std::string cloud_frame_id_{"lidar"};   // 输出状态固定坐标系
  std::chrono::steady_clock::time_point
      last_cloud_receive_time_{};   // 最近一次收到点云的时刻
};

}   // namespace velocity_planner

#endif   // VELOCITY_PLANNER__VELOCITY_PLANNER_NODE_HPP_
