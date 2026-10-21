// Copyright 2026. 点云前向矩形避停状态节点。
#ifndef VELOCITY_PLANNER__VELOCITY_PLANNER_NODE_HPP_
#define VELOCITY_PLANNER__VELOCITY_PLANNER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <weeder_msgs/msg/obstacle_status.hpp>

#include "velocity_planner/obstacle_filter.hpp"

namespace velocity_planner
{

class VelocityPlannerNode : public rclcpp::Node
{
 public:
  explicit VelocityPlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

 private:
  void LoadParameters();
  void SetupInterfaces();
  void CloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void ProcessCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const rclcpp::Time & now);
  void PublishStatus();

  std::string input_cloud_topic_;      // 输入点云话题
  std::string output_status_topic_;    // 输出的避停状态话题
  std::string target_frame_;           // ROI 判定使用的目标坐标系
  double publish_hz_{10.0};            // 状态发布频率
  double cloud_timeout_s_{0.5};        // 点云处理结果最长有效期
  double tf_timeout_s_{0.1};           // TF 查询超时
  double stop_hold_s_{0.3};            // 障碍消失后的停车保持时间
  RoiConfig roi_config_;               // ROI 参数

  std::unique_ptr<RoiFilter> roi_filter_;          // ROI 过滤器
  std::unique_ptr<StopHoldFilter> stop_hold_;      // 停车保持过滤器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ObstacleStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;                                    // 保护点云和状态
  sensor_msgs::msg::PointCloud2::SharedPtr pending_cloud_;
  bool has_pending_cloud_{false};          // 是否存在待处理点云
  bool has_cloud_result_{false};           // 是否已经处理过点云
  bool cached_sensor_valid_{false};        // 最近点云结果是否有效
  bool cached_stop_required_{true};        // 最近点云是否要求停车
  std::size_t cached_obstacle_points_{0};  // 最近 ROI 障碍点数
  float cached_nearest_distance_m_{-1.0F};  // 最近障碍距离
  rclcpp::Time last_cloud_result_time_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;              // TF 查询缓存
  tf2_ros::TransformListener tf_listener_;  // TF 监听器
};

}  // namespace velocity_planner

#endif  // VELOCITY_PLANNER__VELOCITY_PLANNER_NODE_HPP_
