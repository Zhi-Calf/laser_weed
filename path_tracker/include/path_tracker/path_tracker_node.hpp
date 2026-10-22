// 版权所有 2026。封装四轮转向 LQR 控制器的 ROS 2 节点。
//
// 替代早期 Python Stanley 控制器。节点订阅 gnss_localization 的 RTK 位姿和
// global_path_planner 的全覆盖路径，并向 CAN 桥（weeder_base chassis_node）
// 发布同一控制周期内的速度和前轴等效转角。
//
// 运动使能使用系统级 /work_mode 信号，而不是控制器私有话题。
#pragma once

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <weeder_msgs/msg/chassis_cmd.hpp>
#include <weeder_msgs/msg/obstacle_status.hpp>
#include <weeder_msgs/msg/path.hpp>
#include <weeder_msgs/msg/pose.hpp>
#include <weeder_msgs/msg/work_mode.hpp>

#include "path_tracker/lqr.hpp"
#include "path_tracker/obstacle_gate.hpp"
#include "path_tracker/path_profile.hpp"

namespace path_tracker {

class PathTrackerNode : public rclcpp::Node {
public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void loadLogParameters();
  void loadParameters();
  void setupInterfaces();
  void controlTick();

  void poseCallback(const weeder_msgs::msg::Pose::SharedPtr msg);
  void pathCallback(const weeder_msgs::msg::Path::SharedPtr msg);
  void workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);
  void ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg);

  /// 将跟踪目标点发布为 RViz Marker。
  void publishViz(const TrajectoryPoint& target);

  /// 输出零速和零转角。CAN 桥自身还有指令超时停车保护，这里构成双重保护。
  void publishStop();

  bool ApplySteerMode(int mode, double effective_wheelbase);

  // 参数
  std::string log_dir_{"~/.ros/logs"};   // 日志目录
  int log_file_size_mb_{50};             // 单个日志文件大小上限
  int log_file_count_{10};               // 保留日志文件数量
  bool log_enable_console_{true};        // 是否输出到控制台
  std::string log_level_{"info"};        // 日志级别
  double control_hz_{10.0};
  double wheelbase_{1.0};
  int steer_mode_{0};
  double max_steer_deg_{90.0};
  double work_speed_{0.6};
  double turn_speed_{0.3};
  double turn_speed_preview_m_{1.0};
  double turn_speed_hold_m_{0.5};
  double speed_ramp_mps2_{0.5};
  double turn_mode_enter_m_{0.6};
  double turn_mode_exit_m_{0.6};
  double max_lateral_error_m_{0.3};   // 允许的最大横向误差（米）
  // 安全门限：车辆与跟踪目标点的最大允许距离。
  // LQR 误差状态 [e_lateral, e_heading] 不含沿路径方向的分量，
  // 车辆沿路径方向冲过目标点时会表现为完美跟踪。
  // 该门限应大于地头转弯时的合法间距，同时远小于车辆驶出田块的距离。
  double max_track_error_m_{3.0};
  // 目标点前向吸附窗口，单位为路径点数。
  // 按规划器默认 0.2 米分辨率，25 个点等于 5 米路径。
  size_t snap_forward_points_{20};
  double pose_timeout_s_{1.0};
  double path_timeout_s_{5.0};
  double work_mode_timeout_s_{3.0};
  double work_mode_startup_grace_s_{20.0};
  double velocity_planner_status_timeout_s_{0.5};
  bool require_unmanned_{true};
  bool require_velocity_planner_{true};
  std::string frame_id_{"odom"};
  std::string pose_topic_{"/gnss_driver/pose"};
  std::string path_topic_{"/global_path_planner/path"};
  std::string work_mode_topic_{"/work_mode"};
  std::string velocity_planner_status_topic_{"/velocity_planner/status"};
  std::string cmd_topic_{"/chassis/cmd"};
  bool viz_{true};

  // 核心控制器
  std::unique_ptr<LQR> lqr_;
  // 订阅和发布接口
  rclcpp::Subscription<weeder_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<weeder_msgs::msg::WorkMode>::SharedPtr work_mode_sub_;
  rclcpp::Subscription<weeder_msgs::msg::ObstacleStatus>::SharedPtr obstacle_status_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 运行时状态
  RobotState robot_state_;
  bool has_pose_{false};
  bool pose_valid_{false};
  rclcpp::Time boot_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_work_mode_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacle_status_time_{0, 0, RCL_ROS_TIME};
  bool ever_got_work_mode_{false};
  bool has_obstacle_status_{false};
  bool obstacle_sensor_valid_{false};
  bool obstacle_stop_required_{true};
  uint32_t obstacle_points_{0};
  bool unmanned_{false};
  uint8_t last_mode_{0};
  bool has_path_{false};
  size_t path_point_count_{0};
  bool accept_path_message_{true};   // 是否允许接收下一条有效路径
  double last_speed_cmd_{0.0};            // 上一控制周期的速度指令，用于加速度限制
};

}   // namespace path_tracker
