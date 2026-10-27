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

#include "path_tracker/discrete_points_math.hpp"
#include "path_tracker/lqr.hpp"

namespace path_tracker {

class PathTrackerNode : public rclcpp::Node {
public:
  // 创建路径跟踪节点并初始化控制器与接口。
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // 读取日志配置参数。
  void loadLogParameters();
  // 读取 LQR 与安全门控参数。
  void loadParameters();
  // 创建订阅者、发布者和定时器。
  void setupInterfaces();
  // 执行一次控制周期和安全门控。
  void controlTick();

  // 接收车辆位姿并更新控制器状态。
  void poseCallback(const weeder_msgs::msg::Pose::SharedPtr msg);
  // 接收并校验新路径。
  void pathCallback(const weeder_msgs::msg::Path::SharedPtr msg);
  // 接收工作模式并更新使能状态。
  void workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);
  // 接收点云避停状态并更新门控状态。
  void ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg);

  // 将跟踪目标点发布为 RViz Marker。
  void publishViz(const TrajectoryPoint& target);

  // 发布零速、零转角的停车指令。
  void publishStop();

  // 参数
  std::string log_dir_         = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_        = 50;                 // 单个日志文件大小上限
  int log_file_count_          = 10;                 // 保留日志文件数量
  bool log_enable_console_     = true;               // 是否输出到控制台
  std::string log_level_       = "info";             // 日志级别
  double control_hz_           = 10.0;
  double wheelbase_            = 1.0;
  double max_steer_deg_        = 90.0;
  double work_speed_           = 0.6;
  double turn_speed_           = 0.3;
  double turn_speed_preview_m_ = 1.0;
  double turn_speed_hold_m_    = 0.5;
  double speed_ramp_mps2_      = 0.5;
  double turn_mode_enter_m_    = 0.6;
  double turn_mode_exit_m_     = 0.6;
  double max_lateral_error_m_  = 0.3;   // 允许的最大横向误差（米）
  // 安全门限：车辆与跟踪目标点的最大允许距离。
  // LQR 误差状态 [e_lateral, e_heading] 不含沿路径方向的分量，
  // 车辆沿路径方向冲过目标点时会表现为完美跟踪。
  // 该门限应大于地头转弯时的合法间距，同时远小于车辆驶出田块的距离。
  double max_track_error_m_ = 3.0;
  // 目标点前向吸附窗口，单位为路径点数。
  // 按规划器默认 0.2 米分辨率，25 个点等于 5 米路径。
  size_t snap_forward_points_                = 20;
  double pose_timeout_s_                     = 1.0;
  double path_timeout_s_                     = 1.0;
  double work_mode_timeout_s_                = 1.0;
  double velocity_planner_status_timeout_s_  = 1.0;
  bool require_velocity_planner_             = true;
  std::string frame_id_                      = "odom";
  std::string pose_topic_                    = "/gnss_driver/pose";
  std::string path_topic_                    = "/global_path_planner/path";
  std::string work_mode_topic_               = "/work_mode";
  std::string velocity_planner_status_topic_ = "/velocity_planner/status";
  std::string cmd_topic_                     = "/chassis/cmd";
  bool viz_                                  = true;

  // 核心控制器
  std::unique_ptr<LQR> lqr_ = nullptr;
  // 订阅和发布接口
  rclcpp::Subscription<weeder_msgs::msg::Pose>::SharedPtr pose_sub_                      = nullptr;
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_                      = nullptr;
  rclcpp::Subscription<weeder_msgs::msg::WorkMode>::SharedPtr work_mode_sub_             = nullptr;
  rclcpp::Subscription<weeder_msgs::msg::ObstacleStatus>::SharedPtr obstacle_status_sub_ = nullptr;
  rclcpp::Publisher<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_pub_                    = nullptr;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_                 = nullptr;
  rclcpp::TimerBase::SharedPtr timer_                                                    = nullptr;

  // 运行时状态
  RobotState robot_state_                 = {};
  bool has_pose_                          = false;
  bool pose_valid_                        = false;
  rclcpp::Time last_pose_time_            = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_path_time_            = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_work_mode_time_       = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_obstacle_status_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool ever_got_work_mode_                = false;
  bool has_obstacle_status_               = false;
  bool obstacle_sensor_valid_             = false;
  bool obstacle_stop_required_            = true;
  uint32_t obstacle_points_               = 0;
  bool autonomous_driving_mode_active_    = false;   // 自动驾驶模式是否处于激活状态
  uint8_t last_mode_                      = 0;
  bool has_path_                          = false;
  size_t path_point_count_                = 0;
  bool accept_path_message_               = true;   // 是否允许接收下一条有效路径
  double last_speed_cmd_                  = 0.0;    // 上一控制周期速度指令，用于加速度限制
};

}   // namespace path_tracker
