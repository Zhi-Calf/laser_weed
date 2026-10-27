#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <weeder_msgs/msg/chassis_cmd.hpp>
#include <weeder_msgs/msg/crop_row_observation.hpp>
#include <weeder_msgs/msg/obstacle_status.hpp>
#include <weeder_msgs/msg/path.hpp>
#include <weeder_msgs/msg/pose.hpp>
#include <weeder_msgs/msg/work_mode.hpp>

#include "path_tracker/discrete_points_math.hpp"
#include "path_tracker/lqr.hpp"
#include "path_tracker/row_pid.hpp"

namespace path_tracker {

// 基于离散 LQR 与稻行 PID 的四轮转向路径跟踪节点。
class PathTrackerNode : public rclcpp::Node {
public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void loadLogParameters();
  void loadParameters();
  void setupInterfaces();
  void controlTick();

private:
  void poseCallback(const weeder_msgs::msg::Pose::SharedPtr msg);
  void pathCallback(const weeder_msgs::msg::Path::SharedPtr msg);
  void workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);
  void ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg);
  void cropRowCallback(const weeder_msgs::msg::CropRowObservation::SharedPtr msg);

private:
  enum class TrackingMode {
    kLqrRow,
    kPidVision,
    kPidGlobal,
    kLqrTurn,
  };

  TrackingMode SelectTrackingMode(const LQROutput& lqr_output, const rclcpp::Time& now) const;
  RowPidTarget MakeVisionPidTarget(const weeder_msgs::msg::CropRowObservation& observation) const;
  RowPidTarget MakeGlobalPidTarget(const LQROutput& lqr_output) const;
  bool IsCropRowObservationUsable(const weeder_msgs::msg::CropRowObservation& observation, const rclcpp::Time& now) const;
  static const char* TrackingModeName(TrackingMode mode);

private:
  void publishViz(const TrajectoryPoint& target);
  void publishStop();

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_    = 50;                 // 单个日志文件大小上限
  int log_file_count_      = 10;                 // 保留日志文件数量
  bool log_enable_console_ = true;               // 是否输出到控制台
  std::string log_level_   = "info";             // 日志级别

private:
  double control_hz_           = 10.0;         // 控制频率 (Hz)
  double wheelbase_            = 1.0;          // 物理轴距 (m)
  double max_steer_deg_        = 90.0;         // 最大转角 (deg)
  double max_steer_rate_rad_s_ = M_PI / 2.0;   // 最大转角变化率 (rad/s)
  double work_speed_           = 0.6;          // WORK 段速度 (m/s)
  double turn_speed_           = 0.3;          // TURN 段速度 (m/s)
  double turn_speed_preview_m_ = 1.0;          // TURN 前减速距离 (m)
  double turn_speed_hold_m_    = 0.5;          // TURN 后低速保持距离 (m)
  double speed_ramp_mps2_      = 0.5;          // 速度变化率上限 (m/s²)
  double turn_mode_enter_m_    = 0.6;          // 进入掉头模式距离 (m)
  double turn_mode_exit_m_     = 0.6;          // 退出掉头模式距离 (m)

private:
  double max_lateral_error_m_               = 0.3;    // 最大横向误差 (m)
  double max_track_error_m_                 = 3.0;    // 最大目标点距离 (m)
  std::size_t snap_forward_points_          = 20;     // 目标点前向搜索点数
  double pose_timeout_s_                    = 1.0;    // 位姿超时 (s)
  double path_timeout_s_                    = 1.0;    // 路径超时 (s)
  double work_mode_timeout_s_               = 1.0;    // 工作模式超时 (s)
  double velocity_planner_status_timeout_s_ = 1.0;    // 避停状态超时 (s)
  bool require_velocity_planner_            = true;   // 是否启用避停门控

private:
  std::string frame_id_                      = "odom";
  std::string pose_topic_                    = "/gnss_driver/pose";
  std::string path_topic_                    = "/global_path_planner/path";
  std::string work_mode_topic_               = "/work_mode";
  std::string velocity_planner_status_topic_ = "/velocity_planner/status";
  std::string cmd_topic_                     = "/chassis/cmd";
  bool viz_                                  = true;

private:
  bool row_pid_enabled_              = false;
  std::string crop_row_topic_        = "/crop_row/observation";
  std::string crop_row_frame_id_     = "base_link";
  double crop_row_timeout_s_         = 0.3;
  double crop_row_min_confidence_    = 0.5;
  double row_pid_pre_turn_m_         = 3.0;
  double row_pid_lookahead_m_        = 1.0;
  std::size_t row_pid_resume_frames_ = 3;
  RowPidConfig row_pid_config_;
  RowPidController row_pid_;

private:
  std::unique_ptr<LQR> lqr_;

private:
  RobotState robot_state_       = {};
  bool has_pose_                = false;
  bool pose_valid_              = false;
  rclcpp::Time last_pose_time_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_path_time_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_path_                = false;
  std::size_t path_point_count_ = 0;
  bool accept_path_message_     = true;   // 是否允许接收下一条有效路径

private:
  rclcpp::Time last_work_mode_time_    = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool ever_got_work_mode_             = false;
  bool autonomous_driving_mode_active_ = false;   // 自动驾驶模式是否处于激活状态
  std::uint8_t last_mode_              = 0;

private:
  rclcpp::Time last_obstacle_status_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_obstacle_status_               = false;
  bool obstacle_sensor_valid_             = false;
  bool obstacle_stop_required_            = true;
  std::uint32_t obstacle_points_          = 0;

private:
  weeder_msgs::msg::CropRowObservation::SharedPtr latest_crop_row_;
  std::string crop_row_session_;
  std::uint64_t crop_row_sequence_   = 0;
  std::size_t crop_row_valid_streak_ = 0;
  bool vision_reacquire_required_    = true;
  TrackingMode tracking_mode_        = TrackingMode::kLqrRow;

private:
  double last_speed_cmd_     = 0.0;   // 上一控制周期速度指令 (m/s)
  double last_steer_cmd_rad_ = 0.0;   // 上一控制周期转角指令 (rad)

private:
  rclcpp::Subscription<weeder_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<weeder_msgs::msg::WorkMode>::SharedPtr work_mode_sub_;
  rclcpp::Subscription<weeder_msgs::msg::ObstacleStatus>::SharedPtr obstacle_status_sub_;
  rclcpp::Subscription<weeder_msgs::msg::CropRowObservation>::SharedPtr crop_row_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

private:
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

}   // namespace path_tracker
