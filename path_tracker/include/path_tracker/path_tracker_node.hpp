#pragma once

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

#include "path_tracker/global_path_locator.hpp"
#include "path_tracker/lqr.hpp"
#include "path_tracker/lqr_input.hpp"

namespace path_tracker {

// 根据全局路径或稻行视觉中心线生成底盘 LQR 控制指令。
class PathTrackerNode : public rclcpp::Node {
public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum class ReferenceSource {
    kGlobalPath,
    kVisionLine,
  };

  void InitializeLogging();
  void LoadParameters();
  void LoadNodeParameters();
  void InitializePathTracking(const std::vector<TrajectoryPoint>& path);
  LQRConfig LoadLQRConfig(double wheelbase, bool pose_at_vehicle_center);
  GlobalPathLocatorConfig LoadGlobalPathLocatorConfig(double wheelbase, bool pose_at_vehicle_center);
  void SetupInterfaces();
  void ControlTick();

  void PoseCallback(const weeder_msgs::msg::Pose::SharedPtr msg);
  void PathCallback(const weeder_msgs::msg::Path::SharedPtr msg);
  void WorkModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);
  void ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg);
  void CropRowCallback(const weeder_msgs::msg::CropRowObservation::SharedPtr msg);

  ReferenceSource SelectReferenceSource(const PathMatchResult& match, const rclcpp::Time& now) const;
  static const char* ReferenceSourceName(ReferenceSource source);

  void PublishViz(const TrajectoryPoint& target);
  void PublishStop();
  bool CheckLateralError(double lateral_error);

private:
  double control_hz_           = 10.0;         // 控制频率 (Hz)
  double max_steer_rate_rad_s_ = M_PI / 2.0;   // 最大转角变化率 (rad/s)
  double speed_ramp_mps2_      = 0.5;          // 速度变化率上限 (m/s²)

private:
  double max_lateral_error_m_               = 0.3;   // 超限后停车
  double max_track_error_m_                 = 1.0;   // 超限后停车
  double pose_timeout_s_                    = 1.0;
  double path_timeout_s_                    = 1.0;
  double work_mode_timeout_s_               = 1.0;
  double velocity_planner_status_timeout_s_ = 1.0;
  bool require_velocity_planner_            = true;

private:
  std::string frame_id_ = "odom";
  bool viz_             = true;

private:
  bool row_vision_enabled_   = false;
  double crop_row_timeout_s_ = 0.3;

private:
  LQRInputCalculator lqr_input_calculator_;   // LQR 输入计算器
  LQR lqr_;
  GlobalPathLocator path_locator_;

private:
  RobotState robot_state_;
  bool has_pose_               = false;
  bool pose_valid_             = false;   // 位姿通过时效和固定解校验
  rclcpp::Time last_pose_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_path_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_path_               = false;
  int path_point_count_        = 0;
  weeder_msgs::msg::Path::SharedPtr last_loaded_path_;  // 最后成功加载的路径

private:
  rclcpp::Time last_work_mode_time_    = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool ever_got_work_mode_             = false;   // 区分尚未收到与超时
  bool autonomous_driving_mode_active_ = false;
  std::uint8_t last_mode_              = 0;

private:
  rclcpp::Time last_obstacle_status_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_obstacle_status_               = false;
  bool obstacle_sensor_valid_             = false;
  bool obstacle_stop_required_            = true;
  std::uint32_t obstacle_points_          = 0;   // ROI 内障碍点数

private:
  LineReference line_reference_;
  std::string last_crop_row_session_id_;          // 最后受理的视觉会话标识
  std::uint64_t last_crop_row_sequence_    = 0;   // 最后受理的有效帧序号
  rclcpp::Time last_crop_row_capture_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool line_reference_valid_               = false;
  ReferenceSource reference_source_        = ReferenceSource::kGlobalPath;

private:
  double last_speed_cmd_m_s_ = 0.0;   // 上一拍速度指令 (m/s)
  double last_steer_cmd_rad_ = 0.0;   // 用于转角速率限制 (rad)

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
