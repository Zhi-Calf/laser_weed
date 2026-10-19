// Copyright 2026. ROS 2 node wrapping the four-wheel-steering LQR controller.
//
// Replaces the earlier Python Stanley controller. Subscribes to the RTK odometry from
// gnss_localization and the coverage path from global_path_planner, and emits a
// speed + front-wheel steering angle pair for the CAN bridge (weeder_base
// chassis_node).
//
// Motion is gated by the system-wide /work_mode signal rather than a
// controller-private arm topic.
#ifndef LQR_CONTROL__LQR_NODE_HPP_
#define LQR_CONTROL__LQR_NODE_HPP_

#include "lqr_control/fault_codes.hpp"
#include "lqr_control/lqr.hpp"
#include "lqr_control/path_profile.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <weeder_msgs/msg/chassis_cmd.hpp>
#include <weeder_msgs/msg/path.hpp>
#include <weeder_msgs/msg/work_mode.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lqr_control
{

class LqrNode : public rclcpp::Node
{
public:
  LqrNode();

private:
  void loadParameters();
  void setupInterfaces();
  void controlTick();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCallback(const weeder_msgs::msg::Path::SharedPtr msg);
  void workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);

  /// Publish the tracked goal point as an RViz marker.
  void publishViz(const TrajectoryPoint & target);

  /// RTK solution quality gate, derived from the odometry pose covariance that
  /// gnss_driver fills from fix_type (fixed / float / RTD / SPP).
  bool rtkQualityOk(double & worst_sigma) const;

  /// Command zero speed and zero steering. The CAN bridge also stops on its own
  /// cmd timeout, so this is belt-and-braces.
  void publishStop();

  // Parameters
  double control_hz_{10.0};
  double wheelbase_{1.0};
  double effective_wheelbase_{1.0};  ///< after steer_mode adjustment (L, or L/2)
  int steer_mode_{0};
  double max_steer_deg_{90.0};
  double max_speed_{1.4};
  double target_v_{0.5};
  double rtk_max_sigma_{0.05};
  /// Safety gate: max distance from the robot to its tracked goal before motion
  /// is inhibited. Required because the LQR error state [e_lateral, e_heading]
  /// has NO along-track component, so a robot running straight past the goal
  /// reads as perfect tracking (see fault::kTrackErrorLost). Set larger than any
  /// legitimate goal-to-robot gap on a headland turn, and well under any
  /// "left the field" distance.
  double max_track_error_m_{3.0};
  /// Forward-snap window for goal re-anchoring, in path points. At the planner's
  /// default 0.2 m resolution this is 25 points = 5 m of path.
  size_t snap_window_{25};
  double odom_timeout_s_{1.0};
  double path_timeout_s_{5.0};
  double work_mode_timeout_s_{3.0};
  double work_mode_startup_grace_s_{20.0};
  bool require_unmanned_{true};
  std::string frame_id_{"odom"};
  std::string odom_topic_{"/gnss_driver/odom"};
  std::string path_topic_{"/global_path_planner/path"};
  std::string work_mode_topic_{"/work_mode"};
  std::string cmd_topic_{"/chassis/cmd"};
  bool viz_{true};

  // Core
  std::unique_ptr<LQR> lqr_;
  // Subscriptions / publications
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<weeder_msgs::msg::WorkMode>::SharedPtr work_mode_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  mutable std::mutex mutex_;
  RobotState robot_state_;
  bool has_odom_{false};
  rclcpp::Time boot_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_work_mode_time_{0, 0, RCL_ROS_TIME};
  bool ever_got_work_mode_{false};
  bool unmanned_{false};
  uint8_t last_mode_{0};
  bool has_path_{false};
  size_t path_point_count_{0};
  /// Fingerprint of the currently loaded path (point count + first/last xy).
  /// global_path_planner republishes the same path every 100 ms, so the callback
  /// must ignore repeats; without this the tracked goal is reset to index 0 on
  /// every republish and the vehicle never advances along the path.
  size_t path_fp_count_{0};
  double path_fp_x0_{0.0};
  double path_fp_y0_{0.0};
  double path_fp_xn_{0.0};
  double path_fp_yn_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  bool first_tick_{true};
  double odom_sigma_{0.0};   // worst of the two horizontal pose sigmas
};

}  // namespace lqr_control

#endif  // LQR_CONTROL__LQR_NODE_HPP_
