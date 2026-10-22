// 版权所有 2026。封装四轮转向 LQR 控制器的 ROS 2 节点。
//
// 替代早期 Python Stanley 控制器。节点订阅 gnss_localization 的 RTK 位姿和
// global_path_planner 的全覆盖路径，并向 CAN 桥（weeder_base chassis_node）
// 发布同一控制周期内的速度和前轴等效转角。
//
// 运动使能使用系统级 /work_mode 信号，而不是控制器私有话题。
#ifndef PATH_TRACKER__PATH_TRACKER_NODE_HPP_
#define PATH_TRACKER__PATH_TRACKER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <weeder_msgs/msg/chassis_cmd.hpp>
#include <weeder_msgs/msg/obstacle_status.hpp>
#include <weeder_msgs/msg/path.hpp>
#include <weeder_msgs/msg/pose.hpp>
#include <weeder_msgs/msg/work_mode.hpp>

#include "path_tracker/fault_codes.hpp"
#include "path_tracker/lqr.hpp"
#include "path_tracker/obstacle_gate.hpp"
#include "path_tracker/path_profile.hpp"

namespace path_tracker {

class PathTrackerNode : public rclcpp::Node {
public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
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

  void ApplySteerMode(int mode);

  // 参数
  double control_hz_{10.0};
  double wheelbase_{1.0};
  double effective_wheelbase_{1.0};   // 按转向模式调整后的等效轴距（L 或 L/2）
  int steer_mode_{0};
  int fallback_steer_mode_{0};
  int work_steer_mode_{0};
  int turn_steer_mode_{1};
  double max_steer_deg_{90.0};
  double max_speed_{1.4};
  double target_v_{0.5};
  double work_speed_{0.6};
  double turn_speed_{0.3};
  double turn_speed_preview_m_{1.0};
  double turn_speed_hold_m_{0.5};
  double speed_ramp_mps2_{0.5};
  double turn_mode_enter_m_{0.6};
  double turn_mode_exit_m_{0.6};
  bool enable_segment_speed_profile_{true};
  bool enable_turn_steer_mode_switch_{true};
  bool lqr_speed_shaping_enabled_{false};
  // 安全门限：车辆与跟踪目标点的最大允许距离。
  // LQR 误差状态 [e_lateral, e_heading] 不含沿路径方向的分量，
  // 车辆沿路径方向冲过目标点时会表现为完美跟踪（见 fault::kTrackErrorLost）。
  // 该门限应大于地头转弯时的合法间距，同时远小于车辆驶出田块的距离。
  double max_track_error_m_{3.0};
  // 目标点前向吸附窗口，单位为路径点数。
  // 按规划器默认 0.2 米分辨率，25 个点等于 5 米路径。
  size_t snap_window_{25};
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
  mutable std::mutex mutex_;
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
  // 当前路径指纹，由点数、首尾坐标和路径类型哈希组成。
  // global_path_planner 每 100 毫秒重发同一条路径，回调必须忽略重复消息；
  // 否则目标索引每次都会重置为 0，车辆无法沿路径推进。
  size_t path_fp_count_{0};
  double path_fp_x0_{0.0};
  double path_fp_y0_{0.0};
  double path_fp_xn_{0.0};
  double path_fp_yn_{0.0};
  uint64_t path_fp_type_hash_{0};  // 路径类型指纹，避免同几何不同分段时漏更新
  double last_x_{0.0};
  double last_y_{0.0};
  double last_speed_cmd_{0.0};  // 上一控制周期的速度指令，用于加速度限制
  bool path_has_waypoint_types_{false};  // 当前路径是否带合法的分段类型
  bool turn_mode_active_{false};  // 是否处于带滞回的掉头模式区
  bool first_tick_{true};
};

}   // 命名空间 path_tracker

#endif   // PATH_TRACKER__PATH_TRACKER_NODE_HPP_
