#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <weeder_msgs/msg/chassis_cmd.hpp>
#include <weeder_msgs/msg/crop_row_observation.hpp>
#include <weeder_msgs/msg/obstacle_status.hpp>
#include <weeder_msgs/msg/path.hpp>
#include <weeder_msgs/msg/pose.hpp>
#include <weeder_msgs/msg/work_mode.hpp>

#include "path_tracker/global_path_locator.hpp"
#include "path_tracker/lqr.hpp"

namespace path_tracker {

// 四轮转向底盘的统一 LQR 路径跟踪节点。
class PathTrackerNode : public rclcpp::Node {
public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum class ReferenceSource {
    kGlobalPath,   // 全局规划路径
    kVisionLine,   // 视觉稻行中心线
  };

  // 读取日志目录和滚动策略。
  void LoadLogParameters();
  // 读取 LQR、视觉参考和安全门控参数。
  void LoadParameters();
  // 创建订阅、发布和控制定时器。
  void SetupInterfaces();
  // 执行固定周期安全门控、路径投影和控制输出。
  void ControlTick();

  // 更新车辆位姿。
  void PoseCallback(const weeder_msgs::msg::Pose::SharedPtr msg);
  // 校验并载入新的全局路径。
  void PathCallback(const weeder_msgs::msg::Path::SharedPtr msg);
  // 更新系统级自动驾驶使能状态。
  void WorkModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);
  // 更新避停状态缓存。
  void ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg);
  // 更新视觉观测和连续有效计数。
  void CropRowCallback(const weeder_msgs::msg::CropRowObservation::SharedPtr msg);

  // 根据 TURN 距离和视觉状态选择参考源。
  ReferenceSource SelectReferenceSource(const PathMatchResult& match, const rclcpp::Time& now) const;
  // 把视觉消息转换为统一 LQR 直线参考。
  LineReference MakeVisionLineReference(const weeder_msgs::msg::CropRowObservation& observation) const;
  // 检查视觉观测的数据、时间和坐标契约。
  bool IsCropRowObservationUsable(const weeder_msgs::msg::CropRowObservation& observation, const rclcpp::Time& now) const;
  // 返回参考源日志名称。
  static const char* ReferenceSourceName(ReferenceSource source);

  // 发布全局路径投影点。
  void PublishViz(const TrajectoryPoint& target);
  // 发布急停、零速、零转角指令。
  void PublishStop();

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_    = 50;                 // 单文件大小上限 (MB)
  int log_file_count_      = 10;                 // 日志文件保留数量
  bool log_enable_console_ = true;               // 是否输出到控制台
  std::string log_level_   = "info";             // 日志级别

private:
  double control_hz_           = 10.0;         // 控制频率 (Hz)
  double wheelbase_            = 1.0;          // 物理轴距 (m)
  double max_steer_deg_        = 90.0;         // 最大转角 (deg)
  double max_steer_rate_rad_s_ = M_PI / 2.0;   // 最大转角变化率 (rad/s)
  double work_speed_           = 0.6;          // WORK 段速度 (m/s)
  double turn_speed_           = 0.3;          // TURN 段速度 (m/s)
  double speed_ramp_mps2_      = 0.5;          // 速度变化率上限 (m/s²)
  double turn_mode_enter_m_    = 0.6;          // 进入双阿克曼距离 (m)
  double turn_mode_exit_m_     = 0.6;          // 退出双阿克曼距离 (m)

private:
  double max_lateral_error_m_               = 0.3;    // 最大横向误差 (m)
  double max_track_error_m_                 = 3.0;    // 最大投影距离 (m)
  int forward_point_window_                 = 8;      // 最近点前向搜索点数偏移
  double goal_reached_distance_m_           = 1.0;    // 路径终点到达阈值 (m)
  double pose_timeout_s_                    = 1.0;    // 位姿超时 (s)
  double path_timeout_s_                    = 1.0;    // 路径超时 (s)
  double work_mode_timeout_s_               = 1.0;    // 工作模式超时 (s)
  double velocity_planner_status_timeout_s_ = 1.0;    // 避停状态超时 (s)
  bool require_velocity_planner_            = true;   // 是否启用避停门控

private:
  std::string frame_id_                      = "odom";                        // 全局路径坐标系
  std::string pose_topic_                    = "/gnss_driver/pose";           // 定位话题
  std::string path_topic_                    = "/global_path_planner/path";   // 全局路径
  std::string work_mode_topic_               = "/work_mode";                  // 工作模式
  std::string velocity_planner_status_topic_ = "/velocity_planner/status";    // 避停状态
  std::string cmd_topic_                     = "/chassis/cmd";                // 底盘指令
  bool viz_                                  = true;                          // 是否发布可视化

private:
  bool row_vision_enabled_              = false;                     // 是否启用视觉中心线
  std::string crop_row_topic_           = "/crop_row/observation";   // 稻行观测
  std::string crop_row_frame_id_        = "base_link";               // 观测车体参考系
  double crop_row_timeout_s_            = 0.3;                       // 观测超时 (s)
  double crop_row_min_confidence_       = 0.5;                       // 最低置信度
  std::size_t row_vision_resume_frames_ = 3;                         // 视觉恢复帧数

private:
  std::unique_ptr<LQR> lqr_;         // 统一路径控制器
  GlobalPathLocator path_locator_;   // 全局路径最近点搜索

private:
  RobotState robot_state_;                 // 当前车辆状态
  bool has_pose_                = false;   // 是否收到过位姿
  bool pose_valid_              = false;   // 位姿是否可用于控制
  rclcpp::Time last_pose_time_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_path_time_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_path_                = false;   // 是否已载入路径
  int path_point_count_ = 0;               // 已载入路径点数
  bool accept_path_message_     = true;    // 是否接收下一条路径

private:
  rclcpp::Time last_work_mode_time_    = rclcpp::Time(0, 0, RCL_ROS_TIME);   // 最近工作模式时间
  bool ever_got_work_mode_             = false;                              // 是否收到过工作模式
  bool autonomous_driving_mode_active_ = false;                              // 自动驾驶是否激活
  std::uint8_t last_mode_              = 0;                                  // 上一工作模式值

private:
  rclcpp::Time last_obstacle_status_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_obstacle_status_               = false;   // 是否收到避停状态
  bool obstacle_sensor_valid_             = false;   // 避停传感器是否有效
  bool obstacle_stop_required_            = true;    // 是否要求立即停车
  std::uint32_t obstacle_points_          = 0;       // ROI 内障碍点数

private:
  weeder_msgs::msg::CropRowObservation::SharedPtr latest_crop_row_;   // 最新视觉观测
  std::string crop_row_session_;                                      // 视觉发布会话标识
  std::uint64_t crop_row_sequence_   = 0;                             // 会话内最新帧序号
  std::size_t crop_row_valid_streak_ = 0;                             // 连续可用视觉帧数
  bool vision_acquire_paused_        = false;                         // TURN 前后是否暂停视觉恢复计数
  ReferenceSource reference_source_  = ReferenceSource::kGlobalPath;

private:
  double last_speed_cmd_     = 0.0;   // 上一拍速度指令 (m/s)
  double last_steer_cmd_rad_ = 0.0;   // 上一拍转角指令 (rad)

private:
  rclcpp::Subscription<weeder_msgs::msg::Pose>::SharedPtr pose_sub_;                        // 定位订阅
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_;                        // 路径订阅
  rclcpp::Subscription<weeder_msgs::msg::WorkMode>::SharedPtr work_mode_sub_;               // 工作模式订阅
  rclcpp::Subscription<weeder_msgs::msg::ObstacleStatus>::SharedPtr obstacle_status_sub_;   // 避停状态订阅
  rclcpp::Subscription<weeder_msgs::msg::CropRowObservation>::SharedPtr crop_row_sub_;      // 视觉稻行订阅
  rclcpp::Publisher<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_pub_;                      // 底盘指令发布
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;                   // LQR 目标可视化发布

private:
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;   // 固定频率控制定时器
};

}   // namespace path_tracker
