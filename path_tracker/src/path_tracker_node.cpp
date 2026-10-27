// 版权所有 2026。四轮转向 LQR/PID 的 ROS 2 节点实现。
#include "path_tracker/path_tracker_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "path_tracker/logging.hpp"

namespace path_tracker {

namespace {
constexpr double kRad2Deg = 57.29577951308232;

/// 日志中使用的节点名称。
constexpr const char* kNodeName = "path_tracker";

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}   // namespace

// 创建路径跟踪节点并初始化控制器与接口。
// 输入:
//   options : ROS 节点创建选项
PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options) : Node(kNodeName, options) {
  loadLogParameters();
  LogConfig log_config;
  log_config.logger_name         = "path_tracker";
  log_config.log_file_name       = "path_tracker.log";
  log_config.log_dir             = log_dir_;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb_) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count_);
  log_config.enable_console      = log_enable_console_;
  log_config.log_level           = log_level_;
  ConfigureLogging(log_config);
  SPDLOG_INFO("日志配置成功");

  // 必须先在 loadParameters() 之前创建 LQR 对象，否则配置时会访问空指针。
  lqr_ = std::make_unique<LQR>();
  loadParameters();

  setupInterfaces();

  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / control_hz_));
  timer_               = create_wall_timer(period_ns, [this]() {
    controlTick();
  });

  SPDLOG_INFO(
    "LQR 控制器已启动: 轴距={:.3f}m "
    "最大转角={:.1f}deg 频率={:.1f}Hz",
    wheelbase_,
    max_steer_deg_,
    control_hz_);
  SPDLOG_INFO(
    "订阅 位姿={} 路径={} 工作模式={} 避停状态={} 稻行={} | 发布 {} | "
    "避停门控={} 稻行PID={}",
    pose_topic_,
    path_topic_,
    work_mode_topic_,
    velocity_planner_status_topic_,
    crop_row_topic_,
    cmd_topic_,
    require_velocity_planner_ ? "开启" : "关闭",
    row_pid_enabled_ ? "开启" : "关闭");
}

// 读取日志配置参数。
// 输出:
//   日志目录、文件大小、文件数量和日志级别。
void PathTrackerNode::loadLogParameters() {
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 50);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(get_parameter("log_file_size_mb").as_int());
  log_file_count_     = static_cast<int>(get_parameter("log_file_count").as_int());
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

// 读取 LQR、速度策略和安全门控参数。
// 输出:
//   节点成员中的控制器配置和安全阈值。
void PathTrackerNode::loadParameters() {
  control_hz_                         = declare_parameter<double>("control_hz", 10.0);
  wheelbase_                          = declare_parameter<double>("wheelbase", 1.0);
  turn_mode_enter_m_                  = declare_parameter<double>("turn_mode_enter_m", 0.6);
  turn_mode_exit_m_                   = declare_parameter<double>("turn_mode_exit_m", 0.6);
  max_steer_deg_                      = declare_parameter<double>("max_steer_deg", 90.0);
  max_steer_rate_rad_s_               = declare_parameter<double>("max_steer_rate_deg_s", 90.0) / kRad2Deg;
  work_speed_                         = declare_parameter<double>("work_speed_m_s", 0.6);
  turn_speed_                         = declare_parameter<double>("turn_speed_m_s", 0.3);
  turn_speed_preview_m_               = declare_parameter<double>("turn_speed_preview_m", 1.0);
  turn_speed_hold_m_                  = declare_parameter<double>("turn_speed_hold_m", 0.5);
  speed_ramp_mps2_                    = declare_parameter<double>("speed_ramp_mps2", 0.5);
  max_lateral_error_m_                = declare_parameter<double>("max_lateral_error_m", 0.3);
  max_track_error_m_                  = declare_parameter<double>("max_track_error_m", 3.0);
  snap_forward_points_                = static_cast<size_t>(declare_parameter<int64_t>("snap_forward_points", 20));
  pose_timeout_s_                     = declare_parameter<double>("pose_timeout_s", 1.0);
  path_timeout_s_                     = declare_parameter<double>("path_timeout_s", 1.0);
  work_mode_timeout_s_                = declare_parameter<double>("work_mode_timeout_s", 1.0);
  velocity_planner_status_timeout_s_  = declare_parameter<double>("velocity_planner_status_timeout_s", 1.0);
  require_velocity_planner_           = declare_parameter<bool>("require_velocity_planner", true);
  frame_id_                           = declare_parameter<std::string>("frame_id", "odom");
  pose_topic_                         = declare_parameter<std::string>("pose_topic", "/gnss_driver/pose");
  path_topic_                         = declare_parameter<std::string>("path_topic", "/global_path_planner/path");
  work_mode_topic_                    = declare_parameter<std::string>("work_mode_topic", "/work_mode");
  velocity_planner_status_topic_      = declare_parameter<std::string>("velocity_planner_status_topic", "/velocity_planner/status");
  cmd_topic_                          = declare_parameter<std::string>("cmd_topic", "/chassis/cmd");
  viz_                                = declare_parameter<bool>("viz", true);
  row_pid_enabled_                    = declare_parameter<bool>("row_pid_enabled", false);
  crop_row_topic_                     = declare_parameter<std::string>("crop_row_topic", "/crop_row/observation");
  crop_row_frame_id_                  = declare_parameter<std::string>("crop_row_frame_id", "base_link");
  crop_row_timeout_s_                 = declare_parameter<double>("crop_row_timeout_s", 0.3);
  crop_row_min_confidence_            = declare_parameter<double>("crop_row_min_confidence", 0.5);
  row_pid_pre_turn_m_                 = declare_parameter<double>("row_pid_pre_turn_m", 3.0);
  row_pid_lookahead_m_                = declare_parameter<double>("row_pid_lookahead_m", 1.0);
  const int64_t row_pid_resume_frames = declare_parameter<int64_t>("row_pid_resume_frames", 3);
  row_pid_resume_frames_              = static_cast<std::size_t>(std::max<int64_t>(1, row_pid_resume_frames));
  row_pid_config_.kp_rad_per_m        = declare_parameter<double>("row_pid_kp_rad_per_m", 0.4);
  row_pid_config_.ki_rad_per_m_s      = declare_parameter<double>("row_pid_ki_rad_per_m_s", 0.02);
  row_pid_config_.kd_rad_s_per_m      = declare_parameter<double>("row_pid_kd_rad_s_per_m", 0.08);
  row_pid_config_.heading_gain        = declare_parameter<double>("row_pid_heading_gain", 0.3);
  row_pid_config_.derivative_tau_s    = declare_parameter<double>("row_pid_derivative_tau_s", 0.15);
  row_pid_config_.integral_limit_rad  = declare_parameter<double>("row_pid_integral_limit_rad", 0.1);

  if (!std::isfinite(max_steer_rate_rad_s_) || max_steer_rate_rad_s_ <= 0.0) {
    SPDLOG_WARN("max_steer_rate_deg_s={} 非法，回退为 90 deg/s", max_steer_rate_rad_s_ * kRad2Deg);
    max_steer_rate_rad_s_ = M_PI / 2.0;
  }
  if (
    row_pid_enabled_ &&
    (crop_row_topic_.empty() || crop_row_frame_id_.empty() || !std::isfinite(crop_row_timeout_s_) || crop_row_timeout_s_ <= 0.0 || !std::isfinite(crop_row_min_confidence_) || crop_row_min_confidence_ < 0.0 ||
     crop_row_min_confidence_ > 1.0 || !std::isfinite(row_pid_pre_turn_m_) || row_pid_pre_turn_m_ < turn_mode_enter_m_ || !std::isfinite(row_pid_lookahead_m_) || row_pid_lookahead_m_ < 0.0 ||
     row_pid_resume_frames < 1 || !std::isfinite(row_pid_config_.kp_rad_per_m) || !std::isfinite(row_pid_config_.ki_rad_per_m_s) || !std::isfinite(row_pid_config_.kd_rad_s_per_m) ||
     !std::isfinite(row_pid_config_.heading_gain) || !std::isfinite(row_pid_config_.derivative_tau_s) || row_pid_config_.derivative_tau_s <= 0.0 || !std::isfinite(row_pid_config_.integral_limit_rad) ||
     row_pid_config_.integral_limit_rad < 0.0)) {
    throw std::invalid_argument("invalid row PID tracking configuration");
  }
  row_pid_.Configure(row_pid_config_);

  const double lqr_eps              = declare_parameter<double>("lqr_eps", 1e-6);
  const int lqr_max_iteration       = static_cast<int>(declare_parameter<int64_t>("lqr_max_iteration", 1000));
  const double lqr_q_lateral        = declare_parameter<double>("lqr_Q_lateral", 50.0);
  const double lqr_q_heading        = declare_parameter<double>("lqr_Q_heading", 25.0);
  const double lqr_r_steer          = declare_parameter<double>("lqr_R_steer", 25.0);
  const bool pose_at_vehicle_center = declare_parameter<bool>("pose_at_vehicle_center", true);

  LQRConfig lqr_config;
  lqr_config.dt                     = 1.0 / control_hz_;
  lqr_config.eps                    = lqr_eps;
  lqr_config.max_iteration          = lqr_max_iteration;
  lqr_config.q_lateral              = lqr_q_lateral;
  lqr_config.q_heading              = lqr_q_heading;
  lqr_config.r_steer                = lqr_r_steer;
  lqr_config.wheelbase              = wheelbase_;
  lqr_config.turn_mode_enter_m      = turn_mode_enter_m_;
  lqr_config.turn_mode_exit_m       = turn_mode_exit_m_;
  lqr_config.max_steer_rad          = max_steer_deg_ / kRad2Deg;
  lqr_config.snap_forward_points    = snap_forward_points_;
  lqr_config.work_speed             = work_speed_;
  lqr_config.turn_speed             = turn_speed_;
  lqr_config.turn_speed_preview_m   = turn_speed_preview_m_;
  lqr_config.turn_speed_hold_m      = turn_speed_hold_m_;
  lqr_config.pose_at_vehicle_center = pose_at_vehicle_center;
  lqr_->Configure(lqr_config);

  SPDLOG_INFO(
    "控制参数加载完成: control_hz={:.1f}Hz wheelbase={:.3f}m "
    "max_steer={:.1f}° max_steer_rate={:.1f}°/s snap_forward_points={}",
    control_hz_,
    wheelbase_,
    max_steer_deg_,
    max_steer_rate_rad_s_ * kRad2Deg,
    snap_forward_points_);
  SPDLOG_INFO(
    "速度参数: work={:.3f}m/s turn={:.3f}m/s ramp={:.3f}m/s² "
    "preview={:.3f}m hold={:.3f}m enter={:.3f}m exit={:.3f}m",
    work_speed_,
    turn_speed_,
    speed_ramp_mps2_,
    turn_speed_preview_m_,
    turn_speed_hold_m_,
    turn_mode_enter_m_,
    turn_mode_exit_m_);
  SPDLOG_INFO(
    "安全参数: max_lateral_error={:.3f}m max_track_error={:.3f}m "
    "超时 pose={:.2f}s path={:.2f}s work_mode={:.2f}s "
    "obstacle={:.2f}s 避停门控={}",
    max_lateral_error_m_,
    max_track_error_m_,
    pose_timeout_s_,
    path_timeout_s_,
    work_mode_timeout_s_,
    velocity_planner_status_timeout_s_,
    require_velocity_planner_ ? "开启" : "关闭");
  SPDLOG_INFO(
    "话题参数: frame={} pose={} path={} work_mode={} obstacle={} cmd={} "
    "viz={}",
    frame_id_,
    pose_topic_,
    path_topic_,
    work_mode_topic_,
    velocity_planner_status_topic_,
    cmd_topic_,
    viz_);
  SPDLOG_INFO(
    "LQR 参数: dt={:.3f}s eps={:g} max_iter={} "
    "Q_lateral={:.1f} Q_heading={:.1f} R_steer={:.1f} "
    "pose_at_vehicle_center={}",
    lqr_config.dt,
    lqr_config.eps,
    lqr_config.max_iteration,
    lqr_config.q_lateral,
    lqr_config.q_heading,
    lqr_config.r_steer,
    pose_at_vehicle_center);
  SPDLOG_INFO(
    "稻行混合跟踪: enabled={} topic={} pre_turn={:.2f}m "
    "lookahead={:.2f}m resume_frames={} confidence={:.2f}",
    row_pid_enabled_,
    crop_row_topic_,
    row_pid_pre_turn_m_,
    row_pid_lookahead_m_,
    row_pid_resume_frames_,
    crop_row_min_confidence_);
}

// 创建位姿、路径、模式、避停状态订阅及控制输出接口。
// 副作用:
//   初始化 LQR 目标可视化、底盘指令发布和控制定时器。
void PathTrackerNode::setupInterfaces() {
  // gnss_driver 使用 SensorDataQoS（尽力传输，保留最近 5 条）发布位姿。
  // 可靠订阅端无法与该发布端匹配，因此这里使用相同的 QoS。
  pose_sub_ = create_subscription<weeder_msgs::msg::Pose>(pose_topic_, rclcpp::SensorDataQoS(), [this](const weeder_msgs::msg::Pose::SharedPtr msg) {
    poseCallback(msg);
  });

  // global_path_planner 使用 transient_local（瞬态本地持久化）发布路径
  // 晚加入的订阅端必须使用相同持久性才能收到最后一条路径。
  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_           = create_subscription<weeder_msgs::msg::Path>(path_topic_, path_qos, [this](const weeder_msgs::msg::Path::SharedPtr msg) {
    pathCallback(msg);
  });

  // work_mode 使用普通订阅即可，无需额外配置 QoS。
  work_mode_sub_ = create_subscription<weeder_msgs::msg::WorkMode>(work_mode_topic_, 10, [this](const weeder_msgs::msg::WorkMode::SharedPtr msg) {
    workModeCallback(msg);
  });

  // 障碍物数据
  const auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  obstacle_status_sub_    = create_subscription<weeder_msgs::msg::ObstacleStatus>(velocity_planner_status_topic_, obstacle_qos, [this](const weeder_msgs::msg::ObstacleStatus::SharedPtr msg) {
    ObstacleStatusCallback(msg);
  });

  if (row_pid_enabled_) {
    const auto crop_row_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    crop_row_sub_           = create_subscription<weeder_msgs::msg::CropRowObservation>(crop_row_topic_, crop_row_qos, [this](const weeder_msgs::msg::CropRowObservation::SharedPtr msg) {
      cropRowCallback(msg);
    });
  }

  // 发布控制指令
  cmd_pub_ = create_publisher<weeder_msgs::msg::ChassisCmd>(cmd_topic_, 10);
  if (viz_) {
    viz_pub_ = create_publisher<visualization_msgs::msg::Marker>("/path_tracker/target_point", 10);
  }

  const auto t               = now();
  last_pose_time_            = t;
  last_path_time_            = t;
  last_work_mode_time_       = t;
  last_obstacle_status_time_ = t;
}

// 接收车辆位姿并更新控制器状态。
// 输入:
//   msg : 定位位姿、速度、卫星数和有效性标志
// 输出:
//   robot_state_、位姿有效性和最近更新时间。
void PathTrackerNode::poseCallback(const weeder_msgs::msg::Pose::SharedPtr msg) {
  last_pose_time_       = now();
  has_pose_             = true;
  pose_valid_           = msg->valid;
  robot_state_.x        = msg->x;
  robot_state_.y        = msg->y;
  robot_state_.heading  = msg->yaw;
  robot_state_.linear_v = msg->speed_m_s;

  SPDLOG_INFO(
    "收到位姿: x={:.3f} y={:.3f} z={:.3f} yaw={:.2f}° "
    "speed={:.3f}m/s sats={} valid={}",
    msg->x,
    msg->y,
    msg->z,
    msg->yaw * kRad2Deg,
    msg->speed_m_s,
    msg->satellites,
    msg->valid);
}

// 接收并校验新路径，更新跟踪轨迹。
// 输入:
//   msg : x/y 点列和逐点作业段类型
// 输出:
//   LQR 轨迹、路径点数和最近路径时间。
// 失败:
//   x/y 长度不一致或点数不足时忽略该路径并输出停车。
void PathTrackerNode::pathCallback(const weeder_msgs::msg::Path::SharedPtr msg) {
  if (msg->x.size() < 3 || msg->x.size() != msg->y.size()) {
    SPDLOG_WARN("收到的路径无效：x={} y={}（需要 x==y 且不少于 3 点）", msg->x.size(), msg->y.size());
    return;
  }

  // 即使内容被忽略，也必须刷新路径心跳，避免规划器仍在运行时触发路径超时。
  last_path_time_ = now();
  if (!accept_path_message_) {
    return;
  }

  std::vector<std::pair<double, double>> xy;
  xy.reserve(msg->x.size());
  for (size_t i = 0; i < msg->x.size(); ++i) {
    xy.emplace_back(msg->x[i], msg->y[i]);
  }

  std::vector<TrajectoryPoint> traj;
  if (!ComputePathProfile(xy, &traj)) {
    SPDLOG_ERROR("路径剖面计算失败，忽略这条路径");
    return;
  }
  for (size_t i = 0; i < traj.size(); ++i) {
    traj[i].waypoint_type = msg->waypoint_types[i];
  }

  has_path_            = true;
  accept_path_message_ = false;
  path_point_count_    = traj.size();
  last_speed_cmd_      = 0.0;
  row_pid_.Reset();
  crop_row_valid_streak_ = 0;
  tracking_mode_         = TrackingMode::kLqrRow;
  lqr_->SetTrajectory(traj);

  SPDLOG_INFO(
    "已载入新路径: {} 个点, 起点 ({:.2f}, {:.2f}), "
    "终点 ({:.2f}, {:.2f})",
    traj.size(),
    traj.front().x,
    traj.front().y,
    traj.back().x,
    traj.back().y);
}

// 接收工作模式并更新自动控制使能状态。
// 输入:
//   msg : 当前工作模式和名称
// 输出:
//   autonomous_driving_mode_active_ 和最近模式时间。
void PathTrackerNode::workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg) {
  last_work_mode_time_            = now();
  const bool first_mode           = !ever_got_work_mode_;
  ever_got_work_mode_             = true;
  autonomous_driving_mode_active_ = (msg->mode == weeder_msgs::msg::WorkMode::MODE_UNMANNED);

  // 仅在模式变化时记录日志，避免 1 Hz 重发刷屏。
  // last_mode_ 同时用于变化检测；首条消息时其初值为 0。
  if (first_mode || msg->mode != last_mode_) {
    SPDLOG_INFO(
      "工作模式切换: {} -> {} ({}) -> {}",
      static_cast<unsigned>(last_mode_),
      static_cast<unsigned>(msg->mode),
      msg->mode_name,
      autonomous_driving_mode_active_ ? "自动驾驶(接管底盘)" : "非自动驾驶(输出停车)");
    last_mode_           = msg->mode;
    accept_path_message_ = true;
  }
}

// 接收点云避停状态并更新安全门控缓存。
// 输入:
//   msg : 感知有效性、停车要求和障碍统计
// 输出:
//   避停相关成员及最近状态时间。
void PathTrackerNode::ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg) {
  last_obstacle_status_time_ = now();
  has_obstacle_status_       = true;
  obstacle_sensor_valid_     = msg->sensor_valid;
  obstacle_stop_required_    = msg->stop_required;
  obstacle_points_           = msg->obstacle_points;

  if (obstacle_stop_required_) {
    SPDLOG_WARN(
      "收到避停状态: sensor_valid={} stop_required={} points={} "
      "nearest={:.2f}m",
      msg->sensor_valid,
      msg->stop_required,
      msg->obstacle_points,
      msg->nearest_distance_m);
  }
}

// 检查稻行观测是否满足控制输入的数据、时间、坐标系和数值约定。
// 返回:
//   true = 可作为视觉 PID 目标
bool PathTrackerNode::IsCropRowObservationUsable(const weeder_msgs::msg::CropRowObservation& observation, const rclcpp::Time& now) const {
  if (
    !observation.has_capture_time || observation.replay || observation.session_id.empty() || observation.sequence == 0 || observation.status != weeder_msgs::msg::CropRowObservation::VALID ||
    observation.header.frame_id != crop_row_frame_id_ || !std::isfinite(observation.lateral_error_m) || !std::isfinite(observation.heading_error_rad) || !std::isfinite(observation.lookahead_m) ||
    observation.lookahead_m <= 0.0 || !std::isfinite(observation.confidence) || observation.confidence < crop_row_min_confidence_ || observation.confidence > 1.0 || !std::isfinite(observation.processing_ms) ||
    observation.processing_ms < 0.0 || !std::isfinite(observation.frame_age_ms) || observation.frame_age_ms < 0.0) {
    return false;
  }

  const rclcpp::Time capture_time(observation.header.stamp);
  if (capture_time.nanoseconds() <= 0 || capture_time > now) {
    return false;
  }
  return (now - capture_time).seconds() <= crop_row_timeout_s_;
}

// 接收益苗行观测，按会话和新帧序号维护连续有效计数。
// 输入:
//   msg : 稻行中心线观测
void PathTrackerNode::cropRowCallback(const weeder_msgs::msg::CropRowObservation::SharedPtr msg) {
  if (msg->session_id != crop_row_session_) {
    crop_row_session_      = msg->session_id;
    crop_row_sequence_     = 0;
    crop_row_valid_streak_ = 0;
    latest_crop_row_.reset();
  }

  const rclcpp::Time now_time = now();
  const bool usable           = IsCropRowObservationUsable(*msg, now_time);
  if (msg->sequence < crop_row_sequence_) {
    latest_crop_row_.reset();
    crop_row_valid_streak_ = 0;
    return;
  }
  if (msg->sequence == crop_row_sequence_) {
    if (!usable) {
      latest_crop_row_.reset();
      crop_row_valid_streak_ = 0;
    }
    return;
  }

  crop_row_sequence_ = msg->sequence;
  if (usable) {
    latest_crop_row_ = msg;
    ++crop_row_valid_streak_;
  } else {
    latest_crop_row_.reset();
    crop_row_valid_streak_ = 0;
  }
}

// 将视觉中心线转换为 PID 使用的车体坐标前视误差。
RowPidTarget PathTrackerNode::MakeVisionPidTarget(const weeder_msgs::msg::CropRowObservation& observation) const {
  RowPidTarget target;
  target.lateral_error_m   = observation.lateral_error_m;
  target.heading_error_rad = observation.heading_error_rad;
  target.lookahead_m       = row_pid_lookahead_m_;
  return target;
}

// 将当前全局 WORK 目标点及其切向转换为车体坐标下的行内直线。
RowPidTarget PathTrackerNode::MakeGlobalPidTarget(const LQROutput& lqr_output) const {
  const TrajectoryPoint& target = lqr_output.target_pt;
  const double dx               = target.x - robot_state_.x;
  const double dy               = target.y - robot_state_.y;
  const double cos_yaw          = std::cos(robot_state_.heading);
  const double sin_yaw          = std::sin(robot_state_.heading);
  const double target_x_body    = cos_yaw * dx + sin_yaw * dy;
  const double target_y_body    = -sin_yaw * dx + cos_yaw * dy;
  const double heading_error    = NormalizeAngle(target.heading - robot_state_.heading);
  const double slope            = std::tan(heading_error);

  RowPidTarget pid_target;
  pid_target.lateral_error_m   = target_y_body - slope * target_x_body;
  pid_target.heading_error_rad = heading_error;
  pid_target.lookahead_m       = row_pid_lookahead_m_;
  return pid_target;
}

// 根据全局路径进度和视觉观测状态选择当前控制模式。
PathTrackerNode::TrackingMode PathTrackerNode::SelectTrackingMode(const LQROutput& lqr_output, const rclcpp::Time& now_time) const {
  if (lqr_output.steer_mode == 1) {
    return TrackingMode::kLqrTurn;
  }
  if (!row_pid_enabled_) {
    return TrackingMode::kLqrRow;
  }
  if (lqr_output.distance_to_turn_m <= row_pid_pre_turn_m_) {
    return TrackingMode::kPidGlobal;
  }
  if (latest_crop_row_ && crop_row_valid_streak_ >= row_pid_resume_frames_ && IsCropRowObservationUsable(*latest_crop_row_, now_time)) {
    return TrackingMode::kPidVision;
  }
  return TrackingMode::kLqrRow;
}

const char* PathTrackerNode::TrackingModeName(TrackingMode mode) {
  switch (mode) {
    case TrackingMode::kLqrRow:
      return "lqr_row";
    case TrackingMode::kPidVision:
      return "pid_vision";
    case TrackingMode::kPidGlobal:
      return "pid_global";
    case TrackingMode::kLqrTurn:
      return "lqr_turn";
  }
  return "unknown";
}

// 发布零速、零转角和未使能状态的停车指令。
// 副作用:
//   发布 /chassis/cmd，并清零速度和转角变化率限制状态。
void PathTrackerNode::publishStop() {
  weeder_msgs::msg::ChassisCmd zero;
  zero.gear            = 0;
  zero.steer_mode      = 0;
  zero.release_brake   = false;
  zero.estop           = false;
  zero.ctrl_mode       = false;
  zero.speed_m_s       = 0.0;
  zero.steer_angle_rad = 0.0;
  cmd_pub_->publish(zero);
  last_speed_cmd_     = 0.0;
  last_steer_cmd_rad_ = 0.0;
  row_pid_.Reset();
  crop_row_valid_streak_ = 0;
  tracking_mode_         = TrackingMode::kLqrRow;
}

// 执行一次安全门控、轨迹推进和 LQR 控制。
// 副作用:
//   发布底盘指令，并在任一安全门控失败时输出停车。
void PathTrackerNode::controlTick() {
  const auto t = now();

  // ---- 门控 1：工作模式，也就是系统级使能信号 ----
  const bool mode_fresh = ever_got_work_mode_ && (t - last_work_mode_time_).seconds() <= work_mode_timeout_s_;
  if (!mode_fresh || !autonomous_driving_mode_active_) {
    SPDLOG_WARN(
      "未使能：需在 {} 上以不低于 {:.2f} Hz 持续发布 MODE_UNMANNED，"
      "输出停车",
      work_mode_topic_,
      1.0 / work_mode_timeout_s_);
    publishStop();
    return;
  }

  // ---- 门控 2：RTK 位姿存在、新鲜且为固定解 ----
  if (!has_pose_) {
    SPDLOG_WARN("等待定位数据（{}），输出停车", pose_topic_);
    publishStop();
    return;
  }
  if ((t - last_pose_time_).seconds() > pose_timeout_s_) {
    SPDLOG_WARN("定位数据超时（{:.2f}s > {:.2f}s），输出停车", (t - last_pose_time_).seconds(), pose_timeout_s_);
    publishStop();
    return;
  }
  if (!pose_valid_) {
    SPDLOG_WARN("定位位姿无效（非 RTK 固定解），输出停车");
    publishStop();
    return;
  }

  // ---- 门控 3：存在可跟踪且未超时的路径 ----
  if (!has_path_) {
    SPDLOG_WARN("等待路径（{}），输出停车", path_topic_);
    publishStop();
    return;
  }
  if ((t - last_path_time_).seconds() > path_timeout_s_) {
    SPDLOG_WARN("路径超时（{:.2f}s > {:.2f}s），输出停车", (t - last_path_time_).seconds(), path_timeout_s_);
    publishStop();
    return;
  }

  // ---- 门控 4：点云避停安全门控 ----
  if (require_velocity_planner_) {
    const double obstacle_status_age_s = has_obstacle_status_ ? (t - last_obstacle_status_time_).seconds() : 0.0;
    if (!has_obstacle_status_ || obstacle_status_age_s > velocity_planner_status_timeout_s_) {
      SPDLOG_WARN(
        "避停状态不可用或超时（{:.2f}s > {:.2f}s），"
        "输出停车",
        obstacle_status_age_s,
        velocity_planner_status_timeout_s_);
      publishStop();
      return;
    }
    if (!obstacle_sensor_valid_) {
      SPDLOG_WARN("避停感知数据无效，输出停车");
      publishStop();
      return;
    }
    if (obstacle_stop_required_) {
      SPDLOG_WARN("ROI 内检测到 {} 个障碍点，输出停车", obstacle_points_);
      publishStop();
      return;
    }
  }

  // ---- 所有门控通过，开始运行控制器 ----

  LQRInput lqr_input;
  lqr_input.robot_state = robot_state_;
  LQROutput out;
  if (!lqr_->ComputeCommand(lqr_input, out)) {
    SPDLOG_INFO("已到达路径末端，输出停车");
    publishStop();
    return;
  }

  if (latest_crop_row_ && !IsCropRowObservationUsable(*latest_crop_row_, t)) {
    SPDLOG_WARN("稻行观测已过期或无效，退出视觉 PID");
    latest_crop_row_.reset();
    crop_row_valid_streak_ = 0;
  }
  const TrackingMode previous_mode = tracking_mode_;
  TrackingMode selected_mode       = SelectTrackingMode(out, t);
  if (selected_mode == TrackingMode::kLqrTurn && previous_mode != TrackingMode::kLqrTurn) {
    crop_row_valid_streak_ = 0;
    row_pid_.Reset();
  } else if (previous_mode == TrackingMode::kLqrTurn && selected_mode != TrackingMode::kLqrTurn) {
    crop_row_valid_streak_ = 0;
    row_pid_.Reset();
    selected_mode = TrackingMode::kLqrRow;
  } else if (selected_mode != previous_mode) {
    row_pid_.Reset();
  }
  tracking_mode_ = selected_mode;

  RowPidOutput pid_output;
  RowPidTarget pid_target;
  double target_delta           = out.vel_cmd.steer_angle;
  double lateral_error_for_gate = out.lateral_error;
  int steer_mode                = out.steer_mode;
  const double max_steer_rad    = max_steer_deg_ / kRad2Deg;
  if (tracking_mode_ == TrackingMode::kPidVision) {
    pid_target             = MakeVisionPidTarget(*latest_crop_row_);
    pid_output             = row_pid_.Compute(pid_target, 1.0 / control_hz_);
    target_delta           = std::clamp(pid_output.steer_rad, -max_steer_rad, max_steer_rad);
    lateral_error_for_gate = pid_target.lateral_error_m;
    steer_mode             = 0;
  } else if (tracking_mode_ == TrackingMode::kPidGlobal) {
    pid_target             = MakeGlobalPidTarget(out);
    pid_output             = row_pid_.Compute(pid_target, 1.0 / control_hz_);
    target_delta           = std::clamp(pid_output.steer_rad, -max_steer_rad, max_steer_rad);
    lateral_error_for_gate = pid_target.lateral_error_m;
    steer_mode             = 0;
  }
  if ((tracking_mode_ == TrackingMode::kPidVision || tracking_mode_ == TrackingMode::kPidGlobal) && !pid_output.valid) {
    SPDLOG_WARN("稻行 PID 输入无效，降级为单阿克曼 LQR");
    tracking_mode_ = TrackingMode::kLqrRow;
    row_pid_.Reset();
    target_delta           = out.vel_cmd.steer_angle;
    lateral_error_for_gate = out.lateral_error;
    steer_mode             = 0;
  }
  if (tracking_mode_ != previous_mode) {
    SPDLOG_INFO("跟踪源切换: {} -> {}", TrackingModeName(previous_mode), TrackingModeName(tracking_mode_));
  }

  // 超横差保护
  const double lateral_error_abs = std::fabs(lateral_error_for_gate);
  if (lateral_error_abs > max_lateral_error_m_) {
    SPDLOG_ERROR("横向误差 {:.3f} m > {:.3f} m，输出停车", lateral_error_abs, max_lateral_error_m_);
    publishStop();
    return;
  }

  // ---- 门控 5：车辆必须仍在跟踪目标点附近 ----
  //
  // 该门控用于弥补 LQR 误差状态 [e_lateral, e_heading] 不含沿路径方向误差的问题。
  // 车辆沿路径方向冲过目标点时，横向和航向误差都为 0，控制器会误判为完美跟踪，
  // 从而持续输出全速零转角。
  //
  // 2026-10-15 仿真中曾出现目标点冻结在路径起点、车辆驶出 238 米，
  // 转角为 2.4e-14 且没有任何告警的情况，所有误差指标仍显示正常。
  //
  // 本检查放在 ComputeCommand() 之后，使前向吸附先完成重新锚定；
  // 只有真正丢失跟踪时才会触发停车。
  const double dist_to_goal = out.distance_to_goal;
  if (dist_to_goal > max_track_error_m_) {
    SPDLOG_ERROR(
      "偏离目标 {:.2f} m > {:.2f} m"
      "（横向/航向误差可能同时为 0，LQR 无沿路径误差项），"
      "输出停车。idx={}/{}",
      dist_to_goal,
      max_track_error_m_,
      out.target_index,
      path_point_count_);
    publishStop();
    return;
  }

  // 速度平滑
  const double target_v       = out.vel_cmd.linear_v;
  const double max_speed_step = speed_ramp_mps2_ / control_hz_;
  double v                    = last_speed_cmd_;
  if (target_v > v) {
    v = std::min(target_v, v + max_speed_step);
  } else {
    v = std::max(target_v, v - max_speed_step);
  }
  last_speed_cmd_ = v;

  const double max_steer_step = max_steer_rate_rad_s_ / control_hz_;
  const double delta          = std::clamp(target_delta, last_steer_cmd_rad_ - max_steer_step, last_steer_cmd_rad_ + max_steer_step);
  last_steer_cmd_rad_         = delta;

  // 速度与转角在同一帧发布，保证 CAN 桥使用的是同一控制周期内的运动学组合。
  // 转角单位为 rad、左正，与 ADCU_toVCU_01 的符号约定一致。
  weeder_msgs::msg::ChassisCmd cmd;
  cmd.gear            = 1;
  cmd.steer_mode      = static_cast<uint8_t>(steer_mode);
  cmd.release_brake   = true;
  cmd.estop           = false;
  cmd.ctrl_mode       = true;
  cmd.speed_m_s       = v;
  cmd.steer_angle_rad = delta;
  cmd_pub_->publish(cmd);

  double vision_age_ms     = -1.0;
  double vision_confidence = 0.0;
  if (latest_crop_row_) {
    vision_age_ms     = (t - rclcpp::Time(latest_crop_row_->header.stamp)).seconds() * 1000.0;
    vision_confidence = latest_crop_row_->confidence;
  }
  SPDLOG_DEBUG(
    "mode={} steer_mode={} type={} turn_distance={:.2f}m "
    "v={:.3f}/{:.3f} m/s delta={:+.2f}/{:+.2f} deg "
    "(fb={:+.2f} ff={:+.2f}) e_lat={:+.3f} m "
    "e_head={:+.3f} rad pid={:+.2f}/{:+.2f}/{:+.2f}/{:+.2f} "
    "vision_age={:.1f}ms vision_conf={:.2f} d_goal={:.2f} m idx={}/{}",
    TrackingModeName(tracking_mode_),
    steer_mode,
    static_cast<unsigned>(out.target_pt.waypoint_type),
    out.distance_to_turn_m,
    v,
    target_v,
    delta * kRad2Deg,
    target_delta * kRad2Deg,
    out.steer_fb * kRad2Deg,
    out.steer_ff * kRad2Deg,
    out.lateral_error,
    out.heading_error,
    pid_output.p_term_rad * kRad2Deg,
    pid_output.i_term_rad * kRad2Deg,
    pid_output.d_term_rad * kRad2Deg,
    pid_output.heading_term_rad * kRad2Deg,
    vision_age_ms,
    vision_confidence,
    dist_to_goal,
    out.target_index,
    path_point_count_);

  if (viz_) {
    publishViz(out.target_pt);
  }
}

// 发布当前跟踪目标点的 RViz Marker。
// 输入:
//   target : 当前目标轨迹点
void PathTrackerNode::publishViz(const TrajectoryPoint& target) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id    = frame_id_;
  m.header.stamp       = now();
  m.ns                 = "lqr_target";
  m.id                 = 0;
  m.type               = visualization_msgs::msg::Marker::SPHERE;
  m.action             = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x    = target.x;
  m.pose.position.y    = target.y;
  m.pose.position.z    = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x            = 0.25;
  m.scale.y            = 0.25;
  m.scale.z            = 0.25;
  m.color.r            = 1.0f;
  m.color.g            = 0.5f;
  m.color.b            = 0.0f;
  m.color.a            = 1.0f;
  viz_pub_->publish(m);
}

}   // namespace path_tracker
