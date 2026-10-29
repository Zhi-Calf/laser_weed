// 版权所有 2026。四轮转向统一 LQR 的 ROS 2 节点实现。
#include "path_tracker/path_tracker_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "path_tracker/logging.hpp"
#include "path_tracker/speed_limiter.hpp"

namespace path_tracker {

namespace {
constexpr double kRad2Deg = 57.29577951308232;

/// 日志中使用的节点名称。
constexpr const char* kNodeName = "path_tracker";

// 节点对外接口话题固定定义，不通过 YAML 配置。
constexpr const char* kPoseTopic                  = "/gnss_driver/pose";
constexpr const char* kPathTopic                  = "/global_path_planner/path";
constexpr const char* kWorkModeTopic              = "/work_mode";
constexpr const char* kVelocityPlannerStatusTopic = "/velocity_planner/status";
constexpr const char* kCmdTopic                   = "/chassis/cmd";
constexpr const char* kCropRowTopic               = "/crop_row/observation";
constexpr const char* kVizTopic                   = "/path_tracker/target_point";

}   // namespace

// 创建路径跟踪节点并初始化控制器与接口。
// 输入:
//   options : ROS 节点创建选项
PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options) : Node(kNodeName, options) {
  InitializeLogging();
  LoadParameters();
  SetupInterfaces();

  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / control_hz_));
  timer_               = create_wall_timer(period_ns, [this]() {
    ControlTick();
  });

  SPDLOG_INFO("路径跟踪节点已启动");
  SPDLOG_INFO(
    "订阅 位姿={} 路径={} 工作模式={} 避停状态={} 稻行={} | 发布 {} | "
    "避停门控={} 视觉LQR={}",
    kPoseTopic,
    kPathTopic,
    kWorkModeTopic,
    kVelocityPlannerStatusTopic,
    kCropRowTopic,
    kCmdTopic,
    require_velocity_planner_ ? "开启" : "关闭",
    row_vision_enabled_ ? "开启" : "关闭");
}

// 读取日志参数并初始化 spdlog。
// 副作用:
//   日志配置仅在本函数内构造，之后由日志模块持有。
void PathTrackerNode::InitializeLogging() {
  const std::string log_dir      = declare_parameter<std::string>("log_dir", "/opt/agri/logs");
  const int64_t log_file_size_mb = declare_parameter<int64_t>("log_file_size_mb", 50);
  const int64_t log_file_count   = declare_parameter<int64_t>("log_file_count", 10);
  const bool log_enable_console  = declare_parameter<bool>("log_enable_console", true);
  const std::string log_level    = declare_parameter<std::string>("log_level", "info");

  LogConfig log_config;
  log_config.logger_name         = "path_tracker";
  log_config.log_file_name       = "path_tracker.log";
  log_config.log_dir             = log_dir;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count);
  log_config.enable_console      = log_enable_console;
  log_config.log_level           = log_level;
  ConfigureLogging(log_config);
  SPDLOG_INFO("日志配置成功");
}

// 按参数使用者加载节点、LQR 和 GlobalPathLocator 配置。
// 副作用:
//   初始化节点运行状态，并配置 LQR 和全局路径定位器。
void PathTrackerNode::LoadParameters() {
  const double wheelbase            = declare_parameter<double>("wheelbase", 1.0);
  const bool pose_at_vehicle_center = declare_parameter<bool>("pose_at_vehicle_center", true);
  LoadNodeParameters();
  lqr_.Configure(LoadLQRConfig(wheelbase, pose_at_vehicle_center));
  path_locator_.Configure(LoadGlobalPathLocatorConfig(wheelbase, pose_at_vehicle_center));
}

// 读取节点调度、输出限幅、安全门控和视觉参数。
// 输出:
//   PathTrackerNode 的运行时配置成员。
void PathTrackerNode::LoadNodeParameters() {
  control_hz_                        = declare_parameter<double>("control_hz", 10.0);
  max_steer_rate_rad_s_              = declare_parameter<double>("max_steer_rate_deg_s", 90.0) / kRad2Deg;
  speed_ramp_mps2_                   = declare_parameter<double>("speed_ramp_mps2", 0.5);
  max_lateral_error_m_               = declare_parameter<double>("max_lateral_error_m", 0.3);
  max_track_error_m_                 = declare_parameter<double>("max_track_error_m", 3.0);
  pose_timeout_s_                    = declare_parameter<double>("pose_timeout_s", 1.0);
  path_timeout_s_                    = declare_parameter<double>("path_timeout_s", 1.0);
  work_mode_timeout_s_               = declare_parameter<double>("work_mode_timeout_s", 1.0);
  velocity_planner_status_timeout_s_ = declare_parameter<double>("velocity_planner_status_timeout_s", 1.0);
  require_velocity_planner_          = declare_parameter<bool>("require_velocity_planner", true);
  frame_id_                          = declare_parameter<std::string>("frame_id", "odom");
  viz_                               = declare_parameter<bool>("viz", true);
  row_vision_enabled_                = declare_parameter<bool>("row_vision_enabled", false);
  crop_row_timeout_s_                = declare_parameter<double>("crop_row_timeout_s", 0.3);

  if (!std::isfinite(max_steer_rate_rad_s_) || max_steer_rate_rad_s_ <= 0.0) {
    SPDLOG_WARN("max_steer_rate_deg_s={} 非法，回退为 90 deg/s", max_steer_rate_rad_s_ * kRad2Deg);
    max_steer_rate_rad_s_ = M_PI / 2.0;
  }
  SPDLOG_INFO(
    "节点控制参数: control_hz={:.1f}Hz max_steer_rate={:.1f}°/s "
    "speed_ramp={:.3f}m/s²",
    control_hz_,
    max_steer_rate_rad_s_ * kRad2Deg,
    speed_ramp_mps2_);
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
  SPDLOG_INFO("视觉稻行 LQR: enabled={} topic={} timeout={:.2f}s frame={} viz={}", row_vision_enabled_, kCropRowTopic, crop_row_timeout_s_, frame_id_, viz_);
}

// 读取 LQR 权重、目标速度和转角上限。
// 输入:
//   wheelbase              : 物理轴距 (m)
//   pose_at_vehicle_center : 位姿是否位于车体几何中心
// 返回:
//   尚未应用的 LQR 完整配置。
LQRConfig PathTrackerNode::LoadLQRConfig(double wheelbase, bool pose_at_vehicle_center) {
  const double max_steer_deg  = declare_parameter<double>("max_steer_deg", 90.0);
  const double work_speed     = declare_parameter<double>("work_speed_m_s", 0.6);
  const double turn_speed     = declare_parameter<double>("turn_speed_m_s", 0.3);
  const double lqr_eps        = declare_parameter<double>("lqr_eps", 1e-6);
  const int lqr_max_iteration = static_cast<int>(declare_parameter<int64_t>("lqr_max_iteration", 1000));
  const double lqr_q_lateral  = declare_parameter<double>("lqr_Q_lateral", 50.0);
  const double lqr_q_heading  = declare_parameter<double>("lqr_Q_heading", 25.0);
  const double lqr_r_steer    = declare_parameter<double>("lqr_R_steer", 25.0);

  LQRConfig lqr_config;
  lqr_config.dt                     = 1.0 / control_hz_;
  lqr_config.eps                    = lqr_eps;
  lqr_config.max_iteration          = lqr_max_iteration;
  lqr_config.q_lateral              = lqr_q_lateral;
  lqr_config.q_heading              = lqr_q_heading;
  lqr_config.r_steer                = lqr_r_steer;
  lqr_config.wheelbase              = wheelbase;
  lqr_config.max_steer_rad          = max_steer_deg / kRad2Deg;
  lqr_config.work_speed             = work_speed;
  lqr_config.turn_speed             = turn_speed;
  lqr_config.pose_at_vehicle_center = pose_at_vehicle_center;
  return lqr_config;
}

// 读取 GlobalPathLocator 专属参数，并应用共享几何配置。
// 输入:
//   wheelbase              : 物理轴距 (m)
//   pose_at_vehicle_center : 位姿是否位于车体几何中心
// 返回:
//   尚未应用的全局路径定位器完整配置。
GlobalPathLocatorConfig PathTrackerNode::LoadGlobalPathLocatorConfig(double wheelbase, bool pose_at_vehicle_center) {
  const int forward_point_window       = static_cast<int>(declare_parameter<int64_t>("forward_point_window", 8));
  const double turn_mode_enter_m       = declare_parameter<double>("turn_mode_enter_m", 0.6);
  const double turn_mode_exit_m        = declare_parameter<double>("turn_mode_exit_m", 0.6);
  const double goal_reached_distance_m = declare_parameter<double>("goal_reached_distance_m", 1.0);

  GlobalPathLocatorConfig path_locator_config;
  path_locator_config.forward_point_window    = forward_point_window;
  path_locator_config.turn_mode_enter_m       = turn_mode_enter_m;
  path_locator_config.turn_mode_exit_m        = turn_mode_exit_m;
  path_locator_config.wheelbase_m             = wheelbase;
  path_locator_config.goal_reached_distance_m = goal_reached_distance_m;
  path_locator_config.pose_at_vehicle_center  = pose_at_vehicle_center;
  return path_locator_config;
}

// 创建位姿、路径、模式、避停状态订阅及控制输出接口。
// 副作用:
//   初始化 LQR 目标可视化、底盘指令发布和控制定时器。
void PathTrackerNode::SetupInterfaces() {
  // gnss_driver 使用 SensorDataQoS（尽力传输，保留最近 5 条）发布位姿。
  // 可靠订阅端无法与该发布端匹配，因此这里使用相同的 QoS。
  pose_sub_ = create_subscription<weeder_msgs::msg::Pose>(kPoseTopic, rclcpp::SensorDataQoS(), [this](const weeder_msgs::msg::Pose::SharedPtr msg) {
    PoseCallback(msg);
  });

  // global_path_planner 使用 transient_local（瞬态本地持久化）发布路径
  // 晚加入的订阅端必须使用相同持久性才能收到最后一条路径。
  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_           = create_subscription<weeder_msgs::msg::Path>(kPathTopic, path_qos, [this](const weeder_msgs::msg::Path::SharedPtr msg) {
    PathCallback(msg);
  });

  // work_mode 使用普通订阅即可，无需额外配置 QoS。
  work_mode_sub_ = create_subscription<weeder_msgs::msg::WorkMode>(kWorkModeTopic, 10, [this](const weeder_msgs::msg::WorkMode::SharedPtr msg) {
    WorkModeCallback(msg);
  });

  // 障碍物数据
  const auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  obstacle_status_sub_    = create_subscription<weeder_msgs::msg::ObstacleStatus>(kVelocityPlannerStatusTopic, obstacle_qos, [this](const weeder_msgs::msg::ObstacleStatus::SharedPtr msg) {
    ObstacleStatusCallback(msg);
  });

  if (row_vision_enabled_) {
    const auto crop_row_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    crop_row_sub_           = create_subscription<weeder_msgs::msg::CropRowObservation>(kCropRowTopic, crop_row_qos, [this](const weeder_msgs::msg::CropRowObservation::SharedPtr msg) {
      CropRowCallback(msg);
    });
  }

  // 发布控制指令
  cmd_pub_ = create_publisher<weeder_msgs::msg::ChassisCmd>(kCmdTopic, 10);
  if (viz_) {
    viz_pub_ = create_publisher<visualization_msgs::msg::Marker>(kVizTopic, 10);
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
void PathTrackerNode::PoseCallback(const weeder_msgs::msg::Pose::SharedPtr msg) {
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
//   x/y/waypoint_types 长度不一致或点数不足时忽略该路径并输出停车。
void PathTrackerNode::PathCallback(const weeder_msgs::msg::Path::SharedPtr msg) {
  if (msg->x.size() < 3 || msg->x.size() != msg->y.size() || msg->x.size() != msg->waypoint_types.size()) {
    SPDLOG_WARN(
      "收到的路径无效：x={} y={} waypoint_types={}"
      "（需要三者等长且不少于 3 点）",
      msg->x.size(),
      msg->y.size(),
      msg->waypoint_types.size());
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
  path_point_count_    = static_cast<int>(traj.size());
  last_speed_cmd_m_s_  = 0.0;
  last_steer_cmd_rad_  = 0.0;
  reference_source_    = ReferenceSource::kGlobalPath;
  path_locator_.SetPath(traj);
  lqr_.Reset();

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
void PathTrackerNode::WorkModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg) {
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

// 缓存最新的有效稻行参考线。
// 输入:
//   msg : 稻行中心线观测
void PathTrackerNode::CropRowCallback(const weeder_msgs::msg::CropRowObservation::SharedPtr msg) {
  if (msg->sequence == crop_row_sequence_) {
    return;
  }
  crop_row_sequence_ = msg->sequence;

  const rclcpp::Time now_time = now();
  const rclcpp::Time capture_time(msg->header.stamp);
  const bool invalid_capture_time   = capture_time.nanoseconds() <= 0;
  const bool capture_time_in_future = capture_time > now_time;
  const bool observation_timeout    = (now_time - capture_time).seconds() > crop_row_timeout_s_;
  if (msg->status != weeder_msgs::msg::CropRowObservation::VALID || invalid_capture_time || capture_time_in_future || observation_timeout) {
    line_reference_valid_ = false;
    return;
  }

  line_reference_.lateral_error_m   = msg->lateral_error_m;
  line_reference_.heading_error_rad = msg->heading_error_rad;
  line_reference_.curvature         = 0.0;
  last_crop_row_time_               = now_time;
  line_reference_valid_             = true;
}

// 根据全局路径 TURN 距离和视觉观测状态选择参考源。
// 输入:
//   match    : 当前全局路径最近点
//   now_time : 当前 ROS 时间
// 返回:
//   视觉在 TURN 前后预切换区内始终退出；参考线必须有效且未超时
PathTrackerNode::ReferenceSource PathTrackerNode::SelectReferenceSource(const PathMatchResult& match, const rclcpp::Time& now_time) const {
  if (!row_vision_enabled_) {
    return ReferenceSource::kGlobalPath;
  }
  const bool near_turn = match.work_mode == PathWorkMode::kTurn;
  if (near_turn) {
    return ReferenceSource::kGlobalPath;
  }
  const bool reference_fresh = line_reference_valid_ && (now_time - last_crop_row_time_).seconds() <= crop_row_timeout_s_;
  if (reference_fresh) {
    return ReferenceSource::kVisionLine;
  }
  return ReferenceSource::kGlobalPath;
}

// 返回参考源日志名称。
const char* PathTrackerNode::ReferenceSourceName(ReferenceSource source) {
  switch (source) {
    case ReferenceSource::kGlobalPath:
      return "global_path";
    case ReferenceSource::kVisionLine:
      return "vision_line";
  }
  return "unknown";
}

// 发布零速、零转角和未使能状态的停车指令。
// 副作用:
//   发布 /chassis/cmd，并清零速度斜坡和转角速率状态。
void PathTrackerNode::PublishStop() {
  weeder_msgs::msg::ChassisCmd zero;
  zero.gear            = 0;
  zero.steer_mode      = 0;
  zero.release_brake   = false;
  zero.estop           = true;
  zero.ctrl_mode       = false;
  zero.speed_m_s       = 0.0;
  zero.steer_angle_rad = 0.0;
  cmd_pub_->publish(zero);
  lqr_.Reset();
  last_speed_cmd_m_s_ = 0.0;
  last_steer_cmd_rad_ = 0.0;
  reference_source_   = ReferenceSource::kGlobalPath;
}

// 执行一次安全门控、轨迹推进和 LQR 控制。
// 副作用:
//   发布底盘指令，并在任一安全门控失败时输出停车。
void PathTrackerNode::ControlTick() {
  const auto t = now();

  // ---- 门控 1：工作模式，也就是系统级使能信号 ----
  const bool mode_fresh = ever_got_work_mode_ && (t - last_work_mode_time_).seconds() <= work_mode_timeout_s_;
  if (!mode_fresh || !autonomous_driving_mode_active_) {
    SPDLOG_WARN(
      "未使能：需在 {} 上以不低于 {:.2f} Hz 持续发布 MODE_UNMANNED，"
      "输出停车",
      kWorkModeTopic,
      1.0 / work_mode_timeout_s_);
    PublishStop();
    return;
  }

  // ---- 门控 2：RTK 位姿存在、新鲜且为固定解 ----
  if (!has_pose_) {
    SPDLOG_WARN("等待定位数据（{}），输出停车", kPoseTopic);
    PublishStop();
    return;
  }
  if ((t - last_pose_time_).seconds() > pose_timeout_s_) {
    SPDLOG_WARN("定位数据超时（{:.2f}s > {:.2f}s），输出停车", (t - last_pose_time_).seconds(), pose_timeout_s_);
    PublishStop();
    return;
  }
  if (!pose_valid_) {
    SPDLOG_WARN("定位位姿无效（非 RTK 固定解），输出停车");
    PublishStop();
    return;
  }

  // ---- 门控 3：存在可跟踪且未超时的路径 ----
  if (!has_path_) {
    SPDLOG_WARN("等待路径（{}），输出停车", kPathTopic);
    PublishStop();
    return;
  }
  if ((t - last_path_time_).seconds() > path_timeout_s_) {
    SPDLOG_WARN("路径超时（{:.2f}s > {:.2f}s），输出停车", (t - last_path_time_).seconds(), path_timeout_s_);
    PublishStop();
    return;
  }
  if (path_point_count_ < 2 || path_locator_.path_point_count() != path_point_count_) {
    SPDLOG_ERROR("全局路径未有效载入：记录点数={} 定位器点数={}，输出停车", path_point_count_, path_locator_.path_point_count());
    PublishStop();
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
      PublishStop();
      return;
    }
    if (!obstacle_sensor_valid_) {
      SPDLOG_WARN("避停感知数据无效，输出停车");
      PublishStop();
      return;
    }
    if (obstacle_stop_required_) {
      SPDLOG_WARN("ROI 内检测到 {} 个障碍点，输出停车", obstacle_points_);
      PublishStop();
      return;
    }
  }

  // ---- 所有门控通过，开始运行控制器 ----

  const PathMatchResult center_result = path_locator_.Project(robot_state_);
  if (!center_result.success) {
    SPDLOG_ERROR("全局路径最近点搜索失败，输出停车");
    PublishStop();
    return;
  }
  if (center_result.at_path_end) {
    SPDLOG_INFO("已到达路径末端，输出停车");
    PublishStop();
    return;
  }
  const PathMatchResult& center_match = center_result;

  const LQROutput global_output = lqr_.ComputeCommand(robot_state_, center_match);
  if (!global_output.success) {
    SPDLOG_ERROR("全局路径控制输入无效，输出停车");
    PublishStop();
    return;
  }

  const double dist_to_goal = std::hypot(robot_state_.x - center_match.nearest_point.x, robot_state_.y - center_match.nearest_point.y);

  const ReferenceSource previous_source = reference_source_;
  ReferenceSource selected_source       = SelectReferenceSource(center_match, t);
  LQROutput selected_output             = global_output;
  if (selected_source == ReferenceSource::kVisionLine) {
    LQROutput vision_output = lqr_.ComputeLineCommand(robot_state_, line_reference_, global_output.vel_cmd.linear_v);
    if (vision_output.success) {
      vision_output.target_pt        = center_match.target_point;
      vision_output.distance_to_goal = dist_to_goal;
      vision_output.target_index     = center_match.point_index;
      selected_output                = vision_output;
    } else {
      SPDLOG_WARN("视觉直线 LQR 输入无效，降级为全局路径 LQR");
      line_reference_valid_ = false;
      selected_source       = ReferenceSource::kGlobalPath;
      selected_output       = global_output;
    }
  }
  reference_source_ = selected_source;
  if (reference_source_ != previous_source) {
    SPDLOG_INFO("参考源切换: {} -> {}", ReferenceSourceName(previous_source), ReferenceSourceName(reference_source_));
  }

  const double target_delta           = selected_output.vel_cmd.steer_angle;
  const double lateral_error_for_gate = selected_output.lateral_error;
  const int steer_mode                = selected_output.steer_mode;

  // 超横差保护
  const double lateral_error_abs = std::fabs(lateral_error_for_gate);
  if (lateral_error_abs > max_lateral_error_m_) {
    SPDLOG_ERROR("横向误差 {:.3f} m > {:.3f} m，输出停车", lateral_error_abs, max_lateral_error_m_);
    PublishStop();
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
  // 本检查放在最近点搜索之后，用新的路径点发现车辆冲出有效路径。
  // 只有真正丢失跟踪时才会触发停车。
  if (dist_to_goal > max_track_error_m_) {
    SPDLOG_ERROR(
      "偏离目标 {:.2f} m > {:.2f} m"
      "（横向/航向误差可能同时为 0，LQR 无沿路径误差项），"
      "输出停车。idx={}/{}",
      dist_to_goal,
      max_track_error_m_,
      center_match.point_index,
      path_point_count_);
    PublishStop();
    return;
  }

  const double target_v = selected_output.vel_cmd.linear_v;
  const double v        = LimitSpeed(target_v, last_speed_cmd_m_s_, speed_ramp_mps2_, 1.0 / control_hz_);
  last_speed_cmd_m_s_   = v;

  const double delta  = LimitSteerAngle(target_delta, last_steer_cmd_rad_, max_steer_rate_rad_s_, 1.0 / control_hz_);
  last_steer_cmd_rad_ = delta;

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

  const double vision_age_ms = line_reference_valid_ ? (t - last_crop_row_time_).seconds() * 1000.0 : -1.0;
  SPDLOG_DEBUG(
    "source={} steer_mode={} type={} turn_mode={} "
    "v={:.3f}/{:.3f} m/s delta={:+.2f}/{:+.2f} deg "
    "(fb={:+.2f} ff={:+.2f}) e_lat={:+.3f} m "
    "e_head={:+.3f} rad vision_age={:.1f}ms line_valid={} "
    "d_goal={:.2f} m control_idx={} center_idx={}/{}",
    ReferenceSourceName(reference_source_),
    steer_mode,
    static_cast<unsigned>(center_match.target_point.waypoint_type),
    center_match.work_mode == PathWorkMode::kTurn ? "turn" : "work",
    v,
    target_v,
    delta * kRad2Deg,
    target_delta * kRad2Deg,
    selected_output.steer_fb * kRad2Deg,
    selected_output.steer_ff * kRad2Deg,
    selected_output.lateral_error,
    selected_output.heading_error,
    vision_age_ms,
    line_reference_valid_,
    dist_to_goal,
    selected_output.target_index,
    center_match.point_index,
    path_point_count_);

  if (viz_) {
    PublishViz(selected_output.target_pt);
  }
}

// 发布当前跟踪目标点的 RViz Marker。
// 输入:
//   target : 当前目标轨迹点
void PathTrackerNode::PublishViz(const TrajectoryPoint& target) {
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
