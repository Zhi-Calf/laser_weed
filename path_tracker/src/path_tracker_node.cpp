// 版权所有 2026。四轮转向 LQR 的 ROS 2 节点实现。
#include "path_tracker/path_tracker_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace path_tracker {

namespace {
/// 限制重复告警频率，避免 10 Hz 控制日志占满输出。
constexpr int kWarnIntervalMs = 3000;
constexpr double kRad2Deg     = 57.29577951308232;

/// 日志中使用的节点名称。
constexpr const char* kNodeName = "path_tracker";

// 计算路径段类型指纹。
uint64_t HashWaypointTypes(const std::vector<uint8_t>& types) {
  uint64_t hash = 1469598103934665603ULL;
  for (const uint8_t type : types) {
    hash ^= type;
    hash *= 1099511628211ULL;
  }
  return hash;
}

// 根据转向模式返回等效轴距。
double EffectiveWheelbase(double wheelbase, int mode) {
  return mode == 1 ? wheelbase * 0.5 : wheelbase;
}
}   // namespace

PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options) : Node(kNodeName, options) {
  // 必须先在 loadParameters() 之前创建 LQR 对象，否则配置时会访问空指针。
  lqr_ = std::make_unique<LQR>();
  loadParameters();

  setupInterfaces();

  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / control_hz_));
  timer_               = create_wall_timer(period_ns, [this]() {
    controlTick();
  });

  RCLCPP_INFO(get_logger(), "LQR 控制器已启动: 轴距=%.3fm 等效轴距=%.3fm 最大转角=%.1fdeg 最大速度=%.2fm/s 频率=%.1fHz", wheelbase_, effective_wheelbase_, max_steer_deg_, max_speed_, control_hz_);
  RCLCPP_INFO(
    get_logger(),
    "订阅 位姿=%s 路径=%s 工作模式=%s 避停状态=%s | 发布 %s | 避停门控=%s",
    pose_topic_.c_str(),
    path_topic_.c_str(),
    work_mode_topic_.c_str(),
    velocity_planner_status_topic_.c_str(),
    cmd_topic_.c_str(),
    require_velocity_planner_ ? "enabled" : "disabled");
}

void PathTrackerNode::loadParameters() {
  control_hz_          = declare_parameter<double>("control_hz", 10.0);
  wheelbase_           = declare_parameter<double>("wheelbase", 1.0);
  fallback_steer_mode_ = declare_parameter<int>("steer_mode", 0);
  if (fallback_steer_mode_ < 0 || fallback_steer_mode_ > 3) {
    RCLCPP_WARN(get_logger(), "steer_mode=%d 非法, 回退为 0 (单阿克曼)", fallback_steer_mode_);
    fallback_steer_mode_ = 0;
  }
  work_steer_mode_               = declare_parameter<int>("work_steer_mode", 0);
  turn_steer_mode_               = declare_parameter<int>("turn_steer_mode", 1);
  enable_turn_steer_mode_switch_ = declare_parameter<bool>("enable_turn_steer_mode_switch", true);
  turn_mode_enter_m_             = declare_parameter<double>("turn_mode_enter_m", 0.6);
  turn_mode_exit_m_              = declare_parameter<double>("turn_mode_exit_m", 0.6);
  max_steer_deg_                 = declare_parameter<double>("max_steer_deg", 90.0);
  max_speed_                     = declare_parameter<double>("max_speed_m_s", 1.4);
  target_v_                      = declare_parameter<double>("lqr_target_v", 0.5);
  work_speed_                    = declare_parameter<double>("work_speed_m_s", 0.6);
  turn_speed_                    = declare_parameter<double>("turn_speed_m_s", 0.3);
  turn_speed_preview_m_          = declare_parameter<double>("turn_speed_preview_m", 1.0);
  turn_speed_hold_m_             = declare_parameter<double>("turn_speed_hold_m", 0.5);
  speed_ramp_mps2_               = declare_parameter<double>("speed_ramp_mps2", 0.5);
  enable_segment_speed_profile_  = declare_parameter<bool>("enable_segment_speed_profile", true);
  lqr_speed_shaping_enabled_     = declare_parameter<bool>("lqr_enable_speed_shaping", false);
  max_track_error_m_             = declare_parameter<double>("max_track_error_m", 3.0);
  // rclcpp 将所有整数参数保存为 int64_t，因此这里显式按 int64_t 声明。
  // 使用普通比较完成限幅，避免 std::max 的模板类型推导失败。
  // 取值范围限制为 [0, 1000]，防止过大的吸附窗口拖慢每拍计算。
  int64_t snap_window_raw = declare_parameter<int64_t>("snap_window", 25);
  if (snap_window_raw < 0) {
    snap_window_raw = 0;
  }
  if (snap_window_raw > 1000) {
    snap_window_raw = 1000;
  }
  snap_window_                       = static_cast<size_t>(snap_window_raw);
  pose_timeout_s_                    = declare_parameter<double>("pose_timeout_s", 1.0);
  path_timeout_s_                    = declare_parameter<double>("path_timeout_s", 5.0);
  work_mode_timeout_s_               = declare_parameter<double>("work_mode_timeout_s", 3.0);
  work_mode_startup_grace_s_         = declare_parameter<double>("work_mode_startup_grace_s", 20.0);
  velocity_planner_status_timeout_s_ = declare_parameter<double>("velocity_planner_status_timeout_s", 0.5);
  require_unmanned_                  = declare_parameter<bool>("require_unmanned", true);
  require_velocity_planner_          = declare_parameter<bool>("require_velocity_planner", true);
  frame_id_                          = declare_parameter<std::string>("frame_id", "odom");
  pose_topic_                        = declare_parameter<std::string>("pose_topic", "/gnss_driver/pose");
  path_topic_                        = declare_parameter<std::string>("path_topic", "/global_path_planner/path");
  work_mode_topic_                   = declare_parameter<std::string>("work_mode_topic", "/work_mode");
  velocity_planner_status_topic_     = declare_parameter<std::string>("velocity_planner_status_topic", "/velocity_planner/status");
  cmd_topic_                         = declare_parameter<std::string>("cmd_topic", "/chassis/cmd");
  viz_                               = declare_parameter<bool>("viz", true);

  // 参数保护：非正轴距会使 B_d 除零，非正控制频率会生成无效定时器周期。
  control_hz_           = std::clamp(control_hz_, 1.0, 200.0);
  wheelbase_            = std::max(wheelbase_, 0.05);
  max_speed_            = std::clamp(max_speed_, 0.1, 1.4);        // 协议速度上限 1400 毫米/秒
  max_steer_deg_        = std::clamp(max_steer_deg_, 1.0, 90.0);   // 协议转角范围 ±90 度
  target_v_             = std::clamp(target_v_, 0.0, max_speed_);
  work_speed_           = std::clamp(work_speed_, 0.0, max_speed_);
  turn_speed_           = std::clamp(turn_speed_, 0.0, max_speed_);
  turn_speed_preview_m_ = std::max(turn_speed_preview_m_, 0.0);
  turn_speed_hold_m_    = std::max(turn_speed_hold_m_, 0.0);
  speed_ramp_mps2_      = std::max(speed_ramp_mps2_, 0.01);
  turn_mode_enter_m_    = std::max(turn_mode_enter_m_, 0.0);
  turn_mode_exit_m_     = std::max(turn_mode_exit_m_, 0.0);
  // 即使配置为 0 也强制使用正数下限，防止误配置关闭 LQR 缺失沿路径误差时
  // 唯一可用的安全保护。
  max_track_error_m_                 = std::max(max_track_error_m_, 0.1);
  snap_window_                       = std::min(snap_window_, static_cast<size_t>(1000));
  velocity_planner_status_timeout_s_ = std::max(velocity_planner_status_timeout_s_, 0.1);
  if (velocity_planner_status_topic_.empty()) {
    velocity_planner_status_topic_ = "/velocity_planner/status";
  }

  if (work_steer_mode_ < 0 || work_steer_mode_ > 1) {
    RCLCPP_WARN(get_logger(), "work_steer_mode=%d 非法, 回退为 0 (单阿克曼)", work_steer_mode_);
    work_steer_mode_ = 0;
  }
  if (turn_steer_mode_ < 0 || turn_steer_mode_ > 1) {
    RCLCPP_WARN(get_logger(), "turn_steer_mode=%d 非法, 回退为 1 (对称阿克曼)", turn_steer_mode_);
    turn_steer_mode_ = 1;
  }
  if (fallback_steer_mode_ > 1) {
    RCLCPP_ERROR(get_logger(), "steer_mode=%d 不支持；无路径类型时只允许 0 或 1", fallback_steer_mode_);
    fallback_steer_mode_ = 0;
  }
  steer_mode_          = fallback_steer_mode_;
  effective_wheelbase_ = EffectiveWheelbase(wheelbase_, steer_mode_);
  effective_wheelbase_ = std::max(effective_wheelbase_, 0.05);

  lqr_->configure(
    1.0 / control_hz_,
    declare_parameter<double>("lqr_eps", 1e-6),
    declare_parameter<int>("lqr_max_iteration", 1000),
    declare_parameter<double>("lqr_Q_lateral", 50.0),
    declare_parameter<double>("lqr_Q_heading", 25.0),
    declare_parameter<double>("lqr_R_steer", 25.0),
    effective_wheelbase_,
    max_steer_deg_ / kRad2Deg,
    max_speed_,
    declare_parameter<double>("lqr_min_speed_factor", 0.3),
    declare_parameter<double>("lqr_heading_deadband", 0.1),
    declare_parameter<double>("lqr_heading_max", 1.0),
    target_v_,
    declare_parameter<double>("lqr_min_vel", 0.1));

  lqr_->setSpeedShaping(
    declare_parameter<double>("lqr_curvature_threshold", 0.2),
    declare_parameter<double>("lqr_curvature_span", 0.5),
    declare_parameter<double>("lqr_speed_scale", 0.4),
    declare_parameter<int>("lqr_lookahead_index", 30),
    declare_parameter<int>("lqr_lookback_start", 20));
  lqr_->SetSpeedShapingEnabled(lqr_speed_shaping_enabled_);
  lqr_->SetSegmentSpeedProfile(work_speed_, turn_speed_, turn_speed_preview_m_, turn_speed_hold_m_, enable_segment_speed_profile_);

  // 在弧长累计之外，将目标点重新锚定到窗口内最近的路径点。
  // 否则目标点会完全依赖里程计，打滑或跳变后无法自动纠正。
  lqr_->setSnapWindow(snap_window_);
}

void PathTrackerNode::setupInterfaces() {
  // gnss_driver 使用 SensorDataQoS（尽力传输，保留最近 5 条）发布位姿。
  // 可靠订阅端无法与该发布端匹配，因此这里使用相同的 QoS。
  pose_sub_ = create_subscription<weeder_msgs::msg::Pose>(pose_topic_, rclcpp::SensorDataQoS(), [this](const weeder_msgs::msg::Pose::SharedPtr msg) {
    poseCallback(msg);
  });

  // global_path_planner 使用 transient_local（瞬态本地持久化）发布路径，
  // 晚加入的订阅端必须使用相同持久性才能收到最后一条路径。
  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_           = create_subscription<weeder_msgs::msg::Path>(path_topic_, path_qos, [this](const weeder_msgs::msg::Path::SharedPtr msg) {
    pathCallback(msg);
  });

  // /work_mode 是系统级底盘控制权总线，由模式管理器以锁存方式发布，
  // 晚加入的订阅端可立即获得当前模式。QoS 与路径话题保持一致。
  const auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  work_mode_sub_      = create_subscription<weeder_msgs::msg::WorkMode>(work_mode_topic_, mode_qos, [this](const weeder_msgs::msg::WorkMode::SharedPtr msg) {
    workModeCallback(msg);
  });

  const auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  obstacle_status_sub_    = create_subscription<weeder_msgs::msg::ObstacleStatus>(velocity_planner_status_topic_, obstacle_qos, [this](const weeder_msgs::msg::ObstacleStatus::SharedPtr msg) {
    ObstacleStatusCallback(msg);
  });

  cmd_pub_ = create_publisher<weeder_msgs::msg::ChassisCmd>(cmd_topic_, 10);
  if (viz_) {
    viz_pub_ = create_publisher<visualization_msgs::msg::Marker>("/path_tracker/target_point", 10);
  }

  const auto t               = now();
  boot_time_                 = t;
  last_pose_time_            = t;
  last_path_time_            = t;
  last_work_mode_time_       = t;
  last_obstacle_status_time_ = t;
}

void PathTrackerNode::poseCallback(const weeder_msgs::msg::Pose::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_pose_time_       = now();
  has_pose_             = true;
  pose_valid_           = msg->valid;
  robot_state_.x        = msg->x;
  robot_state_.y        = msg->y;
  robot_state_.heading  = msg->yaw;
  robot_state_.linear_v = msg->speed_m_s;
}

void PathTrackerNode::pathCallback(const weeder_msgs::msg::Path::SharedPtr msg) {
  if (msg->x.size() < 2 || msg->x.size() != msg->y.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "收到的路径无效：x=%zu y=%zu (需要 x==y 且 >= 2)", msg->x.size(), msg->y.size());
    return;
  }

  std::vector<std::pair<double, double>> xy;
  xy.reserve(msg->x.size());
  for (size_t i = 0; i < msg->x.size(); ++i) {
    xy.emplace_back(msg->x[i], msg->y[i]);
  }

  bool has_waypoint_types = !msg->waypoint_types.empty() && msg->waypoint_types.size() == msg->x.size();
  if (!msg->waypoint_types.empty() && !has_waypoint_types) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      kWarnIntervalMs,
      "waypoint_types 长度=%zu 与路径点数=%zu 不一致，"
      "将回退到固定速度/模式",
      msg->waypoint_types.size(),
      msg->x.size());
  }

  std::vector<uint8_t> waypoint_types(msg->x.size(), 0);
  if (has_waypoint_types) {
    for (size_t i = 0; i < msg->waypoint_types.size(); ++i) {
      if (msg->waypoint_types[i] > 1) {
        has_waypoint_types = false;
        break;
      }
      waypoint_types[i] = msg->waypoint_types[i];
    }
  }
  if (!has_waypoint_types) {
    std::fill(waypoint_types.begin(), waypoint_types.end(), 0);
  }
  const uint64_t waypoint_type_hash = HashWaypointTypes(waypoint_types);

  std::lock_guard<std::mutex> lock(mutex_);
  // 每次回调都必须刷新时间戳，确保 path_timeout_s 能看到规划器的周期重发；
  // 否则规划器仍在运行时也会被误判为路径超时。
  last_path_time_ = now();
  has_path_       = true;

  // 在锁内检查路径指纹，读取 path_fp_* 成员。
  //
  // global_path_planner 每 100 毫秒重发相同路径。如果每次重发都重新加载轨迹，
  // 目标索引会反复重置为 0，车辆会被固定在路径起点，表现为
  // “橙色目标点始终不动”。只有路径点数、首尾点或分段类型真正变化时才重置跟踪。
  // 原始 x/y 可直接用于指纹比较，避免在数千点路径上重复计算路径剖面。
  const double tol     = 1e-6;
  const bool same_path = (path_fp_count_ == xy.size()) && (path_fp_type_hash_ == waypoint_type_hash) && (std::fabs(xy.front().first - path_fp_x0_) < tol) && (std::fabs(xy.front().second - path_fp_y0_) < tol) &&
                         (std::fabs(xy.back().first - path_fp_xn_) < tol) && (std::fabs(xy.back().second - path_fp_yn_) < tol);
  if (same_path) {
    return;   // 相同路径重发，保留当前目标索引
  }

  std::vector<TrajectoryPoint> traj;
  // 规划器路径默认不带速度信息，因此先写入节点配置的回退速度。
  if (!computePathProfile(xy, target_v_, &traj)) {
    RCLCPP_ERROR(get_logger(), "路径剖面计算失败，忽略这条路径");
    return;
  }
  for (size_t i = 0; i < traj.size(); ++i) {
    traj[i].waypoint_type = waypoint_types[i];
    if (has_waypoint_types && enable_segment_speed_profile_) {
      traj[i].v = traj[i].waypoint_type == 1 ? turn_speed_ : work_speed_;
    }
  }

  path_fp_count_           = traj.size();
  path_fp_x0_              = traj.front().x;
  path_fp_y0_              = traj.front().y;
  path_fp_xn_              = traj.back().x;
  path_fp_yn_              = traj.back().y;
  path_fp_type_hash_       = waypoint_type_hash;
  path_point_count_        = traj.size();
  path_has_waypoint_types_ = has_waypoint_types;
  turn_mode_active_        = false;
  last_speed_cmd_          = 0.0;
  lqr_->setTrajectory(traj);
  first_tick_ = true;   // 重置行驶距离累计器

  RCLCPP_INFO(
    get_logger(),
    "已载入新路径: %zu 个点, 起点 (%.2f, %.2f), "
    "终点 (%.2f, %.2f), 分段类型=%s",
    traj.size(),
    traj.front().x,
    traj.front().y,
    traj.back().x,
    traj.back().y,
    has_waypoint_types ? "enabled" : "fallback");
}

void PathTrackerNode::workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_work_mode_time_  = now();
  const bool first_mode = !ever_got_work_mode_;
  ever_got_work_mode_   = true;
  unmanned_             = (msg->mode == weeder_msgs::msg::WorkMode::MODE_UNMANNED);

  // 仅在模式变化时记录日志，避免 1 Hz 重发刷屏。
  // last_mode_ 同时用于变化检测；首条消息时其初值为 0。
  if (first_mode || msg->mode != last_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "工作模式切换: %u -> %u (%s) -> %s",
      static_cast<unsigned>(last_mode_),
      static_cast<unsigned>(msg->mode),
      msg->mode_name.c_str(),
      unmanned_ ? "无人(接管底盘)" : "非无人(输出停车)");
    last_mode_ = msg->mode;
  }
}

void PathTrackerNode::ObstacleStatusCallback(const weeder_msgs::msg::ObstacleStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_obstacle_status_time_ = now();
  has_obstacle_status_       = true;
  obstacle_sensor_valid_     = msg->sensor_valid;
  obstacle_stop_required_    = msg->stop_required;
  obstacle_points_           = msg->obstacle_points;
}

void PathTrackerNode::ApplySteerMode(int mode) {
  if (mode != 0 && mode != 1) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "拒绝不支持的转向模式 %d，输出停车", mode);
    publishStop();
    return;
  }
  if (steer_mode_ == mode) {
    return;
  }
  steer_mode_          = mode;
  effective_wheelbase_ = std::max(EffectiveWheelbase(wheelbase_, steer_mode_), 0.05);
  lqr_->SetWheelbase(effective_wheelbase_);
  RCLCPP_INFO(get_logger(), "转向模式切换为 %d (%s)，等效轴距 %.3f m", steer_mode_, steer_mode_ == 1 ? "对称阿克曼" : "单阿克曼", effective_wheelbase_);
}

void PathTrackerNode::publishStop() {
  weeder_msgs::msg::ChassisCmd zero;
  zero.gear            = 0;
  zero.steer_mode      = 0;
  zero.release_brake   = false;
  zero.estop           = true;
  zero.ctrl_mode       = false;
  zero.speed_m_s       = 0.0;
  zero.steer_angle_rad = 0.0;
  cmd_pub_->publish(zero);
  last_speed_cmd_ = 0.0;
}

void PathTrackerNode::controlTick() {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto t            = now();
  const double since_boot = (t - boot_time_).seconds();

  // 启动宽限期：上电后尚未收到任何数据时抑制告警，避免启动阶段误报。
  // 与 MegaTrak 的 *_startup_grace_s 语义一致。宽限期只抑制告警，不放行运动。
  const bool in_grace = since_boot < work_mode_startup_grace_s_;

  // ---- 门控 1：工作模式，也就是系统级使能信号 ----
  // 只有 MODE_UNMANNED 才允许运动。过期的 /work_mode 按非无人模式处理：
  // 模式管理器以 1 Hz 发布，超过 work_mode_timeout_s 未更新说明 APP 或链路失效。
  if (require_unmanned_) {
    const bool fresh = ever_got_work_mode_ && (t - last_work_mode_time_).seconds() <= work_mode_timeout_s_;
    if (!fresh || !unmanned_) {
      if (!in_grace) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "未使能：需在 %s 上以 >= %.2fHz 持续发布 MODE_UNMANNED，输出停车", work_mode_topic_.c_str(), 1.0 / work_mode_timeout_s_);
      }
      publishStop();
      return;
    }
  }

  // ---- 门控 2：RTK 位姿存在、新鲜且为固定解 ----
  if (!has_pose_) {
    if (!in_grace) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "等待定位数据 (%s)，输出停车", pose_topic_.c_str());
    }
    publishStop();
    return;
  }
  if ((t - last_pose_time_).seconds() > pose_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "定位数据超时 (%.2fs > %.2fs)，输出停车", (t - last_pose_time_).seconds(), pose_timeout_s_);
    publishStop();
    return;
  }

  if (!pose_valid_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "定位位姿无效（非 RTK 固定解），输出停车");
    publishStop();
    return;
  }

  // ---- 门控 3：存在可跟踪且未超时的路径 ----
  if (!has_path_) {
    if (!in_grace) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "等待路径 (%s)，输出停车", path_topic_.c_str());
    }
    publishStop();
    return;
  }
  if ((t - last_path_time_).seconds() > path_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "路径超时 (%.2fs > %.2fs)，输出停车", (t - last_path_time_).seconds(), path_timeout_s_);
    publishStop();
    return;
  }

  // ---- 门控 4：当前转向模式必须满足阿克曼模型假设 ----
  if (steer_mode_ == 2 || steer_mode_ == 3) {
    publishStop();
    return;
  }

  // ---- 门控 5：点云避停安全门控 ----
  const double obstacle_status_age_s = has_obstacle_status_ ? (t - last_obstacle_status_time_).seconds() : 0.0;
  const ObstacleGateDecision obstacle_decision =
    EvaluateObstacleGate(require_velocity_planner_, has_obstacle_status_, obstacle_status_age_s, velocity_planner_status_timeout_s_, obstacle_sensor_valid_, obstacle_stop_required_);
  if (obstacle_decision != ObstacleGateDecision::kAllow) {
    switch (obstacle_decision) {
      case ObstacleGateDecision::kUnavailable:
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          kWarnIntervalMs,
          "避停状态不可用或超时 (%.2fs > %.2fs)，故障码 %u，输出停车",
          obstacle_status_age_s,
          velocity_planner_status_timeout_s_,
          fault::kVelocityPlannerUnavailable);
        break;
      case ObstacleGateDecision::kSensorInvalid:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "避停感知数据无效，故障码 %u，输出停车", fault::kVelocityPlannerUnavailable);
        break;
      case ObstacleGateDecision::kStopRequired:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "ROI 内检测到 %u 个障碍点，故障码 %u，输出停车", obstacle_points_, fault::kObstacleInRoi);
        break;
      case ObstacleGateDecision::kAllow:
        break;
    }
    publishStop();
    return;
  }

  // ---- 所有门控通过，开始运行控制器 ----

  lqr_->setRobotState(robot_state_);

  // 按上一控制周期以来的实际位移推进目标点，方式与 MegaTrak 的纯追踪一致。
  if (first_tick_) {
    first_tick_ = false;
    last_x_     = robot_state_.x;
    last_y_     = robot_state_.y;
  } else {
    const double travelled = std::hypot(robot_state_.x - last_x_, robot_state_.y - last_y_);
    last_x_                = robot_state_.x;
    last_y_                = robot_state_.y;
    if (!lqr_->advanceGoal(travelled)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs, "已到达路径末端，输出停车");
      publishStop();
      return;
    }
  }

  // 根据路径段类型选择转向模式。进入和退出使用不同搜索方向，
  // 通过 turn_mode_active_ 保持滞回，避免边界反复切换。
  int desired_steer_mode = fallback_steer_mode_;
  if (enable_turn_steer_mode_switch_ && path_has_waypoint_types_) {
    if (!turn_mode_active_ && lqr_->HasTurnNear(turn_mode_enter_m_, 0.0)) {
      turn_mode_active_ = true;
    } else if (turn_mode_active_ && !lqr_->HasTurnNear(turn_mode_enter_m_, turn_mode_exit_m_)) {
      turn_mode_active_ = false;
    }
    desired_steer_mode = turn_mode_active_ ? turn_steer_mode_ : work_steer_mode_;
  }
  ApplySteerMode(desired_steer_mode);

  // ---- 门控 6：车辆必须仍在跟踪目标点附近 ----
  //
  // 该门控用于弥补 LQR 误差状态 [e_lateral, e_heading] 不含沿路径方向误差的问题。
  // 车辆沿路径方向冲过目标点时，横向和航向误差都为 0，控制器会误判为完美跟踪，
  // 从而持续输出全速零转角。
  //
  // 2026-10-15 仿真中曾出现目标点冻结在路径起点、车辆驶出 238 米，
  // 转角为 2.4e-14 且没有任何告警的情况，所有误差指标仍显示正常。
  //
  // 本检查放在 advanceGoal() 之后，使前向吸附先完成重新锚定；
  // 只有真正丢失跟踪时才会触发停车。
  const double dist_to_goal = lqr_->distToGoal();
  if (dist_to_goal > max_track_error_m_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      kWarnIntervalMs,
      "偏离目标 %.2f m > %.2f m（横向/航向误差可能同时为 0 —— LQR 无沿路径误差项），"
      "输出停车。idx=%zu/%zu",
      dist_to_goal,
      max_track_error_m_,
      lqr_->nearestIndex(),
      path_point_count_);
    publishStop();
    return;
  }

  const LQROutput out         = lqr_->getControlOutput();
  const double target_v       = out.vel_cmd.linear_v;
  const double max_speed_step = speed_ramp_mps2_ / control_hz_;
  double v                    = last_speed_cmd_;
  if (target_v > v) {
    v = std::min(target_v, v + max_speed_step);
  } else {
    v = std::max(target_v, v - max_speed_step);
  }
  last_speed_cmd_    = v;
  const double delta = out.vel_cmd.steer_angle;   // 弧度，左转为正

  // 速度与转角在同一帧发布，保证 CAN 桥使用的是同一控制周期内的运动学组合。
  // 转角单位为 rad、左正，与 ADCU_toVCU_01 的符号约定一致。
  weeder_msgs::msg::ChassisCmd cmd;
  cmd.gear            = 1;
  cmd.steer_mode      = static_cast<uint8_t>(steer_mode_);
  cmd.release_brake   = true;
  cmd.estop           = false;
  cmd.ctrl_mode       = true;
  cmd.speed_m_s       = v;
  cmd.steer_angle_rad = delta;
  cmd_pub_->publish(cmd);

  RCLCPP_DEBUG(
    get_logger(),
    "mode=%d type=%u v=%.3f/%.3f m/s delta=%+.2f deg "
    "(fb=%+.2f ff=%+.2f) e_lat=%+.3f m e_head=%+.3f rad "
    "d_goal=%.2f m idx=%zu/%zu",
    steer_mode_,
    static_cast<unsigned>(out.target_pt.waypoint_type),
    v,
    target_v,
    delta * kRad2Deg,
    out.steer_fb * kRad2Deg,
    out.steer_ff * kRad2Deg,
    out.lateral_error,
    out.heading_error,
    dist_to_goal,
    lqr_->nearestIndex(),
    path_point_count_);

  if (viz_) {
    publishViz(out.target_pt);
  }
}

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
