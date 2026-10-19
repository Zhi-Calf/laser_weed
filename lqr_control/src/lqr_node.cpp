// Copyright 2026. ROS 2 node implementation for the four-wheel-steering LQR.
#include "lqr_control/lqr_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace lqr_control
{

namespace
{
/// Throttle repeated warnings so the log stays readable at 10 Hz.
constexpr int kWarnIntervalMs = 3000;
constexpr double kRad2Deg = 57.29577951308232;

/// Node name used in logs.
constexpr const char * kNodeName = "lqr_control";
}  // namespace

LqrNode::LqrNode()
: Node(kNodeName)
{
  // Order matters: the LQR object must exist before loadParameters() configures
  // it, otherwise loadParameters() dereferences a null pointer.
  lqr_ = std::make_unique<LQR>();
  loadParameters();

  setupInterfaces();

  const auto period_ns = std::chrono::nanoseconds(
    static_cast<int64_t>(1.0e9 / control_hz_));
  timer_ = create_wall_timer(period_ns, [this]() {controlTick();});

  RCLCPP_INFO(get_logger(),
    "LQR 控制器已启动: 轴距=%.3fm 等效轴距=%.3fm 最大转角=%.1fdeg 最大速度=%.2fm/s 频率=%.1fHz",
    wheelbase_, effective_wheelbase_, max_steer_deg_, max_speed_, control_hz_);
  RCLCPP_INFO(get_logger(),
    "订阅 里程计=%s 路径=%s 工作模式=%s | 发布 %s",
    odom_topic_.c_str(), path_topic_.c_str(), work_mode_topic_.c_str(),
    cmd_topic_.c_str());
}

void LqrNode::loadParameters()
{
  control_hz_ = declare_parameter<double>("control_hz", 10.0);
  wheelbase_ = declare_parameter<double>("wheelbase", 1.0);
  steer_mode_ = declare_parameter<int>("steer_mode", 0);
  max_steer_deg_ = declare_parameter<double>("max_steer_deg", 90.0);
  max_speed_ = declare_parameter<double>("max_speed_m_s", 1.4);
  target_v_ = declare_parameter<double>("lqr_target_v", 0.5);
  rtk_max_sigma_ = declare_parameter<double>("rtk_max_sigma", 0.05);
  max_track_error_m_ = declare_parameter<double>("max_track_error_m", 3.0);
  // rclcpp stores every integer parameter as int64_t, so declare_parameter<int>
  // actually yields int64_t. Passing that to std::max(0, ...) mixes int with
  // int64_t and template deduction fails to compile. Declare as int64_t and clamp
  // with plain comparisons (no template deduction to get wrong). Bounded to
  // [0, 1000] so a huge value cannot make the per-tick snap search expensive.
  int64_t snap_window_raw = declare_parameter<int64_t>("snap_window", 25);
  if (snap_window_raw < 0) {
    snap_window_raw = 0;
  }
  if (snap_window_raw > 1000) {
    snap_window_raw = 1000;
  }
  snap_window_ = static_cast<size_t>(snap_window_raw);
  odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 1.0);
  path_timeout_s_ = declare_parameter<double>("path_timeout_s", 5.0);
  work_mode_timeout_s_ = declare_parameter<double>("work_mode_timeout_s", 3.0);
  work_mode_startup_grace_s_ = declare_parameter<double>("work_mode_startup_grace_s", 20.0);
  require_unmanned_ = declare_parameter<bool>("require_unmanned", true);
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/gnss_driver/odom");
  path_topic_ = declare_parameter<std::string>("path_topic", "/global_path_planner/path");
  work_mode_topic_ = declare_parameter<std::string>("work_mode_topic", "/work_mode");
  cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/weeder/chassis/cmd");
  viz_ = declare_parameter<bool>("viz", true);

  // Sanity guards: a non-positive wheelbase would divide by zero in B_d, and a
  // non-positive control rate would make the timer period invalid.
  control_hz_ = std::clamp(control_hz_, 1.0, 200.0);
  wheelbase_ = std::max(wheelbase_, 0.05);
  max_speed_ = std::clamp(max_speed_, 0.1, 1.4);           // protocol ceiling 1400 mm/s
  max_steer_deg_ = std::clamp(max_steer_deg_, 1.0, 90.0);  // protocol +/-90 deg
  target_v_ = std::clamp(target_v_, 0.0, max_speed_);
  // 0 disables the along-track safety gate entirely; require a positive floor so
  // a typo cannot silently turn off the only guard against the LQR's missing
  // along-track error term.
  max_track_error_m_ = std::max(max_track_error_m_, 0.1);
  snap_window_ = std::min(snap_window_, static_cast<size_t>(1000));

  // Effective wheelbase depends on the steering mode, because the Ackermann
  // assumption behind B_d = v*dt/L only holds for modes 0 and 1.
  //   0 single Ackermann (front wheels only)  -> L
  //   1 dual Ackermann (front + rear oppose)  -> L/2: the same curvature is
  //     reached with half the steering angle, so the effective wheelbase halves
  //   2 crab / translation                    -> not applicable
  //   3 in-place rotation                     -> not applicable
  effective_wheelbase_ = wheelbase_;
  if (steer_mode_ == 1) {
    effective_wheelbase_ = wheelbase_ * 0.5;
  } else if (steer_mode_ == 2 || steer_mode_ == 3) {
    RCLCPP_ERROR(get_logger(),
      "steer_mode=%d (平移/原地转向) 下阿克曼假设不成立，B 矩阵 v*dt/L 无效，"
      "本控制器不能用于该模式。请改用 0(单阿克曼) 或 1(双阿克曼)。",
      steer_mode_);
  }
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

  // Re-anchor the tracked goal to the nearest path point within this window, on
  // top of the arc-length accumulation. Without it the goal trusts odometry
  // exclusively and can drift away from the robot with nothing pulling it back.
  lqr_->setSnapWindow(snap_window_);
}

void LqrNode::setupInterfaces()
{
  // gnss_driver publishes odom with SensorDataQoS (best effort, keep last 5);
  // a reliable subscriber would never match that publisher, so mirror its QoS.
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {odomCallback(msg);});

  // global_path_planner publishes ~/path transient_local (latched), so a late
  // subscriber must use matching durability to receive the last path.
  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic_, path_qos,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {pathCallback(msg);});

  // /work_mode is the system-wide "who controls the chassis" bus, published
  // latched by the mode manager so a late subscriber learns the current mode
  // immediately. Same QoS contract as global_path_planner's subscription.
  const auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  work_mode_sub_ = create_subscription<weeder_msgs::msg::WorkMode>(
    work_mode_topic_, mode_qos,
    [this](const weeder_msgs::msg::WorkMode::SharedPtr msg) {workModeCallback(msg);});

  cmd_pub_ = create_publisher<weeder_msgs::msg::ChassisCmd>(cmd_topic_, 10);
  if (viz_) {
    viz_pub_ = create_publisher<visualization_msgs::msg::Marker>("/lqr_control/target_point", 10);
  }

  const auto t = now();
  boot_time_ = t;
  last_odom_time_ = t;
  last_path_time_ = t;
  last_work_mode_time_ = t;
}

void LqrNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_time_ = now();
  has_odom_ = true;

  robot_state_.x = msg->pose.pose.position.x;
  robot_state_.y = msg->pose.pose.position.y;
  robot_state_.linear_v = msg->twist.twist.linear.x;

  // Quaternion -> yaw. gnss_driver publishes odom->base_link with the corrected
  // ENU heading, so this yaw is already in the same frame as the planner path.
  const auto & q = msg->pose.pose.orientation;
  const tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  robot_state_.heading = yaw;

  // RTK quality gate. gnss_driver fills the pose covariance from fix_type:
  // acc_rtk_fixed (0.02 m) for a fixed solution, 1e3 m otherwise, so a degraded
  // solution shows up as a hugely inflated sigma. Take the worse horizontal one.
  const double cov_xx = msg->pose.covariance[0];
  const double cov_yy = msg->pose.covariance[7];
  const double worst_var = std::max(std::max(cov_xx, cov_yy), 0.0);
  odom_sigma_ = std::sqrt(worst_var);
}

void LqrNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
      "收到的路径只有 %zu 个点，无法跟踪 (需要 >= 2)", msg->poses.size());
    return;
  }

  std::vector<std::pair<double, double>> xy;
  xy.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    xy.emplace_back(p.pose.position.x, p.pose.position.y);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // Always refresh the freshness timestamp: the path_timeout_s gate must see the
  // periodic republishes, or the planner would look stale even while alive.
  last_path_time_ = now();
  has_path_ = true;

  // Fingerprint check (inside the lock: it reads the path_fp_* members).
  //
  // global_path_planner republishes the identical path every 100 ms (its
  // timerCallback calls publishPath() unconditionally). Reloading the trajectory
  // on each republish would reset the tracked goal to index 0 every tick, pinning
  // the vehicle to the path start -- the "orange target point never moves"
  // symptom. Only a genuinely new path (different length or endpoints) may reset
  // tracking. computePathProfile copies x/y verbatim, so the raw xy values are
  // directly comparable with the stored fingerprint, letting us skip the reload
  // before paying for the profile computation on a multi-thousand-point path.
  const double tol = 1e-6;
  const bool same_path =
    (path_fp_count_ == xy.size()) &&
    (std::fabs(xy.front().first - path_fp_x0_) < tol) &&
    (std::fabs(xy.front().second - path_fp_y0_) < tol) &&
    (std::fabs(xy.back().first - path_fp_xn_) < tol) &&
    (std::fabs(xy.back().second - path_fp_yn_) < tol);
  if (same_path) {
    return;  // identical republish: keep tracking the existing goal index
  }

  std::vector<TrajectoryPoint> traj;
  // The planner path carries no velocity information (z encodes segment type
  // WORK/TURN), so the desired speed comes from the node parameter.
  if (!computePathProfile(xy, target_v_, &traj)) {
    RCLCPP_ERROR(get_logger(), "路径剖面计算失败，忽略这条路径");
    return;
  }

  path_fp_count_ = traj.size();
  path_fp_x0_ = traj.front().x;
  path_fp_y0_ = traj.front().y;
  path_fp_xn_ = traj.back().x;
  path_fp_yn_ = traj.back().y;
  path_point_count_ = traj.size();
  lqr_->setTrajectory(traj);
  first_tick_ = true;  // reset the travelled-distance accumulator

  RCLCPP_INFO(get_logger(),
    "已载入新路径: %zu 个点, 起点 (%.2f, %.2f), 终点 (%.2f, %.2f)",
    traj.size(), traj.front().x, traj.front().y, traj.back().x, traj.back().y);
}

void LqrNode::workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_work_mode_time_ = now();
  const bool first_mode = !ever_got_work_mode_;
  ever_got_work_mode_ = true;
  unmanned_ = (msg->mode == weeder_msgs::msg::WorkMode::MODE_UNMANNED);

  // Log only when the mode value actually changes (not on every 1 Hz republish),
  // so the log stays readable during a run. last_mode_ doubles as the change
  // detector; on the very first message it holds its 0 initial value.
  if (first_mode || msg->mode != last_mode_) {
    RCLCPP_INFO(get_logger(), "工作模式切换: %u -> %u (%s) -> %s",
      static_cast<unsigned>(last_mode_), static_cast<unsigned>(msg->mode),
      msg->mode_name.c_str(),
      unmanned_ ? "无人(接管底盘)" : "非无人(输出停车)");
    last_mode_ = msg->mode;
  }
}

bool LqrNode::rtkQualityOk(double & worst_sigma) const
{
  worst_sigma = odom_sigma_;
  // A sigma of exactly 0 means the driver never filled the covariance; treat
  // that as "unknown quality" rather than "perfect", and reject it.
  return odom_sigma_ > 0.0 && odom_sigma_ <= rtk_max_sigma_;
}

void LqrNode::publishStop()
{
  weeder_msgs::msg::ChassisCmd zero;
  zero.speed_m_s = 0.0;
  zero.steer_angle_rad = 0.0;
  cmd_pub_->publish(zero);
}

void LqrNode::controlTick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto t = now();
  const double since_boot = (t - boot_time_).seconds();

  // Startup grace: while nothing has ever arrived, stay quiet instead of
  // reporting a fault during boot (mirrors MegaTrak's *_startup_grace_s). It
  // only suppresses the report; it never allows motion.
  const bool in_grace = since_boot < work_mode_startup_grace_s_;

  // ---- Gate 1: work mode (the system-wide arm signal) ----
  // Motion is allowed only in MODE_UNMANNED. A stale /work_mode counts as
  // "not unmanned": the mode manager publishes at 1 Hz, so a gap longer than
  // work_mode_timeout_s means the APP or the link died.
  if (require_unmanned_) {
    const bool fresh = ever_got_work_mode_ &&
      (t - last_work_mode_time_).seconds() <= work_mode_timeout_s_;
    if (!fresh || !unmanned_) {
      if (!in_grace) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
          "未使能：需在 %s 上以 >= %.2fHz 持续发布 MODE_UNMANNED，输出停车",
          work_mode_topic_.c_str(), 1.0 / work_mode_timeout_s_);
      }
      publishStop();
      return;
    }
  }

  // ---- Gate 2: RTK pose present, fresh, and of fixed-solution quality ----
  if (!has_odom_) {
    if (!in_grace) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
        "等待定位数据 (%s)，输出停车", odom_topic_.c_str());
    }
    publishStop();
    return;
  }
  if ((t - last_odom_time_).seconds() > odom_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
      "定位数据超时 (%.2fs > %.2fs)，输出停车",
      (t - last_odom_time_).seconds(), odom_timeout_s_);
    publishStop();
    return;
  }

  double sigma = 0.0;
  if (!rtkQualityOk(sigma)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
      "定位精度不足 (sigma=%.3f > %.3f，可能非固定解)，输出停车", sigma, rtk_max_sigma_);
    publishStop();
    return;
  }

  // ---- Gate 3: a path to track, still fresh ----
  if (!has_path_) {
    if (!in_grace) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
        "等待路径 (%s)，输出停车", path_topic_.c_str());
    }
    publishStop();
    return;
  }
  if ((t - last_path_time_).seconds() > path_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
      "路径超时 (%.2fs > %.2fs)，输出停车",
      (t - last_path_time_).seconds(), path_timeout_s_);
    publishStop();
    return;
  }

  // ---- Gate 4: the Ackermann model must actually apply ----
  if (steer_mode_ == 2 || steer_mode_ == 3) {
    publishStop();
    return;
  }

  // ---- All gates passed: run the controller ----

  lqr_->setRobotState(robot_state_);

  // Advance the tracked goal by the distance actually travelled since last tick
  // (pure-pursuit style goal advancement, as in MegaTrak).
  if (first_tick_) {
    first_tick_ = false;
    last_x_ = robot_state_.x;
    last_y_ = robot_state_.y;
  } else {
    const double travelled = std::hypot(robot_state_.x - last_x_, robot_state_.y - last_y_);
    last_x_ = robot_state_.x;
    last_y_ = robot_state_.y;
    if (!lqr_->advanceGoal(travelled)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
        "已到达路径末端，输出停车");
      publishStop();
      return;
    }
  }

  // ---- Gate 5: the robot must still be near its tracked goal ----
  //
  // This gate exists because the LQR error state [e_lateral, e_heading] has NO
  // along-track component. A robot that runs straight past the goal along the
  // path's own direction reads as PERFECT tracking: both errors are exactly 0,
  // so the controller happily commands full speed with zero steering, forever.
  //
  // Observed in simulation (2026-10-15): goal frozen at the path start while the
  // robot ran 238 m downrange, steer_angle_rad = 2.4e-14, no warning logged. The
  // vehicle drove out of the field with every error metric at zero.
  //
  // Placed AFTER advanceGoal() so the snap-forward has already had its chance to
  // re-anchor the goal; only a genuine loss of tracking reaches this check.
  const double dist_to_goal = lqr_->distToGoal();
  if (dist_to_goal > max_track_error_m_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnIntervalMs,
      "偏离目标 %.2f m > %.2f m（横向/航向误差可能同时为 0 —— LQR 无沿路径误差项），"
      "输出停车。idx=%zu/%zu",
      dist_to_goal, max_track_error_m_, lqr_->nearestIndex(), path_point_count_);
    publishStop();
    return;
  }

  const LQROutput out = lqr_->getControlOutput();
  const double v = out.vel_cmd.linear_v;
  const double delta = out.vel_cmd.steer_angle;  // rad, left positive

  // 速度与转角在同一帧发布，保证 CAN 桥使用的是同一控制周期内的运动学组合。
  // 转角单位为 rad、左正，与 ADCU_toVCU_01 的符号约定一致。
  weeder_msgs::msg::ChassisCmd cmd;
  cmd.speed_m_s = v;
  cmd.steer_angle_rad = delta;
  cmd_pub_->publish(cmd);

  RCLCPP_DEBUG(get_logger(),
    "v=%.3f m/s delta=%+.2f deg (fb=%+.2f ff=%+.2f) e_lat=%+.3f m e_head=%+.3f rad "
    "d_goal=%.2f m idx=%zu/%zu",
    v, delta * kRad2Deg, out.steer_fb * kRad2Deg, out.steer_ff * kRad2Deg,
    out.lateral_error, out.heading_error, dist_to_goal,
    lqr_->nearestIndex(), path_point_count_);

  if (viz_) {
    publishViz(out.target_pt);
  }
}

void LqrNode::publishViz(const TrajectoryPoint & target)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = now();
  m.ns = "lqr_target";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = target.x;
  m.pose.position.y = target.y;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.25;
  m.scale.y = 0.25;
  m.scale.z = 0.25;
  m.color.r = 1.0f;
  m.color.g = 0.5f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;
  viz_pub_->publish(m);
}

}  // namespace lqr_control
