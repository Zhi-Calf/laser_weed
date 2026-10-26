#include "sim_vehicle/sim_vehicle_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace sim_vehicle {
namespace {

constexpr double kMinimumGeometryM = 0.05;
constexpr double kMinimumRadiusM = 0.01;
constexpr double kMinimumSimHz = 1.0;
constexpr double kMinimumSteerRad = 0.01;
constexpr double kZeroSpeedMps = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kBodyMarkerId = 0;
constexpr int32_t kBumperMarkerId = 5;

double ClampNonNegative(double value) { return std::max(value, 0.0); }

}  // namespace

// 初始化车辆节点并启动定时器。
SimVehicleNode::SimVehicleNode(const rclcpp::NodeOptions& options)
    : Node("sim_vehicle", options) {
  Initialize();
}

// 按参数、接口和运行状态的顺序完成节点初始化。
void SimVehicleNode::Initialize() {
  LoadParameters();
  wheel_state_ =
    ComputeFourWheelState(0.0, 0.0, 0, wheelbase_m_, wheel_track_m_,
                          wheel_radius_m_, max_plant_steer_rad_);
  last_cmd_time_ = now();
  SetupInterfaces();
  SetupTimers();
  PublishSlowTopics();

  RCLCPP_INFO(get_logger(),
              "四轮转向仿真车辆已启动: 轴距=%.3fm 轮距=%.3fm "
              "轮半径=%.3fm dt=%.3fs 初始位姿=(%.2f, %.2f, %.1f°) "
              "模式=%s",
              wheelbase_m_, wheel_track_m_, wheel_radius_m_, dt_s_, x_m_, y_m_,
              yaw_rad_ * 180.0 / kPi, unmanned_ ? "UNMANNED" : "MANUAL");
  RCLCPP_INFO(get_logger(),
              "订阅 /chassis/cmd + /sim/unmanned | "
              "发布 /gnss_driver/pose + /gnss_driver/origin + "
              "/work_mode + /tf");
}

// 读取仿真车辆的全部节点参数。
void SimVehicleNode::LoadParameters() {
  LoadGeometryParameters();
  LoadPoseParameters();
  start_unmanned_ = declare_parameter<bool>("start_unmanned", false);
  unmanned_ = start_unmanned_;
}

// 读取车辆几何、速度和定时周期参数。
void SimVehicleNode::LoadGeometryParameters() {
  wheelbase_m_ = std::max(declare_parameter<double>("wheelbase", kWheelbaseM),
                          kMinimumGeometryM);
  wheel_track_m_ = std::max(
    declare_parameter<double>("wheel_track", kWheelTrackM), kMinimumGeometryM);
  wheel_radius_m_ =
    std::max(declare_parameter<double>("wheel_radius_m", kWheelRadiusM),
             kMinimumRadiusM);
  vehicle_length_m_ =
    std::max(declare_parameter<double>("vehicle_length_m", kVehicleLengthM),
             0.1);
  vehicle_width_m_ =
    std::max(declare_parameter<double>("vehicle_width_m", kVehicleWidthM), 0.1);
  vehicle_height_m_ =
    std::max(declare_parameter<double>("vehicle_height_m", kVehicleHeightM),
             0.1);
  dt_s_ =
    1.0 / std::max(declare_parameter<double>("sim_hz", 50.0), kMinimumSimHz);
  max_speed_m_s_ = std::min(
    ClampNonNegative(declare_parameter<double>("max_speed_m_s", kMaxSpeedMps)),
    kMaxSpeedMps);
  max_plant_steer_rad_ =
    std::min(std::max(declare_parameter<double>("max_plant_steer_deg",
                                                kMaxSteerRad * 180.0 / kPi) *
                        kPi / 180.0,
                      kMinimumSteerRad),
             kMaxSteerRad);
  cmd_timeout_s_ =
    ClampNonNegative(declare_parameter<double>("cmd_timeout_s", 0.3));
  cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/chassis/cmd");
}

// 读取初始位姿、模拟原点与路径对齐参数。
void SimVehicleNode::LoadPoseParameters() {
  x_m_ = declare_parameter<double>("init_x", 0.0);
  y_m_ = declare_parameter<double>("init_y", 0.0);
  yaw_rad_ = WrapAngle(declare_parameter<double>("init_yaw", 0.0));
  origin_lat_ = declare_parameter<double>("origin_lat", 34.6198653);
  origin_lon_ = declare_parameter<double>("origin_lon", 112.4497271);
  origin_alt_m_ = declare_parameter<double>("origin_alt", 100.0);
  align_to_path_start_ = declare_parameter<bool>("align_to_path_start", true);
  align_lateral_offset_m_ =
    declare_parameter<double>("align_lateral_offset", 0.2);
}

// 创建订阅者、发布者及其 QoS 配置。
void SimVehicleNode::SetupInterfaces() {
  chassis_cmd_sub_ = create_subscription<weeder_msgs::msg::ChassisCmd>(
    cmd_topic_, 10,
    [this](const weeder_msgs::msg::ChassisCmd::SharedPtr message) {
      OnChassisCmd(message);
    });
  unmanned_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/sim/unmanned", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
      OnUnmanned(message);
    });

  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_ = create_subscription<weeder_msgs::msg::Path>(
    "/global_path_planner/path", latched_qos,
    [this](const weeder_msgs::msg::Path::SharedPtr message) {
      OnPath(message);
    });

  pose_pub_ = create_publisher<weeder_msgs::msg::Pose>("/gnss_driver/pose",
                                                       rclcpp::SensorDataQoS());
  pose_stamped_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/gnss_driver/pose_stamped", rclcpp::SensorDataQoS());
  origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    "/gnss_driver/origin", latched_qos);
  work_mode_pub_ =
    create_publisher<weeder_msgs::msg::WorkMode>("/work_mode", latched_qos);
  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/sim_vehicle/marker_array", 10);
  chassis_state_pub_ =
    create_publisher<weeder_msgs::msg::ChassisState>("/chassis/state", 10);
}

// 创建仿真、底盘反馈和慢速话题定时器。
void SimVehicleNode::SetupTimers() {
  const auto tick_period = std::chrono::duration<double>(dt_s_);
  tick_timer_ =
    rclcpp::create_timer(this, get_clock(), tick_period, [this]() { Tick(); });
  chassis_state_timer_ =
    rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100),
                         [this]() { PublishChassisState(); });
  slow_topic_timer_ =
    rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1),
                         [this]() { PublishSlowTopics(); });
}

// 接收底盘指令并检查控制模式、急停和数值有效性。
void SimVehicleNode::OnChassisCmd(
  const weeder_msgs::msg::ChassisCmd::SharedPtr message) {
  last_cmd_time_ = now();
  cmd_gear_ = message->gear;
  cmd_release_brake_ = message->release_brake;
  cmd_estop_ = message->estop;
  cmd_ctrl_mode_ = message->ctrl_mode;

  const bool command_inhibited =
    message->estop || message->gear != 1 || !message->release_brake ||
    !message->ctrl_mode || !std::isfinite(message->speed_m_s) ||
    !std::isfinite(message->steer_angle_rad);
  if (command_inhibited) {
    speed_cmd_m_s_ = 0.0;
    steer_cmd_rad_ = 0.0;
    return;
  }

  if (message->steer_mode > 1) {
    if (steer_mode_ != message->steer_mode) {
      RCLCPP_WARN(get_logger(),
                  "四轮转向仿真仅支持 steer_mode 0/1，"
                  "收到 %d，车辆停车",
                  static_cast<int>(message->steer_mode));
    }
    steer_mode_ = message->steer_mode;
    speed_cmd_m_s_ = 0.0;
    steer_cmd_rad_ = 0.0;
    return;
  }

  if (steer_mode_ != message->steer_mode) {
    steer_mode_ = message->steer_mode;
    RCLCPP_INFO(get_logger(),
                "转向模式切换为 %d（等效轴距 %.3fm）",
                static_cast<int>(steer_mode_), EffectiveWheelbase());
  }
  speed_cmd_m_s_ = std::clamp(message->speed_m_s, 0.0, max_speed_m_s_);
  const double clamped_steer = std::clamp(
    message->steer_angle_rad, -max_plant_steer_rad_, max_plant_steer_rad_);
  if (std::abs(clamped_steer - message->steer_angle_rad) > 1e-9) {
    ++steer_clamped_count_;
    if (steer_clamped_count_ % 50 == 1) {
      RCLCPP_WARN(get_logger(),
                  "转向指令 %.1f° 超出仿真车辆上限 %.1f°，"
                  "已夹取（累计 "
                  "%zu 次）。若频繁出现说明 lqr_Q/R 或 wheelbase "
                  "配得不合理。",
                  message->steer_angle_rad * 180.0 / kPi,
                  max_plant_steer_rad_ * 180.0 / kPi, steer_clamped_count_);
    }
  }
  steer_cmd_rad_ = clamped_steer;
}

// 处理手动和无人驾驶模式的切换。
void SimVehicleNode::OnUnmanned(const std_msgs::msg::Bool::SharedPtr message) {
  if (message->data == unmanned_) {
    return;
  }
  unmanned_ = message->data;
  RCLCPP_INFO(get_logger(), "工作模式切换 -> %s",
              unmanned_ ? "UNMANNED（允许 path_tracker 驱动）"
                        : "MANUAL（规划器自动规划）");
}

// 收到首条路径后，将车辆放到路径起点的横向偏差处。
void SimVehicleNode::OnPath(const weeder_msgs::msg::Path::SharedPtr message) {
  if (aligned_ || !align_to_path_start_ || message->x.size() < 2 ||
      message->x.size() != message->y.size()) {
    return;
  }
  aligned_ = true;
  const double start_yaw =
    std::atan2(message->y[1] - message->y[0], message->x[1] - message->x[0]);
  x_m_ = message->x[0] - std::sin(start_yaw) * align_lateral_offset_m_;
  y_m_ = message->y[0] + std::cos(start_yaw) * align_lateral_offset_m_;
  yaw_rad_ = start_yaw;
  RCLCPP_INFO(get_logger(),
              "已对齐到路径起点 (%.2f, %.2f)，航向 %.1f°，"
              "故意留横向偏差 %.2fm 以观察收敛。"
              "路径共 %zu 点。",
              x_m_, y_m_, yaw_rad_ * 180.0 / kPi, align_lateral_offset_m_,
              message->x.size());
}

// 执行固定步长的车辆运动学积分。
void SimVehicleNode::Tick() {
  const rclcpp::Time current_time = now();
  const double elapsed_s = (current_time - last_cmd_time_).seconds();
  if (elapsed_s > cmd_timeout_s_) {
    speed_cmd_m_s_ = 0.0;
    steer_cmd_rad_ = 0.0;
  }

  speed_m_s_ = speed_cmd_m_s_;
  if (speed_m_s_ > kZeroSpeedMps) {
    x_m_ += speed_m_s_ * std::cos(yaw_rad_) * dt_s_;
    y_m_ += speed_m_s_ * std::sin(yaw_rad_) * dt_s_;
    yaw_rad_ = WrapAngle(yaw_rad_ + speed_m_s_ / EffectiveWheelbase() *
                                      std::tan(steer_cmd_rad_) * dt_s_);
  }
  wheel_state_ = ComputeFourWheelState(speed_m_s_, steer_cmd_rad_, steer_mode_,
                                       wheelbase_m_, wheel_track_m_,
                                       wheel_radius_m_, max_plant_steer_rad_);
  PublishPose();
  PublishTf();
  PublishMarker();
}

// 发布模拟 RTK 的车辆位姿。
void SimVehicleNode::PublishPose() {
  weeder_msgs::msg::Pose pose;
  pose.header.stamp = now();
  pose.header.frame_id = "odom";
  pose.x = x_m_;
  pose.y = y_m_;
  pose.z = 0.0;
  pose.roll = 0.0;
  pose.pitch = 0.0;
  pose.yaw = yaw_rad_;
  pose.satellites = 30;
  pose.valid = true;
  pose.speed_m_s = speed_m_s_;
  pose_pub_->publish(pose);

  geometry_msgs::msg::PoseStamped stamped;
  stamped.header = pose.header;
  stamped.pose.position.x = x_m_;
  stamped.pose.position.y = y_m_;
  stamped.pose.position.z = 0.0;
  const double half_yaw = yaw_rad_ * 0.5;
  stamped.pose.orientation.z = std::sin(half_yaw);
  stamped.pose.orientation.w = std::cos(half_yaw);
  pose_stamped_pub_->publish(stamped);
}

// 发布 odom 到 base_link 的平面变换。
void SimVehicleNode::PublishTf() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = x_m_;
  transform.transform.translation.y = y_m_;
  transform.transform.translation.z = 0.0;
  const double half_yaw = yaw_rad_ * 0.5;
  transform.transform.rotation.z = std::sin(half_yaw);
  transform.transform.rotation.w = std::cos(half_yaw);

  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(transform);
  tf_pub_->publish(message);
}

// 发布车体和四轮可视化。
void SimVehicleNode::PublishMarker() {
  visualization_msgs::msg::MarkerArray markers;
  AppendBodyMarkers(markers);
  if (steer_mode_ == 0 || steer_mode_ == 1) {
    AppendWheelMarkers(markers);
  }
  marker_pub_->publish(markers);
}

// 向 MarkerArray 添加绿色车体和黄色车头标识。
void SimVehicleNode::AppendBodyMarkers(
  visualization_msgs::msg::MarkerArray& markers) {
  const double body_center_x_m = steer_mode_ == 0 ? wheelbase_m_ * 0.5 : 0.0;

  auto body = CreateMarker(static_cast<int32_t>(kBodyMarkerId),
                           visualization_msgs::msg::Marker::CUBE);
  const auto [body_x_m, body_y_m] = LocalToWorld(body_center_x_m, 0.0);
  body.pose.position.x = body_x_m;
  body.pose.position.y = body_y_m;
  body.pose.position.z = wheel_radius_m_ + vehicle_height_m_ * 0.5;
  SetYawOrientation(body, yaw_rad_);
  body.scale.x = vehicle_length_m_;
  body.scale.y = vehicle_width_m_;
  body.scale.z = vehicle_height_m_;
  body.color.r = 0.12F;
  body.color.g = 0.62F;
  body.color.b = 0.24F;
  body.color.a = 1.0F;
  markers.markers.push_back(body);

  auto bumper =
    CreateMarker(kBumperMarkerId, visualization_msgs::msg::Marker::CUBE);
  const auto [bumper_x_m, bumper_y_m] =
    LocalToWorld(body_center_x_m + vehicle_length_m_ * 0.5 - 0.05, 0.0);
  bumper.pose.position.x = bumper_x_m;
  bumper.pose.position.y = bumper_y_m;
  bumper.pose.position.z = body.pose.position.z + vehicle_height_m_ * 0.5;
  SetYawOrientation(bumper, yaw_rad_);
  bumper.scale.x = 0.10;
  bumper.scale.y = vehicle_width_m_ * 0.86;
  bumper.scale.z = 0.10;
  bumper.color.r = 0.95F;
  bumper.color.g = 0.72F;
  bumper.color.b = 0.12F;
  bumper.color.a = 1.0F;
  markers.markers.push_back(bumper);
}

// 向 MarkerArray 添加四个随转向角变化的车轮圆柱。
void SimVehicleNode::AppendWheelMarkers(
  visualization_msgs::msg::MarkerArray& markers) {
  const auto positions =
    ComputeWheelPositions(steer_mode_, wheelbase_m_, wheel_track_m_);
  // 可视化时将车轮放到车体顶面之上，避免俯视遮挡。
  const double wheel_visual_z_m =
    wheel_radius_m_ + vehicle_height_m_ + wheel_radius_m_ * 0.05;
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    auto wheel = CreateMarker(static_cast<int32_t>(index + 1),
                              visualization_msgs::msg::Marker::CYLINDER);
    const auto [wheel_x_m, wheel_y_m] =
      LocalToWorld(positions[index].x_m, positions[index].y_m);
    wheel.pose.position.x = wheel_x_m;
    wheel.pose.position.y = wheel_y_m;
    wheel.pose.position.z = wheel_visual_z_m;
    SetWheelOrientation(wheel, yaw_rad_ + wheel_state_.steer_rad[index]);
    wheel.scale.x = wheel_radius_m_ * 2.0;
    wheel.scale.y = wheel_radius_m_ * 2.0;
    wheel.scale.z = std::max(wheel_radius_m_ * 0.48, 0.08);
    wheel.color.r = 0.75F;
    wheel.color.g = 0.75F;
    wheel.color.b = 0.75F;
    wheel.color.a = 1.0F;
    markers.markers.push_back(wheel);
  }
}

// 发布四轮转角、轮速和底盘控制状态。
void SimVehicleNode::PublishChassisState() {
  weeder_msgs::msg::ChassisState state;
  state.header.stamp = now();
  state.header.frame_id = "base_link";
  state.state01_valid = true;
  state.gear = cmd_gear_;
  state.steer_mode = steer_mode_;
  state.estop = cmd_estop_;
  state.release_brake = cmd_release_brake_;
  state.ctrl_mode = cmd_ctrl_mode_;
  state.speed_raw = static_cast<uint16_t>(std::clamp<std::int64_t>(
    static_cast<std::int64_t>(std::lround(speed_m_s_ * 1000.0)), 0, 65535));
  state.speed_m_s = static_cast<float>(speed_m_s_);
  state.wheel_track_cm = static_cast<uint8_t>(std::clamp<std::int64_t>(
    static_cast<std::int64_t>(std::lround(wheel_track_m_ * 100.0)), 0, 255));
  state.alive_counter = alive_counter_;
  ++alive_counter_;

  state.state02_valid = true;
  state.state03_valid = true;
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    state.motor_speed_rpm[index] =
      static_cast<uint16_t>(std::clamp<std::int64_t>(
        static_cast<std::int64_t>(
          std::lround(wheel_state_.wheel_rad_s[index] * 60.0 / (2.0 * kPi))),
        0, 65535));
    state.steer_angle_deg[index] =
      static_cast<float>(wheel_state_.steer_rad[index] * 180.0 / kPi);
  }
  chassis_state_pub_->publish(state);
}

// 低频重发 GNSS 原点和工作模式，便于晚加入节点接收。
void SimVehicleNode::PublishSlowTopics() {
  const rclcpp::Time stamp = now();
  sensor_msgs::msg::NavSatFix origin;
  origin.header.stamp = stamp;
  origin.header.frame_id = "gnss_link";
  origin.latitude = origin_lat_;
  origin.longitude = origin_lon_;
  origin.altitude = origin_alt_m_;
  origin.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  origin_pub_->publish(origin);

  weeder_msgs::msg::WorkMode mode;
  mode.header.stamp = stamp;
  if (unmanned_) {
    mode.mode = weeder_msgs::msg::WorkMode::MODE_UNMANNED;
    mode.mode_name = "unmanned";
  } else {
    mode.mode = weeder_msgs::msg::WorkMode::MODE_MANUAL;
    mode.mode_name = "manual";
  }
  work_mode_pub_->publish(mode);
}

// 返回当前转向模式对应的等效轴距。
double SimVehicleNode::EffectiveWheelbase() const {
  return steer_mode_ == 1 ? wheelbase_m_ * 0.5 : wheelbase_m_;
}

// 创建使用 odom 坐标系和公共显示属性的 Marker。
visualization_msgs::msg::Marker SimVehicleNode::CreateMarker(
  int32_t marker_id, int32_t marker_type) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = now();
  marker.ns = "sim_vehicle";
  marker.id = marker_id;
  marker.type = marker_type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  return marker;
}

// 将车辆局部坐标转换到 odom 平面坐标。
std::pair<double, double> SimVehicleNode::LocalToWorld(double local_x_m,
                                                       double local_y_m) const {
  const double cos_yaw = std::cos(yaw_rad_);
  const double sin_yaw = std::sin(yaw_rad_);
  return {
    x_m_ + local_x_m * cos_yaw - local_y_m * sin_yaw,
    y_m_ + local_x_m * sin_yaw + local_y_m * cos_yaw,
  };
}

// 设置仅绕 z 轴旋转的 Marker 四元数。
void SimVehicleNode::SetYawOrientation(visualization_msgs::msg::Marker& marker,
                                       double yaw_rad) {
  const double half_yaw = yaw_rad * 0.5;
  marker.pose.orientation.z = std::sin(half_yaw);
  marker.pose.orientation.w = std::cos(half_yaw);
}

// 设置车轮朝向，使圆柱轴线沿车轮轮轴方向。
void SimVehicleNode::SetWheelOrientation(
  visualization_msgs::msg::Marker& marker, double yaw_rad) {
  const double half_yaw = yaw_rad * 0.5;
  const double cos_half = std::cos(half_yaw);
  const double sin_half = std::sin(half_yaw);
  const double sqrt_half = std::sqrt(0.5);
  marker.pose.orientation.x = -sqrt_half * cos_half;
  marker.pose.orientation.y = -sqrt_half * sin_half;
  marker.pose.orientation.z = sqrt_half * sin_half;
  marker.pose.orientation.w = sqrt_half * cos_half;
}

}  // namespace sim_vehicle
