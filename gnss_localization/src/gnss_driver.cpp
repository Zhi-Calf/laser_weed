#include "gnss_localization/gnss_driver.hpp"

#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <limits>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "gnss_localization/logging.hpp"

namespace gnss_localization {

namespace {
constexpr double kDegToRad              = M_PI / 180.0;
constexpr double kVerticalAccuracyScale = 2.0;
constexpr double kVelocityStd           = 0.05;        // m/s (人工标定: 速度精度 < 0.05 m/s 1-sigma)
constexpr int64_t kGpsEpochUnixSec      = 315964800;   // 1980-01-06 00:00:00 UTC
constexpr double kSecondsPerWeek        = 604800.0;

// 标定参数的启动有效性校验。非有限值一律退回 0.0（未标定）并告警，
// 避免 NaN 让 odom 的位置和航向整体失效。
double SanitizeFiniteCalib(const char* name, double value) {
  if (std::isfinite(value)) {
    return value;
  }
  SPDLOG_WARN("{} 启动值不是有限数值 ({}, 可能来自旧版本写坏的 yaml): 退回 0.0 (未标定)", name, value);
  return 0.0;
}

// 校验正数参数，非法配置回退到安全默认值。
double PositiveOr(const char* name, double value, double fallback) {
  if (std::isfinite(value) && value > 0.0) {
    return value;
  }
  SPDLOG_WARN("{} 必须为正数 (收到 {}), 使用默认值 {}", name, value, fallback);
  return fallback;
}

}   // namespace

GnssDriver::GnssDriver(const rclcpp::NodeOptions& options) : Node("gnss_driver", options) {
  // 日志器先于参数读取: 顺序反了的话, 后面的参数打印进不了滚动文件。
  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  LogConfig log_config;
  log_config.logger_name         = "gnss_localization";
  log_config.log_file_name       = "gnss_localization.log";
  log_config.log_dir             = log_dir_;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb_) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count_);
  log_config.enable_console      = log_enable_console_;
  log_config.log_level           = log_level_;
  try {
    ConfigureLogging(log_config);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", e.what());
  }

  loadParameters();

  if (!serial_.open(serial_port_path_, baud_rate_)) {
    throw std::runtime_error("无法打开 GNSS 串口: " + serial_port_path_);
  }
  has_giavp_       = false;
  last_status_     = 0;
  last_giavp_time_ = std::chrono::steady_clock::now();
  SPDLOG_INFO("串口已打开: {}", serial_port_path_);

  rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS(rclcpp::KeepLast(5));
  fix_pub_               = create_publisher<sensor_msgs::msg::NavSatFix>("~/fix", qos_sensor);
  pose_pub_              = create_publisher<weeder_msgs::msg::Pose>("~/pose", qos_sensor);
  pose_stamped_pub_      = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_stamped", 10);
  odom_raw_pub_          = create_publisher<nav_msgs::msg::Odometry>("~/odom_raw", qos_sensor);
  imu_pub_               = create_publisher<sensor_msgs::msg::Imu>("~/imu", qos_sensor);
  vel_pub_               = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/vel", qos_sensor);
  origin_llh_pub_        = create_publisher<sensor_msgs::msg::NavSatFix>("~/origin", rclcpp::QoS(1).transient_local());
  tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  timer_                 = create_wall_timer(std::chrono::milliseconds(10), std::bind(&GnssDriver::timerCallback, this));

  SPDLOG_INFO("GNSS 驱动已启动, 100 Hz 轮询串口 {}", serial_port_path_);
}

GnssDriver::~GnssDriver() {
  timer_.reset();
  serial_.close();
}

// 根据串口、数据新鲜度和定位质量记录健康状态。
void GnssDriver::updateHealth() {
  if (!serial_.is_open()) {
    updateHealthState(HealthState::kSerial, "GNSS 串口已断开");
    return;
  }

  const double silent_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_giavp_time_).count();
  if (silent_seconds >= giavp_timeout_sec_) {
    updateHealthState(HealthState::kGiavpTimeout, "GIAVP 数据超时");
    return;
  }
  if (!has_giavp_) {
    updateHealthState(HealthState::kWaiting, "GNSS 串口已打开，等待 GIAVP 数据");
    return;
  }
  if (last_status_ != kStatusRtkFixed) {
    updateHealthState(HealthState::kLowStatus, "GNSS 解算状态低于 RTK 固定解");
    return;
  }
  updateHealthState(HealthState::kOk, "GNSS 定位正常，RTK 固定解");
}

// 健康状态变化时只记录一次日志，避免 100 Hz 定时器刷屏。
void GnssDriver::updateHealthState(HealthState state, const std::string& message) {
  if (last_health_state_ == state)
    return;
  last_health_state_ = state;
  if (state == HealthState::kOk || state == HealthState::kWaiting) {
    SPDLOG_INFO("{}", message);
  } else {
    SPDLOG_WARN("{}", message);
  }
}

// 判断当前帧是否具备可用于 ENU 投影的位置。
bool GnssDriver::positionUsable(const GiavpData& data) const {
  return data.status != 0 && std::isfinite(data.latitude_deg) && std::isfinite(data.longitude_deg) && std::isfinite(data.altitude_m);
}

void GnssDriver::loadLogParameters() {
  // spdlog 滚动日志参数
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 5);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");
  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

void GnssDriver::loadParameters() {
  declare_parameter("serial_port", "/dev/ttyS0");
  declare_parameter("baud_rate", 115200);
  declare_parameter("frame_id", "gnss_link");
  declare_parameter("origin_lat", std::numeric_limits<double>::quiet_NaN());
  declare_parameter("origin_lon", std::numeric_limits<double>::quiet_NaN());
  declare_parameter("origin_alt", std::numeric_limits<double>::quiet_NaN());
  declare_parameter("heading_std_deg", 0.2);
  declare_parameter("acc_rtk_fixed", 0.02);
  declare_parameter("acc_rtk_float", 0.3);
  declare_parameter("acc_rtd", 1.0);
  declare_parameter("acc_spp", 1.5);
  declare_parameter("calib_offset_x", 0.0);         // 杆臂 x (m, 车体系: 前 +x)
  declare_parameter("calib_offset_y", 0.0);         // 杆臂 y (m, 车体系: 左 +y)
  declare_parameter("calib_offset_heading", 0.0);   // 航向安装偏角 (rad)
  declare_parameter("debug_giavp", false);
  declare_parameter("gnss_log_interval", 500);
  declare_parameter("pose_log_interval", 10);
  declare_parameter("giavp_timeout_sec", 1.0);
  declare_parameter("use_gps_timestamp", true);
  declare_parameter("gps_utc_leap_seconds", 18);

  serial_port_path_       = get_parameter("serial_port").as_string();
  baud_rate_              = get_parameter("baud_rate").as_int();
  frame_id_               = get_parameter("frame_id").as_string();
  const double origin_lat = get_parameter("origin_lat").as_double();
  const double origin_lon = get_parameter("origin_lon").as_double();
  const double origin_alt = get_parameter("origin_alt").as_double();
  heading_std_deg_        = get_parameter("heading_std_deg").as_double();
  acc_rtk_fixed_          = get_parameter("acc_rtk_fixed").as_double();
  acc_rtk_float_          = get_parameter("acc_rtk_float").as_double();
  acc_rtd_                = get_parameter("acc_rtd").as_double();
  acc_spp_                = get_parameter("acc_spp").as_double();
  calib_offset_x_         = SanitizeFiniteCalib("calib_offset_x", get_parameter("calib_offset_x").as_double());
  calib_offset_y_         = SanitizeFiniteCalib("calib_offset_y", get_parameter("calib_offset_y").as_double());
  calib_offset_heading_   = SanitizeFiniteCalib("calib_offset_heading", get_parameter("calib_offset_heading").as_double());
  debug_giavp_            = get_parameter("debug_giavp").as_bool();
  gnss_log_interval_      = static_cast<int>(std::max<int64_t>(1, get_parameter("gnss_log_interval").as_int()));
  pose_log_interval_      = static_cast<int>(std::max<int64_t>(1, get_parameter("pose_log_interval").as_int()));
  giavp_timeout_sec_      = PositiveOr("giavp_timeout_sec", get_parameter("giavp_timeout_sec").as_double(), 1.0);
  use_gps_timestamp_      = get_parameter("use_gps_timestamp").as_bool();
  gps_utc_leap_seconds_   = static_cast<int>(std::max<int64_t>(0, get_parameter("gps_utc_leap_seconds").as_int()));

  // ---- ENU 原点: 优先使用 YAML 配置；未配置时沿用首次 RTK 固定解 ----
  const bool origin_any   = !std::isnan(origin_lat) || !std::isnan(origin_lon) || !std::isnan(origin_alt);
  const bool origin_valid = std::isfinite(origin_lat) && std::isfinite(origin_lon) && std::isfinite(origin_alt) && std::abs(origin_lat) <= 90.0 && std::abs(origin_lon) <= 180.0;
  if (origin_valid) {
    datum_lat_deg_     = origin_lat;
    datum_lon_deg_     = origin_lon;
    datum_alt_m_       = origin_alt;
    datum_proj_        = std::make_unique<GeographicLib::LocalCartesian>(origin_lat, origin_lon, origin_alt);
    datum_from_config_ = true;
    SPDLOG_INFO("ENU 原点已从 YAML 设置: 纬度={:.8f} 经度={:.8f} 高程={:.3f}", datum_lat_deg_, datum_lon_deg_, datum_alt_m_);
  } else if (origin_any) {
    SPDLOG_WARN("origin_lat/lon/alt 配置非法或不完整 (lat={} lon={} alt={}), 回退为首次 RTK 固定解", origin_lat, origin_lon, origin_alt);
  } else {
    SPDLOG_INFO("未配置 origin_lat/lon/alt, ENU 原点将在首次 RTK 固定解时设置");
  }

  // 详细打印所有参数
  SPDLOG_INFO("========== GNSS 驱动参数加载 ==========");
  SPDLOG_INFO("串口: {}, 波特率: {}, frame_id: {}", serial_port_path_, baud_rate_, frame_id_);
  SPDLOG_INFO("ENU 原点来源: {}", datum_from_config_ ? "YAML" : "首次 RTK 固定解");
  SPDLOG_INFO("航向: std={:.2f}°", heading_std_deg_);
  SPDLOG_INFO("位置精度 (m): 固定解={:.3f} 浮点解={:.3f} RTD={:.3f} 单点={:.3f}", acc_rtk_fixed_, acc_rtk_float_, acc_rtd_, acc_spp_);
  SPDLOG_INFO(
    "标定杆臂 (base_link->设备中心, 车体系): x={:.3f} y={:.3f} m, "
    "航向偏角={:.4f} rad",
    calib_offset_x_,
    calib_offset_y_,
    calib_offset_heading_);
  SPDLOG_INFO("debug_giavp: {}", debug_giavp_ ? "是" : "否");
  SPDLOG_INFO("GNSS 日志间隔: {} 条 (10Hz 下约 {:.1f}s), 位姿日志间隔: {} 条 (10Hz 下约 {:.1f}s)", gnss_log_interval_, gnss_log_interval_ / 10.0, pose_log_interval_, pose_log_interval_ / 10.0);
  SPDLOG_INFO("GIAVP 超时: {:.2f}s", giavp_timeout_sec_);
  SPDLOG_INFO("GPS 时间戳: {}, UTC 闰秒: {}s", use_gps_timestamp_ ? "启用" : "禁用", gps_utc_leap_seconds_);
  SPDLOG_INFO("日志级别: {}, 目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}", log_level_, log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "是" : "否");
  SPDLOG_INFO("=========================================");
}

// 定时器回调
void GnssDriver::timerCallback() {
  serial_.read_lines([this](const std::string& line) {
    processLine(line);
  });
  updateHealth();

  // ENU 起点按 1Hz 低频重发 (原点设置后): transient_local 缓存负责秒收,
  // 低频重发兜底 gnss_driver 重启后缓存丢失的场景。
  if (datum_proj_) {
    const auto now_t = now();
    if ((now_t - last_origin_publish_time_).seconds() >= 1.0) {
      publishOriginLlh();
      last_origin_publish_time_ = now_t;
    }
  }
}

// 处理一行数据
void GnssDriver::processLine(const std::string& line) {
  if (debug_giavp_) {
    std::string preview = line.size() > 120u ? line.substr(0, 120) + "..." : line;
    SPDLOG_DEBUG("收到原始报文: {}", preview);
  }

  if (!parser_.parse(line))
    return;

  last_giavp_time_ = std::chrono::steady_clock::now();
  has_giavp_       = true;
  last_status_     = parser_.data().status;

  handleGiavp(parser_.data());
}

// 处理 GIAVP 数据
void GnssDriver::handleGiavp(const GiavpData& d) {
  // 上电后首次进入 RTK 固定解时, 记录该点为 ENU 原点并发布。
  // datum_proj_ 是否为空即表示原点是否已设置。
  if (!datum_proj_ && d.status == kStatusRtkFixed) {
    // 设原点前校验经纬度合法性, 非法则跳过本帧不设原点, 等待下一帧固定解
    // (避免非法值传入 LocalCartesian 构造函数抛异常导致节点崩溃)。
    if (std::isfinite(d.latitude_deg) && std::isfinite(d.longitude_deg) && std::isfinite(d.altitude_m) && std::abs(d.latitude_deg) <= 90.0 && std::abs(d.longitude_deg) <= 180.0) {
      datum_lat_deg_ = d.latitude_deg;
      datum_lon_deg_ = d.longitude_deg;
      datum_alt_m_   = d.altitude_m;
      {
        std::lock_guard<std::mutex> lock(datum_mutex_);
        datum_proj_ = std::make_unique<GeographicLib::LocalCartesian>(d.latitude_deg, d.longitude_deg, d.altitude_m);
      }
      SPDLOG_INFO("ENU 原点已设置 (上电后首次固定解): 纬度={:.8f} 经度={:.8f} 高程={:.3f}", datum_lat_deg_, datum_lon_deg_, datum_alt_m_);
      publishOriginLlh();
      last_origin_publish_time_ = now();
    } else {
      SPDLOG_WARN("固定解经纬度非法, 跳过本帧不设原点: lat={} lon={} alt={}", d.latitude_deg, d.longitude_deg, d.altitude_m);
    }
  }

  publishFix(d);
  publishPose(d);
  publishImu(d);
  publishVelocity(d);
}

int GnssDriver::statusRank(int status) {
  // 质量排序: 0 (无) < 1 (SPP) < 2 (RTD) < 5 (RTK 浮点) < 4 (RTK 固定)
  switch (status) {
    case 0:
      return 0;
    case 1:
      return 1;
    case 2:
      return 2;
    case 5:
      return 3;
    case kStatusRtkFixed:
      return 4;
    default:
      return 0;
  }
}

double GnssDriver::positionAccuracyForStatus(int status) const {
  switch (status) {
    case kStatusRtkFixed:
      return acc_rtk_fixed_;
    case 5:
      return acc_rtk_float_;
    case 2:
      return acc_rtd_;
    case 1:
      return acc_spp_;
    default:
      return acc_spp_ * 10.0;   // 初始化/未知: 极低可信度
  }
}

// 把 GIAVP 的 GPS 周和周内秒转换为 ROS 时间，非法时回退到本机时间。
rclcpp::Time GnssDriver::timestampFor(const GiavpData& data) const {
  if (!use_gps_timestamp_ || data.gps_week <= 0 || !std::isfinite(data.gps_time_sec) || data.gps_time_sec < 0.0 || data.gps_time_sec >= kSecondsPerWeek) {
    return now();
  }

  const double unix_seconds = static_cast<double>(kGpsEpochUnixSec) + static_cast<double>(data.gps_week) * kSecondsPerWeek + data.gps_time_sec - static_cast<double>(gps_utc_leap_seconds_);
  if (!std::isfinite(unix_seconds) || unix_seconds <= 0.0) {
    return now();
  }

  int64_t seconds      = static_cast<int64_t>(std::floor(unix_seconds));
  uint32_t nanoseconds = static_cast<uint32_t>(std::llround((unix_seconds - static_cast<double>(seconds)) * 1e9));
  if (nanoseconds >= 1000000000U) {
    ++seconds;
    nanoseconds = 0;
  }
  return rclcpp::Time(seconds, nanoseconds, RCL_ROS_TIME);
}

// 发布ENU原点
void GnssDriver::publishOriginLlh() {
  std::lock_guard<std::mutex> lock(datum_mutex_);
  if (!datum_proj_ || !origin_llh_pub_) {
    return;
  }
  auto msg            = sensor_msgs::msg::NavSatFix();
  msg.header.stamp    = now();
  msg.header.frame_id = frame_id_;
  msg.latitude        = datum_lat_deg_;
  msg.longitude       = datum_lon_deg_;
  msg.altitude        = datum_alt_m_;
  msg.status.status   = datum_from_config_ ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;   // YAML 原点 / 首次固定解
  origin_llh_pub_->publish(msg);
}

// 发布经纬度数据
void GnssDriver::publishFix(const GiavpData& d) {
  auto msg            = sensor_msgs::msg::NavSatFix();
  msg.header.stamp    = timestampFor(d);
  msg.header.frame_id = frame_id_;

  switch (d.status) {
    case 0:   // 无解
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case 1:   // SPP
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case 2:   // RTD
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case kStatusRtkFixed:   // RTK 固定解 (4) — 唯一标 GBAS_FIX
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case 5:   // RTK 浮点解 (5) — 非固定, 不夸大精度
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    default:
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
  }
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // 坐标: 无解 (status==0) 时经纬高为残值 (可能为 0 或上一帧),
  // 发布 NaN 明确表示无效, 避免下游忽略 status 直接读坐标时拿到错误位置。
  if (d.status == 0) {
    msg.latitude  = std::numeric_limits<double>::quiet_NaN();
    msg.longitude = std::numeric_limits<double>::quiet_NaN();
    msg.altitude  = std::numeric_limits<double>::quiet_NaN();
  } else {
    msg.latitude  = d.latitude_deg;
    msg.longitude = d.longitude_deg;
    msg.altitude  = d.altitude_m;
  }

  const bool valid_position = positionUsable(d);
  double h_std              = valid_position ? positionAccuracyForStatus(d.status) : 1e3;
  double h_var              = h_std * h_std;
  double v_var              = std::pow(h_std * kVerticalAccuracyScale, 2);

  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.position_covariance[0]   = h_var;   // 东向方差
  msg.position_covariance[4]   = h_var;   // 北向方差
  msg.position_covariance[8]   = v_var;   // 天向方差

  fix_pub_->publish(msg);
}

// 发布修正位姿和仅供标定使用的原始里程计。
void GnssDriver::publishPose(const GiavpData& d) {
  if (!positionUsable(d))
    return;

  double dev_x = 0.0, dev_y = 0.0, dev_z = 0.0;
  // datum_proj_ 加锁为防御性保留 (当前读写均在 timer 回调线程, 无跨线程竞争)。
  {
    std::lock_guard<std::mutex> lock(datum_mutex_);
    if (!datum_proj_) {
      return;
    }
    datum_proj_->Forward(d.latitude_deg, d.longitude_deg, d.altitude_m, dev_x, dev_y, dev_z);
  }

  const double calib_x       = calib_offset_x_;
  const double calib_y       = calib_offset_y_;
  const double calib_heading = calib_offset_heading_;

  // 修正航向 = 测向航向 + 标定残差航向偏角。
  const double yaw_raw                 = headingToEnuYaw(d.heading_deg);
  const double corrected_yaw_unwrapped = yaw_raw + calib_heading;
  const double yaw_corrected           = std::atan2(std::sin(corrected_yaw_unwrapped), std::cos(corrected_yaw_unwrapped));

  // 设备中心 -> base_link 杆臂 (车体系 -> ENU)。杆臂只有一个来源: 标定值 calib_offset_*。
  // base = dev − R(yaw_corrected)·calib
  const double ct = std::cos(yaw_corrected);
  const double st = std::sin(yaw_corrected);
  Eigen::Matrix2d R;
  R << ct, -st, st, ct;   // 车体系 -> ENU 的旋转矩阵

  const Eigen::Vector2d calib(calib_x, calib_y);
  const Eigen::Vector2d dev(dev_x, dev_y);

  const Eigen::Vector2d base = dev - R * calib;
  const double base_x        = base.x();
  const double base_y        = base.y();
  // z 不做杆臂补偿 (标定只有 x/y/heading 三个量, 没有垂向杆臂): 直接用设备中心的高程。
  const double base_z = dev_z;

  // 位姿仅允许自动驾驶使用 RTK 固定解; 其他状态继续发布以便诊断。
  const bool rtk_fixed = (d.status == kStatusRtkFixed);
  const double h_std   = rtk_fixed ? acc_rtk_fixed_ : 1e3;   // 1e3 m -> 方差 1e6
  const double h_var   = h_std * h_std;
  const double yaw_var = std::pow(heading_std_deg_ * kDegToRad, 2);

  // 车体系速度: 把 ENU 速度投影到航向方向 (倒车得负的前进速度)。
  // 两套航向各算一次: 修正后的给 ~/pose, 原始的给 ~/odom_raw。
  const double forward_speed_from_enu = d.ve_ms * ct + d.vn_ms * st;
  const double forward_speed          = d.speed_status == 1 ? d.vehicle_speed_kmh / 3.6 : forward_speed_from_enu;
  const double forward_speed_raw      = d.speed_status == 1 ? d.vehicle_speed_kmh / 3.6 : d.ve_ms * std::cos(yaw_raw) + d.vn_ms * std::sin(yaw_raw);
  const double speed_var              = rtk_fixed ? (kVelocityStd * kVelocityStd) : 1e6;

  const auto stamp = timestampFor(d);

  // 原始里程计继续保留协方差，供标定节点判断 RTK 固定解。
  const auto fill_cov = [&](nav_msgs::msg::Odometry& m) {
    m.pose.covariance[0]   = h_var;
    m.pose.covariance[7]   = h_var;
    m.pose.covariance[14]  = rtk_fixed ? std::pow(h_std * kVerticalAccuracyScale, 2) : 1e6;   // 垂直精度约为水平 2 倍
    m.pose.covariance[21]  = 1e6;
    m.pose.covariance[28]  = 1e6;
    m.pose.covariance[35]  = rtk_fixed ? yaw_var : 1e6;
    m.twist.covariance[0]  = speed_var;
    m.twist.covariance[7]  = speed_var;
    m.twist.covariance[14] = 1e6;
    m.twist.covariance[35] = rtk_fixed ? std::pow(0.5 * kDegToRad, 2) : 1e6;
  };

  // ---- 修正位姿: base_link, 修正航向 ----
  {
    auto pose            = weeder_msgs::msg::Pose();
    pose.header.stamp    = stamp;
    pose.header.frame_id = "odom";
    pose.x               = base_x;
    pose.y               = base_y;
    pose.z               = base_z;
    pose.roll            = -d.roll_deg * kDegToRad;
    pose.pitch           = -d.pitch_deg * kDegToRad;
    pose.yaw             = yaw_corrected;
    pose.satellites      = static_cast<uint32_t>(std::max(d.nsv1, 0));
    pose.valid           = rtk_fixed;
    pose.speed_m_s       = forward_speed;

    pose_pub_->publish(pose);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header          = pose.header;
    pose_stamped.pose.position.x = base_x;
    pose_stamped.pose.position.y = base_y;
    pose_stamped.pose.position.z = base_z;
    tf2::Quaternion pose_quaternion;
    pose_quaternion.setRPY(pose.roll, pose.pitch, pose.yaw);
    pose_stamped.pose.orientation = tf2::toMsg(pose_quaternion);
    pose_stamped_pub_->publish(pose_stamped);

    if (++pose_log_counter_ >= pose_log_interval_) {
      pose_log_counter_ = 0;
      SPDLOG_INFO(
        "位姿: x={:.3f} y={:.3f} z={:.3f} yaw={:.1f}° 速度={:.2f}m/s "
        "卫星={} valid={}",
        base_x,
        base_y,
        base_z,
        yaw_corrected / kDegToRad,
        forward_speed,
        d.nsv1,
        rtk_fixed);
    }
  }

  if (++gnss_log_counter_ >= gnss_log_interval_) {
    gnss_log_counter_ = 0;
    SPDLOG_INFO(
      "GNSS: status={} 卫星={}/{} 基线={:.2f}m 纬度={:.6f} "
      "经度={:.6f} 高程={:.1f} 航向={:.1f}",
      d.status,
      d.nsv1,
      d.nsv2,
      d.baseline_m,
      d.latitude_deg,
      d.longitude_deg,
      d.altitude_m,
      d.heading_deg);
  }

  // ---- 原始里程计: 设备中心 + 原始测向航向 (标定用, 见 hpp 成员注释) ----
  // 位置 = dev (**设备中心, 不做任何杆臂/航向修正**); 航向 = 原始测向 (未加 calib_heading)。
  // 标定只需要这一份原始数据: 测出的杆臂/偏角就是绝对量, 直接写回 calib_offset_*。
  // 刻意不发 TF: TF 树只保留 odom->base_link 一套 (发布的是修正后的)。
  {
    auto raw                 = nav_msgs::msg::Odometry();
    raw.header.stamp         = stamp;
    raw.header.frame_id      = "odom";
    raw.child_frame_id       = "gnss_link";   // 设备中心 (与 GIAVP 输出位置一致)
    raw.pose.pose.position.x = dev_x;
    raw.pose.pose.position.y = dev_y;
    raw.pose.pose.position.z = dev_z;

    tf2::Quaternion q_raw;
    q_raw.setRPY(0.0, 0.0, yaw_raw);
    raw.pose.pose.orientation = tf2::toMsg(q_raw);

    raw.twist.twist.linear.x  = forward_speed_raw;
    raw.twist.twist.linear.y  = 0.0;
    raw.twist.twist.linear.z  = 0.0;
    raw.twist.twist.angular.z = d.gyro_z_dps * kDegToRad;
    fill_cov(raw);

    odom_raw_pub_->publish(raw);
  }

  // 实时广播 TF: odom -> base_link，与修正位姿一致。
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = stamp;
    tf.header.frame_id         = "odom";
    tf.child_frame_id          = "base_link";
    tf.transform.translation.x = base_x;
    tf.transform.translation.y = base_y;
    tf.transform.translation.z = base_z;
    tf2::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, yaw_corrected);
    tf.transform.rotation = tf2::toMsg(q_tf);
    tf_broadcaster_->sendTransform(tf);
  }
}

// 发布IMU数据
void GnssDriver::publishImu(const GiavpData& d) {
  auto msg            = sensor_msgs::msg::Imu();
  msg.header.stamp    = timestampFor(d);
  msg.header.frame_id = frame_id_;

  double yaw_rad = headingToEnuYaw(d.heading_deg);
  // 设备约定 Pitch+ 为车头抬起、Roll- 为车身向右倾；
  // ROS 右手系中 Pitch+ 为车头向下、Roll+ 为车身向右倾，因此二者取反。
  const double roll_rad  = -d.roll_deg * kDegToRad;
  const double pitch_rad = -d.pitch_deg * kDegToRad;

  // 四元数及其协方差。
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  msg.orientation               = tf2::toMsg(q);
  const bool valid_position     = positionUsable(d);
  double yaw_var                = valid_position ? std::pow(heading_std_deg_ * kDegToRad, 2) : 1e6;
  double rp_var                 = valid_position ? std::pow(0.2 * kDegToRad, 2) : 1e6;
  msg.orientation_covariance[0] = rp_var;
  msg.orientation_covariance[4] = rp_var;
  msg.orientation_covariance[8] = yaw_var;

  // 设备车体系: X 右, Y 前, Z 上。ROS 车体系: x 前, y 左, z 上。
  // 映射: ros_x = dev_y, ros_y = -dev_x, ros_z = dev_z。
  msg.angular_velocity.x                = d.gyro_y_dps * kDegToRad;
  msg.angular_velocity.y                = -d.gyro_x_dps * kDegToRad;
  msg.angular_velocity.z                = d.gyro_z_dps * kDegToRad;
  double gyro_var                       = std::pow(0.5 * kDegToRad, 2);
  msg.angular_velocity_covariance[0]    = gyro_var;
  msg.angular_velocity_covariance[4]    = gyro_var;
  msg.angular_velocity_covariance[8]    = gyro_var;
  msg.linear_acceleration.x             = d.acc_y;
  msg.linear_acceleration.y             = -d.acc_x;
  msg.linear_acceleration.z             = d.acc_z;
  double acc_var                        = valid_position ? 0.01 : 1e6;
  msg.linear_acceleration_covariance[0] = acc_var;
  msg.linear_acceleration_covariance[4] = acc_var;
  msg.linear_acceleration_covariance[8] = acc_var;

  if (!first_heading_logged_) {
    SPDLOG_INFO(
      "首个 GIAVP 航向: 原始={:.2f}° -> ENU 偏航角={:.1f}° "
      "(车头朝北约 90°; 朝东 0°; 朝南 -90°; 朝西 180°)",
      d.heading_deg,
      yaw_rad * 180.0 / M_PI);
    first_heading_logged_ = true;
  }

  imu_pub_->publish(msg);
}

// 发布车体线速度和角速度。
void GnssDriver::publishVelocity(const GiavpData& d) {
  auto msg            = geometry_msgs::msg::TwistWithCovarianceStamped();
  msg.header.stamp    = timestampFor(d);
  msg.header.frame_id = frame_id_;

  // ROS 车体系: x 前, y 左, z 上。
  // 纵向速度: 协议 VehicleSpeed 为有符号车体速度 (km/h, 倒车为负), 直接转 m/s。
  // 横向速度: 由 ENU 速度 (ve/vn) 经本帧航向旋转投影得到。
  // 垂向速度: 直接取 ENU 天向速度 vu。
  const double yaw = headingToEnuYaw(d.heading_deg);
  const double ct  = std::cos(yaw);
  const double st  = std::sin(yaw);

  const double vx_body = d.speed_status == 1 ? (d.vehicle_speed_kmh / 3.6)      // 协议车速 (带符号)
                                             : (d.ve_ms * ct + d.vn_ms * st);   // 无车速时回退 ENU 投影
  const double vy_body = -d.ve_ms * st + d.vn_ms * ct;

  msg.twist.twist.linear.x  = vx_body;   // 前进
  msg.twist.twist.linear.y  = vy_body;   // 向左
  msg.twist.twist.linear.z  = d.vu_ms;   // 向上
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = d.gyro_z_dps * kDegToRad;

  msg.twist.covariance.fill(0.0);
  const double speed_var   = positionUsable(d) ? kVelocityStd * kVelocityStd : 1e6;
  msg.twist.covariance[0]  = speed_var;   // vx
  msg.twist.covariance[7]  = speed_var;   // vy
  msg.twist.covariance[14] = speed_var;   // vz
  msg.twist.covariance[21] = 1e6;         // wx (未用)
  msg.twist.covariance[28] = 1e6;         // wy (未用)
  msg.twist.covariance[35] = positionUsable(d) ? std::pow(0.5 * kDegToRad, 2) : 1e6;

  vel_pub_->publish(msg);
}

// 转与X轴正方向夹角 (弧度)
double GnssDriver::headingToEnuYaw(double heading_deg) const {
  // 航向: 北起算, 顺时针为正 (度)。
  // ENU yaw: 东起算, 逆时针为正 (弧度)。
  double yaw_deg = 90.0 - heading_deg;
  double yaw_rad = yaw_deg * kDegToRad;
  while (yaw_rad > M_PI)
    yaw_rad -= 2.0 * M_PI;
  while (yaw_rad < -M_PI)
    yaw_rad += 2.0 * M_PI;
  return yaw_rad;
}

}   // namespace gnss_localization
