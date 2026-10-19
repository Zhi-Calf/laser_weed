#include "gnss_localization/gnss_driver.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <filesystem>
#include <limits>
#include <system_error>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "gnss_localization/fault_codes.hpp"
#include "yaml_persist.hpp"

namespace gnss_localization {

namespace {
constexpr double kDegToRad              = M_PI / 180.0;
constexpr double kVerticalAccuracyScale = 2.0;
constexpr double kVelocityStd           = 0.05;   // m/s (人工标定: 速度精度 < 0.05 m/s 1-sigma)

// 标定参数的**启动**有效性校验: yaml 里可能躺着旧版本写坏的 nan (那时 onSetParameters 的范围判断
// 拦不住 NaN, 见那里的说明)。非有限值一律退回 0.0 (未标定) 并告警 —— 比起带着 NaN 跑 (odom 的
// 位置/航向整体变 NaN, 且从数值上完全看不出来), 退回"未标定"是唯一诚实且可恢复的处置。
double SanitizeFiniteCalib(const char* name, double value) {
  if (std::isfinite(value)) {
    return value;
  }
  SPDLOG_WARN("{} 启动值不是有限数值 ({}, 可能来自旧版本写坏的 yaml): 退回 0.0 (未标定)", name, value);
  return 0.0;
}

// 扩展路径中的环境变量
std::string ExpandPath(std::string path) {
  if (path.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home) {
      path = std::string(home) + path.substr(1);
    }
  }
  std::string out;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i] != '$') {
      out += path[i];
      continue;
    }
    size_t j = i + 1;
    while (j < path.size() && (std::isalnum(static_cast<unsigned char>(path[j])) || path[j] == '_')) {
      ++j;
    }
    const std::string name = path.substr(i + 1, j - i - 1);
    const char* val        = name.empty() ? nullptr : std::getenv(name.c_str());
    if (val) {
      out += val;
    }
    i = j - 1;
  }
  return out;
}

// 解析日志目录
std::string ResolveLogDir(const std::string& configured) {
  std::error_code ec;
  const std::string preferred = ExpandPath(configured);
  if (!preferred.empty()) {
    std::filesystem::create_directories(preferred, ec);
    if (!ec && access(preferred.c_str(), W_OK) == 0) {
      return preferred;
    }
  }

  const char* ros_home = std::getenv("ROS_HOME");
  const char* home     = std::getenv("HOME");
  std::string fallback;
  if (ros_home && *ros_home) {
    fallback = std::string(ros_home) + "/logs";
  } else if (home && *home) {
    fallback = std::string(home) + "/.ros/logs";
  } else {
    return "";
  }

  std::filesystem::create_directories(fallback, ec);
  if (ec || access(fallback.c_str(), W_OK) != 0) {
    return "";
  }
  return fallback;
}

// 配置日志
void ConfigureLogger(const std::string& log_dir, size_t max_file_size_bytes, size_t max_file_count, bool enable_console, const std::string& log_level) {
  constexpr const char* kLogPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";
  const std::string dir             = ResolveLogDir(log_dir);
  if (dir.empty()) {
    throw std::runtime_error("no writable log directory (tried \"" + log_dir + "\")");
  }

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((std::filesystem::path(dir) / "gnss_localization.log").string(), max_file_size_bytes, max_file_count));
  if (enable_console) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>("gnss_localization", sinks.begin(), sinks.end());
  logger->set_pattern(kLogPattern);
  const spdlog::level::level_enum level = spdlog::level::from_str(log_level);
  if (level == spdlog::level::off && log_level != "off") {
    logger->set_level(spdlog::level::info);
    SPDLOG_WARN("无法识别的日志级别 \"{}\", 已回退到 info", log_level);
  } else {
    logger->set_level(level);
  }
  spdlog::set_default_logger(logger);
  if (dir != ExpandPath(log_dir)) {
    SPDLOG_WARN("日志目录 \"{}\" 不可写，已回退到 \"{}\"", log_dir, dir);
  }
  spdlog::flush_on(spdlog::level::warn);
  spdlog::flush_every(std::chrono::seconds(3));
}

}   // namespace

GnssDriver::GnssDriver(const rclcpp::NodeOptions& options) : Node("gnss_driver", options) {
  // 日志器先于参数读取: 顺序反了的话, 后面的参数打印进不了滚动文件。
  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  try {
    ConfigureLogger(log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024U * 1024U, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", e.what());
  }

  loadParameters();

  SPDLOG_INFO("正在打开串口 {}，波特率 {} (8N1)", serial_port_path_, baud_rate_);

  if (!serial_.open(serial_port_path_, baud_rate_)) {
    SPDLOG_ERROR("打开串口 {} 失败, 心跳将报告串口故障", serial_port_path_);
  }

  rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS(rclcpp::KeepLast(5));
  fix_pub_               = create_publisher<sensor_msgs::msg::NavSatFix>("~/fix", qos_sensor);
  odom_pub_              = create_publisher<nav_msgs::msg::Odometry>("~/odom", qos_sensor);
  odom_raw_pub_          = create_publisher<nav_msgs::msg::Odometry>("~/odom_raw", qos_sensor);
  imu_pub_               = create_publisher<sensor_msgs::msg::Imu>("~/imu", qos_sensor);
  vel_pub_               = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/vel", qos_sensor);
  status_pub_            = create_publisher<weeder_msgs::msg::GnssStatus>("~/status", qos_sensor);
  origin_llh_pub_        = create_publisher<sensor_msgs::msg::NavSatFix>("~/origin", rclcpp::QoS(1).transient_local());
  tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  timer_                 = create_wall_timer(std::chrono::milliseconds(10), std::bind(&GnssDriver::timerCallback, this));
  params_cb_handle_      = add_on_set_parameters_callback(std::bind(&GnssDriver::onSetParameters, this, std::placeholders::_1));

  SPDLOG_INFO("GNSS 驱动已启动, 100 Hz 轮询串口 {}", serial_port_path_);
}

GnssDriver::~GnssDriver() {
  timer_.reset();
  serial_.close();
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
  declare_parameter("heading_std_deg", 0.2);
  declare_parameter("acc_rtk_fixed", 0.02);
  declare_parameter("acc_rtk_float", 0.3);
  declare_parameter("acc_rtd", 1.0);
  declare_parameter("acc_spp", 1.5);
  declare_parameter("calib_offset_x", 0.0);             // 杆臂 x (m, 车体系: 前 +x)
  declare_parameter("calib_offset_y", 0.0);             // 杆臂 y (m, 车体系: 左 +y)
  declare_parameter("calib_offset_heading", 0.0);       // 航向安装偏角 (rad)
  declare_parameter("calib_offset_max_xy", 2.5);        // calib_offset_x/y 范围上限 (m, 绝对杆臂)
  declare_parameter("calib_offset_max_heading", 0.6);   // calib_offset_heading 范围上限 (rad, ≈34°)
  declare_parameter("gnss_config_path", "");
  declare_parameter("debug_giavp", false);
  declare_parameter("status_log_interval", 500);
  declare_parameter("odom_log_interval", 10);

  serial_port_path_         = get_parameter("serial_port").as_string();
  baud_rate_                = get_parameter("baud_rate").as_int();
  frame_id_                 = get_parameter("frame_id").as_string();
  heading_std_deg_          = get_parameter("heading_std_deg").as_double();
  acc_rtk_fixed_            = get_parameter("acc_rtk_fixed").as_double();
  acc_rtk_float_            = get_parameter("acc_rtk_float").as_double();
  acc_rtd_                  = get_parameter("acc_rtd").as_double();
  acc_spp_                  = get_parameter("acc_spp").as_double();
  calib_offset_x_           = SanitizeFiniteCalib("calib_offset_x", get_parameter("calib_offset_x").as_double());
  calib_offset_y_           = SanitizeFiniteCalib("calib_offset_y", get_parameter("calib_offset_y").as_double());
  calib_offset_heading_     = SanitizeFiniteCalib("calib_offset_heading", get_parameter("calib_offset_heading").as_double());
  calib_offset_max_xy_      = std::max(0.0, get_parameter("calib_offset_max_xy").as_double());
  calib_offset_max_heading_ = std::max(0.0, get_parameter("calib_offset_max_heading").as_double());
  gnss_config_path_         = get_parameter("gnss_config_path").as_string();
  debug_giavp_              = get_parameter("debug_giavp").as_bool();
  status_log_interval_      = static_cast<int>(std::max<int64_t>(1, get_parameter("status_log_interval").as_int()));
  odom_log_interval_        = static_cast<int>(std::max<int64_t>(1, get_parameter("odom_log_interval").as_int()));

  // 详细打印所有参数
  SPDLOG_INFO("========== GNSS 驱动参数加载 ==========");
  SPDLOG_INFO("串口: {}, 波特率: {}, frame_id: {}", serial_port_path_, baud_rate_, frame_id_);
  SPDLOG_INFO("航向: std={:.2f}°", heading_std_deg_);
  SPDLOG_INFO("位置精度 (m): 固定解={:.3f} 浮点解={:.3f} RTD={:.3f} 单点={:.3f}", acc_rtk_fixed_, acc_rtk_float_, acc_rtd_, acc_spp_);
  SPDLOG_INFO(
    "标定杆臂 (base_link->设备中心, 车体系): x={:.3f} y={:.3f} m, 航向偏角={:.4f} rad (范围: xy≤{:.2f}m 航向≤{:.2f}rad)",
    calib_offset_x_,
    calib_offset_y_,
    calib_offset_heading_,
    calib_offset_max_xy_,
    calib_offset_max_heading_);
  SPDLOG_INFO("参数持久化路径: {}", gnss_config_path_.empty() ? "(未配置, 跳过写回)" : gnss_config_path_);
  SPDLOG_INFO("debug_giavp: {}", debug_giavp_ ? "是" : "否");
  SPDLOG_INFO("状态日志间隔: {} 条 (10Hz 下约 {:.1f}s), 里程计日志间隔: {} 条 (10Hz 下约 {:.1f}s)", status_log_interval_, status_log_interval_ / 10.0, odom_log_interval_, odom_log_interval_ / 10.0);
  SPDLOG_INFO("日志级别: {}, 目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}", log_level_, log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "是" : "否");
  SPDLOG_INFO("=========================================");
}

// 更新参数回调
rcl_interfaces::msg::SetParametersResult GnssDriver::onSetParameters(const std::vector<rclcpp::Parameter>& parameters) {
  // ---- 第一轮: 校验 ----
  for (const auto& p : parameters) {
    const std::string& name = p.get_name();
    if (name == "calib_offset_x" || name == "calib_offset_y") {
      const double v = p.as_double();
      if (!std::isfinite(v)) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason     = name + " 必须是有限数值, 收到 " + p.value_to_string() + " (杆臂不接受 nan/inf)";
        SPDLOG_WARN("标定参数拒绝: {} = {} 不是有限数值", name, p.value_to_string());
        return result;
      }
      if (std::abs(v) > calib_offset_max_xy_) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason     = name + " 超出范围 [-" + std::to_string(calib_offset_max_xy_) + ", " + std::to_string(calib_offset_max_xy_) + "] m";
        SPDLOG_WARN("标定参数拒绝: {} = {} 超出 ±{} m", name, v, calib_offset_max_xy_);
        return result;
      }
    } else if (name == "calib_offset_heading") {
      const double v = p.as_double();
      // 同 calib_offset_x/y: 先判有限, 再判范围 (NaN 会让 odom 的航向修正整体变 NaN)。
      if (!std::isfinite(v)) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason     = name + " 必须是有限数值, 收到 " + p.value_to_string() + " (安装偏角不接受 nan/inf)";
        SPDLOG_WARN("标定参数拒绝: {} = {} 不是有限数值", name, p.value_to_string());
        return result;
      }
      if (std::abs(v) > calib_offset_max_heading_) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason     = name + " 超出范围 [-" + std::to_string(calib_offset_max_heading_) + ", " + std::to_string(calib_offset_max_heading_) + "] rad";
        SPDLOG_WARN("标定参数拒绝: {} = {} 超出 ±{} rad", name, v, calib_offset_max_heading_);
        return result;
      }
    }
  }

  // ---- 第二轮: 应用 (改内存) ----
  // 只持 calib_mutex_ 改内存; 落盘放在锁外 —— publishOdom 每帧都要拿这把锁, 文件 I/O
  // (含 fsync) 不该拖住 10 Hz 的里程计发布。
  std::vector<std::pair<std::string, double>> updates;
  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    for (const auto& p : parameters) {
      const std::string& name = p.get_name();
      if (name == "calib_offset_x") {
        calib_offset_x_ = p.as_double();
        SPDLOG_INFO("标定参数更新: calib_offset_x = {:.6f}", calib_offset_x_);
        updates.emplace_back("calib_offset_x", calib_offset_x_);
      } else if (name == "calib_offset_y") {
        calib_offset_y_ = p.as_double();
        SPDLOG_INFO("标定参数更新: calib_offset_y = {:.6f}", calib_offset_y_);
        updates.emplace_back("calib_offset_y", calib_offset_y_);
      } else if (name == "calib_offset_heading") {
        calib_offset_heading_ = p.as_double();
        SPDLOG_INFO("标定参数更新: calib_offset_heading = {:.6f}", calib_offset_heading_);
        updates.emplace_back("calib_offset_heading", calib_offset_heading_);
      }
    }
  }

  // ---- 第三轮: 落盘 (锁外) ----
  PersistOutcome outcome = PersistOutcome::kPersisted;
  if (!updates.empty()) {
    outcome = persistParamsToYaml(updates);
  }

  // 参数已在内存生效, 所以 successful 一律为 true: 若因落盘失败回 false, 调用方
  // (tractor_calibration) 会按"未生效"回滚自己的 yaml, 反而变成"文件旧值 / 车上新值"
  // 的不一致。落盘失败/未配置路径改用 reason 明确告知, 由调用方显示给现场。
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  switch (outcome) {
    case PersistOutcome::kSkippedNoPath:
      result.reason = "updated (gnss_config_path 未配置: 仅内存生效, 重启后会丢失)";
      SPDLOG_WARN("标定参数已生效, 但 gnss_config_path 未配置, 重启后会丢失");
      break;
    case PersistOutcome::kFailed:
      result.reason = "updated (警告: 写入 " + gnss_config_path_ + " 失败, 重启后会丢失; 参数当前已生效)";
      SPDLOG_ERROR("标定参数已生效, 但持久化到 {} 失败: 重启后会丢失", gnss_config_path_);
      break;
    default:
      result.reason = "updated";
      break;
  }
  return result;
}

// 把参数写回 yaml: 路径为空/日志/结果映射在这一层, 文本替换与原子落盘的规则见 src/yaml_persist.hpp
// (与 tractor_calibration 的 persistCalibToYaml 同一套语义 —— 现场同一份 yaml 可能被多个节点写,
// 两边的分层与文案保持一致才好排查)。
GnssDriver::PersistOutcome GnssDriver::persistParamsToYaml(const std::vector<std::pair<std::string, double>>& updates) {
  // 没有要写回的参数: 跳过
  if (updates.empty()) {
    return PersistOutcome::kPersisted;   // 没有要写的键
  }
  if (gnss_config_path_.empty()) {
    return PersistOutcome::kSkippedNoPath;   // 部署时未配置持久化路径 (文档化的行为)
  }

  // 写回实现按字符串值处理 (与 tractor_calibration / global_path_planner 共用同一套规则), 这里把
  // double 转成字符串: std::to_string 与标定侧的 FormatDouble 同为固定 6 位小数, 写进 yaml 的格式一致。
  std::vector<std::pair<std::string, std::string>> text_updates;
  text_updates.reserve(updates.size());
  for (const auto& [key, value] : updates) {
    text_updates.emplace_back(key, std::to_string(value));
  }

  YamlUpdateReport report;
  const YamlPersistStatus status = PersistYamlKeys(gnss_config_path_, text_updates, &report);
  if (report.duplicate_keys > 0) {
    SPDLOG_WARN("{} 中存在重复键 ({} 处, 已全部替换为同一值): YAML 解析器对重复键取最后一个, 建议手工清理重复行", gnss_config_path_, report.duplicate_keys);
  }
  if (!report.mapping_found) {
    SPDLOG_WARN("{} 里没找到 ros__parameters 映射, 新增键只能追加到文件末尾 (可能不生效)", gnss_config_path_);
  }
  if (report.dir_sync_failed) {
    SPDLOG_WARN("{} 的父目录 fsync 失败 (本次写入已生效, 但掉电后可能丢这次 rename)", gnss_config_path_);
  }
  switch (status) {
    case YamlPersistStatus::kUnchanged:
      SPDLOG_INFO("参数值与 {} 中已有内容一致, 未重写文件", gnss_config_path_);
      return PersistOutcome::kPersisted;
    case YamlPersistStatus::kReadFailed:
      SPDLOG_WARN("无法读取 gnss 配置文件 {} (参数未写回)", gnss_config_path_);
      return PersistOutcome::kFailed;
    case YamlPersistStatus::kWriteFailed:
      SPDLOG_ERROR("写入 {} 失败 (参数未写回)", gnss_config_path_);
      return PersistOutcome::kFailed;
    case YamlPersistStatus::kPersisted:
    default:
      SPDLOG_INFO("标定参数已持久化到 {} (逐行写入, 保留注释, 原子替换)", gnss_config_path_);
      return PersistOutcome::kPersisted;
  }
}

// 定时器回调
void GnssDriver::timerCallback() {
  serial_.read_lines([this](const std::string& line) {
    processLine(line);
  });

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
  publishOdom(d);
  publishImu(d);
  publishVelocity(d);
  publishStatus(d);
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
  msg.status.status   = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;   // ENU 原点来自 RTK 固定解
  origin_llh_pub_->publish(msg);
}

// 发布经纬度数据
void GnssDriver::publishFix(const GiavpData& d) {
  auto msg            = sensor_msgs::msg::NavSatFix();
  msg.header.stamp    = now();
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
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
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

  double h_std = positionAccuracyForStatus(d.status);
  double h_var = h_std * h_std;
  double v_var = std::pow(h_std * kVerticalAccuracyScale, 2);

  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.position_covariance[0]   = h_var;   // east
  msg.position_covariance[4]   = h_var;   // north
  msg.position_covariance[8]   = v_var;   // up

  fix_pub_->publish(msg);
}

// 发布里程计数据
void GnssDriver::publishOdom(const GiavpData& d) {
  double dev_x = 0.0, dev_y = 0.0, dev_z = 0.0;
  // datum_proj_ 加锁为防御性保留 (当前读写均在 timer 回调线程, 无跨线程竞争)。
  {
    std::lock_guard<std::mutex> lock(datum_mutex_);
    if (!datum_proj_) {
      return;
    }
    datum_proj_->Forward(d.latitude_deg, d.longitude_deg, d.altitude_m, dev_x, dev_y, dev_z);
  }

  // 读取标定值 (参数回调线程 vs 定时器线程, 真跨线程, 必须加锁)。
  double calib_x, calib_y, calib_heading;
  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    calib_x       = calib_offset_x_;
    calib_y       = calib_offset_y_;
    calib_heading = calib_offset_heading_;
  }

  // 修正航向 = 测向航向 + 标定残差航向偏角。
  const double yaw_raw       = headingToEnuYaw(d.heading_deg);
  const double yaw_corrected = yaw_raw + calib_heading;

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

  // 位置精度 -> 协方差。
  // 里程计仅供自动驾驶跟踪使用, 仅在 RTK 固定解 (status==4) 时可信;
  // 其余状态 (浮点/RTD/SPP/无解) 把协方差设为极大值, 下游按阈值即可判断不可用。
  const bool rtk_fixed = (d.status == kStatusRtkFixed);
  const double h_std   = rtk_fixed ? acc_rtk_fixed_ : 1e3;   // 1e3 m -> 方差 1e6
  const double h_var   = h_std * h_std;
  const double yaw_var = std::pow(heading_std_deg_ * kDegToRad, 2);

  // 车体系速度: 把 ENU 速度投影到航向方向 (倒车得负的前进速度)。
  // 两套航向各算一次: 修正后的给 ~/odom, 原始的给 ~/odom_raw。
  const double forward_speed     = d.ve_ms * ct + d.vn_ms * st;
  const double forward_speed_raw = d.ve_ms * std::cos(yaw_raw) + d.vn_ms * std::sin(yaw_raw);
  const double speed_var         = rtk_fixed ? (kVelocityStd * kVelocityStd) : 1e6;

  const auto stamp = now();

  // 两套里程计共用同一套协方差规则: 标定节点判"RTK 固定解"只看这几个对角元
  // (calibration_manager 的 kCovInvalidThreshold), 两边必须一致, 否则同一帧数据会
  // 在不同话题上有不同的可用性判定。
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

  // ---- 修正里程计: base_link, 修正航向 ----
  {
    auto odom            = nav_msgs::msg::Odometry();
    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.position.x = base_x;
    odom.pose.pose.position.y = base_y;
    odom.pose.pose.position.z = base_z;

    tf2::Quaternion q_corr;
    q_corr.setRPY(0.0, 0.0, yaw_corrected);
    odom.pose.pose.orientation = tf2::toMsg(q_corr);

    odom.twist.twist.linear.x  = forward_speed;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.linear.z  = 0.0;
    odom.twist.twist.angular.z = d.gyro_z_dps * kDegToRad;
    fill_cov(odom);

    odom_pub_->publish(odom);

    // 按条数间隔打印修正里程计 (base_link 系, 含杆臂/标定/航向修正)。
    if (++odom_log_counter_ >= odom_log_interval_) {
      odom_log_counter_ = 0;
      SPDLOG_INFO(
        "里程计: x={:.3f} y={:.3f} z={:.3f} 航向={:.1f}° 前进速度={:.2f}m/s 角速度={:.1f}°/s {}",
        base_x,
        base_y,
        base_z,
        yaw_corrected / kDegToRad,
        forward_speed,
        d.gyro_z_dps,
        rtk_fixed ? "固定解" : "非固定解");
    }
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

  // 实时广播 TF: odom -> base_link (杆臂 + 标定残差 + 航向偏角组合, 与修正里程计一致)。
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
  msg.header.stamp    = now();
  msg.header.frame_id = frame_id_;

  double yaw_rad   = headingToEnuYaw(d.heading_deg);
  double roll_rad  = d.roll_deg * kDegToRad;
  double pitch_rad = d.pitch_deg * kDegToRad;

  // 四元数及其协方差。
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  msg.orientation               = tf2::toMsg(q);
  double yaw_var                = std::pow(heading_std_deg_ * kDegToRad, 2);
  double rp_var                 = std::pow(0.2 * kDegToRad, 2);   // 人工标定: pitch/roll 0.2 deg (1-sigma)
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
  double acc_var                        = 0.01;
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
  msg.header.stamp    = now();
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
  double speed_var         = kVelocityStd * kVelocityStd;
  msg.twist.covariance[0]  = speed_var;   // vx
  msg.twist.covariance[7]  = speed_var;   // vy
  msg.twist.covariance[14] = speed_var;   // vz
  msg.twist.covariance[21] = 1e6;         // wx (未用)
  msg.twist.covariance[28] = 1e6;         // wy (未用)
  msg.twist.covariance[35] = std::pow(0.5 * kDegToRad, 2);

  vel_pub_->publish(msg);
}

// 发布GNSS状态信息
void GnssDriver::publishStatus(const GiavpData& d) {
  auto msg            = weeder_msgs::msg::GnssStatus();
  msg.header.stamp    = now();
  msg.header.frame_id = frame_id_;

  msg.fix_type       = static_cast<uint8_t>(d.status);
  msg.num_satellites = static_cast<uint8_t>(d.nsv1);
  msg.hdop           = 0.0f;   // GIAVP 中无此数据
  msg.pdop           = 0.0f;
  msg.vdop           = 0.0f;
  msg.diff_age       = 0.0f;
  msg.latitude_deg   = d.latitude_deg;
  msg.longitude_deg  = d.longitude_deg;
  msg.altitude_m     = static_cast<float>(d.altitude_m);
  msg.heading_deg    = static_cast<float>(d.heading_deg);

  if (d.speed_status == 1) {
    // 协议车速带符号 (倒车为负), 直接使用。
    msg.speed_kmh = static_cast<float>(d.vehicle_speed_kmh);
  } else {
    // 无车速时回退: ENU 速度投影到航向方向, 保留前进/倒退符号。
    const double yaw = headingToEnuYaw(d.heading_deg);
    const double fwd = d.ve_ms * std::cos(yaw) + d.vn_ms * std::sin(yaw);
    msg.speed_kmh    = static_cast<float>(fwd * 3.6);
  }

  status_pub_->publish(msg);

  if (++status_log_counter_ >= status_log_interval_) {
    status_log_counter_ = 0;
    SPDLOG_INFO("状态={} 卫星={}/{} 基线={:.2f}m 纬度={:.6f} 经度={:.6f} 高程={:.1f} 航向={:.1f}", d.status, d.nsv1, d.nsv2, d.baseline_m, d.latitude_deg, d.longitude_deg, d.altitude_m, d.heading_deg);
  }
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
