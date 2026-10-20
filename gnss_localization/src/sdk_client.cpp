#include "gnss_localization/sdk_client.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <system_error>

#include "gnss_localization/fault_codes.hpp"
#include "gnss_localization/serial_port.hpp"
#include "qxwz_sdk.h"

namespace gnss_localization {

namespace {

// SDK 的 C 回调无法捕获 this, 用一个全局指针桥接到当前实例。
// (千寻 SDK 为进程内单实例, 全局单例指针足够)。
SdkClient* g_sdk_instance = nullptr;

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
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((std::filesystem::path(dir) / "sdk_client.log").string(), max_file_size_bytes, max_file_count));
  if (enable_console) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>("sdk_client", sinks.begin(), sinks.end());
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

// SDK 认证回调
void sdkAuthCallback(qxwz_int32_t status_code, qxwz_sdk_cap_info_t* cap_info) {
  if (!g_sdk_instance) {
    return;
  }
  if (status_code == QXWZ_SDK_STAT_AUTH_SUCC || status_code == QXWZ_SDK_STAT_LOCAL_AUTH_SUCC) {
    g_sdk_instance->sdk_auth_ready_.store(true);
    SPDLOG_INFO("千寻 SDK 认证成功, 能力数: {}", cap_info ? cap_info->caps_num : 0);
  } else {
    // 认证失败是致命错误: 写入错误码, 由主循环结束会话并上报 FAULT_SDK_ERROR。
    SPDLOG_ERROR("千寻 SDK 认证失败, code: {}", status_code);
    g_sdk_instance->sdk_error_code_.store(status_code);
  }
}

// SDK 启动回调
void sdkStartCallback(qxwz_int32_t status_code, qxwz_uint32_t cap_id) {
  if (!g_sdk_instance) {
    return;
  }
  if (status_code == QXWZ_SDK_STAT_CAP_START_SUCC || status_code == QXWZ_SDK_STAT_CAP_RESTART_SUCC) {
    SPDLOG_INFO("千寻 SDK start 成功: cap_id={}", cap_id);
  } else {
    // 启动能力异步失败会导致永远收不到 RTCM, 属致命错误。
    // 单独用 sdk_nosr_start_failed_ 标志, 与"认证失败/运行中错误"区分, 避免语义混淆。
    SPDLOG_ERROR("千寻 SDK start 失败: code={}, cap_id={}", status_code, cap_id);
    g_sdk_instance->sdk_nosr_start_failed_.store(true);
  }
}

// SDK 能力状态回调: 1.29 SDK 新增。能力级错误不一定走 status_cb,
// 这里把能力服务端异常和长时间无差分数据纳入现有会话健康判定。
void sdkCapStatusCallback(qxwz_uint32_t cap_id, qxwz_int32_t status_code) {
  if (!g_sdk_instance) {
    return;
  }
  if (status_code == QXWZ_SDK_ERR_CAP_DIFF_DATA_TIME_OUT || status_code == QXWZ_SDK_ERR_CAP_DATA_PENDING ||
      status_code == QXWZ_SDK_ERR_RTCM_DATA_EXCEPTION) {
    const int n = g_sdk_instance->transient_error_count_.fetch_add(1) + 1;
    SPDLOG_WARN("千寻 SDK 能力 {} 一分钟无差分数据 (code={}), 累计 {}", cap_id, status_code, n);
    return;
  }
  if (status_code < 0) {
    SPDLOG_ERROR("千寻 SDK 能力 {} 异常: code={}", cap_id, status_code);
    g_sdk_instance->sdk_error_code_.store(status_code);
    return;
  }
  SPDLOG_DEBUG("千寻 SDK 能力 {} 状态: code={}", cap_id, status_code);
}

// 判断 status 回调码是否为"进行中/非错误"的瞬时提示 (不应判为故障)。
// SDK 约定: >= 0 为状态/进度码 (0..999); 负数为错误码; 其中少数负码表示"仍在进行中"。
bool IsTransientStatus(qxwz_int32_t code) {
  if (code >= 0) {
    return true;   // 状态/进度提示
  }
  switch (code) {
    case QXWZ_SDK_ERR_AUTHING:             // -7   认证进行中
    case QXWZ_SDK_ERR_STARTING:            // -8   启动进行中
    case QXWZ_SDK_ERR_GETTING_COORD_SYS:   // -12  正在获取坐标系
    case QXWZ_SDK_ERR_SETTING_COORD_SYS:   // -13  正在设置坐标系
    case QXWZ_SDK_ERR_ACTIVATING:          // -19  激活进行中
    case QXWZ_SDK_ERR_RESUMING_DSK:        // -20  恢复 DSK 中
    case QXWZ_SDK_ERR_IN_PROGRESS:         // -21  通用"进行中"
    case QXWZ_SDK_ERR_API_CALLED_TOO_FAST: // -30  调用过快, 稍后重试
    case QXWZ_SDK_ERR_DEC_INCOMP_DATA:     // -611 数据不完整 (等待后续数据, 非致命)
      return true;
    default:
      return false;
  }
}

// 判断 status 回调码是否为 SDK 可自行恢复的网络/数据层瞬时错误。
// 这些错误不影响 SDK 整体状态, 仅记日志, 不结束会话 (让 SDK 内部重试)。
// 注意: 不包含 QXWZ_SDK_ERR_FAIL(-1)。该码语义过宽 (通用失败), 无法判定为
// 可恢复, 宁可上报故障触发重连, 也不静默吞掉潜在致命错误。
bool IsRetryableStatus(qxwz_int32_t code) {
  switch (code) {
    case QXWZ_SDK_ERR_NETWORK_UNAVAILABLE:   // -101 网络暂不可用 (SDK 内部重试)
    case QXWZ_SDK_ERR_DEC_INVALID_DATA:      // -602 单帧数据非法
    case QXWZ_SDK_ERR_DEC_CRC_CHECK_FAIL:    // -604 单帧 CRC 失败
    case QXWZ_SDK_ERR_DEC_XOR_CHECK_FAIL:    // -606 单帧 XOR 校验失败
    case QXWZ_SDK_ERR_DEC_UNKNOWN:           // -700 解码未知错误
    case QXWZ_SDK_ERR_CAP_DATA_PENDING:      // -220 差分数据准备中
    case QXWZ_SDK_ERR_RTCM_DATA_EXCEPTION:   // -511 NOSR 数据异常, SDK 可重试
      return true;
    default:
      return false;
  }
}

// SDK 状态回调
void sdkStatusCallback(qxwz_int32_t status_code) {
  if (!g_sdk_instance) {
    return;
  }
  if (IsTransientStatus(status_code)) {
    SPDLOG_DEBUG("千寻 SDK status: code={}", status_code);
    return;
  }
  if (IsRetryableStatus(status_code)) {
    // 网络/数据层瞬时错误, SDK 内部会重试, 不立即结束会话; 但累计计数,
    // 由 runSession 主循环判定"持续高频错误 = 差分链路质量持续差"后重建会话。
    const int n = g_sdk_instance->transient_error_count_.fetch_add(1) + 1;
    SPDLOG_WARN("千寻 SDK 瞬时错误 (可恢复): code={}, 累计 {}", status_code, n);
    return;
  }
  // 其余均为 SDK 致命错误: 能力(-201..-219)、认证(-301..-317)、
  // 系统(-505..-509)、解码(-601..-610 去掉非致命码)、计费策略(-801..-802)、NOSR 数据格式(-921..-923) 等。
  // 这些无法自动恢复, 上报故障结束会话触发重连。
  SPDLOG_ERROR("千寻 SDK 内部错误: code={}", status_code);
  g_sdk_instance->sdk_error_code_.store(status_code);
}

// SDK NOSR 坐标框架/历元设置回调
void sdkSetNosrCoordCallback(qxwz_int32_t status_code) {
  SPDLOG_INFO("千寻 SDK set_nosr_coord_epoch_config 回调: code={}", status_code);
  if (g_sdk_instance) {
    g_sdk_instance->sdk_coordsys_code_.store(status_code);
    g_sdk_instance->sdk_coordsys_done_.store(true);
  }
}

// SDK 数据回调
void sdkDataCallback(qxwz_sdk_data_type_e type, const qxwz_void_t* data, qxwz_uint32_t len) {
  if (!g_sdk_instance || type != QXWZ_SDK_DATA_TYPE_RAW_NOSR) {
    return;
  }
  if (len == 0) {
    return;
  }
  const auto* buffer = static_cast<const uint8_t*>(data);

  // RTCM 数据直接透传给接收机 (不做 CRC 校验: SDK 已保证数据完整, 接收机自行做帧同步与 CRC)。
  SPDLOG_DEBUG("收到 RTCM, len={}", len);
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  g_sdk_instance->last_rtcm_ms_.store(now_ms);
  if (g_sdk_instance->writeRtcm(buffer, len)) {
    // 写成功: 清零连续失败计数, 供主循环做"收 RTCM 但写不进串口"的故障判定。
    g_sdk_instance->consecutive_write_fail_.store(0);
  } else {
    // 写失败: 累加连续失败计数 (与 RTCM 帧间隔解耦, 避免低频帧率下的误判)。
    g_sdk_instance->consecutive_write_fail_.fetch_add(1);
    SPDLOG_WARN("写 RTCM 到串口失败 (len={}, 连续失败 {} 次)", len, g_sdk_instance->consecutive_write_fail_.load());
  }
}

}   // namespace

SdkClient::SdkClient(const rclcpp::NodeOptions& options) : Node("sdk_client", options) {
  g_sdk_instance = this;

  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  try {
    ConfigureLogger(log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024U * 1024U, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", e.what());
  }

  loadParameters();

  serial_ = std::make_unique<SerialPort>();

  running_       = true;
  worker_thread_ = std::thread(&SdkClient::workerLoop, this);
}

SdkClient::~SdkClient() {
  running_ = false;
  // 先置空全局指针, SDK 内部残存线程若再触发回调将看到 nullptr 并直接返回,
  // 避免在对象析构期间访问 this。
  g_sdk_instance = nullptr;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void SdkClient::loadLogParameters() {
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

void SdkClient::loadParameters() {
  declare_parameter("serial_port", "/dev/ttysWK0");
  declare_parameter("serial_baud", 115200);
  declare_parameter("sdk_key", "");
  declare_parameter("sdk_secret", "");
  declare_parameter("dev_id", "");
  declare_parameter("dev_type", "");
  declare_parameter("sdk_openapi_host", "openapi.qxwz.com");
  declare_parameter("sdk_openapi_port", 8443);
  declare_parameter("sdk_oss_host", "oss.qxwz.com");
  declare_parameter("sdk_oss_port", 8008);
  declare_parameter("sdk_coord_frame", static_cast<int>(kQxwzCoordFrameWgs84));
  declare_parameter("sdk_coord_epoch", "2005.0");
  declare_parameter("gga_interval_sec", 1.0);
  declare_parameter("sdk_reconnect_delay_sec", 1.0);
  declare_parameter("first_gga_timeout_sec", 30.0);
  declare_parameter("first_rtcm_timeout_sec", 30.0);
  declare_parameter("sdk_auth_timeout_sec", 10.0);
  declare_parameter("coordsys_timeout_sec", 1.0);
  declare_parameter("gga_timeout_sec", 30.0);
  declare_parameter("rtcm_timeout_sec", 10.0);
  declare_parameter("serial_write_fail_count", 10);
  declare_parameter("sdk_transient_error_max", 20);

  serial_port_               = get_parameter("serial_port").as_string();
  serial_baud_               = get_parameter("serial_baud").as_int();
  sdk_key_                   = get_parameter("sdk_key").as_string();
  sdk_secret_                = get_parameter("sdk_secret").as_string();
  dev_id_                    = get_parameter("dev_id").as_string();
  dev_type_                  = get_parameter("dev_type").as_string();
  sdk_openapi_host_          = get_parameter("sdk_openapi_host").as_string();
  sdk_openapi_port_          = get_parameter("sdk_openapi_port").as_int();
  sdk_oss_host_              = get_parameter("sdk_oss_host").as_string();
  sdk_oss_port_              = get_parameter("sdk_oss_port").as_int();
  sdk_coord_frame_           = get_parameter("sdk_coord_frame").as_int();
  sdk_coord_epoch_           = get_parameter("sdk_coord_epoch").as_string();
  gga_interval_sec_          = get_parameter("gga_interval_sec").as_double();
  sdk_reconnect_delay_sec_   = get_parameter("sdk_reconnect_delay_sec").as_double();
  first_gga_timeout_sec_     = get_parameter("first_gga_timeout_sec").as_double();
  first_rtcm_timeout_sec_    = get_parameter("first_rtcm_timeout_sec").as_double();
  sdk_auth_timeout_sec_      = get_parameter("sdk_auth_timeout_sec").as_double();
  coordsys_timeout_sec_      = get_parameter("coordsys_timeout_sec").as_double();
  gga_timeout_sec_           = get_parameter("gga_timeout_sec").as_double();
  rtcm_timeout_sec_          = get_parameter("rtcm_timeout_sec").as_double();
  serial_write_fail_count_   = get_parameter("serial_write_fail_count").as_int();
  sdk_transient_error_max_   = get_parameter("sdk_transient_error_max").as_int();

  // 1.29 SDK 关闭了默认域名，init 必须拿到完整的 serv_conf；端口非法时回退到官方默认值。
  const auto valid_port = [](int port) { return port > 0 && port <= 65535; };
  if (sdk_openapi_host_.empty()) {
    SPDLOG_WARN("sdk_openapi_host 为空, 使用默认 openapi.qxwz.com");
    sdk_openapi_host_ = "openapi.qxwz.com";
  }
  if (sdk_oss_host_.empty()) {
    SPDLOG_WARN("sdk_oss_host 为空, 使用默认 oss.qxwz.com");
    sdk_oss_host_ = "oss.qxwz.com";
  }
  if (sdk_coord_epoch_.empty()) {
    SPDLOG_WARN("sdk_coord_epoch 为空, 使用默认 2005.0");
    sdk_coord_epoch_ = "2005.0";
  }
  if (!valid_port(sdk_openapi_port_)) {
    SPDLOG_WARN("sdk_openapi_port={} 非法, 使用默认 8443", sdk_openapi_port_);
    sdk_openapi_port_ = 8443;
  }
  if (!valid_port(sdk_oss_port_)) {
    SPDLOG_WARN("sdk_oss_port={} 非法, 使用默认 8008", sdk_oss_port_);
    sdk_oss_port_ = 8008;
  }

  SPDLOG_INFO("========== 千寻 SDK 客户端参数加载 ==========");
  SPDLOG_INFO("串口: {}@{}", serial_port_, serial_baud_);
  SPDLOG_INFO("SDK: key={} dev_id={} dev_type={}", sdk_key_.empty() ? "(未配置)" : sdk_key_, dev_id_, dev_type_);
  SPDLOG_INFO("SDK 服务器: openapi={}:{}, oss={}:{}", sdk_openapi_host_, sdk_openapi_port_, sdk_oss_host_, sdk_oss_port_);
  SPDLOG_INFO("NOSR 坐标框架: {}, 历元: {}", sdk_coord_frame_, sdk_coord_epoch_);
  SPDLOG_INFO("GGA 上传周期: {:.1f}s, 重连延迟: {:.1f}s, 首 GGA 超时: {:.1f}s, 首 RTCM 超时: {:.1f}s", gga_interval_sec_, sdk_reconnect_delay_sec_, first_gga_timeout_sec_, first_rtcm_timeout_sec_);
  SPDLOG_INFO("GGA 停摆超时: {:.1f}s, RTCM 停摆超时: {:.1f}s", gga_timeout_sec_, rtcm_timeout_sec_);
  SPDLOG_INFO(
    "SDK 认证超时: {:.1f}s, 设置坐标系超时: {:.1f}s, 串口写失败容忍帧数: {}, SDK 瞬时错误累计上限: {}",
    sdk_auth_timeout_sec_,
    coordsys_timeout_sec_,
    serial_write_fail_count_,
    sdk_transient_error_max_);
  SPDLOG_INFO("日志级别: {}, 目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}", log_level_, log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "是" : "否");
  const char* sdk_ver = qxwz_sdk_version();
  const char* sdk_build = qxwz_sdk_get_build_info();
  SPDLOG_INFO("千寻 SDK 版本: {} (build: {})", sdk_ver ? sdk_ver : "(未知)", sdk_build ? sdk_build : "(未知)");
  SPDLOG_INFO("=============================================");
}

// 串口写入 RTCM 数据
bool SdkClient::writeRtcm(const uint8_t* data, uint32_t len) {
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_ || !serial_->is_open()) {
    return false;
  }
  return serial_->write_raw(reinterpret_cast<const char*>(data), static_cast<size_t>(len));
}

// 打开串口 (失败不记日志, 由调用方按节奏记录)。每次都重新打开, 确保干净的重连状态。
bool SdkClient::openSerial() {
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_->open(serial_port_, serial_baud_)) {
    SPDLOG_WARN("打开串口 {} 失败", serial_port_);
    return false;
  }
  SPDLOG_INFO("串口已打开: {}", serial_port_);
  return true;
}

bool SdkClient::checkGga(const std::string& line) const {
  // 语句类型: 必须是 $xxGGA 开头
  if (line.size() < 6 || line[0] != '$' || line.compare(3, 3, "GGA") != 0) {
    return false;
  }
  // 异或校验和
  const size_t star = line.find('*');
  if (star == std::string::npos || star + 3 > line.size()) {
    return false;
  }
  unsigned char cs = 0;
  for (size_t i = 1; i < star; ++i) {
    cs ^= static_cast<unsigned char>(line[i]);
  }
  char hex[3]                    = {line[star + 1], line[star + 2], 0};
  char* end                      = nullptr;
  const unsigned long parsed_val = std::strtoul(hex, &end, 16);
  if (end != hex + 2) {
    return false;   // 校验和字段不是合法的 2 位十六进制
  }
  const unsigned int expected = static_cast<unsigned int>(parsed_val);
  if (cs != expected) {
    return false;
  }

  // 校验定位状态 (Fix Quality): 过滤掉无效定位 (0) 的 GGA,
  // 避免把接收机尚未定位的位置上传给差分服务, 导致选错差分源。
  // 字段序 (逗号分隔): 1=类型,2=时间,3=纬度,4=纬度半球,5=经度,6=经度半球,7=FixQuality,8=卫星数,...
  // 故 Fix Quality 是第 7 个字段 = 第 6 个逗号之后。
  size_t comma_count = 0;
  size_t field_start = 0;
  for (size_t i = 1; i < star; ++i) {
    if (line[i] == ',') {
      ++comma_count;
      if (comma_count == 6) {
        field_start = i + 1;   // 第 7 个字段 (Fix Quality) 起始位置
        break;
      }
    }
  }
  if (comma_count != 6) {
    return false;   // 字段不足, 无法判断定位状态
  }
  const size_t field_end        = line.find(',', field_start);
  const std::string fix_quality = (field_end == std::string::npos || field_end > star) ? line.substr(field_start, star - field_start) : line.substr(field_start, field_end - field_start);
  if (fix_quality.empty()) {
    return false;
  }
  const int quality = std::atoi(fix_quality.c_str());
  return quality > 0;   // 0 = 无效定位, 其余 (1/2/4/5/6...) 均可用于差分请求
}

std::optional<std::string> SdkClient::tryReadGga() {
  std::optional<std::string> result;
  // 与 SDK data 回调线程的 writeRtcm / openSerial 互斥: 避免读到/写到正被重建的 fd。
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_->read_lines([&](const std::string& line) {
    if (checkGga(line) && !result.has_value()) {
      // 上传给 qxwz_sdk_upload_gga 的是纯 GGA 语句 (无行结束符), 与 SDK demo 的
      // DEMO_GGA_STR (结尾 *xx, 无 \r\n) 保持一致; 不像 ntrip_client 需补 \r\n
      // (后者直接写 NTRIP socket, 依赖 NMEA 的 CRLF 行尾)。
      result = line;
    }
  });
  return result;
}

std::optional<std::pair<std::string, std::chrono::steady_clock::time_point>> SdkClient::waitForGga(double timeout_seconds) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    auto gga = tryReadGga();
    if (gga) {
      return std::make_pair(*gga, std::chrono::steady_clock::now());
    }
    // GGA 为 1Hz, 轮询 50ms 足够 (数据在内核缓冲区不会丢, 只影响发现延迟)。
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return std::nullopt;
}

// 初始化 SDK 并认证
bool SdkClient::initSdk() {
  // 重置回调状态, 确保重连时干净 (上一轮的错误/认证状态不残留)。
  sdk_auth_ready_.store(false);
  sdk_error_code_.store(0);
  sdk_nosr_start_failed_.store(false);
  last_rtcm_ms_.store(0);
  consecutive_write_fail_.store(0);
  transient_error_count_.store(0);

  // 配置账号信息
  qxwz_sdk_config_t sdk_config{};
  sdk_config.key_type = QXWZ_SDK_KEY_TYPE_DSK;
  std::strncpy(sdk_config.key, sdk_key_.c_str(), sizeof(sdk_config.key) - 1);
  std::strncpy(sdk_config.secret, sdk_secret_.c_str(), sizeof(sdk_config.secret) - 1);
  std::strncpy(sdk_config.dev_id, dev_id_.c_str(), sizeof(sdk_config.dev_id) - 1);
  std::strncpy(sdk_config.dev_type, dev_type_.c_str(), sizeof(sdk_config.dev_type) - 1);

  // 配置回调
  sdk_config.status_cb     = sdkStatusCallback;
  sdk_config.data_cb       = sdkDataCallback;
  sdk_config.auth_cb       = sdkAuthCallback;
  sdk_config.start_cb      = sdkStartCallback;
  sdk_config.cap_status_cb = sdkCapStatusCallback;

  // 1.29 SDK 默认域名关闭，服务器域名/端口必须显式配置。
  std::strncpy(sdk_config.serv_conf.openapi_host, sdk_openapi_host_.c_str(), sizeof(sdk_config.serv_conf.openapi_host) - 1);
  sdk_config.serv_conf.openapi_port = static_cast<qxwz_uint32_t>(sdk_openapi_port_);
  std::strncpy(sdk_config.serv_conf.oss_host, sdk_oss_host_.c_str(), sizeof(sdk_config.serv_conf.oss_host) - 1);
  sdk_config.serv_conf.oss_port = static_cast<qxwz_uint32_t>(sdk_oss_port_);

  // NOSR 数据格式: 标准 RTCM
  qxwz_sdk_nosr_data_format_t data_fmt = QXWZ_SDK_NOSR_STD_FORMAT;
  const int fmt_ret                    = qxwz_sdk_config(QXWZ_SDK_CONF_NOSR_DATA_FORMAT, static_cast<qxwz_void_t*>(&data_fmt));
  if (fmt_ret < 0) {
    SPDLOG_WARN("千寻 SDK 设置 NOSR 数据格式失败: {}", fmt_ret);
  }

  // 初始化 SDK
  int ret = qxwz_sdk_init(&sdk_config);
  if (ret < 0) {
    SPDLOG_ERROR("千寻 SDK init 失败: {}", ret);
    qxwz_sdk_cleanup();
    return false;
  }

  // 认证 SDK
  ret = qxwz_sdk_auth();
  if (ret < 0) {
    SPDLOG_ERROR("千寻 SDK auth 调用失败: {}", ret);
    qxwz_sdk_cleanup();
    return false;
  }
  return true;
}

// 启动 NOSR 能力
bool SdkClient::startNosr() {
  // 认证成功后启动 NOSR 能力
  qxwz_sdk_cap_info_t cap_info{};
  if (qxwz_sdk_get_cap_info(&cap_info) < 0) {
    SPDLOG_ERROR("获取能力信息失败, 无法启动 NOSR");
    return false;
  }
  for (qxwz_uint32_t i = 0; i < cap_info.caps_num; ++i) {
    if (cap_info.caps[i].cap_id == QXWZ_SDK_CAP_ID_NOSR && cap_info.caps[i].state == QXWZ_SDK_CAP_STATE_INSERVICE) {
      const int ret = qxwz_sdk_start(QXWZ_SDK_CAP_ID_NOSR);
      if (ret < 0) {
        SPDLOG_ERROR("请求启动 NOSR 失败: {}", ret);
        return false;
      }
      SPDLOG_INFO("已请求启动 NOSR 能力");
      return true;
    }
  }
  SPDLOG_ERROR("NOSR 能力不在服务中");
  return false;
}

// 主线程
void SdkClient::workerLoop() {
  bool serial_failing  = false;   // 处于串口连续失败序列中
  bool connect_failing = false;   // 处于 SDK 初始化/认证连续失败序列中

  while (running_) {
    // ---- 1. 打开串口 (固定 200ms 重试节奏) ----
    if (!openSerial()) {
      if (!serial_failing) {
        serial_failing = true;
        SPDLOG_WARN("串口打开失败: {}, 以 200ms 节奏重试", serial_port_);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    serial_failing = false;
    SPDLOG_INFO("串口打开成功: {}", serial_port_);

    // ---- 2. 等待首个 GGA (确认串口+接收机链路正常; GGA 由接收机独立吐出, 不依赖 SDK) ----
    SPDLOG_INFO("等待串口 GGA ...");
    const auto first_gga = waitForGga(first_gga_timeout_sec_);
    if (!first_gga) {
      SPDLOG_WARN("等待首个 GGA 超时, 200ms 后重试");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    SPDLOG_INFO("收到首个 GGA");

    // ---- 3. 初始化 + 认证 SDK ----
    if (!initSdk()) {
      if (!connect_failing) {
        connect_failing = true;
        SPDLOG_WARN("千寻 SDK 初始化/认证调用失败, 以 {}s 重试", sdk_reconnect_delay_sec_);
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(sdk_reconnect_delay_sec_));
      continue;
    }
    connect_failing = false;

    // ---- 4. 等待 SDK 认证回调 ----
    // 认证结果有两种明确结局: 成功置 sdk_auth_ready_, 失败写 sdk_error_code_。
    // 循环同时检查两者, 认证失败立即退出, 不必等到超时。
    const auto auth_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(sdk_auth_timeout_sec_);
    while (running_ && !sdk_auth_ready_.load() && sdk_error_code_.load() == 0 && std::chrono::steady_clock::now() < auth_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!sdk_auth_ready_.load()) {
      if (!connect_failing) {
        connect_failing = true;
      }
      const int32_t auth_err = sdk_error_code_.load();
      if (auth_err != 0) {
        // 认证失败 (回调已返回错误码)。
        SPDLOG_ERROR("千寻 SDK 认证失败, code={}, {:.0f}s 后重连", auth_err, sdk_reconnect_delay_sec_);
      } else {
        // 认证回调迟迟未到: 真正的超时。
        SPDLOG_WARN("等待 SDK 认证超时, {:.0f}s 后重连", sdk_reconnect_delay_sec_);
      }
      qxwz_sdk_cleanup();
      sdk_auth_ready_.store(false);
      sdk_error_code_.store(0);
      std::this_thread::sleep_for(std::chrono::duration<double>(sdk_reconnect_delay_sec_));
      continue;
    }
    connect_failing = false;
    SPDLOG_INFO("千寻 SDK 认证成功");

    // ---- 5. 设置 NOSR 坐标框架/历元 ----
    sdk_coordsys_done_.store(false);
    sdk_coordsys_code_.store(0);
    qxwz_nosr_coord_epoch_config_t coord_config{};
    coord_config.coord_frame_value = sdk_coord_frame_;
    std::strncpy(coord_config.epoch, sdk_coord_epoch_.c_str(), sizeof(coord_config.epoch) - 1);
    const int coord_ret = qxwz_sdk_set_nosr_coord_epoch_config(&coord_config, sdkSetNosrCoordCallback);
    const bool coord_async = (coord_ret == QXWZ_SDK_STAT_OK || coord_ret == QXWZ_SDK_ERR_GETTING_COORD_SYS || coord_ret == QXWZ_SDK_ERR_SETTING_COORD_SYS);
    if (!coord_async) {
      SPDLOG_WARN("请求设置 NOSR 坐标框架/历元失败: code={}, 沿用当前配置", coord_ret);
    } else {
      const auto coordsys_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(coordsys_timeout_sec_);
      while (running_ && !sdk_coordsys_done_.load() && std::chrono::steady_clock::now() < coordsys_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (!sdk_coordsys_done_.load()) {
        SPDLOG_WARN("NOSR 坐标框架/历元设置超时, 沿用当前配置");
      } else {
        const int32_t coordsys_ret = sdk_coordsys_code_.load();
        if (coordsys_ret != QXWZ_SDK_STAT_OK) {
          SPDLOG_WARN("NOSR 坐标框架/历元设置失败: code={}, 沿用当前配置", coordsys_ret);
        } else {
          SPDLOG_INFO("NOSR 坐标框架/历元设置成功 (frame={}, epoch={})", sdk_coord_frame_, sdk_coord_epoch_);
        }
      }
    }

    // ---- 6. 启动 NOSR 能力 (失败则无法收 RTCM, 直接重连)。  ----
    if (!startNosr()) {
      SPDLOG_WARN("启动 NOSR 失败, {:.0f}s 后重连", sdk_reconnect_delay_sec_);
      qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR);
      qxwz_sdk_cleanup();
      std::this_thread::sleep_for(std::chrono::duration<double>(sdk_reconnect_delay_sec_));
      continue;
    }
    // NOSR 启动失败检查 (进入数据交互前的同步快照): start_cb 可能已早于 startNosr()
    // 返回就被调用, 这里先行捕获, 避免无谓进入 runSession。若 start_cb 延迟到运行期
    // 才回调, 则由 runSession 内部的持续检查兜底 (见 runSession)。
    if (sdk_nosr_start_failed_.load()) {
      SPDLOG_ERROR("NOSR 启动异步失败, {:.0f}s 后重连", sdk_reconnect_delay_sec_);
      qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR);
      qxwz_sdk_cleanup();
      std::this_thread::sleep_for(std::chrono::duration<double>(sdk_reconnect_delay_sec_));
      continue;
    }

    // ---- 7. 数据交互阶段。 ----
    runSession(first_gga->first, first_gga->second);

    // 重置 SDK 错误状态, 避免下一轮会话误判 (重连时 initSdk 也会重置)。
    sdk_error_code_.store(0);
    sdk_nosr_start_failed_.store(false);
    consecutive_write_fail_.store(0);
    transient_error_count_.store(0);

    // 会话结束 (running_ 变 false 或数据超时): 停止 NOSR 并清理 SDK。
    qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR);
    qxwz_sdk_cleanup();
  }
  if (serial_->is_open()) {
    serial_->close();
  }
  qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR);
  qxwz_sdk_cleanup();
  SPDLOG_INFO("千寻 SDK 客户端已停止");
}

// 数据交互阶段: 周期上传 GGA, 并做 GGA/RTCM 数据超时判定。
// 返回结束原因 (数据超时或正常退出), 由 workerLoop 据此上报对应故障码。
SdkClient::SessionEnd SdkClient::runSession(const std::string& first_gga, std::chrono::steady_clock::time_point first_gga_time) {
  std::string latest_gga   = first_gga;
  const auto session_begin = std::chrono::steady_clock::now();
  std::optional<std::chrono::steady_clock::time_point> last_gga_sent;
  auto last_gga_time = first_gga_time;   // GGA 停摆判定的计时基准 (以首帧 GGA 到达时刻起算)

  while (running_) {
    const auto gga = tryReadGga();
    const auto now = std::chrono::steady_clock::now();

    if (gga) {
      latest_gga    = *gga;
      last_gga_time = now;
    }

    // 周期上传 GGA (用即时时间戳); 首轮因 last_gga_sent 为 nullopt 而立即上传 first_gga。
    if (!latest_gga.empty() && (!last_gga_sent || std::chrono::duration<double>(now - *last_gga_sent).count() >= gga_interval_sec_)) {
      qxwz_sdk_upload_gga(latest_gga.c_str(), static_cast<qxwz_uint32_t>(latest_gga.size()));
      SPDLOG_DEBUG("上传 GGA (len={})", latest_gga.size());
      last_gga_sent = now;
    }

    // GGA 数据超时判定: 以首帧 GGA 到达时刻为起点, 之后每收新 GGA 更新; 始终检查,
    // 持续 gga_timeout_sec_ 无新 GGA 即判停摆。
    if (std::chrono::duration<double>(now - last_gga_time).count() >= gga_timeout_sec_) {
      SPDLOG_WARN("GGA 数据超时");
      return SessionEnd::kGgaTimeout;
    }

    // RTCM 数据超时判定: SDK data 回调会更新 last_rtcm_ms_ (0 = 尚未收到首帧)。
    // 首帧前以会话起点起算 (大宽限, 等差分服务挂载); 首帧后以最后帧时间起算 (小超时, 判停摆)。
    // 注意: 首帧判定用 `last_ms != 0` 而非额外标志, 避免"首帧永不到达时永不超时" (静默失效)。
    const int64_t last_ms = last_rtcm_ms_.load();
    if (last_ms == 0) {
      if (std::chrono::duration<double>(now - session_begin).count() >= first_rtcm_timeout_sec_) {
        SPDLOG_WARN("等待首帧 RTCM 超时");
        return SessionEnd::kRtcmTimeout;
      }
    } else {
      if (std::chrono::duration<double>(now - std::chrono::steady_clock::time_point(std::chrono::milliseconds(last_ms))).count() >= rtcm_timeout_sec_) {
        SPDLOG_WARN("RTCM 数据超时");
        return SessionEnd::kRtcmTimeout;
      }
    }

    // 串口写 RTCM 失败判定: 连续写失败达到阈值才判故障 (与 RTCM 帧间隔解耦,
    // 避免低频帧率/采样相位导致的误判)。写成功即清零计数。
    if (consecutive_write_fail_.load() >= serial_write_fail_count_) {
      SPDLOG_ERROR("串口写 RTCM 连续失败 {} 次, 结束会话", serial_write_fail_count_);
      return SessionEnd::kSerialWriteFail;
    }

    // SDK 内部错误判定: status 回调写入 sdk_error_code_, 立即结束会话并上报。
    // (运行中随时可能发生, 故在此高频轮询处检查。)
    const int32_t sdk_err = sdk_error_code_.load();
    if (sdk_err != 0) {
      SPDLOG_ERROR("检测到 SDK 内部错误, code={}, 结束会话", sdk_err);
      return SessionEnd::kSdkError;
    }

    // SDK 瞬时错误累计判定: 单帧网络/数据质量类错误 SDK 会自行重试, 但持续高频
    // (累计超阈值) 说明差分链路质量持续差, 会话虽存活但数据不可用, 需重建。
    if (transient_error_count_.load() >= sdk_transient_error_max_) {
      SPDLOG_ERROR("SDK 瞬时错误累计 {} 次 (超阈值 {}), 差分链路质量持续差, 结束会话重建", transient_error_count_.load(), sdk_transient_error_max_);
      return SessionEnd::kSdkTransientError;
    }

    // NOSR 启动异步失败判定: start 是异步的, start_cb 可能在 startNosr() 返回后才回调,
    // 故外层 workerLoop 的同步检查可能漏掉; 这里持续检查, 一旦失败立即结束会话。
    if (sdk_nosr_start_failed_.load()) {
      SPDLOG_ERROR("检测到 NOSR 启动异步失败, 结束会话");
      return SessionEnd::kNosrStartFail;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return SessionEnd::kDisconnected;   // running_ 变 false, 正常退出
}

}   // namespace gnss_localization
