// 底盘 ROS2 节点实现。
//
// 下行: /weeder/chassis/cmd (速度 + 转角) -> ADCU_toVCU_01 (20ms 周期)
// 上行: VCU Chassis_State_01~11/21/22 -> /weeder/chassis/state
//
// 时钟约定 (踩过的坑, 别改回去):
//   - 全部时间戳用 this->now() (RCL_ROS_TIME)。跨时钟源相减会抛
//     std::runtime_error, 异常从定时器回调冒出会直接 terminate 进程。
//   - 所有"上次时刻"在构造函数里对齐到启动时刻。停在 epoch 0 的话, 启动后第一个
//     20ms 周期的 age 就是十亿秒级, 必然立刻进入超时分支 -> 一启动就锁急停。
#include "weeder_base/chassis_node.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

// 协议硬上限 (见 weeder_base/protocol/can_protocol.hpp 的 ADCU_toVCU_01):
//   speed_raw   uint16, mm/s, 协议范围 0~1400
//   steer_angle int16,  0.1°, 协议范围 ±90.0°
constexpr double kMaxSpeedLimitMps = 1.4;
constexpr double kMaxSteerLimitDeg = 90.0;

// 默认值 (与 config/weeder_base.yaml 保持一致, 改一边记得改另一边)
constexpr double kDefaultPublishHz            = 20.0;
constexpr double kDefaultCmdTimeoutS          = 0.5;
constexpr double kDefaultVcuTimeoutS          = 1.0;
constexpr double kDefaultWatchdogLogIntervalS = 5.0;
constexpr const char* kDefaultCmdTopic        = "/weeder/chassis/cmd";

}   // namespace

namespace detail {

// 扩展路径中的环境变量 (~ 与 $VAR)
std::string ExpandPath(std::string path) {
  if (path.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home)
      path = std::string(home) + path.substr(1);
  }
  std::string out;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i] != '$') {
      out += path[i];
      continue;
    }
    size_t j = i + 1;
    while (j < path.size() && (std::isalnum(static_cast<unsigned char>(path[j])) || path[j] == '_'))
      ++j;
    const std::string name = path.substr(i + 1, j - i - 1);
    const char* val        = name.empty() ? nullptr : std::getenv(name.c_str());
    if (val)
      out += val;
    i = j - 1;
  }
  return out;
}

// 解析日志目录: 优先用配置的, 不可写则回退到 $ROS_HOME/logs 或 ~/.ros/logs
std::string ResolveLogDir(const std::string& configured) {
  std::error_code ec;
  const std::string preferred = ExpandPath(configured);
  if (!preferred.empty()) {
    std::filesystem::create_directories(preferred, ec);
    if (!ec && access(preferred.c_str(), W_OK) == 0)
      return preferred;
  }

  const char* ros_home = std::getenv("ROS_HOME");
  const char* home     = std::getenv("HOME");
  std::string fallback;
  if (ros_home && *ros_home)
    fallback = std::string(ros_home) + "/logs";
  else if (home && *home)
    fallback = std::string(home) + "/.ros/logs";
  else
    return "";

  std::filesystem::create_directories(fallback, ec);
  if (ec || access(fallback.c_str(), W_OK) != 0)
    return "";
  return fallback;
}

// 配置 spdlog 滚动文件输出 (与 tractor_base 同一套配置)
void ConfigureLogger(const std::string& log_dir, size_t max_file_size_bytes, size_t max_file_count, bool enable_console, const std::string& log_level) {
  constexpr const char* kLogPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";
  const std::string dir             = ResolveLogDir(log_dir);
  if (dir.empty()) {
    throw std::runtime_error("no writable log directory (tried \"" + log_dir + "\")");
  }

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((std::filesystem::path(dir) / "weeder_base.log").string(), max_file_size_bytes, max_file_count));
  if (enable_console) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>("weeder_base", sinks.begin(), sinks.end());
  logger->set_pattern(kLogPattern);
  // spdlog::level::from_str 区分大小写, 认不出级别名时返回 off —— 也就是 yaml 里写错一个
  // 字母 ("Info"/"warning") 就会把全部日志静默关掉, 现场表现为"日志文件不再增长"。
  // 这里兜底回退到 info 并告警。
  const spdlog::level::level_enum level = spdlog::level::from_str(log_level);
  if (level == spdlog::level::off && log_level != "off") {
    logger->set_level(spdlog::level::info);
    SPDLOG_WARN("无法识别的日志级别 \"{}\", 已回退到 info", log_level);
  } else {
    logger->set_level(level);
  }
  spdlog::set_default_logger(logger);
  if (dir != ExpandPath(log_dir)) {
    SPDLOG_WARN("日志目录 \"{}\" 不可写, 已回退到 \"{}\"", log_dir, dir);
  }
  spdlog::flush_on(spdlog::level::warn);
  spdlog::flush_every(3s);
}

}   // namespace detail

namespace weeder_base {

ChassisNode::ChassisNode(const rclcpp::NodeOptions& options) : Node("weeder_chassis", options) {

  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  try {
    detail::ConfigureLogger(log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024U * 1024U, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败, 使用默认日志器: {}", e.what());
  }

  loadParameters();

  // ---- CAN 驱动 ----
  // CAN 是底盘控制的命根子, 打不开即快速失败 (进程退出, 由 launch respawn 拉起):
  // 静默"正常运行"却一帧都发不出去, 比明确报错危险得多。
  can_driver_ = std::make_unique<CanDriver>(can_interface_);
  // 只接收协议文档中 VCU -> ADCU 的全部周期帧。漏配会导致对应解码分支
  // 永远收不到数据, 因此这里与 onCanReceive() 的 switch 保持一一对应。
  can_driver_->setFilters({
    protocol::can_id::CHASSIS_STATE_01,
    protocol::can_id::CHASSIS_STATE_02,
    protocol::can_id::CHASSIS_STATE_03,
    protocol::can_id::CHASSIS_STATE_04,
    protocol::can_id::CHASSIS_STATE_05,
    protocol::can_id::CHASSIS_STATE_06,
    protocol::can_id::CHASSIS_STATE_07,
    protocol::can_id::CHASSIS_STATE_08,
    protocol::can_id::CHASSIS_STATE_09,
    protocol::can_id::CHASSIS_STATE_10,
    protocol::can_id::CHASSIS_STATE_11,
    protocol::can_id::CHASSIS_STATE_21,
    protocol::can_id::CHASSIS_STATE_22,
  });
  if (!can_driver_->open()) {
    throw std::runtime_error("无法打开 CAN 接口: " + can_interface_);
  }
  can_driver_->setReceiveCallback([this](const protocol::CanFrame& frame) {
    onCanReceive(frame);
  });
  can_driver_->startReceiveThread();

  // ---- 时间基准 ----
  // 必须与 this->now() 同为 RCL_ROS_TIME: 跨时钟源相减会抛 std::runtime_error,
  // 异常从定时器回调冒出会直接 terminate 进程。
  // 同时对齐到启动时刻: 否则 last_cmd_time_ 停留在 epoch 0, 启动后第一个 20ms
  // 周期的 age 就是十亿秒级, 必然立即进入超时分支。
  const rclcpp::Time now   = this->now();
  last_cmd_time_           = now;
  last_cmd_stale_log_time_ = now;
  last_vcu_stale_log_time_ = now;
  last_vcu_stuck_log_time_ = now;
  last_send_fail_log_time_ = now;
  last_vcu_ns_.store(now.nanoseconds(), std::memory_order_relaxed);
  last_vcu_alive_change_ns_.store(now.nanoseconds(), std::memory_order_relaxed);

  // ---- ROS 话题 ----
  // 速度与转角必须来自同一条消息、同一控制周期。
  cmd_sub_        = create_subscription<weeder_msgs::msg::ChassisCmd>(cmd_topic_, 10, std::bind(&ChassisNode::onChassisCmd, this, std::placeholders::_1));
  state_pub_      = create_publisher<weeder_msgs::msg::ChassisState>("/weeder/chassis/state", 10);
  state_text_pub_ = create_publisher<std_msgs::msg::String>("/weeder/chassis/state_text", 10);

  // ---- 定时器 ----
  // 20ms = ADCU_toVCU_01 的协议周期; VCU 靠该帧的生命信号 (alive_counter) 判断
  // ADCU 是否存活, 所以周期必须守住。
  send_timer_ = create_wall_timer(20ms, std::bind(&ChassisNode::sendCommand, this));

  const auto pub_period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_hz_));
  pub_timer_            = create_wall_timer(pub_period, std::bind(&ChassisNode::publishState, this));

  SPDLOG_INFO(
    "ChassisNode 启动: 接口={} cmd_timeout={:.2f}s vcu_timeout={:.2f}s max_speed={:.2f} m/s max_steer={:.1f} deg publish={:.1f} Hz",
    can_interface_,
    cmd_timeout_s_,
    vcu_timeout_s_,
    max_speed_m_s_,
    max_steer_deg_,
    publish_hz_);
  SPDLOG_INFO("底盘指令: {} | steer_mode={}", cmd_topic_, static_cast<unsigned>(steer_mode_));
}

ChassisNode::~ChassisNode() {
  // 析构体在成员销毁之前执行: 先停定时器, 再停接收线程并注销回调,
  // 避免 CAN 线程在 this 已销毁后回调 onCanReceive()。
  send_timer_.reset();
  pub_timer_.reset();
  cmd_sub_.reset();
  if (can_driver_) {
    can_driver_->stopReceiveThread();   // join 返回后才可能销毁 this
    can_driver_->setReceiveCallback(nullptr);
    can_driver_->close();
  }
}

void ChassisNode::loadLogParameters() {
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 50);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

void ChassisNode::loadParameters() {
  declare_parameter("can_interface", "vcan0");
  declare_parameter("publish_hz", kDefaultPublishHz);
  declare_parameter("cmd_timeout_s", kDefaultCmdTimeoutS);
  declare_parameter("vcu_timeout_s", kDefaultVcuTimeoutS);
  declare_parameter("max_speed_m_s", kMaxSpeedLimitMps);
  declare_parameter("max_steer_deg", kMaxSteerLimitDeg);
  declare_parameter("cmd_topic", kDefaultCmdTopic);
  declare_parameter("steer_mode", 0);
  declare_parameter("watchdog_log_interval_s", kDefaultWatchdogLogIntervalS);

  can_interface_                 = get_parameter("can_interface").as_string();
  publish_hz_                    = get_parameter("publish_hz").as_double();
  cmd_timeout_s_                 = get_parameter("cmd_timeout_s").as_double();
  vcu_timeout_s_                 = get_parameter("vcu_timeout_s").as_double();
  max_speed_m_s_                 = get_parameter("max_speed_m_s").as_double();
  max_steer_deg_                 = get_parameter("max_steer_deg").as_double();
  cmd_topic_                     = get_parameter("cmd_topic").as_string();
  watchdog_log_interval_s_       = get_parameter("watchdog_log_interval_s").as_double();
  const int64_t steer_mode_param = get_parameter("steer_mode").as_int();

  // ---- 参数边界校验 ----
  // 两类错都得拦:
  //   1. 数值越界: std::clamp(v, 0.0, max) 在 max<0 时是 UB (precondition 违反);
  //      车速/转角还有协议硬上限 (1400 mm/s, ±90.0°)。
  //   2. 语义错位: steer_mode 必须与 lqr_control 的同名参数一致, 单/双阿克曼的
  //      等效轴距差一倍, 对不上会让实际转角差一倍 —— "改了没生效" 类事故里最难查的一种。
  // 一律回退到默认值 + 告警, 不用 std::max(0.0, x) 静默吸收错误值: 那样现场看到的
  // 只是"限速改了没生效", 日志里却没有任何线索。
  if (publish_hz_ <= 0.0) {
    SPDLOG_WARN("publish_hz={:.3f} 非法, 回退为 {:.1f} Hz", publish_hz_, kDefaultPublishHz);
    publish_hz_ = kDefaultPublishHz;
  }
  if (cmd_timeout_s_ <= 0.0) {
    SPDLOG_WARN("cmd_timeout_s={:.3f} 非法, 回退为 {:.1f} s", cmd_timeout_s_, kDefaultCmdTimeoutS);
    cmd_timeout_s_ = kDefaultCmdTimeoutS;
  }
  if (vcu_timeout_s_ <= 0.0) {
    SPDLOG_WARN("vcu_timeout_s={:.3f} 非法, 回退为 {:.1f} s", vcu_timeout_s_, kDefaultVcuTimeoutS);
    vcu_timeout_s_ = kDefaultVcuTimeoutS;
  }
  if (watchdog_log_interval_s_ <= 0.0) {
    SPDLOG_WARN("watchdog_log_interval_s={:.3f} 非法, 回退为 {:.1f} s", watchdog_log_interval_s_, kDefaultWatchdogLogIntervalS);
    watchdog_log_interval_s_ = kDefaultWatchdogLogIntervalS;
  }
  if (max_speed_m_s_ < 0.0 || max_speed_m_s_ > kMaxSpeedLimitMps) {
    SPDLOG_WARN("max_speed_m_s={:.3f} 超出 [0, {:.1f}] (协议上限 1400 mm/s), 回退为 {:.1f}", max_speed_m_s_, kMaxSpeedLimitMps, kMaxSpeedLimitMps);
    max_speed_m_s_ = kMaxSpeedLimitMps;
  }
  if (max_steer_deg_ < 0.0 || max_steer_deg_ > kMaxSteerLimitDeg) {
    SPDLOG_WARN("max_steer_deg={:.1f} 超出 [0, {:.1f}] (协议上限 ±90.0°), 回退为 {:.1f}", max_steer_deg_, kMaxSteerLimitDeg, kMaxSteerLimitDeg);
    max_steer_deg_ = kMaxSteerLimitDeg;
  }
  if (cmd_topic_.empty()) {
    SPDLOG_WARN("cmd_topic 为空, 回退为 {}", kDefaultCmdTopic);
    cmd_topic_ = kDefaultCmdTopic;
  }
  if (steer_mode_param < 0 || steer_mode_param > 3) {
    SPDLOG_WARN("steer_mode={} 非法 (取值 0~3), 回退为 0 (单阿克曼)", steer_mode_param);
    steer_mode_ = 0;
  } else {
    steer_mode_ = static_cast<uint8_t>(steer_mode_param);
  }

  SPDLOG_INFO(
    "参数: publish_hz={:.1f} cmd_timeout={:.2f}s vcu_timeout={:.2f}s max_speed={:.2f} m/s max_steer={:.1f} deg watchdog_log={:.1f}s",
    publish_hz_,
    cmd_timeout_s_,
    vcu_timeout_s_,
    max_speed_m_s_,
    max_steer_deg_,
    watchdog_log_interval_s_);
}

// ---- 下行: 速度 + 前轮转角合并指令 ----
void ChassisNode::onChassisCmd(const weeder_msgs::msg::ChassisCmd::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  last_cmd_time_ = this->now();

  cmd01_.gear      = 1;   // D 档
  cmd01_.ctrl_mode = 1;   // 无人驾驶
  // 收到新指令 -> 解除超时看门狗锁存的急停。
  cmd01_.estop = 0;

  // m/s -> mm/s (协议速度 uint16, 分辨率 1 mm/s)
  const double v   = std::clamp(msg->speed_m_s, 0.0, max_speed_m_s_);
  cmd01_.speed_raw = static_cast<uint16_t>(std::llround(v * 1000.0));

  // rad -> 0.1°。左正右负与协议同向, 不取反。
  const double steer_deg   = msg->steer_angle_rad * 180.0 / M_PI;
  const double clamped_deg = std::clamp(steer_deg, -max_steer_deg_, max_steer_deg_);
  cmd01_.steer_angle       = static_cast<int16_t>(std::llround(clamped_deg * 10.0));
}

// ---- 20ms 周期下发 ADCU_toVCU_01 ----
void ChassisNode::sendCommand() {
  if (!can_driver_ || !can_driver_->isOpen()) {
    return;   // 驱动侧已报过"未打开", 不要以 50 Hz 刷日志
  }

  protocol::AdcuCommand01 out_cmd{};
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const rclcpp::Time now     = this->now();
    const double cmd_age       = (now - last_cmd_time_).seconds();
    const double vcu_age       = static_cast<double>(now.nanoseconds() - last_vcu_ns_.load(std::memory_order_relaxed)) * 1e-9;
    const double vcu_alive_age = static_cast<double>(now.nanoseconds() - last_vcu_alive_change_ns_.load(std::memory_order_relaxed)) * 1e-9;

    // motion_inhibit: 任一看门狗触发即禁止运动, 转向强制回正。
    // 集中在一处裁决, 避免看门狗置 0 后又被下面的来源选择覆盖。
    bool motion_inhibit = false;

    // 看门狗 1: 底盘合并指令超时 -> 停车 + 急停 (新指令可解除)
    if (cmd_age > cmd_timeout_s_) {
      cmd01_.speed_raw = 0;
      cmd01_.estop     = 1;
      motion_inhibit   = true;
      if (now - last_cmd_stale_log_time_ >= rclcpp::Duration::from_seconds(watchdog_log_interval_s_)) {
        last_cmd_stale_log_time_ = now;
        SPDLOG_WARN("底盘指令超时 {:.2f}s > {:.2f}s, 已停车并置急停", cmd_age, cmd_timeout_s_);
      }
    }

    // 看门狗 2: VCU 状态帧超时 -> 停车。
    // 不锁存 estop: VCU 恢复上报后自动回到正常控制, 无需人工干预。
    if (vcu_age > vcu_timeout_s_) {
      cmd01_.speed_raw = 0;
      motion_inhibit   = true;
      if (now - last_vcu_stale_log_time_ >= rclcpp::Duration::from_seconds(watchdog_log_interval_s_)) {
        last_vcu_stale_log_time_ = now;
        SPDLOG_WARN("VCU 状态帧超时 {:.2f}s > {:.2f}s, 已停车", vcu_age, vcu_timeout_s_);
      }
    } else if (vcu_alive_age > vcu_timeout_s_) {
      // State_01 还在发, 但 AliveCounter 不递增: 说明 VCU 控制程序卡死。
      // 与"完全收不到帧"分开告警, 方便现场判断是线束问题还是 VCU 问题。
      cmd01_.speed_raw = 0;
      motion_inhibit   = true;
      if (now - last_vcu_stuck_log_time_ >= rclcpp::Duration::from_seconds(watchdog_log_interval_s_)) {
        last_vcu_stuck_log_time_ = now;
        SPDLOG_WARN("VCU alive_counter 已 {:.2f}s 未变化 (>{:.2f}s), 已停车", vcu_alive_age, vcu_timeout_s_);
      }
    }

    if (motion_inhibit) {
      cmd01_.steer_angle = 0;
    }

    // 转向模式: 必须与 lqr_control 的 steer_mode 一致, 否则等效轴距对不上
    cmd01_.steer_mode = steer_mode_;

    cmd01_.alive_counter = static_cast<uint8_t>((cmd01_.alive_counter + 1) & 0xFF);
    out_cmd              = cmd01_;   // 复制出临界区, CAN 发送不持锁
  }

  const protocol::CanFrame f = protocol::CanProtocol::encodeAdcuCmd01(out_cmd);
  if (!can_driver_->send(f)) {
    // 驱动侧不打印单帧发送失败 (会以 50 Hz 刷屏), 这里限流告警。
    const rclcpp::Time now = this->now();
    if (now - last_send_fail_log_time_ >= rclcpp::Duration::from_seconds(watchdog_log_interval_s_)) {
      last_send_fail_log_time_ = now;
      SPDLOG_ERROR("CAN 发送失败 (ID=0x{:08X}), 底盘未收到指令", f.id);
    }
  }
}

// ---- 上行: CAN 接收线程回调 (VCU -> ADCU) ----
void ChassisNode::onCanReceive(const protocol::CanFrame& frame) {
  switch (frame.id) {
    case protocol::can_id::CHASSIS_STATE_01: {
      protocol::ChassisState01 s;
      if (!protocol::CanProtocol::decodeState01(frame, s)) {
        return;
      }
      const int64_t now_ns = this->now().nanoseconds();
      // 心跳时间戳用 atomic 记录, sendCommand() 无需为读时间戳抢 state_mutex_。
      last_vcu_ns_.store(now_ns, std::memory_order_relaxed);

      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!vcu_alive_initialized_ || s.alive_counter != last_vcu_alive_counter_) {
        last_vcu_alive_counter_ = s.alive_counter;
        vcu_alive_initialized_  = true;
        last_vcu_alive_change_ns_.store(now_ns, std::memory_order_relaxed);
      }
      state01_       = s;
      state01_valid_ = true;

      if (!ctrl_mode_logged_once_ || s.ctrl_mode != last_logged_ctrl_mode_) {
        SPDLOG_INFO("VCU 驾驶模式: {} -> {}", last_logged_ctrl_mode_ ? "无人驾驶" : "遥控", s.ctrl_mode ? "无人驾驶" : "遥控");
        last_logged_ctrl_mode_ = s.ctrl_mode;
        ctrl_mode_logged_once_ = true;
      }
      break;
    }

    case protocol::can_id::CHASSIS_STATE_02: {
      protocol::ChassisState02 s;
      if (!protocol::CanProtocol::decodeState02(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state02_       = s;
      state02_valid_ = true;
      break;
    }

    case protocol::can_id::CHASSIS_STATE_03: {
      protocol::ChassisState03 s;
      if (!protocol::CanProtocol::decodeState03(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state03_       = s;
      state03_valid_ = true;
      break;
    }

    case protocol::can_id::CHASSIS_STATE_04:
    case protocol::can_id::CHASSIS_STATE_05:
    case protocol::can_id::CHASSIS_STATE_06:
    case protocol::can_id::CHASSIS_STATE_07:
    case protocol::can_id::CHASSIS_STATE_08:
    case protocol::can_id::CHASSIS_STATE_09:
    case protocol::can_id::CHASSIS_STATE_10:
    case protocol::can_id::CHASSIS_STATE_11: {
      protocol::MotorElecState s;
      if (!protocol::CanProtocol::decodeMotorElec(frame, s)) {
        return;
      }
      const bool is_drive = frame.id <= protocol::can_id::CHASSIS_STATE_07;
      const uint32_t base = is_drive ? protocol::can_id::CHASSIS_STATE_04 : protocol::can_id::CHASSIS_STATE_08;
      // ID 低 16 位一致, 每下一路增加 0x10000。
      const size_t index = static_cast<size_t>((frame.id - base) >> 16);
      if (index >= kWheelCount) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (is_drive) {
        drive_motors_[index]      = s;
        drive_motor_valid_[index] = true;
      } else {
        steer_motors_[index]      = s;
        steer_motor_valid_[index] = true;
      }
      break;
    }

    case protocol::can_id::CHASSIS_STATE_21: {
      protocol::ChassisState21 s;
      if (!protocol::CanProtocol::decodeState21(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state21_       = s;
      state21_valid_ = true;
      break;
    }

    case protocol::can_id::CHASSIS_STATE_22: {
      protocol::ChassisState22 s;
      if (!protocol::CanProtocol::decodeState22(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state22_       = s;
      state22_valid_ = true;
      if (!bms_error_logged_once_ || s.error_code != last_bms_error_code_) {
        if (s.error_code == 0) {
          SPDLOG_INFO("BMS 故障码: {} -> 0 (恢复正常)", last_bms_error_code_);
        } else {
          SPDLOG_WARN("BMS 故障码: {} -> {}", last_bms_error_code_, s.error_code);
        }
        last_bms_error_code_   = s.error_code;
        bms_error_logged_once_ = true;
      }
      break;
    }

    default:
      break;
  }
}

// ---- 上行: 完整状态 + 兼容文本发布 (publish_hz) ----
void ChassisNode::publishState() {
  auto msg             = std::make_unique<weeder_msgs::msg::ChassisState>();
  msg->header.stamp    = this->now();
  msg->header.frame_id = "base_link";
  protocol::ChassisState01 text_state01;
  protocol::ChassisState02 text_state02;
  protocol::ChassisState03 text_state03;
  {
    // 与 CAN 接收线程共享全部状态, 读取必须加锁, 否则是数据竞争 (UB)。
    std::lock_guard<std::mutex> lock(state_mutex_);
    const protocol::ChassisState01& s1 = state01_;
    msg->state01_valid                 = state01_valid_;
    msg->gear                          = s1.gear;
    msg->steer_mode                    = s1.steer_mode;
    msg->estop                         = s1.estop != 0;
    msg->reset                         = s1.reset != 0;
    msg->release_brake                 = s1.release_brake != 0;
    msg->ctrl_mode                     = s1.ctrl_mode != 0;
    msg->speed_raw                     = s1.speed_raw;
    msg->speed_m_s                     = s1.speed_m_s;
    msg->wheel_track_cm                = s1.wheel_track_cm;
    msg->sound                         = s1.sound != 0;
    msg->light_green                   = s1.light_green != 0;
    msg->light_yellow                  = s1.light_yellow != 0;
    msg->light_red                     = s1.light_red != 0;
    msg->headlight                     = s1.headlight != 0;
    msg->turn_left                     = s1.turn_left != 0;
    msg->turn_right                    = s1.turn_right != 0;
    msg->steer_done_fl                 = s1.steer_done_fl != 0;
    msg->steer_done_fr                 = s1.steer_done_fr != 0;
    msg->steer_done_rl                 = s1.steer_done_rl != 0;
    msg->steer_done_rr                 = s1.steer_done_rr != 0;
    msg->bumper                        = s1.bumper != 0;
    msg->bypass                        = s1.bypass != 0;
    msg->battery_charging              = s1.battery_charging != 0;
    msg->battery_full                  = s1.battery_full != 0;
    msg->alive_counter                 = s1.alive_counter;

    msg->state02_valid   = state02_valid_;
    msg->motor_speed_rpm = {state02_.motor_speed_fl, state02_.motor_speed_fr, state02_.motor_speed_rl, state02_.motor_speed_rr};
    msg->state03_valid   = state03_valid_;
    msg->steer_angle_deg = {
      static_cast<float>(state03_.steer_angle_fl) * 0.1f,
      static_cast<float>(state03_.steer_angle_fr) * 0.1f,
      static_cast<float>(state03_.steer_angle_rl) * 0.1f,
      static_cast<float>(state03_.steer_angle_rr) * 0.1f};

    for (size_t i = 0; i < kWheelCount; ++i) {
      msg->drive_motor_valid[i] = drive_motor_valid_[i];
      msg->drive_bus_voltage[i] = drive_motors_[i].bus_voltage;
      msg->drive_out_current[i] = drive_motors_[i].out_current;
      msg->drive_motor_temp[i]  = drive_motors_[i].motor_temp;
      msg->drive_error_code[i]  = drive_motors_[i].error_code;

      msg->steer_motor_valid[i] = steer_motor_valid_[i];
      msg->steer_bus_voltage[i] = steer_motors_[i].bus_voltage;
      msg->steer_out_current[i] = steer_motors_[i].out_current;
      msg->steer_motor_temp[i]  = steer_motors_[i].motor_temp;
      msg->steer_error_code[i]  = steer_motors_[i].error_code;
    }

    msg->state21_valid            = state21_valid_;
    msg->bms_soc                  = state21_.soc_raw;
    msg->bms_voltage              = state21_.voltage;
    msg->bms_current              = state21_.current;
    msg->bms_full_charge_capacity = state21_.full_charge_capacity;

    msg->state22_valid        = state22_valid_;
    msg->bms_cycle_count      = state22_.cycle_count;
    msg->bms_error_code       = state22_.error_code;
    msg->bms_max_cell_voltage = state22_.max_cell_voltage;
    msg->system_voltage       = state22_.system_voltage;

    text_state01 = state01_;
    text_state02 = state02_;
    text_state03 = state03_;
  }
  state_pub_->publish(std::move(msg));

  std_msgs::msg::String text;
  text.data = fmtState(text_state01, text_state02, text_state03);
  state_text_pub_->publish(text);
}

std::string ChassisNode::fmtState(const protocol::ChassisState01& s1, const protocol::ChassisState02& s2, const protocol::ChassisState03& s3) {
  char buf[384];
  std::snprintf(
    buf,
    sizeof(buf),
    "speed=%u mm/s gear=%u estop=%u mode=%u alive=%u bumper=%u "
    "rpm=[%u,%u,%u,%u] steer=[%.1f,%.1f,%.1f,%.1f]",
    static_cast<unsigned>(s1.speed_raw),
    static_cast<unsigned>(s1.gear),
    static_cast<unsigned>(s1.estop),
    static_cast<unsigned>(s1.ctrl_mode),
    static_cast<unsigned>(s1.alive_counter),
    static_cast<unsigned>(s1.bumper),
    static_cast<unsigned>(s2.motor_speed_fl),
    static_cast<unsigned>(s2.motor_speed_fr),
    static_cast<unsigned>(s2.motor_speed_rl),
    static_cast<unsigned>(s2.motor_speed_rr),
    static_cast<double>(s3.steer_angle_fl) * 0.1,
    static_cast<double>(s3.steer_angle_fr) * 0.1,
    static_cast<double>(s3.steer_angle_rl) * 0.1,
    static_cast<double>(s3.steer_angle_rr) * 0.1);
  return std::string(buf);
}

}   // namespace weeder_base
