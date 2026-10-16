#include "tractor_monitor/monitor_node.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <system_error>
#include <vector>

#include "tractor_monitor/heartbeat_publisher.hpp"

namespace tractor_monitor {

using tractor_msgs::msg::Heartbeat;
using tractor_msgs::msg::NodeHealth;
using tractor_msgs::msg::SystemHealth;

// watch_table.hpp keeps its own Status enum so it can stay message-free. That
// only works if the two never drift.
static_assert(static_cast<uint8_t>(Status::kOk) == Heartbeat::STATUS_OK, "");
static_assert(static_cast<uint8_t>(Status::kStopImplement) == Heartbeat::STATUS_STOP_IMPLEMENT, "");
static_assert(static_cast<uint8_t>(Status::kStopMotion) == Heartbeat::STATUS_STOP_MOTION, "");
static_assert(static_cast<uint8_t>(Status::kFault) == Heartbeat::STATUS_FAULT, "");

namespace {

uint8_t criticalityToMsg(Criticality c) {
  switch (c) {
    case Criticality::kCritical:
      return NodeHealth::CRIT_CRITICAL;
    case Criticality::kDegraded:
      return NodeHealth::CRIT_DEGRADED;
    case Criticality::kIgnore:
      return NodeHealth::CRIT_IGNORE;
  }
  return NodeHealth::CRIT_DEGRADED;
}

// Never watched, whatever the config says. See the class comment.
bool isChassisNode(const std::string& name) {
  return name == "tractor_base" || name == "chassis_interface" || name == "chassis_interface_node";
}

/// Expand a leading ~ and any $VAR in a configured path.
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

/// A writable log directory: the configured one when it can be created,
/// otherwise $ROS_HOME/logs (~/.ros/logs by ROS convention).
///
/// The shipped defaults (/opt/agri/logs, and /home/asd/logs in the yamls) are
/// the tractor's paths: one needs root, the other needs a user named asd. On
/// any other machine create_directories threw, the caller caught it, and file
/// logging was dropped entirely for the run -- so a laptop or CI reproduction
/// had no rotating log to look at afterwards. Falling back keeps the logs.
/// Where the configured path works, nothing changes.
///
/// Returns empty when nothing is writable, which tells the caller to throw and
/// fall back to console-only, exactly as before.
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

void ConfigureLogger(const std::string& log_dir, size_t max_file_size_bytes, size_t max_file_count, bool enable_console, const std::string& log_level) {
  constexpr const char* kLogPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";
  const std::string dir             = ResolveLogDir(log_dir);
  if (dir.empty()) {
    throw std::runtime_error("no writable log directory (tried \"" + log_dir + "\")");
  }

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((std::filesystem::path(dir) / "tractor_monitor.log").string(), max_file_size_bytes, max_file_count));
  if (enable_console) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>("tractor_monitor", sinks.begin(), sinks.end());
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
    SPDLOG_WARN("日志目录 \"{}\" 不可写，已回退到 \"{}\"", log_dir, dir);
  }
  spdlog::flush_on(spdlog::level::warn);
  spdlog::flush_every(std::chrono::seconds(3));
}

}   // namespace

double MonitorNode::nowSeconds() const {
  return this->now().seconds();
}

void MonitorNode::loadLogParameters() {
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", log_file_size_mb_);
  declare_parameter("log_file_count", log_file_count_);
  declare_parameter("log_enable_console", log_enable_console_);
  declare_parameter("log_level", "info");
  declare_parameter("unknown_node_log_interval_s", unknown_node_log_interval_s_);
  declare_parameter("status_repeat_log_interval_s", status_repeat_log_interval_s_);

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();

  unknown_node_log_interval_s_  = get_parameter("unknown_node_log_interval_s").as_double();
  status_repeat_log_interval_s_ = get_parameter("status_repeat_log_interval_s").as_double();
}

void MonitorNode::loadParameters() {
  declare_parameter("heartbeat_rate", heartbeat_rate_);
  declare_parameter("check_rate", check_rate_);
  declare_parameter("startup_grace_s", startup_grace_s_);
  declare_parameter("default_timeout_s", default_timeout_s_);
  declare_parameter("monitored_nodes", std::vector<std::string>{});

  heartbeat_rate_    = get_parameter("heartbeat_rate").as_double();
  check_rate_        = get_parameter("check_rate").as_double();
  startup_grace_s_   = get_parameter("startup_grace_s").as_double();
  default_timeout_s_ = get_parameter("default_timeout_s").as_double();

  const auto names = get_parameter("monitored_nodes").as_string_array();

  std::vector<WatchConfig> configs;
  std::set<std::string> seen;
  for (const auto& name : names) {
    if (name.empty()) {
      continue;
    }
    if (isChassisNode(name)) {
      SPDLOG_WARN(
        "monitored_nodes 含有底盘节点 '{}' —— 已忽略。tractor_base 是本节点裁决结果的"
        "消费者，不能同时是被监控对象（否则停车条件依赖于被停的节点）。",
        name);
      continue;
    }
    if (!seen.insert(name).second) {
      SPDLOG_WARN("monitored_nodes 中 '{}' 重复，已忽略重复项", name);
      continue;
    }

    WatchConfig cfg;
    cfg.name = name;

    declare_parameter(name + ".timeout_s", default_timeout_s_);
    declare_parameter(name + ".criticality", std::string("degraded"));
    declare_parameter(name + ".required", true);
    cfg.timeout_s = get_parameter(name + ".timeout_s").as_double();
    cfg.required  = get_parameter(name + ".required").as_bool();

    const auto crit_str = get_parameter(name + ".criticality").as_string();
    bool crit_ok        = false;
    cfg.criticality     = criticalityFromString(crit_str, &crit_ok);
    if (!crit_ok) {
      // Loud, because the operator asked for something specific and did not get
      // it. Falls back to degraded: a typo must neither stop the tractor nor
      // silently disarm the watch.
      SPDLOG_ERROR(
        "{}.criticality = '{}' 无法识别（应为 critical|degraded|ignore），"
        "按 degraded 处理：该模块丢失只上报、不停车",
        name,
        crit_str);
    }
    if (cfg.timeout_s <= 0.0) {
      SPDLOG_ERROR("{}.timeout_s = {:.3f} 非法，改用 {:.3f}", name, cfg.timeout_s, default_timeout_s_);
      cfg.timeout_s = default_timeout_s_;
    }

    configs.push_back(cfg);
  }

  SPDLOG_INFO("==== tractor_monitor 配置 ====");
  SPDLOG_INFO("心跳发布频率: {:.1f} Hz -> {}", heartbeat_rate_, kMonitorHeartbeatTopic);
  SPDLOG_INFO("检查频率: {:.1f} Hz, 启动宽限期: {:.1f} s", check_rate_, startup_grace_s_);
  SPDLOG_INFO("日志级别: {}, 目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}", log_level_, log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "是" : "否");
  for (const auto& c : configs) {
    SPDLOG_INFO("  监控 {:<28} timeout={:.2f}s  {:<8} {}", c.name, c.timeout_s, toString(c.criticality), c.required ? "required" : "optional(出现后才监控)");
  }
  if (configs.empty()) {
    // Not fatal, but it means the aggregate is permanently OK and tractor_base
    // will never be told to stop. Say so rather than looking healthy.
    SPDLOG_WARN(
      "监控列表为空 —— 本节点将持续上报 OK，tractor_base 永远不会因监控而停车。"
      "请检查 monitored_nodes 参数是否加载。");
  }
  SPDLOG_INFO("==============================");

  table_ = std::make_unique<WatchTable>(std::move(configs), startup_grace_s_, nowSeconds());
}

MonitorNode::MonitorNode(const rclcpp::NodeOptions& options) : Node("tractor_monitor", options) {
  // 日志器先于 loadParameters(): 后者自己就会打配置校验的 WARN/ERROR, 顺序反了
  // 那些行就只到终端、进不了 tractor_monitor.log。
  loadLogParameters();
  try {
    ConfigureLogger(log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024U * 1024U, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", e.what());
  }

  // 纪元零点, 且用节点时钟的类型。默认构造的 rclcpp::Time 是 RCL_SYSTEM_TIME,
  // 与 this->now() 的 RCL_ROS_TIME 相减会抛 —— 第一次节流检查就崩。
  // 用零点而不是 this->now(), 是为了保持 RCLCPP_*_THROTTLE 的语义: 第一条必打。
  last_unknown_node_log_time_  = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_unknown_status_log_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_status_repeat_log_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  loadParameters();

  hb_sub_      = create_subscription<Heartbeat>(kHeartbeatTopic, 50, std::bind(&MonitorNode::onHeartbeat, this, std::placeholders::_1));
  verdict_pub_ = create_publisher<Heartbeat>(kMonitorHeartbeatTopic, 10);

  // 全表快照。/monitor_heartbeat 只有聚合裁决(tractor_base 消费的那个),
  // /heartbeat 只有各模块自报的流 —— 想一眼看到每个节点现在什么状态, 看这个。
  health_pub_ = create_publisher<SystemHealth>(kSystemHealthTopic, 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(check_rate_, 0.1));
  check_timer_      = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&MonitorNode::onCheckTimer, this));
}

void MonitorNode::onHeartbeat(const Heartbeat::SharedPtr msg) {
  // Liveness is judged on ARRIVAL time, not on msg->header.stamp. A node whose
  // clock is wrong, or which stamps a message it then sits on, must not be able
  // to look fresher (or staler) than it is.
  // 未知 status 按 kFault 收下, 不原样透传: 见 statusFromWire。最可能的成因是
  // 对端还是旧版本消息定义 —— 那种情况下这条心跳的其余字段也不可信, 停车是唯一
  // 安全的解释。
  bool status_known = true;
  const Status status = statusFromWire(msg->status, &status_known);
  if (!status_known) {
    const rclcpp::Time now = this->now();
    if ((now - last_unknown_status_log_time_).seconds() >= unknown_node_log_interval_s_) {
      SPDLOG_WARN("节点 '{}' 上报了未知 status={}（按 FAULT 处理，通常意味着消息版本不一致）",
                  msg->node_name, static_cast<int>(msg->status));
      last_unknown_status_log_time_ = now;
    }
  }

  const bool known = table_->observe(msg->node_name, status, msg->detail, msg->seq, nowSeconds(), msg->fault_source_node);
  if (!known) {
    // Throttled, not once-only: a node babbling on the bus that nobody watches
    // is worth noticing more than once, but not at its publish rate.
    const rclcpp::Time now = this->now();
    if ((now - last_unknown_node_log_time_).seconds() >= unknown_node_log_interval_s_) {
      SPDLOG_INFO("收到未在监控列表中的节点心跳: '{}'（已忽略）", msg->node_name);
      last_unknown_node_log_time_ = now;
    }
  }
}

void MonitorNode::onCheckTimer() {
  const Verdict v = table_->evaluate(nowSeconds());

  Heartbeat msg;
  msg.header.stamp = this->now();
  msg.node_name    = "tractor_monitor";
  msg.status       = static_cast<uint8_t>(v.status);
  msg.seq          = seq_++;
  msg.detail       = v.detail;
  // 透传"导致裁决的模块"自报的来源 (空 = 都是它们自己的问题; 多个去重并列): monitor 只搬运,
  // 不替人拍板 —— 合并上报后 APP 靠它直接定位责任方。
  msg.fault_source_node = v.fault_source_node;
  verdict_pub_->publish(msg);

  SystemHealth health;
  health.header.stamp     = msg.header.stamp;
  health.overall          = msg.status;
  health.in_startup_grace = v.in_startup_grace;
  health.nodes.reserve(table_->modules().size());
  for (const auto& kv : table_->modules()) {
    const ModuleState& m = kv.second;
    NodeHealth nh;
    nh.node_name   = m.config.name;
    nh.status      = static_cast<uint8_t>(m.effective);
    nh.alive       = m.alive;
    nh.ever_seen   = m.ever_seen;
    nh.age_s       = m.age_s;
    nh.criticality = criticalityToMsg(m.config.criticality);
    nh.counted     = m.counted;
    nh.restarts    = m.restarts;
    nh.detail      = m.last_detail;
    health.nodes.push_back(std::move(nh));
  }
  health_pub_->publish(health);

  // Log on every TRANSITION, then throttle. A status that never changes should
  // not fill the log, but the moment it changes is the line an operator greps
  // for afterwards.
  if (msg.status != last_logged_status_) {
    last_logged_status_ = msg.status;
    if (v.status == Status::kFault) {
      SPDLOG_ERROR("系统状态 -> FAULT（全局停止: 停运动+停作业+退出自主）: {}", v.detail);
    } else if (v.status == Status::kStopMotion) {
      SPDLOG_WARN("系统状态 -> STOP_MOTION（停止跟踪/运动）: {}", v.detail);
    } else if (v.status == Status::kStopImplement) {
      SPDLOG_WARN("系统状态 -> STOP_IMPLEMENT（停止作业）: {}", v.detail);
    } else {
      SPDLOG_INFO("系统状态 -> OK（全部被监控模块心跳正常）");
    }
  } else if (v.status != Status::kOk) {
    // msg.header.stamp 上面刚取过同一个时钟, 直接复用。
    const rclcpp::Time now(msg.header.stamp, get_clock()->get_clock_type());
    if ((now - last_status_repeat_log_time_).seconds() >= status_repeat_log_interval_s_) {
      SPDLOG_WARN("系统状态 {}: {}", toString(v.status), v.detail);
      last_status_repeat_log_time_ = now;
    }
  }
}

}   // namespace tractor_monitor
