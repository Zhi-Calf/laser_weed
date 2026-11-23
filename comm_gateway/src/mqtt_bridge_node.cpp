#include "comm_gateway/mqtt_bridge_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <thread>
#include <utility>

#include "comm_gateway/logging.hpp"

namespace comm_gateway {

using vehicle_interfaces::msg::ChassisState;
using vehicle_interfaces::msg::GnssStatus;
using vehicle_interfaces::msg::WorkMode;

namespace {

constexpr const char* kNodeName = "mqtt_bridge_node";
constexpr int kQosAtMostOnce  = 0;
constexpr int kQosAtLeastOnce = 1;

// inbox 上限: 下行异常刷屏时宁可丢最旧的, 也不让内存无限涨。
constexpr size_t kInboxMax = 200;
// commandId 去重窗口
constexpr size_t kRecentCommandMax = 256;
// 断线期间暂存的可靠事件上限 (超过丢最旧的)
constexpr size_t kOutboxMax = 500;
// drain 周期: 下行指令最多多等这么久
constexpr auto kDrainPeriod = std::chrono::milliseconds(20);

std::chrono::nanoseconds PeriodFromHz(double hz) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / std::max(hz, 0.1)));
}

// WorkMode.mode_name 的约定写法 (见 WorkMode.msg)
const char* WorkModeName(uint8_t mode) {
  switch (mode) {
    case WorkMode::MODE_UNMANNED:
      return "unmanned";
    case WorkMode::MODE_CALIBRATION:
      return "calibration";
  }
  return "manual";
}

std::string Now() {
  return farm::FormatRfc3339(std::chrono::system_clock::now());
}

}   // namespace

MqttBridgeNode::MqttBridgeNode(const rclcpp::NodeOptions& options) : Node(kNodeName, options) {
  loadLogParameters();
  try {
    ConfigureLogger(kNodeName, log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024 * 1024, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
  } catch (const std::exception& e) {
    SPDLOG_ERROR("日志初始化失败, 仅输出到终端: {}", e.what());
  }

  if (!loadParameters()) {
    // 设备身份配错: 连上也会被网关拒收或认错设备，不启动连接。
    SPDLOG_ERROR("设备身份参数无效，未启动 MQTT 连接");
    return;
  }

  setupRos();
  setupMqtt();
}

MqttBridgeNode::~MqttBridgeNode() {
  if (mqtt_) {
    if (mqtt_->connected()) {
      // 主动下线: 网关也会按连接生命周期生成离线, 这里补上原因。
      publishEvent(topics_.status, farm::type::kDeviceOffline, session_trace_id_, farm::DeviceOfflinePayload("SHUTDOWN"), false);
      // publish 只是入队; 给网络线程一点时间把它写出去再断开。
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    mqtt_.reset();   // 先停网络线程, 之后回调不会再进来
  }
}

void MqttBridgeNode::loadLogParameters() {
  declare_parameter("log_dir", "/home/asd/logs");
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

bool MqttBridgeNode::loadParameters() {
  declare_parameter("product_key", "tractor_v1");
  declare_parameter("device_name", "robot_001");
  declare_parameter("device_id", "111");
  declare_parameter("device_secret", "");
  declare_parameter("mqtt_host", "127.0.0.1");
  declare_parameter("mqtt_port", 1883);
  declare_parameter("mqtt_keepalive_s", 60);
  declare_parameter("mqtt_tls_ca_file", "");
  declare_parameter("mqtt_reconnect_min_s", 1);
  declare_parameter("mqtt_reconnect_max_s", 10);
  declare_parameter("telemetry_rate_hz", telemetry_rate_hz_);
  declare_parameter("work_mode_rate_hz", work_mode_rate_hz_);
  declare_parameter("data_timeout_s", data_timeout_s_);

  product_key_                 = get_parameter("product_key").as_string();
  device_name_                 = get_parameter("device_name").as_string();
  device_id_                   = get_parameter("device_id").as_string();
  const std::string secret     = get_parameter("device_secret").as_string();
  mqtt_config_.host            = get_parameter("mqtt_host").as_string();
  mqtt_config_.port            = static_cast<int>(get_parameter("mqtt_port").as_int());
  // keepalive 至少保留 5 s，避免错误配置导致链路被过早判定超时。
  mqtt_config_.keepalive_s     = std::max(5, static_cast<int>(get_parameter("mqtt_keepalive_s").as_int()));
  mqtt_config_.tls_ca_file     = ExpandPath(get_parameter("mqtt_tls_ca_file").as_string());
  mqtt_config_.reconnect_min_s = std::max(1, static_cast<int>(get_parameter("mqtt_reconnect_min_s").as_int()));
  mqtt_config_.reconnect_max_s = std::max(mqtt_config_.reconnect_min_s, static_cast<int>(get_parameter("mqtt_reconnect_max_s").as_int()));
  telemetry_rate_hz_           = get_parameter("telemetry_rate_hz").as_double();
  // 下游 3 s 不更新即停车: 续发至少 1 Hz 才留得出余量
  work_mode_rate_hz_           = std::max(1.0, get_parameter("work_mode_rate_hz").as_double());
  data_timeout_s_              = get_parameter("data_timeout_s").as_double();

  if (product_key_.empty() || device_name_.empty() || !farm::IsValidDeviceId(device_id_)) {
    SPDLOG_ERROR("设备身份无效: product_key=\"{}\" device_name=\"{}\" device_id=\"{}\" (device_id 应为平台分配的正整数, 无前导零)", product_key_, device_name_, device_id_);
    return false;
  }

  // §2.2 设备三元组认证。没配 secret 时匿名连接, 只适合本机 mosquitto 联调。
  mqtt_config_.client_id = farm::MqttClientId(product_key_, device_name_);
  if (!secret.empty()) {
    mqtt_config_.username = farm::MqttUsername(product_key_, device_name_);
    mqtt_config_.password = farm::MqttPassword(product_key_, device_name_, secret);
  }
  // 不注册 LWT: 网关按连接生命周期生成离线事件, 文档明确不要旧格式的 online 遗嘱 (§2.2)。

  topics_           = farm::Topics::For(product_key_, device_name_);
  session_trace_id_ = farm::NewId("trace-" + device_id_);

  SPDLOG_INFO("MQTT broker: {}:{} (client_id={}, 认证={}, TLS={}, keepalive={} s)", mqtt_config_.host, mqtt_config_.port, mqtt_config_.client_id,
              secret.empty() ? "匿名 (未配 device_secret)" : "设备三元组 HMAC", mqtt_config_.tls_ca_file.empty() ? "关" : mqtt_config_.tls_ca_file, mqtt_config_.keepalive_s);
  SPDLOG_INFO("FarmDevice v{}: deviceId={}, 下行 {}", farm::kSchemaRevision, device_id_, topics_.control_downstream);
  SPDLOG_INFO("遥测 {:.1f} Hz, /work_mode 续发 {:.1f} Hz, 数据过期 {:.1f} s", telemetry_rate_hz_, work_mode_rate_hz_, data_timeout_s_);
  if (secret.empty()) {
    SPDLOG_WARN("device_secret 为空: 以匿名方式连接, robot-cloud 网关会拒绝认证");
  }
  return true;
}

void MqttBridgeNode::setupRos() {
  // /work_mode 的订阅方都是 reliable + transient_local (见 path_controller), 发布方必须一致,
  // 否则 QoS 不兼容, 一帧都收不到。
  work_mode_pub_ = create_publisher<WorkMode>(kWorkModeTopic, rclcpp::QoS(1).reliable().transient_local());

  gnss_sub_ = create_subscription<GnssStatus>(kGnssStatusTopic, rclcpp::SensorDataQoS(), [this](const GnssStatus::SharedPtr msg) {
    gnss_       = *msg;
    gnss_stamp_ = Clock::now();
  });
  chassis_sub_ = create_subscription<ChassisState>(kChassisStateTopic, 10, [this](const ChassisState::SharedPtr msg) {
    chassis_       = *msg;
    chassis_stamp_ = Clock::now();
  });

  drain_timer_     = create_wall_timer(kDrainPeriod, std::bind(&MqttBridgeNode::drainInbox, this));
  telemetry_timer_ = create_wall_timer(PeriodFromHz(telemetry_rate_hz_), std::bind(&MqttBridgeNode::publishTelemetry, this));
  work_mode_timer_ = create_wall_timer(PeriodFromHz(work_mode_rate_hz_), std::bind(&MqttBridgeNode::publishWorkMode, this));
}

void MqttBridgeNode::setupMqtt() {
  mqtt_ = std::make_unique<MqttClient>(
      mqtt_config_,
      [this](bool connected, const std::string& reason) {
        onMqttConnection(connected, reason);
      },
      [this](MqttClient::Message msg) {
        onMqttMessage(std::move(msg));
      });

  std::string err;
  if (!mqtt_->start(err)) {
    // 配置级错误（如 TLS 证书）重试不会恢复，节点继续运行但不连接。
    SPDLOG_ERROR("MQTT 启动失败: {}", err);
  }
}

// ---------------------------------------------------------------------------
// MQTT 网络线程
// ---------------------------------------------------------------------------

void MqttBridgeNode::onMqttConnection(bool connected, const std::string& reason) {
  if (!connected) {
    SPDLOG_WARN("MQTT 未连接: {}", reason);
    // 断链后 /work_mode 停止续发，下游按自身超时逻辑停车。
    disconnected_event_.store(true);
    return;
  }

  SPDLOG_INFO("MQTT 已连接 {}:{}", mqtt_config_.host, mqtt_config_.port);
  // clean session: 每次 (重) 连都要重新订阅 (§2.3)。
  mqtt_->subscribe(topics_.control_downstream, kQosAtLeastOnce);
  connected_event_.store(true);
}

void MqttBridgeNode::onMqttMessage(MqttClient::Message msg) {
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  if (inbox_.size() >= kInboxMax) {
    inbox_.pop_front();
    SPDLOG_WARN("下行消息积压超过 {} 条, 丢弃最旧的一条", kInboxMax);
  }
  inbox_.push_back(std::move(msg));
}

// ---------------------------------------------------------------------------
// 执行器线程: 连接
// ---------------------------------------------------------------------------

void MqttBridgeNode::drainInbox() {
  // 断开先于连接处理: 一次 drain 里两者都发生时, 以"最后连上"收尾。
  if (disconnected_event_.exchange(false)) {
    onDisconnected();
  }
  if (connected_event_.exchange(false)) {
    onConnected();
  }

  std::deque<MqttClient::Message> batch;
  {
    std::lock_guard<std::mutex> lock(inbox_mutex_);
    batch.swap(inbox_);
  }
  for (const auto& msg : batch) {
    handleCommand(msg);
  }
}

void MqttBridgeNode::onConnected() {
  if (!mqtt_->connected()) {
    return;   // 刚连上又断了: 等下一次连接事件
  }
  // 上线 -> 断线期间暂存的可靠事件 -> 当前模式 (§2.3 第 4、6 步): 按发生顺序, 当前状态最后到
  publishEvent(topics_.status, farm::type::kDeviceOnline, session_trace_id_, farm::DeviceOnlinePayload(), false);
  flushOutbox();
  publishEvent(topics_.status, farm::type::kModeStatus, session_trace_id_, farm::ModeStatusPayload(farm::ModeFromRos(held_mode_), "ACTIVE", std::nullopt, std::nullopt, ""), false);
  publishWorkMode();
}

void MqttBridgeNode::onDisconnected() {
  if (held_mode_ == WorkMode::MODE_MANUAL) {
    return;
  }
  // 宁可停车: 立刻发一帧 MANUAL 解除无人 / 标定, 之后不再续发 (未连接时 publishWorkMode 不发)。
  // 重连后保持 MANUAL, 要恢复作业必须由平台重新下发 MODE_SET。
  SPDLOG_WARN("与网关断开: 工作模式由 {} 复位为 MANUAL", farm::ModeFromRos(held_mode_));
  held_mode_ = WorkMode::MODE_MANUAL;
  WorkMode out;
  out.header.stamp = now();
  out.mode         = held_mode_;
  out.mode_name    = WorkModeName(held_mode_);
  work_mode_pub_->publish(out);
  publishEvent(topics_.status, farm::type::kModeStatus, session_trace_id_, farm::ModeStatusPayload("MANUAL", "ACTIVE", std::nullopt, std::nullopt, "与网关断开, 已复位为手动"), true);
}

// ---------------------------------------------------------------------------
// 执行器线程: 下行
// ---------------------------------------------------------------------------

void MqttBridgeNode::handleCommand(const MqttClient::Message& msg) {
  if (msg.topic != topics_.control_downstream) {
    SPDLOG_WARN("收到未订阅的 topic: {}", msg.topic);
    return;
  }
  if (msg.retained) {
    SPDLOG_WARN("丢弃 retain 的下行指令 (broker 存的旧指令): {}", msg.payload);
    return;
  }

  farm::Command cmd;
  std::string ack_code;
  std::string err;
  const auto result = farm::ParseCommand(msg.payload, device_id_, std::chrono::system_clock::now(), cmd, ack_code, err);
  if (result == farm::ParseResult::kDrop) {
    SPDLOG_WARN("丢弃下行指令 ({}): {}", err, msg.payload);
    return;
  }
  if (replayIfDuplicate(cmd.command_id)) {
    return;
  }
  if (result == farm::ParseResult::kReject) {
    SPDLOG_WARN("拒绝指令 commandId={} type={}: [{}] {}", cmd.command_id, cmd.message_type, ack_code, err);
    sendAck(cmd.command_id, cmd.trace_id, farm::stage::kRejected, ack_code, err);
    return;
  }

  SPDLOG_INFO("指令 commandId={} type={} payload={}", cmd.command_id, cmd.message_type, cmd.payload.dump());
  if (cmd.message_type == farm::type::kModeSet) {
    handleModeSet(cmd);
  } else if (cmd.message_type == "EMERGENCY_STOP") {
    SPDLOG_ERROR("收到 EMERGENCY_STOP, 但远程急停尚未接入 —— 已回 REJECTED, 车辆不会因此停车");
    sendAck(cmd.command_id, cmd.trace_id, farm::stage::kRejected, farm::code::kUnsupported, "远程急停尚未接入本设备, 请使用车上急停");
  } else {
    sendAck(cmd.command_id, cmd.trace_id, farm::stage::kRejected, farm::code::kUnsupported, "本设备尚未支持 " + cmd.message_type);
  }
}

bool MqttBridgeNode::replayIfDuplicate(const std::string& command_id) {
  auto it = recent_command_acks_.find(command_id);
  if (it != recent_command_acks_.end()) {
    SPDLOG_INFO("重复的指令 commandId={}, 重发已有的 {} 条 ACK", command_id, it->second.size());
    for (const auto& raw : it->second) {
      publishReliable(topics_.control_upstream, raw);
    }
    return true;
  }
  recent_command_acks_.emplace(command_id, std::vector<std::string>{});
  recent_command_order_.push_back(command_id);
  if (recent_command_order_.size() > kRecentCommandMax) {
    recent_command_acks_.erase(recent_command_order_.front());
    recent_command_order_.pop_front();
  }
  return false;
}

void MqttBridgeNode::handleModeSet(const farm::Command& cmd) {
  farm::ModeSet req;
  std::string err;
  if (!farm::ParseModeSet(cmd.payload, req, err)) {
    sendAck(cmd.command_id, cmd.trace_id, farm::stage::kRejected, farm::code::kInvalidMessage, err);
    return;
  }
  const auto mode = farm::ModeToRos(req.mode);
  if (!mode) {
    sendAck(cmd.command_id, cmd.trace_id, farm::stage::kRejected, farm::code::kUnsupported, "本设备没有 " + req.mode + " 模式 (支持 MANUAL / UNMANNED / CALIBRATION)");
    return;
  }

  sendAck(cmd.command_id, cmd.trace_id, farm::stage::kReceived, farm::code::kOk, "");
  held_mode_ = *mode;
  publishWorkMode();
  SPDLOG_INFO("工作模式 -> {} (commandId={})", req.mode, cmd.command_id);
  sendAck(cmd.command_id, cmd.trace_id, farm::stage::kCompleted, farm::code::kOk, "已切换为 " + req.mode);
  // ACTIVE = 本节点已开始按该模式续发 /work_mode。下游是否真的进入作业 (有无路径、定位是否就绪)
  // 由各节点心跳反映, 见 FAULT_EVENT 与遥测 healthStatus。
  publishEvent(topics_.status, farm::type::kModeStatus, cmd.trace_id, farm::ModeStatusPayload(req.mode, "ACTIVE", cmd.command_id, req.task_id, ""), true);
}

// ---------------------------------------------------------------------------
// 执行器线程: 上行
// ---------------------------------------------------------------------------

void MqttBridgeNode::sendAck(const std::string& command_id, const std::string& trace_id, const std::string& stage, const std::string& code, const std::string& message,
                             std::optional<farm::json> result) {
  farm::UpstreamHeader h{device_id_, farm::type::kCommandAck, farm::NewId("evt"), trace_id, Now(), command_id};
  const std::string raw = farm::EncodeUpstream(h, farm::AckPayload(stage, code, message, std::move(result)));
  auto it               = recent_command_acks_.find(command_id);
  if (it != recent_command_acks_.end()) {
    it->second.push_back(raw);
  }
  publishReliable(topics_.control_upstream, raw);
}

void MqttBridgeNode::publishEvent(const std::string& topic, const char* message_type, const std::string& trace_id, farm::json payload, bool reliable) {
  farm::UpstreamHeader h{device_id_, message_type, farm::NewId("evt"), trace_id, Now(), std::nullopt};
  const std::string raw = farm::EncodeUpstream(h, std::move(payload));
  if (reliable) {
    publishReliable(topic, raw);
  } else if (mqtt_) {
    mqtt_->publish(topic, raw, kQosAtLeastOnce, false);
  }
}

void MqttBridgeNode::publishReliable(const std::string& topic, const std::string& raw) {
  // 有积压时排到队尾再按序补发, 保证同一设备的可靠事件不乱序。
  if (outbox_.empty() && mqtt_ && mqtt_->publish(topic, raw, kQosAtLeastOnce, false)) {
    return;
  }
  if (outbox_.size() >= kOutboxMax) {
    SPDLOG_WARN("断线暂存的可靠事件超过 {} 条, 丢弃最旧的一条", kOutboxMax);
    outbox_.pop_front();
  }
  outbox_.emplace_back(topic, raw);
  flushOutbox();
}

void MqttBridgeNode::flushOutbox() {
  size_t sent = 0;
  while (!outbox_.empty() && mqtt_ && mqtt_->publish(outbox_.front().first, outbox_.front().second, kQosAtLeastOnce, false)) {
    outbox_.pop_front();
    ++sent;
  }
  if (sent > 0) {
    SPDLOG_INFO("补发断线期间的可靠事件 {} 条", sent);
  }
}

void MqttBridgeNode::publishWorkMode() {
  // 只在连着网关时续发: 断链即停, 下游 3 s 后停车 (见类注释)。
  if (!mqtt_ || !mqtt_->connected()) {
    return;
  }
  WorkMode out;
  out.header.stamp = now();
  out.mode         = held_mode_;
  out.mode_name    = WorkModeName(held_mode_);
  work_mode_pub_->publish(out);
}

void MqttBridgeNode::publishTelemetry() {
  if (!mqtt_ || !mqtt_->connected()) {
    return;
  }
  const auto now     = Clock::now();
  const auto timeout = std::chrono::duration<double>(data_timeout_s_);
  const GnssStatus* gnss      = (gnss_ && now - gnss_stamp_ <= timeout) ? &*gnss_ : nullptr;
  const ChassisState* chassis = (chassis_ && now - chassis_stamp_ <= timeout) ? &*chassis_ : nullptr;

  farm::TelemetryExtras extras;
  extras.control_mode = farm::ModeFromRos(held_mode_);

  farm::UpstreamHeader h{device_id_, farm::type::kTelemetry, farm::NewId("evt"), session_trace_id_, Now(), std::nullopt};
  // 遥测高频且只看最新值: QoS 0, 断线不暂存 (§4.3)
  mqtt_->publish(topics_.telemetry, farm::EncodeUpstream(h, farm::TelemetryPayload(gnss, chassis, extras)), kQosAtMostOnce, false);
}

}   // namespace comm_gateway
