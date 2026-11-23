#include "comm_gateway/mqtt_client.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <system_error>
#include <utility>

#include <mqtt/async_client.h>

namespace comm_gateway {

namespace {

// 构造 Paho 连接 URI。TLS 是否启用只由 CA 文件是否配置决定。
std::string MakeServerUri(const MqttClient::Config& config) {
  const std::string scheme = config.tls_ca_file.empty() ? "tcp://" : "ssl://";
  return scheme + config.host + ":" + std::to_string(config.port);
}

// 在 Paho 异常和通用标准异常边界上提取可读原因。
std::string ExceptionReason(const std::exception& error) {
  return error.what() == nullptr ? "未知错误" : error.what();
}

}   // namespace

// 把 Paho 异步 connect token 的首次失败转成节点可记录的连接事件。
class MqttClient::ConnectListener : public mqtt::iaction_listener {
 public:
  explicit ConnectListener(MqttClient& owner) : owner_(owner) {}

  void on_failure(const mqtt::token& token) override {
    if (owner_.stopping_.load() || !owner_.on_connection_) {
      return;
    }
    owner_.connected_.store(false);
    owner_.on_connection_(
        false, "MQTT 连接失败，将自动重连: " +
                   mqtt::exception::error_str(token.get_return_code()));
  }

  void on_success(const mqtt::token& /*token*/) override {}

 private:
  MqttClient& owner_;  // 生命周期由 MqttClient 保证
};

MqttClient::MqttClient(Config config, ConnectionCallback on_connection,
                       MessageCallback on_message)
    : config_(std::move(config)),
      on_connection_(std::move(on_connection)),
      on_message_(std::move(on_message)) {}

MqttClient::~MqttClient() {
  stopping_.store(true);
  connected_.store(false);
  if (!client_) {
    return;
  }

  // 主动断开不会触发 broker 的 LWT；离线事件由节点在析构前自行发布。
  try {
    if (client_->is_connected()) {
      client_->disconnect()->wait_for(2000);
    }
    client_->disable_callbacks();
  } catch (const std::exception&) {
    // 析构路径不能向节点传播异常，Paho 后台线程会随 client_ 一起释放。
  }
  client_.reset();
  connect_listener_.reset();
}

bool MqttClient::start(std::string& err) {
  if (client_) {
    err = "MQTT 客户端已经启动";
    return false;
  }
  if (config_.host.empty() || config_.port <= 0 || config_.port > 65535) {
    err = "MQTT broker 地址无效 (host=\"" + config_.host +
          "\", port=" + std::to_string(config_.port) + ")";
    return false;
  }
  if (config_.will_qos < 0 || config_.will_qos > 2) {
    err = "LWT QoS 必须在 0~2 之间";
    return false;
  }
  if (!config_.tls_ca_file.empty()) {
    std::error_code file_error;
    if (!std::filesystem::exists(config_.tls_ca_file, file_error)) {
      err = file_error ? "无法访问 TLS CA 文件: " + config_.tls_ca_file +
                             " (" + file_error.message() + ")"
                       : "TLS CA 文件不存在: " + config_.tls_ca_file;
      return false;
    }
  }

  try {
    client_ = std::make_unique<mqtt::async_client>(
        MakeServerUri(config_), config_.client_id);
    setupCallbacks();
    connect_listener_ = std::make_unique<ConnectListener>(*this);

    mqtt::connect_options options;
    options.set_mqtt_version(MQTTVERSION_3_1_1);
    options.set_clean_session(true);
    options.set_keep_alive_interval(std::max(config_.keepalive_s, 5));
    options.set_automatic_reconnect(
        std::max(config_.reconnect_min_s, 1),
        std::max(config_.reconnect_max_s, config_.reconnect_min_s));
    if (!config_.username.empty()) {
      options.set_user_name(config_.username);
      options.set_password(config_.password);
    }
    if (!config_.tls_ca_file.empty()) {
      mqtt::ssl_options tls;
      tls.set_trust_store(config_.tls_ca_file);
      tls.set_verify(true);
      options.set_ssl(tls);
    }
    if (!config_.will_topic.empty()) {
      mqtt::will_options will(config_.will_topic, config_.will_payload,
                              config_.will_qos, config_.will_retain);
      options.set_will(will);
    }
    client_->connect(options, nullptr, *connect_listener_);
  } catch (const std::exception& error) {
    err = "Paho MQTT 启动失败: " + ExceptionReason(error);
    client_.reset();
    connect_listener_.reset();
    return false;
  }
  return true;
}

bool MqttClient::publish(const std::string& topic,
                         const std::string& payload,
                         int qos,
                         bool retain) {
  if (!client_ || !connected_.load() || stopping_.load()) {
    return false;
  }
  try {
    client_->publish(topic, payload.data(), payload.size(), qos, retain);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool MqttClient::subscribe(const std::string& topic, int qos) {
  if (!client_ || stopping_.load()) {
    return false;
  }
  try {
    client_->subscribe(topic, qos);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

// 注册 Paho 回调。回调只转交连接状态和消息，不直接修改 ROS 节点状态。
void MqttClient::setupCallbacks() {
  client_->set_connected_handler([this](const std::string& cause) {
    connected_.store(true);
    if (!stopping_.load() && on_connection_) {
      on_connection_(true, cause.empty() ? "已连接 MQTT broker" : cause);
    }
  });
  client_->set_connection_lost_handler([this](const std::string& cause) {
    connected_.store(false);
    if (!stopping_.load() && on_connection_) {
      on_connection_(false, cause.empty() ? "MQTT 连接断开" : cause);
    }
  });
  client_->set_message_callback([this](mqtt::const_message_ptr msg) {
    if (!msg || stopping_.load() || !on_message_) {
      return;
    }
    Message out;
    out.topic = msg->get_topic();
    out.payload = msg->get_payload_str();
    out.retained = msg->is_retained();
    on_message_(std::move(out));
  });
}

}   // namespace comm_gateway
