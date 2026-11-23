#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vehicle_interfaces/msg/chassis_state.hpp>
#include <vehicle_interfaces/msg/gnss_status.hpp>
#include <vehicle_interfaces/msg/work_mode.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "comm_gateway/farm_device.hpp"
#include "comm_gateway/mqtt_client.hpp"

namespace comm_gateway {

// ---- 对接的 ROS 接口 (绝对名, 与各节点的实际发布/服务名一致) ----
inline constexpr const char* kWorkModeTopic          = "/work_mode";
inline constexpr const char* kGnssStatusTopic        = "/gnss_driver/status";
inline constexpr const char* kChassisStateTopic      = "/chassis_interface/chassis_state";

// 农机 <-> robot-cloud IoT 网关的 MQTT 桥。协议: FarmDevice v1 (见 farm_device.hpp)。
//
// 线程模型: MQTT 网络线程只把收到的消息塞进 inbox_、把连接变化记进两个原子标志, 由执行器线程上的
// drain 定时器统一处理, 所以所有 ROS 状态缓存都只在执行器线程里读写。
// 上行方向: 执行器线程直接调 MqttClient::publish (Paho 保证调用线程安全)。
//
// 工作模式: MODE_SET 是单发指令, 而 path_tracker_node 要求 /work_mode 持续刷新
// (超时后按未 arm 停车)。所以本节点保存平台最后一次设定的模式, 在**与网关保持连接期间**按
// work_mode_rate_hz 续发。安全约定 (都是"宁可停车"方向):
//   * 断开网关即停止续发 -> 3 s 后下游停车;
//   * 断开时把保存的模式复位为 MANUAL -> 重连后不会自动恢复无人作业, 必须重新下发 MODE_SET;
//   * 下行不使用 retain (§2.2), 收到 retain 的指令一律丢弃 —— 那是 broker 存的旧指令。
class MqttBridgeNode : public rclcpp::Node {
public:
  explicit MqttBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MqttBridgeNode() override;

private:
  using Clock = std::chrono::steady_clock;

  void loadLogParameters();
  bool loadParameters();
  void setupRos();
  void setupMqtt();

  // MQTT 网络线程
  void onMqttConnection(bool connected, const std::string& reason);
  void onMqttMessage(MqttClient::Message msg);

  // 执行器线程: 连接
  void drainInbox();
  void onConnected();
  void onDisconnected();

  // 执行器线程: 下行
  void handleCommand(const MqttClient::Message& msg);
  bool replayIfDuplicate(const std::string& command_id);
  void handleModeSet(const farm::Command& cmd);

  // 执行器线程: 上行
  void sendAck(const std::string& command_id, const std::string& trace_id, const std::string& stage, const std::string& code, const std::string& message,
               std::optional<farm::json> result = std::nullopt);
  void publishEvent(const std::string& topic, const char* message_type, const std::string& trace_id, farm::json payload, bool reliable);
  void publishReliable(const std::string& topic, const std::string& raw);
  void flushOutbox();
  void publishWorkMode();
  void publishTelemetry();

  // ---- 参数 ----
  std::string product_key_;
  std::string device_name_;
  std::string device_id_;
  MqttClient::Config mqtt_config_;
  double telemetry_rate_hz_ = 1.0;
  double work_mode_rate_hz_ = 2.0;
  double data_timeout_s_    = 1.0;

  std::string log_dir_;
  int log_file_size_mb_    = 50;
  int log_file_count_      = 10;
  bool log_enable_console_ = true;
  std::string log_level_;

  farm::Topics topics_;
  std::string session_trace_id_;   // 设备自发事件 (上线 / 遥测 / 故障) 共用的 traceId

  std::unique_ptr<MqttClient> mqtt_;

  // MQTT 线程 -> 执行器线程
  std::mutex inbox_mutex_;
  std::deque<MqttClient::Message> inbox_;
  std::atomic<bool> connected_event_{false};
  std::atomic<bool> disconnected_event_{false};

  // ---- ROS ----
  rclcpp::Publisher<vehicle_interfaces::msg::WorkMode>::SharedPtr work_mode_pub_;

  rclcpp::Subscription<vehicle_interfaces::msg::GnssStatus>::SharedPtr gnss_sub_;
  rclcpp::Subscription<vehicle_interfaces::msg::ChassisState>::SharedPtr
      chassis_sub_;

  rclcpp::TimerBase::SharedPtr drain_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr work_mode_timer_;

  // ---- 执行器线程独占的状态 ----
  uint8_t held_mode_ = vehicle_interfaces::msg::WorkMode::MODE_MANUAL;   // 平台最后一次设定的模式

  // commandId 去重 (QoS 1 可能重复投递): 记下每条指令已发出的 ACK 原文, 重复时原样重发
  // (§5.1: 重投同一个事件保持 eventId、timestamp 和内容不变)。
  std::deque<std::string> recent_command_order_;
  std::unordered_map<std::string, std::vector<std::string>> recent_command_acks_;

  // 可靠事件 (ACK / 状态 / 故障) 在断线期间暂存, 重连后按序补发 (§4.3)。只在内存里, 进程重启即丢。
  std::deque<std::pair<std::string, std::string>> outbox_;

  std::optional<vehicle_interfaces::msg::GnssStatus> gnss_;
  Clock::time_point gnss_stamp_;
  std::optional<vehicle_interfaces::msg::ChassisState> chassis_;
  Clock::time_point chassis_stamp_;
};

}   // namespace comm_gateway
