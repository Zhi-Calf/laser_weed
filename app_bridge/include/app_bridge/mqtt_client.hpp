#ifndef APP_BRIDGE_MQTT_CLIENT_H_
#define APP_BRIDGE_MQTT_CLIENT_H_

// Eclipse Paho MQTT C++ 的薄封装。
//
// 线程：Paho 内部线程调用连接与消息回调。回调里不访问 ROS 状态，只把消息
// 放入节点队列；节点执行器线程负责后续处理。publish() / subscribe() 可从
// 任意线程调用。

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace mqtt {
class async_client;
}  // namespace mqtt

namespace app_bridge {

class MqttClient {
public:
  struct Config {
    std::string host = "127.0.0.1";  // broker 主机名或 IP
    int port = 1883;                 // broker TCP 端口
    std::string client_id;           // MQTT clientId
    std::string username;            // 空 = 匿名
    std::string password;            // 设备认证密码
    int keepalive_s = 10;            // keepalive 周期，s
    std::string tls_ca_file;         // 空 = 不启用 TLS
    int reconnect_min_s = 1;         // 首次重连等待，s
    int reconnect_max_s = 10;        // 指数退避上限，s
    std::string will_topic;          // 空 = 不注册 LWT
    std::string will_payload;        // LWT 消息体
    int will_qos = 1;                // LWT QoS，取值 0~2
    bool will_retain = true;         // LWT 是否保留
  };

  struct Message {
    std::string topic;           // 消息主题
    std::string payload;         // 消息体，按原始字节保存
    bool retained = false;       // broker 的 retain 标志
  };

  using ConnectionCallback =
      std::function<void(bool connected, const std::string& reason)>;
  using MessageCallback = std::function<void(Message msg)>;

  MqttClient(Config config, ConnectionCallback on_connection,
             MessageCallback on_message);
  ~MqttClient();

  MqttClient(const MqttClient&) = delete;
  MqttClient& operator=(const MqttClient&) = delete;

  // 配置并启动异步客户端；只有本地配置无效时返回 false。
  bool start(std::string& err);

  // 未连接时返回 false，由节点可靠消息队列决定是否暂存。
  bool publish(const std::string& topic, const std::string& payload, int qos,
               bool retain);
  bool subscribe(const std::string& topic, int qos);

  bool connected() const {
    return connected_.load();
  }

 private:
  class ConnectListener;

  void setupCallbacks();

  Config config_;                        // 建连参数
  ConnectionCallback on_connection_;     // 连接状态回调
  MessageCallback on_message_;           // 下行消息回调
  std::unique_ptr<ConnectListener> connect_listener_;  // 首次连接结果
  std::unique_ptr<mqtt::async_client> client_;  // Paho 异步客户端
  std::atomic<bool> connected_{false};   // 当前连接状态
  std::atomic<bool> stopping_{false};    // 析构时抑制迟到回调
};

}   // namespace app_bridge

#endif  // APP_BRIDGE_MQTT_CLIENT_H_
