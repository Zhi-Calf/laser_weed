#pragma once

#include <memory>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <thread>

namespace httplib {
class Server;
struct Request;
struct Response;
}   // namespace httplib

namespace weeder_comm_gateway {

// ---- 对接的 ROS 接口 ----
inline constexpr const char* kGlobalPathTopic = "/global_path_planner/path";
inline constexpr const char* kGnssOriginTopic = "/gnss_driver/origin";

// APP <-> 农机的 HTTP 服务。协议: doc/APP接口_MQTT_HTTP_v0.1.md §4。
//
//   PUT  /api/v1/path/file      APP 上传路径文件 -> 校验 -> 落盘到 path_file
//   POST /api/v1/path/download  农机按 url 下载路径文件 -> 校验 -> 落盘到 path_file
//   GET  /api/v1/path           当前规划路径 (ENU -> 经纬度)
//
// 只负责"把文件放到位": 让规划器加载仍要 APP 走 MQTT 的 set_source (path_source=1)。
// 这样换路径只有一个入口, 且受规划器"仅手动模式可切"的约束。
//
// 线程: httplib 在自己的线程池里跑处理函数; ROS 订阅回调在执行器线程。两边只通过 path_mutex_
// 保护的快照交换数据。落盘由 write_mutex_ 串行化 (上传与下载不会交错写同一个文件)。
class HttpBridgeNode : public rclcpp::Node {
public:
  explicit HttpBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HttpBridgeNode() override;

private:
  void loadLogParameters();
  void loadParameters();
  void setupRoutes();
  void startServer();

  // 返回 false 时 res 已填好错误应答
  bool authorized(const httplib::Request& req, httplib::Response& res) const;
  void handlePutPathFile(const httplib::Request& req, httplib::Response& res);
  void handlePostPathDownload(const httplib::Request& req, httplib::Response& res);
  void handleGetPath(const httplib::Request& req, httplib::Response& res);

  // 校验 + 原子落盘。成功返回 true; 失败时 status/message 是要回给 APP 的。
  bool validateAndSave(const std::string& content, int& status, std::string& message);

  // ---- 参数 ----
  std::string bind_address_;
  int port_ = 8080;
  std::string tls_cert_file_;
  std::string tls_key_file_;
  std::string auth_token_;
  std::string path_file_;
  size_t max_body_bytes_     = 20 * 1024 * 1024;
  double download_timeout_s_ = 30.0;

  std::string log_dir_;
  int log_file_size_mb_    = 50;
  int log_file_count_      = 10;
  bool log_enable_console_ = true;
  std::string log_level_;

  std::unique_ptr<httplib::Server> server_;
  std::thread server_thread_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;

  std::mutex path_mutex_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  std::optional<sensor_msgs::msg::NavSatFix> origin_;

  std::mutex write_mutex_;
};

}   // namespace weeder_comm_gateway
