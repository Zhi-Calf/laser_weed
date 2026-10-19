#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <weeder_msgs/msg/heartbeat.hpp>
#include <weeder_msgs/msg/system_health.hpp>

#include "tractor_monitor/watch_table.hpp"

namespace tractor_monitor {

// Watches every module EXCEPT tractor_base, and heartbeats a single aggregate
// verdict to tractor_base. Those two halves are the whole node.
//
// tractor_base is excluded structurally, not by config discipline: it is the
// CONSUMER of the verdict, so including it would make the stop condition depend
// on the node being stopped.
class MonitorNode : public rclcpp::Node {
public:
  explicit MonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // loadParameters() 之前单独跑: 它自己就会打 WARN/ERROR(未识别的 criticality、
  // 非法 timeout、监控列表为空), 日志器必须先配好, 否则这些行只到终端、进不了
  // tractor_monitor.log —— 而它们正是事后要翻的东西。
  void loadLogParameters();
  void loadParameters();
  void onHeartbeat(const weeder_msgs::msg::Heartbeat::SharedPtr msg);
  void onCheckTimer();
  double nowSeconds() const;

  std::unique_ptr<WatchTable> table_;

  rclcpp::Subscription<weeder_msgs::msg::Heartbeat>::SharedPtr hb_sub_;
  rclcpp::Publisher<weeder_msgs::msg::Heartbeat>::SharedPtr verdict_pub_;
  rclcpp::Publisher<weeder_msgs::msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  double heartbeat_rate_    = 10.0;
  double check_rate_        = 10.0;
  double startup_grace_s_   = 15.0;
  double default_timeout_s_ = 1.0;

  // spdlog 滚动日志
  std::string log_dir_;
  int log_file_size_mb_    = 50;
  int log_file_count_      = 10;
  bool log_enable_console_ = true;
  std::string log_level_;

  // RCLCPP_*_THROTTLE 的替代: spdlog 没有节流宏, 手动记上次打印时刻。
  // 初值必须是"纪元零点且用节点时钟的类型"(见构造函数), 不能是 this->now():
  // 后者会把第一条消息吞掉一个完整间隔, 而 RCLCPP_*_THROTTLE 是必打第一条的。
  double unknown_node_log_interval_s_  = 60.0;
  double status_repeat_log_interval_s_ = 5.0;
  rclcpp::Time last_unknown_node_log_time_;
  rclcpp::Time last_unknown_status_log_time_;
  rclcpp::Time last_status_repeat_log_time_;

  uint32_t seq_               = 0;
  uint8_t last_logged_status_ = 255;   // force a log on the first evaluation
};

}   // namespace tractor_monitor
