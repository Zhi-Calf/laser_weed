#pragma once

#include "weeder_base/can_bridge.hpp"  // 提供 CanFrame（经桥接引入 laser_weed::CanFrame）

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace weeder_base {

// SocketCAN 驱动（Linux 专用）。
// 取代原先的桩实现：真正 open/bind/收发，并起独立接收线程。
//
// 线程安全约定：
//   - setReceiveCallback() 必须在 startReceiveThread() 之前调用；
//     或在 stopReceiveThread() 之后调用（本类不保护 callback_ 的并发读写）。
//   - 析构会先停线程再关 socket，因此不要在接收回调里销毁本对象。
class CanDriver {
public:
  using CanReceiveCallback = std::function<void(const CanFrame&)>;

  explicit CanDriver(const std::string& interface);
  ~CanDriver();

  bool open();
  void close();
  bool send(const CanFrame& frame);
  void setReceiveCallback(CanReceiveCallback cb);
  void startReceiveThread();
  void stopReceiveThread();
  void setFilters(const std::vector<uint32_t>& ids);

private:
  void rxLoop();

  std::string interface_;
  int socket_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  CanReceiveCallback callback_;
  std::vector<uint32_t> filters_;
  rclcpp::Logger get_logger() { return rclcpp::get_logger("CanDriver"); }
};

} // namespace weeder_base
