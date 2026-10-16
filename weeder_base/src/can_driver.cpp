// CanDriver 最小桩：提供节点所需的 open/close/send/setFilters/接收线程接口
// 真实环境替换为完整 SocketCAN 实现即可
// （本桩不依赖 rclcpp，便于纯 C++ 环境验证；ROS2 环境下直接用真实 CanDriver 即可）
#include "weeder_base/can_driver.hpp"
#include <iostream>

namespace weeder_base {

CanDriver::CanDriver(const std::string& interface) : interface_(interface) {}

bool CanDriver::open() {
  socket_fd_ = 1;  // 桩：标记已"打开"
  std::cout << "[CanDriver桩] 已打开接口 " << interface_ << std::endl;
  return true;
}

void CanDriver::close() {
  socket_fd_ = -1;
}

bool CanDriver::send(const CanFrame& frame) {
  // 桩：打印前几帧便于观察，真实实现走 SocketCAN write()
  static int cnt = 0;
  if (cnt++ < 5) {
    std::cout << "[CanDriver桩] 发送 ID=0x" << std::hex << frame.id << std::dec
              << " d[1]=" << (int)frame.data[1]
              << " d[3]=" << (int)(int8_t)frame.data[3]
              << " d[7]=" << (int)frame.data[7] << std::endl;
  }
  return true;
}

void CanDriver::setReceiveCallback(CanReceiveCallback cb) {
  callback_ = std::move(cb);
}

void CanDriver::startReceiveThread() {
  running_ = true;
  // 桩：不实际读 socket，由测试代码手动喂帧
}

void CanDriver::stopReceiveThread() {
  running_ = false;
}

void CanDriver::setFilters(const std::vector<uint32_t>& ids) {
  filters_ = ids;
  std::cout << "[CanDriver桩] 设置 " << ids.size() << " 个滤波 ID" << std::endl;
}
CanDriver::~CanDriver() {
  stopReceiveThread();
  if (socket_fd_ >= 0) {
    close();
  }
}
}  // namespace weeder_base
