#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "weeder_base/protocol/can_protocol.hpp"

namespace weeder_base {

// ============================================================
// SocketCAN 驱动 (Linux 专用)
// ============================================================
//
// 与 MegaTrak tractor_base 的同名驱动同源:
//   open()  = socket(PF_CAN, SOCK_RAW, CAN_RAW) + ioctl(SIOCGIFINDEX) + bind()
//   收帧    = 独立线程轮询 (SO_RCVTIMEO 100ms), 转成 CanFrame 后回调
//   发帧    = 加锁串行化 write(), 多定时器不会交错写坏一帧
//
// 线程安全约定:
//   - setReceiveCallback() 必须在 startReceiveThread() 之前, 或
//     stopReceiveThread() 之后调用 (本类不保护 callback_ 的并发读写)。
//   - 接收线程用 SO_RCVTIMEO 超时轮询而不是阻塞 read(), 所以
//     stopReceiveThread() 能在一个超时周期内 join 返回; 析构先停线程再关
//     socket, 因此在接收回调里销毁本对象是安全的。
class CanDriver {
public:
  using CanReceiveCallback = std::function<void(const protocol::CanFrame&)>;

  explicit CanDriver(const std::string& interface = "can0");
  ~CanDriver();

  CanDriver(const CanDriver&)            = delete;
  CanDriver& operator=(const CanDriver&) = delete;

  bool open();
  void close();
  bool isOpen() const {
    return socket_fd_ >= 0;
  }

  bool send(const protocol::CanFrame& frame);
  void setReceiveCallback(CanReceiveCallback callback);
  void startReceiveThread();
  void stopReceiveThread();

  // 只接收给定的 CAN ID (CAN_RAW_FILTER, 内核态过滤)。
  //   29 位扩展帧 ID 自动带 CAN_EFF_FLAG, 11 位标准帧原样使用;
  //   空列表 = 全收 (默认, 也是 reset 的方式)。
  // 未 open() 时只记下, 由 open() 生效。注意: 表里漏掉的 ID 会在内核里被丢掉,
  // 不回调 onCanReceive —— 将来解码 State_02~23 时记得同步加进滤波表。
  void setFilters(const std::vector<uint32_t>& ids);

private:
  void receiveLoop();
  void applyFilters();

private:
  CanReceiveCallback receive_callback_;
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  std::mutex send_mutex_;   // 串行化 send()

private:
  std::string interface_;           // CAN 接口名
  int socket_fd_ = -1;              // CAN 套接字 fd, -1 = 未打开
  std::vector<uint32_t> filters_;   // 只接收的 CAN ID (空 = 全收)
};

}   // namespace weeder_base
