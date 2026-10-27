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

  // 使用 CAN 接口名创建驱动。
  explicit CanDriver(const std::string& interface = "can0");
  // 停止接收线程并关闭 socket。
  ~CanDriver();

  CanDriver(const CanDriver&)            = delete;
  CanDriver& operator=(const CanDriver&) = delete;

  // 打开并绑定 SocketCAN 接口。
  bool open();
  // 停止接收线程并关闭 socket。
  void close();
  // 返回 socket 是否处于打开状态。
  bool isOpen() const {
    return socket_fd_ >= 0;
  }

  // 发送一帧 CAN 数据。
  bool send(const protocol::CanFrame& frame);
  // 设置接收线程的帧回调。
  void setReceiveCallback(CanReceiveCallback callback);
  // 启动 CAN 接收线程。
  void startReceiveThread();
  // 停止并等待 CAN 接收线程退出。
  void stopReceiveThread();

  // 设置内核态 CAN ID 过滤表，空列表表示接收全部帧。
  void setFilters(const std::vector<uint32_t>& ids);

private:
  // 轮询 socket 并分发接收帧。
  void receiveLoop();
  // 将当前过滤表应用到 socket。
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
