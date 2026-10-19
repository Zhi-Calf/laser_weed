// SocketCAN 驱动实现 (Linux)。
//
// 与 tractor_base/src/can_driver.cpp 同源, 两处差异:
//   1. weeder_base::protocol::CanFrame 没有 timestamp_us 字段, 不填时间戳;
//   2. 滤波由 setFilters() 下发 CAN_RAW_FILTER (tractor_base 没有这个开关)。
#include "weeder_base/can_driver.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace weeder_base {

CanDriver::CanDriver(const std::string& interface) : interface_(interface) {}

CanDriver::~CanDriver() {
  // 顺序不可换: 先 join 接收线程, 再关 socket。反过来的话, 阻塞在 read() 上的
  // 线程会拿到 EBADF/半帧数据, 却仍然持有 this。
  stopReceiveThread();
  close();
}

bool CanDriver::open() {
  if (socket_fd_ >= 0) {
    return true;   // 幂等
  }

  // 1. 创建套接字
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    const int err = errno;
    SPDLOG_ERROR("创建 CAN 套接字失败: {}", std::strerror(err));
    socket_fd_ = -1;
    return false;
  }

  // 2. 取接口索引 (接口不存在时这里失败, 报 ENODEV)
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    const int err = errno;
    SPDLOG_ERROR("获取 CAN 接口 {} 的索引失败: {}", interface_, std::strerror(err));
    close();
    return false;
  }

  // 3. 绑定套接字到接口
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int err = errno;
    SPDLOG_ERROR("绑定 CAN 接口 {} 失败: {}", interface_, std::strerror(err));
    close();
    return false;
  }

  // 4. 接收超时 100ms: 接收线程靠这个周期醒来检查 running_, 否则
  //    stopReceiveThread() 会一直卡在阻塞 read() 上无法 join。
  struct timeval tv;
  tv.tv_sec  = 0;
  tv.tv_usec = 100000;
  if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    // 非致命: 只是让停线程慢一点, 不值得为它放弃整条 CAN 链路。
    const int err = errno;
    SPDLOG_WARN("设置 CAN 接收超时失败: {}", std::strerror(err));
  }

  // 5. 若有待生效的滤波表 (setFilters 在 open 之前被调过), 此时下发
  applyFilters();

  SPDLOG_INFO("CAN 接口 {} 已打开 (fd={})", interface_, socket_fd_);
  return true;
}

void CanDriver::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanDriver::send(const protocol::CanFrame& frame) {
  if (socket_fd_ < 0) {
    SPDLOG_WARN("CAN 未打开, 丢弃发送帧 ID=0x{:08X}", frame.id);
    return false;
  }
  if (frame.dlc > 8) {
    SPDLOG_ERROR("CAN 帧 DLC={} 非法 (>8), 丢弃 ID=0x{:08X}", frame.dlc, frame.id);
    return false;
  }

  struct can_frame cf;
  std::memset(&cf, 0, sizeof(cf));
  // 本协议全部是 29 位扩展帧 (见 weeder_base/protocol/can_protocol.hpp),
  // 但驱动不假设这一点: 标准帧走 SFF 掩码, 扩展帧才带 CAN_EFF_FLAG。
  cf.can_id  = frame.is_extended ? static_cast<canid_t>((frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG) : static_cast<canid_t>(frame.id & CAN_SFF_MASK);
  cf.can_dlc = frame.dlc;
  std::memcpy(cf.data, frame.data, frame.dlc);

  std::lock_guard<std::mutex> lock(send_mutex_);
  const ssize_t nbytes = ::write(socket_fd_, &cf, sizeof(cf));
  if (nbytes != static_cast<ssize_t>(sizeof(cf))) {
    // 调用方 (ChassisNode) 负责限流告警, 这里不重复打日志。
    return false;
  }
  return true;
}

void CanDriver::setReceiveCallback(CanReceiveCallback callback) {
  receive_callback_ = std::move(callback);
}

void CanDriver::startReceiveThread() {
  if (running_) {
    return;
  }
  if (socket_fd_ < 0) {
    // 在 fd=-1 上 read() 会立刻返回 EBADF, 变成空转刷日志的死循环。
    SPDLOG_ERROR("CAN 未打开, 不启动接收线程");
    return;
  }
  running_        = true;
  receive_thread_ = std::thread(&CanDriver::receiveLoop, this);
}

void CanDriver::stopReceiveThread() {
  running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void CanDriver::setFilters(const std::vector<uint32_t>& ids) {
  filters_ = ids;
  applyFilters();
}

void CanDriver::applyFilters() {
  if (socket_fd_ < 0) {
    return;   // open() 末尾会再调一次
  }

  if (filters_.empty()) {
    // 空表 = 恢复全收 (len=0 即清掉内核里的滤波表)
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0) {
      const int err = errno;
      SPDLOG_WARN("取消 CAN 滤波失败: {}", std::strerror(err));
      return;
    }
    SPDLOG_INFO("CAN 滤波: 全收");
    return;
  }

  std::vector<struct can_filter> raw;
  raw.reserve(filters_.size());
  for (const uint32_t id : filters_) {
    struct can_filter f;
    // 收到帧的 can_id 带 CAN_EFF_FLAG, 所以扩展帧的掩码也必须把它算进去,
    // 否则永远匹配不上。
    if ((id & ~static_cast<uint32_t>(CAN_SFF_MASK)) != 0) {
      f.can_id   = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
      f.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
    } else {
      f.can_id   = id & CAN_SFF_MASK;
      f.can_mask = CAN_SFF_MASK;
    }
    raw.push_back(f);
  }

  const socklen_t len = static_cast<socklen_t>(raw.size() * sizeof(struct can_filter));
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, raw.data(), len) < 0) {
    const int err = errno;
    SPDLOG_ERROR("下发 CAN 滤波表失败 ({} 项): {}", raw.size(), std::strerror(err));
    return;
  }
  SPDLOG_INFO("CAN 滤波: 只收 {} 个 ID", raw.size());
}

void CanDriver::receiveLoop() {
  struct can_frame cf;

  while (running_) {
    const ssize_t nbytes = ::read(socket_fd_, &cf, sizeof(cf));
    if (nbytes < 0) {
      const int err = errno;
      // SO_RCVTIMEO 到期 (EAGAIN/EWOULDBLOCK) 是正常情况: 回到 while 检查 running_。
      if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
        continue;
      }
      if (running_) {
        SPDLOG_ERROR("CAN 接收失败: {}", std::strerror(err));
        // 接口被拔掉/关闭时会持续失败, 退避 100ms 免得刷爆日志。
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      continue;
    }
    if (nbytes != static_cast<ssize_t>(sizeof(cf)) || !receive_callback_) {
      continue;
    }

    protocol::CanFrame frame;
    frame.is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    frame.id          = frame.is_extended ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
    frame.dlc         = cf.can_dlc > 8 ? 8 : cf.can_dlc;   // 经典 CAN 不会 >8, 防御性截断
    std::memcpy(frame.data, cf.data, frame.dlc);

    receive_callback_(frame);
  }
}

}   // namespace weeder_base
