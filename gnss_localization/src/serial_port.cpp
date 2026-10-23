#include "gnss_localization/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <system_error>
#include <vector>

namespace gnss_localization {

namespace {

// 生成带系统错误码的可读错误信息。
std::string ErrnoMessage(const std::string& action) {
  return action + ": " + std::error_code(errno, std::generic_category()).message();
}

}   // namespace

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
  close();
}

// 打开串口并配置为 8N1 原始模式；同时申请进程级独占。
bool SerialPort::open(const std::string& device, int baud_rate) {
  std::lock_guard<std::mutex> lock(mutex_);
  CloseUnlocked();

  const std::optional<speed_t> speed = BaudToSpeed(baud_rate);
  if (!speed.has_value()) {
    SetErrorUnlocked("不支持的串口波特率 " + std::to_string(baud_rate) + "（支持 9600/19200/38400/57600/115200/230400/460800/921600）");
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    SetErrorUnlocked(ErrnoMessage("打开串口 " + device + " 失败"));
    return false;
  }

  if (::ioctl(fd_, TIOCEXCL) != 0) {
    SetErrorUnlocked(ErrnoMessage("设置串口独占失败"));
    CloseUnlocked();
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    SetErrorUnlocked(ErrnoMessage("读取串口参数失败"));
    CloseUnlocked();
    return false;
  }

  if (cfsetispeed(&tty, *speed) != 0 || cfsetospeed(&tty, *speed) != 0) {
    SetErrorUnlocked(ErrnoMessage("设置串口波特率失败"));
    CloseUnlocked();
    return false;
  }

  // 8N1、无软硬件流控，并禁止关闭时拉低 DTR。
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag |= CLOCAL | CREAD | CS8;
  tty.c_cflag &= ~HUPCL;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    SetErrorUnlocked(ErrnoMessage("应用串口参数失败"));
    CloseUnlocked();
    return false;
  }
  if (tcflush(fd_, TCIOFLUSH) != 0) {
    SetErrorUnlocked(ErrnoMessage("清空串口缓冲失败"));
    CloseUnlocked();
    return false;
  }

  line_assembler_.Reset();
  last_error_.clear();
  return true;
}

// 关闭串口并清空当前组帧状态。
void SerialPort::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  CloseUnlocked();
}

// 返回当前串口是否仍处于打开状态。
bool SerialPort::is_open() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

// 在整次写入共享的总超时内发送全部字节；
// 致命错误会关闭串口。
bool SerialPort::write_raw(const char* data, std::size_t size, std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    SetErrorUnlocked("串口未打开");
    return false;
  }
  if (data == nullptr && size > 0) {
    SetErrorUnlocked("串口写入数据为空");
    return false;
  }
  if (size == 0) {
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t written = 0;
  while (written < size) {
    if (std::chrono::steady_clock::now() >= deadline) {
      SetErrorUnlocked("串口写入超时");
      CloseUnlocked();
      return false;
    }
    const ssize_t count = ::write(fd_, data + written, size - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count == 0) {
      SetErrorUnlocked("串口写入返回 0 字节");
      CloseUnlocked();
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      SetErrorUnlocked(ErrnoMessage("串口写入失败"));
      CloseUnlocked();
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      SetErrorUnlocked("串口写入超时");
      CloseUnlocked();
      return false;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const int wait_ms    = static_cast<int>(std::max<int64_t>(1, remaining.count()));

    pollfd pfd{fd_, POLLOUT, 0};
    const int poll_result = ::poll(&pfd, 1, wait_ms);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      SetErrorUnlocked(ErrnoMessage("等待串口可写失败"));
      CloseUnlocked();
      return false;
    }
    if (poll_result == 0) {
      SetErrorUnlocked("串口写入超时");
      CloseUnlocked();
      return false;
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      SetErrorUnlocked("串口写通道已失效");
      CloseUnlocked();
      return false;
    }
  }
  return true;
}

// 非阻塞读取当前所有可用字节，
// 并在锁外回调完整 NMEA 行。
int SerialPort::read_lines(const std::function<void(const std::string&)>& line_callback) {
  std::vector<std::string> lines;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) {
      SetErrorUnlocked("串口未打开");
      return 0;
    }

    while (true) {
      pollfd pfd{fd_, POLLIN, 0};
      const int poll_result = ::poll(&pfd, 1, 0);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        SetErrorUnlocked(ErrnoMessage("串口读取轮询失败"));
        CloseUnlocked();
        break;
      }
      if (poll_result == 0) {
        break;
      }
      if ((pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        SetErrorUnlocked("串口读通道已失效");
        CloseUnlocked();
        break;
      }
      if ((pfd.revents & POLLIN) == 0) {
        break;
      }

      const ssize_t count = ::read(fd_, read_buf_, kReadBufSize);
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        SetErrorUnlocked(ErrnoMessage("串口读取失败"));
        CloseUnlocked();
        break;
      }
      if (count == 0) {
        SetErrorUnlocked("串口读取返回 0 字节");
        CloseUnlocked();
        break;
      }

      line_assembler_.Push(read_buf_, static_cast<std::size_t>(count), [&lines](const std::string& line) {
        lines.push_back(line);
      });
    }
  }

  if (line_callback) {
    for (const std::string& line : lines) {
      line_callback(line);
    }
  }
  return static_cast<int>(lines.size());
}

// 返回最近一次串口操作的错误描述。
std::string SerialPort::last_error() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

// 将标准波特率映射为 termios 常量；不支持时返回空。
std::optional<speed_t> SerialPort::BaudToSpeed(int baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return std::nullopt;
  }
}

// 在已持有互斥锁时关闭 fd 并清空组帧状态。
void SerialPort::CloseUnlocked() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  line_assembler_.Reset();
}

// 在已持有互斥锁时保存错误信息。
void SerialPort::SetErrorUnlocked(const std::string& message) {
  last_error_ = message;
}

}   // namespace gnss_localization
