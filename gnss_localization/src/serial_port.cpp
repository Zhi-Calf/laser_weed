#include "gnss_localization/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

namespace gnss_localization {

namespace {

// 生成包含系统错误码的可读信息。
std::string ErrnoMessage(int error_code, const std::string& action) {
  return action + ": " + std::error_code(error_code, std::generic_category()).message();
}

}   // namespace

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
  close();
}

// 打开串口并配置为 8N1 原始模式。
bool SerialPort::open(const std::string& device, int baud_rate) {
  const std::optional<speed_t> speed = BaudToSpeed(baud_rate);
  if (!speed.has_value()) {
    SPDLOG_ERROR(
      "不支持的串口波特率 {}（支持 "
      "9600/19200/38400/57600/115200/230400/460800/921600）",
      baud_rate);
    return false;
  }

  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    const int error_code = errno;
    SPDLOG_ERROR("{}", ErrnoMessage(error_code, "打开串口 " + device + " 失败"));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const int error_code = errno;
    SPDLOG_ERROR("{}", ErrnoMessage(error_code, "读取串口 " + device + " 参数失败"));
    close();
    return false;
  }

  if (cfsetispeed(&tty, *speed) != 0 || cfsetospeed(&tty, *speed) != 0) {
    const int error_code = errno;
    SPDLOG_ERROR("{}", ErrnoMessage(error_code, "设置串口 " + device + " 波特率失败"));
    close();
    return false;
  }

  // 8N1: 8 数据位、无校验、1 停止位。
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag |= CLOCAL | CREAD | CS8;

  // 关闭时禁止拉低 DTR，避免接收机复位。
  tty.c_cflag &= ~HUPCL;

  // 原始输入，不做流控或转义。
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // 原始输出，不做后处理。
  tty.c_oflag &= ~OPOST;

  // 关闭规范模式、回显和信号。
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const int error_code = errno;
    SPDLOG_ERROR("{}", ErrnoMessage(error_code, "应用串口 " + device + " 参数失败"));
    close();
    return false;
  }
  if (tcflush(fd_, TCIOFLUSH) != 0) {
    const int error_code = errno;
    SPDLOG_ERROR("{}", ErrnoMessage(error_code, "清空串口 " + device + " 缓冲失败"));
    close();
    return false;
  }

  line_pos_ = 0;
  return true;
}

// 关闭串口并清空当前行缓冲。
void SerialPort::close() {
  if (fd_ >= 0) {
    const int fd = fd_;
    fd_          = -1;
    if (::close(fd) != 0) {
      const int error_code = errno;
      SPDLOG_ERROR("关闭串口失败: {}", ErrnoMessage(error_code, "close"));
    }
  }
  line_pos_ = 0;
}

// 返回串口是否已打开。
bool SerialPort::is_open() const {
  return fd_ >= 0;
}

// 写入原始字节，缓冲区满时最多等待 1 秒。
bool SerialPort::write_raw(const char* data, std::size_t size) {
  if (fd_ < 0) {
    SPDLOG_ERROR("串口写入失败: 串口未打开");
    return false;
  }
  if (data == nullptr) {
    if (size == 0) {
      return true;
    }
    SPDLOG_ERROR("串口写入失败: 数据为空");
    return false;
  }

  std::size_t written = 0;
  while (written < size) {
    const ssize_t count = ::write(fd_, data + written, size - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count == 0) {
      SPDLOG_ERROR("串口写入失败: write 返回 0 字节");
      close();
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      pollfd pfd{fd_, POLLOUT, 0};
      const int poll_result = ::poll(&pfd, 1, 1000);
      if (poll_result < 0) {
        const int error_code = errno;
        if (error_code == EINTR) {
          continue;
        }
        SPDLOG_ERROR("{}", ErrnoMessage(error_code, "等待串口可写失败"));
        close();
        return false;
      }
      if (poll_result == 0) {
        SPDLOG_ERROR("串口写入超时");
        close();
        return false;
      }
      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        SPDLOG_ERROR("串口写通道已失效");
        close();
        return false;
      }
      continue;
    }
    const int error_code = errno;
    SPDLOG_ERROR("{}", ErrnoMessage(error_code, "串口写入失败"));
    close();
    return false;
  }
  return true;
}

// 非阻塞读取完整行，返回本次处理的行数。
int SerialPort::read_lines(const std::function<void(const std::string&)>& line_callback) {
  if (fd_ < 0) {
    return 0;
  }

  int lines_processed = 0;
  while (true) {
    pollfd pfd{fd_, POLLIN, 0};
    const int poll_result = ::poll(&pfd, 1, 0);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      const int error_code = errno;
      SPDLOG_ERROR("{}", ErrnoMessage(error_code, "串口读取轮询失败"));
      close();
      break;
    }
    if (poll_result == 0) {
      break;
    }
    if ((pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      SPDLOG_ERROR("串口读通道已失效");
      close();
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
      const int error_code = errno;
      SPDLOG_ERROR("{}", ErrnoMessage(error_code, "串口读取失败"));
      close();
      break;
    }
    if (count == 0) {
      break;
    }

    for (ssize_t i = 0; i < count; ++i) {
      const unsigned char byte = static_cast<unsigned char>(read_buf_[i]);
      if (byte == '\n' || byte == '\r') {
        std::size_t length = line_pos_;
        if (length > 0 && line_buf_[length - 1] == '\r') {
          --length;
        }
        if (length > 0) {
          line_callback(std::string(line_buf_, length));
          ++lines_processed;
        }
        line_pos_ = 0;
        continue;
      }
      if (byte == '$') {
        if (line_pos_ > 0) {
          if (line_buf_[0] != '$' || line_pos_ > 1) {
            line_callback(std::string(line_buf_, line_pos_));
            ++lines_processed;
          }
          line_pos_ = 0;
        }
        line_buf_[line_pos_++] = static_cast<char>(byte);
        continue;
      }
      if (byte < 0x20 || byte > 0x7e) {
        if (line_pos_ > 0 && line_buf_[0] == '$') {
          line_callback(std::string(line_buf_, line_pos_));
          ++lines_processed;
        }
        line_pos_ = 0;
        continue;
      }
      if (line_pos_ < kLineBufSize - 1) {
        line_buf_[line_pos_++] = static_cast<char>(byte);
      } else {
        line_pos_ = 0;
      }
    }
  }
  return lines_processed;
}

// 将标准波特率映射为 termios 常量。
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

}   // namespace gnss_localization
