#include "gnss_localization/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace gnss_localization {

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
  close();
}

bool SerialPort::open(const std::string& device, int baud_rate) {
  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  speed_t spd = baud_to_speed(baud_rate);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  // 8N1: 8 数据位、无校验、1 停止位。
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag |= CS8;

  // close 时禁止拉低 DTR (避免接收机复位/重新 warm up)
  tty.c_cflag &= ~HUPCL;

  // 原始输入 (不做任何流控/转义)
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // 原始输出 (不做后处理)
  tty.c_oflag &= ~OPOST;

  // 关闭规范模式、回显、信号
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // 非阻塞: 有数据立即返回
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIOFLUSH);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  line_pos_ = 0;
  return true;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  line_pos_ = 0;
}

bool SerialPort::is_open() const {
  return fd_ >= 0;
}

bool SerialPort::write_raw(const char* data, size_t size) {
  if (fd_ < 0)
    return false;

  size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd_, data + written, size - written);
    if (n > 0) {
      written += static_cast<size_t>(n);
    } else if (n == 0) {
      return false;
    } else {
      if (errno == EINTR)
        continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 输出缓冲区满, 等待 POLLOUT 后重试
        struct pollfd pfd{};
        pfd.fd        = fd_;
        pfd.events    = POLLOUT;
        const int ret = ::poll(&pfd, 1, 1000);
        if (ret <= 0)
          return false;
        if (pfd.revents & (POLLERR | POLLHUP)) {
          close();   // 物理断开时关闭 fd_，让 is_open() 返回 false
          return false;
        }
        continue;
      }
      return false;
    }
  }
  return true;
}

int SerialPort::read_lines(const std::function<void(const std::string&)>& line_callback) {
  if (fd_ < 0)
    return 0;

  int lines_processed = 0;

  while (true) {
    // pollfd 每次循环重建, 规避 revents 残留的未定义行为。
    struct pollfd pfd{};
    pfd.fd     = fd_;
    pfd.events = POLLIN;

    const int poll_ret = ::poll(&pfd, 1, 0);
    if (poll_ret < 0) {
      if (errno == EINTR)
        continue;   // 信号打断, 重试 poll
      break;        // 真正的 poll 错误
    }
    if (poll_ret == 0)
      break;   // 超时 (0s 超时, 即本次无数据)

    // 检测串口断开/挂起/异常
    if (pfd.revents & (POLLHUP | POLLERR)) {
      close();   // 物理断开时关闭 fd_，让 is_open() 返回 false
      break;
    }
    if (!(pfd.revents & POLLIN))
      break;

    // poll 告警有数据, 但非阻塞 read 仍可能返回 EAGAIN (数据被竞态消费)。
    ssize_t n = ::read(fd_, read_buf_, kReadBufSize);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        break;   // 暂无可读数据, 下次 poll 再试
      break;     // 真正的 IO 错误
    }
    if (n == 0)
      break;   // 对端关闭 (串口断开)

    for (ssize_t i = 0; i < n; ++i) {
      unsigned char c = static_cast<unsigned char>(read_buf_[i]);

      if (c == '\n' || c == '\r') {
        size_t len = line_pos_;
        if (len > 0 && line_buf_[len - 1] == '\r')
          --len;
        if (len > 0) {
          line_callback(std::string(line_buf_, len));
          ++lines_processed;
        }
        line_pos_ = 0;
      } else if (c == '$') {
        // 遇到 '$' 时如果已有缓存, flush 前一句 (但跳过孤立的 '$', 预防连续 $$ 产生无效回调)。
        if (line_pos_ > 0) {
          if (line_buf_[0] != '$' || line_pos_ > 1) {
            line_callback(std::string(line_buf_, line_pos_));
            ++lines_processed;
          }
          line_pos_ = 0;
        }
        line_buf_[line_pos_++] = static_cast<char>(c);
      } else if (c < 0x20 || c > 0x7e) {
        // 二进制字节 (如 UBX); 若有未完成的 NMEA 行则先结束它
        if (line_pos_ > 0 && line_buf_[0] == '$') {
          line_callback(std::string(line_buf_, line_pos_));
          ++lines_processed;
        }
        line_pos_ = 0;
      } else if (line_pos_ < kLineBufSize - 1) {
        line_buf_[line_pos_++] = static_cast<char>(c);
      } else {
        line_pos_ = 0;
      }
    }
  }

  return lines_processed;
}

speed_t SerialPort::baud_to_speed(int baud) {
  switch (baud) {
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
      SPDLOG_WARN(
        "SerialPort: unsupported baud rate {} (supported: 9600/19200/38400/57600/115200/230400/460800/921600), "
        "falling back to 115200",
        baud);
      return B115200;
  }
}

}   // namespace gnss_localization