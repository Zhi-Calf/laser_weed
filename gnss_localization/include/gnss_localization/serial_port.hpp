#pragma once

#include <termios.h>   // speed_t

#include <cstdint>
#include <functional>
#include <string>

namespace gnss_localization {

// 定义串口
class SerialPort {
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort&)            = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  bool open(const std::string& device, int baud_rate);
  void close();
  bool is_open() const;

  // 写原始字节 (不加行终止符)。用于 RTCM/UBX 二进制载荷。
  bool write_raw(const char* data, size_t size);

  // 获取底层文件描述符 (-1 表示未打开)。
  int fd() const {
    return fd_;
  }

  // 非阻塞读: 每收到一个完整行就回调 line_callback。
  // 返回本次处理完成的完整行数。
  int read_lines(const std::function<void(const std::string&)>& line_callback);

private:
  static constexpr size_t kReadBufSize = 4096;
  static constexpr size_t kLineBufSize = 1024;

  int fd_{-1};
  char read_buf_[kReadBufSize];
  char line_buf_[kLineBufSize];
  size_t line_pos_{0};

  static speed_t baud_to_speed(int baud);
};

}   // namespace gnss_localization