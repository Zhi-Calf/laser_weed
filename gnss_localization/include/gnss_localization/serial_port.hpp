#pragma once

#if !defined(__linux__)
#error "gnss_localization SerialPort 仅支持 Linux"
#endif

#include <termios.h>

#include <cstddef>
#include <functional>
#include <optional>
#include <string>

namespace gnss_localization {

// Linux 串口打开、原始字节读写和 NMEA 行提取。
class SerialPort {
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort&)            = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  bool open(const std::string& device, int baud_rate);
  void close();
  bool is_open() const;
  bool write_raw(const char* data, std::size_t size);
  int read_lines(const std::function<void(const std::string&)>& line_callback);

private:
  static std::size_t constexpr kReadBufSize = 4096;
  static std::size_t constexpr kLineBufSize = 1024;
  static std::optional<speed_t> BaudToSpeed(int baud_rate);

  int fd_ = -1;                   // 串口文件描述符
  char read_buf_[kReadBufSize];   // 单次读取缓冲
  char line_buf_[kLineBufSize];   // 当前文本行缓冲
  std::size_t line_pos_ = 0;      // 当前行长度
};

}   // namespace gnss_localization
