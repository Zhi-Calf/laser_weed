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
//
// 生命周期: 析构时自动关闭文件描述符。读写接口不会跨对象共享内部缓冲;
// 同一实例的并发访问由调用方串行化。
class SerialPort {
public:
  // 构造未打开的串口对象。
  SerialPort();
  // 关闭串口并释放文件描述符。
  ~SerialPort();

  SerialPort(const SerialPort&)            = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // 打开串口并配置为 8N1 原始模式; 失败时保持关闭状态。
  bool open(const std::string& device, int baud_rate);

  // 关闭串口并清空当前行缓冲; 重复调用安全。
  void close();

  // 返回文件描述符是否有效。
  bool is_open() const;

  // 写入完整原始字节流，失败时关闭串口并返回 false。
  bool write_raw(const char* data, std::size_t size);

  // 非阻塞读取完整文本行，并返回本次处理的行数。
  int read_lines(const std::function<void(const std::string&)>& line_callback);

private:
  static std::size_t constexpr kReadBufSize = 4096;
  static std::size_t constexpr kLineBufSize = 1024;
  // 将整数波特率映射为 termios 常量。
  static std::optional<speed_t> BaudToSpeed(int baud_rate);

  int fd_ = -1;                   // 串口文件描述符
  char read_buf_[kReadBufSize];   // 单次读取缓冲
  char line_buf_[kLineBufSize];   // 当前文本行缓冲
  std::size_t line_pos_ = 0;      // 当前行长度
};

}   // namespace gnss_localization
