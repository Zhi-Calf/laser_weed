#ifndef GNSS_LOCALIZATION_SERIAL_PORT_H_
#define GNSS_LOCALIZATION_SERIAL_PORT_H_

#if !defined(__linux__)
#error "gnss_localization SerialPort 仅支持 Linux"
#endif

#include <termios.h>   // speed_t

#include <chrono>
#include <cstddef>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "gnss_localization/nmea_line_assembler.hpp"

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

  bool write_raw(const char* data, std::size_t size, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  int read_lines(const std::function<void(const std::string&)>& line_callback);
  std::string last_error() const;

private:
  static constexpr std::size_t kReadBufSize = 4096;
  static std::optional<speed_t> BaudToSpeed(int baud_rate);
  void CloseUnlocked();
  void SetErrorUnlocked(const std::string& message);

  mutable std::mutex mutex_;
  int fd_ = -1;
  char read_buf_[kReadBufSize];
  NmeaLineAssembler line_assembler_;
  std::string last_error_;
};

}   // namespace gnss_localization

#endif   // GNSS_LOCALIZATION_SERIAL_PORT_H_
