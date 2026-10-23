#ifndef GNSS_LOCALIZATION_NMEA_LINE_ASSEMBLER_H_
#define GNSS_LOCALIZATION_NMEA_LINE_ASSEMBLER_H_

#include <array>
#include <cstddef>
#include <functional>
#include <string>

namespace gnss_localization {

// 从串口字节流中提取以 '$' 开头的完整文本行。
class NmeaLineAssembler {
public:
  void Reset();
  void Push(const char* data, std::size_t size, const std::function<void(const std::string&)>& line_callback);

private:
  static constexpr std::size_t kMaxLineLength = 1024;

  void PushByte(unsigned char byte, const std::function<void(const std::string&)>& line_callback);

  std::array<char, kMaxLineLength> line_buffer_{};
  std::size_t line_length_{0};
  bool collecting_{false};
  bool discarding_{false};
};

}   // namespace gnss_localization

#endif   // GNSS_LOCALIZATION_NMEA_LINE_ASSEMBLER_H_
