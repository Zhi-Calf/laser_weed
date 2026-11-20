#include "gnss_localization/inspvaa_parser.hpp"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <system_error>
#include <vector>

namespace gnss_localization {

namespace {

constexpr std::size_t kInspvaaPayloadFields = 12;
constexpr uint32_t kCrc32Polynomial          = 0xEDB88320U;
constexpr double kSecondsPerWeek             = 604800.0;

// 按逗号拆分报文，保留空字段。
std::vector<std::string> SplitFields(const std::string& body) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= body.size()) {
    const std::size_t comma = body.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(body.substr(start));
      break;
    }
    fields.push_back(body.substr(start, comma - start));
    start = comma + 1;
  }
  return fields;
}

// 解析有限浮点数。
bool ParseDouble(const std::string& text, double* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  double parsed = 0.0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (result.ec != std::errc() || result.ptr != text.data() + text.size() || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

// 解析无符号整数。
bool ParseUint32(const std::string& text, uint32_t* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  uint32_t parsed = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (result.ec != std::errc() || result.ptr != text.data() + text.size()) {
    return false;
  }
  *value = parsed;
  return true;
}

// 将单个十六进制字符转换为数值。
int HexValue(char character) {
  if (character >= '0' && character <= '9') {
    return character - '0';
  }
  if (character >= 'A' && character <= 'F') {
    return character - 'A' + 10;
  }
  if (character >= 'a' && character <= 'f') {
    return character - 'a' + 10;
  }
  return -1;
}

// 校验 INSPVAA 使用的 CRC32: 初值为 0，不执行最终异或。
uint32_t InspvaaCrc32(std::string_view body) {
  uint32_t crc = 0;
  for (const unsigned char byte : body) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kCrc32Polynomial : crc >> 1U;
    }
  }
  return crc;
}

// 解析报文末尾的 8 位十六进制 CRC32。
bool ParseCrc32(const std::string& text, uint32_t* value) {
  if (text.size() != 8 || value == nullptr) {
    return false;
  }
  uint32_t parsed = 0;
  for (const char character : text) {
    const int digit = HexValue(character);
    if (digit < 0) {
      return false;
    }
    parsed = (parsed << 4U) | static_cast<uint32_t>(digit);
  }
  *value = parsed;
  return true;
}

// 将 INSPVAA 状态文本映射为协议枚举值。
bool ParseStatus(const std::string& text, int* status) {
  if (text == "INS_INACTIVE") {
    *status = 0;
  } else if (text == "INS_ALIGNING") {
    *status = 1;
  } else if (text == "INS_HIGH_VARIANCE") {
    *status = 2;
  } else if (text == "INS_SOLUTION_GOOD") {
    *status = 3;
  } else if (text == "INS_SOLUTION_FREE") {
    *status = 6;
  } else if (text == "INS_ALIGNMENT_COMPLETE") {
    *status = 7;
  } else if (text == "DETERMINING_ORIENTATION") {
    *status = 8;
  } else if (text == "WAITING_INITIALPOS") {
    *status = 9;
  } else if (text == "WAITING_AZIMUTH") {
    *status = 10;
  } else if (text == "INITIALIZING_BIASES") {
    *status = 11;
  } else if (text == "MOTION_DETECT") {
    *status = 12;
  } else if (text == "WAITING_ALIGNMENTORIENTATION") {
    *status = 14;
  } else {
    return false;
  }
  return true;
}

}   // namespace

// 解析一帧带 CRC32 的 INSPVAA 融合导航数据。
// 输入:
//   line : 不含 CR/LF 的完整 INSPVAA 文本
// 返回:
//   true = CRC、字段和数值范围均合法
bool InspvaaParser::parse(const std::string& line) {
  if (line.rfind("#INSPVAA,", 0) != 0) {
    return false;
  }
  const std::size_t star = line.rfind('*');
  if (star == std::string::npos || star + 9 != line.size()) {
    return false;
  }
  const std::string body = line.substr(1, star - 1);
  uint32_t expected_crc  = 0;
  if (!ParseCrc32(line.substr(star + 1), &expected_crc) || InspvaaCrc32(body) != expected_crc) {
    return false;
  }

  const std::size_t semicolon = body.find(';');
  if (semicolon == std::string::npos || body.find(';', semicolon + 1) != std::string::npos) {
    return false;
  }
  const std::vector<std::string> fields = SplitFields(body.substr(semicolon + 1));
  if (fields.size() != kInspvaaPayloadFields) {
    return false;
  }

  InspvaaData data;
  const bool numeric_fields =
    ParseUint32(fields[0], &data.gps_week) && ParseDouble(fields[1], &data.gps_time_sec) && ParseDouble(fields[2], &data.latitude_deg) &&
    ParseDouble(fields[3], &data.longitude_deg) && ParseDouble(fields[4], &data.height_m) && ParseDouble(fields[5], &data.vn_ms) &&
    ParseDouble(fields[6], &data.ve_ms) && ParseDouble(fields[7], &data.vu_ms) && ParseDouble(fields[8], &data.roll_deg) &&
    ParseDouble(fields[9], &data.pitch_deg) && ParseDouble(fields[10], &data.heading_deg) && ParseStatus(fields[11], &data.ins_status);
  if (!numeric_fields) {
    return false;
  }

  const bool ranges_valid =
    data.gps_week > 0 && data.gps_time_sec >= 0.0 && data.gps_time_sec < kSecondsPerWeek && std::abs(data.latitude_deg) <= 90.0 &&
    std::abs(data.longitude_deg) <= 180.0 && data.roll_deg >= -180.0 && data.roll_deg <= 180.0 && data.pitch_deg >= -90.0 &&
    data.pitch_deg <= 90.0 && data.heading_deg >= 0.0 && data.heading_deg < 360.0;
  if (!ranges_valid) {
    return false;
  }

  data.valid = true;
  data_      = data;
  return true;
}

}   // namespace gnss_localization
