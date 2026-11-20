#include "gnss_localization/gga_status_parser.hpp"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <system_error>
#include <vector>

namespace gnss_localization {

namespace {

// 按逗号拆分报文。
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

// 校验 NMEA XOR 校验和。
bool VerifyChecksum(const std::string& line) {
  if (line.size() < 8 || line.front() != '$') {
    return false;
  }
  const std::size_t star = line.find('*');
  if (star == std::string::npos || star + 3 != line.size()) {
    return false;
  }
  uint8_t computed = 0;
  for (std::size_t index = 1; index < star; ++index) {
    computed ^= static_cast<uint8_t>(line[index]);
  }
  const char hex[3] = {line[star + 1], line[star + 2], '\0'};
  char* end         = nullptr;
  const unsigned long expected = std::strtoul(hex, &end, 16);
  return end == hex + 2 && computed == static_cast<uint8_t>(expected);
}

// 解析有限双精度数。
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

// 解析非负整数。
bool ParseNonNegativeInt(const std::string& text, int* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  int parsed = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (result.ec != std::errc() || result.ptr != text.data() + text.size() || parsed < 0) {
    return false;
  }
  *value = parsed;
  return true;
}

}   // namespace

// 解析 GGA，仅提取融合驱动关心的质量和卫星字段。
bool GgaStatusParser::parse(const std::string& line) {
  if (!VerifyChecksum(line)) {
    return false;
  }
  const std::size_t star = line.find('*');
  const std::vector<std::string> fields = SplitFields(line.substr(1, star - 1));
  if (fields.size() < 9 || fields[0].size() != 5 || fields[0].compare(2, 3, "GGA") != 0) {
    return false;
  }

  GgaFixStatus data;
  if (!ParseNonNegativeInt(fields[6], &data.fix_quality) || data.fix_quality > 8) {
    return false;
  }
  if (!fields[7].empty() && !ParseNonNegativeInt(fields[7], &data.satellites)) {
    return false;
  }
  if (!fields[8].empty() && !ParseDouble(fields[8], &data.hdop)) {
    return false;
  }
  if (fields.size() > 13 && !fields[13].empty()) {
    if (!ParseDouble(fields[13], &data.differential_age_sec) || data.differential_age_sec < 0.0) {
      return false;
    }
  }
  data.valid = true;
  data_      = data;
  return true;
}

}   // namespace gnss_localization
