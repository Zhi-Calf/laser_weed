#include "gnss_localization/giavp_parser.hpp"

#include <charconv>
#include <cmath>
#include <cstdlib>
#include <system_error>

namespace gnss_localization {

namespace {
constexpr size_t kGiavpFieldCount = 24;  // 包含语句头在内的字段数
}  // 匿名命名空间

// 校验 NMEA 校验和，并要求语句在校验值后立即结束。
bool GiavpParser::verifyChecksum(const std::string& sentence) {
  if (sentence.empty() || sentence[0] != '$')
    return false;

  const size_t star = sentence.find('*');
  if (star == std::string::npos || star + 3 != sentence.size())
    return false;

  uint8_t computed = 0;
  for (size_t i = 1; i < star; ++i) {
    computed ^= static_cast<uint8_t>(sentence[i]);
  }

  const char hex_str[3] = {sentence[star + 1], sentence[star + 2], '\0'};
  char* end = nullptr;
  const unsigned long expected = std::strtoul(hex_str, &end, 16);
  if (end != hex_str + 2)
    return false;
  return computed == static_cast<uint8_t>(expected);
}

// 按逗号切分 GIAVP 语句体。
std::vector<std::string> GiavpParser::splitFields(const std::string& body) {
  std::vector<std::string> fields;
  fields.reserve(kGiavpFieldCount);

  size_t start = 0;
  while (start <= body.size()) {
    const size_t pos = body.find(',', start);
    if (pos == std::string::npos) {
      fields.push_back(body.substr(start));
      break;
    }
    fields.push_back(body.substr(start, pos - start));
    start = pos + 1;
  }
  return fields;
}

// 严格解析有限浮点数，拒绝空字段、尾随垃圾和溢出值。
bool GiavpParser::parseDouble(const std::string& text, double* value) {
  if (text.empty() || value == nullptr)
    return false;
  const char* begin = text.data();
  const char* end = begin + text.size();
  double parsed = 0.0;
  const std::from_chars_result result =
      std::from_chars(begin, end, parsed);
  if (result.ec != std::errc() || result.ptr != end ||
      !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

// 严格解析整数，拒绝空字段和尾随垃圾。
bool GiavpParser::parseInt(const std::string& text, int* value) {
  if (text.empty() || value == nullptr)
    return false;
  const char* begin = text.data();
  const char* end = begin + text.size();
  int parsed = 0;
  const std::from_chars_result result =
      std::from_chars(begin, end, parsed);
  if (result.ec != std::errc() || result.ptr != end)
    return false;
  *value = parsed;
  return true;
}

// 解析完整 GIAVP 语句，任一字段损坏时整句判无效。
bool GiavpParser::parse(const std::string& line) {
  if (line.size() < 7 || line[0] != '$')
    return false;

  const size_t star = line.find('*');
  if (star == std::string::npos || !verifyChecksum(line))
    return false;
  const std::string body = line.substr(1, star - 1);

  const std::vector<std::string> fields = splitFields(body);
  if (fields.empty() || fields[0] != "GIAVP" ||
      fields.size() != kGiavpFieldCount) {
    return false;
  }

  GiavpData data;
  const bool parsed =
      parseInt(fields[1], &data.gps_week) &&
      parseDouble(fields[2], &data.gps_time_sec) &&
      parseDouble(fields[3], &data.heading_deg) &&
      parseDouble(fields[4], &data.pitch_deg) &&
      parseDouble(fields[5], &data.roll_deg) &&
      parseDouble(fields[6], &data.latitude_deg) &&
      parseDouble(fields[7], &data.longitude_deg) &&
      parseDouble(fields[8], &data.altitude_m) &&
      parseDouble(fields[9], &data.ve_ms) &&
      parseDouble(fields[10], &data.vn_ms) &&
      parseDouble(fields[11], &data.vu_ms) &&
      parseDouble(fields[12], &data.baseline_m) &&
      parseInt(fields[13], &data.nsv1) &&
      parseInt(fields[14], &data.nsv2) &&
      parseInt(fields[15], &data.status) &&
      parseInt(fields[16], &data.speed_status) &&
      parseDouble(fields[17], &data.vehicle_speed_kmh) &&
      parseDouble(fields[18], &data.acc_x) &&
      parseDouble(fields[19], &data.acc_y) &&
      parseDouble(fields[20], &data.acc_z) &&
      parseDouble(fields[21], &data.gyro_x_dps) &&
      parseDouble(fields[22], &data.gyro_y_dps) &&
      parseDouble(fields[23], &data.gyro_z_dps);
  if (!parsed)
    return false;

  if (std::abs(data.latitude_deg) > 90.0 ||
      std::abs(data.longitude_deg) > 180.0) {
    return false;
  }
  if (data.status != 0 && data.status != 1 && data.status != 2 &&
      data.status != 4 && data.status != 5 && data.status != 6) {
    return false;
  }

  data.valid = true;
  data_ = data;
  return true;
}

}  // gnss_localization 命名空间
