#include "gnss_localization/giavp_parser.hpp"

#include <cmath>
#include <cstdlib>

namespace gnss_localization {

namespace {
constexpr size_t kGiavpFieldCount = 24;   // "GIAVP" + 23 个数据字段
}

// NMEA校验和
bool GiavpParser::verifyChecksum(const std::string& sentence) {
  if (sentence.empty() || sentence[0] != '$')
    return false;

  auto star = sentence.find('*');
  if (star == std::string::npos || star + 3 > sentence.size())
    return false;

  uint8_t computed = 0;
  for (size_t i = 1; i < star; ++i) {
    computed ^= static_cast<uint8_t>(sentence[i]);
  }

  char hex_str[3] = {sentence[star + 1], sentence[star + 2], '\0'};
  auto expected   = static_cast<uint8_t>(std::strtoul(hex_str, nullptr, 16));

  return computed == expected;
}

// 字段分割
std::vector<std::string> GiavpParser::splitFields(const std::string& body) {
  std::vector<std::string> fields;
  fields.reserve(kGiavpFieldCount);

  size_t start = 0;
  while (start <= body.size()) {
    auto pos = body.find(',', start);
    if (pos == std::string::npos) {
      fields.push_back(body.substr(start));
      break;
    }
    fields.push_back(body.substr(start, pos - start));
    start = pos + 1;
  }
  return fields;
}

// 安全转换
double GiavpParser::safeStod(const std::string& s, double fallback) {
  if (s.empty())
    return fallback;
  try {
    return std::stod(s);
  } catch (const std::exception&) {
    return fallback;
  }
}

// 安全转换
int GiavpParser::safeStoi(const std::string& s, int fallback) {
  if (s.empty())
    return fallback;
  try {
    return std::stoi(s);
  } catch (const std::exception&) {
    return fallback;
  }
}

// 解析
bool GiavpParser::parse(const std::string& line) {
  if (line.size() < 7 || line[0] != '$')
    return false;

  auto star = line.find('*');
  std::string body;
  if (star != std::string::npos && star + 3 <= line.size()) {
    if (!verifyChecksum(line))
      return false;
    body = line.substr(1, star - 1);
  } else {
    // 无校验和终止符 —— 接受 '$' 之后整行
    body = line.substr(1);
  }

  auto fields = splitFields(body);
  if (fields.empty() || fields[0] != "GIAVP")
    return false;
  if (fields.size() < kGiavpFieldCount)
    return false;

  GiavpData d;
  d.gps_week          = safeStoi(fields[1]);
  d.gps_time_sec      = safeStod(fields[2]);
  d.heading_deg       = safeStod(fields[3]);
  d.pitch_deg         = safeStod(fields[4]);
  d.roll_deg          = safeStod(fields[5]);
  d.latitude_deg      = safeStod(fields[6]);
  d.longitude_deg     = safeStod(fields[7]);
  d.altitude_m        = safeStod(fields[8]);
  d.ve_ms             = safeStod(fields[9]);
  d.vn_ms             = safeStod(fields[10]);
  d.vu_ms             = safeStod(fields[11]);
  d.baseline_m        = safeStod(fields[12]);
  d.nsv1              = safeStoi(fields[13]);
  d.nsv2              = safeStoi(fields[14]);
  d.status            = safeStoi(fields[15]);
  d.speed_status      = safeStoi(fields[16]);
  d.vehicle_speed_kmh = safeStod(fields[17]);
  d.acc_x             = safeStod(fields[18]);
  d.acc_y             = safeStod(fields[19]);
  d.acc_z             = safeStod(fields[20]);
  d.gyro_x_dps        = safeStod(fields[21]);
  d.gyro_y_dps        = safeStod(fields[22]);
  d.gyro_z_dps        = safeStod(fields[23]);

  // 合理性检查 ①: **整句的数值字段必须全部有限**。GIAVP 的每个数值字段都会被发到下游消息
  // (fix / odom / imu / vel / status), 任一 nan/inf 都会污染那条 ROS 消息 (例如 odom 的角速度、
  // imu 的加速度), 而下游从数值本身看不出"这帧坏了"。所以判据放在整句一级: 有一个非有限值就整句
  // 判无效 —— 与"经纬度越界"一样由驱动的数据超时兜底 (last_giavp_time_ 不刷新 -> 判故障),
  // 而不是发一条带 NaN 的消息出去。
  // ⚠ 空字段/解析不了的值在 safeStod 里已经变成 0.0 (有限), 不会因此丢帧; 只有报文里真出现
  //   "nan"/"inf" 文本 (或数字溢出成 inf) 才算无效 —— 不改真实数据的处置。
  const double numeric_fields[] = {d.gps_time_sec, d.heading_deg, d.pitch_deg,   d.roll_deg,     d.latitude_deg, d.longitude_deg,
                                   d.altitude_m,   d.ve_ms,       d.vn_ms,       d.vu_ms,        d.baseline_m,   d.vehicle_speed_kmh,
                                   d.acc_x,        d.acc_y,       d.acc_z,       d.gyro_x_dps,   d.gyro_y_dps,   d.gyro_z_dps};
  for (const double v : numeric_fields) {
    if (!std::isfinite(v)) {
      return false;
    }
  }

  // 合理性检查 ②: 经纬度范围 (度, WGS84)
  if (std::abs(d.latitude_deg) > 90.0 || std::abs(d.longitude_deg) > 180.0) {
    return false;
  }

  d.valid = true;
  data_   = d;
  return true;
}

}   // namespace gnss_localization