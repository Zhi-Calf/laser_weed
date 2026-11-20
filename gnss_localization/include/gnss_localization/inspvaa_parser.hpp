#pragma once

#include <cstdint>
#include <string>

namespace gnss_localization {

// INSPVAA 融合导航数据，位置为 WGS84 椭球高，速度为东北天方向。
struct InspvaaData {
  uint32_t gps_week{0};          // GPS 周
  double gps_time_sec{0.0};      // GPS 周内秒
  double latitude_deg{0.0};      // WGS84 纬度
  double longitude_deg{0.0};     // WGS84 经度
  double height_m{0.0};          // WGS84 椭球高 (m)
  double vn_ms{0.0};             // 北向速度 (m/s)
  double ve_ms{0.0};             // 东向速度 (m/s)
  double vu_ms{0.0};             // 天向速度 (m/s)
  double roll_deg{0.0};          // 横滚角 (deg)
  double pitch_deg{0.0};         // 俯仰角 (deg)
  double heading_deg{0.0};       // 真北顺时针航向 (deg)
  int ins_status{0};             // INSPVAA 惯导解算状态
  bool valid{false};
};

// 严格校验 CRC32、字段和数值范围的 INSPVAA 文本解析器。
class InspvaaParser {
public:
  InspvaaParser() = default;

  bool parse(const std::string& line);

  const InspvaaData& data() const {
    return data_;
  }

private:
  InspvaaData data_;
};

}   // namespace gnss_localization
