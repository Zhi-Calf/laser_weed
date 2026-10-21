#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace gnss_localization {

/// 双天线 MEMS 组合导航系统解析后的 $GIAVP 语句。
///
/// 报文格式:
/// $GIAVP,GPSWeek,GPSTime,Heading,Pitch,Roll,Lat,Lon,Alt,Ve,Vn,Vu,
///        Baseline,Nsv1,Nsv2,Status,SpeedStatus,VehicleSpeed,
///        AccX,AccY,AccZ,GyroX,GyroY,GyroZ*cs
struct GiavpData {
  int gps_week{0};
  double gps_time_sec{0.0};        // 周内秒
  double heading_deg{0.0};         // 0-359.99, 真北起算, 顺时针
  double pitch_deg{0.0};           // -90~90
  double roll_deg{0.0};            // -180~180
  double latitude_deg{0.0};        // 十进制度, 带符号
  double longitude_deg{0.0};       // 十进制度, 带符号
  double altitude_m{0.0};          // 海拔高度 (m)
  double ve_ms{0.0};               // 东向速度 (m/s)
  double vn_ms{0.0};               // 北向速度 (m/s)
  double vu_ms{0.0};               // 天向速度 (m/s)
  double baseline_m{0.0};          // 双天线基线长度 (m)
  int nsv1{0};                     // 天线 1 卫星数
  int nsv2{0};                     // 天线 2 卫星数
  int status{0};                   // 0=初始化/无解, 1=SPP, 2=RTD, 4=RTK 固定解, 5=RTK 浮点解
  int speed_status{0};             // 0=无车速, 1=有车速
  double vehicle_speed_kmh{0.0};   // 车速 (km/h)
  double acc_x{0.0};               // m/s^2
  double acc_y{0.0};               // m/s^2
  double acc_z{0.0};               // m/s^2
  double gyro_x_dps{0.0};          // deg/s
  double gyro_y_dps{0.0};          // deg/s
  double gyro_z_dps{0.0};          // deg/s
  bool valid{false};
};

// 解析 $GIAVP 语句
class GiavpParser {
public:
  GiavpParser() = default;

  bool parse(const std::string& line);

  const GiavpData& data() const {
    return data_;
  }

private:
  static bool verifyChecksum(const std::string& sentence);
  static std::vector<std::string> splitFields(const std::string& body);
  static bool parseDouble(const std::string& text, double* value);
  static bool parseInt(const std::string& text, int* value);

private:
  GiavpData data_;
};

}   // namespace gnss_localization
