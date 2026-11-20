#pragma once

#include <string>

namespace gnss_localization {

// GGA 中供融合驱动使用的定位质量字段。
struct GgaFixStatus {
  int fix_quality{0};             // 0=无解, 2=RTD, 4=RTK Fixed, 5=RTK Float
  int satellites{0};              // 当前参与解算的卫星数
  double hdop{0.0};               // 水平精度因子
  double differential_age_sec{-1.0};   // 差分龄期 (s), 空字段为 -1
  bool valid{false};
};

// 校验 GGA 并提取定位质量、卫星数和差分龄期。
class GgaStatusParser {
public:
  GgaStatusParser() = default;

  bool parse(const std::string& line);

  const GgaFixStatus& data() const {
    return data_;
  }

private:
  GgaFixStatus data_;
};

}   // namespace gnss_localization
