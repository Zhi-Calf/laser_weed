#pragma once

#include <string>

namespace gnss_localization {

// 校验一帧 NMEA GGA 语句的格式、校验和、定位状态和必需字段。
// 输入:
//   sentence : 完整 GGA 语句 (含 '$' 与校验和; 不含 CR/LF)
// 返回:
//   true = 格式与字段均有效; false = 任意一项不满足
// 约束:
//   只做格式与取值范围校验, 不判断 RTK 固定解, 也不会修改外部状态。
bool IsValidGga(const std::string& sentence);

}   // namespace gnss_localization
