#pragma once

#include <string>

namespace gnss_localization {

// 校验 NMEA GGA 帧的格式、校验和、定位状态和必需字段。
bool IsValidGga(const std::string& sentence);

}   // namespace gnss_localization
