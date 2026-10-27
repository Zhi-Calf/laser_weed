#include "gnss_localization/gga_parser.hpp"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <string>
#include <system_error>
#include <vector>

namespace gnss_localization {

namespace {

constexpr std::size_t kMinimumGgaFields = 13;
constexpr std::size_t kMaximumGgaFields = 15;

// 判断字符是否为 ASCII 十进制数字。
// 输入:
//   character : 待判断字符
// 返回:
//   true = '0'..'9'
bool IsAsciiDigit(char character) {
  return character >= '0' && character <= '9';
}

// 判断字符是否为 ASCII 大写字母。
// 输入:
//   character : 待判断字符
// 返回:
//   true = 'A'..'Z'
bool IsUpperTalkerCharacter(char character) {
  return character >= 'A' && character <= 'Z';
}

// 将单个十六进制字符转换为数值。
// 输入:
//   character : 十六进制字符
// 返回:
//   0..15；非法字符返回 -1。
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

// 按逗号拆分 GGA 报文字段。
// 输入:
//   body : 不含起始 '$' 和校验和的报文体
// 返回:
//   按出现顺序保存的字段，空字段保留为空字符串。
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

// 严格解析正整数文本。
// 输入:
//   text : 待解析文本
// 输出:
//   value : 解析结果；失败时不修改
// 返回:
//   true = 全文均为合法整数
bool ParseInt(const std::string& text, int* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  const char* begin                   = text.data();
  const char* end                     = begin + text.size();
  int parsed                          = 0;
  const std::from_chars_result result = std::from_chars(begin, end, parsed);
  if (result.ec != std::errc() || result.ptr != end) {
    return false;
  }
  *value = parsed;
  return true;
}

// 严格解析有限浮点数文本。
// 输入:
//   text : 待解析文本
// 输出:
//   value : 解析结果；失败时不修改
// 返回:
//   true = 全文合法且结果有限
bool ParseDouble(const std::string& text, double* value) {
  if (text.empty() || value == nullptr) {
    return false;
  }
  const char* begin                   = text.data();
  const char* end                     = begin + text.size();
  double parsed                       = 0.0;
  const std::from_chars_result result = std::from_chars(begin, end, parsed);
  if (result.ec != std::errc() || result.ptr != end || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

// 校验 GGA 的 UTC 时间字段。
// 输入:
//   text : hhmmss 或 hhmmss.fff
// 返回:
//   true = 格式合法且时分秒在协议范围内
bool IsValidUtcTime(const std::string& text) {
  const std::size_t dot = text.find('.');
  if (dot != std::string::npos && (dot + 1 == text.size() || text.find('.', dot + 1) != std::string::npos)) {
    return false;
  }
  const std::string whole = dot == std::string::npos ? text : text.substr(0, dot);
  if (whole.size() != 6) {
    return false;
  }
  for (char character : whole) {
    if (!IsAsciiDigit(character)) {
      return false;
    }
  }
  if (dot != std::string::npos) {
    for (std::size_t i = dot + 1; i < text.size(); ++i) {
      if (!IsAsciiDigit(text[i])) {
        return false;
      }
    }
  }

  const int hour   = (whole[0] - '0') * 10 + (whole[1] - '0');
  const int minute = (whole[2] - '0') * 10 + (whole[3] - '0');
  const int second = (whole[4] - '0') * 10 + (whole[5] - '0');
  return hour <= 23 && minute <= 59 && second <= 60;
}

// 校验 GGA 的度分格式坐标。
// 输入:
//   text           : 经纬度字段
//   degree_digits  : 度数位数
//   maximum_degrees: 允许的最大度数
// 返回:
//   true = 格式合法且不超过纬度/经度上限
bool IsValidCoordinate(const std::string& text, std::size_t degree_digits, double maximum_degrees) {
  const std::size_t dot = text.find('.');
  if (dot == std::string::npos || dot + 1 == text.size() || text.find('.', dot + 1) != std::string::npos || dot != degree_digits + 2) {
    return false;
  }
  for (char character : text) {
    if (character != '.' && !IsAsciiDigit(character)) {
      return false;
    }
  }

  const std::string degrees_text = text.substr(0, degree_digits);
  int degrees                    = 0;
  if (!ParseInt(degrees_text, &degrees)) {
    return false;
  }
  const std::string minutes_text = text.substr(degree_digits, 2) + text.substr(dot);
  double minutes                 = 0.0;
  if (!ParseDouble(minutes_text, &minutes) || minutes >= 60.0) {
    return false;
  }
  if (degrees > static_cast<int>(maximum_degrees) || (degrees == static_cast<int>(maximum_degrees) && minutes > 0.0)) {
    return false;
  }
  return true;
}

// 校验 GGA 可选的基站 ID。
// 输入:
//   text : 0~4 位字母或数字
// 返回:
//   true = 字段为空或格式合法
bool IsValidStationId(const std::string& text) {
  if (text.empty()) {
    return true;
  }
  if (text.size() > 4) {
    return false;
  }
  for (char character : text) {
    const bool valid = IsAsciiDigit(character) || (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z');
    if (!valid) {
      return false;
    }
  }
  return true;
}

}   // namespace

// 校验一帧 GGA 的格式、校验和、位置和差分可用性。
// 输入:
//   sentence : 完整 GGA 语句，不含 CR/LF
// 返回:
//   true = 格式、校验和及字段均有效
// 约束:
//   只做格式校验，不判断 RTK 固定解。
bool IsValidGga(const std::string& sentence) {
  if (sentence.size() < 8 || sentence.front() != '$') {
    return false;
  }
  const std::size_t star = sentence.find('*');
  if (star == std::string::npos || star + 3 != sentence.size()) {
    return false;
  }

  const int high = HexValue(sentence[star + 1]);
  const int low  = HexValue(sentence[star + 2]);
  if (high < 0 || low < 0) {
    return false;
  }
  unsigned char checksum = 0;
  for (std::size_t i = 1; i < star; ++i) {
    checksum ^= static_cast<unsigned char>(sentence[i]);
  }
  if (checksum != static_cast<unsigned char>((high << 4) | low)) {
    return false;
  }

  const std::vector<std::string> fields = SplitFields(sentence.substr(1, star - 1));
  if (fields.size() < kMinimumGgaFields || fields.size() > kMaximumGgaFields) {
    return false;
  }
  const std::string& type = fields[0];
  if (type.size() != 5 || type.compare(2, 3, "GGA") != 0 || !IsUpperTalkerCharacter(type[0]) || !IsUpperTalkerCharacter(type[1])) {
    return false;
  }
  if (!IsValidUtcTime(fields[1]) || !IsValidCoordinate(fields[2], 2, 90.0) || (fields[3] != "N" && fields[3] != "S") || !IsValidCoordinate(fields[4], 3, 180.0) || (fields[5] != "E" && fields[5] != "W")) {
    return false;
  }

  int fix_quality     = 0;
  int satellite_count = 0;
  if (!ParseInt(fields[6], &fix_quality) || fix_quality < 1 || fix_quality > 8 || !ParseInt(fields[7], &satellite_count) || satellite_count < 0) {
    return false;
  }

  double numeric = 0.0;
  if (
    !ParseDouble(fields[8], &numeric) || numeric < 0.0 || !ParseDouble(fields[9], &numeric) || (fields[10] != "M" && fields[10] != "m") || !ParseDouble(fields[11], &numeric) ||
    (fields[12] != "M" && fields[12] != "m")) {
    return false;
  }

  if (fields.size() > 13 && !fields[13].empty()) {
    if (!ParseDouble(fields[13], &numeric) || numeric < 0.0) {
      return false;
    }
  }
  if (fields.size() > 14 && !IsValidStationId(fields[14])) {
    return false;
  }
  return true;
}

}   // namespace gnss_localization
