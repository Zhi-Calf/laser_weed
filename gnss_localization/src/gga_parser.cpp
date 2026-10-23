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

bool IsAsciiDigit(char character) {
  return character >= '0' && character <= '9';
}

bool IsUpperTalkerCharacter(char character) {
  return character >= 'A' && character <= 'Z';
}

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

bool IsValidUtcTime(const std::string& text) {
  const std::size_t dot = text.find('.');
  if (dot != std::string::npos &&
      (dot + 1 == text.size() ||
       text.find('.', dot + 1) != std::string::npos)) {
    return false;
  }
  const std::string whole =
      dot == std::string::npos ? text : text.substr(0, dot);
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

bool IsValidCoordinate(
    const std::string& text, std::size_t degree_digits,
    double maximum_degrees) {
  const std::size_t dot = text.find('.');
  if (dot == std::string::npos || dot + 1 == text.size() ||
      text.find('.', dot + 1) != std::string::npos ||
      dot != degree_digits + 2) {
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
  const std::string minutes_text =
      text.substr(degree_digits, 2) + text.substr(dot);
  double minutes                 = 0.0;
  if (!ParseDouble(minutes_text, &minutes) || minutes >= 60.0) {
    return false;
  }
  if (degrees > static_cast<int>(maximum_degrees) ||
      (degrees == static_cast<int>(maximum_degrees) && minutes > 0.0)) {
    return false;
  }
  return true;
}

bool IsValidStationId(const std::string& text) {
  if (text.empty()) {
    return true;
  }
  if (text.size() > 4) {
    return false;
  }
  for (char character : text) {
    const bool valid =
        IsAsciiDigit(character) ||
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z');
    if (!valid) {
      return false;
    }
  }
  return true;
}

}   // namespace

// 校验一帧 GGA 的格式、校验和、位置和差分可用性。
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

  const std::vector<std::string> fields =
      SplitFields(sentence.substr(1, star - 1));
  if (fields.size() < kMinimumGgaFields || fields.size() > kMaximumGgaFields) {
    return false;
  }
  const std::string& type = fields[0];
  if (type.size() != 5 || type.compare(2, 3, "GGA") != 0 ||
      !IsUpperTalkerCharacter(type[0]) ||
      !IsUpperTalkerCharacter(type[1])) {
    return false;
  }
  if (!IsValidUtcTime(fields[1]) ||
      !IsValidCoordinate(fields[2], 2, 90.0) ||
      (fields[3] != "N" && fields[3] != "S") ||
      !IsValidCoordinate(fields[4], 3, 180.0) ||
      (fields[5] != "E" && fields[5] != "W")) {
    return false;
  }

  int fix_quality     = 0;
  int satellite_count = 0;
  if (!ParseInt(fields[6], &fix_quality) ||
      fix_quality < 1 || fix_quality > 8 ||
      !ParseInt(fields[7], &satellite_count) ||
      satellite_count < 0) {
    return false;
  }

  double numeric = 0.0;
  if (!ParseDouble(fields[8], &numeric) || numeric < 0.0 ||
      !ParseDouble(fields[9], &numeric) ||
      (fields[10] != "M" && fields[10] != "m") ||
      !ParseDouble(fields[11], &numeric) ||
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
