#include "comm_gateway/path_file.hpp"

#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <vector>

namespace comm_gateway::path_file {

namespace {

struct Point {
  double lat     = 0.0;
  double lon     = 0.0;
  bool turn_flag = false;
};

// 参与 crc32 的规范化文本: "lon,lat,turn_flag;lon,lat,turn_flag;..." (6 位小数, 负零归一, 行尾无分隔符)
std::string CanonicalString(const std::vector<Point>& points) {
  std::string text;
  text.reserve(points.size() * 32u + 1u);
  char buf[96];
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      text += ';';
    }
    const double lon = (points[i].lon == 0.0) ? 0.0 : points[i].lon;
    const double lat = (points[i].lat == 0.0) ? 0.0 : points[i].lat;
    std::snprintf(buf, sizeof(buf), "%.6f,%.6f,%d", lon, lat, points[i].turn_flag ? 1 : 0);
    text += buf;
  }
  return text;
}

}   // namespace

uint32_t Crc32Ieee(const std::string& data) {
  uint32_t crc = 0xFFFFFFFFu;
  for (const char ch : data) {
    crc ^= static_cast<uint32_t>(static_cast<unsigned char>(ch));
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

bool Validate(const std::string& text, Summary& out, std::string& err) {
  out = Summary{};
  const nlohmann::json doc = nlohmann::json::parse(text, nullptr, false);
  if (doc.is_discarded()) {
    err = "不是合法 JSON";
    return false;
  }
  if (!doc.is_object()) {
    err = "顶层必须是 JSON 对象";
    return false;
  }

  const auto unit = doc.find("unit");
  if (unit == doc.end() || !unit->is_string() || unit->get<std::string>() != "wgs84") {
    err = "unit 必须为 \"wgs84\"";
    return false;
  }

  Summary parsed;
  if (const auto it = doc.find("task_name"); it != doc.end() && it->is_string()) {
    parsed.task_name = it->get<std::string>();
  }

  bool crc_given    = false;
  uint32_t crc_file = 0;
  if (const auto it = doc.find("crc32"); it != doc.end() && !it->is_null()) {
    const double v = it->is_number() ? it->get<double>() : -1.0;
    if (v < 0.0 || v > 4294967295.0 || std::floor(v) != v) {
      err = "crc32 必须是 0 ~ 4294967295 的整数";
      return false;
    }
    crc_file  = static_cast<uint32_t>(v);
    crc_given = true;
  }

  const auto path = doc.find("path");
  if (path == doc.end() || !path->is_array()) {
    err = "缺少 path 数组";
    return false;
  }

  std::vector<Point> points;
  points.reserve(path->size());
  for (size_t i = 0; i < path->size(); ++i) {
    const auto& item        = (*path)[i];
    const std::string where = "path[" + std::to_string(i) + "]";
    if (!item.is_object()) {
      err = where + " 必须是对象";
      return false;
    }
    const auto lat = item.find("lat");
    const auto lon = item.find("lon");
    if (lat == item.end() || !lat->is_number() || lon == item.end() || !lon->is_number()) {
      err = where + " 缺少 lat / lon (或不是数值)";
      return false;
    }
    Point p;
    p.lat = lat->get<double>();
    p.lon = lon->get<double>();
    if (std::abs(p.lat) > 90.0 || std::abs(p.lon) > 180.0) {
      err = where + " 经纬度越界 (lat=" + std::to_string(p.lat) + ", lon=" + std::to_string(p.lon) + ")";
      return false;
    }
    if (const auto tf = item.find("turn_flag"); tf != item.end() && !tf->is_null()) {
      if (!tf->is_boolean()) {
        err = where + " 字段 turn_flag 必须是布尔";
        return false;
      }
      p.turn_flag = tf->get<bool>();
    }
    points.push_back(p);
  }
  if (points.size() < 2) {
    err = "路点数量不足 (" + std::to_string(points.size()) + " < 2)";
    return false;
  }

  if (crc_given) {
    const uint32_t computed = Crc32Ieee(CanonicalString(points));
    if (computed != crc_file) {
      err = "crc32 不匹配 (文件 " + std::to_string(crc_file) + ", 实际 " + std::to_string(computed) + ")";
      return false;
    }
    parsed.crc32_verified = true;
  }
  parsed.num_points = points.size();
  out               = std::move(parsed);
  return true;
}

}   // namespace comm_gateway::path_file
