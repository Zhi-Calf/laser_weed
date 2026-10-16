#pragma once

// file 模式 (path_source=1) 的 **JSON 全覆盖路径文件**解析。
//
// 放在 src/ 下 (不进 install): 节点 (src/global_path_planner.cpp) 与单测
// (test/test_file_path_parser.cpp) 共用同一份实现 —— 解析规则只写一遍, 与 bow_detail.hpp 同一套路。
//
// ---------------------------------------------------------------------------
// 文件格式 (云端/现场工具产出; **只支持 JSON**, 旧版逐行 "lat lon [alt]" 的 txt 已不再支持):
// {
//   "task_name": "farm_full_coverage",     // 只进日志
//   "unit": "wgs84",                       // **只认 "wgs84"** (经纬度); 其它值一律判无效
//   "crc32": 4025357972,                   // path[] 的校验值, 见下面 "crc32 校验"
//   "path": [
//     {"lon":113.6470,"lat":34.7580,"turn_flag":false},
//     {"lon":113.6490,"lat":34.7588,"turn_flag":true}
//   ]
// }
//
// **文件是"骨架"(粗点), 不是采好的最终路径**: 本节点把它当 BowSkeleton, 走与 local 弓形规划
// **同一套**采样实现 —— 按 path_resolution 采出 /global_path_planner/path (给路径跟踪)、按
// in_row_resolution 采出 /global_path_planner/pto_path (给执行机构/PTO); 两条都逐点带段类型 (z)
// 与航向 (orientation)。
//
// 字段语义:
//   * lon/lat  : 经纬度 (度, WGS84) —— 本节点用 gnss 的 ENU 原点把它们投影成平面坐标;
//   * turn_flag: true = "**从上一个点到这一点**的这一段是掉头/过渡段" —— 把掉头段上除起点 (它与
//                上一作业行共用) 以外的点都标上即可; 缺省 false。采样后掉头段上的点 z=1、
//                作业行上的点 z=0 (与 /global_path_planner/path 的"z 当段类型"约定一致,
//                见 bow.hpp 的"段类型"说明);
//   * **航向不在文件里给, 发布时也不带航向**: 本节点发布的 /global_path_planner/path 与
//                /global_path_planner/pto_path 的 pose.orientation
//                一律是单位四元数 (w=1) —— 路径朝向由**消费端按相邻点自己求切线** (tractor_kpi 的
//                路径分段器就是这么做的: 检测到 identity 就按几何算方向)。所以文件不需要 yaw,
//                节点也不算 (掉头段粗不粗只影响 z 的段标记与几何, 不影响朝向);
//   * 未知字段一律忽略: 文件里若还带着 yaw / speed / vehicle_width (都已从格式里删掉), 不报错也
//                不使用 —— nav_msgs/Path 里本来就没有能承载 speed/vehicle_width 的字段, 而航向由
//                节点自己算。
//
// crc32 校验 (规则如下, 云端照此实现即可):
//   * 算法: 标准 CRC-32 (IEEE 802.3: 反射多项式 0xEDB88320, 初值 0xFFFFFFFF, 结果再取反) ——
//     就是 zlib / Python `zlib.crc32` 的那一个;
//   * 覆盖对象: **只覆盖 path[]** (task_name / unit 等元数据不参与, 改任务名不必重算);
//   * 规范化文本 (逐字节固定, 见 BuildPathCanonicalString):
//       lon,lat,turn_flag;lon,lat,turn_flag;...
//     - lon/lat 定长 6 位小数, turn_flag 写 0/1;
//     - 点之间用 ';' 分隔, **行尾不加任何分隔符**; 没有空格、没有换行;
//     - 负零归一成 0 (否则 "-0.000000" 与 "0.000000" 会算出不同的 crc);
//   * 云端 Python 可一行复现 (与 C++ 侧逐字节一致):
//       s = ";".join(f"{p['lon']:.6f},{p['lat']:.6f},{1 if p['turn_flag'] else 0}"
//                    for p in doc["path"])
//       crc32 = zlib.crc32(s.encode("utf-8")) & 0xFFFFFFFF
//   * 判据: 文件里**给了** crc32 -> 必须匹配, 不匹配按无效处理 (传输/落盘损坏就在这里拦住 ——
//     别拿一条被改坏的路去开车); 文件里**没给** -> 跳过校验 (现场手工调试的小文件), 由调用方
//     在日志里记"未校验"。
//
// 返回 true = 成功 (out 填好, reason 清空); 返回 false = 失败 (out 保持空, reason 是中文原因,
// 含定位信息, 直接进日志与心跳 detail)。
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace global_path_planner {

// 只认这一个 unit: 经纬度。将来若支持平面坐标 (ENU), 再加一个取值并在这里分支。
inline constexpr const char* kSupportedUnit = "wgs84";

struct GeoPathPoint {
  double lat     = 0.0;
  double lon     = 0.0;
  bool turn_flag = false;
};

struct GeoPathFile {
  std::string task_name;             // 只进日志
  int64_t crc32           = 0;       // 文件里声明的 path[] 校验值 (没给时为 0)
  bool crc32_verified     = false;   // 文件里给了 crc32 且已校验通过 (没给 -> false, 调用方记"未校验")
  uint32_t crc32_computed = 0;       // 本机按同一算法算出的值 (校验失败时报"实际是多少")
  std::vector<GeoPathPoint> points;
};

// ---- crc32 校验 (算法与规则见文件头说明) ----

// 标准 CRC-32 (IEEE 802.3; 与 zlib / Python zlib.crc32 同值)。逐位实现: 一共几十行, 不值得
// 为它引一个库 (zlib 目前也不是本包的依赖); 数据量是"一条路径", 逐位足够快。
inline uint32_t Crc32Ieee(const std::string& data) {
  uint32_t crc = 0xFFFFFFFFu;
  for (const char ch : data) {
    crc ^= static_cast<uint32_t>(static_cast<unsigned char>(ch));
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

// 把 path[] 规范化成参与 crc32 的文本 —— **逐字节**与云端约定一致:
//   lon,lat,turn_flag;lon,lat,turn_flag;...   (见文件头 "crc32 校验")
// 定点小数是为了让"同一个坐标的不同写法"(113.647 / 113.64700000000001) 算出同一个值。
inline std::string BuildPathCanonicalString(const std::vector<GeoPathPoint>& points) {
  std::string text;
  text.reserve(points.size() * 32u + 1u);
  char buf[96];
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      text += ';';   // 分隔符只在点之间: 行尾不加 (云端按 join 实现即可)
    }
    const GeoPathPoint& p = points[i];
    // 负零归一: -0.0 打印成 "-0.000000", 与 "+0.000000" 是两个不同的字节串。
    const double lon = (p.lon == 0.0) ? 0.0 : p.lon;
    const double lat = (p.lat == 0.0) ? 0.0 : p.lat;
    std::snprintf(buf, sizeof(buf), "%.6f,%.6f,%d", lon, lat, p.turn_flag ? 1 : 0);
    text += buf;
  }
  return text;
}

// 解析 JSON 全覆盖路径文件。
// 返回 true = 成功: out 填好, reason 清空;
// 返回 false = 失败: out 保持**空**, reason 是中文原因 (含定位信息, 直接进日志与心跳 detail)。
inline bool ParseGeoPathFileJson(const std::string& file_path, GeoPathFile& out, std::string& reason) {
  // 失败时 out 一律保持**空**: 解析结果先全部写进 parsed, 走到最后 (点数够 + crc32 过了) 才交给
  // out —— 否则"点数不足"这类失败会把读到的半个文件留在 out 里, 现场就直接发到 /path 上了。
  out = GeoPathFile{};
  reason.clear();
  GeoPathFile parsed;

  // 失败出口: 统一"填原因 + 返回 false" (原因就是调用方要的现场信息)。
  const auto fail = [&reason](std::string text) {
    reason = std::move(text);
    return false;
  };

  std::ifstream in(file_path);
  if (!in.is_open()) {
    return fail("文件不存在或不可读");
  }
  std::ostringstream buf;
  buf << in.rdbuf();
  const std::string text = buf.str();

  // 不抛异常的解析形式: 现场给的是人手/工具产出的文件, 一个逗号错不该把节点弄崩。
  const nlohmann::json doc = nlohmann::json::parse(text, nullptr, false);
  if (doc.is_discarded()) {
    return fail("不是合法的 JSON (旧版逐行 \"lat lon [alt]\" 的 txt 已不再支持; 文件格式见 config/global_path_planner.yaml)");
  }
  if (!doc.is_object()) {
    return fail("顶层必须是 JSON 对象");
  }

  // unit: 必须显式给出且被支持 —— 写错单位会让整条路径落在错误的位置上, 宁可判无效。
  const auto unit_it = doc.find("unit");
  if (unit_it == doc.end() || !unit_it->is_string()) {
    return fail(std::string("缺少 unit 字段 (必须为 \"") + kSupportedUnit + "\")");
  }
  if (unit_it->get<std::string>() != kSupportedUnit) {
    return fail(std::string("unit 不支持: \"") + unit_it->get<std::string>() + "\" (只支持 \"" + kSupportedUnit + "\")");
  }

  // 可选元数据: task_name 只进日志; crc32 是**必校验**的 (给了就必须匹配, 见下)。
  if (const auto it = doc.find("task_name"); it != doc.end() && it->is_string()) {
    parsed.task_name = it->get<std::string>();
  }
  bool crc32_given = false;
  if (const auto it = doc.find("crc32"); it != doc.end() && !it->is_null()) {
    // 宽松收数值写法 (2147356890 与 2147356890.0 都认), 但值必须是 uint32 范围内的整数。
    if (!it->is_number()) {
      return fail("crc32 必须是整数 (uint32)");
    }
    const double crc_value = it->get<double>();
    if (crc_value < 0.0 || crc_value > 4294967295.0 || std::floor(crc_value) != crc_value) {
      return fail("crc32 必须是 0 ~ 4294967295 的整数");
    }
    parsed.crc32 = static_cast<int64_t>(crc_value);
    crc32_given  = true;
  }

  const auto path_it = doc.find("path");
  if (path_it == doc.end() || !path_it->is_array()) {
    return fail("缺少 path 数组");
  }

  parsed.points.reserve(path_it->size());
  size_t index = 0;
  for (const auto& item : *path_it) {
    const std::string where = "path[" + std::to_string(index) + "]";
    ++index;
    if (!item.is_object()) {
      return fail(where + " 必须是对象");
    }

    const auto lat_it = item.find("lat");
    const auto lon_it = item.find("lon");
    if (lat_it == item.end() || !lat_it->is_number()) {
      return fail(where + " 缺少 lat (或类型不是数值)");
    }
    if (lon_it == item.end() || !lon_it->is_number()) {
      return fail(where + " 缺少 lon (或类型不是数值)");
    }
    GeoPathPoint p;
    p.lat = lat_it->get<double>();
    p.lon = lon_it->get<double>();
    if (!std::isfinite(p.lat) || !std::isfinite(p.lon) || std::abs(p.lat) > 90.0 || std::abs(p.lon) > 180.0) {
      return fail(where + " 经纬度非法 (lat=" + std::to_string(p.lat) + ", lon=" + std::to_string(p.lon) + ")");
    }

    // turn_flag 之外的逐点字段 (yaw 等) 一律忽略: 航向由节点按几何自己算 (见文件头说明)。
    if (const auto tf = item.find("turn_flag"); tf != item.end() && !tf->is_null()) {
      if (!tf->is_boolean()) {
        return fail(where + " 字段 turn_flag 必须是布尔");
      }
      p.turn_flag = tf->get<bool>();
    }

    parsed.points.push_back(p);
  }

  if (parsed.points.size() < 2) {
    return fail("有效路点数量不足 (" + std::to_string(parsed.points.size()) + " < 2)");
  }

  // crc32: 一律先算出本机值 (校验失败时要报"实际是多少"), 文件里给了就必须一致。
  parsed.crc32_computed = Crc32Ieee(BuildPathCanonicalString(parsed.points));
  if (crc32_given) {
    if (static_cast<uint32_t>(parsed.crc32) != parsed.crc32_computed) {
      return fail("crc32 不匹配 (文件 " + std::to_string(parsed.crc32) + ", 实际算得 " + std::to_string(parsed.crc32_computed) + "): 文件可能在传输/落盘时损坏, 或云端算法与文件头约定不一致");
    }
    parsed.crc32_verified = true;
  }
  out = std::move(parsed);
  return true;
}

}   // namespace global_path_planner
