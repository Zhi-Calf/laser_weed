// file 模式 (path_source=1) 的 **JSON 全覆盖路径文件**解析契约 (src/file_path_parser.hpp)。
//
// 文件由云端/外部工具产出, 解析的职责是"要么读对, 要么说清哪里不对" —— 两条都钉在这里:
//   * 合法样例: 每个字段都要落到位 (含 turn_flag -> 段类型, crc32 校验, 缺省值);
//   * 非法输入: 必须**拒绝**并给出中文原因 (旧版逐行 txt、crc32 不匹配都在拒绝之列),
//     而且不能留下半截结果 (失败时 points 必须为空, 否则节点会拿半个文件去发路径)。
// crc32 那几条尤其重要: 它是**跨语言契约** (云端 Python zlib.crc32 与这里的逐字节实现必须同值),
// 所以既钉住"给定样例算出哪个数", 也钉住"规范化文本长什么样"。
// 另外: 文件里**没有 yaw** (航向由节点按几何自己算) —— 带 yaw 的老文件必须照收, 且 yaw **不参与**
// crc32 (见 UnknownFieldsAreIgnoredAndNotHashed)。
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "file_path_parser.hpp"

using global_path_planner::BuildPathCanonicalString;
using global_path_planner::Crc32Ieee;
using global_path_planner::GeoPathFile;
using global_path_planner::GeoPathPoint;
using global_path_planner::ParseGeoPathFileJson;

namespace {

// 现场样例: 3 条垄 x 4 点, 每垄最后一个点是掉头起点 -> 3 个 turn_flag=true。
// crc32 = 云端按同一算法算出的值 (4025357972 = CRC32-IEEE("lon,lat,turn_flag;..."), 见下面的 Crc32 用例)。
// 文件里**不写 yaw**: 航向由节点按各段几何自己算。
constexpr const char* kSampleJson = R"({
  "task_name": "farm_full_coverage",
  "unit": "wgs84",
  "crc32": 4025357972,
  "path": [
    {"lon":113.6470,"lat":34.7580,"turn_flag":false},
    {"lon":113.6480,"lat":34.7580,"turn_flag":false},
    {"lon":113.6490,"lat":34.7580,"turn_flag":false},
    {"lon":113.6490,"lat":34.7588,"turn_flag":true},
    {"lon":113.6480,"lat":34.7588,"turn_flag":false},
    {"lon":113.6470,"lat":34.7588,"turn_flag":false},
    {"lon":113.6470,"lat":34.7596,"turn_flag":true},
    {"lon":113.6480,"lat":34.7596,"turn_flag":false},
    {"lon":113.6490,"lat":34.7596,"turn_flag":false},
    {"lon":113.6490,"lat":34.7604,"turn_flag":true},
    {"lon":113.6480,"lat":34.7604,"turn_flag":false},
    {"lon":113.6470,"lat":34.7604,"turn_flag":false}
  ]
})";

// 临时文件: 构造时写内容, 析构时删除 (名字带用例名, 失败时可以直接去 /tmp 看原文)。
class TempPathFile {
 public:
  TempPathFile(const std::string& name, const std::string& content)
      : path_("/tmp/megatrak_file_path_parser_" + name + ".json") {
    std::ofstream out(path_, std::ios::binary | std::ios::trunc);
    out << content;
  }
  ~TempPathFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  TempPathFile(const TempPathFile&)            = delete;
  TempPathFile& operator=(const TempPathFile&) = delete;
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

// 把一份 JSON 包成"只有 path 的最小合法文件"(unit 必须给, 否则单元先被拒)。
std::string WrapPath(const std::string& path_array, const std::string& unit = "wgs84") {
  return std::string("{ \"unit\": \"") + unit + "\", \"path\": " + path_array + " }";
}

// 解析并返回"失败原因"(成功时返回空串) —— 大多数用例只关心"成不成 + 原因里有什么";
// 想直接断言 bool 返回值的用例走三参形式 (见 SampleFileParsesEveryField / TooFewPointsIsRejected)。
std::string ParseReason(const std::string& file_path, GeoPathFile& out) {
  std::string reason;
  ParseGeoPathFileJson(file_path, out, reason);
  return reason;
}

}   // namespace

// 合法样例: 逐字段核对 (点数 / 元数据 / turn_flag / crc32)。
TEST(FilePathParser, SampleFileParsesEveryField) {
  const TempPathFile file("sample", kSampleJson);
  GeoPathFile parsed;

  // 成功路径直接用三参 API: 返回 true, 且 reason 被**清空** (调用方可以复用同一个变量)。
  std::string reason = "上一次的失败原因";
  ASSERT_TRUE(ParseGeoPathFileJson(file.path(), parsed, reason)) << reason;
  EXPECT_TRUE(reason.empty());

  ASSERT_EQ(parsed.points.size(), 12u);
  EXPECT_EQ(parsed.task_name, "farm_full_coverage");
  EXPECT_EQ(parsed.crc32, 4025357972);
  EXPECT_TRUE(parsed.crc32_verified);                              // 文件给了 crc32 且匹配
  EXPECT_EQ(parsed.crc32_computed, 4025357972u);

  // 第 1 点: 作业段 (turn_flag=false) —— 发布时 z=0。
  EXPECT_NEAR(parsed.points[0].lon, 113.6470, 1e-9);
  EXPECT_NEAR(parsed.points[0].lat, 34.7580, 1e-9);
  EXPECT_FALSE(parsed.points[0].turn_flag);

  // 第 4 点: 掉头段的落点 (turn_flag=true) —— 发布时 z=1。
  EXPECT_TRUE(parsed.points[3].turn_flag);

  // 3 条垄各一个掉头点: 这就是 /path 上 z=1 的段数来源。
  size_t turns = 0;
  for (const auto& p : parsed.points) {
    turns += p.turn_flag ? 1u : 0u;
  }
  EXPECT_EQ(turns, 3u);
}

// 缺省值: turn_flag 可以不给 (-> false); crc32 不给则**跳过校验** (现场手工调试的小文件),
// 但本机仍会把算出来的值带回来供日志使用。
TEST(FilePathParser, OptionalFieldsDefaultAndCrc32IsSkippedWhenAbsent) {
  const TempPathFile file("defaults", WrapPath(R"([{"lon":1.0,"lat":2.0},{"lon":3.0,"lat":4.0}])"));
  GeoPathFile parsed;

  ASSERT_TRUE(ParseReason(file.path(), parsed).empty());
  ASSERT_EQ(parsed.points.size(), 2u);
  EXPECT_FALSE(parsed.points[0].turn_flag);
  EXPECT_TRUE(parsed.task_name.empty());          // 元数据缺失不致命
  EXPECT_EQ(parsed.crc32, 0);
  EXPECT_FALSE(parsed.crc32_verified);            // 没给 -> 未校验
  EXPECT_EQ(parsed.crc32_computed, Crc32Ieee(BuildPathCanonicalString(parsed.points)));   // 但仍算了
}

// 旧版逐行 txt 一律判无效: 现场若还留着 "lat lon [alt]" 文件, 必须走"无效 + 故障码 10402"这条路,
// 而不是被当成某种奇怪 JSON 静默读成 0 个点。
TEST(FilePathParser, LegacyTextFileIsRejected) {
  const TempPathFile file("legacy_txt", "34.7580 113.6470\n34.7588 113.6480\n");
  GeoPathFile parsed;

  const std::string reason = ParseReason(file.path(), parsed);
  EXPECT_FALSE(reason.empty());
  EXPECT_NE(reason.find("JSON"), std::string::npos) << reason;
  EXPECT_TRUE(parsed.points.empty());
}

// unit 必须显式给出且只认 wgs84 (写错单位会让整条路径落到错误的位置上, 宁可判无效)。
TEST(FilePathParser, UnitMustBeWgs84) {
  const TempPathFile missing("unit_missing", R"({"path":[{"lon":1.0,"lat":2.0},{"lon":3.0,"lat":4.0}]})");
  const TempPathFile wrong("unit_wrong", WrapPath(R"([{"lon":1.0,"lat":2.0},{"lon":3.0,"lat":4.0}])", "enu"));
  GeoPathFile parsed;

  const std::string r1 = ParseReason(missing.path(), parsed);
  EXPECT_FALSE(r1.empty());
  EXPECT_NE(r1.find("unit"), std::string::npos) << r1;

  const std::string r2 = ParseReason(wrong.path(), parsed);
  EXPECT_FALSE(r2.empty());
  EXPECT_NE(r2.find("unit"), std::string::npos) << r2;
}

// 点数不足 2: 下游拿 1 个点没法跟踪 -> 无效。
TEST(FilePathParser, TooFewPointsIsRejected) {
  const TempPathFile file("too_few", WrapPath(R"([{"lon":1.0,"lat":2.0}])"));
  GeoPathFile parsed;

  // 失败路径直接断言 bool 契约: false + 原因 + out 保持空。
  std::string reason;
  EXPECT_FALSE(ParseGeoPathFileJson(file.path(), parsed, reason));
  EXPECT_NE(reason.find("1 < 2"), std::string::npos) << reason;
  EXPECT_TRUE(parsed.points.empty());
}
// 顶点字段问题: 缺 lon / lat 越界 / turn_flag 不是布尔 —— 都要指出是哪个点、哪个字段。
TEST(FilePathParser, BadPointFieldsAreRejectedWithLocation) {
  GeoPathFile parsed;

  {
    const TempPathFile file("no_lon", WrapPath(R"([{"lat":2.0},{"lon":3.0,"lat":4.0}])"));
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("path[0]"), std::string::npos) << reason;
    EXPECT_NE(reason.find("lon"), std::string::npos) << reason;
  }
  {
    const TempPathFile file("lat_range", WrapPath(R"([{"lon":1.0,"lat":91.5},{"lon":3.0,"lat":4.0}])"));
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("path[0]"), std::string::npos) << reason;
    EXPECT_NE(reason.find("经纬度非法"), std::string::npos) << reason;
  }
  {
    const TempPathFile file("turn_flag_type", WrapPath(R"([{"lon":1.0,"lat":2.0,"turn_flag":1},{"lon":3.0,"lat":4.0}])"));
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("path[0]"), std::string::npos) << reason;
    EXPECT_NE(reason.find("turn_flag"), std::string::npos) << reason;
  }
}

// 顶层结构不对 (数组 / 缺 path) 与文件不存在: 都要给原因, 且不崩。
TEST(FilePathParser, BadTopLevelAndMissingFileAreRejected) {
  GeoPathFile parsed;

  {
    const TempPathFile file("top_level", R"([{"lon":1.0,"lat":2.0}])");
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("JSON 对象"), std::string::npos) << reason;
  }
  {
    const TempPathFile file("no_path", R"({"unit":"wgs84","task_name":"x"})");
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("path"), std::string::npos) << reason;
  }
  {
    const std::string reason = ParseReason("/tmp/megatrak_file_path_parser_does_not_exist.json", parsed);
    EXPECT_NE(reason.find("不存在"), std::string::npos) << reason;
  }
}

// 失败不留半截: 同一个 GeoPathFile 先成功解析, 再用它接一次失败 —— points 必须被清空
// (节点侧就是拿这个对象填路径的, 半截数据会直接发到 /path 上)。
TEST(FilePathParser, FailureClearsPreviousResult) {
  const TempPathFile good("clear_good", kSampleJson);
  const TempPathFile bad("clear_bad", WrapPath(R"([{"lon":1.0,"lat":2.0}])"));
  GeoPathFile parsed;
  std::string reason;   // 同一个变量连用两次: 成功要清空, 失败要写上

  ASSERT_TRUE(ParseGeoPathFileJson(good.path(), parsed, reason)) << reason;
  EXPECT_TRUE(reason.empty());
  ASSERT_EQ(parsed.points.size(), 12u);

  EXPECT_FALSE(ParseGeoPathFileJson(bad.path(), parsed, reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_TRUE(parsed.points.empty());
  EXPECT_EQ(parsed.crc32, 0);
  EXPECT_FALSE(parsed.crc32_verified);
}

// ---- crc32 契约 (跨语言: 云端 Python zlib.crc32 必须与这里同值) ----

// 算法本身: 标准 CRC-32 (IEEE 802.3) 的已知向量 —— "123456789" 的 crc32 是 0xCBF43926
// (即 zlib.crc32(b"123456789") == 3421780262)。这条是"多项式/初值/取反都写对了"的依据。
TEST(FilePathParser, Crc32MatchesKnownIeeeVector) {
  EXPECT_EQ(Crc32Ieee("123456789"), 0xCBF43926u);
  EXPECT_EQ(Crc32Ieee(""), 0x00000000u);
}

// 规范化文本: 逐字节钉住格式 (定长小数 / ';' 只在点之间 / 行尾不加分隔符 / 负零归一) ——
// 云端就是照这个字符串算 crc, 这里一改那边立刻不匹配。
TEST(FilePathParser, CanonicalStringFormatIsFixed) {
  const std::vector<GeoPathPoint> points = {
    GeoPathPoint{34.7580, 113.6470, false},   // 顺序: lat, lon, turn_flag
    GeoPathPoint{34.7588, 113.6490, true},
  };
  EXPECT_EQ(BuildPathCanonicalString(points),
            "113.647000,34.758000,0;113.649000,34.758800,1");

  // 负零 (-0.0) 必须与 0 得到同一个串: 否则"同一个坐标换个写法"就会校验失败。
  const std::vector<GeoPathPoint> neg_zero = {GeoPathPoint{-0.0, -0.0, false}};
  EXPECT_EQ(BuildPathCanonicalString(neg_zero), "0.000000,0.000000,0");
}

// 样例整条路径的 crc32 (与云端对齐): 文件头与 README/yaml 里给的就是这个数。
TEST(FilePathParser, SampleCrc32EqualsZlibValueForCanonicalText) {
  const TempPathFile file("crc_sample", kSampleJson);
  GeoPathFile parsed;
  ASSERT_TRUE(ParseReason(file.path(), parsed).empty());
  EXPECT_EQ(Crc32Ieee(BuildPathCanonicalString(parsed.points)), 4025357972u);
}

// crc32 不匹配 -> 完全无效 (这就是"传输/落盘损坏"的拦截点), 原因里要能看出"文件是多少、实际是多少"。
TEST(FilePathParser, Crc32MismatchIsRejected) {
  const TempPathFile file("crc_bad", R"({
      "unit": "wgs84",
      "crc32": 12345,
      "path": [{"lon":1.0,"lat":2.0,"turn_flag":false},
               {"lon":3.0,"lat":4.0,"turn_flag":false}]
    })");
  GeoPathFile parsed;

  const std::string reason = ParseReason(file.path(), parsed);
  EXPECT_FALSE(reason.empty());
  EXPECT_NE(reason.find("crc32 不匹配"), std::string::npos) << reason;
  EXPECT_NE(reason.find("12345"), std::string::npos) << reason;
  EXPECT_NE(reason.find("实际算得"), std::string::npos) << reason;
  EXPECT_TRUE(parsed.points.empty());   // 不匹配 = 一点都不能用, 不留半截
}

// 只改一个坐标的第 6 位小数 -> crc32 也必须不匹配 (证明校验真的覆盖内容, 不是走过场)。
TEST(FilePathParser, Crc32DetectsTamperedCoordinate) {
  std::string tampered = kSampleJson;
  const size_t pos = tampered.find("113.6470,");
  ASSERT_NE(pos, std::string::npos);
  tampered.replace(pos, 9, "113.6471,");   // 长度不变: 只动最后一个数字
  const TempPathFile file("crc_tamper", tampered);
  GeoPathFile parsed;

  const std::string reason = ParseReason(file.path(), parsed);
  EXPECT_FALSE(reason.empty());
  EXPECT_NE(reason.find("crc32 不匹配"), std::string::npos) << reason;
}

// crc32 的写法: 必须是 0 ~ 4294967295 的整数 (字符串 / 负数 / 超出 uint32 一律拒, 并说清是 crc32)。
TEST(FilePathParser, Crc32MustBeUint32Integer) {
  GeoPathFile parsed;

  {
    const TempPathFile file("crc_neg", R"({"unit":"wgs84","crc32":-1,"path":[{"lon":1.0,"lat":2.0},{"lon":3.0,"lat":4.0}]})");
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("crc32"), std::string::npos) << reason;
  }
  {
    const TempPathFile file("crc_huge", R"({"unit":"wgs84","crc32":4294967296,"path":[{"lon":1.0,"lat":2.0},{"lon":3.0,"lat":4.0}]})");
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("crc32"), std::string::npos) << reason;
  }
  {
    const TempPathFile file("crc_str", R"({"unit":"wgs84","crc32":"abc","path":[{"lon":1.0,"lat":2.0},{"lon":3.0,"lat":4.0}]})");
    const std::string reason = ParseReason(file.path(), parsed);
    EXPECT_NE(reason.find("crc32"), std::string::npos) << reason;
  }
}

// 未知字段忽略 (前向兼容): 老文件里还带着 yaw / speed / vehicle_width —— 不报错;
// 而且它们**不参与** crc32 (校验值仍等于只按 lon/lat/turn_flag 算出来的那个)。
TEST(FilePathParser, UnknownFieldsAreIgnoredAndNotHashed) {
  const TempPathFile file("extra_fields", R"({
      "task_name": "legacy",
      "vehicle_width": 2.2,
      "unit": "wgs84",
      "crc32": 90886824,
      "path": [{"lon":113.6470,"lat":34.7580,"yaw":0.0,"speed":0.8,"turn_flag":false},
               {"lon":113.6490,"lat":34.7588,"yaw":90.0,"speed":0.5,"turn_flag":true}]
    })");
  GeoPathFile parsed;

  ASSERT_TRUE(ParseReason(file.path(), parsed).empty());
  ASSERT_EQ(parsed.points.size(), 2u);
  EXPECT_TRUE(parsed.crc32_verified);                        // 90886824 就是这两点的校验值 (yaw 不参与)
  EXPECT_EQ(parsed.crc32_computed, 90886824u);
  EXPECT_EQ(parsed.task_name, "legacy");
}

