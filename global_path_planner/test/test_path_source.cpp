// path_source 取值域的契约 (include/global_path_planner/global_path_planner.hpp)。
//
// path_source 是**对外接口**: 只在 weeder_msgs/srv/SetPathSource 的请求字段里出现 (本节点没有
// 同名参数 —— 换来源走服务, 见 global_path_planner.hpp)。它一旦改号, App/MQTT/脚本与现场文档就
// 会对不上, 所以这里把"哪个号是哪个模式"钉死:
//   * 0 = local (本机弓形规划) / 1 = file (固定路径 JSON 文件), 与 global_path_planner.hpp 里的
//     PathSource 枚举**必须一致**;
//   * 其余取值 (含 file 的旧值 2) 一律拒绝, 且不改动 out (调用方据此保持原来源) —— 拒绝原因里已经写明
//     "只能是 0 (local) / 1 (file)" 与收到的值, 不为某个废弃取值单开提示。
// 以及 file 来源要加载的文件路径判据 (isUsableFilePath): file 请求的 path_file 必须是"展开后非空的
// 绝对路径" —— 位置只能从服务请求来 (path_file 参数已删除), 所以这条判据就是"能不能受理"的一半。
// 以及 local 来源要带的**多边形顶点**判据 (parsePolygonVertices): local 的顶点也只能从服务请求来
// (yaml 里的 poly_count / poly_* 已删除), 所以这条判据就是"能不能受理"的另一半 —— 与 path_file 那条
// 完全对称: 数量 [3, kMaxPolygonVertices] / poly 长度 == 3 * poly_count / 每个顶点三个分量都有限且
// |lat| <= 90、|lon| <= 180。⚠ 它只判"像不像一批顶点": 合法但几何上排不出路径是规划阶段的事
// (10403..10409), 不在受理处判。
// 以及随请求携带的另外两类字段 (**原同名参数 ridge_width / field_margin / in_row_resolution 已删除**,
// 所以"取值非法"只能在这里挡):
//   * parseLocalGeometryConfig —— local 的 ridge_width (> 0 的有限数, 单位 m) 与 field_margin
//     (>= 0 的有限数); ⚠ file 请求**不判也不生效** (几何全由文件决定);
//   * parseInRowResolution —— in_row_resolution, **两种来源都判** (file 的 PTO 掉头段也按它采):
//     0.0 (= 不产出 pto_path) 或 [kMinInRowResolution, kMaxInRowResolution] 内的有限数 —— 与旧参数
//     set 时的校验**逐字同一套判据**, 只是判的时机从"参数回调"搬到"服务受理";
//   * parseFirstRowMarginForRidge —— **yaml 参数** first_row_margin 与**请求里的** ridge_width 的相容性
//     ([kMinFirstRowMargin, 垄宽/2] m, 仅 local): 不相容 -> 整条请求被拒, **不夹取** (改造前是 .cpp 里的
//     ClampFirstRowMargin 把生效值悄悄夹进区间)。默认 0.005 m 因此要求 ridge_width >= 0.01 m。
// ⚠ 它们都只判"取值像不像话"; 合法但几何上排不出路径同样留给规划阶段 (10404..10410)。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "global_path_planner/global_path_planner.hpp"

using global_path_planner::isUsableFilePath;
using global_path_planner::kMaxInRowResolution;
using global_path_planner::kMaxPolygonVertices;
using global_path_planner::kMinInRowResolution;
using global_path_planner::kPathSourceFile;
using global_path_planner::kPathSourceLocal;
using global_path_planner::LocalGeometryConfig;
using global_path_planner::parseFirstRowMarginForRidge;
using global_path_planner::parseInRowResolution;
using global_path_planner::parseLocalGeometryConfig;
using global_path_planner::parsePathSource;
using global_path_planner::parsePolygonVertices;
using global_path_planner::pathSourceName;
using global_path_planner::PathSource;
// ⚠ kMinFirstRowMargin / kDefaultFirstRowMargin 是 bow.hpp 里的**全局**常量 (那个头文件不开 namespace),
// 所以这里不给它们写 using 声明 —— 与 test_bow_planner.cpp 的用法一致。

// 号与模式的对应关系 (改号就是改对外接口, 这条必须先红)。
TEST(PathSource, WireValuesAre0Local1File) {
  EXPECT_EQ(kPathSourceLocal, 0);
  EXPECT_EQ(kPathSourceFile, 1);
  EXPECT_EQ(static_cast<int>(PathSource::kLocal), 0);
  EXPECT_EQ(static_cast<int>(PathSource::kFile), 1);
}

// 合法取值: 解析成功并写出对应枚举。
TEST(PathSource, LegalValuesParse) {
  PathSource out = PathSource::kFile;   // 先塞个"错"的值, 确认真的被改写
  EXPECT_TRUE(parsePathSource(0, out));
  EXPECT_EQ(out, PathSource::kLocal);
  EXPECT_TRUE(parsePathSource(1, out));
  EXPECT_EQ(out, PathSource::kFile);

  EXPECT_STREQ(pathSourceName(PathSource::kLocal), "local");
  EXPECT_STREQ(pathSourceName(PathSource::kFile), "file");
}

// 非法取值: 拒绝且**不改动** out (调用方靠这一点保证"来源不变")。
TEST(PathSource, IllegalValuesAreRejectedWithoutTouchingOut) {
  for (const int64_t value : {2, 3, -1, 99, 100}) {
    PathSource out = PathSource::kFile;
    EXPECT_FALSE(parsePathSource(value, out)) << "取值 " << value << " 不该被接受";
    EXPECT_EQ(out, PathSource::kFile) << "取值 " << value << " 被拒时不该改动 out";
  }
}

// file 请求的文件路径判据: 收的是**已经展开** (~ / $VAR 由调用方 ExpandPath 处理) 的字符串 ——
// 只有"非空 + 绝对路径"才可用; 空与相对路径一律不可用 (服务据此 accepted=false, 来源与 /path 不变)。
TEST(PathSource, FilePathMustBeNonEmptyAbsolute) {
  EXPECT_TRUE(isUsableFilePath("/opt/agri/path/route.json"));
  EXPECT_TRUE(isUsableFilePath("/route.json"));
  EXPECT_TRUE(isUsableFilePath("/tmp/route.json"));

  EXPECT_FALSE(isUsableFilePath(""));                 // 请求没带 path_file
  EXPECT_FALSE(isUsableFilePath("route.json"));       // 裸文件名
  EXPECT_FALSE(isUsableFilePath("tmp/route.json"));   // 相对路径
  EXPECT_FALSE(isUsableFilePath("./route.json"));
  EXPECT_FALSE(isUsableFilePath("~/route.json"));   // 未展开的 ~ 不算绝对路径 (调用方先展开)
  EXPECT_FALSE(isUsableFilePath("$HOME/route.json"));
}

// ---- local 请求的多边形顶点 (SetPathSource 的 poly_count + poly -> 顶点数组) ----

// 合法: 3 个顶点也能受理 (地块边界最少 3 个点), 分量按 (lat, lon, alt) 逐位落位, reason 被清空。
TEST(PolygonVertices, ParsesFlatArrayIntoLatLonAltTriples) {
  std::string reason = "上一个请求留下的脏串";
  const auto vertices = parsePolygonVertices(3, {34.70316078, 112.39118753, 102.0,
                                                 34.70316002, 112.39133512, 202.0,
                                                 34.70320482, 112.39131299, 101.0}, reason);
  ASSERT_EQ(vertices.size(), 3u);
  EXPECT_DOUBLE_EQ(vertices[0][0], 34.70316078);
  EXPECT_DOUBLE_EQ(vertices[0][1], 112.39118753);
  EXPECT_DOUBLE_EQ(vertices[0][2], 102.0);
  EXPECT_DOUBLE_EQ(vertices[1][2], 202.0);
  EXPECT_DOUBLE_EQ(vertices[2][0], 34.70320482);
  EXPECT_DOUBLE_EQ(vertices[2][2], 101.0);
  EXPECT_TRUE(reason.empty()) << "受理成功时不该留下原因: " << reason;
}

// 上限值本身是**收下**的 (判据是 [3, kMaxPolygonVertices] 闭区间): 边界值能收, 越一个就拒。
TEST(PolygonVertices, AcceptsTheVertexLimitItself) {
  std::string reason;
  const std::vector<double> poly(static_cast<size_t>(kMaxPolygonVertices) * 3u, 1.0);
  const auto vertices = parsePolygonVertices(kMaxPolygonVertices, poly, reason);
  EXPECT_EQ(vertices.size(), kMaxPolygonVertices);
  EXPECT_TRUE(reason.empty()) << reason;
}

// 顶点太少 (0 / 1 / 2 个): 拒收并说明是哪一条判据 —— 门槛与 planGeometry 的 cell_.size() < 3 同源。
TEST(PolygonVertices, RejectsTooFewVertices) {
  for (const uint32_t count : {0u, 1u, 2u}) {
    std::string reason;
    const std::vector<double> poly(static_cast<size_t>(count) * 3u, 1.0);
    EXPECT_TRUE(parsePolygonVertices(count, poly, reason).empty()) << "poly_count = " << count << " 不该被接受";
    EXPECT_NE(reason.find("poly_count"), std::string::npos) << reason;
  }
}

// 顶点太多 (超过 kMaxPolygonVertices): 拒收 (请求体量兜底), 原因里给出上限值。
TEST(PolygonVertices, RejectsTooManyVertices) {
  const uint32_t count = kMaxPolygonVertices + 1u;
  std::string reason;
  const std::vector<double> poly(static_cast<size_t>(count) * 3u, 1.0);
  EXPECT_TRUE(parsePolygonVertices(count, poly, reason).empty());
  EXPECT_NE(reason.find(std::to_string(kMaxPolygonVertices)), std::string::npos) << reason;
}

// 两个字段必须自洽: 只给 count 不给数组 / 少一个分量 / 多一个分量一律拒 (现场拼脚本最容易错在这儿)。
TEST(PolygonVertices, RejectsCountAndLengthMismatch) {
  for (const std::vector<double>& poly : {std::vector<double>{},      // 只给了 poly_count
                                          std::vector<double>{34.7, 112.3},  // 连一个顶点都不够
                                          std::vector<double>(8, 1.0),       // 3 个顶点少一个分量
                                          std::vector<double>(12, 1.0)}) {   // 3 个顶点多一个分量
    std::string reason;
    EXPECT_TRUE(parsePolygonVertices(3, poly, reason).empty()) << "poly 长度 " << poly.size() << " 不该被接受";
    EXPECT_NE(reason.find("poly 长度"), std::string::npos) << reason;
  }
}

// 任何一个分量不是有限数 (NaN / inf, 现场常见于上游把空字段解析成 inf): 拒收, 不交给几何引擎。
TEST(PolygonVertices, RejectsNonFiniteComponents) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::vector<double>> bad = {
    {nan, 112.3, 102.0, 34.8, 112.4, 202.0, 34.9, 112.5, 302.0},   // lat
    {34.7, inf, 102.0, 34.8, 112.4, 202.0, 34.9, 112.5, 302.0},    // lon
    {34.7, 112.3, -inf, 34.8, 112.4, 202.0, 34.9, 112.5, 302.0},   // alt
  };
  for (const auto& poly : bad) {
    std::string reason;
    EXPECT_TRUE(parsePolygonVertices(3, poly, reason).empty());
    EXPECT_NE(reason.find("非有限数"), std::string::npos) << reason;
  }
}

// 经纬度越界 (|lat| > 90 / |lon| > 180) 拒收; 边界值恰好等于上限**要收下** (判据是闭区间);
// alt 不判区间 (只要求有限) —— 地块高程可以是任意值。
TEST(PolygonVertices, RejectsOutOfRangeLatLonButAcceptsTheLimits) {
  std::string reason;
  EXPECT_TRUE(parsePolygonVertices(3, {90.5, 112.3, 1.0, 34.8, 112.4, 1.0, 34.9, 112.5, 1.0}, reason).empty());
  EXPECT_NE(reason.find("越界"), std::string::npos) << reason;
  EXPECT_TRUE(parsePolygonVertices(3, {34.7, 180.5, 1.0, 34.8, 112.4, 1.0, 34.9, 112.5, 1.0}, reason).empty());
  EXPECT_NE(reason.find("越界"), std::string::npos) << reason;
  EXPECT_TRUE(parsePolygonVertices(3, {34.7, -112.3, 1.0, 34.8, -180.0001, 1.0, 34.9, 112.5, 1.0}, reason).empty());
  EXPECT_NE(reason.find("越界"), std::string::npos) << reason;

  const auto ok = parsePolygonVertices(3, {-90.0, -180.0, 1e6, 90.0, 180.0, -1e6, 0.0, 0.0, 0.0}, reason);
  ASSERT_EQ(ok.size(), 3u) << "边界值 (lat = ±90 / lon = ±180) 该被收下: " << reason;
  EXPECT_DOUBLE_EQ(ok[0][0], -90.0);
  EXPECT_DOUBLE_EQ(ok[0][1], -180.0);
  EXPECT_DOUBLE_EQ(ok[0][2], 1e6);
  EXPECT_DOUBLE_EQ(ok[1][0], 90.0);
  EXPECT_DOUBLE_EQ(ok[1][1], 180.0);
}

// ---- local 请求的几何参数 (SetPathSource 的 ridge_width + field_margin -> LocalGeometryConfig) ----

// 合法: 垄宽 > 0 且余量 >= 0 就受理, 两个分量逐位落位, reason 被清空。
// ⚠ field_margin = 0 是**合法**值 (文档里就是"明确不内缩"), 不是"没填"—— 不该被当成缺失拒收。
TEST(LocalGeometry, ParsesPositiveRidgeAndNonNegativeMargin) {
  std::string reason = "上一个请求留下的脏串";
  LocalGeometryConfig out;
  EXPECT_TRUE(parseLocalGeometryConfig(2.5, 0.0, out, reason));
  EXPECT_DOUBLE_EQ(out.ridge_width, 2.5);
  EXPECT_DOUBLE_EQ(out.field_margin, 0.0);
  EXPECT_TRUE(reason.empty()) << reason;

  // 小垄宽 (0.01 m) 与大余量同样合法 —— 判据里没有上界 (排不排得出路径是规划阶段的事)
  EXPECT_TRUE(parseLocalGeometryConfig(0.01, 25.0, out, reason));
  EXPECT_DOUBLE_EQ(out.ridge_width, 0.01);
  EXPECT_DOUBLE_EQ(out.field_margin, 25.0);
}

// 垄宽非法: <= 0 (-0.0 也算) 或 NaN/±inf 一律拒, reason 里点出 ridge_width, 且**不改动 out**。
// 垄宽 0 会让相邻垄线重叠 (几何退化), 负值让扫描方向反过来 —— 都在受理处挡掉, 不进规划引擎。
TEST(LocalGeometry, RejectsNonPositiveOrNonFiniteRidgeWidth) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {0.0, -0.0, -1.0, nan, inf, -inf}) {
    std::string reason;
    LocalGeometryConfig out;
    out.ridge_width  = 7.0;   // 先塞个"旧值", 确认被拒时不动它
    out.field_margin = 8.0;
    EXPECT_FALSE(parseLocalGeometryConfig(bad, 0.5, out, reason)) << "ridge_width = " << bad << " 不该被接受";
    EXPECT_NE(reason.find("ridge_width"), std::string::npos) << reason;
    EXPECT_DOUBLE_EQ(out.ridge_width, 7.0) << "被拒时不该改动 out";
    EXPECT_DOUBLE_EQ(out.field_margin, 8.0) << "被拒时不该改动 out";
  }
}

// 田边余量非法: 负值或 NaN/±inf 一律拒, reason 里点出 field_margin。
// ⚠ 界值 0 合法 (见上一条), 所以这里取 -0.001 而不是 0 —— 判据是 field_margin < 0 才拒。
TEST(LocalGeometry, RejectsNegativeOrNonFiniteFieldMargin) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {-0.001, -1.0, nan, inf, -inf}) {
    std::string reason;
    LocalGeometryConfig out;
    out.ridge_width  = 2.5;
    out.field_margin = 0.5;
    EXPECT_FALSE(parseLocalGeometryConfig(2.5, bad, out, reason)) << "field_margin = " << bad << " 不该被接受";
    EXPECT_NE(reason.find("field_margin"), std::string::npos) << reason;
    EXPECT_DOUBLE_EQ(out.field_margin, 0.5) << "被拒时不该改动 out";
  }
}

// ---- yaml 参数 first_row_margin 与请求里的垄宽必须相容 (相容性判据, **仅 local**) ----
//
// 这条判据取代了旧的夹取 (ClampFirstRowMargin): 越界不再"悄悄换个值", 而是把整条 local 请求拒掉 ——
// 所以两端都必须钉死, 而且它们都是**跨接口**的: 下界来自 bow.hpp 的常量 (Clipper2 量化网格), 上界来自
// **请求里的** ridge_width (同一个 first_row_margin 在窄垄宽上合法、在更窄的垄宽上就非法)。

// 下界常量本身也钉一下 (改它 = 改现场能受理的最小余量; 上界是 0.5 * 请求的 ridge_width, 没有常量)。
TEST(FirstRowMarginRange, LowerBoundConstantIsTheClipperGridFloor) {
  EXPECT_DOUBLE_EQ(kMinFirstRowMargin, 0.001);
  EXPECT_DOUBLE_EQ(kDefaultFirstRowMargin, 0.005);
}

// 合法: 落在 [kMinFirstRowMargin, 垄宽/2] 内 —— 两端都取得到 (界值判成开区间会误拒合法请求),
// 且成功时 reason 被清空 (受理处直接把它当 reason 回给调用方)。
TEST(FirstRowMarginRange, AcceptsClosedIntervalBounds) {
  std::string reason = "上一个请求留下的脏串";
  EXPECT_TRUE(parseFirstRowMarginForRidge(kDefaultFirstRowMargin, 2.5, reason));   // 默认值 + 常见垄宽
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseFirstRowMarginForRidge(kMinFirstRowMargin, 2.5, reason)) << "恰好等于下界该收";
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseFirstRowMarginForRidge(1.25, 2.5, reason)) << "恰好等于 垄宽/2 = 1.25 该收";
  EXPECT_TRUE(reason.empty()) << reason;
  // 默认值 0.005 m 的**最小可用垄宽**: 2 * 0.005 = 0.01 m (恰好等于上界)
  EXPECT_TRUE(parseFirstRowMarginForRidge(kDefaultFirstRowMargin, 0.01, reason));
  EXPECT_TRUE(reason.empty()) << reason;
}

// 小于下界 (含 0 / 负数 / 刚好差一个 ulp) -> 拒, reason 里点出 first_row_margin 与 ridge_width
// (现场据此知道该改哪一边)。下界存在的理由: Clipper2 量化网格 ≈1 mm, 更小的余量会被量化吃掉。
TEST(FirstRowMarginRange, RejectsBelowLowerBound) {
  for (const double bad : {0.0, -0.0, -1.0, 0.0009, std::nextafter(kMinFirstRowMargin, 0.0)}) {
    std::string reason;
    EXPECT_FALSE(parseFirstRowMarginForRidge(bad, 2.5, reason)) << "first_row_margin = " << bad << " 不该被接受";
    EXPECT_NE(reason.find("first_row_margin"), std::string::npos) << reason;
    EXPECT_NE(reason.find("ridge_width"), std::string::npos) << reason;
  }
}

// 超过 垄宽/2 -> 拒 —— **上界随请求变**: 同一个余量在窄垄宽上被拒、在宽垄宽上被接受 (这正是"必须判在受理处"
// 的理由); 默认值 0.005 m 配 0.009 m 的垄宽 (上界 0.0045) 同样被拒。
TEST(FirstRowMarginRange, RejectsAboveHalfRidgeWidth) {
  std::string reason;
  EXPECT_FALSE(parseFirstRowMarginForRidge(0.01, 0.019, reason)) << "0.019/2 = 0.0095 < 0.01: 该拒";
  EXPECT_NE(reason.find("first_row_margin"), std::string::npos) << reason;
  EXPECT_TRUE(parseFirstRowMarginForRidge(0.01, 0.02, reason)) << "0.02/2 = 0.01: 同一个值这里该收";
  EXPECT_TRUE(reason.empty()) << reason;

  EXPECT_FALSE(parseFirstRowMarginForRidge(kDefaultFirstRowMargin, 0.009, reason));
  EXPECT_NE(reason.find("ridge_width"), std::string::npos) << reason;
  EXPECT_FALSE(parseFirstRowMarginForRidge(std::nextafter(1.25, 2.0), 2.5, reason)) << "刚超过 垄宽/2: 该拒";
}

// 非有限 (NaN / ±inf) -> 拒: 这种值必须在受理处消失, 否则会一路糊到几何里 (Clipper 偏置 / 行距递推)。
TEST(FirstRowMarginRange, RejectsNonFinite) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {nan, inf, -inf}) {
    std::string reason;
    EXPECT_FALSE(parseFirstRowMarginForRidge(bad, 2.5, reason)) << "first_row_margin = " << bad << " 不该被接受";
    EXPECT_NE(reason.find("first_row_margin"), std::string::npos) << reason;
  }
}

// ---- 随请求携带的 PTO 行内分辨率 (in_row_resolution, **两种来源都必填**) ----

// 取值域本身是**对外接口**: 服务定义里逐字写着 0.0 = 不产出 pto_path 与 [0.01, 100] 这个区间,
// 常量改了服务文档就与代码分叉 (与 path_source 的 0/1 同一种"号与语义必须对齐"的约定)。
TEST(InRowResolution, WireValueRangeIsPartOfTheInterface) {
  EXPECT_DOUBLE_EQ(kMinInRowResolution, 0.01);
  EXPECT_DOUBLE_EQ(kMaxInRowResolution, 100.0);
}

// 合法: 0.0 (不产出 pto_path) 与**闭区间两端**都要收下 —— 界值判成开区间就会误拒合法请求。
TEST(InRowResolution, AcceptsZeroAndClosedRangeBounds) {
  std::string reason = "上一个请求留下的脏串";
  EXPECT_TRUE(parseInRowResolution(0.0, reason));
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseInRowResolution(kMinInRowResolution, reason)) << reason;
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseInRowResolution(kMaxInRowResolution, reason)) << reason;
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseInRowResolution(1.0, reason)) << reason;
}

// 非法: 越界 (两侧各取一点 + 1e-6 这种"点数爆炸"的极小值) / 负值 / NaN / ±inf 一律拒 ——
// 极小值那一条是这条判据存在的**主要理由** (点数按 1/步长 增长, 300 m 的垄给 1e-6 = 3e8 个点),
// NaN/Inf 则会让 PTO 路径静默消失 (have_pto 恒为 false)。
// reason 里必须给出取值域 (含 0.0 这个特例) 与收到的值, 现场据此直接改请求。
TEST(InRowResolution, RejectsOutOfRangeAndNonFinite) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {0.009, 100.001, 1e-6, -1.0, nan, inf, -inf}) {
    std::string reason;
    EXPECT_FALSE(parseInRowResolution(bad, reason)) << "in_row_resolution = " << bad << " 不该被接受";
    EXPECT_NE(reason.find("in_row_resolution"), std::string::npos) << reason;
    EXPECT_NE(reason.find("0.0"), std::string::npos) << "reason 该点出 0.0 这个合法特例: " << reason;
  }
}
