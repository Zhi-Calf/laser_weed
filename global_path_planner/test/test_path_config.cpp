// global_path_planner YAML 配置的取值契约 (本地弓形规划)。
//
// 这些校验函数在节点启动读取 YAML 时调用；参数非法会让节点启动失败，而不是
// 在运行期静默回退：
//   * poly: 必须是 3 的倍数且至少 3 个顶点，每个顶点为 (lat, lon, alt)，
//     经纬度有限且不越界;
//   * ridge_width > 0、field_margin >= 0;
//   * first_row_margin 必须落在 [kMinFirstRowMargin, ridge_width/2]。
// 合法但几何上排不出路径仍属于规划阶段错误 (10402..10408)。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "global_path_planner/global_path_planner.hpp"

using global_path_planner::kMaxPolygonVertices;
using global_path_planner::LocalGeometryConfig;
using global_path_planner::parseFirstRowMarginForRidge;
using global_path_planner::parseLocalGeometryConfig;
using global_path_planner::parsePolygonVertices;
// ⚠ kMinFirstRowMargin 是 bow.hpp 里的全局常量 (那个头文件不开 namespace),
// 所以这里不给它写 using 声明 —— 与 test_bow_planner.cpp 的用法一致。

// ---- YAML 多边形的顶点判据 ----

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

// 顶点太多 (超过 kMaxPolygonVertices): 拒收, 原因里给出上限值。
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

// ---- YAML 几何参数 (ridge_width + field_margin -> LocalGeometryConfig) ----

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

// ---- YAML first_row_margin 与 ridge_width 必须相容 (仅 local) ----
//
// 越界不再"悄悄换个值", 而是让节点启动失败 —— 所以两端都必须钉死:
// 下界来自 bow.hpp 的常量 (Clipper2 量化网格), 上界来自 YAML ridge_width。

// 下界常量本身也钉一下 (改它 = 改现场能用的最小余量; 上界是 0.5 * ridge_width, 没有常量)。
TEST(FirstRowMarginRange, LowerBoundConstantIsTheClipperGridFloor) {
  EXPECT_DOUBLE_EQ(kMinFirstRowMargin, 0.001);
}

// 合法: 落在 [kMinFirstRowMargin, 垄宽/2] 内 —— 两端都取得到,
// 且成功时 reason 被清空。
TEST(FirstRowMarginRange, AcceptsClosedIntervalBounds) {
  std::string reason = "上一个请求留下的脏串";
  EXPECT_TRUE(parseFirstRowMarginForRidge(0.005, 2.5, reason));   // 默认值 + 常见垄宽
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseFirstRowMarginForRidge(kMinFirstRowMargin, 2.5, reason)) << "恰好等于下界该收";
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_TRUE(parseFirstRowMarginForRidge(1.25, 2.5, reason)) << "恰好等于 垄宽/2 = 1.25 该收";
  EXPECT_TRUE(reason.empty()) << reason;
  // 默认值 0.005 m 的**最小可用垄宽**: 2 * 0.005 = 0.01 m (恰好等于上界)
  EXPECT_TRUE(parseFirstRowMarginForRidge(0.005, 0.01, reason));
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

// 超过 垄宽/2 -> 拒 —— 上界随 ridge_width 变化; 默认值 0.005 m
// 配 0.009 m 的垄宽 (上界 0.0045) 同样被拒。
TEST(FirstRowMarginRange, RejectsAboveHalfRidgeWidth) {
  std::string reason;
  EXPECT_FALSE(parseFirstRowMarginForRidge(0.01, 0.019, reason)) << "0.019/2 = 0.0095 < 0.01: 该拒";
  EXPECT_NE(reason.find("first_row_margin"), std::string::npos) << reason;
  EXPECT_TRUE(parseFirstRowMarginForRidge(0.01, 0.02, reason)) << "0.02/2 = 0.01: 同一个值这里该收";
  EXPECT_TRUE(reason.empty()) << reason;

  EXPECT_FALSE(parseFirstRowMarginForRidge(0.005, 0.009, reason));
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
