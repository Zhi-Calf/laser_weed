// PlanBowShapedPath::minWidthDirection 的单元测试 (src/bow.cpp 的"步骤 3")。
//
// 被测方法语义: 遍历多边形的每条边, 取"沿该边方向走垄时, 垂直方向的投影宽度"最小的边方向作为
// 扫描方向 (垄跨度最小 -> 垄数/掉头最少)。两条规则:
//   * 相邻边短于 kMinVertexSpacing (1e-6 m) 时跳过 (退化/重合边的方向没有意义);
//   * 只有"严格更小" (width < best_width - kEpsilon, kEpsilon = 1e-9) 才更新 -> 并列时取**先遇到**
//     的边, 且差值落在 kEpsilon 以内视为并列。
//
// 观测通道: 该方法是 private, 单测**不直接调用**它, 而是走公开接口 plan() —— generateWaypoints()
// 里把 minWidthDirection 的方向写进 getSweepDirDeg(), 所以它就是该方法的返回值。
// 方向是有向的, 但"扫描方向"本身无向 (v 与 -v 等价), 因此断言统一按 mod 180 比较。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "global_path_planner/bow.hpp"
#include "bow_test_facade.hpp"   // 测试专用: 把旧的有状态 API 投影到 plan(BowInput)->BowOutput 上

using bow_test::BowFacade;

namespace {

constexpr double kRidgeWidth = 3.0;
constexpr double kResolution = 0.1;

// 轴对齐矩形 (逆时针给出, 顺序交给 preparePolygon 规范化)。
Polygon_2 Rect(double width, double height) {
  return {Point_2(0.0, 0.0), Point_2(width, 0.0), Point_2(width, height), Point_2(0.0, height)};
}

// 绕原点旋转 deg 度的矩形。
Polygon_2 RotatedRect(double width, double height, double deg) {
  const double a = deg * M_PI / 180.0;
  const double c = std::cos(a);
  const double s = std::sin(a);
  const auto rot = [&](double x, double y) {
    return Point_2(c * x - s * y, s * x + c * y);
  };
  return {rot(0.0, 0.0), rot(width, 0.0), rot(width, height), rot(0.0, height)};
}

// 两个方向按 mod 180 的最小夹角 (deg) —— 无向朝向 (v 与 -v 等价), 并正确处理
// "≈ -1e-6° 与 0° 其实是同一个方向"这种跨 0/180 边界的比较 (先做差再规整, 不先各自规整)。
double AngleGapMod180(double a, double b) {
  double d = std::fmod(a - b, 180.0);
  if (d < 0.0)
    d += 180.0;
  return std::min(d, 180.0 - d);
}

// 跑一次规划, 取回 minWidthDirection 的结果 (经 getSweepDirDeg() 观测)。
// 注意: 只要 preparePolygon 通过, 方向就已写入 —— 即便后续内缩/垄线阶段失败也不影响该值;
// 反过来, 若 preparePolygon 失败则保持默认 0, 会伪装成"方向为 0°"。因此每个用例都先断言
// status == kOk, 保证观测到的确实是 minWidthDirection 的输出。
struct SweepProbe {
  PlanStatus status = PlanStatus::kOk;
  double deg        = 0.0;
};

SweepProbe Probe(const Polygon_2& field, double ridge = kRidgeWidth, double res = kResolution, double margin = 0.0) {
  BowFacade planner;
  const BowOutput result = planner.plan(field, ridge, res, margin);
  return SweepProbe{result.status, planner.getSweepDirDeg()};
}

}   // namespace

// 宽矩形 20 x 5: 沿长边 (水平) 走垄时垂直宽度 = 5, 沿短边 (竖直) 时 = 20 -> 取水平方向 0°。
TEST(BowMinWidthDirection, WideRectangleSweepsAlongLongEdge) {
  const SweepProbe probe = Probe(Rect(20.0, 5.0));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 0.0, 1e-9);
}

// 高矩形 5 x 20: 对称情形 -> 取竖直方向 90°。
TEST(BowMinWidthDirection, TallRectangleSweepsAlongLongEdge) {
  const SweepProbe probe = Probe(Rect(5.0, 20.0));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 90.0, 1e-9);
}

// 正方形 10 x 10: 四条边的垂直宽度都是 10 -> "严格更小"不成立, 取先遇到的边。
// preparePolygon 把起点规范到 (最小 x, 再最小 y) 且顺时针, 第一条边因此竖直 -> 90°。
TEST(BowMinWidthDirection, SquareTieKeepsFirstEdge) {
  const SweepProbe probe = Probe(Rect(10.0, 10.0));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 90.0, 1e-9);
}

// 差值 1e-6 > kEpsilon (1e-9): 水平边垂直宽度 9.999999 确实更小 -> 必须切换到 0°。
TEST(BowMinWidthDirection, DifferenceBeyondEpsilonSwitchesDirection) {
  const SweepProbe probe = Probe(Rect(10.0, 10.0 - 1e-6));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 0.0, 1e-9);
}

// 差值 5e-10 < kEpsilon: width < best - kEpsilon 不成立 -> 视为并列, 仍取先遇到的竖直边 90°。
TEST(BowMinWidthDirection, DifferenceWithinEpsilonKeepsFirstEdge) {
  const SweepProbe probe = Probe(Rect(10.0, 10.0 - 5e-10));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 90.0, 1e-9);
}

// 旋转 30° 的 20 x 5 矩形: 长边方向的垂直宽度 = 5 最小 -> 扫描方向随地块一起转 30°。
TEST(BowMinWidthDirection, RotatedRectangleFollowsRotation) {
  const SweepProbe probe = Probe(RotatedRect(20.0, 5.0, 30.0));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(AngleGapMod180(probe.deg, 30.0), 0.0, 1e-6);
}

// 旋转 -60° 的 20 x 5 矩形 -> 无向朝向 120° (=-60 mod 180), 验证符号/绕向不敏感。
TEST(BowMinWidthDirection, RotatedRectangleNegativeAngle) {
  const SweepProbe probe = Probe(RotatedRect(20.0, 5.0, -60.0));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(AngleGapMod180(probe.deg, 120.0), 0.0, 1e-6);
}

// 梯形 (与现场 yaml 同构地块的等比例缩小版): 上下水平边方向垂直宽度都是 5.2, 两条斜边 ~11.8 / ~12.0
// -> 取水平方向 0°。
TEST(BowMinWidthDirection, TrapezoidFieldSweepsHorizontally) {
  const Polygon_2 field  = {Point_2(0.0, 0.0), Point_2(12.0, 0.0), Point_2(11.0, 5.2), Point_2(-0.2, 5.2)};
  const SweepProbe probe = Probe(field, kRidgeWidth, kResolution, 1.0);
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(AngleGapMod180(probe.deg, 0.0), 0.0, 1e-9);
}

// 直角三角形 (0,0) (10,0) (0,4): 三条边的垂直宽度分别是 4 (水平边) / ~3.714 (斜边) / 10 (竖直边)。
// 最小宽度落在**斜边**上 —— 最小宽度方向不一定是"最长边方向", 断言方向与斜边平行。
TEST(BowMinWidthDirection, RightTriangleSweepsAlongHypotenuse) {
  const Polygon_2 field  = {Point_2(0.0, 0.0), Point_2(10.0, 0.0), Point_2(0.0, 4.0)};
  const SweepProbe probe = Probe(field);
  ASSERT_EQ(probe.status, PlanStatus::kOk);

  // 斜边 (最长边) 的无向朝向: 由两个端点独立算出, 与被测实现无关。
  const double hyp_deg = std::atan2(4.0 - 0.0, 0.0 - 10.0) * 180.0 / M_PI;
  EXPECT_NEAR(AngleGapMod180(probe.deg, hyp_deg), 0.0, 1e-6);
}

// 极扁矩形 100 x 0.5 -> 沿 100 m 的长边走垄 (0°), 掉头次数最少。
TEST(BowMinWidthDirection, ExtremelyWideRectangle) {
  const SweepProbe probe = Probe(Rect(100.0, 0.5));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 0.0, 1e-9);
}

// 极窄高矩形 0.5 x 100 -> 取竖直方向 90°。
TEST(BowMinWidthDirection, ExtremelyTallRectangle) {
  const SweepProbe probe = Probe(Rect(0.5, 100.0));
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(probe.deg, 90.0, 1e-9);
}

// 相邻边短于 kMinVertexSpacing (1e-6) 时跳过: 构造近似 20 x 5 的矩形, 左上角多一个距原顶点
// ~7.1e-7 的顶点, 规范化后多边形里因此有一条亚微米边 (它若参与比较, 垂直宽度会是 20 ——
// 一个毫无意义的"扫描方向候选")。断言跳过退化边后仍取水平长边方向 0°, 且规划照常成功。
TEST(BowMinWidthDirection, SkipsSubMicronEdge) {
  // 输入顺序刻意让两个近重合点 (0.0000005, 5.0000005) 与 (0,5) 不相邻 —— 否则 preparePolygon 的去重
  // 会先把它们合并, 亚微米边就构造不出来了。
  const Polygon_2 field  = {Point_2(0.0, 0.0), Point_2(20.0, 0.0), Point_2(0.0000005, 5.0000005), Point_2(20.0, 5.0), Point_2(0.0, 5.0)};
  const SweepProbe probe = Probe(field);
  ASSERT_EQ(probe.status, PlanStatus::kOk);
  EXPECT_NEAR(AngleGapMod180(probe.deg, 0.0), 0.0, 1e-4);
}

// 顶点顺序 / 绕向无关: 同一矩形用 4 种输入顺序, 扫描方向必须逐位一致。
// (preparePolygon 把起点规范到 (最小 x, 再最小 y) 且顺时针, 喂给 minWidthDirection 的环是确定的。)
TEST(BowMinWidthDirection, IndependentOfVertexOrderAndWinding) {
  const Polygon_2 ccw           = {Point_2(0.0, 0.0), Point_2(20.0, 0.0), Point_2(20.0, 5.0), Point_2(0.0, 5.0)};
  const Polygon_2 cw            = {Point_2(20.0, 5.0), Point_2(0.0, 5.0), Point_2(0.0, 0.0), Point_2(20.0, 0.0)};
  const Polygon_2 rotated_start = {Point_2(20.0, 0.0), Point_2(20.0, 5.0), Point_2(0.0, 5.0), Point_2(0.0, 0.0)};
  const Polygon_2 shifted_start = {Point_2(0.0, 5.0), Point_2(0.0, 0.0), Point_2(20.0, 0.0), Point_2(20.0, 5.0)};

  const SweepProbe ref = Probe(ccw);
  ASSERT_EQ(ref.status, PlanStatus::kOk);

  const SweepProbe a = Probe(cw);
  const SweepProbe b = Probe(rotated_start);
  const SweepProbe c = Probe(shifted_start);
  ASSERT_EQ(a.status, PlanStatus::kOk);
  ASSERT_EQ(b.status, PlanStatus::kOk);
  ASSERT_EQ(c.status, PlanStatus::kOk);

  EXPECT_DOUBLE_EQ(a.deg, ref.deg);
  EXPECT_DOUBLE_EQ(b.deg, ref.deg);
  EXPECT_DOUBLE_EQ(c.deg, ref.deg);
}
