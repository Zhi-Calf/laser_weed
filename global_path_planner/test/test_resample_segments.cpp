// 外部骨架 (file 模式) 的采样契约 (src/bow.cpp 的 resampleBowSkeleton)。
//
// file 模式下发的 JSON 是**骨架** (粗点 + 逐点 turn_flag), 节点要按 path_resolution 采出
// /global_path_planner/path、按 in_row_resolution 采出 /global_path_planner/pto_path ——
// 与 local 模式**共用** resampleBowSkeleton。这里钉住外部骨架特有的两条:
//   ① **段类型**按 BowSkeleton::seg_types 判 (外部文件的 turn_flag 标在"掉头段走完的那个点"上,
//      这个口径表达不成点类型: 掉头段与其后的作业行共用一个端点);
//   ② **粗掉头段** (比 path_resolution 长的掉头段, 外部骨架才会出现) 在 /global_path_planner/pto_path
//      里也要按 path_resolution 采出弧上的点, 不能只留两个端点 (否则 PTO 路径在掉头处是折线)。
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "global_path_planner/bow.hpp"

namespace {

BowSkeleton SkeletonOf(std::vector<Point_2> points, std::vector<uint8_t> types, std::vector<uint8_t> seg_types) {
  BowSkeleton s;
  s.points    = std::move(points);
  s.types     = std::move(types);
  s.seg_types = std::move(seg_types);
  return s;
}

// 竖直/水平直线上的等间隔点 (便于按坐标断言采样位置)。
std::vector<Point_2> Line(const Point_2& from, const Point_2& to, size_t count) {
  std::vector<Point_2> pts;
  for (size_t i = 0; i < count; ++i) {
    const double t = (count == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1);
    pts.emplace_back(from.x() + (to.x() - from.x()) * t, from.y() + (to.y() - from.y()) * t);
  }
  return pts;
}

// 逐点间距 (相邻两点距离), 用来检查采样密度。
std::vector<double> Steps(const std::vector<Point_2>& pts) {
  std::vector<double> out;
  for (size_t i = 1; i < pts.size(); ++i) {
    out.push_back(std::hypot(pts[i].x() - pts[i - 1].x(), pts[i].y() - pts[i - 1].y()));
  }
  return out;
}

}   // namespace

// ① 段类型以 seg_types 为准: 掉头段 (A->B) 与作业段 (B->C) 共用端点 B, 只有段类型能表达清楚。
// 采样结果: 段端点用自己的**点**类型, 插值出来的点取所在**段**的类型。
TEST(ExternalSkeleton, SegmentTypesWinOverPointTypeRule) {
  // A(0,0) -- B(10,0) -- C(20,0); 点类型: B 是掉头点; 段类型: A->B 是掉头段, B->C 是作业段。
  const BowSkeleton sk = SkeletonOf(Line({0.0, 0.0}, {20.0, 0.0}, 3),
                                    {kSegWork, kSegTurn, kSegWork},
                                    {kSegTurn, kSegWork});
  std::vector<Point_2> path;
  std::vector<uint8_t> path_types;
  std::vector<Point_2> pto;
  std::vector<uint8_t> pto_types;

  resampleBowSkeleton(sk, 2.5, 5.0, path, pto, pto_types, &path_types);

  // /path: A --2.5/5/7.5-- B --12.5/15/17.5-- C (段端点原样保留, 段内按 path_resolution=2.5 插值)。
  ASSERT_EQ(path.size(), 9u);
  EXPECT_EQ(path_types.size(), path.size());
  EXPECT_DOUBLE_EQ(path[0].x(), 0.0);
  EXPECT_DOUBLE_EQ(path[1].x(), 2.5);
  EXPECT_DOUBLE_EQ(path[4].x(), 10.0);
  EXPECT_DOUBLE_EQ(path[8].x(), 20.0);
  EXPECT_EQ(path_types[0], kSegWork);   // 首点: 自己的点类型 (它不在掉头段上)
  EXPECT_EQ(path_types[1], kSegTurn);   // 掉头段内插出来的点
  EXPECT_EQ(path_types[4], kSegTurn);   // 掉头段终点 = 文件里标 turn_flag 的那个点
  EXPECT_EQ(path_types[5], kSegWork);   // 作业段内插出来的点 —— 只看点类型会把它误判成掉头
  EXPECT_EQ(path_types[8], kSegWork);

  // PTO: 作业段按 in_row_resolution=5 加密 (1 个内插点), 掉头段按 path_resolution=2.5。
  ASSERT_EQ(pto.size(), 7u);
  EXPECT_EQ(pto_types.size(), pto.size());
  EXPECT_DOUBLE_EQ(pto[4].x(), 10.0);
  EXPECT_EQ(pto_types[4], kSegTurn);
  EXPECT_EQ(pto_types[6], kSegWork);
}

// ② 粗掉头段 (比 path_resolution 长) 在 PTO 里也必须采出弧上的点 —— file 模式的掉头段可能只有
// 两个端点, 若原样保留会是一条折线 (弧上最近点定位与航向差分最敏感)。
TEST(ExternalSkeleton, CoarseTurnIsInterpolatedForPtoAsWell) {
  // 一条 20 m 的掉头段两端点 + 一条作业段; 掉头段在文件里没有中间点。
  const BowSkeleton sk = SkeletonOf({Point_2(0.0, 0.0), Point_2(0.0, 20.0), Point_2(10.0, 20.0)},
                                    {kSegWork, kSegTurn, kSegWork},
                                    {kSegTurn, kSegWork});
  std::vector<Point_2> path;
  std::vector<uint8_t> path_types;
  std::vector<Point_2> pto;
  std::vector<uint8_t> pto_types;

  resampleBowSkeleton(sk, 2.0, 4.0, path, pto, pto_types, &path_types);

  // /path: 掉头段 20 m / 2.0 m -> 首点 + 9 个插值点 + 端点; 作业段 10 m / 2.0 -> 4 个插值点 + 端点。
  ASSERT_EQ(path.size(), 1u + 9u + 1u + 4u + 1u);
  // PTO: 掉头段用 path_resolution=2.0 (9 个插值点, 与 /path 同密度), 作业段用 in_row_resolution=4.0。
  ASSERT_EQ(pto.size(), 1u + 9u + 1u + 2u + 1u);
  size_t pto_turn = 0;
  for (uint8_t t : pto_types) {
    pto_turn += (t == kSegTurn) ? 1u : 0u;
  }
  EXPECT_EQ(pto_turn, 10u);   // 掉头段的 9 个插值点 + 端点, 一个都不少
  // 掉头段上的 PTO 点间距 = path_resolution (2.0 m), 不是 in_row_resolution。
  for (size_t i = 1; i <= 9; ++i) {
    EXPECT_NEAR(std::hypot(pto[i].x() - pto[i - 1].x(), pto[i].y() - pto[i - 1].y()), 2.0, 1e-9) << i;
  }
}

// ③ 本机规划的老行为不变: 掉头弧已在几何阶段采好 (每段都短于 path_resolution) -> PTO 里原样保留,
// 不重复插值 (与单测 F2cPto.TurnArcsAreNotResampled 同一条约定)。
TEST(ExternalSkeleton, PresampledShortTurnSegmentsAreKeptAsIs) {
  const BowSkeleton sk = SkeletonOf({Point_2(0.0, 0.0), Point_2(0.1, 0.1), Point_2(0.2, 0.0), Point_2(10.0, 0.0)},
                                    {kSegWork, kSegTurn, kSegTurn, kSegWork},
                                    {kSegTurn, kSegTurn, kSegWork});
  std::vector<Point_2> path;
  std::vector<uint8_t> path_types;
  std::vector<Point_2> pto;
  std::vector<uint8_t> pto_types;

  resampleBowSkeleton(sk, 5.0, 10.0, path, pto, pto_types, &path_types);

  // 弧上的两个短段 (0.1 m 级) 不产生新点: PTO 的弧点与骨架逐点相同。
  ASSERT_GE(pto.size(), 3u);
  EXPECT_DOUBLE_EQ(pto[0].x(), 0.0);
  EXPECT_DOUBLE_EQ(pto[1].x(), 0.1);
  EXPECT_DOUBLE_EQ(pto[2].x(), 0.2);
  EXPECT_EQ(pto_types[1], kSegTurn);
  EXPECT_EQ(pto_types[2], kSegTurn);
}

// ④ 采样密度: 外部骨架的直段按 path_resolution 采 /path、按 in_row_resolution 采 PTO
// (这就是 file 模式要的"文件是骨架、节点负责插值")。
TEST(ExternalSkeleton, ResolutionsDriveSamplingDensity) {
  const BowSkeleton sk = SkeletonOf({Point_2(0.0, 0.0), Point_2(10.0, 0.0)},
                                    {kSegWork, kSegWork},
                                    {kSegWork});
  std::vector<Point_2> path;
  std::vector<uint8_t> path_types;
  std::vector<Point_2> pto;
  std::vector<uint8_t> pto_types;

  resampleBowSkeleton(sk, 0.2, 1.0, path, pto, pto_types, &path_types);

  ASSERT_EQ(path.size(), 51u);   // 10 m / 0.2 m -> 50 段
  ASSERT_EQ(pto.size(), 11u);    // 10 m / 1.0 m -> 10 段
  for (double step : Steps(path)) {
    EXPECT_NEAR(step, 0.2, 1e-9);
  }
  for (double step : Steps(pto)) {
    EXPECT_NEAR(step, 1.0, 1e-9);
  }
}

// ⑤ 只有段类型 (没有点类型) 也要能用: 逐点类型退化成"该点的出段类型", 不崩、不错位。
TEST(ExternalSkeleton, SegmentTypesOnlyIsEnough) {
  const BowSkeleton sk = SkeletonOf(Line({0.0, 0.0}, {20.0, 0.0}, 3), /*types=*/{}, {kSegTurn, kSegWork});
  std::vector<Point_2> path;
  std::vector<uint8_t> path_types;
  std::vector<Point_2> pto;
  std::vector<uint8_t> pto_types;

  resampleBowSkeleton(sk, 5.0, 10.0, path, pto, pto_types, &path_types);

  ASSERT_EQ(path_types.size(), path.size());
  EXPECT_EQ(path_types[0], kSegTurn);   // 首点: 没有点类型 -> 用"出段"类型 A->B = 掉头
  EXPECT_EQ(path_types[2], kSegWork);   // B 点: 出段 B->C 是作业段
  EXPECT_EQ(path_types[4], kSegWork);   // 末点: 没有出段 -> 用最后一段 (作业)
  EXPECT_EQ(path_types[1], kSegTurn);   // 段 A->B 内插出来的点: 按**段**类型 -> 掉头
}

// ⑥ 退化: 段类型长度不对 (不是 n-1) 时忽略它, 回落到点类型规则; 连点类型也没有 -> 不产出 PTO
// (宁可不发, 也不发一份错位的标签)。
TEST(ExternalSkeleton, BrokenSegTypesFallBackAndMissingLabelsSkipPto) {
  const auto points = Line({0.0, 0.0}, {20.0, 0.0}, 3);

  {
    // 段类型只给了 1 个 (应为 2) -> 忽略, 按点类型规则: 段 A->B / B->C 都碰掉头点 -> 都是掉头段。
    const BowSkeleton sk = SkeletonOf(points, {kSegWork, kSegTurn, kSegWork}, {kSegTurn});
    std::vector<Point_2> path, pto;
    std::vector<uint8_t> path_types, pto_types;
    resampleBowSkeleton(sk, 5.0, 10.0, path, pto, pto_types, &path_types);
    ASSERT_EQ(path_types.size(), path.size());
    EXPECT_EQ(path_types[1], kSegTurn);
    EXPECT_EQ(path_types[3], kSegTurn);
  }
  {
    // 什么标签都没有 -> /path 照常采 (类型一律 WORK), PTO 不产出。
    const BowSkeleton sk = SkeletonOf(points, {}, {});
    std::vector<Point_2> path, pto;
    std::vector<uint8_t> path_types, pto_types;
    resampleBowSkeleton(sk, 5.0, 10.0, path, pto, pto_types, &path_types);
    EXPECT_EQ(path.size(), 5u);
    for (uint8_t t : path_types) {
      EXPECT_EQ(t, kSegWork);
    }
    EXPECT_TRUE(pto.empty());
    EXPECT_TRUE(pto_types.empty());
  }
}
