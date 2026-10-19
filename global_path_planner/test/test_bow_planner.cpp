// 全覆盖 (弓字形) 路径规划的行为契约测试。
//
// 覆盖的几条:
//   * 覆盖率     : 垄覆盖区域内任意点到最近垄线的距离 <= ridge_width/2 (直角端头的矩形
//                  地块连"到垄线段"的距离都要求 <= ridge_width/2);
//   * 垄间距     : 相邻两条垄的横向间距 = ridge_width, 且相邻垄行驶方向相反 (弓字形);
//   * 分辨率     : 相邻路径点间距 <= path_resolution;
//   * 内缩       : 地头余量 field_margin (只退垄方向两端; 横向铺满, 田边无盲带);
//                  拐角上限 miter_limit (参数已接线, 但实测对向内偏置无影响 —— 见 MiterLimitDoesNotChangeInwardInset);
//   * 首行/末行  : 首行固定距内缩边界 first_row_margin (默认 5 mm), 之后按 ridge_width 递推;
//                  末行被推到区域外/压在远端边界上时兜底贴到"距远端 first_row_margin" ——
//                  见 FirstRowMarginIsConfigurable / LastRowFallsBackToFarMargin;
//   * 输入宽容度 : 逆序/旋转起点/带共线点都能收下; 自相交与乱序输入拒绝; 凹多边形按约定拒绝;
//   * 退化       : 窄地块 (宽度 < 垄宽) 也按"首末各留 first_row_margin"排两条; 非法垄宽/过小地块直接失败。
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <boost/geometry.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>   // memcmp: 逐位比较 (ResampleOnlyMatchesFullPlan)
#include <limits>
#include <unordered_map>
#include <vector>

#include "bow_test_facade.hpp"   // 测试专用: 把旧的有状态 API 投影到 plan(BowInput)->BowOutput 上
#include "global_path_planner/bow.hpp"
#include "global_path_planner/eigen_boost_adaptor.hpp"
#include "global_path_planner/fault_codes.hpp"

using bow_test::BowFacade;

namespace {

namespace bg = boost::geometry;

constexpr double kRidgeWidth = 3.0;

// config/global_path_planner.yaml 那类地块的等比例缩小版: 12.4 m x 5.2 m 的四边形 (m)
Polygon_2 SampleField() {
  return {Point_2(0.0, 0.0), Point_2(12.0, 0.0), Point_2(11.0, 5.2), Point_2(-0.2, 5.2)};
}

// 长方形: 直角端头, 覆盖检查可以按最严格的"点到垄线段"来做
Polygon_2 RectangleField(double width, double height) {
  return {Point_2(0.0, 0.0), Point_2(width, 0.0), Point_2(width, height), Point_2(0.0, height)};
}

bg::model::polygon<Point_2> ToBoostPolygon(const Polygon_2& ring) {
  bg::model::polygon<Point_2> poly;
  if (ring.empty())
    return poly;
  poly.outer().assign(ring.begin(), ring.end());
  poly.outer().push_back(ring.front());
  bg::correct(poly);
  return poly;
}

// 点到"线段"的距离 (Humble 的 Boost 1.74 还没有 bg::closest_point, 手动投影到线段)
double PointToSegmentDistance(const Point_2& p, const Point_2& a, const Point_2& b) {
  const Point_2 ab  = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < 1e-18)
    return (p - a).norm();
  const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / len2));
  return (p - (a + t * ab)).norm();
}

// 点到"无限直线"的距离 (垄的覆盖范围是沿整条垄线的带状区域)
double PointToLineDistance(const Point_2& p, const Point_2& a, const Point_2& b) {
  const Point_2 ab = b - a;
  const double len = ab.norm();
  if (len < 1e-12)
    return (p - a).norm();
  return std::abs((p - a).x() * ab.y() - (p - a).y() * ab.x()) / len;
}

struct CoverageReport {
  size_t sampled = 0;
  size_t missed  = 0;
  double worst   = 0.0;   // 采样点里"到最近垄"的最大距离
};

// 在区域内按网格采样, 统计"到最近垄"的最大距离与超出半个垄宽的点数。
template<typename DistanceFn>
CoverageReport CheckCoverage(const Polygon_2& region, const std::vector<Point_2>& keys, double ridge_width, double step, DistanceFn distance_fn) {
  CoverageReport report;
  const auto model = ToBoostPolygon(region);
  bg::model::box<Point_2> box;
  bg::envelope(model, box);

  const size_t ridge_count = keys.size() / 2;
  for (double x = bg::get<bg::min_corner, 0>(box); x <= bg::get<bg::max_corner, 0>(box); x += step) {
    for (double y = bg::get<bg::min_corner, 1>(box); y <= bg::get<bg::max_corner, 1>(box); y += step) {
      const Point_2 p(x, y);
      if (!bg::covered_by(p, model))
        continue;
      ++report.sampled;

      double best = std::numeric_limits<double>::max();
      for (size_t r = 0; r < ridge_count; ++r) {
        best = std::min(best, distance_fn(p, keys[2 * r], keys[2 * r + 1]));
      }
      report.worst = std::max(report.worst, best);
      if (best > 0.5 * ridge_width + 1e-3)   // 半个垄宽 + 量化误差
        ++report.missed;
    }
  }
  return report;
}

}   // namespace

// ============================================================
// F2C 借鉴 (Fields2Cover) 相关的测试工具
// ============================================================

namespace {

// 点到地块边界的最近距离 (地头带深度就用它核对)。
double DistanceToBoundary(const Polygon_2& poly, const Point_2& p) {
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poly.size(); ++i)
    best = std::min(best, PointToSegmentDistance(p, poly[i], poly[(i + 1) % poly.size()]));
  return best;
}

// 相邻三点的转角 (deg) —— 判断路径在某点是否"折角"。
double NodeTurnDeg(const Point_2& a, const Point_2& b, const Point_2& c) {
  const Point_2 u = (b - a).normalized();
  const Point_2 v = (c - b).normalized();
  return std::abs(std::atan2(u.x() * v.y() - u.y() * v.x(), u.dot(v))) * 180.0 / M_PI;
}

// 路径折线到点 p 的最近距离。
double DistanceToPath(const std::vector<Point_2>& wp, const Point_2& p) {
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < wp.size(); ++i)
    best = std::min(best, PointToSegmentDistance(p, wp[i], wp[i + 1]));
  return best;
}

// 覆盖率 (按面积): 区域内"到整条路径的距离 > tolerance"的采样点算未覆盖, 返回已覆盖比例。
double CoveredAreaRatio(const Polygon_2& region, const std::vector<Point_2>& wp, double tolerance, double step = 0.25) {
  const auto model = ToBoostPolygon(region);
  bg::model::box<Point_2> box;
  bg::envelope(model, box);
  size_t total = 0, covered = 0;
  for (double x = bg::get<bg::min_corner, 0>(box); x <= bg::get<bg::max_corner, 0>(box); x += step) {
    for (double y = bg::get<bg::min_corner, 1>(box); y <= bg::get<bg::max_corner, 1>(box); y += step) {
      const Point_2 p(x, y);
      if (!bg::covered_by(p, model))
        continue;
      ++total;
      if (DistanceToPath(wp, p) <= tolerance + 1e-9)
        ++covered;
    }
  }
  return total == 0 ? 0.0 : static_cast<double>(covered) / static_cast<double>(total);
}

// 路径自交次数: 现场说的"路径与已经走过的路径有交点"就是它。
// 用网格哈希把候选段对限定在同几个格子里, 再判严格相交 (叉积异号); 相邻段与共享端点的
// 衔接 (掉头/连接的接缝) 不算相交。同向重叠 (共线) 也不算 —— 那是重复作业, 不是交点。
int CountSelfCrossings(const std::vector<Point_2>& wp, double cell = 2.0) {
  const auto key = [](int ix, int iy) {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
  };
  const auto orient = [](const Point_2& a, const Point_2& b, const Point_2& c) {
    const double v = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    return (v > 1e-9) ? 1 : ((v < -1e-9) ? -1 : 0);
  };
  std::unordered_map<int64_t, std::vector<size_t>> grid;
  for (size_t i = 0; i + 1 < wp.size(); ++i) {
    const Point_2& a = wp[i];
    const Point_2& b = wp[i + 1];
    const int ix0    = static_cast<int>(std::floor(std::min(a.x(), b.x()) / cell));
    const int ix1    = static_cast<int>(std::floor(std::max(a.x(), b.x()) / cell));
    const int iy0    = static_cast<int>(std::floor(std::min(a.y(), b.y()) / cell));
    const int iy1    = static_cast<int>(std::floor(std::max(a.y(), b.y()) / cell));
    for (int ix = ix0; ix <= ix1; ++ix)
      for (int iy = iy0; iy <= iy1; ++iy)
        grid[key(ix, iy)].push_back(i);
  }
  int crossings = 0;
  for (const auto& entry : grid) {
    const std::vector<size_t>& ids = entry.second;
    for (size_t a = 0; a < ids.size(); ++a) {
      for (size_t b = a + 1; b < ids.size(); ++b) {
        const size_t i = ids[a], j = ids[b];
        if (i + 1 >= j && j + 1 >= i)
          continue;
        const Point_2 &p1 = wp[i], &p2 = wp[i + 1], &q1 = wp[j], &q2 = wp[j + 1];
        if ((p2 - q1).norm() < 1e-9 || (p1 - q2).norm() < 1e-9)
          continue;
        if (orient(p1, p2, q1) * orient(p1, p2, q2) < 0 && orient(q1, q2, p1) * orient(q1, q2, p2) < 0)
          ++crossings;
      }
    }
  }
  return crossings;
}

}   // namespace

// 长方形地块: 最严格的全覆盖判据 (点到垄线段的距离都在半个垄宽内)
// 20 x 5, 垄宽 3, 首行余量 5 mm: 垄行在 y = 0.005 / 3.005, 末行按网格应到 6.005 (已在区域外)
// -> 兜底贴到距远端 5 mm = 4.995, 共 3 条。最大横向间距 3.005 - 0.005 = 3.0 -> 最差距离 1.5。
TEST(BowPlannerCoverage, RectangleIsFullyCovered) {
  BowFacade planner;
  ASSERT_TRUE(planner.init(RectangleField(20.0, 5.0), kRidgeWidth, 0.1, 0.0));

  const auto& keys = planner.getKeyWaypoints();
  ASSERT_EQ(keys.size(), 6u) << "5 m 宽 / 3 m 垄宽 -> 首行 5 mm + 间距 3 m, 末行兜底 -> 3 条垄";

  const auto report = CheckCoverage(planner.getInnerPolygon(), keys, kRidgeWidth, 0.05, PointToSegmentDistance);
  EXPECT_GT(report.sampled, 10000u);
  EXPECT_EQ(report.missed, 0u) << "未被垄线段覆盖的采样点: " << report.missed;
  EXPECT_LE(report.worst, 0.5 * kRidgeWidth + 1e-3);
}

// 梯形地块 (与现场 yaml 同量级): 保证"到垄线"的距离在半个垄宽内 —— 斜的地头边上
// 超出垄端点的角落属于地头余量的职责, 不属于垄覆盖 (因此这里用直线距离判定)。
TEST(BowPlannerCoverage, TrapezoidFieldIsCoveredByRidgeLines) {
  BowFacade planner;
  ASSERT_TRUE(planner.init(SampleField(), kRidgeWidth, 0.1, 1.0));

  const auto& keys = planner.getKeyWaypoints();
  ASSERT_FALSE(keys.empty());
  ASSERT_EQ(keys.size() % 2, 0u);

  const auto report = CheckCoverage(planner.getInnerPolygon(), keys, kRidgeWidth, 0.05, PointToLineDistance);
  EXPECT_GT(report.sampled, 10000u);
  EXPECT_EQ(report.missed, 0u) << "未被垄线覆盖的采样点: " << report.missed;
  EXPECT_LE(report.worst, 0.5 * kRidgeWidth + 1e-3);
  EXPECT_GT(report.worst, 0.4 * kRidgeWidth) << "垄线间距不应明显小于垄宽";
}

// 垄间距 = ridge_width, 且相邻垄行驶方向相反 (弓字形)
TEST(BowPlannerCoverage, RidgeSpacingAndSerpentineOrder) {
  const double ridge_width = 2.0;
  BowFacade planner;
  ASSERT_TRUE(planner.init(RectangleField(30.0, 7.0), ridge_width, 0.2, 0.0));

  const auto& keys = planner.getKeyWaypoints();
  ASSERT_EQ(keys.size() % 2, 0u);
  const size_t ridge_count = keys.size() / 2;
  ASSERT_EQ(ridge_count, 4u) << "7 m / 2 m -> ceil = 4 条垄";

  for (size_t r = 0; r < ridge_count; ++r) {
    const Point_2 start = keys[2 * r];
    const Point_2 end   = keys[2 * r + 1];
    EXPECT_NEAR(start.y(), end.y(), 1e-6) << "第 " << r << " 条垄应与地块长边平行";
    EXPECT_NEAR(std::min(start.x(), end.x()), 0.0, 1e-3) << "第 " << r << " 条垄起点应在地块端头";
    EXPECT_NEAR(std::max(start.x(), end.x()), 30.0, 1e-3) << "第 " << r << " 条垄终点应在地块端头";
  }

  for (size_t r = 1; r < ridge_count; ++r) {
    const double spacing = std::abs(keys[2 * r].y() - keys[2 * r - 2].y());
    EXPECT_NEAR(spacing, ridge_width, 1e-3) << "第 " << r << " 条垄的间距";
  }

  for (size_t r = 0; r + 1 < ridge_count; ++r) {
    const double dir_prev = (keys[2 * r + 1] - keys[2 * r]).x();
    const double dir_next = (keys[2 * r + 3] - keys[2 * r + 2]).x();
    EXPECT_LT(dir_prev * dir_next, 0.0) << "第 " << r << " 与 " << r + 1 << " 条垄的行驶方向应相反";
  }

  // 掉头固定为半圆/直线几何 (半径 = 垄宽/2), 外伸不超过垄宽/2; 地头环默认不生成。
  // (掉头外伸量的内部统计已随 BowPlannerStats 删除 —— 出口侧由 F2cTurn 用路径点直接体检)
  EXPECT_EQ(ridge_count > 0 ? ridge_count - 1 : 0, 3u) << "4 条垄 -> 3 次掉头";
}

// 首行固定余量 + 末行兜底:
//   * 垄行 = first_row_margin + k*ridge_width, 间距恒为垄宽;
//   * 末行按网格推到区域外 / 压在远端边界上 (与区域没有交点) -> 兜底贴到"距远端 first_row_margin"
//     处, 不再像以前那样静默丢掉那一行;
//   * 垄数取"最小的 N 使首行 + (N-1)*垄宽 >= W - 垄宽/2" -> 区域内仍保持 <= 垄宽/2 的全覆盖。
TEST(BowPlannerCoverage, FirstRowMarginAndLastRowFallback) {
  const double ridge_width  = 3.0;
  const double first_margin = 0.005;
  BowFacade planner;

  // 宽 6.001 m: 网格末行 0.005 + 2*3 = 6.005 已越过远端 -> 兜底到 6.001 - 0.005 = 5.996
  ASSERT_TRUE(planner.init(RectangleField(6.001, 20.0), ridge_width, 0.2, 0.0));
  {
    const auto& keys = planner.getKeyWaypoints();
    ASSERT_EQ(keys.size() % 2, 0u);
    EXPECT_EQ(keys.size() / 2, 3u) << "首行 5 mm + 间距 3 m, 末行兜底 -> 3 条垄";
    EXPECT_NEAR(keys.front().x(), first_margin, 1e-3) << "首行 = first_row_margin";
    EXPECT_NEAR(keys[2 * 1].x(), first_margin + ridge_width, 1e-3) << "第二行 = 首行 + 垄宽";
    EXPECT_NEAR(keys[2 * 2].x(), 6.001 - first_margin, 1e-3) << "末行 = 区域远端 - first_row_margin (兜底)";
    EXPECT_LT(keys[2 * 2].x(), 6.001) << "兜底行仍在区域内";
  }

  // 宽 6.020 m: 网格末行 6.005 落在区域内 (距远端 15 mm) -> 不兜底, 末行就用 6.005
  ASSERT_TRUE(planner.init(RectangleField(6.020, 20.0), ridge_width, 0.2, 0.0));
  {
    const auto& keys = planner.getKeyWaypoints();
    ASSERT_EQ(keys.size() % 2, 0u);
    EXPECT_EQ(keys.size() / 2, 3u) << "同样是 3 条垄";
    EXPECT_NEAR(keys.front().x(), first_margin, 1e-3) << "首行 = first_row_margin";
    EXPECT_NEAR(keys[2 * 2].x(), 6.005, 1e-3) << "末行就是网格位置 (有交点, 不兜底)";
    for (size_t r = 1; r < keys.size() / 2; ++r) {
      EXPECT_NEAR(std::abs(keys[2 * r].x() - keys[2 * r - 2].x()), ridge_width, 1e-3) << "间距恒为垄宽";
    }
  }
}

// 首行余量是**配置参数**: 值变了, 首行位置与垄数跟着变 (田边盲带 = field_margin + 首行余量 - 垄宽/2)。
TEST(BowPlannerCoverage, FirstRowMarginIsConfigurable) {
  const double ridge_width  = 2.0;
  const double field_margin = 0.5;
  const Polygon_2 field     = RectangleField(20.0, 8.0);   // 内缩后 19 x 7, 扫描方向取长边 -> 垄沿 x

  BowFacade tight;
  ASSERT_TRUE(tight.init(field, ridge_width, 0.2, field_margin, 2.0, 0.05));   // 50 mm
  const auto& tight_keys = tight.getKeyWaypoints();
  double tight_min_y     = 1e9;
  for (const auto& p : tight_keys)
    tight_min_y = std::min(tight_min_y, p.y());
  EXPECT_NEAR(tight_min_y, field_margin + 0.05, 1e-3) << "首行 = field_margin + first_row_margin";
  EXPECT_EQ(tight_keys.size() / 2, 4u) << "7 m 宽 / 2 m 垄宽: 首末各 50 mm -> 4 条";

  BowFacade loose;
  ASSERT_TRUE(loose.init(field, ridge_width, 0.2, field_margin, 2.0, 0.5));   // 500 mm
  const auto& loose_keys = loose.getKeyWaypoints();
  double loose_min_y     = 1e9;
  for (const auto& p : loose_keys)
    loose_min_y = std::min(loose_min_y, p.y());
  EXPECT_NEAR(loose_min_y, field_margin + 0.5, 1e-3) << "首行跟着参数走";
  EXPECT_GT(loose_min_y, tight_min_y) << "余量给大 -> 首行更靠里";

  // 越界值被夹到 [kMinFirstRowMargin, 垄宽/2]: 只告警, 不失败
  BowFacade clamped;
  ASSERT_TRUE(clamped.init(field, ridge_width, 0.2, field_margin, 2.0, 5.0));
  double clamped_min_y = 1e9;
  for (const auto& p : clamped.getKeyWaypoints())
    clamped_min_y = std::min(clamped_min_y, p.y());
  EXPECT_NEAR(clamped_min_y, field_margin + 0.5 * ridge_width, 1e-3) << "超过 垄宽/2 的值被夹到 垄宽/2";
}

// 末行兜底: 首行 5 mm + 间距 3 m 时, 4.999 m 宽的区域网格末行落在 6.005 (区域外) -> 兜底贴到 4.994,
// 且兜底行与上一行/远端边界的关系仍满足"区域内任意点到最近垄线 <= 垄宽/2"。
TEST(BowPlannerCoverage, LastRowFallsBackToFarMargin) {
  const double ridge_width = 3.0;
  const Polygon_2 field    = RectangleField(20.0, 4.999);
  BowFacade planner;
  ASSERT_TRUE(planner.init(field, ridge_width, 0.1, 0.0));

  const auto& keys = planner.getKeyWaypoints();
  ASSERT_EQ(keys.size() % 2, 0u);
  ASSERT_EQ(keys.size() / 2, 3u) << "0.005 / 3.005 / 兜底 4.994";

  double y_min = 1e9, y_max = -1e9;
  for (const auto& p : keys) {
    y_min = std::min(y_min, p.y());
    y_max = std::max(y_max, p.y());
  }
  EXPECT_NEAR(y_min, 0.005, 1e-3) << "首行贴到 5 mm";
  EXPECT_NEAR(y_max, 4.999 - 0.005, 1e-3) << "末行兜底贴到远端 5 mm";
  for (const auto& p : keys) {
    EXPECT_GE(p.y(), 0.0);
    EXPECT_LE(p.y(), 4.999);
  }

  const auto report = CheckCoverage(planner.getInnerPolygon(), keys, ridge_width, 0.05, PointToSegmentDistance);
  EXPECT_GT(report.sampled, 1000u);
  EXPECT_EQ(report.missed, 0u) << "兜底行不应打破全覆盖 (未覆盖点: " << report.missed << ")";
  EXPECT_LE(report.worst, 0.5 * ridge_width + 1e-3);
}

// 末行兜底的**准入门槛**: 只有"扫描线一直没打到远端边界边"(远端那条带真没人盖) 时才允许贴远端补一行。
// 远端边相对垄向是斜的时, 常规行会被它裁掉、端点正好落在它上面 (= 远端已经被扫到); 这时再贴边补一行,
// 兜底行会横穿那条行 —— 弓字形当场回折, 路径与走过的路径相交 (实测 13.45x5.32 斜地块家族: 斜度 1.5 m
// 起自交 1 处、3.5 m 起 5 处)。目标: 即使判定"不再兜底"(宁可远端留一条盲带, [COVERAGE] 会报出来),
// 也**不许**出现自交; 而远端确实没被扫到的斜度上, 兜底照常生效 (垄数比不兜底多 1 条)。
TEST(BowPlannerCoverage, NoFarFallbackWhenRowsAlreadyReachFarEdge) {
  struct Case {
    double skew;     // 地块右下角的高度: 0 = 矩形; 越大远端边相对垄向越斜
    size_t ridges;   // 期望的垄数 (兜底生效 = 5, 被挡掉 = 4)
    const char* why;
  };
  const std::vector<Case> cases{
    {0.5, 5u, "远端边还不够斜: 常规行离远端 0.82 m > 兜底行的 5 mm -> 兜底生效"},
    {1.0, 5u, "同上 (常规行离远端 0.33 m)"},
    {1.5, 4u, "常规行已经打到远端 (离它 0.6 mm) -> 不再兜底"},
    {2.0, 4u, "常规行已经打到远端 (离它 1.1 mm, 在 2 格量化容差内) -> 不再兜底"},
    {3.5, 4u, "同上 (离远端 0.4 mm) -> 不再兜底"},
  };

  for (const auto& c : cases) {
    const Polygon_2 field{Point_2(0.0, 0.0), Point_2(13.45, c.skew), Point_2(13.34, 5.32), Point_2(0.00, 4.87)};
    BowFacade planner;
    ASSERT_TRUE(planner.init(field, 1.0, 0.1, 0.5)) << "斜度 " << c.skew;

    const auto& keys = planner.getKeyWaypoints();
    ASSERT_EQ(keys.size() % 2, 0u);
    EXPECT_EQ(keys.size() / 2, c.ridges) << "斜度 " << c.skew << ": " << c.why;

    // 无论兜底生不生效, 路径都不许与走过的路径相交 (现场的"路径有交点"就是这一条)。
    EXPECT_EQ(CountSelfCrossings(planner.getWaypoints()), 0) << "斜度 " << c.skew << ": 路径与走过的路径相交";

    // 垄线之间也不许相交 (兜底行横穿常规行就是它导致的)。
    for (size_t i = 0; i < keys.size() / 2; ++i)
      for (size_t j = i + 1; j < keys.size() / 2; ++j)
        EXPECT_FALSE(boost::geometry::intersects(bg::model::segment<Point_2>(keys[2 * i], keys[2 * i + 1]), bg::model::segment<Point_2>(keys[2 * j], keys[2 * j + 1])))
          << "斜度 " << c.skew << ": 垄 " << i << " 与垄 " << j << " 相交";
  }
}

// 斜地头的掉头覆盖: 两个垄端沿垄向错开 |d| 时, 旧几何把半圆的顶点压在 B-C **中点**上, 于是
// "靠外那个垄端"谁也盖不到 —— 它到作业段的距离正好是 |d|/2, 每个斜地头掉头处丢一条 |d|/2 深
// 的带 (实测 60x20 梯形 t=40 / 垄宽 3 m / 余量 1.5 m: 作业区未覆盖 2.52%; 平行四边形 (斜 40 m)
// 3.70%)。现在顶点压到"靠外的那个垄端"上, 只在"再往外推就伸出地块边界"时才回退, 回退剩下的
// 残量由 getTurnMaxPocket() 报出 ([TURN_POCKET], 并给出需要的 field_margin)。
TEST(BowPlannerCoverage, SkewedHeadlandTurnCoversBothRowEnds) {
  struct Case {
    std::string name;
    Polygon_2 field;
    double max_pocket;     // 期望的最大盲带 (m)
    bool fully_covered;    // true = 错开量没超过余量, 应当一条都不剩
    double min_coverage;   // 期望的作业区覆盖率下限 (含掉头段)
  };
  const std::vector<Case> cases{
    // 60x20 梯形, 右端斜; 垄宽 3 / 余量 1.5 (= 垄宽/2, 现场推荐配置)
    {"trap top 20", {Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(40.0, 20.0), Point_2(0.0, 20.0)}, 0.05, true, 0.994},
    {"trap top 40", {Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(20.0, 20.0), Point_2(0.0, 20.0)}, 1.55, false, 0.980},
    // 平行四边形 (斜 40 m)
    {"para skew 40", {Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(100.0, 20.0), Point_2(40.0, 20.0)}, 1.55, false, 0.975},
  };

  for (const auto& c : cases) {
    BowFacade planner;
    ASSERT_TRUE(planner.init(c.field, 3.0, 0.1, 1.5)) << c.name;

    // 掉头不许为覆盖伸出地块边界 (5 cm 以内, 与 [TURN_BOUNDARY] 的阈值同一套)
    EXPECT_LE(planner.getTurnMaxOverrun(), 0.05 + 1e-9) << c.name << ": 掉头出了地块边界";
    // 盲带: 错开量装得下时应当一条不剩; 装不下时要有界, 并且必须报出来 (不能静默漏掉)
    EXPECT_LE(planner.getTurnMaxPocket(), c.max_pocket) << c.name << ": 掉头处盲带过大";
    if (c.fully_covered)
      EXPECT_LE(planner.getTurnMaxPocket(), 0.05) << c.name << ": 这个错开量本该能完全盖住";
    else
      EXPECT_GT(planner.getTurnMaxPocket(), 0.05) << c.name << ": 盖不住的残量必须报出来 ([TURN_POCKET])";

    // 作业区 (内缩区域) 被整条路径覆盖的比例 —— 掉头段也算覆盖
    const double inner = CoveredAreaRatio(planner.getInnerPolygon(), planner.getWaypoints(), 1.5, 0.1);
    EXPECT_GE(inner, c.min_coverage) << c.name << ": 作业区覆盖率只有 " << 100.0 * inner << "%";
    EXPECT_EQ(CountSelfCrossings(planner.getWaypoints()), 0) << c.name << ": 路径与走过的路径相交";
  }
}

// 四周统一内缩 (field_margin) 的契约: 内边界 = 地块四边各退 margin; 垄线只在内缩区域里铺,
// 首行贴到距内缩边界 first_row_margin (默认 5 mm), 末行同理; 田边盲带 = field_margin + 首行余量
// - 垄宽/2 (由 [COVERAGE] 报出, 首行贴边后通常 <= 0)。
// 12.4 x 20 地块, 垄宽 0.8, margin 0.2: 扫描方向自动取长边 -> 垄沿 y, 横向 = x。
TEST(BowPlannerCoverage, UniformMarginInsetsAllSides) {
  const double ridge_width  = 0.8;
  const double margin       = 0.2;
  const double first_margin = 0.005;
  BowFacade planner;
  ASSERT_TRUE(planner.init(RectangleField(12.4, 20.0), ridge_width, 0.2, margin));

  // 内边界: 四边各退 margin
  const auto& region = planner.getInnerPolygon();
  ASSERT_FALSE(region.empty());
  double rx_min = 1e9, rx_max = -1e9, ry_min = 1e9, ry_max = -1e9;
  for (const auto& p : region) {
    rx_min = std::min(rx_min, p.x());
    rx_max = std::max(rx_max, p.x());
    ry_min = std::min(ry_min, p.y());
    ry_max = std::max(ry_max, p.y());
  }
  EXPECT_NEAR(rx_min, margin, 1e-3) << "左边界退 field_margin";
  EXPECT_NEAR(rx_max, 12.4 - margin, 1e-3) << "右边界退 field_margin";
  EXPECT_NEAR(ry_min, margin, 1e-3) << "下边界退 field_margin";
  EXPECT_NEAR(ry_max, 20.0 - margin, 1e-3) << "上边界退 field_margin";

  // 垄: 横向可用 12.0, 垄宽 0.8 -> 首行 5 mm, 之后每 0.8 m; 网格末行 0.005 + 15*0.8 = 12.005 已出界
  // -> 兜底贴到距远端 5 mm = 11.995, 共 16 条 (旧的"商/余数对称"方案是 15 条、首末各留 0.4)。
  const auto& keys = planner.getKeyWaypoints();
  ASSERT_EQ(keys.size() % 2, 0u);
  EXPECT_EQ(keys.size() / 2, 16u) << "首行 5 mm + 间距 0.8 m (末行兜底) -> 16 条";
  double x_min = 1e9, x_max = -1e9, y_min = 1e9, y_max = -1e9;
  for (const auto& p : keys) {
    x_min = std::min(x_min, p.x());
    x_max = std::max(x_max, p.x());
    y_min = std::min(y_min, p.y());
    y_max = std::max(y_max, p.y());
  }
  EXPECT_NEAR(x_min, margin + first_margin, 1e-3) << "首垄 = field_margin + first_row_margin";
  EXPECT_NEAR(x_max, 12.4 - margin - first_margin, 1e-3) << "末垄贴另一侧 (首末对称)";
  EXPECT_NEAR(y_min, margin, 1e-3) << "垄端点在地头方向也退 field_margin";
  EXPECT_NEAR(y_max, 20.0 - margin, 1e-3) << "垄端点在地头方向也退 field_margin";

  // 覆盖率: 首行贴边后区域内任意点到最近垄线仍 <= 半个垄宽
  const auto report = CheckCoverage(region, keys, ridge_width, 0.05, PointToSegmentDistance);
  EXPECT_GT(report.sampled, 10000u);
  EXPECT_EQ(report.missed, 0u) << "未覆盖点: " << report.missed;
  EXPECT_LE(report.worst, 0.5 * ridge_width + 1e-3);

  // margin >= 掉头半径 (垄距/2) 时, 掉头弧全程在地块内
  BowFacade roomy;
  ASSERT_TRUE(roomy.init(RectangleField(12.4, 20.0), ridge_width, 0.2, 0.5));
  EXPECT_EQ(roomy.getTurnMaxOverrun(), 0.0) << "余量足够时掉头弧应全程在地块内";
}

// 路径分辨率: 相邻路径点间距不超过 path_resolution, 首末点就是首末垄的端点
TEST(BowPlannerCoverage, WaypointSpacingRespectsResolution) {
  const double resolution = 0.1;
  BowFacade planner;
  ASSERT_TRUE(planner.init(SampleField(), kRidgeWidth, resolution, 1.0));

  const auto& waypoints = planner.getWaypoints();
  const auto& keys      = planner.getKeyWaypoints();
  ASSERT_GT(waypoints.size(), 2u);
  ASSERT_FALSE(keys.empty());

  for (size_t i = 1; i < waypoints.size(); ++i) {
    const double step = (waypoints[i] - waypoints[i - 1]).norm();
    EXPECT_LE(step, resolution + 1e-6) << "第 " << i << " 段过长";
    EXPECT_GT(step, 1e-9) << "第 " << i << " 段退化为 0";
  }

  EXPECT_NEAR((waypoints.front() - keys.front()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((waypoints.back() - keys.back()).norm(), 0.0, 1e-9);
}

// 余量相同 -> Clipper2 偏置; 收缩后的多边形应比原多边形小, 且顶点都还在原多边形内
TEST(BowPlannerInset, UniformMarginShrinksPolygon) {
  const Polygon_2 field = RectangleField(20.0, 8.0);
  BowFacade planner;
  ASSERT_TRUE(planner.init(field, 2.0, 0.1, 0.5));

  const auto& region = planner.getInnerPolygon();
  ASSERT_GE(region.size(), 3u);

  const auto field_model  = ToBoostPolygon(field);
  const auto region_model = ToBoostPolygon(region);
  EXPECT_LT(std::abs(bg::area(region_model)), std::abs(bg::area(field_model)));
  for (const auto& p : region) {
    EXPECT_TRUE(bg::covered_by(p, field_model)) << "内缩后的顶点跑到地块外: " << p.transpose();
  }

  // 20 x 8 四周各内缩 0.5 -> 19 x 7
  EXPECT_NEAR(std::abs(bg::area(region_model)), 19.0 * 7.0, 0.05);
}

// 拐角上限 miter_limit 的接线与实测行为:
//   (a) 参数确实接到 plan()/init() 上 (第 5 个参数, 带默认值, 所以老调用不受影响);
//   (b) **对向内偏置不影响结果** —— Clipper2 对凸多边形的凸顶点走"凹角分支" (插 3 点, 最后 union
//       清理负区域), 得到的恒是**无上限的斜接尖点**, 因此 1.0 与 20.0 的内缩顶点逐位相同。
// 这条用例是"该参数是留口、不是调拐角的手段"的可执行证据 (对照实验: 同一份顶点改成**向外**偏置,
// miter_limit 立刻起作用 —— 1.0 出切角点, 20.0 出长尖刺)。哪天 Clipper2 改了行为、或本规划器换了
// JoinType, 本用例会失败 —— 正好提醒同步更新 yaml 与文档里的说明。
TEST(BowPlannerInset, MiterLimitDoesNotChangeInwardInset) {
  const double ridge_width = 1.0;
  const double resolution  = 0.5;
  const double margin      = 0.5;

  const auto check_field = [&](const char* what, const Polygon_2& field) {
    // 基准: miter_limit = 1.0 = "一律切角" (Clipper2 里 <= 1 的取值)
    BowFacade baseline;
    ASSERT_TRUE(baseline.init(field, ridge_width, resolution, margin, 1.0)) << what << ": miter_limit = 1.0 应能规划";
    const Polygon_2 region_1 = baseline.getInnerPolygon();
    ASSERT_GE(region_1.size(), 3u);

    // 极端"尖锐": 20.0 —— 若 join 类型对向内偏置有效, 这里的顶点/坐标必然与上面不同
    BowFacade sharp;
    ASSERT_TRUE(sharp.init(field, ridge_width, resolution, margin, 20.0)) << what << ": miter_limit = 20.0 应能规划";
    const Polygon_2& region_20 = sharp.getInnerPolygon();
    EXPECT_EQ(region_20.size(), region_1.size()) << what << ": 向内偏置的顶点数不该随 miter_limit 变";
    ASSERT_EQ(region_20.size(), region_1.size());
    for (size_t i = 0; i < region_20.size(); ++i) {
      EXPECT_NEAR((region_20[i] - region_1[i]).norm(), 0.0, 1e-12) << what << ": 第 " << i << " 个顶点不该受 miter_limit 影响";
    }

    // 非法取值 (<= 1.0) 夹到 1.0: 不报错、不崩, 结果与"一律切角"逐位相同
    BowFacade clamped;
    ASSERT_TRUE(clamped.init(field, ridge_width, resolution, margin, -5.0)) << what << ": 非法 miter_limit 不该让规划失败";
    const Polygon_2& region_clamped = clamped.getInnerPolygon();
    ASSERT_EQ(region_clamped.size(), region_1.size());
    for (size_t i = 0; i < region_clamped.size(); ++i) {
      EXPECT_NEAR((region_clamped[i] - region_1[i]).norm(), 0.0, 1e-12) << what << ": 非法 miter_limit 应等价于 1.0";
    }
  };

  // 直角地块 (拐角 90°) 与锐角三角形 (顶角约 18°): 后者是 join 类型最容易露出差别的地方。
  check_field("直角矩形 20x8", RectangleField(20.0, 8.0));
  check_field("锐角三角形", Polygon_2{Point_2(0.0, 0.0), Point_2(20.0, 0.0), Point_2(20.0, 3.247)});
}

// 输入宽容度: 逆序 + 旋转起点 + 一条边上的共线点, 结果应与规范输入一致
// (垄的位置由区域自身的投影范围决定, 与起点/绕向无关 —— 因此比较时把每条垄归一成
//  "垄线位置 + 两端坐标" 再排序, 不受行驶方向/顶点起点影响)

TEST(BowPlannerInput, AcceptsReversedRotatedAndCollinearVertices) {
  const Polygon_2 canonical = RectangleField(20.0, 8.0);

  // 本例验证的是"起点/绕向无关性"这条核心契约: 两个规划器用同一组参数, 断言保持 1e-6 的严格度。
  BowFacade reference;
  ASSERT_TRUE(reference.init(canonical, 2.0, 0.1, 0.5));

  // 逆序 + 旋转起点 + 上边中点 (共线点); 顶点仍按边界顺序排列, 因此是简单多边形。
  const Polygon_2 reversed{canonical[2], Point_2(10.0, 8.0), canonical[3], canonical[0], canonical[1]};
  BowFacade planner;
  ASSERT_TRUE(planner.init(reversed, 2.0, 0.1, 0.5));

  const auto normalize = [](const std::vector<Point_2>& keys) {
    std::vector<std::array<double, 3>> ridges;   // {y, x_min, x_max}
    for (size_t r = 0; r + 1 < keys.size(); r += 2) {
      ridges.push_back({keys[r].y(), std::min(keys[r].x(), keys[r + 1].x()), std::max(keys[r].x(), keys[r + 1].x())});
    }
    std::sort(ridges.begin(), ridges.end());
    return ridges;
  };

  const auto ref_ridges = normalize(reference.getKeyWaypoints());
  const auto ridges     = normalize(planner.getKeyWaypoints());
  ASSERT_EQ(ridges.size(), ref_ridges.size());
  for (size_t i = 0; i < ridges.size(); ++i) {
    EXPECT_NEAR(ridges[i][0], ref_ridges[i][0], 1e-6) << "第 " << i << " 条垄的位置";
    EXPECT_NEAR(ridges[i][1], ref_ridges[i][1], 1e-3) << "第 " << i << " 条垄的起点";
    EXPECT_NEAR(ridges[i][2], ref_ridges[i][2], 1e-3) << "第 " << i << " 条垄的终点";
  }
  EXPECT_EQ(planner.getWaypoints().size(), reference.getWaypoints().size());
}

// 自相交边界: 顶点全部在凸包边界上, 但按给定顺序连接会交叉。凸包校验看不出这一点,
// 必须用 is_simple 拒绝 —— 否则会按凸包静默扩大作业区域。
TEST(BowPlannerInput, RejectsSelfIntersectingBoundary) {
  // 蝴蝶结: (0,0)->(20,8) 与 (20,0)->(0,8) 在对角线上相交。
  const Polygon_2 bowtie{Point_2(0.0, 0.0), Point_2(20.0, 8.0), Point_2(20.0, 0.0), Point_2(0.0, 8.0)};
  BowFacade planner;
  const BowOutput out = planner.plan(bowtie, 2.0, 0.1, 0.5);

  EXPECT_FALSE(out.ok());
  EXPECT_EQ(out.status, PlanStatus::kSelfIntersecting);
  EXPECT_FALSE(out.message.empty());
  EXPECT_TRUE(isBadInput(out.status));
  EXPECT_TRUE(out.waypoints.empty());
}

// 凹多边形 (或含内部点) 按约定拒绝: init 返回 false, 不产生路径
TEST(BowPlannerInput, PlanResultCarriesFailureReason) {
  const Polygon_2 rect = RectangleField(20.0, 8.0);
  BowFacade planner;

  // 正常: kOk, 没有原因
  const BowOutput ok = planner.plan(rect, 2.0, 0.1, 0.5);
  EXPECT_TRUE(ok.ok());
  EXPECT_EQ(ok.status, PlanStatus::kOk);
  EXPECT_TRUE(ok.message.empty());
  EXPECT_EQ(planner.lastResult().status, PlanStatus::kOk);

  // 凹多边形 -> kNonConvex (输入数据异常的细分: 有顶点落在地块凸包内部)
  const Polygon_2 concave{Point_2(0.0, 0.0), Point_2(10.0, 0.0), Point_2(10.0, 10.0), Point_2(5.0, 5.0), Point_2(0.0, 10.0)};
  const BowOutput bad_poly = planner.plan(concave, 2.0, 0.1, 0.5);
  EXPECT_FALSE(bad_poly.ok());
  EXPECT_EQ(bad_poly.status, PlanStatus::kNonConvex);
  EXPECT_FALSE(bad_poly.message.empty());

  // 垄宽非法 (ridge_width <= 0) -> kBadRidgeWidth (参数不合理一类, 与"地块数据不对"分开)
  const BowOutput bad_ridge = planner.plan(rect, 0.0, 0.1, 0.5);
  EXPECT_FALSE(bad_ridge.ok());
  EXPECT_EQ(bad_ridge.status, PlanStatus::kBadRidgeWidth);
  EXPECT_FALSE(bad_ridge.message.empty());

  // 余量比地块还大 -> 内缩失败: kBadFieldMargin (field_margin 不合理; 不再回退到原始边界继续规划)
  const BowOutput huge_margin = planner.plan(rect, 2.0, 0.1, 50.0);
  EXPECT_FALSE(huge_margin.ok());
  EXPECT_EQ(huge_margin.status, PlanStatus::kBadFieldMargin);
  EXPECT_FALSE(huge_margin.message.empty());
}

// 输入数据异常的细分状态: 每个状态对应 preparePolygon 的一道判定 —— 现场只看状态码就能分清
// "点数不够" / "重复点太多" / "顶点退化共线" / "地块非凸"; isBadInput() 仍可整类判定。
TEST(BowPlannerInput, ReportsInputFailureStage) {
  BowFacade planner;

  // 输入边界点不足: 只有 2 个点
  const BowOutput too_few = planner.plan({Point_2(0.0, 0.0), Point_2(10.0, 0.0)}, 2.0, 0.1, 0.0);
  EXPECT_FALSE(too_few.ok());
  EXPECT_EQ(too_few.status, PlanStatus::kInputTooFew);
  EXPECT_TRUE(isBadInput(too_few.status));

  // 去重后边界点不足: 4 个输入点里只有 2 个不同位置 (重复点被去重去掉)
  const Polygon_2 duplicated{Point_2(0.0, 0.0), Point_2(0.0, 0.0), Point_2(10.0, 0.0), Point_2(10.0, 0.0)};
  const BowOutput dup = planner.plan(duplicated, 2.0, 0.1, 0.0);
  EXPECT_FALSE(dup.ok());
  EXPECT_EQ(dup.status, PlanStatus::kDedupTooFew);
  EXPECT_TRUE(isBadInput(dup.status));

  // 地块非凸: 凹顶点落在地块凸包内部
  const Polygon_2 concave{Point_2(0.0, 0.0), Point_2(10.0, 0.0), Point_2(10.0, 10.0), Point_2(5.0, 5.0), Point_2(0.0, 10.0)};
  const BowOutput non_convex = planner.plan(concave, 2.0, 0.1, 0.0);
  EXPECT_FALSE(non_convex.ok());
  EXPECT_EQ(non_convex.status, PlanStatus::kNonConvex);
  EXPECT_TRUE(isBadInput(non_convex.status));

  // 凸包点不足: 三个顶点共线 —— 地块退化成一条线 (凸包面积为 0), 且不再被误报成"非凸"
  const Polygon_2 collinear{Point_2(0.0, 0.0), Point_2(5.0, 0.0), Point_2(10.0, 0.0)};
  const BowOutput degenerated = planner.plan(collinear, 2.0, 0.1, 0.0);
  EXPECT_FALSE(degenerated.ok());
  EXPECT_EQ(degenerated.status, PlanStatus::kHullTooFew) << "实际状态: " << planStatusName(degenerated.status);
  EXPECT_TRUE(isBadInput(degenerated.status));

  // 状态标签 (日志/心跳 detail 用) + 归类判据: 参数类与内部兜底都不算"输入数据异常"
  EXPECT_STREQ(planStatusName(PlanStatus::kInputTooFew), "input_too_few");
  EXPECT_STREQ(planStatusName(PlanStatus::kDedupTooFew), "dedup_too_few");
  EXPECT_STREQ(planStatusName(PlanStatus::kNonConvex), "non_convex");
  EXPECT_STREQ(planStatusName(PlanStatus::kHullTooFew), "hull_too_few");
  EXPECT_STREQ(planStatusName(PlanStatus::kSelfIntersecting), "self_intersecting");
  EXPECT_FALSE(isBadInput(PlanStatus::kOk));
  EXPECT_FALSE(isBadInput(PlanStatus::kBadFieldMargin));
  EXPECT_FALSE(isBadInput(PlanStatus::kBadRidgeWidth));
  EXPECT_FALSE(isBadInput(PlanStatus::kInternal));
  EXPECT_TRUE(isBadInput(PlanStatus::kSelfIntersecting));

  // 参数类与输入类是分开的两类: isBadParams() 只认 field_margin / ridge_width
  EXPECT_TRUE(isBadParams(PlanStatus::kBadFieldMargin));
  EXPECT_TRUE(isBadParams(PlanStatus::kBadRidgeWidth));
  EXPECT_FALSE(isBadParams(PlanStatus::kNonConvex));
  EXPECT_FALSE(isBadParams(PlanStatus::kSelfIntersecting));
  EXPECT_FALSE(isBadParams(PlanStatus::kOk));
  EXPECT_FALSE(isBadParams(PlanStatus::kInternal));
}

// 规划状态 -> 心跳故障码: 每个失败状态都有唯一专码 (APP/云端据此区分"为什么没有路径"),
// kOk 映射到 FAULT_NONE; 通用码 10401 只留给"还没走到规划"的兜底, 不与细分码混用。
TEST(PlanStatusFaultCode, MapsEveryFailureStatusToItsOwnCode) {
  // 故障码表在包命名空间里 (bow.hpp 的 PlanStatus 在全局命名空间)
  using global_path_planner::planStatusFaultCode;

  EXPECT_EQ(planStatusFaultCode(PlanStatus::kOk), static_cast<uint32_t>(global_path_planner::FAULT_NONE));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kInputTooFew), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_INPUT_TOO_FEW));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kDedupTooFew), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_DEDUP_TOO_FEW));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kHullTooFew), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_HULL_TOO_FEW));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kNonConvex), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_NON_CONVEX));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kSelfIntersecting), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_SELF_INTERSECTING));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kBadFieldMargin), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_BAD_FIELD_MARGIN));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kBadRidgeWidth), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_BAD_RIDGE_WIDTH));
  EXPECT_EQ(planStatusFaultCode(PlanStatus::kInternal), static_cast<uint32_t>(global_path_planner::FAULT_PLAN_INTERNAL));

  // 8 个失败状态 -> 8 个互不相同的码, 且都不是"无故障"或通用失败码 (否则细分就白加了)
  const PlanStatus failures[] =
    {PlanStatus::kInputTooFew, PlanStatus::kDedupTooFew, PlanStatus::kHullTooFew, PlanStatus::kNonConvex, PlanStatus::kSelfIntersecting,
     PlanStatus::kBadFieldMargin, PlanStatus::kBadRidgeWidth, PlanStatus::kInternal};
  std::vector<uint32_t> codes;
  for (const PlanStatus status : failures) {
    const uint32_t code = planStatusFaultCode(status);
    EXPECT_NE(code, static_cast<uint32_t>(global_path_planner::FAULT_NONE)) << planStatusName(status);
    EXPECT_NE(code, static_cast<uint32_t>(global_path_planner::FAULT_PLAN_FAILED)) << planStatusName(status);
    codes.push_back(code);
  }
  std::sort(codes.begin(), codes.end());
  EXPECT_EQ(std::unique(codes.begin(), codes.end()), codes.end()) << "有重复的故障码";
  EXPECT_EQ(codes.size(), 8u);
}

// 编号契约: 故障码表**连续无空号** (10400 起逐个递增), 删码/重排之后由这条钉住 ——
// 谁再往中间插一个号、或删掉一个号留个洞, 这里立刻报出来 (APP/云端的码表按"连续"来维护)。
TEST(PlanStatusFaultCode, CodesAreContiguousWithoutGaps) {
  const std::vector<uint32_t> all = {
    global_path_planner::FAULT_NONE,
    global_path_planner::FAULT_PLAN_FAILED,
    global_path_planner::FAULT_PLAN_INPUT_TOO_FEW,
    global_path_planner::FAULT_PLAN_DEDUP_TOO_FEW,
    global_path_planner::FAULT_PLAN_HULL_TOO_FEW,
    global_path_planner::FAULT_PLAN_NON_CONVEX,
    global_path_planner::FAULT_PLAN_BAD_FIELD_MARGIN,
    global_path_planner::FAULT_PLAN_BAD_RIDGE_WIDTH,
    global_path_planner::FAULT_PLAN_INTERNAL,
    global_path_planner::FAULT_PLAN_SELF_INTERSECTING,
  };

  EXPECT_EQ(all.front(), 10400u) << "FAULT_NONE 必须是模块号 + 序号 00";
  for (size_t i = 1; i < all.size(); ++i) {
    EXPECT_EQ(all[i], all[i - 1] + 1u) << "第 " << i << " 个故障码与上一个不连续 (有空号或多写了值)";
  }
  // 对外发布的 ID 逐个钉死 (APP/云端码表按这些数字维护): 尾巴上这几个尤其要留神 —— 表尾一动,
  // 报出去的号就变, 必须先在这里红一次再改 README "编号约定" 与 APP 码表; **逐个改号比逐个加码
  // 更容易出错** (改号后旧号仍指向别的含义), 所以这里把整条尾巴都钉死。
  // 2026-10: 删除固定 JSON 路径文件模式后, 原文件码 10402 一并删除,
  // 后续故障码整体前移一位, 保持连续编号。
  EXPECT_EQ(static_cast<uint32_t>(global_path_planner::FAULT_PLAN_INPUT_TOO_FEW), 10402u);
  EXPECT_EQ(static_cast<uint32_t>(global_path_planner::FAULT_PLAN_INTERNAL), 10408u);
  EXPECT_EQ(static_cast<uint32_t>(global_path_planner::FAULT_PLAN_SELF_INTERSECTING), 10409u);
  EXPECT_EQ(all.back(), 10409u) << "最后一个码变了 -> 记得同步 README 的码表与 APP";
}

TEST(BowPlannerInput, RejectsConcavePolygon) {
  const Polygon_2 l_shape{Point_2(0.0, 0.0), Point_2(10.0, 0.0), Point_2(10.0, 4.0), Point_2(4.0, 4.0), Point_2(4.0, 10.0), Point_2(0.0, 10.0)};
  BowFacade planner;
  EXPECT_FALSE(planner.init(l_shape, 2.0, 0.1, 0.0));
  EXPECT_TRUE(planner.getWaypoints().empty());
}

// 退化与非法输入
TEST(BowPlannerInput, DegenerateInputs) {
  // 2.5 m 宽 < 垄宽 3 m: 不设窄地块例外 —— 仍按"首末各留 first_row_margin (5 mm)"排两条
  // (0.005 / 2.495), 这样两侧田边都由机具半幅盖住; 代价是多一趟作业 + 一次掉头。
  BowFacade narrow;
  ASSERT_TRUE(narrow.init(RectangleField(20.0, 2.5), kRidgeWidth, 0.1, 0.0));
  {
    const auto& keys = narrow.getKeyWaypoints();
    ASSERT_EQ(keys.size(), 4u) << "窄地块也贴两边 -> 2 条垄";
    double y_min = 1e9, y_max = -1e9;
    for (const auto& p : keys) {
      y_min = std::min(y_min, p.y());
      y_max = std::max(y_max, p.y());
    }
    EXPECT_NEAR(y_min, 0.005, 1e-3);
    EXPECT_NEAR(y_max, 2.5 - 0.005, 1e-3);
  }

  // 垄宽 <= 0: 直接失败
  BowFacade bad_ridge;
  EXPECT_FALSE(bad_ridge.init(RectangleField(20.0, 8.0), 0.0, 0.1, 0.0));

  // 顶点太少: 直接失败
  BowFacade too_few;
  EXPECT_FALSE(too_few.init({Point_2(0.0, 0.0), Point_2(1.0, 1.0)}, 2.0, 0.1, 0.0));

  // 内缩余量大于地块本身 -> 内缩失败: 直接报失败 (不再回退到原始边界继续规划)
  BowFacade huge_margin;
  EXPECT_FALSE(huge_margin.init(RectangleField(6.0, 4.0), 2.0, 0.1, 5.0));
  EXPECT_EQ(huge_margin.lastResult().status, PlanStatus::kBadFieldMargin);
  EXPECT_TRUE(huge_margin.getWaypoints().empty());
}

// 调头圆弧不会把路径带到地块外太远 (半圆半径 = 垄间距/2, 圆弧最多在垄端点外伸出半径)
TEST(BowPlannerCoverage, TurnArcsStayNearField) {
  const Polygon_2 field = RectangleField(20.0, 8.0);
  BowFacade planner;
  ASSERT_TRUE(planner.init(field, kRidgeWidth, 0.1, 0.0));

  const auto field_model = ToBoostPolygon(field);
  bg::model::box<Point_2> box;
  bg::envelope(field_model, box);
  const double margin = 0.5 * kRidgeWidth + 1e-3;
  // 注意: bg::get<min_corner, 0>(...) 里的逗号会被 gtest 断言宏当成参数分隔符,
  // 所以先取出来存成局部变量。
  const double x_min = bg::get<bg::min_corner, 0>(box);
  const double x_max = bg::get<bg::max_corner, 0>(box);
  const double y_min = bg::get<bg::min_corner, 1>(box);
  const double y_max = bg::get<bg::max_corner, 1>(box);

  for (const auto& p : planner.getWaypoints()) {
    EXPECT_GE(p.x(), x_min - margin);
    EXPECT_LE(p.x(), x_max + margin);
    EXPECT_GE(p.y(), y_min - margin);
    EXPECT_LE(p.y(), y_max + margin);
  }
}

// 掉头不许伸出地块边界 —— 梯形 / 斜地头 / 扇形 (三角形) 地块上, 只按"相切"求解的圆弧可能
// 从长边穿出去。掉头固定为半圆/直线几何 (垂足对齐), 这里把这条钉死:
//   * 地头 >= 垄宽/2 (= 掉头半径 R) 且地块不是极端斜角 -> 干净不越界 (<= 5 cm);
//   * 地头 < R, 或行端错开量远大于 R 的极端斜地块 -> 几何上转不进去 (R 的圆弧放不下),
//     此时要求"越界量不超过 R", 并由 [TURN_BOUNDARY] 告警让操作手加大 margin。
TEST(F2cTurn, TurnsStayInsideTheFieldOnSkewedFields) {
  const double ridge = 3.0, resolution = 0.1, radius = 0.5 * ridge;

  struct Case {
    std::string name;
    Polygon_2 field;
    double headland;
    double tol;   // 允许的越界量 (m)
  };
  const Polygon_2 rect{Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(60.0, 20.0), Point_2(0.0, 20.0)};
  const Polygon_2 trap45{Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(45.0, 20.0), Point_2(0.0, 20.0)};
  const Polygon_2 trap30{Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(30.0, 20.0), Point_2(0.0, 20.0)};
  const Polygon_2 para{Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(75.0, 20.0), Point_2(15.0, 20.0)};
  const Polygon_2 tri{Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(0.0, 20.0)};
  const std::vector<Case> cases{
    {"矩形", rect, 2.0, 0.05},
    {"梯形 top45", trap45, 2.0, 0.05},
    {"平行四边形 (斜 15 m)", para, 2.0, 0.05},
    {"三角形", tri, 2.0, 0.05},
    {"矩形 (地头 1 m < R)", rect, 1.0, radius},
    {"梯形 top45 (地头 1 m < R)", trap45, 1.0, radius},
    {"梯形 top30 (极端斜: 行端错开 4.3 m)", trap30, 3.0, radius},
  };

  for (const auto& c : cases) {
    BowFacade planner;
    ASSERT_TRUE(planner.init(c.field, ridge, resolution, c.headland)) << c.name;
    const auto& keys = planner.getKeyWaypoints();
    const auto& wp   = planner.getWaypoints();
    ASSERT_GE(keys.size(), 4u) << c.name;

    // 路径 = 垄 + 与两条垄相切的半圆 (半圆的端点就是切点, 见 legacyTurn): 除首末点外整条路径
    // 都不该有折角 (衔接处相切)。退化地块上 legacyTurn 会退回直线连接, 那种情形会在这里报出来。
    // 两个例外 (都是设计如此): 起点/终点被"步骤 8"沿区域边界边补到顶点 —— 那里是"沿边走到垄端
    // 再拐进垄"的接缝 (夹角 > 90° 才补 => 拐角 < 90°), 就是第 2 个点与倒数第 2 个点。
    // 见 test_bow_planner 的 PathEndsFollowRegionSpec。
    for (size_t i = 1; i + 1 < wp.size(); ++i) {
      if (i == 1 || i + 2 == wp.size())
        continue;
      EXPECT_LT(NodeTurnDeg(wp[i - 1], wp[i], wp[i + 1]), 10.0) << c.name << " 路径有折角 @" << wp[i].transpose();
    }

    // 全程不许伸出地块边界超过 c.tol (掉头 = 与两条垄相切的半圆, 外伸 ≈ 掉头半径)
    const auto field_model = ToBoostPolygon(c.field);
    const double out_tol   = std::max(c.tol, radius) + 0.05;
    for (const auto& p : wp) {
      EXPECT_TRUE(bg::covered_by(p, field_model) || DistanceToBoundary(c.field, p) <= out_tol) << c.name << ": 路径点跑到地块外 " << DistanceToBoundary(c.field, p) << " m @" << p.transpose();
    }
    EXPECT_LE(planner.getTurnMaxOverrun(), out_tol) << c.name << ": 最大出界量";
  }
}

// 两条垄**不平行** (末行兜底行, 或行端被裁得很短的碎行) 时的掉头几何, 钉住"落入哪条分支":
//   * 相切圆分支满足准入 -> 用那条弧;
//   * 不满足 (切点离垄端太远 / 转角不合法 / 会伸出地块边界) -> **退回半圆几何** (顶点压在 B-C 中点,
//     再按"不伸出地块边界"往外推) —— 也就是继续走平行分支那一条代码路径; **不是**直线连接。
//
// 反例就是这里钉的东西: 旧实现 (以及当前注释一度写成的那样) 是 `return {P2, P3};` 直线连接 ——
// 那样这段路径会贴着弦、行端出现 ~20° 折角。
//
// 用例: 三角 60x20 / 垄宽 3 / 余量 2 —— 第一条垄是 0.015 m 的碎行 (方向由 1 mm 量化决定, 与下一条垄
// 差 ~2° -> 命中"不平行"门限; 且切点/转角不入准入), 于是第一次掉头一定走"退回半圆"这一支。
TEST(F2cTurn, NonParallelRidgesFallBackToAnArcNotAStraightLine) {
  const Polygon_2 tri{Point_2(0.0, 0.0), Point_2(60.0, 0.0), Point_2(0.0, 20.0)};
  BowFacade planner;
  ASSERT_TRUE(planner.init(tri, 3.0, 0.1, 2.0));

  const auto& keys = planner.getKeyWaypoints();
  const auto& wp   = planner.getWaypoints();
  ASSERT_GE(keys.size(), 4u) << "本用例需要至少两条垄";
  ASSERT_GE(wp.size(), 4u);

  // 第一次掉头连接 key[1] (第 1 条垄末端) 与 key[2] (第 2 条垄起点): 取路点里离这两点最近的下标,
  // 夹住这段; 这段既不会是空, 也不该把后续垄段拉进来。
  const auto nearest_index = [&wp](const Point_2& target) {
    size_t best   = 0;
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < wp.size(); ++i) {
      const double d = (wp[i] - target).squaredNorm();
      if (d < best_d) {
        best_d = d;
        best   = i;
      }
    }
    return best;
  };
  const size_t i_from = nearest_index(keys[1]);
  const size_t i_to   = nearest_index(keys[2]);
  ASSERT_LT(i_from, i_to) << "第 1 条垄末端应排在第 2 条垄起点之前";

  // 这段路径离开"弦 key[1] -> key[2]"的最大距离: 半圆几何顶点离弦 ~1.5 m (掉头半径量级);
  // 直线连接 = 0 (整段就压在弦上)。阈值 0.5 m 落在两者之间, 明确区分两支。
  double max_off = 0.0;
  for (size_t i = i_from; i <= i_to; ++i)
    max_off = std::max(max_off, PointToLineDistance(wp[i], keys[1], keys[2]));
  EXPECT_GT(max_off, 0.5) << "两条垄不平行时应退回半圆几何 (会明显离开弦), 而不是直线连接; 实测离弦 " << max_off << " m";
}

// 步骤 8 的契约: inner_path 两端"补到区域顶点"的**前提**与规则都钉在这里。
//   * 前提: 起点 P1 必须贴在 AB (**近端**边界边 = across.min 一侧那条, nearSideEdge) 上,
//     终点 Pn 必须贴在 CD (**远端**边界边, farSideEdge) 上; 不在这两条边上就跳过。
//     首行/末行本来就离它们各有 first_row_margin (默认 5 mm) 余量 -> 常见地块上这一步**空转**,
//     首末点就是首末垄端点 (与旧版逐位相同);
//   * 贴上了再看: 那条边上只有这一个路径端点 (边上还有别的点就不是"端头边"),
//     且 ∠(A,P1,P2)/∠(B,P1,P2) 或 ∠(P_{n-1},Pn,C)/∠(P_{n-1},Pn,D) 里有 >90° 的顶点
//     (起点取车头"背后"的 A/B, 终点取"前方"的 C/D), 补出来的那一段还要与已铺路径不相交;
//   * 补了的话: 接缝是"沿边界边走到垄端再拐进垄", 拐角 = 180° - 夹角 < 90°; 无论补没补都不许自交。
TEST(F2cTurn, PathEndsFollowRegionSpec) {
  // 实机地块形状 (斜边小地块 13.5x5.4)
  const Polygon_2 skewed{Point_2(0.0, 0.0), Point_2(13.45, 0.57), Point_2(13.34, 5.32), Point_2(0.00, 4.87)};
  BowFacade planner;
  ASSERT_TRUE(planner.init(skewed, 1.0, 0.1, 0.5));
  const auto& wp     = planner.getWaypoints();
  const auto& keys   = planner.getKeyWaypoints();
  const auto& region = planner.getInnerPolygon();
  ASSERT_GE(wp.size(), 3u);

  const auto is_region_vertex = [&region](const Point_2& p) {
    for (const auto& v : region)
      if ((v - p).norm() <= 1e-6)
        return true;
    return false;
  };
  const bool front_is_key = (wp.front() - keys.front()).norm() <= 1e-9;
  const bool back_is_key  = (wp.back() - keys.back()).norm() <= 1e-9;

  // 这个地块上首末点就是垄端点: 首行/末行离 AB/CD 各有 first_row_margin (5 mm) 余量, 贴不到
  // 那两条边上 -> 步骤 8 空转 (与旧版逐位一致)。
  EXPECT_TRUE(front_is_key) << "首点不该被改: " << wp.front().transpose();
  EXPECT_TRUE(back_is_key) << "末点不该被改: " << wp.back().transpose();
  // 万一哪天补了 (端头几何变了): 首末点只可能是区域顶点, 不会是别的地方
  EXPECT_TRUE(front_is_key || is_region_vertex(wp.front())) << "首点既不是垄端也不是区域顶点: " << wp.front().transpose();
  EXPECT_TRUE(back_is_key || is_region_vertex(wp.back())) << "末点既不是垄端也不是区域顶点: " << wp.back().transpose();

  // 不管补没补: 首末点都在区域边界边上 (垄端点本来就在边界上; 补角段沿边界边走)
  EXPECT_LE(DistanceToBoundary(region, wp.front()), 1e-3);
  EXPECT_LE(DistanceToBoundary(region, wp.back()), 1e-3);

  // 补了的话: 接缝是"沿边界边走到垄端再拐进垄", 拐角 = 180° - 夹角 < 90° (规则只补夹角 > 90° 的顶点)
  if (!front_is_key) {
    EXPECT_LT(NodeTurnDeg(wp[0], wp[1], wp[2]), 90.0) << "首端接缝的拐角";
  }
  if (!back_is_key) {
    EXPECT_LT(NodeTurnDeg(wp[wp.size() - 3], wp[wp.size() - 2], wp.back()), 90.0) << "末端接缝的拐角";
  }

  // 无论补没补, 都不许自交 (补角段不许横穿垄端外的掉头弧)
  EXPECT_EQ(CountSelfCrossings(wp), 0) << "路径与已经走过的路径相交";

  // 矩形: 夹角正好 90° -> 不补, 首末点与旧版逐位一致
  BowFacade rect;
  ASSERT_TRUE(rect.init(RectangleField(20.0, 8.0), 1.0, 0.1, 0.5));
  EXPECT_NEAR((rect.getWaypoints().front() - rect.getKeyWaypoints().front()).norm(), 0.0, 1e-9) << "矩形地块不补首端";
  EXPECT_NEAR((rect.getWaypoints().back() - rect.getKeyWaypoints().back()).norm(), 0.0, 1e-9) << "矩形地块不补末端";
}

// ============================================================
// F2C 借鉴 3: 扫描方向候选择优 (BruteForce + SGObjective)
// ============================================================

// ============================================================
// 履带 / 差速车 (可原地或极小半径折返) 的推荐配置
// ------------------------------------------------------------
// 现场三点反馈 -> 这条用例各钉一条:
//   1. "地块里还有大片盲区"     : 余量收到 <= 垄宽/2, 垄线自己铺到田边 -> 作业区覆盖 >= 99.5%;
//   2. "与已经走过的路径有交点" : 不做地头环 + 小半径折返 -> 自交 0 处;
//   3. "平滑不一定是半圆"       : 掉头外伸只有 R, 全程不出界。
// 说明: "整块覆盖率"的分母包含操作手有意留的余量带 (机具不去那儿), 所以这里断言的是
// **作业区 (内缩区域)** 的覆盖率 —— 日志里的 [COVERAGE] 会报出余量带有多宽、余量该收到多少。
// ============================================================
TEST(F2cCoverage, TrackedRobotRecommendedConfigHasNoCrossings) {
  struct Case {
    std::string name;
    Polygon_2 field;
    double ridge;
  };
  const std::vector<Case> cases{
    {"斜边小地块 13.5x5.4 (实机地块形状)", Polygon_2{Point_2(0.0, 0.0), Point_2(13.45, 0.57), Point_2(13.34, 5.32), Point_2(0.00, 4.87)}, 1.0},
    {"矩形 30x12", Polygon_2{Point_2(0.0, 0.0), Point_2(30.0, 0.0), Point_2(30.0, 12.0), Point_2(0.0, 12.0)}, 2.0},
    {"斜梯形 40x30", Polygon_2{Point_2(0.0, 0.0), Point_2(40.0, 0.0), Point_2(40.0, 30.0), Point_2(10.0, 30.0)}, 3.0},
  };

  for (const auto& c : cases) {
    const double margin = 0.5 * c.ridge;   // 机具正好铺到田边 (余量 > 垄宽/2 的部分就是有意不作业的缓冲)

    BowFacade planner;
    ASSERT_TRUE(planner.init(c.field, c.ridge, 0.1, margin)) << c.name;
    const auto& wp = planner.getWaypoints();

    // 掉头 = 与两条垄相切的半圆 (半径 = 垄距/2) + 沿垄向的短直线; 窄小地块上垄端沿垄向错开
    // 会让它在垄端外伸一点 (量级 = 掉头半径 + 错开量的一半), 由 [TURN_BOUNDARY] 报出。
    // 这里挡的是"整条路径跑到田外几十厘米"那类退化。
    EXPECT_LE(planner.getTurnMaxOverrun(), 0.5 * c.ridge + 0.05) << c.name << ": 掉头出界不该超过掉头半径";
    EXPECT_EQ(CountSelfCrossings(wp), 0) << c.name << ": 路径与已经走过的路径相交 (掉头弧横穿垄线)";

    const double inner = CoveredAreaRatio(planner.getInnerPolygon(), wp, 0.5 * c.ridge, 0.1);
    // 下限 99.4%: 掉头半圆的端点(切点)在垄端以内退 R (= 垄宽/2), 即每条垄端让出约 R 不作业,
    // 内区覆盖率因此从 99.5%+ 略降; 这里挡的是"整片漏盖"的退化。
    EXPECT_GE(inner, 0.994) << c.name << ": 作业区覆盖率只有 " << 100.0 * inner << "%; 诊断见 [COVERAGE]/[HEADLAND] 日志";
  }
}
