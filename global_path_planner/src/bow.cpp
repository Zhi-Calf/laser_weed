#include "global_path_planner/bow.hpp"

#include <clipper2/clipper.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "bow_detail.hpp"   // 常量 + 几何小工具
#include "global_path_planner/eigen_boost_adaptor.hpp"

namespace bg = boost::geometry;
namespace c2 = Clipper2Lib;

using namespace bow_detail;

// ============================================================
// 各步骤的实现 (全部是 PlanBowShapedPath 的私有成员函数; 声明与契约见 bow.hpp)
// ============================================================
bool PlanBowShapedPath::preparePolygon(const Polygon_2& cell, Polygon_2& out) {
  // 输入边界点至少 3 个
  if (cell.size() < 3) {
    SPDLOG_ERROR("输入边界点不足: {} 个", cell.size());
    fail(PlanStatus::kInputTooFew, "输入边界点不足 (" + std::to_string(cell.size()) + " 个, 需要 >= 3)");
    return false;
  }

  // 去重后的边界点至少 3 个
  Polygon_2 pts = cell;
  removeDuplicateVertices(pts);
  if (pts.size() < 3) {
    SPDLOG_ERROR("去重后边界点不足: {} 个", pts.size());
    fail(PlanStatus::kDedupTooFew, "去重后的边界点不足 (" + std::to_string(pts.size()) + " 个, 重复点太多)");
    return false;
  }

  // 规范输入边界点顺序 (顺时针)
  bg::model::multi_point<Point_2> input;
  input.reserve(pts.size());
  for (const auto& p : pts)
    input.push_back(p);
  bg::model::polygon<Point_2> hull;
  bg::convex_hull(input, hull);
  bg::correct(hull);

  // 凸包点不足
  Polygon_2 ordered(hull.outer().begin(), hull.outer().end() - 1);   // 去掉闭合点
  if (ordered.size() < 3 || std::abs(bg::area(hull)) <= kMinHullArea) {
    SPDLOG_ERROR("凸包点不足: {} 个 (面积 {:.6f} m^2, 顶点退化或共线)", ordered.size(), std::abs(bg::area(hull)));
    fail(PlanStatus::kHullTooFew, "凸包点不足 (" + std::to_string(ordered.size()) + " 个或面积为 0, 地块顶点退化/共线)");
    return false;
  }

  // 输入边界必须是简单多边形
  bg::model::linestring<Point_2> input_line(pts.begin(), pts.end());
  input_line.push_back(pts.front());
  if (!bg::is_simple(input_line)) {
    SPDLOG_ERROR("地块边界自相交: 顶点按给定顺序连接后不是简单多边形 (is_simple == false)");
    fail(PlanStatus::kSelfIntersecting, "地块边界自相交: 顶点必须按边界顺序给出, 相邻边不能交叉 (自相交/乱序输入会被拒绝)");
    return false;
  }

  // 地块非凸
  for (const auto& p : pts) {
    if (bg::within(p, hull)) {
      SPDLOG_ERROR("地块非凸多边形，弓字形规划需要凸多边形 (顶点 ({:.3f}, {:.3f}) 在凸包内部)", p.x(), p.y());
      fail(PlanStatus::kNonConvex, "地块非凸 (顶点落在地块的凸包内部)");
      return false;
    }
  }

  // 起点固定到"最小 x, 再最小 y"的顶点 (而不是跟着输入的第一个顶点)。这样同一个简单地块
  // 无论起点/绕向如何, 喂给 Clipper2 的路径完全一致 —— 1 mm 量化下的内缩结果、以及端部补点
  // 那种"阈值判定"才是确定的; 否则"起点/绕向无关"会被放大成可见差异。
  size_t start = 0;
  for (size_t i = 1; i < ordered.size(); ++i) {
    const double dx = ordered[i].x() - ordered[start].x();
    if (dx < -kMinVertexSpacing || (std::abs(dx) <= kMinVertexSpacing && ordered[i].y() < ordered[start].y()))
      start = i;
  }
  std::rotate(ordered.begin(), ordered.begin() + static_cast<long>(start), ordered.end());

  SPDLOG_INFO("地块顶点数: {}, 面积: {:.3f} m^2", ordered.size(), std::abs(bg::area(hull)));
  out = ordered;
  return true;
}

// ============================================================
// 按 field_margin_ 统一内缩四周 -> 交给 Clipper2 偏置 (negative delta = 向内)
// ============================================================
Polygon_2 PlanBowShapedPath::offsetUniformly(const Polygon_2& poly, double margin) {
  // 凸多边形向内偏置不会进入 Clipper2 的 Miter join 分支, 拐角上限固定使用库默认值。
  const c2::PathsD inset = c2::InflatePaths(c2::PathsD{toClipperPath(poly)}, -margin, c2::JoinType::Miter, c2::EndType::Polygon, 2.0, kClipperPrecision);
  if (inset.empty()) {
    SPDLOG_WARN("边界偏置 {:.3f} m 后区域为空", margin);
    return {};
  }
  return fromClipperPath(largestPath(inset));
}

// ============================================================
// 内缩入口 (选择偏置方式 + 结果校验/规范化)
// ============================================================
Polygon_2 PlanBowShapedPath::insetPolygon(const Polygon_2& poly) {
  // 余量 <= 0: **不内缩, 原样返回地块边界**。
  const double margin = std::max(0.0, field_margin_);
  if (margin <= kEpsilon) {
    SPDLOG_INFO("偏置 {:.3f} <= 0, 不内缩 (原样使用地块边界)", margin);
    return poly;
  }

  Polygon_2 inset = offsetUniformly(poly, margin);
  if (inset.empty()) {
    SPDLOG_ERROR("field_margin_ 不合理: 偏置 {:.3f} m 后区域为空", margin);
    fail(PlanStatus::kBadFieldMargin, "field_margin_ 不合理: 内缩后区域为空或退化");
    return {};
  }

  // Clipper2 的输出先去掉重合点, 再用 Boost.Geometry 校验 (简单多边形 + 有面积)。
  c2::PathD path = toClipperPath(inset);
  c2::StripDuplicates(path, true);
  inset = fromClipperPath(path);
  if (!isValidRing(inset, kEpsilon)) {
    SPDLOG_ERROR("field_margin_ 不合理: 偏置结果不是有效多边形 (顶点数 {})", inset.size());
    fail(PlanStatus::kBadFieldMargin, "field_margin_ 不合理: 内缩后区域为空或退化");
    return {};
  }

  inset = normalizedRing(inset);
  rotateToNearest(inset, poly.front());
  return inset;
}

// ============================================================
// 扫描方向 = 让"垂直方向宽度"最小的边方向 (垄数最少 = 掉头次数最少)
// ============================================================
Point_2 PlanBowShapedPath::minWidthDirection(const Polygon_2& poly) {
  double best_width = std::numeric_limits<double>::max();
  Point_2 best_dir(1.0, 0.0);
  for (size_t i = 0; i < poly.size(); ++i) {
    const Point_2 edge = poly[(i + 1) % poly.size()] - poly[i];
    if (edge.norm() < kMinVertexSpacing)
      continue;
    const Point_2 dir  = edge.normalized();
    const double width = projectRange(poly, perpendicular(dir)).width();
    if (width < best_width - kEpsilon) {
      best_width = width;
      best_dir   = dir;
    }
  }

  SPDLOG_INFO("扫描方向 (最小宽度边方向): ({:.4f}, {:.4f}), 垂直方向宽度: {:.3f} m", best_dir.x(), best_dir.y(), best_width);
  return best_dir;
}

// ============================================================
// 扫描线 ∩ 区域 -> 垄线段
// ============================================================
std::vector<Segment_2> PlanBowShapedPath::makeRidgeSegments(const Polygon_2& region, const Point_2& sweep_dir, const Point_2& offset_dir, const Segment_2& far_side_edge) {
  std::vector<Segment_2> ridges;
  const RidgeLayout layout = computeRidgeLayout(region, sweep_dir, offset_dir, ridge_width_, first_row_margin_);
  if (layout.row_count == 0) {
    SPDLOG_WARN("内缩区域在垄间距方向上的宽度约为 0, 无法生成垄", field_margin_);
    return ridges;
  }

  SPDLOG_INFO("生成 {} 行 (首行内移 {:.3f} m, 垄宽 {:.3f} m, 区域宽度 {:.3f} m)", layout.row_count, layout.first_offset, ridge_width_, layout.region_width);

  // 扫描线: 沿 sweep_dir 贯穿区域, 两端各留 kLineOverhang, offset 沿 offset_dir。
  const auto clip_row = [&](double offset, bool reverse_dir) {
    return clipLineToRegion(region, sweep_dir * (layout.along.min - kLineOverhang) + offset_dir * offset, sweep_dir * (layout.along.max + kLineOverhang) + offset_dir * offset, sweep_dir, reverse_dir);
  };

  bool reverse_row = false;
  RidgeBuildStats stats;

  for (int k = 0; k < layout.row_count; ++k) {
    const double offset        = layout.across.min + layout.first_offset + static_cast<double>(k) * ridge_width_;
    std::vector<Segment_2> row = clip_row(offset, reverse_row);

    // 末行兜底只在"还没有任何扫描行贴到远端边界"时启用，避免兜底行横穿已排出的行。
    if (row.empty() && k == layout.row_count - 1) {
      if (stats.far_edge_clearance <= first_row_margin_ + 2.0 * kClipperGrid) {
        SPDLOG_WARN("末行与区域没有交点, 但已有扫描行铺到远端边界边 (距它 {:.3f} m <= 兜底行的 {:.3f} m); 不再贴边兜底", stats.far_edge_clearance, first_row_margin_);
      } else {
        row = makeFarEdgeFallbackRow(region, sweep_dir, far_side_edge, first_row_margin_, reverse_row);
        if (!row.empty()) {
          SPDLOG_WARN("使用末行兜底");
          stats.used_fallback = true;
        }
      }
    }
    if (row.empty()) {
      SPDLOG_WARN("第 {} 行 (距近端 {:.3f} m) 与区域没有交点, 该行跳过{}", k, offset - layout.across.min, k == layout.row_count - 1 ? " (兜底后仍为空)" : "");
      continue;
    }
    if (stats.have_row && std::abs(offset - stats.prev_offset) < kMinVertexSpacing) {
      SPDLOG_WARN("第 {} 行与上一行重合 (相距 < 1 µm), 不再重复排", k);
      break;
    }

    ridges.insert(ridges.end(), row.begin(), row.end());
    reverse_row = !reverse_row;   // 下一行反向 -> 弓字形
    stats.record(row, offset, offset_dir, layout.across, first_row_margin_, far_side_edge);
  }

  // 日志
  if (stats.have_row) {
    SPDLOG_INFO(
      "垄数: {} (计划 {} 条; 区域宽度 {:.3f} m, 垄宽 {:.3f} m, 首行余量 {:.3f} m,  首行距近端 {:.3f} m / 末行距远端 {:.3f} m{}",
      ridges.size(),
      layout.row_count,
      layout.region_width,
      ridge_width_,
      layout.first_offset,
      stats.first_margin,
      stats.last_margin,
      stats.used_fallback ? " (末行兜底: 贴远端)" : "");
  }

  return ridges;
}

// 把边展开为顶点序列
std::vector<Point_2> PlanBowShapedPath::flattenRidges(const std::vector<Segment_2>& ridges) {
  std::vector<Point_2> key;
  key.reserve(ridges.size() * 2);
  for (const auto& seg : ridges) {
    key.push_back(seg.first);
    key.push_back(seg.second);
  }
  return key;
}

// ============================================================
// 掉头 (滚动圆弧)
// ============================================================
std::vector<Point_2> PlanBowShapedPath::smoothTurns(const std::vector<Point_2>& key_waypoints, std::vector<uint8_t>* types) {
  if (key_waypoints.size() < 4) {
    if (types)
      types->assign(key_waypoints.size(), kSegWork);   // 只有一条垄: 整条就是行内作业段
    return key_waypoints;                              // 只有一条垄: 没有掉头需要处理
  }

  std::vector<Point_2> result;
  result.reserve(key_waypoints.size());
  result.push_back(key_waypoints.front());
  if (types) {
    types->clear();
    types->reserve(key_waypoints.size());
    types->push_back(kSegWork);
  }

  const auto append = [&](const Point_2& p, uint8_t type) {
    if (!result.empty() && (p - result.back()).norm() <= kMinVertexSpacing)
      return;
    result.push_back(p);
    if (types)
      types->push_back(type);
  };

  // 滑动窗口 [P1,P2,P3,P4]: 生成 P2->P3 这一段的掉头; 每步前进两个点, 于是下一窗口的
  // P1/P2 正好是本窗口的 P3/P4 (垄 i 的末端 -> 垄 i+1 的起点 -> ... )。
  size_t i = 0;
  while (i + 3 < key_waypoints.size()) {
    const std::vector<Point_2> turn = legacyTurn(key_waypoints[i], key_waypoints[i + 1], key_waypoints[i + 2], key_waypoints[i + 3]);
    output_.turn_max_overrun        = std::max(output_.turn_max_overrun, maxOverrunOf(turn));
    if (!turn.empty()) {
      // T1 仍在当前垄上, 保持 WORK; 若与上一个点重合, append 会跳过。
      append(turn.front(), kSegWork);

      if (turn.size() == 2) {
        // 退化直线兜底 {P2,P3}: 中间插一个 TURN 点, 让两段连接线都被识别为掉头段,
        // 同时保留 P2/P3 为 WORK, 不污染两侧的垄线作业段。
        const Point_2 mid = 0.5 * (turn.front() + turn.back());
        if ((mid - turn.front()).norm() > kMinVertexSpacing && (mid - turn.back()).norm() > kMinVertexSpacing) {
          append(mid, kSegTurn);
        }
        append(turn.back(), kSegWork);
      } else {
        // 非平行半圆的出口 t2 可能不在下一条垄起点 P3 上。此时把 t2 标 TURN,
        // 让 t2 -> 下一个切点 T1' 的直线连接段被识别为掉头段; 不改变路径几何。
        const bool has_exit_connector = (turn.back() - key_waypoints[i + 2]).norm() > kMinVertexSpacing;
        for (size_t k = 1; k + 1 < turn.size(); ++k)
          append(turn[k], kSegTurn);
        append(turn.back(), has_exit_connector ? kSegTurn : kSegWork);
      }
    }
    i += 2;
  }

  append(key_waypoints.back(), kSegWork);
  return result;
}

// ============================================================
// inner_path 两端补到区域顶点 (贴边行走的那一小段)
// ============================================================
// 垄端点由"扫描线 ∩ 区域"裁得, 本来就落在区域边界边上; 于是:
//   * 起点 P1 在边 AB 上: 看 ∠(A, P1, P2) —— > 90° 的那个顶点在车头"背后"(该边相对垄向往回斜),
//     把它放到 P1 前面: 车先从顶点沿这条边走到 P1 再拐进垄, 端头那块三角带才走到, 且那处拐角
//     (180° - ∠) < 90°;
//   * 终点 Pn 在边 CD 上: 同理看 ∠(P_{n-1}, Pn, C) —— > 90° 的顶点在车头"前方", 接在 Pn 后面
//     继续走到它, 正好收边。
// 两个条件都不满足 (矩形地块: 边界边 ⊥ 垄向, 夹角正好 = 90°; 或端点没贴在边界边上) 就原样返回 ——
// 那些情形与旧版逐位相同。
std::vector<Point_2> PlanBowShapedPath::extendEndsToRegionCorners(
  const std::vector<Point_2>& path,
  const Segment_2& near_side_edge,
  const Segment_2& far_side_edge,
  const std::vector<uint8_t>* in_types,
  std::vector<uint8_t>* out_types) {
  // 逐点类型: 与返回的路径保持同步 (发布时写入 Path.waypoint_types)。先按输入的路径初始化,
  // 后面的早退分支 (原样返回) 也就自动是对的; 长度对不上 (不是同一条路径) 时按 WORK 起, 免得把
  // 错位的标签传下去。
  if (out_types != nullptr) {
    if (in_types != nullptr && in_types->size() == path.size())
      *out_types = *in_types;
    else
      out_types->assign(path.size(), kSegWork);
  }
  if (path.size() < 2)
    return path;
  // 退化地块 (取不到有效的 AB/CD) 时原样返回
  if ((near_side_edge.second - near_side_edge.first).norm() < kMinVertexSpacing || (far_side_edge.second - far_side_edge.first).norm() < kMinVertexSpacing)
    return path;

  std::vector<Point_2> out = path;
  tryExtendPathEnd({"首端 (AB)", near_side_edge, 0, path[0] - path[1], true}, path, out, out_types);
  tryExtendPathEnd({"末端 (CD)", far_side_edge, path.size() - 1, path.back() - path[path.size() - 2], false}, path, out, out_types);
  return out;
}

bool PlanBowShapedPath::tryExtendPathEnd(const EndExtensionSpec& spec, const std::vector<Point_2>& path, std::vector<Point_2>& out, std::vector<uint8_t>* out_types) const {
  if (spec.end_index >= path.size()) {
    return false;
  }
  // 与旧版一致: 端头首段 (或末段) 退化时不补角, 否则搜索方向无法可靠归一化。
  if (!(spec.search_dir.norm() > kMinVertexSpacing)) {
    return false;
  }
  const Point_2& end = path[spec.end_index];
  if (!pointOnSegment(end, spec.edge)) {
    return false;
  }

  Point_2 corner;
  if (!pickEdgeCorner(end, spec.search_dir, spec.edge, corner)) {
    return false;
  }

  const size_t others = countOtherPointsOnEdge(path, spec.edge, end);
  if (others > 0) {
    SPDLOG_INFO("{}: 那条边上还有 {} 个路径点 (不是端头边), 不补 —— 前提是只有 path 的起点/终点在 AB/CD 上", spec.label, others);
    return false;
  }

  const Segment_2 extension = spec.at_front ? Segment_2{corner, end} : Segment_2{end, corner};
  if (segmentCrossesPath(extension, path)) {
    SPDLOG_INFO("{}顶点 ({:.3f}, {:.3f}) 在 {:.2f} m 外且那段会横穿已铺路径: 不补 (只补不穿)", spec.label, corner.x(), corner.y(), (corner - end).norm());
    return false;
  }

  SPDLOG_INFO("{}沿区域边界边补到顶点: 垄端 ({:.3f}, {:.3f}) -> 顶点 ({:.3f}, {:.3f}), 补 {:.3f} m", spec.label, end.x(), end.y(), corner.x(), corner.y(), (corner - end).norm());
  if (spec.at_front) {
    out.insert(out.begin(), corner);
    if (out_types != nullptr) {
      out_types->insert(out_types->begin(), kSegWork);
    }
  } else {
    out.push_back(corner);
    if (out_types != nullptr) {
      out_types->push_back(kSegWork);
    }
  }
  return true;
}

// 计算点超过地块边界的量
double PlanBowShapedPath::overrunOf(const Point_2& p) {
  if (field_.size() < 3)
    return 0.0;   // 没有地块信息时不限制 (理论上不会发生)
  const Point_2 inner = vertexMean(field_);
  double worst        = 0.0;
  for (size_t i = 0; i < field_.size(); ++i) {
    const Point_2 a   = field_[i];
    const Point_2 b   = field_[(i + 1) % field_.size()];
    const Point_2 dir = (b - a).normalized();
    const Point_2 n   = inwardNormal(dir, (a + b) * 0.5, inner);
    worst             = std::max(worst, -(p - a).dot(n));
  }
  return worst;
}

// 计算边超过地块边界的量
double PlanBowShapedPath::maxOverrunOf(const std::vector<Point_2>& pts) {
  double worst = 0.0;
  for (const auto& p : pts)
    worst = std::max(worst, overrunOf(p));
  return worst;
}

void PlanBowShapedPath::updateTurnPocket(double pocket, double margin_hint) {
  output_.turn_max_pocket         = pocket;
  output_.turn_pocket_margin_hint = margin_hint;
}

// 按弧长和单步转角上限采样一段圆弧。
std::vector<Point_2> PlanBowShapedPath::sampleCircularArc(const Point_2& center, double radius, double start_angle, double sweep, const Point_2& t1, const Point_2& t2) const {
  const int by_len = static_cast<int>(std::ceil(std::abs(sweep) * radius / std::max(path_resolution_, 0.01)));
  const int by_ang = static_cast<int>(std::ceil(std::abs(sweep) / kMaxArcStepRad));
  const int steps  = std::max(2, std::max(by_len, by_ang));
  std::vector<Point_2> arc;
  arc.reserve(static_cast<size_t>(steps) + 1);
  arc.push_back(t1);
  for (int k = 1; k < steps; ++k) {
    const double a = start_angle + sweep * static_cast<double>(k) / static_cast<double>(steps);
    arc.push_back(center + radius * Point_2(std::cos(a), std::sin(a)));
  }
  arc.push_back(t2);
  return arc;
}

// ============================================================
// 掉头几何: 输入校验 + 两条垄的法向、半径
// ============================================================
std::optional<PlanBowShapedPath::TurnGeometry> PlanBowShapedPath::makeTurnGeometry(const Point_2& P1, const Point_2& P2, const Point_2& P3, const Point_2& P4) const {
  if ((P2 - P1).norm() < kMinVertexSpacing || (P3 - P2).norm() < kMinVertexSpacing) {
    SPDLOG_WARN("垄端间距过小: ({:.3f},{:.3f})->({:.3f},{:.3f}) 或 ({:.3f},{:.3f})->({:.3f},{:.3f}), 忽略掉头", P1.x(), P1.y(), P2.x(), P2.y(), P2.x(), P2.y(), P3.x(), P3.y());
    return std::nullopt;
  }

  const Point_2 in_dir = (P2 - P1).normalized();
  Point_2 n            = Point_2(-in_dir.y(), in_dir.x());
  const double lat     = (P3 - P2).dot(n);
  if (std::abs(lat) < kMinVertexSpacing) {
    SPDLOG_WARN("两条垄几乎共线: ({:.3f},{:.3f})->({:.3f},{:.3f}) 与 ({:.3f},{:.3f})->({:.3f},{:.3f}), 忽略掉头", P1.x(), P1.y(), P2.x(), P2.y(), P2.x(), P2.y(), P3.x(), P3.y());
    return std::nullopt;
  }
  if (lat < 0.0) {
    n = -n;
  }

  const Point_2 out_dir = ((P4 - P3).norm() > kMinVertexSpacing) ? (P4 - P3).normalized() : -in_dir;
  Point_2 n_cd          = perpendicular(out_dir);
  if (n_cd.dot(P2 - P3) < 0.0) {
    n_cd = -n_cd;
  }
  const Point_2 n_ab  = n;
  const double d_b_cd = (P2 - P3).dot(n_cd);
  const double d_c_ab = (P3 - P2).dot(n_ab);
  const double radius = 0.5 * std::min(d_b_cd, d_c_ab);
  if (!(radius > kTurnCornerMinRadius) || !std::isfinite(radius)) {
    SPDLOG_WARN("半径非法: ({:.3f},{:.3f})->({:.3f},{:.3f}) 与 ({:.3f},{:.3f})->({:.3f},{:.3f}), 忽略掉头", P1.x(), P1.y(), P2.x(), P2.y(), P2.x(), P2.y(), P3.x(), P3.y());
    return std::nullopt;
  }

  return TurnGeometry{P2, P3, in_dir, n_ab, radius};
}

// ============================================================
// 所有掉头: 半圆，必要时沿垄向外推
// ============================================================
std::vector<Point_2> PlanBowShapedPath::buildHalfCircleTurn(const TurnGeometry& geometry) {
  const Point_2& P2  = geometry.p2;
  const Point_2& P3  = geometry.p3;
  const Point_2& n   = geometry.n_ab;
  const double cross = geometry.in_dir.x() * n.y() - geometry.in_dir.y() * n.x();
  const double sweep = (cross > 0.0) ? M_PI : -M_PI;

  const auto build_arc = [&](const Point_2& center) {
    const Point_2 t1 = center - geometry.radius * n;
    const Point_2 t2 = center + geometry.radius * n;
    const double a0  = std::atan2(t1.y() - center.y(), t1.x() - center.x());
    return sampleCircularArc(center, geometry.radius, a0, sweep, t1, t2);
  };

  const Point_2 base_center = 0.5 * (P2 + P3) - geometry.radius * geometry.in_dir;
  const double push         = 0.5 * std::abs((P3 - P2).dot(geometry.in_dir));
  double used_push          = 0.0;
  std::vector<Point_2> arc  = build_arc(base_center);
  if (push > kEpsilon) {
    for (int step = kTurnOutwardSteps; step >= 1; --step) {
      const double candidate_push    = push * static_cast<double>(step) / static_cast<double>(kTurnOutwardSteps);
      std::vector<Point_2> candidate = build_arc(base_center + candidate_push * geometry.in_dir);
      if (maxOverrunOf(candidate) <= kTurnOverrunTolerance) {
        arc       = std::move(candidate);
        used_push = candidate_push;
        break;
      }
    }
  }

  const double reach   = (base_center + used_push * geometry.in_dir).dot(geometry.in_dir) + geometry.radius;
  const double far_end = std::max(P2.dot(geometry.in_dir), P3.dot(geometry.in_dir));
  const double pocket  = std::max(0.0, far_end - reach);
  if (pocket > output_.turn_max_pocket) {
    updateTurnPocket(pocket, field_margin_ + maxOverrunOf(build_arc(base_center + push * geometry.in_dir)));
  }
  return arc;
}

std::vector<Point_2> PlanBowShapedPath::legacyTurn(const Point_2& P1, const Point_2& P2, const Point_2& P3, const Point_2& P4) {
  const auto geometry = makeTurnGeometry(P1, P2, P3, P4);
  if (!geometry) {
    return {P2, P3};
  }
  return buildHalfCircleTurn(*geometry);
}

// ============================================================
// 主流程 (generateWaypoints): 顶点准备 -> 内缩 -> 扫描方向 -> 垄线 -> 掉头/插值 -> 端头补角 -> 汇总
// ============================================================
// 不需要旧版开头那段"清空上一轮结果与观测值": plan() 一进来就把入参落到成员上, 并把 output_ 重置
// 成初值 (那一段是给"同一个对象重复调用"做幂等的, 现在由 plan() 开头那几行保证)。
void PlanBowShapedPath::generateWaypoints() {

  // 1. 由输入地块构造有效凸多边形 (失败时内部已报错)
  Polygon_2 poly;
  if (!preparePolygon(cell_, poly))
    return;
  field_ = poly;   // 掉头"不许出界"的判据 (凸地块)

  // 2. 先内缩 (四周统一按 field_margin_): 后面所有几何都在这个"可作业区域"上做 ----
  Polygon_2 region = insetPolygon(poly);
  if (region.size() < 3)
    return;
  output_.inner_polygon = region;

  // 3. 再定扫描方向 (在**内缩区域**上取"垂直方向宽度最小的边方向") ----
  const Point_2 sweep_dir = minWidthDirection(region);
  output_.sweep_dir_deg   = std::atan2(sweep_dir.y(), sweep_dir.x()) * 180.0 / M_PI;

  // 4. 垄间距方向: 垂直于垄向并指向区域内部 (凸区域顶点均值必在内部)。
  Point_2 offset_dir = perpendicular(sweep_dir);
  if ((vertexMean(region) - region.front()).dot(offset_dir) < 0.0)
    offset_dir = -offset_dir;

  // 5. 边界边: CD = 末行兜底要贴的**远端边界边** (farSideEdge); AB = **近端边界边** (nearSideEdge, 步骤 8 用)
  const Segment_2 far_side_edge  = farSideEdge(region, offset_dir);
  const Segment_2 near_side_edge = nearSideEdge(region, offset_dir);

  // 6. 垄线段 (牛耕法: 扫描线 ∩ 区域; 末行兜底用 far_side_edge)
  const std::vector<Segment_2> ridges = makeRidgeSegments(region, sweep_dir, offset_dir, far_side_edge);
  if (ridges.empty()) {
    SPDLOG_ERROR("内缩区域里生成不出垄线 (ridge_width_ {:.3f} m, field_margin_ {:.3f} m)", ridge_width_, field_margin_);
    if (output_.ok())
      fail(PlanStatus::kBadRidgeWidth, "ridge_width 不合理: 内缩区域里生成不出垄线 (区域太窄, 或扫描线裁剪后为空); 垄宽 " + std::to_string(ridge_width_) + " m");
    return;
  }

  // 7. 内区弓字形: 掉头固定用半圆/直线几何 (legacyTurn) —— 这里**不插值**, 统一在拼接后插值一次。
  output_.key_waypoints = flattenRidges(ridges);
  for (size_t i = 0; i < output_.key_waypoints.size(); ++i) {
    SPDLOG_INFO("关键点[{}]: x = {:.4f}, y = {:.4f}", i, output_.key_waypoints[i].x(), output_.key_waypoints[i].y());
  }
  std::vector<uint8_t> inner_types;
  const std::vector<Point_2> inner_path = smoothTurns(output_.key_waypoints, &inner_types);

  // 8. 两端补到区域顶点 (见 extendEndsToRegionCorners):
  //    对于 ABCD 的 region (AB = near_side_edge 近端边界边, CD = far_side_edge 远端边界边),
  //    检查 inner_path (P1 P2 ... Pn) 的起点/终点是否贴在 AB / CD 上, 不在就跳过 (原样);
  //    在的话 —— 起点: 看 ∠(A, P1, P2)、∠(B, P1, P2), 把 > 90° 的那个顶点 (A/B) 放到 P1 前面
  //    (车先从顶点沿 AB 走到 P1, 端头那块三角带才走到); 终点: 看 ∠(P_{n-1}, Pn, C)、∠(P_{n-1}, Pn, D),
  //    把 > 90° 的那个顶点 (C/D) 接在 Pn 后面 (继续走到它, 正好收边)。
  std::vector<uint8_t> inner_types_ends;   // 与 inner_path_ends 1:1 的类型 (补进去的顶点 = WORK, 见 extendEndsToRegionCorners)
  const std::vector<Point_2> inner_path_ends = extendEndsToRegionCorners(inner_path, near_side_edge, far_side_edge, &inner_types, &inner_types_ends);

  // 9. 采样: 结构路径按 path_resolution 插值成 /path。
  {
    output_.waypoints.clear();
    output_.waypoint_types.clear();
    const auto& pts       = inner_path_ends;
    const bool types_ok   = (inner_types_ends.size() == pts.size());
    const auto point_type = [&](size_t i) -> uint8_t {
      return types_ok ? inner_types_ends[i] : kSegWork;
    };
    const auto segment_type = [&](size_t i) -> uint8_t {
      if (!types_ok)
        return kSegWork;
      return (inner_types_ends[i] == kSegWork && inner_types_ends[i + 1] == kSegWork) ? kSegWork : kSegTurn;
    };
    const auto emit = [this](const Point_2& p, uint8_t type) {
      output_.waypoints.push_back(p);
      output_.waypoint_types.push_back(type);
    };

    if (!pts.empty()) {
      if (pts.size() < 2 || !(path_resolution_ > kEpsilon)) {
        for (size_t i = 0; i < pts.size(); ++i)
          emit(pts[i], point_type(i));
      } else {
        output_.waypoints.reserve(pts.size() * 2);
        output_.waypoint_types.reserve(pts.size() * 2);
        emit(pts.front(), point_type(0));
        bg::model::linestring<Point_2> segment;
        segment.resize(2);
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
          const uint8_t seg_type = segment_type(i);
          segment[0]             = pts[i];
          segment[1]             = pts[i + 1];
          const double length    = bg::length(segment);
          if (length > path_resolution_) {
            for (double d = path_resolution_; d < length - kMinVertexSpacing; d += path_resolution_) {
              Point_2 p;
              bg::line_interpolate(segment, d, p);
              emit(p, seg_type);
            }
          }
          emit(pts[i + 1], point_type(i + 1));
        }
      }
    }
  }

  // 按掉头出界量告警 (阈值 5 cm)
  if (output_.turn_max_overrun > 0.05) {
    SPDLOG_WARN("掉头最多有 {:.2f} m 伸出地块边界 (该处的垄端外侧没有留出余量)。", output_.turn_max_overrun);
  }

  //  斜地头处掉头够不到"靠外那个垄端"留下的盲带 (垄端沿垄向错开越多越宽):
  //  半圆掉头的顶点已经推到了"还不伸出地块边界"的最外位置, 剩下的只能靠加大 field_margin_
  //  (把内缩区域收小, 让半圆有地方伸出去) 或减小垄宽 (半圆半径跟着变小) 来消。
  if (output_.turn_max_pocket > 0.05) {
    SPDLOG_WARN(
      "斜地头处每个掉头最多有 {:.2f} m 宽的带作业不到 (半圆掉头半径 {:.2f} m, 够不到沿垄向靠外的那个垄端)。"
      "想盖住: 把 field_margin 加大到 >= {:.2f} m (当前 {:.2f} m)。",
      output_.turn_max_pocket,
      0.5 * ridge_width_,
      output_.turn_pocket_margin_hint,
      field_margin_);
  }

  // 汇总输出本次规划结果
  SPDLOG_INFO(
    "扫描方向 {:.1f}°: {} 条垄; 掉头 {} 次 (半圆, 半径 {:.3f} m = 垄距/2), 掉头最大出界 {:.2f} m",
    output_.sweep_dir_deg,
    ridges.size(),
    ridges.size() > 0 ? ridges.size() - 1 : 0,
    0.5 * ridge_width_,
    output_.turn_max_overrun);
}

// ============================================================
// 公开接口
// ============================================================
// 规划入口。状态在对象里 (见 bow.hpp 的私有成员), 所以这是**非 const** 方法: 一个实例同一时刻只
// 用于一次规划 (本节点按单线程执行器跑: 没有工作线程也没有锁 —— 见 global_path_planner.cpp 里
// updatePathFromInputs / planGeometry 的并发说明); 不同实例之间没有共享
// 状态, 所以"每线程一个实例"可以并行 (见 test/test_bow_parallel.cpp)。
// 结果整包返回, 对象上不导出 getter。
BowOutput PlanBowShapedPath::plan(const BowInput& in) {
  // 入参落到成员上 (类的状态); 几何步骤只读这些成员, **绝不回写调用方传进来的 in**。
  cell_             = in.cell;
  ridge_width_      = in.ridge_width;
  path_resolution_  = in.path_resolution;
  field_margin_     = in.field_margin;
  first_row_margin_ = in.first_row_margin;
  output_           = BowOutput{};   // 本次调用的输出/观测值 (成员只作暂存, 不对外暴露)

  // 首行余量: 下界 kMinFirstRowMargin (Clipper2 量化网格 ≈1 mm, 更小会被量化吃掉), 上界 垄宽/2
  // (超过就破了近端覆盖契约 —— 首行离边界太近, 机具半幅盖不住田边那侧)。越界只告警并夹取,
  // 与 field_margin 同一套"不让参数把规划弄失败"的约定。
  // NaN 不必单独兜: std::max/min 的比较对 NaN 恒为 false, 会停在左侧实参 (先夹到下界, 再由
  // min 保持), 结果落在合法域里; 上面那个"值变了就告警"照样会报出来。
  first_row_margin_ = std::max(kMinFirstRowMargin, in.first_row_margin);
  if (ridge_width_ > kEpsilon)
    first_row_margin_ = std::min(first_row_margin_, 0.5 * ridge_width_);
  if (first_row_margin_ != in.first_row_margin) {
    SPDLOG_WARN("first_row_margin 不合理: {} (取值域 [{:.3f}, 垄宽/2 = {:.3f}] m), 本次按 {:.3f} m 处理", in.first_row_margin, kMinFirstRowMargin, 0.5 * ridge_width_, first_row_margin_);
  }

  // 参数校验: 垄宽必须为正 —— 否则生成垄线时要除以它, 而且根本排不出垄线。
  // 刻意放在**入口**一次, 而不是让 makeRidgeSegments 在里面重复设置状态: 几何函数只管
  // "能不能生成出垄线" (排不出就返回空), 失败原因由调用方兜底。
  if (!(ridge_width_ > kEpsilon)) {
    SPDLOG_ERROR("ridge_width 不合理: {:.3f} m (必须 > 0), 无法生成垄线", ridge_width_);
    fail(PlanStatus::kBadRidgeWidth, "ridge_width 不合理 (" + std::to_string(ridge_width_) + " m, 必须 > 0), 无法生成垄线");
    return std::move(output_);
  }

  generateWaypoints();
  if (output_.waypoints.empty() && output_.ok())
    fail(PlanStatus::kInternal, "规划流程结束但路点为空 (未知原因)");   // 兜底
  return std::move(output_);
}
