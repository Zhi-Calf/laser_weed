#pragma once

// ---------------------------------------------------------------------------
// bow.cpp 的内部实现细节 (常量 + 几何小工具)。
//
// 全部 inline, 每个 TU 一份定义, 无 ODR 问题; 本头只给包内实现用, 不进 install (不在 include/ 里)。
//
// 用法: #include "bow_detail.hpp" + using namespace bow_detail; (让调用点不必逐处加前缀)。
// ---------------------------------------------------------------------------

#include <clipper2/clipper.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "global_path_planner/bow.hpp"
#include "global_path_planner/eigen_boost_adaptor.hpp"

namespace bg = boost::geometry;
namespace c2 = Clipper2Lib;

namespace bow_detail {

// ============================================================
// 常量
// ============================================================

// Clipper2 的坐标量化精度 (小数位)
inline constexpr int kClipperPrecision = 3;

// Clipper2 量化网格的边长 (m, ≈1 mm)。与首行余量下界表达的是同一个物理尺度，
// 避免两处各写一个 0.001 后悄悄分叉。
inline constexpr double kClipperGrid = kMinFirstRowMargin;

// 相邻顶点最小间距
inline constexpr double kMinVertexSpacing = 1e-6;

// 扫描线两端各超出区域投影范围的长度 (m)
inline constexpr double kLineOverhang = 1.0;

// 掉头圆弧的采样上限 (rad/步)
inline constexpr double kMaxArcStepRad = 5.0 * M_PI / 180.0;

// 相邻两条垄算不算"平行"的判据 (方向叉积 = |sin 夹角|)
inline constexpr double kTurnParallelTol = 0.01;

// 把半圆的顶点从"B-C 中点"往外推、推到"盖住靠外那个垄端"所需的步数
inline constexpr int kTurnOutwardSteps = 20;

// 掉头允许伸出**地块边界**的量 (m)
inline constexpr double kTurnOverrunTolerance = 0.05;

// 两条垄**不平行**时"与两条垄都相切的圆"的最小可用半径 (m)
inline constexpr double kTurnCornerMinRadius = 0.05;

// 浮点数相等: 绝对值差不超过它
inline constexpr double kEpsilon = 1e-9;

// 凸包的最小有效面积 (m^2): 顶点全部共线时凸包退化成一条线 —— Boost 的 convex_hull 可能给出
// 2 个点 (线段), 也可能给出 3 个点 (面积 0 的退化多边形), 两种都要判成"凸包点不足"。
// 1 mm² 这个阈值远小于任何真实地块, 不会误伤。
inline constexpr double kMinHullArea = 1e-6;

// ============================================================
// 小工具: 坐标转换 / 投影 / 向量
// ============================================================

inline c2::PointD toClipper(const Point_2& p) {
  return c2::PointD(p.x(), p.y());
}

inline Point_2 fromClipper(const c2::PointD& p) {
  return Point_2(p.x, p.y);
}

inline c2::PathD toClipperPath(const Polygon_2& poly) {
  c2::PathD path;
  path.reserve(poly.size());
  for (const auto& p : poly)
    path.push_back(toClipper(p));
  return path;
}

inline Polygon_2 fromClipperPath(const c2::PathD& path) {
  Polygon_2 poly;
  poly.reserve(path.size());
  for (const auto& p : path)
    poly.push_back(fromClipper(p));
  return poly;
}

inline Point_2 perpendicular(const Point_2& dir) {
  return Point_2(-dir.y(), dir.x());
}

// 顶点均值 —— 对凸多边形必在内部, 用来判定"内法线朝哪边" (与顶点绕行方向无关)。
inline Point_2 vertexMean(const Polygon_2& poly) {
  Point_2 sum = Point_2::Zero();
  for (const auto& p : poly)
    sum += p;
  return poly.empty() ? sum : sum / static_cast<double>(poly.size());
}

// 多边形在 dir 上的投影范围 [min(p·dir), max(p·dir)]。
struct ProjectionRange {
  double min = 0.0;
  double max = 0.0;
  double width() const {
    return max - min;
  }
};

inline ProjectionRange projectRange(const Polygon_2& poly, const Point_2& dir) {
  ProjectionRange r{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  for (const auto& p : poly) {
    const double d = p.dot(dir);
    r.min          = std::min(r.min, d);
    r.max          = std::max(r.max, d);
  }
  return r;
}

// 一次垄布局的几何参数。
struct RidgeLayout {
  ProjectionRange across;      // 垄间距方向
  ProjectionRange along;       // 垄方向
  double region_width = 0.0;   // 垄间距方向宽度 (m)
  double first_offset = 0.0;   // 首行相对近端边界的偏移 (m)
  int row_count       = 0;     // 计划行数; 0 = 区域退化
};

// 根据区域和垄宽计算行数、首行偏移及两个方向的投影范围。
inline RidgeLayout computeRidgeLayout(const Polygon_2& region, const Point_2& sweep_dir, const Point_2& offset_dir, double ridge_width, double first_row_margin) {
  RidgeLayout layout;
  layout.across       = projectRange(region, offset_dir);
  layout.along        = projectRange(region, sweep_dir);
  layout.region_width = layout.across.width();
  if (layout.region_width <= kEpsilon || !(ridge_width > kEpsilon)) {
    return layout;
  }

  layout.first_offset = std::min(first_row_margin, 0.5 * layout.region_width);
  layout.row_count    = 1;
  while (layout.first_offset + static_cast<double>(layout.row_count - 1) * ridge_width < layout.region_width - 0.5 * ridge_width - kEpsilon) {
    ++layout.row_count;
  }
  return layout;
}

// "远端边界边": 边**中点**沿 offset_dir 投影最大的那条边 —— 兜底行要贴的就是它。
inline Segment_2 farSideEdge(const Polygon_2& poly, const Point_2& offset_dir) {
  const Point_2* best_a = nullptr;
  const Point_2* best_b = nullptr;
  double best           = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < poly.size(); ++i) {
    const Point_2& a = poly[i];
    const Point_2& b = poly[(i + 1) % poly.size()];
    if ((b - a).norm() < kMinVertexSpacing)
      continue;
    const double mid = 0.5 * (a + b).dot(offset_dir);
    if (mid > best) {
      best   = mid;
      best_a = &a;
      best_b = &b;
    }
  }
  if (best_a == nullptr || best_b == nullptr)
    return Segment_2{Point_2::Zero(), Point_2::Zero()};
  return Segment_2{*best_a, *best_b};
}

// "近端边界边 AB": 边**中点**沿 offset_dir 投影最小的那条边 —— 首行贴的就是它 (步骤 8 的 AB)。
inline Segment_2 nearSideEdge(const Polygon_2& poly, const Point_2& offset_dir) {
  const Point_2* best_a = nullptr;
  const Point_2* best_b = nullptr;
  double best           = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poly.size(); ++i) {
    const Point_2& a = poly[i];
    const Point_2& b = poly[(i + 1) % poly.size()];
    if ((b - a).norm() < kMinVertexSpacing)
      continue;
    const double mid = 0.5 * (a + b).dot(offset_dir);
    if (mid < best) {
      best   = mid;
      best_a = &a;
      best_b = &b;
    }
  }
  if (best_a == nullptr || best_b == nullptr)
    return Segment_2{Point_2::Zero(), Point_2::Zero()};
  return Segment_2{*best_a, *best_b};
}

// 内法线 (指向多边形内部): 由边方向 + "该边中点到内部点"的方向决定。
inline Point_2 inwardNormal(const Point_2& edge_dir, const Point_2& edge_mid, const Point_2& inner_point) {
  Point_2 n = perpendicular(edge_dir).normalized();
  if ((inner_point - edge_mid).dot(n) < 0.0)
    n = -n;
  return n;
}

// Clipper2 的裁剪结果里取面积最大的一条 (理论上只会有一条)。
inline const c2::PathD& largestPath(const c2::PathsD& paths) {
  size_t best      = 0;
  double best_area = -1.0;
  for (size_t i = 0; i < paths.size(); ++i) {
    const double a = std::abs(c2::Area(paths[i]));
    if (a > best_area) {
      best_area = a;
      best      = i;
    }
  }
  return paths[best];
}

// 用区域裁剪一条无限延伸方向上的直线段，返回落在区域内的开路径段。
// reverse_dir=true 时，返回段的顺序和每段方向都反转，供弓字形换向使用。
inline std::vector<Segment_2> clipLineToRegion(const Polygon_2& region, const Point_2& line_begin, const Point_2& line_end, const Point_2& dir, bool reverse_dir) {
  std::vector<Segment_2> row;
  c2::ClipperD clipper(kClipperPrecision);
  clipper.AddOpenSubject(c2::PathsD{c2::PathD{toClipper(line_begin), toClipper(line_end)}});
  clipper.AddClip(c2::PathsD{toClipperPath(region)});
  c2::PathsD closed_paths, open_paths;
  if (!clipper.Execute(c2::ClipType::Intersection, c2::FillRule::NonZero, closed_paths, open_paths) || open_paths.empty())
    return row;

  row.reserve(open_paths.size());
  for (const auto& path : open_paths) {
    if (path.size() < 2)
      continue;
    Segment_2 seg{fromClipper(path.front()), fromClipper(path.back())};
    if ((seg.second - seg.first).norm() < kMinVertexSpacing)
      continue;
    // Clipper2 不保证开路径顶点顺序与 subject 同向，这里统一到 +dir。
    if ((seg.second - seg.first).dot(dir) < 0.0)
      std::swap(seg.first, seg.second);
    row.push_back(seg);
  }

  std::sort(row.begin(), row.end(), [&dir](const Segment_2& s1, const Segment_2& s2) {
    return s1.first.dot(dir) < s2.first.dot(dir);
  });
  if (reverse_dir) {
    std::reverse(row.begin(), row.end());
    for (auto& seg : row)
      std::swap(seg.first, seg.second);
  }
  return row;
}

// 一行路径（可能由多段组成）到目标边界线段的最短距离。
inline double rowDistanceToSegment(const std::vector<Segment_2>& row, const Segment_2& target) {
  const bg::model::segment<Point_2> target_seg(target.first, target.second);
  double best = std::numeric_limits<double>::max();
  for (const auto& seg : row)
    best = std::min(best, bg::distance(bg::model::segment<Point_2>(seg.first, seg.second), target_seg));
  return best;
}

// 沿远端边界边的内法线退 first_row_margin 生成兜底行；失败返回空。
inline std::vector<Segment_2> makeFarEdgeFallbackRow(const Polygon_2& region, const Point_2& sweep_dir, const Segment_2& far_side_edge, double first_row_margin, bool reverse_row) {
  if ((far_side_edge.second - far_side_edge.first).norm() < kMinVertexSpacing) {
    return {};
  }

  Point_2 dir = (far_side_edge.second - far_side_edge.first).normalized();
  if (dir.dot(sweep_dir) < 0.0) {
    dir = -dir;
  }
  const Point_2 in = inwardNormal(dir, 0.5 * (far_side_edge.first + far_side_edge.second), vertexMean(region));
  Point_2 p0       = far_side_edge.first + first_row_margin * in;
  Point_2 p1       = far_side_edge.second + first_row_margin * in;
  if ((p1 - p0).dot(dir) < 0.0) {
    std::swap(p0, p1);
  }
  return clipLineToRegion(region, p0 - dir * kLineOverhang, p1 + dir * kLineOverhang, dir, reverse_row);
}

// 行循环中的累计状态与首末行诊断。
struct RidgeBuildStats {
  bool used_fallback        = false;
  bool have_row             = false;
  double prev_offset        = 0.0;
  double first_margin       = 0.0;
  double last_margin        = 0.0;
  double far_edge_clearance = std::numeric_limits<double>::max();

  void record(const std::vector<Segment_2>& row, double offset, const Point_2& offset_dir, const ProjectionRange& across, double first_row_margin, const Segment_2& far_side_edge) {
    far_edge_clearance = std::min(far_edge_clearance, rowDistanceToSegment(row, far_side_edge));
    if (used_fallback) {
      prev_offset = 0.5 * (row.front().first + row.front().second).dot(offset_dir);
      last_margin = first_row_margin;
    } else {
      prev_offset = offset;
      last_margin = across.max - offset;
    }
    if (!have_row) {
      first_margin = offset - across.min;
    }
    have_row = true;
  }
};

// 两条线段是否**严格相交** (互相穿过) —— 端点相触、T 型接触、共线重叠都不算。
// 用 Boost.Geometry 判断一般相交，再排除"任一端点落在另一线段上"的情形，保持补角逻辑需要的
// 严格穿越语义：补角段与路径首段/末段共享端点时必须允许插入，而真正的横穿必须拒绝。
inline bool properlyCrosses(const Segment_2& s, const Segment_2& t) {
  const bg::model::segment<Point_2> bs{s.first, s.second};
  const bg::model::segment<Point_2> bt{t.first, t.second};
  if (!bg::intersects(bs, bt)) {
    return false;
  }
  // bg::intersects 也覆盖端点接触 / T 型接触 / 共线重叠；这些不是严格穿越。
  if (bg::intersects(s.first, bt) || bg::intersects(s.second, bt) || bg::intersects(t.first, bs) || bg::intersects(t.second, bs)) {
    return false;
  }
  return true;
}

// 点到线段最近的投影点 (退化线段返回起点)。
inline Point_2 closestPointOnSegment(const Point_2& p, const Segment_2& edge) {
  const Point_2 ab  = edge.second - edge.first;
  const double len2 = ab.squaredNorm();
  if (len2 < kMinVertexSpacing * kMinVertexSpacing) {
    return edge.first;
  }
  const double t = std::max(0.0, std::min(1.0, (p - edge.first).dot(ab) / len2));
  return edge.first + t * ab;
}

// 点是否贴在边界线段上 (容差 = Clipper2 量化网格)。
inline bool pointOnSegment(const Point_2& p, const Segment_2& edge) {
  if ((edge.second - edge.first).norm() < kMinVertexSpacing) {
    return false;
  }
  return (p - closestPointOnSegment(p, edge)).norm() <= kClipperGrid;
}

// 选择边界线段上位于 search_dir 正半平面、且离 p 最近的顶点。
inline bool pickEdgeCorner(
  const Point_2& p, const Point_2& search_dir, const Segment_2& edge, Point_2& out) {
  const double dir_len = search_dir.norm();
  if (!(dir_len > kEpsilon)) {
    return false;
  }
  const Point_2 dir = search_dir / dir_len;
  bool found        = false;
  double best_len   = 0.0;
  for (int k = 0; k < 2; ++k) {
    const Point_2& corner = (k == 0) ? edge.first : edge.second;
    const double len      = (corner - p).norm();
    if (len <= kMinVertexSpacing) {
      continue;
    }
    const double cos_a = (corner - p).dot(dir) / len;
    if (cos_a >= -kEpsilon && (!found || len < best_len)) {
      out      = corner;
      best_len = len;
      found    = true;
    }
  }
  return found;
}

// 边界线段上除 end 之外的路径点数。
inline size_t countOtherPointsOnEdge(
  const std::vector<Point_2>& path, const Segment_2& edge, const Point_2& end) {
  size_t others = 0;
  for (const auto& p : path) {
    if ((p - end).norm() <= kMinVertexSpacing) {
      continue;
    }
    if (pointOnSegment(p, edge)) {
      ++others;
    }
  }
  return others;
}

// 线段是否严格横穿路径中的任意一段。
inline bool segmentCrossesPath(const Segment_2& segment, const std::vector<Point_2>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (properlyCrosses(segment, Segment_2{path[i], path[i + 1]})) {
      return true;
    }
  }
  return false;
}

// 把环的起点旋转到离 target 最近的顶点 (日志/可视化与配置顶点顺序对得上)。
inline void rotateToNearest(Polygon_2& poly, const Point_2& target) {
  if (poly.empty())
    return;
  size_t best   = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poly.size(); ++i) {
    const double d = (poly[i] - target).squaredNorm();
    if (d < best_d) {
      best_d = d;
      best   = i;
    }
  }
  std::rotate(poly.begin(), poly.begin() + static_cast<long>(best), poly.end());
}

// 去掉相邻 (含首尾) 重合的顶点。
inline void removeDuplicateVertices(Polygon_2& poly) {
  Polygon_2 out;
  out.reserve(poly.size());
  for (const auto& p : poly) {
    if (out.empty() || (p - out.back()).norm() > kMinVertexSpacing)
      out.push_back(p);
  }
  while (out.size() > 1 && (out.front() - out.back()).norm() <= kMinVertexSpacing)
    out.pop_back();
  poly = std::move(out);
}

// 用 Boost.Geometry 校验: 不自交 (is_simple) + 面积大于阈值。
inline bool isValidRing(const Polygon_2& poly, double min_area) {
  if (poly.size() < 3)
    return false;

  Polygon_2 ring = poly;
  ring.push_back(poly.front());   // Boost 的环要求首尾闭合
  if (!bg::is_simple(ring))
    return false;
  return std::abs(bg::area(ring)) > min_area;
}

// 把环整理成 Boost 约定的顺时针方向 (逆序输入也能收下)。
inline Polygon_2 normalizedRing(const Polygon_2& poly) {
  bg::model::polygon<Point_2> model;
  model.outer().assign(poly.begin(), poly.end());
  model.outer().push_back(poly.front());
  bg::correct(model);
  Polygon_2 out(model.outer().begin(), model.outer().end() - 1);
  if (out.size() != poly.size())
    out = poly;   // correct() 理论上不动顶点数; 万一动了就用原环 (不冒险)
  return out;
}

}   // namespace bow_detail
