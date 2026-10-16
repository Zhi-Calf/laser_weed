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

// Clipper2 量化网格的边长 (m, ≈1 mm)
inline constexpr double kClipperGrid = 1e-3;

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

// 两条线段是否**严格相交** (互相穿过) —— 端点相触、共线重叠都不算。
// 自交/横穿判据统一用这个含义: "走过的垄端正好落在补出来的这一段上"是 T 型路口的正常事。
//
// ⚠ 判据必须是**严格异号** (`o1*o2 < 0`), 不要写成 `o1*o2 < kEpsilon`: kEpsilon = 1e-9 > 0, 于是
// 叉积为 0 的三种情形 (共享端点 / T 型路口 / 共线重叠) 都会被判成"相交", 与上面这句注释相反。
// 唯一调用点 (bow.cpp 的 crosses_path) 测的那一段**总是**与路径首段/末段共享端点 (补角段的一端
// 就是路径端点), 所以带正容差的写法会**永远返回 true** —— 步骤 8 的补角因此从未插入过任何顶点,
// 日志还把"共享端点"报成"会横穿已铺路径", 把排查引向错误方向。
// 实测 (123 个地块, 见 bow.cpp 的 [PATH_END] 日志): 修成严格异号前 17 次拒绝里 15 次只命中共享
// 端点那一段 (纯误报); 修后 14 个地块的补角正常生效、路径 0 处自交, 那 2 处真穿越 (例如三角形
// 60x20 上 18.8 m 的沿边段穿 6 段掉头弧) 仍被挡住。
inline bool properlyCrosses(const Segment_2& s, const Segment_2& t) {
  const auto orient = [](const Point_2& a, const Point_2& b, const Point_2& c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
  };
  const double o1 = orient(s.first, s.second, t.first);
  const double o2 = orient(s.first, s.second, t.second);
  const double o3 = orient(t.first, t.second, s.first);
  const double o4 = orient(t.first, t.second, s.second);
  return o1 * o2 < 0.0 && o3 * o4 < 0.0;
}

// ============================================================
// 多边形小工具 (顶点规范化 / 校验 / 距离)
// ============================================================

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
// 这里刻意不用 bg::is_valid: 它内部会构造 box<Point_2>, 与 Eigen "默认不初始化" 的构造
// 组合会触发 -Wmaybe-uninitialized 误报 (编译器无法证明 box 角点已赋值), 刷屏且无意义。
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
