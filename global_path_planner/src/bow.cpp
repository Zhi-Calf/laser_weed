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

  // 凸包点不足: 顶点数 < 3, 或全部共线 (面积 ~0 —— 此时凸包可能退化成 2 点的"线段", 也可能
  // 是 3 点的退化多边形, 两种都算)。
  // 先判它再判"非凸": 地块整体退化成一条线时谈"非凸"没有意义 —— 这种输入要报的是"凸包点不足",
  // 而不是被非凸判据抢先报成"非凸"。
  Polygon_2 ordered(hull.outer().begin(), hull.outer().end() - 1);   // 去掉闭合点
  if (ordered.size() < 3 || std::abs(bg::area(hull)) <= kMinHullArea) {
    SPDLOG_ERROR("凸包点不足: {} 个 (面积 {:.6f} m^2, 顶点退化或共线)", ordered.size(), std::abs(bg::area(hull)));
    fail(PlanStatus::kHullTooFew, "凸包点不足 (" + std::to_string(ordered.size()) + " 个或面积为 0, 地块顶点退化/共线)");
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

  // 起点固定到"最小 x, 再最小 y"的顶点 (而不是跟着输入的第一个顶点)。这样同一地块无论输入
  // 顶点顺序如何, 喂给 Clipper2 的路径完全一致 —— 1 mm 量化下的内缩结果、以及端部补点那种
  // "阈值判定"才是确定的; 否则"顶点顺序无关"会被放大成可见差异。
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
  const c2::PathsD inset = c2::InflatePaths(c2::PathsD{toClipperPath(poly)}, -margin, c2::JoinType::Miter, c2::EndType::Polygon, miter_limit_, kClipperPrecision);
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
  const ProjectionRange across = projectRange(region, offset_dir);   // 垄间距方向
  const ProjectionRange along  = projectRange(region, sweep_dir);    // 垄方向
  const double region_width    = across.width();
  if (region_width <= kEpsilon) {
    SPDLOG_WARN("内缩区域在垄间距方向上的宽度约为 0, 无法生成垄", field_margin_);
    return ridges;
  }

  // 计算行数
  const double spacing = ridge_width_;
  const double first   = std::min(first_row_margin_, 0.5 * region_width);
  int count            = 1;
  while (first + static_cast<double>(count - 1) * spacing < region_width - 0.5 * spacing - kEpsilon)
    ++count;
  SPDLOG_INFO("生成 {} 行 (首行内移 {:.3f} m, 垄宽 {:.3f} m, 区域宽度 {:.3f} m)", count, first, spacing, region_width);

  // 直线与区域求交点
  const auto clip_line = [&](const Point_2& line_begin, const Point_2& line_end, const Point_2& dir, bool reverse_dir) {
    std::vector<Segment_2> row;
    c2::ClipperD clipper(kClipperPrecision);
    clipper.AddOpenSubject(c2::PathsD{c2::PathD{toClipper(line_begin), toClipper(line_end)}});
    clipper.AddClip(c2::PathsD{toClipperPath(region)});
    c2::PathsD closed_paths, open_paths;
    if (!clipper.Execute(c2::ClipType::Intersection, c2::FillRule::NonZero, closed_paths, open_paths) || open_paths.empty())
      return row;

    // 一条线上可能切出多段 —— 凸区域只会有一段; 若地块内有空洞/凹口 (理论上不会
    // 走到这里) 就按位置依次连过去, 路径仍然连续且不漏垄。
    row.reserve(open_paths.size());
    for (const auto& path : open_paths) {
      if (path.size() < 2)
        continue;
      Segment_2 seg{fromClipper(path.front()), fromClipper(path.back())};
      if ((seg.second - seg.first).norm() < kMinVertexSpacing)
        continue;
      // Clipper2 对开路径的顶点顺序不保证跟 subject 同向 —— 这里统一成 +dir, 下面 reverse_dir 的
      // 反向才有确定语义 (否则"本行反向"会随裁剪结果的朝向随机失效, 相邻行可能同向 -> 弓字形断掉)。
      if ((seg.second - seg.first).dot(dir) < 0.0)
        std::swap(seg.first, seg.second);
      row.push_back(seg);
    }

    // 同一行内按沿行方向的位置排序; 该行整体反向时, 段顺序与每段方向一起反过来,
    // 保证行驶方向单调 (弓字形)。
    std::sort(row.begin(), row.end(), [&dir](const Segment_2& s1, const Segment_2& s2) {
      return s1.first.dot(dir) < s2.first.dot(dir);
    });
    if (reverse_dir) {
      std::reverse(row.begin(), row.end());
      for (auto& seg : row)
        std::swap(seg.first, seg.second);
    }
    return row;
  };

  // 扫描线: 沿 sweep_dir 贯穿区域, 两端各留 kLineOverhang, offset 沿 offset_dir。
  const auto clip_row = [&](double offset, bool reverse_dir) {
    return clip_line(sweep_dir * (along.min - kLineOverhang) + offset_dir * offset, sweep_dir * (along.max + kLineOverhang) + offset_dir * offset, sweep_dir, reverse_dir);
  };

  // 这一行离远端边界边有多远 (m, 0 = 已经压在远端边上)。
  const auto distance_to_far_edge = [&](const std::vector<Segment_2>& row) {
    const bg::model::segment<Point_2> far_seg(far_side_edge.first, far_side_edge.second);
    double best = std::numeric_limits<double>::max();
    for (const auto& seg : row)
      best = std::min(best, bg::distance(bg::model::segment<Point_2>(seg.first, seg.second), far_seg));
    return best;
  };

  bool reverse_row          = false;   // 本行相对上一行的行驶方向是否已反向 (弓字形开关)
  bool used_fallback        = false;   // 末行是否走了兜底 (贴远端边界边)
  bool have_row             = false;   // 是否已经排出过至少一行
  double prev_offset        = 0.0;     // 上一行的 offset (去重用; 兜底行记它自己的中点投影)
  double first_margin       = 0.0;     // 实际首行距近端 (m)
  double last_margin        = 0.0;     // 实际末行距远端 (m)
  double far_edge_clearance = std::numeric_limits<double>::max();

  for (int k = 0; k < count; ++k) {
    double offset              = across.min + first + static_cast<double>(k) * spacing;   // 沿 offset_dir 的绝对坐标
    std::vector<Segment_2> row = clip_row(offset, reverse_row);

    // 末行兜底: 末行被推到区域外 (与区域没有交点) 时, 沿传入的**远端边界边** far_side_edge 再排一行 ——
    // 兜底行 = 它沿**自己的内法线**退 first_row_margin_ 的那条线, 再裁到区域里。
    // 方向仍走 clip_line 的 reverse_row 反向规矩 (相邻行反向 -> 弓字形)。
    //
    // **兜底的准入门槛**: 兜底行落在"距远端边界边 first_row_margin_"处, 所以只有**它是所有行里最靠
    // 远端的那一条**时才能补 —— 也就是"has_row 之前排出的行, 没有一条离远端边界边 <= first_row_margin_"
    // (第一条垄本身也满足: 它是距近端 5 mm, 离远端差着整个区域宽度)。含义就是题设那句:
    // **扫描线一直没打到远端边界边** (远端那条带真没人盖) 才允许贴远端兜底。
    // 只要**已经有行打到远端边界边** (斜地块上远端边是斜的, 行被它裁掉、端点正好落在它上面; 实测
    // 13.45x5.32 这一族从斜度 1.5 m 起就会发生, 行的端点到远端边只剩 0.4~1.5 mm), 兜底行必然横穿那条行:
    // 弓字形当场回折、路径与走过的路径相交 (实测该族自交 1 -> 5 处), 这是现场最不能接受的一种退化。
    // 此时宁可远端留一条盲带 (**[COVERAGE]** 会报出来), 也不加那条横穿的行。
    // 容差 = 2 格 Clipper2 量化网格: 行的端点与远端边各自被量化过一次, 误差叠加 (实测 0.4~1.5 mm)。
    // 判据只认**这条边本身**(线段), 不认"远端边的内缩线"(无限长): 后者在斜地块上每条扫描线都相交,
    // 兜底会被整条挡掉 (F2cTurn 的斜梯形/平行四边形实测: 末行缺失、远端 2.4 m 空), 文档里已记为"试过又删了"。
    if (row.empty() && k == count - 1) {
      if (far_edge_clearance <= first_row_margin_ + 2.0 * kClipperGrid) {
        SPDLOG_WARN("末行与区域没有交点, 但已有扫描行铺到远端边界边 (距它 {:.3f} m <= 兜底行的 {:.3f} m); 不再贴边兜底", far_edge_clearance, first_row_margin_);
      } else if ((far_side_edge.second - far_side_edge.first).norm() > kMinVertexSpacing) {
        Point_2 dir = (far_side_edge.second - far_side_edge.first).normalized();
        if (dir.dot(sweep_dir) < 0.0)
          dir = -dir;   // 统一到"与扫描方向同向"的一侧, 本行反向仍由 reverse_row 决定
        const Point_2 in = inwardNormal(dir, 0.5 * (far_side_edge.first + far_side_edge.second), vertexMean(region));
        Point_2 p0       = far_side_edge.first + first_row_margin_ * in;
        Point_2 p1       = far_side_edge.second + first_row_margin_ * in;
        if ((p1 - p0).dot(dir) < 0.0)
          std::swap(p0, p1);   // 边界边的端点顺序跟着区域环, 与 dir 无关 —— 先把 p0 定到 -dir 端
        row = clip_line(p0 - dir * kLineOverhang, p1 + dir * kLineOverhang, dir, reverse_row);
        if (!row.empty()) {
          SPDLOG_WARN("使用末行兜底");
          used_fallback = true;
        }
      }
    }
    if (row.empty()) {
      SPDLOG_WARN("第 {} 行 (距近端 {:.3f} m) 与区域没有交点, 该行跳过{}", k, offset - across.min, k == count - 1 ? " (兜底后仍为空)" : "");
      continue;
    }
    if (have_row && std::abs(offset - prev_offset) < kMinVertexSpacing) {
      SPDLOG_WARN("第 {} 行与上一行重合 (相距 < 1 µm), 不再重复排", k);
      break;
    }

    ridges.insert(ridges.end(), row.begin(), row.end());
    reverse_row        = !reverse_row;                                              // 下一行反向 -> 弓字形
    far_edge_clearance = std::min(far_edge_clearance, distance_to_far_edge(row));   // 这一行离远端边界边还有多远 -> 末行兜底的准入门槛 (见上面兜底处)

    // 记录各行首/末行距近端 / 远端的距离 (兜底行不记它自己的中点投影, 而是记兜底的那个点):
    if (used_fallback) {
      prev_offset = 0.5 * (row.front().first + row.front().second).dot(offset_dir);
      last_margin = first_row_margin_;
    } else {
      prev_offset = offset;
      last_margin = across.max - offset;
    }
    if (!have_row)
      first_margin = offset - across.min;
    have_row = true;
  }

  // 日志
  if (have_row) {
    SPDLOG_INFO(
      "垄数: {} (计划 {} 条; 区域宽度 {:.3f} m, 垄宽 {:.3f} m, 首行余量 {:.3f} m,  首行距近端 {:.3f} m / 末行距远端 {:.3f} m{}",
      ridges.size(),
      count,
      region_width,
      ridge_width_,
      first,
      first_margin,
      last_margin,
      used_fallback ? " (末行兜底: 贴远端)" : "");
  }

  return ridges;
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

  // 滑动窗口 [P1,P2,P3,P4]: 生成 P2->P3 这一段的掉头; 每步前进两个点, 于是下一窗口的
  // P1/P2 正好是本窗口的 P3/P4 (垄 i 的末端 -> 垄 i+1 的起点 -> ... )。
  size_t i = 0;
  while (i + 3 < key_waypoints.size()) {
    const std::vector<Point_2> turn = legacyTurn(key_waypoints[i], key_waypoints[i + 1], key_waypoints[i + 2], key_waypoints[i + 3]);
    output_.turn_max_overrun        = std::max(output_.turn_max_overrun, maxOverrunOf(turn));
    if (!turn.empty()) {
      if ((turn.front() - result.back()).norm() > kMinVertexSpacing) {
        result.push_back(turn.front());
        if (types)
          types->push_back(kSegWork);   // T1 还在垄上 -> 行内作业段
      }
      // 弧的内部点标 TURN; 末点 (T2, 或退化情形的 P3) 仍落在垄上 -> WORK:
      // T2 是下一条垄作业段的起点, 标成 WORK 才能让"T2 -> 下一个切点"这段长直段被认成行内段
      // (否则整条垄都不会被 PTO 路径粗化, 这个产出就白做了)。退化情形 (legacyTurn 返回 {P2,P3})
      // 两端都是垄端点, 同样都是 WORK。
      result.insert(result.end(), turn.begin() + 1, turn.end());
      if (types) {
        for (size_t k = 1; k + 1 < turn.size(); ++k)
          types->push_back(kSegTurn);
        if (turn.size() >= 2)
          types->push_back(kSegWork);
      }
    }
    i += 2;
  }

  result.push_back(key_waypoints.back());
  if (types)
    types->push_back(kSegWork);
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
  // 逐点类型: 与返回的路径保持同步 (只有 PTO 路径用得上 —— 见 buildPtoPath)。先按输入的路径初始化,
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

  // p 是否贴在 edge 上 (点到线段距离 <= Clipper 量化网格 = 1 mm)。
  const auto on_edge = [](const Point_2& p, const Segment_2& edge) {
    const Point_2 ab  = edge.second - edge.first;
    const double len2 = ab.squaredNorm();
    if (len2 < kMinVertexSpacing * kMinVertexSpacing)
      return false;
    const double t = std::max(0.0, std::min(1.0, (p - edge.first).dot(ab) / len2));
    return (p - (edge.first + t * ab)).norm() <= kClipperGrid;
  };

  // 该边两个顶点里挑"夹角 > 90°"的那个: dot((corner - p), search_dir) 的符号 = cos(夹角) 的符号,
  // 所以判据就是"(corner - p) 落在 search_dir 的正半平面里"。search_dir 是**带符号的搜索方向**
  // (调用处已保证非零): 起点取车头方向的**反向** -> 只认"背后"的顶点; 终点取车头方向 -> 只认"前方"
  // 的顶点。两个分支只差一个符号, 所以不必再传 ahead 布尔量 —— 反向 dir 与"取 cos_a <= 0"严格等价:
  // (corner - p)·(-d) 与 -((corner - p)·d) 逐位相同 (IEEE 取负是精确的), -x >= 0 与 x <= 0 在
  // IEEE 下也等价 (含 ±0: -(+0.0) >= 0.0 为真)。即 ahead 是**冗余参数**, 删掉后结果逐位不变。
  //
  // 直角边的浮点容差 (kEpsilon): 边与 search_dir 垂直时, 点积的符号完全由舍入决定 —— 实测矩形
  // 地块 (边界边 ⊥ 垄向) 那个顶点的余弦就是 ±0 (符号位来自乘积里的 0), 只按 `>= 0` 判会让"选哪个
  // 顶点"变成掷硬币: 合成用例里一条 1 m 的边、p 在 0.3 m 处, 搜索方向的倾斜只差 ±1e-17, 选中的
  // 顶点就在 0.3 m 与 0.7 m 两个顶点之间翻 (差 0.4 m), 还可能把更近的那个否掉、反而选远的。所以:
  //   * 先把 search_dir 归一化, 判据换成**夹角余弦** —— 点积的量纲是 m², 拿长度量纲的 kEpsilon 直接
  //     比会随垄长/距离漂移 (本文件 kTurnParallelTol 用 |sin 夹角|, 也是这套无量纲角度口径);
  //   * |cos| <= kEpsilon 视为满足 -> "直角"归到下面"两个都满足取最近"的兜底上, 与舍入无关。
  //     kEpsilon = 1e-9 ≈ 离直角 1e-9 rad (6e-8 °): 比实测舍入抖动 (1e-17 ~ 1e-16) 高 7 个量级,
  //     又比 1 mm 量化在短垄上造成的**确定性**倾斜 (实测 5e-4 rad 量级, 如 P1-P2 只有 0.28 m 时)
  //     低 5 个量级 —— 那种倾斜可复现、不是噪声, 仍按真实夹角处理。
  // 万一两个都满足 (直角边, 或端点被 1 mm 量化挪到边外一点), 取离 p 近的那个; 长度打平时保留
  // edge.first (遍历顺序固定, 结论同样确定)。
  const auto pick_corner = [](const Point_2& p, const Point_2& search_dir, const Segment_2& edge, Point_2& out) {
    const double dir_len = search_dir.norm();
    if (!(dir_len > kEpsilon))
      return false;
    const Point_2 dir = search_dir / dir_len;
    bool found        = false;
    double best_len   = 0.0;
    for (int k = 0; k < 2; ++k) {
      const Point_2& corner = (k == 0) ? edge.first : edge.second;
      const double len      = (corner - p).norm();
      if (len <= kMinVertexSpacing)
        continue;
      const double cos_a = (corner - p).dot(dir) / len;   // dir 已归一化, 再除 len -> 就是夹角余弦
      if (cos_a >= -kEpsilon && (!found || len < best_len)) {
        out      = corner;
        best_len = len;
        found    = true;
      }
    }
    return found;
  };

  std::vector<Point_2> out = path;

  // 前提: 那条边上**只有 path 的这一个端点** (它才是"端头边 AB/CD")。斜边很长时这条边会一路
  // 穿过其它垄的端点 (那些点也贴在区域边界上), 补过去等于"沿整条边再开一遍" —— 与"只有一个交点"
  // 是同一件事。
  const auto count_other_points_on = [&path](const Segment_2& edge, const Point_2& end) {
    const Point_2 ab  = edge.second - edge.first;
    const double len2 = ab.squaredNorm();
    size_t others     = 0;
    for (const auto& p : path) {
      if ((p - end).norm() <= kMinVertexSpacing)
        continue;   // 端点自己
      const double t = (len2 < kMinVertexSpacing * kMinVertexSpacing) ? 0.0 : std::max(0.0, std::min(1.0, (p - edge.first).dot(ab) / len2));
      if ((p - (edge.first + t * ab)).norm() <= kClipperGrid)
        ++others;
    }
    return others;
  };

  // 准入判据: ① 这条边上只有这一个路径端点 (前提); ② 补出来的那一段与已铺路径只在端点相触
  // (T 型路口), 不严格相交。斜边很长时 >90° 的顶点会在边的另一端, 那一段会横穿垄端外的掉头弧
  // -> 路径自交 (F2cCoverage 的自交判据当场报红); 与 makeRidgeSegments 里"贴边行会横穿已排出的行,
  // 宁可远端留盲带"是同一个取舍。
  // ⚠ 这里要的是 properlyCrosses 的**严格异号**含义: 本段的一端就是路径端点 (和路径首段/末段共享
  // 端点), "相触"必须不算相交 —— 否则这道门会永远返回 true, 补角一次都补不出去 (实测: 见
  // bow_detail.hpp 里 properlyCrosses 的说明)。
  const auto crosses_path = [&path](const Segment_2& seg) {
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      if (properlyCrosses(seg, Segment_2{path[i], path[i + 1]}))
        return true;
    }
    return false;
  };

  const auto add_corner = [&crosses_path, &count_other_points_on, &out_types](const char* which, std::vector<Point_2>& out, const Point_2& end, const Point_2& corner, const Segment_2& edge, bool at_front) {
    const size_t others = count_other_points_on(edge, end);
    if (others > 0) {
      SPDLOG_INFO("{}: 那条边上还有 {} 个路径点 (不是端头边), 不补 —— 前提是只有 path 的起点/终点在 AB/CD 上", which, others);
      return;
    }
    const Segment_2 seg = at_front ? Segment_2{corner, end} : Segment_2{end, corner};
    if (crosses_path(seg)) {
      SPDLOG_INFO("{}顶点 ({:.3f}, {:.3f}) 在 {:.2f} m 外且那段会横穿已铺路径: 不补 (只补不穿)", which, corner.x(), corner.y(), (corner - end).norm());
      return;
    }
    SPDLOG_INFO("{}沿区域边界边补到顶点: 垄端 ({:.3f}, {:.3f}) -> 顶点 ({:.3f}, {:.3f}), 补 {:.3f} m", which, end.x(), end.y(), corner.x(), corner.y(), (corner - end).norm());
    // 补进去的顶点标 kSegWork —— 沿区域边界边补出来的那一段也算**行内作业段** (贴着地头边界作业,
    // 与垄线同一档), 于是 PTO 路径会按 in_row_resolution 粗化它。
    if (at_front) {
      out.insert(out.begin(), corner);
      if (out_types != nullptr)
        out_types->insert(out_types->begin(), kSegWork);
    } else {
      out.push_back(corner);
      if (out_types != nullptr)
        out_types->push_back(kSegWork);
    }
  };

  // 起点: 必须贴在 AB (近端边界边) 上才处理 —— 沿 AB 走到"背后"那个顶点, 插在 P1 前面。
  // 搜索方向取车头方向的反向 (path[0] - path[1]) -> 只认背后的顶点。
  if ((path[1] - path[0]).norm() > kMinVertexSpacing && on_edge(path[0], near_side_edge)) {
    Point_2 corner;
    if (pick_corner(path[0], path[0] - path[1], near_side_edge, corner))
      add_corner("首端 (AB)", out, path[0], corner, near_side_edge, true);
  }

  // 终点: 必须贴在 CD (远端边界边) 上才处理 —— 沿 CD 继续走到"前方"那个顶点, 接在 Pn 后面。
  // 搜索方向 = 车头方向 (path.back() - path[path.size() - 2]) -> 只认前方的顶点。
  if ((path[path.size() - 2] - path.back()).norm() > kMinVertexSpacing && on_edge(path.back(), far_side_edge)) {
    Point_2 corner;
    if (pick_corner(path.back(), path.back() - path[path.size() - 2], far_side_edge, corner))
      add_corner("末端 (CD)", out, path.back(), corner, far_side_edge, false);
  }
  return out;
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

// ============================================================
// 与 AB / CD 都相切的半圆 (半径 = 垄距/2)
// ============================================================
std::vector<Point_2> PlanBowShapedPath::legacyTurn(const Point_2& P1, const Point_2& P2, const Point_2& P3, const Point_2& P4) {
  // 垄端间距太小
  if ((P2 - P1).norm() < kMinVertexSpacing || (P3 - P2).norm() < kMinVertexSpacing) {
    SPDLOG_WARN("垄端间距过小: ({:.3f},{:.3f})->({:.3f},{:.3f}) 或 ({:.3f},{:.3f})->({:.3f},{:.3f}), 忽略掉头", P1.x(), P1.y(), P2.x(), P2.y(), P2.x(), P2.y(), P3.x(), P3.y());
    return {P2, P3};   // 退化: 没有进入方向 / 两端重合 -> 直线连接 (按原垄端点走)
  }

  const Point_2 in_dir = (P2 - P1).normalized();             // 垄向: AB 的方向 (CD 的反方向)
  Point_2 n            = Point_2(-in_dir.y(), in_dir.x());   // 垄向的左法向
  const Point_2 chord  = P3 - P2;                            // 掉头处 B->C 的连接弦
  const double lat     = chord.dot(n);                       // 两条垄的横向间距 (垄距)
  if (std::abs(lat) < kMinVertexSpacing) {
    SPDLOG_WARN("两条垄几乎共线: ({:.3f},{:.3f})->({:.3f},{:.3f}) 与 ({:.3f},{:.3f})->({:.3f},{:.3f}), 忽略掉头", P1.x(), P1.y(), P2.x(), P2.y(), P2.x(), P2.y(), P3.x(), P3.y());
    return {P2, P3};
  }
  if (lat < 0.0)
    n = -n;   // 让 n 指向 C 那一侧 (半圆朝这一侧鼓)

  // 下一条垄的航向: 弓字形里通常是 -in_dir; 末行兜底行平行于远端边界边, 与垄向有夹角。
  const Point_2 out_dir = ((P4 - P3).norm() > kMinVertexSpacing) ? (P4 - P3).normalized() : -in_dir;

  // ---- 圆的直径 = min(B 到 CD, C 到 AB) ----
  // 这两个距离就是"B / C 各自离对面那条垄有多远", 取较小值 = 这一段里两条垄**最窄处的间隙**
  // (两条垄平行时它是常数 = 垄距; 不平行时间隙沿垄向单调变化, 最窄处必在 B 端或 C 端)。
  // 半径取一半, 两个分支共用: 平行 -> 与两条垄都相切的半圆; 不平行 -> 与两条垄都相切的圆。
  Point_2 n_cd = perpendicular(out_dir);   // CD 的法向, 取指向 B 那一侧
  if (n_cd.dot(P2 - P3) < 0.0)
    n_cd = -n_cd;
  const Point_2 n_ab  = n;                     // AB 的法向 (上面已按 lat 的符号定到指向 C 那一侧)
  const double d_b_cd = (P2 - P3).dot(n_cd);   // B 到 CD 所在直线 (n_cd 指向 B 侧, 恒正)
  const double d_c_ab = (P3 - P2).dot(n_ab);   // C 到 AB 所在直线 (n_ab 指向 C 侧, 恒正)
  const double radius = 0.5 * std::min(d_b_cd, d_c_ab);
  if (!(radius > kTurnCornerMinRadius)) {
    SPDLOG_WARN("半径非法: ({:.3f},{:.3f})->({:.3f},{:.3f}) 与 ({:.3f},{:.3f})->({:.3f},{:.3f}), 忽略掉头", P1.x(), P1.y(), P2.x(), P2.y(), P2.x(), P2.y(), P3.x(), P3.y());
    return {P2, P3};
  }

  // to do...这里还有很大的优化空间
  // ---- 两条垄**不平行** (末行兜底行, 或行端被裁得很短的碎行) ----
  // 直线连接会在 B、C 处留下大折角 (两垄越斜越大, 实测 90°~177° —— 现场就是"到点原地转"),
  // 所以先试**与两条垄都相切的圆**过渡; 那条弧不满足下面的准入判据 (或伸出地块边界) 时,
  // **退回半圆几何** —— 也就是继续走本函数后半段那一套 (顶点压在 B-C 中点, 再按"不伸出地块
  // 边界"逐步往外推), 与平行分支**同一条代码路径、同一形状**:
  //   * 这里曾写成"退回直线连接" (旧实现是 `return {P2, P3};`) —— 现在的取舍是: 两条垄接近平行
  //     (夹角 2° 上下, 实测三角 60x20 的碎行、斜四边形 skew=1.0 都是这种) 时, 半圆几何仍与上一
  //     条垄相切、且落点在下一条垄附近, 全程最大折角实测 4.6°/5.0° (< F2cTurn 的 10° 契约),
  //     比直线连接 (行端 20° 上下的折角) 更贴合"零折角、不倒走"的既定目标;
  //   * 真有大幅夹角时半圆的前提不成立, 但那时**同样**不该用折角更大的直线连接, 而是该由
  //     [TURN_POCKET]/[TURN_BOUNDARY] 把盲带与出界量报出来让操作手加大 field_margin —— 这条
  //     行为由 test_bow_planner 的 NonParallelRidgesFallBackToAnArcNotAStraightLine 钉住。
  if (std::abs(in_dir.x() * out_dir.y() - in_dir.y() * out_dir.x()) > kTurnParallelTol) {

    // 直线连接会在 B、C 处留下大折角 (两垄越斜越大, 实测 90°~177° —— 现场就是"到点原地转"),
    // 所以改用**与两条垄都相切的圆**过渡。
    // det = n_ab × n_cd = ±(in_dir × out_dir) = ±sin(两条垄夹角), 是下面解圆心的**除数**:
    // 外层那个平行门限 (kTurnParallelTol = 0.01) 已经保证 |det| >= 0.01 —— 分母天然非零, 所以
    // 这里**不写防御式的 |det| > kEpsilon 判断** (那是同一个数的更松比较, 恒真: 90° 旋转不改变
    // 叉积 cross(perp u, perp v) = cross(u, v), 两个法向相对各自的垄向只差一个符号)。
    const double det = n_ab.x() * n_cd.y() - n_ab.y() * n_cd.x();

    // 解 { (c - B)·n_ab = r, (c - C)·n_cd = r } -> 圆心 c
    const double rhs1 = radius + P2.dot(n_ab);
    const double rhs2 = radius + P3.dot(n_cd);
    const Point_2 center((rhs1 * n_cd.y() - n_ab.y() * rhs2) / det, (n_ab.x() * rhs2 - rhs1 * n_cd.x()) / det);
    const Point_2 t1 = center - radius * n_ab;   // AB 上的切点
    const Point_2 t2 = center - radius * n_cd;   // CD 上的切点

    // 转向: 圆心在 AB 的哪一侧 (n_ab 指向 C 侧) -> 从 T1 出发先向前再折回, 与半圆同一套规矩。
    const double turn_sign = (in_dir.x() * n_ab.y() - in_dir.y() * n_ab.x()) > 0.0 ? 1.0 : -1.0;
    const double a0        = std::atan2(t1.y() - center.y(), t1.x() - center.x());
    const double a1        = std::atan2(t2.y() - center.y(), t2.x() - center.x());
    double sweep           = (turn_sign > 0.0) ? (a1 - a0) : (a0 - a1);
    sweep                  = std::fmod(sweep, 2.0 * M_PI);
    if (sweep < 0.0)
      sweep += 2.0 * M_PI;

    // 准入判据 (全部满足才用这条弧):
    //   * 两个切点都落在**垄端附近** (<= 2 倍垄宽) —— 这条弧是给"两条垄近似平行、只是有点夹角"
    //     (末行兜底行) 用的; 夹角很大时 (实测三角形 60x20 上是 71.6°) 最窄横截面会跑到垄的中间
    //     甚至起点, 作业段会从垄端折回来 (路径原地折回, 实测 161° 折角), 这种形状**不用**这条弧,
    //     退回半圆几何 (见本分支开头的说明: 直线连接折角更大, 已不再使用);
    //   * 切点不重合、转角落在 [30°, 330°]、起点切向确实沿垄 (否则是绕远/几何退化)。
    const double near_tol      = 2.0 * ((ridge_width_ > kEpsilon) ? ridge_width_ : radius);
    const double start_dir_dot = (turn_sign * Point_2(-n_ab.y(), n_ab.x())).dot(in_dir);
    if ((t1 - P2).norm() <= near_tol && (t2 - P3).norm() <= near_tol && (t2 - t1).norm() > kMinVertexSpacing && sweep > 30.0 * M_PI / 180.0 && sweep < 330.0 * M_PI / 180.0 && start_dir_dot > 0.0) {
      const int by_len = static_cast<int>(std::ceil(sweep * radius / std::max(path_resolution_, 0.01)));
      const int by_ang = static_cast<int>(std::ceil(sweep / kMaxArcStepRad));
      const int steps  = std::max(2, std::max(by_len, by_ang));
      std::vector<Point_2> arc;
      arc.reserve(static_cast<size_t>(steps) + 1);
      arc.push_back(t1);
      for (int k = 1; k < steps; ++k) {
        const double a = a0 + turn_sign * sweep * static_cast<double>(k) / static_cast<double>(steps);
        arc.push_back(center + radius * Point_2(std::cos(a), std::sin(a)));
      }
      arc.push_back(t2);

      // 这条弧不许伸出地块边界 —— 伸出去就不用它, 退回半圆几何 (与平行分支同一套"不出界优先")。
      if (maxOverrunOf(arc) <= kTurnOverrunTolerance) {
        // 覆盖自查 (与平行分支同一套): 作业段到切点为止, 机具半幅盖不到的那个垄端 = 掉头处盲带。
        // 这里的盲带与 field_margin_ 无关 (是两垄不平行 + 最窄处决定的), 所以建议值给当前余量。
        const double half_swath = (ridge_width_ > kEpsilon) ? 0.5 * ridge_width_ : radius;
        const double pocket     = std::max(std::max(0.0, P2.dot(in_dir) - (t1.dot(in_dir) + half_swath)), std::max(0.0, P3.dot(in_dir) - (t2.dot(in_dir) + half_swath)));
        if (pocket > output_.turn_max_pocket) {
          output_.turn_max_pocket         = pocket;
          output_.turn_pocket_margin_hint = field_margin_;
        }
        return arc;
      }
    }
  }

  // 绕行方向: n 在垄向左侧 -> 逆时针 (sweep > 0), 右侧 -> 顺时针。
  const double cross = in_dir.x() * n.y() - in_dir.y() * n.x();   // 二维叉积, 符号即 n 相对垄向的侧别
  const double sweep = (cross > 0.0) ? M_PI : -M_PI;

  // 按给定圆心生成半圆 (T1 = 圆心 - R·n 落在 AB 上, T2 = 圆心 + R·n 落在 CD 上)。
  // 采样: 弧长 <= path_resolution_, 且每步转角 <= kMaxArcStepRad。
  const auto build_arc = [&](const Point_2& center) {
    const Point_2 t1 = center - radius * n;
    const Point_2 t2 = center + radius * n;
    const double a0  = std::atan2(t1.y() - center.y(), t1.x() - center.x());                                       // 起始方位角: 圆心指向切点 t1
    const int by_len = static_cast<int>(std::ceil(std::abs(sweep) * radius / std::max(path_resolution_, 0.01)));   // 弧长步长约束下的步数
    const int by_ang = static_cast<int>(std::ceil(std::abs(sweep) / kMaxArcStepRad));                              // 单步转角约束下的步数
    const int steps  = std::max(2, std::max(by_len, by_ang));                                                      // 两约束取严, 至少 2 步
    std::vector<Point_2> arc;
    arc.reserve(static_cast<size_t>(steps) + 2);
    arc.push_back(t1);
    for (int k = 1; k < steps; ++k) {
      const double a = a0 + sweep * static_cast<double>(k) / static_cast<double>(steps);   // 第 k 步方位角 (a0 匀速转到 a0+sweep)
      arc.push_back(center + radius * Point_2(std::cos(a), std::sin(a)));
    }
    arc.push_back(t2);
    return arc;
  };

  // 顶点压在 B-C 中点 = 旧几何 (圆心 = 顶点沿垄内方向退 R); 往外推 push 之后顶点落在"靠外那个
  // 垄端"上 (顶点沿垄向位置 = max(B,C)) —— 推的量就是错开量的一半 |d|/2。
  const Point_2 base_center = 0.5 * (P2 + P3) - radius * in_dir;
  const double push         = 0.5 * std::abs((P3 - P2).dot(in_dir));   // |d|/2

  // 从最外往回收: 每一步都查一次"弧有没有伸出地块边界", 取还不出界里最外的那个位置。
  double used_push         = 0.0;                      // 实际采用的往外推量 (推不动时保持 0)
  std::vector<Point_2> arc = build_arc(base_center);   // 默认按不推的圆心生成弧
  if (push > kEpsilon) {
    for (int step = kTurnOutwardSteps; step >= 1; --step) {
      const double candidate_push    = push * static_cast<double>(step) / static_cast<double>(kTurnOutwardSteps);
      std::vector<Point_2> candidate = build_arc(base_center + candidate_push * in_dir);
      if (maxOverrunOf(candidate) <= kTurnOverrunTolerance) {
        arc       = std::move(candidate);
        used_push = candidate_push;
        break;
      }
    }
  }

  // 覆盖自查: 扫幅沿垄向能盖到 (实际圆心 + R), 靠外那个垄端 (max(B,C)) 盖不到的部分就是掉头处
  // 的盲带。盖不住的原因只有一个 —— 半圆再往外推就要伸出地块边界了, 所以顺带给出"需要多大的
  // field_margin_ 才装得下" (理想位置的出界量 + 当前余量), 由 [TURN_POCKET] 报给操作手。
  const double reach   = (base_center + used_push * in_dir).dot(in_dir) + radius;   // 实际扫幅沿垄向最远能到的位置 (圆心投影 + R)
  const double far_end = std::max(P2.dot(in_dir), P3.dot(in_dir));                  // 靠外垄端沿垄向的坐标 max(B, C)
  const double pocket  = std::max(0.0, far_end - reach);                            // 半圆盖不住的盲带宽度
  if (pocket > output_.turn_max_pocket) {
    output_.turn_max_pocket         = pocket;
    output_.turn_pocket_margin_hint = field_margin_ + maxOverrunOf(build_arc(base_center + push * in_dir));   // 推满 |d|/2 的出界量 + 当前余量 = 装下所需的最小 field_margin_
  }
  return arc;
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
// 主流程 (generateWaypoints): 顶点准备 -> 内缩 -> 扫描方向 -> 垄线 -> 掉头/插值/PTO -> 端头补角 -> 汇总
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
    // 排不出垄线 = 内缩区域相对垄宽太窄 (垄太大), 归**参数类里的 ridge_width** —— 内缩这一步本身
    // 是成功的 (上面已经拿到有效区域), 不该算到 field_margin 头上 (见 bow.hpp 的状态说明)。
    SPDLOG_ERROR("内缩区域里生成不出垄线 (区域太窄, 或扫描线与区域裁剪后为空; ridge_width_ {:.3f} m, field_margin_ {:.3f} m)", ridge_width_, field_margin_);
    if (output_.ok())
      fail(PlanStatus::kBadRidgeWidth, "ridge_width 不合理: 内缩区域里生成不出垄线 (区域太窄, 或扫描线裁剪后为空); 垄宽 " + std::to_string(ridge_width_) + " m");
    return;
  }

  // 7. 内区弓字形: 掉头固定用半圆/直线几何 (legacyTurn) —— 这里**不插值**, 统一在拼接后插值一次。
  output_.key_waypoints = flattenRidges(ridges);
  for (size_t i = 0; i < output_.key_waypoints.size(); ++i) {
    SPDLOG_INFO("关键点[{}]: x = {:.4f}, y = {:.4f}", i, output_.key_waypoints[i].x(), output_.key_waypoints[i].y());
  }
  // 逐点类型 (WORK/TURN): 供 PTO 路径做"行内粗化、掉头保真"的分段采样 (见 buildPtoPath)。
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

  // 9. 采样: 结构路径落进输出 (供"只改分辨率"时复用), 然后一次算出两条路径 ——
  //    ① /global_path_planner/path 细路径 (path_resolution_)
  //    ② /global_path_planner/pto_path 行内粗采样 (行内 in_row_resolution_)。
  //    用**纯函数** resampleBowSkeleton: 节点在"只改分辨率"时会拿同一份 skeleton 再调它, 结果与
  //    这里全量重算逐位相同 (单测 ResampleOnlyMatchesFullPlan)。
  output_.skeleton.points = inner_path_ends;
  output_.skeleton.types  = inner_types_ends;
  resampleBowSkeleton(output_.skeleton, path_resolution_, in_row_resolution_, output_.waypoints, output_.pto_waypoints, output_.pto_seg_types, &output_.waypoint_types);

  if (!output_.pto_waypoints.empty()) {
    size_t n_work = 0, n_turn = 0;
    for (uint8_t t : output_.pto_seg_types) {
      n_work += (t == kSegWork) ? 1U : 0U;
      n_turn += (t == kSegTurn) ? 1U : 0U;
    }
    SPDLOG_INFO(
      "[PTO] 行内粗采样: {} 点 (行内 {} @ {:.2f} m, 掉头 {} @ {:.2f} m) vs 跟踪路径 {} 点 @ {:.2f} m",
      output_.pto_waypoints.size(),
      n_work,
      in_row_resolution_,
      n_turn,
      path_resolution_,
      output_.waypoints.size(),
      path_resolution_);
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
      "斜地头处每个掉头最多有 {:.2f} m 宽的带作业不到 (半径 {:.2f} m 的半圆掉头够不到靠外的那个垄端)。想盖住: 把 field_margin_ 加大到 >= {:.2f} m (当前 {:.2f} m; ",
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
// 采样 (纯函数): 结构路径 -> /global_path_planner/path 细路径 + /global_path_planner/pto_path 行内粗采样路径
// ============================================================
// 改造要点: 这两步原来是类的成员 (resamplePath / buildPtoPath), 现在提到**纯函数**里,
// 让 plan() 与"只改分辨率时的重采样" (GlobalPathPlannerNode::resampleOnly) 共用同一份实现 ——
// 否则两条路径迟早会走样 ("只重采样"必须与"全量重算"逐位相同, 见单测 ResampleOnlyMatchesFullPlan)。
//
// 段类型: 两端都是 WORK 的段 = 行内(作业)段 -> 按 in_row_resolution 加密; 掉头弧 (其余段)
// **原样保留骨架点** —— 弧点在几何阶段 (legacyTurn::build_arc) 已按 "弧长步长 <= path_resolution
// 且每步转角 <= kMaxArcStepRad" 采好, 这里再按 path_resolution 插一遍是空转 (不产生新点);
// 干脆不插还带来两条好处: /global_path_planner/path 与 /global_path_planner/pto_path 的弧**逐点相同**,
// 弧的保真度只由几何阶段决定。
// 逐点类型: 结构点用它自己的类型 (弧的首末切点仍是 WORK = 作业段起讫点), 插值出来的点 = WORK。
void resampleBowSkeleton(
  const BowSkeleton& skeleton,
  double path_resolution,
  double in_row_resolution,
  std::vector<Point_2>& waypoints,
  std::vector<Point_2>& pto_waypoints,
  std::vector<uint8_t>& pto_seg_types,
  std::vector<uint8_t>* waypoint_types) {
  waypoints.clear();
  pto_waypoints.clear();
  pto_seg_types.clear();
  if (waypoint_types != nullptr)
    waypoint_types->clear();

  // 行内分辨率比 path_resolution 还小 = "行内不会更粗": 夹到 path_resolution。
  // 夹取放在**这里**(而不是只放在 plan() 里): 这样"只改分辨率时重采样"与"全量重算"走的是同一条
  // 判据 —— 否则同样的入参在两条路上会得到不同的 PTO 路径 (单测 ResampleOnlyMatchesFullPlan 钉住)。
  double in_row = in_row_resolution;
  if (in_row > kEpsilon && in_row < path_resolution) {
    SPDLOG_INFO("in_row_resolution = {:.3f} m 比 path_resolution = {:.3f} m 还小: 行内不会更粗, 按 {:.3f} m 走", in_row, path_resolution, path_resolution);
    in_row = path_resolution;
  }

  const std::vector<Point_2>& pts = skeleton.points;
  if (pts.empty())
    return;
  const bool types_ok     = (skeleton.types.size() == pts.size());
  const bool seg_types_ok = (skeleton.seg_types.size() + 1 == pts.size());
  const bool labels_ok    = types_ok || seg_types_ok;
  // 段类型判据: 给了逐段类型 (外部骨架) 就以它为准; 否则"两端都是作业点才算行内段, 否则是掉头弧"
  // (标签缺失时一律按行内段 —— 与旧实现同一约定)。
  const auto segment_type = [&](size_t i) -> uint8_t {
    if (seg_types_ok)
      return skeleton.seg_types[i];
    return (types_ok && skeleton.types[i] == kSegWork && skeleton.types[i + 1] == kSegWork) ? kSegWork : (types_ok ? kSegTurn : kSegWork);
  };
  // 逐点类型 (发布时写进 pose.position.z): 有点类型就用它; 只有段类型时用"该点的出段类型"
  // (外部骨架里"出段是掉头段"的那些点就是掉头点); 都没有 -> 一律 WORK。
  const auto point_type = [&](size_t i) -> uint8_t {
    if (types_ok)
      return skeleton.types[i];
    if (seg_types_ok)
      return skeleton.seg_types[std::min(i, skeleton.seg_types.size() - 1)];
    return kSegWork;
  };
  if (pts.size() < 2 || !(path_resolution > kEpsilon)) {
    // 退化: 只有 <2 个点, 或分辨率为 0 -> 原样输出 (与旧实现同一约定)
    waypoints = pts;
    if (in_row > kEpsilon && labels_ok) {
      pto_waypoints = pts;
      pto_seg_types.resize(pts.size());
      for (size_t i = 0; i < pts.size(); ++i)
        pto_seg_types[i] = point_type(i);
    }
    return;
  }

  bg::model::linestring<Point_2> segment;
  segment.resize(2);

  // ---- ① /path: 整条按 path_resolution 均匀插值 (段端点原样保留; 折角不圆滑) ----
  // 实际被细分的只有**长段**: 骨架里一条垄只有两个端点; 本机规划的掉头弧点已在几何阶段按
  // "<= path_resolution 且 <= 5°/步" 采好, 每个弧段都短于 path_resolution -> 走下面的
  // "比分辨率还短"分支原样保留 (所以 /global_path_planner/path 与 /global_path_planner/pto_path 的弧是
  // 逐点相同的同一批点)。
  // 段端点用它自己的**点**类型, 插值出来的点取所在**段**的类型 (与 ② 同一套规矩);
  // 发布时这个类型写进 pose.position.z。
  const bool want_types = (waypoint_types != nullptr);
  const auto emit_fine  = [&waypoints, &waypoint_types, want_types](const Point_2& p, uint8_t type) {
    waypoints.push_back(p);
    if (want_types)
      waypoint_types->push_back(type);
  };
  waypoints.reserve(pts.size() * 2);
  if (want_types)
    waypoint_types->reserve(pts.size() * 2);
  emit_fine(pts.front(), point_type(0));
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const uint8_t seg_type = segment_type(i);
    segment[0]             = pts[i];
    segment[1]             = pts[i + 1];
    const double length    = bg::length(segment);
    if (length <= path_resolution) {   // 比分辨率还短的段不再细分 (本机规划的掉头弧恒走这一支)
      emit_fine(pts[i + 1], point_type(i + 1));
      continue;
    }
    // 段内按弧长均匀取点 (Boost.Geometry line_interpolate), 段端点原样保留。
    for (double d = path_resolution; d < length - kMinVertexSpacing; d += path_resolution) {
      Point_2 p;
      bg::line_interpolate(segment, d, p);
      emit_fine(p, seg_type);
    }
    emit_fine(pts[i + 1], point_type(i + 1));
  }

  // ---- ② /global_path_planner/pto_path: 行内段按 in_row_resolution 加密; 掉头段见下面分支 ----
  if (!(in_row > kEpsilon) || !labels_ok)
    return;   // 没开 PTO 产出, 或标签与点对不上 (宁可不产出, 也不给一份错位的标签)

  const auto emit = [&pto_waypoints, &pto_seg_types](const Point_2& p, uint8_t type) {
    pto_waypoints.push_back(p);
    pto_seg_types.push_back(type);
  };

  emit(pts.front(), point_type(0));
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const uint8_t seg_type = segment_type(i);
    segment[0]             = pts[i];
    segment[1]             = pts[i + 1];
    const double length    = bg::length(segment);

    // 步长: 行内段用 in_row_resolution (PTO 只关心"现在在哪一段作业", 1 m 粒度足够);
    // 掉头段用 path_resolution —— 本机规划的弧点已在几何阶段 (legacyTurn::build_arc) 按
    // "弧长步长 <= path_resolution 且每步转角 <= kMaxArcStepRad(5°)" 采好, 每个弧段都短于
    // path_resolution, 于是走第一支**原样保留、不重复插值** (弧的保真度只由几何阶段决定,
    // /global_path_planner/path 与 /global_path_planner/pto_path 的弧逐点相同); **外部骨架的粗掉头段**
    // (file 模式的掉头可能只有两个
    // 端点, 长达数米) 会落到插值支, 按 path_resolution 采出弧上的点 —— 否则 PTO 路径在掉头
    // 处是一条折线 (弧上最近点定位与航向差分最敏感, 采样太粗会把转弯当成折线)。
    const double step = (seg_type == kSegWork) ? in_row : path_resolution;
    if (length <= step) {
      emit(pts[i + 1], point_type(i + 1));
      continue;
    }
    for (double d = step; d < length - kMinVertexSpacing; d += step) {
      Point_2 p;
      bg::line_interpolate(segment, d, p);
      emit(p, seg_type);
    }
    emit(pts[i + 1], point_type(i + 1));
  }
}

// ============================================================
// 公开接口
// ============================================================
// 规划入口。状态在对象里 (见 bow.hpp 的私有成员), 所以这是**非 const** 方法: 一个实例同一时刻只
// 用于一次规划 (本节点按单线程执行器跑: 没有工作线程也没有锁 —— 见 global_path_planner.cpp 里
// updatePathFromInputs / planGeometry 的并发说明); 不同实例之间没有共享
// 状态, 所以"每线程一个实例"可以并行 (见 test/test_bow_parallel.cpp)。
// 参数夹取顺序与规则、日志文案都与改造前逐字一致; 结果整包返回, 对象上不导出 getter。
BowOutput PlanBowShapedPath::plan(const BowInput& in) {
  // 入参落到成员上 (类的状态); 几何步骤只读这些成员, **绝不回写调用方传进来的 in**。
  cell_              = in.cell;
  ridge_width_       = in.ridge_width;
  path_resolution_   = in.path_resolution;
  field_margin_      = in.field_margin;
  miter_limit_       = in.miter_limit;
  first_row_margin_  = in.first_row_margin;
  in_row_resolution_ = in.in_row_resolution;
  output_            = BowOutput{};   // 本次调用的输出/观测值 (成员只作暂存, 不对外暴露)
  // 拐角上限: Clipper2 只在 > 1 时才用它 (<= 1 时 join 分支一律切角); 这里把非法值 (<= 1 / NaN)
  // 夹到 1, 与 field_margin 的 std::max(0.0, ...) 同一套处理 —— 参数不当不该让规划直接失败,
  // 但要在日志里说清"你给的值没按原样生效"。std::max 对 NaN 返回第一个实参 (= 1.0), 也能兜住。
  miter_limit_ = std::max(1.0, in.miter_limit);
  if (miter_limit_ != in.miter_limit) {
    SPDLOG_WARN("miter_limit 不合理: {} (必须 >= 1.0), 本次按 {:.3f} 处理", in.miter_limit, miter_limit_);
  }

  // 首行余量: 下界 kMinFirstRowMargin (Clipper2 量化网格 ≈1 mm, 更小会被量化吃掉), 上界 垄宽/2
  // (超过就破了近端覆盖契约 —— 首行离边界太近, 机具半幅盖不住田边那侧)。越界只告警并夹取,
  // 与 field_margin / miter_limit 同一套"不让参数把规划弄失败"的约定。
  // NaN 不必单独兜: std::max/min 的比较对 NaN 恒为 false, 会停在左侧实参 (先夹到下界, 再由
  // min 保持), 结果落在合法域里; 上面那个"值变了就告警"照样会报出来。
  first_row_margin_ = std::max(kMinFirstRowMargin, in.first_row_margin);
  if (ridge_width_ > kEpsilon)
    first_row_margin_ = std::min(first_row_margin_, 0.5 * ridge_width_);
  if (first_row_margin_ != in.first_row_margin) {
    SPDLOG_WARN("first_row_margin 不合理: {} (取值域 [{:.3f}, 垄宽/2 = {:.3f}] m), 本次按 {:.3f} m 处理", in.first_row_margin, kMinFirstRowMargin, 0.5 * ridge_width_, first_row_margin_);
  }

  // PTO 路径 (行内粗采样) 的步长: <= 0 = 不产出 (老行为)。比 path_resolution 还小等于"行内不粗化"
  // (输出与 /path 同步长) —— 不是错误, 提示一句即可。
  in_row_resolution_ = std::max(0.0, in.in_row_resolution);
  if (in_row_resolution_ > kEpsilon && in_row_resolution_ < path_resolution_) {
    SPDLOG_INFO("in_row_resolution = {:.3f} m 比 path_resolution = {:.3f} m 还小: 行内不会更粗, 按 {:.3f} m 走", in_row_resolution_, path_resolution_, path_resolution_);
    in_row_resolution_ = path_resolution_;
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
