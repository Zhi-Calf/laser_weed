// 版权所有 2026。路径剖面实现。
//
// 有限差分几何计算移植自 MegaTrak
// control_node/path_controller/src/path_profile.cpp。算法与车辆模型无关，
// 数学计算保持不变，仅输出容器不同。
#include "path_tracker/path_profile.hpp"

#include <cmath>

namespace path_tracker {

bool computePathProfile(const std::vector<std::pair<double, double>>& xy, double default_v, std::vector<TrajectoryPoint>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  const size_t n = xy.size();
  // 至少需要两个点才能确定方向；曲率需要三个点，因此端点继承最近的内点值。
  if (n < 2) {
    return false;
  }

  out->reserve(n);

  // 第一遍：使用前向差分计算航向，同时累计弧长。
  double s = 0.0;
  for (size_t i = 0; i < n; ++i) {
    TrajectoryPoint p;
    p.x = xy[i].first;
    p.y = xy[i].second;
    p.v = default_v;
    p.s = s;

    if (i + 1 < n) {
      const double dx = xy[i + 1].first - xy[i].first;
      const double dy = xy[i + 1].second - xy[i].second;
      p.heading       = std::atan2(dy, dx);
      s += std::hypot(dx, dy);
    } else {
      // 最后一点没有前向差分，继承前一个点的航向。
      p.heading = out->back().heading;
    }
    out->push_back(p);
  }

  // 第二遍：根据相邻路径段方向使用外接圆关系计算曲率。
  for (size_t i = 1; i + 1 < n; ++i) {
    const double x1 = xy[i - 1].first, y1 = xy[i - 1].second;
    const double x2 = xy[i].first, y2 = xy[i].second;
    const double x3 = xy[i + 1].first, y3 = xy[i + 1].second;

    const double d1x = x2 - x1, d1y = y2 - y1;
    const double d2x = x3 - x2, d2y = y3 - y2;

    const double ds1 = std::hypot(d1x, d1y);
    const double ds2 = std::hypot(d2x, d2y);
    if (ds1 < 1e-9 || ds2 < 1e-9) {
      // 重复点形成的退化线段按直线处理。
      continue;
    }

    // 计算相邻路径段之间的有符号夹角，左转为正。
    const double cross  = d1x * d2y - d1y * d2x;
    const double dot    = d1x * d2x + d1y * d2y;
    const double dtheta = std::atan2(cross, dot);

    // 曲率等于转角除以两段平均弧长。
    (*out)[i].kappa = dtheta / ((ds1 + ds2) * 0.5);
  }

  // 第三遍：沿弧长使用中心差分计算曲率变化率 dkappa/ds。
  // 该步骤依赖第二遍得到的相邻曲率，因此单独执行。
  for (size_t i = 1; i + 1 < n; ++i) {
    const double ds  = (*out)[i + 1].s - (*out)[i - 1].s;
    (*out)[i].dkappa = (ds > 1e-9) ? ((*out)[i + 1].kappa - (*out)[i - 1].kappa) / ds : 0.0;
  }

  // 端点继承最近的内点曲率，避免前瞻在路径首尾读到虚假的零曲率，
  // 从而漏掉急弯降速。
  if (n >= 3) {
    out->front().kappa  = (*out)[1].kappa;
    out->front().dkappa = (*out)[1].dkappa;
    out->back().kappa   = (*out)[n - 2].kappa;
    out->back().dkappa  = (*out)[n - 2].dkappa;
  }

  return true;
}

}   // 命名空间 path_tracker
