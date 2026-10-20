// Copyright 2026. Path profile implementation.
//
// Finite-difference geometry ported from MegaTrak
// control_node/path_controller/src/path_profile.cpp. Model-independent, so the
// maths is reused unchanged; only the output container differs.
#include "path_tracker/path_profile.hpp"

#include <cmath>

namespace path_tracker
{

bool computePathProfile(
  const std::vector<std::pair<double, double>> & xy,
  double default_v,
  std::vector<TrajectoryPoint> * out)
{
  if (out == nullptr) {
    return false;
  }
  out->clear();
  const size_t n = xy.size();
  // Need at least two points to define a direction; curvature needs three, and
  // the endpoints below reuse the nearest interior value in that case.
  if (n < 2) {
    return false;
  }

  out->reserve(n);

  // Pass 1: heading (forward difference) and accumulated arc length.
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
      p.heading = std::atan2(dy, dx);
      s += std::hypot(dx, dy);
    } else {
      // Last point inherits the previous heading (no forward difference exists).
      p.heading = out->back().heading;
    }
    out->push_back(p);
  }

  // Pass 2: curvature from the circumscribed-circle formula on consecutive
  // segment directions, then the curvature rate along arc length.
  for (size_t i = 1; i + 1 < n; ++i) {
    const double x1 = xy[i - 1].first, y1 = xy[i - 1].second;
    const double x2 = xy[i].first,     y2 = xy[i].second;
    const double x3 = xy[i + 1].first, y3 = xy[i + 1].second;

    const double d1x = x2 - x1, d1y = y2 - y1;
    const double d2x = x3 - x2, d2y = y3 - y2;

    const double ds1 = std::hypot(d1x, d1y);
    const double ds2 = std::hypot(d2x, d2y);
    if (ds1 < 1e-9 || ds2 < 1e-9) {
      // Degenerate (duplicate) points: treat as straight.
      continue;
    }

    // Signed angle between consecutive segments, left turn positive.
    const double cross = d1x * d2y - d1y * d2x;
    const double dot = d1x * d2x + d1y * d2y;
    const double dtheta = std::atan2(cross, dot);

    // Curvature = turn angle / mean segment arc length.
    (*out)[i].kappa = dtheta / ((ds1 + ds2) * 0.5);
  }

  // Pass 3: curvature rate dkappa/ds by central difference on the arc length.
  // Kept separate from pass 2 because it needs the neighbouring kappa values to
  // be finalised first.
  for (size_t i = 1; i + 1 < n; ++i) {
    const double ds = (*out)[i + 1].s - (*out)[i - 1].s;
    (*out)[i].dkappa = (ds > 1e-9) ? ((*out)[i + 1].kappa - (*out)[i - 1].kappa) / ds : 0.0;
  }

  // Endpoints: clamp to the nearest interior value so look-ahead never reads a
  // zero curvature at the path ends and fails to slow down for a tight turn.
  if (n >= 3) {
    out->front().kappa = (*out)[1].kappa;
    out->front().dkappa = (*out)[1].dkappa;
    out->back().kappa = (*out)[n - 2].kappa;
    out->back().dkappa = (*out)[n - 2].dkappa;
  }

  return true;
}

}  // namespace path_tracker
