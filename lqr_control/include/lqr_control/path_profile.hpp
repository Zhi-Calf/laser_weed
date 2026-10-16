// Copyright 2026. Path profile: heading / arc length / curvature by finite differences.
//
// Ported from MegaTrak control_node/path_controller/src/path_profile.cpp.
// The algorithm is model-independent (pure geometry) so it is reused as-is.
// Only the container changes: it fills std::vector<TrajectoryPoint> directly.
#ifndef LQR_CONTROL__PATH_PROFILE_HPP_
#define LQR_CONTROL__PATH_PROFILE_HPP_

#include "lqr_control/lqr.hpp"

#include <vector>

namespace lqr_control
{

/// Annotate raw x/y points with heading, accumulated arc length, curvature and
/// curvature rate, plus a constant desired speed.
///
/// @param xy           raw path points (ENU metres), at least 2 required
/// @param default_v    speed written into every point's TrajectoryPoint::v
/// @param out          filled trajectory; cleared first. Returns false if xy < 2.
///
/// Curvature sign convention: left (counter-clockwise) turn is positive, which
/// matches the CAN protocol's steer angle sign (left positive).
bool computePathProfile(
  const std::vector<std::pair<double, double>> & xy,
  double default_v,
  std::vector<TrajectoryPoint> * out);

}  // namespace lqr_control

#endif  // LQR_CONTROL__PATH_PROFILE_HPP_
