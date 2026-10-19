// Copyright 2026. Fault codes for the lqr_control node.
//
// Coding convention mirrors MegaTrak (3-digit module number + 2-digit index).
// lqr_control occupies the same role in the chain as MegaTrak's path_controller,
// so it reuses the 101xx range and keeps the same semantics per code, which lets
// the existing APP / cloud fault tables and tractor_monitor watch table apply
// unchanged.
#ifndef LQR_CONTROL__FAULT_CODES_HPP_
#define LQR_CONTROL__FAULT_CODES_HPP_

#include <cstdint>

namespace lqr_control
{
namespace fault
{

/// /work_mode missing, stale, or not MODE_UNMANNED -> not armed, motion withheld.
inline constexpr uint32_t kNoWorkMode = 10101;

/// No usable RTK pose: odometry missing, timed out, or solution degraded below
/// the rtk_max_sigma gate.
inline constexpr uint32_t kNoRtkPose = 10102;

/// No path to track: never received, or the last one went stale.
inline constexpr uint32_t kNoPath = 10103;

/// Steering mode is crab (2) or in-place (3), where the Ackermann assumption
/// behind B = v*dt/L does not hold. Configuration error, not a runtime fault.
inline constexpr uint32_t kBadSteerMode = 10104;

/// Trajectory exhausted: the robot reached the end of the path.
inline constexpr uint32_t kPathFinished = 10105;

/// The robot is farther from the tracked goal than max_track_error_m.
///
/// WHY THIS CODE EXISTS: the LQR error state is [lateral_error, heading_error],
/// which contains NO along-track component. A vehicle that runs straight past
/// the goal along the path's own direction therefore reads as PERFECT tracking
/// (both errors exactly zero) and keeps driving forever. Observed in simulation
/// on 2026-10-15: goal frozen at the path start, robot 238 m downrange,
/// steer_angle_rad = 2.4e-14, everything "nominal". This distance gate is the only
/// thing that catches that failure mode.
inline constexpr uint32_t kTrackErrorLost = 10106;

/// Upstream node names, used as Heartbeat.fault_source_node so the APP can
/// attribute a fault to whoever actually caused it (see tractor_monitor/
//  fault_source.hpp). Empty string (kFaultSourceSelf) means lqr_control itself.
inline constexpr const char * kSourceGnss = "gnss_driver";
inline constexpr const char * kSourcePlanner = "global_path_planner";
inline constexpr const char * kSourceApp = "app";
inline constexpr const char * kSourceVcu = "vcu";

}  // namespace fault
}  // namespace lqr_control

#endif  // LQR_CONTROL__FAULT_CODES_HPP_
