// 版权所有 2026。path_tracker 节点故障码。
//
// 编码规则沿用 MegaTrak：3 位模块号加 2 位故障序号。
// path_tracker 在控制链中承担与 MegaTrak path_controller 相同的职责，
// 因此继续使用 101xx 范围，并保持各故障码的语义，便于日志和健康状态上报。
#ifndef PATH_TRACKER__FAULT_CODES_HPP_
#define PATH_TRACKER__FAULT_CODES_HPP_

#include <cstdint>

namespace path_tracker
{
namespace fault
{

/// /work_mode 缺失、过期或不是 MODE_UNMANNED，系统未使能并禁止运动。
inline constexpr uint32_t kNoWorkMode = 10101;

/// 没有可用 RTK 位姿：位姿缺失、超时或 Pose.valid 为 false。
inline constexpr uint32_t kNoRtkPose = 10102;

/// 没有可跟踪路径：从未收到路径，或最后一条路径已经过期。
inline constexpr uint32_t kNoPath = 10103;

/// 转向模式为平移（2）或原地转向（3），此时 B = v*dt/L 的阿克曼假设不成立。
/// 这属于配置错误，不是运行期故障。
inline constexpr uint32_t kBadSteerMode = 10104;

/// 轨迹已耗尽，车辆到达路径终点。
inline constexpr uint32_t kPathFinished = 10105;

/// 车辆到跟踪目标点的距离超过 max_track_error_m。
///
/// 该故障码存在的原因：LQR 误差状态为 [横向误差, 航向误差]，不含沿路径方向
/// 的误差。车辆沿路径方向冲过目标点时，两项误差都为零，控制器会误判为完美跟踪
/// 并持续行驶。2026-10-15 仿真中曾出现目标点冻结在路径起点、车辆驶出 238 米、
/// steer_angle_rad = 2.4e-14，所有指标仍显示正常的情况。
/// 该距离门控是发现此类失效的唯一保护。
inline constexpr uint32_t kTrackErrorLost = 10106;

/// ROI 内检测到有效障碍点，避停门控主动要求停车。
inline constexpr uint32_t kObstacleInRoi = 10107;

/// 点云、速度规划节点或避停状态消息不可用；按失效停车处理。
inline constexpr uint32_t kVelocityPlannerUnavailable = 10108;

/// 保留上游节点名称，供后续故障归属和健康上报使用。
inline constexpr const char * kSourceGnss = "gnss_driver";
inline constexpr const char * kSourcePlanner = "global_path_planner";
inline constexpr const char * kSourceApp = "app";
inline constexpr const char * kSourceVcu = "vcu";

}  // 命名空间 fault
}  // 命名空间 path_tracker

#endif  // PATH_TRACKER__FAULT_CODES_HPP_
