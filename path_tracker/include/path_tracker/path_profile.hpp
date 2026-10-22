// 版权所有 2026。使用有限差分计算路径航向、弧长和曲率。
//
// 本实现移植自 MegaTrak control_node/path_controller/src/path_profile.cpp。
// 算法是纯几何计算，与车辆模型无关，因此可以原样复用。
// 仅输出容器改为直接填充 std::vector<TrajectoryPoint>。
#ifndef PATH_TRACKER__PATH_PROFILE_HPP_
#define PATH_TRACKER__PATH_PROFILE_HPP_

#include "path_tracker/lqr.hpp"

#include <vector>

namespace path_tracker
{

/// 为原始 x/y 路径点计算航向、累计弧长、曲率和曲率变化率，
/// 并写入统一的目标速度。
///
/// @param xy           原始 ENU 路径点（米），至少需要 2 个点
/// @param default_v    写入每个 TrajectoryPoint::v 的默认速度
/// @param out          输出轨迹，函数开始时清空；xy 少于 2 点时返回 false
///
/// 曲率符号约定：左转（逆时针）为正，与 CAN 协议左正右负的转角约定一致。
bool computePathProfile(
  const std::vector<std::pair<double, double>> & xy,
  double default_v,
  std::vector<TrajectoryPoint> * out);

}  // 命名空间 path_tracker

#endif  // PATH_TRACKER__PATH_PROFILE_HPP_
