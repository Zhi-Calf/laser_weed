// 版权所有 2026。将 Apollo 离散点路径参数适配到轨迹点容器。
#pragma once

#include <vector>

#include "path_tracker/lqr.hpp"

namespace path_tracker {

/// 为原始 x/y 路径点计算航向、累计弧长、曲率和曲率变化率。
///
/// @param xy           原始 ENU 路径点（米），至少需要 3 个有效点
/// @param out          输出轨迹，函数开始时清空；输入无效时返回 false
///
/// 曲率符号约定：左转（逆时针）为正，与 CAN 协议左正右负的转角约定一致。
bool computePathProfile(
  const std::vector<std::pair<double, double>>& xy,
  std::vector<TrajectoryPoint>* out);

}   // namespace path_tracker
