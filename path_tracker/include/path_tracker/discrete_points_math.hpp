/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// 本文件基于 Apollo planning/common/math 的离散点路径参数算法，增加了
// 输入输出有限性检查、退化点检查以及 spdlog 错误日志，并提供轨迹容器适配。
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace path_tracker {

/// 控制器所需的轨迹点及其几何属性。
struct TrajectoryPoint {
  double x              = 0.0;   // ENU 坐标系 x 坐标（米）
  double y              = 0.0;   // ENU 坐标系 y 坐标（米）
  double heading        = 0.0;   // 路径航向角（弧度）
  double kappa          = 0.0;   // 路径曲率（1/米），左转为正
  double dkappa         = 0.0;   // 曲率变化率 d(kappa)/ds
  double s              = 0.0;   // 累计路径弧长（米）
  uint8_t waypoint_type = 0;     // 路径段类型，0=作业，1=掉头
};

// 由二维点列计算航向、累计弧长、曲率和曲率变化率。
bool ComputePathProfile(
  const std::vector<std::pair<double, double>>& xy_points,
  std::vector<double>* headings,
  std::vector<double>* accumulated_s,
  std::vector<double>* kappas,
  std::vector<double>* dkappas);

// 由二维点列计算完整轨迹，曲率符号约定为左正右负。
bool ComputePathProfile(
  const std::vector<std::pair<double, double>>& xy_points,
  std::vector<TrajectoryPoint>* trajectory);

}   // namespace path_tracker
