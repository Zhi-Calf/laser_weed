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
// 输入输出有限性检查、退化点检查以及 spdlog 错误日志。
#pragma once

#include <utility>
#include <vector>

namespace path_tracker {

// 由二维点列计算航向、累计弧长、曲率和曲率变化率。
bool ComputePathProfile(const std::vector<std::pair<double, double>>& xy_points, std::vector<double>* headings, std::vector<double>* accumulated_s, std::vector<double>* kappas, std::vector<double>* dkappas);

}   // namespace path_tracker
