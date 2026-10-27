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
#include "path_tracker/discrete_points_math.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstddef>

namespace path_tracker {
namespace {

void ClearProfile(std::vector<double>* headings, std::vector<double>* accumulated_s, std::vector<double>* kappas, std::vector<double>* dkappas) {
  headings->clear();
  accumulated_s->clear();
  kappas->clear();
  dkappas->clear();
}

}   // namespace

// 使用首末单边差分、中间中心差分计算路径的几何参数。
bool ComputePathProfile(const std::vector<std::pair<double, double>>& xy_points, std::vector<double>* headings, std::vector<double>* accumulated_s, std::vector<double>* kappas, std::vector<double>* dkappas) {
  if (headings == nullptr || accumulated_s == nullptr || kappas == nullptr || dkappas == nullptr) {
    return false;
  }
  ClearProfile(headings, accumulated_s, kappas, dkappas);

  const std::size_t points_size = xy_points.size();
  if (points_size < 3) {
    return false;
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    if (!std::isfinite(xy_points[i].first) || !std::isfinite(xy_points[i].second)) {
      SPDLOG_ERROR("ComputePathProfile: 第 {} 个点不是有限值", i);
      return false;
    }
    if (i > 0 && xy_points[i].first == xy_points[i - 1].first && xy_points[i].second == xy_points[i - 1].second) {
      SPDLOG_ERROR("ComputePathProfile: 第 {} 与第 {} 个点重合，切线没有定义", i - 1, i);
      return false;
    }
  }

  std::vector<double> dxs(points_size, 0.0);
  std::vector<double> dys(points_size, 0.0);
  for (std::size_t i = 0; i < points_size; ++i) {
    if (i == 0) {
      dxs[i] = xy_points[i + 1].first - xy_points[i].first;
      dys[i] = xy_points[i + 1].second - xy_points[i].second;
    } else if (i == points_size - 1) {
      dxs[i] = xy_points[i].first - xy_points[i - 1].first;
      dys[i] = xy_points[i].second - xy_points[i - 1].second;
    } else {
      dxs[i] = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      dys[i] = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
      if (dxs[i] == 0.0 && dys[i] == 0.0) {
        SPDLOG_ERROR("ComputePathProfile: 第 {} 个点前后重合，航向没有定义", i);
        return false;
      }
    }
  }

  headings->resize(points_size);
  accumulated_s->resize(points_size);
  kappas->resize(points_size);
  dkappas->resize(points_size);
  for (std::size_t i = 0; i < points_size; ++i) {
    (*headings)[i] = std::atan2(dys[i], dxs[i]);
  }

  (*accumulated_s)[0] = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    const double dx        = xy_points[i].first - xy_points[i - 1].first;
    const double dy        = xy_points[i].second - xy_points[i - 1].second;
    const double segment_s = std::sqrt(dx * dx + dy * dy);
    (*accumulated_s)[i]    = (*accumulated_s)[i - 1] + segment_s;
  }

  std::vector<double> x_first(points_size, 0.0);
  std::vector<double> y_first(points_size, 0.0);
  for (std::size_t i = 0; i < points_size; ++i) {
    if (i == 0) {
      const double segment_s = (*accumulated_s)[i + 1] - (*accumulated_s)[i];
      x_first[i]             = (xy_points[i + 1].first - xy_points[i].first) / segment_s;
      y_first[i]             = (xy_points[i + 1].second - xy_points[i].second) / segment_s;
    } else if (i == points_size - 1) {
      const double segment_s = (*accumulated_s)[i] - (*accumulated_s)[i - 1];
      x_first[i]             = (xy_points[i].first - xy_points[i - 1].first) / segment_s;
      y_first[i]             = (xy_points[i].second - xy_points[i - 1].second) / segment_s;
    } else {
      const double segment_s = (*accumulated_s)[i + 1] - (*accumulated_s)[i - 1];
      x_first[i]             = (xy_points[i + 1].first - xy_points[i - 1].first) / segment_s;
      y_first[i]             = (xy_points[i + 1].second - xy_points[i - 1].second) / segment_s;
    }
  }

  std::vector<double> x_second(points_size, 0.0);
  std::vector<double> y_second(points_size, 0.0);
  for (std::size_t i = 0; i < points_size; ++i) {
    if (i == 0) {
      const double segment_s = (*accumulated_s)[i + 1] - (*accumulated_s)[i];
      x_second[i]            = (x_first[i + 1] - x_first[i]) / segment_s;
      y_second[i]            = (y_first[i + 1] - y_first[i]) / segment_s;
    } else if (i == points_size - 1) {
      const double segment_s = (*accumulated_s)[i] - (*accumulated_s)[i - 1];
      x_second[i]            = (x_first[i] - x_first[i - 1]) / segment_s;
      y_second[i]            = (y_first[i] - y_first[i - 1]) / segment_s;
    } else {
      const double segment_s = (*accumulated_s)[i + 1] - (*accumulated_s)[i - 1];
      x_second[i]            = (x_first[i + 1] - x_first[i - 1]) / segment_s;
      y_second[i]            = (y_first[i + 1] - y_first[i - 1]) / segment_s;
    }
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    const double speed_squared = x_first[i] * x_first[i] + y_first[i] * y_first[i];
    const double denominator   = std::sqrt(speed_squared) * speed_squared + 1e-6;
    (*kappas)[i]               = (x_first[i] * y_second[i] - y_first[i] * x_second[i]) / denominator;
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    if (i == 0) {
      const double segment_s = (*accumulated_s)[i + 1] - (*accumulated_s)[i];
      (*dkappas)[i]          = ((*kappas)[i + 1] - (*kappas)[i]) / segment_s;
    } else if (i == points_size - 1) {
      const double segment_s = (*accumulated_s)[i] - (*accumulated_s)[i - 1];
      (*dkappas)[i]          = ((*kappas)[i] - (*kappas)[i - 1]) / segment_s;
    } else {
      const double segment_s = (*accumulated_s)[i + 1] - (*accumulated_s)[i - 1];
      (*dkappas)[i]          = ((*kappas)[i + 1] - (*kappas)[i - 1]) / segment_s;
    }
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    if (!std::isfinite((*headings)[i]) || !std::isfinite((*accumulated_s)[i]) || !std::isfinite((*kappas)[i]) || !std::isfinite((*dkappas)[i])) {
      SPDLOG_ERROR("ComputePathProfile: 第 {} 个点的参数不是有限值", i);
      ClearProfile(headings, accumulated_s, kappas, dkappas);
      return false;
    }
  }
  return true;
}

// 将离散点路径几何参数写入轨迹点容器。
bool ComputePathProfile(const std::vector<std::pair<double, double>>& xy_points, std::vector<TrajectoryPoint>* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }
  trajectory->clear();

  std::vector<double> headings;
  std::vector<double> accumulated_s;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  if (!ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  const std::size_t point_count = xy_points.size();
  trajectory->resize(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    TrajectoryPoint& point = (*trajectory)[i];
    point.x                = xy_points[i].first;
    point.y                = xy_points[i].second;
    point.heading          = headings[i];
    point.s                = accumulated_s[i];
    point.kappa            = kappas[i];
    point.dkappa           = dkappas[i];
  }
  return true;
}

}   // namespace path_tracker
