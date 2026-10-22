// 版权所有 2026。将 Apollo 离散点路径参数适配到轨迹点容器。
#include "path_tracker/path_profile.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include "path_tracker/discrete_points_math.hpp"

namespace path_tracker {

// 计算路径几何参数并写入轨迹点字段。
bool computePathProfile(
  const std::vector<std::pair<double, double>>& xy,
  std::vector<TrajectoryPoint>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  std::vector<double> headings;
  std::vector<double> accumulated_s;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  if (!ComputePathProfile(
        xy, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  const std::size_t point_count = xy.size();
  out->resize(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    TrajectoryPoint& point = (*out)[i];
    point.x = xy[i].first;
    point.y = xy[i].second;
    point.heading = headings[i];
    point.s = accumulated_s[i];
    point.kappa = kappas[i];
    point.dkappa = dkappas[i];
  }
  return true;
}

}   // namespace path_tracker
