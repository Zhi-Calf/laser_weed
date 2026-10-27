// 版权所有 2026。点云 ROI 过滤与停车保持逻辑单元测试。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

#include "velocity_planner/obstacle_filter.hpp"

namespace {

struct Point3 {
  float x;   // x 坐标
  float y;   // y 坐标
  float z;   // z 坐标
};

sensor_msgs::msg::PointCloud2 MakeCloud(const std::vector<Point3>& points) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(points.size());

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

velocity_planner::RoiConfig MakeConfig() {
  velocity_planner::RoiConfig config;
  config.forward_min_m       = 0.0;
  config.forward_max_m       = 2.0;
  config.half_width_m        = 0.9;
  config.min_height_m        = 0.1;
  config.max_height_m        = 1.5;
  config.min_obstacle_points = 1;
  return config;
}

}   // namespace

TEST(RoiFilter, DetectsPointInsideRoi) {
  velocity_planner::RoiFilter filter(MakeConfig());
  const auto cloud = MakeCloud({{1.0F, 0.0F, 0.5F}});

  const auto result = filter.Evaluate(cloud);

  EXPECT_TRUE(result.cloud_valid);
  EXPECT_TRUE(result.stop_required);
  EXPECT_EQ(result.obstacle_points, 1U);
  EXPECT_NEAR(result.nearest_distance_m, 1.0F, 1e-6F);
}

TEST(RoiFilter, IgnoresPointsOutsideEveryRoiBoundary) {
  velocity_planner::RoiFilter filter(MakeConfig());
  const auto cloud = MakeCloud({
    {-0.01F, 0.0F, 0.5F},
    {2.01F, 0.0F, 0.5F},
    {1.0F, 0.91F, 0.5F},
    {1.0F, 0.0F, 0.09F},
    {1.0F, 0.0F, 1.51F},
  });

  const auto result = filter.Evaluate(cloud);

  EXPECT_TRUE(result.cloud_valid);
  EXPECT_FALSE(result.stop_required);
  EXPECT_EQ(result.obstacle_points, 0U);
  EXPECT_LT(result.nearest_distance_m, 0.0F);
}

TEST(RoiFilter, IgnoresIndividualNanAndInfinity) {
  velocity_planner::RoiFilter filter(MakeConfig());
  const float nan      = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const auto cloud     = MakeCloud({
        {nan, 0.0F, 0.5F},
        {1.0F, infinity, 0.5F},
        {0.5F, 0.0F, 0.5F},
  });

  const auto result = filter.Evaluate(cloud);

  EXPECT_TRUE(result.cloud_valid);
  EXPECT_TRUE(result.stop_required);
  EXPECT_EQ(result.obstacle_points, 1U);
  EXPECT_NEAR(result.nearest_distance_m, 0.5F, 1e-6F);
}

TEST(RoiFilter, RejectsCloudWithoutFinitePoints) {
  velocity_planner::RoiFilter filter(MakeConfig());
  const float nan      = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const auto cloud     = MakeCloud({
        {nan, 0.0F, 0.5F},
        {1.0F, infinity, 0.5F},
        {1.0F, 0.0F, nan},
  });

  const auto result = filter.Evaluate(cloud);

  EXPECT_FALSE(result.cloud_valid);
  EXPECT_FALSE(result.stop_required);
  EXPECT_EQ(result.obstacle_points, 0U);
}

TEST(RoiFilter, RejectsEmptyCloudWithXyzFields) {
  velocity_planner::RoiFilter filter(MakeConfig());
  const auto cloud = MakeCloud({});

  const auto result = filter.Evaluate(cloud);

  EXPECT_FALSE(result.cloud_valid);
  EXPECT_FALSE(result.stop_required);
  EXPECT_EQ(result.obstacle_points, 0U);
}

TEST(RoiFilter, RequiresConfiguredPointCount) {
  auto config                = MakeConfig();
  config.min_obstacle_points = 3;
  velocity_planner::RoiFilter filter(config);
  const auto two_points   = MakeCloud({
      {0.5F, 0.0F, 0.5F},
      {0.8F, 0.0F, 0.5F},
  });
  const auto three_points = MakeCloud({
    {0.5F, 0.0F, 0.5F},
    {0.8F, 0.0F, 0.5F},
    {1.1F, 0.0F, 0.5F},
  });

  EXPECT_FALSE(filter.Evaluate(two_points).stop_required);
  EXPECT_TRUE(filter.Evaluate(three_points).stop_required);
}

TEST(RoiFilter, RejectsCloudWithoutXyzFields) {
  velocity_planner::RoiFilter filter(MakeConfig());
  sensor_msgs::msg::PointCloud2 cloud;

  const auto result = filter.Evaluate(cloud);

  EXPECT_FALSE(result.cloud_valid);
  EXPECT_FALSE(result.stop_required);
  EXPECT_EQ(result.obstacle_points, 0U);
}

TEST(StopHoldFilter, KeepsStopUntilHoldTimeExpires) {
  velocity_planner::StopHoldFilter hold(0.3);

  EXPECT_TRUE(hold.Update(true, 10.0));
  EXPECT_TRUE(hold.Update(false, 10.1));
  EXPECT_TRUE(hold.Update(false, 10.29));
  EXPECT_FALSE(hold.Update(false, 10.30));
  EXPECT_FALSE(hold.Update(false, 10.4));
}

TEST(StopHoldFilter, TimeRollbackDoesNotCauseInfiniteHold) {
  velocity_planner::StopHoldFilter hold(0.3);

  EXPECT_TRUE(hold.Update(true, 10.0));
  EXPECT_TRUE(hold.Update(false, 9.0));
  EXPECT_FALSE(hold.Update(false, 9.31));
}
