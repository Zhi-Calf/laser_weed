#pragma once

#include <Eigen/Core>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

// 将 Eigen::Vector2d 注册为 Boost.Geometry 的 2D 点类型
// 之后 std::vector<Eigen::Vector2d> 可直接作为 Boost.Geometry 的 ring/polygon 使用
BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double, cs::cartesian, x(), y())

// 将 std::vector<Eigen::Vector2d> 注册为 Boost.Geometry 的 ring（闭合多边形）
// Boost.Geometry 的 ring 要求首尾点一致，这里注册为 open ring（自动闭合）
BOOST_GEOMETRY_REGISTER_RING(std::vector<Eigen::Vector2d>)
