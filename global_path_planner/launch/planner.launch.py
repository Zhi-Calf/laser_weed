#!/usr/bin/env python3
"""
启动 global_path_planner 规划节点（可选 RViz）.

田块顶点、垄宽和田边余量都在
config/global_path_planner.yaml 中配置。poly 的第一个顶点同时作为
ENU 原点；节点启动后立即规划并发布 /global_path_planner/path。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("global_path_planner")
    config_path = LaunchConfiguration("planner_config_file").perform(context)
    rviz_config = os.path.join(share, "rviz", "rviz.rviz")

    actions = [LogInfo(msg=["[planner] 配置文件: ", config_path])]
    actions.append(Node(
        package="global_path_planner",
        executable="global_path_planner_node",
        # 节点名必须与 YAML 顶层键 global_path_planner 一致。
        name="global_path_planner",
        output="screen",
        parameters=[config_path],
    ))

    # 本文件不发布静态 TF；RViz Fixed Frame=odom 需由 gnss_driver 或仿真节点提供。
    actions.append(Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("global_path_planner")
    default_config = os.path.join(share, "config", "global_path_planner.yaml")

    args = [
        DeclareLaunchArgument(
            "planner_config_file",
            default_value=default_config,
            description="规划器配置（包含 poly、ridge_width、field_margin 等参数）"),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="是否启动 RViz；SSH/无头运行时设 false"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
