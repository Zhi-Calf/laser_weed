#!/usr/bin/env python3
"""千寻 SDK 差分客户端节点。

千寻 SDK 差分: 读接收机串口 GGA -> SDK 认证 -> 上传 GGA -> SDK 回调收 RTCM -> 写回串口。

Usage:
  ros2 launch gnss_localization sdk_client.launch.py
  ros2 launch gnss_localization sdk_client.launch.py config_file:=/path/to/sdk_client.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("gnss_localization")
    default_config = os.path.join(share, "config", "sdk_client.yaml")

    config_arg = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=default_config,
            description="sdk_client.yaml"),

        Node(
            package="gnss_localization",
            executable="sdk_client_node",
            name="sdk_client",
            output="screen",
            parameters=[config_arg],
        ),
    ])
