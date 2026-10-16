#!/usr/bin/env python3
"""NTRIP 差分客户端节点。

外置 NTRIP 客户端: 读接收机串口 GGA -> 连 NTRIP caster -> 收 RTCM 写回串口。

Usage:
  ros2 launch gnss_localization ntrip_client.launch.py
  ros2 launch gnss_localization ntrip_client.launch.py config_file:=/path/to/ntrip_client.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("gnss_localization")
    default_config = os.path.join(share, "config", "ntrip_client.yaml")

    config_arg = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=default_config,
            description="ntrip_client.yaml"),

        Node(
            package="gnss_localization",
            executable="ntrip_client_node",
            name="ntrip_client",
            output="screen",
            parameters=[config_arg],
        ),
    ])
