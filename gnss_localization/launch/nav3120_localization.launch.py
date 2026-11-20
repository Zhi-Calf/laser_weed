#!/usr/bin/env python3
"""启动 NAV3120 INSPVAA + GGA 定位驱动。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("gnss_localization")
    default_params = os.path.join(pkg_dir, "config", "nav3120_driver.yaml")

    def launch_setup(context, *args, **kwargs):
        params_file = LaunchConfiguration("params_file").perform(context)
        return [
            Node(
                package="gnss_localization",
                executable="gnss_driver_node",
                name="gnss_driver",
                namespace="",
                parameters=[params_file],
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="NAV3120 driver params YAML",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
