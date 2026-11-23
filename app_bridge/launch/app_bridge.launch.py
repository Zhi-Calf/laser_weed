#!/usr/bin/env python3
"""app_bridge: 同时启动 mqtt_bridge 与 http_bridge。

    ros2 launch app_bridge app_bridge.launch.py
    ros2 launch app_bridge app_bridge.launch.py mqtt_host:=192.168.1.100

各自的参数原样透传给 mqtt_bridge.launch.py / http_bridge.launch.py (见那两个文件)。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('app_bridge'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mqtt_bridge.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'http_bridge.launch.py'))),
    ])
