#!/usr/bin/env python3
"""weeder_comm_gateway: 同时启动 mqtt_bridge_node 与 http_bridge_node。

    ros2 launch weeder_comm_gateway weeder_comm_gateway.launch.py

组合 launch 仅使用两份默认 YAML。需要覆盖参数时，分别启动
mqtt_bridge.launch.py 或 http_bridge.launch.py。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('weeder_comm_gateway'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mqtt_bridge.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'http_bridge.launch.py'))),
    ])
