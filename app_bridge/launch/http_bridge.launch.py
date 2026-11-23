#!/usr/bin/env python3
"""http_bridge: APP <-> 农机的 HTTP 服务 (协议见 doc/APP接口_MQTT_HTTP_v0.1.md §4)。

    ros2 launch app_bridge http_bridge.launch.py
    ros2 launch app_bridge http_bridge.launch.py port:=8443 path_file:=/opt/agri/test.json

参数名为 `http_bridge_config_file` 而非 `config_file`: 多个 launch 组合时同名参数会被最先
运行的 include 抢占。

port / path_file 留空表示"不下发", 沿用 yaml 里的值 —— dict 形式的参数优先级高于
--params-file, 给出具体默认值会让配置文件里的同名项静默失效。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    default_config = os.path.join(
        get_package_share_directory('app_bridge'), 'config', 'http_bridge.yaml')

    cfg = LaunchConfiguration('http_bridge_config_file').perform(context).strip()
    if not cfg:
        cfg = default_config
    if not os.path.isfile(cfg):
        return [LogInfo(msg=f'[http_bridge] FATAL: 配置文件不存在: {cfg}')]

    overrides = {}
    port = LaunchConfiguration('port').perform(context).strip()
    if port:
        overrides['port'] = int(port)
    path_file = LaunchConfiguration('path_file').perform(context).strip()
    if path_file:
        overrides['path_file'] = path_file

    return [
        LogInfo(msg=f'[http_bridge] 配置文件: {cfg}'),
        Node(
            package='app_bridge',
            executable='http_bridge_node',
            name='http_bridge',
            output='screen',
            parameters=[cfg] + ([overrides] if overrides else []),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'http_bridge_config_file', default_value='',
            description='http_bridge.yaml 配置文件路径。留空 -> 包内默认配置。'),
        DeclareLaunchArgument('port', default_value='', description='覆盖 port。留空 -> 沿用配置文件。'),
        DeclareLaunchArgument('path_file', default_value='', description='覆盖 path_file。留空 -> 沿用配置文件。'),
        OpaqueFunction(function=_setup),
    ])
