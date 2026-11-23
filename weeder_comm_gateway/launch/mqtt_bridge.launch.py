#!/usr/bin/env python3
"""mqtt_bridge_node: 农机 <-> robot-cloud IoT 网关的 MQTT 桥。

    ros2 launch weeder_comm_gateway mqtt_bridge.launch.py
    ros2 launch weeder_comm_gateway mqtt_bridge.launch.py mqtt_host:=1.94.199.140 mqtt_port:=11883 \
        device_name:=robot_001 device_id:=111 mqtt_bridge_config_file:=/etc/agri/mqtt_bridge.yaml

device_secret 没有对应的 launch 参数: 命令行参数会出现在 ps 里。把它写进机器上单独的 yaml,
用 mqtt_bridge_config_file 指过去。

参数名为 `mqtt_bridge_config_file` 而非 `config_file`: 多个 launch 组合时同名参数会被最先
运行的 include 抢占。

mqtt_host / mqtt_port / device_name / device_id 留空表示"不下发", 沿用 yaml 里的值 —— dict 形式的参数优先级高于
--params-file, 给出具体默认值会让配置文件里的同名项静默失效。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_OVERRIDE_ARGS = ('mqtt_host', 'mqtt_port', 'device_name', 'device_id')
_INT_ARGS = ('mqtt_port',)


def _setup(context, *args, **kwargs):
    default_config = os.path.join(
        get_package_share_directory('weeder_comm_gateway'), 'config', 'mqtt_bridge.yaml')

    cfg = LaunchConfiguration('mqtt_bridge_config_file').perform(context).strip()
    if not cfg:
        cfg = default_config
    if not os.path.isfile(cfg):
        # 明确指定了却不存在, 不退回包默认值: 操作员是特意要求它的。
        return [LogInfo(msg=f'[mqtt_bridge_node] FATAL: 配置文件不存在: {cfg}')]

    overrides = {}
    for name in _OVERRIDE_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = int(value) if name in _INT_ARGS else value

    return [
        LogInfo(msg=f'[mqtt_bridge_node] 配置文件: {cfg}'),
        Node(
            package='weeder_comm_gateway',
            executable='mqtt_bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            parameters=[cfg] + ([overrides] if overrides else []),
        ),
    ]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'mqtt_bridge_config_file', default_value='',
        description='mqtt_bridge.yaml 配置文件路径。留空 -> 包内默认配置。'))
    for name in _OVERRIDE_ARGS:
        ld.add_action(DeclareLaunchArgument(
            name, default_value='', description=f'覆盖 {name}。留空 -> 沿用配置文件。'))
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
