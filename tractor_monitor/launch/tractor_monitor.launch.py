#!/usr/bin/env python3
"""tractor_monitor: 监控除 tractor_base 外所有模块的心跳，并向 tractor_base 发心跳。

    ros2 launch tractor_monitor tractor_monitor.launch.py

参数名为 `monitor_config_file` 而非 `config_file`。多个 launch 文件组合时,
DeclareLaunchArgument 只在该配置尚未设置时才应用默认值, 于是同名参数会被最先
运行的 include 抢占, 后面的全部静默继承它的值 —— chassis_interface 曾因此加载
了 path_controller.yaml, 跑在错误的限速与轮距上。见 bringup.launch.py 中的记述。

require_* 三个参数存在的原因：bringup.launch.py 可以用 control:=false /
gnss:=false / planner:=false 关掉整条腿。被关掉的模块根本不会启动, 若监控仍然
"要求"它出现, 启动宽限期一过就会判 FAULT 并让底盘急停 —— 每一次局部启动都变成
一次误停车。所以 bringup 会按腿的开关把对应的 required 覆盖掉。

留空表示"不下发"，沿用 yaml 里的值。这是本仓库的既有约定：dict 形式的参数会变成
`-p` 覆盖, 优先级高于任何 --params-file, 所以给出具体默认值会让配置文件里的同名
项静默失效。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# (launch 参数, 被覆盖的节点参数)
# require_planner 同时覆盖 global_path_planner 与 local_path_planner_node:
# 两者都在 planner 腿内一起启停。
_REQUIRE_ARGS = (
    ('require_control', ('path_controller.required',)),
    ('require_gnss', ('gnss_driver.required',)),
    ('require_planner', ('global_path_planner.required',
                         'local_path_planner_node.required')),
)


def _setup(context, *args, **kwargs):
    default_config = os.path.join(
        get_package_share_directory('tractor_monitor'), 'config', 'tractor_monitor.yaml')

    cfg = LaunchConfiguration('monitor_config_file').perform(context).strip()
    if not cfg:
        cfg = default_config
    if not os.path.isfile(cfg):
        # 明确指定了却不存在, 就不要退回包默认值: 操作员是特意要求它的。
        return [LogInfo(msg=f'[monitor] FATAL: 配置文件不存在: {cfg}')]

    overrides = {}
    logs = [LogInfo(msg=f'[monitor] 配置文件: {cfg}')]
    for arg_name, param_names in _REQUIRE_ARGS:
        raw = LaunchConfiguration(arg_name).perform(context).strip().lower()
        if not raw:
            continue
        value = raw in ('true', '1')
        for param_name in param_names:
            overrides[param_name] = value
            if not value:
                logs.append(LogInfo(
                    msg=f'[monitor] {param_name}=false —— 该腿未启动, 其缺席不再判故障'))

    parameters = [cfg] + ([overrides] if overrides else [])

    return logs + [
        Node(
            package='tractor_monitor',
            executable='tractor_monitor_node',
            name='tractor_monitor',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'monitor_config_file', default_value='',
            description='tractor_monitor.yaml 配置文件路径。留空 -> 包内默认配置。'),
    ]
    args += [
        DeclareLaunchArgument(
            name, default_value='',
            description=f'覆盖 {param}。留空 -> 沿用配置文件。'
                        '由 bringup.launch.py 按对应腿的开关下发。')
        for name, param in _REQUIRE_ARGS
    ]

    ld = LaunchDescription()
    for action in args:
        ld.add_action(action)
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
