from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    interface = LaunchConfiguration('interface')

    # 参数默认值集中在 config/weeder_base.yaml, launch 只覆盖 CAN 接口名,
    # 避免"同一个默认值散落在两处, 改一处忘一处"。
    params_file = os.path.join(
        get_package_share_directory('weeder_base'), 'config', 'weeder_base.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'interface',
            default_value='vcan0',
            description='CAN 接口名 (vcan0 用于仿真, can0 用于实机)',
        ),
        Node(
            package='weeder_base',
            executable='chassis_node',
            # 名字必须与 yaml 顶层键 weeder_chassis 一致, 否则 yaml 参数不生效
            name='weeder_chassis',
            output='screen',
            parameters=[params_file, {'can_interface': interface}],
            # CAN 打不开 (vcan0 没起 / 实车线束掉) 时节点构造抛异常退出, 5s 后重试。
            # 不 respawn 的话 launch 会认为"节点在跑", 底盘却一帧都收不到。
            respawn=True,
            respawn_delay=5.0,
        ),
    ])

