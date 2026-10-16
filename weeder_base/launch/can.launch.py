from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interface = LaunchConfiguration('interface', default='vcan0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'interface',
            default_value='vcan0',
            description='CAN 接口名 (vcan0 用于仿真, can0 用于实机)',
        ),
        Node(
            package='weeder_base',
            executable='chassis_node',
            name='weeder_chassis',
            output='screen',
            parameters=[{
                'can_interface': interface,
                'cmd_timeout_s': 0.5,
                'vcu_timeout_s': 1.0,
                'publish_hz': 20.0,
                'max_speed_m_s': 1.4,
                'max_steer_deg': 90.0,
            }],
            arguments=[interface],
        ),
    ])
