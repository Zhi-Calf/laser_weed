# 版权所有 2026。四轮转向 LQR 控制器启动文件。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('path_tracker')
    default_params = os.path.join(pkg_share, 'config', 'path_tracker.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the path_tracker parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (set true under Gazebo/bag replay)',
        ),

        # 必须单独运行 CAN 桥（weeder_base chassis_node），控制指令才能到达车辆。
        # 本启动文件故意不启动 CAN 桥，因为它独占 CAN 接口，
        # 并且可能需要按实车部署方式单独启动。
        Node(
            package='path_tracker',
            executable='path_tracker_node',
            name='path_tracker',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
