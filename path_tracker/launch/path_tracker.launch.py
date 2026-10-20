# Copyright 2026. Launch file for the four-wheel-steering LQR controller.
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

        # The CAN bridge (weeder_base chassis_node) must be running for the
        # commands to reach the vehicle. It is intentionally NOT started here:
        # it owns the CAN interface and may be launched separately on the robot.
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
