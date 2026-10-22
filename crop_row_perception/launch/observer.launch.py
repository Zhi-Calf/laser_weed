"""Observation only. No controller, chassis, work-mode or simulator is started."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('crop_row_perception'), 'config', 'observer.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='crop_row_perception', executable='crop_row_observer',
             name='crop_row_perception', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
