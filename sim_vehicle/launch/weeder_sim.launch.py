# Copyright 2026. 一键启动激光除草机联调仿真。
#
# 把原来需要手动开的 4 个终端 + RViz + 四段式时序敲命令，合成一条：
#
#     ros2 launch sim_vehicle weeder_sim.launch.py
#
# 启动的节点：
#   sim_vehicle          仿真车辆（同时充当 GNSS 定位 + 底盘 + 模式管理方）
#   global_path_planner  从 YAML 读取田块并自动生成弓字形全覆盖路径
#   path_tracker          四轮转向 LQR 控制器
#   sim_orchestrator     等 YAML 路径 → 切 UNMANNED
#   rviz2                预置视图（田块边界/弓字形路径/仿真车/跟踪目标点）
#
# 启动顺序无关：planner 自动规划，编排器只等条件满足后切模式。
#
# ⚠️ 关于 LaunchConfiguration 与参数类型：LaunchConfiguration 是**字符串替换**，
#    直接赋给 declare_parameter<double>() 声明的参数会抛 InvalidParameterTypeException。
#    所以所有数值型覆盖都用 ParameterValue(..., value_type=float/int) 显式转换。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('sim_vehicle')
    planner_share = get_package_share_directory('global_path_planner')
    tracker_share = get_package_share_directory('path_tracker')

    rviz_config = os.path.join(sim_share, 'rviz', 'weeder_sim.rviz')
    planner_params = os.path.join(planner_share, 'config', 'global_path_planner.yaml')
    tracker_params = os.path.join(tracker_share, 'config', 'path_tracker.yaml')

    # ---- Launch arguments ----
    # wheelbase 同时传给仿真车辆与控制器，两者**必须一致**：否则仿真的"真实车辆"
    # 与控制器假设的模型不是同一台车，横向误差会收敛到非零稳态值，看起来像调参没调好。
    wheelbase = ParameterValue(
        LaunchConfiguration('wheelbase'), value_type=float)
    # 垄宽决定弓字形行距，也决定地头掉头半径（= 垄宽/2）。
    ridge_width = ParameterValue(
        LaunchConfiguration('ridge_width'), value_type=float)
    # field_margin 建议 >= ridge_width/2，保证机具半幅不越过田边。
    # 半圆掉头本身切在内缩边界上，不会因半径 1.25m > 内缩 0.5m 而单独鼓出田块；
    # 默认 2.5/2=1.25，给 1.5 留了余量。
    field_margin = ParameterValue(
        LaunchConfiguration('field_margin'), value_type=float)
    target_v = ParameterValue(LaunchConfiguration('target_v'), value_type=float)
    align_offset = ParameterValue(
        LaunchConfiguration('align_offset'), value_type=float)
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'wheelbase', default_value='1.2',
            description='等效轴距(m)，同时传给仿真车辆与 LQR 控制器，两处必须一致'),
        DeclareLaunchArgument(
            'ridge_width', default_value='2.5',
            description='垄宽/行距(m)，决定弓字形行距与地头掉头半径(=垄宽/2)'),
        DeclareLaunchArgument(
            'field_margin', default_value='1.5',
            description='田边内缩余量(m)，建议 >= ridge_width/2 以保证机具半幅不越过田边'),
        DeclareLaunchArgument(
            'target_v', default_value='0.5',
            description='目标线速度(m/s)，协议上限 1.4'),
        DeclareLaunchArgument(
            'align_offset', default_value='0.8',
            description='对齐路径起点时故意留的初始横向偏差(m)，用于观察收敛'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否同时启动 RViz 预置视图'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='墙钟即可，无需 sim time'),

        # ---- 仿真车辆：GNSS 定位 + 底盘 + 模式管理方三合一 ----
        Node(
            package='sim_vehicle',
            executable='sim_vehicle',
            name='sim_vehicle',
            output='screen',
            parameters=[{
                'wheelbase': wheelbase,
                'align_to_path_start': True,
                'align_lateral_offset': align_offset,
                'use_sim_time': use_sim_time,
            }],
        ),

        # ---- 全覆盖规划器：田块和几何参数来自 YAML，命令行可覆盖垄宽/余量 ----
        Node(
            package='global_path_planner',
            executable='global_path_planner_node',
            name='global_path_planner',
            output='screen',
            parameters=[
                planner_params,
                {'ridge_width': ridge_width},
                {'field_margin': field_margin},
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ---- LQR 控制器 ----
        Node(
            package='path_tracker',
            executable='path_tracker_node',
            name='path_tracker',
            output='screen',
            parameters=[
                tracker_params,
                # 命令行覆盖 yaml：wheelbase 与仿真车辆严格一致，target_v 可一键调
                {'wheelbase': wheelbase},
                {'lqr_target_v': target_v},
                # 仿真不启动激光雷达，显式关闭实车默认启用的点云避停门控。
                {'require_velocity_planner': False},
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ---- 编排器：路径生成后自动切 UNMANNED ----
        Node(
            package='sim_vehicle',
            executable='sim_orchestrator',
            name='sim_orchestrator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # ---- RViz 预置视图（rviz:=false 可关掉）----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
