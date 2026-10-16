# Copyright 2026. 一键启动激光除草机联调仿真。
#
# 把原来需要手动开的 4 个终端 + RViz + 四段式时序敲命令，合成一条：
#
#     ros2 launch sim_vehicle weeder_sim.launch.py
#
# 启动的节点：
#   sim_vehicle          仿真车辆（同时充当 GNSS 定位 + 底盘 + 模式管理方）
#   global_path_planner  全覆盖弓字形规划（从 MegaTrak 移植，原样）
#   lqr_control          四轮转向 LQR 控制器
#   sim_orchestrator     编排四段式时序：等原点 → MANUAL → set_source → 等路径 → UNMANNED
#   rviz2                预置视图（田块边界/弓字形路径/仿真车/跟踪目标点）
#
# 启动顺序无关：编排器每个阶段都等条件满足才推进，所以谁先起来都行。
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
    lqr_share = get_package_share_directory('lqr_control')

    rviz_config = os.path.join(sim_share, 'rviz', 'weeder_sim.rviz')
    planner_params = os.path.join(planner_share, 'config', 'global_path_planner.yaml')
    lqr_params = os.path.join(lqr_share, 'config', 'lqr_control.yaml')

    # ---- Launch arguments ----
    # wheelbase 同时传给仿真车辆与控制器，两者**必须一致**：否则仿真的"真实车辆"
    # 与控制器假设的模型不是同一台车，横向误差会收敛到非零稳态值，看起来像调参没调好。
    wheelbase = ParameterValue(
        LaunchConfiguration('wheelbase'), value_type=float)
    # 垄宽决定弓字形行距，也决定地头掉头半径（= 垄宽/2）。
    ridge_width = ParameterValue(
        LaunchConfiguration('ridge_width'), value_type=float)
    # ⚠️ field_margin 必须 >= ridge_width/2，否则半圆掉头会鼓出田块边界
    #    （2026-10-15 仿真里绿箭飞出田块就是这个原因：掉头半径 1.25m > 内缩 0.5m）。
    #    默认 2.5/2=1.25，给 1.5 留了余量。编排器也会在启动前预检并告警。
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
            description='田边内缩余量(m)，必须 >= ridge_width/2 否则掉头出界'),
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
                # 必须非零且 <= lqr_control 的 rtk_max_sigma(0.05)，否则控制器判定
                # "定位质量未知"而一直停车。0.02 对应 gnss_driver 的 acc_rtk_fixed。
                'rtk_sigma': 0.02,
                'use_sim_time': use_sim_time,
            }],
        ),

        # ---- 全覆盖规划器（MegaTrak 原样移植，用它自己的 config）----
        Node(
            package='global_path_planner',
            executable='global_path_planner_node',
            name='global_path_planner',
            output='screen',
            parameters=[planner_params, {'use_sim_time': use_sim_time}],
        ),

        # ---- LQR 控制器 ----
        Node(
            package='lqr_control',
            executable='lqr_node',
            name='lqr_control',
            output='screen',
            parameters=[
                lqr_params,
                # 命令行覆盖 yaml：wheelbase 与仿真车辆严格一致，target_v 可一键调
                {'wheelbase': wheelbase},
                {'lqr_target_v': target_v},
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ---- 编排器：自动完成四段式时序 ----
        Node(
            package='sim_vehicle',
            executable='sim_orchestrator',
            name='sim_orchestrator',
            output='screen',
            parameters=[{
                'path_source': 0,          # 0=本机规划(用内置田块顶点)
                'ridge_width': ridge_width,
                'field_margin': field_margin,
                'in_row_resolution': 1.0,
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
