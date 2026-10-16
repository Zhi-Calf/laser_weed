"""
启动 gnss_driver 节点（双天线 MEMS 组合导航定位驱动）。

gnss_driver 解析 $GIAVP 语句，发布原始经纬度（~/fix）、修正后的里程计
（~/odom）、原始里程计（~/odom_raw，仅供标定）、IMU 姿态（~/imu）、ENU 速度（~/vel）、
GNSS 状态（~/status），以及 ENU 原点（~/origin，上电后首次 RTK 固定解时设定，
之后以 1 Hz 重复发布）。

本节点实时发布 TF `odom -> base_link`，由标定杆臂（calib_offset_x/y）与航向偏角
（calib_offset_heading）构成 —— 杆臂只有这一个来源（不再有"名义杆臂 device_offset_*"），
标定后写回即生效。修正后的里程计（~/odom）已是 ENU 且含同样修正；path_controller
直接订阅它。
~/odom_raw 是不加任何修正的同一份数据（设备中心 gnss_link 位置 + 原始测向航向），
只给 tractor_calibration 标定使用（标定要测物理真值），不发 TF。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gnss_localization')
    default_params = os.path.join(pkg_dir, 'config', 'gnss_driver.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)
    # 参数持久化写回目标: 默认指向运行时加载的 params_file。
    # 经 bringup 启动时, bringup 会显式传 gnss_config_path 指向源配置文件。
    gnss_config_path = LaunchConfiguration('gnss_config_path', default=params_file)

    gnss_driver = Node(
        package='gnss_localization',
        executable='gnss_driver_node',
        name='gnss_driver',
        namespace='',
        parameters=[
            params_file,
            {
                # 使 onSetParameters 动态更新的标定参数能写回并重启后生效。
                'gnss_config_path': gnss_config_path,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='GNSS driver params YAML (default: config/gnss_driver.yaml)',
        ),
        DeclareLaunchArgument(
            'gnss_config_path',
            default_value=params_file,
            description='YAML path to persist runtime-calibrated params back to '
                        '(default: same as params_file)',
        ),
        gnss_driver,
    ])
