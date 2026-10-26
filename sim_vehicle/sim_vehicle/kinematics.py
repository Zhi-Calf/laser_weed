# -*- coding: utf-8 -*-
"""理想四轮转向底盘运动学。

几何参数与 Laser-Weeding-Sim 的 Isaac 场景保持一致。路径跟踪目前只使用
steer_mode 0（单阿克曼）和 1（对称阿克曼）；模式 2/3 的速度语义尚未在
ChassisCmd 中定义，因此由节点按停车处理。
"""

import math
from dataclasses import dataclass


WHEELBASE_M = 1.521
WHEEL_TRACK_M = 1.306
WHEEL_RADIUS_M = 0.29
MAX_SPEED_M_S = 1.4
MAX_STEER_RAD = math.pi / 2.0

CORNERS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class FourWheelState:
    """四轮转向角和轮速；轮速为绕轮轴角速度，单位 rad/s。"""

    steer_rad: dict
    wheel_rad_s: dict
    yaw_rate_rad_s: float
    effective_wheelbase_m: float


def _clamp(value, lower, upper):
    return max(lower, min(value, upper))


def _wheel_velocity(speed_m_s, yaw_rate_rad_s, x_m, y_m):
    """刚体平面上，位置 (x_m, y_m) 处的速度向量。"""
    return speed_m_s - yaw_rate_rad_s * y_m, yaw_rate_rad_s * x_m


def four_wheel_state(
    speed_m_s,
    steer_angle_rad,
    steer_mode,
    wheelbase_m=WHEELBASE_M,
    wheel_track_m=WHEEL_TRACK_M,
    wheel_radius_m=WHEEL_RADIUS_M,
    max_steer_rad=MAX_STEER_RAD,
):
    """由底盘统一指令计算理想四轮转向状态。

    位姿积分仍采用 path_tracker 的等效自行车模型：

        mode 0:  yaw_rate = v * tan(delta) / L
        mode 1:  yaw_rate = v * tan(delta) / (L / 2)

    每个轮子的转角和轮速由该刚体运动反解得到。mode 0 的参考点为后轴中心，
    mode 1 的参考点为几何中心，与对应阿克曼模型一致。
    """
    if steer_mode not in (0, 1):
        return FourWheelState(
            steer_rad={corner: 0.0 for corner in CORNERS},
            wheel_rad_s={corner: 0.0 for corner in CORNERS},
            yaw_rate_rad_s=0.0,
            effective_wheelbase_m=0.0,
        )

    speed_m_s = max(0.0, float(speed_m_s))
    steer_angle_rad = _clamp(
        float(steer_angle_rad), -max_steer_rad, max_steer_rad
    )
    wheelbase_m = max(float(wheelbase_m), 1e-3)
    wheel_track_m = max(float(wheel_track_m), 1e-3)
    wheel_radius_m = max(float(wheel_radius_m), 1e-3)

    effective_wheelbase_m = (
        wheelbase_m if steer_mode == 0 else wheelbase_m * 0.5
    )
    yaw_rate_rad_s = (
        0.0
        if abs(steer_angle_rad) < 1e-12
        else speed_m_s
        * math.tan(steer_angle_rad)
        / effective_wheelbase_m
    )

    if steer_mode == 0:
        # 单阿克曼以车辆后轴中心为参考点。
        positions = {
            "FL": (wheelbase_m, wheel_track_m * 0.5),
            "FR": (wheelbase_m, -wheel_track_m * 0.5),
            "RL": (0.0, wheel_track_m * 0.5),
            "RR": (0.0, -wheel_track_m * 0.5),
        }
    else:
        # 对称阿克曼以前后轴几何中心为参考点。
        half_wheelbase = wheelbase_m * 0.5
        positions = {
            "FL": (half_wheelbase, wheel_track_m * 0.5),
            "FR": (half_wheelbase, -wheel_track_m * 0.5),
            "RL": (-half_wheelbase, wheel_track_m * 0.5),
            "RR": (-half_wheelbase, -wheel_track_m * 0.5),
        }

    steer = {}
    wheel_rad_s = {}
    for corner in CORNERS:
        x_m, y_m = positions[corner]
        vx_m_s, vy_m_s = _wheel_velocity(
            speed_m_s, yaw_rate_rad_s, x_m, y_m
        )

        if steer_mode == 0 and corner.startswith("R"):
            wheel_steer_rad = 0.0
            wheel_ground_speed = vx_m_s
        elif abs(vx_m_s) < 1e-12 and abs(vy_m_s) < 1e-12:
            wheel_steer_rad = 0.0
            wheel_ground_speed = 0.0
        else:
            wheel_steer_rad = math.atan2(vy_m_s, vx_m_s)
            wheel_steer_rad = _clamp(
                wheel_steer_rad, -max_steer_rad, max_steer_rad
            )
            wheel_ground_speed = math.hypot(vx_m_s, vy_m_s)

        steer[corner] = wheel_steer_rad
        wheel_rad_s[corner] = wheel_ground_speed / wheel_radius_m

    return FourWheelState(
        steer_rad=steer,
        wheel_rad_s=wheel_rad_s,
        yaw_rate_rad_s=yaw_rate_rad_s,
        effective_wheelbase_m=effective_wheelbase_m,
    )
