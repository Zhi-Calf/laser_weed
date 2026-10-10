# -*- coding: utf-8 -*-
"""无 ROS 环境下的节点壳烟雾测试。

用桩模块顶替 rclpy/nav_msgs/std_msgs/geometry_msgs/visualization_msgs，
验证 stanley_node 的回调链：
Path -> on_path、Odometry -> on_odom、定时器 -> on_timer -> /cmd_steer 与
/stanley/nearest_point，并检查日志节流与符号约定。
运行：python3 test_node_stub.py
"""

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ---------- 几何/消息桩 ----------
class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z


class _Pose:
    def __init__(self, position=None, orientation=None):
        self.position = position or _Point()
        self.orientation = orientation or _Quat()


class _PoseStamped:
    def __init__(self, pose=None, header=None):
        self.pose = pose or _Pose()
        self.header = header or _Header()


class _Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class _Path:
    def __init__(self, poses=None, header=None):
        self.poses = poses or []
        self.header = header or _Header()


class _PoseCov:
    def __init__(self, pose=None):
        self.pose = pose or _Pose()


class _Linear:
    def __init__(self, x=0.0):
        self.x = x


class _Twist:
    def __init__(self, linear=None):
        self.linear = linear or _Linear()


class _TwistCov:
    def __init__(self, twist=None):
        self.twist = twist or _Twist()


class _Odom:
    def __init__(self, pose=None, twist=None, header=None):
        self.pose = pose or _PoseCov()
        self.twist = twist or _TwistCov()
        self.header = header or _Header()


class _Float64:
    def __init__(self):
        self.data = 0.0


class _Marker:
    SPHERE = 2
    ADD = 0

    def __init__(self):
        self.header = _Header()
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = _Pose()
        self.scale = _Scale()
        self.color = _Color()


class _Scale:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Color:
    def __init__(self):
        self.r = self.g = self.b = self.a = 0.0


# ---------- rclpy 桩 ----------
class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(("info", msg))

    def warn(self, msg, **kw):
        self.lines.append(("warn", msg))


class _Time:
    def to_msg(self):
        return None


class _Clock:
    def __init__(self, node):
        self._node = node

    def now(self):
        class _T:
            def __init__(self, ns):
                self.nanoseconds = ns

            def to_msg(self):
                return None
        return _T(int(self._node.fake_time * 1e9))


class _Pub:
    def __init__(self, topic, log):
        self.topic = topic
        self.log = log

    def publish(self, msg):
        data = msg.data if hasattr(msg, "data") else msg
        self.log.append(data)


class _Param:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, name):
        self._params, self._subs, self._timers = {}, {}, []
        self.pub_log = {}
        self._logger = _Logger()
        self.fake_time = 0.0

    def declare_parameter(self, name, default):
        self._params[name] = _Param(default)

    def get_parameter(self, name):
        return self._params[name]

    def create_subscription(self, mtype, topic, cb, qos):
        self._subs[topic] = cb

    def create_publisher(self, mtype, topic, qos):
        self.pub_log[topic] = []
        return _Pub(topic, self.pub_log[topic])

    def create_timer(self, period, cb):
        self._timers.append((period, cb))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock(self)

    def destroy_node(self):
        pass


def _install_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda *a, **k: None
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    rclpy.node = node_mod

    nav = types.ModuleType("nav_msgs")
    nav_msg = types.ModuleType("nav_msgs.msg")
    nav_msg.Path, nav_msg.Odometry = _Path, _Odom
    nav.msg = nav_msg

    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Float64 = _Float64
    std.msg = std_msg

    geo = types.ModuleType("geometry_msgs")
    geo_msg = types.ModuleType("geometry_msgs.msg")
    geo_msg.Point = _Point
    geo_msg.Quaternion = _Quat
    geo_msg.PoseStamped = _PoseStamped
    geo.msg = geo_msg

    viz = types.ModuleType("visualization_msgs")
    viz_msg = types.ModuleType("visualization_msgs.msg")
    viz_msg.Marker = _Marker
    viz.msg = viz_msg

    sys.modules.update({
        "rclpy": rclpy, "rclpy.node": node_mod,
        "nav_msgs": nav, "nav_msgs.msg": nav_msg,
        "std_msgs": std, "std_msgs.msg": std_msg,
        "geometry_msgs": geo, "geometry_msgs.msg": geo_msg,
        "visualization_msgs": viz, "visualization_msgs.msg": viz_msg,
    })


_install_stubs()
from stanley_control.stanley_node import StanleyNode          # noqa: E402
from stanley_control.stanley_controller import make_path      # noqa: E402


def yaw_to_quat(yaw):
    return _Quat(w=math.cos(yaw / 2.0), z=math.sin(yaw / 2.0))


def main():
    node = StanleyNode()
    # 直线路径
    pts = make_path("straight")
    path = _Path([_PoseStamped(_Pose(_Point(x, y))) for x, y, _ in pts])

    # 路径未就绪时应打印等待日志
    node.fake_time = 0.0
    node.on_timer()
    assert any("等待" in m for _lvl, m in node.get_logger().lines), "未就绪应有等待提示"

    node.on_path(path)
    assert node.path is not None and len(node.path) == len(pts)
    assert any("收到路径" in m for _lvl, m in node.get_logger().lines)

    # 车从 (0, -2, 0) 起步，v = 4 m/s，闭环跑 15 s（步进 fake_time 让日志节流生效）
    x, y, yaw, v, L, dt = 0.0, -2.0, 0.0, 4.0, node.wheelbase, 0.05
    for i in range(int(15.0 / dt)):
        node.fake_time = i * dt
        odom = _Odom(_PoseCov(_Pose(_Point(x, y), yaw_to_quat(yaw))),
                     _TwistCov(_Twist(_Linear(v))))
        node.on_odom(odom)
        node.on_timer()
        steer = node.pub_log["/cmd_steer"][-1] if node.pub_log["/cmd_steer"] else 0.0
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = (yaw + v / L * math.tan(steer) * dt + math.pi) % (2 * math.pi) - math.pi

    e_final = abs(y)
    n_cmd = len(node.pub_log["/cmd_steer"])
    n_marker = len(node.pub_log["/stanley/nearest_point"])
    n_info = sum(1 for lvl, _m in node.get_logger().lines if lvl == "info")
    print("闭环 15s 后横向偏差 |e| = %.4f m" % e_final)
    print("/cmd_steer 发布 %d 条，/stanley/nearest_point 发布 %d 条" % (n_cmd, n_marker))
    print("info 日志 %d 条（15s 内应约 15 条，节流生效）" % n_info)
    assert e_final < 0.05, "节点闭环未收敛"
    assert n_marker == n_cmd, "可视化标记应与指令同频发布"
    assert 12 <= n_info <= 20, "周期日志节流未生效"
    assert node.pub_log["/cmd_steer"][0] > 0.0, "车在路径右侧时首条指令应左转(正)"
    print("桩测试通过：回调链、可视化、日志节流、符号约定均正常。")


if __name__ == "__main__":
    main()
