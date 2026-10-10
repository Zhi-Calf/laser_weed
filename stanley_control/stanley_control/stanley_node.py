# -*- coding: utf-8 -*-
"""Stanley 控制 ROS2 薄壳节点（激光除草机器人项目）

只做三件事：订阅路径/里程计 -> 调纯算法 stanley_controller -> 发布转角指令。
算法零 ROS 依赖，本文件不含任何控制数学，便于与孙勇军的路径规划、
laser_weed_can 的 CAN 桥各自独立演进。

话题约定：
    订阅 /path     nav_msgs/Path      全局路径（规划节点发布）
    订阅 /odom     nav_msgs/Odometry  里程计（位姿 + 车体线速度）
    发布 /cmd_steer std_msgs/Float64  前轮转角指令，弧度，左转(逆时针)为正

下游 CAN 桥接注意（见 laser_weed_can 协议第二版）：
    ADCU_toVCU_01 目标转向角单位 0.1 度、左正右负 —— 与本节点符号同向，
    无需取反，只做 弧度 -> 0.1 度 换算；指令帧 20ms 周期、带 alive 计数。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from .stanley_controller import StanleyController


class StanleyNode(Node):
    def __init__(self):
        super().__init__("stanley_control")
        self.declare_parameter("wheelbase", 1.0)    # 轴距 m，按实车改
        self.declare_parameter("k", 1.2)            # 横向偏差增益
        self.declare_parameter("k_soft", 0.1)       # 低速软化项
        self.declare_parameter("control_hz", 50.0)  # 控制频率，CAN 侧 20ms 可提到 50Hz
        self.declare_parameter("viz", True)         # 是否发布 RViz 可视化标记
        self.declare_parameter("frame_id", "map")   # 可视化坐标系

        self.ctl = StanleyController(k=self.get_parameter("k").value,
                                     k_soft=self.get_parameter("k_soft").value)
        self.wheelbase = self.get_parameter("wheelbase").value
        self.viz = self.get_parameter("viz").value
        self.frame_id = self.get_parameter("frame_id").value
        self.path = None        # [(x, y, yaw), ...]
        self.pose = None        # (x, y, yaw) 后轴
        self.v = 0.0
        self._last_log = -1e9   # 日志节流用的时间戳（初始极小值：首次立即提示）

        self.create_subscription(Path, "/path", self.on_path, 1)
        self.create_subscription(Odometry, "/odom", self.on_odom, 1)
        self.pub = self.create_publisher(Float64, "/cmd_steer", 10)
        self.pub_nearest = self.create_publisher(Marker, "/stanley/nearest_point", 10)
        self.create_timer(1.0 / self.get_parameter("control_hz").value, self.on_timer)

    def on_path(self, msg):
        """Path -> [(x, y, yaw)]，航向由相邻点切线给出。"""
        if not msg.poses:
            return
        pts = []
        for i, p in enumerate(msg.poses):
            x, y = p.pose.position.x, p.pose.position.y
            if i + 1 < len(msg.poses):
                q = msg.poses[i + 1].pose.position
                yaw = math.atan2(q.y - y, q.x - x)
            else:
                yaw = pts[-1][2] if pts else 0.0
            pts.append((x, y, yaw))
        new = self.path is None
        self.path = pts
        if new:
            self.get_logger().info(
                "收到路径 %d 个点，起点 (%.1f, %.1f)，开始控制"
                % (len(pts), pts[0][0], pts[0][1]))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        first = self.pose is None
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.v = msg.twist.twist.linear.x   # 车体系前进速度 m/s
        if first:
            self.get_logger().info("收到里程计，位姿 (%.1f, %.1f)，速度 %.2f m/s"
                                   % (self.pose[0], self.pose[1], self.v))

    def on_timer(self):
        if self.path is None or self.pose is None:
            # 数据未就绪：每 3 秒提示一次等待原因，不刷屏
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_log > 3.0:
                self._last_log = now
                if self.path is None and self.pose is None:
                    self.get_logger().warn("等待 /path 与 /odom ...（可先起 sim_demo）")
                elif self.path is None:
                    self.get_logger().warn("等待 /path（可先起 sim_demo 或规划节点）...")
                else:
                    self.get_logger().warn("等待 /odom ...")
            return
        x, y, yaw = self.pose
        steer, e, idx = self.ctl.compute(self.path, x, y, yaw, self.v, self.wheelbase)
        msg = Float64()
        msg.data = steer  # 弧度，左正；CAN 桥换算成 0.1 度
        self.pub.publish(msg)
        if self.viz:
            self.pub_nearest.publish(self._nearest_marker(idx, e))
        # 周期日志（约 1 秒一条，避免刷屏）
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log > 1.0:
            self._last_log = now
            self.get_logger().info(
                "e=%+.2f m  steer=%+.1f deg  v=%.2f m/s  路径进度 %d/%d"
                % (e, math.degrees(steer), self.v, idx + 1, len(self.path)))

    def _nearest_marker(self, idx, e):
        """RViz 球标记：路径最近点（绿）位置随车移动，颜色深度反映偏差。"""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "stanley_nearest"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        p = Point()
        p.x, p.y, p.z = self.path[idx][0], self.path[idx][1], 0.1
        m.pose.position = p
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.6   # 直径 0.6 m，俯视缩放下仍醒目
        m.color.r, m.color.g, m.color.b = 1.0, 0.8, 0.0   # 橙色，直观醒目
        m.color.a = 1.0
        return m


def main():
    rclpy.init()
    node = StanleyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
