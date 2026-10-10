# -*- coding: utf-8 -*-
"""仿真数据源节点：无实车/无规划节点时，给 stanley_node 喂数据看效果。

发布：
    /path  nav_msgs/Path      一条正弦参考路径（固定，1Hz 重发防丢）
    /odom  nav_msgs/Odometry  自行车模型仿真车体状态（订阅 /cmd_steer 驱动）
订阅：
    /cmd_steer std_msgs/Float64  前轮转角指令（弧度，左正）

配合 RViz2：添加 Display -> Path（话题 /path）与 Marker（话题
/stanley/nearest_point），Fixed Frame 填 map，即可看到车沿路径收敛。
实车联调时不起本节点即可，stanley_node 无需任何改动。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class SimDemo(Node):
    def __init__(self):
        super().__init__("sim_demo")
        self.declare_parameter("speed", 2.0)        # 仿真车速 m/s
        self.declare_parameter("wheelbase", 1.0)    # 与 stanley_node 保持一致
        self.declare_parameter("x0", 0.0)           # 初始位置（故意偏离路径看收敛）
        self.declare_parameter("y0", -1.5)
        self.declare_parameter("yaw0", 0.3)

        self.v = self.get_parameter("speed").value
        self.L = self.get_parameter("wheelbase").value
        self.x0 = self.get_parameter("x0").value
        self.y0 = self.get_parameter("y0").value
        self.yaw0 = self.get_parameter("yaw0").value
        self.x, self.y, self.yaw = self.x0, self.y0, self.yaw0
        self.steer = 0.0

        self.pub_path = self.create_publisher(Path, "/path", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Float64, "/cmd_steer", self.on_steer, 10)
        self.create_timer(1.0, self.send_path)       # 周期重发路径
        self.create_timer(0.02, self.step)           # 50Hz 仿真步进
        self.get_logger().info(
            "仿真数据源已启动：/path + /odom（车初始 (%.1f, %.1f)）" % (self.x, self.y))

    def on_steer(self, msg):
        self.steer = msg.data

    def send_path(self):
        """正弦路径：y = 1.5*sin(0.15*x)，x 取 0~80m、间隔 0.5m。"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        n = 161
        for i in range(n):
            x = 80.0 * i / (n - 1)
            y = 1.5 * math.sin(0.15 * x)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=x, y=y, z=0.0)
            ps.pose.orientation = Quaternion(w=1.0)
            path.poses.append(ps)
        self.pub_path.publish(path)

    def step(self):
        """自行车模型积分（dt=0.02s），发布仿真里程计；跑完路径自动复位重跑。"""
        dt = 0.02
        if self.x >= 80.0:
            self.x, self.y, self.yaw = self.x0, self.y0, self.yaw0
            self.steer = 0.0
            self.get_logger().info("跑完一轮，复位到初始位姿重跑")
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = (self.yaw + self.v / self.L * math.tan(self.steer) * dt
                    + math.pi) % (2 * math.pi) - math.pi

        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = Quaternion(
            w=math.cos(self.yaw / 2.0), z=math.sin(self.yaw / 2.0))
        odom.twist.twist.linear.x = self.v
        self.pub_odom.publish(odom)


def main():
    rclpy.init()
    node = SimDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
