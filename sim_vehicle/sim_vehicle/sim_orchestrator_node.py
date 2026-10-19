# -*- coding: utf-8 -*-
"""联调仿真编排节点：等待 YAML 田块生成路径，然后切 UNMANNED。

global_path_planner 现在启动时直接从 YAML 读取田块和几何参数，不再需要
`set_source` 服务。仿真链路因此简化为：

    ① global_path_planner 启动后立即按 YAML 田块规划
    ② 本节点确认路径非空后切 UNMANNED，lqr_control 开始驱动

田块、垄宽、田边余量和路径来源都在 global_path_planner.yaml 中配置。
"""

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


WAIT_PATH = 0
GO_UNMANNED = 1
DONE = 2
FAILED = 3

PHASE_NAMES = {
    WAIT_PATH: '等待规划器自动发布路径',
    GO_UNMANNED: '切换 UNMANNED 让控制器接管',
    DONE: '完成',
    FAILED: '失败',
}


class SimOrchestratorNode(Node):
    def __init__(self):
        super().__init__('sim_orchestrator')

        self.declare_parameter('timeout_path_s', 30.0)
        self.timeout_path = float(self.get_parameter('timeout_path_s').value)

        self.qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.phase = WAIT_PATH
        self.phase_start = self.get_clock().now()
        self.got_path_points = 0

        self.path_sub = self.create_subscription(
            Path, '/global_path_planner/path', self.on_path, self.qos_latched)
        self.unmanned_pub = self.create_publisher(
            Bool, '/sim/unmanned', self.qos_latched)

        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            '编排节点已启动：等待 YAML 路径生成后切换 UNMANNED')

    def on_path(self, msg):
        self.got_path_points = len(msg.poses)

    def tick(self):
        if self.phase in (DONE, FAILED):
            return

        elapsed = (
            self.get_clock().now() - self.phase_start).nanoseconds * 1e-9

        if self.phase == WAIT_PATH:
            if self.got_path_points >= 2:
                self.get_logger().info(
                    '规划器已发布路径，共 %d 个点' % self.got_path_points)
                self._advance(GO_UNMANNED)
            elif elapsed > self.timeout_path:
                self._fail(
                    '路径一直没发布（收到 %d 个点）' % self.got_path_points,
                    '检查 global_path_planner.yaml：poly 是否合法、ridge_width 是否 > 0、'
                    'field_margin 是否过大。')

        elif self.phase == GO_UNMANNED:
            msg = Bool()
            msg.data = True
            self.unmanned_pub.publish(msg)
            self.get_logger().info(
                '已切 UNMANNED，lqr_control 现在应该开始驱动。')
            self._advance(DONE)

    def _advance(self, new_phase):
        self.phase = new_phase
        self.phase_start = self.get_clock().now()
        if new_phase == DONE:
            self.get_logger().info('=== 时序完成，仿真已在跑 ===')
            self.get_logger().info(
                '看 RViz：绿箭头应从偏离路径处收敛贴上去，沿弓字形行走；'
                '橙球是 lqr_control 的跟踪目标点，应随车前进。')
        else:
            self.get_logger().info(
                '→ 阶段 %d: %s' % (new_phase, PHASE_NAMES[new_phase]))

    def _fail(self, what, hint):
        self.phase = FAILED
        self.get_logger().error('编排失败：%s' % what)
        self.get_logger().error('排查建议：%s' % hint)


def main():
    rclpy.init()
    node = SimOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
