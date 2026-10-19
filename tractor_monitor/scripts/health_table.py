#!/usr/bin/env python3
"""一屏看完所有被监控模块的状态。

    ros2 run tractor_monitor health_table.py

订阅 /system_health（tractor_monitor 发布的全表快照）并原地刷新成一张表。
`ros2 topic echo /system_health` 也能看, 只是一个模块要占十行, 而且
"从未启动"和"启动过又死了"在原始输出里都是 status: 2 —— 得自己去看 ever_seen
才分得清。这里把这件事替你做了。
"""

import sys

import rclpy
from rclpy.node import Node

from weeder_msgs.msg import NodeHealth, SystemHealth

RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
DIM = "\033[2m"

CRIT = {
    NodeHealth.CRIT_CRITICAL: "critical",
    NodeHealth.CRIT_DEGRADED: "degraded",
    NodeHealth.CRIT_IGNORE: "ignore",
}


def render(n: NodeHealth, color: bool):
    """一行。返回 (状态文字, 颜色, 备注)。"""
    if not n.ever_seen:
        # 从未启动。对 required=false 的模块来说这完全正常 —— bringup.launch.py
        # 本来就不启动它们。所以不要标红。
        return "NOT STARTED", DIM, "未启动"
    if not n.alive:
        return "DEAD", RED, f"心跳中断 {n.age_s:.1f}s"
    if n.status == NodeHealth.STATUS_FAULT:
        return "FAULT", RED, n.detail
    if n.status == NodeHealth.STATUS_STOP_MOTION:
        return "STOP_MOTION", YELLOW, n.detail
    if n.status == NodeHealth.STATUS_STOP_IMPLEMENT:
        return "STOP_IMPLEMENT", YELLOW, n.detail
    return "OK", GREEN, n.detail


class HealthTable(Node):
    def __init__(self, color: bool):
        super().__init__('tractor_health_table')
        self.color_ = color
        self.create_subscription(SystemHealth, '/system_health', self._on_health, 10)
        self.get_logger().info('waiting for /system_health ...')

    def _c(self, s, col):
        return f"{col}{s}{RESET}" if self.color_ else s

    def _on_health(self, msg: SystemHealth):
        overall = {
            SystemHealth.STATUS_OK: ("OK", GREEN),
            SystemHealth.STATUS_STOP_IMPLEMENT: ("STOP_IMPLEMENT", YELLOW),
            SystemHealth.STATUS_STOP_MOTION: ("STOP_MOTION", YELLOW),
            SystemHealth.STATUS_FAULT: ("FAULT", RED),
        }.get(msg.overall, ("?", RESET))

        out = ["\033[H\033[J" if self.color_ else ""]
        grace = "  (启动宽限期内)" if msg.in_startup_grace else ""
        # overall 就是 tractor_base 消费的那个裁决 —— FAULT 时底盘停车并置急停。
        out.append(f"{self._c('系统总状态', BOLD)}: "
                   f"{self._c(overall[0], overall[1])}{grace}")
        out.append(f"{self._c('FAULT 时底盘停车并置急停 (tractor_base 消费 /monitor_heartbeat)', DIM)}")
        out.append("")
        out.append(f"{'NODE':<28} {'STATUS':<12} {'CRIT':<9} {'AGE':>7}  {'CNT':<4} NOTE")
        out.append("-" * 100)

        for n in sorted(msg.nodes, key=lambda x: x.node_name):
            label, col, note = render(n, self.color_)
            age = f"{n.age_s:.2f}s" if n.ever_seen else "-"
            # counted=no 解释了"这行是 FAULT, 车却没停"
            cnt = "yes" if n.counted else "no"
            if n.restarts:
                note = f"{note}  [重启过 {n.restarts} 次]"
            out.append(f"{n.node_name:<28} {self._c(f'{label:<12}', col)} "
                       f"{CRIT.get(n.criticality, '?'):<9} {age:>7}  {cnt:<4} {note}")

        out.append("")
        out.append(self._c("CRIT=critical 的模块 DEAD/FAULT 且 CNT=yes -> 整车停车并置急停", DIM))
        print("\n".join(out), flush=True)


def main():
    color = sys.stdout.isatty() and '--no-color' not in sys.argv
    rclpy.init()
    node = HealthTable(color)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
