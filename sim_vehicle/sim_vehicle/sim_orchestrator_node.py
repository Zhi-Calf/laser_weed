# -*- coding: utf-8 -*-
"""联调仿真编排节点：把四段式时序自动化，省掉手动敲命令。

背景（为什么需要它）：global_path_planner 的 require_manual_for_change 默认为 true，
意味着 set_source **只在 MANUAL 模式下才被接受**；而 lqr_control **只在 UNMANNED 下才驱动**。
再加上 set_source 要求 ENU 原点已到位。所以正确顺序必须是：

    ① sim_vehicle 起来（默认 MANUAL，开始发 origin + work_mode）
    ② planner 收到 origin，等 set_source
    ③ MANUAL 下调 set_source → planner 发布 /path
    ④ 切 UNMANNED → lqr_control 才开始驱动

跳步的后果是静默失败：在 UNMANNED 下调 set_source 会返回 accepted=False，
路径永远不生成，而 lqr_control 只会一直刷"等待路径"，很难看出是哪一步错了。

本节点按状态机逐阶段推进，每阶段独立超时，失败时打印**该阶段的具体诊断建议**。
它不关心 sim_vehicle / planner / lqr_control 谁先启动——各阶段都是等条件满足才推进，
所以启动顺序无关。
"""

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from tractor_msgs.srv import SetPathSource

# 默认田块：以 sim_vehicle 的 origin(34.62N, 112.45E) 为中心的 50m(东西) x 30m(南北)
# 矩形，顶点顺序 lat,lon,alt × 4（SW→SE→NE→NW）。算法见 RUN_SIM.md。
DEFAULT_POLY = [
    34.6198653, 112.4497271, 100.0,
    34.6198653, 112.4502729, 100.0,
    34.6201347, 112.4502729, 100.0,
    34.6201347, 112.4497271, 100.0,
]

# 阶段编号
WAIT_ORIGIN = 0
FORCE_MANUAL = 1
WAIT_SERVICE = 2
CALL_SET_SOURCE = 3
WAIT_ACCEPT = 4     # 等服务返回 accepted/reason（不对应用户可见阶段，故不进 PHASE_NAMES）
WAIT_PATH = 5
GO_UNMANNED = 6
DONE = 7
FAILED = 8

PHASE_NAMES = {
    WAIT_ORIGIN: '等待 ENU 原点 (/gnss_driver/origin)',
    FORCE_MANUAL: '确保处于 MANUAL 模式',
    WAIT_SERVICE: '等待 set_source 服务可用',
    CALL_SET_SOURCE: '下发田块边界 (set_source)',
    WAIT_PATH: '等待规划器发布路径',
    GO_UNMANNED: '切换 UNMANNED 让控制器接管',
    DONE: '完成',
    FAILED: '失败',
}


class SimOrchestratorNode(Node):
    def __init__(self):
        super().__init__('sim_orchestrator')

        # ---- 田块与几何参数（对应 SetPathSource 请求字段）----
        self.declare_parameter('poly', DEFAULT_POLY)
        self.declare_parameter('ridge_width', 2.5)
        self.declare_parameter('field_margin', 1.5)
        self.declare_parameter('in_row_resolution', 1.0)
        # path_source: 0=本机规划(用 poly 顶点), 1=读 JSON 文件
        self.declare_parameter('path_source', 0)
        self.declare_parameter('path_file', '')

        # ---- 各阶段超时（秒）----
        self.declare_parameter('timeout_origin_s', 30.0)
        self.declare_parameter('timeout_service_s', 30.0)
        self.declare_parameter('timeout_path_s', 30.0)

        self.poly = [float(v) for v in self.get_parameter('poly').value]
        self.ridge_width = float(self.get_parameter('ridge_width').value)
        self.field_margin = float(self.get_parameter('field_margin').value)
        self.in_row_resolution = float(self.get_parameter('in_row_resolution').value)
        self.path_source = int(self.get_parameter('path_source').value)
        self.path_file = str(self.get_parameter('path_file').value)

        self.timeout_origin = float(self.get_parameter('timeout_origin_s').value)
        self.timeout_service = float(self.get_parameter('timeout_service_s').value)
        self.timeout_path = float(self.get_parameter('timeout_path_s').value)

        # ---- 参数合法性预检（在动任何 ROS 通信之前就拦下配置错误）----
        if not self._precheck_params():
            self.phase = FAILED
            return

        # ---- QoS：origin 与 path 都是 transient_local(latched) 发布，
        #      订阅端必须匹配 durability，否则晚加入就永远收不到那条已发布的消息。
        self.qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.phase = WAIT_ORIGIN
        self.phase_start = self.get_clock().now()
        self.got_origin = False
        self.got_path_points = 0
        self.set_source_future = None

        self.origin_sub = self.create_subscription(
            NavSatFix, '/gnss_driver/origin', self.on_origin, self.qos_latched)
        self.path_sub = self.create_subscription(
            Path, '/global_path_planner/path', self.on_path, self.qos_latched)
        # /sim/unmanned 是 sim_vehicle 的私有控制话题，它收到后自己以 1Hz
        # 持续转发 /work_mode，所以这里 --once 语义即可（用 latch 让它晚启动也能收到）
        self.unmanned_pub = self.create_publisher(
            Bool, '/sim/unmanned', self.qos_latched)

        self.cli = self.create_client(SetPathSource, '/global_path_planner/set_source')

        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info(
            '编排节点已启动，开始按四段式时序推进。田块: path_source=%d 垄宽=%.2fm '
            '田边余量=%.2fm 行内分辨率=%.2fm 顶点数=%d'
            % (self.path_source, self.ridge_width, self.field_margin,
               self.in_row_resolution, len(self.poly) // 3))

    # ============================================================
    # 参数预检
    # ============================================================
    def _precheck_params(self):
        """把 set_source 一定会拒收的配置在本地先拦下，省一轮无效调用。"""
        ok = True

        if self.path_source == 0:
            n = len(self.poly)
            if n < 9 or n % 3 != 0:
                self.get_logger().error(
                    'poly 顶点数组长度必须是 3 的倍数且至少 3 个顶点(9 个数)，当前 %d 个数' % n)
                ok = False
            elif n // 3 < 3:
                self.get_logger().error('田块至少需要 3 个顶点，当前 %d 个' % (n // 3))
                ok = False
            else:
                for i in range(0, n, 3):
                    lat, lon = self.poly[i], self.poly[i + 1]
                    if not (abs(lat) <= 90.0 and abs(lon) <= 180.0):
                        self.get_logger().error(
                            '第 %d 个顶点经纬度越界: lat=%.6f lon=%.6f'
                            % (i // 3, lat, lon))
                        ok = False

            if self.ridge_width <= 0.0:
                self.get_logger().error(
                    'ridge_width 必须 > 0（当前 %.3f），否则垄线重叠退化' % self.ridge_width)
                ok = False
            if self.field_margin < 0.0:
                self.get_logger().error('field_margin 不能为负（当前 %.3f）' % self.field_margin)
                ok = False

            # ⚠️ 这条是最容易踩的坑：弓字形地头用半圆掉头，半径 = 垄宽/2，会从行末端
            # 向外鼓出去；而 field_margin 是唯一的内缩量。若 field_margin < ridge_width/2，
            # 掉头段必然超出田块边界 —— 2026-10-15 仿真里绿箭飞出田块就是这个原因
            # （当时 ridge_width=2.5 → 掉头半径 1.25m，而 field_margin 只有 0.5m）。
            if self.field_margin < self.ridge_width / 2.0:
                self.get_logger().warn(
                    'field_margin(%.2fm) < ridge_width/2(%.2fm)：地头半圆掉头会超出田块边界约 '
                    '%.2fm。规划器不会报错，但路径会画到田外。建议 field_margin >= ridge_width/2。'
                    % (self.field_margin, self.ridge_width / 2.0,
                       self.ridge_width / 2.0 - self.field_margin))

        elif self.path_source == 1:
            if not self.path_file.startswith('/'):
                self.get_logger().error(
                    'path_source=1(file) 时 path_file 必须是绝对路径，当前 "%s"' % self.path_file)
                ok = False
        else:
            self.get_logger().error(
                'path_source 只能是 0(local) 或 1(file)，当前 %d（旧值 2 已废弃会被拒收）'
                % self.path_source)
            ok = False

        # in_row_resolution: 0 = 不产出 pto_path，或必须落在 [0.01, 100]
        if self.in_row_resolution != 0.0 and not (0.01 <= self.in_row_resolution <= 100.0):
            self.get_logger().error(
                'in_row_resolution 必须是 0.0 或落在 [0.01, 100] m 内，当前 %.4f'
                % self.in_row_resolution)
            ok = False

        return ok

    # ============================================================
    # 回调
    # ============================================================
    def on_origin(self, msg):
        if not self.got_origin:
            self.got_origin = True
            self.get_logger().info(
                '收到 ENU 原点: lat=%.7f lon=%.7f alt=%.1f'
                % (msg.latitude, msg.longitude, msg.altitude))

    def on_path(self, msg):
        self.got_path_points = len(msg.poses)

    # ============================================================
    # 状态机
    # ============================================================
    def tick(self):
        if self.phase in (DONE, FAILED):
            return

        now = self.get_clock().now()
        # rclpy Duration has NO .seconds() method (that is the C++ rclcpp API).
        # Python exposes only .nanoseconds, so convert explicitly. Using
        # .seconds() here raised AttributeError and killed the node on its
        # first tick, which silently broke the whole four-phase sequence.
        elapsed = (now - self.phase_start).nanoseconds * 1e-9

        if self.phase == WAIT_ORIGIN:
            if self.got_origin:
                self._advance(FORCE_MANUAL)
            elif elapsed > self.timeout_origin:
                self._fail('等不到 /gnss_driver/origin',
                           '检查 sim_vehicle 是否在跑：ros2 node list | grep sim_vehicle。'
                           'planner 的 origin_timeout_s=3.0，收不到原点它不会规划。')

        elif self.phase == FORCE_MANUAL:
            # 先发一次 MANUAL，确保下一步的 set_source 不会被 require_manual_for_change 拒收。
            # 用户之前可能手动切过 UNMANNED，不重置的话服务调用会静默失败。
            b = Bool()
            b.data = False
            self.unmanned_pub.publish(b)
            self.get_logger().info('已请求 MANUAL 模式（set_source 只在手动模式下被接受）')
            self._advance(WAIT_SERVICE)

        elif self.phase == WAIT_SERVICE:
            if self.cli.service_is_ready():
                self._advance(CALL_SET_SOURCE)
            elif elapsed > self.timeout_service:
                self._fail('set_source 服务不可用',
                           '检查 global_path_planner 是否在跑：'
                           'ros2 service list | grep set_source')

        elif self.phase == CALL_SET_SOURCE:
            req = SetPathSource.Request()
            req.path_source = self.path_source
            req.path_file = self.path_file
            req.ridge_width = self.ridge_width
            req.field_margin = self.field_margin
            req.in_row_resolution = self.in_row_resolution
            if self.path_source == 0:
                req.poly_count = len(self.poly) // 3
                req.poly = self.poly

            self.set_source_future = self.cli.call_async(req)
            self.get_logger().info('已下发 set_source 请求，等待受理结果…')
            self._advance(WAIT_ACCEPT, quiet=True)

        elif self.phase == WAIT_ACCEPT:
            fut = self.set_source_future
            if fut is not None and fut.done():
                resp = fut.result()
                if resp.accepted:
                    self.get_logger().info('set_source 已受理: %s' % resp.reason)
                    self._advance(WAIT_PATH)
                else:
                    self._fail('set_source 被拒收: %s' % resp.reason,
                               '最常见原因：① 当前不是 MANUAL 模式（本节点已尝试重置，'
                               '但 sim_vehicle 可能没收到）；② ridge_width/field_margin/'
                               'in_row_resolution 取值不在允许域内。reason 里写明了是哪一条。')
            elif elapsed > self.timeout_service:
                self._fail('set_source 调用超时',
                           '服务存在但没响应，planner 可能卡在规划里。看它的终端输出。')

        elif self.phase == WAIT_PATH:
            if self.got_path_points >= 2:
                self.get_logger().info('规划器已发布路径，共 %d 个点' % self.got_path_points)
                self._advance(GO_UNMANNED)
            elif elapsed > self.timeout_path:
                self._fail('路径一直没发布（收到 %d 个点）' % self.got_path_points,
                           'set_source 受理了但没排出路径，通常是几何原因：垄宽过大、'
                           'field_margin 吃掉了整块地、或多边形非凸。看 planner 终端的'
                           '心跳故障码 10404..10410 与它的日志。')

        elif self.phase == GO_UNMANNED:
            b = Bool()
            b.data = True
            self.unmanned_pub.publish(b)
            self.get_logger().info(
                '已切 UNMANNED，lqr_control 现在应该开始驱动。')
            self._advance(DONE)

        elif self.phase == DONE:
            pass

    def _advance(self, new_phase, quiet=False):
        self.phase = new_phase
        self.phase_start = self.get_clock().now()
        if not quiet:
            name = PHASE_NAMES.get(new_phase, str(new_phase))
            if new_phase == DONE:
                self.get_logger().info('=== 时序完成，仿真已在跑 ===')
                self.get_logger().info(
                    '现在看 RViz：绿箭头(仿真车)应从偏离路径 0.8m 处收敛贴上去，'
                    '沿弓字形一行行走，地头做半圆掉头。橙球是 lqr_control 的跟踪目标点，'
                    '它应该跟着车一起前进——橙球不动说明目标点没推进。')
                self.get_logger().info(
                    '若车偏离目标超过 max_track_error_m(默认3m)，lqr_control 会停车并报 10106。')
            else:
                self.get_logger().info('→ 阶段 %d: %s' % (new_phase, name))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
