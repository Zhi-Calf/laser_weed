# -*- coding: utf-8 -*-
"""仿真车辆节点（激光除草机联调仿真用）

替代真实的 gnss_localization + 底盘，构成一个闭环：

    path_tracker ──/chassis/cmd──► sim_vehicle ──/gnss_driver/pose──► path_tracker
                                      └─TF odom->base_link─►

同时模拟 gnss_driver 的两个前置输出，让 global_path_planner 能工作：
    /gnss_driver/origin  (NavSatFix, latched)  ENU 投影原点
    /work_mode           (WorkMode, latched)   手动/无人模式切换

运动学：按转向模式选择等效轴距的自行车模型
    x'   = v·cos(yaw)
    y'   = v·sin(yaw)
    yaw' = (v/L_eff)·tan(delta)

    mode 0：L_eff = L（单阿克曼）
    mode 1：L_eff = L/2（前后轮反向等角的对称阿克曼）

⚠️ 这是**理想模型**：无轮胎侧偏、无执行器延迟、无噪声、无打滑。
它验证的是"控制律与话题链路是否正确"，**不能**用来评估实车跟踪精度。
实车精度取决于 RTK 延迟、底盘响应、地面条件，这些仿真里都没有。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from weeder_msgs.msg import ChassisCmd, Path, Pose, WorkMode
from visualization_msgs.msg import Marker

TWO_PI = 2.0 * math.pi


def wrap_angle(a):
    """归一化到 [-pi, pi)。与 path_tracker / stanley 的约定一致。"""
    return (a + math.pi) % TWO_PI - math.pi


class SimVehicleNode(Node):
    def __init__(self):
        super().__init__("sim_vehicle")

        # ---- 车辆参数：wheelbase 必须与 path_tracker 的 wheelbase 一致，
        #      否则仿真的"真实车辆"和控制器假设的模型不是同一台车，
        #      跟踪误差会收敛到一个非零稳态值，看起来像控制器调不好。
        self.declare_parameter("wheelbase", 1.2)
        self.declare_parameter("sim_hz", 50.0)
        self.declare_parameter("max_speed_m_s", 1.4)
        # 自行车模型在 delta -> ±90° 时 tan() 发散。真实车辆也不会边跑边打满
        # 90°，所以这里夹到 60°，既避免 inf/nan 又不至于掩盖控制器的异常输出。
        self.declare_parameter("max_plant_steer_deg", 60.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("cmd_topic", "/chassis/cmd")

        # ---- 初始位姿（ENU，单位 m）。默认在原点，即田块西南角附近。
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 0.0)
        self.declare_parameter("init_yaw", 0.0)

        # ---- 模拟 GNSS 原点（ENU 投影基准）。默认与 planner YAML 的 poly
        #      首个顶点一致，保证仿真里 odom 与 path 使用同一坐标基准。
        self.declare_parameter("origin_lat", 34.6198653)
        self.declare_parameter("origin_lon", 112.4497271)
        self.declare_parameter("origin_alt", 100.0)

        # ---- 工作模式：false = MANUAL（规划器自动规划），
        #      true = UNMANNED（允许 path_tracker 驱动底盘）。
        #      发 /sim/unmanned 切换。见 RUN_SIM.md 的时序说明。
        self.declare_parameter("start_unmanned", False)

        # ---- 自动对齐到路径起点 ----
        #      默认 true：收到第一条 /global_path_planner/path 后，把车瞬移到
        #      路径起点附近（并留一个可配的初始横向偏差），再开始仿真。
        #      否则车从原点 (0,0) 出发，而田块路径可能在几十米外，初始横向误差
        #      巨大，前几秒的跟踪曲线看起来像控制器失灵。
        #      设为 false 则用 init_x/init_y/init_yaw 指定的绝对位姿。
        self.declare_parameter("align_to_path_start", True)
        # 对齐时故意留的初始横向偏差（m），用来观察控制器收敛。0 = 完美对齐。
        self.declare_parameter("align_lateral_offset", 0.8)

        self.wheelbase = max(self.get_parameter("wheelbase").value, 0.05)
        self.dt = 1.0 / max(self.get_parameter("sim_hz").value, 1.0)
        self.max_speed = self.get_parameter("max_speed_m_s").value
        self.max_plant_steer = math.radians(
            max(self.get_parameter("max_plant_steer_deg").value, 1.0))
        self.cmd_timeout_s = self.get_parameter("cmd_timeout_s").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        # ---- 车辆状态
        self.x = self.get_parameter("init_x").value
        self.y = self.get_parameter("init_y").value
        self.yaw = wrap_angle(self.get_parameter("init_yaw").value)
        self.v = 0.0            # 当前实际速度（仿真里等于指令速度）
        self.v_cmd = 0.0
        self.delta_cmd = 0.0
        self.steer_mode = 0
        self.unmanned = self.get_parameter("start_unmanned").value
        self.steer_clamped_count = 0
        self.align_to_path = self.get_parameter("align_to_path_start").value
        self.align_offset = self.get_parameter("align_lateral_offset").value
        self.aligned = False   # 是否已完成对齐（只对齐一次）

        self.last_cmd_time = self.get_clock().now()

        # ---- QoS：必须与下游订阅端匹配，否则一条都收不到 ----
        # path_tracker 用 SensorDataQoS 订阅 pose
        self.qos_sensor = qos_profile_sensor_data
        # planner 与 path_tracker 都用 transient_local + reliable 订阅这两个话题
        self.qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ---- 订阅（控制指令）----
        self.create_subscription(
            ChassisCmd, self.cmd_topic, self.on_chassis_cmd, 10)
        self.create_subscription(
            Bool, "/sim/unmanned", self.on_unmanned, 10)
        # 仅用于"对齐到路径起点"，仿真本身不跟路径（那是 path_tracker 的事）。
        # QoS 必须与 planner 的发布端匹配：transient_local + reliable。
        self.create_subscription(
            Path, "/global_path_planner/path", self.on_path, self.qos_latched)

        # ---- 发布（模拟 gnss_driver 的输出）----
        self.pose_pub = self.create_publisher(
            Pose, "/gnss_driver/pose", self.qos_sensor)
        self.pose_stamped_pub = self.create_publisher(
            PoseStamped, "/gnss_driver/pose_stamped", self.qos_sensor)
        self.origin_pub = self.create_publisher(
            NavSatFix, "/gnss_driver/origin", self.qos_latched)
        self.work_mode_pub = self.create_publisher(
            WorkMode, "/work_mode", self.qos_latched)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.marker_pub = self.create_publisher(
            Marker, "/sim_vehicle/marker", 10)

        # ---- 定时器：物理积分 + pose 发布（同频）----
        self.create_timer(self.dt, self.tick)
        # ---- 定时器：latched 话题 1Hz 重发（模拟 gnss_driver 与 APP 行为，
        #      也让 planner/lqr 的超时判定不会误触发）----
        self.create_timer(1.0, self.publish_slow_topics)

        self.publish_slow_topics()   # 立刻发一次，免得下游等 1 秒

        self.get_logger().info(
            "仿真车辆已启动: 轴距=%.3fm dt=%.3fs 初始位姿=(%.2f, %.2f, %.1f°) "
            "模式=%s"
            % (self.wheelbase, self.dt, self.x, self.y,
               math.degrees(self.yaw),
               "UNMANNED" if self.unmanned else "MANUAL"))
        self.get_logger().info(
            "订阅 /chassis/cmd + /sim/unmanned | "
            "发布 /gnss_driver/pose + /gnss_driver/origin + /work_mode + /tf")

    # ============================================================
    # 回调
    # ============================================================
    def on_chassis_cmd(self, msg):
        """速度与转角来自同一控制周期，保证仿真车辆执行一致的运动学组合。"""
        self.last_cmd_time = self.get_clock().now()
        if msg.steer_mode > 1:
            if self.steer_mode != msg.steer_mode:
                self.get_logger().warn(
                    "仿真仅支持 steer_mode 0/1，收到 %d，车辆停车"
                    % msg.steer_mode)
            self.steer_mode = msg.steer_mode
            self.v_cmd = 0.0
            self.delta_cmd = 0.0
            return
        if self.steer_mode != msg.steer_mode:
            self.steer_mode = msg.steer_mode
            self.get_logger().info(
                "转向模式切换为 %d（等效轴距 %.3fm）"
                % (self.steer_mode, self.effective_wheelbase()))
        self.v_cmd = max(0.0, min(msg.speed_m_s, self.max_speed))
        raw = msg.steer_angle_rad
        clamped = max(-self.max_plant_steer, min(self.max_plant_steer, raw))
        if abs(clamped - raw) > 1e-9:
            # 控制器输出了超出物理范围的转角，记下来便于发现调参问题
            self.steer_clamped_count += 1
            if self.steer_clamped_count % 50 == 1:
                self.get_logger().warn(
                    "转向指令 %.1f° 超出仿真车辆上限 %.1f°，已夹取（累计 %d 次）。"
                    "若频繁出现说明 lqr_Q/R 或 wheelbase 配得不合理。"
                    % (math.degrees(raw), math.degrees(self.max_plant_steer),
                       self.steer_clamped_count))
        self.delta_cmd = clamped

    def effective_wheelbase(self):
        """返回当前模式对应的等效轴距。"""
        return self.wheelbase * (0.5 if self.steer_mode == 1 else 1.0)

    def on_unmanned(self, msg):
        if msg.data != self.unmanned:
            self.unmanned = msg.data
            mode_text = (
                "UNMANNED（允许 path_tracker 驱动）"
                if self.unmanned else "MANUAL（规划器自动规划）")
            self.get_logger().info(
                "工作模式切换 -> %s" % mode_text)

    def on_path(self, msg):
        """收到规划路径后，把车对齐到起点附近（只做一次）。

        对齐后留 align_offset 的横向偏差，这样能直观看到 path_tracker 把车
        收敛回路径的过程。若 align_to_path_start=false 则跳过，用绝对初始位姿。
        """
        if self.aligned or not self.align_to_path or len(msg.x) < 2 or len(msg.x) != len(msg.y):
            return
        self.aligned = True

        start_yaw = math.atan2(msg.y[1] - msg.y[0], msg.x[1] - msg.x[0])

        # 沿路径起点的法向（左侧为正）偏移 align_offset 米作为初始横向误差
        nx = -math.sin(start_yaw)
        ny = math.cos(start_yaw)
        self.x = msg.x[0] + nx * self.align_offset
        self.y = msg.y[0] + ny * self.align_offset
        self.yaw = start_yaw

        self.get_logger().info(
            "已对齐到路径起点 (%.2f, %.2f)，航向 %.1f°，故意留横向偏差 %.2fm 以观察收敛。"
            "路径共 %d 点。"
            % (self.x, self.y, math.degrees(self.yaw),
               self.align_offset, len(msg.x)))

    # ============================================================
    # 主循环
    # ============================================================
    def tick(self):
        now = self.get_clock().now()

        # 指令超时保护：与 chassis_node 的看门狗同理，没有新鲜指令就停车回正。
        # 否则 path_tracker 一旦被门控停掉，仿真车还会按最后一条指令一直跑。
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout_s:
            self.v_cmd = 0.0
            self.delta_cmd = 0.0

        # 输入为前轴中心等效角；根据当前模式选择运动学增益。
        self.v = self.v_cmd
        if self.v > 1e-6:
            self.x += self.v * math.cos(self.yaw) * self.dt
            self.y += self.v * math.sin(self.yaw) * self.dt
            # tan(delta) 在 delta 接近 ±90° 时发散，已由 max_plant_steer 夹住
            self.yaw = wrap_angle(
                self.yaw
                + (self.v / self.effective_wheelbase())
                * math.tan(self.delta_cmd)
                * self.dt)

        self.publish_pose()
        self.publish_tf()
        self.publish_marker()

    def publish_pose(self):
        """模拟 gnss_driver 的 ~/pose：ENU 位姿、三角姿态、卫星数和速度。"""
        m = Pose()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.x = self.x
        m.y = self.y
        m.z = 0.0
        m.roll = 0.0
        m.pitch = 0.0
        m.yaw = self.yaw
        m.satellites = 30
        m.valid = True
        m.speed_m_s = self.v
        self.pose_pub.publish(m)

        pose_stamped = PoseStamped()
        pose_stamped.header = m.header
        pose_stamped.pose.position.x = m.x
        pose_stamped.pose.position.y = m.y
        pose_stamped.pose.position.z = m.z
        half = m.yaw * 0.5
        pose_stamped.pose.orientation.z = math.sin(half)
        pose_stamped.pose.orientation.w = math.cos(half)
        self.pose_stamped_pub.publish(pose_stamped)

    def publish_tf(self):
        """odom -> base_link，与 gnss_driver 广播的同一条 TF。"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        half = self.yaw * 0.5
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)

        msg = TFMessage()
        msg.transforms.append(t)
        self.tf_pub.publish(msg)

    def publish_marker(self):
        """RViz 里显示车体朝向的箭头，便于直观看跟踪效果。"""
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "sim_vehicle"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.position.z = 0.2
        half = self.yaw * 0.5
        m.pose.orientation.z = math.sin(half)
        m.pose.orientation.w = math.cos(half)
        # 箭头长 = 车长量级，宽/高随意，够看清朝向即可
        m.scale.x = 1.5
        m.scale.y = 0.25
        m.scale.z = 0.25
        m.color.r = 0.2
        m.color.g = 0.8
        m.color.b = 0.2
        m.color.a = 1.0
        self.marker_pub.publish(m)

    def publish_slow_topics(self):
        """1Hz 重发 latched 话题。

        origin：planner 的 ENU 投影原点，未到位时规划器不会规划。
        work_mode：path_tracker 要求 UNMANNED 才驱动，靠 /sim/unmanned 切换。
                   必须持续发：path_tracker 的 work_mode_timeout_s=3.0，
                   停发会被判超时。
        """
        now = self.get_clock().now().to_msg()

        o = NavSatFix()
        o.header.stamp = now
        o.header.frame_id = "gnss_link"
        o.latitude = self.get_parameter("origin_lat").value
        o.longitude = self.get_parameter("origin_lon").value
        o.altitude = self.get_parameter("origin_alt").value
        # 协方差置 0 + UNKNOWN 表示"这是原点广播，不是定位解"
        o.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.origin_pub.publish(o)

        w = WorkMode()
        w.header.stamp = now
        if self.unmanned:
            w.mode = WorkMode.MODE_UNMANNED
            w.mode_name = "unmanned"
        else:
            w.mode = WorkMode.MODE_MANUAL
            w.mode_name = "manual"
        self.work_mode_pub.publish(w)


def main():
    rclpy.init()
    node = SimVehicleNode()
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


if __name__ == "__main__":
    main()
