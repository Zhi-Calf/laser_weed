#!/usr/bin/env python3
"""GNSS 终端仪表盘 —— 单窗格实时显示定位/航向/速度状态。

订阅（全部 SensorDataQoS，与 gnss_driver 一致）：
  /gnss_driver/fix        sensor_msgs/NavSatFix
  /gnss_driver/imu        sensor_msgs/Imu                        (ENU yaw)
  /gnss_driver/vel        geometry_msgs/TwistWithCovarianceStamped
  /gnss_driver/status     weeder_msgs/GnssStatus
  /odometry/global nav_msgs/Odometry

用法:
  ros2 run gnss_localization gnss_dashboard.py
  ros2 run gnss_localization gnss_dashboard.py --ros-args -p odom_topic:=/odometry/local
"""

import math
import os
import re
import sys
import time
import unicodedata
from collections import deque

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix

try:
    from weeder_msgs.msg import GnssStatus
except ImportError:  # 未 source weeder_msgs 时降级运行
    GnssStatus = None

# ---------------------------------------------------------------- ANSI
CSI = "\033["
HOME = CSI + "H"
CLR_EOL = CSI + "K"
CLR_EOS = CSI + "J"
HIDE_CUR = CSI + "?25l"
SHOW_CUR = CSI + "?25h"

BOLD = CSI + "1m"
DIM = CSI + "2m"
RESET = CSI + "0m"
RED = CSI + "31m"
GREEN = CSI + "32m"
YELLOW = CSI + "33m"
BLUE = CSI + "34m"
CYAN = CSI + "36m"
GREY = CSI + "90m"

WIDTH_MAX = 62
WIDTH_MIN = 44


def term_width():
    """跟随 pane 实际宽度，避免分屏时横条折行。"""
    try:
        w = os.get_terminal_size().columns
    except OSError:
        w = WIDTH_MAX
    return max(WIDTH_MIN, min(WIDTH_MAX, w - 1))

FIX_NAMES = {
    0: ("NO FIX", RED),
    1: ("SPP (单点)", YELLOW),
    2: ("DGPS (差分)", YELLOW),
    4: ("RTK FIXED (固定解)", GREEN),
    5: ("RTK FLOAT (浮点解)", YELLOW),
    6: ("INS (组合导航)", CYAN),
}

# NavSatStatus.status -> 说明（/gnss_driver/fix 的粗粒度状态）
NAVSAT_NAMES = {-1: "NO_FIX", 0: "FIX", 1: "SBAS", 2: "GBAS"}

STALE_SEC = 2.0  # 超过该秒数未收到消息即标记为陈旧


def yaw_from_quat(q):
    """四元数 -> ENU yaw (rad)，无需 tf2 依赖。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def rp_from_quat(q):
    """四元数 -> (roll, pitch) rad。"""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return roll, math.asin(sinp)


def enu_yaw_to_compass(yaw_rad):
    """ENU yaw (东=0, 逆时针) -> 罗盘角 (北=0, 顺时针)。"""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def compass_label(deg):
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return dirs[int((deg + 22.5) % 360.0 // 45.0)]


ANSI_RE = re.compile(r"\033\[[0-9;?]*[a-zA-Z]")


def char_w(ch):
    """中日韩全角字符占两列。"""
    return 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1


def disp_width(s):
    return sum(char_w(c) for c in ANSI_RE.sub("", s))


def fit(s, w):
    """按显示宽度截断，跳过 ANSI 转义，防止窄 pane 下折行。"""
    if disp_width(s) <= w:
        return s
    out, n, i = [], 0, 0
    while i < len(s):
        m = ANSI_RE.match(s, i)
        if m:
            out.append(m.group())
            i = m.end()
            continue
        cw = char_w(s[i])
        if n + cw > w - 1:
            out.append("…")
            break
        out.append(s[i])
        n += cw
        i += 1
    return "".join(out) + RESET


def bar_width(w):
    """罗盘条宽度：取 4 的倍数 +1，保证 N/E/S/W 刻度整除对齐。"""
    return max(17, ((w - 4) // 4) * 4 + 1)


def heading_bar(deg, width):
    """罗盘条：把 0-360 映射到一条带刻度的横条上。"""
    step = width // 4
    pos = int(round(deg / 360.0 * (width - 1)))
    cells = []
    for i in range(width):
        if i == pos:
            cells.append(BOLD + CYAN + "◆" + RESET)
        elif i % step == 0:
            cells.append(GREY + "|" + RESET)
        else:
            cells.append(GREY + "·" + RESET)
    return "".join(cells)


def heading_bar_labels(width):
    """与 heading_bar 刻度对齐的 N/E/S/W 标签行。"""
    step = width // 4
    row = [" "] * width
    for i, ch in enumerate("NESWN"):
        row[min(i * step, width - 1)] = ch
    return "".join(row)


class RateTracker:
    """滑动窗口频率估计 + 陈旧检测。"""

    def __init__(self, window=25):
        self.stamps = deque(maxlen=window)
        self.count = 0

    def tick(self):
        self.stamps.append(time.monotonic())
        self.count += 1

    @property
    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 1e-6 else 0.0

    @property
    def age(self):
        if not self.stamps:
            return None
        return time.monotonic() - self.stamps[-1]

    @property
    def stale(self):
        a = self.age
        return a is None or a > STALE_SEC


class GnssDashboard(Node):
    def __init__(self):
        super().__init__("gnss_dashboard")

        odom_topic = self.declare_parameter("odom_topic", "/odometry/global").value
        self.refresh_hz = float(self.declare_parameter("refresh_hz", 5.0).value)

        # BEST_EFFORT 订阅可同时兼容 best-effort(驱动) 与 reliable(EKF) 发布者
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.fix = None
        self.imu = None
        self.vel = None
        self.status = None
        self.odom = None
        self.odom_topic = odom_topic

        self.rates = {
            k: RateTracker()
            for k in ("fix", "imu", "vel", "status", "odom")
        }
        self.t_start = time.monotonic()
        self.w = term_width()

        self.create_subscription(NavSatFix, "/gnss_driver/fix", self._on_fix, qos)
        self.create_subscription(Imu, "/gnss_driver/imu", self._on_imu, qos)
        self.create_subscription(TwistWithCovarianceStamped, "/gnss_driver/vel", self._on_vel, qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        if GnssStatus is not None:
            self.create_subscription(GnssStatus, "/gnss_driver/status", self._on_status, qos)

        self.create_timer(1.0 / max(self.refresh_hz, 0.5), self._render)

        sys.stdout.write(HIDE_CUR + CSI + "2J")

    # -------------------------------------------------------- callbacks
    def _on_fix(self, msg):
        self.fix = msg
        self.rates["fix"].tick()

    def _on_imu(self, msg):
        self.imu = msg
        self.rates["imu"].tick()

    def _on_vel(self, msg):
        self.vel = msg
        self.rates["vel"].tick()

    def _on_status(self, msg):
        self.status = msg
        self.rates["status"].tick()

    def _on_odom(self, msg):
        self.odom = msg
        self.rates["odom"].tick()

    # ------------------------------------------------------------ view
    def _render(self):
        L = []
        self.w = term_width()
        rule = GREY + "─" * self.w + RESET

        up = time.monotonic() - self.t_start
        L.append(BOLD + CYAN + " GNSS DASHBOARD ".center(self.w, "═") + RESET)
        L.append(f"{GREY}运行 {up:6.1f}s   刷新 {self.refresh_hz:.0f}Hz   Ctrl-C 退出{RESET}")
        L.append("")

        L += self._sec_fix()
        L.append(rule)
        L += self._sec_heading()
        L.append(rule)
        L += self._sec_motion()
        L.append(rule)
        L += self._sec_position()
        L.append(rule)
        L += self._sec_topics()

        out = [HOME]
        for line in L:
            out.append(fit(line, self.w) + CLR_EOL + "\n")
        out.append(CLR_EOS)
        sys.stdout.write("".join(out))
        sys.stdout.flush()

    def _sec_fix(self):
        L = [BOLD + "▍解算状态" + RESET]
        st = self.status
        if st is None:
            hint = "未收到 /gnss_driver/status" if GnssStatus else "weeder_msgs 未 source"
            L.append(f"  {GREY}{hint}{RESET}")
        else:
            name, color = FIX_NAMES.get(st.fix_type, (f"UNKNOWN({st.fix_type})", RED))
            flag = "" if not self.rates["status"].stale else f" {RED}[STALE]{RESET}"
            L.append(f"  定位质量  {color}{BOLD}{name}{RESET}{flag}")

            sats = st.num_satellites
            sc = GREEN if sats >= 20 else (YELLOW if sats >= 10 else RED)
            hc = GREEN if st.hdop <= 1.0 else (YELLOW if st.hdop <= 2.0 else RED)
            L.append(
                f"  卫星数    {sc}{sats:>3d}{RESET}"
                f"     差分龄期 {st.diff_age:5.1f} s"
            )
            L.append(
                f"  HDOP {hc}{st.hdop:5.2f}{RESET}"
                f"   PDOP {st.pdop:5.2f}   VDOP {st.vdop:5.2f}"
            )
        if self.fix is not None:
            cov = self.fix.position_covariance
            sig_h = math.sqrt(max(cov[0], 0.0))
            sig_v = math.sqrt(max(cov[8], 0.0))
            nav = NAVSAT_NAMES.get(self.fix.status.status, str(self.fix.status.status))
            L.append(
                f"  {GREY}NavSatFix {nav:<6s} σ_h {sig_h:5.3f} m  σ_v {sig_v:5.3f} m{RESET}"
            )
        return L

    def _sec_heading(self):
        L = [BOLD + "▍航向 / 姿态" + RESET]
        have = False

        if self.imu is not None:
            yaw = yaw_from_quat(self.imu.orientation)
            roll, pitch = rp_from_quat(self.imu.orientation)
            comp = enu_yaw_to_compass(yaw)
            flag = f" {RED}[STALE]{RESET}" if self.rates["imu"].stale else ""
            L.append(
                f"  ENU yaw   {BOLD}{CYAN}{math.degrees(yaw):8.2f}°{RESET}"
                f"  ({yaw:+.4f} rad){flag}"
            )
            L.append(
                f"  罗盘方位  {BOLD}{comp:8.2f}°{RESET}  {compass_label(comp):<2s}"
                f"   {GREY}(北=0 顺时针){RESET}"
            )
            bw = bar_width(self.w)
            L.append("  " + heading_bar(comp, bw))
            L.append(f"  {GREY}{heading_bar_labels(bw)}{RESET}")
            rate = math.degrees(self.imu.angular_velocity.z)
            if self.w >= 56:
                L.append(
                    f"  roll {math.degrees(roll):7.2f}°   pitch {math.degrees(pitch):7.2f}°"
                    f"   {GREY}yaw率 {rate:+6.1f}°/s{RESET}"
                )
            else:
                L.append(
                    f"  roll {math.degrees(roll):6.2f}° pitch {math.degrees(pitch):6.2f}°"
                    f" {GREY}yaw率 {rate:+5.1f}°/s{RESET}"
                )
            have = True

        if self.status is not None:
            L.append(
                f"  {GREY}原始 heading (NMEA) {self.status.heading_deg:7.2f}°{RESET}"
            )
            have = True

        if self.odom is not None:
            oy = yaw_from_quat(self.odom.pose.pose.orientation)
            L.append(f"  {GREY}odom yaw  {math.degrees(oy):8.2f}°{RESET}")
            have = True

        if not have:
            L.append(f"  {GREY}无航向数据{RESET}")
        return L

    def _sec_motion(self):
        L = [BOLD + "▍速度" + RESET]
        have = False

        if self.vel is not None:
            t = self.vel.twist.twist.linear
            sp = math.hypot(t.x, t.y)
            flag = f" {RED}[STALE]{RESET}" if self.rates["vel"].stale else ""
            L.append(
                f"  合速度    {BOLD}{sp:6.3f} m/s{RESET}"
                f"  ({sp * 3.6:5.2f} km/h){flag}"
            )
            L.append(
                f"  {GREY}vx {t.x:+7.3f}  vy {t.y:+7.3f}  vz {t.z:+7.3f}  m/s{RESET}"
            )
            have = True

        if self.status is not None:
            L.append(f"  {GREY}GNSS 报文速度 {self.status.speed_kmh:6.2f} km/h{RESET}")
            have = True

        if self.odom is not None:
            ot = self.odom.twist.twist
            L.append(
                f"  {GREY}odom  v {ot.linear.x:+6.3f} m/s"
                f"   ω {ot.angular.z:+6.3f} rad/s{RESET}"
            )
            have = True

        if not have:
            L.append(f"  {GREY}无速度数据{RESET}")
        return L

    def _sec_position(self):
        L = [BOLD + "▍位置" + RESET]
        if self.fix is not None:
            flag = f" {RED}[STALE]{RESET}" if self.rates["fix"].stale else ""
            L.append(
                f"  纬度 {BOLD}{self.fix.latitude:13.8f}{RESET}°"
                f"  经度 {BOLD}{self.fix.longitude:14.8f}{RESET}°{flag}"
            )
            L.append(f"  高程 {self.fix.altitude:9.3f} m")
        elif self.status is not None:
            L.append(
                f"  纬度 {self.status.latitude_deg:13.8f}°"
                f"  经度 {self.status.longitude_deg:14.8f}°"
            )
            L.append(f"  高程 {self.status.altitude_m:9.3f} m")
        else:
            L.append(f"  {GREY}无 /gnss_driver/fix{RESET}")

        if self.odom is not None:
            p = self.odom.pose.pose.position
            frame = self.odom.header.frame_id or "?"
            row = (
                f"  {BLUE}map/odom{RESET}  x {BOLD}{p.x:10.3f}{RESET}"
                f"  y {BOLD}{p.y:10.3f}{RESET}"
            )
            if self.w >= 56:
                row += f"  z {p.z:7.3f}  {GREY}[{frame}]{RESET}"
            L.append(row)
        return L

    def _sec_topics(self):
        L = [BOLD + "▍话题频率" + RESET]
        labels = [
            ("fix", "/gnss_driver/fix"),
            ("imu", "/gnss_driver/imu"),
            ("vel", "/gnss_driver/vel"),
            ("status", "/gnss_driver/status"),
            ("odom", self.odom_topic),
        ]
        for key, name in labels:
            r = self.rates[key]
            if r.count == 0:
                L.append(f"  {name:<18s} {RED}—  无数据{RESET}")
                continue
            color = RED if r.stale else GREEN
            row = f"  {name:<18s} {color}{r.hz:6.1f} Hz{RESET}"
            if self.w >= 56:  # 窄 pane 下省略计数/龄期，只留频率
                row += f"  {GREY}n={r.count:<7d} age={r.age:4.1f}s{RESET}"
            L.append(row)
        return L


def main():
    if not sys.stdout.isatty():
        print("警告: 输出非终端，ANSI 刷屏可能显示异常。", file=sys.stderr)
    rclpy.init()
    node = GnssDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / tmux 关窗都算正常退出，不打印堆栈
    finally:
        sys.stdout.write(SHOW_CUR + "\n")
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
