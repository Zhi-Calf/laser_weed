# -*- coding: utf-8 -*-
"""Stanley 横向控制器（激光除草机器人项目 · 简化版）

控制点取前轴中心，控制律：
    delta = wrap(theta_path - theta) + atan(k * e / (v + k_soft))
其中 e 为前轴相对路径的横向偏差（车在路径右侧为正），
theta_path 为最近路径点航向，theta 为车体航向，v 为车速。
输出转角 delta 以逆时针（左转）为正，与整车协议"左正右负"同向。

零 ROS 依赖：可单独 import 做仿真/单元测试，也可被 stanley_node.py 复用。
对接方式（项目里只需这几行）：
    ctl = StanleyController()
    steer, e, idx = ctl.compute(path, x, y, yaw, v, wheelbase)
path 为 [(x, y, yaw), ...] 的全局路径点列表（yaw 由路径切线方向给出）。
"""

import math


def wrap_angle(a):
    """把角度归一化到 [-pi, pi)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class StanleyController:
    def __init__(self, k=1.2, k_soft=0.1, max_steer=math.radians(35.0)):
        self.k = k            # 横向偏差增益
        self.k_soft = k_soft  # 低速软化项，防止 v->0 时增益爆炸
        self.max_steer = max_steer

    @staticmethod
    def nearest_index(path, x, y):
        """全路径最近点搜索；路径长时可在项目中改为滑窗搜索。"""
        best, bi = float("inf"), 0
        for i, p in enumerate(path):
            d = (p[0] - x) ** 2 + (p[1] - y) ** 2
            if d < best:
                best, bi = d, i
        return bi

    def compute(self, path, x, y, yaw, v, wheelbase):
        """输入后轴位姿与车速，返回 (前轮转角指令, 横向偏差, 最近点索引)。"""
        fx = x + wheelbase * math.cos(yaw)   # 前轴中心
        fy = y + wheelbase * math.sin(yaw)
        i = self.nearest_index(path, fx, fy)
        px, py, pyaw = path[i]
        # 横向偏差：车偏在路径右侧为正
        e = math.sin(pyaw) * (fx - px) - math.cos(pyaw) * (fy - py)
        delta = wrap_angle(pyaw - yaw) + math.atan2(self.k * e, v + self.k_soft)
        delta = max(-self.max_steer, min(self.max_steer, delta))
        return delta, e, i


class BicycleRobot:
    """运动学自行车模型（后轴参考），仅用于仿真自测。"""

    def __init__(self, x, y, yaw, v, wheelbase, dt):
        self.x, self.y, self.yaw = x, y, yaw
        self.v, self.L, self.dt = v, wheelbase, dt

    def step(self, steer):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = wrap_angle(self.yaw + self.v / self.L * math.tan(steer) * self.dt)


def make_path(kind, step=0.5):
    """生成带航向的路径点列表：kind 取 'straight' 或 'sine'。"""
    pts = []
    n = int(60.0 / step)
    for i in range(n + 1):
        x = i * step
        y = 0.0 if kind == "straight" else 2.0 * math.sin(x / 8.0)
        pts.append([x, y, 0.0])
    for i in range(len(pts) - 1):  # 由切线方向补航向
        pts[i][2] = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
    pts[-1][2] = pts[-2][2]
    return pts


def run_case(kind, y0, label):
    path = make_path(kind)
    robot = BicycleRobot(0.0, y0, 0.0, v=4.0, wheelbase=1.0, dt=0.05)
    ctl = StanleyController()
    traj, errs = [], []
    for step in range(int(60.0 / 4.0 / 0.05)):  # 跑完 60 m 路径
        steer, e, _ = ctl.compute(path, robot.x, robot.y, robot.yaw, robot.v, robot.L)
        traj.append((robot.x, robot.y))
        errs.append(e)
        robot.step(steer)
    warm = errs[len(errs) // 5:]  # 去掉前 20% 的收敛段
    print("[%s] 初始偏差 %.2f m | 收敛后最大 |e| = %.3f m | 终点 |e| = %.3f m"
          % (label, y0, max(abs(v) for v in warm), abs(errs[-1])))
    return path, traj, errs


if __name__ == "__main__":
    r1 = run_case("straight", -2.0, "直线·初始偏右2m")
    r2 = run_case("sine", 1.5, "正弦·初始偏左1.5m")

    assert abs(r1[2][-1]) < 0.05, "直线路径未收敛"
    assert max(abs(v) for v in r2[2][len(r2[2]) // 5:]) < 0.3, "正弦路径跟踪偏差过大"
    print("自测通过：直线收敛、正弦跟踪均满足精度要求。")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        for path, traj, _ in (r1, r2):
            ax.plot([p[0] for p in path], [p[1] for p in path], "--", c="gray", lw=1)
            ax.plot([p[0] for p in traj], [p[1] for p in traj], lw=1.5)
        ax.set_title("Stanley control demo")
        ax.axis("equal")
        fig.savefig("stanley_demo.png", dpi=120)
        print("轨迹图已保存: stanley_demo.png")
    except ImportError:
        print("未安装 matplotlib，跳过绘图。")
