# stanley_control

激光除草机器人项目的 Stanley 横向控制 ROS2 薄壳节点包。

分层（与 weeder_base 的"协议层与 ROS 节点解耦"思想一致）：

- `stanley_control/stanley_controller.py` — 纯算法层，零 ROS 依赖；
  单独 `python3 stanley_controller.py` 可跑内置直线/正弦仿真自测。
- `stanley_control/stanley_node.py` — 薄壳节点，只做订阅/换算/发布，不含控制数学。
- `stanley_control/sim_demo.py` — 仿真数据源节点（正弦路径 + 自行车模型车体），
  无实车/无规划节点时给 stanley_node 喂数据，看收敛效果用；实车联调不起它即可。

## 话题约定

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/path` | nav_msgs/Path | 全局路径（规划节点或 sim_demo 发布） |
| 订阅 | `/odom` | nav_msgs/Odometry | 里程计：后轴位姿 + 车体线速度 |
| 发布 | `/cmd_steer` | std_msgs/Float64 | 前轮转角指令，弧度，左转(逆时针)为正 |
| 发布 | `/stanley/nearest_point` | visualization_msgs/Marker | RViz 可视化：路径最近跟踪点（橙色球） |

## 参数

`wheelbase`（轴距 m，默认 1.0，按实车改）、`k`、`k_soft`、`control_hz`（默认 50）、
`viz`（默认 true，发布 RViz 标记）、`frame_id`（默认 map）。

## 构建与运行

把本目录拷贝到 ROS2 工作空间的 `src/` 下：

    colcon build --packages-select stanley_control
    source install/setup.bash

三个终端分别跑（demo 演示）：

    # 终端1：控制节点（会打印 e/转角/进度日志；数据未就绪时每 3 秒提示等待）
    ros2 run stanley_control stanley_node
    # 终端2：仿真数据源（发正弦 /path + 仿真 /odom，订阅 /cmd_steer 闭环）
    ros2 run stanley_control sim_demo
    # 终端3：RViz2 可视化
    rviz2

RViz2 配置：Fixed Frame 填 `map`，Add -> By topic 添加 `/path`（Path 显示）
和 `/stanley/nearest_point`（Marker 显示）。终端1 日志会显示横向偏差收敛过程；
橙色球是 Stanley 跟踪的路径最近点，沿路径向车移动。

纯命令行验证（不开 RViz）：

    ros2 topic echo /cmd_steer            # 看转角指令随收敛变小
    ros2 topic hz /cmd_steer              # 应约等于 control_hz

## 无 ROS 环境自测

    python3 test_node_stub.py   # 桩测试：回调链/可视化/日志节流/符号约定

## 下游 CAN 桥接注意（weeder_base 协议第二版）

- `ADCU_toVCU_01` 目标转向角：单位 0.1 度、左正右负；本节点输出弧度、左正，
  **符号同向无需取反**，只做 弧度 → 0.1 度 换算。
- 指令帧 20ms 周期并带 alive 生命计数，心跳由 CAN 桥节点负责补发。
- Stanley 只输出单一前轮角；四轮转向底盘需先与 VCU 确认转向模式
  （前轮/阿克曼类）后再下发，见 State_01 的 CtrlMode 字段。
