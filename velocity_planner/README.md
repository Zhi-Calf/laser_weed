# velocity_planner

基于速腾 E1R 点云的前向矩形速度规划与避停状态节点。首版只输出停车/放行判定，
暂不计算连续目标速度；节点不发布
`/chassis/cmd`；最终停车由 `path_tracker` 的避停门控执行。

## 数据链路

```text
/rslidar_points
    │ sensor_msgs/msg/PointCloud2
    ▼
velocity_planner_node
    │ ROI：点云自身坐标系下前向矩形
    ▼
/velocity_planner/status
    │ weeder_msgs/msg/ObstacleStatus
    ▼
path_tracker ──/chassis/cmd──► weeder_base
```

节点固定以 10 Hz 发布状态。`sensor_valid=false` 表示启动后尚未收到点云、点云超过
0.5 秒未更新，或最新点云为空/字段无效/没有有限坐标，此时 `stop_required=true`。
ROI 内有效点达到阈值时同样输出停车请求。

## ROI 与参数

默认参数位于 `config/velocity_planner.yaml`：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `input_cloud_topic` | `/rslidar_points` | 驱动输出的点云话题 |
| `roi_forward_min_m` | `0.0` | 雷达坐标系下 ROI 前向起点 |
| `roi_forward_max_m` | `2.0` | 雷达坐标系下 ROI 前向终点 |
| `roi_half_width_m` | `0.9` | 雷达坐标系下 ROI 左右半宽 |
| `roi_min_height_m` | `0.1` | 雷达坐标系下最低点高度 |
| `roi_max_height_m` | `1.5` | 雷达坐标系下最高点高度 |
| `min_obstacle_points` | `3` | 触发停车的最少点数 |
| `stop_hold_s` | `0.3` | 障碍消失后的最短停车时间 |

默认 ROI 只是安全起点。实车必须根据车辆包络、雷达安装位置和实际刹停距离调整。

## 日志

节点使用 spdlog 同时输出控制台和滚动文件，日志文件为
`<log_dir>/velocity_planner.log`。`log_dir` 支持 `~` 与 `$VAR` 展开；
目录不可写时会回退到 `$ROS_HOME/logs` 或 `~/.ros/logs`。滚动文件大小、
保留数量、控制台输出和日志级别可分别通过 `log_file_size_mb`、
`log_file_count`、`log_enable_console` 和 `log_level` 配置。

## 坐标要求

节点直接使用点云消息中的 `header.frame_id`，不查询 TF。E1R 驱动默认输出
`rslidar` 坐标系，因此 ROI 参数直接按雷达安装后的视角配置。状态消息的
`header.frame_id` 也继承最近一帧点云的坐标系。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to velocity_planner
source install/setup.bash
ros2 launch velocity_planner velocity_planner.launch.py
```

查看状态：

```bash
ros2 topic echo /velocity_planner/status
```

## 测试

```bash
colcon test --packages-select velocity_planner
colcon test-result --verbose
```
