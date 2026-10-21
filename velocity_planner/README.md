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
    │ TF：rslidar -> base_link
    │ ROI：base_link 下前向矩形
    ▼
/velocity_planner/status
    │ weeder_msgs/msg/ObstacleStatus
    ▼
path_tracker ──/chassis/cmd──► weeder_base
```

节点以 10Hz 发布状态。`sensor_valid=false` 表示点云缺失、超时、字段无效或
TF 转换失败，此时 `stop_required=true`。ROI 内有效点达到阈值时同样输出停车请求。

## ROI 与参数

默认参数位于 `config/velocity_planner.yaml`：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `input_cloud_topic` | `/rslidar_points` | 驱动输出的点云话题 |
| `target_frame` | `base_link` | ROI 所在坐标系 |
| `roi_forward_min_m` | `0.0` | ROI 前向起点 |
| `roi_forward_max_m` | `2.0` | ROI 前向终点 |
| `roi_half_width_m` | `0.9` | ROI 左右半宽 |
| `roi_min_height_m` | `0.1` | 最低点高度，用于过滤地面 |
| `roi_max_height_m` | `1.5` | 最高点高度 |
| `min_obstacle_points` | `3` | 触发停车的最少点数 |
| `cloud_timeout_s` | `0.5` | 点云结果有效期 |
| `stop_hold_s` | `0.3` | 障碍消失后的最短停车时间 |

默认 ROI 只是安全起点。实车必须根据车辆包络、雷达安装位置和实际刹停距离调整。

## 坐标要求

点云默认位于 `rslidar` 坐标系，节点要求 TF 树中存在
`base_link -> rslidar`。例如：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 1.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id rslidar
```

示例数值不是实车标定值，必须替换。

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
