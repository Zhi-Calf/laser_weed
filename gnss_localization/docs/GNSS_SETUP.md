# 双天线 MEMS 惯性-卫星组合导航系统 现场部署指南

双天线 MEMS 惯性-卫星组合导航系统作为 GNSS 定位备选方案。
驱动节点 `gnss_driver` 解析 `$GIAVP` 语句，发布话题
（`/gnss_driver/fix`、`/gnss_driver/odom`、`/gnss_driver/odom_raw`、
`/gnss_driver/imu`、`/gnss_driver/vel`、`/gnss_driver/status`、
`/gnss_driver/origin`）。修正里程计 `/gnss_driver/odom` 已是 ENU
且含杆臂/标定/航向修正，path_controller 直接订阅它并发布 `/odometry/global`。
`/gnss_driver/odom_raw` 是**原始**数据（设备中心位置 + 原始测向航向，无任何修正），
只给 tractor_calibration 标定用：标定要测物理真值，用修正后的位姿会被当前生效的
标定值污染。标定结果（`calib_offset_x/y` = 绝对杆臂、`calib_offset_heading` = 绝对偏角）
由 gnss_driver 直接施加，是杆臂的**唯一**来源（不再有"名义杆臂 device_offset_*"）。

启动方式：

```bash
# 单独启动定位 (杆臂/航向偏角由标定写回, 无需 launch 参数)
ros2 launch gnss_localization localization.launch.py

# 全栈（双天线 GNSS 为唯一驱动，无需切换参数）
ros2 launch tractor_bringup bringup.launch.py
```

**ENU 原点（datum）优先取 `gnss_driver.yaml` 的 `origin_lat/origin_lon/origin_alt`**。
这三个值必须与 `global_path_planner.yaml` 中 `poly` 的第一个顶点完全一致：
规划器用该顶点做 ENU 投影，`gnss_driver` 用同一坐标把 RTK 位置投影成 `/gnss_driver/odom`。
原点通过 `/gnss_driver/origin` 以 `transient_local` 发布，之后按 1 Hz 重发。
若 `origin_*` 未配置或非法，才回退为上电后首次 RTK 固定解。
`gnss:=false` 可跳过整个 GNSS 分支做台架测试。

---

## 1. 硬件安装（左右安装，对应手册图 14B）

- 主机与车体**刚性固连**，主机前进轴 Y1 与车辆前进方向平行。
- **主天线（接 ANT1）装在车体右侧**，从天线（接 ANT2）装在左侧，两天线连线与横轴 X1 平行。
- 两天线间距（基线）**至少 0.5 m**，越长航向精度越高（1 m 基线对应 0.2° 航向精度）；两天线尽量等高。
- 卫星天线与高速相机（如有）至少保持 20 cm 距离且中间有隔离。

## 2. 接线

| 连接 | 说明 |
|------|------|
| ANT1 / ANT2 | 主/从卫星天线（主=右侧天线） |
| 主电源线 | DC 12 V / 1 A 额定（10~32 V） |
| COM0（RS232 DB9） | 2-TX、3-RX、5-GND，**与主机板交叉连接**（设备 TX → 板子 RX）；域控推荐 **115200**（460800 易乱码） |
| RTCM 口 + 5V | 接 4G Module（RTK 差分数据入口） |
| 4G 天线 | 接 4G Module 的天线接口 |

注意：RTK 差分链路完全由 4G Module 走 RTCM 口注入，**ROS 驱动不发送任何 NTRIP 配置**
（不同于 D300 方案的 `set ntrip ...` 命令）。

## 3. 初始配置实测清单（卷尺，精确到 1 cm）

共需实测 3 组值：

### 3.1 杆臂值 lever_armX/Y/Z（写入设备）

以**主机中心为原点**建立载体直角坐标系（手册图 15）：

- Y 轴 = 车前进方向
- X 轴 = 车右方
- Z 轴 = 天向

测量**主天线（右侧）相位中心**在该坐标系下的三维坐标，单位米。
左右安装时主天线在右侧，通常 X 为正值。

### 3.2 基线长度（验证用）

两天线相位中心间距。上电后与 GIAVP 输出的 Baseline 字段对比，
偏差应在几厘米内；偏差大说明天线接反或杆臂/安装有误。

### 3.3 ROS 侧杆臂：由**标定**确定（`calib_offset_x/y`）

**注意语义与 D300 方案不同！** GIAVP 输出的位置是杆臂补偿后的**主机中心**位置
（不是天线位置），因此需要的是"`base_link`（后轴中心/车辆控制点）到**主机中心**"的杆臂：

- `calib_offset_x`：纵向 (X 向前为正)，出厂初值 ≈ 前方 0.5 m
- `calib_offset_y`：横向 (Y 向左为正；主机不在纵向中轴线上时非零)
- 两者是**绝对杆臂**，由 `tractor_calibration` 的原地旋转标定测出后写回
  `config/gnss_driver.yaml` 并立即生效 —— 不需要现场手工填数，也不再有"名义杆臂"这一层。
- 垂向高度不参与杆臂换算（标定只有 x/y/heading 三个量）：`odom` 的 z 即设备中心高程，
  `base_link -> gnss_link` 静态 TF 也不再发布。

## 4. 设备端一次性配置（UartAssist，波特率 460800）

逐条发送并确认收到对应 `$ack` 回复：

```text
$cmd,setparams,dualant,1*ff          # 左右安装角（必须！默认 0=前后安装，不配航向差 90°）
$cmd,setparams,leverarm,X,Y,Z*ff     # 填入 3.1 实测的杆臂值，单位米
$cmd,setparams,isrtk,1*ff            # 启用 RTK 差分
$cmd,output,com0,baudrate,115200*ff  # 域控 UART 推荐 115200（与 ROS baud_rate 一致）
$cmd,output,com0,giavp,10*ff         # COM0 输出 GIAVP 10 Hz（115200 下不要用 50 Hz）
$cmd,saveconf,1*ff                   # 保存配置
```

发送保存命令后**整机断电重新上电**生效。

> ROS 驱动默认不再在运行中反复发 OUTPUT。输出协议/波特率请一次性在设备端配置并 `saveconf`。
> 若在 115200 下强行 `giavp,50`，串口易卡死：先收到几帧 GIAVP，随后一直 `smart_recovery (no/stale GIAVP)`。

## 5. 4G Module 千寻账号配置（如需更换账号）

1. 4G Module 断电，USB 口插电脑，识别为串口（115200）。
2. 用 Xshell 串口连接，4G Module 上电，出现 `imx6ulevk login:` 时输入 `root` 登录。
3. `ps -aux` 查看 `/home/myfile/quectel-CM` 和 `/home/myfile/tcp ...` 进程 PID，
   `kill -s 9 <PID>` 逐个结束（PID 以实际为准）。
4. `cd /home/myfile`，`vi timeupdate.sh`，修改千寻账号 `userid` 和密码 `password`
   （若参数行有 `#` 注释先去掉）。
5. `Esc` → `:wq` 保存，重新上电。

## 6. 上电验证顺序

1. 上电后 work 灯闪烁（工作正常）、定位灯闪烁（定位成功）；首次定位 < 35 s，定向 < 1 min。
2. 串口收到 `$GIAVP` 语句，频率 ≈ 10 Hz。
3. `ros2 topic echo /gnss_driver/status`：
   - `fix_type = 4`（RTK 固定解）后才可跑作业；4G Module 差分正常时通常 1~2 分钟内收敛。
   - 如长时间停在 5（浮点）/1（单点），检查 4G Module 供电、天线、千寻账号。
4. GIAVP 的 Baseline ≈ 实测基线（3.2），偏差 > 10 cm 需排查天线是否接反。
5. 航向验证：车头朝北时 `heading_deg ≈ 0`，朝东 ≈ 90。若差 90° 的整数倍，
   说明 `dualant` 未配置或配置错误（回到第 4 步）。
6. 位置验证：`ros2 topic echo /odometry/global`，前后推车确认 x/y 变化方向与车体运动一致。

## 7. 常用排查

| 现象 | 排查 |
|------|------|
| 串口无数据 | TX/RX 是否交叉；波特率是否与设备一致（推荐 115200）；设备是否已 `saveconf` 为 GIAVP |
| status 一直 0 | 天线馈线连接；到开阔地带；等待首次定位 |
| status 停在 1/2 | 4G Module 差分未通：查 4G 天线、物联网卡流量、千寻账号 |
| status 停在 5 浮点 | 天线遮挡/多路径；基线过短；等待收敛 |
| 航向差 90° | `dualant` 未配置为 1 |
| 航向差 180° | 主从天线接反（主天线应在右侧接 ANT1） |
| odom 不发布 | status 未达 `odom_min_status`（默认 4）；应急可临时放宽为 5 或 1 |
