#!/usr/bin/env python3
"""
启动 global_path_planner 规划节点 (+ 可选的 RViz 视图).

只起规划器这一个节点 —— 不再编排 GNSS 驱动、底盘与 KPI 评估, 那些在整链入口里:
    ros2 launch tractor_bringup bringup.launch.py
(曾经由本 launch 拉起底盘, 规划器包因此要 exec_depend tractor_bringup、依赖图上有一环, 现在拆
干净了。) 文件名从 start.launch.py 改成 planner.launch.py 也是这个意思: 含义从"整套系统"变成
"只起规划器", 老命令会直接报"文件不存在", 而不是静默地起一个底下没有车的规划器。

节点发布 (节点名 global_path_planner —— 与参数文件顶层键一致):
    /global_path_planner/path             整条弓形路径 (frame odom, 10 Hz, latched; 节点内写作 ~/path)
    ~/path_points, ~/key_points           RViz Marker (frame odom)
    ~/boundary_points, ~/inner_boundary_points
    /global_path_planner/pto_path         PTO 行内粗采样路径 (~/pto_path; in_row_resolution > 0 时才有)
                                          两条路径都是二维: x/y 是 ENU, z 逐点放段类型 (0=作业/1=掉头)

服务 (切换/加载路径来源的**唯一入口** —— 本节点**没有** path_source 参数):
    /global_path_planner/set_source  类型 weeder_msgs/srv/SetPathSource
                                          请求有 path_source (0=local / 1=file), 以及**两种来源各自的
                                          输入** (都在这个请求里, 节点没有对应参数):
                                            * path_file —— file 要加载的 JSON 文件**绝对路径**
                                              (file 时必填; 空/相对路径 -> accepted=false);
                                            * poly_count + poly —— local 的地块顶点 (`poly` 是扁平
                                              数组, 每 3 个数一个顶点: lat, lon, alt; local 时必填;
                                              数量 3..1024 / 长度自洽 / 经纬度不越界, 见服务定义);
                                            * ridge_width + field_margin —— local 的垄宽 (m, 有限且
                                              > 0) 与田边余量 (m, 有限且 >= 0); local 时必填。
                                              ⚠ 这两个**原来是同名节点参数**, 现已删除: 换地块/换垄宽
                                              只能带上新值再调本服务;
                                            * in_row_resolution —— PTO 行内(作业段)采样步长 (m),
                                              **两种来源都必填** (file 的掉头段也按它采): 0.0 = 不产出
                                              pto_path, 否则须是 [0.01, 100] 内的有限数 (原同名参数已删除)
                                          只做**受理**: 校验 (含上面这些字段的取值域) + 登记后立即返回
                                          (accepted = "已受理"), 真正的切换/换顶点/换几何参数/读文件/发布
                                          在下一个定时周期 (100 ms) 内完成
                                          (⚠ 定时器**第一件事**判 ENU 原点: 原点未就绪期间连这份登记
                                          都不消费 —— 受理 != 生效, 等原点就绪那一拍才执行)

路径来源 (启动时**未设置**: 收到上面的服务之前不规划、不读文件、**每拍发一条空 /path**
          —— 把下游可能残留的旧路径覆盖掉, **不报故障码、心跳保持 OK** (来源未设置不算故障);
          ⚠ 与 ENU 原点未就绪的区别**只在报不报码**: 那条路**也每拍发空 /path**, 但要报 10410):
    0   本机规划: 用**这次请求带来的**多边形顶点 (poly_count + poly) + 垄宽 / 田边余量
        (ridge_width + field_margin) 生成弓形路径;
        换地块 = 带上新顶点再调一次本服务 (不必重启, 也不必改 yaml —— 顶点**不在**配置里);
    1   读 JSON 全覆盖路径文件 (**请求里的 path_file** = 该文件的绝对路径; 本节点没有该参数):
        不做本机规划、不需要多边形, 用 gnss_driver 的 ENU 原点把经纬度投到平面后转发到
        /global_path_planner/path;
        文件是**骨架**(粗点 + turn_flag), 节点按 path_resolution 采 /global_path_planner/path、
        按 in_row_resolution 采 /global_path_planner/pto_path (与 local 模式同一套采样实现, 掉头段也照采);
        文件格式与字段语义见配置 config/global_path_planner.yaml 与 README (path[] 每点
        lon/lat/turn_flag: turn_flag -> pose.position.z 段类型; 发布的路径不带航向 ——
        orientation 一律单位四元数, 消费端按几何自己求切线);
        crc32 给了就必须匹配; 旧版逐行 "lat lon [alt]" 的 txt 已不支持;
    (file 曾经是 2 —— 现在 1; 旧值 2 会被服务拒绝, reason 里列出合法取值)
两种来源的 /path 都是 latched, 下游 (local_path_planner -> path_controller) 不用改。
运行期**一个参数都不能改**: 所有参数只在启动时读一次 (要改就改 yaml 后重启, 或起节点时
-p <键>:=<值>), `ros2 param set` 会被**接受但不生效** (参数服务器记新值、节点仍按启动值运行;
⚠ 参数回调已删除, 这种 set **不再有** WARN/reason 提示, 生效值只能看启动日志)。
曾经唯一可改的参数 in_row_resolution 已随几何参数 (ridge_width / field_margin) 改成**由 set_source
请求携带** —— 改它就再调一次服务 (只改这个值时复用上次的结构路径重采, 不重跑偏置/扫描线):
    ros2 service call /global_path_planner/set_source weeder_msgs/srv/SetPathSource \
      "{path_source: 0, ridge_width: 2.5, field_margin: 0.5, in_row_resolution: 2.0, \
        poly_count: 4, poly: [...]}"
受理后**下一个定时周期** (<=100 ms) 内重采样并发布 (⚠ 同样要等 ENU 原点就绪: 原点未就绪期间只有
"已受理 + 登记待消费", 生效与重采样都在原点就绪那一拍补做)。

「工作模式门」的覆盖范围**只有一处**: set_source —— 切/重载路径来源、换几何参数与 PTO 行内分辨率
都走它, 所以都只在**手动模式**下受理 (见下面原因 1)。ENU 原点 (/gnss_driver/origin) **不在这道门内**:
路径与 odom 必须同基准, gnss_driver 重设 datum 时本节点要跟着换基准重投, 所以收到即用 (不看模式)。

⚠️ 单独跑这个 launch 时 RViz 可能什么都不显示 (不是规划器坏了), 原因有三个:

  1. **还没设置路径来源** —— 本节点没有 path_source 参数: 启动后要等 set_source 服务
     (收到之前不规划、不读文件、**每拍发一条空 /path** 覆盖下游残留, **不报故障码** —— 来源未设置
      不算故障, 心跳保持 OK; ⚠ 这与 ENU 原点未就绪那条路的区别**只在报不报码**: 那条路也每拍发空
      /path, 但要报 10410 ENU 原点超时, 见原因 2);
     单跑调试先设来源 (**local 要带上地块顶点与几何参数** —— 它们只能从这里给, 配置里没有):
         ros2 service call /global_path_planner/set_source \
           weeder_msgs/srv/SetPathSource \
           "{path_source: 0, ridge_width: 2.5, field_margin: 0.5, in_row_resolution: 1.0, \
             poly_count: 4, poly: [34.70316078, 112.39118753, 102.0, \
                                   34.70316002, 112.39133512, 202.0, \
                                   34.70320482, 112.39131299, 102.0, \
                                   34.7032093, 112.39117310, 101.0]}"
     ⚠ 该服务**只在手动模式**下受理 (工作模式门 = 信号可信 + MODE_MANUAL): /work_mode 报 MODE_UNMANNED /
     MODE_CALIBRATION 会返回 accepted=false (reason 写明原因); **压根没收到** /work_mode 也**不受理** ——
     不管 work_mode_startup_grace_s (默认 20 s) 还在不在: 那道宽限只决定"开机这段报不报 10411", 不决定
     "放不放行" (放行会回 accepted=true 却出不来路径, 因为检查 ② 对非 kReady 一律撤路径并 return)。
     单跑规划器没有模式发布方时, 只有 -p require_manual_for_change:=false 才能让服务受理 (关掉整道门);
     宽限参数 (或 work_mode_startup_grace_s:=0) 改不了受理结果, 只改"多快报 10411"。
  2. 没有 ENU 原点 —— 规划器要等 gnss_driver 的 /gnss_driver/origin (开机后首次 RTK 固定解后按
     1 Hz 重发)。**这是定时器里检查的第一件事**: 原点没到之前那一拍**不做别的任何事** (不消费 set_source
     登记、不规划、不读文件 —— 顶点投影只跟着原点变化走, 在原点回调里就做完了), 但**每拍发一条空 /path** 把下游打回"无有效路径" (下游据此停车 —— 下游
     缓存的那条可能是本规划器**上一个进程**发的, 一声不吭会让它继续按旧路径走): 开机 origin_startup_grace_s
     (默认 60 s) 内**不算故障** (心跳 OK, 无故障码); 超过宽限 (从未收到) 或收到过之后 origin_timeout_s
     内没有新原点 (超时) 一律报 **FAULT + 10410** (是哪种看 detail), 并清空 /path。
     单跑调试要它直接开跑, 就用**启动参数覆盖**打开 debug_use_poly0_origin: 它把**本次 set_source
     (local) 请求的第一个顶点**当 ENU 原点 (那个顶点与真实 GNSS 基准同源, 所以调试出来的路径就是现场
     会走的路径)。⚠ 注入发生在**受理带顶点的 local 请求**时 (启动时手上没有顶点 —— 顶点现在只能由
     请求带来), 所以单跑调试是**两步**:
         ros2 run global_path_planner global_path_planner_node --ros-args \
             --params-file <配置> -p debug_use_poly0_origin:=true -p require_manual_for_change:=false
         ros2 service call /global_path_planner/set_source weeder_msgs/srv/SetPathSource \
             "{path_source: 0, ridge_width: 2.5, field_margin: 0.5, in_row_resolution: 1.0, \
               poly_count: 4, poly: [34.70316078, 112.39118753, 102.0, ...]}"
     (随包配置里它是 false; 运行期 `ros2 param set` 改不动它 —— 只读参数, 会被接受但不生效。) 本 launch
     **不代开这个开关**, 它只把配置交给节点。⚠ 开着它时上面两条超时判定**整体失效**
     (那个假原点只在受理 local 请求时注入一次, 没有任何东西按 1 Hz 刷新它); 不豁免的话开机
     origin_timeout_s 之后就被判超时、撤掉 /path, 调试时反而什么都看不到;
  3. 没有 TF —— RViz 的 Fixed Frame 是 odom (见 rviz/rviz.rviz), 而本文件**不发布任何静态 TF**
     (不补 map->odom)。要图就与 gnss_driver 一起跑:
         ros2 launch tractor_bringup bringup.launch.py
     或者自己补一个静态变换 (它只让坐标系存在, 不代表任何车辆位姿):
         ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

Usage:
  ros2 launch global_path_planner planner.launch.py
  ros2 launch global_path_planner planner.launch.py rviz:=false
  ros2 launch global_path_planner planner.launch.py planner_config_file:=/path/to.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """解析配置路径, 起规划器 (+ 可选的 RViz)."""
    share = get_package_share_directory("global_path_planner")
    config_path = LaunchConfiguration("planner_config_file").perform(context)
    rviz_config = os.path.join(share, "rviz", "rviz.rviz")

    actions = [LogInfo(msg=["[planner] 配置文件: ", config_path])]

    # launch 只把配置交给节点, **不覆盖任何调试开关**: 要不要用"local 请求的第一个顶点"当 ENU 原点
    # 看配置里的 debug_use_poly0_origin (见文件头), launch 这里不再代开。
    # 也没有 planner_config_path 那一套了 —— 本节点不把参数写回任何 yaml (运行期没有可改的参数,
    # 见文件头), 所以不需要 bringup 再传"写回哪份文件"。
    planner_params = [config_path]

    actions.append(Node(
        package="global_path_planner",
        executable="global_path_planner_node",
        # 节点名必须是 global_path_planner: 参数文件 config/global_path_planner.yaml 的顶层键、
        # 与 README 里的 `ros2 service call /global_path_planner/set_source ...` 都得跟它一致。
        # 名字对不上时参数文件整体不生效 (节点退回代码默认值: 手上没有地块顶点、垄宽 0 -> 根本排不出路径;
        # 要等 set_source 请求)。
        name="global_path_planner",
        output="screen",
        parameters=planner_params,
    ))

    # 注意: 本文件不发布任何静态 TF。RViz 的 Fixed Frame (odom) 要靠 gnss_driver (真机) 或自己
    # 手动补的静态变换提供 —— 单独跑时 RViz 里可能什么都没有, 见文件头的说明。
    actions.append(Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("global_path_planner")
    default_config = os.path.join(share, "config", "global_path_planner.yaml")

    # 名字不叫 config_file: 整链组合时多个 launch 曾用同一个 arg 名, 而 DeclareLaunchArgument 只在
    # "还没被设置过" 时生效 —— 先 include 的那份赢, 其余静默继承它的路径。
    args = [
        DeclareLaunchArgument(
            "planner_config_file", default_value=default_config,
            description="规划器配置 (global_path_planner.yaml)。⚠ 地块顶点、垄宽/田边余量、PTO 行内"
                        "分辨率**都不在这个文件里**: 它们由 /global_path_planner/set_source 请求的 "
                        "poly_count + poly / ridge_width + field_margin / in_row_resolution 字段携带 "
                        "—— 换地块/换垄宽就是带上新值再调一次那个服务 (见 README 的「全局路径来源」一节)。"),

        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="是否起 RViz (用本包自带的 rviz/rviz.rviz)。车上无头运行 (SSH) 时设 false。"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
