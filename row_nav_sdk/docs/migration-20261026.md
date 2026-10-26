# 运行 SDK 源码迁入与验证记录

## 范围与来源

来源：`rk` 提交 `12692314930a3297dc336c42d802eb473e56dcf8`。
首次导入 74 个文件，逐文件 SHA256 与来源一致，记录在 `migration-origin.json`。
另增加本目录独立 CMake 构建、colcon 包登记、来源清单和说明文件。
本目录是今后运行算法唯一维护位置；历史导入哈希不约束后续正常维护。

原 SDK C 头文件及 ABI v1、命名空间、算法实现未因迁入重命名/重写。
共享库为 0.2.0 / SOVERSION 1，保留 12 个 C 导出函数。
训练、数据、权重、视频、工具链归档和实验输出没有搬入。
没有把厂商 RKNN 运行库或原项目硬件控制代码复制进来。

## 最小修改边界

整车仓库新增 `row_nav_sdk/` 和 `scripts/tests/test_build_crop_row.py`；
只调整我们自己的 `crop_row_perception` 构建依赖/说明、统一构建脚本及 SDK 来源记录。
对比迁入前 `fb08dfc`，以下路径无差异：

`gnss_localization`、`global_path_planner`、`path_tracker`、`velocity_planner`、
`weeder_base`、`weeder_msgs`、`sim_vehicle`、`rslidar_sdk`、`rslidar_msg`、整车根 README。

不改同事的消息定义、定位/规划/控制、CAN、原启动文件；不自动加入整车启动。
本轮没有重编或重新验收全部同事模块。新增包会被全量 colcon 发现；只构建原模块时
可跳过 `row_nav_sdk crop_row_perception`，不能把新增依赖说成完全没有构建影响。

旧 `rk` 的 C++ 构建改为引用本目录的源清单、头文件、C API 和 SDK 测试，
原视频/只读 CAN 等开发工具仍在旧仓库。旧源码文件作为冻结快照保留，未删除资料。
旧开发构建不再允许被旧打包脚本误标为只来自 rk 的独立 SDK 发布包。

## MyUbuntu 验证环境

独立工作区：`/tmp/row-nav-migration.MFfOyJ`；没有覆盖远端原开发仓库。
宿主为 x86_64 Ubuntu / GCC 11.4 / OpenCV 4.5.4，独立 CMake 构建无需 ROS。
ROS 使用原测试环境的快照镜像 `laser-weed-sdk-check:20261026-mffoyj`，
容器 `row-nav-migration-dev-MFfOyJ`，挂载该独立工作区为 `/ws`。
容器内补装 `libopencv-dev`，不在宿主机安装 ROS 或更改其他服务。
测试完成后容器退出，未启动现场定位/控制节点。

## 已验证结果

| 项目 | 结果与边界 |
|---|---|
| 独立 SDK CMake | 编译、安装成功，不依赖 ROS 或同级 rk 源码 |
| SDK CTest | C 头文件、SDK 契约、动态 SDK 契约、几何单元组共 4 组全部通过 |
| 统一构建入口 | `row_nav_sdk`、现有 `weeder_msgs`、`crop_row_perception` 按依赖编译安装成功 |
| 统一测试 | SDK 与 ROS 共 11 个 CTest 测试组；节点 GTest 14 个用例通过 |
| colcon 汇总 | 25 tests、0 errors、0 failures、0 skipped；包含测试组与内部用例，非 25 次实机推理 |
| 安装消费 | 在另一 SDK 安装前缀上构建纯 C 示例，清除 LD_LIBRARY_PATH 后 `--version` 返回 0.2.0 |
| 共享库依赖 | 12 个 C API 导出；DT_NEEDED 为 OpenCV/C++ 系统运行库，不含 ROS |
| 旧 rk 工具回归 | 明确引用新源码后，SDK/完整单元/视频集成共 5 个 CTest 组通过；onnx_segment_image 编译通过 |
| 构建入口单元测试 | 本地 6 项通过，覆盖构建范围、RKNN 路径含空格、仅编译模式、错误参数、ROS 环境检查和失败即停止 |

最初测试容器只有 OpenCV 运行库，统一构建缺开发包而失败；补装开发包后通过。
补充 CTest 配置文件生成以兼容 colcon 的 CMake 测试执行器，并再次运行统一入口。
全包共用 SDK 编译参数会让 ROS 包提示部分参数未使用，这是非致命 CMake 警告，
不表示 ROS 包未链接 SDK，也未以隐藏错误的方式处理。

## 动态 BEV 原待办复测

迁入前的动态 SDK 用例曾返回 NO_ROW。调整测试相机可见 ROI 为 1.5–5.5 m 后，
本轮四组 roll/pitch（各 ±0.08 rad）均返回 VALID。
合成行中心真值为 +0.1 m，恢复值范围为 0.0990554–0.0997099 m。
该测试只替换神经网络后端，实际执行去畸变、动态投影、选行与拟合。
没有放宽生产近端支持条件或修改实际算法来令测试通过。
ROS 另验证图像时刻姿态快照、单调时间映射、无姿态不静态回退和配置模式校验。

以上是几何与接口验证，不是公开数据集精度、真机 NPU 性能、真实 GNSS/相机同步、
地形适应性或闭环转向验收。本轮未验证新迁入版本 ARM64/RKNN 交叉编译和板端运行；
后续需针对目标工具链、OpenCV、RKNN runtime/驱动重新验证。
