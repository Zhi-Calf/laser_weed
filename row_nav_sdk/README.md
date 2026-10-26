# RowNav SDK：运行算法的唯一维护位置

本目录是纯 C/C++ / CMake 算法库，不依赖 ROS。从 `rk` 提交 `1269231` 迁入，
初始文件与 SHA256 记录在 `migration-origin.json`。后续算法在这里修改，不回改旧副本。
该清单是历史导入凭据，不是要求源码永远不可变的构建锁。

迁入内容：ONNX/RKNN 推理及后处理、去畸变、BEV 标定读取和动态投影、行关联/拟合/跟踪、
C ABI、诊断输出与相关测试。可选提供离线 BEV 标定工具。
没有迁入训练环境、数据集、权重、视频、CAN/GNSS 驱动或底盘控制。
原 `rk` 未提供独立 SDK LICENSE，本次不擅自添加对外开源授权；包登记为内部项目授权标识。
第三方依赖仍遵守各自许可，对外分发前需确认项目及模型许可。

## 不用 ROS：独立编译 SDK

要求 C++17、CMake 3.16+、OpenCV 4 开发包和 Threads；Linux 通常安装 `libopencv-dev`。

```bash
cmake -S row_nav_sdk -B build/row-nav-sdk \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/install/row-nav-sdk"
cmake --build build/row-nav-sdk --parallel 4
ctest --test-dir build/row-nav-sdk --output-on-failure
cmake --install build/row-nav-sdk
```

安装产物为 `include/rk_row_nav.h`、`lib/librk_row_nav.so`（Linux）、
`lib/cmake/RowNav`、使用示例与配置模板。SDK 0.2 保持 C ABI v1，调用端不需要 OpenCV 头文件。
开发机默认 ONNX。RK 平台增加 `-DRK_NAV_WITH_RKNN=ON -DRKNN_ROOT=/absolute/runtime-root`，
厂商头文件与 `librknnrt.so` 单独提供，不把 x86 库当作 ARM 库。
跨编译还需明确目标工具链和目标 OpenCV，不能只切 RKNN 开关就声称产物可上板。

需要离线 BEV 标定程序时增加 `-DROW_NAV_BUILD_TOOLS=ON`；默认不编译实验/硬件探测程序。
发布包使用本目录 `scripts/package_sdk.py` 对本目录配置生成的构建进行打包。
该脚本的 Git 提交来源是整车仓库，记录未提交变更并生成 SHA256。

## 使用 ROS：一条命令构建我们的模块

`package.xml` 仅使 colcon 将本目录识别为普通 CMake 包，没有引入 rclcpp/ament 依赖。

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_crop_row.sh
source install/crop-row-source/setup.bash
```

顺序由依赖图保证：`row_nav_sdk` 和现有 `weeder_msgs` 先完成，随后编译
`crop_row_perception`。脚本构建及测试，不启动相机、定位、规划或控制节点。
使用新构建前缀以避开原来预编译 SDK 的 CMake 缓存，不删除旧构建目录。
`--build-only` 可跳过测试；RK 平台可传 `--with-rknn --rknn-root PATH`。

正常 `colcon build` 会发现新增包，需要具备它们的开发依赖。
只构建同事原模块时可显式跳过 `row_nav_sdk crop_row_perception`，其源码和原启动文件不变。
适配节点仍保持单独启动，默认没有接入转向控制。
迁入不意味着全部原模块已在本轮重新构建；应区分源码未改与整车重新验收。

安装 overlay 运行前需 source 新路径；不要混用旧 SDK 的 `RowNav_DIR`、头文件或动态库。
安装前缀改变后重新配置构建，或通过安装后的环境脚本设置正确库路径。

## 标定与动态补偿

算法输入仍是图像及可选姿态，接口由 SDK 定义，ROS 层只做消息适配。
模型、内参、BEV 和安装参数必须匹配；模板的验收标记不能随意打开。
动态模式假定局部水平地面与固定车体离地高度，不估计地形坡度/升沉。
详见 `docs/dynamic-bev-20261026.md` 和上级 `crop_row_perception/INTEGRATION.md`。

## 本轮迁入的边界

只新增本目录，并修改我们自己的适配层依赖、构建入口、依赖说明与文档。
未修改同事的定位、规划、控制、底盘、雷达、仿真模块、现有消息或整车启动文件。
旧 `rk` 的资料保留，其 C++ 开发构建改为引用本目录的活动源码；训练/数据脚本留在原仓库。
本目录可独立取出构建，不引用 `../rk`。

SDK 独立构建和 4 组 CTest 已在 MyUbuntu 通过，含动态 SDK 契约。
完整迁移测试记录、统一 ROS 构建和安装验证结果见 `docs/migration-20261026.md`。
这些测试不等于田间精度、板端 NPU 性能或实车控制验收。
