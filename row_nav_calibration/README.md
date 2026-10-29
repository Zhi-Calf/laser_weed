# 标定工具与现场资料

完整的内参、BEV 标定页面与采集程序，与算法库统一安装；不启动车辆控制。

## 构建与安装（开发人员）

在 laser_weed 仓库根目录执行：

```sh
colcon build --packages-up-to row_nav_calibration crop_row_perception
```

也可以只构建标定工具（不需要 ROS）：

```sh
cmake -S row_nav_calibration -B build/calibration -DCMAKE_INSTALL_PREFIX="$PWD/install/calibration"
cmake --build build/calibration -j4
cmake --install build/calibration
```

默认编译 C++ 相机采集程序，需要 C++17、CMake 和 OpenCV 4 开发包。
纯页面演示可加 `-DROW_NAV_CALIBRATION_BUILD_CAPTURE=OFF`，这种安装不能真机拍照。
C++ 交叉编译使用已有工具链文件；Python 依赖在目标机另外安装。

## 准备 Python（部署人员，一次）

```sh
python3 -m venv ~/.venvs/row-nav-calibration
~/.venvs/row-nav-calibration/bin/pip install -r row_nav_calibration/requirements.txt
```

使用 Python 3.10 或以上，隔离安装带 ChArUco 的 OpenCV；不替换板端系统 OpenCV。

## 启动页面

colcon 安装后：

```sh
~/.venvs/row-nav-calibration/bin/python install/row_nav_calibration/bin/row-nav-calibration --mode hardware --host 0.0.0.0 --camera-device /dev/video12
```

如果使用上面的独立 CMake 安装，启动文件位于 `install/calibration/bin/row-nav-calibration`。
`--merge-install` 的 colcon 安装则位于 `install/bin/row-nav-calibration`。
纯演示把 `hardware` 改为 `mock`，演示结果不能发布为正式参数。
先停止占用相机的感知/采集进程，同事在笔记本打开 `http://设备IP:8080`。
不需要板端显示器。页面按钮、照片回显与重拍操作见下面的现场手册。

## 所有现场文件

[现场操作手册](output/calibration-guide-v4/田间标定台现场操作手册_v4.0.docx)

[四张操作示意图](output/calibration-guide-v4/标定操作示意图_v4.pdf)

[A2 内参标定板](output/calibration-guide-v4/内参ChArUco标定板_A2.pdf)

[A4 外参地面标记 01–12](output/calibration-guide-v4/BEV地面棋盘标记_A4_01至12.pdf)

安装后这些文件在 `share/row_nav_calibration/output/calibration-guide-v4`。
打印按实际尺寸，不缩放；按手册测量检查线与格子尺寸。

## 参数与算法接入

默认数据在 `~/.local/share/laser_weed/calibration/`，包括 sessions、imports、runtime。
可通过 `ROW_NAV_CALIBRATION_DATA` 指定数据目录。手册内原开发目录示例不作为安装路径；
以页面发布结果显示的实际路径为准。
通过页面检查并确认后，正式参数在 `runtime/current`。
算法读取其中 `sdk-calibration.json` 指向的内参和 BEV 文件；ROS 使用
`observer.calibration.yaml` 叠加到正常配置，之后重启感知程序。

坐标约定不变：后轮轴中心地面投影为原点，前向 x、左向 y；BEV 为地面映射。
迁入没有改变标定算法或验收门槛。详见 [集成说明](docs/integration.md)。

## 回归测试

在上述 Python 环境安装 pytest，然后：

```sh
cmake -S row_nav_calibration -B build/calibration -DPython3_EXECUTABLE="$HOME/.venvs/row-nav-calibration/bin/python"
cmake --build build/calibration
ctest --test-dir build/calibration --output-on-failure
```

Node 可用时还运行页面失败重试测试。没有硬件时，测试通过仅代表软件流程通过。

本次迁入的已验证项与目标机待验证项见 [验证记录](docs/migration-validation.md)。

## 姿态方向核查

页面已增加首次安装方向核查，操作和 ROS 接入见 [姿态方向核查说明](docs/attitude-direction-review.md)。本步骤不会自动开启动态 BEV。

## 动态 BEV 标定

页面可选择同时计算动态 BEV，并发布相机外参、同步姿态证据和匹配的 ROS 覆盖配置。
动态模式需要重新编译采集程序，并使用支持曝光起始时间的相机驱动。
详细坐标约定、操作步骤及目标机待验项见 [动态 BEV 标定说明](docs/dynamic-bev-calibration.md)。
