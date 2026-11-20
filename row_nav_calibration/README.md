# 标定工具与现场资料

新版四步标定页面（含即时照片检查、加载保护和精细选点）已实现：[页面操作说明与实际截图](docs/calibration-workbench.md)、[布局与交互验证](docs/workbench-validation-20261104.md)、[精细选点与照片检查验证](docs/precision-quality-validation-20261106.md)、[报告与缓存验证](docs/review-cache-validation-20261107.md)。

2026-11-03 已在 MyUbuntu 完成 SDK 原生测试、ROS 感知相关包构建与测试，以及 ARM64/RKNN、相机采集程序交叉编译，详见 [验证记录](docs/validation-mybuntu-20261103.md)。真实相机、NPU 和整车验证仍待完成。

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

## 页面中断与重拍（2026-11-04 更新）

- 九宫格中的 ✓ 表示该位置已有照片；可切换近、中、远轮次返回指定位置重拍。浏览器刷新后恢复所在采集组和位置；换浏览器时根据设备上的已拍照片寻找未拍位置。
- 内参检测失败后，页面列出无法读取、分辨率不一致或角点不足的照片；逐张拍摄的照片可点击“去重拍”。未执行到照片检测的故障显示整体原因，详细日志仍保留在测试包。失败后可继续当前会话。
- BEV 点位编辑自动保存在当前浏览器中；刷新后可恢复，未提交时离开页面会提示。浏览器数据被清除或换电脑时，本地草稿不可用。
- 中途可点击“保存点位草稿”，将包括空白字段的草稿保存到设备，换浏览器后仍可继续。点位绑定当前冻结照片，重新拍摄会清空该照片的旧点位。
- 草稿不能发布为算法参数。计算时仍严格检查完整性、重复点、坐标范围、标定点及独立检查点数量；成功计算并人工复核后才能正式发布。

回归证据见 [页面恢复与草稿验证](docs/operator-recovery-validation-20261104.md)。
