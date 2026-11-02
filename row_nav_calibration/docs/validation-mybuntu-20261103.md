# MyUbuntu 编译与回归验证（2026-11-03）

## 源码与环境

本次使用完整 Git 快照 `ed8be85e3ee8f45f9d7770da66f5d1260733f42a`。
随后同步远端主分支到 `f892dc4`；新增的两个提交仅涉及 path_tracker，
已核对 row_nav_sdk、row_nav_calibration、crop_row_perception、weeder_msgs 和 scripts
与本次测试快照没有差异。路径跟踪和完整 ROS 工程不在本次编译结果内。

独立验证目录：
`/home/zhouboyang/workspace/laser-weed-ed8be85-20261103.uWB4nh`。
未覆盖已有训练源码、模型或环境。

- 主机：MyUbuntu，Ubuntu 22.04 x86_64，原生 GCC 11.4。
- 交叉工具链：Buildroot 2020.08 GCC/G++ 9.3.0，glibc 2.31。
- 工具链目录：`/home/zhouboyang/toolchains/aarch64--glibc`。
- 板端 OpenCV 4.5.4 依赖副本：`/home/zhouboyang/sysroots/awu202`。
- RKNN Runtime：`/home/zhouboyang/workspace/rk/third_party/rknn_runtime`，
  库 SHA256 为 `d31fc19c85b85f6091b2bd0f6af9d962d5264a4e410bfb536402ec92bac738e8`。
- 独立 Python 环境：Python 3.10.12、OpenCV 4.14.0、NumPy 2.2.6。
  系统缺少 ensurepip，使用仅放在验证目录中的 virtualenv 引导创建环境，没有更换系统 Python。
- JavaScript 测试使用已有 Node v20.20.1；系统默认 Node v12 不支持此测试入口。

## 已通过

| 检查 | 结果 |
|---|---|
| 原生 C++ SDK 构建 | 通过，ONNX 后端 |
| 原生 CTest | 4 组全部通过，包含 51 个几何/运行核心单元用例，以及 C 头文件、SDK 契约和动态 SDK 契约 |
| 原生相机采集程序 | 编译、链接、安装通过；没有打开真实相机 |
| Python 标定回归 | 67 项通过 |
| 页面失败重试 | 2 项通过 |
| 统一构建入口 | 6 项通过；使用命令替身检查参数与错误传播，非完整 ROS 编译 |
| 安装后的网页启动入口 | 2 项通过，包括会话恢复和禁止模拟参数发布 |
| Python → C++ 动态参数 | Python 生成的合成动态 BEV 与内参被原生 C SDK 接受，动态开关读取为启用 |
| ARM64/RKNN SDK | 编译、链接、安装通过 |
| ARM64 相机采集程序 | 编译、链接、安装通过，含新的 V4L2 曝光起始时间元数据代码 |
| 可选 C++ 标定工具 | 原生与 ARM64 均编译通过；旧版 ChArUco CLI 的布局限制不变，网页使用 Python 求解器 |
| 安装包 C 消费者 | 原生编译运行 `--version` 返回 0.2.0；ARM64 编译通过 |
| 动态依赖 | 原生 ldd 没有缺失；ARM64 ELF、SONAME、RPATH 与版本需求检查通过 |

Python → C++ 检查仅替换神经网络后端为 test-only，实际加载的是 SDK 的标定和动态几何实现。
合成输入保存在验证证据包中，带 TEST_ONLY 标记；没有作为实车标定参数交付。

另外，使用现有两份 ONNX 模型做了真实 CPU 推理连通检查：

| 模型 | 返回码 | 单帧处理耗时 | 输出 |
|---|---:|---:|---|
| yolov8n-seg.onnx | 0 | 51.411 ms | NO_ROW |
| crdld-yolo11n-seg-tracked-v1-100e-rknnopt.onnx | 0 | 56.445 ms | NO_ROW |

输入是室内回放图片，调整到 1920×1080，使用合成几何参数检查调用链。
这是模型加载和推理连通检查，不是田间准确率、连续帧吞吐或板端 NPU 性能结论。

## ARM64 包

压缩包：`artifacts/validation-20261103/laser-weed-arm64-ed8be85-20261103.tar.gz`。
日志与合成验证输入：`artifacts/validation-20261103/validation-evidence-ed8be85-20261103.tar.gz`。
校验和：同目录 `SHA256SUMS`。二进制包保留在本地及 MyUbuntu，不纳入 Git。

MyUbuntu 上对应文件位于上述验证目录的 `artifacts/`。

包内包括：共享 SDK、C 头文件、CMake 导出、C 消费者示例、BEV CLI 工具、
ARM64 相机采集程序、标定页面与计算脚本、操作手册和打印材料、发布清单。
未包含模型、真实标定参数或第三方共享库；目标机需提供匹配的依赖，
Python 依赖按包内 requirements.txt 另行安装。

- SDK 为 AArch64 ELF64，SONAME 为 `librk_row_nav.so.1`，RPATH 为 `$ORIGIN`。
- SDK 最高符号需求：GLIBC 2.29、GLIBCXX 3.4.26。
- 相机采集最高符号需求：GLIBC 2.17、GLIBCXX 3.4.26。
- 实际依赖副本使用 `libopencv_*.so.4.5d`，以及 `librknnrt.so`；不是任意 OpenCV 版本都可替换。
- 交付 SDK 未启用旧版桌面工具选项，不把 aruco/highgui/videoio 直接引入 SDK 依赖。
  相机采集程序独立依赖 videoio。全工具构建另保留在验证目录，供检查编译覆盖。
- 最终库与采集程序没有保留 MyUbuntu `/home/...` 作为运行库搜索路径。

## 复现主要构建

在 MyUbuntu 的源码根目录中：

```sh
export AWU202_TOOLCHAIN=/home/zhouboyang/toolchains/aarch64--glibc
cmake -S row_nav_sdk -B build/sdk-arm64-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/row_nav_sdk/cmake/toolchains/awu202-aarch64.cmake" \
  -DOpenCV_DIR=/home/zhouboyang/sysroots/awu202/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DRK_NAV_WITH_RKNN=ON \
  -DRKNN_ROOT=/home/zhouboyang/workspace/rk/third_party/rknn_runtime \
  -DRK_NAV_BUILD_TESTS=OFF -DROW_NAV_BUILD_TOOLS=ON \
  -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_PREFIX="$PWD/install/arm64-release"
cmake --build build/sdk-arm64-release -j4
cmake --install build/sdk-arm64-release

cmake -S row_nav_calibration -B build/calibration-arm64 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/row_nav_sdk/cmake/toolchains/awu202-aarch64.cmake" \
  -DOpenCV_DIR=/home/zhouboyang/sysroots/awu202/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX="$PWD/install/arm64-release"
cmake --build build/calibration-arm64 -j4
cmake --install build/calibration-arm64
```

## 尚未验收

板子已寄走，ARM64 程序未在板端执行。本次没有核验真实相机 V4L2 的 SOE 时间戳、
GNSS/RTK 消息时钟、NPU 驱动与模型加载、真机推理性能、真实标定精度或车辆姿态补偿。
MyUbuntu 没有 ROS 2 Humble 环境，未构建和运行 crop_row_perception 及全 ROS 工作区。
这些不能由交叉编译或本机测试替代。
