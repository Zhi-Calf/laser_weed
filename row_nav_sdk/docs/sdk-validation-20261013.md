# SDK v0.1 开发机验证记录（2026-10-13）

## 范围

本次增加纯 C ABI、共享库、独立调用示例、安装与打包规则。复用现有分割、去畸变、BEV、
行提取和中心线拟合代码，没有重新训练或更换模型。不包含 CAN 控制、GNSS/IMU 融合。

环境：macOS ARM64、AppleClang 16、OpenCV 4.10.0；RKNN关闭，标定GUI工具关闭。
OpenCV在临时目录构建，不要求修改系统目录权限。此环境产物是dylib，不是Linux so。

## 已验证

- Release全工程构建成功。
- CTest 4/4测试组通过：sdk_c_header、sdk_contract、rk_nav_unit_tests、
  rk_nav_video_integration_test。原有单元测试包含50个通过用例，视频组仅覆盖错误路径，
  不代表真实摄像头采集通过。
- SDK契约覆盖：带stride填充图像、RGB/BGR、输入不被修改、左右符号、无行、低置信度、
  推理错误与异常、推理前/后帧龄失效、错误ABI、扩展结构体尾部保护、分辨率不匹配、
  缓冲区不足、不支持格式、未来时间戳、反复创建销毁、无效标定拒绝。
- 动态库符号表仅导出9个约定C函数，没有公开内部C++符号。
- 打包脚本成功生成压缩包和SHA256清单；文件校验通过。
- 将SDK安装到项目外的新目录，从安装包examples独立配置和编译纯C程序，
  `integration_example --version`返回`0.1.0`，无需项目内部头文件/OpenCV开发头文件。
  本机OpenCV位于非系统路径，运行时显式提供其动态库搜索路径。
- AddressSanitizer和UndefinedBehaviorSanitizer下SDK两个测试组通过；
  第三方OpenCV本身未用sanitizer重新编译，此项不等同于完整内存泄漏认证。

## 复现入口

见sdk-integration.md；本次本机OpenCV配置目录为
`/tmp/rk-sdk-deps/install/lib/cmake/opencv4`，Release构建目录`/tmp/rk-sdk-build`。
临时路径不是交付依赖的固定安装位置。交付方须按目标系统配置OpenCV运行库。

## 未验证／发布门槛

1. MyUbuntu SSH在握手阶段断开，本次未生成或验证Linux ARM64 `.so`。
2. 未进行真实ONNX/RKNN模型推理：契约测试只替换神经网络后端，其他算法步骤真实执行；
   动态链接测试使用真正共享库的初始化和错误路径。
3. 后续恢复MyUbuntu后构建RKNN包，核对架构和动态依赖，使用已验证模型做图片回放，
   再由同事执行板端NPU推理、长时间运行和真车误差验收。
4. 同事集成时必须确认标定文件与输入分辨率一致、输入未重复去畸变、坐标符号一致；
   仅返回OK且观测为VALID才使用导航量。不合法或过期结果不得持续控制车辆。

因此本次交付级别是“SDK开发联调版”，不是“RK3588现场验收完成”。
