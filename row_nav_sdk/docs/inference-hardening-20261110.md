# 推理诊断与解码检查（2026-11-10）

本次只修改 row_nav_sdk，保留既有结构尺寸、ABI 版本和同步调用语义。新增两个导出符号：`rk_row_nav_diagnostics_init`、`rk_row_nav_get_diagnostics`。新调用方需链接本次或更新的库；旧调用方可继续使用原接口。

## 诊断读取

每次 process 返回后，用初始化后的 diagnostics 结构调用 getter。同一 handle 仍要求串行访问。available=1 只表示上次 process 成功返回，包括无行、低置信度和过期帧，不代表导航有效。错误或尚未调用时 available=0；不可将此时的部分计时作为完整样本。

提供去畸变、整个推理、几何/跟踪、总处理和帧龄；RKNN 还提供前处理、输入设置、NPU 执行、输出转换、后处理。`detailed_inference=0` 时细分计时不可用，包括提前拒绝的过期帧和 ONNX。细分项包含在 inference_ms 内，不能重复累加。总耗时包含接口检查等额外开销，不保证精确等于分项之和。当前 BEV、选行、拟合、时序跟踪合计为 geometry_tracking_ms，尚未分别暴露。

C 使用示例已更新。需要批量统计时，调用方将字段写入 CSV，运行 `python3 scripts/summarize_diagnostics.py timing.csv`，输出均值、最近秩 P50/P95 和最大值。脚本只纳入 available=1、有限且非负的数值；RKNN 细分项还要求 detailed_inference=1。应分开采集后端、分辨率、冷启动/预热、负载和有效/过期状态；不要将混合样本统计当作稳定帧率结论。脚本不自动预热或采样。

## 模型范围与资源释放

当前解码器明确支持 640×640、单类、32 维 mask 原型、DFL16、10 或 13 输出的 Rockchip 导出结构。RKNN 初始化检查 batch、RGB 通道、输入输出布局、维度、元素数、输出类型及量化参数，然后检查完整输出结构。ONNX 初始化检查输入配置为 640，实际输出结构仍在 forward 后检查；本次没有实现通用多模型解析。

INT8/UINT8 使用 affine asymmetric scale/zero point 转 float；浮点输出由 RKNN 按 want_float 返回。转换检查缓冲区长度，NCHW float 保留 memcpy 路径。解码拒绝非有限数值和不连续张量。完整 mask 与快速合并 mask 路径仍非逐像素等价；合成回归验证其重叠，真实细行精度需要数据回放验收。

成功获取 RKNN 输出后立即建立自动释放守卫。转换异常、提前返回或内存分配异常均触发释放；本次不改变 rknn_outputs_get 失败时由 runtime 管理资源的约定。

## 验证边界

新增合成输出测试覆盖 10/13 布局、空检测、单实例 mask 两条路径、错误尺寸/布局/阈值、NaN、INT8/UINT8/float 转换、短缓冲区与清理守卫异常释放。C 合约测试覆盖诊断字段、扩展结构尾部保留、错误后失效。测试使用合成张量及模拟分割器，不等于真实权重或 NPU 验收。

2026-11-10 实际验证结果：MyUbuntu 隔离目录 `/home/zhouboyang/workspace/sdk-hardening-20261110`，Ubuntu 主机 OpenCV 4.5.4。主机构建与 CTest 的 sdk_c_header、sdk_contract、sdk_dynamic_contract、rk_nav_unit_tests 四组全部通过。安装包纯 C consumer 构建通过，`--version` 输出 0.2.1。使用现有 AWU202 GCC 9.3 工具链及目标 OpenCV 4.5.4，RKNN=ON、tests=OFF 的交叉编译与安装通过；file 确认 ARM aarch64，nm 确认新增诊断符号导出。CSV 统计用 1–100 的已知样本验证 P95=95，失效样本排除。没有执行板端 NPU、真实权重解码比对、长期内存测试或真实图像精度验收；异常释放测试验证的是守卫机制。
