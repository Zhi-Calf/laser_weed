# 新版算法板端测试说明

本包用于布谷鸟AWU202／RK3588的固定图片测试。不是RK3576包。无需相机、标定板、显示器或板端联网，也不启动ROS和车辆控制。

## 准备

把整个压缩包通过网线连接的笔记本传到板子用户目录，解压后进入解压目录。请保留原包及其.sha256校验文件。Windows可用现有SSH/SFTP工具传文件，Linux/macOS也可用SFTP或scp。板子不需要连接MyUbuntu才能执行，普通笔记本只负责传文件和打开SSH终端。

没有改动系统库，不需要sudo，不要将包中lib文件复制到/usr/lib。包中携带测试用RKNN runtime，仅对测试进程生效；板子仍需已有OpenCV共享库和NPU驱动。库不匹配时把报告发回，不要直接升级系统。

## 1. 检查包

在板端SSH终端、解压目录内运行：

```bash
python3 run_regression.py
```

此操作只检查文件完整性和记录系统信息，不运行NPU。看到 `package_integrity_ok_not_board_test` 只代表包完整，不代表板端推理通过。在x86笔记本上也只能完成这一步。

## 2. 先做一次推理

```bash
python3 run_regression.py --run
```

程序先检查ARM64、设备树中的RK3588、库依赖和SDK版本，再运行12张固定图片。结束会显示结果目录和要发回的压缩包。若失败，同样会生成报告。不要只发送“成功/失败”文字，请发送显示的.tar.gz文件。

`fixed_image_inference_completed_accuracy_and_vehicle_acceptance_pending` 表示固定图片处理已完成，还需要开发人员检查图像和数值，不表示车辆导航验收完成。

## 3. 连续测试

第一次推理无错误后执行：

```bash
python3 run_regression.py --run --rounds 20
```

20轮共240帧，模型在同一进程只加载一次。每轮处理相同12张图，不是不同田间视频。只保存第一轮的mask和叠加图，每帧均记录耗时和实例数，每轮记录进程RSS。第一轮作为预热从汇总耗时排除，但原始计时保留。RSS变化只作诊断，不能单凭20轮判定没有内存泄漏。

若程序没有正常结束，先把当前结果目录整个压缩发回；因断电或被强制终止，自动压缩包可能还没生成。单次超时上限30分钟，不要在测试过程中断电。

## 发回什么

发回程序最后显示的 `board-results-日期时间.tar.gz`。默认结果保存在解压目录旁边，重跑创建新目录，不覆盖已有记录。

- report.json：系统、包身份、依赖检查、错误和耗时汇总。
- inference.log：NPU运行及错误信息。
- inference/frames.csv：每帧完整infer耗时（包括前后处理，不含图片读写）。
- inference/memory.csv：每轮RSS，单位KiB，Linux取不到时为-1。
- inference/masks、overlays：第一轮真实mask和叠加图。

程序不会采集GNSS、IMU、相机或米制导航误差。reference-masks是相同模型主机ONNX CPU参考，不是人工真值；由开发人员收到结果后比对，量化差异不要求每个像素完全相同。画面有绿色行带也不代表田间精度达标。

SDK头文件在include/，ARM64共享库在lib/。本包的测试程序使用同源分割后端，测试分割/NPU/后处理，不会绕过缺少实测标定的问题伪造车辆坐标。正式集成仍需有效相机内外参，并单独验收完整C接口和车辆坐标输出。
