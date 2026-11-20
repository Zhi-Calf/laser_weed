# RK3588板端回归包制作与验证（2026-11-18）

已生成SDK0.2.4的ARM64/RKNN回归包，未执行板端NPU推理。本包用于验证最近NMS与mask重建修改，不替代车辆导航验收。

## 交付物

- 本地Mac：`/Users/zhouboyang/workspace/rk/artifacts/releases/rk3588-regression-sdk024-20261118.tar.gz`，旁边有.sha256校验文件。
- MyUbuntu：`/home/zhouboyang/workspace/rk3588-regression-sdk024-20261118.tar.gz`，约5.9MiB。
- 同事操作手册：[板端测试说明](board-regression-guide.md)，包内README.md是同一份说明。

包中包含ARM64 `.so`/`.h`、同源segmentation_replay可执行文件、当前RK3588 INT8模型、12张固定公开测试图、同输入的CPU参考mask、进程局部使用的librknnrt以及标准库Python自检/回归脚本。无需板端联网或Python第三方库。

使用已有AWU202 GCC9.3交叉编译工具链、目标板OpenCV4.5.4 sysroot，RKNN=ON、tools=ON。本包仍依赖板端OpenCV4.5库（含imgcodecs及其依赖）和匹配的NPU内核驱动。检查失败不自动安装依赖，不写/usr/lib，不启动相机、ROS或车辆控制。

RK3588专用模型，设备树不能确认RK3588时拒绝执行；不能用在RK3576。头文件兼容既有C接口；固定图测试程序直接复用同源分割后端/解码器，未执行完整C API导航process或真实相机标定。

## 可复现打包

`package_board_regression.py`要求构建属于当前源码树且RKNN开启，校验可执行文件/SDK/runtime为ARM64 ELF，执行cmake install到临时目录，再组成新包。记录所有代码文件的SHA256以及源码树摘要，而不是把rsync出来的目录冒充某个已提交版本。

测试图从430张现有test图片按文件名排序后等距选12张；原始文件路径、图片hash、CPU参考mask hash记录在package-info.json。非随机精度样本，CPU参考不是人工真值，不设“必须逐像素一致”的错误验收门槛。模型转换记录随包保存。checksums.json校验包中所有文件；额外文件/改动导致检查失败。

```bash
python3 scripts/package_board_regression.py \
  --build /absolute/arm64-build \
  --model /absolute/rk3588-int8.rknn --runtime /absolute/librknnrt.so \
  --images /absolute/public-test-images --reference /absolute/native-cpu-run \
  --output /absolute/new-kit-directory
```

在板端由同事依次执行 `python3 run_regression.py`、`python3 run_regression.py --run`，第一次无错误后可加 `--rounds 20`。进程只加载一次模型，第一轮生成图像，其余轮记录耗时和每轮RSS。运行失败也写report并打包；强制终止/断电不保证有自动压缩包。结果目录不覆盖已有数据，不放进包内，避免污染下一次完整性检查。

## 实际验证

- MyUbuntu新版隔离OpenCV4.8.1主机环境七组CTest通过，新增包校验、文件篡改拒绝、报告归档及“检查完整不等于验收通过”回归。
- 真实ONNX模型执行2图×3轮共6次infer，CSV6条、RSS3条、mask仅2张，确认单进程重复执行与第一轮保存策略。
- ARM64/RKNN/tools开启交叉编译通过；ELF目标检查与包文件校验通过。
- 在x86主机执行--run被明确拒绝并生成失败压缩包；没有伪装为板端通过。
- 本次没有实机驱动、NPU推理、运行时兼容性、长期内存、量化精度或端到端车辆验证。包中不修改旧算法默认部署，需收到现场结果后再决定正式切换。

测试报告只输出“固定图片推理完成，精度和车辆验收待定”；board_acceptance始终false。完整结果发回后，开发人员再与CPU参考做mask/实例数/时序耗时对照，不把非空mask数量当正确率。

本地下载后归档SHA256：`f37dd292da11fa185dcf99a3e7057f02cf56b6e84e51ec25fc62e11fbd3cf9cf`。Mac上验证归档hash，再从压缩包解压到临时目录，执行只检查模式成功；未执行ARM64二进制。压缩包大文件保留在artifacts/releases，不入Git；制作脚本、操作手册及测试进入Git。
