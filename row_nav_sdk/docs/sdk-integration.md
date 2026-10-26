# 稻行导航 SDK v0.1

交付入口是 `include/rk_row_nav.h` 和 Linux 上的 `lib/librk_row_nav.so.1`。
内部 C++/OpenCV 类型不出现在公开头文件。macOS 开发验证生成 `.dylib`，不能部署到
RK3588。ARM64 Linux 包必须在板端或匹配 sysroot 的交叉编译环境生成。

## 构建、安装、独立集成

```sh
# MyUbuntu / 开发机 ONNX 版本，需要 OpenCV C++ 开发库、CMake、C/C++ 编译器
cmake -S . -B build-sdk -DCMAKE_BUILD_TYPE=Release \
  -DRK_NAV_BUILD_CALIBRATION_TOOLS=OFF
cmake --build build-sdk --parallel 4
ctest --test-dir build-sdk --output-on-failure
python3 scripts/package_sdk.py --build build-sdk --output output/sdk-0.1-host

# 下列示例只读取安装包，不依赖项目源码或 OpenCV 头文件
cmake -S output/sdk-0.1-host/examples -B build-sdk-consumer \
  -DCMAKE_PREFIX_PATH="$PWD/output/sdk-0.1-host"
cmake --build build-sdk-consumer
build-sdk-consumer/integration_example --version
```

RK3588 上增加 `-DRK_NAV_WITH_RKNN=ON -DRKNN_ROOT=/path/to/rknn-runtime`。
交叉编译同时使用 `cmake/toolchains/awu202-aarch64.cmake` 并提供与板端匹配的 OpenCV。
用 `-DRK_ROW_NAV_RELEASE_MODEL=/path/to/verified-model.rknn` 显式包含已验证模型。
默认包没有模型，配置模板也不能直接导航。OpenCV 动态库、RKNN Runtime 与 NPU 驱动
由目标平台提供，打包器不自动复制第三方依赖。RKNN版本兼容、模型精度、量化效果需要
另行记录验收证据。`ldd lib/librk_row_nav.so.1` 必须没有 `not found`。

ONNX/RKNN后端复用现有 Rockchip YOLOv8-seg 解码器，模型输出张量必须与该解码器匹配；
不是任意 Ultralytics 默认导出模型都能直接替换。首版不重新选模型或训练模型。

## 最小调用约定

先用 `rk_row_nav_config_init` 初始化配置，设置模型、内参、BEV文件和后端；调用
`rk_row_nav_create`，成功后循环 `rk_row_nav_process`，最后 `rk_row_nav_destroy`。
完整可编译调用代码位于 `examples/integration_example.c`。示例读取无文件头、连续
存储的 BGR888 原始帧；返回 VALID 才打印导航量。回放示例以送入时间作为采集时间。

- v1支持BGR888/RGB888，单个平面；NV12等格式返回 UNSUPPORTED。
- `stride_bytes >= width*3`，`data_size_bytes >= stride_bytes*height`，所有行含末行
  均须有完整stride空间。调用方负责指针与实际内存匹配；图像不会被改写或在返回后引用。
- 输入为**原始畸变图像**；SDK按内参去畸变，保留原相机矩阵、尺寸。BEV必须在这一
  去畸变坐标系标定。不要提前裁剪、缩放或重复去畸变。
- 所有结构体先调用对应init；`abi_version=1`、`struct_size>=v1大小`。
  库只写已知前缀；不支持的ABI明确失败。初始化函数用于当前头文件大小的自有结构体。
- 每个句柄串行调用，包括销毁；多个句柄可独立调用。销毁后指针不得复用。
- 创建会复制配置所需值；配置路径字符串只需在create期间有效。
- `capture_time_ns` 使用SDK `rk_row_nav_now_ns()`同源单调时钟；Linux为
  CLOCK_MONOTONIC。不能使用UTC、设备独立时钟或另一台机器的时间戳；时间戳0/未来值
  返回参数错误。帧龄在推理前、后检查，默认最大250ms。
- 接口返回码描述调用成功/失败。调用成功仍可能是 NO_ROW、LOW_CONFIDENCE、STALE。
  **只有返回OK且 observation_status=VALID 时导航量才有效**。其他状态及错误将导航量
  清为NaN、置信度为0，不能沿用上一帧数值。结果结构体本身为空或ABI/大小不匹配时，
  库不会写入它；调用方必须先检查返回码，不能继续读取旧结果。
- `get_last_error()`返回当前线程最近create/process错误；字符串有效到下一次同线程
  create/process。创建失败也可查询。异常在C边界捕获。
- `processing_ms`是本次调用处理时长，`frame_age_ms`是输出时距采集的总帧龄。
  参数或推理错误时耗时字段不用于性能统计。

## 坐标和结果定义

车体x向前、y向左。拟合路径 `y = slope*x + intercept`：
`lateral_error_m=intercept`，正值表示路径位于车辆左侧；
`heading_error_rad=atan(slope)`，正值表示路径朝左偏转。
这与“车体相对路径偏差”的符号可能相反，控制团队须按该定义接入。
`lookahead_m`为内部拟合器前视距离（当前2m），横向误差仍在车体x=0处计算。
`confidence`是模型、稻行几何、拟合评分的组合，不是经过校准的正确概率。
算法输出观测；控制器负责转向、限速、急停以及连续失效处理。

中心线先执行100次RANSAC获得初始内点，再进行最多5轮局部优化：最小二乘重拟合、
重新计算所有点的垂直残差、更新5cm阈值内点；内点集合稳定后提前停止。当前模型仍是直线。

内参和BEV文件必须有 `calibration_valid: 1`；模板默认禁用，不能仅为通过加载而改标记。
内参模板须补充实测结果及接受标记。BEV矩阵已编码相机相对地面/车辆关系；v0.1直接
使用这份矩阵，不额外接收一个尚无实现的6自由度外参接口。
当前行距范围0.25–1.5m、置信度阈值可配置；后续可扩展几何参数。

2026-10-20起默认使用帧内多行关联和左右边界联合拟合，随后再拟合导航中心线。
既有BEV YAML可增加`row_extractor`块，`method: nearest_band`回退旧算法，
`expected_spacing_m`指定实际行距（0代表未知），`spacing_tolerance_m`指定容差。
本次C ABI保持不变。完整模块说明、默认参数、合成对照和限制见multi-row-geometry-20261020.md。

跨帧跟踪通过BEV YAML的`row_tracker: { enabled: 1 }`启用，默认关闭。
启用时必须使用associated模式，连续帧的采集时间戳和帧编号都严格递增。
默认同一行对连续3帧确认后才返回VALID；短时丢线只保留内部预测，不返回有效导航量。
跟踪诊断、参数、重捕获与延迟实验见temporal-row-tracking-20261020.md。

## 验证与发布边界

`sdk_c_header`使用纯C编译并动态链接真正SDK，覆盖ABI初始化及错误路径。
`sdk_contract`仅替换神经网络后端，执行真实去畸变、BEV、行提取和拟合，验证带padding的
BGR/RGB、偏差符号、异常、过期帧、无行、低置信度和资源生命周期。测试替身不包含在
发布动态库中。这些结果不等同于真实模型精度、RKNN推理和真车验收。

打包目录必须是新路径；输出包括SHA256SUMS和压缩包校验值。独立工程要验证移动安装
目录后的加载；RK3588包另需实际NPU推理、连续运行的内存/耗时统计和实测导航误差。

## SDK 0.2 动态 BEV 扩展

0.2 新增 `rk_row_nav_attitude_t`、`rk_row_nav_attitude_init`、
`rk_row_nav_dynamic_bev_enabled` 和 `rk_row_nav_process_with_attitude`。
既有 v1 结构体布局、函数和 SOVERSION=1 保持不变；新接入端需要 0.2 或更高版本，
不能将新版头文件与旧版 `.so` 混用。静态模式继续调用原 `rk_row_nav_process`。

动态模式在 BEV YAML 的 `dynamic_bev` 中显式开启，要求独立的
`calibration_valid: 1`、相机安装刚体变换、车体参考点离地高度和标定参考姿态；
默认模板全部为未验收占位值。启动时将物理投影与原实测 BEV 在参考姿态下对照，
不一致即拒绝创建句柄，不能随意放宽阈值来跳过安装或坐标错误。

每帧姿态须为 ROS 车体轴下的 rad，`sample_time_ns` 和图像使用同一单调时钟，
样本不能晚于图像，且两者间隔不能超过配置龄期。动态句柄缺姿态时拒绝处理，
不会静默退回静态 BEV；非法姿态返回错误和无效导航量，不更新时序跟踪。
相机原始图像仍只去畸变一次，动态映射作用于去畸变图像上的分割结果。

几何假设、输出坐标及测试边界见 [动态 BEV 说明](dynamic-bev-20261026.md)。
