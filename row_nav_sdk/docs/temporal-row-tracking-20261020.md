# 跨帧稻行跟踪实现与验证（2026-10-20）

## 接入方式

新增RowTemporalTracker及TrackedPerceptionPipeline。原PerceptionPipeline继续用于单图和
无状态B基线；跟踪包装器在其后关联当前测得的稻行，输出C版本导航观测。
SDK和row_nav_camera接入包装器，公开C头文件、ABI及9个函数不变。
默认关闭，启用时在现有、已接受的BEV YAML中加入：

```yaml
row_extractor:
   method: associated
row_tracker:
   enabled: 1
   confirmation_frames: 3
   max_missed_frames: 3
   max_coast_s: 0.35
   max_gap_s: 0.5
   association_gate_m: 0.12
   heading_gate_rad: 0.08
   target_gate_m: 0.15
   target_heading_gate_rad: 0.08
   smoothing_tau_s: 0.10
```

row_tracker不存在或enabled为0时保持上一版单帧行为。启用跟踪时必须使用associated模式。
enabled只接受整数0/1，计数项只接受有限范围整数，距离、时间和角度参数必须有效。
单张图片工具row_nav_image保留无状态处理，不进行跨图片跟踪。

每个句柄只处理同一路视频；采集时间戳必须为正且严格递增，帧编号也必须严格递增，
可以跳号。时间戳单位纳秒，来自与SDK一致的单调时钟。乱序或重复帧返回参数错误且
不更新历史。切换视频、相机或标定时重新创建SDK句柄；C++包装器也提供resetTracking。

## 跟踪逻辑

- 对当前高置信度、几何有效帧中的多条行进行一对一关联，参考车体x=0处横向位置和航向。
- 为行分配单调递增、句柄内有效的track_id；单帧id继续保留，不改写为长期ID。
- 使用时间间隔相关的alpha-beta滤波估计位置、角度及其变化率，速度估计在丢线时衰减。
  当前只使用视觉历史，不包含轮速、IMU、GNSS或坐标系运动补偿。
- 每条行需要连续确认；新目标还必须是连续同一对相邻行，不能因多条行分别确认就
  立即接受来回变化的目标行间。
- 目标锁定后，配对身份改变或与预测相比跳变过大的帧被拒绝。被拒绝的观测不校正历史，
  下一正常帧可恢复原ID。
- 短丢线只保留预测及ID。超出时间或帧数预算清除旧目标；持续变化也能在旧目标失效后
  重新捕获，避免永久锁死。长帧间隔清空旧模型，重新确认。显式reset不复用旧ID。

输出状态语义：

| 调试状态 | SDK观测 | 含义 |
|---|---|---|
| acquiring | LOW_CONFIDENCE | 本帧有测量，但同一行对尚未完成确认 |
| tracking | VALID | 本帧有高置信度有效测量，且目标已确认 |
| rejected | LOW_CONFIDENCE | 本帧几何可能有效，但不通过时间关联/跳变检查 |
| coasting | NO_ROW或LOW_CONFIDENCE | 历史目标仍保留，当前没有被接受的新观测 |
| lost | NO_ROW或LOW_CONFIDENCE | 历史已失效或尚无可确认目标 |

除VALID外不输出可用横向/航向值，C接口保持NaN、置信度0。预测仅在调试结构中出现，
不能作为识别成功或补发控制量。低置信度帧不用于确认、校正轨迹。

包装器在副本上更新跟踪器，完成后检查处理截止时间，通过后才提交历史。过期工作返回
STALE并丢弃这次状态更新。SDK传入帧龄截止时间，视频程序传入运行配置的截止时间。
处理错误不会提交候选历史。

## 调试数据

NavigationObservation内部增加tracking诊断：状态、原始几何观测、预测、是否因长间隔
重置、选中行对的长期ID，以及各轨迹的确认/当前观测/丢失帧数。
JSON中的tracking保存这些字段；每条行同时保存id与track_id。不可用数值序列化为null，
420行实验JSON已逐行解析检查。

BEV叠加图保留绿色原始中心线；跟踪输出有效时增加蓝色滤波线。丢线时不绘制预测控制线。
C接口继续输出导航结果；应用可使用原有JSON调试工具查看内部状态。

## 连续合成序列实验

固定生成器tests/support/row_sequences.hpp，评估工具tools/row_temporal_eval.cpp。
每个场景60帧、10Hz，共7组420帧；B与C使用同一BEV mask和单帧算法，没有更换模型。
所有场景的初始确认都占用2个非VALID帧，第三帧开始可有效。

| 场景 | B有效帧 | C有效帧 | 结果 |
|---|---:|---:|---|
| 静止交替抖动 | 60 | 58 | 第10帧后横向RMS从15.116毫米降至6.674毫米 |
| 缓慢位置/航向变化 | 60 | 58 | 连续有效阶段保持目标ID |
| 单帧22厘米突跳 | 60 | 57 | 第25帧拒绝；第26帧恢复原ID，无错误观测污染下一帧 |
| 两帧遮挡 | 58 | 56 | 遮挡帧无VALID；第27帧恢复原ID |
| 九帧遮挡 | 51 | 47 | 丢线后进入lost；恢复后再确认2帧，第31帧以新ID恢复 |
| 持续22厘米变化 | 60 | 53 | 旧目标先拒绝；第30帧以新ID重新捕获，延迟约500毫秒 |
| 4厘米阶跃 | 60 | 58 | 阶跃后约100毫秒进入并保持5毫米误差范围 |

滤波有效率低于单帧包含预期的确认、拒绝和重新捕获开销，不能只用平滑后的误差评价方案。
静止RMS统计排除前10帧，C有效帧的对齐测量同属相同静止场景。阶跃响应同时报告，
防止用过大的时间平滑掩盖延迟。持续目标变化和遮挡的500/200毫秒重捕获延迟属于
合成10Hz测试结果，真车控制是否可接受尚待确认。

各场景跟踪流水线几何耗时P95最大约0.924毫秒；MyUbuntu x86_64、Release、OpenCV 4.5.4。
不含相机、去畸变、模型推理、JSON序列化或文件写入，不是RK3588性能。
原始帧表和7个JSONL保存在artifacts/temporal-eval-v3/eval-v3/；帧表归档到
docs/evaluation-data/temporal-20261020.csv。

验证包括：71个单元用例、SDK契约测试、视频接口测试；ASan/UBSan验证；ARM64/RKNN
交叉编译。SDK契约覆盖配置启用、三帧确认、重复帧拒绝及过期帧不推进历史。
本轮未执行板端NPU或真实视频验收。

复现：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRK_NAV_BUILD_CALIBRATION_TOOLS=OFF
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
build/row_temporal_eval /tmp/row-temporal-new-output
```

## 适用边界

需要按实际帧率、运动和行距评估关联门限、丢线时长及滤波时间常数。
没有真实运动补偿时，急转、相机抖动或长时间遮挡可能导致拒绝和换ID。
持续且几何一致的误识别在旧目标过期后仍可能被重新确认；跟踪不能替代模型精度、标定
和实际行距先验。当前未提供外部控制指令、转向逻辑或IMU融合。
