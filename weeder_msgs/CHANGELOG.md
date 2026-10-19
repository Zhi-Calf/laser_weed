# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Removed

- 删除仓库内无调用方的 `CalibrationCommand.srv`、`SetImplementMode.srv`
  和 `ExecutePath.action`。
- 删除 `SetPathSource.srv`；`global_path_planner` 改为从 YAML 读取地块顶点、
  垄宽、田边余量和 PTO 行内分辨率。
- 删除无节点发布或订阅的 `CalibrationResult.msg`、`FaultReport.msg`、
  `Heartbeat.msg`、`ImplementCmd.msg`、`ImplementMode.msg`、`NodeHealth.msg`、
  `RobotStatus.msg` 和 `SystemHealth.msg`。

### Changed

- **BREAKING:** 包名由 `tractor_msgs` 改为 `weeder_msgs`。
- `ChassisState` 改为激光除草机第二版 CAN 协议定义，并从 `weeder_base` 迁入本包。
- **破坏性变更：** 用 `CalibrationCommand.srv` 取代 `CalibrationCmd.msg`。
  标定指令从"向 `/calibration_cmd` 话题发布"改为"调用 `/calibration_manager/command`
  服务"：响应携带 `accepted` + `state` + `reason`，调用方不必再轮询
  `/calibration_manager/status` 判断指令是否被受理（被拒的 START 会直接说明原因）。
  服务端由 `tractor_calibration` 提供；`robot-tool` 的 `tractor_calibration_ui` 必须从
  发布者改为服务客户端。
- **BREAKING:** renamed the package `agri_msgs` -> `tractor_msgs`. Downstream
  code must update `find_package`/`<depend>` entries and the
  `agri_msgs/msg/...` include paths and interface names to `tractor_msgs`.
- Split out of the MegaTrak workspace into a standalone repository, consumed
  there as a git submodule.

## [0.1.0] - 2026-03-03

### Added

- `ChassisCmd.msg` — 车辆运动控制指令（速度 + 前轮转角）
- `ChassisState.msg` — chassis state feedback from VCU CAN bus (V2 protocol)
- `CalibrationCmd.msg` — calibration command (type, action, parameters)
- `CalibrationResult.msg` — calibration result (speed, steering, deadzone, slip)
- `RobotStatus.msg` — robot status for cloud reporting
- `TaskState.msg` — task execution state for telemetry
- Extracted from `robot-core/agri_msgs` into standalone organization-wide package
