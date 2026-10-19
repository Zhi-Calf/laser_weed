#!/usr/bin/env bash
# Copyright 2026. Whole-package port of MegaTrak's RTK + coverage-planning stack
# into the laser-weeder ROS 2 workspace.
#
# What this does (structural only):
#   1. copies tractor_monitor, gnss_localization, global_path_planner and
#      lqr_control into the weeder workspace src/
#   2. builds them in dependency order
# It does NOT edit any config: every value that differs between the tracked
# tractor and the weeder (serial ports, antenna lever arm,差分 accounts, log dirs)
# needs real hardware data that only you have. See PORT_CHECKLIST.md.
#
# Safety: refuses to overwrite an existing destination package. If a dest already
# exists it skips and tells you, so nothing of yours is clobbered silently.
set -euo pipefail

# ---- Set these two before running ----
# Where the VM can see the MegaTrak checkout (the folder that CONTAINS
# sensor_node/, navigation_node/, monitoring_node/ 等)。
# IMPORTANT: this must sit OUTSIDE the workspace src/ (default: ~/MegaTrak-main).
# If the whole MegaTrak tree lives inside src/, colcon finds every package in it
# AND our top-level copies -> "Duplicate package names not supported".
MEGATRAK_SRC="${MEGATRAK_SRC:-$HOME/MegaTrak-main}"
# Where you extracted lqr_control (the folder that CONTAINS lqr_control/).
LQR_SRC="${LQR_SRC:-$HOME}"
# Target workspace.
WS="${WS:-$HOME/ros2_ws}"
SRC_DIR="$WS/src"

echo "== MegaTrak 源: $MEGATRAK_SRC"
echo "== lqr_control 源: $LQR_SRC/lqr_control"
echo "== 目标工作区: $WS"
echo

# Guard: MEGATRAK_SRC must NOT be inside the workspace src/, or colcon will find
# both the in-tree MegaTrak packages and our top-level copies -> duplicate names.
case "$MEGATRAK_SRC/" in
  "$SRC_DIR/"*)
    echo "!! 中止：MEGATRAK_SRC ($MEGATRAK_SRC) 在工作区 src/ 内部。"
    echo "!! 这会导致 colcon 扫到 MegaTrak 树里的包，和我们拷到 src/ 顶层的副本重名。"
    echo "!! 请把 MegaTrak-main 移到 src/ 外面（如 ~/MegaTrak-main），或设 MEGATRAK_SRC 指向外部。"
    exit 1
    ;;
esac

# Packages to copy: <dest-name> <source-path>
copy_pkg() {
  local dest_name="$1" src_path="$2"
  if [[ ! -d "$src_path" ]]; then
    echo "!! 跳过 $dest_name：源不存在 ($src_path)"
    return 1
  fi
  if [[ -e "$SRC_DIR/$dest_name" ]]; then
    echo "!! 跳过 $dest_name：目标已存在 ($SRC_DIR/$dest_name)，不覆盖。请先手动处理。"
    return 1
  fi
  cp -r "$src_path" "$SRC_DIR/$dest_name"
  echo "OK 拷贝 $dest_name"
}

mkdir -p "$SRC_DIR"

fail=0
copy_pkg tractor_monitor   "$MEGATRAK_SRC/monitoring_node/tractor_monitor"      || fail=1
copy_pkg gnss_localization "$MEGATRAK_SRC/sensor_node/gnss_localization"        || fail=1
copy_pkg global_path_planner "$MEGATRAK_SRC/navigation_node/global_path_planner" || fail=1
copy_pkg lqr_control       "$LQR_SRC/lqr_control"                               || fail=1

# weeder_msgs 是本项目自己的接口包，不再从 MegaTrak 的旧消息包拷贝。
if [[ ! -d "$SRC_DIR/weeder_msgs" ]]; then
  echo "!! 缺少 weeder_msgs：请先把项目自有的 weeder_msgs 包放入 $SRC_DIR"
  fail=1
fi

echo
if [[ "$fail" -ne 0 ]]; then
  echo "!! 有包未拷贝（见上）。确认源路径与目标是否已存在后重跑。"
  echo "!! 未继续构建。"
  exit 1
fi

# The params_in_yaml test in gnss_localization / global_path_planner calls
# ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/check_params.py -- a helper shared by
# all packages that lives in the MegaTrak repo ROOT scripts/ (not inside any
# package). After copying packages flat into src/, that relative path resolves to
# $WS/scripts/, so we must stage the shared scripts dir there too, or those two
# tests fail with "check_params.py: No such file or directory". Copy the whole dir
# in case check_params.py imports a sibling module.
if [[ -d "$MEGATRAK_SRC/scripts" ]]; then
  if [[ -e "$WS/scripts" ]]; then
    echo "!! 跳过 scripts/：$WS/scripts 已存在，不覆盖（若缺 check_params.py 请手动核对）。"
  else
    cp -r "$MEGATRAK_SRC/scripts" "$WS/scripts"
    echo "OK 拷贝共享脚本 scripts/ -> $WS/scripts（params_in_yaml 测试依赖）"
  fi
else
  echo "!! 警告：$MEGATRAK_SRC/scripts 不存在，params_in_yaml 测试可能失败（不影响功能）。"
fi

echo "== 构建（依赖优先：weeder_msgs -> tractor_monitor -> 其余）=="
cd "$WS"
# shellcheck disable=SC1091
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
colcon build --packages-up-to lqr_control --symlink-install

echo
echo "== 构建完成。接下来必须做两件事 =="
echo "1) 按 PORT_CHECKLIST.md 改配置（串口/杆臂标定/差分账号/日志路径）—— 未改前不要上实车。"
echo "2) source install/setup.bash 后先跑单元测试："
echo "     colcon test --packages-select lqr_control && colcon test-result --verbose"
