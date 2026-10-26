#!/usr/bin/env bash
# 版权所有 2026。将 MegaTrak 的 RTK 和全覆盖规划软件栈整体移植到
# 激光除草机 ROS 2 工作区。
#
# 本脚本只完成结构性移植：
#   1. 将 gnss_localization、global_path_planner 和 path_tracker 复制到工作区 src/
#   2. 按依赖顺序构建
# 脚本不修改任何配置。串口、天线杆臂、差分账号和日志目录等参数与履带车不同，
# 必须根据实车硬件填写，详见 PORT_CHECKLIST.md。
#
# 安全策略：拒绝覆盖已存在的目标包。目标已存在时会跳过并提示，避免静默破坏文件。
set -euo pipefail

# ---- 运行前设置以下路径 ----
# MegaTrak 仓库所在目录，该目录应包含 sensor_node/、navigation_node/、
# monitoring_node/ 等子目录。
# 重要：该目录必须位于工作区 src/ 之外，默认路径为 ~/MegaTrak-main。
# 如果整个 MegaTrak 树位于 src/ 内，colcon 会同时扫描其中的包和顶层副本，
# 导致“Duplicate package names not supported”错误。
MEGATRAK_SRC="${MEGATRAK_SRC:-$HOME/MegaTrak-main}"
# path_tracker 解压位置，该目录应包含 path_tracker/ 子目录。
PATH_TRACKER_SRC="${PATH_TRACKER_SRC:-$HOME}"
# 目标工作区。
WS="${WS:-$HOME/ros2_ws}"
SRC_DIR="$WS/src"

echo "== MegaTrak 源: $MEGATRAK_SRC"
echo "== path_tracker 源: $PATH_TRACKER_SRC/path_tracker"
echo "== 目标工作区: $WS"
echo

# 路径保护：MEGATRAK_SRC 不得位于工作区 src/ 内，否则 colcon 会同时发现
# MegaTrak 树内包和顶层副本，造成包名重复。
case "$MEGATRAK_SRC/" in
  "$SRC_DIR/"*)
    echo "!! 中止：MEGATRAK_SRC ($MEGATRAK_SRC) 在工作区 src/ 内部。"
    echo "!! 这会导致 colcon 扫到 MegaTrak 树里的包，和我们拷到 src/ 顶层的副本重名。"
    echo "!! 请把 MegaTrak-main 移到 src/ 外面（如 ~/MegaTrak-main），或设 MEGATRAK_SRC 指向外部。"
    exit 1
    ;;
esac

# 待复制的包：<目标名称> <源路径>
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
copy_pkg gnss_localization "$MEGATRAK_SRC/sensor_node/gnss_localization"        || fail=1
copy_pkg global_path_planner "$MEGATRAK_SRC/navigation_node/global_path_planner" || fail=1
copy_pkg path_tracker       "$PATH_TRACKER_SRC/path_tracker"                      || fail=1

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

# gnss_localization 和 global_path_planner 的 params_in_yaml 测试会调用
# ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/check_params.py。该共享脚本位于
# MegaTrak 仓库根目录的 scripts/ 中。包复制到 src/ 后，相对路径会解析为
# $WS/scripts/，因此必须同步复制该目录，否则相关测试会报告文件不存在。
# 这里复制整个目录，以兼容 check_params.py 导入同级模块的情况。
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

echo "== 构建（依赖优先：weeder_msgs -> 其余）=="
cd "$WS"
# shellcheck disable=SC1091
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
colcon build --packages-up-to path_tracker --symlink-install

echo
echo "== 构建完成。接下来必须做两件事 =="
echo "1) 按 PORT_CHECKLIST.md 改配置（串口/杆臂标定/差分账号/日志路径）—— 未改前不要上实车。"
echo "2) source install/setup.bash 后先跑单元测试："
echo "     colcon test --packages-select path_tracker && colcon test-result --verbose"
