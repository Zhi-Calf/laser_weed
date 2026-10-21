#!/usr/bin/env bash
# GNSS 分屏监控 —— 一条命令拉起 tmux 分屏，自动订阅各路 GNSS 话题。
#
#   ./scripts/gnss_monitor.sh              # 默认布局
#   ./scripts/gnss_monitor.sh -o /odometry/local
#   ./scripts/gnss_monitor.sh -s my_ws     # 指定 workspace 根目录
#   ./scripts/gnss_monitor.sh -k           # 关闭已有会话后退出
#
# 布局:
#   ┌────────────────────┬──────────────────┐
#   │                    │ echo /gnss_driver/imu    │
#   │   gnss_dashboard   ├──────────────────┤
#   │   (汇总仪表盘)      │ echo /gnss_driver/pose   │
#   │                    ├──────────────────┤
#   │                    │ echo odom pose    │
#   │                    ├──────────────────┤
#   │                    │ GNSS 解码状态      │
#   └────────────────────┴──────────────────┘
set -euo pipefail

SESSION="gnss_mon"
ODOM_TOPIC="/odometry/global"
# 解析到真实文件：`ros2 run` 走的是 install 下的软链
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
KILL_ONLY=0

# workspace 根目录：认 install/setup.bash 才算数。依次试
# 已 source 的 COLCON_PREFIX_PATH → 源码树上溯 → 当前目录上溯
find_ws() {
  local c prefix="${COLCON_PREFIX_PATH:-}"
  prefix="${prefix%%:*}"
  for c in ${prefix:+"$prefix/.."} "${SCRIPT_DIR}/../../.." "$PWD"; do
    [[ -n "$c" && -f "$c/install/setup.bash" ]] && { (cd "$c" && pwd); return; }
  done
  # 从当前目录逐级上溯
  c="$PWD"
  while [[ "$c" != "/" ]]; do
    [[ -f "$c/install/setup.bash" ]] && { echo "$c"; return; }
    c="$(dirname "$c")"
  done
  (cd "${SCRIPT_DIR}/../../.." && pwd)  # 兜底
}
WS_ROOT="$(find_ws)"

while getopts "o:s:kh" opt; do
  case "$opt" in
    o) ODOM_TOPIC="$OPTARG" ;;
    s) WS_ROOT="$(cd "$OPTARG" && pwd)" ;;
    k) KILL_ONLY=1 ;;
    h) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) exit 1 ;;
  esac
done

command -v tmux >/dev/null || { echo "需要 tmux: sudo apt install tmux"; exit 1; }

if [[ $KILL_ONLY -eq 1 ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null && echo "已关闭 $SESSION" || echo "无会话 $SESSION"
  exit 0
fi

# 每个 pane 都是独立 shell，必须各自 source
SETUP="source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || true; \
source '${WS_ROOT}/install/setup.bash' 2>/dev/null || true"

DASH="${SCRIPT_DIR}/gnss_dashboard.py"

# ros2 CLI 的后台 daemon 在本机反复卡死（topic list/echo 直接超时），
# 所以 echo/hz 一律走 --no-daemon，各 pane 自己发现，互不依赖。
NOD="--no-daemon"
# 驱动用 SensorDataQoS(BEST_EFFORT) 发布，默认 RELIABLE 订阅收不到
BE="--qos-reliability best_effort"

# 掉线自动重连：话题还没起来时不断重试，而不是直接退出留下空白 pane
retry() {
  echo "${SETUP}; while true; do $1 || true; echo '--- 等待话题, 2s 后重试 (Ctrl-C 退出) ---'; sleep 2; done"
}

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "会话 $SESSION 已存在，直接接入（-k 可先关闭）"
  exec tmux attach -t "$SESSION"
fi

tmux new-session  -d -s "$SESSION" -n gnss \
  "$(retry "python3 '${DASH}' --ros-args -p odom_topic:=${ODOM_TOPIC}")"

# 右侧竖列，占 38% 宽（左侧给仪表盘留够 ~62 列）。
# tmux >=3.4 用 `-l <n>%`，旧版只认 `-p <n>`
# 注: 各话题频率已在仪表盘里统计，这里不再开 `ros2 topic hz` pane
# ——它既不支持 --no-daemon 也不支持 QoS 选项，daemon 一卡就哑掉。
IMU_CMD="$(retry "ros2 topic echo ${NOD} ${BE} /gnss_driver/imu --field orientation")"
if tmux split-window -h -l 38% -t "${SESSION}:gnss" "$IMU_CMD" 2>/dev/null; then
  SZ() { echo "-l $1%"; }
else
  SZ() { echo "-p $1"; }
  tmux split-window -h $(SZ 38) -t "${SESSION}:gnss" "$IMU_CMD"
fi

tmux split-window -v $(SZ 66) -t "${SESSION}:gnss.1" \
  "$(retry "ros2 topic echo ${NOD} ${BE} /gnss_driver/pose")"

tmux split-window -v $(SZ 50) -t "${SESSION}:gnss.2" \
  "$(retry "ros2 topic echo ${NOD} ${BE} ${ODOM_TOPIC} --field pose.pose")"

# GNSS 位姿摘要：有效性、卫星数、航向和速度。
GIAVP_AWK='
/^valid:/       {valid=$2}
/^satellites:/  {ns=$2}
/^yaw:/         {yaw=$2}
/^speed_m_s:/   {sp=$2}
/^---/ {
  n = (valid == "true") ? "RTK FIXED" : "INVALID";
  printf "POSE %-10s sats=%-3s yaw=%8s rad  v=%7s m/s\n", n, ns, yaw, sp;
  fflush();
}'
tmux split-window -v $(SZ 50) -t "${SESSION}:gnss.3" \
  "$(retry "stdbuf -oL ros2 topic echo ${NOD} ${BE} /gnss_driver/pose | awk '${GIAVP_AWK}'")"

tmux select-pane -t "${SESSION}:gnss.0"
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" status-left "#[bold] GNSS MONITOR "

exec tmux attach -t "$SESSION"
