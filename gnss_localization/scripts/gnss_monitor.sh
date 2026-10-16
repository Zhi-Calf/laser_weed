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
#   │   (汇总仪表盘)      │ echo /gnss_driver/status │
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
  "$(retry "ros2 topic echo ${NOD} ${BE} /gnss_driver/status")"

tmux split-window -v $(SZ 50) -t "${SESSION}:gnss.2" \
  "$(retry "ros2 topic echo ${NOD} ${BE} ${ODOM_TOPIC} --field pose.pose")"

# GNSS 解码状态。gnss_driver 发布的话题和 D300 完全一样（/gnss_driver/fix、
# /gnss_driver/imu、/gnss_driver/status、/odometry/global），所以上面几个 pane
# 本来就通用；GNSS 特有的信息全在 GnssStatus.fix_type 里，而 `topic echo`
# 只会打印一个裸数字。这个 pane 把 GIAVP 的 status 码翻译成人能读的状态。
# 注: nsv2（第二天线星数）和 baseline 在 giavp_parser 里解析了但没有发布，
# 所以这里显示不了；要看它们得先在 gnss_driver 里发出来。
GIAVP_AWK='
/^fix_type:/       {ft=$2}
/^num_satellites:/ {ns=$2}
/^heading_deg:/    {hd=$2}
/^speed_kmh:/      {sp=$2}
/^---/ {
  n = (ft==0) ? "NO FIX"    : (ft==1) ? "SPP"       : (ft==2) ? "RTD/DGPS" :
      (ft==4) ? "RTK FIXED" : (ft==5) ? "RTK FLOAT" : (ft==6) ? "INS" : "?("ft")";
  printf "GIAVP %-10s sats=%-3s hdg=%8s deg  v=%6s km/h\n", n, ns, hd, sp;
  fflush();
}'
tmux split-window -v $(SZ 50) -t "${SESSION}:gnss.3" \
  "$(retry "stdbuf -oL ros2 topic echo ${NOD} ${BE} /gnss_driver/status | awk '${GIAVP_AWK}'")"

tmux select-pane -t "${SESSION}:gnss.0"
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" status-left "#[bold] GNSS MONITOR "

exec tmux attach -t "$SESSION"
