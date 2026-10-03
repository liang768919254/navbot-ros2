#!/usr/bin/env bash
# ============================================================================
# check_sim.sh —— 只验「仿真层」：车能不能生成 + odom 帧在不在
# ============================================================================
# 为什么单独有这么一个脚本：
#   Nav2 报 `Invalid frame ID "odom"` 的根因在仿真层 —— 车没生成 ⇒ diff_drive
#   插件没加载 ⇒ 没有 /odom、没有 odom→base_footprint。链条断在最底下，
#   报错却出现在 Nav2 里。所以先把仿真层单独验通，再去跑导航。
#
# 用法：
#   bash check_sim.sh            # 清理 → 起仿真 → 过四道门 → 给结论
#   bash check_sim.sh --keep     # 验完不杀进程（方便接着手动看）
#   bash check_sim.sh --gui      # 带 gzclient（要看 3D 窗口时；无头环境别加）
#
# 四道门（和 record_navigation.sh 的 [4/9] 完全一致，改一处记得改两处）：
#   ① /spawn_entity 服务出现                     —— 车能不能生成全靠它
#   ② sim.log 出现 Successfully spawned entity   —— 车真的生成了的唯一判据
#   ③ /clock /scan /odom 话题都在
#   ④ TF odom→base_footprint 能查到              —— Nav2 要的是 TF，不是话题
#
# 本脚本故意**不 source 别的脚本**：单独一个文件丢进容器就能跑。
# ============================================================================

set -e

KEEP=0
GUI=false
for a in "$@"; do
  case "$a" in
    --keep) KEEP=1 ;;
    --gui)  GUI=true ;;
  esac
done

WORKDIR="/mnt/workspace"
T_CLI=12
PIDS=()

pbar() {
  local cur="$1" tot="$2" label="$3"
  [ "$tot" -le 0 ] && tot=1
  [ "$cur" -gt "$tot" ] && cur="$tot"
  local w=20
  local filled bar="" i
  filled=$(( cur * w / tot ))
  for ((i = 0; i < w; i++)); do
    if [ "$i" -lt "$filled" ]; then bar="${bar}#"; else bar="${bar}."; fi
  done
  printf "\r    [%s] %3d%%  %s" "$bar" $(( cur * 100 / tot )) "$label"
}

port_gazebo_busy() {
  (exec 3<>/dev/tcp/127.0.0.1/11345) 2>/dev/null || return 1
  return 0
}

node_present() { timeout -k 3 "$T_CLI" ros2 node list 2>/dev/null | grep -qx "/$1"; }

wait_topic() {   # wait_topic 上限秒 话题名
  local limit="$1" name="$2" i
  for ((i = 0; i < limit; i++)); do
    if timeout -k 3 "$T_CLI" ros2 topic list 2>/dev/null | grep -qx "$name"; then return 0; fi
    sleep 1
  done
  return 1
}

service_present() {
  timeout -k 3 "$T_CLI" ros2 service list 2>/dev/null | grep -qx "/$1"
}

has_tf() {
  local out
  out=$(timeout -k 2 6 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 || true)
  echo "$out" | grep -q "Translation"
}

# 带进度地等一个条件：等待等 label 上限秒 条件命令...
wait_until() {
  local limit="$1" label="$2"; shift 2
  local i
  local t0=$SECONDS
  for ((i = 0; i < limit; i++)); do
    if "$@" >/dev/null 2>&1; then
      pbar "$limit" "$limit" "$label  ✔ 用了 $((SECONDS - t0))s"
      echo ""
      return 0
    fi
    pbar "$i" "$limit" "$label  已等 $((SECONDS - t0))s / ${limit}s"
    sleep 1
  done
  pbar "$limit" "$limit" "$label  ✗ 超时"
  echo ""
  return 1
}

kill_all() {
  local pats="gzserver gzclient rviz2 planner_server controller_server bt_navigator \
amcl map_server smoother_server behavior_server waypoint_follower velocity_smoother \
lifecycle_manager spawn_entity robot_state_publisher Xvfb"
  local p i
  for p in $pats; do pkill -f "$p" 2>/dev/null || true; done
  pkill -f "ros2 launch" 2>/dev/null || true
  for i in 1 2 3 4 5; do
    pgrep -f "gzserver|gzclient" >/dev/null 2>&1 || break
    sleep 1
  done
  if pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then
    echo "  ⚠️  Gazebo 没在 5 秒内退出，升级到 kill -9"
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gzclient 2>/dev/null || true
    sleep 2
  fi
  sleep 1
  return 0
}

cleanup() {
  local rc=$?
  if [ "$KEEP" -eq 1 ]; then
    echo ""
    echo "(--keep：进程保留，收工自己清：bash check_sim.sh 重跑 或 pkill -9 -f gzserver)"
    return 0
  fi
  echo ""
  echo "清理..."
  local p
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  kill_all
  return "$rc"
}
trap cleanup EXIT

show_env() {
  local mem
  mem=$(free -m 2>/dev/null | awk '/Mem:/{print $3"/"$2" MB 已用"}')
  echo "· 环境快照："
  echo "    CPU 核心数：$(nproc 2>/dev/null || echo '?')"
  echo "    负载(1/5/15 分钟)：$(awk '{print $1, $2, $3}' /proc/loadavg 2>/dev/null || echo '?')"
  echo "    内存：${mem:-?}"
  if pgrep -f gzserver >/dev/null 2>&1; then
    echo "    gzserver 进程数与 CPU："
    ps -o pid,pcpu,pmem,etime,cmd -C gzserver 2>/dev/null | sed 's/^/      /' || echo "      ?"
  fi
  if port_gazebo_busy; then
    echo "    ⚠️  11345 被占用（Gazebo master 有残留）"
  else
    echo "    11345 端口空闲 ✅"
  fi
}

fail_out() {
  echo ""
  echo "──────────────────────────────────────────────"
  echo "❌ 仿真层没过：$1"
  echo "──────────────────────────────────────────────"
  show_env
  echo "· sim.log 尾部 30 行："
  tail -30 /tmp/sim.log 2>/dev/null | sed 's/^/    /' || echo "    (空)"
  echo "· 残留进程："
  if pgrep -af "gzserver|gzclient|spawn_entity" >/dev/null 2>&1; then
    pgrep -af "gzserver|gzclient|spawn_entity" 2>/dev/null | sed 's/^/    /' || true
  else
    echo "    （无）"
  fi
  echo ""
  echo "★ 下一步怎么办（按这个顺序试）："
  echo "  1. 再跑一次本脚本 —— Gazebo 第一次起得慢很常见，第二次往往就好了"
  echo "  2. 如果每次都卡在 'Calling service /spawn_entity'，说明 gzserver 活着但不干活，"
  echo "     把世界物理降下来（本世界是 ODE 1kHz，免费实例吃不消）："
  echo "       src/navbot_bringup/worlds/navbot_room.world 里"
  echo "         <max_step_size>0.001</max_step_size>          → 0.004"
  echo "         <real_time_update_rate>1000</real_time_update_rate> → 250"
  echo "      改完要重新 colcon build（world 是 install 里被 install 的资源）"
  echo "  3. 也确认一下 gazebo 本体在：gazebo --version / which gzserver"
  echo "  4. 想录 GIF 就别在这里死磕：先解决仿真层，再跑 record_navigation.sh"
  exit 1
}

echo "===== 仿真层自检（四道门）====="
[ -f /opt/ros/humble/setup.bash ]    || { echo "❌ 找不到 /opt/ros/humble/setup.bash，先跑 setup_env.sh"; exit 1; }
[ -f "$WORKDIR/install/setup.bash" ] || { echo "❌ 找不到 $WORKDIR/install/setup.bash，先 colcon build"; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKDIR/install/setup.bash"

echo "----- [0] 清理残留 -----"
kill_all
echo "  完成"
if pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then
  echo "  ⚠️  还有 Gazebo 残留，先解决它（kill -9 也杀不掉就重启实例）"
  pgrep -af "gzserver|gzclient" | sed 's/^/      /'
fi
if port_gazebo_busy; then
  echo "  ⚠️  11345 端口仍被占用 —— 残留进程在抢 master 端口，这次很可能起不来"
fi

echo "----- [1] 起仿真（gui=$GUI）-----"
: > /tmp/sim.log          # 清掉上一次的日志，免得 grep 到旧内容
ros2 launch navbot_bringup sim_navbot.launch.py gui:=$GUI > /tmp/sim.log 2>&1 &
PIDS+=("$!")
echo "  已起，逐道过门..."

echo "----- [2] 四道门 -----"
# ① /spawn_entity 服务
if wait_until 90 "① /spawn_entity 服务" service_present spawn_entity; then
  echo "    （车能不能生成，全靠这个服务）"
else
  fail_out "/spawn_entity 服务一直不出现 → gazebo_ros_factory 没加载 / gzserver 卡住"
fi

# ② spawn 成功
spawn_grep() { grep -q "Successfully spawned entity" /tmp/sim.log 2>/dev/null; }
if wait_until 120 "② 车已生成" spawn_grep; then
  echo "    $(grep -o 'Successfully spawned entity \[[a-z_]*\]' /tmp/sim.log | tail -1)"
else
  fail_out "sim.log 里始终没有 'Successfully spawned entity'（spawn_entity 卡在 Calling service）"
fi

# ③ 话题
ok=1
for t in /clock /scan /odom; do
  if wait_until 45 "③ 话题 $t" wait_topic 1 "$t"; then
    :
  else
    ok=0; echo "    ❌ $t 一直没出现"
  fi
done
[ "$ok" -eq 1 ] || fail_out "车生成了但 /clock /scan /odom 不全（底盘/雷达插件没起来）"

# ④ TF
tf_ok() { has_tf odom base_footprint; }
if wait_until 30 "④ TF odom→base_footprint" tf_ok; then
  :
else
  fail_out "话题都在但 odom→base_footprint 查不到（diff_drive 插件没发 TF）"
fi

echo ""
echo "══════════════════════════════════════════════"
echo "✅ 仿真层四道门全过"
echo "══════════════════════════════════════════════"
sed -n '/diff_drive/p' /tmp/sim.log | tail -6 | sed 's/^/    /' || true
show_env
echo ""
echo "★ 接着跑："
echo "    bash record_navigation.sh square          # 走方形录 GIF"
echo "    bash record_navigation.sh 1.0 -0.15 90    # 单点"
