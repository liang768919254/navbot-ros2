#!/bin/bash
# ============================================================================
# scan_multiplier.sh —— 扫 SmacPlanner2D 的 cost_travel_multiplier
#
# 背景（2026-11-17）：
#   S35 那轮「cost_penalty 30/120/800」全是空转 —— Humble 的 SmacPlanner2D
#   没有 cost_penalty 这个参数，写进 yaml 不报错、也不生效，一直用默认
#   cost_travel_multiplier=1.0。1.0 时 A* 会穿净空 0.16 m 的桌缝，
#   车外接半径 0.19 m ⇒ 物理过不去 ⇒ 原地磨 / Failed to make progress。
#
# 做法：起一次仿真+导航，然后在同一个活进程里动态 set 这个参数，
#       每个值调 /compute_path_to_pose 量全路线的最小净空（10 s 级）。
#       净空 ≥ 0.31 m 才算「车过得去」（外接半径 0.192 + 致命带 0.12）。
# ============================================================================
set -u
WS=/home/liang/navbot_ws
cd "$WS" || exit 1
set +u
source tools/verify/sim_fullchain/env.sh
set -u

LOG="$WS/log"
SIMLOG="$LOG/scan_sim.log"
NAVLOG="$LOG/scan_nav.log"
say() { echo "[$(date +%H:%M:%S)] $*"; }

SIM_PID=""; NAV_PID=""
cleanup() {
    say "收尾"
    [ -n "$SIM_PID" ] && kill -TERM "-$SIM_PID" 2>/dev/null
    [ -n "$NAV_PID" ] && kill -TERM "-$NAV_PID" 2>/dev/null
    sleep 3
    [ -n "$SIM_PID" ] && kill -KILL "-$SIM_PID" 2>/dev/null
    [ -n "$NAV_PID" ] && kill -KILL "-$NAV_PID" 2>/dev/null
}
trap cleanup EXIT

HOUSE_WORLD="$WS/install/navbot_bringup/share/navbot_bringup/worlds/navbot_house.world"
HOUSE_MAP="$WS/install/navbot_bringup/share/navbot_bringup/maps/navbot_house.yaml"
HOUSE_PARAMS="$WS/install/navbot_bringup/share/navbot_bringup/config/nav2_params_house.yaml"

: > "$SIMLOG"; : > "$NAVLOG"

say "起仿真"
setsid ros2 launch navbot_bringup sim_navbot.launch.py gui:=false \
    world:="$HOUSE_WORLD" x:=-1.0 y:=-1.6 yaw:=0.0 > "$SIMLOG" 2>&1 < /dev/null &
SIM_PID=$!
for _ in $(seq 1 60); do
    grep -q "Successfully spawned entity" "$SIMLOG" 2>/dev/null && break
    sleep 2
done
grep -q "Successfully spawned entity" "$SIMLOG" || { say "spawn 失败"; exit 1; }
say "spawn 完成"

say "起导航栈"
setsid ros2 launch navbot_bringup nav_bringup.launch.py use_sim_time:=true use_rviz:=false \
    map:="$HOUSE_MAP" params_file:="$HOUSE_PARAMS" > "$NAVLOG" 2>&1 < /dev/null &
NAV_PID=$!
for _ in $(seq 1 50); do
    st=$(timeout 12 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
    case "$st" in *active*) break ;; esac
    sleep 3
done
say "导航栈就绪，注入初值"
timeout 90 python3 tools/verify/sim_fullchain/set_pose.py --map -1.0 -1.6 0.0 | tail -3

echo
echo "############ 扫描 cost_travel_multiplier ############"
for m in 1.0 5.0 10.0 20.0 50.0; do
    echo
    say "=== multiplier = $m ==="
    timeout 15 ros2 param set /planner_server GridBased.cost_travel_multiplier "$m" 2>&1 | tail -1
    sleep 1
    timeout 120 python3 tools/verify/path_select.py \
        --route-file tools/verify/routes_house.json --planners GridBased 2>&1 \
        | grep -E "^=== G|GridBased|最小净空" | grep -v "^  →" 
done

echo
say "扫描结束"
