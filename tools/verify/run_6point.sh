#!/bin/bash
# ============================================================================
# run_6point.sh —— 一条命令跑完房子图六点路线（仿真 → 导航 → 初值 → 巡检）
#
# 为什么要有这个脚本（2026-11-17）：
#   助手执行环境里 `setsid nohup ... &` 起的进程会在**本次 Bash 调用结束时被回收**
#   （实测：sleep 120 在下一个调用里已不存在，run_bg.sh 的日志 0 字节）。
#   所以「先起仿真、下条命令再起导航」这种分步做法在本环境不成立，
#   必须把整条链放进一个持续存活的进程里一次跑完。
#
# 用法：
#   bash tools/verify/run_6point.sh [单点超时秒数，默认 420]
#   结果：log/route_s37_rerun.json   日志：log/rerun_sim.log / rerun_nav.log
# ============================================================================
set -u

WS=/home/liang/navbot_ws
cd "$WS" || exit 1
# ★ source ROS 的 setup.bash 时必须关掉 set -u：
#   它的第 8 行就引用 $AMENT_TRACE_SETUP_FILES，严格模式下直接
#   「未绑定的变量」→ 脚本 124 ms 就死（这一版第一次跑就这么挂的）。
set +u
# shellcheck disable=SC1091
source tools/verify/sim_fullchain/env.sh
set -u

TIMEOUT="${1:-420}"
ROUTE="${2:-tools/verify/routes_house.json}"
OUTNAME="${3:-route_s37_rerun.json}"
LOG="$WS/log"
SIMLOG="$LOG/rerun_sim.log"
NAVLOG="$LOG/rerun_nav.log"
OUT="$LOG/$OUTNAME"
HOUSE_MAP="$WS/install/navbot_bringup/share/navbot_bringup/maps/navbot_house.yaml"
HOUSE_PARAMS="$WS/install/navbot_bringup/share/navbot_bringup/config/nav2_params_house.yaml"
HOUSE_WORLD="$WS/install/navbot_bringup/share/navbot_bringup/worlds/navbot_house.world"

say() { echo "[$(date +%H:%M:%S)] $*"; }

PIDS=()
cleanup() {
    say "收尾：结束仿真与导航进程"
    # ★ 不能写 `< <(expand_pids)`：trap 在脚本退出阶段执行时 /dev/fd 已经拆了，
    #   会报 “/dev/fd/63: 没有那个文件或目录”（不影响 kill，但日志难看）。
    #   空数组在 set -u 下的安全展开写法：
    for p in ${PIDS[@]+"${PIDS[@]}"}; do
        kill -TERM "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null
    done
    sleep 3
    for p in ${PIDS[@]+"${PIDS[@]}"}; do
        kill -KILL "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null
    done
}
trap cleanup EXIT

# 清掉可能残留的上一次会话（本环境一般没有，防手滑）
pkill -f "gzserver " 2>/dev/null
pkill -f "ros2 launch navbot" 2>/dev/null
sleep 1

: > "$SIMLOG"; : > "$NAVLOG"; rm -f "$OUT"

# ---------------------------------------------------------------- 1/4 仿真
say "=== 1/4 起仿真（navbot_house, 出生点 -1.0 -1.6, gui:=false）==="
setsid ros2 launch navbot_bringup sim_navbot.launch.py gui:=false \
    world:="$HOUSE_WORLD" x:=-1.0 y:=-1.6 yaw:=0.0 > "$SIMLOG" 2>&1 < /dev/null &
SIM_PID=$!
PIDS+=("$SIM_PID")

ok=0
for i in $(seq 1 90); do
    if grep -q "Successfully spawned entity" "$SIMLOG" 2>/dev/null; then ok=1; break; fi
    sleep 2
done
if [ "$ok" != "1" ]; then
    say "!! 车没 spawn 成功（等了 180 s）。sim.log 末尾："
    tail -25 "$SIMLOG"
    exit 1
fi
say "车已 spawn 成功，世界加载完成"
sleep 4

# ------------------------------------------------------- 1.5/4 时钟闸门
# ★ 2026-11-17 血泪闸门：/clock 收不到时，Nav2 全栈时间恒为 0，
#   goal_checker 永远判「25 s 只走了 0 m」→ Failed to make progress 死循环，
#   而 gz stats 显示 RTF=1.00、车也能被 cmd_vel 推着动一小段 —— 现象极像「DWB 调参没调好」，
#   实际是时钟没通。不加这道闸门就会白跑 10 分钟。
#   已排除的同类现象：残留 gzserver 多实例（ROS 图里两个 /gazebo）会让 /clock 静默消失。
say "校验 /clock（Nav2 全靠它）"
CLOCK_T=$(timeout 30 python3 -c '
import rclpy, time
from rclpy.parameter import Parameter
rclpy.init()
n = rclpy.create_node("clock_check",
                      parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
t0 = time.time(); v = 0.0
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
    v = max(v, n.get_clock().now().nanoseconds / 1e9)
print("%.3f" % v)
n.destroy_node(); rclpy.shutdown()
' 2>/dev/null | tail -1)
CLOCK_T="${CLOCK_T:-0}"
if awk -v t="$CLOCK_T" 'BEGIN{exit !(t > 1.0)}'; then
    say "/clock 正常（读到 ${CLOCK_T}s）"
else
    say "!! /clock 读不到（${CLOCK_T}s）—— 多半是残留 gzserver 顶掉了 gazebo_ros_init"
    say "   查：ros2 node list | grep gazebo 应当只有一个；先清干净再跑"
    exit 1
fi

# ---------------------------------------------------------------- 2/4 导航栈
say "=== 2/4 起 Nav2（house 地图 + house 参数，use_rviz:=false）==="
setsid ros2 launch navbot_bringup nav_bringup.launch.py use_sim_time:=true use_rviz:=false \
    map:="$HOUSE_MAP" params_file:="$HOUSE_PARAMS" > "$NAVLOG" 2>&1 < /dev/null &
NAV_PID=$!
PIDS+=("$NAV_PID")

say "等 bt_navigator 进入 active（最多 150 s）..."
active=0
for i in $(seq 1 50); do
    st=$(timeout 12 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
    case "$st" in
        *active*) active=1; break ;;
    esac
    sleep 3
done
if [ "$active" != "1" ]; then
    say "!! bt_navigator 没 active。nav.log 末尾："
    tail -30 "$NAVLOG"
    exit 1
fi
say "bt_navigator 已 active"
sleep 3

# ---------------------------------------------------------------- 3/4 初值
say "=== 3/4 注入 AMCL 初值（map -1.0 -1.6 yaw 0）==="
timeout 90 python3 tools/verify/sim_fullchain/set_pose.py --map -1.0 -1.6 0.0
SP_RC=$?
say "set_pose 退出码 $SP_RC（0=定位健康）"
if [ "$SP_RC" != "0" ]; then
    say "!! 初值不健康，继续跑只会在错的地图上打转 —— 中止"
    exit 1
fi
sleep 2

# ---------------------------------------------------------------- 4/4 路线
say "=== 4/4 跑路线 ${ROUTE}（单点超时 ${TIMEOUT}s）==="
# ★ -u 关掉 python 的块缓冲：不加的话逐点结果会攒到 4 KB 才吐，
#   在外面只能看到「4/4 开始」然后一片空白，无法判断跑到第几个点。
python3 -u tools/verify/nav_route.py \
    --route-file "$ROUTE" \
    --timeout "$TIMEOUT" \
    --tag s38_rerun \
    --json "$OUT"
RC=$?
say "=== 路线结束 rc=$RC ==="
if [ -f "$OUT" ]; then
    python3 - "$OUT" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
print('PASS %d / %d' % (d['pass'], d['total']))
PY
fi
exit "$RC"
