#!/bin/bash
# ============================================================================
# check_live.sh —— live.sh 跑起来之后，一条命令体检
# ============================================================================
# 用法（另开一个终端）：
#   bash check_live.sh          # 建图模式的检查项
#   bash check_live.sh nav      # 导航模式的检查项
#
# 输出全部是「实测数值 + PASS/FAIL」，不用肉眼判断。
# ============================================================================
set +u
HERE=/home/liang/navbot_ws/tools/verify/sim_fullchain
MODE="${1:-slam}"
source "$HERE/env.sh"

PASS=0; FAIL=0; WARN=0
ok()   { echo "  [PASS] $*"; PASS=$((PASS+1)); }
ng()   { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $*"; WARN=$((WARN+1)); }

echo "=============================================================="
echo " navbot 实时联调体检 —— 模式: $MODE"
echo "=============================================================="

echo ""
echo "---- ① 进程 / 节点 ----"
# ★★ 不要用 pgrep 判进程！
#   如果这个体检脚本是在【助手沙箱里】跑的，pgrep 只能看到沙箱 PID namespace
#   里的进程，看不到小梁自己终端起的 gzserver/rviz2 → 全部误报 FAIL，
#   而 ROS 图明明能连上（节点列表里有 /gazebo、/rviz）。
#   2026-11-10 实测踩过：4 个进程项全 FAIL，但话题/TF 全 PASS，自相矛盾。
#   ⇒ 统一改成查 ROS 图：节点名存在 = 该组件真的活着，且跨 namespace 有效。
NODES=$(timeout 10 ros2 node list 2>/dev/null)
have_node() { echo "$NODES" | grep -qx "$1"; }

if have_node "/gazebo"; then ok "Gazebo 节点在线 (/gazebo)"
else ng "ROS 图里没有 /gazebo —— 仿真没起"; fi

if have_node "/diff_drive"; then ok "差速驱动在线 (/diff_drive)"
else ng "/diff_drive 不在 —— URDF 插件没加载"; fi

if [ "$MODE" = "nav" ]; then
    have_node "/amcl" && ok "AMCL 在线" || ng "AMCL 不在"
    have_node "/planner_server" && ok "planner_server 在线" || ng "planner_server 不在"
else
    have_node "/slam_toolbox" && ok "slam_toolbox 在线" || ng "slam_toolbox 不在"
fi

# RViz 不是 ROS 节点（除非在跑导航插件），只能靠日志判断
if [ -f "$HERE/live_rviz.log" ]; then
    ok "RViz 已启动（live_rviz.log 存在）"
else
    warn "没找到 live_rviz.log —— RViz 可能是你手动另起的，或没起"
fi

echo "  (ROS 图里共 $(echo "$NODES" | grep -c . ) 个节点)"

echo ""
echo "---- ② 激光 ----"
HZ=$(timeout 8 ros2 topic hz /scan --window 20 2>/dev/null | grep -m1 "average rate" | grep -oE "[0-9.]+")
if [ -n "$HZ" ]; then
    if awk -v h="$HZ" 'BEGIN{exit !(h>=7 && h<=12)}'; then
        ok "/scan = ${HZ} Hz（期望 ~10Hz）"
    else
        warn "/scan = ${HZ} Hz（偏离 10Hz，可能 RTF 掉速）"
    fi
else
    ng "/scan 无数据"
fi
# ★ 用 `2>/dev/null` 把 timeout/grep 的「断开的管道」噪声吞掉：
#   `ros2 topic echo ... | head` 里 head 先退出，上游 grep 写管道会报 EPIPE，
#   输出一行 "grep: 写入错误: 断开的管道"，看着像出错其实无害。
RNG=$( { timeout 6 ros2 topic echo /scan --once --field ranges 2>/dev/null \
         | tr ',' '\n' | grep -oE "[0-9]+\.[0-9]+" | sort -n | head -1; } 2>/dev/null )
[ -n "$RNG" ] && echo "  最近障碍距离 ≈ ${RNG} m"

echo ""
echo "---- ③ TF 链 ----"
T1=$(timeout 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 "Translation")
if [ -n "$T1" ]; then ok "map -> odom 存在"; echo "        $T1"
else ng "map -> odom 不存在（SLAM 没发布 或 Fixed Frame 问题）"; fi

T2=$(timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -m1 "Translation")
if [ -n "$T2" ]; then ok "odom -> base_footprint 存在"; echo "        $T2"
else ng "odom -> base_footprint 不存在（diff_drive 插件问题）"; fi

echo ""
echo "---- ④ 地图 ----"
# ★ 注意：`ros2 topic echo` 超时或话题不存在时也会输出空行，不能只用 -n 判空。
#   必须确认 width/height 真的解析出数字。
MAPINFO=$(timeout 10 ros2 topic echo /map --once --field info 2>/dev/null)
W=$(echo "$MAPINFO" | awk '/width:/{print $2; exit}')
H=$(echo "$MAPINFO" | awk '/height:/{print $2; exit}')
if [ -n "$W" ] && [ -n "$H" ] && [ "$W" -gt 0 ] 2>/dev/null && [ "$H" -gt 0 ] 2>/dev/null; then
    ok "/map 在发布 ${W} x ${H}"
    echo "        6m x 5m 房间 @ 0.05m/格 满分 ≈ 120 x 100"
    if [ "$W" -ge 110 ] && [ "$H" -ge 92 ]; then
        ok "覆盖面积达标"
    else
        warn "覆盖还没长满（${W}x${H}），继续开让它扫"
    fi
else
    ng "/map 无数据或尺寸读不出"
fi

echo ""
echo "---- ⑤ RViz 视图参数（Scale 是否真生效）----"
if [ -f "$HERE/live_rviz.log" ]; then
    # ★ indexed_8bit_image 的 GLSL link 报错是已知无害现象：
    #   地图图层在 llvmpipe / 部分 Mesa 驱动下走 glsl120 路径的编译告警，
    #   地图照样能画出来。README「环境相关的非致命现象」里有记录。
    #   所以要把它排除掉再判，否则永远 WARN 一条，久了就没人看。
    REALERR=$(grep -iE "error|failed" "$HERE/live_rviz.log" \
              | grep -v "indexed_8bit_image" \
              | grep -v "GLSL link result" | head -3)
    if [ -n "$REALERR" ]; then
        warn "RViz 日志有真实 error："
        echo "$REALERR" | sed 's/^/        /'
    else
        if grep -q "indexed_8bit_image" "$HERE/live_rviz.log"; then
            ok "RViz 无真实 error（indexed_8bit_image 着色器告警属已知无害项）"
        else
            ok "RViz 日志无 error"
        fi
    fi
else
    warn "还没有 live_rviz.log（RViz 没通过 live.sh 起？）"
fi
# ★ 抓 TopDownOrtho 的 Scale，不能抓 "Marker Scale" —— 那个是 TF 显示的箭头缩放，
#   跟视口缩放完全两回事，用宽松 grep 会抓到 0.5 造成误判。
RCFG=/home/liang/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz/navbot_slam.rviz
[ "$MODE" = "nav" ] && RCFG=/home/liang/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz/navbot_nav.rviz
SCALE=$(awk '/Class: rviz_default_plugins\/TopDownOrtho/{f=1} f&&/^ *Scale:/{print $2; exit}' "$RCFG")
echo "  TopDownOrtho Scale = ${SCALE:-读不出} 像素/米"
if [ "$SCALE" = "150" ]; then
    ok "Scale=150：窗口宽 1500px → 视野 10m，6m 房间应该铺满大半窗口"
else
    warn "Scale=${SCALE}，不是 150（地图会显得偏小或偏大）"
fi
echo "  ★ 仍要眼睛确认一次：地图应铺满窗口，而不是缩在中间一小块。"

if [ "$MODE" = "nav" ]; then
echo ""
echo "---- ⑥ Nav2 节点 ----"
for n in amcl map_server planner_server controller_server bt_navigator; do
    S=$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null)
    case "$S" in
        *active*) ok "$n active" ;;
        "")       ng "$n 不存在" ;;
        *)        ng "$n = $S" ;;
    esac
done
echo ""
echo "---- ⑦ 代价地图 ----"
for t in /global_costmap/costmap /local_costmap/costmap; do
    if timeout 6 ros2 topic echo "$t" --once --field info.width >/dev/null 2>&1; then
        ok "$t 在发布"
    else
        ng "$t 无数据"
    fi
done
echo ""
echo "---- ⑧ AMCL 定位 ----"
AP=$(timeout 8 ros2 topic echo /amcl_pose --once --field pose.pose.position 2>/dev/null)
if [ -n "$AP" ]; then
    ok "AMCL 有位姿"; echo "$AP" | sed 's/^/        /'
    CAR=$(echo "$AP" | grep -E "^\s*[xy]:" | tr -d ' ' | tr '\n' ' ')
    echo ""
    echo "  ★ 判断定位对不对：在 RViz 里看红色激光点是否贴在黑墙上。"
    echo "    激光穿墙 / 整体偏移 = 初值给错了，点 '2D Pose Estimate' 重给。"
else
    ng "AMCL 无位姿（初值还没注入？）"
fi
fi

echo ""
echo "=============================================================="
echo " 合计: PASS=$PASS  FAIL=$FAIL  WARN=$WARN"
[ "$FAIL" -eq 0 ] && echo " 结论: 全通" || echo " 结论: 有 $FAIL 项没过，先修 FAIL 再看 WARN"
echo "=============================================================="
