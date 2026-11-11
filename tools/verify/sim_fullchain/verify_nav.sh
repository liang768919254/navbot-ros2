#!/usr/bin/env bash
# ============================================================================
# test_nav.sh —— Nav2 导航验证
# ============================================================================
# ★★ 两条关键教训（都踩过）：
#
#   1) /initialpose 不能用 `ros2 topic pub --once` 发。
#      --once 发完进程立即退出，DDS 投递未完成，AMCL 收不到
#      （表现为 AMCL 一直刷 "Please set the initial pose"）。
#      必须用 set_pose.py 持续发布几秒。
#
#   2) 必须「先给初值」再等节点 active。
#      lifecycle_manager 按序激活，planner_server 的 global_costmap 在
#      activate 阶段要 Checking transform (map->odom)，此时如果 AMCL 还没
#      初值 → map->odom 不存在 → 卡死 → 后面的 bt_navigator 永远轮不到。
#
# 判据：
#   1. 核心节点 active
#   2. TF 链 map→odom→base_footprint 闭合
#   3. NavigateToPose SUCCEEDED
#   4. 终点误差 < 0.25m
# ============================================================================
set +u
RUN_DIR="/home/liang/navbot_ws/tools/verify/sim_fullchain"
MAP_YAML="${1:-/home/liang/navbot_ws/src/navbot_bringup/maps/navbot_room.yaml}"
GOAL_X="${2:-2.0}"
GOAL_Y="${3:--1.8}"
cd "$RUN_DIR" || exit 1

# ★ REUSE_SIM=1：复用外部已经起好的仿真（典型场景：你自己终端里跑的 GUI 版）。
#   此时既不能 stop_bg.sh all（它会 pkill 掉你的 gzserver），
#   也不能 run_bg.sh sim（11345 端口已被占用）。
REUSE_SIM="${REUSE_SIM:-0}"
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1
    echo "==================== ① 复用已有仿真（REUSE_SIM=1）===================="
else
    bash stop_bg.sh all >/dev/null 2>&1
    sleep 2
    echo "==================== ① 启动仿真 ===================="
    bash run_bg.sh sim ros2 launch navbot_bringup sim_navbot.launch.py gui:=false
    sleep 14
fi
source "$RUN_DIR/env.sh"
for i in $(seq 1 20); do
    if timeout 3 ros2 topic echo /scan --once --field header >/dev/null 2>&1; then
        echo "  /scan OK"; break
    fi
    sleep 2
done

echo "==================== ② 启动 Nav2 ===================="
bash run_bg.sh nav ros2 launch navbot_bringup nav_bringup.launch.py \
    use_rviz:=false use_sim_time:=true map:="$MAP_YAML"
sleep 14

echo "==================== ③ 注入 AMCL 初值（--auto，带验证）===================="
# ★★ S34 改：「不能写死 0 0 1.0 0.0」这个判断是对的，但改成读 /odom 只解决了
#   一半问题 —— 「map 与 odom 重合」的前提只在冷启动成立，跑过导航后偏 1.4 m。
#   set_pose.py --auto 复合 map->odom 算出真正的 map 坐标，
#   并在注入后验证 map->odom 是平面变换，不行就重试 3 次，仍不行返回非 0。
# ★★★ S34 修：`if cmd | sed` 判断的是 sed 的返回码，不是 python 的。
#   失败返回 1 时 sed 仍返回 0 → 永远"通过"，错误被静默吞掉。
#   改为先取输出与返回码，再判断。
SP_OUT=$(timeout 150 python3 set_pose.py --auto 2>&1)
SP_RC=$?
echo "$SP_OUT" | sed 's/^/  /'
if [ "$SP_RC" -ne 0 ]; then
    echo "  !!! 初值注入失败或定位不健康 (rc=$SP_RC)，终止"
    exit 1
fi
echo "  OK 初值注入 + 验证 通过"

echo "---- 等核心节点 active ----"
ALL_ACTIVE=0
for i in $(seq 1 20); do
    sleep 3
    A=$(timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null)
    P=$(timeout 6 ros2 lifecycle get /planner_server 2>/dev/null)
    C=$(timeout 6 ros2 lifecycle get /controller_server 2>/dev/null)
    if echo "$A" | grep -q active && echo "$P" | grep -q active && echo "$C" | grep -q active; then
        ALL_ACTIVE=1; echo "  [$((i*3))s] 全部 active"; break
    fi
    echo "  [$((i*3))s] bt=${A:-?} planner=${P:-?} ctrl=${C:-?}"
done
[ "$ALL_ACTIVE" -eq 1 ] || { echo "!!!! 未全部 active !!!!"; exit 1; }

echo "---- 全部节点状态 ----"
for n in amcl map_server planner_server controller_server bt_navigator behavior_server waypoint_follower velocity_smoother; do
    printf "  %-20s " "$n"
    timeout 5 ros2 lifecycle get "/$n" 2>/dev/null || echo "N/A"
done

echo "---- TF 链 ----"
echo -n "  map->odom  : "; timeout 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 "Translation"
echo -n "  odom->base : "; timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -m1 "Translation"

echo "==================== ④ 导航到 ($GOAL_X, $GOAL_Y) ===================="
RESULT=$(timeout 240 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback 2>&1)
echo "$RESULT" | grep -E "Goal accepted|Goal finished|SUCCEEDED|FAILED|ABORTED|CANCELED" | tail -3

echo "==================== ⑤ 终点核验 ===================="
sleep 2
/home/liang/.workbuddy/binaries/python/envs/default/bin/python - "$GOAL_X" "$GOAL_Y" <<'PY'
import re, subprocess, sys
gx, gy = float(sys.argv[1]), float(sys.argv[2])
out = subprocess.run(['bash','-lc',
  'source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh; timeout 8 ros2 topic echo /amcl_pose --once --field pose.pose.position'],
  capture_output=True, text=True).stdout
mx = re.search(r'x:\s*([-\d.e]+)', out); my = re.search(r'y:\s*([-\d.e]+)', out)
if mx and my:
    x, y = float(mx.group(1)), float(my.group(1))
    err = ((x-gx)**2 + (y-gy)**2) ** 0.5
    print(f"  AMCL 终点 ({x:.3f}, {y:.3f})   目标 ({gx}, {gy})   误差 {err:.3f} m")
    print(f"  判据 误差<0.25m : {'PASS' if err < 0.25 else 'FAIL'}")
else:
    print("  取不到位姿")
PY

echo "==================== 收尾 ===================="
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1   # 保留你的 Gazebo
else
    bash stop_bg.sh all >/dev/null 2>&1
fi
echo "test_nav.sh 结束"
