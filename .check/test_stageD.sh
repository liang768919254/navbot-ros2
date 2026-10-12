#!/usr/bin/env bash
# ===========================================================================
# 阶段D 自检：Nav2 自主导航
#   起 Gazebo → 用阶段C 建的新图起 Nav2 → 自动发初始位姿 → 连发多个目标点
#
# ★★ 与 SLAM 互斥：AMCL 和 slam_toolbox 都会发 map→odom，必须一个跑完再跑另一个
# 用法：bash .check/test_stageD.sh [gui] [profile]
# ===========================================================================
set -o pipefail
WS="/home/liang/桌面/具身智能学习路线/bridge/ros_study"
GUI="${1:-false}"
PROFILE="${2:-sandbox}"

if [ "$PROFILE" = "sandbox" ]; then
    export PATH="$(echo "$PATH" | tr ':' '\n' \
        | grep -v 'workbuddy/binaries/python' | grep -v '^$' | paste -sd: -)"
    export PYTHONPATH="$(echo "$PYTHONPATH" 2>/dev/null | tr ':' '\n' \
        | grep -v 'WorkBuddy' | grep -v '^$' | paste -sd: -)"
    # ★ 沙箱 /dev/shm 仅 10M，FastDDS SHM 传输创建失败 → lifecycle 激活丢失。
    #   强制走 UDP 可绕开。用户自己的终端 /dev/shm 正常时不需要这行。
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/.check/fastdds_udp_only.xml"
fi

cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null
source install/setup.bash

# ★★ 地图必须放纯 ASCII 路径：map_server 是 C++ 实现，读不了中文路径
#    （yaml-cpp 打开含 UTF-8 中文目录名的文件直接报 bad file）。
#    Python 侧工具（map_saver_cli 等）不受影响，所以「存图成功、读图失败」。
MAP_YAML="/home/liang/navbot_maps/navbot_room_new.yaml"
[ -f "$MAP_YAML" ] || { echo "[FAIL] 没找到地图，先跑 test_stageC.sh 并把图拷到英文路径"; exit 1; }

cleanup () {
    kill -INT $SIM_PID 2>/dev/null; kill -INT $NAV_PID 2>/dev/null
    sleep 2
    pkill -9 -f 'gzserver|gzclient' 2>/dev/null
    pkill -9 -f 'nav2\|bt_navigator\|amcl\|planner_server\|controller_server'
    pkill -9 -f robot_state_publisher 2>/dev/null
}
trap cleanup EXIT

echo "########## 阶段D：Nav2 导航自检  gui=$GUI ##########"
# 杀干净残留（尤其 SLAM 遗留）
pkill -9 -f 'gzserver|gzclient' 2>/dev/null
pkill -9 -f async_slam_toolbox 2>/dev/null
pkill -9 -f 'bt_navigator|amcl|planner_server|controller_server' 2>/dev/null
sleep 3

# ---------- ① 起仿真 ----------
echo "-- ① 起 Gazebo..."
ros2 launch navbot_bringup sim_navbot.launch.py gui:=$GUI \
    > "$WS/.check/stageD_sim.log" 2>&1 &
SIM_PID=$!
for i in $(seq 1 30); do
    sleep 2
    grep -qE 'Spawn status: SpawnEntity: Successfully' "$WS/.check/stageD_sim.log" 2>/dev/null \
        && { echo "   [OK] 车已生成"; break; }
done
sleep 4

# ---------- ② 起 Nav2（加载新地图，无 RViz）----------
echo "-- ② 起 Nav2（用本次建的新图）..."
ros2 launch navbot_bringup nav_bringup.launch.py \
    use_sim_time:=true use_rviz:=$GUI map:="$MAP_YAML" \
    > "$WS/.check/stageD_nav.log" 2>&1 &
NAV_PID=$!

# ---------------------------------------------------------------------------
# ★★ 顺序不能反：AMCL 必须先拿到初始位姿才会发布 map→odom。
#    而 planner_server 的 global_costmap 又死等这个 TF ——
#    如果等到 bt_navigator active 才发 initialpose，就会互相卡死：
#      planner_server 等 TF → bt_navigator 等 planner_server → 永远不 active
#    所以：先发初始位姿，再等激活。
# ---------------------------------------------------------------------------
echo "-- ②b 先给 AMCL 初始位姿（等价于 RViz 的 2D Pose Estimate）..."
sleep 8
python3 - <<'PY'
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
n = rclpy.create_node('seed_initialpose')
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
m = PoseWithCovarianceStamped()
m.header.frame_id = 'map'
m.header.stamp = n.get_clock().now().to_msg()
m.pose.pose.orientation.w = 1.0          # 车 spawn 在 (0,0) 朝 +x
m.pose.covariance[0] = 0.25
m.pose.covariance[7] = 0.25
m.pose.covariance[35] = 0.068
for _ in range(8):
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.2)
print('    /initialpose 已发布')
n.destroy_node(); rclpy.shutdown()
PY
sleep 5

# 等生命周期节点全 active
READY=0
for i in $(seq 1 45); do
    sleep 2
    ST=$(timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -oE 'active|unconfigured|inactive' | head -1)
    if [ "$ST" = "active" ]; then
        echo "   [OK] bt_navigator 已 active (等待 $((i*2)) 秒)"
        READY=1
        break
    fi
done
if [ "$READY" -eq 0 ]; then
    echo "   [FAIL] bt_navigator 一直没 active，看 lifecycle 状态："
    for n in bt_navigator amcl map_server planner_server controller_server; do
        echo -n "     $n: "; timeout 5 ros2 lifecycle get /$n 2>/dev/null || echo "?"
    done
    echo "   日志尾部："; tail -30 "$WS/.check/stageD_nav.log"
    exit 1
fi
sleep 5

echo
echo "===== ③ Nav2 节点状态 ====="
for n in bt_navigator amcl map_server planner_server controller_server \
         behavior_server waypoint_follower lifecycle_manager_navigation; do
    ST=$(timeout 5 ros2 lifecycle get /$n 2>/dev/null | grep -oE 'active|unconfigured|inactive' | head -1)
    printf "  %-30s %s\n" "$n" "${ST:-?}"
done

echo
echo "===== ④ 开始连发目标点 ====="
python3 "$WS/.check/nav_runner.py" 2>&1 | sed 's/^/  /'
RC=${PIPESTATUS[0]}

echo
echo "===== ⑤ 日志里搜问题 ====="
grep -iE '\[ERROR\]|\[WARN\].*(timeout|fail)' "$WS/.check/stageD_nav.log" 2>/dev/null | tail -10 | sed 's/^/  /'

echo
echo "########## 阶段D 自检结束 (exit=$RC) ##########"
