#!/usr/bin/env bash
# ===========================================================================
# 阶段B 完整自检：起 Gazebo → spawn 车 → 发速度指令 → 看 odom 是否增长
#
# ★★ 为什么必须「启动」和「测试」在同一个脚本里跑？★★
#    ROS2 的节点发现走 DDS 组播，跨 Bash 调用（不同网络上下文）互相看不见。
#    分两次跑会出现「明明 spawn 成功但 topic 全是 DEAD」的假故障。
#    所以本脚本自包含：自己起、自己测、自己清理。
#
# 用法：bash .check/test_stageB.sh [gui] [profile]
#   gui=true|false   是否起 Gazebo 窗口
#   profile=sandbox  助手沙箱内跑（会清理 WorkBuddy 的 python 污染源）
#           =user    你在自己的 Ubuntu 终端跑（不做任何清理）
# ===========================================================================
set -o pipefail
WS="/home/liang/桌面/具身智能学习路线/bridge/ros_study"
GUI="${1:-true}"
PROFILE="${2:-sandbox}"

if [ "$PROFILE" = "sandbox" ]; then
    # ROS 脚本的 #!/usr/bin/env python3 会选到 managed Python 3.13，
    # 而 ROS 的 numpy 是 3.10 的 C 扩展 → 必须从 PATH 里摘掉它
    export PATH="$(echo "$PATH" | tr ':' '\n' \
        | grep -v 'workbuddy/binaries/python' | grep -v '^$' | paste -sd: -)"
    export PYTHONPATH="$(echo "$PYTHONPATH" 2>/dev/null | tr ':' '\n' \
        | grep -v 'WorkBuddy' | grep -v '^$' | paste -sd: -)"
fi

cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || source /usr/share/gazebo-11/setup.sh 2>/dev/null
source install/setup.bash

cleanup () {
    kill -INT $LP 2>/dev/null
    sleep 2
    pkill -9 -f 'gzserver|gzclient' 2>/dev/null
    pkill -9 -f spawn_entity 2>/dev/null
    pkill -9 -f 'robot_state_publisher' 2>/dev/null
}
trap cleanup EXIT

echo "########## 阶段B 完整自检  gui=$GUI ##########"
echo "python3 = $(python3 -c 'import sys;print(sys.executable, sys.version.split()[0])' 2>&1)"
python3 -c "import numpy" 2>/dev/null && echo "numpy OK" || echo "[FATAL] numpy 缺失"

pkill -9 -f 'gzserver|gzclient' 2>/dev/null; pkill -9 -f spawn_entity 2>/dev/null
sleep 2

ros2 launch navbot_bringup sim_navbot.launch.py gui:=$GUI \
    > "$WS/.check/stageB_launch.log" 2>&1 &
LP=$!

READY=0
for i in $(seq 1 30); do
    sleep 2
    grep -qE 'Spawn status: SpawnEntity: Successfully' "$WS/.check/stageB_launch.log" 2>/dev/null \
        && { echo "[OK] 车已生成 (${i}0 秒内)"; READY=1; break; }
    kill -0 $LP 2>/dev/null || { echo "[FAIL] launch 提前退出"; tail -20 "$WS/.check/stageB_launch.log"; exit 1; }
done
[ "$READY" -eq 0 ] && echo "[WARN] 未在预期时间内 spawn，继续尝试检查话题"
sleep 4

echo
echo "===== ① 话题清单 ====="
ros2 topic list 2>/dev/null | grep -vE '^/(parameter_events|rosout)$' | sed 's/^/  /'

echo
echo "===== ② 核心话题速率 ====="
for TP in /scan /odom /joint_states /tf /clock; do
    HZ=$(timeout 6 ros2 topic hz $TP 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
    [ -n "$HZ" ] && echo "  [OK]   $TP  $HZ" || echo "  [DEAD] $TP"
done

echo
echo "===== ③ 差速驱动通路（真·验收判据）====="
read_odom () {   # $1: x | yaw_x | yaw_z   —— 精确指定取哪个字段
    timeout 6 ros2 topic echo /odom --once 2>/dev/null > "$WS/.check/odom1.txt"
    python3 - "$1" <<'PY'
import sys, re
try:
    txt = open('/home/liang/桌面/具身智能学习路线/bridge/ros_study/.check/odom1.txt').read()
except Exception:
    print('0'); sys.exit()
want = sys.argv[1]
# ★ 坑：直接正则 ^z: 会先命中 position.z（高度，恒为 0），不是朝向
#   必须先把 pose.pose 切成 position / orientation 两段再取值
m_pose = re.search(r'^\s*pose:\s*$', txt, re.M)
body = txt[m_pose.end():] if m_pose else txt
sec = {}
for name in ('position', 'orientation'):
    mm = re.search(r'^\s*' + name + r':\s*$', body, re.M)
    if mm:
        chunk = body[mm.end():]
        for axis in 'xyz' + ('zw' if name == 'orientation' else ''):
            pass
        for axis in 'xyzw':
            v = re.search(r'^\s*' + axis + r':\s*([-\d.eE+]+)', chunk, re.M)
            if v:
                sec[name + '_' + axis] = float(v.group(1))
key = {'x': 'position_x', 'yaw_z': 'orientation_z', 'yaw_w': 'orientation_w',
       'qx': 'orientation_x', 'qy': 'orientation_y'}[want]
print(sec.get(key, 0))
PY
}
stop_robot () {   # 发零速，避免惯性滑行污染读数
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0}, angular: {z: 0.0}}" -1 >/dev/null 2>&1
    sleep 1.5
}

X0=$(read_odom x); echo "  起始 odom.x = $X0"
echo "  → 前进 0.15 m/s × 3s，随后发零速停稳"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 20 >/dev/null 2>&1 &
P1=$!; sleep 3; kill -9 $P1 2>/dev/null; stop_robot
X1=$(read_odom x); echo "  前进后 odom.x = $X1"

Z0=$(read_odom yaw_z); W0=$(read_odom yaw_w)
echo "  → 原地转 0.5 rad/s × 2s，随后停稳"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 20 >/dev/null 2>&1 &
P2=$!; sleep 2; kill -9 $P2 2>/dev/null; stop_robot
Z1=$(read_odom yaw_z); echo "  起始 orientation.z = $Z0  →  现在 = $Z1"

echo
echo "  === 判据 ==="
python3 -c "
import math
x0,x1,z0,z1 = float('$X0'), float('$X1'), float('$Z0'), float('$Z1')
d = x1-x0
print(f' 线位移 Δx = {d:.3f} m   (期望 ≈0.45~0.75，滑行会略多)')
print(' [PASS] 线速度链路通 (cmd_vel → diff_drive → 轮子)' if d > 0.2
      else ' [FAIL] cmd_vel→轮子未接通')
# 用四元数 z 分量反推偏航角： yaw = 2*asin(clamp(z))
try:    yaw = 2*math.asin(max(-1.0, min(1.0, z1)))
except: yaw = 0.0
print(f' 朝向 z 分量 = {z1:.4f}  →  推算偏航 ≈ {math.degrees(yaw):.1f}°')
print(' [PASS] 角速度链路通' if abs(z1) > 0.05
      else ' [FAIL] 转向链路不通')
"

echo
echo "########## 阶段B 自检结束 ##########"
