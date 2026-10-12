#!/usr/bin/env bash
# ===========================================================================
# 驾验：给 /cmd_vel 发速度指令，看 /odom 是否真的增长
#   —— 这是「差速驱动插件到底接没接上」的唯一硬判据
# 用法：bash .check/test_drive.sh [sandbox|user]
# 前置：另一个终端已经 ros2 launch navbot_bringup sim_navbot.launch.py
# ===========================================================================
set -o pipefail
WS="/home/liang/桌面/具身智能学习路线/bridge/ros_study"
PROFILE="${1:-sandbox}"

if [ "$PROFILE" = "sandbox" ]; then
    export PATH="$(echo "$PATH" | tr ':' '\n' \
        | grep -v 'workbuddy/binaries/python' | grep -v '^$' | paste -sd: -)"
    export PYTHONPATH="$(echo "$PYTHONPATH" 2>/dev/null | tr ':' '\n' \
        | grep -v 'WorkBuddy' | grep -v '^$' | paste -sd: -)"
fi

cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null
source install/setup.bash

echo "############ 差速驱动通路测试 ############"

# 读当前 odom 的 x 位移
read_x () {
    timeout 5 ros2 topic echo /odom --once 2>/dev/null \
        | grep -A2 'position:' | sed -n '2p' | tr -d ' ' | tr -d 'x:'
}

read_yaw () {
    timeout 5 ros2 topic echo /odom --once 2>/dev/null \
        | grep -A2 'orientation:' | sed -n '3p' | tr -d ' ' | tr -d 'z:'
}

echo "-- 先确认 /odom 活着 --"
timeout 6 ros2 topic hz /odom 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1

echo
echo "-- ① 前进 0.15 m/s，持续 4 秒 --"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 20 >/dev/null 2>&1 &
PUB=$!
sleep 4
kill -9 $PUB 2>/dev/null
sleep 1
X1=$(read_x); echo "   前进后 odom.x = $X1"

echo
echo "-- ② 原地左转 0.5 rad/s，持续 3 秒 --"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 20 >/dev/null 2>&1 &
PUB2=$!
sleep 3
kill -9 $PUB2 2>/dev/null
sleep 1
YAW=$(read_yaw); echo "   旋转后 odom.yaw(z分量) = $YAW"

echo
echo "========== 判据 =========="
if [ -n "$X1" ] && [ "$X1" != "0.0" ] && [ "$X1" != "0" ]; then
    echo "  [PASS] 线速度生效：odom.x = $X1 (应明显 > 0，前进 4s@0.15 约 0.5~0.6)"
else
    echo "  [FAIL] odom.x 仍是 $X1 —— cmd_vel 没接上，检查 diff_drive 插件的 commandTopic"
fi
if [ -n "$YAW" ] && [ "$YAW" != "0.0" ]; then
    echo "  [PASS] 角速度生效：yaw = $YAW (转过一圈会绕回 0，小角度应 != 0)"
else
    echo "  [FAIL] yaw 无变化 —— 角速度链路不通"
fi
echo "############ 结束 ############"
