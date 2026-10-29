#!/usr/bin/env bash
# navbot_ws/tools/check_sim.sh
#
# 仿真健康检查 —— W3 出口判据，也是 G1（仿真 5 点导航）的前置冒烟。
#
# 用法：
#   终端 1：ros2 launch navbot_bringup sim_navbot.launch.py
#   终端 2：~/navbot_ws/tools/check_sim.sh
#
# 退出码：0 = 全过，1 = 有 FAIL
#
# 判据依据：navbot.xacro 里 diff_drive update_rate=50、ray update_rate=10

set -u

ODOM_MIN=40;  ODOM_MAX=60
SCAN_MIN=7;   SCAN_MAX=13
HZ_TMO=${HZ_TMO:-14}

PASS=0; FAIL=0; SKIP=0
ok()   { printf '  [PASS] %-34s %s\n' "$1" "$2"; PASS=$((PASS+1)); }
ng()   { printf '  [FAIL] %-34s %s\n' "$1" "$2"; FAIL=$((FAIL+1)); }
skip() { printf '  [SKIP] %-34s %s\n' "$1" "$2"; SKIP=$((SKIP+1)); }

# 取最后一次 average rate（ros2 topic hz 是持续输出，靠 timeout 收尾）
rate_of() {
  timeout "$HZ_TMO" ros2 topic hz "$1" 2>/dev/null \
    | awk '/average rate/{r=$3} END{if (r!="") print r}'
}

in_range() {  # $1=值 $2=min $3=max
  awk -v v="$1" -v a="$2" -v b="$3" 'BEGIN{exit !(v+0>=a+0 && v+0<=b+0)}'
}

echo "=============================================="
echo " navbot 仿真健康检查"
echo "=============================================="
echo

echo "-- 0. Gazebo 端口（最常见的启动失败原因）--"
PORT_LISTEN=$(ss -tln 2>/dev/null | grep -c ":11345 ")
GZ_N=$(pgrep -c -x gzserver 2>/dev/null || echo 0)
if [ "$PORT_LISTEN" -gt 0 ] && [ "$GZ_N" -eq 0 ]; then
  ng "端口 11345" "被占但无 gzserver 进程 → 有残留，新 Gazebo 起不来"
  echo "         修法：sudo fuser -k 11345/tcp   或  重启终端/机器"
elif [ "$PORT_LISTEN" -gt 0 ]; then
  ok "端口 11345" "监听中，gzserver ${GZ_N} 个"
else
  ng "端口 11345" "无监听 → Gazebo 没起来（或已退出）"
fi
echo

echo "-- 1. 必备话题 --"
TOPICS=$(timeout 10 ros2 topic list 2>/dev/null)
if [ -z "$TOPICS" ]; then
  echo "  [FAIL] ros2 topic list 为空 —— 仿真没起，或没 source install/setup.bash"
  echo "         先执行：source /opt/ros/humble/setup.bash && source ~/navbot_ws/install/setup.bash"
  exit 1
fi
for t in /clock /odom /scan /tf /tf_static /joint_states /cmd_vel; do
  if printf '%s\n' "$TOPICS" | grep -qx "$t"; then
    ok "$t" "存在"
  else
    ng "$t" "缺失"
  fi
done
echo

echo "-- 2. 发布者 --"
for t in /odom /scan; do
  n=$(timeout 10 ros2 topic info "$t" 2>/dev/null | awk -F': *' '/Publisher count/{print $2}')
  n=${n:-0}
  if [ "$n" -ge 1 ] 2>/dev/null; then
    ok "$t publisher" "count=$n"
  else
    ng "$t publisher" "count=$n（插件没加载）"
  fi
done
echo

echo "-- 3. 频率（每项最多 ${HZ_TMO}s）--"
ODOM_HZ=$(rate_of /odom)
if [ -z "$ODOM_HZ" ]; then
  ng "/odom 频率" "无数据（仿真时钟可能在走但插件没发）"
elif in_range "$ODOM_HZ" "$ODOM_MIN" "$ODOM_MAX"; then
  ok "/odom 频率" "${ODOM_HZ} Hz"
else
  ng "/odom 频率" "${ODOM_HZ} Hz（期望 ${ODOM_MIN}~${ODOM_MAX}）"
fi

SCAN_HZ=$(rate_of /scan)
if [ -z "$SCAN_HZ" ]; then
  ng "/scan 频率" "无数据（★ 先看是不是 gui:=false 起的）"
elif in_range "$SCAN_HZ" "$SCAN_MIN" "$SCAN_MAX"; then
  ok "/scan 频率" "${SCAN_HZ} Hz"
else
  ng "/scan 频率" "${SCAN_HZ} Hz（期望 ${SCAN_MIN}~${SCAN_MAX}）"
fi
echo

echo "-- 4. TF 链（odom -> base_footprint）--"
TF_OUT=$(timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1)
if printf '%s' "$TF_OUT" | grep -q "Translation"; then
  ok "odom -> base_footprint" "可查询"
else
  ng "odom -> base_footprint" "查不到（diff_drive 的 publish_odom_tf 是否为 true？）"
fi
echo

echo "-- 5. /scan 数据有效性 --"
# ★ 必须先判话题是否存在。否则 ros2 topic echo 超时返回空，
#   grep -c "inf" 得到 0，会被误判成"无 inf，全部命中" → 假阳性 PASS。
if ! printf '%s\n' "$TOPICS" | grep -qx "/scan"; then
  skip "/scan 有效回波" "/scan 话题不存在，跳过"
else
  SCAN_ONE=$(timeout 8 ros2 topic echo /scan --once 2>/dev/null | head -40)
  if ! printf '%s' "$SCAN_ONE" | grep -q "ranges:"; then
    ng "/scan 有效回波" "有话题但取不到有效帧"
  else
    INF_N=$(printf '%s' "$SCAN_ONE" | grep -c "inf")
    RANGE_N=$(printf '%s' "$SCAN_ONE" | sed -n '/ranges:/,/]/p' | grep -cE '^-?[0-9]|inf')
    if [ "$INF_N" -gt 0 ]; then
      ok "/scan 有效回波" "含 ${INF_N} 行 inf（无遮挡方向，正常）"
    else
      ok "/scan 有效回波" "无 inf，${RANGE_N} 个测距值"
    fi
  fi
fi
echo

echo "-- 6. 运行中的节点（定位问题用）--"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/  /' | head -20
echo

echo "=============================================="
printf ' 结果：%d PASS / %d FAIL / %d SKIP\n' "$PASS" "$FAIL" "$SKIP"
echo "=============================================="

if [ "$FAIL" -gt 0 ]; then
  cat <<'EOF'

排查顺序（别乱试，按这个顺序）：

 /scan 无数据
   1. 是不是 gui:=false 起的？→ Gazebo Classic 在无渲染时 ray 传感器不工作。
      去掉 gui:=false 重跑（本机有显示器，默认就是 true）。
   2. 仍无 → 看 gzserver 日志里有没有 "laser_controller"；
      没有就是 libgazebo_ros_ray_sensor.so 没加载，检查 URDF 里 plugin 段。
   3. 有插件但仍无 → ros2 topic info /scan 看 publisher 是否 ≥1。

 /odom 无数据
   1. 看 gzserver 日志有没有 "Advertise odometry on [/odom]"
   2. Gazebo 里模型有没有掉下去（spawn 时 -z 0.05，掉穿地面会静止）

 TF 查不到
   1. URDF 里 diff_drive 的 <publish_odom_tf> 是否为 true
   2. 有 /odom 但无 TF → 就是这一项

 两个都无数据
   → /clock 有没有？没有 = 仿真根本没 step，先解决 Gazebo 本身。
EOF
  exit 1
fi

echo
echo " 全过。W3 出口判据达成，可以进 G1（连发 5 个目标点）。"
exit 0
