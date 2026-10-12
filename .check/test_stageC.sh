#!/usr/bin/env bash
# ===========================================================================
# 阶段C 自检：建图（SLAM）
#   起 Gazebo → 起 slam_toolbox → 自动走一圈 → 检查 /map → 存图
#
# ★★ 必须全部在同一个脚本里跑：跨 Bash 调用 ROS 节点互相发现不了 ★★
# 用法：bash .check/test_stageC.sh [gui] [profile] [auto|manual]
#   auto   = 用 auto_scan.py 自动走一圈（无人值守）
#   manual = 只起环境，等你手动 ros2 run teleop_twist_keyboard 走完回来存图
# ===========================================================================
set -o pipefail
WS="/home/liang/桌面/具身智能学习路线/bridge/ros_study"
GUI="${1:-false}"
PROFILE="${2:-sandbox}"
MODE="${3:-auto}"

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

cleanup () {
    kill -INT $SIM_PID 2>/dev/null; kill -INT $SLAM_PID 2>/dev/null
    sleep 2
    pkill -9 -f 'gzserver|gzclient' 2>/dev/null
    pkill -9 -f async_slam_toolbox_node 2>/dev/null
    pkill -9 -f 'robot_state_publisher' 2>/dev/null
}
trap cleanup EXIT

echo "########## 阶段C：SLAM 建图自检  模式=$MODE ##########"
pkill -9 -f 'gzserver|gzclient' 2>/dev/null
pkill -9 -f async_slam_toolbox 2>/dev/null
sleep 3

# ---------- ① 起仿真（不带 RViz，GUI 可选）----------
echo "-- 起 Gazebo..."
ros2 launch navbot_bringup sim_navbot.launch.py gui:=$GUI \
    > "$WS/.check/stageC_sim.log" 2>&1 &
SIM_PID=$!
for i in $(seq 1 30); do
    sleep 2
    grep -qE 'Spawn status: SpawnEntity: Successfully' "$WS/.check/stageC_sim.log" 2>/dev/null \
        && { echo "   [OK] 车已生成"; break; }
done
sleep 3

# ---------- ② 起 slam_toolbox ----------
echo "-- 起 slam_toolbox..."
ros2 launch navbot_bringup slam_mapping.launch.py \
    use_sim_time:=true use_rviz:=$GUI \
    > "$WS/.check/stageC_slam.log" 2>&1 &
SLAM_PID=$!
for i in $(seq 1 25); do
    sleep 2
    ros2 node list 2>/dev/null | grep -q slam_toolbox \
        && { echo "   [OK] slam_toolbox 已上线"; break; }
done
sleep 4

echo
echo "===== 建图前置检查 ====="
ros2 node list 2>/dev/null | sed 's/^/  /'
HZ=$(timeout 8 ros2 topic hz /map 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
echo "  /map 速率: ${HZ:-尚无数据（正常，还没开始走）}"

# ---------- ③ 走一圈 ----------
echo
if [ "$MODE" = "auto" ]; then
    echo "-- 自动走位扫描（约 3 分钟）--"
    python3 "$WS/.check/auto_scan.py" 2>&1 | sed 's/^/  /'
else
    echo "-- 手动模式：请另开终端执行 --"
    echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo "   走一圈覆盖房间（慢速 ~0.13 m/s），回到起点后按 Ctrl-C"
    echo "   然后回来按回车继续存图"
    read -r _
fi

# ---------- ④ 检查地图并保存 ----------
echo
echo "===== ④ 检查 /map 并保存 ====="
ros2 node list 2>/dev/null | grep -q slam_toolbox \
    && echo "  [OK] slam_toolbox 仍活着（存图前提）" \
    || echo "  [FAIL] slam_toolbox 已退出，map_saver 会报 Map not received"

python3 - <<'PY'
import subprocess, re
try:
    out = subprocess.run(['ros2','topic','echo','/map','--once'],
                         capture_output=True, text=True, timeout=15).stdout
    if not out.strip():
        print('  [FAIL] /map 无数据'); raise SystemExit
    w  = re.search(r'^\s*width:\s*(\d+)',   out, re.M)
    h  = re.search(r'^\s*height:\s*(\d+)',  out, re.M)
    res= re.search(r'^\s*resolution:\s*([\d.eE+-]+)', out, re.M)
    org= re.search(r'^\s*origin:\s*$', out, re.M)
    print(f'  地图尺寸 : {w.group(1)} x {h.group(1)} 像素')
    print(f'  分辨率   : {res.group(1)} m/像素')
    if res: print(f'  占地     : {int(w.group(1))*float(res.group(1)):.1f} x '
                  f'{int(h.group(1))*float(res.group(1)):.1f} m')
    data = re.search(r'data:\s*\[([-\d,\s]+)', out)
    if data:
        vals=[int(v) for v in data.group(1).replace('\n',' ').split(',') if v.strip()]
        if vals:
            occ  = sum(1 for v in vals if v > 65)
            free = sum(1 for v in vals if v == 0)
            unk  = sum(1 for v in vals if 0 < v <= 65)
            tot  = len(vals)
            print(f'  占用(墙) {occ*100//tot}%   空地 {free*100//tot}%   未知 {unk*100//tot}%')
            print('  [PASS] 地图有内容' if occ>0 and free>0 else '  [FAIL] 地图疑似空白')
except Exception as e:
    print(f'  [FAIL] 读 /map 失败: {e}')
PY

OUT="$WS/.check/maps"
mkdir -p "$OUT"
echo
echo "  -- 保存地图到 $OUT --"
cd "$OUT"
timeout 60 ros2 run nav2_map_server map_saver_cli -f navbot_room_new 2>&1 | tail -5
ls -la "$OUT" 2>/dev/null | sed 's/^/  /'

echo
echo "########## 阶段C 自检结束 ##########"
