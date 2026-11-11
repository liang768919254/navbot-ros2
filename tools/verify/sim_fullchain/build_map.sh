#!/bin/bash
# ============================================================================
# build_map.sh —— 一键建图：起仿真 → 起 SLAM → 驱车 → 存图 → 关
# ============================================================================
# 为什么全流程写在一个脚本里：
#   本机 Bash 沙箱在【每次工具调用结束时回收所有子进程】，
#   所以不能用「调用1起仿真、调用2驱车、调用3存图」的方式。
#   所有步骤必须在同一次调用（= 同一个脚本进程生命周期）内跑完。
#
# 用法：bash build_map.sh [路线]      路线: loop（默认）| survey
# 产物：maps/navbot_room.pgm/.yaml（覆盖 install 里的副本前，先备份）
# ============================================================================
# ★ 不能用 set -u：ROS 的 setup.bash 里有 AMENT_TRACE_SETUP_FILES 这类
#   未定义变量引用，开了 -u 会在 source 时直接退出（行 8 报"未绑定的变量"）
set +u
RUN_DIR="/home/liang/navbot_ws/tools/verify/sim_fullchain"
MAPS_DIR="/home/liang/navbot_ws/src/navbot_bringup/maps"
ROUTE="${1:-loop}"

cd "$RUN_DIR" || exit 1
# ★ REUSE_SIM=1：复用外部已起好的仿真（你自己终端里的 GUI 版），
#   不清场、不重启，只接管 SLAM 与驱车。
REUSE_SIM="${REUSE_SIM:-0}"
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1
    echo "==================== ① 复用已有仿真（REUSE_SIM=1）===================="
else
    bash stop_bg.sh all >/dev/null 2>&1
    sleep 2
    echo "==================== ① 启动仿真（无 GUI，物理不受渲染影响）===================="
    bash run_bg.sh sim ros2 launch navbot_bringup sim_navbot.launch.py gui:=false
    sleep 14
fi

source "$RUN_DIR/env.sh"

# 等 /scan 真出数据再继续，否则 SLAM 起来没东西可匹配
echo "---- 等待 /scan 有数据 ----"
for i in $(seq 1 20); do
    if timeout 3 ros2 topic echo /scan --once --field header >/dev/null 2>&1; then
        echo "  /scan OK（第 ${i} 次探测）"; break
    fi
    sleep 2
done

echo "==================== ② 启动 slam_toolbox ===================="
bash run_bg.sh slam ros2 launch navbot_bringup slam_mapping.launch.py use_rviz:=false use_sim_time:=true
sleep 12

echo "==================== ③ 驱车覆盖房间（路线=$ROUTE）===================="
timeout 260 python3 "$RUN_DIR/drive.py" "$ROUTE" 2>&1 | grep -E "直线|转向|路线|停车"

echo "==================== ④ 等待地图稳定 ===================="
sleep 4

echo "==================== ⑤ 保存地图 ===================="
OUT="$RUN_DIR/map_built"
rm -f "${OUT}.pgm" "${OUT}.yaml"
timeout 60 ros2 run nav2_map_server map_saver_cli -f "$OUT" 2>&1 | tail -4
if [ -f "${OUT}.pgm" ]; then
    echo "---- 地图已生成 ----"
    ls -l "${OUT}.pgm" "${OUT}.yaml"
    head -c 40 "${OUT}.pgm" | od -c | head -2
    echo "---- 地图质量判据 ----"
    /usr/bin/python3 - "$OUT.pgm" <<'PY'
import sys, collections
f = open(sys.argv[1], 'rb')
assert f.readline().strip() == b'P5'
w, h = map(int, f.readline().split())
maxv = int(f.readline())
data = f.read()
c = collections.Counter(data)
occ = c.get(0, 0)          # 占用（黑）→ 墙
free = c.get(254, 0)       # 空闲（白）→ 可走
unk = c.get(205, 0)        # 未知（灰）→ 没扫到
tot = w * h
print(f"  尺寸 {w}x{h}  总格 {tot}")
print(f"  墙 {occ} ({occ/tot*100:.1f}%)  空闲 {free} ({free/tot*100:.1f}%)  未知 {unk} ({unk/tot*100:.1f}%)")
# 判据：墙要有（说明扫到了结构），未知不能太多（说明覆盖够）
ok_wall = occ > 200
ok_unk = unk / tot < 0.15
print(f"  判据1 墙像素>200        : {'PASS' if ok_wall else 'FAIL'} ({occ})")
print(f"  判据2 未知区域<15%      : {'PASS' if ok_unk else 'FAIL'} ({unk/tot*100:.1f}%)")
print(f"  => {'建图质量合格' if ok_wall and ok_unk else '建图覆盖不足，需重跑'}")
PY
else
    echo "!!!! 地图保存失败 !!!!"
fi

echo "==================== ⑥ 收尾 ===================="
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1   # 保留你的 Gazebo
else
    bash stop_bg.sh all >/dev/null 2>&1
fi
echo "build_map.sh 结束"
