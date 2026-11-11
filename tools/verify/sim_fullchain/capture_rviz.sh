#!/usr/bin/env bash
# ============================================================================
# capture_rviz.sh —— 全流程 + RViz2 可视化分阶段截图（最终版）
# ============================================================================
# 产出 .run/shots/：
#   01_map_rviz.png   建图后的地图 + 全局/局部代价地图 + 激光
#   02_nav_path.png   导航中：全局路径 + 距离剩余
#   03_avoid.png      动态障碍出现，局部代价地图标记并按新路径绕行
#
# ★ 三个已验证的要点：
#   ① 窗口标题是 "<rviz文件路径>* - RViz"，用 wmctrl -l | grep -i rviz 可取到 wid，
#      import -window <wid> 只截该窗口（比截 root 干净）。
#   ② RViz 退出时段错误是 OGRE 在信号处理里清理资源导致的，
#      属退出期问题，不影响运行和截图。用 wmctrl -c 发正常关闭可减少发生。
#   ③ 无 GPU（/dev/dri 不存在）时设 LIBGL_ALWAYS_SOFTWARE=1 走 llvmpipe，
#      OpenGL 4.5 可用；地图层会报 indexed_8bit_image 着色器警告，但渲染正常。
# ============================================================================
set +u
RUN_DIR="/home/liang/navbot_ws/tools/verify/sim_fullchain"
SHOTS="$RUN_DIR/shots"
mkdir -p "$SHOTS"
cd "$RUN_DIR" || exit 1
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1

# ★ RViz 配置必须走 install 的 share 绝对路径。
#   原写法是 "$RUN_DIR/../src/navbot_bringup/rviz/..."，那是这些脚本还放在
#   .run/ 目录时的相对路径；归档到 tools/verify/sim_fullchain/ 后就指向
#   /home/liang/navbot_ws/tools/verify/src/...，根本不存在。
#   配置读不到时 rviz2 会静默退化成默认布局 —— 没有 Map / Costmap / Path，
#   截出来的图看着像 RViz 其实是空白网格，属于「假成功」。
RVIZ_CFG="/home/liang/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz/navbot_nav.rviz"
[ -f "$RVIZ_CFG" ] || { echo "!! RViz 配置不存在: $RVIZ_CFG"; exit 1; }

REUSE_SIM="${REUSE_SIM:-0}"
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1
    echo "=========== ① 复用已有仿真（REUSE_SIM=1）==========="
else
    bash stop_bg.sh all >/dev/null 2>&1
    sleep 2
    echo "=========== ① 仿真 ==========="
    bash run_bg.sh sim ros2 launch navbot_bringup sim_navbot.launch.py gui:=false
    sleep 14
fi
source "$RUN_DIR/env.sh"
for i in $(seq 1 20); do
    timeout 3 ros2 topic echo /scan --once --field header >/dev/null 2>&1 && { echo "  /scan OK"; break; }
    sleep 2
done

echo "=========== ② Nav2 ==========="
bash run_bg.sh nav ros2 launch navbot_bringup nav_bringup.launch.py \
    use_rviz:=false use_sim_time:=true
sleep 14

echo "=========== ③ AMCL 初值（--auto，带验证）==========="
# ★★ S34 改：不再读 /odom 当初值（「map 与 odom 重合」只在冷启动成立）。
#   set_pose.py --auto 复合 map->odom 求真正的 map 坐标，并验证注入结果。
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

for i in $(seq 1 15); do
    timeout 6 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active \
        && { echo "  Nav2 active"; break; }
    sleep 3
done

echo "=========== ④ RViz2 ==========="
setsid nohup rviz2 -d "$RVIZ_CFG" > "$RUN_DIR/rviz.log" 2>&1 < /dev/null &
sleep 20
WID=$(wmctrl -l | grep -i "rviz" | head -1 | awk '{print $1}')
echo "  rviz window id = ${WID:-未找到}"
[ -n "$WID" ] && wmctrl -i -r "$WID" -b add,maximized_vert,maximized_horz 2>/dev/null
[ -n "$WID" ] && wmctrl -i -a "$WID" 2>/dev/null
sleep 4

shot() {
    local f="$1" tag="$2"
    local w
    w=$(wmctrl -l | grep -i rviz | head -1 | awk '{print $1}')
    if [ -n "$w" ]; then
        import -window "$w" "$SHOTS/$f" 2>/dev/null
    else
        import -window root "$SHOTS/$f" 2>/dev/null
    fi
    [ -f "$SHOTS/$f" ] && echo "  [$tag] $f  $(identify -format '%wx%h' "$SHOTS/$f" 2>/dev/null)"
}

sleep 2
shot 01_map_rviz.png "地图+代价地图"

echo "=========== ⑤ 导航并截图 ==========="
cat > "$RUN_DIR/_g1.sh" <<'EOS'
#!/bin/bash
set +u
source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh
exec timeout 220 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 2.6, y: -2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
EOS
chmod +x "$RUN_DIR/_g1.sh"
setsid nohup "$RUN_DIR/_g1.sh" > "$RUN_DIR/g1.log" 2>&1 < /dev/null &
sleep 8
shot 02_nav_path.png "全局路径"
for i in $(seq 1 45); do
    grep -qE "SUCCEEDED|ABORTED" "$RUN_DIR/g1.log" 2>/dev/null && break
    sleep 3
done
echo "  导航结果: $(grep -oE 'SUCCEEDED|ABORTED' "$RUN_DIR/g1.log" | head -1)"

echo "=========== ⑥ 避障并截图 ==========="
cat > "$RUN_DIR/_g2.sh" <<'EOS'
#!/bin/bash
set +u
source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh
exec timeout 220 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -1.9, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
EOS
chmod +x "$RUN_DIR/_g2.sh"
setsid nohup "$RUN_DIR/_g2.sh" > "$RUN_DIR/g2.log" 2>&1 < /dev/null &
sleep 11
/home/liang/.workbuddy/binaries/python/envs/default/bin/python - <<'PY'
import os, subprocess
sdf = ('<sdf version="1.6"><model name="cap_obstacle"><static>true</static>'
       '<pose>0 0 0 0 0 0</pose><link name="link">'
       '<collision name="c"><geometry><box><size>0.35 0.35 0.5</size></box></geometry></collision>'
       '<visual name="v"><geometry><box><size>0.35 0.35 0.5</size></box></geometry>'
       '<material><ambient>1 0.1 0.1 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>'
       '</visual></link></model></sdf>')
X, Y = 1.2, -1.9
payload = ("{name: 'cap_obstacle', xml: '%s', initial_pose: {position: {x: %f, y: %f, z: 0.25}, "
           "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}" % (sdf, X, Y))
s = '/home/liang/navbot_ws/tools/verify/sim_fullchain/_cap_spawn.sh'
open(s, 'w').write('#!/bin/bash\nset +u\nsource /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh\n'
                   'timeout 40 ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "%s"\n' % payload)
os.chmod(s, 0o755)
r = subprocess.run([s], capture_output=True, text=True, timeout=60)
print("  障碍插入:", 'success=True' in r.stdout)
PY
sleep 5
shot 03_avoid.png "避障中"

for i in $(seq 1 30); do
    grep -qE "SUCCEEDED|ABORTED" "$RUN_DIR/g2.log" 2>/dev/null && break
    sleep 3
done
echo "  避障段结果: $(grep -oE 'SUCCEEDED|ABORTED' "$RUN_DIR/g2.log" | head -1)"

echo "=========== 收尾 ==========="
wmctrl -c "RViz" 2>/dev/null
sleep 3
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1   # 保留你的 Gazebo
else
    bash stop_bg.sh all >/dev/null 2>&1
fi
echo "截图清单:"; ls -l "$SHOTS"
