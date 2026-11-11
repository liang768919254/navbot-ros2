#!/bin/bash
# ============================================================================
# live.sh —— 一条命令起「Gazebo + SLAM + RViz 实时联调」全套
# ============================================================================
# 为什么需要这个脚本（2026-11-10 实测后新写）：
#
#   以前助手能在后台起 Gazebo，然后分多次调用一点点推进。
#   2026-11-10 复查发现沙箱行为变了：
#     - setsid nohup 起的进程，跨 Bash 调用 100% 被回收
#       （探针实验：同调用内 START 写入成功，下一次调用进程就没了）
#     - 一条调用内跑完整链路（仿真 14s + SLAM 12s + 建图 240s）会撞
#       工具默认 120s 超时被 SIGTERM
#   ⇒ 助手既不能后台起仿真，也不能一次跑完整链路。
#   ⇒ 剩下的唯一正确做法：全部在【你的终端】里跑，助手只在需要时读日志/截图。
#
# 这个脚本干的事：
#   1. 起 Gazebo GUI（gzserver + gzclient）
#   2. 起 slam_toolbox（建图模式）
#   3. 起 RViz2（map 为 Fixed Frame，Scale=150）
#   4. 全部跑在当前终端，Ctrl+C 一次性全停，不留孤儿
#
# 用法：
#   bash live.sh                  # 建图模式（默认）：SLAM + RViz 看地图生长
#   bash live.sh drive            # 建图模式 + 自动跑 survey 路线扫房间
#   bash live.sh nav              # 导航模式（要先有地图）：Nav2 + AMCL + RViz
#   bash live.sh nav 2.0 -1.8     # 导航模式 + 自动导航到指定点
#
# 结束：Ctrl+C（脚本会清理所有子进程）
# ============================================================================
set +u

MODE="${1:-slam}"
GOAL_X="${2:-}"
GOAL_Y="${3:-}"

HERE=/home/liang/navbot_ws/tools/verify/sim_fullchain

# ★ 必须先设再 source：env.sh 里 `if [ "${NAVBOT_HW_GL:-0}" != "1" ]` 会写死软件渲染
export NAVBOT_HW_GL=1
export NAVBOT_EXT_SIM=1
source "$HERE/env.sh"

PIDS=()
cleanup() {
    echo ""
    echo "== 收尾：停止所有子进程 =="
    for p in "${PIDS[@]}"; do
        kill -INT "$p" 2>/dev/null
    done
    sleep 3
    for p in "${PIDS[@]}"; do
        kill -TERM "$p" 2>/dev/null
    done
    sleep 2
    pkill -x gzserver 2>/dev/null
    pkill -x gzclient 2>/dev/null
    pkill -f slam_toolbox 2>/dev/null
    pkill -f "nav2_" 2>/dev/null
    pkill -f lifecycle_manager 2>/dev/null
    pkill -f rviz2 2>/dev/null
    echo "== 已全部停止 =="
    exit 0
}
trap cleanup INT TERM

echo "=============================================================="
echo " navbot 实时联调 —— 模式: $MODE"
echo "   渲染 : $([ "${LIBGL_ALWAYS_SOFTWARE:-0}" = "1" ] && echo 'llvmpipe 软件' || echo '硬件 GPU')"
echo "   DISPLAY: ${DISPLAY:-<空！>}"
echo "   Ctrl+C 结束全部"
echo "=============================================================="

# ---------------------------------------------------------------- ⓿ 参数闸门
# ★★ 2026-11-10 加。起因：动手改了 controller 链之后，用坏配置跑了一整轮
#    20 分钟仿真才发现不对 —— 因为 yaml 语法合法、ROS 也能起来，
#    只是「控制器关系」错了，运行时才崩。
#    这两个检查都是纯离线的，2 秒跑完，放在起仿真之前。
#
#    ★ 这里只用 `set +e` 显式判断，不中断脚本：小梁同学可能只想看日志，
#      但失败时一定要大声说出来，不能默默继续。
if [ "$MODE" = "nav" ]; then
    echo "[0/3] 参数闸门（离线，2s）..."
    NAVPY=$(command -v /usr/bin/python3 >/dev/null 2>&1 && echo /usr/bin/python3 || echo python3)

    if ! "$NAVPY" /home/liang/navbot_ws/tools/verify/check_controller.py 2>&1 \
            | grep -q "结论: 全通"; then
        echo ""
        echo "  !!! controller 链检查没过："
        "$NAVPY" /home/liang/navbot_ws/tools/verify/check_controller.py 2>&1 \
            | grep -E "FAIL" | sed 's/^/      /'
        echo ""
        echo "  !!! 上面的问题会让导航「能起但走不动」。"
        echo "  !!! 仍要继续就输入 y，取消请按 Ctrl+C"
        read -r -n 1 _ans
        echo ""
        [ "$_ans" = "y" ] || exit 1
    else
        echo "      controller 链 OK"
    fi

    if ! "$NAVPY" /home/liang/navbot_ws/tools/verify/check_limits.py 2>&1 \
            | grep -q "结论: 全通"; then
        echo "  !!! 安全阈值检查没过，见上；继续按 y"
        read -r -n 1 _ans
        echo ""
        [ "$_ans" = "y" ] || exit 1
    else
        echo "      costmap 阈值 OK"
    fi

    # ★★ 2026-11-10 加：控制器链检查的「自证」
    #   S29 的事故是 check_controller.py 自己写错了判据 —— 它全绿，
    #   仿真却起不来（primary_controller 写成了参数块名 → SIGSEGV）。
    #   ⇒ 每轮都跑一次故障注入，确认这些判据**真的还会红**。
    #   9 种注入约 4 秒，比白跑 20 分钟仿真便宜太多。
    #   ★ S32 起注入器会同时跑 check_limits.py（车体模型类故障归它管）。
    echo "[0/3] 控制器判据自证（故障注入，3s）..."
    if ! "$NAVPY" /home/liang/navbot_ws/tools/verify/inject_faults.py 2>&1 \
            | grep -q "全部抓到"; then
        echo ""
        echo "  !!! 有故障没被抓到 —— 说明 check_controller.py 的判据失效了，"
        echo "  !!! 它刚才报的「全通」不可信。"
        "$NAVPY" /home/liang/navbot_ws/tools/verify/inject_faults.py 2>&1 \
            | grep -E "漏掉|跳过" | sed 's/^/      /'
        echo ""
        echo "  !!! 仍要继续就输入 y，取消请按 Ctrl+C"
        read -r -n 1 _ans
        echo ""
        [ "$_ans" = "y" ] || exit 1
    else
        echo "      9 种坏配置全部能被抓到"
    fi
fi

# ---------------------------------------------------------------- ① 仿真
echo "[1/3] 起 Gazebo ..."
ros2 launch navbot_bringup sim_navbot.launch.py gui:=true > "$HERE/live_sim.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 30); do
    sleep 2
    if timeout 2 ros2 topic echo /scan --once --field header >/dev/null 2>&1; then
        echo "      /scan OK (${i}x2s)"
        break
    fi
    [ "$i" = "30" ] && { echo "      !! /scan 一直没出来，看 $HERE/live_sim.log"; }
done

case "$MODE" in
# ---------------------------------------------------------------- ② SLAM
slam|drive)
    echo "[2/3] 起 SLAM (slam_toolbox) ..."
    ros2 launch navbot_bringup slam_mapping.launch.py use_rviz:=false use_sim_time:=true \
        > "$HERE/live_slam.log" 2>&1 &
    PIDS+=($!)
    sleep 10
    grep -q "Registering sensor" "$HERE/live_slam.log" && echo "      激光已接入 SLAM"

    echo "[3/3] 起 RViz (建图视图) ..."
    rviz2 -d /home/liang/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz/navbot_slam.rviz \
        > "$HERE/live_rviz.log" 2>&1 &
    PIDS+=($!)

    if [ "$MODE" = "drive" ]; then
        sleep 6
        echo ""
        echo "== 自动扫房间（survey 路线，约 4 分钟）=="
        echo "   ★ 想看的是：地图从中间一小块，逐渐长满整个 6m x 5m 房间"
        python3 "$HERE/drive.py" survey 2>&1 | grep -vE "^\s*$" || true
        echo ""
        echo "== 扫完。存图 =="
        mkdir -p "$HERE/shots"
        ros2 run nav2_map_server map_saver_cli -f /home/liang/navbot_ws/src/navbot_bringup/maps/navbot_room_new 2>&1 | tail -3
        echo "   图已存到 src/navbot_bringup/maps/navbot_room_new.pgm"
        echo "   满意的话手动替换 navbot_room.pgm/yaml（旧图有 .bak 备份）"
    fi
    ;;
# ---------------------------------------------------------------- ② Nav2
nav)
    echo "[2/3] 起 Nav2 (map_server + AMCL + planner + controller) ..."
    ros2 launch navbot_bringup nav_bringup.launch.py use_rviz:=false use_sim_time:=true \
        > "$HERE/live_nav.log" 2>&1 &
    PIDS+=($!)

    # ★★ 2026-11-10 修：原来这里是 `sleep 12` 然后直接发初值。
    #    这是硬赌 —— 机器忙时 12 秒不够，map_server 还没 activate、
    #    地图没进 AMCL，/initialpose 发出去直接丢，且**发完不检查**。
    #    后果（当天实际踩到）：AMCL 一直喊
    #      "cannot publish a pose or update the transform. Please set the initial pose"
    #    → map->odom 不存在 → global_costmap 刷屏
    #      "Timed out waiting for transform from base_footprint to map"
    #    → RViz 里只有一张网格、中间一个白色方块，看着像地图没了。
    #    改成「等 map_server active 且 /map 有数据」再发，发完立刻验 TF。
    echo "      -- 等 map_server 就绪（最多 60s）--"
    MAP_OK=0
    for i in $(seq 1 30); do
        sleep 2
        st=$(timeout 4 ros2 lifecycle get /map_server 2>/dev/null | head -1)
        if echo "$st" | grep -q active; then
            MAP_OK=1
            echo "      map_server active (${i}x2s)"
            break
        fi
    done
    if [ "$MAP_OK" = "0" ]; then
        echo "      !! map_server 60 秒还没 active"
        echo "         先看日志：grep -E 'map_server|ERROR' $HERE/live_nav.log | tail -20"
        echo "         常见原因：map yaml 路径错、PGM 头损坏"
    fi

    echo "      -- 注入 AMCL 初值（--auto：按 map->odom 反推 map 坐标）--"
    # ★★★ S34 改：原来这里用 odom_pose.py 读 /odom 当初值。
    #   那个做法的前提是「map 与 odom 重合」，只在冷启动成立。
    #   跑过几轮导航后 map->odom 已累积偏移（实测 1.4 m），
    #   拿 odom 坐标当 map 坐标注入 → 粒子云撒错位置 → AMCL 冻结 / 跑出平面。
    #
    #   现在用 set_pose.py --auto：它先读 map->odom 再复合 odom 位姿，
    #   算出真正的 map 坐标，并且**注入后验证** map->odom 是不是平面变换，
    #   不是就自动重试 3 次，还不成就报错退出（不再假装成功）。
    #   ★★★ S34 修：原来写成
    #       if timeout 90 python3 set_pose.py --auto 2>&1 | sed 's/^/      /'; then
    #     `if` 判断的是**管道最后一个命令 sed 的返回码**，不是 python 的！
    #     set_pose 失败返回 1，但 sed 正常退出返回 0 → 永远走「通过」分支。
    #     这是「失败被静默吞掉」的经典写法，比不检查还糟。
    #     ⇒ 先把输出写入变量（含返回码），再判断。
    SP_OUT=$(timeout 150 python3 "$HERE/set_pose.py" --auto 2>&1)
    SP_RC=$?
    echo "$SP_OUT" | sed 's/^/      /'
    if [ "$SP_RC" -eq 0 ]; then
        echo "      初值注入 + 验证 通过"
    else
        echo "      !! 初值注入失败或 map->odom 不健康 (rc=$SP_RC)"
        echo "         看：grep '\[amcl' $HERE/live_nav.log | tail -5"
        echo "         处理：Ctrl+C → pkill -f lifecycle_manager; pkill -f nav2_"
        echo "               → 重新 bash $HERE/live.sh nav"
    fi

    # 剩下几个节点会在 map->odom 出现后自动激活，这里只等 bt_navigator
    for i in $(seq 1 20); do
        sleep 2
        timeout 4 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active && {
            echo "      bt_navigator active (${i}x2s)"; break; }
    done


    echo "[3/3] 起 RViz (导航视图) ..."
    rviz2 -d /home/liang/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz/navbot_nav.rviz \
        > "$HERE/live_rviz.log" 2>&1 &
    PIDS+=($!)

    if [ -n "$GOAL_X" ] && [ -n "$GOAL_Y" ]; then
        sleep 6
        echo ""
        echo "== 自动导航到 ($GOAL_X, $GOAL_Y) =="
        timeout 240 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
          "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, \
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback 2>&1 \
          | grep -E "Goal accepted|SUCCEEDED|ABORTED|FAILED" | tail -3
    fi
    ;;
*)
    echo "!! 未知模式: $MODE"; kill -INT $$; exit 1
    ;;
esac

echo ""
echo "=============================================================="
echo " 全部起来了。现在在 RViz 里你能看到："
case "$MODE" in
slam|drive)
    echo "   · 灰 = 未知区域，白 = 已探测可走，黑 = 墙/障碍"
    echo "   · 绿点 = 激光扫描（贴在墙上说明定位准）"
    echo "   · 地图应该铺满整个窗口（Scale=150）"
    ;;
nav)
    echo "   · 地图 + 蓝紫色膨胀层（GlobalCostmap / LocalCostmap）"
    echo "   · 绿线 = 全局路径，橙线 = 局部轨迹"
    echo "   · 工具栏 'Nav2 Goal' 点一下就能发目标"
    ;;
esac
echo ""
echo " 在【另一个终端】可以："
echo "   ros2 topic hz /scan          看激光频率"
echo "   bash $HERE/drive.py loop     让它自己绕一圈"
echo "=============================================================="
echo " 本终端保持不动，Ctrl+C 一次性全停。"

wait
