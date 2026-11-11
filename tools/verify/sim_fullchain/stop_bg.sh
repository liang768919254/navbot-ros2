#!/bin/bash
# 停止后台任务：./stop_bg.sh <名字> | ./stop_bg.sh all
set -u
RUN_DIR="/home/liang/navbot_ws/tools/verify/sim_fullchain"

stop_one() {
    local n="$1"
    [ -f "$RUN_DIR/$n.pid" ] || { echo "[stop_bg] $n 无 pid 文件"; return; }
    local p; p="$(cat "$RUN_DIR/$n.pid")"
    # 杀整个进程组（setsid 起来的进程组 = pid）
    kill -TERM -"$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null
    sleep 2
    kill -KILL -"$p" 2>/dev/null
    rm -f "$RUN_DIR/$n.pid"
    echo "[stop_bg] $n 已停止"
}

case "${1:-all}" in
    all)
        # ★★ 安全闸门：先探「用户终端里有没有 Gazebo」。
        #   all 模式会 pkill gzserver/gzclient —— 如果这次仿真其实是用户
        #   在自己终端跑的（run_gui.sh），一跑 all 就把他的窗口干掉，
        #   而且报错是「仿真挂了」，看着像助手脚本有问题，极难自证。
        #   已有 REUSE_SIM=1 → 本来就不走 all；这里再兜一层：all 也先看进程
        #   归属，发现 gzserver 的父进程不是助手起的 launcher 就拒绝。
        if [ "${NAVBOT_FORCE_ALL:-0}" != "1" ] && pgrep -x gzserver >/dev/null 2>&1; then
            _ext=0
            for _p in $(pgrep -x gzserver); do
                _ppid=$(awk '{print $4}' "/proc/$_p/stat" 2>/dev/null)
                _pcmd=$(tr '\0' ' ' < "/proc/$_ppid/cmdline" 2>/dev/null)
                case "$_pcmd" in
                    *"$RUN_DIR/run_bg.sh"*|*"run_bg.sh sim"*) ;;   # 助手起的
                    *) _ext=1 ;;
                esac
            done
            if [ "$_ext" = "1" ]; then
                echo "[stop_bg] !! 检测到 Gazebo 不是你（助手）起的 —— 拒绝 all 模式，改走 stack"
                echo "[stop_bg]    确实要连它一起杀：NAVBOT_FORCE_ALL=1 bash stop_bg.sh all"
                set -- stack
            fi
        fi
        ;;
esac

case "${1:-all}" in
    all)
        # 兜底清扫残留，按进程名
        for n in "$RUN_DIR"/*.pid; do
            [ -e "$n" ] && stop_one "$(basename "$n" .pid)"
        done
        pkill -f "gzserver " 2>/dev/null; pkill -f "gzclient " 2>/dev/null
        pkill -f "ros2 launch navbot" 2>/dev/null
        pkill -f robot_state_publisher 2>/dev/null
        pkill -f slam_toolbox 2>/dev/null
        pkill -f "nav2_" 2>/dev/null
        # RViz 只在 FORCE_ALL 时杀（默认可能是用户自己的窗口，见 stack 段注释）
        [ "${NAVBOT_FORCE_ALL:-0}" = "1" ] && pkill -f rviz2 2>/dev/null
        sleep 1; echo "[stop_bg] 全清完成"
        ;;
    stack)
        # 只清「建图 / 导航 / RViz」，保留 Gazebo。
        # 用在「Gazebo 由用户在自己的终端起（GUI），助手只接管上层」的场景。
        #
        # ★ 这里绝对不能 pkill -f "ros2 launch navbot"：
        #   用户终端里跑的就是 ros2 launch navbot_bringup sim_navbot.launch.py，
        #   一并 pkill 会把他的 Gazebo 杀掉。
        #   同理也不能 pkill gzserver / gzclient。
        #
        # ★★ 更隐蔽的一条（2026-11-10 补）：也不能杀 rviz2。
        #   用户可能正用 run_rviz.sh 开着 RViz 看实况，那 RViz 是【他的窗口】，
        #   杀掉了他会以为是 RViz 自己崩了。用户自己起的 RViz 若确实要关，
        #   他自己关窗口即可。助手只清 .run/rviz.pid 记录的那个。
        for n in slam nav rviz; do
            [ -f "$RUN_DIR/$n.pid" ] && stop_one "$n"
        done
        pkill -f slam_toolbox 2>/dev/null
        pkill -f "nav2_" 2>/dev/null
        pkill -f lifecycle_manager 2>/dev/null
        pkill -f "ros2 launch navbot_bringup slam_mapping" 2>/dev/null
        pkill -f "ros2 launch navbot_bringup nav_bringup" 2>/dev/null
        pkill -f set_pose.py 2>/dev/null
        pkill -f drive.py 2>/dev/null
        pkill -f avoid_test.py 2>/dev/null
        sleep 1; echo "[stop_bg] 导航栈已清（Gazebo 与你自己起的 RViz 保留）"
        ;;
    *) stop_one "$1" ;;
esac
