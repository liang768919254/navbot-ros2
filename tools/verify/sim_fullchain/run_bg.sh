#!/bin/bash
# ============================================================================
# 后台启动仿真/导航，进程独立于当前 shell 存活
# 用法：
#   ./run_bg.sh <名字> <命令...>
# 例：
#   ./run_bg.sh sim  ros2 launch navbot_bringup sim_navbot.launch.py gui:=false
#   ./run_bg.sh slam ros2 launch navbot_bringup slam_mapping.launch.py use_rviz:=false
# 日志：.run/<名字>.log    停止：./stop_bg.sh <名字>
# ============================================================================
set -u
RUN_DIR="/home/liang/navbot_ws/tools/verify/sim_fullchain"
NAME="${1:?需要给个名字}"; shift
mkdir -p "$RUN_DIR"

# 已在跑就先不重复起
if [ -f "$RUN_DIR/$NAME.pid" ] && kill -0 "$(cat "$RUN_DIR/$NAME.pid")" 2>/dev/null; then
    echo "[run_bg] $NAME 已在运行 (pid $(cat "$RUN_DIR/$NAME.pid"))，先 stop 再起"
    exit 0
fi

: > "$RUN_DIR/$NAME.log"
setsid nohup bash -c "
    source $RUN_DIR/env.sh
    cd /home/liang/navbot_ws
    exec $*
" > "$RUN_DIR/$NAME.log" 2>&1 < /dev/null &
echo $! > "$RUN_DIR/$NAME.pid"
sleep 1
echo "[run_bg] $NAME 已启动 pid=$! 日志=$RUN_DIR/$NAME.log"
