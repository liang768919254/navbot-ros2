#!/bin/bash
# ============================================================================
# run_gui.sh —— 在你自己的终端里起「带图形窗口」的 Gazebo 仿真
# ============================================================================
# 为什么这件事必须由你在终端做，而不是助手代跑：
#
#   助手的执行环境有 4GB 的「虚拟地址空间」硬上限：
#       Max address space = 4194304 KB  （soft 与 hard 相同）
#   并且没有真实 root 权限，改不动（prlimit 只能降不能升）。
#
#   Gazebo Classic 的 OGRE + Mesa 初始化要几十 GB 的地址空间预留，
#   一撞上限就是：
#       mmap() failed: Cannot allocate memory
#       Failed to create permanent mapping for memfd region ...
#       terminate called recursively   → 核心转储
#
#   你的终端 limits 是 unlimited（已实测 /proc/<pid>/limits 确认），
#   所以 GUI 只能在你这儿起。
#
#   分工：你起 Gazebo（占地址空间的大头），
#         助手跑 SLAM / Nav2 / 驱车（单进程占用小，4GB 够用）。
#
# 用法：
#   bash ~/navbot_ws/tools/verify/sim_fullchain/run_gui.sh
# 结束：Ctrl+C
# ============================================================================
set +u

# 有真显卡时走硬件渲染（本机 Mesa Intel RPL-P，OpenGL 4.6）
# 若要强制软件渲染：NAVBOT_HW_GL=0 bash run_gui.sh
export NAVBOT_HW_GL="${NAVBOT_HW_GL:-1}"

# ★ 标记「仿真是用户在终端跑的」。下游脚本（verify_nav / verify_avoid /
#   build_map / capture_rviz）读到 REUSE_SIM=1 就只清上层，不动 Gazebo。
export NAVBOT_EXT_SIM=1
export REUSE_SIM=1

source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh

echo "=============================================================="
echo " Gazebo 图形仿真"
echo "   世界     : navbot_room.world (6m x 5m)"
echo "   机器人   : navbot（出生点 0,0）"
echo "   DISPLAY  : ${DISPLAY:-<空！会起不来>}"
echo "   渲染     : $([ "${LIBGL_ALWAYS_SOFTWARE:-0}" = "1" ] && echo 'Mesa llvmpipe 软件' || echo '硬件 GPU')"
echo "   Gazebo   : $(gzserver --version 2>/dev/null | head -1)"
echo "=============================================================="
echo " 起来后："
echo "   另开一个终端跑 RViz →  bash ~/navbot_ws/tools/verify/sim_fullchain/run_rviz.sh slam"
echo "   或者让助手接管跑 SLAM / Nav2"
echo "   下游脚本会自动复用这个 Gazebo，不会把它杀掉"
echo "=============================================================="

exec ros2 launch navbot_bringup sim_navbot.launch.py gui:=true
