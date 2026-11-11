#!/bin/bash
# ============================================================================
# navbot 仿真/导航统一环境入口
# ============================================================================
# 为什么要把 PATH 里的托管 python 剔掉：
#   WorkBuddy 把托管的 python3.13 放在 PATH 最前面，而 ROS 2 Humble 的
#   所有 Python 节点（spawn_entity.py / ros2 CLI / slam_toolbox 等）都编译
#   绑定在【系统 python 3.10】上。python3 一旦解析到 3.13：
#     - numpy / yaml 等 dist-packages 全找不到 → ModuleNotFoundError
#     - ROS 的 .so 也 import 不进来
#   所以这里把 /usr/bin 提到最前，确保 python3 == 3.10。
# ============================================================================

# 1) 剔掉托管 python 的路径，把系统路径置顶
CLEAN_PATH=""
IFS=':' read -ra _parts <<< "$PATH"
for _p in "${_parts[@]}"; do
    case "$_p" in
        *".workbuddy/binaries/python"*) continue ;;  # 跳过托管 python
    esac
    CLEAN_PATH="${CLEAN_PATH:+$CLEAN_PATH:}$_p"
done

# 2) 系统路径置顶
export PATH="/usr/bin:/bin:$CLEAN_PATH"

# 3) ROS 环境
source /opt/ros/humble/setup.bash
source /home/liang/navbot_ws/install/setup.bash

# 4) Gazebo Classic 资源路径
#    不 source 这个的话 GAZEBO_RESOURCE_PATH 为空，OGRE 找不到 resources.cfg
#    与 shader 库，表现为：
#      gzclient: "Unable to find shader lib" → 段错误退出
#      gzserver: "Unable to create the rendering window"
#                → "Failed to load the Rendering engine subsystem"
#                → "Unable to load sensors" → "Unable to load gazebo"
#    gzserver 也需要渲染引擎，因为 ray 传感器走 RenderingIface；
#    一旦这里失败，世界根本没加载，/spawn_entity 服务也就不存在。
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# 5) 渲染后端选择
#    本机有真实 GPU：/dev/dri/card1 + renderD128，glxinfo 报
#    "Accelerated: yes / Mesa Intel(R) Graphics (RPL-P) / OpenGL 4.6"。
#    但助手执行环境的 Max address space 只有 4GB（soft 与 hard 同为
#    4194304 KB，没有真实 root 改不动）。Gazebo Classic 的 OGRE + Mesa
#    初始化要预留几十 GB 地址空间，硬件路径和 llvmpipe 路径都会撞上限：
#        mmap() failed: Cannot allocate memory
#        terminate called recursively → 核心转储
#    ⇒ GUI 仿真必须在【用户自己的终端】里起（见 run_gui.sh，那里是
#      unlimited）。本开关只在"地址空间够、但 GPU 不可见"的环境里兜底。
if [ "${NAVBOT_HW_GL:-0}" != "1" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
fi

# 6) 自检：python3 必须是 3.10，否则 ROS 节点全废
_pyver="$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null)"
if [ "$_pyver" != "3.10" ]; then
    echo "[env.sh] 警告：python3 = $_pyver，ROS Humble 需要 3.10" >&2
fi
unset _parts _p _pyver CLEAN_PATH

# 6) /dev/shm 过小（本机仅 10MB）时，FastDDS 的 SHM 传输会失败导致
#    跨进程发现不到节点（ros2 topic list 全空）。强制走 UDP。
#    ★ 2026-11-09 复查：本机 /dev/shm 已恢复为 7.7G，SHM 本身够用。
#      但仍保留 profile —— 之前踩过 SHM 段的坑，UDP 更稳且本机仿真带宽够。
if [ -f "/home/liang/navbot_ws/tools/diag/fastdds_udp_only.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="/home/liang/navbot_ws/tools/diag/fastdds_udp_only.xml"
fi
export ROS_LOCALHOST_ONLY=0

# 7) ★ 给「用户在自己终端起的 Gazebo」一个显式标记
#    助手侧的 stop_bg.sh all 会 pkill gzserver/gzclient。如果 Gazebo 是
#    用户在别的终端跑的，助手一跑 all 模式就把它误杀了。
#    run_gui.sh 会设 NAVBOT_EXT_SIM=1，这里探到就自动把下游脚本切到
#    REUSE_SIM=1 语义（只清上层，不碰 Gazebo）。
#    ⚠️ 这条导出只对 run_gui.sh 那个进程树有效，助手自己的 shell 探不到
#       —— 所以助手侧仍需显式 REUSE_SIM=1。它保证的是「用户开了 Gazebo
#        之后，在自己终端里误跑 verify_nav.sh 不会把 Gazebo 带走」。
if [ "${NAVBOT_EXT_SIM:-0}" = "1" ]; then
    export REUSE_SIM=1
fi
