#!/bin/bash
# ============================================================================
# run_rviz.sh —— 在你自己的终端里起 RViz2
# ============================================================================
# 为什么助手代起不了（实测）：
#   RViz 的 OGRE 初始化走到 "OpenGl version: 4.5 (GLSL 4.5)" 之后就挂：
#       terminate called after throwing an instance of 'std::system_error'
#         what():  资源暂时不可用
#   这是 pthread_create 拿到 EAGAIN —— 渲染线程栈申请不到，根因还是助手
#   执行环境那条 Max address space = 4194304 KB 的硬上限。
#   你的终端是 unlimited。
#
# 用法：
#   bash run_rviz.sh          # 导航视图：地图 + 全局/局部代价地图 + 路径 + 激光
#   bash run_rviz.sh slam     # 建图视图：地图实时生长 + 激光扫描
#   bash run_rviz.sh slam /home/liang/xxx.rviz   # 指定其他配置
#
# 窗口起来后可以最大化（助手截图按窗口 id 抓，尺寸跟着窗口走）。
#
# ★ 关于 Scale：配置里 TopDownOrtho 的 Scale 是 150 像素/米。
#   6m x 5m 的房间 → 900 x 750 像素，窗口 1500x900 正好装下还有余量。
#   如果是 60，整间屋子只有 360 像素宽，看着像地图在角落里缩成一团。
#   ★ 这个值 2026-11-09 之前一直没生效 —— 因为 install/ 下是拷贝不是 symlink，
#     改 src/ 不会同步。已修，install 与 src 现在一致。
# ============================================================================
set +u

MODE="${1:-nav}"
CUSTOM_CFG="${2:-}"

# ★ 开 GUI 时必须走 GPU。env.sh 默认（NAVBOT_HW_GL != 1）会把 LIBGL 设成
#   llvmpipe 软件渲染 —— 那是给助手环境兜底用的。在用户终端里跑软件渲染，
#   RViz 会把 4 个代价地图层 + 2000 个粒子的每帧重绘全压到 CPU，
#   表现是窗口能开但拖动/缩放卡成幻灯片。
#   ★★ 必须放在 source env.sh【之前】—— env.sh 里是 `if [ "${NAVBOT_HW_GL:-0}" != "1" ]`，
#      先 source 再 export 就已经被写成 1 了，改不动。
export NAVBOT_HW_GL="${NAVBOT_HW_GL:-1}"

source /home/liang/navbot_ws/tools/verify/sim_fullchain/env.sh

SHARE=/home/liang/navbot_ws/install/navbot_bringup/share/navbot_bringup/rviz
if [ -n "$CUSTOM_CFG" ]; then
    CFG="$CUSTOM_CFG"
    echo "== RViz: 自定义配置 =="
elif [ "$MODE" = "slam" ]; then
    CFG="$SHARE/navbot_slam.rviz"
    echo "== RViz: 建图视图（slam_toolbox 在跑时地图会实时生长）=="
else
    CFG="$SHARE/navbot_nav.rviz"
    echo "== RViz: 导航视图（Nav2 在跑时能看到代价地图与全局路径）=="
fi

if [ ! -f "$CFG" ]; then
    echo "!! 配置不存在: $CFG"; exit 1
fi

echo "   配置 : $CFG"
echo "   DISPLAY: ${DISPLAY:-<空！>}"
echo "   渲染 : $([ "${LIBGL_ALWAYS_SOFTWARE:-0}" = "1" ] && echo 'llvmpipe 软件' || echo '硬件 GPU')"
exec rviz2 -d "$CFG"
