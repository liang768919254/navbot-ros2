#!/usr/bin/env bash
# ============================================================================
# record_navigation.sh v7 —— 无头环境录制 Nav2 导航为 GIF（防挂死 + 走方形）
# ============================================================================
# 原理不变：Xvfb 虚拟屏跑 RViz → import 定时截图 → convert 合成 GIF
#
# 用法：
#   bash record_navigation.sh [目标x] [目标y] [截图张数] [选项]        # 单点导航
#   bash record_navigation.sh square [边长] [中心x] [中心y] [截图张数] # ★ 走方形
#   bash record_navigation.sh --diag            # 只体检，不启动
#   bash record_navigation.sh --clean-only      # 只清理残留进程
#
#   例子：
#     bash record_navigation.sh 1.0 -0.15 90            # 单点（原用法，没变）
#     bash record_navigation.sh square                  # 走方形，★ 跑完为止（推荐）
#     bash record_navigation.sh square 1.40 -0.5 1.0 200  # 限 200 帧
#     bash record_navigation.sh square 1.20 -0.55 1.00 manual
#
#   ★ 截图张数（第 3/第 5 个参数）的语义：
#       >0  最多拍这么多帧（安全网，可能截断）
#       =0  或省略 ——「跑完为止」：不设帧数上限，直到所有段结束 + 3 帧收尾（方形模式默认）
#     为什么默认改成跑完为止：Gazebo 跟不上真实时间（RTF < 1），车按**仿真时间**走，
#     而截图按**真实时间**拍 —— 一段 1.2m 的导航在墙上可能要几分钟。
#     按帧数硬卡的下场就是「上限到了、第 1 段才刚跑完，后面几段压根没发出去」。
#
#   可选环境变量：
#     INTERVAL=2          每帧间隔秒数（跑完为止模式下想覆盖更长时间就调大它）
#     GIF_DELAY=20        GIF 每帧显示毫秒（20=200ms，即 10 倍速）
#     GIF_SCALE=800x500   GIF 缩放；设 0 保留原分辨率
#     MAX_WALL=1800       跑完为止模式的总时长兜底（秒）
#     STALL_GIVEUP=150    剩余距离连续这么多帧不降就放弃后面的段
#     INIT_X / INIT_Y     初始位姿（map 坐标）。★ 默认 -0.50/-0.50，见 §v4
#     FORCE=1             本地化没起来也强行往下跑（录一段"失败现场"）
#
# 产出：/mnt/workspace/navigation.gif + /mnt/workspace/screenshots/
# ============================================================================
# 【v7 修复】目标响应在传输层丢了（不是导航失败）+ 两处误报
#   实测撞到：Nav2 全部 active、TF 三对全在、车也生成了，但脚本卡在"等目标被接受"，
#   nav.log 里是：
#     [bt_navigator-8] [WARN] Failed to send goal response c21e...(timeout):
#         client will not receive response, at ./src/rmw_response.cpp:154
#   ★ 这句话的含义很容易读错：**响应发不出去 ≠ 目标没被接受**。
#     服务端可能已经在执行了，只是客户端收不到那句 accepted。
#   为什么会丢：客户端和服务端之间那一跳要在发现/匹配完成前把响应传出去；图里刚经历
#   一大串短命进程（本脚本为了体检起了几十个 ros2 CLI）时最容易踩到。
#   ⇒ 现在：
#     · `send_leg_wait()`：一段最多重发 3 次（每次等 15s），重发是安全的 ——
#       Nav2 收到新目标会抢占上一个，不会出现两个目标同时跑
#     · [7/9] 等就绪改成**等日志**（grep 'Managed nodes are active'），
#       不再每 3 秒给 5 个节点各发一次 lifecycle get（那是每轮 5 个 DDS 参与者）
#     · 开目标前 sleep 3，让图安静下来
#     · 诊断修掉两处误报：
#         ① lifecycle_manager **不是** lifecycle 节点，它没有 get_state 服务 ——
#            以前去查它永远显示"服务无响应"，纯属误报
#         ② 11345 被占用时，若是**当前** gzserver 持有那是正常的，
#            只有"端口被占 + 没有 gzserver 进程"才是残留
#
# 【v6 修复】方形默认改成「跑完为止」+ 进度可见 + 卡住能看出来
#   症状：跑 square 得到「第 1/5 段 SUCCEEDED，第 2~5 段 无结果」。
#   ★ 不是导航失败，是**帧数上限（120 帧 × 2s = 4 分钟）到了**，而第 1 段在那前后
#     才刚跑完 —— 第 2 段压根还没发出去，所以它的日志都不存在（=「无结果」）。
#   根因：Gazebo 跟不上真实时间，RTF < 1。车是按**仿真时间**跑的（max_vel_x 0.22 m/s
#     也是仿真时间），而截图循环按**真实时间**计数 —— 墙上时长 = 仿真时长 ÷ RTF。
#     一段 1.2m 的方形边（含起步转向）在仿真里可能就要 40~60s，墙上就是 2~5 分钟。
#   ⇒ 现在：
#     · 方形模式的截图张数默认 0 = **跑完为止**（不设上限，直到所有段结束 + 3 帧收尾），
#       另加 MAX_WALL 总时长兜底（默认 30 分钟），防止无人值守时无限跑
#     · 收尾前先量一次 RTF 并打印（直接解释"为什么这么慢"）
#     · send_leg 加 --feedback，进度行显示「第 N/M 段  剩余 X.XXm  已跑 mm:ss」
#     · 卡住检测：剩余距离连续 60s 不降就报警，连续 STALL_GIVEUP 帧没进展就放弃后面的段
#     · 分段摘要区分「未发送」和「未结束（到上限时还在走）」，并附上怎么调
#
# 【v5 加固】仿真层的门槛从「话题在不在」提高到「车真的生成了 + TF 连上了」
#   上一版只看 /clock /scan /odom 三个话题存不存在，不够：
#   出现过这样一次故障 ——
#     · sim.log 停在 'Calling service /spawn_entity'，后面没有
#       'Spawn status: SpawnEntity: Successfully spawned entity [navbot]'
#     · gzserver 只打了一个空行，diff_drive 插件的日志一行都没有
#     · 于是 Nav2 全部刷 'Invalid frame ID "odom" ... frame does not exist'
#     · 而话题检查却可能放行 ⇒ 白跑一遍
#   车没生成 ⇒ diff_drive 插件没加载 ⇒ /odom 和 odom→base_footprint 都不存在。
#   ⇒ 现在 [4/9] 分四道门，任一道不过就带现场停下：
#       ① /spawn_entity 服务出现（车能不能生成全靠它）
#       ② sim.log 里出现 'Successfully spawned entity'（★ 车真的生成了的判据）
#       ③ 话题 /clock /scan /odom 都在
#       ④ TF odom→base_footprint 真的能查到（Nav2 要的是 TF，不是话题！）
#   ⇒ 另外清理阶段会等 gzserver/gzclient 真的死掉，必要时升级到 kill -9，
#     并检查 11345（Gazebo Classic master 端口）有没有被残留进程占着 ——
#     残留没清干净是「gzserver 活着但 /spawn_entity 不回应」最常见的原因。
#
# 【排查口诀（本次两个故障对比出来的）】看缺的是哪个 TF 帧，就知道查哪一层：
#     ❌ map 帧            → AMCL 没发 map→odom（多半没给初值）→ 查 Nav2 层，见 §v3
#     ❌ odom / base_footprint → diff_drive 插件（Gazebo）或真机串口桥 → 查仿真/硬件层
#     ❌ laser_link        → robot_state_publisher + URDF/joint_states → 查模型层
#
# 【v4 新增】square 模式：连发 4 段 NavigateToPose 绕一个正方形，最后一段回到
#   起点角 → 成环，GIF 首尾能无缝循环。四个角是从地图里离线算出来的（见 §C），
#   不是随手填的。截图循环和分段推进联动：一段结束就发下一段，全部走完再多拍
#   3 帧收尾并自动停（不会白跑满帧数）。
#
# 【v4 修复】初始位姿的落点错了 0.5m —— 这是上一版 GIF 里 "Recoveries: 1" 的原因
#   实测证据（离线量 navbot_room.pgm，再和 navbot_room.world 对照）：
#     · 地图里那根 2.6m 长的隔断墙（world 中心 x=-0.5, y=+0.5）
#       在地图上量出来是 x -1.25~+0.35, y -0.10~+0.10
#     · 三面墙也对得上同一组偏移：west 墙 -3.45 vs world -2.95、east 墙 +2.40 vs +2.95
#     ⇒ map = world + (-0.50, -0.50)。也就是「Gazebo 世界 (0,0)」在地图上其实位于
#       map(-0.50, -0.50)，而 map(0,0) 恰好落在那根隔断墙上（实测是占用格）。
#     车在 Gazebo 里出生在 (0,0)，脚本却告诉 AMCL「你在 map(0,0)」→ 初始位姿偏
#     0.71m → AMCL 只能靠激光匹配硬拉回来 → 表现就是开局一次 recovery、第一段规划特别慢。
#   ⇒ 现在默认 INIT_X/INIT_Y = -0.50/-0.50（实测 map(-0.50,-0.50) 是 free）。
#     真正的地图侧修复是把 navbot_room.yaml 的 origin 从 [-3.5,-3.0] 改成
#     ≈[-2.98,-2.48]（让地图正好以「车的出生点」为中心），改完这里要回到 0/0。
#     验证命令（车在 Gazebo (0,0) 时跑）：
#       ros2 topic echo /amcl_pose --once     # x,y 若是 -0.5/-0.5 → origin 偏了
#
# 【v3 修复】顺序：先建 map 帧（给 AMCL 初值），再等 planner_server
#   /tmp/nav.log 里刷的是：
#     [global_costmap.global_costmap]: Timed out waiting for transform from
#       base_footprint to map ... Invalid frame ID "map" ... frame does not exist
#     [amcl]: AMCL cannot publish a pose... Please set the initial pose...
#   ★ map 帧不是 map_server 建的（它只发 /map 话题，不广播 TF）；
#     map 帧只有 AMCL 的 map→odom 广播会建，而 set_initial_pose: false ⇒
#     不给初值就不建。v1/v2 却先等 planner_server（它在等 map 帧）→ 环形依赖 → 必卡。
#   ⇒ 顺序改成：amcl 就绪 → 发初值（重发直到 map→odom 出现）→ 再等 planner_server。
#
# 【v2 修复】结构性缺陷：v1 验证循环里的 ros2 lifecycle get 没加 timeout。
#   ROS2 CLI 的服务调用没有内建超时，服务端不响应就永久阻塞 ⇒ 光标停住不动。
#   ⇒ 所有 ros2 调用统一走带硬超时的 rcli()/has_tf()。
#
# 【顺带修掉的小坑】
#   · ros2 action send_goal 重定向到文件后 Python stdout 块缓冲 → 日志长时间为空
#     ⇒ stdbuf -oL -eL + PYTHONUNBUFFERED=1
#   · ros2 topic pub --once 没有订阅者时会一直等 ⇒ 一律套 timeout
#   · bash 的 `local a=1 b=$((a+1))`：local 先全部展开再赋值，b 恒为 0（进度条全空）
#     ⇒ 拆行写
#   · bash 不会算浮点 ⇒ 用 awk 包一个 calc()
#
# ============================================================================
# §C 内置方块是怎么算出来的（可复现，别手改）
#   对 navbot_room.pgm 的障碍做距离变换，在「距障碍 >= 0.35m」的掩码上求最大正方形，
#   再用可分离滑动窗口最小值挑「最不贴墙」的位置：
#       净空 0.40m → 最大 1.10m     净空 0.35m → 最大 1.20m   ← 选这档
#       净空 0.30m → 最大 1.30m     净空 0.25m → 最大 1.40m
#   选 1.20m / 0.35m：车半径 0.20、inflation_radius 0.25，0.35m 比两者都宽，
#   路径既不贴墙也不会被膨胀区封死；再加边长会让净空掉到 0.25m 以下（贴边抖）。
#   物理位置：中间那根 2.6m 隔断墙的正上方、北墙下方那条开阔带；四角实测全 free。
# ============================================================================

set -e
# ★ 不开 set -u：容器里 source ROS 的 setup.bash 会引用未定义变量
#   （见 踩坑实测/03_沙箱无pty与set-u冲突.md）

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
MODE="auto"
ARGS=()
for a in "$@"; do
  case "$a" in
    --diag|diag)         MODE="diag"  ;;
    --clean-only|clean)  MODE="clean" ;;
    manual)              MODE="manual";;
    *)                   ARGS+=("$a") ;;
  esac
done

# 浮点计算用 awk（bash 只会整数）
calc() { awk "BEGIN{printf \"%.3f\", $1}"; }

SHAPE="point"
if [ "${ARGS[0]:-}" = "square" ]; then
  SHAPE="square"
  # ★ 注意 ${VAR:--0.55} 是「默认值 -0.55」的写法（:- 后面跟一个负号）
  SQ_SIDE="${ARGS[1]:-1.20}"
  SQ_CX="${ARGS[2]:--0.55}"
  SQ_CY="${ARGS[3]:-1.00}"
  FRAMES="${ARGS[4]:-0}"        # ★ 0 = 跑完为止（不设帧数上限），见下面 MAX_WALL
  FRAMES_FALLBACK=0
  for v in "$SQ_SIDE" "$SQ_CX" "$SQ_CY"; do
    case "$v" in
      ''|*[!0-9.+-]*) echo "❌ 方形参数必须是数字：边长/中心x/中心y = $SQ_SIDE $SQ_CX $SQ_CY"; exit 1 ;;
    esac
  done
  X0=$(calc "$SQ_CX - $SQ_SIDE/2"); X1=$(calc "$SQ_CX + $SQ_SIDE/2")
  Y0=$(calc "$SQ_CY - $SQ_SIDE/2"); Y1=$(calc "$SQ_CY + $SQ_SIDE/2")
  # 从离车最近的角（右下）出发，逆时针绕一圈，最后回到起点角 → 成环
  LEG_X=(); LEG_Y=()
  LEG_X[1]="$X1"; LEG_Y[1]="$Y0"
  LEG_X[2]="$X1"; LEG_Y[2]="$Y1"
  LEG_X[3]="$X0"; LEG_Y[3]="$Y1"
  LEG_X[4]="$X0"; LEG_Y[4]="$Y0"
  LEG_X[5]="$X1"; LEG_Y[5]="$Y0"
  TOTAL=5
else
  GOAL_X="${ARGS[0]:-1.5}"
  GOAL_Y="${ARGS[1]:-1.0}"
  FRAMES="${ARGS[2]:-60}"
  FRAMES_FALLBACK=60
  LEG_X=(); LEG_Y=()
  LEG_X[1]="$GOAL_X"; LEG_Y[1]="$GOAL_Y"
  TOTAL=1
fi

INTERVAL="${INTERVAL:-2}"
GIF_DELAY="${GIF_DELAY:-20}"
GIF_SCALE="${GIF_SCALE:-800x500}"
FORCE="${FORCE:-0}"
# ★ 初始位姿默认值 = 实测「Gazebo 世界 (0,0) 在 map 上的位置」，见文件头 v4 修复
INIT_X="${INIT_X:--0.50}"
INIT_Y="${INIT_Y:--0.50}"

case "$FRAMES" in ''|*[!0-9]*) FRAMES="$FRAMES_FALLBACK" ;; esac
# ★ FRAMES=0 表示「跑完为止」：不设帧数上限，直到所有段结束 + 3 帧收尾
#   为什么需要它：Gazebo 跟不上真实时间（RTF < 1），车是按**仿真时间**走的，
#   而截图是按**真实时间**拍的 —— 同一段导航，墙上要花的秒数是仿真时间的 1/RTF 倍。
#   按帧数硬卡的下场就是「上限到了、可第 1 段才刚跑完，后面 4 段压根没发出去」。
MAX_WALL="${MAX_WALL:-1800}"          # 跑完为止模式下的总时长兜底（秒）
STALL_GIVEUP="${STALL_GIVEUP:-150}"   # 剩余距离连续这么多帧不降就放弃后面的段

WORKDIR="/mnt/workspace"
OUT_DIR="$WORKDIR/screenshots"
OUT_GIF="$WORKDIR/navigation.gif"

T_LC=6          # 单次 ros2 lifecycle 调用硬超时（正常 <1s 就回）
T_CLI=12        # 其它 ros2 命令硬超时
WAIT_NAV2=150   # 等 Nav2 全部 active 的上限

# ★ 必须全部 active 的 5 个：缺一个导航就不动
CRITICAL_NODES=(map_server amcl planner_server controller_server bt_navigator)
# ★ 手动模式分两批激活：先建 map 帧的前置，再其余（照 Nav2 官方顺序）
MANUAL_FIRST=(map_server amcl)
MANUAL_REST=(controller_server smoother_server planner_server behavior_server bt_navigator)

PIDS=()
DID_CLEAN=0
NO_CLEAN=0
KEEP_SCENE_ON_FAIL=1

# ---------------------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------------------
pbar() {  # pbar 当前 总数 标签
  local cur="$1" tot="$2" label="$3"
  [ "$tot" -le 0 ] && tot=1
  [ "$cur" -gt "$tot" ] && cur="$tot"
  # ★ 不能写成 `local w=24 filled=$(( cur * w / tot ))`：
  #   local 会先把所有右侧展开完再赋值，那时 w 还是空的，filled 恒为 0（进度条全空）。
  local w=24
  local filled bar="" i
  filled=$(( cur * w / tot ))
  for ((i = 0; i < w; i++)); do
    if [ "$i" -lt "$filled" ]; then bar="${bar}#"; else bar="${bar}."; fi
  done
  printf "\r  [%s] %3d%%  %s" "$bar" $(( cur * 100 / tot )) "$label"
}

# 进度显示：总数 >0 画进度条，=0（跑完为止模式）只打文本
prog() {
  local cur="$1" tot="$2"; shift 2
  if [ "$tot" -gt 0 ]; then pbar "$cur" "$tot" "$*"; else printf '\r  %s' "$*"; fi
}

fmt_sec() {
  local s="$1"
  if [ "$s" -ge 60 ]; then printf '%dm%02ds' $(( s / 60 )) $(( s % 60 )); else printf '%ds' "$s"; fi
}

# ★★ 所有 ros2 调用的唯一出口：硬超时 + 吞错。永远不挂死。
rcli() { timeout -k 3 "$T_CLI" ros2 "$@" 2>/dev/null || true; }

lc_state() { timeout -k 2 "$T_LC" ros2 lifecycle get "/$1" 2>/dev/null | head -1 || true; }

is_active() { case "$(lc_state "$1")" in *active*) return 0 ;; *) return 1 ;; esac; }

node_present() { timeout -k 3 "$T_CLI" ros2 node list 2>/dev/null | grep -qx "/$1"; }

wait_topic() {  # wait_topic 上限秒数 话题名
  local limit="$1" name="$2" i
  for ((i = 0; i < limit; i++)); do
    if timeout -k 3 "$T_CLI" ros2 topic list 2>/dev/null | grep -qx "$name"; then return 0; fi
    sleep 1
  done
  return 1
}

has_tf() {  # has_tf 父帧 子帧
  local out
  out=$(timeout -k 2 5 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 || true)
  echo "$out" | grep -q "Translation"
}

# Gazebo Classic 的 master 端口是不是被占着（残留进程没清干净的信号）
# ★ 不依赖 ss/netstat，直接用 bash 的 /dev/tcp 探
port_gazebo_busy() {
  (exec 3<>/dev/tcp/127.0.0.1/11345) 2>/dev/null || return 1
  return 0
}

# ---------------------------------------------------------------------------
# 清理
# ---------------------------------------------------------------------------
kill_all() {
  local pats="gzserver gzclient rviz2 planner_server controller_server bt_navigator \
amcl map_server smoother_server behavior_server waypoint_follower velocity_smoother \
lifecycle_manager spawn_entity robot_state_publisher Xvfb"
  local p i
  for p in $pats; do pkill -f "$p" 2>/dev/null || true; done
  pkill -f "ros2 launch" 2>/dev/null || true
  pkill -f "ros2 action send_goal" 2>/dev/null || true

  # ★★ 必须等它们真的死掉。Gazebo Classic 关得慢，不等就会出这种事：
  #    残留 gzserver 占着 master 端口 → 新的 gzserver 起得来但 /spawn_entity 不回应
  #    → 车永远生成不出来 → odom 帧不存在 → Nav2 一直刷 Invalid frame ID "odom"
  for i in 1 2 3 4 5; do
    if ! pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then break; fi
    sleep 1
  done
  if pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then
    echo "  ⚠️  Gazebo 没在 5 秒内退出，升级到 kill -9"
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gzclient 2>/dev/null || true
    sleep 2
  fi
  if pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then
    echo "  ⚠️  gzserver/gzclient 仍然存在，下面这次仿真很可能起不来"
    pgrep -af "gzserver|gzclient" 2>/dev/null | sed 's/^/      /' || true
  fi
  if port_gazebo_busy; then
    echo "  ⚠️  11345 端口（Gazebo master）仍被占用 —— 有残留进程没清掉"
  fi
  sleep 1
  return 0
}

do_clean() {
  echo ""
  echo "清理后台进程..."
  local p
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  kill_all
  echo "  完成。"
}

cleanup() {
  local rc=$?
  [ "$NO_CLEAN" -eq 1 ] && return 0
  [ "$DID_CLEAN" -eq 1 ] && return 0
  if [ "$rc" -ne 0 ] && [ "$KEEP_SCENE_ON_FAIL" -eq 1 ]; then
    echo ""
    echo "⚠️  异常退出（rc=$rc）—— 故意保留现场，方便你手动排查（Gazebo/Nav2 还在跑）。"
    echo "    收工清理：bash record_navigation.sh --clean-only"
    return 0
  fi
  DID_CLEAN=1
  do_clean
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# 现场诊断：只读。卡住时另开一个终端跑：bash record_navigation.sh --diag
# ---------------------------------------------------------------------------
dump_diag() {
  echo ""
  echo "-------------------- 现场诊断 --------------------"
  local nl st n t tl pair ap
  nl=$(timeout -k 3 "$T_CLI" ros2 node list 2>/dev/null || true)

  echo "· Nav2 关键节点："
  for n in map_server amcl planner_server controller_server bt_navigator; do
    if echo "$nl" | grep -qx "/$n"; then
      st=$(lc_state "$n")
      [ -z "$st" ] && st="⚠️ get_state 无响应（多半卡在 configuring/activating 中间态）"
      printf '    %-34s %s\n' "/$n" "$st"
    else
      printf '    %-34s %s\n' "/$n" "❌ 节点不存在（进程没起来 / 已崩 / 名字不对）"
    fi
  done
  # ★ lifecycle_manager 不是 lifecycle 节点，它没有 get_state 服务 ——
  #   所以只查"在不在"，再看它有没有报过 "Managed nodes are active"。
  #   （v6 及以前误查它的状态，永远显示"服务无响应"，是误报）
  for n in lifecycle_manager_localization lifecycle_manager_navigation; do
    if echo "$nl" | grep -qx "/$n"; then
      if grep -q "Managed nodes are active" /tmp/nav.log 2>/dev/null; then
        printf '    %-34s %s\n' "/$n" "✅ 在（nav.log 已有 'Managed nodes are active'）"
      else
        printf '    %-34s %s\n' "/$n" "✅ 在（还没报 'Managed nodes are active'）"
      fi
    else
      printf '    %-34s %s\n' "/$n" "❌ 节点不存在"
    fi
  done

  echo "· ★ 仿真层（缺 odom/base_footprint 帧的锅都在这儿）："
  if grep -q "Successfully spawned entity" /tmp/sim.log 2>/dev/null; then
    echo "    ✅ 车已生成：$(grep -o 'Successfully spawned entity \[[a-z_]*\]' /tmp/sim.log | tail -1)"
  else
    echo "    ❌ sim.log 里没有 'Successfully spawned entity' —— 车没生成（spawn 卡住/失败）"
  fi
  if timeout -k 3 "$T_CLI" ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then
    echo "    ✅ /spawn_entity 服务在（gazebo_ros_factory 已加载）"
  else
    echo "    ❌ /spawn_entity 服务不在（gazebo_ros_factory 没加载 / gzserver 卡住）"
  fi
  if pgrep -f "gzserver|gzclient" >/dev/null 2>&1; then
    echo "    ✅ gzserver 进程数：$(pgrep -c -f gzserver 2>/dev/null | head -1)"
  else
    echo "    ❌ 没有 gzserver 进程"
  fi
  if port_gazebo_busy; then
    if pgrep -f gzserver >/dev/null 2>&1; then
      echo "    （11345 由当前 gzserver 持有 —— 正常）"
    else
      echo "    ⚠️  11345 被占用但没有 gzserver 进程 —— 有残留进程占着 master 端口"
    fi
  else
    echo "    （11345 空闲 —— 没有 Gazebo 在跑）"
  fi

  echo "· ★ TF 关键变换（★ 看缺的是哪个帧，就知道该查哪一层）："
  for pair in "map odom" "odom base_footprint" "map base_footprint"; do
    if has_tf $pair; then
      echo "    ✅ $pair"
    else
      case "$pair" in
        "map odom")         echo "    ❌ $pair 拿不到  → AMCL 没发 map→odom（多半没给初值）→ 查 Nav2 层" ;;
        "odom base_footprint") echo "    ❌ $pair 拿不到  → 底盘插件没发 → 查仿真/硬件层（车生成了吗？）" ;;
        *)                  echo "    ❌ $pair 拿不到" ;;
      esac
    fi
  done

  echo "· 关键话题："
  tl=$(timeout -k 3 "$T_CLI" ros2 topic list 2>/dev/null || true)
  for t in /clock /map /scan /odom /cmd_vel /plan /amcl_pose /initialpose; do
    if echo "$tl" | grep -qx "$t"; then echo "    ✅ $t"; else echo "    ❌ $t"; fi
  done

  echo "· ★ AMCL 当前估计（车在 Gazebo (0,0) 时，这里若是 -0.5/-0.5 → 地图 origin 偏了 0.5m）："
  ap=$(timeout -k 2 6 ros2 topic echo --once /amcl_pose 2>/dev/null || true)
  if [ -n "$ap" ]; then echo "$ap" | grep -E "^[[:space:]]+(x|y|z):" | head -3 | sed 's/^/      /'; else echo "      (拿不到)"; fi

  echo "· /tmp/nav.log 尾部 40 行："
  if [ -s /tmp/nav.log ]; then tail -40 /tmp/nav.log | sed 's/^/    /'; else echo "    (空)"; fi
  echo "· /tmp/sim.log 尾部 15 行："
  if [ -s /tmp/sim.log ]; then tail -15 /tmp/sim.log | sed 's/^/    /'; else echo "    (空)"; fi
  echo "-------------------------------------------------"
}

# ---------------------------------------------------------------------------
# 仿真层没起来时的统一收尾：把「为什么」和「怎么办」一次说清，然后停下
# ---------------------------------------------------------------------------
sim_fail() {
  echo ""
  echo "  ❌ 仿真层没起来：$1"
  echo "     车没生成 ⇒ diff_drive 插件没加载 ⇒ /odom 和 odom→base_footprint 都不存在"
  echo "     ⇒ Nav2 会一直刷 'Invalid frame ID \"odom\" ... frame does not exist'"
  echo "     sim.log 尾部 25 行："
  tail -25 /tmp/sim.log 2>/dev/null | sed 's/^/       /' || true
  echo "     残留进程："
  if pgrep -f "gzserver|gzclient|spawn_entity" >/dev/null 2>&1; then
    pgrep -af "gzserver|gzclient|spawn_entity" 2>/dev/null | sed 's/^/       /' || true
  else
    echo "       （无）"
  fi
  if port_gazebo_busy; then
    echo "     ⚠️  11345 端口仍被占用（Gazebo master 被残留进程占着）"
  fi
  echo "     常见原因：上一次的 Gazebo 残留没清干净（抢 11345 端口）／容器负载过高"
  echo "     ★ 先单独把仿真层验通再回来（只干这一件事，看得更清）：bash check_sim.sh"
  echo "     急救：bash record_navigation.sh --clean-only  然后再跑一次"
  exit 1
}

# ---------------------------------------------------------------------------
# 手动激活单个节点（只在 manual 模式下用；带硬超时 + 状态复核）
# ---------------------------------------------------------------------------
manual_activate() {
  local n="$1" st
  st=$(lc_state "$n")
  case "$st" in
    *active*) echo "    /$n 已是 active，跳过"; return 0 ;;
  esac
  timeout -k 3 15 ros2 lifecycle set "/$n" configure >/dev/null 2>&1 || true
  sleep 1
  timeout -k 3 15 ros2 lifecycle set "/$n" activate  >/dev/null 2>&1 || true
  sleep 1
  st=$(lc_state "$n")
  case "$st" in
    *active*) echo "    ✅ /$n active";              return 0 ;;
    *)        echo "    ❌ /$n 仍是 ${st:-<无响应>}"; return 1 ;;
  esac
}

# ---------------------------------------------------------------------------
# 两种"不启动东西"的模式
# ---------------------------------------------------------------------------
if [ "$MODE" = "diag" ]; then
  NO_CLEAN=1
  echo "===== 体检模式（只读，不动任何进程）====="
  dump_diag
  exit 0
fi

if [ "$MODE" = "clean" ]; then
  echo "===== 只清理 ====="
  do_clean
  DID_CLEAN=1
  exit 0
fi

# ===========================================================================
# 正式流程
# ===========================================================================
echo "===== [1/9] 依赖检查 xvfb + imagemagick ====="
need_install=0
for c in Xvfb import convert; do
  if command -v "$c" >/dev/null 2>&1; then echo "  ✅ $c"; else echo "  ⚠️  缺 $c"; need_install=1; fi
done
if [ "$need_install" -eq 1 ]; then
  echo "  安装中（最多等 120s）..."
  timeout -k 5 120 bash -c 'apt-get install -y xvfb imagemagick >/dev/null 2>&1 || apt install -y xvfb imagemagick >/dev/null 2>&1 || true'
fi
command -v Xvfb   >/dev/null 2>&1 || { echo "❌ xvfb 装不上，无法继续"; exit 1; }
command -v import >/dev/null 2>&1 || { echo "❌ imagemagick(import) 装不上，无法继续"; exit 1; }
echo "  ✅ 依赖齐全"

echo "===== [2/9] 清理残留进程 ====="
kill_all
echo "  完成"

echo "===== [3/9] 起虚拟显示 :1 ====="
export PATH=/usr/bin:$PATH
Xvfb :1 -screen 0 1280x800x24 -nolisten tcp > /tmp/xvfb.log 2>&1 &
PIDS+=("$!")
export DISPLAY=:1
xdpy_ok=0
if command -v xdpyinfo >/dev/null 2>&1; then
  for _ in $(seq 1 20); do
    if xdpyinfo -display :1 >/dev/null 2>&1; then xdpy_ok=1; break; fi
    sleep 0.5
  done
  [ "$xdpy_ok" -eq 1 ] || { echo "  ❌ :1 不可用，看 /tmp/xvfb.log"; exit 1; }
else
  sleep 2   # 没有 xdpyinfo 就退回固定等待
fi
echo "  ✅ DISPLAY=:1 就绪"

echo "===== [4/9] 起仿真（★ 四道门，缺一道就不往下走）====="
[ -f /opt/ros/humble/setup.bash ]    || { echo "❌ 找不到 /opt/ros/humble/setup.bash，先跑 setup_env.sh"; exit 1; }
[ -f "$WORKDIR/install/setup.bash" ] || { echo "❌ 找不到 $WORKDIR/install/setup.bash，先 colcon build"; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKDIR/install/setup.bash"

if port_gazebo_busy; then
  echo "  ⚠️  11345 端口已被占用（Gazebo master）—— 有残留 gzserver，这次很可能起不来"
fi

GUI="${GUI:-false}"
ros2 launch navbot_bringup sim_navbot.launch.py gui:=$GUI > /tmp/sim.log 2>&1 &
PIDS+=("$!")
echo "  仿真已起，逐道过门..."

# ---- ① /spawn_entity 服务：车能不能生成，全看它 ----
echo "  ① 等 /spawn_entity 服务（上限 60s）..."
svc_ok=0
for _ in $(seq 1 60); do
  if timeout -k 3 "$T_CLI" ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then svc_ok=1; break; fi
  sleep 1
done
if [ "$svc_ok" -eq 1 ]; then
  echo "    ✅ /spawn_entity 就绪"
else
  sim_fail "gazebo_ros_factory 插件没加载 / gzserver 卡住（/spawn_entity 服务一直不出现）"
fi

# ---- ② spawn 成功：★ 这是「车真的生成了」的唯一判据 ----
echo "  ② 等 spawn 成功（上限 60s）..."
spawn_ok=0
for _ in $(seq 1 60); do
  if grep -q "Successfully spawned entity" /tmp/sim.log 2>/dev/null; then spawn_ok=1; break; fi
  sleep 1
done
if [ "$spawn_ok" -eq 1 ]; then
  echo "    ✅ $(grep -o 'Successfully spawned entity \[[a-z_]*\]' /tmp/sim.log | tail -1)"
else
  sim_fail "sim.log 里始终没有 'Successfully spawned entity'（spawn_entity 卡在 Calling service）"
fi

# ---- ③ 话题：/clock /scan /odom ----
sim_ok=1
for t in /clock /scan /odom; do
  if wait_topic 45 "$t"; then echo "    ✅ $t"; else echo "    ❌ $t 一直没出现"; sim_ok=0; fi
done
if [ "$sim_ok" -ne 1 ]; then
  sim_fail "车生成了但 /clock /scan /odom 不全（底盘/雷达插件没起来）"
fi

# ---- ④ TF：Nav2 真正需要的是 TF，不是话题 ----
echo "  ④ 校验 TF odom→base_footprint（上限 30s）..."
tf0=0
for _ in $(seq 1 6); do
  if has_tf odom base_footprint; then tf0=1; break; fi
  sleep 3
done
if [ "$tf0" -eq 1 ]; then
  echo "    ✅ odom→base_footprint 已发布"
else
  sim_fail "话题都在但 odom→base_footprint 查不到（diff_drive 插件没发 TF）"
fi

echo "  ✅ 仿真层四道门全过（车已生成 + odom 帧存在）"

echo "===== [5/9] 起导航（RViz）+ 等 map_server / amcl ====="
AUTOSTART=true
[ "$MODE" = "manual" ] && AUTOSTART=false
ros2 launch navbot_bringup nav_bringup.launch.py \
  use_sim_time:=true use_rviz:=true autostart:=$AUTOSTART > /tmp/nav.log 2>&1 &
PIDS+=("$!")
echo "  导航已起（autostart=$AUTOSTART）"

# ★★ 这里只等 map_server + amcl，绝不等 planner_server！
#    planner_server 的 global_costmap 在拿到 map→base_footprint 之前，
#    只会一直刷 'Invalid frame ID "map" ... frame does not exist'，等它 = 死等。
echo "  等 map_server + amcl 出现（每个上限 60s）..."
for n in map_server amcl; do
  found=0
  for _ in $(seq 1 30); do
    if node_present "$n"; then found=1; break; fi
    sleep 2
  done
  if [ "$found" -eq 1 ]; then
    echo "    ✅ /$n"
  else
    echo "    ❌ /$n 没出现"
    dump_diag
    exit 1
  fi
done

if [ "$MODE" = "manual" ]; then
  echo "  手动模式：先激活 map_server + amcl（autostart 已关，不会撞车）"
  for n in "${MANUAL_FIRST[@]}"; do manual_activate "$n" || true; done
else
  echo "  自动模式：交给 Nav2 的 lifecycle_manager（脚本不干预，避免请求撞车）"
  for _ in $(seq 1 30); do
    if is_active amcl; then break; fi
    sleep 2
  done
fi
echo "    /amcl 当前状态：$(lc_state amcl)"

echo "===== [6/9] ★ 先建 map 帧：给 AMCL 初始位姿 ====="
echo "  为什么这一步必须排在 planner_server 前面（实测结论，别调顺序）："
echo "    · map 帧不来自 map_server —— 它只发 /map 话题，不广播任何 TF"
echo "    · map 帧来自 AMCL 的 map→odom 广播，而 AMCL 是 set_initial_pose: false"
echo "    · 不给初值 ⇒ 没有 map 帧 ⇒ global_costmap 拿不到位姿"
echo "      （'Invalid frame ID \"map\"'）⇒ planner_server 起不来、RViz 疯狂丢消息"
echo "  初值 ($INIT_X, $INIT_Y)：这是「Gazebo 世界(0,0) 在地图上的落点」实测值。"
echo "    地图 yaml 的 origin 偏了 0.5m，所以不是 0/0；可用 INIT_X / INIT_Y 覆盖。"

INIT_POSE="{header: {frame_id: 'map'}, pose: {pose: {position: {x: $INIT_X, y: $INIT_Y, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0.0,0.0,0.0,0.0,0.0, 0.0,0.25,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.07]}}"
tf_ok=0
attempt=0
for attempt in $(seq 1 8); do
  # ★ 重发而不是只发一次：AMCL 可能刚 active、订阅还没热起来
  # ★ --once 会等订阅者，RViz 没起来时可能一直挂着 → 套 10s 硬超时
  timeout -k 3 10 ros2 topic pub --once /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped "$INIT_POSE" > /tmp/initpose.log 2>&1 || true
  if has_tf map odom; then tf_ok=1; break; fi
  pbar "$attempt" 8 "第 $attempt 次发初值，等 map→odom"
  sleep 2
done
echo
if [ "$tf_ok" -eq 1 ]; then
  echo "    ✅ map→odom 已建立（第 $attempt 次尝试成功），map 帧有了"
else
  echo "    ❌ 拿不到 map→odom —— AMCL 没接受初值 / 没有 /scan / 地图没加载"
  echo "       此时 planner_server 必然起不来、发目标必然不动，所以停在这里"
  if [ "$FORCE" != "1" ]; then
    echo "       想录一段「失败现场」：FORCE=1 bash record_navigation.sh ${ARGS[*]}"
    dump_diag
    exit 1
  fi
  echo "       FORCE=1 → 继续往下跑（会录到车不动的画面）"
fi

echo "===== [7/9] 等 planner_server / controller_server / bt_navigator 就绪 ====="
if [ "$MODE" = "manual" ]; then
  for n in "${MANUAL_REST[@]}"; do
    if node_present "$n"; then manual_activate "$n" || true; fi
  done
fi

# ★ 这里改成「等日志」而不是「每 3 秒给 5 个节点各查一次 lifecycle」：
#   后者每一轮都会起 5 个短命的 ros2 进程（= 5 个 DDS 参与者），在开目标前制造图抖动；
#   实测到过因此丢掉 action 响应：
#     [bt_navigator] Failed to send goal response ... (timeout): client will not receive response
#   日志法零 DDS 开销，一次 grep 就够。
navactive() { grep -qE "lifecycle_manager_navigation.*Managed nodes are active" /tmp/nav.log 2>/dev/null; }
deadline=$((SECONDS + WAIT_NAV2))
while [ "$SECONDS" -lt "$deadline" ]; do
  if navactive; then break; fi
  prog 0 0 "等 lifecycle_manager 报 'Managed nodes are active'……已用 $(fmt_sec "$SECONDS")"
  sleep 3
done
echo
if navactive; then
  echo "    ✅ lifecycle_manager：Managed nodes are active"
else
  echo "    ⚠️  ${WAIT_NAV2}s 内没等到 'Managed nodes are active'，下面逐个复核"
fi

# 复核一遍（只跑一次，5 个进程）
allok=1
for n in "${CRITICAL_NODES[@]}"; do
  st=$(lc_state "$n")
  case "$st" in
    *active*) echo "    ✅ /$n $st" ;;
    "")       echo "    ❌ /$n <无响应>"; allok=0 ;;
    *)        echo "    ❌ /$n $st";      allok=0 ;;
  esac
done
if [ "$allok" -ne 1 ]; then
  echo "  ❌ Nav2 没全部 active —— 停止录屏，先看下面这份现场"
  echo "     ★ 若卡在 planner_server：回头确认 [6/9] 的 map→odom 是否真的成功，"
  echo "       再看 /tmp/nav.log 有没有在刷 'Invalid frame ID \"map\"'。"
  dump_diag
  exit 1
fi
echo "  ✅ Nav2 全部 active（含 map 帧），用时 $(fmt_sec "$SECONDS")"

# ===========================================================================
echo "===== [8/9] 发目标点 + 截图 ====="
mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/frame_*.png
rm -f /tmp/goal_*.log

# 每段一个独立日志；stdbuf + PYTHONUNBUFFERED 防块缓冲（否则 grep 会误判"没被接受"）
# ★ --feedback：让日志里带出 distance_remaining / number_of_recoveries，
#   进度行才能显示"还剩几米"，也才能区分"慢慢在走"和"卡死了"
send_leg() {
  local i="$1" x="$2" y="$3" msg
  msg="{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {w: 1.0}}}}"
  if command -v stdbuf >/dev/null 2>&1; then
    PYTHONUNBUFFERED=1 stdbuf -oL -eL ros2 action send_goal --feedback /navigate_to_pose \
      nav2_msgs/action/NavigateToPose "$msg" > "/tmp/goal_$i.log" 2>&1 &
  else
    PYTHONUNBUFFERED=1 ros2 action send_goal --feedback /navigate_to_pose \
      nav2_msgs/action/NavigateToPose "$msg" > "/tmp/goal_$i.log" 2>&1 &
  fi
  PIDS+=("$!")
}
leg_done()  { grep -q "Goal finished with status" "/tmp/goal_$1.log" 2>/dev/null; }
leg_state() { grep -oE 'Goal finished with status: [A-Z]+' "/tmp/goal_$1.log" 2>/dev/null | tail -1 | awk '{print $NF}'; }

# ★ 下面两个靠 --feedback：ros2 action send_goal 加了 --feedback 后，日志里会周期性
#   带出 NavigateToPose 的 feedback（distance_remaining / number_of_recoveries）。
#   有了它，进度行才能显示"还剩几米"，也才能判断"车是在慢慢走还是卡死了"。
leg_dist()       { grep -oE 'distance_remaining: [0-9.eE+-]+' "/tmp/goal_$1.log" 2>/dev/null | tail -1 | awk '{print $2}'; }
leg_recoveries() { grep -oE 'number_of_recoveries: [0-9]+' "/tmp/goal_$1.log" 2>/dev/null | tail -1 | awk '{print $2}'; }

# 发一段并确保「被接受」；收不到响应就重发（最多 3 次）
# ★ 为什么要重试：实测撞到过 bt_navigator 的
#     Failed to send goal response c21e... (timeout): client will not receive response
#   也就是目标响应在传输层丢了（图里刚经历大量短命进程的增删、或负载过高时会发生）。
#   这不是"导航失败"——重发一次通常就过（新客户端 + 图已经热了）。
#   而且重发本身是安全的：Nav2 收到新目标会抢占（preempt）上一个。
LAST_GOAL_PID=""
send_leg_wait() {   # 段号 x y → 0=已确认被接受；1=三次都没成
  local i="$1" x="$2" y="$3" attempt accepted
  for attempt in 1 2 3; do
    send_leg "$i" "$x" "$y"
    LAST_GOAL_PID="${PIDS[${#PIDS[@]} - 1]}"
    accepted=0
    for _ in $(seq 1 15); do
      if grep -qi "accepted" "/tmp/goal_$i.log" 2>/dev/null; then accepted=1; break; fi
      sleep 1
    done
    if [ "$accepted" -eq 1 ]; then
      [ "$attempt" -gt 1 ] && echo "    ✅ 第 $attempt 次重发被接受"
      return 0
    fi
    echo "    ⚠️  15s 内没收到 accepted"
    if grep -q "Failed to send goal response" /tmp/nav.log 2>/dev/null; then
      echo "       nav.log 里确认有 'Failed to send goal response ... client will not receive response'"
      echo "       —— 是响应在传输层丢了，不等于目标没被接受；重发一次看"
    fi
    kill "$LAST_GOAL_PID" 2>/dev/null || true      # 收掉这个失联的客户端
    if [ "$attempt" -lt 3 ]; then echo "    ↻ 重发"; fi
  done
  return 1
}

# 量一下「仿真时间推进 / 真实时间」的比值（RTF）
# ★ 这个数直接解释「为什么一段 1.2m 的导航在墙上要几分钟」：
#   车按仿真时间走，截图按真实时间拍，墙上时长 = 仿真时长 / RTF
RTF=""
report_rtf() {
  local s1 s2 t1 t2 wall
  s1=$(timeout -k 2 8 ros2 topic echo --once /clock 2>/dev/null | awk '/^ *sec:/{print $2; exit}')
  [ -z "$s1" ] && return 0
  t1=$SECONDS
  sleep 6
  s2=$(timeout -k 2 8 ros2 topic echo --once /clock 2>/dev/null | awk '/^ *sec:/{print $2; exit}')
  t2=$SECONDS
  [ -z "$s2" ] && return 0
  wall=$((t2 - t1))
  [ "$wall" -le 0 ] && return 0
  RTF=$(awk -v a="$s1" -v b="$s2" -v w="$wall" 'BEGIN{printf "%.2f", (b - a) / w}')
  awk -v r="$RTF" 'BEGIN{
    printf "  仿真时间 / 真实时间（RTF）≈ %s", r
    if (r < 0.6) printf "   ← ★ 仿真比真实时间慢约 %.1f 倍，导航在墙上也要慢这么多\n", 1 / r
    else printf "\n"
  }'
  return 0
}

if [ "$SHAPE" = "square" ]; then
  echo "  ★ 方形模式：边长 ${SQ_SIDE}m，中心 ($SQ_CX, $SQ_CY)，共 $((TOTAL - 1)) 段 + 最后回到起点角"
  echo "    角点顺序："
  for i in $(seq 1 "$TOTAL"); do
    if [ "$i" -eq "$TOTAL" ]; then
      printf '      %d. (%s, %s)   ← 回到起点，成环\n' "$i" "${LEG_X[$i]}" "${LEG_Y[$i]}"
    else
      printf '      %d. (%s, %s)\n' "$i" "${LEG_X[$i]}" "${LEG_Y[$i]}"
    fi
  done
else
  echo "  ★ 单点目标：($GOAL_X, $GOAL_Y)"
fi

echo "  先量一个数（约 8 秒，用来说明为什么这一段在墙上要花好几分钟）："
report_rtf
sleep 3     # 让图安静一下再开目标（前面刚起过一堆短命的 ros2 进程）

leg=1
echo "  → 第 $leg/$TOTAL 段：(${LEG_X[$leg]}, ${LEG_Y[$leg]})"
if send_leg_wait "$leg" "${LEG_X[$leg]}" "${LEG_Y[$leg]}"; then
  echo "    ✅ 第 1 段已被接受"
else
  echo "    ❌ 三次都没被接受，/tmp/goal_1.log 尾部 + nav.log 关键行："
  tail -8 /tmp/goal_1.log 2>/dev/null | sed 's/^/      /' || true
  grep -E "Failed to send goal response|action server|competing" /tmp/nav.log 2>/dev/null | tail -5 | sed 's/^/      /' || true
  dump_diag
  exit 1
fi

if [ "$FRAMES" -gt 0 ]; then
  echo "  开始截图：上限 $FRAMES 张，每 ${INTERVAL}s 一张（时间上限 $(fmt_sec $((FRAMES * INTERVAL)))）"
else
  echo "  开始截图：★ 跑完为止（不设帧数上限），每 ${INTERVAL}s 一张；总时长兜底 $(fmt_sec "$MAX_WALL")"
fi
echo "  一段走完自动发下一段；全部走完再多拍 3 帧收尾并提前退出"
idx=0
finished=0      # 1 = 全部结束或已放弃
tail=-1         # >=0 表示正在收尾，值是还差几帧
sent_max=1      # 已发出去的最大段号（区分「未发送」和「未结束」）
last_d=""       # 上一帧的剩余距离（卡住检测用）
stall_t0=0      # 本次「没进展」是从第几秒开始的
stall_n=0
while [ "$FRAMES" -eq 0 ] || [ "$idx" -lt "$FRAMES" ]; do
  idx=$((idx + 1))
  timeout -k 2 15 import -window root "$OUT_DIR/frame_$(printf %03d "$idx").png" 2>/dev/null || true
  cur_d=$(leg_dist "$leg")

  # ① 推进分段状态机
  if [ "$finished" -eq 0 ] && leg_done "$leg"; then
    st=$(leg_state "$leg")
    [ -z "$st" ] && st="未知"
    rec=$(leg_recoveries "$leg")
    echo ""
    echo "  ✅ 第 $leg/$TOTAL 段结束：$st${rec:+（恢复行为 $rec 次）}"
    if [ "$leg" -ge "$TOTAL" ]; then
      if [ "$SHAPE" = "square" ]; then
        echo "  🏁 方形成环完成（回到起点角）"
      else
        echo "  🏁 导航结束"
      fi
      finished=1
    elif [ "$st" = "SUCCEEDED" ]; then
      leg=$((leg + 1))
      sent_max="$leg"
      last_d=""; stall_n=0; stall_t0=$SECONDS   # 换了目标，卡住检测重新计数
      echo "  → 第 $leg/$TOTAL 段：(${LEG_X[$leg]}, ${LEG_Y[$leg]})"
      if ! send_leg_wait "$leg" "${LEG_X[$leg]}" "${LEG_Y[$leg]}"; then
        echo "    ❌ 第 $leg 段三次都没发出去，放弃后面的段（已拍到的帧照样合成 GIF）"
        finished=1
      fi
    else
      echo "  ⚠️  第 $leg 段未成功（$st），后面的段不再发（已拍到的帧照样合成 GIF）"
      finished=1
    fi
  fi

  # ② 卡住检测：feedback 的 distance_remaining 长时间不降
  if [ "$finished" -eq 0 ]; then
    if [ -n "$cur_d" ]; then
      if [ -n "$last_d" ] && awk -v a="$cur_d" -v b="$last_d" 'BEGIN{exit !(a > b - 0.02)}'; then
        stall_n=$((stall_n + 1))
        [ "$stall_n" -eq 1 ] && stall_t0=$SECONDS
      else
        stall_n=0
      fi
      last_d="$cur_d"
    fi
    if [ "$stall_n" -eq 30 ]; then
      echo ""
      echo "  ⚠️  第 $leg 段剩余距离连续 $(fmt_sec $((SECONDS - stall_t0))) 没降（还剩 ${cur_d:-?}m）—— 车可能卡住了"
      echo "     想看原因：另开一个终端  bash record_navigation.sh --diag"
    fi
    if [ "$stall_n" -ge "$STALL_GIVEUP" ]; then
      echo ""
      echo "  ❌ 第 $leg 段连续 $(fmt_sec $((SECONDS - stall_t0))) 没有任何进展，放弃后面的段"
      finished=1
    fi
  fi

  # ③ 收尾：结束后再拍 3 帧
  if [ "$finished" -eq 1 ] && [ "$tail" -lt 0 ]; then tail=3; fi
  if [ "$tail" -ge 0 ]; then
    prog "$idx" "$FRAMES" "收尾中（还差 $tail 帧）  已跑 $(fmt_sec "$SECONDS")"
    if [ "$tail" -eq 0 ]; then echo ""; break; fi
    tail=$((tail - 1))
    sleep "$INTERVAL"
    continue
  fi

  # ④ 总时长兜底（无人值守时别无限跑）
  if [ "$SECONDS" -ge "$MAX_WALL" ]; then
    echo ""
    echo "  ⚠️  已跑满总时长上限 $(fmt_sec "$MAX_WALL")（可用 MAX_WALL=秒数 调整），提前收尾"
    finished=1; tail=3
    continue
  fi

  # ⑤ 进度显示
  dist_txt=""
  [ -n "$cur_d" ] && dist_txt="  剩余 ${cur_d}m"
  if [ "$FRAMES" -gt 0 ]; then
    prog "$idx" "$FRAMES" "第 $idx/$FRAMES 帧  第 $leg/$TOTAL 段${dist_txt}  已跑 $(fmt_sec "$SECONDS")"
  else
    prog "$idx" 0 "第 $idx 帧  第 $leg/$TOTAL 段${dist_txt}  已跑 $(fmt_sec "$SECONDS")"
  fi
  sleep "$INTERVAL"
done
echo

echo "===== [9/9] 合成 GIF ====="
n=$(ls -1 "$OUT_DIR"/frame_*.png 2>/dev/null | wc -l)
echo "  实际拿到 $n 帧"
if [ "$n" -eq 0 ]; then
  echo "❌ 一张都没截到 —— 是 Xvfb / import 的问题，不是 Nav2 的问题"
  dump_diag
  exit 1
fi
[ "$n" -lt 10 ] && echo "  ⚠️  帧太少（$n），GIF 会很短"

OPT=(-delay "$GIF_DELAY" -loop 0)
if [ "$GIF_SCALE" != "0" ]; then
  OPT+=(-resize "$GIF_SCALE")
  echo "  缩放：$GIF_SCALE（要原分辨率：GIF_SCALE=0）"
fi

echo "  合成中（已用 $(fmt_sec "$SECONDS")）..."
if timeout -k 5 300 convert "${OPT[@]}" "$OUT_DIR"/frame_*.png -layers Optimize "$OUT_GIF" 2>/dev/null; then
  ls -lh "$OUT_GIF"
else
  echo "  ⚠️  带 -layers Optimize 失败，退回普通合成..."
  timeout -k 5 300 convert "${OPT[@]}" "$OUT_DIR"/frame_*.png "$OUT_GIF" || { echo "❌ 合成失败"; exit 1; }
  ls -lh "$OUT_GIF"
fi

echo ""
echo "  导航结果（分段）："
ok_n=0
for i in $(seq 1 "$TOTAL"); do
  st=$(leg_state "$i")
  if [ -z "$st" ]; then
    if [ "$i" -gt "$sent_max" ]; then
      st="未发送（前面的段还没跑完，就到上限了）"
    else
      st="未结束（到上限时还在走）"
    fi
  fi
  if [ "$TOTAL" -gt 1 ]; then
    printf '    第 %d/%d 段 (%s, %s): %s\n' "$i" "$TOTAL" "${LEG_X[$i]}" "${LEG_Y[$i]}" "$st"
  else
    printf '    目标 (%s, %s): %s\n' "${LEG_X[$i]}" "${LEG_Y[$i]}" "$st"
  fi
  rec=$(leg_recoveries "$i")
  if [ -n "$rec" ] && [ "$rec" != "0" ]; then echo "        （恢复行为 $rec 次）"; fi
  if [ "$st" = "SUCCEEDED" ]; then ok_n=$((ok_n + 1)); fi
done
echo "    成功 $ok_n/$TOTAL 段"

if [ "$ok_n" -lt "$TOTAL" ]; then
  echo ""
  echo "  ★ 没跑完的段怎么办："
  if [ "$FRAMES" -gt 0 ]; then
    echo "    · 你这次显式限了帧数（$FRAMES）—— 去掉那个数字就是「跑完为止」（默认行为）"
  fi
  echo "    · Gazebo 跟不上真实时间（RTF${RTF:+ = $RTF}）是常态：车按仿真时间走、"
  echo "      截图按真实时间拍，所以整段导航在墙上要花好几分钟 —— 不是卡住了。"
  echo "    · 想让它看起来快一点：INTERVAL=3 或 4（帧间隔拉长，帧数不变、覆盖更长时间）"
  echo "    · 想省 GIF 体积：GIF_SCALE=640x400"
fi
echo ""
echo "✅ GIF：$OUT_GIF（用时 $(fmt_sec "$SECONDS")）"
echo "   下载查看，或 git add 推送后在 README 引用：![导航演示](navigation.gif)"
