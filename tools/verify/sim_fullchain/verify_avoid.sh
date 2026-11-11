#!/usr/bin/env bash
# ============================================================================
# test_avoid.sh —— 动态避障完整验证
# ============================================================================
# 流程：仿真 → Nav2 → AMCL 初值 → 车出发 → 路径中间插入地图上没有的障碍
#      → 观察是否检测到并绕开 → 是否仍到达目标
#
# 为什么障碍要「中途插入」而不是预先放在世界里：
#   预先放的话，建图时会把它画进地图，车只是「照图走」，证明不了避障能力。
#   中途插入的障碍在地图上不存在，只能靠局部代价地图 + 激光实时发现。
# ============================================================================
set +u
RUN_DIR="/home/liang/navbot_ws/tools/verify/sim_fullchain"
cd "$RUN_DIR" || exit 1

# ★ REUSE_SIM=1：复用外部已起的仿真（你自己终端里的 GUI 版）
REUSE_SIM="${REUSE_SIM:-0}"
if [ "$REUSE_SIM" = "1" ]; then
    bash stop_bg.sh stack >/dev/null 2>&1
    echo "==================== ① 复用已有仿真（REUSE_SIM=1）===================="
else
    bash stop_bg.sh all >/dev/null 2>&1
    sleep 2
    echo "==================== ① 启动仿真 ===================="
    bash run_bg.sh sim ros2 launch navbot_bringup sim_navbot.launch.py gui:=false
    sleep 14
fi
source "$RUN_DIR/env.sh"
for i in $(seq 1 20); do
    if timeout 3 ros2 topic echo /scan --once --field header >/dev/null 2>&1; then
        echo "  /scan OK"; break
    fi
    sleep 2
done

echo "==================== ② 启动 Nav2 ===================="
bash run_bg.sh nav ros2 launch navbot_bringup nav_bringup.launch.py \
    use_rviz:=false use_sim_time:=true
sleep 14

echo "==================== ③ 注入 AMCL 初值（--auto，带验证）===================="
# ★★ S34 改：不再用 odom_pose.py 读 /odom 当初值。
#   那条路的前提「map 与 odom 重合」只在冷启动成立，跑过导航后就偏 1.4 m。
#   set_pose.py --auto 会复合 map->odom 算出真正的 map 坐标，
#   并在注入后验证 map->odom 是平面变换，不行就重试，仍不行就返回非 0。
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

# ★★ S34 修：原来是 `timeout 6 ros2 lifecycle get`，超时返回空串，
#   于是把「查询慢」误判成「节点没 active」。
#   实测 bt_navigator 首次查询可能要 6s 以上（要发现 service）。
#   改成：带重试 + 从 20s 开始给足时间 + 失败时把三次读数都打出来。
lc_active() {
    local n="$1" i st
    for i in 1 2 3 4; do
        st=$(timeout 20 ros2 lifecycle get "/$n" 2>/dev/null | head -1)
        if echo "$st" | grep -q active; then
            echo "$st"; return 0
        fi
        sleep 2
    done
    echo "${st:-<超时>}"; return 1
}
for i in $(seq 1 10); do
    A=$(lc_active bt_navigator)   || A="${A:-?}"
    P=$(lc_active planner_server) || P="${P:-?}"
    if [[ "$A" == *active* && "$P" == *active* ]]; then
        echo "  Nav2 active (bt=$A planner=$P)"; break
    fi
    echo "  [$((i*4))s] bt=${A:-?} planner=${P:-?}"
    sleep 2
done

echo "==================== ③b 定位健康检查 ===================="
# ★★★ S34 修：原来这里用 shell 解析 `ros2 run tf2_ros tf2_echo map odom` 的文本。
#   那个写法有致命缺陷：tf2_echo 每 1 秒打一帧，12 秒打 9 帧，
#   head -20 会抓到 2~3 帧 → 提取出的 z 是多行字符串 → awk 判定失效，
#   却仍然打印出"-0.000"这种半行内容，看着像在工作。
#   ⇒ 改用 loc_check.py：rclpy 直接查 TF，拿到的是 float，没有解析歧义。
# ★ S34 修：同样有 `if ! cmd | sed` 的返回码问题 —— 判断的是 sed 而非 python。
LC_OUT=$(timeout 90 python3 loc_check.py 2>&1)
LC_RC=$?
echo "$LC_OUT" | sed 's/^/  /'
if [ "$LC_RC" -ne 0 ]; then
    echo "  !!! 定位不可信 (rc=$LC_RC)，终止（见上方报告）"
    exit 1
fi

echo "==================== ④ 避障测试 ===================="
# 目标点可传参：bash verify_avoid.sh [goal_x] [goal_y]
# ★★★ S34 修：默认值原来写死 2.7 / -2.0 —— 那是「障碍压在目标点上」的坏用例
#   （障碍 (2.88,-2.10) 与目标 (2.70,-2.00) 相距仅 0.206 m < 障碍半宽+车半宽 0.27 m）。
#   在那种几何下 P3「绕开」和 P4「到达」会同时假通过，测了等于没测。
#   ⇒ 默认值交还给 avoid_test.py（北走廊 -2.50 / 1.90，4.7 m 直线、净空恒 0.550 m）。
#   传入参数时仍按传入值走，且 avoid_test.py 里的 MIN_GOAL_SEP 闸门会作废非法几何。
# ★★★ S34 修（第二次在这个文件里踩同一个坑）：
#   原来 `timeout 320 python3 avoid_test.py | grep -vE "^$"` 的返回码是 grep 的，
#   不是 avoid_test 的。于是「测试 FAIL」和「测试 PASS」从脚本角度完全一样，
#   最后统一打印「test_avoid.sh 结束」，README 里说的判据形同虚设。
#   ⇒ 先把输出和返回码取下来，再决定收尾策略。
AVOID_OUT=$(if [ $# -ge 2 ]; then
                GOAL_X="$1" GOAL_Y="$2" timeout 320 python3 avoid_test.py 2>&1
            else
                timeout 320 python3 avoid_test.py 2>&1
            fi)
AVOID_RC=$?
echo "$AVOID_OUT" | grep -vE "^$"

echo "==================== 收尾 ===================="
# ★ 失败时**保留 Nav2 与 Gazebo**，方便你直接重跑或去 RViz 里看现场。
#   原来不分类一律 stop，失败后想复查还得重启一遍 Nav2 + 重注初值，
#   而重注初值正是上一轮踩过循环依赖的地方，没必要每次都赌。
if [ "$AVOID_RC" -eq 0 ]; then
    if [ "$REUSE_SIM" = "1" ]; then
        bash stop_bg.sh stack >/dev/null 2>&1   # 保留你的 Gazebo
    else
        bash stop_bg.sh all >/dev/null 2>&1
    fi
    echo "test_avoid.sh 结束（避障测试 PASS，环境已收）"
else
    echo "test_avoid.sh 结束（避障测试未通过，rc=$AVOID_RC）"
    echo "  ★ 已保留 Gazebo / Nav2 / RViz，方便你看现场。"
    echo "    直接重跑（复用现有仿真）："
    echo "      REUSE_SIM=1 bash tools/verify/sim_fullchain/verify_avoid.sh"
    echo "    彻底清理：bash tools/verify/sim_fullchain/stop_bg.sh all"
fi
exit $AVOID_RC
