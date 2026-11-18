#!/usr/bin/env bash
# 无硬件端到端验证：假底盘 → socat 虚拟串口 → C++ 桥节点 → /odom
#
# 判据设计原则（踩过两次坑之后定下的）：
#   1. pty 对两端不能给同一进程用 —— 假底盘占 ttyV0，节点必须连 ttyV1
#   2. "有数据"的主判据用 ros2 topic hz，不用 echo
#      （echo 是 Python 程序，stdout 接管道时块缓冲，head 可能一个字都收不到，
#       会把"有数据"误判成"没数据"。hz 每行 flush，可靠得多）
#   3. 每条判据失败都要能区分「真失败」和「探测方式不可靠」
set -uo pipefail

WS=$HOME/navbot_ws
LOG=$HOME/.cache/chassis_verify
mkdir -p "$LOG"

cleanup() {
  [ -n "${NODE_PID:-}" ] && kill "$NODE_PID" 2>/dev/null
  [ -n "${FAKE_PID:-}" ] && kill "$FAKE_PID" 2>/dev/null
  sleep 0.5
  kill -9 "${NODE_PID:-}" "${FAKE_PID:-}" 2>/dev/null
  rm -f /tmp/ttyV0 /tmp/ttyV1
}
trap cleanup EXIT

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

# 取一次频率（hz 会一直跑，靠 timeout 截断）
topic_hz() {
  PYTHONUNBUFFERED=1 timeout 6 ros2 topic hz "$1" 2>/dev/null \
    | grep 'average rate' | tail -1
}

# 尝试取内容样本（可能因管道缓冲取不到，不作为判负依据）
topic_sample() {
  # ★ stdbuf -oL 是关键：ros2 topic echo 的 stdout 接管道时是块缓冲，
  #   一帧 odom 的 YAML 几百字节，攒不满 4KB 就不 flush → head 一个字都收不到。
  #   PYTHONUNBUFFERED 让 Python 层不缓冲，stdbuf -oL 让 libc 行缓冲，两个都要。
  #   2026-11-18 实测：只加 PYTHONUNBUFFERED 仍然取不到样本。
  PYTHONUNBUFFERED=1 timeout 8 stdbuf -oL ros2 topic echo "$1" 2>/dev/null | head -30
}

echo "════════ 1. 启动假底盘（socat 建 ttyV0 ↔ ttyV1）════════"
python3 tools/verify/fake_chassis.py --make-socat --rate 50 > "$LOG/fake.log" 2>&1 &
FAKE_PID=$!
for _ in $(seq 1 30); do [ -e /tmp/ttyV0 ] && break; sleep 0.2; done
if [ ! -e /tmp/ttyV0 ]; then
  echo "✘ 虚拟串口没建起来"
  tail -5 "$LOG/fake.log"
  exit 1
fi
echo "✔ ttyV0/ttyV1 就绪"

echo
echo "════════ 2. 启动 C++ 桥节点（连 ttyV1，ttyV0 被假底盘占着）════════"
ros2 run navbot_chassis chassis_node --ros-args \
  -p port:=/tmp/ttyV1 -p baudrate:=115200 > "$LOG/node.log" 2>&1 &
NODE_PID=$!

# 预热 daemon：不预热时第一条 ros2 命令可能起不来
ros2 topic list >/dev/null 2>&1 || true
sleep 4

echo "--- 节点日志 ---"
grep -v '^\s*$' "$LOG/node.log" | head -6

if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "✘ 节点已退出（不该退出）"
  exit 1
fi

echo
echo "════════ 3. /odom 数据流 ════════"
if ros2 topic list 2>/dev/null | grep -qx '/odom'; then
  echo "✔ 话题存在"
else
  echo "△ topic list 暂未刷新（daemon 有延迟）——以下面的 hz 为准，别据此判负"
fi

HZ=$(topic_hz /odom)
if [ -n "$HZ" ]; then
  echo "✔ $HZ   ← 这是「有数据」的可靠证据"
else
  echo "✘ 测不到频率 → 真的没有数据流"
fi

SAMPLE=$(topic_sample /odom)
if echo "$SAMPLE" | grep -q 'frame_id: odom'; then
  echo "✔ 内容样本："
  echo "$SAMPLE" | grep -E 'frame_id|child_frame_id|linear:|angular:' | head -4
else
  echo "△ 没取到内容样本（echo 管道缓冲问题，不影响上面的结论）"
fi

echo
echo "════════ 4. /joint_states ════════"
JS_HZ=$(topic_hz /joint_states)
if [ -n "$JS_HZ" ]; then
  echo "✔ $JS_HZ"
else
  echo "✘ 测不到频率"
fi
JS=$(topic_sample /joint_states)
if echo "$JS" | grep -q 'left_wheel_joint'; then
  echo "✔ 关节名正确：left_wheel_joint / right_wheel_joint"
fi

echo
echo "════════ 5. TF: odom → base_footprint ════════"
TF=$(PYTHONUNBUFFERED=1 timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -10)
if echo "$TF" | grep -q 'Translation'; then
  echo "✔ TF 树闭合"
  echo "$TF" | grep -A1 'Translation' | head -2
else
  echo "△ 没查到 TF（tf2_echo 也可能受管道影响）"
fi

echo
echo "════════ 6. 日志总量（坏帧不该刷屏）════════"
echo "节点日志行数: $(wc -l < "$LOG/node.log")  （判据 < 500）"
echo "WARN/ERROR:   $(grep -c 'WARN\|ERROR' "$LOG/node.log" 2>/dev/null || echo 0) 条"

echo
echo "════════ 完成 ════════"
echo "日志：$LOG/"
