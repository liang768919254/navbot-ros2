#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对拍脚本：chassis_bridge.py 的纯函数  vs  test_protocol.py 的参考实现

★ 为什么要对拍而不是直接 import？
  chassis_bridge.py 顶部 import 了 rclpy / serial / tf2_ros，
  直接 import 会连带把整个 ROS2 图拉起来（本机没有 ROS2 环境）。
  → 用 AST 只摘出「模块级函数定义」，在独立命名空间里 exec。

★ 对拍策略：不只比「同一个输入同一个输出」，还要比**等价性**：
  1. 随机穷举（10 万组）
  2. 边界值（0 / 负数 / 极值 / uint16 边界）
  3. 往返一致性（逆解 → 正解 回到原值）
  4. 物理锚点（走 1 米 = 10343 脉冲；41 脉冲/20ms = 198 mm/s）
  5. 位移积分 与 速度积分 的相互印证
"""

import ast
import math
import random
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def _find_ws_root(start):
    """逐级向上找第一个含 src/ 或 install/setup.bash 的目录。

    与 verify_e2e.sh 的探测规则一致，所以本脚本放在 <ws>/无硬件自测/、
    <ws>/tools/verify/ 或任何层级都能跑，不绑死目录深度。
    """
    d = start
    while True:
        if os.path.isdir(os.path.join(d, "src")) or \
           os.path.isfile(os.path.join(d, "install/setup.bash")):
            return d
        parent = os.path.dirname(d)
        if parent == d:            # 到根了
            return os.path.dirname(start)   # 兜底：退回上一级
        d = parent


ROOT = _find_ws_root(HERE)
BRIDGE = os.path.join(ROOT, "src/navbot_bridge/navbot_bridge/chassis_bridge.py")
REF = os.path.join(HERE, "test_protocol.py")

# ---------------------------------------------------------------------------
# 1) 用 AST 从 chassis_bridge.py 里摘出模块级函数（跳过 import rclpy 等）
# ---------------------------------------------------------------------------
WANTED = {
    "parse_odom_frame",
    "cmdvel_to_wheel_mms",
    "wheel_mms_to_cmdvel",
    "enc_delta_to_mms",
    "enc_to_rad",
    "wheel_mms_to_rad_s",
}


def load_module_funcs(path, wanted):
    """只执行模块级的 import math / 函数定义，不执行 Node 类与 main()"""
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)

    keep = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            # 只保留 math；rclpy / serial / tf2_ros 一律丢掉
            if isinstance(node, ast.Import):
                if any(a.name == "math" for a in node.names):
                    keep.append(node)
            else:
                if node.module == "math":
                    keep.append(node)
        elif isinstance(node, ast.Assign):
            # ★ 协议常量（ODOM_FIELDS / ODOM_HEADER ...）必须一起带过来
            keep.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in wanted:
            keep.append(node)

    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), path, "exec"), ns)
    got = {n for n in wanted if n in ns}
    missing = wanted - got
    if missing:
        raise SystemExit(f"[FATAL] {path} 缺少函数: {sorted(missing)}")
    return ns


# ---------------------------------------------------------------------------
# 2) 载入两边
# ---------------------------------------------------------------------------
bridge_ns = load_module_funcs(BRIDGE, WANTED)

# test_protocol.py 无外部依赖，直接 import
import importlib.util
spec = importlib.util.spec_from_file_location("test_protocol", REF)
ref = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ref)

R = 0.0325
PPR = 2112.0
L = 0.16

# ---------------------------------------------------------------------------
# 极简对拍框架
# ---------------------------------------------------------------------------
PASS = 0
FAIL = 0
FAILED = []


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
        FAILED.append(f"{name}  {detail}")
        print(f"  ✗ {name}   {detail}")


def section(t):
    print(f"\n[{t}]")


# ===========================================================================
section("1 · parse_odom_frame 逐字节对拍")
# ===========================================================================
good_lines = [
    "O,1234,-5820,5815,120,118,420,415,11850,184320,2510\n",
    "O,0,0,0,0,0,0,0,0,0,0\r\n",
    "O,65535,-32768,32767,-1000,1000,0,0,12000,4294967295,-5000\n",
    "O,1,2,3,4,5,6,7,8,9,10,11\n",
]
bad_lines = [
    "", "\n", "   \n", "O,1,2,3", "O,1,2,3,4,5,6,7,8,9,10,11,12",
    "X,1,2,3,4,5,6,7,8,9,10", "~~~GARBAGE### not a valid frame",
    "O,abc,2,3,4,5,6,7,8,9,10", "1,2,3,4,5,6,7,8,9,10,11",
    "O,1,2,3,4,5,6,7,8,9,", "OO,1,2,3,4,5,6,7,8,9,10",
    "o,1,2,3,4,5,6,7,8,9,10",            # 小写 o 必须拒
    "O,1.5,2,3,4,5,6,7,8,9,10",          # 浮点必须拒
    "O, 1,2,3,4,5,6,7,8,9,10",           # 前导空格 int 能过，两边必须一致
    "O,+1,+2,3,4,5,6,7,8,9,10",          # 带 + 号 int 能过，两边必须一致
    "O,0x10,2,3,4,5,6,7,8,9,10",         # 十六进制必须拒
    "O,1e3,2,3,4,5,6,7,8,9,10",          # 科学计数必须拒
]
for ln in good_lines + bad_lines:
    a = bridge_ns["parse_odom_frame"](ln)
    b = ref.parse_odom_frame(ln)
    tag = repr(ln[:40])
    check(f"parse {tag}", a == b, f"bridge={a} ref={b}")

# 随机字节流模糊测试（模拟乱码）
random.seed(20261007)
mismatch = 0
for _ in range(20000):
    n = random.randint(0, 30)
    alphabet = "0123456789,OVXo,\n\r .+-e"
    s = "".join(random.choice(alphabet) for _ in range(n))
    if bridge_ns["parse_odom_frame"](s) != ref.parse_odom_frame(s):
        mismatch += 1
check("随机乱码 20000 组 解析结果一致", mismatch == 0, f"{mismatch} 组不一致")


# ===========================================================================
section("2 · cmdvel_to_wheel_mms 逆解 对拍")
# ===========================================================================
mismatch = 0
worst = ""
for _ in range(100000):
    v = random.uniform(-3, 3)
    w = random.uniform(-10, 10)
    sep = random.choice([0.16, 0.2, 0.1, 0.3, 0.15])
    a = bridge_ns["cmdvel_to_wheel_mms"](v, w, sep)
    b = ref.cmdvel_to_wheel_mms(v, w, sep)
    if a != b:
        mismatch += 1
        worst = f"v={v} w={w} sep={sep} bridge={a} ref={b}"
check("随机 100000 组 逆解一致", mismatch == 0, worst or f"{mismatch} 组不一致")

# 边界：int() vs round() 的经典分歧点
# ★★ 注意 Python 的 round 是「banker's rounding（四舍六入五取偶）」，
#    round(0.5)=0、round(1.5)=2、round(2.5)=2 —— 不是小学数学的四舍五入。
#    这里只断言「两边一致」，期望值按 Python 真实语义填。
for v, w, expect_l, expect_r, note in [
    (-0.0006, 0.0, -1, -1, "round 给 -1，int() 截断会给 0 ★ 这就是必须用 round 的理由"),
    (-0.0004, 0.0, 0, 0, "接近零的负值 → 0"),
    (0.0005, 0.0, 0, 0, "★ banker: round(0.5)=0 而非 1"),
    (0.0015, 0.0, 2, 2, "round(1.5)=2"),
    (0.0025, 0.0, 2, 2, "★ banker: round(2.5)=2 而非 3"),
    (-0.0025, 0.0, -2, -2, "★ banker 负数对称"),
]:
    a = bridge_ns["cmdvel_to_wheel_mms"](v, w, L)
    b = ref.cmdvel_to_wheel_mms(v, w, L)
    check(f"边界 v={v} → ({expect_l},{expect_r})", a == b == (expect_l, expect_r),
          f"bridge={a} ref={b} 期望=({expect_l},{expect_r}) {note}")

# ★ 显式证明 int() 会在负数上出错（这是协议文档里那条警告的实证）
def _int_variant(v, omega, sep):
    return int((v - omega * sep / 2.0) * 1000), int((v + omega * sep / 2.0) * 1000)


vl_r, _ = bridge_ns["cmdvel_to_wheel_mms"](-0.0006, 0.0, L)
vl_i, _ = _int_variant(-0.0006, 0.0, L)
check("★ int() 截断 -0.6mm/s → 0（错误），round → -1（正确）",
      vl_i == 0 and vl_r == -1, f"int={vl_i} round={vl_r}")


# ===========================================================================
section("3 · wheel_mms_to_cmdvel 正解 对拍")
# ===========================================================================
mismatch = 0
worst = ""
for _ in range(100000):
    vl = random.randint(-3000, 3000)
    vr = random.randint(-3000, 3000)
    sep = random.choice([0.16, 0.2, 0.1, 0.3])
    a = bridge_ns["wheel_mms_to_cmdvel"](vl, vr, sep)
    b = ref.wheel_mms_to_cmdvel(vl, vr, sep)
    if a != b:
        mismatch += 1
        worst = f"vl={vl} vr={vr} sep={sep} bridge={a} ref={b}"
check("随机 100000 组 正解一致", mismatch == 0, worst or f"{mismatch} 组不一致")


# ===========================================================================
section("4 · enc_delta_to_mms 对拍 + 物理锚点")
# ===========================================================================
mismatch = 0
worst = ""
for _ in range(100000):
    d = random.randint(-500, 500)
    r = random.choice([0.0325, 0.05, 0.065, 0.1])
    ppr = random.choice([2112.0, 1024.0, 2048.0, 4096.0])
    dt = random.choice([0.02, 0.01, 0.05, 0.001, 1.0])
    a = bridge_ns["enc_delta_to_mms"](d, r, ppr, dt)
    b = ref.enc_delta_to_mms(d, r, ppr, dt)
    if (a - b) > 1e-9 * max(1.0, abs(a)):
        mismatch += 1
        worst = f"d={d} r={r} ppr={ppr} dt={dt} bridge={a} ref={b}"
check("随机 100000 组 编码器换算一致", mismatch == 0, worst or f"{mismatch} 组不一致")

mm_per_pulse = (2.0 * math.pi * R) / PPR * 1000
check("每脉冲弧长 ≈ 0.0967 mm", abs(mm_per_pulse - 0.0967) < 1e-3,
      f"{mm_per_pulse:.5f}")
v41 = bridge_ns["enc_delta_to_mms"](41, R, PPR, 0.02)
check("41 脉冲/20ms ≈ 198.3 mm/s", abs(v41 - 198.3) < 1.0, f"{v41:.2f}")
v0 = bridge_ns["enc_delta_to_mms"](0, R, PPR, 0.02)
check("0 脉冲 → 0.0（严格相等，不是 1e-17）", v0 == 0.0, f"{v0!r}")
vj = bridge_ns["enc_delta_to_mms"](2, R, PPR, 0.02)
check("2 脉冲抖动 ≈ 9.67 mm/s", abs(vj - 9.67) < 0.1, f"{vj:.3f}")


# ===========================================================================
section("5 · 往返一致性（逆解 → 正解 回到原值）")
# ===========================================================================
worst_v = worst_w = 0.0
for _ in range(20000):
    v0 = random.uniform(-1.0, 1.0)
    w0 = random.uniform(-3.0, 3.0)
    vl, vr = bridge_ns["cmdvel_to_wheel_mms"](v0, w0, L)
    v1, w1 = bridge_ns["wheel_mms_to_cmdvel"](vl, vr, L)
    worst_v = max(worst_v, abs(v1 - v0))
    worst_w = max(worst_w, abs(w1 - w0))
# ★ 理论界推导（不是拍的数）：
#   v  = 两轮均值 → 单轮错 0.5mm/s → |Δv| ≤ 0.0005 m/s
#   ω  = 两轮差 / L → 单轮错 1mm/s → |Δω| ≤ 0.001/0.16 = 0.00625 rad/s
check("往返 |Δv| ≤ 0.0005 m/s", worst_v <= 0.0005 + 1e-9, f"最大 {worst_v:.6f}")
check("往返 |Δω| ≤ 0.00625 rad/s（= 1mm/s ÷ L/2 的理论界）",
      worst_w <= 0.00625 + 1e-9, f"最大 {worst_w:.6f}")


# ===========================================================================
section("6 · 物理锚点：走 1 米 = 10343 脉冲")
# ===========================================================================
turns = 1.0 / (2.0 * math.pi * R)
pulses = turns * PPR
check(f"走 1m ≈ 10343 脉冲（实际 {pulses:.1f}）", 10300 < pulses < 10400,
      f"{pulses:.1f}")

# ★ 关键：用 bridge 自己的 enc_delta_to_mms 反推 —— 端到端验证单位链条
#   走 1 米耗时 5 秒 → 200mm/s；总脉冲 10343 分摊到 250 个 20ms 周期
#
#   ★ 注意这里的量化误差是**物理事实**，不是 bug：
#     10342.6 个脉冲要摊到 250 个周期，每周期只能是整数 → 41 个/周期
#     → 实际走了 41×250 = 10250 个脉冲 = 991.04 mm，少走 8.96 mm（-0.9%）
#     这就是「里程计必然有累计量化误差」的源头，
#     也是为什么真机上必须靠 /scan + AMCL 来修漂移。
n_ticks = 250
dt = 0.02
per_tick = int(round(pulses / n_ticks))
total = sum(
    bridge_ns["enc_delta_to_mms"](per_tick, R, PPR, dt)
    for _ in range(n_ticks)
)
dist_mm = total * dt
check("250 周期 × 41 脉冲 → 991.04 mm（量化误差 -0.9%，符合预期）",
      abs(dist_mm - 991.04) < 0.1, f"{dist_mm:.2f} mm")

# 用「总脉冲数」而不是「逐周期取整」的方式验单位链条（无量化误差）
exact_mm = pulses * mm_per_pulse
check("10342.6 脉冲 × 每脉冲弧长 → 1000.00 mm（单位链条正确）",
      abs(exact_mm - 1000.0) < 0.05, f"{exact_mm:.4f} mm")


# ===========================================================================
section("7 · /joint_states 换算（position 弧度 / velocity rad/s）")
# ===========================================================================
# 走 1 米，轮子应转 1/(2πR) 圈 = 4.8971 圈 = 30.77 rad
rad_per_m = 1.0 / R                       # 弧长 → 弧度：θ = s / r
check(f"走 1m → {rad_per_m:.4f} rad（= 1/r）", abs(rad_per_m - 30.769) < 0.01,
      f"{rad_per_m:.4f}")

# enc_to_rad 端到端：10343 个脉冲 → 应该约 30.77 rad
theta = bridge_ns["enc_to_rad"](round(pulses), R, PPR)
check("10343 脉冲 → ≈ 30.77 rad", abs(theta - 30.769) < 0.01, f"{theta:.4f}")

# 自洽性：enc_to_rad(enc) 与 enc*mm_per_pulse/1000/r 等价
for enc in [0, 1, -1, 10343, -10343, 32767]:
    a = bridge_ns["enc_to_rad"](enc, R, PPR)
    b = enc * mm_per_pulse / 1000.0 / R
    check(f"enc_to_rad 自洽 enc={enc}", abs(a - b) < 1e-12, f"{a} vs {b}")

# velocity: 200 mm/s 线速度 → rad/s
w_rad = bridge_ns["wheel_mms_to_rad_s"](200.0, R)
check("200 mm/s → 6.1538 rad/s", abs(w_rad - 200.0 / 1000.0 / R) < 1e-12,
      f"{w_rad:.4f}")
# 物理校验：线速度 200mm/s、轮半径 32.5mm → 角速度 = v/r
check("角速度 = v/r 物理一致",
      abs(w_rad * R - 0.2) < 1e-12, f"{w_rad * R:.6f}")


# ===========================================================================
section("8 · 位姿积分：直行 / 原地转 / 圆弧 三点物理校验")
# ===========================================================================
def simulate(v_cmd, w_cmd, duration, hz=50.0):
    """用 bridge 的逆解+正解+积分逻辑跑一遍纯运动学仿真（无延迟无噪声）"""
    ticks = int(duration * hz)
    dt = 1.0 / hz
    x = y = yaw = 0.0
    for _ in range(ticks):
        # 逆解（模拟 MCU 收到的目标）
        vl, vr = bridge_ns["cmdvel_to_wheel_mms"](v_cmd, w_cmd, L)
        # 正解（模拟 MCU 反馈）
        v, w = bridge_ns["wheel_mms_to_cmdvel"](vl, vr, L)
        d_center = v * dt
        d_yaw = w * dt
        yaw_mid = yaw + d_yaw / 2.0
        x += d_center * math.cos(yaw_mid)
        y += d_center * math.sin(yaw_mid)
        yaw += d_yaw
    return x, y, yaw


# 直行 0.2 m/s 跑 2 秒 → 前进 0.4 m，yaw=0
x, y, yaw = simulate(0.2, 0.0, 2.0)
check("直行 0.2m/s × 2s → x≈0.40 y≈0 yaw≈0",
      abs(x - 0.4) < 1e-3 and abs(y) < 1e-9 and abs(yaw) < 1e-9,
      f"x={x:.4f} y={y:.2e} yaw={yaw:.2e}")

# 原地转 0.5 rad/s 跑 2 秒 → yaw = 1.0 rad，位置不动
x, y, yaw = simulate(0.0, 0.5, 2.0)
check("原地转 0.5rad/s × 2s → yaw≈1.0 位置不动",
      abs(yaw - 1.0) < 1e-3 and abs(x) < 1e-6 and abs(y) < 1e-6,
      f"x={x:.2e} y={y:.2e} yaw={yaw:.4f}")

# 圆弧：v=0.2, ω=0.5 → 半径 R=v/ω=0.4，转一圈(ωt=2π)应回到起点
circ = 2 * math.pi / 0.5
x, y, yaw = simulate(0.2, 0.5, circ)
check(f"圆弧 v=0.2 ω=0.5 转整圈({circ:.2f}s) → 回起点 x≈0 y≈0 yaw≈2π",
      abs(x) < 5e-3 and abs(y) < 5e-3 and abs(yaw - 2 * math.pi) < 5e-3,
      f"x={x:.5f} y={y:.5f} yaw={yaw:.5f}")

# 圆弧半径校验：整圈的理论圆心距离
x, y, yaw = simulate(0.2, 0.5, 1.0)   # 只跑 1 秒，看半径
r_meas = math.hypot(x, y) if abs(yaw) > 1e-9 else 0.0
# 圆心在 (0, R_c)，1 秒后位置满足 x = R_c*sin(θ), y = R_c*(1-cos(θ))
R_c = 0.2 / 0.5
theta = 0.5 * 1.0
exp_x = R_c * math.sin(theta)
exp_y = R_c * (1 - math.cos(theta))
check(f"圆弧 1s 位置与解析解一致（圆心半径 {R_c}m）",
      abs(x - exp_x) < 1e-3 and abs(y - exp_y) < 1e-3,
      f"x={x:.5f}(exp {exp_x:.5f}) y={y:.5f}(exp {exp_y:.5f})")


# ===========================================================================
section("9 · 负向运动 / 对称性")
# ===========================================================================
x, y, yaw = simulate(-0.2, 0.0, 2.0)
check("倒退 0.2m/s × 2s → x≈-0.40", abs(x + 0.4) < 1e-3, f"x={x:.4f}")

x, y, yaw = simulate(0.0, -0.5, 2.0)
check("右转 ω=-0.5 × 2s → yaw≈-1.0", abs(yaw + 1.0) < 1e-3, f"yaw={yaw:.4f}")

# 逆解对称性：ω 取反 → 左右轮互换再取反
for v0, w0 in [(0.2, 0.5), (0.0, 1.0), (-0.15, 2.0)]:
    l1, r1 = bridge_ns["cmdvel_to_wheel_mms"](v0, w0, L)
    l2, r2 = bridge_ns["cmdvel_to_wheel_mms"](v0, -w0, L)
    check(f"ω 取反对称 v={v0} ω={w0}", (l1, r1) == (r2, l2),
          f"({l1},{r1}) vs ({l2},{r2})")


# ===========================================================================
section("10 · 恒定 cmd_vel 经逆解再正解 不应引入系统性偏差")
# ===========================================================================
worst = 0.0
worst_case = ""
for _ in range(5000):
    v0 = random.uniform(-1, 1)
    w0 = random.uniform(-3, 3)
    long_run = 10.0        # 10 秒
    ticks = int(long_run * 50)
    dt = 0.02
    x = y = 0.0
    vl, vr = bridge_ns["cmdvel_to_wheel_mms"](v0, w0, L)     # 只解一次
    v1, w1 = bridge_ns["wheel_mms_to_cmdvel"](vl, vr, L)
    err = abs(v1 - v0) + abs(w1 - w0)
    if err > worst:
        worst = err
        worst_case = f"v={v0:.4f}→{v1:.4f} w={w0:.4f}→{w1:.4f}"
check("逆解→正解 最大误差在取整理论界内", worst < 0.013, worst_case)


# ===========================================================================
print()
print("=" * 70)
print(f"  对拍汇总：{PASS} 通过 / {FAIL} 失败 / 共 {PASS + FAIL} 条")
if FAILED:
    print("  失败项：")
    for n in FAILED[:30]:
        print(f"    ✗ {n}")
print("=" * 70)
sys.exit(0 if FAIL == 0 else 1)
