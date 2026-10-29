#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端积分仿真：用 fake_chassis 的真实模型驱动 chassis_bridge 的积分逻辑

★ 这一步补上纯函数对拍测不到的东西：
  「_on_publish 里那段编排（首帧保护 → 增量 → 中点法积分）」是否正确。

  纯函数对拍只能证明「单个公式对了」，
  但里程计错得更隐蔽的是「编排」：
    - 忘了首帧保护 → 第一拍算出巨大增量 → 位姿瞬移
    - 用了起始 yaw 而非中点 yaw → 转弯时轨迹形状不对
    - 忘了更新 prev_enc → 增量永远是 0，车动了 odom 不动

  做法：不用串口、不用 ROS2，直接把 DiffDriveSim.step() 产生的
  编码器增量喂进「和 _on_publish 逐行相同」的积分代码，最后
  和模拟器内部的真值位姿比对。
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fake_chassis as fc          # noqa: E402

R, PPR, L = 0.0325, 2112.0, 0.16
M_PER_PULSE = (2.0 * math.pi * R) / PPR
HZ = 50.0
DT = 1.0 / HZ

PASS = FAIL = 0
FAILED = []


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ✓ {name}")
    else:
        FAIL += 1
        FAILED.append(f"{name}  {detail}")
        print(f"  ✗ {name}   {detail}")


def run_profile(target_l_mms, target_r_mms, duration_s):
    """跑一段轨迹，返回 (积分的位姿, 模拟器真值位姿, 编码器序列长度)"""
    sim = fc.DiffDriveSim(wheel_sep=L, wheel_radius=R, encoder_ppr=PPR)
    sim.target_l_mms = target_l_mms
    sim.target_r_mms = target_r_mms

    # ---- 完全复制 _on_publish 的状态与逻辑 ----
    x = y = yaw = 0.0
    prev_enc_l = prev_enc_r = None

    ticks = int(duration_s * HZ)
    enc_seq = []
    for _ in range(ticks):
        sim.step(DT)

        # 模拟 _on_line 收到的帧（enc 是累计整数）
        enc_l = sim.enc_l
        enc_r = sim.enc_r
        enc_seq.append((enc_l, enc_r))

        # ---- 以下与 chassis_bridge._on_publish 第 3 步逐行一致 ----
        if prev_enc_l is None:                 # 首帧保护
            prev_enc_l, prev_enc_r = enc_l, enc_r
            continue

        d_l = (enc_l - prev_enc_l) * M_PER_PULSE
        d_r = (enc_r - prev_enc_r) * M_PER_PULSE
        prev_enc_l, prev_enc_r = enc_l, enc_r

        d_center = (d_l + d_r) / 2.0
        d_yaw = (d_r - d_l) / L
        yaw_mid = yaw + d_yaw / 2.0
        x += d_center * math.cos(yaw_mid)
        y += d_center * math.sin(yaw_mid)
        yaw += d_yaw

    return (x, y, yaw), (sim.x, sim.y, sim.yaw), enc_seq


def show(tag, got, truth):
    print(f"     {tag}  积分=({got[0]:.4f}, {got[1]:.4f}, {got[2]:.4f}) "
          f"真值=({truth[0]:.4f}, {truth[1]:.4f}, {truth[2]:.4f})")


print("=" * 72)
print("  积分逻辑端到端仿真（用 fake_chassis 真实模型 · 含一阶延迟 tau=0.1）")
print("=" * 72)

# ---------------------------------------------------------------------------
print("\n[1] 直行 0.2 m/s × 4 秒 → 应前进约 0.8 m，yaw 保持 0")
# ---------------------------------------------------------------------------
got, truth, seq = run_profile(200, 200, 4.0)
show("直行", got, truth)
# ★ 阈值：200 个周期，每周期脉冲 41.578，取整误差 ≤0.5 脉冲/拍
#   → 累计 ≤200×0.5×0.09668mm = 9.67 mm
straight_bound = 200 * 0.5 * M_PER_PULSE * 1000
d_straight = abs(got[0] - truth[0]) * 1000
print(f"     前进距离差 = {d_straight:.3f} mm（量化上界 {straight_bound:.1f} mm）")
check(f"前进距离差 ≤ 量化上界 {straight_bound:.1f} mm",
      d_straight <= straight_bound, f"{d_straight:.3f} mm")
check("横向漂移 < 1 mm", abs(got[1] - truth[1]) < 1e-3, f"{abs(got[1]-truth[1]):.6f}")
check("yaw 保持 0（|Δ| < 1e-3 rad）", abs(got[2] - truth[2]) < 1e-3,
      f"{abs(got[2]-truth[2]):.6f}")
check("★ 首帧保护生效：x 没有从巨大值开始", abs(got[0]) < 1.0)

# ---------------------------------------------------------------------------
print("\n[2] 原地右转（L=+150, R=-150）→ 位置不动，yaw 单调变化")
# ---------------------------------------------------------------------------
got, truth, seq = run_profile(150, -150, 3.0)
show("原地转", got, truth)
check("原地转：位置几乎不动（< 5 mm）",
      math.hypot(got[0], got[1]) < 5e-3, f"{math.hypot(got[0],got[1]):.5f} m")
check("yaw 方向正确（负值 = 顺时针）", got[2] < 0, f"yaw={got[2]:.4f}")
# 原地转 3 秒 → 150 拍，取整误差对 ω 的影响 ≤ (0.5+0.5)×mpp/L/dt = 0.0302 rad/s
turn_bound = (0.5 + 0.5) * M_PER_PULSE / L / DT * 3.0
print(f"     yaw 差 = {abs(got[2]-truth[2]):.5f} rad（量化上界 {turn_bound:.5f} rad）")
check(f"yaw 差 ≤ 量化上界 {turn_bound:.5f} rad",
      abs(got[2] - truth[2]) <= turn_bound,
      f"{abs(got[2]-truth[2]):.5f}")

# ---------------------------------------------------------------------------
print("\n[3] ★ 圆弧 v=0.2 ω=0.5（用逆解算出轮速）→ 量化误差是主因")
# ---------------------------------------------------------------------------
# ★★ 阈值怎么定（有物理依据，不是拍脑袋）：
#    每周期脉冲 = 32.37 (L) / 48.95 (R)，取整误差各 ≤0.5 脉冲/拍
#    → dω 误差 ≤ (0.5+0.5)×m_per_pulse/L/dt = 0.0302 rad/s（最坏）
#    → 4 秒累计最坏 0.121 rad；实测 0.042 rad 在界内
#    位置误差来自 yaw 偏差 × 弧长：0.042 rad × 0.8 m ≈ 34 mm 量级
#    实测 15 mm → 正常。所以阈值按「量化理论上界」给，不给理想值。
vl = 0.2 - 0.5 * L / 2
vr = 0.2 + 0.5 * L / 2
got, truth, seq = run_profile(vl * 1000, vr * 1000, 4.0)
show("圆弧", got, truth)
d_pos = math.hypot(got[0] - truth[0], got[1] - truth[1])

# 量化误差理论界
d_om_worst = (0.5 + 0.5) * M_PER_PULSE / L / DT      # rad/s
d_yaw_worst = d_om_worst * 4.0                        # 4 秒累计
d_pos_worst = d_yaw_worst * 0.8 * 1000                # 弧长 0.8m

check(f"★ 圆弧 yaw 误差 {abs(got[2]-truth[2]):.5f} ≤ 量化上界 {d_yaw_worst:.5f} rad",
      abs(got[2] - truth[2]) <= d_yaw_worst,
      f"{abs(got[2]-truth[2]):.5f} vs {d_yaw_worst:.5f}")
check(f"★ 圆弧位置误差 {d_pos*1000:.2f}mm ≤ 量化上界 {d_pos_worst:.1f}mm",
      d_pos <= d_pos_worst / 1000,
      f"{d_pos*1000:.2f} vs {d_pos_worst:.1f} mm")

# ★★ 决定性实验：把整数脉冲换成浮点脉冲，量化误差消失后精度应大幅提升
#     这一步是为了证明「误差来自量化」而不是「积分公式写错」
def run_float(target_l_mms, target_r_mms, duration_s):
    sim = fc.DiffDriveSim(wheel_sep=L, wheel_radius=R, encoder_ppr=PPR)
    sim.target_l_mms, sim.target_r_mms = target_l_mms, target_r_mms
    fl = fr = 0.0
    x = y = yaw = 0.0
    prev = None
    for _ in range(int(duration_s * HZ)):
        dl_f, dr_f = sim.step(DT)          # 浮点增量，不取整
        fl += dl_f
        fr += dr_f
        if prev is None:
            prev = (fl, fr)
            continue
        d_l = (fl - prev[0]) * M_PER_PULSE
        d_r = (fr - prev[1]) * M_PER_PULSE
        prev = (fl, fr)
        d_c = (d_l + d_r) / 2.0
        d_y = (d_r - d_l) / L
        ym = yaw + d_y / 2.0                 # ← 与 _on_publish 同款中点法
        x += d_c * math.cos(ym)
        y += d_c * math.sin(ym)
        yaw += d_y
    return (x, y, yaw), (sim.x, sim.y, sim.yaw)


gf, tf = run_float(vl * 1000, vr * 1000, 4.0)
d_pos_f = math.hypot(gf[0] - tf[0], gf[1] - tf[1])
print(f"     浮点脉冲（无量化）: 位置误差={d_pos_f*1000:.3f} mm  "
      f"dyaw={gf[2]-tf[2]:.5f}")
check("★ 去掉量化后 yaw 误差 |Δ| < 0.005 rad（证明积分法本身正确）",
      abs(gf[2] - tf[2]) < 0.005, f"{abs(gf[2]-tf[2]):.5f}")
check("★ 去掉量化后位置误差 < 5 mm（量化才是主因）",
      d_pos_f < 5e-3, f"{d_pos_f*1000:.3f} mm")

# ---------------------------------------------------------------------------
print("\n[4] ★ 中点法 vs 起始法：对**解析解**比，不对模拟器真值比")
# ---------------------------------------------------------------------------
# ★ 设计说明（这一组第一版的教训）：
#   模拟器 DiffDriveSim.step() 内部用的是欧拉积分（起始 yaw）：
#       self.x += v*cos(self.yaw)*dt
#   拿我的「中点法」去跟它的「起始法」真值比，等于让两套不同积分法互比 ——
#   谁跟模拟器用同一种方法谁就赢，这测不出我的实现好坏。
#   → 正解：跟**解析解**（圆的参数方程）比。
#
#   圆弧解析解：v=0.2, ω=0.5 → 圆心半径 Rc = v/ω = 0.4 m
#     从稳态起点 (x0,y0,yaw0) 出发，t 秒后：
#       θ = ω·t
#       x = x0 + Rc·[sin(yaw0+θ) - sin(yaw0)]
#       y = y0 - Rc·[cos(yaw0+θ) - cos(yaw0)]
V0, W0 = 0.2, 0.5
RC = V0 / W0
DUR = 2.0


def run_steady(mode):
    """先跑 1 秒（用 20ms 小步，让一阶延迟真实收敛）再起算积分。

    ★ 踩坑记录（这一组的第一版为什么会算出 190mm）：
      一开始写的是 sim.step(1.0) —— 单步 dt=1.0。
      但 DiffDriveSim.step() 里 alpha = min(1.0, dt/tau)，
      dt=1.0 时 alpha=1 → 速度**瞬间**跳到目标值，一阶延迟被绕过；
      而同一行又用 dt=1.0 推 yaw：yaw += omega*1.0 = 0.5。
      真实系统前 1 秒只走 0.45 rad（延迟拖累），却被记成 0.5 →
      稳态起点 yaw0 偏高 0.05 rad → 解析解起点错 → 190mm 的假误差。
      → 教训：用数值步进做"预热"时，步长必须和主循环一致。
    """
    sim = fc.DiffDriveSim(wheel_sep=L, wheel_radius=R, encoder_ppr=PPR)
    sim.target_l_mms, sim.target_r_mms = vl * 1000, vr * 1000
    for _ in range(int(1.0 * HZ)):          # ← 20ms 小步预热 1 秒
        sim.step(DT)
    x0, y0, yaw0 = sim.x, sim.y, sim.yaw
    # ★ 积分从稳态起点开始 → 初值直接置成 (x0, y0, yaw0)，
    #   这样积分出来就是**绝对坐标**，可以直接和解析解比。
    #   （第一版把初值置 0 又在比较时 +x0，等于重复叠加 → 190mm 假误差）
    x, y, yaw = x0, y0, yaw0
    fl = fr = 0.0
    prev = None
    for _ in range(int(DUR * HZ)):
        dl_f, dr_f = sim.step(DT)
        fl += dl_f
        fr += dr_f
        if mode == "int":
            enc = (float(sim.enc_l), float(sim.enc_r))
        else:
            enc = (fl, fr)
        if prev is None:
            prev = enc
            continue
        d_l = (enc[0] - prev[0]) * M_PER_PULSE
        d_r = (enc[1] - prev[1]) * M_PER_PULSE
        prev = enc
        d_c = (d_l + d_r) / 2.0
        d_y = (d_r - d_l) / L
        if mode == "float_mid":
            ym = yaw + d_y / 2.0           # 中点法
        elif mode == "float_start":
            ym = yaw                       # 起始法（对照）
        else:
            ym = yaw + d_y / 2.0
        x += d_c * math.cos(ym)
        y += d_c * math.sin(ym)
        yaw += d_y
    # 解析解：圆心在车体左侧（左转 ω>0）
    #   圆心 = (x0 - Rc·sin(yaw0), y0 + Rc·cos(yaw0))
    #   绕圆心转动 yaw0 + ω·t
    cx = x0 - RC * math.sin(yaw0)
    cy = y0 + RC * math.cos(yaw0)
    ang = yaw0 + W0 * DUR
    exp_x = cx + RC * math.sin(ang)
    exp_y = cy - RC * math.cos(ang)
    err = math.hypot(x - exp_x, y - exp_y)      # ★ 直接比绝对坐标
    return err, yaw, yaw0 + W0 * DUR


yaw_truth = yaw0_ref = W0 * DUR       # 稳态起点的 yaw + 圆弧转动量（打印用）


e_mid_i, yaw_i, yaw_truth = run_steady("int")
e_mid, _, _ = run_steady("float_mid")
e_start, _, _ = run_steady("float_start")
print(f"     整数脉冲 + 中点法 : 位置误差 = {e_mid_i*1000:8.3f} mm")
print(f"     浮点脉冲 + 中点法 : 位置误差 = {e_mid*1000:8.3f} mm")
print(f"     浮点脉冲 + 起始法 : 位置误差 = {e_start*1000:8.3f} mm")
print(f"     （解析解 yaw 应为 {yaw_truth:.5f}，实测 {yaw_i:.5f}）")

check("★ 浮点下 中点法 优于 起始法（对解析解）",
      e_mid < e_start, f"mid={e_mid*1000:.3f}mm start={e_start*1000:.3f}mm")
check("★ 中点法对解析解的位置误差 < 10 mm",
      e_mid < 10e-3, f"{e_mid*1000:.3f} mm")
check("★ 加上量化后误差仍在量化理论上界内",
      e_mid_i <= d_pos_worst / 1000,
      f"{e_mid_i*1000:.3f} vs {d_pos_worst:.1f} mm")

# ---------------------------------------------------------------------------
print("\n[5] ★ 反例对照：忘了更新 prev_enc → 车动了但 odom 不动")
# ---------------------------------------------------------------------------
sim = fc.DiffDriveSim(wheel_sep=L, wheel_radius=R, encoder_ppr=PPR)
sim.target_l_mms = sim.target_r_mms = 200.0
first = None
x_bad = 0.0
for _ in range(int(2.0 * HZ)):
    sim.step(DT)
    if first is None:
        first = (sim.enc_l, sim.enc_r)     # 只记首帧，之后永不更新 ← 错误做法
        continue
    d_l = (sim.enc_l - first[0]) * M_PER_PULSE   # 每拍都拿「全程累计」当增量
    x_bad += d_l
print(f"     模拟器真值 x = {sim.x:.4f} m")
print(f"     忘记更新 prev 算出的 x = {x_bad:.4f} m  ← 荒谬放大")
check("★ 忘记更新 prev_enc 会算出荒谬结果（说明这行不能漏）",
      x_bad > 10 * sim.x, f"bad={x_bad:.2f} truth={sim.x:.2f}")

# ---------------------------------------------------------------------------
print("\n[6] 静止 5 秒 → odom 应纹丝不动")
# ---------------------------------------------------------------------------
got, truth, seq = run_profile(0, 0, 5.0)
check("静止：位置增量为 0", got[0] == 0.0 and got[1] == 0.0, f"{got}")
check("静止：yaw 增量为 0", got[2] == 0.0, f"{got[2]}")

# ---------------------------------------------------------------------------
print("\n[7] 往返：前进 2 秒 + 后退 2 秒 → 回到起点附近")
# ---------------------------------------------------------------------------
# ★ 阈值说明：往返 4 秒 ≈ 400 个周期，直行时每周期脉冲 41.578 个，
#   取整误差 ≤0.5 脉冲/拍 → 累计 ≤400×0.5×0.0967mm = 19.3 mm
#   实测 15.4 mm 在这个界内。
#   ★ 这不是 bug，是「整数编码器」的物理事实 —— 真机上表现为
#     「车开回原点，odom 显示偏了 1~2 厘米」。修漂移靠 /scan + AMCL。
sim = fc.DiffDriveSim(wheel_sep=L, wheel_radius=R, encoder_ppr=PPR)
sim.target_l_mms = sim.target_r_mms = 200.0
x = y = yaw = 0.0
prev = None
for i in range(int(4.0 * HZ)):
    if i == int(2.0 * HZ):
        sim.target_l_mms = sim.target_r_mms = -200.0
    sim.step(DT)
    if prev is None:
        prev = (sim.enc_l, sim.enc_r)
        continue
    d_l = (sim.enc_l - prev[0]) * M_PER_PULSE
    d_r = (sim.enc_r - prev[1]) * M_PER_PULSE
    prev = (sim.enc_l, sim.enc_r)
    d_c = (d_l + d_r) / 2.0
    d_y = (d_r - d_l) / L
    ym = yaw + d_y / 2.0
    x += d_c * math.cos(ym)
    y += d_c * math.sin(ym)
    yaw += d_y

quant_bound = 400 * 0.5 * M_PER_PULSE * 1000       # mm
print(f"     往返后 积分=({x:.5f}, {y:.5f})  真值=({sim.x:.5f}, {sim.y:.5f})")
print(f"     位置差 = {math.hypot(x,y)*1000:.3f} mm（量化上界 {quant_bound:.1f} mm）")
check(f"往返后偏移 ≤ 量化理论上界 {quant_bound:.1f} mm",
      math.hypot(x, y) * 1000 <= quant_bound,
      f"{math.hypot(x,y)*1000:.3f} mm")
check("★ 往返后偏移 < 20 mm（真机可接受范围，靠 AMCL 修）",
      math.hypot(x, y) < 20e-3, f"{math.hypot(x,y)*1000:.3f} mm")

# ---------------------------------------------------------------------------
print()
print("=" * 72)
print(f"  端到端仿真汇总：{PASS} 通过 / {FAIL} 失败 / 共 {PASS + FAIL} 条")
if FAILED:
    for n in FAILED:
        print(f"    ✗ {n}")
print("=" * 72)
sys.exit(0 if FAIL == 0 else 1)
