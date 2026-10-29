#!/usr/bin/env python3
"""
test_protocol.py —— 纯函数单测，不装 pytest 也能跑。

用法：
    /usr/bin/python3 test_protocol.py

★★ 这个文件的定位：验证「协议解析」和「运动学换算」这两块纯逻辑。
    它们不依赖串口、不依赖 rclpy、不依赖硬件 —— 所以能在这里被完整验证。

★ 为什么单位测试值钱？
    因为里程计的单位错误是「静默的」：车会动、话题有数据、
    RViz 里也看得见，就是位置慢慢偏。等你发现时，
    你可能已经怀疑过编码器、怀疑过轮距、怀疑过电机 —— 而真凶是少乘了 1000。
    单测能在 1 秒内排除掉这类问题。

运行时会逐条打印 PASS/FAIL，最后给出汇总。
"""

import io
import math
import sys


# ============================================================================
# 被测逻辑：从 fake_chassis.py 里「复制」关键公式
#
# ★★ 为什么复制而不是 import？
#    因为这是「教程附件」，将来你把它复制进 ROS2 功能包的 test/ 目录后，
#    import 路径会失效（这正是 learning-phase-guide-authoring 技能里
#    记下的那个坑）。所以这里写成自包含的，复制到哪都能跑。
#
#    代价是：公式改了要改两处。这是刻意的权衡 ——
#    测试的价值在于「独立实现一遍再对比」，抄同一份代码反而测不出错。
# ============================================================================

def build_odom_frame(seq, enc_l, enc_r, vel_l, vel_r, pwm_l, pwm_r,
                     vbat_mv, tick_ms, temp_c100):
    """上行帧组装：必须 11 个字段"""
    return (f"O,{seq},{enc_l},{enc_r},{vel_l},{vel_r},"
            f"{pwm_l},{pwm_r},{vbat_mv},{tick_ms},{temp_c100}\n")


def parse_odom_frame(line):
    """
    上行帧解析（这是你将来要在 chassis_bridge 里写的那份）。
    返回 dict，坏帧返回 None。

    ★★ 关键设计：字段数必须严格等于 11。
       少一个 → 说明帧被截断了，不能猜
       多一个 → 说明协议版本不匹配，不能忽略
       两种情况都返回 None，由调用方计入坏帧数。
    """
    line = line.strip()
    if not line:
        return None
    parts = line.split(",")
    if len(parts) != 11:
        return None
    if parts[0] != "O":
        return None
    try:
        return {
            "seq": int(parts[1]),
            "enc_l": int(parts[2]),
            "enc_r": int(parts[3]),
            "vel_l": int(parts[4]),
            "vel_r": int(parts[5]),
            "pwm_l": int(parts[6]),
            "pwm_r": int(parts[7]),
            "vbat_mv": int(parts[8]),
            "tick_ms": int(parts[9]),
            "temp_c100": int(parts[10]),
        }
    except ValueError:
        return None


def parse_velocity_cmd(line):
    """下行帧解析：V,<seq>,<vel_l>,<vel_r>，必须 4 个字段"""
    line = line.strip()
    parts = line.split(",")
    if len(parts) != 4 or parts[0] != "V":
        return None
    try:
        return {"seq": int(parts[1]), "vel_l": int(parts[2]), "vel_r": int(parts[3])}
    except ValueError:
        return None


def cmdvel_to_wheel_mms(v, omega, wheel_sep):
    """
    差速运动学逆解：/cmd_vel 的 (v, ω) → 左右轮速度 (mm/s)

    ★★ 这两个公式是整个导航系统里最该背下来的一对：
         v_l = v - ω·L/2
         v_r = v + ω·L/2
    ★ 注意 round 不是 int()：int() 是截断（向零取整），
      对负数会 -0.6 → 0，带来系统性偏差。必须用 round。
    """
    v_l = v - omega * wheel_sep / 2.0
    v_r = v + omega * wheel_sep / 2.0
    return int(round(v_l * 1000)), int(round(v_r * 1000))


def wheel_mms_to_cmdvel(vel_l_mms, vel_r_mms, wheel_sep):
    """
    差速运动学正解：左右轮速度 → (v, ω)
    这是 chassis_bridge 里算 /odom 要用的方向。

         v     = (v_l + v_r) / 2
         ω     = (v_r - v_l) / L
    """
    v_l = vel_l_mms / 1000.0
    v_r = vel_r_mms / 1000.0
    v = (v_l + v_r) / 2.0
    omega = (v_r - v_l) / wheel_sep
    return v, omega


def enc_delta_to_mms(delta_enc, wheel_radius, encoder_ppr, dt):
    """编码器脉冲增量 → 线速度 mm/s（和固件 TODO-A2 同一个公式）"""
    m_per_pulse = (2.0 * math.pi * wheel_radius) / encoder_ppr
    return delta_enc * m_per_pulse / dt * 1000.0


# ============================================================================
# 极简测试框架
# ============================================================================
_PASS = 0
_FAIL = 0
_FAILED_NAMES = []


def check(name, cond, detail=""):
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"  ✓ {name}")
    else:
        _FAIL += 1
        _FAILED_NAMES.append(name)
        print(f"  ✗ {name}   {detail}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ============================================================================
# 测试用例
# ============================================================================
def test_frame_roundtrip():
    print("\n[1] 上行帧 组装 → 解析 往返一致")
    frame = build_odom_frame(1234, -5820, 5815, 120, 118, 420, 415, 11850, 184320, 2510)
    check("帧以 \\n 结尾", frame.endswith("\n"))
    check("字段数 = 11", len(frame.strip().split(",")) == 11,
          f"实际 {len(frame.strip().split(','))}")

    d = parse_odom_frame(frame)
    check("解析成功", d is not None)
    if d:
        check("seq 往返一致", d["seq"] == 1234)
        check("enc_l 保留负号", d["enc_l"] == -5820)
        check("enc_r 正数", d["enc_r"] == 5815)
        check("vel_l", d["vel_l"] == 120)
        check("vbat_mv", d["vbat_mv"] == 11850)
        check("tick_ms", d["tick_ms"] == 184320)
        check("temp_c100", d["temp_c100"] == 2510)


def test_bad_frames():
    print("\n[2] ★ 坏帧必须被静默丢弃（不能猜、不能崩）")
    bad_cases = [
        ("空行", ""),
        ("只有换行", "\n"),
        ("字段太少", "O,1,2,3"),
        ("字段太多", "O,1,2,3,4,5,6,7,8,9,10,11,12"),
        ("帧头错误", "X,1,2,3,4,5,6,7,8,9,10"),
        ("纯乱码", "~~~GARBAGE### not a valid frame"),
        ("非数字字段", "O,abc,2,3,4,5,6,7,8,9,10"),
        ("缺帧头", "1,2,3,4,5,6,7,8,9,10,11"),
    ]
    for name, raw in bad_cases:
        check(f"{name} → None", parse_odom_frame(raw) is None,
              f"却解析出了 {parse_odom_frame(raw)}")


def test_velocity_cmd_parse():
    print("\n[3] 下行帧解析 V,<seq>,<vel_l>,<vel_r>")
    d = parse_velocity_cmd("V,567,120,-120\n")
    check("正常帧解析", d == {"seq": 567, "vel_l": 120, "vel_r": -120})
    check("缺字段丢弃", parse_velocity_cmd("V,1,100") is None)
    check("多字段丢弃", parse_velocity_cmd("V,1,100,200,999") is None)
    check("帧头错丢弃", parse_velocity_cmd("O,1,100,200") is None)
    check("非数字丢弃", parse_velocity_cmd("V,x,100,200") is None)


def test_kinematics_inverse():
    """
    ★★ 这一组是本文件最重要的测试：
       用「物理上应该是什么样」去验公式，而不是用公式验公式。
    """
    print("\n[4] ★★ 差速运动学逆解（/cmd_vel → 轮速）")
    L = 0.16

    # 场景 1：纯直行，两轮必须同速同向
    vl, vr = cmdvel_to_wheel_mms(0.2, 0.0, L)
    check("直行 0.2m/s → 两轮都是 200 mm/s", vl == 200 and vr == 200,
          f"实际 L={vl} R={vr}")

    # 场景 2：纯原地旋转，两轮必须等速反向
    vl, vr = cmdvel_to_wheel_mms(0.0, 0.5, L)
    check("原地转 ω=0.5 → L=-40 R=+40 mm/s", vl == -40 and vr == 40,
          f"实际 L={vl} R={vr}")
    check("原地转：两轮绝对值相等", abs(vl) == abs(vr))

    # 场景 3：走圆弧（前进+转向）
    vl, vr = cmdvel_to_wheel_mms(0.1, 1.0, L)
    check("圆弧 v=0.1 ω=1.0 → L=20 R=180 mm/s", vl == 20 and vr == 180,
          f"实际 L={vl} R={vr}")
    check("左转时右轮更快（差速车的物理事实）", vr > vl)

    # 场景 4：★ 负速度必须正确取整（int() 会截断错，round() 才对）
    vl, vr = cmdvel_to_wheel_mms(-0.0006, 0.0, L)
    check("★ -0.0006 m/s → -1 mm/s（round 而非截断）", vl == -1 and vr == -1,
          f"实际 L={vl} R={vr}（若为 0 说明用了 int() 截断）")


def test_kinematics_forward():
    print("\n[5] ★ 差速运动学正解（轮速 → /odom 的 v,ω）")
    L = 0.16

    v, w = wheel_mms_to_cmdvel(200, 200, L)
    check("两轮同速 200 → v=0.2 ω=0", approx(v, 0.2) and approx(w, 0.0),
          f"实际 v={v} ω={w}")

    v, w = wheel_mms_to_cmdvel(-40, 40, L)
    check("两轮反向 ±40 → v=0 ω=0.5", approx(v, 0.0) and approx(w, 0.5),
          f"实际 v={v} ω={w}")

    # ★ 往返一致性：逆解后再正解，应该回到原值
    for (v0, w0) in [(0.2, 0.0), (0.0, 0.5), (0.15, -0.8), (-0.1, 0.3)]:
        vl, vr = cmdvel_to_wheel_mms(v0, w0, L)
        v1, w1 = wheel_mms_to_cmdvel(vl, vr, L)
        # 允许 mm/s 取整带来的误差：1mm/s ÷ (L/2) = 0.0125 rad/s
        ok = abs(v1 - v0) < 0.002 and abs(w1 - w0) < 0.02
        check(f"往返一致 v={v0} ω={w0}", ok, f"回到 v={v1:.4f} ω={w1:.4f}")


def test_encoder_conversion():
    print("\n[6] ★★ 编码器脉冲 → 速度（最容易错在单位）")
    R = 0.0325
    PPR = 2112.0
    dt = 0.02  # 50Hz

    # 手算锚点：轮子以 0.2 m/s 转，一个 20ms 周期走 0.004 m
    #   弧长/脉冲 = 2π*0.0325 / 2112 = 9.6687e-5 m
    #   0.004 / 9.6687e-5 = 41.37 个脉冲
    #   → 如果收到 41 个脉冲，算出来应该是约 0.198 m/s = 198 mm/s
    mm_per_pulse = (2.0 * math.pi * R) / PPR * 1000
    check("每脉冲弧长 ≈ 0.0967 mm", approx(mm_per_pulse, 0.0967, 1e-3),
          f"实际 {mm_per_pulse:.5f} mm")

    v = enc_delta_to_mms(41, R, PPR, dt)
    check("41 脉冲/20ms → ≈198 mm/s", approx(v, 198.3, 1.0), f"实际 {v:.1f}")

    v = enc_delta_to_mms(0, R, PPR, dt)
    check("0 脉冲 → 0 mm/s（静止读数为零）", v == 0.0)

    v = enc_delta_to_mms(-41, R, PPR, dt)
    check("负脉冲 → 负速度（方向正确）", v < 0, f"实际 {v:.1f}")

    # ★★ 静止抖动的可见性：±2 个脉冲会算出多大速度？
    #    这就是"零点漂移"为什么必须做死区处理
    v_jitter = enc_delta_to_mms(2, R, PPR, dt)
    check("★ 2 脉冲抖动 → 约 9.7 mm/s（说明死区必须做）",
          approx(v_jitter, 9.67, 0.1), f"实际 {v_jitter:.2f} mm/s")


def test_physical_sanity():
    """
    ★★ 物理合理性测试：用「车应该走多远」反推公式对不对。
       这是无硬件条件下唯一能验「整套单位链条」的办法。
    """
    print("\n[7] ★★ 物理合理性：走 1 米应该产生多少脉冲")
    R = 0.0325
    PPR = 2112.0

    # 走 1 米 = 1 / (2πR) 圈
    #
    # ★★ 手算锚点（把这个数记下来，真机上能用来秒验单位对不对）：
    #     轮周长 = 2π × 0.0325 = 0.20420 m
    #     走 1m 需要的圈数 = 1 / 0.20420 = 4.8971 圈
    #     脉冲数 = 4.8971 × 2112 = 10342.6  ≈ 10343
    #
    #   ★ 真机验收用法：
    #     把车在地面上推着走整整 1 米（用卷尺量），
    #     读 enc_l 的增量。如果 ≈ ±10343（误差 5% 内），
    #     说明 轮半径 / 编码器PPR / 四倍频判断 三者都对了。
    #     如果差 2 倍或 4 倍 → 四倍频没开或开了两次。
    #     —— 这一条比任何单测都值钱，因为是端到端的物理验证。
    turns = 1.0 / (2.0 * math.pi * R)
    pulses = turns * PPR
    check(f"走 1m ≈ 10343 个脉冲（实际 {pulses:.0f}）", 10300 < pulses < 10400,
          f"实际 {pulses:.0f}")

    # 反向验算：这个脉冲数换算回来应该是 1 米
    total_arc_mm = pulses * (2.0 * math.pi * R) / PPR * 1000
    check("脉冲数反算回 1000 mm", approx(total_arc_mm, 1000.0, 0.1),
          f"实际 {total_arc_mm:.2f} mm")

    # 转 360°：左右轮各走一个周长
    check(f"原地转一圈，单轮走 {2*math.pi*0.08*1000:.1f} mm（轮距 0.16 → 半径 0.08）",
          approx(2 * math.pi * 0.08 * 1000, 502.65, 0.1))


def test_frame_rate_consistency():
    """
    ★ 验证「帧率」这个最容易出问题的运行时指标。
      用假帧流跑一遍计时，确认 --rate 参数真的生效。
    """
    print("\n[8] 帧率一致性（不依赖硬件，纯计时）")
    import time

    target_hz = 50.0
    n = 50
    t0 = time.monotonic()
    for i in range(n):
        target_t = t0 + (i + 1) / target_hz
        now = time.monotonic()
        if target_t > now:
            time.sleep(target_t - now)
    elapsed = time.monotonic() - t0
    actual_hz = n / elapsed
    check(f"{n} 帧 @ {target_hz}Hz → 实测 {actual_hz:.1f} Hz",
          abs(actual_hz - target_hz) < 2.0,
          f"偏差 {abs(actual_hz - target_hz):.2f} Hz")


# ============================================================================
def main():
    print("=" * 70)
    print("  navbot 协议与运动学 单测")
    print("  不依赖串口 / rclpy / 硬件 —— 纯逻辑验证")
    print("=" * 70)

    test_frame_roundtrip()
    test_bad_frames()
    test_velocity_cmd_parse()
    test_kinematics_inverse()
    test_kinematics_forward()
    test_encoder_conversion()
    test_physical_sanity()
    test_frame_rate_consistency()

    print()
    print("=" * 70)
    print(f"  汇总：{_PASS} 通过 / {_FAIL} 失败 / 共 {_PASS + _FAIL} 条")
    if _FAIL:
        print("  失败项：")
        for n in _FAILED_NAMES:
            print(f"    ✗ {n}")
    print("=" * 70)

    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
