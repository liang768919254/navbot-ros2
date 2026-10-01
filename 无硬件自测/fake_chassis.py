#!/usr/bin/env python3
"""
fake_chassis.py —— 假底盘：在 PC 上冒充 STM32，不依赖任何硬件。

用途：在没有 STM32、没有电机、没有编码器的情况下，
      把「PC 侧串口桥 → ROS2 话题」这一半验完。

★★ 这个脚本是本项目「无硬件自测」的核心，它干三件事：
    1. 造一个虚拟串口（用 socat），或者直接读你指定的已存在串口
    2. 按协议 v1 的格式，50Hz 往外吐上行帧（带模拟的运动学）
    3. 同时监听下行 V 指令，改变自己的运动状态

★ 为什么要有它？
    因为真实开发里 90% 的时间花在「PC 侧的对接问题」上，
    而这些问题（解析、单位、时间戳、TF）和硬件一点关系都没有。
    先用它把 PC 侧全跑通，再上硬件，你会省掉一大半排查时间。

------------------------------------------------------------------------------
三种用法：
------------------------------------------------------------------------------
# 用法 1 ★ 推荐：自动造一对虚拟串口（需要 socat）
./fake_chassis.py --make-socat
    → 脚本会用 socat 建出 /tmp/ttyV0（本脚本用）和 /tmp/ttyV1（你的桥用）
    → 然后自动开始发数据
    → 你在另一个终端跑：ros2 run navbot_bringup chassis_bridge --ros-args -p port:=/tmp/ttyV1

# 用法 2：你已经有虚拟串口对了（自己用 socat 开的）
./fake_chassis.py --port /tmp/ttyV0

# 用法 3：预演模式，不占串口，只在终端里打印帧（先看数据长什么样）
./fake_chassis.py --dry-run

------------------------------------------------------------------------------
可编程的「故障注入」——这是它真正的价值
------------------------------------------------------------------------------
  --stall-after N      发 N 帧后彻底静默（模拟 MCU 死机）
  --garbage-every N    每 N 帧插一帧乱码（验证桥的坏帧静默处理）
  --no-seq             不递增 seq（验证桥的"陈旧数据"检测）
  --zero-drift MM      静止时的速度读数恒定偏移（验证零点漂移的可见性）
  --rate HZ            上行频率，默认 50

★ 用 --garbage-every 5 配合桥的日志，能一次性验完
  「坏帧静默计数」这条验收项 —— 这是 sensor_bridge 阶段二那次
  「topic hz = 0.198」事故留下的直接经验。
"""

import argparse
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time


# ============================================================================
# 运动模型：一个简单的差速车运动学模拟
#
# ★ 为什么要有运动模型，而不是随便发随机的数？
#   因为「随便发数」验不出里程计对不对。
#   有运动模型之后，你可以：
#     · 让车前进 1 米，检查 /odom 的 x 是否≈1.0 → 验单位换算
#     · 让车原地转 360°，检查 yaw 是否≈6.283 → 验轮距
#   这两条是「无硬件条件下唯一能验里程计正确性」的办法。
# ============================================================================
class DiffDriveSim:
    """两轮差速车的运动学模拟，单位统一用 SI（m, m/s, rad）"""

    def __init__(self, wheel_sep=0.16, wheel_radius=0.0325, encoder_ppr=2112.0):
        self.L = wheel_sep
        self.R = wheel_radius
        self.PPR = encoder_ppr

        # 车的真实位姿（世界坐标系）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 编码器累计脉冲（整数，和真实 MCU 一致）
        self.enc_l = 0
        self.enc_r = 0

        # 当前轮速（mm/s），由下行指令设置
        self.target_l_mms = 0
        self.target_r_mms = 0

        # 实际上电后电机有响应延迟，模拟 100ms 一阶延迟
        # ★ 这是为了让"收到指令→车动"的时序更接近真实，
        #   如果你的桥在这个延迟下有问题，仿真里就该发现
        self.actual_l_mms = 0.0
        self.actual_r_mms = 0.0

    def step(self, dt):
        """推进一个时间步，返回本步的编码器增量 (delta_l, delta_r)"""
        # 一阶延迟：向目标值靠拢（时间常数 100ms）
        tau = 0.1
        alpha = min(1.0, dt / tau)
        self.actual_l_mms += (self.target_l_mms - self.actual_l_mms) * alpha
        self.actual_r_mms += (self.target_r_mms - self.actual_r_mms) * alpha

        # 轮速 → 本步转过的角度
        # ★ 注意单位：mm/s 要转成 m/s，再算弧长
        v_l = self.actual_l_mms / 1000.0
        v_r = self.actual_r_mms / 1000.0

        # 差速运动学正解
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / self.L

        # 积分位姿（欧拉积分，20ms 步长误差可忽略）
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += omega * dt

        # 位姿 → 编码器脉冲
        # 弧长 = 速度 × 时间；脉冲 = 弧长 / (2πR) × PPR
        arc_l = v_l * dt
        arc_r = v_r * dt
        delta_l = arc_l / (2.0 * math.pi * self.R) * self.PPR
        delta_r = arc_r / (2.0 * math.pi * self.R) * self.PPR

        self.enc_l += int(round(delta_l))
        self.enc_r += int(round(delta_r))

        return delta_l, delta_r


# ============================================================================
# 上行帧组装
# ============================================================================
def build_odom_frame(seq, enc_l, enc_r, vel_l, vel_r, pwm_l, pwm_r,
                     vbat_mv, tick_ms, temp_c100):
    """
    ★ 帧格式必须和 参考源码/firmware/navbot_protocol.md 一字不差。
      改这里就要改那边，反之亦然 —— 这就是协议要有文档的原因。
    """
    return (f"O,{seq},{enc_l},{enc_r},{vel_l},{vel_r},"
            f"{pwm_l},{pwm_r},{vbat_mv},{tick_ms},{temp_c100}\n")


def mms_to_pwm(mms, deadzone=30):
    """
    速度 → PWM 的粗略映射，仅用于让假数据看起来合理。
    ★ 真实映射在你的固件 TODO-B4 里实现。
    这里做线性假设：800 mm/s 对应 PWM 1000
    """
    pwm = int(mms / 800.0 * 1000)
    pwm = max(-1000, min(1000, pwm))
    if 0 < abs(pwm) < deadzone:
        pwm = deadzone if pwm > 0 else -deadzone
    return pwm


# ============================================================================
# 虚拟串口管理
# ============================================================================
def make_socat_pair(link_a="/tmp/ttyV0", link_b="/tmp/ttyV1"):
    """
    ★ 用 socat 造一对「背靠背」的虚拟串口。
      往 ttyV0 写的东西，从 ttyV1 能读到，反之亦然。
      这是在没有硬件时验证串口协议的标准手段。

    ★★ 坑：socat 进程一退出（包括你 Ctrl+C），这两个符号链接就消失了。
      重开时必须重新 ls -l /tmp/ttyV* 确认，否则桥会报
      "could not open port /tmp/ttyV1: [Errno 2] No such file or directory"
      而你要花时间怀疑代码。
    """
    if not shutil.which("socat"):
        print("✗ 没找到 socat。安装：sudo apt install socat", file=sys.stderr)
        print("  或者用 --dry-run 先看数据格式 / --port 指定已有串口", file=sys.stderr)
        return None

    # 清掉可能残留的旧链接
    for p in (link_a, link_b):
        if os.path.islink(p) or os.path.exists(p):
            try:
                os.remove(p)
            except OSError:
                pass

    proc = subprocess.Popen(
        ["socat", "-d", "-d",
         f"pty,raw,echo=0,link={link_a}",
         f"pty,raw,echo=0,link={link_b}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # 等链接出现（socat 要几十毫秒）
    for _ in range(50):
        if os.path.exists(link_a) and os.path.exists(link_b):
            print(f"✓ socat 虚拟串口对已建立：{link_a} <-> {link_b}")
            return proc
        time.sleep(0.05)

    print("✗ socat 起了但链接没出现，检查 /tmp 权限", file=sys.stderr)
    proc.terminate()
    return None


# ============================================================================
# 主循环
# ============================================================================
def run(args):
    sim = DiffDriveSim(
        wheel_sep=args.wheel_sep,
        wheel_radius=args.wheel_radius,
        encoder_ppr=args.encoder_ppr,
    )

    ser = None
    socat_proc = None

    if not args.dry_run:
        if args.make_socat:
            socat_proc = make_socat_pair("/tmp/ttyV0", "/tmp/ttyV1")
            if socat_proc is None:
                return 1
            args.port = "/tmp/ttyV0"

        if not args.port:
            print("✗ 需要 --port 或 --make-socat（或用 --dry-run）", file=sys.stderr)
            return 1

        try:
            import serial
        except ImportError:
            print("✗ 缺 pyserial。装给系统 python：", file=sys.stderr)
            print("    sudo apt install python3-serial", file=sys.stderr)
            print("  ★ 不要用 pip 装 —— 会和 ROS2 的 python 环境打架", file=sys.stderr)
            if socat_proc:
                socat_proc.terminate()
            return 1

        try:
            # ★ 这里的 timeout 很关键：
            #   如果设成 None（阻塞），readline 会永久占住这个循环；
            #   真机上这正是"读串口必须另开线程"的根因。
            #   这里用 0 之外的短超时，让循环能继续跑。
            ser = serial.Serial(args.port, 115200, timeout=0.005)
            print(f"✓ 已打开串口 {args.port} @ 115200 8N1")
        except Exception as e:
            print(f"✗ 打开串口失败：{e}", file=sys.stderr)
            print(f"  检查 {args.port} 是否存在：ls -l {args.port}", file=sys.stderr)
            if socat_proc:
                socat_proc.terminate()
            return 1

    # ------------------------------------------------------------------
    # 优雅退出：Ctrl+C 时清理 socat
    # ------------------------------------------------------------------
    stop = {"flag": False}

    def on_sigint(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, on_sigint)

    print()
    print("=" * 70)
    print(f"  假底盘启动 | 频率 {args.rate} Hz | seq {'关闭' if args.no_seq else '递增'}")
    if args.stall_after:
        print(f"  ★ 故障注入：{args.stall_after} 帧后彻底静默")
    if args.garbage_every:
        print(f"  ★ 故障注入：每 {args.garbage_every} 帧插一帧乱码")
    if args.zero_drift:
        print(f"  ★ 故障注入：静止读数偏移 {args.zero_drift} mm/s")
    print("=" * 70)
    print()

    period = 1.0 / args.rate
    seq = 0
    t0 = time.monotonic()
    next_tick = t0
    frames_sent = 0
    rx_buf = b""
    last_report = t0

    try:
        while not stop["flag"]:
            now = time.monotonic()

            # --------------------------------------------------------------
            # 1) 读下行指令（非阻塞，短超时）
            # --------------------------------------------------------------
            if ser is not None:
                try:
                    chunk = ser.read(256)
                    if chunk:
                        rx_buf += chunk
                        while b"\n" in rx_buf:
                            line, rx_buf = rx_buf.split(b"\n", 1)
                            text = line.decode("ascii", errors="ignore").strip()
                            if text.startswith("V,"):
                                parts = text.split(",")
                                # ★ 严格校验字段数 —— 和协议文档一致
                                if len(parts) == 4:
                                    try:
                                        sim.target_l_mms = int(parts[2])
                                        sim.target_r_mms = int(parts[3])
                                    except ValueError:
                                        pass  # 坏帧静默丢弃
                except Exception:
                    pass  # 串口偶发错误不要崩

            # --------------------------------------------------------------
            # 2) 定时发射上行帧
            # --------------------------------------------------------------
            if now >= next_tick:
                next_tick += period

                # 故障注入：静默
                if args.stall_after and frames_sent >= args.stall_after:
                    time.sleep(0.05)
                    continue

                dt = period
                delta_l, delta_r = sim.step(dt)

                # 静止时的零点漂移注入
                # ★ 为什么值得注入这个？
                #   因为真实编码器在静止时也会有 ±1~3 个脉冲的抖动，
                #   如果桥没有死区处理，/odom 会一直缓慢漂移，
                #   导航时表现为"车停在原地，地图却在动"。
                if args.zero_drift and abs(sim.actual_l_mms) < 1.0:
                    sim.enc_l += args.zero_drift
                    sim.enc_r += args.zero_drift

                # 脉冲增量 → mm/s（和固件 TODO-A2 同一个公式）
                # ★ 提取成变量，方便你对照固件里自己写的实现
                m_per_pulse = (2.0 * math.pi * sim.R) / sim.PPR
                vel_l = int(round(delta_l * m_per_pulse / dt * 1000))
                vel_r = int(round(delta_r * m_per_pulse / dt * 1000))

                pwm_l = mms_to_pwm(sim.actual_l_mms)
                pwm_r = mms_to_pwm(sim.actual_r_mms)

                # 电池电压：从 11.8V 缓慢掉到 11.2V，模拟真实放电
                elapsed = now - t0
                vbat_mv = int(11800 - min(600, elapsed * 1.0))
                tick_ms = int(elapsed * 1000) & 0xFFFFFFFF
                temp_c100 = 2510  # 25.10 °C

                if args.no_seq:
                    out_seq = 0
                else:
                    out_seq = seq
                    seq += 1

                frame = build_odom_frame(
                    out_seq, sim.enc_l, sim.enc_r,
                    vel_l, vel_r, pwm_l, pwm_r,
                    vbat_mv, tick_ms, temp_c100,
                )

                # 故障注入：乱码
                if args.garbage_every and (frames_sent + 1) % args.garbage_every == 0:
                    frame = "~~~GARBAGE###\x00\xff not a valid frame at all \n"

                payload = frame.encode("ascii", errors="replace")

                if ser is not None:
                    try:
                        ser.write(payload)
                    except Exception as e:
                        print(f"  ! 写串口失败：{e}", file=sys.stderr)
                        break
                else:
                    # dry-run：打印出来给你看格式
                    sys.stdout.write(f"[{tick_ms:>6} ms] {frame}")

                frames_sent += 1

            # --------------------------------------------------------------
            # 3) 每秒打一次状态（让你知道它活着）
            # --------------------------------------------------------------
            if now - last_report >= 1.0:
                x, y, yaw = sim.x, sim.y, sim.yaw
                print(f"  [状态] 已发 {frames_sent:>5} 帧 | "
                      f"位姿 x={x:+.3f}m y={y:+.3f}m yaw={yaw:+.3f}rad | "
                      f"目标轮速 L={sim.target_l_mms:>4} R={sim.target_r_mms:>4} mm/s",
                      flush=True)
                last_report = now

            # 让出 CPU，别把这一核跑满
            time.sleep(0.001)

    finally:
        print()
        print("=" * 70)
        print(f"  假底盘退出 | 共发送 {frames_sent} 帧")
        if frames_sent > 0:
            print(f"  实测平均频率 ≈ {frames_sent / max(0.001, time.monotonic() - t0):.1f} Hz")
        print("=" * 70)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if socat_proc is not None:
            socat_proc.terminate()
            try:
                socat_proc.wait(timeout=2)
            except Exception:
                socat_proc.kill()
            print("✓ socat 已清理")

    return 0


def main():
    ap = argparse.ArgumentParser(
        description="假底盘：无硬件验证 PC 侧串口桥",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", help="已存在的串口设备路径（如 /tmp/ttyV0）")
    ap.add_argument("--make-socat", action="store_true",
                    help="自动用 socat 建一对虚拟串口 /tmp/ttyV0 <-> /tmp/ttyV1")
    ap.add_argument("--dry-run", action="store_true",
                    help="不占串口，只在终端打印帧")
    ap.add_argument("--rate", type=float, default=50.0, help="上行频率 Hz（默认 50）")

    # 运动学参数：★ 默认值和 navbot 模型一致，改模型要同步改这里
    ap.add_argument("--wheel-sep", type=float, default=0.16, help="轮距 m（默认 0.16）")
    ap.add_argument("--wheel-radius", type=float, default=0.0325, help="轮半径 m（默认 0.0325）")
    ap.add_argument("--encoder-ppr", type=float, default=2112.0, help="编码器每圈脉冲（默认 2112）")

    # 故障注入
    ap.add_argument("--stall-after", type=int, default=0,
                    help="发 N 帧后静默（模拟 MCU 死机）")
    ap.add_argument("--garbage-every", type=int, default=0,
                    help="每 N 帧插一帧乱码")
    ap.add_argument("--no-seq", action="store_true",
                    help="seq 恒为 0（验证桥的陈旧数据检测）")
    ap.add_argument("--zero-drift", type=int, default=0,
                    help="静止时每个周期给编码器加的偏移（验证零点漂移）")

    args = ap.parse_args()

    if not args.dry_run and not args.port and not args.make_socat:
        ap.print_help()
        print()
        print("★ 第一次用建议先跑：./fake_chassis.py --dry-run")
        print("  看清数据格式之后，再用 --make-socat 接桥。")
        return 1

    return run(args)


if __name__ == "__main__":
    sys.exit(main())
