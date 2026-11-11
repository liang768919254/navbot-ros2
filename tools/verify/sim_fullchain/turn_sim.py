#!/usr/bin/env python3
"""离线复现 drive.py 的 turn() 收敛行为，验证「少转 1~2°」的根因已修掉。

动机（2026-11-10）：
  drive.py loop 的日志里每次转向都报
    转向 -90° 完成，实际 -89°
    转向 +90° 完成，实际 +89°
    ...
  5 次累积 -6°。这不像随机噪声，像【系统性提前 break】。
  本脚本用纯数学模拟「一阶惯性 + 定频控制」的被控对象，
  分别跑老 turn 与新 turn 的判定逻辑，对比残差。

核心结论（跑完就能看到）：
  老逻辑 break 判据是 |accum| >= |angle| - tol，tol=0.03 rad≈1.7°，
  且 break 后直接 stop()，最后一帧角速度仍在途 → 稳态残差≈tol。
  新逻辑把 tol 压到 0.008 rad≈0.46°，并且刹停后再读一次 yaw，
  残差落到 0.1° 量级，6 次转向累积 < 1°。

用法：
  python3 turn_sim.py
"""
import math

DT = 0.05              # 控制周期（drive.py 里 _pump(0.05)）
OMEGA = 0.6            # 额定角速度 rad/s
TAU = 0.12             # 车体+插件的一阶惯性时间常数（估的，只影响形状不影响结论）
YAW_MAX = 6570.0       # /odom 量化：yaw 用 double，量化误差可忽略


class Plant:
    """一阶惯性：dh/dt = (u - h)/TAU，h 是实际角速度。"""

    def __init__(self):
        self.h = 0.0
        self.yaw = 0.0

    def step(self, u):
        self.h += (u - self.h) * DT / TAU
        self.yaw += self.h * DT


def old_turn(plant, angle, tol=0.03):
    """复刻旧逻辑。"""
    accum, prev = 0.0, plant.yaw
    for _ in range(2000):
        if abs(accum) >= abs(angle) - tol:
            break
        u = (OMEGA if angle > 0 else -OMEGA)
        if abs(accum) >= abs(angle) * 0.8:
            u *= 0.5
        plant.step(u)
        accum += plant.yaw - prev
        prev = plant.yaw
    # 旧逻辑：break 后 stop() 只发一条零速、不等惯性衰减
    plant.step(0.0)          # 只走一帧就认为停了
    return accum


def new_turn(plant, angle, tol=0.008):
    """复刻新逻辑：不提前收 + 降速收尾 + 刹停后再读一次。"""
    accum, prev = 0.0, plant.yaw
    for _ in range(2000):
        remain = abs(angle) - abs(accum)
        if remain <= tol:
            break
        if remain < abs(angle) * 0.25:      # 精转段
            w = OMEGA * 0.4 * max(0.15, remain / (abs(angle) * 0.25))
        else:
            w = OMEGA
        plant.step(w if angle > 0 else -w)
        accum += plant.yaw - prev
        prev = plant.yaw
    # 刹停：让惯性真正衰减掉（drive.py 里是 stop() + _pump(0.25)）
    for _ in range(int(0.25 / DT)):
        plant.step(0.0)
    accum += plant.yaw - prev
    return accum


if __name__ == '__main__':
    print('控制周期 %.0f ms, 额定 %.1f rad/s, 惯性 TAU=%.2f s\n' % (DT * 1000, OMEGA, TAU))
    print('%-10s %-14s %-14s' % ('角度', '老逻辑残差', '新逻辑残差'))
    print('-' * 42)
    tot_old = tot_new = 0.0
    for deg in (-90, 90, 90, 90, 90, 90, 90):
        a = math.radians(deg)
        p1, p2 = Plant(), Plant()
        r1 = math.degrees(a - old_turn(p1, a))
        r2 = math.degrees(a - new_turn(p2, a))
        tot_old += r1
        tot_new += r2
        print('%-10s %+13.2f° %+13.2f°' % ('%+d°' % deg, r1, r2))
    print('-' * 42)
    print('%-10s %+13.2f° %+13.2f°' % ('累计', tot_old, tot_new))
    print()
    if abs(tot_old) > abs(tot_new) * 3:
        print('结论：老逻辑的残差是【系统性、同向累积】的（每次都朝少转一侧），')
        print('      不是随机噪声。新逻辑把它压到 1/%.0f 以下。'
              % (abs(tot_old) / max(abs(tot_new), 1e-9)))
    else:
        print('结论：两种逻辑残差量级相当，需要复查模型假设。')
