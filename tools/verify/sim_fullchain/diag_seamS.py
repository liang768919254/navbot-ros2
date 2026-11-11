#!/usr/bin/env python3
"""diag_seamS.py —— 纯几何诊断：缝 S / 东南角到底能不能从南往北过。

起因（2026-11-10 夜，用户反馈）：
  「从上面能过去，从下面就是一直卡在这里」
  RViz 截图里车停在 partition_short 南端西南侧 ≈ (0.9, -1.6)，
  绿色全局路径从车位置向南→向东绕过板南端，再沿板东侧垂直北上。

  这不是「控制器调参」问题，先算清楚：车心放在哪些点，车体圆能放下？
  如果南端西侧那一带根本没有可通行位置，那就是几何/膨胀的问题，
  改 scale / fpd 全部白费。

用法：python3 tools/verify/sim_fullchain/diag_seamS.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import room_geom as rg

R = rg.ROBOT_RADIUS
INF = rg.INFLATION

# ---------------------------------------------------------------- 1. 关键点净空
print("=" * 78)
print("[1] 关键点净空 / 膨胀代价（车心在这些位置能不能站）")
print("=" * 78)
print(f"    robot_radius={R}  inflation_radius={INF}  "
      f"⇒ 车心距障碍 < {R:.2f} 必撞；< {INF:.2f} 带代价；>= {INF:.2f} 零代价")
print()
print(f"    {'点':<26}{'x':>7}{'y':>7}{'净空':>8}{'cost':>7}  判定")
print("    " + "-" * 70)

CASES = [
    ("车当前(截图位置)",      0.90, -1.60),
    ("车当前偏南",            0.85, -1.80),
    ("板南端西侧 x0.95",      0.95, -1.75),
    ("板南端正下 y=-2.05",    1.20, -2.05),
    ("板南端正下 y=-2.02",    1.20, -2.02),
    ("板南端正下 y=-1.98",    1.20, -1.98),
    ("板东侧缝中点(1.55,-0.9)", 1.55, -0.90),
    ("板东侧北上(1.55,-0.2)",  1.55, -0.20),
    ("缝S东侧 x=1.45 y=-1.5",  1.45, -1.50),
    ("缝S东侧 x=1.42 y=-1.5",  1.42, -1.50),
]


def judge(cost, clr):
    if clr < R:
        return "✗ 必撞（车心净空 < 车半径）"
    if cost >= 253:
        return "✗ 致命/内切（规划器不走）"
    if cost > 0:
        return f"△ 可通过但有代价 {cost}"
    return "✓ 零代价"


for name, x, y in CASES:
    clr = rg.clearance(x, y)
    cost = rg.inflate_cost(clr)
    print(f"    {name:<26}{x:>7.2f}{y:>7.2f}{clr:>8.3f}{cost:>7d}  {judge(cost, clr)}")

# ---------------------------------------------------------------- 2. 板南端切口
print()
print("=" * 78)
print("[2] partition_short 南端到南墙之间，到底有多宽？")
print("=" * 78)
short_y0 = -1.795          # 板南端
wall_y = rg.Y_MIN          # 南墙内表面 -2.45
gap = short_y0 - wall_y    # ★ 上减下，别写反（第一版写成 wall_y - short_y0 得 -0.655）
print(f"    板南端 y = {short_y0:.3f}")
print(f"    南墙内表面 y = {wall_y:.3f}")
print(f"    切口宽 = {gap:.3f} m")
print(f"    车体直径 2R = {2*R:.3f} m   →  {'能过' if gap > 2*R else '过不去'}"
      f"（余 {gap - 2*R:+.3f} m）")
print(f"    零代价需 2*inflation = {2*INF:.3f} m → "
      f"{'有零代价通道' if gap > 2*INF else '★ 整条切口都带代价，无零代价通道'}")

# 纵向：车心 y 的取值范围
y_lo = wall_y + R          # 贴南墙
y_hi = short_y0 - R        # 贴板南端
print(f"    车心 y 合法区间 = [{y_lo:.3f}, {y_hi:.3f}]  宽度 {y_hi-y_lo:+.3f} m")
if y_hi - y_lo <= 0:
    print("    ★★ 区间为空：车心在 x≈1.2 竖线上没有任何合法位置")
    print("        ⇒ 只能绕到 x > 1.317+R 或 x < 1.117-R 才过得去")

# ---------------------------------------------------------------- 3. 板东侧通道
print()
print("=" * 78)
print("[3] 板东侧（车要从这里北上）净空")
print("=" * 78)
short_x1 = 1.317           # 板东表面
east_wall = rg.X_MAX
east_w = east_wall - short_x1
print(f"    板东表面 x = {short_x1:.3f}，东墙内表面 x = {east_wall:.3f}")
print(f"    通道宽 = {east_w:.3f} m（车直径 {2*R:.3f}）")
print(f"    车心 x 合法区间 = [{short_x1+R:.3f}, {east_wall-R:.3f}]，"
      f"宽度 {east_w - 2*R:+.3f} m")
print(f"    零代价车心区间 = [{short_x1+INF:.3f}, {east_wall-INF:.3f}]，"
      f"宽度 {east_w - 2*INF:+.3f} m")

# ---------------------------------------------------------------- 4. 逐行扫描南端
print()
print("=" * 78)
print("[4] 南端横向逐 x 扫描：找出「车能站且零代价」的 x 区间")
print("=" * 78)
for y in [-2.05, -2.00, -1.95, -1.90, -1.85, -1.80]:
    ok = [round(x * 0.01, 2) for x in range(60, 300)
          if rg.clearance(x * 0.01, y) >= INF]
    if ok:
        print(f"    y={y:>6.2f}  零代价 x ∈ [{ok[0]:.2f}, {ok[-1]:.2f}]  "
              f"共 {len(ok)} 个 1cm 格")
    else:
        print(f"    y={y:>6.2f}  零代价 x : 无")

# ---------------------------------------------------------------- 5. 结论
print()
print("=" * 78)
print("[5] 结论")
print("=" * 78)
print("""    把上面几块拼起来：
      · 板东侧通道 1.63 m 宽 —— 很宽敞，理论上是主通路
      · 板南端到南墙只有 0.655 m 切口 —— 能过车（2R=0.40），
        但车心纵深只剩 0.255 m，且紧贴南墙/板端两处膨胀
      · 从西侧(x≈0.9)到板东侧(x≈1.55)，绕行必须穿过这 0.255 m 的窄带
    这就是「从上面能过去（从板东侧直下，路径平整），
    从下面过不去（从西侧横向穿入 0.255 m 窄带 + 车头要转 90°）」的几何根源。""")
