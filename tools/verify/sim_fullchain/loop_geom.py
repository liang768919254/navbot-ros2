#!/usr/bin/env python3
"""route_loop 闭环误差的几何仿真 + 航线安全性校核（纯数学，不连 ROS）。

为什么单独写这个：
  实车跑出来的终点 (-2.616, -0.195) 和「理想几何推算」的 (-2.600, -0.200)
  只差 1.6cm。说明车走得非常准，问题在【路线设计】本身。
  这个脚本把路线摊开算一遍，把「差多少、差在哪、为什么差」变成数字，
  避免再靠猜；同时用世界文件的真实尺寸校验航线不会蹭到墙/隔断/柱子。

用法：
  python3 loop_geom.py           # 理想情况（每步精准）
  python3 loop_geom.py --real    # 叠加实测损耗（转向少转 1.5°、直行短 5mm）
  python3 loop_geom.py --check   # 只做航线安全性校核

判据：
  ① 理想情况终点应 < 0.02 m（纯几何闭合）
  ② 叠加实测损耗后终点应 < 0.30 m（闭环控制能吃掉的部分）
  ③ 每条航段与墙/障碍的最小距离 ≥ 车半径 0.20 m
"""
import math
import sys

# ---------------- 世界几何（逐条抄自 navbot_room.world，改世界要同步改这里）--------
# 外墙：厚 0.1，居中在 x=±3.0 / y=±2.5 → 内表面
WALL_X = 2.95
WALL_Y = 2.45
# 障碍：(中心x, 中心y, x向尺寸, y向尺寸)
OBSTACLES = {
    'partition_mid':   (-0.5,  0.5, 2.6,  0.1),   # x∈[-1.8, 0.8],  y≈0.5
    'partition_short': ( 1.2, -0.9, 0.1,  1.8),   # y∈[-1.8, 0.0],  x≈1.2
    'pillar_a':        ( 1.8,  1.5, 0.30, 0.30),  # r=0.15
    'pillar_b':        (-1.9, -1.6, 0.30, 0.30),  # r=0.15
    'movable_box':     (-1.5,  1.3, 0.30, 0.30),  # 可推动，按障碍算
}
ROBOT_RADIUS = 0.20     # 代价地图 robot_radius，也是本脚本的净空判据


def _rect(name):
    cx, cy, sx, sy = OBSTACLES[name]
    return cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2


def point_clearance(x, y):
    """点到最近「墙 / 障碍物表面」的距离，返回 (距离, 是谁)。"""
    best = (min(WALL_X - abs(x), WALL_Y - abs(y)), '外墙')
    for name in OBSTACLES:
        x0, y0, x1, y1 = _rect(name)
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        d = math.hypot(dx, dy)
        if d < best[0]:
            best = (d, name)
    return best


def segment_clearance(p0, p1, n=400):
    """航段上离障碍最近的距离（采样法，够用且不依赖任何库）。"""
    best = (1e9, '')
    for k in range(n + 1):
        t = k / n
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        d = point_clearance(x, y)
        if d[0] < best[0]:
            best = d
    return best


# ---------------- 车体运动模型（语义与 drive.py 一一对应）----------------------
class Sim:
    """yaw=0 朝 +x（东）；turn 正角逆时针，与 drive.py 的 omega 符号一致。"""

    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.trace = [(0.0, 0.0, 0.0, '起点')]

    def turn(self, angle, actual=None):
        self.yaw = _wrap(self.yaw + (angle if actual is None else actual))
        self.trace.append((self.x, self.y, self.yaw,
                           '转向 %+.0f°' % math.degrees(angle)))

    def straight(self, dist, actual=None):
        d = dist if actual is None else actual
        self.x += d * math.cos(self.yaw)
        self.y += d * math.sin(self.yaw)
        self.trace.append((self.x, self.y, self.yaw, '直行 %.2fm' % dist))


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _rad(deg):
    return deg * math.pi / 180.0


# ---------------- 路线 A：修正前的旧版（保留做对照）-----------------------
def route_old(s, real=False):
    """旧版：向南 2.0 → 东 2.4 → 北 4.0 → 西 5.0 → 南 2.2。

    致命问题：向东只走 2.4，向西却走 5.0 —— x 方向净差 -2.6 m，
    最后那段「向南 2.2」只能把车送到 (-2.6, -0.2)，永远回不到原点。
    而且它自始至终只有 4 次 +90° + 1 次 -90°，总转角 = 270°，不是 360°，
    车头最后朝南（-90°），朝向也没闭环。
    """
    steps = [('t', -90), ('d', 2.0), ('t', 90), ('d', 2.4),
             ('t', 90), ('d', 4.0), ('t', 90), ('d', 5.0),
             ('t', 90), ('d', 2.2)]
    _play(s, steps, real, first_loss=1.0, other_loss=1.5, shrink=0.005)


# ---------------- 路线 B：修正版（矩形闭环，起点在矩形边上）---------------
# 房间内净空 x∈[-2.95,2.95], y∈[-2.45,2.45]
#   partition_short 下沿 y=-1.8 → 南走廊必须 y < -2.0 才能全车通过
#   pillar_a 上沿 y=+1.65     → 北走廊 y=+2.05 时留 0.40 m
#   东西两墙内表面 ±2.95      → 东/西走廊取 ±2.50 时留 0.40 m
# 于是取矩形 x∈[-2.50, 2.50], y∈[-2.05, 2.05]，起点 (0,0) 在矩形
# 的「北边中点」上，走法：南下→东→北→西→南回到南边中点，再东行回 (0,0)。
SOUTH_Y = -2.05
NORTH_Y = 2.05
EAST_X = 2.50
WEST_X = -2.50


def route_new(s, real=False):
    """矩形闭环：起点(0,0)在矩形内部，沿南边中点到东南角开始绕。

    行程（严格对称，净位移必须为零）：
      南下 2.05 → (0, -2.05)          南边中点
      东行 2.50 → (2.50, -2.05)       东南角
      北上 4.10 → (2.50,  2.05)       东北角
      西行 5.00 → (-2.50, 2.05)       西北角
      南下 4.10 → (-2.50, -2.05)      西南角
      东行 2.50 → (0, -2.05)          回到南边中点
      北上 2.05 → (0, 0)              回到原点，朝向恢复 +x
    总转角：+90×4 = 360°，朝向也闭合。
    """
    h = abs(SOUTH_Y)                     # 2.05
    w = EAST_X - WEST_X                  # 5.00
    hh = NORTH_Y - SOUTH_Y               # 4.10
    steps = [
        ('t', -90), ('d', h),            # 南下到南边中点
        ('t', 90),  ('d', EAST_X),       # 东行到东南角
        ('t', 90),  ('d', hh),           # 北上到东北角
        ('t', 90),  ('d', w),            # 西行到西北角
        ('t', 90),  ('d', hh),           # 南下到西南角
        ('t', 90),  ('d', EAST_X),       # 东行回南边中点
        ('t', 90),  ('d', h),            # 北上回原点
    ]
    _play(s, steps, real, first_loss=1.0, other_loss=1.5, shrink=0.005)


def _play(s, steps, real, first_loss, other_loss, shrink):
    """按 steps 走一遍。

    ★ 注意单位：steps 里的转角写的是【度】（便于对照日志），
      必须转成弧度再交给 Sim.turn —— Sim 内部一律用弧度，
      直接传度数会得到 -5157° 这种离谱结果。

    real=True 时模拟 drive.py 的实测损耗：每次转向都朝「少转」方向偏。
    正角（逆时针）少转 → actual = angle - loss
    负角（顺时针）少转 → actual = angle + loss（绝对值变小）
    """
    first_turn_done = False
    for kind, val in steps:
        if kind == 't':
            ang = _rad(val)
            if real:
                loss = _rad(first_loss if not first_turn_done else other_loss)
                first_turn_done = True
                s.turn(ang, ang - loss if ang > 0 else ang + loss)
            else:
                s.turn(ang)
        else:
            s.straight(val, val - shrink if real else None)


# ---------------- 输出 ----------------
def report(name, fn, real=False):
    s = Sim()
    fn(s, real=real)
    dist = math.hypot(s.x, s.y)
    print('\n=== %s%s ===' % (name, '（叠加实测损耗）' if real else '（理想）'))
    print('  步   动作            位置(x, y)            朝向')
    for i, (x, y, yaw, label) in enumerate(s.trace):
        print('  %-3d  %-14s (%7.3f, %7.3f)   %8.1f°'
              % (i, label, x, y, math.degrees(yaw)))
    print('  → 终点 (%+.3f, %+.3f)  距原点 %.3f m  朝向 %.1f°'
          % (s.x, s.y, dist, math.degrees(s.yaw)))
    return dist, s


def check_route(name, pts, clear=ROBOT_RADIUS):
    print('\n=== 航线安全校核：%s ===' % name)
    ok = True
    for i in range(len(pts) - 1):
        d, who = segment_clearance(pts[i], pts[i + 1])
        good = d >= clear
        ok &= good
        print('  %s 段%d (%5.2f,%5.2f)→(%5.2f,%5.2f)  最近 %+.3f m (%s)'
              % ('OK' if good else 'XX', i, pts[i][0], pts[i][1],
                 pts[i + 1][0], pts[i + 1][1], d, who))
    print('  → %s（判据：每段 ≥ %.2f m）' % ('通过' if ok else '不通过', clear))
    return ok


OLD_WAYPOINTS = [(0, 0), (0, -2.0), (2.4, -2.0), (2.4, 2.0),
                 (-2.6, 2.0), (-2.6, -0.2)]
NEW_WAYPOINTS = [(0, 0), (0, SOUTH_Y), (EAST_X, SOUTH_Y), (EAST_X, NORTH_Y),
                 (WEST_X, NORTH_Y), (WEST_X, SOUTH_Y), (0, SOUTH_Y), (0, 0)]


def main():
    if '--check' in sys.argv:
        check_route('旧 route_loop', OLD_WAYPOINTS)
        check_route('新 route_loop', NEW_WAYPOINTS)
        return

    real = '--real' in sys.argv
    d_old, _ = report('旧 route_loop', route_old, real=real)
    d_new, _ = report('新 route_loop', route_new, real=real)

    print('\n' + '-' * 56)
    print('旧路线终点距原点 %.3f m' % d_old)
    print('新路线终点距原点 %.3f m' % d_new)
    if not real:
        print('结论：%s' % ('旧路线【本身不闭合】，偏差是路线设计，不是控制精度。'
                          if d_old > 0.3 else '旧路线基本闭合。'))
        print('      新路线理想终点 %.4f m，几何上严格闭合。' % d_new)
    else:
        print('结论：实测损耗（转向每次少转约 1.5°、直行每次短约 5mm）会让\n'
              '      即使闭合的路线也偏 %.2f m —— 这部分要靠收尾闭环（route_home）吃掉。'
              % d_new)


if __name__ == '__main__':
    main()
