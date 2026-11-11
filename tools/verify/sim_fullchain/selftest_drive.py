#!/usr/bin/env python3
"""drive.py 的离线自测：不连 ROS，用假的 Node 复现 turn / straight / route_loop。

为什么要写这个：
  上一轮 drive.py 的 bug（把 mode 当字典键、路线不闭合、turn 少转）
  都是「跑真车才发现」。真车一轮 2 分钟，改一次验一次太慢。
  这里用一阶惯性模型顶替 Gazebo，能在秒级验证控制律与路线几何。

用法：
  python3 selftest_drive.py            # 全部用例
  python3 selftest_drive.py turn       # 只测 turn

注意：
  本脚本只验证【逻辑与几何】，不验证 Gazebo 的物理细节。
  它通过说明「代码按设计意图执行」，最终仍需真机跑一轮确认。
"""
import math
import sys
import types

DT = 0.05


# ---------------- 用假模块顶替 ROS，好让 drive.py 能被 import ----------------
def _install_fake_ros():
    if 'rclpy' in sys.modules:
        return
    class _Any:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            return _Any()

        def __call__(self, *a, **k):
            return _Any()

    class _Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class _Pose:
        def __init__(self):
            self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.orientation = types.SimpleNamespace(
                x=0.0, y=0.0, z=0.0, w=1.0)

    class Node:
        def __init__(self, *a, **k):
            pass

        def get_logger(self):
            return types.SimpleNamespace(
                info=lambda m: None, warn=lambda m: None,
                error=lambda m: None, debug=lambda m: None)

        def create_publisher(self, *a, **k):
            return None

        def create_subscription(self, *a, **k):
            return None

    fake = {
        'rclpy': types.SimpleNamespace(
            init=lambda *a, **k: None, shutdown=lambda *a, **k: None,
            ok=lambda: True, spin_once=lambda *a, **k: None),
        'rclpy.node': types.SimpleNamespace(Node=Node),
        'rclpy.qos': types.SimpleNamespace(
            QoSProfile=_Any,
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)),
        'rclpy.executors': types.SimpleNamespace(
            ExternalShutdownException=type('ExternalShutdownException',
                                           (Exception,), {})),
        'geometry_msgs': types.SimpleNamespace(),
        'geometry_msgs.msg': types.SimpleNamespace(Twist=_Twist),
        'nav_msgs': types.SimpleNamespace(),
        'nav_msgs.msg': types.SimpleNamespace(
            Odometry=type('Odometry', (), {
                '__init__': lambda self: setattr(self, 'pose', _Pose()),
            })),
    }
    for name, mod in fake.items():
        sys.modules.setdefault(name, mod)


# ---------------- 被控对象：一阶惯性 ----------------
class Plant:
    """同一套模型喂给 turn 与 straight。"""

    def __init__(self, tau=0.12, vmax=10.0):
        self.tau = tau
        self.h_lin = 0.0
        self.h_ang = 0.0
        self.x = self.y = self.yaw = 0.0
        self.vmax = vmax

    def step(self, v_cmd, w_cmd):
        self.h_lin += (v_cmd - self.h_lin) * DT / self.tau
        self.h_ang += (w_cmd - self.h_ang) * DT / self.tau
        self.x += self.h_lin * math.cos(self.yaw) * DT
        self.y += self.h_lin * math.sin(self.yaw) * DT
        self.yaw = _wrap(self.yaw + self.h_ang * DT)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ---------------- 把 drive.py 的算法搬到模拟器上跑 ----------------
def sim_turn(plant, angle, omega=0.6, tol=0.004):
    """与 drive.py 的 Driver.turn 同构（只把 _pump 换成 plant.step）。"""
    accum, prev = 0.0, plant.yaw
    steps = 0
    # 粗转 + 精转
    while True:
        remain = abs(angle) - abs(accum)
        if remain <= tol:
            break
        if remain < abs(angle) * 0.25:
            w = omega * 0.4 * max(0.15, remain / (abs(angle) * 0.25))
        else:
            w = omega
        plant.step(0.0, w if angle > 0 else -w)
        accum += _wrap(plant.yaw - prev)
        prev = plant.yaw
        steps += 1
        if steps > 4000:
            raise RuntimeError('turn 不收敛')
    # stop() + _pump(0.25) 刹停
    for _ in range(int(0.25 / DT)):
        plant.step(0.0, 0.0)
    accum += _wrap(plant.yaw - prev)
    return accum


def sim_straight(plant, dist, speed=0.15, tol=0.02):
    """与 drive.py 的 Driver.straight 同构。"""
    sign = 1.0 if dist >= 0 else -1.0
    d = abs(dist)
    x0, y0 = plant.x, plant.y
    steps = 0
    while True:
        moved = math.hypot(plant.x - x0, plant.y - y0)
        if moved >= d - tol:
            break
        remain = d - moved
        v = sign * max(0.05, min(speed, 0.6 * remain + 0.05))
        plant.step(v, 0.0)
        steps += 1
        if steps > 4000:
            raise RuntimeError('straight 不收敛')
    for _ in range(int(0.25 / DT)):
        plant.step(0.0, 0.0)
    return math.hypot(plant.x - x0, plant.y - y0)


# ---------------- 用例 ----------------
RESULTS = []


def case(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print('  %s %-38s %s' % ('PASS' if ok else 'FAIL', name, detail))


def t_turn_accuracy():
    print('\n[1] turn 单次精度')
    for deg in (-90, 90, 90, 90, 90, 90, 90):
        p = Plant()
        a = math.radians(deg)
        got = sim_turn(p, a)
        err = math.degrees(a - got)
        case('turn %+d°' % deg, abs(err) < 0.3, '残差 %+.2f°' % err)


def t_turn_accumulate():
    print('\n[2] turn 7 次累积残差（闭环路线要求）')
    p = Plant()
    total = 0.0
    for deg in (-90, 90, 90, 90, 90, 90, 90):
        a = math.radians(deg)
        total += math.degrees(a - sim_turn(p, a))
    case('7 次转向累计残差', abs(total) < 1.0, '%+.2f°' % total)


def t_straight_accuracy():
    print('\n[3] straight 精度')
    for d in (2.05, 2.50, 4.10, 5.00):
        p = Plant()
        got = sim_straight(p, d)
        case('straight %.2fm' % d, abs(got - d) < 0.03,
             '实际 %.3fm 误差 %+.3f' % (got, got - d))


def _sim_rectangle(plant, reverse=False):
    """复刻 drive.py 的 _drive_rectangle，验证正/反向都能闭合。"""
    S, N, E, Wt = -2.05, 2.05, 2.50, -2.50
    h = abs(S)
    hh = N - S
    w = E - Wt
    first_x = E if not reverse else abs(Wt)
    sgn = 1 if not reverse else -1
    sim_turn(plant, sgn * -math.pi / 2); sim_straight(plant, h)
    sim_turn(plant, sgn * math.pi / 2);  sim_straight(plant, first_x)
    sim_turn(plant, sgn * math.pi / 2);  sim_straight(plant, hh)
    sim_turn(plant, sgn * math.pi / 2);  sim_straight(plant, w)
    sim_turn(plant, sgn * math.pi / 2);  sim_straight(plant, hh)
    sim_turn(plant, sgn * math.pi / 2);  sim_straight(plant, first_x)
    sim_turn(plant, sgn * math.pi / 2);  sim_straight(plant, h)


def t_route_loop_closure():
    print('\n[4] 新 route_loop 闭环（一阶惯性模型，含 turn 残差）')
    p = Plant()
    _sim_rectangle(p, reverse=False)
    err = math.hypot(p.x, p.y)
    case('正向矩形终点距原点 < 0.05m', err < 0.05,
         '终点 (%+.3f, %+.3f) 距原点 %.3f m' % (p.x, p.y, err))
    yaw_after = math.degrees(_wrap(p.yaw))
    case('正向矩形终点朝向 = +90°（净转 +540°）', abs(yaw_after - 90) < 3.0,
         '朝向 %+.1f°' % yaw_after)
    # 再执行 _finish_at_origin 里的「掰回 +x」段
    sim_turn(p, _wrap(-p.yaw))
    case('掰正后朝向回到 +x（< 1°）', abs(math.degrees(_wrap(p.yaw))) < 1.0,
         '朝向 %+.2f°' % math.degrees(_wrap(p.yaw)))
    case('掰正不破坏位置（< 0.05m）', math.hypot(p.x, p.y) < 0.05,
         '终点 (%+.3f, %+.3f)' % (p.x, p.y))

    print('\n[4b] route_survey 的两圈（正 + 反）')
    p2 = Plant()
    _sim_rectangle(p2, reverse=False)
    e1 = math.hypot(p2.x, p2.y)
    case('第 1 圈后距原点 < 0.05m', e1 < 0.05, '%.3f m' % e1)
    _sim_rectangle(p2, reverse=True)
    e2 = math.hypot(p2.x, p2.y)
    case('第 2 圈（反向）后距原点 < 0.10m', e2 < 0.10, '%.3f m' % e2)


def t_old_route_reproduce():
    print('\n[5] 旧 route_loop 复现（证明偏差来自路线本身）')
    p = Plant()
    seq = [(-math.pi / 2, None), (None, 2.0),
           (math.pi / 2, None), (None, 2.4),
           (math.pi / 2, None), (None, 4.0),
           (math.pi / 2, None), (None, 5.0),
           (math.pi / 2, None), (None, 2.2)]
    for ang, dist in seq:
        if ang is not None:
            sim_turn(p, ang)
        else:
            sim_straight(p, dist)
    err = math.hypot(p.x, p.y)
    case('旧路线终点 > 2.0m（应失败）', err > 2.0,
         '终点 (%+.3f, %+.3f) 距原点 %.3f m' % (p.x, p.y, err))
    # 实测值对照
    case('与实测 (-2.616,-0.195) 同量级',
         abs(p.x - (-2.616)) < 0.5 and abs(p.y - (-0.195)) < 0.5,
         '模拟 x=%+.3f y=%+.3f vs 实测 -2.616 -0.195' % (p.x, p.y))


def t_route_names():
    print('\n[6] 路线别名与公共函数')
    _install_fake_ros()
    sys.path.insert(0, '.')
    try:
        import drive
        for r in ('loop', 'survey', 'home'):
            case('route_%s 可调用' % r, callable(getattr(drive, 'route_' + r, None)),
                 'drive.route_%s' % r)
        case('_drive_rectangle 存在', callable(getattr(drive, '_drive_rectangle', None)), '')
        case('_finish_at_origin 存在', callable(getattr(drive, '_finish_at_origin', None)), '')
        # 常量必须与 loop_geom.py 一致，否则几何仿真验的就不是实际跑的路线
        import loop_geom
        same = (drive.SOUTH_Y == loop_geom.SOUTH_Y
                and drive.NORTH_Y == loop_geom.NORTH_Y
                and drive.EAST_X == loop_geom.EAST_X
                and drive.WEST_X == loop_geom.WEST_X)
        case('drive 与 loop_geom 的走廊常量一致', same,
             'drive=%s loop_geom=%s'
             % ((drive.SOUTH_Y, drive.NORTH_Y, drive.EAST_X, drive.WEST_X),
                (loop_geom.SOUTH_Y, loop_geom.NORTH_Y,
                 loop_geom.EAST_X, loop_geom.WEST_X)))
    except Exception as e:
        case('import drive', False, repr(e))


if __name__ == '__main__':
    only = sys.argv[1] if len(sys.argv) > 1 else None
    tests = [
        ('turn', t_turn_accuracy), ('turnacc', t_turn_accumulate),
        ('straight', t_straight_accuracy), ('loop', t_route_loop_closure),
        ('old', t_old_route_reproduce), ('names', t_route_names),
    ]
    for key, fn in tests:
        if only in (None, key):
            fn()
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    print('\n' + '=' * 52)
    print('合计 %d 项，PASS %d，FAIL %d'
          % (len(RESULTS), npass, len(RESULTS) - npass))
    sys.exit(0 if npass == len(RESULTS) else 1)
