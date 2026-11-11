#!/usr/bin/env python3
"""nav_precheck.py —— 导航模式开跑前的一次性体检，全绿再发目标。

为什么写这个：
  live.sh nav 里的流程是「起 Nav2 → sleep 12 → 发 AMCL 初值」。
  sleep 12 是个赌注：机器忙的时候 AMCL 的 lifecycle 还没 active，
  /initialpose 发出去没人接，消息直接丢。表现是 AMCL 粒子云撒在
  地图原点附近，然后 /planning 一路 abort —— 但日志里只会写
  「no valid path」，看起来像规划器坏了，实际是初值没生效。

  所以别赌时间，直接问节点：你到底 active 了没有。

检查项（任一 FAIL 就不该继续往下跑）：
  ① /map 有没有（map_server 起来了吗）
  ② 地图尺寸/原点/分辨率（和 room_geom 的真值对不对得上）
  ③ TF map→odom→base_footprint 是否连成链
  ④ AMCL / planner_server / controller_server / bt_navigator 是否 active
  ⑤ TF 链算出来的车位置，和 /odom 是否自洽（AMCL 收敛判据）
  ⑥ 车当前所在格的代价（是不是已经陷进膨胀区 / 致命区）

用法：
  source ~/navbot_ws/tools/verify/sim_fullchain/env.sh
  python3 nav_precheck.py            # 只体检
  python3 nav_precheck.py --wait 90  # 最多等 90 秒等节点 active（懒人模式）
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, '/home/liang/navbot_ws/tools/verify/sim_fullchain')
import room_geom as G   # noqa: E402

LIFECYCLE_NODES = ['amcl', 'planner_server', 'controller_server',
                   'bt_navigator', 'behavior_server', 'waypoint_follower']

OK, BAD, WARN = 'PASS', 'FAIL', 'WARN'
_rows = []


def rec(level, label, detail=''):
    _rows.append((level, label, detail))
    print('  %-4s %-40s %s' % (level, label, detail))
    return level == OK


def selftest():
    """离线自测：不连 ROS，只验「不依赖 rclpy 的那部分逻辑」。

    能自测的：地图覆盖判定、格代价查询、净空与真值的一致性。
    不能自测的：lifecycle 查询（要真节点）、TF 链（要真变换）——
    那两项只能在真机上跑，所以它们只做「读数并打印」，不做假设。
    """
    import map_quality as MQ
    print('=== nav_precheck --selftest（离线）===')
    fails = []

    def chk(cond, msg):
        print(('  PASS ' if cond else '  FAIL ') + msg)
        if not cond:
            fails.append(msg)

    m = MQ.load_pgm(
        '/home/liang/navbot_ws/src/navbot_bringup/maps/navbot_room.pgm',
        *MQ._read_yaml('/home/liang/navbot_ws/src/navbot_bringup/maps/'
                       'navbot_room.pgm'))

    # 覆盖判定：这份正式地图必须覆盖整个房间
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    mx1 = ox + m.info.width * m.info.resolution
    my1 = oy + m.info.height * m.info.resolution
    chk(ox <= G.X_MIN + 0.10 and mx1 >= G.X_MAX - 0.10 and
        oy <= G.Y_MIN + 0.10 and my1 >= G.Y_MAX - 0.10,
        '正式地图覆盖房间 x[%.2f,%.2f] y[%.2f,%.2f]' % (ox, mx1, oy, my1))

    # 代价查询：墙外一点必须 lethal，房间中心必须 free
    chk(cell_cost(m, 0.0, 0.0)[0] == 'free', '房间中心 (0,0) 判 free')
    chk(cell_cost(m, G.X_MAX + 1.0, 0.0)[0] == 'out',
        '房间外 判 out（不是被当成 free）')
    # 南墙内表面往里 2cm 一定是致命
    chk(cell_cost(m, 0.0, G.Y_MIN + 0.02)[0] in ('lethal', 'unknown'),
        '贴南墙 2cm 判 lethal/unknown')

    # 路线点全部必须 free —— 否则真跑时车一出发就坐在致命格里
    import nav_patrol as NP
    for name, x, y, _mode in NP.ROUTE:
        sem = cell_cost(m, x, y)[0]
        chk(sem == 'free', '%-10s (%+.2f,%+.2f) 判 %s' % (name, x, y, sem))

    print('\n%s' % ('全部 PASS' if not fails else '%d 项 FAIL' % len(fails)))
    return 0 if not fails else 1


def main():
    if '--selftest' in sys.argv:
        return selftest()
    wait = 0.0
    if '--wait' in sys.argv:
        i = sys.argv.index('--wait')
        wait = float(sys.argv[i + 1]) if len(sys.argv) > i + 1 else 60.0

    rclpy.init()
    node = rclpy.create_node('nav_precheck')
    box = {}
    node.create_subscription(
        OccupancyGrid, '/map', lambda m: box.setdefault('map', m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(
        Odometry, '/odom', lambda m: box.__setitem__('odom', m),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose',
        lambda m: box.__setitem__('amcl', m), 10)
    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    # ---------------------------------------------------------- ① /map
    print('\n[1] 地图')
    t0 = time.time()
    while 'map' not in box and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.3)
    m = box.get('map')
    if m is None:
        rec(BAD, '/map 20 秒内没收到', 'map_server 没起来？看 live_nav.log')
        return finish(node)
    rec(OK, '/map 已收到',
        '%d×%d res=%.3f' % (m.info.width, m.info.height,
                            m.info.resolution))
    ox = m.info.origin.position.x
    oy = m.info.origin.position.y
    rec(OK if abs(m.info.resolution - 0.05) < 1e-6 else WARN,
        '分辨率 %.3f' % m.info.resolution, '期望 0.05')
    n_unk = m.data.count(-1)
    rec(OK if 100.0 * n_unk / len(m.data) < 15 else WARN,
        '未知区 %.1f%%' % (100.0 * n_unk / len(m.data)), '期望 < 15%')
    # 地图必须能覆盖整个房间
    mx1 = ox + m.info.width * m.info.resolution
    my1 = oy + m.info.height * m.info.resolution
    covers = (ox <= G.X_MIN + 0.10 and mx1 >= G.X_MAX - 0.10 and
              oy <= G.Y_MIN + 0.10 and my1 >= G.Y_MAX - 0.10)
    if covers:
        rec(OK, '地图覆盖整个房间',
            'x[%.2f,%.2f] y[%.2f,%.2f]' % (ox, mx1, oy, my1))
    else:
        rec(BAD, '地图没覆盖全房间',
            '地图 x[%.2f,%.2f] y[%.2f,%.2f] vs 房间 x[%.2f,%.2f] y[%.2f,%.2f]'
            % (ox, mx1, oy, my1, G.X_MIN, G.X_MAX, G.Y_MIN, G.Y_MAX))

    # ---------------------------------------------------------- ③ TF 链
    # ---------------------------------------------------------- ④ lifecycle
    print('\n[2] 生命周期节点')
    # 用 `ros2 lifecycle get` 的子进程比 rclpy 直连稳（服务名带命名空间时省事）。
    #
    # ★★ 2026-11-10 踩坑：`ros2 lifecycle get` 的输出是 "active [3]"，
    #    不是单纯的 "active"。原来取 split()[-1] 拿到的是 "[3]"，
    #    永远 != 'active'，于是 --wait 90 每次都硬等满 90 秒，
    #    界面上就是「停在这不动了」。
    #    现在取 split()[0]，并用 --verbose 打印原始输出，别再靠猜。
    import subprocess
    verbose = '--verbose' in sys.argv

    def query(n, timeout=6):
        """返回 (原始输出, 状态名)。查不到时状态名为空串。"""
        try:
            r = subprocess.run(['ros2', 'lifecycle', 'get', '/' + n],
                               capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return '<timeout>', ''
        except Exception as ex:                      # noqa: BLE001
            return '<%s>' % ex.__class__.__name__, ''
        raw = (r.stdout or r.stderr or '').strip()
        first = raw.split()[0] if raw.split() else ''
        return raw, first

    t0 = time.time()
    states, raws = {}, {}
    while True:
        for n in LIFECYCLE_NODES:
            if states.get(n) == 'active':
                continue
            raw, st = query(n)
            raws[n] = raw
            if st:
                states[n] = st
        pending = [n for n in LIFECYCLE_NODES if states.get(n) != 'active']
        if not pending or time.time() - t0 > wait:
            break
        # 每轮打印一行，让人看见它在动，不是卡死
        print('      ...等待 %s（已等 %.0fs / 上限 %.0fs）'
              % (','.join(pending), time.time() - t0, wait))
        time.sleep(2)

    for n in LIFECYCLE_NODES:
        st = states.get(n, '')
        detail = '' if st else '原始输出: %s' % (raws.get(n, '<无>')[:60])
        if verbose and st:
            detail = '原始输出: %s' % raws.get(n, '')
        rec(OK if st == 'active' else (BAD if not st else WARN),
            '%-20s %s' % (n, st or '未响应'), detail)

    # ---------------------------------------------------------- ③ TF
    print('\n[3] TF 链')
    for pair in [('map', 'odom'), ('odom', 'base_footprint'),
                 ('map', 'base_footprint')]:
        tf = None
        t0 = time.time()
        while time.time() - t0 < 8:
            try:
                tf = tf_buf.lookup_transform(pair[0], pair[1],
                                             rclpy.time.Time())
                break
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.2)
        if tf is None:
            # ★★ 分情况给出可操作的诊断。原来一律写「slam_toolbox 还在跑？」
            #    是错的 —— 2026-11-10 实际踩到的是**另一个**原因：
            #    map_server 没 activate → 没 /map → AMCL 收不到地图，
            #    一直喊 "Please set the initial pose"，不发布 map->odom。
            #    报错信息和真实原因差得远，所以这里把两种都列出来。
            hint = '查不到'
            if pair[0] == 'map' and m is None:
                hint = '没有 /map → map_server 没 active'
            elif pair[0] == 'map':
                hint = ('AMCL 没发布 map->odom。最常见：**初始位姿没发进去**'
                        '（AMCL 日志会喊 Please set the initial pose）；'
                        '其次是 slam_toolbox 还在跑抢发布者')
            rec(BAD, '%s -> %s' % pair, hint)
        else:
            tr = tf.transform.translation
            rec(OK, '%s -> %s' % pair,
                '(%.3f, %.3f)' % (tr.x, tr.y))

    # ---------------------------------------------------------- ⑤ 位姿自洽
    print('\n[4] 位姿自洽（AMCL 收敛判据）')
    t0 = time.time()
    while ('odom' not in box or 'amcl' not in box) and time.time() - t0 < 12:
        rclpy.spin_once(node, timeout_sec=0.3)
    od = box.get('odom')
    am = box.get('amcl')

    # ★★ /amcl_pose 是**事件驱动**的：AMCL 只在车移动超过 update_min_d=0.1 m
    #    或转动超过 update_min_a=0.1 rad 时才发布一次。
    #    车静止时这个话题上一条消息都没有，拿「收没收到」当判据必然误报
    #    （2026-11-10 实测：车原地不动，/amcl_pose 5 秒 0 条，但 TF 完全正常）。
    #    所以主判据改成 TF 链 —— 它是 20Hz 连续发布的，与车动不动无关。
    tf_mb = tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()) \
        if _can_lookup(tf_buf) else None
    if od is not None and tf_mb is not None:
        ox_, oy_ = od.pose.pose.position.x, od.pose.pose.position.y
        ax_ = tf_mb.transform.translation.x
        ay_ = tf_mb.transform.translation.y
        d = math.hypot(ax_ - ox_, ay_ - oy_)
        rec(OK if d < 0.30 else BAD,
            'map 位姿与 odom 偏差 %.3f m' % d,
            'map(%.2f,%.2f) odom(%.2f,%.2f)  判据 < 0.30' % (ax_, ay_, ox_, oy_))
    else:
        rec(BAD, '拿不到 map→base_footprint 或 /odom', '')

    if am is None:
        rec(WARN, '/amcl_pose 暂无消息（车静止时不发，属正常）',
            'update_min_d=0.1 才触发，跑起来后会有')
    else:
        cov = am.pose.covariance
        rec(OK if cov[0] < 0.10 and cov[7] < 0.10 else WARN,
            'AMCL 位置方差 %.3f / %.3f' % (cov[0], cov[7]),
            '判据 < 0.10（大 = 粒子还散着，初值可能没发进去）')

    # ---------------------------------------------------------- ⑥ 当前位置代价
    print('\n[5] 车当前所在格')
    if od is not None:
        chk = cell_cost(m, od.pose.pose.position.x, od.pose.pose.position.y)
        rec(OK if chk[0] == 'free' else BAD,
            '车位置格代价 %s' % chk[0],
            '%.2f,%.2f → cell=%s' % (od.pose.pose.position.x,
                                     od.pose.pose.position.y, chk[1]))
        clr = G.clearance(od.pose.pose.position.x, od.pose.pose.position.y)
        rec(OK if clr > G.ROBOT_RADIUS else BAD, '净空 %.3f m' % clr,
            '判据 > 车半径 %.2f' % G.ROBOT_RADIUS)

    return finish(node)


def _can_lookup(tf_buf):
    """能否查到 map→base_footprint。查得到 True，查不到 False。"""
    try:
        tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        return True
    except Exception:      # noqa: BLE001
        return False


def cell_cost(m, x, y):
    """返回 (语义, 原始值)。语义：free / unknown / lethal。"""
    res = m.info.resolution
    ix = int((x - m.info.origin.position.x) / res)
    iy = int((y - m.info.origin.position.y) / res)
    if not (0 <= ix < m.info.width and 0 <= iy < m.info.height):
        return 'out', None
    v = m.data[iy * m.info.width + ix]
    return ({0: 'free', -1: 'unknown', 100: 'lethal'}).get(v, str(v)), v


def finish(node):
    node.destroy_node()
    rclpy.shutdown()
    n_bad = sum(1 for r in _rows if r[0] == BAD)
    n_warn = sum(1 for r in _rows if r[0] == WARN)
    print('\n' + '=' * 70)
    if n_bad:
        print(' 结论：%d 项 FAIL，%d 项 WARN —— 先别发目标，修掉再跑' % (n_bad, n_warn))
        for lv, label, detail in _rows:
            if lv == BAD:
                print('   FAIL  %s  %s' % (label, detail))
        return 1
    print(' 结论：全部通过（%d 项 WARN）—— 可以跑 nav_patrol.py' % n_warn)
    return 0


if __name__ == '__main__':
    sys.exit(main())
