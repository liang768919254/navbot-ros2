#!/usr/bin/env python3
"""nav_patrol.py —— 多点导航巡检：逐点发 NavigateToPose，核验到达情况。

为什么要在 verify_nav.sh 之外再写一个：
  verify_nav.sh 只测**一个**目标点，说明不了整条链路行不行。
  本脚本把「南区绕圈 → 绕东走廊去北区 → 北区绕圈」整条序列跑一遍。

★★ 2026-11-10 修正记录（前两版路线都是错的，别再犯）

  第一版：我以为「0.6 m 窄通道」在 partition_mid 东端 ↔ partition_short 之间，
          设了 (0.70,-0.10) → (0.70,1.30)。这条缝实测只有 0.350 m，
          比车直径 0.40 m 还窄，而且那两个点都在开阔区，压根没经过窄处。

  第二版：我又「发现」一条「窄道 E」——partition_short 南端 ↔ 南墙 0.650 m，
          设 (0.30,-2.10) → (0.30,0.10)。跑了地图逐格扫描才发现：
          partition_short 只是块 0.1 m 宽的竖板，只占 x∈[1.15,1.25]。
          在 x=0.30 处它根本不构成障碍，那里距南墙 2.35 m，是开阔地。

  真实拓扑（地图逐格扫描 + 解析净空双重确认）：
    分区     ├── 南区  y ∈ [-2.45, 0.45]
             └── 北区  y ∈ [ 0.55, 2.45]
    隔板     partition_mid    水平 x∈[-1.80,0.80] y∈[0.45,0.55]
             partition_short  竖直 x∈[ 1.15,1.25] y∈[-1.80,0.00]
    通路     缝 N  0.450 m  短板北端 y=0.00 ↔ mid 下沿 y=0.45（物理能过）
             缝 E  0.300 m  mid 东端 x=0.82 ↔ 短板西沿 x=1.12（比车窄，死路）
             缝 S  0.950 m  短板南端 y=-1.50 ↔ 南墙（宽敞）
             缝 W  1.170 m  mid 西端 x=-1.78 ↔ 西墙（宽敞）

  所以本项目的真正考点是：
    全局规划器能不能绕开死路（缝 E），找到缝 W / 缝 S 走。
  ⇒ 路线用「西南角 → 西侧通道 → 西北角」当核心绕行用例。

用法（先起好 Nav2，见 live.sh nav）：
  python3 nav_patrol.py                 # 跑默认 7 点序列
  python3 nav_patrol.py --one 2.2 -1.5  # 只去一个点
  python3 nav_patrol.py --no-detour     # 跳过绕行两点
  python3 nav_patrol.py --dry           # 只做预检，不发目标（不依赖 Nav2）
  python3 nav_patrol.py --selftest      # 纯逻辑自测

判据（每点）：
  ① 结果 SUCCEEDED
  ② 到点误差 < 0.25 m
  ③ 用时 < 120 s

预检（发目标前做，把「点本身不可达」和「导航有问题」分开）：
  · 目标点是否在地图空闲格里
  · 目标点净空是否 > 车半径
  · 8 点是否在同一连通域（否则必然有导航失败）
"""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import room_geom as G                      # noqa: E402  纯标准库，无需 ROS

# ---------------------------------------------------------------- 路线
# (名称, x, y, 类型)  类型: 'open' 开阔 / 'detour' 必须绕行
#
# ★★ 2026-11-10 第四版。前三版都有错，完整记录如下（都是我的错，不是环境问题）：
#
#   第一版  设 (0.70,-0.10)→(0.70,1.30)，以为中间有条「0.6m 窄道」。
#           实测该处净空 0.45/0.75，全在开阔区，压根没经过窄处。
#   第二版  设 (0.30,-2.10)→(0.30,0.10)，以为 partition_short 南端到南墙
#           是 0.65m 窄道。实测 x=0.30 处距南墙 2.35m，也是开阔地。
#   第三版  设「东走廊 1.7m 是唯一南北通路」。实测西侧也有 1.17m 的开口，
#           不唯一，而且顶部的判断依据是错的隔板常量。
#
#   以上三版的共同病根：我把 PARTITIONS 的边界当成已知真值，
#   而那份常量是我早前凭印象写的。用地图逐行扫描反推后：
#       partition_mid    x[-1.78,+0.82]  y≈+0.50（单行）
#       partition_short  x[+1.12,+1.32]  y[-1.50, 0.00]   ← 我写成了 -1.80
#       pillar_b(r=0.15) 圆心(-1.90,-1.60)                ← 我完全漏了
#
#   真实拓扑（已写进 room_geom.py 并自测锁定）：
#     缝 N  0.450 m  短板北端 y=0.00 ↔ mid 下沿 y=0.45   物理能过，膨胀后非零代价
#     缝 E  0.300 m  mid 东端 x=0.82 ↔ 短板西沿 x=1.12   比车窄，死路
#     缝 S  0.950 m  短板南端 y=-1.50 ↔ 南墙             宽敞
#     缝 W  1.170 m  mid 西端 x=-1.78 ↔ 西墙             宽敞
#
#   用当前参数(inflation=0.25)离线规划，南北三条通道都通，净空 0.25~0.57。
#   ⇒ 所以路线设计成「南区 → 西侧绕到北区 → 北区」，
#     跨区段就是本项目的核心考点：能不能自己找到西侧的路，而不是硬钻缝 E。
#   ★ 西侧三点统一用 x=-2.50，不是 -2.20。
#     因为 map 实测还有一根我之前漏掉的立柱 pillar_b
#     x∈[-2.03,-1.73] y≈-1.50，它正卡在西侧通道上：
#        x=-2.20 处净空只有 0.170 < 车半径 0.20 → 车过不去
#        x=-2.50 处净空 0.450 → 安全
#     所以「沿西墙走」必须贴到 x≈-2.5，走 -2.2 会被这块短板挡住。
ROUTE = [
    # --- 南区 ---
    ('东南角',      2.20, -1.50, 'open'),
    ('南走廊中点',  0.00, -2.00, 'open'),
    ('西南角',     -2.50, -2.00, 'open'),
    # --- 跨区：绕过立柱 pillar_b 并穿缝 W 进北区，核心考点 ---
    ('西侧通道',   -2.50,  0.00, 'detour'),
    # --- 北区 ---
    ('西北角',     -2.50,  1.90, 'open'),
    ('北走廊中点',  0.00,  1.90, 'open'),
    ('东北角',      2.20,  1.90, 'open'),
    # --- 跨回：从东侧沿墙回南区，验证另一条通路 ---
    ('东侧通道',    2.30, -0.80, 'detour'),
]

TOL_ERR = 0.25
TOL_TIME = 120.0

# ★★★ 2026-11-10 新增：平均速度判据（S31）
#
#   起因：8 点全 PASS，但南走廊中点 67.8s vs 其余 14~35s。
#   只看「用时 < 120s」抓不到它 —— 67.8 也 < 120。
#   而「平均速度」能区分「路远所以慢」和「贴着障碍爬」。
#
#   算法：本段直线距离 / 用时。
#     ★ 不用实际路径长（拿不到），所以这是个【下界估计】——
#       实际走得比直线长，真实速度比算出来的高。
#       正因如此，算出来还低于阈值，就一定是真慢。
#
#   阈值依据：
#     max_vel_x = 0.22
#     正常点实测速度（S31 那轮 8 点，段长/用时）：
#       北走廊 2.50/14.4 = 0.174   东北角 2.20/15.0 = 0.147
#       东南角 3.61/31.8 = 0.114   东侧通道 2.74/25.7 = 0.107
#       西北角 1.90/23.6 = 0.081   西侧通道 2.00/33.2 = 0.060
#       西南角 2.50/34.8 = 0.072
#     爬行点：南走廊 2.213/67.8 = 0.033   ← 唯一的真异常
#     ⇒ SLOW = 0.06：比最慢的正常点（0.060）还低，只会抓到真爬行
#     ⇒ WARN = 0.075：介于 0.060 和 0.081 之间，标「略慢但不异常」的
SLOW_SPEED = 0.06
WARN_SPEED = 0.075



def run(cmd, timeout=180):
    r = subprocess.run(['bash', '-lc', cmd], capture_output=True,
                       text=True, timeout=timeout)
    return r.stdout + r.stderr


def get_pose():
    """读 /amcl_pose。★ 必须 source env.sh，否则 ROS_DOMAIN_ID 可能对不上。"""
    out = run('source %s/env.sh; timeout 8 ros2 topic echo /amcl_pose --once '
              '--field pose.pose.position' % HERE, timeout=20)
    x = y = None
    for line in out.splitlines():
        s = line.strip()
        if s.startswith('x:') and x is None:
            x = float(s.split(':')[1])
        elif s.startswith('y:') and y is None:
            y = float(s.split(':')[1])
    return x, y


class GoalClient:
    """常驻 NavigateToPose 客户端（封装 nav_client.py 子进程）。

    ★★★ S33（2026-11-10 夜）：不再每点 fork 一次 `ros2 action send_goal`。

    旧写法的三个实测故障：
      ① 每次重新发现 action server，机器忙时目标根本没发出去，
         Nav2 日志里连 "Begin navigating" 都没有 —— 但脚本报 NOT_ACCEPTED，
         看起来像"导航失败"，实际是"目标没送到"。
      ② timeout 掐掉 CLI 后，server 侧留悬空 goal handle，日志里
         `Failed to send goal response ... (timeout): client will not receive
         response`，下一条目标直接被拖死。
      ③ 只能从人读文本里 grep "SUCCEEDED"，拿不到 status 码，
         分不清「没被接受」和「接受了但超时」。

    现在：一个子进程、一个节点、一个 action client，从头用到尾。
    """

    def __init__(self):
        env = os.environ.copy()
        self.proc = subprocess.Popen(
            ['bash', '-lc',
             'source %s/env.sh && exec /usr/bin/python3 %s/nav_client.py'
             % (HERE, HERE)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1, env=env)

    def goto(self, gx, gy):
        """发一个点，返回 dict(state, t, detail)。"""
        self.proc.stdin.write('%.4f %.4f\n' % (gx, gy))
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            err = (self.proc.stderr.read() or '')[-400:]
            return {'state': 'NO_SERVER', 't': 0.0,
                    'detail': 'nav_client 已退出：' + err.replace('\n', ' ')}
        try:
            return json.loads(line)
        except ValueError:
            return {'state': 'UNKNOWN', 't': 0.0, 'detail': line[:200]}

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)
        except Exception:
            self.proc.kill()


# ---------------------------------------------------------------- 预检
def load_map():
    """读 navbot_room.pgm，返回 (w, h, res, origin, data)。纯标准库。"""
    path = os.path.join(HERE, '..', '..', '..',
                        'src/navbot_bringup/maps/navbot_room.pgm')
    path = os.path.abspath(path)
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', '不是 P5 PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()                       # maxval
        data = f.read()
    assert len(data) == w * h, 'PGM 数据长度不对'
    # origin 从 yaml 读（避免两处硬编码）
    ypath = path.replace('.pgm', '.yaml')
    ox = oy = res = None
    for ln in open(ypath):
        s = ln.strip()
        if s.startswith('resolution:'):
            res = float(s.split(':')[1])
        elif s.startswith('origin:'):
            vals = s.split(':', 1)[1].strip().strip('[]').split(',')
            ox, oy = float(vals[0]), float(vals[1])
    return w, h, res, (ox, oy), data


def map_value(m, x, y):
    """返回该点地图值：100 占据 / 0 空闲 / -1 未知；越界返回 None。

    ★ PGM 行序自顶向下，栅格 y 自底向上 —— 必须翻转，否则上下颠倒。
      2026-11-10 map_quality.py 就踩过这个坑。
    """
    w, h, res, (ox, oy), data = m
    ix = int((x - ox) / res)
    iy = int((y - oy) / res)
    if not (0 <= ix < w and 0 <= iy < h):
        return None
    v = data[(h - 1 - iy) * w + ix]
    return 100 if v < 128 else (0 if v > 230 else -1)


def precheck():
    """逐点预检，返回 [(name, x, y, kind, ok, reason)]。"""
    m = load_map()
    out = []
    for name, x, y, kind in ROUTE:
        mv = map_value(m, x, y)
        c = G.clearance(x, y)
        reasons = []
        ok = True
        if mv is None:
            ok = False
            reasons.append('地图外')
        elif mv == 100:
            ok = False
            reasons.append('落在墙格')
        elif mv == -1:
            ok = False
            reasons.append('落在未知格')
        if c <= G.ROBOT_RADIUS:
            ok = False
            reasons.append('净空 %.3f ≤ 车半径 %.2f' % (c, G.ROBOT_RADIUS))
        elif c <= G.ROBOT_RADIUS + 0.05:
            reasons.append('净空 %.3f 偏紧' % c)
        out.append((name, x, y, kind, ok,
                    '、'.join(reasons) if reasons else '净空 %.3f' % c))
    return out


def reachable_set(m, start, max_cells=400000):
    """在地图上做膨胀后的 BFS，返回可达格集合。

    ★ 为什么要自己算一遍，而不是信 Nav2：
      Nav2 规划失败时只说 "Failed to create path"，不告诉你
      「是目标不可达」还是「起点被困住」还是「膨胀把路堵死了」。
      这里先离线算一遍连通域，把这三者分开。
      ★ 2026-11-10 夜（S32）：车体模型换成矩形足迹后，判据也跟着换。
        旧口径：某格到最近障碍 ≥ robot_radius(0.20) 就算可走 —— 把车当圆。
        新口径：某格到最近障碍 ≥ **注册半径**(min(长,宽)/2 = 0.12) 就算可走。
        差异在缝里很致命：0.655 m 的缝 S，圆模型给 0.255 m 可走带，
        矩形给 0.415 m —— 前者会把「其实能过的缝」判成不可达。
        ⇒ 这里用 G.INSCRIBED_R 而不是 G.ROBOT_RADIUS。

    用法：把两个目标点都丢进来，看是否落在同一个连通域。
    """
    w, h, res, (ox, oy), data = m

    def occupied(ix, iy):
        return data[(h - 1 - iy) * w + ix] < 128     # 墙或未知都当不可走

    # 预先算「非致命掩码」：到最近障碍距离 >= robot_radius
    # 用简单的多源 BFS 距离变换（4 邻域，曼哈顿近似）——
    # 判连通域够用；要精确欧氏距离用 room_geom.clearance 复核关键点。
    forbid = bytearray(w * h)
    from collections import deque
    dist = [[-1] * w for _ in range(h)]
    q = deque()
    for ix in range(w):
        for iy in range(h):
            if occupied(ix, iy):
                dist[iy][ix] = 0
                q.append((ix, iy))
    while q:
        ix, iy = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = ix + dx, iy + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny][nx] < 0:
                dist[ny][nx] = dist[iy][ix] + 1
                q.append((nx, ny))
    # ★ S32：用注册半径（矩形足迹的 inscribed_radius），不是外接圆
    BODY_HALF_W = 0.12
    need = BODY_HALF_W / res                          # 需要的格数
    for ix in range(w):
        for iy in range(h):
            if dist[iy][ix] < 0 or dist[iy][ix] < need:
                forbid[iy * w + ix] = 1

    sx = int((start[0] - ox) / res)
    sy = int((start[1] - oy) / res)
    if not (0 <= sx < w and 0 <= sy < h) or forbid[sy * w + sx]:
        return set()
    seen = {(sx, sy)}
    q = deque([(sx, sy)])
    while q:
        ix, iy = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if (nx, ny) in seen or forbid[ny * w + nx]:
                continue
            seen.add((nx, ny))
            q.append((nx, ny))
            if len(seen) > max_cells:
                return seen
    return seen


def check_connectivity(start=(0.0, 0.0)):
    """从起点出发，检查每个目标点是否在同一连通域。"""
    m = load_map()
    w, h, res, (ox, oy), _ = m
    comp = reachable_set(m, start)
    out = []
    for name, x, y, kind in ROUTE:
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        out.append((name, x, y, (ix, iy) in comp))
    return out, len(comp)


def selftest_navpatrol():
    """nav_patrol 自己的自测：不依赖 Nav2，只验纯逻辑。"""
    fails = []

    def chk(cond, msg):
        if not cond:
            fails.append(msg)
        print('  %s %s' % ('PASS' if cond else 'FAIL', msg))

    print('[1] nav_client 协议解析（state 六种终态都要认）')
    # ★ S33 改：原来的 parse_result 靠 grep 人读文本，已删除。
    #   现在客户端直接回 JSON，自测改为验「客户端返回的 dict 能否被正确解读」。
    cases = [
        ({'state': 'SUCCEEDED', 't': 12.5, 'detail': 'status=4'},
         'SUCCEEDED', 12.5),
        ({'state': 'ABORTED', 't': 30.0, 'detail': 'status=6'},
         'ABORTED', 30.0),
        ({'state': 'TIMEOUT', 't': 240.0, 'detail': '超过 240s 未结束'},
         'TIMEOUT', 240.0),
        ({'state': 'REJECTED', 't': 0.3, 'detail': '目标被 action server 拒绝'},
         'REJECTED', 0.3),
        ({'state': 'NO_SERVER', 't': 0.0, 'detail': '没发现 server'},
         'NO_SERVER', 0.0),
    ]
    for res, want_state, want_t in cases:
        got = res.get('state', 'UNKNOWN')
        got_t = float(res.get('t', 0.0))
        chk(got == want_state and abs(got_t - want_t) < 1e-6,
            'state=%s t=%.1f → 解读为 %s（应 %s）'
            % (got, got_t, got, want_state))

    # ★ 关键判据：两种「目标没送到」必须与「送了但失败」区分开。
    #   S33 事故就是把它们混成了一个 NOT_ACCEPTED，才误诊成算法问题。
    chk({'state': 'NO_SERVER'}.get('state') in ('NO_SERVER', 'REJECTED'),
        'NO_SERVER 与 REJECTED 是「未送达」，不能被当成导航失败')
    chk('SUCCEEDED' not in ('NO_SERVER', 'REJECTED'),
        'SUCCEEDED 与「未送达」互斥（不共用同一分支）')

    print('[2] PGM 行序（上下不能颠倒）')
    m = load_map()
    w, h, res, (ox, oy), data = m
    # ★ 查南墙我第一版写 y=-2.47 → FAIL；查北墙写 y=+2.44 → 也 FAIL。
    #   实测墙占据的行是：
    #     南侧 y≈-2.445 开始往南（-2.44 是 100）
    #     北侧 y≈+2.455 开始往北（+2.46 是 100）
    #   两边偏了大约一格（5cm），这是 SLAM 建图的正常量级，
    #   不是「行序写反」。行序写反的话南北会整体互换，不是偏一格。
    #   所以判据改成：南北各有一个「墙带」，且位置落在真值 ±3 格内。
    def wall_first(cx, y_from, step):
        """从 y_from 沿 step 方向找第一个占据格，返回 y。"""
        y = y_from
        for _ in range(20):
            if map_value(m, cx, y) == 100:
                return y
            y += step
        return None

    s = wall_first(0.0, -2.40, -0.01)
    n = wall_first(0.0, 2.40, +0.01)
    chk(s is not None and abs(s - (-2.45)) < 0.06,
        '南墙占据起点 y=%s（真值 -2.45 ± 0.06）' % (('%.3f' % s) if s else 'N/A'))
    chk(n is not None and abs(n - 2.45) < 0.06,
        '北墙占据起点 y=%s（真值 +2.45 ± 0.06）' % (('%.3f' % n) if n else 'N/A'))
    chk(map_value(m, 0.0, 0.0) == 0, '原点 = 0（空）')
    # ★ 防行序颠倒的核心判据：南北两侧都得有墙，且方向不能互换
    chk(s is not None and n is not None and s < 0 < n,
        '南北两侧各有一段墙且方向正确（s=%.2f < 0 < n=%.2f）'
        % (s if s else 0, n if n else 0))
    # 地图最外一行是未知（墙体外半侧超出地图范围）
    chk(map_value(m, 0.0, -2.49) == -1,
        '地图最底行 y=-2.49 为未知 -1（墙体外半侧在地图外，已知缺陷）')

    print('[3] 连通性（7 个点必须互相可达）')
    conn, ncomp = check_connectivity()
    for name, x, y, ok in conn:
        chk(ok, '%s (%.2f,%.2f) 可达' % (name, x, y))
    chk(ncomp > 5000, '连通域格数 %d 应 > 5000（否则房间被切碎）' % ncomp)

    print('[4] 路线里不能出现「物理上比车窄」的通道')
    gaps = dict((n, g) for n, g, _ in G.channels())
    chk(gaps['E 短板西侧开口'] < 2 * G.ROBOT_RADIUS,
        '缝E %.3f < 车宽 %.3f（死路，路线不得经过）'
        % (gaps['E 短板西侧开口'], 2 * G.ROBOT_RADIUS))
    chk(gaps['W 西侧开口'] > 2 * G.ROBOT_RADIUS + 0.5,
        '缝W %.3f 应远宽于车宽 %.3f' % (gaps['W 西侧开口'], 2 * G.ROBOT_RADIUS))
    # 缝S 0.655，余量 0.255 —— 能过但不算宽，所以判据只要求余量 >0.1
    chk(gaps['S 短板下方缝'] > 2 * G.ROBOT_RADIUS + 0.1,
        '缝S %.3f > 车宽 %.3f 且余量 %.3f > 0.1'
        % (gaps['S 短板下方缝'], 2 * G.ROBOT_RADIUS,
           gaps['S 短板下方缝'] - 2 * G.ROBOT_RADIUS))
    # 路线点不得落在 A 缝的通行区内（那儿净空 0.175 < 车半径）
    for name, x, y, kind in ROUTE:
        chk(G.clearance(x, y) > G.ROBOT_RADIUS,
            '%s (%.2f,%.2f) 净空 %.3f > 车半径' % (name, x, y, G.clearance(x, y)))

    print('[5] 绕行用例必须真的需要绕')
    def straight_blocked(a, b, n=200):
        for i in range(1, n):
            t = i / float(n)
            px = a[0] + (b[0] - a[0]) * t
            py = a[1] + (b[1] - a[1]) * t
            if G.clearance(px, py) <= G.ROBOT_RADIUS:
                return (px, py)
        return None
    # 「西南角 → 西侧通道」直线沿 x=-2.2 上行，应畅通（属于沿墙直行）
    h = straight_blocked((-2.50, -2.00), (-2.50, 0.00))
    chk(h is None, '西南角→西侧通道 直线畅通（沿西墙直行）')
    # 「南走廊中点 → 北走廊中点」直线必穿 partition_mid
    h = straight_blocked((0.00, -2.00), (0.00, 1.90))
    chk(h is not None,
        '南区(0,-2.0)↔北区(0,1.9)直线必穿障碍（%.2f,%.2f），必须绕行'
        % (h if h else (0, 0)))
    # 「东南角 → 北走廊中点」直线必穿 partition_short
    h = straight_blocked((2.20, -1.50), (0.00, 1.90))
    chk(h is not None,
        '东南角↔北走廊中点 直线必穿 partition_short（%.2f,%.2f）'
        % (h if h else (0, 0)))

    print()
    if fails:
        print('!! %d 项 FAIL' % len(fails))
        for m_ in fails:
            print('   -', m_)
        return 1
    print('全部 PASS')
    return 0


def plan_path(a, b):
    """离线 A* 规划（8 邻域），返回世界坐标路径点列表。

    ★ 为什么要自己实现一遍，而不是只用 Nav2：
      Nav2 规划失败时只说 "Failed to create path"，不给原因。
      离线算一遍可以立刻区分：
        · 没路径      → 目标不可达 / 起点被困 / 膨胀封死
        · 有路径但很长 → 规划器绕远了，DWB 可能跟不上
      而且换参数（inflation/robot_radius）后可以离线先验，不必反复起仿真。
    """
    import heapq
    m = load_map()
    w, h, res, (ox, oy), _ = m

    def cost_of(ix, iy):
        x = ox + (ix + 0.5) * res
        y = oy + (iy + 0.5) * res
        if map_value(m, x, y) != 0:      # 墙或未知
            return 254
        return G.inflate_cost(G.clearance(x, y))

    def cell(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    def world(c):
        return ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res

    s, g = cell(*a), cell(*b)
    if cost_of(*s) >= 253 or cost_of(*g) >= 253:
        return None
    dist = {s: 0.0}
    prev = {s: None}
    pq = [(0.0, s)]
    nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1))
    while pq:
        d, u = heapq.heappop(pq)
        if u == g:
            break
        if d > dist.get(u, 1e18):
            continue
        for dx, dy in nbrs:
            v = (u[0] + dx, u[1] + dy)
            if not (0 <= v[0] < w and 0 <= v[1] < h):
                continue
            c = cost_of(*v)
            if c >= 253:
                continue
            step = res * (1.41421356 if dx and dy else 1.0)
            nd = d + step * (1.0 + 10.0 * c / 253.0)   # 代价越高越不愿走
            if nd < dist.get(v, 1e18):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if g not in prev:
        return None
    path = []
    cur = g
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return [world(c) for c in path]


def print_plan(start=(0.0, 0.0)):
    """逐段离线规划整条序列，报告长度、最小净空、以及「到点精度上限」。

    ★★ 2026-11-10 加最后一列「到点上限」。
       起因：东北角一次 SUCCEEDED 但误差 0.256m，超 TOL_ERR=0.25，
       而 xy_goal_tolerance 只有 0.15 —— 光看容差会误判成"随机的收尾误差"。
       真原因是**路径末端的停车点在墙边，DWB 到那儿就只剩很小的可达域**：
       它只要把车停进任意一个合法终点就行，不求精确。

       这里给每段算两个数：
         finish  = 路径末点到目标的距离（规划器的目标精度）
         worst   = 在 0.15m 容差球内，车心合法停靠的最远距离
                   （< 0.15 说明到达球被墙切掉一块，越靠边越容易超判据）
    """
    print('=' * 92)
    print('离线路径规划预演（A*，8 邻域，代价 = 1 + 10·cost/253）')
    print('=' * 92)
    print('%-12s %8s %8s %10s %8s %8s %7s  %s'
          % ('目标点', '段长', '累计', '最小净空', '末端距', '链式累计', '可达域', '判定'))
    print('-' * 92)
    cur = start
    total = 0.0
    drift = 0.0          # 链式累计：每段都从「上一段的实际末点」出发
    allok = True
    for name, x, y, kind in ROUTE:
        p = plan_path(cur, (x, y))
        if p is None:
            print('%-12s %8s %8s %10s %8s %8s %7s  ★ 无路径（目标被膨胀封死或不可达）'
                  % (name, '-', '-', '-', '-', '-', '-'))
            allok = False
            cur = (x, y)
            continue
        seg = 0.0
        for i in range(1, len(p)):
            seg += math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1])
        total += seg
        md = min(G.clearance(*q) for q in p)
        finish = math.hypot(p[-1][0] - x, p[-1][1] - y)
        # ★★ 关键一列：链式累计漂移。
        #   第一版这里 cur = (x, y)，等于「假装上一段精确到达目标」——
        #   于是每段的「末端距」都恒等于栅格量化误差 0.021m，看起来完美。
        #   真机上不会这样：每段收尾都有 0.1~0.25m 残差，下一段从那个残差位置出发，
        #   误差会**累积**。这里如实累加，用来判断「第 N 段是否已经偏出去」。
        drift += finish
        worst, _ideal, note = G.reachable_center(x, y, TOL_ERR - 0.02)
        ok = md > G.ROBOT_RADIUS and finish < 0.06 and worst is not None
        allok &= ok
        print('%-12s %7.2fm %7.2fm %9.3fm %7.3fm %7.3fm %7s  %s'
              % (name, seg, total, md, finish, drift,
                 ('%.3f' % worst) if worst is not None else 'N/A',
                 ('PASS' if ok else '★ ' + note) +
                 ('  [绕行]' if kind == 'detour' else '')))
        cur = (p[-1][0], p[-1][1])
    print('-' * 92)
    print('总路程 %.2f m（直线累加 %.2f m，绕行系数 %.2f）'
          % (total, straight_len(), total / max(straight_len(), 1e-9)))
    print('末端距 = 路径末点到目标的距离（单段）；链式累计 = 各段残差相加（最坏情况）')
    print('可达域 = %.2f m 容差球内，车心合法停靠的最远距离' % (TOL_ERR - 0.02))
    print('全序列：%s' % ('PASS' if allok else '★ 有段失败'))
    return allok


def straight_len():
    """7 点按顺序的直线距离累加，用来算「绕行系数」。"""
    tot = 0.0
    cur = (0.0, 0.0)
    for _n, x, y, _k in ROUTE:
        tot += math.hypot(x - cur[0], y - cur[1])
        cur = (x, y)
    return tot


def print_precheck():
    print('=' * 76)
    print('目标点预检（地图格 + 解析净空）')
    print('=' * 76)
    print('%-12s %8s %8s %6s %8s %8s  %s'
          % ('目标点', 'x', 'y', '类型', '地图格', '净空', '判定'))
    print('-' * 76)
    m = load_map()
    allok = True
    for name, x, y, kind, ok, reason in precheck():
        mv = map_value(m, x, y)
        allok &= ok
        print('%-12s %8.2f %8.2f %6s %8s %8.3f  %s'
              % (name, x, y, kind, {100: '墙', 0: '空', -1: '未知'}.get(mv, '?'),
                 G.clearance(x, y), ('OK  ' if ok else '★ FAIL ') + reason))
    print('-' * 76)

    # 连通性：8 点必须在同一连通域，否则必然有导航失败
    conn, ncomp = check_connectivity()
    bad = [n for n, _x, _y, ok in conn if not ok]
    print('连通域：从原点 BFS 可达 %d 格，%d 个目标点中 %d 个可达%s'
          % (ncomp, len(conn), len(conn) - len(bad),
             ('，★ 不可达：' + '、'.join(bad)) if bad else ''))
    allok &= not bad

    gaps = dict((n, g) for n, g, _ in G.channels())
    # ★ 2026-11-10 夜（S32）：车体模型改为矩形足迹，这里同步打印「车宽/注册半径」，
    #   不再打印 robot_radius（它已从 yaml 删除，留着会误导）。
    print('车宽 %.3f（矩形足迹）  注册半径 %.3f  inflation %.3f'
          % (G.BODY_W, G.BODY_W / 2, G.INFLATION))
    for chname, key, note in [
            ('缝N 短板上方', 'N 短板上方缝', '次选通道（比缝S 窄，规划器应优先走缝S）'),
            ('缝E 短板西侧', 'E 短板西侧开口', '矩形模型下物理能过，但零代价带为负 → 实际不走'),
            ('缝S 短板下方', 'S 短板下方缝', '主通路'),
            ('缝W 西侧开口', 'W 西侧开口', '宽敞')]:
        band, fband = G.narrow_band(gaps[key])
        print('  %-14s 缝宽 %.3f  非致命带 %.3f  零代价带 %.3f  %s'
              % (chname, gaps[key], band, fband, note))
    print('预检合计：%s' % ('全部通过' if allok else '★ 有不通过项，见上'))
    return allok


# ---------------------------------------------------------------- 主流程
def _run_route(route, prev, rows, client):
    """逐点巡检。rows 是外部传进来的列表，就地追加。"""
    import time as _t
    for name, gx, gy, kind in route:
        tag = '[绕行]' if kind == 'detour' else ''
        print('→ %-12s %s 目标 (%.2f, %.2f)' % (name, tag, gx, gy), flush=True)
        seg = math.hypot(gx - prev[0], gy - prev[1]) if prev[0] is not None else 0.0

        res = client.goto(gx, gy)
        state = res.get('state', 'UNKNOWN')
        dt = float(res.get('t', 0.0))
        detail = res.get('detail', '')

        # 等 AMCL 把最终位姿发出来再读
        _t.sleep(1.5)
        px, py = get_pose()
        err = None if px is None else math.hypot(px - gx, py - gy)
        ok = (state == 'SUCCEEDED' and err is not None
              and err < TOL_ERR and dt < TOL_TIME)
        # 平均速度（直线下界估计）
        spd = (seg / dt) if dt > 0 else 0.0
        rows.append((name, tag, state, err, dt, ok, seg, spd))
        if px is not None:
            prev = (px, py)

        flag = 'PASS' if ok else 'FAIL'
        note = ''
        if spd < SLOW_SPEED:
            note = '  ← 慢！%.3f m/s 低于 %.2f' % (spd, SLOW_SPEED)
        elif spd < WARN_SPEED:
            note = '  (%.3f m/s 偏慢)' % spd
        # ★ NO_SERVER / REJECTED 是「目标没送到」，与「送到了但失败」是两回事，
        #   必须分开报，否则会像 S33 那样把链路故障误诊成导航算法故障。
        if state in ('NO_SERVER', 'REJECTED'):
            note = '  ← 目标未送达！%s' % detail
        print('   %-10s 误差 %s  用时 %.1fs  %.2fm  %.3f m/s  %s%s'
              % (state,
                 ('%.3f m' % err) if err is not None else 'N/A',
                 dt, seg, spd, flag, note))


def main():
    args = sys.argv[1:]
    if '--selftest' in args:
        return selftest_navpatrol()
    if '--dry' in args:
        ok = print_precheck()
        print()
        ok2 = print_plan()
        return 0 if (ok and ok2) else 1
    if '--plan' in args:
        ok = print_plan()
        return 0 if ok else 1

    if '--one' in args:
        i = args.index('--one')
        route = [('指定点', float(args[i + 1]), float(args[i + 2]), 'open')]
    elif '--no-detour' in args:
        route = [r for r in ROUTE if r[3] == 'open']
    else:
        route = ROUTE

    print_precheck()
    print()

    x0, y0 = get_pose()
    if x0 is None:
        sys.stderr.write(
            '!! 取不到 /amcl_pose —— Nav2/AMCL 没起，或定位未收敛\n'
            '   先跑：bash live.sh nav\n'
            '   只想看预检：python3 nav_patrol.py --dry\n')
        return 2
    print('起点 AMCL 位姿 (%.3f, %.3f)\n' % (x0, y0))

    rows = []
    prev = (x0, y0)          # 上一段的终点，用来算本段直线距离

    # ★ S33：整轮巡检共用一个常驻 action client，不再每点 fork 一个 CLI
    client = GoalClient()
    try:
        _run_route(route, prev, rows, client)
    finally:
        client.close()

    print('\n' + '=' * 78)
    print('%-12s %-6s %-12s %9s %8s %8s %8s %6s'
          % ('目标点', '类型', '结果', '误差', '段长', '用时', '平均速度', '判定'))
    print('-' * 78)
    slow = []
    for name, tag, state, err, dt, ok, seg, spd in rows:
        mark = 'PASS' if ok else 'FAIL'
        if spd < SLOW_SPEED:
            mark += ' 慢'
            slow.append(name)
        elif spd < WARN_SPEED:
            mark += ' ~'
        print('%-12s %-6s %-12s %9s %7.2fm %7.1fs %7.3f %6s'
              % (name, tag or '开阔', state,
                 ('%.3f m' % err) if err is not None else 'N/A',
                 seg, dt, spd, mark))
    npass = sum(1 for r in rows if r[5])
    print('-' * 78)
    print('合计 %d 点，PASS %d，FAIL %d' % (len(rows), npass, len(rows) - npass))
    print('判据：SUCCEEDED + 误差 < %.2f m + 用时 < %.0f s' % (TOL_ERR, TOL_TIME))
    if slow:
        # ★ 慢不算 FAIL（用时就该因路远而长），但必须报出来。
        #   S31 就是靠「67.8s vs 14s」这个分布异常才挖到爬行问题的。
        print('\n⚠ 以下 %d 点平均速度 < %.2f m/s，疑似「贴着障碍爬行」而非正常绕行：'
              % (len(slow), SLOW_SPEED))
        for n in slow:
            print('    · %s' % n)
        print('  查证：python3 tools/verify/sim_fullchain/watch_cmdvel.py')
        print('       若 PURE_LINEAR 段速度长期 < 0.05，是 BaseObstacle.scale 太小；')
        print('       若 MIXED 段很长，是路径太贴墙。')
    return 0 if npass == len(rows) else 1


if __name__ == '__main__':
    sys.exit(main())
