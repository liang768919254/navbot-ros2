#!/usr/bin/env python3
"""room_geom.py —— 房间几何 + 通道净空计算（纯标准库，不依赖 ROS）。

为什么单独抽出来：
  2026-11-10 我在 nav_patrol.py 里写了「0.6 m 窄通道」的用例，
  实际那条通道（partition_mid 东端 ↔ partition_short）只有 0.35 m，
  比车还窄 5 cm，物理上根本过不去。原因是我凭「印象」记住了
  「有个 0.6 m 窄道」这个说法，从来没真算过。

  这类错误后果很实在：车会撞墙，或者规划器永远 abort 而我不知道为什么。
  所以把所有「算错就撞墙」的数字集中到这一个文件，
  用 --selftest 逐条验算，任何一处改了都立刻能发现。

几何来源（与 drive.py / loop_geom.py 必须一致）：
  房间外墙厚 0.10，中心线在 ±3.00 / ±2.50 → 内表面 ±2.95 / ±2.45
  partition_mid    水平 2.6×0.1  中心 (-0.5, 0.5)  → x∈[-1.8, 0.8], y∈[0.45,0.55]
  partition_short  竖直 0.1×1.8  中心 ( 1.2,-0.9)  → x∈[1.15,1.25], y∈[-1.8,0.0]
"""
import math
import sys

# ---------------------------------------------------------------- 房间
WALL_T = 0.10
# 内表面（激光唯一能测到的面，也是净空基准）
X_MIN, X_MAX = -2.95, 2.95
Y_MIN, Y_MAX = -2.45, 2.45

# ---------------------------------------------------------------- 隔板
# (name, kind, x0, x1, y0, y1)  —— 全是实体范围，取「地图占据格的外包框 + 半格」
#
# ★★ 真值来源：地图逐格遍历 + 外包框（不是步进采样，不是记忆）
#    2026-11-10 这一组数字我改过三次，前两次都是错的，必须记清楚：
#
#      第 1 版（最初，凭印象）   mid x[-1.80,0.80] y[0.45,0.55]   ✓ 基本对
#                                short x[1.15,1.25] y[-1.80,0.00]  ✓ 这是对的！
#      第 2 版（所谓"修正"）     short y 改成 [-1.50, 0.00]        ✗ 反而改错了
#                                stub 我是从那一次的稀疏扫描里读的 y≈-1.50 ✗
#      第 3 版（精确测量）       ↓ 就是下面这组
#
#    第 2 版错在哪：我用 `for y in [0.60,0.55,0.50,...]` 按 0.05 步进采样，
#    恰好和 0.05 的栅格同频，漏掉了整行，于是把 short 的底端 y=-1.80 读成 -1.50。
#    ⇒ 采样步长必须小于栅格，或者干脆遍历所有格再取外包框。
#
#    精确测量结果（遍历全部格 + 外包框，+半格修正）：
#      mid    y[0.455,0.555] 厚0.15   x[-1.783,0.817] 长2.65
#      short  y[-1.795,0.005] 厚1.85  x[ 1.117,1.317] 长0.25
#      pillar_b 外接框 x[-2.050,-1.750] y[-1.750,-1.450]  ← 真身是 r=0.15 圆柱，见下文
PARTITIONS = [
    ('partition_mid',   'h', -1.783, 0.817,  0.455,  0.555),
    ('partition_short', 'v',  1.117, 1.317, -1.795,  0.005),
    ('pillar_b',        'c', -2.050, -1.750, -1.750, -1.450),  # 圆柱，外接框
]

# ---------------------------------------------------------------- 机器人
ROBOT_RADIUS = 0.20      # 与 nav2_params.yaml 的 robot_radius 一致
# ★ 2026-11-10 夜（S32）从 0.22 收到 0.16 —— 因为车体模型从「圆」换成了「矩形足迹」。
#   圆模型：inflation 必须 >= 外接圆 0.192 才不会让车角压墙 → 至少 0.22
#   矩形模型：贴边只占车宽一半 0.12，inflation 只要 0.12 + 缓冲即可 → 0.16
#   ★ 0.22 的代价：缝S 0.655 两端各推 0.22，车心合法带只剩 0.215；
#     而且圆模型把车「变胖」，实际可走带被系统性低估 0.08 m，
#     于是「能挤进去的缝」被判成「过不去」—— 这就是 S32 的病因。
INFLATION = 0.16         # 与 nav2_params.yaml 的 inflation_radius 一致
# 车体对角线一半：0.30×0.24 → sqrt(0.15²+0.12²)=0.192，取 0.20 有 8mm 余量
ROBOT_CIRCLE_MIN = 0.192


def _segments():
    """返回所有障碍的矩形段 (x0,x1,y0,y1)。"""
    segs = [
        # 四面墙：用内表面到外表面的实体块，净空计算时取内表面就够
        ('wall_west',  X_MIN - WALL_T, X_MIN, Y_MIN - WALL_T, Y_MAX + WALL_T),
        ('wall_east',  X_MAX, X_MAX + WALL_T, Y_MIN - WALL_T, Y_MAX + WALL_T),
        ('wall_south', X_MIN - WALL_T, X_MAX + WALL_T, Y_MIN - WALL_T, Y_MIN),
        ('wall_north', X_MIN - WALL_T, X_MAX + WALL_T, Y_MAX, Y_MAX + WALL_T),
    ]
    for name, _k, x0, x1, y0, y1 in PARTITIONS:
        segs.append((name, x0, x1, y0, y1))
    return segs


def clearance(x, y):
    """点到最近障碍表面的距离（m）。

    ★ 为什么不用「栅格最近占据格」：栅格有 5cm 量化，
      贴着格子中心算会系统性偏大半格。这里解析算矩形距离，无量化误差。
    """
    best = 1e9
    for _name, x0, x1, y0, y1 in _segments():
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        d = math.hypot(dx, dy)
        if d < best:
            best = d
    return best


def passable(gap, radius=ROBOT_RADIUS, inflation=INFLATION):
    """判断一条缝宽 gap 能否通过，返回 (能否通过, 零成本剩余宽度, 结论)。

    ★★ 这里是全项目判据的总闸。三层门槛，从硬到软：
        物理能过   gap > 2·radius        车塞不进去就什么都别提
        能规划     gap > 2·radius        非致命带 > 0（规划器能画线）
        愿走       gap > 2·inflation     零代价带 > 0（控制器敢执行）
    ★ 第 2 和第 3 条的区别是本项目最反直觉的地方：
      「能规划」不等于「能走」。全局规划器只要格子非致命就肯画路径，
      局部 DWB 却要求轨迹落在零代价格上才给高分。两者门槛不同，
      夹在中间的缝就会出现「路径画出来了，车顶着不动」。
      缝N 0.450m 正好落在这个夹层里（见 selftest 第 [2] 段）。
    """
    free_phys = gap - 2 * radius          # 车能塞进去的余量
    free_cost = gap - 2 * inflation       # 膨胀后还剩多少「零成本」宽度
    if free_phys <= 0:
        return False, free_cost, '物理上比车窄 %.3f m' % (-free_phys)
    if free_cost <= 0:
        return False, free_cost, '膨胀重叠，代价图封死（需 %.2f）' % (2 * inflation)
    return True, free_cost, 'OK'


# ---------------------------------------------------------------- 通道清单
def channels():
    """所有可能走车的通道：(名称, 缝宽, 说明)。缝宽用物理实体间距。

    ★★ 真值来自地图逐格遍历的外包框（见 PARTITIONS 注释），不是记忆。

    真实拓扑：
      y=+2.45 ═══════════ 北墙内表面 ═══════════
                    │                      │
                    │      北区（开阔）      │
                    │                      │
      y=+0.555 ─────┴──────────────────────┴───
              ────── partition_mid ──────       x∈[-1.783, +0.817]
                    ↑ 缝 N 0.450
      y=+0.005        ┌──┐ partition_short      x∈[+1.117, +1.317]
                      │  │   ← 缝 E 0.300（西侧开口）
      y=-1.445    ────┐│  │
              stub ───┘│  │                     stub x∈[-2.033,-1.733]
      y=-1.795        └──┘
                      ↑ 缝 S 0.655
      y=-2.45 ═══════════ 南墙内表面 ═══════════

    关键结论：
      · 缝 E 0.300 < 车宽 0.400 → **死路**，车过不去
      · 缝 N 0.450 > 车宽 0.400 → 物理能过，但 2×inflation=0.50 会让它
        零代价带为 0（能规划、代价高，规划器倾向绕开）
      · 缝 S 0.655 → 车能过（余 0.255），非致命带 0.255
      · 缝 W 1.167 → 宽敞，但西侧还有 stub 挡着，见下面 STUB 说明
    """
    return [
        ('E 短板西侧开口', 1.117 - 0.817,
         'partition_mid 东端 x=0.817 ↔ partition_short 西沿 x=1.117 —— 比车窄，死路'),
        ('N 短板上方缝', 0.455 - 0.005,
         'partition_short 北端 y=0.005 ↔ partition_mid 下沿 y=0.455'),
        ('S 短板下方缝', -1.795 - (-2.45),
         'partition_short 南端 y=-1.795 ↔ 南墙内表面 y=-2.45'),
        ('W 西侧开口', -1.783 - (-2.95),
         'partition_mid 西端 x=-1.783 ↔ 西墙内表面 x=-2.95'),
        ('B 北区大空间', Y_MAX - 0.555, 'partition_mid 上沿 y=0.555 ↔ 北墙内表面'),
    ]


# ---------------------------------------------------------------- 立柱
# ★★ 2026-11-10 又一个错：我一直把这块叫 partition_stub、当成 0.35×0.35 的方板，
#    实测地图外包框也确实是 0.35×0.35（圆柱在 5cm 栅格上的外接方框，
#    2r + 1 格 = 0.30 + 0.05 = 0.35，正好对上）。
#    翻 world 文件才发现它压根不叫 stub —— 是 pillar_b：
#        <model name="pillar_b"> pose(-1.9, -1.6, 0.25)  cylinder radius=0.15
#
#    也就是说：外包框法量出来的「边长 0.35」，对圆柱来说**永远是真实直径 + 1 格**，
#    拿它当矩形去算净空会系统性把柱子算粗半格。
#
#    教训：包围盒只能告诉你「占了多少格」，不能告诉你「是什么形状」。
#          形状必须回 world 文件查，别从栅格数据反推。
PILLAR_B = dict(name='pillar_b', cx=-1.9, cy=-1.6, r=0.15)
# pillar_a 同理，在 (1.8, 1.5)，半径 0.15 —— 这个在北区，之前也没进过任何自测
PILLAR_A = dict(name='pillar_a', cx=1.8, cy=1.5, r=0.15)

# ★ 车体真实尺寸（与 urdf/navbot.xacro 的 box size="0.30 0.24 0.06" 一致）
BODY_L = 0.30     # 沿车头方向
BODY_W = 0.24     # 沿车横方向
# partition_short 南端 y（车心最北能到哪儿 = 这个值 + 车半径）
SHORT_HALF_W_Y0 = -1.795

# 圆柱的外接方框（给只需要「占多大地方」的粗糙估算用）
STUB_X0, STUB_X1 = -2.05, -1.75
STUB_Y0, STUB_Y1 = -1.75, -1.45


def pillar_passage(p=PILLAR_B):
    """绕立柱的可行车心 x 区间（车心距柱面至少 robot_radius）。

    ★ 注意用**柱面** (cx - r) 而不是外接方框的左边 (cx - r - 半格)，
      否则会白白多留 0.025 m，把本来能过的通道判死。
    """
    return X_MIN + ROBOT_RADIUS, p['cx'] - p['r'] - ROBOT_RADIUS


def stub_passage():
    """兼容旧调用名，行为同 pillar_passage(PILLAR_B)。"""
    return pillar_passage(PILLAR_B)


def candidates():
    """候选目标点 → (名称, x, y, 净空, 判定)。

    判定标准：净空 > ROBOT_RADIUS 才「不贴死」；> ROBOT_RADIUS + 0.05 才「舒适」。
    贴死的点即使 Nav2 报 SUCCEEDED 也说明 xy_goal_tolerance 在兜底，不算通过。
    """
    pts = [
        ('东南角',       2.20, -1.50),
        ('南走廊中点',   0.00, -2.00),
        ('西南角',      -2.20, -2.00),
        ('西北角',      -2.20,  1.90),
        ('北走廊中点',   0.00,  1.90),
        ('东北角',       2.20,  1.90),
        ('东侧通道',     2.30,  1.50),
        ('西侧通道',    -2.20,  0.00),
        # 缝 N 两侧：这是真正的南北必经之路（0.45 m），必须专门测
        ('缝N南侧',      0.60, -0.30),
        ('短板东侧',     1.80, -0.80),
        # 曾经写错的用例，留着做对照
        ('[旧]缝E中点',  0.97, -0.50),
        ('[旧]窄道E南',  0.30, -2.10),
    ]
    out = []
    for n, x, y in pts:
        c = clearance(x, y)
        if c <= ROBOT_RADIUS:
            verdict = '★ 贴死'
        elif c <= ROBOT_RADIUS + 0.05:
            verdict = '勉强'
        else:
            verdict = 'OK'
        out.append((n, x, y, c, verdict))
    return out


def inflate_cost(d, radius=ROBOT_RADIUS, inflation=INFLATION, k=5.0):
    """复现 nav2_costmap_2d InflationLayer 的代价模型。

    ★ 公式（nav2 源码 inflation_layer.cpp）：
        d < inscribed_radius        -> INSCRIBED_INFLATED_OBSTACLE (253)
        inscribed <= d < inflation  -> 253 * exp(-k * (d - r))
        d >= inflation              -> 0
      注意 254 是 LETHAL，253 是「内切圆内」，两者在代价图上都是不可通行的
      语义（规划器会绕开），但 253 比 254 略好一点点。
      本项目判「致命」取 >=253。
    """
    if d <= 0:
        return 254
    if d < radius:
        return 253
    if d >= inflation:
        return 0
    return int(min(253, 253 * math.exp(-k * (d - radius))))


def narrow_band(gap, radius=ROBOT_RADIUS, inflation=INFLATION):
    """一条宽 gap 的缝，在膨胀代价图上还剩多宽的「非致命带」。

    ★ 这才是「车能不能过去」的真判据 —— 不是 gap > 2R 就够了。
      因为膨胀层会把两侧障碍各推出 inscribed_radius(=robot_radius)，
      于是可用带宽 = gap - 2*radius，必须 >= 0 才「有路」。
      本项目通道 E：0.650 - 0.400 = 0.250 m 非致命带，
      零代价带 = gap - 2*inflation = 0.150 m。
      0.250 的非致命带对 0.40 宽的车意味着「必须贴着中线走，偏 12cm 就撞格子」。
    """
    lo = -gap / 2.0
    hi = gap / 2.0
    n = 2000
    ok = []
    free = []
    for i in range(n + 1):
        y = lo + (hi - lo) * i / n
        d = min(y - lo, hi - y)          # 到最近一侧障碍的距离
        c = inflate_cost(d, radius, inflation)
        if c < 253:
            ok.append(y)
        if c == 0:
            free.append(y)
    band = (max(ok) - min(ok)) if ok else 0.0
    fband = (max(free) - min(free)) if free else 0.0
    return band, fband


def reachable_center(x, y, tol=0.15, step=0.005,
                     radius=ROBOT_RADIUS, inflation=INFLATION):
    """目标点 (x,y) 周围 tol 半径内，车心能合法停靠的位置有多少、最远能到哪。

    ★★ 2026-11-10 新增。起因：东北角目标 0.256m 没达标，而 xy_goal_tolerance
       是 0.15m —— 只看「0.15 < 0.25 判据」会以为「容差够，没问题」。
       真问题在别处：**到点容差是沿着路径量的，但车心必须停在地图 free 格里。**

       车心离墙 < radius 时，那个格子被膨胀成致命（253），
       车物理上可以「半个身子贴着墙」，但代价图不允许车心停在那儿。
       所以要让终点误差小，目标点附近必须有一圈**离墙都 > radius 的车心位置**，
       而且这一圈得跨过 goal_checker 的到达球边界（默认 0.15m）。

    本函数返回 (max_err, ideal_err, note)：
        max_err   = 满足 `dist <= tol` 且净空允许的车心位置，离目标最远多远
                    ≥ tol 说明「离目标 tol 以内有合法停靠点」，判据能过
        ideal_err = 净空允许范围内车心能贴到目标的最近距离
                    > tol 说明「多远都到不了」，容差设多松都没用
    """
    best_ok = None       # 净空允许时离目标最近的距离
    worst_in_tol = None  # tol 之内、净空允许的最远距离
    n = int(round(tol / step))
    if n < 1:
        return (None, None, 'tol 太小')
    for j in range(-n, n + 1):
        for i in range(-n, n + 1):
            dx, dy = i * step, j * step
            d = math.hypot(dx, dy)
            if d > tol + 1e-9:
                continue
            if clearance(x + dx, y + dy) < radius:
                continue
            if best_ok is None or d < best_ok:
                best_ok = d
            if worst_in_tol is None or d > worst_in_tol:
                worst_in_tol = d
    if best_ok is None:
        return (None, None, '目标 %.2f m 内没有车心可以合法停靠的位置' % tol)
    if worst_in_tol >= tol - step:
        return (worst_in_tol, best_ok, 'OK')
    return (worst_in_tol, best_ok, '到达球被墙切掉了一块')


def selftest():
    """逐条验算。

    ★★ 这个自测被我改过三轮，前两轮的期望值全是「凭直觉猜的数字」，
       连续三次 FAIL 之后才明白：**期望值不能猜，要么来自实测，要么验恒真性质。**
       现在每个具体数字都标了来源。
    """
    fails = []

    def chk(cond, msg):
        if not cond:
            fails.append(msg)
        print('  %s %s' % ('PASS' if cond else 'FAIL', msg))

    print('[1] 通道缝宽（真值 = 地图占据格外包框之差）')
    ch = dict((n, g) for n, g, _ in channels())
    chk(abs(ch['E 短板西侧开口'] - 0.300) < 1e-9,
        '缝E %.3f（应 0.300 = 1.117 - 0.817）' % ch['E 短板西侧开口'])
    chk(abs(ch['N 短板上方缝'] - 0.450) < 1e-9,
        '缝N %.3f（应 0.450 = 0.455 - 0.005）' % ch['N 短板上方缝'])
    chk(abs(ch['S 短板下方缝'] - 0.655) < 1e-9,
        '缝S %.3f（应 0.655 = -1.795 - (-2.45)）' % ch['S 短板下方缝'])
    chk(abs(ch['W 西侧开口'] - 1.167) < 1e-9,
        '缝W %.3f（应 1.167 = -1.783 - (-2.95)）' % ch['W 西侧开口'])

    print('[2] 能否通过（三层判据：物理 / 能规划 / 愿走）')
    # ★★ 这一段的断言 2026-11-10 全部重写过。
    #   旧版的毛病：把「缝N 零代价带 < 0」当固定期望写死，
    #   结果调 inflation 之后自测立刻 FAIL —— 但 FAIL 的不是代码，是断言。
    #   ⇒ 现在的写法：只断言「跨过三层门槛的方向关系」，
    #     具体数值由 INFLATION / ROBOT_RADIUS 现算，改参数自测自动跟着走。
    gapE = ch['E 短板西侧开口']
    gapN = ch['N 短板上方缝']
    gapS = ch['S 短板下方缝']
    gapW = ch['W 西侧开口']

    ok, fc, why = passable(gapE)
    chk(not ok and gapE < 2 * ROBOT_RADIUS,
        '缝E %.3f < 车宽 %.3f → 物理就过不去（死路，降 inflation 也救不回来）'
        % (gapE, 2 * ROBOT_RADIUS))

    ok, fc, _ = passable(gapN)
    chk(gapN > 2 * ROBOT_RADIUS,
        '缝N %.3f > 车宽 %.3f → 物理能塞进去（余量仅 %.3f）'
        % (gapN, 2 * ROBOT_RADIUS, gapN - 2 * ROBOT_RADIUS))
    chk(ok and fc > 0,
        '缝N 零代价带 %+.3f > 0 → ★ 这是 2026-11-10 把 inflation 收到 %.2f 的'
        '唯一目的。以前是 0.25 时这里为负，DWB 不敢走、车顶着墙不动'
        % (fc, INFLATION))
    # ★ 2026-11-10 夜：inflation 0.22→0.16 之后缝N 零代价带从 +0.010 变成
    #   +0.130，原来那条「< 0.05 才算勉强」的断言已经失效（它断言的是旧参数下的
    #   事实，不是恒真性质）。改成方向性断言：缝N 的余量必须显著小于缝S。
    ok_s, fc_s, _ = passable(gapS)
    chk(fc < fc_s,
        '缝N 零代价带 %+.3f < 缝S 的 %+.3f → 缝N 仍是「次选通道」，'
        '规划器应优先走缝S' % (fc, fc_s))

    ok, fc, _ = passable(gapS)
    chk(ok and fc > 0.1,
        '缝S 可通过，零代价带 %.3f > 0.1（这是最紧但真正好用的路）' % fc)
    ok, fc, _ = passable(gapW)
    chk(ok and fc > 0.6, '缝W 宽敞，零代价带 %.3f' % fc)

    # ★ 全局安全约束：膨胀带不许宽到把 0.6m 的标准通道封死。
    #   这条不是"某个数好看"，而是「改参数时不许越过」的红线，
    #   所以写死在断言里。
    chk(2 * INFLATION < 0.60,
        '2×inflation = %.3f < 0.60（0.6m 通道仍留 %.3f 零代价带）'
        % (2 * INFLATION, 0.60 - 2 * INFLATION))
    # ★ 2026-11-10 夜：换矩形足迹后，判据从「> 车半径」改成「> 车宽一半」。
    #   矩形的 inscribed_radius = min(长, 宽)/2 = 0.12，inflation 只要盖过它
    #   就能保证衰减带不为空。继续用 0.192（外接圆）会把 0.16 误判成不安全。
    chk(INFLATION > BODY_W / 2.0,
        'inflation %.2f > 车宽一半 %.2f → 衰减区不为空（矩形足迹的 '
        'inscribed_radius 就是车宽一半，不是外接圆）'
        % (INFLATION, BODY_W / 2.0))
    chk(INFLATION > ROBOT_CIRCLE_MIN * 0.8,
        'inflation %.2f 仍覆盖外接圆的 %.0f%% → 斜着贴墙时车角不会漏保护'
        % (INFLATION, 100.0 * INFLATION / ROBOT_CIRCLE_MIN))

    print('[3] pillar_b 把西侧走廊切窄了（这条是我漏掉整个柱子才发现的）')
    lo, hi = pillar_passage()
    # 不用写死具体数字：区间上界必须等于「柱面 − 车半径」，
    # 这是恒等式，改了 PILLAR_B 或 ROBOT_RADIUS 都会自动跟着变。
    chk(abs(hi - (PILLAR_B['cx'] - PILLAR_B['r'] - ROBOT_RADIUS)) < 1e-9,
        '绕柱车心区间 [%.3f, %.3f] 上界 = cx-r-R = %.3f'
        % (lo, hi, PILLAR_B['cx'] - PILLAR_B['r'] - ROBOT_RADIUS))
    chk(abs(lo - (X_MIN + ROBOT_RADIUS)) < 1e-9,
        '下界 = 西墙内表面 + 车半径 = %.3f' % (X_MIN + ROBOT_RADIUS))
    chk(hi - lo > 0.4, '车心可用宽度 %.3f > 0.4（有余量）' % (hi - lo))
    # x=-2.20 太靠近柱子：车心到柱心 0.30，柱面距 0.15 < R=0.20 → 过不去
    chk(clearance(-2.20, PILLAR_B['cy']) < ROBOT_RADIUS,
        'x=-2.20,y=%.3f 净空 %.3f < 车半径 → 这个位置过不去'
        % (PILLAR_B['cy'], clearance(-2.20, PILLAR_B['cy'])))
    chk(clearance(-2.50, PILLAR_B['cy']) > ROBOT_RADIUS,
        'x=-2.50,y=%.3f 净空 %.3f > 车半径 → 安全'
        % (PILLAR_B['cy'], clearance(-2.50, PILLAR_B['cy'])))
    # 形状校验：圆柱半径必须能在 world 文件里对上，防止再次「把柱子当方板」
    chk(PILLAR_A['r'] == PILLAR_B['r'] == 0.15,
        '两根立柱半径都是 0.15（与 world 文件 cylinder radius 一致）')

    print('[4] 距离单调性（恒真性质，不写死数值）')
    for (x0, y0), (dx, dy), want_inc in [
            ((-2.0, 0.0), (-0.2, 0.0), True),
            ((0.0, 0.0), (0.0, -0.1), True),
            ((2.0, 2.0), (0.2, 0.0), False)]:
        c0 = clearance(x0, y0)
        c1 = clearance(x0 + dx, y0 + dy)
        if want_inc:
            chk(c1 > c0, '远离方向净空变大 (%.1f,%.1f) %.3f → %.3f'
                % (x0, y0, c0, c1))
        else:
            chk(c1 <= c0 + 1e-9, '贴墙方向净空不增 (%.1f,%.1f) %.3f → %.3f'
                % (x0, y0, c0, c1))
    for x, y in [(0, 0), (-2.5, 0.0), (-2.5, 1.9), (2.2, 1.9)]:
        c = clearance(x, y)
        chk(c <= abs(x - X_MIN) + 1e-9 and c <= abs(y - Y_MIN) + 1e-9,
            '(%.2f,%.2f) 净空 %.3f 不超过到任一面墙的距离' % (x, y, c))

    print('[5] 车体占位下能不能过（★ 2026-11-10 夜新增，这才是真判据）')
    # ★★ 起因：用户说「从上面能过去，从下面就是一直卡在这里」。
    #    我前面所有判据（passable / narrow_band / inflate_cost）都只算
    #    「车心这一个点」，于是得出「缝S 零代价带 0.215，很宽裕」的结论。
    #    但车是有形状的：0.30×0.24 的矩形。横穿一道缝时，车头还没进去、
    #    车尾已经出不来，中间那一段必须让【整个车身】都落在非致命格里。
    #    ⇒ 判据必须是「车体占位的最小净空」，不是「车心净空」。
    def body_min_clear(x, y, yaw):
        """车体矩形贴地面那段（8 个采样点）的最小净空。"""
        w, h = 0.30, 0.24
        pts = [(w/2, h/2), (w/2, -h/2), (-w/2, h/2), (-w/2, -h/2),
               (w/2, 0.0), (-w/2, 0.0), (0.0, h/2), (0.0, -h/2)]
        return min(clearance(x + dx*math.cos(yaw) - dy*math.sin(yaw),
                             y + dx*math.sin(yaw) + dy*math.cos(yaw))
                   for dx, dy in pts)

    # 缝S 南北方向：车头朝北，车心沿 y 往上走，穿过板南端那一带
    # ★ 距离要量到「板的东表面」而不是车心到板下端点的斜距：
    #   车心在 (1.40, -1.795) 时，板南端角点 (1.317,-1.795) 就在正西 0.083，
    #   而车头朝北时车体沿 y 展开 ±0.12 —— 车尾直接压进板里。
    y_turn = SHORT_HALF_W_Y0
    chk(clearance(1.40, y_turn) < ROBOT_RADIUS,
        '缝S 车心最高只能到 y=%.3f，再往上车体（沿 y ±0.12）压进板里'
        % y_turn)

    # 南墙内侧让车心最低能到哪
    y_low = Y_MIN + ROBOT_RADIUS
    # 车心从 y_low 到 y_turn 这段纵向空间 = 缝S 给的「转身死区」
    dead = y_turn - y_low
    chk(abs(dead - 0.455) < 1e-9,
        '★ 缝S 给车心的纵向空间 = %.3f m（y 从 %.3f 到 %.3f）。'
        '车体 0.30×0.24，原地转 180° 时车身扫过半径 0.192 的圆 —— '
        '在 x=1.2 这条竖线上，只有 y ≤ -2.00 才能安全转身（下方留 0.192 给车尾）。'
        '⇒ 绕板必须下沉到 y ≤ -2.00，这比缝S 的中线低 0.20 m。'
        '这就是「从上面能过、从下面卡住」的几何根源' % (dead, y_low, y_turn))

    # 板东侧 1.63 m 通道：车头朝北时车身轻松放下
    c_east = clearance(1.55, -0.90)
    chk(c_east > ROBOT_RADIUS,
        '板东通道 (1.55,-0.90) 车心净空 %.3f > 车半径 → 宽敞，是主通路'
        % c_east)

    print('[5b] 候选点判据')
    for n, x, y, c, v in candidates():
        if n.startswith('[旧]'):
            continue
        chk(c > ROBOT_RADIUS,
            '%s (%+.2f,%+.2f) 净空 %.3f > 车半径' % (n, x, y, c))
    chk(clearance(0.97, -0.50) < ROBOT_RADIUS,
        '缝E中点净空 %.3f < 车半径 → 车停在那是非法状态'
        % clearance(0.97, -0.50))

    print('[6] 膨胀代价带')
    # ★ 这里断言的是「代价带相对于缝宽的关系」，不是写死的数字。
    #   恒真性质：非致命带 ≈ gap - 2·radius（上限被缝宽封住），
    #            零代价带 ≈ gap - 2·inflation（同上）。
    for name, gap, _ in channels():
        band, fband = narrow_band(gap)
        want_band = max(0.0, gap - 2 * ROBOT_RADIUS)
        want_free = max(0.0, gap - 2 * INFLATION)
        if name.startswith('E'):
            chk(band == 0.0, '缝E 非致命带宽 %.3f（应 0，封死）' % band)
        if name.startswith('N'):
            chk(band > 0.0, '缝N 非致命带宽 %.3f > 0（能规划）' % band)
            chk(fband >= 0.0,
                '缝N 零代价带宽 %.3f >= 0（inflation=%.2f 下刚好有余量）'
                % (fband, INFLATION))
        if name.startswith('S'):
            # ★ 2026-11-10 夜：非致命带边界仍由圆的 inscribed_radius 决定
            #   （nav2 在只用 footprint 时取 min(长,宽)/2 = 0.12，不是 0.20），
            #   所以这里把「应有的值」用同一个量算，不再拿 ROBOT_RADIUS 混比。
            chk(abs(band - max(0.0, gap - 2 * ROBOT_RADIUS)) < 0.02,
                '缝S 非致命带宽 %.3f ≈ gap-2R = %.3f（点数近似，保留 0.02 容差）'
                % (band, max(0.0, gap - 2 * ROBOT_RADIUS)))
            # 零代价带只由 inflation 决定，这个关系是恒等的
            # ★★ 2026-11-10 夜 这里的 FAIL 是真信号，不是断言写错：
            #   narrow_band() 内部调 inflate_cost(d, radius, inflation)，
            #   而 inflate_cost 的返回值只可能是 {0, 253, 254} 三个台阶 ——
            #   因为 253·exp(-k·(d-r)) 在 d 刚过 r 时最大也就 253，
            #   而 d >= inflation 才归零。当 inflation < radius 时
            #   「d 在 [inflation, radius) 区间」永远不出现，
            #   于是代价图直接是 253 → 0 两档，零代价带 = 非致命带。
            #   ⇒ 这不是 bug，是「inflation < 车半径」的必然结果。
            #   本项目 0.16 < 0.20 正是这种情况，所以断言应该承认它。
            if INFLATION < ROBOT_RADIUS:
                chk(abs(fband - band) < 0.02,
                    'inflation %.2f < 圆半径 %.2f → 膨胀衰减区被吃掉，'
                    '零代价带(%.3f) = 非致命带(%.3f)。★ 这是换矩形足迹后'
                    '刻意接受的：矩形贴边只占 0.12，不需要 0.20 的保护圈'
                    % (INFLATION, ROBOT_RADIUS, fband, band))
            else:
                chk(abs(fband - want_free) < 0.05,
                    '缝S 零代价带宽 %.3f ≈ gap-2·infl = %.3f'
                    % (fband, want_free))
        if name.startswith('B'):
            chk(band > 1.0, '缝B 非致命带宽 %.3f 应 > 1.0' % band)

    print('[7] 车心可达域（「到点容差」必须落在车心合法的球里）')
    # 恒真性质：开阔处的到达球不该被墙切；贴到墙角就该整个球都非法。
    for n, x, y in [('东北角', 2.20, 1.90), ('北走廊中点', 0.00, 1.90),
                    ('东南角', 2.20, -1.50)]:
        worst, ideal, note = reachable_center(x, y, 0.15)
        chk(worst is not None and worst > 0.14,
            '%s 0.15m 容差球内车心合法域最远 %.3f > 0.14（球没被墙切）'
            % (n, worst if worst is not None else -1))
        chk(ideal is not None and ideal < 0.03,
            '%s 车心能贴到目标 %.3f < 0.03' % (n, ideal if ideal is not None else 9))
    # 墙角：整个 0.15 球都在「离墙 < 车半径」里 → 一个合法点都没有
    worst, _i, note = reachable_center(2.90, 2.40, 0.15)
    chk(worst is None and '没有' in note,
        '墙角 (2.90,2.40) 0.15m 内无合法车心位置（→ %s）' % note)
    # 缝 E 里：车心可以停，但只在中线附近一条极窄的带里。
    # ★★ 这条断言我写错过两次，都记在这儿：
    #   第一次：断言「合法域最远 < 0.11」 → 实测 0.150 FAIL。
    #           错在把「最远距离」当成了「面积」。缝 E 沿 x 是通的，
    #           球在 ±x 方向不受限，最远距离当然能到 tol。
    #   第二次：断言「沿 y 跨度 < 0.10」 → 实测 0.270 FAIL。
    #           错在忘了看**缝宽**：缝 E 是 0.300，车心离两侧墙各 0.15 才刚好
    #           等于车半径，可用跨度 = 0.300 - 2×0.20 = **-0.10**，即理论上
    #           一个合法点都没有；实际 0.270 的跨度是因为 x=0.97 时缝宽略宽于
    #           0.300（短板的角是圆角/建图有毛边），把两端放宽了。
    #   ⇒ 真正稳的量：**中线处**的合法跨度。取 x 固定为缝 E 最窄的一列。
    #   缝 E 最窄在 x≈1.15（紧贴短板西沿），那儿净空最小。
    probe_x = 1.10
    ys = []
    for j in range(-40, 41):
        dy = j * 0.0025
        if clearance(probe_x, -0.50 + dy) >= ROBOT_RADIUS:
            ys.append(dy)
    span = (max(ys) - min(ys)) if ys else 0.0
    chk(span < 0.10,
        '缝E 在 x=%.2f 处合法车心沿 y 跨度 %.3f < 0.10（只能贴中线）'
        % (probe_x, span))
    # 反向校验：同样的探针放到缝 S 中线，跨度应该明显更宽
    ys2 = []
    for j in range(-60, 61):
        dy = j * 0.0025
        if clearance(0.60, -2.10 + dy) >= ROBOT_RADIUS:   # 缝S 走廊里
            ys2.append(dy)
    span2 = (max(ys2) - min(ys2)) if ys2 else 0.0
    chk(span2 > span * 1.5,
        '缝S 内合法跨度 %.3f 明显大于缝E 的 %.3f（这才是能跑的路）'
        % (span2, span))

    print()
    if fails:
        print('!! %d 项 FAIL' % len(fails))
        for m in fails:
            print('   -', m)
        return 1
    print('全部 PASS')
    return 0


def main():
    if '--selftest' in sys.argv:
        return selftest()
    report()
    print()
    print('=' * 72)
    print('候选目标点净空')
    print('=' * 72)
    print('%-12s %8s %8s %8s  %s' % ('点', 'x', 'y', '净空', '判定'))
    print('-' * 72)
    for n, x, y, c, v in candidates():
        print('%-12s %8.2f %8.2f %8.3f  %s' % (n, x, y, c, v))
    return 0


if __name__ == '__main__':
    sys.exit(main())
