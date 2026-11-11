#!/usr/bin/env python3
"""map_quality.py —— 用 /map 实测数据量化建图质量，别靠肉眼看截图。

为什么写这个（2026-11-10 的教训）：
  我看了一眼 RViz 截图，看到墙体黑边外侧有一圈绿色点，就断言
  「SLAM 回环对齐得勉强、地图有毛边」，还准备去调 slam_toolbox 的
  minimum_travel_distance / minimum_travel_heading。

  实测一算，全错：
    墙体现在 0.008 m 偏差（不到 1/6 个格子），最差的西墙 0.042 m（< 1 格）。
    未知区只占 2.2%，已知区 97.8%。
    绿色点本来就是激光实时扫描，贴在占据格外侧是正常渲染，不是毛边。

  ⇒ 肉眼看「糊不糊」完全不可靠。要用「地图上的墙 vs world 文件的真值」比对。

判据（本脚本会自评）：
  ① 每条墙的位置偏差 < 1 个栅格（0.05 m）        → 精度合格
  ② 未知区占比 < 15%                              → 覆盖率合格
  ③ 每条墙的「检出率」（真值跨度上有多少比例变成占据格）> 70% → 完整性合格

用法：
  source ~/navbot_ws/tools/verify/sim_fullchain/env.sh
  python3 map_quality.py                 # 默认对照 navbot_room.world
  python3 map_quality.py --save map.yaml # 同时存一份地图快照
"""
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid

# 真值统一从 room_geom 取，避免第二份手抄副本再走岔
# （上一版 WALLS/BOXES 的数字是手抄的，和 room_geom 各改各的，
#   这就是「几何算错三次」之后的第二个坑）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import room_geom as RG
except Exception:      # pragma: no cover - 只在文件缺失时触发
    RG = None

# ---- 真值：逐条抄自 navbot_bringup/worlds/navbot_room.world ----
# ★★ 这里必须用「朝向房间的那一侧表面」，不是墙的中心线！
#    激光只能打到墙的内表面，踩到的格子就记在内表面上。
#    拿中心线当基准会量出「系统性偏内半格」的假象 ——
#    2026-11-10 我就这么误判过一轮，以为地图偏了 0.067 m，
#    换成内表面真值后偏差只有 0.017 m。
#    外墙厚 0.1 → 内表面 = 中心线 ∓ 0.05
#      西墙 中心 -3.0 → 内表面 -2.95（朝 +x）
#      东墙 中心 +3.0 → 内表面 +2.95（朝 -x）
#      南墙 中心 -2.5 → 内表面 -2.45（朝 +y）
#      北墙 中心 +2.5 → 内表面 +2.45（朝 -y）
def _derive_truth():
    """从 room_geom 派生真值表，不再在这里手抄一份。

    ★ 2026-11-10 后的规矩：凡是「算错就撞墙」的数字只允许有**一个**来源。
      上一版 WALLS 是手抄的，和 room_geom.PARTITIONS 各改各的版本，
      room_geom 改了 stub 而 map_quality 还是旧值，表里就一直 FAIL 却查不出原因。

    外墙：room_geom 只给内表面 X_MIN/X_MAX/Y_MIN/Y_MAX，厚度用 WALL_T，
          取「朝房间的那一面」——激光只能打到内表面。
    隔板：PARTITIONS 已经是「地图占据格外包框」，直接取中心线和跨度。
    """
    if RG is None:
        return [], []
    walls = [
        ('wall_south', 'h', RG.Y_MIN, RG.X_MIN, RG.X_MAX, RG.WALL_T),
        ('wall_north', 'h', RG.Y_MAX, RG.X_MIN, RG.X_MAX, RG.WALL_T),
        ('wall_west',  'v', RG.X_MIN, RG.Y_MIN, RG.Y_MAX, RG.WALL_T),
        ('wall_east',  'v', RG.X_MAX, RG.Y_MIN, RG.Y_MAX, RG.WALL_T),
    ]
    boxes = []
    for name, kind, x0, x1, y0, y1 in RG.PARTITIONS:
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        dx, dy = x1 - x0, y1 - y0
        # 厚度（法向尺寸）：水平板看 y 跨度，竖直板看 x 跨度
        short = dy if kind == 'h' else dx
        # 判「独立小块」必须**两个方向都短**，只看 min 会把
        # 0.25×1.80 的 partition_short 也当成方块（0.25 <= 0.40）
        if max(dx, dy) <= 0.45:
            # 独立小块：两侧都是通路 → 走包围盒中心校验
            #
            # ★ 真值尺寸不能直接用 x1-x0！那只是**栅格外接框**：
            #   圆柱 r=0.15 在 5cm 栅格上必然占成 0.35 见方的方块
            #   （2r + 1 格 = 0.30 + 0.05），拿它当真实尺寸会永远差一格。
            #   形状得回 room_geom 的对象元数据查。
            if kind == 'c':
                tw = th = 2.0 * RG.PILLAR_B['r']
            else:
                tw, th = dx, dy
            boxes.append((name, cx, cy,
                          x0 - 0.15, x1 + 0.15, y0 - 0.15, y1 + 0.15,
                          tw, th))
        else:
            # 长条隔板：沿长度方向多点扫，法向取中心线
            c = cy if kind == 'h' else cx
            a, b = (x0, x1) if kind == 'h' else (y0, y1)
            walls.append((name, kind, c, a, b, short))
    return walls, boxes


WALLS, BOXES = _derive_truth()

# ---- 独立方块：两侧都是通路，必须用「包围盒中心」量，不能用逐墙法向扫描 ----
#
# ★★ 2026-11-10 踩坑。上一版把 partition_stub 塞进 WALLS，结果实测偏差 +0.175
#    报 FAIL，我一度以为是地图不准。剖开看才发现是**方法错**：
#
#    stub 在 x=-1.88 处的竖直剖面：占据格 y∈[-1.745, -1.445]
#      → 包围盒中心 -1.595，与真值偏差 0.000
#
#    但 measure_wall 取的是「沿法向离房间中心 (0,0) 最近的占据格」。
#    stub 两侧（上方 y>-1.445、下方 y<-1.745）**都是可走的通路**，
#    「离房间中心最近」这个判据对它毫无意义 —— 从 y=0 往下扫遇到的
#    第一个占据格是 stub **上沿** -1.445，不是中心。
#
#    判据本身没错（外墙只有内表面可见，取最近是对的），错在把它
#    套到了「两侧都可见的独立物体」上。
#
#    所以单开一类：直接穷举该矩形范围内的占据格，取包围盒中心比对。
MAX_DEV = 0.05      # 判据①：偏差上限 = 1 个栅格
MAX_UNKNOWN = 15.0  # 判据②：未知区上限 %
MIN_COVER = 70.0    # 判据③：单条**长墙**检出率下限 %
# 独立小块不适用 MIN_COVER：0.35 m 只有 7 格，遮挡/量化掉 3 格就剩一半，
# 而它的中心一旦被两侧对齐，偏差指标本身就够硬。所以改判「包围盒尺寸」。
BOX_SIZE_TOL = 0.10  # 判据③'：独立小块实测尺寸与真值之差上限 m


def get_map(timeout=25.0):
    rclpy.init()
    node = rclpy.create_node('map_quality')
    box = {}
    node.create_subscription(OccupancyGrid, '/map',
                             lambda m: box.setdefault('m', m), 10)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    m = box.get('m')
    node.destroy_node()
    rclpy.shutdown()
    return m


def _read_yaml(pgm_path):
    """从同名 yaml 里取 resolution / origin，缺省回落到 navbot_room 的量级。"""
    import os
    import re
    res, ox, oy = 0.05, 0.0, 0.0
    y = os.path.splitext(pgm_path)[0] + '.yaml'
    if not os.path.exists(y):
        return res, ox, oy
    with open(y, encoding='utf-8') as f:
        txt = f.read()
    m = re.search(r'resolution:\s*([-\d.]+)', txt)
    if m:
        res = float(m.group(1))
    m = re.search(r'origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)', txt)
    if m:
        ox, oy = float(m.group(1)), float(m.group(2))
    return res, ox, oy


def load_pgm(path, res=0.05, ox=-3.1, oy=-5.49):
    """离线读 PGM，造一个和 OccupancyGrid 同接口的桩，用于不启 ROS 复算判据。

    ★ 行序陷阱：PGM 自顶向下，占位栅格自底向上，读进来必须翻行。
      这一步写错，整套「偏差」指标都会变成「地图整体上下颠倒 + 全 FAIL」，
      而 RViz 看起来只是「地图有点歪」，极难定位。
    """
    with open(path, 'rb') as f:
        raw = f.read()
    # 头部形如: P5 换行 <w> 空格 <h> 换行 255 换行
    parts, idx, nread = [], 0, 0
    while nread < 4:
        while idx < len(raw) and raw[idx:idx + 1].isspace():
            idx += 1
        if raw[idx:idx + 1] == b'#':
            while idx < len(raw) and raw[idx:idx + 1] != b'\n':
                idx += 1
            continue
        j = idx
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        parts.append(raw[idx:j])
        idx = j
        nread += 1
    w, h = int(parts[1]), int(parts[2])
    body = raw[idx + 1: idx + 1 + w * h]

    class _Info:
        pass

    class _M:
        pass

    m = _M()
    m.info = _Info()
    m.info.resolution = res
    m.info.width = w
    m.info.height = h
    m.info.origin = _Info()
    m.info.origin.position = _Info()
    m.info.origin.position.x = ox
    m.info.origin.position.y = oy
    data = [0] * (w * h)
    for iy in range(h):
        base = (h - 1 - iy) * w          # ★ 翻行
        for ix in range(w):
            v = body[base + ix]
            data[iy * w + ix] = 100 if v < 100 else (254 if v > 220 else -1)
    # 参考 map_quality 的取值：PGM 里 0=占据,205=未知,254=空闲
    data = [100 if v == 100 else (-1 if v == -1 else 0) for v in data]
    m.data = data
    return m


def make_grid(m):
    res = m.info.resolution
    ox = m.info.origin.position.x
    oy = m.info.origin.position.y
    w, h = m.info.width, m.info.height
    data = m.data

    def x_of(ix):
        return ox + (ix + 0.5) * res

    def y_of(iy):
        return oy + (iy + 0.5) * res

    def ix_of(x):
        return int((x - ox) / res)

    def iy_of(y):
        return int((y - oy) / res)

    def val(ix, iy):
        if 0 <= ix < w and 0 <= iy < h:
            return data[iy * w + ix]
        return None

    return dict(res=res, ox=ox, oy=oy, w=w, h=h, data=data,
                x_of=x_of, y_of=y_of, ix_of=ix_of, iy_of=iy_of, val=val)


def measure_wall(g, name, kind, c, a, b, thick):
    """量墙的内表面位置，与真值比。

    ★★ 这个方法改过两次，两次都是因为「真值/测量方式」错了而不是地图错了：

    第一版：沿法向从远处往墙走，break 在第一个占据格。
      激光是从房间内部打到墙上的，内表面先被填成占据格，
      「第一个占据格」天然偏向房间内侧 → 系统性把墙量薄。

    第二版：找法向上整段连续占据格，取 (首+尾)/2。
      问题在于外墙只有**内表面一侧**能看到，外面是空的、根本没占据格，
      所以「首尾中点」被硬拉向内侧 0.05 m（半个墙厚）。

    这一版：直接取**离房间最近的占据格**（也就是 `min`，按 |coord| 判内），
      外墙就用内表面，薄隔断两侧都可见时取两侧中点。
      这样测的就正是「激光能看到的那一面」。

    ★ 真正的教训写在 WALLS 的注释里：真值必须取内表面，不能取中心线。
    """
    half = thick / 2.0
    span = half + 3.0 * g['res']
    picks = []
    total = 0
    steps = int(abs(b - a) / g['res'])
    for i in range(steps + 1):
        s = a + (b - a) * i / max(steps, 1)
        total += 1
        occ = []
        k = int(2 * span / g['res']) + 1
        for j in range(-k, k + 1):
            cc = c + j * g['res']
            if kind == 'h':
                v = g['val'](g['ix_of'](s), g['iy_of'](cc))
                coord = g['y_of'](g['iy_of'](cc))
            else:
                v = g['val'](g['ix_of'](cc), g['iy_of'](s))
                coord = g['x_of'](g['ix_of'](cc))
            if v == 100:
                occ.append(coord)
        if not occ:
            continue
        # 内表面 = 靠近房间原点那一侧。房间中心在 (0,0)，
        # 所以「绝对值更小」的那一格就是内表面。
        picks.append(min(occ, key=abs))

    if not picks:
        return dict(name=name, ok=False, dev=None, cover=0.0, n=0)
    picks.sort()
    med = picks[len(picks) // 2]
    cover = 100.0 * len(picks) / max(total, 1)
    return dict(name=name, ok=abs(med - c) < MAX_DEV, dev=med - c,
                mean=med, truth=c, cover=cover, n=len(picks))


def measure_box(g, name, cx, cy, x0, x1, y0, y1):
    """独立小方块的包围盒中心校验。

    为什么不能用 measure_wall：那套「取离房间中心最近的占据格」是为
    **外墙**设计的（外墙只有内表面一侧能被激光打到）。独立方块四侧
    都是通路，最近原则会锁到它靠房间心那一侧的**表面**，而不是中心，
    天然偏出半个尺寸。详见 BOXES 上方注释。

    做法：穷举搜索矩形内所有占据格，取坐标的 min/max 求包围盒中心。
    检出率 = 搜到的占据格数 / 物体的理论格数（长×宽 / 栅格面积）。
    """
    xs, ys = [], []
    for iy in range(int((y0 - g['oy']) / g['res']),
                    int((y1 - g['oy']) / g['res']) + 1):
        for ix in range(int((x0 - g['ox']) / g['res']),
                        int((x1 - g['ox']) / g['res']) + 1):
            if g['val'](ix, iy) != 100:
                continue
            xs.append(g['x_of'](ix))
            ys.append(g['y_of'](iy))

    if not xs:
        return dict(name=name, ok=False, dev=None, cover=0.0, n=0,
                    mean=None, truth=cx)

    bx = (min(xs) + max(xs)) / 2.0
    by = (min(ys) + max(ys)) / 2.0
    # 包围盒尺寸也报出来，方便一眼看出「是尺寸被吃掉」还是「整体平移」
    sx, sy = max(xs) - min(xs) + g['res'], max(ys) - min(ys) + g['res']
    # 偏移取位移更大的那个轴（方块两个方向的中心都必须对）
    dev = max(abs(bx - cx), abs(by - cy))
    return dict(name=name, ok=dev < MAX_DEV, dev=dev, n=len(xs),
                mean=(bx, by), size=(sx, sy), truth=(cx, cy),
                cover=min(100.0, 100.0 * len(xs) *
                          g['res'] * g['res'] / max(sx * sy, 1e-9)))


def main():
    if '--offline' in sys.argv:
        i = sys.argv.index('--offline')
        pgm = sys.argv[i + 1] if len(sys.argv) > i + 1 else 'map_built.pgm'
        # ★ 原点必须从同名 yaml 读，不能用默认值。
        #   默认值一旦和 yaml 不一致，整张地图在世界系里会整体平移，
        #   表现为「每条墙都偏同样的量」——看着像建图坏了，其实是读错了原点。
        res, ox, oy = _read_yaml(pgm)
        m = load_pgm(pgm, res, ox, oy)
        print('  [离线模式] 读 %s（不连 ROS）' % pgm)
        print('  分辨率 %.3f  原点 (%.3f, %.3f)' % (res, ox, oy))
    else:
        m = get_map()
    if m is None:
        sys.stderr.write('!! 25 秒内没收到 /map —— 仿真没起，或 slam_toolbox 没在跑\n'
                         '   先执行：bash live.sh\n')
        sys.exit(2)

    g = make_grid(m)
    data = g['data']
    tot = len(data)
    n_occ = data.count(100)
    n_free = data.count(0)
    n_unk = data.count(-1)
    unk_pct = 100.0 * n_unk / tot

    print('=== 地图概况 ===')
    print('  尺寸 %d×%d  分辨率 %.3f m  原点 (%.3f, %.3f)'
          % (g['w'], g['h'], g['res'], g['ox'], g['oy']))
    print('  占据 %5d 格 %5.1f%%' % (n_occ, 100.0 * n_occ / tot))
    print('  空闲 %5d 格 %5.1f%%' % (n_free, 100.0 * n_free / tot))
    print('  未知 %5d 格 %5.1f%%   ← 判据 < %.0f%%' % (n_unk, unk_pct, MAX_UNKNOWN))

    print('\n=== 逐条墙对比（真值 vs 地图）===')
    print('  %-18s %8s %8s %9s %8s %6s'
          % ('墙体', '真值', '实测', '偏差', '检出率', '判定'))
    print('  ' + '-' * 62)
    all_ok = True
    for name, kind, c, a, b, th in WALLS:
        r = measure_wall(g, name, kind, c, a, b, th)
        if r['dev'] is None:
            print('  %-18s %8.3f %8s %9s %7.0f%% %6s'
                  % (name, c, '未检出', '-', 0.0, 'FAIL'))
            all_ok = False
            continue
        good = r['ok'] and r['cover'] >= MIN_COVER
        all_ok &= good
        print('  %-18s %8.3f %8.3f %+9.3f %7.0f%% %6s'
              % (name, r['truth'], r['mean'], r['dev'], r['cover'],
                 'OK' if good else 'FAIL'))

    for name, cx, cy, x0, x1, y0, y1, tw, th in BOXES:
        r = measure_box(g, name, cx, cy, x0, x1, y0, y1)
        if r['dev'] is None:
            print('  %-18s %8s %8s %9s %7.0f%% %6s'
                  % (name, '(%.3f,%.3f)' % (cx, cy), '未检出', '-',
                     0.0, 'FAIL'))
            all_ok = False
            continue
        # 栅格化会把圆形撑大 1 格，所以比较基准加一个 res
        tw_g, th_g = tw + g['res'], th + g['res']
        size_ok = (abs(r['size'][0] - tw_g) < BOX_SIZE_TOL and
                   abs(r['size'][1] - th_g) < BOX_SIZE_TOL)
        good = r['ok'] and size_ok
        all_ok &= good
        print('  %-18s %8s %8s %+9.3f %7.0f%% %6s'
              % (name, '(%.3f,%.3f)' % r['truth'],
                 '(%.3f,%.3f)' % r['mean'], r['dev'], r['cover'],
                 'OK' if good else 'FAIL'))
        print('  %-18s %8s %8s %9s %8s'
              % ('', '真值尺寸', '%.2f×%.2f' % (tw, th),
                 '实测', '%.2f×%.2f' % r['size']))

    print('\n=== 总判定 ===')
    checks = [
        ('未知区 < %.0f%%' % MAX_UNKNOWN, unk_pct < MAX_UNKNOWN, '%.1f%%' % unk_pct),
        ('所有墙偏差 < %.3f m' % MAX_DEV, all_ok,
         '见上表'),
    ]
    for label, ok, detail in checks:
        print('  %s %-28s %s' % ('PASS' if ok else 'FAIL', label, detail))

    if '--save' in sys.argv:
        i = sys.argv.index('--save')
        path = sys.argv[i + 1] if len(sys.argv) > i + 1 else 'map.yaml'
        save_map(m, path)
        print('\n  地图已存：%s' % path)

    print('\n提醒：绿色激光点贴在占据格外侧是**正常**的 —— 占据格记的是'
          '\n      光线命中位置，实时扫描是本帧命中位置，两者本就该重合。'
          '\n      判断建图好坏请看上表数字，不要看 RViz 截图。')


def save_map(m, path):
    """存成 nav2_map_server 能直接读的 PGM + YAML。

    ★ 三个格式坑（2026-11-10 踩过）：
      1. `image:` 必须写**相对路径**（相对 yaml 所在目录）。写绝对路径在
         nav2_map_server 里能跑，但一旦把包安装到 install/share 就会失效，
         而且仓库里不该出现本机绝对路径。
      2. `free_thresh` 必须和仓库里现有地图一致（本仓库是 0.25），
         否则同一份 PGM 的两版地图判读规则不同，对比会失真。
      3. PGM 行序是**自顶向下**，占位栅格是**自底向上**（origin 在左下），
         写文件时必须 `for iy in reversed(range(h))`，否则地图上下颠倒 ——
         这个错误在 RViz 里看起来像「地图整体歪了」，极难定位。
    """
    import os
    yaml_dir = os.path.dirname(os.path.abspath(path)) or '.'
    pgm_path = os.path.splitext(path)[0] + '.pgm'
    pgm_name = os.path.basename(pgm_path)

    w, h = m.info.width, m.info.height
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        for iy in range(h - 1, -1, -1):     # ★ PGM 自顶向下
            row = bytearray()
            for ix in range(w):
                v = m.data[iy * w + ix]
                # 与 map_server 的 trinary 约定一致：
                #   占据 → 0（黑）  未知 → 205（灰）  空闲 → 254（白）
                row.append(0 if v == 100 else (205 if v == -1 else 254))
            f.write(bytes(row))

    with open(path, 'w', encoding='utf-8') as f:
        f.write('image: %s\n' % pgm_name)          # ★ 相对路径
        f.write('mode: trinary\n')
        f.write('resolution: %.6f\n' % m.info.resolution)
        f.write('origin: [%.6f, %.6f, 0.0]\n'
                % (m.info.origin.position.x, m.info.origin.position.y))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')             # ★ 与现有地图一致


if __name__ == '__main__':
    main()
