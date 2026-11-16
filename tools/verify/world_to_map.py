#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""world_to_map.py —— 从 Gazebo 世界/模型 SDF 直接生成 Nav2 用的占据栅格地图。

为什么需要它（2026-11-16 新写）
--------------------------------------------------------------------------
换复杂地图时，正常流程是「起 SLAM → 遥控走一圈 → map_saver」。三个问题：

  1. 复杂房子（15 m × 10.5 m、5 个房间）靠 auto_scan 走一圈要十几分钟，
     而且无头环境里没法肉眼确认哪块漏扫了；
  2. SLAM 图有噪声（墙厚 1~2 格、门框糊在一起），换图之后出问题
     分不清是「导航参数不对」还是「地图本身烂」；
  3. 世界里每个 box 的坐标是确定的 —— 真值本来就在手里，
     没必要再用激光反推一遍。

所以这个脚本做一件事：**按激光扫描平面切一刀，把 SDF 里的几何栅格化成
占据栅格**。得到的是「几何真值图」：墙是墙、门是门，零噪声。

它不替代 SLAM（真机只能靠 SLAM），但做「导航算法/参数对比」时，
真值图能把「地图误差」这个变量彻底消掉 —— 这轮复杂地图实测就是这么用的。

用法
--------------------------------------------------------------------------
    # 世界文件（模型已内联，或 include model:// 但解析不到时用 --model）
    python3 tools/verify/world_to_map.py \\
        --world src/navbot_bringup/worlds/navbot_house.world \\
        --z 0.12 --out src/navbot_bringup/maps/navbot_house \\
        --seed -2.0 -0.5

    # 单独的模型 SDF（如 turtlebot3_house/model.sdf）
    python3 tools/verify/world_to_map.py --model house_model.sdf --z 0.12 --out /tmp/house

参数要点
--------------------------------------------------------------------------
  --z       激光平面高度（m）。navbot 的 laser_link 离地约 0.10~0.12 m，
            取 0.12。**这个值直接决定桌子/床挡不挡路**：桌子台面在 0.4 m，
            2D 雷达从下面穿过去，只看到四条腿 —— 脚本如实还原这一点，
            不会「为了好看」把家具整个涂黑。
  --band    切片厚度（默认 ±0.03 m）。太薄会漏掉贴地的薄板，
            太厚会把高处的台面误算进来。
  --seed    自由空间连通域的种子点（机器人出生点）。从种子点做 4 邻域
            洪泛，能走到的格子记 free，走不到的记 unknown。
            ★ 这一步是「墙外算未知」的关键，不做的话整张图都是白底，
              Nav2 会规划出穿墙的路径。

输出
--------------------------------------------------------------------------
  <out>.pgm   P5 灰度图，0=占据 / 205=未知 / 254=空闲（ROS map_server 约定）
  <out>.yaml  resolution / origin / negate / 阈值，可直接给 nav2_bringup 的 map 参数
  <out>.png   放大预览图（可选，--png），用来肉眼核对

已知限制
--------------------------------------------------------------------------
  · 只处理 box / cylinder / sphere；mesh 会被跳过并计数（本项目两个世界
    都没有 mesh 碰撞体，跳过数为 0）
  · 只支持绕 Z 的旋转（yaw）。世界里的 roll/pitch 会打警告并忽略
  · 不做膨胀 —— 膨胀是 Nav2 代价地图的事，地图只描述真实占据
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit('需要 numpy：sudo apt install python3-numpy')


# ---------------------------------------------------------------------------
# SDF 解析
# ---------------------------------------------------------------------------

def _f(txt, default=0.0):
    try:
        return float(txt)
    except (TypeError, ValueError):
        return default


def parse_pose(elem, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    """取 <pose>，返回 (x, y, z, roll, pitch, yaw)。没有 <pose> 就给 default。"""
    p = elem.find('pose')
    if p is None or not (p.text or '').strip():
        return default
    vals = [_f(v) for v in p.text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def compose_2d(parent, local):
    """父位姿 ∘ 局部位姿 —— 只算平面部分（x, y, yaw）+ 高度 z。

    参数约定：parent 是本函数上一次的输出 (x, y, z, yaw) 四元组，
    local 是 parse_pose() 的六元组 (x, y, z, r, p, yaw)。

    世界里的墙/家具几乎都是「先摆 model/link，再在里面偏一个位移」，
    旋转只绕 Z。所以平面变换用 2D 刚体复合就够，不需要完整 4x4。
    """
    lx, ly, lz, lyaw = parent
    px, py, pz = local[0], local[1], local[2]
    pyaw = local[5]
    x = lx + px * math.cos(lyaw) - py * math.sin(lyaw)
    y = ly + px * math.sin(lyaw) + py * math.cos(lyaw)
    return (x, y, lz + pz, lyaw + pyaw)


def collect_primitives(root, warn):
    """遍历所有 <link>，把每个碰撞体折成世界坐标下的图元。

    ★ 位姿是三层复合的：model → link → collision。
      navbot_room.world 把 <pose> 写在 <model> 上（link 里没有），
      turtlebot3_house 的 model.sdf 写在 <link> 上（model 上没有）。
      两层都要认，否则所有几何都堆在原点 —— 第一次跑就踩了这个坑：
      9 个图元全在 (0,0)，种子点自然「落在墙里」。

    返回 [(kind, cx, cy, cz, h, params), ...]
      box:      params = (sx, sy, yaw)
      cylinder: params = (r,)
      sphere:   params = (r,)
    """
    out = []
    skipped_mesh = 0

    def walk_model(model, parent_pose):
        nonlocal skipped_mesh
        mpose = compose_2d(parent_pose, parse_pose(model))
        # 嵌套 model（SDF 允许），先递归
        for sub in model.findall('model'):
            walk_model(sub, mpose)
        for link in model.findall('link'):
            lpose = compose_2d(mpose, parse_pose(link))
            if abs(lpose[3]) > 1e-3:
                pass  # 只有 yaw 时是精确的；roll/pitch 在 parse_pose 里已被压平
            # 碰撞体优先；没有 collision 的模型退化为用 visual（只影响美观模型）
            nodes = link.findall('collision') or link.findall('visual')
            for col in nodes:
                cpose = compose_2d(lpose, parse_pose(col))
                geom = col.find('geometry')
                if geom is None:
                    continue
                box = geom.find('box')
                cyl = geom.find('cylinder')
                sph = geom.find('sphere')
                if box is not None:
                    sz = box.find('size')
                    if sz is None or not (sz.text or '').strip():
                        continue
                    sx, sy, sz_z = [_f(v) for v in sz.text.split()[:3]]
                    # compose_2d 返回 (x, y, z, yaw)，只有 4 个分量
                    out.append(('box', cpose[0], cpose[1], cpose[2], sz_z,
                                (sx, sy, cpose[3])))
                elif cyl is not None:
                    r = _f((cyl.findtext('radius') or '0'))
                    h = _f((cyl.findtext('length') or '0'))
                    out.append(('cylinder', cpose[0], cpose[1], cpose[2], h, (r,)))
                elif sph is not None:
                    r = _f((sph.findtext('radius') or '0'))
                    out.append(('sphere', cpose[0], cpose[1], cpose[2], 2 * r, (r,)))
                elif geom.find('mesh') is not None:
                    skipped_mesh += 1

    # 根节点要么是 <sdf>（下挂 <world> 再挂 <model>），要么本身就是模型
    # （model.sdf）。两种都从容器往下一层走一次 —— 用 root.iter('model')
    # 会把嵌套 model 走两遍，栅格化图元数直接翻倍。
    if root.tag == 'model':
        walk_model(root, (0.0, 0.0, 0.0, 0.0))
    else:
        container = root.find('world') if root.find('world') is not None else root
        for model in container.findall('model'):
            walk_model(model, (0.0, 0.0, 0.0, 0.0))

    if skipped_mesh:
        warn('跳过 %d 个 mesh 碰撞体（脚本只支持 box/cylinder/sphere）' % skipped_mesh)
    return out


# ---------------------------------------------------------------------------
# 栅格化
# ---------------------------------------------------------------------------

def rasterize(prims, z_plane, band, res, bounds, conservative=True):
    """把图元投影到 z=z_plane 的平面上，返回占据掩码。

    bounds = (x_min, y_min, x_max, y_max)

    ★★ conservative=True 是必须的（2026-11-16 踩坑后加）：
       「格子中心落在 box 内才算占据」这种天真做法会**让细物体整根消失** ——
       桌腿 0.042 m × 0.042 m，在 0.05 m 分辨率下一格中心都不含，
       于是地图上桌子是个空洞，而激光在 0.10 m 高度看得见桌腿。
       结果就是：全局规划器理直气壮地把路径画穿桌子，局部代价图却把它堵死，
       车开到桌前 0.5 m 再也不动（S35 里"右下房"那一 leg 就是这么挂的）。
       把每个图元按半格（res/2）外扩后再判，任何真实存在的物体至少占一格。
    """
    x_min, y_min, x_max, y_max = bounds
    w = int(math.ceil((x_max - x_min) / res))
    h = int(math.ceil((y_max - y_min) / res))
    mask = np.zeros((h, w), dtype=bool)          # 行 0 = y_min（数学坐标，最后翻转）
    half = res / 2.0 if conservative else 0.0
    hit = 0
    for kind, cx, cy, cz, height, params in prims:
        # 高度不落在切片里就完全跳过 —— 这就是「桌子台面不挡 2D 激光」的实现
        if not (cz - height / 2.0 - band <= z_plane <= cz + height / 2.0 + band):
            continue
        hit += 1
        if kind == 'box':
            sx, sy, yaw = params
            sx += 2 * half
            sy += 2 * half
            if sx <= 0 or sy <= 0:
                continue
            # 旋转矩形的 AABB 作为候选格，再逐格做点在矩形内判定
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            ex = (sx * c + sy * s) / 2.0
            ey = (sx * s + sy * c) / 2.0
            i0 = max(0, int((cx - ex - x_min) / res))
            i1 = min(w - 1, int((cx + ex - x_min) / res) + 1)
            j0 = max(0, int((cy - ey - y_min) / res))
            j1 = min(h - 1, int((cy + ey - y_min) / res) + 1)
            if i1 < i0 or j1 < j0:
                continue
            xs = x_min + (np.arange(i0, i1 + 1) + 0.5) * res - cx
            ys = y_min + (np.arange(j0, j1 + 1) + 0.5) * res - cy
            gx, gy = np.meshgrid(xs, ys)
            ca, sa = math.cos(-yaw), math.sin(-yaw)
            lx = gx * ca - gy * sa
            ly = gx * sa + gy * ca
            inside = (np.abs(lx) <= sx / 2.0) & (np.abs(ly) <= sy / 2.0)
            mask[j0:j1 + 1, i0:i1 + 1] |= inside
        else:
            r = params[0] + half
            i0 = max(0, int((cx - r - x_min) / res))
            i1 = min(w - 1, int((cx + r - x_min) / res) + 1)
            j0 = max(0, int((cy - r - y_min) / res))
            j1 = min(h - 1, int((cy + r - y_min) / res) + 1)
            if i1 < i0 or j1 < j0:
                continue
            xs = x_min + (np.arange(i0, i1 + 1) + 0.5) * res - cx
            ys = y_min + (np.arange(j0, j1 + 1) + 0.5) * res - cy
            gx, gy = np.meshgrid(xs, ys)
            mask[j0:j1 + 1, i0:i1 + 1] |= (gx ** 2 + gy ** 2) <= r ** 2
    return mask, hit


def aabb_of(kind, cx, cy, height, params):
    """图元在世界坐标下的 AABB（忽略 yaw 之外的旋转）。

    ★ 不能用 max(sx, sy)/2 一律当半径：6.0×0.1 的墙会被算成 6 m 的圆，
      地图凭空胀出一大圈，外壳裁剪也跟着失效。按轴各算一半，
      再按 yaw 求旋转后的投影。
    """
    if kind == 'box':
        sx, sy, yaw = params
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        ex = (sx * c + sy * s) / 2.0
        ey = (sx * s + sy * c) / 2.0
    else:
        ex = ey = params[0]
    return (cx - ex, cx + ex, cy - ey, cy + ey)


def flood_fill(free_candidates, seed_ij):
    """从种子点做 4 邻域洪泛，返回可达区域（= 真正的 free）。

    ★ 为什么必须做：世界文件里「墙外」只是一片空，几何上没有占据物。
      不做连通域判定的话，墙外全被当成 free，Nav2 的全局规划器会
      理直气壮地规划一条穿墙直线 —— 而且不报错，只是车顶着墙不动。
    """
    h, w = free_candidates.shape
    reach = np.zeros_like(free_candidates)
    si, sj = seed_ij
    if not (0 <= si < h and 0 <= sj < w) or not free_candidates[si, sj]:
        return reach
    stack = [(si, sj)]
    reach[si, sj] = True
    while stack:
        i, j = stack.pop()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < h and 0 <= nj < w and free_candidates[ni, nj] and not reach[ni, nj]:
                reach[ni, nj] = True
                stack.append((ni, nj))
    return reach


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

def write_pgm(path, img):
    """写 P5 PGM。img 的行序已经是「第 0 行 = 最大 y」（ROS 约定）。"""
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(b'# CREATOR: navbot world_to_map.py\n')
        f.write(('%d %d\n255\n' % (w, h)).encode())
        f.write(img.astype(np.uint8).tobytes())


def write_yaml(path, image_name, res, origin):
    with open(path, 'w') as f:
        f.write('image: %s\n' % image_name)
        f.write('mode: trinary\n')
        f.write('resolution: %.6f\n' % res)
        f.write('origin: [%.6f, %.6f, 0.0]\n' % (origin[0], origin[1]))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')


def main():
    ap = argparse.ArgumentParser(description='SDF 世界/模型 → Nav2 占据栅格地图')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--world', help='Gazebo 世界文件（.world）')
    src.add_argument('--model', help='Gazebo 模型文件（model.sdf）')
    ap.add_argument('--z', type=float, default=0.12, help='激光平面高度（m），默认 0.12')
    ap.add_argument('--band', type=float, default=0.03, help='切片半厚（m），默认 0.03')
    ap.add_argument('--res', type=float, default=0.05, help='分辨率（m/格），默认 0.05')
    ap.add_argument('--pad', type=float, default=0.6, help='地图四周留白（m），默认 0.6')
    ap.add_argument('--seed', nargs=2, type=float, default=None, metavar=('X', 'Y'),
                    help='自由空间种子点（机器人出生点），默认取占据物外包框中心')
    ap.add_argument('--out', required=True, help='输出前缀，会生成 <out>.pgm/.yaml')
    ap.add_argument('--png', action='store_true', help='额外输出放大 3 倍的预览 PNG')
    args = ap.parse_args()

    warns = []

    def warn(msg):
        warns.append(msg)
        print('  [警告] %s' % msg)

    path = args.world or args.model
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        sys.exit('解析 %s 失败：%s' % (path, e))
    root = tree.getroot()

    prims = collect_primitives(root, warn)
    if not prims:
        sys.exit('没解析到任何碰撞体 —— 世界文件里可能是 <include model://...>，'
                 '先把模型内联或用 --model 直接给模型 SDF')
    print('解析：%d 个图元（%s）' % (len(prims), os.path.basename(path)))

    # 地图范围 = 所有图元的 AABB + 留白
    x_min = y_min = float('inf')
    x_max = y_max = float('-inf')
    for kind, cx, cy, cz, height, params in prims:
        x0, x1, y0, y1 = aabb_of(kind, cx, cy, height, params)
        x_min = min(x_min, x0)
        x_max = max(x_max, x1)
        y_min = min(y_min, y0)
        y_max = max(y_max, y1)
    x_min -= args.pad
    x_max += args.pad
    y_min -= args.pad
    y_max += args.pad

    mask, hit = rasterize(prims, args.z, args.band, args.res, (x_min, y_min, x_max, y_max))
    h, w = mask.shape
    print('栅格：%d × %d 格（%.1f m × %.1f m），落在 z=%.2f±%.2f 切片里的图元 %d 个'
          % (w, h, w * args.res, h * args.res, args.z, args.band, hit))

    # 种子点 → 洪泛
    seed_xy = args.seed if args.seed else ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)
    si = int((seed_xy[1] - y_min) / args.res)
    sj = int((seed_xy[0] - x_min) / args.res)
    reach = flood_fill(~mask, (si, sj))
    if not reach.any():
        sys.exit('种子点 (%.2f, %.2f) 落在墙里或全被挡住，换一个 --seed' % tuple(seed_xy))

    img = np.full((h, w), 205, dtype=np.uint8)   # unknown
    img[reach] = 254                             # free
    img[mask] = 0                                # occupied（占据优先，压过 free）

    # ★ 外壳裁剪：房子模型往往有「通向室外」的开口（大门/阳台），
    #   洪泛会顺着开口把整片室外也标成 free，Nav2 于是能规划出跑到房子
    #   外面去的路径。这里用「全高墙体（≥1.5 m）的外包框」当房屋外壳，
    #   壳外的格子一律压回 unknown。
    #   ★ 房间世界（navbot_room）墙只有 0.5 m，没有图元满足条件 ⇒ 跳过，
    #     行为与以前完全一致。
    tall = [p for p in prims if p[4] >= 1.5]
    if tall:
        boxes = [aabb_of(p[0], p[1], p[2], p[4], p[5]) for p in tall]
        ex_min = min(b[0] for b in boxes)
        ex_max = max(b[1] for b in boxes)
        ey_min = min(b[2] for b in boxes)
        ey_max = max(b[3] for b in boxes)
        xs = x_min + (np.arange(w) + 0.5) * args.res
        ys = y_min + (np.arange(h) + 0.5) * args.res
        inside = ((xs >= ex_min) & (xs <= ex_max))[None, :] & \
                 ((ys >= ey_min) & (ys <= ey_max))[:, None]
        clipped = int((reach & ~inside).sum())
        img[(reach) & (~inside)] = 205
        reach = reach & inside
        print('外壳裁剪：墙体外包框 %.2f..%.2f × %.2f..%.2f，压回未知 %d 格（%.1f m²）'
              % (ex_min, ex_max, ey_min, ey_max, clipped, clipped * args.res ** 2))

    # ROS 地图行序：第 0 行对应最大 y
    img = img[::-1, :]

    out = args.out
    os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)
    write_pgm(out + '.pgm', img)
    write_yaml(out + '.yaml', os.path.basename(out) + '.pgm', args.res, (x_min, y_min))
    print('写出：%s.pgm / %s.yaml' % (out, out))
    print('  原点 (%.3f, %.3f)  分辨率 %.3f  自由 %.1f m²  占据 %d 格  未知 %.0f%%'
          % (x_min, y_min, args.res, reach.sum() * args.res ** 2, mask.sum(),
             100.0 * (~reach & ~mask).sum() / (w * h)))

    if args.png:
        try:
            from PIL import Image
        except ImportError:
            print('  [跳过] 没装 Pillow，不出 PNG')
        else:
            Image.fromarray(img).resize((w * 3, h * 3), Image.NEAREST).save(out + '.png')
            print('  预览：%s.png' % out)

    if warns:
        print('共 %d 条警告，见上。' % len(warns))


if __name__ == '__main__':
    main()
