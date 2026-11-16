#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""path_select.py —— 多规划器「算同一批路径 → 逐条打分 → 选一个」的筛选工具。

为什么需要它（2026-11-16 新写）
--------------------------------------------------------------------------
房间图（6×5 m）里 A→B 只有一条路，规划器好坏看不出来。
换成复杂房子（15×10.5 m、5 房间、成环走廊）之后，同一个目标点往往有
**两三条长度接近但风险完全不同**的路线：

    绕远 8 m 走宽走廊     vs     抄近 5 m 挤 0.42 m 的门缝

NavFn 只看「无致命格下的最短」，它会选后者；Smac2D 把膨胀代价算进 g 值，
权重给对了会选前者。到底谁更适合这台车、这个房子，**不能靠读源码猜**，
要拿同一批起终点把三个规划器都跑一遍，量出长度/净空/转角/耗时再比。

这个脚本就是那把尺子。它不下发速度、不驱动车，只调
`/compute_path_to_pose` 动作（Nav2 的 ComputePathToPose 带 planner_id 字段，
一次装载多个插件就能逐个点名），纯规划层对比，**10 秒出结果**，
不用等 20 分钟仿真 —— 先用它筛掉明显不行的，再上真跑。

用法
--------------------------------------------------------------------------
    # 1) 先起仿真 + Nav2（两个终端，见 README「仿真流程」）
    # 2) 跑筛选：起终点直接在命令行给
    python3 tools/verify/path_select.py \\
        --start -2.0 -0.5 \\
        --goal 3.0 2.0  --goal -6.5 -3.0 --goal 5.5 -4.0

    # 3) 或者把一批点写进 JSON（推荐，便于复现）
    python3 tools/verify/path_select.py --route-file tools/verify/routes_house.json

    # 4) 只比两个规划器（快速回归）
    python3 tools/verify/path_select.py --start -2.0 -0.5 --goal 3.0 2.0 \\
        --planners GridBased,Smac2D

输出
--------------------------------------------------------------------------
每个目标一行 × 每个规划器一列，打印：

    长度 m | 最小净空 m | 转角次数 | 平均代价 | 规划耗时 ms | 综合分

「综合分」越低越好，权重写在 SCORE_WEIGHTS 里并可 --weights 覆盖：

    0.45 × 长度（归一化）
  + 0.30 × (1 - 最小净空)      ← 净空越小扣分越多
  + 0.15 × 转角次数（归一化）
  + 0.10 × 平均软代价（归一化）

★ 为什么长度权重最大：这是「导航」不是「越野」，绕远 60% 换 10 cm 净空
  不值。反过来，如果实测发现某条路虽然短但老卡住，就把净空权重调上去
  —— 权重是**可调的判断**，不是真理，所以放在明面上而不是藏进代码。

判据（脚本自己检查，不是人肉看）
--------------------------------------------------------------------------
  · 三个插件都真的被 planner_server 加载了（读参数，不信 yaml）
  · 每条路径的起点必须接近给定 start（偏差 > 0.10 m 报 WARN，
    因为 NavFn 的 tolerance 会让它在起点附近做一点松弛）
  · 路径不得穿过致命格（逐点查地图：占据格上的点计数）
  · 规划耗时 < 500 ms（超了说明参数或地图有问题，先修再谈选路）
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


# 综合分权重（越低越好）。改这里之前先想清楚「这台车最怕什么」。
SCORE_WEIGHTS = {
    'length': 0.45,     # 绕远要罚，但罚不过「贴着墙开」
    'clearance': 0.30,  # 最小净空越小越危险
    'turns': 0.15,      # 转角多 → 多次原地转/减速，时间都花在这
    'cost': 0.10,       # 平均软代价，反映整条路径的"拥挤度"
}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(frame, x, y, yaw=0.0):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    qx, qy, qz, qw = yaw_to_quat(yaw)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class PathSelector(Node):
    def __init__(self, planners, timeout=10.0):
        super().__init__('path_select')
        self.planners = planners
        self.timeout = timeout
        self.map_msg = None
        self.costmap_msg = None

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        # 全局代价地图带膨胀层，用来算「平均软代价」
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_cost, latched)
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    # ---------------------------------------------------------------- 回调
    def _on_map(self, msg):
        self.map_msg = msg

    def _on_cost(self, msg):
        self.costmap_msg = msg

    # ------------------------------------------------------------ 等依赖
    def wait_ready(self):
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map_msg is not None and self.client.server_is_ready():
                break
        if self.map_msg is None:
            raise RuntimeError('/map 没数据 —— 先起 nav_bringup（map_server 要 active）')
        if not self.client.server_is_ready():
            raise RuntimeError('compute_path_to_pose 动作服务器不在 —— planner_server 没起来？')

    # ---------------------------------------------------------- 插件自检
    def loaded_plugins(self):
        """问 planner_server 自己要插件清单，而不是读 yaml（S29 的教训）。

        ★ 坑：Node.get_parameters() 只能读**自己节点**的参数。
          读别人的参数必须走 /planner_server/get_parameters 服务。
          第一版就是直接调 self.get_parameters()，结果永远返回空，
          被当成「插件没加载」—— 又是一次「问自己而不是问系统」。
        """
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, '/planner_server/get_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            return {p: '' for p in self.planners}
        req = GetParameters.Request()
        req.names = ['%s.plugin' % p for p in self.planners]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            return {p: '' for p in self.planners}
        out = {}
        for pid, val in zip(self.planners, res.values):
            out[pid] = val.string_value or ''
        return out

    # ------------------------------------------------------------ 几何量
    def _grid_xy(self, msg, x, y):
        """世界坐标 → 栅格下标；不在图内返回 None。

        ★ 行序坑（第一版就在这里翻车）：OccupancyGrid 的 data 是
          **row 0 = 最小 y**（左下图元起），和 PGM 图片「row 0 = 最大 y」
          正好相反。map_server 读 PGM 时替我们翻了一次。
          对着 PGM 写 `height-1-i` 的人（我）会在这里再翻一次 →
          路径点被镜像到墙里，于是每条路径都「过致命格 100+ 点」。
        """
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        j = int((x - ox) / res)
        i = int((y - oy) / res)
        if 0 <= i < msg.info.height and 0 <= j < msg.info.width:
            return i, j
        return None

    def clearance_field(self):
        """从 /map 算「每格到最近障碍的距离」（米）。用 scipy 的 EDT，一次算完。"""
        from scipy import ndimage
        msg = self.map_msg
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        # ★ 值域坑：OccupancyGrid 里 100=占据 / 0=空闲 / -1=未知，
        #   不是 costmap 的 0~252。第一版写 data > 50 其实是把
        #   「未知」(-1) 当成了可走，于是墙外的空白区净空被高估。
        #   这里按「非 0 即不可碰」处理（未知=不可碰，保守一侧）。
        occ = data != 0
        # EDT 给的是「到最近 True 的距离（格）」，取反后算障碍的距离
        dist = ndimage.distance_transform_edt(~occ) * msg.info.resolution
        return dist

    def cost_field(self, wait=5.0):
        # ★ 代价地图 1 Hz 才发一次，且是 transient_local。
        #   第一版直接读，常常还没收到 → 平均代价恒为 0.000（白跑一轮）。
        #   这里最多等 wait 秒；等不到就返回 None，让上层把这一列标成 "-"
        #   而不是假装 0（「不知道」和「零」必须能区分）。
        t0 = time.time()
        while self.costmap_msg is None and time.time() - t0 < wait:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.costmap_msg is None:
            return None, None
        msg = self.costmap_msg
        data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        return data, msg

    # ------------------------------------------------------------ 发请求
    def compute(self, planner_id, start_xy, goal_xy, goal_yaw=0.0, frame='map'):
        goal = ComputePathToPose.Goal()
        goal.goal = make_pose(frame, goal_xy[0], goal_xy[1], goal_yaw)
        goal.start = make_pose(frame, start_xy[0], start_xy[1], 0.0)
        goal.planner_id = planner_id
        goal.use_start = True

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None, float('nan'), 'REJECTED'
        t0 = time.time()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=self.timeout)
        dt = time.time() - t0
        res = res_future.result()
        if res is None:
            return None, dt, 'TIMEOUT'
        path = res.result.path
        if not path.poses:
            return None, dt, 'EMPTY'
        return path, dt, 'OK'

    # ------------------------------------------------------------ 指标
    def metrics(self, path, dist, cost_grid, cost_msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        seg = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1)]
        length = float(sum(seg))

        # 最小净空：沿路径逐点查 EDT
        min_clr = float('inf')
        hit_lethal = 0
        for x, y in pts:
            ij = self._grid_xy(self.map_msg, x, y)
            if ij is None:
                continue
            v = dist[ij]
            min_clr = min(min_clr, float(v))
            # 致命格：地图占据 或 代价 >= 253
            if v <= 0.0:
                hit_lethal += 1
        if not math.isfinite(min_clr):
            min_clr = 0.0

        # 转角次数：相邻两段的夹角 > 30° 记一次
        turns = 0
        for i in range(1, len(seg)):
            if seg[i - 1] < 1e-6 or seg[i] < 1e-6:
                continue
            a = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
            b = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
            d = abs(math.atan2(math.sin(b - a), math.cos(b - a)))
            if d > math.radians(30.0):
                turns += 1

        # 平均软代价（0..1）。代价地图没收到就给 None —— 「不知道」不能当 0。
        mean_cost = None
        if cost_grid is not None:
            vals = []
            for x, y in pts:
                ij = self._grid_xy(cost_msg, x, y)
                if ij is None:
                    continue
                # /global_costmap/costmap 是 OccupancyGrid：值域 0~100（100=致命），
                # 不是 0~252。除错了的话平均代价会恒显示 0.00（第一版就是）。
                vals.append(min(max(int(cost_grid[ij]), 0), 100) / 100.0)
            if vals:
                mean_cost = float(sum(vals) / len(vals))

        return {
            'length': length,
            'min_clearance': min_clr,
            'turns': turns,
            'mean_cost': mean_cost,
            'lethal_points': hit_lethal,
            'n_pts': len(pts),
            'start': pts[0],
            'end': pts[-1],
        }


def pct_norm(vals):
    """把一组数归一化到 0..1（都相等时全给 0.5，避免除零）。"""
    lo, hi = min(vals), max(vals)
    if hi - lo < 1e-9:
        return [0.5 for _ in vals]
    return [(v - lo) / (hi - lo) for v in vals]


def render_overlay(map_msg, per_goal, out_path, up=2):
    """把三个规划器算出的路径叠在同一张地图上，肉眼比「谁在抄近路」。

    ★ 为什么非要有图：表格只能告诉你「最小净空 0.14 vs 0.21」，
      看不出那 7 cm 是发生在门口还是发生在墙角。图上一眼就能看出
      某条路径是不是贴着一整面墙走 —— 那是表格数字表达不了的信息。
    """
    from PIL import Image, ImageDraw

    res = map_msg.info.resolution
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    w, h = map_msg.info.width, map_msg.info.height
    data = np.array(map_msg.data, dtype=np.int16).reshape(h, w)

    rgb = np.full((h, w, 3), 205, np.uint8)
    rgb[data < 0] = (200, 200, 200)     # 未知
    rgb[data == 0] = (248, 248, 248)    # 空闲
    rgb[data >= 50] = (40, 40, 40)      # 占据

    img = Image.fromarray(rgb[::-1]).resize((w * up, h * up), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    colors = [(230, 120, 30), (30, 110, 220), (150, 60, 180), (0, 150, 120)]

    def px(x, y):
        return ((x - ox) / res * up, ((h - 1) - (y - oy) / res) * up)

    for gi, (name, paths) in enumerate(per_goal):
        for pi, (pid, pts) in enumerate(paths):
            if len(pts) < 2:
                continue
            col = colors[pi % len(colors)]
            draw.line([px(x, y) for x, y in pts], fill=col, width=2)
            draw.ellipse([px(pts[-1][0], pts[-1][1])[0] - 5,
                          px(pts[-1][0], pts[-1][1])[1] - 5,
                          px(pts[-1][0], pts[-1][1])[0] + 5,
                          px(pts[-1][0], pts[-1][1])[1] + 5], outline=col, width=2)
            # 终点旁标序号，避免多目标图里认错
            tx, ty = px(pts[-1][0], pts[-1][1])
            draw.text((tx + 7, ty - 12), '%d' % (gi + 1), fill=col)
    img.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser(description='多规划器路径筛选')
    ap.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'),
                    help='起点（map 坐标）。用 --route-file 时可省略')
    ap.add_argument('--goal', nargs=2, type=float, action='append', metavar=('X', 'Y'),
                    help='目标点，可重复')
    ap.add_argument('--route-file', help='JSON：{"start":[x,y],"goals":[{"name":..,"x":..,"y":..,"yaw":..}]}')
    ap.add_argument('--planners', default='GridBased,NavFn,SmacHybrid',
                    help='planner_id 列表，逗号分隔（必须是 nav2_params 里 planner_plugins 的子集）'
                    '。注意：GridBased 现在指向 SmacPlanner2D，NavFn 才是原来的 NavFn')
    ap.add_argument('--json', help='结果写 JSON')
    ap.add_argument('--png', help='把各规划器路径叠在地图上输出 PNG（肉眼比路线）')
    ap.add_argument('--png-up', type=int, default=2, help='PNG 放大倍数，默认 2')
    ap.add_argument('--timeout', type=float, default=10.0, help='单次动作超时（s）')
    args = ap.parse_args()

    # ---- 目标列表：命令行优先，否则读 JSON
    goals = []
    start = args.start
    if args.route_file:
        with open(args.route_file) as f:
            spec = json.load(f)
        start = spec['start']
        goals = [(g.get('name', 'P%d' % i), g['x'], g['y'], g.get('yaw', 0.0))
                 for i, g in enumerate(spec['goals'])]
    elif args.goal:
        goals = [('P%d' % i, g[0], g[1], 0.0) for i, g in enumerate(args.goal)]
    if not start or not goals:
        ap.error('至少给 --start 和 --goal，或用 --route-file')

    planners = [p.strip() for p in args.planners.split(',') if p.strip()]

    rclpy.init()
    node = PathSelector(planners, timeout=args.timeout)
    try:
        node.wait_ready()
        print('地图 %d×%d 格 / %.3f m  起点 (%.2f, %.2f)  目标 %d 个  规划器 %s'
              % (node.map_msg.info.width, node.map_msg.info.height,
                 node.map_msg.info.resolution, start[0], start[1], len(goals),
                 '/'.join(planners)))

        plugins = node.loaded_plugins()
        missing = [p for p, v in plugins.items() if not v]
        if missing:
            print('  [警告] 这些 planner_id 在 planner_server 里查不到 plugin 参数：%s'
                  % ', '.join(missing))
            print('         说明 nav2_params.yaml 的 planner_plugins 里没加载它 —— 先修配置再来比。')
        else:
            for p in planners:
                print('  %-11s → %s' % (p, plugins[p]))

        dist = node.clearance_field()
        cost_grid, cost_msg = node.cost_field()

        records = []
        per_goal = []
        for name, gx, gy, gyaw in goals:
            print('\n=== %s  (%.2f, %.2f) ===' % (name, gx, gy))
            rows = []
            drawn = []
            for pid in planners:
                path, dt, st = node.compute(pid, start, (gx, gy), gyaw)
                if st != 'OK':
                    print('  %-11s %-8s   （规划失败）' % (pid, st))
                    records.append({'goal': name, 'planner': pid, 'status': st})
                    rows.append(None)
                    continue
                drawn.append((pid, [(p.pose.position.x, p.pose.position.y)
                                    for p in path.poses]))
                m = node.metrics(path, dist, cost_grid, cost_msg)
                # 起点偏差：NavFn 的 tolerance 会松一点，> 0.10 m 报 WARN
                dev = math.hypot(m['start'][0] - start[0], m['start'][1] - start[1])
                m.update({'goal': name, 'planner': pid, 'status': 'OK',
                          'plan_ms': dt * 1000.0, 'start_dev': dev})
                rows.append(m)
                records.append(m)

            ok = [r for r in rows if r]
            if not ok:
                continue
            # 归一化打分（只在成功的规划器之间比）
            nl = pct_norm([r['length'] for r in ok])
            nc = pct_norm([r['min_clearance'] for r in ok])
            nt = pct_norm([r['turns'] for r in ok])
            nk = pct_norm([r['mean_cost'] if r['mean_cost'] is not None else 0.0
                           for r in ok])
            print('  %-11s %7s %9s %6s %8s %8s %7s' %
                  ('planner', '长度m', '最小净空', '转角', '平均代价', '耗时ms', '综合分'))
            for i, r in enumerate(ok):
                score = (SCORE_WEIGHTS['length'] * nl[i]
                         + SCORE_WEIGHTS['clearance'] * (1.0 - nc[i])
                         + SCORE_WEIGHTS['turns'] * nt[i]
                         + SCORE_WEIGHTS['cost'] * nk[i])
                r['score'] = score
                flags = []
                if r['start_dev'] > 0.10:
                    flags.append('起点偏%.2fm' % r['start_dev'])
                if r['lethal_points'] > 0:
                    flags.append('过致命格%d点' % r['lethal_points'])
                if r['plan_ms'] > 500:
                    flags.append('规划慢')
                print('  %-11s %7.2f %9.2f %6d %8.3f %8.1f %7.3f  %s'
                      % (r['planner'], r['length'], r['min_clearance'], r['turns'],
                         r['mean_cost'] if r['mean_cost'] is not None else float('nan'),
                         r['plan_ms'], score, ' '.join(flags)))
            per_goal.append((name, drawn))
            best = min(ok, key=lambda r: r['score'])
            print('  → 本段最优：%s（综合分 %.3f）' % (best['planner'], best['score']))

        # ---- 汇总
        print('\n=== 汇总 ===')
        agg = {}
        for pid in planners:
            rs = [r for r in records if r.get('planner') == pid]
            ok = [r for r in rs if r.get('status') == 'OK']
            agg[pid] = {
                'n': len(rs),
                'ok': len(ok),
                'fail': len(rs) - len(ok),
                'mean_score': float(np.mean([r['score'] for r in ok])) if ok else float('nan'),
                'mean_len': float(np.mean([r['length'] for r in ok])) if ok else float('nan'),
                'min_clr': float(np.min([r['min_clearance'] for r in ok])) if ok else float('nan'),
                'mean_ms': float(np.mean([r['plan_ms'] for r in ok])) if ok else float('nan'),
            }
            print('  %-11s 成功 %d/%d  平均综合分 %.3f  平均长度 %.2f m  最小净空 %.2f m  平均耗时 %.0f ms'
                  % (pid, agg[pid]['ok'], agg[pid]['n'], agg[pid]['mean_score'],
                     agg[pid]['mean_len'], agg[pid]['min_clr'], agg[pid]['mean_ms']))

        ranked = sorted([k for k in agg if agg[k]['ok'] > 0],
                        key=lambda k: agg[k]['mean_score'])
        if ranked:
            print('\n  结论：按当前权重，%s 最优（平均综合分 %.3f）。'
                  % (ranked[0], agg[ranked[0]]['mean_score']))
            print('       权重：%s' % json.dumps(SCORE_WEIGHTS, ensure_ascii=False))
            print('       ★ 这是「规划层」的结论。真要下结论还得看执行层：')
            print('         同一条路径 DWB 能不能跟得住（docs/S35 有实测数据）。')

        if args.png:
            render_overlay(node.map_msg, per_goal, args.png, up=args.png_up)
            print('\n  路径对比图：%s' % args.png)

        if args.json:
            out = {'weights': SCORE_WEIGHTS, 'start': list(start),
                   'planners': planners, 'records': records, 'summary': agg}
            os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
            with open(args.json, 'w') as f:
                json.dump(out, f, ensure_ascii=False, indent=1)
            print('  结果已写入 %s' % args.json)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
