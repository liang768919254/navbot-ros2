#!/usr/bin/env python3
"""
check_limits.py —— 校验 nav2 参数与 docs/SAFETY_LIMITS.md 的推导是否一致

为什么需要它：
  inflation_radius 在两个 costmap 里各写一遍，robot_radius 也是两处。
  靠人肉保证"两处一致"迟早出错，而且出错很隐蔽 ——
  全局代价图按 0.25 规划、局部按 0.30 控车，通道里就会反复 abort，
  报错信息跟参数完全无关，能查一天。

判据（每条都是硬约束，不是偏好）：
  1. global 与 local costmap 的 inflation_radius 必须相等
  2. global 与 local costmap 的 cost_scaling_factor 必须相等
  3. global 与 local costmap 的 robot_radius 必须相等
  4. 2 * inflation_radius < NARROW_PASSAGE_WIDTH（0.6 m 窄通道）
  5. inflation_radius > robot_radius（否则膨胀没有意义）

用法：
  python3 tools/verify/check_limits.py
  python3 tools/verify/check_limits.py --yaml /path/to/nav2_params.yaml
"""
import argparse
import os
import sys

# ── 与 docs/SAFETY_LIMITS.md 同步的常量 ────────────────────────────────
NARROW_PASSAGE_WIDTH = 0.60   # 房间最窄通道全宽
ROBOT_RADIUS = 0.20
# ★ 2026-11-10: 0.25 → 0.22。理由见 nav2_params.yaml 的 inflation_layer 长注释，
#   一句话：0.6m 窄通道只是"够走"，真正卡死车的是缝N 0.450m ——
#   2×0.25=0.50 会把整条缝变成代价格，DWB 不敢走；2×0.22=0.44 留出 +0.010 零代价带。
# ★ 2026-11-10 夜（S32）：模型从「圆」换成「矩形足迹」。
#   0.25 → 0.22（缝N 卡死） → 0.16（换矩形足迹后，只需覆盖车宽一半 0.12 + 4cm 缓冲）
EXPECTED_INFLATION = 0.16
# 本项目最紧的可用缝（缝S 0.655）在膨胀后还剩多少可以走
SEAM_S_WIDTH = 0.655
EXPECTED_COST_SCALING = 5.0
# 车体矩形（与 urdf/navbot.xacro 的 box size="0.30 0.24 0.06" 一致）
EXPECTED_FOOTPRINT = [[0.15, 0.12], [0.15, -0.12], [-0.15, -0.12], [-0.15, 0.12]]
BODY_HALF_W = 0.12      # 车宽一半，贴边时最需要保护的尺寸

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'src', 'navbot_bringup', 'config', 'nav2_params.yaml')

PASS = 0
FAIL = 0


def ok(msg):
    global PASS
    PASS += 1
    print('  [PASS] %s' % msg)


def ng(msg):
    global FAIL
    FAIL += 1
    print('  [FAIL] %s' % msg)


def parse_yaml_minimal(path):
    """只用标准库把 yaml 拍平成 {缩进路径: 值}。

    ★ 不用 PyYAML：这个脚本要能在「没 source ROS、没装 pyyaml」的
      裸 python3 下跑，否则每次查参数都得先进环境，反而没人用。
      拍的粒度够用：我们只关心 name: value 形式，不解析嵌套列表。
    """
    entries = {}
    stack = []      # [(indent, key)]
    with open(path, encoding='utf-8') as f:
        for raw in f:
            line = raw.split('#', 1)[0].rstrip()
            if not line.strip():
                continue
            stripped = line.lstrip()
            indent = len(line) - len(stripped)
            if ':' not in stripped:
                continue
            key, _, val = stripped.partition(':')
            key = key.strip()
            val = val.strip()
            while stack and stack[-1][0] >= indent:
                stack.pop()
            path_parts = [k for _, k in stack] + [key]
            full = '/'.join(path_parts)
            if val:
                entries[full] = val
            else:
                stack.append((indent, key))
    return entries


def find_key(entries, key, containing):
    """找出路径里同时包含 containing 和以 key 结尾的条目。"""
    out = {}
    for full, val in entries.items():
        if containing in full and full.endswith('/' + key):
            out[full] = val
    return out


def to_float(v):
    try:
        return float(str(v).strip().strip('"').strip("'"))
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    args = ap.parse_args()

    if not os.path.isfile(args.yaml):
        print('!! 找不到 yaml: %s' % args.yaml)
        return 2

    print('=' * 66)
    print(' navbot 安全阈值一致性检查')
    print(' yaml: %s' % args.yaml)
    print(' 依据: docs/SAFETY_LIMITS.md')
    print('=' * 66)

    entries = parse_yaml_minimal(args.yaml)

    glob = find_key(entries, 'inflation_radius', 'global_costmap')
    loc = find_key(entries, 'inflation_radius', 'local_costmap')
    g_cs = find_key(entries, 'cost_scaling_factor', 'global_costmap')
    l_cs = find_key(entries, 'cost_scaling_factor', 'local_costmap')
    g_rr = find_key(entries, 'robot_radius', 'global_costmap')
    l_rr = find_key(entries, 'robot_radius', 'local_costmap')

    print('\n---- ① 解析结果 ----')
    for label, d in (('global inflation', glob), ('local  inflation', loc),
                     ('global cost_scaling', g_cs), ('local  cost_scaling', l_cs),
                     ('global robot_radius', g_rr), ('local  robot_radius', l_rr)):
        if d:
            for k, v in d.items():
                print('  %-22s %-8s  <- %s' % (label, v, k))
        else:
            print('  %-22s %s' % (label, '<未找到>'))

    print('\n---- ② 两处必须一致 ----')
    def compare(a, b, name):
        if not a or not b:
            ng('%s 至少一处缺失，无法比较' % name)
            return None
        va = [to_float(x) for x in a.values()]
        vb = [to_float(x) for x in b.values()]
        if None in va or None in vb:
            ng('%s 有非数值项' % name)
            return None
        if max(va) - min(va) < 1e-9 and max(vb) - min(vb) < 1e-9 and abs(va[0] - vb[0]) < 1e-9:
            ok('%s global=local=%g' % (name, va[0]))
            return va[0]
        ng('%s 不一致：global=%s local=%s' % (name, sorted(set(va)), sorted(set(vb))))
        return None

    inflation = compare(glob, loc, 'inflation_radius')
    compare(g_cs, l_cs, 'cost_scaling_factor')

    print('\n---- ②b 车体模型：footprint 与 robot_radius 只能二选一 ----')
    # ★ 2026-11-10 夜（S32）新增。病因见 docs/S32_从下面过不去.md：
    #   用 robot_radius 0.20（外接圆）时，缝S 0.655 m 被两端各推 0.20，
    #   车心合法带只剩 0.655-0.40 = 0.255 m，而车贴边实际只占 0.12 ——
    #   「能过的缝被判死」，车最终停在 (0.97,-1.81) 一片 253 里。
    #   换成 0.30×0.24 矩形后贴边只占 0.12，缝S 出 0.335 m 可用带。
    g_fp = find_key(entries, 'footprint', 'global_costmap')
    l_fp = find_key(entries, 'footprint', 'local_costmap')
    g_rr = find_key(entries, 'robot_radius', 'global_costmap')
    l_rr = find_key(entries, 'robot_radius', 'local_costmap')

    for label, d in (('global footprint', g_fp), ('local  footprint', l_fp),
                     ('global robot_radius', g_rr), ('local  robot_radius', l_rr)):
        if d:
            for k, v in d.items():
                print('  %-22s %s' % (label, v[:60]))
        else:
            print('  %-22s %s' % (label, '<未找到>'))

    def parse_footprint(val):
        import ast
        try:
            pts = ast.literal_eval(str(val).strip().strip('"').strip("'"))
        except Exception:
            return None
        if not isinstance(pts, list) or len(pts) < 3:
            return None
        try:
            return [(float(a), float(b)) for a, b in pts]
        except Exception:
            return None

    fp_global = [parse_footprint(v) for v in g_fp.values()] if g_fp else []
    fp_local = [parse_footprint(v) for v in l_fp.values()] if l_fp else []
    fp_global = [p for p in fp_global if p]
    fp_local = [p for p in fp_local if p]

    if fp_global and fp_local:
        ok('两处都用 footprint（矩形足迹），模型一致')
        pg, pl = fp_global[0], fp_local[0]
        if sorted(map(tuple, pg)) == sorted(map(tuple, pl)):
            ok('global / local footprint 顶点一致')
        else:
            ng('global / local footprint 顶点不一致：%s vs %s' % (pg, pl))
        xs = [p[0] for p in pl]
        ys = [p[1] for p in pl]
        length, width = 2 * max(map(abs, xs)), 2 * max(map(abs, ys))
        if abs(length - 0.30) < 1e-6 and abs(width - 0.24) < 1e-6:
            ok('footprint 尺寸 %.2f x %.2f 与 urdf box size 一致' % (length, width))
        else:
            ng('footprint 尺寸 %.2f x %.2f 与 urdf 的 0.30 x 0.24 不符'
               % (length, width))
        if g_rr or l_rr:
            ng('同时存在 robot_radius（%s / %s）—— nav2 里两者互斥，'
               'footprint 存在时 robot_radius 被忽略，留着只会误导人'
               % (g_rr, l_rr))
        else:
            ok('没有残留 robot_radius（Nav2 中两者互斥，混写会静默取 footprint）')
        # ★ 核心判据：膨胀半径必须覆盖「车最胖的那一边」并留缓冲
        if inflation is not None:
            if inflation <= width / 2:
                ng('inflation %.2f <= 车宽一半 %.2f —— 贴墙时车角直接压进致命格'
                   % (inflation, width / 2))
            elif inflation > width / 2 + 0.10:
                ng('inflation %.2f 比车宽一半 %.2f 多出 %.3f m —— '
                   '缝S(%.3f) 两端各被推 %.2f，可用带只剩 %.3f m，'
                   '正是 S32「从下面过不去」的成因'
                   % (inflation, width / 2, inflation - width / 2,
                      SEAM_S_WIDTH, inflation, SEAM_S_WIDTH - 2 * inflation))
            else:
                ok('inflation %.2f = 车宽一半 %.2f + %.2f 缓冲（合理区间）'
                   % (inflation, width / 2, inflation - width / 2))
    elif fp_global or fp_local:
        ng('只有一处用了 footprint，另一处仍是 robot_radius —— 必然错配')
    else:
        robot_r = compare(g_rr, l_rr, 'robot_radius')
        print('  [WARN] 仍在用 robot_radius（圆模型）。本项目已证明它会把缝S 判死，'
              '若是有意回退，请确认 8 点巡检仍全 PASS')

    print('\n---- ③ 硬约束 ----')
    if inflation is None:
        # ★ 这条分支必须有输出。原来 inflation 为 None 时整段静默跳过，
        #   读者会以为「③ 没输出 = 没问题」，实际是「根本没法判」。
        #   故障注入测试时抓到的：改坏 local 后 ② 报 FAIL，③ 一片空白。
        ng('inflation_radius 两处不一致或缺失，硬约束无法判定')
        # 退一步：用两处中出现的最严（最大）值来判，至少能指出「已经越过窄通道」这件事
        vals = []
        for d in (glob, loc):
            for v in (d or {}).values():
                f = to_float(v)
                if f is not None:
                    vals.append(f)
        if vals:
            worst = max(vals)
            if 2 * worst >= NARROW_PASSAGE_WIDTH:
                ng('最坏情况 inflation=%.2f，2 x = %.2f >= 窄通道 %.2f —— 通道会被封死'
                   % (worst, 2 * worst, NARROW_PASSAGE_WIDTH))
            else:
                print('  最坏情况 inflation=%.2f，2 x = %.2f < %.2f（仅作参考，仍需先统一两处）'
                      % (worst, 2 * worst, NARROW_PASSAGE_WIDTH))
    if inflation is not None:
        limit = NARROW_PASSAGE_WIDTH / 2.0
        if 2 * inflation < NARROW_PASSAGE_WIDTH:
            ok('2 x %.2f = %.2f < 窄通道 %.2f  （余量 %.2f m）'
               % (inflation, 2 * inflation, NARROW_PASSAGE_WIDTH,
                  NARROW_PASSAGE_WIDTH - 2 * inflation))
        else:
            ng('2 x %.2f = %.2f >= 窄通道 %.2f —— 通道会被膨胀封死，规划必失败'
               % (inflation, 2 * inflation, NARROW_PASSAGE_WIDTH))
        # ★ 余量按实际上限算，别写死「0.03」。原来固定说「不足 0.03 m」，
        #   而 0.28 的余量其实是 0.30-0.28=0.02... 但 0.28 时 2x=0.56，
        #   余量 0.04；两个口径混着说会自相矛盾。统一报「距上限还差多少」。
        if inflation > limit:
            pass   # 已由上面的 FAIL 覆盖
        elif limit - inflation < 0.05:
            print('  [WARN] 膨胀半径 %.2f 距上限 %.2f 只剩 %.3f m，'
                  '控制误差可能让起点落进膨胀区'
                  % (inflation, limit, limit - inflation))
        # ★ 2026-11-10 夜（S32）：这段原来是「inflation > robot_radius」，
        #   换成 footprint 后 robot_radius 不存在了，判据改由 ②b 段接管
        #   （比较 inflation 与车宽一半）。这里只在还留着 robot_radius 时才查。

        if abs(inflation - EXPECTED_INFLATION) > 1e-9:
            print('  [WARN] 与 SAFETY_LIMITS.md 记录的 %.2f 不同（当前 %.2f）'
                  % (EXPECTED_INFLATION, inflation))
            print('         —— 若是有意修改，请同步更新 SAFETY_LIMITS.md')
        else:
            ok('与 SAFETY_LIMITS.md 记录一致（%.2f）' % EXPECTED_INFLATION)

        if g_cs:
            cs = [to_float(x) for x in g_cs.values()][0]
            if abs(cs - EXPECTED_COST_SCALING) > 1e-9:
                print('  [WARN] cost_scaling_factor=%.2f，与文档记录 %.2f 不同'
                      % (cs, EXPECTED_COST_SCALING))
            else:
                ok('cost_scaling_factor 与文档一致（%.2f）' % EXPECTED_COST_SCALING)

    print('\n' + '=' * 66)
    print(' 合计: PASS=%d  FAIL=%d' % (PASS, FAIL))
    print(' 结论: %s' % ('全通' if FAIL == 0 else '有 %d 项没过' % FAIL))
    print('=' * 66)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
