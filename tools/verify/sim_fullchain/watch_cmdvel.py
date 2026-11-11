#!/usr/bin/env python3
"""
watch_cmdvel.py —— 旁听 /cmd_vel_nav，用数据证明 shim 到底转没转

为什么需要：
  S28/S29 两轮都在争一件事：「车到底原地转了吗？」
  日志里 shim 的那句 "Robot is not within the new path's rough heading,
  rotating to heading..." 是 DEBUG 级别，默认看不到；
  而 INFO 级别只有 activate 时那一行 "Activating controller"。
  ⇒ 以前只能靠「结果对了」反推「过程对了」，这是不严谨的。

  这个脚本旁听 controller_server 直接发给速度平滑器的指令，
  把每一段运动分类，输出一份「运动画像」：

    PURE_ROTATE    linear ≈ 0 且 |angular| > 阈值   ← shim 原地转的指纹
    PURE_LINEAR    |angular| ≈ 0 且 linear > 阈值
    MIXED          两者都有                          ← 「边转边走」，差速车窄处危险
    STOP           两者都 ≈ 0

判据（和 S28 的分析对齐）：
  · 出现在「起步阶段」的 PURE_ROTATE 段 = shim 介入成功
  · 出现 MIXED 段且时间较长 = 有人在边转边走，检查 rotate_to_heading 逻辑
  · 全程没有 PURE_ROTATE 但大角度转向成功 = DWB 自己转过去了（也可能，但风险高）

用法：
  # 终端 A：先起仿真 + 导航
  bash tools/verify/sim_fullchain/live.sh nav

  # 终端 B：开记录（后台跑，Ctrl+C 结束）
  python3 tools/verify/sim_fullchain/watch_cmdvel.py

  # 终端 C：发目标
  python3 tools/verify/sim_fullchain/nav_patrol.py --one 2.20 -1.50

  # 记录结束后在终端 B 看画像，或事后看文件
  python3 tools/verify/sim_fullchain/watch_cmdvel.py --replay cmdvel.csv
"""
import argparse
import math
import os
import sys

# ★ 阈值不是拍的：
#   linear 0.01 m/s  —— max_vel_x 0.22 的 4.5%，低于这个数是控制器在做数值逼近，不是真的在走
#   angular 0.05 rad/s —— max_vel_theta 1.0 的 5%，同上
#   ★ 差速车 linear 恒 >= 0，所以不判负值
LIN_EPS = 0.01
ANG_EPS = 0.05

# 一段运动至少持续这么久才算「一段」，避免把抖动切成碎片
MIN_SEG_SEC = 0.15


def classify(v, w):
    if abs(v) <= LIN_EPS and abs(w) <= ANG_EPS:
        return 'STOP'
    if abs(v) <= LIN_EPS:
        return 'PURE_ROTATE'
    if abs(w) <= ANG_EPS:
        return 'PURE_LINEAR'
    return 'MIXED'


def segments(samples):
    """把 (t, v, w) 序列压成 (kind, t0, t1, 累计转角, 累计位移) 段列表。"""
    segs = []
    cur = None
    prev_t = None
    for t, v, w in samples:
        kind = classify(v, w)
        if cur is None or cur['kind'] != kind:
            if cur is not None:
                segs.append(cur)
            cur = {'kind': kind, 't0': t, 't1': t, 'dtheta': 0.0, 'dxy': 0.0}
        else:
            if prev_t is not None:
                dt = t - prev_t
                cur['dtheta'] += abs(w) * dt
                cur['dxy'] += abs(v) * dt
            cur['t1'] = t
        prev_t = t
    if cur is not None:
        segs.append(cur)
    # 丢掉太短的碎片（合并回前一段更合理，但这里只做过短过滤，保持简单）
    return [s for s in segs if (s['t1'] - s['t0']) >= MIN_SEG_SEC or s['kind'] != 'STOP']


def report(segs):
    tot = {}
    for s in segs:
        d = s['t1'] - s['t0']
        tot[s['kind']] = tot.get(s['kind'], 0.0) + d

    total = sum(tot.values()) or 1.0
    print('\n' + '=' * 72)
    print(' 运动画像（/cmd_vel_nav 旁听结果）')
    print('=' * 72)
    print(' %-14s %8s %8s' % ('类型', '时长(s)', '占比'))
    print(' ' + '-' * 34)
    for k in ['PURE_ROTATE', 'PURE_LINEAR', 'MIXED', 'STOP']:
        d = tot.get(k, 0.0)
        if d <= 0:
            continue
        tag = ''
        if k == 'PURE_ROTATE':
            tag = '  ← shim 原地转的指纹'
        elif k == 'MIXED':
            tag = '  ← 边转边走，窄处要小心'
        print(' %-14s %8.2f %7.1f%%%s' % (k, d, 100 * d / total, tag))
    print()

    rot = [s for s in segs if s['kind'] == 'PURE_ROTATE' and s['dtheta'] > 0.15]
    if rot:
        print('★ 检测到 %d 段真正的原地转（转角 > 0.15 rad ≈ 8.6°）：' % len(rot))
        for s in rot:
            print('   %8.2fs 起  转 %5.1f°  用时 %.2fs'
                  % (s['t0'], math.degrees(s['dtheta']), s['t1'] - s['t0']))
        print('  ⇒ RotationShimController 确实介入了，DWB 没被要求「摸着墙转」。')
    else:
        print('★ 没有检测到「转角 > 0.15 rad 的纯原地转」。')
        print('  可能情况：')
        print('   a) 本次路线本来就不需要大角度起步（正常）；')
        print('   b) shim 没触发，是 DWB 自己转过去的 —— 若还有卡住，优先级查这里；')
        print('   c) 旁听开始时导航已经走了一段（错过起步瞬间）。')

    mixed = [s for s in segs if s['kind'] == 'MIXED' and (s['t1'] - s['t0']) > 1.0]
    if mixed:
        print('\n! 有 %d 段「边转边走」超过 1 秒：' % len(mixed))
        for s in mixed:
            print('   %8.2fs 起  持续 %.2fs  前进 %.2fm  转 %.1f°'
                  % (s['t0'], s['t1'] - s['t0'], s['dxy'], math.degrees(s['dtheta'])))
        print('  ★ 差速车在窄缝里这么做容易蹭墙，可关注。')
    print('=' * 72)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--replay', help='回放之前录的 csv')
    ap.add_argument('--out', default=None, help='录到哪个 csv（默认 cmdvel_<时间>.csv）')
    ap.add_argument('--topic', default='/cmd_vel_nav')
    args = ap.parse_args()

    if args.replay:
        samples = []
        with open(args.replay, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                p = line.split(',')
                if len(p) < 3:
                    continue
                samples.append((float(p[0]), float(p[1]), float(p[2])))
        print('回放 %d 个采样' % len(samples))
        report(segments(samples))
        return 0

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
    except ImportError:
        print('!! 需要 ROS 2 环境：')
        print('   source /opt/ros/humble/setup.bash && /usr/bin/python3 %s' % sys.argv[0])
        return 2

    import time
    out = args.out or ('cmdvel_%s.csv' % time.strftime('%H%M%S'))

    rclpy.init()
    node = Node('cmdvel_watcher')
    samples = []
    t0 = time.time()

    def cb(msg):
        # ★ 用 time.time() 而不是 msg.header.stamp ——
        #   仿真时间可能为 0（未启用 use_sim_time 的节点），相对时间才是我们要的
        samples.append((time.time() - t0,
                        msg.linear.x, msg.angular.z))

    node.create_subscription(Twist, args.topic, cb, 50)
    print('旁听 %s ... Ctrl+C 结束并出报告' % args.topic)
    print('（同时会写入 %s）' % out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with open(out, 'w', encoding='utf-8') as f:
            f.write('# t,linear.x,angular.z\n')
            for t, v, w in samples:
                f.write('%.4f,%.5f,%.5f\n' % (t, v, w))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('\n采到 %d 帧，已写入 %s' % (len(samples), out))
    report(segments(samples))
    return 0


if __name__ == '__main__':
    sys.exit(main())
