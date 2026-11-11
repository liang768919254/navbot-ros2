#!/usr/bin/env python3
"""
inject_faults.py —— 对 check_controller.py 做故障注入自测

为什么需要：
  S28 的教训是「判据没被验证过，就等于没有判据」。
  但更狠的一次是 S29：check_controller.py 全绿，仿真却起不来 ——
  因为**判据本身写错了方向**（把 primary_controller 当成参数块名去查）。
  所以光有检查不够，必须证明「把真实错误注进去，它真的会红」。

做法：
  复制一份 yaml → 注入一种错误 → 跑 check_controller.py → 断言退出码非 0
  且输出里出现预期的关键词。全部注入完成后删临时文件。

用法：
  python3 tools/verify/inject_faults.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CHECKER = os.path.join(HERE, 'check_controller.py')
CHECK_LIMITS = os.path.join(HERE, 'check_limits.py')
SRC_YAML = os.path.join(
    os.path.dirname(os.path.dirname(HERE)),
    'src', 'navbot_bringup', 'config', 'nav2_params.yaml')

# (说明, 期望输出关键词, [(old, new), ...])
FAULTS = [
    (
        'primary_controller 写成参数块名（S29 真实事故）',
        '不像 pluginlib 类名',
        [('primary_controller: "dwb_core::DWBLocalPlanner"',
          'primary_controller: "FollowPathDWB"')],
    ),
    (
        'primary_controller 写成不存在的类名',
        '找不到',
        [('primary_controller: "dwb_core::DWBLocalPlanner"',
          'primary_controller: "dwb_core::NoSuchPlanner"')],
    ),
    (
        '内层参数另开一个游离块（S29 第二个事故）',
        '无人读取的控制器参数块',
        [],   # 特殊处理：见下
    ),
    (
        'movement_time_allowance 退回 10s',
        'allowance',
        [('movement_time_allowance: 25.0', 'movement_time_allowance: 10.0')],
    ),
    (
        'angular_dist_threshold 设成不启用',
        '超出合理区间',
        [('angular_dist_threshold: 0.35', 'angular_dist_threshold: 1.5')],
    ),
    (
        'simulate_ahead_time 关闭',
        '碰撞预演被关掉',
        [('simulate_ahead_time: 1.0', 'simulate_ahead_time: 0.0')],
    ),
    (
        'BaseObstacle.scale 退回官方默认 0.02（S31 爬行事故）',
        '偏小',
        [('BaseObstacle.scale: 0.5', 'BaseObstacle.scale: 0.02')],
    ),
    (
        'robot_radius 顶替 footprint（S32「从下面过不去」事故）',
        'robot_radius',
        [
            ('      footprint: "[[0.15, 0.12], [0.15, -0.12], [-0.15, -0.12], [-0.15, 0.12]]"\n'
             '      resolution: 0.05',
             '      robot_radius: 0.20\n      resolution: 0.05'),
            ('      footprint: "[[0.15, 0.12], [0.15, -0.12], [-0.15, -0.12], [-0.15, 0.12]]"\n',
             '      robot_radius: 0.20\n'),
            ('        inflation_radius: 0.16\n',
             '        inflation_radius: 0.22\n'),
        ],
    ),
    (
        '膨胀半径过大，把缝S 重新封死（0.16 → 0.30）',
        '通道会被封死',
        [('inflation_radius: 0.16', 'inflation_radius: 0.30')],
    ),
]


def run(yaml_path):
    """★ 2026-11-10 夜（S32）：两个 checker 都要跑。

    原来只跑 check_controller.py，于是「把车体当圆」这类
    costmap / 车体模型类故障根本不在它的管辖范围，
    注入进去也抓不到（会显示「漏掉」而不是「该检查却没检查」）。
    现在两次都跑，任意一次红就算抓到了。
    """
    outs = []
    rc = 0
    for chk in (CHECK_LIMITS, CHECKER):
        r = subprocess.run([sys.executable, chk, '--yaml', yaml_path],
                           capture_output=True, text=True)
        outs.append(r.stdout)
        if r.returncode != 0:
            rc = r.returncode
    return subprocess.CompletedProcess(args=[], returncode=rc,
                                       stdout='\n'.join(outs), stderr='')


def main():
    try:
        import yaml  # noqa: F401
    except ImportError:
        print('!! 需要 PyYAML')
        return 2

    if not os.path.isfile(SRC_YAML):
        print('!! 找不到源 yaml: %s' % SRC_YAML)
        return 2

    base = open(SRC_YAML, encoding='utf-8').read()
    tmpdir = tempfile.mkdtemp(prefix='ctl_fault_')
    passed = failed = 0

    # 先用原配置跑一遍，必须全通 —— 否则注入结果不可信
    good = os.path.join(tmpdir, 'good.yaml')
    open(good, 'w', encoding='utf-8').write(base)
    r = run(good)
    if r.returncode != 0:
        print('[前置失败] 原始配置本身就不过，注入测试无意义：')
        print(r.stdout[-1500:])
        return 1
    print('[前置] 原始配置 全通 ✓\n')

    for idx, (desc, kw, subs) in enumerate(FAULTS, 1):
        text = base
        if desc.startswith('内层参数另开'):
            # 把 FollowPath 里的 DWB 速度参数整段抠出来，另起 FollowPathDWB: 块
            import re
            m = re.search(r'\n(      min_vel_x:.*?)(?=\n\n|\Z)', text, re.S)
            if not m:
                print('  [跳过] %s —— 找不到 min_vel_x 段' % desc)
                continue
            seg = m.group(1)
            text = text.replace(seg, '')
            text = text.replace(
                '    FollowPath:\n',
                '    FollowPathDWB:\n      plugin: "dwb_core::DWBLocalPlanner"\n'
                + seg.replace('      ', '      ', 1) + '\n    FollowPath:\n', 1)
        else:
            for old, new in subs:
                if old not in text:
                    print('  [跳过] %s —— yaml 里找不到 %r' % (desc, old[:40]))
                    text = None
                    break
                text = text.replace(old, new, 1)
            if text is None:
                continue

        p = os.path.join(tmpdir, 'fault_%02d.yaml' % idx)
        open(p, 'w', encoding='utf-8').write(text)
        r = run(p)
        hit = (r.returncode != 0) and (kw in r.stdout)
        if hit:
            print('  [抓到] %s' % desc)
            passed += 1
        else:
            print('  [漏掉] %s' % desc)
            print('         退出码=%d  期望关键词=%r 出现=%s'
                  % (r.returncode, kw, kw in r.stdout))
            print('         ---- 输出尾部 ----')
            for line in r.stdout.strip().split('\n')[-6:]:
                print('         ' + line)
            failed += 1

    shutil.rmtree(tmpdir, ignore_errors=True)
    print('\n' + '=' * 66)
    print(' 故障注入: 抓到 %d / 漏掉 %d' % (passed, failed))
    print(' 结论: %s' % ('全部抓到' if failed == 0 else '有 %d 种错误检查不出来' % failed))
    print('=' * 66)
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
