#!/usr/bin/env python3
"""
check_controller.py —— 校验 controller_server 的「控制器链」配置是否自洽

为什么需要它：
  2026-11-10 把 DWB 改成「RotationShim 外层 + DWB 内层」之后，出现了一类
  **yaml 语法完全合法、但运行起来必然出错** 的配置方式。ROS 的参数声明
  只校验「类型对不对」，不校验「它们之间的关系」，所以只能自己写。

  已经踩过的坑（按发生顺序）：

  ① primary_controller 写成**参数块名**（"FollowPathDWB"）
     → 源码里是 lp_loader_.createUniqueInstance(primary_controller)，
       这是 pluginlib 的**类名**查找，填参数块名必然找不到：
       [FATAL] the class FollowPathDWB ... does not exist
     → 更糟的是 configure() 里 catch 到异常后**只 return 不抛**，
       server 继续走到 activate，随后 SIGSEGV（exit code -11），
       NavigateToPose 动作服务端根本没起来，客户端 0.8s 收到 NOT_ACCEPTED。
     ★ 这与我第一版检查脚本的假设完全相反 —— 我当时以为要填参数块名，
       还专门加了「内层块必须存在且不能为空」的检查，等于把错的固化成对的。

  ② 内层参数块写了但**永远读不到**
     源码：primary_controller_->configure(parent, name, ...)
        name 是 shim 自己的 plugin_name（= "FollowPath"）
     → DWB 的所有参数都从 "FollowPath.*" 读，
       另开一个 FollowPathDWB: 块是无用功，DWB 会用默认值
       （max_vel_x 0.26、min_vel_y 等）跑飞。

  ③ rotate_to_heading_when_stopped 漏了 / false
     → shim 变成「边转边走」，车前方 0.17m 是墙时直接撞。

  ④ progress_checker 没跟着放宽
     → SimpleProgressChecker 只看位移，原地转位移恒 0，
       刚转起来就被判「没进展」，看现象和 S27 的卡死一模一样。

用法：
  python3 tools/verify/check_controller.py
  python3 tools/verify/check_controller.py --yaml /path/to/nav2_params.yaml
  python3 tools/verify/check_controller.py --plugins-dir /opt/ros/humble/lib
"""
import argparse
import glob
import os
import re
import subprocess
import sys

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'src', 'navbot_bringup', 'config', 'nav2_params.yaml')

# ★★ 这个下限是算出来的，不是拍的。
#   转 70°（本项目最坏情况：车头朝东、目标在正东但被短板挡住）：
#     0.8 rad/s + 2.0 rad/s² 的梯形速度曲线 → 加速 0.40 + 匀速 1.13 + 减速 0.40 = 1.93s
#   加上内层 DWB 起步（1~2s）+ planner 降频抖动（实测有 1.67Hz 的时段，约 2~3s）
#     ⇒ 一次「转 + 起步」保守 5.9s
#   若这一次没成功、触发一次恢复动作（spin 6s / backup 7s）
#     ⇒ 需要 12.9s
#   取 15s 作为下限：容得下「转 + 起步 + 一次恢复」，而真卡死（顶墙不动）仍能被抓到。
#
# ★ 我第一版这里写的 8.0，故障注入测试立刻暴露出它太松 ——
#   把 allowance 从 25 改回 10，检查仍然全 PASS，等于这个判据是摆设。
MIN_ALLOWANCE_FOR_ROTATE = 15.0

# ★★ DWB 必须存在的参数。不是「全部参数」，而是「漏了会静默跑飞」的那几个。
#   判断依据：这些项在 DWBLocalPlanner 里没有安全的默认值，或者默认值是
#   按 TurtleBot3 给的，与本车（车长 0.3m、max_vel 0.22）严重不匹配。
REQUIRED_DWB_KEYS = [
    ('max_vel_x', '限速；默认 0.26 是按 TurtleBot3 给的，本车 0.22'),
    ('max_vel_theta', '限角速度；留空会允许打转'),
    ('min_vel_x', '决定能否倒车'),
    ('acc_lim_x', '起步加速度；不设会顿挫'),
    ('critics', '不设等于没配 critic，DWB 无评分'),
    ('vx_samples', '采样数；不设走默认 20，配 critics 后必须复核'),
]

PASS = 0
FAIL = 0
WARN = 0


def ok(msg):
    global PASS
    PASS += 1
    print('  [PASS] %s' % msg)


def ng(msg):
    global FAIL
    FAIL += 1
    print('  [FAIL] %s' % msg)


def warn(msg):
    global WARN
    WARN += 1
    print('  [WARN] %s' % msg)


def find_plugin_classes(plugins_dir):
    """
    从 .so 里提取 pluginlib 导出的类名，返回 set。
    用途：确认 primary_controller 写的是**真实存在的类名**，
    而不是「yaml 里恰好有同名参数块」。
    ★ 这一步是本脚本的核心 —— 上一版就是靠「cs 里有没有这个 key」判断，
      而 key 是我自己瞎起的，当然有，于是错的配置被放行。
    """
    found = set()
    libs = glob.glob(os.path.join(plugins_dir, 'lib*.so'))
    for lib in libs:
        try:
            out = subprocess.run(
                ['strings', lib], capture_output=True, text=True, timeout=20).stdout
        except Exception:
            continue
        # pluginlib 导出时会把完整类名以字符串形式留在 .rodata
        for m in re.finditer(r'\b((?:nav2|dwb)_[a-z0-9_]*::[A-Za-z0-9_]+)\b', out):
            found.add(m.group(1))
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--plugins-dir', default='/opt/ros/humble/lib',
                    help='到哪找 .so 来枚举 pluginlib 类名')
    ap.add_argument('--no-plugin-scan', action='store_true',
                    help='跳过 .so 扫描（无 ROS 环境时用，但会弱化 ② 的判据）')
    args = ap.parse_args()

    if not os.path.isfile(args.yaml):
        print('!! 找不到 yaml: %s' % args.yaml)
        return 2

    try:
        import yaml
    except ImportError:
        print('!! 需要 PyYAML：python3 -m pip install pyyaml')
        return 2

    with open(args.yaml, encoding='utf-8') as f:
        doc = yaml.safe_load(f)

    cs = (doc.get('controller_server') or {}).get('ros__parameters') or {}
    if not cs:
        print('!! yaml 里找不到 controller_server.ros__parameters')
        return 2

    print('=' * 74)
    print(' controller_server 控制器链检查')
    print(' yaml: %s' % args.yaml)
    print('=' * 74)

    plugins = cs.get('controller_plugins') or []
    print('\n---- ① 入口插件 ----')
    if not plugins:
        ng('controller_plugins 为空')
        return 1
    names = list(plugins)
    ok('controller_plugins = %s' % names)
    if len(names) != 1:
        ng('本项目的 BT 只用一个控制器，配了 %d 个会让行为树选错' % len(names))

    entry = names[0]
    entry_cfg = cs.get(entry) or {}
    if not entry_cfg:
        ng('入口插件 %r 没有对应的参数块' % entry)
        return 1
    entry_plugin = entry_cfg.get('plugin', '')
    ok('%s.plugin = %s' % (entry, entry_plugin))

    # 顺便收集 yaml 里所有「看起来像第二个控制器」的块，稍后用来抓坑②
    is_shim = 'rotation_shim' in entry_plugin.lower()
    if not is_shim:
        print('\n---- ② 入口不是 shim ----')
        print('  （跳过内层检查）')
        print('  ★ 提醒：纯 DWB 在「必须原地大角度转向」时会卡死，')
        print('    见 docs/S28_原地转向时DWB瞎了.md')
        _summary()
        return 0 if FAIL == 0 else 1

    print('\n---- ② primary_controller 是不是真实存在的插件类名 ----')
    inner_cls = entry_cfg.get('primary_controller')
    if not inner_cls:
        ng('shim 缺 primary_controller —— 没有内层控制器，shim 无处转交')
    else:
        # 2.1 语法形态：必须是 "包名::类名"
        if '::' not in str(inner_cls):
            ng('primary_controller = %r 不像 pluginlib 类名（应为 "包::类"）' % inner_cls)
            ng('  ★ 源码：lp_loader_.createUniqueInstance(primary_controller)，'
               '这是按【类名】加载，不是按参数块名')
            ng('  ★ 上一版我误以为要填参数块名 "FollowPathDWB"，'
               '结果 FATAL + controller_server SIGSEGV(exit -11)')
            ng('  ★ 正确写法："dwb_core::DWBLocalPlanner"')
        else:
            ok('primary_controller = %r（形态像类名）' % inner_cls)

        # 2.2 名字必须和入口插件不同，否则是自我递归
        if inner_cls == entry_plugin:
            ng('primary_controller 与入口 plugin 相同 —— 无限递归')

        # 2.3 ★ 核心：到 .so 里查这个类真的存在
        if args.no_plugin_scan or not os.path.isdir(args.plugins_dir):
            warn('跳过 .so 扫描（%s 不存在或用 --no-plugin-scan），'
                 '无法确认类名真实存在' % args.plugins_dir)
        else:
            classes = find_plugin_classes(args.plugins_dir)
            if inner_cls in classes:
                ok('类名 %s 在 %s 的 .so 中确实存在'
                   % (inner_cls, args.plugins_dir))
            else:
                # 只报近似项，帮定位
                near = [c for c in classes if 'dwb' in c.lower()][:5]
                ng('类名 %r 在任何 .so 中都找不到 —— '
                   'controller_server 会 FATAL 后 SIGSEGV' % inner_cls)
                if near:
                    ng('  可用的 DWB 类名：%s' % ', '.join(sorted(near)))
                ng('  ★ 这一项是上一版检查漏掉的：只查「yaml 里有没有同名块」，'
                   '而块名是自己起的，永远查得到')

        if 'dwb' not in str(inner_cls).lower():
            ng('内层应为 DWB，实际是 %r' % inner_cls)

    print('\n---- ③ 内层参数落在哪个参数块 ----')
    # 源码：primary_controller_->configure(parent, name, tf, costmap_ros)
    #       name 就是 shim 的 plugin_name（= 入口块名 entry）
    # ⇒ DWB 读的参数前缀是 "<entry>." 而不是别的
    print('  源码 primary_controller_->configure(parent, name, ...) 里')
    print('  name = shim 自己的 plugin_name = %r' % entry)
    print('  ⇒ 内层 DWB 只读 %r 下的参数' % (entry + '.*'))

    # 抓坑②：yaml 里有没有一个「被写出来但没人读」的块
    #
    # ★ 排除那些「是独立子插件、有自己引用入口」的块 —— 它们不归 shim 管：
    #     progress_checker     ← controller_server.progress_checker_plugin 引用
    #     <goal checker 任意名> ← controller_server.goal_checker_plugins 列表引用
    #   我第一版没排它们，误报了 4 条 FAIL，等于把正常配置当错的。
    referenced = set()
    pcp = cs.get('progress_checker_plugin')
    if isinstance(pcp, str):
        referenced.add(pcp)
    for g in (cs.get('goal_checker_plugins') or []):
        referenced.add(g)

    suspicious = []
    for key, val in cs.items():
        if not isinstance(val, dict):
            continue
        if key == entry or key in referenced:
            continue
        p = val.get('plugin', '')
        if isinstance(p, str) and ('dwb' in p.lower() or 'rotation_shim' in p.lower()):
            suspicious.append(key)
    if suspicious:
        for s in suspicious:
            ng('存在无人读取的控制器参数块 %r（plugin=%r）—— '
               'shim 的内层只从 %r 读参数，这块等于没写'
               % (s, cs[s].get('plugin'), entry))
            ng('  ★ 现象：速度/采样用默认值跑飞，而 yaml 看起来「配了」')
    else:
        ok('没有游离的控制器参数块（内层参数都在 %r 下）' % entry)

    # 内层参数是否真在 entry 块里
    missing = [(k, why) for k, why in REQUIRED_DWB_KEYS if k not in entry_cfg]
    if missing:
        for k, why in missing:
            ng('%s.%s 缺失 —— %s' % (entry, k, why))
    else:
        ok('内层 DWB 的 %d 个关键参数都在 %r 下'
           % (len(REQUIRED_DWB_KEYS), entry))

    print('\n---- ④ shim 关键参数 ----')
    th = entry_cfg.get('angular_dist_threshold')
    if th is None:
        ng('缺 angular_dist_threshold —— 用默认 0.785(45°)，'
           '角度差小于 45° 时 shim 不介入')
    elif not (0.05 <= float(th) <= 1.2):
        ng('angular_dist_threshold = %.2f rad (%.0f°) 超出合理区间 '
           '[0.05, 1.2]' % (float(th), float(th) * 57.3))
    else:
        ok('angular_dist_threshold = %.2f rad (%.0f°)'
           % (float(th), float(th) * 57.3))

    w = entry_cfg.get('rotate_to_heading_angular_vel')
    if w is None:
        ng('缺 rotate_to_heading_angular_vel —— 原地转的角速度')
    else:
        ok('rotate_to_heading_angular_vel = %.2f rad/s '
           '（转 70° 约 %.1fs）' % (float(w), 1.222 / float(w)))

    # ★★ rotate_to_heading_when_stopped 在 Humble 的 .so 里根本不存在！
    #   源码 configure() 里声明的只有 angular_dist_threshold / angular_disengage_threshold
    #   / forward_sampling_distance / rotate_to_heading_angular_vel / max_angular_accel
    #   / simulate_ahead_time / primary_controller / rotate_to_goal_heading / closed_loop
    #   没有 when_stopped ⇒ 写了也是无效参数（被忽略）。
    #   这个类里「是否原地转」由 computeRotateToHeadingCommand 的 cmd_vel.twist.linear 恒为 0
    #   保证，本质就是原地转。所以这里不再检查它，改为检查更关键的 simulate_ahead_time。
    if 'rotate_to_heading_when_stopped' in entry_cfg:
        warn('rotate_to_heading_when_stopped 不是 Humble shim 的参数'
             '（源码未声明），写了会被忽略。可删')

    sat = entry_cfg.get('simulate_ahead_time')
    if sat is None:
        print('  （simulate_ahead_time 未设，默认 1.0s）')
    elif float(sat) <= 0:
        ng('simulate_ahead_time = %.2f —— 碰撞预演被关掉，'
           '窄处转身不做 footprint 检查就会撞' % float(sat))
    else:
        ok('simulate_ahead_time = %.2f s（转身前预演碰撞）' % float(sat))

    fsd = entry_cfg.get('forward_sampling_distance')
    if fsd is not None and float(fsd) < 0.2:
        warn('forward_sampling_distance = %.2f 偏小 —— '
             '采样点贴车头，角度差被放大，会频繁误触发原地转' % float(fsd))

    print('\n---- ⑤ progress_checker 是否为原地转留了余量 ----')
    pc = cs.get('progress_checker') or {}
    allow = pc.get('movement_time_allowance')
    rad = pc.get('required_movement_radius')
    if rad is None or allow is None:
        ng('progress_checker 缺 required_movement_radius 或 '
           'movement_time_allowance')
    else:
        ok('required_movement_radius = %.2f, '
           'movement_time_allowance = %.1fs' % (float(rad), float(allow)))
        if float(allow) < MIN_ALLOWANCE_FOR_ROTATE:
            ng('allowance %.1fs < %.1fs：shim 原地转时位移恒为 0，'
               '会「一转向就被判没进展」。'
               '★ 这是 RotationShim 的必备配套，不是可选项'
               % (float(allow), MIN_ALLOWANCE_FOR_ROTATE))
        else:
            ok('allowance %.1fs >= %.1fs，容得下一次原地转 + 起步'
               % (float(allow), MIN_ALLOWANCE_FOR_ROTATE))

    print('\n---- ⑥ 内层 DWB 的 min_vel_x（决定能否倒车） ----')
    mvv = entry_cfg.get('min_vel_x')
    if mvv is None:
        warn('min_vel_x 未设，走默认 0.0（不倒车）')
    elif float(mvv) < 0:
        ok('min_vel_x = %.2f < 0，允许倒车（配合 backup 更灵活）' % float(mvv))
    else:
        print('  min_vel_x = %.2f，不倒车。'
              '★ 本设计依赖 shim 原地转 + backup 恢复，可以不倒车' % float(mvv))

    print('\n---- ⑦ BaseObstacle 的量级（决定「敢不敢贴墙」） ----')
    # ★ 源码事实（base_obstacle.cpp）：
    #     scorePose 里 253/254/255 会 throw IllegalTrajectoryException（轨迹淘汰），
    #     真正参与打分的只有 0..252 的软代价，且**不归一化**。
    #   ⇒ 罚分 = 末端点 cost × scale，最大 252 × scale。
    #   而 PathAlign scale=32、阈值 forward_point_distance：
    #     偏离 forward_point_distance 就到满额 32 分。
    #   「贴墙走」的净收益 ≈ 省 (cost×scale) - 花 (32 × 偏离/阈值)。
    #   scale 太小（官方 0.02）时净收益接近 0，DWB 就贴着膨胀边界以极低速爬。
    bo = entry_cfg.get('BaseObstacle.scale')
    fpd = entry_cfg.get('PathAlign.forward_point_distance')
    pa = entry_cfg.get('PathAlign.scale')
    if bo is None:
        ng('缺 BaseObstacle.scale —— 默认 0.02，对 0.05 分辨率的图几乎无感，'
           '会「贴着障碍爬行」（S31 实测 3.7 cm/s）')
    else:
        bo = float(bo)
        # ★ 2026-11-10 夜（S32）：inflation 0.22→0.16 之后「贴墙时 cost 是多少」
        #   这个数变了。0.16 < 圆半径 0.20 时，膨胀层只剩两档：253 与 0，
        #   车心贴到 0.16~0.20 这一圈会直接落进 253（淘汰），而不是 240 分软罚。
        #   ⇒ 真正的「贴墙」发生在车心距障碍 0.16 附近且刚好逃过 253 的轨迹，
        #     其末端 cost 分布没有单一点值可算，这里取「刚脱离 253 的软代价上限」
        #     作为保守估计：cost 峰值 252，取 180（≈ 0.16 外侧两格）。
        #   ★ 结论不变：scale 越大越愿意离墙。判据退化为「净收益是否为正且够大」。
        WALL_COST = 180
        gain = WALL_COST * bo
        cost_dev = 32.0 * (0.08 / float(fpd)) if (fpd and pa) else 10.2
        net = gain - cost_dev
        if bo < 0.15:
            ng('BaseObstacle.scale = %.2f 偏小：贴墙罚 %.0f 分，'
               '而离开 10cm 花 %.0f 分 → 净赚 %.0f 分，太小，'
               '车会贴着膨胀边界慢速爬行' % (bo, gain, cost_dev, net))
        elif net < 50:
            warn('BaseObstacle.scale = %.2f：净收益 %.0f 分，可能仍偏保守'
                 % (bo, net))
        else:
            ok('BaseObstacle.scale = %.2f：贴墙罚 %.0f 分 vs 偏 8cm 花 %.0f 分 '
               '→ 净赚 %.0f 分，愿意离开障碍' % (bo, gain, cost_dev, net))

    ss = entry_cfg.get('BaseObstacle.sum_scores')
    if ss is True:
        warn('BaseObstacle.sum_scores = true：会累加整条轨迹代价，'
             '量级变 ~(sim_time×freq) 倍，scale 必须重算；'
             '窄缝场景更容易误杀安全轨迹')
    else:
        print('  sum_scores = %r（只看轨迹末端点，与本文档的账一致）' % ss)

    _summary()
    return 0 if FAIL == 0 else 1


def _summary():
    print('\n' + '=' * 74)
    print(' 合计: PASS=%d  FAIL=%d  WARN=%d' % (PASS, FAIL, WARN))
    print(' 结论: %s' % ('全通' if FAIL == 0 else '有 %d 项没过' % FAIL))
    print('=' * 74)


if __name__ == '__main__':
    sys.exit(main())
