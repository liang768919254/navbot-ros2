#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_route.py —— 把一条路线跑完，逐点记录「到没到 / 多久 / 偏多少」。

为什么不用 nav_patrol.py（2026-11-16 新写）
--------------------------------------------------------------------------
sim_fullchain/nav_patrol.py 是为 navbot_room 写死的：ROUTE 常量、8 个点、
还有一堆「缝S 净空」「南北墙带」之类只对那个房间成立的判据。
换到 navbot_house（15×10.5 m、5 个房间）之后那些判据全部失效。

这个脚本只保留**与地图无关**的三件事：

    1. 送目标、等真实 result（区分 SUCCEEDED / ABORTED / TIMEOUT / 未送达）
    2. 计时（墙钟 + 仿真时间两条都记，Gazebo RTF<1 时差得很多）
    3. 到达后查 TF 算终点误差（map→base_footprint 的平面距离）

路线写在 JSON 里（tools/verify/routes_house.json），换地图只换文件。

用法
--------------------------------------------------------------------------
    python3 tools/verify/nav_route.py --route-file tools/verify/routes_house.json
    python3 tools/verify/nav_route.py --route-file ... --timeout 180 --json out.json

判据（和 nav_patrol 一致，便于横向比较）
--------------------------------------------------------------------------
    PASS = SUCCEEDED 且 终点误差 < 0.30 m 且 用时 < timeout
    ★ 误差阈值为什么从 0.25 放宽到 0.30：房子里的目标点净空比房间小
      （有的点离墙 0.55 m），goal_checker 的 xy_goal_tolerance 0.12
      在「贴墙点」上可达域会被压小，0.25 会误判。这条写在判据里。
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time as TimeMsg


TOL_ERR = 0.30          # 终点误差上限（m）
DEFAULT_TIMEOUT = 240.0  # 单点超时（s，墙钟）


def make_goal(frame, x, y, yaw=0.0):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = TimeMsg()          # 空时间戳 = 用最新 TF
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    g = NavigateToPose.Goal()
    g.pose = ps
    return g


class RouteRunner(Node):
    def __init__(self, frame='map', base='base_footprint'):
        # ★ use_sim_time 必须在构造时给（parameter_overrides）：
        #   事后 set_parameters 会因为 value 没包成 ParameterValue 直接 assert 挂掉
        #   —— 第一版就这么崩的，log 里只有一句 AssertionError。
        #   不给的话 t_sim 恒为 0：「墙钟 180 s / 仿真 3 s」和「墙钟 180 s / 仿真 200 s」
        #   是两种完全不同的病，不能没有这个数。
        super().__init__(
            'nav_route',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.frame = frame
        self.base = base
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)

    def wait_ready(self, timeout=30.0):
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('navigate_to_pose 动作服务器不在 —— bt_navigator 没 active？')

    def sim_now(self):
        return self.get_clock().now()

    def pose_error(self, gx, gy):
        """map 系下 base 到目标的平面距离。查不到 TF 返回 None（不假装 0）。"""
        try:
            tf = self.tf.lookup_transform(self.frame, self.base, Time())
        except Exception:  # noqa: BLE001
            return None
        p = tf.transform.translation
        return math.hypot(p.x - gx, p.y - gy)

    def goto(self, gx, gy, gyaw, timeout):
        goal = make_goal(self.frame, gx, gy, gyaw)
        t0 = time.time()
        s0 = self.sim_now()
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return {'state': 'REJECTED', 't': 0.0, 't_sim': 0.0,
                    'detail': '目标未被接收（动作服务器拒绝或已消失）'}
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        if rf.result() is None:
            # 超时：主动取消，避免这条目标继续占着控制器影响下一条
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=10.0)
            dt = time.time() - t0
            return {'state': 'TIMEOUT', 't': dt, 't_sim': 0.0,
                    'detail': '超过 %.0f s 未结束，已发取消' % timeout}
        res = rf.result()
        dt = time.time() - t0
        dt_sim = (self.sim_now() - s0).nanoseconds / 1e9
        state = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.status, 'UNKNOWN')
        err = self.pose_error(gx, gy) if state == 'SUCCEEDED' else None
        return {'state': state, 't': dt, 't_sim': dt_sim,
                'err': err, 'detail': 'status=%d' % res.status}


def main():
    ap = argparse.ArgumentParser(description='按 JSON 路线逐点导航并记录指标')
    ap.add_argument('--route-file', required=True)
    ap.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT, help='单点超时（s）')
    ap.add_argument('--json', help='逐点结果写 JSON')
    ap.add_argument('--tag', default='', help='这次跑的标记（如 navfn / smac2d），写进 JSON')
    args = ap.parse_args()

    with open(args.route_file) as f:
        spec = json.load(f)
    goals = spec['goals']

    rclpy.init()
    node = RouteRunner()
    rows = []
    try:
        node.wait_ready()
        print('路线 %d 点，单点超时 %.0f s，判据 err<%.2f m' % (len(goals), args.timeout, TOL_ERR))
        print('%-14s %-10s %8s %8s %8s  %s' % ('目标', '结果', '墙钟s', '仿真s', '误差m', '判定'))
        for g in goals:
            r = node.goto(g['x'], g['y'], g.get('yaw', 0.0), args.timeout)
            ok = (r['state'] == 'SUCCEEDED'
                  and (r.get('err') is not None and r['err'] < TOL_ERR)
                  and r['t'] < args.timeout)
            r.update({'name': g.get('name', '?'), 'x': g['x'], 'y': g['y'], 'pass': bool(ok)})
            rows.append(r)
            print('%-14s %-10s %8.1f %8.1f %8s  %s'
                  % (r['name'], r['state'], r['t'], r['t_sim'],
                     ('%.3f' % r['err']) if r.get('err') is not None else '   -',
                     'PASS' if ok else 'FAIL'))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    npass = sum(1 for r in rows if r['pass'])
    print('\n合计 %d 点，PASS %d，FAIL %d' % (len(rows), npass, len(rows) - npass))
    tot = sum(r['t'] for r in rows)
    print('总墙钟 %.1f s，总仿真 %.1f s' % (tot, sum(r['t_sim'] for r in rows)))
    errs = [r['err'] for r in rows if r.get('err') is not None]
    if errs:
        print('终点误差 最大 %.3f / 平均 %.3f m' % (max(errs), sum(errs) / len(errs)))

    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'tag': args.tag, 'route': args.route_file,
                       'tol_err': TOL_ERR, 'rows': rows,
                       'pass': npass, 'total': len(rows)}, f,
                      ensure_ascii=False, indent=1)
        print('结果已写入 %s' % args.json)


if __name__ == '__main__':
    main()
