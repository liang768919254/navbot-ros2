#!/usr/bin/env python3
"""检查局部/全局代价地图在机器人周围的占据情况（诊断「幽灵障碍」用）。

用法：
  python3 check_costmap.py            # 同时看 local 与 global
  python3 check_costmap.py local      # 只看局部
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class Checker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.data = {}
        for name in ('local', 'global'):
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(
                OccupancyGrid, f'/{name}_costmap/costmap',
                lambda msg, n=name: self.data.setdefault(n, msg), qos)

    def robot_xy(self, frame):
        try:
            t = self.buf.lookup_transform(frame, 'base_footprint',
                                          rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return (None, None)

    def report(self, name, msg, rx, ry):
        info = msg.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        w, h = info.width, info.height
        grid = msg.data
        lethal = []
        near = {0.3: 0, 0.5: 0, 0.8: 0}
        for j in range(h):
            wy = oy + (j + 0.5) * res
            for i in range(w):
                v = grid[j * w + i]
                if v < 50:      # 只关心 >=50（膨胀内/致命）
                    continue
                wx = ox + (i + 0.5) * res
                d = math.hypot(wx - rx, wy - ry)
                for r in near:
                    if d <= r:
                        near[r] += 1
                if v >= 99 and len(lethal) < 8:
                    lethal.append((round(wx, 2), round(wy, 2), int(v)))
        print(f'[{name}] {w}x{h} res={res:.3f} origin=({ox:.2f},{oy:.2f})')
        print(f'  机器人 ({rx:.2f},{ry:.2f}) 周围占据格数: '
              f'0.3m={near[0.3]} 0.5m={near[0.5]} 0.8m={near[0.8]}')
        if lethal:
            print(f'  致命格样例: {lethal}')


def main():
    rclpy.init()
    node = Checker()
    want = sys.argv[1] if len(sys.argv) > 1 else 'both'
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(node.data) >= (1 if want != 'both' else 2):
            break
    rx, ry = node.robot_xy('map')
    print(f'机器人(map): ({rx},{ry})')
    for name, msg in node.data.items():
        if want != 'both' and want != name:
            continue
        f = msg.header.frame_id
        fx, fy = node.robot_xy(f)
        if fx is None:
            print(f'[{name}] 无法取得 {f} 系下机器人位姿，跳过')
            continue
        print(f'机器人({f}): ({fx:.2f},{fy:.2f})')
        node.report(name, msg, fx, fy)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
