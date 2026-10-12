#!/usr/bin/env python3
"""自动走位 + 全向扫描：把 teleop 手工绕圈的活自动化。

★ 这是「辅助脚本」，不是本项目要学的核心。
  练习阶段请手动 ros2 run teleop_twist_keyboard 走一圈，
  熟练之后想无人值守复现时再用它。

策略：按 waypoint 列表逐点走，每到一点停下来原地慢速转一圈，
让 lidar 把周围扫全 —— 比边走边转的 SLAM 质量高。
每段的控制器是「先原地转到目标方向 → 再直行」，比例控制 + 到位判定。

用法：
  python3 .check/auto_scan.py            # 跑完整条路径
  python3 .check/auto_scan.py --dry-run  # 只打印路径，不动车
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(x, y, z, w):
    """四元数 → 偏航角（弧度）。只关心绕 z 轴转多少。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def norm_angle(a):
    """把角度归一到 [-pi, pi]，避免转 350° 时绕远路。"""
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# 房间 6m x 5m，墙在 x=±3.0 / y=±2.5。内部障碍：
#   partition_mid   (-0.5, 0.5)  水平 2.6m  → x∈[-1.8,0.8]
#   partition_short ( 1.2,-0.9)  竖直 1.8m  → y∈[-1.8,0.0]
#   pillar_a ( 1.8, 1.5)   pillar_b (-1.9,-1.6)   box (-1.5,1.3)
# 下面这条路线刻意绕开全部障碍，并且能扫到四面外墙 + 大部分内部结构。
# ---------------------------------------------------------------------------
WAYPOINTS = [
    ( 0.00,  0.00),
    (-1.20,  0.00),
    (-2.00, -1.00),
    (-1.00, -2.10),
    ( 0.80, -2.10),
    ( 2.30, -2.10),
    ( 2.30, -0.50),
    ( 2.30,  1.20),
    ( 1.00,  1.00),
]

LIN_SPEED = 0.13      # m/s  建图要慢，快了会糊
ANG_SPEED = 0.45      # rad/s
SPIN_TIME = 2 * math.pi / ANG_SPEED + 1.0   # 原地扫一圈所需的秒数
GOAL_TOL = 0.12       # m    到位判定
ANG_TOL = 0.06        # rad


class Navigator:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(Twist, '/cmd_vel', 10)
        node.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.x = self.y = self.yaw = 0.0
        self.got_odom = False

    def cb_odom(self, msg):
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        q = p.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.got_odom = True

    def _sleep(self, sec):
        """用 node 的时钟推进，保证 /clock 仿真时间下也能正常计时。"""
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def send(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def wait_odom(self, timeout=30.0):
        end = time.time() + timeout
        while not self.got_odom and time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.got_odom

    def rotate_to(self, target_yaw):
        """原地转到目标朝向。"""
        while rclpy.ok():
            err = norm_angle(target_yaw - self.yaw)
            if abs(err) < ANG_TOL:
                break
            w = max(-ANG_SPEED, min(ANG_SPEED, 1.6 * err))
            w = math.copysign(max(abs(w), 0.10), w)   # 避免死区卡住
            self.send(0.0, w)
            self._sleep(0.05)
        self.send(0.0, 0.0)
        self._sleep(0.4)

    def drive_to(self, gx, gy, timeout=40.0):
        """先转向再直行到目标点。返回 True 表示成功到位。"""
        self.rotate_to(math.atan2(gy - self.y, gx - self.x))
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist < GOAL_TOL:
                break
            # 走的过程中持续修正朝向（P 控制）
            err = norm_angle(math.atan2(dy, dx) - self.yaw)
            self.send(min(LIN_SPEED, 1.2 * dist),
                      max(-0.25, min(0.25, 1.5 * err)))
            self._sleep(0.05)
        self.send(0.0, 0.0)
        self._sleep(0.5)
        return math.hypot(gx - self.x, gy - self.y) < GOAL_TOL + 0.05

    def spin_scan(self):
        """原地慢速转一圈，让激光把四周扫全。"""
        self.send(0.0, ANG_SPEED)
        self._sleep(SPIN_TIME)
        self.send(0.0, 0.0)
        self._sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true', help='只打印路径不动车')
    args = ap.parse_args()

    if args.dry_run:
        print('规划路径（不包含起点 wait）：')
        prev = WAYPOINTS[0]
        for i, wp in enumerate(WAYPOINTS, 1):
            d = math.hypot(wp[0] - prev[0], wp[1] - prev[1])
            print(f'  {i}. ({wp[0]:5.2f},{wp[1]:5.2f})  段长 {d:.2f} m')
            prev = wp
        total = sum(math.hypot(WAYPOINTS[i][0] - WAYPOINTS[i-1][0],
                               WAYPOINTS[i][1] - WAYPOINTS[i-1][1])
                    for i in range(1, len(WAYPOINTS)))
        print(f'  总里程 ≈ {total:.1f} m，{len(WAYPOINTS)} 次全向扫描，'
              f'预计 {total/LIN_SPEED + len(WAYPOINTS)*SPIN_TIME:.0f} 秒')
        return 0

    rclpy.init()
    node = rclpy.create_node('auto_scan')
    nav = Navigator(node)

    if not nav.wait_odom():
        print('[FAIL] 等不到 /odom，Gazebo 起了吗？', file=sys.stderr)
        rclpy.shutdown()
        return 1
    print(f'[起点] x={nav.x:.2f} y={nav.y:.2f} yaw={math.degrees(nav.yaw):.1f}°')

    nav.spin_scan()          # 起点先扫一圈
    print('  [1/9] 起点扫描完成')

    ok = 0
    for idx, (gx, gy) in enumerate(WAYPOINTS[1:], 2):
        reached = nav.drive_to(gx, gy)
        errx, erry = gx - nav.x, gy - nav.y
        print(f'  [{idx}/{len(WAYPOINTS)}] 目标({gx:5.2f},{gy:5.2f}) '
              f'实际({nav.x:5.2f},{nav.y:5.2f}) 残差{math.hypot(errx,erry):.2f}m '
              f'{"OK" if reached else "未到位"}')
        if reached:
            ok += 1
        nav.spin_scan()
        print(f'         扫描完成')

    nav.send(0.0, 0.0)
    print(f'[完成] {ok}/{len(WAYPOINTS)-1} 个航点到位')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok >= len(WAYPOINTS) - 3 else 1


if __name__ == '__main__':
    sys.exit(main())
