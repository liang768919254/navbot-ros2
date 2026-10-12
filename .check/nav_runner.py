#!/usr/bin/env python3
"""Nav2 导航自动验收：给初始位姿 → 等收敛 → 连发多个目标点 → 判定是否到达。

★ 用法（须在 Gazebo + Nav2 已起来的同一终端环境里跑）：
    python3 .check/nav_runner.py                 # 跑默认 5 个目标点
    python3 .check/nav_runner.py 2.0 1.5         # 只去一个点
"""

import math
import sys

import rclpy
from action_msgs.srv import CancelGoal  # noqa: F401  (仅为引用一致)
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration


# 目标点集：按新地图（origin -3,-2.52）上的可走区挑的分散 5 点，
# 覆盖东/西南/东南/西四个象限 + 一个短程点。中途会自然穿过隔墙之间的门洞。
GOALS = [
    ( 0.00,  0.00, '起点附近（短程热身）'),
    ( 2.30,  1.20, '东北角（pillar_a 旁）'),
    (-2.30, -2.00, '西南角（柱子 b 旁）'),
    ( 2.30, -2.00, '东南角（穿 partition_short 门洞）'),
    (-2.30,  0.00, '西侧（partition_mid 北面）'),
]


def quat_yaw(yaw):
    """偏航角 → 四元数（平面运动只用 z/w）。"""
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def main():
    xs = sys.argv[1:]
    goals = []
    if len(xs) >= 2:
        for i in range(0, len(xs) - 1, 2):
            goals.append((float(xs[i]), float(xs[i + 1]), '命令行指定'))
    else:
        goals = GOALS

    rclpy.init()
    node = rclpy.create_node('nav_runner')

    # ---------------- ① 给 AMCL 初始位姿（等价于 RViz 的 2D Pose Estimate）--------
    # 车是 spawn 在 (0,0,朝+x) 的，所以初值就是这个。
    ini = PoseWithCovarianceStamped()
    ini.header.frame_id = 'map'
    ini.header.stamp = node.get_clock().now().to_msg()
    ini.pose.pose.orientation.w = 1.0
    # 给一点协方差让 AMCL 知道初值不那么准，粒子撒开一点更容易收敛
    ini.pose.covariance[6 * 0 + 0] = 0.25
    ini.pose.covariance[6 * 1 + 1] = 0.25
    ini.pose.covariance[6 * 5 + 5] = 0.068
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    print('[1] 发布初始位姿 (0, 0, 朝+x) 到 /initialpose ...')
    for _ in range(5):
        pub.publish(ini)
        rclpy.spin_once(node, timeout_sec=0.2)

    # 等 AMCL 重新散布粒子
    end = node.get_clock().now() + Duration(seconds=6)
    while node.get_clock().now() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    print('    粒子已散布')

    # ---------------- ② 等 action server ----------------
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    print('[2] 等待 /navigate_to_pose action server ...')
    if not client.wait_for_server(timeout_sec=60.0):
        print('    [FAIL] action server 起不来 —— Nav2 生命周期没全 active？')
        print('    检查： ros2 lifecycle get /bt_navigator')
        return 1
    print('    [OK] server 就绪')

    # ---------------- ③ 逐个发目标点 ----------------
    ok = 0
    for i, (gx, gy, desc) in enumerate(goals, 1):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = node.get_clock().now().to_msg()
        g.pose.pose.position.x = float(gx)
        g.pose.pose.position.y = float(gy)
        g.pose.pose.orientation.w = 1.0   # 简化：都朝 +x，DWB 会自己转向

        print(f'    [{i}/{len(goals)}] 目标 ({gx:5.2f},{gy:5.2f})  {desc}  发送中...')
        send_fut = client.send_goal_async(g)
        rclpy.spin_until_future_complete(node, send_fut, timeout_sec=30)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            print('        [FAIL] goal 被拒')
            continue

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=180)
        result = res_fut.result()
        status = result.status if result else -1
        # 4 = SUCCEEDED
        if status == 4:
            print('        [PASS] 到达 ✓')
            ok += 1
        else:
            print(f'        [FAIL] status={status} '
                  f'(4=SUCCEED, 5=ABORTED, 6=CANCELED)')
        rclpy.spin_once(node, timeout_sec=0.5)

    print()
    print(f'====== 验收结果：{ok}/{len(goals)} 个目标到达 ======')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok == len(goals) else 1


if __name__ == '__main__':
    sys.exit(main())
