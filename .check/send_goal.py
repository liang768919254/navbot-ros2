#!/usr/bin/env python3
# ===========================================================================
# 发导航目标点 —— 等价于 RViz 的「Nav2 Goal」
# ===========================================================================
# 用法：
#   python3 .check/send_goal.py -2.30 0.0          # x y
#   python3 .check/send_goal.py -2.30 0.0 1.57     # x y yaw(弧度)，默认朝 +x
#
# ---------------------------------------------------------------------------
# ★★ 为什么不能直接用 ros2 action send_goal
# ---------------------------------------------------------------------------
#   命令行 YAML 里填不了动态时间戳，pose.header.stamp 会是 0（1970 年）。
#   Nav2 用这个 stamp 去做 TF 变换，拿到的是时间 0 的过时变换，
#   规划器算出的路径会退化（空路径或只有一个点），
#   FollowPath 收到空路径直接返回 SUCCESS ——
#
#   ★ 症状：命令行显示 "Goal finished with status: SUCCEEDED"，
#           但车一动不动，RViz 上那条绿线是上一次的残留。
#
#   本脚本填 header.stamp = 当前时间（nav_runner.py / test_stageD.sh 里
#   5/5 验证过的写法），并实时打印 distance_remaining 让你看着车走完。
# ===========================================================================

import sys
import time
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavGoal(Node):
    def __init__(self):
        super().__init__('send_goal')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._t0 = 0.0

    def feedback_cb(self, fb):
        d = fb.feedback.distance_remaining
        eta = fb.feedback.estimated_time_remaining.sec
        print(f'\r    剩余距离 {d:6.2f} m   预计 {eta:3d} s   '
              f'已用 {time.time() - self._t0:5.1f} s   ', end='', flush=True)

    def send(self, x, y, yaw):
        print('[1] 等待 /navigate_to_pose action server ...')
        if not self._client.wait_for_server(timeout_sec=60.0):
            print('    [FAIL] server 起不来 —— 检查 ros2 lifecycle get /bt_navigator')
            return 1
        print('    [OK] server 就绪')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()   # ★ 关键
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f'[2] 发送目标 ({x}, {y}) 朝 {math.degrees(yaw):.1f}° ...')
        self._t0 = time.time()
        send_fut = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=30)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            print('\n    [FAIL] goal 被拒绝')
            return 1
        print('    goal 已接受，执行中...')

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=300)
        result = res_fut.result()
        status = result.status if result else -1
        print()
        if status == 4:
            print(f'    [PASS] 到达目标 ✓   耗时 {time.time() - self._t0:.1f} s')
            return 0
        names = {1: 'ACCEPTED', 2: 'EXECUTING', 4: 'SUCCEEDED',
                 5: 'ABORTED', 6: 'CANCELED'}
        print(f'    [FAIL] status={status} ({names.get(status, "未知")})')
        return 1


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = NavGoal()
    rc = node.send(x, y, yaw)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
