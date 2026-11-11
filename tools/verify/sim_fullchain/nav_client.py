#!/usr/bin/env python3
"""nav_client.py —— 常驻 rclpy NavigateToPose 客户端（stdin 驱动，一次进程跑完全程）。

★★★ 为什么要这个文件（S33，2026-11-10 夜）

原来 nav_patrol.py 每发一个目标就 fork 一次：
    timeout 240 ros2 action send_goal /navigate_to_pose ...

问题（实测，不是理论推测）：
  1. 每次都要新建一个 ROS 节点、重新发现 action server。
     Gazebo + RViz + Nav2 全跑满时，发现过程可能超过内部等待，
     目标**根本没发出去** —— Nav2 日志里连 "Begin navigating" 都没有。
  2. timeout 杀掉 CLI 后，action server 侧留下悬空的 goal handle，
     日志里出现
       [bt_navigator.rclcpp_action]: Failed to send goal response ...
       (timeout): client will not receive response
     下一条目标就被这个残留拖死。
  3. 输出是给人看的文本，只能靠 grep "SUCCEEDED" 猜终态；
     拿不到 action 的 status 字段，区分不了
     「目标从没被接受」和「被接受了但超时」。

现在改成：一个进程、一个节点、一个 action client，
从 stdin 逐行读 "x y"，每行发一个目标，等真实 result，
把一行 JSON 打到 stdout。父进程（nav_patrol.py）只负责解析。
  · 不再重复发现
  · 客户端始终活着，不会留悬空 handle
  · 拿到的是 action_msgs/GoalStatus 的真实状态码

协议
  stdin  : 每行 "<x> <y>"，读完即退出
  stdout : 每行一个 JSON
           {"state": "SUCCEEDED"|"ABORTED"|"CANCELED"|"TIMEOUT"|"REJECTED",
            "t": 秒, "detail": "..."}
  stderr : 人类可读日志（不参与解析）

注意：必须用系统的 /usr/bin/python3（rclpy 只在系统 Python 里）。
      由 nav_patrol.py 负责 source env.sh 后 exec。
"""
import json
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# 等 action server 出现的最长时间。机器忙时发现很慢，给足。
SERVER_WAIT = 60.0
# 单点导航上限，与 nav_patrol.py 的 TOL_TIME 判据呼应（判据是 120s，
# 这里留余量，让「慢但成功」能被如实记录，而不是粗暴掐断）。
GOAL_TIMEOUT = 240.0

STATE_MAP = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
}


class PatrolClient(Node):
    def __init__(self):
        super().__init__('nav_patrol_client')
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_server(self, timeout=SERVER_WAIT):
        return self.cli.wait_for_server(timeout_sec=timeout)

    def goto(self, x, y):
        """发一个目标并阻塞到终态。返回 (state, elapsed, detail)。"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        t0 = time.time()

        # ---- 1. 发目标，等 accept/reject ----
        send_fut = self.cli.send_goal_async(goal)
        # ★ 必须 spin 才会推进 future；同时留超时，不能裸等
        #
        # ★★ S34 修：原来是 20s，太短。
        #   实测：上一轮测试留下一个 goal handle 没被清理时，bt_navigator
        #   会卡在 rclcpp_action 里等那个旧 client 的超时（要等几十秒到几分钟），
        #   期间新目标 A 得进去但 accept 响应回不来。20s 一到我们就判 REJECTED，
        #   而其实再等十几秒就通了。
        #   放宽到 90s，并且打进度让用户知道不是死机。
        deadline = t0 + 90.0
        last_note = 0.0
        while not send_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            el = time.time() - t0
            if el > deadline:
                return 'REJECTED', el, '发送目标 90s 无响应（server 可能正在处理旧的悬空 goal）'
            if el - last_note >= 10.0:
                last_note = el
                sys.stderr.write('  .. 等 accept %.0fs\n' % el)
                sys.stderr.flush()

        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return 'REJECTED', time.time() - t0, '目标被 action server 拒绝'

        # ---- 2. 等结果 ----
        res_fut = gh.get_result_async()
        while not res_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            dt = time.time() - t0
            if dt > GOAL_TIMEOUT:
                # 主动取消，别把 handle 悬在那儿
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return 'TIMEOUT', dt, '超过 %.0fs 未结束' % GOAL_TIMEOUT
            # 每 10s 打一行进度到 stderr，方便父进程转成进度条
            if int(dt) % 10 == 0 and dt >= 1.0 and abs(dt - round(dt)) < 0.11:
                sys.stderr.write('  .. %.0fs\n' % dt)
                sys.stderr.flush()

        dt = time.time() - t0
        wrapped = res_fut.result()
        if wrapped is None:
            return 'ABORTED', dt, '结果为空'
        state = STATE_MAP.get(wrapped.status, 'UNKNOWN')
        return state, dt, 'status=%d' % wrapped.status


def main():
    rclpy.init()
    node = PatrolClient()
    try:
        if not node.wait_server():
            # 起不来就直接告诉父进程，别让它干等 240 秒
            sys.stdout.write(json.dumps(
                {'state': 'NO_SERVER', 't': 0.0,
                 'detail': '%.0fs 内没发现 /navigate_to_pose action server'
                           % SERVER_WAIT}) + '\n')
            sys.stdout.flush()
            return 3

        sys.stderr.write('[nav_client] action server 已就绪\n')
        sys.stderr.flush()

        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            x, y = float(parts[0]), float(parts[1])
            state, dt, detail = node.goto(x, y)
            sys.stdout.write(json.dumps(
                {'state': state, 't': round(dt, 2), 'detail': detail}) + '\n')
            sys.stdout.flush()
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
