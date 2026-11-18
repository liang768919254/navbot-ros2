#!/usr/bin/env python3
"""动态避障实测：发一个目标，半途在路径上凭空生成障碍，看车是否绕行并到达。

用法：
  python3 tools/verify/dyn_obstacle_test.py --goal 2.0 --goal-y -2.0 \\
      --box 0.0 -2.0 --spawn-after 3.0

输出（机器可读）：
  结果  SUCCEEDED/ABORTED/TIMEOUT
  墙钟  xx.x s
  绕过  是/否（是否与障碍中心保持 > 0.35 m）
  最近  0.xx m（轨迹到障碍中心的最小距离）

设计要点：
  ① 障碍用 Gazebo 的 /spawn_entity 动态生成，不写进 world —— 这样才是"运行中突现"。
  ② 障碍必须够高（z 中心 0.4、高 0.8）才能被 z=0.12 的激光扫到；
     矮于激光平面的盒子在 RViz 里看得见、雷达却扫不到，是最容易误判的坑。
  ③ 结束后自动 /delete_entity 清理，避免污染后续测试。
"""
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
      <visual name="v">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0.85 0.15 0.15 1</ambient><diffuse>0.85 0.15 0.15 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DynTest(Node):
    def __init__(self, box, size):
        super().__init__('dyn_obstacle_test')
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.del_cli = self.create_client(DeleteEntity, '/delete_entity')
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.odom = None
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.box = box
        self.size = size
        self.min_dist = 1e9
        self.dist_to_box = 1e9
        self.track = False

    def _on_odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        d = math.hypot(p.x - self.box[0], p.y - self.box[1])
        self.dist_to_box = d
        if self.track:
            self.min_dist = min(self.min_dist, d)

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spawn_box(self):
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            print('!! /spawn_entity 不可用')
            return False
        sx, sy, sz = self.size
        req = SpawnEntity.Request()
        req.name = 'dyn_box'
        req.xml = BOX_SDF.format(name='dyn_box', sx=sx, sy=sy, sz=sz)
        req.initial_pose = Pose()
        req.initial_pose.position.x = self.box[0]
        req.initial_pose.position.y = self.box[1]
        req.initial_pose.position.z = sz / 2.0
        req.initial_pose.orientation.w = 1.0
        req.reference_frame = 'world'
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        ok = fut.result() is not None and fut.result().success
        print(f'[spawn] {"成功" if ok else "失败"}: ({self.box[0]},{self.box[1]})')
        return ok

    def delete_box(self):
        if not self.del_cli.wait_for_service(timeout_sec=3.0):
            return
        req = DeleteEntity.Request()
        req.name = 'dyn_box'
        fut = self.del_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def navigate(self, gx, gy, gyaw, timeout, spawn_after, spawn_when_dist=0.0):
        self.cli.wait_for_server(timeout_sec=15.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        q = yaw_to_quat(gyaw)
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        t0 = time.time()
        send = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('结果  REJECTED')
            return
        if spawn_when_dist > 0.0:
            # 距离触发：一直等到车接近到 N 米（最贴近真实"忽然出现"的场景）
            t_wait = time.time()
            while self.dist_to_box > spawn_when_dist and time.time() - t_wait < 120.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            print(f'[触发] 车距障碍 {self.dist_to_box:.2f} m 时生成')
        else:
            self.spin_for(spawn_after)
        self.spawn_box()
        self.track = True
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        wall = time.time() - t0
        res = rf.result()
        self.track = False
        state = 'TIMEOUT' if res is None else ('SUCCEEDED' if res.status == 4 else 'ABORTED')
        print(f'结果  {state}')
        print(f'墙钟  {wall:.1f} s')
        print(f'最近  {self.min_dist:.2f} m（轨迹到障碍中心）')
        size_diag = math.hypot(self.size[0], self.size[1]) / 2.0
        print(f'绕过  {"是" if self.min_dist > size_diag + 0.10 else "否"}')
        self.delete_box()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', type=float, required=True)
    ap.add_argument('--goal-y', type=float, required=True)
    ap.add_argument('--goal-yaw', type=float, default=0.0)
    ap.add_argument('--box', nargs=2, type=float, default=[0.0, -2.0])
    ap.add_argument('--size', nargs=3, type=float, default=[0.5, 0.5, 0.8])
    ap.add_argument('--spawn-after', type=float, default=3.0)
    ap.add_argument('--spawn-when-dist', type=float, default=0.0,
                    help='>0 时改为「车接近到该距离才生成障碍」')
    ap.add_argument('--timeout', type=float, default=180.0)
    args = ap.parse_args()
    rclpy.init()
    node = DynTest(args.box, args.size)
    try:
        node.navigate(args.goal, args.goal_y, args.goal_yaw,
                      args.timeout, args.spawn_after, args.spawn_when_dist)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
