#!/usr/bin/env python3
"""打印当前 /odom 位姿："x y qw qz"。**已弃用，仅保留调试用。**

⚠️⚠️ 不要再拿它的输出当 AMCL 初值 ⚠️⚠️
  本文件末尾「坐标说明」原来写着「map 与 odom 初始重合，偏差通常 0.1m 内」。
  这句话只在**冷启动**成立。跑过几轮导航后 map->odom 已累积偏移
  （S34 实测差 1.4 m），拿 odom 坐标注进 /initialpose 就是注错的：
  AMCL 粒子撒到错误位置 → map->odom 跑出平面（z=3.372、roll=-93°）
  → /amcl_pose 冻结 → 之后所有导航结论全不可信。

  正确做法：`set_pose.py --auto`
    它读当前 map->odom，做二维刚体复合 map_pose = map->odom ∘ odom_pose，
    反推出真正的 map 坐标；注入后再反查 map->odom 验证，不健康自动重注。
  冷启动且车确实在原点时，用 `set_pose.py --map 0 0 0` 最稳。

  本文件现在只用于手动排查「odom 到底怎么走的」。
"""
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.stderr.write(
    '[odom_pose] ⚠️  已弃用：不要拿这个输出当 AMCL 初值。\n'
    '             改用 `python3 set_pose.py --auto`（读 map->odom 反推 + 注入后验证）\n')

rclpy.init()
node = rclpy.create_node('odom_pose_probe')
out = []


def cb(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    out.append('%.4f %.4f %.4f %.4f' % (p.x, p.y, q.w, q.z))


node.create_subscription(
    Odometry, '/odom', cb,
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

while not out:
    rclpy.spin_once(node, timeout_sec=0.2)

print(out[0])
node.destroy_node()
rclpy.shutdown()
