#!/usr/bin/env python3
# ===========================================================================
# 给 AMCL 发初始位姿 —— 等价于 RViz 的「2D Pose Estimate」
# ===========================================================================
# 用法：
#   python3 .check/seed_initialpose.py                  # 默认 (0, 0) 朝 +x
#   python3 .check/seed_initialpose.py 0.5 -0.3 1.57    # x y yaw(弧度)
#
# ---------------------------------------------------------------------------
# ★★ 为什么不能直接用 ros2 topic pub --once
# ---------------------------------------------------------------------------
#   1. --once 只发一次。AMCL 用的是 volatile QoS，还没订阅上时消息就丢了，
#      表现就是「命令执行成功、什么都没发生、map→odom 永远不出现」。
#   2. 命令行里没法填 header.stamp，默认填成 0（1970 年）。
#
#   本脚本连发 8 次 + 每次填当前时间戳 —— 这是 test_stageD.sh 里
#   5/5 目标点全部到达验证过的写法。
#
# ★ 时机：必须在 Nav2 launch 起来之后（约 8 秒）再发，发完等 5 秒再验证。
# ===========================================================================

import sys
import math
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0   # 弧度

    rclpy.init()
    n = rclpy.create_node('seed_initialpose')
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    # 平面运动只用 z / w 两个分量
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # 给一点协方差，让粒子撒开一点更容易收敛
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068

    for _ in range(8):
        msg.header.stamp = n.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.2)

    print(f'[seed_initialpose] 已发布 8 次：x={x}  y={y}  '
          f'yaw={yaw:.3f} rad ({math.degrees(yaw):.1f}°)')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
