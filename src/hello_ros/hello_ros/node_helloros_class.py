#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class HelloROSNode(Node):
    """最小 ROS2 节点（类封装版）：每秒打印一条 Hello ROS"""

    def __init__(self, name):                   # 注意是 __init__，不是 __int__
        super().__init__(name)                  # 父类初始化，把节点名注册进 ROS 图
        self.get_logger().info('Hello ROS')     # 节点一起来先打一条
        self.timer = self.create_timer(1.0, self.on_timer)   # 每 1 秒回调一次

    def on_timer(self):
        self.get_logger().info('Hello ROS')


def main(args=None):
    rclpy.init(args=args)                       # 初始化 rclpy
    node = HelloROSNode('node_helloros_class')
    try:
        rclpy.spin(node)                        # 必须 spin，否则回调永不触发
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()                     # 销毁节点
        if rclpy.ok():                          # 关键守卫：信号触发时 context 可能已关
            rclpy.shutdown()                    # 关闭 rclpy


if __name__ == '__main__':
    main()
