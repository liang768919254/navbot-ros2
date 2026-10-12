#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class HelloROSNode(Node):
    """最小 ROS2 节点：每秒打印一条 Hello ROS"""

    def __init__(self, name):
        super().__init__(name)                  # 父类初始化，把节点名注册进 ROS 图
        self.get_logger().info('Hello ROS')     # 节点一起来先打一条
        self.timer = self.create_timer(1.0, self.on_timer)   # 每 1 秒回调一次

    def on_timer(self):
        self.get_logger().info('Hello ROS')


def main(args=None):
    rclpy.init(args=args)                       # 初始化 rclpy
    node = HelloROSNode('node_helloros')
    try:
        rclpy.spin(node)                        # 阻塞在此，直到 Ctrl+C
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()                     # 销毁节点
        if rclpy.ok():                          # 关键：信号触发时 context 可能已关，重复关会抛 RCLError
            rclpy.shutdown()                    # 关闭 rclpy


if __name__ == '__main__':
    main()
