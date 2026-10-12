#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
演示：在其他包里调用自定义接口（客户端：同时订阅 msg + 请求 srv）

这一个节点把两种用法都覆盖了：
  - 订阅 demo_interface/msg/Student     → 收发布者发的学生信息
  - 请求  demo_interface/srv/AddThreeInts → 要一次加法结果
"""

import rclpy                                          # ROS2 Python 接口库
from rclpy.node import Node                           # ROS2 节点类

from demo_interface.msg import Student                # ← 自定义消息
from demo_interface.srv import AddThreeInts           # ← 自定义服务


class DemoClient(Node):
    """既订阅话题，又调用服务的综合客户端"""

    def __init__(self, name):
        super().__init__(name)

        # 1. 订阅者：话题名和发布者一致，才能收到数据
        self.subscription = self.create_subscription(
            Student,
            'student_info',
            self.student_callback,
            10,
        )

        # 2. 服务客户端：先创建对象，等要用的时候再连接
        self.client = self.create_client(AddThreeInts, 'add_three_ints')

        # 每 3 秒调一次加法服务
        self.timer = self.create_timer(3.0, self.call_add_service)
        self.call_count = 0

        self.get_logger().info('客户端已启动')

    def student_callback(self, msg):
        """每收到一条 Student 消息就打印出来"""
        self.get_logger().info(
            f'[订阅到] 姓名={msg.name}, 年龄={msg.age}, '
            f'成绩={msg.score}, 课程={msg.courses}'
        )

    def call_add_service(self):
        """请求一次加法服务"""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('加法服务端还没上线，稍后再试……')
            return

        request = AddThreeInts.Request()          # 构造请求对象
        request.a = 10
        request.b = 20
        request.c = 30 + self.call_count          # 每次让第三个数递增，便于区分

        # 异步发送请求，立刻返回一个 future
        future = self.client.call_async(request)
        future.add_done_callback(self.add_response_callback)

        self.call_count += 1

    def add_response_callback(self, future):
        """服务端返回结果后被调用"""
        response = future.result()
        self.get_logger().info(f'[服务应答] 三数之和 = {response.sum}')


def main(args=None):
    rclpy.init(args=args)

    node = DemoClient('demo_client')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
