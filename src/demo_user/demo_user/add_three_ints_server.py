#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
演示：在其他包里调用自定义接口（服务 srv）

服务端的处理流程只有三步：
  1. 创建服务：指定接口类型 + 服务名 + 回调函数
  2. 回调里从 request 取出参数，算出结果
  3. 把结果塞进 response 返回
"""

import rclpy                                          # ROS2 Python 接口库
from rclpy.node import Node                           # ROS2 节点类

from demo_interface.srv import AddThreeInts           # ← 跨包调用自定义服务


class AddThreeIntsServer(Node):
    """三个整数求和的加法器服务端"""

    def __init__(self, name):
        super().__init__(name)

        # 创建服务
        #   第 1 个参数：接口类型
        #   第 2 个参数：服务名（客户端按这个名字找服务）
        #   第 3 个参数：收到请求时的回调函数
        self.srv = self.create_service(
            AddThreeInts,
            'add_three_ints',
            self.add_three_ints_callback,
        )

        self.get_logger().info('加法服务端已启动，等待请求……')

    def add_three_ints_callback(self, request, response):
        """
        收到请求自动被调用。

        request  —— 请求部分，带有 srv 文件 --- 上方定义的字段 a / b / c
        response —— 应答部分，需要你在下面填 sum 字段
        """
        response.sum = request.a + request.b + request.c

        self.get_logger().info(
            f'收到请求：{request.a} + {request.b} + {request.c} = {response.sum}'
        )

        return response                      # 返回值必须把 response 交回去


def main(args=None):
    rclpy.init(args=args)

    node = AddThreeIntsServer('add_three_ints_server')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
