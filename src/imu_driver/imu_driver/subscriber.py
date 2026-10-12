#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
【模板】ROS 2 订阅者 Subscriber

订阅者的本质：
    "注册一个回调函数，告诉 DDS：这条话题上有新消息就叫我。"
    你不主动去取数据，数据会推过来（push 模型）。

┌─ 订阅者两件套 ────────────────────────────────────┐
│  1. create_subscription(消息类型, 话题名, 回调, QoS)  │
│  2. 在回调里处理 msg（这个回调由 spin() 驱动）         │
└─────────────────────────────────────────────────┘

⚠️ 最关键的一点：
    订阅者的 QoS 必须和发布者**兼容**，否则两边静默地连不上——
    节点都在跑、话题都列得出来，就是收不到数据。
    不兼容的组合见 07_QoS与DDS/README.md。
"""

import rclpy                                  # ROS 2 Python 接口库
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node                   # 节点基类
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

from std_msgs.msg import String               # ★ 必须和发布者用同一种类型


class ImuDriverSubscriber(Node):
    """★ 改名：ImuDriverSubscriber → 你的业务名"""

    def __init__(self, name):
        super().__init__(name)

        # ── 参数：话题名、是否打印内容 ─────────────────────
        self.declare_parameter('topic_name', 'imu_driver_topic')   # ★ 要和发布者一致
        self.declare_parameter('verbose', True)

        topic_name = self.get_parameter('topic_name').value
        self.verbose = self.get_parameter('verbose').value

        # ── QoS：必须与发布者兼容 ────────────────────────
        # ★ 发布者用 RELIABLE，订阅者也用 RELIABLE（最保险）
        # ★ 发布者用 BEST_EFFORT 时，订阅者用这两种都行
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,   # ★ 见 README 的兼容矩阵
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 核心：创建订阅者 ────────────────────────────
        #   第 1 个参数：期望收到的消息类型
        #   第 2 个参数：话题名
        #   第 3 个参数：收到消息时的回调函数（注意别加括号，传函数本身）
        #   第 4 个参数：QoS
        self.subscription_ = self.create_subscription(
            String,
            topic_name,
            self.topic_callback,
            qos,
        )

        self.count = 0

        self.get_logger().info(f'订阅者已启动，正在监听：{topic_name}')

    def topic_callback(self, msg):
        """
        每收到一条消息就被调用一次。
        这个函数运行在 spin() 所在的线程里（默认单线程 → 会阻塞其他回调）。
        干重活/耗时的处理要另开线程，见 08_执行器与回调组/。
        """
        self.count += 1

        if self.verbose:
            self.get_logger().info(f'收到第 {self.count} 条："{msg.data}"')
        else:
            # 只报数量，避免高频刷屏
            if self.count % 100 == 0:
                self.get_logger().info(f'已累计收到 {self.count} 条消息')


def main(args=None):
    rclpy.init(args=args)

    node = ImuDriverSubscriber('imu_driver_subscriber')   # ★ 节点名

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # ★ rclpy.spin() 被外部关闭信号打断时抛的，属于正常退出
        #   （Ctrl+C 竞态、kill、systemd 停服务），不该打印 Traceback
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
