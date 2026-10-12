#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
monitor_node —— sensor_bridge 阶段 1：数据监视器

【这个节点解决什么问题】
数据发出来了，但"发得对不对"没人知道。监视器就是那个盯着数据流的人：
累计了多少帧、现在什么水平、峰值多高。这是所有调试的第一手依据——
以后你接真传感器，第一个动作永远是起一个这样的监视节点。

    订阅 /sensor/value → 累计统计 → 每 2 秒汇报一次
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class MonitorNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('report_period', 2.0)   # 汇报周期（秒）

        # ══════════════════════════════════════════════════════
        # [核心] 统计变量：累计式，不存数组
        #   收到 100 万帧也只占 5 个变量的内存（O(1)）。
        #   初学者最常见的写法是 self.values.append(v) 然后求平均——
        #   跑上半小时内存就顶不住了。养成"边收边算"的习惯。
        # ══════════════════════════════════════════════════════
        self.count = 0                    # 总帧数
        self.total = 0.0                  # 求和（求均值用）
        self.max_value = float('-inf')    # 最大值，初始给最小
        self.min_value = float('inf')     # 最小值，初始给最大
        self.last_value = 0.0             # 最新一帧
        self.noise_value = 0.0            # 噪音值
        # ══════════════════════════════════════════════════════
        # [核心] 订阅者：消息类型 + 话题名 + 回调 + 队列深度
        #   注意参数顺序和发布者不同：回调函数在话题名之后。
        # ══════════════════════════════════════════════════════
        self.sub = self.create_subscription(
            Float32, '/my_sensor/v', self.on_value, 10
        )
        self.sub_noise = self.create_subscription(
            Float32, '/my_sensor/noise', self.on_noise, 10
        )
        # ══════════════════════════════════════════════════════
        # [设计] 汇报用**独立的定时器**，不在回调里判断时间。
        #   数据来的频率和汇报频率是两件不相关的事，解耦后
        #   两边互不影响：发布端改到 1000Hz，汇报依然是 2 秒一次。
        # ══════════════════════════════════════════════════════
        period = self.get_parameter('report_period').value
        self.timer = self.create_timer(period, self.on_report)

        self.get_logger().info(
            f'monitor 已启动 | 监听 /my_sensor/v + /my_sensor/noise | 每 {period}s 汇报一次'
        )

    # ──────────────────────────────────────────────────────────
    def on_value(self, msg):
        """订阅回调：每收到一帧就被调用一次。里面只做最轻的活。"""
        v = msg.data

        self.count += 1
        self.total += v
        self.last_value = v
        if v > self.max_value:
            self.max_value = v
        if v < self.min_value:
            self.min_value = v

    def on_noise(self, msg):
        self.noise_value = msg.data
        # self.get_logger().info(f'噪音={self.noise_value:+.3f}')

    # ──────────────────────────────────────────────────────────
    def on_report(self):
        """定时器回调：定期把统计结果打印出来"""
        # [设计] 一帧都没收到时要能自己说明白，而不是打印一堆 0 或除零崩溃。
        if self.count == 0:
            self.get_logger().warn('尚未收到任何数据，检查发布者是否在运行')
            return

        mean = self.total / self.count
        self.get_logger().info(
            f'帧数={self.count:5d} | 最新={self.last_value:+.3f} | '
            f'均值={mean:+.3f} | 最大={self.max_value:+.3f} | '
            f'最小={self.min_value:+.3f} | 噪音={self.noise_value:+.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    # 一个进程只起一个节点：订阅两条话题，是一个节点里的两个订阅者，
    # 不需要再 new 第二个节点出来。
    node = MonitorNode('monitor')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
