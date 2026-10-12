#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fake_sensor_node —— sensor_bridge 阶段 1：软件假传感器

【这个节点解决什么问题】
手头还没接 STM32 时，先用软件按固定频率造一条"长得像传感器"的数据流。
它的价值不在正弦波本身，而在于：**接口（话题名 + 消息类型）先定死**，
等阶段 2 把串口读上来的真实数值往同一个话题一发，下游（monitor、rqt_plot、
以后的上位机）一行都不用改。

    定时器(10Hz) → 算正弦 + 高斯噪声 → publish 到 /my_sensor/v 和 /my_sensor/noise

作者：小梁同学 / 轻舞   日期：2026-09-16
"""

import math                                  # 标准库，不用装

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32             # 话题上的消息类型
import random


class FakeSensorNode(Node):

    def __init__(self, name):
        super().__init__(name)

        # ══════════════════════════════════════════════════════
        # [核心] 参数三连：declare → get → 用
        #   参数的意义：让"行为"变成可配置项，而不是写死在代码里。
        #   默认值就是"不传参数时"的行为，所以节点永远能单独跑起来。
        # ══════════════════════════════════════════════════════
        self.declare_parameter('amplitude', 1.0)   # 正弦幅值
        self.declare_parameter('freq', 0.5)        # 正弦波自身频率 Hz
        self.declare_parameter('rate', 10.0)       # 发布频率 Hz
        self.declare_parameter('noise', 0.0)       # 模拟噪音 noise
        # [设计] rate 决定定时器周期，属于"结构参数"——只在启动时读一次，
        #        运行中改它不会重建定时器（要改就重启，或用 --ros-args 传）。
        #        而 amplitude / freq 是"数据参数"，每帧现读，随时能改。
        self.rate = self.get_parameter('rate').value
        self.step = 1.0 / self.rate                # 每帧前进的仿真时间

        # ══════════════════════════════════════════════════════
        # [核心] 发布者：消息类型 + 话题名 + 队列深度
        # ══════════════════════════════════════════════════════
        self.pub = self.create_publisher(Float32, '/my_sensor/v', 10)
        self.pub_noise = self.create_publisher(Float32, '/my_sensor/noise', 10)
        # ══════════════════════════════════════════════════════
        # [核心] 定时器：周期秒数 + 回调函数
        #   create_timer(周期, 回调) —— 周期是"秒"，所以要 1/rate
        # ══════════════════════════════════════════════════════
        self.timer = self.create_timer(self.step, self.on_timer)

        self.t = 0.0   # 仿真时间轴（秒）

        self.get_logger().info(
            f'fake_sensor 已启动 | rate={self.rate}Hz | 话题=/my_sensor/v + /my_sensor/noise'
        )

    # ──────────────────────────────────────────────────────────
    def on_timer(self):
        """定时器回调：每 1/rate 秒被调用一次"""
        # [设计] 每帧现读参数 → ros2 param set 立刻生效。
        #        代价是一次字典查找（微秒级，可忽略）。
        #        产品级节点的正规写法是 add_on_set_parameters_callback，
        #        但那要处理返回值与校验，先用这个"够用版"。
        amplitude = self.get_parameter('amplitude').value
        freq = self.get_parameter('freq').value
        noise = self.get_parameter('noise').value
        # 正弦：y = A · sin(2πft)
        # [坑] 这里用自累加的 self.t，不用 time.time()。
        #      time.time() 会受定时器抖动影响，导致波形忽快忽慢；
        #      自累加时间轴是"理想等间隔采样"，波形才干净。
        noise_value = random.gauss(0.0, noise)
        value = amplitude * math.sin(2.0 * math.pi * freq * self.t)
        value = value + noise_value
        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)

        msg_noise = Float32()                      # ← 噪声分量单独发一份
        msg_noise.data = float(noise_value)
        self.pub_noise.publish(msg_noise)
        # [坑] throttle_duration_sec：10Hz 全打日志会刷屏到看不见东西，
        #      这个关键字参数让同一条日志每秒最多出现一次。
        self.get_logger().info(
            f'发布 value={value:+.3f}',
            throttle_duration_sec=1.0,
        )

        self.t += self.step


def main(args=None):
    rclpy.init(args=args)
    # 一个进程只起一个节点。
    # 噪音用的是同一个节点里的第二个发布者（self.pub_noise），
    # 不需要、也不应该再 new 第二个节点出来。
    node = FakeSensorNode('my_sensor')
    try:
        rclpy.spin(node)                       # 阻塞在这里，直到 Ctrl+C
    except (KeyboardInterrupt, ExternalShutdownException):   # 键盘中断 / 外部关闭信号
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
