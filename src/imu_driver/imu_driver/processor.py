#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
【模板】ROS 2 收发一体节点（订阅 → 处理 → 发布）

这是机器人节点最典型的形态，占实际项目的七成以上：
    订阅传感器/指令 → 中间算一下 → 发布结果

例子长这样：
    /cmd_vel_in  → [本节点做限速、平滑] → /cmd_vel_out
    /raw_image   → [本节点做压缩]      → /compressed_image

┌─ 数据流 ─────────────────────────────────────────┐
│  上游话题 ──→ [回调: 处理] ──→ 下游话题            │
│  create_subscription      create_publisher      │
└─────────────────────────────────────────────────┘

★ 标记处是使用模板时要改的地方。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class ImuDriverProcessor(Node):
    """★ 改名：ImuDriverProcessor → 你的业务名，如 VelocityLimiter"""

    def __init__(self, name):
        super().__init__(name)

        # ── 参数 ─────────────────────────────────────
        self.declare_parameter('input_topic', 'imu_driver_input')     # ★ 输入话题
        self.declare_parameter('output_topic', 'imu_driver_output')   # ★ 输出话题
        self.declare_parameter('process_delay_ms', 0)               # 模拟处理耗时
        self.declare_parameter('max_length', 100)                   # 业务参数示例

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.process_delay_ms = self.get_parameter('process_delay_ms').value
        self.max_length = self.get_parameter('max_length').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 出：发布者 ─────────────────────────────────
        self.publisher_ = self.create_publisher(String, out_topic, qos)

        # ── 入：订阅者 ─────────────────────────────────
        self.subscription_ = self.create_subscription(
            String,
            in_topic,
            self.process_callback,
            qos,
        )

        # ── 想定时主动发东西时，再加一个定时器 ──────────────
        # self.timer = self.create_timer(0.1, self.timer_callback)

        self.in_count = 0
        self.out_count = 0

        self.get_logger().info(
            f'处理器已启动：{in_topic} → {out_topic}'
        )

    def process_callback(self, msg):
        """
        ★ 核心业务逻辑写在这里。

        输入：msg（上游来的数据）
        输出：self.publisher_.publish(...)
        """
        self.in_count += 1

        # ═══ ★ 在这里写你的处理逻辑 ═══════════════════
        # 下面是示例：把输入字符串截断 + 加前缀
        # 实际业务里可能是：限速、滤波、坐标变换、协议解析、状态机跳转……
        processed = msg.data

        if self.process_delay_ms > 0:
            # 模拟耗时处理。注意：这会让本节点所有回调都被卡住！
            # 真耗时的处理请用 08_执行器与回调组/ 的多线程方案
            import time
            time.sleep(self.process_delay_ms / 1000.0)

        if len(processed) > self.max_length:
            processed = processed[:self.max_length] + '...'

        result = f'[已处理] {processed}'
        # ═══════════════════════════════════════════

        out_msg = String()
        out_msg.data = result
        self.publisher_.publish(out_msg)

        self.out_count += 1

        self.get_logger().debug(
            f'入 {self.in_count} 条 / 出 {self.out_count} 条'
        )

    # def timer_callback(self):
    #     """定时主动发布（心跳、状态上报等）"""
    #     msg = String()
    #     msg.data = f'heartbeat {self.get_clock().now().nanoseconds}'
    #     self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ImuDriverProcessor('imu_driver_processor')   # ★ 节点名

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
