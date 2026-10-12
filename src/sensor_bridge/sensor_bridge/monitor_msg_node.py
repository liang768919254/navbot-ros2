#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_msg_node.py —— 阶段三订阅端：吃 SensorData，做窗口统计并汇报

比阶段一的 monitor_node 多做了三件事，每件都对应一个真实工程需求：

  1. 【多通道统计】value / temperature 各自一份窗口统计，
     用 Welford 递推（精度无关，float32 也不丢；见 阶段一测试题与答案.md「附2」）
  2. 【丢帧检测】用 msg.seq 判断中间是否漏帧 —— 这是「话题上有数据」和
     「数据一帧没丢」的区别。ros2 topic hz 看不出这个。
  3. 【数据年龄 + 坐标名变化】用 header.stamp 算"这帧到我手里走了多久"，
     frame_id 一变就报一次（TF2 的伏笔：坐标名错了，整条链路的位置都是错的）

汇报样例：
  [monitor_msg] 窗口 5.0s | 帧数=50 | value: 均值=+1650.12 标准差=1.42 min=1647 max=1653
                | temperature: 均值=+25.01 标准差=0.13
                | 丢帧=0 乱序=0 重复=0 | 数据年龄=0.031s | frame_id=sensor_link
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_bridge.sensor_stats import ChannelSet, SeqTracker
from sensor_interfaces.msg import SensorData


class MonitorMsgNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('topic', '/sensor/value')
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('age_warn', 0.5)      # 数据年龄超过这个值就 warn

        topic = self.get_parameter('topic').value
        period = float(self.get_parameter('report_period').value)

        # 统计通道：想加字段就在这里加一个名字，其余代码不用动
        self.ch = ChannelSet(['value', 'temperature', 'raw_adc'])
        self.seq_tracker = SeqTracker()

        self.count = 0
        self.age_worst = 0.0
        self.age_sum = 0.0
        self.last_frame_id = None
        self.first_stamp = None
        self.last_report = time.monotonic()

        # QoS：传感器流用「尽力而为 + 只留最新几帧」比默认的"可靠"更合适。
        # 可靠传输在丢包时会重传 → 数据变成"迟到的旧值"，对实时曲线是负作用。
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(SensorData, topic, self.on_msg, qos)
        self.timer = self.create_timer(period, self.on_report)

        self.get_logger().info(
            f'monitor_msg 启动 | 订阅 {topic} | 窗口 {period}s | 类型 SensorData')

    # ──────────────────────────────────────────────────────────
    def on_msg(self, msg: SensorData):
        self.count += 1

        # ① 丢帧检测：帧号是 MCU 侧的，与 ROS 无关，所以能反映"串口那一段"的质量
        self.seq_tracker.update(msg.seq)

        # ② 数据年龄：header.stamp 是发布端打的（到达时刻），
        #    这里量的是"发布 → 订阅"在 ROS 图里走了多久。
        now = self.get_clock().now()
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = max(0.0, (now.nanoseconds - stamp_ns) * 1e-9)
        self.age_sum += age
        if age > self.age_worst:
            self.age_worst = age
        if age > float(self.get_parameter('age_warn').value):
            self.get_logger().warn(f'数据年龄偏大：{age:.3f}s（下游会看到过期数据）',
                                   throttle_duration_sec=5.0)

        # ③ 坐标名变化：只在"变化"时打日志（日志的价值在变化，不在重复）
        #    这是 TF2 的伏笔 —— frame_id 写错，位置就全错。
        if msg.header.frame_id != self.last_frame_id:
            self.get_logger().info(
                f'frame_id 变更为 "{msg.header.frame_id}"'
                + ('' if self.last_frame_id is None else f'（原为 "{self.last_frame_id}"）'))
            self.last_frame_id = msg.header.frame_id

        # ④ 多通道统计
        for name in self.ch.names:
            self.ch.update(name, float(getattr(msg, name)))

        if self.first_stamp is None:
            self.first_stamp = stamp_ns

    # ──────────────────────────────────────────────────────────
    def on_report(self):
        now = time.monotonic()
        dt = now - self.last_report
        if self.count == 0:
            self.get_logger().warn('本窗口没有收到任何数据，检查发布端是否在运行')
            self.last_report = now
            return

        hz = self.count / dt if dt > 0 else 0.0
        age_avg = self.age_sum / self.count
        lost, ooo, dup = self.seq_tracker.snapshot()

        parts = [f'窗口 {dt:.1f}s | 帧数={self.count} | 实测 {hz:.2f} Hz']
        for name, w in self.ch.report().items():
            if w.n == 0:
                continue
            parts.append(
                f'{name}: 均值={w.mean:+.3f} 标准差={w.std:.3f} '
                f'[{w.vmin:.3f}, {w.vmax:.3f}]')
        parts.append(f'丢帧={lost} 乱序={ooo} 重复={dup}')
        parts.append(f'数据年龄 均={age_avg:.3f}s 峰值={self.age_worst:.3f}s')
        parts.append(f'frame_id={self.last_frame_id}')

        self.get_logger().info(' | '.join(parts))

        # 窗口清零 —— 与阶段一 stats_node 同一条纪律
        self.ch.reset()
        self.seq_tracker.reset_counters()
        self.count = 0
        self.age_sum = 0.0
        self.age_worst = 0.0
        self.last_report = now


# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MonitorMsgNode('monitor_msg')
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
