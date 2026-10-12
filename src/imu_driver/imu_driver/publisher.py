#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
【模板】ROS 2 发布者 Publisher

话题（Topic）的本质：
    多对多的**异步**数据流。发布者把消息丢进话题就走了，完全不知道
    有没有人在听、有几个在听。这种"松耦合"是 ROS 2 灵活性的来源，
    代价是发布者拿不到回执（需要回执就用服务）。

┌─ 发布者三件套 ────────────────────────────────────┐
│  1. create_publisher(消息类型, 话题名, QoS)          │
│  2. 造一个消息对象，往字段里填值                       │
│  3. publish(msg)                                  │
└─────────────────────────────────────────────────┘

★ 标记处是使用模板时要改的地方。
"""

import rclpy                                  # ROS 2 Python 接口库
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node                   # 节点基类
from rclpy.qos import QoSProfile              # QoS 配置
from rclpy.qos import ReliabilityPolicy       # 可靠性策略
from rclpy.qos import HistoryPolicy           # 历史策略

# ★ 消息类型：内置的有 std_msgs / geometry_msgs / sensor_msgs 等
#   自定义的写成 from 你的接口包.msg import 消息名
from std_msgs.msg import String


class ImuDriverPublisher(Node):
    """★ 改名：ImuDriverPublisher → 你的业务名"""

    def __init__(self, name):
        super().__init__(name)

        # ── 声明参数：让话题名、周期能在启动时改（不用重编译）──
        # ★ 这三个参数按需增删
        self.declare_parameter('topic_name', 'imu_driver_topic')   # ★ 话题名
        self.declare_parameter('publish_rate', 1.0)             # 发布频率 Hz
        self.declare_parameter('frame_prefix', 'data')          # 消息内容前缀

        topic_name = self.get_parameter('topic_name').value
        publish_rate = self.get_parameter('publish_rate').value
        self.frame_prefix = self.get_parameter('frame_prefix').value

        # ── 配置 QoS ────────────────────────────────────
        # ★ 按数据类型选（详细说明见 07_QoS与DDS/）：
        #   传感器数据（图像/IMU/激光）→ BEST_EFFORT，丢几帧没关系
        #   控制指令、状态更新     → RELIABLE，一帧都不能丢
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,   # ★ 传感器改 BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,          # 只保留最近 N 条
            depth=10,                                 # 队列深度 N
        )

        # ── 核心：创建发布者 ────────────────────────────
        #   第 1 个参数：消息类型（决定这条话题上能跑什么数据）
        #   第 2 个参数：话题名（订阅者按这个名字找过来）
        #   第 3 个参数：QoS 配置
        self.publisher_ = self.create_publisher(String, topic_name, qos)

        # ── 定时器：周期性触发发布 ───────────────────────
        #   参数是"周期秒数"，1.0 / rate 把 Hz 换成秒
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.count = 0   # 计数器，让每条消息内容不同，便于观察

        self.get_logger().info(
            f'发布者已启动：话题={topic_name}, 频率={publish_rate}Hz'
        )

    def timer_callback(self):
        """定时器回调：每隔一个周期被调用一次"""

        # ── 第 1 步：造消息对象 ─────────────────────────
        msg = String()

        # ── 第 2 步：填字段 ────────────────────────────
        #   字段名必须和 .msg 定义里完全一致，否则 AttributeError
        msg.data = f'{self.frame_prefix} #{self.count}'

        # ── 第 3 步：发出去 ────────────────────────────
        self.publisher_.publish(msg)

        # 日志：生产中建议降级为 debug，避免高频刷屏拖慢节点
        self.get_logger().info(f'已发布："{msg.data}"')

        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = ImuDriverPublisher('imu_driver_publisher')   # ★ 节点名

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
