#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
my_bridge —— 串口桥最小版（0D · 验收 1 / 验收 2）

设计边界（刻意不做的事）：
  - 不做重连        → 见 serial_bridge_node.py（正式版）
  - 不做独立线程    → 见 my_bridge_threaded.py（验收 3）
  - 不做协议解析    → v1 协议只发单值，float() 就够
  - 不做 QoS/仪表盘 → 这里只管"读一行、发一个数"

跑法（四终端）：
  T1  socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
  T2  python3 virtual_serial_sender.py --port /tmp/ttyV0 --rate 20 --v1
  T3  ros2 run sensor_bridge my_bridge --ros-args -p port:=/tmp/ttyV1
  T4  ros2 topic echo /sensor/value --once ; ros2 topic hz /sensor/value
"""

import rclpy
import serial

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32


class SerialBridgeNode(Node):

    def __init__(self, name):
        super().__init__(name)

        # 参数：端口 / 波特率 / 定时器周期
        self.declare_parameter('port', '/tmp/ttyV1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('report_period', 0.2)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        period = self.get_parameter('report_period').value

        # 发布者：/sensor/value，队列深度 10
        self.pub = self.create_publisher(Float32, '/sensor/value', 10)

        # 打开串口。timeout 是这道题的题眼：
        #   timeout=0.2 → readline 最多堵 0.2s，节点还能喘气（验收 1）
        #   timeout=None → readline 永久阻塞，节点当场死给你看（验收 2）
        try:
            self._ser = serial.Serial(port, baud, timeout=None)
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            raise

        # 定时器：在 __init__ 里就把周期钉死了
        # ★ 这就是 0C 里"改 rate 参数频率不变"的同一个根因：
        #   结构参数（period）必须重建定时器才生效，set 参数改不动它
        self.timer = self.create_timer(period, self.on_report)

        self.get_logger().info(
            f'my_bridge 已启动 | port={port} baud={baud} | 每 {period}s 汇报一次')

    def on_report(self):
        """定时器回调：读一行 → 解析 → 发一个 Float32。

        ⚠️ 这个方法有一个已知的设计债（0D 的隐藏考点）：
           每轮只 readline() 一次。当 --rate 20 时，串口每秒来 20 行，
           而本函数每 report_period(5s) 才读 1 行 → 有效吞吐仅 0.2 Hz。
           数据不会丢（内核缓冲 4KB ≈ 512 行），但会**严重滞后且大量滞留**。
           干净做法：while 把缓冲读空，只留最后一个值发布（见线程版）。
        """
        line = self._ser.readline()

        # 超时返回 b''，必须 return —— 这里写 pass 会掉进"解析空串"
        if not line:
            return

        text = line.decode('ascii', errors='ignore').strip()

        # 纯空行（真板子上电横幅后常见），同样 return
        if not text:
            return

        try:
            value = float(text)
        except ValueError:
            # 非单值行：v2 四字段 / 乱码 / 超长行 / 空字段全落这里。
            # 静默计数是正式版的事，最小版只打一条警告。
            self.get_logger().warn(f'无法解析: {text[:60]!r}')
            return

        msg = Float32()
        msg.data = value
        self.pub.publish(msg)

    def destroy_node(self):
        # 清单"常见错法"点名的坑：退出时忘了 close。
        # 不 close 的下场：串口 fd 泄漏，socat 那一端不会立刻收到 EOF，
        # 下次重开端口可能被占用（真板子上表现为"设备忙"）。
        if getattr(self, '_ser', None) is not None and self._ser.is_open:
            self._ser.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode('my_bridge')
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
