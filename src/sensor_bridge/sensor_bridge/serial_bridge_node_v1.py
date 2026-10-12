#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge_node —— 第一版：最小闭环

职责：把 /tmp/ttyV1（或 /dev/ttyUSB0）上来的 ASCII 行，变成 ROS2 话题。

第一版只求三件事：
  1. 独立线程读串口（阻塞的活儿不能占住主线程）
  2. 发布 std_msgs/Float32 到 /sensor/value
  3. Ctrl+C 能干净退出，不打 Traceback

故意不做（留给后面几步，别现在加）：
  · 断线重连        → Step 6
  · 坏帧统计/告警    → Step 5
  · v2 四字段解析    → Step 4（这里只认单值 v1 协议）
  · launch 文件      → Step 7

运行：
  ros2 run sensor_bridge serial_bridge --ros-args -p port:=/tmp/ttyV1
"""

import threading                # 导入线程

import rclpy
import serial                   # 导入串口
from rclpy.executors import ExternalShutdownException               # 导入终端关闭异常
from rclpy.node import Node
from std_msgs.msg import Float32


class SerialBridgeNode(Node):

    def __init__(self, name):
        super().__init__(name)
        # 参数化：命令行 -p port:=xxx 可覆盖，不写就用默认
        self.declare_parameter('port', '/tmp/ttyV1')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value         # baud波特

        self.pub = self.create_publisher(Float32, '/sensor/value', 10)

        # timeout 必须给！不给的话 readline 会永久阻塞（见教程自查题 1）
        self._ser = serial.Serial(port, baud, timeout=0.2)              # serial：串行

        # ── 线程间的「停车信号」──────────────────────────────
        # 用 Event 而不是 bool：Event.is_set() / set() 是线程安全的，
        # 而且自带等待语义。用普通 bool 在多线程下没有任何可见性保证。
        self._stop = threading.Event()

        # daemon（守护进程）=True 只是兜底（主线程死了不让子线程吊住进程）。
        # 真正的干净退出靠 destroy_node() 里的 _stop.set() + close()。
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'已打开 {port} @ {baud}')

    # ── 读线程：所有会阻塞的调用都关在这里 ──────────────────────
    def _reader_loop(self):                 # _reader_loop读取循环
        while not self._stop.is_set():
            line = self._ser.readline()      # 读到 \n 返回；超时返回 b''
            if not line:
                continue                     # 超时，啥也没读到 → 回头查停车信号

            # errors='ignore' 是关键：串口上出现非 ASCII 字节是常态，
            # 用默认的 strict 会抛 UnicodeDecodeError，直接把线程打死。
            text = line.decode('ascii', errors='ignore').strip()            # strip剥去
            if not text:
                continue                     # 空行 / 纯乱码被 ignore 后什么都不剩

            try:
                value = float(text)
            except ValueError:
                continue                     # 上电横幅、乱码、半帧 → 整行丢弃

            msg = Float32()
            msg.data = value
            self.pub.publish(msg)            # 从非 executor 线程 publish 是安全的

    def destroy_node(self):
        # 顺序很重要：先立停车牌，再关串口。
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode('serial_bridge')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # KeyboardInterrupt：自己按的 Ctrl+C
        # ExternalShutdownException：被外部（ros2 node kill / launch 的 SIGINT）关掉
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():          # 上下文可能已被外部关掉，重复 shutdown 会报错
            rclpy.shutdown()


if __name__ == '__main__':
    main()
