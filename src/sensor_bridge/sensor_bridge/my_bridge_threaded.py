#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
my_bridge_threaded —— 0D 验收 3：把读串口挪进独立线程

和 my_bridge.py 的差别，只有三处：

  ① 读动作从「定时器回调」搬到「独立线程的 while 循环」
     → spin 线程再也不碰串口，ros2 node info 秒回

  ② timeout 可以放心用 0.2（甚至 None 也行）
     → 因为阻塞的是子线程，堵死了也只是这一条线程在等，
       执行器照转，订阅/服务/参数全部正常

  ③ 加一把锁保护「最新值」
     → 子线程写 self._latest，定时器线程读 self._latest，
       两个线程动同一块内存，必须串行化

⚠️ 线程版仍然遵守一个规矩：**ROS 的 publish 只能在 spin 线程里调**
   （rclpy 默认不是线程安全的）。所以子线程只负责"读到值 → 存起来"，
   真正 publish 交给定时器回调，两边用锁交接。
   这就是"生产者-消费者 + 最新值"模型，也正好回答第二层第 3 题。
"""

import threading

import rclpy
import serial

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32


class SerialBridgeThreadedNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('port', '/tmp/ttyV1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('report_period', 0.2)   # 发布周期 ≠ 读周期

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        period = self.get_parameter('report_period').value

        self.pub = self.create_publisher(Float32, '/sensor/value', 10)

        try:
            # 注意：这里的 timeout 已经不是"保护节点"的手段了，
            # 只是让子线程有机会周期性检查 _running 标志、好退出。
            self._ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            raise

        # 线程间共享状态：最新值 + 锁 + 退出标志
        self._lock = threading.Lock()
        self._latest = None
        self._has_new = False
        self._running = True
        # 丢帧/坏帧统计，退出时打一条，方便和正式版对账
        self._stat_bad = 0
        self._stat_ok = 0

        # ★ 核心：独立读线程。daemon=True 让主进程退出时不被它吊住
        self._reader = threading.Thread(
            target=self._read_loop, name='serial_reader', daemon=True)
        self._reader.start()

        # 发布定时器：只管把「最新值」发出去，绝不碰串口
        self.timer = self.create_timer(period, self._on_publish)

        self.get_logger().info(
            f'my_bridge_threaded 已启动 | port={port} | '
            f'读线程独立运行，发布周期 {period}s')

    # ── 子线程：读串口（唯一允许阻塞的地方）───────────────────
    def _read_loop(self):
        """独立线程主循环。这里的阻塞不影响 ROS 执行器。"""
        while self._running:
            try:
                line = self._ser.readline()
            except (serial.SerialException, OSError) as e:
                # 正式版在这里做重连；最小线程版先退出循环并告警
                self.get_logger().error(f'串口读异常，读线程退出: {e}')
                break

            if not line:
                continue                      # 超时：正常，继续等

            text = line.decode('ascii', errors='ignore').strip()
            if not text:
                continue

            try:
                value = float(text)
            except ValueError:
                with self._lock:
                    self._stat_bad += 1
                continue                      # 坏帧：计数后跳过，不刷屏

            with self._lock:                  # ← 交接点
                self._latest = value
                self._has_new = True
                self._stat_ok += 1

    # ── spin 线程：发布（ROS 调用只在这里发生）───────────────
    def _on_publish(self):
        with self._lock:
            if not self._has_new:
                return                        # 没有新数据就不发，避免重复推
            value = self._latest
            self._has_new = False

        msg = Float32()
        msg.data = value
        self.pub.publish(msg)

    def destroy_node(self):
        # 先让子线程看见退出信号，再 join，最后关串口
        self._running = False
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        with self._lock:
            self.get_logger().info(
                f'读线程已停止 | 好帧 {self._stat_ok} 坏帧 {self._stat_bad}')
        if getattr(self, '_ser', None) is not None and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeThreadedNode('my_bridge_threaded')
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
