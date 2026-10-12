#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge_node.py —— 把 STM32 经 USB 串口发来的 ASCII 数据搬进 ROS2 话题

【版本说明】
  本文件是「阶段二 + 阶段三」的合并版：
    · 阶段二用它发布 std_msgs/Float32（默认行为，未变）
    · 阶段三不改这里的骨架，只用子类换掉两处钩子 → 发布自定义 SensorData
  两个入口共用同一条桥：
      ros2 run sensor_bridge serial_bridge        # Float32（阶段二）
      ros2 run sensor_bridge serial_bridge_msg    # SensorData（阶段三）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
设计要点（每一处都有理由，别随便改）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. 【阻塞 IO 关在独立读线程里】
   readline() 是阻塞调用。放在 spin 的线程里，整个节点会停摆：
   定时器不响、参数服务不应答、ros2 node info 卡死、Ctrl+C 关不掉。

2. 【读线程只更新"最新帧"，发布交给定时器】
   · 频率解耦：STM32 想发多快都行，下游看到的是等间隔数据
   · 陈旧数据保护：板子挂了就不发，绝不让下游拿到"五分钟前的鬼魂值"

3. 【四种异常全不崩】
   设备不存在 / 断线 / 乱码 / 超长行 —— 分别有明确的处理策略。

4. 【设备名走参数，默认 udev 别名】
   代码里绝不出现 /dev/ttyUSB0（除了注释）。

5. 【两处钩子 = 模板方法模式】（阶段三新增）
   _create_publisher() / _pack_msg() —— 子类只覆盖这两个，
   就能把同一条桥的出口从 Float32 换成任意自定义消息。
   线程模型、重连、参数、统计一行都不用动。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
参数（结构参数 = 启动读一次；数据参数 = 每帧现读）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  port           string  /dev/stm32_bridge   结构   串口设备（udev 别名）
  baud           int     115200              结构   波特率
  topic          string  /sensor/value       结构   发布话题名
  publish_rate   double  10.0                结构   发布频率 Hz（决定定时器）
  report_period  double  5.0                 结构   状态汇报周期 s（决定定时器）
  raw_line_max   int     256                 结构   单行最大字节数
  stale_timeout  double  1.0                 数据   多久没新数据就停发 s
  scale          double  1.0                 数据   主值缩放（整数化传 0.001）

判据回顾（阶段一 A5 / C2 已练过）：
  改它要重建定时器/重开句柄 → 结构参数，只读一次
  改它只影响每帧算出来的数 → 数据参数，现读，ros2 param set 立刻生效
"""

import threading
import time
from typing import Optional, Tuple

import rclpy
import serial                                  # ← 必须是 /usr/bin/python3 能 import 到的那个
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from sensor_bridge.serial_protocol import parse_line


class SerialBridgeNode(Node):

    # ★ 子类在这里登记自己额外需要的参数，基类负责声明。
    #   为什么用类属性：它在 super().__init__() 之前就能读到，
    #   所以子类不用去改基类的 __init__，也不用在节点建好之后补声明。
    #   阶段三的 serial_bridge_msg_node 会把它覆盖成
    #   {'frame_id': 'sensor_link', 'temp_scale': 0.01}
    EXTRA_PARAMS: dict = {}

    # ──────────────────────────────────────────────────────────
    def __init__(self, name):
        super().__init__(name)

        # ══ 结构参数：只在启动时读一次 ══════════════════════════
        self.declare_parameter('port', '/dev/stm32_bridge')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/sensor/value')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('raw_line_max', 256)

        # ══ 数据参数：每次用到现读 ══════════════════════════════
        self.declare_parameter('stale_timeout', 1.0)
        self.declare_parameter('scale', 1.0)

        # ══ 子类补充的参数 ══════════════════════════════════════
        for key, default in self.EXTRA_PARAMS.items():
            self.declare_parameter(key, default)

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._raw_line_max = int(self.get_parameter('raw_line_max').value)
        topic = self.get_parameter('topic').value
        publish_rate = max(float(self.get_parameter('publish_rate').value), 0.1)
        report_period = max(float(self.get_parameter('report_period').value), 0.5)

        # ══ 发布者（钩子 1）═════════════════════════════════════
        self.pub = self._create_publisher(topic)

        # ══ 跨线程共享状态 ══════════════════════════════════════
        self._ser = None                # pyserial 句柄
        self._lock = threading.Lock()   # 只保护"句柄本身"的打开/关闭/取用
        self._connected = False

        # 最新一帧。三个量都是「一次赋值」的标量 / 不可变元组，
        # 所以读线程写、定时器读，不需要额外加锁：
        #   _latest_fields 是 tuple（不可变），不会出现"半新半旧"的组合
        self._latest: Optional[float] = None            # 主值（已按 scale 缩放）
        self._latest_fields: Optional[Tuple[float, ...]] = None   # 原始字段，未缩放
        self._latest_seq: int = 0                       # v1 协议没有帧号，用自增值兜底

        self._last_rx = 0.0             # 上次收到数据的时刻（单调时钟秒）
        self._rx_count = 0
        self._bad_count = 0
        self._drop_count = 0
        self._seq_last = None
        self._stale_warned = False
        self._pack_failed = 0
        # ★ 用"节点启动的这一刻"做基准，而不是 0.0。
        #   写成 0.0 的话，第一个汇报窗口的 dt 会被判成 0 →
        #   实测频率打印成 "0.00 Hz"，看着像坏了，其实只是没基准。
        #   （这是实跑时抓出来的：日志第一行 实测 0.00 Hz，第二行才对）
        self._last_report = time.monotonic()
        self._rx_base = 0
        self._bad_base = 0

        # ══ 读线程 ══════════════════════════════════════════════
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop, name='serial_reader', daemon=True)
        self._thread.start()

        # ══ 发布定时器 / 汇报定时器 ══════════════════════════════
        self.timer = self.create_timer(1.0 / publish_rate, self.on_publish)
        self.report_timer = self.create_timer(report_period, self.on_report)

        self.get_logger().info(
            f'serial_bridge 启动 | 设备={self._port}@{self._baud} | '
            f'话题={topic} | 发布={publish_rate:g}Hz | '
            f'消息={self._msg_type_name()}'
        )

    # ══════════════════════════════════════════════════════════
    #  钩子：子类只覆盖这两个，就换掉了出口
    # ══════════════════════════════════════════════════════════
    def _create_publisher(self, topic):
        """钩子 1：创建发布者。默认发 std_msgs/Float32。"""
        return self.create_publisher(Float32, topic, 10)

    def _pack_msg(self, stamp):
        """钩子 2：把"最新一帧"打包成消息。默认返回 Float32。

        Args:
            stamp: 本次发布的 ROS 时间戳（builtin_interfaces/Time）。
                   Float32 用不上；自定义消息版本要塞进 header.stamp。
        Returns:
            消息对象；返回 None 表示这一帧不能打包（比如字段数不认识），
            基类会跳过发布并计数。
        """
        msg = Float32()
        msg.data = float(self._latest)
        return msg

    def _msg_type_name(self):
        """只用于日志：报出自己发布什么类型。"""
        return 'std_msgs/msg/Float32'

    # ══════════════════════════════════════════════════════════
    #  读线程：所有阻塞的活儿都在这里
    # ══════════════════════════════════════════════════════════
    def _reader_loop(self):
        while not self._stop.is_set():
            # ── 未连接：尝试打开，失败就等 1 秒再试（不崩、不刷屏）──
            if not self._connected:
                if not self._try_open():
                    self._stop.wait(1.0)     # 用 Event.wait 代替 sleep：退出更快
                    continue

            # ── 已连接：读一行 ────────────────────────────────
            try:
                with self._lock:
                    ser = self._ser
                if ser is None:
                    continue
                line = ser.readline()        # timeout 已设，最多阻塞 timeout 秒
            except (serial.SerialException, OSError) as e:
                # 拔 USB 就走这里：关句柄 → 转未连接 → 下轮重试
                self.get_logger().warn(f'串口读取异常，准备重连：{e}',
                                       throttle_duration_sec=5.0)
                self._close_port()
                self._stop.wait(0.5)
                continue

            if not line:
                continue                     # 超时，这一轮没数据

            # ── 解析 ─────────────────────────────────────────
            fields = parse_line(line, max_len=self._raw_line_max)
            if fields is None:
                # 横幅 / 乱码 / 空行 / 超长行 / 半包 —— 一律静默计数，不刷日志
                self._bad_count += 1
                continue

            # 字段布局：
            #   v1: [value]
            #   v2: [seq, value, raw_adc, temperature]
            #   v3: [seq, value, raw_adc, temperature, mcu_tick_ms]
            # 靠字段个数"猜"格式是权宜之计 —— 串口这一端的格式问题，
            # 自定义消息解决不了（它只管 PC 内部和下游）。
            if len(fields) >= 2:
                seq = int(fields[0])
                raw_value = fields[1]
                if self._seq_last is not None:
                    gap = (seq - self._seq_last) % 65536
                    if gap > 1:
                        self._drop_count += gap - 1
                self._seq_last = seq
            else:
                seq = self._rx_count
                raw_value = fields[0]

            scale = float(self.get_parameter('scale').value)   # 数据参数：现读

            # ★ 先写元组（不可变），再写派生标量 —— 顺序让"半新半旧"不可能发生
            self._latest_fields = tuple(fields)
            self._latest = raw_value * scale
            self._latest_seq = seq
            self._last_rx = time.monotonic()   # 单调时钟：NTP 跳变也不会影响判断
            self._rx_count += 1
            self._stale_warned = False

    # ──────────────────────────────────────────────────────────
    def _try_open(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.2)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(
                f'打开 {self._port} 失败（每 1s 重试）：{e}',
                throttle_duration_sec=5.0)
            return False

        with self._lock:
            self._ser = ser
        self._connected = True
        # 状态"跳变"才值得打一条日志：日志的价值在"变化"，不在"重复"
        self.get_logger().info(f'已连接 {self._port} @ {self._baud}，等待数据…')
        return True

    # ──────────────────────────────────────────────────────────
    def _close_port(self):
        with self._lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if self._connected:
            self.get_logger().warn(f'与 {self._port} 的连接已断开，等待重连…')
        self._connected = False

    # ══════════════════════════════════════════════════════════
    #  发布定时器：等间隔发布 + 陈旧数据保护
    # ══════════════════════════════════════════════════════════
    def on_publish(self):
        latest = self._latest
        if latest is None:
            return                           # 从未收到过数据，不发

        stale = float(self.get_parameter('stale_timeout').value)
        if time.monotonic() - self._last_rx > stale:
            if not self._stale_warned:
                self.get_logger().warn(
                    f'已 {stale:g}s 没有新数据，暂停发布（下游不会收到陈旧值）')
                self._stale_warned = True
            return

        # 时间戳取"发布的这一刻"。注意它标的是【到达时刻】，不是【采样时刻】——
        # 差别在 mcu_tick_ms 字段里留着，事后可以校正（见教程 6.2 陷阱②）。
        msg = self._pack_msg(self.get_clock().now().to_msg())
        if msg is None:
            self._pack_failed += 1
            return
        self.pub.publish(msg)

    # ══════════════════════════════════════════════════════════
    #  汇报定时器：窗口统计（复用阶段一 stats_node 的思路）
    # ══════════════════════════════════════════════════════════
    def on_report(self):
        now = time.monotonic()
        dt = now - self._last_report if self._last_report else 0.0

        n_rx = self._rx_count - self._rx_base
        n_bad = self._bad_count - self._bad_base
        hz = n_rx / dt if dt > 0 else 0.0
        gap = now - self._last_rx if self._last_rx else -1.0

        self.get_logger().info(
            f'状态={"已连接" if self._connected else "未连接"}'
            f' | 本窗口收帧={n_rx} 坏帧={n_bad}'
            f' | 实测 {hz:.2f} Hz | 距上次数据 {gap:.2f}s'
            f' | 累计收帧={self._rx_count} 坏帧={self._bad_count}'
            f' 丢帧={self._drop_count} 打包失败={self._pack_failed}'
        )

        # 窗口清零 —— 否则你算的是"开机到现在"，不是"最近一段"
        # （阶段一 stats_node 的 TODO-5 就是这个点，别再用 == 了）
        self._last_report = now
        self._rx_base = self._rx_count
        self._bad_base = self._bad_count

    # ══════════════════════════════════════════════════════════
    def destroy_node(self):
        self._stop.set()
        self._close_port()
        self._thread.join(timeout=1.0)
        super().destroy_node()


# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode('serial_bridge')
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


# ═══════════════════════════════════════════════════════════════
# 安装清单（忘了这两条，换台机器就跑不起来）
# ═══════════════════════════════════════════════════════════════
# 1) setup.py 的 entry_points：
#      'serial_bridge     = sensor_bridge.serial_bridge_node:main',
#      'serial_bridge_msg = sensor_bridge.serial_bridge_msg_node:main',   # 阶段三
#    改完必须 colcon build（entry_points 是安装元数据，--symlink-install 救不了）
#
# 2) package.xml：
#      <exec_depend>python3-serial</exec_depend>
#      <depend>sensor_interfaces</depend>        # 阶段三：自定义消息
#
# 跑法：
#   ros2 run sensor_bridge serial_bridge --ros-args \
#        -p port:=/dev/stm32_bridge -p baud:=115200 -p scale:=0.001
#
#   ros2 launch sensor_bridge serial_bridge_msg.launch.py port:=/dev/ttyUSB0
