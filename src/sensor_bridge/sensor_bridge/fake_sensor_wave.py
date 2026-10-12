#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fake_sensor_wave.py —— 练习脚手架（档 1：骨架已给，只填算法）

【任务】给传感器加一个字符串参数 wave，支持三种波形，
        并且能用 ros2 param set 在运行中**立刻切换**：
            wave='sin'     正弦（= 你已有的行为）
            wave='square'  方波：sin>=0 输出 +amplitude，否则 -amplitude
            wave='saw'     锯齿波：把相位折算到 [0,1) 再映射到 [-1,1]

【关键概念】参数分两类，这个题专门练第二类：
    · 结构参数：rate —— 决定定时器周期，只在 __init__ 读一次
    · 数据参数：amplitude / freq / noise / wave —— 每帧现读，改了立刻生效
  你之前想改 freq 不生效、想改 noise 不生效，根因都是把数据参数
  在 __init__ 里读死了。

【怎么用】搜 "TODO"，共 4 处，按编号填，填完一处跑一次。

【跑法】见文件末尾 STAGE 说明。
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class FakeSensorWaveNode(Node):

    def __init__(self, name):
        super().__init__(name)

        # ── 结构参数：只读一次，决定定时器 ──────────────────────
        self.declare_parameter('rate', 10.0)
        self.rate = self.get_parameter('rate').value
        self.step = 1.0 / self.rate
        self.timer = self.create_timer(self.step, self.on_timer)

        # ── 数据参数：每帧现读 ──────────────────────────────────
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('freq', 0.5)
        self.declare_parameter('noise', 0.0)
        self.declare_parameter('wave', 'sin')      # ← 新增

        self.pub = self.create_publisher(Float32, '/my_sensor/v', 10)

        self.t = 0.0

    # ──────────────────────────────────────────────────────────
    def on_timer(self):
        amplitude = self.get_parameter('amplitude').value
        freq = self.get_parameter('freq').value
        noise = self.get_parameter('noise').value
        wave = self.get_parameter('wave').value

        # 相位：不带单位，留给你在各波形里自己折算
        phase = freq * self.t          # 单位是"周期数"，一圈 = 1.0

        # ═══════════════════════════════════════════════════════
        # TODO-1: 用 phase 算出三种波形，赋给 base（取值范围都在 [-1, 1]）
        #   'sin'    : base = math.sin(2 * math.pi * phase)
        #   'square' : base = 1.0 if math.sin(2*math.pi*phase) >= 0 else -1.0
        #   'saw'    : 把 phase 折算成 [0.0, 1.0) 的小数部分 frac，
        #              再映射：base = 2.0 * frac - 1.0
        #              （提示：phase - math.floor(phase) 就是 frac）
        #   未知波形怎么办？base = 0.0，并 warn 一次（见 TODO-2 下面的提示）
        # 你的代码：
        if wave == 'sin':
            base = math.sin(2 * math.pi * phase)
        elif wave == 'square':
            base = 1.0 if math.sin(2 * math.pi * phase) >= 0 else -1.0
        elif wave == 'saw':
            frac = phase - math.floor(phase)
            base = 2.0 * frac - 1.0
        else:
            base = 0.0
            self.get_logger().warn(f'未知波形 {wave!r}，输出 0', throttle_duration_sec=5.0)

        # ═══════════════════════════════════════════════════════

        # ═══════════════════════════════════════════════════════
        # TODO-2: 加上振幅与噪音，组成最终值
        #   注意顺序：先缩放再叠噪音（否则噪音也被 amplitude 放大）
        #   value = amplitude * base + random.gauss(0.0, noise)
        # 你的代码：
        value = amplitude * base + random.gauss(0.0, noise)
        # ═══════════════════════════════════════════════════════

        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)

        self.get_logger().info(
            f'[{wave}] value={value:+.3f}', throttle_duration_sec=1.0
        )

        # ═══════════════════════════════════════════════════════
        # TODO-3: 推进时间轴
        #   用 self.t += self.step，不要用 time.time()。
        #   为什么？因为 time.time() 会被系统调度抖动干扰，
        #   step 是理论周期，自累加出来的波形才干净。
        # 你的代码：
        self.t += self.step
        # ═══════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorWaveNode('my_sensor_wave')
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
# STAGE 说明：注册 + 验证
# ═══════════════════════════════════════════════════════════════
# 1) setup.py 的 entry_points 加一行：
#      'my_sensor_wave = sensor_bridge.fake_sensor_wave:main',
# 2) colcon build --symlink-install --packages-select sensor_bridge
# 3) 跑起来后开另一个终端，实测运行时切换：
#
#      ros2 param set my_sensor_wave wave square
#      ros2 param set /my_sensor_wave wave saw
#      ros2 param set /my_sensor_wave wave sin
#
#    ★ 验收标准：每敲一条，节点日志里的波形名**立刻**变，
#      且不需要重启节点。如果没变，说明你把 wave 提前读死了。
#
# 【进阶自检】把 wave 改成不认识的值（如 'triangle'）会怎样？
#   理想行为：warn 一次并输出 0。你可以用 TODO-2 下面的位置实现。
