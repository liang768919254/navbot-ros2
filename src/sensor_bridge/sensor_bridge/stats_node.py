#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
stats_node —— 练习脚手架（档 1：骨架已给，只填算法）

【任务】订阅 /my_sensor/v，每 5 秒汇报一次**这一窗口内**的统计：
        帧数 / 均值 / 标准差。
        禁止用 list.append 存数据（必须 O(1) 内存）。

【怎么用】
  1. 全文搜 "TODO"，一共 5 处。
  2. 按编号顺序填，填一处跑一次（每次修改后重启节点即可，不用重编译）。
  3. 5 处全填完再对照答案（答案在 阶段一测试题与答案.md 的 C1 解析里）。

【跑法】
  终端 A: ros2 run sensor_bridge my_sensor
  终端 B: ros2 run sensor_bridge stats_node
          （sensor_bridge 的 setup.py 里还没注册这个入口，见下方 STAGE 说明）
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class StatsNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('report_period', 5.0)   # 窗口长度（秒）

        # ═══════════════════════════════════════════════════════
        # [区域 A] 累加器：从零开始，每收一帧累一次
        #   提示：要算标准差，你需要"值的平方和"。
        #   σ² = Σv²/n − (Σv/n)²      ← 这个公式不需要存任何一帧数据
        # ═══════════════════════════════════════════════════════
        # TODO-1: 在这里声明 3 个累加器（帧数 / 求和 / 平方和）
        #         帧数从 0 开始，两个求和从 0.0 开始
        # 你的代码：
        self.count = 0
        self.total = 0.0
        self.sq_total = 0.0
        # ═══════════════════════════════════════════════════════

        # ── 订阅者（已写好，别动）────────────────────────────────
        self.sub = self.create_subscription(
            Float32, '/my_sensor/v', self.on_value, 10
        )

        # ── 定时器（已写好，别动）────────────────────────────────
        period = self.get_parameter('report_period').value
        self.timer = self.create_timer(period, self.on_report)

        self.get_logger().info(
            f'stats_node 已启动 | 监听 /my_sensor/v | 每 {period}s 汇报一次'
        )

    # ──────────────────────────────────────────────────────────
    def on_value(self, msg):
        """订阅回调：每收到一帧就被调用一次。这里只做加法，不做除法。"""
        # v = msg.data

        # ═══════════════════════════════════════════════════════
        # TODO-2: 累加三个量（帧数、求和、平方和）
        # 你的代码：
        self.count += 1
        self.total += msg.data
        self.sq_total += msg.data ** 2

        # ═══════════════════════════════════════════════════════

    # ──────────────────────────────────────────────────────────
    def on_report(self):
        """定时器回调：汇报后**清空窗口**，下一次重新开始数。"""
        # ═══════════════════════════════════════════════════════
        # TODO-3: 如果这一窗口一帧都没收到，就 warn 一句并 return
        #         提示：参考 monitor_node.py 的 on_report 写法
        # 你的代码：
        if self.count == 0:
            self.get_logger().warn('尚未收到任何数据，检查发布者是否在运行')
            return

        # ═══════════════════════════════════════════════════════

        # ═══════════════════════════════════════════════════════
        # TODO-4: 算均值、方差、标准差，然后打印
        #   均值 mean = 求和 / 帧数
        #   方差 var  = 平方和 / 帧数 − mean ** 2
        #   标准差 std = math.sqrt(max(0.0, var))
        #              ↑ max(0.0, ...) 是兜底：浮点误差可能让 var 变成 -1e-18，
        #                直接开方会 ValueError: math domain error
        # 你的代码：

        mean = self.total / self.count
        var = self.sq_total / self.count - mean ** 2
        std = math.sqrt(max(0.0, var))   # max 兜底浮点误差，防止负数开方

        self.get_logger().info(f' 均值={mean:.3f}|帧数={self.count} |标准差 = {std:.3f}')

        # ═══════════════════════════════════════════════════════

        # ═══════════════════════════════════════════════════════
        # TODO-5: 清空窗口——把 TODO-1 的三个累加器恢复成初始值
        #         为什么要清空？因为需求要的是"这 5 秒内的统计"，
        #         不是"开机到现在的统计"。
        # 你的代码：
        self.count = 0
        self.total = 0.0
        self.sq_total = 0.0
        # ═══════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    node = StatsNode('stats_node')
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
# STAGE 说明：注册运行入口
# ═══════════════════════════════════════════════════════════════
# 新加的 .py 文件光放进包里还不能用 ros2 run 跑，
# 必须在 setup.py 的 entry_points 里登记一行：
#
#   'stats_node = sensor_bridge.stats_node:main',
#
# 然后 colcon build --symlink-install --packages-select sensor_bridge
# （entry_points 改动必须重新 build，--symlink-install 也救不了它）
#
# 这一条也留给你做——它是 C 组题的隐性考点。
