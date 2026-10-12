#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge_msg_node.py —— 阶段三：把串口桥的出口从 Float32 换成自定义 SensorData

这个文件只有 60 行，因为**它不是一条新的桥**——
它是 `SerialBridgeNode` 的子类，只覆盖两个钩子：

    _create_publisher()  → 换成发布 SensorData
    _pack_msg()          → 把「最新一帧字段」打包成 SensorData

线程模型、断线自愈、参数、统计、陈旧数据保护……全部继承，**一行都没重写**。
这就是阶段二把协议字段先定下来的回报：
阶段三不是"重新做一个项目"，而是"给已有的桥换一个出口"。

【两个入口，同一条桥】
    ros2 run sensor_bridge serial_bridge        # std_msgs/Float32（阶段二行为，未变）
    ros2 run sensor_bridge serial_bridge_msg    # sensor_interfaces/SensorData（阶段三）

【新增的两个参数】（在 EXTRA_PARAMS 里登记，基类负责声明）
    frame_id    string  'sensor_link'   写进 header.frame_id，TF2 查变换用
    temp_scale  double  0.01            协议里温度是 int(℃×100)，乘回摄氏度

为什么 temp_scale 默认 0.01：协议用定点整数传温度（2500 → 25.00℃），
这样 MCU 侧完全不用碰浮点 printf —— 沿用阶段二定下的那条规矩。
"""

import rclpy
from rclpy.executors import ExternalShutdownException

from sensor_bridge.sensor_data_codec import pack_sensor_data
from sensor_bridge.serial_bridge_node import SerialBridgeNode
from sensor_interfaces.msg import SensorData


class SerialBridgeMsgNode(SerialBridgeNode):

    # 子类额外需要的参数（基类在 __init__ 里统一声明）
    EXTRA_PARAMS = {
        'frame_id': 'sensor_link',
        'temp_scale': 0.01,
    }

    # ── 钩子 1：换成自定义消息类型 ────────────────────────────
    def _create_publisher(self, topic):
        return self.create_publisher(SensorData, topic, 10)

    # ── 钩子 2：把"最新一帧"打包成 SensorData ─────────────────
    def _pack_msg(self, stamp):
        fields = self._latest_fields
        if fields is None:
            return None

        return pack_sensor_data(
            SensorData,
            fields=fields,
            stamp=stamp,
            frame_id=self.get_parameter('frame_id').value,
            # value 用基类同一个 scale 参数，保持两个入口行为一致
            value_scale=float(self.get_parameter('scale').value),
            temp_scale=float(self.get_parameter('temp_scale').value),
            # v1 协议没有帧号，用基类维护的自增值兜底
            seq_fallback=self._latest_seq,
        )

    def _msg_type_name(self):
        return 'sensor_interfaces/msg/SensorData'


# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeMsgNode('serial_bridge_msg')
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
# 跑法
# ═══════════════════════════════════════════════════════════════
# ros2 run sensor_bridge serial_bridge_msg --ros-args \
#      -p port:=/dev/stm32_bridge -p frame_id:=sensor_link -p temp_scale:=0.01
#
# 看话题类型（应当是 sensor_interfaces/msg/SensorData）：
#   ros2 topic info /sensor/value -v
#
# 看结构化内容：
#   ros2 topic echo /sensor/value --once
#
# 只画 value 字段的曲线（注意要写到字段级）：
#   rqt_plot 里填： /sensor/value/value
