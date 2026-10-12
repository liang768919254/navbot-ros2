#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_data_codec.py —— 串口字段组 ↔ ROS 自定义消息 的纯逻辑打包

【为什么单独拆出来（沿用 serial_protocol.py 的同一条原则）】
  这个模块**不 import ROS、不 import 串口**：消息类型由调用方注入（msg_cls 参数）。
  于是它既能被节点用，也能被单测用（单测把真的 SensorData 类传进来即可）。
  纯逻辑和 IO/框架解耦，就能被测试、被复用、被搬去别的项目。

【协议字段布局】—— 串口这一端的格式，自定义消息解决不了，必须写清楚
  v1:  <value>                                    1 个字段
  v2:  <seq>,<value>,<raw_adc>,<temperature>      4 个字段
  v3:  <seq>,<value>,<raw_adc>,<temperature>,<mcu_tick_ms>   5 个字段

  ★ 2 个或 3 个字段 = 协议错误 → 整帧丢弃，返回 None。
    为什么不做"宽松猜测"？因为猜错的代价是下游拿到一个字段错位的"看起来正常"的值。
    宁可丢帧 + 计数，也不要错位数据。

【一个必须讲清楚的事实】
  自定义消息让 PC 内部和下游"格式自描述"了（ros2 interface show 就能看到字段），
  但**串口那一端仍然要靠字段个数猜**——MCU 只吐一行 ASCII，它不知道什么是 ROS。
  所以：**协议文档必须存在**，且和 .msg 文件一起版本化。
  想彻底解决，得在串口协议里加帧头/版本号（那是 CAN 阶段的事）。
"""

from typing import List, Optional, Sequence, Tuple

# ── 各版本的字段顺序（顺序即协议里的顺序）──────────────────────
LAYOUT_V1: Tuple[str, ...] = ('value',)
LAYOUT_V2: Tuple[str, ...] = ('seq', 'value', 'raw_adc', 'temperature')
LAYOUT_V3: Tuple[str, ...] = ('seq', 'value', 'raw_adc', 'temperature', 'mcu_tick_ms')

# 消息里能填的字段白名单（多出来的协议字段会被忽略，不报错）
KNOWN_FIELDS = ('seq', 'value', 'raw_adc', 'temperature', 'mcu_tick_ms')


def detect_layout(fields: Sequence[float]) -> Tuple[Optional[int], Optional[Tuple[str, ...]]]:
    """按字段个数判定协议版本。

    Returns:
        (版本号, 布局元组)；不认识就返回 (None, None)。
    """
    n = len(fields)
    if n == 1:
        return 1, LAYOUT_V1
    if n == 4:
        return 2, LAYOUT_V2
    if n >= 5:
        return 3, LAYOUT_V3          # 多出来的字段忽略（向前兼容）
    return None, None                # 2 / 3 个字段：协议错误


def pack_sensor_data(msg_cls,
                     fields: Sequence[float],
                     stamp,
                     frame_id: str = 'sensor_link',
                     value_scale: float = 1.0,
                     temp_scale: float = 1.0,
                     seq_fallback: int = 0):
    """把一行串口字段打包成 SensorData 消息。

    Args:
        msg_cls:      消息类（调用方注入，比如 sensor_interfaces.msg.SensorData）
        fields:       一行解析出来的数值（未缩放）
        stamp:        builtin_interfaces/Time，塞进 header.stamp
        frame_id:     坐标名，塞进 header.frame_id
        value_scale:  主值缩放（整数化毫伏 → 工程量；默认 1.0 = 保持毫伏）
        temp_scale:   温度缩放（协议里温度是 int(℃×100)，所以默认传 0.01）
        seq_fallback: 协议没有帧号时（v1）用这个值当帧号

    Returns:
        消息对象；字段布局不认识时返回 None。
    """
    version, layout = detect_layout(fields)
    if version is None:
        return None

    # 字段名 → 值 的映射；缺失的字段（v1 没有 raw/temp）用 0 填
    d = dict(zip(layout, fields))

    msg = msg_cls()

    # Header：时间 + 坐标。这两个字段是所有 ROS2 生态工具的"公共接口"
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    # 帧号：v1 没有，用 PC 侧自增兜底 —— 下游的丢帧检测逻辑不用区分协议版本
    msg.seq = int(d.get('seq', seq_fallback)) & 0xFFFFFFFF

    msg.value = float(d.get('value', 0.0) * value_scale)
    msg.temperature = float(d.get('temperature', 0.0) * temp_scale)
    msg.raw_adc = int(d.get('raw_adc', 0.0))
    msg.mcu_tick_ms = int(d.get('mcu_tick_ms', 0.0))

    return msg


def format_line_v3(seq: int, value_mv: int, raw_adc: int,
                   temp_c_x100: int, mcu_tick_ms: int, eol: str = '\n') -> bytes:
    """组装一行 v3 协议数据（发送端 / 测试脚本共用）。

    全部用整数：MCU 侧因此完全不必碰浮点 printf
    （Cortex-M 上浮点格式化要么要开 MicroLIB / -u_printf_float，要么静默失效，还慢）。
    """
    line = f'{seq},{value_mv},{raw_adc},{temp_c_x100},{mcu_tick_ms}'
    return (line + eol).encode('ascii')


if __name__ == '__main__':
    # 不装 ROS 也能看布局判定对不对
    samples = [
        [1.5],
        [1024, 1650, 2048, 2500],
        [1024, 1650, 2048, 2500, 187340],
        [1024, 1650, 2048, 2500, 187340, 99],   # 多余字段：忽略
        [1024, 1650],                            # 2 个字段：协议错误
        [1024, 1650, 2048],                      # 3 个字段：协议错误
    ]
    for s in samples:
        print(f'{str(s):<46} -> {detect_layout(s)}')
