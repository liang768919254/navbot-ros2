#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
serial_protocol.py —— 串口协议解析/组装（纯函数，不依赖 ROS、不依赖串口）

【为什么单独拆出来】
  解析逻辑是「纯计算」：给一段字节，返回一组数或 None。
  把它和 ROS、和 pyserial 解耦之后，它就能：
    · 脱离 ROS 单独测（不需要 roscore/节点，也不需要真串口）
    · 被发送端脚本、被 PC 上位机、被离线回放工具复用
  这是阶段二最值钱的工程习惯：把纯逻辑从「IO + 框架」里剥出来。

【协议】
  v1:  <value>\n                          例: 1234\n
  v2:  <seq>,<value>,<raw_adc>,<temp>\n    例: 1024,1234,1533,27500\n

  行尾统一 \n（允许 \r\n，strip 会处理掉 \r）
  空行、无 \n 的残行、上电横幅、乱码 → 一律返回 None（整行丢弃，不做部分接受）
"""

from typing import List, Optional

DEFAULT_MAX_LEN = 256


# ──────────────────────────────────────────────────────────────
def parse_line(raw: bytes, max_len: int = DEFAULT_MAX_LEN) -> Optional[List[float]]:
    """
    把串口读到的一行原始字节解析成数值列表。

    Args:
        raw:     readline() 读到的原始字节（含行尾）
        max_len: 单行长度上限，超过则整行丢弃（防止异常输入吃内存）

    Returns:
        数值列表，例如 [1024.0, 1234.0, 1533.0, 27.5]；
        任何一步失败都返回 None —— 调用方只需要判断 None，不用关心失败原因。
    """
    # ① 长度闸门：先挡超长行。必须在 decode 之前，否则已经花掉一次内存分配
    if not raw or len(raw) > max_len:
        return None

    # ② 解码：errors='ignore' 让乱码变成「缺字符」而不是抛异常。
    #    串口上出现非 ASCII 字节是常态（上电横幅、波特率抖动、电气噪声）。
    text = raw.decode('ascii', errors='ignore').strip()

    # ③ 空行 / 纯空白 / 被 ignore 掉之后什么都不剩
    if not text:
        return None

    # ④ 切字段。任一字段不是数 → 整行丢弃。
    #    为什么不做「部分接受」？因为半帧数据比没数据更危险：
    #    下游会以为收到了真实值，其实是残缺值。
    fields: List[float] = []
    for token in text.split(','):
        token = token.strip()
        if not token:
            return None
        try:
            fields.append(float(token))
        except ValueError:
            return None

    return fields if fields else None


# ──────────────────────────────────────────────────────────────
def format_line(values, eol: str = '\n') -> bytes:
    """
    把一组数值拼成一行 ASCII（发送端 / 测试脚本共用）。

    整数原样输出（不补 .0），浮点保留 6 位有效数字后再去掉尾随 0。
    """
    parts = []
    for v in values:
        if isinstance(v, bool):
            raise TypeError('bool 不是有效字段类型')
        if isinstance(v, int):
            parts.append(str(v))
        else:
            s = f'{float(v):.6f}'.rstrip('0').rstrip('.')
            parts.append(s if s not in ('', '-') else '0')
    return (','.join(parts) + eol).encode('ascii')


# ──────────────────────────────────────────────────────────────
def is_v2(fields: List[float]) -> bool:
    """
    靠字段个数猜协议版本 —— 这只是权宜之计，看见它的别扭就对了。

    正式做法是让消息自带描述（阶段三的自定义 SensorData.msg 就是干这个的）：
    格式自描述之后，接收端就不用「猜」了。
    """
    return len(fields) >= 2


if __name__ == '__main__':
    # 手搓几行验一下，不做测试也能有直觉
    samples = [
        b'1.234\r\n',
        b'1024,1234,1533,27.5\n',
        b'STM32 Ready! boot v1.0\r\n',      # 上电横幅
        b'\r\n',                             # 空行
        b'',                                 # 超时（readline 返回空）
        b'\x1f\x9c\x03\r\n',                 # 乱码
        b'1,2,x,4\n',                        # 中间字段坏了
        b'1,2,3\n' + b'A' * 600 + b'\n',     # 超长行
    ]
    for s in samples:
        r = parse_line(s)
        print(f'{s[:40]!r:48} -> {r}')
