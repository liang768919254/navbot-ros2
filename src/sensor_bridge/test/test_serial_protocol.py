#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_serial_protocol.py —— parse_line() 的单测（脱离 ROS、脱离串口）

【怎么用】
  方式 1（不装任何东西，直接跑）：
      python3 test_serial_protocol.py
  方式 2（有 pytest）：
      python3 -m pytest test_serial_protocol.py -v

【正式版放哪】
  已经复制到 ~/ros_study/src/sensor_bridge/test/ 下。
  下面的 import 做了三路回退，所以从哪个目录跑都找得到 serial_protocol：
      ① from sensor_bridge.serial_protocol import ...   ← source install 之后
      ② from serial_protocol import ...                 ← 源码树内的包目录
      ③ 教程附件目录（../参考源码）                      ← 还没复制到工程里时

  ⚠️ 2026-09-22 踩过的坑：这个文件原先只有 ③ 那一条 sys.path（指向教程附件目录），
     复制进工程后 `../参考源码` 根本不存在 → ModuleNotFoundError。
     而 ROS2 环境里 pytest 挂了 launch_testing 插件，它会**先 import 每个测试模块**去找
     launch test 入口，于是 import 失败被包成一长串 pluggy 堆栈抛出来，
     看起来像 pytest 坏了。遇到这种堆栈，**直接翻到最底下几行**看真正的错误。

  然后 colcon test --packages-select sensor_bridge 就会带上它。

【为什么值得写】
  串口测试最难的地方是「造出异常」。真板子上你没法精确制造
  "半包"、"乱码"、"超长行"；但在这里，它们就是几个 bytes 字面量。
  这就是「把纯逻辑从 IO 和框架里剥出来」的全部好处。
"""

import os
import sys

# ── 三路候选目录，按优先级塞进 sys.path ──────────────────────────
#   注意：本文件在 <pkg>/test/ 下，所以 .. = <pkg>，../.. = src/
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.normpath(os.path.join(_HERE, '..', '..')),          # ① 包父目录
           os.path.normpath(os.path.join(_HERE, '..', 'sensor_bridge')),  # ② 包内目录
           os.path.normpath(os.path.join(_HERE, '..', '参考源码'))):     # ③ 教程附件
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:                                    # ① 优先：走正式的包路径
    from sensor_bridge.serial_protocol import format_line, parse_line
except ImportError:                     # ②③ 回退：直接拿模块
    from serial_protocol import format_line, parse_line  # noqa: E402


# ══════════════════════════════════════════════════════════════
#  正常情况
# ══════════════════════════════════════════════════════════════

def test_v1_plain():
    assert parse_line(b'1.234\n') == [1.234]


def test_v1_crlf():
    r"""Windows/STM32 习惯发 \r\n，strip 必须处理掉 \r。"""
    assert parse_line(b'1234\r\n') == [1234.0]


def test_v1_negative_and_exponent():
    assert parse_line(b'-0.5\n') == [-0.5]
    assert parse_line(b'1e3\n') == [1000.0]


def test_v2_multifield():
    assert parse_line(b'1024,1234,1533,27500\n') == [1024.0, 1234.0, 1533.0, 27500.0]


def test_leading_trailing_spaces():
    assert parse_line(b'  1.5 , 2.5  \r\n') == [1.5, 2.5]


# ══════════════════════════════════════════════════════════════
#  脏数据 —— 全部必须返回 None（整行丢弃，不做部分接受）
# ══════════════════════════════════════════════════════════════

def test_empty_bytes():
    """readline 超时返回空 —— 这是最常见的一类输入，必须静默"""
    assert parse_line(b'') is None


def test_blank_lines():
    assert parse_line(b'\n') is None
    assert parse_line(b'\r\n') is None
    assert parse_line(b'   \r\n') is None


def test_boot_banner():
    """板子上电横幅：真机上一定会遇到，绝不能让它进话题"""
    assert parse_line(b'STM32 serial bridge ready | proto=v2\r\n') is None


def test_garbage_bytes():
    """非 ASCII 字节（波特率抖动 / 电气噪声）"""
    assert parse_line(b'\x1f\x9c\x03\x8a\r\n') is None


def test_partial_field_broken():
    """中间字段坏了 → 整行丢弃。半帧数据比没数据更危险"""
    assert parse_line(b'1,2,x,4\n') is None


def test_empty_field():
    assert parse_line(b'1,,3\n') is None
    assert parse_line(b',,,,\n') is None


def test_oversize_line_dropped():
    """超长行必须被闸门挡住，不能进到 decode / split"""
    long_line = b'9' * 600 + b'\n'
    assert parse_line(long_line) is None
    # 刚好卡在上限内的应该正常通过
    assert parse_line(b'1' * 10 + b'\n') == [float('1' * 10)]


def test_max_len_is_configurable():
    assert parse_line(b'1,2,3\n', max_len=4) is None      # 6 字节 > 4
    assert parse_line(b'1,2,3\n', max_len=16) == [1.0, 2.0, 3.0]


# ══════════════════════════════════════════════════════════════
#  format_line（发送端共用）
# ══════════════════════════════════════════════════════════════

def test_format_line_int_no_dot():
    assert format_line([1024, 1234]) == b'1024,1234\n'


def test_format_line_float_trimmed():
    assert format_line([1.5, 27.25]) == b'1.5,27.25\n'


def test_format_parse_roundtrip():
    """组装 → 解析 应该回到原值（浮点允许 1e-6 误差）"""
    src = [1024, 1234.5, 1533, -27.25]
    got = parse_line(format_line(src))
    assert got is not None and len(got) == len(src)
    for a, b in zip(src, got):
        assert abs(float(a) - b) < 1e-6


# ══════════════════════════════════════════════════════════════
#  无 pytest 也能跑
# ══════════════════════════════════════════════════════════════

def _run_all():
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith('test_') and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f'  PASS  {name}')
        except AssertionError as e:
            failed.append(name)
            print(f'  FAIL  {name}  {e}')
        except Exception as e:
            failed.append(name)
            print(f'  ERROR {name}  {type(e).__name__}: {e}')

    print(f'\n{len(tests) - len(failed)}/{len(tests)} passed')
    if failed:
        print('失败：' + ', '.join(failed))
        return 1
    return 0


if __name__ == '__main__':
    print('parse_line / format_line 单测（不需要 ROS、不需要串口）\n')
    sys.exit(_run_all())
