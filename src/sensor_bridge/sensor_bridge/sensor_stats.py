#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_stats.py —— 订阅端的统计与追踪工具（纯逻辑，不 import ROS）

三个东西：
  · Welford    —— O(1) 内存的均值/方差递推，float32 下也不丢精度
  · SeqTracker —— 用帧号检测丢帧 / 乱序（处理无符号回绕）
  · ChannelSet —— 一组命名通道的窗口统计，省得为每个字段抄一遍代码

【为什么用 Welford 而不是 E[v²] − E[v]²】
  不是"Python 里会炸"（实测 float64 下连偏置 1e7 都稳），
  而是它**精度无关**：同一份代码 float32 / float64 都准。
  嵌入式里 float 是默认选择，这段统计迟早要搬去 C —— 那时换公式的代价更大。
  实测数据见 阶段一测试题与答案.md「附2」。

【为什么不存 list】
  数据频率 × 运行时长 是没有上界的。O(1) 内存才能一直跑下去。
  阶段一 A4 已经考过这个点，这里是它的"进阶版实现"。
"""

import math
from typing import Dict, Iterable, List, Optional, Tuple

U32 = 1 << 32
HALF_U32 = 1 << 31


class Welford:
    """在线均值/方差（Welford 递推）。每帧 O(1)，无数组。"""

    __slots__ = ('n', 'mean', 'm2', 'vmin', 'vmax')

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0          # 二阶中心矩的累加量：Σ(v − mean)²
        self.vmin = None
        self.vmax = None

    def update(self, v: float):
        self.n += 1
        d = v - self.mean                 # 与当前均值的偏差 —— 一直是小量
        self.mean += d / self.n
        self.m2 += d * (v - self.mean)    # 关键：累加的是"偏差×偏差"，不是 v²
        if self.vmin is None or v < self.vmin:
            self.vmin = v
        if self.vmax is None or v > self.vmax:
            self.vmax = v

    @property
    def variance(self) -> float:
        return self.m2 / self.n if self.n else 0.0

    @property
    def std(self) -> float:
        # max 兜底：极端数值下 m2 可能出现 -1e-18，开方会 ValueError
        return math.sqrt(max(0.0, self.variance)) if self.n else 0.0


class SeqTracker:
    """用帧号检测丢帧与乱序。

    为什么不能用「差值 != 1 就算丢帧」：
      帧号是 uint32 / uint16，会回绕。255 → 0 是正常的，
      差值直接相减会算出 -255，被当成丢了一大堆。
      正确做法是按无符号环绕距离判断。
    """

    __slots__ = ('last', 'lost', 'out_of_order', 'dup')

    def __init__(self):
        self.reset()

    def reset(self):
        self.last: Optional[int] = None
        self.lost = 0
        self.out_of_order = 0
        self.dup = 0

    def update(self, seq: int, wrap: int = U32):
        seq %= wrap
        if self.last is None:
            self.last = seq
            return
        gap = (seq - self.last) % wrap
        if gap == 1:
            pass                                  # 正常连续
        elif gap == 0:
            self.dup += 1                         # 重复帧
        elif gap < wrap // 2:
            self.lost += gap - 1                  # 向前跳：真丢帧
        else:
            self.out_of_order += 1                # 向后跳：乱序或重启
        self.last = seq

    def snapshot(self) -> Tuple[int, int, int]:
        return self.lost, self.out_of_order, self.dup

    def reset_counters(self):
        self.lost = self.out_of_order = self.dup = 0


class ChannelSet:
    """一组命名通道的窗口统计。

    用法：
        cs = ChannelSet(['value', 'temperature'])
        cs.update('value', 1650.0)
        cs.update('temperature', 25.0)
        cs.report()   -> {'value': (n, mean, std), ...}
        cs.reset()
    """

    def __init__(self, names: Iterable[str]):
        self.names: List[str] = list(names)
        self._ch: Dict[str, Welford] = {n: Welford() for n in self.names}

    def update(self, name: str, v: float):
        ch = self._ch.get(name)
        if ch is None:                     # 没登记的通道：静默忽略，不崩
            return
        ch.update(v)

    def report(self) -> Dict[str, Welford]:
        return self._ch

    def reset(self):
        for ch in self._ch.values():
            ch.reset()


if __name__ == '__main__':
    ok = True

    # ── 1. Welford：无噪声正弦的方差应当 ≈ A²/2 ──────────────
    A = 1.0
    w = Welford()
    for i in range(50):
        w.update(A * math.sin(2 * math.pi * 0.5 * (0.37 + 0.1 * i)))
    print(f'[1] 正弦 50 帧: mean={w.mean:+.4f} var={w.variance:.4f} std={w.std:.4f} '
          f'min={w.vmin:.3f} max={w.vmax:.3f}')
    print(f'    理论:       var≈{A*A/2:.4f}              std≈{A/math.sqrt(2):.4f}')
    if abs(w.variance - 0.4953) > 1e-3:
        print('    FAIL: 方差与理论值不符'); ok = False

    # ── 2. SeqTracker：连续 / 跳号丢帧 / 无符号回绕 ──────────
    t = SeqTracker()
    for s in range(100, 107):            # 连续 7 帧
        t.update(s)
    assert (t.lost, t.out_of_order, t.dup) == (0, 0, 0), t.snapshot()
    for s in (109, 110):                 # 跳过 107、108 → 丢 2 帧
        t.update(s)
    print(f'[2] 跳号 2 帧后: 丢帧={t.lost} 乱序={t.out_of_order} 重复={t.dup}')
    if t.lost != 2:
        print('    FAIL: 丢帧检测不对'); ok = False

    # 单独测回绕：255 → 0 必须算"连续"，不能算"倒退 255"
    t2 = SeqTracker()
    for s in (254, 255, 0, 1, 2):
        t2.update(s, wrap=256)
    print(f'[3] uint8 回绕 255→0: 丢帧={t2.lost} 乱序={t2.out_of_order}')
    if (t2.lost, t2.out_of_order) != (0, 0):
        print('    FAIL: 回绕处理不对（把正常回绕当成了丢帧/乱序）'); ok = False

    # 真倒退（板子重启，帧号从 0 重来）应当记成乱序而不是丢一堆帧
    t3 = SeqTracker()
    for s in (5000, 5001, 3):
        t3.update(s)
    print(f'[4] 板子重启(帧号倒退): 丢帧={t3.lost} 乱序={t3.out_of_order}')
    if t3.out_of_order != 1 or t3.lost != 0:
        print('    FAIL: 重启误判成丢帧'); ok = False

    print('\n' + ('全部自检通过' if ok else '存在失败项'))
