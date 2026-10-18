"""2.10 Hausdorff 距离（含 MOD 变体）—— 两组点集的"最远最近点"。

公式::

    h(A,B) = max_{a∈A} min_{b∈B} ‖a-b‖     （有向距离）
    H(A,B) = max(h(A,B), h(B,A))           （对称距离）

MOD 变体（modified Hausdorff）把 max 换成高分位数（默认 95%），缓解单点
支配——**缓解不等于根除**：离群点占比一旦低于分位数参数会被直接吞掉
（max 版至少不漏报），用 MOD 时离群比例必须远低于分位数参数。

**不保序**：把轨迹当无序点堆，时序完全相反的两条轨迹 H=0——只能做空间
覆盖的辅助指标，不能当轨迹过程对比的主指标。

本项目使用范围：轨迹对比的空间覆盖**辅助**口径（主口径 RMSE+Fréchet）；
两次仿真的散布/覆盖范围比较。
"""

from __future__ import annotations

import math
from typing import Sequence

Point = Sequence[float]


def _dist(p: Point, q: Point) -> float:
    return math.sqrt(sum((pi - qi) ** 2 for pi, qi in zip(p, q)))


def directed_hausdorff(a: Sequence[Point], b: Sequence[Point]) -> float:
    """有向距离 h(A,B)：A 中每点到 B 的最近距离取最大。"""
    if not a or not b:
        raise ValueError("空点集")
    return max(min(_dist(p, q) for q in b) for p in a)


def hausdorff(a: Sequence[Point], b: Sequence[Point]) -> float:
    """对称 Hausdorff 距离 H = max(h(A,B), h(B,A))。"""
    return max(directed_hausdorff(a, b), directed_hausdorff(b, a))


def _quantile(xs: Sequence[float], q: float) -> float:
    """线性插值分位数（numpy 'linear' 口径），标准库实现。"""
    if not 0.0 <= q <= 1.0:
        raise ValueError(f"q∈[0,1], 得到 {q}")
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    pos = q * (len(s) - 1)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)


def mod_hausdorff(
    a: Sequence[Point],
    b: Sequence[Point],
    *,
    q: float = 0.95,
) -> float:
    """MOD 变体：两个方向的逐点最近距离各取 q 分位数，再取 max。"""
    if not a or not b:
        raise ValueError("空点集")
    ha = _quantile([min(_dist(p, qq) for qq in b) for p in a], q)
    hb = _quantile([min(_dist(p, qq) for qq in a) for p in b], q)
    return max(ha, hb)
