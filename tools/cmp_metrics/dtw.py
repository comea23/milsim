"""2.2 动态时间规整（DTW）—— 时间轴弹性对齐的累积代价。

数据前提：长度不等、节奏/采样率不一致、但形状需要整体贴合的序列。
两条序列节奏不一致时允许"拉伸压缩"后逐点配对，配对由动态规划求总代价最小。
时间不倒流：路径只许右/下/斜下，配对保序。

递推（一维标量距离，多维可传自定义 dist）::

    D(i,j) = d(i,j) + min(D(i-1,j), D(i,j-1), D(i-1,j-1))

返回累积代价 D(n,m) 与"平均每步代价"（除以 max(n,m)，便于跨长度比较）。

本项目使用范围：**当前未启用**。milsim 与 AFSIM 均可插值对齐到统一栅格，
RMSE+Fréchet 已覆盖主口径；仅当未来引入"节奏本身就不同、且不宜插值"
的对比（如事件驱动的稀疏采样流）时启用。
"""

from __future__ import annotations

from typing import Callable, Sequence

import math


def dtw_distance(
    x: Sequence,
    y: Sequence,
    dist: Callable[[object, object], float] | None = None,
) -> tuple[float, float]:
    """返回 ``(累积代价, 平均每步代价)``。

    ``dist`` 缺省为一维标量的 ``|x_i - y_j|``；多维点传欧氏距离等。
    """
    n, m = len(x), len(y)
    if n == 0 or m == 0:
        raise ValueError("空序列")
    d = dist if dist is not None else (lambda p, q: abs(p - q))
    # 经典边界：D(0,0)=0、首行/首列其余 inf ⇒ 首行首列自动累加成前缀和；
    # 之后每行 cur[0] 恒 inf，首列只能从上方来。
    inf = math.inf
    prev: list[float] = [0.0] + [inf] * m
    for i in range(1, n + 1):
        cur: list[float] = [inf] * (m + 1)
        for j in range(1, m + 1):
            cost = d(x[i - 1], y[j - 1])
            cur[j] = cost + min(prev[j], cur[j - 1], prev[j - 1])
        prev = cur
    total = prev[m]
    return total, total / max(n, m)
