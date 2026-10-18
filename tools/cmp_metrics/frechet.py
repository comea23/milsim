"""2.3 离散弗雷歇距离 —— 顺序约束下的最坏偏差（"狗绳距离"）。

数据前提：两轨迹起止点对应、整体形状对比；关心"全程任何时候都不许偏太远"
（弹道走廊、安全边界）。人沿一条曲线走、狗沿另一条走，都从头到尾不许回头，
狗绳的最短可能长度即弗雷歇距离 F——**只由最坏时刻决定**（DTW 是平均贴合）。

递推（动态规划，点距取欧氏）::

    dF(i,j) = max( d(i,j), min(dF(i-1,j), dF(i,j-1), dF(i-1,j-1)) )

库口径配 sigmoid 把距离转 [0,1] 相似度：``sim = 1/(1+e^{5F})``。
注意库口径特性：完全重合（F=0）也只有 sim=0.5，判读时以此为基准。

本项目使用范围：轨迹对比的**辅口径**（主口径 RMSE 看平均精度，弗雷歇看
最坏偏差）。直接吃原始点列，天然支持两边点数不等、采样节奏不同——AFSIM
输出不保证 1s 步长时无需插值即可对比。
"""

from __future__ import annotations

import math
from typing import Sequence

Point = Sequence[float]


def _dist(p: Point, q: Point) -> float:
    return math.sqrt(sum((pi - qi) ** 2 for pi, qi in zip(p, q)))


def frechet_distance(p: Sequence[Point], q: Sequence[Point]) -> float:
    """离散弗雷歇距离（欧氏点距）。"""
    n, m = len(p), len(q)
    if n == 0 or m == 0:
        raise ValueError("空序列")
    # ca[i][j]：只走前缀 p[1..i]、q[1..j] 所需最小绳长
    ca = [[math.inf] * m for _ in range(n)]
    for i in range(n):
        for j in range(m):
            d = _dist(p[i], q[j])
            if i == 0 and j == 0:
                ca[i][j] = d
            elif i == 0:
                ca[i][j] = max(d, ca[i][j - 1])
            elif j == 0:
                ca[i][j] = max(d, ca[i - 1][j])
            else:
                ca[i][j] = max(d, min(ca[i - 1][j], ca[i][j - 1], ca[i - 1][j - 1]))
    return ca[n - 1][m - 1]


def frechet_similarity(f: float) -> float:
    """sigmoid 相似度 sim = 1/(1+e^{5F})（库口径，重合=0.5）。"""
    if f < 0.0:
        raise ValueError(f"距离非负, 得到 {f}")
    return 1.0 / (1.0 + math.exp(5.0 * f))
