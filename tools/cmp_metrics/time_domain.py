"""2.1 时域误差指标 —— 等长对齐序列的逐点误差汇总。

数据前提：两条序列点数相同、已按时间对齐、量纲一致。
记待评序列 a、基准序列 b，逐点误差 ``e_i = b_i - a_i``。

本项目使用范围：milsim 与 AFSIM 输出经 ``align`` 插值对齐到同一时间栅格后，
位置/航向/速度等轨迹分量的数值精度主口径（MAE/RMSE/MaxAE）；
相对口径（TIC/RMAE/RRMSE）见 2.7/2.8/2.9 各文件。
"""

from __future__ import annotations

import math
from typing import Sequence

Number = float


def _check(a: Sequence[Number], b: Sequence[Number]) -> None:
    if len(a) != len(b):
        raise ValueError(f"等长对齐前提不成立: len(a)={len(a)}, len(b)={len(b)}")
    if not a:
        raise ValueError("空序列")


def mae(a: Sequence[Number], b: Sequence[Number]) -> Number:
    """平均绝对误差 mean(|e_i|)。"""
    _check(a, b)
    return sum(abs(bi - ai) for ai, bi in zip(a, b)) / len(a)


def mse(a: Sequence[Number], b: Sequence[Number]) -> Number:
    """均方误差 mean(e_i^2)。"""
    _check(a, b)
    return sum((bi - ai) ** 2 for ai, bi in zip(a, b)) / len(a)


def rmse(a: Sequence[Number], b: Sequence[Number]) -> Number:
    """均方根误差 sqrt(mse)。对大误差放大，主口径之一。"""
    return math.sqrt(mse(a, b))


def max_ae(a: Sequence[Number], b: Sequence[Number]) -> Number:
    """最大绝对误差 max(|e_i|)——最坏单点偏差（同族绝对度量）。"""
    _check(a, b)
    return max(abs(bi - ai) for ai, bi in zip(a, b))


def mape(a: Sequence[Number], b: Sequence[Number], *, eps: Number = 1e-12) -> Number:
    """平均绝对百分比误差 mean(|e_i / b_i|)×100%。

    分母取**基准值** b_i；任一基准值为 0（|b_i|<eps）时整项无意义，直接报错。
    返回百分数（1.4142 表示 1.4142%）。
    """
    _check(a, b)
    total = 0.0
    for ai, bi in zip(a, b):
        if abs(bi) < eps:
            raise ValueError(f"基准值过零, MAPE 无意义: b_i={bi}")
        total += abs((bi - ai) / bi)
    return total / len(a) * 100.0


def cosine_similarity(a: Sequence[Number], b: Sequence[Number]) -> Number:
    """余弦相似度——只看方向不看幅值。"""
    _check(a, b)
    dot = sum(ai * bi for ai, bi in zip(a, b))
    na = math.sqrt(sum(ai * ai for ai in a))
    nb = math.sqrt(sum(bi * bi for bi in b))
    if na == 0.0 or nb == 0.0:
        raise ValueError("零向量无方向")
    return dot / (na * nb)


def _ranks(xs: Sequence[Number]) -> list[Number]:
    """平均秩（并列取名次均值），升序。"""
    order = sorted(range(len(xs)), key=lambda i: xs[i])
    ranks = [0.0] * len(xs)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and xs[order[j + 1]] == xs[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1.0  # 名次从 1 起
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    return ranks


def spearman_rho(a: Sequence[Number], b: Sequence[Number]) -> Number:
    """斯皮尔曼等级相关——只看"起伏顺序"不看差多少。

    无并列时等价于 ``1 - 6·Σd²/(n(n²-1))``；有并列时用秩的 Pearson 通用式
    （两种实现数值一致，取通用式以覆盖并列）。
    """
    _check(a, b)
    n = len(a)
    ra, rb = _ranks(a), _ranks(b)
    ma, mb = sum(ra) / n, sum(rb) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    da = math.sqrt(sum((x - ma) ** 2 for x in ra))
    db = math.sqrt(sum((y - mb) ** 2 for y in rb))
    if da == 0.0 or db == 0.0:
        raise ValueError("序列为常数, 秩无差异")
    return num / (da * db)
