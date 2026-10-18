"""2.14 比例 Z 检验 —— 事件率是否显著偏离基准率。

公式::

    z = (p̂ - p0) / sqrt(p0(1-p0)/n),  p̂ = k/n

z 近似标准正态；双尾 p 值用 erf 算（标准库，无 scipy）。
正态近似有效条件：``n·p0·(1-p0) ≥ 5``（期望成功/失败数都不太小），不满足
须换精确二项检验——本实现返回 ``approx_ok`` 标志，不擅自降级。

本项目使用范围：**二期**。命中率/毁伤达成率/发现率对基准率的检验需要大
样本重复仿真（单次对拍给不出 n），当前逐 tick 对拍阶段用不到。
"""

from __future__ import annotations

import math


def _norm_sf2(z: float) -> float:
    """双尾 p：P(|Z|>|z|) = 2·(1-Φ(|z|))，Φ 用 erf。"""
    phi = 0.5 * (1.0 + math.erf(abs(z) / math.sqrt(2.0)))
    return 2.0 * (1.0 - phi)


def proportion_ztest(
    k: int,
    n: int,
    p0: float,
) -> tuple[float, float, bool]:
    """返回 ``(z, 双尾p, 正态近似是否有效)``。

    ``k`` 事件数、``n`` 总次数、``p0`` 基准率。
    """
    if n <= 0:
        raise ValueError(f"n 须为正, 得到 {n}")
    if not 0.0 < p0 < 1.0:
        raise ValueError(f"p0∈(0,1), 得到 {p0}")
    if not 0 <= k <= n:
        raise ValueError(f"k∈[0,n], 得到 {k}")
    phat = k / n
    se = math.sqrt(p0 * (1.0 - p0) / n)
    z = (phat - p0) / se
    ok = n * p0 * (1.0 - p0) >= 5.0
    return z, _norm_sf2(z), ok
