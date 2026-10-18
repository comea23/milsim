"""2.7 TIC（Theil 不等式系数）—— RMSE 的归一化版。

公式::

    U = RMSE / ( RMS(ref) + RMS(pred) )

值域 [0,1]（三角不等式），完全一致 U=0；分母除非两序列全零否则恒正。
对幅值整体缩放不敏感（只与缩放比 k 有关），可跨指标、跨量级比较。

本项目使用范围：轨迹分量（对齐栅格后）与首次探测距离的**相对口径**，
与绝对口径（MAE/RMSE/MaxAE）并用；经验判据 U<0.3。
"""

from __future__ import annotations

import math
from typing import Sequence

from .time_domain import rmse


def _rms(xs: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def tic(pred: Sequence[float], ref: Sequence[float]) -> float:
    """Theil 不等式系数 U∈[0,1]，0=完全一致。

    约定与 2.1 相同：``pred`` 待评、``ref`` 基准（U 对两者地位对称，仅
    文档口径上如此记）。
    """
    if len(pred) != len(ref):
        raise ValueError(f"等长对齐前提不成立: {len(pred)} vs {len(ref)}")
    if not pred:
        raise ValueError("空序列")
    denom = _rms(ref) + _rms(pred)
    if denom == 0.0:
        raise ValueError("两序列全零, U 无定义")
    return rmse(pred, ref) / denom
