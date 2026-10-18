"""2.9 RRMSE（相对均方根误差）—— RMSE 归一化，保留大误差敏感性。

公式（两种分母口径，文献都在用）::

    RRMSE = RMSE / mean(ref)            # denom="mean"：恒为正、均值有意义的量（如探测距离）
    RRMSE = RMSE / sqrt(mean(ref_i^2))  # denom="rms" ：正负振荡信号（均值口径会失真/除零）

本项目使用范围：首次探测距离（mean 口径）与轨迹分量（rms 口径）的相对
口径；与 TIC 2.7 并用（TIC 对称、RRMSE 单侧贴参考）。
"""

from __future__ import annotations

import math
from typing import Sequence

from .time_domain import rmse


def rrmse(
    pred: Sequence[float],
    ref: Sequence[float],
    *,
    denom: str = "rms",
) -> float:
    """相对均方根误差。``denom``：``"mean"`` 除以均值，``"rms"`` 除以均方根。"""
    if len(pred) != len(ref):
        raise ValueError(f"等长对齐前提不成立: {len(pred)} vs {len(ref)}")
    if not ref:
        raise ValueError("空序列")
    if denom == "mean":
        d = sum(ref) / len(ref)
    elif denom == "rms":
        d = math.sqrt(sum(x * x for x in ref) / len(ref))
    else:
        raise ValueError(f"denom 只许 mean|rms, 得到 {denom!r}")
    if d == 0.0:
        raise ValueError(f"分母为 0（denom={denom}）, RRMSE 退化")
    return rmse(pred, ref) / d
