"""2.8 RMAE（相对平均绝对误差）—— MAE 除以参考幅值。

公式::

    RMAE = mean(|ref_i - pred_i|) / mean(|ref_i|)

回答"平均偏差占参考幅值的百分之几"；RMAE=0.1 即平均偏了 10%。
文献另有以基准模型 MAE 为分母的口径（"比笨办法好多少"），两种数值不同，
报告须写死用哪个——本实现取**参考幅值**口径。

本项目使用范围：首次探测距离等标量簇的相对口径（绝对口径 2.1 并用）。
"""

from __future__ import annotations

from typing import Sequence

from .time_domain import mae


def rmae(pred: Sequence[float], ref: Sequence[float]) -> float:
    """相对平均绝对误差（分母 = 参考序列平均幅值）。"""
    if len(pred) != len(ref):
        raise ValueError(f"等长对齐前提不成立: {len(pred)} vs {len(ref)}")
    if not ref:
        raise ValueError("空序列")
    denom = sum(abs(x) for x in ref) / len(ref)
    if denom == 0.0:
        raise ValueError("参考序列全零, RMAE 退化")
    return mae(pred, ref) / denom
