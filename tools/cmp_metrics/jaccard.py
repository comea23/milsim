"""2.13 二值 0/1 向量匹配（Jaccard / SMC）—— 探测/告警类序列的主口径。

两条 0/1 序列逐位对照建 2×2 频数表::

                   待评=1    待评=0
    基准=1         a(都对)   b(漏报)
    基准=0         c(虚警)   d(双0)

    Jaccard = a/(a+b+c)   —— 只看"发生的事"对不对得上（**主口径**：0 是
                             "没发生"，有无穷多种没发生的方式，双 0 不携带
                             模式信息）
    SMC     = (a+d)/n     —— 全维一致率（对照口径；0/1 对称编码才有意义）

漏报 b / 虚警 c 计数给出**错误的方向**：率相同≠模式相同。
红线：0/1 数据禁用均值方差类方法（均值=频率、方差=p(1-p)，对模式全盲）。

本项目使用范围：milsim 与 AFSIM 的逐 tick 探测事件序列对齐到同一时间栅格
后，每对"雷达-目标"的发现/漏发现一致性；通信链路通断序列同理。
"""

from __future__ import annotations

from typing import Sequence


def binary_match(a: Sequence[int], b: Sequence[int]) -> dict[str, int]:
    """2×2 匹配频数表。``a`` 基准、``b`` 待评，元素只许 0/1。"""
    if len(a) != len(b):
        raise ValueError(f"等长对齐前提不成立: {len(a)} vs {len(b)}")
    if not a:
        raise ValueError("空序列")
    counts = {"a": 0, "b": 0, "c": 0, "d": 0}
    for x, y in zip(a, b):
        if x not in (0, 1) or y not in (0, 1):
            raise ValueError(f"只许 0/1, 得到 ({x}, {y})")
        if x == 1 and y == 1:
            counts["a"] += 1
        elif x == 1 and y == 0:
            counts["b"] += 1  # 漏报
        elif x == 0 and y == 1:
            counts["c"] += 1  # 虚警
        else:
            counts["d"] += 1
    return counts


def jaccard_score(a: Sequence[int], b: Sequence[int]) -> float:
    """Jaccard = a/(a+b+c)；无任何"发生"（a+b+c=0）时约定返回 1.0
    （两边都没发生过事件，模式完全一致）。"""
    m = binary_match(a, b)
    denom = m["a"] + m["b"] + m["c"]
    if denom == 0:
        return 1.0
    return m["a"] / denom


def smc_score(a: Sequence[int], b: Sequence[int]) -> float:
    """SMC = (a+d)/n 全维一致率。"""
    m = binary_match(a, b)
    return (m["a"] + m["d"]) / len(a)
