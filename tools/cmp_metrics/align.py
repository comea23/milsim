"""时间轴插值对齐 —— 等长误差指标（2.1/2.7/2.8/2.9）的前置。

背景：milsim 固定步长（典型 1s），**AFSIM 输出采样间隔不受我们控制**
（事件驱动/自有输出节奏），两边点数与时刻一般对不上。等长指标的前提
"第 i 个点对应同一时刻"必须先补齐，这就是"两口径法"里的插值对齐口径：
把两边都重采样到统一等间隔栅格（线性插值）再逐点比。

口径纪律（报告必须写明）：
- 栅格起点 = 两序列共同时间窗的起点（max(各自 t0)），终点同理取 min(t1)；
- 栅格步长 dt 由调用方给定（对拍常用 1.0s）；
- 插值是线性内插，**不外推**——共同窗外的数据丢弃；
- 原始点列口径（不插值）走 2.3 Fréchet / 2.10 Hausdorff，两口径并用可
  分离"采样稀疏成分"与"真实偏差成分"。
"""

from __future__ import annotations

from typing import Sequence


def _check_series(times: Sequence[float], values: Sequence[float]) -> None:
    if len(times) != len(values):
        raise ValueError(f"时间/值不等长: {len(times)} vs {len(values)}")
    if len(times) < 2:
        raise ValueError("至少两个点才能插值")
    if any(t2 <= t1 for t1, t2 in zip(times, times[1:])):
        raise ValueError("时间须严格递增")


def resample(
    times: Sequence[float],
    values: Sequence[float],
    dt: float,
    t0: float | None = None,
    t1: float | None = None,
) -> tuple[list[float], list[float]]:
    """线性插值重采样到等间隔栅格 ``t0, t0+dt, …, ≤t1``。

    返回 ``(栅格时刻, 插值值)``；不外推，栅格严格落在 [t0, t1] 内。
    """
    _check_series(times, values)
    if dt <= 0.0:
        raise ValueError(f"dt 须为正, 得到 {dt}")
    lo = times[0] if t0 is None else t0
    hi = times[-1] if t1 is None else t1
    if lo < times[0] or hi > times[-1]:
        raise ValueError("栅格窗超出数据范围（不外推）")
    if hi < lo:
        raise ValueError(f"时间窗反了: [{lo}, {hi}]")
    grid_t: list[float] = []
    grid_v: list[float] = []
    k = 0
    n = len(times)
    t = lo
    while t <= hi + 1e-12:
        while k < n - 2 and times[k + 1] < t:
            k += 1
        ta, tb = times[k], times[k + 1]
        va, vb = values[k], values[k + 1]
        w = 0.0 if tb == ta else (t - ta) / (tb - ta)
        grid_t.append(t)
        grid_v.append(va + (vb - va) * w)
        t += dt
    return grid_t, grid_v


def align_pairs(
    times_a: Sequence[float],
    values_a: Sequence[float],
    times_b: Sequence[float],
    values_b: Sequence[float],
    dt: float,
) -> tuple[list[float], list[float], list[float]]:
    """把两条序列对齐到**共同时间窗**内的同一等间隔栅格。

    返回 ``(栅格时刻, a 插值列, b 插值列)``，可直接喂 2.1/2.7/2.8/2.9。
    共同窗为空（两序列时间上不重叠）时报错。
    """
    _check_series(times_a, values_a)
    _check_series(times_b, values_b)
    lo = max(times_a[0], times_b[0])
    hi = min(times_a[-1], times_b[-1])
    if hi < lo:
        raise ValueError(f"两序列时间窗不重叠: [{lo}, {hi}]")
    t, va = resample(times_a, values_a, dt, lo, hi)
    _, vb = resample(times_b, values_b, dt, lo, hi)
    return t, va, vb
