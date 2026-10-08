"""仿真时间基准。

内部一律使用 **整数微秒** 表示仿真时间，不用浮点秒。

为什么用整数
------------
事件排序必须绝对稳定。浮点时间在长时间推演中会累积误差，两个"本该同时"
的事件可能因为 1e-15 的差异而顺序颠倒，进而让整个推演结果发散——而且这种
错误极难复现和定位。整数比较没有这个问题，从根上杜绝。

另一个好处：Python 的 int 是任意精度的。C++ 里用 int64 存微秒会在约 29 万年
之后溢出，需要额外处理；这里不需要担心。

代价：模型内部做物理计算时要显式转换。用 to_seconds() / to_micros()，
不要手写 / 1e6，避免常量不一致。

换算约定
--------
    SimTime   绝对时间，单位微秒，从 0 起算
    Duration  时长，单位微秒
"""

from __future__ import annotations

from typing import Final

SimTime = int
Duration = int

US_PER_SECOND: Final[int] = 1_000_000

SECOND: Final[int] = US_PER_SECOND
MILLISECOND: Final[int] = 1_000
MINUTE: Final[int] = 60 * US_PER_SECOND
HOUR: Final[int] = 3600 * US_PER_SECOND

#: 时间轴上的"无穷远"，供空队列的 peek_time() 返回
INF_TIME: Final[int] = 1 << 62

#: 单次推进的硬上限。2 秒是任务级仿真的经验值：
#: 再大则栅栏兜底失效（无事件时段的状态刷新太慢），
#: 再小则大量时间点空转（任务级动作本来就是秒级以上的粒度）。
MAX_STEP: Final[Duration] = 2 * US_PER_SECOND


def to_micros(seconds: float) -> Duration:
    """秒 → 微秒。用 round 而非 int()，避免 0.1 * 1e6 = 99999.999... 被截断成 99999。"""
    return int(round(seconds * US_PER_SECOND))


def to_seconds(t: SimTime) -> float:
    """微秒 → 秒。仅在需要浮点计算时使用。"""
    return t / US_PER_SECOND


def fmt(t: SimTime) -> str:
    """格式化为 HH:MM:SS.mmm。只用于日志和输出，不参与运算。"""
    sign = "-" if t < 0 else ""
    t = abs(t)
    total_ms, _ = divmod(t, 1000)
    sec, ms = divmod(total_ms, 1000)
    minute, sec = divmod(sec, 60)
    hour, minute = divmod(minute, 60)
    return f"{sign}{hour:02d}:{minute:02d}:{sec:02d}.{ms:03d}"
