"""事件定义。

事件是引擎唯一的时间驱动单元。任何在时间轴上发生的动作都表示为一个事件
对象，由 EventQueue 统一排序派发。

排序契约（三重键，与 AFSIM 的 WsfEventManager 一致）
----------------------------------------------------
    1. time       升序   主键
    2. priority   升序   同一时刻的执行次序，数值小者先执行
    3. sequence   升序   入队序号，保证同刻同优先级事件按 FIFO 执行

事件对象本身不实现 __lt__，排序完全由队列依据上述三个字段决定。
这样杜绝了"事件内容意外影响排序"这类隐蔽耦合。

优先级分段约定见 engine.py 的 PRIORITY_* 常量。
"""

from __future__ import annotations

from enum import IntEnum
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:  # 避免运行时循环导入
    from .engine import Engine
    from .time import SimTime


class EventResult(IntEnum):
    """事件的处置方式，作为 execute() 的返回值。

    对应 AFSIM 的 WsfEvent::EventDisposition。用 RESCHEDULE 让周期性事件
    原地重排，避免反复销毁和重建对象——周期事件每秒可能触发上千次，
    省下的分配开销是可观的。
    """

    DELETE = 0      # 执行后从队列移除
    RESCHEDULE = 1  # 重新入队；execute() 内应已更新 self.time


class Event:
    """所有事件的基类。

    子类必须实现 execute()。

    __slots__ 是刻意为之：推演高峰每秒可能创建上千个事件，每个实例省掉一个
    __dict__ 能显著降低内存和分配开销。派生类若需要额外字段，**必须自己声明
    __slots__**，否则会重新获得 __dict__，前功尽弃。
    """

    __slots__ = ("time", "priority", "sequence")

    def __init__(self, time: SimTime, priority: int = 0) -> None:
        self.time = time
        self.priority = priority
        self.sequence: int = -1  # 由 EventQueue.push() 赋值

    def execute(self, engine: Engine) -> EventResult:
        raise NotImplementedError

    def __repr__(self) -> str:
        return (
            f"<{type(self).__name__} t={self.time} "
            f"p={self.priority} seq={self.sequence}>"
        )


class OneShotEvent(Event):
    """执行一次即被移除的事件，回调签名 fn(engine) -> None。"""

    __slots__ = ("_fn",)

    def __init__(
        self,
        time: SimTime,
        fn: Callable[[Engine], None],
        priority: int = 0,
    ) -> None:
        super().__init__(time, priority)
        self._fn = fn

    def execute(self, engine: Engine) -> EventResult:
        self._fn(engine)
        return EventResult.DELETE


class RecurringEvent(Event):
    """可重复执行的事件，回调签名 fn(engine, event) -> EventResult。

    两种用法：

    **固定周期**（构造时给 ``interval``）—— 回调只需返回 ``RESCHEDULE``，
    下次触发时间由事件自己推进：

        RecurringEvent(t, tick, interval=SECOND)

    **变周期**（不给 ``interval``）—— 回调自己设置 ``ev.time`` 再返回
    ``RESCHEDULE``，用于触发间隔会变的场景（如扫描间隔随威胁等级调整）：

        def tick(engine, ev):
            ev.time += compute_next_interval()
            return EventResult.RESCHEDULE

    固定周期务必用第一种。手写 ``ev.time += dt`` 很容易在多处重复实现，
    而每处都是一个算错时间的机会。

    重新入队会分配新的 sequence，所以优先级改动会立即生效
    （与 AFSIM 的说明一致：入队后再改 priority 不影响已排定的顺序）。
    """

    __slots__ = ("_fn", "interval")

    def __init__(
        self,
        time: SimTime,
        fn: Callable[[Engine, RecurringEvent], EventResult],
        priority: int = 0,
        interval: int | None = None,
    ) -> None:
        super().__init__(time, priority)
        if interval is not None and interval <= 0:
            # interval <= 0 意味着时间不推进，事件会无限自激。
            # 与其运行到 DispatchOverflow 才报错，不如构造时就拒绝。
            raise ValueError(f"interval 必须为正数微秒，收到 {interval}")
        self._fn = fn
        self.interval = interval

    def execute(self, engine: Engine) -> EventResult:
        result = self._fn(engine, self)
        if result is EventResult.RESCHEDULE and self.interval is not None:
            self.time += self.interval
        return result
