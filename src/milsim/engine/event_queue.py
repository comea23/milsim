"""时间有序事件队列。

实现要点
--------
堆元素是四元组 ``(time, priority, sequence, event)``。

**sequence 必须全局唯一**——这是让 Python 的 tuple 比较永远不会退化到比较
第四个元素（事件对象本身）的关键。事件对象没有定义 __lt__，一旦比较落到它
身上就会抛 TypeError。因为 sequence 由单调递增的计数器分配，前三项相同时
第四项必然不再参与比较，这个隐患就被彻底封住了。

重新入队（RESCHEDULE）会拿到新的、更大的 sequence，因此重排后的事件会排在
同一时刻其他事件的后面——这正是我们想要的语义：重排意味着"这一轮我让位"，
而不是"我插队"。
"""

from __future__ import annotations

import heapq
from itertools import count
from typing import TYPE_CHECKING, Iterator

from .time import INF_TIME, SimTime

if TYPE_CHECKING:
    from .event import Event


class EventQueue:
    """按 (time, priority, sequence) 排序的事件最小堆。"""

    __slots__ = ("_heap", "_counter")

    def __init__(self) -> None:
        self._heap: list[tuple[SimTime, int, int, Event]] = []
        self._counter = count()

    def push(self, event: Event) -> Event:
        """入队并返回该事件，方便链式写法。"""
        event.sequence = next(self._counter)
        heapq.heappush(
            self._heap,
            (event.time, event.priority, event.sequence, event),
        )
        return event

    def peek(self) -> Event | None:
        """查看下一个待派发事件但不移除。O(1)。"""
        return self._heap[0][3] if self._heap else None

    def peek_time(self) -> SimTime:
        """下一个事件的时刻；队列为空时返回 INF_TIME。O(1)。"""
        return self._heap[0][0] if self._heap else INF_TIME

    def pop(self) -> Event | None:
        """取出下一个待派发事件。O(log n)。"""
        return heapq.heappop(self._heap)[3] if self._heap else None

    def clear(self) -> None:
        """清空队列并重置序号计数器（用于 reset）。"""
        self._heap.clear()
        self._counter = count()

    def __len__(self) -> int:
        return len(self._heap)

    def __bool__(self) -> bool:
        return bool(self._heap)

    def __iter__(self) -> Iterator[Event]:
        """按**内部堆序**遍历，仅供调试查看，不是时间序。"""
        return (item[3] for item in self._heap)
