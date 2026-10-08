"""事件总线：类型化的发布订阅。

用途
----
视图、日志、统计这些模块**只订阅信号**，不反向持有模型引用。这是
实时模式和回放模式能共用一套模型代码的原因，也是模型层能独立测试的原因。

与 ``engine.inbox`` 的区别
--------------------------
方向正好相反，别混用：

* ``inbox`` —— 外部线程把东西**送进**主循环（LLM 回调、UI 指令）
* ``bus``  —— 主循环把发生的事**通知出去**（实体新增、航迹更新）

协议
----
`Signal.emit` 在**引擎主线程**同步调用。订阅者要么快速返回，要么自己
把活儿丢给线程池——在回调里做重活会直接拖慢仿真时间轴。
"""

from __future__ import annotations

from typing import Callable, Generic, Iterator, ParamSpec

P = ParamSpec("P")


class Subscription:
    """一次订阅的凭据。支持 ``with`` 语法自动退订。"""

    __slots__ = ("_signal", "_fn", "_active")

    def __init__(self, signal: "Signal", fn: Callable) -> None:
        self._signal = signal
        self._fn = fn
        self._active = True

    @property
    def active(self) -> bool:
        return self._active

    def unsubscribe(self) -> bool:
        if not self._active:
            return False
        self._active = False
        return self._signal.unsubscribe(self._fn)

    def __enter__(self) -> "Subscription":
        return self

    def __exit__(self, *exc_info) -> None:
        self.unsubscribe()

    def __repr__(self) -> str:
        state = "有效" if self._active else "已退订"
        return f"<Subscription {self._signal.name or '匿名'} {state}>"


class Signal(Generic[P]):
    """一组回调，按订阅顺序依次调用。

    **调用顺序是订阅顺序**，这一点是刻意的：视图先注册就先收到通知，
    日志后注册就后收到。顺序不确定会让"回放时渲染时序和实时不一致"。
    """

    __slots__ = ("name", "_slots")

    def __init__(self, name: str = "") -> None:
        self.name = name
        self._slots: list[Callable[P, None]] = []

    # -- 订阅 --------------------------------------------------------------

    def subscribe(self, fn: Callable[P, None]) -> Subscription:
        """注册回调。同一个函数可以重复注册（会收到多次通知）。"""
        self._slots.append(fn)
        return Subscription(self, fn)

    def unsubscribe(self, fn: Callable[P, None]) -> bool:
        """退订**一个**匹配的回调。返回是否找到并移除。"""
        try:
            self._slots.remove(fn)
        except ValueError:
            return False
        return True

    def clear(self) -> None:
        self._slots.clear()

    # -- 发射 --------------------------------------------------------------

    def emit(self, *args: P.args, **kwargs: P.kwargs) -> int:
        """通知所有订阅者，返回实际收到的数量。

        **迭代的是快照**：回调里订阅或退订不会让本次遍历崩溃。
        代价是本次 emit 已经开始后新加入的订阅者收不到这条通知，
        而刚退订的订阅者可能仍被调用一次。这个语义足够直观，
        比"用墓碑标记维护精确语义"划算得多。

        **异常直接向上抛**。事件总线里静默吞异常会让模型 bug 极难定位——
        订阅者报错说明它确实是错的，就该当场暴露。需要"一个订阅者挂了
        不影响其他"的场景，用 :meth:`emit_safe`。
        """
        if not self._slots:
            return 0
        for fn in tuple(self._slots):
            fn(*args, **kwargs)
        return len(self._slots)

    def emit_safe(
        self,
        *args: P.args,
        on_error: Callable[[Callable, BaseException], None] | None = None,
        **kwargs: P.kwargs,
    ) -> tuple[int, int]:
        """容错发射：单个订阅者抛异常时记录并继续，返回 (成功数, 失败数)。

        只用在**推演收尾的汇报环节**（生成统计、导出日志）——那时
        一个报表模块炸掉不该让整场推演的结果丢失。
        模型层的信号一律用 :meth:`emit`。
        """
        succeeded = failed = 0
        for fn in tuple(self._slots):
            try:
                fn(*args, **kwargs)
                succeeded += 1
            except BaseException as exc:  # noqa: BLE001 — 就是要全捕获
                failed += 1
                if on_error is not None:
                    on_error(fn, exc)
        return succeeded, failed

    # -- 查询 --------------------------------------------------------------

    @property
    def handler_count(self) -> int:
        return len(self._slots)

    def __len__(self) -> int:
        return len(self._slots)

    def __bool__(self) -> bool:
        return bool(self._slots)

    def __iter__(self) -> Iterator[Callable[P, None]]:
        return iter(self._slots)

    def __repr__(self) -> str:
        return f"<Signal {self.name or '匿名'} 订阅者={len(self._slots)}>"


class EventBus:
    """一组相关信号的容器。

    把信号按主题分组（实体、感知、交战……），调用方订阅时不必记住
    信号挂在哪个对象上。

    用法::

        bus = EventBus()
        sub = bus.entity_added.subscribe(on_entity_added)
        ...
        bus.entity_added.emit(entity)
        sub.unsubscribe()
    """

    __slots__ = (
        "entity_added",
        "entity_removed",
        "entity_updated",
        "track_updated",
        "track_dropped",
        "message_delivered",
        "engagement_resolved",
        "decision_issued",
        "task_changed",
        "mover_blocked",
        "simulation_state_changed",
    )

    def __init__(self) -> None:
        self.entity_added: Signal = Signal("entity_added")
        self.entity_removed: Signal = Signal("entity_removed")
        #: 供视图层用。**不要**在这里做重活——它每帧都可能触发。
        self.entity_updated: Signal = Signal("entity_updated")
        self.track_updated: Signal = Signal("track_updated")
        self.track_dropped: Signal = Signal("track_dropped")
        self.message_delivered: Signal = Signal("message_delivered")
        self.engagement_resolved: Signal = Signal("engagement_resolved")
        #: 决策可追溯：每一次决策都应能回放"当时问了什么、答了什么"
        self.decision_issued: Signal = Signal("decision_issued")
        self.task_changed: Signal = Signal("task_changed")
        #: 机动件走不通：``emit(entity_id, blocked, reason)``。
        #: **只在状态跳变时发**（进/出"走不通"各一次），不是每帧发——每帧发
        #: 等于没有信号，订阅方会被迫自己做去重，而那是发布方的事。
        #: 任务层订阅它来重规划：一条任务卡在走不通的单位上，台账必须知道。
        self.mover_blocked: Signal = Signal("mover_blocked")
        self.simulation_state_changed: Signal = Signal("simulation_state_changed")

    def signals(self) -> list[Signal]:
        """所有信号，**按名称排序**以保证可复现。"""
        return sorted(
            (getattr(self, slot) for slot in self.__slots__),
            key=lambda s: s.name,
        )

    def clear(self) -> None:
        for signal in self.signals():
            signal.clear()

    def statistics(self) -> dict[str, int]:
        return {s.name: s.handler_count for s in self.signals() if s.handler_count}

    def __repr__(self) -> str:
        active = self.statistics()
        return f"<EventBus {len(active)} 个信号有订阅者>" if active else "<EventBus 空闲>"
