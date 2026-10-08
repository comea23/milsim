"""事件总线测试。"""

from __future__ import annotations

import pytest

from milsim.services.bus import EventBus, Signal


# ---------------------------------------------------------------------------
# 基本订阅
# ---------------------------------------------------------------------------

def test_subscribe_and_emit() -> None:
    signal: Signal = Signal("test")
    received: list[int] = []
    signal.subscribe(received.append)

    assert signal.emit(1) == 1
    assert signal.emit(2) == 1
    assert received == [1, 2]


def test_emit_with_no_subscribers_returns_zero() -> None:
    signal: Signal = Signal("empty")
    assert signal.emit() == 0


def test_multiple_arguments_pass_through() -> None:
    signal: Signal = Signal("multi")
    captured: list[tuple] = []
    signal.subscribe(lambda a, b, c: captured.append((a, b, c)))

    signal.emit(1, "two", 3.0)
    assert captured == [(1, "two", 3.0)]


def test_handlers_called_in_subscription_order() -> None:
    """顺序必须稳定——否则回放时渲染时序和实时不一致。"""
    signal: Signal = Signal("ordered")
    order: list[str] = []
    for name in ("first", "second", "third"):
        signal.subscribe(lambda n=name: order.append(n))

    signal.emit()
    assert order == ["first", "second", "third"]


def test_same_handler_can_subscribe_twice() -> None:
    signal: Signal = Signal("dup")
    count: list[int] = []
    handler = lambda: count.append(1)  # noqa: E731

    signal.subscribe(handler)
    signal.subscribe(handler)
    signal.emit()
    assert len(count) == 2
    assert signal.handler_count == 2


# ---------------------------------------------------------------------------
# 退订
# ---------------------------------------------------------------------------

def test_unsubscribe_stops_notifications() -> None:
    """用绑定方法订阅也能正常退订。

    ``received.append`` 每次求值都是新的方法对象，但绑定方法的相等性比较
    看的是 ``__self__`` 和 ``__func__``，所以 ``list.remove`` 能匹配上。
    """
    signal: Signal = Signal("unsub")
    received: list[int] = []
    signal.subscribe(received.append)
    assert signal.handler_count == 1

    assert signal.unsubscribe(received.append) is True
    assert signal.handler_count == 0

    signal.emit(1)
    assert received == []


def test_unsubscribe_with_subscription_handle() -> None:
    signal: Signal = Signal("handle")
    received: list[int] = []
    sub = signal.subscribe(received.append)

    signal.emit(1)
    assert sub.active
    assert sub.unsubscribe() is True
    assert not sub.active

    signal.emit(2)
    assert received == [1]

    # 重复退订是安全的
    assert sub.unsubscribe() is False


def test_subscription_as_context_manager() -> None:
    signal: Signal = Signal("ctx")
    received: list[int] = []

    with signal.subscribe(received.append):
        signal.emit(1)

    signal.emit(2)
    assert received == [1]


def test_subscribing_during_emit_does_not_break_iteration() -> None:
    """回调里订阅不该让本次遍历崩溃，新订阅者从下次 emit 开始收到通知。"""
    signal: Signal = Signal("reentrant")
    late: list[int] = []

    def self_subscribe(_value: int) -> None:
        signal.subscribe(late.append)

    signal.subscribe(self_subscribe)
    signal.emit(1)                  # 不崩溃
    assert late == []

    signal.emit(2)
    assert late == [2]


def test_unsubscribing_during_emit_is_safe() -> None:
    """回调里退订也不该崩溃。本次可能仍收到一次，下次不会。"""
    signal: Signal = Signal("unsub_during")
    order: list[str] = []

    def first() -> None:
        order.append("a")
        signal.unsubscribe(second)

    def second() -> None:
        order.append("b")

    signal.subscribe(first)
    signal.subscribe(second)

    signal.emit()
    signal.emit()
    assert order.count("a") == 2
    assert order.count("b") <= 1


# ---------------------------------------------------------------------------
# 异常
# ---------------------------------------------------------------------------

def test_emit_propagates_exceptions() -> None:
    """静默吞异常会让模型 bug 极难定位，所以默认直接抛。"""
    signal: Signal = Signal("boom")
    signal.subscribe(lambda: 1 / 0)

    with pytest.raises(ZeroDivisionError):
        signal.emit()


def test_emit_safe_isolates_failures() -> None:
    """容错模式：一个订阅者炸了不影响其他，返回 (成功, 失败)。"""
    signal: Signal = Signal("safe")
    received: list[int] = []

    signal.subscribe(lambda: received.append(1))
    signal.subscribe(lambda: 1 / 0)
    signal.subscribe(lambda: received.append(3))

    errors: list[type] = []
    succeeded, failed = signal.emit_safe(on_error=lambda fn, exc: errors.append(type(exc)))

    assert received == [1, 3]
    assert (succeeded, failed) == (2, 1)
    assert errors == [ZeroDivisionError]


def test_emit_safe_without_error_hook() -> None:
    signal: Signal = Signal("safe2")
    signal.subscribe(lambda: None)
    assert signal.emit_safe() == (1, 0)


# ---------------------------------------------------------------------------
# 容器
# ---------------------------------------------------------------------------

def test_event_bus_exposes_named_signals() -> None:
    bus = EventBus()
    assert isinstance(bus.entity_added, Signal)
    assert bus.entity_added.name == "entity_added"

    seen: list[str] = []
    bus.entity_added.subscribe(seen.append)
    bus.entity_added.emit("unit-1")
    assert seen == ["unit-1"]


def test_event_bus_signals_are_sorted_by_name() -> None:
    """按名称排序保证遍历可复现。"""
    bus = EventBus()
    names = [s.name for s in bus.signals()]
    assert names == sorted(names)
    assert len(names) == len(EventBus.__slots__)


def test_event_bus_statistics_and_clear() -> None:
    bus = EventBus()
    bus.entity_added.subscribe(lambda e: None)
    bus.track_updated.subscribe(lambda t: None)
    bus.track_updated.subscribe(lambda t: None)

    stats = bus.statistics()
    assert stats == {"entity_added": 1, "track_updated": 2}

    bus.clear()
    assert bus.statistics() == {}


def test_signal_bool_and_len() -> None:
    signal: Signal = Signal("x")
    assert not signal
    assert len(signal) == 0
    signal.subscribe(lambda: None)
    assert signal
    assert len(signal) == 1


def test_subscription_repr_mentions_state() -> None:
    signal: Signal = Signal("named")
    sub = signal.subscribe(lambda: None)
    assert "named" in repr(sub)
    sub.unsubscribe()
    assert "已退订" in repr(sub)
