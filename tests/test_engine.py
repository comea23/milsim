"""引擎行为测试。

这些测试覆盖 M1 里程碑的验收标准：
  * 混合步进的两条分支（事件优先 / 栅栏兜底）各自正确
  * 事件排序与理论顺序逐项一致
  * 同种子两次运行结果完全相同
"""

from __future__ import annotations

import random

import pytest

from milsim.engine import (
    INF_TIME,
    MAX_STEP,
    MILLISECOND,
    PRIORITY_COMMAND,
    PRIORITY_MOVER,
    PRIORITY_SENSOR,
    SECOND,
    DispatchOverflow,
    Engine,
    EngineState,
    EventQueue,
    EventResult,
    RecurringEvent,
    to_micros,
)


def running_engine(**kwargs) -> Engine:
    engine = Engine(**kwargs)
    engine.start()
    return engine


# ---------------------------------------------------------------------------
# 时间基座
# ---------------------------------------------------------------------------

def test_to_micros_does_not_truncate() -> None:
    # 0.1 * 1e6 在浮点下是 99999.999...，直接 int() 会少 1 微秒。
    assert to_micros(0.1) == 100_000
    assert to_micros(0.3) == 300_000
    assert to_micros(2.0) == 2_000_000


# ---------------------------------------------------------------------------
# 栅栏分支：无事件时以 MAX_STEP 为粒度空转推进
# ---------------------------------------------------------------------------

def test_empty_queue_advances_exactly_one_max_step() -> None:
    engine = running_engine()
    assert engine.step_once() == MAX_STEP
    assert engine.step_once() == 2 * MAX_STEP
    assert engine.fence_count == 2


def test_fence_hook_fires_once_per_fence() -> None:
    engine = running_engine()
    seen: list[int] = []
    engine.on_fence(seen.append)
    engine.step_once()
    engine.step_once()
    assert seen == [MAX_STEP, 2 * MAX_STEP]


def test_peek_time_is_infinite_when_empty() -> None:
    engine = running_engine()
    assert engine.queue.peek_time() == INF_TIME


# ---------------------------------------------------------------------------
# 事件分支：事件时刻优先于栅栏
# ---------------------------------------------------------------------------

def test_event_closer_than_fence_wins() -> None:
    engine = running_engine()
    fired: list[int] = []
    engine.schedule(300 * MILLISECOND, lambda eng: fired.append(eng.now))

    assert engine.step_once() == 300 * MILLISECOND
    assert fired == [300 * MILLISECOND]
    assert engine.fence_count == 0  # 这一步没跑到栅栏


def test_event_beyond_fence_is_capped_and_not_delayed() -> None:
    """事件在 10 秒后：前四步被栅栏截断，第五步精确停在事件时刻。"""
    engine = running_engine()
    fired: list[int] = []
    engine.schedule(10 * SECOND, lambda eng: fired.append(eng.now))

    for _ in range(4):
        engine.step_once()
    assert fired == []
    assert engine.now == 8 * SECOND

    assert engine.step_once() == 10 * SECOND
    assert fired == [10 * SECOND]


def test_event_inserted_during_dispatch_runs_in_same_step() -> None:
    """派发中产生的新事件，若时刻不晚于本步目标，必须同步处理掉。"""
    engine = running_engine()
    tail: list[int] = []

    def first(eng: Engine) -> None:
        eng.schedule(0, lambda e2: tail.append(e2.now))

    engine.schedule(500 * MILLISECOND, first)
    engine.step_once()
    assert tail == [500 * MILLISECOND]


# ---------------------------------------------------------------------------
# 排序契约
# ---------------------------------------------------------------------------

def test_same_time_events_ordered_by_priority() -> None:
    engine = running_engine()
    order: list[str] = []
    engine.schedule(1 * SECOND, lambda _: order.append("mover"), PRIORITY_MOVER)
    engine.schedule(1 * SECOND, lambda _: order.append("sensor"), PRIORITY_SENSOR)
    engine.schedule(1 * SECOND, lambda _: order.append("command"), PRIORITY_COMMAND)
    engine.step_once()
    assert order == ["sensor", "command", "mover"]


def test_same_time_same_priority_is_fifo() -> None:
    engine = running_engine()
    order: list[int] = []
    for i in range(20):
        engine.schedule(1 * SECOND, (lambda i: lambda _: order.append(i))(i))
    engine.step_once()
    assert order == list(range(20))


def test_dispatch_order_matches_theory_exactly() -> None:
    """300 个随机事件，派发顺序必须与 (time, priority, seq) 排序完全一致。"""
    rng = random.Random(2024)
    engine = running_engine()
    fired: list[int] = []
    expected: list[tuple[int, int, int]] = []

    for i in range(300):
        t = rng.randrange(0, 5 * SECOND)
        p = rng.choice([0, 100, 200, 300, 400])
        expected.append((t, p, i))
        engine.schedule_at(t, (lambda i: lambda _: fired.append(i))(i), p)

    while engine.queue:
        engine.step_once()

    expected.sort()  # (time, priority, sequence)
    assert [i for _, _, i in expected] == fired


def test_reschedule_receives_new_sequence() -> None:
    """重排的事件必须拿到更大的 sequence，即让位给同刻的其他事件。"""
    queue = EventQueue()
    event = queue.push(RecurringEvent(100, lambda eng, ev: EventResult.RESCHEDULE))
    first_seq = event.sequence
    queue.pop()
    queue.push(event)
    assert event.sequence > first_seq


def test_recurring_event_with_fixed_interval_advances_itself() -> None:
    """固定周期模式：回调只管返回 RESCHEDULE，时间由事件自己推进。"""
    engine = running_engine()
    ticks: list[int] = []

    def tick(eng: Engine, ev: RecurringEvent) -> EventResult:
        ticks.append(ev.time)
        return EventResult.RESCHEDULE if len(ticks) < 5 else EventResult.DELETE

    engine.schedule_recurring(1 * SECOND, 1 * SECOND, tick)
    engine.run_to(6 * SECOND)

    assert ticks == [1 * SECOND, 2 * SECOND, 3 * SECOND, 4 * SECOND, 5 * SECOND]
    assert len(engine.queue) == 0


def test_recurring_event_variable_interval_mode() -> None:
    """变周期模式：不给 interval，由回调自己设置下次时间。"""
    engine = running_engine()
    ticks: list[int] = []

    def tick(eng: Engine, ev: RecurringEvent) -> EventResult:
        ticks.append(ev.time)
        if len(ticks) >= 3:
            return EventResult.DELETE
        ev.time += len(ticks) * SECOND      # 间隔递增：1s, 2s, 3s...
        return EventResult.RESCHEDULE

    engine.add_event(RecurringEvent(1 * SECOND, tick))
    engine.run_to(10 * SECOND)

    assert ticks == [1 * SECOND, 2 * SECOND, 4 * SECOND]


def test_recurring_event_rejects_non_positive_interval() -> None:
    """interval <= 0 会让事件无限自激，构造时就该拒绝。"""
    with pytest.raises(ValueError, match="interval"):
        RecurringEvent(0, lambda eng, ev: EventResult.DELETE, interval=0)


# ---------------------------------------------------------------------------
# 防护与状态机
# ---------------------------------------------------------------------------

def test_self_replicating_event_fails_fast() -> None:
    engine = running_engine(max_dispatch_per_step=50)

    def replicate(eng: Engine) -> None:
        eng.schedule(0, replicate)

    engine.schedule(0, replicate)
    with pytest.raises(DispatchOverflow):
        engine.step_once()


def test_time_never_moves_backwards() -> None:
    engine = running_engine()
    engine.schedule(
        1 * SECOND,
        lambda eng: eng.schedule(-5 * SECOND, lambda _: None),
    )
    engine.step_once()
    assert engine.now == 1 * SECOND
    engine.step_once()
    assert engine.now >= 1 * SECOND


def test_cannot_step_when_paused() -> None:
    engine = running_engine()
    engine.pause()
    assert engine.state is EngineState.PAUSED
    with pytest.raises(RuntimeError):
        engine.step_once()


def test_reset_clears_everything() -> None:
    engine = running_engine()
    engine.schedule(1 * SECOND, lambda _: None)
    engine.step_once()
    engine.reset()
    assert engine.now == 0
    assert len(engine.queue) == 0
    assert engine.state is EngineState.CREATED
    assert engine.dispatch_count == 0


# ---------------------------------------------------------------------------
# 精确停点
# ---------------------------------------------------------------------------

def test_step_until_has_no_overshoot() -> None:
    engine = running_engine()
    engine.step_until(700 * MILLISECOND)
    assert engine.now == 700 * MILLISECOND


def test_step_until_across_multiple_fences() -> None:
    engine = running_engine()
    engine.step_until(5 * SECOND)
    assert engine.now == 5 * SECOND


def test_run_to_advances_time_even_with_empty_queue() -> None:
    """无事发生也要走完时间轴——这是栅栏存在的意义之一。"""
    engine = running_engine()
    engine.run_to(10 * SECOND)
    assert engine.now == 10 * SECOND
    assert engine.fence_count == 5


# ---------------------------------------------------------------------------
# 确定性
# ---------------------------------------------------------------------------

def test_identical_seed_produces_identical_run() -> None:
    """同一随机种子跑两遍，(时刻, 事件编号) 序列必须逐项相同。"""

    def run(seed: int) -> list[tuple[int, int]]:
        rng = random.Random(seed)
        engine = Engine()
        engine.start()
        log: list[tuple[int, int]] = []
        for i in range(200):
            t = rng.randrange(0, 5 * SECOND)
            p = rng.choice([PRIORITY_SENSOR, PRIORITY_COMMAND, PRIORITY_MOVER])
            engine.schedule_at(
                t, (lambda i: lambda eng: log.append((eng.now, i)))(i), p
            )
        engine.run_to(5 * SECOND)
        return log

    assert run(42) == run(42)


def test_different_seed_produces_different_run() -> None:
    def run(seed: int) -> list[int]:
        rng = random.Random(seed)
        engine = Engine()
        engine.start()
        log: list[int] = []
        for i in range(200):
            t = rng.randrange(0, 5 * SECOND)
            engine.schedule_at(t, (lambda eng: log.append(eng.now)))
        engine.run_to(5 * SECOND)
        return log

    assert run(1) != run(2)
