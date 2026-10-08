"""线程安全投递测试。

这是多线程与 LLM 接入共用的地基，先把它测透。
"""

from __future__ import annotations

import threading

from milsim.engine import (
    PRIORITY_SENSOR,
    SECOND,
    Engine,
    EventInbox,
    SyncInbox,
)


def running_engine() -> Engine:
    engine = Engine()
    engine.start()
    return engine


# ---------------------------------------------------------------------------
# 基本语义
# ---------------------------------------------------------------------------

def test_posted_action_runs_at_current_time() -> None:
    engine = running_engine()
    seen: list[int] = []
    engine.inbox.post(lambda: seen.append(engine.now))
    engine.step_once()
    assert seen == [0]


def test_action_takes_effect_in_the_same_step() -> None:
    """时间戳必须是"现在"。

    若晚一个 step 才生效，大模型返回的决策会平白多出最多 2 秒延迟——
    而 2 秒在任务级推演里足以让一个营走出两公里的偏差。
    """
    engine = running_engine()
    engine.run_to(4 * SECOND)

    seen: list[int] = []
    engine.inbox.post(lambda: seen.append(engine.now))
    engine.step_once()

    assert seen == [4 * SECOND]


def test_external_action_precedes_same_time_events() -> None:
    """外部投递用最高优先级：上级命令优先于本级已排定的动作。"""
    engine = running_engine()
    order: list[str] = []

    engine.schedule_at(0, lambda _: order.append("event"), PRIORITY_SENSOR)
    engine.inbox.post(lambda: order.append("external"))
    engine.step_once()

    assert order == ["external", "event"]


# ---------------------------------------------------------------------------
# 并发
# ---------------------------------------------------------------------------

def test_concurrent_post_from_many_threads_is_lossless() -> None:
    """8 个线程各投 500 次，一个都不能丢。"""
    engine = running_engine()
    received: list[int] = []

    def worker(worker_id: int, count: int) -> None:
        for i in range(count):
            engine.inbox.post(lambda w=worker_id, i=i: received.append(w * 1000 + i))

    threads = [
        threading.Thread(target=worker, args=(w, 500)) for w in range(8)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert engine.inbox.posted_total == 4000

    engine.step_once()
    assert len(received) == 4000
    assert len(set(received)) == 4000  # 无重复、无覆盖


def test_producer_running_during_dispatch_is_picked_up_next_step() -> None:
    """派发过程中新投递的动作，最迟下一步被处理——绝不会丢。"""
    engine = running_engine()
    engine.start()
    seen: list[int] = []

    def producer() -> None:
        engine.inbox.post(lambda: seen.append(engine.now))

    engine.schedule(1 * SECOND, lambda _: producer())
    engine.step_once()
    engine.step_once()
    assert seen == [1 * SECOND]


# ---------------------------------------------------------------------------
# SyncInbox
# ---------------------------------------------------------------------------

def test_sync_inbox_executes_immediately() -> None:
    inbox = SyncInbox()
    seen: list[int] = []
    inbox.post(lambda: seen.append(1))
    assert seen == [1]
    assert inbox.drain() == []
    assert inbox.posted_total == 1


def test_drain_empties_the_box() -> None:
    inbox = EventInbox()
    for _ in range(10):
        inbox.post(lambda: None)
    assert len(inbox) == 10
    assert len(inbox.drain()) == 10
    assert len(inbox) == 0
