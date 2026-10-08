"""引擎吞吐基准。

目的不是刷分，而是回答两个工程问题：

  1. 纯 Python 的事件派发循环，每秒能处理多少个事件？
  2. 空转推进（只有栅栏、没有事件）有多快？这决定了"推演三天"要等多久。

这两个数决定了后面是否必须给热点（A* 寻路、通视采样）上 numpy / C 扩展。
不是 pytest 用例，直接运行：

    python tests/bench_engine.py
"""

from __future__ import annotations

import time

from milsim.engine import Engine, EventResult, SECOND


def bench_dense(event_count: int, horizon_us: int) -> None:
    """事件密集：每步恰好一个事件，测派发循环的纯开销。"""
    engine = Engine()
    engine.start()
    sink = 0

    def callback(eng: Engine) -> None:
        nonlocal sink
        sink += 1

    interval = horizon_us // event_count
    for i in range(event_count):
        engine.schedule_at(i * interval, callback)

    t0 = time.perf_counter()
    engine.run_to(horizon_us)
    elapsed = time.perf_counter() - t0

    print(
        f"  事件密集   {event_count:>8,} 事件 / {horizon_us/SECOND:>7.0f} s  "
        f"{elapsed:>7.3f} s   {event_count/elapsed:>12,.0f} 事件/秒"
        f"   {engine.step_count:>9,} 步"
    )


def bench_sparse(horizon_us: int) -> None:
    """空转推进：队列始终为空，测栅栏空转速度。"""
    engine = Engine()
    engine.start()

    t0 = time.perf_counter()
    engine.run_to(horizon_us)
    elapsed = time.perf_counter() - t0

    steps = horizon_us // (2 * SECOND)
    print(
        f"  空转推进   {horizon_us/SECOND:>8.0f} s 时间轴  "
        f"{elapsed:>7.3f} s   {steps/elapsed:>12,.0f} 步/秒"
        f"   {engine.step_count:>9,} 步"
    )

    # 换算成"推演一天需要多久"
    per_day = elapsed / (horizon_us / SECOND) * 86400
    print(f"             → 推演 24 小时（纯空转）约需 {per_day:.2f} s")


def bench_mixed(entity_count: int, interval_us: int, horizon_us: int) -> None:
    """贴近真实场景：每个实体一个周期事件，模拟每 2 秒全量更新。"""
    engine = Engine()
    engine.start()
    sink = 0

    def make_tick():
        def tick(eng: Engine, ev) -> EventResult:
            nonlocal sink
            sink += 1
            # 周期由 RecurringEvent 自己推进，回调不必管时间
            return EventResult.RESCHEDULE
        return tick

    for eid in range(entity_count):
        # 错开首次触发时刻，避免所有实体挤在同一微秒——真实推演里
        # 模型的周期本来就是错开的，这也顺便避开堆的同刻退化路径。
        engine.schedule_recurring(
            (eid * 137) % interval_us,
            interval_us,
            make_tick(),
        )

    t0 = time.perf_counter()
    engine.run_to(horizon_us)
    elapsed = time.perf_counter() - t0

    updates = entity_count * (horizon_us // interval_us)
    print(
        f"  混合负载   {entity_count:>8,} 实体 × {horizon_us/SECOND:>6.0f} s  "
        f"{elapsed:>7.3f} s   {updates/elapsed:>12,.0f} 更新/秒"
        f"   {engine.step_count:>9,} 步"
    )


if __name__ == "__main__":
    print("milsim 引擎基准（纯 Python，单线程）")
    print("-" * 78)
    bench_dense(200_000, 600 * SECOND)
    print()
    bench_sparse(3600 * SECOND)
    print()
    bench_mixed(1_000, 2 * SECOND, 600 * SECOND)
    print("-" * 78)
