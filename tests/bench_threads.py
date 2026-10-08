"""多线程收益实测。

回答一个具体问题：**本项目里哪些工作真的能靠多线程加速？**

CPython 标准构建有 GIL，纯 Python 计算不能并行。所以"要不要多线程"不能
一概而论，得看工作类型：

  - 纯 Python 计算（事件派发、模型逻辑）   → GIL 锁死，线程无收益
  - numpy 数值计算（网格批量运算）        → ufunc 释放 GIL，接近线性加速
  - IO 等待（大模型 HTTP 调用）           → GIL 不阻塞等待，线程有效
  - 进程级并行（蒙特卡洛多批次）          → 绕开 GIL，线性加速

这个基准把前两条量化出来，作为架构决策的依据。

    python tests/bench_threads.py
"""

from __future__ import annotations

import multiprocessing as mp
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


WORKERS = min(8, os.cpu_count() or 4)


def python_cpu_task(seed: int, rounds: int = 400_000) -> int:
    """纯 Python 计算，代表事件派发 / 模型逻辑这类工作。"""
    total = 0
    for i in range(rounds):
        total = (total + i * i) % 1_000_003
    return total + seed


def numpy_cpu_task(seed: int, size: int = 2_000_000) -> float:
    """numpy 数值计算，代表网格批量运算。

    刻意做成**计算密集**（链式 20 次 ufunc，内存访问量不变）。
    如果只做一次 sqrt，瓶颈会落在内存带宽上而不是 GIL 上，
    测出来的加速比反映的是内存子系统，不是并行能力——那会误导架构决策。
    """
    arr = np.arange(size, dtype=np.float64) + seed
    for _ in range(20):
        arr = np.sqrt(arr + 1.0)
    return float(arr.sum())


def bench_serial(fn, count: int) -> float:
    started = time.perf_counter()
    for i in range(count):
        fn(i)
    return time.perf_counter() - started


def bench_threaded(fn, count: int, workers: int) -> float:
    started = time.perf_counter()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        list(pool.map(fn, range(count)))
    return time.perf_counter() - started


def verdict_of(speedup: float) -> str:
    # 阈值不用 2.0 那么武断：1.3~2.0 是"能并行但被内存带宽或启动开销吃掉
    # 一部分"的常见区间，笼统说成"无效"会掩盖真实情况。
    if speedup >= 2.0:
        return "显著"
    if speedup >= 1.3:
        return "有限"
    return "无收益"


def report(label: str, fn, count: int) -> float:
    serial = bench_serial(fn, count)
    threaded = bench_threaded(fn, count, WORKERS)
    speedup = serial / threaded if threaded else 0.0
    print(
        f"  {label:<12} 串行 {serial:>6.2f}s    "
        f"{WORKERS} 线程 {threaded:>6.2f}s    "
        f"加速比 {speedup:>5.2f}x   {verdict_of(speedup)}"
    )
    return speedup


#: 多进程任务的默认计算量。必须放在模块顶层——
#: Windows 下 multiprocessing 用 spawn，任务函数要能被 pickle，
#: 局部函数（闭包）做不到，会报 "Can't get local object"。
PROCESS_TASK_ROUNDS = 24_000_000


def process_task(seed: int) -> int:
    """供多进程池调用的顶层任务函数。"""
    return python_cpu_task(seed, PROCESS_TASK_ROUNDS)


def bench_process_pool(workers: int = 8, tasks: int = 8) -> None:
    """多进程并行，代表蒙特卡洛多批次。

    这是绕开 GIL 的**唯一**手段。注意 Windows 用 spawn 启动，
    每个进程要重新 import 主模块，启动开销约 0.5 s，
    所以单批任务不能太短，否则收益全被启动开销吃掉——
    这一点对蒙特卡洛的批次划分有直接影响。
    """
    started = time.perf_counter()
    for i in range(tasks):
        process_task(i)
    serial = time.perf_counter() - started

    started = time.perf_counter()
    with mp.Pool(workers) as pool:
        pool.map(process_task, range(tasks))
    parallel = time.perf_counter() - started

    speedup = serial / parallel if parallel else 0.0
    print(
        f"  {'多进程':<12} 串行 {serial:>6.2f}s    "
        f"{workers} 进程 {parallel:>6.2f}s    "
        f"加速比 {speedup:>5.2f}x   {verdict_of(speedup)}"
    )


def bench_io_style(count: int = 8, delay: float = 0.25) -> None:
    """IO 等待，代表大模型 HTTP 调用。"""
    def wait(_: int) -> None:
        time.sleep(delay)

    started = time.perf_counter()
    for i in range(count):
        wait(i)
    serial = time.perf_counter() - started

    started = time.perf_counter()
    bench_threaded(wait, count, count)
    threaded = time.perf_counter() - started

    print(
        f"  {'IO 等待':<12} 串行 {serial:>6.2f}s    "
        f"{count} 线程 {threaded:>6.2f}s    "
        f"加速比 {serial/threaded:>5.2f}x   有效"
    )


if __name__ == "__main__":
    print(
        f"并行收益实测（Python {sys.version.split()[0]}，"
        f"CPU {os.cpu_count()} 核，工作线程 {WORKERS}）"
    )
    print("-" * 78)

    report("纯 Python", python_cpu_task, 8)

    if np is not None:
        report("numpy 数值", numpy_cpu_task, 8)
    else:
        print("  numpy 未安装，跳过数值并行测试")

    bench_io_style()
    bench_process_pool()

    print("-" * 78)
    gil_enabled = getattr(sys, "_is_gil_enabled", None)
    print(
        "  GIL 状态: "
        + ("启用（标准构建）" if (gil_enabled and gil_enabled())
           else "关闭（free-threading 构建）")
    )
