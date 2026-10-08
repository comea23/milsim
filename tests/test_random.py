"""确定性随机流测试。

核心是一件事：**同样的种子必须给出同样的推演**。所有测试都在为这个
性质服务——包括那条跨进程验证，它专门拦截"用内置 hash() 派生种子"这个
最难发现的错误。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from milsim.services.random import (
    SequenceGenerator,
    Stream,
    bernoulli,
    jitter,
    seed_for_run,
    RandomPool,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# 种子派生
# ---------------------------------------------------------------------------

def test_same_input_gives_same_seed() -> None:
    pool = RandomPool(42)
    assert pool.seed_for("alpha", 1) == pool.seed_for("alpha", 1)
    assert pool.seed_for(7, 2) == pool.seed_for(7, 2)


def test_different_input_gives_different_seed() -> None:
    pool = RandomPool(42)
    seeds = {
        pool.seed_for("alpha", 0),
        pool.seed_for("alpha", 1),
        pool.seed_for("beta", 0),
        pool.seed_for("alpha", 0) if False else pool.seed_for("alpha", 0),
        RandomPool(43).seed_for("alpha", 0),
    }
    assert len(seeds) == 4


def test_seed_is_stable_across_processes() -> None:
    """★ 种子必须跨进程稳定。

    Python 内置的 ``hash()`` 对字符串带随机化（PYTHONHASHSEED），
    同一个字符串在两个进程里返回不同值。用它派生种子，"同种子可复现"
    会在跨进程时静默失效——本地连跑两次一样，换个终端就变了。

    这个测试在两个不同的 PYTHONHASHSEED 下跑子进程，结果必须一致。
    它拦的正是"有人把 blake2b 换成 hash() 提速"这种改动。
    """
    code = (
        "import sys; sys.path.insert(0, 'src');"
        "from milsim.services.random import RandomPool;"
        "print(RandomPool(42).seed_for('alpha', 1))"
    )

    outputs = set()
    for hash_seed in ("0", "12345"):
        env = {**os.environ, "PYTHONHASHSEED": hash_seed}
        completed = subprocess.run(
            [sys.executable, "-c", code],
            cwd=PROJECT_ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )
        outputs.add(completed.stdout.strip())

    assert len(outputs) == 1, f"不同 PYTHONHASHSEED 下种子不一致：{outputs}"


def test_seed_for_run_is_spread_out() -> None:
    """相邻运行编号的种子要足够分散。

    用 ``base + run`` 这种简单相加会让相邻批次的随机序列相关，
    表现为"每一批结果都差不多"，看起来像模型收敛，实际是随机数不合格。
    """
    seeds = [seed_for_run(1000, i) for i in range(20)]
    assert len(set(seeds)) == 20

    # 相邻种子的差值不应该是常数
    diffs = [b - a for a, b in zip(seeds, seeds[1:])]
    assert len(set(diffs)) > 15


def test_seed_for_run_rejects_negative() -> None:
    with pytest.raises(ValueError, match="运行编号"):
        seed_for_run(0, -1)


# ---------------------------------------------------------------------------
# 流的独立性
# ---------------------------------------------------------------------------

def test_stream_returns_same_instance() -> None:
    """同一个键总是返回同一条流，状态连续。"""
    pool = RandomPool(1)
    a = pool.stream(10, Stream.SENSOR)
    b = pool.stream(10, Stream.SENSOR)
    assert a is b

    first = a.random()
    assert b.random() != first          # 接着往下取，不是重新开始


def test_different_streams_are_independent() -> None:
    """★ 改动一条流不该扰动其他流——这是分流的全部意义。

    如果所有模型共用一条随机流，加一个传感器判定就会连带改变机动模块
    的后续取值，回归测试彻底失去意义。
    """
    def run(extra_draw: bool) -> float:
        pool = RandomPool(7)
        pool.stream(1, Stream.SENSOR).random()
        if extra_draw:
            pool.stream(1, Stream.SENSOR).random()   # 同一实体，同一用途，多取一次
            pool.stream(2, Stream.SENSOR).random()   # 别的实体
            pool.stream(1, Stream.ENGAGE).random()   # 同一实体，别的用途
        return pool.stream(1, Stream.MOVER).random()  # 待观测的那条流

    assert run(False) == run(True)


def test_same_seed_reproduces_whole_sequence() -> None:
    """整条序列可复现，不只是第一个数。"""
    def sequence() -> list[float]:
        pool = RandomPool(2024)
        return [
            pool.stream(eid, Stream.SENSOR).random()
            for eid in (1, 2, 3, 1, 2, 3)
        ]

    assert sequence() == sequence()


def test_different_master_seed_differs() -> None:
    a = RandomPool(1).stream(1, Stream.SENSOR).random()
    b = RandomPool(2).stream(1, Stream.SENSOR).random()
    assert a != b


# ---------------------------------------------------------------------------
# 生命周期
# ---------------------------------------------------------------------------

def test_reset_restores_initial_state() -> None:
    """重跑想定必须先 reset，否则会接着上次的状态继续取数。"""
    pool = RandomPool(5)
    first = pool.stream(1, Stream.SENSOR).random()

    pool.reset()
    assert pool.stream_count == 0
    assert pool.stream(1, Stream.SENSOR).random() == first


def test_clear_unused_releases_streams() -> None:
    pool = RandomPool(3)
    for eid in range(10):
        pool.stream(eid, Stream.SENSOR)
    assert pool.stream_count == 10

    removed = pool.clear_unused(active_owners={1, 2, 3})
    assert removed == 7
    assert pool.stream_count == 3


def test_clear_unused_without_filter_drops_everything() -> None:
    pool = RandomPool(3)
    pool.stream(1, Stream.SENSOR)
    assert pool.clear_unused() == 1
    assert pool.stream_count == 0


# ---------------------------------------------------------------------------
# 实体命名空间
# ---------------------------------------------------------------------------

def test_entity_streams_expose_named_streams() -> None:
    pool = RandomPool(11)
    entity = pool.entity(42)

    assert entity.sensor is pool.stream(42, Stream.SENSOR)
    assert entity.mover is pool.stream(42, Stream.MOVER)
    assert entity.engage is pool.stream(42, Stream.ENGAGE)
    assert entity.comms is pool.stream(42, Stream.COMMS)
    assert entity.decision is pool.stream(42, Stream.DECISION)
    assert entity.damage is pool.stream(42, Stream.DAMAGE)


# ---------------------------------------------------------------------------
# 便利函数
# ---------------------------------------------------------------------------

def test_bernoulli_edges() -> None:
    pool = RandomPool(0)
    rng = pool.stream(1, Stream.SENSOR)

    assert bernoulli(rng, 0.0) is False
    assert bernoulli(rng, -1.0) is False
    assert bernoulli(rng, 1.0) is True
    assert bernoulli(rng, 2.0) is True


def test_bernoulli_is_deterministic() -> None:
    def draws() -> list[bool]:
        rng = RandomPool(9).stream(1, Stream.SENSOR)
        return [bernoulli(rng, 0.5) for _ in range(50)]

    assert draws() == draws()


def test_bernoulli_approximates_probability() -> None:
    rng = RandomPool(5).stream(1, Stream.SENSOR)
    hits = sum(bernoulli(rng, 0.3) for _ in range(5000))
    assert 0.27 < hits / 5000 < 0.33


def test_jitter_bounds_and_determinism() -> None:
    rng = RandomPool(4).stream(1, Stream.MOVER)
    values = [jitter(rng, 100.0, 0.1) for _ in range(500)]
    assert all(90.0 <= v <= 110.0 for v in values)
    assert len(set(values)) > 100          # 确实在扰动

    assert jitter(rng, 100.0, 0.0) == 100.0


def test_jitter_is_deterministic() -> None:
    def draws() -> list[float]:
        rng = RandomPool(8).stream(1, Stream.MOVER)
        return [jitter(rng, 50.0, 0.2) for _ in range(20)]

    assert draws() == draws()


# ---------------------------------------------------------------------------
# ID 分配
# ---------------------------------------------------------------------------

def test_sequence_generator_is_monotonic() -> None:
    gen = SequenceGenerator(start=100)
    assert [gen.next() for _ in range(4)] == [100, 101, 102, 103]


def test_sequence_generator_reset() -> None:
    gen = SequenceGenerator()
    gen.next()
    gen.next()
    gen.reset()
    assert gen.next() == 0


def test_stream_enum_values_are_stable() -> None:
    """枚举值不能变——变了会改变所有派生种子，让历史想定全部失效。

    新增用途请**追加**，不要插在中间。
    """
    assert Stream.INIT == 0
    assert Stream.SENSOR == 1
    assert Stream.MOVER == 2
    assert Stream.ENGAGE == 3
    assert Stream.COMMS == 4
    assert Stream.DECISION == 5
    assert Stream.DAMAGE == 6
