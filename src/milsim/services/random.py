"""确定性随机流。

为什么必须用 hashlib 派生种子
-----------------------------
Python 内置的 ``hash()`` 对字符串**带随机化**（`PYTHONHASHSEED`），
同一个字符串在两个进程里返回不同的值。拿它派生种子，"同种子可复现"
会在跨进程时静默失效——本地跑两次一样，换个终端跑就变了，
而这种错误几乎不可能靠人工发现。

``blake2b`` 是密码学哈希，输入相同则输出永远相同，跨进程、跨版本、跨平台一致。

为什么要按 (实体, 用途) 分流
----------------------------
如果所有模型共用一条随机流，改动**任何一处**代码都会连带改变后续所有
随机数的取值——加一个传感器探测判定，整个机动模块的行为都变了。
回归测试彻底失去意义。

分流后每个实体、每类行为各有一条独立流：改感知不影响机动，
改 A 单位不影响 B 单位。这是"同一种子两次推演逐字节一致"能成立的前提。
"""

from __future__ import annotations

import hashlib
import random
from enum import IntEnum
from itertools import count
from typing import Iterator


class Stream(IntEnum):
    """随机流用途。新增用途请**追加**，不要插在中间——

    插在中间会改变已有用途的编号，进而改变所有派生种子，
    让所有历史想定的推演结果全部失效。
    """

    INIT = 0        # 想定初始化、布势扰动
    SENSOR = 1      # 感知：探测判定、虚警、量测噪声
    MOVER = 2       # 机动：速度扰动、路径选择
    ENGAGE = 3      # 交战：命中判定、毁伤程度
    COMMS = 4       # 通信：丢包、时延
    DECISION = 5    # 决策：规则中的掷骰、行为树随机节点
    DAMAGE = 6      # 故障与损耗


#: 默认流的编号，等价于 ``Stream.INIT``。
DEFAULT_STREAM = int(Stream.INIT)


def seed_for_run(base_seed: int, run_number: int) -> int:
    """为蒙特卡洛的第 ``run_number`` 次运行派生主种子。

    用哈希而不是简单相加（``base + run``），因为相邻的整数种子在
    某些 PRNG 下会生成相关序列——批量推演时表现为"每一批的结果都差不多"，
    看起来像模型收敛，实际是随机数不够随机。
    """
    if run_number < 0:
        raise ValueError(f"运行编号不能为负：{run_number}")
    key = f"run:{base_seed}:{run_number}".encode("utf-8")
    return int.from_bytes(hashlib.blake2b(key, digest_size=8).digest(), "little")


class RandomPool:
    """按 ``(归属者, 用途)`` 派生独立随机流的池子。

    用法::

        pool = RandomPool(master_seed=42)

        # 每个实体、每类行为各拿一条流
        radar_rng = pool.stream(entity_id=12, stream=Stream.SENSOR)
        if radar_rng.random() < 0.7:
            detected = True

    ``stream()`` 对同一个键**总是返回同一个实例**，所以状态是连续的——
    不要指望它每次给你一条全新的流。
    """

    __slots__ = ("master_seed", "_streams", "_lookups")

    def __init__(self, master_seed: int = 0) -> None:
        self.master_seed = int(master_seed)
        self._streams: dict[tuple[str, int], random.Random] = {}
        self._lookups = 0

    # -- 种子派生 ----------------------------------------------------------

    def seed_for(self, owner: "int | str", stream: int = DEFAULT_STREAM) -> int:
        """派生一个 64 位种子。相同输入永远得到相同结果。"""
        key = f"{self.master_seed}|{owner}|{int(stream)}".encode("utf-8")
        return int.from_bytes(hashlib.blake2b(key, digest_size=8).digest(), "little")

    # -- 取流 --------------------------------------------------------------

    def stream(self, owner: "int | str", stream: int = DEFAULT_STREAM) -> random.Random:
        """取得（必要时创建）指定归属者、指定用途的随机流。"""
        cache_key = (str(owner), int(stream))
        rng = self._streams.get(cache_key)
        if rng is None:
            rng = random.Random(self.seed_for(owner, stream))
            self._streams[cache_key] = rng
        self._lookups += 1
        return rng

    def entity(self, entity_id: int) -> "EntityStreams":
        """取得某个实体的全部随机流，按用途访问。"""
        return EntityStreams(self, entity_id)

    # -- 生命周期 ----------------------------------------------------------

    def reset(self) -> None:
        """清空所有流，回到初始状态。

        重跑同一个想定必须调用它——否则第二次运行会接着第一次的状态
        继续取随机数，结果完全不同。
        """
        self._streams.clear()

    @property
    def stream_count(self) -> int:
        return len(self._streams)

    def clear_unused(self, active_owners: "set[int] | None" = None) -> int:
        """释放不再需要的流，返回释放数量。

        实体被销毁后它的流不会再被访问，但会一直占着内存。
        推演中大量单位生灭时（比如导弹），这个清理是必要的。
        """
        if active_owners is None:
            removed = len(self._streams)
            self._streams.clear()
            return removed
        active = {str(o) for o in active_owners}
        stale = [k for k in self._streams if k[0] not in active]
        for key in stale:
            del self._streams[key]
        return len(stale)

    def __len__(self) -> int:
        return len(self._streams)

    def __repr__(self) -> str:
        return f"<RandomPool seed={self.master_seed} 流数={len(self._streams)}>"


class EntityStreams:
    """某个实体的随机流命名空间，避免到处写 ``pool.stream(eid, Stream.XXX)``。"""

    __slots__ = ("_pool", "_entity_id")

    def __init__(self, pool: RandomPool, entity_id: int) -> None:
        self._pool = pool
        self._entity_id = entity_id

    @property
    def sensor(self) -> random.Random:
        return self._pool.stream(self._entity_id, Stream.SENSOR)

    @property
    def mover(self) -> random.Random:
        return self._pool.stream(self._entity_id, Stream.MOVER)

    @property
    def engage(self) -> random.Random:
        return self._pool.stream(self._entity_id, Stream.ENGAGE)

    @property
    def comms(self) -> random.Random:
        return self._pool.stream(self._entity_id, Stream.COMMS)

    @property
    def decision(self) -> random.Random:
        return self._pool.stream(self._entity_id, Stream.DECISION)

    @property
    def damage(self) -> random.Random:
        return self._pool.stream(self._entity_id, Stream.DAMAGE)

    def __repr__(self) -> str:
        return f"<EntityStreams 实体={self._entity_id}>"


class SequenceGenerator:
    """单调 ID 分配器。集中管理是为了让 ID 分配本身也可复现。

    不要在多处各写一个 ``itertools.count()``——那样 ID 会随调用顺序
    变化，而调用顺序又取决于字典遍历、事件顺序等，可复现性就丢了。
    """

    __slots__ = ("_counter", "_prefix")

    def __init__(self, start: int = 0, prefix: str = "") -> None:
        self._counter = count(start)
        self._prefix = prefix

    def next(self) -> int:
        return next(self._counter)

    def reset(self, start: int = 0) -> None:
        self._counter = count(start)

    def __iter__(self) -> Iterator[int]:
        return self._counter

    def __repr__(self) -> str:
        return f"<SequenceGenerator {self._prefix}>"


def bernoulli(rng: random.Random, probability: float) -> bool:
    """按概率掷一次。把 ``rng.random() < p`` 这行固定下来——语义清楚，
    也避免有人写成 ``<=`` 让 p=0 时仍有极微小概率命中。"""
    if probability <= 0.0:
        return False
    if probability >= 1.0:
        return True
    return rng.random() < probability


def jitter(rng: random.Random, value: float, fraction: float) -> float:
    """按比例给数值加扰动。``fraction=0.1`` 表示 ±10% 均匀分布。"""
    if fraction <= 0.0:
        return value
    return value * (1.0 + rng.uniform(-fraction, fraction))
