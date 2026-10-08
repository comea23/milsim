"""决策提供者契约。

设计动机
--------
本项目要支持四种决策模型：行为树、状态机、脚本、大模型。前三种是**同步**的
（给定态势立即产出动作），大模型是**异步**的（延迟 1~30 秒）。

如果为它们分别设计接口，调用方就要写两套代码；更糟的是，先用同步实现跑通、
再换成大模型时会到处崩——因为调用时序变了。

所以这里统一成一个**请求-回调**契约：实现可以在任意时刻回调，
同步实现就是"立刻回调"这一特例。

调用方必须遵守的铁律
--------------------
**在 ``request()`` 返回之前，调用方必须已把自身状态准备好。**

因为回调可能立即发生（同步实现），也可能 10 秒后从别的线程发生（大模型）。
按最坏情况写代码——先设好 ``pending_request_id``，再发请求：

    # 正确
    self._pending = req.request_id
    provider.request(req, self._on_decision)

    # 错误：回调若立即触发，_pending 还是旧值
    provider.request(req, self._on_decision)
    self._pending = req.request_id

线程契约
--------
``resolve`` 回调**可能在其他线程被调用**。回调内部禁止直接改仿真状态，
只能通过 ``engine.inbox.post(...)`` 把动作转交主线程。
详见 ``milsim.engine.inbox`` 的说明。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Protocol, runtime_checkable

from ..engine.time import SimTime

#: 决策来源标识
SOURCE_RULE = "rule"
SOURCE_LLM = "llm"
SOURCE_FALLBACK = "fallback"

#: 决策种类
KIND_TASK_ASSIGN = "task_assign"    # 任务分配
KIND_MANEUVER = "maneuver"          # 机动方案
KIND_ENGAGE = "engage"              # 交战决策
KIND_PRIORITY = "priority"          # 目标排序
KIND_ESCALATE = "escalate"          # 是否上报 / 请求支援


@dataclass(slots=True)
class DecisionRequest:
    """一次决策请求。

    ``situation`` 是给大模型看的自然语言态势摘要，``context`` 是给规则引擎
    和程序化逻辑用的结构化数据。两者并存不是冗余——规则引擎解析自然语言很
    吃力，大模型读结构化数据则浪费上下文。各取所需。
    """

    request_id: int
    sim_time: SimTime
    entity_id: int
    kind: str
    situation: str
    options: tuple[str, ...]
    context: dict[str, Any] = field(default_factory=dict)
    #: 决策时限，单位微秒（相对发起时刻）。超时后由 fallback 兜底。
    #: None 表示不限时（规则引擎常用）。
    deadline_us: int | None = None


@dataclass(slots=True)
class DecisionResponse:
    """一次决策结果。"""

    request_id: int
    action: str
    params: dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    source: str = SOURCE_RULE
    #: 从发起请求到产出结果的实际耗时（微秒）。
    #: 大模型这条会很长——这正是军事上说的"指挥周期"，是有意义的仿真量。
    latency_us: int = 0

    @property
    def is_fallback(self) -> bool:
        return self.source == SOURCE_FALLBACK


#: 回调签名。实现方在产出决策时调用。
ResolveFn = Callable[[DecisionResponse], None]


@runtime_checkable
class DecisionProvider(Protocol):
    """决策提供者。

    实现方只承诺两件事：
      1. ``request()`` 不阻塞调用者（同步实现可以立即回调，但不能阻塞）
      2. 每个请求恰好回调一次 ``resolve``

    第 2 点很重要：大模型可能既超时触发 fallback、又随后收到真实响应，
    实现方必须自己保证只有一个能胜出（见 llm.py 的 finish 门闩）。
    """

    def request(self, request: DecisionRequest, resolve: ResolveFn) -> None:
        ...


class DecisionError(Exception):
    """决策过程中的不可恢复错误。可恢复的错误应由实现方降级处理。"""
