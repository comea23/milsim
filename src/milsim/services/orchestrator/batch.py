"""任务批次：一批任务的**原子性**载体（§4.5.6）。

要解决的问题
------------
一个宏观意图细化为 20 个实体的任务。如果第 10 条校验失败时前 9 条已经登记，
就得到一个物理上不可能出现的半成品态势：半个营在推进、半个营还在原地。
它比"完全没下达"更糟——**后续所有决策都会把这个错误状态当成真实初始条件**，
而且没有任何异常提示过这件事。

原子性的落点在登记，不在投递
----------------------------
§4.5.6 的原文说"全部校验通过后一次性投递"。实现时把"投递"拆成了两步，
因为**任务不是消息**（详见 ``ledger.py`` 模块头）：

1. **登记**（registry）：整批一起进入台账，或一条都不进。这一步必须原子。
2. **通知**（notice）：告诉执行方"有新任务"。这一步**允许**不齐——通信会
   延迟、丢包、绕路，那是物理事实，不该被原子性掩盖。

所以本类只负责第 1 步的载体与校验；第 2 步（什么时候、经不经过链路）属于
M5b 的接线。原子性管"要么全登记要么不登记"，管不住"投递有先后"——后者
靠 :attr:`TaskBatch.effective_at` 统一生效时刻来兜（见 ``task.py`` §③）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ...engine.time import SimTime
from .task import ASSIGNEE_LABELS, Task


@dataclass(slots=True)
class TaskBatch:
    """一批一起下达的任务。

    ``batch_id`` 由台账分配。批次本身也要编号，不是为了好看：复盘时要能回答
    "这 20 条是**同一次**细化出来的吗"，只有批次能回答——按时刻猜的话，
    同一秒内的两个批次会混在一起。
    """

    batch_id: int
    issuer: int
    created_at: SimTime
    tasks: tuple[Task, ...] = ()
    #: 来自哪条宏观意图。
    intent_id: int | None = None
    #: 本批的**统一生效时刻**。``None`` 表示各条按自己声明的时刻生效。
    #:
    #: 给值时**覆盖**批次内每条任务的 ``effective_at``；不给值就一条都不动。
    #: 覆盖而不是"取较晚者"：批次是一个整体，出现"部分任务 30 min 后生效、
    #: 部分立即生效"不是灵活，是没想清楚——真需要分波次就该下两个批次，
    #: 那时两个批次各有各的编号，复盘时也看得清。
    #:
    #: 也刻意**不做"缺省取登记时刻"**：那会把一条声明"T+30min 起"的任务
    #: 静默改成"立即生效"——不报错、看着完全正常，只是提前了半小时开始。
    effective_at: SimTime | None = None
    #: 登记时刻。``None`` 表示还没登记（校验未通过或尚未提交）。
    registered_at: SimTime | None = None
    note: str = ""
    #: 校验发现的问题。登记成功后清空。
    problems: list[str] = field(default_factory=list)

    @property
    def size(self) -> int:
        return len(self.tasks)

    @property
    def is_empty(self) -> bool:
        return not self.tasks

    @property
    def is_registered(self) -> bool:
        return self.registered_at is not None

    def assignees(self) -> tuple[tuple[str, int], ...]:
        """涉及哪些执行者，``(层级, ID)`` 升序去重。

        带上层级是必须的：编队 #3 与实体 #3 是两个不同的执行者，
        只返回整数会把它们混成同一个。
        """
        return tuple(
            sorted({(t.assignee_level, t.assignee) for t in self.tasks})
        )

    def by_level(self, level: str) -> tuple[Task, ...]:
        """按层级筛选。编队级任务要交给中观细化器，实体级直接进微观控制器。"""
        return tuple(t for t in self.tasks if t.assignee_level == level)

    def span(self) -> tuple[SimTime, SimTime | None]:
        """整批的时间跨度。用于画甘特图和估算推演时长。"""
        if not self.tasks:
            return (0, None)
        start = min(t.effective_at for t in self.tasks)
        ends = [t.deadline_at for t in self.tasks]
        finite = [e for e in ends if e is not None]
        return (start, min(finite) if finite and len(finite) == len(ends) else None)

    def describe(self) -> str:
        if self.is_empty:
            return (
                f"批次 #{self.batch_id}：空批次"
                f"（{'已登记' if self.is_registered else '未登记'}）"
            )

        levels: dict[str, int] = {}
        for task in self.tasks:
            levels[task.assignee_level] = levels.get(task.assignee_level, 0) + 1
        breakdown = "，".join(
            f"{ASSIGNEE_LABELS.get(k, k)} {v} 条" for k, v in sorted(levels.items())
        )

        state = (
            f"已于 T+{self.registered_at / 1_000_000:.1f}s 登记"
            if self.is_registered
            else "未登记"
        )
        intent = f"，源自意图 #{self.intent_id}" if self.intent_id else ""
        when = (
            f"统一生效 T+{self.effective_at / 1_000_000:.1f}s"
            if self.effective_at is not None
            else "各条按自身时刻生效"
        )
        return (
            f"批次 #{self.batch_id}（下达者 #{self.issuer}，{state}{intent}）："
            f"{self.size} 条任务 / {len(self.assignees())} 个执行者"
            f"（{breakdown}），{when}"
        )

    def to_dict(self) -> dict[str, Any]:
        start, end = self.span()
        return {
            "batch_id": self.batch_id,
            "issuer": self.issuer,
            "created_at": self.created_at,
            "intent_id": self.intent_id,
            "effective_at": self.effective_at,
            "registered_at": self.registered_at,
            "size": self.size,
            "assignees": [list(pair) for pair in self.assignees()],
            "span": [start, end],
            "note": self.note,
            "tasks": [t.to_dict() for t in self.tasks],
        }

    def __len__(self) -> int:
        return len(self.tasks)

    def __iter__(self):
        return iter(self.tasks)

    def __repr__(self) -> str:
        return (
            f"TaskBatch(#{self.batch_id} "
            f"{'已登记' if self.is_registered else '未登记'} "
            f"{self.size} 条)"
        )


__all__ = ["TaskBatch"]
