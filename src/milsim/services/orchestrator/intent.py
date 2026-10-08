"""战役级意图（§4.5.3②第一级）。

意图与任务的区别
----------------
| | 产出者 | 内容 | 能否多解 |
|---|---|---|---|
| ``Intent`` | 宏观智能体（一方一个） | "夺取 X 区域制空权，重点压制 Y 方向" | **能**——不含执行方式 |
| ``Task`` | 中观细化器（一编队一个） | "1 连于 T+300 s 前占领 1234 高地" | 基本不能——目标已定死 |

所以意图**必须**带一句自然语言的 ``statement``：它的价值恰恰在于"没有唯一
解法"，把意图结构化成"区域 + 兵力 + 时间"三件套之后，剩下的信息全没了。
但也不能只有自由文本——``area`` / ``priority`` / 有效期这几个槽位是细化器
的输入：它要先知道"往哪使劲、多急、什么时候算过期"，才谈得上分解。

一句话：**意图是"结构化的外壳 + 自然语言的内核"**，两者都不可省。

为什么优先级用三级而不是一个整数
--------------------------------
无上界的整数会立刻引出"5 和 7 差在哪"的问题，然后各人给出不同答案，
最后没有人真的用它排序。三级（例行 / 重要 / 紧急）没有歧义，也够用；
真需要更细的排序时，那是细化器的打分函数该干的事，不是意图字段。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ...engine.time import SimTime
from ..command import TimeWindow
from .common import NOBODY, StateChange, TaskError, label, names, transition

# ---------------------------------------------------------------------------
# 优先级
# ---------------------------------------------------------------------------

PRIORITY_ROUTINE = 1
PRIORITY_IMPORTANT = 2
PRIORITY_URGENT = 3

PRIORITIES = frozenset({PRIORITY_ROUTINE, PRIORITY_IMPORTANT, PRIORITY_URGENT})

PRIORITY_LABELS = {
    PRIORITY_ROUTINE: "例行",
    PRIORITY_IMPORTANT: "重要",
    PRIORITY_URGENT: "紧急",
}

# ---------------------------------------------------------------------------
# 意图状态
# ---------------------------------------------------------------------------

INTENT_ACTIVE = "active"
INTENT_SUPERSEDED = "superseded"
INTENT_EXPIRED = "expired"
INTENT_CANCELLED = "cancelled"

INTENT_STATES = frozenset(
    {INTENT_ACTIVE, INTENT_SUPERSEDED, INTENT_EXPIRED, INTENT_CANCELLED}
)

TERMINAL_INTENT_STATES = frozenset(
    {INTENT_SUPERSEDED, INTENT_EXPIRED, INTENT_CANCELLED}
)

#: 合法转移表。意图只有「生效中 → 三选一」，比任务简单得多——
#: 意图不存在"执行进度"，它要么有效，要么被新的取代、超期、或撤销。
INTENT_TRANSITIONS: dict[str, frozenset[str]] = {
    INTENT_ACTIVE: frozenset(
        {INTENT_SUPERSEDED, INTENT_EXPIRED, INTENT_CANCELLED}
    ),
    INTENT_SUPERSEDED: frozenset(),
    INTENT_EXPIRED: frozenset(),
    INTENT_CANCELLED: frozenset(),
}

INTENT_LABELS = {
    INTENT_ACTIVE: "生效中",
    INTENT_SUPERSEDED: "已被取代",
    INTENT_EXPIRED: "已过期",
    INTENT_CANCELLED: "已撤销",
}


@dataclass(slots=True)
class Intent:
    """一条战役级意图。

    ``intent_id`` 由台账分配，与 ``Task`` 同一套规矩（见 ``ledger.py``）。
    """

    intent_id: int
    #: 意图原文。**自然语言**，宏观智能体的产出，也是细化器的主要输入。
    statement: str
    #: 下达者（编队 ID）。``NOBODY`` 表示想定/导演部。
    issuer: int = NOBODY
    #: 主攻/关注区域名。可空——有些意图（"全旅转入防御"）不指向具体区域。
    area: str = ""
    priority: int = PRIORITY_ROUTINE
    issued_at: SimTime = 0
    effective_at: SimTime = 0
    #: 有效期截止。``None`` 表示不限——但**实践中少见**：不过期的意图
    #: 会在意图更替后与新的意图并存，下级不知道该听哪个（§9 第 7 条）。
    expires_at: SimTime | None = None
    #: 约束。给智能体读的一句话，暂不结构化。
    #:
    #: 为什么不做成结构化的：约束的形态太多（"不得越过 X 线""不得使用
    #: 某类弹药""保持无线电静默"），现在没有可信的结构化来源——硬造一套
    #: 只会得到一个没人填的字段。等出现某类约束被反复使用，再把它提上来。
    constraints: tuple[str, ...] = ()
    #: 被本意图取代的旧意图 ID。
    supersedes: int | None = None
    state: str = INTENT_ACTIVE
    note: str = ""
    history: list[StateChange] = field(default_factory=list)

    # -- 派生属性 ----------------------------------------------------------

    @property
    def window(self) -> TimeWindow:
        return TimeWindow(start=self.effective_at, end=self.expires_at)

    @property
    def is_terminal(self) -> bool:
        return self.state in TERMINAL_INTENT_STATES

    def is_running_at(self, at: SimTime) -> bool:
        """此刻是否仍在生效。**生效 = 状态有效且在有效期内**，两个条件都要。

        只看状态会漏掉"过期了但还没巡检到"的窗口期；只看时间会漏掉"已被
        撤销但仍然在有效期内"——那条更危险，因为细化的任务还在跑。
        """
        return self.state == INTENT_ACTIVE and self.window.covers(at)

    @property
    def priority_label(self) -> str:
        return PRIORITY_LABELS.get(self.priority, str(self.priority))

    # -- 校验 --------------------------------------------------------------

    def validate(self) -> list[str]:
        problems: list[str] = []

        if not self.statement.strip():
            problems.append(
                "意图原文不能为空——意图的价值就在这句话里，"
                "只剩区域和优先级的话，细化器无从判断该干什么"
            )

        if self.priority not in PRIORITIES:
            problems.append(
                f"未知的优先级 {self.priority!r}"
                f"（可用：{names(PRIORITIES, PRIORITY_LABELS)}）"
            )

        if self.state not in INTENT_STATES:
            problems.append(
                f"未知的意图状态 {self.state!r}"
                f"（可用：{names(INTENT_STATES, INTENT_LABELS)}）"
            )

        if self.expires_at is not None and self.expires_at <= self.effective_at:
            problems.append(
                f"有效期截止（{self.expires_at}）不晚于生效时刻"
                f"（{self.effective_at}）——这条意图一出生就是过期的"
            )

        if self.issuer < 0:
            problems.append(f"下达者 ID 不能为负，实际是 {self.issuer}")

        return problems

    # -- 状态转移 ----------------------------------------------------------

    def _change(
        self, target: str, *, at: SimTime, by: int = NOBODY, reason: str = ""
    ) -> StateChange:
        transition(
            current=self.state,
            target=target,
            table=INTENT_TRANSITIONS,
            labels=INTENT_LABELS,
            what="意图状态",
        )
        record = StateChange(
            at=at, from_state=self.state, state=target, by=by, reason=reason
        )
        self.state = target
        self.history.append(record)
        return record

    def supersede(
        self, *, at: SimTime, by_intent: int, by: int = NOBODY, reason: str = ""
    ) -> StateChange:
        """被新意图取代。``by_intent`` 是新意图的 ID——不记下来的话，
        "这条意图为什么失效"在复盘时查不到。"""
        if by_intent <= 0:
            raise TaskError(f"取代者的意图 ID 必须为正数，实际是 {by_intent}")
        return self._change(
            INTENT_SUPERSEDED,
            at=at,
            by=by,
            reason=reason or f"被意图 #{by_intent} 取代",
        )

    def expire(self, *, at: SimTime, reason: str = "") -> StateChange:
        return self._change(
            INTENT_EXPIRED, at=at, reason=reason or "超过有效期"
        )

    def cancel(self, *, at: SimTime, by: int = NOBODY, reason: str = "") -> StateChange:
        """撤销。与过期的区别是**主体**：撤销是人做的决定，过期是时间到了。"""
        return self._change(
            INTENT_CANCELLED, at=at, by=by, reason=reason or "上级撤销"
        )

    # -- 输出 --------------------------------------------------------------

    def describe(self) -> str:
        span = self.window.describe()
        area = f"，重点 {self.area}" if self.area else ""
        tail = "；".join(self.constraints)
        constraints = f"，约束：{tail}" if tail else ""
        state = INTENT_LABELS.get(self.state, self.state)
        return (
            f"意图 #{self.intent_id}（{state}，{self.priority_label}）："
            f"{self.statement}{area}（{span}{constraints}）"
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "intent_id": self.intent_id,
            "statement": self.statement,
            "issuer": self.issuer,
            "area": self.area,
            "priority": self.priority,
            "issued_at": self.issued_at,
            "effective_at": self.effective_at,
            "expires_at": self.expires_at,
            "constraints": list(self.constraints),
            "supersedes": self.supersedes,
            "state": self.state,
            "note": self.note,
            "history": [
                {
                    "at": c.at,
                    "from": c.from_state,
                    "to": c.state,
                    "by": c.by,
                    "reason": c.reason,
                }
                for c in self.history
            ],
        }

    def __repr__(self) -> str:
        return (
            f"Intent(#{self.intent_id} {self.priority_label} "
            f"{label(self.state, INTENT_LABELS)} {self.statement[:18]!r})"
        )


__all__ = [
    "INTENT_ACTIVE",
    "INTENT_CANCELLED",
    "INTENT_EXPIRED",
    "INTENT_LABELS",
    "INTENT_STATES",
    "INTENT_SUPERSEDED",
    "INTENT_TRANSITIONS",
    "Intent",
    "PRIORITIES",
    "PRIORITY_IMPORTANT",
    "PRIORITY_LABELS",
    "PRIORITY_ROUTINE",
    "PRIORITY_URGENT",
    "TERMINAL_INTENT_STATES",
]
