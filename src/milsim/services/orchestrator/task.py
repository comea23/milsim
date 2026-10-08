"""结构化任务与状态机（§4.5.2c）。

为什么不能用自由字典
--------------------
在 M5a 之前，跨实体传的只有 ``Message.payload: dict[str, Any]``。上级写
``{"target": 1234, "deadline": 300000000}``，下级读的时候要猜 key 名、猜单位。
三个后果，每一个都会拖慢排查：

1. **拼错不报错。** ``targt`` 要等执行时才炸，而且炸在下级那边——排查要跨
   两个实体、两份日志。对比想定层：参数拼错当场报"是否想写 target？"。
2. **没有状态归属。** "一连正在执行什么任务""谁判定它完成了"没有地方记。
   消息投递完就从收件箱消失了，副本飘在执行方手里，与下达方已无从对照。
3. **没有失效处理。** 目标实体被摧毁了，那条任务还挂着等执行。

所以任务是**结构化数据**：类型走白名单、目标要素按类型必填、时限走整数
微秒、状态转移走白名单表。

四个刻意的设计
--------------
**① 状态转移走白名单表，不用一组布尔标志位。**

``is_started / is_done / is_failed`` 这组标志位能表达出
``is_done and is_failed`` 这种不可能的组合，而且没人拦得住——它不报错，
只是让下游看到一个自相矛盾的任务。白名单表的非法转移当场报错。

**② 开工不由台账宣布。**

``PENDING → ACTIVE`` 只走 :meth:`Task.activate`，而它要求 ``by`` 是具体
的执行者。台账单方面宣布开工，会造出"台账说在打、实际没人动"——这类
不一致比任务失败更难查，因为它不产生任何异常，只让复盘时的兵力统计
一直是错的。

**③ 生效时刻与截止时刻是两件事。**

``effective_at`` 是"最早可开始"，不是"最晚"。一条命令细化成 20 条任务，
通信可能把它们分批送到（延迟、丢包、绕路）。若没有统一的生效时刻，先
收到的先动，阵线就变成斜的——而每条任务单独看都是合规的。

统一生效时刻让批次对**投递抖动**免疫。这是 §4.5.6 的原子性在通信层需要
补的那一半：原子性只保证"要么全登记要么不登记"，管不住"投递有先后"。

**④ 目标要素按类型必填。**

``engage`` 不写目标实体就是一句空话，可它在自由字典里完全合法。类型与
必需要素的对应表在 :data:`KIND_REQUIREMENTS`，构造时就校验。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from ...engine.time import SimTime
from ..command import TimeWindow
from .common import (
    LEVEL_ENTITY,
    LEVEL_UNIT,
    NOBODY,
    StateChange,
    TaskError,
    label,
    names,
    transition,
)

# ---------------------------------------------------------------------------
# 任务类型
# ---------------------------------------------------------------------------

#: 机动到指定位置。不强调"控制"，只要求到达。
KIND_MOVE = "move"
#: 占领 / 进驻指定区域。
KIND_OCCUPY = "occupy"
#: 侦察指定区域。要求"看得见"，不要求"拿得下"。
KIND_RECON = "recon"
#: 防守指定区域。
KIND_DEFEND = "defend"
#: 原地保持。目标要素为空——"原地"就是执行者当前位置。
KIND_HOLD = "hold"
#: 打击指定目标。
KIND_ENGAGE = "engage"
#: 回传情报。不要求位置，也不要求目标。
KIND_REPORT = "report"

TASK_KINDS = frozenset(
    {KIND_MOVE, KIND_OCCUPY, KIND_RECON, KIND_DEFEND, KIND_HOLD, KIND_ENGAGE,
     KIND_REPORT}
)

KIND_LABELS = {
    KIND_MOVE: "机动",
    KIND_OCCUPY: "占领",
    KIND_RECON: "侦察",
    KIND_DEFEND: "防守",
    KIND_HOLD: "原地保持",
    KIND_ENGAGE: "打击",
    KIND_REPORT: "回报",
}

#: 每个类型**必须**有的目标要素。取值是两个记号：
#:
#: - ``"place"``：必须有区域名或坐标（二选一）
#: - ``"target"``：必须有目标实体
#:
#: 空元组表示不要求——``hold`` 与 ``report`` 是仅有的两个。
#: 新增类型必须在这里出现（白名单式：不在表里的类型在构造时就被拒），
#: 免得"某个类型没人校验"这种事悄悄发生。
KIND_REQUIREMENTS: dict[str, frozenset[str]] = {
    KIND_MOVE: frozenset({"place"}),
    KIND_OCCUPY: frozenset({"place"}),
    KIND_RECON: frozenset({"place"}),
    KIND_DEFEND: frozenset({"place"}),
    KIND_HOLD: frozenset(),
    KIND_ENGAGE: frozenset({"target"}),
    KIND_REPORT: frozenset(),
}

#: 要素记号 → 缺了它时的说法。错误消息用。
REQUIREMENT_LABELS = {
    "place": "必须指明目标区域（area）或坐标（point）",
    "target": "必须指明目标实体（target）",
}

# ---------------------------------------------------------------------------
# 执行者层级
# ---------------------------------------------------------------------------
#
# ``LEVEL_UNIT`` / ``LEVEL_ENTITY`` 定义在 ``common`` 里（ID 空间标签），
# 这里沿用它们标注"N 级执行者"。同一个字符串在两处指的其实是同一件事：
# **那个 ID 属于哪个空间**。

ASSIGNEE_LEVELS = frozenset({LEVEL_UNIT, LEVEL_ENTITY})

ASSIGNEE_LABELS = {LEVEL_UNIT: "编队", LEVEL_ENTITY: "实体"}

# ---------------------------------------------------------------------------
# 任务状态
# ---------------------------------------------------------------------------

STATE_PENDING = "pending"
STATE_ACTIVE = "active"
STATE_COMPLETED = "completed"
STATE_FAILED = "failed"
STATE_ABORTED = "aborted"
STATE_EXPIRED = "expired"

TASK_STATES = frozenset(
    {STATE_PENDING, STATE_ACTIVE, STATE_COMPLETED, STATE_FAILED, STATE_ABORTED,
     STATE_EXPIRED}
)

#: 终态。进入之后**任何**转移都是错误，包括"再完成一次"。
TERMINAL_STATES = frozenset(
    {STATE_COMPLETED, STATE_FAILED, STATE_ABORTED, STATE_EXPIRED}
)

#: 合法转移表。
#:
#: ``pending`` 可以直接到 ``failed`` / ``expired`` / ``aborted``：任务还没
#: 开工，目标就被打掉了、或者期限过了，这两种都真实存在，不该逼它先"开工"。
#: 但 ``pending → completed`` **不在表里**——没开工就完成是笔误。
TASK_TRANSITIONS: dict[str, frozenset[str]] = {
    STATE_PENDING: frozenset(
        {STATE_ACTIVE, STATE_FAILED, STATE_ABORTED, STATE_EXPIRED}
    ),
    STATE_ACTIVE: frozenset(
        {STATE_COMPLETED, STATE_FAILED, STATE_ABORTED, STATE_EXPIRED}
    ),
    STATE_COMPLETED: frozenset(),
    STATE_FAILED: frozenset(),
    STATE_ABORTED: frozenset(),
    STATE_EXPIRED: frozenset(),
}

STATE_LABELS = {
    STATE_PENDING: "待命",
    STATE_ACTIVE: "执行中",
    STATE_COMPLETED: "已完成",
    STATE_FAILED: "已失败",
    STATE_ABORTED: "已中止",
    STATE_EXPIRED: "已过期",
}


# ---------------------------------------------------------------------------
# 目标要素
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Objective:
    """任务的**目标状态**。只写打到什么程度，不写怎么打（§4.5.3②）。

    这是"上级只下意图、不下动作"在任务层的落点：``occupy`` 一条高地，
    至于从哪个方向进、走哪条路、要不要先侦察，都是执行方的事。若这里出现
    "沿 045° 以 15 m/s 进入"，那这条任务已经越级写到了动作层。

    空间用**区域名或坐标**二选一，不是两者都能填：

    - ``area`` 指想定里 ``zone`` 声明的区域名。这是宏观/中观层该用的粒度
      ——"占领 1234 高地"里的 1234 高地就是一个区域。
    - ``point`` 是战区局部坐标（米）。这是中观细化后的粒度：细化器查完地图
      才算出具体路点。两种都存在是正常的，但**同一条任务只该用一种**，
      同时填会让"以哪个为准"变成隐式规则。
    """

    #: 区域名（想定的 ``zone`` 块）。与 ``point`` 互斥。
    area: str = ""
    #: 战区局部坐标，单位米。与 ``area`` 互斥。
    point: tuple[float, float] | None = None
    #: 达成判定的容差半径（米）。0 表示不判空间精度。
    radius_m: float = 0.0
    #: 目标实体 ID（实体 ID 空间）。``engage`` 类任务必须有。
    target: int | None = None
    #: 完成判据。给执行方和细化器读的一句话。
    #:
    #: 刻意保持自然语言、不结构化：判据是"怎么算完成"的表述，各兵种、
    #: 各任务类型差异极大，现在没有可信的结构化来源。硬造一套只会得到
    #: 一个没人填的字段。
    criterion: str = ""

    @property
    def has_place(self) -> bool:
        """是否有空间要素。``point`` 可能合法地取 ``(0.0, 0.0)``，
        所以判的是"是不是 None"，不是"真值"。"""
        return bool(self.area) or self.point is not None

    @property
    def place_kind(self) -> str:
        """``"area"`` / ``"point"`` / ``""``。同时填时以 ``area`` 为准并
        由 :meth:`Task.validate` 报错，这里不重复判断。"""
        if self.area:
            return "area"
        if self.point is not None:
            return "point"
        return ""

    def describe(
        self, *, names_of: Callable[[int, str], str] | None = None
    ) -> str:
        parts: list[str] = []
        if self.area:
            tail = f"（半径 {self.radius_m / 1000:.1f} km）" if self.radius_m else ""
            parts.append(f"区域 {self.area}{tail}")
        elif self.point is not None:
            x, y = self.point
            parts.append(f"坐标 ({x / 1000:.1f}, {y / 1000:.1f}) km")
        if self.target is not None:
            parts.append(f"目标实体 {show_id(self.target, LEVEL_ENTITY, names_of)}")
        if self.criterion:
            parts.append(f"判据「{self.criterion}」")
        return "，".join(parts) if parts else "无目标要素"

    def to_dict(self) -> dict[str, Any]:
        return {
            "area": self.area,
            "point": list(self.point) if self.point is not None else None,
            "radius_m": self.radius_m,
            "target": self.target,
            "criterion": self.criterion,
        }


def show_id(
    value: int, level: str, names_of: Callable[[int, str], str] | None
) -> str:
    """把 ID 换成名字，换不了就退回 ``#ID``。

    **层级必须一起传。** 编队 ID 与实体 ID 是两个都从 0 开始的整数空间，
    撞号是常态——只给一个整数的话，解析器无法判断该查编队表还是注册表，
    于是"三营"会被显示成恰好同号的那个平台。这类错不报错，只是把名字
    显示错了，最难被发现。
    """
    if names_of is None:
        return f"#{value}"
    try:
        return names_of(value, level)
    except Exception:  # 名字查不到不该让打印炸掉
        return f"#{value}"


def missing_requirements(kind: str, objective: Objective) -> list[str]:
    """按类型检查目标要素，返回**缺了什么**（人话）。

    公开出来是因为细化器需要它：大模型产出的任务先本地过一遍，能当场告诉
    它"``engage`` 必须有 target"，比让它自己从失败里猜要快得多。
    """
    required = KIND_REQUIREMENTS.get(kind)
    if required is None:
        return [f"不认识的任务类型 {kind!r}（可用：{names(TASK_KINDS, KIND_LABELS)}）"]

    missing: list[str] = []
    for token in sorted(required):
        if token == "place" and not objective.has_place:
            missing.append(REQUIREMENT_LABELS[token])
        elif token == "target" and objective.target is None:
            missing.append(REQUIREMENT_LABELS[token])
    return missing


# ---------------------------------------------------------------------------
# 任务
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class Task:
    """一条结构化任务。

    ``task_id`` 由 :class:`~milsim.services.orchestrator.ledger.TaskLedger`
    分配——**不自己编号**。编号要有唯一来源，否则两条同号任务会同时存在，
    而下游按 ID 索引时只会看见后写进去的那条。
    """

    task_id: int
    kind: str
    assignee: int
    objective: Objective
    #: 执行者层级。``assignee`` 是编队 ID 还是实体 ID 靠它区分。
    #:
    #: 这个字段不能省。两个 ID 空间都是整数，编队 #3 和实体 #3 同时存在是
    #: 常态；只存一个整数的话，"同一执行者同一时刻两条任务"的查重会把它们
    #: 判成同一个执行者，报到上级那里就是两条来源不明的冲突告警。
    assignee_level: str = LEVEL_ENTITY
    #: 下达者（编队 ID）。``NOBODY`` 表示想定/导演部。
    issuer: int = NOBODY
    #: 下达时刻。
    issued_at: SimTime = 0
    #: 最早可开始时刻（统一生效时刻）。送达更早也不许提前动。
    effective_at: SimTime = 0
    #: 截止时刻。``None`` 表示不限期。
    deadline_at: SimTime | None = None
    #: 来自哪条宏观意图。分解关系的上端。
    intent_id: int | None = None
    #: 上级任务 ID。分解关系的上端（任务拆任务时用）。
    parent_id: int | None = None
    state: str = STATE_PENDING
    #: 执行进度 0.0~1.0。由执行方上报，台账不推。
    progress: float = 0.0
    note: str = ""
    #: 状态变更记录。第一条是"下达"。
    history: list[StateChange] = field(default_factory=list)

    # -- 派生属性 ----------------------------------------------------------

    @property
    def window(self) -> TimeWindow:
        """有效区间 ``[effective_at, deadline_at)``。

        复用指挥关系那套 :class:`~milsim.services.command.TimeWindow`：
        右开在这里同样是刻意的——``deadline 8 h`` 与 ``from 8 h`` 相接时
        不该有重叠也不该有空隙。
        """
        return TimeWindow(start=self.effective_at, end=self.deadline_at)

    @property
    def is_terminal(self) -> bool:
        return self.state in TERMINAL_STATES

    @property
    def is_open(self) -> bool:
        return not self.is_terminal

    # -- 校验 --------------------------------------------------------------

    def validate(self) -> list[str]:
        """自身是否合规。返回**全部**问题，不是第一个。

        逐条报错是有意的：大模型产出的细化结果常有多处不合规，一次只报
        一个的话，来回几轮就没人愿意用了。
        """
        problems: list[str] = []

        if self.kind not in TASK_KINDS:
            problems.append(
                f"不认识的任务类型 {self.kind!r}"
                f"（可用：{names(TASK_KINDS, KIND_LABELS)}）"
            )
        else:
            problems.extend(missing_requirements(self.kind, self.objective))

        if self.assignee_level not in ASSIGNEE_LEVELS:
            problems.append(
                f"未知的执行者层级 {self.assignee_level!r}"
                f"（可用：{names(ASSIGNEE_LEVELS, ASSIGNEE_LABELS)}）"
            )

        if self.assignee < 0:
            problems.append(
                f"执行者 ID 不能为负，实际是 {self.assignee}"
                "（0 是合法编号——实体与编队的 ID 空间都从 0 开始）"
            )

        if self.objective.area and self.objective.point is not None:
            problems.append(
                "目标区域（area）与坐标（point）只能填一个——"
                "同时填会让\"以哪个为准\"变成隐式规则"
            )

        if self.objective.radius_m < 0.0:
            problems.append(f"容差半径不能为负，实际是 {self.objective.radius_m}")

        if self.objective.target is not None and self.objective.target < 0:
            problems.append(
                f"目标实体 ID 不能为负，实际是 {self.objective.target}"
            )

        if self.deadline_at is not None and self.deadline_at <= self.effective_at:
            problems.append(
                f"截止时刻（{self.deadline_at}）不晚于生效时刻"
                f"（{self.effective_at}）——这条任务一出生就是过期的"
            )

        if not 0.0 <= self.progress <= 1.0:
            problems.append(f"进度必须在 0~1 之间，实际是 {self.progress}")

        if self.state not in TASK_STATES:
            problems.append(
                f"未知的任务状态 {self.state!r}"
                f"（可用：{names(TASK_STATES, STATE_LABELS)}）"
            )

        if self.intent_id is not None and self.intent_id <= 0:
            problems.append(f"意图 ID 必须为正数，实际是 {self.intent_id}")

        return problems

    # -- 状态转移 ----------------------------------------------------------

    def _change(
        self,
        target: str,
        *,
        at: SimTime,
        by: int,
        by_level: str = LEVEL_UNIT,
        reason: str = "",
    ) -> StateChange:
        transition(
            current=self.state,
            target=target,
            table=TASK_TRANSITIONS,
            labels=STATE_LABELS,
            what="任务状态",
        )
        record = StateChange(
            at=at,
            from_state=self.state,
            state=target,
            by=by,
            by_level=by_level,
            reason=reason,
        )
        self.state = target
        self.history.append(record)
        return record

    def activate(self, *, at: SimTime, by: int) -> StateChange:
        """开工。**必须由执行方上报**（``by`` 填执行者）。

        见模块头 §②：台账或其他层代它宣布开工，会造出"台账说在打、实际
        没人动"的不一致，而且它不抛异常，只让兵力统计一直是错的。
        """
        if by == NOBODY:
            raise TaskError(
                f"任务 #{self.task_id} 开工必须由执行方上报，"
                "不能由系统代宣布——否则台账会显示在打、实际没人动"
            )
        return self._change(
            STATE_ACTIVE, at=at, by=by, by_level=self.assignee_level
        )

    def complete(self, *, at: SimTime, by: int, reason: str = "") -> StateChange:
        """完成。同样要求执行方上报——完成是有主体的事实，不是推断。"""
        if by == NOBODY:
            raise TaskError(
                f"任务 #{self.task_id} 完成必须由执行方上报，不能由系统代判"
            )
        return self._change(
            STATE_COMPLETED,
            at=at,
            by=by,
            by_level=self.assignee_level,
            reason=reason,
        )

    def fail(self, *, at: SimTime, by: int = NOBODY, reason: str) -> StateChange:
        """失败。可下达方判（目标没了），也可执行方报（打不动）。"""
        return self._change(STATE_FAILED, at=at, by=by, reason=reason)

    def abort(self, *, at: SimTime, by: int = NOBODY, reason: str) -> StateChange:
        """中止。通常是上级取消，或执行者已不存在。"""
        return self._change(STATE_ABORTED, at=at, by=by, reason=reason)

    def expire(self, *, at: SimTime, reason: str = "") -> StateChange:
        """过期。由台账在巡检时判。"""
        return self._change(
            STATE_EXPIRED, at=at, by=NOBODY, reason=reason or "超过截止时刻"
        )

    def set_progress(self, value: float, *, at: SimTime) -> None:
        """上报进度。不记历史——进度是连续量，每帧记一次会让历史爆掉。"""
        if not 0.0 <= value <= 1.0:
            raise TaskError(f"进度必须在 0~1 之间，实际是 {value}")
        self.progress = float(value)

    # -- 输出 --------------------------------------------------------------

    def describe(
        self, *, names_of: Callable[[int, str], str] | None = None
    ) -> str:
        """一行摘要。``names_of(id, level) -> str`` 可选，用于把 ID 换成名字。

        **层级一起传进去**的理由见 :func:`show_id`。默认不查名字：
        ``Task`` 不认识注册表（不该为了打印去依赖它），换名由持有世界的
        那一层做。
        """
        who = (
            f"{ASSIGNEE_LABELS.get(self.assignee_level, self.assignee_level)}"
            f" {show_id(self.assignee, self.assignee_level, names_of)}"
        )
        issuer = (
            "导演部" if self.issuer == NOBODY
            else show_id(self.issuer, LEVEL_UNIT, names_of)
        )
        progress = f"，进度 {self.progress:.0%}" if self.progress else ""
        return (
            f"任务 #{self.task_id} [{KIND_LABELS.get(self.kind, self.kind)}] "
            f"{who} ← {issuer}：{self.objective.describe(names_of=names_of)}"
            f"（{self.window.describe()}，"
            f"{STATE_LABELS.get(self.state, self.state)}{progress}）"
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_id": self.task_id,
            "kind": self.kind,
            "assignee": self.assignee,
            "assignee_level": self.assignee_level,
            "issuer": self.issuer,
            "objective": self.objective.to_dict(),
            "issued_at": self.issued_at,
            "effective_at": self.effective_at,
            "deadline_at": self.deadline_at,
            "intent_id": self.intent_id,
            "parent_id": self.parent_id,
            "state": self.state,
            "progress": self.progress,
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
            f"Task(#{self.task_id} {label(self.kind, KIND_LABELS)} "
            f"{self.assignee_level}:{self.assignee} {label(self.state, STATE_LABELS)})"
        )


__all__ = [
    "ASSIGNEE_LABELS",
    "ASSIGNEE_LEVELS",
    "KIND_DEFEND",
    "KIND_ENGAGE",
    "KIND_HOLD",
    "KIND_LABELS",
    "KIND_MOVE",
    "KIND_OCCUPY",
    "KIND_RECON",
    "KIND_REPORT",
    "KIND_REQUIREMENTS",
    "LEVEL_ENTITY",
    "LEVEL_UNIT",
    "Objective",
    "REQUIREMENT_LABELS",
    "STATE_ACTIVE",
    "STATE_ABORTED",
    "STATE_COMPLETED",
    "STATE_EXPIRED",
    "STATE_FAILED",
    "STATE_LABELS",
    "STATE_PENDING",
    "TASK_KINDS",
    "TASK_STATES",
    "TASK_TRANSITIONS",
    "TERMINAL_STATES",
    "Task",
    "missing_requirements",
    "show_id",
]
