"""命令台账：意图与任务的编号、校验、原子登记、失效检测。

任务不是消息
------------
实现 M5a 时推翻了 §4.5.6 的一个说法。原文是"全部构造 + 校验通过后
**一次性投递**"，实现时发现投递这一步根本不该存在——**任务不是消息**：

| | 消息 | 任务 |
|---|---|---|
| 生命周期 | 一次性事件，投递完就从收件箱消失 | **有状态**，从待命到终态 |
| 归属 | 发给谁就归谁 | 始终归台账，执行方只是引用 |
| 能否重读 | ``take_messages`` 取走后就没有了 | 随时可查 |

把任务塞进收件箱有两个后果，都很隐蔽：

1. ``take_messages`` 是"取走并清空"（刻意的，见 ``store.py``）。执行方读
   一次之后，任务就只剩它手里的副本，而台账那份还在——**两份状态开始各自
   漂移**，且没有任何机制能发现。之后"这一连到底该在哪"就说不清了。
2. 编队 ID 与实体 ID 是两个空间。编队 #3 与实体 #3 同时存在是常态，而收件箱
   是按整数索引的——一条给编队 #3 的任务会落进实体 #3 的收件箱。

所以改成：**任务登记在台账，执行方按需查询**（``tasks_of`` / ``active_task_of``）。
原子性的落点随之从"投递"移到"登记"：整批一起进台账，或一条都不进。

通信依然要建模，只是位置变了：链路管的是"执行方**能不能知道**有新任务"，
不管"任务是什么"。这属于 M5b 的接线（通知走 ``store.enqueue_message``，
编队级通知路由到该编队指挥部的宿主实体）。

台账不认识世界
--------------
台账不知道实体在哪、活没活、编队有几个成员。需要世界知识的地方都从**参数**
传进来（``check`` / ``is_alive``），它不持有任何世界对象。

这不是洁癖：装配期 ``formations`` 会被整只换掉（``_build_command_chain``），
构造时抓下的引用会指向那只空树，而且**不报错**——投影器就踩过这个坑。
调用时才取引用，这个问题从根上不存在。

与 ``CommandChain`` 的分工
--------------------------
``CommandChain`` 管**关系**：谁能指挥谁、配属到谁名下。
``CommandLedger`` 管**文书**：下过哪些意图和任务、现在什么状态。
两张东西互不引用——关系里查不到"三营在打什么"，文书里也查不到"三营听谁的"。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, Sequence

from ...engine.time import SimTime
from ..bus import Signal
from .batch import TaskBatch
from .common import NOBODY, StateChange, TaskError
from .intent import (
    INTENT_ACTIVE,
    INTENT_LABELS,
    PRIORITY_ROUTINE,
    Intent,
)
from .task import (
    ASSIGNEE_LABELS,
    LEVEL_ENTITY,
    LEVEL_UNIT,
    STATE_LABELS,
    TASK_STATES,
    Objective,
    Task,
)

#: ``check`` 回调：给一条任务，返回问题（``str``）或没问题（``None``）。
CheckFn = Callable[[Task], "str | None"]
#: ``is_alive`` 回调：给一个实体 ID，回答它是否还在。
AliveFn = Callable[[int], bool]


@dataclass(frozen=True, slots=True)
class SweepReport:
    """一次巡检判定了什么失效。

    分成"任务"和"意图"两组而不是一个混起来的列表：两者的后续处理完全不同
    ——失效的任务要通知执行方停下，失效的意图要通知细化器别再往下拆。
    """

    at: SimTime
    tasks: tuple[Task, ...] = ()
    intents: tuple[Intent, ...] = ()

    @property
    def is_empty(self) -> bool:
        return not self.tasks and not self.intents

    def describe(self) -> str:
        if self.is_empty:
            return f"T+{self.at / 1_000_000:.1f}s 巡检：无失效"
        parts: list[str] = []
        if self.tasks:
            detail = "；".join(
                f"#{t.task_id} → {STATE_LABELS.get(t.state, t.state)}"
                f"（{t.history[-1].reason}）"
                for t in self.tasks
            )
            parts.append(f"任务 {len(self.tasks)} 条失效：{detail}")
        if self.intents:
            parts.append(
                f"意图 {len(self.intents)} 条失效："
                + "、".join(f"#{i.intent_id}" for i in self.intents)
            )
        return f"T+{self.at / 1_000_000:.1f}s 巡检：" + "；".join(parts)


class CommandLedger:
    """命令台账。一次推演一个。

    四件事：**编号**（唯一来源）、**校验**（整批一起过）、**登记**（原子）、
    **巡检**（失效检测）。
    """

    __slots__ = (
        "_tasks",
        "_intents",
        "_batches",
        "_task_seq",
        "_intent_seq",
        "_batch_seq",
        "on_issued",
        "on_invalidated",
    )

    def __init__(self) -> None:
        self._tasks: dict[int, Task] = {}
        self._intents: dict[int, Intent] = {}
        self._batches: dict[int, TaskBatch] = {}
        self._task_seq = 0
        self._intent_seq = 0
        self._batch_seq = 0
        #: 批次登记成功时发出，参数是 :class:`TaskBatch`。
        #: 给录制、前端、日志用——**不是**给执行方用的（它查台账）。
        self.on_issued: Signal = Signal("ledger_issued")
        #: 巡检判定失效时发出，参数是 :class:`SweepReport`。
        self.on_invalidated: Signal = Signal("ledger_invalidated")

    # ------------------------------------------------------------------
    # 意图
    # ------------------------------------------------------------------

    def draft_intent(
        self,
        *,
        statement: str,
        issuer: int = NOBODY,
        at: SimTime = 0,
        area: str = "",
        priority: int = PRIORITY_ROUTINE,
        effective_at: SimTime | None = None,
        expires_at: SimTime | None = None,
        constraints: Iterable[str] = (),
        supersedes: int | None = None,
        note: str = "",
    ) -> Intent:
        """构造一条意图并编号。**不登记**——登记走 :meth:`issue_intent`。"""
        self._intent_seq += 1
        intent = Intent(
            intent_id=self._intent_seq,
            statement=statement,
            issuer=issuer,
            area=area,
            priority=priority,
            issued_at=at,
            effective_at=at if effective_at is None else effective_at,
            expires_at=expires_at,
            constraints=tuple(constraints),
            supersedes=supersedes,
            note=note,
        )
        intent.history.append(
            StateChange(
                at=at,
                from_state="",
                state=INTENT_ACTIVE,
                by=issuer,
                reason="下达",
            )
        )
        return intent

    def validate_intent(self, intent: Intent) -> list[str]:
        problems = list(intent.validate())

        if intent.intent_id in self._intents:
            problems.append(
                f"意图 #{intent.intent_id} 已经登记过——重复登记会让两条同号"
                "意图并存"
            )

        if intent.supersedes is not None:
            old = self._intents.get(intent.supersedes)
            if old is None:
                problems.append(
                    f"意图 #{intent.intent_id} 声称取代意图 "
                    f"#{intent.supersedes}，但台账里没有这条"
                )
            elif old.is_terminal:
                problems.append(
                    f"意图 #{intent.intent_id} 声称取代意图 "
                    f"#{intent.supersedes}，但它已经是"
                    f"{INTENT_LABELS.get(old.state, old.state)}——"
                    "取代一条已经失效的意图通常意味着上游状态不同步"
                )

        return problems

    def issue_intent(self, intent: Intent) -> Intent:
        """校验 + 登记。顺带把被取代的那条置为失效。"""
        problems = self.validate_intent(intent)
        if problems:
            raise TaskError(
                "意图未通过校验，**没有下达**：\n  - " + "\n  - ".join(problems)
            )

        self._intents[intent.intent_id] = intent

        if intent.supersedes is not None:
            old = self._intents[intent.supersedes]
            # 校验阶段已确认它不是终态，这里不会再抛
            old.supersede(
                at=intent.issued_at,
                by_intent=intent.intent_id,
                by=intent.issuer,
            )

        return intent

    def get_intent(self, intent_id: int) -> Intent | None:
        return self._intents.get(intent_id)

    def intents(self) -> tuple[Intent, ...]:
        return tuple(self._intents[k] for k in sorted(self._intents))

    def running_intents(self, at: SimTime) -> tuple[Intent, ...]:
        """此刻仍在生效的意图，按 (优先级降序, 编号升序)。

        并列时的次序是**显式**的：同优先级就按编号，先下先算。让排序依赖
        字典遍历顺序的话，换个 ``PYTHONHASHSEED`` 结果就变了——而"同种子
        两次推演逐项一致"是这个项目的硬承诺（§6）。
        """
        live = [i for i in self._intents.values() if i.is_running_at(at)]
        live.sort(key=lambda i: (-i.priority, i.intent_id))
        return tuple(live)

    def primary_intent(self, at: SimTime) -> Intent | None:
        live = self.running_intents(at)
        return live[0] if live else None

    # ------------------------------------------------------------------
    # 任务：构造
    # ------------------------------------------------------------------

    def draft(
        self,
        *,
        kind: str,
        assignee: int,
        objective: Objective | None = None,
        assignee_level: str = LEVEL_ENTITY,
        issuer: int = NOBODY,
        at: SimTime = 0,
        effective_at: SimTime | None = None,
        deadline_at: SimTime | None = None,
        intent_id: int | None = None,
        parent_id: int | None = None,
        note: str = "",
    ) -> Task:
        """构造一条任务并编号。**不登记**——登记走 :meth:`issue`。

        编号在这里分配而不是在登记时：细化器常常要先把整批任务构造出来
        （还要在它们之间建立 ``parent_id`` 引用），那时就需要 ID 了。
        """
        self._task_seq += 1
        task = Task(
            task_id=self._task_seq,
            kind=kind,
            assignee=assignee,
            objective=objective if objective is not None else Objective(),
            assignee_level=assignee_level,
            issuer=issuer,
            issued_at=at,
            effective_at=at if effective_at is None else effective_at,
            deadline_at=deadline_at,
            intent_id=intent_id,
            parent_id=parent_id,
            note=note,
        )
        task.history.append(
            StateChange(
                at=at, from_state="", state=task.state, by=issuer, reason="下达"
            )
        )
        return task

    # ------------------------------------------------------------------
    # 任务：校验与登记
    # ------------------------------------------------------------------

    def validate(
        self,
        tasks: Sequence[Task],
        *,
        effective_at: SimTime | None = None,
        check: CheckFn | None = None,
    ) -> list[str]:
        """整批一次校验，返回**全部**问题。

        ``effective_at`` 是批次的统一生效时刻。传了它就按它校验时间约束，
        因为 :meth:`issue` 会用批次值**覆盖**每条任务自己的 ``effective_at``
        ——覆盖发生之后再报"截止早于生效"就晚了，那时整批已经登记。
        """
        problems: list[str] = []
        seen: set[int] = set()

        for task in tasks:
            head = f"任务 #{task.task_id}"
            problems.extend(f"{head}：{p}" for p in task.validate())

            if task.task_id in seen:
                problems.append(f"{head}：同一批次里出现了两次")
            if task.task_id in self._tasks:
                problems.append(
                    f"{head}：已经登记过——重复登记会让两条同号任务并存，"
                    "而下游按 ID 索引时只会看见后进来的那条"
                )
            seen.add(task.task_id)

            if task.intent_id is not None and task.intent_id not in self._intents:
                problems.append(
                    f"{head}：引用了不存在的意图 #{task.intent_id}"
                    "（意图要先于任务下达——顺序反了会让\"这条任务为了什么\""
                    "永远查不到）"
                )

            if effective_at is not None and task.deadline_at is not None:
                if task.deadline_at <= effective_at:
                    problems.append(
                        f"{head}：截止时刻（{task.deadline_at}）不晚于批次的"
                        f"统一生效时刻（{effective_at}）"
                    )

            if check is not None:
                extra = check(task)
                if extra:
                    problems.append(f"{head}：{extra}")

        problems.extend(self._overlap_problems(tasks))
        return problems

    def _overlap_problems(self, tasks: Sequence[Task]) -> list[str]:
        """同一执行者同一时刻只能有一条任务。

        与指挥关系那条"同一单位同一时刻只能有一个作战上级"是同一类约束：
        都是防**一个单位被要求同时做两件物理上做不到的事**。放过去的话，
        兵力统计会把同一支部队算两次，而且两条任务看起来都合规。
        """
        fresh = {t.task_id for t in tasks}
        pool: list[Task] = list(tasks)
        pool.extend(t for t in self._tasks.values() if t.task_id not in fresh)

        grouped: dict[tuple[str, int], list[Task]] = {}
        for task in pool:
            if task.is_terminal:
                continue
            grouped.setdefault((task.assignee_level, task.assignee), []).append(task)

        problems: list[str] = []
        for key in sorted(grouped):
            level, assignee = key
            items = sorted(grouped[key], key=lambda t: (t.window.start, t.task_id))
            for index, left in enumerate(items):
                for right in items[index + 1:]:
                    if not left.window.overlaps(right.window):
                        continue
                    both_new = left.task_id in fresh and right.task_id in fresh
                    where = "批次内" if both_new else "与已登记的任务冲突"
                    problems.append(
                        f"{ASSIGNEE_LABELS.get(level, level)} #{assignee} "
                        f"在同一时刻有两条任务（{where}）："
                        f"#{left.task_id} {left.window.describe()} 与 "
                        f"#{right.task_id} {right.window.describe()}——"
                        "同一执行者同一时刻只能执行一条"
                    )
        return problems

    def issue(
        self,
        tasks: Sequence[Task],
        *,
        issuer: int = NOBODY,
        at: SimTime = 0,
        intent_id: int | None = None,
        effective_at: SimTime | None = None,
        note: str = "",
        check: CheckFn | None = None,
    ) -> TaskBatch:
        """**原子登记**一整批任务。

        先校验、后登记，中间没有任何会失败的动作——原子性就是这么来的，
        不是靠回滚。**任何一条不合格，整批都不进台账**，错误消息里列出
        全部问题，一次改完。

        这不是讲究：半批生效会产生一个物理上不存在的态势，而后续所有决策
        都会拿它当真实初始条件（§4.5.6）。
        """
        batch_tasks = tuple(tasks)
        if not batch_tasks:
            raise TaskError(
                "空批次没有意义——细化器应当返回错误，而不是空任务列表。"
                "静默的空批会让上层以为\"已经下达了\"，实际什么都没发生"
            )

        stamp = at if effective_at is None else effective_at
        if effective_at is None:
            problems = self.validate(batch_tasks, check=check)
        else:
            problems = self.validate(batch_tasks, effective_at=stamp, check=check)
        if problems:
            raise TaskError(
                f"任务批次未通过校验，**{len(batch_tasks)} 条一条都没有登记**："
                "\n  - " + "\n  - ".join(problems)
            )

        # -- 以下只做赋值，不会失败 --
        #
        # **只有调用方显式给了统一生效时刻才覆盖。** 早先的写法是
        # "effective_at 缺省取登记时刻，然后一律覆盖"，结果一条本来声明
        # "T+30min 起"的任务，在被 issue 之后静默变成了"立即生效"——
        # 这种"参数缺省把已有信息抹掉"是最难查的一类：不报错，任务看着
        # 完全正常，只是提前了半小时开始。测试里一条窗口相接的任务就此
        # 变成重叠，才把它暴露出来。
        batch_effective: SimTime | None = effective_at
        if batch_effective is not None:
            for task in batch_tasks:
                task.effective_at = batch_effective

        self._batch_seq += 1
        resolved_intent = intent_id
        if resolved_intent is None:
            inherited = {t.intent_id for t in batch_tasks}
            resolved_intent = inherited.pop() if len(inherited) == 1 else None

        batch = TaskBatch(
            batch_id=self._batch_seq,
            issuer=issuer,
            created_at=at,
            tasks=batch_tasks,
            intent_id=resolved_intent,
            effective_at=batch_effective,
            registered_at=at,
            note=note,
        )
        self._batches[batch.batch_id] = batch
        for task in batch_tasks:
            self._tasks[task.task_id] = task

        self.on_issued.emit(batch)
        return batch

    # ------------------------------------------------------------------
    # 巡检
    # ------------------------------------------------------------------

    def sweep(self, at: SimTime, *, is_alive: AliveFn | None = None) -> SweepReport:
        """失效检测。返回**本次**被判失效的任务与意图。

        检测四件事：

        1. 执行者实体已不存在 → 中止
        2. 目标实体已不存在 → 失败
        3. 超过截止时刻仍未完结 → 过期
        4. 意图超过有效期 → 过期

        **先查世界、后查期限**：世界带来的原因更具体。"目标在 T+50 被打掉"
        比"期限在 T+600 到了"有用得多——后者只说明巡检间隔太长。

        **不做的事**：不检测"到点没开工"。开工必须由执行方上报
        （``task.py`` §②），台账代它宣布开工，就会造出"台账说在打、实际
        没人动"的不一致，而且不抛任何异常。

        ``is_alive`` 留空时只查期限——离线规划、单元测试都用得上。
        """
        invalidated: list[Task] = []

        for task in sorted(self._tasks.values(), key=lambda t: t.task_id):
            if task.is_terminal:
                continue

            if is_alive is not None:
                if task.assignee_level == LEVEL_ENTITY and not is_alive(task.assignee):
                    task.abort(
                        at=at,
                        reason=f"执行者实体 #{task.assignee} 已不存在",
                    )
                    invalidated.append(task)
                    continue

                target = task.objective.target
                if target is not None and not is_alive(target):
                    task.fail(
                        at=at,
                        reason=f"目标实体 #{target} 已不存在",
                    )
                    invalidated.append(task)
                    continue

            if task.deadline_at is not None and at >= task.deadline_at:
                task.expire(
                    at=at,
                    reason=f"超过截止时刻 T+{task.deadline_at / 1_000_000:.1f}s",
                )
                invalidated.append(task)

        expired: list[Intent] = []
        for intent in sorted(self._intents.values(), key=lambda i: i.intent_id):
            if intent.state != INTENT_ACTIVE or intent.expires_at is None:
                continue
            if at >= intent.expires_at:
                intent.expire(
                    at=at,
                    reason=f"超过有效期 T+{intent.expires_at / 1_000_000:.1f}s",
                )
                expired.append(intent)

        report = SweepReport(at=at, tasks=tuple(invalidated), intents=tuple(expired))
        if not report.is_empty:
            self.on_invalidated.emit(report)
        return report

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------

    def get(self, task_id: int) -> Task | None:
        return self._tasks.get(task_id)

    def get_batch(self, batch_id: int) -> TaskBatch | None:
        return self._batches.get(batch_id)

    def tasks(self) -> tuple[Task, ...]:
        return tuple(self._tasks[k] for k in sorted(self._tasks))

    def batches(self) -> tuple[TaskBatch, ...]:
        return tuple(self._batches[k] for k in sorted(self._batches))

    def open_tasks(self) -> tuple[Task, ...]:
        return tuple(t for t in self.tasks() if t.is_open)

    def tasks_of(
        self,
        assignee: int,
        *,
        level: str | None = None,
        at: SimTime | None = None,
        open_only: bool = True,
    ) -> tuple[Task, ...]:
        """某执行者的任务，按编号升序。

        ``level`` 强烈建议传：两个 ID 空间都是整数，不限定层级就会把编队 #3
        和实体 #3 的任务混在一起还给调用方。
        """
        out = [
            t
            for t in self._tasks.values()
            if t.assignee == assignee and (level is None or t.assignee_level == level)
        ]
        if open_only:
            out = [t for t in out if t.is_open]
        if at is not None:
            out = [t for t in out if t.window.covers(at)]
        return tuple(sorted(out, key=lambda t: t.task_id))

    def active_task_of(
        self, assignee: int, *, level: str, at: SimTime
    ) -> Task | None:
        """某执行者此刻正在做的那一条。

        重叠已在 :meth:`issue` 拦住，所以这里最多一条。真出现两条说明台账的
        不变量已经破了——**报错，不猜**。随便返回一条会让调用方拿着一个
        说不清来路的任务继续往下走。
        """
        candidates = self.tasks_of(assignee, level=level, at=at)
        if len(candidates) > 1:
            ids = "、".join(f"#{t.task_id}" for t in candidates)
            raise TaskError(
                f"{ASSIGNEE_LABELS.get(level, level)} #{assignee} 在 "
                f"T+{at / 1_000_000:.1f}s 同时命中多条任务（{ids}）——"
                "台账的重叠拦截失效了，这是不变量被破坏，不是正常情况"
            )
        return candidates[0] if candidates else None

    def clear(self) -> None:
        """清空。测试与推演复位用。序号**不重置**——重置会让新任务拿到
        与旧日志里相同的编号，两段日志再也分不开。"""
        self._tasks.clear()
        self._intents.clear()
        self._batches.clear()

    # ------------------------------------------------------------------
    # 输出
    # ------------------------------------------------------------------

    def statistics(self) -> dict[str, Any]:
        by_state = {state: 0 for state in sorted(TASK_STATES)}
        for task in self._tasks.values():
            by_state[task.state] = by_state.get(task.state, 0) + 1
        return {
            "intents": len(self._intents),
            "tasks": len(self._tasks),
            "open": sum(1 for t in self._tasks.values() if t.is_open),
            "batches": len(self._batches),
            "by_state": by_state,
        }

    def describe(self) -> str:
        stats = self.statistics()
        live = " / ".join(
            f"{STATE_LABELS.get(k, k)} {v}"
            for k, v in stats["by_state"].items()
            if v
        )
        return (
            f"台账：意图 {stats['intents']} 条 / 任务 {stats['tasks']} 条"
            f"（未完结 {stats['open']}）/ 批次 {stats['batches']} 个"
            + (f"｜{live}" if live else "")
        )

    def __len__(self) -> int:
        return len(self._tasks)

    def __contains__(self, task_id: object) -> bool:
        return task_id in self._tasks

    def __repr__(self) -> str:
        stats = self.statistics()
        return (
            f"CommandLedger(意图 {stats['intents']} / 任务 {stats['tasks']} / "
            f"批次 {stats['batches']})"
        )


def world_checker(
    registry: Any,
    store: Any,
    formations: Any = None,
) -> CheckFn:
    """按**世界现状**校验一条任务能否成立，做 :meth:`CommandLedger.issue` 的
    ``check`` 参数。

    查四件事：执行者是否存在、是否还活着、目标实体是否存在、下达者编队是否
    已定义。都是"下达之前就该挡住"的问题——等执行时才发现，任务已经在台账
    里躺了几分钟，而这几分钟里态势摘要会一直把它算作"在办事项"。

    参数用 ``Any`` 而不是具体类型，是为了让本模块**不导入** registry /
    store / formation：台账只在调用时刻用它们，编译期的类型依赖没必要存在。
    （更重要的是：装配期 ``formations`` 会被整只换掉，任何在台账构造时抓住
    的引用都会指向旧树且不报错——投影器就踩过这个坑。）

    返回**闭包**而不是让台账持有这三个对象，正是为了把"取引用"推迟到调用时。
    """
    def check(task: Task) -> str | None:
        if task.assignee_level == LEVEL_UNIT:
            if formations is None:
                return None
            if formations.get(task.assignee) is None:
                return f"执行者编队 #{task.assignee} 未定义"
        else:
            if registry.get(task.assignee) is None:
                return f"执行者实体 #{task.assignee} 未登记"
            if not store.is_alive(task.assignee):
                return f"执行者实体 #{task.assignee} 已失去战斗力"

        if task.issuer != NOBODY and formations is not None:
            if formations.get(task.issuer) is None:
                return f"下达者编队 #{task.issuer} 未定义"

        target = task.objective.target
        if target is not None and registry.get(target) is None:
            return f"目标实体 #{target} 未登记"

        return None

    return check


__all__ = ["AliveFn", "CheckFn", "CommandLedger", "SweepReport", "world_checker"]
