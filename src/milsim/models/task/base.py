"""任务管理器（§5.19，v0.13.59）——AFSIM ``WsfTaskManager`` 的对应物。

读过 AFSIM 源码之后的对照（swdev/src/core/wsf/source/processor/）：

======================  ================================================
AFSIM                   milsim
======================  ================================================
``WsfTask``             :class:`Task`（taskId/taskType/target/资源/状态）
``WsfTaskManager``      :class:`TaskManager`（双账本 + 消息族 + 重传）
``TaskAssignMessage``   ``TASK_ASSIGN`` 报文（走 §5.15 通信网）
``TaskAck``（序列号）    ``TASK_ACK`` 报文
``TaskStatusMessage``   ``TASK_STATUS`` 报文
``TaskCancelMessage``   ``TASK_CANCEL`` 报文
``TaskManagerComponent`` **执行器协议**：``Component.TASKS`` + acquire/
                        poll/release（挂载顺序认领，替代组件角色查询）
``FireSalvo``→完成      ``Weapon`` 认领 ``attack``：齐射→弹终结→上报
``TrackDropped→取消``   目标实体注销 → 自动取消（我们按实体 ID 对账，
                        关联层在真值 ID 接口下不存在，§5.13）
``PendingMessageList``  ``_pending``：可靠重传（用户拍板①）
======================  ================================================

**与 §4.5 意图台账（``CommandLedger``）的关系**：那是**战略层**的意图
簿记（KIND_*、优先级、有效期、世界检查器），管"上级想让发生什么"；
本层是**实体级**的执行任务（AFSIM 语义），管"谁、干什么、干完没有"。
``KIND_ENGAGE`` 意图的执行器落点就是这里的 ``assign_task``（§5.18 已
预留这句话，本版兑现）。

三条 AFSIM 语义照搬，两条本版裁定
--------------------------------

1. **任务跟着目标生死走**：目标实体注销 ⇒ 在身任务自动收尾（attack 报
   ``complete/target_destroyed``，其余报 ``canceled/target_lost``）。
2. **任务建在通信之上**：跨实体指派走 §5.15 报文——路不通，任务就到
   不了；重传耗尽报 ``failed/comm_lost``。**同实体指派不走报文**（载具
   内部部件交互，军事上瞬时无损，与 ``store.post_message`` 同一口径）。
3. **执行器认领制**：任务管理器不干活，按挂载顺序问"谁认领
   ``attack``"——多挂载（SAM/SSM 两槽）自然形成选择链：前一个槽
   拒绝（类别不符/没弹），后一个槽接手。

裁定（用户拍板②③）：**任务类型覆盖全部组件**——attack（武器）/ move
（机动）/ track（航迹）/ jam（干扰）/ comm（传信）/ resupply（补给）；
**不做全局矩阵指派，但支持一对多**——一次 ``assign_task`` 给多个受令
实体各建一项任务，共享 ``group_id``，各任务独立生命周期。

★ 本版与 AFSIM 的显式差距：没有 ``OperatingLevel``（HITL 批准门槛——
人就在回路里下 assign，门槛无意义）；没有 ``on_task_*`` 脚本钩子
（Python 直接订阅 ``bus.task_changed``）；没有 QuantumTasker（用户拍板③，
全局矩阵指派不做）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ...engine import PRIORITY_COMMAND, EventResult
from ...services.params import Params
from ...services.type_registry import register_component
from ..component import Component

# -- 报文种类（与 TRACK_REPORT 同一层：字符串常量，进 store 的收件箱） ------

#: 指派（派令方 → 受令方）。载荷 = 任务的完整快照（:meth:`Task.to_dict`）。
TASK_ASSIGN = "task_assign"
#: 受令回执（受令方 → 派令方）。派令方收到即停重传。
TASK_ACK = "task_ack"
#: 取消（派令方 → 受令方）。
TASK_CANCEL = "task_cancel"
#: 状态回报（受令方 → 派令方）：accepted / active / 终态。
TASK_STATUS = "task_status"
#: 传信任务的报文载体（``comm`` 任务发的东西本身）。
COMM_NOTE = "comm_note"

# -- 任务状态机：assigned → active → complete / failed / canceled ------------

#: 已指派（等受令方回执 / 等本方执行器认领）。
ASSIGNED = "assigned"
#: 执行中（受令方执行器已认领）。
ACTIVE = "active"
#: 干完了（含 attack 打完收工）。
COMPLETE = "complete"
#: 干不成（无认领 / 通信耗尽 / 受令方阵亡）。
FAILED = "failed"
#: 取消（派令方撤令 / 目标消亡 / 被指派的任务自动收编）。
CANCELED = "canceled"

#: 终态集合。进终态即离账入史，报文再晚到也不翻案。
_TERMINAL = (COMPLETE, FAILED, CANCELED)

#: 归档上限：长推演的台账不能无界增长，只留最近这些条供查询。
HISTORY_CAP = 256


@dataclass(eq=False)
class Task:
    """一项实体级执行任务。**纯数据 + 一个上报出口**。

    ``report`` 是执行器唯一的汇报通道：任务管理器派发时把 ``_manager``
    缝上，执行器拿到任务后喊一声 ``task.report("complete", "arrived")``
    即可——不需要持有管理器引用，也不需要订阅任何总线信号。
    """

    task_id: int
    task_type: str
    #: 一对多指派的**组号**：一次 assign 的 N 项任务共享同一个组号，
    #: 撤令按组撤、查询按组查，组内各项任务的生命周期彼此独立。
    group_id: str
    assigner_id: int
    assignee_id: int
    #: 目标实体 ID。``move``/``comm`` 这类对点任务为 ``None``。
    target_id: int | None = None
    #: 任务参数（执行器各自解释）：move 用 ``points``，attack 用
    #: ``rounds``/``weapon_class``，comm 用 ``to``/``text``……
    payload: dict[str, Any] = field(default_factory=dict)
    status: str = ASSIGNED
    #: 给人看的一句补充：``arrived`` / ``target_destroyed`` / ``comm_lost``…
    sub: str = ""
    resource: str = ""
    assign_time_us: int = 0
    update_time_us: int = 0
    _manager: Any = field(default=None, repr=False, compare=False)

    def to_dict(self) -> dict[str, Any]:
        """上线报文 / 前端快照用的纯数据形态（**不含**管理器引用）。"""
        return {
            "task_id": self.task_id,
            "task_type": self.task_type,
            "group_id": self.group_id,
            "assigner_id": self.assigner_id,
            "assignee_id": self.assignee_id,
            "target_id": self.target_id,
            "payload": dict(self.payload),
            "status": self.status,
            "sub": self.sub,
            "resource": self.resource,
            "assign_time_us": self.assign_time_us,
            "update_time_us": self.update_time_us,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Task":
        """报文 → 任务。缺键按默认补——受令方不挑剔派令方的多余字段。"""
        return cls(
            task_id=int(data["task_id"]),
            task_type=str(data["task_type"]),
            group_id=str(data.get("group_id", "")),
            assigner_id=int(data["assigner_id"]),
            assignee_id=int(data["assignee_id"]),
            target_id=data.get("target_id"),
            payload=dict(data.get("payload") or {}),
            status=ASSIGNED,          # 状态不随报文走：受令方自己从 assigned 起算
            sub="",
            assign_time_us=int(data.get("assign_time_us", 0)),
        )

    def report(self, status: str, sub: str = "") -> None:
        """执行器上报进展/终态。没有管理器（离线构造）就静默。"""
        if self._manager is not None:
            self._manager.report(self, status, sub)

    def __repr__(self) -> str:
        return (
            f"<Task {self.task_id} {self.task_type} {self.status}"
            f"{'/' + self.sub if self.sub else ''}"
            f" 派{self.assigner_id}→受{self.assignee_id}"
            f"{' 目标' + str(self.target_id) if self.target_id is not None else ''}>"
        )


@dataclass(slots=True)
class _Pending:
    """一条在途可靠报文的重传凭据（AFSIM ``PendingMessage`` 的对应物）。"""

    kind: str
    task_id: int
    recipient_id: int
    payload: dict[str, Any]
    retries_left: int
    next_us: int
    sent_count: int = 1


@register_component("TASK_MANAGER")
class TaskManager(Component):
    """一个平台上的任务台账 + 收发机。**不装它，该实体不参与任务层。**

    想定挂 ``component task TASK_MANAGER``。装了它的实体可以：

    - **派令**：``sim.command(实体id, "assign_task", {...})``；
    - **收令**：别的实体的任务管理器把 ``TASK_ASSIGN`` 发过来（走
      §5.15 通信网），本实体按挂载顺序找认领的部件执行；
    - **撤令**：``cancel_task``（按任务 ID，一撕全撕：组内只撕那一项）。

    台账节拍默认 1 s（``PRIORITY_COMMAND`` 档，在决策之后、机动之前）：
    收报文 → 重传 → 轮询在身任务 → 目标/受令人死亡对账。
    """

    SLOT_HINT = "task"

    #: 运行期指令（§5.18 词汇表的 v0.13.59 扩充）。
    COMMANDS = {
        "assign_task": (
            "指派任务：args = {'task_type': 类型, 'assignees': 实体名/编号或其列表"
            "（缺省=自己）, 'target': 目标实体名/编号（对点任务必给）,"
            " 'payload': {...}, 'group': 组号（缺省自动编）}"
        ),
        "cancel_task": "取消任务：args = {'task_id': 编号}（撤在身或已派出的那一项）",
    }

    PARAMS = {
        #: 台账节拍（微秒）。1 s：与通信件的 2 s 投递拍错开——重传间隔才是
        #: 那个"一个时间步"的尺度，台账本身要转得比它快。
        "period": Params.duration(1_000_000, minimum=100_000),
        #: 可靠报文的**重传间隔**（微秒）。默认 2 s = 一个时间步（用户口径
        #: "一个时间步内均可完成通信"——一拍没到，下一拍再试）。
        "retry_interval": Params.duration(2_000_000, minimum=1),
        #: 重传次数上限。耗尽 ⇒ 任务 ``failed/comm_lost``（派令侧）或
        #: 回报/撤令作废（其余方向）。
        "max_retries": Params.integer(5, minimum=1),
    }

    __slots__ = (
        "_mount",
        "_period",
        "_retry_us",
        "_max_retries",
        "_seq",
        "_sent",
        "_received",
        "_history",
        "_pending",
        "_seen",
        "_retries",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self._mount = mount
        self._period = int(self.spec["period"])
        self._retry_us = int(self.spec["retry_interval"])
        self._max_retries = int(self.spec["max_retries"])
        self._seq = 0
        #: **我派出去的**（AFSIM ``mXmtdTaskList``）：task_id → Task。
        self._sent: dict[int, Task] = {}
        #: **我收到的**（``mRcvdTaskList``）：task_id → Task。
        self._received: dict[int, Task] = {}
        #: 终态归档（有界，防长推演无界增长）。
        self._history: list[Task] = []
        #: 在途可靠报文（两个方向共用一张重传表）。
        self._pending: list[_Pending] = []
        #: 受令方去重账： ``(派令方, task_id)`` 集合——重传副本只补 ack，
        #: 不重复派发（AFSIM 用序列号查重，我们用二元组，同一个意思）。
        self._seen: set[tuple[int, int]] = set()
        #: 重传计数（诊断）。
        self._retries = 0
        mount.every(self._period, self._tick, PRIORITY_COMMAND)
        #: 机动受阻要让人看得见（bus 里 ``mover_blocked`` 的承诺）：
        # 不改状态——机动件自己会重规划；台账只把"卡住了"广播出去。
        blocked = getattr(mount.bus, "mover_blocked", None)
        if blocked is not None:
            blocked.subscribe(self._on_mover_blocked)

    def _on_mover_blocked(self, entity_id: int, blocked: bool, reason: str) -> None:
        if not blocked:
            return
        for ledger in (self._sent, self._received):
            for task in ledger.values():
                if (task.status == ACTIVE and task.task_type == "move"
                        and task.assignee_id == entity_id):
                    self._emit(task, "blocked")

    # -- 台账节拍 ----------------------------------------------------------

    def _tick(self, engine: Any, event: Any) -> str:
        now = engine.now
        self._take_messages(now)
        self._retransmit(now)
        self._poll_tasks()
        self._reconcile_dead(now)
        return EventResult.RESCHEDULE

    # -- 收报文 ------------------------------------------------------------

    def _take_messages(self, now: int) -> None:
        """把收件箱里四种任务报文各归各位。

        ★ 用 :meth:`~milsim.services.store.EntityStore.take_messages_of_kind`
        **分种类取**而不是全取：平台上同时挂着航迹件时两者共用一个收件箱，
        全取会把 ``TRACK_REPORT`` 一起吃掉（与航迹件同一条纪律）。
        """
        store = self._mount.store
        me = self._mount.entity_id
        for message in store.take_messages_of_kind(me, TASK_ASSIGN):
            self._on_assign(message, now)
        for message in store.take_messages_of_kind(me, TASK_ACK):
            self._on_ack(message)
        for message in store.take_messages_of_kind(me, TASK_STATUS):
            self._on_status(message)
        for message in store.take_messages_of_kind(me, TASK_CANCEL):
            self._on_cancel(message, now)

    def _on_assign(self, message: Any, now: int) -> None:
        """收指派：查重 → 补 ack → 派给本方执行器。"""
        key = (message.sender_id, int(message.payload.get("task_id", -1)))
        if key in self._seen:
            self._send(TASK_ACK, key[1], message.sender_id,
                       {"task_id": key[1]}, now)     # 重传副本：只补 ack
            return
        self._seen.add(key)
        task = Task.from_dict(message.payload)
        task._manager = self
        task.update_time_us = now
        self._received[task.task_id] = task
        self._emit(task, "assigned")
        # 回执先于执行：派令方收到 ack 就停重传；执行成不成，后面有
        # TASK_STATUS 说话。先把回执发出去，认领失败也不影响"收到了"。
        self._send(TASK_ACK, task.task_id, task.assigner_id,
                   {"task_id": task.task_id}, now)
        self._dispatch(task, now)

    def _on_ack(self, message: Any) -> None:
        """收回执：停重传。任务是否真被认领，等 TASK_STATUS。"""
        task_id = int(message.payload.get("task_id", -1))
        self._pending[:] = [
            p for p in self._pending
            if not (p.kind == TASK_ASSIGN and p.task_id == task_id)
        ]

    def _on_status(self, message: Any) -> None:
        """收状态回报：更新派令侧账本。任务已归档 ⇒ 忽略（幂等）。"""
        task = self._sent.get(int(message.payload.get("task_id", -1)))
        if task is None:
            return
        status = str(message.payload.get("status", ""))
        if status not in (ASSIGNED, ACTIVE, *_TERMINAL):
            return
        changed = status != task.status or message.payload.get("sub", "") != task.sub
        task.status = status
        task.sub = str(message.payload.get("sub", ""))
        task.resource = str(message.payload.get("resource", task.resource))
        task.update_time_us = int(message.payload.get("update_time_us", 0))
        if changed:
            self._emit(task, status)
        if status in _TERMINAL:
            self._archive(task, self._sent)

    def _on_cancel(self, message: Any, now: int) -> None:
        """收撤令：撕掉在身的那一项，回报派令方。已终态 ⇒ 忽略（幂等）。"""
        task_id = int(message.payload.get("task_id", -1))
        task = self._received.get(task_id)
        if task is None or task.status in _TERMINAL:
            return
        self._release_executor(task)
        self.report(task, CANCELED, "assigner_canceled")

    # -- 派发与执行器 ------------------------------------------------------

    def _dispatch(self, task: Task, now: int) -> None:
        """把收到的任务派给本方执行器：按挂载顺序问，第一个接下的干。

        全都拒绝 ⇒ ``failed/unclaimed``——"没人认领"是受令方的姿态，
        必须让派令方知道，不能静默挂着。
        """
        why = "没有部件认领"
        for part in self._mount.entity.parts():
            if task.task_type not in getattr(part, "TASKS", ()):
                continue
            ok, why = part.acquire_task(task)
            if ok:
                task.status = ACTIVE
                task.sub = why
                task.resource = part.slot
                task.update_time_us = now
                self._emit(task, ACTIVE)
                if task.assigner_id != self._mount.entity_id:
                    self._report_message(task, now)
                return
        self.report(task, FAILED, f"unclaimed: {why}")

    def _part_by_slot(self, slot: str) -> Any:
        for part in self._mount.entity.parts():
            if part.slot == slot:
                return part
        return None

    def _release_executor(self, task: Task) -> None:
        if not task.resource:
            return
        part = self._part_by_slot(task.resource)
        if part is not None:
            part.release_task(task)

    def _poll_tasks(self) -> None:
        """轮询**执行侧**的在身任务：执行器说有变化就记账。

        ★ 执行侧 = ``assignee == 我``，**两个账本都要看**——本地指派
        （自己派给自己）的任务记在 ``_sent`` 里（它同时是派令侧与执行侧），
        只轮 ``_received`` 会让本地任务永远停在 active。

        ★ 轮询而不是回调：部件忘了接上报路径这种事，轮询的后果是
        "任务停在 active"（看得见），回调的后果是"什么都不发生"
        （看不见）。
        """
        me = self._mount.entity_id
        for ledger in (self._received, self._sent):
            for task in list(ledger.values()):
                if task.assignee_id != me:
                    continue
                if task.status != ACTIVE or not task.resource:
                    continue
                part = self._part_by_slot(task.resource)
                if part is None:
                    continue
                verdict = part.poll_task(task)
                if verdict is None:
                    continue
                status, sub = verdict
                if status in (COMPLETE, FAILED):
                    self.report(task, status, sub)

    def _reconcile_dead(self, now: int) -> None:
        """目标/受令人死亡对账（AFSIM ``TrackDropped`` 的对应物）。

        - 执行侧（含本地指派）：目标注销 ⇒ attack 报
          ``complete/target_destroyed``（打完了就是打完了，执行器来不及
          自己报也一样），其余任务 ``canceled/target_lost``；
        - 派令侧：受令人注销 ⇒ 没人执行也没人回报，``canceled/assignee_lost``。
        """
        registry = self._mount.registry
        me = self._mount.entity_id
        for ledger in (self._received, self._sent):
            for task in list(ledger.values()):
                if task.assignee_id != me:
                    continue
                if task.status != ACTIVE or task.target_id is None:
                    continue
                if registry.get(task.target_id) is None:
                    self._release_executor(task)
                    if task.task_type == "attack":
                        self.report(task, COMPLETE, "target_destroyed")
                    else:
                        self.report(task, CANCELED, "target_lost")
        for task in list(self._sent.values()):
            if task.status in (ASSIGNED, ACTIVE) and task.assignee_id != me:
                if registry.get(task.assignee_id) is None:
                    task.status = CANCELED
                    task.sub = "assignee_lost"
                    task.update_time_us = now
                    self._emit(task, CANCELED)
                    self._archive(task, self._sent)
                    self._pending[:] = [
                        p for p in self._pending if p.task_id != task.task_id
                    ]

    # -- 指派（运行期指令 + 内部入口） --------------------------------------

    def execute(self, cmd: str, args: dict) -> tuple[bool, str]:
        ok, why = super().execute(cmd, args)
        if not ok and cmd not in self.COMMANDS:
            return ok, why
        if cmd == "assign_task":
            return self._cmd_assign(args)
        if cmd == "cancel_task":
            return self._cmd_cancel(args)
        return False, f"指令 {cmd!r} 没有实现"

    def _cmd_assign(self, args: dict) -> tuple[bool, str]:
        """``assign_task``：解析 → 逐个受令人建任务 → 发报文/本地派发。"""
        task_type = str(args.get("task_type", "")).strip()
        if not task_type:
            return False, "assign_task 需要 task_type（attack/move/track/jam/comm/resupply）"

        registry = self._mount.registry
        raw = args.get("assignees")
        if raw is None:
            raw_list: list[Any] = [self._mount.entity_id]
        elif isinstance(raw, (str, int)) and not isinstance(raw, bool):
            raw_list = [raw]
        elif isinstance(raw, list) and raw:
            raw_list = raw
        else:
            return False, "assignees 要是实体名/编号或它们的非空列表"
        assignees: list[int] = []
        for item in raw_list:
            if isinstance(item, bool) or not isinstance(item, (str, int)):
                return False, f"assignees 里的 {item!r} 不是实体名或编号"
            if isinstance(item, int):
                if registry.get(item) is None:
                    return False, f"受令实体 {item} 不存在"
                assignees.append(item)
            else:
                found = registry.by_name(str(item))
                if found is None:
                    return False, f"受令实体 {item!r} 不在这份想定里"
                assignees.append(found.entity_id)

        target_id: int | None = None
        raw_target = args.get("target")
        if raw_target is not None:
            if isinstance(raw_target, bool) or not isinstance(raw_target, (str, int)):
                return False, "target 要是实体名或编号"
            if isinstance(raw_target, int):
                if registry.get(raw_target) is None:
                    return False, f"目标实体 {raw_target} 不存在"
                target_id = raw_target
            else:
                found = registry.by_name(str(raw_target))
                if found is None:
                    return False, f"目标 {raw_target!r} 不在这份想定里"
                target_id = found.entity_id

        payload = args.get("payload")
        if payload is not None and not isinstance(payload, dict):
            return False, "payload 要是字典"
        group = str(args.get("group") or "").strip()
        if not group:
            group = f"G{self._seq + 1}"

        now = self._mount.now
        ids: list[int] = []
        made: list[Task] = []
        for aid in assignees:
            task = Task(
                task_id=self._new_id(),
                task_type=task_type,
                group_id=group,
                assigner_id=self._mount.entity_id,
                assignee_id=aid,
                target_id=target_id,
                payload=dict(payload or {}),
                assign_time_us=now,
                update_time_us=now,
            )
            task._manager = self
            made.append(task)

        for task in made:
            self._sent[task.task_id] = task
            ids.append(task.task_id)
            if task.assignee_id == self._mount.entity_id:
                self._emit(task, ASSIGNED)
                self._dispatch(task, self._mount.now)
            elif self._has_task_manager(task.assignee_id):
                self._emit(task, ASSIGNED)
                self._send(TASK_ASSIGN, task.task_id, task.assignee_id,
                           task.to_dict(), task.assign_time_us)
            else:
                # 受令方没有任务台账——跨实体的令它接不住。当场记失败，
                # 比让报文死在收件箱里诚实。
                task.status = FAILED
                task.sub = "no_task_manager"
                self._emit(task, FAILED)
                self._archive(task, self._sent)
        n_ok = sum(1 for t in made if t.status != FAILED)
        return True, (
            f"指派 {len(made)} 项 {task_type} 任务（组 {group}），"
            f"{n_ok} 项已发出：{ids}"
        )

    def _cmd_cancel(self, args: dict) -> tuple[bool, str]:
        raw = args.get("task_id")
        if isinstance(raw, bool) or not isinstance(raw, int):
            return False, "cancel_task 需要 task_id（整数编号）"
        now = self._mount.now
        task = self._received.get(raw)
        if task is not None and task.status not in _TERMINAL:
            self._release_executor(task)
            self.report(task, CANCELED, "assigner_canceled")
            return True, f"任务 {raw} 已取消"
        task = self._sent.get(raw)
        if task is not None and task.status not in _TERMINAL:
            task.status = CANCELED
            task.sub = "cancel_sent"
            task.update_time_us = now
            self._emit(task, CANCELED)
            self._archive(task, self._sent)
            self._release_executor(task)
            if task.assignee_id != self._mount.entity_id:
                self._send(TASK_CANCEL, task.task_id, task.assignee_id,
                           {"task_id": task.task_id}, now)
            return True, f"任务 {raw} 撤令已发（受令方回报前先记 canceled）"
        hit = next((t for t in self._history if t.task_id == raw), None)
        if hit is not None:
            return False, f"任务 {raw} 已是终态（{hit.status}），不重复处理"
        return False, f"任务 {raw} 不在本台账里"

    def _has_task_manager(self, entity_id: int) -> bool:
        """受令方装没装任务台账。跨实体指派的前置检查。"""
        found = self._mount.registry.get(entity_id)
        if found is None:
            return False
        for part in getattr(found, "parts", lambda: [])():
            if isinstance(part, TaskManager):
                return True
        return False

    # -- 状态上报（执行器经 Task.report 走到这里） ---------------------------

    def report(self, task: Task, status: str, sub: str = "") -> None:
        """记账 + 广播 + （跨实体时）回报派令方。终态同时离账入史。"""
        now = self._mount.now
        changed = status != task.status or sub != task.sub
        task.status = status
        if sub:
            task.sub = sub
        task.update_time_us = now
        if changed:
            self._emit(task, status)
        if status in _TERMINAL:
            ledger = (self._received if task.task_id in self._received
                      else self._sent)
            self._archive(task, ledger)
            if task.assigner_id != self._mount.entity_id:
                self._report_message(task, now)

    def _report_message(self, task: Task, now: int) -> None:
        self._send(
            TASK_STATUS, task.task_id, task.assigner_id,
            {
                "task_id": task.task_id,
                "status": task.status,
                "sub": task.sub,
                "resource": task.resource,
                "update_time_us": task.update_time_us,
            },
            now,
        )

    # -- 可靠发送（用户拍板①） ----------------------------------------------

    def _send(self, kind: str, task_id: int, recipient_id: int,
              payload: dict[str, Any], now: int) -> None:
        """把一条任务报文送上路。路 ∈ {通信网, 直投}，凭是否建网定：

        - **建了网**（§5.15）：报文进**在途队列**，由通信件按拍投递——
          路不通就压着，派令侧按拍重传（``retry_interval`` 一拍一发），
          耗尽 ``max_retries`` 记失败。这就是"任务建在通信之上"。
        - **没建网**：不限制 = 直投收件箱（与 ``post_message`` 同一口径，
          旧想定/单机推演零负担），无需重传。
        """
        comm = self._mount.comm_service()
        store = self._mount.store
        me = self._mount.entity_id
        if comm is None or comm.is_empty:
            store.post_message(me, recipient_id, kind,
                               sent_at=now, deliver_at=now,
                               payload=dict(payload))
            return
        store.enqueue_message(me, recipient_id, kind,
                              sent_at=now, deliver_at=now,
                              payload=dict(payload))
        self._pending.append(_Pending(
            kind=kind,
            task_id=task_id,
            recipient_id=recipient_id,
            payload=dict(payload),
            retries_left=self._max_retries,
            next_us=now + self._retry_us,
        ))

    def _retransmit(self, now: int) -> None:
        """到点的可靠报文重发；耗尽 ⇒ 作废。返回值即重传次数（诊断）。

        ★ 重发**不查拓扑**：路通不通由投递阶段判（§5.15 deliver），发信
        方看不出对方被压了——那本来就是真实世界的样子。压着压着路通了，
        报文就过去了；一直不通，``max_retries`` 耗尽认输。
        """
        still: list[_Pending] = []
        for p in self._pending:
            if now < p.next_us:
                still.append(p)
                continue
            if p.retries_left <= 0:
                self._give_up(p, now)
                continue
            p.retries_left -= 1
            p.sent_count += 1
            p.next_us = now + self._retry_us
            self._retries += 1
            self._mount.store.enqueue_message(
                self._mount.entity_id, p.recipient_id, p.kind,
                sent_at=now, deliver_at=now, payload=dict(p.payload),
            )
            still.append(p)
        self._pending = still

    def _give_up(self, p: _Pending, now: int) -> None:
        if p.kind != TASK_ASSIGN:
            return          # ack/状态/撤令丢了：下轮重传副本会再兜底
        task = self._sent.get(p.task_id)
        if task is None or task.status in _TERMINAL:
            return
        task.status = FAILED
        task.sub = "comm_lost"
        task.update_time_us = now
        self._emit(task, FAILED)
        self._archive(task, self._sent)

    # -- 账面 ---------------------------------------------------------------

    def _new_id(self) -> int:
        """任务 ID = 实体 ID × 2²⁰ + 本台账序号——全网唯一且可读出出处。"""
        self._seq += 1
        return self._mount.entity_id * (1 << 20) + self._seq

    def _emit(self, task: Task, action: str) -> None:
        emit = getattr(self._mount.bus, "task_changed", None)
        if emit is not None:
            emit.emit(task, action)

    def _archive(self, task: Task, ledger: dict[int, Task]) -> None:
        ledger.pop(task.task_id, None)
        self._history.append(task)
        if len(self._history) > HISTORY_CAP:
            del self._history[: len(self._history) - HISTORY_CAP]

    # -- 查询（测试 / 前端 / 上层智能体都走这三个） ---------------------------

    def snapshot(self) -> list[dict[str, Any]]:
        """台账全量快照：在途 + 归档，每项带 ``ledger`` 标签。"""
        out: list[dict[str, Any]] = []
        for ledger, tag in ((self._sent, "sent"), (self._received, "received"),
                            (self._history, "history")):
            for task in ledger.values() if isinstance(ledger, dict) else ledger:
                d = task.to_dict()
                d["ledger"] = tag
                out.append(d)
        return out

    def statistics(self) -> dict[str, int]:
        """诊断快照："令发出去了怎么没动静"靠这几个数归因。"""
        by_status: dict[str, int] = {}
        for ledger in (self._sent, self._received, self._history):
            for task in ledger.values() if isinstance(ledger, dict) else ledger:
                by_status[task.status] = by_status.get(task.status, 0) + 1
        return {
            "sent": len(self._sent),
            "received": len(self._received),
            "pending": len(self._pending),
            "retries": self._retries,
            "history": len(self._history),
            **by_status,
        }

    def __repr__(self) -> str:
        stats = self.statistics()
        return (
            f"<TaskManager eid={self.entity_id} "
            f"派出 {stats['sent']} / 在身 {stats['received']} / "
            f"在途 {stats['pending']} / 重传 {stats['retries']}>"
        )


#: 框架任务层清单（models/__init__ 汇总用）。
FRAMEWORK_TASKS: tuple[type, ...] = (TaskManager,)


__all__ = [
    "ASSIGNED",
    "ACTIVE",
    "CANCELED",
    "COMPLETE",
    "FAILED",
    "COMM_NOTE",
    "FRAMEWORK_TASKS",
    "TASK_ACK",
    "TASK_ASSIGN",
    "TASK_CANCEL",
    "TASK_STATUS",
    "Task",
    "TaskManager",
]
