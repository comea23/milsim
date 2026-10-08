"""分级控制骨架的测试（M5a，§4.5.2c / §4.5.3 / §4.5.6）。

重点不在"能不能跑"，而在**几个不报错的错误**是否被拦住：

- 半批生效（原子性）——不报错，只产生一个物理上不可能的态势
- 同一执行者同时两条任务——不报错，兵力统计把同一支部队算两次
- 编队 ID 与实体 ID 混用——不报错，任务发给错误的对象
- 台账代执行方宣布开工——不报错，兵力统计一直是错的
"""

from __future__ import annotations

import pytest

from milsim.errors import MilsimError
from milsim.services import (
    INTENT_ACTIVE,
    INTENT_CANCELLED,
    INTENT_EXPIRED,
    INTENT_SUPERSEDED,
    KIND_DEFEND,
    KIND_ENGAGE,
    KIND_HOLD,
    KIND_MOVE,
    KIND_OCCUPY,
    KIND_RECON,
    KIND_REPORT,
    LEVEL_ENTITY,
    LEVEL_UNIT,
    NOBODY,
    PRIORITY_IMPORTANT,
    PRIORITY_ROUTINE,
    PRIORITY_URGENT,
    STATE_ABORTED,
    STATE_ACTIVE,
    STATE_COMPLETED,
    STATE_EXPIRED,
    STATE_FAILED,
    STATE_PENDING,
    CommandLedger,
    Objective,
    TaskError,
    missing_requirements,
    world_checker,
)

H = 3_600_000_000
MIN = 60_000_000


# ---------------------------------------------------------------------------
# 桩：世界的最小替身
# ---------------------------------------------------------------------------

class _StubRegistry:
    def __init__(self, ids: tuple[int, ...] = ()) -> None:
        self._ids = set(ids)

    def get(self, entity_id: int):
        return object() if entity_id in self._ids else None


class _StubStore:
    def __init__(self, dead: tuple[int, ...] = ()) -> None:
        self._dead = set(dead)

    def is_alive(self, entity_id: int) -> bool:
        return entity_id not in self._dead


class _StubFormations:
    def __init__(self, ids: tuple[int, ...] = ()) -> None:
        self._ids = set(ids)

    def get(self, unit_id: int):
        return object() if unit_id in self._ids else None


def hold(ledger: CommandLedger, assignee: int, **kwargs):
    """一条最省事合法的任务：原地保持，不要求任何目标要素。"""
    kwargs.setdefault("at", 0)
    return ledger.draft(kind=KIND_HOLD, assignee=assignee, **kwargs)


# ---------------------------------------------------------------------------
# 任务类型与目标要素
# ---------------------------------------------------------------------------

def test_kind_whitelist_rejects_unknown_with_hint() -> None:
    ledger = CommandLedger()
    task = ledger.draft(kind="charge", assignee=1, at=0)
    problems = task.validate()
    assert any("不认识的任务类型" in p for p in problems)
    # 错误消息要列出可用值，否则用户得去翻源码
    assert KIND_OCCUPY in problems[0]


def test_occupy_requires_place() -> None:
    ledger = CommandLedger()
    task = ledger.draft(kind=KIND_OCCUPY, assignee=1, at=0)
    problems = task.validate()
    assert any("必须指明目标区域" in p for p in problems)


def test_engage_requires_target() -> None:
    ledger = CommandLedger()
    task = ledger.draft(kind=KIND_ENGAGE, assignee=1, objective=Objective(area="X"), at=0)
    assert any("必须指明目标实体" in p for p in task.validate())


@pytest.mark.parametrize("kind", [KIND_HOLD, KIND_REPORT])
def test_kinds_without_requirements_are_valid(kind: str) -> None:
    ledger = CommandLedger()
    task = ledger.draft(kind=kind, assignee=1, at=0)
    assert task.validate() == []


@pytest.mark.parametrize("kind", [KIND_MOVE, KIND_OCCUPY, KIND_RECON, KIND_DEFEND])
def test_place_kinds_accept_point_as_well_as_area(kind: str) -> None:
    ledger = CommandLedger()
    by_area = ledger.draft(kind=kind, assignee=1, objective=Objective(area="HILL"), at=0)
    by_point = ledger.draft(
        kind=kind, assignee=1, objective=Objective(point=(1000.0, -2000.0)), at=0
    )
    assert by_area.validate() == []
    assert by_point.validate() == []


def test_point_at_origin_is_still_a_place() -> None:
    """``(0.0, 0.0)`` 是合法坐标。判"是不是 None"，不能判真值——
    判真值会让战区原点上的任务被判成"没写位置"。"""
    ledger = CommandLedger()
    task = ledger.draft(kind=KIND_MOVE, assignee=1, objective=Objective(point=(0.0, 0.0)), at=0)
    assert task.objective.has_place
    assert task.validate() == []


def test_area_and_point_are_mutually_exclusive() -> None:
    ledger = CommandLedger()
    task = ledger.draft(
        kind=KIND_OCCUPY,
        assignee=1,
        objective=Objective(area="HILL", point=(1.0, 2.0)),
        at=0,
    )
    assert any("只能填一个" in p for p in task.validate())


def test_missing_requirements_is_usable_by_refiners() -> None:
    """细化器要能在本地先过一遍，而不是等 issue 抛错再猜。"""
    assert missing_requirements(KIND_ENGAGE, Objective()) == [
        "必须指明目标实体（target）"
    ]
    assert missing_requirements(KIND_OCCUPY, Objective(area="X")) == []
    assert "不认识的任务类型" in missing_requirements("charge", Objective())[0]


# ---------------------------------------------------------------------------
# 任务自身校验
# ---------------------------------------------------------------------------

def test_validate_reports_all_problems_not_just_the_first() -> None:
    """一次报全。只报第一个的话，大模型改一次只修一个错，
    来回几轮就没人愿意用了。"""
    ledger = CommandLedger()
    task = ledger.draft(
        kind="charge",
        assignee=-5,
        objective=Objective(area="A", point=(1.0, 2.0), radius_m=-1.0, target=-3),
        at=0,
        deadline_at=0,
    )
    problems = task.validate()
    assert len(problems) >= 5
    assert any("执行者 ID 不能为负" in p for p in problems)
    assert any("容差半径" in p for p in problems)
    assert any("目标实体 ID 不能为负" in p for p in problems)


def test_zero_is_a_valid_id() -> None:
    """实体与编队的 ID 空间**都从 0 开始**，0 是合法编号。

    这条不报错但很致命：把 0 当"没有"（哨兵值、判空、``if id:``），
    就会让"导演部"和"0 号实体"变成同一个东西。
    """
    ledger = CommandLedger()
    task = hold(ledger, 0, issuer=0)
    assert task.validate() == []
    assert task.assignee == 0

    striker = ledger.draft(
        kind=KIND_ENGAGE, assignee=0, objective=Objective(target=0), at=0
    )
    assert striker.validate() == []


def test_nobody_does_not_collide_with_id_zero() -> None:
    assert NOBODY < 0
    ledger = CommandLedger()
    task = hold(ledger, 0)
    task.abort(at=MIN, reason="上级取消")
    # by=NOBODY 记的是"系统判定"，不是"0 号实体判定"
    assert task.history[-1].by == NOBODY
    assert "由系统判定" in task.history[-1].describe()

    # 而真正由 0 号实体判定时，要显示出编号
    task2 = hold(ledger, 0)
    task2.abort(at=MIN, by=0, reason="执行方上报")
    assert "#0" in task2.history[-1].describe()


def test_history_marks_the_id_space_of_the_judge() -> None:
    """"谁判定的"必须带空间标签：编队 #0 与实体 #0 同号是常态，
    只有裸整数的话，"旅部下达"会被解析成恰好同号的那个平台。"""
    ledger = CommandLedger()
    task = hold(ledger, 0, assignee_level=LEVEL_ENTITY, issuer=0)

    # 下达记录：判定者是下达方，落在编队空间
    assert task.history[0].by == 0
    assert task.history[0].by_level == LEVEL_UNIT
    assert "编队#0" in task.history[0].describe()

    # 开工：执行方上报，落在执行者自己的空间
    task.activate(at=MIN, by=0)
    assert task.history[-1].by_level == LEVEL_ENTITY

    # 中止：上级/系统判定，回到编队空间
    task.abort(at=2 * MIN, reason="上级取消")
    assert task.history[-1].by_level == LEVEL_UNIT


def test_deadline_must_be_after_effective() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 1, effective_at=H, deadline_at=H)
    assert any("一出生就是过期的" in p for p in task.validate())


def test_progress_bounds_are_checked() -> None:
    task = CommandLedger().draft(kind=KIND_HOLD, assignee=1, at=0)
    with pytest.raises(TaskError, match="进度必须在 0~1"):
        task.set_progress(1.2, at=0)


def test_unknown_assignee_level_is_rejected() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 1, assignee_level="squad")
    assert any("未知的执行者层级" in p for p in task.validate())


# ---------------------------------------------------------------------------
# 状态机
# ---------------------------------------------------------------------------

def test_happy_path_pending_active_completed() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7, issuer=3)
    assert task.state == STATE_PENDING

    task.activate(at=MIN, by=7)
    assert task.state == STATE_ACTIVE

    task.complete(at=2 * MIN, by=7, reason="已到位")
    assert task.state == STATE_COMPLETED
    assert task.is_terminal


def test_pending_may_fail_directly() -> None:
    """还没开工目标就被打掉了——这是真实情形，不该逼它先"开工"。"""
    task = CommandLedger().draft(kind=KIND_HOLD, assignee=7, at=0)
    task.fail(at=MIN, reason="目标已消失")
    assert task.state == STATE_FAILED


def test_pending_to_completed_is_illegal() -> None:
    """没开工就完成是笔误。白名单表拦得住这类"没想到"的组合。"""
    task = CommandLedger().draft(kind=KIND_HOLD, assignee=7, at=0)
    with pytest.raises(TaskError, match="非法的任务状态转移"):
        task.complete(at=MIN, by=7)


@pytest.mark.parametrize(
    ("reach_state", "further"),
    [
        (STATE_ACTIVE, "complete_then_abort"),
        (STATE_FAILED, "abort"),
        (STATE_ABORTED, "complete"),
        (STATE_EXPIRED, "activate"),
    ],
)
def test_terminal_states_are_frozen(reach_state: str, further: str) -> None:
    """终态之后**任何**转移都是错误——包括"再完成一次"。

    刻意试的是**另一个**终态：试同一个会命中"重复设置"那条更具体的错误，
    而这里要验证的是"白名单表里终态的出边为空"。
    """
    task = CommandLedger().draft(kind=KIND_HOLD, assignee=7, at=0)
    if reach_state == STATE_ACTIVE:
        task.activate(at=MIN, by=7)
        if further == "complete_then_abort":
            task.complete(at=2 * MIN, by=7)
        assert task.state == STATE_COMPLETED
    elif reach_state == STATE_FAILED:
        task.fail(at=MIN, reason="打不动")
    elif reach_state == STATE_ABORTED:
        task.abort(at=MIN, reason="上级取消")
    else:
        task.expire(at=MIN)

    assert task.state in {STATE_COMPLETED, STATE_FAILED, STATE_ABORTED, STATE_EXPIRED}
    if further == "abort":
        with pytest.raises(TaskError, match="已是终态"):
            task.abort(at=2 * MIN, reason="再试一次")
    elif further == "complete":
        with pytest.raises(TaskError, match="已是终态"):
            task.complete(at=2 * MIN, by=7)
    else:
        with pytest.raises(TaskError, match="已是终态"):
            task.activate(at=2 * MIN, by=7)


def test_activating_twice_is_an_error_not_a_noop() -> None:
    """重复设置通常意味着在重放一段已经处理过的输入——那值得报出来。"""
    task = CommandLedger().draft(kind=KIND_HOLD, assignee=7, at=0)
    task.activate(at=MIN, by=7)
    with pytest.raises(TaskError, match="重复设置"):
        task.activate(at=2 * MIN, by=7)


def test_ledger_may_not_announce_start_on_behalf_of_executor() -> None:
    """台账代宣布开工 → "台账说在打、实际没人动"，而且不抛异常。"""
    task = CommandLedger().draft(kind=KIND_HOLD, assignee=7, at=0)
    with pytest.raises(TaskError, match="必须由执行方上报"):
        task.activate(at=MIN, by=NOBODY)
    with pytest.raises(TaskError, match="必须由执行方上报"):
        task.complete(at=MIN, by=NOBODY)


def test_history_records_who_and_why() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7, issuer=3, at=0)
    task.activate(at=MIN, by=7)
    task.fail(at=2 * MIN, by=3, reason="目标已不存在")

    assert [c.state for c in task.history] == [STATE_PENDING, STATE_ACTIVE, STATE_FAILED]
    assert task.history[0].by == 3 and task.history[0].reason == "下达"
    assert task.history[2].by == 3 and "目标已不存在" in task.history[2].reason
    assert "→" in task.history[1].describe()


def test_window_is_right_open() -> None:
    """``deadline 1h`` 与 ``from 1h`` 相接时不该重叠也不该有空隙。"""
    ledger = CommandLedger()
    first = hold(ledger, 7, effective_at=0, deadline_at=H)
    second = hold(ledger, 7, effective_at=H, deadline_at=2 * H)
    assert not first.window.overlaps(second.window)
    ledger.issue([first, second], at=0)


# ---------------------------------------------------------------------------
# 台账：编号与查询
# ---------------------------------------------------------------------------

def test_ids_are_sequential_and_deterministic() -> None:
    def build():
        ledger = CommandLedger()
        return [hold(ledger, i + 1, at=0).task_id for i in range(3)]

    assert build() == [1, 2, 3]
    assert build() == [1, 2, 3]


def test_ids_do_not_reset_after_clear() -> None:
    """重置序号会让新任务拿到与旧日志相同的编号，两段日志再也分不开。"""
    ledger = CommandLedger()
    hold(ledger, 1, at=0)
    ledger.clear()
    assert hold(ledger, 1, at=0).task_id == 2


def test_tasks_of_filters_by_level_and_openness() -> None:
    ledger = CommandLedger()
    entity_task = hold(ledger, 5)
    unit_task = hold(ledger, 5, assignee_level=LEVEL_UNIT)
    ledger.issue([entity_task, unit_task], at=0)

    assert len(ledger.tasks_of(5)) == 2
    assert ledger.tasks_of(5, level=LEVEL_ENTITY) == (entity_task,)
    assert ledger.tasks_of(5, level=LEVEL_UNIT) == (unit_task,)

    entity_task.abort(at=0, reason="取消")
    assert ledger.tasks_of(5, level=LEVEL_ENTITY) == ()
    assert ledger.open_tasks() == (unit_task,)


def test_active_task_of_returns_the_one_in_window() -> None:
    ledger = CommandLedger()
    earlier = hold(ledger, 7, effective_at=0, deadline_at=MIN)
    ledger.issue([earlier], at=0)
    later = hold(ledger, 7, effective_at=MIN, deadline_at=2 * MIN)
    ledger.issue([later], at=0)

    assert ledger.active_task_of(7, level=LEVEL_ENTITY, at=MIN // 2) is earlier
    assert ledger.active_task_of(7, level=LEVEL_ENTITY, at=MIN + 1) is later


def test_broken_overlap_invariant_raises_instead_of_guessing() -> None:
    """重叠已被 issue 拦住，所以 active_task_of 最多命中一条。
    真命中两条说明不变量破了——报错，不猜。随便返回一条会让调用方
    拿着说不清来路的任务继续往下走。"""
    ledger = CommandLedger()
    first = hold(ledger, 7)
    ledger.issue([first], at=0)
    intruder = hold(ledger, 7)
    # 绕过 issue 直接塞进去，模拟不变量被破坏
    ledger._tasks[intruder.task_id] = intruder

    with pytest.raises(TaskError, match="不变量被破坏"):
        ledger.active_task_of(7, level=LEVEL_ENTITY, at=0)


# ---------------------------------------------------------------------------
# 原子性
# ---------------------------------------------------------------------------

def test_issue_registers_the_whole_batch() -> None:
    ledger = CommandLedger()
    tasks = [hold(ledger, 100 + i, at=0) for i in range(5)]
    batch = ledger.issue(tasks, issuer=9, at=0)

    assert batch.size == 5
    assert batch.is_registered
    assert len(ledger) == 5
    assert ledger.get_batch(batch.batch_id) is batch


def test_one_bad_task_means_the_whole_batch_is_not_registered() -> None:
    """§4.5.6 的核心。半批生效会产生一个物理上不存在的态势，
    而后续所有决策都会拿它当真实初始条件。"""
    ledger = CommandLedger()
    good = [hold(ledger, 100 + i, at=0) for i in range(9)]
    bad = ledger.draft(kind="charge", assignee=200, at=0)

    with pytest.raises(TaskError) as excinfo:
        ledger.issue(good + [bad], issuer=9, at=0)

    message = str(excinfo.value)
    assert "10 条一条都没有登记" in message
    assert "不认识的任务类型" in message
    # 前 9 条也一条都不在
    assert len(ledger) == 0
    assert ledger.batches() == ()


def test_empty_batch_is_rejected() -> None:
    ledger = CommandLedger()
    with pytest.raises(TaskError, match="空批次没有意义"):
        ledger.issue([], at=0)


def test_duplicate_registration_is_rejected() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7)
    ledger.issue([task], at=0)
    with pytest.raises(TaskError, match="已经登记过"):
        ledger.issue([task], at=0)


def test_batch_stamps_unified_effective_time() -> None:
    """统一生效时刻**覆盖**每条任务自己的值——批次是一个整体。"""
    ledger = CommandLedger()
    tasks = [hold(ledger, 100 + i, effective_at=0) for i in range(3)]
    batch = ledger.issue(tasks, at=0, effective_at=30 * MIN)

    assert batch.effective_at == 30 * MIN
    assert all(t.effective_at == 30 * MIN for t in tasks)


def test_batch_effective_time_checked_against_deadlines() -> None:
    """覆盖发生之后再报"截止早于生效"就晚了——那时整批已经登记。"""
    ledger = CommandLedger()
    task = hold(ledger, 7, effective_at=0, deadline_at=10 * MIN)
    with pytest.raises(TaskError, match="不晚于批次的统一生效时刻"):
        ledger.issue([task], at=0, effective_at=30 * MIN)


def test_batch_inherits_intent_from_tasks() -> None:
    ledger = CommandLedger()
    intent = ledger.issue_intent(
        ledger.draft_intent(statement="夺取高地", issuer=1, at=0)
    )
    task = hold(ledger, 7, intent_id=intent.intent_id)
    batch = ledger.issue([task], issuer=1, at=0)
    assert batch.intent_id == intent.intent_id


def test_task_referencing_missing_intent_is_rejected() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7, intent_id=42)
    with pytest.raises(TaskError, match="引用了不存在的意图"):
        ledger.issue([task], at=0)


def test_issue_emits_signal_for_recorders() -> None:
    ledger = CommandLedger()
    seen = []
    ledger.on_issued.subscribe(seen.append)
    batch = ledger.issue([hold(ledger, 7)], issuer=1, at=0)
    assert seen == [batch]


# ---------------------------------------------------------------------------
# 同一执行者同一时刻只能有一条任务
# ---------------------------------------------------------------------------

def test_same_assignee_overlap_within_batch_is_rejected() -> None:
    ledger = CommandLedger()
    first = hold(ledger, 7, effective_at=0, deadline_at=2 * H)
    second = hold(ledger, 7, effective_at=H, deadline_at=3 * H)
    with pytest.raises(TaskError) as excinfo:
        ledger.issue([first, second], at=0)
    assert "同一执行者同一时刻只能执行一条" in str(excinfo.value)
    assert "批次内" in str(excinfo.value)


def test_sequential_tasks_are_allowed() -> None:
    ledger = CommandLedger()
    first = hold(ledger, 7, effective_at=0, deadline_at=H)
    second = hold(ledger, 7, effective_at=H, deadline_at=2 * H)
    ledger.issue([first, second], at=0)
    assert len(ledger) == 2


def test_overlap_with_an_already_registered_task_is_rejected() -> None:
    ledger = CommandLedger()
    ledger.issue([hold(ledger, 7, effective_at=0, deadline_at=4 * H)], at=0)
    with pytest.raises(TaskError, match="与已登记的任务冲突"):
        ledger.issue([hold(ledger, 7, effective_at=H, deadline_at=9 * H)], at=0)


def test_terminal_task_does_not_block_a_new_one() -> None:
    ledger = CommandLedger()
    old = hold(ledger, 7, effective_at=0, deadline_at=4 * H)
    ledger.issue([old], at=0)
    old.abort(at=MIN, reason="取消后重新下达")
    ledger.issue([hold(ledger, 7, effective_at=H, deadline_at=9 * H)], at=0)
    assert len(ledger.open_tasks()) == 1


def test_different_assignees_never_conflict() -> None:
    ledger = CommandLedger()
    ledger.issue(
        [hold(ledger, 7, effective_at=0, deadline_at=H),
         hold(ledger, 8, effective_at=0, deadline_at=H)],
        at=0,
    )
    assert len(ledger) == 2


def test_unit_and_entity_with_same_id_are_not_the_same_assignee() -> None:
    """两个 ID 空间都是整数。不区分层级的话，编队 #3 与实体 #3 会被判成
    同一个执行者，报到上级那里就是两条来源不明的冲突告警。"""
    ledger = CommandLedger()
    for_unit = hold(ledger, 3, assignee_level=LEVEL_UNIT, effective_at=0, deadline_at=H)
    for_entity = hold(ledger, 3, assignee_level=LEVEL_ENTITY, effective_at=0, deadline_at=H)
    batch = ledger.issue([for_unit, for_entity], at=0)

    assert batch.size == 2
    assert batch.assignees() == ((LEVEL_ENTITY, 3), (LEVEL_UNIT, 3))
    assert len(ledger.tasks_of(3, level=LEVEL_UNIT)) == 1
    assert len(ledger.tasks_of(3, level=LEVEL_ENTITY)) == 1


# ---------------------------------------------------------------------------
# check 回调与世界校验
# ---------------------------------------------------------------------------

def test_check_callback_blocks_task_before_registration() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 999)
    with pytest.raises(TaskError, match="编队 #999 不存在"):
        ledger.issue([task], at=0, check=lambda t: "编队 #999 不存在")


def test_world_checker_flags_unknown_assignee() -> None:
    check = world_checker(_StubRegistry([1]), _StubStore(), _StubFormations([1]))
    assert "未登记" in check(hold(CommandLedger(), 42))


def test_world_checker_flags_dead_assignee() -> None:
    check = world_checker(_StubRegistry([1]), _StubStore(dead=(1,)), _StubFormations([1]))
    assert "已失去战斗力" in check(hold(CommandLedger(), 1))


def test_world_checker_flags_unknown_unit() -> None:
    check = world_checker(_StubRegistry([1]), _StubStore(), _StubFormations([1]))
    task = hold(CommandLedger(), 9, assignee_level=LEVEL_UNIT)
    assert "执行者编队 #9 未定义" in check(task)


def test_world_checker_flags_unknown_target() -> None:
    check = world_checker(_StubRegistry([1]), _StubStore(), _StubFormations([1]))
    task = CommandLedger().draft(
        kind=KIND_ENGAGE, assignee=1, objective=Objective(target=77), at=0
    )
    assert "目标实体 #77 未登记" in check(task)


def test_world_checker_passes_for_a_healthy_world() -> None:
    check = world_checker(_StubRegistry([1, 2]), _StubStore(), _StubFormations([1, 9]))
    assert check(hold(CommandLedger(), 1, issuer=1)) is None
    assert check(hold(CommandLedger(), 1, issuer=NOBODY)) is None
    assert check(hold(CommandLedger(), 9, assignee_level=LEVEL_UNIT, issuer=1)) is None


def test_world_checker_skips_unit_liveness() -> None:
    """编队没有生死——它的存续由中观控制器判断（宿主被摧毁）。
    台账拿实体存活去查编队 ID，只会查到"查无此实体"。"""
    check = world_checker(
        _StubRegistry([1]), _StubStore(dead=(1,)), _StubFormations([1, 9])
    )
    assert check(hold(CommandLedger(), 9, assignee_level=LEVEL_UNIT, issuer=1)) is None


# ---------------------------------------------------------------------------
# 巡检：失效检测
# ---------------------------------------------------------------------------

def test_sweep_expires_task_past_deadline() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7, effective_at=0, deadline_at=H)
    ledger.issue([task], at=0)

    report = ledger.sweep(H)
    assert task.state == STATE_EXPIRED
    assert report.tasks == (task,)
    assert "超过截止时刻" in report.describe()


def test_sweep_is_idempotent() -> None:
    """第二次巡检不该重复报同一件事——否则上级会收到同一件事三遍。"""
    ledger = CommandLedger()
    ledger.issue([hold(ledger, 7, effective_at=0, deadline_at=H)], at=0)
    assert len(ledger.sweep(H).tasks) == 1
    assert ledger.sweep(2 * H).is_empty


def test_sweep_aborts_task_whose_executor_is_gone() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7)
    ledger.issue([task], at=0)

    report = ledger.sweep(MIN, is_alive=lambda eid: False)
    assert report.tasks == (task,)
    assert task.state == STATE_ABORTED
    assert "执行者实体 #7 已不存在" in task.history[-1].reason


def test_sweep_fails_task_whose_target_is_gone() -> None:
    ledger = CommandLedger()
    task = ledger.draft(
        kind=KIND_ENGAGE, assignee=1, objective=Objective(target=2), at=0
    )
    ledger.issue([task], at=0)

    ledger.sweep(MIN, is_alive=lambda eid: eid != 2)
    assert task.state == STATE_FAILED
    assert "目标实体 #2 已不存在" in task.history[-1].reason


def test_world_cause_wins_over_deadline() -> None:
    """"目标在 T+50 被打掉"比"期限在 T+600 到了"有用得多——
    后者只说明巡检间隔太长。"""
    ledger = CommandLedger()
    task = ledger.draft(
        kind=KIND_ENGAGE, assignee=1, objective=Objective(target=2), at=0,
        deadline_at=H,
    )
    ledger.issue([task], at=0)

    ledger.sweep(2 * H, is_alive=lambda eid: eid != 2)
    assert task.state == STATE_FAILED


def test_sweep_without_liveness_only_checks_deadlines() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7)
    ledger.issue([task], at=0)
    assert ledger.sweep(MIN).is_empty
    assert task.state == STATE_PENDING


def test_sweep_does_not_announce_start() -> None:
    """到点没开工是执行方的事。台账替它宣布开工，兵力统计就一直是错的。"""
    ledger = CommandLedger()
    task = hold(ledger, 7, effective_at=MIN)
    ledger.issue([task], at=0)
    ledger.sweep(H)
    assert task.state == STATE_PENDING


def test_sweep_expires_intent() -> None:
    ledger = CommandLedger()
    intent = ledger.issue_intent(
        ledger.draft_intent(statement="固守待援", issuer=1, at=0, expires_at=H)
    )
    report = ledger.sweep(H)
    assert intent.state == INTENT_EXPIRED
    assert report.intents == (intent,)


def test_sweep_emits_signal_with_the_report() -> None:
    ledger = CommandLedger()
    ledger.issue([hold(ledger, 7, effective_at=0, deadline_at=H)], at=0)
    seen = []
    ledger.on_invalidated.subscribe(seen.append)
    ledger.sweep(H)
    assert len(seen) == 1 and len(seen[0].tasks) == 1


def test_sweep_skips_terminal_tasks() -> None:
    ledger = CommandLedger()
    task = hold(ledger, 7, effective_at=0, deadline_at=H)
    ledger.issue([task], at=0)
    task.activate(at=MIN, by=7)
    task.complete(at=2 * MIN, by=7)
    assert ledger.sweep(2 * H).is_empty
    assert task.state == STATE_COMPLETED


# ---------------------------------------------------------------------------
# 意图
# ---------------------------------------------------------------------------

def test_intent_requires_a_statement() -> None:
    ledger = CommandLedger()
    intent = ledger.draft_intent(statement="   ", issuer=1, at=0)
    problems = intent.validate()
    assert any("意图原文不能为空" in p for p in problems)
    with pytest.raises(TaskError, match="没有下达"):
        ledger.issue_intent(intent)


def test_intent_priority_is_a_closed_set() -> None:
    """无上界的整数会立刻引出"5 和 7 差在哪"，然后各人给出不同答案。"""
    ledger = CommandLedger()
    intent = ledger.draft_intent(statement="推进", issuer=1, at=0, priority=7)
    assert any("未知的优先级" in p for p in intent.validate())


def test_intent_effective_at_defaults_to_issue_time() -> None:
    ledger = CommandLedger()
    intent = ledger.draft_intent(statement="推进", issuer=1, at=5 * MIN)
    assert intent.effective_at == 5 * MIN


def test_intent_running_requires_both_state_and_window() -> None:
    ledger = CommandLedger()
    intent = ledger.issue_intent(
        ledger.draft_intent(statement="推进", issuer=1, at=0, expires_at=H)
    )
    assert intent.is_running_at(MIN)
    assert not intent.is_running_at(2 * H)

    intent.cancel(at=MIN, by=1, reason="情况变化")
    assert intent.state == INTENT_CANCELLED
    # 撤销了即使还在有效期内也不再生效——只看时间会漏掉这一条
    assert not intent.is_running_at(2 * MIN)


def test_supersede_marks_the_old_intent() -> None:
    ledger = CommandLedger()
    first = ledger.issue_intent(ledger.draft_intent(statement="夺取高地", issuer=1, at=0))
    second = ledger.issue_intent(
        ledger.draft_intent(statement="转入防御", issuer=1, at=H, supersedes=first.intent_id)
    )

    assert first.state == INTENT_SUPERSEDED
    assert second.state == INTENT_ACTIVE
    assert f"被意图 #{second.intent_id} 取代" in first.history[-1].reason
    assert ledger.running_intents(H) == (second,)


def test_superseding_a_missing_intent_is_rejected() -> None:
    ledger = CommandLedger()
    intent = ledger.draft_intent(statement="推进", issuer=1, at=0, supersedes=99)
    with pytest.raises(TaskError, match="台账里没有这条"):
        ledger.issue_intent(intent)


def test_superseding_a_terminal_intent_is_rejected() -> None:
    """取代一条已经失效的意图，通常意味着上游状态不同步。"""
    ledger = CommandLedger()
    first = ledger.issue_intent(ledger.draft_intent(statement="夺取", issuer=1, at=0))
    first.cancel(at=MIN, by=1, reason="取消")
    second = ledger.draft_intent(statement="转入防御", issuer=1, at=H, supersedes=first.intent_id)
    with pytest.raises(TaskError, match="已经是"):
        ledger.issue_intent(second)


def test_running_intents_ordered_by_priority_then_id() -> None:
    """并列时的次序是显式的：同优先级按编号。依赖字典遍历顺序的话，
    换个 PYTHONHASHSEED 结果就变了。"""
    ledger = CommandLedger()
    routine = ledger.issue_intent(
        ledger.draft_intent(statement="例行巡逻", issuer=1, at=0, priority=PRIORITY_ROUTINE)
    )
    urgent = ledger.issue_intent(
        ledger.draft_intent(statement="紧急增援", issuer=1, at=0, priority=PRIORITY_URGENT)
    )
    important = ledger.issue_intent(
        ledger.draft_intent(statement="重要目标", issuer=1, at=0, priority=PRIORITY_IMPORTANT)
    )
    assert ledger.running_intents(0) == (urgent, important, routine)
    assert ledger.primary_intent(0) is urgent


def test_intent_window_and_describe() -> None:
    ledger = CommandLedger()
    intent = ledger.issue_intent(
        ledger.draft_intent(
            statement="夺取 116 高地", issuer=1, at=0, area="HILL_116",
            priority=PRIORITY_URGENT, expires_at=H, constraints=("不得越境",),
        )
    )
    assert "夺取 116 高地" in intent.describe()
    assert "紧急" in intent.describe() and "不得越境" in intent.describe()
    assert intent.to_dict()["constraints"] == ["不得越境"]


# ---------------------------------------------------------------------------
# 杂项
# ---------------------------------------------------------------------------

def test_task_error_is_a_configuration_error() -> None:
    """落在 MilsimError 下面：对调用方就是"打一句人话给用户看"。"""
    assert issubclass(TaskError, MilsimError)
    assert issubclass(TaskError, ValueError)


def test_statistics_counts_by_state() -> None:
    ledger = CommandLedger()
    done = hold(ledger, 1)
    ledger.issue([done, hold(ledger, 2), hold(ledger, 3)], at=0)
    done.activate(at=0, by=1)
    done.complete(at=MIN, by=1)

    stats = ledger.statistics()
    assert stats["tasks"] == 3
    assert stats["open"] == 2
    assert stats["by_state"][STATE_COMPLETED] == 1
    assert stats["by_state"][STATE_PENDING] == 2
    assert "已完成 1" in ledger.describe()


def test_task_to_dict_is_json_friendly() -> None:
    import json

    ledger = CommandLedger()
    task = hold(ledger, 7, issuer=3, at=0, deadline_at=H)
    task.activate(at=MIN, by=7)
    payload = task.to_dict()
    json.dumps(payload)  # 不抛就算过
    assert payload["state"] == STATE_ACTIVE
    assert payload["history"][-1]["by"] == 7


def test_task_describe_can_resolve_names() -> None:
    """层级要一起传进去：编队 ID 与实体 ID 是两个从 1 开始的整数空间，
    只给整数的话解析器无法判断该查编队表还是注册表。"""
    ledger = CommandLedger()
    task = hold(ledger, 7, issuer=3, at=0)
    assert "#7" in task.describe()

    seen: list[tuple[int, str]] = []

    def resolve(ident: int, level: str) -> str:
        seen.append((ident, level))
        return "三营" if ident == 3 else f"#{ident}"

    text = task.describe(names_of=resolve)
    assert "三营" in text
    assert (7, LEVEL_ENTITY) in seen
    assert (3, LEVEL_UNIT) in seen
    # 名字查不到不该让打印炸掉
    assert "#7" in task.describe(names_of=lambda i, lvl: (_ for _ in ()).throw(KeyError(i)))


def test_batch_span_and_breakdown() -> None:
    ledger = CommandLedger()
    entity_task = hold(ledger, 1, effective_at=0, deadline_at=H)
    unit_task = hold(ledger, 2, assignee_level=LEVEL_UNIT, effective_at=0, deadline_at=2 * H)
    batch = ledger.issue([entity_task, unit_task], at=0, effective_at=MIN)

    assert batch.span() == (MIN, H)
    assert "实体 1 条" in batch.describe() and "编队 1 条" in batch.describe()
    assert batch.by_level(LEVEL_UNIT) == (unit_task,)
    assert batch.to_dict()["size"] == 2
    assert len(batch) == 2
