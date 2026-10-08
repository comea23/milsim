"""侧挂控制器与编队私有状态的测试（M5a，§4.5.3① / §3.9.5）。

两条边界是重点：

- 控制器与编队状态**不是实体**：不进注册表、不进存储、不进空间索引。
  破了这条，敌方雷达就会"探测到一个营"。
- 编队状态里存的必须是**算不出来的**东西：任务来自上级、报告是事件、
  本级判断是结论、累计战损是总量。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.models import (
    LEVEL_ENTITY,
    LEVEL_STRATEGIC,
    LEVEL_UNIT,
    NO_HOST,
    Controller,
    UnitState,
)
from milsim.services import (
    KIND_HOLD,
    LEVEL_UNIT as TASK_LEVEL_UNIT,
    CommandLedger,
    TaskError,
)
from milsim.services.registry import EntityRegistry
from milsim.services.store import EntityStore

H = 3_600_000_000
MIN = 60_000_000


def make_controller(**kwargs) -> Controller:
    kwargs.setdefault("level", LEVEL_UNIT)
    kwargs.setdefault("host_id", 77)
    kwargs.setdefault("name", "三营指挥所")
    kwargs.setdefault("unit_state", UnitState(unit_id=3))
    return Controller(**kwargs)


def make_task(assignee: int = 3, level: str = TASK_LEVEL_UNIT, **kwargs):
    return CommandLedger().draft(kind=KIND_HOLD, assignee=assignee,
                                 assignee_level=level, **kwargs)


# ---------------------------------------------------------------------------
# 不是实体
# ---------------------------------------------------------------------------

def test_controller_never_enters_the_entity_spaces() -> None:
    """破了这条，敌方雷达就会"探测到一个营"，而营没有坐标。"""
    registry = EntityRegistry()
    store = EntityStore()

    controller = make_controller()
    controller.unit_state.assign_task(make_task())

    assert len(registry) == 0
    assert controller.host_id not in registry
    # 编队 ID 也没有落进存储：拿它去查存活只会得到"查无此实体"
    assert store.is_alive(controller.unit_state.unit_id) is False


# ---------------------------------------------------------------------------
# 宿主与生命周期
# ---------------------------------------------------------------------------

def test_host_is_alive_by_default() -> None:
    controller = make_controller()
    assert controller.is_active
    assert not controller.host_lost


def test_host_loss_deactivates_the_controller() -> None:
    """宿主的存活检查就是"斩首行动"的全部实现——没有特例逻辑。"""
    controller = make_controller()
    assert controller.check_host(is_alive=lambda eid: True, at=MIN)
    assert controller.is_active

    assert controller.check_host(is_alive=lambda eid: False, at=2 * MIN) is False
    assert not controller.is_active
    assert controller.host_lost
    assert "宿主实体 #77 已不存在" in controller.lost_reason


def test_deactivate_is_idempotent_and_reports_only_the_first_time() -> None:
    """重复失效返回 False：调用方常要据此决定"要不要发通报"，
    否则每次巡检都会再发一遍同一件事。"""
    controller = make_controller()
    assert controller.deactivate(at=MIN, reason="撤编") is True
    assert controller.deactivate(at=2 * MIN, reason="撤编") is False
    assert controller.lost_reason == "撤编"


def test_deactivate_calls_the_hook() -> None:
    calls = []

    class Watcher(Controller):
        level = LEVEL_UNIT

        def on_deactivated(self, *, at: int, reason: str) -> None:
            calls.append((at, reason))

    controller = Watcher(host_id=77, unit_state=UnitState(unit_id=3))
    controller.deactivate(at=MIN, reason="宿主被摧毁")
    assert calls == [(MIN, "宿主被摧毁")]


def test_controller_without_host_never_loses_it() -> None:
    """战略级代表"一方"，没有物理载体可供人打掉。"""
    controller = Controller(level=LEVEL_STRATEGIC, host_id=NO_HOST, name="红方")
    assert controller.check_host(is_alive=lambda eid: False, at=MIN)
    assert controller.is_active


def test_inactive_controller_stays_inactive() -> None:
    controller = make_controller()
    controller.deactivate(at=MIN, reason="撤编")
    # 宿主"活着"也不会让它复活——失效是单向的
    assert controller.check_host(is_alive=lambda eid: True, at=2 * MIN) is False


def test_subclass_declares_its_level() -> None:
    class BattalionHQ(Controller):
        level = LEVEL_UNIT

    controller = BattalionHQ(host_id=77, unit_state=UnitState(unit_id=3))
    assert controller.level == LEVEL_UNIT
    assert "编队级" in controller.describe()


# ---------------------------------------------------------------------------
# 层级与状态的搭配
# ---------------------------------------------------------------------------

def test_unit_level_requires_unit_state() -> None:
    with pytest.raises(ConfigurationError, match="必须带一份编队私有状态"):
        Controller(level=LEVEL_UNIT, host_id=77)


def test_other_levels_reject_unit_state() -> None:
    with pytest.raises(ConfigurationError, match="不该带编队私有状态"):
        Controller(level=LEVEL_STRATEGIC, unit_state=UnitState(unit_id=3))


def test_unknown_level_is_rejected() -> None:
    with pytest.raises(ConfigurationError, match="未知的控制器层级"):
        Controller(level="battalion")


# ---------------------------------------------------------------------------
# 两级槽位
# ---------------------------------------------------------------------------

def test_on_intent_rejects_terminal_intent() -> None:
    ledger = CommandLedger()
    intent = ledger.issue_intent(ledger.draft_intent(statement="推进", issuer=1, at=0))
    intent.cancel(at=MIN, by=1, reason="情况变化")

    controller = make_controller()
    with pytest.raises(ConfigurationError, match="已经是终态"):
        controller.on_intent(intent)


def test_on_intent_replaces_the_current_one() -> None:
    ledger = CommandLedger()
    first = ledger.issue_intent(ledger.draft_intent(statement="夺取高地", issuer=1, at=0))
    controller = make_controller()
    controller.on_intent(first)
    assert controller.current_intent is first


def test_tasks_are_kept_in_issue_order_not_arrival_order() -> None:
    """同一批任务的到达顺序取决于通信，而"先看哪条"应该由编号决定。"""
    ledger = CommandLedger()
    controller = make_controller()
    tasks = [make_task(assignee=3) for _ in range(3)]
    tasks[0].task_id, tasks[1].task_id, tasks[2].task_id = 30, 10, 20

    for task in (tasks[0], tasks[2], tasks[1]):  # 乱序到达
        controller.on_task(task)

    assert [t.task_id for t in controller.tasks] == [10, 20, 30]
    assert len(ledger) == 0  # 台账在这条测试里没被用上，明示一下


def test_open_tasks_and_idle() -> None:
    controller = make_controller()
    assert controller.is_idle

    task = make_task(assignee=3)
    controller.on_task(task)
    assert not controller.is_idle

    task.activate(at=MIN, by=3)
    task.complete(at=2 * MIN, by=3)
    assert controller.is_idle
    assert len(controller.tasks) == 1
    assert controller.forget(task.task_id)
    assert not controller.forget(task.task_id)


def test_controller_to_dict() -> None:
    controller = make_controller()
    controller.on_task(make_task(assignee=3))
    payload = controller.to_dict()
    assert payload["level"] == LEVEL_UNIT
    assert payload["host_id"] == 77
    assert payload["active"] is True
    assert payload["task_ids"] == [1]


# ---------------------------------------------------------------------------
# 编队私有状态
# ---------------------------------------------------------------------------

def test_assign_task_rejects_entity_level_task() -> None:
    """编队状态里挂一条实体级任务，会让"三营在干什么"显示出某个连的动作，
    而两条数据单独看都合规。"""
    state = UnitState(unit_id=3)
    with pytest.raises(TaskError, match="只能接收编队级任务"):
        state.assign_task(make_task(assignee=3, level=LEVEL_ENTITY))


def test_assign_task_rejects_a_foreign_unit() -> None:
    state = UnitState(unit_id=3)
    with pytest.raises(TaskError, match="不是本编队"):
        state.assign_task(make_task(assignee=9, level=TASK_LEVEL_UNIT))


def test_assign_task_rejects_terminal_task() -> None:
    state = UnitState(unit_id=3)
    task = make_task(assignee=3, level=TASK_LEVEL_UNIT)
    task.abort(at=MIN, reason="取消")
    with pytest.raises(TaskError, match="已经是终态"):
        state.assign_task(task)


def test_clear_task_only_clears_the_matching_one() -> None:
    """迟到的完成回调不能清掉后来的任务。

    任务 #1 的完成上报在 #2 已经接手之后才到，若按"清空"处理，三营就会在
    态势里显示"无所事事"，而它正在执行 #2。
    """
    state = UnitState(unit_id=3)
    first = make_task(assignee=3, level=TASK_LEVEL_UNIT)
    second = make_task(assignee=3, level=TASK_LEVEL_UNIT)
    state.assign_task(first)
    state.assign_task(second)

    assert state.clear_task(first) is False
    assert state.current_task is second
    assert state.clear_task(second) is True
    assert state.current_task is None
    assert state.clear_task() is False


def test_record_loss_accumulates_and_survives_reinforcement() -> None:
    """"完好度 60%"与"已损失 40%"不是一回事：补进一个满编连之后完好度回到
    90%，累计战损只增不减。复盘问"这个营被打掉多少"，只有后者答得上。"""
    state = UnitState(unit_id=3)
    assert state.record_loss(0.4) == pytest.approx(0.4)
    # 补充兵力（完好度回升）不影响累计战损
    assert state.record_loss(0.7) == pytest.approx(1.1)
    assert state.cumulative_loss > 1.0


def test_negative_loss_is_rejected() -> None:
    with pytest.raises(ConfigurationError, match="战损是跌幅"):
        UnitState(unit_id=3).record_loss(-0.2)


def test_report_records_time_and_assessment() -> None:
    state = UnitState(unit_id=3)
    assert "尚未报告" in state.describe()

    state.report(at=H, assessment="当面之敌疑为佯动")
    assert state.last_report_at == H
    assert "佯动" in state.describe()
    assert state.to_dict()["assessment"] == "当面之敌疑为佯动"


def test_unit_state_to_dict_uses_task_id_not_object() -> None:
    state = UnitState(unit_id=3)
    task = make_task(assignee=3, level=TASK_LEVEL_UNIT)
    state.assign_task(task)
    payload = state.to_dict()
    assert payload["current_task"] == task.task_id
    assert payload["unit_id"] == 3
