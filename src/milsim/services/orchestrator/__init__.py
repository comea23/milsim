"""分级控制骨架：意图 → 任务 → 动作 三级链条的中间两级。

| 模块 | 内容 |
|---|---|
| ``common`` | 状态变更记录 + 状态转移白名单校验 + ``TaskError`` |
| ``task`` | ``Task`` 状态机、``Objective`` 目标要素、类型白名单 |
| ``intent`` | ``Intent`` 战役级意图（结构化外壳 + 自然语言内核） |
| ``batch`` | ``TaskBatch`` 原子性载体 |
| ``ledger`` | ``CommandLedger`` 命令台账：编号 / 校验 / 原子登记 / 失效检测 |

三级的产出与归属（§4.5.3②）：

```text
战略级（一方一个）    Intent    "夺取 X 区域制空权"        —— 可多解
      ↓ 细化
战役级（一编队一个）  Task[]    "一连 T+300s 前占领 1234"  —— 目标已定死
      ↓ 细化
实体级（一平台一个）  Action    "以 15 m/s 向 045° 机动"    —— 落到动作层
```

本包只管前两级。``Action`` 是实体级控制器的产出，属 M4a/M5b；
而**谁在什么条件下把意图拆成任务**（细化器）属 M5b 的接线。

一处与文档原文不同的判断
------------------------
§4.5.6 说"校验通过后一次性**投递**"，实现时改为"一次性**登记**"——
因为任务不是消息。理由写在 ``ledger.py`` 模块头，那里也说明了通信层
该管什么、不该管什么。
"""

from .batch import TaskBatch
from .common import NOBODY, StateChange, TaskError
from .intent import (
    INTENT_ACTIVE,
    INTENT_CANCELLED,
    INTENT_EXPIRED,
    INTENT_LABELS,
    INTENT_STATES,
    INTENT_SUPERSEDED,
    INTENT_TRANSITIONS,
    PRIORITIES,
    PRIORITY_IMPORTANT,
    PRIORITY_LABELS,
    PRIORITY_ROUTINE,
    PRIORITY_URGENT,
    TERMINAL_INTENT_STATES,
    Intent,
)
from .ledger import AliveFn, CheckFn, CommandLedger, SweepReport, world_checker
from .task import (
    ASSIGNEE_LEVELS,
    ASSIGNEE_LABELS,
    KIND_DEFEND,
    KIND_ENGAGE,
    KIND_HOLD,
    KIND_LABELS,
    KIND_MOVE,
    KIND_OCCUPY,
    KIND_RECON,
    KIND_REPORT,
    KIND_REQUIREMENTS,
    LEVEL_ENTITY,
    LEVEL_UNIT,
    REQUIREMENT_LABELS,
    STATE_ABORTED,
    STATE_ACTIVE,
    STATE_COMPLETED,
    STATE_EXPIRED,
    STATE_FAILED,
    STATE_LABELS,
    STATE_PENDING,
    TASK_KINDS,
    TASK_STATES,
    TASK_TRANSITIONS,
    TERMINAL_STATES,
    Objective,
    Task,
    missing_requirements,
)

__all__ = [
    # -- 公共 --
    "NOBODY",
    "StateChange",
    "TaskError",
    # -- 意图 --
    "Intent",
    "INTENT_ACTIVE",
    "INTENT_CANCELLED",
    "INTENT_EXPIRED",
    "INTENT_LABELS",
    "INTENT_STATES",
    "INTENT_SUPERSEDED",
    "INTENT_TRANSITIONS",
    "TERMINAL_INTENT_STATES",
    "PRIORITIES",
    "PRIORITY_IMPORTANT",
    "PRIORITY_LABELS",
    "PRIORITY_ROUTINE",
    "PRIORITY_URGENT",
    # -- 任务 --
    "ASSIGNEE_LEVELS",
    "ASSIGNEE_LABELS",
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
    "STATE_ABORTED",
    "STATE_ACTIVE",
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
    # -- 批次与台账 --
    "AliveFn",
    "CheckFn",
    "CommandLedger",
    "SweepReport",
    "TaskBatch",
    "world_checker",
]
