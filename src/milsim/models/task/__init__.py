"""任务层组件族（§5.19，v0.13.59）——AFSIM 任务管理器的对应物。

只导出与 :mod:`milsim.models` 约定一致的族清单与类名。
"""

from __future__ import annotations

from .base import (
    COMM_NOTE,
    TASK_ACK,
    TASK_ASSIGN,
    TASK_CANCEL,
    TASK_STATUS,
    FRAMEWORK_TASKS,
    Task,
    TaskManager,
)

__all__ = [
    "COMM_NOTE",
    "FRAMEWORK_TASKS",
    "TASK_ACK",
    "TASK_ASSIGN",
    "TASK_CANCEL",
    "TASK_STATUS",
    "Task",
    "TaskManager",
]
