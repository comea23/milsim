"""编队级私有状态：**算不出来的那些东西**（§3.9.5）。

可推导 vs 需记忆
----------------
投影层算出来的 ``FormationInfo``（战力、规模、质心）**全部是可推导的**：
"三营战力 87%" 随时能从三个连的完好度现算，所以一个字节都不用存。

但有些东西下层推不出来，必须记：

| 要记的 | 为什么下层推不出来 |
|---|---|
| 当前在执行什么任务 | 任务来自**上级**，不在下层数据里 |
| 上次向旅部报告是几点 | 这是个**事件**，不是状态 |
| 本级判断 | "我认为当面之敌是佯动"——本级的情报**结论**，可能是错的 |
| 累计战损 | 从开战到现在的**总量**，不等于"当前完好度" |

最后一条最容易漏。"完好度 60%"与"已损失 40%"看着是一回事，其实不是：
补进一个满编连之后完好度可能回到 90%，而累计战损只增不减。复盘时问
"这个营到底被打掉多少"，只有后者答得上。

为什么不进注册表和空间索引
--------------------------
本类**不是实体**，没有 ``entity_id``，不进 :class:`EntityRegistry`、
不进 :class:`EntityStore`、不进空间索引。

否则会出现一个荒谬但自然的结果：**敌方雷达"探测到一个营"**。编队没有位置，
"探测到编队"要么编不出坐标，要么得拿成员质心冒充——那就是聚合泄漏。
编队状态只能侧挂，不能落进实体空间。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..engine.time import SimTime
from ..errors import ConfigurationError
from ..services.orchestrator import LEVEL_UNIT, Task, TaskError


@dataclass(slots=True)
class UnitState:
    """一个编队的私有状态。挂在编队级控制器上（``controller.py``）。

    四个字段一个都不能少，但也不该多——**凡是能从下层推出来的都不放这里**。
    放进去的代价是：它和下层数据之间多了一份可能不一致的副本。
    """

    unit_id: int
    #: 上次向上级报告的时刻。这是**事件**的痕迹，不是状态。
    last_report_at: SimTime = 0
    #: 本级判断。自然语言，可以是错的——"本级的情报结论"与"客观事实"
    #: 是两件事，混起来就丢掉了"营长判断错了"这个可以建模的东西。
    assessment: str = ""
    #: 累计战损：历次完好度跌幅的**累加**，可超过 1.0。
    #:
    #: 用累加而不是"当前缺失比例"，是为了让"被打残 → 补充 → 再被打残"
    #: 留下痕迹。0~1 的当前完好度会把这两次打成一次。
    cumulative_loss: float = 0.0
    #: 当前执行的任务。**只由 :meth:`assign_task` 写入**，见那里的说明。
    _current_task: Task | None = field(default=None, init=False, repr=False)

    # -- 当前任务 ----------------------------------------------------------

    @property
    def current_task(self) -> Task | None:
        return self._current_task

    @property
    def task_id(self) -> int | None:
        return None if self._current_task is None else self._current_task.task_id

    def assign_task(self, task: Task) -> None:
        """接受一条任务。

        三处校验都是"错了也不报错、只会让下游拿到错东西"的类型：

        - 层级与编队必须对上。编队级状态里挂一条实体级任务，会让"三营在
          干什么"显示出某个连的动作，而两条数据单独看都合规。
        - 终态任务不能挂上来。它已经结束了，再挂成"当前任务"等于让编队
          永远在做一个已经完成的事。
        """
        if task.assignee_level != LEVEL_UNIT:
            raise TaskError(
                f"编队 #{self.unit_id} 的状态只能接收编队级任务，"
                f"而任务 #{task.task_id} 是{task.assignee_level} 级"
            )
        if task.assignee != self.unit_id:
            raise TaskError(
                f"任务 #{task.task_id} 的执行者是编队 #{task.assignee}，"
                f"不是本编队 #{self.unit_id}"
            )
        if task.is_terminal:
            raise TaskError(
                f"任务 #{task.task_id} 已经是终态，不能挂成当前任务"
            )
        self._current_task = task

    def clear_task(self, task: Task | None = None) -> bool:
        """清掉当前任务，返回是否真的清了。

        必须传**那条任务本身**：迟到的完成回调不能清掉后来的任务。若只按
        "清空"处理，任务 #7 的完成上报在 #9 已经接手之后才到，就会把 #9
        一起清掉——然后三营在态势里显示"无所事事"，而它正在执行 #9。
        """
        if self._current_task is None:
            return False
        if task is not None and task is not self._current_task:
            return False
        self._current_task = None
        return True

    # -- 报告与战损 --------------------------------------------------------

    def report(self, *, at: SimTime, assessment: str | None = None) -> None:
        """记一次向上级报告。``assessment`` 给了就一并更新本级判断。"""
        self.last_report_at = at
        if assessment is not None:
            self.assessment = assessment

    def record_loss(self, amount: float) -> float:
        """累加一次战损，返回累计值。

        ``amount`` 是**完好度的跌幅**，不是"当前还剩多少"。两者容易混，
        而混了之后"损失"会随补充而减少——那就不叫损失了。
        """
        if amount < 0.0:
            raise ConfigurationError(
                f"战损是跌幅，不能为负（实际 {amount}）——"
                "补充兵力是另一件事，不该用负的战损表达"
            )
        self.cumulative_loss += float(amount)
        return self.cumulative_loss

    # -- 输出 --------------------------------------------------------------

    def describe(self) -> str:
        task = (
            f"任务 #{self.task_id}"
            if self._current_task is not None
            else "无任务"
        )
        reported = (
            f"上次报告 T+{self.last_report_at / 1_000_000:.1f}s"
            if self.last_report_at
            else "尚未报告"
        )
        parts = [f"编队 #{self.unit_id}：{task}", reported]
        if self.cumulative_loss:
            parts.append(f"累计战损 {self.cumulative_loss:.0%}")
        if self.assessment:
            parts.append(f"本级判断「{self.assessment}」")
        return "，".join(parts)

    def to_dict(self) -> dict[str, Any]:
        return {
            "unit_id": self.unit_id,
            "current_task": self.task_id,
            "last_report_at": self.last_report_at,
            "assessment": self.assessment,
            "cumulative_loss": self.cumulative_loss,
        }

    def __repr__(self) -> str:
        return (
            f"UnitState(unit={self.unit_id} task={self.task_id} "
            f"loss={self.cumulative_loss:.2f})"
        )


__all__ = ["UnitState"]
