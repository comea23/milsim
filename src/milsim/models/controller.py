"""侧挂控制器：宏观 / 中观智能体的宿主与生命周期（§4.5.3①）。

控制器不是 Entity
-----------------
宏观智能体是"一方"的控制器，中观是"一个编队"的控制器。它们没有位置、
没有传感器、不参与交战。硬塞进 :class:`~milsim.models.entity.Entity` 有三个
具体后果：

- **ID 分配**：它不需要 ``entity_id``，却会占掉一个号
- **格归属**：它在哪个网格单元里？没有答案，只能填个假值
- **存活状态**：谁把它"打掉"？

最后一条最糟：**雷达会"探测到一个旅部"**，而旅部没有坐标。要么编不出坐标，
要么拿下辖部队质心冒充——那是聚合泄漏换了个地方出现。

所以控制器**侧挂**在 ``Simulation`` 上，不进注册表、不进存储、不进空间索引。

但它必须有宿主
--------------
控制器通常挂在某个指挥部实体上。宿主被摧毁 → 控制器失效。
这样"斩首行动"是自然产物，而不是一条特例逻辑（"如果旅部被打掉就……"）。

宿主可以是"没有"（:data:`NO_HOST`）：战略级控制器代表**一方**，而"一方"
没有具体的物理载体，不会有谁来打掉它。这时它不因斩首而失效——这是对的，
不是遗漏。

三级控制器
----------
| 层级 | 一个实例代表 | 宿主 | 产出 |
|---|---|---|---|
| :data:`LEVEL_STRATEGIC` | 一方 | 该方指挥部（可无） | ``Intent`` |
| :data:`LEVEL_UNIT` | 一个编队 | 编队内指定的指挥部实体 | ``Task[]`` |
| :data:`LEVEL_ENTITY` | 一个平台 | 自己 | ``Action`` |

**M5a 只固定生命周期**：宿主检查、Intent / Task 两级槽位、失效回调。
"谁在什么节拍上想一次、想完把产出交给谁"是接线，属 M5b——提前定下
决策方法的签名，等于在没想清楚之前先签了一份要版本化的契约（§5.4）。
"""

from __future__ import annotations

from typing import Any, Callable

from ..engine.time import SimTime
from ..errors import ConfigurationError
from ..services.orchestrator import Intent, Task

#: 没有宿主：战略级控制器、导演部代理。这类控制器不会因斩首而失效。
#:
#: 取 -1 而不是 0：实体与编队的 ID 空间都从 0 开始，用 0 当哨兵值会把
#: "没有宿主"和"0 号实体"变成同一件事。理由同 ``common.NOBODY``。
NO_HOST = -1

LEVEL_STRATEGIC = "strategic"
LEVEL_UNIT = "unit"
LEVEL_ENTITY = "entity"

CONTROLLER_LEVELS = frozenset({LEVEL_STRATEGIC, LEVEL_UNIT, LEVEL_ENTITY})

CONTROLLER_LABELS = {
    LEVEL_STRATEGIC: "战略级",
    LEVEL_UNIT: "编队级",
    LEVEL_ENTITY: "实体级",
}

#: 宿主存活判据：给一个实体 ID，回答它还在不在。
HostCheckFn = Callable[[int], bool]


class Controller:
    """侧挂控制器基类。

    使用者继承它实现三级控制器。**本类不持有引擎、不持有世界**——需要时
    由调用方在方法参数里给（同 ``CommandLedger`` 的取引用纪律：装配期
    ``formations`` 会被整只换掉，构造时抓住的引用会静默指向旧对象）。
    """

    #: 子类覆盖。空串表示不声明层级（仅做测试桩时如此）。
    #:
    #: 刻意**不用** ``__slots__``：子类要覆盖这个类属性来声明自己的层级，
    #: 而"父类 slot 描述符 + 子类同名类属性"的组合在赋值时会走出一条谁都不
    #: 愿读第二遍的 MRO 规则。控制器的数量是"每编队一个"而不是"每实体一个"，
    #: 省这点内存不值得换那个坑。
    level: str = ""

    def __init__(
        self,
        *,
        level: str = "",
        host_id: int = NO_HOST,
        name: str = "",
        unit_state: Any = None,
    ) -> None:
        resolved = level or self.level
        if resolved and resolved not in CONTROLLER_LEVELS:
            raise ConfigurationError(
                f"未知的控制器层级 {resolved!r}，可用："
                + " / ".join(sorted(CONTROLLER_LEVELS))
            )
        if resolved == LEVEL_UNIT and unit_state is None:
            raise ConfigurationError(
                f"编队级控制器 {name or '<未命名>'} 必须带一份编队私有状态"
                "（UnitState）——省掉它的话，\"这个编队在执行什么任务\""
                "就没有归属地了"
            )
        if resolved != LEVEL_UNIT and unit_state is not None:
            raise ConfigurationError(
                f"{CONTROLLER_LABELS.get(resolved, resolved)}控制器不该带编队"
                "私有状态：那是编队级的东西"
            )

        self.level: str = resolved
        self.host_id: int = int(host_id)
        self.name: str = name or f"{CONTROLLER_LABELS.get(resolved, '控制器')}"
        #: 编队级控制器才有。类型是 ``UnitState``，这里用 Any 是为了让
        #: controller 模块不依赖 unit_state 模块（两者互不引用，谁先导入都行）。
        self.unit_state = unit_state
        self._intent: Intent | None = None
        self._tasks: dict[int, Task] = {}
        self._active: bool = True
        self._lost_reason: str = ""
        self._lost_at: SimTime = 0

    # -- 生命周期 ----------------------------------------------------------

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def host_lost(self) -> bool:
        return bool(self._lost_reason)

    @property
    def lost_reason(self) -> str:
        return self._lost_reason

    def deactivate(self, *, at: SimTime = 0, reason: str = "") -> bool:
        """失效。返回是否**本次**才失效（重复失效返回 ``False``）。

        返回布尔而不是无返回值：调用方常要据此决定"要不要发一条通报"，
        而重复发通报会让上级收到同一件事三遍——每次巡检一遍。
        """
        if not self._active:
            return False
        self._active = False
        self._lost_reason = reason or "失效"
        self._lost_at = at
        self.on_deactivated(at=at, reason=self._lost_reason)
        return True

    def check_host(self, *, is_alive: HostCheckFn, at: SimTime = 0) -> bool:
        """查宿主是否还在。不在了就让控制器失效。

        宿主是"自己"时（实体级控制器）不查：实体级控制器随实体存亡，
        把自己查一遍只会绕成一个圈。
        """
        if self.host_id == NO_HOST or self.host_id == self._self_host():
            return self._active

        if not self._active:
            return False

        if not is_alive(self.host_id):
            self.deactivate(
                at=at,
                reason=f"宿主实体 #{self.host_id} 已不存在（{self.name} 失效）",
            )
            return False
        return True

    def _self_host(self) -> int:
        """实体级控制器以自己为宿主时的 ID。基类不知道自己的实体 ID，
        子类覆盖（M5b）。返回 :data:`NO_HOST` 表示"没有这回事"。"""
        return NO_HOST

    # -- 两级槽位 ----------------------------------------------------------

    def on_intent(self, intent: Intent) -> None:
        """收到一条意图。战略级控制器用它替换当前意图。

        基类只记不改——**"收到新意图要不要中止手头的事"是战术判断**，
        由子类决定。基类替它决定的话，所有控制器都会用同一套（假设的）
        规则，而那套规则没人验证过。
        """
        if intent.is_terminal:
            raise ConfigurationError(
                f"意图 #{intent.intent_id} 已经是终态，不该再下达给控制器"
            )
        self._intent = intent

    @property
    def current_intent(self) -> Intent | None:
        return self._intent

    def on_task(self, task: Task) -> None:
        """收到一条任务，记下来。

        按编号存储而不是按到达顺序：同一批任务的到达顺序取决于通信，
        而"先看哪条"应该由编号（下达顺序）决定——那是确定的。
        """
        self._tasks[task.task_id] = task

    @property
    def tasks(self) -> tuple[Task, ...]:
        return tuple(self._tasks[k] for k in sorted(self._tasks))

    @property
    def open_tasks(self) -> tuple[Task, ...]:
        return tuple(t for t in self.tasks if t.is_open)

    @property
    def is_idle(self) -> bool:
        """手上没有任何未完结的任务。"""
        return not self.open_tasks

    def forget(self, task_id: int) -> bool:
        return self._tasks.pop(task_id, None) is not None

    # -- 回调 --------------------------------------------------------------

    def on_deactivated(self, *, at: SimTime, reason: str) -> None:
        """失效时的钩子。子类覆盖——典型动作是把手上的意图/任务交还上级。

        基类不替它做，理由同 :meth:`on_intent`：宿主没了之后该"继续执行"
        还是"就地待命"是条令问题（§9 第 7 条），不能由框架默认。
        """

    # -- 输出 --------------------------------------------------------------

    def describe(self) -> str:
        level = CONTROLLER_LABELS.get(self.level, self.level or "未定级")
        host = (
            "无宿主"
            if self.host_id == NO_HOST
            else f"宿主 #{self.host_id}"
            + (f"（{self._lost_reason}）" if self.host_lost else "")
        )
        state = "生效" if self._active else "已失效"
        parts = [
            f"{level}控制器 {self.name}：{state}，{host}",
            f"任务 {len(self._tasks)} 条（未完结 {len(self.open_tasks)}）",
        ]
        if self._intent is not None:
            parts.append(f"意图 #{self._intent.intent_id}")
        if self.unit_state is not None:
            parts.append(self.unit_state.describe())
        return "，".join(parts)

    def to_dict(self) -> dict[str, Any]:
        return {
            "level": self.level,
            "name": self.name,
            "host_id": self.host_id,
            "active": self._active,
            "lost_reason": self._lost_reason,
            "intent_id": None if self._intent is None else self._intent.intent_id,
            "task_ids": [t.task_id for t in self.tasks],
        }

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}({self.name} level={self.level or '-'} "
            f"host={self.host_id} {'生效' if self._active else '已失效'})"
        )


__all__ = [
    "CONTROLLER_LABELS",
    "CONTROLLER_LEVELS",
    "Controller",
    "HostCheckFn",
    "LEVEL_ENTITY",
    "LEVEL_STRATEGIC",
    "LEVEL_UNIT",
    "NO_HOST",
]
