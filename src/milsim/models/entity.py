"""实体：部件的容器。

职责边界
--------
| 实体做什么 | 实体**不**做什么 |
|---|---|
| 装部件、按槽查部件 | 存位置（在 ``EntityStore``） |
| 转发 initialize / shutdown | 存阵营类型（在 ``EntityRegistry``） |
| 满足 ``RegisteredEntity`` 协议 | 参与调度（在 ``Engine``） |

实体自己几乎没有状态——**这是刻意的**。位置、航迹、损耗、消息全在
``EntityStore`` 里，因为那里的存储布局是为高频读写设计的（结构数组、
按 ID 索引），而且能靠视图对象控制谁能改什么。实体上再存一份就是双份真相。

实体也不是"对象"意义上的行为主体——行为在部件里。实体只是把部件按槽
组织起来，让"这个单位的雷达在哪"有个地方问。
"""

from __future__ import annotations

from typing import Any, Iterator

from ..errors import ConfigurationError
from .component import Component


class Entity:
    """一个平台实例。

    ``__slots__`` 里必须有 ``entity_id`` 与 ``name``——注册表通过
    ``setattr`` 写这两个字段，缺了会在注册时报错。
    """

    __slots__ = ("entity_id", "name", "platform_params", "_slots", "_order")

    def __init__(self) -> None:
        #: 由注册表分配。单调递增、永不复用（见 §3.7.2）。
        self.entity_id = -1
        self.name = ""
        #: 平台属性（§5.6），装配时解析好注入。
        #:
        #: 放实体上而不是让组件各自去查类型表：平台属性是**同一份**，
        #: 每个组件查一遍就多一份可能不一致的解析结果；而且"装配期解析、
        #: 运行期只读"这条纪律需要一个明确的落点。空字典而不是 ``None``，
        #: 调用方少一个判空分支——而漏判空是"参数静默丢失"的常见入口。
        self.platform_params: dict[str, Any] = {}
        self._slots: dict[str, Component] = {}
        self._order: list[str] = []

    # -- 部件管理 ----------------------------------------------------------

    def add(self, slot: str, component: Component) -> Component:
        """挂一个部件到槽上。

        同槽重复挂**直接报错**而不是覆盖——想定里同一槽写两次是笔误
        （想挂两个雷达需要两个不同的槽名），静默覆盖会丢掉一个部件而
        毫无提示。
        """
        if slot in self._slots:
            existing = self._slots[slot]
            raise ConfigurationError(
                f"{self.name or '实体'} 的槽 {slot!r} 已经有部件 "
                f"{existing.describe()}，不能再挂 {component.describe()}"
            )
        self._slots[slot] = component
        self._order.append(slot)
        component.bind(self)
        return component

    def component(self, slot: str) -> Component | None:
        return self._slots.get(slot)

    def require(self, slot: str) -> Component:
        """取部件，没有就报错。用于"这个实体必须有雷达"这类前提。"""
        found = self._slots.get(slot)
        if found is None:
            available = "、".join(self._order) or "（一个都没有）"
            raise KeyError(
                f"{self.name or '实体'} 没有 {slot!r} 槽。已有的槽：{available}"
            )
        return found

    def slots(self) -> list[str]:
        """槽名，**按挂载顺序**。保持顺序是为了遍历结果可复现。"""
        return list(self._order)

    def parts(self) -> list[Component]:
        return [self._slots[slot] for slot in self._order]

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        """按挂载顺序初始化各部件。

        顺序固定不是形式主义：部件常要读另一个部件的状态（通信部件要用
        机动部件的位置），挂载顺序就是它们的依赖顺序。
        """
        for slot in self._order:
            self._slots[slot].initialize(mount)

    def shutdown(self) -> None:
        """逆序关闭——先关依赖别人的，再关被依赖的。"""
        for slot in reversed(self._order):
            self._slots[slot].shutdown()

    # -- 协议 --------------------------------------------------------------

    def __contains__(self, slot: str) -> bool:
        return slot in self._slots

    def __iter__(self) -> Iterator[Component]:
        return iter(self.parts())

    def __len__(self) -> int:
        return len(self._slots)

    def __repr__(self) -> str:
        return (
            f"<Entity {self.name or '?'}(id={self.entity_id}) "
            f"{len(self._slots)} 个部件：{'、'.join(self._order) or '无'}>"
        )


__all__ = ["Entity"]
