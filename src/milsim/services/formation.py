"""编队树：谁属于谁。

术语
----
本文统一用「**编队**」指组织单位（英文 ``Formation``），不用「编成」。
两者在军语里本有细微差别（编成侧重"由什么组成"，编队侧重"一起行动"），
但对本项目没有影响，统一成一个词以免文档里两个说法来回换。

**注意「编队」在此不是"队形"的意思**（飞机编队、舰艇编队那种空间排列）。
本模块只表达组织隶属，不表达几何队形。

为什么必须显式声明
------------------
态势投影（见 ``situation.py``）需要知道"哪几个平台算一个营"。这个信息
**不能推断**：

- 按实体名推断（``RED_1_2_3`` 属于 ``RED_1``）：命名不规范时静默建错拓扑
- 按位置聚类推断：部队展开后队形散开，聚类会把两个连合成一个"营"

推断出来的错误指挥关系**不会报错**，只会让命令发给错误的人——这类 bug
要等到推演跑到一半、某个单位始终不动时才暴露，复盘极其费劲。

所以编队是**声明式数据**，由想定层提供，本模块只负责存、查、自检。

指挥关系不在这棵树里
--------------------
本模块表达的是**隶属**（谁属于谁），不是**指挥**（谁给谁下命令）。
军事上两者会分离：配属、越级、协同。所以指挥关系是另一张**图**，
见 ``CommandChain``（M5a）。

不能把两者塞进同一张图的原因：隶属是**排他的**（一个连只属于一个营），
指挥是**可叠加的**（可以有多个指挥来源，按优先级取舍）。用图装隶属会
让"一个连属于两个营"变成合法状态，而那是错的。

与 ``Unit`` 的关系
------------------
本模块是**拓扑**（声明式，静态）；``UnitState``（M5a）装的是编队级的
**私有状态**（当前任务、上次报告时刻、本级判断）——那些推导不出来、
必须记住的东西。态势投影只是这棵树的一个**只读消费者**，不改任何状态。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator

from ..errors import ConfigurationError

#: 未归属任何编队的实体所用的伪编队 ID。仅投影时用，不占真实 ID 空间。
UNASSIGNED_ID = -1

#: 伪编队的显示名。
UNASSIGNED_NAME = "未编队"


@dataclass(slots=True)
class Formation:
    """一个编队节点。

    ``echelon`` 是纯显示标签（"旅" / "营" / "连" / "brigade" …），
    框架**不解释**它的含义。理由：不同国家、不同军种的层级名和层数都不同，
    硬编码成枚举会把使用者锁死。层级关系由树的深度表达，不由标签表达。
    """

    unit_id: int
    name: str
    side: str
    echelon: str = ""
    parent_id: int | None = None
    #: 直属的平台实体（不含下级编队的成员）
    members: list[int] = field(default_factory=list)
    #: 下级编队
    children: list[int] = field(default_factory=list)

    def describe(self) -> str:
        label = f"（{self.echelon}）" if self.echelon else ""
        return f"{self.name}{label}"


class FormationTree:
    """编队树。同时是索引：反查实体属于哪个编队。

    容量上刻意做成 ``list`` 而非 ``set``——遍历顺序必须稳定，
    否则同种子的两次推演会因聚合顺序不同而分叉（§6.3 确定性要求）。
    """

    __slots__ = ("_units", "_by_name", "_by_entity", "_next_id")

    def __init__(self) -> None:
        self._units: dict[int, Formation] = {}
        self._by_name: dict[str, int] = {}
        self._by_entity: dict[int, int] = {}
        self._next_id = 0

    # -- 构建 --------------------------------------------------------------

    def add_unit(
        self,
        name: str,
        *,
        side: str,
        echelon: str = "",
        parent: str | int | None = None,
    ) -> int:
        """新增一个编队节点，返回其 ID。

        父节点必须**已存在**。这从根上杜绝了成环——想造成环就得引用一个
        尚未创建的节点，那是不可能的。``set_parent()`` 才需要环检测。
        """
        if not name or name.strip() != name:
            raise ConfigurationError(f"编队名不能为空且首尾不能有空白：{name!r}")
        if name in self._by_name:
            raise ConfigurationError(
                f"编队名 {name!r} 已存在（id={self._by_name[name]}）"
            )
        if not side:
            raise ConfigurationError(f"编队 {name!r} 必须声明阵营")

        parent_id = self._resolve_unit(parent) if parent is not None else None
        if parent_id is not None:
            parent_side = self._units[parent_id].side
            if parent_side != side:
                raise ConfigurationError(
                    f"编队 {name!r} 的阵营 {side!r} 与上级 "
                    f"{self._units[parent_id].name!r} 的 {parent_side!r} 不一致"
                )

        unit_id = self._next_id
        self._next_id += 1
        self._units[unit_id] = Formation(
            unit_id=unit_id,
            name=name,
            side=side,
            echelon=echelon,
            parent_id=parent_id,
        )
        self._by_name[name] = unit_id
        if parent_id is not None:
            self._units[parent_id].children.append(unit_id)
        return unit_id

    def add_member(self, unit: str | int, entity_id: int) -> None:
        """把一个平台实体编入某个编队。一个实体只能属于一个编队。"""
        unit_id = self._resolve_unit(unit)
        existing = self._by_entity.get(entity_id)
        if existing is not None:
            raise ConfigurationError(
                f"实体 {entity_id} 已编入 {self._units[existing].name!r}，"
                f"不能再编入 {self._units[unit_id].name!r}"
            )
        self._units[unit_id].members.append(entity_id)
        self._by_entity[entity_id] = unit_id

    def add_members(self, unit: str | int, entity_ids: "list[int] | tuple[int, ...]") -> None:
        for entity_id in entity_ids:
            self.add_member(unit, entity_id)

    def set_parent(self, unit: str | int, parent: str | int | None) -> None:
        """改隶属（转隶）。会做环检测与阵营一致性检查。"""
        unit_id = self._resolve_unit(unit)
        parent_id = self._resolve_unit(parent) if parent is not None else None

        if parent_id is not None:
            if parent_id == unit_id:
                raise ConfigurationError(
                    f"编队 {self._units[unit_id].name!r} 不能以自己为上级"
                )
            if self._units[parent_id].side != self._units[unit_id].side:
                raise ConfigurationError(
                    f"转隶会跨越阵营：{self._units[unit_id].name!r}（"
                    f"{self._units[unit_id].side}）→ "
                    f"{self._units[parent_id].name!r}（{self._units[parent_id].side}）"
                )
            # 父节点不能是自己的下级——否则成环
            cursor: int | None = parent_id
            chain: list[str] = [self._units[parent_id].name]
            while cursor is not None:
                if cursor == unit_id:
                    raise ConfigurationError(
                        "转隶会成环："
                        + " → ".join([self._units[unit_id].name, *chain])
                    )
                cursor = self._units[cursor].parent_id
                if cursor is not None:
                    chain.append(self._units[cursor].name)

        node = self._units[unit_id]
        old = node.parent_id
        if old is not None:
            self._units[old].children.remove(unit_id)
        node.parent_id = parent_id
        if parent_id is not None:
            self._units[parent_id].children.append(unit_id)

    # -- 查询 --------------------------------------------------------------

    def get(self, unit_id: int) -> Formation | None:
        return self._units.get(unit_id)

    def by_name(self, name: str) -> Formation | None:
        unit_id = self._by_name.get(name)
        return self._units[unit_id] if unit_id is not None else None

    def id_of(self, name: str) -> int:
        unit_id = self._by_name.get(name)
        if unit_id is None:
            raise ConfigurationError(f"未定义的编队 {name!r}")
        return unit_id

    def resolve(self, ref: "str | int") -> int:
        """名字或 ID → ID。名字认不出来时报错并给拼写建议。

        对外提供这个入口，是为了让指挥层（``command.py``）不必去碰
        ``_resolve_unit`` 这种内部方法。
        """
        return self._resolve_unit(ref)

    def parent_of(self, unit_id: int) -> int | None:
        node = self._units.get(unit_id)
        return node.parent_id if node is not None else None

    def children_of(self, unit_id: int) -> tuple[int, ...]:
        node = self._units.get(unit_id)
        return tuple(node.children) if node is not None else ()

    def members_of(self, unit_id: int) -> tuple[int, ...]:
        """**直属**平台实体，不含下级编队的成员。"""
        node = self._units.get(unit_id)
        return tuple(node.members) if node is not None else ()

    def descendants_of(self, unit_id: int) -> tuple[int, ...]:
        """递归的所有下级编队（不含自身），深度优先、顺序稳定。"""
        out: list[int] = []
        stack = list(reversed(self.children_of(unit_id)))
        while stack:
            current = stack.pop()
            out.append(current)
            stack.extend(reversed(self.children_of(current)))
        return tuple(out)

    def all_entities_below(self, unit_id: int) -> tuple[int, ...]:
        """递归的所有平台实体：本级直属 + 所有下级编队的。"""
        out = list(self.members_of(unit_id))
        for child in self.descendants_of(unit_id):
            out.extend(self.members_of(child))
        return tuple(out)

    def unit_of_entity(self, entity_id: int) -> int | None:
        """反查实体所属编队。未编入返回 ``None``。"""
        return self._by_entity.get(entity_id)

    def side_of(self, unit_id: int) -> str:
        node = self._units.get(unit_id)
        if node is None:
            raise ConfigurationError(f"未定义的编队 id={unit_id}")
        return node.side

    def roots(self) -> tuple[int, ...]:
        """没有上级的编队，按 ID 升序。"""
        return tuple(sorted(
            uid for uid, node in self._units.items() if node.parent_id is None
        ))

    def units_of_side(self, side: str) -> tuple[int, ...]:
        return tuple(sorted(
            uid for uid, node in self._units.items() if node.side == side
        ))

    def sides(self) -> list[str]:
        seen: dict[str, None] = {}
        for uid in sorted(self._units):
            seen.setdefault(self._units[uid].side, None)
        return list(seen)

    def depth_of(self, unit_id: int) -> int:
        """层级深度，根为 0。用来给聚合分层（§3.9.4）。"""
        depth = 0
        cursor = self._units.get(unit_id)
        while cursor is not None and cursor.parent_id is not None:
            depth += 1
            cursor = self._units.get(cursor.parent_id)
        return depth

    # -- 自检 --------------------------------------------------------------

    def validate(self) -> list[str]:
        """全量自检。编队拓扑错了不会报错、只会让命令发错人，
        所以这个方法是刚需，不是锦上添花。
        """
        problems: list[str] = []

        for uid in sorted(self._units):
            node = self._units[uid]
            if node.parent_id is not None:
                parent = self._units.get(node.parent_id)
                if parent is None:
                    problems.append(f"编队 {node.name!r} 的上级 id={node.parent_id} 不存在")
                else:
                    if parent.side != node.side:
                        problems.append(
                            f"编队 {node.name!r}（{node.side}）挂在 "
                            f"{parent.name!r}（{parent.side}）下"
                        )
                    if uid not in parent.children:
                        problems.append(f"父子索引不一致：{node.name!r} 未出现在上级的 children 里")

            if len(node.children) != len(set(node.children)):
                problems.append(f"编队 {node.name!r} 的 children 有重复")
            if len(node.members) != len(set(node.members)):
                problems.append(f"编队 {node.name!r} 的 members 有重复")

            # 根节点不可能成环；从每个节点往上走，超过总节点数即为环
            cursor, steps = node.parent_id, 0
            while cursor is not None:
                steps += 1
                if steps > len(self._units):
                    problems.append(f"编队 {node.name!r} 的上级链成环")
                    break
                parent = self._units.get(cursor)
                cursor = parent.parent_id if parent is not None else None

        # 反向索引与 members 必须一致
        for entity_id, uid in sorted(self._by_entity.items()):
            node = self._units.get(uid)
            if node is None:
                problems.append(f"实体的编队索引指向不存在的 id={uid}")
            elif entity_id not in node.members:
                problems.append(
                    f"反向索引不一致：实体 {entity_id} 记在 {node.name!r}，"
                    "但该编队的 members 里没有它"
                )

        for uid in sorted(self._units):
            for entity_id in self._units[uid].members:
                if self._by_entity.get(entity_id) != uid:
                    problems.append(
                        f"实体 {entity_id} 在 {self._units[uid].name!r} 的 members 里，"
                        "但反向索引指向别处"
                    )

        return problems

    # -- 内部 --------------------------------------------------------------

    def _resolve_unit(self, ref: "str | int") -> int:
        if isinstance(ref, int):
            if ref not in self._units:
                raise ConfigurationError(f"未定义的编队 id={ref}")
            return ref
        unit_id = self._by_name.get(ref)
        if unit_id is None:
            hint = self._name_hint(ref)
            raise ConfigurationError(f"未定义的编队 {ref!r}{hint}")
        return unit_id

    def _name_hint(self, name: str) -> str:
        from difflib import get_close_matches

        close = get_close_matches(name, self._by_name, n=1)
        return f"（是否想写 {close[0]!r}？）" if close else ""

    # -- 协议 --------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._units)

    def __contains__(self, unit_id: object) -> bool:
        return unit_id in self._units

    def __iter__(self) -> Iterator[Formation]:
        for uid in sorted(self._units):
            yield self._units[uid]

    def __repr__(self) -> str:
        counts: dict[str, int] = {}
        for node in self._units.values():
            counts[node.side] = counts.get(node.side, 0) + 1
        detail = "、".join(f"{side} {n} 个" for side, n in sorted(counts.items()))
        return f"<FormationTree {detail or '空'} / 实体 {len(self._by_entity)}>"


__all__ = [
    "UNASSIGNED_ID",
    "UNASSIGNED_NAME",
    "Formation",
    "FormationTree",
]
