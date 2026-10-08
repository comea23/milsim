"""指挥关系：三层，不是两层（设计见 §3.10）。

三层各用各的结构
----------------
============  ==============  ==============  ==========================
层            约束            结构            谁在用
============  ==============  ==============  ==========================
行政隶属      排他、稳定      树              补给、兵员补充、归建、复盘
作战归属      排他、可变      树              态势聚合、任务下达
指令通道      可叠加、临时    图              越级、协同、直接支援
============  ==============  ==============  ==========================

关键在二三两层的区别：

- **作战归属在任一时刻是唯一的** —— 一个连在某一刻只听一个上级作战指挥。
  配属是"**改接**"，不是"多接一条"。所以它是树。
- **指令通道可以叠加** —— 旅长可以越过营直接给某个连下令，这不改变那个连的
  作战归属。所以它才是图。

把第三层的"可叠加"误当成第二层的性质，就会得出"指挥关系是图"的结论，
进而让态势聚合出现**重复计数**——一个连同时挂在两个营下面，两边各算它
一份战力，而且不报错。本模块的 ``validate()`` 第一条就是防这个。

本模块不改任何状态
------------------
``parent_of(unit, at)`` 是 ``at`` 的纯函数：行政隶属树 + 配属调整 → 作战归属。
所以态势能被任意回溯——"红方当时以为一连在哪个营下面"是可以回答的。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from ..engine.time import SimTime
from ..errors import ConfigurationError
from .formation import FormationTree

#: 时间窗下界的默认值。
ALWAYS = 0


@dataclass(frozen=True, slots=True)
class TimeWindow:
    """一个左闭右开区间 ``[start, end)``。``end=None`` 表示不限。

    右开是刻意的：``until 8 h`` 与 ``from 8 h`` 相接时不应有重叠也不应有
    空隙。闭区间会让交接时刻同时属于两个窗口，触发"一个单位两个作战上级"
    的误报。
    """

    start: SimTime = ALWAYS
    end: SimTime | None = None

    def covers(self, at: SimTime) -> bool:
        if at < self.start:
            return False
        return self.end is None or at < self.end

    def overlaps(self, other: "TimeWindow") -> bool:
        a_end = self.end
        b_end = other.end
        # 无穷大用 None 表示，单独处理，避免引入浮点 inf 与 int 混算
        if b_end is not None and self.start >= b_end:
            return False
        if a_end is not None and other.start >= a_end:
            return False
        return True

    @property
    def is_open(self) -> bool:
        return self.start == ALWAYS and self.end is None

    def describe(self) -> str:
        if self.is_open:
            return "全程"

        def moment(value: SimTime) -> str:
            seconds = value / 1_000_000.0
            if seconds % 3600.0 == 0:
                return f"{seconds / 3600:.0f} h"
            if seconds % 60.0 == 0:
                return f"{seconds / 60:.0f} min"
            return f"{seconds:g} s"

        if self.end is None:
            return f"{moment(self.start)} 起"
        return f"{moment(self.start)} ~ {moment(self.end)}"


@dataclass(frozen=True, slots=True)
class Attachment:
    """配属：把 ``unit`` 的**作战归属**改接到 ``parent``。

    行政隶属不动——这正是它和 ``subordinate`` 的区别。
    """

    unit: int
    parent: int
    window: TimeWindow
    note: str = ""
    line: int = 0

    @property
    def is_permanent(self) -> bool:
        return self.window.is_open


@dataclass(frozen=True, slots=True)
class Directive:
    """直接指挥通道（越级）。

    ``superior`` 可以直接给 ``subordinate`` 下任务，**不改变**后者的作战归属。
    所以它进的是图，不是树。
    """

    superior: int
    subordinate: int
    window: TimeWindow
    line: int = 0


@dataclass(frozen=True, slots=True)
class Coordination:
    """协同通道。对等，不产生指挥方向。"""

    a: int
    b: int
    window: TimeWindow
    line: int = 0


class CommandChain:
    """三层指挥关系。``formations`` 提供行政隶属（第一层）。"""

    __slots__ = (
        "formations",
        "_attachments",
        "_directives",
        "_coordinations",
        "_cache_at",
        "_cache_map",
    )

    def __init__(self, formations: FormationTree | None = None) -> None:
        self.formations = formations if formations is not None else FormationTree()
        self._attachments: list[Attachment] = []
        self._directives: list[Directive] = []
        self._coordinations: list[Coordination] = []
        self._cache_at: SimTime | None = None
        self._cache_map: dict[int, int | None] = {}

    # -- 构建 --------------------------------------------------------------

    def attach(
        self,
        unit: "str | int",
        parent: "str | int",
        *,
        window: TimeWindow | None = None,
        note: str = "",
        line: int = 0,
    ) -> Attachment:
        """配属。改名/改名到自己的下级/跨阵营，都在这里当场报错。"""
        unit_id = self.formations.resolve(unit)
        parent_id = self.formations.resolve(parent)

        if unit_id == parent_id:
            raise ConfigurationError(
                f"编队 {self.formations.get(unit_id).name!r} 不能配属给自己"
            )
        if self.formations.side_of(unit_id) != self.formations.side_of(parent_id):
            raise ConfigurationError(
                f"跨阵营配属：{self.formations.get(unit_id).name!r}"
                f"（{self.formations.side_of(unit_id)}）→ "
                f"{self.formations.get(parent_id).name!r}"
                f"（{self.formations.side_of(parent_id)}）"
            )
        if parent_id in self.formations.descendants_of(unit_id):
            # 上级配属给自己的下级在语义上讲不通，而且几乎必然是笔误。
            # 不做动态成环检测时的兜底——静态就能拦住就别留到运行时。
            raise ConfigurationError(
                f"编队 {self.formations.get(unit_id).name!r} 不能配属到自己的下级 "
                f"{self.formations.get(parent_id).name!r}"
            )

        attachment = Attachment(
            unit=unit_id,
            parent=parent_id,
            window=window or TimeWindow(),
            note=note,
            line=line,
        )
        self._attachments.append(attachment)
        self._invalidate()
        return attachment

    def direct(
        self,
        superior: "str | int",
        subordinate: "str | int",
        *,
        window: TimeWindow | None = None,
        line: int = 0,
    ) -> Directive:
        """建立一条直接指挥通道（越级）。"""
        superior_id = self.formations.resolve(superior)
        subordinate_id = self.formations.resolve(subordinate)

        if superior_id == subordinate_id:
            raise ConfigurationError(
                f"编队 {self.formations.get(superior_id).name!r} 不能对自己建立直接通道"
            )
        if self.formations.side_of(superior_id) != self.formations.side_of(subordinate_id):
            raise ConfigurationError(
                f"跨阵营的直接指挥通道："
                f"{self.formations.get(superior_id).name!r} → "
                f"{self.formations.get(subordinate_id).name!r}"
            )

        directive = Directive(
            superior=superior_id,
            subordinate=subordinate_id,
            window=window or TimeWindow(),
            line=line,
        )
        self._directives.append(directive)
        return directive

    def coordinate(
        self,
        a: "str | int",
        b: "str | int",
        *,
        window: TimeWindow | None = None,
        line: int = 0,
    ) -> Coordination:
        """建立一条协同通道。对等，不产生指挥方向。"""
        a_id = self.formations.resolve(a)
        b_id = self.formations.resolve(b)

        if a_id == b_id:
            raise ConfigurationError(
                f"编队 {self.formations.get(a_id).name!r} 不能与自己建立协同"
            )
        if self.formations.side_of(a_id) != self.formations.side_of(b_id):
            raise ConfigurationError(
                f"跨阵营的协同通道：{self.formations.get(a_id).name!r} 与 "
                f"{self.formations.get(b_id).name!r}"
            )

        coordination = Coordination(
            a=a_id, b=b_id, window=window or TimeWindow(), line=line
        )
        self._coordinations.append(coordination)
        return coordination

    # -- 作战归属（第二层）-------------------------------------------------

    def parent_of(self, unit: int, at: SimTime) -> int | None:
        """``unit`` 在 ``at`` 时刻的**作战上级**。没有配属时就是行政上级。"""
        return self._operational_map(at).get(unit)

    def chain_of(self, unit: int, at: SimTime) -> tuple[int, ...]:
        """从根到自己。第三层（指令通道）不在这条链上。"""
        chain: list[int] = [unit]
        cursor = self.parent_of(unit, at)
        guard = len(self.formations) + 1
        while cursor is not None and guard > 0:
            chain.append(cursor)
            cursor = self.parent_of(cursor, at)
            guard -= 1
        chain.reverse()
        return tuple(chain)

    def root_of(self, unit: int, at: SimTime) -> int:
        return self.chain_of(unit, at)[0]

    def children_of(self, unit: int, at: SimTime) -> tuple[int, ...]:
        """作战下级。顺序稳定（按 ID 升序）。"""
        return tuple(
            child
            for child, parent in sorted(self._operational_map(at).items())
            if parent == unit
        )

    def subtree_of(self, unit: int, at: SimTime) -> tuple[int, ...]:
        """递归的作战下级（含自身），深度优先、顺序稳定。"""
        out: list[int] = []
        stack = [unit]
        while stack:
            current = stack.pop()
            out.append(current)
            stack.extend(reversed(self.children_of(current, at)))
        return tuple(out)

    def is_attached(self, unit: int, at: SimTime) -> bool:
        """此刻是否处于配属状态（作战上级 ≠ 行政上级）。"""
        return self.parent_of(unit, at) != self.formations.parent_of(unit)

    def attach_of(self, unit: int, at: SimTime) -> Attachment | None:
        """此刻生效的那条配属记录。反查归建时刻、备注用。"""
        for attachment in sorted(self._attachments, key=_attachment_key):
            if attachment.unit == unit and attachment.window.covers(at):
                return attachment
        return None

    def operational_map(self, at: SimTime) -> dict[int, int | None]:
        """``unit_id → 作战上级``，一次算好，投影时复用。"""
        return dict(self._operational_map(at))

    # -- 指令通道（第三层）-------------------------------------------------

    def direct_superiors_of(self, unit: int, at: SimTime) -> tuple[int, ...]:
        """此刻能越过中间层级、直接给 ``unit`` 下任务的单位。"""
        return tuple(sorted(
            item.superior for item in self._directives
            if item.subordinate == unit and item.window.covers(at)
        ))

    def direct_subordinates_of(self, unit: int, at: SimTime) -> tuple[int, ...]:
        return tuple(sorted(
            item.subordinate for item in self._directives
            if item.superior == unit and item.window.covers(at)
        ))

    def can_issue_direct(self, superior: int, subordinate: int, at: SimTime) -> bool:
        """``superior`` 有没有资格给 ``subordinate`` 下命令。

        两条路径任一成立即可：走作战归属链（正常指挥），或走直接通道（越级）。
        """
        if superior in self.chain_of(subordinate, at):
            return True
        return superior in self.direct_superiors_of(subordinate, at)

    def coordinators_of(self, unit: int, at: SimTime) -> tuple[int, ...]:
        peers = {
            item.b if item.a == unit else item.a
            for item in self._coordinations
            if unit in (item.a, item.b) and item.window.covers(at)
        }
        return tuple(sorted(peers))

    # -- 自检 --------------------------------------------------------------

    def validate(self, at: SimTime | None = None) -> list[str]:
        """全量自检。``at`` 为空时检查所有时间点的静态问题。"""
        problems: list[str] = []
        problems.extend(self._check_window_conflicts())
        problems.extend(self._check_redundant_directives())

        if at is not None:
            problems.extend(self._check_cycles(at))
        else:
            for moment in self._sample_times():
                for problem in self._check_cycles(moment):
                    if problem not in problems:
                        problems.append(problem)

        return problems

    def _check_window_conflicts(self) -> list[str]:
        """**本模块最要紧的一条校验。**

        作战归属是排他的，同一单位同一时刻不能有两个上级。时间段重叠时
        投影会把同一个实体算两次，兵力统计直接翻倍——而且不报错。
        """
        problems: list[str] = []
        grouped: dict[int, list[Attachment]] = {}
        for attachment in self._attachments:
            grouped.setdefault(attachment.unit, []).append(attachment)

        for unit_id in sorted(grouped):
            items = sorted(grouped[unit_id], key=lambda a: (a.window.start, a.line))
            name = self.formations.get(unit_id).name
            for index, left in enumerate(items):
                for right in items[index + 1:]:
                    if left.window.overlaps(right.window):
                        problems.append(
                            f"{name!r} 的配属时间段重叠："
                            f"{left.window.describe()}（配属到 "
                            f"{self.formations.get(left.parent).name!r}）与 "
                            f"{right.window.describe()}（配属到 "
                            f"{self.formations.get(right.parent).name!r}）——"
                            "同一时刻只能有一个作战上级"
                        )
        return problems

    def _check_redundant_directives(self) -> list[str]:
        """直接通道若已在作战归属链上，就是冗余——警告而非错误。"""
        problems: list[str] = []
        for item in sorted(self._directives, key=lambda d: (d.superior, d.subordinate)):
            if item.superior in self.chain_of(item.subordinate, item.window.start):
                problems.append(
                    f"直接通道 {self.formations.get(item.superior).name!r} → "
                    f"{self.formations.get(item.subordinate).name!r} 是冗余的："
                    "后者本来就在前者的作战归属链上"
                )
        return problems

    def _check_cycles(self, at: SimTime) -> list[str]:
        problems: list[str] = []
        parents = self._operational_map(at)
        for unit_id in sorted(parents):
            seen: set[int] = set()
            cursor: int | None = unit_id
            while cursor is not None:
                if cursor in seen:
                    cycle = " → ".join(
                        self.formations.get(u).name for u in sorted(seen)
                    )
                    problems.append(f"作战归属成环（含 {cycle}）")
                    break
                seen.add(cursor)
                cursor = parents.get(cursor)
        return problems

    def _sample_times(self) -> list[SimTime]:
        """所有时间窗的端点。成环只可能在配属生效/失效的瞬间出现或消失。"""
        moments: set[SimTime] = {0}
        for attachment in self._attachments:
            moments.add(attachment.window.start)
            if attachment.window.end is not None:
                moments.add(attachment.window.end)
        return sorted(moments)

    # -- 内部 --------------------------------------------------------------

    def _operational_map(self, at: SimTime) -> dict[int, int | None]:
        """算 ``at`` 时刻的作战归属：行政隶属打底，配属覆盖。

        每个单位最多有一条配属生效（``_check_window_conflicts`` 保证），
        所以覆盖是确定的、与遍历顺序无关。
        """
        if self._cache_at == at:
            return self._cache_map

        parents: dict[int, int | None] = {
            unit.unit_id: unit.parent_id for unit in self.formations
        }
        # 按 (unit, start) 排序后覆盖，结果与注册顺序无关
        for attachment in sorted(self._attachments, key=_attachment_key):
            if attachment.window.covers(at):
                parents[attachment.unit] = attachment.parent

        self._cache_at = at
        self._cache_map = parents
        return parents

    def _invalidate(self) -> None:
        self._cache_at = None
        self._cache_map = {}

    # -- 协议 --------------------------------------------------------------

    @property
    def attachments(self) -> tuple[Attachment, ...]:
        return tuple(sorted(self._attachments, key=_attachment_key))

    @property
    def directives(self) -> tuple[Directive, ...]:
        return tuple(sorted(self._directives, key=lambda d: (d.superior, d.subordinate)))

    @property
    def coordinations(self) -> tuple[Coordination, ...]:
        return tuple(sorted(
            self._coordinations, key=lambda c: (min(c.a, c.b), max(c.a, c.b))
        ))

    def __len__(self) -> int:
        return len(self.formations)

    def __iter__(self) -> Iterator[int]:
        return iter(sorted(unit.unit_id for unit in self.formations))

    def __repr__(self) -> str:
        return (
            f"<CommandChain 编队 {len(self.formations)} / "
            f"配属 {len(self._attachments)} / 直接通道 {len(self._directives)} / "
            f"协同 {len(self._coordinations)}>"
        )


def _attachment_key(attachment: Attachment) -> tuple[int, SimTime, int]:
    """排序键。与注册顺序无关，保证同种子两次运行结果一致。"""
    return (attachment.unit, attachment.window.start, attachment.line)


__all__ = [
    "ALWAYS",
    "Attachment",
    "CommandChain",
    "Coordination",
    "Directive",
    "TimeWindow",
]
