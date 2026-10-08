"""态势投影：``EntityStore`` → 某方认知的战场（设计见 §3.9）。

定位
----
``EntityStore`` 是**真实世界**：上帝视角、精确、覆盖全体。
本模块产出的是**某一方所认知的战场**：过滤过、带误差、可能漏报。

两者不是同一个东西。本模块存在的全部理由就是把它们分开——
红方宏观智能体不该知道蓝方坦克的真实坐标，它只知道雷达报上来的航迹。

一条必须守住的边界
------------------
**本模块不能只服务于前端。** 若只给前端加投影、各层智能体仍直读
``EntityStore``，智能体照旧上帝视角作弊。一份投影，两类消费者。

唯一不能弄错的顺序
------------------
正确：``EntityStore → 按方过滤 → 聚合成编队 → 裁剪 → 消费者``
错误：``EntityStore → 聚合成编队 → 按方过滤 → 消费者``

第二种顺序下，敌方编队的质心是**用真实位置算出来的**。即使随后把敌方
单个实体过滤掉，"质心在 (x, y)"这个聚合量已经携带了真实信息——
表现为宏观智能体总能精确知道敌方主力方向，明明一个目标都没探测到。
这个错误**不会报错**，只会让结果"看起来合理"。

两个正交的标记
--------------
``from_truth`` 与 ``is_own`` 是两件事，不能合并：

| | 含义 | 上帝视角 | 红方视角 |
|---|---|---|---|
| ``from_truth`` | 数据来源是真实状态还是航迹 | 全 True | 己方 True / 敌方 False |
| ``is_own`` | 是否属于观察方 | 全 False（没有"我方"） | 己方 True / 敌方 False |

合并的话，上帝视角下"敌方"这个概念会不成立却又被迫给出一个值。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any, Callable

from ..engine.time import SimTime
from .command import CommandChain
from .formation import UNASSIGNED_ID, UNASSIGNED_NAME, FormationTree
from .registry import EntityRegistry
from .store import Contact, EntityStore

#: 上帝视角标识：投影时不传 ``side`` 即为此。
OMNISCIENT: None = None

#: 按**作战归属**聚合（默认）。指挥员关心的是"我手下哪些部队在打"。
BASIS_OPERATIONAL = "operational"

#: 按**行政隶属**聚合。补给、归建、复盘"这兵是谁的"用这个。
BASIS_ADMINISTRATIVE = "administrative"

#: 番号未识别时的显示名。
UNIDENTIFIED_UNIT = "未识别编队"
UNIDENTIFIED_CONTACT = "未识别目标"


def format_sim_time(micros: SimTime) -> str:
    """微秒 → ``T+HH:MM:SS``。仅用于人读的输出。"""
    total_s, _ = divmod(int(micros), 1_000_000)
    hours, rest = divmod(total_s, 3600)
    minutes, seconds = divmod(rest, 60)
    return f"T+{hours:02d}:{minutes:02d}:{seconds:02d}"


@dataclass(frozen=True, slots=True)
class EntityInfo:
    """投影后的单个实体。

    ``from_truth`` 为假表示这是**航迹**而非真实目标——位置是估计值，
    带 ``quality`` 与 ``age_us``。这个区分不能丢：前端把它们画成两种
    符号，决策层据此判断情报可靠性。

    ``name`` 与 ``designation`` 是两个东西：

    - ``name`` 是**真实番号**，内部标识用，**不得显示给敌对视角**
    - ``designation`` 是**显示名**。己方为真实番号；敌方需情报识别，
      未识别时是"未识别目标"

    ``to_dict()`` 只导出 ``designation``，不导出 ``name``。
    """

    entity_id: int
    name: str
    designation: str
    side: str
    type_name: str
    is_own: bool
    from_truth: bool
    has_position: bool
    x: float
    y: float
    z: float
    heading_deg: float
    speed_mps: float
    #: 完好度。仅己方可知——敌方战损要靠 BDA 推断，本层不做。
    strength: float | None
    #: 信息质量。真实数据恒为 1.0；航迹取其 quality。
    quality: float
    #: 信息年龄（微秒）。真实数据为 0。
    age_us: SimTime
    #: 番号识别置信度。己方恒为 1.0；敌方由情报决定，未识别为 0.0。
    #: 这是留给 M4 感知层的**接口位**——识别会出错是真实存在的现象。
    designation_confidence: float = 1.0

    def distance_to(self, x: float, y: float) -> float:
        return hypot(self.x - x, self.y - y)


@dataclass(frozen=True, slots=True)
class FormationInfo:
    """编队级聚合。单实体信息对指挥层没有意义，编队才有。"""

    unit_id: int
    name: str
    designation: str
    side: str
    echelon: str
    parent_id: int | None
    is_own: bool
    from_truth: bool
    #: 此刻是否处于配属状态（作战上级 ≠ 行政上级）。见 §3.10。
    attached: bool
    has_position: bool
    x: float
    y: float
    z: float
    #: 成员到质心的最大距离。反映展开程度。
    spread_m: float
    #: 该方**确认存在**的实体数。
    observed: int
    #: 编制数。**只有己方知道**——敌方编制是未知量，
    #: 把观测数当成编制数是最常见的建模错误。
    declared: int | None
    #: 战力（下级完好度均值）。仅己方算得出。
    strength: float | None
    #: 聚合置信度。真实数据 1.0；航迹取质量均值。
    confidence: float
    #: 番号是否已识别。己方恒为真；敌方要靠情报。
    identified: bool
    #: 番号识别置信度。留给 M4 感知层的接口位——识别会出错是真实现象。
    designation_confidence: float
    #: 最新信息的时刻。
    latest_at: SimTime

    def describe(self) -> str:
        """显示名。**未识别时不要暴露番号和层级**。"""
        if not self.identified:
            return self.designation
        label = f"（{self.echelon}）" if self.echelon else ""
        return f"{self.designation}{label}"


@dataclass(frozen=True, slots=True)
class Situation:
    """一份态势投影。**不可变**——它是某一时刻的快照，改了就没法回溯。"""

    #: ``None`` 表示上帝视角（复盘用）。
    side: str | None
    at: SimTime
    formations: tuple[FormationInfo, ...]
    entities: tuple[EntityInfo, ...]

    @property
    def is_omniscient(self) -> bool:
        return self.side is None

    @property
    def own(self) -> tuple[FormationInfo, ...]:
        """观察方自己的编队。上帝视角下为空——那时请用 ``by_side``。"""
        if self.is_omniscient:
            return ()
        return tuple(f for f in self.formations if f.is_own)

    @property
    def contacts(self) -> tuple[FormationInfo, ...]:
        """观察方探测到的敌方。上帝视角下为空——那时一切都是真实数据。"""
        if self.is_omniscient:
            return ()
        return tuple(f for f in self.formations if not f.is_own)

    @property
    def by_side(self) -> dict[str, tuple[FormationInfo, ...]]:
        """按阵营分组。两种视角都适用，上帝视角下用它。"""
        grouped: dict[str, list[FormationInfo]] = {}
        for info in self.formations:
            grouped.setdefault(info.side, []).append(info)
        return {side: tuple(items) for side, items in grouped.items()}

    def entity(self, entity_id: int) -> EntityInfo | None:
        for info in self.entities:
            if info.entity_id == entity_id:
                return info
        return None

    def summary(self) -> str:
        who = self.side if self.side else "上帝视角"
        return (
            f"[{who}] {format_sim_time(self.at)} "
            f"编队 {len(self.formations)} / 实体 {len(self.entities)}"
        )

    def to_dict(self) -> dict[str, Any]:
        """给前端的结构。与摘要文本同源，保证"显示的和算的一致"。"""
        return {
            "side": self.side,
            "at": self.at,
            "at_text": format_sim_time(self.at),
            "formations": [
                {
                    "unit_id": f.unit_id,
                    "name": f.name,
                    "side": f.side,
                    "designation": f.designation,
                    "identified": f.identified,
                    "designation_confidence": f.designation_confidence,
                    "echelon": f.echelon if f.identified else "",
                    "parent_id": f.parent_id,
                    "is_own": f.is_own,
                    "from_truth": f.from_truth,
                    "attached": f.attached,
                    "x": f.x,
                    "y": f.y,
                    "z": f.z,
                    "spread_m": f.spread_m,
                    "observed": f.observed,
                    "declared": f.declared,
                    "strength": f.strength,
                    "confidence": f.confidence,
                    "latest_at": f.latest_at,
                }
                for f in self.formations
            ],
            "entities": [
                {
                    "entity_id": e.entity_id,
                    "designation": e.designation,
                    "designation_confidence": e.designation_confidence,
                    "side": e.side,
                    # type_name 也脱敏：型号是情报，不是观测值
                    "type_name": e.type_name if e.is_own or e.designation_confidence > 0 else "",
                    "is_own": e.is_own,
                    "from_truth": e.from_truth,
                    "x": e.x,
                    "y": e.y,
                    "z": e.z,
                    "heading_deg": e.heading_deg,
                    "strength": e.strength,
                    "quality": e.quality,
                    "age_us": e.age_us,
                }
                for e in self.entities
            ],
        }


# ---------------------------------------------------------------------------
# 投影器
# ---------------------------------------------------------------------------
class SituationProjector:
    """把 ``EntityStore`` 投影成某方认知的态势。

    纯函数式：``project()`` **不修改任何状态**。三个好处（§3.9.6）：

    1. 前端与智能体拿到同一份数据 —— "显示的和算的不会不一致"
    2. 可任意回溯：``project(side="red", at=T)`` 得到 T 时刻红方看到什么
    3. 可并行算多个视角而互不干扰
    """

    __slots__ = ("registry", "store", "formations", "chains", "_clock")

    def __init__(
        self,
        registry: EntityRegistry,
        store: EntityStore,
        formations: FormationTree | None = None,
        clock: Callable[[], SimTime] | None = None,
        chains: CommandChain | None = None,
    ) -> None:
        self.registry = registry
        self.store = store
        if formations is None and chains is not None:
            formations = chains.formations
        self.formations = formations if formations is not None else FormationTree()
        self.chains = chains
        self._clock = clock

    # -- 对外 --------------------------------------------------------------

    def project(
        self,
        *,
        side: str | None = OMNISCIENT,
        at: SimTime | None = None,
        basis: str = BASIS_OPERATIONAL,
    ) -> Situation:
        """产出某方在 ``at`` 时刻认知的态势。

        ``side=None`` 得到上帝视角（真实世界）。复盘/效能评估用这个，
        指挥训练与对抗评估用具体阵营。

        ``basis`` 决定编队的**层级**按哪套关系算（§3.10.2）：

        - ``operational``（默认）—— 作战归属。指挥员关心的是"我手下哪些
          部队在打"，配属来的部队应该算在受援单位名下
        - ``administrative`` —— 行政隶属。补给、归建、复盘"这兵是谁的"

        注意两种依据下**平台挂在哪一级是不变的**（配属改的是编队之间的
        上下级，不是"哪个平台属于哪个连"），变的是 ``parent_id`` 这条链。
        """
        if basis not in (BASIS_OPERATIONAL, BASIS_ADMINISTRATIVE):
            raise ValueError(
                f"未知的聚合依据 {basis!r}（只支持 "
                f"{BASIS_OPERATIONAL!r} / {BASIS_ADMINISTRATIVE!r}）"
            )
        if at is None:
            now = self._clock() if self._clock is not None else self._latest_known_time()
        else:
            now = at

        infos = self._collect(side, now)
        formations = self._aggregate(side, infos, now, basis)
        return Situation(
            side=side,
            at=now,
            formations=formations,
            entities=tuple(infos.values()),
        )

    def sides(self) -> list[str]:
        """场上所有阵营，顺序稳定。"""
        return self.registry.sides()

    def known_ids(self, side: str | None, at: SimTime | None = None) -> tuple[int, ...]:
        """该方"知道存在"的实体 ID。**这是视野限制最直接的口径**。"""
        if at is None:
            at = self._clock() if self._clock is not None else self._latest_known_time()
        return tuple(self._collect(side, at))

    # -- 内部 --------------------------------------------------------------

    def _latest_known_time(self) -> SimTime:
        """既没给时钟也没给时刻时的兜底：取所有航迹里最新的时刻。

        比默默用 0 好——用 0 会把所有信息年龄都算成 0，
        看起来"情报全都是新鲜的"。
        """
        latest = 0
        for entity_id in self.registry.all_ids():
            for contact in self.store.contacts_of(entity_id):
                if contact.detected_at > latest:
                    latest = contact.detected_at
        return latest

    def _collect(self, side: str | None, at: SimTime) -> dict[int, EntityInfo]:
        """第 1 步：**按方过滤**。必须在聚合之前（见模块 docstring）。"""
        infos: dict[int, EntityInfo] = {}

        if side is None:
            for entity_id in self.registry.all_ids():
                info = self._from_truth(entity_id, at, viewer_side=None)
                if info is not None:
                    infos[entity_id] = info
            return infos

        own_ids = set(self.registry.by_side(side))

        # 己方：真实状态。C2 链路共享己方位置，不走"探测"。
        for entity_id in sorted(own_ids):
            info = self._from_truth(entity_id, at, viewer_side=side)
            if info is not None:
                infos[entity_id] = info

        # 敌方：只保留被探测到的，用航迹位置
        for target_id, contact in self._fuse_contacts(own_ids):
            info = self._from_contact(target_id, contact, at)
            if info is not None:
                infos[target_id] = info

        return infos

    def _from_truth(
        self, entity_id: int, at: SimTime, viewer_side: str | None
    ) -> EntityInfo | None:
        """用**真实**状态构造。仅用于己方实体与上帝视角。"""
        if not self.store.has(entity_id):
            return None
        entity = self.registry.get(entity_id)
        if entity is None:
            return None

        position = self.store.position_of(entity_id)
        pose = self.store.pose_of(entity_id)
        heading = pose[3] if pose is not None else 0.0
        speed = pose[4] if pose is not None else 0.0
        side = self.registry.side_of(entity_id)

        return EntityInfo(
            entity_id=entity_id,
            name=entity.name,
            designation=entity.name,
            side=side,
            type_name=self.registry.type_of(entity_id),
            is_own=(viewer_side is not None and side == viewer_side),
            from_truth=True,
            has_position=position is not None,
            x=position[0] if position else 0.0,
            y=position[1] if position else 0.0,
            z=position[2] if position else 0.0,
            heading_deg=heading,
            speed_mps=speed,
            strength=self.store.damage_of(entity_id),
            quality=1.0,
            age_us=0,
            designation_confidence=1.0,
        )

    def _from_contact(
        self, target_id: int, contact: Contact, at: SimTime
    ) -> EntityInfo | None:
        """用**航迹**构造。位置是估计值，不是真实值。"""
        entity = self.registry.get(target_id)
        if entity is None:
            return None

        # 直接相减，**不特判 detected_at == 0**。
        #
        # 一度写成 ``if contact.detected_at else 0``，意思是"没打时间戳就当
        # 刚更新"——但那是在**伪造情报新鲜度**：一条 T=0 时刻的航迹，
        # 到 T=600 s 仍显示"刚刚更新"，决策层会当成实时情报用。
        #
        # 时间戳为 0 的诚实的读法是"信息来自 T=0，已经 600 秒了"。
        # 感知模型本来就该在写航迹时打上 detected_at；没打是它的问题，
        # 不该由投影层替它掩盖。
        age = max(0, at - contact.detected_at)

        return EntityInfo(
            entity_id=target_id,
            name=entity.name,
            # **不暴露真实番号**。识别敌方番号是情报产物，不是观测值——
            # 直接显示等于白送对方建制。位置留在这里，等 M4 感知层
            # 给出识别结果（含置信度）再填。
            designation=UNIDENTIFIED_CONTACT,
            side=self.registry.side_of(target_id),
            type_name=self.registry.type_of(target_id),
            is_own=False,
            from_truth=False,
            has_position=True,
            x=contact.x,
            y=contact.y,
            z=contact.z,
            heading_deg=0.0,
            speed_mps=0.0,
            # 敌方完好度不可知——本层不做 BDA 推断
            strength=None,
            quality=contact.quality,
            age_us=age,
            designation_confidence=0.0,
        )

    def _fuse_contacts(self, own_ids: set[int]) -> list[tuple[int, Contact]]:
        """融合本方所有观察者的航迹。

        同一目标可能被多部雷达同时跟踪。取**质量最高**的一条；
        质量相同则取**最新**的。规则写死为字典序比较，否则聚合结果
        会随遍历顺序变化，破坏可复现性。
        """
        best: dict[int, Contact] = {}
        for observer in sorted(own_ids):
            for contact in self.store.contacts_of(observer):
                if contact.target_id in own_ids:
                    continue                     # 自己人的航迹不是敌情
                previous = best.get(contact.target_id)
                if previous is None or (
                    (contact.quality, contact.detected_at)
                    > (previous.quality, previous.detected_at)
                ):
                    best[contact.target_id] = contact
        return [(target_id, best[target_id]) for target_id in sorted(best)]

    def _aggregate(
        self,
        side: str | None,
        infos: dict[int, EntityInfo],
        at: SimTime,
        basis: str,
    ) -> tuple[FormationInfo, ...]:
        """第 2 步：聚合成编队。输入已经是**过滤后**的实体集合。"""
        grouped: dict[int, list[EntityInfo]] = {}
        loose: dict[str, list[EntityInfo]] = {}

        for entity_id in sorted(infos):
            info = infos[entity_id]
            unit_id = self.formations.unit_of_entity(entity_id)
            if unit_id is None:
                loose.setdefault(info.side, []).append(info)
            else:
                grouped.setdefault(unit_id, []).append(info)

        # 番号未识别的敌方编队要编号：否则多个未知编队都叫"未识别编队"，
        # 没法分辨。按 unit_id 升序编号，顺序稳定、可复现。
        unidentified = _number_unidentified(
            sorted(
                unit_id for unit_id in grouped
                if side is not None and self.formations.side_of(unit_id) != side
            )
        )

        out: list[FormationInfo] = []
        for unit_id in sorted(grouped):
            node = self.formations.get(unit_id)
            if node is None:
                continue
            is_own = side is not None and node.side == side
            from_truth = side is None or is_own
            identified = from_truth          # 己方与上帝视角都认得番号

            if identified:
                designation = node.name
            else:
                designation = unidentified.get(unit_id, UNIDENTIFIED_UNIT)

            out.append(self._make_info(
                unit_id=unit_id,
                name=node.name,
                designation=designation,
                side=node.side,
                echelon=node.echelon,
                parent_id=self._parent_of(unit_id, at, basis),
                members=grouped[unit_id],
                is_own=is_own,
                from_truth=from_truth,
                attached=self._is_attached(unit_id, at, basis),
                identified=identified,
                declared=len(self.formations.all_entities_below(unit_id)),
                at=at,
            ))

        # 未编队实体单独成伪编队，**按阵营分**——红蓝的散兵不能合并
        for loose_side in sorted(loose):
            members = loose[loose_side]
            is_own = side is not None and loose_side == side
            from_truth = side is None or is_own
            out.append(self._make_info(
                unit_id=UNASSIGNED_ID,
                name=UNASSIGNED_NAME,
                designation=UNASSIGNED_NAME,
                side=loose_side,
                echelon="",
                parent_id=None,
                members=members,
                is_own=is_own,
                from_truth=from_truth,
                attached=False,
                identified=from_truth,
                declared=len(members) if is_own else None,
                at=at,
            ))

        return tuple(out)

    def _parent_of(self, unit_id: int, at: SimTime, basis: str) -> int | None:
        """按依据取上级。配属改的是这条链，不动"平台属于哪个连"。"""
        if basis == BASIS_ADMINISTRATIVE or self.chains is None:
            return self.formations.parent_of(unit_id)
        return self.chains.parent_of(unit_id, at)

    def _is_attached(self, unit_id: int, at: SimTime, basis: str) -> bool:
        """在本视图下，这个单位的作战上级是否与行政上级不同。

        必须跟着 ``basis`` 走：行政视角里它挂在行政上级下面，
        再标一个"（配属）"就自相矛盾了。
        """
        if basis != BASIS_OPERATIONAL or self.chains is None:
            return False
        return self.chains.is_attached(unit_id, at)

    def _make_info(
        self,
        *,
        unit_id: int,
        name: str,
        designation: str,
        side: str,
        echelon: str,
        parent_id: int | None,
        members: list[EntityInfo],
        is_own: bool,
        from_truth: bool,
        attached: bool,
        identified: bool,
        declared: int | None,
        at: SimTime,
    ) -> FormationInfo:
        located = [m for m in members if m.has_position]

        if located:
            count = float(len(located))
            cx = sum(m.x for m in located) / count
            cy = sum(m.y for m in located) / count
            cz = sum(m.z for m in located) / count
            spread = max(((m.x - cx) ** 2 + (m.y - cy) ** 2) ** 0.5 for m in located)
        else:
            cx = cy = cz = spread = 0.0

        if from_truth:
            alive = [m for m in members if (m.strength or 0.0) > 0.0]
            strength = (
                sum(m.strength or 0.0 for m in alive) / len(alive) if alive else 0.0
            )
            confidence = 1.0
            declared = declared if declared is not None else len(members)
            designation_confidence = 1.0
        else:
            strength = None
            confidence = (
                sum(m.quality for m in members) / len(members) if members else 0.0
            )
            # **必须置空**。declared 传进来的是编队树里的真实编制数——
            # 留着它就等于把敌方番号规模白送给对手：
            # "至少观测到 1 个目标，但这支部队编制是 3 个连"。
            # 这个泄漏不报错、看着还挺合理，是最难发现的一类。
            declared = None
            designation_confidence = 0.0

        age = max((m.age_us for m in members), default=0)

        return FormationInfo(
            unit_id=unit_id,
            name=name,
            designation=designation,
            side=side,
            echelon=echelon,
            parent_id=parent_id,
            is_own=is_own,
            from_truth=from_truth,
            attached=attached,
            has_position=bool(located),
            x=cx,
            y=cy,
            z=cz,
            spread_m=spread,
            observed=len(members),
            declared=declared,
            strength=strength,
            confidence=confidence,
            identified=identified,
            designation_confidence=designation_confidence,
            latest_at=at - age,
        )

    def __repr__(self) -> str:
        return (
            f"<SituationProjector 编队 {len(self.formations)} / "
            f"实体 {len(self.registry)}>"
        )


# ---------------------------------------------------------------------------
# 摘要器：态势 → 给大模型读的自然语言
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class DigestOptions:
    """裁剪参数。

    **这里只做"看多少"（scope），不做"能知道什么"（epistemic）。**
    按方过滤已在投影里完成，不可关。把这个区别守住，才不会出现
    "为了省 token 缩小视野"与"为了正确性过滤敌方"互相干扰（§3.9.2）。
    """

    include_own: bool = True
    include_contacts: bool = True
    #: 以某点为中心的视野半径（米）。``None`` 表示不限。
    center: tuple[float, float] | None = None
    radius_m: float | None = None
    #: 最多列几个编队。给中心时应先按邻近排序再截断。
    max_formations: int = 24
    #: 低于此置信度的接触不列。情报质量差时不必占上下文。
    min_confidence: float = 0.0
    max_chars: int = 4000


def digest(situation: Situation, options: DigestOptions | None = None) -> str:
    """把态势压成给大模型读的文本。

    刻意用**固定的、结构化的**格式而不是自由散文：模型要从里面读数值做
    判断，格式抖动会让它把"12.3 km"读成"12.3"（丢掉单位）。
    """
    opts = options or DigestOptions()
    lines: list[str] = []

    who = situation.side if situation.side else "上帝视角"
    lines.append(f"【{who}态势】{format_sim_time(situation.at)}")

    if situation.is_omniscient:
        for side in sorted(situation.by_side):
            items = _select(situation.by_side[side], opts)
            lines.append("")
            lines.append(f"{side}（{len(items)}）")
            for info in items:
                lines.append("  · " + _describe(info, situation.at, own=True))
    else:
        if opts.include_own:
            own = _select(situation.own, opts)
            lines.append("")
            lines.append(f"本方编队（{len(own)}）")
            if not own:
                lines.append("  （无）")
            for info in own:
                lines.append("  · " + _describe(info, situation.at, own=True))

        if opts.include_contacts:
            contacts = _select(situation.contacts, opts)
            lines.append("")
            lines.append(f"敌方动向 · 已探测（{len(contacts)}）")
            if not contacts:
                lines.append("  （无）")
            for info in contacts:
                lines.append("  · " + _describe(info, situation.at, own=False))

    text = "\n".join(lines)
    if len(text) > opts.max_chars:
        text = text[: opts.max_chars - 20] + "\n…（已截断）"
    return text


def _number_unidentified(unit_ids: list[int]) -> dict[int, str]:
    """给番号未识别的敌方编队编号。

    不编号的话多个未知编队都叫"未识别编队"，前端和摘要里分不出是几个。
    按 unit_id 升序编号，顺序稳定、可复现。

    用 ``#N`` 而不是空格分隔，是为了和己方的"编队 3"（编制数）
    在文本上**一眼可辨**——两者混起来读很容易把序号当成规模。
    """
    return {
        unit_id: f"{UNIDENTIFIED_UNIT}#{index}"
        for index, unit_id in enumerate(unit_ids, start=1)
    }


def _select(
    items: tuple[FormationInfo, ...], opts: DigestOptions
) -> list[FormationInfo]:
    """裁剪：按视野半径过滤、按邻近程度排序、按数量截断。

    只做 scope，不做 epistemic——按方过滤已在投影阶段完成。
    """
    kept = [f for f in items if f.is_own or f.confidence >= opts.min_confidence]

    if opts.radius_m is not None and opts.center is not None:
        cx, cy = opts.center
        kept = [
            f for f in kept
            if f.has_position and hypot(f.x - cx, f.y - cy) <= opts.radius_m
        ]

    if opts.center is not None:
        cx, cy = opts.center
        kept.sort(key=lambda f: hypot(f.x - cx, f.y - cy))
    else:
        kept.sort(key=lambda f: (f.side, f.unit_id))

    return kept[: opts.max_formations]


def _describe(info: FormationInfo, at: SimTime, *, own: bool) -> str:
    parts = [info.describe()]

    if info.side:
        parts.append(f"[{info.side}]")

    if info.attached:
        # 配属状态要标出来：否则读的人以为这支部队本来就归当前上级
        parts.append("（配属）")

    if info.has_position:
        parts.append(f"位置 ({info.x / 1000:.1f}, {info.y / 1000:.1f}) km")
        if info.observed > 1:
            parts.append(f"散布 {info.spread_m / 1000:.1f} km")
    else:
        parts.append("位置未知")

    if own and info.declared is not None:
        parts.append(f"编队 {info.declared}")

    # 敌方用"至少观测到"——编制数是未知量，不能把观测数当编制数
    parts.append(
        f"在位 {info.observed}" if own else f"至少观测到 {info.observed} 个目标"
    )

    if info.strength is not None:
        parts.append(f"战力 {info.strength:.0%}")

    if not own:
        parts.append(f"置信度 {info.confidence:.2f}")
        age_s = max(0, at - info.latest_at) // 1_000_000
        if age_s > 0:
            parts.append(f"最后更新 {age_s} 秒前")

    return "  ".join(parts)


__all__ = [
    "BASIS_ADMINISTRATIVE",
    "BASIS_OPERATIONAL",
    "OMNISCIENT",
    "UNIDENTIFIED_CONTACT",
    "UNIDENTIFIED_UNIT",
    "DigestOptions",
    "EntityInfo",
    "FormationInfo",
    "Situation",
    "SituationProjector",
    "digest",
    "format_sim_time",
]
