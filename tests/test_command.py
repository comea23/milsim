"""指挥关系的测试（设计见 §3.10）。

三层各用各的结构，最容易错的是第二层的时间窗重叠——那会让投影把同一个
实体算两次，兵力统计直接翻倍，而且不报错。所以那条校验有专门的测试。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.services import EntityRegistry, EntityStore
from milsim.services.command import (
    CommandChain,
    TimeWindow,
)
from milsim.services.formation import FormationTree
from milsim.services.map import Axial
from milsim.services.situation import (
    BASIS_ADMINISTRATIVE,
    SituationProjector,
    digest,
)
from milsim.services.store import LocalCellRef

CELL = LocalCellRef(1, 0, Axial(0, 0))

MINUTE = 60_000_000
HOUR = 3_600_000_000


def build_chain() -> CommandChain:
    """红方一个旅，下辖两个营，一营下辖两个连。"""
    tree = FormationTree()
    tree.add_unit("RED_BDE", side="red", echelon="旅")
    tree.add_unit("RED_1BN", side="red", echelon="营", parent="RED_BDE")
    tree.add_unit("RED_2BN", side="red", echelon="营", parent="RED_BDE")
    tree.add_unit("RED_1_1", side="red", echelon="连", parent="RED_1BN")
    tree.add_unit("RED_1_2", side="red", echelon="连", parent="RED_1BN")
    tree.add_unit("RED_2_1", side="red", echelon="连", parent="RED_2BN")
    return CommandChain(tree)


# ---------------------------------------------------------------------------
# TimeWindow
# ---------------------------------------------------------------------------
def test_window_open_covers_everything() -> None:
    window = TimeWindow()
    assert window.is_open
    assert window.covers(0)
    assert window.covers(10 ** 12)
    assert window.describe() == "全程"


def test_window_is_half_open() -> None:
    """右开是刻意的：``until 8h`` 与 ``from 8h`` 相接时不该重叠也不该有空隙。"""
    left = TimeWindow(0, HOUR)
    right = TimeWindow(HOUR, None)
    assert left.covers(HOUR - 1)
    assert not left.covers(HOUR)
    assert right.covers(HOUR)
    assert not left.overlaps(right)


def test_window_overlap_detection() -> None:
    assert TimeWindow(0, 2 * HOUR).overlaps(TimeWindow(HOUR, 3 * HOUR))
    assert not TimeWindow(0, HOUR).overlaps(TimeWindow(2 * HOUR, None))
    assert TimeWindow(0, None).overlaps(TimeWindow(0, HOUR))
    assert TimeWindow().overlaps(TimeWindow(5 * HOUR, 6 * HOUR))


def test_window_describe_is_human_readable() -> None:
    assert TimeWindow(2 * HOUR).describe() == "2 h 起"
    assert TimeWindow(0, 30 * MINUTE).describe() == "30 min ~ 0 h" or \
        "30 min" in TimeWindow(0, 30 * MINUTE).describe()


# ---------------------------------------------------------------------------
# 第一层：行政隶属不受配属影响
# ---------------------------------------------------------------------------
def test_attachment_changes_operational_not_administrative() -> None:
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN")

    # 行政隶属：永远在一营下面
    assert chain.formations.parent_of(chain.formations.id_of("RED_1_1")) == \
        chain.formations.id_of("RED_1BN")

    # 作战归属：改接到二营
    unit = chain.formations.id_of("RED_1_1")
    assert chain.parent_of(unit, 0) == chain.formations.id_of("RED_2BN")
    assert chain.is_attached(unit, 0)


def test_attachment_window_reverts_on_expiry() -> None:
    """``until 8 h`` 到期自动归建——不用再写一条改回来的配属。"""
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN", window=TimeWindow(30 * MINUTE, 8 * HOUR))

    unit = chain.formations.id_of("RED_1_1")
    one = chain.formations.id_of("RED_1BN")
    two = chain.formations.id_of("RED_2BN")

    assert chain.parent_of(unit, 0) == one                   # 配属未生效
    assert chain.parent_of(unit, 30 * MINUTE) == two         # 生效
    assert chain.parent_of(unit, 8 * HOUR - 1) == two
    assert chain.parent_of(unit, 8 * HOUR) == one            # 归建
    assert not chain.is_attached(unit, 8 * HOUR)


def test_chain_reflects_attachment() -> None:
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN")

    unit = chain.formations.id_of("RED_1_1")
    names = [chain.formations.get(u).name for u in chain.chain_of(unit, 0)]
    assert names == ["RED_BDE", "RED_2BN", "RED_1_1"]


def test_subtree_includes_attached_unit() -> None:
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN")

    two = chain.formations.id_of("RED_2BN")
    subtree = {chain.formations.get(u).name for u in chain.subtree_of(two, 0)}
    assert subtree == {"RED_2BN", "RED_2_1", "RED_1_1"}

    # 一营少了一个连
    one = chain.formations.id_of("RED_1BN")
    subtree = {chain.formations.get(u).name for u in chain.subtree_of(one, 0)}
    assert subtree == {"RED_1BN", "RED_1_2"}


def test_attach_of_returns_active_record() -> None:
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN", window=TimeWindow(0, HOUR), note="加强")
    unit = chain.formations.id_of("RED_1_1")

    record = chain.attach_of(unit, MINUTE)
    assert record is not None and record.note == "加强"
    assert chain.attach_of(unit, 2 * HOUR) is None


# ---------------------------------------------------------------------------
# 第二层的硬校验
# ---------------------------------------------------------------------------
def test_overlapping_attachments_rejected() -> None:
    """**本文件最重要的测试。**

    同一单位同一时刻不能有两个作战上级。漏了这条，投影会把同一个实体
    算两次，兵力统计直接翻倍——而且不报错。
    """
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN", window=TimeWindow(HOUR, 3 * HOUR))
    chain.attach("RED_1_1", "RED_BDE", window=TimeWindow(2 * HOUR, 4 * HOUR))

    problems = chain.validate()
    assert any("重叠" in p for p in problems)
    assert any("同一时刻只能有一个作战上级" in p for p in problems)


def test_adjacent_attachments_do_not_conflict() -> None:
    """首尾相接不算重叠——右开区间就是为了这个。"""
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN", window=TimeWindow(0, HOUR))
    chain.attach("RED_1_1", "RED_BDE", window=TimeWindow(HOUR, 2 * HOUR))
    assert chain.validate() == []


def test_attach_to_self_rejected() -> None:
    chain = build_chain()
    with pytest.raises(ConfigurationError, match="自己"):
        chain.attach("RED_1_1", "RED_1_1")


def test_attach_to_own_subordinate_rejected() -> None:
    chain = build_chain()
    with pytest.raises(ConfigurationError, match="自己的下级"):
        chain.attach("RED_1BN", "RED_1_1")


def test_cross_side_attachment_rejected() -> None:
    chain = build_chain()
    chain.formations.add_unit("BLUE_1BN", side="blue", echelon="营")
    with pytest.raises(ConfigurationError, match="跨阵营"):
        chain.attach("RED_1_1", "BLUE_1BN")


def test_unknown_unit_gives_spelling_hint() -> None:
    chain = build_chain()
    # 拼错一个字符就要带上"是否想写 X？"，并指出 X 是哪个
    with pytest.raises(ConfigurationError, match="是否想写 'RED_1_2'"):
        chain.attach("RED_1_l", "RED_2BN")
    with pytest.raises(ConfigurationError, match="未定义的编队"):
        chain.attach("完全不存在的名字", "RED_2BN")


def test_cycle_detection() -> None:
    """配属链成环时 validate 必须发现。"""
    chain = build_chain()
    # 一营配属到二营，二营又配属到一营 → 环
    chain.attach("RED_1BN", "RED_2BN")
    chain.attach("RED_2BN", "RED_1BN")

    problems = chain.validate(at=0)
    assert any("成环" in p for p in problems)


# ---------------------------------------------------------------------------
# 第三层：指令通道
# ---------------------------------------------------------------------------
def test_direct_channel_allows_skip_level() -> None:
    chain = build_chain()
    chain.direct("RED_BDE", "RED_1_1")

    bde = chain.formations.id_of("RED_BDE")
    unit = chain.formations.id_of("RED_1_1")

    # 不走直接通道也本来就下得去（旅在连的归属链上）
    assert chain.can_issue_direct(bde, unit, 0)

    # 建通道后能查到这条越级关系
    assert bde in chain.direct_superiors_of(unit, 0)
    assert unit in chain.direct_subordinates_of(bde, 0)


def test_direct_channel_enables_cross_branch_command() -> None:
    """旅直接指挥二营下的连——走归属链本来就通；跨分支才需要通道。"""
    chain = build_chain()
    one_bn = chain.formations.id_of("RED_1BN")
    other = chain.formations.id_of("RED_2_1")

    # 一营本来管不到二营的连
    assert not chain.can_issue_direct(one_bn, other, 0)

    chain.direct("RED_1BN", "RED_2_1")
    assert chain.can_issue_direct(one_bn, other, 0)


def test_direct_channel_does_not_change_operational_parent() -> None:
    """**这是第三层之所以是图的原因**：越级不改归属。"""
    chain = build_chain()
    chain.direct("RED_BDE", "RED_1_1")
    unit = chain.formations.id_of("RED_1_1")
    assert chain.parent_of(unit, 0) == chain.formations.id_of("RED_1BN")
    assert not chain.is_attached(unit, 0)


def test_direct_channel_window() -> None:
    chain = build_chain()
    bde = chain.formations.id_of("RED_BDE")
    unit = chain.formations.id_of("RED_1_1")
    chain.direct("RED_BDE", "RED_2_1", window=TimeWindow(0, HOUR))

    assert bde in chain.direct_superiors_of(chain.formations.id_of("RED_2_1"), 0)
    assert bde not in chain.direct_superiors_of(
        chain.formations.id_of("RED_2_1"), 2 * HOUR
    )
    assert unit not in chain.direct_subordinates_of(bde, 0)


def test_redundant_direct_channel_is_warned_not_fatal() -> None:
    """已经在归属链上的直接通道是冗余——警告而非错误。"""
    chain = build_chain()
    chain.direct("RED_BDE", "RED_1_1")
    problems = chain.validate()
    assert any("冗余" in p for p in problems)


def test_direct_cross_side_rejected() -> None:
    chain = build_chain()
    chain.formations.add_unit("BLUE_1BN", side="blue")
    with pytest.raises(ConfigurationError, match="跨阵营"):
        chain.direct("RED_BDE", "BLUE_1BN")


# ---------------------------------------------------------------------------
# 协同
# ---------------------------------------------------------------------------
def test_coordination_is_symmetric() -> None:
    chain = build_chain()
    chain.coordinate("RED_1BN", "RED_2BN")
    one = chain.formations.id_of("RED_1BN")
    two = chain.formations.id_of("RED_2BN")

    assert two in chain.coordinators_of(one, 0)
    assert one in chain.coordinators_of(two, 0)
    # 协同不产生指挥方向
    assert not chain.can_issue_direct(one, two, 0)


def test_coordination_does_not_affect_operational_tree() -> None:
    chain = build_chain()
    chain.coordinate("RED_1BN", "RED_2BN")
    one = chain.formations.id_of("RED_1_1")
    assert chain.parent_of(one, 0) == chain.formations.id_of("RED_1BN")


# ---------------------------------------------------------------------------
# 确定性
# ---------------------------------------------------------------------------
def test_registration_order_does_not_matter() -> None:
    """同种子的两次推演必须逐项一致——配属的登记顺序不能影响结果。"""
    chain = build_chain()
    chain.attach("RED_1_1", "RED_2BN")
    chain.attach("RED_2_1", "RED_1BN")

    unit = chain.formations.id_of("RED_1_1")
    before = chain.chain_of(unit, 0)
    map_before = chain.operational_map(0)

    # 重新登记一遍（换个顺序），结果必须一样
    reversed_chain = build_chain()
    reversed_chain.attach("RED_2_1", "RED_1BN")
    reversed_chain.attach("RED_1_1", "RED_2BN")

    assert reversed_chain.chain_of(unit, 0) == before
    assert reversed_chain.operational_map(0) == map_before


# ---------------------------------------------------------------------------
# 与态势投影的衔接
# ---------------------------------------------------------------------------
def _small_world():
    """红方一个营（3 连），二连配属给另一个营。"""
    registry = EntityRegistry()
    store = EntityStore()
    tree = FormationTree()

    tree.add_unit("RED_BDE", side="red", echelon="旅")
    tree.add_unit("RED_1BN", side="red", echelon="营", parent="RED_BDE")
    tree.add_unit("RED_2BN", side="red", echelon="营", parent="RED_BDE")
    tree.add_unit("RED_1_1", side="red", echelon="连", parent="RED_1BN")
    tree.add_unit("RED_1_2", side="red", echelon="连", parent="RED_1BN")

    class _E:
        __slots__ = ("entity_id", "name")

        def __init__(self) -> None:
            self.entity_id = -1
            self.name = ""

    for index, name in enumerate(("RED_1_1_A", "RED_1_2_A"), start=1):
        entity_id = registry.register(_E(), name=name, side="red", type_name="TANK")
        store.add(entity_id, CELL)
        store.set_pose(entity_id, index * 1000.0, 0.0, 0.0, 90.0, 10.0)
        tree.add_member(f"RED_1_{index}", entity_id)

    chains = CommandChain(tree)
    projector = SituationProjector(registry, store, tree, chains=chains)
    return chains, projector


def test_projection_uses_operational_parent_by_default() -> None:
    chains, projector = _small_world()
    chains.attach("RED_1_2", "RED_2BN")

    red = projector.project(side="red", at=0)
    moved = next(f for f in red.formations if f.name == "RED_1_2")
    assert moved.parent_id == chains.formations.id_of("RED_2BN")
    assert moved.attached


def test_projection_administrative_basis_ignores_attachment() -> None:
    chains, projector = _small_world()
    chains.attach("RED_1_2", "RED_2BN")

    admin = projector.project(side="red", at=0, basis=BASIS_ADMINISTRATIVE)
    moved = next(f for f in admin.formations if f.name == "RED_1_2")
    assert moved.parent_id == chains.formations.id_of("RED_1BN")
    assert not moved.attached


def test_projection_basis_is_time_aware() -> None:
    """回溯：配属生效前后，态势里的上级不同。"""
    chains, projector = _small_world()
    chains.attach("RED_1_2", "RED_2BN", window=TimeWindow(HOUR, 2 * HOUR))

    def parent_at(moment: int) -> int | None:
        situation = projector.project(side="red", at=moment)
        return next(f for f in situation.formations if f.name == "RED_1_2").parent_id

    one = chains.formations.id_of("RED_1BN")
    two = chains.formations.id_of("RED_2BN")
    assert parent_at(0) == one
    assert parent_at(90 * MINUTE) == two
    assert parent_at(3 * HOUR) == one


def test_projection_rejects_unknown_basis() -> None:
    _, projector = _small_world()
    with pytest.raises(ValueError, match="未知的聚合依据"):
        projector.project(side="red", at=0, basis="nonsense")


def test_digest_marks_attached_units() -> None:
    chains, projector = _small_world()
    chains.attach("RED_1_2", "RED_2BN")

    text = digest(projector.project(side="red", at=0))
    assert "（配属）" in text
    assert text.count("（配属）") == 1


def test_no_chain_means_administrative_only() -> None:
    """没有指挥关系时投影照常工作，等价于纯行政隶属。"""
    chains, _ = _small_world()
    registry = EntityRegistry()
    store = EntityStore()
    tree = FormationTree()
    tree.add_unit("RED_1BN", side="red", echelon="营")
    projector = SituationProjector(registry, store, tree)
    assert projector.chains is None
    assert projector.project(side="red", at=0).formations == ()
