"""态势投影与编队树的测试。

本文件里最重要的是 ``test_enemy_centroid_uses_only_detected_members``——
它守的是"先过滤、再聚合"这个顺序。顺序反了不会报错，只会让宏观智能体
凭空知道敌方主力在哪，所以必须有测试把它钉死。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.services import EntityStore, EntityRegistry
from milsim.services.formation import UNASSIGNED_ID, FormationTree
from milsim.services.map import Axial
from milsim.services.situation import (
    DigestOptions,
    SituationProjector,
    digest,
    format_sim_time,
)
from milsim.services.store import LocalCellRef


class StubEntity:
    """注册表只要求 ``entity_id`` 与 ``name`` 可写。"""

    __slots__ = ("entity_id", "name")

    def __init__(self) -> None:
        self.entity_id = -1
        self.name = ""


CELL = LocalCellRef(1, 0, Axial(0, 0))

#: 红方三个连的真实 x 坐标 → 真实质心 3000
RED_X = (0.0, 3000.0, 6000.0)
#: 蓝方三个连的真实 x 坐标 → 真实质心 56000
BLUE_X = (50000.0, 56000.0, 62000.0)


def build_world():
    """红方一个营（3 连）+ 蓝方一个营（3 连），无任何航迹。"""
    registry = EntityRegistry()
    store = EntityStore()
    tree = FormationTree()

    tree.add_unit("RED_BDE", side="red", echelon="旅")
    tree.add_unit("RED_1BN", side="red", echelon="营", parent="RED_BDE")
    tree.add_unit("BLUE_1BN", side="blue", echelon="营")

    ids: dict[str, int] = {}

    for i, x in enumerate(RED_X, start=1):
        name = f"RED_1_{i}"
        eid = registry.register(
            StubEntity(), name=name, side="red", type_name="TANK"
        )
        store.add(eid, CELL)
        store.set_pose(eid, x, 0.0, 0.0, 90.0, 10.0)
        tree.add_member("RED_1BN", eid)
        ids[name] = eid

    for i, x in enumerate(BLUE_X, start=1):
        name = f"BLUE_1_{i}"
        eid = registry.register(
            StubEntity(), name=name, side="blue", type_name="TANK"
        )
        store.add(eid, CELL)
        store.set_pose(eid, x, 0.0, 0.0, 270.0, 10.0)
        tree.add_member("BLUE_1BN", eid)
        ids[name] = eid

    projector = SituationProjector(registry, store, tree)
    return registry, store, tree, projector, ids


# ---------------------------------------------------------------------------
# FormationTree
# ---------------------------------------------------------------------------
def test_tree_builds_hierarchy() -> None:
    _, _, tree, _, _ = build_world()
    assert len(tree) == 3
    assert tree.parent_of(tree.id_of("RED_1BN")) == tree.id_of("RED_BDE")
    assert tree.children_of(tree.id_of("RED_BDE")) == (tree.id_of("RED_1BN"),)
    assert tree.depth_of(tree.id_of("RED_BDE")) == 0
    assert tree.depth_of(tree.id_of("RED_1BN")) == 1
    assert set(tree.sides()) == {"red", "blue"}


def test_tree_collects_entities_recursively() -> None:
    _, _, tree, _, ids = build_world()
    below = tree.all_entities_below(tree.id_of("RED_BDE"))
    assert set(below) == {ids[f"RED_1_{i}"] for i in (1, 2, 3)}
    # BDE 自己没有直属实体，成员全在下一级
    assert tree.members_of(tree.id_of("RED_BDE")) == ()
    assert len(tree.members_of(tree.id_of("RED_1BN"))) == 3


def test_tree_rejects_duplicate_unit_name() -> None:
    tree = FormationTree()
    tree.add_unit("A", side="red")
    with pytest.raises(ConfigurationError, match="已存在"):
        tree.add_unit("A", side="red")


def test_tree_rejects_cross_side_parent() -> None:
    tree = FormationTree()
    tree.add_unit("RED", side="red")
    with pytest.raises(ConfigurationError, match="阵营"):
        tree.add_unit("BLUE", side="blue", parent="RED")


def test_tree_rejects_entity_in_two_units() -> None:
    tree = FormationTree()
    tree.add_unit("A", side="red")
    tree.add_unit("B", side="red")
    tree.add_member("A", 7)
    with pytest.raises(ConfigurationError, match="已编入"):
        tree.add_member("B", 7)


def test_tree_set_parent_detects_cycle() -> None:
    """A 是 B 的上级，再把 A 挂到 B 下就会成环。"""
    tree = FormationTree()
    tree.add_unit("A", side="red")
    tree.add_unit("B", side="red", parent="A")
    tree.add_unit("C", side="red", parent="B")

    with pytest.raises(ConfigurationError, match="成环"):
        tree.set_parent("A", "C")


def test_tree_set_parent_rejects_self() -> None:
    tree = FormationTree()
    tree.add_unit("A", side="red")
    with pytest.raises(ConfigurationError, match="自己"):
        tree.set_parent("A", "A")


def test_tree_set_parent_works_for_transfer() -> None:
    """转隶：从 B 挪到 C，父子索引两侧都要更新。"""
    tree = FormationTree()
    tree.add_unit("A", side="red")
    tree.add_unit("B", side="red", parent="A")
    tree.add_unit("C", side="red", parent="A")

    tree.set_parent("B", "C")
    assert tree.parent_of(tree.id_of("B")) == tree.id_of("C")
    assert tree.children_of(tree.id_of("A")) == (tree.id_of("C"),)
    assert tree.children_of(tree.id_of("C")) == (tree.id_of("B"),)
    assert tree.validate() == []


def test_tree_unknown_name_gives_spelling_hint() -> None:
    _, _, tree, _, _ = build_world()
    with pytest.raises(ConfigurationError, match="是否想写 'RED_1BN'"):
        tree.add_member("RED_1BM", 999)


def test_tree_validate_clean_after_normal_use() -> None:
    _, _, tree, _, _ = build_world()
    assert tree.validate() == []


def test_tree_validate_catches_broken_reverse_index() -> None:
    """人为破坏反向索引，自检必须发现。"""
    _, _, tree, _, _ = build_world()
    entity_id = next(iter(tree.all_entities_below(tree.id_of("RED_1BN"))))
    tree._by_entity[entity_id] = tree.id_of("BLUE_1BN")     # 模拟 bug
    problems = tree.validate()
    assert any("反向索引" in p for p in problems)


# ---------------------------------------------------------------------------
# 投影：上帝视角 vs 按方视角
# ---------------------------------------------------------------------------
def test_god_view_sees_everything_as_truth() -> None:
    _, _, _, projector, _ = build_world()
    situation = projector.project(at=0)

    assert situation.is_omniscient
    assert len(situation.entities) == 6
    assert all(e.from_truth for e in situation.entities)
    assert all(not e.is_own for e in situation.entities)
    # 上帝视角下 own / contacts 都为空，用 by_side 分组
    assert situation.own == ()
    assert situation.contacts == ()
    assert set(situation.by_side) == {"red", "blue"}


def test_side_view_excludes_undetected_enemy() -> None:
    """一个目标都没探测到时，敌方实体**完全不应出现**。"""
    _, _, _, projector, _ = build_world()
    situation = projector.project(side="red", at=0)

    assert len(situation.entities) == 3
    assert {e.side for e in situation.entities} == {"red"}
    assert situation.contacts == ()
    assert {f.side for f in situation.own} == {"red"}


def test_enemy_centroid_uses_only_detected_members() -> None:
    """**本文件最重要的测试**：守"先过滤、再聚合"的顺序。

    红方只探测到蓝方 3 个连中的第 1 个。那么红方视角下 BLUE_1BN 的位置
    必须等于**那一条航迹的位置**，而不是三个连的真实质心。

    如果实现成"先聚合再过滤"，质心会是真实质心 56000 m —— 正好落在
    三个连的中间，看起来是个"很合理的估计"，但红方其实一个目标都没定准。
    """
    registry, store, tree, projector, ids = build_world()
    observer = ids["RED_1_1"]
    watched = ids["BLUE_1_1"]

    # 蓝方三个连真实在 50000 / 56000 / 62000，真实质心 56000
    # 红方只看到第一个，且位置带误差
    store.add_contact(
        observer, watched, quality=0.8, detected_at=1_000_000,
        x=50_100.0, y=50.0, z=0.0, range_m=50_100.0,
    )

    red = projector.project(side="red", at=1_000_000)
    enemy = next(f for f in red.contacts if f.name == "BLUE_1BN")

    assert enemy.observed == 1
    assert enemy.x == pytest.approx(50_100.0)      # 航迹位置
    assert enemy.y == pytest.approx(50.0)
    assert enemy.x != pytest.approx(56_000.0)      # 不是真实质心

    # 上帝视角下同一编队是三个连的真实质心 —— 两个视角必须不同
    god = projector.project(at=1_000_000)
    truth = next(f for f in god.formations if f.name == "BLUE_1BN")
    assert truth.observed == 3
    assert truth.x == pytest.approx(56_000.0)
    assert truth.x != pytest.approx(enemy.x)


def test_enemy_declared_and_strength_are_unknown() -> None:
    """敌方编制数与战损**不可知**——不能拿编队树的真实值顶替。"""
    _, store, _, projector, ids = build_world()
    store.add_contact(
        ids["RED_1_1"], ids["BLUE_1_1"], quality=0.9, detected_at=0,
        x=50_000.0, y=0.0, z=0.0, range_m=50_000.0,
    )

    red = projector.project(side="red", at=0)
    enemy = next(f for f in red.contacts if f.name == "BLUE_1BN")
    assert enemy.declared is None          # 编制数未知（真实是 3）
    assert enemy.strength is None          # 战力未知
    assert enemy.confidence == pytest.approx(0.9)

    own = next(f for f in red.own if f.name == "RED_1BN")
    assert own.declared == 3
    assert own.strength == pytest.approx(1.0)


def test_own_positions_are_exact_and_contacts_are_not() -> None:
    _, store, _, projector, ids = build_world()
    store.add_contact(
        ids["RED_1_1"], ids["BLUE_1_2"], quality=0.6, detected_at=500_000,
        x=55_800.0, y=120.0, z=0.0, range_m=55_800.0,
    )
    red = projector.project(side="red", at=1_000_000)

    own = next(e for e in red.entities if e.name == "RED_1_2")
    assert own.from_truth and own.is_own
    assert own.x == pytest.approx(3000.0)      # 精确
    assert own.age_us == 0
    assert own.heading_deg == pytest.approx(90.0)

    contact = next(e for e in red.entities if e.name == "BLUE_1_2")
    assert not contact.from_truth and not contact.is_own
    assert contact.x == pytest.approx(55_800.0)     # 估计值，真实是 56000
    assert contact.quality == pytest.approx(0.6)
    assert contact.age_us == 500_000                # 信息年龄
    assert contact.strength is None


def test_contact_fusion_prefers_best_quality() -> None:
    """同一目标被两部雷达跟踪时取质量最高的那条，规则要确定。"""
    _, store, _, projector, ids = build_world()
    target = ids["BLUE_1_1"]

    store.add_contact(ids["RED_1_1"], target, quality=0.5, detected_at=100,
                      x=1.0, y=1.0, z=0.0, range_m=1.0)
    store.add_contact(ids["RED_1_2"], target, quality=0.9, detected_at=200,
                      x=2.0, y=2.0, z=0.0, range_m=2.0)

    red = projector.project(side="red", at=200)
    contact = next(e for e in red.entities if e.entity_id == target)
    assert contact.quality == pytest.approx(0.9)
    assert contact.x == pytest.approx(2.0)


def test_friendly_tracks_are_not_treated_as_enemy() -> None:
    """雷达误报出的友军航迹不该出现在"敌方动向"里。"""
    _, store, _, projector, ids = build_world()
    store.add_contact(
        ids["RED_1_1"], ids["RED_1_2"], quality=1.0, detected_at=0,
        x=3000.0, y=0.0, z=0.0, range_m=3000.0,
    )
    red = projector.project(side="red", at=0)
    assert red.contacts == ()
    assert len(red.entities) == 3


def test_unassigned_entities_group_per_side() -> None:
    """未编入编队的实体单独成组，且红蓝不能合并。"""
    registry, store, tree, projector, _ = build_world()
    loose_red = registry.register(StubEntity(), name="RED_LOOSE", side="red")
    store.add(loose_red, CELL)
    store.set_pose(loose_red, 900.0, 0.0, 0.0, 0.0, 0.0)

    god = projector.project(at=0)
    loose = [f for f in god.formations if f.unit_id == UNASSIGNED_ID]
    assert len(loose) == 1 and loose[0].side == "red" and loose[0].observed == 1

    # 蓝方也加一个，应该成为**另一个**伪编队
    loose_blue = registry.register(StubEntity(), name="BLUE_LOOSE", side="blue")
    store.add(loose_blue, CELL)
    store.set_pose(loose_blue, 99_000.0, 0.0, 0.0, 0.0, 0.0)

    god = projector.project(at=0)
    loose = [f for f in god.formations if f.unit_id == UNASSIGNED_ID]
    assert len(loose) == 2
    assert {f.side for f in loose} == {"red", "blue"}


def test_projection_is_pure_and_repeatable() -> None:
    """投影不改状态，两次结果完全一致。"""
    registry, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.7,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    before = store.statistics()
    first = projector.project(side="red", at=0)
    second = projector.project(side="red", at=0)
    assert first == second
    assert store.statistics() == before

    # 两个视角可以同时存在且互不影响
    god = projector.project(at=0)
    assert len(god.entities) == 6 and len(first.entities) == 4


def test_projection_can_be_rewound() -> None:
    """回溯：同一份数据在 T=0 与 T=600s 得到不同态势。

    这是复盘的核心问题——"红方当时以为蓝方在哪"。
    """
    _, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.8,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    early = projector.project(side="red", at=0)
    late = projector.project(side="red", at=600_000_000)

    early_contact = next(e for e in early.entities if not e.from_truth)
    late_contact = next(e for e in late.entities if not e.from_truth)
    assert early_contact.age_us == 0
    assert late_contact.age_us == 600_000_000


def test_known_ids_is_the_visibility_boundary() -> None:
    _, store, _, projector, ids = build_world()
    assert len(projector.known_ids("red", at=0)) == 3

    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.5,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)
    known = projector.known_ids("red", at=0)
    assert len(known) == 4
    assert ids["BLUE_1_1"] in known
    assert ids["BLUE_1_2"] not in known


# ---------------------------------------------------------------------------
# 摘要器
# ---------------------------------------------------------------------------
def test_digest_text_for_side_view() -> None:
    _, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.8,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    text = digest(projector.project(side="red", at=0))
    assert "【red态势】" in text
    assert "本方编队" in text
    assert "敌方动向 · 已探测" in text
    assert "RED_1BN" in text
    # **敌方番号必须脱敏**：红方不该知道当面之敌叫 BLUE_1BN、是个营
    assert "BLUE_1BN" not in text
    assert "未识别编队#1" in text
    # 敌方用"至少观测到"，不暴露编制数
    assert "至少观测到 1 个目标" in text
    # 蓝方真实编制数是 3，绝不能作为"编队 3"出现在敌方段落里
    assert "编队 3" not in text.split("敌方动向")[1]


def test_digest_never_leaks_enemy_strength() -> None:
    """摘要文本里绝不能出现敌方的战力百分比。"""
    _, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.8,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    text = digest(projector.project(side="red", at=0))
    enemy_section = text.split("敌方动向")[1]
    assert "战力" not in enemy_section


def test_digest_god_view_lists_both_sides() -> None:
    _, _, _, projector, _ = build_world()
    text = digest(projector.project(at=0))
    assert "【上帝视角态势】" in text
    assert "red（" in text and "blue（" in text
    # 上帝视角下一切是真实数据，战力可以出现
    assert "战力 100%" in text


def test_digest_respects_radius_clip() -> None:
    """视野裁剪只做 scope，不改变"能知道什么"。"""
    _, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.8,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    red = projector.project(side="red", at=0)
    # 以原点为中心、半径 10 km —— 敌方在 50 km 外，应被裁掉
    clipped = digest(red, DigestOptions(center=(0.0, 0.0), radius_m=10_000.0))
    assert "未识别编队" not in clipped
    assert "RED_1BN" in clipped

    wide = digest(red, DigestOptions(center=(0.0, 0.0), radius_m=100_000.0))
    assert "未识别编队#1" in wide


def test_digest_min_confidence_filters_weak_contacts() -> None:
    _, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.2,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    red = projector.project(side="red", at=0)
    assert "未识别编队" in digest(red)
    assert "未识别编队" not in digest(red, DigestOptions(min_confidence=0.5))


def test_digest_can_hide_sections() -> None:
    _, _, _, projector, _ = build_world()
    red = projector.project(side="red", at=0)
    text = digest(red, DigestOptions(include_contacts=False))
    assert "敌方动向" not in text
    text = digest(red, DigestOptions(include_own=False))
    assert "本方编队" not in text


def test_digest_truncates_within_budget() -> None:
    _, _, _, projector, _ = build_world()
    text = digest(projector.project(at=0), DigestOptions(max_chars=120))
    assert len(text) <= 120
    assert "已截断" in text


# ---------------------------------------------------------------------------
# 结构化输出与前端消费者
# ---------------------------------------------------------------------------
def test_to_dict_matches_digest_source() -> None:
    """结构给前端、文本给大模型，两者必须同源——这是"显示的和算的一致"。"""
    _, store, _, projector, ids = build_world()
    store.add_contact(ids["RED_1_1"], ids["BLUE_1_1"], quality=0.8,
                      detected_at=0, x=50_000.0, y=0.0, z=0.0, range_m=50_000.0)

    situation = projector.project(side="red", at=0)
    payload = situation.to_dict()

    assert payload["side"] == "red"
    assert payload["at_text"] == format_sim_time(0)
    enemy = next(f for f in payload["formations"] if f["name"] == "BLUE_1BN")
    assert enemy["declared"] is None
    assert enemy["strength"] is None
    assert enemy["from_truth"] is False
    assert enemy["x"] == pytest.approx(50_000.0)


def test_format_sim_time() -> None:
    assert format_sim_time(0) == "T+00:00:00"
    assert format_sim_time(1_000_000) == "T+00:00:01"
    assert format_sim_time(3661_000_000) == "T+01:01:01"
