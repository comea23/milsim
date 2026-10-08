"""实体注册表测试。

重点守住三条约定：ID 不复用、索引有序、注销摘干净。
第三条用 `validate()` 做全量自检——它是发现"幽灵实体"最有效的手段。
"""

from __future__ import annotations

import pytest

from milsim.services.registry import EntityRegistry, RegisteredEntity


class StubEntity:
    """三行 stub 就能满足协议——这正是依赖倒置带来的好处。

    注意 `__slots__` 里必须有 entity_id 和 name，因为注册时要赋值。
    """

    __slots__ = ("entity_id", "name")

    def __init__(self) -> None:
        self.entity_id = -1
        self.name = ""


def make_registry() -> EntityRegistry:
    return EntityRegistry()


# ---------------------------------------------------------------------------
# 基本登记
# ---------------------------------------------------------------------------

def test_register_assigns_id_and_name() -> None:
    registry = make_registry()
    entity = StubEntity()

    eid = registry.register(entity, name="SAM_1", side="red", type_name="SAM")

    assert eid == 0
    assert entity.entity_id == 0
    assert entity.name == "SAM_1"
    assert registry.get(0) is entity
    assert len(registry) == 1


def test_ids_are_monotonic_and_never_reused() -> None:
    """★ 注销后 ID 不能被下一个实体复用。

    复用会让"已销毁实体"的陈旧引用静默指向新实体——表现为某个单位的
    行为莫名其妙受另一个单位影响，而且极难定位。
    """
    registry = make_registry()
    a = registry.register(StubEntity(), name="A")
    b = registry.register(StubEntity(), name="B")
    assert (a, b) == (0, 1)

    registry.unregister(a)

    c = registry.register(StubEntity(), name="C")
    assert c == 2, "ID 被复用了"
    assert c != a


def test_duplicate_name_rejected() -> None:
    """重名报错，不静默改名。"""
    registry = make_registry()
    registry.register(StubEntity(), name="SAM_1")

    with pytest.raises(ValueError, match="已被占用"):
        registry.register(StubEntity(), name="SAM_1")


def test_name_becomes_available_after_unregister() -> None:
    registry = make_registry()
    eid = registry.register(StubEntity(), name="SAM_1")
    registry.unregister(eid)
    assert registry.register(StubEntity(), name="SAM_1") == 1


def test_stub_satisfies_protocol() -> None:
    """结构化类型：不需要显式继承，只要属性对得上。"""
    entity = StubEntity()
    entity.entity_id = 1
    entity.name = "x"
    assert isinstance(entity, RegisteredEntity)


# ---------------------------------------------------------------------------
# 索引
# ---------------------------------------------------------------------------

def test_indexes_are_sorted() -> None:
    """所有索引桶保持升序——遍历顺序影响推演可复现性。"""
    registry = make_registry()
    for i in range(10):
        registry.register(StubEntity(), name=f"R{i}", side="red", type_name="T")

    red = registry.by_side("red")
    assert red == sorted(red)
    assert red == list(range(10))

    t = registry.by_type("T")
    assert t == sorted(t)


def test_index_results_are_copies() -> None:
    """返回副本，调用方改不到内部结构。"""
    registry = make_registry()
    registry.register(StubEntity(), name="A", side="red")

    result = registry.by_side("red")
    result.append(999)
    assert registry.by_side("red") == [0]


def test_unknown_key_returns_empty() -> None:
    registry = make_registry()
    assert registry.by_side("blue") == []
    assert registry.by_type("NOPE") == []
    assert registry.by_tag("nope") == []
    assert registry.by_name("nobody") is None
    assert registry.get(42) is None


def test_tags_are_deduplicated_and_sorted() -> None:
    """标签去重排序——否则从 set 传来的标签会让索引顺序不确定。"""
    registry = make_registry()
    eid = registry.register(
        StubEntity(), name="A", tags=["sam", "radar", "sam", "aaa"]
    )
    assert registry.tags_of(eid) == ("aaa", "radar", "sam")
    assert registry.tags() == ["aaa", "radar", "sam"]


def test_multiple_entities_share_index_bucket() -> None:
    registry = make_registry()
    for i in range(3):
        registry.register(StubEntity(), name=f"U{i}", side="red", type_name="INF")
    assert registry.by_side("red") == [0, 1, 2]
    assert registry.by_type("INF") == [0, 1, 2]


def test_sides_and_types_listing() -> None:
    registry = make_registry()
    registry.register(StubEntity(), name="A", side="red", type_name="SAM")
    registry.register(StubEntity(), name="B", side="blue", type_name="TANK")
    registry.register(StubEntity(), name="C", side="red", type_name="TANK")

    assert registry.sides() == ["blue", "red"]
    assert registry.types() == ["SAM", "TANK"]


# ---------------------------------------------------------------------------
# 注销 —— 幽灵实体的防线
# ---------------------------------------------------------------------------

def test_unregister_removes_from_all_indexes() -> None:
    """★ 注销必须从**所有**索引摘干净，否则留下幽灵实体。"""
    registry = make_registry()
    eid = registry.register(
        StubEntity(), name="A", side="red", type_name="SAM", tags=["sam", "ad"]
    )

    assert registry.unregister(eid) is True
    assert registry.get(eid) is None
    assert registry.by_name("A") is None
    assert registry.by_side("red") == []
    assert registry.by_type("SAM") == []
    assert registry.by_tag("sam") == []
    assert registry.by_tag("ad") == []
    assert eid not in registry
    assert registry.validate() == []


def test_unregister_one_of_many_keeps_others() -> None:
    registry = make_registry()
    ids = [
        registry.register(StubEntity(), name=f"U{i}", side="red", type_name="INF")
        for i in range(4)
    ]

    registry.unregister(ids[1])
    assert registry.by_side("red") == [0, 2, 3]
    assert registry.by_type("INF") == [0, 2, 3]
    assert registry.all_ids() == [0, 2, 3]
    assert registry.validate() == []


def test_unregister_unknown_returns_false() -> None:
    registry = make_registry()
    assert registry.unregister(99) is False


def test_empty_buckets_are_removed() -> None:
    """空桶要删掉，不能让它们一直挂着。"""
    registry = make_registry()
    eid = registry.register(StubEntity(), name="A", side="red", type_name="SAM")
    registry.unregister(eid)
    assert registry.by_side("red") == []
    assert registry.sides() == []
    assert registry.validate() == []


def test_validate_passes_through_many_operations() -> None:
    """密集生灭之后自检仍然干净——模拟推演中大量单位被摧毁的场景。"""
    registry = make_registry()
    created = []
    for i in range(100):
        created.append(
            registry.register(
                StubEntity(),
                name=f"U{i}",
                side="red" if i % 2 else "blue",
                type_name=f"T{i % 5}",
                tags=[f"tag{i % 3}"],
            )
        )

    # 每隔三个销毁一个
    for eid in created[::3]:
        registry.unregister(eid)

    assert registry.validate() == []
    assert len(registry) == 100 - len(created[::3])


# ---------------------------------------------------------------------------
# 遍历与统计
# ---------------------------------------------------------------------------

def test_iteration_is_ascending() -> None:
    registry = make_registry()
    ids = [registry.register(StubEntity(), name=f"U{i}") for i in range(5)]
    registry.unregister(ids[2])

    assert list(registry) == [0, 1, 3, 4]
    assert [e.entity_id for e in registry.entities()] == [0, 1, 3, 4]


def test_all_ids_is_a_copy() -> None:
    registry = make_registry()
    registry.register(StubEntity(), name="A")
    snapshot = registry.all_ids()
    snapshot.append(999)
    assert registry.all_ids() == [0]


def test_statistics() -> None:
    registry = make_registry()
    registry.register(StubEntity(), name="A", side="red", type_name="SAM", tags=["t"])
    registry.register(StubEntity(), name="B", side="blue", type_name="TANK")
    registry.unregister(1)

    stats = registry.statistics()
    assert stats["entities"] == 1
    assert stats["sides"] == 1
    assert stats["registered_names"] == 1
    assert stats["unregistered"] == 1


def test_clear_resets_everything() -> None:
    registry = make_registry()
    registry.register(StubEntity(), name="A", side="red")
    registry.clear()

    assert len(registry) == 0
    assert registry.by_side("red") == []
    assert registry.validate() == []


def test_empty_side_and_type_are_valid_keys() -> None:
    """不指定阵营/类型时用空字符串占位，不该报错也不该变成"查不到"。"""
    registry = make_registry()
    eid = registry.register(StubEntity(), name="NEUTRAL")
    assert registry.side_of(eid) == ""
    assert registry.type_of(eid) == ""
    assert registry.by_side("") == [eid]
    assert registry.validate() == []


def test_id_start_offset() -> None:
    registry = EntityRegistry(id_start=1000)
    assert registry.register(StubEntity(), name="A") == 1000


def test_registry_repr() -> None:
    registry = make_registry()
    registry.register(StubEntity(), name="A", side="red", type_name="SAM")
    text = repr(registry)
    assert "1 实体" in text
    assert "1 阵营" in text
