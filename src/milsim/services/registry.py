"""实体注册表：身份管理。

职责边界
--------
| 职责 | **不**职责 |
|---|---|
| ID 分配与回收 | 实体行为 |
| 名称 / 阵营 / 类型 / 标签索引 | 仿真逻辑 |
| 存活状态维护 | **状态数据**（位置、损耗等归 EntityStore） |

和 ``EntityStore`` 的分工是按**变更频率**切的：身份只在创建和销毁时变，
状态每帧都在变。混在一起的话，每帧几万次位置写入会不断扰动阵营索引的
内部结构，两边都做不好。

为什么不放引擎
--------------
"注册表不该是引擎的工作吗"——结论是留在服务层，理由是**引擎用不到它**。
引擎只负责事件排序与时间推进，事件回调里已经带了需要的一切；
放进去会让引擎的职责变成"时间 + 实体"两件事，想单独测时间推进
就得先绕开实体。

AFSIM 的 ``WsfSimulation`` 确实持有平台表，但它是**仿真器**不是**引擎**——
它管时间是因为它同时管平台，这两件事在那边是同一个对象的职责。
本项目的类比对象是"``WsfSimulation`` 减去平台管理那一半"。

分层桥接
--------
注册表用 :class:`RegisteredEntity` 协议声明它对实体的全部要求，
因此 ``services`` 不需要 import ``models``。依赖方向仍是单向的
（models → services），这不是绕过分层，是依赖倒置的标准形态。
"""

from __future__ import annotations

from bisect import insort
from typing import Iterator, Protocol, Sequence, runtime_checkable

from ..errors import ConfigurationError
from .random import SequenceGenerator


@runtime_checkable
class RegisteredEntity(Protocol):
    """注册表对实体的全部要求。

    只要这两个属性**可写**，任何对象都能被登记——包括测试用的三行 stub。
    实体类的 ``__slots__`` 里必须包含它们，否则注册时赋值会失败。
    """

    entity_id: int
    name: str


def _attach(index: dict[str, list[int]], key: str, entity_id: int) -> None:
    """把 ID 插入索引桶，保持升序。

    ID 单调分配，所以追加是常态（O(1)）；乱序插入才走 ``insort``。
    保持有序是为了遍历结果可复现——顺序不确定会让同种子的两次推演分叉。
    """
    bucket = index.get(key)
    if bucket is None:
        index[key] = [entity_id]
    elif not bucket or entity_id > bucket[-1]:
        bucket.append(entity_id)
    else:
        insort(bucket, entity_id)


def _detach(index: dict[str, list[int]], key: str, entity_id: int) -> None:
    bucket = index.get(key)
    if bucket is None:
        return
    try:
        bucket.remove(entity_id)
    except ValueError:
        return
    if not bucket:
        del index[key]


class EntityRegistry:
    """实体身份表。线程不安全——只在引擎主线程使用。

    用法::

        registry = EntityRegistry()
        eid = registry.register(unit, name="SAM_1", side="red",
                                type_name="SAM_BATTALION", tags=("sam",))
        registry.by_side("red")        # → [eid, ...]
    """

    __slots__ = (
        "_entities",
        "_meta",
        "_by_name",
        "_by_side",
        "_by_type",
        "_by_tag",
        "_ids",
        "_ids_gen",
        "unregister_count",
    )

    def __init__(self, id_start: int = 0) -> None:
        self._entities: dict[int, RegisteredEntity] = {}
        #: entity_id → (side, type_name, tags)
        #:
        #: 自带一份元数据是刻意的：注销时要**精确**从各个索引里摘掉这个实体。
        #: 否则只能遍历所有桶去找它，注销代价从 O(1) 退化成 O(实体总数)——
        #: 而导弹这类实体恰恰是大量生灭的。
        self._meta: dict[int, tuple[str, str, tuple[str, ...]]] = {}
        self._by_name: dict[str, int] = {}
        self._by_side: dict[str, list[int]] = {}
        self._by_type: dict[str, list[int]] = {}
        self._by_tag: dict[str, list[int]] = {}
        self._ids: list[int] = []
        self._ids_gen = SequenceGenerator(id_start, prefix="entity")
        self.unregister_count = 0

    # -- 登记与注销 --------------------------------------------------------

    def register(
        self,
        entity: RegisteredEntity,
        *,
        name: str,
        side: str = "",
        type_name: str = "",
        tags: Sequence[str] = (),
    ) -> int:
        """登记一个实体，分配 ID 并写回 ``entity.entity_id`` / ``entity.name``。

        重名直接报错。**不做"自动改名"**——静默改名会让想定里引用的名字
        和实际对象对不上，这类 bug 查起来极其费劲。
        """
        if name in self._by_name:
            # 重名几乎总是想定里写了两个同名平台。归为配置错误，
            # 让工具脚本能统一呈现，而不是抛个裸 ValueError
            raise ConfigurationError(
                f"实体名 {name!r} 已被占用（id={self._by_name[name]}）"
            )

        entity_id = self._ids_gen.next()
        setattr(entity, "entity_id", entity_id)
        setattr(entity, "name", name)

        # tags 去重后排序：传入顺序不定（比如从 set 来）时结果仍然确定
        tag_tuple = tuple(sorted(set(tags)))

        self._entities[entity_id] = entity
        self._meta[entity_id] = (side, type_name, tag_tuple)
        self._by_name[name] = entity_id
        self._ids.append(entity_id)          # 递增分配，天然有序

        _attach(self._by_side, side, entity_id)
        _attach(self._by_type, type_name, entity_id)
        for tag in tag_tuple:
            _attach(self._by_tag, tag, entity_id)

        return entity_id

    def unregister(self, entity_id: int) -> bool:
        """注销实体，从**所有**索引里摘掉。返回是否找到。

        漏摘任何一个索引都会留下"幽灵实体"——它已经不在表里了，
        却仍然出现在 ``by_side("red")`` 的结果中。
        """
        entity = self._entities.pop(entity_id, None)
        if entity is None:
            return False

        meta = self._meta.pop(entity_id, None)
        name = getattr(entity, "name", None)
        if name is not None:
            self._by_name.pop(name, None)

        if meta is not None:
            side, type_name, tag_tuple = meta
            _detach(self._by_side, side, entity_id)
            _detach(self._by_type, type_name, entity_id)
            for tag in tag_tuple:
                _detach(self._by_tag, tag, entity_id)

        try:
            self._ids.remove(entity_id)
        except ValueError:
            pass

        self.unregister_count += 1
        return True

    def clear(self) -> None:
        self._entities.clear()
        self._meta.clear()
        self._by_name.clear()
        self._by_side.clear()
        self._by_type.clear()
        self._by_tag.clear()
        self._ids.clear()

    # -- 查询 --------------------------------------------------------------

    def get(self, entity_id: int) -> RegisteredEntity | None:
        return self._entities.get(entity_id)

    def by_name(self, name: str) -> RegisteredEntity | None:
        entity_id = self._by_name.get(name)
        return None if entity_id is None else self._entities[entity_id]

    def by_side(self, side: str) -> list[int]:
        """返回副本——调用方拿到之后可以随便改，不会影响内部索引。"""
        return list(self._by_side.get(side, ()))

    def by_type(self, type_name: str) -> list[int]:
        return list(self._by_type.get(type_name, ()))

    def by_tag(self, tag: str) -> list[int]:
        return list(self._by_tag.get(tag, ()))

    def all_ids(self) -> list[int]:
        """全部实体 ID，升序。"""
        return list(self._ids)

    def side_of(self, entity_id: int) -> str:
        meta = self._meta.get(entity_id)
        return meta[0] if meta else ""

    def type_of(self, entity_id: int) -> str:
        meta = self._meta.get(entity_id)
        return meta[1] if meta else ""

    def tags_of(self, entity_id: int) -> tuple[str, ...]:
        meta = self._meta.get(entity_id)
        return meta[2] if meta else ()

    def sides(self) -> list[str]:
        """出现过的所有阵营，**按字母序**以保证可复现。"""
        return sorted(self._by_side)

    def types(self) -> list[str]:
        return sorted(self._by_type)

    def tags(self) -> list[str]:
        return sorted(self._by_tag)

    def has(self, entity_id: int) -> bool:
        return entity_id in self._entities

    # -- 诊断 --------------------------------------------------------------

    def statistics(self) -> dict[str, int]:
        return {
            "entities": len(self._entities),
            "sides": len(self._by_side),
            "types": len(self._by_type),
            "tags": len(self._by_tag),
            "registered_names": len(self._by_name),
            "unregistered": self.unregister_count,
        }

    def validate(self) -> list[str]:
        """自检：各个索引之间必须完全一致。测试和调试用。"""
        problems: list[str] = []
        ids = set(self._ids)

        if ids != set(self._entities):
            problems.append("_ids 与 _entities 不一致")
        if len(self._ids) != len(self._entities):
            problems.append("_ids 中有重复")

        for entity_id, (side, type_name, tag_tuple) in self._meta.items():
            if entity_id not in ids:
                problems.append(f"元数据残留：id={entity_id}")
            if entity_id not in self._by_side.get(side, ()):
                problems.append(f"阵营索引缺失：id={entity_id} side={side}")
            if entity_id not in self._by_type.get(type_name, ()):
                problems.append(f"类型索引缺失：id={entity_id} type={type_name}")
            for tag in tag_tuple:
                if entity_id not in self._by_tag.get(tag, ()):
                    problems.append(f"标签索引缺失：id={entity_id} tag={tag}")

        for index_name, index in (
            ("阵营", self._by_side),
            ("类型", self._by_type),
            ("标签", self._by_tag),
        ):
            for key, bucket in index.items():
                if not bucket:
                    problems.append(f"{index_name}索引有空桶：{key!r}")
                if bucket != sorted(bucket):
                    problems.append(f"{index_name}索引未排序：{key!r}")
                for entity_id in bucket:
                    if entity_id not in ids:
                        problems.append(f"{index_name}索引有幽灵 ID：{entity_id}")

        return problems

    # -- 协议 --------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._entities)

    def __contains__(self, entity_id: int) -> bool:
        return entity_id in self._entities

    def __iter__(self) -> Iterator[int]:
        """遍历实体 ID，升序。"""
        return iter(self._ids)

    def entities(self) -> Iterator[RegisteredEntity]:
        """按 ID 升序遍历实体对象。"""
        return (self._entities[i] for i in self._ids)

    def __repr__(self) -> str:
        return (
            f"<EntityRegistry {len(self._entities)} 实体 / "
            f"{len(self._by_side)} 阵营 / {len(self._by_type)} 类型>"
        )
