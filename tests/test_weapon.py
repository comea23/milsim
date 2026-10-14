"""实体命令层 + 发射/补给组件（§5.18，v0.13.58）。

组件级用轻量桩（registry 用真的 ``EntityRegistry``——它不依赖任何服务；
store/spawn 用桩），把每条指令的**检查链**逐道过一遍。真想定端到端
（fire → 弹实体出生 → 制导接管）由 ``mover_demo`` 风格的装配测试覆盖
在 ``test_simulation.py`` 的既有机制上，这里不重复起引擎。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from milsim.errors import ConfigurationError
from milsim.models.component import Component
from milsim.models.entity import Entity
from milsim.models.supply import Supply
from milsim.models.weapon import Weapon
from milsim.services.bus import EventBus
from milsim.services.registry import EntityRegistry


# ---------------------------------------------------------------------------
# 命令层：Component.execute 的闸门与 Entity.command 的分发
# ---------------------------------------------------------------------------


class _Echo(Component):
    COMMANDS = {"ping": "回一声"}

    def execute(self, cmd, args):
        ok, why = super().execute(cmd, args)
        if not ok and cmd not in self.COMMANDS:
            return ok, why
        return True, f"pong:{args.get('x', '')}"


def _entity_with(*parts: Component) -> Entity:
    e = Entity()
    for i, p in enumerate(parts):
        e.add(f"slot{i}", p)
    return e


def test_base_component_rejects_undeclared() -> None:
    ok, why = Component(spec={}).execute("fire", {})
    assert ok is False
    assert "不接受指令" in why


def test_entity_command_dispatches_in_mount_order() -> None:
    a, b = _Echo(spec={}), _Echo(spec={})
    entity = _entity_with(a, b)
    ok, why = entity.command("ping", {"x": 1})
    assert ok and why == "pong:1"          # 两个都能接——先挂的接


def test_entity_command_rejects_when_nobody_declares() -> None:
    entity = _entity_with(_Echo(spec={}))
    ok, why = entity.command("fire", {})
    assert ok is False
    assert "没有部件接受" in why


# ---------------------------------------------------------------------------
# Weapon：fire 的检查链 + reload
# ---------------------------------------------------------------------------


def _weapon_spec(**over):
    spec = {
        "weapon_class": "SSM",
        "weapon_type": "MY_MISSILE",
        "range_min": 0.0,
        "range_max": 40_000.0,
        "stock": 2,
        "az_min": -180.0,
        "az_max": 180.0,
        "el_min": -90.0,
        "el_max": 90.0,
        "dead_zone": (),
    }
    spec.update(over)
    return spec


class _StubEntity:
    def __init__(self, eid: int, name: str) -> None:
        self.entity_id = eid
        self.name = name
        self._slots: dict[str, object] = {}

    def add(self, slot: str, part: object) -> None:
        self._slots[slot] = part

    def component(self, slot: str):
        return self._slots.get(slot)

    def parts(self) -> list:
        return list(self._slots.values())


class _StubGuidance:
    def __init__(self) -> None:
        self.target_id: int | None = None

    def assign_target_id(self, eid: int) -> None:
        self.target_id = eid


def _rig(
    *,
    stock: int = 2,
    range_max: float = 40_000.0,
    heading: float = 0.0,
    target_pos: tuple[float, float, float] = (30_000.0, 0.0, 0.0),
    **spec_over,
):
    """发射方 id=0 在原点（体轴朝 ``heading``），目标 id=1 在 ``target_pos``，
    弹出生 id=10。其余关键字原样透传给 ``_weapon_spec``。"""
    reg = EntityRegistry()
    shooter, target = _StubEntity(0, "DDG"), _StubEntity(1, "TARGET")
    reg.register(shooter, name="DDG", side="red", type_name="SHIP")
    reg.register(target, name="TARGET", side="blue", type_name="SHIP")

    positions = {0: (0.0, 0.0, 0.0), 1: target_pos}
    poses = {
        0: (0.0, 0.0, 0.0, heading, 0.0),
        1: (*target_pos, 0.0, 0.0),
    }
    spawned: list[dict] = []

    def spawn(type_name, **kw):
        missile = _StubEntity(-1, "M-1")
        guide = _StubGuidance()
        missile.add("guidance", guide)
        mid = reg.register(missile, name="M-1", side="red", type_name=type_name)
        spawned.append({"type": type_name, "guide": guide, **kw})
        return mid

    store = SimpleNamespace(
        position_of=lambda eid: positions.get(eid),
        pose_of=lambda eid: poses.get(eid),
    )

    mount = SimpleNamespace(
        entity_id=0,
        registry=reg,
        store=store,
        bus=EventBus(),
        spawn=spawn,
        my_cell=lambda: "CELL",
    )

    weapon = Weapon(spec=_weapon_spec(stock=stock, range_max=range_max, **spec_over))
    weapon.bind(shooter)
    weapon.initialize(mount)
    return weapon, spawned, SimpleNamespace(registry=reg, store=store)


def test_weapon_fire_spawns_missile_and_assigns_target() -> None:
    weapon, spawned, _ = _rig()
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok, why
    assert len(spawned) == 1
    call = spawned[0]
    assert call["type"] == "MY_MISSILE"
    assert call["side"] == "red"
    assert call["cell"] == "CELL"
    assert call["heading_deg"] == pytest.approx(90.0)   # 正东
    assert call["guide"].target_id == 1                 # 目标交给了制导管
    assert "余弹 1" in why
    assert weapon.stock() == 1                          # 扣了一发


def test_weapon_fire_rejects_out_of_envelope() -> None:
    weapon, spawned, _ = _rig(range_max=10_000.0)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok is False
    assert "包线外" in why
    assert spawned == []                                # 弹没出生
    assert weapon.stock() == 2                          # 也没扣弹


def test_weapon_fire_rejects_when_dry() -> None:
    weapon, spawned, _ = _rig(stock=0)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok is False
    assert "弹药耗尽" in why and spawned == []


# ---------------------------------------------------------------------------
# Weapon：三维射界（v0.13.60）——水平扇区/高低扇区/结构死区，全部体轴
# ---------------------------------------------------------------------------


def test_weapon_rejects_target_above_elevation_envelope() -> None:
    # 目标在 (30km, 0, 20km)：仰角 atan2(20, 30) ≈ 33.7° > 30 → 拒
    weapon, spawned, _ = _rig(el_max=30.0, target_pos=(30_000.0, 0.0, 20_000.0))
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok is False and "高低射界" in why and spawned == []


def test_weapon_elevation_envelope_admits_level_target() -> None:
    # 同一包线，平飞目标仰角 0 → 过。默认 ±90 不设限的行为由其余用例覆盖。
    weapon, spawned, _ = _rig(el_max=30.0)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok and why and len(spawned) == 1


def test_weapon_rejects_target_in_structural_dead_zone() -> None:
    # 发射方朝北（heading 0），目标正东 = 体轴 90°；死区 [60..120] 罩住它
    weapon, spawned, _ = _rig(dead_zone=["60..120"])
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok is False and "结构死区" in why and spawned == []
    assert weapon.stock() == 2                       # 拒发不扣弹


def test_weapon_azimuth_sector_rotates_with_heading() -> None:
    # 窄射界 [-10..10]：朝北时正东目标在体轴 90° → 拒；
    # 转向东（heading 90）后同一目标进体轴 0° → 过，弹头朝目标。
    weapon, spawned, _ = _rig(az_min=-10.0, az_max=10.0, heading=0.0)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok is False and "水平射界" in why and spawned == []

    weapon, spawned, _ = _rig(az_min=-10.0, az_max=10.0, heading=90.0)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok, why
    assert spawned[0]["heading_deg"] == pytest.approx(90.0)


def test_weapon_circular_arc_sector_covers_the_rear() -> None:
    # az 170..-170（lo > hi = 跨 ±180 环形）= 正后方扇区。
    # y 轴指南：正南目标 (0, +30km) 方位 180°，体轴也是 180° → 在扇区内；
    # 正东目标（体轴 90°）不在 → 拒。
    weapon, _, _ = _rig(az_min=170.0, az_max=-170.0,
                        target_pos=(0.0, 30_000.0, 0.0))
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok, why

    weapon, spawned, _ = _rig(az_min=170.0, az_max=-170.0)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok is False and "水平射界" in why and spawned == []


def test_weapon_fire_rejects_same_side() -> None:
    reg = EntityRegistry()
    shooter, target = _StubEntity(0, "A"), _StubEntity(1, "B")
    reg.register(shooter, name="A", side="red", type_name="T")
    reg.register(target, name="B", side="red", type_name="T")
    weapon = Weapon(spec=_weapon_spec(range_max=0.0))
    weapon.bind(shooter)
    weapon.initialize(
        SimpleNamespace(
            entity_id=0,
            registry=reg,
            store=SimpleNamespace(
                position_of=lambda eid: (0.0, 0.0, 0.0)
            ),
            bus=EventBus(),
            spawn=None,
            my_cell=lambda: None,
        )
    )
    ok, why = weapon.execute("fire", {"target": 1})
    assert ok is False and "同属" in why


def test_weapon_fire_needs_a_target_ref() -> None:
    weapon, _, _ = _rig()
    ok, why = weapon.execute("fire", {})
    assert ok is False and "target" in why


def test_weapon_reload_refills_to_capacity() -> None:
    weapon, _, _ = _rig(stock=2)
    weapon._stock = 0
    ok, why = weapon.execute("reload", {"rounds": 5})
    assert ok and weapon.stock() == 2                   # 弹匣上限 2
    assert "装填 2 发" in why


def test_weapon_unknown_command_falls_to_base_gate() -> None:
    weapon, _, _ = _rig()
    ok, why = weapon.execute("self_destruct", {})
    assert ok is False and "不接受指令" in why


# ---------------------------------------------------------------------------
# Supply：resupply 的距离与转手
# ---------------------------------------------------------------------------


class _StubWeaponSlot:
    def __init__(self, capacity: int, stock: int) -> None:
        self._capacity = capacity
        self._stock = stock

    def stock(self) -> int:
        return self._stock

    def add_stock(self, n: int) -> int:
        before = self._stock
        self._stock = min(self._capacity, self._stock + n)
        return self._stock - before


class _StubMoverSlot:
    def __init__(self, cap: float, fuel: float) -> None:
        self._cap = cap
        self._fuel = fuel

    def fuel_capacity_kg(self) -> float:
        return self._cap

    def fuel_kg(self) -> float:
        return self._fuel

    def add_fuel(self, kg: float) -> float:
        before = self._fuel
        self._fuel = min(self._cap, self._fuel + kg)
        return self._fuel - before


def _supply_rig(*, positions: dict, target_parts: list):
    reg = EntityRegistry()
    truck, target = _StubEntity(0, "TRUCK"), _StubEntity(1, "NEEDY")
    reg.register(truck, name="TRUCK", side="red", type_name="T")
    reg.register(target, name="NEEDY", side="red", type_name="T")
    target._slots = {f"s{i}": p for i, p in enumerate(target_parts)}
    registry = SimpleNamespace(
        get=reg.get, by_name=reg.by_name, side_of=reg.side_of
    )
    mount = SimpleNamespace(
        entity_id=0,
        registry=registry,
        store=SimpleNamespace(position_of=lambda eid: positions.get(eid)),
    )

    supply = Supply(spec={"ammo": 4, "fuel_kg": 500.0, "resupply_radius": 500.0})
    supply.bind(truck)
    supply.initialize(mount)
    return supply


def test_supply_transfers_ammo_and_fuel_in_radius() -> None:
    wslot = _StubWeaponSlot(capacity=4, stock=1)
    mslot = _StubMoverSlot(cap=1000.0, fuel=100.0)
    supply = _supply_rig(
        positions={0: (0.0, 0.0, 0.0), 1: (300.0, 0.0, 0.0)},
        target_parts=[wslot, mslot],
    )
    ok, why = supply.execute("resupply", {"supply_to": "NEEDY"})
    assert ok, why
    assert wslot.stock() == 4                 # 弹匣差 3 发，车上 4 发弹够补满
    assert mslot.fuel_kg() == pytest.approx(600.0)  # 差 900，车上油 500 全给
    assert supply.ammo_left() == 1
    assert supply.fuel_left_kg() == pytest.approx(0.0)


def test_supply_rejects_out_of_radius() -> None:
    supply = _supply_rig(
        positions={0: (0.0, 0.0, 0.0), 1: (10_000.0, 0.0, 0.0)},
        target_parts=[_StubWeaponSlot(4, 0)],
    )
    ok, why = supply.execute("resupply", {"supply_to": 1})
    assert ok is False and "补给半径" in why


def test_supply_rejects_when_nothing_needed() -> None:
    supply = _supply_rig(
        positions={0: (0.0, 0.0, 0.0), 1: (10.0, 0.0, 0.0)},
        target_parts=[_StubWeaponSlot(4, 4), _StubMoverSlot(1000.0, 1000.0)],
    )
    ok, why = supply.execute("resupply", {"supply_to": 1})
    assert ok is False and "不缺" in why


# ---------------------------------------------------------------------------
# Mover 油料数学（不起引擎，直接打私有状态的口径）
# ---------------------------------------------------------------------------


def test_mover_fuel_math_and_clamps() -> None:
    from milsim.models.mover.base import Mover

    mover = Mover.__new__(Mover)            # 只测油料方法，不接线
    mover._fuel_cap = 1000.0
    mover._fuel_kg = 1000.0
    mover.spec = {"fuel_rate": 10.0}        # 10 kg/km
    mover._burn_fuel(1500.0)                # 走 1.5 km
    assert mover._fuel_kg == pytest.approx(985.0)
    mover._burn_fuel(10_000_000.0)          # 远超油量：clamp 到 0
    assert mover._fuel_kg == 0.0
    assert mover._out_of_fuel() is True
    assert mover.add_fuel(5000.0) == pytest.approx(1000.0)   # 加满为止
    assert mover._out_of_fuel() is False
    mover._fuel_cap = 0.0                   # 无油模型：加不进、烧不掉
    mover._fuel_kg = 0.0
    assert mover.add_fuel(100.0) == 0.0
    assert mover._out_of_fuel() is False


# ---------------------------------------------------------------------------
# 装配期校验
# ---------------------------------------------------------------------------


def test_weapon_requires_a_missile_type() -> None:
    weapon = Weapon(spec=_weapon_spec(weapon_type=""))
    entity = _StubEntity(1, "DDG")
    weapon.bind(entity)
    with pytest.raises(ConfigurationError):
        weapon.initialize(
            SimpleNamespace(registry=EntityRegistry(), bus=EventBus())
        )
