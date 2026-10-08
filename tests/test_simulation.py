"""装配与驱动的测试。

这一层把前面所有模块接起来，所以重点测**接线对不对**而不是算法对不对：

- 战区是否**按需激活**（声明 3 个只用到 1 个时，另外 2 个不该占资源）
- 位置是否落到正确的层（战区外退回全球层）
- 组件是否真的挂上、参数是否三层合并正确
- 决策 provider 是否按配置造出来、降级链是否闭合
- 周期事件是否真的被引擎驱动

最容易出静默错误的是前两条：实体"看起来建好了"，但格归属挂在错误的
战区里、或者压根没进空间索引，要等到雷达搜不到它才暴露。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from milsim.decision.llm import LLMDecisionProvider, MockLLMClient
from milsim.decision.rule import RuleDecisionProvider
from milsim.engine import EventResult
from milsim.models.component import Component
from milsim.models.entity import Entity
from milsim.services.params import Params
from milsim.services.scenario import ScenarioError, load_scenario
from milsim.services.store import LocalCellRef
from milsim.services.type_registry import ComponentRegistry
from milsim.simulation import Simulation, SimulationError

DEMO = Path(__file__).resolve().parents[1] / "scenarios" / "demo.txt"


# ---------------------------------------------------------------------------
# 测试用的假组件
# ---------------------------------------------------------------------------

class FakeRadar(Component):
    """最小可用的雷达桩：注册周期扫描，数自己扫了几次。"""

    PARAMS = {
        "detect_range": Params.distance(40_000.0),
        "scan_interval": Params.duration(2_000_000),
        "track_capacity": Params.integer(40),
        "fov": Params.angle(120.0),
        "frequency": Params.frequency(9.4e9),
        "modes": Params.strings(),
    }

    def initialize(self, mount) -> None:
        self.scans = 0
        self.mount = mount
        self.view = mount.sensor_view()
        mount.every(self.spec["scan_interval"], self._scan)

    def _scan(self, engine, event) -> EventResult:
        self.scans += 1
        return EventResult.RESCHEDULE


class Exploding(Component):
    """构造就失败的组件，用来验证报错能指出是哪个槽。"""

    PARAMS = {"must_be_declared": Params.distance(1.0)}

    def __init__(self, *, spec) -> None:
        super().__init__(spec=spec)
        raise RuntimeError("构造失败")


def registry_with_radar() -> ComponentRegistry:
    registry = ComponentRegistry()
    registry.register("HEX_SEARCH_RADAR", FakeRadar)
    return registry


def demo_simulation(**kwargs) -> Simulation:
    kwargs.setdefault("components", registry_with_radar())
    kwargs.setdefault("llm_client_factory", lambda profile: MockLLMClient())
    return Simulation.from_scenario_file(DEMO, **kwargs)


# ---------------------------------------------------------------------------
# 装配：战区按需激活
# ---------------------------------------------------------------------------

def test_only_zones_with_entities_get_activated() -> None:
    """声明 3 个战区，只有 1 个有实体落入——另外 2 个不该占任何资源。

    这是跨洲想定能跑起来的前提：全球战区全加载是不现实的。
    """
    text = """
simulation
    max_step 2 s
end_simulation

zone NORTH
    anchor 39.9042 116.4074
    radius 120 km
    resolution 100 m
end_zone

zone SOUTH
    anchor 22.5 114.0
    radius 120 km
    resolution 100 m
end_zone

zone FAR
    anchor -33.87 151.21
    radius 120 km
    resolution 100 m
end_zone

platform_type P
end_platform_type

platform A P
    position latlng 39.95 116.35
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    sim.build()

    assert sim.zone_ids == {"NORTH": 1, "SOUTH": 2, "FAR": 3}
    assert [z.spec.name for z in sim.maps.active_zones] == ["NORTH"]
    assert sim.maps.stats["declared"] == 3
    assert sim.maps.stats["activated"] == 1


def test_entity_outside_any_zone_is_rejected_at_build() -> None:
    """★ v0.13.39 起：落不到任何战区的经纬度平台**装配期报错**。

    旧行为是静默退回全球层 + ``(0, 0, 0)``。那条路之所以必须封掉：``GlobalCellRef``
    **不带任何米制坐标**，所以那三个 0 是**写死的**，不是算出来的。后果是实体全部
    叠在原点、雷达 ``span_m <= 0`` 一步早退、**退出码 0**、图上什么都没有——
    而参数表上看不出任何问题（§9-55）。⇒ "战区外"这个概念在当前版本**没有可用
    的几何表示**，唯一正确的做法是让想定作者知道。
    """
    text = """
zone NORTH
    anchor 39.9042 116.4074
    radius 120 km
end_zone

platform_type P
end_platform_type

platform FERRY P
    position latlng 22.5 114.0
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="落在所有战区之外"):
        sim.build()


def test_latlng_platform_without_any_zone_is_rejected_at_build() -> None:
    """★ 一个 ``zone`` 都没有时，经纬度平台同样报错（提示不同：先加战区）。

    这一条与上一条分开测，因为**改法不同**：这里要的是"加一个战区"，那里要的是
    "把位置挪进去或把战区调大"。一句话糊住两种情形，作者不知道该改哪一处。
    """
    text = """
platform_type P
end_platform_type

platform A P
    position latlng 39.9 116.4
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="一个 zone 都没有"):
        sim.build()


def test_entity_inside_zone_gets_local_cell() -> None:
    sim = demo_simulation()
    sim.build()

    entity_id = sim.registry.by_name("SAM_1").entity_id
    ref = sim.store.cell_of(entity_id)
    assert isinstance(ref, LocalCellRef)
    assert ref.zone_id == sim.zone_ids["NORTH_FRONT"]

    x, y, z = sim.store.position_of(entity_id)
    assert (x, y) != (0.0, 0.0)                # 局部坐标非零，说明真的投影了
    assert z == 0.0


def test_overlapping_zones_are_rejected() -> None:
    """战区重叠会让实体坐标在加载边界来回跳，所以直接报错。"""
    text = """
zone A
    anchor 39.9042 116.4074
    radius 120 km
end_zone

zone B
    anchor 39.95 116.40
    radius 120 km
end_zone

platform_type P
end_platform_type

platform X P
    position latlng 39.93 116.39
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="同时落在多个战区"):
        sim.build()


# ---------------------------------------------------------------------------
# 装配：网格坐标
# ---------------------------------------------------------------------------

def test_hex_position_single_zone_needs_no_zone_name() -> None:
    text = """
zone ONLY
    anchor 39.9042 116.4074
    radius 120 km
    resolution 100 m
end_zone

platform_type P
end_platform_type

platform A P
    position hex 10 5 @ layer 0
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    sim.build()
    ref = sim.store.cell_of(sim.registry.by_name("A").entity_id)
    assert isinstance(ref, LocalCellRef)
    assert (ref.axial.q, ref.axial.r) == (10, 5)


def test_hex_position_requires_zone_when_ambiguous() -> None:
    """两个战区时，同一个 (q, r) 是两个地方，必须写明属于哪个。"""
    text = """
zone A
    anchor 39.9042 116.4074
    radius 120 km
end_zone

zone B
    anchor -33.87 151.21
    radius 120 km
end_zone

platform_type P
end_platform_type

platform X P
    position hex 10 5
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="@ zone"):
        sim.build()


def test_hex_position_outside_zone_is_rejected() -> None:
    """战区外没有网格。不校验的话实体会"站在虚空里"——格归属写着战区，
    实际位置却在几百公里外。"""
    text = """
zone NORTH
    anchor 39.9042 116.4074
    radius 120 km
    resolution 100 m
end_zone

platform_type P
end_platform_type

platform A P
    position hex 1204 883 @ layer 0
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="在战区 NORTH 之外"):
        sim.build()


def test_hex_position_with_explicit_zone() -> None:
    text = """
zone NORTH
    anchor 39.9042 116.4074
    radius 120 km
    resolution 100 m
end_zone

zone SOUTH
    anchor -33.87 151.21
    radius 120 km
    resolution 100 m
end_zone

platform_type P
end_platform_type

platform A P
    position hex 10 5 @ zone SOUTH layer 1
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    sim.build()
    ref = sim.store.cell_of(sim.registry.by_name("A").entity_id)
    assert isinstance(ref, LocalCellRef)
    assert ref.layer == 1
    assert ref.zone_id == sim.zone_ids["SOUTH"]


# ---------------------------------------------------------------------------
# 装配：实体与组件
# ---------------------------------------------------------------------------

def test_entities_are_registered_with_side_and_type() -> None:
    sim = demo_simulation()
    sim.build()

    sam = sim.entity_by_name("SAM_1")
    assert sam is not None
    assert sim.registry.side_of(sam.entity_id) == "red"
    assert sim.registry.type_of(sam.entity_id) == "SAM_BATTALION"
    assert sim.registry.by_side("red") == [0, 1]


def test_components_are_mounted_on_declared_slot() -> None:
    sim = demo_simulation()
    sim.build()

    sam = sim.entity_by_name("SAM_1")
    assert sam.slots() == ["sensor"]
    radar = sam.component("sensor")
    assert isinstance(radar, FakeRadar)
    assert radar.entity is sam                      # bind 过了
    assert radar.slot == "sensor"


def test_three_layer_parameter_merge_reaches_component() -> None:
    """类默认值 ← component_type 链 ← 平台挂载差量，最终要落到部件上。"""
    sim = demo_simulation()
    sim.build()

    radar = sim.entity_by_name("SAM_1").component("sensor")
    # demo.txt 里 SAM_BATTALION 的 sensor 槽写了 detect_range 60 km，
    # 覆盖类型定义里的 40 km，也覆盖类的默认值
    assert radar.spec["detect_range"] == pytest.approx(60_000.0)
    # fov 从 component_type HEX_SEARCH_RADAR 继承，单位换算成度
    assert radar.spec["fov"] == pytest.approx(120.0)
    # frequency 从 9.4 GHz 换算成 Hz
    assert radar.spec["frequency"] == pytest.approx(9.4e9)
    # 父类型 RADAR_BASE 的 track_capacity 继承下来
    assert radar.spec["track_capacity"] == 40


def test_component_construction_failure_names_the_slot() -> None:
    text = """
zone AO
    anchor 39.9042 116.4074
    radius 20 km
end_zone

platform_type P
    component sensor HEX_SEARCH_RADAR
    end_component
end_platform_type

platform A P
    position latlng 39.95 116.35
end_platform
"""
    registry = ComponentRegistry()
    registry.register("HEX_SEARCH_RADAR", Exploding)
    sim = Simulation.from_scenario(text, components=registry)
    with pytest.raises(Exception, match="sensor"):
        sim.build()


def test_duplicate_slot_on_entity_is_rejected() -> None:
    """同槽挂两次是笔误——静默覆盖会丢一个部件而毫无提示。"""
    entity = Entity()
    entity.name = "X"
    entity.add("sensor", FakeRadar(spec=_spec("HEX_SEARCH_RADAR")))
    with pytest.raises(ValueError, match="sensor"):
        entity.add("sensor", FakeRadar(spec=_spec("HEX_SEARCH_RADAR")))


def _spec(type_name: str, slot: str = "sensor"):
    """造一个最小 ResolvedComponent 用于单测。"""
    from milsim.services.type_registry import ResolvedComponent

    return ResolvedComponent(slot, type_name, FakeRadar, {})


# ---------------------------------------------------------------------------
# 决策接线
# ---------------------------------------------------------------------------

def test_rule_profile_becomes_rule_provider() -> None:
    sim = demo_simulation()
    sim.build()
    assert isinstance(sim.decisions["RULE_DEFAULT"], RuleDecisionProvider)


def test_llm_profile_becomes_llm_provider_with_fallback() -> None:
    sim = demo_simulation()
    sim.build()
    provider = sim.decisions["TACTICAL_LLM"]
    assert isinstance(provider, LLMDecisionProvider)


def test_llm_client_factory_receives_the_profile() -> None:
    """工厂收到的是配置本身——密钥从哪个环境变量读由它决定。"""
    seen: list[tuple[str, str, str]] = []

    def factory(profile):
        seen.append((profile.name, profile.model, profile.api_key_env))
        return MockLLMClient()

    sim = demo_simulation(llm_client_factory=factory)
    sim.build()

    assert seen == [("TACTICAL_LLM", "deepseek-chat", "MILSIM_LLM_KEY")]


def test_llm_without_client_factory_is_an_error() -> None:
    """想定写了用大模型却不给客户端——报错，不静默退化成规则。

    "配了却没生效"是最难发现的一类问题。
    """
    sim = Simulation.from_scenario_file(DEMO, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="客户端工厂"):
        sim.build()


def test_fallback_chain_resolves_to_another_profile() -> None:
    text = """
zone AO
    anchor 39.9042 116.4074
    radius 20 km
end_zone

platform_type P
end_platform_type

decision BASE
    provider rule
end_decision

decision LLM_TOP
    provider    llm
    model       "m"
    api_key_env K
    fallback    BASE
end_decision

platform A P
    position latlng 39.9 116.4
end_platform
"""
    sim = Simulation.from_scenario(
        text,
        components=registry_with_radar(),
        llm_client_factory=lambda profile: MockLLMClient(),
    )
    sim.build()
    assert isinstance(sim.decisions["BASE"], RuleDecisionProvider)
    assert isinstance(sim.decisions["LLM_TOP"], LLMDecisionProvider)


def test_fallback_cycle_is_rejected() -> None:
    """fallback 成环会在递归构造时栈溢出，报错信息还不清不楚。"""
    text = """
decision A
    provider rule
    fallback B
end_decision

decision B
    provider rule
    fallback A
end_decision
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    with pytest.raises(ScenarioError, match="成环"):
        sim.build()


def test_default_rule_policy_picks_first_option() -> None:
    from milsim.decision.base import DecisionRequest

    sim = demo_simulation()
    sim.build()
    provider = sim.decisions["RULE_DEFAULT"]

    resolved = []
    provider.request(
        DecisionRequest(
            request_id=1,
            sim_time=0,
            entity_id=0,
            kind="maneuver",
            situation="",
            options=("advance", "hold"),
        ),
        resolved.append,
    )
    assert resolved[0].action == "advance"


# ---------------------------------------------------------------------------
# 驱动
# ---------------------------------------------------------------------------

def test_initialize_is_idempotent() -> None:
    sim = demo_simulation()
    sim.initialize()
    state = sim.engine.state
    sim.initialize()
    assert sim.engine.state is state
    assert sim._initialized


def test_build_is_idempotent() -> None:
    sim = demo_simulation()
    sim.build()
    count = len(sim.entities)
    sim.build()
    assert len(sim.entities) == count


def test_periodic_events_actually_run() -> None:
    """组件在自己的 initialize 里注册节拍，引擎要真的按节拍叫它。"""
    sim = demo_simulation()
    sim.initialize()
    sim.run_for(5_000_000)                 # 5 秒，扫描间隔 2 秒

    for entity in sim.entities.values():
        radar = entity.component("sensor")
        # t = 0, 2, 4 秒各扫一次（引擎的栅栏把时间对齐到 2 秒边界）
        assert radar.scans >= 3


def test_run_for_without_explicit_initialize() -> None:
    """run_for 自己会补 initialize——少写一行不该是错误。"""
    sim = demo_simulation()
    assert sim.run_for(2_000_000) >= 2_000_000
    assert sim._initialized


def test_repeat_run_advances_time_monotonically() -> None:
    sim = demo_simulation()
    sim.initialize()
    first = sim.run_for(2_000_000)
    second = sim.run_for(2_000_000)
    assert second > first


def test_report_counts_entities_and_zones() -> None:
    sim = demo_simulation()
    sim.initialize()
    sim.run_for(4_000_000)
    report = sim.shutdown()

    assert report.entities == 2
    assert report.alive == 2
    assert report.zones_active == 1
    assert report.steps > 0
    assert report.events_dispatched > 0
    assert "推演到" in report.summary()


def test_shutdown_is_safe_without_initialize() -> None:
    sim = demo_simulation()
    sim.build()
    report = sim.shutdown()
    assert report.entities == 2


def test_mount_context_exposes_views() -> None:
    sim = demo_simulation()
    sim.initialize()
    entity_id = sim.registry.by_name("SAM_1").entity_id
    mount = sim.mount_for(entity_id)

    assert mount.entity_id == entity_id
    assert mount.my_position() is not None
    assert mount.my_cell() is not None
    assert mount.now == sim.engine.now
    assert "SAM_1" in repr(mount)


def test_mount_for_unknown_entity_is_an_error() -> None:
    sim = demo_simulation()
    sim.build()
    with pytest.raises(SimulationError, match="不在本仿真里"):
        sim.mount_for(9999)


def test_activate_zone_manually() -> None:
    """想在空战区里做推演（比如先看地形）时显式激活。"""
    sim = demo_simulation()
    sim.build()
    zone = sim.activate_zone("NORTH_FRONT")
    assert zone.spec.name == "NORTH_FRONT"
    with pytest.raises(SimulationError, match="未声明的战区"):
        sim.activate_zone("NO_SUCH")


def test_alive_entities_iterates_in_id_order() -> None:
    sim = demo_simulation()
    sim.build()
    names = [e.name for e in sim.alive_entities()]
    assert names == ["SAM_1", "SAM_2"]


def test_entity_by_name_returns_none_for_unknown() -> None:
    sim = demo_simulation()
    sim.build()
    assert sim.entity_by_name("NOBODY") is None


def test_validate_is_clean_after_full_build() -> None:
    sim = demo_simulation()
    sim.initialize()
    sim.run_for(2_000_000)
    assert sim.validate() == []


def test_validate_catches_scenario_level_problems() -> None:
    text = """
platform_type P
end_platform_type

platform A P
    position latlng 39.9 116.4
    decision NO_SUCH
end_platform
"""
    sim = Simulation.from_scenario(text, components=registry_with_radar())
    problems = sim.validate()
    assert any("NO_SUCH" in p for p in problems)


def test_scenario_zone_radius_validated_by_zone_spec() -> None:
    """战区半径超出合理范围时，语义层就该拦住。"""
    text = "zone TINY\n    anchor 39.9 116.4\n    radius 100 m\nend_zone\n"
    with pytest.raises(Exception, match="不能小于"):
        load_scenario(text)


# ---------------------------------------------------------------------------
# 编队与指挥关系的接线（§3.10）
# ---------------------------------------------------------------------------
COMMAND_SCENARIO = """
zone AO
    anchor     39.9042 116.4074
    radius     60 km
    resolution 500 m
end_zone

formation RED_BDE
    side red
    echelon 旅
end_formation
formation RED_1BN : RED_BDE
    side red
    echelon 营
end_formation
formation RED_2BN : RED_BDE
    side red
    echelon 营
end_formation
formation RED_1_1 : RED_1BN
    side red
    echelon 连
end_formation
formation RED_2_1 : RED_2BN
    side red
    echelon 连
end_formation

command
    attach RED_1_1 to RED_2BN from 30 min until 8 h
end_command

platform_type TANK
    side red
end_platform_type

platform T_A TANK
    position  latlng 39.95 116.35
    formation RED_1_1
end_platform
platform T_B TANK
    position  latlng 39.95 116.36
    formation RED_2_1
end_platform
"""


def test_simulation_builds_command_chain() -> None:
    sim = Simulation.from_scenario(COMMAND_SCENARIO)
    sim.build()

    assert len(sim.formations) == 5
    assert len(sim.commands.attachments) == 1

    # 平台真的编入了编队
    one = sim.formations.resolve("RED_1_1")
    assert len(sim.formations.members_of(one)) == 1
    assert sim.formations.unit_of_entity(sim.registry.by_name("T_A").entity_id) == one


def test_simulation_projection_reflects_attachment() -> None:
    """态势里的一连，T+0 在一营下，T+1h 在二营下（配属生效）。"""
    sim = Simulation.from_scenario(COMMAND_SCENARIO)
    sim.build()

    one = sim.formations.resolve("RED_1_1")
    bn1 = sim.formations.resolve("RED_1BN")
    bn2 = sim.formations.resolve("RED_2BN")

    def parent_at(moment: int) -> int | None:
        situation = sim.projector.project(side="red", at=moment)
        return next(f for f in situation.formations if f.unit_id == one).parent_id

    assert parent_at(0) == bn1
    assert parent_at(3_600_000_000) == bn2
    assert parent_at(9 * 3_600_000_000) == bn1


def test_simulation_rejects_conflicting_attachments() -> None:
    """配属时间重叠必须在装配时报错，不能等到投影才重复计数。"""
    text = COMMAND_SCENARIO.replace(
        "    attach RED_1_1 to RED_2BN from 30 min until 8 h\n",
        "    attach RED_1_1 to RED_2BN from 1 h until 3 h\n"
        "    attach RED_1_1 to RED_BDE from 2 h until 4 h\n",
    )
    sim = Simulation.from_scenario(text)
    with pytest.raises(ScenarioError, match="指挥关系自检未通过"):
        sim.build()


def test_simulation_without_formations_still_works() -> None:
    """没写 formation 块的想定照常能跑——编队是可选能力。"""
    text = (
        "zone AO\n    anchor 39.9042 116.4074\n    radius 20 km\nend_zone\n"
        "platform_type TANK\n    side red\nend_platform_type\n"
        "platform T_A TANK\n    position latlng 39.95 116.35\nend_platform\n"
    )
    sim = Simulation.from_scenario(text)
    sim.build()
    assert len(sim.formations) == 0
    situation = sim.projector.project(side="red", at=0)
    # 没有编队时，实体归入"未编队"伪编队
    assert len(situation.formations) == 1
    assert situation.formations[0].name == "未编队"


def test_the_ew_facade_can_read_a_platforms_own_tracks() -> None:
    """装配层把 ``store.contacts_of`` 注入电子战门面（v0.13.34，§5.14.11 ④）。

    ★ 这条守的是"**正式装配也能照敌航迹置向**"（用户的 ⟨乙⟩ 裁定）：门面拿到的
    是 store 的一个**只读切片**（"按实体取它自己那张航迹表"），而不是整个
    ``EntityStore``——所以这里既验证它**接通了**，也验证它读到的就是**那个实体
    自己的**航迹（别人的看不见）。
    """
    text = (
        "zone AO\n    anchor 39.9042 116.4074\n    radius 20 km\nend_zone\n"
        "platform_type TANK\n    side red\nend_platform_type\n"
        "platform T_A TANK\n    position latlng 39.95 116.35\nend_platform\n"
    )
    sim = Simulation.from_scenario(text)
    sim.build()
    eid = sim.registry.all_ids()[0]

    # 写一条只有 A 自己看得见的航迹
    sim.store.add_contact(eid, 999, x=1.0, y=2.0, z=3.0)
    mine = sim.jam._tracks_of(eid)
    assert [(c.target_id, c.x) for c in mine] == [(999, 1.0)]

    # 别人（一个不存在的实体）的航迹表是空的 —— 门面拿不到别人的东西
    assert sim.jam._tracks_of(12345) == []

