"""平台型参数表测试：``platform_type`` 的属性也进同一套校验（§5.6）。

为什么平台也要有参数表
----------------------
在这之前 ``platform_type`` 的属性是**自由字典**：写什么、拼错什么都不报错，
参数库也管不到它。后果是"想定里写了 ``sdie red``，实体建档时 side 是空串"，
而空串看起来跟"没指定"一模一样——一条想定的阵营归属就这么静默丢了。

这一组盯三件事：

1. **隐式根 ``PLATFORM``**：平台类型没有算法，链上找不到具体类落到根是
   语义正确的默认，不是兜底（组件侧则必须指向具体算法，找不到就报错）。
2. **与组件参数走同一段解析**：拼错给建议、类型与范围照查、三层覆盖顺序
   一致。两条入口的差别（一条走了 ``ParamSet.resolve``、另一条忘了）正是
   本项目最防的那类错。
3. **平台层只留参数、不含行为**：一旦平台自己会动，"这辆车能跑多快"就有
   两处真相（平台属性一份、机动件参数一份），而两份数字迟早不同步。

除接线以外一律用**独立注册表**，不碰全局表：想定级的用例会把类注册进去，
用全局表跑会让测试之间互相污染，而那种污染只在换顺序执行时才暴露。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.models.platform import Platform
from milsim.services.params import ParamSpec, Params, UnknownParameterError
from milsim.services.type_registry import (
    ROOT_PLATFORM,
    ComponentFactory,
    ComponentRegistry,
    DuplicateComponentError,
    PlatformRegistry,
    TypeRegistry,
    UnknownComponentError,
)
from milsim.simulation import Simulation


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------

class ArmorHull(Platform):
    """一个自己型号族的平台类：多一个"有人读"的属性。"""

    PARAMS = {
        **Platform.PARAMS,
        "crew": Params.integer(3, minimum=0),
    }


@pytest.fixture
def rig() -> tuple[ComponentFactory, TypeRegistry, PlatformRegistry]:
    """独立注册表 + 一份只含平台类型的想定类型表。"""
    from milsim.models import register_framework

    components, platforms = ComponentRegistry(), PlatformRegistry()
    register_framework(components, platforms)
    platforms.register("ARMOR_HULL", ArmorHull)

    types = TypeRegistry()
    types.define_platform_type("TANK")
    # 型号族既要在**代码层**注册（platforms.register），也要在**想定层**
    # 声明（define_platform_type）——两件事：前者给参数表，后者给继承链。
    # 只做前者的话，想定里 `: ARMOR_HULL` 会报"继承自未定义的类型"。
    types.define_platform_type("ARMOR_HULL")
    types.define_platform_type("ARMOR_MBT", parent="ARMOR_HULL", attributes={"crew": 4})
    types.define_platform_type(
        "ARMOR_BASE", parent="ARMOR_HULL", attributes={"side": "red"}
    )
    return ComponentFactory(components, types, platforms), types, platforms


# ---------------------------------------------------------------------------
# 注册与隐式根
# ---------------------------------------------------------------------------

def test_importing_models_registers_the_root_platform() -> None:
    """``import milsim.models`` 就等于"把框架自带的平台类装进全局表"。"""
    from milsim.services.type_registry import default_platforms

    assert default_platforms().get(ROOT_PLATFORM) is Platform
    assert Platform.PLATFORM_NAME == ROOT_PLATFORM
    assert ROOT_PLATFORM in default_platforms()


def test_platform_type_without_a_parent_falls_back_to_the_root(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    """没有父类型的平台类型落到隐式根。

    这不是兜底：平台类型没有算法，行为在组件里。所有平台共享同一组基础
    属性，链上找不到具体类落到根是**语义正确**的默认。父类型写错这件事
    由 ``TypeRegistry.platform_chain`` 把关（未定义就报错），所以这里不会
    掩盖拼错。
    """
    factory, _, _ = rig
    assert factory.platform_class("TANK") is Platform
    assert factory.platform_class("ARMOR_MBT") is ArmorHull


def test_platform_param_set_owner_says_which_layer_it_came_from(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    """报错时要能看出**参数表来自哪一层**。

    落到根是常态，但"本该指向某个型号族却没指向"只有把类名打出来才看得见。
    """
    factory, _, _ = rig
    assert ROOT_PLATFORM in factory.platform_param_set("TANK").owner
    assert "ARMOR_MBT" in factory.platform_param_set("ARMOR_MBT").owner


def test_missing_root_is_reported_with_a_hint() -> None:
    """连根都没有时要说清"是不是没 import milsim.models"。"""
    empty = PlatformRegistry()
    types = TypeRegistry()
    types.define_platform_type("TANK")
    factory = ComponentFactory(ComponentRegistry(), types, empty)
    with pytest.raises(UnknownComponentError, match="PLATFORM"):
        factory.platform_class("TANK")


def test_duplicate_platform_registration_is_loud() -> None:
    """重名会静默覆盖，所以直接报错。"""
    platforms = PlatformRegistry()
    platforms.register("A", ArmorHull)
    with pytest.raises(DuplicateComponentError, match="override"):
        platforms.register("A", Platform)
    platforms.register("A", Platform, override=True)      # 明确要替换才允许
    assert platforms.get("A") is Platform


def test_register_framework_fills_an_empty_registry() -> None:
    """``register_framework`` 要把框架自带的实现装进**指定的**注册表。

    漏加一个类的表现是想定里写了那个名字却报"没有实现类"——错得很响，
    不会静默降级；但更要紧的是它得**真的装上**，否则独立注册表里空无一物。
    """
    from milsim.models import FRAMEWORK_COMPONENTS, FRAMEWORK_PLATFORMS, register_framework

    components, platforms = ComponentRegistry(), PlatformRegistry()
    assert len(components) == 0 and len(platforms) == 0
    register_framework(components, platforms)

    for cls in FRAMEWORK_COMPONENTS:
        assert components.get(cls.COMPONENT_NAME) is cls
    for cls in FRAMEWORK_PLATFORMS:
        assert platforms.get(cls.PLATFORM_NAME) is cls

    # 装好之后工厂就能解析平台与组件了
    types = TypeRegistry()
    types.define_platform_type("TANK")
    factory = ComponentFactory(components, types, platforms)
    assert factory.platform_class("TANK") is Platform
    mover = factory.build("mover", "GROUND_MOVER")
    assert float(mover.spec["max_speed"]) > 0.0


# ---------------------------------------------------------------------------
# 参数解析：与组件同一套
# ---------------------------------------------------------------------------

def test_platform_type_attributes_are_resolved(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    factory, _, _ = rig
    assert factory.resolved_platform_params("TANK") == {
        "side": "",
        "decision": "",
        "radar_cross_section": 1.0,
    }
    assert factory.resolved_platform_params("ARMOR_BASE")["side"] == "red"
    assert factory.resolved_platform_params("ARMOR_MBT")["crew"] == 4


def test_class_defaults_are_kept_when_nothing_overrides_them(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    """型号族声明的默认值要真的起作用——否则"派生一个型号族"毫无收益。"""
    factory, _, _ = rig
    assert factory.resolved_platform_params("ARMOR_BASE")["crew"] == 3


def test_override_wins_over_the_type_chain(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    """三层顺序：类默认值 ← 平台类型链 ← 本次覆盖。"""
    factory, _, _ = rig
    got = factory.resolved_platform_params("ARMOR_MBT", {"crew": 5, "side": "blue"})
    assert got["crew"] == 5
    assert got["side"] == "blue"


def test_typo_in_a_platform_attribute_is_loud(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    """拼错一个键就报错，并给拼写建议。

    这条是 §9 第 15 条的正面：从前平台属性是自由字典，"写了没人读、拼错
    也不报错"。现在它走的是与组件参数**完全同一段** ``ParamSet.resolve``。
    """
    factory, _, _ = rig
    with pytest.raises(UnknownParameterError) as info:
        factory.resolved_platform_params("TANK", {"sdie": "red"})
    message = str(info.value)
    assert "sdie" in message
    assert "'side'" in message            # 拼写建议


def test_platform_attribute_range_is_checked(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    factory, _, _ = rig
    with pytest.raises(ConfigurationError, match="不能小于"):
        factory.resolved_platform_params("ARMOR_MBT", {"crew": -1})


def test_platform_attribute_type_is_checked(
    rig: tuple[ComponentFactory, TypeRegistry, PlatformRegistry]
) -> None:
    """``decision`` 是字符串：写个数进去要报错，不能悄悄转成 "3"。"""
    factory, _, _ = rig
    with pytest.raises(ConfigurationError):
        factory.resolved_platform_params("TANK", {"decision": 3})


def test_bad_declaration_is_reported_by_validate_class() -> None:
    """平台类自己把 ``PARAMS`` 写错（值不是 ``Params.*``）也要能查出来。"""

    class Broken(Platform):
        PARAMS = {"side": "red"}          # 裸字符串，不是 ParamSpec

    platforms = PlatformRegistry()
    platforms.register("BROKEN", Broken)
    problems = platforms.validate_class("BROKEN")
    assert problems and "Params.*" in problems[0]
    assert platforms.validate_class("没有这个") == ["未注册的平台名 '没有这个'"]


# ---------------------------------------------------------------------------
# 平台层只留参数
# ---------------------------------------------------------------------------

def test_platform_has_params_but_no_behaviour() -> None:
    """平台类**不含行为**：没有生命周期钩子、没有状态。

    一旦平台自己会动，"这辆车能跑多快"就有两处真相（平台属性一份、机动件
    参数一份），而两份数字迟早不同步。
    """
    assert not hasattr(Platform, "update")
    assert not hasattr(Platform, "initialize")
    assert not hasattr(Platform, "shutdown")
    assert not getattr(Platform, "PARAMS", {}).get("max_speed")


def test_every_declared_platform_key_is_a_param_spec() -> None:
    for name, spec in Platform.PARAMS.items():
        assert isinstance(spec, ParamSpec), f"{name} 不是 Params.* 的返回值"


def test_the_platform_table_only_holds_keys_someone_reads() -> None:
    """**只加"有人读"的参数。**

    三个键各有消费者（都写在这里，因为这条测试逼的就是这个）：

    * ``side`` —— 实体建档时取用；
    * ``decision`` —— 决策接线取用；
    * ``radar_cross_section`` —— **感知层**：雷达的第 5~7 步要拿目标的 RCS 去算
      回波功率（``RadarSensor._rcs_of`` 走 ``mount.target_platform_param``）。
      呼号、编制等仍然没有消费者，等就位再加。

    这条测试**故意**会在加键时失败：它逼着加键的人回答"谁读这个值"。
    §9 第 15 条要消灭的是"写了没人读、拼错也不报错"的平台属性，不是要换
    一张更大的空表。
    """
    assert set(Platform.PARAMS) == {"side", "decision", "radar_cross_section"}


# ---------------------------------------------------------------------------
# 想定层：平台类型也进校验
# ---------------------------------------------------------------------------

GOOD = """
zone AO
    anchor     39.9042 116.4074
    radius     20 km
    resolution 500 m
end_zone

platform_type P
    side red
end_platform_type

platform U1 P
    position latlng 39.95 116.40
end_platform
"""


def scenario_with_platform_block(body: str) -> str:
    return f"""
zone AO
    anchor     39.9042 116.4074
    radius     20 km
    resolution 500 m
end_zone

platform_type P
{body}
end_platform_type

platform U1 P
    position latlng 39.95 116.40
end_platform
"""


def test_a_clean_scenario_validates() -> None:
    assert Simulation.from_scenario(GOOD).validate() == []


def test_scenario_reports_a_typo_in_platform_attributes() -> None:
    """平台属性写错要在**想定检查**阶段报出来，而不是等建档时才炸。"""
    problems = " ".join(Simulation.from_scenario(scenario_with_platform_block("    sdie red")).validate())
    assert "P" in problems and "sdie" in problems
    assert "side" in problems          # 拼写建议一路透传到想定层


def test_scenario_reports_platform_decision_that_does_not_exist() -> None:
    """平台类型上的 ``decision`` 也要指向真实存在的配置。

    漏掉这条的话 ``effective_decision`` 会**静默**退回默认配置——想定作者
    以为改成了大模型，实际还在跑规则。
    """
    text = scenario_with_platform_block("    decision NO_SUCH_PROFILE")
    problems = " ".join(Simulation.from_scenario(text).validate())
    assert "NO_SUCH_PROFILE" in problems


def test_scenario_reports_bad_params_of_unused_platform_types() -> None:
    """定义了但**没实例化**的平台类型也要查。

    一份想定里可能有十个平台类型而只实例化三个，另外七个里的错值会静静
    躺着，等哪天有人开始用才炸。
    """
    text = GOOD + """
platform_type UNUSED
    sdie blue
end_platform_type
"""
    problems = " ".join(Simulation.from_scenario(text).validate())
    assert "UNUSED" in problems and "sdie" in problems


def test_platform_side_reaches_the_entity_registry() -> None:
    """``side`` 的消费者是实体建档——这条把"参数真的被读"钉住。

    测试断了的话，平台属性表就只剩"能被校验"这一个价值，而它本来是要
    真的影响装配的。
    """
    sim = Simulation.from_scenario(GOOD)
    sim.build()
    entity_id = sim.registry.by_name("U1").entity_id
    assert sim.platform_params[entity_id]["side"] == "red"
    assert sim.registry.side_of(entity_id) == "red"


def test_platform_type_inheritance_in_a_scenario() -> None:
    """派生平台类型继承父类型的属性，子类型写的覆盖父类型。"""
    text = """
zone AO
    anchor     39.9042 116.4074
    radius     20 km
    resolution 500 m
end_zone

platform_type BASE
    side red
end_platform_type

platform_type DERIVED : BASE
    side blue
end_platform_type

platform U1 BASE
    position latlng 39.95 116.40
end_platform
"""
    sim = Simulation.from_scenario(text)
    sim.build()
    # 想定层只存"写了什么"（差量）——所以 side 是子类型覆盖后的值，
    # 而 decision 根本不在里面：它来自代码层的类默认值。
    assert sim.spec.types.resolve_platform("DERIVED").attributes == {"side": "blue"}
    resolved = sim.factory.resolved_platform_params("DERIVED")
    assert resolved["side"] == "blue"
    assert resolved["decision"] == ""
