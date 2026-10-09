"""类型系统测试：继承合并、组件注册、三层参数覆盖。

重点在两处：

1. **参数覆盖是整体替换不是合并**——合并语义会让"父类加了个元素、
   所有子类行为都变了"这种事极难追查。
2. **找不到实现类要明确报错**——不回退、不静默，并且报错里带上继承链
   和拼写建议。
"""

from __future__ import annotations

import pytest

from milsim.services.params import (
    ParamError,
    Params,
    UnknownParameterError,
    with_unit,
)
from milsim.services.type_registry import (
    ComponentFactory,
    ComponentRegistry,
    ComponentSpec,
    DuplicateComponentError,
    DuplicateTypeError,
    InheritanceCycleError,
    TypeDefinitionError,
    TypeRegistry,
    UnknownComponentError,
    UnknownTypeError,
    register_component,
    reset_default_registry,
)


def _reinstall_framework() -> None:
    """把框架参考实现装回**全局**注册表。

    ``reset_default_registry()`` 清的是全局表的内容——测试隔离的前提是
    "测完把现场恢复"，否则字母序排在本文件之后的组件测试（它们 import
    时注册的类已经被清掉）会拿到一张空表。只恢复组件侧：本文件的用例
    不碰平台表。
    """
    from milsim import models
    from milsim.services.type_registry import default_registry

    models.register_framework(components=default_registry())


# ---------------------------------------------------------------------------
# 定义
# ---------------------------------------------------------------------------

def test_define_and_fetch() -> None:
    registry = TypeRegistry()
    registry.define_component_type("RADAR_BASE", attrs={"scan_interval": 2_000_000})
    entry = registry.component_type("RADAR_BASE")
    assert entry.name == "RADAR_BASE"
    assert entry.parent is None
    assert registry.component_names() == ["RADAR_BASE"]


def test_duplicate_component_type_rejected() -> None:
    """重名会静默覆盖，所以直接报错——并指出第一次定义在哪一行。"""
    registry = TypeRegistry()
    registry.define_component_type("A", line=10)
    with pytest.raises(DuplicateTypeError) as info:
        registry.define_component_type("A", line=40)
    assert "10" in str(info.value)


def test_duplicate_platform_type_rejected() -> None:
    registry = TypeRegistry()
    registry.define_platform_type("SAM")
    with pytest.raises(DuplicateTypeError):
        registry.define_platform_type("SAM")


@pytest.mark.parametrize("bad", ["", "   ", "A B", "A:B", "A,B", "A#B"])
def test_invalid_type_names_rejected(bad: str) -> None:
    """名字里的空白和词法字符会破坏块状文本解析，在定义处就拦住。"""
    registry = TypeRegistry()
    with pytest.raises(TypeDefinitionError):
        registry.define_component_type(bad)


def test_unknown_type_lists_available() -> None:
    registry = TypeRegistry()
    registry.define_component_type("RADAR_BASE")
    with pytest.raises(UnknownTypeError) as info:
        registry.component_type("RADAR_BSE")
    message = str(info.value)
    assert "RADAR_BASE" in message          # 可用名单
    assert "是否想写" in message             # 拼写建议


def test_unknown_type_when_registry_empty() -> None:
    with pytest.raises(UnknownTypeError, match="一个组件类型都没有"):
        TypeRegistry().component_type("ANY")


# ---------------------------------------------------------------------------
# 继承链
# ---------------------------------------------------------------------------

def test_chain_is_leaf_first() -> None:
    registry = TypeRegistry()
    registry.define_component_type("A")
    registry.define_component_type("B", parent="A")
    registry.define_component_type("C", parent="B")
    assert registry.component_chain("C") == ["C", "B", "A"]


def test_chain_detects_cycle() -> None:
    """成环必须报错，并把环画出来——否则作者要在几十行定义里自己找。"""
    registry = TypeRegistry()
    registry.define_component_type("A", parent="B")
    registry.define_component_type("B", parent="A")
    with pytest.raises(InheritanceCycleError) as info:
        registry.component_chain("A")
    message = str(info.value)
    assert "A" in message and "B" in message and "→" in message


def test_chain_detects_self_inheritance() -> None:
    registry = TypeRegistry()
    registry.define_component_type("A", parent="A")
    with pytest.raises(InheritanceCycleError):
        registry.component_chain("A")


def test_chain_reports_missing_parent() -> None:
    registry = TypeRegistry()
    registry.define_component_type("A", parent="NOPE")
    with pytest.raises(UnknownTypeError) as info:
        registry.component_chain("A")
    assert "NOPE" in str(info.value)


def test_forward_reference_is_allowed() -> None:
    """先写子类、后写父类必须能通过——这是设计时就定下的要求。"""
    registry = TypeRegistry()
    registry.define_component_type("CHILD", parent="PARENT", attrs={"a": 1})
    registry.define_component_type("PARENT", attrs={"b": 2})
    chain = registry.component_chain("CHILD")
    assert chain == ["CHILD", "PARENT"]
    assert registry.resolve_component_attrs("CHILD") == {"a": 1, "b": 2}


# ---------------------------------------------------------------------------
# 参数合并
# ---------------------------------------------------------------------------

def test_leaf_overrides_root() -> None:
    registry = TypeRegistry()
    registry.define_component_type("BASE", attrs={"scan_interval": 2_000_000, "range": 40_000.0})
    registry.define_component_type(
        "CHILD", parent="BASE", attrs={"scan_interval": 3_000_000}
    )
    merged = registry.resolve_component_attrs("CHILD")
    assert merged["scan_interval"] == 3_000_000
    assert merged["range"] == 40_000.0        # 未覆盖的继承下来


def test_list_override_replaces_entirely() -> None:
    """列表覆盖是**整体替换**，不是合并。

    合并语义看似聪明，实际会让"父类加了个元素、所有子类行为都变了"
    这种事极难追查。
    """
    registry = TypeRegistry()
    registry.define_component_type("BASE", attrs={"modes": ("GUN",)})
    registry.define_component_type(
        "CHILD", parent="BASE", attrs={"modes": ("SARH", "IR")}
    )
    assert registry.resolve_component_attrs("CHILD")["modes"] == ("SARH", "IR")


def test_three_level_merge() -> None:
    registry = TypeRegistry()
    registry.define_component_type("A", attrs={"x": 1, "y": 1, "z": 1})
    registry.define_component_type("B", parent="A", attrs={"y": 2, "z": 2})
    registry.define_component_type("C", parent="B", attrs={"z": 3})
    assert registry.resolve_component_attrs("C") == {"x": 1, "y": 2, "z": 3}


def test_resolve_returns_copy() -> None:
    """返回副本——调用方改了不该影响缓存。"""
    registry = TypeRegistry()
    registry.define_component_type("A", attrs={"x": 1})
    first = registry.resolve_component_attrs("A")
    first["x"] = 999
    assert registry.resolve_component_attrs("A")["x"] == 1


def test_cache_invalidated_by_new_definition() -> None:
    """定义了新类型之后，之前解析过的结果必须重算。

    漏掉失效的话会表现为"改了想定却没生效"——最让人怀疑自己眼睛的一类 bug。
    """
    registry = TypeRegistry()
    registry.define_component_type("A", attrs={"x": 1})
    assert registry.resolve_component_attrs("A") == {"x": 1}

    registry.define_component_type("B", parent="A", attrs={"x": 2})
    assert registry.resolve_component_attrs("B") == {"x": 2}

    # 再给 A 补一个参数，B 的解析结果必须跟着变
    registry._components["A"].attrs["y"] = 7
    registry.clear_cache()
    assert registry.resolve_component_attrs("B") == {"x": 2, "y": 7}


def test_add_component_invalidates_cache() -> None:
    registry = TypeRegistry()
    registry.define_platform_type("P")
    registry.resolve_platform("P")
    registry.add_component("P", ComponentSpec("sensor", "RADAR"))
    assert "sensor" in registry.resolve_platform("P").components


# ---------------------------------------------------------------------------
# 平台类型
# ---------------------------------------------------------------------------

def test_platform_attributes_inherit() -> None:
    registry = TypeRegistry()
    registry.define_platform_type("BASE_UNIT", attributes={"side": "red", "max_speed": 15.0})
    registry.define_platform_type("SAM", parent="BASE_UNIT", attributes={"max_speed": 20.0})
    resolved = registry.resolve_platform("SAM")
    assert resolved.attributes == {"side": "red", "max_speed": 20.0}


def test_platform_components_inherit_by_slot() -> None:
    registry = TypeRegistry()
    registry.define_platform_type("BASE_UNIT")
    registry.add_component("BASE_UNIT", ComponentSpec("mover", "GROUND_MOVER"))
    registry.add_component("BASE_UNIT", ComponentSpec("comm", "RADIO"))

    registry.define_platform_type("SAM", parent="BASE_UNIT")
    registry.add_component("SAM", ComponentSpec("mover", "FAST_MOVER"))

    resolved = registry.resolve_platform("SAM")
    assert resolved.components["mover"].type_name == "FAST_MOVER"   # 覆盖
    assert resolved.components["comm"].type_name == "RADIO"         # 继承
    assert resolved.slots() == ["comm", "mover"]


def test_duplicate_slot_in_same_platform_rejected() -> None:
    """同一平台块里同一槽写两次是笔误，不是"挂两个组件"。"""
    registry = TypeRegistry()
    registry.define_platform_type("SAM")
    registry.add_component("SAM", ComponentSpec("sensor", "RADAR_A", line=20))
    with pytest.raises(DuplicateTypeError) as info:
        registry.add_component("SAM", ComponentSpec("sensor", "RADAR_B", line=25))
    message = str(info.value)
    assert "RADAR_A" in message and "RADAR_B" in message


def test_component_spec_rejects_empty_fields() -> None:
    with pytest.raises(Exception):
        ComponentSpec("", "RADAR")
    with pytest.raises(Exception):
        ComponentSpec("sensor", "")


def test_validate_clean_registry() -> None:
    registry = TypeRegistry()
    registry.define_component_type("A")
    registry.define_platform_type("P")
    registry.add_component("P", ComponentSpec("sensor", "A"))
    assert registry.validate() == []


def test_validate_reports_cycle() -> None:
    registry = TypeRegistry()
    registry.define_component_type("A", parent="B")
    registry.define_component_type("B", parent="A")
    problems = registry.validate()
    assert any("成环" in p for p in problems)


# ---------------------------------------------------------------------------
# 组件类注册
# ---------------------------------------------------------------------------

class FakeSensor:
    """最小组件 stub——只满足"能用 spec 构造"这一条约定。"""

    PARAMS = {
        "detect_range": Params.distance(40_000.0),
        "scan_interval": Params.duration(2_000_000),
    }

    def __init__(self, *, spec) -> None:
        self.spec = spec


class FastSensor(FakeSensor):
    PARAMS = {
        **FakeSensor.PARAMS,
        "scan_interval": Params.duration(500_000),
        "eccm_level": Params.integer(0),
    }


def test_registry_register_and_get() -> None:
    registry = ComponentRegistry()
    registry.register("FAKE", FakeSensor)
    assert registry.get("FAKE") is FakeSensor
    assert registry.names() == ["FAKE"]
    assert "FAKE" in registry


def test_duplicate_registration_rejected() -> None:
    """两个模块注册同名组件时静默覆盖，会表现为"我的实现改了却不起作用"。"""
    registry = ComponentRegistry()
    registry.register("FAKE", FakeSensor)
    with pytest.raises(DuplicateComponentError) as info:
        registry.register("FAKE", FastSensor)
    assert "override=True" in str(info.value)


def test_explicit_override_is_allowed() -> None:
    registry = ComponentRegistry()
    registry.register("FAKE", FakeSensor)
    registry.register("FAKE", FastSensor, override=True)
    assert registry.get("FAKE") is FastSensor


def test_resolve_for_walks_up_the_chain() -> None:
    """想定层可以有代码里不存在的纯参数模板，实现沿链向上找。"""
    registry = ComponentRegistry()
    registry.register("FAKE", FakeSensor)
    found = registry.resolve_for(["MY_RADAR", "FAKE"])
    assert found is FakeSensor


def test_resolve_for_reports_chain_and_names() -> None:
    registry = ComponentRegistry()
    registry.register("FAKE", FakeSensor)
    with pytest.raises(UnknownComponentError) as info:
        registry.resolve_for(["NOPE", "ALSO_NOPE"])
    message = str(info.value)
    assert "NOPE" in message
    assert "FAKE" in message           # 已注册名单
    assert "→" in message              # 继承链


def test_resolve_for_suggests_close_name() -> None:
    registry = ComponentRegistry()
    registry.register("HEX_SEARCH_RADAR", FakeSensor)
    with pytest.raises(UnknownComponentError, match="是否想写"):
        registry.resolve_for(["HEX_SEARCH_RADER"])


def test_validate_class_checks_params() -> None:
    registry = ComponentRegistry()
    registry.register("FAKE", FakeSensor)
    assert registry.validate_class("FAKE") == []

    class Broken(FakeSensor):
        PARAMS = {"bad": "not a spec"}     # type: ignore[dict-item]

    registry.register("BROKEN", Broken)
    assert registry.validate_class("BROKEN") != []


def test_register_component_decorator_uses_default_registry() -> None:
    reset_default_registry()

    @register_component("TEST_RADAR")
    class TestRadar(FakeSensor):
        pass

    from milsim.services.type_registry import default_registry

    assert default_registry().get("TEST_RADAR") is TestRadar
    assert TestRadar.COMPONENT_NAME == "TEST_RADAR"      # 便于调试
    reset_default_registry()
    _reinstall_framework()


def test_decorator_returns_class_unchanged() -> None:
    """装饰器必须原样返回类——包装成别的对象会让继承它的代码拿到怪东西。"""
    reset_default_registry()

    @register_component("TEST_AGAIN")
    class Sub(FakeSensor):
        def detect(self) -> str:
            return "ok"

    assert issubclass(Sub, FakeSensor)
    assert Sub.__name__ == "Sub"
    assert "detect" in Sub.__dict__              # 方法还挂在原类上，没被搬走
    reset_default_registry()
    _reinstall_framework()


# ---------------------------------------------------------------------------
# 工厂：三层参数覆盖
# ---------------------------------------------------------------------------

def make_factory() -> ComponentFactory:
    components = ComponentRegistry()
    components.register("HEX_SEARCH_RADAR", FakeSensor)
    components.register("PHASED_ARRAY_RADAR", FastSensor)

    types = TypeRegistry()
    types.define_component_type("RADAR_BASE", attrs={"scan_interval": 3_000_000})
    types.define_component_type("HEX_SEARCH_RADAR", parent="RADAR_BASE")
    return ComponentFactory(components, types)


def test_build_uses_class_defaults() -> None:
    factory = make_factory()
    component = factory.build("sensor", "PHASED_ARRAY_RADAR")
    assert component.spec.params["detect_range"] == 40_000.0
    assert component.spec.params["eccm_level"] == 0


def test_build_applies_type_chain_values() -> None:
    factory = make_factory()
    component = factory.build("sensor", "HEX_SEARCH_RADAR")
    # RADAR_BASE 里写了 scan_interval 3 s，覆盖类的 2 s
    assert component.spec.params["scan_interval"] == 3_000_000


def test_build_applies_mount_overrides_last() -> None:
    """挂载差量优先级最高——同一型号雷达在不同平台上微调，靠的就是这一层。"""
    factory = make_factory()
    component = factory.build(
        "sensor",
        "HEX_SEARCH_RADAR",
        {"detect_range": with_unit(60, "km"), "scan_interval": with_unit(5, "s")},
    )
    assert component.spec.params["detect_range"] == pytest.approx(60_000.0)
    assert component.spec.params["scan_interval"] == 5_000_000


def test_build_without_type_block_is_allowed() -> None:
    """只注册了类、没写 component_type 块也能构造——参数全走类默认值。"""
    factory = make_factory()
    component = factory.build("sensor", "PHASED_ARRAY_RADAR")
    assert component.spec.params["scan_interval"] == 500_000


def test_build_parameter_template_without_class() -> None:
    """想定层定义新型号名、代码层只有父类实现——参数用新型号的。"""
    factory = make_factory()
    factory.types.define_component_type(
        "MY_CUSTOM_RADAR", parent="HEX_SEARCH_RADAR", attrs={"detect_range": 80_000.0}
    )
    component = factory.build("sensor", "MY_CUSTOM_RADAR")
    assert component.spec.impl_class is FakeSensor
    assert component.spec.type_name == "MY_CUSTOM_RADAR"
    assert component.spec.params["detect_range"] == 80_000.0


def test_build_rejects_unknown_parameter() -> None:
    """挂载时写错参数名必须报错，且报错里带槽名与类型名。"""
    factory = make_factory()
    with pytest.raises(UnknownParameterError) as info:
        factory.build("sensor", "HEX_SEARCH_RADAR", {"detect_rang": 1.0})
    assert "detect_range" in str(info.value)


def test_build_rejects_unknown_type() -> None:
    factory = make_factory()
    with pytest.raises(UnknownComponentError):
        factory.build("sensor", "NO_SUCH_RADAR")


def test_resolved_component_accessors() -> None:
    factory = make_factory()
    spec = factory.build("sensor", "PHASED_ARRAY_RADAR").spec
    assert spec["detect_range"] == 40_000.0
    assert spec.get("missing", "default") == "default"
    assert "PHASED_ARRAY_RADAR" in repr(spec)


def test_param_set_exposes_schema_for_documentation() -> None:
    factory = make_factory()
    schema = factory.param_set("PHASED_ARRAY_RADAR").schema()
    assert "detect_range" in schema
    assert "eccm_level" in schema


def test_build_for_platform_instantiates_every_slot() -> None:
    factory = make_factory()
    factory.types.define_platform_type("SAM", attributes={"side": "red"})
    factory.types.add_component("SAM", ComponentSpec("sensor", "HEX_SEARCH_RADAR"))
    factory.types.add_component("SAM", ComponentSpec("mover", "PHASED_ARRAY_RADAR"))

    built = factory.build_for_platform("SAM")
    assert sorted(built) == ["mover", "sensor"]
    assert built["sensor"].spec.slot == "sensor"
    assert built["mover"].spec.type_name == "PHASED_ARRAY_RADAR"


def test_build_for_platform_reports_missing_implementation() -> None:
    """某个槽构造失败时要指出是哪个槽，而不是一句"平台实例化失败"。"""
    factory = make_factory()
    factory.types.define_platform_type("BAD")
    factory.types.add_component("BAD", ComponentSpec("sensor", "NO_SUCH"))
    with pytest.raises(UnknownComponentError):
        factory.build_for_platform("BAD")


def test_build_for_platform_reports_parameter_error_with_slot_name() -> None:
    factory = make_factory()
    factory.types.define_platform_type("BAD")
    factory.types.add_component(
        "BAD", ComponentSpec("sensor", "HEX_SEARCH_RADAR", {"detect_rang": 1})
    )
    with pytest.raises(ParamError) as info:
        factory.build_for_platform("BAD")
    assert "sensor" in str(info.value)


def test_factory_without_arguments_uses_default_registry() -> None:
    """不传参时用全局注册表，类型表初始为空——想定层的东西由装配层填。"""
    factory = ComponentFactory()
    assert factory.types.component_names() == []
    assert factory.types.platform_names() == []


# ---------------------------------------------------------------------------
# 集成：使用者继承框架组件的完整流程
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _clean_default_registry():
    """每条测试前后都清空全局注册表，避免互相污染。

    ★ 结尾清空之后要把框架参考实现装回去：清空的代价不能转嫁给
    字母序排在本文件后面的测试（它们的类是在 import 时注册进全局
    表的，清一次就全没了）。
    """
    reset_default_registry()
    yield
    reset_default_registry()
    _reinstall_framework()


def test_user_extension_workflow_end_to_end() -> None:
    """框架给参考实现，使用者继承它扩展——从头到尾走一遍。

    这条测试验的是整个 M3c-3 的设计目标：**使用者不必改框架，
    只要继承参考实现、合并 PARAMS、覆盖需要改的方法。**
    """

    # ---- 框架侧：抽象基类 ----
    class Sensor:
        PARAMS: dict = {}

        def __init__(self, *, spec) -> None:
            self.spec = spec

        def detect(self, target_id: int) -> float:
            raise NotImplementedError

    # ---- 框架侧：参考实现（想定里可直接用）----
    @register_component("HEX_SEARCH_RADAR")
    class HexSearchRadar(Sensor):
        PARAMS = {
            "detect_range": Params.distance(40_000.0),
            "scan_interval": Params.duration(2_000_000),
        }

        def detect(self, target_id: int) -> float:
            return min(1.0, self.spec["detect_range"] / 100_000.0)

    # ---- 使用者侧：继承参考实现，扩展抗干扰等级 ----
    @register_component("PHASED_ARRAY_RADAR")
    class PhasedArrayRadar(HexSearchRadar):
        PARAMS = {
            **HexSearchRadar.PARAMS,                      # 继承全部参数
            "eccm_level": Params.integer(0, minimum=0, maximum=10),
            "scan_interval": Params.duration(500_000),    # 覆盖
        }

        def detect(self, target_id: int) -> float:
            base = super().detect(target_id)              # 复用父类算法
            return min(1.0, base * (1.0 + 0.1 * self.spec["eccm_level"]))

    # ---- 想定层：定义型号与平台 ----
    factory = ComponentFactory()
    types = factory.types
    types.define_component_type("RADAR_BASE", attrs={"scan_interval": 3_000_000})
    types.define_component_type("PHASED_ARRAY_RADAR", parent="RADAR_BASE")

    types.define_platform_type("BASE_UNIT", attributes={"side": "red"})
    types.define_platform_type("SAM_BATTALION", parent="BASE_UNIT")
    types.add_component(
        "SAM_BATTALION",
        ComponentSpec(
            "sensor",
            "PHASED_ARRAY_RADAR",
            {"detect_range": with_unit(60, "km"), "eccm_level": 4},
        ),
    )

    # ---- 装配 ----
    built = factory.build_for_platform("SAM_BATTALION")
    radar = built["sensor"]

    # 三层覆盖：类默认值 ← 类型链 ← 挂载差量
    assert radar.spec["detect_range"] == pytest.approx(60_000.0)     # 挂载层
    assert radar.spec["scan_interval"] == 3_000_000                 # 类型链（RADAR_BASE）
    assert radar.spec["eccm_level"] == 4                            # 挂载层

    # 行为扩展生效：抗干扰等级抬高了探测概率
    plain = HexSearchRadar(spec=radar.spec)
    assert radar.detect(1) > plain.detect(1)

    # 平台属性沿继承链合并
    resolved = types.resolve_platform("SAM_BATTALION")
    assert resolved.attributes["side"] == "red"

    # ---- 使用者把参数写错了，报错要能直接定位 ----
    with pytest.raises(UnknownParameterError, match="eccm_levl"):
        factory.build("sensor", "PHASED_ARRAY_RADAR", {"eccm_levl": 3})

    # ---- 使用者把值写超范围了，也要拦住 ----
    with pytest.raises(ParamError, match="不能大于"):
        factory.build("sensor", "PHASED_ARRAY_RADAR", {"eccm_level": 99})

    # ---- 整个注册表自检干净 ----
    assert types.validate() == []
    assert factory.components.validate_class("PHASED_ARRAY_RADAR") == []
