"""异常层次的测试。

这组测试守的是一条边界：**配置错误**（用户改想定就能解决）与**代码缺陷**
（要改框架）必须走不同出口。前者给一句人话，后者让栈回溯露出来。

混在一起的两个后果都很糟：把代码缺陷吞掉变成"看起来跑完了但结果不对"，
或者把配置错误抛成栈回溯让使用者以为框架坏了。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError, MilsimError
from milsim.services.input.lexer import LexError
from milsim.services.input.parser import ParseError
from milsim.services.params import CoordValue, ParamError
from milsim.services.scenario import ScenarioError, load_scenario
from milsim.services.type_registry import TypeDefinitionError
from milsim.simulation import SimulationError


def _token():
    from milsim.services.input.lexer import Token, TokenKind

    return Token(TokenKind.NAME, "x", "x", 1, 1)


#: 想定/配置层面的异常。工具脚本捕 `MilsimError` 就能全部接住。
CONFIG_ERRORS = [
    LexError("x", 1, 1),
    ParseError("x", _token()),
    ScenarioError("x", 1),
    ParamError("x"),
    TypeDefinitionError("x"),
]


# ---------------------------------------------------------------------------
# 层次
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("error", CONFIG_ERRORS, ids=lambda e: type(e).__name__)
def test_configuration_errors_share_base(error: Exception) -> None:
    assert isinstance(error, MilsimError)
    assert isinstance(error, ConfigurationError)


@pytest.mark.parametrize("error", CONFIG_ERRORS, ids=lambda e: type(e).__name__)
def test_they_remain_value_errors(error: Exception) -> None:
    """历史代码与测试里有 `except ValueError`，改成单一继承会静默失效。"""
    assert isinstance(error, ValueError)


def test_simulation_error_is_milsim_but_not_configuration() -> None:
    """装配阶段的错误未必是"配置写错"——也可能是调用顺序不对。"""
    error = SimulationError("x")
    assert isinstance(error, MilsimError)
    assert isinstance(error, RuntimeError)
    assert not isinstance(error, ConfigurationError)


def test_code_defects_are_not_swallowed() -> None:
    """捕 MilsimError 不该顺手把代码缺陷也吞掉。

    这是刻意留的边界：`TypeError`、`AttributeError` 这类必须冒出来。
    """
    for defect in (TypeError("x"), AttributeError("x"), KeyError("x")):
        assert not isinstance(defect, MilsimError)


def test_catching_base_handles_every_stage() -> None:
    """解析、语义、参数三层各自的错误都能被同一个 except 接住。"""
    samples = [
        ("zone A $\nend_zone\n", LexError),           # 词法
        ("zone A\n", ParseError),                     # 语法
        ("zone A\n    radius 5 m\nend_zone\n", ScenarioError),   # 语义
    ]
    for text, expected in samples:
        with pytest.raises(MilsimError) as info:
            load_scenario(text)
        assert type(info.value) is expected


# ---------------------------------------------------------------------------
# 坐标范围：两条构造路径都要覆盖
# ---------------------------------------------------------------------------

def test_coord_value_rejects_out_of_range_latitude() -> None:
    with pytest.raises(ParamError, match="纬度"):
        CoordValue("latlng", lat=95.0, lng=116.0)


def test_coord_value_rejects_out_of_range_longitude() -> None:
    with pytest.raises(ParamError, match="经度"):
        CoordValue("latlng", lat=39.0, lng=200.0)


def test_coord_value_accepts_boundaries() -> None:
    assert CoordValue("latlng", lat=90.0, lng=180.0).lat == 90.0
    assert CoordValue("latlng", lat=-90.0, lng=-180.0).lng == -180.0


def test_latlng_with_zone_is_rejected() -> None:
    with pytest.raises(ParamError, match="不该带战区"):
        CoordValue("latlng", lat=39.0, lng=116.0, zone="A")


def test_anchor_pair_path_also_checks_range() -> None:
    """`anchor 95.0 116.4` 走的是裸数对分支，不经过解析器的坐标检查。

    范围校验放在值类型自己身上，两条构造路径就都覆盖了——只放解析器里
    会漏掉这一条（实测漏过一次）。
    """
    with pytest.raises(MilsimError, match="纬度"):
        load_scenario("zone A\n    anchor 95.0 116.4\nend_zone\n")


# ---------------------------------------------------------------------------
# validate 补的两项
# ---------------------------------------------------------------------------

def test_validate_reports_inheritance_cycle() -> None:
    """继承链在实例化时才解析，但校验阶段必须能发现成环。

    否则一个成环的类型定义会静静躺着，等第一次实例化时才炸——
    而那时已经离写错的地方很远了。
    """
    text = (
        "component_type A : B\nend_component_type\n"
        "component_type B : A\nend_component_type\n"
    )
    spec = load_scenario(text)
    assert any("成环" in p for p in spec.validate())


def test_validate_reports_missing_parent_type() -> None:
    text = "component_type A : NO_SUCH\nend_component_type\n"
    spec = load_scenario(text)
    assert any("NO_SUCH" in p for p in spec.validate())


def test_validate_reports_duplicate_platform_name() -> None:
    text = (
        "platform_type P\nend_platform_type\n"
        "platform A P\n    position latlng 39.9 116.4\nend_platform\n"
        "platform A P\n    position latlng 39.8 116.3\nend_platform\n"
    )
    spec = load_scenario(text)
    problems = spec.validate()
    assert any("定义了 2 次" in p for p in problems)


def test_duplicate_platform_also_fails_at_build() -> None:
    """校验能发现，装配时也会真的报错——两层都要拦。"""
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    text = (
        "zone AO\n    anchor 39.9042 116.4074\n    radius 20 km\nend_zone\n"
        "platform_type P\nend_platform_type\n"
        "platform A P\n    position latlng 39.9 116.4\nend_platform\n"
        "platform A P\n    position latlng 39.8 116.3\nend_platform\n"
    )
    sim = Simulation.from_scenario(text, components=ComponentRegistry())
    with pytest.raises(MilsimError, match="已被占用"):
        sim.build()


def test_clean_scenario_has_no_problems() -> None:
    text = (
        "zone AO\n    anchor 39.9042 116.4074\n    radius 20 km\nend_zone\n"
        "component_type A\nend_component_type\n"
        "platform_type P\nend_platform_type\n"
        "platform X P\n    position latlng 39.9 116.4\nend_platform\n"
    )
    assert load_scenario(text).validate() == []
