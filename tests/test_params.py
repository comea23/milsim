"""参数模式与单位系统的测试。

单位换算是这套框架里最容易出静默错误的地方——错了不会报错，只会让
探测距离、通视范围、时间步长全部偏差一个数量级，而且看上去"很正常"。
所以这部分测试最密。
"""

from __future__ import annotations

import pytest

from milsim.services.params import (
    BASE_UNIT,
    Dimension,
    MissingParameterError,
    ParamError,
    ParamSet,
    Params,
    UnknownParameterError,
    WithUnit,
    _UNITS,
    known_units,
    with_unit,
)


# ---------------------------------------------------------------------------
# 量纲与单位换算
# ---------------------------------------------------------------------------

def test_distance_units() -> None:
    spec = Params.distance(0.0)
    assert spec.convert(40, "km") == pytest.approx(40_000.0)
    assert spec.convert(40, "m") == pytest.approx(40.0)
    assert spec.convert(10, "nm") == pytest.approx(18_520.0)
    assert spec.convert(1, "mi") == pytest.approx(1609.344)


def test_time_units_are_exact_integers() -> None:
    """时间必须走整数乘法，不经过浮点。

    `2 s` 要直接得到 2_000_000，而不是 `int(2.0 * 1e6)` 那种先算成秒
    再乘回来的路径——那正是引擎层好不容易消灭的浮点误差的来源。
    """
    spec = Params.duration(0)
    assert spec.convert(2, "s") == 2_000_000
    assert spec.convert(500, "ms") == 500_000
    assert spec.convert(5, "min") == 300_000_000
    assert spec.convert(1, "h") == 3_600_000_000
    assert isinstance(spec.convert(2, "s"), int)


def test_time_unit_factors_must_stay_integers() -> None:
    """防回归：有人把 1_000_000 改成 1e6 就会打破上一条测试的保证。"""
    for unit, factor in _UNITS[Dimension.TIME].items():
        assert isinstance(factor, int), f"{unit} 的换算因子必须是 int"


def test_fractional_seconds_round_exactly() -> None:
    """`0.1 s` 在浮点下是 99999.999...，必须 round 成 100000 而不是截断。"""
    spec = Params.duration(0)
    assert spec.convert(0.1, "s") == 100_000
    assert spec.convert(0.001, "s") == 1_000
    assert spec.convert(2.5, "s") == 2_500_000


def test_angle_units() -> None:
    spec = Params.angle(0.0)
    assert spec.convert(45, "deg") == pytest.approx(45.0)
    assert spec.convert(3.14159265358979, "rad") == pytest.approx(180.0, abs=1e-6)


def test_speed_units() -> None:
    spec = Params.speed(0.0)
    assert spec.convert(36, "km/h") == pytest.approx(10.0)
    assert spec.convert(1, "kn") == pytest.approx(0.514444)
    assert spec.convert(250, "m/s") == pytest.approx(250.0)


def test_unitless_value_uses_base_unit() -> None:
    """无单位的值按**基本单位**处理，不做静默猜测。

    `detect_range 40` 是 40 米，不是 40 千米。猜错一次就是几万公里偏差。
    """
    assert Params.distance(0.0).convert(40, None) == pytest.approx(40.0)
    assert Params.duration(0).convert(40, None) == 40
    assert Params.angle(0.0).convert(40, None) == pytest.approx(40.0)


def test_base_units_are_declared() -> None:
    assert BASE_UNIT[Dimension.DISTANCE] == "m"
    assert BASE_UNIT[Dimension.TIME] == "us"
    assert BASE_UNIT[Dimension.ANGLE] == "deg"
    assert BASE_UNIT[Dimension.SPEED] == "m/s"


def test_unknown_unit_error_lists_options() -> None:
    """单位写错必须报错，并把可用单位列出来——比一句"未知单位"有用得多。"""
    with pytest.raises(ParamError) as info:
        Params.distance(0.0).convert(40, "kmm")
    message = str(info.value)
    assert "kmm" in message
    assert "km" in message
    assert "m" in message


def test_unit_on_unitless_parameter_is_rejected() -> None:
    """纯数值参数不该接受单位——`track_capacity 40 km` 是明显的笔误。"""
    with pytest.raises(ParamError, match="不带单位"):
        Params.integer(0).convert(40, "km")


def test_known_units_put_base_unit_first() -> None:
    """报错里列可用单位时，基本单位排最前——作者最常想用的就是它。"""
    for dimension in (
        Dimension.DISTANCE,
        Dimension.TIME,
        Dimension.ANGLE,
        Dimension.SPEED,
    ):
        units = known_units(dimension)
        assert units[0] == BASE_UNIT[dimension], dimension
    rest = known_units(Dimension.DISTANCE)[1:]
    assert rest == sorted(rest, key=lambda u: (len(u), u))


def test_param_error_carries_offending_names() -> None:
    """异常必须带上出错的参数名——语义层靠它把报错定位到具体哪一行。"""
    with pytest.raises(UnknownParameterError) as info:
        radar_params().resolve({"detect_rang": 1, "fovv": 2})
    assert set(info.value.names) == {"detect_rang", "fovv"}


def test_unit_error_carries_parameter_name() -> None:
    with pytest.raises(ParamError) as info:
        radar_params().resolve({"detect_range": with_unit(40, "kmm")})
    assert info.value.names == ("detect_range",)


def test_range_error_carries_parameter_name() -> None:
    params = ParamSet(
        {"fov": Params.angle(120.0, minimum=0.0, maximum=360.0)}, owner="RADAR"
    )
    with pytest.raises(ParamError) as info:
        params.resolve({"fov": -5.0})
    assert info.value.names == ("fov",)


def test_missing_error_carries_parameter_names() -> None:
    params = ParamSet({"aim": Params.distance()}, owner="GUN")
    with pytest.raises(MissingParameterError) as info:
        params.resolve({})
    assert info.value.names == ("aim",)


def test_error_message_includes_parameter_name_not_default_value() -> None:
    """报错前缀是"类型名.参数名"，不是"参数类型 = 默认值"。

    后者在组件有二十个参数时毫无用处——根本不知道是哪一个写错了。
    """
    with pytest.raises(ParamError) as info:
        radar_params().resolve({"detect_range": with_unit(40, "kmm")})
    message = str(info.value)
    assert "HEX_SEARCH_RADAR.detect_range" in message
    assert "100000" not in message


# ---------------------------------------------------------------------------
# 类型校验
# ---------------------------------------------------------------------------

def test_boolean_rejects_numbers() -> None:
    with pytest.raises(ParamError, match="true / false"):
        Params.boolean().convert(1, None)


def test_number_rejects_bool() -> None:
    """bool 是 int 的子类，不排除的话 `true` 会被当成 1。"""
    with pytest.raises(ParamError, match="需要数值"):
        Params.number().convert(True, None)


def test_integer_rejects_fraction() -> None:
    with pytest.raises(ParamError, match="需要整数"):
        Params.integer().convert(40.5, None)


def test_integer_accepts_whole_float() -> None:
    """`40.0` 是 float 但值是整数，容忍它——想定作者不该为这类事被卡住。"""
    assert Params.integer().convert(40.0, None) == 40


def test_string_rejects_number() -> None:
    with pytest.raises(ParamError, match="需要字符串"):
        Params.string().convert(7, None)


def test_strings_rejects_bare_string() -> None:
    """`modes SARH` 与 `modes [SARH]` 的意图不同，前者是笔误，必须报错。"""
    with pytest.raises(ParamError, match="需要列表"):
        Params.strings().convert("SARH", None)


def test_strings_returns_immutable_tuple() -> None:
    result = Params.strings().convert(["SARH", "IR"], None)
    assert result == ("SARH", "IR")
    assert isinstance(result, tuple)


def test_strings_rejects_non_string_element() -> None:
    with pytest.raises(ParamError, match="必须是字符串"):
        Params.strings().convert(["SARH", 7], None)


def test_choices_are_enforced() -> None:
    spec = Params.string(choices=("SARH", "IR", "GUN"))
    assert spec.convert("IR", None) == "IR"
    with pytest.raises(ParamError, match="只能是"):
        spec.convert("LASER", None)


def test_choices_apply_to_list_elements() -> None:
    spec = Params.strings(choices=("SARH", "IR"))
    with pytest.raises(ParamError, match="只能是"):
        spec.convert(["SARH", "LASER"], None)


def test_range_is_enforced() -> None:
    spec = Params.angle(minimum=0.0, maximum=360.0)
    assert spec.convert(180.0, None) == 180.0
    with pytest.raises(ParamError, match="不能小于"):
        spec.convert(-1.0, None)
    with pytest.raises(ParamError, match="不能大于"):
        spec.convert(400.0, None)


def test_range_checked_after_unit_conversion() -> None:
    """范围校验必须作用在**换算后**的值上。

    先校验后换算的话，`detect_range 500 km` 在 max=100000 米时会被判合法
    （500 < 100000），实际却是 500 公里。
    """
    spec = Params.distance(maximum=100_000.0)
    assert spec.convert(50, "km") == pytest.approx(50_000.0)
    with pytest.raises(ParamError, match="不能大于"):
        spec.convert(500, "km")


def test_coord_requires_coord_value() -> None:
    from milsim.services.params import CoordValue

    spec = Params.coord()
    assert spec.convert(CoordValue("latlng", 39.9, 116.4), None).lat == 39.9
    with pytest.raises(ParamError, match="需要坐标"):
        spec.convert("39.9,116.4", None)


def test_coord_rejects_unknown_form() -> None:
    from milsim.services.params import CoordValue

    with pytest.raises(ParamError, match="未知坐标形式"):
        CoordValue("polar", 1.0, 2.0)


# ---------------------------------------------------------------------------
# ParamSet：整体解析
# ---------------------------------------------------------------------------

def radar_params() -> ParamSet:
    return ParamSet(
        {
            "detect_range": Params.distance(40_000.0),
            "scan_interval": Params.duration(2_000_000),
            "fov": Params.angle(120.0),
            "track_capacity": Params.integer(40),
            "modes": Params.strings(default=("SARH",)),
            "enabled": Params.boolean(default=True),
        },
        owner="HEX_SEARCH_RADAR",
    )


def test_resolve_fills_defaults() -> None:
    result = radar_params().resolve({})
    assert result["detect_range"] == 40_000.0
    assert result["scan_interval"] == 2_000_000
    assert result["modes"] == ("SARH",)
    assert result["enabled"] is True


def test_resolve_converts_units() -> None:
    result = radar_params().resolve(
        {"detect_range": with_unit(60, "km"), "scan_interval": with_unit(3, "s")}
    )
    assert result["detect_range"] == pytest.approx(60_000.0)
    assert result["scan_interval"] == 3_000_000


def test_resolve_accepts_bare_values_and_withunit() -> None:
    """裸值与 WithUnit 都要能混着用——不是每个参数都需要单位。"""
    result = radar_params().resolve(
        {
            "fov": 90.0,                                  # 裸值，按基本单位（度）
            "detect_range": with_unit(60, "km"),          # 带单位
            "enabled": False,
        }
    )
    assert result["fov"] == pytest.approx(90.0)
    assert result["detect_range"] == pytest.approx(60_000.0)
    assert result["enabled"] is False


def test_unknown_parameter_error_suggests_correction() -> None:
    """拼错的参数名必须报错，并给出最接近的名字。

    静默忽略拼错的参数，比直接报错糟糕得多——你会花半天纳闷
    "为什么探测距离没生效"。
    """
    with pytest.raises(UnknownParameterError) as info:
        radar_params().resolve({"detect_rang": with_unit(40, "km")})
    message = str(info.value)
    assert "detect_rang" in message
    assert "detect_range" in message          # 修正建议
    assert "HEX_SEARCH_RADAR" in message      # 出错的类型


def test_unknown_parameter_without_close_match_still_reports() -> None:
    with pytest.raises(UnknownParameterError) as info:
        radar_params().resolve({"totally_unrelated": 1})
    assert "totally_unrelated" in str(info.value)


def test_unknown_parameter_lists_all_offenders() -> None:
    """一次把错的参数全报出来，别让人改一个跑一次。"""
    with pytest.raises(UnknownParameterError) as info:
        radar_params().resolve({"detect_rang": 1, "fovv": 2})
    message = str(info.value)
    assert "detect_rang" in message and "fovv" in message


def test_missing_required_parameter() -> None:
    params = ParamSet({"aim_point": Params.coord()}, owner="GUN")
    with pytest.raises(MissingParameterError) as info:
        params.resolve({})
    assert "aim_point" in str(info.value)


def test_resolve_does_not_mutate_input() -> None:
    values = {"detect_range": with_unit(60, "km")}
    radar_params().resolve(values)
    assert values == {"detect_range": with_unit(60, "km")}


def test_resolve_returns_fresh_dict_each_call() -> None:
    """两次解析的结果不能共享可变对象——否则改一次会污染另一次。"""
    params = radar_params()
    first = params.resolve({})
    first["modes"] = ("MUTATED",)
    assert params.resolve({})["modes"] == ("SARH",)


def test_defaults_are_copies() -> None:
    params = radar_params()
    defaults = params.defaults()
    defaults["track_capacity"] = 999
    assert params.defaults()["track_capacity"] == 40


def test_schema_is_readable() -> None:
    schema = radar_params().schema()
    assert "detect_range" in schema
    assert "距离" in schema
    assert "scan_interval" in schema
    assert "时间" in schema


def test_empty_param_set() -> None:
    params = ParamSet({}, owner="RELAY")
    assert params.resolve({}) == {}
    assert len(params) == 0
    assert "无参数" in params.schema()


def test_from_class_reads_params_attribute() -> None:
    class FakeSensor:
        COMPONENT_NAME = "FAKE"
        PARAMS = {"detect_range": Params.distance(40_000.0)}

    params = ParamSet.from_class(FakeSensor)
    assert params.owner == "FAKE"
    assert "detect_range" in params


def test_from_class_allows_missing_params() -> None:
    """没有 PARAMS 的组件给空表，不报错——只转发消息的组件确实没参数。"""
    class Bare:
        pass

    params = ParamSet.from_class(Bare)
    assert len(params) == 0
    assert params.owner == "Bare"


def test_from_class_rejects_non_spec_values() -> None:
    """声明成裸字符串的话，错误会跑到很远的地方才炸，所以在声明处就拦住。"""
    class Broken:
        COMPONENT_NAME = "BROKEN"
        PARAMS = {"detect_range": 40_000.0}

    with pytest.raises(ParamError, match="必须是 Params"):
        ParamSet.from_class(Broken)


def test_from_class_rejects_non_mapping_params() -> None:
    class Broken:
        COMPONENT_NAME = "BROKEN"
        PARAMS = ["detect_range"]

    with pytest.raises(ParamError, match="必须是字典"):
        ParamSet.from_class(Broken)


def test_merged_with_prefers_right_hand_side() -> None:
    base = ParamSet({"a": Params.number(1.0), "b": Params.number(2.0)}, owner="BASE")
    extra = ParamSet({"b": Params.number(20.0)}, owner="EXTRA")
    merged = base.merged_with(extra)
    assert merged.spec("a") is not None
    assert merged.spec("b").default == 20.0
    assert merged.owner == "EXTRA"


def test_with_unit_helper() -> None:
    wrapped = with_unit(40, "km")
    assert isinstance(wrapped, WithUnit)
    assert (wrapped.value, wrapped.unit) == (40, "km")


def test_required_names() -> None:
    params = ParamSet(
        {"a": Params.distance(), "b": Params.distance(1.0)}, owner="T"
    )
    assert params.required_names() == ["a"]
    assert "a" not in params.defaults()


# ---------------------------------------------------------------------------
# 功率（W）—— 加它是因为"用 number 凑功率"会让 250 kW 与 250 变成同一个数
# ---------------------------------------------------------------------------

def test_power_units() -> None:
    spec = Params.power(0.0)
    # ★ 裸数 = 基本单位（W）。这一条是**向后兼容的判据**：v0.13.29 及以前
    #   ``peak_power`` 是 ``Params.number``，想定里写的都是裸瓦数。
    assert spec.convert(250.0, None) == pytest.approx(250.0)
    assert spec.convert(250.0, "W") == pytest.approx(250.0)
    assert spec.convert(250.0, "kW") == pytest.approx(250_000.0)
    assert spec.convert(2.0, "MW") == pytest.approx(2_000_000.0)
    assert spec.convert(250.0, "kw") == pytest.approx(250_000.0)


def test_power_refuses_the_megawatt_milliwatt_trap() -> None:
    """``MW`` 与 ``mW`` 相差 10⁹ 倍，**两个都在表里就都能查到、都不报错**。

    所以刻意只收 ``W`` / ``kW`` / ``MW``（及对应小写）。这条用例钉住
    "毫瓦与毫瓦级缩写必须被拒"——报错里还会带出可用单位清单。
    """
    spec = Params.power(0.0)
    for bad in ("mW", "mw", "uW", "dBW", "dBm", "KW"):
        with pytest.raises(ParamError, match="未知单位"):
            spec.convert(1.0, bad)


def test_power_dimension_is_wired_in_all_five_places() -> None:
    """★ 加一个量纲要改**五处**，漏一处不会报错、只会在某个调用点炸。

    这条用例本身就是踩出来的：``dimension_name()`` 是一张独立的映射表，
    加 ``POWER`` 时漏了它，症状是**第一次调用 ``describe()`` 才 KeyError**
    （而参数解析一路正常）。所以这里逐个点名五处，缺一即失败。
    """
    spec = Params.power(0.0)
    assert Dimension.POWER in BASE_UNIT                       # ① 基本单位
    assert Dimension.POWER in _UNITS                          # ② 换算表
    assert "W" in known_units(Dimension.POWER)                # ③ 单位清单
    assert spec.dimension_name() == "功率"                    # ④ 中文名
    assert spec.type_label() == "功率(W)"                     # ⑤ 报错用的类型名
