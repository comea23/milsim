"""想定语言的测试：词法、语法、语义三层。

分三层测是因为它们的失败模式不同：

- **词法**：单位切分、字符串闭合、行列号——错了会静默改变数值含义
- **语法**：块配对、值形态——错了会让后面的内容被误解析到别的块里
- **语义**：键名、单位换算、引用——错了会让人对着"配了没生效"发呆

其中"块配对"和"单位换算"是重点：这两类错误不会当场炸，只会让推演结果
偏掉，事后极难反查。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from milsim.services.input.lexer import (
    LexError,
    TokenKind,
    format_error,
    tokenize,
)
from milsim.services.input.parser import (
    ParseError,
    TaggedValue,
    parse,
)
from milsim.services.params import CoordValue, WithUnit
from milsim.services.scenario import (
    DecisionProfile,
    ScenarioError,
    _split_tags,
    load_scenario,
)

DEMO = Path(__file__).resolve().parents[1] / "scenarios" / "demo.txt"


# ===========================================================================
# 词法
# ===========================================================================

def kinds(text: str) -> list[TokenKind]:
    return [t.kind for t in tokenize(text) if t.kind is not TokenKind.NEWLINE]


def tokens_of(text: str):
    return [t for t in tokenize(text) if t.kind is not TokenKind.NEWLINE]


def test_basic_words_and_numbers() -> None:
    toks = tokens_of("radius 120 km")
    assert [t.kind for t in toks[:4]] == [
        TokenKind.NAME,
        TokenKind.NUMBER,
        TokenKind.UNIT,
        TokenKind.EOF,
    ]
    assert toks[0].text == "radius"
    assert toks[1].value == 120


def test_unit_with_and_without_space_are_identical() -> None:
    """``40km`` 与 ``40 km`` 必须切出完全一样的结果。

    让空格的有无承担不同语义，是最没必要的一类坑。
    """
    tight = tokens_of("detect_range 40km")
    spaced = tokens_of("detect_range 40 km")
    assert [(t.kind, t.text) for t in tight] == [(t.kind, t.text) for t in spaced]


def test_unit_is_not_swallowed_when_absent() -> None:
    """没有单位时不能误吞后面的词。"""
    toks = tokens_of("layers 3")
    assert [t.kind for t in toks] == [TokenKind.NAME, TokenKind.NUMBER, TokenKind.EOF]


def test_overlong_word_is_not_treated_as_unit() -> None:
    """``40something`` 不该被当成"数值 + 单位"。

    单位表里最长的是 km/h，超过长度上限的一律不认——切成数字加标识符，
    语法层会报错，比悄悄当成单位好。
    """
    toks = tokens_of("range 40something")
    assert [t.kind for t in toks[:3]] == [
        TokenKind.NAME,
        TokenKind.NUMBER,
        TokenKind.NAME,
    ]


def test_number_forms() -> None:
    assert tokens_of("-5")[0].value == -5
    assert tokens_of("3.14")[0].value == pytest.approx(3.14)
    assert tokens_of("1e6")[0].value == pytest.approx(1e6)
    assert tokens_of("2.5E-3")[0].value == pytest.approx(2.5e-3)
    # 前导零要按十进制读，`int("007")` 会失败
    assert tokens_of("007")[0].value == 7


def test_comments_are_dropped() -> None:
    assert kinds("# 整行注释\n") == [TokenKind.EOF]
    toks = tokens_of("radius 120 km   # 行尾注释")
    assert [t.text for t in toks[:3]] == ["radius", "120", "km"]


def test_strings_with_escapes() -> None:
    toks = tokens_of('name "RED \\"ACE\\""')
    assert toks[1].kind is TokenKind.STRING
    assert toks[1].value == 'RED "ACE"'


def test_boolean_literals() -> None:
    toks = tokens_of("enabled true\nother false")
    assert toks[0].text == "enabled"
    assert toks[1].kind is TokenKind.BOOLEAN and toks[1].value is True
    assert toks[2].text == "other"
    assert toks[3].kind is TokenKind.BOOLEAN and toks[3].value is False


def test_tagged_value() -> None:
    toks = tokens_of("terrain procedural seed=42")
    assert toks[2].kind is TokenKind.TAGGED
    assert toks[2].tag == "seed"
    assert toks[2].value == 42
    assert toks[2].text == "seed=42"


def test_loose_equals_is_rejected() -> None:
    """``seed = 42`` 不支持。等号两侧有空格时报错，而不是含糊地切。

    含糊地切会让作者以为写的是内联标注，实际被当成两个东西——
    这类"看着像对、其实没生效"的写法必须当场拒绝。
    """
    with pytest.raises(LexError, match="无法识别的字符"):
        list(tokenize("seed = 42"))
    with pytest.raises(LexError, match="无法识别的字符"):
        list(tokenize("seed= 42"))


def test_line_and_column_are_accurate() -> None:
    toks = tokens_of("a 1\n  b 2")
    assert toks[0].text == "a"
    assert (toks[0].line, toks[0].column) == (1, 1)
    assert toks[2].text == "b"
    assert (toks[2].line, toks[2].column) == (2, 3)
    assert (toks[3].line, toks[3].column) == (2, 5)


def test_unterminated_string_reports_position() -> None:
    with pytest.raises(LexError) as info:
        list(tokenize('name "abc\n'))
    assert info.value.line == 1
    assert info.value.column == 6


def test_illegal_character_reports_position() -> None:
    with pytest.raises(LexError, match="无法识别的字符"):
        list(tokenize("radius 120 $"))


def test_format_error_draws_caret() -> None:
    source = "detect_range 40 kmm"
    try:
        list(tokenize(source))
    except LexError as exc:
        rendered = format_error(exc, path="demo.txt", source=source)
        assert "demo.txt:1" in rendered
        assert "^" in rendered
        assert "detect_range 40 kmm" in rendered


# ===========================================================================
# 语法
# ===========================================================================

def test_block_and_assignments() -> None:
    blocks = parse(
        "zone A\n    radius 120 km\n    layers 3\nend_zone\n"
    )
    assert len(blocks) == 1
    block = blocks[0]
    assert block.keyword == "zone"
    assert block.names == ["A"]
    assert block.line == 1 and block.end_line == 4
    assert [a.key for a in block.assignments()] == ["radius", "layers"]


def test_header_with_inheritance() -> None:
    blocks = parse(
        "component_type HEX_SEARCH_RADAR : RADAR_BASE\n"
        "    detect_range 40 km\n"
        "end_component_type\n"
    )
    assert blocks[0].names == ["HEX_SEARCH_RADAR"]
    assert blocks[0].parent == "RADAR_BASE"


def test_header_with_two_names() -> None:
    blocks = parse("platform SAM_1 SAM_BATTALION\nend_platform\n")
    assert blocks[0].names == ["SAM_1", "SAM_BATTALION"]
    assert blocks[0].parent is None


def test_nested_component_block() -> None:
    blocks = parse(
        "platform_type SAM\n"
        "    side red\n"
        "    component sensor HEX_SEARCH_RADAR\n"
        "        detect_range 60 km\n"
        "    end_component\n"
        "end_platform_type\n"
    )
    children = blocks[0].blocks("component")
    assert len(children) == 1
    assert children[0].names == ["sensor", "HEX_SEARCH_RADAR"]
    assert children[0].get("detect_range").value == WithUnit(60, "km")


def test_decision_key_is_assignment_inside_platform() -> None:
    """``decision`` 在 platform 块里是**键**，不是子块。

    这是块关键字按上下文判断的关键用例：同一个名字在顶层开启一个块，
    在平台块里只是一行引用。只看名字分不出来，看它落在谁里面才知道。
    """
    blocks = parse(
        "platform SAM_1 SAM\n"
        "    decision TACTICAL_LLM\n"
        "end_platform\n"
    )
    assert blocks[0].blocks() == []
    assert blocks[0].get("decision").value == "TACTICAL_LLM"


def test_attr_is_a_child_block_only_inside_type_blocks() -> None:
    """``attr`` 与 ``component`` 同族：**只有按上下文才分得清**。

    它在 ``component_type`` / ``platform_type`` 里是一个块（参数库用它装
    "组件没有消费者"的量，见 ``services/library.py``），别处就只是普通的键
    ——所以写在 ``zone`` 里不会静默变成块，写在这里也不会被当成参数。
    """
    blocks = parse(
        "component_type LIB_TEST_RADAR\n"
        "    attr\n"
        "        SurfacePoK 85\n"
        "    end_attr\n"
        "end_component_type\n"
    )
    assert blocks[0].assignments() == []
    attr_blocks = blocks[0].blocks("attr")
    assert len(attr_blocks) == 1
    assert attr_blocks[0].get("SurfacePoK").value == 85


def test_decision_at_top_level_is_block() -> None:
    blocks = parse("decision D1\n    provider rule\nend_decision\n")
    assert blocks[0].keyword == "decision"
    assert blocks[0].names == ["D1"]


def test_mismatched_end_is_reported_precisely() -> None:
    """结束标记写错会在大范围里误解析，所以报错要给期望与实际。"""
    with pytest.raises(ParseError) as info:
        parse("component_type A\nend_platform_type\n")
    message = str(info.value)
    assert "end_component_type" in message      # 期望
    assert "end_platform_type" in message       # 实际


def test_missing_end_reports_block_start() -> None:
    with pytest.raises(ParseError) as info:
        parse("zone A\n    radius 1 km\n")
    message = str(info.value)
    assert "缺少 end_zone" in message
    assert "第 1 行" in message


def test_orphan_end_is_reported() -> None:
    with pytest.raises(ParseError, match="没有对应的开始块"):
        parse("end_zone\n")


def test_top_level_parameter_is_rejected() -> None:
    with pytest.raises(ParseError, match="顶层不能直接写参数"):
        parse("radius 120 km\n")


def test_assignment_without_value() -> None:
    with pytest.raises(ParseError, match="缺少值"):
        parse("zone A\n    radius\nend_zone\n")


def test_coordinate_latlng() -> None:
    blocks = parse("platform A B\n    position latlng 39.95 116.35\nend_platform\n")
    value = blocks[0].get("position").value
    assert isinstance(value, CoordValue)
    assert value.form == "latlng"
    assert (value.lat, value.lng) == (39.95, 116.35)


def test_coordinate_hex_with_layer() -> None:
    blocks = parse("platform A B\n    position hex 1204 883 @ layer 2\nend_platform\n")
    value = blocks[0].get("position").value
    assert (value.form, value.q, value.r, value.layer) == ("hex", 1204, 883, 2)


def test_coordinate_hex_defaults_to_layer_zero() -> None:
    blocks = parse("platform A B\n    position hex 10 20\nend_platform\n")
    assert blocks[0].get("position").value.layer == 0


def test_coordinate_rejects_bad_latitude() -> None:
    with pytest.raises(ParseError, match="纬度"):
        parse("platform A B\n    position latlng 95.0 116.0\nend_platform\n")


def test_coordinate_rejects_bad_longitude() -> None:
    with pytest.raises(ParseError, match="经度"):
        parse("platform A B\n    position latlng 39.0 200.0\nend_platform\n")


def test_coordinate_missing_second_number() -> None:
    with pytest.raises(ParseError, match="需要两个数值"):
        parse("platform A B\n    position latlng 39.9\nend_platform\n")


def test_hex_at_requires_keyword() -> None:
    # @ 后面跟了东西但不是关键字
    with pytest.raises(ParseError, match="应是 layer N 或 zone NAME"):
        parse("platform A B\n    position hex 1 2 @ 3\nend_platform\n")
    # @ 后面什么都没有
    with pytest.raises(ParseError, match="应是 layer N 或 zone NAME"):
        parse("platform A B\n    position hex 1 2 @\nend_platform\n")
    # 关键字写错
    with pytest.raises(ParseError, match="只能是 layer 或 zone"):
        parse("platform A B\n    position hex 1 2 @ foo 1\nend_platform\n")


def test_hex_coordinate_can_name_its_zone() -> None:
    """网格坐标是相对战区锚点的，多战区时必须写明属于哪个战区。"""
    blocks = parse(
        "platform A B\n    position hex 1204 883 @ zone NORTH_FRONT layer 2\n"
        "end_platform\n"
    )
    value = blocks[0].get("position").value
    assert (value.q, value.r, value.layer, value.zone) == (1204, 883, 2, "NORTH_FRONT")


def test_bare_pair_becomes_tuple() -> None:
    """``anchor 39.9 116.4`` 是两个数——语法层不猜它是什么，语义层才解释。"""
    blocks = parse("zone A\n    anchor 39.9042 116.4074\nend_zone\n")
    assert blocks[0].get("anchor").value == (39.9042, 116.4074)


def test_list_values() -> None:
    blocks = parse("component_type A\n    modes [SARH, IR]\nend_component_type\n")
    assert blocks[0].get("modes").value == ("SARH", "IR")


def test_empty_list() -> None:
    blocks = parse("component_type A\n    modes []\nend_component_type\n")
    assert blocks[0].get("modes").value == ()


def test_list_with_units() -> None:
    blocks = parse("component_type A\n    ranges [40 km, 20 km]\nend_component_type\n")
    assert blocks[0].get("ranges").value == (WithUnit(40, "km"), WithUnit(20, "km"))


def test_list_missing_comma() -> None:
    with pytest.raises(ParseError, match="需要逗号"):
        parse("component_type A\n    modes [SARH IR]\nend_component_type\n")


def test_list_not_closed() -> None:
    with pytest.raises(ParseError, match="没有闭合"):
        parse("component_type A\n    modes [SARH, IR\nend_component_type\n")


def test_tagged_value_in_parser() -> None:
    blocks = parse("zone A\n    terrain procedural seed=42\nend_zone\n")
    value = blocks[0].get("terrain").value
    assert isinstance(value, tuple)
    assert value[0] == "procedural"
    assert value[1] == TaggedValue("seed", 42)


def test_multiple_values_become_tuple() -> None:
    blocks = parse("zone A\n    terrain procedural seed=42 extra\nend_zone\n")
    value = blocks[0].get("terrain").value
    assert value[0] == "procedural"
    assert value[2] == "extra"


def test_percent_and_colon_are_not_confused() -> None:
    """块头的冒号与值的冒号不会互串。"""
    blocks = parse("platform_type P : BASE\n    side red\nend_platform_type\n")
    assert blocks[0].parent == "BASE"
    assert blocks[0].get("side").value == "red"


# ===========================================================================
# 语义：整体
# ===========================================================================

def demo_text() -> str:
    return DEMO.read_text(encoding="utf-8")


def test_demo_scenario_parses_cleanly() -> None:
    spec = load_scenario(demo_text(), name="demo.txt")
    assert spec.validate() == []
    assert spec.source_name == "demo.txt"


def test_demo_zones() -> None:
    spec = load_scenario(demo_text())
    zone = spec.zone("NORTH_FRONT")
    assert zone is not None
    assert zone.anchor.form == "latlng"
    assert zone.anchor.lat == pytest.approx(39.9042)
    assert zone.radius_m == pytest.approx(120_000.0)      # 120 km 换算成米
    assert zone.resolution_m == pytest.approx(100.0)
    assert zone.layers == 3
    assert zone.terrain == "procedural"
    assert zone.seed == 42


def test_demo_platforms() -> None:
    spec = load_scenario(demo_text())
    assert [p.name for p in spec.platforms] == ["SAM_1", "SAM_2"]
    first, second = spec.platforms
    assert first.type_name == "SAM_BATTALION"
    assert first.position.form == "latlng"
    assert first.heading_deg == pytest.approx(45.0)
    assert second.position.form == "hex"
    assert (second.position.q, second.position.r) == (100, 60)


def test_demo_decisions() -> None:
    spec = load_scenario(demo_text())
    rule = spec.decisions["RULE_DEFAULT"]
    llm = spec.decisions["TACTICAL_LLM"]

    assert rule.provider == "rule" and not rule.uses_llm
    assert llm.provider == "llm" and llm.uses_llm
    assert llm.model == "deepseek-chat"
    assert llm.timeout_us == 8_000_000
    assert llm.cadence_us == 30_000_000
    assert llm.timeout_s == pytest.approx(8.0)
    assert llm.fallback == "RULE_DEFAULT"
    assert llm.temperature == pytest.approx(0.2)
    assert llm.max_context == 4096


def test_demo_settings() -> None:
    spec = load_scenario(demo_text())
    assert spec.settings.max_step_us == 2_000_000
    assert spec.settings.max_step_s == pytest.approx(2.0)
    assert spec.settings.seed == 20260917
    assert spec.settings.name == "北线防空"


def test_demo_component_parameters_keep_units() -> None:
    """想定层的参数值原样带着单位——换算要有实现类的 PARAMS 才能做。"""
    spec = load_scenario(demo_text())
    attrs = spec.types.resolve_component_attrs("HEX_SEARCH_RADAR")
    assert attrs["detect_range"] == WithUnit(40, "km")
    assert attrs["fov"] == WithUnit(120, "deg")
    assert attrs["frequency"] == WithUnit(9.4, "GHz")
    assert attrs["modes"] == ("SARH", "IR")
    # 父类型 RADAR_BASE 的参数继承下来了
    assert attrs["scan_interval"] == WithUnit(2, "s")
    assert attrs["track_capacity"] == 40


def test_demo_platform_type_components() -> None:
    spec = load_scenario(demo_text())
    resolved = spec.types.resolve_platform("SAM_BATTALION")
    assert resolved.attributes["side"] == "red"          # 从 BASE_UNIT 继承
    assert resolved.components["sensor"].type_name == "HEX_SEARCH_RADAR"
    assert resolved.components["sensor"].attrs["detect_range"] == WithUnit(60, "km")


# -- 列表字面量在语义层不能被拆散 -----------------------------------------
#
# 语法层把 ``[x]`` 与裸 ``x`` 分得很清楚：前者是 ``('x',)``，后者是 ``'x'``。
# 语义层曾经只看"主值有几个"，把长度为 1 的元组当成"单值 + 标签"里的那个
# 单值拆掉，于是两者的区别在这里丢失。后果是 ``blocked_terrain [山地]``
# 到 ``ParamKind.STRINGS`` 手上变成字符串，直接报"需要列表"——而
# ``[山地, 水域]`` 两个元素反而没事。**只有最短、最常写的那种写法会翻车。**


def test_split_tags_keeps_the_tuple_shape() -> None:
    assert _split_tags(("procedural", TaggedValue("seed", 42))) == (
        "procedural",
        {"seed": 42},
    )
    assert _split_tags(("a", "b")) == (("a", "b"), {})
    assert _split_tags(("a",)) == (("a",), {})      # 单元素列表
    assert _split_tags(()) == ((), {})              # 空列表
    assert _split_tags("a") == ("a", {})            # 裸标量
    assert _split_tags(TaggedValue("seed", 42)) == (None, {"seed": 42})


def test_single_element_list_survives_the_semantic_layer() -> None:
    spec = load_scenario(
        "component_type A\n    blocked_terrain [山地]\nend_component_type\n"
    )
    assert spec.types.resolve_component_attrs("A")["blocked_terrain"] == ("山地",)


def test_empty_list_survives_the_semantic_layer() -> None:
    """``[]`` 是"我要一个空列表"，与"这个参数没给值"不是一回事。

    两者如果都变成 ``None``，用 ``[]`` 去清空一个**非空默认值**就永远
    做不到，而且不报错——参数静默保持默认值。
    """
    spec = load_scenario(
        "component_type A\n    blocked_terrain []\nend_component_type\n"
    )
    assert spec.types.resolve_component_attrs("A")["blocked_terrain"] == ()


def test_single_value_with_a_tag_still_unwraps() -> None:
    """带标签的单值仍要拆成标量——它是 ``值 + 至少一个 name=v``，两个元素。"""
    spec = load_scenario(
        "zone A\n"
        "    anchor 39.9042 116.4074\n"
        "    terrain procedural seed=31\n"
        "end_zone\n"
    )
    zone = spec.zones[0]
    assert zone.terrain == "procedural"
    assert zone.seed == 31


def test_llm_profiles_listed() -> None:
    spec = load_scenario(demo_text())
    assert [p.name for p in spec.llm_profiles()] == ["TACTICAL_LLM"]


def test_summary_is_informative() -> None:
    summary = load_scenario(demo_text(), name="demo.txt").summary()
    assert "2 组件类型" in summary
    assert "1 战区" in summary
    assert "1 份用大模型" in summary


# ===========================================================================
# 语义：校验与报错
# ===========================================================================

def test_unknown_parameter_gets_suggestion_and_line() -> None:
    text = (
        "component_type RADAR\n"
        "    scan_interval 2 s\n"
        "end_component_type\n"
        "zone A\n"
        "    anchor 39.9 116.4\n"
        "    radiuss 120 km\n"        # 拼错
        "end_zone\n"
    )
    # 想定解析阶段不校验组件参数（那要等工厂），但 zone 的键是框架known的
    with pytest.raises(Exception) as info:
        load_scenario(text)
    assert "radiuss" in str(info.value)


def test_zone_unknown_parameter_suggests_close_match() -> None:
    text = "zone A\n    anchor 39.9 116.4\n    radiu 120 km\nend_zone\n"
    with pytest.raises(Exception) as info:
        load_scenario(text)
    message = str(info.value)
    assert "radiu" in message and "radius" in message


def test_unknown_parameter_reports_exact_line() -> None:
    """报错要点到**出错那一行**，不是块首——拼错的地方才是有用的信息。"""
    text = (
        "zone A\n"
        "    anchor 39.9 116.4\n"
        "    radius 100 km\n"
        "    radiuss 200 km\n"
        "end_zone\n"
    )
    with pytest.raises(ScenarioError) as info:
        load_scenario(text)
    assert info.value.line == 4
    assert "radiuss" in str(info.value)


def test_bad_unit_reports_exact_line_and_parameter() -> None:
    text = "zone A\n    anchor 39.9 116.4\n    radius 120 kmm\nend_zone\n"
    with pytest.raises(ScenarioError) as info:
        load_scenario(text)
    assert info.value.line == 3
    message = str(info.value)
    assert "radius" in message          # 参数名
    assert "kmm" in message             # 写错的东西
    assert "km" in message              # 可用单位里包含正确的那个


def test_out_of_range_reports_exact_line() -> None:
    text = "simulation\n    seed 1\n    max_step 5 s\nend_simulation\n"
    with pytest.raises(ScenarioError) as info:
        load_scenario(text)
    assert info.value.line == 3


def test_missing_required_falls_back_to_block_line() -> None:
    """参数压根没写，没有"出错那一行"可指，回退到块首。"""
    text = "zone A\n    radius 10 km\nend_zone\n"
    with pytest.raises(ScenarioError) as info:
        load_scenario(text)
    assert info.value.line == 1
    assert "anchor" in str(info.value)


def test_duplicate_parameter_rejected() -> None:
    """同一个键写两次时静默取后者，会让人对着"改了没生效"发呆。"""
    text = "zone A\n    anchor 39.9 116.4\n    radius 100 km\n    radius 200 km\nend_zone\n"
    with pytest.raises(ScenarioError, match="写了两次"):
        load_scenario(text)


def test_duplicate_zone_rejected() -> None:
    text = (
        "zone A\n    anchor 39.9 116.4\nend_zone\n"
        "zone A\n    anchor 40.0 117.0\nend_zone\n"
    )
    with pytest.raises(ScenarioError, match="重复定义"):
        load_scenario(text)


def test_duplicate_decision_rejected() -> None:
    text = (
        "decision D\n    provider rule\nend_decision\n"
        "decision D\n    provider rule\nend_decision\n"
    )
    with pytest.raises(ScenarioError, match="重复定义"):
        load_scenario(text)


def test_duplicate_component_type_rejected() -> None:
    text = (
        "component_type A\n    x 1\nend_component_type\n"
        "component_type A\n    x 2\nend_component_type\n"
    )
    with pytest.raises(Exception, match="重复定义"):
        load_scenario(text)


def test_zone_requires_anchor() -> None:
    with pytest.raises(Exception, match="anchor"):
        load_scenario("zone A\n    radius 100 km\nend_zone\n")


def test_zone_anchor_must_be_latlng() -> None:
    """战区锚点定义了局部平面网格的原点，用网格坐标没有意义。"""
    text = "zone A\n    anchor hex 100 200\nend_zone\n"
    with pytest.raises(ScenarioError, match="必须用 latlng"):
        load_scenario(text)


def test_platform_needs_two_names() -> None:
    with pytest.raises(ScenarioError, match="两个名字"):
        load_scenario("platform SAM_1\n    position latlng 39.9 116.4\nend_platform\n")


def test_platform_position_must_be_coordinate() -> None:
    text = 'platform A B\n    position "somewhere"\nend_platform\n'
    with pytest.raises(ScenarioError, match="需要坐标"):
        load_scenario(text)


def test_platform_type_rejects_instance_keys() -> None:
    """位置和航向是每一件装备自己的属性，不是型号的属性。"""
    text = "platform_type P\n    position latlng 39.9 116.4\nend_platform_type\n"
    with pytest.raises(ScenarioError, match="不该写 'position'"):
        load_scenario(text)


def test_component_type_rejects_attr() -> None:
    """``attr`` 是**参数库**的东西（毁伤概率、国别这类组件没有消费者的量）。

    语法层放它进来是因为它和 ``component`` 一样只有按上下文才分得清；
    语义层必须把它挡在想定之外——想定里写一段属性**没有读者**，不报错
    就是静默失效，那正是这套语法最不愿意发生的事。
    """
    text = (
        "component_type R\n"
        "    attr\n"
        "        国别 美国\n"
        "    end_attr\n"
        "end_component_type\n"
    )
    with pytest.raises(ScenarioError) as info:
        load_scenario(text)
    message = str(info.value)
    assert "attr" in message
    assert "参数库" in message


def test_platform_type_rejects_attr() -> None:
    text = (
        "platform_type T\n"
        "    attr\n"
        "        国别 美国\n"
        "    end_attr\n"
        "end_platform_type\n"
    )
    with pytest.raises(ScenarioError) as info:
        load_scenario(text)
    message = str(info.value)
    assert "attr" in message
    assert "参数库" in message


def test_max_step_cannot_exceed_engine_limit() -> None:
    """2 秒是引擎的硬约束（§2.4 的栅栏），在解析阶段就拦住。"""
    text = "simulation\n    max_step 5 s\nend_simulation\n"
    with pytest.raises(Exception) as info:
        load_scenario(text)
    assert "不能大于" in str(info.value)


def test_simulation_rejects_bad_seed() -> None:
    text = "simulation\n    seed -1\nend_simulation\n"
    with pytest.raises(Exception, match="不能小于"):
        load_scenario(text)


def test_unknown_block_is_rejected() -> None:
    with pytest.raises(ParseError, match="顶层不能直接写参数"):
        load_scenario("wargame A\nend_wargame\n")


# ---------------------------------------------------------------------------
# 大模型决策配置的校验
# ---------------------------------------------------------------------------

def test_llm_requires_model() -> None:
    text = (
        "decision L\n"
        "    provider llm\n"
        "    api_key_env K\n"
        "end_decision\n"
    )
    with pytest.raises(ScenarioError, match="没有写 model"):
        load_scenario(text)


def test_llm_requires_api_key_env() -> None:
    """想定里只写环境变量名——密钥本身绝不进文件。"""
    text = (
        "decision L\n"
        "    provider llm\n"
        '    model "m"\n'
        "end_decision\n"
    )
    with pytest.raises(ScenarioError, match="api_key_env"):
        load_scenario(text)


def test_provider_must_be_known() -> None:
    text = "decision L\n    provider magic\nend_decision\n"
    with pytest.raises(Exception, match="只能是"):
        load_scenario(text)


def test_decision_rejects_inheritance() -> None:
    text = "decision L : BASE\n    provider rule\nend_decision\n"
    with pytest.raises(ScenarioError, match="不支持继承"):
        load_scenario(text)


def test_validate_flags_missing_fallback_for_llm() -> None:
    """没有 fallback 的大模型决策在推演里等于定时炸弹。"""
    text = (
        "decision L\n"
        "    provider llm\n"
        '    model "m"\n'
        "    api_key_env K\n"
        "end_decision\n"
    )
    spec = load_scenario(text)
    problems = spec.validate()
    assert any("fallback" in p for p in problems)


def test_validate_flags_unknown_platform_type() -> None:
    text = (
        "decision D\n    provider rule\nend_decision\n"
        "platform A NO_SUCH_TYPE\n"
        "    position latlng 39.9 116.4\n"
        "end_platform\n"
    )
    spec = load_scenario(text)
    assert any("NO_SUCH_TYPE" in p for p in spec.validate())


def test_validate_flags_unknown_decision_reference() -> None:
    text = (
        "platform_type P\nend_platform_type\n"
        "platform A P\n"
        "    position latlng 39.9 116.4\n"
        "    decision NO_SUCH\n"
        "end_platform\n"
    )
    spec = load_scenario(text)
    problems = spec.validate()
    assert any("NO_SUCH" in p for p in problems)


def test_validate_flags_platform_without_position() -> None:
    text = (
        "platform_type P\nend_platform_type\n"
        "platform A P\nend_platform\n"
    )
    spec = load_scenario(text)
    assert any("position" in p for p in spec.validate())


def test_validate_clean_for_minimal_scenario() -> None:
    text = (
        "platform_type P\nend_platform_type\n"
        "decision D\n    provider rule\nend_decision\n"
        "platform A P\n"
        "    position latlng 39.9 116.4\n"
        "    decision D\n"
        "end_platform\n"
    )
    assert load_scenario(text).validate() == []


def test_forward_reference_to_decision_defined_later() -> None:
    """决策配置可以先用后定义——想定文件不必为了顺序而重排。"""
    text = (
        "platform_type P\nend_platform_type\n"
        "platform A P\n"
        "    position latlng 39.9 116.4\n"
        "    decision LATER\n"
        "end_platform\n"
        "decision LATER\n    provider rule\nend_decision\n"
    )
    spec = load_scenario(text)
    assert spec.validate() == []
    assert spec.effective_decision(spec.platforms[0]).name == "LATER"


def test_effective_decision_prefers_instance_over_type() -> None:
    """整营默认走规则、个别关键岗位走大模型——两级引用才不用逐实例写。"""
    text = (
        "platform_type P\n    decision BASE_RULE\nend_platform_type\n"
        "decision BASE_RULE\n    provider rule\nend_decision\n"
        "decision SPECIAL\n    provider rule\nend_decision\n"
        "platform A P\n"
        "    position latlng 39.9 116.4\n"
        "    decision SPECIAL\n"
        "end_platform\n"
        "platform B P\n"
        "    position latlng 39.8 116.3\n"
        "end_platform\n"
    )
    spec = load_scenario(text)
    assert spec.effective_decision(spec.platforms[0]).name == "SPECIAL"
    assert spec.effective_decision(spec.platforms[1]).name == "BASE_RULE"


def test_effective_decision_is_none_without_any_reference() -> None:
    text = (
        "platform_type P\nend_platform_type\n"
        "platform A P\n    position latlng 39.9 116.4\nend_platform\n"
    )
    spec = load_scenario(text)
    assert spec.effective_decision(spec.platforms[0]) is None


def test_decision_profile_defaults() -> None:
    profile = DecisionProfile(name="D")
    assert profile.provider == "rule"
    assert profile.timeout_us == 8_000_000
    assert profile.cadence_us == 30_000_000
    assert not profile.uses_llm


# ---------------------------------------------------------------------------
# 编队与指挥关系（§3.10）
# ---------------------------------------------------------------------------
FORMATION_BLOCK = (
    "formation RED_BDE\n"
    "    side red\n"
    "    echelon 旅\n"
    "end_formation\n"
    "formation RED_1BN : RED_BDE\n"
    "    side red\n"
    "    echelon 营\n"
    "end_formation\n"
    "formation RED_1_1 : RED_1BN\n"
    "    side red\n"
    "    echelon 连\n"
    "end_formation\n"
)


def test_formation_uses_colon_for_parent() -> None:
    """父级用块头冒号表示，与 ``component_type X : Y`` 同一套写法。"""
    spec = load_scenario(FORMATION_BLOCK)
    decl = {d.name: d for d in spec.formations}
    assert decl["RED_BDE"].parent is None
    assert decl["RED_1BN"].parent == "RED_BDE"
    assert decl["RED_1_1"].parent == "RED_1BN"
    assert decl["RED_1_1"].echelon == "连"
    assert decl["RED_1_1"].side == "red"


def test_formation_allows_forward_reference() -> None:
    """先写连、后写营也认得出来——平铺写法天然支持。"""
    text = (
        "formation RED_1_1 : RED_1BN\n    side red\nend_formation\n"
        "formation RED_1BN\n    side red\nend_formation\n"
    )
    spec = load_scenario(text)
    assert spec.validate() == []
    chain = spec.command_chain()
    assert chain.formations.parent_of(
        chain.formations.resolve("RED_1_1")
    ) == chain.formations.resolve("RED_1BN")


def test_formation_requires_side() -> None:
    with pytest.raises(ScenarioError, match="必须声明阵营"):
        load_scenario("formation RED_BDE\n    echelon 旅\nend_formation\n")


def test_formation_unknown_parameter_lists_choices() -> None:
    with pytest.raises(ScenarioError, match="未知参数"):
        load_scenario(
            "formation RED_BDE\n    side red\n    echelons 旅\nend_formation\n"
        )


def test_nested_subordinate_is_not_a_keyword() -> None:
    """曾经设计过嵌套的 subordinate 块，已改为平铺——它不该被当成块。"""
    with pytest.raises(ParseError):
        parse(
            "formation RED_BDE\n    side red\n"
            "    subordinate RED_1BN : 营\n    end_subordinate\n"
            "end_formation\n"
        )


def test_platform_declares_its_formation() -> None:
    spec = load_scenario(
        FORMATION_BLOCK
        + "platform_type TANK\nend_platform_type\n"
        + "platform T1 TANK\n"
        + "    position latlng 39.9 116.4\n"
        + "    formation RED_1_1\n"
        + "end_platform\n"
    )
    assert spec.platforms[0].formation == "RED_1_1"
    assert spec.validate() == []


def test_formation_is_instance_only() -> None:
    """编队归属是实例属性（同 position），写在 platform_type 里是概念错误。"""
    with pytest.raises(ScenarioError, match="不该写"):
        load_scenario(
            FORMATION_BLOCK
            + "platform_type TANK\n    formation RED_1_1\nend_platform_type\n"
        )


def test_platform_referencing_unknown_formation() -> None:
    spec = load_scenario(
        FORMATION_BLOCK
        + "platform_type TANK\nend_platform_type\n"
        + "platform T1 TANK\n"
        + "    position latlng 39.9 116.4\n"
        + "    formation RED_9_9\n"
        + "end_platform\n"
    )
    problems = spec.validate()
    assert any("未定义的编队" in p for p in problems)


def test_command_block_parses_all_three_actions() -> None:
    spec = load_scenario(
        FORMATION_BLOCK
        + "command\n"
        + "    attach RED_1_1 to RED_1BN from 30 min until 8 h\n"
        + "    direct RED_BDE to RED_1_1\n"
        + "    coordinate RED_BDE with RED_1BN\n"
        + "end_command\n"
    )
    assert len(spec.attachments) == 1
    assert spec.attachments[0].unit == "RED_1_1"
    assert spec.attachments[0].parent == "RED_1BN"
    assert spec.attachments[0].start_us == 30 * 60_000_000
    assert spec.attachments[0].until_us == 8 * 3_600_000_000
    assert len(spec.directives) == 1
    assert len(spec.coordinations) == 1


def test_command_without_time_clause_is_permanent() -> None:
    spec = load_scenario(
        FORMATION_BLOCK + "command\n    attach RED_1_1 to RED_1BN\nend_command\n"
    )
    assert spec.attachments[0].start_us == 0
    assert spec.attachments[0].until_us is None


def test_command_connector_is_optional() -> None:
    """``to`` / ``with`` 是给人读的，语义上可有可无。"""
    spec = load_scenario(
        FORMATION_BLOCK + "command\n    attach RED_1_1 RED_1BN\nend_command\n"
    )
    assert spec.attachments[0].parent == "RED_1BN"


def test_command_rejects_unknown_action() -> None:
    with pytest.raises(ScenarioError, match="不认识的指令"):
        load_scenario(FORMATION_BLOCK + "command\n    attachh A to B\nend_command\n")


def test_command_rejects_bad_time_clause() -> None:
    with pytest.raises(ScenarioError, match="时间子句只认 from / until"):
        load_scenario(
            FORMATION_BLOCK
            + "command\n    attach RED_1_1 to RED_1BN since 30 min\nend_command\n"
        )


def test_command_rejects_reversed_window() -> None:
    with pytest.raises(ScenarioError, match="until 必须晚于 from"):
        load_scenario(
            FORMATION_BLOCK
            + "command\n    attach RED_1_1 to RED_1BN from 8 h until 30 min\nend_command\n"
        )


def test_command_rejects_duplicate_time_keyword() -> None:
    with pytest.raises(ScenarioError, match="写了两次"):
        load_scenario(
            FORMATION_BLOCK
            + "command\n    attach RED_1_1 to RED_1BN from 1 h from 2 h\nend_command\n"
        )


def test_command_needs_two_names() -> None:
    with pytest.raises(ScenarioError, match="需要两个编队名"):
        load_scenario(FORMATION_BLOCK + "command\n    attach RED_1_1\nend_command\n")


def test_command_time_uses_integer_micros() -> None:
    """时间必须走整数微秒——浮点会让"同刻"的事件顺序漂移（§2.1）。"""
    spec = load_scenario(
        FORMATION_BLOCK
        + "command\n    attach RED_1_1 to RED_1BN from 0.1 s\nend_command\n"
    )
    assert spec.attachments[0].start_us == 100_000
    assert isinstance(spec.attachments[0].start_us, int)


def test_command_chain_overlap_detected_via_scenario() -> None:
    """想定层就能拦住配属时间重叠，不必等装配。"""
    spec = load_scenario(
        FORMATION_BLOCK
        + "command\n"
        + "    attach RED_1_1 to RED_1BN from 1 h until 3 h\n"
        + "    attach RED_1_1 to RED_BDE from 2 h until 4 h\n"
        + "end_command\n"
    )
    chain = spec.command_chain()
    assert any("重叠" in p for p in chain.validate())


def test_command_chain_rejects_cross_side_in_scenario() -> None:
    spec = load_scenario(
        FORMATION_BLOCK
        + "formation BLUE_1BN\n    side blue\nend_formation\n"
        + "command\n    attach RED_1_1 to BLUE_1BN\nend_command\n"
    )
    with pytest.raises(Exception, match="跨阵营"):
        spec.command_chain()


def test_command_chain_detects_bad_parent() -> None:
    spec = load_scenario(
        "formation RED_1_1 : 不存在的上级\n    side red\nend_formation\n"
    )
    with pytest.raises(ScenarioError, match="上级找不到"):
        spec.command_chain()
