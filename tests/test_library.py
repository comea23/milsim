"""参数库：型号参数存进数据库，想定只写型号名（§5.5）。

这一组测试盯的是**库不能破坏项目的两条底线**：

1. 拼错当场报错。库里的值最终仍走 ``ParamSet.resolve``，量纲、白名单、
   范围、拼写建议一个都不能少。
2. 文本与数据库是同一份内容。``to_text`` / ``from_text`` 必须**逐字往返**，
   否则"数据库里是什么"和"review 时看到的是什么"会分家。
"""

from __future__ import annotations

import importlib.util
import sqlite3
from pathlib import Path

import pytest

from milsim.errors import ConfigurationError
from milsim.models.component import Component
from milsim.services import ParamLibrary
from milsim.services.library import (
    KIND_COMPONENT,
    KIND_PLATFORM,
    DuplicateModelError,
    LibraryError,
    LibrarySchemaError,
    Literal,
    ModelRecord,
    literal_of,
    parse_literal,
)
from milsim.services.params import Params, WithUnit
from milsim.services.scenario import load_scenario
from milsim.services.type_registry import (
    DuplicateTypeError,
    TypeRegistry,
    register_component,
)
from milsim.simulation import Simulation

# ---------------------------------------------------------------------------
# 测试用组件
# ---------------------------------------------------------------------------


@register_component("LIB_TEST_RADAR")
class LibraryTestRadar(Component):
    """把所有参数种类都覆盖一遍，省得一个种类一条测试。"""

    PARAMS = {
        "detect_range": Params.distance(40_000.0, minimum=0.0),
        "scan_interval": Params.duration(2_000_000),
        "fov": Params.angle(120.0),
        "eccm": Params.integer(0, minimum=0, maximum=5),
        "active": Params.boolean(True),
        "mode": Params.string("search"),
        "bands": Params.strings(("x",)),
    }


@register_component("LIB_TEST_MOVER")
class LibraryTestMover(Component):
    PARAMS = {"max_speed": Params.speed(20.0)}


LIB_TEXT = """
component_type LIB_TEST_RADAR
    note   "测试雷达（公开资料估算）"
    source "单元测试"
    detect_range  150 km
    scan_interval 1 s
    fov           120 deg
    eccm          2
    active        true
    mode          "1"
    bands         a b c
end_component_type

component_type LONG_RANGE_RADAR : LIB_TEST_RADAR
    note          "远程型：只改探测距离，其余全部继承"
    detect_range  300 km
end_component_type

component_type LIB_TEST_MOVER
    max_speed 18 m/s
end_component_type

platform_type LIB_TEST_TANK
    note "测试底盘"
    side red
end_platform_type
"""

SCENARIO = """
zone AO
    anchor     39.9042 116.4074
    radius     20 km
    resolution 500 m
end_zone

platform_type PLAT_A
    side red
    component radar_1 {radar}
        eccm 3
    end_component
    component long_1 {inherited}
    end_component
    component mover_1 LIB_TEST_MOVER
    end_component
end_platform_type

platform P1 PLAT_A
    position latlng 39.95 116.40
end_platform
"""


def make_library(text: str = LIB_TEXT, *, origin: str = "<测试库>") -> ParamLibrary:
    return ParamLibrary.from_text(text, origin=origin)


def make_sim(library=None, *, radar: str = "LIB_TEST_RADAR",
             inherited: str = "LIB_TEST_RADAR", build: bool = True) -> Simulation:
    text = SCENARIO.format(radar=radar, inherited=inherited)
    sim = Simulation.from_scenario(text, library=library)
    if build:
        sim.build()
    return sim


def part_of(sim: Simulation, slot: str):
    return sim.entities[sim.registry.by_name("P1").entity_id].require(slot)


def radar_of(sim: Simulation):
    return part_of(sim, "radar_1")


# ---------------------------------------------------------------------------
# 文本 → 库
# ---------------------------------------------------------------------------


def test_text_parses_models_with_parent_and_metadata() -> None:
    lib = make_library()
    long_range = lib.get("LONG_RANGE_RADAR", KIND_COMPONENT)
    assert long_range is not None
    assert long_range.parent == "LIB_TEST_RADAR"
    assert long_range.params["detect_range"] == Literal("300", "km")
    assert long_range.note == "远程型：只改探测距离，其余全部继承"

    tank = lib.get("LIB_TEST_TANK", KIND_PLATFORM)
    assert tank is not None
    assert tank.note == "测试底盘"
    assert tank.params["side"] == Literal("red", as_text=False)


def test_library_text_rejects_non_type_blocks() -> None:
    """库里写 zone / platform 是**没人生成它**的——必须报错，不能静默丢。"""
    with pytest.raises(LibraryError, match="只认 component_type / platform_type"):
        make_library("zone AO\n    radius 20 km\nend_zone\n")


def test_library_text_rejects_nested_blocks() -> None:
    """``component`` 是**想定**里的子块，参数库里没有它的位置。

    唯一的例外是 ``attr``（见下面那一节），所以报错要把这件事说出来——
    只说"不认子块"的话，作者会转而去别处找地方存那些外部资料里的量。
    """
    with pytest.raises(LibraryError) as info:
        make_library(
            "platform_type T\n"
            "    component r LIB_TEST_RADAR\n"
            "    end_component\n"
            "end_platform_type\n"
        )
    message = str(info.value)
    assert "没有嵌套结构" in message
    assert "attr" in message


def test_duplicate_param_in_text_is_rejected() -> None:
    with pytest.raises(LibraryError, match="写了两次"):
        make_library(
            "component_type LIB_TEST_RADAR\n"
            "    eccm 2\n"
            "    eccm 3\n"
            "end_component_type\n"
        )


def test_duplicate_model_in_same_library_is_rejected() -> None:
    with pytest.raises(DuplicateModelError, match="重复定义"):
        make_library(
            "component_type LIB_TEST_RADAR\nend_component_type\n"
            "component_type LIB_TEST_RADAR\nend_component_type\n"
        )


def test_same_name_across_kinds_is_allowed() -> None:
    """组件型与平台型是两个名字空间——同名不冲突，注册表也分两个字典。"""
    lib = make_library(
        "component_type SAME\nend_component_type\n"
        "platform_type SAME\nend_platform_type\n"
    )
    assert lib.has("SAME", KIND_COMPONENT)
    assert lib.has("SAME", KIND_PLATFORM)
    assert len(lib) == 2


# ---------------------------------------------------------------------------
# 字面量
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "text, expected",
    [
        ("40", 40),
        ("-5", -5),
        ("2.5", 2.5),
        ("0.5e3", 500.0),
        ("true", True),
        ("false", False),
        ("procedural", "procedural"),
        ("a b c", ("a", "b", "c")),
        ("a/b.c-d_e", "a/b.c-d_e"),
    ],
)
def test_literal_syntax(text: str, expected: object) -> None:
    assert parse_literal(Literal(text)) == expected


def test_literal_with_unit_becomes_with_unit() -> None:
    assert parse_literal(Literal("150", "km")) == WithUnit(150.0, "km")


def test_numeric_looking_string_needs_quotes() -> None:
    """``"1"`` 是字符串，``1`` 是数——库里靠引号区分，文本形式也照此回写。"""
    assert literal_of("1") == Literal("1", as_text=True)
    assert literal_of("true") == Literal("true", as_text=True)
    assert literal_of("a b") == Literal("a b", as_text=True)
    assert literal_of("procedural") == Literal("procedural", as_text=False)
    assert parse_literal(literal_of("1")) == "1"


def test_chinese_punctuation_forces_quotes() -> None:
    """中文括号不在裸名字符集里——不加引号的话整个库文件读不回来。"""
    assert literal_of("某型（改）") == Literal("某型（改）", as_text=True)


def test_empty_value_is_rejected() -> None:
    with pytest.raises(LibraryError, match="不能为空"):
        Literal("   ")


def test_multi_value_with_unit_is_rejected() -> None:
    with pytest.raises(LibraryError, match="多值参数不支持单位"):
        literal_of((WithUnit(1.0, "km"), WithUnit(2.0, "km")))


# ---- 短列表：空格形式表达不了"一个元素"与"没有元素" ---------------------
#
# 这两个长度必须写成 `[...]`，否则**回写丢信息**：`()` 会回写成空串
# （Literal 拒绝空值，"把想定片段收编进库"整个失败），`(x,)` 会回写成
# `x`——读回来是字符串、不再是列表，于是 strings 参数报"需要列表"。
# 两处都不报"写的时候出了错"，只在下一次读的时候现形。


@pytest.mark.parametrize(
    "value, text",
    [
        ((), "[]"),
        (("x",), "[x]"),
        (("a", "b"), "a b"),
        (("a", "b", "c"), "a b c"),
    ],
)
def test_list_literals_round_trip(value, text) -> None:
    literal = literal_of(value)
    assert literal == Literal(text)
    assert parse_literal(literal) == value


def test_empty_list_literal_is_not_an_empty_value() -> None:
    """``[]`` 的**文本**非空，所以 Literal 收得下它。把空列表写成空串就
    撞上"参数值不能为空"，那是回写路径整个失败，不是一条参数的问题。"""
    assert literal_of(()) == Literal("[]")
    assert literal_of(()).value == "[]"


def test_list_elements_keep_their_quotes() -> None:
    """元素含空格时必须带引号，否则回写后**变成两个元素**。

    早先这里用的是字面量的 ``value``（不带引号），于是 `("某 型", "甲")`
    回写成 `某 型 甲`——读回来是三个元素。不报错，只是列表变长。
    """
    literal = literal_of(("某 型", "甲"))
    assert literal == Literal('"某 型" 甲')
    assert parse_literal(literal) == ("某 型", "甲")


def test_list_element_with_a_comma_survives() -> None:
    """逗号在引号里，切分时**不算分隔符**——列表不会因为一个逗号多出一项。"""
    assert parse_literal(literal_of(("a,b", "c"))) == ("a,b", "c")
    assert parse_literal(literal_of(("a,b",))) == ("a,b",)


def test_list_element_with_a_quote_is_refused() -> None:
    """双引号装不下——想定语言没有转义序列。拒绝回写，不静默切错。"""
    with pytest.raises(LibraryError, match="含双引号"):
        literal_of(('a"b',))


def test_single_element_list_survives_the_library() -> None:
    """``bands [x]``：库文件读进来、回写出去、再读回来还是"一个元素的列表"。"""
    lib = make_library(
        "component_type LIB_TEST_RADAR\n"
        "    bands [x]\n"
        "end_component_type\n"
    )
    record = lib.get("LIB_TEST_RADAR", KIND_COMPONENT)
    assert record is not None
    assert parse_literal(record.params["bands"]) == ("x",)

    again = ParamLibrary.from_text(lib.to_text(), origin=lib.origin)
    assert again.to_text() == lib.to_text()

    params = radar_of(make_sim(lib)).spec.params
    assert params["bands"] == ("x",)


# ---------------------------------------------------------------------------
# 文本 ↔ 库 往返
# ---------------------------------------------------------------------------


def test_text_round_trip_is_byte_identical() -> None:
    """**逐字往返**。这是数据库能放心当存储的前提。

    漏一个引号就整个读不回来（中文括号那次就是这样），而"读不回来"至少
    还看得见；真正难查的是**加载进来但值变了**——比如一个 150 km 被读成
    多值，或者一个字符串 "1" 变成了整数 1。
    """
    lib = make_library()
    again = ParamLibrary.from_text(lib.to_text(), origin=lib.origin)
    assert again.to_text() == lib.to_text()
    assert again.revision == lib.revision


def test_text_round_trip_keeps_awkward_values() -> None:
    lib = make_library(
        "component_type LIB_TEST_RADAR\n"
        '    mode   "1"\n'
        '    bands  "a b"\n'
        "    eccm   0\n"
        "end_component_type\n"
    )
    again = ParamLibrary.from_text(lib.to_text(), origin=lib.origin)
    record = again.get("LIB_TEST_RADAR")
    assert record is not None
    assert record.params["mode"] == Literal("1", as_text=True)
    assert record.params["bands"] == Literal("a b", as_text=True)
    assert record.params["eccm"] == Literal("0", as_text=False)


def test_newline_in_note_is_refused_on_write() -> None:
    """想定语言没有转义、也没有多行字符串——装不下的内容必须拒绝回写，
    而不是悄悄截断（截断不报错，下次读回来就是另一个值）。"""
    lib = ParamLibrary(
        [ModelRecord(KIND_COMPONENT, "X", {}, note="第一行\n第二行")],
        origin="<测试>",
    )
    with pytest.raises(LibraryError, match="换行"):
        lib.to_text()


# ---------------------------------------------------------------------------
# attr：存得下、但不进参数面
# ---------------------------------------------------------------------------
#
# 外部资料（装备手册、推演软件导出）带一批**组件现在没有消费者**的量：
# 毁伤概率、圆概率误差、国别……把它们塞进 `model_param` 是不行的——那一张表
# 的每一行都要能对上组件 PARAMS 里的一个键，塞进去只会让 lint 报一屏
# "未知参数"，而真正拼错的 `detec_range` 就淹在里面了。
#
# 所以它们走另一张表 `model_attr`，文本侧是型号里的一个 `attr` 内层块。
# 这一节盯的是三件事：**往返不丢**、**不进注册表**、**修订号跟着变**。


ATTR_TEXT = (
    "component_type LIB_TEST_RADAR\n"
    "    detect_range 150 km\n"
    "    attr\n"
    "        SurfacePoK 85\n"
    "        国别        美国\n"
    "    end_attr\n"
    "end_component_type\n"
)


def test_attr_parses_into_its_own_field() -> None:
    """属性进 ``record.attrs``，参数进 ``record.params``——两个字段，不是
    一个字典加标志位。分开正是为了"某次改查询忘了带条件"不会把属性
    当参数灌进注册表。"""
    record = make_library(ATTR_TEXT).get("LIB_TEST_RADAR")
    assert record is not None
    assert record.attrs == {"SurfacePoK": Literal("85"), "国别": Literal("美国")}
    assert record.params["detect_range"] == Literal("150", "km")
    assert "SurfacePoK" not in record.params


def test_attr_round_trips_through_the_text() -> None:
    """库文本 → 库 → 库文本，**逐字一致**。属性那一块也要能回读。"""
    lib = make_library(ATTR_TEXT)
    again = ParamLibrary.from_text(lib.to_text(), origin=lib.origin)
    assert again.to_text() == lib.to_text()
    assert again.revision == lib.revision


def test_attr_keys_align_by_display_width() -> None:
    """中文键按**显示宽度**对齐（``国别`` 占两列，不是四个字符位）。

    ``str.ljust`` 数的是字符数——用它对齐的话，第一个中文键开始整列右移；
    而 ``attr`` 的键恰恰经常是中文，一列参差不齐的键值对读第二遍就是噪音。
    """
    lines = make_library(ATTR_TEXT).to_text().splitlines()
    assert "        SurfacePoK  85" in lines
    assert "        国别        美国" in lines


def test_revision_changes_when_an_attr_changes() -> None:
    """属性改了，指纹要跟着变——否则"这次跑的是哪版参数"答不上来。

    症状很隐蔽：只改了一条 PoK 的库报出与上一版**相同的修订号**，于是
    没有任何标记能说明"这次跑的不是同一份数据"。
    """
    before = make_library(ATTR_TEXT)
    after = make_library(ATTR_TEXT.replace("SurfacePoK 85", "SurfacePoK 80"))
    assert before.revision != after.revision


def test_attr_does_not_become_a_component_param() -> None:
    """属性**不进参数面**：装配出来的 params 里没有它，也没有"未知参数"报错。"""
    lib = make_library(ATTR_TEXT)
    radar = radar_of(make_sim(lib))
    assert radar.spec.params["detect_range"] == 150_000.0
    assert "SurfacePoK" not in radar.spec.params
    assert "国别" not in radar.spec.params


def test_attr_never_reaches_the_type_registry() -> None:
    """``seed_into`` **只读 params**。属性进不了类型表，也就进不了注册表。"""
    types = TypeRegistry()
    make_library(ATTR_TEXT).seed_into(types)
    assert "SurfacePoK" not in types.resolve_component_attrs("LIB_TEST_RADAR")


def test_attr_key_is_free_but_param_typo_is_still_loud() -> None:
    """``attr`` 的键**不查参数表**（它的消费者是人和报表）；参数那一侧照查。

    两条路必须分得清：属性若也写进 ``model_param``，第一段会报出一屏
    "未知参数"，第二段那个真正拼错的 ``detect_rnage`` 就淹在里面了。
    """
    good = make_library(
        "component_type LIB_TEST_RADAR\n"
        "    detect_range 150 km\n"
        "    attr\n"
        "        SurfacePoK 85\n"
        "        随便哪个键 1\n"
        "    end_attr\n"
        "end_component_type\n"
    )
    assert good.validate() == []

    typo = make_library(
        "component_type LIB_TEST_RADAR\n"
        "    detect_rnage 150 km\n"
        "    attr\n"
        "        SurfacePoK 85\n"
        "    end_attr\n"
        "end_component_type\n"
    )
    problems = " ".join(typo.validate())
    assert "detect_rnage" in problems          # 参数拼错照样报
    assert "SurfacePoK" not in problems        # 属性永远不进 lint 输出


def test_attr_works_on_platform_models_too() -> None:
    """``platform_type`` 与 ``component_type`` 都收 ``attr``——同一段代码读。"""
    lib = make_library(
        "platform_type LIB_TEST_TANK\n"
        "    side red\n"
        "    attr\n"
        "        服役年份 1998\n"
        "    end_attr\n"
        "end_platform_type\n"
    )
    record = lib.get("LIB_TEST_TANK", KIND_PLATFORM)
    assert record is not None
    assert record.attrs == {"服役年份": Literal("1998")}

    types = TypeRegistry()
    lib.seed_into(types)
    assert "服役年份" not in types.resolve_platform("LIB_TEST_TANK").attributes


def test_attr_survives_the_database(tmp_path: Path) -> None:
    lib = make_library(ATTR_TEXT)
    back = ParamLibrary.from_sqlite(lib.write_sqlite(tmp_path / "models.db"))
    assert back.revision == lib.revision
    assert back.get("LIB_TEST_RADAR").attrs == lib.get("LIB_TEST_RADAR").attrs


def test_library_without_the_attr_table_still_reads(tmp_path: Path) -> None:
    """``model_attr`` 是**后加的侧表**，早先建出来的库没有它。

    缺了只意味着"这个库没有属性"，不是库坏了——这正是
    ``LIBRARY_SCHEMA_VERSION`` 没有动号的理由（改号会让所有既有 .db
    报"需要迁移"，而迁移要做的事其实是零）。这里就是那条理由的可执行版本。
    """
    lib = make_library(ATTR_TEXT)
    file = lib.write_sqlite(tmp_path / "old.db")

    conn = sqlite3.connect(file)
    conn.execute("DROP TABLE model_attr")
    conn.commit()
    conn.close()

    back = ParamLibrary.from_sqlite(file)
    record = back.get("LIB_TEST_RADAR")
    assert record.params["detect_range"] == Literal("150", "km")   # 参数还在
    assert record.attrs == {}                                     # 属性没了
    assert back.revision != lib.revision                          # 指纹自然也不同


def test_attr_after_a_name_is_rejected() -> None:
    """``attr`` 是块，不是带名字的声明——``attr FOO`` 一定不是想写的东西。"""
    with pytest.raises(LibraryError, match="不接名字"):
        make_library(
            "component_type LIB_TEST_RADAR\n"
            "    attr FOO\n"
            "    end_attr\n"
            "end_component_type\n"
        )


def test_two_attr_blocks_are_rejected() -> None:
    """一个型号只写一个 ``attr`` 块；写两个的话，同名属性算哪个生效就得靠猜。"""
    with pytest.raises(LibraryError, match="一个型号只写一个"):
        make_library(
            "component_type LIB_TEST_RADAR\n"
            "    attr\n"
            "        SurfacePoK 85\n"
            "    end_attr\n"
            "    attr\n"
            "        国别 美国\n"
            "    end_attr\n"
            "end_component_type\n"
        )


def test_duplicate_attr_is_rejected() -> None:
    with pytest.raises(LibraryError, match="写了两次"):
        make_library(
            "component_type LIB_TEST_RADAR\n"
            "    attr\n"
            "        SurfacePoK 85\n"
            "        SurfacePoK 80\n"
            "    end_attr\n"
            "end_component_type\n"
        )


def test_block_inside_attr_is_loud() -> None:
    """``attr`` 里不能嵌块——语法层就没给它子块，所以 ``end_component``
    会撞上"结束标记不匹配"。**报了就好**：要紧的是不静默。"""
    with pytest.raises(LibraryError, match="end_attr"):
        make_library(
            "component_type LIB_TEST_RADAR\n"
            "    attr\n"
            "        component r LIB_TEST_RADAR\n"
            "        end_component\n"
            "    end_attr\n"
            "end_component_type\n"
        )


# ---------------------------------------------------------------------------
# 数据库
# ---------------------------------------------------------------------------


def test_sqlite_round_trip(tmp_path: Path) -> None:
    lib = make_library()
    file = lib.write_sqlite(tmp_path / "models.db")
    back = ParamLibrary.from_sqlite(file)

    assert back.revision == lib.revision
    assert len(back) == len(lib)
    assert back.get("LONG_RANGE_RADAR").parent == "LIB_TEST_RADAR"
    # to_text 的抬头带来源，所以要比就比**同来源**的文本
    same_origin = make_library(origin=str(file))
    assert back.to_text() == same_origin.to_text()


def test_sqlite_revision_survives_rebuild(tmp_path: Path) -> None:
    """修订只依赖内容：库文件路径不同、建立时间不同，都不该影响它。"""
    first = make_library(origin="a.txt").write_sqlite(tmp_path / "a.db", )
    second = make_library(origin="b.txt").write_sqlite(tmp_path / "b.db")
    assert ParamLibrary.from_sqlite(first).revision == ParamLibrary.from_sqlite(
        second
    ).revision


def test_revision_changes_when_a_value_changes() -> None:
    before = make_library()
    after = make_library(LIB_TEXT.replace("detect_range  150 km", "detect_range  160 km"))
    assert before.revision != after.revision


def test_write_sqlite_refuses_to_overwrite(tmp_path: Path) -> None:
    """参数库不做隐式覆盖——重建一个库该是显式动作。"""
    file = tmp_path / "models.db"
    make_library().write_sqlite(file)
    with pytest.raises(LibraryError, match="已存在"):
        make_library().write_sqlite(file)
    make_library().write_sqlite(file, overwrite=True)


def test_duplicate_param_is_blocked_by_primary_key(tmp_path: Path) -> None:
    """数据库比文本多一层保护：主键挡住重复参数，不必靠人眼。"""
    file = make_library().write_sqlite(tmp_path / "models.db")
    conn = sqlite3.connect(file)
    try:
        with pytest.raises(sqlite3.IntegrityError):
            conn.execute(
                "INSERT INTO model_param(kind, model, param, value)"
                " VALUES ('component', 'LIB_TEST_RADAR', 'eccm', '9')"
            )
    finally:
        conn.close()


def test_schema_version_mismatch_is_loud(tmp_path: Path) -> None:
    file = tmp_path / "models.db"
    make_library().write_sqlite(file)
    conn = sqlite3.connect(file)
    conn.execute("UPDATE library_meta SET value = '99' WHERE key = 'schema_version'")
    conn.commit()
    conn.close()

    with pytest.raises(LibrarySchemaError, match="结构版本是 99"):
        ParamLibrary.from_sqlite(file)


def test_missing_file_is_loud(tmp_path: Path) -> None:
    with pytest.raises(LibraryError, match="不存在"):
        ParamLibrary.from_sqlite(tmp_path / "nope.db")


def test_foreign_file_is_loud(tmp_path: Path) -> None:
    file = tmp_path / "other.db"
    conn = sqlite3.connect(file)
    conn.execute("CREATE TABLE t(x)")
    conn.commit()
    conn.close()

    with pytest.raises(LibrarySchemaError, match="不像一个参数库"):
        ParamLibrary.from_sqlite(file)


def test_from_path_picks_backend_by_suffix(tmp_path: Path) -> None:
    db = make_library().write_sqlite(tmp_path / "models.db")
    txt = tmp_path / "models.txt"
    txt.write_text(make_library().to_text(), encoding="utf-8")

    assert ParamLibrary.from_path(db).revision == make_library().revision
    assert ParamLibrary.from_path(txt).revision == make_library().revision


# ---------------------------------------------------------------------------
# 合并
# ---------------------------------------------------------------------------


def test_cross_library_duplicate_is_rejected_and_names_both() -> None:
    """覆盖式叠加要显式。默认按报错处理——两个库都给同一型号一套参数时，
    "哪套生效"已经没人说得清了。"""
    base = make_library(origin="base.db")
    extra = make_library(
        "component_type LIB_TEST_RADAR\n    eccm 1\nend_component_type\n",
        origin="extra.db",
    )
    with pytest.raises(DuplicateModelError, match="base.db.*extra.db"):
        base.merged_with(extra)


def test_merge_of_disjoint_libraries_keeps_everything() -> None:
    base = make_library(origin="base.db")
    extra = make_library(
        "component_type OTHER_RADAR\n    eccm 1\nend_component_type\n",
        origin="extra.db",
    )
    merged = base.merged_with(extra)
    assert len(merged) == len(base) + 1
    assert "OTHER_RADAR" in merged.to_text()


# ---------------------------------------------------------------------------
# 灌进类型表
# ---------------------------------------------------------------------------


def test_seeding_makes_library_types_available() -> None:
    lib = make_library()
    types = TypeRegistry()
    assert lib.seed_into(types) == len(lib)
    assert types.has_component_type("LIB_TEST_RADAR")
    assert types.has_component_type("LONG_RANGE_RADAR")
    assert types.has_component_type("LIB_TEST_MOVER")
    assert types.has_platform_type("LIB_TEST_TANK")


def test_scenario_redefining_library_model_is_rejected_with_a_hint() -> None:
    """库里已有这个型号，想定里再定义一次必须报错，并告诉作者怎么改。"""
    text = (
        "component_type LIB_TEST_RADAR\n    eccm 1\nend_component_type\n"
    )
    with pytest.raises(DuplicateTypeError) as info:
        load_scenario(text, library=make_library())
    message = str(info.value)
    assert "参数库" in message
    assert "要改就派生" in message


def test_scenario_can_derive_from_a_library_model() -> None:
    """微调的正确做法是派生——名字上就能看出"这是改过的"。"""
    spec = load_scenario(
        "component_type LIB_TEST_RADAR_LOW_POWER : LIB_TEST_RADAR\n"
        "    detect_range 60 km\n"
        "end_component_type\n",
        library=make_library(),
    )
    assert spec.origin_of("LIB_TEST_RADAR").startswith("<测试库>")
    assert spec.origin_of("LIB_TEST_RADAR_LOW_POWER") == "想定"


def test_library_accepts_a_path_and_a_sequence(tmp_path: Path) -> None:
    db = make_library().write_sqlite(tmp_path / "models.db")
    text = "zone AO\n    anchor 39.9 116.4\n    radius 20 km\nend_zone\n"

    assert len(load_scenario(text, library=str(db)).libraries) == 1
    assert len(load_scenario(text, library=[str(db)]).libraries) == 1


def test_bad_library_argument_is_loud() -> None:
    with pytest.raises(ConfigurationError, match="只接受 ParamLibrary"):
        load_scenario("", library=42)


# ---------------------------------------------------------------------------
# 装配：参数真的生效了，而且是同一套校验
# ---------------------------------------------------------------------------


def test_assembly_uses_library_values_with_units_converted() -> None:
    """150 km → 150000.0 m；1 s → 1000000（**整数**微秒，不走浮点）。"""
    radar = radar_of(make_sim(make_library()))
    params = radar.spec.params
    assert params["detect_range"] == 150_000.0
    assert params["scan_interval"] == 1_000_000
    assert isinstance(params["scan_interval"], int)
    assert params["fov"] == 120.0


def test_without_library_class_defaults_apply() -> None:
    """没有库时的行为与从前完全一样——这是不能破的兼容性。"""
    radar = radar_of(make_sim(None))
    assert radar.spec.params["detect_range"] == 40_000.0
    assert radar.spec.params["eccm"] == 3      # 挂载差量仍然是 3


def test_mount_diff_still_beats_the_library() -> None:
    """优先级：类默认 ← 参数库 ← 想定里的 component_type ← 挂载差量。"""
    radar = radar_of(make_sim(make_library()))
    assert radar.spec.params["eccm"] == 3      # 库里是 2，挂载差量写 3


def test_inheritance_inside_library_works() -> None:
    """派生型号只写差量，其余从**库里的父型号**继承。"""
    sim = make_sim(make_library(), inherited="LONG_RANGE_RADAR")
    inherited = part_of(sim, "long_1")
    params = inherited.spec.params
    assert params["detect_range"] == 300_000.0     # 自己写的
    assert params["fov"] == 120.0                  # 从库里父型号 LIB_TEST_RADAR 继承
    assert params["eccm"] == 2                     # 继承的，**不是** radar_1 槽那个 3
    assert params["bands"] == ("a", "b", "c")


def test_two_slots_of_the_same_family_do_not_share_state() -> None:
    """同一实体上挂两个同族型号：挂载差量只影响那一个槽。"""
    sim = make_sim(make_library(), inherited="LONG_RANGE_RADAR")
    assert radar_of(sim).spec.params["eccm"] == 3
    assert part_of(sim, "long_1").spec.params["eccm"] == 2


def test_boolean_and_multi_and_string_params_survive_the_library() -> None:
    params = radar_of(make_sim(make_library())).spec.params
    assert params["active"] is True
    assert params["bands"] == ("a", "b", "c")
    assert params["mode"] == "1"               # 字符串，不是数


def test_library_values_go_through_the_same_validation() -> None:
    """拼错参数名——库里的值一样要走 ``ParamSet.resolve``，一样给拼写建议。"""
    broken = make_library(
        "component_type LIB_TEST_RADAR\n"
        "    detect_rnage 150 km\n"
        "end_component_type\n"
    )
    assert "是否想写 'detect_range'" in " ".join(broken.validate())

    # 装配层也拦得住：挂上去的那一刻就报，不是等它跑起来
    sim = make_sim(broken, build=False)
    joined = " ".join(sim.spec.validate(sim.factory))
    assert "是否想写 'detect_range'" in joined


def test_out_of_range_library_value_is_loud() -> None:
    broken = make_library(
        "component_type LIB_TEST_RADAR\n    eccm 9\nend_component_type\n"
    )
    assert "不能大于 5" in " ".join(broken.validate())


def test_mounted_library_model_is_validated_at_assembly() -> None:
    """库里的型号**用到哪几条校验哪几条**——不因为"值来自库"就放行。"""
    broken = make_library(
        "component_type LIB_TEST_RADAR\n    eccm 2\nend_component_type\n"
        "component_type LONG_RANGE_RADAR : LIB_TEST_RADAR\n"
        "    eccm 9\n"
        "end_component_type\n"
    )
    sim = make_sim(broken, inherited="LONG_RANGE_RADAR", build=False)
    problems = " ".join(sim.spec.validate(sim.factory))
    assert "不能大于 5" in problems


def test_unknown_unit_in_library_is_loud() -> None:
    broken = make_library(
        "component_type LIB_TEST_RADAR\n"
        "    detect_range 150 光年\n"
        "end_component_type\n"
    )
    problems = " ".join(broken.validate())
    assert "未知单位" in problems


# ---------------------------------------------------------------------------
# 全量 lint
# ---------------------------------------------------------------------------


def test_validate_reports_unregistered_models() -> None:
    lib = make_library(
        "component_type NO_SUCH_CLASS\n    eccm 1\nend_component_type\n"
    )
    problems = " ".join(lib.validate())
    assert "NO_SUCH_CLASS" in problems
    assert "找不到对应的组件实现" in problems


def test_validate_reports_bad_platform_param() -> None:
    """平台属性**有参数表了**（§5.6）：库里写错一个键就报错，不再"无法校验"。"""
    lib = make_library(
        "platform_type LIB_TEST_TANK\n"
        "    sdie red\n"                      # side 拼错
        "end_platform_type\n"
    )
    problems = " ".join(lib.validate())
    assert "LIB_TEST_TANK" in problems
    assert "sdie" in problems
    assert "side" in problems          # 拼写建议
    assert "无法校验" not in problems


def test_validate_accepts_platform_models() -> None:
    """库里的平台型号参数合法时**一条问题都不报**——"能校验"而不是"跳过了"。"""
    lib = make_library(
        "platform_type LIB_TEST_TANK\n"
        "    side red\n"
        "end_platform_type\n"
    )
    assert lib.validate() == []


def test_validate_is_clean_for_a_good_library_of_components() -> None:
    lib = make_library(
        "component_type LIB_TEST_RADAR\n"
        "    detect_range 150 km\n"
        "    eccm 2\n"
        "end_component_type\n"
        "component_type LONG_RANGE_RADAR : LIB_TEST_RADAR\n"
        "    detect_range 300 km\n"
        "end_component_type\n"
        "component_type LIB_TEST_MOVER\n"
        "    max_speed 30 m/s\n"
        "end_component_type\n"
    )
    assert lib.validate() == []


# ---------------------------------------------------------------------------
# 可复现
# ---------------------------------------------------------------------------


def test_revision_reaches_the_report() -> None:
    """库是想定文件**之外**的输入。结果里必须能查到用的是哪一版参数。"""
    lib = make_library()
    sim = make_sim(lib)
    sim.initialize()
    report = sim.shutdown()
    assert report.library_revision == lib.revision
    assert lib.revision in report.summary()


def test_no_library_means_empty_revision() -> None:
    sim = make_sim(None)
    sim.initialize()
    report = sim.shutdown()
    assert report.library_revision == ""
    assert "参数库" not in report.summary()


def test_summary_separates_library_types_from_scenario_types() -> None:
    spec = load_scenario("", library=make_library())
    assert "其中 3 个来自库" in spec.summary()
    assert f"修订 {make_library().revision}" in spec.summary()


def test_sorted_records_is_independent_of_insertion_order() -> None:
    """顺序必须稳定：修订、``to_text``、错误信息都依赖它。"""
    forward = make_library()
    backward = ParamLibrary(
        list(reversed(forward.sorted_records())), origin=forward.origin
    )
    assert backward.to_text() == forward.to_text()
    assert backward.revision == forward.revision


# ---------------------------------------------------------------------------
# 命令行工具：build 不做隐式重建
# ---------------------------------------------------------------------------


def _load_tool():
    """把 ``tools/model_library.py`` 当模块加载。

    工具是参数库的命令行外壳（build / dump / lint / show），它那两条保护
    ——"不静默抹掉导入工具写进去的型号"、"dump 写文件而不是靠 shell 重定向"
    ——守的是**数据**，值得钉住。
    """
    path = Path(__file__).resolve().parents[1] / "tools" / "model_library.py"
    spec = importlib.util.spec_from_file_location("_model_library_tool", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_refuses_to_drop_models_it_did_not_grow(tmp_path, capsys) -> None:
    """库**不是只由文本长出来的**：导入工具会直接往库文件里写型号。

    拿一份文本 ``build`` 上去会 DROP 重建，那些型号**静默消失**——症状要到
    某份想定突然报"型号不存在"时才出现，那时候没人会想到是上一次 build
    干的。所以默认拒绝，要重建得显式加 ``--force``。
    """
    tool = _load_tool()
    source = tmp_path / "lib.txt"
    source.write_text(LIB_TEXT, encoding="utf-8")
    target = tmp_path / "models.db"

    assert tool.cmd_build([str(source), "-o", str(target)]) == 0

    # 模拟"导入工具写进去的型号"
    imported = make_library(
        "component_type IMPORTED_RADAR\n    eccm 1\nend_component_type\n"
    )
    ParamLibrary.from_sqlite(target).merged_with(imported).write_sqlite(
        target, overwrite=True
    )
    assert ParamLibrary.from_sqlite(target).has("IMPORTED_RADAR")

    capsys.readouterr()
    assert tool.cmd_build([str(source), "-o", str(target)]) == 1
    assert "IMPORTED_RADAR" in capsys.readouterr().err
    assert ParamLibrary.from_sqlite(target).has("IMPORTED_RADAR")   # 没被删

    assert tool.cmd_build([str(source), "-o", str(target), "--force"]) == 0
    assert not ParamLibrary.from_sqlite(target).has("IMPORTED_RADAR")
