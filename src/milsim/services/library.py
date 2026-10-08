"""参数库：型号参数存进数据库，想定只写型号名（§5.5）。

要解决的问题
------------
同一型雷达的参数会出现在几十份想定里。抄来抄去的结果是**同一型号有几十份
可能不一致的副本**，而且没人说得清哪份是对的。参数库把"型号 → 参数"抽出来
存一次，想定里只写型号名：

.. code-block:: text

    platform SAM_1 SAM_BATTALION
        component radar_1 AN_APG_77      # ← 参数在库里，这里不写
    end_platform

一条不能破的纪律
----------------
**数据库只管值，schema 留在代码里。**

型号的参数表（有哪些参数、什么量纲、取值白名单、范围）来自组件类的
``PARAMS``。库里的值最终仍然走 :meth:`~milsim.services.params.ParamSet.resolve`
那一条路——量纲换算、单位校验、白名单、范围、拼写建议一个都不少。

反过来说，如果让数据库自己定义"有哪些参数"（一张宽表、或者无类型的
键值对），这个项目最值钱的属性就没了：``detec_range`` 拼错一个字母会
**静默失效**，而不是报"是否想写 'detect_range'？"。§3.11.3 对网格通道
说过同一句话——通道必须有清单，否则退化成无类型字典。

所以库里存的是**字面量文本**（人写得出的东西），不是已经换算好的内部值：

.. code-block:: text

    model_param: detect_range | 150 | km

存 ``150 / km`` 而不是 ``150000.0``，是为了让人能读、能改、能在 code review
里看出"这型雷达的探测距离被人改过"。

``attr``：存得下、但**不进参数面**
----------------------------------
外部资料（装备手册、推演软件导出）常常带一批**组件现在没有消费者**的量：
毁伤概率、圆概率误差、装甲、发射包络、国别、服役年份……把它们塞进
``model_param`` 是不行的——那一张表的每一行都要能对上组件 ``PARAMS`` 里的
一个键，塞进去只会让 ``lint`` 报一屏"未知参数"，而真正拼错的 ``detec_range``
就淹在里面了。

所以它们进**另一张表** ``model_attr``，文本侧写在一个 ``attr`` 内层块里：

.. code-block:: text

    component_type MOZI_W_000000000003 : MISSILE_MOVER
        note "AGM-84A 型“鱼叉”反舰导弹"
        mass 522 kg
        attr
            SurfacePoK 85
            国别        美国
        end_attr
    end_component_type

三条纪律：

- **``attr`` 里的键是自由的**（没有 PARAMS 可查），所以拼错不报错——它的
  消费者是"人"和报表/前端，不是 ``ParamSet.resolve``。这与参数那一侧
  **故意不同**，别指望它给拼写建议。
- **它进不了类型注册表**：``seed_into`` 只读 ``params``。两张表分开正是
  为了这件事——同一张表加个标志位的话，将来某次改查询忘了带条件，
  属性就会当成参数灌进注册表，症状是"想定里多了一堆未知参数"。
- **修订号包含它**：改了属性值，库的指纹要跟着变，否则"这次跑的是哪版
  参数"就答不上来了。

装载顺序
--------
:meth:`ParamLibrary.seed_into` 把库里的型号灌进 ``TypeRegistry``，**等同于
想定里写了一堆 ``component_type`` 块**——只是这些块来自数据库。于是参数
解析链一行都不用改，继承（``parent``）、覆盖、白名单全都自动成立：

.. code-block:: text

    类默认值（PARAMS）  ←  参数库（本模块）  ←  想定里的 component_type  ←  挂载差量

**库与想定同名怎么办：报错，不合并。** 库已经给了这个型号的参数，想定里
再定义一个同名型号，两处的值哪个生效就该靠猜了。要改就在想定里派生：
``component_type AN_APG_77_TUNED : AN_APG_77``，名字上就能看出"这是改过的"。

两个库同名同理——报错并指出两个库各自是谁。"基础库 + 战役修正库"这种
覆盖式叠加看起来方便，但它是**静默的**：翻十份文件都看不出某个值到底
以哪份为准。真需要时再显式加参数，不要默认。
"""

from __future__ import annotations

import hashlib
import re
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping, Sequence
from unicodedata import east_asian_width

from ..errors import ConfigurationError
from .input import LexError, ParseError, is_bare_word, parse
from .params import WithUnit
from .type_registry import ComponentFactory, TypeRegistry

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

#: 型号种类。与想定里的块关键字一一对应。
KIND_COMPONENT = "component"
KIND_PLATFORM = "platform"
KINDS: tuple[str, ...] = (KIND_COMPONENT, KIND_PLATFORM)

#: 种类 → 想定块关键字。库文本用的就是这套语法，不另造格式。
KIND_KEYWORDS: dict[str, str] = {
    KIND_COMPONENT: "component_type",
    KIND_PLATFORM: "platform_type",
}
KEYWORD_KINDS: dict[str, str] = {v: k for k, v in KIND_KEYWORDS.items()}

#: 库专用键：属于**元数据**，不参与参数校验。
#:
#: 为什么要有 ``source``：军事模型的参数值可信度差别极大——"公开资料
#: 估算"和"实测标定"是两个性质的东西，而它们在数据库里长得一模一样。
#: 半年后有人问"这个 150 km 哪来的"，没有这一列就只能靠猜。
METADATA_KEYS: tuple[str, ...] = ("note", "source")

#: 内层属性块的块名。块内的键**不查参数表**——见模块头"``attr``：存得下、
#: 但不进参数面"。
ATTR_KEYWORD = "attr"

#: 数据库结构版本。将来改表结构时用它决定怎么读旧库。
#:
#: 新增 ``model_attr`` 时**没有**动这个号：它是一张可选的侧表，缺了只意味着
#: "这个库没有属性"，旧库照样读得对。改号会让所有既有 .db 报"需要迁移"，
#: 而迁移要做的事其实是零。
LIBRARY_SCHEMA_VERSION = 1

_INT_RE = re.compile(r"[+-]?\d+")
_FLOAT_RE = re.compile(r"[+-]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?")


# ---------------------------------------------------------------------------
# 错误
# ---------------------------------------------------------------------------


class LibraryError(ConfigurationError):
    """参数库本身的错误：文件读不了、表结构不对、型号重复。"""


class LibrarySchemaError(LibraryError):
    """库的表结构不是本版本认识的样子。"""


class DuplicateModelError(LibraryError):
    """同一型号被声明了两次（同库内或跨库）。

    不静默取后者：两个来源都写了同一个型号，说明"哪个值生效"这件事
    已经没人知道了。
    """


# ---------------------------------------------------------------------------
# 数据
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Literal:
    """一个参数的**字面量**：值文本 + 单位（+ 是否强制按文本解释）。

    ``unit`` 非空时值必须是数值，读出来是 :class:`WithUnit`——与想定解析器
    产出的东西完全一样，所以后续的 :meth:`ParamSet.resolve` 分不出这个值
    是从想定来的还是从库来的，也不必分。

    ``as_text`` 的含义只有一条：**这个值回写时要不要加引号**。

    什么时候需要加：值会被词法层读成别的东西时——长得像数值（``"1"``）、
    含空白（会被切成多值）、含不在裸名字符集里的字符（``（``、``,``）。
    判据在 :func:`~milsim.services.input.lexer.is_bare_word`，就是词法层
    自己的那份定义。

    这不是个边角功能。中文括号、逗号在备注和型号名里很常见，而
    ``to_text`` 漏加一个引号，库文件就**整个读不回来**——是逐字往返
    测试抓出来的，不是看代码看出来的。
    """

    value: str
    unit: str = ""
    as_text: bool = False

    def __post_init__(self) -> None:
        if not str(self.value).strip():
            raise LibraryError("参数值不能为空")
        if self.as_text and self.unit:
            raise LibraryError(
                f"值 {self.value!r} 既标为文本又带单位 {self.unit!r}，两者矛盾"
            )

    def describe(self) -> str:
        if self.unit:
            return f"{self.value} {self.unit}"
        if self.as_text:
            return f'"{self.value}"'
        return self.value


@dataclass(frozen=True, slots=True)
class ModelRecord:
    """库里的一行型号。``params`` 是**原始字面量**，未经换算。

    ``attrs`` 与 ``params`` 的区别只有一条：**``attrs`` 不查参数表、不进
    类型注册表**（见模块头）。所以两者必须是两个字段而不是"一个字典 +
    标志位"——一个字段的话，某次改查询忘了带条件就会把属性当参数灌进去。
    """

    kind: str
    name: str
    params: Mapping[str, Literal] = None  # type: ignore[assignment]
    parent: str | None = None
    note: str = ""
    source: str = ""
    line: int = 0
    attrs: Mapping[str, Literal] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise LibraryError(
                f"型号种类只能是 {'、'.join(KINDS)}，收到 {self.kind!r}"
            )
        if not self.name:
            raise LibraryError("型号名不能为空")
        if self.params is None:
            object.__setattr__(self, "params", {})
        if self.attrs is None:
            object.__setattr__(self, "attrs", {})

    def __str__(self) -> str:
        keyword = KIND_KEYWORDS[self.kind]
        parent = f" : {self.parent}" if self.parent else ""
        extra = f"，{len(self.attrs)} 条属性" if self.attrs else ""
        return f"{keyword} {self.name}{parent}（{len(self.params)} 个参数{extra}）"


# ---------------------------------------------------------------------------
# 字面量 ↔ 值
# ---------------------------------------------------------------------------


def _split_items(body: str, separators: str) -> tuple[str, ...]:
    """按分隔符切分多值文本，**引号内的分隔符不算**，并去掉外层引号。

    为什么不能直接 ``split()``：:func:`literal_of` 会给含空格、逗号或长得
    像数值的元素加引号，而引号里面恰恰可能含分隔符。粗暴切分的后果是
    "库里存的是一个值、读出来成了两个"——静默的列表丢项，比报错难查得多。
    """
    items: list[str] = []
    current: list[str] = []
    quoted = False

    for char in body:
        if char == '"':
            quoted = not quoted
            continue
        if not quoted and char in separators:
            if current:
                items.append("".join(current))
                current = []
            continue
        current.append(char)

    if current:
        items.append("".join(current))
    return tuple(items)


def parse_literal(literal: Literal) -> Any:
    """字面量 → 想定解析器会产出的那种 Python 值。

    **按语法解释，不查参数表。** 判据只有文本本身：

    ==================  ==========================
    写法                 解释
    ==================  ==========================
    ``150 km``           ``WithUnit(150.0, "km")``
    ``40``               整数
    ``0.5``              浮点
    ``true`` / ``false`` 布尔（大小写不敏感）
    ``a b c``            三元素元组（多值参数）
    ``[a]``              单元素元组
    ``[]``               空元组
    ``procedural``       字符串
    ``"40"``             字符串（``as_text``，强制）
    ==================  ==========================

    方括号那两行是必须的：空格形式下"一个元素"与"没有元素"都写不出来
    （前者被读成字符串、后者是空值），所以这两个只能靠 ``[...]``。

    为什么不查参数表：装载参数库时**组件类可能还没注册**（库里有 200 个
    型号，这次只想定只用得上 30 个）。查表就得给未注册的型号安排一条
    降级路径，而两条路径迟早会不一致。按语法解释只有一条路径，代价是
    上面那个 ``as_text`` 的例外——它罕见，且报错时看得到值长什么样。
    """
    text = literal.value

    if literal.as_text:
        return text

    if literal.unit:
        try:
            return WithUnit(float(text), literal.unit)
        except ValueError as exc:
            raise LibraryError(
                f"值 {text!r} 带了单位 {literal.unit!r}，但它不是数值——"
                "单位要写在单独的单位列里，值本身只能是数字"
            ) from exc

    if text in ("true", "false"):
        return text == "true"

    # 整数优先：`40` 不该变成 40.0，否则整数参数会报"需要整数"
    if _INT_RE.fullmatch(text):
        return int(text)
    if _FLOAT_RE.fullmatch(text):
        return float(text)

    # 列表。**必须先于空格切分判断**：空格形式表达不了"空列表"与"一个
    # 元素"——`()` 会回写成空串、`(x,)` 会回写成 `x`（读回来是字符串，
    # 不再是列表）。那两种只有 `[...]` 写得出来，见 :func:`literal_of`。
    if len(text) >= 2 and text.startswith("[") and text.endswith("]"):
        return _split_items(text[1:-1], ", \t")

    items = _split_items(text, " \t")
    if len(items) > 1:
        return items
    if items:
        return items[0]
    return text


def _element_text(part: Literal) -> str:
    """列表里一个元素的写法。

    用 ``describe()``（会补引号）而不是 ``value``：元素含空格、逗号或长得像
    数值时，裸写出去会被读成别的东西——``("某 型",)`` 回写成 ``某 型`` 就成
    了两个元素，而 :func:`parse_literal` 不会报错，只会给出一个更长的列表。

    双引号是唯一装不下的东西：想定语言没有转义序列，所以 ``"`` 一旦出现在
    元素里，引号配对就错了，切分结果无法预料。与其静默损坏，不如拒绝回写。
    """
    if '"' in part.value:
        raise LibraryError(
            f"列表元素 {part.value!r} 里含双引号——想定语言没有转义序列，"
            "写出去读回来会切错。请改用别的字符。"
        )
    return part.describe()


def literal_of(value: Any) -> Literal:
    """:meth:`parse_literal` 的逆：解析器产出的值 → 字面量。

    用于把想定片段收编成参数库（见 :meth:`ParamLibrary.from_text` 的
    姊妹路径 ``from_blocks``），以及 :meth:`ParamLibrary.to_text` 的回写。
    """
    if isinstance(value, WithUnit):
        return Literal(_number_text(value.value), value.unit or "")

    if isinstance(value, bool):
        return Literal("true" if value else "false")

    if isinstance(value, (int, float)):
        return Literal(_number_text(value))

    if isinstance(value, str):
        # 会被读成别的东西的值必须标成 as_text——**判据只有一份**，
        # 在词法层（is_bare_word）。这里自己写一遍字符集，两边迟早漂移，
        # 而漂移的后果是库文件读不回来
        return Literal(value, as_text=not is_bare_word(value))

    if isinstance(value, Sequence):
        parts = [literal_of(item) for item in value]
        units = {part.unit for part in parts if part.unit}
        if units:
            raise LibraryError(
                f"多值参数不支持单位（收到 {units}）——"
                "需要单位说明它是数值，那就不是多值参数"
            )
        # 用 describe() 而不是 value：值里含空格、逗号或长得像数值时必须
        # 带引号，否则回写后会**变成好几个元素**（或被读成数值）——那是
        # 静默的数据损坏，比报错难查得多。
        texts = [_element_text(part) for part in parts]
        # 0 个或 1 个元素**必须写成 [...]**：空格形式下 `()` 回写成空串
        # （Literal 拒绝空值，"把想定片段收编进库"会整个失败），`(x,)`
        # 回写成 `x`——读回来是单个字符串、不再是列表，于是 `strings`
        # 参数报"需要列表"。两处都是**回写丢信息**。
        if len(texts) < 2:
            return Literal("[" + ", ".join(texts) + "]")
        return Literal(" ".join(texts))

    raise LibraryError(
        f"不支持的字面量类型 {type(value).__name__}：{value!r}"
    )


def _quote_body(text: str) -> str:
    """把文本放进双引号里，顺带挡住两种**装不下**的内容。

    想定语言没有转义序列，所以文本里既不能有双引号，也不能换行。这两种
    情况在库里能存（数据库是自由的），但**回写成文本会丢信息**。丢了不报错
    才是最难查的：一个备注被截断，下次读回来就是另一个值。所以这里直接
    拒绝回写，而不是悄悄截掉。
    """
    if '"' in text:
        raise LibraryError(
            f"文本 {text!r} 里含双引号——想定语言没有转义，库文本装不下它。"
            "请改用中文引号，或只把它留在数据库里"
        )
    if "\n" in text or "\r" in text:
        raise LibraryError(
            f"文本 {text!r} 里有换行——库文本一行一个参数，装不下多行内容"
        )
    return text


def _number_text(value: Any) -> str:
    """数值 → 文本。整数不写成 ``40.0``，浮点用最短表示。"""
    if isinstance(value, bool):  # bool 是 int 的子类，先拦
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    number = float(value)
    if number.is_integer() and abs(number) < 1e15:
        # 1.0 → "1"。微秒这类整数参数不能写成 1.0，否则读回来是浮点，
        # 而 ParamSpec 的整数校验会拒绝它
        return str(int(number))
    return repr(number)


def _display_width(text: str) -> int:
    """文本在等宽终端里占几列。**中文算两列。**

    ``str.ljust`` 数的是字符数，中日韩字符一个字符占两列——用 ``ljust`` 对齐
    的列在终端里会从第一个中文键开始整列右移。参数名是 ASCII 时看不出问题，
    而 ``attr`` 的键恰恰经常是中文（``国别`` / ``服役年份`` / ``类别``），
    一列参差不齐的键值对在读第二遍时就变成噪音了。
    """
    return sum(2 if east_asian_width(ch) in "WF" else 1 for ch in text)


def _pad(text: str, width: int) -> str:
    """按**显示宽度**左对齐补空格。"""
    return text + " " * max(0, width - _display_width(text))


# ---------------------------------------------------------------------------
# 库
# ---------------------------------------------------------------------------


class ParamLibrary:
    """一个只读的参数库。型号 → 参数（字面量）。

    构造入口有三条：:meth:`from_sqlite`（正常用法）、:meth:`from_text`
    （文本形式，可 diff、可 code review）、:meth:`from_blocks`（把一份
    想定片段直接当库用，用于把历史想定里的型号抽出来）。
    """

    __slots__ = ("_models", "origin", "_revision", "_text_backed")

    def __init__(
        self,
        records: Iterable[ModelRecord] = (),
        *,
        origin: str = "<参数库>",
        text_backed: bool = False,
    ) -> None:
        self.origin = origin
        #: 行号是否指向 ``origin`` 里真实存在的行。文本库为真（能报"第 12 行"），
        #: 数据库为假——库里存的行号指向的是**当初写文本时**那一行，
        #: 库文件里并没有那一行，报出来只会让作者去数 .db 的行。
        self._text_backed = text_backed
        self._models: dict[tuple[str, str], ModelRecord] = {}
        self._revision = ""
        for record in records:
            key = (record.kind, record.name)
            existing = self._models.get(key)
            if existing is not None:
                raise DuplicateModelError(
                    f"{origin}：型号 {record.name!r}（{record.kind}）重复定义"
                    f"（第一次在第 {existing.line} 行）——"
                    "同库内重名会静默覆盖，必须显式改名"
                )
            self._models[key] = record

    # -- 构造 --------------------------------------------------------------

    @classmethod
    def from_text(cls, text: str, *, origin: str = "<文本库>") -> "ParamLibrary":
        """库文本 → 库。

        格式就是**想定语法**（``component_type`` / ``platform_type`` 块），
        外加两个库专用键 ``note`` / ``source``。不另造一套格式的理由：
        这套语法已经把继承、单位、行号、报错都写好了，再造一套只会让
        两边的行为慢慢分叉。

        代价是库文本**不能直接当想定用**（``note`` 会被当成未知参数）——
        这是有意的：库元数据属于库，想定里写它没有意义。
        """
        try:
            blocks = parse(text)
        except (ParseError, LexError) as exc:
            raise LibraryError(f"{origin}：解析失败——{exc}") from exc

        records: list[ModelRecord] = []
        for block in blocks:
            kind = KEYWORD_KINDS.get(block.keyword)
            if kind is None:
                allowed = " / ".join(sorted(KEYWORD_KINDS))
                raise LibraryError(
                    f"{origin} 第 {block.line} 行：参数库只认 {allowed} 块，"
                    f"收到 {block.keyword!r}——库不是想定，"
                    "zone / platform / command 之类写在这里没人读"
                )
            records.append(_record_from_block(block, kind, origin))
        return cls(records, origin=origin, text_backed=True)

    @classmethod
    def from_sqlite(cls, path: str | Path) -> "ParamLibrary":
        """读一个库文件。以**只读**方式打开——装载不该改动数据。"""
        file = Path(path)
        if not file.exists():
            raise LibraryError(f"参数库文件不存在：{file}")
        try:
            conn = sqlite3.connect(f"file:{file}?mode=ro", uri=True)
        except sqlite3.Error as exc:
            raise LibraryError(f"打开参数库 {file} 失败：{exc}") from exc

        try:
            conn.row_factory = sqlite3.Row
            return cls(_read_records(conn, origin=str(file)), origin=str(file))
        finally:
            conn.close()

    @classmethod
    def from_path(cls, path: str | Path) -> "ParamLibrary":
        """按后缀选：``.db`` / ``.sqlite`` 走数据库，其它当文本库。"""
        file = Path(path)
        if file.suffix.lower() in (".db", ".sqlite", ".sqlite3"):
            return cls.from_sqlite(file)
        if not file.exists():
            raise LibraryError(f"参数库文件不存在：{file}")
        return cls.from_text(file.read_text(encoding="utf-8"), origin=str(file))

    @classmethod
    def from_blocks(
        cls, blocks: Sequence[Any], *, origin: str = "<片段>"
    ) -> "ParamLibrary":
        """把一份想定片段里的型号声明抽成库（用于收编历史想定）。"""
        records = [
            _record_from_block(block, KEYWORD_KINDS[block.keyword], origin)
            for block in blocks
            if block.keyword in KEYWORD_KINDS
        ]
        return cls(records, origin=origin, text_backed=True)

    # -- 查询 --------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._models)

    def __iter__(self) -> Iterator[ModelRecord]:
        return iter(self.sorted_records())

    def sorted_records(self) -> list[ModelRecord]:
        """按 (种类, 名字) 排序。**顺序稳定**，与插入顺序和哈希无关。"""
        return [self._models[key] for key in sorted(self._models)]

    def records(self, kind: str | None = None) -> list[ModelRecord]:
        return [r for r in self.sorted_records() if kind is None or r.kind == kind]

    def get(self, name: str, kind: str = KIND_COMPONENT) -> ModelRecord | None:
        return self._models.get((kind, name))

    def has(self, name: str, kind: str = KIND_COMPONENT) -> bool:
        return (kind, name) in self._models

    @property
    def revision(self) -> str:
        """内容指纹。用于把"这次跑用的是哪版参数"记进结果。

        **必须只依赖内容**，不依赖路径、时间、插入顺序——两次构造出的
        相同内容要给出同一个 revision，否则它没法用来判断"参数变没变"。
        """
        if not self._revision:
            digest = hashlib.sha256()
            for record in self.sorted_records():
                digest.update(
                    f"{record.kind}|{record.name}|{record.parent or ''}|"
                    f"{record.note}|{record.source}\n".encode("utf-8")
                )
                for name in sorted(record.params):
                    digest.update(
                        f"  {name}={record.params[name].describe()}\n".encode("utf-8")
                    )
                # 属性也进指纹：它同样是"这次跑用的是哪版库"的一部分。
                # 漏掉它的话，只改了一条 PoK 的库会报出与上一版相同的修订号，
                # 于是没有任何标记能说明"这次跑的不是同一份数据"
                for name in sorted(record.attrs):
                    digest.update(
                        f"  @{name}={record.attrs[name].describe()}\n".encode("utf-8")
                    )
            self._revision = digest.hexdigest()[:12]
        return self._revision

    # -- 合并 --------------------------------------------------------------

    def merged_with(self, other: "ParamLibrary") -> "ParamLibrary":
        """两个库合成一个。**同名报错**——见模块头对覆盖式叠加的判断。"""
        clash = set(self._models) & set(other._models)
        if clash:
            kind, name = sorted(clash)[0]
            raise DuplicateModelError(
                f"型号 {name!r}（{kind}）在 {self.origin} 与 {other.origin} "
                f"里都有定义——两个库对同一个型号给了两套参数，"
                f"哪套生效不能靠猜。共 {len(clash)} 个冲突"
            )
        return ParamLibrary(
            list(self._models.values()) + list(other._models.values()),
            origin=f"{self.origin} + {other.origin}",
        )

    def origin_of(self, record: ModelRecord) -> str:
        """某个型号的出处文本，用于报错。"""
        if self._text_backed and record.line:
            return f"{self.origin} 第 {record.line} 行"
        return self.origin

    def _scratch_types(self) -> TypeRegistry:
        """灌过本库的**独立**类型表。校验时用它，不借用别人的。"""
        types = TypeRegistry()
        self.seed_into(types)
        return types

    # -- 装载 --------------------------------------------------------------

    def seed_into(self, types: TypeRegistry) -> int:
        """把库里的型号灌进类型注册表。返回灌进去的型号数。

        灌进去之后，``types`` 里这些型号与想定里写的 ``component_type``
        **没有任何区别**：继承、参数解析、校验走的是同一段代码。

        同名冲突由注册表报（想定里又定义了一次），错误信息会带上 ``origin``。
        """
        count = 0
        for record in self.sorted_records():
            origin = self.origin_of(record)
            attrs = {
                name: parse_literal(literal)
                for name, literal in record.params.items()
            }
            if record.kind == KIND_COMPONENT:
                types.define_component_type(
                    record.name,
                    parent=record.parent,
                    attrs=attrs,
                    line=record.line,
                    origin=origin,
                )
            else:
                types.define_platform_type(
                    record.name,
                    parent=record.parent,
                    attributes=attrs,
                    line=record.line,
                    origin=origin,
                )
            count += 1
        return count

    # -- 校验 --------------------------------------------------------------

    def validate(self, components: Any = None) -> list[str]:
        """把库里的每一个型号都过一遍参数校验。返回问题列表。

        **自己造一个灌过本库的类型表**，不复用调用方的 factory。这不是洁癖：
        拿一个没灌过这个库的 factory 来校验，库里的派生型号（``A : B``）
        会断在 ``B`` 上，报出"找不到实现类"——而真正的问题可能只是某个参数
        值超了范围。错误信息指错了地方，比不报还费时间。

        ``components`` 是代码层的组件注册表，不给就用全局默认的那个。传它
        的理由只有一个：本次进程里额外注册了插件组件，想让库里的型号也能
        解析到它们。

        正常装配只校验"用到的那些型号"，这里是全量检查，给 lint 用——库里
        有 200 个型号、这次只用 3 个时，另外 197 个里的错值不会被装配发现，
        但它们**已经在库里了**。

        组件型号与平台型号走**同一套** ``ParamSet.resolve``（§5.5.1、§5.6）。
        唯一会"校验不到"的情形是型号沿继承链追不到实现类（型号名拼错，
        或那个类还没 import）——这种情况如实报出来，不假装干净。
        """
        factory = ComponentFactory(components, self._scratch_types())
        problems: list[str] = []

        for record in self.sorted_records():
            values = {
                name: parse_literal(literal)
                for name, literal in record.params.items()
            }
            try:
                if record.kind == KIND_COMPONENT:
                    param_set = factory.param_set(record.name)
                else:
                    param_set = factory.platform_param_set(record.name)
            except ConfigurationError as exc:
                what = "组件" if record.kind == KIND_COMPONENT else "平台"
                problems.append(
                    f"{self.origin} 型号 {record.name!r}：找不到对应的{what}实现"
                    f"（{exc}）"
                )
                continue

            try:
                param_set.resolve(values, owner=f"型号 {record.name}")
            except ConfigurationError as exc:
                problems.append(f"{self.origin} 型号 {record.name!r}：{exc}")

        return problems

    # -- 输出 --------------------------------------------------------------

    def to_text(self) -> str:
        """库 → 文本。**逐字节可回读**（:meth:`from_text` 的逆）。

        这是让参数库能进版本控制的那一半：数据库是二进制的，diff 不出来，
        code review 也看不见。文本是**给人看的那一份**。
        """
        lines: list[str] = [
            f"# 参数库：{self.origin}",
            f"# 型号 {len(self)} 个，修订 {self.revision}",
            "",
        ]
        for record in self.sorted_records():
            head = KIND_KEYWORDS[record.kind] + " " + record.name
            if record.parent:
                head += f" : {record.parent}"
            lines.append(head)
            # note / source 与参数**对齐到同一列**：文本是要给人看的，
            # 一列参差不齐的键值对在读第二遍时就变成噪音了
            names = list(record.params) + [k for k in METADATA_KEYS if getattr(record, k)]
            width = max((_display_width(n) for n in names), default=0)
            if record.note:
                lines.append(
                    f'    {_pad("note", width)}  "{_quote_body(record.note)}"'
                )
            if record.source:
                lines.append(
                    f'    {_pad("source", width)}  "{_quote_body(record.source)}"'
                )
            for name in sorted(record.params):
                lines.append(
                    f"    {_pad(name, width)}  {record.params[name].describe()}"
                )
            if record.attrs:
                # 属性块内部的宽度**单独算**：混进参数列的宽度会让参数那一列
                # 为了迁就一个长属性名而整体右移，读起来反而更散
                lines.append(f"    {ATTR_KEYWORD}")
                attr_width = max(_display_width(name) for name in record.attrs)
                for name in sorted(record.attrs):
                    lines.append(
                        f"        {_pad(name, attr_width)}  "
                        f"{record.attrs[name].describe()}"
                    )
                lines.append(f"    end_{ATTR_KEYWORD}")
            lines.append(f"end_{KIND_KEYWORDS[record.kind]}")
            lines.append("")
        return "\n".join(lines)

    def write_sqlite(self, path: str | Path, *, overwrite: bool = False) -> Path:
        """落成一个库文件。

        ``overwrite`` 时**在同一次连接里 DROP 重建**，不删文件：删了再建
        有一个"文件在某一刻不存在"的窗口，别人（或另一个进程）正好读到
        就是"参数库不存在"。DROP 没有这个窗口。
        """
        file = Path(path)
        if file.exists() and not overwrite:
            raise LibraryError(
                f"{file} 已存在。参数库**不做隐式覆盖**——"
                "先确认这是要重建的库，或者换个文件名"
            )

        conn = sqlite3.connect(file)
        try:
            if overwrite:
                _reset_schema(conn)
            conn.executescript(_SCHEMA)
            conn.executemany(
                "INSERT INTO library_meta(key, value) VALUES (?, ?)",
                [
                    ("schema_version", str(LIBRARY_SCHEMA_VERSION)),
                    ("revision", self.revision),
                    ("origin", self.origin),
                ],
            )
            for record in self.sorted_records():
                conn.execute(
                    "INSERT INTO model(kind, name, parent, note, source, line)"
                    " VALUES (?, ?, ?, ?, ?, ?)",
                    (
                        record.kind,
                        record.name,
                        record.parent,
                        record.note,
                        record.source,
                        record.line,
                    ),
                )
                conn.executemany(
                    "INSERT INTO model_param(kind, model, param, value, unit,"
                    " as_text) VALUES (?, ?, ?, ?, ?, ?)",
                    [
                        (
                            record.kind,
                            record.name,
                            name,
                            literal.value,
                            literal.unit,
                            1 if literal.as_text else 0,
                        )
                        for name, literal in sorted(record.params.items())
                    ],
                )
                if record.attrs:
                    conn.executemany(
                        "INSERT INTO model_attr(kind, model, attr, value, unit,"
                        " as_text) VALUES (?, ?, ?, ?, ?, ?)",
                        [
                            (
                                record.kind,
                                record.name,
                                name,
                                literal.value,
                                literal.unit,
                                1 if literal.as_text else 0,
                            )
                            for name, literal in sorted(record.attrs.items())
                        ],
                    )
            conn.commit()
        finally:
            conn.close()
        return file

    def describe(self) -> str:
        parts = []
        for kind in KINDS:
            count = sum(1 for r in self._models.values() if r.kind == kind)
            if count:
                parts.append(f"{count} 个{'组件' if kind == KIND_COMPONENT else '平台'}型号")
        params = sum(len(r.params) for r in self._models.values())
        attrs = sum(len(r.attrs) for r in self._models.values())
        return (
            f"{self.origin}（修订 {self.revision}）："
            + "、".join(parts or ["空库"])
            + f"，共 {params} 个参数"
            + (f"、{attrs} 条附加属性" if attrs else "")
        )

    def __repr__(self) -> str:
        return f"<ParamLibrary {self.origin} {len(self)} 个型号>"


# ---------------------------------------------------------------------------
# 表结构
# ---------------------------------------------------------------------------

_SCHEMA = """
CREATE TABLE library_meta(
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE TABLE model(
    kind   TEXT NOT NULL,
    name   TEXT NOT NULL,
    parent TEXT,
    note   TEXT NOT NULL DEFAULT '',
    source TEXT NOT NULL DEFAULT '',
    line   INTEGER NOT NULL DEFAULT 0,
    PRIMARY KEY (kind, name)
);

CREATE TABLE model_param(
    kind    TEXT NOT NULL,
    model   TEXT NOT NULL,
    param   TEXT NOT NULL,
    value   TEXT NOT NULL,
    unit    TEXT NOT NULL DEFAULT '',
    as_text INTEGER NOT NULL DEFAULT 0,
    PRIMARY KEY (kind, model, param),
    FOREIGN KEY (kind, model) REFERENCES model(kind, name) ON DELETE CASCADE
);

CREATE TABLE model_attr(
    kind    TEXT NOT NULL,
    model   TEXT NOT NULL,
    attr    TEXT NOT NULL,
    value   TEXT NOT NULL,
    unit    TEXT NOT NULL DEFAULT '',
    as_text INTEGER NOT NULL DEFAULT 0,
    PRIMARY KEY (kind, model, attr),
    FOREIGN KEY (kind, model) REFERENCES model(kind, name) ON DELETE CASCADE
);
"""


def _reset_schema(conn: sqlite3.Connection) -> None:
    """清掉库里的旧结构。**先删子表**，否则外键会拦住。"""
    conn.executescript(
        "DROP TABLE IF EXISTS model_attr;"
        "DROP TABLE IF EXISTS model_param;"
        "DROP TABLE IF EXISTS model;"
        "DROP TABLE IF EXISTS library_meta;"
    )


def _read_records(conn: sqlite3.Connection, *, origin: str) -> list[ModelRecord]:
    version = _meta(conn, "schema_version")
    if version is None:
        raise LibrarySchemaError(
            f"{origin} 不像一个参数库：library_meta 表里没有 schema_version"
            "（model / model_param 表也不一定在）"
        )
    if version != str(LIBRARY_SCHEMA_VERSION):
        raise LibrarySchemaError(
            f"{origin} 的结构版本是 {version}，本程序只认识 "
            f"{LIBRARY_SCHEMA_VERSION}——库需要迁移，不能硬读"
        )

    try:
        rows = conn.execute(
            "SELECT kind, name, parent, note, source, line FROM model"
            " ORDER BY kind, name"
        ).fetchall()
        param_rows = conn.execute(
            "SELECT kind, model, param, value, unit, as_text FROM model_param"
            " ORDER BY kind, model, param"
        ).fetchall()
    except sqlite3.Error as exc:
        raise LibrarySchemaError(f"{origin} 的表结构不对：{exc}") from exc

    # model_attr 是**可选侧表**：这个表是后加的，早先建出来的库没有它。
    # 缺了只说明"这个库没有属性"，不是库坏了——所以只有它**存在但读不动**
    # 才报错。反过来，如果连"有没有这张表"都查不了，那就是库坏了。
    if _has_table(conn, "model_attr"):
        try:
            attr_rows = conn.execute(
                "SELECT kind, model, attr, value, unit, as_text FROM model_attr"
                " ORDER BY kind, model, attr"
            ).fetchall()
        except sqlite3.Error as exc:
            raise LibrarySchemaError(
                f"{origin} 的 model_attr 表结构不对：{exc}"
            ) from exc
    else:
        attr_rows = []

    grouped: dict[tuple[str, str], dict[str, Literal]] = {}
    for row in param_rows:
        grouped.setdefault((row["kind"], row["model"]), {})[row["param"]] = Literal(
            str(row["value"]), str(row["unit"] or ""), bool(row["as_text"])
        )

    grouped_attrs: dict[tuple[str, str], dict[str, Literal]] = {}
    for row in attr_rows:
        grouped_attrs.setdefault((row["kind"], row["model"]), {})[row["attr"]] = Literal(
            str(row["value"]), str(row["unit"] or ""), bool(row["as_text"])
        )

    records: list[ModelRecord] = []
    for row in rows:
        key = (row["kind"], row["name"])
        records.append(
            ModelRecord(
                kind=str(row["kind"]),
                name=str(row["name"]),
                params=grouped.get(key, {}),
                parent=row["parent"],
                note=str(row["note"] or ""),
                source=str(row["source"] or ""),
                line=int(row["line"] or 0),
                attrs=grouped_attrs.get(key, {}),
            )
        )
    return records


def _has_table(conn: sqlite3.Connection, name: str) -> bool:
    """库里有没有这张表。查不动就当没有——真坏了会在读数据时报出来。"""
    try:
        row = conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?", (name,)
        ).fetchone()
    except sqlite3.Error:
        return False
    return row is not None


def _meta(conn: sqlite3.Connection, key: str) -> str | None:
    try:
        row = conn.execute(
            "SELECT value FROM library_meta WHERE key = ?", (key,)
        ).fetchone()
    except sqlite3.Error:
        return None
    return str(row["value"]) if row else None


# ---------------------------------------------------------------------------
# 文本 → 记录
# ---------------------------------------------------------------------------


def _record_from_block(block: Any, kind: str, origin: str) -> ModelRecord:
    keyword = KIND_KEYWORDS[kind]
    where = f"{origin} 第 {block.line} 行"

    if not block.names:
        raise LibraryError(f"{where}：{keyword} 后面缺少型号名")
    if len(block.names) > 1:
        raise LibraryError(
            f"{where}：{keyword} 只接一个型号名，"
            f"多出了 {'、'.join(block.names[1:])}"
        )
    nested = block.blocks()
    attr_blocks = [child for child in nested if child.keyword == ATTR_KEYWORD]
    others = [child for child in nested if child.keyword != ATTR_KEYWORD]
    if others:
        raise LibraryError(
            f"{where}：{keyword} {block.name} 里出现了子块 "
            f"{others[0].keyword!r}——参数库没有嵌套结构，"
            f"只有一个可选的 {ATTR_KEYWORD} 属性块"
        )
    if len(attr_blocks) > 1:
        raise LibraryError(
            f"{where}：{keyword} {block.name} 里有 {len(attr_blocks)} 个 "
            f"{ATTR_KEYWORD} 块——一个型号只写一个；写两个的话，"
            "同名属性算哪个生效就得靠猜了"
        )

    attrs: dict[str, Literal] = {}
    for attr_block in attr_blocks:
        if attr_block.names:
            raise LibraryError(
                f"{origin} 第 {attr_block.line} 行：{ATTR_KEYWORD} 后面不接名字"
                f"（多出了 {'、'.join(attr_block.names)}）——"
                "它是块，不是带名字的声明"
            )
        if attr_block.blocks():
            # 语法层没有给 attr 子块（``CHILD_BLOCKS`` 里没有 "attr" 这一项），
            # 所以这一支现在到不了。留着是因为"到不了"依赖**另一个文件里的
            # 一行常量**——那一行改了，这里就会静默吞掉嵌套内容，而属性是
            # 给人和报表看的，少一条没人会发现。
            raise LibraryError(
                f"{origin} 第 {attr_block.line} 行：{ATTR_KEYWORD} 里不能再嵌块"
                f"（{attr_block.blocks()[0].keyword!r}）"
            )
        for assignment in attr_block.assignments():
            key = assignment.key
            if key in attrs:
                raise LibraryError(
                    f"{origin} 第 {assignment.line} 行：型号 {block.name} 的"
                    f"属性 {key!r} 写了两次——静默取后者的话，"
                    "\"明明改了却没生效\"会变成常态"
                )
            attrs[key] = literal_of(assignment.value)

    params: dict[str, Literal] = {}
    note = ""
    source = ""
    for assignment in block.assignments():
        key = assignment.key
        if key in METADATA_KEYS:
            text = _assigned_text(assignment.value, where, key)
            if key == "note":
                note = text
            else:
                source = text
            continue
        if key in params:
            raise LibraryError(
                f"{where}：型号 {block.name} 的参数 {key!r} 写了两次"
                "——静默取后者会让人对着"
                f"\"明明改了却没生效\"发呆。另一处在第 {assignment.line} 行"
            )
        params[key] = literal_of(assignment.value)
    return ModelRecord(
        kind=kind,
        name=block.name,
        params=params,
        parent=block.parent,
        note=note,
        source=source,
        line=block.line,
        attrs=attrs,
    )


def _assigned_text(value: Any, where: str, key: str) -> str:
    if isinstance(value, WithUnit):
        raise LibraryError(f"{where}：{key} 是文本，不该带单位")
    if isinstance(value, str):
        return value
    if isinstance(value, Sequence):
        return " ".join(str(item) for item in value)
    return _number_text(value)


__all__ = [
    "ATTR_KEYWORD",
    "KIND_COMPONENT",
    "KIND_PLATFORM",
    "KINDS",
    "KIND_KEYWORDS",
    "LIBRARY_SCHEMA_VERSION",
    "DuplicateModelError",
    "LibraryError",
    "LibrarySchemaError",
    "Literal",
    "ModelRecord",
    "ParamLibrary",
    "literal_of",
    "parse_literal",
]
