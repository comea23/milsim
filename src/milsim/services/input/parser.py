"""想定语言的语法分析：token 流 → 块结构节点树。

语法只有两条产生式
------------------
整门语言就这两句：``Block`` 和 ``Assignment``。**刻意不引入更多**——
想定是配置不是程序，每多一条产生式，想定作者要学的东西、解析器要测的
分支就各多一份。

.. code-block:: text

    Block      := KEYWORD name* (':' name)? NEWLINE item* 'end_'KEYWORD
    Assignment := name value NEWLINE

``Block`` 的头部按关键字解释：``platform SAM_1 SAM_BATTALION`` 是"实例名 +
类型名"，``component_type X : Y`` 是"名字 + 父类型"。语法层不做这个判断，
只把裸名字与冒号后的父名分开——**块头怎么读由语义层决定**，否则每加一种
块都要动语法层。

值只有六种形态
--------------
数值、带单位数值、字符串、布尔、列表、坐标。多值只用于坐标（``latlng``
需要两个数），其余一律单值——单值后面还有内容就是笔误，直接报错并指出
多出来的那一截在哪。

不配对是这里最要紧的错误
------------------------
``end_component_type`` 与 ``end_platform`` 写混、漏写 ``end_xxx``，都会在
几百行的想定里造成大范围误解析。所以配对检查给出**期望的结束标记**与
**实际读到的**，并且报出块开始的位置。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, Sequence

from ...errors import ConfigurationError
from ..params import CoordValue, WithUnit
from .lexer import Token, TokenKind, tokenize

#: 会开启一个块的顶层关键字。
#:
#: 这是语法层唯一认识的关键字集合——判断"这一行是块还是赋值"必须有个依据，
#: 而基于"名字后面有没有跟东西"去猜会误判（``layers 3`` 也跟了东西）。
BLOCK_KEYWORDS = frozenset(
    {
        "component_type",
        "platform_type",
        "component",
        "zone",
        "platform",
        "decision",
        "simulation",
        "formation",
        "command",
        "network",
    }
)

#: **每个块里允许出现哪些子块。** 键为空串表示顶层。
#:
#: 为什么要按上下文区分：``decision`` 既是块关键字（定义一份决策配置），
#: 又是平台块里的一个键（``decision TACTICAL_LLM`` 引用它）。只看名字
#: 分不出来，看它在谁里面就一目了然。``component`` 也一样——只有
#: ``platform_type`` 里才是块，别处写了就是普通的键。
#:
#: ``command`` 块里没有子块：``attach`` / ``direct`` / ``coordinate`` 都是
#: **赋值行**（一行多值）。这样读起来紧凑，也不必为每条配属再写一对
#: ``end_xxx``。
#:
#: ``attr`` 是**参数库**的内层属性块（``services/library.py``）：型号可以带
#: 一批"组件没有消费者"的量（毁伤概率、圆概率误差、国别……）。语法层放它进来
#: 是因为它和 ``component`` 一样**只有按上下文才分得清**——它与 ``component``
#: 的区别是：``component`` 在想定里也真有人读，``attr`` 只有库读。
#: 所以``component_type`` 与 ``platform_type`` 都收它，而**想定侧一律报错**
#: （``scenario.py`` 会明说"这里没有消费者"），不然想定里写一段属性会**静默
#: 失效**——那正是这套语法最不愿意发生的事。
CHILD_BLOCKS: dict[str, frozenset[str]] = {
    "": frozenset(
        {
            "component_type",
            "platform_type",
            "zone",
            "platform",
            "decision",
            "simulation",
            "formation",
            "command",
            "network",
        }
    ),
    "platform_type": frozenset({"component", "attr"}),
    "component_type": frozenset({"attr"}),
}

_NO_CHILDREN: frozenset[str] = frozenset()

#: 坐标前缀。``latlng 39.95 116.35`` 与 ``hex 1204 883 @ layer 0``。
COORD_PREFIXES = frozenset({"latlng", "hex"})


class ParseError(ConfigurationError):
    """语法错误。带行列与长度，便于在原文里画箭头。"""

    def __init__(self, message: str, token: Token, length: int = 1) -> None:
        self.line = token.line
        self.column = token.column
        self.length = max(1, length)
        super().__init__(message)


@dataclass(frozen=True, slots=True)
class TaggedValue:
    """``seed=42`` 里的内联标注。"""

    tag: str
    value: object

    def __str__(self) -> str:
        return f"{self.tag}={self.value}"


@dataclass(slots=True)
class Assignment:
    """一行 ``key value``。"""

    key: str
    value: object
    line: int
    column: int

    def __repr__(self) -> str:
        return f"<{self.key} = {self.value!r} @{self.line}>"


@dataclass(slots=True)
class Block:
    """一个 ``KEYWORD ... end_KEYWORD`` 块。

    ``names`` 是块头里的裸名字（``platform SAM_1 SAM_BATTALION`` 有两个），
    ``parent`` 是冒号后的父类型名（``component_type X : Y`` 里的 Y）。
    """

    keyword: str
    names: list[str] = field(default_factory=list)
    parent: str | None = None
    items: list = field(default_factory=list)
    line: int = 0
    end_line: int = 0

    @property
    def name(self) -> str:
        """块头的第一个名字。没有则为空串（``simulation`` 就没有）。"""
        return self.names[0] if self.names else ""

    def assignments(self) -> list[Assignment]:
        return [item for item in self.items if isinstance(item, Assignment)]

    def blocks(self, keyword: str | None = None) -> list["Block"]:
        found = [item for item in self.items if isinstance(item, Block)]
        if keyword is None:
            return found
        return [block for block in found if block.keyword == keyword]

    def get(self, key: str) -> Assignment | None:
        for item in self.items:
            if isinstance(item, Assignment) and item.key == key:
                return item
        return None

    def __repr__(self) -> str:
        head = " ".join(self.names)
        tail = f" : {self.parent}" if self.parent else ""
        return f"<Block {self.keyword} {head}{tail} ({len(self.items)} 项) @{self.line}>"


class Parser:
    """token 流 → 块列表。一次性使用。"""

    __slots__ = ("_tokens", "_position", "_count")

    def __init__(self, tokens: Sequence[Token] | Iterator[Token]) -> None:
        # 需要前瞻，所以直接列表化。想定文件不过几百行，这点内存不值一提。
        self._tokens = list(tokens)
        self._position = 0
        self._count = len(self._tokens)

    # -- 基础动作 ----------------------------------------------------------

    def _peek(self, offset: int = 0) -> Token:
        index = self._position + offset
        if index >= self._count:
            return self._tokens[-1]           # 末尾的 EOF
        return self._tokens[index]

    def _next(self) -> Token:
        token = self._peek()
        if self._position < self._count:
            self._position += 1
        return token

    def _skip_newlines(self) -> None:
        while self._peek().kind is TokenKind.NEWLINE:
            self._position += 1

    def _read_to_end_of_line(self) -> list[Token]:
        """读一行（不含 NEWLINE）。EOF 也算行尾。"""
        tokens: list[Token] = []
        while self._peek().kind not in (TokenKind.NEWLINE, TokenKind.EOF):
            tokens.append(self._next())
        return tokens

    # -- 主入口 ------------------------------------------------------------

    def parse(self) -> list[Block]:
        """解析整份想定，返回顶层块列表。"""
        blocks: list[Block] = []

        while True:
            self._skip_newlines()
            token = self._peek()
            if token.kind is TokenKind.EOF:
                return blocks

            if token.kind is not TokenKind.NAME:
                raise ParseError(
                    f"期望块声明（如 platform / zone / component_type），"
                    f"读到 {self._describe(token)}",
                    token,
                )

            if token.text.startswith("end_"):
                raise ParseError(
                    f"{token.text!r} 没有对应的开始块——检查是否多写了一个结束标记",
                    token,
                )

            if token.text not in BLOCK_KEYWORDS:
                raise ParseError(
                    f"顶层不能直接写参数 {token.text!r}——"
                    f"请放进块里（可用的块：{'、'.join(sorted(BLOCK_KEYWORDS))}）",
                    token,
                )

            blocks.append(self._parse_block())

    def _parse_block(self) -> Block:
        keyword_token = self._next()
        keyword = keyword_token.text
        expected_end = f"end_{keyword}"
        # 子块集合由**当前块自己**决定，不是父块。``CHILD_BLOCKS`` 的键是
        # "哪个块里面"，值是"它里面能放哪些子块"——所以要用 keyword 去查。
        allowed_children = CHILD_BLOCKS.get(keyword, _NO_CHILDREN)

        names, parent = self._parse_header(keyword_token)
        block = Block(keyword, names, parent, [], keyword_token.line)

        while True:
            self._skip_newlines()
            token = self._peek()

            if token.kind is TokenKind.EOF:
                raise ParseError(
                    f"第 {keyword_token.line} 行的 {keyword} 块没有结束——"
                    f"缺少 {expected_end}",
                    keyword_token,
                )

            if token.kind is not TokenKind.NAME:
                raise ParseError(
                    f"期望参数名或 {expected_end}，读到 {self._describe(token)}", token
                )

            if token.text.startswith("end_"):
                if token.text != expected_end:
                    raise ParseError(
                        f"结束标记不匹配：第 {keyword_token.line} 行开的是 {keyword} 块，"
                        f"应该用 {expected_end}，实际写到 {token.text!r}",
                        token,
                    )
                block.end_line = token.line
                self._next()
                return block

            # 子块是**按上下文**判断的，不是按名字。见 CHILD_BLOCKS 的说明。
            if token.text in allowed_children:
                block.items.append(self._parse_block())
                continue

            block.items.append(self._parse_assignment())

    def _parse_header(self, keyword_token: Token) -> tuple[list[str], str | None]:
        """解析块头：裸名字序列，加可选的 ``: PARENT``。"""
        names: list[str] = []
        parent: str | None = None
        saw_colon = False
        length = 0

        while True:
            token = self._peek()
            if token.kind in (TokenKind.NEWLINE, TokenKind.EOF):
                break

            if token.kind is TokenKind.COLON:
                if saw_colon:
                    raise ParseError("块头里出现了两个冒号", token)
                saw_colon = True
                self._next()
                continue

            if token.kind is not TokenKind.NAME:
                raise ParseError(
                    f"块头只接受名字与冒号，读到 {self._describe(token)}",
                    token,
                )

            length += 1
            if saw_colon:
                if parent is not None:
                    raise ParseError(
                        f"冒号后只能跟一个父类型，多出了 {token.text!r}", token
                    )
                parent = token.text
            else:
                names.append(token.text)
            self._next()

        if saw_colon and parent is None:
            raise ParseError("冒号后缺少父类型名", keyword_token, length or 1)

        return names, parent

    def _parse_assignment(self) -> Assignment:
        key_token = self._next()
        rest = self._read_to_end_of_line()

        if not rest:
            raise ParseError(
                f"参数 {key_token.text!r} 缺少值"
                f"（如果这是个块，块名需要是 {key_token.text!r} 之外的关键字）",
                key_token,
            )

        value = self._parse_values(rest, key_token)
        return Assignment(key_token.text, value, key_token.line, key_token.column)

    # -- 值 ----------------------------------------------------------------

    def _parse_values(self, tokens: list[Token], key_token: Token) -> object:
        """一行里的所有值。单个返回标量，多个返回元组。

        为什么要支持"一行多值"：``anchor 39.9042 116.4074`` 就是两个数，
        ``terrain procedural seed=42`` 是"类型 + 内联标注"。语法层**不去猜**
        它是什么意思——那是语义层拿着 schema 才能判断的事。语法层只保证
        "这一行有哪些值、各自在哪一列"。
        """
        values: list[object] = []
        index = 0

        while index < len(tokens):
            value, consumed = self._parse_value(tokens[index:], key_token)
            values.append(value)
            index += consumed

        return values[0] if len(values) == 1 else tuple(values)

    def _parse_value(self, tokens: list[Token], key_token: Token) -> tuple[object, int]:
        """返回 ``(值, 消耗的 token 数)``。"""
        head = tokens[0]

        if head.kind is TokenKind.LBRACKET:
            return self._parse_list(tokens, key_token)

        if head.is_name(*COORD_PREFIXES):
            return self._parse_coord(tokens, key_token)

        if head.kind is TokenKind.TAGGED:
            return TaggedValue(head.tag, head.value), 1

        if head.kind is TokenKind.NUMBER:
            if len(tokens) > 1 and tokens[1].kind is TokenKind.UNIT:
                return WithUnit(head.value, tokens[1].text), 2
            return head.value, 1

        if head.kind in (TokenKind.STRING, TokenKind.NAME, TokenKind.BOOLEAN):
            return head.value, 1

        raise ParseError(f"无法解析的值：{self._describe(head)}", head)

    def _parse_list(
        self, tokens: list[Token], key_token: Token
    ) -> tuple[tuple, int]:
        if tokens[0].kind is not TokenKind.LBRACKET:
            raise ParseError("内部错误：列表解析被调用于非列表", tokens[0])

        items: list[object] = []
        index = 1

        # 空列表
        if tokens[index].kind is TokenKind.RBRACKET:
            return (), 2

        while True:
            if index >= len(tokens):
                raise ParseError(
                    f"列表没有闭合的 ]（{key_token.text!r} 这一行）", key_token
                )

            token = tokens[index]
            if token.kind in (
                TokenKind.STRING,
                TokenKind.NAME,
                TokenKind.BOOLEAN,
            ):
                items.append(token.value)
                index += 1
            elif token.kind is TokenKind.NUMBER:
                if index + 1 < len(tokens) and tokens[index + 1].kind is TokenKind.UNIT:
                    items.append(WithUnit(token.value, tokens[index + 1].text))
                    index += 2
                else:
                    items.append(token.value)
                    index += 1
            elif token.kind is TokenKind.TAGGED:
                items.append(TaggedValue(token.tag, token.value))
                index += 1
            else:
                raise ParseError(
                    f"列表元素只能是数值、字符串或布尔，"
                    f"读到 {self._describe(token)}",
                    token,
                )

            if index >= len(tokens):
                raise ParseError(
                    f"列表没有闭合的 ]（{key_token.text!r} 这一行）", key_token
                )

            separator = tokens[index]
            if separator.kind is TokenKind.RBRACKET:
                return tuple(items), index + 1
            if separator.kind is not TokenKind.COMMA:
                raise ParseError(
                    f"列表元素之间需要逗号，读到 {self._describe(separator)}",
                    separator,
                )
            index += 1

    def _parse_coord(
        self, tokens: list[Token], key_token: Token
    ) -> tuple[CoordValue, int]:
        """``latlng 39.95 116.35`` 或 ``hex 1204 883 @ layer 0``。"""
        form = tokens[0].text
        index = 1

        if index + 1 >= len(tokens):
            raise ParseError(
                f"{form} 需要两个数值（{form} lat lng 或 {form} q r）", key_token
            )

        first, second = tokens[index], tokens[index + 1]
        for token in (first, second):
            if token.kind is not TokenKind.NUMBER:
                raise ParseError(f"{form} 的参数必须是数值", token)
        index += 2

        if form == "latlng":
            lat, lng = float(first.value), float(second.value)
            if not (-90.0 <= lat <= 90.0):
                raise ParseError(f"纬度 {lat} 超出 [-90, 90]", first)
            if not (-180.0 <= lng <= 180.0):
                raise ParseError(f"经度 {lng} 超出 [-180, 180]", second)
            return CoordValue("latlng", lat=lat, lng=lng), index

        q, r = int(first.value), int(second.value)

        # 网格坐标可带 "@ layer N" 与 "@ zone NAME"。
        # **一个 @ 后面可以连续写多个子句**，顺序任意：``@ zone X layer 2``
        # 读起来比 ``@ zone X @ layer 2`` 自然，两者也都支持。
        # 多个战区时 zone 必须写：同一个 (q, r) 在两个战区里是两个地方。
        layer = 0
        zone = ""

        while index < len(tokens) and tokens[index].kind is TokenKind.AT:
            index += 1
            if index >= len(tokens) or tokens[index].kind is not TokenKind.NAME:
                token = tokens[index] if index < len(tokens) else key_token
                raise ParseError("@ 后面应是 layer N 或 zone NAME", token)

            while index < len(tokens) and tokens[index].kind is TokenKind.NAME:
                keyword = tokens[index]
                if not keyword.is_name("layer", "zone"):
                    raise ParseError("@ 后面只能是 layer 或 zone", keyword)
                index += 1

                if keyword.text == "layer":
                    if index >= len(tokens) or tokens[index].kind is not TokenKind.NUMBER:
                        token = tokens[index] if index < len(tokens) else key_token
                        raise ParseError("layer 后面必须是层号", token)
                    layer = int(tokens[index].value)
                    if layer < 0:
                        raise ParseError(f"层号不能为负：{layer}", tokens[index])
                    index += 1
                else:
                    if index >= len(tokens) or tokens[index].kind is not TokenKind.NAME:
                        token = tokens[index] if index < len(tokens) else key_token
                        raise ParseError("zone 后面必须是战区名", token)
                    zone = tokens[index].text
                    index += 1

        return CoordValue("hex", q=q, r=r, layer=layer, zone=zone), index

    # -- 展示 --------------------------------------------------------------

    def _describe(self, token: Token) -> str:
        if token.kind is TokenKind.EOF:
            return "文件结尾"
        if token.kind is TokenKind.NEWLINE:
            return "行尾"
        return f"{token.text!r}"


def parse(text: str) -> list[Block]:
    """想定文本 → 顶层块列表。"""
    return Parser(tokenize(text)).parse()


__all__ = [
    "BLOCK_KEYWORDS",
    "CHILD_BLOCKS",
    "COORD_PREFIXES",
    "Assignment",
    "Block",
    "ParseError",
    "Parser",
    "TaggedValue",
    "parse",
]
