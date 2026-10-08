"""想定语言的词法分析：字符流 → 带行号的 token 流。

为什么要有独立的一层
--------------------
"边读边建"的解析器任何一处出错都只能报"解析失败"，而想定文件动辄几百行，
没有结构就没法定位。分词、块结构、语义三段分开，每段独立可测，报错也才能
指到具体行列。

词法层的四条约定
----------------
**① 换行有意义。** 块状语言是面向行的——一行就是一条声明，行的结束就是
声明的结束。``NEWLINE`` 因此是要保留的 token，不能当空白丢掉。

**② 数值与单位一起切，空格可有可无。** ``40km`` 与 ``40 km`` 必须切出
完全一样的结果。让作者为空格的有无承担不同语义，是最没必要的一类坑。

**③ 分词阶段不换算单位。** 不知道参数的量纲就判断不了 ``kmm`` 错在哪，
也报不出"距离可用 m / km / nmi"这种有用的信息。换算留给 ``ParamSet``，
它才拿着量纲。词法层只负责把 ``km`` 这个字符串切出来。

**④ ``name=value`` 切成一个 token。** ``seed=42`` 这种内联标注在配置里
很常见，让词法层直接识别，语法层就不必为它写一条产生式。等号**两侧不能
有空格**——``seed = 42`` 那种写法语义含糊，不如不支持。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterator

from ...errors import ConfigurationError

#: 单位候选的最大长度。比这更长的词不可能是单位（表里最长的是 km/h）。
#: 挡住它，``40something`` 就会被切成数字加标识符，语法层随即报"多余内容"，
#: 比悄悄当成单位好。
MAX_UNIT_LENGTH = 6


class TokenKind(IntEnum):
    NAME = 0            # 标识符与关键字（关键字由语法层判断）
    NUMBER = 1          # 整数或浮点
    STRING = 2          # "双引号字符串"
    UNIT = 3            # 数值之后紧邻的单位，如 km / s / m/s
    TAGGED = 4          # name=value 内联标注
    BOOLEAN = 5         # true / false
    LBRACKET = 6
    RBRACKET = 7
    COMMA = 8
    COLON = 9           # 继承："A : B"
    AT = 10             # 网格坐标里的 "@"
    NEWLINE = 11
    EOF = 12


@dataclass(frozen=True, slots=True)
class Token:
    kind: TokenKind
    text: str
    value: object          # 已归一的字面量值（数字是 int/float，字符串是 str）
    line: int
    column: int
    #: TAGGED 专用：``seed=42`` 的标签名
    tag: str = ""

    def is_name(self, *names: str) -> bool:
        return self.kind is TokenKind.NAME and self.text in names

    def __str__(self) -> str:
        return f"{self.kind.name}({self.text!r})@{self.line}:{self.column}"


class LexError(ConfigurationError):
    """词法错误。带行列号，便于在原文里画箭头。"""

    def __init__(self, message: str, line: int, column: int, source: str = "") -> None:
        self.line = line
        self.column = column
        self.source_line = source
        super().__init__(message)


def _is_name_start(ch: str) -> bool:
    return ch.isalpha() or ch == "_"


def _is_name_char(ch: str) -> bool:
    # 允许点与斜杠：model 名、提示词文件路径这类值常含它们，不加引号也能写
    return ch.isalnum() or ch in "_-./"


def _is_unit_char(ch: str) -> bool:
    return ch.isalnum() or ch in "/_"


def is_bare_word(text: str) -> bool:
    """``text`` 原样写进想定，会不会被读成一个**字符串**？

    参数库（§5.5）用它决定"这个值回写时要不要加引号"。判否的四种情形：

    - 空串，或含空白（会被切成多值）
    - ``true`` / ``false``（词法层读成布尔）
    - 看起来像数字（读成数）
    - 含任何不在裸名字符集里的字符（``（``、``,`` 之类，词法层直接报错）

    为什么放在词法层：**字符集只有这一份定义**。散到参数库那边自己写一遍
    ``[A-Za-z0-9_./-]``，两边迟早漂移；而漂移的后果不是那句报错，是引号
    加错了地方——一个带空格的文本被读成多值，一个中文括号让整个库文件
    读不进来。
    """
    if not text or any(ch.isspace() for ch in text):
        return False
    if text in ("true", "false"):
        return False
    if _looks_like_number(text, 0):
        return False
    return _is_name_start(text[0]) and all(_is_name_char(c) for c in text[1:])


def _looks_like_number(text: str, index: int) -> bool:
    if index >= len(text):
        return False
    ch = text[index]
    if ch.isdigit():
        return True
    if ch == "." and index + 1 < len(text) and text[index + 1].isdigit():
        return True
    if ch == "-" and index + 1 < len(text):
        nxt = text[index + 1]
        if nxt.isdigit():
            return True
        return nxt == "." and index + 2 < len(text) and text[index + 2].isdigit()
    return False


class Lexer:
    """一次性的字符扫描器。``tokenize(text)`` 是它的便捷入口。

    写成类而不是一长串生成器，是因为扫描过程中要频繁在前瞻与位置推进之间
    来回，闭包变量会越写越绕。状态放进实例字段读起来清楚得多。
    """

    __slots__ = ("_text", "_length", "_index", "_line", "_column", "_lines")

    def __init__(self, text: str) -> None:
        self._text = text
        self._length = len(text)
        self._index = 0
        self._line = 1
        self._column = 1
        self._lines = text.splitlines()

    # -- 基础动作 ----------------------------------------------------------

    def _peek(self, offset: int = 0) -> str:
        position = self._index + offset
        return self._text[position] if position < self._length else ""

    def _advance(self, count: int = 1) -> None:
        self._index += count
        self._column += count

    def _source_line(self, lineno: int) -> str:
        if 1 <= lineno <= len(self._lines):
            return self._lines[lineno - 1]
        return ""

    def _fail(self, message: str, line: int | None = None, column: int | None = None):
        at_line = self._line if line is None else line
        at_column = self._column if column is None else column
        return LexError(message, at_line, at_column, self._source_line(at_line))

    # -- 主循环 ------------------------------------------------------------

    def tokens(self) -> Iterator[Token]:
        while self._index < self._length:
            ch = self._peek()

            if ch == "\n":
                yield Token(TokenKind.NEWLINE, "\\n", None, self._line, self._column)
                self._advance()
                self._line += 1
                self._column = 1
                continue

            if ch in " \t\r":
                self._advance()
                continue

            if ch == "#":
                while self._index < self._length and self._peek() != "\n":
                    self._advance()
                continue

            if ch == '"':
                yield self._read_string()
                continue

            punctuation = {
                "[": TokenKind.LBRACKET,
                "]": TokenKind.RBRACKET,
                ",": TokenKind.COMMA,
                ":": TokenKind.COLON,
                "@": TokenKind.AT,
            }
            if ch in punctuation:
                yield Token(punctuation[ch], ch, None, self._line, self._column)
                self._advance()
                continue

            if _looks_like_number(self._text, self._index):
                yield from self._read_number()
                continue

            if _is_name_start(ch):
                yield from self._read_word()
                continue

            raise self._fail(f"无法识别的字符 {ch!r}")

        yield Token(TokenKind.EOF, "", None, self._line, self._column)

    # -- 各类 token --------------------------------------------------------

    def _read_string(self) -> Token:
        line, column = self._line, self._column
        self._advance()                      # 开引号
        buffer: list[str] = []

        while True:
            ch = self._peek()
            if ch == "":
                raise self._fail("字符串没有闭合的引号", line, column)
            if ch == "\n":
                raise self._fail(
                    "字符串跨行未闭合（块状语言里一条声明就是一行）", line, column
                )
            if ch == "\\" and self._peek(1) != "":
                buffer.append(self._peek(1))
                self._advance(2)
                continue
            if ch == '"':
                self._advance()
                break
            buffer.append(ch)
            self._advance()

        raw = "".join(buffer)
        return Token(TokenKind.STRING, raw, raw, line, column)

    def _read_word(self) -> Iterator[Token]:
        line, column = self._line, self._column
        start = self._index
        while self._index < self._length and _is_name_char(self._peek()):
            self._advance()
        word = self._text[start:self._index]

        # name=value：等号必须紧跟，中间不能有空格
        if self._peek() == "=" and self._peek(1) not in ("", " ", "\t", "\n"):
            self._advance()                  # 等号
            value, raw = self._read_inline_value(line, column)
            yield Token(
                TokenKind.TAGGED, f"{word}={raw}", value, line, column, tag=word
            )
            return

        if word in ("true", "false"):
            yield Token(TokenKind.BOOLEAN, word, word == "true", line, column)
            return

        yield Token(TokenKind.NAME, word, word, line, column)

    def _read_inline_value(self, line: int, column: int) -> tuple[object, str]:
        if self._peek() == '"':
            token = self._read_string()
            return token.value, str(token.value)
        start = self._index
        while self._index < self._length and _is_name_char(self._peek()):
            self._advance()
        word = self._text[start:self._index]
        if not word:
            raise self._fail("name= 后面没有值", line, column)
        return _as_number_or_text(word), word

    def _read_number(self) -> Iterator[Token]:
        line, column = self._line, self._column
        start = self._index

        if self._peek() == "-":
            self._advance()
        while self._peek().isdigit():
            self._advance()
        if self._peek() == ".":
            self._advance()
            while self._peek().isdigit():
                self._advance()
        if self._peek() in ("e", "E"):
            probe = 1
            if self._peek(probe) in "+-":
                probe += 1
            if self._peek(probe).isdigit():
                self._advance(probe)
                while self._peek().isdigit():
                    self._advance()

        raw = self._text[start:self._index]
        value = _as_number_or_text(raw)
        yield Token(TokenKind.NUMBER, raw, value, line, column)

        unit = self._try_read_unit()
        if unit is not None:
            yield unit

    def _try_read_unit(self) -> Token | None:
        """读数值之后的单位，**允许一个空格**。

        先记下位置，试着跳过一个空格；若后面不是单位形状的词就回退——
        这样 ``40 km`` 与 ``40km`` 结果一致，而 ``hex 1204 883`` 里的
        883 也不会误吞后面的 ``@``。
        """
        line, column = self._line, self._column
        saved = (self._index, self._line, self._column)

        if self._peek() == " ":
            self._advance()
            line, column = self._line, self._column

        if not (self._peek().isalpha() or self._peek() == "/"):
            self._index, self._line, self._column = saved
            return None

        start = self._index
        while self._index < self._length and _is_unit_char(self._peek()):
            self._advance()
        word = self._text[start:self._index]

        if not word or len(word) > MAX_UNIT_LENGTH:
            self._index, self._line, self._column = saved
            return None

        return Token(TokenKind.UNIT, word, word, line, column)


def _as_number_or_text(raw: str) -> object:
    """``42`` → int，``4.5`` → float，其余原样返回字符串。

    前导零要当十进制读（``int(raw)`` 在 ``"007"`` 上会失败），想定里的
    编号常带前导零。
    """
    if not raw:
        return raw
    body = raw[1:] if raw[0] in "+-" else raw
    if not body:
        return raw
    try:
        if "." in body or "e" in body or "E" in body:
            return float(raw)
        return int(raw, 10)
    except ValueError:
        return raw


def tokenize(text: str) -> Iterator[Token]:
    """把想定文本切成 token 流。最后一定有一个 ``EOF``。"""
    return Lexer(text).tokens()


def format_error(error, path: str = "<想定>", source: str = "") -> str:
    """把词法/语法错误渲染成带原文与箭头的多行文本。

    只报 "parse error" 等于让人大海寻针——所以定位是强制的，不是可选的。
    """
    lines = source.splitlines()
    line = getattr(error, "line", 1)
    column = getattr(error, "column", 1)
    length = getattr(error, "length", 1)

    out = [f"{path}:{line}: {error.args[0]}"]
    if 1 <= line <= len(lines):
        text = lines[line - 1]
        out.append(f"  {line:>4} | {text}")
        caret = " " * max(0, column - 1) + "^" * max(1, length)
        out.append(f"       | {caret}")
    return "\n".join(out)


__all__ = [
    "MAX_UNIT_LENGTH",
    "LexError",
    "Lexer",
    "Token",
    "TokenKind",
    "format_error",
    "is_bare_word",
    "tokenize",
]
