"""想定语言：块状文本 → 声明对象。

三段流水线，每段独立可测：

.. code-block:: text

    文本 → Lexer → Parser → semantics → ScenarioSpec
           分词      块结构    键名校验      纯数据声明
          带行号     节点树    单位换算

- :mod:`~milsim.services.input.lexer` —— 分词、单位切分、行列号
- :mod:`~milsim.services.input.parser` —— 块嵌套与 ``end_xxx`` 配对
- :mod:`~milsim.services.scenario` —— 语义校验与单位换算

不合并成"边读边建"的解析器，是因为那样任何一处错误都只能报"解析失败"，
而想定文件动辄几百行，没有结构就没法定位。
"""

from .lexer import (
    LexError,
    Token,
    TokenKind,
    format_error,
    is_bare_word,
    tokenize,
)
from .parser import (
    BLOCK_KEYWORDS,
    CHILD_BLOCKS,
    Assignment,
    Block,
    ParseError,
    Parser,
    TaggedValue,
    parse,
)

__all__ = [
    "BLOCK_KEYWORDS",
    "CHILD_BLOCKS",
    "Assignment",
    "Block",
    "LexError",
    "ParseError",
    "Parser",
    "TaggedValue",
    "Token",
    "TokenKind",
    "format_error",
    "is_bare_word",
    "parse",
    "tokenize",
]
