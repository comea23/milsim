"""参数库工具：文本 ↔ 数据库，外加全量检查。

用法::

    python tools/model_library.py build library/demo_models.txt -o library/demo_models.db
    python tools/model_library.py dump  library/demo_models.db [-o 输出文本]
    python tools/model_library.py lint  library/demo_models.db
    python tools/model_library.py show  library/demo_models.db LIB_TEST_RADAR

两种内容，两张表
----------------
``model_param`` 的每一行都要能对上组件 ``PARAMS`` 里的一个键；对不上的
（毁伤概率、圆概率误差、国别……外部资料里那些**组件还没有消费者**的量）
写进 ``model_attr``，文本侧是型号里的一个 ``attr`` 内层块：

.. code-block:: text

    component_type MOZI_W_000000000003 : MISSILE_MOVER
        mass 522 kg
        attr
            SurfacePoK 85
        end_attr
    end_component_type

``lint`` 只查前者（属性没有参数表可查）。前端/报表要的是这两张表**全部**。

``build`` 为什么默认拒绝重建
---------------------------
库不是只由文本长出来的：导入工具会直接往库文件里写型号。拿一份文本
``build`` 上去会 ``DROP`` 重建，那些型号**静默消失**——症状要到某份想定
突然报"型号不存在"时才出现。所以 ``build`` 先比对一眼，要重建得加 ``--force``。

为什么要有"文本"这一半
----------------------
数据库是二进制的：改了一个值，``git diff`` 看不见，code review 也看不见。
所以库有**两种形态，一份内容**：

.. code-block:: text

    作者改的          *.txt   ← 文本形式，能 diff、能 review
       │  build
       ▼
    想定读的          *.db    ← 只读，快速、可查询、有主键约束

``dump`` 是反向：把库重新导出成文本，用来确认"库里到底是什么"。
它是**规范形式**——按名字排序、丢掉注释、参数按名排——所以
``build`` 之后 ``dump`` **不会**逐字回到手写的那份 ``.txt``（注释和
书写顺序都不保留），但会回到**上一次 dump 的那一份**：实测
``dump`` → ``build`` → ``dump`` 两次输出逐字相同，修订号也相同
（``3563a828a841``）。所以"我改的"和"跑的"不会分家——要逐字比就比
dump 出来的两份，别拿手写的 ``.txt`` 去比。

``lint`` 为什么要单独一步
-------------------------
装配只校验**用到**的型号。库里有 200 个型号、这份想定只用 3 个时，
另外 197 个里的错值不会被装配发现——但它们已经在库里了，下一次有人
用到就会炸。``lint`` 是全量检查，适合进 CI。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.errors import MilsimError  # noqa: E402
from milsim.services.library import (  # noqa: E402
    KIND_COMPONENT,
    KIND_PLATFORM,
    ParamLibrary,
)

#: 业务代码里注册的组件，靠 import 触发注册。没有它们，库里的型号
#: 找不到实现类，``lint`` 会把每个型号都报成"未注册"。
_COMPONENT_MODULES = ("milsim.models", "demo_components")


def _load_components() -> list[str]:
    """导入组件模块，返回导入失败的模块名。

    失败不致命：库里可能只放平台型号，或者这次只做 ``dump``。
    但**要把失败说出来**——"lint 说全通过"和"lint 其实没检查"是两回事。
    """
    failed: list[str] = []
    for name in _COMPONENT_MODULES:
        try:
            __import__(name)
        except ImportError:
            failed.append(name)
    return failed


def _usage() -> int:
    print(__doc__.strip(), file=sys.stderr)
    return 2


def cmd_build(argv: list[str]) -> int:
    """文本 → 库文件。

    **默认拒绝"抹掉源里没有的型号"。** 库不是只由文本长出来的：导入工具
    （``tools/mozi_import.py`` 之类）会直接往库文件里写型号。拿一份文本
    ``build`` 上去会 ``DROP`` 重建，那些型号**静默消失**，而症状要到"某份
    想定突然报型号不存在"时才出现——那时候没人会想到是上一次 build 干的。
    所以先比对一眼，要重建得显式加 ``--force``。
    """
    flags = {name for name in argv if name.startswith("--")}
    positional = [name for name in argv if not name.startswith("--")]

    if "-o" not in positional:
        print("build 需要 -o <输出文件>", file=sys.stderr)
        return 2
    cut = positional.index("-o")
    sources = positional[:cut]
    target = Path(positional[cut + 1])
    if not sources:
        print("build 至少需要一个源文件", file=sys.stderr)
        return 2

    merged: ParamLibrary | None = None
    for name in sources:
        file = Path(name)
        if not file.exists():
            print(f"源文件不存在：{file}", file=sys.stderr)
            return 1
        library = ParamLibrary.from_text(
            file.read_text(encoding="utf-8"), origin=str(file)
        )
        merged = library if merged is None else merged.merged_with(library)

    assert merged is not None

    if target.exists() and "--force" not in flags:
        try:
            existing = ParamLibrary.from_path(target)
        except MilsimError as exc:
            print(f"{target} 读不了，无法确认它装了什么：{exc}", file=sys.stderr)
            print("确认要重建就加 --force", file=sys.stderr)
            return 1
        extra = sorted(
            record.name
            for record in existing.sorted_records()
            if not merged.has(record.name, record.kind)
        )
        if extra:
            print(
                f"{target} 里有 {len(extra)} 个源文件里没有的型号"
                f"（如 {extra[0]!r}）——重建会**静默删掉**它们。",
                file=sys.stderr,
            )
            print(
                "  这些型号多半是导入工具写进去的：先 dump 出来留一份，"
                "或者确认不要了就加 --force",
                file=sys.stderr,
            )
            return 1

    merged.write_sqlite(target, overwrite=True)
    print(f"{target}：{merged.describe()}")

    remaining = _load_components()
    if remaining:
        print(
            f"注意：{'、'.join(remaining)} 没能导入，"
            "带 --lint 也无法校验这些型号",
            file=sys.stderr,
        )
    if "--lint" in flags:
        return _lint(merged)
    return 0


def _lint(library: ParamLibrary) -> int:
    problems = library.validate()
    if not problems:
        print("全量校验通过")
        return 0
    print(f"发现 {len(problems)} 个问题：", file=sys.stderr)
    for item in problems:
        print(f"  - {item}", file=sys.stderr)
    return 1


def cmd_dump(argv: list[str]) -> int:
    """库 → 规范文本。给 ``-o`` 就写文件，否则打到标准输出。

    写文件这条路是给**生成的库**用的：库文件是二进制的，翻不出"这个值哪来的"，
    所以每次重建之后要 dump 一份出来进版本控制。重定向 ``>`` 也能做同一件事，
    但那会把编码交给 shell——Windows 上默认不是 UTF-8，dump 出来就是乱码。
    """
    target: Path | None = None
    if "-o" in argv:
        cut = argv.index("-o")
        if cut + 1 >= len(argv):
            print("dump -o 后面要跟文件名", file=sys.stderr)
            return 2
        target = Path(argv[cut + 1])
        argv = argv[:cut] + argv[cut + 2 :]

    if len(argv) != 1:
        return _usage()
    text = ParamLibrary.from_path(argv[0]).to_text()
    if target is None:
        print(text, end="")
        return 0
    target.write_text(text, encoding="utf-8")
    print(f"{target}：{len(text.splitlines())} 行")
    return 0


def cmd_lint(argv: list[str]) -> int:
    if len(argv) != 1:
        return _usage()
    library = ParamLibrary.from_path(argv[0])
    print(f"{library.describe()}")
    remaining = _load_components()
    if remaining:
        print(
            f"警告：{'、'.join(remaining)} 没能导入——"
            "下面的结果**没有**校验这些模块里的组件",
            file=sys.stderr,
        )
    return _lint(library)


def cmd_show(argv: list[str]) -> int:
    if len(argv) != 2:
        return _usage()
    library = ParamLibrary.from_path(argv[0])
    for kind in (KIND_COMPONENT, KIND_PLATFORM):
        record = library.get(argv[1], kind)
        if record is None:
            continue
        print(record)
        if record.parent:
            print(f"  继承自   {record.parent}")
        if record.note:
            print(f"  备注     {record.note}")
        if record.source:
            print(f"  来源     {record.source}")
        for name in sorted(record.params):
            print(f"  {name:<16} {record.params[name].describe()}")
        if record.attrs:
            print(f"  附加属性（组件没有这个参数位，只有库与报表读）{len(record.attrs)} 条")
            for name in sorted(record.attrs):
                print(f"  @{name:<15} {record.attrs[name].describe()}")
        return 0
    print(f"库里没有型号 {argv[1]!r}", file=sys.stderr)
    return 1


COMMANDS = {
    "build": cmd_build,
    "dump": cmd_dump,
    "lint": cmd_lint,
    "show": cmd_show,
}


def main(argv: list[str]) -> int:
    if len(argv) < 2 or argv[1] not in COMMANDS:
        return _usage()
    try:
        return COMMANDS[argv[1]](argv[2:])
    except MilsimError as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
