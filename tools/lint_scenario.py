"""想定文件检查器：解析 + 校验，出错时打印带箭头的定位。

用法::

    python tools/lint_scenario.py scenarios/demo.txt

成功时打印摘要与各类声明；失败时打印错误位置与原文片段，退出码为 1。

为什么要有这个工具：想定是**手写**的，而手写的东西一定会写错。错误报得
不清楚，作者就只能靠猜——所以这个工具的价值全在报错质量上，不在功能多少。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.errors import MilsimError  # noqa: E402
from milsim.services.input.lexer import LexError, format_error  # noqa: E402
from milsim.services.input.parser import ParseError  # noqa: E402
from milsim.services.scenario import load_scenario  # noqa: E402


def _report(error: BaseException, path: str, source: str) -> None:
    """词法/语法错误能画箭头，语义错误只有行号——两者都渲染。"""
    if isinstance(error, (LexError, ParseError)):
        print(format_error(error, path=path, source=source), file=sys.stderr)
        return

    line = getattr(error, "line", 0)
    print(f"{path}:{line}: {error}", file=sys.stderr)
    if line:
        lines = source.splitlines()
        if 1 <= line <= len(lines):
            print(f"  {line:>4} | {lines[line - 1]}", file=sys.stderr)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        return 2

    path = Path(argv[1])
    if not path.exists():
        print(f"文件不存在：{path}", file=sys.stderr)
        return 2

    source = path.read_text(encoding="utf-8")

    try:
        spec = load_scenario(source, name=path.name)
    except MilsimError as exc:
        # 捕统一基类而不是逐个列：漏一种就是栈回溯，而想定作者看不懂栈
        _report(exc, path.name, source)
        return 1

    print(spec.summary())
    print()

    if spec.zones:
        print("战区：")
        for zone in spec.zones:
            print(
                f"  {zone.name:<16} {zone.anchor.lat:.4f}, {zone.anchor.lng:.4f}  "
                f"半径 {zone.radius_m / 1000:.0f} km  网格 {zone.resolution_m:.0f} m  "
                f"{zone.layers} 层  约 {zone.estimated_cells():,} 单元"
            )

    if spec.platforms:
        print("平台：")
        for platform in spec.platforms:
            decision = spec.effective_decision(platform)
            where = (
                f"latlng {platform.position.lat:.4f},{platform.position.lng:.4f}"
                if platform.position and platform.position.form == "latlng"
                else f"hex {platform.position.q} {platform.position.r}"
                if platform.position
                else "无位置"
            )
            print(
                f"  {platform.name:<16} {platform.type_name:<18} {where:<28} "
                f"决策 {decision.name if decision else '（默认）'}"
            )

    if spec.decisions:
        print("决策配置：")
        for profile in spec.decisions.values():
            if profile.uses_llm:
                detail = (
                    f"{profile.model}  超时 {profile.timeout_s:g}s  "
                    f"间隔 {profile.cadence_s:g}s  降级 {profile.fallback or '（无）'}"
                )
            else:
                detail = profile.provider
            print(f"  {profile.name:<16} {detail}")

    problems = spec.validate()
    print()
    if problems:
        print(f"跨块校验发现 {len(problems)} 个问题：", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    print("校验通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
