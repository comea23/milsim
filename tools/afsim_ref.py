"""跑 AFSIM 原 demo，把它的轨迹采成 CSV，当作对照的**参照**。

用法::

    python tools/afsim_ref.py <AFSIM 脚本> <输出 csv> [--seconds N] [--tag KIN]

可选参数：

``--seconds N``
    把 ``end_time`` 换成 N 秒。原 demo 的航路多是 ``go_to`` 闭环，200 min 只是
    "绕很多圈"。
``--drop-name NAME``
    整块删掉叫 NAME 的 ``platform_type`` / ``platform``。用于**本项不比较**的
    平台（例如 ``mover_demo`` 里的旋翼机——我们没有旋翼机件，而且它的
    ``mode`` 块在 2.9.0 上直接报错）。
``--dump 路径``
    把实际喂给 AFSIM 的脚本另存一份。参照数据的"改了什么"与数据本身同样重要。
``--replace 正则 替换``
    跑之前先改一处文本（可重复给），用来做**参数扫描**：同一个 demos 脚本，
    把 ``update_interval 0.1 sec`` 换成别的节拍再跑一遍，就有了一族参照。
    写成命令行参数而不是另存一份改过的脚本——**改了哪一行、换成什么，与
    数据在一起**，不会出现"数据在、当初怎么跑的忘了"。

它回答的问题只有一个：**"拿什么跟我们的复现比"**。原 demo 只写事件管道
（``.aer`` 回放文件），那是二进制的，读不了也画不了；AFSIM 的事件日志里
**没有**"位置更新"这一类事件（``PLATFORM_ADDED`` / ``PLATFORM_DELETED``
只给首末状态）。所以参照轨迹只能靠脚本观察器采：

    observer
       enable MOVER_UPDATED <脚本名>
    end_observer

``MOVER_UPDATED`` 在机动件每次更新时回调（原 demo 里是 ``update_interval
0.1 sec``，即 10 Hz），回调里 ``writeln`` 出来的行按前缀收成 CSV。

**不改 demos/ 下的原文件**：本工具读原文、在内存里改两处、写进临时目录再跑。
两处改动都写在下面（``_prepare``），并且原样打印，改了什么一目了然。

改的两处：

1. ``end_time`` 换成 ``--seconds``。原 demo 里航路是 ``go_to`` 闭环，
   200 min 只是"绕很多圈"；截成一段就够了，而且产物小两个数量级。
2. 顶层的 ``simulation ... end_simulation`` **删掉**。AFSIM 2.9.0 没有这个
   命令（实测 ``***** ERROR: Unknown command: simulation``），
   ``demos/mover_demo`` 那两份汉化脚本带着它，**原样跑不起来**——
   这不是我们的问题，但要让参照跑起来就得先摘掉它；时间步长由
   ``end_time`` + 事件驱动决定，本来也不需要它。

**不改文本、但要"搬过去"的一样东西**：原 demo 的素材路径都是相对于它自己
那个目录的（``include_once movers/air_mover.txt``、``log_file
output/$(CASE).log``、``dted 1 dted/w107``）。脚本进了临时目录，这些路径
就悬空了——实测 ``***** FATAL: Cannot open file: movers/air_mover.txt``。
所以 ``_stage_assets`` 按同样的相对结构把这些素材**镜像**进临时目录，并在
临时目录里建出输出路径的父目录。写脚本的人看到的仍是原 demo 的写法。

采集到的 CSV 列：``time_s, platform, lat_deg, lon_deg, alt_m, speed_mps,
heading_deg``。经纬度是 AFSIM 的 ``WsfGeoPoint``，海拔是 ``Altitude()``（MSL）。

**一个必须知道的数值坑**：AFSIM 脚本 ``writeln`` 打浮点用的是 ``%g``，只有
**6 位有效数字**——绝对经纬度落盘就成了 11 m 的分辨率，而机动件每帧只走约
10 m，位移与量化噪声同量级。所以观察器打的是**相对锚点的偏移**，落盘时再加
回去（见 ``_LAT_ANCHOR``）。直接打印绝对经纬度的老 CSV 会把 600 s 的航迹
算成 43.5 km（真值约 60 km），**按弧长做的对照全都会被这个噪声污染**。
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

#: 参照轨迹的行前缀。换成别的 tag 可以在同一次运行里分开采（demo 里平台多）。
DEFAULT_TAG = "KIN"

#: 观察器脚本的名字。写短一点，它在输出里出现一次是采集标记，不是噪声。
_OBSERVER_SCRIPT = "afsimTrace"

#: **打印经纬度用的锚点**。这不是"另一个战区原点"，是绕开 AFSIM ``writeln``
#: 的数值精度用的——它的浮点默认转换是 ``%g``（6 位有效数字），直接打印
#: ``33.9124`` 只有 4 位小数，折合 **11 m**；而机动件每帧走 ``v·dt`` ≈ 10 m，
#: **位移与量化噪声同量级**，按坐标差算出来的弧长是噪声不是航程（实测：横跨
#: 600 s 的航迹算成 43.5 km，真值约 60 km）。打印``纬 - 33 / 经 + 84`` 之后
#: 数值落在 0.1~1 之间，同一个 ``%g`` 下分辨率变成 **0.1 m** 上下，比帧位移
#: 小两个数量级。落盘时再加回锚点，所以 CSV 里仍是绝对经纬度。
_LAT_ANCHOR = 33.0
_LNG_ANCHOR = -84.0


def _offset_expr(anchor: float) -> str:
    """把"减去锚点"写成一个 AFSIM 认的加法表达式（省掉 ``- -84.0`` 这种双负号）。"""
    return f"- {anchor}" if anchor >= 0.0 else f"+ {-anchor}"


def _observer_block(tag: str) -> str:
    """追加在原脚本末尾的采集块（``script`` + ``observer``）。

    ``TAG=`` 那两行是给下游解析用的哨兵：AFSIM 自己的诊断输出也走 stdout，
    靠"以逗号开头、以 ``TIME_NOW`` 开头"认行不可靠，认前缀才可靠。

    经纬度打的是**相对锚点的偏移**，理由见 ``_LAT_ANCHOR``。
    """
    return f"""
# ============================================================================
# 采集块：由 milsim/tools/afsim_ref.py 追加，**不属于原 demo**
# MOVER_UPDATED 是机动件每次更新时的回调（脚本观察器接口），
# 原 demo 只写二进制 .aer，事件日志里没有逐帧位置事件，所以轨迹只能这样采。
# 经纬度减了锚点（{_LAT_ANCHOR} / {_LNG_ANCHOR}）：writeln 的 %g 只有 6 位有效
# 数字，直接打印绝对经纬度会被量化到 11 m，而每帧只走 10 m。
# ============================================================================
script void {_OBSERVER_SCRIPT}(WsfPlatform aPlatform, WsfMover aMover)
   WsfGeoPoint p = aPlatform.Location();
   writeln("{tag},", TIME_NOW, ",", aPlatform.Name(), ",",
           p.Latitude() {_offset_expr(_LAT_ANCHOR)}, ",",
           p.Longitude() {_offset_expr(_LNG_ANCHOR)}, ",",
           aPlatform.Altitude(), ",", aPlatform.Speed(), ",",
           aPlatform.Heading());
end_script

observer
   enable MOVER_UPDATED {_OBSERVER_SCRIPT}
end_observer
"""


def _strip_top_level_block(text: str, opener: str, closer: str) -> tuple[str, int]:
    """删掉**顶层**的 ``opener ... closer`` 块，返回 ``(新文本, 删了几行)``。

    只在行首（column 0）认，免得把嵌在别处的同名行也吃掉。AFSIM 的块不嵌套
    同名块，所以第一对就是唯一一对。
    """
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    depth = 0
    dropped = 0
    for line in lines:
        head = line.rstrip("\r\n")
        if depth == 0 and head == opener:
            depth = 1
            dropped += 1
            continue
        if depth == 1 and head == closer:
            depth = 0
            dropped += 1
            continue
        if depth == 1:
            dropped += 1
            continue
        out.append(line)
    return "".join(out), dropped


def _strip_name_in_platform_types(text: str) -> tuple[str, int]:
    """删掉 ``platform_type`` 块里的 ``name "..."`` 行。

    AFSIM 的 ``platform_type`` **没有** ``name`` 命令（平台名是 ``platform
    <名字> <型号>`` 里的第一个 token，型号本身不叫名字）。汉化版的
    ``demos/mover_demo/*.txt`` 在型号里写了 ``name "Demo-AirMover-Air"``，
    原样跑的第二条 FATAL 就是这个（第一条是顶层 ``simulation`` 块）。
    """
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    inside = False
    dropped = 0
    for line in lines:
        head = line.strip()
        if head.startswith("platform_type "):
            inside = True
        elif head == "end_platform_type":
            inside = False
        if inside and re.match(r'^name\s+"[^"]*"\s*$', head):
            dropped += 1
            continue
        out.append(line)
    return "".join(out), dropped


def _drop_named_blocks(text: str, names: set[str]) -> tuple[str, list[str]]:
    """删掉顶层 ``platform_type <名>`` / ``platform <名>`` 的整块。

    **为什么需要它**：``mover_demo`` 那份展示稿里有**旋翼机**
    （``WSF_ROTORCRAFT_MOVER``，带 ``mode HOVER/CRUISE/DASH`` 三档）。它的
    ``mode`` 块在 AFSIM 2.9.0 上直接报 ``'mode' cannot be used in this
    context``——不是参数问题，是那套写法在这个版本里不成立。而我们这边**根本
    没有旋翼机件**，所以对它是"不比较"，不是"比较失败"。让它整块离场，
    剩下的 7 个平台才跑得起来。

    只认行首（顶层），块体按缩进无关的方式整段吃到底。
    """
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    dropped: list[str] = []
    skip_to: str | None = None

    for line in lines:
        head = line.rstrip("\r\n")
        if skip_to is not None:
            if head.strip() == skip_to:
                skip_to = None
            dropped.append(head.strip())
            continue
        words = head.split()
        if len(words) >= 2 and words[0] in ("platform_type", "platform", "end_platform_type"):
            if words[0] == "end_platform_type":
                out.append(line)
                continue
            if words[0] == "platform_type" and words[1] in names:
                skip_to = "end_platform_type"
                dropped.append(head.strip())
                continue
            if words[0] == "platform" and words[1] in names:
                # platform 块以 ``end_platform`` 收尾；它也可能没有（单行实例）。
                skip_to = "end_platform"
                dropped.append(head.strip())
                continue
        out.append(line)

    if not dropped:
        return text, []
    summary = sorted(names)
    return "".join(out), [f"整块删掉了 {n}" for n in summary]


def _prepare(
    source: Path,
    seconds: float | None,
    drop_names: set[str],
    replacements: tuple[tuple[str, str], ...] = (),
) -> tuple[str, list[str]]:
    """读原脚本 → 改 ``end_time`` / 摘掉汉化版多出来的命令 → 追加采集块。

    返回 ``(文本, 改动说明)``。改动说明直接打印出来——参照数据的"改了什么"
    跟数据本身一样重要。
    """
    text = source.read_text(encoding="utf-8", errors="replace")
    notes: list[str] = []

    if seconds is not None:
        new_text, count = re.subn(
            r"(?mi)^\s*end_time\s+.*$", f"end_time {seconds:g} sec", text
        )
        if count:
            notes.append(f"end_time → {seconds:g} sec（原脚本 {count} 处）")
            text = new_text
        else:
            notes.append(f"原脚本没有 end_time，追加 end_time {seconds:g} sec")
            text += f"\nend_time {seconds:g} sec\n"

    for pattern, swap in replacements:
        text, count = re.subn(pattern, swap, text)
        notes.append(f"`{pattern}` → `{swap}`（{count} 处）")

    # 逐条**依次**应用，别写成 ((说明, f(text)) for ...)——那样两个 f 都拿到
    # 改动前的 text，后一步会把前一步的成果整段盖掉（本工具第一次就是这么
    # 错的：simulation 块"删了"却还在，因为 name 那一支用原文覆盖了它）。
    for label, step in (
        ("删掉顶层 simulation 块",
         lambda t: _strip_top_level_block(t, "simulation", "end_simulation")),
        ("删掉 platform_type 里的 name 行", _strip_name_in_platform_types),
        ("删掉 --drop-name 指定的整块", lambda t: _drop_named_blocks(t, drop_names)),
    ):
        text, dropped = step(text)
        if not dropped:
            continue
        if label.startswith("删掉 --drop-name"):
            notes.extend(f"{dropped_i}（本项不复现它）" for dropped_i in dropped)
        else:
            notes.append(
                f"{label}（{dropped} 行）——汉化版多出来的命令，"
                "AFSIM 2.9.0 认不出，原样跑会 FATAL"
            )

    return text + _observer_block(DEFAULT_TAG), notes


#: ``include`` / ``include_once`` 引用的素材。只认整行，避免吃到注释里的例子。
_INCLUDE = re.compile(r"(?mi)^\s*include(?:_once)?\s+(\S+)\s*$")

#: ``dted <级别> <目录>``——地形瓦片不是单个文件，要整个目录。
_DTED = re.compile(r"(?mi)^\s*dted\s+\S+\s+(\S+)\s*$")

#: ``file_path <目录>``。**它才是 ``include`` 的基准**，不是被包含文件所在目录：
#: ``alv_routing/setup.txt`` 写 ``file_path .``，于是
#: ``platforms/alv.txt`` 里的 ``include processors/alv_task_mgr.txt`` 解成
#: ``alv_routing/processors/...``，而不是 ``alv_routing/platforms/processors/...``。
_FILE_PATH = re.compile(r"(?mi)^\s*file_path\s+(\S+)\s*$")

#: 形如 ``目录/文件`` 的 token。用来**建空目录**（供 ``log_file`` /
#: ``event_pipe`` / ``event_output`` 落盘）。字符集刻意限定 ASCII——``\w``
#: 在 Python 3 里匹配汉字，中文注释里的"甲/乙"会被当成路径建出目录来。
_REL_PATH = re.compile(
    r"(?<![A-Za-z0-9_.$()/\\:-])([A-Za-z0-9_.$()-]+(?:/[A-Za-z0-9_.$()-]+)+)"
)


def _mirror_dir(src: Path, dst: Path) -> int:
    """把 ``src`` 整棵复制进 ``dst``，返回复制了几个文件。"""
    count = 0
    for item in src.rglob("*"):
        if not item.is_file():
            continue
        target = dst / item.relative_to(src)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(item, target)
        count += 1
    return count


def _stage_assets(source: Path, workdir: Path, text: str) -> list[str]:
    """把想定引用的**素材**按同样的相对结构镜像进 ``workdir``。

    **为什么需要**：本工具把改动后的脚本写进临时目录再跑——那是"不碰
    ``demos/`` 下原文件"的实现方式。但原 demo 的素材路径都是**相对于原想定
    目录**的：``include_once movers/air_mover.txt``、``log_file
    output/$(CASE).log``、``dted 1 dted/w107``。临时目录里没有这些东西，
    AFSIM 会当场 ``FATAL: Cannot open file: movers/air_mover.txt``。
    （``demos/kinematic_mover`` 那份之所以一直能跑，是因为它自包含：没有
    include、``event_pipe file`` 也不带子目录。）

    镜像三类东西，**都不改文本**——``include`` 的语义原样保留，读脚本的人
    看到的就是原 demo 的写法：

    1. ``include`` / ``include_once`` 的目标文件，按同样的相对路径复制，并
       **递归**处理被包含文件里的 include；
    2. ``dted`` 引用的目录（地形瓦片是目录不是文件）；
    3. 所有形如 ``目录/文件`` 的 token 的**父目录**——建空目录即可，AFSIM
       自己会把 ``.log`` / ``.aer`` / ``.evt`` 写进去。

    **所有路径都以想定所在目录为基准**，因为那是 ``file_path`` 的默认值。
    脚本若声明了别的 ``file_path``（``ballistic/setup.txt`` 有
    ``file_path ../base_types``），本工具**不镜像那些目录**，只把这件事报
    出来——静默按错的前提去找，比不找更坏。
    """
    notes: list[str] = []
    base = source.parent          # 基准目录 = 想定目录，见上面 file_path 那条
    pending: list[str] = [text]   # 待扫 include 的文本（想定本身 + 递归进来的）
    seen: set[Path] = set()
    copied = 0

    while pending:
        body = pending.pop()
        for match in _INCLUDE.finditer(body):
            raw = match.group(1).strip().strip('"')
            target = (base / raw).resolve()
            if not target.is_file():
                notes.append(f"include 的目标找不到：{raw}（找过 {target}）")
                continue
            rel = Path(os.path.normpath(raw))
            dest = workdir / rel
            if dest in seen:
                continue
            seen.add(dest)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(target, dest)
            copied += 1
            pending.append(target.read_text(encoding="utf-8", errors="replace"))
        for match in _DTED.finditer(body):
            raw = match.group(1).strip().strip('"')
            target = (base / raw).resolve()
            if target.is_dir():
                copied += _mirror_dir(target, workdir / Path(os.path.normpath(raw)))
            else:
                notes.append(f"dted 的目录找不到：{raw}（找过 {target}）")

    extra = sorted({
        m.group(1).strip().strip('"') for m in _FILE_PATH.finditer(text)
    } - {".", "./"})
    if extra:
        notes.append(
            "脚本声明了额外的搜索路径 "
            + "、".join(extra)
            + "——本工具只镜像想定目录，落在那些路径下的素材会报「找不到」"
        )

    for match in _REL_PATH.finditer(text):
        (workdir / Path(match.group(1)).parent).mkdir(parents=True, exist_ok=True)

    if copied:
        notes.append(f"镜像了 {copied} 个 include / 地形素材到临时目录（原 demo 只读）")
    return notes


def _mission_exe() -> Path:
    """找 mission.exe：环境变量 → 相对本项目的常见位置。

    不写死盘符：AFSIM 装在哪儿是使用方的事，找不到时把找过的地方都列出来。
    """
    import os

    env = os.environ.get("AFSIM_MISSION")
    seen: list[str] = []
    candidates = []
    if env:
        candidates.append(Path(env))
        seen.append(f"$AFSIM_MISSION={env}")
    # tools/ 的上一级是 milsim/，再上一级就是 AFSIM 根（本项目寄居在里面）。
    root = Path(__file__).resolve().parents[1].parent
    candidates.append(root / "bin" / "mission.exe")
    seen.append(str(root / "bin" / "mission.exe"))
    for path in candidates:
        if path.is_file():
            return path
    raise SystemExit(
        "找不到 mission.exe。设环境变量 AFSIM_MISSION 指向它，"
        "或把本工具放在 AFSIM 根的 milsim/tools/ 下。找过：\n  "
        + "\n  ".join(seen)
    )


#: AFSIM 报未知命令时的两行：``ERROR: Unknown command: <X>`` /
#: ``'<文件>', line <N>, near column <C>``。按行号删掉再跑，直到解析通过。
_UNKNOWN_CMD = re.compile(
    r"ERROR:\s*Unknown command:\s*(\S+)\s*\n[^\n']*'([^']*)',\s*line\s+(\d+)"
)

#: 上限。汉化版的 showcase 脚本是要一条条摘的，但无限摘下去会变成"把它删空"。
#: 超过这个数就说明对面那个脚本压根不是给 AFSIM 2.9.0 写的，该报出来而不是硬凑。
_PRUNE_LIMIT = 40


def _run_once(mission: Path, workdir: Path, text: str):
    """把 text 落到 ``afsim_ref.txt`` 跑一次，返回 CompletedProcess。"""
    script = workdir / "afsim_ref.txt"
    script.write_text(text, encoding="utf-8", newline="\r\n")
    return subprocess.run(
        [str(mission), "-sm", script.name],
        cwd=str(workdir),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=1800,
    )


def _prune_unknown(mission: Path, workdir: Path, text: str):
    """反复"跑 → 摘掉报错那一行 → 再跑"，直到跑通。返回 ``(文本, 进程, 摘掉的)``。

    **为什么要自动摘**：``demos/mover_demo`` 那两份是汉化版作者写的**参数
    展示稿**，不是能跑的想定——实测它对 AFSIM 2.9.0 报一串 ``Unknown
    command``（``simulation``、型号里的 ``name``、``maximum_impact_speed``…）。
    逐条手工摘等于替原作者改稿，而且改到哪一条停手是随意的；自动摘的好处是
    **摘了什么全在日志里**，读的人能自己判断这份参照还剩下多少原意。
    """
    removed: list[tuple[int, str]] = []
    proc = _run_once(mission, workdir, text)

    for _ in range(_PRUNE_LIMIT):
        if proc.returncode == 0:
            break
        match = _UNKNOWN_CMD.search(proc.stdout)
        if match is None:
            break
        line_no = int(match.group(3))          # 报错行号是唯一要用的
        lines = text.splitlines(keepends=True)
        if not (1 <= line_no <= len(lines)):
            break
        removed.append((line_no, lines[line_no - 1].strip()))
        del lines[line_no - 1]
        text = "".join(lines)
        proc = _run_once(mission, workdir, text)

    return text, proc, removed


def _restore_anchor(row: str) -> str:
    """把 ``time, name, lat偏移, lng偏移, alt, speed, heading`` 里的偏移加回锚点。

    落盘的 CSV 因此仍是**绝对经纬度**——偏移只是绕开 ``%g`` 的手段，不该
    渗到数据格式里让下游多记住一件事。
    """
    parts = row.split(",")
    if len(parts) < 4:
        return row
    parts[2] = f"{float(parts[2]) + _LAT_ANCHOR:.10f}"
    parts[3] = f"{float(parts[3]) + _LNG_ANCHOR:.10f}"
    return ",".join(parts)


def capture(
    source: Path,
    out_csv: Path,
    seconds: float | None,
    drop_names: set[str],
    dump: Path | None = None,
    replacements: tuple[tuple[str, str], ...] = (),
) -> int:
    """跑一次 AFSIM，把 ``TAG,`` 开头的行写成 CSV。返回采到的样本数。"""
    mission = _mission_exe()
    text, notes = _prepare(source, seconds, drop_names, replacements)

    for note in notes:
        print(f"  · {note}")

    with tempfile.TemporaryDirectory(prefix="milsim_afsim_") as work:
        workdir = Path(work)
        for note in _stage_assets(source, workdir, text):
            print(f"  · {note}")
        text, proc, removed = _prune_unknown(mission, workdir, text)

        rows: list[str] = []
        for line in proc.stdout.splitlines():
            if line.startswith(f"{DEFAULT_TAG},"):
                # 掐掉前缀，否则写出来的 CSV 第一列是标签而不是时间
                rows.append(_restore_anchor(line.split(",", 1)[1]))

    if dump is not None:
        dump.parent.mkdir(parents=True, exist_ok=True)
        dump.write_text(text, encoding="utf-8")
        print(f"  · 实际喂给 AFSIM 的脚本存了一份 → {dump}")

    if removed:
        commands = sorted({code.split()[0] for _no, code in removed if code})
        print(f"  · 汉化版脚本里有 {len(removed)} 行是 AFSIM 2.9.0 认不出的命令，"
              f"已逐行摘掉才能跑：{', '.join(commands)}")
    if proc.returncode != 0:
        tail = "\n".join(proc.stdout.splitlines()[-25:])
        raise SystemExit(f"AFSIM 跑失败（退出码 {proc.returncode}）：\n{tail}")
    if not rows:
        raise SystemExit("AFSIM 跑通了但一个样本都没采到——MOVER_UPDATED 没触发？")

    header = "time_s,platform,lat_deg,lon_deg,alt_m,speed_mps,heading_deg"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    out_csv.write_text(header + "\n" + "\n".join(rows) + "\n", encoding="utf-8")

    print(f"  · 采到 {len(rows)} 个样本、"
          f"{len({r.split(',')[1] for r in rows})} 个平台 → {out_csv}")
    return len(rows)


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    source = Path(argv[1])
    out_csv = Path(argv[2])
    seconds: float | None = None
    drop_names: set[str] = set()
    dump: Path | None = None
    replacements: list[tuple[str, str]] = []
    rest = list(argv[3:])
    index = 0
    while index < len(rest):
        item = rest[index]
        if item == "--seconds" and index + 1 < len(rest):
            seconds = float(rest[index + 1])
            index += 2
            continue
        if item == "--drop-name" and index + 1 < len(rest):
            drop_names.add(rest[index + 1])
            index += 2
            continue
        if item == "--dump" and index + 1 < len(rest):
            dump = Path(rest[index + 1])
            index += 2
            continue
        if item == "--replace" and index + 2 < len(rest):
            replacements.append((rest[index + 1], rest[index + 2]))
            index += 3
            continue
        print(f"无法识别的参数：{item}", file=sys.stderr)
        return 2
    if not source.is_file():
        print(f"找不到 AFSIM 脚本：{source}", file=sys.stderr)
        return 2
    print(f"[AFSIM 参照] {source.name}")
    capture(source, out_csv, seconds, drop_names, dump, tuple(replacements))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
