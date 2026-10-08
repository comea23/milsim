"""探测 → 通信传航迹 → 引导干扰：两边并排看**四个中间量**。

★ **这个工具是"数值对照"，不是"相等性检验"**（v0.13.38 二次核查）：
  两边比的是**不同粒度、不同门限、不同节拍**的量，所以**读数接近不等于模型一致**。
  实测的不一致项与真因（详见 `demos/afsim_comm_track_jam/README.md`）：

    ① 探测次数      milsim 480 vs AFSIM 1922  ⇒ 扇扫步长 vs 每秒转一圈（§9-70）
    ② 传航迹        delivered 119 vs 69       ⇒ 同量级（1.7×），可对照
    ③ 置向          aimed 1165（每拍）vs TASK_ASSIGNED 2（事件）⇒ 同名不同义
    ④ 压制          Pd 1e-6 vs 通过率 42.2%   ⇒ **两边 N 不同、门限差 8.95 dB**（§9-68）

  ⇒ **真正逐位可对的是 `S/I`**（§5.14.13 已有 `Δ(S/I) ≤ 8.6e-04 dB`、37/37 的
    既成结论），**不是 `Detected`**（§9-67：`Detected = (Pd ≥ RequiredPd)`，而
    AFSIM 的 `RequiredPd` 是随机量 `1−U(0,1)`，两边门限不可比）。

这条链跨三个部件（感知 / 通信 / 电子战），且 AFSIM 侧**没有任何一份 demo
同时具备这两个环节**，所以 AFSIM 侧的基线是拼的（见
``demos/afsim_comm_track_jam/comm_track_jam.txt`` 的页首注释）：

    milsim                                        AFSIM
    ─────────────────────────────────────────────────────────────────────
    ① 探测      SCOUT_1 的雷达建航迹              SENSOR_DETECTION_ATTEMPT
                                                  (Detected: 1)
    ② 传航迹    comm.delivered（通信件投递）      MESSAGE_RECEIVED
    ③ 置向      jam.aimed（挑中敌航迹）           TASK_ASSIGNED /
                                                  JAMMING_REQUEST_INITIATED
    ④ 压制      受害雷达 jamming_w>0 且 Pd 掉      JAMMING_ATTEMPT +
                                                  S/(N+C+J) < S/N

★ **只比值、不比机制**（用户裁定）：AFSIM 的干扰机是
  ``StartJamming(TRACK, ...)`` 按**目标对象**打、天线由内部
  ``WithinFieldOfView`` 判；milsim 的 ``aim_at`` 读**本机航迹表**算方位角。
  语义等价（都是"看见才打"），机制不同 —— 这条差异显式记在 README 里，
  不当 bug。

用法::

    # milsim 侧
    python tools/demo_comm_track_jam.py --ours-only

    # 两边一起（要先设 AFSIM_MISSION 或把本工具放在 AFSIM 根的 milsim/tools/）
    python tools/demo_comm_track_jam.py \\
        --afsim ../demos/electronic_warfare/comm_jamming.txt \\
        --repro demos/afsim_comm_track_jam/comm_track_jam.txt
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from milsim.models import register_framework  # noqa: E402
from milsim.services.type_registry import (  # noqa: E402
    ComponentRegistry,
    PlatformRegistry,
)
from milsim.simulation import Simulation  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

#: 事件块的头一行：时间戳在最前面（``T=`` 那种格式没有），字段用空格分隔。
#: ★ ``re.S`` **必须**：一个事件块是**多行**字符串（``\`` 续行拼回来的），
#: 而 ``.*$`` 在不加 ``re.S`` 时匹配不到跨行内容 —— 踩过：漏掉 ``re.S``
#: 时 5000 个块只认出 40 个（只有不带续行的那几类能认出）。
_EVENT = re.compile(r"^([-+0-9.eE]+)\s+([A-Z_]{6,})\s(.*)$", re.S)


def parse_args(argv: list[str]) -> dict:
    opts: dict = {
        "ours_only": False,
        "afsim": None,
        "repro": None,
        "seconds": 480.0,
    }
    index = 1
    while index < len(argv):
        flag = argv[index]
        if flag == "--ours-only":
            opts["ours_only"] = True
            index += 1
        elif flag == "--afsim" and index + 1 < len(argv):
            opts["afsim"] = argv[index + 1]
            index += 2
        elif flag == "--repro" and index + 1 < len(argv):
            opts["repro"] = argv[index + 1]
            index += 2
        elif flag == "--seconds" and index + 1 < len(argv):
            opts["seconds"] = float(argv[index + 1])
            index += 2
        else:
            print(f"无法识别的参数：{flag}", file=sys.stderr)
            raise SystemExit(2)
    return opts


# --------------------------------------------------------------------- milsim


def build_repro(path: Path) -> Simulation:
    components = ComponentRegistry()
    platforms = PlatformRegistry()
    register_framework(components, platforms)
    sim = Simulation.from_scenario(
        path.read_text(encoding="utf-8"),
        name=path.name,
        components=components,
        platforms=platforms,
    )
    return sim


def run_ours(path: Path, seconds: float) -> dict:
    """跑 milsim 侧想定，采四个中间量。"""
    sim = build_repro(path)
    sim.initialize()

    total_us = int(seconds * 1_000_000)
    step_us = 2_000_000
    #: ④ 压制前 / 后的两个读数，用来判"Pd 掉没掉"。
    first: dict[str, float] = {}
    last: dict[str, float] = {}
    first_pd: dict[str, float] = {}
    last_pd: dict[str, float] = {}

    name_of = {e.entity_id: e.name for e in sim.entities.values()}
    radars: dict[int, object] = {}
    for eid, entity in sim.entities.items():
        for part in entity.parts():
            if hasattr(part, "last_pd"):
                radars[eid] = part

    elapsed = 0
    #: 时序因果：四段链路各自"第一次发生"的时刻（s），与 AFSIM 侧对齐着看。
    timeline: dict[str, float] = {}
    while elapsed < total_us:
        sim.run_for(step_us)
        elapsed += step_us
        # ① 探测：第一张航迹落进任一张表
        if "detect" not in timeline and any(
            sim.store.contacts_of(e.entity_id) for e in sim.entities.values()
        ):
            timeline["detect"] = elapsed / 1_000_000
        # ② 传航迹：通信件第一次真投递进收件箱
        if "delivered" not in timeline and sim.comm.delivered > 0:
            timeline["delivered"] = elapsed / 1_000_000
        # ③ 置向：干扰机第一次挑中敌航迹
        if "aimed" not in timeline and sim.jam.aimed > 0:
            timeline["aimed"] = elapsed / 1_000_000
        # 采样：每台雷达第一次 / 最后一次非零 jamming_w 与对应 Pd
        for eid, radar in radars.items():
            name = name_of[eid]
            if name not in first and radar.jamming_w > 0.0:
                first[name] = radar.jamming_w
                first_pd[name] = radar.last_pd
                timeline.setdefault("jamming", elapsed / 1_000_000)
            if radar.jamming_w > 0.0:
                last[name] = radar.jamming_w
                last_pd[name] = radar.last_pd

    # ① 探测：谁的表里有几条航迹
    detections = {
        e.name: len(sim.store.contacts_of(e.entity_id)) for e in sim.entities.values()
    }

    return {
        "detections": detections,
        "delivered": sim.comm.delivered,
        "routed": sim.comm.routed,
        "screened": sim.comm.screened,
        "jammed_nodes": sim.comm.jammed,
        "epoch": sim.comm.epoch,
        "aimed": sim.jam.aimed,
        "aim_failed": sim.jam.aim_failed,
        "first_jamming_w": first,
        "last_jamming_w": last,
        "first_pd": first_pd,
        "last_pd": last_pd,
        "radars": {name_of[eid]: r for eid, r in radars.items()},
        "timeline": timeline,
    }


# ---------------------------------------------------------------------- AFSIM


def _mission_exe() -> Path:
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
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise SystemExit(
        "找不到 mission.exe。设环境变量 AFSIM_MISSION 指向它，"
        "或把本工具放在 AFSIM 根的 milsim/tools/ 下。找过：\n  "
        + "\n  ".join(seen)
    )


def run_afsim(script: Path) -> dict:
    """跑 AFSIM 侧想定，解析 ``.evt`` 采四个中间量。

    ★ ``cwd`` 必须是**想定所在目录**（AFSIM 的 ``include`` /
    ``define_path_variable`` 都相对它解析），``event_output`` 的父目录要
    先建好 —— 这两条都是 AFSIM 侧的静默坑（不建目录会 FATAL，但报的是
    找不到文件而不是"目录不存在"）。
    """
    mission = _mission_exe()
    workdir = script.parent
    (workdir / "output").mkdir(parents=True, exist_ok=True)

    completed = subprocess.run(
        [str(mission), "-sm", script.name],
        cwd=str(workdir),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=1800,
    )
    # 事件文件名由想定里的 define_path_variable CASE 定，别写死。
    evt = sorted((workdir / "output").glob("*.evt"))
    if completed.returncode != 0 or not evt:
        tail = "\n".join(completed.stdout.splitlines()[-25:])
        raise SystemExit(f"AFSIM 跑失败（exit={completed.returncode}）：\n{tail}")

    return parse_evt(evt[-1], completed.stdout)


def parse_evt(path: Path, stdout: str = "") -> dict:
    """从 ``.evt`` 里数四个中间量。**只数事件、不重算物理量**。

    ★ ``SENSOR_DETECTION_ATTEMPT`` 块**不带** ``Detected`` 字段 —— 除非
    想定的 sensor 块里写了 ``swerling_case`` / ``probability_of_false_alarm``
    / ``required_pd`` 三件套（缺了它们 AFSIM 不做概率判决，块到 ``RcvrBeam``
    那行就结束；实测连 "Detected" 这个词都不出现）。所以下面顺带**数一下
    块里有没有结算行**，缺了就显式记一笔，免得"0 次"被误读成"没探到"。
    """
    from collections import Counter

    counts: Counter[str] = Counter()
    #: 每种事件的第一条时刻（看时序因果：谁先谁后）。
    first_at: dict[str, float] = {}
    #: ④ 压制的数值落点：被干扰后 ``S/(N+C+J)`` 应当小于 ``S/N``。
    sn_ratio: list[float] = []
    s_jam_ratio: list[float] = []
    text = path.read_text(encoding="utf-8", errors="replace")

    # ★ AFSIM 的 .evt 是**续行**格式：一条事件后面跟若干以空格开头的续行，
    #   除最后一行外都以 ``\`` 结尾。必须先拼回逻辑行，否则
    #   ``S/N:`` / ``Detected:`` 这些**只出现在续行里**的字段永远搜不到
    #   （踩过：只按行首匹配，Detected 恒得 0）。
    blocks: list[str] = []
    pending: list[str] = []
    for raw in text.splitlines():
        if raw.endswith("\\"):
            pending.append(raw[:-1])
            continue
        if pending:
            pending.append(raw)
            blocks.append("\n".join(pending))
            pending = []
        else:
            blocks.append(raw)
    if pending:
        blocks.append("\n".join(pending))

    for block in blocks:
        head = _EVENT.match(block)
        if not head:
            continue
        when, kind = head.group(1), head.group(2)
        counts[kind] += 1
        first_at.setdefault(kind, float(when))
        if kind != "SENSOR_DETECTION_ATTEMPT":
            continue
        #: ``Detected: 1`` / ``Detected: 0`` —— 真实字段名就是这个（不是
        #: ``Detected: true``，实测过，写 true 会恒得 0）。
        if "Detected: 1" in block:
            counts["_DETECTED_TRUE"] += 1
            first_at.setdefault("_DETECTED_TRUE", float(when))
        elif "Detected: 0" in block:
            counts["_DETECTED_FALSE"] += 1
        elif "Detected:" not in block:
            counts["_NO_VERDICT"] += 1
        #: ④ 压制的两个数（同一次探测里 S/N 与 S/(N+C+J) 的对比）。
        sn = re.search(r"S/N:\s*([-+0-9.eE]+)", block)
        sj = re.search(r"S/\(N\+C\+J\):\s*([-+0-9.eE]+)", block)
        if sn:
            sn_ratio.append(float(sn.group(1)))
        if sj:
            s_jam_ratio.append(float(sj.group(1)))

    return {
        "evt": path,
        "counts": counts,
        "first_at": first_at,
        "stdout": stdout,
        "started_jamming": len(re.findall(r"started jamming", stdout)),
        "received_track": len(re.findall(r"received track", stdout)),
        "sn_min": min(sn_ratio) if sn_ratio else None,
        "sn_max": max(sn_ratio) if sn_ratio else None,
        "sj_min": min(s_jam_ratio) if s_jam_ratio else None,
        "sj_max": max(s_jam_ratio) if s_jam_ratio else None,
    }


# ----------------------------------------------------------------------- 呈现


def report(ours: dict, afsim: dict | None) -> list[str]:
    lines: list[str] = []
    lines.append("=" * 74)
    lines.append("探测 → 通信传航迹 → 引导干扰：四个中间量对拍")
    lines.append("=" * 74)

    lines.append("")
    lines.append("【① 探测】")
    for name, n in sorted(ours["detections"].items()):
        lines.append(f"    milsim  {name:<12} 航迹 {n} 条")
    if afsim is not None:
        c = afsim["counts"]
        lines.append(
            "    AFSIM   SENSOR_DETECTION_ATTEMPT %d 次"
            "  →  Detected: 1 %d 次 / Detected: 0 %d 次"
            % (
                c.get("SENSOR_DETECTION_ATTEMPT", 0),
                c.get("_DETECTED_TRUE", 0),
                c.get("_DETECTED_FALSE", 0),
            )
        )
        if c.get("_NO_VERDICT"):
            lines.append(
                "            ⚠ %d 次块里没有结算行（sensor 块缺 swerling_case /"
                " probability_of_false_alarm / required_pd 三件套）"
                % c["_NO_VERDICT"]
            )
        lines.append(
            "            SENSOR_TRACK_INITIATED %d 条 / SENSOR_TRACK_UPDATED %d 次"
            % (
                c.get("SENSOR_TRACK_INITIATED", 0),
                c.get("SENSOR_TRACK_UPDATED", 0),
            )
        )

    lines.append("")
    lines.append("【② 传航迹（经通信件）】")
    lines.append(
        "    milsim  delivered=%d  routed=%d  screened=%d  epoch=%d"
        % (ours["delivered"], ours["routed"], ours["screened"], ours["epoch"])
    )
    if afsim is not None:
        c = afsim["counts"]
        lines.append(
            "    AFSIM   MESSAGE_QUEUED=%d  MESSAGE_TRANSMITTED=%d"
            "  MESSAGE_DELIVERY_ATTEMPT=%d  MESSAGE_RECEIVED=%d  MESSAGE_DISCARDED=%d"
            % (
                c.get("MESSAGE_QUEUED", 0),
                c.get("MESSAGE_TRANSMITTED", 0),
                c.get("MESSAGE_DELIVERY_ATTEMPT", 0),
                c.get("MESSAGE_RECEIVED", 0),
                c.get("MESSAGE_DISCARDED", 0),
            )
        )
        lines.append(f"            （脚本侧收到航迹 {afsim['received_track']} 次）")

    lines.append("")
    lines.append("【③ 置向（照航迹）】")
    lines.append(
        "    milsim  aimed=%d  aim_failed=%d"
        % (ours["aimed"], ours["aim_failed"])
    )
    if afsim is not None:
        c = afsim["counts"]
        lines.append(
            "    AFSIM   TASK_ASSIGNED=%d  JAMMING_REQUEST_INITIATED=%d"
            "  TASK_CANCELED=%d  JAMMING_REQUEST_CANCELED=%d"
            % (
                c.get("TASK_ASSIGNED", 0),
                c.get("JAMMING_REQUEST_INITIATED", 0),
                c.get("TASK_CANCELED", 0),
                c.get("JAMMING_REQUEST_CANCELED", 0),
            )
        )
        lines.append(f"            （脚本侧 started jamming {afsim['started_jamming']} 次）")

    lines.append("")
    lines.append("【④ 压制生效】")
    lines.append("     ★ 两边『判决通过率』不可比：AFSIM N=16、milsim N=1 ⇒ 门限差 8.95 dB（§9-68）")
    for name, radar in sorted(ours["radars"].items()):
        w0 = ours["first_jamming_w"].get(name)
        w1 = ours["last_jamming_w"].get(name)
        p0 = ours["first_pd"].get(name)
        p1 = ours["last_pd"].get(name)
        if w1 is None:
            lines.append(f"    milsim  {name:<12} 全程未被压（jamming_w 恒 0）")
        else:
            lines.append(
                f"    milsim  {name:<12} jamming_w {w0:.4g} → {w1:.4g} W"
                f"   Pd {p0:.4f} → {p1:.4f}"
            )
    if afsim is not None:
        counts = afsim["counts"]
        lines.append("    AFSIM   JAMMING_ATTEMPT=%d 次" % (counts.get("JAMMING_ATTEMPT", 0),))
        #: ★ "压制生效"唯一能拿到的**数值**：同一批探测里 S/N 与 S/(N+C+J)
        #: 的极值。被压制的那几次必然 ``S/(N+C+J) < S/N``。
        if afsim["sj_min"] is not None:
            lines.append(
                "            S/N 区间 [%.4g, %.4g] dB     S/(N+C+J) 区间 [%.4g, %.4g] dB"
                % (afsim["sn_min"], afsim["sn_max"], afsim["sj_min"], afsim["sj_max"])
            )
            gap = afsim["sn_min"] - afsim["sj_min"]
            lines.append(
                "            最深一次压制：S/(N+C+J) 比 S/N 低 %.4g dB"
                % gap
            )
            if gap <= 0.0:
                lines.append(
                    "            ⚠ 没有任何一次探测被压（S/(N+C+J) 从未低于 S/N）"
                )

    lines.append("")
    lines.append("【时序因果（第一次出现的时刻，s）】")
    lines.append("    —— 四段链路的先后顺序就是「这条链真的串起来了」的证据。")
    lines.append("    %-22s %-14s %-14s" % ("段", "milsim", "AFSIM"))
    ours_tl = ours["timeline"]
    ours_rows = (
        ("① 探测", "detect", "SENSOR_DETECTION_ATTEMPT"),
        ("② 传航迹", "delivered", "MESSAGE_RECEIVED"),
        ("③ 置向", "aimed", "TASK_ASSIGNED"),
        ("④ 压制", "jamming", "JAMMING_ATTEMPT"),
    )
    for label, our_key, af_key in ours_rows:
        our_t = ours_tl.get(our_key)
        our_s = "t = %.3f" % our_t if our_t is not None else "（未发生）"
        if afsim is not None and af_key in afsim["first_at"]:
            af_s = "t = %.3f" % afsim["first_at"][af_key]
        elif afsim is not None:
            af_s = "（未发生）"
        else:
            af_s = "—"
        lines.append("    %-22s %-14s %-14s" % (label, our_s, af_s))
    return lines


def main(argv: list[str]) -> int:
    opts = parse_args(argv)

    if opts["repro"] is None:
        opts["repro"] = str(ROOT / "scenarios" / "comm_track_jam.txt")

    ours = run_ours(Path(opts["repro"]), opts["seconds"])

    afsim = None
    if not opts["ours_only"]:
        target = opts["afsim"]
        if target is None:
            target = str(
                ROOT / "demos" / "afsim_comm_track_jam" / "comm_track_jam.txt"
            )
        afsim = run_afsim(Path(target))

    for line in report(ours, afsim):
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
