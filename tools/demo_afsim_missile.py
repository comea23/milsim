"""AFSIM `demos/new_guidance/` 的**弹**对照：弹体 + 制导件（§5.11）。

前三份工具比的都是"**谁跟得住航线**"——跑机动件。这一份比的是 §5.8 的那两个
部件：`MissileMover`（能飞多快、烧多少油）与 `GuidanceComputer`（要往哪飞）。
两边各是：

.. code-block:: text

    AFSIM    WSF_GUIDED_MOVER      +  WSF_GUIDANCE_COMPUTER
    本项目   MISSILE_MOVER         +  ASM_GUIDANCE（跃升俯冲，v0.13.22 起）
                                      CRUISE_GUIDANCE / BALLISTIC_GUIDANCE

脚本选的是三份里**最简**的那一份 `new_guidance3.txt`：蓝方战斗机在
39.5N/89.3W、25000 ft、450 kts 上对着一条**预装订航迹**把 `blue_asm_missile`
打出去，目标是 38.6N/90.3W 的静止目标。选它是因为另两份（`new_guidance1/2`）
多出来的是雷达、航迹与中途更新，那些属感知/数据链，不是弹体这一层的事。

★ 这一份**不是原 demo 的等价复现**，读结果之前先读 `demos/afsim_mover/README.md`
   §11。截至 v0.13.22：
   · **阶段表照抄过来了**（`ASM_GUIDANCE` 的 `CRUISE → POPUP → DIVE`），三段
     都飞得出来，巡航高度与参照同为 75 m；
   · **横向仍然对不上**——本项目没有弹的航路跟随（`MissileMover.move_to_point`
     直接报错），所以复现侧是**直扑目标**，参照侧先走完预规划航路再转向；
   · **末段仍然打不中**：同样的弹道倾角下复现侧速度高约 100 m/s，弹从目标
     上方 772 m 掠过。根因是弹体缺"转弯耗能"（诱导阻力不在阻力里），
     见 §9-36。

用法::

    # 采参照 + 跑复现 + 对照 + 出图（一条命令跑完）
    python tools/demo_afsim_missile.py \
        --afsim ../demos/new_guidance/new_guidance3.txt \
        --repro demos/afsim_mover/missile_guidance.txt \
        --ref   demos/afsim_mover/ref_new_guidance3.csv \
        --out   demos/afsim_mover/repro_new_guidance3.csv \
        --plot  docs/images/afsim_missile_compare.png \
        --seconds 700

`--afsim` 与 `--ref` 都给时采参照（需要 `AFSIM_MISSION` 指向 `mission.exe`）；
只给 `--ref` 时直接读现成的参照 CSV——**参照数据是怎么产生的和参照数据一样
重要**，所以采它的那条命令要能重跑。
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "src"))
sys.path.insert(0, str(_HERE))

import afsim_ref  # noqa: E402  采参照（与前三份共用，语义只有一处）

_M_PER_FT = 0.3048
_MPS_PER_KT = 0.514444

#: 参照里的弹叫什么。这一份的弹是 `fighter_blue_asm_missile_1`——**载机名是
#: 前缀**，所以"名字里含 fighter"不能当"这是载机"的判据（踩过，见 :func:`read_ref`）。
_MISSILE = "missile"

#: 参照侧的三个质量/时刻数——**人工从 AFSIM 的输出里抄的，不是采出来的**。
#: :mod:`afsim_ref` 的观测器只写 ``time/platform/lat/lon/alt/speed/heading``
#: 七列，**没有质量**；这两个质量只能读 AFSIM 自己的 stdout / ``mission.log``::
#:
#:     New Phase: POPUP T=604.1  Alt 75 m  Downrange 129423 m  Mass 499.16 kg
#:     （发射：39.5N/89.3W、7620 m、231.5 m/s；stage 1 写的是 initial_mass 519 kg）
#:
#: 要闭环这一条，得给观测器加一列 ``aPlatform.Mass()`` 再重采一次——那会改到
#: 另外三份 CSV 的列，所以**不顺手改**，记在 README §11.2 的"未闭环项"里。
_REF_LAUNCH_KG = 519.0
_REF_POPUP_T_S = 604.1
_REF_POPUP_KG = 499.16


# ---------------------------------------------------------------------------
# 读参照
# ---------------------------------------------------------------------------

def read_ref(path: Path) -> list[dict[str, float]]:
    """读参照 CSV，**只留弹那一路**。

    ``speed_mps`` 那一列是 AFSIM 的 ``Speed()``——**三维**速度矢量的大小。
    本项目写进存储的是**水平**速率，所以复现侧写 CSV 时也要写
    :meth:`~milsim.models.mover.missile.MissileMover.total_speed_mps`，
    否则两边同名的一列不是同一件事（前三份踩过，见 README §7.3）。
    """
    rows: list[dict[str, float]] = []
    with path.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            # **不能用"名字里有没有载机"来筛**：这一份的弹叫
            # `fighter_blue_asm_missile_1`（挂架命名，AFSIM 把载机名当前缀），
            # 它**也含** `fighter` ——第一版就是这么把弹一起滤掉、
            # 然后报"一个弹的样本都没有"的。认 "missile" 才唯一。
            if "missile" not in row["platform"]:
                continue
            rows.append({
                "t": float(row["time_s"]),
                "lat": float(row["lat_deg"]),
                "lon": float(row["lon_deg"]),
                "alt": float(row["alt_m"]),
                "spd": float(row["speed_mps"]),
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def dump_names(path: Path) -> list[str]:
    with path.open(encoding="utf-8") as handle:
        return sorted({row["platform"] for row in csv.DictReader(handle)})


# ---------------------------------------------------------------------------
# 跑复现
# ---------------------------------------------------------------------------

def run_repro(scenario: Path, seconds: float):
    """跑本项目那份想定，按**弹自己的节拍**采样本，返回 ``(数据, 汇总)``。

    ★ 采样间隔取 2 s，不是为了好看：引擎只在**事件时刻**停
    （``run_for`` 是上限不是步长），而弹的 ``period`` 默认就是 2 s。
    按 0.5 s 请求的话每次实际仍推进 2 s，拿"请求次数 × 间隔"当时间轴会
    **把 546 s 记成 136.5 s**——本轮就是这么错了一次的。所以时间一律取
    ``body.elapsed_s()``（弹自己的钟），不自己算。
    """
    from milsim.services.type_registry import default_platforms, default_registry
    from milsim.simulation import Simulation

    source = scenario.read_text(encoding="utf-8")
    sim = Simulation.from_scenario(
        source, name=scenario.name,
        components=default_registry(), platforms=default_platforms(),
    )
    sim.build()
    sim.initialize()

    eid = body = guide = None
    for entity in sim.entities.values():
        mover = entity.component("mover")
        if mover is not None and mover.guidance() is not None:
            eid, body, guide = entity.entity_id, mover, mover.guidance()
    if body is None:
        raise SystemExit(f"{scenario.name}: 没找到带制导件的弹平台")

    frame = sim.maps.active_zones[0].frame

    samples: list[dict[str, float]] = []
    seen = -1.0
    for _ in range(int(seconds * 1_000_000 / 2_000_000)):
        sim.run_for(2_000_000)
        if body.elapsed_s() <= seen:
            continue
        seen = body.elapsed_s()
        x, y, z, _heading, _speed = sim.store.pose_of(eid)
        lat, lon = frame.world_to_geo(x, y)
        samples.append({
            "t": body.elapsed_s(), "x": x, "y": y, "lat": lat, "lon": lon,
            "alt": z, "spd_h": _speed, "spd": body.total_speed_mps(),
            "mass": body.mass_kg(),
        })
        if body.terminate_reason():
            break

    summary = {
        "reason": body.terminate_reason(),
        "hit": guide.hit(),
        "fuze_m": guide.detonation_range_m(),
        "path_km": body.travelled_m() / 1000.0,
        "peak_mps": body.max_speed_mps(),
        "peak_g": body.peak_accel() / 9.80665,
        "cap_g": float(body.spec["radial_accel"]) / 9.80665,
        "fuel_kg": body.fuel_kg(),
        "mass_kg": body.mass_kg(),
        "launch_kg": float(body.spec["mass"]),
        "seconds": body.elapsed_s(),
        "timeline": guide.timeline(),
        "frame": frame,
        "guide_reason": guide.terminate_reason(),
    }
    return samples, summary


# ---------------------------------------------------------------------------
# 对照
# ---------------------------------------------------------------------------

def _to_local(frame, lat: float, lon: float) -> tuple[float, float]:
    return frame.geo_to_world(lat, lon)


def _run(ref: list[dict[str, float]], key: str) -> list[float]:
    """把一串读数里**连续相同的**压成一个值（巡航段就是它）。"""
    out: list[float] = []
    for row in ref:
        if not out or abs(row[key] - out[-1]) > 1e-9:
            out.append(row[key])
    return out


def _plateau(rows: list[dict[str, float]], key: str,
             quantum: float) -> tuple[float, float, float]:
    """最长的"保持在一个量化档里"的读数段 → ``(值, 起, 止)``。巡航高度/速率靠它认。

    **必须量化之后再比**：AFSIM 的观测值经 ``writeln`` 的 ``%g`` 打出来，
    巡航段的高度是 74.98 / 75.01 / 75.00 这样的数——逐字相等判"保持"会把
    533 s 的巡航段切成 2 s 一段，然后报出**第一段**（爬升前的 7620 m）
    当巡航高度。第一版就是这么错的，而且数看着完全合理。
    """
    best = (0.0, 0.0, 0.0, 0.0)          # 长度, 值, 起, 止
    start, prev = rows[0]["t"], round(rows[0][key] / quantum)
    for row in rows[1:]:
        level = round(row[key] / quantum)
        if level != prev:
            if row["t"] - start > best[0]:
                best = (row["t"] - start, prev * quantum, start, row["t"])
            start, prev = row["t"], level
    if rows[-1]["t"] - start > best[0]:
        best = (rows[-1]["t"] - start, prev * quantum, start, rows[-1]["t"])
    return best[1], best[2], best[3]


def _flat_len(rows: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(rows, rows[1:]))


def report(ref, samples, summary, frame) -> None:
    rt = ref[-1]["t"]
    ref_local = [_to_local(frame, r["lat"], r["lon"]) for r in ref]
    our_local = [(s["x"], s["y"]) for s in samples]

    # 两边**同一套算法**认巡航段：高度量化 5 m、速率量化 1 m/s。
    # 速率量化档要大于巡航段的抖动（参照 240.0~240.2 m/s）。
    r_alt, r_a0, r_a1 = _plateau(ref, "alt", 5.0)
    r_spd, r_s0, r_s1 = _plateau(ref, "spd", 1.0)
    mine = [{"t": s["t"], "alt": s["alt"], "spd": s["spd"]} for s in samples]
    o_alt, o_a0, o_a1 = _plateau(mine, "alt", 5.0)
    o_spd, o_s0, o_s1 = _plateau(mine, "spd", 1.0)

    print("\n" + "=" * 78)
    print("弹：弹体（质量/推力/燃料/阻力）+ 制导件（阶段表）")
    print("=" * 78)
    print(f"{'':<16}{'AFSIM':>20}{'本项目':>20}")
    rows = [
        ("飞行时间 s", f"{rt:.1f}", f"{summary['seconds']:.1f}"),
        ("巡航高度 m", f"{r_alt:.0f}（{r_a0:.0f}~{r_a1:.0f} s）",
         f"{o_alt:.0f}（{o_a0:.0f}~{o_a1:.0f} s）"),
        ("巡航速率 m/s", f"{r_spd:.1f}（{r_s0:.0f}~{r_s1:.0f} s）",
         f"{o_spd:.1f}（{o_s0:.0f}~{o_s1:.0f} s）"),
        ("水平平移 km", f"{math.dist(ref_local[0], ref_local[-1]) / 1000:.1f}",
         f"{math.dist(our_local[0], our_local[-1]) / 1000:.1f}"),
        ("水平航迹 km", f"{_flat_len(ref_local) / 1000:.1f}",
         f"{_flat_len(our_local) / 1000:.1f}"),
        ("峰值速率 m/s", f"{max(r['spd'] for r in ref):.1f}",
         f"{summary['peak_mps']:.1f}"),
        ("终止质量 kg", "499.16（604.1 s 时）", f"{summary['mass_kg']:.1f}"),
        ("峰值过载 g", "无显式上限",
         f"{summary['peak_g']:.2f} / 上限 {summary['cap_g']:.1f}"),
        ("终止", "命中（脱靶 0.0 m）",
         f"{summary['reason']}（起爆 {summary['fuze_m']:.1f} m）"),
    ]
    for label, a, b in rows:
        print(f"{label:<16}{a:>20}{b:>20}")

    ours = [r for r in summary["timeline"]]
    print("\n阶段：本项目 "
          + " → ".join(f"{r[0]}({r[1]:.0f}~{'—' if r[2] is None else f'{r[2]:.0f}'}s)"
                       for r in ours))
    print("      参照是同名的三段（CRUISE → POPUP → DIVE），但**观测器采不到**"
          "——它那 7 列里没有 phase，\n      只能从 stdout 的 phase change 事件"
          "手抄时刻：POPUP T=604.1、DIVE T=610.1。")

    burn = summary["launch_kg"] - summary["mass_kg"]
    ref_burn = _REF_LAUNCH_KG - _REF_POPUP_KG
    print(f"\n烧掉的燃料：参照 {ref_burn:.2f} kg / {_REF_POPUP_T_S:.1f} s = "
          f"{ref_burn / _REF_POPUP_T_S:.4f} kg/s"
          f"（{_REF_POPUP_T_S:.1f} s 时还没进 POPUP，所以这一段里"
          f"没有跃升 / 俯冲）")
    print(f"            本项目 {burn:.2f} kg / {summary['seconds']:.1f} s = "
          f"{burn / summary['seconds']:.4f} kg/s（含末段俯冲）")
    print("            **两个全程平均不可比**：窗口不一样（参照那段含起飞与"
          "爬升加速、不含末段；\n              本项目那段含末段）。要比就比巡航段。")
    seg = [s for s in samples if o_a0 <= s["t"] <= o_a1]
    if len(seg) > 2:
        dm = seg[0]["mass"] - seg[-1]["mass"]
        dt = seg[-1]["t"] - seg[0]["t"]
        print(f"                     ├ 本项目单看巡航段 {dm:.3f} kg / {dt:.0f} s"
              f" = {dm / dt:.4f} kg/s")
    print(f"                     └ 本项目满推力 0.2000 kg/s（= 参照的 "
          f"fuel_mass ÷ thrust_duration），全程平均油门 "
          f"{burn / summary['seconds'] / 0.2 * 100:.1f}%")


# ---------------------------------------------------------------------------
# 落盘与出图
# ---------------------------------------------------------------------------

_HEADER = "time_s,platform,lat_deg,lon_deg,alt_m,speed_mps,heading_deg"


def write_csv(path: Path, samples, name: str) -> None:
    """写成本项目复现 CSV。列名与参照**逐字相同**，便于下游一起读。

    ``speed_mps`` 那一列是**总**速率（水平 + 垂向），与参照的 ``Speed()``
    同口径——见 :func:`read_ref` 的说明。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [_HEADER]
    for s in samples:
        lines.append(f"{s['t']:.1f},{name},{s['lat']:.10f},{s['lon']:.10f},"
                     f"{s['alt']:.6f},{s['spd']:.6f},0.0")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  · 复现 {len(samples)} 个样本 → {path}")


def draw(path: Path, ref, samples, frame) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                       # pragma: no cover
        print(f"  · 没装 matplotlib（{exc}），跳过出图")
        return

    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.4), dpi=110)
    rt = [r["t"] for r in ref]
    ot = [s["t"] for s in samples]
    ref_local = [_to_local(frame, r["lat"], r["lon"]) for r in ref]
    our_local = [(s["x"], s["y"]) for s in samples]

    ax = axes[0]
    ax.plot(rt, [r["alt"] for r in ref], color="#1f77b4", lw=1.4, label="AFSIM")
    ax.plot(ot, [s["alt"] for s in samples], color="#d62728", lw=1.4, label="本项目")
    ax.set_xlabel("时间 s"); ax.set_ylabel("高度 m")
    ax.set_title("高度剖面"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(rt, [r["spd"] for r in ref], color="#1f77b4", lw=1.4, label="AFSIM")
    ax.plot(ot, [s["spd"] for s in samples], color="#d62728", lw=1.4, label="本项目")
    ax.set_xlabel("时间 s"); ax.set_ylabel("总速率 m/s")
    ax.set_title("速率剖面"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot([p[0] / 1000 for p in ref_local], [p[1] / 1000 for p in ref_local],
            color="#1f77b4", lw=1.4, label="AFSIM（走预规划航路）")
    ax.plot([p[0] / 1000 for p in our_local], [p[1] / 1000 for p in our_local],
            color="#d62728", lw=1.4, label="本项目（直扑，无航路跟随）")
    ax.scatter([ref_local[0][0] / 1000], [ref_local[0][1] / 1000],
               color="#1f77b4", s=28, marker="o")
    ax.scatter([our_local[0][0] / 1000], [our_local[0][1] / 1000],
               color="#d62728", s=28, marker="o")
    ax.scatter([ref_local[-1][0] / 1000], [ref_local[-1][1] / 1000],
               color="black", s=40, marker="*", label="目标")
    ax.set_xlabel("东向 km"); ax.set_ylabel("北向 km")
    ax.set_title("地面航迹（形状本来就不同，见 §11）")
    ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_aspect("equal")

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    print(f"  · 图 → {path}")


# ---------------------------------------------------------------------------

def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    opts: dict[str, str] = {}
    rest = list(argv[1:])
    index = 0
    while index < len(rest):
        item = rest[index]
        if item.startswith("--") and index + 1 < len(rest):
            opts[item[2:]] = rest[index + 1]
            index += 2
            continue
        print(f"无法识别的参数：{item}", file=sys.stderr)
        return 2

    ref_path = Path(opts["ref"])
    if opts.get("afsim"):
        print(f"[AFSIM 参照] {opts['afsim']}")
        afsim_ref.capture(
            Path(opts["afsim"]), ref_path,
            float(opts.get("seconds", 700.0)), set(), None, (),
        )

    ref = read_ref(ref_path)
    if not ref:
        raise SystemExit(f"{ref_path}: 一个弹的样本都没有")

    samples, summary = run_repro(Path(opts["repro"]),
                                float(opts.get("seconds", 700.0)))
    if opts.get("out"):
        write_csv(Path(opts["out"]), samples, Path(opts["repro"]).stem)
    report(ref, samples, summary, summary["frame"])
    if opts.get("plot"):
        draw(Path(opts["plot"]), ref, samples, summary["frame"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
