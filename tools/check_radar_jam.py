"""雷达 + 干扰的配合体检：组件级装配，扫距离看功率账。

与单测的分工：``test_percep.py`` 把桩做到最小来测单条性质；本工具把
**活的**雷达与**活的**电子战门面装在一起（真 ``JamService`` + 真
``JammerSpec``），扫一段距离把每个点的功率账摊开——

* 干净单脉冲 SNR 与受扰后 ``S/(N+J)`` 两条曲线（J 由组件自己算：
  名册 → 几何 → F_BW → 主/副瓣 → 极化 → Jam.1）；
* ``J/S`` 的对数斜率（支援干扰 J 常数 ⇒ ∝ R⁴；自卫干扰 J ∝ R⁻² ⇒ ∝ R²）；
* **烧穿距离** R*（J/S = 1 处：回波功率追平干扰功率）；
* 检测概率两支（无扰 / 受扰）与标定锚点 ``Pd(detect_range) = required_pd``。

用法::

    python tools/check_radar_jam.py                          # 标定雷达 + 支援干扰
    python tools/check_radar_jam.py --mode self              # 自卫干扰（与目标同址）
    python tools/check_radar_jam.py --no-jam                 # 只测雷达
    python tools/check_radar_jam.py --peak-power 250e3       # 逐参法（雷达方程）
    python tools/check_radar_jam.py --plot tools/check_radar_jam.png

默认走**标定法**（不给 ``peak_power``）：``detect_range`` 处 Pd 精确等于
``required_pd``，噪声功率用 ``--noise-power``（默认 1e-12 W）补上——标定法
要有绝对的 N，J/S 才有分母；没被干扰时它不影响任何数字。
"""

from __future__ import annotations

import argparse
import sys
from math import cos, log10, radians, sin
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR.parent / "src") not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR.parent / "src"))

from milsim.models.percep import equation as eq  # noqa: E402
from milsim.models.percep.search_radar import HexSearchRadar  # noqa: E402
from milsim.services.ew import JamService, JammerSpec  # noqa: E402
from milsim.services.map import Axial  # noqa: E402
from milsim.services.store import EntityStore, LocalCellRef  # noqa: E402

_SIZE = 1000.0
RADAR_ID = 0
TARGET_ID = 1
JAMMER_ID = 2
#: 目标摆在哪个方位（度，正北 0 顺时针）。90° = 正东，世界坐标里就是 (r, 0)。
TARGET_BEARING = 90.0


class _Spec(dict):
    slot = "sensor"
    type_name = "HEX_SEARCH_RADAR"


class _Engine:
    def __init__(self) -> None:
        self.now = 0
        self.beat = None

    def schedule_recurring(self, delay, interval, fn, priority=0):
        self.beat = (interval, fn)
        return None

    def tick_to(self, seconds: float) -> None:
        self.now = int(round(seconds * 1_000_000))
        if self.beat is not None:
            self.beat[1](self, None)


class _Streams:
    def __init__(self, seed: int) -> None:
        import random

        self._random = random.Random(seed)

    @property
    def sensor(self):
        return self._random


class _Mount:
    """只装雷达真正用到的那几个成员（与 test_percep 的桩同一份约定）。"""

    def __init__(self, engine, store, entity_id, *, jam=None, seed=0):
        self.engine = engine
        self.store = store
        self.entity_id = entity_id
        self.now = engine.now
        self.nav = None
        self._streams = _Streams(seed)
        self.jam = jam

    def sensor_view(self):
        return self.store.sensor_view(self.entity_id)

    def jam_service(self):
        return self.jam

    def cell_span_m(self):
        return _SIZE * 3 ** 0.5

    def every(self, interval_us, fn, priority=0):
        return self.engine.schedule_recurring(0, interval_us, fn, priority)

    def streams(self):
        return self._streams

    def target_platform_param(self, entity_id, name, default=None):
        return default


class _Sides:
    """阵营判定桩：雷达 red，目标与干扰机 blue，同阵营不打那道筛不会误筛。"""

    def __init__(self) -> None:
        self._sides = {RADAR_ID: "red", TARGET_ID: "blue", JAMMER_ID: "blue"}

    def side_of(self, entity_id: int) -> str:
        return self._sides.get(entity_id, "")


def add_entity(store: EntityStore, entity_id: int, x: float, y: float) -> None:
    """先落格（名义格跟着位置走），再写 pose——顺序反了 ``set_pose`` 会 KeyError。"""
    store.add(entity_id, LocalCellRef(1, 0, Axial(round(x / _SIZE), 0)))
    store.set_pose(entity_id, x, y, 0.0)


def jammer_xy(args: argparse.Namespace, target_range_m: float) -> tuple[float, float]:
    """干扰机的世界坐标（正北 = -y）。``self`` 模式与目标同方位同距离（自卫）。"""
    if args.mode == "self":
        r, b = target_range_m, radians(TARGET_BEARING)
    else:
        r, b = args.jam_range, radians(args.jam_bearing)
    return r * sin(b), -r * cos(b)


def build(args: argparse.Namespace):
    """装雷达 + 电子战门面，返回 ``(radar, store, frame, jam, jam_frequency)``。"""
    values = {name: item.default for name, item in HexSearchRadar.PARAMS.items()}
    values.update(dict(
        detect_range=args.detect_range,
        reference_rcs=args.rcs,
        required_pd=args.required_pd,
        probability_of_false_alarm=args.pfa,
        pulses_integrated=args.pulses,
        frequency=args.frequency,
        bandwidth=args.bandwidth,
        beam_width=args.beam_width,
        beamwidth_h=args.beamwidth_h,
        beamwidth_v=args.beamwidth_v,
        antenna_gain_db=args.gain_db,
        system_loss_db=args.system_loss,
        receive_loss_db=args.receive_loss,
        swerling_case=args.swerling,
    ))
    if args.peak_power > 0.0:
        values["peak_power"] = args.peak_power
        if args.noise_power is not None:
            raise SystemExit(
                "逐参法（--peak-power）不许再给 --noise-power：那时 N 由"
                "天线温度 + 馈线 + 噪声系数链算出，再给一个就是同一个量的"
                "第二个出处。要加噪声余量请调 --system-loss 或 --gain-db"
            )
    elif args.noise_power is None:
        # 标定法要参与干扰求值就必须有绝对的 N（J/S 的分母）；没干扰时它不影响
        # 任何数字。默认 1e-12 W = -120 dBm，工程上说得过去的接收机噪声量级。
        values["noise_power"] = 1.0e-12
    else:
        values["noise_power"] = args.noise_power

    jam_frequency = (args.jam_frequency if args.jam_frequency > 0.0
                     else args.frequency)

    store = EntityStore()
    add_entity(store, RADAR_ID, 0.0, 0.0)
    add_entity(store, TARGET_ID,
               1000.0 * sin(radians(TARGET_BEARING)), 0.0)

    jam = None
    if not args.no_jam:
        jam = JamService(store, _Sides())
        jx, jy = jammer_xy(args, 1000.0)
        add_entity(store, JAMMER_ID, jx, jy)
        jam.register(JammerSpec(
            entity_id=JAMMER_ID,
            power_w=args.jam_power,
            gain=eq.linear(args.jam_gain_db),
            frequency_hz=jam_frequency,
            bandwidth_hz=args.jam_bw,
            duty_cycle=1.0,
            use_peak_power=False,
            internal_loss=1.0,
            antenna_height_m=0.0,
            polarization=-1.0,
            polarization_type="default",
            ignore_same_side=True,
            beam_width_deg=0.0,
            aim_at="",
            tag="JAM_1",
        ))

    radar = HexSearchRadar(spec=_Spec(values))
    radar.initialize(_Mount(_Engine(), store, RADAR_ID, jam=jam))
    return radar, store, jam, jam_frequency


def sweep(radar, store: EntityStore, args: argparse.Namespace,
          has_jam: bool) -> list[dict]:
    """扫距离：每点挪目标（自卫模式连干扰机一起挪），算完整的功率账。"""
    rcs = args.rcs
    r_max = args.rmax if args.rmax > 0.0 else 1.6 * args.detect_range
    rows: list[dict] = []
    n = 241
    for i in range(1, n + 1):
        r = r_max * i / n
        x = r * sin(radians(TARGET_BEARING))
        store.set_pose(TARGET_ID, x, 0.0, 0.0)
        if has_jam and args.mode == "self":
            store.set_pose(JAMMER_ID, x, 0.0, 0.0)
        j = radar.received_jamming_w(TARGET_BEARING) if has_jam else 0.0
        snr = radar.single_pulse_snr(r, rcs)
        snr_j = (radar.single_pulse_snr(r, rcs, jammer_noise_w=j)
                 if has_jam else snr)
        s_w = snr * radar.noise_w
        js = (j / s_w) if (has_jam and s_w > 0.0) else 0.0
        pd = radar.detection_probability(r, rcs)
        pd_j = (radar.detection_probability(r, rcs, jammer_noise_w=j)
                if has_jam else pd)
        rows.append(dict(r=r, snr=snr, snr_j=snr_j, j=j, js=js, pd=pd, pd_j=pd_j))
    return rows


def burnthrough_range(rows: list[dict]) -> float | None:
    """J/S = 1 的距离：在相邻采样点之间线性插值。找不到（全程同侧）给 None。"""
    for a, b in zip(rows, rows[1:]):
        if a["js"] >= 1.0 > b["js"]:
            t = (a["js"] - 1.0) / (a["js"] - b["js"])
            return a["r"] + t * (b["r"] - a["r"])
        if a["js"] < 1.0 <= b["js"]:
            t = (1.0 - a["js"]) / (b["js"] - a["js"])
            return a["r"] + t * (b["r"] - a["r"])
    return None


def report(radar, rows: list[dict], args: argparse.Namespace,
           jam_frequency: float, has_jam: bool) -> None:
    calibrated = args.peak_power <= 0.0
    print("== 雷达（" + ("标定法：detect_range 处 Pd = required_pd" if calibrated
                        else "逐参法：雷达方程") + "）")
    print(f"  标称 {args.detect_range / 1000:.1f} km @ RCS {args.rcs:g} m2、"
          f"required_pd {args.required_pd:g}、Pfa {args.pfa:g}、"
          f"swerling {args.swerling}、积累 {args.pulses if args.pulses else '(按驻留)'}")
    print(f"  频率 {args.frequency / 1e9:.2f} GHz、接收带宽 {args.bandwidth / 1e6:.1f} MHz"
          f"、接收增益 {eq.db(radar.gain):.2f} dB、噪声 N = {radar.noise_w:.3e} W"
          f"（{eq.dbm(radar.noise_w):.1f} dBm）")
    if calibrated and radar.snr_at_reference > 0.0:
        print(f"  标定锚点 SNR = {eq.db(radar.snr_at_reference):.4f} dB"
              f"（该 SNR 下 Pd = {args.required_pd:g}）")

    if not has_jam:
        return
    print(f"== 干扰机（{args.mode}）")
    if args.mode == "standoff":
        print(f"  方位 {args.jam_bearing:g} 度、距离 {args.jam_range / 1000:.0f} km"
              f"（波束指目标 {TARGET_BEARING:g} 度，与干扰机夹 "
              f"{abs(args.jam_bearing - TARGET_BEARING):g} 度）")
    else:
        print("  与目标同址伴随（自卫干扰），波束指目标即干扰走主瓣")
    if args.jam_bw <= 0.0:
        fbw_note = "瞄准式（连续波）F_BW = 1"
    else:
        ratio = eq.bandwidth_overlap_ratio(
            jam_frequency, args.jam_bw, args.frequency, args.bandwidth)
        fbw_note = f"阻塞式 F_BW 约 {ratio:.3f}"
    print(f"  功率 {args.jam_power:g} W × 增益 {args.jam_gain_db:g} dB；"
          f"频率 {jam_frequency / 1e9:.2f} GHz、带宽 {args.jam_bw / 1e6:.1f} MHz"
          f"（{fbw_note}）")
    j0 = next((row["j"] for row in rows if row["j"] > 0.0), 0.0)
    print(f"  到达功率 J = {j0:.3e} W（{eq.dbm(j0):.1f} dBm）")
    if radar.noise_w > 0.0 and j0 > 0.0:
        print(f"  J/N = {j0 / radar.noise_w:.3g}"
              f"（{eq.db(j0 / radar.noise_w):.1f} dB）")
    j_side = radar.received_jamming_w((TARGET_BEARING + 180.0) % 360.0)
    if j0 > 0.0 and j_side > 0.0:
        print(f"  波束指干扰机 J = {j0:.3e} W，波束背离 180 度 J = {j_side:.3e} W："
              f"副瓣抑制 {eq.db(j0 / j_side):.1f} dB"
              f"（{args.beam_width:g} 度主瓣内 0 dB、主瓣外 -13 dB 的方向图）")


def print_table(rows: list[dict], args: argparse.Namespace) -> None:
    header = (f"{'R km':>8}{'SNR dB':>10}{'S/(N+J) dB':>12}"
              f"{'J/S':>10}{'Pd 无扰':>10}{'Pd 受扰':>10}")
    print("\n" + header)
    stride = max(1, len(rows) // 24)
    indexes = sorted(set(range(0, len(rows), stride)) | {len(rows) - 1})
    for i in indexes:
        row = rows[i]
        print(f"{row['r'] / 1000:>8.1f}{eq.db(row['snr']):>10.2f}"
              f"{eq.db(row['snr_j']):>12.2f}{row['js']:>10.3g}"
              f"{row['pd']:>10.4f}{row['pd_j']:>10.4f}")


def selftest(radar, rows: list[dict], args: argparse.Namespace,
             has_jam: bool) -> bool:
    ok = True

    # 体检 1：零干扰恒等——jammer_noise_w=0 必须与不给这个参数逐位相同。
    probes = (0.3, 0.6, 0.9, 1.2, 1.5)
    identity = all(
        radar.single_pulse_snr(f * args.detect_range, args.rcs)
        == radar.single_pulse_snr(f * args.detect_range, args.rcs,
                                  jammer_noise_w=0.0)
        for f in probes
    )
    if identity:
        print("[体检 1 绿] 零干扰恒等：J=0 与无干扰逐位相同（5 个探测点）")
    else:
        print("[体检 1 红] 零干扰恒等被破坏——噪声支的实现改坏了退化路径")
        ok = False

    if has_jam:
        pts = [(log10(row["r"]), log10(row["js"]))
               for row in rows[len(rows) // 6:] if row["js"] > 0.0]
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        slope = (sum((p[0] - mx) * (p[1] - my) for p in pts)
                 / sum((p[0] - mx) ** 2 for p in pts))
        expect = 2.0 if args.mode == "self" else 4.0
        if abs(slope - expect) > 0.05:
            print(f"[体检 2 红] J/S 对数斜率 {slope:.4f}，应为 {expect:.0f}"
                  f"（J/S ∝ R^{expect:.0f}）")
            ok = False
        else:
            print(f"[体检 2 绿] J/S 对数斜率 {slope:.4f} ≈ R^{expect:.0f}"
                  f"（{'自卫' if args.mode == 'self' else '支援'}干扰口径）")

    # 体检 3：标定锚点——标定法的存在意义就是这一条。
    if args.peak_power <= 0.0:
        anchor = radar.detection_probability(args.detect_range, args.rcs)
        if abs(anchor - args.required_pd) <= 1.0e-6:
            print(f"[体检 3 绿] 标定锚点：Pd({args.detect_range / 1000:g} km) = "
                  f"{anchor:.6f} = required_pd")
        else:
            print(f"[体检 3 红] 标定锚点漂了：Pd = {anchor:.6f}，"
                  f"required_pd = {args.required_pd:g}")
            ok = False
    return ok


def draw(rows: list[dict], radar, args: argparse.Namespace,
         burn: float | None, out: Path, has_jam: bool) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    km = [row["r"] / 1000.0 for row in rows]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10.0, 7.5), dpi=110,
                                   sharex=True)
    ax1.plot(km, [eq.db(row["snr"]) for row in rows],
             color="#185FA5", lw=2.0, label="干净单脉冲 SNR")
    if has_jam:
        ax1.plot(km, [eq.db(row["snr_j"]) for row in rows],
                 color="#A32D2D", lw=2.0, ls="--", label="受扰 S/(N+J)")
    if radar.snr_at_reference > 0.0:
        ax1.axhline(eq.db(radar.snr_at_reference), color="#888888",
                    ls=":", lw=1.2, label=f"Pd={args.required_pd:g} 的门限 SNR")
    if burn is not None:
        ax1.axvline(burn / 1000.0, color="#BA7517", ls="-.", lw=1.4,
                    label=f"烧穿 {burn / 1000:.1f} km")
    ax1.set_ylabel("SNR dB", fontsize=10)
    if args.mode == "standoff":
        note = "支援干扰：J 常数，J/S 随 R^4 涨"
    else:
        note = "自卫干扰：J 与 S 同随 R 变，J/S 随 R^2 涨"
    ax1.set_title("单脉冲 SNR（" + note + "）", fontsize=11)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8)

    ax2.plot(km, [row["pd"] for row in rows],
             color="#185FA5", lw=2.0, label="Pd 无干扰")
    if has_jam:
        ax2.plot(km, [row["pd_j"] for row in rows],
                 color="#A32D2D", lw=2.0, ls="--", label="Pd 受扰")
    ax2.axhline(args.required_pd, color="#888888", ls=":", lw=1.2,
                label=f"required_pd = {args.required_pd:g}")
    if burn is not None:
        ax2.axvline(burn / 1000.0, color="#BA7517", ls="-.", lw=1.4,
                    label=f"烧穿 {burn / 1000:.1f} km")
    ax2.set_xlabel("距离 km", fontsize=10)
    ax2.set_ylabel("检测概率", fontsize=10)
    ax2.set_ylim(-0.02, 1.05)
    ax2.set_title("检测概率（受扰支的 J 由组件链路自己算：名册-几何-F_BW-主副瓣-极化）",
                  fontsize=11)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="upper right")

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"图已写入 {out}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="雷达 + 干扰配合体检")
    parser.add_argument("--detect-range", type=float, default=30_000.0,
                        help="标称探测距离 m（默认 30000）")
    parser.add_argument("--rcs", type=float, default=5.0,
                        help="目标 RCS m2（默认 5）")
    parser.add_argument("--required-pd", type=float, default=0.5)
    parser.add_argument("--pfa", type=float, default=1.0e-6)
    parser.add_argument("--pulses", type=int, default=0,
                        help="积累脉冲数（0 = 按参数默认）")
    parser.add_argument("--frequency", type=float, default=3.0e9,
                        help="工作频率 Hz（默认 3 GHz）")
    parser.add_argument("--bandwidth", type=float, default=2.0e6,
                        help="接收带宽 Hz（默认 2 MHz）")
    parser.add_argument("--beam-width", type=float, default=2.0,
                        help="扫描波束宽度（主瓣判定用，默认 2 度）")
    parser.add_argument("--beamwidth-h", type=float, default=2.0,
                        help="方位波束宽度（增益换算用，默认 2 度）")
    parser.add_argument("--beamwidth-v", type=float, default=10.0,
                        help="俯仰波束宽度（默认 10 度）")
    parser.add_argument("--gain-db", type=float, default=0.0,
                        help="显式天线增益 dB（0 = 由波束宽度换算）")
    parser.add_argument("--system-loss", type=float, default=0.0)
    parser.add_argument("--receive-loss", type=float, default=0.0,
                        help="接收支路内损 dB（干扰链路唯一扣的）")
    parser.add_argument("--swerling", type=int, default=0, choices=(0, 1))
    parser.add_argument("--peak-power", type=float, default=0.0,
                        help="发射峰值功率 W（给了走逐参法，默认 0 = 标定法）")
    parser.add_argument("--noise-power", type=float, default=None,
                        help="标定法的接收噪声功率 W（默认 1e-12）")
    parser.add_argument("--mode", default="standoff", choices=("standoff", "self"),
                        help="standoff = 远距支援；self = 自卫（与目标同址）")
    parser.add_argument("--jam-power", type=float, default=200.0)
    parser.add_argument("--jam-gain-db", type=float, default=20.0)
    parser.add_argument("--jam-frequency", type=float, default=0.0,
                        help="干扰频率 Hz（0 = 跟雷达同频）")
    parser.add_argument("--jam-bw", type=float, default=0.0,
                        help="干扰带宽 Hz（0 = 连续波瞄准式）")
    parser.add_argument("--jam-range", type=float, default=50_000.0,
                        help="支援干扰机的距离 m（默认 50 km）")
    parser.add_argument("--jam-bearing", type=float, default=90.0,
                        help="支援干扰机的方位度（默认与目标同方位 = 主瓣）")
    parser.add_argument("--rmax", type=float, default=0.0,
                        help="扫描上界 m（0 = 1.6 倍 detect_range）")
    parser.add_argument("--no-jam", action="store_true", help="只测雷达")
    parser.add_argument("--no-selftest", action="store_true")
    parser.add_argument("--plot", default="", help="出图路径（不给就不画）")
    args = parser.parse_args(argv)

    radar, _store, jam, jam_frequency = build(args)
    rows = sweep(radar, _store, args, jam is not None)

    burn = None if jam is None else burnthrough_range(rows)
    report(radar, rows, args, jam_frequency, jam is not None)
    print_table(rows, args)
    if burn is not None:
        print(f"\n烧穿距离 R* = {burn / 1000:.2f} km"
              f"（J/S = 1：干扰功率 = 回波功率；此后信号赢了干扰）")
    elif jam is not None:
        if rows[-1]["js"] > 1.0:
            print("\n扫描范围内 J/S 始终 > 1（全程被压制）——调小 --jam-power/"
                  "--jam-gain-db，或加大 --jam-bw/--jam-range 再看")
        else:
            print("\n扫描范围内 J/S 始终 < 1（干扰赢不了）——加大 --jam-power/"
                  "--jam-gain-db，或把 --mode 换成 self 再看")

    if not args.no_selftest and not selftest(radar, rows, args,
                                             jam is not None):
        return 1
    if args.plot:
        draw(rows, radar, args, burn, Path(args.plot), jam is not None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
