"""雷达 + 干扰组件配合测试：组件级装配，扫距离画完整的功率账（§5.12/§5.14）。

不跑想定：直接把 :class:`~milsim.models.percep.search_radar.HexSearchRadar`
与一份**真的** :class:`~milsim.services.ew.JamService` 装配在一起（与
``tests/test_percep.py`` 的 ``_ew_build`` 同一套桩手法，但做成活的——目标与
干扰机的位置可以在扫描中随时挪），然后扫距离回答四个问题：

1. **干净 SNR 怎么落** —— ``single_pulse_snr(R, rcs)``，R⁻⁴；
2. **干扰进来压掉多少** —— ``single_pulse_snr(R, rcs, jammer_noise_w=J)``，
   其中 ``J = received_jamming_w(波束方位)`` 走名册 + 几何 + Jam.1 全链；
3. **烧穿在哪** —— ``J/S = 1/SINAD − 1/SNR``（代数恒等，不需要知道 N），
   ``J/S = 1`` 处就是烧穿距离；
4. **Pd 掉多少** —— ``detection_probability`` 干净 vs 受扰两条曲线。

两种干扰几何（``--mode``）：

* ``standoff``（默认）**远距支援干扰**：干扰机固定在 ``--jam-bearing`` /
  ``--jam-range``，J 是常数 ⇒ ``J/S ∝ R⁴``（对数图上 40 dB/十倍距）；
* ``self`` **自卫干扰**：干扰机与目标同址伴随，J ∝ R⁻² ⇒ ``J/S ∝ R²``
  （20 dB/十倍距）。两条斜率由 ``--selftest`` 实测把守。

用法::

    python tools/check_radar_jam.py                       # 默认：30 km 标定雷达 + 远距支援干扰
    python tools/check_radar_jam.py --mode self           # 自卫干扰：J/S ∝ R²
    python tools/check_radar_jam.py --no-jam              # 只测雷达
    python tools/check_radar_jam.py --peak-power 2.5e5    # 切逐参法（N 走温度链）
    python tools/check_radar_jam.py --jam-bw 0            # 连续波瞄准式（F_BW = 1）

★ **标定法接干扰必须给绝对噪声功率**（``--noise-power``，默认 1e-12 W）：
标定法把 N 折进"距离 ↔ Pd"曲线里，手上没有绝对的 N，``J/S`` 无从谈起
（``jamming_to_signal`` 当场报 ``ValueError``）。切逐参法（``--peak-power``）
时 N 由"天线温度 + 馈线 + 噪声系数"链算出，**不许**再给 ``--noise-power``。
"""

from __future__ import annotations

import argparse
import sys
from math import atan2, cos, degrees, log10, radians, sin
from pathlib import Path
from types import SimpleNamespace

_TOOLS_DIR = Path(__file__).resolve().parent
for _p in (str(_TOOLS_DIR), str(_TOOLS_DIR.parent / "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# 表格按显示宽度对齐（中文列头占两列）——复用 demo_mover 的实现，不另写一份。
from demo_mover import _pad  # noqa: E402

from milsim.models.percep import equation as eq  # noqa: E402
from milsim.models.percep.search_radar import HexSearchRadar  # noqa: E402
from milsim.services.ew import JamService, JammerSpec  # noqa: E402
from milsim.services.map import LocalFrame  # noqa: E402
from milsim.services.store import EntityStore, LocalCellRef  # noqa: E402

#: 网格边长（米）——只用来给实体一个名义格；位置的真出处是 ``set_pose``。
_SIZE = 1000.0

RADAR_ID = 0
TARGET_ID = 1
JAMMER_ID = 2

#: 目标方位（度，正北 0 顺时针）。波束每拍都指向它 ⇒ 干扰从"波束指着目标"
#: 这一时刻的增益进——与 AFSIM 求值时机原话一致。
TARGET_BEARING = 90.0


# ---------------------------------------------------------------------------
# 装配桩：与 tests/test_percep.py 的 _Spec/_Engine/_Mount 同一套手法
# ---------------------------------------------------------------------------

class _Spec(dict):
    slot = "sensor"
    type_name = "HEX_SEARCH_RADAR"


class _Engine:
    def __init__(self) -> None:
        self.now = 0

    def schedule_recurring(self, delay, interval, fn, priority=0):
        return None


class _Mount:
    """雷达真正用到的那几个成员。``nav=None`` ⇒ 不判几何（缺数据 ≠ 被遮挡）。"""

    def __init__(self, engine, store, jam) -> None:
        self.engine = engine
        self.store = store
        self.entity_id = RADAR_ID
        self.now = engine.now
        self.nav = None
        self._streams_seed = 0
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
        import random

        return SimpleNamespace(sensor=random.Random(0))

    def target_platform_param(self, entity_id, name, default=None):
        return default


class _Sides:
    """阵营判定桩：雷达 red、目标与干扰机 blue ⇒ 不会被"同阵营不打"筛掉。"""

    def __init__(self) -> None:
        self._sides = {RADAR_ID: "red", TARGET_ID: "blue", JAMMER_ID: "blue"}

    def side_of(self, entity_id: int) -> str:
        return self._sides.get(entity_id, "")

    def by_name(self, name: str):
        return None          # 本工具不用 aim_at 按名解析，恒 None 即可


def bearing_to(x: float, y: float) -> float:
    """从原点看 ``(x, y)`` 的方位角（与组件的 ``atan2(dx, −dy)`` 同一口径）。"""
    return degrees(atan2(x, -y)) % 360.0


def add_entity(store: EntityStore, frame: LocalFrame, entity_id: int,
               x: float, y: float) -> None:
    """首次登记实体：名义格与 pose 同源（``add`` 只能调一次，之后只挪 pose）。"""
    cell = frame.from_world(x, y)
    store.add(entity_id, LocalCellRef(1, 0, cell))
    store.set_pose(entity_id, x, y, 0.0)


def move_jammer(store: EntityStore, args: argparse.Namespace,
                target_range_m: float) -> tuple[float, float]:
    """按模式放干扰机（只挪 pose，不重复 add），返回 ``(x, y)``。

    ``self`` 模式每个扫描点都会调一次——干扰机与目标同址伴随，J 随 R⁻² 走；
    ``standoff`` 模式位置固定，只在装配时调一次。
    """
    if args.mode == "self":
        r = target_range_m
        b = radians(TARGET_BEARING)
    else:
        r = args.jam_range
        b = radians(args.jam_bearing)
    x, y = r * sin(b), -r * cos(b)
    store.set_pose(JAMMER_ID, x, y, 0.0)
    return x, y


def build_radar(args: argparse.Namespace):
    """装一部雷达 + 一份活的电子战门面，返回 ``(radar, store, jam_frequency)``。"""
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
        swerling_case=args.swerling,
    ))
    if args.peak_power > 0.0:
        values["peak_power"] = args.peak_power
        if args.noise_power is not None:
            raise SystemExit(
                "逐参法（--peak-power）不许再给 --noise-power：那时 N 由"
                "天线温度 + 馈线 + 噪声系数链算出，再给一个就是同一个量的"
                "第二个出处。要加噪声余量请调 --system-loss 或给 --gain-db"
            )
    else:
        # 标定法必须给绝对噪声功率（J/S 要绝对的 N），默认 kB·T·B（290 K、
        # 2 MHz）量级的工程口径 1e-12 W；用户显式给的就照写。
        values["noise_power"] = (args.noise_power if args.noise_power is not None
                                 else 1.0e-12)

    jam_frequency = args.jam_frequency if args.jam_frequency > 0.0 else args.frequency

    store = EntityStore()
    frame = LocalFrame(0.0, 0.0, _SIZE)
    add_entity(store, frame, RADAR_ID, 0.0, 0.0)
    add_entity(store, frame, TARGET_ID, 1000.0, 0.0)

    jam = JamService(store, _Sides())
    if not args.no_jam:
        # 首放：standoff 在指定方位/距离；self 与目标初位同址（扫描时逐点挪）。
        # ★ 先 add 再 set_pose——store 对没登记的实体挪位置当场 KeyError。
        if args.mode == "self":
            r_jam, b_jam = 1000.0, radians(TARGET_BEARING)
        else:
            r_jam, b_jam = args.jam_range, radians(args.jam_bearing)
        add_entity(store, frame, JAMMER_ID,
                   r_jam * sin(b_jam), -r_jam * cos(b_jam))
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
    radar.initialize(_Mount(_Engine(), store, jam))
    return radar, store, jam_frequency


def sweep(radar, store: EntityStore, args: argparse.Namespace) -> list[dict]:
    """扫距离：每点挪目标（与 self 模式的干扰机），算完整的功率账。"""
    rcs = args.rcs
    r_max = args.rmax if args.rmax > 0.0 else 1.6 * args.detect_range
    rows: list[dict] = []
    n = 241
    for i in range(n):
        r = r_max * (i + 1) / n            # 从 r_max/n 起步，避开 R→0 的 inf
        x = r * sin(radians(TARGET_BEARING))
        store.set_pose(TARGET_ID, x, 0.0, 0.0)
        if not args.no_jam and args.mode == "self":
            move_jammer(store, args, r)

        snr = radar.single_pulse_snr(r, rcs)
        row = {"r": r, "snr": snr}
        if args.no_jam:
            row.update(j=0.0, sinad=snr, js=0.0,
                       pd_clean=radar.detection_probability(r, rcs), pd_jam=None)
        else:
            # 波束指着目标那一拍，干扰从该方位漏进来的功率（名册+几何+Jam.1 全链）。
            j_w = radar.received_jamming_w(TARGET_BEARING)
            sinad = radar.single_pulse_snr(r, rcs, jammer_noise_w=j_w)
            # J/S = 1/SINAD − 1/SNR：代数恒等（SINAD = SNR/(1+J/N)），
            # 全程不需要知道绝对的 N——标定法下也成立。
            js = (1.0 / sinad - 1.0 / snr) if snr > 0.0 and sinad > 0.0 else 0.0
            row.update(j=j_w, sinad=sinad, js=max(js, 0.0),
                       pd_clean=radar.detection_probability(r, rcs),
                       pd_jam=radar.detection_probability(r, rcs, jammer_noise_w=j_w))
        rows.append(row)
    return rows


def burnthrough_range(rows: list[dict]) -> float | None:
    """J/S 穿越 1 的距离（线性插值）。J/S 单调增（R⁴ 或 R²）⇒ 交点唯一。"""
    js = [row["js"] for row in rows]
    for i in range(len(rows) - 1):
        if js[i] < 1.0 <= js[i + 1]:
            t = (1.0 - js[i]) / (js[i + 1] - js[i])
            return rows[i]["r"] + t * (rows[i + 1]["r"] - rows[i]["r"])
    return None


def selftest(radar, rows: list[dict], args: argparse.Namespace) -> bool:
    """三条数学体检，红任何一条都 exit 1：

    1. **零干扰恒等**：``jammer_noise_w=0`` 时受扰 SNR 必须逐位等于干净值
       （``snr_under_jamming`` 在 J/S=0 处是恒等映射，不是"相差极小"）；
    2. **J/S 斜率**：standoff 的 J ∝ R⁻²、S ∝ R⁻⁴ ⇒ J/S ∝ R⁴；self 的
       J、S 同址 ⇒ J/S ∝ R²。对数斜率实测，偏差 >0.05 即红；
    3. **标定法锚点**：``Pd(detect_range, reference_rcs) == required_pd``
       是标定法的定义本身。
    """
    ok = True
    probe = rows[len(rows) // 3]
    identical = radar.single_pulse_snr(probe["r"], args.rcs)
    if identical != probe["snr"]:
        print(f"[体检 1 红] 零干扰恒等被破坏：{identical!r} != {probe['snr']!r}")
        ok = False
    else:
        print("[体检 1 绿] 零干扰恒等：jammer_noise_w=0 逐位等于干净 SNR")

    if not args.no_jam:
        pts = [(log10(row["r"]), log10(row["js"]))
               for row in rows[len(rows) // 6:] if row["js"] > 0.0]
        if not pts:
            print("[体检 2 灰] 到达干扰功率为 0（频带完全错开），斜率无从量起")
        else:
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

    if radar.range_model == "标定法":
        pd = radar.detection_probability(radar.detect_range_m, args.rcs)
        if abs(pd - args.required_pd) > 1.0e-6:
            print(f"[体检 3 红] 标定法锚点失守：Pd({args.detect_range/1000:.0f} km)"
                  f" = {pd:.8f} ≠ required_pd {args.required_pd}")
            ok = False
        else:
            print(f"[体检 3 绿] 标定法锚点：Pd({args.detect_range/1000:.0f} km, "
                  f"RCS {args.rcs:g} m²) = {pd:.6f} = required_pd")
    else:
        print("[体检 3 灰] 逐参法没有 Pd 锚点（探测距离由方程自己算）")
    return ok


def print_report(radar, rows: list[dict], args: argparse.Namespace,
                 jam_frequency: float) -> None:
    """头部信息 + 均匀采样表。"""
    model = radar.range_model
    print(f"== 雷达（{model}）")
    print(f"  探测距离 {args.detect_range/1000:.1f} km @ Pd {args.required_pd:.2f}"
          f"（参考 RCS {args.rcs:g} m²）；N={radar.pulses}、Pfa {args.pfa:g}、"
          f"Swerling {radar.swerling_case}")
    if model == "标定法":
        print(f"  标定锚点 snr_at_reference = {10*log10(radar.snr_at_reference):.3f} dB；"
              f"噪声功率 N = {radar.noise_w:.3e} W（{eq.dbm(radar.noise_w):.1f} dBm）")
    else:
        print(f"  峰值功率 {radar.power_w/1000:.1f} kW、G {10*log10(radar.gain):.1f} dB、"
              f"λ {radar.wavelength_m*100:.2f} cm；N = {radar.noise_w:.3e} W"
              f"（{eq.dbm(radar.noise_w):.1f} dBm，温度链）")
    if args.no_jam:
        print("== 干扰：--no-jam，只测雷达")
        return

    j0 = max((row["j"] for row in rows), default=0.0)
    print(f"== 干扰机（{args.mode}）")
    if args.mode == "standoff":
        print(f"  方位 {args.jam_bearing:g}°、距离 {args.jam_range/1000:.0f} km"
              f"（波束指目标 {TARGET_BEARING:g}° ⇒ 与干扰机夹 "
              f"{abs(args.jam_bearing - TARGET_BEARING):g}°）")
    else:
        print("  与目标同址伴随（自卫干扰），波束指目标 ⇒ 干扰走主瓣")
    if args.jam_bw <= 0.0:
        fbw_note = "瞄准式（连续波）F_BW = 1"
    else:
        ratio = eq.bandwidth_overlap_ratio(
            jam_frequency, args.jam_bw, args.frequency, args.bandwidth)
        fbw_note = f"阻塞式 F_BW ≈ {ratio:.3f}"
    print(f"  功率 {args.jam_power:g} W × 增益 {args.jam_gain_db:g} dB；"
          f"频率 {jam_frequency/1e9:.2f} GHz、带宽 {args.jam_bw/1e6:.1f} MHz"
          f"（{fbw_note}）")
    if j0 <= 0.0:
        print("  到达功率 J = 0（频带完全错开或没登记上）——受扰曲线与干净重合")
        return
    print(f"  到达功率 J = {j0:.3e} W（{eq.dbm(j0):.1f} dBm）；"
          f"J/N = {j0/radar.noise_w:.3g}（{10*log10(j0/radar.noise_w):.1f} dB）")
    if args.mode == "standoff" and j0 > 0.0:
        # 对照：波束若正对干扰机，走主瓣增益——同一个干扰机，差一个副瓣抑制。
        j_main = radar.received_jamming_w(args.jam_bearing)
        print(f"  对照：波束正对干扰机（主瓣进）J = {j_main:.3e} W"
              f" —— 副瓣抑制了 {10*log10(j_main/j0):.1f} dB")

    print()
    head = (f"{_pad('R km', 7, right=True)}{_pad('SNR dB', 9, right=True)}")
    if not args.no_jam:
        head += (_pad('S/(N+J) dB', 11, right=True) + _pad('压掉 dB', 9, right=True)
                 + _pad('J/S dB', 9, right=True))
    head += _pad('Pd 净', 8, right=True)
    if not args.no_jam:
        head += _pad('Pd 扰', 8, right=True)
    print(head)
    stride = max(1, len(rows) // 24)
    sampled = rows[::stride]
    if sampled[-1] is not rows[-1]:
        sampled.append(rows[-1])
    for row in sampled:
        line = f"{row['r']/1000:>7.2f}{10*log10(row['snr']):>9.2f}"
        if not args.no_jam:
            drop = 10*log10(row['snr']) - 10*log10(row['sinad'])
            line += (f"{10*log10(row['sinad']):>11.2f}{drop:>9.2f}"
                     f"{10*log10(row['js']):>9.2f}" if row['js'] > 0.0
                     else f"{'—':>11}{'—':>9}{'—':>9}")
        line += f"{row['pd_clean']:>8.4f}"
        if not args.no_jam:
            line += "     —" if row["pd_jam"] is None else f"{row['pd_jam']:>8.4f}"
        print(line)


def draw(rows: list[dict], radar, args: argparse.Namespace,
         burn: float | None, out: Path) -> None:
    """双面板：上 = SNR/SINAD/J-S 曲线 + 烧穿线；下 = Pd 两条曲线。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    r_km = [row["r"] / 1000.0 for row in rows]
    snr_db = [10 * log10(row["snr"]) for row in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.5, 8.0), dpi=110,
                                   sharex=True)

    ax1.plot(r_km, snr_db, color="#2c6fbb", lw=1.8, label="SNR（干净）")
    if not args.no_jam:
        ax1.plot(r_km, [10 * log10(row["sinad"]) for row in rows],
                 color="#c0392b", lw=1.8, label="S/(N+J)（受扰）")
        ax3 = ax1.twinx()
        ax3.plot(r_km, [10 * log10(row["js"]) for row in rows],
                 color="#d9822b", lw=1.2, ls=":", label="J/S")
        ax3.axhline(0.0, color="#d9822b", lw=0.8, ls=":", alpha=0.6)
        ax3.set_ylabel("J/S dB", fontsize=9, color="#d9822b")
        ax3.tick_params(labelsize=8, colors="#d9822b")
        if burn is not None:
            ax1.axvline(burn / 1000.0, color="#c0392b", ls="--", lw=1.2)
            ax1.annotate(f"烧穿 {burn/1000:.1f} km（J/S=1）",
                         (burn / 1000.0, ax1.get_ylim()[1]),
                         textcoords="offset points", xytext=(6, -14),
                         fontsize=9, color="#c0392b")
    ax1.axvline(args.detect_range / 1000.0, color="#888888", ls=":", lw=1.2,
                label=f"探测距离 {args.detect_range/1000:.0f} km")
    ax1.set_ylabel("单脉冲 SNR dB", fontsize=9)
    ax1.set_title(f"雷达功率账（{radar.range_model}，探测距离 "
                  f"{args.detect_range/1000:.0f} km @ Pd {args.required_pd:.2f}、"
                  f"RCS {args.rcs:g} m²）\n"
                  + ("远距支援干扰：J 常数 → J/S ∝ R⁴"
                     if args.mode == "standoff" and not args.no_jam else
                     "自卫干扰：J ∝ R⁻² → J/S ∝ R²" if not args.no_jam else
                     "无干扰基线"), fontsize=10)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="upper right")
    ax1.tick_params(labelsize=8)

    ax2.plot(r_km, [row["pd_clean"] for row in rows], color="#2c6fbb", lw=1.8,
             label="Pd（干净）")
    if not args.no_jam:
        ax2.plot(r_km, [row["pd_jam"] for row in rows], color="#c0392b", lw=1.8,
                 label="Pd（受扰）")
    ax2.axhline(args.required_pd, color="#888888", ls=":", lw=1.0,
                label=f"required_pd {args.required_pd:.2f}")
    if burn is not None:
        ax2.axvline(burn / 1000.0, color="#c0392b", ls="--", lw=1.2)
    ax2.set_xlabel("目标距离 km", fontsize=9)
    ax2.set_ylabel("单次扫描检测概率", fontsize=9)
    ax2.set_ylim(-0.02, 1.02)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="upper right")
    ax2.tick_params(labelsize=8)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"\n图已写入 {out}")


def run(args: argparse.Namespace) -> int:
    radar, store, jam_frequency = build_radar(args)
    rows = sweep(radar, store, args)

    burn = None if args.no_jam else burnthrough_range(rows)
    print_report(radar, rows, args, jam_frequency)
    if not args.no_jam:
        j_max = max((row["j"] for row in rows), default=0.0)
        if j_max <= 0.0:
            print("\n到达干扰功率为 0（频带完全错开或没登记上）——受扰曲线应与"
                  "干净重合；想让它进得来就核对 --jam-bw / --jam-frequency")
        elif burn is not None:
            print(f"\n烧穿距离 R* = {burn/1000:.2f} km"
                  f"（J/S = 1：干扰功率 = 回波功率；比它近信号赢，比它远干扰赢）")
        else:
            print("\n扫描范围内 J/S 始终 > 1（全程被压制）——调小 --jam-power /"
                  " --jam-gain-db 或加大 --jam-bw / --jam-range 再看")

    if not args.no_selftest and not selftest(radar, rows, args):
        return 1
    if args.plot and args.plot != "none":
        draw(rows, radar, args, burn, Path(args.plot))
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="雷达 + 干扰组件配合测试（功率账 / 烧穿 / Pd）")
    # -- 雷达 ---------------------------------------------------------------
    parser.add_argument("--detect-range", type=float, default=30_000.0,
                        help="探测距离 m（默认 30000）")
    parser.add_argument("--rcs", type=float, default=5.0,
                        help="目标 RCS m²（默认 5）")
    parser.add_argument("--required-pd", type=float, default=0.5,
                        help="标定法 Pd 目标（默认 0.5）")
    parser.add_argument("--pfa", type=float, default=1.0e-6, help="虚警概率")
    parser.add_argument("--pulses", type=int, default=16,
                        help="非相干积累脉冲数（默认 16）")
    parser.add_argument("--swerling", type=int, default=0,
                        help="起伏模型：0 非起伏（默认）、1 Swerling 1")
    parser.add_argument("--frequency", type=float, default=4.0e9,
                        help="工作频率 Hz（默认 4 GHz）")
    parser.add_argument("--bandwidth", type=float, default=2.0e6,
                        help="接收机带宽 Hz（默认 2 MHz；F_BW 分母）")
    parser.add_argument("--gain-db", type=float, default=0.0,
                        help="逐参法天线峰值增益 dB（默认 0 = 由波束宽度换算；"
                             "标定法不读它）")
    parser.add_argument("--beamwidth-h", type=float, default=2.0,
                        help="方位波束宽度°（默认 2；标定法接干扰的接收增益"
                             "只能由 G=4π/(θH·θV) 换算，没有它 J 算不出来）")
    parser.add_argument("--beamwidth-v", type=float, default=10.0,
                        help="俯仰波束宽度°（默认 10）")
    parser.add_argument("--beam-width", type=float, default=2.0,
                        help="瞬时波束张角°（默认 2；主瓣/副瓣判定的主瓣宽，"
                             "0 = 各向同性 ⇒ 干扰永远按副瓣 −13 dB 进）")
    parser.add_argument("--system-loss", type=float, default=0.0,
                        help="双程总损耗 dB（默认 0）")
    parser.add_argument("--peak-power", type=float, default=0.0,
                        help="给 >0 切逐参法（N 走温度链）；默认 0 = 标定法")
    parser.add_argument("--noise-power", type=float, default=None,
                        help="标定法绝对噪声功率 W（默认 1e-12）；逐参法不许给")
    # -- 干扰 ---------------------------------------------------------------
    parser.add_argument("--mode", choices=("standoff", "self"), default="standoff",
                        help="standoff=远距支援（默认）；self=自卫（与目标同址）")
    parser.add_argument("--no-jam", action="store_true", help="只测雷达")
    parser.add_argument("--jam-power", type=float, default=2000.0,
                        help="干扰机功率 W（默认 2000）")
    parser.add_argument("--jam-gain-db", type=float, default=20.0,
                        help="干扰机增益 dB（默认 20）")
    parser.add_argument("--jam-frequency", type=float, default=0.0,
                        help="干扰机频率 Hz（默认 0 = 跟雷达同频）")
    parser.add_argument("--jam-bw", type=float, default=50.0e6,
                        help="干扰带宽 Hz（默认 50 MHz 阻塞式；0 = 连续波瞄准）")
    parser.add_argument("--jam-range", type=float, default=120_000.0,
                        help="standoff 干扰机距离 m（默认 120 km；self 忽略）")
    parser.add_argument("--jam-bearing", type=float, default=200.0,
                        help="standoff 干扰机方位°（默认 200；self 忽略）")
    # -- 输出 ---------------------------------------------------------------
    parser.add_argument("--rmax", type=float, default=0.0,
                        help="扫描上限 m（默认 1.6 × 探测距离）")
    parser.add_argument("--plot", default="check_radar_jam.png",
                        help="输出图路径；传 none 关闭")
    parser.add_argument("--no-selftest", action="store_true",
                        help="跳过三条数学体检")
    return run(parser.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
