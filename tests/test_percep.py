"""感知件（探测链第 4~8 步）的测试。

分三层，每层都能单独核对：

1. :mod:`milsim.models.percep.equation` —— **纯函数**，只 import ``math``。
   与三路独立来源对齐：规格书快查表、N=1 的闭式解、教科书的总 SNR 表；
2. 探测链（几何 / 功率 / 信噪比 / 检测概率 / 建航）—— 用桩 mount 把它从
   引擎与地图里摘出来，量纲与参数解析由 ``test_params`` / ``test_type_registry``
   覆盖，端到端由 ``python tools/run_scenario.py scenarios/patrol.txt`` 覆盖；
3. ``check_line_of_sight`` 的 ``terrain_floor`` 口径 —— 水域通道存的是海床。

★ 本文件里有两条**守门人**用例，各盯住一个曾经静默通过的错：

* :func:`test_swerling1_does_not_saturate_on_huge_snr` —— ``j > 200_000``
  的兜底截断让 4 km 处的 Pd 从 1.0 变成 0.0486（v0.13.25 修）；
* :func:`test_water_is_sea_level_not_seabed` —— 把海床当地表会让海面上的
  舰船被自己脚下"地形"挡住，而报出来的理由是假的。
"""

from __future__ import annotations

import random

import pytest

from milsim.errors import ConfigurationError
from milsim.models.jam import RfJammer
from milsim.models.percep import equation as eq
from milsim.models.percep import search_radar as percep_radar
from milsim.models.percep import track_manager as percep_track
from milsim.services.ew import (
    DEFAULT_SIDELOBE_LEVEL_DB,
    DeceptionSpec,
    FalseTargetSpec,
    JamService,
    JammerSpec,
)
from milsim.services.map import (
    Axial,
    HexGrid,
    LocalFrame,
    check_line_of_sight,
    generate_flat,
)
from milsim.services.store import (
    IFF_FOE,
    IFF_FRIEND,
    MAX_RELAY_HOPS,
    ORIGIN_LOCAL,
    TRACK_REPORT,
    EntityStore,
    LocalCellRef,
    phantom_id,
)

HexSearchRadar = percep_radar.HexSearchRadar
TrackManager = percep_track.TrackManager

#: 网格边长（米）。与 ``test_demo_components`` 同一套坐标约定。
_SIZE = 1000.0

#: 桩 mount 报出的相邻格心距 = 边长 × √3（pointy-top）。
_SPAN_M = _SIZE * 3 ** 0.5

#: 轴向环 6 上的六个格心正好落在 30° 的整数倍上（见 test_demo_components）。
_EAST = (6, 0)          # 90°
_WEST = (-6, 0)         # 270°

#: "一圈 1 秒"的 ``scan_interval``（微秒）——1 s 一拍 ⇒ Δ = 360°，**每拍都照到**
#: 同一个目标。M/N 建航的用例要靠它把"被照射"变成逐拍发生的事，否则判据里
#: 混进扫描周期，读起来就不是在测 M/N 了。
_REV_PER_BEAT = 1_000_000

#: 几何判决要用的**架高**（米）。★ 凡是要测几何的用例都必须显式给一个。
#: 因为 ``antenna_height = 0`` 表示"**没有架高这一项** ⇒ 不判几何"，不是
#: "天线贴在地面上"：贴地天线对同样贴地的目标，含地球曲率的余隙在任意距离上
#: 恒为负 ⇒ 一部不写架高的雷达**一条航迹都建不起来**（`library_demo.txt`
#: 实测过）。不显式给架高的话，几何那几条用例测的就是"跳过"而不是"判决"，
#: 而且**照样是绿的**——这正是要防的那种"绿着但没测到东西"。
_MAST_M = 10.0

#: **"一次命中即建航"**（v0.13.39）。
#:
#: 生产默认自 v0.13.39 起是 ``3/5 建、2/5 维持``（雷达常规：建航要严、维持要松）。
#: 但**测几何 / 量测误差 / 欺骗 / 幽灵的那一族用例量的是那件事本身，不是 M/N**：
#: 跑一拍就要问"航迹表里有没有它"，拿生产默认的话每一拍都在等 5 取 3，
#: 量到的立刻从"几何判决"变成"建航律"——两件事搅在一起，坏了也看不出是哪件。
#:
#: ⇒ 这类用例**显式**带上它。**刻意不写进 ``_build``/``_params`` 的默认值**：
#: 那等于让测试自己藏起产品口径，产品默认改了这里也不会响——而"改默认值要
#: 惊动谁"这件事本身是有信息量的，不该被测试悄悄盖掉。**测 M/N 本身的用例**
#: （如 ``test_track_waits_for_enough_hits_to_establish``）当然不写它，它们
#: 要的正是把 ``3/5`` 显式钉住。
_INSTANT_TRACK = {
    "hits_to_establish": 1,
    "establish_window": 1,
    "hits_to_maintain": 1,
    "maintain_window": 1,
}


# ---------------------------------------------------------------------------
# 桩：引擎 / 存储 / 装配上下文
# ---------------------------------------------------------------------------


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
        self.beat[1](self, None)


class _Streams:
    """``mount.streams()`` 的桩：``sensor`` 是**属性**（与 ``EntityStreams`` 同形）。"""

    def __init__(self, seed: int) -> None:
        self._random = random.Random(seed)

    @property
    def sensor(self):
        return self._random


class _Mount:
    """只留雷达真正用到的那几个成员。

    ``nav`` 默认 ``None``，那就是"没有逐格精度"那条路径——几何判定不参与，
    与 ``cell_span_m() == 0`` 是同一条约定。通视本身另有桩 nav 的用例。

    ``jam`` 默认 ``None``，那是"这份装配没有电子战"——与真装配里
    ``MountContext.jam = None`` 同义。给一个真
    :class:`~milsim.services.ew.JamService` 就能测压制（§5.14）。
    """

    def __init__(
        self, engine, store, entity_id, *, nav=None, rcs=None, seed=0, jam=None
    ):
        self.engine = engine
        self.store = store
        self.entity_id = entity_id
        self.now = engine.now
        self.nav = nav
        self._rcs = rcs
        self._streams = _Streams(seed)
        self.jam = jam

    def sensor_view(self):
        return self.store.sensor_view(self.entity_id)

    def jam_service(self):
        return self.jam

    def cell_span_m(self):
        return _SPAN_M

    def every(self, interval_us, fn, priority=0):
        return self.engine.schedule_recurring(0, interval_us, fn, priority)

    def streams(self):
        return self._streams

    def target_platform_param(self, entity_id, name, default=None):
        if name == "radar_cross_section" and self._rcs is not None:
            return self._rcs
        return default


class _ForcedPd(HexSearchRadar):
    """检测概率被钉住的雷达——第 8 步的用例只测**建航逻辑**，不测方程。

    方程是另一批用例的事；把它们搅在一起会让"建航判据错了"与"Pd 算错了"
    表现成同一个现象。
    """

    forced_pd = 1.0

    def detection_probability(self, range_m, rcs_m2=None, *, jammer_noise_w=0.0):
        return self.forced_pd


def _params(**overrides):
    values = {name: item.default for name, item in HexSearchRadar.PARAMS.items()}
    values.update(overrides)
    return _Spec(values)


def _build(
    cls=HexSearchRadar,
    *,
    entities=(),
    nav=None,
    target_rcs=None,
    seed=0,
    **params,
):
    engine = _Engine()
    store = EntityStore()
    frame = LocalFrame(0.0, 0.0, _SIZE)
    store.add(0, LocalCellRef(1, 0, Axial(0, 0)))
    for entity_id, col, row in entities:
        cell = Axial(col, row)
        store.add(entity_id, LocalCellRef(1, 0, cell))
        x, y = frame.to_world(cell)
        store.set_pose(entity_id, x, y, 0.0)

    radar = cls(spec=_params(**params))
    radar.initialize(_Mount(engine, store, 0, nav=nav, rcs=target_rcs, seed=seed))
    return radar, engine, store


def _run(engine, seconds: float, step_s: float = 1.0) -> None:
    """从 t=0 敲到 ``seconds``（两端都含）。

    ★ **只能用来"从头跑一段"**：它总是从 0 开始，续时间轴会把时钟倒回去
    （负的 dt 会被组件挡掉，但正的那几拍会**再照一遍**）。要接着往下走就
    直接 ``engine.tick_to(...)``。
    """
    beat = 0
    while beat * step_s <= seconds + 1e-9:
        engine.tick_to(beat * step_s)
        beat += 1


class _BlockingNav:
    """桩导航门面：通视一律判否，并且**记下被问过几次**。"""

    def __init__(self, visible: bool = False) -> None:
        self.visible = visible
        self.asked = 0

    def line_of_sight(self, ref, observer_xy, target_xy, observer_z, target_z, **kw):
        self.asked += 1
        return self.visible


# ===========================================================================
# 第一层：equation（纯函数）
# ===========================================================================


def test_peak_gain_matches_the_spec_quick_table() -> None:
    """``G = 4π/(θ_H·θ_V)``，两个角都取**弧度**。

    规格书快查表 7.2 给的是整档工程值（1°×3° ⇒ 40 dBi、2°×10° ⇒ 33、
    5°×20° ⇒ 27），本式给 41.4 / 33.1 / 26.2 dBi——差 ≤1.4 dB，方向一致。
    按**度数**直算同样两个角会得到 −15.6 dBi（荒谬），所以这条用例真正守的是
    "有没有换成弧度"。
    """
    assert 10 * __import__("math").log10(eq.peak_gain(2.0, 10.0)) == pytest.approx(33.1, abs=0.2)
    assert 10 * __import__("math").log10(eq.peak_gain(1.0, 3.0)) == pytest.approx(41.4, abs=0.2)
    assert 10 * __import__("math").log10(eq.peak_gain(5.0, 20.0)) == pytest.approx(26.2, abs=0.2)


def test_received_power_falls_off_as_the_fourth_power() -> None:
    near = eq.received_power(1.0e5, eq.linear(30.0), eq.wavelength(4.0e9), 1.0, 10_000.0, 10.0)
    far = eq.received_power(1.0e5, eq.linear(30.0), eq.wavelength(4.0e9), 1.0, 20_000.0, 10.0)
    assert near / far == pytest.approx(16.0)          # 距离翻倍 ⇒ 1/16
    assert eq.received_power(1.0e5, 1.0, 0.075, 0.0, 10_000.0) == 0.0   # 零 RCS 无回波


def test_thermal_noise_is_k_times_times_b() -> None:
    assert eq.thermal_noise(20_000.0, 290.0) == pytest.approx(
        eq.BOLTZMANN * 290.0 * 20_000.0
    )
    with pytest.raises(ValueError):
        eq.thermal_noise(0.0, 290.0)


def test_false_alarm_threshold_halves_its_argument() -> None:
    """N=1 时门限 = 2·(−ln Pfa)。

    尾概率写成 ``e^{-u}Σ_{k<N}u^k/k!`` 时那个 ``u`` 是 ``V/2``；漏掉 2 倍
    会让门限偏低 3 dB（N=1 时 Pd 从 0.036 变成 0.19）。
    """
    import math

    assert eq.false_alarm_threshold(1, 1.0e-6) == pytest.approx(-2.0 * math.log(1.0e-6))
    assert eq.false_alarm_threshold(1, 1.0e-6) == pytest.approx(27.631, abs=0.01)


def test_swerling1_single_pulse_matches_the_closed_form() -> None:
    """N=1、平方律检波、Swerling 1 有闭式 ``Pd = Pfa^(1/(1+S))``。

    第三路（蒙特卡洛 30 万次）在 `tools/verify_radar_spec.py` 里，不塞进单元
    测试——单元测试不该跑几秒的随机数。
    """
    for s_db in (0.0, 6.0, 12.0, 20.0, 30.0):
        s = eq.linear(s_db)
        assert eq.swerling1_pd(s, 1, 1.0e-6) == pytest.approx(1.0e-6 ** (1.0 / (1.0 + s)))


def test_swerling1_does_not_saturate_on_huge_snr() -> None:
    """★ 守门人：``N·S`` 很大时 Pd 必须仍然趋近 1。

    上一版用 ``j`` 的双重和，而 ``j`` 的衰减尺度是 ``1 + N·S``；实现里有个
    ``j > 200_000`` 的兜底截断，于是近程强信号（4 km 处 ``N·S ≈ 4×10⁶``）被
    悄悄截成 ``200000/(1+N·S) = 0.0486``——**一个看起来很像概率的数**，
    航迹表照常写，所以一直没露头。收缩掉外层几何和后与 ``N·S`` 无关。

    ``N·S = 4.012×10⁶`` 处的真值不是"机器精度的 1"：``1 − Pd = 1.158×10⁻⁵``。
    这个尾巴**不是截断误差**（把停止上界加大到十万项也不变），而是
    ``β = N·S/(1+N·S)`` 与 1 的差被那个尺度的项数放大的结果；独立路径
    （按 Swerling 1 的物理模型直接抽：Rayleigh 幅度在驻留期内恒定 +
    非中心 χ² 平方律非相干积累）在 5 组参数上给到 ≤1.3e-4 的一致度。
    所以断言写成 ``1e-4``，而**不是** ``1e-9``。
    """
    assert eq.swerling1_pd(62_690.0, 64, 1.0e-6) == pytest.approx(1.0, abs=1e-4)
    assert eq.swerling1_pd(1.0e6, 64, 1.0e-6) == pytest.approx(1.0, abs=1e-5)
    # 同一组参数在"修前"会返回 0.04862 —— 这条断言就是不许它退回去
    assert eq.swerling1_pd(62_690.0, 64, 1.0e-6) > 0.99

    # 大 N·S 段的尾巴按 1/(N·S) 走：能量再放大 10 倍，漏检率就降 10 倍。
    near = 1.0 - eq.swerling1_pd(62_690.0, 64, 1.0e-6)
    far = 1.0 - eq.swerling1_pd(626_900.0, 64, 1.0e-6)
    assert near / far == pytest.approx(10.0, rel=1e-3)


def test_swerling1_needs_more_total_energy_as_pulses_grow() -> None:
    """★ 反直觉判据：Swerling 1 的**总** SNR 随 N **上升**。

    慢起伏目标几乎吃不到非相干积累的增益，所以同样的 Pd 要更多总能量
    （Pd=0.5、Pfa=1e-6：N=1 要 12.77 dB、N=100 要 18.96 dB）。
    **如果实现出来发现它随 N 下降，说明算成了 Swerling 2。**

    ★ v0.13.40：``snr_for_pd`` 默认改成反解**非起伏**曲线，所以这条**必须**
    显式写 ``nonfluctuating=False``——它测的是 Swerling 1。反解与正算用的是
    同一条曲线（``swerling1_pd``），所以判据本身没变。
    """
    import math

    totals = [
        10 * math.log10(n * eq.snr_for_pd(0.5, n, 1.0e-6, nonfluctuating=False))
        for n in (1, 8, 64, 100)
    ]
    assert totals == sorted(totals)
    assert totals[0] == pytest.approx(12.77, abs=0.02)
    assert totals[-1] == pytest.approx(18.96, abs=0.02)


def test_nonfluctuating_pd_single_pulse_matches_monte_carlo() -> None:
    """★ N=1 的非起伏 Pd 与 30 万次蒙特卡洛对齐（`tools/verify_radar_spec.py`）。

    `Pfa=1e-6`、`N=1` 时门限 ``V = −2·ln(Pfa) = 27.631``（见
    :func:`false_alarm_threshold` 那条测试），Pd 就是 Marcum Q
    ``Q_1(√(2S), √V)``。四个锚点是硬判据：

    * 0 dB  → 0.00012（MC 0.000122）
    * 1 dB  → 0.00023（MC 0.000229）
    * 10 dB → 0.24805（MC 0.24811）
    * 12.7721 dB → 0.83775（MC 0.83822）

    这几条同时**钉死两件事**：门限没算错、Marcum Q 的数值没崩。
    """
    assert eq.nonfluctuating_pd(eq.linear(0.0), 1, 1.0e-6) == pytest.approx(
        0.000122, abs=1e-5
    )
    assert eq.nonfluctuating_pd(eq.linear(1.0), 1, 1.0e-6) == pytest.approx(
        0.000229, abs=1e-5
    )
    assert eq.nonfluctuating_pd(eq.linear(10.0), 1, 1.0e-6) == pytest.approx(
        0.248049, abs=5e-4
    )
    assert eq.nonfluctuating_pd(eq.linear(12.7721), 1, 1.0e-6) == pytest.approx(
        0.837747, abs=5e-4
    )


def test_nonfluctuating_pd_never_exceeds_one_on_huge_snr() -> None:
    """★ 守门人：`a ≫ b`（高 SNR）时 Marcum Q 必须**趋近 1**，不能返回 0。

    这是本项目踩得最深的一个数值坑：Marcum Q 的"级数形式"
    ``e^{−(a²+b²)/2}·Σ_k (a/b)^k·I_k(ab)`` 在 `a ≫ b`（恰好是高 SNR 工作区）
    收敛极慢，且 ``(a/b)^k`` 指数发散、``I_k(ab)`` 在 `k ≳ ab` 后正向递推失真
    ——**实测在 SNR=30 dB 会给回 0**（真值 1）。修法是改用 Poisson 权重 ×
    中心 χ² 上尾的混合表示（见 ``_marcum_q`` 的 docstring）。

    所以这条断言不许它退回任何"级数在尾部崩掉"的实现：N=1/16/256 三档、
    20/30/60 dB 三个 SNR，全部必须 >0.999。
    """
    for n in (1, 16, 256):
        for s_db in (20.0, 30.0, 60.0):
            pd = eq.nonfluctuating_pd(eq.linear(s_db), n, 1.0e-6)
            assert pd > 0.999, f"N={n} {s_db} dB -> {pd}"


def test_nonfluctuating_and_swerling1_curves_cross_over() -> None:
    """★ 判"两个口径谁更保守"的物理判据：**两条曲线交叉，不是单向的**。

    `Pfa=1e-6`、`N=1` 实测（见 ``nonfluctuating_pd`` 的 docstring 表）：

    * **低 SNR 段**（0/6/10 dB）：Pd 对 SNR 是**凸**的 ⇒ 对指数分布求平均
      （Swerling 1）反而抬高 Pd，"偶发强回波"贡献了大部分检测概率
      ⇒ **Swerling 1 > 非起伏**；
    * **高 SNR 段**（12.7721/15/20 dB）：Pd 已饱和、变**凹** ⇒ 平均把弱回波的
      漏检摊进来 ⇒ **非起伏 > Swerling 1**。

    这条断言的价值在于：**任何"单向"的说法都是错的**。当初把默认口径从
    Swerling 1 换成固定 RCS 时，一句"Swerling 1 永远 ≤ 非起伏"就写反了低 SNR
    那一段——所以这里正反两段都钉住，不许退回单向叙述。
    """
    for s_db in (0.0, 6.0, 10.0):
        s = eq.linear(s_db)
        nf = eq.nonfluctuating_pd(s, 1, 1.0e-6)
        sw = eq.swerling1_pd(s, 1, 1.0e-6)
        assert sw > nf, f"{s_db} dB（低 SNR）：应 Swerling1 > 非起伏，实为 {sw} vs {nf}"

    for s_db in (12.7721, 15.0, 20.0):
        s = eq.linear(s_db)
        nf = eq.nonfluctuating_pd(s, 1, 1.0e-6)
        sw = eq.swerling1_pd(s, 1, 1.0e-6)
        assert nf > sw, f"{s_db} dB（高 SNR）：应 非起伏 > Swerling1，实为 {nf} vs {sw}"


def test_snr_for_pd_defaults_to_nonfluctuating_and_inverts_it() -> None:
    """★ v0.13.40：``snr_for_pd`` 默认反解**非起伏**曲线，且与正算自洽。

    重新钉一遍那个移动过的锚点（`Pd=0.5`、``Pfa=1e-6``）：
    非起伏 **11.2426 dB**、Swerling 1 **12.7719 dB**（差 **1.5293 dB**）。
    标定法"反解 ⇒ 正算"要逐字回到 0.5——`Pd(8000 m)=0.5000000002` 那条
    想定锚点就是它的外部见证。
    """
    a = eq.snr_for_pd(0.5, 1, 1.0e-6)  # 默认 = 非起伏
    b = eq.snr_for_pd(0.5, 1, 1.0e-6, nonfluctuating=True)
    c = eq.snr_for_pd(0.5, 1, 1.0e-6, nonfluctuating=False)

    assert a == pytest.approx(b)                      # 默认就是 True
    assert eq.db(a) == pytest.approx(11.2426, abs=1e-4)
    assert eq.db(c) == pytest.approx(12.7719, abs=1e-4)
    assert eq.db(c) - eq.db(a) == pytest.approx(1.5293, abs=1e-4)

    # 反解回算必须回到 0.5（用同一条曲线）
    assert eq.nonfluctuating_pd(a, 1, 1.0e-6) == pytest.approx(0.5, abs=1e-8)
    assert eq.swerling1_pd(c, 1, 1.0e-6) == pytest.approx(0.5, abs=1e-8)


def test_nonfluctuating_pd_rejects_bad_inputs() -> None:
    """值域守门：脉冲数 < 1、Pfa 不在 (0,1) 都要报错，不许静默给个数。"""
    import pytest as _pytest

    with _pytest.raises(ValueError):
        eq.nonfluctuating_pd(1.0, 0, 1.0e-6)
    with _pytest.raises(ValueError):
        eq.nonfluctuating_pd(1.0, 1, 0.0)
    with _pytest.raises(ValueError):
        eq.nonfluctuating_pd(1.0, 1, 1.0)

    # snr <= 0 的退化：Pd = Pfa（"只有虚警"）
    assert eq.nonfluctuating_pd(0.0, 1, 1.0e-6) == pytest.approx(1.0e-6)
    assert eq.nonfluctuating_pd(-5.0, 1, 1.0e-6) == pytest.approx(1.0e-6)


def test_dwell_time_and_pulses_in_dwell() -> None:
    """驻留时间与引擎步长无关；一个驻留期至少算一次检测机会。"""
    assert eq.dwell_time(2.0, 36.0) == pytest.approx(2.0 / 36.0)
    with pytest.raises(ValueError):
        eq.dwell_time(2.0, 0.0)
    assert eq.pulses_in_dwell(0.0, 1000.0) == 1        # 连一个脉冲都放不下
    assert eq.pulses_in_dwell(0.02, 1000.0) == 20
    assert eq.pulses_in_dwell(1.0, 0.0) == 1           # 没给 PRF


# ===========================================================================
# 第二层：探测链
# ===========================================================================


def test_calibrated_path_hits_the_target_pd_at_detect_range() -> None:
    """标定法的定义：``detect_range`` 处 Pd 恰好等于 ``required_pd``。

    两个数据表数字（探测距离、转速）就能给出一条**有物理形状**的曲线，
    这是让已落库的 6 型雷达（只有这两个真参数）能用上方程的关键。
    """
    radar, _, _ = _build(detect_range=30_000.0, required_pd=0.5)
    assert radar.range_model == "标定法"
    assert radar.detection_probability(30_000.0) == pytest.approx(0.5, abs=1e-9)
    assert radar.detection_probability(15_000.0) > 0.9
    assert radar.detection_probability(100_000.0) < 0.01


def test_calibrated_path_is_monotone_in_range() -> None:
    radar, _, _ = _build(detect_range=30_000.0)
    values = [radar.detection_probability(r) for r in range(1_000, 80_000, 1_000)]
    assert values == sorted(values, reverse=True)


def test_target_rcs_scales_the_calibrated_curve() -> None:
    """标定法下 RCS 的相对关系仍然生效——``σ/σ_ref`` 就是雷达方程的线性部分。"""
    radar, _, _ = _build(detect_range=30_000.0, reference_rcs=1.0)
    small = radar.single_pulse_snr(30_000.0, 1.0)
    large = radar.single_pulse_snr(30_000.0, 4.0)
    assert large / small == pytest.approx(4.0)


def test_explicit_path_reproduces_the_radar_equation() -> None:
    """逐参法：单脉冲 SNR 与手算的 ``Pt·G²·λ²·σ /((4π)³R⁴L·kTB)`` 一致。

    参数照抄 AFSIM 的 ``base_types/sensors/radar/pd_radar.txt``。
    """
    import math

    radar, _, _ = _build(
        peak_power=100.0e3,
        frequency=4.0e9,
        antenna_gain_db=30.0,
        system_loss_db=10.0,
        noise_figure_db=3.0,
        bandwidth=20_000.0,
        pulses_integrated=64,
        detect_range=200_000.0,
        max_range=200_000.0,
    )
    assert radar.range_model == "逐参法"
    assert radar.pulses == 64
    assert radar.wavelength_m == pytest.approx(eq.C_LIGHT / 4.0e9)

    noise = eq.thermal_noise(20_000.0, eq.STANDARD_TEMPERATURE * eq.linear(3.0))
    power = eq.received_power(
        100.0e3, eq.linear(30.0), eq.C_LIGHT / 4.0e9, 1.0, 64_000.0, eq.linear(10.0)
    )
    assert radar.single_pulse_snr(64_000.0) == pytest.approx(power / noise)
    # 64 km 处单脉冲 SNR = 10.5599（10.2366 dB）。AFSIM 在同一点报 12.04 dB
    # （它的 signal_to_noise 是**积累后**口径），差 1.80 dB —— 口径表见 §5.12.8，
    # 这个差**不是**要拿调参去凑的数。
    assert radar.single_pulse_snr(64_000.0) == pytest.approx(10.5599, abs=1e-3)
    assert 10 * math.log10(radar.single_pulse_snr(64_000.0)) == pytest.approx(10.2366, abs=1e-3)
    assert noise == pytest.approx(1.597759e-16, rel=1e-5)


def test_cascade_noise_temperature_reduces_to_the_blake_steps() -> None:
    """级联式 = AFSIM 三步展开的等价化简；默认值退化成 ``T_s = T0·F``。

    AFSIM 的算法（``docs/receiver.rst`` 的 *Receiver Noise*，引 Blake 1986
    *Radar Range Performance* 第 4 章）：

    .. code-block:: text

        T_a = T0 + (0.876·Tant − 254)/L_ohmic
        T_l = T0·(L_r − 1)
        T_r = T0·(F − 1)
        T_s = T_a + T_l + L_r·T_r

    把 ``T_l + L_r·T_r`` 展开，那一对 ``±T0·L_r`` 抵消，剩
    ``T0·(L_r·F − 1) + T_a``。**这一条守的是"化简没抄错"**：两边各自
    算出同一个 ``T_s``。
    """
    t0 = eq.STANDARD_TEMPERATURE
    lr = eq.linear(2.0)
    f = eq.linear(3.0)
    ta = 42.8036

    # ``T_a`` 直接给，等价于 AFSIM 内部按 Tant 与 antenna_ohmic_loss 算出的那一项
    expanded = ta + t0 * (lr - 1.0) + lr * t0 * (f - 1.0)
    assert eq.cascade_noise_temperature(ta, lr, f) == pytest.approx(expanded, abs=1e-9)

    # 默认（馈线 0 dB、T_a = T0）⇒ 退化成 T0·F，即 v0.13.25 的口径
    assert eq.cascade_noise_temperature(t0, 1.0, f) == pytest.approx(t0 * f)

    with pytest.raises(ValueError):
        eq.cascade_noise_temperature(0.0, lr, f)
    with pytest.raises(ValueError):
        eq.cascade_noise_temperature(ta, 0.5, f)          # 线性损耗不得小于 1
    with pytest.raises(ValueError):
        eq.cascade_noise_temperature(ta, lr, 0.5)         # 噪声系数不得小于 1


def test_receive_line_loss_only_enters_the_noise_branch() -> None:
    """馈线损耗**不衰减信号**，只抬高噪声温度（AFSIM 的 ``receive_line_loss`` 口径）。

    这是本轮对齐 AFSIM 的另一半：它的 ``signal_to_noise`` 里没有馈线，馈线
    只出现在级联噪声温度上。判据用**比值**而不是绝对值——绝对值里混着信号
    与噪声两件事，分开对才说得清是哪一支在动。
    """
    common = dict(
        peak_power=100.0e3,
        frequency=4.0e9,
        antenna_gain_db=30.0,
        system_loss_db=8.0,
        noise_figure_db=3.0,
        antenna_noise_temperature=290.0,
        bandwidth=20_000.0,
        detect_range=200_000.0,
    )
    clean, _, _ = _build(receive_line_loss_db=0.0, **common)
    wired, _, _ = _build(receive_line_loss_db=2.0, **common)

    # 信号路径一模一样 —— 馈线不在 ``loss`` 里
    assert wired.loss == clean.loss
    # 噪声只因级联式里多了一段馈线温度而变高
    ratio = (
        eq.cascade_noise_temperature(290.0, eq.linear(2.0), eq.linear(3.0))
        / eq.cascade_noise_temperature(290.0, 1.0, eq.linear(3.0))
    )
    assert wired.noise_w / clean.noise_w == pytest.approx(ratio)
    # 于是 SNR 按噪声比值的倒数下降，**不是**按损耗再降一次
    assert wired.single_pulse_snr(64_000.0) == pytest.approx(
        clean.single_pulse_snr(64_000.0) / ratio
    )


def test_afsim_radar_parameters_reproduce_its_noise_power() -> None:
    """★ 守门人：照 AFSIM 的 ``pd_radar.txt`` 填参数，噪声功率逐位命中。

    那只雷达（``demos/base_types/sensors/radar/pd_radar.txt``）写着
    ``noise_figure 3 db``、``receive_line_loss 2 db``、``transmitter internal_loss 2 db``
    与 ``receiver internal_loss 6 db``；它的 ``horizontal_map`` 逐点输出里
    ``noise_power`` 恒为 **−157.329 dBW**。

    三条口径各就各位之后才命中：

    * ``system_loss_db = 8``（= tx 2 + rx 6，**不含**馈线）；
    * ``receive_line_loss_db = 2``（只进噪声）；
    * ``antenna_noise_temperature = 42.8036``（从它的 ``noise_power`` 反推）。

    ★ 后两个是**一对**：只给馈线而把温度留在室温会跑出 917.06 K（−155.965 dBW），
    比不给还偏 1.365 dB。这也解释了为什么 AFSIM 文档里"给了 ``noise_figure``
    而两个损耗项都没给"要单列一条支路。
    """
    import math

    radar, _, _ = _build(
        peak_power=100.0e3,
        frequency=4.0e9,
        antenna_gain_db=30.0,
        system_loss_db=8.0,
        noise_figure_db=3.0,
        receive_line_loss_db=2.0,
        antenna_noise_temperature=42.8036,
        bandwidth=20_000.0,
        detect_range=200_000.0,
    )
    assert 10 * math.log10(radar.noise_w) == pytest.approx(-157.329, abs=5e-4)

    # 反面对照：T_a 留在默认 290 K ⇒ 噪声高出 1.365 dB（差的那一项就是它）
    loose, _, _ = _build(
        peak_power=100.0e3,
        frequency=4.0e9,
        antenna_gain_db=30.0,
        system_loss_db=8.0,
        noise_figure_db=3.0,
        receive_line_loss_db=2.0,
        bandwidth=20_000.0,
        detect_range=200_000.0,
    )
    assert 10 * math.log10(loose.noise_w / radar.noise_w) == pytest.approx(1.3646, abs=1e-3)


def test_receiver_only_parameters_also_count_as_the_explicit_path() -> None:
    """只写噪声系数而不写发射机参数也要**报错**，不许静默退回标定法。

    标定法**根本不用噪声**，所以"我明明填了噪声系数"会一点没生效，而数字
    看着仍然合理——与"弹的推重比 < 1 跑出假航迹"是同一类静默退化。
    """
    with pytest.raises(ConfigurationError) as info:
        _build(noise_figure_db=3.0)
    assert "peak_power" in str(info.value)
    assert "逐参法" in str(info.value)


def test_gain_can_come_from_two_beamwidths_instead() -> None:
    radar, _, _ = _build(
        peak_power=100.0e3,
        frequency=4.0e9,
        beamwidth_h=2.0,
        beamwidth_v=10.0,
        bandwidth=20_000.0,
        detect_range=200_000.0,
    )
    assert radar.range_model == "逐参法"
    assert radar.gain == pytest.approx(eq.peak_gain(2.0, 10.0))


def test_bandwidth_falls_back_to_one_over_pulse_width() -> None:
    radar, _, _ = _build(
        peak_power=100.0e3,
        frequency=4.0e9,
        antenna_gain_db=30.0,
        pulse_width=10.0,                 # 微秒 ⇒ B = 1/τ = 100 kHz
        detect_range=200_000.0,
    )
    assert radar.noise_w == pytest.approx(
        eq.thermal_noise(100_000.0, eq.STANDARD_TEMPERATURE)
    )


def test_partial_transmitter_config_is_an_error() -> None:
    """半给必须**当场报错**，不许静默退回标定法。

    静默退回的后果是"我明明填了功率"却一点没生效，而数字看着仍然合理——
    与"弹的推重比 < 1 跑出假航迹"是同一类。
    """
    with pytest.raises(ConfigurationError) as info:
        _build(peak_power=100.0e3)
    assert "frequency" in str(info.value)
    assert "逐参法" in str(info.value)

    with pytest.raises(ConfigurationError) as info:
        _build(peak_power=100.0e3, frequency=4.0e9)       # 缺增益
    assert "antenna_gain_db" in str(info.value)

    with pytest.raises(ConfigurationError) as info:
        _build(peak_power=100.0e3, frequency=4.0e9, antenna_gain_db=30.0)   # 缺带宽
    assert "bandwidth" in str(info.value)


def test_range_beyond_the_search_bound_is_not_illuminated() -> None:
    """``detect_range`` 在标定法里只是**标定锚点**；搜索上界看 ``max_range``。

    上界没抬起来时，超出标称距离的目标**连第 4~7 步都不进**——这一条把
    "搜索范围"与"检测概率曲线"两件事分开，免得读成"Pd 在标称距离处被硬截断"。

    判据用几何判决被问过几次：距离门排在它前面，所以"没问过"就说明在那个门
    上被拦下了。只有 ``contacts_seen`` 是空的不能说明问题——**没扫到**、**超出
    上界**、**几何遮挡**都会让它为空，而这三件事的修法完全不同。

    目标在六边距离 6 的那一环上（约 10.39 km）；上界 10 km 时索引半径算出来是
    6 ⇒ **它确实是候选**，不是"索引根本没查到它"。
    """
    _ForcedPd.forced_pd = 1.0
    nav = _BlockingNav(visible=True)
    radar, engine, _ = _build(
        cls=_ForcedPd,
        detect_range=10_000.0,
        entities=((1, *_EAST),),
        nav=nav,
        antenna_height=_MAST_M,
        **_INSTANT_TRACK,
    )
    engine.tick_to(1.0)
    assert nav.asked == 0, "超出搜索上界的候选不该进几何判决"
    assert radar.last_pd == 0.0
    assert radar.contacts_seen == []

    wide_nav = _BlockingNav(visible=True)
    wide, engine2, _ = _build(
        cls=_ForcedPd,
        detect_range=10_000.0,
        max_range=20_000.0,
        entities=((1, *_EAST),),
        nav=wide_nav,
        antenna_height=_MAST_M,
        **_INSTANT_TRACK,
    )
    engine2.tick_to(1.0)
    assert wide_nav.asked == 1, "上界抬起来之后同一个目标就该被判"
    assert wide.contacts_seen == [1]


def test_geometry_blocking_counts_as_a_miss() -> None:
    """★ 几何遮挡是**漏检**，不是"没扫到"。

    波束确实照到了它，只是什么也没收到——所以它要进 M/N 的历史（记一次未
    命中）。把它当"没扫到"会让躲在山后的目标永远不进建航判据的分母，
    于是它一出山就立刻建航。

    一拍 1 s、一圈 2 s ⇒ 每拍 180°，正东的目标只落在**奇数拍**的弧
    ``[0°, 180°)`` 里（偶数拍是 ``[180°, 360°)``）——所以两次照射之间要隔一拍。

    ★ 几何判决**只在给了架高时才跑**（见 :meth:`_has_line_of_sight`），所以
    这条用例必须显式写 ``antenna_height=_MAST_M``；不写的话门面根本不会被
    碰到、``nav.asked`` 恒为 0。
    """
    nav = _BlockingNav(visible=False)
    _ForcedPd.forced_pd = 1.0
    radar, engine, _ = _build(
        cls=_ForcedPd,
        entities=((1, *_EAST),),
        nav=nav,
        antenna_height=_MAST_M,
        **_INSTANT_TRACK,
    )

    engine.tick_to(1.0)                              # 第 1 次被照射
    assert nav.asked == 1
    assert radar.geometry_blocked == 1
    assert radar.contacts_seen == []                 # 挡住 ⇒ 还差一次
    assert list(radar._hits[1]) == [False]           # 但历史里记下了这一笔

    nav.visible = True
    engine.tick_to(3.0)                              # 第 2 次被照射
    assert radar.contacts_seen == [1]


def test_geometry_is_skipped_when_there_is_no_grid() -> None:
    """没有逐格精度时**不判**几何——数据缺失不能被伪装成"被遮挡"。"""
    radar, engine, _ = _build(entities=((1, *_EAST),), **_INSTANT_TRACK)
    engine.tick_to(2.0)
    assert radar.geometry_blocked == 0
    assert radar.contacts_seen == [1]


def test_geometry_is_skipped_without_a_mast_height() -> None:
    """★ ``antenna_height = 0`` ⇒ **不问几何**（连门面都不碰）。

    这条是本项目踩过的坑：`scenarios/library_demo.txt` 里的雷达来自库里
    的真实型号表，**表里没有"天线架高"这一项**；若把 ``0`` 读成"天线贴在
    地面上"，含地球曲率的余隙在任意距离上恒为负 ⇒ **一条航迹都建不起来**，
    而进程退出码 0、输出看着正常。

    所以 ``0`` 的语义定成"没有这一项 ⇒ 不判"，与 ``nav is None``（没有地形
    数据）走同一条口子。判据用 ``nav.asked``：只要被问过一次就说明没走这条
    口子。这里故意给一个**通视一律判否**的桩门面，好让"判了会怎样"显形。
    """
    nav = _BlockingNav(visible=False)
    radar, engine, _ = _build(
        entities=((1, *_EAST),), nav=nav, **_INSTANT_TRACK
    )
    engine.tick_to(2.0)
    assert nav.asked == 0, "没写架高就不该去问几何判决"
    assert radar.geometry_blocked == 0
    assert radar.contacts_seen == [1]


def test_quality_is_the_detection_probability() -> None:
    """``quality`` 从"1 − 距离/探测距离"改成 **Pd**。

    旧的量是纯几何衰减，与功率、RCS、积累脉冲数都无关；新的量才是"这条
    航迹有多可信"的直接读数。这是一处**有意的行为变更**，不是回归。
    """
    _ForcedPd.forced_pd = 0.4
    radar, engine, _ = _build(cls=_ForcedPd, entities=((1, *_EAST),), seed=7)
    _run(engine, 40.0)
    contacts = radar.view.my_contacts()
    assert contacts, "40 拍、Pd=0.4，一次都没命中说明采样没接上"
    assert all(c.quality == pytest.approx(0.4) for c in contacts)
    assert radar.last_pd == pytest.approx(0.4)


def test_detection_sampling_is_reproducible() -> None:
    """检测抽样走 ``mount.streams().sensor``：同种子 ⇒ 逐字相同的航迹。"""
    _ForcedPd.forced_pd = 0.4
    first, engine_a, _ = _build(
        cls=_ForcedPd, entities=((1, *_EAST),), seed=2026, **_INSTANT_TRACK
    )
    _run(engine_a, 60.0)
    second, engine_b, _ = _build(
        cls=_ForcedPd, entities=((1, *_EAST),), seed=2026, **_INSTANT_TRACK
    )
    _run(engine_b, 60.0)
    assert first.contacts_seen == second.contacts_seen
    assert 0 < len(first.contacts_seen) < 60          # 确实是随机而不是恒真


# ===========================================================================
# 第二层（续）：第 8 步 M/N 建航
# ===========================================================================


def test_track_waits_for_enough_hits_to_establish() -> None:
    """``3/5``：最近 5 次被照射里命中 3 次才建航。

    一圈 1 s、一拍 1 s ⇒ 每拍都照到（不然"最近 5 次被照射"要跨 10 拍，
    判据的行数会跟着扫描周期漂，用例读起来就不是在测 M/N 了）。
    """
    _ForcedPd.forced_pd = 0.0
    radar, engine, _ = _build(
        cls=_ForcedPd,
        entities=((1, *_EAST),),
        scan_interval=_REV_PER_BEAT,
        hits_to_establish=3,
        establish_window=5,
    )
    _run(engine, 5.0)                                 # 5 次被照射，一次不中
    assert radar.contacts_seen == []
    assert list(radar._hits[1]) == [False] * 5        # 窗口封顶 5

    _ForcedPd.forced_pd = 1.0
    engine.tick_to(6.0)
    assert radar.contacts_seen == []                  # 4 次里只有 1 次命中
    engine.tick_to(7.0)
    assert radar.contacts_seen == []                  # 2 次
    engine.tick_to(8.0)
    assert radar.contacts_seen == [1], "最近 5 次里攒够 3 次命中就该建航"


def test_track_is_dropped_when_hits_stop() -> None:
    """``1/3``：最近 3 次被照射里一次都没命中 ⇒ 撤航（并把航迹删掉）。

    建航与撤航用的是**不同**的窗口：建航 ``3/5`` 要严（压住虚警），
    维持 ``1/3`` 要松（别把已经跟上的目标轻易丢掉）。
    """
    _ForcedPd.forced_pd = 1.0
    radar, engine, _ = _build(
        cls=_ForcedPd,
        entities=((1, *_EAST),),
        scan_interval=_REV_PER_BEAT,
        hits_to_establish=3,
        establish_window=5,
        hits_to_maintain=1,
        maintain_window=3,
    )
    _run(engine, 3.0)
    assert radar.contacts_seen == [1], "3 次全中就该建航"
    assert radar.view.my_contacts()

    # ★ 后面这几拍不能用 ``_run``——它**从 t=0 敲起**，会把时钟倒回去再照一遍
    #   （那几拍是命中，航迹又会被建起来）。续时间轴要用 ``tick_to``。
    _ForcedPd.forced_pd = 0.0
    for beat in (4.0, 5.0, 6.0):
        engine.tick_to(beat)
    assert radar.view.my_contacts() == [], "连续 3 次被照射都没命中就该撤航"
    assert radar.contacts_seen == [1], "撤的是航迹，不是探测历史"
    assert 1 in radar._hits, "目标还在，建航状态要留着——它可能再被照到"


def test_only_illuminated_sweeps_enter_the_window() -> None:
    """窗口的分母是"被照射"，不是"每一拍"。

    天线一圈 4 s、1 s 一拍 ⇒ 每拍 90°；正东的目标只在 ``[90°,180°)`` 那一拍
    被照到，所以 4 拍（一整圈）里窗口只该长 1。
    把"波束不在那边"也算成未命中，会让高转速雷达的建航判据随扫描周期漂移。
    """
    _ForcedPd.forced_pd = 1.0
    radar, engine, _ = _build(
        cls=_ForcedPd,
        entities=((1, *_EAST),),
        scan_interval=4_000_000,
    )
    _run(engine, 4.0)                                 # 一整圈
    assert len(radar._hits[1]) == 1


def test_a_dead_target_forgets_its_track_state() -> None:
    """目标没了就把建航状态清掉——不清的话实体 ID 被复用时，
    新目标一出生就"已经建航"，而且不产生任何日志。"""
    _ForcedPd.forced_pd = 1.0
    radar, engine, store = _build(cls=_ForcedPd, entities=((1, *_EAST),))
    _run(engine, 6.0)
    assert radar.view.my_contacts()

    store.remove(1)
    for beat in (7.0, 8.0, 9.0):
        engine.tick_to(beat)
    assert 1 not in radar._hits
    assert radar.view.my_contacts() == []


# ===========================================================================
# 第三层：通视的地表口径
# ===========================================================================


def _water_grid() -> HexGrid:
    """一块"海床"：整片 −375 m，中间隆起一格 −100 m。"""
    grid = HexGrid(zone_id=1, layer=0, size=_SIZE)
    generate_flat(grid, -8, -8, 16, 16)
    from milsim.services.map import offset_to_axial

    for col in range(-2, 13):
        for row in range(-3, 4):
            grid.set_elevation(offset_to_axial(col, row), -375.0)
    grid.set_elevation(offset_to_axial(5, 0), -100.0)
    return grid


def test_water_is_sea_level_not_seabed() -> None:
    """★ 守门人：水域通道存的是**海床**，不是海面。

    ``check_line_of_sight`` 默认把网格高程当地表——陆地上对，水面上错：一条
    z = 0 的舰船会算出"离地 −375 m"，被自己脚下的地形挡住，**而报出来的
    理由是假的**。``terrain_floor=0.0`` 把水面按海平面算，视线立刻通。
    """
    from milsim.services.map import offset_to_axial

    grid = _water_grid()
    start, goal = offset_to_axial(0, 0), offset_to_axial(10, 0)

    seabed = check_line_of_sight(grid, start, goal, 10.0, 10.0)
    assert not seabed.visible
    assert seabed.min_clearance < 0

    sea_level = check_line_of_sight(grid, start, goal, 10.0, 10.0, terrain_floor=0.0)
    assert sea_level.visible
    assert sea_level.min_clearance > 0


def test_terrain_floor_does_not_lift_land() -> None:
    """下限只抬"低于下限"的地形：山还是山，不会被抚平。"""
    from milsim.services.map import offset_to_axial

    grid = HexGrid(zone_id=1, layer=0, size=_SIZE)
    generate_flat(grid, -8, -8, 16, 16)
    for row in range(-5, 6):
        grid.set_elevation(offset_to_axial(5, row), 500.0)

    result = check_line_of_sight(
        grid,
        offset_to_axial(0, 0),
        offset_to_axial(10, 0),
        2.0,
        2.0,
        terrain_floor=0.0,
    )
    assert not result.visible
    assert result.blocking_elevation == pytest.approx(500.0)


# ===========================================================================
# 第四层：量测误差（第 7b 步）—— 位置从"真值"变成"量测"
# ===========================================================================

#: 逐参法的收发预算：100 kW 峰值、4 GHz、30 dB 增益、1 MHz 带宽、
#: 3 dB 噪声系数、脉冲宽 1 µs（⇒ 距离分辨单元 ``c·τ/2 = 149.896 m``）。
#: ★ **故意不给波束宽度**：给了增益就不需要它，而"缺波束宽度"正好是
#: :func:`test_snr_path_requires_a_beamwidth` 要触发的那条报错。
_BUDGET = dict(
    peak_power=1.0e5,
    frequency=4.0e9,
    antenna_gain_db=30.0,
    bandwidth=1.0e6,
    noise_figure_db=3.0,
    pulse_width=1.0,
)


def test_angle_sigma_matches_the_beamwidth_over_ten_rule() -> None:
    """``σ_θ = θ_3dB/(k_m·√(2·SNR))``，``k_m = 1.6``。

    ★ 这条守的是**两条独立来源的交点**，所以那个 1.6 不是"调出来的"：
    教科书（Barton, *Radar Equations for Modern Radar*）给 ``k_m ∈ [1.5, 1.8]``；
    AFSIM 自带的 ``demos/air_to_air/sensors/radar/aesa.txt`` 里写着
    ``azimuth_error_sigma 0.3 deg // beamwidth / 10``，即 ``σ_θ = θ/10``。
    取 ``k_m = 1.6`` 时，SNR = 13 dB（正是那份文件 ``detection_probability``
    表里 Pd≈0.95 的那一档）给出 ``θ/10.1`` —— 两条来源对上了。
    """
    snr_13db = eq.linear(13.0)
    sigma = eq.angle_measurement_sigma(3.0, snr_13db)
    assert sigma == pytest.approx(0.2968, abs=0.002)
    assert sigma / 3.0 == pytest.approx(0.1, rel=0.05)
    # 波束越窄、SNR 越高 ⇒ 越准（这两条是量纲方向，与具体系数无关）
    assert eq.angle_measurement_sigma(1.0, snr_13db) < sigma
    assert eq.angle_measurement_sigma(3.0, eq.linear(23.0)) < sigma
    # 没有回波就没有角度可言
    assert eq.angle_measurement_sigma(3.0, 0.0) == float("inf")
    with pytest.raises(ValueError):
        eq.angle_measurement_sigma(0.0, snr_13db)


def test_range_sigma_is_the_resolution_cell_over_root_two_snr() -> None:
    """``σ_R = c·τ/(2·√(2·SNR))``：``c·τ/2`` 是脉冲的**距离分辨单元**。

    τ = 1 µs ⇒ 分辨单元 149.896 m；SNR = 13 dB 时再除以 ``√(2·19.95) = 6.317``
    ⇒ 23.73 m。与角度那条是**同一个自变量**（SNR），这正是不该把它写成
    "Pd 决定精度"的原因。
    """
    cell = eq.C_LIGHT * 1.0e-6 / 2.0
    assert cell == pytest.approx(149.896, abs=0.01)
    assert eq.range_measurement_sigma(1.0e-6, 0.5) == pytest.approx(cell)
    assert eq.range_measurement_sigma(1.0e-6, eq.linear(13.0)) == pytest.approx(
        23.73, abs=0.05
    )


def test_measurement_errors_are_off_by_default() -> None:
    """★ 守门人：默认（两个 σ 都是 0）⇒ 航迹位置**逐字等于**目标真实位置。

    AFSIM 的 ``override_measurement_with_truth`` 默认 false —— 它默认**加**
    误差、并把带误差的位置写进航迹。我们默认不加，等于把那个**测试开关**
    当成了默认值。所以必须有这条用例把"默认 = 真值"钉死：日后谁把默认值
    一改，全世界的对照数字都会跟着漂，**而漂了也没人知道**。
    """
    radar, engine, store = _build(
        entities=[(1, 6, 0)], **_BUDGET, **_INSTANT_TRACK
    )
    _run(engine, 2.0)
    contact = store.contacts_of(0)[0]
    truth = store.position_of(1)
    assert contact.x == pytest.approx(truth[0])
    assert contact.y == pytest.approx(truth[1])
    assert contact.z == pytest.approx(truth[2])


def test_direct_range_sigma_moves_the_range_but_not_the_bearing() -> None:
    """直接给 ``range_error_sigma`` ⇒ **方位不动、距离抖动**。

    误差加在**极坐标**上（方位靠天线、距离靠时延，两套完全不同的机制），
    所以距离误差不会污染方位。若改成在 x/y 上各加一个同方差噪声，这条
    就守不住了——那正是"切向大、径向小"这个各向异性被抹掉的样子。
    """
    radar, engine, store = _build(
        entities=[(1, 6, 0)], range_error_sigma=100.0, seed=11,
        **_BUDGET, **_INSTANT_TRACK,
    )
    _run(engine, 2.0)
    contact = store.contacts_of(0)[0]
    me, truth = store.position_of(0), store.position_of(1)
    dx, dy = truth[0] - me[0], truth[1] - me[1]
    true_range = (dx * dx + dy * dy) ** 0.5

    assert contact.bearing_deg == pytest.approx(90.0)
    assert abs(contact.range_m - true_range) < 500.0      # 100 m σ 的 5 倍
    # 位置被推离了真值，但仍在**同一方位**上（正东 ⇒ y 不变）
    assert contact.x == pytest.approx(truth[0], abs=500.0)
    assert contact.y == pytest.approx(truth[1], abs=1e-6)


def test_two_error_paths_are_mutually_exclusive() -> None:
    """两条路同时给 ⇒ **当场报错**，不静默取一个。

    AFSIM 里 ``compute_measurement_errors`` 打开时 sigma 会被忽略。照抄那个
    行为，"我明明填了 ``azimuth_error_sigma``"就变成一句废话——而数字看着
    完全正常。本项目明确要消灭这类静默退化（与"半给发射机参数"同一条）。
    """
    with pytest.raises(ConfigurationError):
        _build(
            entities=[(1, 6, 0)],
            compute_measurement_errors=True,
            azimuth_error_sigma=0.3,
            **_BUDGET,
        )


def test_snr_path_requires_a_beamwidth() -> None:
    """开了由 SNR 反算却不给波束宽度 ⇒ 报错，**不退回零误差**。

    ``_BUDGET`` 只给增益、不给波束宽度（逐参法本来也不需要它）——正好触发
    这条。退回零误差的症状是"开了开关却一点误差都没有"，而参数表看着正常。
    """
    with pytest.raises(ConfigurationError):
        _build(entities=[(1, 6, 0)], compute_measurement_errors=True, **_BUDGET)


def test_snr_path_requires_a_pulse_width() -> None:
    """同上：距离误差 ∝ ``c·τ/2``，没有 τ 就算不出来。"""
    budget = {k: v for k, v in _BUDGET.items() if k != "pulse_width"}
    budget.update(beamwidth_h=2.0, beamwidth_v=2.0)
    with pytest.raises(ConfigurationError):
        _build(entities=[(1, 6, 0)], compute_measurement_errors=True, **budget)


def test_snr_path_makes_far_targets_coarser() -> None:
    """由 SNR 反算 ⇒ **远的目标误差大**。

    ★ 这条故意**不**断言"Pd 高所以 σ 小"：σ 与 Pd 是同一个 SNR 的两个**并列**
    输出，不是父子。写成"Pd ⇒ σ"就等于把同源误读成因果——日后有人把
    ``required_pd`` 调高，会以为误差该跟着变小，而**那不会发生**。

    ``SNR ∝ R⁻⁴`` ⇒ ``σ ∝ R²``：距离 ×20 ⇒ σ ×400。
    """
    radar, engine, store = _build(
        entities=[(1, 6, 0)],
        compute_measurement_errors=True,
        beamwidth_h=2.0,
        beamwidth_v=2.0,
        **_BUDGET,
    )
    near_bearing, near_range = radar._measurement_sigmas(10_000.0, 1.0)
    far_bearing, far_range = radar._measurement_sigmas(200_000.0, 1.0)
    assert far_bearing > near_bearing
    assert far_range > near_range
    assert far_bearing / near_bearing == pytest.approx(400.0, rel=0.02)
    assert far_range / near_range == pytest.approx(400.0, rel=0.02)


# ===========================================================================
# 第五层：平台级航迹管理器（汇聚 + 过时清理 + 转发）
# ===========================================================================


class _Named:
    """``registry.by_name`` 的最小返回值：只要 ``entity_id``。"""

    def __init__(self, entity_id: int) -> None:
        self.entity_id = entity_id


class _Registry:
    def __init__(self, names: dict[str, int] | None = None) -> None:
        self._names = dict(names or {})

    def by_name(self, name: str):
        entity_id = self._names.get(name)
        return None if entity_id is None else _Named(entity_id)


class _TrackSpec(dict):
    slot = "track"
    type_name = "TRACK_MANAGER"


class _TrackMount(_Mount):
    """在桩 mount 上加两样航迹管理器需要的：``track_view`` 与 ``registry``。"""

    def __init__(self, *args, names=None, **kw):
        super().__init__(*args, **kw)
        self.registry = _Registry(names)

    def track_view(self):
        return self.store.track_view(self.entity_id)


def _track_params(**overrides):
    values = {name: item.default for name, item in TrackManager.PARAMS.items()}
    values.update(overrides)
    return _TrackSpec(values)


def _track_rig(specs, *, names=None, **params):
    """造一个只装了 TrackManager 的平台。

    ``specs`` 是 ``[(entity_id, col, row)]``，**第一个是本平台**。
    返回 ``(manager, engine, store)``。
    """
    engine = _Engine()
    store = EntityStore()
    frame = LocalFrame(0.0, 0.0, _SIZE)
    for entity_id, col, row in specs:
        cell = Axial(col, row)
        store.add(entity_id, LocalCellRef(1, 0, cell))
        x, y = frame.to_world(cell)
        store.set_pose(entity_id, x, y, 0.0)

    manager = TrackManager(spec=_track_params(**params))
    mount = _TrackMount(engine, store, specs[0][0], names=names)
    manager.initialize(mount)
    return manager, engine, store


#: 本平台 0 / 友邻 1 / 目标 2 —— 与 ``test_store.py`` 的航迹用例同一套编号。
_ME, _PEER, _BANDIT = 0, 1, 2

_RIG = ((0, 0, 0), (1, 3, 0), (2, 6, 0))

#: 一份格式正确的航迹报文。``origin_id`` 必须是**具体实体 ID**——
#: 哨兵 ``ORIGIN_LOCAL`` 上了线就报错（见 ``test_store`` 的守门用例）。
_REPORT = {
    "target_id": _BANDIT,
    "quality": 0.6,
    "detected_at": 4_000,
    "bearing_deg": 90.0,
    "range_m": 6_000.0,
    "x": 6_000.0,
    "y": 0.0,
    "z": 0.0,
    "origin_id": _PEER,
    "hops": 0,
    # ★ 线格式 v0.13.30 起多了 ``phantom``（`Contact.phantom`）、v0.13.33 起多了
    # ``iff``（`Contact.iff`）。手写报文必须给全：`from_dict` 故意**不补默认值**
    # ——补了的话少一个字段就静默变成"真目标"或"未识别"。
    "phantom": False,
    "iff": IFF_FOE,
}


def test_track_manager_absorbs_a_relayed_report() -> None:
    """通信来的报告进得了本平台的航迹表，且**原始量测者**留在标记里。"""
    manager, engine, store = _track_rig(_RIG)
    store.post_message(_PEER, _ME, TRACK_REPORT, sent_at=1_000, deliver_at=1_000,
                       payload=dict(_REPORT))

    engine.tick_to(3.0)                       # 一拍（period 3 s）
    assert manager.ticks == 1
    assert manager.absorbed == 1

    contacts = store.contacts_of(_ME)
    assert [c.target_id for c in contacts] == [_BANDIT]
    assert contacts[0].origin_id == _PEER
    assert contacts[0].hops == 1
    assert not contacts[0].is_local
    assert store.contacts_of(_PEER) == []     # 只写进自己的域


def test_track_manager_does_not_eat_other_kinds_of_mail() -> None:
    """★ 守门人：航迹管理器**只取**航迹报告，别的报文原样留在收件箱里。

    一个平台上同时挂着指挥控制器与航迹管理器，两者共用一个收件箱。
    ``take_messages`` 是"取走即清空"，先跑的那个把第二个的信也吃了——
    症状是"指令偶尔不生效"，发作频率取决于同刻事件先后，基本复现不出来。
    """
    manager, engine, store = _track_rig(_RIG)
    store.post_message(_PEER, _ME, TRACK_REPORT, payload=dict(_REPORT))
    store.post_message(_PEER, _ME, "ADVANCE", payload={"to": "90"})
    store.post_message(_PEER, _ME, TRACK_REPORT, payload=dict(_REPORT))

    engine.tick_to(3.0)

    assert manager.absorbed == 1                       # 两遍同目标 ⇒ 覆盖更新
    left = store.peek_messages(_ME)
    assert [m.kind for m in left] == ["ADVANCE"]       # ★ 指令还在
    assert left[0].payload == {"to": "90"}


def test_track_manager_keeps_tracks_forever_by_default() -> None:
    """默认 ``drop_after_inactive 0`` ⇒ **一条都不撤**。

    与 AFSIM 同口径（它的 ``drop_after_inactive`` 不写就是无限）。这条守的是
    "新行为必须靠显式开关打开"：默认清理的话，所有历史想定的航迹条数都会变，
    而变的原因藏在参数默认值里。
    """
    manager, engine, store = _track_rig(_RIG)
    store.add_contact(_ME, _BANDIT, quality=0.5, detected_at=0, x=6_000.0)

    for second in (3.0, 6.0, 60.0, 600.0):
        engine.tick_to(second)
    assert manager.expired == 0
    assert store.contact_count(_ME) == 1


def test_track_manager_drops_tracks_after_inactivity() -> None:
    """``drop_after_inactive`` 到了就撤——判据是**严格大于**。

    ★ 与 M/N 撤航是**两件事**：M/N 管"还跟不跟得住"（传感器侧，看被照射
    次数），这里管"多久没有任何更新"（平台侧，看时间戳）。航迹**位置**从
    最后一次量测起就不动了（没有滤波器，不拿真位置外推），但"位置过时"
    与"航迹撤除"不是同一个时刻。
    """
    manager, engine, store = _track_rig(_RIG, drop_after_inactive=5_000_000)
    # ★ ``detected_at`` 的单位是**微秒**（SimTime），不是秒。写成 1_000 的话
    #   实际是 1 毫秒，航迹在第一拍就超时了——而断言看起来只是在测"相等不算超时"。
    store.add_contact(_ME, _BANDIT, quality=0.5, detected_at=1_000_000, x=6_000.0)

    engine.tick_to(3.0)                       # 3 s − 1 s = 2 s ≤ 5 s ⇒ 留着
    assert store.contact_count(_ME) == 1
    engine.tick_to(6.0)                       # 6 s − 1 s = 5 s，**相等不算超时**
    assert manager.expired == 0
    assert store.contact_count(_ME) == 1
    engine.tick_to(9.0)                       # 8 s > 5 s ⇒ 撤
    assert manager.expired == 1
    assert store.contact_count(_ME) == 0
    assert store.validate() == []


def test_track_manager_can_let_a_stale_track_die_and_a_fresh_one_survive() -> None:
    """撤的是**过时的那条**，不是整张表。"""
    manager, engine, store = _track_rig(_RIG, drop_after_inactive=5_000_000)
    store.add_contact(_ME, 1, detected_at=1_000_000, x=1.0)
    store.add_contact(_ME, _BANDIT, detected_at=1_000_000, x=2.0)

    engine.tick_to(6.0)
    assert store.contact_count(_ME) == 2
    store.add_contact(_ME, 1, detected_at=8_000_000, x=3.0)   # 只刷新其中一条
    engine.tick_to(9.0)

    assert [c.target_id for c in store.contacts_of(_ME)] == [1]
    assert manager.expired == 1


def test_share_requires_recipients_and_recipients_require_share() -> None:
    """★ 半给不行：一对参数给了半个，**装配期就报错**。

    静默的后果分别是"我明明配了共享，对端什么都没收到"与"我写了收件人，
    却什么都没发生"，两种都让参数表看起来完全正常。
    """
    with pytest.raises(ConfigurationError, match="share_recipients"):
        _track_rig(_RIG, share_interval=2_000_000)

    with pytest.raises(ConfigurationError, match="share_interval"):
        _track_rig(_RIG, names={"PEER": _PEER}, share_recipients=["PEER"])


def test_share_recipients_must_be_resolvable_names() -> None:
    """收件人名字**装配期**就解析，认不出来当场报错。

    放到运行期再解析的代价是"收件人还没建档时不发"——而航迹共享没有确认
    机制，少发一个谁都不会报错。
    """
    with pytest.raises(ConfigurationError, match="不在名册上"):
        _track_rig(_RIG, names={"PEER": _PEER}, share_interval=2_000_000,
                   share_recipients=["PEER", "GHOST"])

    with pytest.raises(ConfigurationError, match="自己"):
        _track_rig(_RIG, names={"ME": _ME}, share_interval=2_000_000,
                   share_recipients=["ME"])


def test_track_manager_sends_reports_onto_the_inflight_queue() -> None:
    """转发进的是**在途队列**，不是收件箱——直塞等于宣告通信瞬时无损。

    搬进收件箱的 ``deliver_due`` 目前没有生产调用方（通信件还没写，§9-44），
    所以这里由测试**扮演通信件**：等价于"链路已建模为理想、零延迟"。
    """
    manager, engine, store = _track_rig(
        _RIG, names={"PEER": _PEER}, share_interval=3_000_000,
        share_recipients=["PEER"],
    )
    store.add_contact(_ME, _BANDIT, quality=0.6, detected_at=100, x=6_000.0,
                      y=0.0, bearing_deg=90.0, range_m=6_000.0)

    engine.tick_to(3.0)
    assert manager.shared == 1
    assert len(store.inflight_messages()) == 1
    assert store.peek_messages(_PEER) == []        # 还没到，得由"通信件"搬

    store.deliver_due(3_000_000)
    delivery = store.take_messages(_PEER)
    assert [m.kind for m in delivery] == [TRACK_REPORT]
    assert delivery[0].payload["origin_id"] == _ME      # ★ 哨兵已落成实体 ID
    assert delivery[0].payload["origin_id"] != ORIGIN_LOCAL


def test_a_report_that_went_all_the_way_round_does_not_come_back() -> None:
    """端到端：A 测到 → 告诉 B → B 再告诉 A ⇒ A **不收**，A 那条仍是本机量测。

    ``circular_report_rejection``。判据要盯"来源标记有没有被洗掉"，不能只
    盯航迹条数：同一个目标本来就覆盖更新，条数永远是 1。
    """
    manager_a, engine, store = _track_rig(_RIG, names={"PEER": _PEER},
                                          share_interval=3_000_000,
                                          share_recipients=["PEER"])
    store.add_contact(_ME, _BANDIT, quality=0.6, detected_at=100, x=6_000.0)
    engine.tick_to(3.0)                                  # A 发出 1 条

    # -- 扮演通信件：搬到 B 的收件箱 --
    store.deliver_due(3_000_000)
    manager_b = TrackManager(spec=_track_params())
    manager_b.initialize(_TrackMount(_Engine(), store, _PEER))
    manager_b.absorb(3_000_000)
    assert store.contacts_of(_PEER)[0].origin_id == _ME

    # -- B 转回给 A --
    for message in store.track_view(_PEER).share_tracks(_ME, sent_at=4_000_000,
                                                        deliver_at=4_000_000):
        assert message.kind == TRACK_REPORT
    store.deliver_due(4_000_000)
    manager_a.absorb(4_000_000)

    assert manager_a.absorbed == 0
    assert manager_a.rejected_circular == 1
    mine = store.contacts_of(_ME)[0]
    assert mine.is_local and mine.origin_id == ORIGIN_LOCAL and mine.hops == 0


def test_relay_hops_are_bounded_across_platforms() -> None:
    """跳数上限在**跨平台**这条路上也生效。

    报告每转发一次就多一个收信人，没有上限时一次三角转发就能让航迹表按
    指数涨——而表涨了不报错，只是内存慢慢爬。
    """
    manager, engine, store = _track_rig(_RIG)
    payload = dict(_REPORT, hops=MAX_RELAY_HOPS)
    store.post_message(_PEER, _ME, TRACK_REPORT, payload=payload)
    engine.tick_to(3.0)

    assert manager.absorbed == 0
    assert manager.rejected_hops == 1
    assert store.contact_count(_ME) == 0


# ---------------------------------------------------------------------------
# 干扰方程（Jam.1 + RF.5）—— 纯函数层
#
# 这一层只钉"式子对不对"，不钉"谁在压谁"（那是 services/ew 与干扰机的事）。
# 全部用**闭式解**当期望值，不写死数字：数字写死会随常数漂移而变红，
# 而那正是我们想让它变的。
# ---------------------------------------------------------------------------

def test_bandwidth_overlap_spot_barrage_and_miss() -> None:
    """瞄准式不吃亏、阻塞式按 ``B_r/B_j`` 打折、错开归零。"""
    f_r, b_r = 10.0e9, 10.0e6
    # 瞄准式：1 MHz 落在接收带正中 ⇒ 干扰机的谱**全在里面** ⇒ 1.0
    assert eq.bandwidth_overlap_ratio(10.0e9, 1.0e6, f_r, b_r) == pytest.approx(1.0)
    # 阻塞式：100 MHz 盖住 10 MHz 的接收带 ⇒ 只有 1/10 的功率进得去
    assert eq.bandwidth_overlap_ratio(10.0e9, 100.0e6, f_r, b_r) == pytest.approx(0.1)
    # 完全错开：谱边刚好贴到接收带边（半开，判 0）
    assert eq.bandwidth_overlap_ratio(10.06e9, 10.0e6, f_r, b_r) == 0.0
    # 部分重叠：偏移 5 MHz、谱宽 10 MHz ⇒ 重叠 5 MHz ⇒ 0.5
    assert eq.bandwidth_overlap_ratio(10.005e9, 10.0e6, f_r, b_r) == pytest.approx(0.5)


def test_bandwidth_overlap_matches_the_afsim_closed_form() -> None:
    """与 RF.5 原文的分段式对齐——期望值**手算**，不用实现里的中间量。"""
    # 干扰机谱 [9.998, 10.018] GHz ^ 接收带 [9.995, 10.005] GHz = [9.998, 10.005]
    # ⇒ 重叠 7 MHz，分母是干扰机谱宽 20 MHz ⇒ 0.35
    assert eq.bandwidth_overlap_ratio(10.008e9, 20.0e6, 10.0e9, 10.0e6) == pytest.approx(
        7.0e6 / 20.0e6
    )
    # 接收带完全被盖住时不受 ``min(..., 1.0)`` 之外的影响：谱比接收带窄就封顶 1
    assert eq.bandwidth_overlap_ratio(10.0e9, 2.0e6, 10.0e9, 10.0e6) == pytest.approx(1.0)


def test_bandwidth_overlap_of_a_cw_jammer_is_all_or_nothing() -> None:
    """带宽取 0 是**连续波**，不是"除零错误"——落到带内=1、带外=0。"""
    assert eq.bandwidth_overlap_ratio(10.001e9, 0.0, 10.0e9, 10.0e6) == 1.0
    assert eq.bandwidth_overlap_ratio(10.02e9, 0.0, 10.0e9, 10.0e6) == 0.0
    with pytest.raises(ValueError, match="接收机带宽"):
        eq.bandwidth_overlap_ratio(10.0e9, 1.0e6, 10.0e9, 0.0)


def test_one_way_loss_is_half_the_decibels() -> None:
    """单程等效损耗 = 双程损耗的**一半 dB**。误用整份会偏 2 dB。"""
    for db in (0.0, 2.0, 4.0, 6.0, 10.0):
        assert eq.db(eq.one_way_loss(db)) == pytest.approx(db / 2.0)
    # 口径本身：4 dB 双程 ⇒ 线性 10^0.4 ⇒ 单程 10^0.2
    assert eq.one_way_loss(4.0) == pytest.approx(10.0**0.2)


def test_jamming_power_matches_the_closed_form_and_falls_off_as_r_squared() -> None:
    """Jam.1 的闭式解，以及与回波**不同的**衰减指数（R⁻² 对 R⁻⁴）。"""
    p_j, g_j, g_r = 100.0, 1000.0, 1000.0
    lam, r = eq.wavelength(10.0e9), 50_000.0
    expected = p_j * g_j * g_r * lam**2 / (eq.FOUR_PI**2 * r**2)
    assert eq.jamming_power(p_j, g_j, g_r, lam, r) == pytest.approx(expected)
    # 距离加倍 ⇒ 干扰四分之一（回波是十六分之一）
    assert eq.jamming_power(p_j, g_j, g_r, lam, 2.0 * r) == pytest.approx(expected / 4.0)


def test_zero_jamming_leaves_the_snr_untouched() -> None:
    """★ 守门人：「没挂干扰机 ⇒ 输出逐字不变」的**数学**依据。

    不是靠一个 ``if 有干扰机`` 挡住的，是 ``J/S = 0`` 时本式退化成恒等式。
    所以这条既要断言相等，也要断言是**逐位**相等——近似相等会放过一个
    写成 ``sn / (1 + js * sn + 1e-12)`` 的实现。
    """
    for sn in (0.0, 1e-6, 0.5, 13.0, 1e6):
        assert eq.snr_under_jamming(sn, 0.0) == sn
    assert eq.snr_under_jamming(0.0, 5.0) == 0.0


def test_snr_under_jamming_is_algebraically_identical_to_s_over_n_plus_j() -> None:
    """``SNR/(1+(J/S)·SNR)`` 与 ``S/(N+J)`` 必须**逐位**相同。

    两个写法各自成立，但只有这一个不用重走雷达方程：``S`` 与 ``N`` 只以
    比值出现。用一组 (S, N, J) 直接展开 ``S/(N+J)`` 做对照。
    """
    for s, n, j in ((1e-12, 1e-15, 3e-15), (1e-9, 1e-12, 1.0e-12),
                    (4.0, 1.0, 7.0)):
        sn = s / n
        js = eq.jamming_to_signal(j, n, sn)
        assert eq.snr_under_jamming(sn, js) == pytest.approx(s / (n + j), rel=1e-15)


def test_jamming_to_signal_needs_an_absolute_noise_power() -> None:
    """标定法 ``noise_w = 0`` ⇒ 接不了干扰，**当场报错**而不是当成没有干扰。

    静默返回 0 的症状是"标定法雷达挂了干扰机却一点不受影响"，而那正是
    最像"干扰没生效"、实际"参数没给全"的一类。
    """
    with pytest.raises(ValueError, match="噪声功率"):
        eq.jamming_to_signal(1e-12, 0.0, 100.0)
    assert eq.jamming_to_signal(0.0, 1e-15, 100.0) == 0.0


def test_self_protection_jamming_grows_as_r_squared_so_burn_through_exists() -> None:
    """★ 自卫式干扰的 ``J/S ∝ R²`` —— 这就是"烧穿距离"存在的原因，也是
    ``R_bt ∝ P_j^(−1/2)`` 的来源。

    回波 ∝ R⁻⁴、干扰 ∝ R⁻² ⇒ ``J/S ∝ R²``：**远距离干扰赢、近距离信号赢**，
    中间必有一个交点。功率 ×4 ⇒ 同一距离的 ``J/S`` ×4 ⇒ 交点距离**减半**。
    """
    lam = eq.wavelength(10.0e9)
    noise_w = 1e-15

    def js(range_m: float, power_w: float) -> float:
        snr = 1.0e4 * (10_000.0 / range_m) ** 4          # 回波 ∝ R⁻⁴
        j = eq.jamming_power(power_w, 1000.0, 1000.0, lam, range_m)
        return eq.jamming_to_signal(j, noise_w, snr)

    # J/S 随距离平方涨：距离加倍 ⇒ 比值 ×4
    assert js(40_000.0, 100.0) == pytest.approx(4.0 * js(20_000.0, 100.0))
    # 功率 ×4 ⇒ 同样只需要一半的距离就达到同一个 J/S ⇒ 烧穿距离 ∝ P^(-1/2)
    assert js(5_000.0, 400.0) == pytest.approx(js(10_000.0, 100.0))


# ---------------------------------------------------------------------------
# 干扰接入层：名册（services/ew）→ 干扰机（models/jam）→ 雷达第 6 步
#
# 上一层钉"式子对不对"，这一层钉"**接线对不对**"：谁登记、谁筛、谁算、
# 哪几个量从哪来。全部用**比值**当断言（同一条链上换一个参数），不写死数字。
# ---------------------------------------------------------------------------

#: 标定法雷达 + **接干扰要的那四项**。
#:
#: 标定法把噪声折进了标定曲线，所以它没有绝对的 N、λ、G——干扰那一支要用，
#: 就得单独给（§5.14）。``beamwidth_h/v`` 在标定法里不改变"走哪条路"：
#: 决定走不走雷达方程的是 ``peak_power``。
_EW_RADAR = dict(
    detect_range=30_000.0,
    frequency=4.0e9,
    bandwidth=1.0e6,
    beamwidth_h=2.0,
    beamwidth_v=2.0,
    noise_power=1.0e-15,
)

#: 逐参法的收发预算（§5.12.8 的同一条链）。用在只有逐参法才表达得了的
#: 那些量上——例如 ``system_loss_db``（写进标定法就不再是标定法了）。
_EW_EXPLICIT = dict(
    detect_range=30_000.0,
    frequency=4.0e9,
    bandwidth=1.0e6,
    antenna_gain_db=30.0,
    peak_power=1.0e5,
)


class _Sides:
    """``EntityRegistry`` 的桩：只回答 ``side_of`` 与 ``by_name``。

    真注册表要造 ``RegisteredEntity`` 并走一整套别名 / 阵营解析，而这一层
    测的是"名册与几何"，不是"阵营从哪来"。默认给空串——**假值**会让
    ``JamService`` 里"同阵营不打"那道筛子不生效，正好测不同阵营那条路。

    ``names``（v0.13.34 加）是把**实体名映射到实体 ID**，给"照敌航迹置向"
    里 ``aim_at`` 按名解析那一条用（⟨容错⟩口径：名与 ID 都认）。不传 ⇒
    ``by_name`` 恒返 ``None``，与加这一项之前逐字相同。
    """

    def __init__(self, sides=None, names=None) -> None:
        self._sides = dict(sides or {})
        self._names = dict(names or {})

    def side_of(self, entity_id: int) -> str:
        return self._sides.get(entity_id, "")

    def by_name(self, name: str):
        entity_id = self._names.get(name)
        if entity_id is None:
            return None
        return _NamedEntity(entity_id)


class _NamedEntity:
    """``by_name`` 的返回值桩：``JamService`` 只读它的 ``entity_id``。"""

    __slots__ = ("entity_id",)

    def __init__(self, entity_id: int) -> None:
        self.entity_id = entity_id


def _jammer_spec(entity_id: int, **overrides) -> JammerSpec:
    fields = dict(
        power_w=100.0,
        gain=1000.0,
        frequency_hz=4.0e9,
        bandwidth_hz=0.0,
        duty_cycle=1.0,
        use_peak_power=False,
        internal_loss=1.0,
        # ★ **-1 = 未给**（不是 1.0）：极化的默认口径是"按两端的**类型**查
        # AFSIM 那张 7×7 表"，而桩的两端类型都是 ``"default"`` ⇒ 表给 1.0。
        # 桩上写死 1.0 的话，等于给每条用例都偷偷加了一个"永远不扣"的覆盖，
        # 极化那几条便测不到真东西（v0.13.31）。
        polarization=-1.0,
        polarization_type="default",
        ignore_same_side=True,
        tag="STUB",
    )
    fields.update(overrides)
    return JammerSpec(entity_id=entity_id, **fields)


def _ew_build(*, entities=(), jammers=(), sides=None, seed=0, nav=None, **params):
    """建一部雷达 + 一份**真的**电子战门面（名册里装着 ``jammers``）。

    ``_build`` 只给得起 ``jam=None``（"这份装配没有电子战"）；要测压制就得
    有一份活的门面，而它必须在 ``EntityStore`` 之后才建得出来——所以这个
    辅助函数与 ``_build`` 并列，而不是给它加参数。返回 ``(雷达, 引擎, 存储, 门面)``。

    ``nav`` 与 ``_build`` 同名同义：干扰机那一侧的遮蔽判据要用它（v0.13.31）。
    不传 ⇒ 组件里的 ``_nav`` 是 ``None`` ⇒ **不判几何**（缺数据 ≠ 被遮挡）。
    """
    engine = _Engine()
    store = EntityStore()
    frame = LocalFrame(0.0, 0.0, _SIZE)
    store.add(0, LocalCellRef(1, 0, Axial(0, 0)))
    for entity_id, col, row in entities:
        cell = Axial(col, row)
        store.add(entity_id, LocalCellRef(1, 0, cell))
        x, y = frame.to_world(cell)
        store.set_pose(entity_id, x, y, 0.0)

    jam = JamService(store, _Sides(sides))
    for entity_id, overrides in jammers:
        jam.register(_jammer_spec(entity_id, **overrides))

    radar = HexSearchRadar(spec=_params(**params))
    radar.initialize(_Mount(engine, store, 0, nav=nav, seed=seed, jam=jam))
    return radar, engine, store, jam


def _range_between(store, target_id: int) -> tuple[float, float]:
    """``(斜距, 方位)``：从雷达（实体 0）到目标。斜距进方程、方位当波束指向。"""
    import math

    me = store.position_of(0)
    other = store.position_of(target_id)
    horizontal = math.hypot(other[0] - me[0], other[1] - me[1])
    return math.sqrt(horizontal * horizontal + (other[2] - me[2]) ** 2), math.degrees(
        math.atan2(other[0] - me[0], -(other[1] - me[1]))
    ) % 360.0


# -- 名册（services/ew）-----------------------------------------------------

def test_jam_service_needs_the_store_and_the_registry() -> None:
    """名册为空时**连位置都不查**——这就是"没挂干扰机 ⇒ 逐字不变"的来源。"""
    store = EntityStore()
    store.add(0, LocalCellRef(1, 0, Axial(0, 0)))
    jam = JamService(store, _Sides())

    assert jam.is_empty
    assert jam.specs() == []
    assert jam.threats(0, beam_bearing_deg=90.0, main_lobe_deg=5.0) == ()
    assert jam.queries == 0        # 空名册：一次查询都不算


def test_jam_service_refuses_two_machines_on_one_entity() -> None:
    """一个实体上挤**两台不同的机器**当场报错；**多天线照收**。

    ★ v0.13.33 把判据从"实体"收到了"**机器**"这一层：旧规则（一个实体只能有
    一台）只在"一台机器一个固定方向"的前提下成立；一旦拆成多天线，**同一条
    天线的**答案仍是唯一的。于是判据要能分开这两句：

    * 同名 ⇒ 这是同一台机器的**多条天线**，登记（§5.14.11 ③）；
    * 不同名 ⇒ 两台机器挤在一个实体上，报错——分不出谁是谁，而两台的参数
      都是用户自己写的、看起来都对。

    静默接收的症状是"我加的那台好像没生效"（被后登记的顶掉了）或者"两台拼成
    一台"（各天线功率加起来超过机器的额定功率）。
    """
    jam = JamService(EntityStore(), _Sides())
    jam.register(_jammer_spec(3))

    # ① 不同名的第二台 ⇒ 报错，且**没登记进去**（报错在写入之前）
    with pytest.raises(ConfigurationError, match="一台机器"):
        jam.register(_jammer_spec(3, tag="另一台"))
    assert len(jam) == 1 and jam.antennas_of(3) == 1

    # ② 同名的第二条天线 ⇒ 登记（这就是"多天线"）
    jam.register(_jammer_spec(3, tag="STUB"))
    assert jam.antennas_of(3) == 2
    assert len(jam) == 2                       # 名册的行数 = 天线条数
    assert jam.machine_names == ["STUB"]        # 机器仍然只有一台

    # ③ 逐条注销：**按卡**只撤一条，另一条还在
    first, second = jam.specs()
    assert jam.unregister(first)
    assert jam.antennas_of(3) == 1
    assert jam.specs() == [second]
    assert jam.unregister(second)
    assert jam.is_empty and len(jam) == 0

    # ④ 旧形态（只给实体 ID）语义是**整台机器**下架，并且"没有它"回 False
    jam.register(_jammer_spec(3))
    jam.register(_jammer_spec(3))
    assert jam.unregister(3) and len(jam) == 0
    assert jam.unregister(3) is False


def test_sides_and_collocation_filter_the_roster() -> None:
    """三道筛：不是自己、同阵营不打、不与自己同处一点。

    ★ "同阵营不打"**要两边都知道阵营**才有意义：受害方自己没有阵营时这一项
    判不了，于是**不过滤**。这是刻意的——把"不知道"当成"不同阵营"会让一部
    没编队的雷达连友军干扰机一起吃，而它不会报任何错。
    """
    store = EntityStore()
    frame = LocalFrame(0.0, 0.0, _SIZE)
    store.add(0, LocalCellRef(1, 0, Axial(0, 0)))
    store.add(1, LocalCellRef(1, 0, Axial(6, 0)))
    x, y = frame.to_world(Axial(6, 0))
    store.set_pose(0, 0.0, 0.0, 0.0)
    store.set_pose(1, x, y, 0.0)

    def roster(sides):
        jam = JamService(store, _Sides(sides))
        jam.register(_jammer_spec(0))      # 自己（同一个实体的自卫干扰机）
        jam.register(_jammer_spec(1))      # 另一台
        return jam

    # ① 受害方阵营未知 ⇒ 只筛掉"自己"，另一台留下
    alone = roster({1: "RED"}).threats(0, beam_bearing_deg=90.0, main_lobe_deg=2.0)
    assert [t.entity_id for t in alone] == [1]

    # ② 两边同为 RED ⇒ 友军那台被筛掉
    assert roster({0: "RED", 1: "RED"}).threats(
        0, beam_bearing_deg=90.0, main_lobe_deg=2.0
    ) == ()

    # ③ 对面阵营 ⇒ 它才是威胁；方位与斜距是**真实几何**算出来的
    other = JamService(store, _Sides({0: "RED", 1: "BLUE"}))
    other.register(_jammer_spec(1, bandwidth_hz=1.0e6))
    rows = other.threats(0, beam_bearing_deg=90.0, main_lobe_deg=2.0)
    assert len(rows) == 1
    assert rows[0].bearing_deg == pytest.approx(90.0, abs=1e-6)
    # 相邻格心距 × 6 —— 斜距是**真实几何**算出来的，不是接收机"知道"的
    assert rows[0].slant_m == pytest.approx(_SPAN_M * 6.0)
    assert rows[0].in_main_lobe and rows[0].off_beam_deg == pytest.approx(0.0)
    # ④ 波束转开 180° ⇒ 同一台干扰机掉进副瓣
    away = other.threats(0, beam_bearing_deg=270.0, main_lobe_deg=2.0)
    assert not away[0].in_main_lobe
    assert away[0].off_beam_deg == pytest.approx(180.0)


def test_angular_offset_wraps_across_zero() -> None:
    """跨零度的最小夹角——直接相减会让 359° 与 1° 差出 358°（主副瓣判反）。"""
    from milsim.services.ew import angular_offset_deg

    assert angular_offset_deg(1.0, 359.0) == pytest.approx(2.0)
    assert angular_offset_deg(359.0, 1.0) == pytest.approx(2.0)
    assert angular_offset_deg(0.0, 180.0) == pytest.approx(180.0)
    assert angular_offset_deg(10.0, 10.0) == 0.0


# -- 干扰机组件（models/jam）------------------------------------------------

class _JamSpec(dict):
    slot = "jammer"
    type_name = "RF_JAMMER"


class _JamMount:
    """干扰机 ``initialize`` 只用到这两样：自己的实体 ID 与电子战门面。"""

    def __init__(self, entity_id: int, jam) -> None:
        self.entity_id = entity_id
        self.jam = jam

    def jam_service(self):
        return self.jam


def _jam_params(**overrides) -> _JamSpec:
    """``RF_JAMMER`` 的一张全默认参数表，再覆写若干项。

    ★ 三种效果共用**一张**表（``services.ew`` 里只有一台干扰机的型号），所以
    这里没有"压制干扰机"与"欺骗干扰机"两套构造器——两张卡的存在性由**该组的
    值**决定（见 ``RfJammer.deception_card`` / ``false_target_card``）。
    """
    values = {name: item.default for name, item in RfJammer.PARAMS.items()}
    values.update(overrides)
    return _JamSpec(values)


def test_noise_jammer_registers_once_and_unregisters() -> None:
    """干扰机**没有 update**：它做的唯一一件事就是登记，关机时注销。"""
    jam = JamService(EntityStore(), _Sides())
    jammer = RfJammer(
        spec=_jam_params(peak_power=100.0e3, frequency=4.0e9, bandwidth=2.0e6)
    )
    jammer.initialize(_JamMount(3, jam))
    assert len(jam) == 1 and not jam.is_empty
    spec = jam.specs()[0]
    assert spec.entity_id == 3
    assert spec.radiated_power_w() == pytest.approx(100.0e3)
    assert spec.bandwidth_hz == pytest.approx(2.0e6)

    jammer.shutdown()
    assert jam.is_empty and len(jam) == 0


def test_one_machine_can_carry_several_antennas_sharing_the_power() -> None:
    """★ 一个部件实例 = **一条天线**；同平台上 N 个同名部件 = 一台机器的 N 条天线。

    两件事同时钉住（§5.14.11 ③）：

    * **默认功率分摊**：想定里那一个 ``peak_power`` 是**整台机器**的，不是
      每条天线的。同平台上两条同名天线 ⇒ 各拿一半（``100 kW / 2``），
      两条**加起来**才是 100 kW——不除的话"加一条天线"会静默地把系统功率
      翻倍，而参数表看起来完全正常。
    * **逐条注销**：关机是**按天线**发生的。整实体整批删的话，先关的那条会
      把后关的那条一起抹掉，症状是"我只关了一条天线，另一条也不压了"。
    """
    jam = JamService(EntityStore(), _Sides())
    first = RfJammer(spec=_jam_params(peak_power=100.0e3, frequency=4.0e9))
    first.spec.slot = "jammer2"                     # 槽名尾数 = 天线条数
    second = RfJammer(spec=_jam_params(peak_power=100.0e3, frequency=4.0e9))
    second.spec.slot = "jammer2"
    first.initialize(_JamMount(3, jam))
    second.initialize(_JamMount(3, jam))

    assert len(jam) == 2                            # 名册的行数 = 天线条数
    assert jam.antennas_of(3) == 2
    assert jam.machine_names == ["RF_JAMMER"]        # 机器仍然只有一台
    share, total = jam.specs()[0].power_w, first.total_power_w
    assert share == pytest.approx(50.0e3)
    assert total == pytest.approx(100.0e3)           # 机器总量是**除之前**那个数
    assert jam.specs()[0].antenna_count == 2
    assert jam.specs()[0].power_share() == pytest.approx(0.5)

    # 关机默认只撤自己那一条
    first.shutdown()
    assert jam.antennas_of(3) == 1
    assert jam.specs() == [second.jammer_spec]

    # 同一条天线再关一次 ⇒ 什么也不做（幂等），而不是顺手删掉别人
    first.shutdown()
    assert jam.antennas_of(3) == 1


def test_an_antenna_can_override_its_own_power() -> None:
    """每条天线自己写 ``peak_power`` ⇒ 压过**分摊**，但仍是按条数除过的数。

    ★ 用户那句话的精确落点："每个天线的功率可以自行设置，若不单独设置，
    则默认天线平分干扰机功率"——**两条都要走同一个口子**（``峰值 ÷ 条数``），
    否则一条天线写的 ``30 kW`` 会变成"这台机器给这条天线 30 kW"，而另一条
    还在按"总量 ÷ 条数"分，两条的口径就不一样了。所以：

    * 不写 ⇒ ``100 kW / 2 = 50 kW``（默认平分）；
    * 写了 ``30 kW`` ⇒ ``30 kW / 2 = 15 kW``（同样按 2 条除，口径一致）。

    ⇒ 判据是**份额的来源只有一个**（``_share_power``），不是"写了的那个数
    原封不动进去"。
    """
    jam = JamService(EntityStore(), _Sides())
    default = RfJammer(spec=_jam_params(peak_power=100.0e3, frequency=4.0e9))
    default.spec.slot = "jammer2"                    # 默认拿 50 kW
    own = RfJammer(spec=_jam_params(peak_power=30.0e3, frequency=4.0e9))
    own.spec.slot = "jammer2"                        # 30 kW / 2 = 15 kW
    default.initialize(_JamMount(4, jam))
    own.initialize(_JamMount(4, jam))

    powers = sorted(spec.power_w for spec in jam.specs())
    assert powers == [pytest.approx(15.0e3), pytest.approx(50.0e3)]


def test_a_single_antenna_keeps_the_old_power_bit_for_bit() -> None:
    """★ 守门人：**没写天线（槽名无尾数）⇒ 份额恒等于总量**。

    这不是特判出来的，是 ``total / 1`` 本身给的——所以旧想定那个数**逐位**
    不变（用 ``==`` 而不是 ``approx``：``100 kW / 1`` 必须一字不差）。
    """
    jam = JamService(EntityStore(), _Sides())
    jammer = RfJammer(spec=_jam_params(peak_power=100.0e3, frequency=4.0e9))
    assert jammer.spec.slot == "jammer"              # 桩的槽名没有尾数
    jammer.initialize(_JamMount(5, jam))
    spec = jam.specs()[0]
    assert spec.power_w == 100.0e3                   # 逐位
    assert spec.antenna_count == 1
    assert spec.power_share() == 1.0
    assert "×" not in jammer.describe()              # 单天线不挂标记


def test_the_antenna_count_is_read_off_the_slot_name() -> None:
    """天线条数只能从**槽名尾数**读，且读不出数字就是 1。

    ``jammer`` ⇒ 1、``jammer2`` ⇒ 2、``jammer_a`` ⇒ 1、``2nd_jammer`` ⇒ 1
    （尾数取不到 ⇒ 不猜），``jammer0`` ⇒ 0（⇒ 份额按 1 处理，不除零）。
    """
    cases = {
        "jammer": 1,
        "jammer2": 2,
        "jammer12": 12,
        "jammer_a": 1,
        "2nd_jammer": 1,
        "jammer0": 0,
    }
    for slot, expected in cases.items():
        jammer = RfJammer(spec=_jam_params(peak_power=1.0e3, frequency=4.0e9))
        jammer.spec.slot = slot
        assert jammer._antenna_count() == expected, slot
        # 尾数 0 时不除零：份额按 1 条算
        assert jammer._share_power(1000.0, expected) == pytest.approx(
            1000.0 if expected <= 1 else 1000.0 / expected
        )


def test_the_slot_tail_makes_the_antenna_count_visible() -> None:
    """``describe`` 里的 ``×N`` 是**多天线唯一能被看见的地方**。

    想定把同一个槽名写两遍与写 ``jammer2`` 一次，在**部件表**上不一样
    （前者两个部件），但"名义上几条"要靠这个标记才读得出来——尾数不同 ⇒
    各算 1 条，于是"我以为装了 3 条、其实只有 1 条"不会静默过去。
    """
    jam = JamService(EntityStore(), _Sides())
    two = RfJammer(spec=_jam_params(peak_power=100.0e3, frequency=4.0e9))
    two.spec.slot = "jammer2"
    two.initialize(_JamMount(6, jam))
    assert "×2" in two.describe()

    # 尾数不同的两条各算 1 条：各拿自己的全额（这正是"两台机器挤一个实体"
    # 的**名义**形态，靠 describe 与名册条数看得出来）
    three = RfJammer(spec=_jam_params(peak_power=90.0e3, frequency=4.0e9))
    three.spec.slot = "jammer3"
    other = RfJammer(spec=_jam_params(peak_power=60.0e3, frequency=4.0e9))
    other.spec.slot = "jammer3"
    three.initialize(_JamMount(8, jam))
    other.initialize(_JamMount(8, jam))
    assert three.describe().count("×3") == 1
    assert [spec.power_w for spec in jam.specs()] == pytest.approx(
        [50.0e3, 30.0e3, 20.0e3]
    )


def test_noise_jammer_refuses_a_half_written_configuration() -> None:
    """半给不行：**装配期**就报错，不留到求值时算出一个看着合理的假数。

    一台没有功率或没有频率的干扰机不是"不干扰"，是参数没写全——静默跑下去
    的症状是"挂了干扰机却好像没生效"。
    """
    jam = JamService(EntityStore(), _Sides())

    with pytest.raises(ConfigurationError) as info:
        RfJammer(spec=_jam_params(peak_power=100.0e3)).initialize(_JamMount(1, jam))
    assert "frequency" in str(info.value)
    assert "半给" in str(info.value)

    with pytest.raises(ConfigurationError) as info:
        RfJammer(spec=_jam_params(frequency=4.0e9)).initialize(_JamMount(1, jam))
    assert "peak_power" in str(info.value)

    with pytest.raises(ConfigurationError) as info:
        RfJammer(
            spec=_jam_params(peak_power=100.0e3, frequency=4.0e9, duty_cycle=0.0)
        ).initialize(_JamMount(1, jam))
    assert "duty_cycle" in str(info.value)
    assert jam.is_empty                    # 三次都没登记进去


def test_noise_jammer_needs_the_ew_facade() -> None:
    """没有门面就无处登记 ⇒ 报错，**不静默变成一台不存在的干扰机**。"""
    with pytest.raises(ConfigurationError, match="电子战"):
        RfJammer(
            spec=_jam_params(peak_power=100.0e3, frequency=4.0e9)
        ).initialize(_JamMount(1, None))


def test_the_duty_cycle_enters_the_jamming_power_by_default() -> None:
    """★ 用户问的"脉冲还是宽频"里的**时域**那一半，钉在源码口径上。

    AFSIM 的 ``WsfEM_Xmtr::GetPower()`` 默认走 ``GetAveragePower()``
    （``mUsePeakPower`` 初值 ``false``、``mDutyCycle`` 初值 ``1.0``），
    所以脉冲干扰的占空比**直接进 J**：1% ⇒ −20 dB。只有明确写
    ``use_peak_power true`` 的那一台（峰值功率受限）才不吃这个折扣。
    """
    cw = _jammer_spec(1, power_w=100.0, duty_cycle=1.0)
    pulsed = _jammer_spec(1, power_w=100.0, duty_cycle=0.01)
    peak_limited = _jammer_spec(1, power_w=100.0, duty_cycle=0.01, use_peak_power=True)

    assert cw.radiated_power_w() == pytest.approx(100.0)
    assert pulsed.radiated_power_w() == pytest.approx(1.0)
    assert peak_limited.radiated_power_w() == pytest.approx(100.0)
    # −20 dB 就是 1% 占空比的折扣
    assert eq.db(pulsed.radiated_power_w() / cw.radiated_power_w()) == pytest.approx(-20.0)


# -- 雷达第 6 步 ------------------------------------------------------------

def test_no_jammer_leaves_the_detection_chain_bit_for_bit() -> None:
    """★ 守门人：「没挂干扰机 ⇒ 一个数都不变」。

    两条路各建一次同一部雷达：①``jam = None``（这份装配没有电子战）、
    ②有门面但**名册是空的**。后者比前者多一次属性查询，除此之外必须**逐位**
    相同——``last_pd`` 用 ``==`` 而不是 ``approx``，因为"逐字不变"是这个
    里程碑唯一的验收判据（§5.14），近似相等会放过一个
    ``sn / (1 + js * sn + 1e-12)`` 的实现。
    """
    plain, engine_a, _ = _build(entities=((1, 6, 0),), seed=7)
    _run(engine_a, 4.0)

    empty, engine_b, _, jam = _ew_build(entities=((1, 6, 0),), seed=7)
    _run(engine_b, 4.0)

    assert plain.detections > 0
    assert empty.detections == plain.detections
    assert empty.contacts_seen == plain.contacts_seen
    assert empty.last_pd == plain.last_pd          # 逐位
    assert empty.jamming_w == 0.0
    assert empty.jammed_evals == 0
    assert jam.queries == 0                        # 空名册连查都没查过


def test_a_calibrated_radar_must_be_given_the_four_jamming_quantities() -> None:
    """标定法接干扰要四样：绝对噪声功率、天线增益、频率、带宽。

    **一次报全**，而不是让求值算出 0 或无穷。报错时机在"第一次真有干扰要
    算"的时候——干扰机是运行期才知道在不在的，装配期查不了。
    """
    radar, _, _, _ = _ew_build(entities=((1, 6, 0), (2, 6, 0)), jammers=((2, {}),))
    with pytest.raises(ConfigurationError) as info:
        radar.received_jamming_w(90.0)
    message = str(info.value)
    for name in ("noise_power", "beamwidth_h", "frequency", "bandwidth"):
        assert name in message


def test_self_protection_jamming_pushes_the_detection_probability_down() -> None:
    """压制干扰的**后果只有一条**：``S/(N+J)`` 变小 ⇒ Pd 变小。

    干扰机与目标同处一格（自卫式），所以天线指着目标时它就在**主瓣**里。
    """
    entities = ((1, 17, 0), (2, 17, 0))
    plain, _, store_a, _ = _ew_build(entities=entities, **_EW_RADAR)
    radar, _, store_b, jam = _ew_build(
        entities=entities, jammers=((2, {"power_w": 1.0e-8}),), **_EW_RADAR
    )
    slant, bearing = _range_between(store_b, 1)

    j_w = radar.received_jamming_w(bearing)
    assert j_w > 0.0
    assert jam.threats_seen == 1

    plain_snr = plain.single_pulse_snr(slant, 1.0)
    jammed_snr = radar.single_pulse_snr(slant, 1.0, jammer_noise_w=j_w)
    # 干扰进的是**噪声**：SN = (S/N) / (1 + J/N)
    assert jammed_snr == pytest.approx(plain_snr / (1.0 + j_w / radar.noise_w), rel=1e-12)
    assert jammed_snr < plain_snr
    # 干扰认的是这个 SNR ⇒ Pd 跟着下来（"能烧穿就烧穿"）
    assert radar.detection_probability(
        slant, 1.0, jammer_noise_w=j_w
    ) < plain.detection_probability(slant, 1.0)


def test_the_three_scaling_laws_of_jamming_power() -> None:
    """同一台干扰机换三个参数，J 各差一个**确定**的倍数。

    这是"参数真的接进去了"的判据：三个倍数都能由定义式直接写出来，
    所以实现里少乘一项、乘错方向，立刻不对称。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    radar, _, store, jam = _ew_build(entities=entities, **_EW_RADAR)
    _slant, bearing = _range_between(store, 1)

    def with_jammer(**overrides) -> float:
        # 换一台干扰机就是**换一张卡**：注销旧的、登记新的（不重建雷达）
        jam.unregister(2)
        jam.register(_jammer_spec(2, **overrides))
        return radar.received_jamming_w(bearing)

    spot = with_jammer(bandwidth_hz=0.0)

    # ① 频域：阻塞式铺开 10 倍带宽 ⇒ 只有 1/10 的功率落进接收带
    assert with_jammer(bandwidth_hz=10.0e6) == pytest.approx(spot / 10.0)
    # ② 时域：占空比 1% ⇒ 平均功率 1/100（AFSIM 源码口径）
    assert with_jammer(duty_cycle=0.01) == pytest.approx(spot / 100.0)
    assert with_jammer(bandwidth_hz=0.0, duty_cycle=1.0) == pytest.approx(spot)

    # ③ 波段：完全错开 ⇒ **一分钱都不进**（跳频就是靠这一项）
    assert with_jammer(frequency_hz=10.0e9) == 0.0
    # ④ 副瓣：把波束转到干扰机的反方向 ⇒ 只剩 −13 dB 的接收增益
    # （先把卡换回瞄准式——上一步留在名册里的那台还在带外）
    assert with_jammer(bandwidth_hz=0.0) == pytest.approx(spot)
    assert radar.received_jamming_w(bearing + 180.0) == pytest.approx(
        spot * eq.linear(DEFAULT_SIDELOBE_LEVEL_DB)
    )


def test_the_jamming_path_takes_only_the_receive_branch_loss() -> None:
    """干扰只扣**受害方接收支路**内损（AFSIM 的 ``receive_loss``，§9-54）。

    ★ **v0.13.36 口径修正**：旧口径拿雷达的双程总损耗劈一半（``√L``）扣给
    干扰，比 AFSIM 高 **+2.4999 dB**（两个独立 AFSIM 场景同值，§5.14.13）。
    现在只扣 ``receive_loss_db`` 那一份。

    这一条只能在逐参法上测（``receive_loss_db`` 才是它的归宿；标定法另有一条）。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    clean, _, _, _ = _ew_build(entities=entities, jammers=((2, {}),), **_EW_EXPLICIT)
    # ① 只给 system_loss_db（双程总损耗）⇒ **干扰看不到它**：J 与 clean 逐字相同。
    #    这正是"旧口径错在哪"的直接断言：旧口径会在这里扣 ½·4 = 2 dB。
    sysl, _, _, _ = _ew_build(
        entities=entities, jammers=((2, {}),), **dict(_EW_EXPLICIT, system_loss_db=4.0)
    )
    assert clean.range_model == sysl.range_model == "逐参法"
    assert sysl.received_jamming_w(90.0) == pytest.approx(clean.received_jamming_w(90.0))
    # ② 给 receive_loss_db = 4 dB ⇒ J **正好**降 4 dB（不是 2 dB）
    rx, _, _, _ = _ew_build(
        entities=entities, jammers=((2, {}),), **dict(_EW_EXPLICIT, receive_loss_db=4.0)
    )
    assert eq.db(rx.received_jamming_w(90.0) / clean.received_jamming_w(90.0)) == (
        pytest.approx(-4.0)
    )
    # ③ 回波那一支不受 receive_loss_db 影响（它是"接收支路内损"，只对干扰生效）
    assert rx.loss == pytest.approx(clean.loss)


def test_a_lone_transmit_loss_is_rejected() -> None:
    """只写 ``transmit_loss_db`` 而没写 ``system_loss_db`` ⇒ 当场报错。

    发射内损**不进干扰链路**（信号不是这部雷达发出去的），而回波走的是
    ``system_loss_db``。所以单独一个 ``transmit_loss_db`` 谁都影响不到——
    必须报错，不能静默吞掉（症状是"我写了 2 dB，J 一点没变"）。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    with pytest.raises(ConfigurationError, match="transmit_loss_db"):
        _ew_build(
            entities=entities,
            jammers=((2, {}),),
            **dict(_EW_EXPLICIT, transmit_loss_db=2.0),
        )


def test_the_calibrated_and_the_explicit_radar_agree_on_jamming_power() -> None:
    """标定法与逐参法算出的 J **必须一致**——它是同一个物理量。

    标定法从 ``beamwidth_h/v`` 换增益、从 ``frequency`` 取波长，逐参法用
    ``antenna_gain_db``——两条路各推一次同一个数，只要口径一致就该对上。
    对不上就说明标定法那一支把公式重写了一遍（"能算的数别另写一个数"）。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    calibrated, _, _, _ = _ew_build(entities=entities, jammers=((2, {}),), **_EW_RADAR)
    explicit, _, _, _ = _ew_build(
        entities=entities,
        jammers=((2, {}),),
        **dict(
            _EW_RADAR,
            noise_power=0.0,          # 逐参法自己算 N，不许再给一个
            antenna_gain_db=eq.db(eq.peak_gain(2.0, 2.0)),
            peak_power=1.0e5,
        ),
    )
    assert explicit.range_model == "逐参法"
    assert calibrated.range_model == "标定法"
    assert explicit.received_jamming_w(90.0) == pytest.approx(
        calibrated.received_jamming_w(90.0), rel=1e-6
    )


def test_the_explicit_path_refuses_a_second_noise_power() -> None:
    """逐参法的 N 由噪声温度链算出来，**不许**再给一个 ``noise_power``。"""
    with pytest.raises(ConfigurationError, match="noise_power"):
        _build(
            peak_power=1.0e5,
            frequency=4.0e9,
            antenna_gain_db=30.0,
            bandwidth=1.0e6,
            noise_power=1.0e-15,
        )


def test_a_calibrated_radar_can_carry_a_band_without_becoming_explicit() -> None:
    """★ 判据重划：``frequency`` / ``beamwidth_h·v`` **不**决定走哪条路。

    它们只在标定法里也有意义（接干扰要用），所以"给标定法雷达补一个频率"
    不该被判成"写了发射机参数却缺 peak_power"。决定走哪条路的只有
    ``peak_power`` 与那三个**只进噪声温度支**的接收机参数。
    """
    radar, _, _, _ = _ew_build(entities=((1, 6, 0),), **_EW_RADAR)
    assert radar.range_model == "标定法"
    assert radar.frequency_hz == pytest.approx(4.0e9)
    assert radar.bandwidth_hz == pytest.approx(1.0e6)
    assert radar.wavelength_m == pytest.approx(eq.wavelength(4.0e9))
    assert radar.gain == pytest.approx(eq.peak_gain(2.0, 2.0))
    assert radar.noise_w == pytest.approx(1.0e-15)


# -- 欺骗（假信号）----------------------------------------------------------

def _deception_rig(
    card: DeceptionSpec,
    *,
    power_w: float = 1.0e-12,
    entities=((1, 6, 0), (2, 6, 0)),
    **params,
):
    """一部标定法雷达 + 一台带**欺骗卡**的干扰机。返回 ``(雷达, 引擎, 存储, 门面)``。

    ``power_w`` 默认极小：欺骗要单独测，就得让**功率那一支几乎不动** Pd，
    否则"航迹被挪走了"与"航迹根本没建起来"会混成同一个现象（见下面那条
    关于"欺骗与压制共用一个功率"的说明）。

    ★ ``scan_interval`` 压到 1 s（= 本族用例的拍照间隔，也是 ``beam_step``
    的默认值）**是刻意的**。默认周期 2 s 配 1 s 一拍照 ⇒ 每拍只转 180°，
    方位 90° 处的目标**隔一拍才被照到一次**；而 ``jamming_range_m`` 是
    "被照射那一刻"写的诊断字段，于是"每一拍都读一次它"的写法在偶数拍读到
    的是**上一拍**的数——"拖引没长"与"停机不清表"这两种现象会一起被读成
    "参数没生效"，而参数表上看不出任何问题（实测就栽在这里）。
    周期 == 拍照间隔 ⇒ 每拍正好转过整圈，``fov = 360`` 走"本拍覆盖整片
    视场"那一支 ⇒ **每拍每个候选都求值一次**，拖引行程与真实经过的时间
    一一对应，读诊断的时机才不再是这件事的一部分。
    """
    return _ew_build(
        entities=entities,
        jammers=((2, {"power_w": power_w, "deception": card}),),
        scan_interval=1_000_000,
        **_EW_RADAR,
        **params,
    )


def test_the_deception_gate_needs_enough_jamming_power() -> None:
    """★ 门限：``J/S`` 不到 ``required_j_to_s_db`` 就**一点都不骗**。

    少了这一道，一台 1 W 的干扰机也能把航迹拉到几百米外——而那在物理上
    没有依据（假信号要盖过回波才像真的）。
    """
    radar, _, store, jam = _ew_build(entities=((1, 6, 0), (2, 6, 0)), **_EW_RADAR)
    slant, bearing = _range_between(store, 1)
    sn = radar.single_pulse_snr(slant, 1.0)
    card = DeceptionSpec(required_j_to_s_db=3.0, range_bias_m=800.0)

    # 先量"每瓦进来多少 J"：功率与 J 成正比（Jam.1 是线性的），所以一次标定
    # 就能把"要某个 J/S 该给多少瓦"反算出来，不必重推整条链。
    jam.register(_jammer_spec(2, power_w=1.0, bandwidth_hz=1.0e6))
    j_per_watt = radar.received_jamming_w(bearing)
    assert j_per_watt > 0.0

    def power_for(j_to_s: float) -> float:
        return j_to_s / eq.jamming_to_signal(j_per_watt, radar.noise_w, sn)

    # 门限以下（0.9 倍）⇒ 只有功率，没有偏置
    jam.unregister(2)
    jam.register(
        _jammer_spec(
            2,
            power_w=power_for(eq.linear(3.0) * 0.9),
            bandwidth_hz=1.0e6,
            deception=card,
        )
    )
    under = radar.jamming_effect(bearing, sn)
    assert under.power_w > 0.0 and under.pullers == 0
    assert under.range_bias_m == 0.0 and under.azimuth_bias_deg == 0.0

    # 门限以上 ⇒ 偏置就是卡上那个值（静态偏置，没有拖引）
    jam.unregister(2)
    jam.register(
        _jammer_spec(
            2,
            power_w=power_for(eq.linear(3.0) * 1.1),
            bandwidth_hz=1.0e6,
            deception=card,
        )
    )
    over = radar.jamming_effect(bearing, sn)
    assert over.pullers == 1
    assert over.range_bias_m == pytest.approx(800.0)
    assert over.peak_j_to_s >= eq.linear(3.0)


def test_the_deception_bias_lands_on_the_contact() -> None:
    """★ 偏置落在**航迹**上——这是欺骗唯一能被看见的地方。

    压制让航迹**建不起来**（Pd 低），欺骗让航迹**建得起来但是错的**。
    所以判据要读 ``Contact``：距离多了偏置那么多、方位转了那么多。

    门限开到极低（``-200 dB``）是有意的：本版欺骗与压制**共用同一个功率**，
    骗得成的功率按定义已经压住了 Pd（§9-50）。要单独看"偏置怎么落在量测上"，
    就把门限放开、功率压到极小——这时 Pd 几乎不动，航迹照常建起来。
    """
    card = DeceptionSpec(
        required_j_to_s_db=-200.0, range_bias_m=1500.0, azimuth_bias_deg=4.0
    )
    radar, engine, store, _ = _deception_rig(card, **_INSTANT_TRACK)
    slant, bearing = _range_between(store, 1)
    _run(engine, 1.0)

    assert radar.jamming_range_m == pytest.approx(1500.0)
    assert radar.jamming_azimuth_deg == pytest.approx(4.0)
    contact = store.contacts_of(radar.view.entity_id)[0]
    # 两个 σ 都是 0 ⇒ 逐字可算：水平距离 = 真斜距 + 偏置（高差为 0）
    assert contact.range_m == pytest.approx(slant + 1500.0, rel=1e-12)
    assert contact.bearing_deg == pytest.approx(bearing + 4.0, abs=1e-9)


def test_range_pull_off_grows_with_time_and_stops_at_the_holdout() -> None:
    """拖引（RGPO）：行程 ``= 速率 × 被跟踪时间``，到 ``holdout`` 就停住。

    ``recycle`` 关掉时"拉满就停在那儿"——真雷达的跟踪环路可能被拉到门限外
    （那是它自己的事），而干扰机的动作就是**拉到顶为止**。
    """
    card = DeceptionSpec(
        required_j_to_s_db=-200.0,
        range_bias_m=100.0,
        walkoff_rate_mps=50.0,
        holdout_m=300.0,
    )
    radar, engine, _, _ = _deception_rig(card)

    _run(engine, 1.0)                       # 第一帧只把行程表建起来：行程 0
    assert radar.jamming_range_m == pytest.approx(100.0)
    engine.tick_to(2.0)                     # 被跟踪 1 s ⇒ 50 m
    assert radar.jamming_range_m == pytest.approx(150.0)
    engine.tick_to(5.0)                     # 4 s ⇒ 200 m
    assert radar.jamming_range_m == pytest.approx(300.0)
    engine.tick_to(7.0)                     # 6 s ⇒ 300 m：正好封顶
    assert radar.jamming_range_m == pytest.approx(400.0)
    engine.tick_to(9.0)                     # 8 s ⇒ 仍 300 m：拉满就停
    assert radar.jamming_range_m == pytest.approx(400.0)


def test_recycle_lets_the_pull_off_start_over() -> None:
    """``recycle`` = **拉满就松手重来**：先把波门拖走，再突然松手让环路扑空。"""
    card = DeceptionSpec(
        required_j_to_s_db=-200.0,
        range_bias_m=100.0,
        walkoff_rate_mps=50.0,
        holdout_m=300.0,
        recycle=True,
    )
    radar, engine, _, _ = _deception_rig(card)

    _run(engine, 1.0)
    engine.tick_to(7.0)
    # 刚好拉满的那一帧就松手 ⇒ 偏置回到静态值（不是 400）
    assert radar.jamming_range_m == pytest.approx(100.0)
    engine.tick_to(8.0)                     # 新一轮 1 s ⇒ 50 m
    assert radar.jamming_range_m == pytest.approx(150.0)


def test_the_pull_off_forgets_a_jammer_that_stops() -> None:
    """干扰机一停，行程表清空；再开机**从头拉**。

    不清的后果是"停了很久又开机，第一帧就报出一个已经拉了几百米的偏置"——
    参数表与日志都看不出这个数是从哪儿来的。
    """
    card = DeceptionSpec(
        required_j_to_s_db=-200.0,
        range_bias_m=100.0,
        walkoff_rate_mps=50.0,
        holdout_m=300.0,
    )
    radar, engine, _, jam = _deception_rig(card)
    _run(engine, 1.0)
    engine.tick_to(7.0)
    assert radar.jamming_range_m == pytest.approx(400.0)

    # 关机（名册里没有它了）⇒ 下一拍行程表被清掉，偏置归零
    jam.unregister(2)
    engine.tick_to(8.0)
    assert radar.jamming_range_m == 0.0
    assert radar.jamming_w == 0.0

    # 再开机 ⇒ 从静态偏置重新开始，而不是接着上次的 400
    jam.register(
        _jammer_spec(2, power_w=1.0e-12, deception=card)
    )
    engine.tick_to(9.0)
    assert radar.jamming_range_m == pytest.approx(100.0)


def test_the_three_effects_share_one_table_and_one_component() -> None:
    """★ 组件层：**一个组件、一张表、三组卡**——三张卡可以同时挂在同一台机器上。

    这就是不把压制 / 欺骗 / 假目标拆成三个组件类型的直接理由：三组量共用同一份
    功率（``peak_power`` 等），拆开之后每一型都要把核心那张表再抄一份；而门面
    规定"一个实体只能有一台干扰机" ⇒ "同时压制 + 欺骗 + 假目标"会变成**写不
    出来**的配置。AFSIM 的口径也是并列的（一个 ``technique`` 里多个 ``effect``）。
    """
    jam = JamService(EntityStore(), _Sides())

    # ① 三组卡一项都不给 ⇒ 纯压制：两张卡都交不出来
    RfJammer(
        spec=_jam_params(peak_power=1.0e3, frequency=4.0e9)
    ).initialize(_JamMount(1, jam))
    plain = jam.specs()[0]
    assert plain.deception is None and plain.false_targets is None

    # ② 同一台机器：欺骗卡与假目标卡**同时**挂上
    both = RfJammer(
        spec=_jam_params(
            peak_power=1.0e3,
            frequency=4.0e9,
            deception_required_j_to_s_db=6.0,
            deception_range_bias_m=250.0,
            deception_walkoff_rate_mps=-12.0,
            deception_holdout_m=900.0,
            deception_recycle=True,
            false_target_pulse_density=0.1,
        )
    )
    both.initialize(_JamMount(2, jam))
    spec = jam.specs()[1]
    card = spec.deception
    assert card is not None
    assert card.required_j_to_s_db == pytest.approx(6.0)
    assert card.range_bias_m == pytest.approx(250.0)
    # 速率**带符号**：往哪边拉是这张卡的一部分
    assert card.walkoff_rate_mps == pytest.approx(-12.0)
    assert card.holdout_m == pytest.approx(900.0)
    assert card.recycle is True
    ft = spec.false_targets
    assert ft is not None
    assert ft.pulse_density == pytest.approx(0.1)

    # ③ 只给 recycle / 门限 **不算**有欺骗卡：卡的存在性由那三个位移量决定
    #    （一张什么都不搬的卡没有任何可观察后果，让它存在只会多一个"到底骗不骗"
    #    的出口）
    lazy = RfJammer(
        spec=_jam_params(
            peak_power=1.0e3, frequency=4.0e9, deception_recycle=True
        )
    )
    lazy.initialize(_JamMount(3, jam))
    assert jam.specs()[2].deception is None


def test_the_false_target_density_and_the_quantity_are_mutually_exclusive() -> None:
    """两个都给 ⇒ **装配期报错**（AFSIM 原文 "mutually exclusive"）。

    静默取其一的话，症状是"我改的那个参数没反应"，而参数表上两个值都写着、
    看起来都对——正是本项目最要消灭的那类静默退化。
    """
    jam = JamService(EntityStore(), _Sides())
    with pytest.raises(ConfigurationError) as info:
        RfJammer(
            spec=_jam_params(
                peak_power=1.0e3,
                frequency=4.0e9,
                false_target_pulse_density=0.1,
                false_target_quantity=50,
            )
        ).initialize(_JamMount(1, jam))
    assert "mutually exclusive" in str(info.value)
    assert jam.is_empty                      # 没登记进去（报错在登记之前）

    RfJammer(
        spec=_jam_params(
            peak_power=1.0e3, frequency=4.0e9, false_target_quantity=50
        )
    ).initialize(_JamMount(1, jam))
    assert jam.specs()[0].false_targets.quantity == 50


# -- 假目标（幽灵）----------------------------------------------------------

#: 假目标那一支要的**波形量**（§5.14）。
#:
#: ★ ``pulses_integrated`` 显式给：不给的话 ``N`` 会由 ``驻留时间 × PRF``
#: 反算（见 ``_resolve_pulses``），于是条数公式里多挂了一个"N 从哪来"的变量
#: ——用例失败时读不出到底是公式错了还是 N 的算法错了。
#:
#: ``τ = 1 µs`` ⇒ 分辨单元 ``c·τ/2`` ≈ 149.896 m；``PRF = 10 kHz`` ⇒
#: ``PRI = 100 µs``、不模糊距离 ``c·PRI/2`` ≈ 14.99 km，而一个 PRI 里正好
#: 放得下 **100** 个分辨单元（这正是条数公式的第一项）。
_FT_WAVE = dict(
    pulse_width=1.0,
    pulse_repetition_frequency=10_000.0,
    pulses_integrated=4,
)


def _false_target_rig(
    card: FalseTargetSpec,
    *,
    power_w: float = 1.0e-18,
    entities=((1, 6, 0), (2, 6, 0)),
    **params,
):
    """一部标定法雷达 + 一台带**假目标卡**的干扰机。返回 ``(雷达, 引擎, 存储, 门面)``。

    ``power_w`` 默认小到看不见 ⇒ **压制那一支几乎不动 Pd**：三种效果在同一台
    机器上**同时生效**（§5.14），功率给大了之后"屏幕上多了幽灵"与"真目标根本
    没被看见"会混成同一个现象。

    ``scan_interval`` 取 :data:`_REV_PER_BEAT`（= 拍照间隔）⇒ 每拍正好转过整圈、
    ``fov = 360`` 走"本拍覆盖整片视场"那一支 ⇒ **每拍都铺一次幽灵**。理由与
    ``_deception_rig`` 同：否则"扫到没有"的周期会混进判据里，读起来就不是在
    测假目标了。
    """
    waves = dict(_FT_WAVE)
    waves.update(params)                  # 用例可以覆写成"缺项"（波形量是同一张表）
    return _ew_build(
        entities=entities,
        jammers=((2, {"power_w": power_w, "false_targets": card}),),
        scan_interval=_REV_PER_BEAT,
        **_EW_RADAR,
        **waves,
    )


def test_the_false_target_count_follows_the_afsim_formula() -> None:
    """★ 条数公式照抄 AFSIM（``WSF_FT_EFFECT``），**逐项**对：

    ``(PRI/PW) · (ScanTime/PRI) · NumberPulsesIntegrated · jamming_pulse_density``

    四项各说各的事（一个 PRI 里有多少个分辨单元 / 一次扫描有多少个 PRI /
    一次照射积累多少脉冲 / 每个脉冲有多大概率被当成假回波），所以不能只对一个
    乘积：少乘一项、乘错方向，单看总数都可能"看着挺合理"。
    """
    density = 0.1
    radar, engine, _, _ = _false_target_rig(FalseTargetSpec(pulse_density=density))
    _run(engine, 1.0)

    pri_s = 1.0 / 10_000.0
    pw_s = 1.0e-6
    scan_s = _REV_PER_BEAT / 1.0e6
    assert radar.pulses == 4
    assert radar.false_targets_seen == int(
        (pri_s / pw_s) * (scan_s / pri_s) * radar.pulses * density
    )
    assert radar.false_targets_seen == 400_000            # 100 × 10 000 × 4 × 0.1
    # 想造 40 万条，真正上屏幕的是**三个上界的最小值**（下一条用例逐段钉）
    assert radar.phantoms_seen == 100


def test_the_phantom_chain_starts_outside_the_jammer_and_steps_by_the_range_cell() -> None:
    """幽灵链的**几何**：一条一个分辨单元，从干扰机之外**一个单元**起铺。

    * 间距 ``ΔR = c·τ/2``——比它更密的两条回波雷达分不开，"一条幽灵占一个
      距离门"的物理依据就在这；
    * 起点是 ``R_j + ΔR`` 而不是 ``R_j``：**假目标落不到干扰机前面**
      （AFSIM 的 ``false_target`` 里 ``range_constrained`` 默认 ``false``，
      要往前铺得预知对方的脉冲重复序列，那是另一类技术）；
    * 方位取**干扰机自己的**方位（条数公式里没有目标，所以没有"某个目标的
      方位"可用）；
    * 坐标与极坐标**自洽**（``x = d·sinθ``、``y = −d·cosθ``）：写错一个方向，
      屏幕上那串幽灵会**整个转了 90°**，而条数与距离全都正常。
    """
    import math

    radar, engine, store, _ = _false_target_rig(FalseTargetSpec(pulse_density=0.1))
    _run(engine, 1.0)

    slant, bearing = _range_between(store, 2)             # 干扰机
    cell = eq.range_gate(1.0e-6)
    assert cell == pytest.approx(299_792_458.0 * 1.0e-6 / 2.0)

    assert radar.phantoms_seen == 100
    by_id = {c.target_id: c for c in store.contacts_of(0)}
    me = store.position_of(0)
    for k in range(radar.phantoms_seen):
        contact = by_id[phantom_id(k)]
        assert contact.phantom is True
        assert contact.range_m == pytest.approx(slant + (k + 1) * cell, rel=1e-9)
        assert contact.bearing_deg == pytest.approx(bearing)
        # 幽灵连"真实高度"都不存在 ⇒ 高度填**量测者自己的**（§9-53）
        assert contact.z == pytest.approx(me[2])
        angle = math.radians(contact.bearing_deg)
        assert contact.x == pytest.approx(me[0] + contact.range_m * math.sin(angle))
        assert contact.y == pytest.approx(me[1] - contact.range_m * math.cos(angle))


def test_the_false_target_count_is_capped_by_capacity_unambiguous_range_and_max_range() -> None:
    """★ 三个上界各管一件事，上屏幕的是**三者的最小**（而且都是算出来的，不是拍的）。

    ========================  ==================================================
    ``false_target_capacity``  雷达的槽位（参数，默认 1000）
    ``PRI/PW``          一个不模糊距离里放得下多少个分辨单元（这里 = 100）
    ``max_range_m``     探测半径——更远的幽灵这部雷达根本看不见
    ========================  ==================================================

    ``false_targets_seen``（干扰机**想造**多少）与 ``phantoms_seen``（**实际
    铺了**多少）是**两个数**，所以每一段都要同时报出来：混成一个数就再也分不
    清"干扰机没劲"与"雷达屏幕满了"。
    """
    card = FalseTargetSpec(pulse_density=0.1)             # 想造 40 万条

    # ① 容量 1000 是三个上界里**最大**的那个 ⇒ 被 PRI/PW = 100 封住
    radar, engine, _, _ = _false_target_rig(card, false_target_capacity=1_000)
    _run(engine, 1.0)
    assert radar.false_targets_seen == 400_000
    assert radar.phantoms_seen == 100

    # ② 容量压到 2 ⇒ 由它封（"想造多少"不受槽位影响：两个数各记各的）
    radar, engine, _, _ = _false_target_rig(card, false_target_capacity=2)
    _run(engine, 1.0)
    assert radar.false_targets_seen == 400_000
    assert radar.phantoms_seen == 2

    # ③ 容量放大到 1 万 ⇒ 仍是 PRI/PW 封（**不是**容量）
    radar, engine, _, _ = _false_target_rig(card, false_target_capacity=10_000)
    _run(engine, 1.0)
    assert radar.phantoms_seen == 100

    # ④ 探测半径压到比不模糊距离还小 ⇒ 由 max_range 封
    radar, engine, store, _ = _false_target_rig(
        card, false_target_capacity=10_000, max_range=11_000.0
    )
    _run(engine, 1.0)
    slant, _bearing = _range_between(store, 2)
    assert radar.phantoms_seen == int((11_000.0 - slant) / eq.range_gate(1.0e-6))
    assert radar.phantoms_seen == 4


def test_false_targets_need_the_pulse_width_and_the_pulse_repetition_frequency() -> None:
    """缺波形量 ⇒ **铺幽灵那一刻**报错，而且一次报全。

    ★ 不在 ``initialize`` 里查：有没有假目标干扰机是**运行期**才知道的（干扰机
    登记在名册上，而雷达装配时名册还是空的）。★ 也**不许取默认值**：条数公式
    在 ``PW`` / ``PRI`` 为 0 时没有意义（除零 / 无穷），而它算出来的东西会
    **直接变成航迹表里的行**——取一个"看着合理"的默认脉宽，症状就是屏幕上凭空
    多出一片幽灵，而参数表上每一项都正常。
    """
    card = FalseTargetSpec(pulse_density=0.1)

    _, engine, _, _ = _false_target_rig(card, pulse_width=0.0)
    with pytest.raises(ConfigurationError, match="pulse_width"):
        _run(engine, 1.0)

    _, engine, _, _ = _false_target_rig(card, pulse_repetition_frequency=0.0)
    with pytest.raises(ConfigurationError, match="pulse_repetition_frequency"):
        _run(engine, 1.0)

    # 两项都缺 ⇒ 报错信息里两项都在（补一个再跑一次才知道还缺什么，就是两次迭代）
    radar, engine, _, _ = _false_target_rig(
        card, pulse_width=0.0, pulse_repetition_frequency=0.0
    )
    with pytest.raises(ConfigurationError) as info:
        _run(engine, 1.0)
    assert "pulse_width" in str(info.value)
    assert "pulse_repetition_frequency" in str(info.value)
    assert radar.phantoms_seen == 0        # 报错在铺之前：屏幕上一条都没有


def test_a_real_target_inside_the_nearest_phantom_is_always_allowed() -> None:
    """★ 几何优先权与洪泛抽签各管一端。

    ``false_target_screener`` 原文："A real target that has a range from the
    sensor that is less than that of the closest false target **will be
    allowed as a track**." 它比所有假回波都先回来，雷达先看到的就是它。

    ⇒ 判据不能只看"条数超了容量没有"：真目标落在幽灵链**内侧**时**一次随机数
    都不该抽**（``jamming_blocked`` 恒 0），否则近处目标会因为"屏幕上幽灵多"
    而被随机地挡掉——那不是筛选，是掷骰子。

    外侧那一支走 ``WSF_SIMPLE_FT_EFFECT`` 的抽签式
    ``Blocked = UniformRandomDraw(0,1) > TrackCapacity / NumberFalseTargets``：
    容量 1、条数 8 ⇒ 通过概率 1/8，40 拍里必然被挡过。
    """
    card = FalseTargetSpec(quantity=8)

    # 真目标与干扰机**同格** ⇒ 它比整条幽灵链（从 R_j + ΔR 起）都近
    radar, engine, _, _ = _false_target_rig(
        card, entities=((1, 6, 0), (2, 6, 0)), false_target_capacity=1
    )
    _run(engine, 20.0)
    assert radar.phantoms_seen == 1        # 槽位只给 1 条
    assert radar.detections > 0            # 真目标确实在被看见
    assert radar.jamming_blocked == 0      # 一次都没被挡

    # 真目标挪到幽灵链**外侧**（(7,0) 比 R_j + ΔR 远）⇒ 开始抽签
    radar, engine, _, _ = _false_target_rig(
        card, entities=((1, 7, 0), (2, 6, 0)), false_target_capacity=1
    )
    _run(engine, 40.0)
    assert radar.jamming_blocked > 0
    assert radar.jamming_blocked <= radar.ticks


def test_the_phantom_contacts_are_cleared_every_beat_and_can_be_relayed() -> None:
    """幽灵是"这一拍的屏幕"，不是跟踪记忆；而且它**照样会转发**。

    ★ 撤的动作只能撤**自己铺过的那一批 ID**（``_phantoms``）：``_prune`` 只在
    ``_hits`` / ``_tracks`` 上走，幽灵不在里面，漏撤的症状是它们**永远挂在
    屏幕上**。

    ★ 转发那一支**不拦幽灵**：AFSIM 的假目标就是要污染对方的情报链，拦在本
    平台上等于把干扰的效果限制在单平台（§9-53）。所以线格式里必须有
    ``phantom`` 这一项——少一个字段 ``Contact.from_dict`` 会当场报错，而**不是**
    静静把它读成真目标。
    """
    radar, engine, store, jam = _false_target_rig(
        FalseTargetSpec(pulse_density=0.1),
        entities=((1, 6, 0), (2, 6, 0), (3, -6, 0)),
        **_INSTANT_TRACK,
    )
    _run(engine, 2.0)
    assert radar.phantoms_seen == 100
    assert sum(c.phantom for c in store.contacts_of(0)) == 100

    # -- 转发：幽灵与真航迹走同一条路 --
    payloads = [
        m.payload for m in store.track_view(0).share_tracks(3, sent_at=0, deliver_at=0)
    ]
    phantom_mail = [p for p in payloads if p["phantom"]]
    assert len(phantom_mail) == 100
    assert len(phantom_mail) < len(payloads)          # 真航迹也在同一条路上
    contact, _verdict = store.relay_contact(
        3, phantom_mail[0], sender_id=0, received_at=2_000_000
    )
    assert contact is not None and contact.phantom is True
    assert contact.is_local is False                  # 来源标记没被洗掉
    assert store.contacts_of(3)[0].phantom is True

    # -- 干扰机关机 ⇒ 下一拍一条幽灵都不剩 --
    jam.unregister(2)
    engine.tick_to(3.0)
    assert radar.phantoms_seen == 0
    assert radar.false_targets_seen == 0              # 与"还想造多少"一起归零
    assert not any(c.phantom for c in store.contacts_of(0))
    assert store.validate() == []


def test_false_targets_that_fit_the_screen_do_not_disturb_the_random_stream() -> None:
    """★ 守门人：**装得下就不抽签** ⇒ 整条检测链与"没有假目标卡"逐位相同。

    ``WSF_SIMPLE_FT_EFFECT.use_random_calculation_draw`` 的式子是
    ``Blocked = UniformRandomDraw(0,1) > TrackCapacity / NumberFalseTargets``；
    条数不超过容量时那个不等式**必然**不成立（容量/条数 ≥ 1），AFSIM 与我们
    都把它短路掉。短路不是优化而是**可复现性的一部分**：不短路的话每一拍会多抽
    一个随机数，整条随机流移位，于是"同一颗种子 ⇒ 同一场推演"在**别的**随机量
    （检测抽签）上失效——而每一拍看起来只差一点点。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    card = FalseTargetSpec(quantity=8)

    # ① 没有假目标卡（只有同一台机器的功率）——这是基准。★ ``scan_interval``
    #    必须与下面那一份**一样**：它决定"一拍转多少度"，而两条装配如果转法不同，
    #    被照射的次数就不同，比出来的差别是扫描律的差别、不是假目标的差别。
    plain, engine_a, _, _ = _ew_build(
        entities=entities, jammers=((2, {"power_w": 1.0e-9}),), seed=11,
        scan_interval=_REV_PER_BEAT, **_EW_RADAR, **_FT_WAVE,
    )
    _run(engine_a, 6.0)

    # ② 同一台机器多挂一张假目标卡，但**槽位装得下**（容量 ≫ 条数）
    radar, engine_b, store, _ = _false_target_rig(
        card, power_w=1.0e-9, entities=entities, seed=11,
        false_target_capacity=10_000,
    )
    _run(engine_b, 6.0)

    assert radar.phantoms_seen == 8                   # 幽灵真的铺上去了……
    assert radar.jamming_blocked == 0
    # ……而检测链**逐位**与基准相同（`last_pd` 用 `==`：近似相等会放过一个
    # "每一拍多抽一个随机数"的实现）
    assert radar.detections == plain.detections
    assert radar.contacts_seen == plain.contacts_seen
    assert radar.last_pd == plain.last_pd


# ===========================================================================
# 第五层：极化两端类型 / 干扰机侧遮蔽 / 欺骗链自己的功率（v0.13.31）
# ===========================================================================

#: AFSIM ``doc/receiver.rst`` 里那张 ``F_POL`` 表，**独立抄一份**。
#:
#: ★ 独立抄是刻意的：直接引用 ``equation._POLARIZATION_EFFECTS`` 去比它自己，
#: 那不是"验证"而是"照镜子"——实现改成什么，用例就跟着改成什么。这一份与
#: 实现里那一份是**两个出处**，任何一份被改坏，下面那条用例都会红。
#: 列序 = ``POLARIZATION_KINDS``。
_POL_TABLE_FROM_DOC: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("horizontal", (1.0, 0.0, 0.5, 0.5, 0.5, 0.5, 1.0)),
    ("vertical", (0.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0)),
    ("slant_45", (0.5, 0.5, 1.0, 0.0, 0.5, 0.5, 1.0)),
    ("slant_135", (0.5, 0.5, 0.0, 1.0, 0.5, 0.5, 1.0)),
    ("left_circular", (0.5, 0.5, 0.5, 0.5, 1.0, 0.0, 1.0)),
    ("right_circular", (0.5, 0.5, 0.5, 0.5, 0.0, 1.0, 1.0)),
    ("default", (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)),
)


def test_the_polarization_table_is_the_afsim_one_cell_by_cell() -> None:
    """★ ``F_POL`` 就是那张表：**七个类型、取值只有 0 / 0.5 / 1**。

    AFSIM 里它**不是**一个手填的 dB，而是"接收机主极化 × 来波极化"查出来的
    一个**比例**（``WsfEM_Rcvr::UpdatePolarizationEffects`` 与 ``receiver.rst``
    两处逐格一致）。这条用例守四件事：

    1. **逐格**与文档一致（正交 → 0、差 45° → 0.5、匹配 → 1）；
    2. **对称**——``A→B`` 必须等于 ``B→A``（互易，物理上就该如此）；
    3. ``default`` 那一**行**一**列**恒为 1——"没声明极化"不产生失配，这正是
       "没写这一项 ⇒ 旧想定数字逐字不变"的依据；
    4. 未知类型**报错**，不许静默当 1.0——静默的话，"类型拼错"与"完全匹配"
       表现成同一个现象（一分不扣），而参数表上看不出任何异常。
    """
    kinds = eq.POLARIZATION_KINDS
    assert len(kinds) == 7

    for receiver, row in _POL_TABLE_FROM_DOC:
        for transmitter, expect in zip(kinds, row, strict=True):
            assert eq.polarization_effect(receiver, transmitter) == expect, (
                receiver,
                transmitter,
            )

    for a in kinds:
        for b in kinds:
            assert eq.polarization_effect(a, b) == eq.polarization_effect(b, a)

    assert all(eq.polarization_effect("default", b) == 1.0 for b in kinds)
    assert all(eq.polarization_effect(a, "default") == 1.0 for a in kinds)

    assert {
        eq.polarization_effect(a, b) for a in kinds for b in kinds
    } == {0.0, 0.5, 1.0}

    with pytest.raises(ValueError, match="未知的极化类型"):
        eq.polarization_effect("diagonal", "vertical")


def _jammer_bearing() -> float:
    """实体 2（干扰机）相对实体 0（雷达）的方位。几何与装配无关，量一次就够。"""
    _, _, store, _ = _ew_build(entities=((1, 6, 0), (2, 6, 0)), **_EW_RADAR)
    _, bearing = _range_between(store, 2)
    return bearing


def _polar_rig(rx_type: str, tx_type: str, *, override: float = -1.0):
    """收极化 ``rx_type`` 的雷达 + 发极化 ``tx_type`` 的干扰机。

    ``override = -1`` ⇒ 干扰机**没给**显式覆盖 ⇒ 按两端类型查表（默认口径）。
    """
    return _ew_build(
        entities=((1, 6, 0), (2, 6, 0)),
        jammers=(
            (
                2,
                {
                    "power_w": 1.0,
                    "bandwidth_hz": 1.0e6,
                    "polarization": override,
                    "polarization_type": tx_type,
                },
            ),
        ),
        polarization=rx_type,
        **_EW_RADAR,
    )


def test_the_cross_polarized_jammer_is_attenuated_by_the_two_types() -> None:
    """正交极化 ⇒ 一分都进不来；差 45° ⇒ 一半；匹配 ⇒ 全份。

    ★ 守的是"**类型**真的进了方程"。三个数之间是 0 / 半个 / 一个的整倍关系，
    写反一个类型会让 J 成倍地变——而在参数表上看，两行都只是字符串。
    """
    bearing = _jammer_bearing()

    def j_for(rx_type: str, tx_type: str) -> float:
        radar, _, _, _ = _polar_rig(rx_type, tx_type)
        return radar.received_jamming_w(bearing)

    both_undeclared = j_for("default", "default")
    matched = j_for("vertical", "vertical")
    half = j_for("vertical", "slant_45")
    crossed = j_for("vertical", "horizontal")

    # 两端都没声明 ⇒ 与"两端都匹配"**逐位同值**（不扣）
    assert both_undeclared == matched
    assert matched > 0.0
    assert half == pytest.approx(matched * 0.5, rel=1e-15)   # 45° ⇒ −3.01 dB
    assert crossed == 0.0                                    # 正交 ⇒ 全挡


def test_an_explicit_polarization_override_beats_the_two_types() -> None:
    """显式覆盖（= AFSIM 的 ``polarization_effect``）压过查表。

    ★ "没给"与"给了恰好 1.0"必须是**两件事**：给了 1.0 就永远不扣，不管两端
    类型是什么；没给才去查表。用一个数兼作两义，这两种意图就再也分不开了。
    """
    bearing = _jammer_bearing()

    reference, _, _, _ = _polar_rig("vertical", "vertical")
    full = reference.received_jamming_w(bearing)

    # 两端口径正交，但显式覆盖成 1.0 ⇒ 照收全份
    forced, _, _, _ = _polar_rig("vertical", "horizontal", override=1.0)
    assert forced.received_jamming_w(bearing) == full

    # 覆盖成 0.0 ⇒ 全挡（0.0 是被当作"给了 0"，不是"没给"）
    muted, _, _, _ = _polar_rig("vertical", "vertical", override=0.0)
    assert muted.received_jamming_w(bearing) == 0.0


def test_a_jammer_behind_the_terrain_is_masked_out() -> None:
    """② 山那面的干扰机进不来：判据与受害方自己那条第 4 步是**同一个调用**。

    一起守三件事：

    * 遮挡时 J 归零，且 ``jamming_masked`` 记账（与目标的 ``geometry_blocked``
      分开计——合成一个数就分不清"真的没干扰"与"干扰被山挡了"）；
    * 把遮挡关掉，**同一套几何**下 J 就回来（说明差别真来自几何，而不是参数
      被哪一层吞了）；
    * ``antenna_height = 0`` ⇒ **不判**（"没有架高这一项" ≠ "天线贴在地面上"），
      所以这条用例必须显式写架高，否则门面根本不会被问到。

    ★ 用桩导航而不是真地形：这一层测的是"**有没有把几何接进干扰链**"，
    地形通视本身已经由第 4 步那几条用例守着（桩能把输入分得开，真地形不能）。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    jammers = ((2, {"power_w": 1.0, "bandwidth_hz": 1.0e6}),)

    nav = _BlockingNav(visible=False)
    radar, _, store, _ = _ew_build(
        entities=entities,
        jammers=jammers,
        nav=nav,
        antenna_height=_MAST_M,
        **_EW_RADAR,
    )
    _, bearing = _range_between(store, 2)
    assert nav.asked == 0 and radar.jamming_masked == 0

    assert radar.received_jamming_w(bearing) == 0.0
    assert radar.jamming_masked == 1
    assert nav.asked == 1

    nav.visible = True
    assert radar.received_jamming_w(bearing) > 0.0
    assert radar.jamming_masked == 1                 # 通视的不计数

    # 没给架高 ⇒ 不判几何：连门面都不碰（同 ``_has_line_of_sight`` 的边界）
    never = _BlockingNav(visible=False)
    blind, _, store2, _ = _ew_build(
        entities=entities, jammers=jammers, nav=never, **_EW_RADAR
    )
    _, bearing2 = _range_between(store2, 2)
    assert blind.received_jamming_w(bearing2) > 0.0
    assert never.asked == 0
    assert blind.jamming_masked == 0


def test_a_masked_jammer_lays_no_phantoms() -> None:
    """② 延伸到第 7c 步：被挡住的干扰机**连幽灵也造不出来**。

    ★ 遮蔽那一判必须在假目标之前。放在后面的话，山那面的机器照样能把屏幕
    撒满——"看不见的干扰机在造假目标"这个组合没有物理落点。同时
    ``false_targets_seen`` 也必须是 0：它记的是"这台想造多少"，而进不来的
    机器一条都不想造（记进去等于把"没进来"算成"屏幕被占了"）。
    """
    blocked, engine, _, _ = _false_target_rig(
        FalseTargetSpec(quantity=8),
        power_w=1.0e-9,
        nav=_BlockingNav(visible=False),
        antenna_height=_MAST_M,
    )
    _run(engine, 1.0)
    assert blocked.phantoms_seen == 0
    assert blocked.false_targets_seen == 0
    assert blocked.jamming_masked == 1


def test_false_targets_need_enough_arriving_power() -> None:
    """④ 假目标的**到达功率门限**：到不了门限，一个幽灵都不显形。

    ★ 门限判的是**到达功率**（W）而不是 ``J/S``：铺幽灵那条路是**每拍一次**
    的，它没有单脉冲 SNR。用 ``J/S`` 的话，同一台干扰机在"逐目标"与"逐拍"
    两个视图里会得到两个答案，而两个答案都不报错。

    ★ 默认 0 = 不判，所以这条用例得自己给门限。先量一次"这台机器在雷达输入端
    有多少瓦"，再在这条线的上下各取一档——判据钉的是**行为**（过门限 / 不过
    门限），不是某一个算出来的具体数字。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    card = FalseTargetSpec(quantity=8)

    probe, _, store, _ = _false_target_rig(card, power_w=1.0e-9, entities=entities)
    _, bearing = _range_between(store, 2)
    arrival = probe.received_jamming_w(bearing)
    assert arrival > 0.0

    # 门限比到达功率高 50% ⇒ 一条都铺不出来
    tight, engine_a, _, _ = _false_target_rig(
        card,
        power_w=1.0e-9,
        entities=entities,
        false_target_min_power=arrival * 1.5,
    )
    _run(engine_a, 1.0)
    assert tight.phantoms_seen == 0

    # 门限比到达功率低 50% ⇒ 该铺的照铺（8 条：``quantity`` 说了算）
    loose, engine_b, _, _ = _false_target_rig(
        card,
        power_w=1.0e-9,
        entities=entities,
        false_target_min_power=arrival * 0.5,
    )
    _run(engine_b, 1.0)
    assert loose.phantoms_seen == 8


def test_the_deception_chain_can_have_its_own_power() -> None:
    """① 欺骗与压制是**两条链**：压制极小、欺骗够大 ⇒ 骗得成，却不压制。

    ★ 这正是 v0.13.31 之前**表达不出来**的那句话（§9-50 结案）。判据要同时
    看两个数：

    * 并进噪声的那一份 ``power_w`` —— 给了欺骗功率之后必须**逐位不变**
      （欺骗的功率是信号路的功率，不是噪声）；
    * 偏置 ``range_bias_m`` —— 这一份才该从 0 变成卡上那个值。

    只看后者的话，"欺骗顺手把 Pd 也压下去了"这个老毛病会被漏掉；只看前者
    的话，两个变体根本分不出来。
    """
    entities = ((1, 6, 0), (2, 6, 0))
    radar, _, store, jam = _ew_build(
        entities=entities, jammers=(), scan_interval=_REV_PER_BEAT, **_EW_RADAR
    )
    slant, bearing = _range_between(store, 1)
    sn = radar.single_pulse_snr(slant, 1.0)

    # 一次标定：这台机器每瓦换来多少 ``J/S``（Jam.1 是线性的，不必重推整条链）
    jam.register(_jammer_spec(2, power_w=1.0, bandwidth_hz=1.0e6))
    j_per_watt = radar.received_jamming_w(bearing)
    jam.unregister(2)
    assert j_per_watt > 0.0
    per_watt_js = eq.jamming_to_signal(j_per_watt, radar.noise_w, sn)
    enough = eq.linear(3.0) * 2.0 / per_watt_js         # 刚好过 3 dB 门限，留一倍
    tiny = enough / 1.0e6                               # 压制链：低 60 dB

    shared = DeceptionSpec(required_j_to_s_db=3.0, range_bias_m=800.0)
    own = DeceptionSpec(required_j_to_s_db=3.0, range_bias_m=800.0, power_w=enough)

    jam.register(_jammer_spec(2, power_w=tiny, bandwidth_hz=1.0e6, deception=shared))
    before = radar.jamming_effect(bearing, sn)
    jam.unregister(2)
    jam.register(_jammer_spec(2, power_w=tiny, bandwidth_hz=1.0e6, deception=own))
    after = radar.jamming_effect(bearing, sn)

    # 不给欺骗功率 ⇒ 两链共用那一份：压制弱 ⇒ 骗不成（只有功率，没有偏置）
    assert before.power_w > 0.0
    assert before.pullers == 0 and before.range_bias_m == 0.0

    # 给了 ⇒ 门限过了、偏置有了，而并进噪声的那一份**逐位不变**
    assert after.power_w == before.power_w
    assert after.pullers == 1
    assert after.range_bias_m == pytest.approx(800.0)


def test_the_deception_power_needs_a_deception_card() -> None:
    """① 组件侧：给了欺骗链的功率、却没给任何位移量 ⇒ **装配期报错**。

    那张卡不存在，于是两个功率参数永远不会被任何地方读到——正是本项目最想
    消灭的"参数表上有这一项却没人读"。静默忽略的症状是"我调了欺骗功率，
    一点反应都没有"，而参数表上那个值明明写着。

    ★ 同一条用例也钉住默认口径：不给欺骗功率 ⇒ 卡在、而 ``power_w = 0``
    （= 与压制共用同一份），这是"旧想定数字逐字不变"的来处。
    """
    jam = JamService(EntityStore(), _Sides())
    with pytest.raises(ConfigurationError) as info:
        RfJammer(
            spec=_jam_params(
                peak_power=100.0e3, frequency=4.0e9, deception_peak_power=50.0e3
            )
        ).initialize(_JamMount(1, jam))
    assert "deception_peak_power" in str(info.value)
    assert jam.is_empty

    # 有了位移量 ⇒ 卡成立，功率落在卡上（占空比默认 1 ⇒ 峰值就是平均功率）
    with_power = RfJammer(
        spec=_jam_params(
            peak_power=100.0e3,
            frequency=4.0e9,
            deception_range_bias_m=800.0,
            deception_peak_power=50.0e3,
        )
    )
    with_power.initialize(_JamMount(1, jam))
    card = jam.specs()[0].deception
    assert card is not None
    assert card.power_w == pytest.approx(50.0e3)

    # 不给 ⇒ 卡在，而功率是 0（"与压制共用"）
    without = RfJammer(
        spec=_jam_params(
            peak_power=100.0e3, frequency=4.0e9, deception_range_bias_m=800.0
        )
    )
    without.initialize(_JamMount(2, jam))
    plain = [spec for spec in jam.specs() if spec.entity_id == 2][0].deception
    assert plain is not None and plain.power_w == 0.0
    assert [spec for spec in jam.specs() if spec.entity_id == 1][0].deception.power_w == pytest.approx(
        50.0e3
    )


# -- 照敌航迹置向 + 连续方向图（v0.13.34）-----------------------------------

def _aim_build(poses, sides, names=None, contacts=()):
    """建一个**只有实体与航迹**的最小场景：``(store, 桩)``。

    实体位置直接写世界坐标，格归属给一个无关紧要的格——``_eligible`` 用的是
    ``position_of``（真值），不是格。

    ``contacts`` 是 ``(owner_id, target_id, x, y, z, iff, phantom)``，**写进
    owner 自己的航迹表**——这正是 v0.13.34 那套"照敌航迹置向"的数据来源。
    """
    store = EntityStore()
    for entity_id, (x, y, z) in poses.items():
        store.add(entity_id, LocalCellRef(1, 0, Axial(0, 0)))
        store.set_pose(entity_id, x, y, z)
    for owner, target, x, y, z, iff, phantom in contacts:
        store.add_contact(owner, target, x=x, y=y, z=z, iff=iff, phantom=phantom)
    return store, _Sides(sides, names)


def test_the_antenna_pattern_is_flat_without_a_beam_width() -> None:
    """不写波束宽度 ⇒ **各向同性**：任何偏角都返 1.0，逐位相同。

    这是"默认档零回归"最直接的一条证据：它不是"角度算出来恰好是 1"，而是
    这一支根本不看角度。
    """
    sidelobe = eq.linear(DEFAULT_SIDELOBE_LEVEL_DB)
    for off in (0.0, 1.0, 90.0, 180.0, 359.9):
        assert eq.antenna_pattern_factor(off, 0.0, sidelobe) == 1.0
    # 负的波束宽度也按"没写"处理（参数层已有最小值约束，这里是第二道）
    assert eq.antenna_pattern_factor(30.0, -5.0, sidelobe) == 1.0


def test_the_antenna_pattern_drops_three_db_at_the_lobe_edge() -> None:
    """主瓣边缘**恰好**是半功率点（0.5 = −3.0103 dB），轴心是 1.0。

    ★ 这条钉住的是 cos² 里那个 ``π/4``：写成 ``π/2`` 时边缘会 ``cos(π/2)²``
    ⇒ 归零（还留一个 ``3.7e-33`` 的浮点尾巴），而"边缘 3 dB"那句话就不再成立
    ——症状是"波束中心照常、稍微偏一点就一文不值"，而参数表一切正常。
    """
    sidelobe = eq.linear(DEFAULT_SIDELOBE_LEVEL_DB)
    assert eq.antenna_pattern_factor(0.0, 10.0, sidelobe) == 1.0
    edge = eq.antenna_pattern_factor(5.0, 10.0, sidelobe)
    assert edge == pytest.approx(0.5)
    assert eq.db(edge) == pytest.approx(-3.0103, abs=1e-3)
    # 半偏角处是 cos²(π/8) ≈ 0.8536，介于轴心与边缘之间
    mid = eq.antenna_pattern_factor(2.5, 10.0, sidelobe)
    assert 0.5 < mid < 1.0


def test_outside_the_main_lobe_the_pattern_is_the_flat_sidelobe() -> None:
    """主瓣外**平坦取 −13 dB**（⟨最省⟩）——不编副瓣起伏。

    ★ 也要钉住那个**跳变**：边缘 0.5 到主瓣外 0.0501 之间没有过渡段。这是
    口径的固有代价（任何过渡段的形状都是猜的），如实测出来，免得将来有人
    把它当成 bug 顺手"修"成一条曲线。
    """
    sidelobe = eq.linear(DEFAULT_SIDELOBE_LEVEL_DB)
    for off in (5.0001, 30.0, 90.0, 180.0):
        assert eq.antenna_pattern_factor(off, 10.0, sidelobe) == sidelobe
    assert eq.db(sidelobe) == pytest.approx(-13.0)
    # 边缘 vs 刚过边缘：**不连续**
    assert eq.antenna_pattern_factor(5.0, 10.0, sidelobe) == pytest.approx(0.5)
    assert eq.antenna_pattern_factor(5.0001, 10.0, sidelobe) == pytest.approx(
        sidelobe
    )


def test_aiming_needs_an_enemy_track_in_the_jammers_own_table() -> None:
    """**本机航迹表里没有敌航迹 ⇒ 不辐射**（用户口径："看见才打"）。

    三条并列：表为空、只有友航迹、只有幽灵航迹——三种都**一个都压不到**。
    幽灵不算：那是干扰自己造 / 别人造的假目标，照着它打是自欺。
    """
    poses = {1: (200.0, 0.0, 0.0), 2: (2000.0, 0.0, 0.0)}
    sides = {1: "RED", 2: "BLUE"}
    spec = _jammer_spec(2, aim_at="nearest", beam_width_deg=10.0)

    for contacts in (
        (),
        ((2, 3, 0.0, 1000.0, 0.0, IFF_FRIEND, False),),
        ((2, 4, 0.0, 1000.0, 0.0, IFF_FOE, True),),
    ):
        store, sides_stub = _aim_build(poses, sides, contacts=contacts)
        jam = JamService(store, sides_stub, tracks_of=store.contacts_of)
        jam.register(spec)
        assert jam.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0) == ()
        assert jam.aim_failed == 1 and jam.aimed == 0


def test_without_an_injected_track_source_aiming_radiates_nothing() -> None:
    """**没注入取航迹的函数**（正式装配的现状）⇒ 写了 ``aim_at`` 也不辐射。

    用户裁定：这条路子**本次只在测试里启用**，正式版不启用。所以正式装配下
    一个写了 ``aim_at`` 的干扰机是**哑的**——这是"没有航迹就不该照着打"的
    正确表现，而不是缺陷。
    """
    poses = {1: (200.0, 0.0, 0.0), 2: (2000.0, 0.0, 0.0)}
    store, sides_stub = _aim_build(
        poses,
        {1: "RED", 2: "BLUE"},
        contacts=((2, 3, 0.0, 1000.0, 0.0, IFF_FOE, False),),
    )
    jam = JamService(store, sides_stub)          # 不传 tracks_of
    jam.register(_jammer_spec(2, aim_at="nearest", beam_width_deg=10.0))
    assert jam.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0) == ()
    assert jam.aim_failed == 1


def test_nearest_picks_the_closest_enemy_track() -> None:
    """``aim_at="nearest"`` ⇒ 挑斜距**最近**的那条敌航迹（用户原话）。

    本机在原点附近，敌 A 在北 1 km、敌 B 在东 5 km ⇒ 选 A（正北）。受害方在
    正东方向 ⇒ 与天线指向夹角 90°。
    """
    poses = {1: (0.0, 0.0, 0.0), 2: (500.0, 0.0, 0.0)}
    store, sides_stub = _aim_build(
        poses,
        {1: "RED", 2: "BLUE"},
        contacts=(
            (2, 10, 500.0, 1000.0, 0.0, IFF_FOE, False),   # 北 1 km（近）
            (2, 11, 5500.0, 0.0, 0.0, IFF_FOE, False),      # 东 5 km（远）
        ),
    )
    jam = JamService(store, sides_stub, tracks_of=store.contacts_of)
    jam.register(_jammer_spec(2, aim_at="nearest", beam_width_deg=0.0))
    threats = jam.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0)
    assert len(threats) == 1
    # 天线指北（对 10 号：dx=0, dy=+1000 ⇒ 0°），受害方在正东 ⇒ 夹角 90°
    assert threats[0].antenna_off_deg == pytest.approx(90.0)


def test_aim_at_accepts_both_an_entity_name_and_an_id() -> None:
    """⟨容错⟩：``aim_at`` **实体名与实体 ID 都认**，两条给出同一结果。"""
    poses = {1: (0.0, 0.0, 0.0), 2: (500.0, 0.0, 0.0)}
    contacts = (
        (2, 10, 500.0, 1000.0, 0.0, IFF_FOE, False),
        (2, 11, 5500.0, 0.0, 0.0, IFF_FOE, False),
    )
    store, sides_stub = _aim_build(
        poses, {1: "RED", 2: "BLUE"}, names={"foe_east": 11}, contacts=contacts
    )
    for wanted in ("11", "foe_east"):
        jam = JamService(store, sides_stub, tracks_of=store.contacts_of)
        jam.register(_jammer_spec(2, aim_at=wanted, beam_width_deg=0.0))
        threats = jam.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0)
        assert len(threats) == 1
        # 干扰机在 (500,0)，敌 11 在 (5500,0) ⇒ 天线指**正东**（90°）；
        # 受害方在 (0,0) ⇒ 从干扰机看它在**正西**（270°）⇒ 夹角 180°。
        # 两条给出同一个数，正是"名与 ID 都认"的证据。
        assert threats[0].antenna_off_deg == pytest.approx(180.0)
        assert jam.aimed == 1 and jam.aim_failed == 0


def test_an_unknown_aim_target_radiates_nothing() -> None:
    """写了挑不到的 ``aim_at`` ⇒ **不辐射**，且计一次失败（不静默退化）。"""
    poses = {1: (0.0, 0.0, 0.0), 2: (500.0, 0.0, 0.0)}
    store, sides_stub = _aim_build(
        poses,
        {1: "RED", 2: "BLUE"},
        contacts=((2, 10, 500.0, 1000.0, 0.0, IFF_FOE, False),),
    )
    jam = JamService(store, sides_stub, tracks_of=store.contacts_of)
    jam.register(_jammer_spec(2, aim_at="nobody", beam_width_deg=0.0))
    assert jam.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0) == ()
    assert jam.aim_failed == 1 and jam.aimed == 0


def test_the_jammer_beam_blocks_a_victim_it_is_not_pointing_at() -> None:
    """**天线主瓣盖不住受害方 ⇒ 这门对它没贡献**（第四道筛）。

    干扰机天线指着东边的敌航迹，受害方在正北 ⇒ 夹角 90° 远大于主瓣半宽 5°。
    加宽到 360° ⇒ 主瓣包住全场，同样几何下就够得着了。
    """
    poses = {1: (0.0, 5000.0, 0.0), 2: (500.0, 0.0, 0.0)}   # 受害方在正北 5km
    store, sides_stub = _aim_build(
        poses,
        {1: "RED", 2: "BLUE"},
        contacts=((2, 11, 5500.0, 0.0, 0.0, IFF_FOE, False),),  # 敌在东 5km
    )

    narrow = JamService(store, sides_stub, tracks_of=store.contacts_of)
    narrow.register(_jammer_spec(2, aim_at="11", beam_width_deg=10.0))
    assert narrow.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0) == ()
    assert narrow.aimed == 1        # 瞄上了，只是盖不住

    wide = JamService(store, sides_stub, tracks_of=store.contacts_of)
    wide.register(_jammer_spec(2, aim_at="11", beam_width_deg=360.0))
    threats = wide.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0)
    assert len(threats) == 1
    assert threats[0].beam_width_deg == pytest.approx(360.0)


def test_a_jammer_without_aim_at_keeps_the_old_isotropic_behaviour() -> None:
    """不写 ``aim_at`` ⇒ **逐字旧逻辑**：不查航迹、不看朝向，``antenna_off`` 为 None。

    ★ 这条是"老想定零回归"的守门员：``antenna_off_deg is None`` ⇒ 受害方那侧
    的方向图因子恒 1.0 ⇒ J 与 v0.13.33 逐位相同。即使 ``tracks_of`` 已注入、
    表里有敌航迹，也不该被这套新逻辑碰到。
    """
    poses = {1: (0.0, 0.0, 0.0), 2: (500.0, 0.0, 0.0)}
    store, sides_stub = _aim_build(
        poses,
        {1: "RED", 2: "BLUE"},
        contacts=((2, 10, 500.0, 1000.0, 0.0, IFF_FOE, False),),
    )
    for spec in (
        _jammer_spec(2),                                   # 两个都没写
        _jammer_spec(2, beam_width_deg=10.0),              # 只写宽度没写置向
    ):
        jam = JamService(store, sides_stub, tracks_of=store.contacts_of)
        jam.register(spec)
        threats = jam.threats(1, beam_bearing_deg=0.0, main_lobe_deg=2.0)
        assert len(threats) == 1
        assert threats[0].antenna_off_deg is None
        assert threats[0].beam_width_deg == spec.beam_width_deg
        assert jam.aimed == 0 and jam.aim_failed == 0   # 一次航迹都没查
