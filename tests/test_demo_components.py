"""演示雷达（``tools/demo_components.py``）的扫描律测试。

``tools/`` 不在包的搜索路径上，所以按文件加载——与 ``tests/test_library.py``
加载 ``tools/model_library.py`` 是同一手法。

这里测的是**扫描律本身**：相位怎么推进、账目闭不闭合、方位门拦不拦得住、
三种扫描模式各走各的路。用桩 mount / 桩引擎把它从地图与引擎里摘出来，跑的是
纯算术：量纲与参数解析由 ``test_params`` / ``test_type_registry`` 覆盖，端到端
由 ``python tools/run_scenario.py scenarios/patrol.txt`` 覆盖（实测：一圈 2 s、
1 s 一拍 ⇒ 120 s 正好 60 圈，扇扫 120° 朝 90° ⇒ 每拍都把整片视场扫过一遍）。
"""

from __future__ import annotations

import importlib.util
import random
from pathlib import Path

import pytest

from milsim.errors import ConfigurationError
from milsim.services.map import Axial, LocalFrame
from milsim.services.store import EntityStore, LocalCellRef
from milsim.services.type_registry import default_registry

_TOOL = Path(__file__).resolve().parents[1] / "tools" / "demo_components.py"

#: 网格边长（米）。
_SIZE = 1000.0

#: 桩 mount 报出的相邻格心距 = 边长 × √3（pointy-top）。组件自己不做这个换算。
_SPAN_M = _SIZE * 3 ** 0.5

#: **"一次命中即建航"**（v0.13.39）。
#:
#: 生产默认自 v0.13.39 起是 ``3/5 建、2/5 维持``（雷达常规：建航要严、维持要松）。
#: 但**这个文件量的是扫描律，不是 M/N**：跑一拍就要问"照到了没有"，
#: 拿生产默认的话每一拍都在等 5 取 3，量到的立刻从"扫描律"变成"建航律"。
#:
#: ⇒ 需要"一拍见航迹"的用例**显式**带上它。**刻意不写进 ``_spec`` 的默认值**：
#: 那等于让测试自己藏起产品口径，产品默认改了这里也不会响 —— 而"改默认值要
#: 惊动谁"这件事本身是有信息量的，不该被测试悄悄盖掉。
_INSTANT_TRACK = {
    "hits_to_establish": 1,
    "establish_window": 1,
    "hits_to_maintain": 1,
    "maintain_window": 1,
}


def _load_radar_class():
    """按文件加载演示雷达类，**并把全局注册表恢复原样**。

    这个模块在 import 时就往全局表登记 ``HEX_SEARCH_RADAR``；不还原的话，
    "哪个测试文件先跑"会决定别的测试会不会撞上 ``DuplicateComponentError``。
    """
    registry = default_registry()
    before = registry.get("HEX_SEARCH_RADAR")
    spec = importlib.util.spec_from_file_location("radar_under_test", _TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if before is None:
        registry._classes.pop("HEX_SEARCH_RADAR", None)          # noqa: SLF001
    else:
        registry.register("HEX_SEARCH_RADAR", before, override=True)
    return module.HexSearchRadar


HexSearchRadar = _load_radar_class()


class _Spec(dict):
    """解析完的组件参数：**基本单位**（米 / 微秒 / 度）。"""

    slot = "sensor"
    type_name = "HEX_SEARCH_RADAR"


def _spec(**overrides) -> _Spec:
    """按组件的 ``PARAMS`` 默认值造一份参数，再盖掉用例要给的那几个。

    手抄一张默认值表会随每次加参数而腐烂，而且**腐烂时不报错**：新参数
    悄悄取到 ``KeyError`` 之前，那些用例一直绿着。从 ``PARAMS`` 取就永远同步
    ——这也是"组件的参数表只有一处出处"在测试侧的体现。
    """
    values = {name: item.default for name, item in HexSearchRadar.PARAMS.items()}
    values.update(overrides)
    return _Spec(values)


class _Engine:
    """只提供 ``now`` 与 ``schedule_recurring``。"""

    def __init__(self) -> None:
        self.now = 0
        self.beat = None

    def schedule_recurring(self, delay, interval, fn, priority=0):
        self.beat = (interval, fn)
        return None

    def tick_to(self, seconds: float) -> None:
        """把时钟拨到 ``seconds``，然后敲一拍（相当于引擎推进一步）。"""
        self.now = int(round(seconds * 1_000_000))
        interval, fn = self.beat
        fn(self, None)


class _Mount:
    """装配期上下文的桩：只留雷达真正用到的成员。

    v0.13.25 起雷达多了两个门面依赖：``nav``（第 4 步的几何判决）与
    ``streams().sensor``（第 7 步的检测抽样）。两者都给"缺席"的默认值——
    ``nav = None`` 就是"没有逐格精度"（与 ``cell_span_m() == 0`` 同源），
    抽样给一个定种子。量纲与参数解析由 ``test_params`` / ``test_type_registry``
    覆盖，几何与抽样本身由 ``test_percep`` 覆盖，这里测的**只有扫描律**。

    v0.13.30 起又多一个 ``jam_service()``（第 6 步的干扰项，§5.14）：
    与 ``nav`` 一样给缺席默认值。**它是方法而不是属性**，所以这里必须真的
    提供它——``MountContext`` 上有这个方法，桩漏了会是一个与扫描律毫无
    关系的 AttributeError。这个文件测的只有扫描律，干扰一律为 0。
    """

    def __init__(self, engine, store, entity_id, now, seed=0):
        self.engine = engine
        self.store = store
        self.entity_id = entity_id
        self.now = now
        self.nav = None
        self._streams = _Streams(seed)

    def jam_service(self):
        return None

    def sensor_view(self):
        return self.store.sensor_view(self.entity_id)

    def cell_span_m(self):
        return _SPAN_M

    def every(self, interval_us, fn, priority=0):
        return self.engine.schedule_recurring(0, interval_us, fn, priority)

    def streams(self):
        return self._streams

    def target_platform_param(self, entity_id, name, default=None):
        return default


class _Streams:
    """``mount.streams()`` 的桩：``sensor`` 是**属性**（与 ``EntityStreams`` 同形）。"""

    def __init__(self, seed: int) -> None:
        self._random = random.Random(seed)

    @property
    def sensor(self):
        return self._random


class _Certain(HexSearchRadar):
    """检测概率钉成 1 的雷达——**扫描律的用例不该被抽样搅进来**。

    第 7 步接上以后，"照到"与"测到"是两件事：默认参数下 10.4 km 处的 Pd
    约 0.9997，跑 300 拍就有百分之一二的概率漏一次，于是
    "某方位每圈被照到一次"这种用例会**偶发**地少一条。钉住 Pd 之后，
    这一族用例量到的仍然是 v0.13.24 那套语义（照到即建航）。
    """

    def detection_probability(self, range_m, m2=None, *, jammer_noise_w=0.0):
        return 1.0


def _build(
    *,
    cls=_Certain,
    period_s: float,
    step_s: float,
    detect_range_m: float = 100_000.0,
    entities=(),
    fov_deg: float = 360.0,
    scan_center_deg: float = 0.0,
    beam_width_deg: float = 0.0,
    **params,
):
    """造一部雷达 + 一个装着目标的存储。

    ``entities`` 是 ``(entity_id, col, row)`` 三元组。坐标经 ``LocalFrame``
    换成世界位置写进去——**位置与格必须自洽**，否则 ``query_sector``（按格判）
    与 ``bearing_to``（按真实位置判）会互相打架，测出来的东西没有意义。

    ★ **六个轴向格心正好落在 30° 的整数倍上**，所以方位用例都挑轴向格：
    ``(6,0)`` 正东 90°、``(0,6)`` 150°、``(-6,6)`` 210°、``(-6,0)`` 270°、
    ``(0,-6)`` 330°、``(6,-6)`` 30°（都在六边距离 6 的那一环上，约 10.4 km）。
    弧是**半开**的，所以用例避开弧的两个端点（压端点的那一格归谁要看约定，
    测它是在测误差而不是测行为）。

    ``fov_deg`` 默认 360（圆周），与组件的默认值一致——**三种扫描模式在这里
    分开测**，别让"圆周的那几条"偷偷跑在扇扫上。

    ``cls`` 默认 ``_Certain``（检测概率恒 1）：这一族用例量的是**扫描律**，
    第 7 步的抽样会把"照到"变成"有时照到"。要看探测链本身请用 ``test_percep``。

    ``**params`` 透传给 :func:`_spec`，用于覆盖扫描律以外的组件参数
    （当前唯一用户是 :data:`_INSTANT_TRACK`——见它的说明）。
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

    spec = _spec(
        detect_range=detect_range_m,
        scan_interval=int(round(period_s * 1_000_000)),
        beam_step=int(round(step_s * 1_000_000)),
        fov=fov_deg,
        scan_center=scan_center_deg,
        beam_width=beam_width_deg,
        **params,
    )
    radar = cls(spec=spec)
    radar.initialize(_Mount(engine, store, 0, engine.now))
    return radar, engine, store


def _run(engine, seconds: float, step_s: float = 1.0) -> None:
    """从 t=0 敲到 ``seconds``（两端都含）。"""
    beat = 0
    while beat * step_s <= seconds + 1e-9:
        engine.tick_to(beat * step_s)
        beat += 1


def _seen(radar) -> list[int]:
    return [contact.target_id for contact in radar.view.my_contacts()]


# ---------------------------------------------------------------------------
# 相位推进
# ---------------------------------------------------------------------------

def test_first_beat_only_aligns_the_clock() -> None:
    """装配时刻那一拍不推进相位——与机动件"第一帧只走时钟不挪窝"同一条约定。"""
    radar, engine, _ = _build(period_s=2.0, step_s=1.0)
    engine.tick_to(0.0)
    assert radar.ticks == 0
    assert radar.phase_deg == pytest.approx(0.0)
    assert radar.swept_deg == pytest.approx(0.0)


def test_phase_advances_by_the_rotation_period() -> None:
    """一圈 2 s、1 s 一拍 ⇒ 每拍 180°，120 s 正好 60 圈。"""
    radar, engine, _ = _build(period_s=2.0, step_s=1.0)
    _run(engine, 120)
    assert radar.ticks == 120
    assert radar.sweeps == pytest.approx(60.0)


def test_slow_radar_does_not_round_up_to_one_sweep_per_beat() -> None:
    """6 rpm（一圈 10 s）在 1 s 节拍下每拍只转 36°，**不是每拍一整圈**。

    这正是"相位当状态"要解决的那个问题：``max(1, round(N_scans))`` 会把
    6 rpm 当成 60 rpm 用，某方位一秒钟的照射次数被放大 10 倍。
    """
    radar, engine, _ = _build(period_s=10.0, step_s=1.0)
    engine.tick_to(1.0)
    assert radar.phase_deg == pytest.approx(36.0)      # 不是 360°
    assert radar.sweeps == pytest.approx(0.1)


def test_phase_is_state_not_reset_every_beat() -> None:
    """相位跨拍保留：两拍下来转过 2Δ，而不是"每拍都从正北开始扫"。"""
    radar, engine, _ = _build(period_s=4.0, step_s=1.0)
    engine.tick_to(1.0)
    assert radar.phase_deg == pytest.approx(90.0)
    engine.tick_to(2.0)
    assert radar.phase_deg == pytest.approx(180.0)


def test_a_delayed_beat_still_turns_the_full_angle() -> None:
    """引擎推迟了事件也照样转够——相位按**真实经过时间**推进。

    按名义步长算的话，这次 3 s 的间隔只会转 1 s 的量，落后永远补不回来。
    """
    radar, engine, _ = _build(period_s=4.0, step_s=1.0)
    engine.tick_to(3.0)
    assert radar.swept_deg == pytest.approx(270.0)


def test_a_beat_longer_than_a_rotation_covers_the_whole_circle() -> None:
    """一拍就转完一圈以上 ⇒ 整盘都照过了，正西的目标也躲不掉。"""
    radar, engine, _ = _build(
        period_s=1.0, step_s=1.0, entities=((1, -6, 0),), **_INSTANT_TRACK
    )
    engine.tick_to(1.0)
    assert radar.phase_deg == pytest.approx(0.0)          # 整圈回到 0
    assert _seen(radar) == [1]


# ---------------------------------------------------------------------------
# 方位门与距离门
# ---------------------------------------------------------------------------

def test_a_target_in_the_swept_arc_is_reported_with_its_bearing() -> None:
    """这一拍扫过 [90°, 180°)，正东（90°）的目标在起点上，航迹带真实方位。"""
    radar, engine, _ = _build(
        period_s=4.0, step_s=1.0, entities=((1, 6, 0),), **_INSTANT_TRACK
    )
    engine.tick_to(2.0)
    contacts = radar.view.my_contacts()
    assert [contact.target_id for contact in contacts] == [1]
    assert contacts[0].bearing_deg == pytest.approx(90.0)


def test_target_outside_the_swept_arc_is_not_reported() -> None:
    """正西（270°）的目标要等天线转到那一侧才被照到。

    弧是**半开区间**：``[180°, 270°)`` 不含 270°，``[270°, 360°)`` 才含。
    闭区间的话它会被相邻两帧各算一次，而天线在那一刻只经过一次。
    """
    radar, engine, _ = _build(
        period_s=4.0, step_s=1.0, entities=((1, -6, 0),), **_INSTANT_TRACK
    )
    for beat in (1.0, 2.0, 3.0):
        engine.tick_to(beat)
        assert _seen(radar) == [], f"第 {beat:g} 拍不该照到正西的目标"
    engine.tick_to(4.0)
    assert _seen(radar) == [1]


def test_target_beyond_detect_range_is_not_reported() -> None:
    """格在扇区里但超出探测距离，照样不报。"""
    radar, engine, _ = _build(
        period_s=4.0, step_s=1.0, detect_range_m=2_000.0, entities=((1, 6, 0),)
    )
    engine.tick_to(1.0)
    assert _seen(radar) == []


def test_a_target_is_illuminated_once_per_rotation() -> None:
    """正东的目标每**一圈**被照到一次——与一拍转多少度无关。

    一圈 10 s、一拍 1 s（每拍 36°）时，300 s 应该正好照到 30 次。
    这就是"漏扫"与"漏检"分得开的判据：没扫到的那几拍里，目标不在航迹表里
    并不代表探测失败。
    """
    radar, engine, _ = _build(
        period_s=10.0, step_s=1.0, entities=((1, 6, 0),), **_INSTANT_TRACK
    )
    _run(engine, 300)
    assert len(radar.contacts_seen) == 30
    assert set(radar.contacts_seen) == {1}


def test_a_target_on_a_frame_boundary_is_still_illuminated_once() -> None:
    """恰好落在帧边界上的方位只算一次。

    一圈 4 s、一拍 1 s ⇒ 每拍 90°，边界正好压在 90° 上（正东那个目标）。
    闭区间会把这一帧的终点与下一帧的起点各算一次，一圈数出两次来。
    """
    radar, engine, _ = _build(
        period_s=4.0, step_s=1.0, entities=((1, 6, 0),), **_INSTANT_TRACK
    )
    _run(engine, 16)                                     # 4 圈
    assert len(radar.contacts_seen) == 4


# ---------------------------------------------------------------------------
# 扇区扫描（0 < fov < 360）：相位在视场里往复，**到界反向**
# ---------------------------------------------------------------------------

def test_sector_scan_turns_around_at_the_edge() -> None:
    """视场 90°、一圈 6 s（60°/拍）：到 +45° 边界折返，而不是继续转出去。

    一拍 60° 要分两段记：先走满 45° 碰到边界，折返后再走剩下的 15°——
    所以第 1 拍停在 **+30°、方向为负**。第 3 拍走到 −45° 再折返，停在 0°。
    """
    radar, engine, _ = _build(period_s=6.0, step_s=1.0, fov_deg=90.0)

    engine.tick_to(1.0)
    assert radar.phase_deg == pytest.approx(30.0)
    assert radar.spin == -1

    engine.tick_to(2.0)
    assert radar.phase_deg == pytest.approx(-30.0)
    assert radar.spin == -1

    engine.tick_to(3.0)
    assert radar.phase_deg == pytest.approx(0.0)
    assert radar.spin == 1


def test_sector_scan_never_leaves_its_field() -> None:
    """跑 200 拍，相位始终在 ±fov/2 之内。

    这就是"扇扫不能当圆周算"的可观测形式：``mod 360`` 的写法会让相位一路
    转出去，越过视场边界之后照到的都是雷达物理上到不了的方位。
    """
    radar, engine, _ = _build(period_s=6.0, step_s=1.0, fov_deg=90.0)
    for beat in range(1, 201):
        engine.tick_to(float(beat))
        assert -45.0 - 1e-9 <= radar.phase_deg <= 45.0 + 1e-9, (
            f"第 {beat} 拍相位 {radar.phase_deg:.3f}° 跑出了视场 ±45°"
        )


def test_sector_scan_covers_the_whole_field_when_a_beat_is_long_enough() -> None:
    """一拍转过的角度 ≥ 视场宽 ⇒ 两端都到过，本拍覆盖的是**整片视场**。

    视场 120°、一圈 2 s（180°/拍）就是这个情形——扇扫速率远高于节拍时必然
    发生，不是特例。目标在视场中心 90°：视场是 [30°, 150°)（半开）。
    """
    radar, engine, _ = _build(
        period_s=2.0,
        step_s=1.0,
        fov_deg=120.0,
        scan_center_deg=90.0,
        entities=((1, 6, 0), (2, -6, 6)),
        **_INSTANT_TRACK,
    )
    engine.tick_to(1.0)
    assert _seen(radar) == [1]                    # 90° 在内、210° 在外


def test_sector_scan_never_reports_a_bearing_it_cannot_reach() -> None:
    """★ 这条是"扇扫被当成圆周"那个**静默错误**的守门人。

    视场 120° 朝正东（中心 90°）⇒ 雷达物理上照不到正西（270°）。圆周的写法
    会把相位一路转到 270° 并在那边报出目标——而**航迹表看上去完全正常**。
    同时它每拍都把整片视场扫过一遍，所以正东那个目标每拍都被照到。
    """
    radar, engine, _ = _build(
        period_s=2.0,
        step_s=1.0,
        fov_deg=120.0,
        scan_center_deg=90.0,
        entities=((1, 6, 0), (2, -6, 0)),
        **_INSTANT_TRACK,
    )
    _run(engine, 600)
    assert set(radar.contacts_seen) == {1}
    assert len(radar.contacts_seen) == 600        # 600 拍，每拍一次
    assert radar.sweeps == pytest.approx(300.0)   # 累计转角 600×180° ÷ 360°


def test_sector_scan_takes_its_center_from_scan_center() -> None:
    """同一个 60° 视场，中心挪到 330° ⇒ 照北偏西那个目标，照不到正东的。"""
    radar, engine, _ = _build(
        period_s=6.0,
        step_s=1.0,
        fov_deg=60.0,
        scan_center_deg=330.0,
        entities=((1, 0, -6), (2, 6, 0)),
        **_INSTANT_TRACK,
    )
    engine.tick_to(1.0)
    assert _seen(radar) == [1]                    # 330° 在 [300°, 360°) 里
    assert radar.mode == "扇扫 60° 朝 330°"


def test_a_folded_beat_covers_two_arcs_without_double_counting() -> None:
    """一拍里折返 ⇒ 弧是**两段**，而且它们**叠着**（重叠 15°）。

    视场 90°、一拍 60°：先走 (0°, 45°]，掉头再走 [30°, 45°)。正东 30° 那个
    目标两段都罩得住 ⇒ 索引两段都把它交出来 ⇒ **去重必须生效**（否则它会被
    记两次，`contacts_seen` 里出现两个 1）。下一拍天线扫向另一边，换成 330°。
    """
    radar, engine, _ = _build(
        period_s=6.0,
        step_s=1.0,
        fov_deg=90.0,
        entities=((1, 6, -6), (2, 0, -6)),
        **_INSTANT_TRACK,
    )
    engine.tick_to(1.0)
    assert radar.contacts_seen == [1]             # 两段弧叠着，仍只记一次

    engine.tick_to(2.0)
    assert radar.contacts_seen == [1, 2]          # 第 2 拍扫向 330° 那一侧


# ---------------------------------------------------------------------------
# 固定指向（fov == 0）：相位钉住，波束宽度说了算
# ---------------------------------------------------------------------------

def test_stare_mode_pins_the_antenna_to_the_bearing() -> None:
    """fov = 0 ⇒ 天线不转：相位一直是 0、``swept_deg`` 一直是 0。

    它靠 ``beam_width`` 决定看得见什么：20° 的波束朝 90° ⇒ [80°, 100°)。
    """
    radar, engine, _ = _build(
        period_s=10.0,
        step_s=1.0,
        fov_deg=0.0,
        scan_center_deg=90.0,
        beam_width_deg=20.0,
        entities=((1, 6, 0), (2, 0, -6)),
        **_INSTANT_TRACK,
    )
    assert radar.mode == "固定指向 90°"
    for beat in range(1, 6):
        engine.tick_to(float(beat))
        assert radar.phase_deg == pytest.approx(0.0)
        assert _seen(radar) == [1]                # 90° 在波束里、330° 不在
    assert radar.swept_deg == pytest.approx(0.0)
    assert radar.sweeps == pytest.approx(0.0)


def test_stare_mode_needs_a_beam_width() -> None:
    """固定指向 + 零宽波束是**错配置**，当场报错而不是跑出一条空航迹。

    零宽波束下"照到"退化成一条没有面积的射线，命中要靠目标中心恰好落在
    相位上——症状是"一条航迹都没有"，而参数表看起来一个都没错。
    """
    with pytest.raises(ConfigurationError) as info:
        _build(period_s=10.0, step_s=1.0, fov_deg=0.0)
    assert "beam_width" in str(info.value)


# ---------------------------------------------------------------------------
# 波束宽度：一次照射的瞬时张角（与 fov 是两件事）
# ---------------------------------------------------------------------------

def test_beam_width_widens_what_one_beat_lights_up() -> None:
    """波束有宽度时，本拍照到的是弧的两端各外扩半个波束。

    一圈 8 s、一拍 1 s ⇒ 每拍 45°，边界压在 90° 上（正东那个目标）。零宽时
    它属于第 3 拍 ``[90°, 135°)``；波束 20° 时第 2 拍 ``[45°, 90°)`` 外扩成
    ``[35°, 100°)`` 就已经把它罩住了。
    """
    zero, engine_a, _ = _build(period_s=8.0, step_s=1.0, entities=((1, 6, 0),))
    engine_a.tick_to(2.0)
    assert _seen(zero) == []                       # 半开：90° 不在 [45°, 90°) 里

    wide, engine_b, _ = _build(
        period_s=8.0, step_s=1.0, beam_width_deg=20.0, entities=((1, 6, 0),),
        **_INSTANT_TRACK,
    )
    engine_b.tick_to(2.0)
    assert _seen(wide) == [1]


def test_mode_describes_the_scan_law() -> None:
    """``mode`` 是给想定输出看的一句话，三种模式各一个说法。"""
    assert _build(period_s=4.0, step_s=1.0)[0].mode == "圆周"
    assert (
        _build(period_s=6.0, step_s=1.0, fov_deg=90.0, scan_center_deg=45.0)[0].mode
        == "扇扫 90° 朝 45°"
    )
    assert (
        _build(
            period_s=10.0, step_s=1.0, fov_deg=0.0,
            scan_center_deg=180.0, beam_width_deg=5.0,
        )[0].mode
        == "固定指向 180°"
    )
