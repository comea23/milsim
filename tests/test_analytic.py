"""解析弹道机动件测试：球面弹道解算 + 开普勒时间律（§5.8）。

这一组钉三类东西：

- **物理锚点**：Φ=90°、弹道高点 1200 km 的解必须给出 7.2 km/s 级的发射
  速度与 31 min 级的全程——那是真实洲际弹道的量级，也是平面模型
  给不出的数（平面要 9.9 km/s）。公式与实现**分开写**：测试里按模块头
  的闭式独立重算一遍，实现错了这里能看见。
- **几何口径**：落点精确等于目标平面坐标（投影在两边抵消）、
  存储速度是水平分量（地面弧长速率）、垂向速率是径向速度。
- **防呆校验**：没目标、目标重合、射程超限、高点不高——全都报错
  而不是飞出一条静默歪掉的弹道。名字写错**装配时**就报（initialize 查
  存在性）；其余发生在**第一次推进**的解算里（射程与高点要在拿到
  发射点、目标点之后才算得出来），测试里靠先推两帧触发。

桩直接复用 :mod:`test_missile`（同一套 mount/view/registry），只是本件
**没有 guidance_slot 这个参数**——它压根没有制导槽。
"""

from __future__ import annotations

from math import atan2, cos, degrees, hypot, pi, sin, sqrt
from typing import Any

import pytest

from milsim.errors import ConfigurationError
from milsim.models.mover import AnalyticBallisticMover
from milsim.services.type_registry import ComponentFactory

from test_missile import (
    TARGET_ID,
    TARGET_NAME,
    FakeEntity,
    FakeMount,
    FakeRegistry,
    FakeView,
    Rig,
)

PERIOD_S = 5.0
PERIOD_US = int(PERIOD_S * 1_000_000)

R = 6_371_000.0
MU = 3.986_004_418e14

#: Φ=90°、弹道高点 1200 km 的档位（洲际锚点）。
ICBM = {
    "apogee_altitude": 1_200_000.0,
    "lethal_radius": 30.0,
}


# ---------------------------------------------------------------------------
# 闭式解：测试独立重算一遍（与 analytic.py 的实现分开写）
# ---------------------------------------------------------------------------

def closed_form(flat_m: float, apogee_m: float, z0: float = 0.0) -> dict[str, float]:
    """模块头那组闭式的独立副本：e / p / a / n / M₁ / h / v₁ / t_f。"""
    r1 = R + z0
    ra = R + apogee_m
    phi = flat_m / R
    e = (r1 - ra) / (r1 * cos(phi / 2.0) - ra)
    p = ra * (1.0 - e)
    a = p / (1.0 - e * e)
    n = sqrt(MU / a**3)
    nu1 = pi - phi / 2.0
    E1 = 2.0 * atan2(sqrt(1.0 - e) * sin(nu1 / 2.0), sqrt(1.0 + e) * cos(nu1 / 2.0))
    M1 = E1 - e * sin(E1)
    t_end = (2.0 * pi - 2.0 * M1) / n
    h = sqrt(MU * p)
    v_t = h / r1
    v_r = (MU / h) * e * sin(nu1)
    return {
        "e": e, "p": p, "a": a, "n": n, "M1": M1, "h": h, "t_end": t_end,
        "v": sqrt(v_t * v_t + v_r * v_r),
        "v_t": v_t, "v_r": v_r, "gamma": atan2(v_r, v_t),
    }


def state_at(ref: dict[str, float], t: float) -> tuple[float, float, float]:
    """独立闭式的根数 → ``t`` 时刻 ``(σ̇, v_r, z)``。

    开普勒牛顿解的独立副本（实现里那份在 ``analytic.py``）。置位路径
    的三个量——存储的水平速率、垂向速率、高度——在这里重算一遍，
    实现的求值链路错了（分量口径、时间律、置位时机）这里能看见。
    """
    M = ref["M1"] + ref["n"] * t
    E = M
    for _ in range(64):
        f = E - ref["e"] * sin(E) - M
        fp = 1.0 - ref["e"] * cos(E)
        step = f / fp
        E -= step
        if abs(step) <= 1e-13:
            break
    nu = 2.0 * atan2(
        sqrt(1.0 + ref["e"]) * sin(E / 2.0),
        sqrt(1.0 - ref["e"]) * cos(E / 2.0),
    )
    r = ref["a"] * (1.0 - ref["e"] * cos(E))
    v_r = (MU / ref["h"]) * ref["e"] * sin(nu)
    v_t = ref["h"] / r
    return v_t * R / r, v_r, r - R


# ---------------------------------------------------------------------------
# 装配（本件没有 guidance_slot，桩比 test_missile.build 短一截）
# ---------------------------------------------------------------------------

def build(
    params: dict[str, Any] | None = None,
    *,
    target: tuple[float, float, float] | None = None,
    x: float = 0.0, y: float = 0.0, z: float = 0.0,
) -> Rig:
    body_fields = dict(params or {})
    positions: dict[int, tuple[float, float, float]] = {}
    table: dict[str, int] = {}
    if target is not None:
        positions[TARGET_ID] = target
        table[TARGET_NAME] = TARGET_ID
        body_fields.setdefault("target_name", TARGET_NAME)
    body = ComponentFactory().build("mover", "ANALYTIC_BALLISTIC_MOVER", body_fields)
    view = FakeView(x, y, z)
    mount = FakeMount(view, registry=FakeRegistry(table), positions=positions)
    body.initialize(mount)
    return Rig(body, None, view, mount)


def north(distance_m: float) -> tuple[float, float, float]:
    """正北 ``distance_m`` 米。航向 0 = 北 = y 减小（§5.7）。"""
    return (0.0, -distance_m, 0.0)


def fly_recording(rig: Rig) -> tuple[float, float]:
    """飞完全程，返回 ``(最高点 m, 峰值总速 m/s)``。"""
    period_us = int(rig.body.spec["period"])
    now = 0
    best_z = 0.0
    best_v = 0.0
    for _ in range(6000):
        rig.body.update(now)
        now += period_us
        best_z = max(best_z, rig.view.my_position()[2])
        best_v = max(best_v, _total_speed(rig))
        if rig.body.arrived():
            break
    return best_z, best_v


def _total_speed(rig: Rig) -> float:
    return hypot(rig.view.speed, rig.body.vertical_speed_mps())


# ---------------------------------------------------------------------------
# 物理锚点
# ---------------------------------------------------------------------------

def test_icbm_physics_anchors() -> None:
    """Φ=90°（≈10008 km）+ 高点 1200 km：速度、时间、高点逐一对锚。

    7.2 km/s 与 31 min 是**真实洲际弹道的量级**——平面模型给不出
    （平面要 9.9 km/s 才飞得了 10000 km）。三层对拍互相独立：数字锚
    钉住闭式本身（手算区间，两份同源公式一起错时兜底），分量对拍钉住
    实现的置位路径，vis-viva 钉住能量守恒（与时间律无关的另一条通道）。
    """
    flat = R * pi / 2.0
    ref = closed_form(flat, 1_200_000.0)
    # 数字锚（手算值）：发射速度 7.2 km/s 上下、全程 1870 s 上下。
    assert 7_150.0 < ref["v"] < 7_250.0
    assert 1860.0 < ref["t_end"] < 1880.0

    rig = build(ICBM, target=north(flat))
    body = rig.body
    assert isinstance(body, AnalyticBallisticMover)
    body.update(0)
    body.update(PERIOD_US)                   # 第一次推进：解算 + 置位 t = 5 s
    assert body.flight_time_s() == pytest.approx(ref["t_end"], rel=1e-9)

    # 分量对拍：第一个推进帧的三个量与独立闭式逐一对上。
    sigma_dot, v_r_ref, z_ref = state_at(ref, PERIOD_S)
    assert rig.view.speed == pytest.approx(sigma_dot, rel=1e-9)
    assert body.vertical_speed_mps() == pytest.approx(v_r_ref, rel=1e-9)
    assert rig.view.my_position()[2] == pytest.approx(z_ref, abs=1e-4)

    # vis-viva：能量只由轨道根数决定、与"走到哪了"无关——发射速度量级
    # 由此独立复核。分量口径：σ̇·(r/R⊕) 还原成轨道横向速度。
    z = rig.view.my_position()[2]
    r_now = R + z
    v_t = rig.view.speed * r_now / R
    v_energy = sqrt(MU * (2.0 / r_now - 1.0 / ref["a"]))
    assert hypot(v_t, body.vertical_speed_mps()) == pytest.approx(
        v_energy, rel=1e-9
    )


def test_icbm_hits_and_apogee_is_the_parameter() -> None:
    """飞完 10008 km：落点 = 目标、弹道高点 = 参数、原因 = 命中。"""
    flat = R * pi / 2.0
    rig = build(ICBM, target=north(flat))
    best_z, _ = fly_recording(rig)
    assert rig.body.arrived()
    assert rig.body.hit()
    assert rig.body.terminate_reason() == "命中"
    assert rig.body.detonation_range_m() == pytest.approx(0.0, abs=1.0)
    assert rig.distance_to(north(flat)) == pytest.approx(0.0, abs=1.0)
    # 高点：对称弹道顶点 = apogee_altitude（帧采样在顶点附近，留 5 km 余量）。
    assert best_z == pytest.approx(1_200_000.0, abs=5_000.0)


def test_flight_time_is_close_form_not_a_guess() -> None:
    """三个射程档的全程都与独立闭式对拍（解算发生在第一次推进）。"""
    for flat_km, apogee_km in ((1_000.0, 250.0), (3_000.0, 600.0), (10_008.0, 1_200.0)):
        flat = flat_km * 1_000.0
        ref = closed_form(flat, apogee_km * 1_000.0)
        rig = build(
            {"apogee_altitude": apogee_km * 1_000.0}, target=north(flat)
        )
        rig.body.update(0)
        rig.body.update(PERIOD_US)               # 解算发生在第一次推进
        assert rig.body.flight_time_s() == pytest.approx(ref["t_end"], rel=1e-9)


def test_hits_any_azimuth() -> None:
    """偏东的目标一样命中：横向没有"正北才成立"的符号死角。"""
    flat = 3_000_000.0
    east = 1_500_000.0
    target = (east, -flat, 0.0)
    rig = build({"apogee_altitude": 600_000.0}, target=target)
    rig.fly()
    assert rig.body.hit()
    assert rig.distance_to(target) == pytest.approx(0.0, abs=1.0)
    # 航向恒为发射方位（东北向）：atan2(dx, −dy)。
    assert rig.view.heading == pytest.approx(degrees(atan2(east, flat)), abs=1e-9)


def test_altitude_profile_rises_then_falls() -> None:
    """z 剖面：先升后降，速度随高度变（轨道角动量守恒的地面投影）。"""
    flat = 3_000_000.0
    rig = build({"apogee_altitude": 600_000.0}, target=north(flat))
    period_us = int(rig.body.spec["period"])
    now = 0
    seen: list[tuple[float, float]] = []
    for _ in range(6000):
        rig.body.update(now)
        now += period_us
        seen.append((rig.view.my_position()[2], rig.view.speed))
        if rig.body.arrived():
            break
    zs = [z for z, _ in seen]
    peak = zs.index(max(zs))
    assert 0.0 < peak < len(zs) - 1          # 顶点在中间，不在两头
    assert zs[0] < zs[peak] * 0.01           # 发射点贴近地面
    assert zs[-1] <= 1.0                     # 落回地面
    # 顶点附近最慢（势能最大），两头快——不是匀速置位。
    assert seen[peak][1] < seen[1][1]


def test_first_tick_only_starts_the_clock() -> None:
    """接到命令的第一帧不挪窝（与全部机动件同一条约定）。"""
    rig = build(ICBM, target=north(R * pi / 2.0))
    rig.body.update(0)
    assert rig.view.my_position() == (0.0, 0.0, 0.0)


def test_assign_target_point_before_first_advance() -> None:
    """打"某个坐标"：assign_target_point 在第一次推进前给。"""
    flat = 2_000_000.0
    rig = build({"apogee_altitude": 400_000.0})
    rig.body.assign_target_point(*north(flat))
    rig.fly()
    assert rig.body.hit()
    assert rig.distance_to(north(flat)) == pytest.approx(0.0, abs=1.0)


def test_assign_target_point_after_solving_is_rejected() -> None:
    """解算锁定之后再改目标 = 报错，不是静默改弹道。"""
    rig = build({"apogee_altitude": 400_000.0}, target=north(2_000_000.0))
    rig.body.update(0)
    rig.body.update(PERIOD_US)               # 第一次推进：解算发生
    with pytest.raises(ConfigurationError):
        rig.body.assign_target_point(1.0, 2.0, 3.0)


# ---------------------------------------------------------------------------
# 装配期校验：都报错，不飞歪
# ---------------------------------------------------------------------------

def test_unknown_target_name_is_rejected() -> None:
    """target_name 写错：装配期（initialize 查存在性）就报错，不留到
    飞行时才发现没目标——写错名字与"没找到目标"在结果里长得一样。"""
    with pytest.raises(ConfigurationError):
        _build_with_bad_name()


def _build_with_bad_name() -> None:
    body = ComponentFactory().build(
        "mover", "ANALYTIC_BALLISTIC_MOVER",
        {"apogee_altitude": 400_000.0, "target_name": "NO_SUCH"},
    )
    view = FakeView()
    body.initialize(
        FakeMount(view, registry=FakeRegistry({}), positions={})
    )


def test_missing_target_is_rejected_at_first_advance() -> None:
    """既没有 target_name 也没有 assign：它不是积分器，没有目标没有弹道。"""
    rig = build({"apogee_altitude": 400_000.0})
    with pytest.raises(ConfigurationError):
        rig.body.update(0)
        rig.body.update(PERIOD_US)


def test_coincident_target_is_rejected() -> None:
    """目标与发射点重合（≤ 100 m）：解算不出一条没有射程的弹道。"""
    rig = build({"apogee_altitude": 400_000.0}, target=(0.0, -50.0, 0.0))
    with pytest.raises(ConfigurationError):
        rig.body.update(0)
        rig.body.update(PERIOD_US)


def test_overlong_range_is_rejected() -> None:
    """超过 170° 的地心角拒绝解算——椭圆在 180° 上退化。"""
    rig = build({"apogee_altitude": 1_200_000.0}, target=north(R * 3.0))
    with pytest.raises(ConfigurationError):
        rig.body.update(0)
        rig.body.update(PERIOD_US)


def test_apogee_below_launch_altitude_is_rejected() -> None:
    """弹道高点不高于发射点：解算拒绝（对称弹道要求 ra > r₁）。"""
    rig = build(
        {"apogee_altitude": 100.0},
        target=north(2_000_000.0),
        z=500.0,
    )
    with pytest.raises(ConfigurationError):
        rig.body.update(0)
        rig.body.update(PERIOD_US)


class LiveEngagement:
    """``position_of`` 读**活** dict 的 engagement 视图。

    ``test_missile.FakeEngagement`` 是快照（构造时拷贝一份 dict）——那对
    静止目标是对的，但模拟不了"目标平台在飞行中移动"：本件的锁定语义
    恰恰要用到"解算读一次实时位置、终止再读一次"。生产引擎里
    ``EngagementView.position_of`` 就是实时的。
    """

    def __init__(self, positions: dict[int, tuple[float, float, float]]) -> None:
        self._positions = positions

    def position_of(self, target_id: int) -> tuple[float, float, float] | None:
        return self._positions.get(target_id)


def test_moved_target_lands_on_the_locked_point() -> None:
    """解算锁定**解算时刻**的目标位置：目标被移走后弹落在旧位置上，
    按 lethal_radius 判"落地"——洲际弹打装订坐标的作战方式（§5.8）。"""
    flat = 2_000_000.0
    body_fields = {"apogee_altitude": 400_000.0, "lethal_radius": 30.0,
                   "target_name": TARGET_NAME}
    body = ComponentFactory().build("mover", "ANALYTIC_BALLISTIC_MOVER", body_fields)
    view = FakeView()
    positions = {TARGET_ID: north(flat)}
    mount = FakeMount(
        view, registry=FakeRegistry({TARGET_NAME: TARGET_ID}), positions=positions,
    )
    mount.engagement_view = lambda: LiveEngagement(positions)   # 活视图
    body.initialize(mount)
    rig = Rig(body, None, view, mount)

    # 第一次推进前把目标挪到旁边 50 km：解算读实时位置，锁的就是挪后那份。
    moved = (500_000.0, -flat, 0.0)
    positions[TARGET_ID] = moved
    body.update(0)
    body.update(PERIOD_US)                   # 第一次推进：解算用挪后的位置
    positions[TARGET_ID] = north(flat)       # 之后再"移回"（模拟目标移动）
    # 续推到终止：时钟接在已推进的时刻上（elapsed = last_tick − started，
    # 从 0 重来会把钟拨回去）。
    now = 2 * PERIOD_US
    for _ in range(6000):
        body.update(now)
        now += PERIOD_US
        if body.arrived():
            break
    assert body.terminate_reason() == "落地"
    # 落点 = 解算目标（挪后位置），不是想定里那一份。
    assert rig.distance_to(moved) == pytest.approx(0.0, abs=1.0)


def test_registered_in_the_framework_inventory() -> None:
    """注册名能被装配（FRAMEWORK_MOVERS 的清单口径）。"""
    body = ComponentFactory().build(
        "mover", "ANALYTIC_BALLISTIC_MOVER", {"apogee_altitude": 400_000.0}
    )
    assert isinstance(body, AnalyticBallisticMover)
