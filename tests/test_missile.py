"""弹组件测试：弹体 + 制导件（§5.8）。

弹的失败方式和平台机动件不同：平台件是"数字对不上"，弹是**"参数看起来一个
都没错，而弹根本飞不起来"**。这一组盯的就是后者，每一条都对应一个实测过的
症状：

- 从静止起步的巡航弹在发射位上原地打转（推力比重量小，而参数全对）
- 速度近零时角速度形式发散，2 s 内航迹倾角在 −167° 与 +101° 之间乱跳
- 两边各算一次 ``g·cosγ`` → 重力被抵掉两次，弹朝**反方向**飞
- 引信在"第一次进入杀伤半径"就点火 → 报出来的 25 m 被当成脱靶量
- 末段交接太晚（4 km）→ 6 g 只够改 28° 方向，35~45 km 的目标被打飞
- 阶段表最后一段写了推进条件 → 那句话静默不生效
- 制导槽名写错 → 退化成无控弹，"导引头没工作"与"打歪了"在结果里长得一样

用**手写桩**而不是真战区：这里要钉的是"弹怎么用指令与目标位置"，不是"导航
算得对不对"。真接线（``NavService`` 注入、平台属性、事件节拍）是另一组集成
用例的事，想定里接弹还没做。

**桩比 ``test_mover.py`` 多三样**：``mount.entity.component(slot)``（弹体找
制导件）、``mount.registry.by_name()`` 与 ``mount.engagement_view()``（制导件
找目标）。
"""

from __future__ import annotations

from dataclasses import replace
from math import degrees, atan2, hypot
from typing import Any

import pytest

from milsim.errors import ConfigurationError
from milsim.models.guidance import (
    ASM_PHASES,
    BALLISTIC_PHASES,
    CRUISE_PHASES,
    GuidanceComputer,
)
from milsim.models.guidance.contract import (
    HORIZ_HOLD,
    HORIZ_PN,
    HORIZ_TARGET,
    VERT_BALANCE,
    VERT_FPA,
    VERT_FREE,
    VERT_PN,
    FlightState,
    Phase,
    Until,
)
from milsim.models.mover import MissileMover
from milsim.services.params import STANDARD_GRAVITY, ParamError
from milsim.services.type_registry import ComponentFactory

PERIOD_S = 2.0
PERIOD_US = int(PERIOD_S * 1_000_000)

#: 弹道弹的配套性能参数（与 ``guidance/ballistic.py`` 的模块头一致）。
BALLISTIC_BODY = {
    "mass": 800.0,
    "fuel_mass": 400.0,
    "thrust": 66_000.0,
    "specific_impulse": 250_000_000,
    "drag_area": 0.09,
    "radial_accel": 40.0,
    "max_speed": 1500.0,
    "launch_fpa": 85.0,
}

#: 巡航弹的配套性能参数（与 ``guidance/cruise.py`` 的模块头一致）。
CRUISE_BODY = {
    "mass": 1200.0,
    "fuel_mass": 250.0,
    "thrust": 1600.0,
    "specific_impulse": 3000_000_000,
    "drag_area": 0.024,
    "radial_accel": 12.0,
    "max_speed": 300.0,
    "launch_speed": 240.0,
}

#: 目标实体 ID。名字 → ID 与 ID → 位置两张表都用它。
TARGET_ID = 7
TARGET_NAME = "TGT"


# ---------------------------------------------------------------------------
# 桩
# ---------------------------------------------------------------------------

class FakeEntity:
    """``mount.entity``：弹体靠它找制导件。"""

    def __init__(self, components: dict[str, Any] | None = None) -> None:
        self._components = dict(components or {})

    @property
    def entity_id(self) -> int:
        return 1

    def component(self, slot: str) -> Any:
        return self._components.get(slot)

    def slots(self) -> list[str]:
        return list(self._components)


class FakeRegistered:
    def __init__(self, entity_id: int) -> None:
        self.entity_id = entity_id


class FakeRegistry:
    """``mount.registry``：制导件靠它把 ``target_name`` 换成实体 ID。"""

    def __init__(self, table: dict[str, int] | None = None) -> None:
        self._table = dict(table or {})

    def by_name(self, name: str) -> FakeRegistered | None:
        entity_id = self._table.get(name)
        return None if entity_id is None else FakeRegistered(entity_id)


class FakeEngagement:
    """``mount.engagement_view()``：制导件靠它取目标**真值**位置。"""

    def __init__(self, positions: dict[int, tuple[float, float, float]]) -> None:
        self._positions = dict(positions)

    def position_of(self, target_id: int) -> tuple[float, float, float] | None:
        return self._positions.get(target_id)


class FakeView:
    """``MoverView`` 的五个方法 + 一份轨迹，供"挪了多少"用。"""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 heading: float = 0.0, speed: float = 0.0) -> None:
        self.x, self.y, self.z = x, y, z
        self.heading, self.speed = heading, speed
        self.cell: Any = None
        self.track: list[tuple[float, float, float]] = [(x, y, z)]
        self.alive = True

    def my_pose(self) -> tuple[float, float, float, float, float]:
        return (self.x, self.y, self.z, self.heading, self.speed)

    def my_position(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def my_cell(self) -> Any:
        return self.cell

    def my_alive(self) -> bool:
        return self.alive

    def set_pose(self, x: float, y: float, z: float, heading: float = 0.0,
                 speed: float = 0.0) -> None:
        self.x, self.y, self.z = x, y, z
        self.heading, self.speed = heading, speed
        self.track.append((x, y, z))


class StubNav:
    """只要 ``ground_at``。测"往哪飞"不需要地形与通行门槛。"""

    def __init__(self, ground: float = 0.0) -> None:
        self._ground = ground

    def ground_at(self, ref: Any, x: float, y: float) -> float:
        return self._ground


class FakeMount:
    """``MountContext`` 里弹体与制导件真正用到的那几项。"""

    def __init__(self, view: FakeView, *, nav: Any = None,
                 entity: FakeEntity | None = None,
                 registry: Any = None,
                 positions: dict[int, tuple[float, float, float]] | None = None,
                 cell_span_m: float = 0.0) -> None:
        self._view = view
        self.nav = nav
        self.bus = None
        self.entity = entity
        self.registry = registry
        self._positions = positions or {}
        self._span_m = cell_span_m
        self.schedules: list[tuple[int, int]] = []

    def mover_view(self) -> FakeView:
        return self._view

    def engagement_view(self) -> FakeEngagement:
        return FakeEngagement(self._positions)

    def cell_span_m(self) -> float:
        return self._span_m

    def every(self, interval_us: int, fn: Any, priority: int = 0) -> None:
        self.schedules.append((interval_us, priority))


# ---------------------------------------------------------------------------
# 装配
# ---------------------------------------------------------------------------

class Rig:
    """一份装好线的"弹 + 制导件"，外加它能看到的一切。"""

    def __init__(self, body: MissileMover, guidance: GuidanceComputer | None,
                 view: FakeView, mount: FakeMount) -> None:
        self.body = body
        self.guidance = guidance
        self.view = view
        self.mount = mount

    def fly(self, *, frames: int = 6000) -> int:
        """按弹自己的周期推进，返回最后一帧的时刻（微秒）。"""
        period_us = int(self.body.spec["period"])
        now = 0
        for _ in range(frames):
            self.body.update(now)
            now += period_us
            if self.body.arrived():
                break
        return now - period_us

    def distance_to(self, target: tuple[float, float, float]) -> float:
        x, y, z = self.view.my_position()
        return hypot(hypot(target[0] - x, target[1] - y), target[2] - z)


def build(
    body_params: dict[str, Any] | None = None,
    *,
    body_type: str = "MISSILE_MOVER",
    guidance_type: str | None = None,
    guidance_params: dict[str, Any] | None = None,
    guidance_slot: str | None = None,
    target: tuple[float, float, float] | None = None,
    x: float = 0.0, y: float = 0.0, z: float = 0.0,
    heading: float = 0.0, speed: float = 0.0,
    nav: Any = None,
    with_entity: bool = True,
) -> Rig:
    """造一枚装好线的弹。

    ``guidance_type`` 给 ``None`` 时**不装制导件**（无控弹），此时
    ``guidance_slot`` 自动置空——那是"这型弹就是无控的"的正常配置，
    不是漏装。``guidance_slot`` 显式给值用于测"槽名写错"那一条。
    """
    factory = ComponentFactory()
    guidance: GuidanceComputer | None = None
    components: dict[str, Any] = {}
    if guidance_type is not None:
        params = dict(guidance_params or {})
        if target is not None:
            params.setdefault("target_name", TARGET_NAME)
        guidance = factory.build("guidance", guidance_type, params)
        components["guidance"] = guidance

    body_fields = dict(body_params or {})
    if guidance_slot is None:
        body_fields.setdefault("guidance_slot", "guidance" if guidance else "")
    else:
        body_fields["guidance_slot"] = guidance_slot
    body = factory.build("mover", body_type, body_fields)

    entity = FakeEntity(components) if with_entity else None
    if entity is not None:
        body.bind(entity)
        if guidance is not None:
            guidance.bind(entity)

    positions: dict[int, tuple[float, float, float]] = {}
    table: dict[str, int] = {}
    if target is not None:
        positions[TARGET_ID] = target
        table[TARGET_NAME] = TARGET_ID

    view = FakeView(x, y, z, heading, speed)
    mount = FakeMount(
        view, nav=nav if nav is not None else StubNav(0.0), entity=entity,
        registry=FakeRegistry(table), positions=positions,
    )
    body.initialize(mount)
    if guidance is not None:
        guidance.initialize(mount)
    return Rig(body, guidance, view, mount)


def near(target: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    """目标**正北** ``distance`` 米处。航向 0 = 北 = ``y`` 减小（§5.7）。"""
    return (target[0], target[1] - distance, target[2])


def offset(target: tuple[float, float, float], north: float, east: float) -> tuple[float, float, float]:
    """目标**正北** ``north`` 米、再**往东** ``east`` 米处。

    横向偏移是**必须测的**：正北的目标上 ``λ̇_h`` 恒为 0，横向那条环等于
    没接进去——实测过一个符号错误只在这个方向上看不出来（见
    ``test_heading_law_pushes_the_right_way``）。
    """
    return (target[0] + east, target[1] - north, target[2])


# ---------------------------------------------------------------------------
# 几何量的**符号**：唯一一处"错了还能看起来正常工作"的地方
# ---------------------------------------------------------------------------

def test_closing_speed_is_positive_when_approaching() -> None:
    """接近速率与两个视线角速率的符号逐个钉住。

    垂向比例导引用的是 ``N·V_c·λ̇_v``——``V_c`` 与 ``λ̇_v`` **各自反一次号会
    互相抵消**，所以"垂向看起来正常"根本不能证明符号是对的。只有把三个量
    单独拿出来对，才拦得住那个错误（横向那次是 ``N·V_c·λ̇_h``，只反了 ``V_c``
    一个 ⇒ 正反馈）。

    返回值是六项 ``(斜距, 接近速率, 方位角, 高低角, 方位角速率, 高低角速率)``
    ——两个位置量在前、两个速率量在后。高低角是**位置量**，它与前两个一起
    被推进条件读（``target_elevation``），与后面的速率量不是一回事：跃升段里
    ``λ̇_v`` 可以是正的（还在抬头看目标）而 ``λ_v`` 已经该转负了。
    """
    # 1) 正北的目标、朝正北飞：接近速率 = 自己的速率
    north = FlightState(x=0.0, y=0.0, z=0.0, heading=0.0, speed=200.0,
                        vertical_speed=0.0, flight_path_angle=0.0)
    distance, closing, azimuth, elevation, az_rate, el_rate = \
        GuidanceComputer._geometry(north, (0.0, -20_000.0, 0.0))
    assert distance == pytest.approx(20_000.0)
    assert closing == pytest.approx(200.0)          # 正 = 在靠近
    assert azimuth == pytest.approx(0.0)
    assert elevation == pytest.approx(0.0)          # 同高 ⇒ 视线是水平的
    assert az_rate == pytest.approx(0.0)
    assert el_rate == pytest.approx(0.0)

    # 2) 反着飞：接近速率必须是负的
    away = replace(north, heading=180.0)
    assert GuidanceComputer._geometry(away, (0.0, -20_000.0, 0.0))[1] == \
        pytest.approx(-200.0)

    # 3) 目标在北偏东 2 km：视线在**右边**、而且还在继续往右转
    _, _, azimuth, _, az_rate, _ = GuidanceComputer._geometry(
        north, (2_000.0, -20_000.0, 0.0))
    assert azimuth == pytest.approx(degrees(atan2(2_000.0, 20_000.0)))
    assert az_rate > 0.0                            # 正 = 往右转

    # 4) 目标在**下面**：视线在往下转（λ̇_v < 0），**而且高低角本身是负的**
    above = replace(north, z=10_000.0)
    _, _, _, elevation, _, el_rate = GuidanceComputer._geometry(
        above, (0.0, -20_000.0, 0.0))
    assert elevation == pytest.approx(-degrees(atan2(10_000.0, 20_000.0)))
    assert el_rate < 0.0


def test_target_elevation_is_the_los_angle_itself() -> None:
    """``target_elevation`` 是视线的**位置**量，不是速率的别名。

    AFSIM 的 ``POPUP`` 段用 ``target_elevation < -20 deg`` 切进 ``DIVE``
    （"跃升到目标掉到视线下方 20°"）。它和 ``target_elevation_rate`` 必须
    分开算：跃升**中**弹还在抬头看目标（``λ̇_v`` 仍可能是正的），而"目标
    已经在下方 20°"这件事已经成立了——拿速率去近似这个判据会晚一整段。

    竖直视线（弹正在目标正上方）那一支也钉住：**方位角**无定义而归 0，
    **高低角**有定义，是 ±90°。给 0 的话 DIVE 的判据会在最该成立的地方
    反而不成立。
    """
    level = FlightState(x=0.0, y=0.0, z=1_000.0, heading=0.0, speed=200.0,
                        vertical_speed=0.0, flight_path_angle=0.0)

    # 目标同高 ⇒ 0°；在上方 ⇒ 正；在下方 ⇒ 负
    assert GuidanceComputer._geometry(level, (0.0, -10_000.0, 1_000.0))[3] == \
        pytest.approx(0.0)
    assert GuidanceComputer._geometry(level, (0.0, -10_000.0, 11_000.0))[3] == \
        pytest.approx(45.0)          # 高差 = 水平距离 ⇒ 正好 45°
    assert GuidanceComputer._geometry(level, (0.0, -10_000.0, 0.0))[3] == \
        pytest.approx(-degrees(atan2(1_000.0, 10_000.0)))

    # 视线竖直：方位角 0（无定义），高低角 ±90
    assert GuidanceComputer._geometry(level, (0.0, 0.0, 5_000.0))[3] == \
        pytest.approx(90.0)
    assert GuidanceComputer._geometry(level, (0.0, 0.0, 0.0))[3] == \
        pytest.approx(-90.0)

    # 没有目标：0.0 而不是 inf —— "目标掉到下方 20°"这条判据不会凭空成立
    assert GuidanceComputer._geometry(level, None)[3] == 0.0
    assert not Until.below("target_elevation", -20.0).test(
        FlightState(target_elevation=0.0))


def test_heading_law_pushes_the_right_way() -> None:
    """横向比例导引要把弹"往目标那一侧"推，不是往外推。

    这一条是**实测查出来的**：修之前 ``V_c`` 反号，于是 ``HORIZ_PN`` 成了正
    反馈——打正北的目标偏 14.6 m，**目标往东挪 500 m 就变成偏 15.7 km**，
    也就是那枚弹只能打正好在发射方位线上的目标。
    """
    right = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                  guidance_params={"phases": (_seek_phase(HORIZ_PN),),
                                   "target_name": TARGET_NAME},
                  target=(2_000.0, -20_000.0, 0.0))
    assert right.guidance.command(_seek_state(z=1_000.0)).lateral_accel > 0.0

    # 目标在**左边**：必须反过来
    left = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                 guidance_params={"phases": (_seek_phase(HORIZ_PN),),
                                  "target_name": TARGET_NAME},
                 target=(-2_000.0, -20_000.0, 0.0))
    assert left.guidance.command(_seek_state(z=1_000.0)).lateral_accel < 0.0


def test_vertical_law_pulls_toward_the_target_below() -> None:
    """垂向比例导引的符号：目标在**下面**就要比 1 g 平衡更小（往下拉）。

    垂向那一对（``V_c`` 与 ``λ̇_v``）反号会互相抵消，所以它单独测才有意义——
    "看起来正常"证明不了什么。
    """
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                guidance_params={"phases": (_seek_phase(HORIZ_PN, VERT_PN),),
                                 "target_name": TARGET_NAME},
                target=(0.0, -20_000.0, 0.0))
    # 弹在 10 km 高、目标是地面目标 ⇒ 视线在往下转 ⇒ 拉下去
    below = rig.guidance.command(_seek_state(z=10_000.0))
    assert below.vertical_accel < STANDARD_GRAVITY
    # 同一个式子，把高度放到目标以下 ⇒ 要往上拉
    above = rig.guidance.command(_seek_state(z=-500.0))
    assert above.vertical_accel > STANDARD_GRAVITY


def test_heading_law_holds_the_heading_when_there_is_nothing_to_turn() -> None:
    """``HORIZ_HOLD`` 不看目标：横向指令恒为 0（发射段与自由飞段用它）。"""
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                guidance_params={"phases": (_seek_phase(HORIZ_HOLD),),
                                 "target_name": TARGET_NAME},
                target=(2_000.0, -20_000.0, 0.0))
    assert rig.guidance.command(_seek_state(z=1_000.0)).lateral_accel == 0.0


def _seek_phase(horizontal: str, vertical: str = VERT_BALANCE) -> Phase:
    """单段剧本：只用给定的两个导引律。"""
    return Phase("SEEK", vertical=vertical, horizontal=horizontal, gain=4.0)


def _seek_state(z: float) -> FlightState:
    """一个朝正北、200 m/s 的采样状态（位置在原点）。"""
    return FlightState(
        flight_time=1.0, x=0.0, y=0.0, z=z, ground_z=0.0,
        heading=0.0, speed=200.0, vertical_speed=0.0, flight_path_angle=0.0,
        mass=1_150.0,
    )


# ---------------------------------------------------------------------------
# 装配期校验：宁可报错，不要"参数看着都对、弹却不对"
# ---------------------------------------------------------------------------

def test_guidance_slot_typo_names_the_slots_that_exist() -> None:
    """槽名写错要在装配期报错，并列出**已有的槽**。

    退化成无控弹的话，"弹笔直往前飞"与"导引头没工作"在结果里长得一模一样，
    而排查方向完全不同。
    """
    with pytest.raises(ConfigurationError) as excinfo:
        build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
              guidance_slot="seekers")
    message = str(excinfo.value)
    assert "seekers" in message
    assert "guidance" in message          # 已有的槽列出来了


def test_empty_slot_is_an_uncontrolled_missile_not_a_bug() -> None:
    """``guidance_slot ""`` = 这型弹就是无控的（火箭弹）。"""
    rig = build(BALLISTIC_BODY)
    assert rig.body.guidance() is None


def test_empty_phase_table_is_rejected() -> None:
    """空阶段表 = 没有剧本的制导件，什么也做不了。"""
    with pytest.raises(ConfigurationError) as excinfo:
        build({}, guidance_type="CRUISE_GUIDANCE",
              guidance_params={"phases": (), "target_name": ""})
    assert "阶段表" in str(excinfo.value)


def test_last_phase_must_not_carry_an_until() -> None:
    """最后一段写推进条件 → 没有下一段可去，那句话不会生效。"""
    phases = (
        Phase("ONLY", until=Until.after(1.0)),
    )
    with pytest.raises(ConfigurationError) as excinfo:
        build({}, guidance_type="CRUISE_GUIDANCE",
              guidance_params={"phases": phases, "target_name": ""})
    assert "ONLY" in str(excinfo.value)


def test_unknown_target_name_is_rejected() -> None:
    """目标名字写错 → 报错，而不是变成"弹笔直飞向原点"。"""
    with pytest.raises(ConfigurationError) as excinfo:
        build({}, guidance_type="CRUISE_GUIDANCE",
              guidance_params={"target_name": "NOT_THERE"})
    assert "NOT_THERE" in str(excinfo.value)


def test_fuel_heavier_than_the_body_is_rejected() -> None:
    """燃料比总质量还多 → 净重为负，那是参数错了不是模型不行。"""
    with pytest.raises(ConfigurationError) as excinfo:
        build({**BALLISTIC_BODY, "mass": 100.0, "fuel_mass": 200.0})
    assert "燃料" in str(excinfo.value)


def test_too_many_substeps_is_rejected_instead_of_silently_coarsening() -> None:
    """子步数超上限要报错。**悄悄放粗**会让末端精度随想定里的周期变化，
    而那个变化没有任何地方会写出来。"""
    with pytest.raises(ConfigurationError) as excinfo:
        build({**BALLISTIC_BODY, "period": 60_000_000, "substep": 1_000})
    assert "子步" in str(excinfo.value)


def test_missile_refuses_cell_and_point_destinations() -> None:
    """弹不接受"按格/按点下达目的地"——那种命令等于把制导件绕过。"""
    rig = build(BALLISTIC_BODY)
    with pytest.raises(ConfigurationError):
        rig.body.move_to_point(0.0, -1000.0, 0.0)
    with pytest.raises(ConfigurationError):
        rig.body.move_to_cell(object())


def test_phase_rejects_an_altitude_with_a_conflicting_law() -> None:
    """``altitude=50`` 配 ``VERT_FREE`` 要让**构造阶段**就报错。

    不报的话那句高度指令静默失效，而写它的人正盯着"为什么没爬到 50 m"。
    """
    with pytest.raises(ParamError) as excinfo:
        Phase("ODD", altitude=50.0, vertical=VERT_FREE)
    assert "不会生效" in str(excinfo.value)


# ---------------------------------------------------------------------------
# 弹体：三自由度质点
# ---------------------------------------------------------------------------

def test_first_tick_only_starts_the_clock() -> None:
    """接到命令的第一帧**不挪窝**（与平台件同一条约定，§5.7）。

    一帧代表区间 ``[now, now+period]`` 上的位移，而这个区间还没过去。
    """
    rig = build(BALLISTIC_BODY)
    rig.body.update(0)
    x, y, z = rig.view.my_position()
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert rig.view.track == [(0.0, 0.0, 0.0)]


def test_uncontrolled_missile_is_a_rocket() -> None:
    """无控弹烧完燃料自己落地：弹道是抛物线，不是一条直线。

    它同时钉住"**先离地才判落地**"：地面发射的弹在点火那一刻 z 正好等于
    地面高程，少了那个门槛它会在第一帧就"落地"并停在发射位上。

    射程 31.9 km（最高 7.25 km）是**实测值，不是真空公式算的**：无控弹一离轨
    就做重力转向，发射角 60° 在 2 s 内就被压到 35°（``missile.py`` 模块头
    有一张三档发射角的表）。所以这个数既不等于 ``v²·sin2θ/g``，也不随发射角
    单调——45° 反而只有 15.2 km。
    """
    rig = build({**BALLISTIC_BODY, "launch_fpa": 60.0})
    elapsed = rig.fly()
    assert rig.body.terminate_reason() == "落地"
    assert elapsed > 60_000_000          # 飞了 60 s 以上，不是第一帧就停
    assert rig.body.max_altitude_m() > 3000.0
    assert rig.body.fuel_kg() == 0.0
    assert rig.body.mass_kg() == pytest.approx(400.0)     # 净重
    assert rig.body.burnout() is True
    downrange = hypot(rig.view.x, rig.view.y)
    assert 30_000.0 < downrange < 33_000.0, downrange
    assert 6_000.0 < rig.body.max_altitude_m() < 9_000.0


def test_burnout_is_false_before_ignition() -> None:
    """``burnout`` = **点过火、而现在没有推力**，不是"thrusting 为假"。

    刚离轨那一瞬 thrusting 也是假的，拿它当推进条件会让弹第一帧就跳过助推段。
    """
    rig = build(BALLISTIC_BODY)
    assert rig.body.burnout() is False          # 还没点火
    rig.body.update(0)
    rig.body.update(PERIOD_US)                  # 推进一帧：发动机在工作
    assert rig.body.thrusting() is True
    assert rig.body.burnout() is False          # 有推力 ⇒ 不是 burnout
    rig.fly(frames=20)
    assert rig.body.burnout() is True           # 燃料烧完之后


def test_drag_is_not_decoration() -> None:
    """去掉阻力，射程明显变长——所以"拟合射程"离不开它。"""
    with_drag = build({**BALLISTIC_BODY, "launch_fpa": 45.0})
    with_drag.fly()
    without = build({**BALLISTIC_BODY, "launch_fpa": 45.0, "drag_area": 0.0})
    without.fly()
    far = hypot(without.view.x, without.view.y)
    near_ = hypot(with_drag.view.x, with_drag.view.y)
    assert far > near_ * 1.5, (near_, far)


def test_stored_speed_is_the_horizontal_component() -> None:
    """存储里那一格是**水平**速率，不是总速率。

    写总速率进去的话，``KinematicsTable.advance``（``dx = d·sinθ``）会把它
    当成水平位移用——两个数都在同一行里，谁也看不出哪个是假的。
    """
    rig = build(BALLISTIC_BODY)
    rig.body.update(0)
    rig.body.update(PERIOD_US)
    rig.body.update(2 * PERIOD_US)
    horizontal = rig.body.speed_mps()
    vertical = rig.body.vertical_speed_mps()
    assert horizontal == pytest.approx(rig.view.speed)     # 存储与查询一致
    assert rig.body.total_speed_mps() == pytest.approx(
        hypot(horizontal, vertical)
    )
    # 垂直发射段：总速率比水平分量大得多
    assert rig.body.total_speed_mps() > horizontal * 3.0
    assert rig.body.flight_path_angle_deg() == pytest.approx(
        degrees(atan2(vertical, horizontal)), abs=1e-6
    )


def test_missile_ground_is_the_surface_it_can_hit_not_the_seabed() -> None:
    """弹的"地面"是 ``max(高程, 0)``——**水域的高程通道放的是海床**。

    实测战区里一片水面的高程是 **−423.5 m**，而弹撞到**海面**就结束了，撞不到
    海床。不夹这一下有两个后果，两个都是实测的：

    - 巡航段"离地 50 m"跟的是海床：飞过海岸线时指令高度从 ``地面 + 50`` 掉到
      ``−423 + 50 = −373 m``，弹在半路一头扎进水里——**参数表里一个数都没错**；
    - 落地判据也用它，弹要沉到水下几百米才算"落地"。

    陆上不受影响（高程为正，``max`` 取它自己）。**与四个平台机动件的口径故意
    不同**：地面件是在地上跑的，"地面"就是地形；弹是在空中飞的，"地面"是它能
    撞到的那个面。
    """
    assert build(CRUISE_BODY, nav=StubNav(-423.5)).body._ground_z(
        None, 0.0, 0.0
    ) == 0.0
    assert build(CRUISE_BODY, nav=StubNav(343.5)).body._ground_z(
        None, 0.0, 0.0
    ) == 343.5
    # 没有地图时仍然是 0（战区外也要落地，不能一直往下飞）
    assert build(CRUISE_BODY, nav=None).body._ground_z(None, 0.0, 0.0) == 0.0


def test_cruise_missile_cruises_above_the_sea_not_above_the_seabed() -> None:
    """同一条的端到端形态：海面上巡航的高度是**海拔 50 m**，不是海床之上 50 m。

    100 km 的目标是为了让第 60 帧仍然在巡航段里（40 km 的靶子那时已经进末段、
    开始俯冲了，量到的是"正在往下扎"而不是巡航高度）。
    """
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                target=near((0.0, 0.0, 0.0), 100_000.0), nav=StubNav(-423.5))
    period_us = int(rig.body.spec["period"])
    for k in range(60):
        rig.body.update(k * period_us)
    assert rig.guidance.phase_name() == "CRUISE"
    assert 40.0 < rig.view.z < 160.0, rig.view.z


def test_level_flight_reports_no_path_angle() -> None:
    """平飞的弹道倾角是 0——垂直速率与地面高度都不是零，是**倾角**为零。"""
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                target=near((0.0, 0.0, 0.0), 40_000.0))
    rig.fly(frames=60)
    assert abs(rig.body.flight_path_angle_deg()) < 10.0
    assert rig.body.vertical_speed_mps() == pytest.approx(0.0, abs=2.0)


# ---------------------------------------------------------------------------
# 制导：阶段推进与"关掉制导"
# ---------------------------------------------------------------------------

def test_phase_advances_as_soon_as_the_condition_holds() -> None:
    """条件一成立就换段；而"一帧里跨两段"只对**非时间条件**成立。

    ``Until.after(0.0)`` 是 ``phase_time > 0``——**严格大于**，所以 t = 0 那一刻
    它不成立（"过了 0 秒"读起来像立刻，其实是"过了正的时长"）。而换段之后
    ``phase_time`` 归零，于是时间条件一次只能推进一步。这不是缺漏，是"阶段从
    此刻开始算"的定义；高度、距离这些不受影响，一成立就一路跨过去。
    """
    time_based = (
        Phase("SKIP", until=Until.after(0.0)),
        Phase("ALSO_SKIP", until=Until.after(0.0)),
        Phase("STAY"),
    )
    rig = build({}, guidance_type="CRUISE_GUIDANCE",
                guidance_params={"phases": time_based, "target_name": ""})
    assert rig.guidance.phase_name() == "SKIP"
    rig.guidance.command(FlightState(flight_time=0.0))     # phase_time 恰好 0
    assert rig.guidance.phase_name() == "SKIP"
    rig.guidance.command(FlightState(flight_time=0.1))     # 过了正的时长
    assert rig.guidance.phase_name() == "ALSO_SKIP"
    rig.guidance.command(FlightState(flight_time=0.2))
    assert rig.guidance.phase_name() == "STAY"
    assert [row[0] for row in rig.guidance.timeline()] == [
        "SKIP", "ALSO_SKIP", "STAY"
    ]

    value_based = (
        Phase("HIGH", until=Until.above("altitude", 1_000.0)),
        Phase("STILL_HIGH", until=Until.above("altitude", 10_000.0)),
        Phase("LOW"),
    )
    rig = build({}, guidance_type="CRUISE_GUIDANCE",
                guidance_params={"phases": value_based, "target_name": ""})
    rig.guidance.command(FlightState(flight_time=1.0, z=20_000.0))
    assert rig.guidance.phase_name() == "LOW"              # 一次跨两段
    assert [row[0] for row in rig.guidance.timeline()] == [
        "HIGH", "STILL_HIGH", "LOW"
    ]


def test_ballistic_segment_switches_guidance_off() -> None:
    """``BALLISTIC`` 段的指令是"什么都不接管"——这就是"弹道"。

    不写成"一个永远到不了的时刻"（AFSIM 的 ``guidance_delay 5000 sec``），
    而是三样都空：无法向加速度、无横向加速度、无速率指令。
    """
    rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                target=near((0.0, 0.0, 0.0), 80_000.0))
    # **按阶段走，不按固定帧数走**：自由飞那一段的长短由交接距离与目标距离
    # 一起决定（80 km 的目标是 14.9~101.1 s，49 km 的目标只剩 1.5 s），写死
    # 帧数会在调交接距离时变成一个假的失败。
    for k in range(400):
        if rig.guidance.phase_name() == "BALLISTIC":
            break
        rig.body.update(k * PERIOD_US)
    assert rig.guidance.phase_name() == "BALLISTIC"
    table = rig.guidance.phase_table()
    ballistic = [p for p in table if p.name == "BALLISTIC"][0]
    assert ballistic.vertical == VERT_FREE
    assert ballistic.horizontal == HORIZ_HOLD

    command = rig.guidance.command(FlightState(
        flight_time=40.0, x=0.0, y=-15_000.0, z=14_000.0, ground_z=0.0,
        heading=0.0, speed=300.0, vertical_speed=140.0,
        flight_path_angle=25.0, mass=400.0, thrusting=False, burnout=True,
        downrange=16_000.0,
    ))
    assert command.vertical_accel == 0.0
    assert command.lateral_accel == 0.0
    assert command.speed_mps is None
    assert command.terminate is False


def test_target_law_without_a_target_is_an_error() -> None:
    """要用目标导引却没有目标 → 报错，而不是飞向原点。"""
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE")
    with pytest.raises(ConfigurationError) as excinfo:
        rig.fly(frames=200)
    assert "target" in str(excinfo.value).lower()


def test_assign_target_point_replaces_the_named_target() -> None:
    """没有目标实体时可以打"某个坐标"（运行期指定）。"""
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE")
    rig.guidance.assign_target_point(0.0, -20_000.0, 0.0)
    assert rig.guidance.target_point() == (0.0, -20_000.0, 0.0)


# ---------------------------------------------------------------------------
# 端到端：跃升俯冲（反舰弹）——``target_elevation`` 唯一的存在理由
# ---------------------------------------------------------------------------

def test_asm_table_splits_on_the_target_angle() -> None:
    """反舰弹那张表**本身**的验收：三段，且 ``POPUP → DIVE`` 看的是**目标**。

    这比"飞出来三段"更强：它钉的是**哪一句**在切段。把判据换成 ``descending``
    那张表照样能跑（只是永远切不过去），所以只测"飞出了 DIVE"拦不住那个错。
    """
    assert [phase.name for phase in ASM_PHASES] == ["CRUISE", "POPUP", "DIVE"]
    first, second, third = ASM_PHASES
    assert first.until is not None and first.until.variable == "range"
    assert second.until is not None
    assert second.until.variable == "target_elevation"
    assert second.until.op == "<"
    assert second.until.value == -20.0
    # 俯冲段不夹过载（留给弹体）——照抄巡航弹末段那个 30 时实测打不中
    assert third.max_accel is None
    # 前两段的横向律都一样：**航向指目标**。这不是"照抄参照"——原脚本那两段
    # 跟的是预规划航路（``allow_route_following true``），我们没有那个能力
    # （§9-31），直扑是"没做横向规划"的默认行为。
    assert first.horizontal == HORIZ_TARGET
    assert second.horizontal == HORIZ_TARGET


def test_popup_then_dive_splits_on_the_target_angle_not_on_our_own_climb() -> None:
    """跃升段**在爬升**时切进俯冲——``descending`` 在这里永远不成立。

    这一条直接钉住 §9-33：AFSIM 反舰弹写的是
    ``next_phase DIVE when target_elevation < -20 deg``（"跃升到目标掉到视线
    下方 20°"），而它回答的是"**看目标**的角度"，不是"**自己**在不在下降"。
    补 ``target_elevation`` 之前，这张表飞不出 POPUP/DIVE 两段。
    """
    # 弹体要拉得出 **4.6 g** 的垂向过载。参照那枚反舰弹在 POPUP 段用 6 s 把
    # 弹道倾角从 0 抬到 67.6°，折合约 4.8 g（§5.11）；`CRUISE_BODY` 默认的
    # 12 m/s²（1.2 g）只够抬到 15°，弹会在目标**头顶 750 m** 掠过。实测扫过
    # 一遍：12 / 30 m/s² 都不命中（最近 754 / 804 m），**45 起才落进杀伤半径**
    # （28.8 m）。所以"这张表"与"这具弹体"是配套的——换表也得看弹拉不拉得动。
    rig = build({**CRUISE_BODY, "radial_accel": 45.0},
                guidance_type="ASM_GUIDANCE",
                guidance_params={"target_name": TARGET_NAME},
                target=near((0.0, 0.0, 0.0), 40_000.0))
    assert rig.guidance.phase_table() == ASM_PHASES      # 用的是生产那张表
    period_us = int(rig.body.spec["period"])
    seen: list[str] = []
    z_at: dict[str, float] = {}
    top = 0.0
    for k in range(6_000):
        rig.body.update(k * period_us)
        name = rig.guidance.phase_name()
        if not seen or seen[-1] != name:
            seen.append(name)
            z_at[name] = rig.view.z
        top = max(top, rig.view.z)
        if rig.body.arrived():
            break

    assert seen == ["CRUISE", "POPUP", "DIVE"]
    # 跃起来了：巡航只在离地 50 m，切进 DIVE 时已经高出它一个量级
    assert top > 500.0
    assert z_at["DIVE"] > z_at["POPUP"] + 100.0
    assert rig.guidance.hit()

    # 反过来：跃升**整段都在爬升**，所以 "自己在下降" 没有一帧成立——
    # 这两个判据回答的不是同一个问题，不能互相代替（这就是补之前的症状）。
    climbing = FlightState(vertical_speed=80.0, target_elevation=5.0)
    assert Until.event("descending").test(climbing) is False
    assert Until.below("target_elevation", -20.0).test(climbing) is False
    assert Until.below("target_elevation", -20.0).test(
        replace(climbing, target_elevation=-25.0)) is True


def test_cruise_missile_hits_a_static_target() -> None:
    """40 km 外一个静止目标：三段走完、命中、起爆距离在杀伤半径内。"""
    rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                target=near((0.0, 0.0, 0.0), 40_000.0))
    rig.fly()
    assert rig.guidance.hit() is True
    assert rig.guidance.terminate_reason() == "命中"
    assert rig.guidance.detonation_range_m() <= 30.0
    assert [row[0] for row in rig.guidance.timeline()] == [
        "LAUNCH", "CRUISE", "TERMINAL"
    ]
    # 巡航段贴地 50 m 离地高度（不是海拔——山地上"离地"才成立）
    assert rig.body.max_altitude_m() < 200.0


def test_cruise_missile_hits_at_any_azimuth_and_out_to_its_range() -> None:
    """40 km / 1000 km 上往东偏 0 与 10 km 都命中；2200 km 是射程尽头；20 km 是下边缘。

    （2200 km 只测正北：它一趟要 4600 帧，横向那一路在探针里扫过了——见
    ``guidance/cruise.py`` 的包线说明。）

    下边缘的来处：巡航高度只有 50 m 离地，而末段交接条件是 ``range < 20 km``。
    目标太近时**交接发生在发射爬升段里**（弹刚起来就进末段），于是这条
    20 km 就是"目标必须在交接距离之外"——它是**算法边界**，不是弹打不动。
    """
    for distance, east in ((40_000.0, 0.0), (40_000.0, 10_000.0),
                           (1_000_000.0, 0.0), (1_000_000.0, 10_000.0),
                           (2_200_000.0, 0.0)):
        rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                    target=offset((0.0, 0.0, 0.0), distance, east))
        rig.fly(frames=6000)
        assert rig.guidance.hit() is True, (distance, east)

    # 10 km：太小了，弹爬升都还没做完就该"交接"了
    too_close = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                      target=near((0.0, 0.0, 0.0), 10_000.0))
    too_close.fly(frames=3000)
    assert too_close.guidance.hit() is False
    assert too_close.body.terminate_reason() == "落地"


def test_cruise_missile_accuracy_beats_the_lethal_radius_by_an_order() -> None:
    """把杀伤半径收到 5 m 仍然命中 ⇒ **真实脱靶量** ≤ 5 m。

    实测第一通过距离 0.36~3.44 m。``detonation_range_m()`` 报的 25~29 m 是
    起爆瞬间的距离，不是脱靶量——两个数差一个量级，不要混。

    这条用例是**"落地盖掉命中"那个 bug 被查出来的地方**：末段一个子步走约
    5 m，而杀伤半径也只有 5 m，"最后一小步同时穿过引信点火区与地面"是常见
    情形。判据次序反着写的时候，100 km 那一发报的是"落地"——而它其实落在
    目标 0.4 m 处。40 km 那一发只是碰巧没撞上这个次序，所以**只有扫距离
    才能发现**。见 ``MissileMover._advance`` 里那段注释。
    """
    for distance in (40_000.0, 100_000.0, 200_000.0):
        rig = build(CRUISE_BODY, guidance_type="CRUISE_GUIDANCE",
                    guidance_params={"lethal_radius": 5.0},
                    target=near((0.0, 0.0, 0.0), distance))
        rig.fly(frames=3000)
        assert rig.guidance.hit() is True, (
            distance, rig.body.terminate_reason()
        )
        assert rig.guidance.detonation_range_m() <= 5.0


def test_cruise_missile_from_rest_is_rejected() -> None:
    """``launch_speed`` 非有不可：这台涡扇的推重比只有 0.14。

    没有初速时它**建立不起速度系**——所以那不是"飞得慢一点"，而是一个参数
    就错的配置，第一次推进就报错（见
    ``MissileMover._require_static_launch``：本件不建升力，法向加速度只能靠
    转动速度矢量兑现，而转动速率 ``a_n / V`` 在 V ≈ 0 时没有上界）。
    """
    with pytest.raises(ConfigurationError) as excinfo:
        rig = build({**CRUISE_BODY, "launch_speed": 0.0},
                    guidance_type="CRUISE_GUIDANCE",
                    target=near((0.0, 0.0, 0.0), 40_000.0))
        rig.body.update(0)
        rig.body.update(PERIOD_US)
    message = str(excinfo.value)
    assert "launch_speed" in message and "推重比" in message

    # 载机给的那份初速写在**实体速度**里同样成立——不是非用 launch_speed 不可
    dropped = build({**CRUISE_BODY, "launch_speed": 0.0},
                    guidance_type="CRUISE_GUIDANCE",
                    target=near((0.0, 0.0, 0.0), 40_000.0), speed=240.0)
    dropped.fly()
    assert dropped.guidance.hit() is True


# ---------------------------------------------------------------------------
# 端到端：弹道弹
# ---------------------------------------------------------------------------

def test_ballistic_free_flight_range_is_what_the_fuel_buys() -> None:
    """没有目标时它一路自由飞，射程就是燃料买来的那一段。

    "射程"不是参数——改 ``fuel_mass`` 才改射程（216 kg → 18.0 km，
    400 kg → 50.7 km，500 kg → 86.5 km；口径是"有制导、没有目标"，不是无控弹）。
    """
    rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE")
    rig.fly()
    assert rig.body.terminate_reason() == "落地"
    assert rig.guidance.phase_name() == "BALLISTIC"     # 从未进末段
    assert rig.guidance.terminated() is False
    downrange = hypot(rig.view.x, rig.view.y)
    assert 45_000.0 < downrange < 53_000.0, downrange


def test_ballistic_missile_hits_inside_its_envelope() -> None:
    """49 km 目标：四段走完、末段比例导引收口、命中。"""
    rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                target=near((0.0, 0.0, 0.0), 49_000.0))
    rig.fly()
    assert [row[0] for row in rig.guidance.timeline()] == [
        "LIFTOFF", "PITCH_OVER", "BALLISTIC", "TERMINAL"
    ]
    assert rig.guidance.hit() is True
    assert rig.guidance.detonation_range_m() <= 30.0
    assert rig.body.max_altitude_m() > 15_000.0


def test_ballistic_envelope_ends_are_computed_not_guarded() -> None:
    """包线两端：25 km 与 85 km 命中，20 km 与 90 km 打飞。

    这四个数都是**实测**的（见 ``guidance/ballistic.py`` 的包线表）。
    边界不是拦出来的：打飞的表现是"落地"，而落点差着几公里——把
    ``lethal_radius`` 调大也救不回来。
    """
    for distance, expect_hit in ((25_000.0, True), (85_000.0, True),
                                 (20_000.0, False), (90_000.0, False)):
        rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                    target=near((0.0, 0.0, 0.0), distance))
        rig.fly()
        assert rig.guidance.hit() is expect_hit, distance


def test_ballistic_missile_hits_a_target_off_the_launch_azimuth() -> None:
    """往东偏 10 km 的目标照样命中——**横向那条环必须真的接进去**。

    此前 ``V_c`` 反号时，同一个目标（30 km 处往东偏 500 m）会打飞 15.7 km。
    """
    for distance in (30_000.0, 49_000.0, 80_000.0):
        rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                    target=offset((0.0, 0.0, 0.0), distance, 10_000.0))
        rig.fly()
        assert rig.guidance.hit() is True, distance


def test_ballistic_accuracy_beats_the_lethal_radius_by_an_order() -> None:
    """把杀伤半径收到 5 m 仍然命中 ⇒ **真实脱靶量** ≤ 5 m。

    与巡航弹那条同源，但它此前**没有用例看着**——"5 m 也全中"这句话只写在
    ``guidance/ballistic.py`` 的包线表下面。实测十组（25/30/49/60/85 km ×
    正北 / 偏东 10 km）起爆距离 0.03~4.71 m，这里取四组当代表。

    量真实脱靶量只能在**子步**上采：弹体一帧一写（``_commit`` 在一帧末尾，
    2 s），从外面读位置得到的是 800 m 一个点，什么也量不出来。
    """
    for distance, east in ((25_000.0, 0.0), (49_000.0, 0.0),
                           (60_000.0, 10_000.0), (85_000.0, 0.0)):
        rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                    guidance_params={"lethal_radius": 5.0},
                    target=offset((0.0, 0.0, 0.0), distance, east))
        rig.fly(frames=6000)
        assert rig.guidance.hit() is True, (
            distance, east, rig.body.terminate_reason()
        )
        assert rig.guidance.detonation_range_m() <= 5.0


def test_ballistic_missile_falls_short_below_its_minimum_range() -> None:
    """20 km 在最小射程之下：程序转弯段转不过来的角度就是打不到的近距离。

    报出来的原因是"落地"而不是"命中"——包线边界是**算出来的**，不是拦出来的。
    """
    rig = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                target=near((0.0, 0.0, 0.0), 20_000.0))
    rig.fly()
    assert rig.guidance.hit() is False
    assert rig.body.terminate_reason() == "落地"


def test_ballistic_terminal_handover_needs_to_be_early_enough() -> None:
    """末段交接太晚就打飞：4 km 处 6 g 只够改 28° 方向。

    同一个目标（80 km），交接距离 4 km 打飞、44 km（参考值）命中——**这个
    参数是扫出来的，不是拍的**。
    """
    late = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                 guidance_params={"phases": _ballistic_phases(4_000.0)},
                 target=near((0.0, 0.0, 0.0), 80_000.0))
    late.fly()
    assert late.guidance.hit() is False

    early = build(BALLISTIC_BODY, guidance_type="BALLISTIC_GUIDANCE",
                  guidance_params={"phases": _ballistic_phases(44_000.0)},
                  target=near((0.0, 0.0, 0.0), 80_000.0))
    early.fly()
    assert early.guidance.hit() is True


def _ballistic_phases(handover_m: float) -> tuple[Phase, ...]:
    """参考阶段表，只换末段交接距离（其余一个字都不动）。"""
    return (
        Phase("LIFTOFF", until=Until.after(3.0)),
        Phase("PITCH_OVER", until=Until.event("burnout"),
              flight_path_angle=50.0, vertical=VERT_FPA),
        Phase("BALLISTIC", until=Until.below("range", handover_m),
              vertical=VERT_FREE),
        Phase("TERMINAL", vertical=VERT_PN, horizontal=HORIZ_PN,
              gain=4.0, max_accel=60.0),
    )


def test_speed_ceiling_is_a_guard_not_a_cruise_speed() -> None:
    """``max_speed`` 是防跑飞的闸：卡住它时"射程不够"表现为物理受限，
    而不是参数没生效。"""
    rig = build({**BALLISTIC_BODY, "max_speed": 500.0})
    rig.fly()
    assert rig.body.max_speed_mps() <= 500.0 + 1e-6
    assert hypot(rig.view.x, rig.view.y) < 30_000.0


# ---------------------------------------------------------------------------
# 参考实现的阶段表本身
# ---------------------------------------------------------------------------

def test_reference_phase_tables_are_exported_and_well_formed() -> None:
    """三张参考阶段表：名字不重复、只有最后一段不带推进条件。"""
    for table in (ASM_PHASES, CRUISE_PHASES, BALLISTIC_PHASES):
        names = [phase.name for phase in table]
        assert len(names) == len(set(names))
        assert all(phase.until is not None for phase in table[:-1])
        assert table[-1].until is None


def test_every_phase_names_a_law_that_exists() -> None:
    """垂向律/水平律都必须是定义过的字符串——拼错会让那一句静默失效。"""
    from milsim.models.guidance.contract import HORIZONTAL_LAWS, VERTICAL_LAWS

    for table in (ASM_PHASES, CRUISE_PHASES, BALLISTIC_PHASES):
        for phase in table:
            assert phase.vertical in VERTICAL_LAWS
            assert phase.horizontal in HORIZONTAL_LAWS


def test_guidance_is_a_component_with_contract_params() -> None:
    """制导件是组件：参数表是契约，且**没有**自己的 update 节拍。"""
    assert issubclass(GuidanceComputer, object)
    for name in ("phases", "target_name", "lethal_radius", "max_flight_time"):
        assert name in GuidanceComputer.PARAMS
    assert GuidanceComputer.SLOT_HINT == "guidance"
    # **只有弹体注册节拍，制导件一条也没有**：它由弹体的子步循环叫（§5.8 的
    # 分工）。制导件自己挂一条 0.02 s 的周期事件的话，整个引擎的步长都会被
    # 它拉到 0.02 s（§2.4）。
    rig = build({}, guidance_type="CRUISE_GUIDANCE",
                guidance_params={"target_name": ""})
    assert [interval for interval, _priority in rig.mount.schedules] == [
        int(rig.body.spec["period"])
    ]


def test_missile_is_registered_under_its_type_name() -> None:
    """注册表里认得出弹体与三个参考制导件。"""
    factory = ComponentFactory()
    assert factory.resolve_class("MISSILE_MOVER") is MissileMover
    for name in ("ASM_GUIDANCE", "CRUISE_GUIDANCE", "BALLISTIC_GUIDANCE"):
        assert issubclass(factory.resolve_class(name), GuidanceComputer)
