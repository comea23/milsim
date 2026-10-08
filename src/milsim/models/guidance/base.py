"""弹的制导件：按阶段剧本把"往哪飞"算成一小步加速度指令（§5.8）。

它是**被叫的**，不是自己跑的
---------------------------
AFSIM 的 ``WSF_GUIDANCE_COMPUTER`` 是"被 mover 每个积分步调用"的处理器，
它自己不注册节拍。这里照搬——理由不只是抄得像，而是**引擎的步进由最近事件
决定**（§2.4：``t_target = min(队列最近事件, 栅栏)``）：

- 制导件要是自己按 0.02 s 注册节拍，**整个引擎的步长都会被拉到 0.02 s**，
  几千个实体的推演会慢两个数量级；
- 而"末端比例导引要 0.02 s 的闭环"这件事是真的需要。

解法就是文档里那条：需要高精度的模型**在自身 update 内做子步积分**。弹体每个
子步调一次 :meth:`command`，制导闭环因此活在弹体的一帧之内，全局时间轴不动。
代价是制导件的 :meth:`~milsim.models.component.Component.update` 不被使用——
它没有自己的节拍，这一点在 :meth:`initialize` 里没什么可注册的，看着有点空，
但那是这个分工的必然结果。

三道校验，都是"宁可装配期报错"的
--------------------------------
1. **阶段表非空**。空表 = 这枚弹没有剧本，它飞不出任何形状。
2. **最后一段不写推进条件**。写了也没有下一段可去，那句话不会生效——
   静默无效的配置比报错糟糕得多。
3. **``target_name`` 必须真实存在**。写错一个名字的表现是弹笔直飞向原点，
   而"那是打歪了"与"那是根本没找到目标"在结果里长得一模一样。

增益与限幅的归属
----------------
制导件夹一道（本阶段 ``max_accel``）、弹体再夹一道（弹体自己的
``radial_accel``），**取更严的那个**。分开不是为了重复：前者是"这一阶段允许
拉几个 g"（战术选择），后者是"这具弹体拉得出几个 g"（性能）。中间那段
可以互相覆盖着调，而两条约束各自有唯一的出处。
"""

from __future__ import annotations

from dataclasses import replace
from math import atan2, cos, degrees, hypot, inf, radians, sin, sqrt
from typing import Any

from ...errors import ConfigurationError
from ...services.params import Params, STANDARD_GRAVITY
from ..component import Component
from .contract import (
    HORIZ_HOLD,
    HORIZ_PN,
    HORIZ_TARGET,
    FlightState,
    GuidanceCommand,
    Phase,
    VERT_ALTITUDE,
    VERT_BALANCE,
    VERT_FPA,
    VERT_FREE,
    VERT_PN,
)

#: 命中原因写在诊断里，用同一份字符串比较（不要到处写中文字面量）。
REASON_HIT = "命中"
REASON_TIMEOUT = "飞太久"


def wrap180(angle: float) -> float:
    """把角度折进 ``(-180, 180]``。方位偏差算错一圈是 360° 的错。"""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


class GuidanceComputer(Component):
    """制导件基类（抽象层）。参考实现见 :mod:`milsim.models.guidance.cruise`
    与 :mod:`milsim.models.guidance.ballistic`。

    它**只管飞行**：命中判定用"到目标的距离 ≤ ``lethal_radius``"，至于打中了
    之后毁伤多少、谁掉血，那是交战裁决器的事（M4b）——本件不碰战损。
    """

    SLOT_HINT = "guidance"

    PARAMS = {
        #: **阶段表**——这一型弹怎么飞。它是参数而不是想定语法，理由见
        #: :mod:`milsim.models.guidance.contract` 的模块说明。
        "phases": Params.phases(),
        #: 目标平台名（想定里的 ``platform`` 名）。留空 = 由运行期指令点指定
        #: （``assign_target_point``）。**名字写错会在装配期报错**，不会
        #: 变成"弹笔直飞向原点"。
        "target_name": Params.string(""),
        #: 命中判据（米）：到目标的距离小于它就判命中。近炸引信的作用距离，
        #: 或直接就是战斗部杀伤半径——这里不区分，因为毁伤是 M4b 的事。
        "lethal_radius": Params.distance(30.0, minimum=0.0),
        #: 比例导引的默认增益 ``N``（AFSIM 的 ``proportional_navigation_gain``
        #: 默认 3）。阶段可以用 ``gain`` 覆盖。
        "intercept_gain": Params.number(3.0, minimum=0.0),
        #: 指向目标的默认增益（1/秒）——航向环路的时间常数是它的倒数。
        #: 它是**追踪导引**（AFSIM 的 ``velocity_pursuit_gain``），对付静止
        #: 目标够用；对付机动目标要用比例导引。
        "pursuit_gain": Params.number(1.0, minimum=0.0),
        #: 高度外环增益（1/秒）：每米高度差要多少垂直速率。
        "altitude_gain": Params.number(0.03, minimum=0.0),
        #: 高度内环增益（1/秒）：垂直速率差要多少垂向加速度。
        #: 内外环差一个量级，串起来才不会互相打架（外环 ≈ 30 s、内环 ≈ 1 s）。
        "climb_gain": Params.number(1.0, minimum=0.0),
        #: 弹道倾角环路增益（1/秒）：倾角误差要多少转向率。
        "fpa_gain": Params.number(1.0, minimum=0.0),
        #: 兜底：飞这么久还没命中/落地就判"失的"。它不是模型参数，
        #: 是一道**防跑飞**的闸——没有它的表现是推演永远跑不完。
        "max_flight_time": Params.duration(10_800_000_000, minimum=1),
    }

    __slots__ = (
        "_view",
        "_phases",
        "_index",
        "_phase_started_s",
        "_timeline",
        "_done",
        "_reason",
        "_detonation",
        "_point",
        "_target_id",
        "_lethal",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self._view = mount.engagement_view()
        self._phases: tuple[Phase, ...] = tuple(self.spec["phases"])
        if not self._phases:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 阶段表是空的——"
                "没有剧本的制导件什么也做不了"
            )
        last = self._phases[-1]
        if last.until is not None:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 最后一个阶段 "
                f"{last.name!r} 写了推进条件（{last.until.describe()}），"
                "但后面没有阶段可去——那句话不会生效。"
                "不推进的写法是把 until 留空"
            )

        self._index = 0
        self._phase_started_s = 0.0
        self._timeline: list[list[Any]] = [[self._phases[0].name, 0.0, None]]
        self._done = False
        self._reason = ""
        self._detonation = inf
        self._point: tuple[float, float, float] | None = None
        self._lethal = float(self.spec["lethal_radius"])

        name = str(self.spec["target_name"])
        self._target_id: int | None = None
        if name:
            found = mount.registry.by_name(name)
            if found is None:
                raise ConfigurationError(
                    f"{self.type_name or type(self).__name__}: 目标 {name!r} 不在"
                    "这份想定里——检查 platform 名字是否写错"
                )
            self._target_id = found.entity_id

    # -- 运行期：目标 ------------------------------------------------------

    def assign_target_point(self, x: float, y: float, z: float) -> None:
        """直接指定打击点（世界坐标）。用于没有目标实体、或打"某个坐标"的场合。"""
        self._point = (float(x), float(y), float(z))

    def target_point(self) -> tuple[float, float, float] | None:
        """目标当前所在。**取的是真值**。

        真值在这里不是偷懒：感知件（M4a-2）还没落地，而"导引头能看到多少"
        属于感知模型的事。本件接收的是一份目标位置，它从哪来是上层的事——
        换成航迹只要把这一处换掉。
        """
        if self._point is not None:
            return self._point
        if self._target_id is None or self._view is None:
            return None
        return self._view.position_of(self._target_id)

    # -- 运行期：一帧指令（弹体调） ----------------------------------------

    def command(self, state: FlightState) -> GuidanceCommand:
        """弹体在一个子步开始处调用。返回这一小步要什么。

        顺序是**先判终止、再推进阶段、最后算律**：终止要抢先，否则弹在命中
        的那一小步还在拉杆。
        """
        if self._done:
            return GuidanceCommand(
                phase=self.phase_name(), thrusting=False,
                terminate=True, reason=self._reason,
            )

        target = self.target_point()
        geo = self._geometry(state, target)
        phase = self._phases[self._index]

        st = replace(
            state,
            phase_time=max(0.0, state.flight_time - self._phase_started_s),
            target_range=geo[0],
            closing_speed=geo[1],
            target_heading=geo[2],
            target_elevation=geo[3],
            target_azimuth_rate=geo[4],
            target_elevation_rate=geo[5],
        )

        if target is not None and geo[0] <= self._lethal:
            return self._stop(REASON_HIT, geo[0], phase)
        if state.flight_time >= float(self.spec["max_flight_time"]) / 1_000_000.0:
            return self._stop(REASON_TIMEOUT, geo[0], phase)
        # 没进引信点火区就一直飞。**不要把"飞过目标"也判成终止**：那样
        # 报出来的会是"第一次进入杀伤半径那一刻的距离"，而不是真的脱靶量
        # （见 :meth:`detonation_range_m`），而且一头扎进地面时那个判据根本
        # 来不及成立——弹先撞地，"命中"就变成了"落地"。

        st, phase = self._advance(st)
        return self._law(st, phase, target)

    def _stop(self, reason: str, detonation: float, phase: Phase) -> GuidanceCommand:
        """终止：**只记状态，不动弹体**。

        弹体拿到 ``terminate=True`` 之后自己停下——它还要把位置、速度、空间
        索引收拾干净。制导件去改弹体的位置，等于把"能飞多远"的决定权从弹体
        手里拿走，而弹体才是知道地形与落点的那一个。
        """
        self._done = True
        self._reason = reason
        self._detonation = detonation
        return GuidanceCommand(
            phase=phase.name, thrusting=False, terminate=True, reason=reason
        )

    # -- 阶段推进 ----------------------------------------------------------

    def _advance(self, st: FlightState) -> tuple[FlightState, Phase]:
        """把该跳的阶段跳掉，返回（补全后的状态，当前阶段）。

        用循环而不是 if：一帧里跨两个阶段是可能的（比如助推段很短、而弹体
        的 tick 比它长）。循环次数**封顶**在阶段数——``Until.after(0)`` 这种
        写法会让"进入新阶段"立刻又满足条件，没有封顶就是死循环。
        """
        for _ in range(len(self._phases)):
            phase = self._phases[self._index]
            if phase.until is None or not phase.until.test(st):
                return st, phase
            if self._index + 1 >= len(self._phases):
                return st, phase
            self._timeline[-1][2] = st.flight_time
            self._index += 1
            self._phase_started_s = st.flight_time
            self._timeline.append([self._phases[self._index].name, st.flight_time, None])
            st = replace(st, phase_time=0.0)
        return st, self._phases[self._index]

    # -- 导引律 ------------------------------------------------------------

    def _law(
        self, st: FlightState, phase: Phase, target: tuple[float, float, float] | None
    ) -> GuidanceCommand:
        vx, vy, vz = self._velocity(st)
        speed = sqrt(vx * vx + vy * vy + vz * vz)
        cos_gamma = cos(radians(st.flight_path_angle))
        gravity_term = STANDARD_GRAVITY * cos_gamma

        vertical = self._vertical_law(st, phase, gravity_term, speed)
        lateral = self._horizontal_law(st, phase, target)

        # 过载限幅夹的是**机动那一份**：平飞时抵消重力的那 1 g 不占过载预算，
        # 否则一架只能拉 1 g 的飞机连平飞都维持不了。
        cap = phase.max_accel
        if cap is not None:
            vertical, lateral = self._clip(vertical - gravity_term, lateral, cap, gravity_term)

        return GuidanceCommand(
            phase=phase.name,
            thrusting=bool(phase.motor),
            speed_mps=phase.speed,
            lateral_accel=lateral,
            vertical_accel=vertical,
        )

    @staticmethod
    def _clip(
        vertical_maneuver: float, lateral: float, cap: float, gravity_term: float
    ) -> tuple[float, float]:
        need = hypot(vertical_maneuver, lateral)
        if need <= cap or need <= 0.0:
            return gravity_term + vertical_maneuver, lateral
        scale = cap / need
        return gravity_term + vertical_maneuver * scale, lateral * scale

    def _vertical_law(
        self, st: FlightState, phase: Phase, gravity_term: float, speed: float
    ) -> float:
        """本阶段的垂向加速度（**总**量，含抵消重力的那一份）。"""
        if phase.vertical == VERT_FREE:
            # 没人管垂直：只剩重力。弹道弹的中段就是它。
            return 0.0
        if phase.vertical == VERT_BALANCE:
            return gravity_term
        if phase.vertical == VERT_ALTITUDE:
            assert phase.altitude is not None       # Phase 已校验
            z_cmd = phase.altitude
            if phase.altitude_agl:
                z_cmd += st.ground_z
            vz_cmd = float(self.spec["altitude_gain"]) * (z_cmd - st.z)
            error = vz_cmd - st.vertical_speed
            return gravity_term + float(self.spec["climb_gain"]) * error
        if phase.vertical == VERT_FPA:
            assert phase.flight_path_angle is not None
            error = radians(phase.flight_path_angle - st.flight_path_angle)
            return gravity_term + speed * float(self.spec["fpa_gain"]) * error
        if phase.vertical == VERT_PN:
            gain = self._gain(st, phase)
            return gain * st.closing_speed * st.target_elevation_rate + gravity_term
        raise ConfigurationError(f"阶段 {phase.name}: 垂向律 {phase.vertical!r} 没实现")

    def _horizontal_law(
        self, st: FlightState, phase: Phase, target: tuple[float, float, float] | None
    ) -> float:
        """本阶段的水平横向加速度（m/s²，正 = 右转）。"""
        if phase.horizontal == HORIZ_HOLD:
            return 0.0
        if target is None:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 阶段 {phase.name!r} 要用"
                f" {phase.horizontal!r} 导引，但制导件没有目标——"
                "写 target_name，或在运行期调 assign_target_point()"
            )
        if phase.horizontal == HORIZ_TARGET:
            error = wrap180(st.target_heading - st.heading)
            h_speed = hypot(st.speed, 1e-12)
            return float(self.spec["pursuit_gain"]) * h_speed * radians(error)
        if phase.horizontal == HORIZ_PN:
            gain = self._gain(st, phase)
            return gain * st.closing_speed * st.target_azimuth_rate
        raise ConfigurationError(
            f"阶段 {phase.name}: 水平律 {phase.horizontal!r} 没实现"
        )

    def _gain(self, st: FlightState, phase: Phase) -> float:
        if phase.gain is not None:
            return phase.gain
        if phase.horizontal == HORIZ_PN or phase.vertical == VERT_PN:
            return float(self.spec["intercept_gain"])
        return float(self.spec["pursuit_gain"])

    # -- 几何 --------------------------------------------------------------

    @staticmethod
    def _velocity(state: FlightState) -> tuple[float, float, float]:
        """速度矢量。水平分量由"速率 + 航向"得到，垂向分量单独存。

        与 :meth:`~milsim.services.store.KinematicsTable.advance` 用同一套
        几何（``dx = d·sinθ``、``dy = −d·cosθ``），否则"弹走的方向"与
        "运动学表以为的方向"会差一个镜像。
        """
        heading = radians(state.heading)
        return (
            state.speed * sin(heading),
            -state.speed * cos(heading),
            state.vertical_speed,
        )

    @staticmethod
    def _geometry(
        state: FlightState, target: tuple[float, float, float] | None
    ) -> tuple[float, float, float, float, float, float]:
        """一次算全目标几何（六项，**两个位置量在前、两个速率量在后**）：

        .. code-block:: text

            (斜距, 接近速率, 方位角°, 高低角°, 方位角速率, 高低角速率)

        前五项与弹体侧无关、只由视线决定，第六项（高低角）是后补的：AFSIM 的
        ``POPUP`` 段用 ``target_elevation < -20 deg`` 切进 ``DIVE``，而推进条件
        只认 :class:`FlightState` 里的量。它是 ``λ̇_v`` 的位置对照物，两者必须
        从**同一支视线**上取——分开推一遍的话，一个用 ``r_xy`` 一个用 ``r``，
        症状是"条件看着会成立、实际差几度"（§9-33）。

        没有目标时返回 ``(inf, 0, 0, 0, 0, 0)``——推进条件里的 ``range`` 因此是
        ``inf``，"``range < 5000``"这条永远不会成立，而不会变成 0 让弹立刻
        进入末段；``target_elevation`` 是 ``0.0``，所以 "``< -20``" 与
        "``> 89``" 也都不成立。

        **接近速率是"距离在变小"的速率**，所以正数表示在靠近：推进条件里
        ``closing_speed > 0`` 读起来才是"在接近"，比例导引里 ``V_c > 0``
        也才是**负反馈**（``a = N·V_c·λ̇`` 的这个符号是整套导引律的根）。

        两个视线角速率是比例导引的 λ̇（``r_xy`` 是水平投影距离）：

        .. code-block:: text

            方位角 λ_h = atan2(dx, −dy)     ← 与 heading 同一套约定
                λ̇_h = dλ_h/dt = (dy·vx − dx·vy) / r_xy²
            高低角 λ_v = atan2(dz, r_xy)
                λ̇_v = dλ_v/dt = (dz·(dx·vx + dy·vy)/r_xy − r_xy·vz) / r²
            V_c  = −d|r|/dt = (dx·vx + dy·vy + dz·vz) / r

        ★ **``V_c`` 与两个 λ̇ 的符号必须同时是对的**。垂向 PN 用的是
        ``N·V_c·λ̇_v``，而 ``V_c`` 与 ``λ̇_v`` 各自反一次号会在那里**互相
        抵消**：于是"垂向看起来正常工作"完全不能证明符号是对的——它只证明
        反了两次。**实测过**：修之前那版把 ``V_c`` 与 ``λ̇_v`` 都写反了，
        垂向照常命中，而横向的 ``N·V_c·λ̇_h`` 只有 ``V_c`` 反号 ⇒ 正反馈：
        打正北的目标 14.6 m，**目标往东偏 500 m 就变成偏 15.7 km**——
        也就是说那枚弹只能打正好在发射方位线上的目标。两个符号一起改回来
        之后，30/49 km 上往东偏 0.5~20 km 全部命中。

        两个分母在对应方向退化成 0 时取 0 而不是让它发散：那两个**视线角速率**
        本来就是"没有定义"，而发散的一条指令会把弹甩出去。

        高低角在"视线竖直"那一支里是个例外——**它有定义**，就是 ±90°。所以
        那一支只把无定义的三项（方位角、两个角速率）归 0，高低角照实报：
        正下方的目标必须报 −90°，否则一个"目标掉到视线下方 20°"的判据会在
        最该成立的地方反而报成"目标在水平线上"。

        ``elevation`` 与 ``elevation_rate`` 的**符号约定同源**（都按
        ``atan2(dz, r_xy)``），所以 ``target_elevation < 0`` 与
        ``λ̇_v`` 的符号在"目标在下方"时是一致的——这一点值得单独钉住，因为
        垂向 PN 那两个量反号会互相抵消（见上）。
        """
        if target is None:
            return inf, 0.0, 0.0, 0.0, 0.0, 0.0

        dx = target[0] - state.x
        dy = target[1] - state.y
        dz = target[2] - state.z
        vx = state.speed * sin(radians(state.heading))
        vy = -state.speed * cos(radians(state.heading))
        vz = state.vertical_speed

        r_xy = hypot(dx, dy)
        distance = sqrt(r_xy * r_xy + dz * dz)
        if distance <= 1e-9:
            # 弹就在目标上：连"从哪看"都没有了，六项全 0。
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # 正 = 在靠近。**不带负号**：d|r|/dt 本身是负的（在接近），而这里要
        # 的是它的相反数（见上面那条 ★）。
        closing = (dx * vx + dy * vy + dz * vz) / distance
        if r_xy <= 1e-9:
            # 视线竖直：**方位角**没有定义（水平面里没有方向），高低角有——
            # 正上/正下就是 ±90°。
            return distance, closing, 0.0, 90.0 if dz >= 0.0 else -90.0, 0.0, 0.0

        azimuth = degrees(atan2(dx, -dy))
        elevation = degrees(atan2(dz, r_xy))
        azimuth_rate = (dy * vx - dx * vy) / (r_xy * r_xy)
        elevation_rate = (
            dz * ((dx * vx + dy * vy) / r_xy) - r_xy * vz
        ) / (distance * distance)
        return distance, closing, azimuth, elevation, azimuth_rate, elevation_rate

    # -- 诊断 --------------------------------------------------------------

    def phase_name(self) -> str:
        return self._phases[self._index].name if self._phases else ""

    def phase_index(self) -> int:
        return self._index

    def phase_table(self) -> tuple[Phase, ...]:
        return self._phases

    def timeline(self) -> list[tuple[str, float, float | None]]:
        return [(str(row[0]), float(row[1]), row[2]) for row in self._timeline]

    def terminated(self) -> bool:
        return self._done

    def terminate_reason(self) -> str:
        return self._reason

    def hit(self) -> bool:
        return self._reason == REASON_HIT

    def detonation_range_m(self) -> float:
        """**起爆瞬间**到目标的距离（米）。没终止是 ``inf``。

        它不是脱靶量。引信模型是"距离**首次**小于 ``lethal_radius`` 就起爆"，
        于是这个数总是略小于 ``lethal_radius``——哪怕弹是正正穿过目标。
        实测两个参考实现（``ballistic.py`` 与 ``cruise.py`` 的包线表；
        v0.13.22 加的 ``asm.py`` **没有**做过包线扫描，它的结论只在
        ``demos/afsim_mover/README.md`` §11 那一个算例上）：``lethal_radius 30``
        时报 23~30 m，而把引信收到 **5 m 仍然全部命中**（25~85 km 的弹道弹、
        20~2200 km 的巡航弹）。

        要量真实脱靶量：把 ``lethal_radius`` 设成 0（引信不点火），弹会一直
        飞下去，然后算它到目标的最小距离。**★ 那个最小值只能在子步上采**：
        弹体是**一帧一写**的（``_commit`` 在 ``_advance`` 末尾，一帧 2 s），
        从外面读位置得到的是每隔 ``2 s × 400 m/s = 800 m`` 一个点，量什么都
        量不出来。子步分辨率要么把 ``substep`` 调到很小再逐帧读，要么给
        ``MissileMover._step`` 挂一层（``guidance/ballistic.py`` 的包线表
        就是这么量的）。**读那个数还要留神采样间隔**：子步 0.02 s，末段
        200~400 m/s ⇒ 采样间距 4~8 m，所以量出来的 ~2 m 是**采样精度**而不是
        脱靶量（轨迹穿过目标点、而采样落在最近点前后 2 m 时，读数也是 ~2 m）。
        真正说明精度的是"``lethal_radius`` 收到 5 m 仍然命中"这件事。

        为什么不做成"飞到最近点再起爆"：**弹会先撞地**。俯冲角 60° 时最近点
        通常就在地面附近，等下一个子步再判"距离开始变大"，弹已经在地下了，
        于是"命中"变成"落地"。
        """
        return self._detonation

    def describe(self) -> str:
        return f"{self.type_name or type(self).__name__}({self.slot})"


__all__ = ["REASON_HIT", "REASON_TIMEOUT", "GuidanceComputer", "wrap180"]
