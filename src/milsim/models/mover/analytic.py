"""解析弹道机动件：不开普勒积分、不逐子步推——整条弹道**解算出来**（§5.8）。

它为什么存在
------------
``MissileMover`` 是平面 3-DOF：直角坐标、常数 ``g``、无地球曲率。打 85 km
它够准（实测包线 25~85 km），打洲际（10000 km 级、弹道高点 1200 km 量级）
它**物理上不成立**——那一档 ``g`` 要变三成，平面几何把四分之一个地球当成
一张平纸。逐子步积分这条路要球面化，动的是 ``KinematicsTable`` 与全部实体
坐标；本件走另一条：**弹道不在仿真里"飞"出来，而是在第一次推进前一次
解算出来**——给定发射点、目标点与弹道高点，球面椭圆弹道有闭式解，之后
每一帧只是按开普勒时间律**置位**。积分的物理错误从根上就不参与。

一句话：``MissileMover`` 回答"这一小步怎么飞"，本件回答"第 t 秒它该在哪"。

弹道参数化：一个形状参数 + 目标位置
----------------------------------
对称弹道（发射点与落点关于弹道顶点对称，`ν₁ = π − Φ/2`、`ν₂ = π + Φ/2`，
`Φ` 是地心射程角）的全部几何由两个量决定：

.. code-block:: text

    Φ   地心射程角 = 平面距离 ÷ R⊕          （目标位置给）
    rₐ  弹道顶点地心距 = R⊕ + apogee_altitude（参数给）

椭圆偏心率**闭式**（把两个端点条件代进 `r = p/(1−e·cosν)` 消去 p——顶点
在 ν=π，是**远**地点，所以分母带负号）：

.. code-block:: text

    e = (r₁ − rₐ) / (r₁·cos(Φ/2) − rₐ)      r₁ = R⊕ + z₀（发射点半径）
    p = rₐ·(1 − e)      a = p/(1 − e²)      n = √(μ/a³)      h = √(μ·p)

速度沿轨道的分量（真近点角 `ν` 处）：

.. code-block:: text

    v_r = (μ/h)·e·sinν        （径向 = 垂向速率）
    v_t = h/r                 （横向）
    v²  = v_r² + v_t²         （与 vis-viva 互相印证，测试里有对拍）

时间律是开普勒方程 `M = E − e·sinE`（每帧牛顿解，收敛判据 1e-12，不收敛
报错——宁可炸，不给一条静默歪掉的弹道）。**物理锚点**（测试钉住）：
Φ=90°、弹道高点 1200 km ⇒ 发射速度 7.2 km/s、全程 31.2 min——与真实
洲际弹道的量级一致（7 km/s 级、30 min 级）。平面模型给不出这两个数
（它要 9.9 km/s 才能飞 10000 km），这正是本件存在的理由。

三个**已知简化**，都写在明处
----------------------------
1. **主动段不建模**。真实弹用 100~200 s 把弹从 0 加速到关机速度；本件从
   发射点起就以轨道速度运动（Φ=90° 档首帧 γ≈21°、7.2 km/s）。早期预警段
   要"哪一秒爬到什么高度"的读者要知道这段是失真的；换真主动段等分级与
   推力程序进来再说。
2. **目标位置解算时锁定**。洲际弹打的是发射前装订的坐标，飞行中不跟随
   移动目标——这不是偷懒，是这一档弹的作战方式。
3. **发射/落点取同一半径** `r₁ = R⊕ + max(z₀, 0)`。弹道解算在球面上，
   落点高度因此等于发射点高度；平原（z₀=0）就是海平面，高原上两头的 z
   与当地地面一致，中间的剖面是椭圆——那一米的出入比主动段的简化小
   两个量级。

引擎口径的三条对齐
------------------
1. **平面距离 = 地面弧长**：`s = √(Δx²+Δy²)`，`Φ = s/R⊕`，反过来地面轨迹
   弧长 `σ = R⊕·(ν−ν₁)` 再沿发射方位线摆回平面坐标（`x = x₀+σ·sinA`、
   `y = y₀−σ·cosA`）。两头用同一个映射，弹的落点**精确**等于目标的平面
   坐标（不是"投影误差"——投影在两边抵消）。
2. **存储的 speed 是水平分量**（与 ``MissileMover`` 同一条约定）：本件写
   `σ̇ = R⊕·ν̇ = v_t·R⊕/r`（地面弧长速率，`z=0` 时退化为 `v_t`），垂向
   速率口径对齐 :meth:`vertical_speed_mps` 的读者（径向速度 `v_r`）。
3. **航向恒为发射方位**：弹道面固定，平面投影里就是一条直线，`heading`
   从头到尾不变。这不是"不会转弯"，是这条弹道本来就不拐弯。

终止与命中
----------
解算保证落点 = 目标，所以终止原因几乎恒为**命中**（`detonation_range_m`
≈ 0）。要"打偏"只有一种途径：装配后**移动目标平台**——解算锁的是旧
位置，弹落在旧位置上。`lethal_radius` 的口径与 ``GuidanceComputer`` 一致：
终点到目标的距离 ≤ 它才判命中，否则"落地"。

节拍：继承 ``Mover`` 默认 5 s（洲际全程 ~360 帧）。每帧一次开普勒牛顿
解，成本可忽略——**不要**把它调小来"提高轨迹精度"，轨迹精度由牛顿
收敛判据（1e-12）决定，与节拍无关；节拍只决定别人多久看它一眼。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, pi, radians, sin, sqrt
from typing import Any

from ...errors import ConfigurationError
from ...services.params import Params
from ...services.type_registry import register_component
from .base import Mover

#: 地球平均半径（米）。球面弹道的唯一几何常数。
R_EARTH_M = 6_371_000.0
#: 地球引力常数 μ = GM（m³/s²）。WGS-84 值，与 R⊕ 一起构成弹道的时间基准。
EARTH_MU = 3.986_004_418e14

#: 解算的射程上限（度）。Φ→180° 时椭圆退化（e→1），170° 留出余量；
#: 超过的射程在球面上也早就不走"最省能量的对称弹道"了。
MAX_RANGE_DEG = 170.0

#: 发射点与目标重合的判据（米）。比 1 米大是为了容忍落格取整。
MIN_RANGE_M = 100.0

#: 开普勒牛顿迭代的收敛判据（弧度）与次数上限。不收敛就报错：
#: 一条静默歪掉的弹道比一次推演崩溃糟得多。
KEPLER_TOL = 1e-12
KEPLER_MAX_ITER = 32

#: 命中原因写在诊断里，与 ``GuidanceComputer`` 用同一对字符串口径。
REASON_HIT = "命中"
REASON_LAND = "落地"


@dataclass(frozen=True, slots=True)
class _Solution:
    """一次解算的全部常量。之后每帧只按 ``t`` 重放，不再解几何。"""

    #: 发射点（平面坐标与高程基准）。
    x0: float
    y0: float
    z0: float
    #: 发射方位（度，0 = 北，90 = 东）。航向全程用它。
    azimuth_deg: float
    #: 轨道根数：半长轴、偏心率、平均角速度、角动量。
    a: float
    e: float
    mean_motion: float
    ang_mom: float
    #: 发射时刻的真近点角（= π − Φ/2）与平近点角（时间原点在发射时刻）。
    nu1: float
    M1: float
    #: 地心射程角（弧度）与总飞行时间（秒）。
    phi: float
    t_end: float
    #: 总地面弧长（米）＝ R⊕·Φ ＝ 发射点到目标的平面距离。落点校验用。
    sigma_end: float


@register_component("ANALYTIC_BALLISTIC_MOVER")
class AnalyticBallisticMover(Mover):
    """解析弹道弹体：发射前解出整条球面弹道，之后每帧按时间律置位。

    **它没有制导件、也不要阶段表**——弹道由"目标位置 + 弹道高点"唯一
    决定，没有"想怎么飞"的自由度。目标写在 ``target_name``（装配期校验
    存在性），或运行期 :meth:`assign_target_point` 给坐标（打"某个点"的
    场合，与 ``GuidanceComputer`` 同一条语义）。
    """

    SLOT_HINT = "mover"

    #: 不吃地形：洲际弹道大部分时间在大气层外，战区外也照样飞（与
    #: ``MissileMover.PROFILE = None`` 同一条理由）。
    PROFILE = None

    PARAMS = {
        **Mover.PARAMS,
        #: 目标平台名（想定里的 ``platform`` 名）。名字写错**装配期报错**
        #: （照抄 ``GuidanceComputer`` 的那条：写错名字与"没找到目标"在
        #: 结果里长得一模一样）。留空 = 运行期 :meth:`assign_target_point`。
        "target_name": Params.string(""),
        #: 命中判据（米）：落点到目标的距离 ≤ 它才判命中。解算保证落点
        #: 就是目标，这个参数只在"目标被移动过"的场合起作用。
        "lethal_radius": Params.distance(30.0, minimum=0.0),
        #: **弹道高点**（米，海拔）。本件唯一的形状参数：射程由目标位置定，
        #: 高点定的是"这条弹道有多高、飞多久"。1200 km 是 10000 km 级
        #: 洲际弹道的典型值；打 1000 km 级的中程弹配 250 km 上下。
        "apogee_altitude": Params.distance(1_200_000.0, minimum=1.0),
    }

    __slots__ = (
        "_view",
        "_target_id",
        "_point",
        "_lethal",
        "_solved",
        "_sol",
        "_done",
        "_reason",
        "_detonation",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        super().initialize(mount)
        self._view = mount.engagement_view()
        self._lethal = float(self.spec["lethal_radius"])
        self._point: tuple[float, float, float] | None = None
        self._solved = False
        self._sol: _Solution | None = None
        self._done = False
        self._reason = ""
        self._detonation = float("inf")

        self._target_id: int | None = None
        name = str(self.spec["target_name"])
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
        """直接指定打击点（世界坐标）。**解算前**给才有效：解算发生在
        第一次推进，之后锁定——洲际弹打的是装订坐标（见模块头）。"""
        if self._solved:
            raise ConfigurationError(
                f"{self.describe()}: 弹道已经解算、目标已锁定——"
                "assign_target_point 要在第一次推进之前给"
            )
        self._point = (float(x), float(y), float(z))

    def terminated(self) -> bool:
        return self._done

    def terminate_reason(self) -> str:
        return self._reason

    def hit(self) -> bool:
        return self._reason == REASON_HIT

    def detonation_range_m(self) -> float:
        """落点到（解算时锁定的）目标的距离。没终止是 ``inf``。"""
        return self._detonation

    def flight_time_s(self) -> float:
        """解算出的总飞行时间（秒）。没解算是 0。诊断与预警计算用。"""
        return self._sol.t_end if self._sol is not None else 0.0

    # -- 推进 --------------------------------------------------------------

    def arrived(self) -> bool:
        """弹不"到达"，它**终止**（与 ``MissileMover`` 同一条覆盖理由：
        基类"没有目的地就算到了"会让本件一帧都不动——解算前它确实没有
        目的地，弹道在第一次推进时才解算）。"""
        return self._done

    def vertical_speed_mps(self) -> float:
        """最近一帧的垂向速率（m/s，正 = 上升）。径向速度就是它。"""
        if self._sol is None:
            return 0.0
        return self._evaluate(self._frame_t())[4]

    def _advance(
        self, dt_s: float, pose: tuple[float, float, float, float, float]
    ) -> None:
        """按开普勒时间律把弹放到 ``t`` 时刻该在的位置。

        ``t`` 是**弹道时间**（发射时刻起算）＝ ``elapsed_s()``：引擎"第一帧
        只记钟"的约定把 ``t=0`` 锚在第一次 tick 上，第一次 :meth:`_advance`
        发生在 ``t = period``，之后每帧 +period。置位不插值、不积分——
        解析式在任意 ``t`` 都有精确值。
        """
        if self._done:
            return
        if not self._solved:
            self._sol = self._solve(pose)
            self._solved = True
        sol = self._sol
        assert sol is not None

        t = self._frame_t()
        if t < sol.t_end:
            x, y, z, speed_h, _vz = self._evaluate(t)
            self._commit(x, y, z, sol.azimuth_deg, speed_h)
            return

        # 终点：平面位置精确落在解算目标上，z 落回发射高程。**只提交一次**
        # （终点与"t 那一刻"不是两个位置——里程不许把最后这一段记两遍）。
        x, y, z, speed_h, _vz = self._evaluate(sol.t_end)
        self._commit(x, y, z, sol.azimuth_deg, speed_h)
        target = self._target_position()
        self._detonation = (
            float("inf")
            if target is None
            else hypot(hypot(target[0] - x, target[1] - y), target[2] - z)
        )
        self._done = True
        self._reason = (
            REASON_HIT if self._detonation <= self._lethal else REASON_LAND
        )

    def _frame_t(self) -> float:
        """当前帧对应的弹道时刻（秒）。终止后 ``elapsed_s`` 冻结在终点。"""
        return self.elapsed_s()

    def _target_position(self) -> tuple[float, float, float] | None:
        """解算时锁定的那个目标点（诊断口径用的也是它）。"""
        if self._point is not None:
            return self._point
        if self._target_id is None or self._view is None:
            return None
        return self._view.position_of(self._target_id)

    # -- 解算 --------------------------------------------------------------

    def _solve(self, pose: tuple[float, float, float, float, float]) -> _Solution:
        """发射点 + 目标点 + 弹道高点 → 整条弹道。**只发生一次**。

        ``pose`` 是第一次推进时的存储位置——引擎"第一帧只记钟不挪窝"的
        约定保证了它就是发射点。
        """
        x0, y0 = float(pose[0]), float(pose[1])
        z0 = max(0.0, float(pose[2]))
        who = self.type_name or type(self).__name__

        target = self._target_position()
        if target is None:
            raise ConfigurationError(
                f"{who}: 解析弹道没有目标——它不是积分器，没有目标就没有"
                "弹道。写 target_name，或在第一次推进前调 assign_target_point()"
            )
        xt, yt, _ = target

        flat = hypot(xt - x0, yt - y0)
        if flat <= MIN_RANGE_M:
            raise ConfigurationError(
                f"{who}: 目标与发射点重合（{flat:g} m）——解析弹道解不出"
                "一条没有射程的弹道"
            )
        phi = flat / R_EARTH_M
        if degrees(phi) > MAX_RANGE_DEG:
            raise ConfigurationError(
                f"{who}: 射程 {flat / 1000.0:.0f} km 折合地心角 "
                f"{degrees(phi):.1f}°，超过解算上限 {MAX_RANGE_DEG:g}°"
                "——椭圆在 180° 上退化"
            )
        apogee = float(self.spec["apogee_altitude"])
        r1 = R_EARTH_M + z0
        ra = R_EARTH_M + apogee
        if ra <= r1:
            raise ConfigurationError(
                f"{who}: 弹道高点 {apogee:g} m 不高于发射点 {z0:g} m——"
                "apogee_altitude 要比发射点高"
            )

        # 闭式根数（推导见模块头）。Φ<180° 时分子分母恒同号且 |分母|>|分子|
        # ⇒ 0 < e < 1 恒成立，这里没有参数死角；断言只是防将来改动。
        e = (r1 - ra) / (r1 * cos(phi / 2.0) - ra)
        p = ra * (1.0 - e)
        a = p / (1.0 - e * e)
        assert 0.0 < e < 1.0 and a > 0.0, (e, a)

        nu1 = pi - phi / 2.0
        E1 = _nu_to_E(nu1, e)
        M1 = E1 - e * sin(E1)
        n = sqrt(EARTH_MU / a**3)
        # 对称弹道：落点的平近点角 M₂ = 2π − M₁ ⇒ 全程 ΔM = 2π − 2M₁。
        t_end = (2.0 * pi - 2.0 * M1) / n
        return _Solution(
            x0=x0,
            y0=y0,
            z0=z0,
            azimuth_deg=degrees(atan2(xt - x0, -(yt - y0))) % 360.0,
            a=a,
            e=e,
            mean_motion=n,
            ang_mom=sqrt(EARTH_MU * p),
            nu1=nu1,
            M1=M1,
            phi=phi,
            t_end=t_end,
            sigma_end=R_EARTH_M * phi,
        )

    # -- 弹道求值 ----------------------------------------------------------

    def _evaluate(self, t: float) -> tuple[float, float, float, float, float]:
        """弹道时间 ``t`` → ``(x, y, z, 水平速率, 垂向速率)``。

        一次求值只解**一次**开普勒方程（``E``），其余全是闭式。``t`` 截在
        终点：椭圆轨道不截的话 ``t`` 越过 ``t_end`` 后弹会沿椭圆**转第二圈**
        ——那不是弹道，是卫星。
        """
        sol = self._sol
        assert sol is not None
        t = min(max(0.0, t), sol.t_end)

        E = self._E_at(t)
        nu = 2.0 * atan2(
            sqrt(1.0 + sol.e) * sin(E / 2.0),
            sqrt(1.0 - sol.e) * cos(E / 2.0),
        )
        r = sol.a * (1.0 - sol.e * cos(E))
        sigma = R_EARTH_M * (nu - sol.nu1)
        az = radians(sol.azimuth_deg)

        v_r = (EARTH_MU / sol.ang_mom) * sol.e * sin(nu)
        v_t = sol.ang_mom / r
        sigma_rate = R_EARTH_M * v_t / r
        return (
            sol.x0 + sigma * sin(az),
            sol.y0 - sigma * cos(az),
            r - R_EARTH_M,
            sigma_rate,
            v_r,
        )

    def _E_at(self, t: float) -> float:
        """``t`` 时刻的偏近点角：M = M₁ + n·t，牛顿解开普勒方程。

        本弹道的 M 全程落在 ``[M₁, 2π−M₁]``（对称于远地点，**远离近地点
        那个病态区**），高偏心率下牛顿照样收敛；迭代不收敛报错而不是
        带病返回。
        """
        sol = self._sol
        assert sol is not None
        M = sol.M1 + sol.mean_motion * t
        E = M
        for _ in range(KEPLER_MAX_ITER):
            f = E - sol.e * sin(E) - M
            fp = 1.0 - sol.e * cos(E)
            step = f / fp
            E -= step
            if abs(step) <= KEPLER_TOL:
                return E
        raise RuntimeError(
            f"{self.describe()}: 开普勒方程 {KEPLER_MAX_ITER} 次迭代不收敛"
            f"（M={M:.6f}, e={sol.e:.6f}）——弹道时间律坏了，宁可停演"
        )


def _nu_to_E(nu: float, e: float) -> float:
    """真近点角 → 偏近点角（装配期用一次）。"""
    return 2.0 * atan2(
        sqrt(1.0 - e) * sin(nu / 2.0),
        sqrt(1.0 + e) * cos(nu / 2.0),
    )


__all__ = [
    "EARTH_MU",
    "R_EARTH_M",
    "AnalyticBallisticMover",
]
