"""弹的机动件：三自由度质点，按制导件的指令积分（§5.8）。

它与四个"平台"机动件的区别只有一处，但那一处是本质的
----------------------------------------------------
``GroundMover`` / ``AirMover`` 那些件跟踪**航路点**：目的地是一串点，件负责
把"想去哪"变成"每帧挪多少"。弹不一样——它的轨迹是**制导件按阶段给的指令**
积出来的，中途没有"路点"这回事。所以本件整段覆盖 :meth:`Mover._advance`，
基类的寻路、通行门槛、几何限速一概不用（``PROFILE = None``：天空没有"难走"
这回事，本来也查不出代价）。

覆盖 ``_advance`` 而不是照基类的四个钩子来，是这条差异的必然结果：基类的
推进是"沿航向走、走到路点"的循环，而这里是"按加速度积分"的循环。两者共用
的只有 :meth:`Mover._commit`（写回位置、累计里程、同步空间索引）与
:meth:`Mover.elapsed_s`。**这不是把基类掏空**——基类里那些钩子（``_plan`` /
``_waypoint_z`` / ``_surface_z`` / ``_approach_z``）没有一个是弹会用到的。

状态只有三个：速率、弹道倾角、航向
----------------------------------
三自由度质点运动学在**速度矢量**上做，而速度矢量在存储里放不下三份
（``set_pose`` 只有"水平速率 + 航向"）。所以：

| 分量 | 存在哪 |
|---|---|
| 水平速率 | 实体存储（``speed_mps``，与四个平台件同一条约定） |
| 航向 | 实体存储（``heading_deg``，折进 ``[0, 360)``） |
| **垂向速率** | 本件自己（``_vz``）——存储里没有这一格 |

于是"总速率 = ``hypot(水平, 垂向)``、弹道倾角 = ``atan2(垂向, 水平)``"都是
**派生量**，不另存一份。想从中读出"它现在多快"的调用方要用
:meth:`total_speed_mps`；存储里那个数是**水平**速率，写总速率进去的话，
``KinematicsTable.advance`` 那一套（``dx = d·sinθ``）会把它当成水平位移用。

**唯一的例外是滞空为零时那个倾角**：``atan2(0, 0)`` 没有定义，而两个部件都要
用它去算"抵消重力的那一份"，所以那一处由 :meth:`_flight_path_angle_deg` 一处
给出（落在发射姿态上），由弹体与制导件**共用**——两个"倾角"就是两个重力，
那是弹朝反方向飞的来处（见 :class:`~milsim.models.guidance.contract.FlightState`
的 ``flight_path_angle``）。

★ 速度按**矢量**积分，不按"速率 + 角速度"积分
-----------------------------------------------
教科书上的 3-DOF 方程组常写成"速率 + 角速度"：

.. code-block:: text

    dV/dt  = (T − D)/m − g·sinγ          沿速度方向
    dγ/dt  = (a_v − g·cosγ)/V            弹道倾角
    dψ/dt  =  a_h / (V·cosγ)             航向
    dm/dt  = −T/(Isp·g)                  烧燃料

前两个式子里的 ``a_v`` 是制导件给的**总**法向加速度：``a_v = 0`` 就是
**自由飞行**（只剩重力，弹道弹的中段），``a_v = g·cosγ`` 是平衡（定高平飞），
``a_v`` 更大是往上拉。三种情形同一个方程，没有"模式开关"。

**本件不按这组式子积分，而是把同一个方程写成矢量形式：**

.. code-block:: text

    dv⃗/dt = a_along·v̂ + (a_v − g·cosγ)·n̂ + a_h·m̂

其中 ``v̂`` 是速度单位矢量，``n̂ = ∂v̂/∂γ``（速度系"上"），
``m̂ = ∂v̂/∂ψ / cosγ``（速度系"右"），三者两两正交。两者**在 V > 0 时
一阶等价**（展开后逐分量就是上面那三行），差别全在 ``V → 0`` 那一头：

- 角速度形式要**除以 V**，分母趋零时一个子步能把弹转过 180°。实测过
  （改用矢量形式之前）：巡航弹（推力 1.6 kN / 质量 1200 kg = 0.14 g，从静止
  起步）在 2 s 内航迹倾角在 −167° 与 +101° 之间乱跳，弹永远爬不到 130 m——
  **症状是"参数没生效"，根因是数值发散**。
- 矢量形式没有这个除法（一个子步最多把速度矢量转过 90°），而且物理上更对：
  **速度为零时速度矢量朝着合力的方向长出来**，那正是发射瞬间该发生的事。

**换了积分形式并没有让那枚弹飞起来**——它只是把"数值发散"换成了"物理上爬不
上去"：推重比 0.14 意味着推力只有重量的七分之一，而本件不建升力，所以没有
任何东西能把速度矢量转上去。这种配置现在**直接报错**（见
:meth:`MissileMover._require_static_launch`）。这一条是实测出来的：**换算法
修不好一个参数就错的配置**。

所以"第一帧朝哪飞"只需要一个方向——**发射姿态**（:attr:`launch_fpa`），
不需要一个"最低可操纵速率"那样的假常数。

发射状态（``launch_speed`` / ``launch_fpa``）
---------------------------------------------
弹从发射位离开的那一刻有两个数：多快、朝上多少。它们是**发射平台**的属性
（AFSIM 里由 ``WSF_LAUNCH_COMPUTER`` 按交战几何算出来），不是"这一阶段想干
什么"——所以它们在本件，不在阶段表里。两处都指向同一个量的话，不一致的表现
是"弹朝一个奇怪的方向飞"。

**为什么非得有初速**：从静止起步的弹只能靠自身推力加速，而**巡航弹的推力
比它的重量小**（1.6 kN 的涡扇发不动一枚 1200 kg 的弹，推重比只有 0.14）。
真实世界里那段速度是**载机或助推器**给的：

.. code-block:: text

    launch_speed  240 m/s      载机投放 / 助推段烧完那一刻交给弹的速率
    launch_fpa      0 deg      同那一刻的弹道倾角

**``launch_speed`` 不是"第二条真相"**：只在弹还没有速度时生效一次，之后速率
由积分给出。发射平台本来就在动的话（载机投放），那份速度已经写在实体的速度
字段里，这个参数保持 0 即可。二者都有时以**实体速度**为准——它更"真"。

**从静止起步要求起飞推重比 ≥ 1**：两者都没有（没有 ``launch_speed``、实体
速度也是 0）时，这枚弹只能靠自己的推力离地，而本件没有升力模型，推重比小于
1 的弹建立不起速度系——那种配置在 :meth:`_require_static_launch` 里直接报错。

发射角 ≠ 弹道倾角：无控弹一离轨就开始**重力转向**
---------------------------------------------------
无控弹离开发射位之后只剩重力与阻力，于是它做一个标准的**重力转向**：转弯率
是 ``γ̇ = −g·cosγ / V``（这是本件在"法向加速度 = −g·cosγ"下的恒等式），
V 小的时候这个率很大——"抬头 45° 打出去"的弹几秒钟就压成十几度。实测
（``substep 20 ms``，无控，燃料 400 kg）：

=======  =============  ==========  ==========
发射倾角  2 s 时的倾角    水平射程     最高点
=======  =============  ==========  ==========
45°      13.3°           15.2 km      0.5 km
60°      35.0°           31.9 km      7.3 km
89°      80.1°            6.3 km     42.9 km
=======  =============  ==========  ==========

两个后果：

1. **射程不随发射角单调变化**——45° 反而不如 60°，因为前者在低速段被重力
   转向按得更低。"最优发射角 45°"是**真空**里的结论（``R = v²·sin2θ/g``），
   有阻力与重力转向就不成立。
2. 所以**有制导的弹不靠发射角成形**，它按阶段表把倾角**程序化**地转过去
   （:mod:`milsim.models.guidance.ballistic` 的 ``PITCH_OVER`` 段：发射姿态
   85°，转到 50° 为止）。这张表为什么长成那样，答案就在这里。

三个物理量：推力、质量、重力
----------------------------
AFSIM 的 ``WSF_GUIDED_MOVER`` 含分级、推力、比冲、燃烧率、气动。这里砍到
足以"拟合出弹道效果"的最小集，砍掉的都写在下面的"已知限制"里。

阻力用**指数大气**（``ρ = ρ0·e^{−z/H}``）配一个 ``drag_area``。它不是装饰：
没有它，弹道弹的射程会明显偏长（纯开普勒弹道），而"拟合出一型弹的射程"
正是本件存在的理由。密度标高与海平面密度是**模型常数**（写在模块头上），
不是每一型弹的参数——它们是大气属性，不是弹的属性。

已知限制（有意留的，不是漏的）
------------------------------
- **推力沿速度方向**，不建姿态：真实弹在主动段是"推力沿弹轴、弹轴按程序
  转"，这里把两者叠成一句。要分开就得有攻角与姿态回路，那是 6-DOF。
- **气动只算阻力**，不算升力与动压：过载上限取 ``radial_accel`` 这个常数，
  不随高度与速度变。真实的机动能力 ∝ 动压 ``q``，薄大气里拉不出过载——
  那需要气动表（AFSIM 的 ``aero`` 块），属后续里程碑。
- **不分级**：``mass`` / ``fuel_mass`` / ``thrust`` / ``Isp`` 各一个，一级
  烧完就结束。三级固体弹要么拆成三个平台，要么等分级做进来。
- **不转弯消耗动力**：诱导阻力不在 ``D`` 里，所以持续盘旋不掉速。
- **V ≈ 0 时速度系没有定义**：法向与横向指令的作用方向来自速度矢量，而零
  矢量的方向任意，转弯率 ``a_n / V`` 也没有上界。弹体靠**发射姿态**
  （``launch_fpa``）给出初始指向，靠**起飞推重比 ≥ 1** 挡住这个角落——它
  不是被修掉了，是被挡住了。真要有推力矢量的姿态回路，那是 6-DOF 的事。
- **目标高度取的是实体自己的 z**，而想定里用 ``hex`` 落格的平台 z **恒为 0**
  （`Simulation._locate` 的约定，与地形无关）。实测这片战区的陆地格高程是
  128~515 m，所以摆在陆地上的静止靶标实际"埋"在地面以下，弹扑过去报的是
  **"落地"而不是"命中"**——而模型这边一个数都没错。打**水面**目标没有这个
  问题（水域 z = 0 就是海面，也正是水面舰的位置约定）。要打地面目标，得先
  让落格把地形高程算进来（§9-22）。
"""

from __future__ import annotations

from math import atan2, cos, degrees, exp, hypot, radians, sin
from typing import Any

from ...errors import ConfigurationError
from ...services.params import Params, STANDARD_GRAVITY
from ...services.type_registry import register_component
from ..guidance.contract import (
    SPEED_EPS,
    FlightState,
    GuidanceCommand,
)
from .base import Mover

#: 海平面大气密度（``kg/m³``）。
RHO0 = 1.225
#: 指数大气的密度标高（米）。8500 m 是标准大气的常用值：用它算出来的
#: 10 km 处密度是地面的 30.9%，与标准大气表（33.7%）差 8%——这一层不需要
#: 更准，因为它影响的是阻力而不是精度。
SCALE_HEIGHT_M = 8500.0

#: 默认节拍（微秒）。取 2 s 是为了**与全局最大步长对齐**（§2.4）：弹的每一
#: 帧内部还要分子步，节拍本身没有理由比别的模型细。
MISSILE_PERIOD_US = 2_000_000
#: 默认子步长（微秒）。末端比例导引要 0.02 s 量级的闭环，而引擎的步长不能
#: 被它拉细（§2.4：需要高精度的模型**在自身 update 内做子步积分**）。
SUBSTEP_US = 20_000
#: 一帧最多分多少子步。超了就报错而不是悄悄放粗：悄悄放粗会让"末端精度"
#: 随想定里的周期变化，而那个变化没有任何地方会写出来。
MAX_SUBSTEPS = 2_000
#: 离地多高才算"真的飞起来了"（米）。落地判据要等它先成立——地面发射的弹
#: 在点火那一刻 z 正好等于地面高程，不设这个门槛它会在第一帧就判落地。
AIRBORNE_CLEARANCE_M = 1.0


@register_component("MISSILE_MOVER")
class MissileMover(Mover):
    """三自由度质点弹体：只受推力、阻力、重力，方向由制导指令给。

    它**不含制导**——没有制导件时它是无控弹（只有重力、阻力、推力），
    那不是错误配置，是火箭弹。有制导件时它每个子步向它要一次指令
    （§5.8 的分工：弹体管"能不能做到"，制导件管"要往哪飞"）。
    """

    __slots__ = (
        "_vz",
        "_mass",
        "_dry_mass",
        "_fuel",
        "_ever_thrust",
        "_thrusting",
        "_airborne",
        "_guidance",
        "_substeps",
        "_launched",
        "_done",
        "_reason",
        "_flight_flat_m",
        "_max_alt_m",
        "_max_speed_mps",
        "_peak_accel",
    )

    #: 不吃地形：弹飞过山而不是绕过山。取 ``None`` 同时意味着"没有地图也能
    #: 飞"，而地面/水面/水下件需要地图才能工作。
    PROFILE = None

    PARAMS = {
        **Mover.PARAMS,
        #: 起飞总质量（含燃料）。
        "mass": Params.mass(1_000.0, minimum=1e-6),
        #: 燃料质量。**烧完之后质量就是 ``mass − fuel_mass``**——火箭的加速度
        #: 随燃料减少而上升这件事就是从这里出来的，不需要另写一条曲线。
        "fuel_mass": Params.mass(400.0, minimum=0.0),
        #: 真空推力。
        "thrust": Params.force(0.0, minimum=0.0),
        #: 比冲（秒）。基本单位是秒——它本来就是时间量纲。
        #: **不要拿 number 凑**：写 ``250`` 与写 ``250 ms`` 会得到差 1000 倍的
        #: 燃料消耗率，而单看数字看不出来。
        "specific_impulse": Params.duration(250_000_000, minimum=1),
        #: **零升阻力面积** ``C_d·A``（m²）。0 = 不算阻力。
        #: 它是阻力公式里 ``½ρV²·(C_d·A)`` 的那个乘积，不是弹的任何一块
        #: 真实面积——把两者混起来的后果是阻力差一个系数，而射程差一半。
        "drag_area": Params.area(0.0, minimum=0.0),
        #: 弹体能拉出的**机动**加速度上限（m/s²，0 = 不限制）。与四个平台件
        #: 的 ``radial_accel`` 同一个口径：制导件先按本阶段 ``max_accel`` 夹
        #: 一道，这里再按弹体夹一道，取更严的。
        "radial_accel": Params.accel(0.0, minimum=0.0),
        #: 最大速率（m/s）。它是**封顶**，不是巡航速率：真实弹的极速由推力
        #: 与阻力算出来，这个数只防"参数写错导致跑到第一宇宙速度"。
        "max_speed": Params.speed(3_000.0, minimum=0.0),
        #: 节拍（微秒）。见 :data:`MISSILE_PERIOD_US`。
        "period": Params.duration(MISSILE_PERIOD_US, minimum=100),
        #: 子步长（微秒）。见 :data:`SUBSTEP_US`。
        "substep": Params.duration(SUBSTEP_US, minimum=100),
        #: 制导件在哪个槽。留空 = **这型弹是无控弹**（火箭弹），只有重力、
        #: 阻力与推力。槽名写错会**在装配期报错**，不会退化成无控弹——
        #: "弹笔直往前飞"与"导引头没工作"在结果里长得一模一样。
        "guidance_slot": Params.string("guidance"),
        #: **发射初速**（m/s）：载机投放或助推段烧完那一刻交给弹的速率。
        #: 从静止起步的巡航弹（推力 < 重力）靠自己是爬不起来的——真实世界
        #: 里那段速度是载机或助推器给的。**只在弹还没有速度时生效一次**，
        #: 之后速率由积分给出；发射平台本来就在动的话（载机投放）那份速度
        #: 已经在实体的速度字段里，这个参数保持 0 即可。
        "launch_speed": Params.speed(0.0, minimum=0.0),
        #: **发射姿态**：离开发射位那一刻的弹道倾角（度）。垂直发射写 85~89。
        #: 它是弹在**还没有速度**时唯一的指向来源——零矢量的方向任意，而这
        #: 个参数给出那个方向。速度起来之后由积分决定，本参数不再起作用。
        #: 阶段的 ``flight_path_angle`` 管的是"离开发射位之后转到多少度"，
        #: 与本参数不是同一个东西。
        "launch_fpa": Params.angle(0.0),
        #: 油门增益（1/秒）：有速率指令时，按"还差多少速率"调推力。
        #: 它决定巡航能不能稳在指令速率上；推力不够时它自然饱和到满推力，
        #: 于是"射程不够"表现得像物理而不是像参数没生效。
        "throttle_gain": Params.number(0.5, minimum=0.0),
    }

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        super().initialize(mount)

        self._mass = float(self.spec["mass"])
        self._fuel = float(self.spec["fuel_mass"])
        if self._fuel > self._mass:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 燃料 {self._fuel:g} kg "
                f"比总质量 {self._mass:g} kg 还多——净重会变成负数"
            )
        #: 净重（烧完燃料之后的质量）。定死一次，之后质量永远是"净重 + 剩油"。
        self._dry_mass = self._mass - self._fuel
        isp_s = float(self.spec["specific_impulse"]) / 1_000_000.0
        if isp_s <= 0.0:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 比冲必须是正的"
            )

        period_us = float(self.spec["period"])
        substep_us = float(self.spec["substep"])
        count = max(1, int(round(period_us / substep_us)))
        if count > MAX_SUBSTEPS:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 一帧要分 {count} 个子步"
                f"（节拍 {period_us / 1e6:g} s ÷ 子步 {substep_us / 1e6:g} s），"
                f"超过上限 {MAX_SUBSTEPS}——把 substep 调大或把 period 调小"
            )
        self._substeps = count

        self._vz = 0.0
        self._launched = False
        self._ever_thrust = False
        self._thrusting = False
        self._airborne = False
        self._done = False
        self._reason = ""
        self._flight_flat_m = 0.0
        self._max_alt_m = 0.0
        self._max_speed_mps = 0.0
        self._peak_accel = 0.0

        self._guidance: Any = None
        slot = str(self.spec["guidance_slot"])
        if slot and mount.entity is not None:
            found = mount.entity.component(slot)
            if found is None:
                available = "、".join(mount.entity.slots()) or "（一个都没有）"
                raise ConfigurationError(
                    f"{self.type_name or type(self).__name__}: 制导槽 {slot!r} 里"
                    f"没有部件。已有的槽：{available}。"
                    f"无控弹要显式写 guidance_slot \"\""
                )
            self._guidance = found

    # -- 目的地：弹不接受 ------------------------------------------------

    def move_to_cell(self, cell: Any) -> bool:
        raise ConfigurationError(
            f"{self.describe()}: 弹不接受按格下达目的地——它的轨迹由制导件"
            "按阶段决定（§5.8）。要打哪个目标写制导件的 target_name"
        )

    def move_to_point(self, x: float, y: float, z: float) -> bool:
        raise ConfigurationError(
            f"{self.describe()}: 弹不接受按点下达目的地——它的轨迹由制导件"
            "按阶段决定（§5.8）。要把弹打向某个坐标，调制导件的"
            " assign_target_point()"
        )

    # -- 查询 --------------------------------------------------------------

    def arrived(self) -> bool:
        """弹不"到达"，它**终止**：命中、落地、或制导件叫停。

        覆盖基类是因为基类那条"没有目的地就算到了"对弹不成立——弹从上线
        那一刻就没有航路点，照基类判它一开始就是"到了"，于是**一帧都不动**。
        """
        return self._done

    def vertical_speed_mps(self) -> float:
        """垂向速率（m/s，正 = 上升）。存储里放不下的那一格。"""
        return self._vz

    def total_speed_mps(self) -> float:
        """总速率（m/s）。**不是**存储里那个数——那个是水平分量。"""
        return hypot(self.speed_mps(), self._vz)

    def flight_path_angle_deg(self) -> float:
        """弹道倾角（度，正 = 爬升）。

        与 :meth:`_flight_path_angle_deg` 是**同一个数**，不是在这里另算一遍：
        速度为零时那个 ``atan2(0, 0)`` 没有定义，弹体与制导件都用发射姿态
        （见那里的说明），查询接口也必须给同一个值——两个"倾角"的话，诊断里
        报出来的数与模型里用的数会不一样。
        """
        psi = radians(self.heading_deg())
        speed = self.speed_mps()
        return self._flight_path_angle_deg(
            speed * sin(psi), -speed * cos(psi), self._vz
        )

    def mass_kg(self) -> float:
        return self._mass

    def fuel_kg(self) -> float:
        return self._fuel

    def thrusting(self) -> bool:
        """这一帧发动机在工作吗（燃料没烧完、制导件没关机）。"""
        return self._thrusting

    def burnout(self) -> bool:
        """点过火、而现在没有推力了。推进条件里的 ``burnout`` 就是它。"""
        return self._ever_thrust and not self._thrusting

    def terminate_reason(self) -> str:
        return self._reason

    def max_altitude_m(self) -> float:
        return self._max_alt_m

    def max_speed_mps(self) -> float:
        return self._max_speed_mps

    def peak_accel(self) -> float:
        """全程见过的最大的**机动**加速度（m/s²）。诊断用：看它有没有打到
        过载上限上——打到了说明这一段是"拉不动"而不是"没拉"。"""
        return self._peak_accel

    def guidance(self) -> Any:
        return self._guidance

    def describe(self) -> str:
        return f"{self.type_name or type(self).__name__}({self.slot})"

    # -- 推进 --------------------------------------------------------------

    def _advance(
        self, dt_s: float, pose: tuple[float, float, float, float, float]
    ) -> None:
        """一帧 = ``_substeps`` 个子步。每个子步向制导件要一次指令。

        **本帧积分的区间是 ``(t₀, t₀+dt]``，``t₀ = 已走过的时长 − dt``**：
        位置是区间**起点**那一刻的位置（引擎的离散约定，§5.7 的"第一帧不挪窝"
        同一条）。子步的时刻因此是 ``t₀ + (k+1)·h``，起点恰好落在 0。

        一个子步里的次序是固定的：**判终止（制导件先、落地后）→ 积分**。
        终止为什么要有次序、而且两条判据要看**同一个位置**，见下面那段注释。
        """
        if self._done or dt_s <= 0.0:
            return

        x = float(pose[0])
        y = float(pose[1])
        z = float(pose[2])
        heading = float(pose[3])
        speed = float(pose[4])
        ref = self.view.my_cell()
        h = dt_s / self._substeps
        t0 = max(0.0, self.elapsed_s() - dt_s)
        start = (x, y, z)

        psi = radians(heading)
        vx, vy, vz = speed * sin(psi), -speed * cos(psi), self._vz

        # 发射：初速与姿态只在弹还**从静止出发**时用得着一次。载机投放的情形
        # 那份速度已经在实体的速度字段里了（`speed`）。两者都没有的话，这枚弹
        # 得靠自己的推力离开地面——那要求起飞推重比 ≥ 1。
        if not self._launched:
            self._launched = True
            if hypot(hypot(vx, vy), vz) <= SPEED_EPS:
                seed = float(self.spec["launch_speed"])
                if seed > 0.0:
                    fpa = radians(float(self.spec["launch_fpa"]))
                    vx = seed * sin(psi) * cos(fpa)
                    vy = -seed * cos(psi) * cos(fpa)
                    vz = seed * sin(fpa)
                else:
                    self._require_static_launch()

        ground = self._ground_z(ref, x, y)
        # 循环多跑一遍：最后一遍只是**补判触地点**。落在帧最后一子步的位置也
        # 要过一遍判据，否则弹会带着 z = 地面 的位置再飞一整帧。
        for step in range(self._substeps + 1):
            if self._done:
                break
            horizontal = hypot(vx, vy)
            # 航向每**子步**重算，不是每帧一算：转向是连续的，拿帧初的航向
            # 去算这一帧里 100 个子步的偏差，等于把制导环的采样率降到 0.5 Hz。
            if horizontal > SPEED_EPS:
                heading = degrees(atan2(vx, -vy)) % 360.0
            # 弹道倾角也只算一次，**两个部件共用它**（见 _flight_path_angle_deg）。
            fpa = self._flight_path_angle_deg(vx, vy, vz)
            command = self._command(
                FlightState(
                    flight_time=t0 + min(step + 1, self._substeps) * h,
                    x=x, y=y, z=z, ground_z=ground,
                    heading=heading, speed=horizontal, vertical_speed=vz,
                    flight_path_angle=fpa,
                    mass=self._mass,
                    thrusting=self._thrusting,
                    burnout=self.burnout(),
                    downrange=self._flight_flat_m,
                )
            )
            # **制导件的终止先判，而且与落地判据看同一个位置**。反过来的话，
            # 末段"最后一小步同时穿过引信点火区与地面"会被报成"落地"——
            # 实测过：100 km 上的巡航弹在目标 0.4 m 处触地，报的是"落地"，
            # 于是"打中了"这件事在诊断里看不出来。
            if command.terminate:
                self._done = True
                self._reason = command.reason or "制导终止"
                break
            # 落地判据要**先离地才成立**：地面发射的弹在点火那一刻 z 正好
            # 等于地面高程，少了这一条它会在第一帧就"落地"并停在发射位上。
            if z > ground + AIRBORNE_CLEARANCE_M:
                self._airborne = True
            elif self._airborne and z <= ground:
                z = ground
                self._done = True
                self._reason = "落地"
                break
            if step == self._substeps:
                break                       # 最后一遍只是补判，不再积分
            vx, vy, vz, x, y, z = self._step(
                h, command, vx, vy, vz, x, y, z, heading, fpa, ground
            )
            # 弹不进地下（撞地/撞山的那一小步）。**压在这里而不是压进判据里**：
            # 判据要靠"z ≤ 地面"来分辨触地，压在判据之后它就永远看不到。
            # 顺手把它留成下一子步判据要用的那个地面高程——同一个数，不查两遍。
            ground = self._ground_z(ref, x, y)
            z = max(z, ground)
            self._max_alt_m = max(self._max_alt_m, z)
            self._max_speed_mps = max(
                self._max_speed_mps, hypot(hypot(vx, vy), vz)
            )

        self._vz = vz
        moved = hypot(x - start[0], y - start[1])
        self._flight_flat_m += moved
        horizontal = hypot(vx, vy)
        if horizontal > SPEED_EPS:
            heading = degrees(atan2(vx, -vy)) % 360.0
        # 写**水平**速率：存储里那一格是水平分量（见模块头）。
        self._commit(x, y, z, heading, horizontal)

    def _command(self, state: FlightState) -> GuidanceCommand:
        if self._guidance is None:
            # 无控弹：烧完为止，方向没人管。垂直方向因此只剩重力。
            return GuidanceCommand(phase="无控", thrusting=True)
        return self._guidance.command(state)

    def _require_static_launch(self) -> None:
        """从静止起步的弹要证明它离得开地面：**起飞推重比 ≥ 1**。

        为什么非要这条：本件没有升力模型（见模块头"已知限制"），只有推力、
        阻力、重力，而且**推力沿速度方向**。所以"爬升"这条指令只能靠转动
        速度矢量来兑现，转动速率是 ``a_n / V``——V ≈ 0 时它没有上界。
        推重比大于 1 的弹零点几秒就把速度建立起来，那一段转瞬即逝；推重比
        小于 1 的弹连速度都建不起来，法向指令只会让它原地乱转。实测（涡扇
        巡航弹，推重比 0.14）：速度在 0.1~0.5 m/s 之间把航向在 0°/180° 来回
        翻，1000 s 烧掉 100 kg 燃料而位置只挪了 4 km——**那不是"爬不上去"，
        那是"方向在乱转"**，而参数表看起来一个都没错。

        真实世界的解是**发射平台给的初速**（载机投放、助推器、发射筒的弹射
        行程），所以修法就是 ``launch_speed``，或者把第一阶段改成不要求法向
        加速度的写法。

        判据用**起飞质量**（含燃料）：从静止起步的那一刻就是它。烧掉燃料
        之后推重比会上升，但那一份燃料是在乱转的速度矢量上烧掉的。
        """
        thrust = float(self.spec["thrust"])
        takeoff = float(self.spec["mass"])
        ratio = thrust / (takeoff * STANDARD_GRAVITY)
        if ratio >= 1.0:
            return
        raise ConfigurationError(
            f"{self.type_name or type(self).__name__}: 这枚弹从静止起步，而起飞"
            f"推重比只有 {ratio:.2f}（推力 {thrust:g} N ÷ 起飞质量 {takeoff:g} kg"
            f" × g）。本件不建升力，推力小于重量的弹建立不起速度系——"
            "V ≈ 0 时法向指令只会把速度矢量原地乱转（实测：推重比 0.14 时"
            "速度在 0.1~0.5 m/s 之间把航向在 0°/180° 来回翻，1000 s 烧掉 "
            "100 kg 燃料而位置只挪了 4 km）。**给 launch_speed**（载机投放 / "
            "助推段交出来的那个速率），或者把第一阶段改成不要求法向加速度的写法"
        )

    def _flight_path_angle_deg(self, vx: float, vy: float, vz: float) -> float:
        """速度矢量此刻的弹道倾角（度，正 = 爬升）。

        速度为零时用**发射姿态**——这一步是必需的，不是"顺手兜底"：
        ``atan2(0, 0)`` 没有定义，而制导件与弹体都要用这个 γ 去算"抵消重力的
        那一份加速度"。两边各推一次就会推出两个数（一个 0、一个发射姿态），
        于是重力被抵掉两次，弹朝**反方向**飞。所以它只在这里算，一次。
        """
        if hypot(hypot(vx, vy), vz) <= SPEED_EPS:
            return float(self.spec["launch_fpa"])
        return degrees(atan2(vz, hypot(vx, vy)))

    def _step(
        self,
        h: float,
        command: GuidanceCommand,
        vx: float,
        vy: float,
        vz: float,
        x: float,
        y: float,
        z: float,
        heading: float,
        gamma_deg: float,
        ground: float,
    ) -> tuple[float, float, float, float, float, float]:
        """一个子步的积分。返回新的 ``(vx, vy, vz, x, y, z)``。

        顺序与基类同一条纪律：**先定这一小步的力，再按力改速度，最后按新
        速度挪位置**。先挪位置再改速度，等于这一小步用的是上一小步的速度。

        ``gamma_deg`` 由调用方算一次（:meth:`_flight_path_angle_deg`），
        **同一个值也发给了制导件**——两边各算一次会不一致（见那里的说明）。

        ``ground`` 本步不用：落地要等**先离地**才成立，那是 :meth:`_advance`
        的事。参数留着是为了将来做地形跟随的预判时不必改签名。
        """
        mass = self._mass
        v = hypot(hypot(vx, vy), vz)
        gamma = radians(gamma_deg)

        # 1) 速度单位矢量，以及由它定出的速度系三轴。
        #    静止时速度矢量的方向没有定义（零矢量的朝向任意），此时用发射
        #    姿态给定——垂直发射的弹就是从这里获得初始指向的。
        if v <= SPEED_EPS:
            psi = radians(heading)
            ux = sin(psi) * cos(gamma)
            uy = -cos(psi) * cos(gamma)
            uz = sin(gamma)
        else:
            ux, uy, uz = vx / v, vy / v, vz / v
            psi = atan2(ux, -uy)

        sin_g, cos_g = sin(gamma), cos(gamma)
        # n̂ = ∂v̂/∂γ（速度系"上"），m̂ = 水平横向（速度系"右"）。
        # 两者都与 v̂ 正交，所以三个方向上的加速度可以直接相加。
        nx, ny = -sin(psi) * sin_g, cos(psi) * sin_g
        mx, my = cos(psi), sin(psi)

        # 2) 发动机：燃料还在烧、而且制导件没关机
        want_thrust = command.thrusting and self._fuel > 0.0
        thrust_max = float(self.spec["thrust"]) if want_thrust else 0.0

        # 3) 阻力。z 取海平面为 0（与高程同一基准），负高度的密度按海平面算。
        rho = RHO0 * exp(-max(0.0, z) / SCALE_HEIGHT_M)
        drag = 0.5 * rho * v * v * float(self.spec["drag_area"])

        # 4) 油门：给了速率指令就按它调推力，要不到就满推力
        isp_s = float(self.spec["specific_impulse"]) / 1_000_000.0
        if command.speed_mps is not None and thrust_max > 0.0:
            gain = float(self.spec["throttle_gain"])
            need = mass * (
                gain * (command.speed_mps - v)
                + STANDARD_GRAVITY * sin_g
            ) + drag
            thrust = min(max(need, 0.0), thrust_max)
        else:
            thrust = thrust_max

        self._thrusting = thrust > 0.0
        if self._thrusting:
            self._ever_thrust = True
            burned = thrust / (isp_s * STANDARD_GRAVITY) * h
            self._fuel = max(0.0, self._fuel - burned)
        # 质量 = 净重 + **剩下的燃料**。不累加"已经烧掉多少"：那会积累浮点
        # 误差，而且与"还剩多少"构成两个真相。火箭加速度随燃料减少而上升，
        # 就是这一行走出来的——不需要另写一条加速度曲线。
        self._mass = self._dry_mass + self._fuel

        # 5) 过载限幅：夹的是**机动那一份**（扣掉抵消重力的部分）。
        #    平飞时那一份不占过载预算，否则只能拉 1 g 的弹连平飞都维持不了。
        gravity_term = STANDARD_GRAVITY * cos_g
        vertical_maneuver = command.vertical_accel - gravity_term
        lateral = command.lateral_accel
        need = hypot(vertical_maneuver, lateral)
        limit = float(self.spec["radial_accel"])
        self._peak_accel = max(
            self._peak_accel, min(need, limit) if limit > 0.0 else need
        )
        if limit > 0.0 and need > limit:
            scale = limit / need
            vertical_maneuver *= scale
            lateral *= scale

        # 6) 三个方向上的加速度合成后加到速度矢量上。
        #    切向那一份含重力沿速度方向的分量（−g·sinγ），法向那一份已经由
        #    `vertical_maneuver` 扣掉了重力的转向效应（−g·cosγ）——所以重力
        #    在两个方向各出现一次，不是重复计算：它本来就分成这两个分量。
        a_along = (thrust - drag) / mass - STANDARD_GRAVITY * sin_g
        vx += (a_along * ux + vertical_maneuver * nx + lateral * mx) * h
        vy += (a_along * uy + vertical_maneuver * ny + lateral * my) * h
        vz += (a_along * uz + vertical_maneuver * cos_g) * h

        # 速率封顶。它是**防跑飞**的闸，不是巡航速率（见 PARAMS 的说明）。
        ceiling = float(self.spec["max_speed"])
        if ceiling > 0.0:
            v_now = hypot(hypot(vx, vy), vz)
            if v_now > ceiling:
                scale = ceiling / v_now
                vx *= scale
                vy *= scale
                vz *= scale

        # 7) 位置：用**本子步结束时**的速度矢量。
        return vx, vy, vz, x + vx * h, y + vy * h, z + vz * h

    def _ground_z(self, ref: Any, x: float, y: float) -> float:
        """(x, y) 处**弹撞得到的那个面**的高程。没有地图或落在战区外时取海平面 0。

        **★ 取 ``max(高程, 0)``，不是高程**：高程通道在水域放的是**海床**
        （实测战区里一片水面的高程是 −423.5 m），而弹撞到**海面**就结束了，
        撞不到海床。不夹这一下有两个后果，两个都是实测出来的：

        - 巡航段的"离地 50 m"跟随的是**海床**——飞过海岸线时指令高度从
          ``地面 + 50`` 掉到 ``−423 + 50 = −373 m``，弹于是在半路**一头扎进
          水里**（而参数表里一个数都没错）。
        - 落地判据用的是海床，于是弹要沉到水下几百米才算"落地"。

        陆上不受影响（高程为正，``max`` 取它自己）。这一条与四个平台机动件的
        口径**故意不同**：地面件是在地上跑的，"地面"就是地形；弹是在空中飞的，
        "地面"是它能撞到的那个面。海床留给鱼雷去撞。

        战区外返回 0 而不是"不知道"：弹在战区外依然要落地，而把"没有数据"
        当成"没有地面"会让它一直往下飞。这一条在弹道弹上看得见——它的射程
        往往超过一个战区的半径。
        """
        if self._nav is None:
            return 0.0
        ground = self._nav.ground_at(ref, x, y)
        return 0.0 if ground is None else max(0.0, float(ground))


__all__ = [
    "MAX_SUBSTEPS",
    "MISSILE_PERIOD_US",
    "RHO0",
    "SCALE_HEIGHT_M",
    "SUBSTEP_US",
    "MissileMover",
]
