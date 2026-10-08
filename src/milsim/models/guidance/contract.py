"""弹的两个部件之间交换的全部词汇：阶段表、飞行状态、一帧指令（§5.8）。

弹是**两个部件**，职责按"想要"与"能做到"切开：

.. code-block:: text

    MissileMover      弹体    能飞多快、能转多急、还剩多少燃料、重力怎么拉
    GuidanceComputer  制导件  下一小步要往哪飞、什么时候换阶段、什么时候算命中

AFSIM 也是这么分的：``WSF_GUIDED_MOVER`` 是 3-DOF 质点、不含制导，
``WSF_GUIDANCE_COMPUTER`` 是处理器、不自己动，前者**每个积分步**问后者要一次
加速度。这里照搬这条：弹体在自己的子步循环里调
:meth:`GuidanceComputer.command` 拿一帧指令。

为什么单独一个文件
------------------
两者在一个积分步里交换一次数据，交换的东西必须有**唯一一份**定义：各写一份
的话，弹体读 ``cmd.speed``、制导件写 ``cmd.speed_mps``，症状是速度指令静默
失效——键名对不上不报错，只表现为"这条指令没生效"。

为什么阶段表是**组件参数**而不是想定语法
--------------------------------------
AFSIM 的弹道形态**不是靠换运动器类实现的**：同一型 ``WSF_GUIDED_MOVER`` 换
一套 phase 剧本就成了另一种弹。剧本在 AFSIM 里写在想定里；这里它是参数
（``Params.phases``），理由是想定语言目前只有"块 / 赋值"两条产生式，要容纳
阶段块得同时动解析器白名单、语义层与组件读取三处，而阶段表的形状是固定的
——没必要为它另开一门嵌套语法。写好的阶段表就是这一型弹的属性：

.. code-block:: text

    component_type MISSILE_MOVER          ← 先声明同名空块把类引进类型表
    end_component_type
    component_type SSM_CRUISE : MISSILE_MOVER     ← 弹体：性能参数
        mass   1300 kg
        thrust 1.6 kN
    end_component_type
    component_type GUID_CRUISE : CRUISE_GUIDANCE  ← 制导：阶段剧本（默认值即剧本）
    end_component_type

三点关于"重力"的话，先说在前面
------------------------------
1. :attr:`GuidanceCommand.vertical_accel` 是**总**垂向加速度，**含抵消重力
   的那一份**。所以它取 0 就是"没人管垂直"——弹只剩重力，那正是弹道弹中段。
   这个约定让"自由飞行"不需要一个额外的开关：剧本里写
   ``vertical=VERT_FREE`` 就够了。
2. 过载限幅夹的是**机动那一份**（``vertical_accel − g·cosγ``），不是总量。
   平飞时抵消重力的那 1 g 不占过载预算，否则一架只能拉 1 g 的飞机连平飞都
   维持不了。
3. ``g`` 只有一处定义（:data:`milsim.services.params.STANDARD_GRAVITY`）。
4. "抵消重力的那一份"要用**同一个 γ** 算两遍：制导件算 ``a_v`` 时用
   :attr:`FlightState.flight_path_angle`，弹体扣的时候也要用它。两边各推一次
   会推出两个数——v = 0 时 ``atan2`` 那一套没有定义，一边给 0、另一边给
   发射姿态，于是"抵消重力"变成"多加了一份重力"，弹会朝**反方向**飞。
   所以那个 γ 由弹体给、由两边共用，不从 ``speed`` 与 ``vertical_speed`` 推。

**发射姿态不在指令里**
----------------------
"弹从发射筒/挂架离开时朝向哪"是**发射平台**的属性，不是这一阶段战术的一部分：
AFSIM 里它由 ``WSF_LAUNCH_COMPUTER`` 按交战几何算出来。milsim 不做发射计算机，
所以它落在弹体的 ``launch_fpa`` / ``launch_speed`` 两个参数上（见
:mod:`milsim.models.mover.missile`）。若把它放进阶段表，同一个数就有两处
出处，而两处不一致的表现是"弹朝一个奇怪的方向飞"——正是上面第 4 条那个
症状。阶段表只管**离开发射位之后**往哪飞。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import inf

from ...services.params import STANDARD_GRAVITY, ParamError

#: 速率低于它就认为"弹还没有速度"（m/s）。此时**速度矢量的方向没有定义**
#: （零矢量的朝向任意），所以垂直发射的初始指向只能来自弹体的**发射姿态**
#: （``launch_fpa``）——那是发射平台的属性、不在指令里，见模块说明的
#: "发射姿态不在指令里"。
SPEED_EPS = 1e-6

# ---------------------------------------------------------------------------
# 垂向律
# ---------------------------------------------------------------------------

#: 不接管垂向：只有重力与阻力，速度矢量自己转。**弹道弹的中段就是它**——
#: "关掉制导让重力接管"在 AFSIM 里写成 ``guidance_delay 5000 sec``（一个
#: 永远不会到的时刻），在这里写成 ``vertical=VERT_FREE``，意思一样但不用
#: 靠一个大到不可能的数来表达。
VERT_FREE = "free"
#: 1 g 平衡：抵消重力的转向分量，弹道自然保持。定高、定倾角都在它上面加修正。
VERT_BALANCE = "balance"
#: 保持弹道倾角（``flight_path_angle``，度，正 = 爬升）。
VERT_FPA = "flight_path_angle"
#: 保持高度（``altitude``，米；``altitude_agl`` 为真时是离地高度）。
VERT_ALTITUDE = "altitude"
#: 垂向比例导引：``N·Vc·λ̇_v + g·cosγ``——末段与水平 PN 成对使用。
#: 重力补偿**显式写出来**（AFSIM 的 ``GRAVITY_BIAS`` 是同一个东西）：
#: 少了它，比例导引会把弹往地上压。
VERT_PN = "pn"

VERTICAL_LAWS: tuple[str, ...] = (
    VERT_FREE,
    VERT_BALANCE,
    VERT_FPA,
    VERT_ALTITUDE,
    VERT_PN,
)

# ---------------------------------------------------------------------------
# 水平律
# ---------------------------------------------------------------------------

#: 保持进入本阶段时的航向。发射段用它：刚离轨时不该急着转向。
HORIZ_HOLD = "hold"
#: 指向目标当前位置。**这就是追踪导引**（AFSIM 的 velocity pursuit）：指令
#: 角速度与方位偏差成正比，转弯需求最小、对付静止目标够用，但对机动目标是
#: 落后追击、要的过载反而更大。
HORIZ_TARGET = "target"
#: 比例导引：``a = N·Vc·λ̇``。对付机动目标的正解，末段用它。
HORIZ_PN = "pn"

HORIZONTAL_LAWS: tuple[str, ...] = (
    HORIZ_HOLD,
    HORIZ_TARGET,
    HORIZ_PN,
)

# ---------------------------------------------------------------------------
# 推进条件
# ---------------------------------------------------------------------------

#: 可以拿来推进阶段的**连续变量**：名字 → 单位（诊断与文档用）。
#:
#: 名字与 AFSIM 的 ``next_phase X when <变量> <比较符> <值>`` 对齐，只是把那个
#: 小语言换成结构化字段——字符串条件要么自己写一个小解析器（多一处会静默
#: 解析错的地方），要么就得允许"看着像条件、实际没生效"的写法。
#:
#: ``target_elevation`` 是最后补上的一个，它对应 AFSIM 的同名量：**视线高低角**
#: （度，正 = 目标在弹上方）。反舰弹的 ``POPUP`` 段就是靠
#: ``next_phase DIVE when target_elevation < -20 deg`` 切进俯冲的——"跃升到
#: 目标掉到视线下方 20°"。它**不是** ``descending``：跃升段里弹正在**爬升**，
#: 而条件已经该成立了。两者差的是"看目标的角度"与"自己的垂向状态"。
UNTIL_VARIABLES: dict[str, str] = {
    "phase_time": "s",          # 进入本阶段之后过了多久
    "flight_time": "s",         # 从开工算起
    "altitude": "m",            # 海拔
    "altitude_agl": "m",        # 离地高度
    "speed": "m/s",             # 水平速率
    "vertical_speed": "m/s",    # 正 = 上升
    "range": "m",               # 到目标的斜距
    "closing_speed": "m/s",
    "downrange": "m",           # 已飞水平里程
    "target_elevation": "deg",  # 视线高低角：正 = 目标在**上方**
}

#: 布尔事件：出现即推进，不带比较符与门限。
#:
#: ``burnout`` 是"点火过、而现在没有推力了"——**不是** "thrusting 为假"：
#: 刚离轨的那一瞬也是假的，拿它当条件会让弹在第一帧就跳过助推段。
UNTIL_FLAGS: tuple[str, ...] = ("burnout", "descending")

_COMPARISONS = {
    "<": lambda a, b: a < b,
    "<=": lambda a, b: a <= b,
    ">": lambda a, b: a > b,
    ">=": lambda a, b: a >= b,
}


@dataclass(frozen=True, slots=True)
class Until:
    """什么时候进入下一阶段。

    ``variable`` 取 :data:`UNTIL_VARIABLES` 里的名字，配 ``op`` 与 ``value``
    （基本单位：秒 / 米 / 米每秒）；布尔事件取 :data:`UNTIL_FLAGS`，不带
    比较符。三个构造器让剧本读起来像句子：

    .. code-block:: python

        Until.after(16.0)                 # 本阶段过了 16 s
        Until.above("altitude", 20000.0)  # 爬到 20 km 以上
        Until.below("range", 5000.0)      # 快到目标了
        Until.event("burnout")            # 发动机熄火
    """

    variable: str
    op: str = ">"
    value: float = 0.0

    def __post_init__(self) -> None:
        if self.variable in UNTIL_FLAGS:
            if self.op:
                raise ParamError(
                    f"事件 {self.variable!r} 不带比较符（写成 Until.event(...)），"
                    f"收到 op={self.op!r}"
                )
            return
        if self.variable not in UNTIL_VARIABLES:
            known = "、".join(sorted(UNTIL_VARIABLES) + list(UNTIL_FLAGS))
            raise ParamError(
                f"阶段推进条件里的变量 {self.variable!r} 不认识（可用：{known}）"
            )
        if self.op not in _COMPARISONS:
            options = "、".join(sorted(_COMPARISONS))
            raise ParamError(
                f"阶段推进条件的比较符 {self.op!r} 不认识（可用：{options}）"
            )

    # -- 构造器 ------------------------------------------------------------

    @classmethod
    def after(cls, seconds: float) -> "Until":
        return cls("phase_time", ">", float(seconds))

    @classmethod
    def above(cls, variable: str, value: float) -> "Until":
        return cls(variable, ">", float(value))

    @classmethod
    def below(cls, variable: str, value: float) -> "Until":
        return cls(variable, "<", float(value))

    @classmethod
    def event(cls, name: str) -> "Until":
        return cls(name, "", 0.0)

    # -- 求值 --------------------------------------------------------------

    def test(self, state: "FlightState") -> bool:
        if self.variable in UNTIL_FLAGS:
            return state.flag(self.variable)
        return _COMPARISONS[self.op](state.value(self.variable), self.value)

    def describe(self) -> str:
        if self.variable in UNTIL_FLAGS:
            return self.variable
        return f"{self.variable} {self.op} {self.value:g}"


# ---------------------------------------------------------------------------
# 飞行状态（弹体 → 制导件）
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class FlightState:
    """弹体报上来的状态。制导件的**全部**输入。

    ``target_range`` 与 ``closing_speed`` 由**制导件自己填**（它才知道目标在
    哪），弹体那侧留默认值。所以弹体造出来的状态里 ``range`` 是 ``inf``——
    这不影响它自己用，只影响推进条件，而那些条件本来就是制导件在求值。
    """

    #: 从开工算起（秒）。**弹体自己的钟**，不是引擎时刻——弹在发射车上等
    #: 一小时再打出去，``flight_time`` 也该是 0。
    flight_time: float = 0.0
    #: 进入当前阶段之后过了多久（秒）。由制导件填。
    phase_time: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    #: 脚下地面高程（米）。弹体查过地形，制导件因此不必认识地图服务。
    ground_z: float = 0.0
    #: 航向（度，0 = 北，90 = 东）。
    heading: float = 0.0
    #: 水平速率（m/s）——与存储里那个数同源。
    speed: float = 0.0
    #: 垂向速率（m/s，正 = 上升）。
    vertical_speed: float = 0.0
    #: **弹道倾角**（度，正 = 爬升）。由弹体给，**不是**从上面两个数推的。
    #:
    #: 为什么要单独一格：``atan2(vertical_speed, speed)`` 在**速度为零**时
    #: 没有定义（零矢量的方向任意），而弹体那一刻用的是发射姿态。两边各推
    #: 一次的话会推出两个不同的数——实测过：垂直发射的弹（发射倾角 85°）
    #: 在第一个子步里，制导件的 ``g·cosγ`` 用的是 γ=0 而弹体用的是 85°，
    #: 于是"抵消重力的那一份"多出 8.95 m/s²，沿法向把弹推向**反方向**，
    #: 整条弹道跑到南边去了（落点 +17.96 km，本该是 −17.5 km）。
    #:
    #: 有了这一格，两个部件用的是**同一个** γ，"抵消重力"才不会重复计算。
    flight_path_angle: float = 0.0
    #: 当前质量（kg，已扣掉烧掉的燃料）。
    mass: float = 0.0
    #: 这一小步发动机在工作吗。
    thrusting: bool = False
    #: 点过火、而现在没有推力了（燃料烧完或本阶段关机）。
    burnout: bool = False
    #: 已飞水平里程（米）。
    downrange: float = 0.0
    #: 到目标的斜距（米）。无目标 = ``inf``。
    target_range: float = inf
    #: 接近速率（m/s，正 = 在靠近）。
    closing_speed: float = 0.0
    #: 目标在哪个方位（度，0 = 北，90 = 东）。
    target_heading: float = 0.0
    #: **视线高低角**（度，正 = 目标在弹**上方**）。
    #:
    #: 与下面 :attr:`target_elevation_rate` 是同一支视线的位置与速度：那个是
    #: ``λ̇_v``（rad/s），这个是 ``λ_v`` 本身（``atan2(dz, r_xy)``）。加它是因为
    #: 推进条件需要"目标在视线下方多少度"这一类判据（AFSIM 的 ``POPUP`` 段用
    #: ``target_elevation < -20 deg`` 切进 ``DIVE``），而"自己在下降"
    #: （``descending``）回答的是另一个问题——跃升段里弹在爬升，条件却已经该
    #: 成立了。没有目标时是 0.0：``< -20`` 与 ``> 89`` 因此都不成立，弹不会
    #: 因为"没找到目标"而跳段。
    target_elevation: float = 0.0
    #: 视线方位角变化率（rad/s）——水平比例导引的 λ̇。
    target_azimuth_rate: float = 0.0
    #: 视线高低角变化率（rad/s）——垂向比例导引的 λ̇。
    target_elevation_rate: float = 0.0

    # -- 派生 --------------------------------------------------------------

    @property
    def altitude_agl(self) -> float:
        return self.z - self.ground_z

    @property
    def descending(self) -> bool:
        return self.vertical_speed < 0.0

    # -- 条件求值 ----------------------------------------------------------

    def value(self, name: str) -> float:
        if name == "phase_time":
            return self.phase_time
        if name == "flight_time":
            return self.flight_time
        if name == "altitude":
            return self.z
        if name == "altitude_agl":
            return self.altitude_agl
        if name == "speed":
            return self.speed
        if name == "vertical_speed":
            return self.vertical_speed
        if name == "range":
            return self.target_range
        if name == "closing_speed":
            return self.closing_speed
        if name == "downrange":
            return self.downrange
        if name == "target_elevation":
            return self.target_elevation
        raise ParamError(f"飞行状态里没有 {name!r} 这个量")

    def flag(self, name: str) -> bool:
        if name == "burnout":
            return self.burnout
        if name == "descending":
            return self.descending
        raise ParamError(f"飞行状态里没有 {name!r} 这个事件")


# ---------------------------------------------------------------------------
# 一帧指令（制导件 → 弹体）
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class GuidanceCommand:
    """制导件给弹体的一小步指令。

    **每一项都是"想要"，不是"做到"**：弹体按自己的能力（推力、燃料、过载
    上限）把它变成实际位移，做不到就是做不到——那正是推进条件要看的信号。
    """

    #: 当前阶段名。只为诊断：弹体不据它做判断。
    phase: str = ""
    #: 这一小步允许发动机工作吗。燃料烧完自动变假（弹体管这件事）。
    thrusting: bool = True
    #: 速率指令（m/s）。``None`` = **不接管**：速率由推力、重力、阻力决定，
    #: 那才是助推段与自由飞段的正确行为。给了值就是"油门按它调"。
    speed_mps: float | None = None
    #: 水平横向加速度（m/s²，正 = 右转，航向增大）。
    lateral_accel: float = 0.0
    #: **总**垂向加速度（m/s²，正 = 向上），含抵消重力的那一份。
    #: 0 = 没人管垂直，只剩重力。
    vertical_accel: float = 0.0
    #: 到此为止。弹体停止积分并记下原因。
    terminate: bool = False
    #: 停止的原因（``"命中"`` / ``"飞太久"`` 之类），写进诊断。
    reason: str = ""


# ---------------------------------------------------------------------------
# 阶段（组件参数里的剧本）
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Phase:
    """一个飞行阶段：**要什么** + **什么时候换**。

    它对应 AFSIM 的 ``phase`` 块。两个字段决定"要什么"——``vertical`` 管
    垂直那一层，``horizontal`` 管水平横向那一层，两者独立，可以任意组合
    （``VERT_FREE`` + ``HORIZ_PN`` 是合法的：只做水平导引、垂直交给重力）。

    ``vertical`` 留空时**由给定量推出**：给了 ``altitude`` 就是定高，给了
    ``flight_path_angle`` 就是定倾角，两个都没给就是 1 g 平衡。显式写下的
    ``vertical`` 与给定量冲突时报错——不报的话，``altitude=50`` 配
    ``vertical=VERT_FREE`` 会让那句高度指令**静默失效**，而写它的人正盯着
    "为什么没爬到 50 m"发呆。
    """

    name: str
    #: 什么时候进下一阶段。``None`` = 停在本阶段（一直飞下去）。
    until: Until | None = None
    #: 高度指令（米）。默认是**海拔**——与实体的 z 同基准。
    altitude: float | None = None
    #: 高度是离地的吗。真 = 距地面 ``altitude`` 米（AFSIM 的 ``agl``），
    #: 那是巡航弹地形跟随的写法。
    altitude_agl: bool = False
    #: 弹道倾角指令（度，正 = 爬升）。
    flight_path_angle: float | None = None
    #: 速率指令（m/s）。``None`` = 不接管（见 :attr:`GuidanceCommand.speed_mps`）。
    speed: float | None = None
    #: 垂向律，取 ``VERT_*`` 之一。``None`` = 由给定量推。
    vertical: str | None = None
    #: 水平律，取 ``HORIZ_*`` 之一。
    horizontal: str = HORIZ_HOLD
    #: 导引律增益。比例导引是 ``N``（AFSIM 默认 3），追踪导引是 1/时间常数。
    #: ``None`` = 用组件的 ``intercept_gain`` / ``pursuit_gain``。
    gain: float | None = None
    #: 本阶段的机动加速度上限（m/s²）。``None`` = 用弹体自己的
    #: ``radial_accel``。制导件先夹一道、弹体再夹一道，取更严的那个。
    max_accel: float | None = None
    #: 本阶段允许点火吗。假 = 提前关机。
    motor: bool = True

    def __post_init__(self) -> None:
        if not self.name:
            raise ParamError("飞行阶段必须有名字——诊断时全靠它")

        law = self.vertical
        if law is None:
            if self.altitude is not None:
                law = VERT_ALTITUDE
            elif self.flight_path_angle is not None:
                law = VERT_FPA
            else:
                law = VERT_BALANCE
            object.__setattr__(self, "vertical", law)

        if law not in VERTICAL_LAWS:
            options = "、".join(VERTICAL_LAWS)
            raise ParamError(
                f"阶段 {self.name}: 垂向律 {law!r} 不认识（可用：{options}）"
            )
        if self.horizontal not in HORIZONTAL_LAWS:
            options = "、".join(HORIZONTAL_LAWS)
            raise ParamError(
                f"阶段 {self.name}: 水平律 {self.horizontal!r} 不认识（可用：{options}）"
            )

        if law == VERT_ALTITUDE and self.altitude is None:
            raise ParamError(f"阶段 {self.name}: 定高律缺 altitude")
        if law != VERT_ALTITUDE and self.altitude is not None:
            raise ParamError(
                f"阶段 {self.name}: 写了 altitude 却把垂向律定成 {law!r}"
                f"——那句高度指令不会生效"
            )
        if law == VERT_FPA and self.flight_path_angle is None:
            raise ParamError(f"阶段 {self.name}: 定倾角律缺 flight_path_angle")
        if law != VERT_FPA and self.flight_path_angle is not None:
            raise ParamError(
                f"阶段 {self.name}: 写了 flight_path_angle 却把垂向律定成 {law!r}"
                f"——那句倾角指令不会生效"
            )
        if self.altitude is not None and self.altitude < 0.0:
            raise ParamError(f"阶段 {self.name}: 高度不能为负，收到 {self.altitude}")
        if self.speed is not None and self.speed < 0.0:
            raise ParamError(f"阶段 {self.name}: 速率不能为负，收到 {self.speed}")
        if self.gain is not None and self.gain < 0.0:
            raise ParamError(f"阶段 {self.name}: 增益不能为负，收到 {self.gain}")
        if self.max_accel is not None and self.max_accel < 0.0:
            raise ParamError(
                f"阶段 {self.name}: 过载上限不能为负，收到 {self.max_accel}"
            )

    # -- 诊断 --------------------------------------------------------------

    def describe(self) -> str:
        bits = [f"垂向={self.vertical}"]
        if self.altitude is not None:
            bits.append(
                f"高度={self.altitude:g} m{'(ag l)' if self.altitude_agl else ''}"
            )
        if self.flight_path_angle is not None:
            bits.append(f"倾角={self.flight_path_angle:g}°")
        if self.speed is not None:
            bits.append(f"速率={self.speed:g} m/s")
        bits.append(f"水平={self.horizontal}")
        if self.until is not None:
            bits.append(f"到 {self.until.describe()}")
        else:
            bits.append("不再推进")
        return f"{self.name}[{'，'.join(bits)}]"


def gravity() -> float:
    """标准重力加速度（``m/s²``）。转出来是为了让调用点读起来像句话。"""
    return STANDARD_GRAVITY


__all__ = [
    "HORIZ_HOLD",
    "HORIZ_PN",
    "HORIZ_TARGET",
    "HORIZONTAL_LAWS",
    "SPEED_EPS",
    "FlightState",
    "GuidanceCommand",
    "Phase",
    "UNTIL_FLAGS",
    "UNTIL_VARIABLES",
    "Until",
    "VERT_ALTITUDE",
    "VERT_BALANCE",
    "VERT_FPA",
    "VERT_FREE",
    "VERT_PN",
    "VERTICAL_LAWS",
    "gravity",
]
