"""反舰弹的制导件：贴海巡航 → 末段跃升 → 俯冲（§5.8，验收见 §5.11）。

剧本（照抄 AFSIM ``demos/new_guidance`` 那份想定）
--------------------------------------------------
.. code-block:: text

    CRUISE    贴地 75 m 定高、537 mph 定速，斜距 3 km 处交班
    POPUP     一路爬到 6000 m ——"我们只需向上飞行直到目标俯仰角达到正确值"
    DIVE      两个方向都用比例导引俯冲下去，过载上限留给弹体

与 :class:`~milsim.models.guidance.cruise.CruiseGuidance` 的**唯一**差别
就是这张表：同样是"定高 + 定速 + 末段导引"，用的是同一套律。它单独一个
类，是因为"跃升俯冲"这件事需要 ``target_elevation`` 这个推进条件才表达
得出来（见下），不是因为算法不同。

``POPUP → DIVE`` 靠的是"**看目标**的角度"
------------------------------------------
原脚本写 ``next_phase DIVE when target_elevation < -20 deg``。它**不是**
"自己在下降"：跃升那整段弹都在爬升，``descending`` 一帧都不成立——所以
在补上 ``target_elevation`` 之前，这张表飞不出后两段（§9-33）。两者差的
是"看目标的角度"与"自己的垂向状态"。

配套的弹体**得拉得出 4.6 g**
----------------------------
实测（``tests/test_missile.py``）：同一张表配 ``radial_accel`` 12 / 30 m/s²
时**不命中**——弹在目标**头顶 750 m** 掠过（只来得及抬头 15°）；45 m/s²
起才落进杀伤半径（最近通过 28.8 m）。参照那枚弹在 POPUP 段用 6 s 把弹道
倾角从 0 抬到 67.6°，折合约 4.8 g（§5.11）——两边对得上。

所以"换一种弹 = 换一张表"这句话有个前提：**弹体拉得动这张表要的机动**。
表与弹体是配套的，不是随便配。

不做的两样（**v0.13.66 起都补上了**）
------------------------------------
1. ~~**航路跟随**（§9-31）~~ ⇒ 补上：:data:`ASM_ROUTE_PHASES` 把巡航段的水平律
   换成 ``route``（v0.13.63 的途径点律），注册名 :class:`RouteAsmGuidance`
   （``ROUTE_ASM_GUIDANCE``）。途径点序列写在制导件的 ``route`` 参数里
   （想定里在 guidance 组件块赋值，平面坐标米），按序飞完后自动转回追踪
   目标——与参照"先走完 mover 里的 route、再转向目标"同一形状。巡航段的
   交接条件仍是 ``range < 3 km``：航路没走完但目标已近，照切 POPUP（参照
   的 ``next_phase POPUP when target_slant_range < 3000 m`` 也是这么写的）。
2. ~~**俯仰角上限**（§9-35）~~ ⇒ 补上：:class:`~milsim.models.guidance.contract.Phase`
   新增 ``max_pitch_angle``（度，只认定高律），定高律的垂直速率指令被钳到
   ``±v·sin(限值)``——与参照 ``maximum_pitch_angle 50 deg`` 同一语义
   （对称、作用于定高程序）。路线表的巡航段带着 ``50.0``；无航路的
   :class:`AsmGuidance` 保持旧表逐字不变。
"""

from __future__ import annotations

from dataclasses import replace

from ...services.params import Params
from ...services.type_registry import register_component
from .base import GuidanceComputer
from .contract import (
    HORIZ_PN,
    HORIZ_ROUTE,
    HORIZ_TARGET,
    VERT_PN,
    Phase,
    Until,
)

#: 反舰弹的阶段表。**换掉它就是另一种弹**（这就是"phase 作为组件参数"的意思）。
ASM_PHASES: tuple[Phase, ...] = (
    # 巡航段：贴地 75 m、240 m/s（原脚本的 537 mph）。航向指目标直扑——
    # 要沿预规划航路飞用下面的 ASM_ROUTE_PHASES（v0.13.66）。
    Phase(
        "CRUISE",
        until=Until.below("range", 3_000.0),
        altitude=75.0,
        altitude_agl=True,
        speed=240.0,
        horizontal=HORIZ_TARGET,
    ),
    # 跃升段：爬到 6000 m。原脚本的注释是"指令一个极高的高度……我们只需向上
    # 飞行直到目标俯仰角达到正确值"——那个"正确值"就是下一段的 −20°。
    # **不写 speed**：爬升要靠动能换高度（实测参照那枚 6 s 里从 240 掉到
    # 178 m/s），这里也不接管速率。
    Phase(
        "POPUP",
        until=Until.below("target_elevation", -20.0),
        altitude=6_000.0,
        horizontal=HORIZ_TARGET,
    ),
    # 俯冲段：**不写 max_accel**。过载上限留给弹体——俯冲要拉得比巡航狠，
    # 照抄巡航弹末段那个 30 m/s² 时实测打不中。
    Phase(
        "DIVE",
        vertical=VERT_PN,
        horizontal=HORIZ_PN,
        gain=5.0,
    ),
)


@register_component("ASM_GUIDANCE")
class AsmGuidance(GuidanceComputer):
    """反舰弹制导：贴海巡航 + 末段跃升俯冲。

    配一个能拉 4.6 g 的弹体（``radial_accel 45 m/s2`` 起）才成立，见模块头。
    要另一种反舰弹（不同巡航高度、不同跃升高度、不同切段角）就派生一个类改
    ``phases`` 默认值。
    """

    __slots__ = ()

    PARAMS = {
        **GuidanceComputer.PARAMS,
        #: 剧本。框架给的是上面那张表；要另一种反舰弹就在派生类里换掉它。
        "phases": Params.phases(ASM_PHASES),
    }


#: 途径点反舰弹的阶段表（v0.13.66，§9-31/§9-35）：与 :data:`ASM_PHASES`
#: 差两处——巡航段水平律 ``target`` → ``route``（沿制导件 ``route`` 参数的
#: 途径点飞，走完转回追踪目标），且巡航段带 ``max_pitch_angle 50``（参照
#: 脚本的 ``maximum_pitch_angle 50 deg``：高空发射转入贴海时俯冲角被夹在
#: 50°，而不是定高律按"高度差 × 增益"算出的近乎垂直下扎）。POPUP / DIVE
#: 两段照旧——参照的俯仰限位也只写在 CRUISE 段。
ASM_ROUTE_PHASES: tuple[Phase, ...] = (
    replace(
        ASM_PHASES[0],
        horizontal=HORIZ_ROUTE,
        max_pitch_angle=50.0,
    ),
    ASM_PHASES[1],
    ASM_PHASES[2],
)


@register_component("ROUTE_ASM_GUIDANCE")
class RouteAsmGuidance(AsmGuidance):
    """途径点反舰弹：贴海巡航（沿 ``route`` 途径点）+ 末段跃升俯冲。

    与 :class:`AsmGuidance` 只差默认表里那一个水平律与巡航段的俯仰限位。
    ``route`` 留空会在装配期报错（基类初始化的检查）；途径点的几何语义
    （半径内算过点、转弯受过载限制）见
    :data:`~milsim.models.guidance.contract.HORIZ_ROUTE`。
    """

    __slots__ = ()

    PARAMS = {
        **AsmGuidance.PARAMS,
        "phases": Params.phases(ASM_ROUTE_PHASES),
    }


__all__ = [
    "ASM_PHASES",
    "ASM_ROUTE_PHASES",
    "AsmGuidance",
    "RouteAsmGuidance",
]
