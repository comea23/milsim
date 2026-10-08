"""巡航导弹的制导件：爬升 → 定高巡航 → 末段导引（§5.8）。

剧本（AFSIM 的 ``blue_cruise_missile.txt`` 是同一形状）
------------------------------------------------------
.. code-block:: text

    LAUNCH    爬升到巡航高度、把头对准目标
    CRUISE    贴地 50 m 定高、240 m/s 定速、航向始终指目标
    TERMINAL  末段比例导引 + 垂向比例导引

三段的**形态**差异全在表里，弹体一点都不知道：它只会"按法向加速度转、
按速率指令调油门"。所以换一种弹只要换这张表。

参考配套（弹体的性能参数，写在想定的 ``component_type`` 里）
----------------------------------------------------------
.. code-block:: text

    component_type SSM_CRUISE : MISSILE_MOVER
        mass             1200 kg
        fuel_mass         250 kg
        thrust            1.6 kN
        specific_impulse  3000 s
        drag_area         0.024 m2
        radial_accel      12 m/s2
        max_speed         300 m/s
        launch_speed      240 m/s    ← 载机投放 / 助推段交出来的初速
        launch_fpa          0 deg
    end_component_type

``launch_speed`` **非有不可**：这台涡扇的推重比只有 ``1.6 kN ÷ (1200 kg ×
9.8) = 0.14``——**比重量小**，从静止起步它连速度系都建立不起来。真实世界那段
速度是载机或助推器给的（AFSIM 靠 ``WSF_LAUNCH_COMPUTER`` 与发射平台状态给）。

**没有初速时那不是一个"飞得慢一点"的配置，而是一个错配置**：本件不建升力
（见 :mod:`milsim.models.mover.missile` 的"已知限制"），推力小于重量的弹没有
任何东西能把速度矢量转上去，实测 1000 s 里速度在 0.1~0.5 m/s 之间把航向在
0°/180° 来回翻、烧掉 100 kg 燃料而位置只挪了 4 km。所以它现在**报错**（
:meth:`~milsim.models.mover.missile.MissileMover._require_static_launch`），
而不是跑出一条看着像"参数没生效"的假航迹。

其余几个数不是随手写的：``3000 s`` 是**涡扇**的比冲量级（火箭是 250 s 上下），
``0.024 m²`` 与 ``1.6 kN`` 配起来在 240 m/s 巡航时阻力约 0.85 kN——推力用掉
一半，真实巡航弹就是被阻力而不是被推力限制的。

**射程是算出来的（实测）**
------------------------
250 kg 燃料 ÷ 约 0.0286 kg/s（240 m/s 定速巡航时的耗油率）≈ 8700 s，乘
240 m/s 得到 **2000 km 量级**。实测**包线 20~2200 km**（正北与往东偏 10 km
两种目标都测过）：40 / 100 / 500 / 1000 / 1900 / 2100 / 2200 km 全部命中，
**2300 km 起燃料烧完**（终止原因"飞太久"）——所以射程由燃料定，改
``fuel_mass`` 就改射程。

**下边缘 20 km 是算法边界，不是弹打不动**：末段交接条件是 ``range < 20 km``，
目标更近时"交接"发生在**发射爬升段里**——弹刚离开发射位就进末段，于是它带着
爬升的姿态去追一个就在眼前的目标，一头扎进地里。10 km 的目标实测就是这样
（报"落地"，离目标 9.6 km）。要让巡航弹打近目标得改阶段表（比如把交接条件
再带一个高度条件），不是改参数。

**打击精度（实测）**：全程采样的最近通过距离 0.08~2.22 m。这里有个读法要
注意——末段的采样间隔是 ``0.02 s × 240 m/s ≈ 5 m``，所以 ~2 m 这个数是
**采样精度**而不是脱靶量（轨迹穿过目标时，落在最近点前后 2 m 的那次采样也会
读成 ~2 m）。真正说明精度的是：``lethal_radius`` 收到 **5 m** 时全部仍然命中。
``detonation_range_m()`` 报的 23~30 m 又是**第三个数**（第一个落进引信点火区
的采样点的距离），见
:meth:`~milsim.models.guidance.base.GuidanceComputer.detonation_range_m`。

不做的：地形跟随用的是"离地 50 m"这个**跟随高度**，不是真正的跟随算法
（没有前瞻、不预判前方山脊）。AFSIM 的 ``commanded_altitude 500 ft agl``
语义与这一样：它也只保证"此刻离地多少"。
"""

from __future__ import annotations

from ...services.params import Params
from ...services.type_registry import register_component
from .base import GuidanceComputer
from .contract import (
    HORIZ_PN,
    HORIZ_TARGET,
    VERT_PN,
    Phase,
    Until,
)

#: 巡航弹的阶段表。**换掉它就是另一种弹**（这就是"phase 作为组件参数"的意思）。
CRUISE_PHASES: tuple[Phase, ...] = (
    # 发射段：一面爬高一面加速。航向先保持——刚离轨的弹急着转向只会白费能量。
    Phase(
        "LAUNCH",
        until=Until.above("altitude_agl", 130.0),
        altitude=150.0,
        altitude_agl=True,
        speed=240.0,
    ),
    # 巡航段：贴地 50 m 定高定速，航向总是指着目标。**离地**高度是刻意的——
    # 海拔高度在山地上会一头撞山，而"离地 50 m"在任何地形上都成立。
    Phase(
        "CRUISE",
        until=Until.below("range", 20_000.0),
        altitude=50.0,
        altitude_agl=True,
        speed=240.0,
        horizontal=HORIZ_TARGET,
    ),
    # 末段：两个方向都用比例导引。垂向那一路非有不可——只用水平导引的弹会
    # 平着撞向目标点，而目标在地面上。
    Phase(
        "TERMINAL",
        vertical=VERT_PN,
        horizontal=HORIZ_PN,
        gain=4.0,
        max_accel=30.0,
    ),
)


@register_component("CRUISE_GUIDANCE")
class CruiseGuidance(GuidanceComputer):
    """巡航弹制导：定高 + 定速 + 末段比例导引。

    这是**参考实现**，不是演示件——想定里直接写
    ``component guidance CRUISE_GUIDANCE`` 配 ``MISSILE_MOVER`` 就能用。
    要另一种巡航弹（不同巡航高度、不同速度、不同导引增益）就派生一个类，
    改 ``phases`` 默认值，或者在想定的 ``component_type`` 里改本件的参数。
    """

    __slots__ = ()

    PARAMS = {
        **GuidanceComputer.PARAMS,
        #: 剧本。框架给的是上面那张表；要另一种巡航弹就在派生类里换掉它。
        "phases": Params.phases(CRUISE_PHASES),
    }


__all__ = ["CRUISE_PHASES", "CruiseGuidance"]
