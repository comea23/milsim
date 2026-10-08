"""水面机动件：与地面件同一套 A* 与路点跟随，换一张画像、恒贴海面。"""

from __future__ import annotations

from typing import Any

from ...services.map.nav import PROFILE_WATER
from ...services.params import Params
from ...services.type_registry import register_component
from .base import Mover


@register_component("WATER_MOVER")
class WaterMover(Mover):
    """水面机动：同一套 A* 与路点跟随，换一张画像。

    只覆盖 :attr:`PROFILE` 与两个高度钩子——这就是把"能不能走"的判断放进
    :class:`~milsim.services.map.path.CostConfig` 的收益：舰船不必有自己
    的一套寻路代码，否则"起点在陆地上"这类判定就会在地面口径下被判成
    "不可通行"而**永远找不到路**（不报错，只是不动）。

    **不继承 `GroundMover`。** 早先它是 ``WaterMover(GroundMover)``，但两个
    高度钩子都被它覆盖了，从父类**一分复用都没有**——那条继承只剩一个效果：
    改地面件会牵动水面件。两者是并列的参考实现，共用的部分本来就在基类里。

    **吃水是这条船的参数，不是这张画像的**：``min_water_depth`` 写在这个
    型号的参数里（想定或参数库），参考实现不给默认值。给一个"看起来合理"
    的默认吃水才是真正的危险——每条船都会被套上一个没人声明过的数字，
    而且浅水区从此静默地"能过"。所以默认 0 = 不限制，写没写一目了然。

    **转向速率覆盖成 3 °/s。** 舰船的转弯半径是几百米量级，而基类默认的
    30 °/s 是车辆量级（20 m/s 下半径不到 40 m）——船要真按那个转，就成
    了在港池里打转的摩托艇。覆盖不是"另立一个数"，是把"这是船"这句话
    落到数字上。**不写 ``radial_accel``**：舰船的航速基本不变，而"回转
    半径与航速无关"（战术直径）这条与固定 ``turn_rate`` 等价；过载口径
    是给速度会变的件用的（§5.7.11）。

    三个通道的第二层都给得很小（``linear_accel 0.1`` / ``angular_accel 0.3``）：
    一条船 120 s 才加到全速、10 s 才建立起满舵角速度。它们在图上看得出来
    ——起弯处是**渐弯**的，而不是一段折线接一段圆弧。垂向那一层不接通：
    水面的 z 恒为海平面，是**位形约束**，没有速率这回事。

    另外一件还没做的事：潮汐、海况、浅水效应（越浅越慢）都没有。现在
    水深只是一个**通过/不通过**的门槛。
    """

    __slots__ = ()

    PROFILE = PROFILE_WATER

    PARAMS = {
        **Mover.PARAMS,
        "turn_rate": Params.angle_rate(3.0, minimum=0.0),
        #: 舰船的直线加速度很小：0.1 m/s²（约 0.01 g）。0→12 m/s 要 120 s，
        #: 这正是"船不像车"那件事——它是**排水量**决定的，不是螺旋桨决定的。
        "linear_accel": Params.accel(0.10, minimum=0.0),
        #: 转向建立 0.3 °/s²：满舵角速度 3 °/s 要 10 s 才建立起来。船越大
        #: 越慢，而它带来的后果在图上看得出来——航迹的起弯处是**渐弯**的，
        #: 不是一段折线接一段圆弧。
        "angular_accel": Params.angle_accel(0.3, minimum=0.0),
    }

    def _surface_z(self, ref: Any, x: float, y: float, z: float) -> float:
        """水面恒为海平面 0 m（§9 第 13 条：海平面定死 0，水深由网格派生）。"""
        return 0.0

    def _waypoint_z(self, ref: Any, x: float, y: float) -> float:
        return 0.0


__all__ = ["WaterMover"]
