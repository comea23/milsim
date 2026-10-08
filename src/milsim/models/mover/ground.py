"""地面机动件：A* 绕地形 + 路点跟随 + 按地形减速，z 贴地。

只依赖 :mod:`milsim.models.mover.base`——**不继承任何别的参考实现**。
水位件与它共用同一套寻路与推进（都在基类里），差别全在画像与高度，
那两样各写各的就够了。
"""

from __future__ import annotations

from typing import Any

from ...services.map.nav import PROFILE_GROUND
from ...services.params import Params
from ...services.type_registry import register_component
from .base import Mover


@register_component("GROUND_MOVER")
class GroundMover(Mover):
    """地面机动：A* 绕地形 + 路点跟随 + 按地形减速，z 贴地。

    这是**参考实现**，不是演示件——想定里直接写 ``component mover GROUND_MOVER``
    就能用。要改底盘特性（履带 / 轮式 / 半履带）就派生一个新组件类，
    参数差异写在 ``component_type`` 里（§5.3：代码管算法，想定管参数）。

    三个通道的第二层都给了（``linear_accel`` / ``angular_accel`` /
    ``radial_accel``）。垂向那一层**不接通**：地面件的 z 是"贴当地地面高程"，
    是**位形约束**而不是速率控制——车翻过一道梁时 z 是被地形带上去的，
    没有"多久能爬到那个高度"这回事（图见 ``docs/images/mover.png`` 的
    高度那一面板：三条地面/水面线是实测出来的，不是算出来的）。
    """

    __slots__ = ()

    PROFILE = PROFILE_GROUND

    PARAMS = {
        **Mover.PARAMS,
        #: **抓地力上限** 0.5 g ≈ 5 m/s²。它才是高速端真正卡住转向的那一项：
        #: 25 m/s 下角速度上界是 ``min(30, 5/25 rad/s = 11.5 °/s) = 11.5 °/s``，
        #: 半径 125 m。只写 ``turn_rate 30`` 的话，90 km/h 下等于 1.2 g 侧向
        #: 过载——那是赛车轮胎，不是轮式侦察车（§5.7.11）。
        "radial_accel": Params.accel(5.0, minimum=0.0),
        #: 直线加速度 3 m/s²（约 0.3 g）：0→25 m/s 要 8 s。
        "linear_accel": Params.accel(3.0, minimum=0.0),
        #: 转向建立速率 12 °/s²：方向盘到满舵不到 1 s。**在 5 s 的机动周期
        #: 上它一帧就饱和**——这不是"参数不生效"，是车辆的真实响应本来就比
        #: 一帧短。周期调到 0.5 s 时它立刻看得出来（见 ``test_mover.py`` 里
        #: 那条按时间步长测角加速度的用例）。
        "angular_accel": Params.angle_accel(12.0, minimum=0.0),
    }

    def _surface_z(self, ref: Any, x: float, y: float, z: float) -> float:
        """贴地：z 拉回当前格的地面高程。

        不做的是**地形跟随的动力学**（爬坡限速、越障、悬挂）——那需要
        坡度剖面与车辆模型，属后续里程碑。这里只保证"位置数据自洽"：
        一辆车在山顶时 z 就该是山顶的高程。
        """
        if self._nav is None:
            return z
        ground = self._nav.ground_at(ref, x, y)
        return ground if ground is not None else z

    def _waypoint_z(self, ref: Any, x: float, y: float) -> float:
        if self._nav is None:
            return 0.0
        ground = self._nav.ground_at(ref, x, y)
        return ground if ground is not None else 0.0


__all__ = ["GroundMover"]
