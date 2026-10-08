"""空中机动件：直线飞向目的地，按爬升/下降率改高度。"""

from __future__ import annotations

from typing import Any

from ...services.params import Params
from ...services.type_registry import register_component
from .base import Mover


@register_component("AIR_MOVER")
class AirMover(Mover):
    """空中机动：直线飞向目的地，按爬升/下降率改高度。

    空中**不做寻路**：天空没有通行代价，A* 在没有代价的地形上会退化成
    直线，白花搜索时间（§3.11 已说明空中不吃分层数据）。所以这里直接连
    直线，把复杂度花在高度剖面上。

    **画像留空（``PROFILE = None``）** 不是漏配：它同时意味着"不吃通行
    代价"与"没有地图时也照样飞"，而基类的落脚点检查在没有画像时恒为放行。

    **角速度上界由过载给，不是由固定的转弯率给**（``radial_accel 9.4 m/s2``
    约 0.96 g）。180 m/s 的巡航速度下它解出 2.99 °/s、半径 3.44 km——这才是
    飞机的量级（基类默认的 30 °/s 会给出 115 m，那是架航模）。换成固定
    ``turn_rate`` 则不对：同一条 3 °/s 的曲线在 60 m/s 隐含 0.31 g、在
    180 m/s 隐含 0.96 g——把过载上限藏进了速度里，低速端浪费能力、高速端
    要求不该有的过载（§5.7.11）。

    ``turn_rate 12 °/s`` 保留作**低速端的机构上限**：过载那一项在 v→0 时
    发散（``a/v → ∞``），而真实飞机不会因为飞得慢就能瞬间转过来。两项取小
    之后 R(v) 是 U 形：高速由过载压住、低速由机构压住。

    于是飞机**进弯之前要减速**（基类的限速按"转弯半径 ≤ ``dist/(2·sin θ)``"
    给，θ 是航向误差），最后几公里明显慢下来：那是物理，不是 bug。

    不做的是**耦合剖面**（爬升时水平速度下降、能量高度、攻角与升力）——
    那要气动与推力模型，属后续里程碑。本件的高度与水平运动是解耦的：
    水平按 :attr:`PARAMS` ``max_speed`` 走，垂直按爬升/下降率走，两者各有
    自己的加速度上限（``linear_accel`` / ``vertical_accel``）。
    """

    __slots__ = ()

    PROFILE = None

    PARAMS = {
        **Mover.PARAMS,
        "max_speed": Params.speed(200.0, minimum=0.0),
        "turn_rate": Params.angle_rate(12.0, minimum=0.0),
        #: **径向加速度（过载）上限** 9.4 m/s² ≈ 0.96 g。它按 ``ω ≤ a/v``
        #: 换成角速度上界，于是"速度越高越转不动"是**算出来的**，不是写死的。
        "radial_accel": Params.accel(9.4, minimum=0.0),
        #: 水平加速 5 m/s²：0→180 m/s 要 36 s。真实战斗机比这猛得多，但这里
        #: 是**巡航机**的量级——写 20 m/s² 会让每一次调速都变成瞬变，而那正是
        #: 这一层想消掉的东西。
        "linear_accel": Params.accel(5.0, minimum=0.0),
        #: 垂向加速度 8 m/s²：满爬升率 40 m/s 要 5 s 建立。它属于**推重比**
        #: 那一头，比水平加速度大是正常的——爬升靠剩余推力，而水平加速还要
        #: 克服全部阻力。
        "vertical_accel": Params.accel(8.0, minimum=0.0),
        #: 转向建立 6 °/s²：从平飞把角速度推到低速端的机构上限 12 °/s 要 2 s
        #: 的滚转时间。与地面件同理，5 s 的周期上一帧就饱和——那是飞机响应
        #: 本来就比一帧快。**注意它管的是"建立"，不管"上界"**：巡航速度下
        #: 真正卡住转向的是 ``radial_accel`` 那一项（3 °/s），不是这个 12。
        "angular_accel": Params.angle_accel(6.0, minimum=0.0),
        #: 巡航高度（米，**海拔**——与实体的 z 同基准）。
        #: 战区网格的高程以海平面为 0，所以"离地 3000 m"要加上地面高程，
        #: 这里刻意用海拔：空中单位关心的是与地面的绝对高度差（雷达视距、
        #: 地空导弹包线都按海拔算）。
        "cruise_altitude": Params.distance(3000.0, minimum=0.0),
        "climb_rate": Params.speed(40.0, minimum=0.0),
        "descent_rate": Params.speed(60.0, minimum=0.0),
        #: 高度到位容差。判"到达"时用：到了目标点上空还在爬升的那一段
        #: 不该算到达，否则飞机会在离巡航高度还差几千米时停住。
        "altitude_tolerance": Params.distance(100.0, minimum=0.0),
    }

    def _speed_at(self, ref: Any) -> float:
        """空中不受地形影响：天空没有"这里难走"这回事。"""
        return float(self.spec["max_speed"])

    def _waypoint_z(self, ref: Any, x: float, y: float) -> float:
        return float(self.spec["cruise_altitude"])

    def _approach_z(
        self, ref: Any, x: float, y: float, z: float, dt_s: float
    ) -> float:
        """按爬升/下降率走向高度目标，速率本身按 ``vertical_accel`` 建立。

        目标取**下一个航路点**的高度；航路点全走完了就继续朝最后一个爬/降。
        少了后半句，飞机会在到达目标点上空的那一刻停在半空——而它离巡航
        高度可能还差几千米。

        "速率 + 加速度"两步交给基类的 :meth:`Mover._glide_z`：它与水下件
        共用同一段（两者的差别只在参数方向，一个爬上、一个潜下）。

        不覆盖 :meth:`Mover._advance`：水平推进那一段本来只差一句"不要检查
        通行性"，而基类的检查在没有画像时（``PROFILE = None``，空中正是
        如此）恒为"能过"，所以那段代码可以完全共用。早先这里有一份复制品，
        后来发现它和基类只差这一处——两处各改一次迟早会漏一处。
        """
        target = self.next_waypoint()
        if target is None and self._waypoints:
            target = self._waypoints[-1]
        if target is None:
            return z
        return self._glide_z(
            z,
            target[2],
            float(self.spec["climb_rate"]),
            float(self.spec["descent_rate"]),
            dt_s,
        )

    def _at_altitude(self, target: tuple[float, float, float] | None) -> bool:
        if target is None:
            return True
        pose = self.view.my_pose()
        if pose is None:
            return True
        return abs(pose[2] - target[2]) <= float(self.spec["altitude_tolerance"])


__all__ = ["AirMover"]
