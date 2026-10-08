"""搜索雷达：旋转圆周 / 扇扫 / 固定指向。

它是 :class:`~milsim.models.percep.base.RadarSensor` 的**第一个具体实现**，
也是 ``library/*.txt`` 里那些型号（``RADAR_SEARCH_S`` / ``RADAR_SEARCH_L`` /
``RADAR_AA_1``）落到的那一层。注册名 ``HEX_SEARCH_RADAR`` 从 v0.13.23 起
没变过——改名的代价是所有历史想定与整份参数库同时失效。

它自己不写任何算法：参数、扫描律、几何、方程、建航全在基类里。这个文件
存在的意义是**族约定**（§5.7）：一族一个子包、一个实现一个文件、清单只在
子包里定义一处。所以"再加一型雷达"永远是"新增一个文件 + 在清单里加一行"，
而不是回到基类里加一个分支。
"""

from __future__ import annotations

from ...services.type_registry import register_component
from .base import RadarSensor


@register_component("HEX_SEARCH_RADAR")
class HexSearchRadar(RadarSensor):
    """把本帧照到的弧里的目标，按雷达方程判一次"看不看得见"，再按 M/N 建航。

    参数与库的对应关系（``library/external.txt``）：``scan_interval`` 就是
    "转一圈的时间"，radartutorial 那张表的 ``antenna_rotation`` 换算过来
    （JY-14 的 ``6 rpm`` ↔ 10 s 一圈，两边吻合）。

    ``fov`` / ``scan_center`` / ``beam_width`` 三个键合起来决定扫描模式。
    **默认 ``fov = 360`` 是圆周扫描**：已落库的 6 型雷达不写这三个键，行为与
    v0.13.24 逐字相同。

    与 v0.13.24 的差别只有一个：**``quality`` 的含义变了**。以前是
    ``1 - 距离/探测距离``（纯几何衰减，与功率无关），现在是**检测概率 Pd**
    ——"这条航迹有多可信"的直接读数。同时 ``detect_range`` 以外不再是硬截断
    （除非 ``max_range`` 也钉在那里），几何遮挡与 M/N 也会各挡掉一部分。
    想定输出里 ``航迹 n 条`` 因此可能比以前少，**那不是回归**。
    """


__all__ = ["HexSearchRadar"]
