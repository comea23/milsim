"""平台基类：平台型参数表的落点（§5.6）。

它**不是**行为主体
------------------
行为在组件里（§5）。平台类只做一件事：给 ``platform_type`` 一个能挂
``PARAMS`` 的地方，让"平台属性"也能像组件参数一样被校验、被参数库收录、
被同一套 ``ParamSet.resolve`` 解析。

所以这里没有 ``update()``、没有 ``initialize()``、没有 ``__slots__`` 状态。
AFSIM 的 ``WsfPlatform`` 是行为主体（平台自己会动、会感知），本项目刻意
不这样切：一旦平台会动，"这辆车能跑多快"就有两处真相——平台属性一份、
机动件参数一份——而两份数字迟早不同步。§5.3 已经定过：**代码层管算法
（组件类），想定层管参数（类型块）**，平台层只留参数。

为什么不干脆不写类、只写一张表
------------------------------
``PARAMS`` 的合并靠 ``**父类.PARAMS``（§5.2 ①），那是类继承给的。用一张
裸表就得多写一套合并逻辑，而且"这型平台的参数从哪继承来"要翻两个地方。

隐式根 ``PLATFORM``
-------------------
每个平台类型都共享一组基础参数（阵营、决策配置名），所以
:class:`~milsim.services.type_registry.PlatformRegistry` 把 ``PLATFORM``
当作**隐式根**：类型链上找不到具体类时落到它，而不是报错。这与组件侧
要求显式写 ``component_type HEX_SEARCH_RADAR`` 空块的差异是刻意的——
组件类型必须指向一个具体算法，平台没有算法。

新增参数的门槛
--------------
**只加"有人读"的参数。** ``side`` 由实体建档时取用、``decision`` 由决策接线
取用、``radar_cross_section`` 由感知层取用（雷达方程里的 σ 是目标的属性，
不是雷达的——见 ``RadarSensor._rcs_of``）。呼号、编制、车长还没人读，就先
不加。§9 第 15 条要消灭的正是"写了没人读、拼错也不报错"的平台属性，不是要
换一张更大的空表。

新增带默认值的参数不算破坏性改动（§5.4），所以后续里程碑往里加键是安全的。
"""

from __future__ import annotations

from typing import Mapping

from ..services.params import ParamSpec, Params
from ..services.type_registry import ROOT_PLATFORM, register_platform


@register_platform(ROOT_PLATFORM)
class Platform:
    """平台基类——**所有平台类型的隐式根**。

    用法（扩展一个自己型号族）::

        @register_platform("ARMOR_HULL")
        class ArmorHull(Platform):
            PARAMS = {**Platform.PARAMS, "crew": Params.integer(3, minimum=0)}

    想定里 ``platform_type TANK_MBT : ARMOR_HULL`` 就会用上这张表；
    不写父类型的平台类型落到 :class:`Platform` 本身。
    """

    #: 参数声明。键名是**契约**——改名等于破坏所有历史想定的兼容性。
    PARAMS: Mapping[str, ParamSpec] = {
        #: 阵营。实体建档时取用（``EntityRegistry.register(side=...)``）。
        #: 空串表示"未指定"——建帐之后不是 `red`/`blue` 的那一类。
        "side": Params.string(""),
        #: 决策配置名。装配时接线到 ``DecisionProvider``；空串表示用默认那份。
        #: 只校验"这个名字存在"，具体存在与否由想定层的跨块校验负责
        #: （``choices`` 是静态表，而决策配置名是想定期才有的）。
        "decision": Params.string(""),
        #: **雷达截面积**（m²）。感知层读它算回波功率。
        #: 默认 1 m² 与雷达的 ``reference_rcs`` 默认值相同——所以两边都不写
        #: 时不会出现"标定按 1 m²、目标按别的面积"这种前后矛盾。
        #: 雷达看见的**不是几何面积**：同一型飞机迎头与侧向能差一个数量级，
        #: 所以这个数是"给这台雷达的一个等效值"，不是外形尺寸。
        "radar_cross_section": Params.area(1.0, minimum=0.0),
    }

    #: 注册名。由 :func:`register_platform` 写入，便于调试与报错。
    PLATFORM_NAME = ""

    def __repr__(self) -> str:
        return f"<{type(self).__name__} {self.PLATFORM_NAME or '?'}>"


__all__ = ["Platform"]
