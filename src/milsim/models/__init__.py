"""模型层：实体、部件、平台、侧挂控制器。

依赖方向：``models → services``，单向。模型层可以用服务层的任何东西，
服务层不知道模型层的存在——``EntityRegistry`` 只通过 ``RegisteredEntity``
协议声明它需要什么（两个属性），不 import 本层的任何类。

本层现有的东西分两类，**边界就在"是不是实体"上**：

| | 是实体吗 | 有 ID 空间 | 进注册表 / 存储 / 空间索引 | 雷达能探测到吗 |
|---|---|---|---|---|
| :class:`Entity` | 是 | 实体 ID | 进 | 能 |
| :class:`Controller` | **不是** | 无 | **不进** | **不能** |
| :class:`UnitState` | **不是** | 无（用编队 ID） | **不进** | **不能** |
| :class:`Platform` | **不是**（是类型） | 无 | **不进** | **不能** |

这条边界不是洁癖：编队级控制器若有实体身份，敌方雷达就会"探测到一个旅部"，
而旅部没有坐标——只能拿下辖部队质心冒充，聚合泄漏就换了个地方出现。

本层的两类东西容易混，分法是**参数 vs 行为**（§5.3）：

| | 是什么 | 有什么 |
|---|---|---|
| :class:`Platform` | 想定里的 ``platform_type`` 在代码侧的锚点 | **只有** ``PARAMS``（平台属性表） |
| :class:`Component` | 想定里的 ``component`` 在代码侧的锚点 | ``PARAMS`` + 生命周期钩子 + 算法 |

平台**不含行为**：行为在组件里。一旦平台自己会动，"这辆车能跑多快"就有
两处真相（平台属性一份、机动件参数一份），而两份数字迟早不同步。

导入本包会**注册**框架自带的参考实现（组件与平台各一批）——注册靠装饰器
在模块级执行，所以 ``import milsim.models`` 就是"把框架的东西装进注册表"。

组件多了以后按**族**分文件，每族一个子包、族里每个组件一个文件
（机动件是 :mod:`milsim.models.mover`）。本文件只做导出与汇总，
不写任何实现——这样"加一个组件"永远是"新增一个文件 + 在族清单里加一行"，
而不是在一千行的公共文件里找一个插入点。
"""

from __future__ import annotations

from typing import Any

from .comm import (
    FRAMEWORK_COMMS,
    CommNode,
    RfComm,
)
from .component import Component
from .controller import (
    CONTROLLER_LABELS,
    CONTROLLER_LEVELS,
    LEVEL_ENTITY,
    LEVEL_STRATEGIC,
    LEVEL_UNIT,
    NO_HOST,
    Controller,
    HostCheckFn,
)
from .entity import Entity
from .guidance import (
    FRAMEWORK_GUIDANCE,
    GuidanceComputer,
)
from .jam import (
    FRAMEWORK_JAMMERS,
    Jammer,
    RfJammer,
)
from .mount import MountContext
from .mover import (
    FRAMEWORK_MOVERS,
    MOVER_PERIOD_US,
    AirMover,
    GroundMover,
    MissileMover,
    Mover,
    SubsurfaceMover,
    WaterMover,
)
from .platform import Platform
from .percep import (
    FRAMEWORK_SENSORS,
    FRAMEWORK_TRACKS,
    TRACK_PERIOD_US,
    HexSearchRadar,
    RadarSensor,
    TrackManager,
)
from .unit_state import UnitState

#: 框架自带的**参考实现**（组件）。想定里可以直接写这些名字，不需要
#: 额外声明。
#:
#: 清单是**展开**来的，不是抄一遍：每个组件族自己维护自己那份
#: （机动件是 :data:`milsim.models.mover.FRAMEWORK_MOVERS`，制导件是
#: :data:`milsim.models.guidance.FRAMEWORK_GUIDANCE`，感知件是
#: :data:`milsim.models.percep.FRAMEWORK_SENSORS`，航迹处理件是
#: :data:`milsim.models.percep.FRAMEWORK_TRACKS`，通信件是
#: :data:`milsim.models.comm.FRAMEWORK_COMMS`），这里只负责把它们
#: 汇总。手工重列一遍的代价是"加了组件、想定里写了名字，却报没有实现类"
#: ——而报错信息看不出是漏登记还是拼错名。
FRAMEWORK_COMPONENTS: tuple[type, ...] = (
    *FRAMEWORK_MOVERS,
    *FRAMEWORK_GUIDANCE,
    *FRAMEWORK_SENSORS,
    *FRAMEWORK_TRACKS,
    *FRAMEWORK_JAMMERS,
    *FRAMEWORK_COMMS,
)

#: 框架自带的平台类。``PLATFORM`` 是所有平台类型的隐式根。
FRAMEWORK_PLATFORMS: tuple[type, ...] = (Platform,)


def register_framework(components: Any = None, platforms: Any = None) -> None:
    """把框架自带的参考实现装进**指定的**注册表。

    装饰器在模块导入时已经把类登记进全局表了，所以 ``import milsim.models``
    就等于"装好了"。这个函数是给"想用独立注册表跑"的场合用的：
    ``tools/run_scenario.py`` 就是——跑一份想定不该被别处注册过的东西影响，
    而一份全新的独立注册表里什么都没有。

    清单**必须在这里维护**。漏加一个类的表现是想定里写了那个组件名却报
    "没有实现类"——错得很响，不会静默降级。
    """
    for cls in FRAMEWORK_COMPONENTS:
        if components is not None:
            components.register(cls.COMPONENT_NAME, cls)
    for cls in FRAMEWORK_PLATFORMS:
        if platforms is not None:
            platforms.register(cls.PLATFORM_NAME, cls)


__all__ = [
    # -- 实体与部件 --
    "Component",
    "Entity",
    "MountContext",
    # -- 框架自带实现的清单 --
    "FRAMEWORK_COMPONENTS",
    "FRAMEWORK_PLATFORMS",
    "register_framework",
    # -- 平台型参数表（§5.6）--
    "Platform",
    # -- 机动件（§5.7）--
    "MOVER_PERIOD_US",
    "AirMover",
    "GroundMover",
    "MissileMover",
    "Mover",
    "SubsurfaceMover",
    "WaterMover",
    # -- 制导件（§5.8）--
    "FRAMEWORK_GUIDANCE",
    "GuidanceComputer",
    # -- 感知件（§5.12）--
    "FRAMEWORK_SENSORS",
    "HexSearchRadar",
    "RadarSensor",
    # -- 平台级航迹处理件（§5.13）--
    "FRAMEWORK_TRACKS",
    "TRACK_PERIOD_US",
    "TrackManager",
    # -- 干扰机（施扰侧，§5.14）--
    "FRAMEWORK_JAMMERS",
    "Jammer",
    "RfJammer",
    # -- 通信件（节点侧，§5.15）--
    "FRAMEWORK_COMMS",
    "CommNode",
    "RfComm",
    # -- 侧挂控制器与编队私有状态（§4.5.3①、§3.9.5） --
    "CONTROLLER_LABELS",
    "CONTROLLER_LEVELS",
    "LEVEL_ENTITY",
    "LEVEL_STRATEGIC",
    "LEVEL_UNIT",
    "NO_HOST",
    "Controller",
    "HostCheckFn",
    "UnitState",
]
