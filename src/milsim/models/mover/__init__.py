"""机动件包：一个基类 + 六个参考实现，**各自一个文件**（§5.7、§5.8）。

.. code-block:: text

    models/mover/
    ├── __init__.py      本文件：只做导出，不含任何实现
    ├── base.py          Mover（抽象基类 + 三维质点运动学 + 航路点跟随）
    ├── ground.py        GroundMover     GROUND_MOVER
    ├── water.py         WaterMover      WATER_MOVER
    ├── subsurface.py    SubsurfaceMover SUBSURFACE_MOVER
    ├── air.py           AirMover        AIR_MOVER
    ├── missile.py       MissileMover    MISSILE_MOVER（按指令积分，跟踪航路点）
    └── analytic.py      AnalyticBallisticMover
                         ANALYTIC_BALLISTIC_MOVER（球面弹道解算，逐帧置位）

两条组织约定，都是"以后加第五个件时照抄就行"的那种：

1. **每个组件一个文件，文件只依赖 ``base``。** 参考实现之间**不互相继承**。
   曾经有过 ``WaterMover(GroundMover)``，但两个高度钩子都被它覆盖了、
   一分复用都没有——那条继承只剩一个效果：改地面件会牵动水面件。共用的
   部分本来就在基类里，横着的继承链只会把两个无关的组件绑在一起。
   使用者自己的型号仍然**应该**继承参考实现（``MyShip(WaterMover)``），
   那是纵向扩展、不是横向耦合。
2. **清单只有一份。** 下面这个元组是本包全部组件的唯一出处，
   :data:`milsim.models.FRAMEWORK_COMPONENTS` 直接展开它，不另抄一遍
   ——两份清单漏同步的症状是"想定里写了名字却报没有实现类"。

导入本包即完成注册（装饰器在模块导入时执行）。
"""

from __future__ import annotations

from .air import AirMover
from .analytic import AnalyticBallisticMover
from .base import (
    BLOCKABLE_TERRAIN,
    MARCH_STEPS_PER_CELL,
    MOVER_PERIOD_US,
    STEER_BEFORE_MOVE_DEG,
    Mover,
    signed_angle,
)
from .ground import GroundMover
from .missile import MissileMover
from .subsurface import SubsurfaceMover
from .water import WaterMover

#: 框架自带的机动件。想定里可以直接写这些注册名，不需要额外声明。
#: **加组件只改这一处**（外加新组件自己的那个文件）。
FRAMEWORK_MOVERS: tuple[type, ...] = (
    AirMover,
    AnalyticBallisticMover,
    GroundMover,
    MissileMover,
    SubsurfaceMover,
    WaterMover,
)

__all__ = [
    # -- 基类与工具 --
    "BLOCKABLE_TERRAIN",
    "MARCH_STEPS_PER_CELL",
    "MOVER_PERIOD_US",
    "STEER_BEFORE_MOVE_DEG",
    "Mover",
    "signed_angle",
    # -- 参考实现 --
    "AirMover",
    "AnalyticBallisticMover",
    "GroundMover",
    "MissileMover",
    "SubsurfaceMover",
    "WaterMover",
    # -- 清单 --
    "FRAMEWORK_MOVERS",
]
