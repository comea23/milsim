"""制导件包：一个基类 + 三个参考实现，**各自一个文件**（§5.8）。

.. code-block:: text

    models/guidance/
    ├── __init__.py    本文件：只做导出，不含任何实现
    ├── contract.py    两个部件之间的词汇：阶段表 / 飞行状态 / 一帧指令
    ├── base.py        GuidanceComputer  基类 + 导引律
    ├── cruise.py      CruiseGuidance    CRUISE_GUIDANCE     巡航剧本
    ├── asm.py         AsmGuidance       ASM_GUIDANCE        跃升俯冲剧本
    └── ballistic.py   BallisticGuidance BALLISTIC_GUIDANCE  弹道剧本

组织约定与 :mod:`milsim.models.mover` 一样：**清单只有一份**（下面那个
元组），:data:`milsim.models.FRAMEWORK_COMPONENTS` 直接展开它，不另抄一遍。

**为什么是三个类、而不是每种弹一个**：轨迹形态的差异在 ``phases`` 这张表
里，不在算法里。"换一种弹"= 换一张表（或者换派生类里的一个默认值），而
不是换一个类。分成三个，是因为它们连**律的用法**都不一样：巡航弹全程制导、
反舰弹末段还要靠"看目标的角度"切段（要 ``target_elevation``）、弹道弹中段
干脆把关掉制导。**表与弹体是配套的**——反舰那张表要弹体拉得出 4.6 g，
换表不看弹体的话会得到一条"从目标头顶 750 m 掠过"的假航迹（``asm.py``）。
"""

from __future__ import annotations

from .ballistic import BALLISTIC_PHASES, BallisticGuidance
from .base import REASON_HIT, REASON_TIMEOUT, GuidanceComputer, wrap180
from .contract import (
    HORIZ_HOLD,
    HORIZ_PN,
    HORIZ_TARGET,
    HORIZONTAL_LAWS,
    SPEED_EPS,
    VERT_ALTITUDE,
    VERT_BALANCE,
    VERT_FPA,
    VERT_FREE,
    VERT_PN,
    VERTICAL_LAWS,
    FlightState,
    GuidanceCommand,
    Phase,
    Until,
)
from .asm import ASM_PHASES, AsmGuidance
from .cruise import CRUISE_PHASES, CruiseGuidance

#: 框架自带的制导件。想定里可以直接写这些注册名，不需要额外声明。
#: **加组件只改这一处**（外加新组件自己的那个文件）。
FRAMEWORK_GUIDANCE: tuple[type, ...] = (
    AsmGuidance,
    BallisticGuidance,
    CruiseGuidance,
)

__all__ = [
    # -- 词汇（弹体也要用）--
    "HORIZ_HOLD",
    "HORIZ_PN",
    "HORIZ_TARGET",
    "HORIZONTAL_LAWS",
    "SPEED_EPS",
    "VERT_ALTITUDE",
    "VERT_BALANCE",
    "VERT_FPA",
    "VERT_FREE",
    "VERT_PN",
    "VERTICAL_LAWS",
    "FlightState",
    "GuidanceCommand",
    "Phase",
    "Until",
    # -- 基类与工具 --
    "REASON_HIT",
    "REASON_TIMEOUT",
    "GuidanceComputer",
    "wrap180",
    # -- 参考实现 --
    "ASM_PHASES",
    "AsmGuidance",
    "BALLISTIC_PHASES",
    "BallisticGuidance",
    "CRUISE_PHASES",
    "CruiseGuidance",
    # -- 清单 --
    "FRAMEWORK_GUIDANCE",
]
