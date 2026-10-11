"""毁伤组件族：弹侧的战斗部与命中裁决（§5.17）。

族约定与其它族一致：基类不注册，参考实现一个（``base.Warhead``）。
"""

from __future__ import annotations

from .base import WARHEAD_FACTORS, WARHEAD_KINDS, Warhead, hit_probability
from .prob_table import ProbTable, load_default

#: 框架自带的战斗部参考实现。想定里 ``component damage WARHEAD`` 直接用。
FRAMEWORK_DAMAGE: tuple[type, ...] = (Warhead,)

__all__ = [
    "FRAMEWORK_DAMAGE",
    "ProbTable",
    "WARHEAD_FACTORS",
    "WARHEAD_KINDS",
    "Warhead",
    "hit_probability",
    "load_default",
]
