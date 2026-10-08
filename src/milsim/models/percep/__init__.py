"""感知件族：雷达探测链（第 1~8 步）+ 平台级航迹管理。

一族一个子包，一个实现一个文件（§5.7）。两个注册名：

* ``HEX_SEARCH_RADAR`` —— :class:`~milsim.models.percep.search_radar.HexSearchRadar`，
  探测链八步的落点；
* ``TRACK_MANAGER`` —— :class:`~milsim.models.percep.track_manager.TrackManager`，
  平台级的航迹汇聚 / 清理 / 转发。★ 它**不是传感器**（不判"能不能发现"），
  所以清单也分成两条 —— 混在 ``FRAMEWORK_SENSORS`` 里的话，"感知件有几种"
  这个问题就没有答案了。

本子包对外只暴露：**实现类**与**两份清单**。``models/__init__.py`` 只负责把
清单展开进 ``FRAMEWORK_COMPONENTS``，不在那里重列一遍类名——重列的代价是
"加了组件、想定里写了名字，却报没有实现类"，而报错信息看不出是漏登记还是
拼错名。
"""

from __future__ import annotations

from .base import (
    DEFAULT_REFERENCE_RCS,
    DEFAULT_REQUIRED_PD,
    RadarSensor,
)
from .search_radar import HexSearchRadar
from .track_manager import TRACK_PERIOD_US, TrackManager

#: 框架自带的感知件。想定里可以直接写这些注册名，不需要额外声明。
#: **加组件只改这一处**（外加新组件自己的那个文件）。
FRAMEWORK_SENSORS: tuple[type, ...] = (HexSearchRadar,)

#: 框架自带的**平台级航迹处理件**。与传感器分开列：它不是传感器，
#: 不挂 ``sensor`` 槽，不该出现在"雷达有几种"的清点里。
FRAMEWORK_TRACKS: tuple[type, ...] = (TrackManager,)

__all__ = [
    # -- 基类与默认值 --
    "DEFAULT_REFERENCE_RCS",
    "DEFAULT_REQUIRED_PD",
    "RadarSensor",
    # -- 参考实现 --
    "HexSearchRadar",
    "TrackManager",
    "TRACK_PERIOD_US",
    # -- 清单 --
    "FRAMEWORK_SENSORS",
    "FRAMEWORK_TRACKS",
]
