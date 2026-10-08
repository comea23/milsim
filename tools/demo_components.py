"""**兼容壳**：演示雷达已搬进框架，本文件只转发。

v0.13.25 起，雷达的扫描律 + 探测链（第 1~8 步）在
:mod:`milsim.models.percep`——它不再是"让装配链路跑得起来"的演示件，而是
能直接用在想定里的参考实现（与机动件在 v0.13.15 经历的那次搬迁同一条路）。

保留下来的理由有两个，都不是"顺手":

* ``tests/test_demo_components.py`` 按**文件路径**加载本文件来测扫描律
  （``tools/`` 不在包的搜索路径上），搬走实现而不留转发会让那 20 条用例
  一起失效；
* ``tools/run_scenario.py`` 用它把独立注册表装满。

真正的实现与全部文档都在 :class:`~milsim.models.percep.search_radar.HexSearchRadar`。
**新代码请直接 import 框架里的那个类**，不要再往这里加东西——一处实现两处
入口正是本项目反复消灭的那类重复。
"""

from __future__ import annotations

from milsim.models.percep import HexSearchRadar

__all__ = ["HexSearchRadar", "register_all"]


def register_all(registry) -> None:
    """把感知件登记到指定注册表。

    装饰器在模块导入时已经把类登记进**全局**表了；这个函数是给"想用独立
    注册表跑"的场合用的——比如跑想定时不希望被全局状态影响。
    """
    for cls in (HexSearchRadar,):
        registry.register(cls.COMPONENT_NAME, cls, override=True)
