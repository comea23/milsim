"""milsim —— 任务级军事仿真器。

分层（依赖严格单向：models → services → engine）：

    engine/     引擎层：整数微秒时间、事件、事件队列、混合步进、线程投递
    decision/   决策抽象层：规则 / 脚本 / 大模型（异步 + 降级）
    services/   基础服务层：地图、空间索引、注册表、状态存储、类型系统、
                想定语言、事件总线、确定性随机
    models/     模型层：实体、部件、装配上下文（具体模型属 M4）
    simulation.py  装配点：把以上三层接起来跑

典型用法::

    from milsim.simulation import Simulation

    sim = Simulation.from_scenario_file("scenarios/patrol.txt")
    sim.initialize()
    sim.run_for(60_000_000)
    print(sim.shutdown().summary())

设计要点见 docs/00-框架设计.md。
"""

from .errors import ConfigurationError, MilsimError

__version__ = "0.0.1"

__all__ = ["ConfigurationError", "MilsimError", "__version__"]
