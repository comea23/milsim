"""部件基类：模型层的最小契约。

三层组件体系里这是最上面那层（§5.1）：

.. code-block:: text

    使用者扩展    MyRadar(PhasedArrayRadar)        写自己的算法
                        ↑ 继承
    参考实现      PhasedArrayRadar(HexSearchRadar)  可直接用、能跑通
                        ↑ 继承
    抽象基类      Component                        只定义契约（本文件）

契约只有四条，刻意不多加：

| 成员 | 谁实现 | 稳定性 |
|---|---|---|
| ``PARAMS`` | 每个组件 | **契约**——键名不能删改 |
| ``initialize(mount)`` | 需要接线的组件 | 契约 |
| ``update(...)`` | 有周期节拍的组件 | 契约 |
| ``shutdown()`` | 需要释放资源的组件 | 契约 |

四个都是**可选**的：只转发消息的通信组件可能一个都不需要覆盖。

关于 ``__slots__``
------------------
基类带 ``__slots__``，子类**不强制**——不写 ``__slots__`` 的子类会有
``__dict__``，可以随手加属性。这是刻意的：使用者扩展时不该先研究槽位。
框架自己的参考实现会写 ``__slots__``，省内存也顺便暴露拼错的属性名。
"""

from __future__ import annotations

from typing import Any, Mapping

from ..services.params import ParamSpec


class Component:
    """部件基类。

    用法（参考实现）::

        @register_component("HEX_SEARCH_RADAR")
        class HexSearchRadar(Component):
            PARAMS = {
                "detect_range":  Params.distance(40_000.0),
                "scan_interval": Params.duration(2_000_000),
            }

            def initialize(self, mount):
                self._view = mount.sensor_view()
                mount.every(self.spec["scan_interval"], self._scan, PRIORITY_SENSOR)

            def _scan(self, engine, event):
                ...
                return EventResult.RESCHEDULE

    使用者继承它时，``PARAMS`` 用 ``**父类.PARAMS`` 显式合并：

    .. code-block:: python

        class MyRadar(HexSearchRadar):
            PARAMS = {**HexSearchRadar.PARAMS, "eccm_level": Params.integer(0)}
    """

    #: 参数声明。键名是**契约**——改名等于破坏所有历史想定的兼容性。
    #: 新增带默认值的参数不算破坏，这是扩展的唯一安全方式。
    PARAMS: Mapping[str, ParamSpec] = {}

    #: 本部件的**运行期指令词汇表**（v0.13.58）：``{指令名: 一句话说明}``。
    #:
    #: 指令体系的分工（§5.18）：指令下到**实体**（``Entity.command``），实体
    #: 按挂载顺序找第一个声明了这条指令的部件执行——调用方**不寻址槽位**，
    #: 只说"这个实体，fire"。"哪个部件接"是实体内部的分工。
    #: 同一实体上两个部件声明同名指令：**先挂的接**（挂载顺序确定，可复现）。
    #: 空表 = 本部件没有任何运行期指令（纯被动计算，如雷达、战斗部）。
    COMMANDS: Mapping[str, str] = {}

    #: 本部件**认领的任务类型**（v0.13.59，§5.19）：``("attack", "move", …)``。
    #:
    #: 任务层（§5.19）把收到的任务派给同实体上第一个认领该类型的部件；
    #: 认领了就要实现下面的**执行器协议**三个方法。空元组 = 不参与任务层
    #: （默认，历史部件零改动）。
    TASKS: tuple[str, ...] = ()

    #: 这个部件通常挂在哪个槽。**只是文档**，装配不靠它判断——
    #: 槽名由想定里的 ``component <slot> <type>`` 决定。
    SLOT_HINT = ""

    __slots__ = ("spec", "entity", "enabled")

    def __init__(self, *, spec: Any) -> None:
        #: 装配产出的完整描述（含默认值）。组件自己不必再查表、判空。
        self.spec = spec
        #: 宿主实体。由 :meth:`bind` 注入，``initialize`` 之前为 None。
        self.entity: Any = None
        #: 关掉后引擎仍然会叫它，但 :meth:`update` 默认直接返回。
        #: 用于"雷达被击伤后关机"这类状态，比反复注销/重注册事件简单。
        self.enabled = True

    # -- 身份 --------------------------------------------------------------

    @property
    def entity_id(self) -> int:
        """宿主实体 ID。未绑定返回 -1——不抛异常，调试时更方便。"""
        return self.entity.entity_id if self.entity is not None else -1

    @property
    def slot(self) -> str:
        return getattr(self.spec, "slot", "")

    @property
    def type_name(self) -> str:
        return getattr(self.spec, "type_name", "")

    def param(self, name: str, default: Any = None) -> Any:
        """读一个参数。``spec`` 是映射，所以 ``self.spec["x"]`` 也行。"""
        try:
            return self.spec[name]
        except KeyError:
            return default

    # -- 生命周期（都可选覆盖） --------------------------------------------

    def bind(self, entity: Any) -> None:
        """挂到宿主实体。装配层调用，**不要覆盖**。"""
        self.entity = entity

    def initialize(self, mount: Any) -> None:
        """接线：拿视图、注册周期事件。

        这是唯一能拿到 ``mount.engine`` 的地方——运行期不允许改事件队列。
        """

    def update(self, ctx: Any = None) -> None:
        """一次更新。有周期节拍的组件覆盖它。

        注意这**不是**引擎直接调的：引擎调的是组件自己注册的事件回调，
        回调里再决定要不要走 update（可能要先判 ``enabled``）。
        """

    def shutdown(self) -> None:
        """释放资源。有线程池、文件句柄之类的组件覆盖它。"""

    # -- 运行期指令（v0.13.58，§5.18） --------------------------------------

    def execute(self, cmd: str, args: Mapping[str, Any]) -> tuple[bool, str]:
        """执行一条运行期指令。返回 ``(是否成功, 给人看的原因)``。

        基类**只认识自己 ``COMMANDS`` 里声明过的**：没声明的一律拒绝——
        "不接受指令"与"指令失败"要分得清，前者说明调用方找错了对象。
        有指令的部件覆盖本方法，但**先查一遍父类**（保持同一道闸）：

        .. code-block:: python

            def execute(self, cmd, args):
                ok, why = super().execute(cmd, args)
                if not ok and cmd in self.COMMANDS:
                    ...  # 真正的执行
                return ok, why
        """
        if cmd not in self.COMMANDS:
            return False, (
                f"{self.describe()} 不接受指令 {cmd!r}"
                + (f"（可接受：{'、'.join(self.COMMANDS)}）" if self.COMMANDS else "")
            )
        return False, f"{self.describe()} 的指令 {cmd!r} 没有实现"

    # -- 任务执行器协议（v0.13.59，§5.19） ----------------------------------
    #
    # 认领了 TASKS 的部件按需覆盖这三个方法；基类给出"不接"的默认实现，
    # 让任务管理器的派发链得到一致的拒绝理由而不是 AttributeError。

    def acquire_task(self, task: Any) -> tuple[bool, str]:
        """接一项任务。返回 ``(是否接下, 给人看的原因)``。

        接下 = 开干（或登记在案）；拒绝 = 本部件干不了，任务管理器会
        按挂载顺序问下一个认领同类型的部件。**不要在这里抛异常**——
        与 :meth:`execute` 同一条纪律，失败是回执不是事故。
        """
        return False, f"{self.describe()} 不认领 {getattr(task, 'task_type', '?')} 任务"

    def poll_task(self, task: Any) -> tuple[str, str] | None:
        """查一项在身任务的进展。返回 ``(状态, 子状态)`` 或 ``None``（无变化）。

        只对**还挂在自己名下**的任务负责：任务已被取消/移交时返回
        ``None``。由任务管理器的台账节拍来问（§5.19），部件自己不用
        记得汇报——轮询比回调少一条"忘了接"的静默故障路径。
        """
        return None

    def release_task(self, task: Any) -> None:
        """交还一项任务（取消/收编）。尽力而为，不报错。"""

    # -- 展示 --------------------------------------------------------------

    def describe(self) -> str:
        return f"{self.type_name or type(self).__name__}({self.slot or '?'})"

    def __repr__(self) -> str:
        state = "" if self.enabled else " 已关闭"
        return f"<{type(self).__name__} {self.describe()} eid={self.entity_id}{state}>"


__all__ = ["Component"]
