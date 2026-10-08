"""装配期上下文：部件在 ``initialize()`` 时拿到的接线。

为什么要有这一层
----------------
部件的构造签名保持干净（``__init__(self, *, spec)``），服务引用不在构造时传。
装配第 7 步才注入——这样单元测试里造一个部件不需要先造引擎、地图、随机池。

为什么是一个对象而不是逐个参数
------------------------------
部件在 ``initialize()`` 里要做的事很杂：注册周期事件、拿自己的视图、取随机流。
拆成七八个参数的话，每个部件的签名都不一样，而且加一个服务就要改所有部件。

代价是"这个部件到底用了哪些服务"不再一眼可见。缓解办法是**约定只在这里
读一次**：``initialize()`` 里把需要的东西存成实例字段，``update()`` 里只用
自己的字段，不再回头找 mount。这样依赖面在 initialize 里就固定下来了。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from ..services.registry import EntityRegistry
from ..services.store import (
    CommView,
    EngagementView,
    EntityStore,
    MoverView,
    SensorView,
    TrackView,
)


@dataclass(frozen=True, slots=True)
class MountContext:
    """一个实体在装配期能看到的全部。

    与运行期的区别：这里能拿到 ``engine``（注册自己的节拍），运行期不能——
    部件不该在推演中途改事件队列。
    """

    engine: Any
    store: EntityStore
    registry: EntityRegistry
    maps: Any
    random: Any
    bus: Any
    entity_id: int
    entity: Any = None
    #: 局部坐标 → 格归属的转换器，由装配层注入。
    #: 机动组件拿到它才能"位置一变，空间索引跟着变"。
    locator: Callable[[float, float, float], Any] | None = None
    #: 导航门面（:class:`~milsim.services.map.nav.NavService`）。
    #: 寻路、格↔坐标、通行快慢都从它走——**组件不该 import 地图服务的
    #: 内部结构**（战区 / 分块 / 通道）。战区外或纯单元测试里可以是 None。
    nav: Any = None
    #: 电子战门面（:class:`~milsim.services.ew.JamService`）。
    #: **施扰侧**（干扰机）往它登记自己的能力，**受害侧**（雷达 / 通信接收机）
    #: 在"要检测 / 要收信"的那一刻向它问"谁在压我"（§5.14）。
    #: 战区外或纯单元测试里可以是 None——那时**干扰机**在 ``initialize`` 里
    #: 会当场报错（登记不上去的干扰机等于不存在，而"等于不存在"正是不许静默
    #: 退化的那类）；受害侧则按"没有门面 ⇒ 没有干扰"处理。
    jam: Any = None
    #: 通信网门面（:class:`~milsim.services.comm.CommService`，§5.15）。
    #: **通信件**往它登记"我在哪个网"，发信方在"要发信"的那一刻向它问
    #: "从 A 到 B 现在通不通"。与 ``jam`` 一样，可以是 None——那时**通信件**
    #: 在 ``initialize`` 里会当场报错（登记不上去的通信件等于哑掉，而"哑掉"
    #: 看起来像被干扰，正是最不该静默发生的那种）。
    comm: Any = None
    #: 这个实体的平台属性（§5.6），装配期已解析并校验过。
    platform_params: Mapping[str, Any] = field(default_factory=dict)

    # -- 时间 --------------------------------------------------------------

    @property
    def now(self) -> int:
        """当前仿真时刻（微秒）。装配期通常是 0。"""
        return self.engine.now

    # -- 注册自己的节拍 ----------------------------------------------------

    def every(
        self,
        interval_us: int,
        fn: Callable[[Any, Any], Any],
        priority: int = 0,
    ) -> Any:
        """按固定周期调用 ``fn(engine, event)``，返回 ``RESCHEDULE`` 即自动重排。

        这就是"模型自定周期"的落点：部件在自己的 ``initialize()`` 里声明
        自己的更新间隔，引擎负责按时叫它。不同部件可以有完全不同的节拍——
        雷达可能 0.5 s 扫一次，后勤 60 s 算一次，互不牵连。
        """
        return self.engine.schedule_recurring(0, interval_us, fn, priority)

    def after(
        self,
        delay_us: int,
        fn: Callable[[Any, Any], Any],
        priority: int = 0,
    ) -> Any:
        """延迟一次。用于"3 秒后再检查一次"这类一次性动作。"""
        return self.engine.schedule_recurring(delay_us, 1, fn, priority)

    def once(self, delay_us: int, fn: Callable[[Any], None], priority: int = 0) -> Any:
        """延迟执行一次，回调签名 ``fn(engine)``。"""
        return self.engine.schedule(delay_us, fn, priority)

    # -- 收窄视图 ----------------------------------------------------------

    def sensor_view(self) -> SensorView:
        return self.store.sensor_view(self.entity_id)

    def mover_view(self, locator: Callable | None = None) -> MoverView:
        """机动视图。不传 ``locator`` 时用装配层注入的那个。

        ``locator`` 由 Simulation 提供而不是让组件自己算：换算要用到战区
        投影坐标系，而组件不该知道地图服务的存在（§5 的窄接口原则）。
        """
        return self.store.mover_view(
            self.entity_id, locator if locator is not None else self.locator
        )

    def engagement_view(self) -> EngagementView:
        return self.store.engagement_view(self.entity_id)

    def comm_view(self) -> CommView:
        return self.store.comm_view(self.entity_id)

    def track_view(self) -> TrackView:
        """平台级航迹管理器的视图（§5.13）。

        比 :meth:`sensor_view` 多一个"取信"与一个"把自己的航迹发出去"，
        **少** 一切真实位置——航迹管理器不判"能不能发现"，只做汇聚与清理。
        """
        return self.store.track_view(self.entity_id)

    # -- 其它 --------------------------------------------------------------

    def jam_service(self) -> Any:
        """电子战门面（§5.14）。**可能返回 None**。

        做成方法而不是让组件直接读 ``mount.jam``：门面是装配层注入的，
        将来换成"多张名册按战区分开"时，只有这一个函数要改。
        调用方要能分得清 ``None``（这份装配根本没有电子战）与
        "门面在、但名册是空的"——前者是装配缺陷，后者是"这场没有干扰"。
        """
        return self.jam

    def comm_service(self) -> Any:
        """通信网门面（§5.15）。**可能返回 None**。

        与 :meth:`jam_service` 同一条理由：做成方法而不是让组件直接读
        ``mount.comm``，将来换成"每个战区一张通信网名册"时只改这一个函数。
        调用方要能分清 ``None``（这份装配根本没有通信）与"门面在、但没建网"
        ——前者是装配缺陷，后者是"这场没有通信约束"。
        """
        return self.comm

    def cell_span_m(self) -> float:
        """当前所在一层的**相邻格中心距**（米）。

        组件拿到的是米（``detect_range 30 km``），而空间索引按格数搜——
        这个换算需要一个知道战区投影尺寸的角色。让装配层算好交出来，
        组件就不必认识地图服务。

        注意是 ``边长 × √3`` 而不是边长——pointy-top 布局下相邻格中心
        间距比边长长 √3 倍，这两个数搞混会让所有距离判定偏差 1.73 倍。

        战区外（全球层）返回 0.0，调用方应据此判断"这里没有逐格精度"。
        """
        from math import sqrt

        ref = self.store.cell_of(self.entity_id)
        if ref is None or not hasattr(ref, "zone_id"):     # GlobalCellRef
            return 0.0
        zone = self.maps.zone(ref.zone_id)
        if zone is None:
            return 0.0
        return zone.spec.resolution_of_layer(ref.layer) * sqrt(3.0)

    def streams(self) -> Any:
        """这个实体的确定性随机流组。按 (实体, 用途) 分流，保证可复现。"""
        return self.random.entity(self.entity_id)

    def platform_param(self, name: str, default: Any = None) -> Any:
        """读一个平台属性（§5.6）。装配期已解析、已校验，这里只是取值。

        读不到返回 ``default`` 而不是抛异常：平台属性表会随里程碑增长
        （新增带默认值的参数不算破坏性改动），组件不该因为"这型平台没写
        这个属性"就崩掉。
        """
        return self.platform_params.get(name, default)

    def target_platform_param(
        self, entity_id: int, name: str, default: Any = None
    ) -> Any:
        """读**别人**的平台属性（§5.6）。

        感知层要它拿目标的 ``radar_cross_section``：雷达方程里的 σ 是**目标**
        的属性，不是雷达的，所以给目标挂一个 RCS 参数而已，不该逼每台雷达
        自己维护一张"谁有多大反射面积"的表。

        **只读属性，不读位置。** 位置一律走
        :class:`~milsim.services.store.SensorView`——那里有"能不能看真实位置"
        这条分层约束（真实位置是上帝视角的原始量），这里没有，所以不开口子。
        """
        entity = self.registry.get(entity_id)
        if entity is None:
            return default
        params = getattr(entity, "platform_params", None)
        if not params:
            return default
        return params.get(name, default)

    def side_of(self, entity_id: int) -> str:
        """某个实体属于哪一方（空串 = 没声明）。

        给感知层判**敌我**用（v0.13.33 的 IFF 简化版）。

        ★ 为什么这是"想定给定的敌我态势"而不是上帝视角：真实的敌我识别
        是一个**独立的询问/应答系统**，AFSIM 也把它简化成想定里的映射表
        （``iff_mapping`` 块，``WsfIFF_Manager::GetIFF_Status`` —— 查三张表
        (方,方)/(方,类别)/(方,默认)，都没查到就"同方为友、否则为敌"）。
        我们照这个口径**取简版**：同方 FRIEND、异方 FOE、查不到 UNKNOWN。
        雷达并没有"探测"出敌我，是态势库里本来就写着"蓝方对我方是敌"。

        ★ 与 ``target_platform_param`` 一样**只读属性**：不在这里开位置
        的口子（位置一律走 ``SensorView``）。
        """
        return self.registry.side_of(entity_id)

    def my_position(self) -> tuple[float, float, float] | None:
        return self.store.position_of(self.entity_id)

    def my_cell(self) -> Any:
        return self.store.cell_of(self.entity_id)

    def __repr__(self) -> str:
        name = getattr(self.entity, "name", "?")
        return f"<MountContext {name}(id={self.entity_id}) @{self.now}>"


__all__ = ["MountContext"]
