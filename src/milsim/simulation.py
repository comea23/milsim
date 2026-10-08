"""仿真器：把三层装起来跑的那一层。

定位
----
| 层 | 一句话职责 | 知道什么 |
|---|---|---|
| ``Engine`` | 事件在正确的时刻发生 | 只有时间和事件 |
| ``services`` | 世界有哪些资源可查 | 地图、索引、存储、随机 |
| ``models`` | 实体怎么决策怎么动 | 自己的行为 |
| **``Simulation``** | **把它们接起来跑** | 全知道——因为接线是它的活儿 |

这也是为什么 ``Engine`` 里不塞注册表：一旦引擎开始管实体，它就没法脱离
军事仿真复用了（拿去做离散事件调度器都不行）。装配的活儿集中在这里一处。

装配七步（§3.8.8）
------------------
1. 解析全文 → 节点树 —— 已由 ``load_scenario`` 完成
2. 建 ``TypeRegistry`` —— 同上
3. 建 ``MapService``，**声明**战区（不激活，不占资源）
4. 记录平台声明（位置暂存，不定位）
5. **只激活有实体落入的战区** ← 按需加载的落点
6. 实例化实体（此时才能算网格坐标）
7. 注入服务引用（``MountContext``）

第 5 步是跨洲想定能跑起来的前提：声明了 20 个战区但只有 3 个有实体，
就只加载那 3 个。全球战区全加载是不现实的。

大模型在哪接上
--------------
装配期才构造客户端——**解析阶段不碰网络**（§3.8.2）。这里通过
``llm_client_factory(profile)`` 把"用哪个模型、密钥从哪个环境变量读"
变成真实的客户端，工厂由调用方提供（这样 services 层不需要 HTTP 依赖，
测试也能塞一个假的）。

**没有客户端的 llm 配置会直接报错**，不会静默退化成规则——想定写了
"这里要用大模型"却被悄悄换掉，是最难发现的一类问题。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterator

from .errors import ConfigurationError, MilsimError
from .decision.base import DecisionRequest
from .decision.llm import LLMDecisionProvider
from .decision.rule import RuleDecisionProvider
from .engine import Engine
from .models.entity import Entity
from .models.mount import MountContext
from .services.bus import EventBus
from .services.command import CommandChain
from .services.comm import CommNet, CommService
from .services.ew import JamService
from .services.formation import FormationTree
from .services.situation import SituationProjector
from .services.map import (
    DEFAULT_RESOLUTION,
    MapService,
    NavService,
    Zone,
    ZoneSpec,
    great_circle_m,
)
from .services.map.grid import (
    CHUNK_SIZE,
    LEVEL_SURFACE,
    HexGrid,
    level_of_altitude,
)
from .services.map.hex import Axial
from .services.map.terrain import generate_flat, generate_terrain
from .services.random import RandomPool
from .services.registry import EntityRegistry
from .services.scenario import (
    DecisionProfile,
    PlatformDecl,
    ScenarioError,
    ScenarioSpec,
    load_scenario,
)
from .services.store import (
    CellRef,
    EntityStore,
    LocalCellRef,
)
from .services.type_registry import (
    ComponentFactory,
    ComponentRegistry,
    PlatformRegistry,
)


class SimulationError(MilsimError, RuntimeError):
    """装配或驱动阶段的错误。"""


#: 没有提供规则策略时的兜底：选第一个选项。
#:
#: 刻意做得很笨——它是**骨架**，不是决策逻辑。真正的规则表由 M4/M5 提供。
#: 兜底策略静默"看起来很聪明"反而危险：会让人以为决策已经接上了。
def _default_rule_policy(request: DecisionRequest) -> str:
    return request.options[0] if request.options else "hold"


@dataclass(slots=True)
class SimulationReport:
    """一次推演的收尾统计。"""

    sim_time_us: int = 0
    steps: int = 0
    events_dispatched: int = 0
    fences: int = 0
    entities: int = 0
    alive: int = 0
    zones_active: int = 0
    decisions: dict[str, Any] = field(default_factory=dict)
    #: 这次跑用的参数库修订。空串表示没用参数库。
    #:
    #: 它进报告而不是只留在日志里，是因为**参数库是想定文件之外的输入**：
    #: 同一份想定、同一个种子，库里的一个值被改过，结果就不同。复盘时
    #: "哪一版参数"必须和"哪个想定"一样可查。
    library_revision: str = ""

    def summary(self) -> str:
        seconds = self.sim_time_us / 1_000_000.0
        library = f" / 参数库修订 {self.library_revision}" if self.library_revision else ""
        return (
            f"推演到 {seconds:.1f} s：{self.steps} 步 / "
            f"{self.events_dispatched} 个事件 / "
            f"{self.entities} 实体（存活 {self.alive}）/ "
            f"{self.zones_active} 个战区已加载"
            f"{library}"
        )


class Simulation:
    """一次推演的全部状态与装配逻辑。

    用法::

        sim = Simulation.from_scenario_file("scenarios/demo.txt")
        sim.initialize()
        sim.run_for(30 * MINUTE)
        print(sim.shutdown().summary())
    """

    def __init__(
        self,
        spec: ScenarioSpec,
        *,
        components: ComponentRegistry | None = None,
        platforms: PlatformRegistry | None = None,
        llm_client_factory: Callable[[DecisionProfile], Any] | None = None,
        rule_policy: Callable[[DecisionRequest], Any] | None = None,
        zone_loader: Callable[[Zone], None] | None = None,
        global_resolution: int = DEFAULT_RESOLUTION,
        seed: int | None = None,
    ) -> None:
        self.spec = spec
        self.components = components
        self._llm_client_factory = llm_client_factory
        self._rule_policy = rule_policy or _default_rule_policy
        # 没给 loader 就用**惰性地形生成**（§3.11.5）：战区激活只建坐标系，
        # 实体踩到哪才生成哪。给了 loader 的话，那个 loader 负责给战区装上
        # ``chunk_loader``（而不是一次性把所有地形生成出来）。
        self._zone_loader = (
            zone_loader if zone_loader is not None else self._build_terrain_loader()
        )
        self._global_resolution = global_resolution

        # -- 三层 --
        self.engine = Engine(max_step=spec.settings.max_step_us)
        self.registry = EntityRegistry()
        self.store = EntityStore()
        self.bus = EventBus()
        self.random = RandomPool(spec.settings.seed if seed is None else seed)
        self.maps = MapService(global_resolution, self._zone_loader)
        #: 导航门面（§5.7）。组件拿到的就是它——组件不该 import 地图服务的
        #: 内部结构（战区 / 分块 / 通道），那会让换一种存储就要改所有组件。
        self.nav = NavService(self.maps)
        #: 电子战门面（§5.14）。**一张全局名册**：干扰机在 ``initialize`` 里
        #: 登记、在 ``shutdown`` 里注销，受害接收机在检测那一刻来查。它不进
        #: 事件队列、不推时间——"谁在压我"是个**瞬时几何问题**，不是过程。
        #:
        #: ★ ``tracks_of``（v0.13.34，§5.14.11 ④）：门面要能读"**某实体自己
        #: 那张航迹表**"才能实现"照敌航迹置向"。注入的是 ``EntityStore`` 的
        #: ``contacts_of`` 这一个**只读切片**（不是整个 store）——门面能看什么
        #: 由这一句决定，将来要做"按战区隔离"也只改这里。没有航迹表的干扰机
        #: 写了 ``aim_at`` 就**不辐射**（"看见才打"，§9-59）。
        self.jam = JamService(self.store, self.registry, tracks_of=self.store.contacts_of)
        #: 通信网门面（§5.15）。**一张全局拓扑**：通信件在 ``initialize`` 里
        #: 登记"我在哪个网"，发信方在"要发信"的那一刻问它"从 A 到 B 现在通不通"。
        #: ★ 与电子战门面不同，它**有节拍**——"一个时间步内完成通信"是一句关于
        #: 时间的话，所以每拍要有人来提交拓扑、搬报文（通信件的 ``_tick``）。
        #:
        #: ★ ``store`` 注入进来**只为** ``deliver`` 搬报文；"通不通"的判据一个
        #: 字都不靠它（那是纯拓扑）。
        #:
        #: ★ 想定里没写 ``network`` 块 ⇒ 名册为空 ⇒ :meth:`CommService.path_exists`
        #: 直接返回"不限制"，旧想定的消息投递**逐字不变**（验收判据，不是优化）。
        #:
        #: ★ ``jam``（v0.13.38，第二步）：通信被压制要算 ``S/(N+J)``，而"谁在
        #: 压我"的答案与雷达问的是**同一张名册**——施扰端复用 ``RF_JAMMER``
        #: （用户裁定②A；AFSIM 的 ``WsfEW_CommComponent`` 也是调雷达那同一个
        #: 函数）。这里把电子战门面交给通信门面，于是"通信专用干扰机"这套
        #: 语法**根本不需要存在**。``None`` ⇒ 不判压制（零回归）。
        self.comm = CommService(self.store, self.registry, jam=self.jam)
        self.factory = ComponentFactory(components, spec.types, platforms)
        #: 编队树（行政隶属，第一层）。想定没写 formation 块时是空树。
        self.formations = FormationTree()
        #: 指挥关系（第二、三层）。装配时按想定声明构建。
        self.commands = CommandChain(self.formations)
        #: 态势投影器。**前端与各层智能体都走它**，不直读 EntityStore（§3.9）。
        self.projector = SituationProjector(
            self.registry,
            self.store,
            self.formations,
            clock=lambda: self.engine.now,
            chains=self.commands,
        )

        # -- 装配产出 --
        self.entities: dict[int, Entity] = {}
        #: 实体 ID → 解析后的平台属性（§5.6）。组件通过 ``mount.platform_param``
        #: 读它——比让每个组件自己 ``resolve_platform`` 少一次解析，也多一层
        #: "平台属性已经校验过"的保证。
        self.platform_params: dict[int, dict[str, Any]] = {}
        self.zone_ids: dict[str, int] = {}
        self.decisions: dict[str, Any] = {}
        self._built = False
        self._initialized = False

    # -- 便捷构造 ----------------------------------------------------------

    @classmethod
    def from_scenario(
        cls,
        text: str,
        *,
        name: str = "<想定>",
        library: Any = None,
        **kwargs: Any,
    ) -> "Simulation":
        """从想定文本建一次推演。

        ``library`` 是参数库（§5.5）。**必须显式给**：库文件在哪是一个
        不能从想定里推出来的事实，而"默认去某个固定路径找"会让同一份想定
        在不同机器上读到不同的参数，且不报错。
        """
        return cls(load_scenario(text, name=name, library=library), **kwargs)

    @classmethod
    def from_scenario_file(cls, path: str | Path, **kwargs: Any) -> "Simulation":
        file = Path(path)
        return cls.from_scenario(
            file.read_text(encoding="utf-8"), name=file.name, **kwargs
        )

    # -- 装配 --------------------------------------------------------------

    def build(self) -> "Simulation":
        """执行 §3.8.8 的第 3~7 步。可重复调用（幂等）。"""
        if self._built:
            return self

        self._declare_zones()          # 第 3 步
        # 第 4 步在 _warm_zones 里顺带完成——平台声明本来就在 spec 里
        self._warm_zones()             # 第 5 步
        self._build_command_chain()    # 编队与指挥关系（平台编入前必须先有树）
        self._create_entities()        # 第 6 步
        self._declare_networks()       # 通信网（实体建好才有 ID）
        self._wire_decisions()         # 决策接线
        self._built = True
        return self

    # -- 编队与指挥关系 ----------------------------------------------------

    def _build_command_chain(self) -> None:
        """按想定声明建编队树与指挥关系（§3.10）。

        必须在 ``_create_entities`` 之前——平台要编入编队，树得先在那儿。
        """
        if not self.spec.formations and not self.spec.attachments:
            return

        self.commands = self.spec.command_chain()
        self.formations = self.commands.formations

        # 投影器早先拿的是空树，整只换掉比改它的内部字段干净
        self.projector = SituationProjector(
            self.registry,
            self.store,
            self.formations,
            clock=lambda: self.engine.now,
            chains=self.commands,
        )

        problems = self.commands.validate()
        if problems:
            raise ScenarioError(
                "指挥关系自检未通过：\n  - " + "\n  - ".join(problems)
            )

    # -- 通信网（§5.15） ---------------------------------------------------

    def _declare_networks(self) -> None:
        """想定的 ``network`` 块 → 门面的图。**平台名在这里解析成实体 ID。**

        为什么放在 ``_create_entities`` 之后：实体 ID 是那一步才分配的。
        与编队归属同一条理由——名字是想定语言的东西，ID 是装配产物的东西。

        ★ **单边通信自动入网**（用户裁定，原话："如果想定中有一个实体单边向
        通信网中的某个实体通信的内容，则该实体也加入同一通信网"）：想定里谁
        的收发件人写的是网内成员、而自己不在任何网里 ⇒ 它**自动并入那个网**。
        这一步**在装配期做**（那时才拿得到全部实体的收发件人），结果是
        ``CommNet.auto_members``——与想定点名进来的 ``members`` 分开存，
        好让"谁是想定点名进来的、谁是发信蹭进来的"在复盘时看得见。

        ★ 一个实体**只能属于一个网**：想定把它列进两个网 ⇒ 当场报错。门面
        按实体建索引，"它算哪个网的"必须唯一（否则寻路结果随遍历顺序变，不可
        复现）。跨网通信是**网关**的事，属后续，不是"多挂几个网"。
        """
        if not self.spec.networks:
            return

        entity_of = self._entity_by_decl_name()
        #: 实体 ID → 网名。**谁是点名进来的**先建，再叠自动入网的。
        named: dict[int, str] = {}
        for decl in self.spec.networks:
            for member in decl.members:
                entity_id = entity_of.get(member)
                if entity_id is None:
                    raise ScenarioError(
                        f"通信网 {decl.name} 的成员 {member!r} 不在想定里"
                        f"（检查网络块第 {decl.line} 行：成员写的是**平台实例名**）",
                        decl.line,
                    )
                old = named.get(entity_id)
                if old is not None and old != decl.name:
                    raise ScenarioError(
                        f"平台 {member!r} 同时被列进通信网 {old!r} 与 {decl.name!r}"
                        "——一个节点只能属于一个网（跨网通信要走网关，属后续）",
                        decl.line,
                    )
                named[entity_id] = decl.name

        #: 自动入网：谁给网内成员发过信、而自己不在网里 ⇒ 并进那个网。
        auto = self._auto_join_networks(named)

        for decl in self.spec.networks:
            self.comm.declare(
                CommNet(
                    name=decl.name,
                    members=tuple(
                        entity_of[m] for m in decl.members
                    ),
                    links=tuple(
                        (entity_of[a], entity_of[b]) for a, b in decl.links
                    ),
                    auto_members=tuple(
                        sorted(e for e, n in auto.items() if n == decl.name)
                    ),
                )
            )

        problems: list[str] = []
        for decl in self.spec.networks:
            for a, b in decl.links:
                for endpoint in (a, b):
                    if endpoint not in decl.members:
                        problems.append(
                            f"通信网 {decl.name} 的 link 端点 {endpoint!r} "
                            "不是本网成员（link 只在网内连边）"
                        )
        if problems:
            raise ScenarioError("通信网自检未通过：\n  - " + "\n  - ".join(problems))

    def _entity_by_decl_name(self) -> dict[str, int]:
        """平台实例名 → 实体 ID。

        ★ 用**声明名**而不是 ``registry.by_name``：装配期两种查法结果一样，但
        声明名是"想定里写的那个字符串"，而 ``by_name`` 走的是注册表的规范名
        （将来加了别名之类的会出现差异，那时这条路径会静默错位）。
        """
        out: dict[str, int] = {}
        for entity_id, entity in self.entities.items():
            name = getattr(entity, "name", "")
            if name:
                out[name] = entity_id
        return out

    def _auto_join_networks(self, named: dict[int, str]) -> dict[int, str]:
        """单边通信自动入网（§5.15 裁定③）。返回 ``实体 ID → 网名``。

        判据：某个实体**不在任何网里**，但它某台通信设备的收件人名里
        （:class:`~milsim.models.percep.track_manager.TrackManager` 的
        ``share_recipients``，或组件上写的 ``comm_recipients``）有网内成员
        ⇒ 它并入那个成员的网。挑不到唯一答案时**按网名排序取第一个**——
        可复现优先于"更好的猜测"。

        ★ 本版只从**现有想定语法里唯一的"发给谁"出口**取（航迹共享的
        ``share_recipients``）：本项目的报文发送还没有别的**声明式**出口，
        指挥件那条路属后续。等有了，加进下面那个 for 循环的取值列表即可。
        """
        joins: dict[int, str] = {}
        entity_of = self._entity_by_decl_name()
        for entity_id, entity in self.entities.items():
            if entity_id in named:
                continue           # 已经在网里，不重复并入
            recipients: set[str] = set()
            for part in entity.parts():
                spec = getattr(part, "spec", None)
                value = spec.get("share_recipients") if hasattr(spec, "get") else None
                if value:
                    recipients.update(str(v) for v in value)
            if not recipients:
                continue
            nets = {
                named[target]
                for target in (entity_of.get(name) for name in recipients)
                if target in named
            }
            if nets:
                joins[entity_id] = sorted(nets)[0]
        return joins

    # -- 第 3 步：声明战区 -------------------------------------------------

    def _declare_zones(self) -> None:
        """把想定里的战区名映射成 zone_id，并声明（**不激活**）。

        zone_id 必须落在 0..4095（12 位可编码范围），所以只能用序号，
        不能用名字的哈希——哈希会超范围且不稳定。
        """
        for index, decl in enumerate(self.spec.zones, start=1):
            try:
                spec = ZoneSpec(
                    zone_id=index,
                    name=decl.name,
                    lat=decl.anchor.lat,
                    lng=decl.anchor.lng,
                    radius_m=decl.radius_m,
                    resolution_m=decl.resolution_m,
                    layer_count=decl.layers,
                    # 这两个早先被静默丢弃：ZoneSpec 没有对应字段，装配时
                    # 没往下传，于是"想定里写了 terrain 却没人生成地形"。
                    terrain=decl.terrain,
                    seed=decl.seed,
                )
            except ValueError as exc:
                # ZoneSpec 的校验错误在这里统一带上战区名与行号。
                # 未知的 terrain 取值也走这条路——**必须在装配时报错**，
                # 静默忽略会让人以为"写了 terrain 就有地形"（§9-12）。
                raise ScenarioError(
                    f"战区 {decl.name}：{exc}", decl.line
                ) from exc

            self.zone_ids[decl.name] = index
            self.maps.declare(spec)

    # -- 第 5 步：只激活有实体的战区 ---------------------------------------

    def _build_terrain_loader(self) -> Callable[[Zone], None]:
        """造一个战区 loader：给战区装上**逐块**地形生成（§3.11.5）。

        生成粒度就是被踩到的那一块（32×32 格）——这是可能的，因为噪声已经
        锚在**世界坐标**上（`terrain.fbm` 的晶格由世界坐标哈希决定），
        相邻块拼起来连续。用"本块的分位数"定水陆边界的写法在这里行不通，
        所以分类阈值也是固定值（`terrain._terrain_thresholds`）。

        逐块生成的收益不只是省内存：一个 60 km/200 m 的战区整块生成要
        一百多万格，边界上还有一大片永远没人去的块。
        """
        def prepare(zone: Zone) -> None:
            spec = zone.spec
            if spec.terrain == "none":
                # 不生成：读到的都是默认值（可通行的平原 + 海平面高程）。
                # 这是想定明确要的，不是遗漏。
                return

            def load(grid: HexGrid, cx: int, cy: int, level: int) -> None:
                # 空中层没有可生成的内容（风、空域管制），留空比填假值好
                if level != LEVEL_SURFACE:
                    return
                col0 = cx * CHUNK_SIZE
                row0 = cy * CHUNK_SIZE
                if spec.terrain == "flat":
                    generate_flat(
                        grid, col0, row0, CHUNK_SIZE, CHUNK_SIZE, level=level
                    )
                else:
                    generate_terrain(
                        grid, col0, row0, CHUNK_SIZE, CHUNK_SIZE,
                        seed=spec.seed, level=level,
                    )

            zone.chunk_loader = load

        return prepare

    def _warm_zones(self) -> None:
        """算出每个平台落在哪个战区，然后只激活被用到的那些。

        这是"按需加载"的落点。跨洲想定声明 20 个战区只用到 3 个时，
        另外 17 个一个字节的网格都不占。
        """
        needed: dict[int, str] = {}

        for decl in self.spec.platforms:
            zone_name = self._zone_of(decl)
            if zone_name is None:
                continue                 # 全球层，不需要局部战区
            zone_id = self.zone_ids.get(zone_name)
            if zone_id is None:
                raise ScenarioError(
                    f"平台 {decl.name} 引用了未声明的战区 {zone_name!r}",
                    decl.line,
                )
            needed[zone_id] = zone_name

        for zone_id in sorted(needed):
            self.maps.activate(zone_id)

    def _zone_of(self, decl: PlatformDecl) -> str | None:
        """平台属于哪个战区。返回 ``None`` 表示它还在全球层。

        两种位置写法走两条路：

        - ``latlng`` —— 按大圆距离对每个战区做判定，可能落空（跨洲航渡中）
        - ``hex``    —— 坐标本身相对某个战区锚点，战区必须能确定：
          写了 ``@ zone X`` 就用 X；没写则要求全想定只有一个战区
        """
        position = decl.position
        if position is None:
            return None

        if position.form == "hex":
            if position.zone:
                return position.zone
            if len(self.spec.zones) == 1:
                return self.spec.zones[0].name
            available = "、".join(z.name for z in self.spec.zones)
            raise ScenarioError(
                f"平台 {decl.name} 用了网格坐标但没写 @ zone——"
                f"本文件有 {len(self.spec.zones)} 个战区（{available}），"
                f"同一个 (q, r) 在不同战区里是两个地方。"
                f"写成 position hex {position.q} {position.r} @ zone <战区名>",
                decl.line,
            )

        hits = [
            zone
            for zone in self.spec.zones
            if great_circle_m(zone.anchor.lat, zone.anchor.lng,
                              position.lat, position.lng) <= zone.radius_m
        ]
        if not hits:
            return None
        if len(hits) > 1:
            # 战区重叠几乎总是想定写错了，而且会让实体在两个战区之间来回跳
            names = "、".join(z.name for z in hits)
            raise ScenarioError(
                f"平台 {decl.name} 的位置同时落在多个战区里：{names}——"
                "战区重叠会让实体坐标在加载边界来回切换，请调整半径或锚点",
                decl.line,
            )
        return hits[0].name

    # -- 第 6 步：实例化实体 -----------------------------------------------

    def _create_entities(self) -> None:
        for decl in self.spec.platforms:
            self._create_one(decl)

    def _create_one(self, decl: PlatformDecl) -> Entity:
        resolved = self.spec.types.resolve_platform(decl.type_name)
        entity = Entity()

        # 平台属性走**与组件参数同一套**解析（§5.6）：未知键报错带拼写建议、
        # 类型与范围校验、量纲换算。写错的平台属性不再"既不报错也不生效"。
        try:
            params = self.factory.resolved_platform_params(decl.type_name)
        except MilsimError as exc:
            raise ScenarioError(
                f"平台 {decl.name} 的平台属性有问题：{exc}", decl.line
            ) from exc
        entity.platform_params = dict(params)

        entity_id = self.registry.register(
            entity,
            name=decl.name,
            side=str(params.get("side", "")),
            type_name=decl.type_name,
            tags=tuple(sorted(resolved.components)),
        )
        self.platform_params[entity_id] = entity.platform_params

        ref, x, y, z = self._locate(decl)
        self.store.add(entity_id, ref)
        self.store.set_pose(entity_id, x, y, z, decl.heading_deg, 0.0)

        for slot, part in self.factory.build_for_platform(decl.type_name).items():
            entity.add(slot, part)

        if decl.formation:
            try:
                self.formations.add_member(decl.formation, entity_id)
            except ConfigurationError as exc:
                raise ScenarioError(
                    f"平台 {decl.name} 的编队归属有问题：{exc}", decl.line
                ) from exc

        self.entities[entity_id] = entity
        return entity

    def _locate(
        self, decl: PlatformDecl
    ) -> tuple[CellRef, float, float, float]:
        """算出实体的格归属与局部世界坐标。

        ★ **v0.13.39 起，落不到任何一个战区 ⇒ 装配期报错**（不再静默退回全球层）。

        旧行为是"战区外用全球层单元"，注释说"那里没有逐格机动，坐标意义不大"
        ——这句话只对了一半。真相是 :class:`GlobalCellRef` **只带一个 ``h3_cell``、
        不带任何米制坐标**，所以下面那行 ``return ..., 0.0, 0.0, 0.0`` 里的三个 0
        **是写死的**，不是"算出来就是 0"。后果链完全静默：

          平台的 ``latlng`` 落不到战区 ⇒ ``x, y, z = 0, 0, 0``
          ⇒ 所有这样的平台**叠在原点**（彼此不可分辨）
          ⇒ 雷达的 ``span_m``（由候选实体包围盒算出）``<= 0``
          ⇒ ``_sweep`` 在第 1658 行**一步早退**（``ticks`` 照涨、航迹恒 0）
          ⇒ **退出码 0**，图上什么都没有，参数表上看不出任何问题。

        ⇒ **"格归属必须给、坐标意义不大"是个假命题**：给了格归属却没有坐标，
        等于给了一张没有刻度的尺。本版把这条出口封掉——**当前没有任何合法路径
        能走到 ``GlobalCellRef``**（跨洲航渡需要真正的米制全球坐标，本版没有，
        见 §9-55）。要放在战区外就得先加战区。

        两种情形都报错，分两句提示（前一句是"一个战区都没有"，后一句是
        "有战区但落外面"，改法不同）：
        """
        position = decl.position
        assert position is not None, "装配前的校验已经保证了 position 非空"

        if position.form == "hex":
            zone = self._active_zone(position.zone or self._zone_of(decl) or "")
            axial = Axial(position.q, position.r)
            x, y = zone.frame.to_world(axial)
            self._check_inside(decl, zone, x, y)
            ref = self._enter(zone.spec.zone_id, position.layer, axial, 0.0)
            return ref, x, y, 0.0

        loc = self.maps.locate(position.lat, position.lng)
        if loc.is_local:
            assert loc.zone is not None and loc.axial is not None
            x, y = loc.zone.frame.to_world(loc.axial)
            ref = self._enter(loc.zone.spec.zone_id, 0, loc.axial, 0.0)
            return ref, x, y, 0.0

        # 落不到任何一个战区 ⇒ 没有米制坐标可用，**不能**拿 (0,0,0) 冒充。
        lat, lng = position.lat, position.lng
        if not self.spec.zones:
            raise ScenarioError(
                f"平台 {decl.name} 用了经纬度坐标（{lat} {lng}），但本文件"
                "**一个 zone 都没有**——没有战区就没有米制坐标，实体只能被"
                "摆到 (0,0,0)，而所有这样的平台会**叠在原点**（雷达参数一切"
                "正常但一条航迹都建不起来，也不报错）。"
                "请至少声明一个 zone 把平台框进去，例如：\n"
                "    zone <名字>\n"
                "        anchor <纬度> <经度>\n"
                "        radius <半径>\n"
                f"    end_zone     ← 让 {decl.name} 的位置落在它的半径内",
                decl.line,
            )
        names = "、".join(z.name for z in self.spec.zones)
        raise ScenarioError(
            f"平台 {decl.name} 的位置（{lat} {lng}）落在所有战区之外——"
            f"本文件有 {len(self.spec.zones)} 个战区（{names}），但都没覆盖这一点。"
            "战区外没有米制坐标，实体只能被摆到 (0,0,0) 并与其它越界平台叠在"
            "一起（雷达参数一切正常但一条航迹都建不起来，也不报错）。"
            "请把位置挪进某个战区，或把那个战区的半径 / 锚点调大。",
            decl.line,
        )

    def _enter(self, zone_id: int, layer: int, axial: Axial, z: float) -> LocalCellRef:
        """实体进入某一格：**顺手生成那一块地形**（§3.11.5）。

        "默认不生成、写了实体位置才加载相应地形"的落点就在这里。装配期
        （``_locate``）与运行期（``make_locator``）都走这一个方法——
        两处各写一遍的写法迟早会有一处忘了接，而"忘了接"不会报错，
        只是地形永远是默认值。
        """
        self.maps.load_chunk_at(zone_id, axial, layer, LEVEL_SURFACE)
        grid = self.maps.grid(zone_id, layer)
        # 垂向层按**离地高度**分（不是海拔）：高原上的坦克 AGL 只有几米，
        # 用海拔会把它算成低空。
        ground = grid.elevation(axial) if grid is not None else 0.0
        return LocalCellRef(zone_id, layer, axial, level_of_altitude(z - ground))

    def _check_inside(
        self, decl: PlatformDecl, zone: Zone, x: float, y: float
    ) -> None:
        """网格坐标必须在战区半径内。

        轴向坐标是**相对战区锚点**的，战区外根本没有网格。不校验的话，
        想定里把 ``(q, r)`` 写大一位就会让实体"落在战区里"（格归属写着
        zone1）而实际位置在几百公里外——雷达查得到它、地形却是空的，
        表现为"单位站在虚空里"。
        """
        offset = math.hypot(x, y)
        radius = zone.spec.radius_m
        if offset > radius:
            raise ScenarioError(
                f"平台 {decl.name} 的网格坐标 ({decl.position.q}, {decl.position.r}) "
                f"在战区 {zone.spec.name} 之外：距锚点 {offset/1000:.1f} km，"
                f"而战区半径只有 {radius/1000:.0f} km。"
                f"网格坐标是相对战区锚点的，战区外的位置请用 latlng",
                decl.line,
            )

    def _active_zone(self, name: str) -> Zone:
        zone_id = self.zone_ids.get(name)
        if zone_id is None:
            raise SimulationError(f"未声明的战区 {name!r}")
        zone = self.maps.zone(zone_id)
        if zone is None:
            raise SimulationError(
                f"战区 {name!r} 未激活——网格坐标属于某个战区，用它之前"
                "必须先让该战区有实体落入（或显式调用 sim.activate_zone）"
            )
        return zone

    def activate_zone(self, name: str) -> Zone:
        """显式激活一个战区。

        ``build()`` 只激活有实体落入的那些；想在空战区里做推演（比如
        先看地形）时用它补一个。
        """
        zone_id = self.zone_ids.get(name)
        if zone_id is None:
            available = "、".join(sorted(self.zone_ids)) or "（一个都没有）"
            raise SimulationError(f"未声明的战区 {name!r}。已声明：{available}")
        return self.maps.activate(zone_id)

    # -- 决策接线 ----------------------------------------------------------

    def _wire_decisions(self) -> None:
        self._check_fallback_cycles()
        for name in sorted(self.spec.decisions):
            self.decisions[name] = self._make_provider(name, ())

    def _check_fallback_cycles(self) -> None:
        """装配前先把 fallback 的环拦掉。

        不能只靠构造时的 ``chain`` 检测：``rule`` 这类不需要降级的 provider
        根本不走递归构造，成环配置会被静默接受。等到真需要降级的那一刻
        （模型服务挂了）才发现，那就太晚了。
        """
        for name in sorted(self.spec.decisions):
            seen = [name]
            cursor = self.spec.decisions[name].fallback
            while cursor and cursor in self.spec.decisions:
                if cursor in seen:
                    cycle = " → ".join([*seen, cursor])
                    raise ScenarioError(f"决策配置的 fallback 成环：{cycle}")
                seen.append(cursor)
                cursor = self.spec.decisions[cursor].fallback

    def _make_provider(self, name: str, chain: tuple[str, ...]) -> Any:
        """按配置造 provider，降级链递归构造。

        ``chain`` 记录已经构造过的配置名，用来挡住 ``A → B → A`` 这种环——
        递归构造时成环会直接栈溢出，报错信息还不清不楚。
        """
        existing = self.decisions.get(name)
        if existing is not None:
            return existing

        if name in chain:
            cycle = " → ".join([*chain, name])
            raise ScenarioError(f"决策配置的 fallback 成环：{cycle}")

        profile = self.spec.decisions.get(name)
        if profile is None:
            # 不是配置名，那就是内置 provider 名（如 "rule"）
            if name in ("rule", "script", "behavior_tree"):
                return RuleDecisionProvider(self._rule_policy, name=name)
            raise ScenarioError(f"未知的决策配置或提供者 {name!r}")

        provider = self._build_provider(profile, (*chain, name))
        self.decisions[name] = provider
        return provider

    def _build_provider(self, profile: DecisionProfile, chain: tuple[str, ...]) -> Any:
        if profile.provider == "llm":
            if self._llm_client_factory is None:
                raise ScenarioError(
                    f"决策配置 {profile.name} 用大模型，但没有提供客户端工厂——"
                    "想定只声明'用哪个模型'，真正的客户端由装配层构造"
                    "（解析阶段不碰网络，见 §3.8.2）",
                    profile.line,
                )
            client = self._llm_client_factory(profile)
            fallback = (
                self._make_provider(profile.fallback, chain)
                if profile.fallback
                else RuleDecisionProvider(self._rule_policy, name="fallback")
            )
            return LLMDecisionProvider(
                client,
                fallback,
                default_timeout_us=profile.timeout_us,
                name=profile.name,
            )

        # rule / script / behavior_tree 目前都落到规则提供者上——
        # 行为树与脚本实现属 M4，装配层先保证接线是通的
        return RuleDecisionProvider(self._rule_policy, name=profile.name)

    # -- 第 7 步：注入服务 -------------------------------------------------

    def initialize(self) -> None:
        """引擎启动 + 各部件接线。"""
        if not self._built:
            self.build()
        if self._initialized:
            return

        self.engine.start()
        for entity_id in sorted(self.entities):
            entity = self.entities[entity_id]
            entity.initialize(self.mount_for(entity_id))

        self._initialized = True

    def mount_for(self, entity_id: int) -> MountContext:
        """给某个实体造一份装配上下文。测试里也用它手工接线。"""
        entity = self.entities.get(entity_id)
        if entity is None:
            raise SimulationError(f"实体 {entity_id} 不在本仿真里")
        return MountContext(
            engine=self.engine,
            store=self.store,
            registry=self.registry,
            maps=self.maps,
            random=self.random,
            bus=self.bus,
            entity_id=entity_id,
            entity=entity,
            locator=self.make_locator(entity_id),
            nav=self.nav,
            jam=self.jam,
            comm=self.comm,
            platform_params=self.platform_params.get(entity_id, {}),
        )

    def make_locator(self, entity_id: int):
        """造一个"局部坐标 → 格归属"的转换器给机动组件用。

        只处理局部层。全球层暂时不动——从局部坐标反推 H3 单元需要经纬度，
        而战区外没有局部坐标系可用。跨洲航渡中的格同步属 M4，
        那时实体自己会带着经纬度。
        """

        def locate(x: float, y: float, z: float) -> CellRef:
            ref = self.store.cell_of(entity_id)
            if isinstance(ref, LocalCellRef):
                zone = self.maps.zone(ref.zone_id)
                if zone is not None:
                    # 与装配期走同一条路径（含"踩到哪生成哪"）
                    return self._enter(
                        ref.zone_id, ref.layer, zone.frame.from_world(x, y), z
                    )
            assert ref is not None, "已登记的实体必须有格归属"
            return ref

        return locate

    # -- 驱动 --------------------------------------------------------------

    def run_for(self, duration_us: int) -> int:
        """推进 ``duration_us`` 微秒，返回结束时刻。

        内部就是个循环——引擎不知道自己在推进一场军事推演。换成资源调度、
        生产线仿真，同一套 ``Engine`` 照样跑。
        """
        if not self._initialized:
            self.initialize()
        return self.engine.run_to(self.engine.now + duration_us)

    def run_until(self, sim_time_us: int) -> int:
        if not self._initialized:
            self.initialize()
        return self.engine.run_to(sim_time_us)

    def step(self) -> int:
        if not self._initialized:
            self.initialize()
        return self.engine.step_once()

    # -- 收尾 --------------------------------------------------------------

    def shutdown(self) -> SimulationReport:
        """关闭部件、回收线程池、产出统计。"""
        for entity_id in sorted(self.entities, reverse=True):
            self.entities[entity_id].shutdown()

        for provider in self.decisions.values():
            close = getattr(provider, "shutdown", None)
            if callable(close):
                close()

        return self.report()

    def report(self) -> SimulationReport:
        decisions: dict[str, Any] = {}
        for name, provider in sorted(self.decisions.items()):
            stats = getattr(provider, "stats", None)
            if stats is not None:
                decisions[name] = stats

        return SimulationReport(
            sim_time_us=self.engine.now,
            steps=self.engine.step_count,
            events_dispatched=self.engine.dispatch_count,
            fences=self.engine.fence_count,
            entities=len(self.entities),
            alive=len(self.store.alive_ids()),
            zones_active=len(self.maps.active_zones),
            decisions=decisions,
            library_revision=self.spec.library_revision,
        )

    # -- 查询 --------------------------------------------------------------

    def alive_entities(self) -> Iterator[Entity]:
        for entity_id in self.store.alive_ids():
            entity = self.entities.get(entity_id)
            if entity is not None:
                yield entity

    def entity_by_name(self, name: str) -> Entity | None:
        found = self.registry.by_name(name)
        if found is None:
            return None
        return self.entities.get(found.entity_id)

    def validate(self) -> list[str]:
        """全量自检。测试与调试用。"""
        problems = list(self.spec.validate(self.factory))
        problems.extend(self.store.validate())
        problems.extend(self.registry.validate())
        return problems

    def __repr__(self) -> str:
        return (
            f"<Simulation {self.registry} 实体 / "
            f"{len(self.zone_ids)} 战区声明 / {len(self.maps.active_zones)} 已激活>"
        )


__all__ = ["Simulation", "SimulationError", "SimulationReport"]
