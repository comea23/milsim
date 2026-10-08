"""想定语义层：块节点树 → 强类型声明（``ScenarioSpec``）。

这一层做四件事，别的都不做：

1. **键名校验** —— 未知参数报错（带拼写建议），重复参数报错
2. **单位换算** —— 复用 ``ParamSet``，``120 km`` → ``120000.0``
3. **引用校验** —— 平台引用的类型存在、决策配置存在
4. **产出纯数据** —— ``ScenarioSpec`` 里全是基本类型与 dataclass

不在这里做的：**激活战区、实例化实体、连接大模型**。那是装配层
（M3c-5 的 ``Simulation``）的事。分开的好处是想定解析可以完全离线测试——
不需要地图、不需要网络、不需要 API 密钥。

大模型驱动怎么落地
------------------
想定里加一个 ``decision`` 块，平台通过 ``decision NAME`` 引用它：

.. code-block:: text

    decision TACTICAL_LLM
        provider    llm
        model       "deepseek-chat"
        endpoint    "https://api.deepseek.com/v1"
        api_key_env "MILSIM_LLM_KEY"
        timeout     8 s
        cadence     30 s
        fallback    RULE_DEFAULT
    end_decision

    platform SAM_1 SAM_BATTALION
        position latlng 39.95 116.35
        decision TACTICAL_LLM
    end_platform

两条安全纪律，写在代码里也写进文档：

**① 想定里绝不写密钥，只写环境变量名。** 想定文件会进版本库、会被复制、
会出现在截图里。``api_key_env "MILSIM_LLM_KEY"`` 存的是**名字**，
密钥本身由装配层从环境读。

**② 解析阶段不碰网络。** 这里产出的只是"想用什么模型、超时多久、降级到什么"
这份声明。真正的客户端由装配层构造——所以换模型不需要改想定，改配置即可；
想定解析也能在没有网络的环境里跑测试。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

from ..errors import ConfigurationError
from .command import CommandChain, TimeWindow
from .formation import FormationTree
from .input.parser import (
    Assignment,
    Block,
    TaggedValue,
    parse,
)
from .map.zone import (
    MAX_CELL_SIZE_M,
    MAX_ZONE_RADIUS_M,
    MIN_CELL_SIZE_M,
    MIN_ZONE_RADIUS_M,
    TERRAIN_SOURCES,
)
from .params import (
    CoordValue,
    ParamError,
    ParamSet,
    Params,
    WithUnit,
)
from .library import KIND_COMPONENT, ParamLibrary
from .type_registry import ComponentSpec, TypeDefinitionError, TypeRegistry

# ---------------------------------------------------------------------------
# 框架已知块的参数模式
# ---------------------------------------------------------------------------

#: 决策提供者。与 §4.3 的四种实现一一对应。
PROVIDERS = ("rule", "script", "behavior_tree", "llm")

#: 战区声明的参数。
#:
#: 范围用的是 `zone.py` 的常量而**不是另抄一份**——想定层的校验和
#: `ZoneSpec` 的校验必须是同一套，否则会出现"lint 说没问题、
#: 装配时才抛 ValueError"这种最恼人的体验。
ZONE_PARAMS = ParamSet(
    {
        "anchor": Params.coord(),
        "radius": Params.distance(
            100_000.0, minimum=MIN_ZONE_RADIUS_M, maximum=MAX_ZONE_RADIUS_M
        ),
        "resolution": Params.distance(
            1_000.0, minimum=MIN_CELL_SIZE_M, maximum=MAX_CELL_SIZE_M
        ),
        "layers": Params.integer(1, minimum=1, maximum=8),
        # 取值走白名单：写了不认识的地形来源必须当场报错，不能静默丢弃
        # ——"想定里写了 terrain 却没人生成地形"是 §9-12 记的那个坑。
        "terrain": Params.string("procedural", choices=TERRAIN_SOURCES),
        "seed": Params.integer(0, minimum=0),
    },
    owner="zone",
)

#: 决策配置的参数。
DECISION_PARAMS = ParamSet(
    {
        "provider": Params.string("rule", choices=PROVIDERS),
        "model": Params.string(""),
        "endpoint": Params.string(""),
        "api_key_env": Params.string(""),
        "timeout": Params.duration(8_000_000, minimum=1_000),
        "cadence": Params.duration(30_000_000, minimum=0),
        "fallback": Params.string(""),
        "max_context": Params.integer(4096, minimum=256),
        "temperature": Params.number(0.0, minimum=0.0, maximum=2.0),
        "prompt": Params.string(""),
    },
    owner="decision",
)

#: 全局设置。``max_step`` 的上限就是引擎的硬约束（§2.4 的 2 秒栅栏）——
#: 在这里拦住，比等引擎跑到一半才发现推不动要好。
SIMULATION_PARAMS = ParamSet(
    {
        "max_step": Params.duration(2_000_000, minimum=1_000, maximum=2_000_000),
        "seed": Params.integer(0, minimum=0),
        "name": Params.string(""),
    },
    owner="simulation",
)

#: 编队声明的参数。``side`` 必填——编队的阵营决定了谁能配属它。
#: ``echelon`` 是**纯显示标签**（"旅" / "营" / "连"），框架不解释它的
#: 含义：各国各军种的层级名与层数都不同，硬编码成枚举会把使用者锁死。
#: 层级关系由 ``formation X : Y`` 表达，不由标签表达。
FORMATION_PARAMS = ParamSet(
    {"side": Params.string(""), "echelon": Params.string("")},
    owner="formation",
)

#: 时间子句的量纲。单独拎出来给 ``from 30 min`` / ``until 8 h`` 用。
#: 走统一的单位表，**整数微秒**，不经浮点（§3.8.3）。
_DURATION = Params.duration(0)

#: 平台**实例**才有的键。写在 ``platform_type`` 里是概念错误——位置、
#: 航向、编队归属都是每一件装备自己的属性，不是型号的属性。
INSTANCE_ONLY_KEYS = frozenset({"position", "heading", "formation"})


class ScenarioError(ConfigurationError):
    """想定语义错误。带行号，与词法/语法错误统一处理。"""

    def __init__(self, message: str, line: int = 0, column: int = 1) -> None:
        self.line = line
        self.column = column
        self.length = 1
        super().__init__(message)


# ---------------------------------------------------------------------------
# 声明对象：全是纯数据
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class ZoneDecl:
    """一个战区的声明。**只是声明**——激活与加载网格由装配层决定。"""

    name: str
    anchor: CoordValue
    radius_m: float
    resolution_m: float
    layers: int
    terrain: str
    seed: int
    line: int = 0

    def estimated_cells(self) -> int:
        """粗略估算单元数，用于装配前提示规模。"""
        area = math.pi * self.radius_m * self.radius_m
        per_cell = (self.resolution_m * math.sqrt(3.0)) ** 2 * 0.866
        return int(area / per_cell) if per_cell > 0 else 0


@dataclass(frozen=True, slots=True)
class DecisionProfile:
    """一份决策配置。

    ``fallback`` 指向另一份配置（或内置 provider 名）。**没有 fallback 的
    大模型决策在推演里等于定时炸弹**——模型服务抖一下整个想定就跑不完，
    所以 ``provider=llm`` 时装配层会要求它非空。
    """

    name: str
    provider: str = "rule"
    model: str = ""
    endpoint: str = ""
    api_key_env: str = ""
    timeout_us: int = 8_000_000
    cadence_us: int = 30_000_000
    fallback: str = ""
    max_context: int = 4096
    temperature: float = 0.0
    prompt: str = ""
    line: int = 0

    @property
    def uses_llm(self) -> bool:
        return self.provider == "llm"

    @property
    def timeout_s(self) -> float:
        return self.timeout_us / 1_000_000.0

    @property
    def cadence_s(self) -> float:
        return self.cadence_us / 1_000_000.0


@dataclass(frozen=True, slots=True)
class PlatformDecl:
    """一个平台实例的声明。位置待装配阶段定位（战区要先激活）。"""

    name: str
    type_name: str
    position: CoordValue | None = None
    heading_deg: float = 0.0
    decision: str = ""
    #: 所属编队（行政隶属）。写在 ``platform`` 块里而不是 ``formation`` 块里——
    #: 一个连有几十个平台，列在编队块里会让它爆掉。归属是**实例属性**
    #: （同 ``position``），不是型号属性。
    formation: str = ""
    attrs: Mapping[str, Any] = field(default_factory=dict)
    line: int = 0


@dataclass(frozen=True, slots=True)
class FormationDecl:
    """一个编队节点的声明（行政隶属，第一层）。"""

    name: str
    side: str
    echelon: str = ""
    parent: str | None = None
    line: int = 0


@dataclass(frozen=True, slots=True)
class AttachmentDecl:
    """配属声明（作战归属，第二层）。``until_us=None`` 表示不限期。"""

    unit: str
    parent: str
    start_us: int = 0
    until_us: int | None = None
    note: str = ""
    line: int = 0


@dataclass(frozen=True, slots=True)
class DirectiveDecl:
    """直接指挥通道（第三层，越级）。"""

    superior: str
    subordinate: str
    start_us: int = 0
    until_us: int | None = None
    line: int = 0


@dataclass(frozen=True, slots=True)
class CoordinationDecl:
    """协同通道（第三层，对等）。"""

    a: str
    b: str
    start_us: int = 0
    until_us: int | None = None
    line: int = 0


@dataclass(frozen=True, slots=True)
class NetworkDecl:
    """一个通信网的声明（§5.15）。**纯数据**——建图与寻路在
    :class:`~milsim.services.comm.CommService`。

    ★ 成员与链路用**平台实例名**（不是实体 ID）：想定是给**人**看的，
    实体 ID 是装配期才有的东西（`_create_entities` 里才分配）。名字 → ID
    的解析在装配层做，这一步只管语法。

    ``links`` 为空 ⇒ "**入网即全连通**"（AFSIM 的默认行为：同网成员两两有边）；
    非空 ⇒ 只用这些**有向边**（"真实节点分布"就是这个意思）。
    """

    name: str
    members: tuple[str, ...] = ()
    #: 显式有向边：``(源平台名, 目的平台名)``。空 ⇒ 全连通。
    links: tuple[tuple[str, str], ...] = ()
    line: int = 0


@dataclass(frozen=True, slots=True)
class SimulationSettings:
    max_step_us: int = 2_000_000
    seed: int = 0
    name: str = ""

    @property
    def max_step_s(self) -> float:
        return self.max_step_us / 1_000_000.0


@dataclass(slots=True)
class ScenarioSpec:
    """一份想定的全部声明。装配层拿它去建世界。"""

    types: TypeRegistry
    zones: list[ZoneDecl] = field(default_factory=list)
    platforms: list[PlatformDecl] = field(default_factory=list)
    decisions: dict[str, DecisionProfile] = field(default_factory=dict)
    formations: list[FormationDecl] = field(default_factory=list)
    attachments: list[AttachmentDecl] = field(default_factory=list)
    directives: list[DirectiveDecl] = field(default_factory=list)
    coordinations: list[CoordinationDecl] = field(default_factory=list)
    #: 通信网声明（§5.15）。空 ⇒ 想定里根本没提通信 ⇒ 消息投递不受限
    #: （"没建网 ⇒ 零开销"是验收判据，见 :class:`CommService.is_empty`）。
    networks: list[NetworkDecl] = field(default_factory=list)
    settings: SimulationSettings = field(default_factory=SimulationSettings)
    source_name: str = "<想定>"
    #: 装载时注入的参数库（§5.5）。型号**已经灌进 ``types``**，这里留引用
    #: 是为了能回答两个问题：这次跑用的是哪一版参数
    #: （:attr:`library_revision`），以及某个型号是库给的还是想定写的。
    libraries: list[ParamLibrary] = field(default_factory=list)

    # -- 查询 --------------------------------------------------------------

    @property
    def library_revision(self) -> str:
        """所有参数库的修订拼接。空串表示这次跑没有用参数库。

        记这个值是为了**可复现**：同一个想定文件两次跑出不同结果时，第一件
        要查的事就是"参数被改过没有"。没有这个值，就只能靠回忆了。
        """
        return "+".join(lib.revision for lib in self.libraries)

    def origin_of(self, name: str, kind: str = "component") -> str:
        """某个型号是哪儿来的：库文件路径，或 ``想定``。

        组件型与平台型是两个名字空间（同名可以并存），所以种类要传。
        """
        registry = self.types
        if kind == "component":
            entry = registry.find_component_type(name)
        else:
            entry = registry.find_platform_type(name)
        if entry is None:
            return ""
        return entry.origin or "想定"

    # -- 通信网查询（§5.15） ------------------------------------------------

    def network_of(self, platform_name: str) -> str:
        """某个平台在哪个通信网里；不在任何网里返空串。

        ★ **一个实体只能属于一个网**：查出来两个 ⇒ 这本来就是想定错误，
        但这里**不报错**——报错的位置在装配层（那里才有实体 ID 与行号上下文），
        这里只回答"第一个匹配的"。装配层用的是它自己建的 ``名字→网名`` 索引，
        不走这里。
        """
        for net in self.networks:
            if platform_name in net.members:
                return net.name
        return ""

    def network_names(self) -> list[str]:
        """全部网名，声明顺序（可复现）。"""
        return [net.name for net in self.networks]

    def zone(self, name: str) -> ZoneDecl | None:
        for zone in self.zones:
            if zone.name == name:
                return zone
        return None

    def decision_for(self, platform: PlatformDecl) -> DecisionProfile | None:
        """平台用哪份决策配置。没引用则返回 ``None``（由装配层决定默认值）。"""
        return self.decisions.get(platform.decision) if platform.decision else None

    def effective_decision(self, platform: PlatformDecl) -> DecisionProfile | None:
        """实例写了的优先，否则用平台类型上的默认。

        两级是有用的：整个营默认走规则，个别关键岗位（指挥所）才走大模型——
        不必给每个实例都写一行 ``decision``。
        """
        if platform.decision:
            return self.decisions.get(platform.decision)

        resolved = self.types.resolve_platform(platform.type_name)
        inherited = resolved.attributes.get("decision")
        if isinstance(inherited, str) and inherited:
            return self.decisions.get(inherited)
        return None

    def default_decision(self) -> DecisionProfile | None:
        for name in sorted(self.decisions):
            if name.upper().startswith("DEFAULT"):
                return self.decisions[name]
        return None

    def llm_profiles(self) -> list[DecisionProfile]:
        return [p for p in self.decisions.values() if p.uses_llm]

    # -- 校验 --------------------------------------------------------------

    def validate(self, factory=None) -> list[str]:
        """跨块一致性自检。返回问题列表（空表示干净）。"""
        problems: list[str] = []

        # 类型表自身的自检：继承成环、父类型缺失。这些错误定义时不报
        # （继承链在实例化时才解析），但**校验阶段必须能发现**——
        # 否则一个成环的类型定义会静静躺着，等第一次实例化时才炸
        problems.extend(self.types.validate())

        seen: dict[str, int] = {}
        for platform in self.platforms:
            seen[platform.name] = seen.get(platform.name, 0) + 1
        for name, count in sorted(seen.items()):
            if count > 1:
                problems.append(
                    f"平台 {name!r} 定义了 {count} 次——注册表会因为重名报错，"
                    "但那是装配阶段的事，想定检查就该拦下"
                )

        for platform in self.platforms:
            if not self.types.has_platform_type(platform.type_name):
                problems.append(
                    f"平台 {platform.name} 引用了未定义的平台类型 "
                    f"{platform.type_name!r}"
                )
            if platform.decision and platform.decision not in self.decisions:
                available = "、".join(sorted(self.decisions)) or "（一份都没有）"
                problems.append(
                    f"平台 {platform.name} 引用了未定义的决策配置 "
                    f"{platform.decision!r}。已定义：{available}"
                )
            if platform.position is None:
                problems.append(f"平台 {platform.name} 没有 position")

        for name, profile in sorted(self.decisions.items()):
            if not profile.uses_llm:
                continue
            if not profile.model:
                problems.append(f"决策配置 {name} 用 llm 但没写 model")
            if not profile.api_key_env:
                problems.append(
                    f"决策配置 {name} 用 llm 但没写 api_key_env"
                    "（想定里只写环境变量名，不写密钥）"
                )
            if not profile.fallback:
                problems.append(
                    f"决策配置 {name} 用 llm 但没有 fallback——"
                    "模型服务抖一下整个想定就跑不完"
                )
            if profile.fallback and (
                profile.fallback not in self.decisions
                and profile.fallback not in PROVIDERS
            ):
                problems.append(
                    f"决策配置 {name} 的 fallback {profile.fallback!r} 不存在"
                )
                continue

            # fallback 成环是**配置问题**，不该等构造时才暴露——而且
            # rule 这类不需要降级的 provider 根本不走递归构造，那时才发现不了
            seen = [name]
            cursor = profile.fallback
            while cursor and cursor in self.decisions:
                if cursor in seen:
                    problems.append(
                        "决策配置的 fallback 成环："
                        + " → ".join([*seen, cursor])
                    )
                    break
                seen.append(cursor)
                cursor = self.decisions[cursor].fallback

        if factory is not None:
            # 平台**类型**上的参数也要校验（§5.6）。只校验"实例用到的型号"
            # 不够：一份想定里可能定义了十个平台类型而只实例化三个，另外
            # 七个里的错值会静静躺着，等哪天有人开始用才炸。
            for name in sorted(self.types.platform_names()):
                try:
                    factory.resolved_platform_params(name)
                except (ParamError, TypeDefinitionError) as exc:
                    problems.append(f"平台类型 {name}：{exc}")

            # 平台类型上的 decision 也要指向真实存在的配置。实例级的那条
            # 在下面查；漏掉类型级的话，`effective_decision` 会**静默**退回
            # 默认配置——想定作者以为改成了大模型，实际还在跑规则。
            for name in sorted(self.types.platform_names()):
                try:
                    attrs = self.types.resolve_platform(name).attributes
                except TypeDefinitionError:
                    continue          # 继承链本身有问题，上面已经报过
                choice = attrs.get("decision")
                if isinstance(choice, str) and choice and choice not in self.decisions:
                    available = "、".join(sorted(self.decisions)) or "（一份都没有）"
                    problems.append(
                        f"平台类型 {name} 引用了未定义的决策配置 "
                        f"{choice!r}。已定义：{available}"
                    )

            for platform in self.platforms:
                resolved = self.types.resolve_platform(platform.type_name)
                for slot in resolved.slots():
                    spec = resolved.components[slot]
                    try:
                        factory.resolved_params(spec.type_name, spec.attrs)
                    except (ParamError, TypeDefinitionError) as exc:
                        problems.append(
                            f"平台 {platform.name} 的 {slot} 槽：{exc}"
                        )

        problems.extend(self._validate_command())
        return problems

    def _validate_command(self) -> list[str]:
        """编队与指挥关系的跨块校验。

        单独跑一遍而不是复用 ``command_chain()``：后者会**抛异常**，
        而校验的职责是**收集全部问题**一次报出来。
        """
        problems: list[str] = []
        known = {decl.name for decl in self.formations}

        for platform in self.platforms:
            if platform.formation and platform.formation not in known:
                hint = _name_hint(platform.formation, known)
                problems.append(
                    f"平台 {platform.name} 引用了未定义的编队 "
                    f"{platform.formation!r}{hint}"
                )

        def check(name: str, where: str) -> None:
            if name not in known:
                problems.append(f"{where} 引用了未定义的编队 {name!r}"
                                + _name_hint(name, known))

        for item in self.attachments:
            check(item.unit, f"attach {item.unit} to {item.parent}")
            check(item.parent, f"attach {item.unit} to {item.parent}")
        for item in self.directives:
            check(item.superior, f"direct {item.superior} to {item.subordinate}")
            check(item.subordinate, f"direct {item.superior} to {item.subordinate}")
        for item in self.coordinations:
            check(item.a, f"coordinate {item.a} with {item.b}")
            check(item.b, f"coordinate {item.a} with {item.b}")

        return problems

    def command_chain(self) -> CommandChain:
        """按声明构建三层指挥关系（§3.10）。

        由装配层调用。放在这里而不是 ``Simulation`` 里，是为了让指挥关系
        能脱离完整仿真单独测——毕竟"配属时间段重叠"这类问题跟引擎无关。
        """
        tree = FormationTree()

        # 父节点必须先建。声明顺序不保证父在前（允许前向引用），
        # 所以按"父链已知"反复扫，扫不动了就是有未定义或成环。
        pending = list(self.formations)
        while pending:
            progressed = False
            for decl in list(pending):
                if decl.parent is None or decl.parent in _names_in(tree):
                    tree.add_unit(
                        decl.name,
                        side=decl.side,
                        echelon=decl.echelon,
                        parent=decl.parent,
                    )
                    pending.remove(decl)
                    progressed = True
            if not progressed:
                names = "、".join(d.name for d in pending)
                raise ScenarioError(
                    f"编队 {names} 的上级找不到——要么上级未定义，要么存在循环引用"
                )

        chains = CommandChain(tree)
        for item in self.attachments:
            chains.attach(
                item.unit,
                item.parent,
                window=TimeWindow(item.start_us, item.until_us),
                note=item.note,
                line=item.line,
            )
        for item in self.directives:
            chains.direct(
                item.superior,
                item.subordinate,
                window=TimeWindow(item.start_us, item.until_us),
                line=item.line,
            )
        for item in self.coordinations:
            chains.coordinate(
                item.a,
                item.b,
                window=TimeWindow(item.start_us, item.until_us),
                line=item.line,
            )
        return chains

    def summary(self) -> str:
        """一行概览，装配前打日志用。"""
        cells = sum(z.estimated_cells() for z in self.zones)
        llm = len(self.llm_profiles())
        chain = (
            f" / 编队 {len(self.formations)}（配属 {len(self.attachments)}）"
            if self.formations else ""
        )
        # 组件类型总数里**含库提供的型号**，分开报出来才不会让人以为
        # 想定里写了十几个 component_type 块
        from_library = sum(
            len(lib.records(KIND_COMPONENT)) for lib in self.libraries
        )
        components = f"{len(self.types.component_names())} 组件类型"
        library = ""
        if self.libraries:
            components += f"（其中 {from_library} 个来自库）"
            library = (
                f" / 参数库 {len(self.libraries)} 个"
                f"（{sum(len(lib) for lib in self.libraries)} 个型号，"
                f"修订 {self.library_revision}）"
            )
        return (
            f"{self.source_name}: "
            f"{components} / "
            f"{len(self.types.platform_names())} 平台类型 / "
            f"{len(self.zones)} 战区（约 {cells:,} 单元）/ "
            f"{len(self.platforms)} 平台 / "
            f"{len(self.decisions)} 决策配置（其中 {llm} 份用大模型）"
            f"{chain}"
            f"{library}"
        )

    def __repr__(self) -> str:
        return f"<ScenarioSpec {self.summary()}>"


# ---------------------------------------------------------------------------
# 语义解析
# ---------------------------------------------------------------------------

#: 参数库参数的形态。写成字符串别名是因为这几个类型只在文档上有意义。
LibraryItem = "ParamLibrary | str | Path"
LibrarySource = "LibraryItem | Sequence[LibraryItem] | None"


def _resolve_libraries(source: Any) -> list[ParamLibrary]:
    """参数库参数 → 库对象列表。

    允许直接给路径，是因为最常见的使用方式是"库文件就在想定旁边"；
    每次都要先 ``ParamLibrary.from_sqlite(...)`` 只是多一行样板。
    """
    if source is None:
        return []

    items = source if isinstance(source, (list, tuple)) else [source]
    resolved: list[ParamLibrary] = []
    for item in items:
        if isinstance(item, ParamLibrary):
            resolved.append(item)
        elif isinstance(item, (str, Path)):
            resolved.append(ParamLibrary.from_path(item))
        else:
            raise ConfigurationError(
                "参数库（library）只接受 ParamLibrary、库文件路径、"
                f"或它们的序列，收到 {type(item).__name__}"
            )
    return resolved


def load_scenario(
    text: str,
    *,
    name: str = "<想定>",
    library: "LibrarySource" = None,
) -> ScenarioSpec:
    """想定文本 → ``ScenarioSpec``。

    ``library`` 是参数库（§5.5）：一个 :class:`ParamLibrary`、一个库文件
    路径、或者它们的序列。库里的型号会被灌进类型表，想定里直接写型号名即可。
    """
    return build_spec(parse(text), name=name, library=library)


def build_spec(
    blocks: Sequence[Block],
    *,
    name: str = "<想定>",
    library: "LibrarySource" = None,
) -> ScenarioSpec:
    """块列表 → ``ScenarioSpec``。

    按块逐个派发，未知块类型直接报错——**不静默跳过**。写错块名的想定
    如果照常启动，"为什么我配的东西没生效"能查一整天。

    **参数库先于任何块灌进类型表。** 顺序不能反：想定里写
    ``component_type X : LIB_RADAR`` 时，父类型得已经在表里（继承链在
    实例化时才解析，但重名检测在定义时就报）。反过来的后果是——
    库里的型号晚一步进来，与想定里的重名不会被发现，于是库里那份静默
    被忽略了。
    """
    spec = ScenarioSpec(TypeRegistry(), source_name=name)
    spec.libraries = _resolve_libraries(library)
    for lib in spec.libraries:
        lib.seed_into(spec.types)

    for block in blocks:
        keyword = block.keyword
        if keyword == "component_type":
            _read_component_type(spec, block)
        elif keyword == "platform_type":
            _read_platform_type(spec, block)
        elif keyword == "zone":
            _read_zone(spec, block)
        elif keyword == "platform":
            _read_platform(spec, block)
        elif keyword == "decision":
            _read_decision(spec, block)
        elif keyword == "simulation":
            _read_simulation(spec, block)
        elif keyword == "formation":
            _read_formation(spec, block)
        elif keyword == "command":
            _read_command(spec, block)
        elif keyword == "network":
            _read_network(spec, block)
        else:
            raise ScenarioError(
                f"不认识的块 {keyword!r}"
                f"（可用的块：component_type / platform_type / zone / "
                f"platform / decision / simulation / formation / command / "
                f"network）",
                block.line,
            )

    return spec


# -- 各块 ----------------------------------------------------------------

def _read_component_type(spec: ScenarioSpec, block: Block) -> None:
    if not block.names:
        raise ScenarioError("component_type 后面缺少类型名", block.line)
    if len(block.names) > 1:
        raise ScenarioError(
            f"component_type 只接一个名字，多出了 {'、'.join(block.names[1:])}",
            block.line,
        )

    values, _ = _collect_params(block, owner=f"component_type {block.name}")

    for child in block.blocks():
        raise ScenarioError(
            f"component_type {block.name} 里出现了子块 {child.keyword!r}——"
            "想定里不写子块。若是 attr：它是**参数库**的东西"
            "（存毁伤概率、国别这类组件没有消费者的量），写进想定没有读者，"
            "要写就写到 library/*.txt 的型号里",
            child.line,
        )

    # 参数值原样存着（可能是 WithUnit），单位换算交给 ComponentFactory——
    # 只有它拿得到实现类的 PARAMS，才知道每个参数是什么量纲
    spec.types.define_component_type(
        block.name, parent=block.parent, attrs=values, line=block.line
    )


def _read_platform_type(spec: ScenarioSpec, block: Block) -> None:
    if not block.names:
        raise ScenarioError("platform_type 后面缺少类型名", block.line)
    if len(block.names) > 1:
        raise ScenarioError(
            f"platform_type 只接一个名字，多出了 {'、'.join(block.names[1:])}",
            block.line,
        )

    values, _ = _collect_params(block, owner=f"platform_type {block.name}")

    for key in sorted(INSTANCE_ONLY_KEYS & set(values)):
        raise ScenarioError(
            f"platform_type {block.name} 里不该写 {key!r}——"
            "位置和航向是每一件装备自己的属性，写在实例上（platform 块）",
            block.line,
        )

    spec.types.define_platform_type(
        block.name, parent=block.parent, attributes=values, line=block.line
    )

    for child in block.blocks("component"):
        if len(child.names) != 2:
            raise ScenarioError(
                f"component 需要「槽名 类型名」两个名字，"
                f"实际给了 {len(child.names)} 个"
                f"（如 component sensor HEX_SEARCH_RADAR）",
                child.line,
            )
        slot, type_name = child.names
        attrs, _ = _collect_params(child, owner=f"{block.name}.{slot}")
        spec.types.add_component(
            block.name, ComponentSpec(slot, type_name, attrs, child.line)
        )

    for child in block.blocks():
        if child.keyword != "component":
            hint = (
                "（attr 是**参数库**的东西，想定里没有读者——要写就写到"
                " library/*.txt 的型号里）"
                if child.keyword == "attr"
                else ""
            )
            raise ScenarioError(
                f"platform_type 里不认识的子块 {child.keyword!r}"
                f"（只允许 component）{hint}",
                child.line,
            )


def _resolve(
    params: ParamSet,
    values: Mapping[str, Any],
    owner: str,
    fallback_line: int,
    lines: Mapping[str, int] | None = None,
) -> dict[str, Any]:
    """解析参数，把 ``ParamError`` 统一成 ``ScenarioError``。

    统一成一种异常，调用方（lint 工具、装配层）只要捕一个就能拿到全部
    想定层面的错误——否则每加一种校验都要去改调用点的 except 列表。

    未知参数优先报**出错那一行**：拼写错误在几十行的块里，块首行号帮不上忙。
    """
    try:
        return params.resolve(values, owner=owner)
    except ParamError as exc:
        line = fallback_line
        # 异常带着出错的参数名，据此回退到那一行；缺参数之类的错误没有
        # 对应行，就用块首行号
        if lines:
            for name in exc.names:
                if name in lines:
                    line = lines[name]
                    break
        raise ScenarioError(str(exc), line) from exc


def _read_zone(spec: ScenarioSpec, block: Block) -> None:
    if not block.names:
        raise ScenarioError("zone 后面缺少战区名", block.line)
    if spec.zone(block.name) is not None:
        raise ScenarioError(
            f"战区 {block.name!r} 重复定义——重名会静默覆盖", block.line
        )

    values, lines = _collect_params(
        block, owner=f"zone {block.name}", coord_keys=("anchor",)
    )
    resolved = _resolve(
        ZONE_PARAMS, values, f"zone {block.name}", block.line, lines
    )

    anchor = resolved["anchor"]
    if not isinstance(anchor, CoordValue):
        raise ScenarioError(f"战区 {block.name} 的 anchor 必须是坐标", block.line)
    if anchor.form != "latlng":
        raise ScenarioError(
            f"战区 {block.name} 的 anchor 必须用 latlng——战区锚点定义了局部平面"
            "网格的原点，用网格坐标没有意义",
            block.line,
        )

    spec.zones.append(
        ZoneDecl(
            name=block.name,
            anchor=anchor,
            radius_m=float(resolved["radius"]),
            resolution_m=float(resolved["resolution"]),
            layers=int(resolved["layers"]),
            terrain=str(resolved["terrain"]),
            seed=int(resolved["seed"]),
            line=block.line,
        )
    )


def _read_platform(spec: ScenarioSpec, block: Block) -> None:
    if len(block.names) != 2:
        raise ScenarioError(
            f"platform 需要「实例名 类型名」两个名字，实际给了 {len(block.names)} 个"
            f"（如 platform SAM_1 SAM_BATTALION）",
            block.line,
        )
    instance_name, type_name = block.names

    values, _ = _collect_params(
        block, owner=f"platform {instance_name}", coord_keys=("position",)
    )

    position = values.pop("position", None)
    if position is not None and not isinstance(position, CoordValue):
        raise ScenarioError(
            f"平台 {instance_name} 的 position 需要坐标"
            f"（latlng 39.95 116.35 或 hex 1204 883 @ layer 0）",
            block.line,
        )

    heading = 0.0
    if "heading" in values:
        raw = values.pop("heading")
        try:
            heading = float(Params.angle(0.0).convert(*_split_unit(raw), owner="heading"))
        except ParamError as exc:
            raise ScenarioError(f"平台 {instance_name}：{exc}", block.line) from exc

    decision = str(values.pop("decision", "") or "")
    formation = str(values.pop("formation", "") or "")

    spec.platforms.append(
        PlatformDecl(
            name=instance_name,
            type_name=type_name,
            position=position,
            heading_deg=heading,
            decision=decision,
            formation=formation,
            attrs=values,
            line=block.line,
        )
    )


def _read_decision(spec: ScenarioSpec, block: Block) -> None:
    if not block.names:
        raise ScenarioError("decision 后面缺少配置名", block.line)
    if block.name in spec.decisions:
        raise ScenarioError(
            f"决策配置 {block.name!r} 重复定义"
            f"（第一次在第 {spec.decisions[block.name].line} 行）",
            block.line,
        )

    values, lines = _collect_params(block, owner=f"decision {block.name}")
    resolved = _resolve(
        DECISION_PARAMS, values, f"decision {block.name}", block.line, lines
    )

    if block.parent is not None:
        # 决策配置继承看着省事，实际会让"这份配置到底问了哪个模型"变成
        # 沿着链往上找的活。想定文件就几十行，复制一遍更清楚。
        raise ScenarioError(
            f"决策配置 {block.name} 不支持继承（写的是 : {block.parent}）——"
            "配置项不多，直接写全更清楚",
            block.line,
        )

    profile = DecisionProfile(
        name=block.name,
        provider=str(resolved["provider"]),
        model=str(resolved["model"]),
        endpoint=str(resolved["endpoint"]),
        api_key_env=str(resolved["api_key_env"]),
        timeout_us=int(resolved["timeout"]),
        cadence_us=int(resolved["cadence"]),
        fallback=str(resolved["fallback"]),
        max_context=int(resolved["max_context"]),
        temperature=float(resolved["temperature"]),
        prompt=str(resolved["prompt"]),
        line=block.line,
    )

    if profile.uses_llm and not profile.model:
        raise ScenarioError(
            f"决策配置 {block.name} 的 provider 是 llm，但没有写 model", block.line
        )
    if profile.uses_llm and not profile.api_key_env:
        raise ScenarioError(
            f"决策配置 {block.name} 用大模型但没写 api_key_env——"
            "想定里只写环境变量名，密钥本身不要进文件",
            block.line,
        )
    if profile.fallback and (
        profile.fallback not in spec.decisions and profile.fallback not in PROVIDERS
    ):
        # 前向引用是允许的（后面才定义的配置也能被引用），所以这里只做
        # "不是内置 provider 就记为待校验"，真正的检查放在 validate()
        pass

    spec.decisions[block.name] = profile


def _read_simulation(spec: ScenarioSpec, block: Block) -> None:
    if block.names:
        raise ScenarioError(
            f"simulation 块不带名字，多出了 {'、'.join(block.names)}", block.line
        )
    values, lines = _collect_params(block, owner="simulation")
    resolved = _resolve(
        SIMULATION_PARAMS, values, "simulation", block.line, lines
    )
    spec.settings = SimulationSettings(
        max_step_us=int(resolved["max_step"]),
        seed=int(resolved["seed"]),
        name=str(resolved["name"]),
    )


# -- 编队与指挥关系 ------------------------------------------------------

def _read_formation(spec: ScenarioSpec, block: Block) -> None:
    """``formation`` 块：行政隶属（第一层）。

    父级用块头冒号表示，与 ``component_type X : Y`` 同一套写法::

        formation RED_BDE
            side red
            echelon 旅
        end_formation

        formation RED_1BN : RED_BDE
            side red
            echelon 营
        end_formation

    为什么**不做嵌套**（原来设计过 ``subordinate`` 块）：嵌套要求每层都
    配对 ``end_``，四层编队要写八个结束标记，错一个就报"结束标记不匹配"。
    平铺写法每个编队一个 ``end_formation``，配不错；而且天然支持前向
    引用——先写连、后写营也认得出来。
    """
    if not block.names:
        raise ScenarioError("formation 后面缺少编队名", block.line)
    if len(block.names) > 1:
        raise ScenarioError(
            f"formation 只接一个编队名，多出了 {'、'.join(block.names[1:])}",
            block.line,
        )

    name = block.names[0]
    values, lines = _collect_params(block, owner=f"formation {name}")
    resolved = _resolve(FORMATION_PARAMS, values, f"formation {name}", block.line, lines)

    if not resolved["side"]:
        raise ScenarioError(
            f"编队 {name} 必须声明阵营（side red / side blue）——"
            "阵营决定谁能配属它",
            block.line,
        )

    spec.formations.append(
        FormationDecl(
            name=name,
            side=str(resolved["side"]),
            echelon=str(resolved["echelon"]),
            parent=block.parent,
            line=block.line,
        )
    )


#: ``command`` 块里允许出现的指令词。
_COMMAND_ACTIONS = frozenset({"attach", "direct", "coordinate"})


def _read_command(spec: ScenarioSpec, block: Block) -> None:
    """``command`` 块：作战调整（第二、三层）。

    块体里是**赋值行**，一行多值::

        attach RED_1_1 to RED_2BN from 30 min until 8 h
        direct RED_BDE to RED_RECON_1
        coordinate RED_1BN with RED_2BN

    语法层把它们拆成 ``("RED_1_1", "to", "RED_2BN", "from", WithUnit(...), …)``，
    这里负责解释——**语法的归语法、语义的归语义**。
    """
    if block.names:
        raise ScenarioError(
            f"command 块不带名字，多出了 {'、'.join(block.names)}", block.line
        )

    for assignment in block.assignments():
        action = assignment.key
        if action not in _COMMAND_ACTIONS:
            raise ScenarioError(
                f"command 块里不认识的指令 {action!r}"
                f"（可用：attach / direct / coordinate）",
                assignment.line,
                assignment.column,
            )

        values = list(_flatten(assignment.value))
        # 动作词后面紧跟两个名字，中间可以插一个连接词（to / with）
        names, rest = _split_action_values(values, action, assignment)
        window_start, window_end = _read_window(rest, action, assignment)

        if action == "attach":
            spec.attachments.append(AttachmentDecl(
                unit=names[0], parent=names[1],
                start_us=window_start, until_us=window_end,
                line=assignment.line,
            ))
        elif action == "direct":
            spec.directives.append(DirectiveDecl(
                superior=names[0], subordinate=names[1],
                start_us=window_start, until_us=window_end,
                line=assignment.line,
            ))
        else:
            spec.coordinations.append(CoordinationDecl(
                a=names[0], b=names[1],
                start_us=window_start, until_us=window_end,
                line=assignment.line,
            ))


#: ``network`` 块里允许出现的指令词。
_NETWORK_ACTIONS = frozenset({"member", "link"})


def _read_network(spec: ScenarioSpec, block: Block) -> None:
    """``network`` 块：一个通信网的成员与链路（§5.15）。

    块体里是**赋值行**，一行多值::

        network BLUE_NET
            member AWACS_1 AWACS_2 SAM_1
            link AWACS_1 AWACS_2
            link AWACS_2 SAM_1
        end_network

    语义（用户裁定，原话）：

    * "**所有加入通信网中的实体，隐形形成拓扑**" ⇒ ``member`` 收人，
      拓扑由成员与链路**算出来**（默认全连通，见下）；
    * "**全连通，但拓扑上可达**" ⇒ **不写 ``link`` 就是全连通**（同 AFSIM 的
      ``network_name`` 默认：同网成员两两有边）；
    * "**如果想定中有一个实体单边向通信网中的某个实体通信的内容，则该实体
      也加入同一通信网**" ⇒ 想定里谁给网内成员发过信，谁就自动入网。这一步
      **在装配期由报文发送侧落成员表**（见 :meth:`ScenarioSpec.network_of`
      与 §5.15），语法层不做（那时还不知道谁会发信）。

    ★ 为什么成员写**平台实例名**而不是实体 ID：实体 ID 是装配期才分配的，
    想定是给人看的。名字 → ID 的解析放在装配层（同 ``formation`` 的写法）。

    ★ 为什么一张网**只能有一个 ``end_network`` 块**：两张同名网会被静默合并，
    而症状是"我明明分了两个网，它们却通了"。成员写两行却是**允许**的——
    ``member A B`` 后接 ``member C D`` 是"名单接着写"，不是两张网。
    """
    if not block.names:
        raise ScenarioError("network 后面缺少网名", block.line)
    if len(block.names) > 1:
        raise ScenarioError(
            f"network 只接一个网名，多出了 {'、'.join(block.names[1:])}",
            block.line,
        )

    name = block.names[0]
    if any(net.name == name for net in spec.networks):
        raise ScenarioError(
            f"通信网 {name!r} 重复定义——两张同名网会被静默合并，"
            "而症状是『我分了两个网，它们却通了』",
            block.line,
        )

    members: list[str] = []
    links: list[tuple[str, str]] = []
    for assignment in block.assignments():
        action = assignment.key
        if action not in _NETWORK_ACTIONS:
            raise ScenarioError(
                f"network 块里不认识的指令 {action!r}（可用：member / link）",
                assignment.line,
                assignment.column,
            )
        values = list(_flatten(assignment.value))
        if action == "member":
            if not values:
                raise ScenarioError(
                    "member 后面缺少平台名（如 member AWACS_1 SAM_1）",
                    assignment.line,
                    assignment.column,
                )
            for item in values:
                if not isinstance(item, str):
                    raise ScenarioError(
                        f"member 需要平台名，读到 {item!r}"
                        "（名字不能加引号，也不能是数字）",
                        assignment.line,
                        assignment.column,
                    )
                if item not in members:
                    members.append(item)
        else:
            pair = _network_link(values, assignment)
            if pair not in links:
                links.append(pair)

    spec.networks.append(
        NetworkDecl(
            name=name,
            members=tuple(members),
            links=tuple(links),
            line=block.line,
        )
    )


def _network_link(
    values: list[object], assignment: Assignment
) -> tuple[str, str]:
    """``link A B`` 的两个端点。中间的连接词 ``to`` 可有可无。

    ★ **有向**（``(A, B)`` 表示"A 能发给 B"），照 AFSIM 的
    ``comm_link_list`` 原文："The order specified matters, as a link is only
    created from the first comm specified, to the second."
    """
    names: list[str] = []
    for item in values:
        if isinstance(item, str) and item in _CONNECTORS:
            continue
        if not isinstance(item, str):
            raise ScenarioError(
                f"link 需要平台名，读到 {item!r}（名字不能加引号）",
                assignment.line,
                assignment.column,
            )
        names.append(item)
        if len(names) == 2:
            break
    if len(names) < 2:
        raise ScenarioError(
            f"link 需要两个平台名，只给了 {len(names)} 个（如 link AWACS_1 SAM_1）",
            assignment.line,
            assignment.column,
        )
    return names[0], names[1]


def _flatten(value: object) -> tuple[object, ...]:
    """一行的值。单值就是它自己，多值是元组。"""
    if isinstance(value, tuple):
        return value
    return (value,)

#: 允许出现的连接词。写出来是为了读着顺，语义上可有可无。
_CONNECTORS = frozenset({"to", "with", "under"})


def _split_action_values(
    values: list[object], action: str, assignment: Assignment
) -> tuple[tuple[str, str], list[object]]:
    """从一行里取出两个目标名，剩下的留给时间子句。"""
    names: list[str] = []
    index = 0
    while index < len(values) and len(names) < 2:
        item = values[index]
        if isinstance(item, str) and item in _CONNECTORS:
            index += 1
            continue
        if not isinstance(item, str):
            raise ScenarioError(
                f"{action} 需要编队名，读到 {item!r}（名字不能加引号）",
                assignment.line,
                assignment.column,
            )
        names.append(item)
        index += 1

    if len(names) < 2:
        raise ScenarioError(
            f"{action} 需要两个编队名，只给了 {len(names)} 个"
            f"（如 {action} RED_1_1 {'to' if action != 'coordinate' else 'with'} RED_2BN）",
            assignment.line,
            assignment.column,
        )
    return (names[0], names[1]), values[index:]


def _read_window(
    rest: list[object], action: str, assignment: Assignment
) -> tuple[int, int | None]:
    """解析 ``from 30 min`` / ``until 8 h`` 子句。

    时间是**相对想定起点**的时长，不支持绝对时刻——想定语言里没有
    真实世界时钟。
    """
    start = 0
    end: int | None = None
    seen: set[str] = set()
    index = 0

    while index < len(rest):
        keyword = rest[index]
        if not isinstance(keyword, str) or keyword not in ("from", "until"):
            raise ScenarioError(
                f"{action} 的时间子句只认 from / until，读到 {keyword!r}"
                f"（正确写法：{action} … from 30 min until 8 h）",
                assignment.line,
                assignment.column,
            )
        if keyword in seen:
            raise ScenarioError(
                f"{action} 的 {keyword!r} 写了两次", assignment.line, assignment.column
            )
        seen.add(keyword)

        if index + 1 >= len(rest):
            raise ScenarioError(
                f"{keyword!r} 后面缺少时长（如 {keyword} 30 min）",
                assignment.line,
                assignment.column,
            )
        moment = _duration_us(rest[index + 1], keyword, assignment)
        if keyword == "from":
            start = moment
        else:
            end = moment
        index += 2

    if end is not None and end <= start:
        raise ScenarioError(
            f"until 必须晚于 from（from={start / 60_000_000:g} min，"
            f"until={end / 60_000_000:g} min）",
            assignment.line,
            assignment.column,
        )
    return start, end


def _duration_us(value: object, keyword: str, assignment: Assignment) -> int:
    """时长 → 微秒。走统一的单位表，**不经浮点**（§3.8.3）。"""
    bare, unit = _split_unit(value)
    try:
        return int(_DURATION.convert(bare, unit, owner=keyword))
    except Exception as exc:
        raise ScenarioError(
            f"{keyword} 的时长 {value!r} 无法解析：{exc}",
            assignment.line,
            assignment.column,
        ) from exc


# -- 值归一化 ------------------------------------------------------------

def _name_hint(name: str, known: "set[str] | dict[str, int]") -> str:
    """拼错时给建议。想定里打错一个字母而不报错，比报错糟糕得多。"""
    from difflib import get_close_matches

    close = get_close_matches(name, list(known), n=1)
    return f"（是否想写 {close[0]!r}？）" if close else ""


def _names_in(tree: "FormationTree") -> set[str]:
    return {node.name for node in tree}


def _split_unit(value: object) -> tuple[object, str | None]:
    if isinstance(value, WithUnit):
        return value.value, value.unit
    return value, None


def _collect_params(
    block: Block, *, owner: str, coord_keys: Sequence[str] = ()
) -> tuple[dict[str, Any], dict[str, int]]:
    """把一个块里的赋值行收成一个参数表，外加**每个参数的行号**。

    行号单独带出来，是为了让"未知参数"这类报错能指到出错的那一行——
    拼写错误在几十行的块里，块起始行号帮不上什么忙。

    处理两件事：

    **① ``key v1 name=v2`` 等价于两行。** 内联标注的标签名直接当参数名用，
    这样 ``terrain procedural seed=42`` 里的 ``seed`` 与单独写一行 ``seed 42``
    完全一样，语义层不必为它单独开一条规则。

    **② 重复参数直接报错。** 同一个键写两次时静默取后者，会让人对着
    "明明改了却没生效"发呆——而原因只是文件里还有一行更靠后的同名参数。
    """
    values: dict[str, Any] = {}
    lines: dict[str, int] = {}

    for assignment in block.assignments():
        value, tags = _split_tags(assignment.value)

        if value is not None:
            if assignment.key in coord_keys and _is_pair(value):
                value = CoordValue(
                    "latlng", lat=float(value[0]), lng=float(value[1])
                )
            _put(values, lines, assignment, assignment.key, value, owner)

        for tag, tagged_value in tags.items():
            _put(values, lines, assignment, tag, tagged_value, owner)

    return values, lines


def _put(
    values: dict[str, Any],
    lines: dict[str, int],
    assignment: Assignment,
    key: str,
    value: object,
    owner: str,
) -> None:
    if key in values:
        raise ScenarioError(
            f"{owner}: 参数 {key!r} 写了两次（第 {assignment.line} 行又写了一次）——"
            "重复的取值会被静默丢掉一个",
            assignment.line,
            assignment.column,
        )
    values[key] = value
    lines[key] = assignment.line


def _split_tags(value: object) -> tuple[object | None, dict[str, object]]:
    """把 ``(主值, name=v, name2=v2)`` 拆成主值与标注表。

    **单元素的元组要原样留着**：``('山地',)`` 只可能来自列表字面量
    ``[山地]``——"单值 + 标签"至少有两个元素（值本身 + 至少一个
    ``name=v``）。以前这里把长度为 1 的元组也拆成裸值，于是想定里写
    ``blocked_terrain [山地]`` 到参数层变成了字符串 ``'山地'``，
    ``ParamKind.STRINGS`` 直接报"需要列表"；而写 ``[山地, 水域]``
    反而没事——**只有"只封一种地貌"这种最常见的写法会翻车**。

    **空元组也原样留着**：``[]`` 是"我明确要一个空列表"，不能与"这个
    参数没给值"混为一谈。混了的话（两者都返回 ``None``），想用 ``[]``
    清空一个非空默认值就永远做不到，而且**不会报错**——参数静默地保持
    默认值，正是这个项目里最难查的那类缺陷。
    """
    if isinstance(value, TaggedValue):
        return None, {value.tag: value.value}
    if not isinstance(value, tuple):
        return value, {}

    main: list[object] = []
    tags: dict[str, object] = {}
    for item in value:
        if isinstance(item, TaggedValue):
            tags[item.tag] = item.value
        else:
            main.append(item)

    if not main:
        # 长度 0 的元组 = 列表字面量 []；其余情形是"一行只有 name=v"
        return ((), tags) if len(value) == 0 else (None, tags)
    if len(main) == 1:
        # 整个值只有一个元素：它来自列表字面量 [x]，保留元组；
        # 否则是"单值 + 标签"，拆出标量。
        return (tuple(main), tags) if len(value) == 1 else (main[0], tags)
    return tuple(main), tags


def _is_pair(value: object) -> bool:
    return (
        isinstance(value, tuple)
        and len(value) == 2
        and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value)
    )


__all__ = [
    "DECISION_PARAMS",
    "INSTANCE_ONLY_KEYS",
    "PROVIDERS",
    "SIMULATION_PARAMS",
    "ZONE_PARAMS",
    "DecisionProfile",
    "NetworkDecl",
    "PlatformDecl",
    "ScenarioError",
    "ScenarioSpec",
    "SimulationSettings",
    "ZoneDecl",
    "build_spec",
    "load_scenario",
]
