"""类型系统：原型 + 差量实例，以及组件的代码层注册。

两个名字空间，各管一件事
------------------------
| 层面 | 管什么 | 声明在哪 |
|---|---|---|
| **代码层** | 行为是什么算法 | Python 类 + ``@register_component("NAME")`` |
| **想定层** | 参数取什么值 | ``component_type`` / ``platform_type`` 块 |

两层**不重叠**，这正是"一套算法两个型号"和"一组参数两套算法"都能表达的原因。
绑定关系是**按名字对齐**的：想定层定义 ``component_type HEX_SEARCH_RADAR``
时，代码层若注册了同名组件，两者就绑在一起。

沿继承链向上找实现
------------------
想定层可以定义纯参数模板：``component_type MY_RADAR : HEX_SEARCH_RADAR``——
代码层并没有 ``MY_RADAR`` 这个类。实例化时沿继承链向上找**第一个**在
代码注册表里的名字，用那个类，参数则用 ``MY_RADAR`` 的（已合并父链差量）。

找不到实现类就明确报错，不猜、不回退到父类静默构造——
报错信息里带上继承链和已注册的名字（含拼写建议），照着一行就能改。

参数合并是**分层覆盖**，不是合并
--------------------------------
三个来源，后者覆盖前者：

1. 实现类的 ``PARAMS`` 默认值
2. ``component_type`` 继承链上的参数（叶覆盖根）
3. 平台里挂载组件时写的差量（``component sensor X`` 块内）

列表参数也是整体替换，不合并。``modes [SARH, IR]`` 覆盖 ``modes [GUN]`` 后
就是两个元素，不是三个——合并语义看似聪明，实际会让"父类加了个元素、
所有子类行为都变了"这种事极难追查。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from difflib import get_close_matches
from typing import Any, Callable, Iterator, Mapping, Sequence

from ..errors import ConfigurationError
from .params import ParamError, ParamSet

# ---------------------------------------------------------------------------
# 异常
# ---------------------------------------------------------------------------

class TypeDefinitionError(ConfigurationError):
    """类型定义的错误。统一基类，便于装配层一次性捕获并汇总报错。"""


class UnknownTypeError(TypeDefinitionError):
    """引用了未定义的类型。"""


class DuplicateTypeError(TypeDefinitionError):
    """重复定义同名类型。"""


class InheritanceCycleError(TypeDefinitionError):
    """继承成环。"""


class UnknownComponentError(TypeDefinitionError):
    """找不到组件实现类。"""


class ComponentBuildError(TypeDefinitionError):
    """组件构造时自身抛了异常。

    包一层是为了把"想定层的哪个平台、哪个槽"写进消息——原始异常来自
    组件代码，消息里只有它自己的上下文。
    """


class DuplicateComponentError(TypeDefinitionError):
    """重复注册同名组件类。"""


def _missing_base_hint(message: str) -> str:
    """给"父类型未定义"补上修法。

    最常见的成因不是拼错，而是**父类型写成了代码层的组件类名**：
    ``component_type RADAR_X : HEX_SEARCH_RADAR`` 里的 HEX_SEARCH_RADAR
    只是 ``@register_component`` 注册过的一个类，不是 ``component_type``。
    这两层是刻意分开的（§5.3：代码层管算法，想定层管参数），所以要用它
    的默认参数就得先声明一个**同名空块**把它引进类型表，再由它派生。

    这个坑在参数库里尤其容易踩：库里的型号往往按实物命名
    （``RADAR_SEARCH_S``），而代码层的类名是算法的名字
    （``HEX_SEARCH_RADAR``），两边不同名时第一反应就是直接继承类名。
    """
    return (
        f"{message}。注意 ``:`` 后面只能是**已声明的类型**——"
        "代码层用 @register_component 注册的类名不能直接当父类型。"
        "先声明一个同名空块把它引进来，再派生：\n"
        "    component_type <那个类名>\n"
        "    end_component_type\n"
        "    component_type <你的型号> : <那个类名>"
    )


def _check_name(name: str, what: str) -> None:
    if not name or not name.strip():
        raise TypeDefinitionError(f"{what}名不能为空")
    if any(c.isspace() for c in name):
        raise TypeDefinitionError(f"{what}名不能含空白：{name!r}")
    # 想定文件用的是块状文本语言，这几个字符会破坏词法
    for bad in ",:[]{}#":
        if bad in name:
            raise TypeDefinitionError(f"{what}名不能含 {bad!r}：{name!r}")


def _place_of(entry: Any) -> str:
    """一个已定义类型的出处。参数库来的说库名，想定来的说行号。"""
    return entry.origin or (f"第 {entry.line} 行" if entry.line else "位置未记录")


def _duplicate_message(
    what: str, name: str, existing: Any, origin: str, line: int
) -> str:
    """重名报错。

    这类报错的价值全在**"另一处在哪"**上：只说重了，作者得把想定从第一行
    查到末一行；说清另一处是参数库还是第几行，一眼就能决定改哪边。
    """
    current = origin or (f"第 {line} 行" if line else "位置未记录")
    message = (
        f"{what} {name!r} 重复定义：另一处在{_place_of(existing)}，"
        f"本处在{current}。重名会静默覆盖，必须显式改名"
    )
    if existing.origin:
        # 库已经给了这个型号的参数。想微调就该派生，这样从名字上就能看出
        # "这是改过的"——直接重定义会让"哪个值生效"变成需要查代码的事。
        message += (
            f"——参数库已经给了 {name} 的参数，想定里不该再定义一次；"
            f"要改就派生：{what} <新名> : {existing.name}"
        )
    return message


# ---------------------------------------------------------------------------
# 类型定义：存的是差量
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class ComponentType:
    """组件类型。``attrs`` 是与父类的**差量**，不是完整参数。"""

    name: str
    parent: str | None = None
    attrs: dict[str, Any] = field(default_factory=dict)
    line: int = 0
    #: 谁声明的。空串 = 想定里的 ``component_type`` 块；非空 = 参数库
    #: （形如 ``models/army.db 第 12 行``）。
    #:
    #: 用处只有一个，但很关键：重名时报错得说清**另一处在哪**。没有它，
    #: 报出来的是"组件类型 'AN_APG_77' 重复定义（第一次在第 0 行）"——
    #: 第 0 行是哪儿？作者会把想定从第一行查到末一行。
    origin: str = ""

    def __repr__(self) -> str:
        base = f" : {self.parent}" if self.parent else ""
        return f"<ComponentType {self.name}{base} {len(self.attrs)} 项差量>"


@dataclass(frozen=True, slots=True)
class ComponentSpec:
    """平台里挂载一个组件的声明。``attrs`` 同样是差量。"""

    slot: str
    type_name: str
    attrs: Mapping[str, Any] = field(default_factory=dict)
    line: int = 0

    def __post_init__(self) -> None:
        if not self.slot or not self.slot.strip():
            raise TypeDefinitionError("组件的 slot 不能为空")
        if not self.type_name or not self.type_name.strip():
            raise TypeDefinitionError(f"槽 {self.slot!r} 未指定组件类型")

    def __repr__(self) -> str:
        return f"<ComponentSpec {self.slot}={self.type_name} {len(self.attrs)} 项差量>"


@dataclass(slots=True)
class PlatformType:
    """平台类型。``attributes`` 是差量；``components`` 是槽 → 声明。"""

    name: str
    parent: str | None = None
    attributes: dict[str, Any] = field(default_factory=dict)
    components: dict[str, ComponentSpec] = field(default_factory=dict)
    line: int = 0
    #: 同 :attr:`ComponentType.origin`。
    origin: str = ""

    def __repr__(self) -> str:
        base = f" : {self.parent}" if self.parent else ""
        return (
            f"<PlatformType {self.name}{base} "
            f"{len(self.attributes)} 项属性 / {len(self.components)} 个组件>"
        )


@dataclass(frozen=True, slots=True)
class ResolvedPlatform:
    """平台类型的解析结果：属性与组件都已沿继承链合并。"""

    name: str
    attributes: Mapping[str, Any]
    components: Mapping[str, ComponentSpec]

    def slots(self) -> list[str]:
        return sorted(self.components)


@dataclass(frozen=True, slots=True)
class ResolvedComponent:
    """组件实例化后的完整描述——**参数已含默认值**，组件不必再查表。

    与 :class:`ComponentSpec` 的区别：那个是"想定写了什么"（差量），
    这个是"这一份实例最终是什么"（全量）。
    """

    slot: str
    type_name: str
    impl_class: type
    params: Mapping[str, Any]

    def __getitem__(self, name: str) -> Any:
        return self.params[name]

    def get(self, name: str, default: Any = None) -> Any:
        return self.params.get(name, default)

    def __repr__(self) -> str:
        return (
            f"<ResolvedComponent {self.slot}={self.type_name} "
            f"({self.impl_class.__name__}) {len(self.params)} 个参数>"
        )


# ---------------------------------------------------------------------------
# 类型表
# ---------------------------------------------------------------------------

class TypeRegistry:
    """组件类型与平台类型的表。纯声明，不依赖引擎、地图或实体。

    继承链**在实例化时解析并缓存**，不是定义时。这样才支持"先写子类、
    后写父类"的想定写法，也让继承成环这类错误定位在真正用到的时刻，
    而不是在一堆类型定义中间报一句没头没尾的话。
    """

    __slots__ = ("_components", "_platforms", "_cache")

    def __init__(self) -> None:
        self._components: dict[str, ComponentType] = {}
        self._platforms: dict[str, PlatformType] = {}
        #: 解析结果缓存。任何定义变动都清空——比逐条失效简单，也不会漏。
        self._cache: dict[tuple[str, str], dict[str, Any]] = {}

    # -- 定义 --------------------------------------------------------------

    def define_component_type(
        self,
        name: str,
        *,
        parent: str | None = None,
        attrs: Mapping[str, Any] | None = None,
        line: int = 0,
        origin: str = "",
    ) -> ComponentType:
        _check_name(name, "组件类型")
        if name in self._components:
            raise DuplicateTypeError(
                _duplicate_message("组件类型", name, self._components[name], origin, line)
            )
        entry = ComponentType(name, parent, dict(attrs or {}), line, origin)
        self._components[name] = entry
        self._cache.clear()
        return entry

    def define_platform_type(
        self,
        name: str,
        *,
        parent: str | None = None,
        attributes: Mapping[str, Any] | None = None,
        line: int = 0,
        origin: str = "",
    ) -> PlatformType:
        _check_name(name, "平台类型")
        if name in self._platforms:
            raise DuplicateTypeError(
                _duplicate_message("平台类型", name, self._platforms[name], origin, line)
            )
        entry = PlatformType(name, parent, dict(attributes or {}), {}, line, origin)
        self._platforms[name] = entry
        self._cache.clear()
        return entry

    def add_component(self, platform_name: str, spec: ComponentSpec) -> None:
        """给平台挂一个组件。

        同一平台块里同一槽写两次直接报错——想定作者的意图多半是一个槽
        写了两遍，而不是想在同一个槽上挂两个组件（那需要两个不同的槽名）。
        静默让后者覆盖前者，会丢掉一个组件而毫无提示。
        """
        platform = self.platform_type(platform_name)
        existing = platform.components.get(spec.slot)
        if existing is not None:
            raise DuplicateTypeError(
                f"平台类型 {platform_name} 的槽 {spec.slot!r} 重复定义："
                f"已有 {existing.type_name}，第 {spec.line} 行又写了 {spec.type_name}"
            )
        platform.components[spec.slot] = spec
        self._cache.clear()

    # -- 查询 --------------------------------------------------------------

    def find_component_type(self, name: str) -> ComponentType | None:
        return self._components.get(name)

    def find_platform_type(self, name: str) -> PlatformType | None:
        """查不到返回 ``None``（对应 :meth:`find_component_type`）。

        ``is not None`` 形式的判断要用它——:meth:`platform_type` 查不到会
        抛异常，拿它做"在不在"的判断就只能靠 ``try``。
        """
        return self._platforms.get(name)

    def component_type(self, name: str) -> ComponentType:
        entry = self._components.get(name)
        if entry is None:
            raise UnknownTypeError(self._unknown_message(name, self._components, "组件类型"))
        return entry

    def platform_type(self, name: str) -> PlatformType:
        entry = self._platforms.get(name)
        if entry is None:
            raise UnknownTypeError(self._unknown_message(name, self._platforms, "平台类型"))
        return entry

    def has_component_type(self, name: str) -> bool:
        return name in self._components

    def has_platform_type(self, name: str) -> bool:
        return name in self._platforms

    def component_names(self) -> list[str]:
        return sorted(self._components)

    def platform_names(self) -> list[str]:
        return sorted(self._platforms)

    def _unknown_message(
        self, name: str, table: Mapping[str, Any], what: str
    ) -> str:
        close = get_close_matches(name, list(table), n=1, cutoff=0.7)
        hint = f"（是否想写 {close[0]!r}？）" if close else ""
        if not table:
            return f"未定义{what} {name!r}{hint}——本文件里一个{what}都没有"
        available = "、".join(sorted(table))
        return f"未定义{what} {name!r}{hint}。已定义的有：{available}"

    # -- 继承链 ------------------------------------------------------------

    def component_chain(self, name: str) -> list[str]:
        """从 ``name`` 到根的类型名，**叶在前**。成环时报错。"""
        return self._chain(name, self._components, "组件类型")

    def platform_chain(self, name: str) -> list[str]:
        return self._chain(name, self._platforms, "平台类型")

    def _chain(
        self, name: str, table: Mapping[str, Any], what: str
    ) -> list[str]:
        chain: list[str] = []
        seen: set[str] = set()
        current: str | None = name

        while current is not None:
            if current in seen:
                # 把环画出来，否则作者要在几十行定义里自己找
                cycle = " → ".join([*chain[chain.index(current):], current])
                raise InheritanceCycleError(f"{what}继承成环：{cycle}")
            seen.add(current)

            entry = table.get(current)
            if entry is None:
                if not chain:
                    # 起点就不存在：给出可用名单，比"自己继承自己"清楚
                    raise UnknownTypeError(
                        self._unknown_message(current, table, what)
                    )
                parent_of = chain[-1]
                raise UnknownTypeError(
                    f"{parent_of!r} 继承自未定义的{what} {current!r}"
                )
            chain.append(current)
            current = entry.parent

        return chain

    # -- 解析（带缓存） ----------------------------------------------------

    def resolve_component_attrs(self, name: str) -> dict[str, Any]:
        """组件类型链上的参数，**根在前、叶覆盖**。"""
        key = ("component", name)
        cached = self._cache.get(key)
        if cached is not None:
            return dict(cached)

        merged: dict[str, Any] = {}
        for type_name in reversed(self.component_chain(name)):
            merged.update(self._components[type_name].attrs)

        self._cache[key] = merged
        return dict(merged)

    def resolve_platform(self, name: str) -> ResolvedPlatform:
        """平台属性与组件槽都沿继承链合并。

        组件按**槽**合并：子平台写了同名槽就覆盖父平台的，没写的继承下来。
        这样"通用营部 + 特定型号"的写法才成立。
        """
        key = ("platform_attrs", name)
        cached = self._cache.get(key)
        if cached is None:
            merged: dict[str, Any] = {}
            for type_name in reversed(self.platform_chain(name)):
                merged.update(self._platforms[type_name].attributes)
            self._cache[key] = merged
            cached = merged
        attributes = dict(cached)

        components: dict[str, ComponentSpec] = {}
        for type_name in reversed(self.platform_chain(name)):
            components.update(self._platforms[type_name].components)

        return ResolvedPlatform(name, attributes, components)

    # -- 诊断 --------------------------------------------------------------

    def statistics(self) -> dict[str, int]:
        return {
            "component_types": len(self._components),
            "platform_types": len(self._platforms),
            "cached_resolutions": len(self._cache),
        }

    def validate(self) -> list[str]:
        """全量自检：父类型存在、无环、组件类型引用可解析。测试与装配用。"""
        problems: list[str] = []

        for name in self._components:
            try:
                self.component_chain(name)
            except TypeDefinitionError as exc:
                problems.append(str(exc))

        for name in self._platforms:
            try:
                chain = self.platform_chain(name)
            except TypeDefinitionError as exc:
                problems.append(str(exc))
                continue
            for type_name in chain:
                for slot, spec in self._platforms[type_name].components.items():
                    if not spec.type_name:
                        problems.append(f"{name} 的槽 {slot} 未指定类型")
                    if slot != spec.slot:
                        problems.append(f"{name} 的槽 {slot} 与声明 {spec.slot} 不一致")

        return problems

    def clear_cache(self) -> None:
        self._cache.clear()

    def __repr__(self) -> str:
        return (
            f"<TypeRegistry {len(self._components)} 组件类型 / "
            f"{len(self._platforms)} 平台类型>"
        )


# ---------------------------------------------------------------------------
# 代码层：组件类注册
# ---------------------------------------------------------------------------

class ComponentRegistry:
    """组件类型名 → Python 类的表。

    想定里**不写 Python 路径**（``milsim.models.percep.HexSearchRadar`` 那种）。
    那是安全漏洞——想定文件会变成任意代码执行入口——也会让想定和代码版本
    强耦合，换个模块路径所有想定都得改。名字出现在代码里：::

        @register_component("HEX_SEARCH_RADAR")
        class HexSearchRadar(Sensor): ...
    """

    __slots__ = ("_classes",)

    def __init__(self) -> None:
        self._classes: dict[str, type] = {}

    def register(
        self, name: str, component_class: type, *, override: bool = False
    ) -> None:
        """注册一个组件类。

        默认**不允许覆盖**：两个模块注册同名组件时静默覆盖，会表现为
        "我的实现明明改了却不起作用"——极难定位。要覆盖必须显式写
        ``override=True``，让意图留在代码里。
        """
        _check_name(name, "组件")
        if not isinstance(component_class, type):
            raise DuplicateComponentError(
                f"{name} 的注册对象不是类：{component_class!r}"
            )

        existing = self._classes.get(name)
        if existing is not None and not override:
            raise DuplicateComponentError(
                f"组件名 {name!r} 已被 {existing.__module__}.{existing.__qualname__} "
                f"占用，又想注册 {component_class.__module__}."
                f"{component_class.__qualname__}——确实要替换请传 override=True"
            )

        self._classes[name] = component_class

    def get(self, name: str) -> type | None:
        return self._classes.get(name)

    def contains(self, name: str) -> bool:
        return name in self._classes

    def names(self) -> list[str]:
        return sorted(self._classes)

    def resolve_for(self, type_chain: Sequence[str]) -> type:
        """沿继承链找**第一个**已注册的实现类。

        想定层允许定义代码里不存在的纯参数模板
        （``component_type MY_RADAR : HEX_SEARCH_RADAR``），这时沿链向上
        找到 ``HEX_SEARCH_RADAR`` 的类来用。
        """
        for name in type_chain:
            found = self._classes.get(name)
            if found is not None:
                return found

        leaf = type_chain[0] if type_chain else "?"
        chain = " → ".join(type_chain)
        close = get_close_matches(leaf, list(self._classes), n=1, cutoff=0.6)
        hint = f"（是否想写 {close[0]!r}？）" if close else ""
        raise UnknownComponentError(
            f"{leaf!r} 没有实现类{hint}。继承链：{chain}。"
            f"已注册的组件：{'、'.join(self.names()) or '（空）'}"
        )

    def validate_class(self, name: str) -> list[str]:
        """检查某个已注册类的 ``PARAMS`` 声明是否规范。"""
        cls = self._classes.get(name)
        if cls is None:
            return [f"未注册的组件名 {name!r}"]
        try:
            ParamSet.from_class(cls, owner=name)
        except ParamError as exc:
            return [str(exc)]
        return []

    def clear(self) -> None:
        self._classes.clear()

    def __len__(self) -> int:
        return len(self._classes)

    def __contains__(self, name: str) -> bool:
        return name in self._classes

    def __iter__(self) -> Iterator[str]:
        return iter(self.names())

    def __repr__(self) -> str:
        return f"<ComponentRegistry {len(self._classes)} 个组件类>"


#: 全局默认注册表。装饰器把类注册到这里，装配层也用它。
#:
#: 不做自动扫描注册是刻意的：显式 import 让依赖可见、加载顺序确定，也不会
#: 因为某个插件模块有语法错误就连带整个组件注册失败。需要插件目录时由装配层
#: 显式扫描——扫描的代码只有一处，出问题一眼能看到。
_DEFAULT_REGISTRY = ComponentRegistry()


def default_registry() -> ComponentRegistry:
    return _DEFAULT_REGISTRY


def reset_default_registry() -> None:
    """清空全局注册表。测试隔离用——生产代码不该调它。"""
    _DEFAULT_REGISTRY.clear()


def register_component(
    name: str, *, override: bool = False
) -> Callable[[type], type]:
    """把组件类登记到全局注册表。

    支持两种用法，后者便于使用者从框架的参考实现派生出自己的型号::

        @register_component("HEX_SEARCH_RADAR")
        class HexSearchRadar(Sensor): ...

        @register_component("PHASED_ARRAY_RADAR")
        class PhasedArrayRadar(HexSearchRadar): ...
    """

    def decorator(component_class: type) -> type:
        _DEFAULT_REGISTRY.register(name, component_class, override=override)
        component_class.COMPONENT_NAME = name      # 便于调试与报错
        return component_class

    return decorator


# ---------------------------------------------------------------------------
# 代码层：平台类注册（§5.6）
# ---------------------------------------------------------------------------

#: 平台类型链的**隐式根**。任何 ``platform_type`` 即使不写父类型，也总能沿
#: 链落到这个名字上的类——它提供基础参数表（阵营、决策配置）。
ROOT_PLATFORM = "PLATFORM"


class PlatformRegistry:
    """平台类型名 → Python 类的表。

    与 :class:`ComponentRegistry` 的**唯一结构差异**是隐式根
    （:data:`ROOT_PLATFORM`），而这不是为了省事：

    - 组件类型必须指向一个**具体算法**（``HEX_SEARCH_RADAR``），链上没有
      那个名字就说明型号名拼错了或空块忘了写——报错是对的。
    - 平台类型没有算法，行为在组件里。所有平台共享同一组基础参数，链上
      找不到具体类时落到根是**语义正确**的默认，不是兜底。

    兜底会不会掩盖拼错？不会：父类型存不存在由
    :meth:`TypeRegistry.platform_chain` 把关（未定义就报错）。这里只是
    "这条链不指向任何型号族"，而不是"这条链指错了"。
    """

    __slots__ = ("_classes",)

    def __init__(self) -> None:
        self._classes: dict[str, type] = {}

    def register(
        self, name: str, platform_class: type, *, override: bool = False
    ) -> None:
        _check_name(name, "平台")
        if not isinstance(platform_class, type):
            raise DuplicateComponentError(
                f"{name} 的注册对象不是类：{platform_class!r}"
            )

        existing = self._classes.get(name)
        if existing is not None and not override:
            raise DuplicateComponentError(
                f"平台名 {name!r} 已被 {existing.__module__}.{existing.__qualname__} "
                f"占用，又想注册 {platform_class.__module__}."
                f"{platform_class.__qualname__}——确实要替换请传 override=True"
            )
        self._classes[name] = platform_class

    def get(self, name: str) -> type | None:
        return self._classes.get(name)

    def contains(self, name: str) -> bool:
        return name in self._classes

    def names(self) -> list[str]:
        return sorted(self._classes)

    def resolve_for(self, type_chain: Sequence[str]) -> type:
        """沿继承链找**第一个**已注册的平台类，找不到落到根。"""
        for name in type_chain:
            found = self._classes.get(name)
            if found is not None:
                return found

        root = self._classes.get(ROOT_PLATFORM)
        if root is None:
            leaf = type_chain[0] if type_chain else "?"
            raise UnknownComponentError(
                f"{leaf!r} 找不到平台类，连根 {ROOT_PLATFORM!r} 都没有注册。"
                "是不是没有 import milsim.models？（平台基类在那里注册）"
            )
        return root

    def resolve_name(self, type_chain: Sequence[str]) -> str:
        """链上命中的注册名。用于报错时说清"参数表来自哪一层"。"""
        for name in type_chain:
            if name in self._classes:
                return name
        return ROOT_PLATFORM

    def validate_class(self, name: str) -> list[str]:
        cls = self._classes.get(name)
        if cls is None:
            return [f"未注册的平台名 {name!r}"]
        try:
            ParamSet.from_class(cls, owner=name)
        except ParamError as exc:
            return [str(exc)]
        return []

    def clear(self) -> None:
        self._classes.clear()

    def __len__(self) -> int:
        return len(self._classes)

    def __contains__(self, name: str) -> bool:
        return name in self._classes

    def __iter__(self) -> Iterator[str]:
        return iter(self.names())

    def __repr__(self) -> str:
        return f"<PlatformRegistry {len(self._classes)} 个平台类>"


_DEFAULT_PLATFORMS = PlatformRegistry()


def default_platforms() -> PlatformRegistry:
    return _DEFAULT_PLATFORMS


def reset_default_platforms() -> None:
    """清空全局平台注册表。测试隔离用——生产代码不该调它。"""
    _DEFAULT_PLATFORMS.clear()


def register_platform(
    name: str, *, override: bool = False
) -> Callable[[type], type]:
    """把平台类登记到全局平台注册表（§5.6）::

        @register_platform("PLATFORM")
        class Platform: ...

        @register_platform("ARMOR_HULL")
        class ArmorHull(Platform): ...
    """

    def decorator(platform_class: type) -> type:
        _DEFAULT_PLATFORMS.register(name, platform_class, override=override)
        platform_class.PLATFORM_NAME = name
        return platform_class

    return decorator


# ---------------------------------------------------------------------------
# 工厂：类型名 → 组件实例
# ---------------------------------------------------------------------------

class ComponentFactory:
    """把"槽 + 组件类型名 + 差量"变成组件实例。

    三层参数覆盖在 :meth:`build` 里一次完成，调用方（想定 Builder）只用
    一行，不必知道覆盖顺序。

    它也管**平台型**的参数（:meth:`resolved_platform_params`）。分成两个
    类看着更"单一职责"，但那会让"参数校验"这件事有两个入口——而两条入口
    的差别（一个走 ``ParamSet.resolve``，另一个忘了走）恰恰是本项目最防的
    那类错。平台与组件的参数解析必须是同一段代码。
    """

    __slots__ = ("components", "types", "platforms")

    def __init__(
        self,
        components: ComponentRegistry | None = None,
        types: TypeRegistry | None = None,
        platforms: PlatformRegistry | None = None,
    ) -> None:
        self.components = components if components is not None else _DEFAULT_REGISTRY
        self.types = types if types is not None else TypeRegistry()
        self.platforms = platforms if platforms is not None else _DEFAULT_PLATFORMS

    # -- 解析 --------------------------------------------------------------

    def resolve_class(self, type_name: str) -> type:
        """找到实现类，沿想定层的继承链向上找。"""
        if self.types.has_component_type(type_name):
            try:
                chain = self.types.component_chain(type_name)
            except UnknownTypeError as exc:
                raise UnknownTypeError(_missing_base_hint(str(exc))) from exc
        else:
            # 没写 component_type 块是允许的——直接用代码层的声明与默认值
            chain = [type_name]
        return self.components.resolve_for(chain)

    def param_set(self, type_name: str) -> ParamSet:
        """实现类声明的参数表。用于校验、生成文档、查看默认值。"""
        return ParamSet.from_class(self.resolve_class(type_name), owner=type_name)

    def resolved_params(self, type_name: str, overrides: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """完整参数：类默认值 ← 类型链 ← 挂载差量。暴露出来便于装配期校验。"""
        values: dict[str, Any] = {}
        if self.types.has_component_type(type_name):
            values.update(self.types.resolve_component_attrs(type_name))
        if overrides:
            values.update(overrides)
        return self.param_set(type_name).resolve(values, owner=type_name)

    # -- 构造 --------------------------------------------------------------

    def build(
        self,
        slot: str,
        type_name: str,
        overrides: Mapping[str, Any] | None = None,
        **kwargs: Any,
    ) -> Any:
        """构造组件实例。

        构造签名约定为 ``cls(*, spec=ResolvedComponent, **kwargs)``——
        组件拿到的是**含默认值的完整参数**，自己不必再查表、不必再判空。
        """
        impl = self.resolve_class(type_name)
        params = self.resolved_params(type_name, overrides)
        spec = ResolvedComponent(slot, type_name, impl, params)
        return impl(spec=spec, **kwargs)

    # -- 平台型（§5.6）-----------------------------------------------------

    def platform_class(self, platform_type_name: str) -> type:
        """平台类型链命中的平台类（找不到落到隐式根 ``PLATFORM``）。"""
        return self.platforms.resolve_for(self.types.platform_chain(platform_type_name))

    def platform_param_set(self, platform_type_name: str) -> ParamSet:
        """平台型的参数表。``owner`` 里带上**参数表来自哪个类**——
        平台类型落到根是常态，但落错了地方（本该指向某个型号族却没指向）
        只有把类名打出来才看得出来。"""
        cls = self.platform_class(platform_type_name)
        matched = self.platforms.resolve_name(
            self.types.platform_chain(platform_type_name)
        )
        return ParamSet.from_class(cls, owner=f"平台型 {platform_type_name}（{matched}）")

    def resolved_platform_params(
        self,
        platform_type_name: str,
        overrides: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """完整平台参数：类默认值 ← 平台类型链 ← 本次覆盖。

        与 :meth:`resolved_params` 同一套顺序、同一段 ``ParamSet.resolve``。
        """
        values: dict[str, Any] = {}
        values.update(self.types.resolve_platform(platform_type_name).attributes)
        if overrides:
            values.update(overrides)
        return self.platform_param_set(platform_type_name).resolve(
            values, owner=f"平台型 {platform_type_name}"
        )

    def build_for_platform(
        self, platform_type_name: str, **kwargs: Any
    ) -> dict[str, Any]:
        """把某个平台类型下的全部组件一次建好，返回槽 → 实例。

        逐个槽独立构造：某个槽失败时报错能指出是哪个槽，而不是把整个
        平台装配搞成一句"实例化失败"。
        """
        resolved = self.types.resolve_platform(platform_type_name)
        built: dict[str, Any] = {}
        for slot in resolved.slots():
            spec = resolved.components[slot]
            try:
                built[slot] = self.build(slot, spec.type_name, spec.attrs, **kwargs)
            except TypeDefinitionError:
                # 类型错误的message 里已经带了类型名与继承链，够定位了
                raise
            except ParamError as exc:
                raise ParamError(f"平台 {platform_type_name} 的 {slot} 槽：{exc}") from exc
            except Exception as exc:
                # 组件自己的 __init__ 抛的异常。不包一层的话，几十个槽里
                # 哪个炸的、炸在哪个平台上，全靠翻栈——而栈里是工厂内部调用，
                # 看不出想定层的对应关系
                raise ComponentBuildError(
                    f"平台 {platform_type_name} 的 {slot} 槽"
                    f"（{spec.type_name}）构造失败："
                    f"{type(exc).__name__}: {exc}"
                ) from exc
        return built

    def __repr__(self) -> str:
        return (
            f"<ComponentFactory {len(self.components)} 个组件类 / "
            f"{len(self.types)} 个想定类型>"
        )


__all__ = [
    "ComponentBuildError",
    "ComponentFactory",
    "ComponentRegistry",
    "ComponentSpec",
    "ComponentType",
    "DuplicateComponentError",
    "DuplicateTypeError",
    "InheritanceCycleError",
    "PlatformRegistry",
    "PlatformType",
    "ROOT_PLATFORM",
    "ResolvedComponent",
    "ResolvedPlatform",
    "TypeDefinitionError",
    "TypeRegistry",
    "UnknownComponentError",
    "UnknownTypeError",
    "default_platforms",
    "default_registry",
    "register_component",
    "register_platform",
    "reset_default_platforms",
    "reset_default_registry",
]
