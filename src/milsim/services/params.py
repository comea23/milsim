"""参数模式：想定参数必须先声明。

为什么参数必须先声明
--------------------
想定里写 ``detect_range 40``——40 是米还是千米？解析器无从判断，除非有人
告诉它。所以每个组件类用 ``PARAMS`` 声明自己接受哪些参数、什么量纲、
默认值是多少。声明跟着**组件类**走，不集中在一张全局表里，这样加一个
组件类型不必改解析器。

无单位的值按参数的**基本单位**处理
---------------------------------
``detect_range 40`` 得到 ``40.0`` 米，**不是** 40000 米。这条和"参数声明
默认单位"是同一件事：基本单位由量纲决定（距离米、时间微秒、角度度、
速度 m/s），想定作者必须显式写单位才能表示别的量级。

不做静默猜测是刻意的：猜错一次就是几万公里的偏差，而错误会一路传到
通视判定、探测距离、寻路代价里，根本看不出是哪来的。

时间为什么直接转微秒
--------------------
``2 s`` 必须直接得到整数 ``2_000_000``，而不是 ``int(2.0 * 1e6)`` 那种
"先算成秒、再乘回来"的路径——后者会把引擎层好不容易消灭的浮点误差
重新引进来。所以时间维度的换算因子是**整数**，走整数乘法。小数输入
（``0.1 s``）由 ``round`` 收尾，得到精确的 ``100000``。

拼错的参数名必须报错
--------------------
想定里把 ``detect_range`` 打成 ``detect_rang`` 而仿真照常启动，比直接
报错糟糕得多——你会花半天纳闷"为什么探测距离没生效"。所以未知参数一律
报错，并且用 ``difflib`` 给出最接近的名字。
"""

from __future__ import annotations

from dataclasses import dataclass
from difflib import get_close_matches
from enum import IntEnum
from math import pi
from typing import Any, Iterator, Mapping, Sequence

from ..errors import ConfigurationError


# ---------------------------------------------------------------------------
# 异常
# ---------------------------------------------------------------------------

class ParamError(ConfigurationError):
    """参数取值错误。

    ``names`` 是**出错的参数名**（可能不止一个）。让调用方——想定语义层——
    能据此查到具体哪一行：拼写错误在几十行的块里，块起始行号帮不上忙。
    """

    def __init__(self, message: str, names: Sequence[str] = ()) -> None:
        self.names: tuple[str, ...] = tuple(names)
        super().__init__(message)


class UnknownParameterError(ParamError):
    """想定里出现了 ``PARAMS`` 中没有声明的参数。"""


class MissingParameterError(ParamError):
    """必需参数没有给值。"""


# ---------------------------------------------------------------------------
# 量纲与单位
# ---------------------------------------------------------------------------

#: 标准重力加速度（``m/s²``）。**只在这一个地方定义**：它既是
#: ``Dimension.ACCELERATION`` 里 ``g`` 这个单位的换算因子，也是弹道件自由
#: 飞行那一段用的重力常数。两处各写一个 9.80665 的话，"过载写了多少个 g"
#: 与"弹道算出来的重力"迟早会分家，而分家之后单位看着都对。
STANDARD_GRAVITY = 9.80665


class Dimension(IntEnum):
    """参数的量纲。``NONE`` 表示纯数值，不带单位。"""

    NONE = 0
    DISTANCE = 1
    TIME = 2
    ANGLE = 3
    SPEED = 4
    FREQUENCY = 5
    AREA = 6
    #: 角速度（``deg/s``）。**刻意与 ``ANGLE`` 分开**：机动件的 ``turn_rate``
    #: 是"每秒转多少度"，与"总共转多少度"（航向、视场）不是一个量纲。合成
    #: 一个的话，想定里 ``turn_rate 12 deg`` 会被读成"转 12 度"——单位看着
    #: 对、意思全错，而且不报错。
    ANGULAR_SPEED = 7
    #: **加速度**（``m/s²``）。三个平动通道里"速率的变化率"都是它：
    #: ``linear_accel``（前向）、``vertical_accel``（垂向），
    #: 以及 ``radial_accel``（径向——过载约束，§5.7.11）。
    #:
    #: 它与 ``SPEED`` **必须分开**：``1.5 m/s`` 与 ``1.5 m/s2`` 差一个字符，
    #: 而后者才是"每秒多快"，前者是"多快"。合成一个量纲的话，想定里把
    #: ``linear_accel 2 m/s`` 写错成速度单位会被安静地接受——那是把一个
    #: 加速度当速度用，数值还会大两个数量级。
    ACCELERATION = 8
    #: **角加速度**（``deg/s²``）——角速度的**变化率**。
    #:
    #: 同样刻意独立：``turn_rate`` 是"每秒转多少度"（``deg/s``），
    #: ``angular_accel`` 是"每秒多转多少度每秒"（``deg/s²``）。两者相差一阶，
    #: 混用的后果是"转向建立有多快"这件事被写成"转得多快"，而数字对不上时
    #: 单看想定文件是查不出来的。
    ANGULAR_ACCEL = 9
    #: **质量**（``kg``）。弹的重力、推力加速度、燃料消耗都从它来，
    #: 而"烧掉多少燃料"这件事必须有一个数——不然弹道件的加速度是常数，
    #: 而真实火箭的加速度随燃料减少而**上升**。
    MASS = 10
    #: **力**（``N``）。发动机推力。
    #:
    #: 与 ``ACCELERATION`` 分开是有代价的（多一个量纲），但合起来更糟：
    #: 推力是"这台发动机给多少"，加速度是"这枚弹因此多快"——前者与质量
    #: 无关、后者才是弹的属性。写成加速度的话，换装一台发动机就要把质量
    #: 一起改，而两个数字的一致性没有任何东西在保证。
    FORCE = 11
    #: **功率**（``W``）。发射机的峰值功率、干扰机的功率都从它来。
    #:
    #: 为什么不用 ``number`` 凑：功率的**数量级跨度极大**——雷达发射机在
    #: 10⁵ W 量级、干扰机可能写 1 kW、接收机噪声在 10⁻¹⁴ W 量级。用无单位的
    #: 数写，``250 kW`` 与 ``250`` 会得到同一个数（差 1000 倍），而**参数表上
    #: 看不出错**，症状是"填了功率但探测距离不对"。已落库的
    #: `library/external.txt` 里那型雷达的发射功率原文就是 ``"250 kW"``
    #: （``peak_power_raw``，尚未解析），这个量纲正是它的归宿。
    POWER = 12


class ParamKind(IntEnum):
    """参数的值类型。"""

    NUMBER = 0
    INTEGER = 1
    BOOLEAN = 2
    STRING = 3
    STRINGS = 4
    COORD = 5
    #: **阶段表**——一串有先后次序的飞行阶段（弹的制导剧本）。
    #:
    #: 它是唯一一个**元素不是标量**的种类，所以刻意自立一类而不是拿
    #: ``STRINGS`` 凑：``STRINGS`` 的元素是字符串，读者会以为阶段名就是
    #: 全部内容；而阶段真正的信息在它带的那些量（高度、弹道倾角、速率、
    #: 导引律、过载上限）上。
    #:
    #: 元素类型由**用它的组件**定义（弹的制导件用自己的 ``Phase``），参数
    #: 系统只保证"是一个非空的、带名字的阶段序列"——量纲与取值范围的校验
    #: 在组件侧，因为只有它知道每个字段是什么意思。
    PHASES = 6


#: 每个量纲的**基本单位**。无单位的值按它处理。
BASE_UNIT: dict[Dimension, str] = {
    Dimension.DISTANCE: "m",
    Dimension.TIME: "us",          # 微秒，与引擎的 SimTime 一致
    Dimension.ANGLE: "deg",
    Dimension.SPEED: "m/s",
    Dimension.FREQUENCY: "Hz",
    Dimension.AREA: "m2",
    Dimension.ANGULAR_SPEED: "deg/s",
    Dimension.ACCELERATION: "m/s2",
    Dimension.ANGULAR_ACCEL: "deg/s2",
    Dimension.MASS: "kg",
    Dimension.FORCE: "N",
    Dimension.POWER: "W",
}

#: 单位换算表：``1 该单位 = factor 个基本单位``。
#:
#: **时间维度的因子刻意用整数**（``s`` 是 1_000_000 而不是 1e6），这样
#: ``2 s`` 走整数乘法得到精确的 2_000_000，不会经过浮点。
#:
#: ``nmi`` 是海里的标准缩写，``nm`` 保留作别名——但**不要**在本表里新增
#: 与已有单位形近的别名，``nm`` 在别处是纳米，混用迟早出事。
_UNITS: dict[Dimension, dict[str, float | int]] = {
    Dimension.DISTANCE: {
        "m": 1.0,
        "km": 1000.0,
        "cm": 0.01,
        "mm": 0.001,
        "nmi": 1852.0,             # 海里
        "nm": 1852.0,              # 别名，为兼容早期想定
        "mi": 1609.344,
        "ft": 0.3048,
    },
    Dimension.TIME: {
        "us": 1,
        "ms": 1_000,
        "s": 1_000_000,
        "sec": 1_000_000,
        "min": 60_000_000,
        "h": 3_600_000_000,
        "d": 86_400_000_000,
    },
    Dimension.ANGLE: {
        "deg": 1.0,
        "rad": 180.0 / pi,
    },
    Dimension.SPEED: {
        "m/s": 1.0,
        "km/h": 1000.0 / 3600.0,
        "kmh": 1000.0 / 3600.0,
        "kn": 0.514444,            # 节
        "kt": 0.514444,
        "mph": 0.44704,
    },
    Dimension.FREQUENCY: {
        "Hz": 1.0,
        "kHz": 1e3,
        "MHz": 1e6,
        "GHz": 1e9,
    },
    Dimension.AREA: {
        "m2": 1.0,
        "km2": 1e6,
    },
    Dimension.ANGULAR_SPEED: {
        "deg/s": 1.0,
        "rad/s": 180.0 / pi,
    },
    Dimension.ACCELERATION: {
        "m/s2": 1.0,
        "g": STANDARD_GRAVITY,     # 标准重力加速度，过载用它写更好读
    },
    Dimension.ANGULAR_ACCEL: {
        "deg/s2": 1.0,
        "rad/s2": 180.0 / pi,
    },
    Dimension.MASS: {
        "kg": 1.0,
        "t": 1000.0,
        "g": 0.001,                # 克。与 ACCELERATION 的 "g" 同名不同义，
                                   # 靠量纲区分——各查各的表，不会串
        "lb": 0.45359237,
    },
    Dimension.FORCE: {
        "N": 1.0,
        "kN": 1000.0,
        "lbf": 4.4482216152605,
    },
    Dimension.POWER: {
        # 大小写敏感（同 Hz/N），所以这里**只给能一眼读出量级的三个**。
        # ★ 刻意**不给** ``mW``/``mw``：``MW``（兆瓦）与 ``mW``（毫瓦）
        #   只差一个字母的大小写、相差 10⁹ 倍，而写错时**不报错**
        #   （两个都在表里就都能查到）。同 `nm` 那条注释的教训。
        #   要毫瓦请写 ``0.001 W``。
        "W": 1.0,
        "kW": 1e3,
        "MW": 1e6,
        # 小写别名只给这两个——它们与上面一一对应，不存在歧义。
        "w": 1.0,
        "kw": 1e3,
    },
}


def known_units(dimension: Dimension) -> list[str]:
    """某个量纲下所有可用的单位名。

    基本单位排最前——报错时作者最常想用的就是它。其余按先短后长、同长按
    字母序，让 ``m`` 排在 ``km`` 前、``km`` 排在 ``nmi`` 前。
    """
    units = list(_UNITS.get(dimension, {}))
    base = BASE_UNIT.get(dimension)
    units.sort(key=lambda u: (u != base, len(u), u))
    return units


# ---------------------------------------------------------------------------
# 坐标
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class CoordValue:
    """坐标值。想定里两种写法：

    - ``latlng 39.95 116.35``（经纬度，度）
    - ``hex 1204 883 @ zone NORTH_FRONT layer 0``（战区内的网格坐标）

    两种都保留原样，**不在解析阶段换算**——从经纬度到网格坐标需要战区
    锚点，而战区要等装配阶段才激活。这里存的是"作者写了什么"。

    ``hex`` 坐标**必须能确定战区**：轴向坐标是相对某个战区锚点的，
    同一个 ``(1204, 883)`` 在两个战区里是两个不同的地方。战区只有一个时
    ``zone`` 可以省略（装配层能唯一确定），多个战区时必须写明。
    """

    form: str                       # "latlng" 或 "hex"
    lat: float = 0.0
    lng: float = 0.0
    q: int = 0
    r: int = 0
    layer: int = 0
    #: 网格坐标所属的战区名。仅 ``form == "hex"`` 时有意义。
    zone: str = ""

    def __post_init__(self) -> None:
        if self.form not in ("latlng", "hex"):
            raise ParamError(f"未知坐标形式 {self.form!r}（只支持 latlng / hex）")

        if self.form != "latlng":
            return

        if self.zone:
            raise ParamError("latlng 坐标不该带战区——经纬度本身就唯一确定了位置")

        # 范围校验放在**值类型自己**身上而不是解析器里：坐标有两条构造
        # 路径（带 latlng 前缀的、以及 `anchor 39.9 116.4` 这种裸数对的），
        # 只在一处校验的话另一条会漏过去——实测漏过一次
        if not -90.0 <= self.lat <= 90.0:
            raise ParamError(f"纬度 {self.lat} 超出 [-90, 90]")
        if not -180.0 <= self.lng <= 180.0:
            raise ParamError(f"经度 {self.lng} 超出 [-180, 180]")


# ---------------------------------------------------------------------------
# 参数声明
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class ParamSpec:
    """一个参数的完整声明：**值类型、量纲、默认值**，外加范围与枚举约束。"""

    kind: ParamKind
    dimension: Dimension = Dimension.NONE
    default: Any = None
    required: bool = False
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[str, ...] = ()
    description: str = ""

    # -- 换算与校验 --------------------------------------------------------

    def convert(self, value: Any, unit: str | None, *, owner: str = "") -> Any:
        """把想定里的原始值换算成内部表示。

        ``unit`` 为 ``None`` 表示作者没写单位，此时按**基本单位**处理。
        ``owner`` 是出错信息的前缀，通常形如 ``zone NORTH_FRONT.radius``——
        带上参数名才能在一堆参数里定位到写错的那一个。
        """
        where = owner or self.type_label()

        if self.dimension is not Dimension.NONE:
            return self._convert_quantity(value, unit, where=where)

        if self.kind is ParamKind.NUMBER:
            number = self._as_number(value, unit, where)
            self._check_range(number, where)
            return number

        if self.kind is ParamKind.INTEGER:
            number = self._as_number(value, unit, where)
            if not float(number).is_integer():
                raise ParamError(f"{where}: 需要整数，收到 {value!r}")
            self._check_range(number, where)
            return int(number)

        if self.kind is ParamKind.BOOLEAN:
            if not isinstance(value, bool):
                raise ParamError(f"{where}: 需要 true / false，收到 {value!r}")
            return value

        if self.kind is ParamKind.STRING:
            if not isinstance(value, str):
                raise ParamError(f"{where}: 需要字符串，收到 {value!r}")
            self._check_choices(value, where)
            return value

        if self.kind is ParamKind.STRINGS:
            return self._convert_strings(value, where)

        if self.kind is ParamKind.PHASES:
            return self._convert_phases(value, where)

        if self.kind is ParamKind.COORD:
            if not isinstance(value, CoordValue):
                raise ParamError(f"{where}: 需要坐标，收到 {value!r}")
            return value

        raise ParamError(f"{where}: 未支持的参数类型 {self.kind!r}")

    def _convert_quantity(self, value: Any, unit: str | None, *, where: str) -> Any:
        number = self._as_number(value, unit, where)
        table = _UNITS[self.dimension]

        if unit is None:
            # 基本单位不需要换算——时间维度因此不经过浮点
            factor: float | int = 1
        else:
            if unit not in table:
                options = "、".join(known_units(self.dimension))
                raise ParamError(
                    f"{where}: 未知单位 {unit!r}（{self.dimension_name()}可用：{options}）"
                )
            factor = table[unit]

        scaled = number * factor

        if self.kind is ParamKind.INTEGER:
            # round 而不是 int()：int(0.1 * 1e6) 会少 1 微秒（浮点表示问题）
            result: Any = int(round(scaled))
        else:
            result = float(scaled)

        self._check_range(result, where)
        return result

    def _as_number(self, value: Any, unit: str | None, where: str) -> float:
        # bool 是 int 的子类，不排除的话 `true` 会被当成 1
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ParamError(f"{where}: 需要数值，收到 {value!r}")
        if self.dimension is Dimension.NONE and unit is not None:
            raise ParamError(f"{where}: 该参数不带单位，但写成了 {unit!r}")
        return value

    def _convert_strings(self, value: Any, where: str) -> tuple[str, ...]:
        if isinstance(value, str) or not isinstance(value, Sequence):
            raise ParamError(f"{where}: 需要列表，收到 {value!r}")
        items = tuple(value)
        for item in items:
            if not isinstance(item, str):
                raise ParamError(f"{where}: 列表元素必须是字符串，收到 {item!r}")
            self._check_choices(item, where)
        return items

    def _convert_phases(self, value: Any, where: str) -> tuple[Any, ...]:
        """阶段表：非空、每个元素带名字。

        **取值范围的校验不在这里。** 参数系统只知道"这是一个阶段表"，不知道
        每个字段（高度、弹道倾角、导引律、过载上限）是什么意思——那是弹的
        制导件的事，所以范围校验在组件侧。这里只挡住三种一眼能看出来的错：
        不是序列、空的、元素没有名字。
        """
        if isinstance(value, str) or not isinstance(value, Sequence):
            raise ParamError(f"{where}: 需要阶段序列，收到 {value!r}")
        items = tuple(value)
        if not items:
            raise ParamError(f"{where}: 阶段表不能为空——一枚弹至少要有一个阶段")
        for item in items:
            name = getattr(item, "name", None)
            if not isinstance(name, str) or not name:
                raise ParamError(
                    f"{where}: 阶段表里的每一项都要有 name，收到 {item!r}"
                )
        return items

    def _check_choices(self, value: str, where: str) -> None:
        if self.choices and value not in self.choices:
            options = "、".join(self.choices)
            raise ParamError(f"{where}: 只能是 {options}，收到 {value!r}")

    def _check_range(self, value: float, where: str) -> None:
        if self.minimum is not None and value < self.minimum:
            raise ParamError(f"{where}: 不能小于 {self.minimum}，收到 {value}")
        if self.maximum is not None and value > self.maximum:
            raise ParamError(f"{where}: 不能大于 {self.maximum}，收到 {value}")

    # -- 展示 --------------------------------------------------------------

    def dimension_name(self) -> str:
        if self.dimension is Dimension.NONE:
            return ""
        return {
            Dimension.DISTANCE: "距离",
            Dimension.TIME: "时间",
            Dimension.ANGLE: "角度",
            Dimension.SPEED: "速度",
            Dimension.FREQUENCY: "频率",
            Dimension.AREA: "面积",
            Dimension.ANGULAR_SPEED: "角速度",
            Dimension.ACCELERATION: "加速度",
            Dimension.ANGULAR_ACCEL: "角加速度",
            Dimension.MASS: "质量",
            Dimension.FORCE: "力",
            Dimension.POWER: "功率",
        }[self.dimension]

    def type_label(self) -> str:
        """给报错和文档看的人类可读类型名。"""
        if self.dimension is not Dimension.NONE:
            return f"{self.dimension_name()}({BASE_UNIT[self.dimension]})"
        return {
            ParamKind.NUMBER: "数值",
            ParamKind.INTEGER: "整数",
            ParamKind.BOOLEAN: "布尔",
            ParamKind.STRING: "字符串",
            ParamKind.STRINGS: "字符串列表",
            ParamKind.PHASES: "阶段表",
            ParamKind.COORD: "坐标",
        }[self.kind]

    def describe(self) -> str:
        """``detect_range: 距离(m) = 40000.0`` 这样的单行描述。"""
        if self.required:
            tail = "必填"
        else:
            tail = f"= {self.default!r}"
        if self.choices:
            tail += f" 可选 {list(self.choices)}"
        return f"{self.type_label()} {tail}"

    def converted_default(self) -> Any:
        """默认值已按内部表示给出，这里只做一次类型规整。"""
        if self.required:
            return None
        if self.kind in (ParamKind.STRINGS, ParamKind.PHASES) and not isinstance(
            self.default, tuple
        ):
            return tuple(self.default)
        return self.default


class Params:
    """``ParamSpec`` 的声明工厂。

    用法::

        class HexSearchRadar:
            PARAMS = {
                "detect_range":  Params.distance(40_000.0),
                "scan_interval": Params.duration(2_000_000),
                "fov":           Params.angle(120.0),
                "modes":         Params.strings(default=("SARH",)),
                "enabled":       Params.boolean(default=True),
            }

    写成静态方法而不是模块级函数，是为了让 ``PARAMS`` 表读起来整齐——
    ``Params.distance(...)`` 一眼能看出是参数声明，不是普通函数调用。
    """

    @staticmethod
    def number(
        default: float = 0.0, **kw: Any
    ) -> ParamSpec:
        return ParamSpec(ParamKind.NUMBER, Dimension.NONE, default, **kw)

    @staticmethod
    def integer(default: int = 0, **kw: Any) -> ParamSpec:
        return ParamSpec(ParamKind.INTEGER, Dimension.NONE, default, **kw)

    @staticmethod
    def distance(default: float | None = None, **kw: Any) -> ParamSpec:
        return _quantity(ParamKind.NUMBER, Dimension.DISTANCE, default, **kw)

    @staticmethod
    def duration(default: int | None = None, **kw: Any) -> ParamSpec:
        """时间参数。**默认值也是微秒**，与引擎的 ``SimTime`` 一致。"""
        return _quantity(ParamKind.INTEGER, Dimension.TIME, default, **kw)

    @staticmethod
    def angle(default: float | None = None, **kw: Any) -> ParamSpec:
        return _quantity(ParamKind.NUMBER, Dimension.ANGLE, default, **kw)

    @staticmethod
    def speed(default: float | None = None, **kw: Any) -> ParamSpec:
        return _quantity(ParamKind.NUMBER, Dimension.SPEED, default, **kw)

    @staticmethod
    def angle_rate(default: float | None = None, **kw: Any) -> ParamSpec:
        """角速度。基本单位 ``deg/s``，想定里通常就写 ``12 deg/s``。

        机动件的转向速率用它。**不要拿 ``angle`` 凑**：那个是"多少度"，
        这里问的是"每秒多少度"，两个量纲混用的后果是单位看着对、意思全错。
        """
        return _quantity(
            ParamKind.NUMBER, Dimension.ANGULAR_SPEED, default, **kw
        )

    @staticmethod
    def accel(default: float | None = None, **kw: Any) -> ParamSpec:
        """加速度。基本单位 ``m/s²``，想定里写 ``3 m/s2``，或按过载写 ``9 g``。

        机动件的 ``linear_accel`` / ``vertical_accel`` 用它；``radial_accel``
        也是它（径向加速度就是过载约束，用 ``g`` 写最直观）。

        **不要拿 ``speed`` 凑**：那个是"多快"，这里问的是"每秒快多少"。
        """
        return _quantity(
            ParamKind.NUMBER, Dimension.ACCELERATION, default, **kw
        )

    @staticmethod
    def angle_accel(default: float | None = None, **kw: Any) -> ParamSpec:
        """角加速度。基本单位 ``deg/s²``——角速度的建立快慢。

        机动件的 ``angular_accel`` 用它。与 :meth:`angle_rate` **不是一个
        量纲**：那个是"转得多快"，这个是"转向建立得多快"。
        """
        return _quantity(
            ParamKind.NUMBER, Dimension.ANGULAR_ACCEL, default, **kw
        )

    @staticmethod
    def frequency(default: float | None = None, **kw: Any) -> ParamSpec:
        """频率。基本单位 Hz，想定里通常写 ``9.4 GHz``。"""
        return _quantity(ParamKind.NUMBER, Dimension.FREQUENCY, default, **kw)

    @staticmethod
    def area(default: float | None = None, **kw: Any) -> ParamSpec:
        """面积。基本单位 m²，RCS 之类用它。"""
        return _quantity(ParamKind.NUMBER, Dimension.AREA, default, **kw)

    @staticmethod
    def boolean(default: bool = False, **kw: Any) -> ParamSpec:
        return ParamSpec(ParamKind.BOOLEAN, Dimension.NONE, default, **kw)

    @staticmethod
    def string(default: str = "", **kw: Any) -> ParamSpec:
        return ParamSpec(ParamKind.STRING, Dimension.NONE, default, **kw)

    @staticmethod
    def strings(default: Sequence[str] = (), **kw: Any) -> ParamSpec:
        return ParamSpec(
            ParamKind.STRINGS, Dimension.NONE, tuple(default), **kw
        )

    @staticmethod
    def coord(**kw: Any) -> ParamSpec:
        return ParamSpec(ParamKind.COORD, Dimension.NONE, None, required=True, **kw)

    @staticmethod
    def mass(default: float | None = None, **kw: Any) -> ParamSpec:
        """质量。基本单位 ``kg``，想定里通常写 ``1300 kg`` 或 ``1.3 t``。

        弹的加速度、燃料消耗、重力都从它算出来。**不要拿 ``number`` 凑**：
        那个无单位，写 ``1300`` 与写 ``1300 g`` 会得到同一个数。
        """
        return _quantity(ParamKind.NUMBER, Dimension.MASS, default, **kw)

    @staticmethod
    def force(default: float | None = None, **kw: Any) -> ParamSpec:
        """力（推力）。基本单位 ``N``，想定里通常写 ``180 kN``。"""
        return _quantity(ParamKind.NUMBER, Dimension.FORCE, default, **kw)

    @staticmethod
    def power(default: float | None = None, **kw: Any) -> ParamSpec:
        """功率。基本单位 ``W``，想定里通常写 ``250 kW`` / ``100 kW``。

        ★ **dB 形式（dBW / dBm）刻意不收**：``_convert_quantity`` 走的是
        "乘一个因子"的换算，而 dB 是**对数**——放进 ``_UNITS`` 会得到一个
        把一个数乘以 1.0 就当作 dBW 的结果，静默错 10 倍以上。
        本项目所有 dB 量（``antenna_gain_db`` / ``system_loss_db`` …）
        一律是**无单位的纯数**（``Params.number``），功率这一项保持同形：
        要 dBW 请在剧本/文档里换算成瓦再写。
        """
        return _quantity(ParamKind.NUMBER, Dimension.POWER, default, **kw)

    @staticmethod
    def phases(default: Sequence[Any] = (), **kw: Any) -> ParamSpec:
        """阶段表。元素是**用它的组件**定义的阶段描述（弹的 ``Phase``）。

        它在这里是一个参数、而不是组件类上的一个裸属性，理由是
        ``PARAMS`` 是**契约**（§5.1）：写在参数表里的东西会出现在
        ``schema()`` 里、会走同一条"类默认值 ← 类型链 ← 挂载差量"的解析
        路径，将来想定层支持阶段块时也不必再动它。
        """
        return ParamSpec(ParamKind.PHASES, Dimension.NONE, tuple(default), **kw)


def _quantity(
    kind: ParamKind,
    dimension: Dimension,
    default: float | int | None,
    **kw: Any,
) -> ParamSpec:
    """构造带量纲的参数声明。``default=None`` 表示必填。"""
    if default is None:
        return ParamSpec(kind, dimension, None, required=True, **kw)
    return ParamSpec(kind, dimension, default, **kw)


# ---------------------------------------------------------------------------
# 参数集合：一整组声明 + 一次解析
# ---------------------------------------------------------------------------

class ParamSet:
    """一组 ``ParamSpec``，负责把想定里的一组原始值解析成内部表示。

    解析做四件事：**拒绝未知键**、**补齐默认值**、**换算单位**、**校验范围**。
    出错信息里带上 ``owner``（类型名），否则"未知参数 detect_rang"这种报错
    在几十个类型的想定里根本定位不到。
    """

    __slots__ = ("_specs", "owner")

    def __init__(self, specs: Mapping[str, ParamSpec], owner: str = "") -> None:
        self._specs: dict[str, ParamSpec] = dict(specs)
        self.owner = owner

    @classmethod
    def from_class(cls, component_class: type, owner: str = "") -> "ParamSet":
        """从组件类的 ``PARAMS`` 声明构造。

        类没有 ``PARAMS`` 时给空表而不是报错——不是每个组件都需要参数
        （比如一个只转发消息的通信组件）。但声明了就必须是 ``ParamSpec``，
        写错成裸字符串会让解析在很远的地方炸，所以在这里就拦住。
        """
        declared = getattr(component_class, "PARAMS", None)
        name = owner or getattr(component_class, "COMPONENT_NAME", "") or (
            getattr(component_class, "__name__", "?")
        )
        if declared is None:
            return cls({}, name)
        if not isinstance(declared, Mapping):
            raise ParamError(f"{name}.PARAMS 必须是字典，收到 {type(declared).__name__}")

        for key, spec in declared.items():
            if not isinstance(spec, ParamSpec):
                raise ParamError(
                    f"{name}.PARAMS[{key!r}] 必须是 Params.* 的返回值，"
                    f"收到 {type(spec).__name__}"
                )
        return cls(declared, name)

    # -- 查询 --------------------------------------------------------------

    def __contains__(self, name: str) -> bool:
        return name in self._specs

    def __len__(self) -> int:
        return len(self._specs)

    def __iter__(self) -> Iterator[str]:
        return iter(self._specs)

    def names(self) -> list[str]:
        return list(self._specs)

    def spec(self, name: str) -> ParamSpec | None:
        return self._specs.get(name)

    def required_names(self) -> list[str]:
        return [n for n, s in self._specs.items() if s.required]

    def defaults(self) -> dict[str, Any]:
        """所有带默认值的参数，按声明顺序。不改动调用方的任何东西。"""
        return {
            name: spec.converted_default()
            for name, spec in self._specs.items()
            if not spec.required
        }

    # -- 解析 --------------------------------------------------------------

    def resolve(
        self, values: Mapping[str, Any] | None = None, *, owner: str = ""
    ) -> dict[str, Any]:
        """原始值 → 内部表示。返回**新字典**，不修改入参。"""
        where = owner or self.owner
        raw = dict(values) if values else {}
        result = self.defaults()

        unknown = sorted(set(raw) - set(self._specs))
        if unknown:
            raise UnknownParameterError(
                f"{where}: 未知参数 " + "、".join(self._hint(k) for k in unknown),
                names=unknown,
            )

        for name, value in raw.items():
            spec = self._specs[name]
            bare, unit = pop_unit(value)
            # 报错信息里要带**参数名**，只给类型名的话，一个组件有二十个参数
            # 时根本不知道是哪一个写错了
            try:
                result[name] = spec.convert(
                    bare, unit, owner=f"{where}.{name}" if where else name
                )
            except ParamError as exc:
                # 补上参数名，让语义层能定位到行——convert 自己不知道名字
                if not exc.names:
                    exc.names = (name,)
                raise

        missing = [n for n in self.required_names() if result.get(n) is None]
        if missing:
            # 缺参数没有"出错那一行"可指，行号由调用方回退到块首
            raise MissingParameterError(
                f"{where}: 缺少必填参数 " + "、".join(sorted(missing)),
                names=missing,
            )

        return result

    def _hint(self, name: str) -> str:
        """给拼错的参数名一个修正建议。

        省下的是"为什么这个参数没生效"的半小时——报错直接告诉你可能想写什么。
        """
        close = get_close_matches(name, list(self._specs), n=1, cutoff=0.7)
        return f"{name!r}（是否想写 {close[0]!r}？）" if close else repr(name)

    # -- 组合 --------------------------------------------------------------

    def merged_with(self, other: "ParamSet") -> "ParamSet":
        """参数表合并，``other`` 优先。用于"实现类参数 + 想定层参数"的叠加。"""
        specs = dict(self._specs)
        specs.update(other._specs)
        return ParamSet(specs, other.owner or self.owner)

    def with_owner(self, owner: str) -> "ParamSet":
        return ParamSet(self._specs, owner)

    # -- 展示 --------------------------------------------------------------

    def schema(self) -> str:
        """多行 schema 表，用于调试与文档。"""
        if not self._specs:
            return f"{self.owner}: （无参数）"
        lines = [f"{self.owner}:"]
        width = max(len(n) for n in self._specs)
        for name, spec in self._specs.items():
            lines.append(f"    {name:<{width}}  {spec.describe()}")
        return "\n".join(lines)

    def __repr__(self) -> str:
        return f"<ParamSet {self.owner or '?'} {len(self._specs)} 个参数>"


def pop_unit(value: Any) -> tuple[Any, str | None]:
    """拆出 ``(数值, 单位)``。

    解析器产出的是 ``WithUnit(value, unit)`` 这样的二元组，而 ``ParamSpec``
    只关心裸值。写成模块函数是为了让"带单位的原始值"这个约定只有一处定义
    ——解析器与 ``ParamSet`` 都照它来，不会各写一份。
    """
    if isinstance(value, WithUnit):
        return value.value, value.unit
    return value, None


@dataclass(frozen=True, slots=True)
class WithUnit:
    """想定里的"数值 + 单位"。解析器产出它，``ParamSpec.convert`` 消费它。"""

    value: float
    unit: str | None = None


def with_unit(value: float, unit: str | None = None) -> WithUnit:
    return WithUnit(value, unit)


__all__ = [
    "BASE_UNIT",
    "CoordValue",
    "Dimension",
    "MissingParameterError",
    "ParamError",
    "ParamKind",
    "ParamSet",
    "ParamSpec",
    "Params",
    "STANDARD_GRAVITY",
    "UnknownParameterError",
    "WithUnit",
    "known_units",
    "with_unit",
]
