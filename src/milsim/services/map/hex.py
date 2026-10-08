"""局部平面六边形网格的坐标系统。

定位
----
每个"战区"挂一张局部平面网格，边长 100 m ~ 2 km 可配。网格运算全部在平面
坐标下完成，与全球层的衔接交给 ``zone.py``。

三套坐标各司其职
----------------
    axial  (q, r)        运算用。邻居偏移恒定、距离可闭式求解
    offset (col, row)    存储用。可以铺成二维数组
    world  (x, y)        投影用。单位米，相对战区锚点

几何朝向取 **pointy-top**（尖顶朝上）：东西方向为列，符合兵棋阅读习惯。

为什么用等距方位投影（AED）而不是横轴墨卡托
--------------------------------------------
AED 从锚点出发的**径向距离精确保距**——实测 1360 km 半径内相对误差 0.0000%，
切向拉伸也只有 0.1% 量级（500 km 处 c/sin(c) = 1.00103）。
对网格用途来说"距离准"远比"形状准"重要，而且 AED 有闭式反解，
不需要 TM 那套子午线弧长迭代。

代价是非保角，远离锚点后形状逐渐变形——但那是几百公里外的事，
那时本来就该交给全球层处理了。
"""

from __future__ import annotations

import math
from typing import Final, NamedTuple

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

#: 地球平均半径（米）。用球面而非椭球：对局部网格自洽性够用，
#: 且 AED 的径向保距性质在球面下是精确的。需要椭球精度的长距离量算
#: 请另用测地函数，不要拿网格投影坐标去做洲际距离计算。
EARTH_RADIUS: Final[float] = 6_371_008.8

_SQRT3: Final[float] = math.sqrt(3.0)

# ---------------------------------------------------------------------------
# 单元编码
#
# [63:52]  zone_id   12 bit   最多 4096 个战区
# [51:48]  layer      4 bit   战区内分辨率层 0-15
# [47:44]  flags      4 bit   保留位
# [43:22]  q         22 bit   带偏移，范围 ±2,097,152
# [21:0]   r         22 bit   带偏移
#
# 22 bit 坐标在 100 m 网格上覆盖 ±209,715 km——远超任何单个战区的需要。
# 12 bit 战区数按"每个战区约 500 km 见方"估算，全球约需 2000 个，余量充足。
# ---------------------------------------------------------------------------

ZONE_BITS: Final[int] = 12
LAYER_BITS: Final[int] = 4
FLAGS_BITS: Final[int] = 4
COORD_BITS: Final[int] = 22

COORD_OFFSET: Final[int] = 1 << (COORD_BITS - 1)      # 2_097_152
COORD_MASK: Final[int] = (1 << COORD_BITS) - 1
COORD_LIMIT: Final[int] = COORD_OFFSET                # q/r 允许 ±LIMIT

ZONE_MASK: Final[int] = (1 << ZONE_BITS) - 1
LAYER_MASK: Final[int] = (1 << LAYER_BITS) - 1
FLAGS_MASK: Final[int] = (1 << FLAGS_BITS) - 1


class Axial(NamedTuple):
    """轴向坐标。六个邻居方向的偏移量恒定，是网格运算的主力表示。"""

    q: int
    r: int

    def __add__(self, other: "Axial") -> "Axial":      # type: ignore[override]
        return Axial(self.q + other.q, self.r + other.r)

    def __sub__(self, other: "Axial") -> "Axial":      # type: ignore[override]
        return Axial(self.q - other.q, self.r - other.r)

    def __mul__(self, k: int) -> "Axial":              # type: ignore[override]
        return Axial(self.q * k, self.r * k)

    __rmul__ = __mul__

    @property
    def s(self) -> int:
        """立方坐标的第三个分量，恒有 q + r + s == 0。"""
        return -self.q - self.r


#: 六个邻居方向，从正东开始顺时针。
#:
#: 顺时针顺序不是随意定的：``ring()`` 依赖它才能正确绕圈。
#: 改动顺序会让所有环形、邻域查询静默出错。
DIRECTIONS: Final[tuple[Axial, ...]] = (
    Axial(+1, 0),    # 0  E
    Axial(+1, -1),   # 1  NE
    Axial(0, -1),    # 2  NW
    Axial(-1, 0),    # 3  W
    Axial(-1, +1),   # 4  SW
    Axial(0, +1),    # 5  SE
)


# ---------------------------------------------------------------------------
# 基本运算
# ---------------------------------------------------------------------------

def distance(a: Axial, b: Axial) -> int:
    """六边形距离（相隔几格）。

    ``|Δq| + |Δr| + |Δq+Δr|`` 必为偶数，所以整数除法是精确的。
    """
    dq = a.q - b.q
    dr = a.r - b.r
    return (abs(dq) + abs(dr) + abs(dq + dr)) // 2


def neighbors(c: Axial) -> tuple[Axial, ...]:
    """六个相邻单元，顺序与 DIRECTIONS 一致。"""
    return tuple(Axial(c.q + d.q, c.r + d.r) for d in DIRECTIONS)


def neighbor(c: Axial, direction: int) -> Axial:
    d = DIRECTIONS[direction % 6]
    return Axial(c.q + d.q, c.r + d.r)


def ring(center: Axial, radius: int) -> list[Axial]:
    """以 center 为心、半径 radius 的那一圈（不含内圈）。"""
    if radius <= 0:
        return [center]

    results: list[Axial] = []
    current = center + DIRECTIONS[4] * radius      # 从西南角起步
    for i in range(6):
        for _ in range(radius):
            results.append(current)
            current = neighbor(current, i)
    return results


def spiral(center: Axial, radius: int) -> list[Axial]:
    """从中心向外螺旋展开，含中心。按半径分层，适合做"由近及远"的搜索。"""
    results = [center]
    for k in range(1, radius + 1):
        results.extend(ring(center, k))
    return results


def _round_half_up(x: float) -> int:
    """四舍五入，远离零方向。

    **不能用内置 round()** —— Python 的 round 是银行家舍入（round(0.5) == 0，
    round(1.5) == 2），会让边界单元的归属左右不对称，破坏网格的对偶性。
    """
    return math.floor(x + 0.5)


def cube_round(fq: float, fr: float) -> Axial:
    """把浮点轴向坐标吸附到最近的合法单元。

    做法是先分别取整三个立方分量，再把偏差最大的那个分量修正回去，
    保证结果满足 q + r + s == 0。这是六边形"像素取整"的标准解法。
    """
    fs = -fq - fr
    q = _round_half_up(fq)
    r = _round_half_up(fr)
    s = _round_half_up(fs)

    dq = abs(q - fq)
    dr = abs(r - fr)
    ds = abs(s - fs)

    if dq > dr and dq > ds:
        q = -r - s
    elif dr > ds:
        r = -q - s
    # else: s 的偏差最大，但它由 q、r 决定，不需要显式修正

    return Axial(q, r)


def line(a: Axial, b: Axial) -> list[Axial]:
    """两单元之间的连线（含首尾）。

    用于通视判定：沿这条线逐格采样高程，判断视线是否被地形遮挡。
    """
    n = distance(a, b)
    if n == 0:
        return [a]

    results: list[Axial] = []
    for i in range(n + 1):
        t = i / n
        results.append(
            cube_round(a.q + (b.q - a.q) * t, a.r + (b.r - a.r) * t)
        )
    return results


# ---------------------------------------------------------------------------
# 偏移坐标（存储用）
# ---------------------------------------------------------------------------

def axial_to_offset(c: Axial) -> tuple[int, int]:
    """轴向 → odd-r 偏移坐标 (col, row)，可直接索引二维数组。"""
    return c.q + (c.r - (c.r & 1)) // 2, c.r


def offset_to_axial(col: int, row: int) -> Axial:
    """odd-r 偏移 → 轴向。

    ``row - (row & 1)`` 恒为偶数，所以对负数用 ``//`` 向下取整仍是精确的
    （Python 的 ``&`` 对负数按补码处理，-3 & 1 == 1，符合"奇数"的判定）。
    """
    return Axial(col - (row - (row & 1)) // 2, row)


# ---------------------------------------------------------------------------
# 单元编码
# ---------------------------------------------------------------------------

def encode(q: int, r: int, zone_id: int = 0, layer: int = 0, flags: int = 0) -> int:
    """打包成 64 位单元标识。"""
    if not (-COORD_LIMIT <= q < COORD_LIMIT):
        raise ValueError(f"q={q} 超出 ±{COORD_LIMIT} 的可编码范围")
    if not (-COORD_LIMIT <= r < COORD_LIMIT):
        raise ValueError(f"r={r} 超出 ±{COORD_LIMIT} 的可编码范围")
    if not 0 <= zone_id <= ZONE_MASK:
        raise ValueError(f"zone_id={zone_id} 超出 0..{ZONE_MASK}")

    return (
        ((zone_id & ZONE_MASK) << (LAYER_BITS + FLAGS_BITS + 2 * COORD_BITS))
        | ((layer & LAYER_MASK) << (FLAGS_BITS + 2 * COORD_BITS))
        | ((flags & FLAGS_MASK) << (2 * COORD_BITS))
        | ((q + COORD_OFFSET) << COORD_BITS)
        | (r + COORD_OFFSET)
    )


def encode_axial(c: Axial, zone_id: int = 0, layer: int = 0, flags: int = 0) -> int:
    return encode(c.q, c.r, zone_id, layer, flags)


def decode(cell: int) -> tuple[Axial, int, int, int]:
    """解包，返回 (轴向坐标, zone_id, layer, flags)。"""
    zone_id = (cell >> (LAYER_BITS + FLAGS_BITS + 2 * COORD_BITS)) & ZONE_MASK
    layer = (cell >> (FLAGS_BITS + 2 * COORD_BITS)) & LAYER_MASK
    flags = (cell >> (2 * COORD_BITS)) & FLAGS_MASK
    q = ((cell >> COORD_BITS) & COORD_MASK) - COORD_OFFSET
    r = (cell & COORD_MASK) - COORD_OFFSET
    return Axial(q, r), zone_id, layer, flags


def cell_zone(cell: int) -> int:
    return (cell >> (LAYER_BITS + FLAGS_BITS + 2 * COORD_BITS)) & ZONE_MASK


# ---------------------------------------------------------------------------
# 局部投影
# ---------------------------------------------------------------------------

def _clamp_unit(x: float) -> float:
    """把浮点夹到 [-1, 1]。

    acos/asin 的参数由三角函数算出，理论值域就是 [-1,1]，但浮点舍入
    会偶尔越界（典型是 acos(1.0000000000000002)），直接抛 ValueError。
    这个 clamp 是必须的，不是防御性冗余。
    """
    return -1.0 if x < -1.0 else (1.0 if x > 1.0 else x)


def heading_to_offset(heading_deg: float, distance_m: float) -> tuple[float, float]:
    """航向 + 距离 → 世界坐标位移 ``(dx, dy)``，单位米。

    **y 轴指向南**（图像坐标习惯，见 :meth:`LocalFrame.to_world` 的
    ``y = 1.5 × size × r``），所以北向分量对应**负**的 y 位移：

    .. code-block:: text

        dx =  distance × sin(θ)      东向
        dy = -distance × cos(θ)      南向为正，所以向北是负

    写成 ``(d·cosθ, d·sinθ)`` 是个很像对的错误——θ = 45° 与 225° 的位移
    恰好对称，画出来"看起来在动"，只有对照地图才会发现整体偏了 90°。
    所以这个换算**只在这里写一次**，谁都别自己推。
    """
    radians = math.radians(heading_deg)
    return distance_m * math.sin(radians), -distance_m * math.cos(radians)


def offset_to_heading(dx: float, dy: float) -> float:
    """世界坐标位移 ``(dx, dy)`` → 航向角，单位度，``[0, 360)``。

    :func:`heading_to_offset` 的逆。写在它旁边是刻意的：这对换算互为逆
    运算，隔开两个文件放迟早有一边改了另一边没改——而"改了没改"在这类
    公式上看不出来（45° 与 225° 的位移恰好对称，画出来都在动）。

    零点与符号跟正变换一致：0° 指北（**负** y），90° 指东（正 x）。
    位移为零时返回 0——"没有方向"在这里没有更好的表示，而抛异常会让
    调用方在"实体原地未动"这个正常情形下崩掉。
    """
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.degrees(math.atan2(dx, -dy)) % 360.0


class LocalFrame:
    """一个战区内的局部平面坐标系。

    锚点是战区的地理中心，网格坐标原点就落在那里。向外辐射时误差随距离
    缓慢增长，所以**战区不能划得太大**——500 km 半径内可放心使用，
    超过 1000 km 建议拆分成多个战区。
    """

    __slots__ = ("lat0", "lng0", "size", "_p1", "_l1")

    def __init__(self, lat0: float, lng0: float, size: float) -> None:
        """
        参数
        ----
        lat0, lng0
            锚点的纬度、经度（度）。
        size
            六边形边长（米）。100 ~ 2000。
        """
        if not 100 <= size <= 2000:
            raise ValueError(f"边长为 {size} m，应在 100 ~ 2000 m 之间")

        self.lat0 = lat0
        self.lng0 = lng0
        self.size = size
        self._p1 = math.radians(lat0)
        self._l1 = math.radians(lng0)

    # -- 平面坐标 <-> 网格坐标 --------------------------------------------

    def to_world(self, c: Axial) -> tuple[float, float]:
        """单元中心 → 平面坐标（米，相对锚点）。pointy-top 布局。

        **y 轴指向南**（图像坐标习惯，原点在西北角）。算方位角时必须记住
        这一点——用 ``atan2(dx, -dy)`` 而不是 ``atan2(dx, dy)``，
        否则东北会被算成西南，雷达扫描方向整体偏 180°。
        绘图时若希望"北在上"，记得反转 y 轴。
        """
        return (
            self.size * _SQRT3 * (c.q + c.r * 0.5),
            self.size * 1.5 * c.r,
        )

    def from_world(self, x: float, y: float) -> Axial:
        """平面坐标 → 所属单元。

        先反解出浮点轴向坐标，再 ``cube_round`` 吸附到最近的合法单元。
        直接对两个分量分别取整是错的——那会在单元边界附近给出错误的格子。
        """
        r_f = y / (1.5 * self.size)
        q_f = x / (_SQRT3 * self.size) - r_f * 0.5
        return cube_round(q_f, r_f)

    # -- 平面坐标 <-> 经纬度（等距方位投影）-------------------------------

    def geo_to_world(self, lat: float, lng: float) -> tuple[float, float]:
        """经纬度 → 平面坐标。从锚点出发的径向距离精确保距。"""
        p = math.radians(lat)
        dl = math.radians(lng) - self._l1

        cos_c = _clamp_unit(
            math.sin(self._p1) * math.sin(p)
            + math.cos(self._p1) * math.cos(p) * math.cos(dl)
        )
        c = math.acos(cos_c)
        if c < 1e-12:
            return 0.0, 0.0

        k = c / math.sin(c)
        x = k * math.cos(p) * math.sin(dl)
        y = k * (
            math.cos(self._p1) * math.sin(p)
            - math.sin(self._p1) * math.cos(p) * math.cos(dl)
        )
        return x * EARTH_RADIUS, y * EARTH_RADIUS

    def world_to_geo(self, x: float, y: float) -> tuple[float, float]:
        """平面坐标 → 经纬度。AED 反解，闭式，无需迭代。

        注意两个 ρ 不是一回事：``rho_m`` 是投影平面上的**米**制距离，
        而 ``c = rho_m / R`` 才是球面上的角距（弧度）。
        Snyder 公式里分母用的是前者——用成后者会让反解结果飞到地球另一端。
        """
        rho_m = math.hypot(x, y)
        if rho_m < 1e-9:
            return self.lat0, self.lng0

        c = rho_m / EARTH_RADIUS
        sin_c, cos_c = math.sin(c), math.cos(c)

        p = math.asin(
            _clamp_unit(
                cos_c * math.sin(self._p1)
                + y * sin_c * math.cos(self._p1) / rho_m
            )
        )
        lng = self._l1 + math.atan2(
            x * sin_c,
            rho_m * math.cos(self._p1) * cos_c
            - y * math.sin(self._p1) * sin_c,
        )

        return math.degrees(p), _normalize_lng(math.degrees(lng))

    # -- 网格坐标 <-> 经纬度（便捷组合）-----------------------------------

    def to_geo(self, c: Axial) -> tuple[float, float]:
        x, y = self.to_world(c)
        return self.world_to_geo(x, y)

    def from_geo(self, lat: float, lng: float) -> Axial:
        x, y = self.geo_to_world(lat, lng)
        return self.from_world(x, y)

    def __repr__(self) -> str:
        return f"<LocalFrame ({self.lat0:.4f}, {self.lng0:.4f}) size={self.size}>"


def _normalize_lng(lng: float) -> float:
    """把经度规整到 [-180, 180)。

    AED 反解在锚点靠近 ±180° 时会算出越界的经度（如 190°），
    必须绕回来，否则后续与 H3 的交互会出错。
    """
    lng = math.fmod(lng + 180.0, 360.0)
    if lng < 0.0:
        lng += 360.0
    return lng - 180.0
