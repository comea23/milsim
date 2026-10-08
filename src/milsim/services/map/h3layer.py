"""全球层：基于 H3 的球面六边形网格。

为什么全球六边形必须用 H3 而不能自研
-------------------------------------
欧拉公式限定了正多面体只有 5 种，其中**没有六边形版本**。所以六边形无法
像四边形那样直接平铺球面，必须借助二十面体（icosahedron）投影——这正是
H3 在做的事。自研这条路要处理面片边界、投影畸变、跨面索引，
Uber 在这上面迭代了八年，没有理由重造。

H3 的分辨率只能取 7 的幂次（每级边长 ×√7 ≈ 2.646），所以它拿不到精确的
100 m / 2 km。这是**全球层只做粗粒度**的根本原因——战术精度交给局部平面
网格（见 hex.py），两者用经纬度衔接。

| res | 平均边长 | 全球单元数 | 典型用途 |
|-----|---------|-----------|---------|
| 4   | 26.1 km | 28.8 万   | 洲际态势 |
| 5   | 9.85 km | 201.7 万  | 跨区机动（默认） |
| 6   | 3.72 km | 1411.8 万 | 战役方向 |
| 7   | 1.41 km | 9882.5 万 | 接近局部层下限 |

存储是**稀疏的**：只记录真正用到的单元，不预分配全球数组。全球 200 万个
单元里，一次推演真正涉及的通常只有几千个。
"""

from __future__ import annotations

import math
from typing import Iterator

try:
    import h3
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "全球层需要 h3 库，请安装：pip install h3"
    ) from exc

from .hex import Axial, LocalFrame

#: 默认全球层分辨率。9.85 km 的粒度足以支撑跨区兵力调动，
#: 同时把"全球只有 200 万个格"这个量级控制在可管理的范围。
DEFAULT_RESOLUTION = 5

#: 各分辨率对应的平均边长（米），由 h3 实测得出，用于选型参考。
_EDGE_LENGTH_CACHE: dict[int, float] = {}


def edge_length_m(resolution: int) -> float:
    """指定分辨率的平均边长（米）。"""
    cached = _EDGE_LENGTH_CACHE.get(resolution)
    if cached is None:
        cached = h3.average_hexagon_edge_length(resolution, unit="m")
        _EDGE_LENGTH_CACHE[resolution] = cached
    return cached


def resolution_for_edge_length(target_m: float) -> int:
    """选出边长最接近 target_m 的分辨率。

    注意 H3 的分辨率是离散的（×√7 递进），返回值只是**最接近**的那个，
    实测边长与目标值可能有 30% 以上的偏差。要精确边长请用局部网格。
    """
    best, best_err = 0, float("inf")
    for res in range(0, 16):
        err = abs(edge_length_m(res) - target_m)
        if err < best_err:
            best, best_err = res, err
    return best


class GlobalLayer:
    """全球层：球面六边形网格，稀疏存储。

    单元标识是 H3 的 64 位整数。转成字符串（如 ``"8531aa43fffffff"``）只用于
    日志和调试——整数在字典里更快、更省内存，也便于排序。
    """

    __slots__ = ("resolution", "_cells", "_default_attrs")

    def __init__(self, resolution: int = DEFAULT_RESOLUTION) -> None:
        if not 0 <= resolution <= 15:
            raise ValueError(f"H3 分辨率应在 0..15，收到 {resolution}")
        self.resolution = resolution
        #: 稀疏表。没记录的单元一律视为默认值（默认是"无数据"）。
        self._cells: dict[int, dict] = {}
        self._default_attrs: dict = {}

    # -- 坐标与拓扑 --------------------------------------------------------

    def cell_at(self, lat: float, lng: float) -> int:
        """经纬度 → 所属单元。"""
        return h3.latlng_to_cell(lat, lng, self.resolution)

    def center(self, cell: int) -> tuple[float, float]:
        """单元中心经纬度 (lat, lng)。"""
        return h3.cell_to_latlng(cell)

    def boundary(self, cell: int) -> list[tuple[float, float]]:
        """单元多边形顶点，用于可视化与区域判定。"""
        return h3.cell_to_boundary(cell)

    def resolution_of(self, cell: int) -> int:
        return h3.get_resolution(cell)

    def distance(self, a: int, b: int) -> int:
        """两单元之间的**格数**距离（不是米）。

        跨多个二十面体面时 H3 无法直接计算，会抛异常——那时请改用
        大圆距离（``great_circle_m``）做粗判。
        """
        return h3.grid_distance(a, b)

    def neighbors(self, cell: int) -> tuple[int, ...]:
        """相邻单元。

        **不是恒定 6 个** —— H3 在二十面体的 12 个顶点处是五边形单元，
        那里只有 5 个邻居。任何"六边形必有 6 邻居"的假设在这里都不成立。

        （h3 的 grid_disk 返回 list，不是 set——这点容易记错。）
        """
        return tuple(c for c in h3.grid_disk(cell, 1) if c != cell)

    def disk(self, cell: int, k: int) -> list[int]:
        """半径 k 格内的所有单元（含自身）。"""
        return list(h3.grid_disk(cell, k))

    def ring(self, cell: int, k: int) -> list[int]:
        """半径恰为 k 的一圈。"""
        return list(h3.grid_ring(cell, k))

    # -- 层级（H3 的父子嵌套是精确的）--------------------------------------

    def parent(self, cell: int, resolution: int) -> int:
        """上卷到指定分辨率。这是 H3 相对自建多层网格的最大优势：
        父子关系精确，聚合不需要近似。"""
        return h3.cell_to_parent(cell, resolution)

    def children(self, cell: int, resolution: int) -> list[int]:
        """下钻到指定分辨率。注意数量随层级指数增长（res 差 1 级 = ×7）。"""
        return h3.cell_to_children(cell, resolution)

    def coarsen(self, cell: int) -> int:
        """上卷一级。"""
        return h3.cell_to_parent(cell, h3.get_resolution(cell) - 1)

    # -- 区域查询 ----------------------------------------------------------

    def cells_within(self, lat: float, lng: float, radius_m: float) -> list[int]:
        """覆盖以 (lat, lng) 为心、radius_m 为半径的圆形区域的所有单元。

        做法是 disk 取超集再按实际距离筛掉边角。k 的估算用平均边长——
        单元大小有 ±20% 的波动，所以估算要留余量。
        """
        center = self.cell_at(lat, lng)
        k = int(math.ceil(radius_m / edge_length_m(self.resolution))) + 1
        return [
            c for c in h3.grid_disk(center, k)
            if great_circle_m(lat, lng, *self.center(c)) <= radius_m
        ]

    def cells_in_bbox(
        self, lat_min: float, lng_min: float, lat_max: float, lng_max: float
    ) -> list[int]:
        """覆盖经纬度矩形的所有单元。

        用 H3 的多边形接口。注意跨 ±180° 经线的矩形需要调用方先拆成两块，
        这个函数不做自动拆分——静默拆分会让调用方以为拿到了完整结果。
        """
        if lng_min > lng_max:
            raise ValueError(
                "跨 ±180° 经线的范围请拆成两个矩形分别调用，本函数不自动处理"
            )
        polygon = [
            (lat_min, lng_min), (lat_min, lng_max),
            (lat_max, lng_max), (lat_max, lng_min),
        ]
        shape = h3.LatLngPoly(polygon)
        return list(h3.h3shape_to_cells(shape, self.resolution))

    # -- 稀疏属性 ----------------------------------------------------------

    def attrs(self, cell: int) -> dict:
        """读取单元属性。没有记录时返回默认值，不自动创建条目——
        查询不应该有副作用，否则遍历一遍全球就把内存撑爆了。"""
        return self._cells.get(cell, self._default_attrs)

    def set_attrs(self, cell: int, **values) -> None:
        entry = self._cells.get(cell)
        if entry is None:
            entry = dict(self._default_attrs)
            self._cells[cell] = entry
        entry.update(values)

    def __contains__(self, cell: int) -> bool:
        return cell in self._cells

    def __len__(self) -> int:
        return len(self._cells)

    def __iter__(self) -> Iterator[int]:
        """遍历**已记录**的单元。顺序是插入序，可复现。"""
        return iter(self._cells)

    def clear(self) -> None:
        self._cells.clear()

    # -- 与局部网格的衔接 --------------------------------------------------

    def to_local(self, cell: int, frame: LocalFrame) -> Axial:
        """全球单元 → 局部网格坐标。

        精度取决于两者是否在同一个战区：单元中心可能落在 frame 覆盖范围之外，
        此时返回的坐标虽然在数学上合法，但已经远离锚点、投影误差不可忽略了。
        调用前应确认 cell 落在 frame 所属战区范围内（由 zone.py 负责）。
        """
        lat, lng = self.center(cell)
        return frame.from_geo(lat, lng)

    def from_local(self, cell: Axial, frame: LocalFrame) -> int:
        """局部网格坐标 → 全球单元。"""
        lat, lng = frame.to_geo(cell)
        return self.cell_at(lat, lng)

    def __repr__(self) -> str:
        return (
            f"<GlobalLayer res={self.resolution} "
            f"({edge_length_m(self.resolution) / 1000:.2f} km) "
            f"tracked={len(self._cells)}>"
        )


# ---------------------------------------------------------------------------
# 测地辅助
# ---------------------------------------------------------------------------

def great_circle_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    """球面大圆距离（米）。haversine 形式，在短距离上数值稳定。

    **不要**用 ``h3.great_circle_distance`` 做这件事——它的 unit 参数行为
    与直觉不符（实测传 unit='m' 返回的数值对不上米），而且我们需要的只是
    一个明确的球面距离。自己实现 20 行，行为完全可控。
    """
    p1, l1 = math.radians(lat1), math.radians(lng1)
    p2, l2 = math.radians(lat2), math.radians(lng2)
    dp = p2 - p1
    dl = l2 - l1
    a = (
        math.sin(dp * 0.5) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl * 0.5) ** 2
    )
    return 2.0 * 6_371_008.8 * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    """初始方位角（度，正北为 0，顺时针）。用于把航线转成航向。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lng2 - lng1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def interpolate_great_circle(
    lat1: float, lng1: float, lat2: float, lng2: float, fraction: float
) -> tuple[float, float]:
    """沿大圆插值。fraction=0 取起点，=1 取终点。

    跨洲机动时用这个生成中间路点——直接对经纬度做线性插值会在高纬度
    跑出明显偏离大圆的航线。
    """
    p1, l1 = math.radians(lat1), math.radians(lng1)
    p2, l2 = math.radians(lat2), math.radians(lng2)

    d = 2.0 * math.asin(
        min(
            1.0,
            math.sqrt(
                math.sin((p2 - p1) * 0.5) ** 2
                + math.cos(p1) * math.cos(p2) * math.sin((l2 - l1) * 0.5) ** 2
            ),
        )
    )
    if d < 1e-12:
        return lat1, lng1

    sin_d = math.sin(d)
    a = math.sin((1.0 - fraction) * d) / sin_d
    b = math.sin(fraction * d) / sin_d

    x = a * math.cos(p1) * math.cos(l1) + b * math.cos(p2) * math.cos(l2)
    y = a * math.cos(p1) * math.sin(l1) + b * math.cos(p2) * math.sin(l2)
    z = a * math.sin(p1) + b * math.sin(p2)

    return (
        math.degrees(math.atan2(z, math.hypot(x, y))),
        math.degrees(math.atan2(y, x)),
    )


def sample_great_circle(
    lat1: float, lng1: float, lat2: float, lng2: float, step_m: float
) -> list[tuple[float, float]]:
    """按固定地面步长采样大圆航线，含首尾。"""
    total = great_circle_m(lat1, lng1, lat2, lng2)
    if total <= step_m:
        return [(lat1, lng1), (lat2, lng2)]

    count = int(math.ceil(total / step_m))
    points = [
        interpolate_great_circle(lat1, lng1, lat2, lng2, i / count)
        for i in range(count + 1)
    ]
    # 首尾直接用原始值覆盖：插值在 fraction=0/1 处会引入 ULP 级漂移
    # （实测 116.4074 变成 116.40740000000001），调用方拿它去查 H3 单元
    # 可能落到相邻格。这是必须钉死的，不是洁癖。
    points[0] = (lat1, lng1)
    points[-1] = (lat2, lng2)
    return points


def cells_for_route(
    layer: GlobalLayer,
    lat1: float,
    lng1: float,
    lat2: float,
    lng2: float,
) -> list[int]:
    """一条大圆航线经过的全球层单元序列（去重，保持先后顺序）。

    跨洲机动的粗路径就靠这个：全球层粒度 9.85 km，一条洲际航线大约
    产生几百到几千个单元，完全够用。进入战区后再由局部网格接管。
    """
    from collections import OrderedDict

    step = edge_length_m(layer.resolution) * 0.5
    ordered: "OrderedDict[int, None]" = OrderedDict()
    for lat, lng in sample_great_circle(lat1, lng1, lat2, lng2, step):
        ordered[layer.cell_at(lat, lng)] = None
    return list(ordered)
