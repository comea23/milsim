"""通视判定。

必须考虑地球曲率
----------------
100 km 外地面因曲率下沉 ``d²/(2R)`` ≈ **785 米**，200 km 外 3.14 公里。
不考虑它的话，通视判定会系统性偏乐观——平地上的雷达会"看见"几百公里外
根本不在视线内的目标。这在军事上就是"雷达地平线"，不是可忽略的二阶效应。

大气折射
--------
电磁波在标准大气中会向下弯曲，等效地球半径约为真实半径的 4/3。
雷达和通信通常按 **4/3 地球模型**计算；光学器材接近真实半径。
``refraction_factor`` 默认 4/3，做光电探测时传 1.0。

精度取舍
--------
沿格心连线逐格采样，属于**廉价近似**：把起伏地形的物理通视简化为逐格判定。
单元尺度以下的起伏会被抹平。任务级仿真接受这个近似，而且它确定性好、
与网格寻路共用同一套坐标。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, sqrt

from .grid import HexGrid
from .hex import EARTH_RADIUS, Axial, line

#: 标准大气下的等效地球半径系数。
DEFAULT_REFRACTION = 4.0 / 3.0

#: 单次通视判定的最大采样点数。超长距离会降采样——
#: 500 km 在 100 m 网格上是 5000 个采样点，逐点查询高程太慢。
#: 代价是可能漏掉窄而尖锐的山脊，这是可接受的精度换取。
MAX_SAMPLES = 400


@dataclass(slots=True)
class LineOfSight:
    """通视结果。"""

    visible: bool
    distance_m: float
    #: 遮挡最严重的单元。可见时为 None。
    blocking_cell: Axial | None = None
    blocking_elevation: float = 0.0
    #: 最小余隙（米）。正值表示视线在各地形之上；负值表示被遮挡的量。
    min_clearance: float = 0.0
    #: 观察者需要抬高多少米才能建立通视。0 表示无需抬高。
    required_raise_m: float = 0.0
    samples: int = 0

    def __bool__(self) -> bool:
        return self.visible

    def __repr__(self) -> str:
        state = "通视" if self.visible else f"遮挡({self.min_clearance:.0f}m)"
        return f"<LoS {self.distance_m/1000:.1f}km {state} 采样={self.samples}>"


def check_line_of_sight(
    grid: HexGrid,
    observer: Axial,
    target: Axial,
    observer_height: float,
    target_height: float,
    *,
    earth_curvature: bool = True,
    refraction_factor: float = DEFAULT_REFRACTION,
    required_clearance: float = 0.0,
    max_samples: int = MAX_SAMPLES,
    terrain_floor: float | None = None,
) -> LineOfSight:
    """判定两点之间是否通视。

    .. note::

        **无线电地平线不是另加的一条判据**——它就是 ``earth_curvature=True``
        那部分几何的结果。30 m 天线对 10 m 目标的距离上限 32 km 会自己浮现：
        越过地平线后曲率下沉量超过视线高度，``min_clearance`` 转负。所以
        调用方**不要**再单独调一次 :func:`radio_horizon_m` 去提前剔除——
        同一个量两处各判一次，迟早会出现"过了地平线却报通视"的不一致。

    参数
    ----
    observer_height, target_height
        相对地面的高度（米）。海拔由网格高程提供。
    required_clearance
        要求的最小余隙（米）。雷达需要一定净空才能稳定跟踪，
        传 0 表示只要不被地形挡住即可。
    terrain_floor
        地形高程的**下限**（米）。``None``（默认）表示把网格高程当地表——
        陆地上这就是对的。但**水域通道里存的是海床**（见 §3.11 的通道
        口径，某格实测 −423.5 m），于是海面上一条 z = 0 的舰船会算出
        ``h = 海床 + 0 = −375 m``，被自己脚下的"地形"挡得严严实实，
        而报出来的理由是"被地形遮挡"——**那个理由是假的**。
        感知 / 交战层传 ``0.0``，表示"水面按海平面算"。
    """
    if terrain_floor is None:
        def ground(cell: Axial) -> float:
            return float(grid.elevation(cell))
    else:
        floor = float(terrain_floor)

        def ground(cell: Axial) -> float:
            return max(float(grid.elevation(cell)), floor)

    if observer == target:
        return LineOfSight(True, 0.0)

    cells = line(observer, target)
    total_cells = len(cells)

    # pointy-top 下相邻格中心距 = 边长 × √3
    step_m = grid.size * sqrt(3.0)
    total_m = (total_cells - 1) * step_m

    # 超长距离降采样。必须保留首尾，且步长要整除——否则终点会被丢掉。
    stride = 1
    if total_cells > max_samples:
        stride = ceil(total_cells / max_samples)
        sampled = cells[::stride]
        if sampled[-1] != cells[-1]:
            sampled.append(cells[-1])
        cells = sampled

    samples = len(cells)
    if samples <= 2:
        return LineOfSight(True, total_m, samples=samples)

    r_eff = EARTH_RADIUS * refraction_factor if earth_curvature else 0.0

    h_observer = ground(cells[0]) + observer_height
    h_target = ground(cells[-1]) + target_height

    min_clearance = float("inf")
    blocking: Axial | None = None
    blocking_elevation = 0.0
    required_raise = 0.0

    for i in range(1, samples - 1):
        cell = cells[i]
        # 采样点在**完整路径**上的比例。降采样后仍要按真实距离算，
        # 否则曲率修正会算错——这是降采样最容易引入的隐蔽 bug。
        t = (i * stride) / (total_cells - 1)
        if t <= 0.0 or t >= 1.0:
            continue

        sight_height = h_observer + (h_target - h_observer) * t

        if r_eff > 0.0:
            d = t * total_m
            sight_height -= d * (total_m - d) / (2.0 * r_eff)

        terrain = ground(cell)
        clearance = sight_height - terrain - required_clearance

        if clearance < min_clearance:
            min_clearance = clearance
            blocking = cell
            blocking_elevation = terrain

        # 只抬观察者时，该点的视线抬升量是 Δ×(1-t)
        if clearance < 0.0:
            denominator = 1.0 - t
            if denominator > 1e-9:
                raise_needed = (-clearance) / denominator
                if raise_needed > required_raise:
                    required_raise = raise_needed

    visible = min_clearance >= 0.0
    return LineOfSight(
        visible=visible,
        distance_m=total_m,
        blocking_cell=None if visible else blocking,
        blocking_elevation=blocking_elevation if not visible else 0.0,
        min_clearance=0.0 if min_clearance == float("inf") else min_clearance,
        required_raise_m=required_raise,
        samples=samples,
    )


def radio_horizon_m(
    observer_height: float,
    target_height: float = 0.0,
    refraction_factor: float = DEFAULT_REFRACTION,
) -> float:
    """无线电地平线距离（米）。

    ``d = √(2·R_eff·h₁) + √(2·R_eff·h₂)``

    这是军事上判断"雷达最远能看到多低的目标"的标准公式。
    例如 30 m 高的雷达天线对 10 m 高的目标，地平线约 32 km——
    再远的低空目标就在地球曲率之下了，与雷达功率无关。
    """
    r_eff = EARTH_RADIUS * refraction_factor
    return sqrt(2.0 * r_eff * max(0.0, observer_height)) + sqrt(
        2.0 * r_eff * max(0.0, target_height)
    )


def visible_targets(
    grid: HexGrid,
    observer: Axial,
    targets: list[Axial],
    observer_height: float,
    target_height: float,
    *,
    required_clearance: float = 0.0,
    earth_curvature: bool = True,
) -> list[Axial]:
    """批量通视判定，返回可见的目标。

    这是传感器探测的入口。目前是逐个判定的循环——真正的加速要走
    数据并行（一次性算完所有目标），那属于模型层的优化，这里先保证正确。
    """
    return [
        t
        for t in targets
        if check_line_of_sight(
            grid,
            observer,
            t,
            observer_height,
            target_height,
            required_clearance=required_clearance,
            earth_curvature=earth_curvature,
        ).visible
    ]


def horizon_drop_m(distance_m: float, refraction_factor: float = DEFAULT_REFRACTION) -> float:
    """给定距离处地球曲率造成的下沉量（米）。

    单独暴露出来，便于在别处核对量级——785 m @ 100 km 这种数字
    看一眼就知道是否漏算了曲率。
    """
    r_eff = EARTH_RADIUS * refraction_factor
    return distance_m * distance_m / (2.0 * r_eff)
