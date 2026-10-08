"""A* 寻路，支持直接搜索与分层搜索的自动切换。

为什么需要分层
--------------
A* 的复杂度随搜索空间增长。战术级几十公里还好，但几百公里的洲际机动在
100 m 网格上是几百万个节点——直接搜索会把单次寻路拖到几十秒。

分层做法：先在**粗网格**（每 8×8 个细格聚合成一个粗格）上搜出一条走廊，
再在细网格上逐段细化。搜索空间缩小约 64 倍，代价是粗层的乐观估计偶尔会
给出走不通的走廊——那时回退到直接搜索。

启发式必须可采纳
----------------
``CostConfig.min_cost`` 是"任何一格的最低通行代价"，启发式取
``六边形距离 × min_cost``。这个下界不能高估，否则 A* 不再保证最优解。
如果要更快的次优解，调 ``heuristic_weight > 1``（加权 A*）——那是显式的
取舍，而不是偷偷破坏可采纳性。

通行是**运载器**的解释，不是地形的属性
--------------------------------------
`move_cost` 这条通道是被**地面口径**烘死的：水域 = 不可通行（§3.11.3）。
舰船要的恰好相反，所以本模块把"能不能走"从通道里拿出来，交给
:class:`CostConfig`：

- ``costs=None``（默认）→ 读 ``move_cost`` 通道，即地面口径，行为与从前一致
- ``costs={地形类型: 代价}`` → 由**运载器**给表，地形只提供事实
  （``terrain`` / ``elevation`` / 派生的水深）

现成的三张画像：:data:`GROUND_TRAVEL` / :data:`WATER_TRAVEL` /
:data:`AMPHIBIOUS_TRAVEL`。

**凡是用到"能不能走"的地方都必须走 config**，包括起点检查、目标检查与
粗层采样。早先这几处直接调 `grid.is_passable()`（地面口径），后果是
**舰船的寻路当场返回"走不通"**——因为它的起点在水里，而"水里不可通行"。
不报错、不告警，只是永远找不到路。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, replace
from itertools import count
from math import sqrt
from typing import Mapping

from ...errors import ConfigurationError
from .grid import IMPASSABLE, LEVEL_SURFACE, HexGrid
from .hex import (
    DIRECTIONS,
    Axial,
    axial_to_offset,
    distance as hex_distance,
    neighbors,
    offset_to_axial,
)
from .terrain import (
    AMPHIBIOUS_COST_TABLE,
    MOVE_COST_TABLE,
    TERRAIN_NAMES,
    WATER_COST_TABLE,
)

INF = float("inf")

_SQRT3 = sqrt(3.0)

#: 超过这个格数就切到分层搜索。按格数而非距离判断——搜索空间的大小取决于
#: 格数，与格边长无关。120 格直接搜索约展开 1.4 万节点，实测在毫秒级。
DIRECT_SEARCH_MAX_CELLS = 120

#: 粗网格的聚合系数：每个粗格覆盖 8×8 个细格。
COARSE_FACTOR = 8


@dataclass(slots=True)
class CostConfig:
    """通行代价的参数。

    **这是运载器的画像**：同一片地形，坦克、舰船、两栖车看到的是三张
    不同的成本表。

    画像之外还有三个**门槛**，它们描述的是"这辆车过不去什么"，而不是
    "这辆车走得有多慢"：

    - :attr:`blocked_terrain` —— 过不去哪些**地貌**（轮式进不了山地）；
    - :attr:`min_water_depth` —— 至少要多深的水（舰船吃水、潜艇潜深）；
    - :attr:`max_slope` —— 爬得上多陡的坡。

    为什么门槛不并进 ``costs`` 表里：表是**介质**给的（陆地这片地形对
    一般地面车辆意味着什么），门槛是**具体底盘**给的。并进去的话，"轮式"
    与"履带"就得各存一张表，两张表里的平原代价迟早不同步——而不报错。
    做减法只需改一个数字，做"另一张表"要把六个数字都抄一遍。
    """

    #: 启发式下界（每格的最低通行代价）。必须 ≤ 实际最小单步代价，
    #: 否则破坏可采纳性。默认 1 对应平原的 move_cost。
    #: 给了 ``costs`` 时，构造时会校验它不超过表里的最小值。
    min_cost: float = 1.0

    #: 坡度惩罚权重。实际单步代价 = 基础代价 × (1 + slope_weight × 坡度)。
    #: 坡度 = 高差 / 相邻格中心距（无量纲）。
    #: **舰船要设 0**——船不吃坡度，给它叠坡度惩罚会让航线被海底起伏带偏。
    slope_weight: float = 2.0

    #: 按**地形类型**给的成本表。``None`` 表示读 ``move_cost`` 通道
    #: （地面口径，与从前一致）。
    #:
    #: 为什么按地形类型而不是"存多套 move_cost 通道"：同一格存两套成本，
    #: 两套数字会各自演化，迟早不同步；而地形类型是**事实**，只有一份。
    costs: Mapping[int, float] | None = None

    #: 在哪个**垂向层**上寻路（§3.11）。默认 0 = 地表带。
    #: 空中寻路暂时也用 0——天空没有分层数据可读，硬切成空层只是增加空块。
    level: int = LEVEL_SURFACE

    #: 启发式权重。1.0 是标准 A*（保证最优）；> 1 是加权 A*，
    #: 搜索更快但可能给出次优路径。回退路径会用 1.3 来止损。
    heuristic_weight: float = 1.0

    #: 单次搜索展开节点上限。防止超长寻路把主线程卡死——
    #: 宁可返回"找不到"让上层降级（改走直线），也不要静默卡住整个推演。
    max_expansions: int = 400_000

    #: 过不去的**地貌编号**。空集 = 什么地貌都能走。
    #: 这是"可通行能力"，与"走得快慢"是两件事：轮式与履带在平原上只是
    #: 速度不同，到了山地是**能不能去**的区别。
    blocked_terrain: frozenset[int] = frozenset()

    #: 最小水深（米）。该格水深不足即不可通行；``0`` = 不限制。
    #: 水面舰写吃水；**水下件写"定深 + 离底余量"**——同一个门槛，两个名字。
    #: 注意它对潜艇**不是反的**：航渡时水越深越好，约束是单边的
    #: （``水深 ≥ 定深 + 离底余量``）。真正双边的是 **z** 的可行区间
    #: （见 :class:`~milsim.models.mover.SubsurfaceMover`），不是水柱高度。
    min_water_depth: float = 0.0

    #: 坡度硬上限（无量纲 = 高差 / 格中心距）。超过即不可通行；
    #: ``None`` = 不限制。0 不是"只能走水平格"而是一个**错误值**——
    #: 构造时会报错，免得"忘了把参数里的 0（不限制）翻译成 None"这种错
    #: 悄悄把整张图封死。
    max_slope: float | None = None

    def __post_init__(self) -> None:
        if self.min_cost <= 0.0:
            raise ConfigurationError(
                f"启发式下界 min_cost 必须为正，实际 {self.min_cost}"
            )
        if self.min_water_depth < 0.0:
            raise ConfigurationError(
                f"最小水深不能为负，实际 {self.min_water_depth}"
            )
        if self.max_slope is not None and self.max_slope <= 0.0:
            raise ConfigurationError(
                f"坡度上限必须为正（不限制请写 None，不要写 0），"
                f"实际 {self.max_slope}"
            )
        unknown = [c for c in self.blocked_terrain if c not in TERRAIN_NAMES]
        if unknown:
            raise ConfigurationError(
                "blocked_terrain 里有未知的地貌编号 "
                + "、".join(str(c) for c in sorted(unknown))
                + "——写错编号不会报错，只会表现为'这条门槛没生效'"
            )
        if self.costs is None:
            return
        best = min(self.costs.values())
        if self.min_cost > best:
            raise ConfigurationError(
                f"启发式下界 min_cost={self.min_cost} 大于成本表的最小值 "
                f"{best}——启发式会高估代价，破坏 A* 的可采纳性：路径不再保证"
                "最优，而且**不会有任何报错**"
            )

    def cost_at(self, grid: HexGrid, cell: Axial) -> float:
        """这一格的单步基础代价（不含坡度）。不可通行返回 ``IMPASSABLE``。

        **所有"能不能走"的判断都要走这里**，不要直接读 ``grid.move_cost()``
        ——那条通道是地面口径的，会让舰船的起点被判成不可通行。

        三个**节点级**门槛也在这里判：地貌禁行与最小水深都只看这一格，
        与从哪一格过来无关。坡度是**边**的属性，留给 :func:`step_cost`。
        """
        if self.blocked_terrain:
            if int(grid.terrain(cell, self.level)) in self.blocked_terrain:
                return IMPASSABLE
        if self.min_water_depth > 0.0:
            if float(grid.water_depth(cell, self.level)) < self.min_water_depth:
                return IMPASSABLE
        if self.costs is None:
            return float(grid.move_cost(cell, self.level))
        return float(self.costs.get(grid.terrain(cell, self.level), IMPASSABLE))

    def is_passable(self, grid: HexGrid, cell: Axial) -> bool:
        return self.cost_at(grid, cell) < IMPASSABLE

    def gate_reason(self, grid: HexGrid, cell: Axial) -> str:
        """这一格过不去的**原因**（人话）；过得去返回空串。

        逐条门槛查一遍并把**数字**带出来。"水深 3.0 m，不到要求的 70.0 m"
        比"目的格不可通行"省掉一次翻地图的工夫——而那是**必须**省掉的：
        潜艇被拦下来时地貌写的仍是**水域**，只报地貌会把人引向错误的方向
        （去查"为什么水域不能走"，而答案在深度里）。

        诊断用，所以不追求与 :meth:`cost_at` 的判定顺序一致（两者看的是
        同一批门槛，只是这里要按"最能说清问题"的顺序排）。
        """
        terrain = int(grid.terrain(cell, self.level))
        name = TERRAIN_NAMES.get(terrain, "")
        if terrain in self.blocked_terrain:
            return f"地貌是{name or terrain}，这类底盘过不去"
        if self.min_water_depth > 0.0:
            depth = float(grid.water_depth(cell, self.level))
            if depth < self.min_water_depth:
                return (
                    f"水深 {depth:.1f} m，不到要求的 {self.min_water_depth:.1f} m"
                )
        if self.costs is None:
            blocked = float(grid.move_cost(cell, self.level)) >= IMPASSABLE
        else:
            blocked = float(self.costs.get(terrain, IMPASSABLE)) >= IMPASSABLE
        if blocked:
            return f"地貌是{name or terrain}，这类运载器走不了"
        return ""

    def with_heuristic_weight(self, weight: float) -> "CostConfig":
        """同画像、只改启发式权重。回退搜索用（换画像的代价是错的）。"""
        return replace(self, heuristic_weight=weight)

    def with_capabilities(
        self,
        *,
        blocked_terrain: frozenset[int] = frozenset(),
        min_water_depth: float = 0.0,
        max_slope: float | None = None,
    ) -> "CostConfig":
        """在这张画像上叠一层"这辆车的通行能力"，返回**新**配置。

        为什么不原地改：``GROUND_TRAVEL`` 这类是模块级共享对象，就地改会把
        门槛加到所有用同一张画像的实体上——表现为"改了一辆车，整个战区的
        车都过不去山地了"，而且不报错。所以这里一律返回新对象。

        为什么用 ``dataclasses.replace`` 而不是手抄字段：手抄的写法每加一个
        字段就多一处漏抄的机会，漏抄的后果是"这个门槛在某些调用路径上不
        生效"——正是本类要防的那类问题。
        """
        return replace(
            self,
            blocked_terrain=blocked_terrain,
            min_water_depth=min_water_depth,
            max_slope=max_slope,
        )


#: 地面部队画像：读 ``move_cost`` 通道（默认口径，等价于 ``costs=None``）。
GROUND_TRAVEL = CostConfig(
    costs=MOVE_COST_TABLE, slope_weight=2.0, min_cost=1.0
)

#: 舰船画像：**水域可通行、陆地不可**，与地面表恰好互补；不吃坡度。
WATER_TRAVEL = CostConfig(
    costs=WATER_COST_TABLE, slope_weight=0.0, min_cost=1.0
)

#: 两栖画像：水陆都能走，山地不可。
AMPHIBIOUS_TRAVEL = CostConfig(
    costs=AMPHIBIOUS_COST_TABLE, slope_weight=1.0, min_cost=2.0
)


@dataclass(slots=True)
class PathResult:
    """寻路结果。"""

    cells: list[Axial]
    cost: float
    expanded: int
    used_hierarchy: bool = False
    #: 分层失败后回退到直接搜索（且用了加权启发式）。
    #: 路径仍然可行，但不保证最优。
    degraded: bool = False

    @property
    def length_cells(self) -> int:
        return len(self.cells)

    def __repr__(self) -> str:
        return (
            f"<Path {len(self.cells)} 格 代价={self.cost:.1f} "
            f"展开={self.expanded} 分层={self.used_hierarchy}"
            f"{' 降级' if self.degraded else ''}>"
        )


# ---------------------------------------------------------------------------
# 代价
# ---------------------------------------------------------------------------

def step_cost(grid: HexGrid, frm: Axial, to: Axial, config: CostConfig) -> float:
    """从 frm 走到 to 的代价。不可通行返回 ``INF``。

    基础代价来自 **config 的画像**，不是 ``move_cost`` 通道——见模块头。

    坡度**上限**（:attr:`CostConfig.max_slope`）在这里判，不在
    :meth:`CostConfig.cost_at` 里：坡度是**边**的属性，"这一格能不能上"
    取决于从哪一格来，而 ``cost_at`` 只看得到目标格自己。
    """
    base = config.cost_at(grid, to)
    if base >= IMPASSABLE:
        return INF

    # pointy-top 下相邻格中心距 = 边长 × √3
    ground = grid.size * _SQRT3
    if ground > 0 and (config.slope_weight or config.max_slope is not None):
        dh = grid.elevation(to, config.level) - grid.elevation(frm, config.level)
        slope = abs(dh) / ground
        if config.max_slope is not None and slope > config.max_slope:
            return INF
        if config.slope_weight:
            base *= 1.0 + config.slope_weight * slope
    return base


def heuristic(frm: Axial, to: Axial, config: CostConfig) -> float:
    """到目标的代价下界。距离乘最低单步代价——不高估即可采纳。"""
    return hex_distance(frm, to) * config.min_cost * config.heuristic_weight


# ---------------------------------------------------------------------------
# 直接搜索
# ---------------------------------------------------------------------------

def _reconstruct(came_from: dict[Axial, Axial], current: Axial) -> list[Axial]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _astar(
    grid: HexGrid,
    start: Axial,
    goal: Axial,
    config: CostConfig,
    forbidden: set[Axial] | None,
    allowed: set[Axial] | None = None,
) -> PathResult | None:
    """标准 A*。

    ``allowed`` 限定搜索范围（走廊搜索用）。限定范围会缩小搜索空间，
    但不影响结果的最优性——只要真正的最优路径落在范围内。
    """
    if start == goal:
        return PathResult([start], 0.0, 0)

    if not config.is_passable(grid, goal):
        return None

    counter = count()
    open_heap: list[tuple[float, int, Axial]] = [
        (heuristic(start, goal, config), next(counter), start)
    ]
    came_from: dict[Axial, Axial] = {}
    g_score: dict[Axial, float] = {start: 0.0}
    closed: set[Axial] = set()

    expanded = 0
    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            return PathResult(
                _reconstruct(came_from, current), g_score[current], expanded
            )
        if current in closed:
            continue
        closed.add(current)

        expanded += 1
        if expanded > config.max_expansions:
            # 不抛异常：长距离寻路失败是可预期的，交给上层降级处理
            return None

        g_current = g_score[current]
        for nb in neighbors(current):
            if nb in closed:
                continue
            if forbidden is not None and nb in forbidden:
                continue
            if allowed is not None and nb not in allowed:
                continue

            cost = step_cost(grid, current, nb, config)
            if cost == INF:
                continue

            tentative = g_current + cost
            if tentative < g_score.get(nb, INF):
                g_score[nb] = tentative
                came_from[nb] = current
                heapq.heappush(
                    open_heap, (tentative + heuristic(nb, goal, config), next(counter), nb)
                )

    return None


# ---------------------------------------------------------------------------
# 分层搜索
# ---------------------------------------------------------------------------

class CoarseView:
    """细网格的粗粒度视图，供分层寻路的粗层使用。

    粗格代价用**稀疏采样取最小值**，而不是遍历全部 64 个细格：
    遍历会让粗层搜索慢到抵消分层带来的收益。采样 5 个点（中心 + 四角）
    已经能避开"中心正好落在水里"的误判。
    """

    __slots__ = ("grid", "factor", "config", "_cost_cache")

    def __init__(self, grid: HexGrid, config: CostConfig, factor: int = COARSE_FACTOR) -> None:
        self.grid = grid
        self.factor = factor
        self.config = config
        self._cost_cache: dict[tuple[int, int], float] = {}

    def to_coarse(self, cell: Axial) -> tuple[int, int]:
        col, row = axial_to_offset(cell)
        return col // self.factor, row // self.factor

    def to_axial(self, ccol: int, crow: int) -> Axial:
        """粗格的代表点：中心处的细格。"""
        half = self.factor // 2
        return offset_to_axial(ccol * self.factor + half, crow * self.factor + half)

    def coarse_neighbors(self, ccol: int, crow: int) -> list[tuple[int, int]]:
        """粗格的六邻居。

        **必须沿方向走 ``factor`` 步**，不能只看代表点的 1 格邻居——
        粗格有 factor 格宽，走 1 步必然还落在同一个粗格里，
        结果是"所有粗格都没有邻居"，粗层搜索立刻失败。

        这个错误很隐蔽：代码不报错，只是分层永远失败并静默回退到直接搜索，
        表现为"寻路能用但分层从没生效过"。
        """
        center = self.to_axial(ccol, crow)
        seen: dict[tuple[int, int], None] = {}
        for direction in DIRECTIONS:
            moved = Axial(
                center.q + direction.q * self.factor,
                center.r + direction.r * self.factor,
            )
            key = self.to_coarse(moved)
            if key != (ccol, crow):
                seen[key] = None
        return list(seen)

    def cost(self, frm: tuple[int, int], to: tuple[int, int]) -> float:
        """粗格间移动代价。取目标粗格的最小采样代价。"""
        key = to
        cached = self._cost_cache.get(key)
        if cached is None:
            cached = self._sample_cost(*to)
            self._cost_cache[key] = cached
        return cached

    def _sample_cost(self, ccol: int, crow: int) -> float:
        factor = self.factor
        half = factor // 2
        base_col = ccol * factor
        base_row = crow * factor

        offsets = (
            (half, half),
            (0, 0),
            (factor - 1, 0),
            (0, factor - 1),
            (factor - 1, factor - 1),
        )

        best = INF
        for dcol, drow in offsets:
            cell = offset_to_axial(base_col + dcol, base_row + drow)
            # 走 config 的画像，不读 move_cost 通道：粗层若按地面口径采样，
            # 舰船的粗路径会以为陆地可通行，细层再逐段失败、整体降级。
            cost = self.config.cost_at(self.grid, cell)
            if cost < IMPASSABLE:
                best = min(best, float(cost))
        return best


def build_corridor(
    chain: list[tuple[int, int]],
    factor: int = COARSE_FACTOR,
    margin: int | None = None,
) -> set[Axial]:
    """把粗路径展开成细格走廊。

    每个粗格贡献一个 ``factor + 2×margin`` 见方的细格区域，相邻粗格的区域
    互相重叠，保证走廊连通。

    走廊是分层搜索的关键：有了它，细层只需要在粗路径附近搜索，
    而**不必强制穿过粗格中心**。后者会让路径在中心点附近来回折返，
    实测 400 格的直线路径因此多出 4 格。
    """
    if margin is None:
        margin = factor

    span = factor + 2 * margin
    corridor: set[Axial] = set()

    for ccol, crow in chain:
        base_col = ccol * factor
        base_row = crow * factor
        for dcol in range(-margin, span - margin):
            for drow in range(-margin, span - margin):
                corridor.add(offset_to_axial(base_col + dcol, base_row + drow))

    return corridor


def _hierarchical(
    grid: HexGrid,
    start: Axial,
    goal: Axial,
    config: CostConfig,
    forbidden: set[Axial] | None,
) -> PathResult | None:
    """先粗后细：粗层找走廊，再逐段细化，最后平滑。"""
    view = CoarseView(grid, config)
    c_start = view.to_coarse(start)
    c_goal = view.to_coarse(goal)
    if c_start == c_goal:
        return None      # 同格内不值得分层，让调用方直接搜

    # -- 粗层 A* --
    counter = count()
    open_heap: list[tuple[float, int, tuple[int, int]]] = [
        (_coarse_dist(view, c_start, c_goal), next(counter), c_start)
    ]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    g: dict[tuple[int, int], float] = {c_start: 0.0}
    closed: set[tuple[int, int]] = set()
    expanded = 0
    coarse_limit = max(1000, config.max_expansions // (COARSE_FACTOR * COARSE_FACTOR))

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur == c_goal:
            break
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if expanded > coarse_limit:
            return None

        for nb in view.coarse_neighbors(*cur):
            if nb in closed:
                continue
            c = view.cost(cur, nb)
            if c == INF:
                continue
            tentative = g[cur] + c
            if tentative < g.get(nb, INF):
                g[nb] = tentative
                came[nb] = cur
                heapq.heappush(
                    open_heap,
                    (tentative + _coarse_dist(view, nb, c_goal), next(counter), nb),
                )
    else:
        return None      # 堆空仍未到目标

    # -- 还原粗路径 --
    chain = [c_goal]
    node = c_goal
    while node in came:
        node = came[node]
        chain.append(node)
    chain.reverse()

    # -- 走廊搜索 --
    #
    # 不在粗格中心之间逐段连接。那样会强制路径穿过每个中心点，在中心附近
    # 来回折返——实测 400 格的直线路径因此多出 4 格，而且事后平滑很难修得干净。
    #
    # 改为：把整条粗路径展开成走廊，在走廊内做**一次** A*。
    # 路径自然就是走廊内的最优解，既没有折返，也不需要平滑。
    corridor = build_corridor(chain)
    corridor.add(start)
    corridor.add(goal)

    result = _astar(grid, start, goal, config, forbidden, allowed=corridor)
    if result is None:
        return None      # 粗层判断失误，交给上层回退全图搜索

    return PathResult(
        result.cells,
        result.cost,
        result.expanded + expanded,
        used_hierarchy=True,
    )


def path_cost(grid: HexGrid, cells: list[Axial], config: CostConfig) -> float:
    """累计一条路径的代价。不可通行返回 ``INF``。"""
    total = 0.0
    for a, b in zip(cells, cells[1:]):
        cost = step_cost(grid, a, b, config)
        if cost == INF:
            return INF
        total += cost
    return total


def _coarse_dist(view: CoarseView, a: tuple[int, int], b: tuple[int, int]) -> float:
    """粗格启发式距离。

    取两个粗格**代表点**的六边形距离再除以聚合系数，作为细格距离的下界。

    不能拿粗格坐标直接套立方距离公式——粗格坐标是**偏移坐标**
    ``(col, row)``，立方距离只在轴向坐标下成立。两者在同一行上会碰巧
    相等，换个方向就错，属于典型的"测试时看不出、上线才炸"的坑。
    """
    return hex_distance(view.to_axial(*a), view.to_axial(*b)) / view.factor


# ---------------------------------------------------------------------------
# 对外入口
# ---------------------------------------------------------------------------

def find_path(
    grid: HexGrid,
    start: Axial,
    goal: Axial,
    *,
    forbidden: set[Axial] | None = None,
    config: CostConfig | None = None,
) -> PathResult | None:
    """寻路。按距离自动选择直接搜索或分层搜索。

    返回 ``None`` 表示确实走不通（目标不可通行、被完全包围、或超出展开上限）。
    调用方应当有降级策略——通常是改走直线，而不是让实体停在原地。
    """
    config = config or CostConfig()

    if start == goal:
        return PathResult([start], 0.0, 0)
    if not config.is_passable(grid, start):
        return None

    span = hex_distance(start, goal)

    if span <= DIRECT_SEARCH_MAX_CELLS:
        return _astar(grid, start, goal, config, forbidden)

    result = _hierarchical(grid, start, goal, config, forbidden)
    if result is not None:
        return result

    # 分层失败（粗走廊实际走不通，或段细化超限）→ 回退直接搜索。
    # 用加权启发式止损：路径可能次优，但总比让实体停在原地好。
    fallback_config = config.with_heuristic_weight(
        max(config.heuristic_weight, 1.3)
    )
    result = _astar(grid, start, goal, fallback_config, forbidden)
    if result is not None:
        result.degraded = True
    return result


def reachable_cells(
    grid: HexGrid,
    start: Axial,
    budget: float,
    config: CostConfig | None = None,
) -> dict[Axial, float]:
    """以 start 为中心做代价受限的 Dijkstra，返回可达单元及其代价。

    用于回答"这个单位在 X 秒内能到哪"——比逐目标寻路高效得多。
    """
    config = config or CostConfig()
    if not config.is_passable(grid, start):
        return {}

    counter = count()
    heap: list[tuple[float, int, Axial]] = [(0.0, next(counter), start)]
    best: dict[Axial, float] = {start: 0.0}

    while heap:
        g, _, cur = heapq.heappop(heap)
        if g > best.get(cur, INF):
            continue

        for nb in neighbors(cur):
            cost = step_cost(grid, cur, nb, config)
            if cost == INF:
                continue
            tentative = g + cost
            if tentative <= budget and tentative < best.get(nb, INF):
                best[nb] = tentative
                heapq.heappush(heap, (tentative, next(counter), nb))

    return best
