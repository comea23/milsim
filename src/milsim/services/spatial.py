"""空间索引：通过网格编码快速查询实体。

典型用法（雷达探测）
--------------------
::

    origin = index.entity_cell(radar_id)              # 雷达所在网格
    candidates = index.entities_in_cone(              # 向某方向搜索
        origin, direction=DIR_NE, max_steps=40, half_width=12
    )
    for target_id in candidates:
        if has_line_of_sight(...) and within_range(...):   # 精筛
            report(target_id)

索引只做**粗筛**：哪些格子里有东西。几何与物理判定（通视、探测概率、
信号强度）由模型层负责。这样索引保持纯粹——它不懂雷达，只懂"谁在哪个格子里"。

为什么不用"遍历所有实体"代替
----------------------------
实体少时遍历确实更简单，但任务级推演的实体数会到几千到几万，
而雷达每帧都可能扫描。用索引把"全表扫描"换成"只查雷达覆盖到的格子"，
是这类系统能实时跑起来的前提。

可复现性
--------
格子内的实体列表**始终保持 entity_id 升序**。查询结果的顺序也完全确定
（按距离由近及远、同距离按网格方向）。

这一点是刻意的：如果内部用 ``set`` 存储，遍历顺序随哈希值变化，
同一个种子两次推演可能给出不同的发现顺序，进而让后续决策分叉。
"""

from __future__ import annotations

from bisect import insort
from dataclasses import dataclass
from math import atan2, cos, degrees, floor, radians, sin, sqrt
from typing import Final, Iterable, Iterator

from .map.hex import DIRECTIONS, Axial, distance, ring

#: 网格方向与方位角的对应（pointy-top，0° = 正北，顺时针）。
#:
#: `DIRECTIONS` 从正东开始顺时针排列，所以：
#:     0 E→90°, 1 NE→30°, 2 NW→330°, 3 W→270°, 4 SW→210°, 5 SE→150°
BEARING_OF_DIRECTION = (90.0, 30.0, 330.0, 270.0, 210.0, 150.0)

_SQRT3 = sqrt(3.0)


def direction_from_bearing(bearing_deg: float) -> int:
    """方位角（度，正北为 0，顺时针）→ 最接近的网格方向索引。

    六边形只有 6 个离散方向，任意方位角都要吸附到最近的 60° 扇区。

    **不能用内置 ``round()``**：它是银行家舍入，正北（0°）恰好落在
    NE 与 NW 的中间（都是差 30°），``round(1.5)`` 归到 2、``round(-1.5)``
    归到 -2，看似无害；但换成别的角度组合就会出现"北偏西、南偏东"这种
    不对称映射，雷达扫描会有系统性偏移。统一用 ``floor(x + 0.5)``
    保证南北严格对称。
    """
    return int(floor((90.0 - bearing_deg) / 60.0 + 0.5)) % 6


def bearing_of_direction(direction: int) -> float:
    """网格方向索引 → 方位角。"""
    return BEARING_OF_DIRECTION[direction % 6]


#: 六条网格轴**按方位角升序**：相邻两条轴正好夹出一个 60° 楔形。
#:
#: :data:`BEARING_OF_DIRECTION` 是 ``(90, 30, 330, 270, 210, 150)``，升序后
#: 得到 ``(1, 0, 5, 4, 3, 2)``，即方位角 ``30° + 60°·k``。平面被这六条轴
#: 切成六个楔形，任何一个非原点的格都**恰好**落在其中一个里——这是
#: :func:`cells_in_sector` 能"只铺扇区内的格"的全部依据。
_AXIS_ORDER: Final[tuple[int, ...]] = tuple(sorted(range(6), key=bearing_of_direction))


def bearing_of_offset(east: float, south: float, /) -> float:
    """世界坐标偏移 → 方位角（度，正北 0°，顺时针）。

    参数名把**符号约定钉在签名上**：第二个分量是 ``south``（南向）而不是
    ``north``——战区的 y 轴指向南（图像坐标习惯，原点在左上、y 向下，见
    :func:`bearing_between`）。写 ``atan2(东向, 北向)`` 只差一个负号，但整体
    偏 **180°**："目标在西边"会算成"在东边"，而图、数都不报错。

    方位角是**世界坐标**上的量，只在同一战区的局部网格内有意义；跨战区要用
    真实经纬度算。
    """
    return (degrees(atan2(east, -south)) + 360.0) % 360.0


def bearing_between(origin: Axial, target: Axial) -> float:
    """两个轴向坐标之间的方位角（度，正北 0°，顺时针）。

    **符号约定容易搞反**：``LocalFrame.to_world`` 里 ``y = 1.5 × size × r``，
    所以 r 增大对应世界坐标 y 增大。而网格方向 ``DIRECTIONS[1] = (+1, -1)``
    标的是 NE（东北），意味着 y 增大其实指向**南**——这是图像坐标的习惯
    （原点在左上、y 向下）。

    因此方位角是 ``atan2(dx, -dy)``（东向分量、北向分量），
    不是 ``atan2(dx, dy)``。后者会把东北算成西南，雷达扫描方向整体偏 180°。

    **另一个更隐蔽的坑**：dx/dy 必须用**世界坐标米数**的比例，不能直接用
    格数 ``(q + r/2)`` 和 ``r``。两者差一个 ``√3 : 1.5`` 的系数，直接用格数
    会让所有非正东/正西方向的方位角偏差几度——东北算成 26.6° 而不是 30°。
    单看一个方向不容易发现，整个扇区扫下来就是十几公里的偏移。

    仅在**同一战区**的局部网格内有意义——跨战区要用真实经纬度算方位。

    这里只是把格坐标换算成世界偏移再交给 :func:`bearing_of_offset`；
    用真实位置算同一个角度时走那边，两条路必须给出同一个数。
    """
    dq = target.q - origin.q
    dr = target.r - origin.r
    # 世界坐标偏移：东向 = √3·(dq + dr/2)，南向 = 1.5·dr
    return bearing_of_offset(_SQRT3 * (dq + dr * 0.5), 1.5 * dr)


# ---------------------------------------------------------------------------
# 网格几何：生成搜索区域
# ---------------------------------------------------------------------------

def cells_in_cone(
    origin: Axial,
    direction: int,
    max_steps: int,
    half_width: int = 0,
    min_steps: int = 1,
) -> Iterator[Axial]:
    """锥形/扇形区域，按距离由近及远逐层产出。

    ``half_width`` 是每一侧展开的步数，**不随距离增长**——所以得到的是
    一个平行边界的"走廊"。要模拟雷达的扇形张角，用
    :func:`cells_in_sector` 按方位角判定。

    第 k 层的格子 = ``origin + d*k + w``，其中 w 沿 d+1 方向偏移
    ``[-half_width, half_width]``。六边形网格里 ``d+1`` 与 ``d`` 相差 60°，
    所以这个偏移正好铺开成一个锥面。
    """
    if max_steps < 1:
        return
    axis = DIRECTIONS[direction % 6]
    spread = DIRECTIONS[(direction + 1) % 6]

    for k in range(max(1, min_steps), max_steps + 1):
        base_q = origin.q + axis.q * k
        base_r = origin.r + axis.r * k
        for w in range(-half_width, half_width + 1):
            yield Axial(base_q + spread.q * w, base_r + spread.r * w)


def cells_in_disk(origin: Axial, radius: int, min_radius: int = 0) -> Iterator[Axial]:
    """圆形区域（环形），由近及远。"""
    if radius < 0:
        return
    if min_radius <= 0:
        yield origin
        start = 1
    else:
        start = max(1, min_radius)
    for k in range(start, radius + 1):
        yield from ring(origin, k)


def cells_in_sector(
    origin: Axial,
    bearing_start: float,
    bearing_end: float,
    max_steps: int,
) -> Iterator[Axial]:
    """按**方位角**定义的扇区，由近及远逐层产出。模拟雷达的实际扫描范围。

    与 :func:`cells_in_cone` 的区别：这里的张角随距离自然展开，
    而不是固定宽度的走廊。

    **只铺扇区内的格，代价与扇区自身的格数成正比，与圆盘大小无关。**
    依据是平面被 :data:`_AXIS_ORDER` 那六条轴切成六个 60° 楔形。环 ``k`` 上
    落在某个楔形里的格是

        ``P(t) = t · 低轴 + (k − t) · 高轴``，``t = 0 … k``

    它有双重身份：既是"沿低轴走 t 步、再沿高轴走 k−t 步"，也是六边形距离
    恒为 ``t + (k − t) = k`` 的那一圈。于是"半径"直接就是"低轴步数 + 高轴
    步数 ≤ max_steps"，**不需要先取外接圆盘**。

    楔形内的方位角随 ``t`` **单调递减**（``t=0`` 压在高轴上、``t=k`` 压在低轴
    上），所以"方位角落在这段弧里"在每一环上都是一段连续区间，两端各用一次
    叉积判据算出来即可——全程**不需要逐格 ``atan2``**（30 km / 200 m 格、
    半径 87 格实测见 §5.12）。

    **★ 早先的实现是"先取外接圆盘再逐格筛方位角"，那是实现缺陷、不是负结果**：
    代价与扇区宽度无关、只与圆盘大小有关（同一配置下全盘查询 4.6 ms，
    36° 扇区反而 8.1 ms，1.8 倍）。贵的地方也不是三角函数（实测可忽略），
    而是**白白生成并筛掉了两万个扇区外的格**。

    **是生成器，不是列表**（与 :func:`cells_in_cone` / :func:`cells_in_disk`
    一致），由近及远逐层产出。要重复遍历就自己 ``list(...)``——一次用完是常态。

    弧是**半开区间** ``[bearing_start, bearing_start + span)``：方位角恰好压在
    弧端起点的格归本扇区，压在终点的格不归。相邻两楔形共用一条轴，半开正是
    让"每格恰好出现一次"成立的那条约定。

    方位角跨越 0° 时（如 350° → 10°）会自动处理环绕。两个边界相等
    （``span == 0``）**按整圈解释**，这是方位角取模的习惯：``bearing_end``
    与 ``bearing_start`` 相差 0° 与相差 360° 在模意义下是同一个数。
    ⇒ 想要"这一帧只照一条射线"的调用方**不能**把 0 传进来，得自己判。

    ``origin`` 那一格**无条件收进结果**：目标贴在脚下时方位角没有定义，
    而"脚下的东西看不见"显然更错。``max_steps <= 0`` 时结果就是 ``origin``
    一格，与 :func:`cells_in_disk` 的 ``radius=0`` 一致。

    判定用的是**格心的方位角**，所以扇区边界有"格心半张角"量级的量化误差
    （200 m 格 / 30 km 处 ≈ 0.33°）。要精确到真实位置就由模型层用实际坐标
    复判一次——索引只做粗筛。
    """
    span = (bearing_end - bearing_start) % 360.0
    if span == 0.0:
        span = 360.0
    radius = max(0, int(round(max_steps)))
    yield origin
    if radius == 0:
        return

    start = bearing_start % 360.0
    end = start + span

    # 六个楔形与扇区的交弧先算完（与环号无关），内层循环只剩整数与浮点乘加。
    # 楔形每 360° 重复一次，而扇区最宽 360°，所以每个楔形最多有两个拷贝与它
    # 相交——整圈时正是贴着首尾的那两条。这段每调一次只做 6 轮，不进内层。
    arcs: list[tuple[Axial, Axial, float, float, float, float, float, float, float, float]] = []
    for w in range(6):
        low_axis = DIRECTIONS[_AXIS_ORDER[w]]
        high_axis = DIRECTIONS[_AXIS_ORDER[(w + 1) % 6]]
        low_deg = BEARING_OF_DIRECTION[_AXIS_ORDER[w]]
        eu, su = _world_offset(low_axis)
        ev, sv = _world_offset(high_axis)
        for shift in (-360.0, 0.0, 360.0):
            lo = low_deg + shift
            a = start if start > lo else lo
            b = end if end < lo + 60.0 else lo + 60.0
            if a >= b:
                continue
            arcs.append(
                (low_axis, high_axis, eu, su, ev, sv,
                 sin(radians(a)), cos(radians(a)), sin(radians(b)), cos(radians(b)))
            )
    if not arcs:
        return

    oq = origin.q
    orr = origin.r
    for k in range(1, radius + 1):
        for low_axis, high_axis, eu, su, ev, sv, sa, ca, sb, cb in arcs:
            # t = 0 压在高轴上，半开区间把那条轴让给相邻楔形；t = k 压在低轴上。
            top = _axis_steps(k, sa, ca, eu, su, ev, sv)
            if top < 1:
                continue
            base = _axis_steps(k, sb, cb, eu, su, ev, sv) + 1
            if base < 1:
                base = 1
            if base > top:
                continue
            for t in range(base, top + 1):
                j = k - t
                yield Axial(oq + low_axis.q * t + high_axis.q * j,
                            orr + low_axis.r * t + high_axis.r * j)


def _world_offset(direction: Axial) -> tuple[float, float]:
    """网格方向 → 世界偏移 ``(东向, 南向)``。

    与 :func:`bearing_between` 用的是同一套换算（``√3·(q + r/2)`` 与 ``1.5·r``）。
    改一处必须改两处，否则扇区判据与方位角判据会各说各话——而两边都不报错。
    """
    return _SQRT3 * (direction.q + 0.5 * direction.r), 1.5 * direction.r


def _axis_steps(
    k: int,
    s: float,
    c: float,
    eu: float,
    su: float,
    ev: float,
    sv: float,
) -> int:
    """环 ``k`` 上"方位角 ≥ x"的最大步数 t（x 的正弦/余弦为 ``s``/``c``）。

    ``P(t) = t·低轴 + (k−t)·高轴``，方位角随 ``t`` **单调递减**，所以满足
    "≥ x" 的是一段前缀，取上界即可，不必逐格算 ``atan2``。

    判据用叉积而不是角度：方位角 ≥ x ⟺ ``sin(方位角 − x) ≥ 0`` ⟺
    ``sin(x)·南向 + cos(x)·东向 ≥ 0``。格心方位与边界 x 的夹角不超过 60°
    （楔形只有 60° 宽），远小于 180°，正弦的符号判据没有歧义。

    **容差是必须的**：轴上的格恰好压在边界上，``sin``/``cos`` 的舍入会让本应为
    0 的乘积变成 ∓1e−16，不兜住就会把整条轴漏掉（"某方向上突然看不见"）。
    相邻格心的最小夹角约 ``60°/k``（正弦 ≈ 0.01），比 ``1e−9·k`` 大十个量级，
    所以不会误收弧外的格。
    """
    a = k * (s * sv + c * ev)
    b = s * (su - sv) + c * (eu - ev)
    tol = -1e-9 * k
    if b >= 0.0:
        return k if a >= tol else -1
    if a < tol:
        return -1
    g = int(-a / b)
    if g > k:
        g = k
    elif g < 0:
        g = 0
    while g > 0 and a + b * g < tol:
        g -= 1
    while g < k and a + b * (g + 1) >= tol:
        g += 1
    return g


# ---------------------------------------------------------------------------
# 索引
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class IndexStats:
    entities: int = 0
    occupied_cells: int = 0
    moves: int = 0
    queries: int = 0

    @property
    def occupancy(self) -> float:
        """平均每格实体数。接近 1 说明实体很稀疏，索引收益大。"""
        return self.entities / self.occupied_cells if self.occupied_cells else 0.0


class SpatialIndex:
    """网格空间索引：``cell → [entity_id]``。

    一个实体同时只能在一个格子里。移到别的格子必须先 ``update``。
    """

    __slots__ = ("_cells", "_entity_cell", "_moves", "_queries")

    def __init__(self) -> None:
        self._cells: dict[Axial, list[int]] = {}
        self._entity_cell: dict[int, Axial] = {}
        self._moves = 0
        self._queries = 0

    # -- 维护 --------------------------------------------------------------

    def add(self, entity_id: int, cell: Axial) -> None:
        """把实体登记到格子。重复登记同一实体视为移动。"""
        existing = self._entity_cell.get(entity_id)
        if existing is not None:
            if existing == cell:
                return
            self._detach(entity_id, existing)

        bucket = self._cells.get(cell)
        if bucket is None:
            self._cells[cell] = [entity_id]
        elif not bucket or entity_id > bucket[-1]:
            # 实体 ID 单调分配，顺序插入是常态——保持 O(1)
            bucket.append(entity_id)
        else:
            insort(bucket, entity_id)

        self._entity_cell[entity_id] = cell

    def remove(self, entity_id: int) -> bool:
        cell = self._entity_cell.pop(entity_id, None)
        if cell is None:
            return False
        self._detach(entity_id, cell)
        return True

    def _detach(self, entity_id: int, cell: Axial) -> None:
        bucket = self._cells.get(cell)
        if bucket is None:
            return
        try:
            bucket.remove(entity_id)
        except ValueError:
            return
        if not bucket:
            del self._cells[cell]

    def update(self, entity_id: int, cell: Axial) -> bool:
        """更新实体位置。返回**是否发生了跨格移动**。

        返回值的用处：调用方可以只在真正跨格时才做后续处理
        （比如刷新雷达的快照），避免每帧都重算。
        """
        previous = self._entity_cell.get(entity_id)
        if previous == cell:
            return False
        self.add(entity_id, cell)
        if previous is not None:
            self._moves += 1
        return True

    # -- 查询 --------------------------------------------------------------

    def entities_at(self, cell: Axial) -> list[int]:
        """某个格子里的实体。返回的是**内部列表的副本**，调用方可以随意改。"""
        self._queries += 1
        bucket = self._cells.get(cell)
        return list(bucket) if bucket else []

    def entities_in(self, cells: Iterable[Axial]) -> list[int]:
        """一批格子里的实体，按遍历顺序拼接。

        同一实体不会重复出现（每个实体只属于一个格子）。
        格子列表本身有序时，结果顺序也是确定的。
        """
        self._queries += 1
        result: list[int] = []
        get = self._cells.get
        for cell in cells:
            bucket = get(cell)
            if bucket:
                result.extend(bucket)
        return result

    def entities_in_cone(
        self,
        origin: Axial,
        direction: int,
        max_steps: int,
        half_width: int = 0,
        min_steps: int = 1,
    ) -> list[int]:
        """锥形搜索。结果按距离由近及远——符合"先发现近处目标"的直觉。"""
        return self.entities_in(
            cells_in_cone(origin, direction, max_steps, half_width, min_steps)
        )

    def entities_in_sector(
        self,
        origin: Axial,
        bearing_start: float,
        bearing_end: float,
        max_steps: int,
    ) -> list[int]:
        """扇区搜索（按方位角），结果按距离由近及远。

        这是"**顺着天线方向找格**"的落点：旋转天线的相位每帧推进一小片，
        把这一片交给这里，拿到的就是"波束此刻照到的那批实体"。全盘查询
        给不出这个——它只按距离筛，弧外的目标会一起回来。

        **代价**见 :func:`cells_in_sector`：只铺扇区内的格，与扇区宽度成正比。
        30 km / 200 m 格（半径 87 格）实测：全盘 4.6 ms、90° 1.2 ms、
        36° 0.58 ms、10° 0.17 ms——**比 :meth:`entities_in_disk` 便宜，不是更贵**。
        
        与 :meth:`entities_in_cone` 的分工：锥形是"六边形方向上的固定宽度
        走廊"（适合"往东看"），扇区是"真实张角"（适合"波束此刻指哪"）。
        """
        return self.entities_in(
            cells_in_sector(origin, bearing_start, bearing_end, max_steps)
        )

    def entities_in_disk(
        self, origin: Axial, radius: int, min_radius: int = 0
    ) -> list[int]:
        """圆形搜索（全向）。"""
        return self.entities_in(cells_in_disk(origin, radius, min_radius))

    def entity_cell(self, entity_id: int) -> Axial | None:
        return self._entity_cell.get(entity_id)

    def has(self, entity_id: int) -> bool:
        return entity_id in self._entity_cell

    def cells(self) -> Iterator[Axial]:
        """遍历**有实体的**格子。顺序是插入序，可复现。"""
        return iter(self._cells)

    def entity_ids(self) -> list[int]:
        """所有已登记的实体 ID，升序。诊断与自检用。"""
        return sorted(self._entity_cell)

    def __iter__(self) -> Iterator[int]:
        """遍历实体 ID，升序。"""
        return iter(self.entity_ids())

    # -- 诊断 --------------------------------------------------------------

    def statistics(self) -> IndexStats:
        return IndexStats(
            entities=len(self._entity_cell),
            occupied_cells=len(self._cells),
            moves=self._moves,
            queries=self._queries,
        )

    def clear(self) -> None:
        self._cells.clear()
        self._entity_cell.clear()

    def __len__(self) -> int:
        return len(self._entity_cell)

    def __contains__(self, entity_id: int) -> bool:
        return entity_id in self._entity_cell

    def __repr__(self) -> str:
        stats = self.statistics()
        return (
            f"<SpatialIndex {stats.entities} 实体 / "
            f"{stats.occupied_cells} 格，占据率 {stats.occupancy:.2f}>"
        )


class LayeredSpatialIndex:
    """按 ``(zone_id, layer)`` 分层的索引集合。

    实体在跨洲航渡时属于全球层，进入战区后属于某个局部层。用同一套
    坐标编码索引不了两边的实体，所以按层分开存，查询时显式指定层。

    不自动跨层合并结果是刻意的：不同层的格子粒度差几个数量级，
    悄悄混在一起返回会让调用方拿到语义不明的结果。
    """

    __slots__ = ("_layers",)

    def __init__(self) -> None:
        self._layers: dict[tuple[int, int], SpatialIndex] = {}

    def layer(self, zone_id: int = 0, layer: int = 0) -> SpatialIndex:
        """取得（必要时创建）某一层的索引。"""
        key = (zone_id, layer)
        index = self._layers.get(key)
        if index is None:
            index = SpatialIndex()
            self._layers[key] = index
        return index

    def find(self, entity_id: int) -> tuple[tuple[int, int], Axial] | None:
        """在所有层里找实体，返回 (层键, 格子)。

        实体数量少时这个线性扫描可以接受；如果层数很多需要改成
        维护一张 entity → layer 的反查表。
        """
        for key, index in self._layers.items():
            cell = index.entity_cell(entity_id)
            if cell is not None:
                return key, cell
        return None

    def keys(self) -> list[tuple[int, int]]:
        """所有层的键，**按序**返回以保证可复现。"""
        return sorted(self._layers)

    def total_entities(self) -> int:
        return sum(len(index) for index in self._layers.values())

    def statistics(self) -> dict[tuple[int, int], IndexStats]:
        return {key: index.statistics() for key, index in sorted(self._layers.items())}

    def clear(self) -> None:
        self._layers.clear()

    def __len__(self) -> int:
        return len(self._layers)

    def __repr__(self) -> str:
        return (
            f"<LayeredSpatialIndex {len(self._layers)} 层 / "
            f"{self.total_entities()} 实体>"
        )


# ---------------------------------------------------------------------------
# 便捷判定
# ---------------------------------------------------------------------------

def cells_within_bearing(
    origin: Axial,
    target: Axial,
    bearing: float,
    tolerance_deg: float,
) -> bool:
    """目标是否落在以 ``bearing`` 为中心、半角 ``tolerance_deg`` 的扇区内。

    比"取最近的网格方向"更精确——用于对已经筛出的候选做最终的角度确认。
    """
    delta = abs((bearing_between(origin, target) - bearing + 180.0) % 360.0 - 180.0)
    return delta <= tolerance_deg


def approximate_distance_m(origin: Axial, target: Axial, size: float) -> float:
    """两格的近似地面距离（米）。

    六边形网格上"步数 × 格距"是下界（斜向走更远），但用于粗筛足够。
    精确距离要用心算世界坐标或测地公式。
    """
    return distance(origin, target) * size * sqrt(3.0)
