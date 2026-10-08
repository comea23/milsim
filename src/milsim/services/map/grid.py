"""网格数据层：分块存储 + 属性通道 + 垂向分层（§3.11）。

设计要点
--------
**分块（chunk）**：每块 32×32 个单元，按需加载。全球或大战区级别的网格不可能
整块常驻内存，分块让"只加载用得到的区域"成为默认行为。

**结构数组（SoA）**：每个属性是一条独立的 numpy 数组，而不是"每个单元一个
结构体"。寻路时只扫 ``move_cost`` 一条通道，缓存友好；加新属性时加一列即可，
不会让单元结构膨胀。

**为什么用 odd-r 偏移坐标分块**：六边形的轴向坐标没法直接铺成二维数组，
而偏移坐标可以。分块键就是 ``(col // 32, row // 32)``。

**负数取整**：Python 的 ``//`` 向下取整，``%`` 返回非负值，所以 ``col = -1``
会落到块 ``-1``、块内偏移 ``31``。这个行为正是我们要的（C 语言的截断取整
会把 -1 映射到块 0 偏移 -1，直接崩）。但别换成 C 语义的实现。

垂向分层（level）
-----------------
块键是 **三元组** ``(cx, cy, cz)``，``cz`` 就是垂向层号。三维化之所以几乎
不花代价，是因为分块**本来就是惰性分配**的（``ensure_chunk``）——没写过的
层一个字节都不占。实测：一个 60 km 半径、200 m 边长的战区，按包围盒全 dense
是 4.3 MB/层，20 层就是 86 MB；而实际只有被写过的块会占内存。

分层规则（§3.11.1）：

- **只有空中分层**。0 是地表带（含海面、水下、建筑），往上按高度带切。
- **水下不分层**：潜艇的深度是连续的 ``z``，网格只需要提供"水深"来判触底
  与航路。给水深再做垂直分层没有意义——水深是 10~100 m 量级，而网格水平
  边长是 200 m 量级，分层只会得到一堆空层。

``level`` 与 ``layer`` 是**两个不同的东西**，别混：

| 字段 | 含义 | 谁在用 |
|---|---|---|
| ``HexGrid.layer`` | **分辨率层**（LOD），边长逐层翻倍 | `ZoneSpec.layer_count` |
| ``HexGrid`` 块键的 ``cz`` | **垂向层**，按高度带切 | 本模块的 ``level`` 参数 |

海拔基准
--------
``elevation`` 是**绝对海拔**，**海平面 = 0 m**，水下取负。这样水深可以直接
派生：``water_depth = max(0, -elevation)``。早先的水位是噪声的分位数
（没有绝对基准），于是"水深 12 m 能否过 8 m 吃水的舰"这类判断根本算不出来。
"""

from __future__ import annotations

import difflib
from dataclasses import dataclass
from typing import Any

import numpy as np

from ...errors import ConfigurationError
from .hex import Axial, axial_to_offset, offset_to_axial

#: 分块边长（单元数）。32×32 = 1024 个单元/块。
#: 太小则块数量爆炸、字典查找变多；太大则按需加载失去意义。
CHUNK_SIZE = 32

#: 不可通行的 move_cost 取值。
IMPASSABLE = 65535


# ---------------------------------------------------------------------------
# 通道清单
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class ChannelSpec:
    """一条通道的元数据。

    "网格是个数据库，里面可以写各种信息"——方向对，但通道必须有**清单**：
    名称、类型、单位、默认值、谁写、谁读。少了清单，它就退化成一个无类型
    字典，把本项目最值钱的属性（拼错当场报错）弄丢。
    """

    name: str
    dtype: str
    default: float
    unit: str
    writer: str
    reader: str


#: **通道清单**。加通道只需往这张表加一行——数组会按表自动建出来。
#:
#: 注意 ``move_cost`` 是**地面口径**：水域被标成不可通行。舰船不能用它，
#: 而应该用 ``move_cost`` 之外的事实（``terrain`` / ``elevation`` / 派生出的
#: 水深）自己算成本——见 ``path.CostConfig.costs``。同一格存多套成本会让
#: 两套数字各自演化，迟早不同步。
CHANNELS: dict[str, ChannelSpec] = {
    "elevation": ChannelSpec(
        "elevation", "float32", 0.0, "m（海平面 = 0，水下为负）",
        "地形生成 / 想定导入", "通视、寻路、雷达地平线",
    ),
    "terrain": ChannelSpec(
        "terrain", "uint8", 0.0, "枚举（见 terrain.TERRAIN_*）",
        "地形生成", "通行派生、遮蔽、UI",
    ),
    "cover": ChannelSpec(
        "cover", "uint8", 0.0, "0~255", "地形生成", "遮蔽、命中判定",
    ),
    "move_cost": ChannelSpec(
        "move_cost", "uint16", 1.0, "相对权重（无量纲，65535 = 不可通行）",
        "地形生成（**地面口径**）", "A*（地面）",
    ),
}

# 未加载块的默认值。战区激活时会加载真实数据，这些默认值只用于
# 战区边界外的零星查询——但必须是"可通行"，否则边缘的实体会被
# 莫名其妙地卡住。默认高程 0 与"海平面 = 0 m"的基准一致。
DEFAULT_ELEVATION = CHANNELS["elevation"].default
DEFAULT_TERRAIN = int(CHANNELS["terrain"].default)
DEFAULT_COVER = int(CHANNELS["cover"].default)
DEFAULT_MOVE_COST = int(CHANNELS["move_cost"].default)


# ---------------------------------------------------------------------------
# 垂向层
# ---------------------------------------------------------------------------

#: 地表带。地面、海面、水下、建筑都算这一层。
LEVEL_SURFACE = 0

#: 空中各层的高度带 ``(下界, 上界)``，单位米。层号 = 下标 + 1。
VERTICAL_LEVELS: tuple[tuple[float, float], ...] = (
    (1000.0, 3000.0),     # level 1：低空
    (3000.0, 8000.0),     # level 2：中空
    (8000.0, 20000.0),    # level 3：高空
)

#: 层号总数（含地表带）。
VERTICAL_LEVEL_COUNT = 1 + len(VERTICAL_LEVELS)


def level_of_altitude(height_agl: float) -> int:
    """**离地高度**（AGL，米）→ 层号。

    注意是"离地"不是"海拔"。实体的 ``z`` 是海拔（海平面 = 0），所以调用方
    要传 ``z - grid.elevation(cell)``——否则**高原上的坦克会被算成低空**：
    它在 1000 m 海拔的高原上，AGL 却只有 2 m。这个错误不报错，只会让地面
    单位被索引进空中层。

    分层是**索引分桶**用的，不是物理边界：实体跨过 1000 m 不会"换层"而丢失
    位置，只是被索引到另一个桶里。所以分桶算错最多是查询变慢，不会让实体
    消失——这条性质让分层可以放心地按整数切。
    """
    for index, (low, _high) in enumerate(VERTICAL_LEVELS, start=1):
        if height_agl < low:
            return index - 1
    return VERTICAL_LEVEL_COUNT - 1


def level_band(level: int) -> tuple[float, float] | None:
    """层号 → 高度带。地表带返回 ``None``（它含水下，没有单一区间）。"""
    if level <= LEVEL_SURFACE:
        return None
    if level <= len(VERTICAL_LEVELS):
        return VERTICAL_LEVELS[level - 1]
    return None


# ---------------------------------------------------------------------------
# 分块
# ---------------------------------------------------------------------------

class Chunk:
    """一个 32×32 的属性块——**属于某一个垂向层**。

    通道数组由 :data:`CHANNELS` 生成，不写死在本类里。四个内置通道暴露成
    属性（``chunk.elevation`` 等），既保住既有写法，又给"按名字取通道"
    留了正路。
    """

    __slots__ = ("cx", "cy", "cz", "_arrays")

    def __init__(self, cx: int, cy: int, cz: int = 0) -> None:
        self.cx = cx
        self.cy = cy
        self.cz = cz
        self._arrays: dict[str, np.ndarray] = {
            name: np.full(
                (CHUNK_SIZE, CHUNK_SIZE), spec.default, dtype=np.dtype(spec.dtype)
            )
            for name, spec in CHANNELS.items()
        }

    # -- 通道访问 ----------------------------------------------------------

    def array(self, name: str) -> np.ndarray:
        """按名取通道数组（批量读写用）。

        **名字不认识就抛，不返回 ``None``**：写了个不存在的通道名如果不报错，
        数据就静默丢了，而读的一方拿到默认值还以为一切正常。
        """
        try:
            return self._arrays[name]
        except KeyError:
            near = difflib.get_close_matches(name, CHANNELS, n=1)
            hint = f"，是否想写 {near[0]!r}？" if near else ""
            raise ConfigurationError(
                f"未知的网格通道 {name!r}{hint}"
                f"（可用：{' / '.join(sorted(CHANNELS))}）"
            ) from None

    @property
    def elevation(self) -> np.ndarray:
        return self._arrays["elevation"]

    @property
    def terrain(self) -> np.ndarray:
        return self._arrays["terrain"]

    @property
    def cover(self) -> np.ndarray:
        return self._arrays["cover"]

    @property
    def move_cost(self) -> np.ndarray:
        return self._arrays["move_cost"]

    # -- 诊断 --------------------------------------------------------------

    @property
    def key(self) -> tuple[int, int, int]:
        return self.cx, self.cy, self.cz

    def memory_bytes(self) -> int:
        return sum(arr.nbytes for arr in self._arrays.values())

    def __repr__(self) -> str:
        return f"<Chunk ({self.cx},{self.cy},L{self.cz})>"


class HexGrid:
    """一张战区网格：分块存储 + 属性访问 + 垂向分层。

    不直接持有坐标系——坐标换算由 ``LocalFrame`` 负责。这里只认偏移坐标，
    因为分块、切片、numpy 索引全都在偏移坐标下最自然。

    所有通道访问都接受 ``level`` 关键字参数，默认 ``0``（地表带），
    所以既有的"二维"调用一字不改仍然正确。
    """

    __slots__ = ("zone_id", "layer", "size", "_chunks", "read_count", "miss_count")

    def __init__(self, zone_id: int = 0, layer: int = 0, size: float = 1000.0) -> None:
        self.zone_id = zone_id
        #: **分辨率层**（LOD），不是垂向层。垂向层是块键的第三维。
        self.layer = layer
        self.size = size
        self._chunks: dict[tuple[int, int, int], Chunk] = {}
        #: 诊断计数：命中/未命中未加载块的比例，用来发现"忘了加载数据"
        self.read_count = 0
        self.miss_count = 0

    # -- 分块定位 ----------------------------------------------------------

    @staticmethod
    def chunk_key(col: int, row: int) -> tuple[int, int]:
        return col // CHUNK_SIZE, row // CHUNK_SIZE

    @staticmethod
    def in_chunk(col: int, row: int) -> tuple[int, int]:
        return col % CHUNK_SIZE, row % CHUNK_SIZE

    def chunk(self, cx: int, cy: int, cz: int = LEVEL_SURFACE) -> Chunk | None:
        return self._chunks.get((cx, cy, cz))

    def ensure_chunk(self, cx: int, cy: int, cz: int = LEVEL_SURFACE) -> Chunk:
        """取得分块，不存在则创建。写入路径用。"""
        key = (cx, cy, cz)
        chunk = self._chunks.get(key)
        if chunk is None:
            chunk = Chunk(cx, cy, cz)
            self._chunks[key] = chunk
        return chunk

    def ensure_chunk_for(self, cell: Axial, level: int = LEVEL_SURFACE) -> Chunk:
        col, row = axial_to_offset(cell)
        return self.ensure_chunk(*self.chunk_key(col, row), level)

    # -- 单点读取 ----------------------------------------------------------

    def _lookup(self, cell: Axial, level: int = LEVEL_SURFACE) -> tuple[Chunk | None, int, int]:
        col, row = axial_to_offset(cell)
        cx, cy = self.chunk_key(col, row)
        chunk = self._chunks.get((cx, cy, level))
        self.read_count += 1
        if chunk is None:
            self.miss_count += 1
        ix, iy = self.in_chunk(col, row)
        return chunk, ix, iy

    def elevation(self, cell: Axial, level: int = LEVEL_SURFACE) -> float:
        """海拔（米），海平面 = 0，水下为负。"""
        chunk, ix, iy = self._lookup(cell, level)
        return DEFAULT_ELEVATION if chunk is None else float(chunk.elevation[iy, ix])

    def terrain(self, cell: Axial, level: int = LEVEL_SURFACE) -> int:
        chunk, ix, iy = self._lookup(cell, level)
        return DEFAULT_TERRAIN if chunk is None else int(chunk.terrain[iy, ix])

    def cover(self, cell: Axial, level: int = LEVEL_SURFACE) -> int:
        chunk, ix, iy = self._lookup(cell, level)
        return DEFAULT_COVER if chunk is None else int(chunk.cover[iy, ix])

    def move_cost(self, cell: Axial, level: int = LEVEL_SURFACE) -> int:
        """通行代价（**地面口径**）。舰船的通行要自己按地形算成本，见
        ``path.CostConfig.costs``。"""
        chunk, ix, iy = self._lookup(cell, level)
        return DEFAULT_MOVE_COST if chunk is None else int(chunk.move_cost[iy, ix])

    def is_passable(self, cell: Axial, level: int = LEVEL_SURFACE) -> bool:
        return self.move_cost(cell, level) < IMPASSABLE

    def water_depth(self, cell: Axial, level: int = LEVEL_SURFACE) -> float:
        """水深（米），**派生量**，陆地恒为 0。

        派生而不是存成通道：它只依赖 ``elevation`` 与海平面基准，多存一份
        就多一处可能不同步的地方。将来若要挖泥、潮汐、河口，再固化成通道。
        """
        return max(0.0, -self.elevation(cell, level))

    # -- 单点写入 ----------------------------------------------------------

    def set_elevation(self, cell: Axial, value: float, level: int = LEVEL_SURFACE) -> None:
        chunk = self.ensure_chunk_for(cell, level)
        ix, iy = self.in_chunk(*axial_to_offset(cell))
        chunk.elevation[iy, ix] = value

    def set_terrain(self, cell: Axial, value: int, level: int = LEVEL_SURFACE) -> None:
        chunk = self.ensure_chunk_for(cell, level)
        ix, iy = self.in_chunk(*axial_to_offset(cell))
        chunk.terrain[iy, ix] = value

    def set_cover(self, cell: Axial, value: int, level: int = LEVEL_SURFACE) -> None:
        chunk = self.ensure_chunk_for(cell, level)
        ix, iy = self.in_chunk(*axial_to_offset(cell))
        chunk.cover[iy, ix] = value

    def set_move_cost(self, cell: Axial, value: int, level: int = LEVEL_SURFACE) -> None:
        chunk = self.ensure_chunk_for(cell, level)
        ix, iy = self.in_chunk(*axial_to_offset(cell))
        chunk.move_cost[iy, ix] = value

    def set_channel(
        self, cell: Axial, name: str, value: float, level: int = LEVEL_SURFACE
    ) -> None:
        """按名写一个通道。通道名的拼写检查与 :meth:`Chunk.array` 一致。"""
        chunk = self.ensure_chunk_for(cell, level)
        ix, iy = self.in_chunk(*axial_to_offset(cell))
        chunk.array(name)[iy, ix] = value

    def set_cell(
        self,
        cell: Axial,
        *,
        level: int = LEVEL_SURFACE,
        elevation: float | None = None,
        terrain: int | None = None,
        cover: int | None = None,
        move_cost: int | None = None,
    ) -> None:
        chunk = self.ensure_chunk_for(cell, level)
        ix, iy = self.in_chunk(*axial_to_offset(cell))
        if elevation is not None:
            chunk.elevation[iy, ix] = elevation
        if terrain is not None:
            chunk.terrain[iy, ix] = terrain
        if cover is not None:
            chunk.cover[iy, ix] = cover
        if move_cost is not None:
            chunk.move_cost[iy, ix] = move_cost

    # -- 批量访问（数据并行的入口）-----------------------------------------

    def region(
        self,
        col0: int,
        row0: int,
        cols: int,
        rows: int,
        *,
        level: int = LEVEL_SURFACE,
    ) -> np.ndarray:
        """读取一个矩形区域的 ``move_cost``。

        **只用于完全落在单个分块内的区域** —— 跨块时返回默认值的填充结果。
        这是刻意限制：跨块拼接要处理非对齐边界，而且会让 numpy 的零拷贝
        切片退化成拷贝。粗粒度聚合寻路（path.py）会把区域对齐到块内，
        所以这个限制不会造成麻烦。

        返回形状 ``(rows, cols)``，越界或不存在的部分为 ``DEFAULT_MOVE_COST``。
        """
        if cols <= 0 or rows <= 0:
            raise ValueError("区域尺寸必须为正")

        out = np.full((rows, cols), DEFAULT_MOVE_COST, dtype=np.uint16)

        cx, cy = self.chunk_key(col0, row0)
        # 检查整个区域是否落在同一块内
        if self.chunk_key(col0 + cols - 1, row0 + rows - 1) != (cx, cy):
            return out
        if self.chunk_key(col0, row0 + rows - 1) != (cx, cy):
            return out
        if self.chunk_key(col0 + cols - 1, row0) != (cx, cy):
            return out

        chunk = self._chunks.get((cx, cy, level))
        if chunk is None:
            return out

        ix0, iy0 = self.in_chunk(col0, row0)
        out[:, :] = chunk.move_cost[iy0:iy0 + rows, ix0:ix0 + cols]
        return out

    def min_cost_in_chunk(self, cx: int, cy: int, cz: int = LEVEL_SURFACE) -> int:
        """分块内的最小通行代价。粗粒度寻路用它作为粗格的乐观估计。

        "乐观"是刻意的：只要块内有一个能过的格子，粗层就认为这个粗格可过。
        宁可让粗层找到一条实际走不通的路径（细化阶段会失败并回退），
        也不要因为取平均而把可行路径判死。
        """
        chunk = self._chunks.get((cx, cy, cz))
        if chunk is None:
            return DEFAULT_MOVE_COST
        return int(chunk.move_cost.min())

    # -- 诊断 --------------------------------------------------------------

    @property
    def chunk_count(self) -> int:
        return len(self._chunks)

    @property
    def cell_count(self) -> int:
        """已分配块覆盖的**体素**数（块数 × 1024），不是格数。"""
        return len(self._chunks) * CHUNK_SIZE * CHUNK_SIZE

    @property
    def levels(self) -> list[int]:
        """已分配块占用了哪些垂向层，升序。"""
        return sorted({cz for _cx, _cy, cz in self._chunks})

    @property
    def memory_bytes(self) -> int:
        return sum(c.memory_bytes() for c in self._chunks.values())

    @property
    def miss_rate(self) -> float:
        """未命中未加载块的比例。活跃推演中这个值应该接近 0；
        偏高说明战区边界划小了，或者数据加载有遗漏。"""
        return self.miss_count / self.read_count if self.read_count else 0.0

    def statistics(self) -> dict[str, Any]:
        return {
            "zone_id": self.zone_id,
            "layer": self.layer,
            "chunks": self.chunk_count,
            "cells": self.cell_count,
            "levels": self.levels,
            "memory_mb": self.memory_bytes / 1024 / 1024,
            "reads": self.read_count,
            "miss_rate": self.miss_rate,
        }

    def clear(self) -> None:
        self._chunks.clear()

    def __contains__(self, cell: Axial) -> bool:
        col, row = axial_to_offset(cell)
        cx, cy = self.chunk_key(col, row)
        return (cx, cy, LEVEL_SURFACE) in self._chunks

    def __repr__(self) -> str:
        return (
            f"<HexGrid zone={self.zone_id} layer={self.layer} "
            f"chunks={self.chunk_count} levels={self.levels}>"
        )


def cells_of_chunk(cx: int, cy: int) -> list[Axial]:
    """列举一个分块覆盖的**候选**单元（偏移坐标网格点）。

    注意这些是偏移坐标的矩形点，转换成轴向坐标后**不是**一个六边形团簇，
    而是边界呈锯齿状的区域。计算"哪些实体落在这一块"时要用这个结果逐个
    判定，不能假设它是个规整的六边形。
    """
    col0, row0 = cx * CHUNK_SIZE, cy * CHUNK_SIZE
    return [
        offset_to_axial(col, row)
        for row in range(row0, row0 + CHUNK_SIZE)
        for col in range(col0, col0 + CHUNK_SIZE)
    ]


__all__ = [
    "CHANNELS",
    "CHUNK_SIZE",
    "DEFAULT_COVER",
    "DEFAULT_ELEVATION",
    "DEFAULT_MOVE_COST",
    "DEFAULT_TERRAIN",
    "IMPASSABLE",
    "LEVEL_SURFACE",
    "VERTICAL_LEVELS",
    "VERTICAL_LEVEL_COUNT",
    "ChannelSpec",
    "Chunk",
    "HexGrid",
    "cells_of_chunk",
    "level_band",
    "level_of_altitude",
]
