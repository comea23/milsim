"""程序化地形生成。

用途
----
先用生成地形把"加载 → 存储 → 寻路 → 通视"整条链路打通，避免在真实 DEM 的
格式、坐标基准、投影对齐上卡住。之后接真实数据只需换一个 loader，
下游代码一行都不用改。

一个关键设计：噪声坐标按**米**换算
------------------------------------
同一片地理区域，用 100 m 网格和 2 km 网格分别生成地形，结果应当**一致**——
只是采样密度不同。所以噪声的输入是"格中心的世界坐标（米）÷ 特征尺度"，
而不是"格索引 ÷ 某常数"。

真实接入 DEM 时这个约束会自然消失（数据本来就是按地理坐标给的），
但在此之前，它保证了多分辨率测试不会互相矛盾。

海拔基准：海平面 = 0 m
------------------------
高程是**绝对海拔**，**海平面取 0 m**，水下为负。这样水深可以直接派生：
``water_depth = max(0, -elevation)``，于是"水深 12 m 能否过 8 m 吃水的舰"
这类判断才算得出来。

早先的实现里"水位"是噪声的一个**分位数**——它只规定了陆海比例，没有绝对
基准，水下高程只是 ±900 m 噪声里的低值。改法是：水位仍按分位数定
**陆海比例**，但生成后把整个高程场**平移**，使水位对齐 ``water_level_m``
（默认 0 m）。地形形状不变，只换基准。
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from .grid import CHUNK_SIZE, IMPASSABLE, LEVEL_SURFACE, HexGrid

# ---------------------------------------------------------------------------
# 地形类型
# ---------------------------------------------------------------------------

TERRAIN_WATER = 0
TERRAIN_PLAIN = 1
TERRAIN_FOREST = 2
TERRAIN_HILL = 3
TERRAIN_MOUNTAIN = 4
TERRAIN_URBAN = 5

TERRAIN_NAMES = {
    TERRAIN_WATER: "水域",
    TERRAIN_PLAIN: "平原",
    TERRAIN_FOREST: "林地",
    TERRAIN_HILL: "丘陵",
    TERRAIN_MOUNTAIN: "山地",
    TERRAIN_URBAN: "城镇",
}

#: 通行代价。数值本身只是相对权重，但必须与 path.py 的启发式下界一致
#: （下界取 1），否则 A* 的可采纳性会被破坏。
#:
#: **这张表是地面口径的**：水域 = 不可通行。它只适合地面部队——
#: 舰船要的是恰好互补的一张表（见 :data:`WATER_COST_TABLE`）。
#: 通行是**运载器的解释**，不是地形的属性（§3.11.3）。
MOVE_COST_TABLE = {
    TERRAIN_WATER: IMPASSABLE,
    TERRAIN_PLAIN: 1,
    TERRAIN_FOREST: 3,
    TERRAIN_HILL: 2,
    TERRAIN_MOUNTAIN: 6,
    TERRAIN_URBAN: 1,
}

#: 舰船的成本表：**水域可通行，陆地不可**——与地面表恰好互补。
#: 这张表本身就是"通行不是地形属性"最直观的证据。
WATER_COST_TABLE = {
    TERRAIN_WATER: 1,
    TERRAIN_PLAIN: IMPASSABLE,
    TERRAIN_FOREST: IMPASSABLE,
    TERRAIN_HILL: IMPASSABLE,
    TERRAIN_MOUNTAIN: IMPASSABLE,
    TERRAIN_URBAN: IMPASSABLE,
}

#: 两栖：水陆都能走，陆上略慢（上下滩、装载）且不能爬山。
AMPHIBIOUS_COST_TABLE = {
    TERRAIN_WATER: 2,
    TERRAIN_PLAIN: 2,
    TERRAIN_FOREST: 5,
    TERRAIN_HILL: 4,
    TERRAIN_MOUNTAIN: IMPASSABLE,
    TERRAIN_URBAN: 3,
}

#: 遮蔽度 0-255。森林和城镇提供最好的掩护，水域几乎为零。
COVER_TABLE = {
    TERRAIN_WATER: 0,
    TERRAIN_PLAIN: 10,
    TERRAIN_FOREST: 180,
    TERRAIN_HILL: 80,
    TERRAIN_MOUNTAIN: 120,
    TERRAIN_URBAN: 200,
}


# ---------------------------------------------------------------------------
# 噪声
# ---------------------------------------------------------------------------

#: 晶格哈希的乘子。**用整数哈希而不是 numpy 的 rng**：rng 依赖生成时的
#: 数组形状与遍历顺序，换个分块大小或起点就变了——逐块生成的地形会在
#: 块接缝处对不上。哈希只依赖世界坐标，所以谁生成、什么时候生成都一样。
_HASH_X = 73856093
_HASH_Y = 19349663
_HASH_SEED = 83492791


def _hash_lattice(ix: np.ndarray, iy: np.ndarray, seed: int) -> np.ndarray:
    """世界坐标晶格点 → [0, 1) 的确定值。同样的坐标永远同样的值。"""
    mask = np.uint64(0xFFFFFFFF)
    hx = (ix.astype(np.int64).astype(np.uint64) * np.uint64(_HASH_X)) & mask
    hy = (iy.astype(np.int64).astype(np.uint64) * np.uint64(_HASH_Y)) & mask
    hs = np.uint64((seed + 1) * _HASH_SEED & 0xFFFFFFFF)
    h = hx ^ hy ^ hs
    h ^= h >> np.uint64(13)
    h = (h * np.uint64(1274126177)) & mask
    h ^= h >> np.uint64(16)
    return (h & np.uint64(0xFFFFFF)).astype(np.float64) / 16777216.0


def _smoothstep(t: np.ndarray) -> np.ndarray:
    """平滑插值权重 ``3t² - 2t³``。

    线性插值会在晶格线上留下折痕，地形看起来像折纸——这是值噪声最常见的
    观感缺陷，而且它不影响任何断言，只是"看着不对劲"。
    """
    return t * t * (3.0 - 2.0 * t)


def value_noise(
    col0: int,
    row0: int,
    cols: int,
    rows: int,
    *,
    spacing_m: float,
    size_m: float,
    seed: int,
) -> np.ndarray:
    """单八度值噪声，值域 [0, 1]。**晶格锚在世界坐标上**。

    ``spacing_m`` 是晶格间距（米）。同样的世界位置、同样的间距，无论从哪个
    分块、以什么顺序请求，得到的值完全一样——这是逐块生成能拼起来的前提。
    """
    if spacing_m <= 0.0:
        raise ValueError(f"晶格间距必须为正，实际 {spacing_m}")

    per_cell = size_m / spacing_m
    xs = (col0 + np.arange(cols)) * per_cell
    ys = (row0 + np.arange(rows)) * per_cell

    x0 = np.floor(xs)
    y0 = np.floor(ys)
    fx = _smoothstep(xs - x0)[None, :]
    fy = _smoothstep(ys - y0)[:, None]

    ix = x0.astype(np.int64)[None, :]
    iy = y0.astype(np.int64)[:, None]

    v00 = _hash_lattice(ix, iy, seed)
    v10 = _hash_lattice(ix + 1, iy, seed)
    v01 = _hash_lattice(ix, iy + 1, seed)
    v11 = _hash_lattice(ix + 1, iy + 1, seed)

    top = v00 * (1.0 - fx) + v10 * fx
    bottom = v01 * (1.0 - fx) + v11 * fx
    return top * (1.0 - fy) + bottom * fy


def fbm(
    col0: int,
    row0: int,
    cols: int,
    rows: int,
    *,
    feature_scale_m: float = 15_000.0,
    size_m: float = 1000.0,
    seed: int = 0,
    octaves: int = 6,
    lacunarity: float = 2.0,
    gain: float = 0.5,
) -> np.ndarray:
    """分形布朗运动：多个八度的值噪声叠加，值域约 [0, 1]。

    每个八度频率翻倍、振幅减半，产生自然界常见的自相似起伏。
    **所有八度都锚在世界坐标上**，所以相邻分块拼起来是连续的（§3.11.5）。

    每个八度换一个种子：不换的话，粗间距的八度会取到同一批晶格点，
    叠加出来是"一个地形反复加深"，不是分形。
    """
    result = np.zeros((rows, cols), dtype=np.float64)
    amplitude = 1.0
    norm = 0.0
    spacing = feature_scale_m

    for index in range(max(1, octaves)):
        result += amplitude * value_noise(
            col0, row0, cols, rows,
            spacing_m=spacing, size_m=size_m, seed=seed + index * 7919,
        )
        norm += amplitude
        amplitude *= gain
        spacing /= lacunarity

    return result / norm if norm else result


#: 换算分位数用的样本格边长与范围。只要够大、够细，具体取值不影响结果。
_SAMPLE_SIZE_M = 400.0
_SAMPLE_SPAN = 256


@lru_cache(maxsize=32)
def _raw_sample(octaves: int, gain: float) -> np.ndarray:
    """一大片 fBm 样本，用来把"占比"换算成固定阈值。

    噪声是**平稳**的（统计性质不随位置变），所以这片样本的分布对任何位置
    都成立。缓存住：只在第一次用到时算一遍。
    """
    return fbm(
        -_SAMPLE_SPAN // 2, -_SAMPLE_SPAN // 2, _SAMPLE_SPAN, _SAMPLE_SPAN,
        feature_scale_m=15_000.0, size_m=_SAMPLE_SIZE_M,
        seed=20_260_101, octaves=octaves, gain=gain,
    )


@lru_cache(maxsize=32)
def _terrain_thresholds(
    water_level_frac: float, octaves: int, gain: float
) -> tuple[float, float, float]:
    """``(水域上界, 丘陵下界, 山地下界)``，都是 fBm 值域里的**固定**阈值。

    **为什么不能用分位数**：分位数是"当前这块区域"的统计量。逐块生成时
    每块各算各的，于是——一块里 34% 是水、另一块里 34% 也是水，但**阈值
    不同**，海岸线就在块接缝处跳变。而且这个现象只在接缝上看得出来，
    不报错、不影响任何单块断言。

    用固定阈值 + 一次大样本换算，既保住了"占比"这个参数的语义，又让
    每一块都用同一把尺子。
    """
    sample = _raw_sample(octaves, gain)
    water = float(np.quantile(sample, water_level_frac))
    land = sample[sample >= water]
    if land.size == 0:
        return water, water, water
    hill = float(np.quantile(land, 0.55))
    mountain = float(np.quantile(land, 0.85))
    return water, hill, mountain


# ---------------------------------------------------------------------------
# 生成
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class TerrainStats:
    """生成结果的概览。用来确认地形参数合理，而不是一片死平或全是山。"""

    cells: int = 0
    min_elevation: float = 0.0
    max_elevation: float = 0.0
    mean_elevation: float = 0.0
    water_ratio: float = 0.0
    mountain_ratio: float = 0.0
    #: 不可通行比例。**这是地面口径**：它等于"水域占比"，因为地面部队
    #: 过不了水。对舰船恰恰相反——留着这个名字是提醒读者它绑定了运载器，
    #: 不要拿它当通用的"可通行性"。
    impassable_ratio: float = 0.0
    #: 最大水深（米）。由海拔基准派生，舰船的吃水判据。
    max_water_depth: float = 0.0

    def __str__(self) -> str:
        return (
            f"{self.cells} 格，高程 {self.min_elevation:.0f}~{self.max_elevation:.0f} m "
            f"(均 {self.mean_elevation:.0f})，水域 {self.water_ratio:.1%}"
            f"（最深 {self.max_water_depth:.0f} m），山地 {self.mountain_ratio:.1%}，"
            f"不可通行 {self.impassable_ratio:.1%}（地面口径）"
        )


def generate_terrain(
    grid: HexGrid,
    col0: int,
    row0: int,
    cols: int,
    rows: int,
    *,
    seed: int = 0,
    feature_scale_m: float = 15_000.0,
    amplitude_m: float = 900.0,
    water_level_frac: float = 0.34,
    forest_frac: float = 0.28,
    octaves: int = 6,
    urban_clusters: int = 0,
    water_level_m: float = 0.0,
    level: int = LEVEL_SURFACE,
) -> TerrainStats:
    """在网格上生成一片地形。

    参数
    ----
    grid
        目标网格。属性直接写入，已有数据会被覆盖。
    col0, row0, cols, rows
        偏移坐标下的矩形范围。
    feature_scale_m
        地形特征的横向尺度。山地起伏约每 15 km 一个山脊。
    amplitude_m
        高程起伏幅度（米），相对于基准面。
    water_level_frac
        fBm 值低于此分位数的区域判为水域。它定的是**陆海比例**，
        不是水位高度——水位高度由 ``water_level_m`` 定。
    forest_frac
        中低海拔区域中判为林地的比例。
    urban_clusters
        随机撒布的城镇数量。城镇不可设在水域或高山。
    water_level_m
        水位的**绝对海拔**（米），默认 0（海平面）。生成后整个高程场会被
        平移，使水位落在这个值上，于是水下高程为负、水深可直接派生。
    level
        写入哪个**垂向层**（§3.11）。默认 0 = 地表带。空中层目前不生成
        任何数据——那里的通道还没有内容（风、空域管制），先留空比填假值好。

    返回
    ----
    :class:`TerrainStats`
    """
    if cols <= 0 or rows <= 0:
        raise ValueError("生成范围必须为正")

    shape = (rows, cols)
    size_m = grid.size

    # 所有噪声都锚在**世界坐标**上（§3.11.5）：同样的世界位置，
    # 无论从哪个分块、以什么顺序请求，得到的值都一样。所以逐块生成
    # 拼起来是连续的，而不是一块一块互不相干的地形。
    field = fbm(
        col0, row0, cols, rows,
        feature_scale_m=feature_scale_m, size_m=size_m,
        seed=seed, octaves=octaves,
    )

    # 森林用一层独立的噪声，与高程解耦——否则树会长在山脊线上连成一片
    forest_field = fbm(
        col0, row0, cols, rows,
        feature_scale_m=feature_scale_m * 0.5, size_m=size_m,
        seed=seed + 104_729, octaves=4,
    )

    # 阈值是**固定的**，不是本块的分位数——用分位数会让海岸线在块接缝处
    # 跳变（见 _terrain_thresholds 的说明）。
    water_cut, hill_cut, mountain_cut = _terrain_thresholds(
        water_level_frac, octaves, 0.5
    )

    # **把水位对齐到绝对基准**（§3.11.4）：平移之后 0 m 就是海平面，
    # 水下为负，`water_depth = -elevation` 才有意义。
    # 系数 2.0 保持原有幅度语义：偏离水位半个值域 ≈ amplitude_m 米。
    elevation = (field - water_cut) * (2.0 * amplitude_m) + water_level_m

    terrain = np.full(shape, TERRAIN_PLAIN, dtype=np.uint8)
    terrain[field < water_cut] = TERRAIN_WATER

    land = terrain != TERRAIN_WATER
    high = land & (field >= mountain_cut)
    mid = land & ~high & (field >= hill_cut)

    terrain[mid] = TERRAIN_HILL
    terrain[high] = TERRAIN_MOUNTAIN

    # 森林只长在中低海拔：林线以上不该有树
    plantable = land & ~high & (forest_field > 1.0 - forest_frac)
    terrain[plantable] = TERRAIN_FOREST

    if urban_clusters > 0:
        # 城镇是**跨格的特征**，逐块生成会把一个镇子切碎在块边界上。
        # 所以它只在"一次生成一整片"时可用（想定层也还没暴露这个参数）。
        _place_urban_clusters(np.random.default_rng(seed), terrain, elevation,
                              urban_clusters)

    _blit(grid, col0, row0, terrain, elevation, level=level)

    water_mask = terrain == TERRAIN_WATER
    mountain_mask = terrain == TERRAIN_MOUNTAIN
    total = terrain.size

    # 水深是派生量：只有水下部分取正值。用 -elevation 在非水域上会得到
    # 陆地的负海拔，所以必须按 water_mask 掩掉。
    depths = np.where(water_mask, np.maximum(0.0, -elevation), 0.0)

    return TerrainStats(
        cells=int(total),
        min_elevation=float(elevation.min()),
        max_elevation=float(elevation.max()),
        mean_elevation=float(elevation.mean()),
        water_ratio=float(water_mask.sum() / total),
        mountain_ratio=float(mountain_mask.sum() / total),
        impassable_ratio=float(water_mask.sum() / total),
        max_water_depth=float(depths.max()) if water_mask.any() else 0.0,
    )


def _place_urban_clusters(
    rng: np.random.Generator,
    terrain: np.ndarray,
    elevation: np.ndarray,
    count: int,
    radius: int = 2,
) -> None:
    """在适宜位置撒布城镇。

    城镇建在水域和高山上没有意义，所以先筛出候选点再撒。
    以候选点为中心的一小片区域整体转为城镇。
    """
    rows, cols = terrain.shape
    usable = (terrain != TERRAIN_WATER) & (terrain != TERRAIN_MOUNTAIN)

    candidates = np.argwhere(usable)
    if len(candidates) == 0:
        return

    picked = rng.choice(len(candidates), size=min(count, len(candidates)), replace=False)
    for idx in picked:
        cy, cx = candidates[idx]
        y0, y1 = max(0, cy - radius), min(rows, cy + radius + 1)
        x0, x1 = max(0, cx - radius), min(cols, cx + radius + 1)
        patch = terrain[y0:y1, x0:x1]
        # 城镇不吞掉水域——河流穿城而过是真实的，但把水面填成城区就很怪
        patch[patch != TERRAIN_WATER] = TERRAIN_URBAN


def _blit(
    grid: HexGrid,
    col0: int,
    row0: int,
    terrain: np.ndarray,
    elevation: np.ndarray,
    *,
    level: int = LEVEL_SURFACE,
) -> None:
    """把属性数组按分块写入网格的某一垂向层。"""
    rows, cols = terrain.shape
    cost = np.zeros_like(terrain, dtype=np.uint16)
    cover = np.zeros_like(terrain, dtype=np.uint8)
    for t, c in MOVE_COST_TABLE.items():
        cost[terrain == t] = c
    for t, c in COVER_TABLE.items():
        cover[terrain == t] = c

    cx0 = col0 // CHUNK_SIZE
    cx1 = (col0 + cols - 1) // CHUNK_SIZE
    cy0 = row0 // CHUNK_SIZE
    cy1 = (row0 + rows - 1) // CHUNK_SIZE

    for cy in range(cy0, cy1 + 1):
        for cx in range(cx0, cx1 + 1):
            chunk_col0 = cx * CHUNK_SIZE
            chunk_row0 = cy * CHUNK_SIZE

            # 数组坐标 = 网格偏移坐标 - 起始偏移
            ax0 = chunk_col0 - col0
            ay0 = chunk_row0 - row0
            ax1 = ax0 + CHUNK_SIZE
            ay1 = ay0 + CHUNK_SIZE

            # 与本块的交集（边界块可能只覆盖一部分）
            sx0, sx1 = max(0, ax0), min(cols, ax1)
            sy0, sy1 = max(0, ay0), min(rows, ay1)
            if sx0 >= sx1 or sy0 >= sy1:
                continue

            # 块内起始索引
            dx0 = sx0 - ax0
            dy0 = sy0 - ay0
            dw = sx1 - sx0
            dh = sy1 - sy0

            chunk = grid.ensure_chunk(cx, cy, level)
            chunk.terrain[dy0:dy0 + dh, dx0:dx0 + dw] = terrain[sy0:sy1, sx0:sx1]
            chunk.elevation[dy0:dy0 + dh, dx0:dx0 + dw] = elevation[sy0:sy1, sx0:sx1]
            chunk.move_cost[dy0:dy0 + dh, dx0:dx0 + dw] = cost[sy0:sy1, sx0:sx1]
            chunk.cover[dy0:dy0 + dh, dx0:dx0 + dw] = cover[sy0:sy1, sx0:sx1]


def generate_flat(
    grid: HexGrid,
    col0: int,
    row0: int,
    cols: int,
    rows: int,
    *,
    level: int = LEVEL_SURFACE,
) -> TerrainStats:
    """生成全平地形。

    仅用于验证算法逻辑的边界情况（比如"平地通视必须全部可见"）
    **不要**用来做正式测试——平地上测不出地球曲率、也测不出地形遮蔽，
    这类缺陷会一路潜伏到接入真实地形才暴露。
    """
    terrain = np.full((rows, cols), TERRAIN_PLAIN, dtype=np.uint8)
    elevation = np.zeros((rows, cols), dtype=np.float32)
    _blit(grid, col0, row0, terrain, elevation, level=level)

    return TerrainStats(
        cells=rows * cols,
        min_elevation=0.0,
        max_elevation=0.0,
        mean_elevation=0.0,
        water_ratio=0.0,
        mountain_ratio=0.0,
        impassable_ratio=0.0,
        max_water_depth=0.0,
    )
