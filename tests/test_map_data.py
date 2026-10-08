"""网格数据层测试：分块存储、地形生成、寻路、通视。

重点覆盖三件容易出错的事：
  1. 分块定位在**负坐标**下的行为（Python 的 // 和 % 语义与 C 不同）
  2. 地球曲率——平坦地形上超视距目标必须被判为不可见
  3. 分层寻路与直接搜索的一致性
"""

from __future__ import annotations

import math

import pytest

from milsim.services.map import (
    CHUNK_SIZE,
    IMPASSABLE,
    TERRAIN_FOREST,
    TERRAIN_MOUNTAIN,
    Axial,
    CostConfig,
    HexGrid,
    check_line_of_sight,
    distance,
    find_path,
    generate_flat,
    generate_terrain,
    horizon_drop_m,
    neighbors,
    offset_to_axial,
    radio_horizon_m,
    reachable_cells,
)
from milsim.services.map.path import DIRECT_SEARCH_MAX_CELLS, _astar


SIZE = 1000.0

#: pointy-top 布局下，**相邻格中心的距离是边长 × √3**，不是边长。
#:
#: 这个数字极其容易记错，而且错了不报错——只是所有通视距离、曲率下沉量、
#: 坡度全都偏差 √3 倍。本文件的早期版本就在这里栽过：把 20 格当成 20 km，
#: 实际是 34.6 km，于是"20 km 处不该被曲率遮挡"的断言无端失败。
CELL_GROUND_M = SIZE * math.sqrt(3.0)


def ax(col: int, row: int) -> Axial:
    """偏移坐标 → 轴向坐标。测试里用偏移坐标表达位置更直观。"""
    return offset_to_axial(col, row)


def cells_spanning(meters: float) -> int:
    """跨过指定地面距离需要多少格。"""
    return max(1, int(round(meters / CELL_GROUND_M)))


def flat_grid(size: float = SIZE, extent: int = 64) -> HexGrid:
    """一块以原点为中心的全平网格。"""
    grid = HexGrid(zone_id=1, layer=0, size=size)
    half = extent // 2
    generate_flat(grid, -half, -half, extent, extent)
    return grid


# ---------------------------------------------------------------------------
# 分块存储
# ---------------------------------------------------------------------------

def test_chunk_key_handles_negative_coordinates() -> None:
    """负坐标必须向下取整分块——用 C 的截断取整会把 -1 映射到块 0，
    让块内偏移变成 -1，直接索引越界。"""
    assert HexGrid.chunk_key(0, 0) == (0, 0)
    assert HexGrid.chunk_key(31, 31) == (0, 0)
    assert HexGrid.chunk_key(32, 0) == (1, 0)
    assert HexGrid.chunk_key(-1, -1) == (-1, -1)
    assert HexGrid.chunk_key(-32, -32) == (-1, -1)
    assert HexGrid.chunk_key(-33, -33) == (-2, -2)


def test_in_chunk_handles_negative_coordinates() -> None:
    assert HexGrid.in_chunk(0, 0) == (0, 0)
    assert HexGrid.in_chunk(31, 31) == (31, 31)
    assert HexGrid.in_chunk(32, 32) == (0, 0)
    assert HexGrid.in_chunk(-1, -1) == (CHUNK_SIZE - 1, CHUNK_SIZE - 1)
    assert HexGrid.in_chunk(-32, -32) == (0, 0)


def test_unloaded_chunk_returns_safe_defaults() -> None:
    """未加载的块必须返回"可通行"的默认值。

    返回不可通行会让战区边缘的实体莫名其妙卡住；返回 0 高程则让
    通视判定退化。默认值的选择是有意的。
    """
    grid = HexGrid()
    cell = ax(0, 0)
    assert grid.elevation(cell) == 0.0
    assert grid.move_cost(cell) == 1
    assert grid.cover(cell) == 0
    assert grid.is_passable(cell)


def test_set_and_read_back() -> None:
    grid = HexGrid()
    cell = ax(5, -7)
    grid.set_cell(cell, elevation=123.5, terrain=TERRAIN_FOREST, cover=180, move_cost=3)
    assert grid.elevation(cell) == pytest.approx(123.5)
    assert grid.terrain(cell) == TERRAIN_FOREST
    assert grid.cover(cell) == 180
    assert grid.move_cost(cell) == 3


def test_partial_update_does_not_clear_other_channels() -> None:
    grid = HexGrid()
    cell = ax(1, 1)
    grid.set_cell(cell, elevation=50.0, terrain=TERRAIN_FOREST)
    grid.set_cell(cell, elevation=60.0)
    assert grid.terrain(cell) == TERRAIN_FOREST
    assert grid.elevation(cell) == pytest.approx(60.0)


def test_writes_land_in_the_right_chunk() -> None:
    """跨块边界的写入不能串块。"""
    grid = HexGrid()
    for col in (31, 32, 33, -1, -32, -33):
        grid.set_elevation(ax(col, 0), float(col))
    for col in (31, 32, 33, -1, -32, -33):
        assert grid.elevation(ax(col, 0)) == pytest.approx(float(col))


def test_miss_rate_tracking() -> None:
    grid = HexGrid()
    grid.elevation(ax(0, 0))
    grid.elevation(ax(500, 500))
    assert grid.read_count == 2
    assert grid.miss_count == 2
    assert grid.miss_rate == pytest.approx(1.0)

    grid.set_elevation(ax(0, 0), 1.0)
    grid.elevation(ax(0, 0))
    assert grid.miss_rate == pytest.approx(2 / 3)


def test_region_reads_move_cost_block() -> None:
    grid = HexGrid()
    for col in range(3):
        grid.set_move_cost(ax(col, 0), 5)
    block = grid.region(0, 0, 3, 3)
    assert block.shape == (3, 3)
    assert block[0, 0] == 5
    assert block[0, 2] == 5


def test_region_rejects_multi_chunk_span() -> None:
    """跨块的区域查询返回默认值——这是刻意限制，不是 bug。

    拼接跨块区域要处理非对齐边界，会让 numpy 零拷贝切片退化成拷贝。
    粗粒度寻路会把区域对齐到块内，所以这个限制不会造成麻烦。
    """
    grid = HexGrid()
    grid.set_move_cost(ax(40, 0), 5)
    block = grid.region(0, 0, 64, 4)          # 跨越 0 号和 1 号块
    assert (block == 1).all()


def test_statistics() -> None:
    grid = flat_grid(extent=64)
    stats = grid.statistics()
    assert stats["chunks"] == 4                # 64×64 需要 2×2 个 32 格块
    assert stats["cells"] == 4 * CHUNK_SIZE * CHUNK_SIZE
    assert stats["memory_mb"] > 0


# ---------------------------------------------------------------------------
# 地形生成
# ---------------------------------------------------------------------------

def test_generation_is_deterministic() -> None:
    a, b = HexGrid(), HexGrid()
    sa = generate_terrain(a, -32, -32, 64, 64, seed=42)
    sb = generate_terrain(b, -32, -32, 64, 64, seed=42)
    assert sa == sb
    for cell in (ax(0, 0), ax(10, -5), ax(-20, 15), ax(30, 30)):
        assert a.elevation(cell) == b.elevation(cell)
        assert a.move_cost(cell) == b.move_cost(cell)


def test_different_seed_gives_different_terrain() -> None:
    a, b = HexGrid(), HexGrid()
    generate_terrain(a, -32, -32, 64, 64, seed=1)
    generate_terrain(b, -32, -32, 64, 64, seed=2)
    assert any(
        a.elevation(ax(c, 0)) != b.elevation(ax(c, 0)) for c in range(0, 64, 7)
    )


def test_generated_terrain_is_not_degenerate() -> None:
    """生成的地形要有起伏、有水域，不能是一片死平或全是水。"""
    grid = HexGrid()
    stats = generate_terrain(grid, -64, -64, 128, 128, seed=3)
    assert stats.max_elevation - stats.min_elevation > 200
    assert 0.0 < stats.water_ratio < 0.9
    assert stats.impassable_ratio == pytest.approx(stats.water_ratio)


def test_move_cost_and_cover_are_written() -> None:
    grid = HexGrid()
    generate_terrain(grid, -32, -32, 64, 64, seed=5)
    costs = {grid.move_cost(ax(c, 0)) for c in range(-30, 30, 3)}
    assert costs - {IMPASSABLE}         # 至少有可通行的格子
    covers = {grid.cover(ax(c, 0)) for c in range(-30, 30, 3)}
    assert len(covers) > 1              # 遮蔽度不是单一值


def test_forest_grows_below_treeline() -> None:
    """森林不该长在山顶——林线以上没有树。"""
    grid = HexGrid()
    generate_terrain(grid, -64, -64, 128, 128, seed=11)

    forest_heights = []
    mountain_heights = []
    for col in range(-60, 60, 2):
        for row in range(-60, 60, 2):
            cell = ax(col, row)
            t = grid.terrain(cell)
            if t == TERRAIN_FOREST:
                forest_heights.append(grid.elevation(cell))
            elif t == TERRAIN_MOUNTAIN:
                mountain_heights.append(grid.elevation(cell))

    assert forest_heights and mountain_heights
    assert max(forest_heights) < min(mountain_heights)


def test_flat_generation() -> None:
    grid = HexGrid()
    stats = generate_flat(grid, -16, -16, 32, 32)
    assert stats.max_elevation == 0.0
    assert grid.elevation(ax(0, 0)) == 0.0
    assert grid.is_passable(ax(0, 0))


def test_generation_respects_grid_size_for_consistency() -> None:
    """特征尺度按米算：网格边长不同时，同一地点的地形应当接近。

    这条保证了多分辨率测试不会互相矛盾——2 km 网格和 1 km 网格
    在同一个地理位置上不该长出完全不同的山。
    """
    coarse = HexGrid(size=2000.0)
    fine = HexGrid(size=1000.0)
    # 同样覆盖约 128 km
    generate_terrain(coarse, -32, -32, 64, 64, seed=99, feature_scale_m=30_000)
    generate_terrain(fine, -64, -64, 128, 128, seed=99, feature_scale_m=30_000)

    # 粗网格的 (0,0) 对应细网格的 (0,0)，高程量级应当接近
    for cell in (ax(0, 0), ax(5, 3), ax(-8, 4)):
        c = coarse.elevation(cell)
        f = fine.elevation(cell)
        assert abs(c - f) < 600, f"粗 {c:.0f} vs 细 {f:.0f} 差得太多"


# ---------------------------------------------------------------------------
# 寻路
# ---------------------------------------------------------------------------

def test_straight_path_on_flat_ground() -> None:
    grid = flat_grid()
    result = find_path(grid, ax(0, 0), ax(20, 0))
    assert result is not None
    assert result.cells[0] == ax(0, 0)
    assert result.cells[-1] == ax(20, 0)
    assert len(result.cells) == 21
    assert not result.used_hierarchy          # 20 格，走直接搜索
    assert not result.degraded


def test_same_start_and_goal() -> None:
    grid = flat_grid()
    result = find_path(grid, ax(3, 3), ax(3, 3))
    assert result is not None
    assert result.cells == [ax(3, 3)]
    assert result.cost == 0.0


def test_path_avoids_impassable_wall() -> None:
    grid = flat_grid(extent=64)
    for row in range(-32, 32):
        grid.set_move_cost(ax(10, row), IMPASSABLE)

    result = find_path(grid, ax(0, 0), ax(20, 0))
    assert result is not None
    assert all(grid.is_passable(c) for c in result.cells)
    assert len(result.cells) > 21             # 必须绕路


def test_unreachable_returns_none() -> None:
    grid = flat_grid()
    start = ax(0, 0)
    for nb in neighbors(start):
        grid.set_move_cost(nb, IMPASSABLE)
    assert find_path(grid, start, ax(20, 0)) is None


def test_impassable_goal_returns_none() -> None:
    grid = flat_grid()
    grid.set_move_cost(ax(10, 0), IMPASSABLE)
    assert find_path(grid, ax(0, 0), ax(10, 0)) is None


def test_path_prefers_cheaper_terrain() -> None:
    """两条等长路线时，应选走通行代价更低的那条。"""
    grid = flat_grid(extent=64)
    for row in range(0, 32):
        for col in range(0, 21):
            grid.set_move_cost(ax(col, row), 8)      # 上半区贵

    result = find_path(grid, ax(10, 0), ax(10, 20))
    assert result is not None
    # 应当绕到下半区（row < 0）而不是直穿贵区
    assert any(c.r < 0 for c in result.cells)


def test_slope_penalty_affects_route() -> None:
    """陡坡应当被避开。"""
    grid = flat_grid(extent=64)
    # 在正中间竖一道陡坡（但不是墙）
    for row in range(-32, 32):
        grid.set_elevation(ax(10, row), 400.0)

    result = find_path(grid, ax(5, 0), ax(15, 0))
    assert result is not None
    # 上坡下坡各一次，代价必然高于纯平地
    flat_cost = 10.0
    assert result.cost > flat_cost


def test_hierarchical_search_for_long_path() -> None:
    grid = flat_grid(extent=512)
    start, goal = ax(-200, 0), ax(200, 0)
    assert abs(goal.q - start.q) > DIRECT_SEARCH_MAX_CELLS

    result = find_path(grid, start, goal)
    assert result is not None
    assert result.used_hierarchy
    assert result.cells[0] == start
    assert result.cells[-1] == goal


def test_hierarchical_matches_direct_length() -> None:
    """无障碍时，分层与直接搜索的路径长度应当接近。

    平滑之前分层路径会比直接搜索长几格——粗格中心强制路径穿过每个中心点，
    产生一串锯齿。``smooth_path`` 就是为消掉这个而存在的。
    """
    grid = flat_grid(extent=512)
    start, goal = ax(-100, 0), ax(100, 0)

    hierarchical = find_path(grid, start, goal)
    direct = _astar(grid, start, goal, CostConfig(), None)

    assert hierarchical is not None and direct is not None
    assert hierarchical.used_hierarchy
    assert abs(len(hierarchical.cells) - len(direct.cells)) <= 2


def test_path_cells_are_always_adjacent() -> None:
    """路径必须逐格相邻——相邻两点之间的距离恰好是 1。

    这是路径最基本的合法性不变量。平滑算法若写成"跳到最远可达点"，
    会得到一条看起来像路径、实际相邻点相距几十格的**跳跃列表**：
    路径长度从 200 变成 5，一切依赖路径长度的计算全部静默出错。

    直接搜索和分层搜索都要守住这条。
    """
    grid = flat_grid(extent=512)

    for start, goal in [(ax(0, 0), ax(50, 0)), (ax(-200, 0), ax(200, 0))]:
        result = find_path(grid, start, goal)
        assert result is not None
        for a, b in zip(result.cells, result.cells[1:]):
            assert distance(a, b) == 1, (
                f"路径中断：{a} → {b} 相距 {distance(a, b)} 格"
            )


def test_corridor_covers_coarse_chain() -> None:
    """走廊必须完整覆盖粗路径经过的每个粗格。"""
    from milsim.services.map import build_corridor

    chain = [(0, 0), (1, 0), (2, 0), (3, 0)]
    corridor = build_corridor(chain, factor=8)

    for ccol, crow in chain:
        center = offset_to_axial(ccol * 8 + 4, crow * 8 + 4)
        assert center in corridor

    # 每个粗格贡献 (8 + 2×8)² = 576 个细格，去重后总量应当是这个量级
    assert len(corridor) > 4 * 8 * 8


def test_corridor_adjacent_coarse_cells_overlap() -> None:
    """相邻粗格的走廊区域必须重叠，否则细层搜索会被切断。"""
    from milsim.services.map import build_corridor

    corridor = build_corridor([(0, 0)], factor=8)
    neighbor_corridor = build_corridor([(1, 0)], factor=8)
    assert corridor & neighbor_corridor, "相邻粗格的走廊没有交集"


def test_hierarchical_handles_cluttered_terrain() -> None:
    """密集障碍下分层仍须给出合法路径。"""
    import random

    grid = flat_grid(extent=256)
    rng = random.Random(11)
    for _ in range(3000):
        col = rng.randint(-110, 110)
        row = rng.randint(-110, 110)
        grid.set_move_cost(ax(col, row), IMPASSABLE)

    start, goal = ax(-100, 0), ax(100, 0)
    result = find_path(grid, start, goal)
    if result is not None:            # 可能被障碍完全堵死，那也是合法结果
        assert result.cells[0] == start
        assert result.cells[-1] == goal
        assert all(grid.is_passable(c) for c in result.cells)


def test_hierarchical_falls_back_when_corridor_blocked() -> None:
    """粗层的乐观估计可能给出走不通的走廊——必须能回退。"""
    grid = flat_grid(extent=512)
    # 造一个粗格级别的迷宫：交替留缝，让粗层难以正确判断
    for row in range(-260, 260):
        if (row // 8) % 2 == 0:
            for col in range(-260, 260, 16):
                grid.set_move_cost(ax(col, row), IMPASSABLE)

    result = find_path(grid, ax(-200, 0), ax(200, 0))
    # 无论走通与否，都不能抛异常；走通则路径必须合法
    if result is not None:
        assert all(grid.is_passable(c) for c in result.cells)


def test_reachable_cells_respects_budget() -> None:
    grid = flat_grid()
    reach = reachable_cells(grid, ax(0, 0), budget=5.0)
    assert ax(0, 0) in reach
    assert ax(5, 0) in reach
    assert ax(6, 0) not in reach
    assert all(cost <= 5.0 for cost in reach.values())


def test_reachable_cells_stops_at_water() -> None:
    """水墙要挡住扩展——所以墙必须长到超出预算可绕行的范围。

    战区外是未加载区域，默认**可通行**（否则边缘实体会莫名卡死）。
    墙只画到网格边界的话，扩展会从外面绕过去，测试就成了假阳性。
    """
    grid = flat_grid(extent=64)
    for row in range(-200, 201):
        grid.set_move_cost(ax(3, row), IMPASSABLE)

    reach = reachable_cells(grid, ax(0, 0), budget=100.0)
    assert ax(2, 0) in reach
    assert ax(3, 0) not in reach
    assert ax(4, 0) not in reach         # 绕行距离远超预算


# ---------------------------------------------------------------------------
# 通视
# ---------------------------------------------------------------------------

def test_los_on_flat_ground_short_range() -> None:
    """10 km 内、天线够高时应当通视。"""
    grid = flat_grid()
    n = cells_spanning(10_000.0)
    result = check_line_of_sight(grid, ax(0, 0), ax(n, 0), 30.0, 30.0)
    assert result.visible
    assert result.min_clearance > 0


def test_los_blocked_by_ridge() -> None:
    grid = flat_grid(size=1000.0)
    for row in range(-5, 6):
        grid.set_elevation(ax(5, row), 500.0)

    result = check_line_of_sight(grid, ax(0, 0), ax(10, 0), 2.0, 2.0)
    assert not result.visible
    assert result.blocking_cell is not None
    assert result.min_clearance < 0
    assert result.required_raise_m > 500


def test_los_reports_required_raise() -> None:
    """报告"观察者要抬多高才能看见"，这是可以被上层用来决策的。"""
    grid = flat_grid(size=1000.0)
    for row in range(-5, 6):
        grid.set_elevation(ax(5, row), 200.0)

    result = check_line_of_sight(grid, ax(0, 0), ax(10, 0), 2.0, 2.0)
    assert not result.visible
    # 视线中点需抬到 200 m，观察者抬高量约为 (200-2)/0.5 ≈ 396 m
    assert 350 < result.required_raise_m < 450


def test_earth_curvature_hides_distant_flat_ground() -> None:
    """★ 核心：平坦地形上，超视距目标必须被判为不可见。

    10 m 高的观察者对 10 m 高的目标，4/3 地球模型下地平线约 26 km。
    50 km 处必然被曲率遮蔽。不考虑曲率的实现会错误地判为"可见"——
    这正是漏算曲率最典型的后果。
    """
    grid = flat_grid(extent=256)
    n = cells_spanning(50_000.0)          # 50 km

    with_curvature = check_line_of_sight(
        grid, ax(0, 0), ax(n, 0), 10.0, 10.0, earth_curvature=True
    )
    assert not with_curvature.visible, "50 km 外 10 m 高的目标不可能在地平线上"

    without = check_line_of_sight(
        grid, ax(0, 0), ax(n, 0), 10.0, 10.0, earth_curvature=False
    )
    assert without.visible, "关掉曲率应当变为可见——这正是漏算时的错误结论"


def test_earth_curvature_does_not_hide_nearby_targets() -> None:
    """曲率不能把近距离目标也判死——过度修正和漏算一样糟。"""
    grid = flat_grid()
    for km in (1.0, 3.0, 5.0, 10.0):
        n = cells_spanning(km * 1000.0)
        result = check_line_of_sight(grid, ax(0, 0), ax(n, 0), 10.0, 10.0)
        assert result.visible, f"{km} km 处不该被曲率遮挡"


def test_clearance_matches_curvature_formula() -> None:
    """实测余隙必须与独立的曲率公式吻合。

    这是交叉验证：los.py 里的曲率修正若写错（比如分母少个 2、
    或者用了真实半径而非 4/3），这个测试会当场抓到。
    """
    grid = flat_grid(extent=256)
    n = cells_spanning(100_000.0)          # 100 km
    distance = n * CELL_GROUND_M

    result = check_line_of_sight(grid, ax(0, 0), ax(n, 0), 10.0, 10.0)

    # 两端视线高度都是 10 m，中点处下沉最大
    expected = 10.0 - horizon_drop_m(distance / 2.0)
    assert abs(result.min_clearance - expected) < 1.0


def test_horizon_drop_magnitude() -> None:
    """曲率下沉的量级必须对——这是最容易写错的地方。

    两个数不能搞混：
      * 真实地球半径下 100 km 下沉约 **785 m**（光学器材用这个）
      * 4/3 等效地球模型下约 **589 m**（雷达/通信，大气折射让电波弯曲，
        等效于地球变平）
    """
    assert 760 < horizon_drop_m(100_000, refraction_factor=1.0) < 810
    assert 570 < horizon_drop_m(100_000, refraction_factor=4 / 3) < 610
    assert horizon_drop_m(0.0) == 0.0
    assert horizon_drop_m(50_000, 1.0) < horizon_drop_m(100_000, 1.0)


def test_radio_horizon_value() -> None:
    """10 m 天线对 10 m 目标，4/3 地球模型下地平线约 26 km。"""
    d = radio_horizon_m(10.0, 10.0)
    assert 25_000 < d < 27_000

    # 天线越高看得越远
    assert radio_horizon_m(30.0, 10.0) > radio_horizon_m(10.0, 10.0)


def test_los_required_clearance_is_enforced() -> None:
    """要求净空时，贴地通过的视线应被判为不可用。"""
    grid = flat_grid(size=1000.0)
    for row in range(-5, 6):
        grid.set_elevation(ax(5, row), 10.0)     # 刚好挡在视线（10 m）附近

    loose = check_line_of_sight(
        grid, ax(0, 0), ax(10, 0), 10.0, 10.0, required_clearance=0.0
    )
    strict = check_line_of_sight(
        grid, ax(0, 0), ax(10, 0), 10.0, 10.0, required_clearance=50.0
    )
    assert loose.min_clearance > strict.min_clearance


def test_los_downsamples_long_paths() -> None:
    """超长路径必须降采样，且降采样后曲率修正仍按真实距离算。"""
    grid = flat_grid(size=1000.0, extent=512)
    result = check_line_of_sight(
        grid, ax(0, 0), ax(400, 0), 10.0, 10.0, max_samples=100
    )
    assert result.samples <= 105
    # 400 km 必然在地平线以下
    assert not result.visible


def test_los_same_cell() -> None:
    grid = flat_grid()
    result = check_line_of_sight(grid, ax(0, 0), ax(0, 0), 2.0, 2.0)
    assert result.visible
    assert result.distance_m == 0.0


def test_los_against_generated_terrain() -> None:
    """在真实起伏地形上跑一遍，确保不抛异常且结果自洽。"""
    grid = HexGrid(size=500.0)
    generate_terrain(grid, -64, -64, 128, 128, seed=17)

    visible_count = 0
    for col in range(-40, 40, 8):
        result = check_line_of_sight(
            grid, ax(0, 0), ax(col, 0), 2.0, 2.0, earth_curvature=True
        )
        if result.visible:
            visible_count += 1
        if not result.visible:
            assert result.blocking_cell is not None
            assert result.min_clearance < 0

    # 起伏地形上不该全部可见，也不该全部不可见
    assert 0 < visible_count < 11
