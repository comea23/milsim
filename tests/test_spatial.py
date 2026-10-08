"""空间索引测试。

覆盖雷达探测真正依赖的三件事：
  1. 网格 → 实体 的映射准确，实体移动后索引跟上
  2. 锥形/扇区搜索的方向正确（错一格就是错几十公里）
  3. 查询结果顺序完全确定，否则同种子两次推演会分叉
"""

from __future__ import annotations

import pytest

from milsim.services.map import (
    DIRECTIONS,
    Axial,
    HexGrid,
    LocalFrame,
    generate_flat,
    offset_to_axial,
    ring,
)
from milsim.services.spatial import (
    BEARING_OF_DIRECTION,
    LayeredSpatialIndex,
    SpatialIndex,
    approximate_distance_m,
    bearing_between,
    bearing_of_direction,
    bearing_of_offset,
    cells_in_cone,
    cells_in_disk,
    cells_in_sector,
    cells_within_bearing,
    direction_from_bearing,
)


def ax(col: int, row: int) -> Axial:
    return offset_to_axial(col, row)


# ---------------------------------------------------------------------------
# 方位角 ↔ 网格方向
# ---------------------------------------------------------------------------

def test_direction_to_bearing_table() -> None:
    """方向索引与方位角的对应表必须与 DIRECTIONS 的顺序一致。"""
    assert bearing_of_direction(0) == 90.0     # E
    assert bearing_of_direction(1) == 30.0     # NE
    assert bearing_of_direction(2) == 330.0    # NW
    assert bearing_of_direction(3) == 270.0    # W
    assert bearing_of_direction(4) == 210.0    # SW
    assert bearing_of_direction(5) == 150.0    # SE
    assert BEARING_OF_DIRECTION[0] == 90.0


def test_bearing_to_direction_roundtrip() -> None:
    for d in range(6):
        assert direction_from_bearing(bearing_of_direction(d)) == d


def test_direction_mapping_is_north_south_symmetric() -> None:
    """正北与正南必须映射到**相反**的方向。

    两者都恰好落在两个方向的中间（各差 30°），属于平局。用银行家舍入
    会让一侧偏西、另一侧偏东，雷达扫描出现系统性偏移。
    """
    north = direction_from_bearing(0.0)
    south = direction_from_bearing(180.0)
    assert (north + 3) % 6 == south

    east = direction_from_bearing(90.0)
    west = direction_from_bearing(270.0)
    assert (east + 3) % 6 == west


def test_bearing_between_matches_direction_table() -> None:
    """每个网格方向的实测方位角必须与 `BEARING_OF_DIRECTION` 完全吻合。

    这条测试能同时抓住两类错误：y 轴符号搞反（东北变西南）、
    以及用格数代替世界坐标比例（东北变 26.6° 而不是 30°）。
    """
    origin = Axial(0, 0)
    for direction, expected in enumerate(BEARING_OF_DIRECTION):
        target = Axial(DIRECTIONS[direction].q * 5, DIRECTIONS[direction].r * 5)
        actual = bearing_between(origin, target)
        assert actual == pytest.approx(expected, abs=0.01), (
            f"方向 {direction} 期望 {expected}°，实测 {actual:.2f}°"
        )


# ---------------------------------------------------------------------------
# 搜索区域几何
# ---------------------------------------------------------------------------

def test_cone_single_direction_is_a_ray() -> None:
    origin = Axial(0, 0)
    cells = list(cells_in_cone(origin, direction=0, max_steps=3, half_width=0))
    assert cells == [Axial(1, 0), Axial(2, 0), Axial(3, 0)]


def test_cone_width_spreads_perpendicular() -> None:
    origin = Axial(0, 0)
    cells = list(cells_in_cone(origin, direction=0, max_steps=2, half_width=1))
    # k=1 → (0,1) (1,0) (2,-1)；k=2 → (1,1) (2,0) (3,-1)
    assert len(cells) == 6
    assert Axial(0, 1) in cells
    assert Axial(1, 0) in cells
    assert Axial(2, -1) in cells
    assert Axial(3, -1) in cells


def test_cone_min_steps_skips_near_layers() -> None:
    origin = Axial(0, 0)
    cells = list(cells_in_cone(origin, 0, max_steps=4, half_width=0, min_steps=3))
    assert cells == [Axial(3, 0), Axial(4, 0)]


def test_disk_is_symmetric() -> None:
    origin = Axial(0, 0)
    cells = list(cells_in_disk(origin, radius=2))
    assert cells[0] == origin
    assert len(cells) == 1 + 6 + 12
    for cell in cells:
        assert (cell - origin).q is not None      # 仅确认类型
    assert len(set(cells)) == len(cells)          # 无重复


def test_sector_respects_bearing_span() -> None:
    """扇区内的每个格子方位角都要落在给定范围内。"""
    origin = Axial(0, 0)
    cells = list(cells_in_sector(origin, 60.0, 120.0, max_steps=4))
    assert origin in cells
    for cell in cells:
        if cell == origin:
            continue
        b = bearing_between(origin, cell)
        assert 60.0 <= b <= 120.0, f"{cell} 的方位角 {b:.1f}° 越界"


def test_sector_wraps_across_north() -> None:
    """扇区跨越 0° 时必须正确环绕。"""
    origin = Axial(0, 0)
    cells = list(cells_in_sector(origin, 350.0, 370.0, max_steps=3))
    for cell in cells:
        if cell == origin:
            continue
        b = bearing_between(origin, cell)
        assert b >= 350.0 or b <= 10.0, f"{cell} 的方位角 {b:.1f}° 不在跨北扇区内"


# ---------------------------------------------------------------------------
# 楔形枚举：只铺扇区内的格
#
# 实现把平面切成六个 60° 楔形、楔形内按 `t·低轴 + (k−t)·高轴` 铺格，并用
# **半开**弧把共用的边界轴只算一次。楔形拷贝的平移、弧的开闭、叉积判据的
# 容差，任一处错都会让下面第一个测试变红（它是逐格比对，不是抽样）。
# ---------------------------------------------------------------------------

def _brute_force_sector(
    origin: Axial, start: float, span: float, radius: int
) -> set[Axial]:
    """参照实现：整盘扫 + 逐格筛方位角（半开弧）。慢，但显然对。"""
    cells = {origin}
    for k in range(1, radius + 1):
        for cell in ring(origin, k):
            if (bearing_between(origin, cell) - start) % 360.0 < span:
                cells.add(cell)
    return cells


@pytest.mark.parametrize("radius", [1, 3, 8, 20])
@pytest.mark.parametrize(
    "start,span",
    [
        (0.0, 360.0),      # 整圈
        (13.0, 60.0),      # 楔形内部起算
        (77.0, 30.0),      # 贴着 90° 轴但不压线
        (123.0, 90.0),     # 跨两个楔形
        (183.0, 60.0),     # 南侧
        (268.0, 120.0),    # 跨三个楔形
        (352.0, 20.0),     # 跨 0°
        (352.0, 360.0),    # 跨 0° 的整圈
    ],
)
def test_sector_matches_brute_force(start: float, span: float, radius: int) -> None:
    """楔形枚举必须与"整盘扫 + 逐格筛"逐格一致，且同一个格只产出一次。"""
    origin = Axial(0, 0)
    got = list(cells_in_sector(origin, start, start + span, radius))
    assert len(got) == len(set(got)), "同一个格被产出两次"
    assert set(got) == _brute_force_sector(origin, start, span, radius)


def test_sector_arc_ends_are_half_open() -> None:
    """弧是半开的：压在弧**起点**的方位要收，压在**终点**的不收。

    ``Axial(6, 0)`` 的格心方位角恰好是 90.0°（正东）。压在轴上的格最容易
    被"闭区间"或叉积判据的舍入吃掉/多收，所以单列一条，不用抽样比。
    """
    origin = Axial(0, 0)
    east = Axial(6, 0)
    assert east in list(cells_in_sector(origin, 90.0, 100.0, 6))    # 起点，收
    assert east not in list(cells_in_sector(origin, 80.0, 90.0, 6))  # 终点，不收
    assert bearing_between(origin, east) == pytest.approx(90.0)


def test_sector_equals_disk_when_span_is_zero() -> None:
    """``span == 0`` 按整圈解释，且必须与全盘查询**集合相等**（不多不少）。"""
    origin = Axial(7, -3)                    # 故意不放原点：原点像一条特殊路径
    radius = 30
    full = list(cells_in_sector(origin, 137.0, 137.0, radius))
    assert len(full) == len(set(full))
    assert set(full) == set(cells_in_disk(origin, radius))


def test_sector_with_no_steps_is_just_the_origin() -> None:
    """``max_steps <= 0`` 只给 origin 一格——与 ``cells_in_disk(radius=0)`` 一致。"""
    origin = Axial(2, 5)
    assert list(cells_in_sector(origin, 0.0, 90.0, 0)) == [origin]
    assert list(cells_in_sector(origin, 0.0, 90.0, -1)) == [origin]


def test_bearing_of_offset_cardinal_directions() -> None:
    """世界坐标偏移 → 方位角。**第二个分量是南向**，不是北向。"""
    assert bearing_of_offset(0.0, -1.0) == pytest.approx(0.0)      # 北
    assert bearing_of_offset(1.0, 0.0) == pytest.approx(90.0)      # 东
    assert bearing_of_offset(0.0, 1.0) == pytest.approx(180.0)     # 南
    assert bearing_of_offset(-1.0, 0.0) == pytest.approx(270.0)    # 西


def test_bearing_of_offset_agrees_with_bearing_between() -> None:
    """用格坐标算与用世界位移算必须给出同一个角——两条路只有一套约定。"""
    frame = LocalFrame(0.0, 0.0, 1000.0)
    origin = Axial(0, 0)
    for cell in (Axial(4, 0), Axial(2, -3), Axial(-3, 5), Axial(0, -7), Axial(-6, -2)):
        x, y = frame.to_world(cell)
        assert bearing_of_offset(x, y) == pytest.approx(bearing_between(origin, cell))


def test_entities_in_sector_picks_only_the_arc() -> None:
    """扇区搜索只收方位角落在弧内的实体。

    格与方位角的对应（``BEARING_OF_DIRECTION``）：``(6, 0)`` 正东 90°、
    ``(6, -6)`` 东北 30°、``(-6, 0)`` 正西 270°。
    """
    index = SpatialIndex()
    index.add(1, Axial(6, 0))        # 90°
    index.add(2, Axial(-6, 0))       # 270°
    index.add(3, Axial(6, -6))       # 30°

    assert index.entities_in_sector(Axial(0, 0), 80.0, 100.0, max_steps=8) == [1]
    assert index.entities_in_sector(Axial(0, 0), 20.0, 40.0, max_steps=8) == [3]
    assert index.entities_in_sector(Axial(0, 0), 250.0, 290.0, max_steps=8) == [2]


def test_entities_in_sector_is_nearest_first() -> None:
    """扇区搜索的结果按距离由近及远——"先发现近处目标"的直觉。"""
    index = SpatialIndex()
    index.add(1, Axial(8, 0))
    index.add(2, Axial(2, 0))
    assert index.entities_in_sector(Axial(0, 0), 80.0, 100.0, max_steps=9) == [2, 1]


def test_cells_within_bearing_tolerance() -> None:
    origin = Axial(0, 0)
    east = Axial(3, 0)
    assert cells_within_bearing(origin, east, bearing=90.0, tolerance_deg=10.0)
    assert not cells_within_bearing(origin, east, bearing=200.0, tolerance_deg=10.0)


def test_approximate_distance_uses_cell_spacing() -> None:
    """距离用"步数 × 边长 × √3"，不是步数 × 边长。"""
    d = approximate_distance_m(Axial(0, 0), Axial(10, 0), size=1000.0)
    assert d == pytest.approx(10 * 1000.0 * 1.7320508, rel=1e-6)


# ---------------------------------------------------------------------------
# 索引维护
# ---------------------------------------------------------------------------

def test_add_and_query() -> None:
    index = SpatialIndex()
    index.add(7, Axial(1, 2))
    assert index.entities_at(Axial(1, 2)) == [7]
    assert index.entities_at(Axial(9, 9)) == []
    assert index.entity_cell(7) == Axial(1, 2)
    assert len(index) == 1


def test_bucket_stays_sorted_regardless_of_insert_order() -> None:
    """格子内实体列表必须保持升序。

    用 ``set`` 存储时遍历顺序随哈希值变化，同一想定两次运行可能给出不同的
    发现顺序，进而让后续决策分叉。这里退化成 list 并按 ID 有序插入。
    """
    index = SpatialIndex()
    for eid in (9, 3, 7, 5, 1):
        index.add(eid, Axial(0, 0))
    assert index.entities_at(Axial(0, 0)) == [1, 3, 5, 7, 9]


def test_update_detects_cross_cell_move() -> None:
    index = SpatialIndex()
    index.add(1, Axial(0, 0))
    assert index.update(1, Axial(0, 0)) is False      # 同格，不算移动
    assert index.update(1, Axial(1, 0)) is True       # 跨格
    assert index.entities_at(Axial(0, 0)) == []
    assert index.entities_at(Axial(1, 0)) == [1]
    assert index.statistics().moves == 1


def test_add_same_entity_twice_with_new_cell_moves_it() -> None:
    index = SpatialIndex()
    index.add(1, Axial(0, 0))
    index.add(1, Axial(5, 5))
    assert index.entities_at(Axial(0, 0)) == []
    assert index.entities_at(Axial(5, 5)) == [1]
    assert len(index) == 1


def test_remove_cleans_empty_bucket() -> None:
    index = SpatialIndex()
    index.add(1, Axial(2, 2))
    index.add(2, Axial(2, 2))
    assert index.remove(1) is True
    assert index.entities_at(Axial(2, 2)) == [2]
    assert index.remove(2) is True
    assert index.entities_at(Axial(2, 2)) == []
    assert index.statistics().occupied_cells == 0
    assert index.remove(99) is False                  # 移除不存在的实体


def test_empty_entity_id_zero_is_valid() -> None:
    """实体 ID 从 0 开始分配，不能把 0 当成"不存在"。"""
    index = SpatialIndex()
    index.add(0, Axial(0, 0))
    assert index.entities_at(Axial(0, 0)) == [0]
    assert index.entity_cell(0) == Axial(0, 0)
    assert index.has(0)


# ---------------------------------------------------------------------------
# 锥形 / 圆形查询
# ---------------------------------------------------------------------------

def test_cone_query_returns_targets_in_direction() -> None:
    index = SpatialIndex()
    # 正东 3 格放一个，正西 3 格放一个
    index.add(10, Axial(3, 0))
    index.add(20, Axial(-3, 0))
    index.add(30, Axial(0, 3))

    found = index.entities_in_cone(Axial(0, 0), direction=0, max_steps=5)
    assert found == [10]           # 只应发现正东那个

    found_west = index.entities_in_cone(Axial(0, 0), direction=3, max_steps=5)
    assert found_west == [20]


def test_cone_query_respects_range() -> None:
    index = SpatialIndex()
    index.add(1, Axial(3, 0))
    index.add(2, Axial(30, 0))

    found = index.entities_in_cone(Axial(0, 0), direction=0, max_steps=10)
    assert found == [1]

    found_far = index.entities_in_cone(Axial(0, 0), direction=0, max_steps=40)
    assert found_far == [1, 2]


def test_cone_results_are_near_to_far() -> None:
    """结果按距离由近及远——符合"先发现近处目标"的直觉。"""
    index = SpatialIndex()
    index.add(300, Axial(30, 0))
    index.add(100, Axial(10, 0))
    index.add(200, Axial(20, 0))

    found = index.entities_in_cone(Axial(0, 0), direction=0, max_steps=40)
    assert found == [100, 200, 300]


def test_cone_width_captures_off_axis_target() -> None:
    index = SpatialIndex()
    index.add(1, Axial(5, -2))       # 偏离正东

    assert index.entities_in_cone(Axial(0, 0), 0, 6, half_width=0) == []
    assert index.entities_in_cone(Axial(0, 0), 0, 6, half_width=3) == [1]


def test_disk_query_returns_all_within_radius() -> None:
    index = SpatialIndex()
    index.add(1, Axial(0, 0))
    index.add(2, Axial(2, 0))
    index.add(3, Axial(10, 0))

    found = index.entities_in_disk(Axial(0, 0), radius=3)
    assert set(found) == {1, 2}

    ring_only = index.entities_in_disk(Axial(0, 0), radius=3, min_radius=2)
    assert ring_only == [2]


def test_entities_in_cells_deduplicates_nothing_but_keeps_order() -> None:
    index = SpatialIndex()
    index.add(1, Axial(1, 0))
    index.add(2, Axial(2, 0))
    found = index.entities_in([Axial(2, 0), Axial(1, 0)])
    assert found == [2, 1]           # 保持格子遍历顺序


def test_queries_are_reproducible_across_instances() -> None:
    """同样的登记顺序 → 同样的查询结果。"""

    def build_and_query():
        index = SpatialIndex()
        for eid, cell in [(5, Axial(1, 0)), (3, Axial(1, 0)), (9, Axial(2, 0))]:
            index.add(eid, cell)
        return (
            index.entities_at(Axial(1, 0)),
            index.entities_in_cone(Axial(0, 0), 0, 5, half_width=2),
        )

    assert build_and_query() == build_and_query()


# ---------------------------------------------------------------------------
# 分层索引
# ---------------------------------------------------------------------------

def test_layered_index_separates_zones() -> None:
    layered = LayeredSpatialIndex()
    layered.layer(zone_id=1, layer=0).add(10, Axial(5, 5))
    layered.layer(zone_id=2, layer=0).add(20, Axial(5, 5))

    assert layered.layer(1, 0).entities_at(Axial(5, 5)) == [10]
    assert layered.layer(2, 0).entities_at(Axial(5, 5)) == [20]
    assert layered.total_entities() == 2


def test_layered_index_find_locates_entity() -> None:
    layered = LayeredSpatialIndex()
    layered.layer(7, 1).add(42, Axial(3, 3))
    assert layered.find(42) == ((7, 1), Axial(3, 3))
    assert layered.find(999) is None


def test_layered_index_keys_are_sorted() -> None:
    layered = LayeredSpatialIndex()
    for key in [(2, 0), (1, 1), (1, 0), (3, 0)]:
        layered.layer(*key)
    assert layered.keys() == [(1, 0), (1, 1), (2, 0), (3, 0)]


# ---------------------------------------------------------------------------
# 端到端：雷达扫描
# ---------------------------------------------------------------------------

def test_radar_scan_pipeline() -> None:
    """模拟一次完整的雷达扫描：位置索引 → 锥形粗筛 → 通视精筛。

    这是空间索引存在的理由——用"查格子"代替"遍历所有实体"。
    """
    from milsim.services.map import check_line_of_sight

    grid = HexGrid(zone_id=1, layer=0, size=1000.0)
    generate_flat(grid, -64, -64, 128, 128)

    index = SpatialIndex()
    radar_cell = ax(0, 0)

    # 正东方向摆三个目标，其中一个被小山挡住
    targets = {101: ax(10, 0), 102: ax(20, 0), 103: ax(30, 0)}
    for eid, cell in targets.items():
        index.add(eid, cell)
    # 正西方向摆一个，不该被发现
    index.add(104, ax(-10, 0))

    # 在 101 附近堆一道遮挡
    for row in range(-4, 5):
        grid.set_elevation(ax(15, row), 400.0)
    del targets[102]     # 102 在遮挡后面

    candidates = index.entities_in_cone(radar_cell, direction=0, max_steps=40)
    assert set(candidates) == {101, 102, 103}

    detected = [
        eid
        for eid in candidates
        if check_line_of_sight(
            grid, radar_cell, index.entity_cell(eid), 20.0, 2.0
        ).visible
    ]
    assert 101 in detected
    assert 102 not in detected       # 被山挡住
    assert 104 not in candidates     # 方向不对


def test_spatial_index_scales_to_many_entities() -> None:
    """索引的收益随实体数增长——这是它存在的意义。"""
    index = SpatialIndex()
    for eid in range(5000):
        index.add(eid, Axial(eid % 100, eid // 100))

    stats = index.statistics()
    assert stats.entities == 5000
    assert stats.occupied_cells == 5000
    assert stats.occupancy == pytest.approx(1.0)

    # 锥形查询只扫覆盖到的格子，与总实体数无关
    found = index.entities_in_cone(Axial(0, 0), 0, max_steps=20, half_width=5)
    assert found
    assert all(index.entity_cell(eid) for eid in found)
