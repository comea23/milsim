"""网格三维化与通道体系的测试（M2c，§3.11）。

三块重点，每块都有一个"不报错的错误"要钉住：

- **垂向分层与通道**：`layer`（分辨率）与 `level`（垂向）不能混；写错通道名
  必须当场报错，不能静默丢数据。
- **海平面基准**：水位要对齐 0 m，否则水深算不出来（旧实现里水位是噪声的
  分位数，没有绝对基准）。
- **按运载器派生通行**：水域对地面不可通行、对舰船可通行。早先寻路把
  `is_passable()`（地面口径）用在了起点检查上，于是**舰船永远找不到路**
  且不报错。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.services import (
    Axial,
    EntityStore,
    LocalCellRef,
)
from milsim.simulation import Simulation
from milsim.services.map import (
    CHUNK_SIZE,
    CHANNELS,
    IMPASSABLE,
    LEVEL_SURFACE,
    VERTICAL_LEVELS,
    CostConfig,
    CoarseView,
    HexGrid,
    MapService,
    ZoneSpec,
    generate_terrain,
    level_band,
    level_of_altitude,
)
from milsim.services.map.grid import Chunk
from milsim.services.map.path import (
    AMPHIBIOUS_TRAVEL,
    GROUND_TRAVEL,
    WATER_TRAVEL,
    find_path,
    step_cost,
)
from milsim.services.map.terrain import (
    TERRAIN_HILL,
    TERRAIN_MOUNTAIN,
    TERRAIN_PLAIN,
    TERRAIN_WATER,
    MOVE_COST_TABLE,
)
from milsim.services.scenario import ScenarioError, load_scenario

H = 3_600_000_000


def cell(q: int, r: int) -> Axial:
    return Axial(q, r)


def make_grid(size: float = 1000.0) -> HexGrid:
    return HexGrid(zone_id=1, layer=0, size=size)


def paint(grid: HexGrid, target, terrain: int, *, level: int = LEVEL_SURFACE) -> None:
    """按地形类型上色，通行代价跟着走地面表。"""
    for c in target:
        grid.set_cell(
            c, level=level, terrain=terrain, move_cost=MOVE_COST_TABLE[terrain]
        )


# ---------------------------------------------------------------------------
# 垂向分层
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    ("height", "expected"),
    [
        (-100.0, 0),      # 水下
        (0.0, 0),         # 地表
        (999.9, 0),       # 刚好在地表带上界之下
        (1000.0, 1),      # 低空下界
        (2999.0, 1),
        (3000.0, 2),      # 中空
        (7999.0, 2),
        (8000.0, 3),      # 高空
        (99_000.0, 3),
    ],
)
def test_altitude_bands(height: float, expected: int) -> None:
    assert level_of_altitude(height) == expected


def test_level_band_describes_the_band() -> None:
    assert level_band(LEVEL_SURFACE) is None      # 地表带含水下，没有单一区间
    assert level_band(1) == VERTICAL_LEVELS[0]
    assert level_band(3) == VERTICAL_LEVELS[2]
    assert level_band(99) is None


def test_vertical_levels_are_chunk_keys() -> None:
    """垂向层是块键的第三维：不同层是不同的块。"""
    grid = make_grid()
    grid.set_elevation(cell(0, 0), 10.0, level=0)
    grid.set_elevation(cell(0, 0), 1500.0, level=1)

    assert grid.elevation(cell(0, 0), level=0) == pytest.approx(10.0)
    assert grid.elevation(cell(0, 0), level=1) == pytest.approx(1500.0)
    assert grid.levels == [0, 1]
    assert grid.chunk_count == 2


def test_writing_one_level_does_not_touch_another() -> None:
    """写空中层不该让地表层凭空多出块——反过来说，地表被踩过的格，
    在空中层仍是"未生成"（默认值），这正是稀疏分配要的效果。"""
    grid = make_grid()
    grid.set_cell(cell(0, 0), terrain=TERRAIN_WATER, move_cost=IMPASSABLE, level=2)

    assert grid.terrain(cell(0, 0), level=0) == 0          # 默认值
    assert grid.move_cost(cell(0, 0), level=0) == 1        # 未生成 = 可通行
    assert grid.terrain(cell(0, 0), level=2) == TERRAIN_WATER


def test_unallocated_levels_cost_nothing() -> None:
    """稀疏分配：没写过的层一个字节都不占（这是"默认不生成"的真正理由）。"""
    grid = make_grid()
    grid.set_cell(cell(0, 0), elevation=5.0)
    one = grid.memory_bytes

    grid.set_cell(cell(0, 0), elevation=5000.0, level=1)
    assert grid.memory_bytes == 2 * one


def test_default_level_keeps_two_dimensional_calls_working() -> None:
    """所有通道访问都默认 level=0，所以既有的二维写法一字不改仍然正确。"""
    grid = make_grid()
    grid.set_cell(cell(3, 4), elevation=123.0, terrain=TERRAIN_PLAIN)
    assert grid.elevation(cell(3, 4)) == pytest.approx(123.0)
    assert grid.terrain(cell(3, 4)) == TERRAIN_PLAIN
    assert grid.is_passable(cell(3, 4))


# ---------------------------------------------------------------------------
# 通道清单
# ---------------------------------------------------------------------------

def test_channel_manifest_is_complete() -> None:
    """每个通道都要声明七件事，缺一项就等于没人对它负责。"""
    assert set(CHANNELS) == {"elevation", "terrain", "cover", "move_cost"}
    for name, spec in CHANNELS.items():
        assert spec.name == name
        assert spec.dtype
        assert spec.unit
        assert spec.writer and spec.reader


def test_chunk_arrays_come_from_the_manifest() -> None:
    chunk = Chunk(0, 0)
    assert set(chunk._arrays) == set(CHANNELS)
    for name, spec in CHANNELS.items():
        assert chunk.array(name).dtype == spec.dtype
        assert float(chunk.array(name)[0, 0]) == pytest.approx(spec.default)


def test_named_channel_access_matches_properties() -> None:
    chunk = Chunk(0, 0)
    assert chunk.array("elevation") is chunk.elevation
    assert chunk.array("move_cost") is chunk.move_cost


def test_unknown_channel_reports_the_typo() -> None:
    """写错通道名必须当场报错。静默丢一个通道的数据，读的一边拿到默认值
    还以为一切正常——这是最难查的一类。"""
    chunk = Chunk(0, 0)
    with pytest.raises(ConfigurationError, match="是否想写 'elevation'"):
        chunk.array("elevetion")
    with pytest.raises(ConfigurationError, match="未知的网格通道"):
        chunk.array("完全不存在")


def test_set_channel_by_name() -> None:
    grid = make_grid()
    grid.set_channel(cell(1, 1), "cover", 200)
    assert grid.cover(cell(1, 1)) == 200


# ---------------------------------------------------------------------------
# 海平面基准与水深
# ---------------------------------------------------------------------------

def test_water_level_is_an_absolute_datum() -> None:
    """生成后水位对齐 0 m：水域高程 ≤ 0，陆地 > 0。

    旧实现里水位是噪声的分位数，水下高程只是 ±900 m 噪声里的低值——
    于是"水深 12 m 能否过 8 m 吃水的舰"根本算不出来。
    """
    grid = make_grid(size=200.0)
    stats = generate_terrain(grid, 0, 0, 96, 96, seed=7)

    water, land = [], []
    for col in range(96):
        for row in range(96):
            from milsim.services.map.hex import offset_to_axial

            c = offset_to_axial(col, row)
            if grid.terrain(c) == TERRAIN_WATER:
                water.append(grid.elevation(c))
            else:
                land.append(grid.elevation(c))

    assert water and land
    assert max(water) <= 0.0
    assert min(land) > 0.0
    assert stats.max_water_depth > 0.0
    assert stats.max_water_depth == pytest.approx(-min(water), rel=0.01)


def test_water_depth_is_derived_from_elevation() -> None:
    grid = make_grid()
    seabed = cell(0, 0)
    grid.set_cell(seabed, terrain=TERRAIN_WATER, elevation=-12.0)
    assert grid.water_depth(seabed) == pytest.approx(12.0)

    hill = cell(1, 0)
    grid.set_cell(hill, terrain=TERRAIN_PLAIN, elevation=300.0)
    assert grid.water_depth(hill) == 0.0


def test_water_level_can_be_shifted_for_inland_lakes() -> None:
    """高原湖：水位不在 0 m，而在指定的海拔上。"""
    grid = make_grid(size=200.0)
    generate_terrain(grid, 0, 0, 64, 64, seed=3, water_level_m=1500.0)

    from milsim.services.map.hex import offset_to_axial

    water = [
        grid.elevation(offset_to_axial(col, row))
        for col in range(64)
        for row in range(64)
        if grid.terrain(offset_to_axial(col, row)) == TERRAIN_WATER
    ]
    assert water
    assert max(water) <= 1500.0
    assert min(water) > 1000.0     # 明显高于海平面


def test_land_water_ratio_still_follows_the_quantile() -> None:
    """平移只换基准，不改形状——陆海比例仍由 water_level_frac 控制。"""
    grid = make_grid(size=200.0)
    stats = generate_terrain(grid, 0, 0, 96, 96, seed=11, water_level_frac=0.34)
    assert 0.2 < stats.water_ratio < 0.5


def test_unloaded_cells_are_at_sea_level_and_passable() -> None:
    """未生成 ≠ 不可通行，也不该是莫名其妙的高程。默认 0 恰好是海平面——
    与绝对基准一致。"""
    grid = make_grid()
    far = cell(900, -400)
    assert grid.elevation(far) == 0.0
    assert grid.is_passable(far)
    assert grid.water_depth(far) == 0.0


# ---------------------------------------------------------------------------
# 按运载器派生通行
# ---------------------------------------------------------------------------

def test_ground_and_ship_tables_are_complementary() -> None:
    """同一片地形，坦克与舰船看到的是**互补**的两张表。这张表就是
    "通行是运载器的解释、不是地形的属性"最直观的证据。"""
    grid = make_grid()
    sea = cell(0, 0)
    shore = cell(1, 0)
    paint(grid, [sea], TERRAIN_WATER)
    paint(grid, [shore], TERRAIN_PLAIN)

    assert GROUND_TRAVEL.cost_at(grid, sea) >= IMPASSABLE
    assert GROUND_TRAVEL.cost_at(grid, shore) < IMPASSABLE

    assert WATER_TRAVEL.cost_at(grid, sea) < IMPASSABLE
    assert WATER_TRAVEL.cost_at(grid, shore) >= IMPASSABLE


def test_ship_profile_ignores_slope() -> None:
    """船不吃坡度。给它叠坡度惩罚会让航线被海底起伏带偏。"""
    grid = make_grid()
    a, b = cell(0, 0), cell(1, 0)
    paint(grid, [a, b], TERRAIN_WATER)
    grid.set_elevation(a, -50.0)
    grid.set_elevation(b, -150.0)

    assert step_cost(grid, a, b, WATER_TRAVEL) == pytest.approx(1.0)
    assert step_cost(grid, a, b, GROUND_TRAVEL) > 1.0


def test_ship_pathfinder_works_where_ground_one_fails() -> None:
    """这条是当时那个 bug 的回归：`find_path` 早先用 `grid.is_passable()`
    （地面口径）检查起点，于是**舰船一出发就返回"走不通"**，不报错。"""
    grid = make_grid(size=1000.0)
    sea = [cell(q, 0) for q in range(8)]
    paint(grid, sea, TERRAIN_WATER)

    start, goal = cell(0, 0), cell(7, 0)
    water_path = find_path(grid, start, goal, config=WATER_TRAVEL)
    assert water_path is not None
    assert water_path.cells[0] == start and water_path.cells[-1] == goal

    # 地面部队在同一条路上确实走不通——它连起点都站不住
    assert find_path(grid, start, goal, config=GROUND_TRAVEL) is None


def test_coarse_layer_uses_the_mover_profile() -> None:
    """粗层若按地面口径采样，舰船的粗路径会以为陆地可通行，
    细层再逐段失败、整体降级——表现是"能用但从没分层成功过"。"""
    grid = make_grid(size=1000.0)
    paint(grid, [cell(q, r) for q in range(16) for r in range(16)], TERRAIN_WATER)

    sea_view = CoarseView(grid, WATER_TRAVEL)
    ground_view = CoarseView(grid, GROUND_TRAVEL)
    assert sea_view.cost((0, 0), (0, 0)) < IMPASSABLE
    assert ground_view.cost((0, 0), (0, 0)) >= IMPASSABLE


def test_amphibious_profile_crosses_both() -> None:
    grid = make_grid()
    sea, shore, mountain = cell(0, 0), cell(1, 0), cell(2, 0)
    from milsim.services.map.terrain import TERRAIN_MOUNTAIN

    paint(grid, [sea], TERRAIN_WATER)
    paint(grid, [shore], TERRAIN_PLAIN)
    paint(grid, [mountain], TERRAIN_MOUNTAIN)

    assert AMPHIBIOUS_TRAVEL.cost_at(grid, sea) < IMPASSABLE
    assert AMPHIBIOUS_TRAVEL.cost_at(grid, shore) < IMPASSABLE
    assert AMPHIBIOUS_TRAVEL.cost_at(grid, mountain) >= IMPASSABLE


def test_heuristic_bound_must_not_overestimate() -> None:
    """`min_cost` 大于成本表最小值会让启发式高估——A* 不再保证最优，
    而且**不会有任何报错**。所以构造时就拦。"""
    with pytest.raises(ConfigurationError, match="可采纳性"):
        CostConfig(min_cost=5.0, costs=WATER_TRAVEL.costs)
    with pytest.raises(ConfigurationError, match="必须为正"):
        CostConfig(min_cost=0.0)


def test_default_profile_reads_the_ground_channel() -> None:
    """``costs=None`` 等价于地面口径——既有代码的行为一字不变。"""
    grid = make_grid()
    cell_a = cell(0, 0)
    paint(grid, [cell_a], TERRAIN_WATER)

    default = CostConfig()
    assert default.cost_at(grid, cell_a) == pytest.approx(
        float(grid.move_cost(cell_a))
    )
    assert default.cost_at(grid, cell_a) == pytest.approx(
        GROUND_TRAVEL.cost_at(grid, cell_a)
    )


def test_fallback_profile_keeps_the_mover_identity() -> None:
    """回退搜索换的是启发式权重，不是运载器——换画像的代价是错的。"""
    faster = WATER_TRAVEL.with_heuristic_weight(1.3)
    assert faster.costs is WATER_TRAVEL.costs
    assert faster.slope_weight == WATER_TRAVEL.slope_weight
    assert faster.level == WATER_TRAVEL.level
    assert faster.heuristic_weight == pytest.approx(1.3)


# ---------------------------------------------------------------------------
# 底盘门槛：轮式与履带的可通行能力不同（§5.7）
# ---------------------------------------------------------------------------
#
# 门槛是"过不过得去"，与"走得快慢"是两件事。它们叠在**介质画像**上，
# 因为画像回答的是"这片地形对地面车辆意味着什么"，而"轮式进不了山地"
# 是具体底盘的属性。
#
# 为什么不给每个底盘各存一张成本表：那样平原的代价会在两张表里各写一遍，
# 迟早不同步；而不报错。这里做的是**减法**。


def test_blocked_terrain_closes_it_for_this_chassis_only() -> None:
    """同一张 ground 画像，轮式把山地封掉，履带不封。

    两个底盘走的**不是同一条路**——这才是"可通行能力不同"，而不是
    "其中一辆慢一点"。
    """
    grid = make_grid(size=1000.0)
    track = [cell(q, 0) for q in range(6)]
    paint(grid, track, TERRAIN_PLAIN)
    paint(grid, [cell(2, 0), cell(3, 0)], TERRAIN_MOUNTAIN)   # 中间是山

    wheeled = GROUND_TRAVEL.with_capabilities(
        blocked_terrain=frozenset({TERRAIN_MOUNTAIN})
    )
    tracked = GROUND_TRAVEL

    assert wheeled.cost_at(grid, cell(2, 0)) >= IMPASSABLE
    assert tracked.cost_at(grid, cell(2, 0)) < IMPASSABLE

    start, goal = cell(0, 0), cell(5, 0)
    assert tracked.is_passable(grid, goal)
    # 山地横在必经之路上：轮式绕不过去（两侧都是未上色的默认格 = 水域）
    assert find_path(grid, start, goal, config=wheeled) is None
    assert find_path(grid, start, goal, config=tracked) is not None


def test_blocked_terrain_can_be_disjoint_from_the_cost_table() -> None:
    """封掉的是**地貌**，不是"最贵的那几种"。

    封丘陵（代价 2）而留着山地（代价 6）是合法的——门槛与表里的数字
    没有关系，所以调表的数值不会悄悄改变谁能过。
    """
    grid = make_grid(size=1000.0)
    a = cell(0, 0)
    paint(grid, [a], TERRAIN_HILL)
    blocked = GROUND_TRAVEL.with_capabilities(
        blocked_terrain=frozenset({TERRAIN_HILL})
    )
    assert blocked.cost_at(grid, a) >= IMPASSABLE
    assert GROUND_TRAVEL.cost_at(grid, a) < IMPASSABLE
    assert MOVE_COST_TABLE[TERRAIN_HILL] < MOVE_COST_TABLE[TERRAIN_MOUNTAIN]


def test_min_water_depth_is_the_draft_gate() -> None:
    """吃水就是"这一格的水够不够深"。浅的那格拦下，深的那格放行。"""
    grid = make_grid(size=1000.0)
    shallow, deep = cell(0, 0), cell(1, 0)
    paint(grid, [shallow, deep], TERRAIN_WATER)
    grid.set_elevation(shallow, -4.0)      # 水深 4 m
    grid.set_elevation(deep, -30.0)        # 水深 30 m

    assert grid.water_depth(shallow) == pytest.approx(4.0)

    # 不写吃水 = 不限制（既有行为一字不变）
    assert WATER_TRAVEL.cost_at(grid, shallow) < IMPASSABLE

    draft = WATER_TRAVEL.with_capabilities(min_water_depth=6.0)
    assert draft.cost_at(grid, shallow) >= IMPASSABLE
    assert draft.cost_at(grid, deep) < IMPASSABLE


def test_min_water_depth_closes_every_land_cell() -> None:
    """陆地格的水深恒为 0，所以给地面件写吃水等于封死自己。

    这不是陷阱而是定义：参数说的是"至少这么深的水"。写错了要看得出来，
    而不是被框架悄悄当成"水深不影响我"。
    """
    grid = make_grid(size=1000.0)
    land = cell(0, 0)
    paint(grid, [land], TERRAIN_PLAIN)
    assert grid.water_depth(land) == 0.0

    drafted = GROUND_TRAVEL.with_capabilities(min_water_depth=1.0)
    assert drafted.cost_at(grid, land) >= IMPASSABLE


def test_gate_reason_says_which_gate_blocked() -> None:
    """走不通时要说**准**是哪条门槛，并带上数字。

    这不是锦上添花：潜艇被拦下来时那一格在地貌上写的仍是**水域**，只报
    地貌会把作者引去查一张查不出问题的地图——而答案是"水深 4 m，不到
    要求的 70 m"。三种门槛给三种说法，过的格给空串。
    """
    grid = make_grid(size=1000.0)
    shallow, mountain, plain, deep = cell(0, 0), cell(1, 0), cell(2, 0), cell(3, 0)
    paint(grid, [shallow, deep], TERRAIN_WATER)
    paint(grid, [mountain], TERRAIN_MOUNTAIN)
    paint(grid, [plain], TERRAIN_PLAIN)
    grid.set_elevation(shallow, -4.0)         # 水深 4 m
    grid.set_elevation(deep, -120.0)          # 水深 120 m

    submarine = WATER_TRAVEL.with_capabilities(min_water_depth=70.0)
    assert "水深" in submarine.gate_reason(grid, shallow)
    assert "4.0 m" in submarine.gate_reason(grid, shallow)
    assert "70.0 m" in submarine.gate_reason(grid, shallow)
    assert submarine.gate_reason(grid, deep) == ""

    wheeled = GROUND_TRAVEL.with_capabilities(
        blocked_terrain=frozenset({TERRAIN_MOUNTAIN})
    )
    assert "山地" in wheeled.gate_reason(grid, mountain)
    assert wheeled.gate_reason(grid, plain) == ""
    # 地貌本身就不在这类运载器的画像里（地面件走不了水），也要说得出来
    assert "水域" in wheeled.gate_reason(grid, shallow)


def test_gate_reason_agrees_with_passability() -> None:
    """解释与判断是同一批门槛的两个视图——两处说不到一起就是两个真相。"""
    grid = make_grid(size=1000.0)
    cells = [cell(q, 0) for q in range(4)]
    paint(grid, [cells[0], cells[3]], TERRAIN_WATER)
    paint(grid, [cells[1]], TERRAIN_PLAIN)
    paint(grid, [cells[2]], TERRAIN_MOUNTAIN)
    grid.set_elevation(cells[0], -6.0)
    grid.set_elevation(cells[3], -200.0)

    profiles = (
        WATER_TRAVEL.with_capabilities(min_water_depth=70.0),
        GROUND_TRAVEL.with_capabilities(blocked_terrain=frozenset({TERRAIN_MOUNTAIN})),
        AMPHIBIOUS_TRAVEL,
    )
    for config in profiles:
        for c in cells:
            said = config.gate_reason(grid, c)
            assert (said == "") is config.is_passable(grid, c), (
                f"{c}：能走却说不出理由、或走不了却给了空理由（{said!r}）"
            )


def test_max_slope_gate_is_per_edge_not_per_cell() -> None:
    """坡度是**边**的属性：同一格从平处来能上、从陡处来上不去。

    所以它在 ``step_cost`` 里判，不在 ``cost_at`` 里——后者只看得到
    目标格自己，会把"这一格能不能上"错答成"这一格存不存在"。
    """
    grid = make_grid(size=100.0)
    low, mid, high = cell(0, 0), cell(1, 0), cell(2, 0)
    paint(grid, [low, mid, high], TERRAIN_PLAIN)
    # 格中心距 = 边长 × √3 ≈ 173 m，所以 60 m 的高差 ≈ 坡度 0.35
    grid.set_elevation(low, 0.0)
    grid.set_elevation(mid, 60.0)
    grid.set_elevation(high, 120.0)

    gentle = GROUND_TRAVEL.with_capabilities(max_slope=0.5)
    steep = GROUND_TRAVEL.with_capabilities(max_slope=0.2)

    assert step_cost(grid, low, mid, gentle) < float("inf")
    assert step_cost(grid, low, mid, steep) == float("inf")
    # 节点级判定看不到边，所以两辆车都认为 mid 本身可通行
    assert steep.cost_at(grid, mid) < IMPASSABLE


def test_capabilities_never_mutate_the_shared_profile() -> None:
    """共享画像是模块级对象。就地改它会把门槛加到**所有**同画像的实体上
    ——表现为"改了一辆车，全战区的车都过不去山地了"，而且不报错。"""
    before = (
        GROUND_TRAVEL.blocked_terrain,
        GROUND_TRAVEL.min_water_depth,
        GROUND_TRAVEL.max_slope,
    )
    _ = GROUND_TRAVEL.with_capabilities(
        blocked_terrain=frozenset({TERRAIN_MOUNTAIN}),
        min_water_depth=9.0,
        max_slope=0.1,
    )
    after = (
        GROUND_TRAVEL.blocked_terrain,
        GROUND_TRAVEL.min_water_depth,
        GROUND_TRAVEL.max_slope,
    )
    assert after == before
    assert GROUND_TRAVEL.blocked_terrain == frozenset()


def test_copy_paths_carry_every_field() -> None:
    """`with_heuristic_weight`（回退搜索用）手抄字段时漏一个，那个门槛就
    会在回退路径上**悄悄失效**。改成 `dataclasses.replace` 之后不可能漏，
    这条测试钉住它。"""
    drafted = GROUND_TRAVEL.with_capabilities(
        blocked_terrain=frozenset({TERRAIN_MOUNTAIN}),
        min_water_depth=6.0,
        max_slope=0.3,
    )
    faster = drafted.with_heuristic_weight(1.3)
    assert faster.blocked_terrain == drafted.blocked_terrain
    assert faster.min_water_depth == drafted.min_water_depth
    assert faster.max_slope == drafted.max_slope
    assert faster.costs is drafted.costs


def test_bad_capability_values_are_rejected() -> None:
    """写错门槛值要当场报错。

    ``max_slope = 0`` 尤其重要：参数里 0 表示"不限制"，忘了翻译成 ``None``
    就会把**每一格**都封死——那看起来像"这支部队一动不动"。
    """
    with pytest.raises(ConfigurationError, match="坡度上限必须为正"):
        CostConfig(max_slope=0.0)
    with pytest.raises(ConfigurationError, match="最小水深不能为负"):
        CostConfig(min_water_depth=-1.0)
    with pytest.raises(ConfigurationError, match="未知的地貌编号"):
        CostConfig(blocked_terrain=frozenset({99}))


# ---------------------------------------------------------------------------
# 三维范围查询
# ---------------------------------------------------------------------------

def build_store(positions: dict[int, tuple[float, float, float]]) -> EntityStore:
    store = EntityStore()
    for entity_id, (x, y, z) in sorted(positions.items()):
        store.add(entity_id)
        store.set_cell(entity_id, LocalCellRef(1, 0, cell(0, 0)))
        store.set_pose(entity_id, x, y, z)
    return store


def test_ball_query_returns_distances_sorted() -> None:
    store = build_store({1: (100.0, 0.0, 0.0), 2: (10.0, 0.0, 0.0), 3: (50.0, 0.0, 0.0)})
    found = store.entities_in_ball((0.0, 0.0, 0.0), 200.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=100.0)
    assert [eid for eid, _ in found] == [2, 3, 1]
    assert found[0][1] == pytest.approx(10.0)


def test_ball_query_is_three_dimensional() -> None:
    """垂直方向远、水平方向近的目标必须被排除——这正是"同格"判据做不到的：
    同一格里 3 楼爆炸与地下室同格。"""
    store = build_store({1: (10.0, 0.0, 0.0), 2: (10.0, 0.0, 800.0)})
    found = store.entities_in_ball((0.0, 0.0, 0.0), 100.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=100.0)
    assert [eid for eid, _ in found] == [1]


def test_ball_query_includes_vertical_neighbours() -> None:
    """但真正在同一球内的垂直邻居必须被找到——空中与地面的耦合靠它。"""
    store = build_store({1: (30.0, 0.0, 40.0)})
    found = store.entities_in_ball((0.0, 0.0, 0.0), 50.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=100.0)
    assert [eid for eid, dist in found] == [1]
    assert found[0][1] == pytest.approx(50.0)


def test_ball_query_finds_targets_in_neighbouring_cells() -> None:
    """完备性：候选按**水平**半径取，所以邻格里的目标不会漏。

    设成"格边长 1 km、杀伤半径 300 m"——这正是实战尺度（榴弹破片几十米）。
    目标与球心相隔一格（1 km 的格距），拿"同格"当判据连它都看不见。"""
    store = build_store({1: (900.0, 0.0, 0.0)})
    store.set_cell(1, LocalCellRef(1, 0, cell(1, 0)))     # 落在邻格
    found = store.entities_in_ball((0.0, 0.0, 0.0), 1000.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=1000.0)
    assert [eid for eid, distance in found] == [1]
    assert found[0][1] == pytest.approx(900.0)

    # 同一格里也确实有实体（证明它不在球心的那一格）
    assert store.cell_of(1).axial != cell(0, 0)


def test_ball_query_respects_radius_exactly() -> None:
    store = build_store({1: (99.0, 0.0, 0.0), 2: (101.0, 0.0, 0.0)})
    found = store.entities_in_ball((0.0, 0.0, 0.0), 100.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=100.0)
    assert [eid for eid, _ in found] == [1]


def test_ball_query_skips_the_dead_by_default() -> None:
    store = build_store({1: (10.0, 0.0, 0.0), 2: (20.0, 0.0, 0.0)})
    store.apply_damage(2, 1.0)

    alive = store.entities_in_ball((0.0, 0.0, 0.0), 100.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=100.0)
    everything = store.entities_in_ball((0.0, 0.0, 0.0), 100.0,
                                        ref=LocalCellRef(1, 0, cell(0, 0)),
                                        cell_size_m=100.0, alive_only=False)
    assert [eid for eid, _ in alive] == [1]
    assert [eid for eid, _ in everything] == [1, 2]


def test_ball_query_order_is_reproducible() -> None:
    """同距离按 ID 升序，保证"同种子两次一致"。"""
    store = build_store({7: (30.0, 0.0, 0.0), 3: (-30.0, 0.0, 0.0), 5: (0.0, 30.0, 0.0)})
    found = store.entities_in_ball((0.0, 0.0, 0.0), 100.0,
                                   ref=LocalCellRef(1, 0, cell(0, 0)),
                                   cell_size_m=100.0)
    assert [eid for eid, _ in found] == [3, 5, 7]


def test_ball_query_guards_its_parameters() -> None:
    store = build_store({1: (10.0, 0.0, 0.0)})
    ref = LocalCellRef(1, 0, cell(0, 0))
    assert store.entities_in_ball((0.0, 0.0, 0.0), 0.0, ref=ref, cell_size_m=100.0) == []
    assert store.entities_in_ball((0.0, 0.0, 0.0), -5.0, ref=ref, cell_size_m=100.0) == []
    with pytest.raises(ValueError, match="网格边长必须为正"):
        store.entities_in_ball((0.0, 0.0, 0.0), 10.0, ref=ref, cell_size_m=0.0)


# ---------------------------------------------------------------------------
# 惰性生成与接通装配
# ---------------------------------------------------------------------------

SCENARIO = """
zone AO
    anchor     39.9042 116.4074
    radius     20 km
    resolution 500 m
    terrain    {terrain}
    seed       20260918
end_zone

platform_type TANK
    side red
end_platform_type

platform T1 TANK
    position  latlng 39.9500 116.4000
end_platform
"""


def test_activation_creates_no_grids() -> None:
    """战区**激活**只建坐标系与索引——一个网格、一个块都不建（§3.11.5）。"""
    maps = MapService()
    maps.declare(ZoneSpec(zone_id=1, name="AO", lat=39.9042, lng=116.4074,
                          radius_m=20_000.0, resolution_m=500.0))
    zone = maps.activate(1)

    assert zone.grids == {}
    assert zone.loaded_chunks == 0
    assert maps.stats["chunks_generated"] == 0
    assert maps.grid(1).chunk_count == 0      # 现建的空网格，里面什么都没有


def test_entities_are_what_triggers_generation() -> None:
    """触发点是**实体落格**，不是战区激活。

    所以"声明 20 个战区只用到 3 个"之外还有更细一层：用到的那个战区里，
    也只生成被踩到的范围。"""
    sim = Simulation.from_scenario(SCENARIO.format(terrain="procedural")).build()
    zone = sim.maps.zone(1)
    assert zone is not None
    assert zone.loaded_chunks == 1
    assert sim.maps.stats["chunks_generated"] == 1


def test_entity_position_triggers_the_load() -> None:
    """实体踩到哪才生成哪——"写了实体位置再加载相应地形"的落点。"""
    sim = Simulation.from_scenario(SCENARIO.format(terrain="procedural"))
    sim.build()
    sim.initialize()

    zone = sim.maps.zone(1)
    assert zone.loaded_chunks > 0
    assert sim.maps.stats["chunks_generated"] > 0

    grid = sim.maps.grid(1)
    assert grid is not None
    entity_id = sim.registry.by_name("T1").entity_id
    ref = sim.store.cell_of(entity_id)
    # 地形真的生成了：高程不再全是默认值
    assert grid.elevation(ref.axial) != 0.0 or grid.terrain(ref.axial) != 0


def test_loading_is_idempotent() -> None:
    sim = Simulation.from_scenario(SCENARIO.format(terrain="procedural")).build()
    sim.initialize()
    first = sim.maps.stats["chunks_generated"]

    zone = sim.maps.zone(1)
    grid = zone.grid(0)
    entity_id = sim.registry.by_name("T1").entity_id
    ref = sim.store.cell_of(entity_id)

    assert zone.ensure_loaded(ref.axial) is False    # 同一块不重复生成
    assert sim.maps.stats["chunks_generated"] == first
    assert grid.chunk_count > 0


def test_terrain_none_generates_nothing() -> None:
    """想定明确说不生成，就是一个块都不生成——不是遗漏。"""
    sim = Simulation.from_scenario(SCENARIO.format(terrain="none"))
    sim.build()
    sim.initialize()
    assert sim.maps.stats["chunks_generated"] == 0
    assert sim.maps.grid(1).chunk_count == 0


def test_flat_terrain_is_flat() -> None:
    sim = Simulation.from_scenario(SCENARIO.format(terrain="flat"))
    sim.build()
    sim.initialize()
    grid = sim.maps.grid(1)
    entity_id = sim.registry.by_name("T1").entity_id
    ref = sim.store.cell_of(entity_id)
    assert grid.elevation(ref.axial) == 0.0
    assert grid.is_passable(ref.axial)


def test_unknown_terrain_is_rejected_before_assembly() -> None:
    """未知的地形来源必须在**解析阶段**就报错，不能静默丢弃——否则会让人
    以为"写了 terrain 就有地形"（§9-12）。解析阶段报错还带上准确行号。"""
    with pytest.raises(ScenarioError, match="收到 'skyrim'"):
        load_scenario(SCENARIO.format(terrain="skyrim"))

    with pytest.raises(ScenarioError, match="terrain"):
        Simulation.from_scenario(SCENARIO.format(terrain="skyrim")).build()


def test_same_seed_generates_the_same_terrain() -> None:
    """生成必须确定：同一块任何时候生成结果一致，否则回放与
    "同种子两次一致"的承诺就崩了。"""
    def snapshot():
        sim = Simulation.from_scenario(SCENARIO.format(terrain="procedural"))
        sim.build()
        sim.initialize()
        grid = sim.maps.grid(1)
        return [grid.elevation(c) for c in (cell(q, 0) for q in range(-20, 20))]

    assert snapshot() == snapshot()


def test_different_seed_generates_different_terrain() -> None:
    def snapshot(seed: int):
        text = SCENARIO.format(terrain="procedural").replace("20260918", str(seed))
        sim = Simulation.from_scenario(text)
        sim.build()
        sim.initialize()
        grid = sim.maps.grid(1)
        return [grid.elevation(c) for c in (cell(q, 0) for q in range(-20, 20))]

    assert snapshot(1) != snapshot(2)


def test_zone_spec_carries_terrain_declarations() -> None:
    """想定写的东西必须传到 ZoneSpec——早先这两个字段是被静默丢掉的。"""
    sim = Simulation.from_scenario(SCENARIO.format(terrain="flat")).build()
    spec = sim.maps.spec(1)
    assert spec is not None
    assert spec.terrain == "flat"
    assert spec.seed == 20260918


def test_zone_spec_rejects_unknown_terrain() -> None:
    with pytest.raises(ValueError, match="未知的地形来源"):
        ZoneSpec(zone_id=1, name="AO", lat=39.9, lng=116.4,
                 radius_m=20_000.0, resolution_m=500.0, terrain="real_dem")


def test_grid_accessor_returns_none_for_inactive_zone() -> None:
    maps = MapService()
    maps.declare(ZoneSpec(zone_id=1, name="AO", lat=39.9, lng=116.4,
                          radius_m=20_000.0, resolution_m=500.0))
    assert maps.grid(1) is None
    assert maps.load_chunk_at(1, cell(0, 0)) is False


def test_chunk_generation_is_chunk_aligned() -> None:
    """生成范围对齐到分块——不对齐会让粗粒度寻路悄悄拿到默认值。"""
    sim = Simulation.from_scenario(SCENARIO.format(terrain="flat")).build()
    sim.initialize()
    grid = sim.maps.grid(1)
    for (cx, cy, cz) in grid._chunks:
        assert cz == LEVEL_SURFACE
        assert isinstance(cx, int) and isinstance(cy, int)
    assert grid.chunk_count % 1 == 0
    assert CHUNK_SIZE == 32


# ---------------------------------------------------------------------------
# 逐块生成：接缝必须对得上
# ---------------------------------------------------------------------------

def test_chunks_match_a_single_big_generation() -> None:
    """**逐块生成与整块生成必须完全一致。**

    这是世界坐标锚定噪声的核心性质。用旧实现（晶格锚在数组角上）时，
    相邻块各生成各的噪声，接缝处高程直接跳变——而跳变不报错，只是让
    寻路与通视在边界上莫名其妙。所以这条必须钉死。
    """
    whole = make_grid(size=200.0)
    generate_terrain(whole, 0, 0, 64, 64, seed=5)

    piecewise = make_grid(size=200.0)
    # 覆盖与整块生成相同的范围：2×2 个分块
    for cx in (0, 1):
        for cy in (0, 1):
            generate_terrain(piecewise, cx * 32, cy * 32, 32, 32, seed=5)

    from milsim.services.map.hex import offset_to_axial

    for col in range(0, 64, 5):
        for row in range(0, 64, 7):
            c = offset_to_axial(col, row)
            assert whole.elevation(c) == pytest.approx(piecewise.elevation(c))
            assert whole.terrain(c) == piecewise.terrain(c)


def test_water_edge_is_continuous_across_chunks() -> None:
    """水陆边界不能因为换块而跳变。

    旧实现按"本块的分位数"定水位：一块里 34% 是水、另一块里 34% 也是水，
    但阈值不同，海岸线就在接缝处错开。现在阈值是固定值，所以相邻两格
    只要高程接近，分类就不会突变。
    """
    grid = make_grid(size=200.0)
    for cx in range(4):
        generate_terrain(grid, cx * 32, 0, 32, 32, seed=9)

    from milsim.services.map.hex import offset_to_axial

    # 沿接缝扫描：跨过块边界的相邻两格，高程差应当与块内相邻格同量级
    seam_jumps = []
    inside_jumps = []
    for row in range(0, 32):
        left = grid.elevation(offset_to_axial(31, row))
        right = grid.elevation(offset_to_axial(32, row))
        seam_jumps.append(abs(right - left))
        a = grid.elevation(offset_to_axial(10, row))
        b = grid.elevation(offset_to_axial(11, row))
        inside_jumps.append(abs(b - a))

    assert max(seam_jumps) < 10 * max(1.0, max(inside_jumps))


def test_terrain_thresholds_are_shared_not_per_rect() -> None:
    """分类阈值是固定值，不随"这一次生成多大一块"变化。"""
    from milsim.services.map.terrain import _terrain_thresholds

    first = _terrain_thresholds(0.34, 6, 0.5)
    second = _terrain_thresholds(0.34, 6, 0.5)
    assert first == second
    water, hill, mountain = first
    assert water < hill < mountain


def test_urban_clusters_still_work_for_whole_region_generation() -> None:
    """城镇是跨格的特征——逐块生成会把它切碎，所以只支持整块生成。
    想定层还没暴露这个参数，但函数得留着能跑。"""
    grid = make_grid(size=200.0)
    stats = generate_terrain(grid, 0, 0, 96, 96, seed=4, urban_clusters=3)
    assert stats.cells == 96 * 96

    from milsim.services.map.hex import offset_to_axial

    urban = [
        offset_to_axial(col, row)
        for col in range(96)
        for row in range(96)
        if grid.terrain(offset_to_axial(col, row)) == 5      # TERRAIN_URBAN
    ]
    assert urban


def test_entity_level_is_derived_from_height_above_ground() -> None:
    """垂向层按**离地高度**分，不是海拔：高原上的坦克 AGL 只有几米，
    用海拔会把它算成低空。"""
    from milsim.services.map import level_of_altitude

    plateau_ground = 1200.0
    tank_z = plateau_ground + 2.0
    assert level_of_altitude(tank_z - plateau_ground) == LEVEL_SURFACE
    # 而直接拿海拔去分就会错成低空
    assert level_of_altitude(tank_z) == 1
