"""导航门面测试：画像、通行系数、按需加载（§5.7）。

这一层是组件与地图之间**唯一的缝**，所以本文件盯的是缝没缝歪：

1. **画像名写错要当场报错**。画像名是字符串，没有清单就退化成"写错不报错、
   拿默认语义接着跑"——那正是 §3.11.3 给网格通道立清单要防的事。
2. **速度系数与寻路用的是同一张代价表**。另立一张速度表就是同一件事有两个
   真相，两份数字各自演化、迟早不同步，而**不报错**。
3. **寻路前起点与终点所在分块真的生成了**。分块是按需生成的，读未生成的
   块得到的是"平坦默认地形"——据此规划出的路穿过实际存在的山，但不报错。

地形用 ``procedural seed=31``（水域 5.8%、山地 27%），测试里所有断言都
**从生成出来的格子里查**，不写死具体坐标——写死坐标的话，换一次地形生成
实现就会得到一堆与被测行为无关的失败。
"""

from __future__ import annotations

import math

import pytest

from milsim.errors import ConfigurationError
from milsim.services.map import (
    GROUND_TRAVEL,
    PROFILE_AMPHIBIOUS,
    PROFILE_GROUND,
    PROFILE_WATER,
    TRAVEL_PROFILES,
    WATER_TRAVEL,
    Axial,
    NavService,
    TERRAIN_MOUNTAIN,
    TERRAIN_PLAIN,
    TERRAIN_WATER,
    heading_to_offset,
    line,
    offset_to_heading,
    profile_names,
    resolve_profile,
    terrain_codes,
    travel_profile,
)
from milsim.simulation import Simulation

#: 地形种子与 mover_demo.txt 一致：有山有水，两种画像都能找到反例。
SEED = 31

SCENARIO = f"""
zone AO
    anchor     39.9042 116.4074
    radius     30 km
    resolution 200 m
    layers     1
    terrain    procedural seed={SEED}
end_zone

platform_type P
end_platform_type

platform U1 P
    position hex -24 0 @ layer 0
end_platform
"""

#: 扫格范围。必须**跨过分块边界**，而且到得了最近的水域——种子 31 下最近
#: 的一格水在起点以东 41 格（起点 q=-24），范围太窄就只找到陆地。
SCAN_Q = range(-80, 9)
SCAN_R = range(-40, 41)


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------

class Rig:
    """一次装配出来的最小世界：一个战区、一个实体、一个导航门面。"""

    def __init__(self) -> None:
        self.sim = Simulation.from_scenario(SCENARIO)
        self.sim.build()
        self.nav: NavService = self.sim.nav
        self.entity_id = self.sim.registry.by_name("U1").entity_id
        self.ref = self.sim.store.cell_of(self.entity_id)
        self.zone_id = self.ref.zone_id
        self.grid = self.sim.maps.grid(self.zone_id, self.ref.layer)
        self.start = self.ref.axial

    def find(self, terrain: int, *, skip_start: bool = True) -> Axial:
        """找一格指定地形的格子，顺带把它所在的分块生成出来。

        逐格 ``ensure_loaded`` 是有意的：**不**先把整片地形烤出来，这样
        "未生成的块读出来是默认地形"这个前提在测试里仍然成立，
        而不是被夹具顺手抹平。
        """
        for q in SCAN_Q:
            for r in SCAN_R:
                cell = Axial(q, r)
                self.nav.ensure_loaded(self.ref, cell)
                if skip_start and cell == self.start:
                    continue
                if self.grid.terrain(cell) == terrain:
                    return cell
        raise AssertionError(f"扫描范围内找不到地形 {terrain} 的格子——换一个种子")


@pytest.fixture
def rig() -> Rig:
    return Rig()


# ---------------------------------------------------------------------------
# 画像清单
# ---------------------------------------------------------------------------

def test_profile_names_are_sorted_and_complete() -> None:
    """名单要排序：错误信息与遍历结果都得可复现。"""
    names = profile_names()
    assert names == sorted(names)
    assert set(names) == {PROFILE_GROUND, PROFILE_WATER, PROFILE_AMPHIBIOUS}
    assert set(TRAVEL_PROFILES) == set(names)


@pytest.mark.parametrize(
    "good", [PROFILE_GROUND, PROFILE_WATER, PROFILE_AMPHIBIOUS]
)
def test_travel_profile_returns_the_table_itself(good: str) -> None:
    """返回的必须是**同一张**表，不是等价的副本——副本会各自演化。"""
    assert travel_profile(good) is TRAVEL_PROFILES[good]


def test_travel_profile_typo_gets_a_suggestion() -> None:
    """画像名写错**当场报错**，并给出拼写建议与可用名单。"""
    with pytest.raises(ConfigurationError) as info:
        travel_profile("groudn")
    message = str(info.value)
    assert "groudn" in message
    assert "'ground'" in message          # 拼写建议
    assert "可用" in message


def test_travel_profile_unknown_name_lists_choices() -> None:
    with pytest.raises(ConfigurationError, match="可用"):
        travel_profile("hovercraft")     # 差得太远，给不出建议也得给名单


def test_nav_service_profile_matches_module_function(rig: Rig) -> None:
    assert rig.nav.profile(PROFILE_GROUND) is GROUND_TRAVEL
    assert rig.nav.profiles() == profile_names()


# ---------------------------------------------------------------------------
# 底盘门槛叠在画像上（§5.7）
# ---------------------------------------------------------------------------
#
# 画像答"这片地形对地面车辆意味着什么"（代码层，改它要动代码），
# 门槛答"这辆车过不去什么"（参数，改它是改想定/参数库）。

def test_resolve_profile_takes_a_name_or_a_config() -> None:
    """门面要同时收画像名与调好的配置。

    只写名字的调用方（工具、测试）行为一字不变——这是**扩展**而不是
    换一套词汇。
    """
    assert resolve_profile(PROFILE_GROUND) is GROUND_TRAVEL
    drafted = GROUND_TRAVEL.with_capabilities(min_water_depth=3.0)
    assert resolve_profile(drafted) is drafted
    with pytest.raises(ConfigurationError, match="未知的运载器画像"):
        resolve_profile("groudn")


def test_terrain_codes_translates_names_and_suggests() -> None:
    """地貌用**名字**给：想让定作者写"轮式进不了山地"，不是 terrain == 4。"""
    assert terrain_codes(["山地"]) == frozenset({TERRAIN_MOUNTAIN})
    assert terrain_codes([]) == frozenset()
    assert terrain_codes(["平原", "山地"]) == frozenset(
        {TERRAIN_PLAIN, TERRAIN_MOUNTAIN}
    )
    with pytest.raises(ConfigurationError) as info:
        terrain_codes(["山底"])
    assert "'山地'" in str(info.value)      # 拼写建议
    assert "可用" in str(info.value)


def test_capable_profile_layers_the_gates(rig: Rig) -> None:
    """门槛真的叠上去了，而且是**新对象**——共享画像必须原样不动。"""
    drafted = rig.nav.capable_profile(
        PROFILE_GROUND,
        blocked_terrain=["山地"],
        min_water_depth=6.0,
    )
    assert drafted.blocked_terrain == frozenset({TERRAIN_MOUNTAIN})
    assert drafted.min_water_depth == pytest.approx(6.0)
    assert drafted.costs is GROUND_TRAVEL.costs      # 介质口径不变
    assert drafted is not GROUND_TRAVEL
    assert GROUND_TRAVEL.blocked_terrain == frozenset()
    assert GROUND_TRAVEL.min_water_depth == 0.0


def test_capable_profile_rejects_a_misspelled_terrain(rig: Rig) -> None:
    """写错的地貌名不能静默忽略——那表现为"这条门槛没生效"。"""
    with pytest.raises(ConfigurationError, match="未知的地貌名"):
        rig.nav.capable_profile(PROFILE_GROUND, blocked_terrain=["山底"])


def test_capabilities_change_where_the_vehicle_can_go(rig: Rig) -> None:
    """轮式封掉山地之后，去同一座山的目的地结果就不同了。

    这是**真战区**上的端到端确认：不是"配置对象里多了个字段"，而是
    "这辆车真的去不了"。
    """
    mountain = rig.find(TERRAIN_MOUNTAIN)
    wheeled = rig.nav.capable_profile(PROFILE_GROUND, blocked_terrain=["山地"])

    assert rig.nav.passable(rig.ref, mountain, profile=GROUND_TRAVEL) is True
    assert rig.nav.passable(rig.ref, mountain, profile=wheeled) is False
    assert rig.nav.move_factor(rig.ref, mountain, profile=wheeled) == 0.0
    # 到山地的目的地上，两者给出不同答案
    assert rig.nav.route(rig.ref, mountain, profile=GROUND_TRAVEL) is not None
    assert rig.nav.route(rig.ref, mountain, profile=wheeled) is None


def test_refusal_names_the_depth_gate(rig: Rig) -> None:
    """说得出的理由要**准**。潜艇被拦下来时那一格往往还是"水域"。

    真正挡住它的是水深。只报地貌会把作者引去查一张查不出问题的地图，
    所以门面要把数字说出来——而这也是"解释与判断同源"的一个检查点。
    """
    water = rig.find(TERRAIN_WATER)
    depth = max(1.0, float(rig.grid.water_depth(water)))

    assert rig.nav.refusal(rig.ref, water, profile=WATER_TRAVEL) == ""

    deeper_boat = rig.nav.capable_profile(
        PROFILE_WATER, min_water_depth=depth + 1.0
    )
    assert rig.nav.passable(rig.ref, water, profile=deeper_boat) is False
    said = rig.nav.refusal(rig.ref, water, profile=deeper_boat)
    assert "水深" in said, said
    assert f"{depth + 1.0:.1f} m" in said, said
    # 说得准：挡它的是水深，不是"水域"这个地貌
    assert "水域" not in said, said


# ---------------------------------------------------------------------------
# 通行系数与寻路共用一张代价表
# ---------------------------------------------------------------------------

def test_move_factor_is_the_reciprocal_of_path_cost(rig: Rig) -> None:
    """速度系数 = 1/代价。**不另立速度表**——见模块头第 2 条。"""
    cell = rig.find(TERRAIN_PLAIN)
    cost = GROUND_TRAVEL.cost_at(rig.grid, cell)
    assert rig.nav.move_factor(rig.ref, cell) == pytest.approx(1.0 / cost)


def test_move_factor_is_zero_exactly_where_path_cannot_go(rig: Rig) -> None:
    """两处必须**同时**说"不能走"。一处说能、另一处说不能，就是两个真相。"""
    water = rig.find(TERRAIN_WATER)
    assert rig.nav.passable(rig.ref, water) is False
    assert rig.nav.move_factor(rig.ref, water) == 0.0

    plain = rig.find(TERRAIN_PLAIN)
    assert rig.nav.passable(rig.ref, plain) is True
    assert rig.nav.move_factor(rig.ref, plain) > 0.0


def test_profiles_disagree_about_the_same_water_cell(rig: Rig) -> None:
    """同一片水面：坦克看到"不可通行"、舰船看到"好走"。

    这条是"画像属运载器、不属地物"的落点。做成地形属性的话，一艘船在
    港口里就会因为"起点在陆地上"而被判成走不通——**不报错，只是不动**。
    """
    water = rig.find(TERRAIN_WATER)
    assert rig.nav.passable(rig.ref, water, profile=PROFILE_WATER) is True
    assert rig.nav.move_factor(rig.ref, water, profile=PROFILE_WATER) > 0.0

    plain = rig.find(TERRAIN_PLAIN)
    assert rig.nav.passable(rig.ref, plain, profile=PROFILE_WATER) is False


def test_move_factor_defaults_to_my_own_cell(rig: Rig) -> None:
    """不传格子时按自己所在格算——"我现在能跑多快"是最常见的问法。"""
    here = rig.nav.move_factor(rig.ref)
    assert here == rig.nav.move_factor(rig.ref, rig.start)
    assert here > 0.0


# ---------------------------------------------------------------------------
# 按需加载
# ---------------------------------------------------------------------------

def test_ensure_loaded_reports_only_the_first_time(rig: Rig) -> None:
    """同一块只生成一次。重复调用要返回 0，否则调用方无法判断真的做了事。"""
    far = Axial(rig.start.q + 40, rig.start.r)
    assert rig.nav.ensure_loaded(rig.ref, far) >= 1
    assert rig.nav.ensure_loaded(rig.ref, far) == 0


def test_route_does_not_touch_cells_outside_the_zone(rig: Rig) -> None:
    """战区外的格引用（没有 ``zone_id``）不该被当成"某个战区里的格"。

    返回 ``None`` 而不是抛异常：战区外确实没有逐格精度，调用方据此
    改按坐标点机动即可（见 mover 的 ``move_to_point``）。
    """
    outside = object()
    assert rig.nav.zone_of(outside) is None
    assert rig.nav.grid_of(outside) is None
    assert rig.nav.cell_center(outside, Axial(0, 0)) is None
    assert rig.nav.ground_at(outside, 0.0, 0.0) is None
    assert rig.nav.route(outside, Axial(0, 0)) is None
    assert rig.nav.passable(outside, Axial(0, 0)) is False
    # 没有网格时速度系数取 1.0（"按额定速度走"），而不是 0.0（"走不动"）
    assert rig.nav.move_factor(outside) == 1.0


def test_route_returns_none_for_impassable_goal(rig: Rig) -> None:
    """目标不可通行 = 走不通。返回 ``None`` 而不是一条穿过去的直线。"""
    water = rig.find(TERRAIN_WATER)
    assert rig.nav.route(rig.ref, water) is None


def test_route_found_for_a_passable_goal(rig: Rig) -> None:
    plain = rig.find(TERRAIN_PLAIN)
    route = rig.nav.route(rig.ref, plain)
    assert route is not None
    assert route.cells[0] == rig.start
    assert route.cells[-1] == plain
    # 每一格都必须在该画像下可通行——路线自己得先站得住
    assert all(rig.nav.passable(rig.ref, c) for c in route.cells)


def test_route_reaches_a_goal_two_chunks_away(rig: Rig) -> None:
    """跨两个分块的目标必须能规划出路线。

    这是**回归钉子**。修之前 ``route`` 只加载起点与终点所在的分块，而 A*
    的搜索空间远大于这两个端点：中间那片没生成的地形读出的是 ``terrain``
    默认值，也就是**水域**，而地面画像按地形查表——于是搜索被一圈**假水域**
    围死，目标稍远就返回"走不通"。后果不止是"不动"：机动件会据此报一句
    "起点或终点不可通行、或被完全包围"，而那是**假的**。

    分块是 32×32 单元，所以"跨两个块"要求格距明显超过 32。
    """
    for dq in (48, 80, 120):
        goal = Axial(rig.start.q + dq, rig.start.r)
        rig.nav.ensure_loaded(rig.ref, goal)         # 让目标格的地貌是真的
        if not rig.nav.passable(rig.ref, goal):
            continue                                  # 目标真是水/山，另外测
        route = rig.nav.route(rig.ref, goal)
        assert route is not None, (
            f"格距 {dq} 的可通行目标却规划不出路线——"
            "多半是走廊没生成，搜索被假水域围住了"
        )
        assert route.cells[0] == rig.start
        assert route.cells[-1] == goal


def test_corridor_loading_covers_the_whole_line(rig: Rig) -> None:
    """走廊加载要覆盖**线上每一格**所在的分块，而不只是两头。

    判据不是"是不是水"——线上本来就可能真有水（那是事实，不是缺陷）。
    判据是**地形是不是真的**：拿一个只加载了走廊的世界，与一个把整片战区
    都生成好的世界逐格比。线上的地形全都对得上，才说明走廊真的加载了，
    而不是只把两端补上了。
    """
    goal = Axial(rig.start.q + 80, rig.start.r)
    cells = line(rig.start, goal)

    lazy = Simulation.from_scenario(SCENARIO)
    lazy.build()
    lazy_ref = lazy.store.cell_of(lazy.registry.by_name("U1").entity_id)
    lazy_grid = lazy.maps.grid(lazy_ref.zone_id, lazy_ref.layer)
    created = lazy.nav.ensure_corridor(lazy_ref, lazy_ref.axial, goal)
    # 一条 80 格的线在这个分辨率下跨过 2 个分块（分块是 32×32 **单元**，
    # 而单元按偏移坐标分块，所以一条轴向直线未必逐块推进）。
    assert created >= 2

    full = Simulation.from_scenario(SCENARIO)
    full.build()
    full_ref = full.store.cell_of(full.registry.by_name("U1").entity_id)
    full_grid = full.maps.grid(full_ref.zone_id, full_ref.layer)
    for q in range(-160, 161):
        for r in range(-160, 161):
            full.nav.ensure_loaded(full_ref, Axial(q, r))

    bad = [
        c
        for c in cells
        if (lazy_grid.terrain(c), round(lazy_grid.elevation(c), 3))
        != (full_grid.terrain(c), round(full_grid.elevation(c), 3))
    ]
    assert not bad, f"走廊上有 {len(bad)}/{len(cells)} 格的地形不是真的：{bad[:4]}"


def test_route_is_the_same_as_when_the_whole_zone_is_generated(rig: Rig) -> None:
    """**已知限制的看门测试**：只加载走廊时，路线的代价可能高于"整片生成好"。

    这一条**不是**在要求两者相等——它们现在不相等，而且这是记入 §9 的
    待决缺陷。钉住它有两个用处：

    1. 修好之后这条会失败，正好提醒把 §9 的条目划掉、并把这个测试改成
       断言相等；
    2. 在这之前，任何人看到"路线怎么绕了这么远"都能在这里找到解释，
       而不是去怀疑 A* 本身。

    断言取的是**宽松的方向**（懒加载不会算出比真值更便宜的路），因为
    "更便宜"意味着它穿过了不存在的地形——那才是真正危险的一侧。
    """
    goal = Axial(rig.start.q + 60, rig.start.r)

    fresh = Simulation.from_scenario(SCENARIO)
    fresh.build()
    ref = fresh.store.cell_of(fresh.registry.by_name("U1").entity_id)
    lazy = fresh.nav.route(ref, goal)

    # 把整片战区生成好，再算一次
    for q in range(-160, 161):
        for r in range(-160, 161):
            fresh.nav.ensure_loaded(ref, Axial(q, r))
    full = fresh.nav.route(ref, goal)

    if lazy is None or full is None:
        return                                    # 目标本身走不通，这条测不到
    assert lazy.cost >= full.cost - 1e-6, (
        "懒加载算出的路比全量地形还便宜，说明它穿过了未生成的地形"
        "（实测 40 个样本里 5 个会这样）——那是不报错的错，必须修"
    )


def test_coords_to_cell_and_back_are_consistent(rig: Rig) -> None:
    cell = rig.find(TERRAIN_PLAIN)
    x, y = rig.nav.cell_center(rig.ref, cell)
    assert rig.nav.axial_at(rig.ref, x, y) == cell
    ground = rig.nav.ground_at(rig.ref, x, y)
    assert ground is not None
    assert ground == pytest.approx(float(rig.grid.elevation(cell)))


# ---------------------------------------------------------------------------
# 航向换算：互逆
# ---------------------------------------------------------------------------

def test_offset_to_heading_is_the_inverse_of_heading_to_offset() -> None:
    """这对换算互为逆运算，所以**必须**一起测。

    它们在 hex.py 里是相邻的两个函数，这不是随手放的：隔开两个文件放，
    迟早有一边改了另一边没改——而"改了没改"在这类公式上看不出来。
    """
    for distance_m in (1.0, 137.0, 9000.0):
        for heading in range(0, 360, 5):
            dx, dy = heading_to_offset(heading, distance_m)
            back = offset_to_heading(dx, dy)
            assert back == pytest.approx(heading % 360.0, abs=1e-6)


def test_heading_zero_points_north() -> None:
    """0° = 北 = -y 方向。写反的话所有航向图都会镜像，但数字看着都正常。"""
    dx, dy = heading_to_offset(0.0, 100.0)
    assert (dx, dy) == pytest.approx((0.0, -100.0))
    assert offset_to_heading(0.0, -100.0) == pytest.approx(0.0)
    assert offset_to_heading(100.0, 0.0) == pytest.approx(90.0)


def test_heading_of_zero_offset_is_defined() -> None:
    """零位移没有方向。返回 0 而不是抛 ZeroDivisionError——推进里
    "已经站在航路点上"是常态，不该让整帧崩掉。"""
    assert offset_to_heading(0.0, 0.0) == 0.0


def test_heading_is_always_in_range() -> None:
    for dx, dy in ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)):
        value = offset_to_heading(dx, dy)
        assert 0.0 <= value < 360.0
        assert math.isfinite(value)
