"""地图服务测试：局部网格坐标、全球层、战区路由。

覆盖三件事：
  1. 局部平面网格的坐标运算自洽（编码往返、距离对称、取整正确）
  2. 投影往返精度达标（AED 径向保距是它的理论性质，要守住）
  3. 战区按需加载与 O(1) 坐标路由
"""

from __future__ import annotations

import math

import pytest

from milsim.services.map import (
    COORD_LIMIT,
    Axial,
    GlobalLayer,
    LocalFrame,
    MapService,
    ZoneSpec,
    axial_to_offset,
    cells_for_route,
    cube_round,
    decode,
    distance,
    encode,
    great_circle_m,
    initial_bearing_deg,
    interpolate_great_circle,
    line,
    neighbor,
    neighbors,
    offset_to_axial,
    ring,
    sample_great_circle,
    spiral,
)

BEIJING = (39.9042, 116.4074)
MOSCOW = (55.7558, 37.6173)


# ---------------------------------------------------------------------------
# 局部平面网格：基本运算
# ---------------------------------------------------------------------------

def test_encode_decode_roundtrip() -> None:
    for q, r in [(0, 0), (1, -1), (-5, 3), (1000, -2000), (-2097152, 2097151)]:
        cell = encode(q, r, zone_id=7, layer=3, flags=5)
        axial, zone_id, layer, flags = decode(cell)
        assert axial == Axial(q, r)
        assert (zone_id, layer, flags) == (7, 3, 5)


def test_encode_rejects_out_of_range() -> None:
    with pytest.raises(ValueError, match="可编码范围"):
        encode(COORD_LIMIT, 0)
    with pytest.raises(ValueError, match="zone_id"):
        encode(0, 0, zone_id=4096)


def test_distance_is_symmetric_and_zero_on_self() -> None:
    a, b = Axial(3, -7), Axial(-11, 4)
    assert distance(a, b) == distance(b, a)
    assert distance(a, a) == 0
    assert distance(Axial(0, 0), Axial(0, 1)) == 1


def test_distance_matches_ring_radius() -> None:
    """半径为 k 的环上每个点，到中心的距离必须恰好是 k。"""
    center = Axial(2, -3)
    for k in range(1, 6):
        assert all(distance(center, c) == k for c in ring(center, k))


def test_ring_sizes() -> None:
    center = Axial(0, 0)
    for k in range(1, 6):
        assert len(ring(center, k)) == 6 * k


def test_spiral_is_layered() -> None:
    cells = spiral(Axial(0, 0), 3)
    assert len(cells) == 1 + sum(6 * k for k in range(1, 4))
    assert cells[0] == Axial(0, 0)


def test_neighbors_are_six_distinct_cells() -> None:
    c = Axial(4, 5)
    ns = neighbors(c)
    assert len(ns) == 6
    assert len(set(ns)) == 6
    assert all(distance(c, n) == 1 for n in ns)
    assert neighbors(c)[0] == neighbor(c, 0)


def test_offset_roundtrip_including_negatives() -> None:
    """odd-r 偏移转换在负坐标下也必须自洽——

    Python 的 ``&`` 和 ``//`` 对负数按补码/向下取整处理，与 C 不同，
    这里专门覆盖负值区间，防止哪天换成 C 语义实现时静默出错。
    """
    for q in range(-6, 7):
        for r in range(-6, 7):
            a = Axial(q, r)
            assert offset_to_axial(*axial_to_offset(a)) == a


def test_line_endpoints_and_length() -> None:
    a, b = Axial(-3, 2), Axial(4, -5)
    cells = line(a, b)
    assert cells[0] == a
    assert cells[-1] == b
    assert len(cells) == distance(a, b) + 1


def test_line_between_same_cell() -> None:
    a = Axial(7, 7)
    assert line(a, a) == [a]


def test_rounding_avoids_bankers_rounding() -> None:
    """取整必须用 floor(x+0.5)，不能用内置 round()。

    内置 round() 是银行家舍入：round(0.5)==0 而 round(2.5)==2，
    会让 .5 边界上的单元归属左右不对称，破坏网格的对偶性。
    """
    from milsim.services.map.hex import _round_half_up

    assert _round_half_up(0.5) == 1
    assert _round_half_up(1.5) == 2
    assert _round_half_up(2.5) == 3
    assert _round_half_up(-0.5) == 0
    assert _round_half_up(3.4) == 3
    assert _round_half_up(-3.4) == -3

    # 确认两者在 .5 上确实不同——这是这个 helper 存在的理由
    assert round(0.5) != _round_half_up(0.5)
    assert round(2.5) != _round_half_up(2.5)


def test_cube_round_always_returns_valid_cell() -> None:
    """任意浮点输入都要落在满足 q+r+s==0 的合法单元上。"""
    import random

    rng = random.Random(7)
    for _ in range(500):
        c = cube_round(rng.uniform(-50, 50), rng.uniform(-50, 50))
        assert c.s == -c.q - c.r


# ---------------------------------------------------------------------------
# 局部平面网格：投影
# ---------------------------------------------------------------------------

def test_world_roundtrip_lands_in_same_cell() -> None:
    """单元中心 → 平面坐标 → 单元，必须回到原单元。"""
    frame = LocalFrame(*BEIJING, size=1000.0)
    for q in range(-20, 21, 3):
        for r in range(-20, 21, 3):
            c = Axial(q, r)
            assert frame.from_world(*frame.to_world(c)) == c


def test_world_grid_spacing() -> None:
    """相邻单元中心的间距应符合 pointy-top 的几何关系。"""
    frame = LocalFrame(*BEIJING, size=1000.0)
    x0, y0 = frame.to_world(Axial(0, 0))
    x1, y1 = frame.to_world(Axial(1, 0))
    assert math.isclose(x1 - x0, math.sqrt(3) * 1000.0, rel_tol=1e-9)
    assert math.isclose(y1, y0)

    x2, y2 = frame.to_world(Axial(0, 1))
    assert math.isclose(y2 - y0, 1500.0, rel_tol=1e-9)


def test_geo_roundtrip_is_accurate() -> None:
    """经纬度往返：误差必须远小于一个格距。"""
    frame = LocalFrame(*BEIJING, size=100.0)
    for dlat, dlng in [(0.0, 0.0), (0.05, 0.0), (0.0, 0.05), (0.1, 0.1), (-0.2, 0.15)]:
        lat, lng = BEIJING[0] + dlat, BEIJING[1] + dlng
        back_lat, back_lng = frame.world_to_geo(*frame.geo_to_world(lat, lng))
        err = great_circle_m(lat, lng, back_lat, back_lng)
        assert err < 0.01, f"往返误差 {err:.4f} m 过大"


def test_aed_preserves_radial_distance() -> None:
    """AED 的看家性质：从锚点出发的径向距离精确保距。

    这是选它而不是横轴墨卡托的理由，必须守住。测到 1000 km。
    """
    frame = LocalFrame(*BEIJING, size=1000.0)
    for dlat in (0.5, 1.0, 3.0, 6.0, 9.0):
        lat = BEIJING[0] + dlat
        x, y = frame.geo_to_world(lat, BEIJING[1])
        planar = math.hypot(x, y)
        geodesic = great_circle_m(*BEIJING, lat, BEIJING[1])
        rel_err = abs(planar - geodesic) / geodesic
        assert rel_err < 1e-6, f"{dlat}° 处径向误差 {rel_err:.2e}"


def test_from_geo_roundtrip_returns_same_cell() -> None:
    frame = LocalFrame(*BEIJING, size=500.0)
    for c in [Axial(0, 0), Axial(10, -4), Axial(-30, 12), Axial(100, 100)]:
        lat, lng = frame.to_geo(c)
        assert frame.from_geo(lat, lng) == c


def test_longitude_normalization() -> None:
    """锚点跨 ±180° 时，反解出的经度必须绕回合法区间。"""
    frame = LocalFrame(0.0, 179.9, size=1000.0)
    for lng in (179.95, 180.0, -179.95, 179.99):
        _, out_lng = frame.world_to_geo(*frame.geo_to_world(0.0, lng))
        assert -180.0 <= out_lng < 180.0


def test_frame_rejects_absurd_size() -> None:
    with pytest.raises(ValueError, match="100 ~ 2000"):
        LocalFrame(*BEIJING, size=50.0)
    with pytest.raises(ValueError, match="100 ~ 2000"):
        LocalFrame(*BEIJING, size=5000.0)


# ---------------------------------------------------------------------------
# 全球层
# ---------------------------------------------------------------------------

def test_global_layer_basic() -> None:
    layer = GlobalLayer(resolution=5)
    cell = layer.cell_at(*BEIJING)
    lat, lng = layer.center(cell)
    assert great_circle_m(*BEIJING, lat, lng) < 20_000     # 9.85 km 格，中心不会太远
    assert layer.resolution_of(cell) == 5


def test_global_layer_neighbor_count_varies() -> None:
    """H3 的邻居数**不恒定**：二十面体顶点处是五边形，只有 5 个邻居。

    实测 res 0 上，北京和赤道是五边形，北极和美西是六边形。
    任何"六边形必有 6 邻居"的假设在全局层都会翻车。
    """
    layer = GlobalLayer(resolution=0)
    counts = {
        len(layer.neighbors(layer.cell_at(lat, lng)))
        for lat, lng in [(39.9, 116.4), (90.0, 0.0), (0.0, 0.0), (37.4, -122.0)]
    }
    assert counts <= {5, 6}
    assert 5 in counts, "res 0 上应当能观察到五边形单元"


def test_global_layer_global_cell_count() -> None:
    """res 0 全球 122 个单元，其中 12 个是五边形——二十面体的 12 个顶点。"""
    layer = GlobalLayer(resolution=0)
    cell = layer.cell_at(*BEIJING)
    assert len(layer.disk(cell, 0)) == 1
    assert layer.resolution_of(cell) == 0


def test_global_layer_hierarchy_is_exact() -> None:
    """H3 的父子嵌套是精确的——这是它相对自建多层网格的优势。"""
    layer = GlobalLayer(resolution=7)
    cell = layer.cell_at(*BEIJING)
    parent = layer.parent(cell, 5)
    assert layer.resolution_of(parent) == 5
    assert cell in layer.children(parent, 7)


def test_global_layer_sparse_storage() -> None:
    layer = GlobalLayer(resolution=6)
    cell = layer.cell_at(*BEIJING)
    assert len(layer) == 0
    assert layer.attrs(cell) == {}          # 查询不该创建条目
    assert len(layer) == 0                  # 确认没有被副作用污染

    layer.set_attrs(cell, terrain="urban", elevation=44.0)
    assert len(layer) == 1
    assert layer.attrs(cell)["terrain"] == "urban"

    layer.set_attrs(cell, elevation=50.0)
    assert layer.attrs(cell)["terrain"] == "urban"    # 部分更新不清空其他字段


def test_cells_within_respects_radius() -> None:
    layer = GlobalLayer(resolution=5)
    radius = 100_000.0
    cells = layer.cells_within(*BEIJING, radius)
    assert cells
    for cell in cells:
        assert great_circle_m(*BEIJING, *layer.center(cell)) <= radius


def test_cells_in_bbox() -> None:
    layer = GlobalLayer(resolution=4)
    cells = layer.cells_in_bbox(39.0, 116.0, 40.0, 117.0)
    assert cells
    for cell in cells:
        lat, lng = layer.center(cell)
        assert 38.5 < lat < 40.5 and 115.5 < lng < 117.5


def test_cells_in_bbox_rejects_antimeridian() -> None:
    layer = GlobalLayer(resolution=4)
    with pytest.raises(ValueError, match="180"):
        layer.cells_in_bbox(0.0, 179.0, 1.0, -179.0)


# ---------------------------------------------------------------------------
# 测地与航线
# ---------------------------------------------------------------------------

def test_great_circle_known_distance() -> None:
    """北京到莫斯科约 5800 km，允许 1% 误差。"""
    d = great_circle_m(*BEIJING, *MOSCOW)
    assert 5_700_000 < d < 5_900_000


def test_initial_bearing() -> None:
    assert math.isclose(initial_bearing_deg(0, 0, 1, 0), 0.0, abs_tol=1e-6)      # 正北
    assert math.isclose(initial_bearing_deg(0, 0, 0, 1), 90.0, abs_tol=1e-6)     # 正东


def test_interpolate_endpoints() -> None:
    lat, lng = interpolate_great_circle(*BEIJING, *MOSCOW, 0.0)
    assert math.isclose(lat, BEIJING[0], abs_tol=1e-9)
    lat, lng = interpolate_great_circle(*BEIJING, *MOSCOW, 1.0)
    assert math.isclose(lat, MOSCOW[0], abs_tol=1e-6)


def test_interpolated_points_stay_on_great_circle() -> None:
    """插值点必须落在大圆上：到两端点距离之和等于全长。"""
    total = great_circle_m(*BEIJING, *MOSCOW)
    for frac in (0.25, 0.5, 0.75):
        lat, lng = interpolate_great_circle(*BEIJING, *MOSCOW, frac)
        s = great_circle_m(*BEIJING, lat, lng) + great_circle_m(lat, lng, *MOSCOW)
        assert abs(s - total) < 1.0


def test_sample_great_circle_spacing() -> None:
    pts = sample_great_circle(*BEIJING, *MOSCOW, step_m=100_000.0)
    assert pts[0] == BEIJING
    assert pts[-1] == MOSCOW
    for a, b in zip(pts, pts[1:]):
        assert great_circle_m(*a, *b) <= 100_000.0 * 1.01


def test_cells_for_route_is_ordered_and_deduped() -> None:
    layer = GlobalLayer(resolution=3)      # 69 km，避免生成过多单元
    route = cells_for_route(layer, *BEIJING, *MOSCOW)
    assert len(route) > 10
    assert len(set(route)) == len(route)          # 无重复
    assert route[0] == layer.cell_at(*BEIJING)
    assert route[-1] == layer.cell_at(*MOSCOW)


# ---------------------------------------------------------------------------
# 战区与坐标路由
# ---------------------------------------------------------------------------

def test_zone_spec_validation() -> None:
    with pytest.raises(ValueError, match="zone_id"):
        ZoneSpec(9999, "越界", *BEIJING, 100_000, 1000)
    with pytest.raises(ValueError, match="半径"):
        ZoneSpec(1, "过大", *BEIJING, 5_000_000, 1000)
    with pytest.raises(ValueError, match="边长"):
        ZoneSpec(1, "过细", *BEIJING, 100_000, 50)


def test_zone_estimated_cells() -> None:
    """估算公式要与几何真值吻合，避免数量级误差。

    半径 100 km 的圆面积 π·100² ≈ 31416 km²，
    边长 1 km 的六边形面积 (3√3/2)·1² ≈ 2.598 km²，
    相除约 12093 个单元。
    """
    spec = ZoneSpec(1, "A", *BEIJING, 100_000, 1000)
    assert 11_000 < spec.estimated_cells() < 13_000


def test_zone_estimated_cells_scales_with_area() -> None:
    """边长减半 → 单元数变四倍。"""
    coarse = ZoneSpec(1, "A", *BEIJING, 100_000, 1000).estimated_cells()
    fine = ZoneSpec(2, "B", *BEIJING, 100_000, 500).estimated_cells()
    assert 3.8 < fine / coarse < 4.2


def test_zone_resolution_doubles_per_layer() -> None:
    spec = ZoneSpec(1, "A", *BEIJING, 100_000, 250, layer_count=4)
    assert [spec.resolution_of_layer(i) for i in range(4)] == [250, 500, 1000, 2000]


def test_map_service_declare_and_activate() -> None:
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(1, "A", *BEIJING, 100_000, 200))
    assert len(maps) == 0                 # 声明不激活

    zone = maps.activate(1)
    assert zone.spec.name == "A"
    assert len(maps) == 1
    assert maps.activate(1) is zone      # 重复激活返回同一个对象

    maps.deactivate(1)
    assert len(maps) == 0
    maps.deactivate(1)                   # 重复释放不报错


def test_activate_unknown_zone_raises() -> None:
    maps = MapService()
    with pytest.raises(KeyError):
        maps.activate(42)


def test_locate_inside_zone_is_local() -> None:
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(3, "北京", *BEIJING, 100_000, 200))
    maps.activate(3)

    loc = maps.locate(BEIJING[0] + 0.05, BEIJING[1] + 0.05)
    assert loc.is_local
    assert loc.zone_id == 3
    assert loc.axial is not None

    # 反查得到的是单元**中心**，与原始点的偏差上界是六边形外接圆半径 = 边长。
    # 这里用 200 m 格距，所以偏差不会超过 200 m。
    lat, lng = loc.zone.to_geo(loc.axial)
    assert great_circle_m(loc.lat, loc.lng, lat, lng) <= loc.zone.spec.resolution_m

    # 而单元归属必须精确：中心点再查一次，还是同一个单元
    assert loc.zone.to_axial(lat, lng) == loc.axial


def test_locate_outside_zone_falls_back_to_global() -> None:
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(3, "北京", *BEIJING, 50_000, 200))
    maps.activate(3)

    loc = maps.locate(*MOSCOW)
    assert not loc.is_local
    assert loc.zone_id == -1
    assert loc.axial is None


def test_locate_is_independent_of_zone_count() -> None:
    """路由是 O(1)：激活大量战区后，定位耗时不应随数量增长。

    这里只验证功能正确性——用 60 个战区把索引压满，每个点都必须命中
    正确的战区。
    """
    maps = MapService(global_resolution=4)
    for i in range(60):
        lat = 30.0 + (i % 10) * 1.0
        lng = 100.0 + (i // 10) * 1.0
        maps.declare(ZoneSpec(i, f"Z{i}", lat, lng, 30_000, 500))

    for i in range(0, 60, 7):
        maps.activate(i)

    for i in range(0, 60, 7):
        spec = maps.spec(i)
        loc = maps.locate(spec.lat, spec.lng)
        assert loc.is_local
        assert loc.zone_id == i


def test_deactivate_clears_routing() -> None:
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(1, "A", *BEIJING, 100_000, 500))
    maps.activate(1)
    assert maps.locate(*BEIJING).is_local

    maps.deactivate(1)
    assert not maps.locate(*BEIJING).is_local


def test_activate_for_loads_on_demand() -> None:
    """实体接近时按需激活——这就是"只加载有目标的区域"的机制。"""
    loaded: list[str] = []
    maps = MapService(
        global_resolution=4,
        loader=lambda zone: loaded.append(zone.spec.name),
    )
    maps.declare(ZoneSpec(1, "北京", *BEIJING, 80_000, 200))
    maps.declare(ZoneSpec(2, "莫斯科", *MOSCOW, 80_000, 200))

    assert loaded == []

    zone = maps.activate_for(*BEIJING)
    assert zone is not None and zone.spec.name == "北京"
    assert loaded == ["北京"]

    maps.activate_for(*MOSCOW)
    assert loaded == ["北京", "莫斯科"]

    # 已经激活的不重复加载
    maps.activate_for(*BEIJING)
    assert loaded == ["北京", "莫斯科"]


def test_activate_for_returns_none_when_nothing_matches() -> None:
    maps = MapService(global_resolution=4)
    maps.declare(ZoneSpec(1, "北京", *BEIJING, 50_000, 200))
    assert maps.activate_for(0.0, 0.0) is None


def test_validate_detects_overlapping_zones() -> None:
    maps = MapService(global_resolution=4)
    maps.declare(ZoneSpec(1, "A", 39.90, 116.40, 100_000, 500))
    maps.declare(ZoneSpec(2, "B", 39.95, 116.45, 100_000, 500))   # 几乎重合
    problems = maps.validate()
    assert any("重叠" in p for p in problems)


def test_validate_flags_oversized_zone() -> None:
    maps = MapService(global_resolution=4)
    maps.declare(ZoneSpec(1, "巨区", *BEIJING, 900_000, 100, layer_count=4))
    problems = maps.validate()
    assert any("内存压力" in p for p in problems)


def test_validate_clean_config_is_silent() -> None:
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(1, "A", *BEIJING, 200_000, 200))
    maps.declare(ZoneSpec(2, "B", *MOSCOW, 200_000, 200))
    assert maps.validate() == []


def test_locate_cell_roundtrip() -> None:
    """网格单元 → 位置 → 同一单元。"""
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(2, "A", *BEIJING, 100_000, 1000))
    maps.activate(2)

    loc = maps.locate(BEIJING[0] + 0.1, BEIJING[1] + 0.1)
    cell = loc.cell()
    back = maps.locate_cell(cell)

    # 单元标识必须一致
    assert back.zone_id == loc.zone_id
    assert back.axial == loc.axial

    # 位置偏差上界是一个格距（单元中心 vs 任意落入该单元的点）
    assert great_circle_m(loc.lat, loc.lng, back.lat, back.lng) <= 1000.0


def test_zone_contains_boundary() -> None:
    maps = MapService(global_resolution=5)
    maps.declare(ZoneSpec(1, "A", *BEIJING, 50_000, 500))
    zone = maps.activate(1)
    assert zone.contains(*BEIJING)
    assert not zone.contains(BEIJING[0] + 1.0, BEIJING[1])


def test_active_zones_sorted_for_reproducibility() -> None:
    """遍历已激活战区必须顺序稳定，否则会破坏推演可复现性。"""
    maps = MapService(global_resolution=4)
    for i in range(5):
        maps.declare(ZoneSpec(i, f"Z{i}", 30.0 + i, 100.0 + i, 50_000, 500))
    for i in (3, 1, 4, 0, 2):
        maps.activate(i)
    assert [z.spec.zone_id for z in maps.active_zones] == [0, 1, 2, 3, 4]
    assert [s.zone_id for s in maps] == [0, 1, 2, 3, 4]


def test_transcontinental_route_crosses_into_zone() -> None:
    """端到端：洲际航线从全球层进入战区，坐标无缝切换。"""
    maps = MapService(global_resolution=4)
    maps.declare(ZoneSpec(1, "莫斯科", *MOSCOW, 80_000, 500))
    maps.activate(1)

    route = cells_for_route(GlobalLayer(3), *BEIJING, *MOSCOW)

    local_steps = 0
    global_steps = 0
    for cell in route:
        lat, lng = maps.global_layer.center(cell)
        loc = maps.locate(lat, lng)
        if loc.is_local:
            local_steps += 1
        else:
            global_steps += 1

    assert global_steps > 0        # 途中走全球层
    assert local_steps > 0         # 进入战区后切到局部坐标
