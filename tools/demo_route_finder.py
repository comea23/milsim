"""AFSIM `3d_route_finder` 的对照：同一组分层禁飞圆柱，谁选得短。

用法::

    python tools/demo_route_finder.py \
        --afsim demos/route_finder_demos/3d_route_finder.txt \
        --repro demos/afsim_mover/route_finder.txt \
        --ref   demos/afsim_mover/ref_3d_route_finder.csv \
        --route demos/afsim_mover/ref_3d_route_finder_route.csv \
        --out   demos/afsim_mover/repro_3d_route_finder.csv \
        --plot  docs/images/afsim_route_finder_compare.png \
        --seconds 1500

它做四件事：

1. **读 AFSIM 脚本**（原 demo），把这一问的**输入**解出来——SAM 在哪、目标在
   哪、`mRadiusPerAltitude` 是多少、层高从哪到哪、步长多少。这些都是原脚本
   的事实，**不在这里再抄一份**：改原脚本，这里的口径跟着变。
2. **跑 AFSIM 采两样东西**：飞出来的航迹（观察器逐帧）、以及**选中的那条
   航路本身**（脚本里 `shortestRoute` 的航路点）。后者要往脚本里注入一段
   `writeln`，`afsim_ref.py` 的命令行放不下多行注入，所以在这里调它的
   :func:`afsim_ref.capture`。
3. **在 milsim 里把同一件事重做一遍**：逐层建"禁飞圆柱"、调
   :meth:`NavService.route`，并把**无禁区的底数**也跑出来。
4. **对照**：两边各选了哪一层、航路多长、绕的哪一侧、离圆柱留了多少余量。

**这一份与前几份（kinematic_mover / aircraft_mover）比的东西不一样**：前几份
是"给定航线谁跟得住"，这一份是"**给定禁飞区谁选得短**"——不跑动力学，比的是
地图服务。所以复现想定 `demos/afsim_mover/route_finder.txt` 里没有
`component_type`（选路与机动件无关）。

**比航路长度时有一个必须说清的口径差**（这是这一份最容易被读错的地方）：

* AFSIM 的 route finder 输出的是**切线与圆弧拼成的折线**，航路点可以落在任意
  位置，所以它能贴到"直线 + 0.6%"；
* milsim 的 A* 走**格心**，路径长度天然是 `步数 × 格心距`，这一个方向本身就
  比直线长 2.6%（六边格在偏离格轴 30° 时最长 +15.5%）——**这是格网的底数，
  不是绕行**。

所以工具额外跑一条"**无禁区**"的 A*，把底数量出来。读了它才知道 milsim 那些
+7% 里有多少是格网、多少是绕行。

**还有一处是原 demo 自己的缺陷，必须点名**：`on_initialize2` 里的循环写着

    beg = path.Front().Location();   beg.SetAltitudeAGL(src.Altitude());
    end = path.Back().Location();    end.SetAltitudeAGL(tgt.Altitude());

`beg` / `end` 是**循环外的局部量**，下一层却拿上一层的 `path.Front()` 去覆盖它们
——于是 31 层下来端点被一路往里拖。实测：原样跑，选中的航路两端各缩进
**6.30 / 6.31 km**；把每层的 `beg`/`end` 从 `src`/`tgt` 重建后，缩进回到
**0.08 / 0.08 km**，而且选出来的航路点与"只跑最底一层"逐字相同。工具每次都会跑
这一组诊断并把两个数打出来，因为**"参照数据是怎么产生的"和参照数据一样重要**。
"""

from __future__ import annotations

import csv
import math
import re
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from milsim.services.map import Axial, TERRAIN_NAMES  # noqa: E402
from milsim.services.map.path import CostConfig  # noqa: E402
from milsim.simulation import Simulation  # noqa: E402

# 显示宽度补齐、数字格式化、航线解析：**与 demo_afsim_mover 共用同一份实现**。
# 两台工具比的是同一件事的两个侧面（跟航线 / 选航线），各自留一份就会各演化
# 一份口径。
from demo_afsim_mover import (  # noqa: E402
    _FT,
    _number,
    _offset_to_polyline,
    _pad,
    parse_afsim,
)

import afsim_ref  # noqa: E402

_MLAT = 111132.0

#: 往 AFSIM 脚本里注入的两个锚点。**注入的是纯诊断输出**（`writeln`），不改
#: 脚本的任何行为——它只是把 `src` / `tgt` / `shortestRoute` 打出来。
_INJECT_TGT = r"WsfGeoPoint tgt = WsfSimulation\.FindPlatform\(mTargetPlatform\)\.Location\(\);"
_INJECT_ROUTE = r"PLATFORM\.SetRoute\(shortestRoute\);"

#: 原 demo 的循环体开头。诊断运行在这里插一句"把 beg/end 从 src/tgt 重建"。
_INJECT_LOOP = r"\{\n\s*mFinder\.ClearAvoidances\(\);"

_DUMP_SRC_TGT = r"""
      writeln("KIN,SRC,,", src.Latitude() - 33, ",", src.Longitude() + 84, ",", src.Altitude());
      writeln("KIN,TGT,,", tgt.Latitude() - 33, ",", tgt.Longitude() + 84, ",", tgt.Altitude());
"""

_DUMP_ROUTE = r"""
   for (int k = 0; k < shortestRoute.Size(); k = k + 1)
   {
      WsfGeoPoint q = shortestRoute.Waypoint(k).Location();
      writeln("KIN,R", k, ",,", q.Latitude() - 33, ",", q.Longitude() + 84, ",", q.Altitude());
   }
"""

#: 诊断运行：每层把 `beg` / `end` 从 `src` / `tgt` 重建。见模块 docstring 末段。
_REBUILD = (
    r"\{\n\s*mFinder\.ClearAvoidances\(\);",
    "{\n         beg = WsfGeoPoint(src);\n         end = WsfGeoPoint(tgt);\n"
    "         mFinder.ClearAvoidances();",
)

#: 打印经纬度/航向用的锚点偏移（AFSIM `writeln` 的 `%g` 只有 6 位有效数字）。
#: 与 `afsim_ref.py` 一致：`观察器` 打的是相对锚点的偏移，落盘时加回去。
_LAT_ANCHOR = 33.0
_LNG_ANCHOR = -84.0


# ---------------------------------------------------------------------------
# AFSIM 脚本 → 这一问的输入
# ---------------------------------------------------------------------------


def _metres(expr: str) -> float:
    """``40000 * MATH.M_PER_FT()`` / ``1000`` → 米。"""
    match = re.search(r"([\d.]+)\s*\*\s*MATH\.M_PER_FT\(\)", expr)
    if match:
        return float(match.group(1)) * _FT
    return float(expr.strip())


class RouteDemo:
    """AFSIM `3d_route_finder` 这一问的输入。"""

    __slots__ = ("striker", "start", "target", "avoids", "radius_per_alt",
                 "ceiling", "floor", "increment", "speed", "max_arc", "response")

    def __init__(self) -> None:
        #: 执行选路的平台名。
        self.striker = ""
        #: ``(lat, lng, alt_m)``，起点。**取自脚本自己的 route 首点**——AFSIM 里
        #: 平台没有单写 `position` 时，初始位置就是航路首点。
        self.start: tuple[float, float, float] = (0.0, 0.0, 0.0)
        #: ``(lat, lng, alt_m)``，目标。
        self.target: tuple[float, float, float] = (0.0, 0.0, 0.0)
        #: ``[(名字, lat, lng), ...]`` 被规避的 SAM。
        self.avoids: list[tuple[str, float, float]] = []
        #: 圆柱半径 = 层高(米) × 这个系数。
        self.radius_per_alt = 0.0
        self.ceiling = 0.0
        self.floor = 0.0
        self.increment = 0.0
        #: 交给 route finder 的航路速度（它只用来定转弯弧）。
        self.speed = 0.0
        self.max_arc: float | None = None
        self.response = ""

    @property
    def ladder(self) -> list[float]:
        """从顶到底的层高（米）。原脚本是 ``alt -= altIncrement`` 的降序循环。"""
        out: list[float] = []
        alt = self.ceiling
        while alt >= self.floor - 1e-6:
            out.append(alt)
            alt -= self.increment
        return out


def _platform_meta(text: str) -> dict[str, tuple[str, bool]]:
    """``{平台名: (型号, 挂了机动件吗)}``。型号用来认 SAM，机动件用来认执行者。"""
    out: dict[str, tuple[str, bool]] = {}
    for match in re.finditer(r"(?mi)^platform\s+(\S+)\s+(\S+)\s*$", text):
        stop = re.search(r"(?mi)^end_platform\s*$", text[match.end():])
        body = text[match.end(): match.end() + stop.start()] if stop else ""
        out[match.group(1)] = (match.group(2), "add mover" in body)
    return out


def parse_route_demo(path: Path) -> RouteDemo:
    """解出 AFSIM 脚本里这一问的全部输入。"""
    text = path.read_text(encoding="utf-8", errors="replace")
    demo = RouteDemo()

    def grab(pattern: str, group: int = 1) -> str:
        match = re.search(pattern, text)
        if match is None:
            raise SystemExit(f"{path.name} 里找不到 `{pattern}`——这份脚本是这一问吗？")
        return match.group(group)

    target_name = grab(r'mTargetPlatform\s*=\s*"([^"]+)"')
    avoid_type = grab(r'mAvoidPlatformType\s*=\s*"([^"]+)"')
    # **`^` 不能省**：原脚本里那条真参数上面压着一行注释掉的旧值
    # （`#double mRadiusPerAltitude = 2.5;`），不锚行首就会把注释读成参数，
    # 半径系数悄悄变成 2.5——而 2.5 也是"一个合理的数"，图和数据都不会报错。
    demo.radius_per_alt = float(grab(
        r"(?mi)^\s*(?:double\s+)?mRadiusPerAltitude\s*=\s*([\d.]+)\s*;"))
    demo.ceiling = _metres(grab(r"(?mi)^\s*double\s+mAltCeiling\s*=\s*([^;]+);"))
    demo.floor = _metres(grab(r"(?mi)^\s*double\s+mAltFloor\s*=\s*([^;]+);"))
    demo.increment = _metres(grab(r"(?mi)^\s*double\s+altIncrement\s*=\s*([^;]+);"))
    demo.speed = float(grab(r"mFinder\.Route\([^;]*?,\s*([\d.]+)\s*\)"))
    arc = re.search(r"(?mi)^\s*\S*SetMaxArcLength\(([\d.]+)\)", text)
    demo.max_arc = float(arc.group(1)) if arc else None
    demo.response = grab(r'SetImpossibleRouteResponse\("(\w+)"\)')

    platforms = parse_afsim(path)
    meta = _platform_meta(text)
    for name, (kind, has_mover) in meta.items():
        if kind != avoid_type or name not in platforms:
            continue
        plat = platforms[name]
        demo.avoids.append((name, plat.lat, plat.lng))
    demo.avoids.sort()

    target = platforms.get(target_name)
    if target is None:
        raise SystemExit(f"找不到目标平台 {target_name}")
    demo.target = (target.lat, target.lng, target.alt)

    for name, (_kind, has_mover) in meta.items():
        if not has_mover or name not in platforms:
            continue
        if meta[name][0] == avoid_type:
            continue
        route = platforms[name].route
        if not route:
            continue
        demo.striker = name
        demo.start = (route[0][0], route[0][1], route[0][2])
        break
    if not demo.striker:
        raise SystemExit("认不出哪个平台在选路（没有带 `add mover` 且带 route 的平台）")
    return demo


# ---------------------------------------------------------------------------
# AFSIM 侧：跑两次，一次采航迹，一次把选中的航路 dump 出来
# ---------------------------------------------------------------------------


def _route_injections(extra: tuple[tuple[str, str], ...] = ()) -> tuple[tuple[str, str], ...]:
    return (
        (_INJECT_TGT, "WsfGeoPoint tgt = WsfSimulation.FindPlatform(mTargetPlatform)"
                      ".Location();" + _DUMP_SRC_TGT),
        (_INJECT_ROUTE, "PLATFORM.SetRoute(shortestRoute);" + _DUMP_ROUTE),
    ) + extra


def read_route_dump(path: Path):
    """读 dump 出来的 ``SRC`` / ``TGT`` / ``R<k>`` 行 → ``(src, tgt, [(lat,lng,alt)])``。

    这三类行都塞在**同一个 CSV 的 time_s 列**里（观察器打的是
    ``KIN,<名字>,...``），所以按名字认，而不是按列位置认。
    """
    src = tgt = None
    route: list[tuple[int, float, float, float]] = []
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        parts = line.split(",")
        if len(parts) < 5 or not parts[2]:
            continue
        try:
            lat, lng, alt = float(parts[2]), float(parts[3]), float(parts[4])
        except ValueError:
            continue
        if parts[0] == "SRC":
            src = (lat, lng, alt)
        elif parts[0] == "TGT":
            tgt = (lat, lng, alt)
        elif parts[0].startswith("R") and parts[0][1:].isdigit():
            route.append((int(parts[0][1:]), lat, lng, alt))
    route.sort()
    return src, tgt, [(lat, lng, alt) for _k, lat, lng, alt in route]


def capture_reference(script: Path, flown: Path, route_csv: Path, seconds: float) -> tuple[tuple, tuple]:
    """跑 AFSIM：采航迹、dump 选中航路，并做一次"重建 beg/end"的诊断运行。

    三次运行都不改 ``demos/`` 下的原文件——改动只存在于喂给 AFSIM 的那份临时文本。
    """
    print(f"[参照] AFSIM {script.name}")
    afsim_ref.capture(script, flown, seconds, set(), None)
    afsim_ref.capture(script, route_csv, 5.0, set(), None, _route_injections())

    with tempfile.TemporaryDirectory(prefix="milsim_rf_probe_") as work:
        fixed = Path(work) / "fixed.csv"
        afsim_ref.capture(script, fixed, 5.0, set(), None,
                          _route_injections((_REBUILD,)))
        fixed_src, fixed_tgt, fixed_route = read_route_dump(fixed)

    src, tgt, route = read_route_dump(route_csv)
    return (src, tgt, route), (fixed_src, fixed_tgt, fixed_route)


# ---------------------------------------------------------------------------
# milsim 侧：逐层建禁区、选路
# ---------------------------------------------------------------------------


class Layer:
    """一层高度上的选路结果。"""

    __slots__ = ("alt_m", "radius_m", "blocked", "steps", "length_m", "expanded",
                 "degraded", "polyline", "clearance_m", "jumps", "hierarchy")

    def __init__(self, alt_m: float, radius_m: float) -> None:
        self.alt_m = alt_m
        self.radius_m = radius_m
        self.blocked = 0
        self.steps = 0
        self.length_m = 0.0
        self.expanded = 0
        self.degraded = False
        #: 航路的格心折线（战区平面坐标，米）。
        self.polyline: list[tuple[float, float]] = []
        #: 到每个 SAM 的最近距离（米）。
        self.clearance_m: list[float] = []
        #: 连续两格**不相邻**的次数。相邻步长恒为格心距，跳步会把折线拉直，
        #: 于是折线斜切进圆柱里——"余量"为负就是这么来的。
        self.jumps = 0
        self.hierarchy = False


def _air_profile() -> CostConfig:
    """空中画像：**地形无关、等代价**。

    ``costs={}`` 不是"全通行"——`cost_at` 里是
    `self.costs.get(terrain, IMPASSABLE)`，表里没有的地貌一律不可通行，于是
    整张图只有 { } 那么大。必须显式给每种地貌一个代价。

    `TRAVEL_PROFILES` 里没有"空中"（只有 ground / water / 两栖），因为空中件
    本来就不受地形约束——`mover.travel_config()` 对空中件返回 ``None``。
    """
    return CostConfig(costs={code: 1.0 for code in TERRAIN_NAMES},
                      slope_weight=0.0, min_cost=1.0)


def _cylinder(nav, ref, grid, cx: float, cy: float, radius: float) -> set:
    """以 ``(cx, cy)`` 为心、``radius`` 为半径的**圆柱**落在哪些格里。

    判据是**格心落在圆内**。另一半选择是"格只要与圆相交就算"，那会把边上的格
    多封一圈（最多 0.87 km，半个格心距）。两者是包围关系，所以工具把选中航路
    的**实际余量**也打出来：余量大于半个格心距时，两种判据给出同一条航路。

    **`reach` 必须按内切半径算，不能按格心距算。** 枚举的范围是"六边距离 ≤
    reach"的球，那是个**六边形**：外接半径（六个顶点方向）= `reach · 格心距`，
    内切半径（六条边中点方向）= `reach · 格心距 · √3/2` = `reach · 1.5 · 格边长`
    ——**差 13.4%**。按格心距定 `reach` 时，圆会从六边形的六条边那里漏出去一圈，
    航路正好从缺口钻进来。这不是理论担忧，是实测的：

    | 层 | 半径 | 旧 reach 覆盖到 | 航路实测最近 |
    |---|---|---|---|
    | 12000 ft | 25.56 km | 22.14 km | 25.24 km（余量 −0.37） |
    | 40000 ft | 85.34 km | 76.50 km | 77.92 km（余量 −7.42） |

    两个都落在"内切半径 ~ 半径"那一段里，所以症状是**"余量为负"而不是"航路穿心"**。
    """
    center = nav.axial_at(ref, cx, cy)
    if center is None:
        return set()
    span = grid.size * math.sqrt(3.0)
    reach = int(radius / (span * math.sqrt(3.0) / 2.0)) + 2
    out = set()
    for dq in range(-reach, reach + 1):
        for dr in range(max(-reach, -dq - reach), min(reach, -dq + reach) + 1):
            cell = Axial(center.q + dq, center.r + dr)
            point = nav.cell_center(ref, cell)
            if point is None:
                continue
            if math.hypot(point[0] - cx, point[1] - cy) <= radius:
                out.add(cell)
    return out


def select_routes(demo: RouteDemo, repro: Path) -> dict:
    """在 milsim 里把这一问重做一遍。返回一个装满中间量的字典。"""
    sim = Simulation.from_scenario_file(repro)
    sim.build()
    sim.initialize()

    entity = sim.registry.by_name(demo.striker)
    if entity is None:
        raise SystemExit(f"复现想定里没有平台 {demo.striker}")
    ref = sim.store.cell_of(entity.entity_id)
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    frame = sim.maps.zone(ref.zone_id).frame
    profile = _air_profile()

    sx, sy = frame.geo_to_world(demo.start[0], demo.start[1])
    tx, ty = frame.geo_to_world(demo.target[0], demo.target[1])
    start = sim.nav.axial_at(ref, sx, sy)
    goal = sim.nav.axial_at(ref, tx, ty)
    straight = math.hypot(tx - sx, ty - sy)

    centers: dict = {}

    def centre(cell):
        if cell not in centers:
            centers[cell] = sim.nav.cell_center(ref, cell)
        return centers[cell]

    sams = []
    for name, lat, lng in demo.avoids:
        cx, cy = frame.geo_to_world(lat, lng)
        sams.append((name, cx, cy))

    def walk(cells):
        points = [centre(c) for c in cells]
        length = 0.0
        for a, b in zip(points, points[1:]):
            if a is None or b is None:
                continue
            length += math.hypot(b[0] - a[0], b[1] - a[1])
        return [p for p in points if p is not None], length

    def clearance(polyline):
        if len(polyline) < 2:
            return [math.inf] * len(sams)
        return [_gap(cx, cy, polyline) for _n, cx, cy in sams]

    def settle(layer: Layer, result) -> None:
        layer.polyline, layer.length_m = walk(result.cells)
        layer.steps = max(0, len(result.cells) - 1)
        layer.expanded = result.expanded
        layer.degraded = result.degraded
        layer.hierarchy = result.used_hierarchy
        layer.clearance_m = clearance(layer.polyline)
        # 相邻步是格心距；不是相邻就是跳步。跳步会把折线拉直，斜切进圆柱。
        layer.jumps = sum(
            1 for a, b in zip(result.cells, result.cells[1:])
            if max(abs(b.q - a.q), abs(b.r - a.r), abs(b.q + b.r - a.q - a.r)) > 1
        )

    free = Layer(0.0, 0.0)
    result = sim.nav.route(ref, goal, profile=profile, forbidden=set())
    if result is not None:
        settle(free, result)

    layers: list[Layer] = []
    for alt_m in demo.ladder:
        radius = alt_m * demo.radius_per_alt
        layer = Layer(alt_m, radius)
        blocked: set = set()
        for _name, cx, cy in sams:
            blocked |= _cylinder(sim.nav, ref, grid, cx, cy, radius)
        layer.blocked = len(blocked)
        result = sim.nav.route(ref, goal, profile=profile, forbidden=blocked)
        if result is not None:
            settle(layer, result)
        else:
            layer.clearance_m = [math.inf] * len(sams)
        layers.append(layer)

    return {
        "sim": sim, "ref": ref, "grid": grid, "frame": frame,
        "start": start, "goal": goal, "straight": straight,
        "sams": sams, "free": free, "layers": layers,
    }


# ---------------------------------------------------------------------------
# 对照
# ---------------------------------------------------------------------------


def _flat(points, frame):
    return [frame.geo_to_world(lat, lng) for lat, lng, _alt in points]


def _world_route(points, frame) -> list[tuple[float, float, float]]:
    """``[(lat, lng, alt)]`` → ``[(x, y, alt)]``，战区平面坐标。"""
    out = []
    for lat, lng, alt in points:
        x, y = frame.geo_to_world(lat, lng)
        out.append((x, y, alt))
    return out


def _gap(cx: float, cy: float, polyline) -> float:
    """点 ``(cx, cy)`` 到折线的最近距离（米）。

    复用 `demo_afsim_mover._offset_to_polyline`——它按航迹的
    ``(t, x, y, ...)`` 六元组取点，所以这里补一个占位的首列。
    """
    if len(polyline) < 2:
        return math.inf
    return _offset_to_polyline((0.0, cx, cy), polyline)


def _lengths(polyline) -> tuple[float, float]:
    """``(水平长度, 三维长度)``。三维按相邻点的高差拼——一条纯平面折线的高差为 0。"""
    horizontal = 0.0
    three_d = 0.0
    for a, b in zip(polyline, polyline[1:]):
        horizontal += math.hypot(b[0] - a[0], b[1] - a[1])
        three_d += math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)
    return horizontal, three_d


def _profile(polyline, origin, axis, normal, step: float = 2000.0):
    """折线的 ``(沿线 m, 横向 m)`` 剖面（按固定弧长步长重采样）。"""
    out = []
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        span = math.hypot(bx - ax, by - ay)
        if span <= 0.0:
            continue
        pieces = max(1, int(span / step))
        for index in range(1, pieces + 1):
            ratio = index / pieces
            x = ax + (bx - ax) * ratio
            y = ay + (by - ay) * ratio
            dx, dy = x - origin[0], y - origin[1]
            out.append((dx * axis[0] + dy * axis[1], dx * normal[0] + dy * normal[1]))
    return out


def _axis_frame(data: dict, demo: RouteDemo):
    """``(原点, 单位切向, 单位法向)``——沿线/横向的度量基准。

    法向是"切向逆时针转 90°"，所以**横向为正 = 左 = 南**（切向大致朝西）。
    """
    frame = data["frame"]
    sx, sy = frame.geo_to_world(demo.start[0], demo.start[1])
    tx, ty = frame.geo_to_world(demo.target[0], demo.target[1])
    straight = data["straight"]
    axis = ((tx - sx) / straight, (ty - sy) / straight)
    return (sx, sy), axis, (-axis[1], axis[0])


def _excursion(polyline, origin, axis, normal) -> tuple[float, float]:
    """折线的横向偏移范围 ``(最小, 最大)``，米。"""
    values = [cross for _along, cross in _profile(polyline, origin, axis, normal)]
    return (min(values), max(values)) if values else (0.0, 0.0)


def report(demo: RouteDemo, data: dict, afsim, straight: float) -> None:
    grid = data["grid"]
    span = grid.size * math.sqrt(3.0)
    free = data["free"]
    layers = data["layers"]

    print()
    print(f"格 {grid.size:.0f} m，格心距 {span:.1f} m；"
          f"起终点直线 {straight / 1000:.2f} km")
    print(f"AFSIM 的层梯：{demo.ceiling / _FT:.0f} ft → {demo.floor / _FT:.0f} ft，"
          f"步长 {demo.increment / _FT:.0f} ft，共 {len(demo.ladder)} 层；"
          f"半径 = 层高 × {demo.radius_per_alt:g}")
    print(f"规避 {len(demo.avoids)} 个 SAM："
          + "、".join(name for name, _lat, _lng in demo.avoids))

    widths = (9, 10, 11, 10, 10, 11, 9, 9, 9, 9)
    print()
    print(_pad("层 ft", widths[0], right=True)
          + _pad("层 m", widths[1], right=True)
          + _pad("半径 km", widths[2], right=True)
          + _pad("禁区格", widths[3], right=True)
          + _pad("航路格", widths[4], right=True)
          + _pad("水平 km", widths[5], right=True)
          + _pad("比直线", widths[6], right=True)
          + _pad("比底数", widths[7], right=True)
          + _pad("余量 km", widths[8], right=True)
          + _pad("展开", widths[9], right=True))
    for layer in layers:
        gap = (min(layer.clearance_m) - layer.radius_m) / 1000.0
        print(_pad(f"{layer.alt_m / _FT:.0f}", widths[0], right=True)
              + _pad(f"{layer.alt_m:.0f}", widths[1], right=True)
              + _pad(f"{layer.radius_m / 1000:.1f}", widths[2], right=True)
              + _pad(f"{layer.blocked}", widths[3], right=True)
              + _pad(f"{layer.steps}", widths[4], right=True)
              + _pad(f"{layer.length_m / 1000:.2f}", widths[5], right=True)
              + _pad(f"{layer.length_m / straight:.3f}", widths[6], right=True)
              + _pad(f"{layer.length_m / free.length_m:.3f}" if free.length_m else "—",
                     widths[7], right=True)
              + _pad(_number(gap, 2), widths[8], right=True)
              + _pad(f"{layer.expanded}{' ＊' if layer.degraded else ''}",
                     widths[9], right=True))
    print(_pad("底数", widths[0], right=True)
          + _pad("—", widths[1], right=True)
          + _pad("—", widths[2], right=True)
          + _pad("0", widths[3], right=True)
          + _pad(f"{free.steps}", widths[4], right=True)
          + _pad(f"{free.length_m / 1000:.2f}", widths[5], right=True)
          + _pad(f"{free.length_m / straight:.3f}", widths[6], right=True)
          + _pad("1.000", widths[7], right=True)
          + _pad("—", widths[8], right=True)
          + _pad(f"{free.expanded}", widths[9], right=True))

    best = min(layers, key=lambda item: item.length_m)
    print()
    print("＊ = 这一层的搜索**降级**了（禁区把战区切碎，`degraded` 置位）——"
          "这一行的航程只能当参考，别当上界。")
    print(f"“比底数” = 该层航程 ÷ 无禁区那一趟的航程，**把格网的各向异性除掉了**；"
          f"底数本身比直线长 {free.length_m / straight - 1:.1%}。")
    print(f"milsim 选中的是 **{best.alt_m / _FT:.0f} ft** 那一层"
          f"（半径 {best.radius_m / 1000:.1f} km，水平 {best.length_m / 1000:.2f} km）。")
    print("  " + "；".join(
        f"{name} {value / 1000:.1f} km"
        for (name, _cx, _cy), value in zip(data["sams"], best.clearance_m)
    ) + "（到各 SAM 的最近距离）")

    if any(layer.jumps for layer in layers) or free.jumps:
        worst = max(layer.jumps for layer in layers + [free])
        print(f"⚠ 有层的航路含**跨格跳步**（最多 {worst} 次）——折线在跳步处被拉直，"
              "会斜切进圆柱，所以那几层的“余量”可能为负。")

    src, tgt, route, fixed_src, fixed_tgt, fixed_route = afsim
    #: **拿"重建 beg/end"那一趟做对照**。原样那趟的端点被 demo 的循环拖了
    #: 6.3 km，航路因此比"起终点直线"还短（0.970）——用它比长度是错的。
    #: 两趟选中的航路点逐点相同（差 <100 m），所以这不是"换了个答案"。
    picked = fixed_route or route
    if picked:
        world = _world_route([(lat, lng, alt) for lat, lng, alt in picked], data["frame"])
        horizontal, three_d = _lengths(world)
        chosen_alt = min(alt for _lat, _lng, alt in picked)
        print()
        print(f"AFSIM 选中的航路：{len(picked)} 个航路点，"
              f"水平 {horizontal / 1000:.2f} km（比直线 {horizontal / straight:.3f}），"
              f"三维 {three_d / 1000:.2f} km；最低层 {chosen_alt / _FT:.0f} ft。")
        for index, (lat, lng, alt) in enumerate(picked):
            print(f"    R{index}  {lat:9.5f} {lng:9.5f}  {alt:8.1f} m"
                  f"  ({alt / _FT:6.0f} ft)")
        gaps = [_gap(cx, cy, [(x, y) for x, y, _z in world])
                for _n, cx, cy in data["sams"]]
        print("    到各 SAM 最近：" + "  ".join(
            f"{name} {gap / 1000:.1f} km"
            for (name, _cx, _cy), gap in zip(data["sams"], gaps)))
        radius = chosen_alt * demo.radius_per_alt
        print(f"    （该层半径 {radius / 1000:.1f} km，最近余量 "
              f"{(min(gaps) - radius) / 1000:+.2f} km）")

    origin, axis, normal = _axis_frame(data, demo)
    print()
    print("绕行幅度（横向偏移 min→max km，正 = 左 = 南）：")
    if picked:
        low, high = _excursion(_flat(picked, data["frame"]), origin, axis, normal)
        print(f"    AFSIM   {low / 1000:+7.2f} → {high / 1000:+7.2f}")
    low, high = _excursion(best.polyline, origin, axis, normal)
    print(f"    milsim  {low / 1000:+7.2f} → {high / 1000:+7.2f}"
          f"（选中的 {best.alt_m / _FT:.0f} ft 层）")
    for name, cx, cy in data["sams"]:
        gap = _gap(cx, cy, best.polyline)
        along = (cx - origin[0]) * axis[0] + (cy - origin[1]) * axis[1]
        cross = (cx - origin[0]) * normal[0] + (cy - origin[1]) * normal[1]
        print(f"      {name} 沿线 {along / 1000:7.1f} km 横向 {cross / 1000:+7.2f} km，"
              f"该层要求 |横向 − {cross / 1000:+.2f}| ≥ {best.radius_m / 1000:.1f} km"
              f"（复现实际留 {gap / 1000:.1f} km）")

    if src and tgt and route and fixed_route:
        print()
        print("原 demo 的循环把上一层的 path.Front()/Back() 写回 beg/end，"
              "端点被逐层往里拖：")
        for label, begin, points in (("原样", src, route),
                                     ("重建 beg/end", fixed_src, fixed_route)):
            if begin is None:
                continue
            head = (points[0][0], points[0][1])
            tail = (points[-1][0], points[-1][1])
            print(f"    {label:12}起点缩进 {_geo(begin, head) / 1000:6.3f} km  "
                  f"终点缩进 {_geo(tgt, tail) / 1000:6.3f} km  "
                  f"航路点 {len(points)} 个")


def _geo(a, b) -> float:
    dy = (b[0] - a[0]) * _MLAT
    dx = (b[1] - a[1]) * _MLAT * math.cos(math.radians(0.5 * (a[0] + b[0])))
    return math.hypot(dx, dy)


def write_csv(demo: RouteDemo, data: dict, out: Path, afsim) -> None:
    names = [name for name, _cx, _cy in data["sams"]]
    header = (["variant", "alt_ft", "alt_m", "radius_km", "blocked_cells",
               "route_cells", "route_km", "ratio_straight", "ratio_floor",
               "expanded", "degraded"]
              + [f"clear_{name}_km" for name in names] + ["min_margin_km"])
    rows = []
    free = data["free"]
    rows.append(["free", "0", "0", "0", "0", f"{free.steps}",
                 f"{free.length_m / 1000:.3f}", f"{free.length_m / data['straight']:.4f}",
                 "1.0000", f"{free.expanded}", "0"]
                + ["" for _ in names] + [""])
    for layer in data["layers"]:
        margin = min(layer.clearance_m) - layer.radius_m
        rows.append([
            "milsim", f"{layer.alt_m / _FT:.0f}", f"{layer.alt_m:.1f}",
            f"{layer.radius_m / 1000:.3f}", f"{layer.blocked}", f"{layer.steps}",
            f"{layer.length_m / 1000:.3f}", f"{layer.length_m / data['straight']:.4f}",
            f"{layer.length_m / free.length_m:.4f}" if free.length_m else "",
            f"{layer.expanded}", "1" if layer.degraded else "0",
        ] + [f"{value / 1000:.3f}" for value in layer.clearance_m]
          + [f"{margin / 1000:.3f}"])
    src, tgt, route, fixed_src, fixed_tgt, fixed_route = afsim
    picked = fixed_route or route
    if picked:
        world = _world_route(picked, data["frame"])
        horizontal, _three_d = _lengths(world)
        chosen = min(alt for _lat, _lng, alt in picked)
        radius = chosen * demo.radius_per_alt
        gaps = [_gap(cx, cy, [(x, y) for x, y, _z in world])
                for _n, cx, cy in data["sams"]]
        rows.append(["afsim", f"{chosen / _FT:.0f}", f"{chosen:.1f}",
                     f"{radius / 1000:.3f}", "", f"{len(picked) - 1}",
                     f"{horizontal / 1000:.3f}", f"{horizontal / data['straight']:.4f}",
                     "", "", ""]
                    + [f"{value / 1000:.3f}" for value in gaps]
                    + [f"{(min(gaps) - radius) / 1000:.3f}"])
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    print(f"  · 对照表 {len(rows)} 行 → {out}")


def draw(demo: RouteDemo, data: dict, afsim, out: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    colors = {"afsim": "#c0392b", "milsim": "#2c6fbb", "free": "#7f8c8d",
              "zone": "#d9822b"}
    frame = data["frame"]
    layers = data["layers"]
    best = min(layers, key=lambda item: item.length_m)
    src, tgt, route, fixed_src, fixed_tgt, fixed_route = afsim
    picked = fixed_route or route

    sx, sy = frame.geo_to_world(demo.start[0], demo.start[1])
    tx, ty = frame.geo_to_world(demo.target[0], demo.target[1])
    straight = data["straight"]
    axis = ((tx - sx) / straight, (ty - sy) / straight)
    normal = (-axis[1], axis[0])

    fig = plt.figure(figsize=(19.0, 5.4), dpi=110)

    # --- 左：地图 ---------------------------------------------------------
    ax1 = fig.add_subplot(1, 3, 1)
    for name, cx, cy in data["sams"]:
        for radius, alpha in ((best.radius_m, 0.13),):
            circle = plt.Circle((cx, cy), radius, color=colors["zone"],
                                alpha=alpha, zorder=0)
            ax1.add_patch(circle)
        ax1.plot([cx], [cy], "x", color=colors["zone"], ms=7, mew=2, zorder=3)
        ax1.annotate(name, (cx, cy), textcoords="offset points", xytext=(6, 6),
                     fontsize=7, color=colors["zone"])
    ax1.plot([sx, tx], [sy, ty], ":", color="#9aa0a6", lw=1.0, label="起终点直线", zorder=1)
    if len(data["free"].polyline) > 1:
        ax1.plot([p[0] for p in data["free"].polyline],
                 [p[1] for p in data["free"].polyline],
                 "--", color=colors["free"], lw=1.0, alpha=0.8,
                 label=f"无禁区底数 {data['free'].length_m / 1000:.0f} km", zorder=2)
    if picked:
        points = _flat(picked, frame)
        ax1.plot([p[0] for p in points], [p[1] for p in points], "-o",
                 color=colors["afsim"], lw=2.0, ms=3.5,
                 label=f"AFSIM 选中航路（{len(picked)} 点）", zorder=4)
    if len(best.polyline) > 1:
        ax1.plot([p[0] for p in best.polyline], [p[1] for p in best.polyline],
                 "-", color=colors["milsim"], lw=1.4,
                 label=f"milsim A*（{best.steps} 格）", zorder=4)
    ax1.plot([sx], [sy], "o", color="#1f7a6b", ms=6, zorder=5)
    ax1.annotate("起点", (sx, sy), textcoords="offset points", xytext=(6, -12), fontsize=7)
    ax1.plot([tx], [ty], "o", color="#5b2c8d", ms=6, zorder=5)
    ax1.annotate("目标", (tx, ty), textcoords="offset points", xytext=(6, -12), fontsize=7)
    ax1.set_aspect("equal")
    ax1.set_xlabel("东向 m（相对战区锚点）", fontsize=9)
    ax1.set_ylabel("北向 m", fontsize=9)
    ax1.set_title(f"{title}\n禁飞圆柱按选中的 {best.alt_m / _FT:.0f} ft 层"
                  f"（半径 {best.radius_m / 1000:.1f} km）画", fontsize=10)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=7, loc="best")
    ax1.tick_params(labelsize=8)

    # --- 中：横向偏移 -----------------------------------------------------
    ax2 = fig.add_subplot(1, 3, 2)
    profiles = {}
    if picked:
        profiles["afsim"] = _profile(_flat(picked, frame), (sx, sy), axis, normal)
    if len(best.polyline) > 1:
        profiles["milsim"] = _profile(best.polyline, (sx, sy), axis, normal)
    for key, label, style in (("afsim", "AFSIM 选中航路", "-"), ("milsim", "milsim A*", "-")):
        if key not in profiles:
            continue
        ax2.plot([p[0] / 1000 for p in profiles[key]],
                 [p[1] / 1000 for p in profiles[key]],
                 style, color=colors[key], lw=2.0 if key == "afsim" else 1.6, label=label)
    # **纵轴按两条航线的实际绕行幅度定**，不按 SAM 散布定：远处那几个 SAM 会把
    # 纵轴撑到 ±125 km，把 ±10 km 的绕行压成一条直线，那这格就白画了。
    # 每个 SAM 画成一条**竖带**（它在横向上要求的禁区宽度），超出视界的部分自然截断。
    spans = [p[1] for points in profiles.values() for p in points]
    low, high = (min(spans), max(spans)) if spans else (0.0, 0.0)
    pad = max(2000.0, 0.35 * (high - low))
    ax2.set_ylim((low - pad) / 1000, (high + pad) / 1000)
    for name, cx, cy in data["sams"]:
        along = (cx - sx) * axis[0] + (cy - sy) * axis[1]
        cross = (cx - sx) * normal[0] + (cy - sy) * normal[1]
        ax2.plot([along / 1000, along / 1000],
                 [(cross - best.radius_m) / 1000, (cross + best.radius_m) / 1000],
                 "-", color=colors["zone"], lw=6, alpha=0.25, zorder=0)
        ax2.plot([along / 1000], [cross / 1000], "x", color=colors["zone"], ms=8, mew=2)
        if ax2.get_ylim()[0] <= (cross + best.radius_m) / 1000 <= ax2.get_ylim()[1]:
            ax2.annotate(f"{name} r={best.radius_m / 1000:.0f}",
                         (along / 1000, (cross + best.radius_m) / 1000),
                         textcoords="offset points", xytext=(4, 4), fontsize=7,
                         color=colors["zone"])
    ax2.axhline(0.0, color="#9aa0a6", lw=0.8, ls=":")
    ax2.set_xlabel("沿线位置 km", fontsize=9)
    ax2.set_ylabel("横向偏移 km（正 = 左/南）", fontsize=9)
    ax2.set_title("两边各自绕到哪一侧：粗竖带 = SAM 的禁区宽度\n"
                  "（同侧、且都在带外 = 选路结论一致）", fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=7, loc="best")
    ax2.tick_params(labelsize=8)

    # --- 右：层梯 ---------------------------------------------------------
    ax3 = fig.add_subplot(1, 3, 3)
    alts = [layer.alt_m for layer in layers]
    ax3.plot([layer.length_m / 1000 for layer in layers], alts, "-o",
             color=colors["milsim"], lw=1.8, ms=4, label="milsim A* 水平航程")
    ax3.axvline(data["free"].length_m / 1000, color=colors["free"], ls="--", lw=1.2,
                label=f"无禁区底数 {data['free'].length_m / 1000:.0f} km")
    ax3.axvline(straight / 1000, color="#9aa0a6", ls=":", lw=1.2,
                label=f"起终点直线 {straight / 1000:.0f} km")
    if picked:
        horizontal, _three = _lengths(_world_route(picked, frame))
        chosen = min(alt for _lat, _lng, alt in picked)
        ax3.plot([horizontal / 1000], [chosen], "*", color=colors["afsim"], ms=15,
                 label=f"AFSIM 选中（{chosen / _FT:.0f} ft）")
    ax3.axhline(best.alt_m, color=colors["milsim"], lw=0.7, ls=":", alpha=0.6)
    ax3.set_ylabel("层高 m", fontsize=9)
    ax3.set_xlabel("水平航程 km", fontsize=9)
    ax3.set_title("逐层航程：层越低半径越小、越好绕\n"
                  "（左边那条虚线是格网的底数，不是绕行）", fontsize=10)
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=7, loc="best")
    ax3.tick_params(labelsize=8)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"  · 图已写入 {out}")


# ---------------------------------------------------------------------------


def main(argv: list[str]) -> int:
    options = {
        "--afsim": None, "--repro": None, "--ref": None, "--route": None,
        "--out": None, "--plot": None, "--seconds": 1500.0,
        "--title": "AFSIM 3d_route_finder 复现",
    }
    index = 1
    while index < len(argv):
        item = argv[index]
        if item not in options or index + 1 >= len(argv):
            print(f"无法识别的参数：{item}", file=sys.stderr)
            print(__doc__)
            return 2
        options[item] = float(argv[index + 1]) if item == "--seconds" else argv[index + 1]
        index += 2
    for key in ("--afsim", "--repro", "--ref", "--route", "--out", "--plot"):
        if options[key] is None:
            print(f"缺少参数 {key}", file=sys.stderr)
            return 2

    script = Path(str(options["--afsim"]))
    demo = parse_route_demo(script)
    print(f"[输入] {script.name}：{demo.striker} 从 "
          f"({demo.start[0]:.5f}, {demo.start[1]:.5f}) 到 "
          f"({demo.target[0]:.5f}, {demo.target[1]:.5f})，"
          f"规避 {len(demo.avoids)} 个 {demo.response} 响应的 SAM")

    as_is, rebuilt = capture_reference(
        script, Path(str(options["--ref"])), Path(str(options["--route"])),
        float(options["--seconds"]))
    #: ``(src, tgt, route, fixed_src, fixed_tgt, fixed_route)``——后半截是"把
    #: `beg`/`end` 每层重建"那次诊断运行的结果，与前半截同构。
    afsim = as_is + rebuilt

    print(f"[复现] {Path(str(options['--repro'])).name}")
    data = select_routes(demo, Path(str(options["--repro"])))
    report(demo, data, afsim, data["straight"])
    write_csv(demo, data, Path(str(options["--out"])), afsim)
    draw(demo, data, afsim, Path(str(options["--plot"])), str(options["--title"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
