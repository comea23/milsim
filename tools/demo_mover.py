"""机动模型演示：同一个目的地，四种运载器怎么走（§5.7）。

用法::

    python tools/demo_mover.py                                   # 默认 scenarios/mover_demo.txt
    python tools/demo_mover.py scenarios/mover_demo.txt --seconds 900
    python tools/demo_mover.py --plot docs/images/mover.png

它回答四个"看数字才敢信"的问题：

1. **地面件真的在绕地形吗** —— 打印 A* 路线长度与直线距离的比值。接近 1
   说明那片地形没有障碍，"绕行"无从体现；大于 1 才是绕了路。
2. **地形真的让车变慢了吗** —— 打印平均速度 / 峰值速度的比值，并把瞬时
   速度画出来。两者相等多半是画像没接上——它不报错，只是没有减速。
3. **空中件是不是直线** —— 它的路线长度与直线距离的比值应当 ≈ 1；顺便看
   它是不是按爬升率爬到巡航高度。
4. **水下件是不是真的待在水下** —— 它的 z 要从 0 潜到 -定深（按速率，不是
   瞬移），而且那道**水深门槛**要拦得住浅水（门槛 = 定深 + 离底余量，
   是算出来的，不是另写的一个数）。

弹另有**一节**（§5.8）：它不"到达"，它**终止**，所以那一节打的是阶段时间线、
最高点、峰值速度、剩余燃料与**起爆距离**——"打没打中"是四个数一起说的，
单看命中判词看不出是"收口收住了"还是"刚好飘进杀伤半径"。

数字全部来自真实装配出来的世界：地形按需生成、路线由 A* 算、位置由引擎
一步步推出来。刻意不做任何示意性作图。
"""

from __future__ import annotations

import sys
import unicodedata
from dataclasses import replace
from math import cos, hypot, radians, sin
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.services.map import Axial, TERRAIN_NAMES  # noqa: E402
from milsim.services.map.hex import offset_to_axial  # noqa: E402
from milsim.simulation import Simulation  # noqa: E402

DEFAULT_SCENARIO = "scenarios/mover_demo.txt"
#: 默认推演时长。给得比"轮式绕完那趟"宽一点：想定里轮式封了山地，绕行比
#: 可以到 7 倍以上，按最省的那条直线估时长会让它在表里显示成"在途"——
#: 那是时长不够，不是模型不对，但两件事在输出上长得一模一样。
DEFAULT_SECONDS = 6000.0
#: 记录间隔。**不是**机动件的更新周期——采样粒度与模型节拍是两件事，
#: 混在一起的话，改模型周期就会连带改掉图的密度。
SAMPLE_US = 5_000_000

#: 想找到"最需要绕行"的目的地，就在这些方向上试。
CANDIDATE_DIRECTIONS = 12
CANDIDATE_RADIUS_CELLS = 26          # 约 9 km（格中心距 = 边长 × √3 = 346 m）


def center_of(sim: Simulation, ref, cell: Axial) -> tuple[float, float]:
    zone = sim.maps.zone(ref.zone_id)
    return zone.frame.to_world(cell)


def _pad(text: str, width: int, *, right: bool = False) -> str:
    """按**显示宽度**补齐（东亚宽字符占两列）。

    ``str.ljust`` 补不齐：``len("走不通")`` 是 3、``len("28 格")`` 是 4，
    可屏幕上前者占 6 列、后者只占 5 列——``len()`` 数字符，屏幕数的是列。
    底下这几张表就是这工具的全部产出，列一歪读起来就费劲。
    """
    fill = " " * max(0, width - sum(
        2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text
    ))
    return fill + text if right else text + fill


def candidate_cells(start: Axial) -> list[tuple[float, Axial]]:
    """起点的等角候选环，返回 ``(方位角, 格)`` 12 组。

    环是**绕起点**的：一组固定的偏移量加到起点上。第一版把
    ``offset_to_axial`` 的结果直接当绝对坐标用了——那等于把整圈挂在**原点**
    上，只在"起点恰好靠近原点"时看不出毛病（陆上那两件就是这样，起点
    ``-24 0`` 离原点不远，选出来的目的地看着也合理）。潜艇起在 ``60 -10``，
    同一圈候选就全落在陆地上，12 个方向一个都走不通，装配直接退出。

    偏移量是**常量**（与起点无关），所以"同一份想定两次跑出的图完全一样"
    这条性质不受影响。
    """
    cells: list[tuple[float, Axial]] = []
    for index in range(CANDIDATE_DIRECTIONS):
        angle = index * (360.0 / CANDIDATE_DIRECTIONS)
        offset = offset_to_axial(
            round(CANDIDATE_RADIUS_CELLS * sin(radians(angle))),
            round(CANDIDATE_RADIUS_CELLS * cos(radians(angle))),
        )
        cells.append((angle, Axial(start.q + offset.q, start.r + offset.r)))
    return cells


def choose_goal(sim: Simulation, ref, start: Axial, profile: object = "ground") -> tuple[Axial, object]:
    """挑一个"最需要绕行"的目的地，返回 ``(格, 路线)``。

    判据是**代价 / 直线距离**最大的那个候选。代价而不是格数：代价里含
    地形权重，格数只数格子——绕远但好走的路在格数上反而更长。
    候选顺序固定（等角散点），所以同一份想定两次跑出的图完全一样。

    ``profile`` 传主跟踪底盘（轮式）的代价配置：它**封了山地**，所以挑出来
    的目的地是"这辆车绕远也到得了"的那一类。不这么做的话，绕行比最大的
    候选往往是一片山地——车根本进不去，主表里它的里程就是 0，"地形把它
    拖慢了没有"这个问题也就无从回答。**"进不去"由 gate_section 专门讲。**

    开头的区域预热见 :meth:`NavService.ensure_region`：A* 绕行会伸出
    ``ensure_corridor`` 的沿线带，带外读到默认地貌（编号 0 = 水域），
    地面画像下是一片假墙。
    """
    sx, sy = center_of(sim, ref, start)

    # 先把起点周围一大片地形生成出来。不预热的话，下面那个"代价 / 直线
    # 距离"最大的判据选的其实是"被假水域堵得最死的方向"。
    sim.nav.ensure_region(ref, start, CANDIDATE_RADIUS_CELLS + 24)

    best: tuple[Axial, object, float] | None = None

    for _, cell in candidate_cells(start):
        route = sim.nav.route(ref, cell, profile=profile)
        if route is None or len(route.cells) < 3:
            continue
        gx, gy = center_of(sim, ref, cell)
        straight = hypot(gx - sx, gy - sy)
        if straight <= 0.0:
            continue
        ratio = route.cost / straight
        if best is None or ratio > best[2]:
            best = (cell, route, ratio)

    if best is None:
        raise SystemExit("这个战区里找不到任何可到达的目的地——换一个地形种子")
    return best[0], best[1]


def route_length_m(sim: Simulation, ref, cells) -> float:
    total = 0.0
    for a, b in zip(cells, cells[1:]):
        ax, ay = center_of(sim, ref, a)
        bx, by = center_of(sim, ref, b)
        total += hypot(bx - ax, by - ay)
    return total


def gate_section(sim: Simulation, ref, wheeled, tracked) -> None:
    """轮式 vs 履带：同一批目的地，一个绕远、一个根本到不了。

    "可通行能力不同"如果只体现为速度差，那就是一句空话。这张表让它落到
    **格数**上：同一片丘陵，履带走 62 格，轮式绕 136 格；同一片山地，
    履带进得去，轮式进不去。两个底盘是**同一个组件类**的两组参数，
    不是两个类——差别全在这里的 ``blocked_terrain`` 上。
    """
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    print()
    print(f"通行能力对照（同一批目的地，半径 {CANDIDATE_RADIUS_CELLS} 格）：")
    print(f"{_pad('方向', 6, right=True)}  {_pad('目的地', 6)}"
          f"{_pad('履带', 10, right=True)}{_pad('轮式', 10, right=True)}   差异")
    for angle, cell in candidate_cells(ref.axial):
        tracked_route = sim.nav.route(ref, cell, profile=tracked)
        wheeled_route = sim.nav.route(ref, cell, profile=wheeled)
        if tracked_route is None and wheeled_route is None:
            continue
        kind = TERRAIN_NAMES.get(grid.terrain(cell), "?")
        left = f"{tracked_route.length_cells} 格" if tracked_route else "走不通"
        right = f"{wheeled_route.length_cells} 格" if wheeled_route else "走不通"
        if tracked_route and wheeled_route:
            ratio = wheeled_route.length_cells / max(1, tracked_route.length_cells)
            note = f"轮式绕 {ratio:.1f} 倍" if ratio > 1.05 else "一样"
        elif tracked_route:
            note = "只有履带进得去"
        else:
            note = "只有轮式进得去"
        print(f"{_pad(f'{angle:.0f}°', 6, right=True)}  {_pad(kind, 6)}"
              f"{_pad(left, 10, right=True)}{_pad(right, 10, right=True)}   {note}")


def draft_section(sim: Simulation, eid, anchor: Axial) -> None:
    """吃水门槛：同一片水域，吃水不同能去的格数不同。

    吃水是**这条船**的参数（``min_water_depth``），不是水面画像的。陆地格
    的水深恒为 0，所以这里只数水域格。三种吃水用同一套 A*、同一片地形，
    差别只在那个数——它要是不生效，三行的可达数就会一模一样。

    量的是 ``anchor`` 那片水（出发点），不是"它现在停在哪"：位置一挪，
    这张表的数字就该跟着变，那是两件事混在一起了。所以**参考格也要挪到
    ``anchor``**——只挪窗口、航线还从当前位置算，量出来的就成了"从它现在
    的位置能去到出发点周围哪些格"。
    """
    mover = sim.entities[eid].require("mover")
    ref = replace(sim.store.cell_of(eid), axial=anchor)
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    sim.nav.ensure_region(ref, anchor, 24)

    neighbours = [
        Axial(anchor.q + dq, anchor.r + dr)
        for dq in range(-16, 17)
        for dr in range(-16, 17)
        if grid.terrain(Axial(anchor.q + dq, anchor.r + dr)) == 0
    ]
    draft = float(mover.spec["min_water_depth"])
    print()
    print(f"吃水门槛（船位 {anchor}，水深 {grid.water_depth(anchor):.1f} m，"
          f"周围水域格 {len(neighbours)} 个）：")
    for label, gate in (
        ("无吃水限制", 0.0),
        (f"吃水 {draft:.0f} m（本船）", draft),
        ("吃水 30 m（假设更深的船）", 30.0),
    ):
        profile = sim.nav.capable_profile("water", min_water_depth=gate)
        # 起点自己先判一次：门槛高过它那一格的水深时，它**根本停不到那儿**。
        # 不单独判的话数出来是 1 —— 那是"从起点到起点"的平凡路线，
        # 看起来像"还剩一个地方可以去"。
        afloat = sim.nav.passable(ref, anchor, profile=profile)
        reached = 0
        if afloat:
            reached = 1 + sum(
                1 for cell in neighbours
                if cell != anchor
                and sim.nav.route(ref, cell, profile=profile) is not None
            )
        print(f"    {_pad(label, 28)}可达 {reached:>4} / {len(neighbours)} 格"
              f"{'' if afloat else '  ← 连船位那一格都不够深：它停不到那儿'}")


def depth_gate_section(sim: Simulation, eid, anchor: Axial) -> None:
    """定深门槛：同一片海，潜得越深，能去的格越少。

    这张表每一行改的其实是**今天潜多深**——因为门槛不是另写的一个数，而是
    「定深 + 离底余量」算出来的（§5.7.10）。它要是不生效，四行会一模一样。

    量的是 ``anchor`` 那片海（出发点），不是**跑完之后**停在哪。用当前位置
    量过一次，四个数全一样——因为潜艇最后停在 552 m 深的水里，四档门槛
    都拦不住它，看起来像"门槛是个摆设"。事实是门槛活得好好的，只是那片海
    太深：换回出发点（169 m）四档立刻分开。

    最后一档是**四倍定深**，而它落到了 0：出发点那一格水深 169 m，压到
    240 m 的定深上连**起点自己**都过不了门槛。这不是"沿途被挡"，是
    "它根本没法出现在那里"——门槛判的是**格**，起点也是一格。
    """
    mover = sim.entities[eid].require("mover")
    ref = replace(sim.store.cell_of(eid), axial=anchor)
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    sim.nav.ensure_region(ref, anchor, 20)

    span = 14
    water = [
        Axial(anchor.q + dq, anchor.r + dr)
        for dq in range(-span, span + 1)
        for dr in range(-span, span + 1)
        if grid.terrain(Axial(anchor.q + dq, anchor.r + dr)) == 0
    ]
    clearance = float(mover.spec["hull_clearance"])
    ordered = float(mover.spec["cruise_depth"])
    print()
    print(f"定深门槛（潜艇出发点 {anchor}，水深 {grid.water_depth(anchor):.1f} m，"
          f"周围水域格 {len(water)} 个；离底余量 {clearance:.0f} m）：")
    for label, depth in (
        ("不限制（贴着水面走）", 0.0),
        (f"本艇 {ordered:.0f} m", ordered),
        (f"加倍 {ordered * 2:.0f} m", ordered * 2.0),
        (f"四倍 {ordered * 4:.0f} m（比出发点还深）", ordered * 4.0),
    ):
        gate = 0.0 if depth <= 0.0 else depth + clearance
        profile = sim.nav.capable_profile("water", min_water_depth=gate)
        # 出发点自己先判一次：门槛高过它那一格的水深时，它**根本潜不到那儿**。
        # 不单独判的话数出来是 1 —— 那是"从起点到起点"的平凡路线，
        # 看起来像"还剩一个地方可以去"。
        afloat = sim.nav.passable(ref, anchor, profile=profile)
        reached = 0
        if afloat:
            reached = 1 + sum(
                1 for cell in water
                if cell != anchor
                and sim.nav.route(ref, cell, profile=profile) is not None
            )
        print(f"    定深 {_pad(label, 28)}门槛 {gate:>6.1f} m  "
              f"可达 {reached:>4} / {len(water)} 格"
              f"{'' if afloat else '  ← 连出发点那一格都不够深：一步都动不了'}")


def run(scenario: str, seconds: float, plot: str | None) -> int:
    sim = Simulation.from_scenario_file(Path(scenario))
    sim.build()
    sim.initialize()

    names = ("CAR_1", "CAR_2", "JET_1", "SUB_1")
    ids = {name: sim.registry.by_name(name).entity_id for name in names}
    movers = {name: sim.entities[eid].require("mover") for name, eid in ids.items()}

    ref = sim.store.cell_of(ids["CAR_1"])
    start = ref.axial
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    goal, route = choose_goal(sim, ref, start, movers["CAR_1"].travel_config())

    sx, sy = center_of(sim, ref, start)
    gx, gy = center_of(sim, ref, goal)
    straight_m = hypot(gx - sx, gy - sy)
    route_m = route_length_m(sim, ref, route.cells)

    # 水下件在另一个域里（起点在深水），所以它有自己的目的地与自己的直线
    # 距离。拿陆上那件的直线去算它的"绕行比"会得出一个没有意义的数。
    sub_ref = sim.store.cell_of(ids["SUB_1"])
    sub_start = sub_ref.axial
    sub_goal, sub_route = choose_goal(
        sim, sub_ref, sub_start, movers["SUB_1"].travel_config()
    )
    ssx, ssy = center_of(sim, sub_ref, sub_start)
    sgx, sgy = center_of(sim, sub_ref, sub_goal)
    sub_straight_m = hypot(sgx - ssx, sgy - ssy)

    print(f"战区：{grid.size:.0f} m 格；起点 {start} → 目的地 {goal}"
          f"（{TERRAIN_NAMES.get(grid.terrain(goal), '?')}）")
    print(f"直线 {straight_m/1000:.2f} km；A* 路线 {route.length_cells} 格 / "
          f"{route_m/1000:.2f} km（绕行比 {route_m/straight_m:.3f}）")
    print(f"潜艇：起点 {sub_start} → 目的地 {sub_goal}（水深 "
          f"{grid.water_depth(sub_goal):.1f} m）；直线 {sub_straight_m/1000:.2f} km，"
          f"A* 路线 {sub_route.length_cells} 格")
    print()

    # -- 下达机动命令（运行期，不是想定） ---------------------------------
    if not movers["CAR_1"].move_to_cell(goal):
        print(f"警告：CAR_1 没有路线——{movers['CAR_1'].blocked_reason()}")
    if not movers["CAR_2"].move_to_cell(goal):
        print(f"警告：CAR_2 没有路线——{movers['CAR_2'].blocked_reason()}")
    movers["JET_1"].move_to_point(gx, gy, float(movers["JET_1"].spec["cruise_altitude"]))
    if not movers["SUB_1"].move_to_cell(sub_goal):
        print(f"警告：SUB_1 没有路线——{movers['SUB_1'].blocked_reason()}")

    # -- 推演并记录 --------------------------------------------------------
    history = {name: {"t": [], "x": [], "y": [], "z": [], "v": []} for name in names}
    for _ in range(max(1, int(seconds / (SAMPLE_US / 1_000_000.0)))):
        sim.run_for(SAMPLE_US)
        now = sim.engine.now / 1_000_000.0
        for name, eid in ids.items():
            pose = sim.store.pose_of(eid)
            if pose is None:
                continue
            history[name]["t"].append(now)
            history[name]["x"].append(pose[0])
            history[name]["y"].append(pose[1])
            history[name]["z"].append(pose[2])
            history[name]["v"].append(pose[4])

    # -- 表 -----------------------------------------------------------------
    print(f"{_pad('载具', 8)}{_pad('里程 km', 9, right=True)}"
          f"{_pad('平均 m/s', 10, right=True)}{_pad('峰值 m/s', 10, right=True)}"
          f"{_pad('用时 s', 9, right=True)}{_pad('到达', 6, right=True)}"
          f"{_pad('状态', 8, right=True)}")
    stats: dict[str, dict[str, float]] = {}
    for name in names:
        mover = movers[name]
        row = history[name]
        travelled = mover.travelled_m()
        # 用时与峰值都取内部记账/真实采样，不从"里程÷速度"倒推——倒推出来的
        # 平均速度会随采样间隔变（采到 5 s 一次就少算几秒），看起来像模型
        # 跑得比额定还快。
        span = mover.elapsed_s()
        avg = travelled / span if span > 0.0 else 0.0
        peak = max(row["v"]) if row["v"] else 0.0
        state = "走不通" if mover.blocked() else ("已到" if mover.arrived() else "在途")
        stats[name] = {"travelled": travelled, "avg": avg, "peak": peak, "span": span}
        print(f"{_pad(name, 8)}{travelled/1000:>9.2f}{avg:>10.2f}{peak:>10.2f}"
              f"{span:>9.1f}{_pad('是' if mover.arrived() else '否', 6, right=True)}"
              f"{_pad(state, 8, right=True)}")

    print()
    for name in ("CAR_1", "CAR_2"):
        row = stats[name]
        slow = row["avg"] / row["peak"] if row["peak"] > 0 else 0.0
        print(f"{name}：里程 / 直线 = {row['travelled']/straight_m:.3f}，"
              f"平均 / 峰值速度 = {slow:.3f}"
              f"（明显小于 1 才说明地形真的把它拖慢了）")
    jet = stats["JET_1"]
    print(f"JET_1：里程 / 直线 = {jet['travelled']/straight_m:.3f}"
          f"（水平飞的是直线，多出来的是爬升 {movers['JET_1'].spec['cruise_altitude']:.0f} m "
          f"的高度，里程按三维距离算）")

    sub = movers["SUB_1"]
    ordered = float(sub.spec["cruise_depth"])
    tolerance = float(sub.spec["depth_tolerance"])
    sub_z = sim.store.pose_of(ids["SUB_1"])[2]
    print(f"SUB_1：里程 / 直线 = {stats['SUB_1']['travelled']/sub_straight_m:.3f}，"
          f"定深 -{ordered:.0f} m → 实际 {sub_z:.1f} m（容差 ±{tolerance:.0f} m）；"
          f"多出来的那一截里有下潜的 {ordered:.0f} m（按三维距离算）")
    print(f"       被海底顶住（z 高于定深）："
          f"{'是——它同时报了走不通' if sub.depth_limited() else '否'}")

    # 三节"门槛"实测：可通行能力、吃水、定深都是**参数**，不落到数字上看不出来。
    # 后两节量的是**出发点**那片水，不是"跑完停在哪"——位置一挪数字就变的话，
    # 这张表测的就成了"它到哪了"，而不是"门槛生不生效"。
    gate_section(
        sim, ref,
        movers["CAR_1"].travel_config(),
        movers["CAR_2"].travel_config(),
    )
    ship = sim.registry.by_name("SHIP_1")
    if ship is None:
        print()
        print("（想定里没有 SHIP_1，跳过吃水一节）")
    else:
        draft_section(sim, ship.entity_id, sim.store.cell_of(ship.entity_id).axial)
    depth_gate_section(sim, ids["SUB_1"], sub_start)

    missile_section(sim)

    if plot:
        draw(sim, ids, history, ref, route.cells, straight_m, Path(plot))
    return 0


# ---------------------------------------------------------------------------
# 弹：它不"到达"，它终止（§5.8）
# ---------------------------------------------------------------------------

def missile_section(sim: Simulation) -> None:
    """弹的那一节：阶段时间线 + 命中/起爆距离 + 有没有打到过载上限。

    **为什么这四个数要一起看**：``命中`` 只说"最后一小步落在杀伤半径里"。
    它落进去是**收口收住了**（末段比例导引在干活）还是**刚好飘进去**（末段
    根本没转过弯来），单看判词分不出。所以一并给：末段起了多久、峰值速度
    多少（有没有被 ``max_speed`` 卡住）、过载有没有打满、油还剩多少。

    弹在**想定里**接的两件东西也在这里被验一遍：弹体从自己实体的
    ``guidance`` 槽里找到了制导件、制导件按 ``target_name`` 找到了靶标。
    两条线任何一条没接上都不会走到这里——它们要么装配期报错，要么根本不动。
    """
    names = [n for n in ("SSM_CRUISE_1", "SRBM_1")
             if sim.registry.by_name(n) is not None]
    if not names:
        print()
        print("（想定里没有弹，跳过弹那一节）")
        return

    print()
    print("=" * 78)
    print("弹：弹体管“能不能做到”，制导件管“要往哪飞”（轨迹形态 = 一张阶段表）")
    print("=" * 78)
    print(f"{_pad('弹', 15)}{_pad('终止', 8)}{_pad('起爆 m', 9, right=True)}"
          f"{_pad('射程 km', 9, right=True)}{_pad('最高 km', 9, right=True)}"
          f"{_pad('峰值 m/s', 10, right=True)}{_pad('过载 g', 8, right=True)}"
          f"{_pad('剩油 kg', 9, right=True)}{_pad('用时 s', 8, right=True)}")
    for name in names:
        eid = sim.registry.by_name(name).entity_id
        body = sim.entities[eid].require("mover")
        guide = body.guidance()
        x, y, _z = sim.store.pose_of(eid)[:3]
        traveled = body.travelled_m()
        accel = body.peak_accel() / 9.80665
        cap = float(body.spec["radial_accel"]) / 9.80665
        print(f"{_pad(name, 15)}{_pad(body.terminate_reason(), 8)}"
              f"{guide.detonation_range_m():>9.2f}{traveled/1000:>9.1f}"
              f"{body.max_altitude_m()/1000:>9.2f}"
              f"{body.max_speed_mps():>10.0f}"
              f"{accel:>7.1f}{'*' if cap > 0 and accel >= cap * 0.98 else ' '}"
              f"{body.fuel_kg():>9.1f}{body.elapsed_s():>8.0f}")
        print(f"{_pad('', 15)}阶段：" + " → ".join(
            f"{row[0]}({row[1]:.0f}~{'—' if row[2] is None else f'{row[2]:.0f}'}s)"
            for row in guide.timeline()))
        print(f"{_pad('', 15)}终点 ({x/1000:.1f}, {y/1000:.1f}) km；"
              f"制导判词 {guide.terminate_reason()!r}，命中 {guide.hit()}，"
              f"总质量 {body.mass_kg():.0f} kg")
    print()
    print("“过载 g”带 * = 全程打到过弹体的 radial_accel 上限（说明那一段是"
          "“拉不动”而不是“没拉”，标称值就在参数表里）。")
    print("起爆距离是“第一次落进引信点火区”的那一步，总贴着 lethal_radius，"
          "**不是脱靶量**；量脱靶量要在子步上采（§5.8）。")


# ---------------------------------------------------------------------------
# 画图
# ---------------------------------------------------------------------------

#: 地形 → 配色。浅色底，与文档的浅色主题一致。
#: 键必须与 ``TERRAIN_NAMES`` 的值**逐字相同**：写错不会报错，只是那一
#: 类地貌拿不到颜色（画成白色）、图例里也整个消失。
TERRAIN_PALETTE = {
    "水域": "#a9c9e8",
    "平原": "#efe9d5",
    "林地": "#a3c894",
    "丘陵": "#dcc791",
    "山地": "#b9a99b",
    "城镇": "#d2d2d2",
}

TRACKS = {
    "CAR_1": ("#c0392b", "-", "轮式 25 m/s"),
    "CAR_2": ("#d9822b", "-", "履带 15 m/s"),
    "JET_1": ("#2c6fbb", "--", "空中 180 m/s"),
}

#: 潜艇曲线的颜色。**它不出现在轨迹图里**：潜艇在另一个域（深水），把它
#: 的航迹画进陆上那张图会把视野拉大将近一倍，而地形正是那张图要讲的东西。
#: 它的深度剖面画在第三张图上——那才是"它是潜艇"的证据。
SUB_COLOR = "#1f7a6b"


def draw(sim, ids, history, ref, route_cells, straight_m, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Patch

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    grid = sim.maps.grid(ref.zone_id, ref.layer)
    size = grid.size

    # 视野按**实际走过的范围**定，不写死格数：写死的话，换一份想定要么
    # 什么都看不见、要么半张图是空白。
    xs = [v for name in TRACKS for v in history[name]["x"]]
    ys = [v for name in TRACKS for v in history[name]["y"]]
    margin = 4.0 * size * 1.732
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin

    zone = sim.maps.zone(ref.zone_id)
    corners = [zone.frame.from_world(x, y) for x, y in
               ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
    q_lo = min(c.q for c in corners) - 4
    q_hi = max(c.q for c in corners) + 4
    r_lo = min(c.r for c in corners) - 4
    r_hi = max(c.r for c in corners) + 4

    polys = []
    colors = []
    present: set[str] = set()
    for q in range(q_lo, q_hi + 1):
        for r in range(r_lo, r_hi + 1):
            cell = Axial(q, r)
            cx, cy = center_of(sim, ref, cell)
            if not (x0 <= cx <= x1 and y0 <= cy <= y1):
                continue
            angles = [radians(60 * k + 30) for k in range(6)]
            polys.append([(cx + size * cos(a), cy + size * sin(a)) for a in angles])
            name = TERRAIN_NAMES.get(grid.terrain(cell), "")
            present.add(name)
            colors.append(TERRAIN_PALETTE.get(name, "#ffffff"))

    fig = plt.figure(figsize=(19.0, 5.2), dpi=110)

    ax = fig.add_subplot(1, 4, 1)
    ax.add_collection(PolyCollection(polys, facecolors=colors, edgecolors="none"))
    sx, sy = center_of(sim, ref, route_cells[0])
    gx, gy = center_of(sim, ref, route_cells[-1])
    ax.plot([sx, gx], [sy, gy], ":", color="#555555", lw=1.5, label="直线距离")
    for name, (color, style, label) in TRACKS.items():
        row = history[name]
        ax.plot(row["x"], row["y"], style, color=color, lw=2.0, label=label)
    # A* 路线单独画成细线：它是"打算怎么走"，实体轨迹是"实际怎么走"。
    # 两条分不开的话，"沿一条不存在的路一直走下去"这类问题就看不出来。
    route_x = [center_of(sim, ref, cell)[0] for cell in route_cells]
    route_y = [center_of(sim, ref, cell)[1] for cell in route_cells]
    ax.plot(route_x, route_y, "-", color="#333333", lw=0.8, alpha=0.7, label="A* 路线")
    ax.plot(sx, sy, "o", color="#111111", ms=7)
    ax.plot(gx, gy, "*", color="#111111", ms=14)
    ax.annotate("起点", (sx, sy), textcoords="offset points", xytext=(8, -12), fontsize=9)
    ax.annotate("目的地", (gx, gy), textcoords="offset points", xytext=(8, 6), fontsize=9)
    ax.autoscale_view()
    ax.set_aspect("equal")
    ax.set_title(f"轨迹与地形（{size:.0f} m 格）\n直线 {straight_m/1000:.2f} km", fontsize=11)
    # 地形图例：底图按地貌上色，不给图例的话读图的人分不出哪片是水、哪片是山，
    # 而"绕行"这件事全靠地形才看得出来。轨迹那条按 TRACKS 补回来——换自定义
    # handles 会把它原本的 label 一起丢掉。
    handles = [
        Patch(facecolor=TERRAIN_PALETTE[name], label=name)
        for name in sorted(present)
        if name in TERRAIN_PALETTE
    ]
    handles.append(
        plt.Line2D([], [], color="#555555", ls=":", lw=1.5, label="直线距离")
    )
    handles.extend(
        plt.Line2D([], [], color=color, ls=style, lw=2.0, label=label)
        for color, style, label in TRACKS.values()
    )
    handles.append(plt.Line2D([], [], color="#333333", lw=0.8, label="A* 路线"))
    handles.append(
        plt.Line2D([], [], color="#111111", marker="o", ls="none", ms=7, label="起点")
    )
    handles.append(
        plt.Line2D([], [], color="#111111", marker="*", ls="none", ms=10, label="目的地")
    )
    # 图例放到坐标区**下面**：地形是这张图要说的东西，图例压在图上就会
    # 盖掉最容易出事的那一块（起点附近），而那张图恰恰是靠地形才读得懂。
    ax.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.06),
        fontsize=7.5,
        frameon=False,
        ncol=4,
        columnspacing=1.0,
    )
    ax.tick_params(labelsize=8)

    ax2 = fig.add_subplot(1, 4, 2)
    for name in ("CAR_1", "CAR_2"):
        ax2.plot(history[name]["t"], history[name]["v"],
                 color=TRACKS[name][0], lw=1.6, label=TRACKS[name][2])
    ax2.set_xlabel("时间 s", fontsize=9)
    ax2.set_ylabel("速度 m/s", fontsize=9)
    ax2.set_title("速度：速度 = 最高速度 / 当地通行代价\n（曲线分段就是它在穿过不同地形）",
                  fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    ax2.tick_params(labelsize=8)

    ax3 = fig.add_subplot(1, 4, 3)
    ax3.plot(history["CAR_1"]["t"], history["CAR_1"]["z"],
             color=TRACKS["CAR_1"][0], lw=1.6, label="轮式 z（贴地）")
    ax3.plot(history["JET_1"]["t"], history["JET_1"]["z"],
             color=TRACKS["JET_1"][0], lw=1.6, label="空中 z（爬到巡航）")
    ax3.set_xlabel("时间 s", fontsize=9)
    ax3.set_ylabel("海拔 m", fontsize=9)
    ax3.set_title("高度：地面贴地、空中按爬升率\n"
                  "（z 不按自己的速率改，位置数据就自相矛盾）",
                  fontsize=10)
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=8)
    ax3.tick_params(labelsize=8)

    # 潜艇**单独一格**。和空中挤在一起的话，它那 60 m 的下潜被 4000 m 的
    # 巡航高度压成一根贴在 0 上的直线——"它是潜艇"这件事反而看不出来，
    # 而这一格的全部意义就是"看它有没有真的潜下去、有没有按速率潜"。
    ax4 = fig.add_subplot(1, 4, 4)
    sub = sim.entities[ids["SUB_1"]].require("mover")
    ordered = float(sub.spec["cruise_depth"])
    dive = float(sub.spec["dive_rate"])
    ax4.plot(history["SUB_1"]["t"], history["SUB_1"]["z"],
             color=SUB_COLOR, lw=1.8, label=f"潜艇 z（定深 -{ordered:.0f} m）")
    ax4.axhline(-ordered, color=SUB_COLOR, ls=":", lw=1.2, label="命令的定深")
    ax4.axhline(0.0, color="#888888", ls="-", lw=0.8, label="海平面")
    # 横轴只画**下潜那一段**。整段推演 6000 s 的话，那 30 s 的下潜被压成
    # 左边缘一根垂直线，斜率和速率都读不出来——"按速率"这件事就白画了。
    dive_s = ordered / dive if dive > 0.0 else 0.0
    window = max(60.0, dive_s * 4.0)
    last = history["SUB_1"]["t"][-1] if history["SUB_1"]["t"] else window
    ax4.set_xlim(0.0, min(window, last))
    ax4.axvline(dive_s, color=SUB_COLOR, ls=":", lw=1.0, alpha=0.7)
    ax4.annotate(f"{dive_s:.0f} s 到底", (dive_s, -ordered),
                 textcoords="offset points", xytext=(6, 6),
                 fontsize=8, color=SUB_COLOR)
    ax4.set_xlabel(f"时间 s（只画前 {min(window, last):.0f} s）", fontsize=9)
    ax4.set_ylabel("海拔 m", fontsize=9)
    ax4.set_title(f"潜艇：按 dive_rate {dive:.0f} m/s 下潜，"
                  f"{dive_s:.0f} s 到底\n（之后一直平着走；"
                  f"一步到就是瞬移）",
                  fontsize=10)
    ax4.grid(alpha=0.3)
    ax4.legend(fontsize=8, loc="center right")
    ax4.tick_params(labelsize=8)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    # bbox_inches="tight"：图例挂在坐标区外面，不收紧就会被裁掉一条。
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"图已写入 {out}")


def main(argv: list[str]) -> int:
    scenario = DEFAULT_SCENARIO
    seconds = DEFAULT_SECONDS
    plot: str | None = None

    rest = list(argv[1:])
    index = 0
    while index < len(rest):
        item = rest[index]
        if item == "--seconds" and index + 1 < len(rest):
            seconds = float(rest[index + 1])
            index += 2
        elif item == "--plot" and index + 1 < len(rest):
            plot = rest[index + 1]
            index += 2
        elif item.startswith("-"):
            print(f"无法识别的参数：{item}", file=sys.stderr)
            return 2
        else:
            scenario = item
            index += 1

    return run(scenario, seconds, plot)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
