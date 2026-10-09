"""机动组件单独测试：下达目的地 → 推演采样 → 数值表 + 轨迹/速度/高度图（§5.7）。

与 ``demo_mover.py`` 的分工：demo 讲"四类运载器怎么走"的完整故事（含门槛
对照与弹那一节）；本工具只做**组件体检**——想定装配好、命令一下、推演一
跑，回答三个问题：

1. **轨迹对不对** —— 地面件绕地形（A*）、空中件走直线、水面/水下件待在
   自己的域里；
2. **速度对不对** —— 按"最高速度 ÷ 当地通行代价"随地形变化（曲线分段就
   是在穿越地貌），而不是恒速瞬移；
3. **高度对不对** —— 空中按爬升率爬到巡航高度、潜艇按下潜率到定深，地面
   贴地 z≈0。

用法::

    python tools/check_mover.py                          # 全部载具，默认想定
    python tools/check_mover.py --who CAR_1,JET_1        # 只测两件
    python tools/check_mover.py --seconds 3000 --plot none
    python tools/check_mover.py scenarios/other.txt      # 换想定

默认复用 ``scenarios/mover_demo.txt``（四类机动件 + 船，地形种子固定），
目的地由 :func:`demo_mover.choose_goal` 现场挑——同一份想定两次跑出的数字
完全一样。
"""

from __future__ import annotations

import argparse
import sys
from math import cos, hypot, radians, sin
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent
for _p in (str(_TOOLS_DIR), str(_TOOLS_DIR.parent / "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# 复用 demo_mover 的选目的地与画图素材——两处各写一份迟早分叉。
from demo_mover import (  # noqa: E402
    SAMPLE_US,
    TERRAIN_PALETTE,
    _pad,
    center_of,
    choose_goal,
)
from milsim.services.map import Axial, TERRAIN_NAMES  # noqa: E402
from milsim.simulation import Simulation  # noqa: E402

DEFAULT_SCENARIO = "scenarios/mover_demo.txt"
DEFAULT_SECONDS = 6000.0

#: 想定里全部机动件。地面件（轮式/履带/潜艇/船）走 ``move_to_cell``（A* 绕
#: 地形）；空中件走 ``move_to_point``（直线 + 爬升）。
CELL_MOVERS = ("CAR_1", "CAR_2", "SUB_1", "SHIP_1")
POINT_MOVERS = ("JET_1",)
ALL_MOVERS = CELL_MOVERS + POINT_MOVERS

#: 载具 → (颜色, 线型, 图例)。潜艇不进轨迹图（另一域，会把视野拉大一倍），
#: 它的深度剖面单独一格——与 demo 同一个理由。
TRACKS = {
    "CAR_1": ("#c0392b", "-", "轮式 25 m/s"),
    "CAR_2": ("#d9822b", "-", "履带 15 m/s"),
    "JET_1": ("#2c6fbb", "--", "空中 180 m/s"),
    "SUB_1": ("#1f7a6b", "-.", "潜艇"),
    "SHIP_1": ("#7d5ba6", ":", "舰船"),
}


def issue_orders(sim: Simulation, movers: dict, refs: dict, goals: dict) -> None:
    """下达机动命令（运行期，不是想定）。

    * **CAR_1** 用自己的通行画像挑"最需要绕行"的目的地；**CAR_2 跟去同一
      处**——同一目的地、两种底盘，里程差才是通行能力的直接读数；
    * **SUB_1 / SHIP_1** 在自己的水域里各自挑（判据相同、画像不同：潜艇的
      代价配置带定深，船的带吃水）；挑不到就跳过并说明，不硬退出——
      组件体检要的是"能测的那几件先测到"；
    * **JET_1** 飞往 CAR_1 的目的地正上方：直线对绕行，同一对起终点才有
      对照意义（CAR_1 缺席时朝正东飞一段固定距离）。
    """
    if "CAR_1" in movers:
        ref = refs["CAR_1"]
        goal, route = choose_goal(sim, ref, ref.axial, movers["CAR_1"].travel_config())
        goals["CAR_1"] = goal
        movers["CAR_1"].move_to_cell(goal)
        if "CAR_2" in movers and not movers["CAR_2"].move_to_cell(goal):
            print(f"警告：CAR_2 没有路线——{movers['CAR_2'].blocked_reason()}")
    elif "CAR_2" in movers:
        ref = refs["CAR_2"]
        goal, _route = choose_goal(
            sim, ref, ref.axial, movers["CAR_2"].travel_config()
        )
        goals["CAR_2"] = goal
        movers["CAR_2"].move_to_cell(goal)

    for name in ("SUB_1", "SHIP_1"):
        if name not in movers:
            continue
        ref = refs[name]
        # choose_goal 挑不到可达目的地时是 raise SystemExit（demo 的口径），
        # 不是返回 None——这里接住它跳过该件，组件体检不因一件缺席而中断。
        try:
            goal, _route = choose_goal(
                sim, ref, ref.axial, movers[name].travel_config()
            )
        except SystemExit:
            print(f"警告：{name} 挑不到可达目的地（水域太浅或太窄），跳过")
            continue
        goals[name] = goal
        if not movers[name].move_to_cell(goal):
            print(f"警告：{name} 没有路线——{movers[name].blocked_reason()}")

    if "JET_1" in movers:
        jet = movers["JET_1"]
        alt = float(jet.spec["cruise_altitude"])
        if "CAR_1" in goals:
            gx, gy = center_of(sim, refs["CAR_1"], goals["CAR_1"])
        else:
            px, py = sim.store.pose_of(jet.entity_id)[:2]
            gx, gy = px, py + 15_000.0          # 兜底：朝正北飞一段固定距离
        jet.move_to_point(gx, gy, alt)


def sample(sim: Simulation, ids: dict, seconds: float) -> dict:
    """逐 ``SAMPLE_US`` 推演并记录 ``(t, x, y, z, v)``。"""
    history = {name: {"t": [], "x": [], "y": [], "z": [], "v": []} for name in ids}
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
    return history


def report(sim: Simulation, movers: dict, history: dict, straight_m: float) -> None:
    """数值表：里程 / 平均·峰值速度 / 用时 / 到达状态 / 末位置。

    用时与平均速度取组件内部记账（``elapsed_s`` / ``travelled_m``），不从
    "里程 ÷ 墙钟"倒推——采样间隔一变，倒推值就跟着变，看起来像模型跑得
    比额定还快。
    """
    print()
    print(f"{_pad('载具', 8)}{_pad('里程 km', 9, right=True)}"
          f"{_pad('平均 m/s', 10, right=True)}{_pad('峰值 m/s', 10, right=True)}"
          f"{_pad('用时 s', 9, right=True)}{_pad('到达', 6, right=True)}"
          f"{_pad('状态', 8, right=True)}   末位置 km")
    for name, mover in movers.items():
        row = history[name]
        travelled = mover.travelled_m()
        span = mover.elapsed_s()
        avg = travelled / span if span > 0.0 else 0.0
        peak = max(row["v"]) if row["v"] else 0.0
        state = "走不通" if mover.blocked() else ("已到" if mover.arrived() else "在途")
        x, y, z = row["x"][-1] / 1000.0, row["y"][-1] / 1000.0, row["z"][-1]
        print(f"{_pad(name, 8)}{travelled/1000:>9.2f}{avg:>10.2f}{peak:>10.2f}"
              f"{span:>9.1f}{_pad('是' if mover.arrived() else '否', 6, right=True)}"
              f"{_pad(state, 8, right=True)}   ({x:.1f}, {y:.1f}, z={z:.0f})")
    if "CAR_1" in movers and straight_m > 0.0:
        row = history["CAR_1"]
        print()
        print(f"CAR_1：里程 / 直线 = {movers['CAR_1'].travelled_m()/straight_m:.3f}"
              f"（明显大于 1 才说明 A* 真的在地形上绕了路）")


def draw(sim: Simulation, ids: dict, refs: dict, history: dict,
         goals: dict, route_cells, out: Path) -> None:
    """四联图：轨迹+地形 / 速度-时间 / 高度-时间 / 潜艇深度剖面。

    地形配色与图例位置沿用 demo_mover 的口径（浅色底、图例挂坐标区外）；
    潜艇不进轨迹图而单独一格——60 m 的下潜被 4000 m 巡航高度压扁后什么都
    看不出来，"按速率下潜"这条证据就丢了。
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Patch

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    ref = refs["CAR_1"] if "CAR_1" in refs else next(iter(refs.values()))
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    size = grid.size
    has_sub = "SUB_1" in history

    # 视野按实际走过的范围定；轨迹图只画同一域的几件（不含潜艇）。
    plotted = [n for n in history if n != "SUB_1"]
    xs = [v for name in plotted for v in history[name]["x"]]
    ys = [v for name in plotted for v in history[name]["y"]]
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

    polys, colors, present = [], [], set()
    for q in range(q_lo, q_hi + 1):
        for r in range(r_lo, r_hi + 1):
            cell = Axial(q, r)
            cx, cy = center_of(sim, ref, cell)
            if not (x0 <= cx <= x1 and y0 <= cy <= y1):
                continue
            angles = [radians(60 * k + 30) for k in range(6)]
            polys.append([(cx + size * cos(a), cy + size * sin(a))
                          for a in angles])
            name = TERRAIN_NAMES.get(grid.terrain(cell), "")
            present.add(name)
            colors.append(TERRAIN_PALETTE.get(name, "#ffffff"))

    ncol = 4 if has_sub else 3
    fig = plt.figure(figsize=(4.8 * ncol + 1.0, 5.0), dpi=110)

    ax = fig.add_subplot(1, ncol, 1)
    ax.add_collection(PolyCollection(polys, facecolors=colors, edgecolors="none"))
    for name in plotted:
        color, style, label = TRACKS[name]
        ax.plot(history[name]["x"], history[name]["y"], style,
                color=color, lw=2.0, label=label)
    if route_cells:
        rx = [center_of(sim, ref, cell)[0] for cell in route_cells]
        ry = [center_of(sim, ref, cell)[1] for cell in route_cells]
        ax.plot(rx, ry, "-", color="#333333", lw=0.8, alpha=0.7, label="A* 路线")
    for name, cell in goals.items():
        gx, gy = center_of(sim, refs.get(name, ref), cell)
        ax.plot(gx, gy, "*", color="#111111", ms=12)
    ax.autoscale_view()
    ax.set_aspect("equal")
    ax.set_title(f"轨迹与地形（{size:.0f} m 格）", fontsize=11)
    handles = [Patch(facecolor=TERRAIN_PALETTE[n], label=n)
               for n in sorted(present) if n in TERRAIN_PALETTE]
    handles.extend(plt.Line2D([], [], color=TRACKS[n][0], ls=TRACKS[n][1],
                              lw=2.0, label=TRACKS[n][2]) for n in plotted)
    if route_cells:
        handles.append(plt.Line2D([], [], color="#333333", lw=0.8, label="A* 路线"))
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.06),
              fontsize=7.5, frameon=False, ncol=4, columnspacing=1.0)
    ax.tick_params(labelsize=8)

    col = 2
    ax2 = fig.add_subplot(1, ncol, col)
    for name in history:
        color, _style, label = TRACKS[name]
        ax2.plot(history[name]["t"], history[name]["v"], color=color, lw=1.6,
                 label=label)
    ax2.set_xlabel("时间 s", fontsize=9)
    ax2.set_ylabel("速度 m/s", fontsize=9)
    ax2.set_title("速度：曲线分段 = 穿越不同地形\n（恒速一条直线才是异常）",
                  fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=7.5)
    ax2.tick_params(labelsize=8)

    col += 1
    ax3 = fig.add_subplot(1, ncol, col)
    for name in history:
        if name == "SUB_1":
            continue
        color, _style, label = TRACKS[name]
        ax3.plot(history[name]["t"], history[name]["z"], color=color, lw=1.6,
                 label=f"{label} z")
    ax3.set_xlabel("时间 s", fontsize=9)
    ax3.set_ylabel("海拔 m", fontsize=9)
    ax3.set_title("高度：地面贴地、空中按爬升率\n（z 与位置矛盾 = 模型错了）",
                  fontsize=10)
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=7.5)
    ax3.tick_params(labelsize=8)

    if has_sub:
        ax4 = fig.add_subplot(1, ncol, 4)
        sub = sim.entities[ids["SUB_1"]].require("mover")
        ordered = float(sub.spec["cruise_depth"])
        dive = float(sub.spec["dive_rate"])
        ax4.plot(history["SUB_1"]["t"], history["SUB_1"]["z"],
                 color=TRACKS["SUB_1"][0], lw=1.8,
                 label=f"潜艇 z（定深 -{ordered:.0f} m）")
        ax4.axhline(-ordered, color=TRACKS["SUB_1"][0], ls=":", lw=1.2,
                    label="命令的定深")
        ax4.axhline(0.0, color="#888888", ls="-", lw=0.8, label="海平面")
        # 横轴只画下潜那一段：整段推演 6000 s 的话，30 s 的下潜被压成左边缘
        # 一根垂直线，"按速率"这件事就读不出来了。
        dive_s = ordered / dive if dive > 0.0 else 0.0
        window = max(60.0, dive_s * 4.0)
        last = history["SUB_1"]["t"][-1] if history["SUB_1"]["t"] else window
        ax4.set_xlim(0.0, min(window, last))
        ax4.axvline(dive_s, color=TRACKS["SUB_1"][0], ls=":", lw=1.0, alpha=0.7)
        ax4.annotate(f"{dive_s:.0f} s 到底", (dive_s, -ordered),
                     textcoords="offset points", xytext=(6, 6),
                     fontsize=8, color=TRACKS["SUB_1"][0])
        ax4.set_xlabel(f"时间 s（只画前 {min(window, last):.0f} s）", fontsize=9)
        ax4.set_ylabel("海拔 m", fontsize=9)
        ax4.set_title(f"潜艇：dive_rate {dive:.0f} m/s，"
                      f"{dive_s:.0f} s 到底", fontsize=10)
        ax4.grid(alpha=0.3)
        ax4.legend(fontsize=7.5, loc="center right")
        ax4.tick_params(labelsize=8)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"\n图已写入 {out}")


def run(scenario: str, who: list[str], seconds: float, plot: str) -> int:
    sim = Simulation.from_scenario_file(Path(scenario))
    sim.build()
    sim.initialize()

    names = [n for n in ALL_MOVERS if not who or n in who]
    if not names:
        print(f"--who 里没有想定中存在的载具；可选：{'、'.join(ALL_MOVERS)}")
        return 2
    ids = {name: sim.registry.by_name(name).entity_id for name in names
           if sim.registry.by_name(name) is not None}
    missing = [n for n in names if n not in ids]
    if missing:
        print(f"警告：想定里没有 {'、'.join(missing)}，跳过")
        names = list(ids)
    movers = {name: sim.entities[eid].require("mover") for name, eid in ids.items()}
    refs = {name: sim.store.cell_of(eid) for name, eid in ids.items()}

    goals: dict[str, Axial] = {}
    issue_orders(sim, movers, refs, goals)

    # 直线距离（对照"绕行比"用）：取 CAR_1（或唯一地面件）起点 → 目的地。
    straight_m = 0.0
    base = "CAR_1" if "CAR_1" in goals else ("CAR_2" if "CAR_2" in goals else None)
    if base:
        ref = refs[base]
        sx, sy = center_of(sim, ref, ref.axial)
        gx, gy = center_of(sim, ref, goals[base])
        straight_m = hypot(gx - sx, gy - sy)
        grid = sim.maps.grid(ref.zone_id, ref.layer)
        print(f"战区：{grid.size:.0f} m 格；{base} 目的地 "
              f"{goals[base]}（{TERRAIN_NAMES.get(grid.terrain(goals[base]), '?')}），"
              f"直线 {straight_m/1000:.2f} km")

    history = sample(sim, ids, seconds)
    report(sim, movers, history, straight_m)

    if plot and plot != "none":
        route_cells = None
        if "CAR_1" in movers:
            route = sim.nav.route(refs["CAR_1"], goals["CAR_1"],
                                  profile=movers["CAR_1"].travel_config())
            if route:
                route_cells = route.cells
        draw(sim, ids, refs, history, goals, route_cells, Path(plot))
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="机动组件单独测试（轨迹/速度/高度）")
    parser.add_argument("scenario", nargs="?", default=DEFAULT_SCENARIO,
                        help=f"想定文件（默认 {DEFAULT_SCENARIO}）")
    parser.add_argument("--who", default="",
                        help="只测这些载具，逗号分隔（默认全部）")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS,
                        help=f"推演时长 s（默认 {DEFAULT_SECONDS:.0f}）")
    parser.add_argument("--plot", default="check_mover.png",
                        help="输出图路径；传 none 关闭画图")
    args = parser.parse_args(argv)
    who = [w.strip() for w in args.who.split(",") if w.strip()]
    return run(args.scenario, who, args.seconds, args.plot)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
