"""机动组件的单独体检：下达目的地，看轨迹、速度、高度与格归属。

与 ``demo_mover.py`` 的分工：demo 讲"模型为什么长这样"，本工具只回答
"装配出来的机动件动没动、怎么动、动得对不对"。复用同一份想定
``scenarios/mover_demo.txt``（不另建想定），数字全部来自真实装配：
地形按需生成、地面/水面/水下走 A*、空中走直线、位置由引擎逐帧推出。

用法::

    python tools/check_mover.py                          # 全部载具 + 表
    python tools/check_mover.py --plot tools/check_mover.png
    python tools/check_mover.py --who CAR_1,JET_1 --seconds 3000

输出两样：

1. **数值表**——里程、平均/峰值速度、用时、到达状态，外加**跨格次数**：
   每个采样拍数一次 ``store.cell_of``，与上一拍不同就 +1。它 > 0 直接
   证明"机动跨过本格边界后格归属会被同步更新"（空间索引不丢目标）；
2. **四联图**（可选）——俯视轨迹 + 地形底图 / 速度-时间 / 高度-时间 /
   潜艇深度单独一格（60 m 的下潜挤进 4000 m 的高度轴就什么都看不见）。
"""

from __future__ import annotations

import argparse
import sys
from math import cos, radians, sin
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent
_ROOT = _TOOLS_DIR.parent
for _p in (str(_ROOT / "src"), str(_TOOLS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from demo_mover import SAMPLE_US, TERRAIN_PALETTE, _pad, center_of, choose_goal  # noqa: E402
from milsim.services.map import Axial, TERRAIN_NAMES  # noqa: E402
from milsim.simulation import Simulation  # noqa: E402

#: 想定里的全部机动件。缺哪个跳哪个——工具不该绑定一份会演化的想定。
ALL_UNITS = ("CAR_1", "CAR_2", "JET_1", "SUB_1", "SHIP_1")

#: 走 A* 的那几件（地面 / 水面 / 水下）：目的地要挑"走得通"的格。
ASTER_UNITS = ("CAR_1", "CAR_2", "SUB_1", "SHIP_1")

#: 轨迹配色（与 demo_mover 的 TRACKS 同色系，补上舰船）。
COLORS = {
    "CAR_1": "#c0392b",
    "CAR_2": "#d9822b",
    "JET_1": "#2c6fbb",
    "SHIP_1": "#6a4c93",
    "SUB_1": "#1f7a6b",
}


def dispatch(sim: Simulation, ids: dict, movers: dict) -> dict:
    """下达目的地：A* 族用 ``choose_goal`` 挑可达格，飞机直线飞向 CAR_1 的目的地。

    返回 ``{name: (goal_axial | None, 目的地世界坐标)}``，画图要标目的地。
    """
    goals: dict[str, tuple[object | None, tuple[float, float]]] = {}
    for name in ASTER_UNITS:
        if name not in movers:
            continue
        eid = ids[name]
        ref = sim.store.cell_of(eid)
        try:
            goal, _route = choose_goal(
                sim, ref, ref.axial, movers[name].travel_config()
            )
        except SystemExit:
            # 挑不到可达目的地（近岸水浅/被地形围死）不是工具的错——
            # 跳过这件，其余照测。
            print(f"警告：{name} 周边挑不到可达目的地，跳过该件的 A* 测试")
            continue
        if not movers[name].move_to_cell(goal):
            print(f"警告：{name} 没有路线——{movers[name].blocked_reason()}")
        goals[name] = (goal, center_of(sim, ref, goal))

    jet = movers.get("JET_1")
    if jet is not None:
        if "CAR_1" in goals:
            gx, gy = goals["CAR_1"][1]
        else:
            px, py = sim.store.pose_of(ids["JET_1"])[:2]
            gx, gy = px, py + 15_000.0
        jet.move_to_point(gx, gy, float(jet.spec["cruise_altitude"]))
        goals["JET_1"] = (None, (gx, gy))
    return goals


def sample(sim: Simulation, ids: dict, seconds: float) -> tuple[dict, dict]:
    """逐采样拍记录 ``(t, x, y, z, v)``，并数**跨格次数**。

    跨格次数的口径：把本拍的 ``store.cell_of`` 与上一拍比，不同就 +1。
    采样粒度（5 s）比机动件的更新周期粗，所以这是**下界**——真实跨格
    只会更多，不会更少；它 > 0 就足以证明格归属在跟着机动更新。
    """
    history = {n: {"t": [], "x": [], "y": [], "z": [], "v": []} for n in ids}
    crossings = {n: 0 for n in ids}
    last_cell = {n: sim.store.cell_of(eid) for n, eid in ids.items()}

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
            cell = sim.store.cell_of(eid)
            if cell is not None and cell != last_cell[name]:
                crossings[name] += 1
                last_cell[name] = cell
    return history, crossings


def report(sim: Simulation, ids: dict, movers: dict, history: dict,
           crossings: dict) -> None:
    ref = sim.store.cell_of(ids[next(iter(ids))])
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    print(f"战区：{grid.size:.0f} m 格；采样间隔 {SAMPLE_US / 1_000_000:.0f} s\n")
    print(f"{_pad('载具', 8)}{_pad('里程 km', 9, right=True)}"
          f"{_pad('平均 m/s', 10, right=True)}{_pad('峰值 m/s', 10, right=True)}"
          f"{_pad('用时 s', 9, right=True)}{_pad('跨格', 6, right=True)}"
          f"{_pad('到达', 6, right=True)}{_pad('状态', 8, right=True)}")
    for name in history:
        mover = movers[name]
        row = history[name]
        travelled = mover.travelled_m()
        span = mover.elapsed_s()
        avg = travelled / span if span > 0.0 else 0.0
        peak = max(row["v"]) if row["v"] else 0.0
        state = "走不通" if mover.blocked() else ("已到" if mover.arrived() else "在途")
        print(f"{_pad(name, 8)}{travelled / 1000:>9.2f}{avg:>10.2f}{peak:>10.2f}"
              f"{span:>9.1f}{crossings[name]:>6d}"
              f"{_pad('是' if mover.arrived() else '否', 6, right=True)}"
              f"{_pad(state, 8, right=True)}")
    print("\n跨格 > 0 = 机动跨过本格边界后，store.cell_of 与空间索引跟着更新了"
          "（5 s 才采一拍，这只是下界）")


def draw(sim: Simulation, ref, ids: dict, movers: dict, history: dict,
         goals: dict, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Patch

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    plane_names = [n for n in ("CAR_1", "CAR_2", "JET_1", "SHIP_1") if n in history]
    grid = sim.maps.grid(ref.zone_id, ref.layer)
    size = grid.size

    # 地形底图：视野按实际走过的范围定（与 demo_mover 同一招）。
    xs = [v for n in plane_names for v in history[n]["x"]]
    ys = [v for n in plane_names for v in history[n]["y"]]
    if xs:
        margin = 4.0 * size * 1.732
        x0, x1 = min(xs) - margin, max(xs) + margin
        y0, y1 = min(ys) - margin, max(ys) + margin
        zone = sim.maps.zone(ref.zone_id)
        corners = [zone.frame.from_world(x, y)
                   for x, y in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
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

    fig = plt.figure(figsize=(19.0, 5.2), dpi=110)

    ax = fig.add_subplot(1, 4, 1)
    if polys:
        ax.add_collection(PolyCollection(polys, facecolors=colors, edgecolors="none"))
    for name in plane_names:
        row = history[name]
        ax.plot(row["x"], row["y"], lw=2.0, color=COLORS[name], label=name)
        if name in goals and goals[name][0] is not None:
            gx, gy = goals[name][1]
            ax.plot(gx, gy, "*", color=COLORS[name], ms=12)
    ax.autoscale_view()
    ax.set_aspect("equal")
    ax.set_title(f"轨迹与地形（{size:.0f} m 格，* = 目的地）", fontsize=10)
    handles = [Patch(facecolor=TERRAIN_PALETTE[n], label=n)
               for n in sorted(present) if n in TERRAIN_PALETTE]
    handles.extend(plt.Line2D([], [], color=COLORS[n], lw=2.0, label=n)
                   for n in plane_names)
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.06),
              fontsize=7.5, frameon=False, ncol=3)
    ax.tick_params(labelsize=8)

    ax2 = fig.add_subplot(1, 4, 2)
    for name in history:
        ax2.plot(history[name]["t"], history[name]["v"],
                 lw=1.5, color=COLORS[name], label=name)
    ax2.set_xlabel("时间 s", fontsize=9)
    ax2.set_ylabel("速度 m/s", fontsize=9)
    ax2.set_title("速度（分段 = 穿过不同地形代价）", fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    ax2.tick_params(labelsize=8)

    ax3 = fig.add_subplot(1, 4, 3)
    for name in plane_names:
        ax3.plot(history[name]["t"], history[name]["z"],
                 lw=1.5, color=COLORS[name], label=name)
    ax3.set_xlabel("时间 s", fontsize=9)
    ax3.set_ylabel("海拔 m", fontsize=9)
    ax3.set_title("高度（地面贴地 / 空中按爬升率）", fontsize=10)
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=8)
    ax3.tick_params(labelsize=8)

    ax4 = fig.add_subplot(1, 4, 4)
    if "SUB_1" in history:
        ordered = float(movers["SUB_1"].spec["cruise_depth"])
        ax4.plot(history["SUB_1"]["t"], history["SUB_1"]["z"],
                 lw=1.8, color=COLORS["SUB_1"], label="SUB_1 深度")
        ax4.axhline(-ordered, color=COLORS["SUB_1"], ls=":", lw=1.2,
                    label=f"命令定深 -{ordered:.0f} m")
        ax4.axhline(0.0, color="#888888", lw=0.8, label="海平面")
        ax4.legend(fontsize=8)
    else:
        ax4.text(0.5, 0.5, "（想定里没有 SUB_1）", ha="center", va="center",
                 transform=ax4.transAxes)
    ax4.set_xlabel("时间 s", fontsize=9)
    ax4.set_ylabel("海拔 m", fontsize=9)
    ax4.set_title("潜艇深度（单独一格才看得出下潜速率）", fontsize=10)
    ax4.grid(alpha=0.3)
    ax4.tick_params(labelsize=8)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"图已写入 {out}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="机动组件单独体检")
    parser.add_argument("scenario", nargs="?", default="scenarios/mover_demo.txt",
                        help="想定文件（默认 scenarios/mover_demo.txt）")
    parser.add_argument("--seconds", type=float, default=6000.0,
                        help="推演时长 s（默认 6000）")
    parser.add_argument("--who", default="",
                        help="只测这些载具，逗号分隔（默认全部）")
    parser.add_argument("--plot", default="",
                        help="出图路径（不给就不画）")
    args = parser.parse_args(argv)

    wanted = ([w.strip() for w in args.who.split(",") if w.strip()]
              if args.who else list(ALL_UNITS))
    unknown = [w for w in wanted if w not in ALL_UNITS]
    if unknown:
        print(f"不认识的载具名：{'、'.join(unknown)}（可选：{'、'.join(ALL_UNITS)}）")
        return 2

    scenario = Path(args.scenario)
    if not scenario.is_absolute() and not scenario.exists():
        scenario = _ROOT / scenario
    sim = Simulation.from_scenario_file(scenario)
    sim.build()
    sim.initialize()

    names = tuple(n for n in wanted if sim.registry.by_name(n) is not None)
    if not names:
        print("想定里一个要测的载具都没有")
        return 2
    ids = {n: sim.registry.by_name(n).entity_id for n in names}
    movers = {n: sim.entities[eid].require("mover") for n, eid in ids.items()}

    goals = dispatch(sim, ids, movers)
    history, crossings = sample(sim, ids, args.seconds)
    report(sim, ids, movers, history, crossings)

    if args.plot:
        ref = sim.store.cell_of(ids[names[0]])
        draw(sim, ref, ids, movers, history, goals, Path(args.plot))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
