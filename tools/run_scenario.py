"""跑一份想定：装配 → 推演 → 打印结果（可选画轨迹图）。

用法::

    python tools/run_scenario.py scenarios/patrol.txt
    python tools/run_scenario.py scenarios/patrol.txt --seconds 300
    python tools/run_scenario.py scenarios/patrol.txt --seconds 300 --plot trail.png

    # 型号参数来自参数库（可重复给多个库）
    python tools/run_scenario.py scenarios/library_demo.txt \
        --library library/demo_models.db

机动件（``GROUND_MOVER``）来自框架的参考实现；感知件还是
``tools/demo_components.py`` 里那个演示用的最简雷达——真正的感知模型
（雷达方程 + 通视 + 航迹管理）在 M4a 的下一小节。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import demo_components  # noqa: E402
from milsim.errors import MilsimError  # noqa: E402
from milsim.models import register_framework  # noqa: E402
from milsim.services.input.lexer import LexError, format_error  # noqa: E402
from milsim.services.map import Axial  # noqa: E402
from milsim.services.type_registry import (  # noqa: E402
    ComponentRegistry,
    PlatformRegistry,
)
from milsim.simulation import Simulation  # noqa: E402


def parse_args(argv: list[str]) -> tuple[str, float, str | None, list[str]]:
    if len(argv) < 2:
        print(__doc__.strip(), file=sys.stderr)
        raise SystemExit(2)

    path = argv[1]
    seconds = 120.0
    plot: str | None = None
    #: 参数库**必须显式给**。默认去某个固定路径找，会让同一份想定在不同
    #: 机器上读到不同参数，而且不报错——那是"结果对不上"最难查的一种。
    libraries: list[str] = []

    rest = argv[2:]
    index = 0
    while index < len(rest):
        flag = rest[index]
        if flag == "--seconds" and index + 1 < len(rest):
            seconds = float(rest[index + 1])
            index += 2
        elif flag == "--plot" and index + 1 < len(rest):
            plot = rest[index + 1]
            index += 2
        elif flag == "--library" and index + 1 < len(rest):
            libraries.append(rest[index + 1])
            index += 2
        else:
            print(f"无法识别的参数：{flag}", file=sys.stderr)
            raise SystemExit(2)
    return path, seconds, plot, libraries


def main(argv: list[str]) -> int:
    path, seconds, plot_path, libraries = parse_args(argv)
    file = Path(path)
    source = file.read_text(encoding="utf-8")

    # 用独立注册表而不是全局表：跑想定不该被别处注册过的东西影响。
    # 框架自带的参考实现要**显式**装进来——独立注册表里一开始什么都没有，
    # 而想定里写的 GROUND_MOVER 之类正是框架的组件。
    components = ComponentRegistry()
    platforms = PlatformRegistry()
    register_framework(components, platforms)
    demo_components.register_all(components)

    sim = Simulation.from_scenario(
        source,
        name=file.name,
        components=components,
        platforms=platforms,
        library=libraries or None,
    )

    try:
        sim.build()
    except MilsimError as exc:
        # 组件参数不匹配、类型找不到、战区坐标越界——都在这里统一呈现，
        # 不让想定作者看到栈回溯
        if isinstance(exc, (LexError,)) or hasattr(exc, "line"):
            print(format_error(exc, path=file.name, source=source), file=sys.stderr)
        else:
            print(f"{file.name}: {exc}", file=sys.stderr)
        return 1

    print(sim.spec.summary())
    print(f"已加载战区：{'、'.join(z.spec.name for z in sim.maps.active_zones) or '（无）'}")
    print()

    sim.initialize()

    positions: dict[str, list[tuple[float, float]]] = {
        e.name: [_xy(sim, e.entity_id)] for e in sim.entities.values()
    }
    for entity in sim.entities.values():
        radar = entity.component("sensor")
        if radar is not None:
            radar.contacts_seen = []

    # 分段推进并记录轨迹，这样画图时能看到路径而不是只有两个端点
    steps = 20
    for _ in range(steps):
        sim.run_for(int(seconds / steps * 1_000_000))
        for entity in sim.entities.values():
            positions[entity.name].append(_xy(sim, entity.entity_id))

    report = sim.shutdown()

    print("实体：")
    for entity in sim.entities.values():
        trail = positions[entity.name]
        dx = trail[-1][0] - trail[0][0]
        dy = trail[-1][1] - trail[0][1]
        distance = sum(
            ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
            for a, b in zip(trail, trail[1:])
        )
        parts = "、".join(entity.slots())
        line = (
            f"  {entity.name:<8} {parts:<20} 行进 {distance/1000:>7.2f} km"
            f"（净位移 {dx/1000:>+7.2f}, {dy/1000:>+7.2f} km）"
        )
        radar = entity.component("sensor")
        if radar is not None:
            line += (
                f"  波束推进 {radar.ticks} 拍 / 转过 {radar.sweeps:.2f} 圈"
                f"（{radar.mode}）"
                f"  航迹 {len(sim.store.contacts_of(entity.entity_id))} 条"
            )
        print(line)

    print()
    print(report.summary())

    for entity in sim.entities.values():
        contacts = sim.store.contacts_of(entity.entity_id)
        if not contacts:
            continue
        print(f"  {entity.name} 的航迹：")
        for contact in contacts:
            target = sim.entities.get(contact.target_id)
            name = target.name if target else f"#{contact.target_id}"
            print(
                f"    → {name:<8} 距离 {contact.range_m/1000:>6.2f} km  "
                f"方位 {contact.bearing_deg:>5.1f}°  "
                f"质量 {contact.quality:.2f}  于 {contact.detected_at/1e6:.0f} s"
            )

    if plot_path:
        _plot(positions, sim, Path(plot_path))

    return 0


def _xy(sim: Simulation, entity_id: int) -> tuple[float, float]:
    position = sim.store.position_of(entity_id)
    return (position[0], position[1]) if position else (0.0, 0.0)


def _plot(positions: dict[str, list[tuple[float, float]]], sim: Simulation, out: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("（未安装 matplotlib，跳过绘图）", file=sys.stderr)
        return

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False

    figure, axes = plt.subplots(figsize=(9, 8))

    # 先画战区边界——它给出"这块地在哪、多大"的参照
    for zone in sim.maps.active_zones:
        centre_x, centre_y = zone.frame.to_world(Axial(0, 0))
        circle = plt.Circle(
            (centre_x / 1000.0, centre_y / 1000.0),
            zone.spec.radius_m / 1000.0,
            fill=False,
            edgecolor="gray",
            linestyle="--",
            linewidth=1.0,
            alpha=0.6,
            label=f"战区 {zone.spec.name}",
        )
        axes.add_patch(circle)

    for entity in sim.entities.values():
        trail = positions[entity.name]
        side = sim.registry.side_of(entity.entity_id)
        color = "red" if side == "red" else "blue"
        xs = [p[0] / 1000.0 for p in trail]
        ys = [p[1] / 1000.0 for p in trail]
        axes.plot(xs, ys, "-", color=color, linewidth=1.6, label=entity.name)
        axes.plot(xs[0], ys[0], "o", color=color, markersize=6)
        axes.plot(xs[-1], ys[-1], "s", color=color, markersize=6)

    # 标出航迹连线，直观看"谁发现了谁"
    for entity in sim.entities.values():
        me = sim.store.position_of(entity.entity_id)
        for contact in sim.store.contacts_of(entity.entity_id):
            target = sim.store.position_of(contact.target_id)
            if not me or not target:
                continue
            axes.annotate(
                "",
                xy=(target[0] / 1000.0, target[1] / 1000.0),
                xytext=(me[0] / 1000.0, me[1] / 1000.0),
                arrowprops=dict(arrowstyle="->", color="gray", alpha=0.5, linestyle=":"),
            )

    # 坐标范围要显式设定。只沿一条直线航行时，另一轴的坐标是 ~1e-15 级的
    # 浮点残差，matplotlib 会据此把轴缩放到荒谬的尺度（图上出现 "1e-15"），
    # 轨迹被压成一条线。给一个最小跨度，让它有个合理的画面。
    xs = [point[0] for trail in positions.values() for point in trail]
    ys = [point[1] for trail in positions.values() for point in trail]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 2_000.0)
    centre_x = (max(xs) + min(xs)) / 2.0
    centre_y = (max(ys) + min(ys)) / 2.0
    axes.set_xlim((centre_x - span * 0.6) / 1000.0, (centre_x + span * 0.6) / 1000.0)
    axes.set_ylim((centre_y - span * 0.6) / 1000.0, (centre_y + span * 0.6) / 1000.0)

    axes.set_aspect("equal")
    axes.set_xlabel("东向 (km)")
    axes.set_ylabel("南向 (km)")
    axes.set_title(f"{sim.spec.settings.name or '推演'} — 轨迹与探测")
    axes.grid(True, alpha=0.25)
    axes.axhline(0.0, color="black", linewidth=0.5, alpha=0.4)
    axes.legend()
    figure.tight_layout()
    figure.savefig(out, dpi=130, facecolor="white")
    print(f"\n轨迹图已保存：{out}")


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
