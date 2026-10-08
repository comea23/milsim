"""态势投影演示：真实世界 vs 各方认知。

用途是**直观验证**一件事：某方看不到它没探测到的目标，而编队级聚合
不能泄漏真实信息（§3.9）。

    python tools/visualize_situation.py [输出路径]

三张面板：

1. **真实世界**（上帝视角）—— 全体实体的精确位置
2. **红方态势** —— 红方自己的部队 + 它探测到的蓝方（航迹位置，带误差）
3. **蓝方态势** —— 对称

在 2、3 面板里，真实位置用灰色空心圈标出并注明"仅复盘可见"。
它是叠加的分析信息，不属于被投影的态势本身。
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.services import Axial, EntityRegistry, EntityStore  # noqa: E402
from milsim.services.formation import FormationTree  # noqa: E402
from milsim.services.situation import (  # noqa: E402
    DigestOptions,
    SituationProjector,
    digest,
)
from milsim.services.store import LocalCellRef  # noqa: E402

CELL = LocalCellRef(1, 0, Axial(0, 0))

SIDE_COLOR = {"red": "#C0392B", "blue": "#2E6DA4"}
SIDE_LABEL = {"red": "红方", "blue": "蓝方"}

#: (编队名, 阵营, 基准位置 km, 各连相对偏移 km)
#:
#: 蓝方两个营刻意摆成"前卫 + 主力"两线。红方只探测到前卫，
#: 于是红方视角下编队质心落在**前卫**上，而真实质心在两线中间——
#: 这个偏差就是"先过滤再聚合"与"先聚合再过滤"的分水岭。
ORDER_OF_BATTLE = [
    ("RED_1BN", "red", (-42.0, -14.0), [(-2.5, -2.0), (2.5, -1.0), (-1.5, 2.0), (3.0, 2.5)]),
    ("RED_2BN", "red", (-38.0, 16.0), [(-2.0, -2.5), (2.0, -1.5), (-2.5, 2.0), (1.5, 2.5)]),
    ("BLUE_1BN", "blue", (40.0, -12.0),
     [(-3.0, -3.0), (3.0, -3.0), (-3.0, 9.0), (3.0, 9.0)]),
    ("BLUE_2BN", "blue", (44.0, 18.0),
     [(-3.0, -3.0), (3.0, -3.0), (-3.0, 9.0), (3.0, 9.0)]),
]


class _Entity:
    __slots__ = ("entity_id", "name")

    def __init__(self) -> None:
        self.entity_id = -1
        self.name = ""


def build_world():
    registry = EntityRegistry()
    store = EntityStore()
    tree = FormationTree()

    truth: dict[int, tuple[float, float]] = {}
    by_name: dict[str, int] = {}

    for unit, side, base, offsets in ORDER_OF_BATTLE:
        tree.add_unit(unit, side=side, echelon="营")
        for index, (dx, dy) in enumerate(offsets, start=1):
            name = f"{unit}_{index}"
            x = (base[0] + dx) * 1000.0
            y = (base[1] + dy) * 1000.0
            entity_id = registry.register(
                _Entity(), name=name, side=side, type_name="TANK"
            )
            store.add(entity_id, CELL)
            store.set_pose(entity_id, x, y, 0.0, 90.0, 8.0)
            tree.add_member(unit, entity_id)
            truth[entity_id] = (x, y)
            by_name[name] = entity_id

    # -- 红方的探测结果：只发现 BLUE_1BN 的**前卫两个连**，位置带误差 --
    # 主力（后线两个连）与整个 BLUE_2BN 完全没被发现——那是投影最容易泄漏的地方
    detections = [
        ("RED_1BN_1", "BLUE_1BN_1", 0.82, 900.0, -400.0),
        ("RED_1BN_2", "BLUE_1BN_2", 0.55, -700.0, 1100.0),
    ]
    now = 120_000_000                     # T+02:00
    for observer, target, quality, ex, ey in detections:
        tx, ty = truth[by_name[target]]
        store.add_contact(
            by_name[observer],
            by_name[target],
            quality=quality,
            detected_at=now - 45_000_000,   # 情报已滞后 45 秒
            x=tx + ex,
            y=ty + ey,
            z=0.0,
            range_m=((tx + ex) ** 2 + (ty + ey) ** 2) ** 0.5,
        )

    projector = SituationProjector(registry, store, tree)
    return registry, store, tree, projector, truth, by_name, now


def _setup_axes(ax, title: str, extent_km: float) -> None:
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=13, pad=10)
    ax.set_xlabel("东向 (km)", fontsize=10)
    ax.set_ylabel("南向 (km)", fontsize=10)
    ax.set_xlim(-extent_km, extent_km)
    ax.set_ylim(extent_km, -extent_km)      # 北在上
    ax.grid(True, alpha=0.2)
    ax.axhline(0.0, color="#888", linewidth=0.5, alpha=0.4)
    ax.axvline(0.0, color="#888", linewidth=0.5, alpha=0.4)
    ax.tick_params(labelsize=9)


def draw_truth(ax, situation, truth, *, ghost: bool = False) -> None:
    for info in situation.entities:
        x, y = info.x / 1000.0, info.y / 1000.0
        color = SIDE_COLOR[info.side]
        if ghost:
            ax.plot([x], [y], marker="o", markersize=5, markerfacecolor="none",
                    markeredgecolor="#999999", markeredgewidth=0.8,
                    linestyle="", zorder=2)
        else:
            ax.plot([x], [y], marker="o", markersize=8, color=color,
                    markeredgecolor="white", markeredgewidth=0.8,
                    linestyle="", zorder=4)

    for info in situation.formations:
        if not info.has_position:
            continue
        x, y = info.x / 1000.0, info.y / 1000.0
        color = SIDE_COLOR[info.side]
        radius = max(info.spread_m / 1000.0, 1.5)
        circle = plt.Circle((x, y), radius, fill=False, linewidth=1.2,
                            edgecolor=color, linestyle="-" if not ghost else "--",
                            alpha=0.35 if ghost else 0.85, zorder=3)
        ax.add_patch(circle)
        ax.plot([x], [y], marker="+", markersize=14, color=color,
                markeredgewidth=1.8, linestyle="", zorder=5)
        ax.annotate(info.name, (x, y), textcoords="offset points", xytext=(0, 14),
                    ha="center", fontsize=8, color=color)

        ax.text(x, y - radius - 3.0, _formation_facts(info), ha="center",
                fontsize=7.5, color=color, alpha=0.9)


def _formation_facts(info) -> str:
    """编队级的关键差异：编制数只有己方知道。"""
    if info.declared is None:
        return f"编制未知 · 置信度 {info.confidence:.2f}"
    return f"编队 {info.declared} · 战力 {info.strength:.0%}"


def _mark_centroid_gap(ax, situation, truth, by_name, unit: str) -> None:
    """标出"以为的质心"与"真实质心"的偏差。

    这是全图最想说明的一件事：红方以为敌人在前卫位置，真实主力还在后面。
    如果投影顺序写反了，红方视角的质心会精确落在真实质心上——
    一个它根本不该知道的数字。
    """
    info = next((f for f in situation.formations if f.name == unit), None)
    if info is None or not info.has_position:
        return

    members = [eid for name, eid in by_name.items() if name.startswith(unit)]
    if not members:
        return
    tx = sum(truth[e][0] for e in members) / len(members) / 1000.0
    ty = sum(truth[e][1] for e in members) / len(members) / 1000.0

    ax.plot([info.x / 1000.0, tx], [info.y / 1000.0, ty],
            linestyle=":", color="#A32D2D", linewidth=1.4, zorder=6)
    gap = ((info.x / 1000.0 - tx) ** 2 + (info.y / 1000.0 - ty) ** 2) ** 0.5
    ax.annotate(
        f"以为的质心 vs 真实质心：偏差 {gap:.1f} km",
        ((info.x / 1000.0 + tx) / 2, (info.y / 1000.0 + ty) / 2),
        textcoords="offset points", xytext=(0, -42), ha="center", fontsize=8,
        color="#A32D2D",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.9,
                  ec="#A32D2D", linewidth=0.5),
    )


def main() -> None:
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False

    registry, store, tree, projector, truth, by_name, now = build_world()
    extent = 62.0

    god = projector.project(at=now)
    red = projector.project(side="red", at=now)
    blue = projector.project(side="blue", at=now)

    fig, axes = plt.subplots(1, 3, figsize=(21, 7.6))

    _setup_axes(axes[0], "① 真实世界（上帝视角 · 复盘用）", extent)
    draw_truth(axes[0], god, truth)
    axes[0].text(0.02, 0.02,
                 f"实体 {len(god.entities)} 个，编队 4 个\n"
                 "全部为精确值，无误差无延迟",
                 transform=axes[0].transAxes, fontsize=9, va="bottom",
                 bbox=dict(boxstyle="round,pad=0.4", fc="white",
                           alpha=0.85, ec="#888", linewidth=0.5))

    _setup_axes(axes[1], "② 红方态势（宏观智能体实际读到的）", extent)
    draw_truth(axes[1], red, truth)
    _draw_truth_ghosts(axes[1], truth, by_name, "blue")
    _mark_centroid_gap(axes[1], red, truth, by_name, "BLUE_1BN")
    axes[1].text(0.02, 0.02,
                 f"实体 {len(red.entities)} 个，编队 {len(red.formations)} 个\n"
                 "BLUE_2BN 完全未出现（未被探测）\n"
                 "BLUE_1BN 只显示 2 个前卫，位置为航迹估计值",
                 transform=axes[1].transAxes, fontsize=9, va="bottom",
                 bbox=dict(boxstyle="round,pad=0.4", fc="white",
                           alpha=0.85, ec="#888", linewidth=0.5))

    _setup_axes(axes[2], "③ 蓝方态势（对称的视野限制）", extent)
    draw_truth(axes[2], blue, truth)
    _draw_truth_ghosts(axes[2], truth, by_name, "red")
    axes[2].text(0.02, 0.02,
                 f"实体 {len(blue.entities)} 个，编队 {len(blue.formations)} 个\n"
                 "蓝方没有任何红方航迹\n"
                 "（本演示只给了红方探测数据）",
                 transform=axes[2].transAxes, fontsize=9, va="bottom",
                 bbox=dict(boxstyle="round,pad=0.4", fc="white",
                           alpha=0.85, ec="#888", linewidth=0.5))

    legend = [
        plt.Line2D([], [], marker="o", color=SIDE_COLOR["red"], linestyle="",
                   markersize=8, label="红方实体"),
        plt.Line2D([], [], marker="o", color=SIDE_COLOR["blue"], linestyle="",
                   markersize=8, label="蓝方实体"),
        plt.Line2D([], [], marker="o", markerfacecolor="none",
                   markeredgecolor="#999", linestyle="", markersize=6,
                   label="真实位置（仅复盘可见）"),
        plt.Line2D([], [], marker="+", color="#555", linestyle="",
                   markersize=12, markeredgewidth=1.8, label="编队质心"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=4, fontsize=10,
               frameon=False, bbox_to_anchor=(0.5, 0.005))

    fig.suptitle("态势投影 —— 同一份 EntityStore，三个视角",
                 fontsize=17, y=0.97)
    fig.tight_layout(rect=(0, 0.05, 1, 0.95))

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path(__file__).resolve().parents[1] / "docs" / "images" / "situation.png"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120, facecolor="white")
    print(f"已保存：{out}")

    print()
    print("=" * 72)
    print("红方视角的实体清单（视野限制的直接证据）")
    print("=" * 72)
    print(f"  认识到的实体：{len(projector.known_ids('red', at=now))} / "
          f"场上真实 {len(registry.all_ids())}")
    missing = set(registry.all_ids()) - set(projector.known_ids("red", at=now))
    print(f"  不知道的实体：{sorted(registry.get(i).name for i in missing)}")

    print()
    print("=" * 72)
    print("给大模型看的态势摘要")
    print("=" * 72)
    print(digest(red, DigestOptions(center=(0.0, 0.0), radius_m=120_000.0)))


def _draw_truth_ghosts(ax, truth, by_name, side: str) -> None:
    """画该阵营的真实位置，供对照。仅复盘可见，不属于态势本身。"""
    for name, entity_id in by_name.items():
        if not name.startswith(side.upper()):
            continue
        x, y = truth[entity_id]
        ax.plot([x / 1000.0], [y / 1000.0], marker="o", markersize=5,
                markerfacecolor="none", markeredgecolor="#999999",
                markeredgewidth=0.8, linestyle="", zorder=1)


if __name__ == "__main__":
    main()
