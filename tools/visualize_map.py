"""地图效果可视化。

生成一张四联图，直观确认网格数据层是否正常：

  1. 地形类型 —— 有山有水有林，不是噪声糊成一团
  2. 高程 —— 起伏是否自然，量级是否合理
  3. 寻路 —— 是否真的绕开了水域而不是穿过去
  4. 通视 —— 是否体现了地形遮挡与地球曲率（远处应当被地平线切掉）

    python tools/visualize_map.py [输出路径]
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.services.map import (  # noqa: E402
    TERRAIN_FOREST,
    TERRAIN_HILL,
    TERRAIN_MOUNTAIN,
    TERRAIN_PLAIN,
    TERRAIN_URBAN,
    TERRAIN_WATER,
    Axial,
    HexGrid,
    check_line_of_sight,
    find_path,
    generate_terrain,
    offset_to_axial,
    ring,
)

# Windows 中文字体。缺席时退化成方块，不影响图形本身。
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

EXTENT = 64              # 网格边长（格）
SIZE = 1000.0            # 六边形边长（米）
SEED = 20260917
FEATURE_SCALE = 18_000.0 # 地形特征尺度（米）

#: 相邻格中心距 = 边长 × √3。绘图时必须用它换算比例，
#: 用边长会让画出来的六边形互相重叠。
CELL_GROUND_M = SIZE * np.sqrt(3.0)

TERRAIN_COLORS = {
    TERRAIN_WATER: "#2b6ca3",
    TERRAIN_PLAIN: "#cfe0a8",
    TERRAIN_FOREST: "#3d7a45",
    TERRAIN_HILL: "#c8a866",
    TERRAIN_MOUNTAIN: "#8d8175",
    TERRAIN_URBAN: "#b4453f",
}

TERRAIN_LABELS = {
    TERRAIN_WATER: "水域",
    TERRAIN_PLAIN: "平原",
    TERRAIN_FOREST: "林地",
    TERRAIN_HILL: "丘陵",
    TERRAIN_MOUNTAIN: "山地",
    TERRAIN_URBAN: "城镇",
}


# ---------------------------------------------------------------------------
# 几何
# ---------------------------------------------------------------------------

def axial_center(q: int, r: int, size: float = SIZE) -> tuple[float, float]:
    """pointy-top 六边形中心的世界坐标（米）。与 hex.py 的 to_world 一致。"""
    return size * np.sqrt(3.0) * (q + r * 0.5), size * 1.5 * r


def hex_polygons(centers: np.ndarray, size: float = SIZE) -> np.ndarray:
    """把网格中心点批量转成六边形顶点，形状 (N, 6, 2)。"""
    cx = centers[..., 0].ravel()
    cy = centers[..., 1].ravel()
    angles = np.radians(np.arange(6) * 60.0 + 30.0)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    verts = np.empty((cx.size, 6, 2), dtype=np.float64)
    verts[..., 0] = cx[:, None] + size * cos_a[None, :]
    verts[..., 1] = cy[:, None] + size * sin_a[None, :]
    return verts


def passable_near(grid: HexGrid, cell: Axial, radius: int = 8) -> Axial:
    """在附近找一个可通行单元。起点落在水里时用它兜底。"""
    if grid.is_passable(cell):
        return cell
    for k in range(1, radius + 1):
        for candidate in ring(cell, k):
            if grid.is_passable(candidate):
                return candidate
    return cell


# ---------------------------------------------------------------------------
# 场景
# ---------------------------------------------------------------------------

def build_scene() -> tuple[HexGrid, np.ndarray, np.ndarray, np.ndarray]:
    grid = HexGrid(zone_id=1, layer=0, size=SIZE)
    half = EXTENT // 2
    generate_terrain(
        grid,
        -half,
        -half,
        EXTENT,
        EXTENT,
        seed=SEED,
        feature_scale_m=FEATURE_SCALE,
        amplitude_m=1100.0,
        water_level_frac=0.30,
        forest_frac=0.30,
        urban_clusters=6,
    )

    shape = (EXTENT, EXTENT)
    terrain = np.zeros(shape, dtype=np.uint8)
    elevation = np.zeros(shape, dtype=np.float32)
    centers = np.zeros(shape + (2,), dtype=np.float64)

    for i, row in enumerate(range(-half, half)):
        for j, col in enumerate(range(-half, half)):
            cell = offset_to_axial(col, row)
            terrain[i, j] = grid.terrain(cell)
            elevation[i, j] = grid.elevation(cell)
            centers[i, j] = axial_center(cell.q, cell.r)

    return grid, terrain, elevation, centers


# ---------------------------------------------------------------------------
# 绘图
# ---------------------------------------------------------------------------

def _setup_axes(ax, title: str) -> None:
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=13, pad=10)
    ax.set_xticks([])
    ax.set_yticks([])
    # 世界坐标的 y 轴指向南（图像坐标习惯），反转后才"北在上"。
    # 不反转的话地图上下颠倒，方位角相关的判读会整体反过来。
    ax.invert_yaxis()
    for spine in ax.spines.values():
        spine.set_visible(False)


def draw_terrain(ax, terrain: np.ndarray, verts: np.ndarray) -> None:
    facecolors = [TERRAIN_COLORS[int(t)] for t in terrain.ravel()]
    ax.add_collection(
        PolyCollection(verts, facecolors=facecolors, edgecolors="none")
    )
    _autoscale(ax, verts)
    _setup_axes(ax, "地形类型")

    handles = [
        plt.Line2D([], [], marker="s", linestyle="", markersize=7,
                   markerfacecolor=TERRAIN_COLORS[t], markeredgecolor="none",
                   label=TERRAIN_LABELS[t])
        for t in sorted(TERRAIN_COLORS)
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.85)


def draw_elevation(ax, elevation: np.ndarray, verts: np.ndarray) -> None:
    values = elevation.ravel().astype(float)
    low, high = values.min(), values.max()
    span = max(high - low, 1.0)
    cmap = plt.get_cmap("terrain")
    facecolors = [cmap((v - low) / span) for v in values]

    ax.add_collection(
        PolyCollection(verts, facecolors=facecolors, edgecolors="none")
    )
    _autoscale(ax, verts)
    _setup_axes(ax, f"高程（{low:.0f} ~ {high:.0f} m）")


def draw_path(ax, terrain, elevation, verts, grid) -> None:
    draw_base(ax, terrain, verts)

    start = passable_near(grid, offset_to_axial(-EXTENT // 2 + 4, -6))
    goal = passable_near(grid, offset_to_axial(EXTENT // 2 - 5, 6))
    result = find_path(grid, start, goal)

    if result is not None:
        pts = np.array([axial_center(c.q, c.r) for c in result.cells])
        ax.plot(pts[:, 0], pts[:, 1], color="#111111", linewidth=2.6,
                solid_capstyle="round", zorder=5)
        ax.plot(pts[:, 0], pts[:, 1], color="#ffd400", linewidth=1.5,
                solid_capstyle="round", zorder=6)

        info = (
            f"{len(result.cells)} 格 / {len(result.cells)*CELL_GROUND_M/1000:.0f} km\n"
            f"{'分层' if result.used_hierarchy else '直接'}搜索，"
            f"展开 {result.expanded:,} 节点"
        )
        ax.text(0.02, 0.02, info, transform=ax.transAxes, fontsize=9,
                va="bottom", ha="left",
                bbox=dict(boxstyle="round,pad=0.4", fc="white", alpha=0.85,
                          ec="#888888", linewidth=0.5))
    else:
        ax.text(0.5, 0.5, "无可行路径", transform=ax.transAxes,
                fontsize=12, ha="center", va="center", color="#cc0000")

    for cell, color, label in ((start, "#00a0ff", "起点"), (goal, "#ff6600", "终点")):
        x, y = axial_center(cell.q, cell.r)
        ax.plot([x], [y], marker="o", markersize=9, color=color,
                markeredgecolor="white", markeredgewidth=1.4, zorder=7,
                linestyle="")
        ax.annotate(label, (x, y), textcoords="offset points", xytext=(9, 9),
                    fontsize=9, color=color, weight="bold", zorder=8)

    _setup_axes(ax, "A* 寻路（水域不可通行）")


def find_high_point(grid: HexGrid) -> Axial:
    """找地图上的最高点作为观察点。

    站在山脊上视野开阔，地平线效应才看得清楚；站在山谷里会被四周
    地形挡死，图上只剩一小块亮斑，什么也说明不了。
    """
    half = EXTENT // 2
    best = offset_to_axial(0, 0)
    best_h = -1e30
    for row in range(-half, half):
        for col in range(-half, half):
            cell = offset_to_axial(col, row)
            h = grid.elevation(cell)
            if h > best_h:
                best, best_h = cell, h
    return best


def draw_visibility(ax, terrain, verts, grid) -> None:
    """三色通视图：真实可见 / 只在漏算曲率时才"可见" / 完全不可见。

    中间那一色是这张图的价值所在——它直接量化了"忘记地球曲率会多看到
    多少东西"。军事上那些其实在地平线以下的低空目标，正是这样被误判成
    可探测的。
    """
    observer = find_high_point(grid)
    obs_height = 30.0            # 30 m 天线
    tgt_height = 2.0             # 2 m 高的目标

    shape = (EXTENT, EXTENT)
    with_curvature = np.zeros(shape, dtype=bool)
    without_curvature = np.zeros(shape, dtype=bool)
    half = EXTENT // 2

    for i, row in enumerate(range(-half, half)):
        for j, col in enumerate(range(-half, half)):
            target = offset_to_axial(col, row)
            if target == observer:
                with_curvature[i, j] = True
                without_curvature[i, j] = True
                continue
            without_curvature[i, j] = check_line_of_sight(
                grid, observer, target, obs_height, tgt_height,
                earth_curvature=False,
            ).visible
            with_curvature[i, j] = check_line_of_sight(
                grid, observer, target, obs_height, tgt_height,
                earth_curvature=True,
            ).visible

    CURVE_ONLY = "#ffd400"       # 只在漏算曲率时才"可见"
    facecolors = []
    for i in range(shape[0]):
        for j in range(shape[1]):
            base = TERRAIN_COLORS[int(terrain[i, j])]
            if with_curvature[i, j]:
                facecolors.append(base)
            elif without_curvature[i, j]:
                facecolors.append(CURVE_ONLY)
            else:
                facecolors.append(_dim(base, 0.30))

    ax.add_collection(
        PolyCollection(verts, facecolors=facecolors, edgecolors="none")
    )
    _autoscale(ax, verts)

    total = with_curvature.size
    real = with_curvature.sum() / total
    naive = without_curvature.sum() / total
    ox, oy = axial_center(observer.q, observer.r)
    ax.plot([ox], [oy], marker="*", markersize=15, color="#ff2200",
            markeredgecolor="white", markeredgewidth=1.2, zorder=7,
            linestyle="")
    ax.text(
        0.02, 0.02,
        f"观察点 30 m / 目标 2 m\n"
        f"真实可见 {real:.0%}\n"
        f"漏算曲率会误判为可见 {naive:.0%}（+{(naive-real)*100:.0f} 个百分点）",
        transform=ax.transAxes, fontsize=9, va="bottom", ha="left",
        bbox=dict(boxstyle="round,pad=0.4", fc="white", alpha=0.9,
                  ec="#888888", linewidth=0.5),
    )
    _setup_axes(ax, "通视覆盖（黄＝漏算地球曲率才会误判为可见）")


def draw_base(ax, terrain, verts) -> None:
    facecolors = [TERRAIN_COLORS[int(t)] for t in terrain.ravel()]
    ax.add_collection(
        PolyCollection(verts, facecolors=facecolors, edgecolors="none")
    )


def _dim(hex_color: str, factor: float) -> tuple[float, float, float]:
    """把十六进制颜色压暗。"""
    hex_color = hex_color.lstrip("#")
    rgb = np.array([int(hex_color[i:i + 2], 16) / 255.0 for i in (0, 2, 4)])
    return tuple(np.clip(rgb * factor + 0.06, 0.0, 1.0))


def _autoscale(ax, verts: np.ndarray) -> None:
    """PolyCollection 不自动调整坐标范围，必须手动设。"""
    ax.set_xlim(verts[..., 0].min() * 1.02, verts[..., 0].max() * 1.02)
    ax.set_ylim(verts[..., 1].min() * 1.02, verts[..., 1].max() * 1.02)


# ---------------------------------------------------------------------------

def build_resolution_scene(size: float, span_m: float):
    """在**同一地理范围**上生成不同分辨率的网格。

    覆盖范围按边长反比换算，所以三种分辨率看到的是同一片地方。
    地形由同一个种子的噪声按米生成，因此细节应当一致、只是采样密度不同——
    这正是"多层网格共享地形"要验证的性质。
    """
    cells = max(8, int(round(span_m / (size * np.sqrt(3.0)))))
    grid = HexGrid(zone_id=1, layer=0, size=size)
    half = cells // 2
    generate_terrain(
        grid, -half, -half, cells, cells,
        seed=SEED * 7,
        feature_scale_m=FEATURE_SCALE,
        amplitude_m=1100.0,
        water_level_frac=0.30,
        forest_frac=0.30,
        urban_clusters=2,
    )

    shape = (cells, cells)
    terrain = np.zeros(shape, dtype=np.uint8)
    centers = np.zeros(shape + (2,), dtype=np.float64)
    for i, row in enumerate(range(-half, half)):
        for j, col in enumerate(range(-half, half)):
            cell = offset_to_axial(col, row)
            terrain[i, j] = grid.terrain(cell)
            centers[i, j] = axial_center(cell.q, cell.r, size)
    return grid, terrain, centers


def draw_resolution_comparison(out: Path) -> None:
    """三种分辨率覆盖同一片区域。"""
    span_m = 60_000.0
    sizes = [500.0, 1000.0, 2000.0]

    fig, axes = plt.subplots(1, 3, figsize=(18, 7))

    for ax, size in zip(axes, sizes):
        grid, terrain, centers = build_resolution_scene(size, span_m)
        verts = hex_polygons(centers, size)
        draw_base(ax, terrain, verts)
        _autoscale(ax, verts)

        cells = terrain.size
        _setup_axes(
            ax,
            f"边长 {size:.0f} m　{cells:,} 格\n"
            f"格距 {size * np.sqrt(3):.0f} m",
        )
        print(f"  {size:>6.0f} m 边长 → {cells:>6,} 格 "
              f"（同一 {span_m/1000:.0f} km × {span_m/1000:.0f} km 区域）")

    fig.suptitle(
        f"多分辨率对比 —— 同一片 {span_m/1000:.0f} km 区域，格距 = 边长 × √3",
        fontsize=16, y=0.98,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=130, facecolor="white")
    print(f"已保存：{out}")


def main() -> None:
    grid, terrain, elevation, centers = build_scene()
    verts = hex_polygons(centers)

    water = float((terrain == TERRAIN_WATER).mean())
    print(f"地形：{EXTENT}×{EXTENT} 格，边长 {SIZE:.0f} m "
          f"（相邻格中心距 {CELL_GROUND_M:.0f} m）")
    print(f"      覆盖约 {EXTENT * CELL_GROUND_M / 1000:.0f} km × "
          f"{EXTENT * CELL_GROUND_M / 1000:.0f} km")
    print(f"      高程 {elevation.min():.0f} ~ {elevation.max():.0f} m，"
          f"水域 {water:.1%}")

    fig, axes = plt.subplots(2, 2, figsize=(16, 15))
    draw_terrain(axes[0, 0], terrain, verts)
    draw_elevation(axes[0, 1], elevation, verts)
    draw_path(axes[1, 0], terrain, elevation, verts, grid)
    draw_visibility(axes[1, 1], terrain, verts, grid)

    fig.suptitle("milsim 网格数据层 —— 地形 / 高程 / 寻路 / 通视",
                 fontsize=17, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.985))

    out_dir = Path(__file__).resolve().parents[1] / "docs" / "images"
    out_dir.mkdir(parents=True, exist_ok=True)

    overview = Path(sys.argv[1]) if len(sys.argv) > 1 else out_dir / "map_overview.png"
    overview.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(overview, dpi=130, facecolor="white")
    print(f"已保存：{overview}")

    print("\n多分辨率对比：")
    draw_resolution_comparison(out_dir / "map_resolution.png")


if __name__ == "__main__":
    main()
