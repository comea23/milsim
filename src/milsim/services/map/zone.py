"""战区管理与坐标路由。

架构
----
想定是**跨洲**的，但地图**按需加载**——只加载有目标的区域。所以地图服务
分成两层，职责完全分开：

    全球层 (GlobalLayer)     常驻。H3 球面六边形，9.85 km 粒度，
                             负责跨区机动、全局态势、战区索引
    局部层 (Zone → LocalFrame) 按需。平面六边形，100 m ~ 2 km 精确分辨率，
                             负责战术细节：通视、逐格机动、火力范围

一个实体在跨洲航渡时用全球层坐标，进入战区后切到局部坐标，
出场后再切回全球层。这个切换对模型层是透明的——模型只调 ``locate()``。

坐标路由为什么是 O(1)
---------------------
战区激活时会把自己覆盖的全球层单元登记到索引表里（``cell → zone_id``）。
之后定位一个经纬度，只需一次 H3 查询加一次字典查找，不需要遍历战区列表。

战区不能划太大
--------------
AED 投影的误差随离锚点的距离增长。500 km 半径内可放心使用，
超过 1000 km 建议拆成多个战区——这也是 ``ZoneSpec.radius_m`` 有上限的原因。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterator

from .grid import LEVEL_SURFACE, HexGrid
from .hex import COORD_LIMIT, Axial, LocalFrame, axial_to_offset, decode, encode
from .h3layer import DEFAULT_RESOLUTION, GlobalLayer, great_circle_m

#: 单个战区的最大半径。超过这个值 AED 投影的切向变形开始明显，
#: 应该拆成多个战区。
MAX_ZONE_RADIUS_M = 1_000_000.0

#: 单个战区的最小半径。太小的话连一个网格单元都放不下。
MIN_ZONE_RADIUS_M = 1_000.0

#: 网格边长的允许范围。100 m 到 2 km 是需求定下的跨度（§0 决策 1），
#: 再细则内存吃不消，再粗则战术精度不够。想定层也按这对常量校验，
#: 避免"想定检查通过、装配时才炸"。
MIN_CELL_SIZE_M = 100.0
MAX_CELL_SIZE_M = 2_000.0

#: 战区地形的来源。**白名单**——想定里写了别的值必须在装配时报错，
#: 不能静默忽略（否则会让人以为"写了 terrain 就有地形"）。
#:
#: - ``procedural``：程序化**懒生成**（默认）。实体走到哪生成哪一块。
#: - ``flat``：全平地形。只用于算法验证的边界情况。
#: - ``none``：不生成。读到的都是默认值（可通行的平原、海平面高程）。
TERRAIN_SOURCES = frozenset({"procedural", "flat", "none"})


@dataclass(slots=True)
class ZoneSpec:
    """战区的静态定义，想定里声明，激活前不占任何资源。"""

    zone_id: int
    name: str
    lat: float
    lng: float
    radius_m: float
    resolution_m: float = 1000.0
    #: 需要建立几层网格。层号越小越精细。第一层由 resolution_m 决定，
    #: 后续层按同样的边长逐层加倍。
    layer_count: int = 1
    #: 地形来源，取值见 :data:`TERRAIN_SOURCES`。
    terrain: str = "procedural"
    #: 地形噪声种子。**必须参与生成结果的确定性**——同一块任何时候生成
    #: 都得一样，否则回放与"同种子两次一致"的承诺就崩了。
    seed: int = 0

    def __post_init__(self) -> None:
        if not 0 <= self.zone_id <= 0xFFF:
            raise ValueError(f"zone_id={self.zone_id} 超出 12 位可编码范围 0..4095")
        if not MIN_ZONE_RADIUS_M <= self.radius_m <= MAX_ZONE_RADIUS_M:
            raise ValueError(
                f"战区半径 {self.radius_m/1000:.1f} km 超出 "
                f"{MIN_ZONE_RADIUS_M/1000:.0f}~{MAX_ZONE_RADIUS_M/1000:.0f} km 的合理范围"
            )
        if not MIN_CELL_SIZE_M <= self.resolution_m <= MAX_CELL_SIZE_M:
            raise ValueError(
                f"网格边长 {self.resolution_m} m 应在 "
                f"{MIN_CELL_SIZE_M:.0f} ~ {MAX_CELL_SIZE_M:.0f} m 之间"
            )
        if not 1 <= self.layer_count <= 8:
            raise ValueError(f"层数 {self.layer_count} 应在 1 ~ 8 之间")
        if self.terrain not in TERRAIN_SOURCES:
            raise ValueError(
                f"未知的地形来源 {self.terrain!r}，可用："
                + " / ".join(sorted(TERRAIN_SOURCES))
            )

    def resolution_of_layer(self, layer: int) -> float:
        """第 layer 层的网格边长。每层翻倍，1000 → 2000 → 4000..."""
        return self.resolution_m * (2 ** layer)

    def cell_span(self) -> int:
        """半轴上的格子数。用于估算网格规模。"""
        return int(math.ceil(self.radius_m / self.resolution_m)) + 2

    def estimated_cells(self, layer: int = 0) -> int:
        """第 layer 层的单元总数估算（六边形密铺，面积比 = 边长比²）。"""
        # 六边形面积 = (3√3/2) * size²，圆面积 = πr²
        hex_area = 2.598076211 * self.resolution_of_layer(layer) ** 2
        return int(math.pi * self.radius_m ** 2 / hex_area)


@dataclass(slots=True)
class Location:
    """一次定位的结果。"""

    lat: float
    lng: float
    global_cell: int
    zone: "Zone | None" = None
    axial: Axial | None = None

    @property
    def is_local(self) -> bool:
        """是否落在某个已激活的战区内（即拥有精确的局部坐标）。"""
        return self.zone is not None

    @property
    def zone_id(self) -> int:
        return self.zone.spec.zone_id if self.zone else -1

    def cell(self, layer: int = 0) -> int:
        """当前所在的网格单元标识。在战区外用全球层单元，在战区内用局部单元。"""
        if self.zone is None:
            return self.global_cell
        assert self.axial is not None
        return encode(self.axial.q, self.axial.r, self.zone.spec.zone_id, layer)

    def __repr__(self) -> str:
        if self.zone is None:
            return f"<Location ({self.lat:.5f},{self.lng:.5f}) 全球层>"
        return (
            f"<Location ({self.lat:.5f},{self.lng:.5f}) "
            f"战区{self.zone.spec.name} 局部{tuple(self.axial)}>"
        )


class Zone:
    """一个已激活的战区。持有投影坐标系与**按需生成**的网格数据。

    惰性生成（§3.11.5）：激活时一个块都不建。实体落到哪一格，才生成包含
    那一块的范围。所以"声明 20 个战区只用到 3 个"之外，还有更细一层——
    用到的那个战区里，也只生成真正被踩到的块。
    """

    __slots__ = ("spec", "frame", "grids", "chunk_loader", "_cell_bounds", "_loaded")

    def __init__(self, spec: ZoneSpec) -> None:
        self.spec = spec
        self.frame = LocalFrame(spec.lat, spec.lng, spec.resolution_m)
        #: 分辨率层 → HexGrid。缺层时由 :meth:`grid` 现建（空网格，不生成地形）。
        self.grids: dict[int, HexGrid] = {}
        #: 分块生成器，由装配层注入：``(grid, cx, cy, cz) -> None``。
        #: ``None`` 表示这个战区不做生成（``terrain none`` 或没装 loader）。
        self.chunk_loader: Callable[[HexGrid, int, int, int], None] | None = None
        #: 已生成的 ``(layer, level, cx, cy)``。按**分辨率层**分别记账——
        #: 同一个空间位置在不同 LOD 上是不同的块。
        self._loaded: set[tuple[int, int, int, int]] = set()
        self._cell_bounds = self._compute_bounds()

    # -- 网格 --------------------------------------------------------------

    def grid(self, layer: int = 0) -> HexGrid:
        """取（必要时创建）某一**分辨率层**的网格。

        只创建空网格，**不生成地形**——生成走 :meth:`ensure_loaded`。
        """
        grid = self.grids.get(layer)
        if grid is None:
            grid = HexGrid(
                zone_id=self.spec.zone_id,
                layer=layer,
                size=self.spec.resolution_of_layer(layer),
            )
            self.grids[layer] = grid
        return grid

    def ensure_loaded(
        self, cell: Axial, layer: int = 0, level: int = LEVEL_SURFACE
    ) -> bool:
        """确保**包含 ``cell`` 的那一块**已生成。返回本次是否新生成。

        这是"实体走到哪就生成哪"的落点。同一块只会生成一次（``_loaded`` 记账），
        重复调用无副作用。

        生成必须**确定**：同一 ``(zone, layer, level, chunk)`` 任何时候生成的
        结果都得一样，否则回放对不上。所以生成器只许用战区的 ``seed``，
        不许用全局随机源或时间。
        """
        if self.chunk_loader is None:
            return False
        col, row = axial_to_offset(cell)
        cx, cy = HexGrid.chunk_key(col, row)
        key = (layer, level, cx, cy)
        if key in self._loaded:
            return False
        self._loaded.add(key)
        self.chunk_loader(self.grid(layer), cx, cy, level)
        return True

    @property
    def loaded_chunks(self) -> int:
        """已生成的分块数。诊断用——它应当随实体活动范围增长，而不是
        一开工就等于整个战区。"""
        return len(self._loaded)

    def _compute_bounds(self) -> tuple[int, int, int, int]:
        """战区的轴向坐标包围盒，用于网格分配和越界判定。"""
        frame = self.frame
        span = self.spec.cell_span()
        corners = [
            frame.from_geo(lat, lng)
            for lat, lng in (
                (self.spec.lat, self.spec.lng),
                (self.spec.lat, self.spec.lng + 0.001),
                (self.spec.lat + 0.001, self.spec.lng),
            )
        ]
        q_min = min(c.q for c in corners) - span
        q_max = max(c.q for c in corners) + span
        r_min = min(c.r for c in corners) - span
        r_max = max(c.r for c in corners) + span
        return q_min, r_min, q_max, r_max

    def contains(self, lat: float, lng: float) -> bool:
        return (
            great_circle_m(self.spec.lat, self.spec.lng, lat, lng)
            <= self.spec.radius_m
        )

    def to_axial(self, lat: float, lng: float) -> Axial:
        return self.frame.from_geo(lat, lng)

    def to_geo(self, c: Axial) -> tuple[float, float]:
        return self.frame.to_geo(c)

    @property
    def bounds(self) -> tuple[int, int, int, int]:
        """(q_min, r_min, q_max, r_max)"""
        return self._cell_bounds

    def __repr__(self) -> str:
        return (
            f"<Zone {self.spec.zone_id}:{self.spec.name} "
            f"r={self.spec.radius_m/1000:.0f}km size={self.spec.resolution_m:.0f}m>"
        )


#: 战区激活时的数据加载回调。由上层（想定装配、地形数据库）实现，
#: 地图服务本身不关心数据从哪来。
ZoneLoader = Callable[[Zone], None]


class MapService:
    """地图服务总入口：全球层 + 按需激活的战区。

    典型用法::

        maps = MapService(global_resolution=5)
        maps.declare(ZoneSpec(1, "战区A", 39.90, 116.40, 200_000, 100))
        maps.declare(ZoneSpec(2, "战区B", 33.31, 44.36, 150_000, 200))

        maps.activate(1)                 # 此时才建坐标系、加载数据
        loc = maps.locate(39.91, 116.42)
        assert loc.is_local
    """

    __slots__ = ("global_layer", "_specs", "_active", "_index", "_loader", "stats")

    def __init__(
        self,
        global_resolution: int = DEFAULT_RESOLUTION,
        loader: ZoneLoader | None = None,
    ) -> None:
        self.global_layer = GlobalLayer(global_resolution)
        self._specs: dict[int, ZoneSpec] = {}
        self._active: dict[int, Zone] = {}
        #: 全球层单元 → zone_id。让 locate() 保持 O(1)。
        self._index: dict[int, int] = {}
        self._loader = loader
        self.stats = {
            "declared": 0,
            "activated": 0,
            "deactivated": 0,
            "index_conflicts": 0,
            "chunks_generated": 0,
        }

    # -- 战区声明与生命周期 ------------------------------------------------

    def declare(self, spec: ZoneSpec) -> ZoneSpec:
        """登记战区定义。不分配任何网格资源。"""
        if spec.zone_id in self._specs:
            raise ValueError(f"zone_id={spec.zone_id} 已声明（{self._specs[spec.zone_id].name}）")
        self._specs[spec.zone_id] = spec
        self.stats["declared"] = len(self._specs)
        return spec

    def declare_all(self, specs: list[ZoneSpec]) -> None:
        for spec in specs:
            self.declare(spec)

    def activate(self, zone_id: int) -> Zone:
        """激活战区：建立投影坐标系、登记索引、触发数据加载。

        这就是"按需加载"的落地点——只有被激活的战区才占内存。
        """
        if zone_id in self._active:
            return self._active[zone_id]

        spec = self._specs.get(zone_id)
        if spec is None:
            raise KeyError(f"未声明的 zone_id={zone_id}")

        zone = Zone(spec)
        self._active[zone_id] = zone

        # 登记全球层索引，之后 locate() 才能 O(1) 命中
        covered = self.global_layer.cells_within(
            spec.lat, spec.lng, spec.radius_m + spec.resolution_m
        )
        for cell in covered:
            existing = self._index.get(cell)
            if existing is not None and existing != zone_id:
                # 战区重叠时后激活的优先。不静默处理——重叠几乎总是
                # 想定写错了，而且会导致实体在两个战区之间来回跳。
                self.stats["index_conflicts"] += 1
            self._index[cell] = zone_id

        if self._loader is not None:
            # loader 现在负责"给战区装上分块生成器"（惰性生成），
            # 而不是"一次性把地形全生成出来"。见 Zone.chunk_loader。
            self._loader(zone)

        self.stats["activated"] += 1
        return zone

    def grid(self, zone_id: int, layer: int = 0) -> HexGrid | None:
        """取某个战区某一分辨率层的网格。未激活或未建层时返回 ``None``。

        这是模型层读地图的**唯一入口**——`services` 之外不该直接碰 `Zone`。
        """
        zone = self._active.get(zone_id)
        if zone is None:
            return None
        # 现建空网格而不是返回 None：空网格是一份**有意义**的数据
        # （可通行的平原 + 海平面高程），模型层不必到处判断 None。
        # "没有数据"与"数据是默认值"在这里是同一件事。
        return zone.grid(layer)

    def load_chunk_at(
        self,
        zone_id: int,
        cell: Axial,
        layer: int = 0,
        level: int = LEVEL_SURFACE,
    ) -> bool:
        """确保包含 ``cell`` 的分块已生成。返回本次是否新生成。

        由装配层注入的 locator 在"实体落到某格"时调用——这就是
        "默认不生成、写了实体位置才加载相应地形"的落点（§3.11.5）。
        未激活的战区返回 ``False``（当地图不存在处理，不报错）。
        """
        zone = self._active.get(zone_id)
        if zone is None:
            return False
        created = zone.ensure_loaded(cell, layer, level)
        if created:
            self.stats["chunks_generated"] += 1
        return created

    def deactivate(self, zone_id: int) -> None:
        """释放战区资源。索引会一并清理，但**不回收**被其他战区占用的单元。"""
        zone = self._active.pop(zone_id, None)
        if zone is None:
            return

        for cell in list(self._index):
            if self._index[cell] == zone_id:
                del self._index[cell]

        self.stats["deactivated"] += 1

    @property
    def active_zones(self) -> list[Zone]:
        """已激活的战区，按 zone_id 升序——保证遍历可复现。"""
        return [self._active[k] for k in sorted(self._active)]

    def zone(self, zone_id: int) -> Zone | None:
        return self._active.get(zone_id)

    def spec(self, zone_id: int) -> ZoneSpec | None:
        return self._specs.get(zone_id)

    # -- 坐标路由（核心）---------------------------------------------------

    def locate(self, lat: float, lng: float) -> Location:
        """定位一个经纬度：优先返回战区内的局部坐标，否则退回全球层。

        O(1)：一次 H3 查询 + 一次字典查找，与战区数量无关。
        """
        cell = self.global_layer.cell_at(lat, lng)
        zone_id = self._index.get(cell)

        if zone_id is not None:
            zone = self._active.get(zone_id)
            if zone is not None and zone.contains(lat, lng):
                return Location(
                    lat, lng, cell, zone=zone, axial=zone.to_axial(lat, lng)
                )
            # 索引命中但实际不在半径内（单元跨越战区边界），
            # 或者战区刚被释放。都退回全球层，不做二次线性扫描——
            # 那种兜底会让最坏情况退化成 O(战区数)。
        return Location(lat, lng, cell)

    def locate_cell(self, cell: int, layer: int = 0) -> Location:
        """由网格单元标识反查位置。"""
        axial, zone_id, _layer, _flags = decode(cell)
        zone = self._active.get(zone_id)
        if zone is None:
            lat, lng = self.global_layer.center(cell)
            return Location(lat, lng, cell)
        lat, lng = zone.to_geo(axial)
        return Location(lat, lng, self.global_layer.cell_at(lat, lng), zone, axial)

    def activate_for(self, lat: float, lng: float) -> Zone | None:
        """如果该点落在某个**已声明但未激活**的战区内，就地激活它。

        用于"实体接近战区时自动加载"。
        """
        for zone_id in sorted(self._specs):
            if zone_id in self._active:
                continue
            spec = self._specs[zone_id]
            if (
                great_circle_m(spec.lat, spec.lng, lat, lng)
                <= spec.radius_m
            ):
                return self.activate(zone_id)
        return None

    # --- 校验与诊断 -------------------------------------------------------

    def validate(self) -> list[str]:
        """检查战区配置中的问题。想在推演前跑一次。"""
        problems: list[str] = []

        for zone_id in sorted(self._specs):
            spec = self._specs[zone_id]
            span = spec.cell_span()
            if span >= COORD_LIMIT:
                problems.append(
                    f"战区 {spec.name}: 仅 {spec.resolution_m:.0f} m 网格就跨越 "
                    f"{span} 格，超出可编码范围"
                )
            total = sum(spec.estimated_cells(i) for i in range(spec.layer_count))
            if total > 50_000_000:
                problems.append(
                    f"战区 {spec.name}: {spec.layer_count} 层网格估算共 "
                    f"{total:,} 单元，内存压力过大，建议调大边长或减少层数"
                )

        ids = sorted(self._specs)
        for i, a_id in enumerate(ids):
            a = self._specs[a_id]
            for b_id in ids[i + 1:]:
                b = self._specs[b_id]
                gap = great_circle_m(a.lat, a.lng, b.lat, b.lng)
                if gap < a.radius_m and gap < b.radius_m:
                    problems.append(
                        f"战区 {a.name} 与 {b.name} 重叠（中心相距 "
                        f"{gap/1000:.0f} km），实体可能在两者间反复切换"
                    )
        return problems

    def __iter__(self) -> Iterator[ZoneSpec]:
        return (self._specs[k] for k in sorted(self._specs))

    def __len__(self) -> int:
        return len(self._active)

    def __repr__(self) -> str:
        return (
            f"<MapService 已声明={len(self._specs)} 已激活={len(self._active)} "
            f"全球res={self.global_layer.resolution}>"
        )
