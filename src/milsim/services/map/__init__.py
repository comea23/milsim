"""地图服务：两层网格 + 坐标路由 + 数据层。

    hex.py       局部平面六边形网格的坐标系统（零依赖，纯 stdlib）
    h3layer.py   全球层，基于 H3 的球面六边形网格
    zone.py      战区管理、按需加载、统一坐标路由
    grid.py      分块存储 + 属性通道（依赖 numpy）
    terrain.py   程序化地形生成（依赖 numpy）
    path.py      A* 寻路，直接搜索与分层搜索自动切换
    los.py       通视判定，含地球曲率与大气折射
    nav.py       导航门面：画像 / 寻路 / 格↔坐标，给组件用的窄接口

分层依赖：``hex`` 不依赖任何东西；``h3layer`` 依赖 ``hex``；
``grid`` 依赖 ``hex``；``terrain`` 依赖 ``grid``；
``path`` / ``los`` 依赖 ``grid``；``nav`` 依赖 ``path``；
``zone`` 依赖 ``hex`` + ``h3layer``。

引擎层不依赖本包；本包也不依赖引擎层——地图是纯被动的数据服务，
所有时间推进都由引擎驱动。
"""

from .grid import (
    CHANNELS,
    CHUNK_SIZE,
    DEFAULT_COVER,
    DEFAULT_ELEVATION,
    DEFAULT_MOVE_COST,
    DEFAULT_TERRAIN,
    IMPASSABLE,
    LEVEL_SURFACE,
    VERTICAL_LEVELS,
    VERTICAL_LEVEL_COUNT,
    ChannelSpec,
    Chunk,
    HexGrid,
    cells_of_chunk,
    level_band,
    level_of_altitude,
)
from .h3layer import (
    DEFAULT_RESOLUTION,
    GlobalLayer,
    cells_for_route,
    edge_length_m,
    great_circle_m,
    initial_bearing_deg,
    interpolate_great_circle,
    resolution_for_edge_length,
    sample_great_circle,
)
from .hex import (
    COORD_LIMIT,
    DIRECTIONS,
    EARTH_RADIUS,
    Axial,
    LocalFrame,
    axial_to_offset,
    cell_zone,
    cube_round,
    decode,
    distance,
    encode,
    encode_axial,
    heading_to_offset,
    line,
    neighbor,
    neighbors,
    offset_to_axial,
    offset_to_heading,
    ring,
    spiral,
)
from .los import (
    DEFAULT_REFRACTION,
    LineOfSight,
    check_line_of_sight,
    horizon_drop_m,
    radio_horizon_m,
    visible_targets,
)
from .nav import (
    PROFILE_AMPHIBIOUS,
    PROFILE_GROUND,
    PROFILE_WATER,
    TRAVEL_PROFILES,
    NavService,
    profile_names,
    resolve_profile,
    terrain_codes,
    travel_profile,
)
from .path import (
    AMPHIBIOUS_TRAVEL,
    COARSE_FACTOR,
    DIRECT_SEARCH_MAX_CELLS,
    GROUND_TRAVEL,
    WATER_TRAVEL,
    CostConfig,
    CoarseView,
    PathResult,
    build_corridor,
    find_path,
    path_cost,
    reachable_cells,
    step_cost,
)
from .terrain import (
    COVER_TABLE,
    MOVE_COST_TABLE,
    TERRAIN_FOREST,
    TERRAIN_HILL,
    TERRAIN_MOUNTAIN,
    TERRAIN_NAMES,
    TERRAIN_PLAIN,
    TERRAIN_URBAN,
    TERRAIN_WATER,
    TerrainStats,
    generate_flat,
    generate_terrain,
)
from .zone import (
    MAX_ZONE_RADIUS_M,
    MIN_ZONE_RADIUS_M,
    Location,
    MapService,
    Zone,
    ZoneLoader,
    ZoneSpec,
)

__all__ = [
    # -- 局部平面网格 --
    "Axial",
    "LocalFrame",
    "DIRECTIONS",
    "EARTH_RADIUS",
    "COORD_LIMIT",
    "distance",
    "neighbors",
    "neighbor",
    "ring",
    "spiral",
    "line",
    "cube_round",
    "axial_to_offset",
    "offset_to_axial",
    "heading_to_offset",
    "offset_to_heading",
    "encode",
    "encode_axial",
    "decode",
    "cell_zone",
    # -- 全球层 --
    "GlobalLayer",
    "DEFAULT_RESOLUTION",
    "edge_length_m",
    "resolution_for_edge_length",
    "great_circle_m",
    "initial_bearing_deg",
    "interpolate_great_circle",
    "sample_great_circle",
    "cells_for_route",
    # -- 战区 --
    "MapService",
    "Zone",
    "ZoneSpec",
    "ZoneLoader",
    "Location",
    "MIN_ZONE_RADIUS_M",
    "MAX_ZONE_RADIUS_M",
    # -- 网格数据 --
    "HexGrid",
    "Chunk",
    "CHUNK_SIZE",
    "IMPASSABLE",
    "DEFAULT_ELEVATION",
    "DEFAULT_TERRAIN",
    "DEFAULT_COVER",
    "DEFAULT_MOVE_COST",
    "cells_of_chunk",
    # -- 通道清单与垂向分层（§3.11）--
    "CHANNELS",
    "ChannelSpec",
    "LEVEL_SURFACE",
    "VERTICAL_LEVELS",
    "VERTICAL_LEVEL_COUNT",
    "level_of_altitude",
    "level_band",
    # -- 地形生成 --
    "generate_terrain",
    "generate_flat",
    "TerrainStats",
    "TERRAIN_WATER",
    "TERRAIN_PLAIN",
    "TERRAIN_FOREST",
    "TERRAIN_HILL",
    "TERRAIN_MOUNTAIN",
    "TERRAIN_URBAN",
    "TERRAIN_NAMES",
    "MOVE_COST_TABLE",
    "COVER_TABLE",
    # -- 寻路 --
    "find_path",
    "reachable_cells",
    "step_cost",
    "path_cost",
    "build_corridor",
    "CostConfig",
    "PathResult",
    "CoarseView",
    "COARSE_FACTOR",
    "DIRECT_SEARCH_MAX_CELLS",
    "GROUND_TRAVEL",
    "WATER_TRAVEL",
    "AMPHIBIOUS_TRAVEL",
    # -- 导航门面 --
    "NavService",
    "TRAVEL_PROFILES",
    "PROFILE_GROUND",
    "PROFILE_WATER",
    "PROFILE_AMPHIBIOUS",
    "profile_names",
    "resolve_profile",
    "terrain_codes",
    "travel_profile",
    # -- 通视 --
    "check_line_of_sight",
    "visible_targets",
    "radio_horizon_m",
    "horizon_drop_m",
    "LineOfSight",
    "DEFAULT_REFRACTION",
]
