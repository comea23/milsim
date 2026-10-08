"""导航门面：把世界知识收成组件能用的几个动作。

为什么要有这一层
----------------
机动组件需要三样世界知识：**怎么走到目的地**（A*）、**这一格好不好走**
（通行代价）、**格与局部坐标怎么互换**（战区投影）。它们都在本包里，但
组件不该 import 地图服务——一旦 import，组件就与战区、分块、通道的内部
结构绑死，换一种存储要改所有组件（§5 的窄接口原则；装配层注入 ``locator``
是同一个理由）。

于是把这三样收成 :class:`NavService`，由装配层放进
:class:`~milsim.models.mount.MountContext`。组件侧只看到 ``mount.plan_route``
这类调用，看不到 ``Zone`` / ``HexGrid`` / ``find_path``。

术语：``profile``（画像）
------------------------
画像是**运载器的解释**，不是地形的属性（§3.11.3）：同一片水面，坦克看到
"不可通行"、舰船看到"好走"、两栖车看到"能走但慢"。所以画像名是字符串
（``ground`` / ``water`` / ``amphibious``）——想定与组件参数里写的是
"这是一艘船"，不是"这是 slope_weight=0 的配置"。

画像之后还有一层**底盘门槛**：同样是陆地，轮式与履带看到的是同一张
``ground`` 表，但轮式过不去山地、履带过得去。这一层由组件参数给出
（``blocked_terrain`` / ``min_water_depth`` / ``max_slope``），叠在画像上
算出一份 :class:`~milsim.services.map.path.CostConfig`，再交给本门面
（见 :func:`resolve_profile`）。分开的理由：画像是**介质口径**，改它要动
代码；门槛是**这辆车**的属性，改它是改参数——而且"哪些地貌过不去"做成
从表里**减**，比每个底盘各存一张表更不容易漂移。

寻路看到的地形是**按需生成的**
------------------------------
分块只在实体踩到它时才生成（§3.11.5）。未生成的格读出来是通道默认值，
而这里有个**必须说清**的事实：``terrain`` 的默认值是 ``0``，也就是**水域**；
``move_cost`` 的默认值是 ``1``（可通行）。两者对"未知地形"的态度正好相反，
于是同一片没生成的区域：

- 走 ``move_cost`` 通道（``CostConfig(costs=None)``）看到的是"可通行的平地"
  ——A* 会据此规划出一条穿山的路，而那条路**不一定存在**，且不报错；
- 走地形表的画像（:data:`TRAVEL_PROFILES` 里那三张）看到的是**水域**
  ——地面件会认为整片都不可通行，A* 被一圈假水域围住。

两种都不对，只是错的方向相反。本模块的对策是**寻路前把起点到终点的走廊
先真实生成**（:meth:`NavService.ensure_corridor`）：只加载端点不够，因为
A* 的搜索空间远大于端点。这一条是实测出来的——修之前，"隔两个分块"的地面
目标在种子 31 的战区上**永远返回走不通**，而机动件会据此报一句假的
"起点或终点不可通行、或被完全包围"。

**这没有解决问题，只是把它推远了。** 走廊之外仍然是"假水域"，于是：

- 目标稍远时仍可能返回走不通（实测 40 个跨区目标里 2 个）；
- 找到的路线可能为了躲假水域而绕远，代价最高到"整片战区都生成好"时的
  **两倍**（实测 40 个样本里 20 个偏高，最大 196.3 → 364.8）。

把默认地貌改成平原也是错的：那会让搜索**穿过实际存在的山**，实测 40 个
样本里 5 个代价明显偏低（478.8 → 245.1）——它不报错，只是把一条不存在的
路算得很好看。两种默认值都不对，根子在"**还没生成**"和"**真的是水域**"
在 ``terrain`` 通道里本来就分不开。真正的修法要在数据模型上动手（给通道
加"已生成"掩码、或让搜索在判断某格前先把它生成出来），收益与代价都还没
称过，所以整条记入 §9 待决，不在这里猜一个补丁。

当下的两条兜底：走廊会先加载（把最常见的假"走不通"消掉），机动组件在
"下一格突然不可通行"时会重规划（见 :mod:`milsim.models.mover`）。
"""

from __future__ import annotations

from difflib import get_close_matches
from typing import Any, Mapping, Sequence

from ...errors import ConfigurationError
from .grid import IMPASSABLE, LEVEL_SURFACE, HexGrid
from .hex import Axial, line
from .los import check_line_of_sight
from .path import (
    AMPHIBIOUS_TRAVEL,
    GROUND_TRAVEL,
    WATER_TRAVEL,
    CostConfig,
    PathResult,
    find_path,
)
from .terrain import TERRAIN_NAMES

#: 地面画像：读 ``move_cost`` 通道（水域不可通行）。
PROFILE_GROUND = "ground"

#: 水面画像：水域可通行、陆地不可——与地面表恰好互补，且不吃坡度。
PROFILE_WATER = "water"

#: 两栖画像：水陆都能走，山地不可。
PROFILE_AMPHIBIOUS = "amphibious"

#: 画像清单。**必须有清单**，否则就退化成"组件随手写字符串、写错不报错"，
#: 与 §3.11.3 给网格通道立清单是同一条道理。
TRAVEL_PROFILES: Mapping[str, CostConfig] = {
    PROFILE_GROUND: GROUND_TRAVEL,
    PROFILE_WATER: WATER_TRAVEL,
    PROFILE_AMPHIBIOUS: AMPHIBIOUS_TRAVEL,
}


def profile_names() -> list[str]:
    """可用画像名，**排序**——保证错误信息与遍历结果可复现。"""
    return sorted(TRAVEL_PROFILES)


def travel_profile(name: str) -> CostConfig:
    """画像名 → 代价配置。名字写错**当场报错并给拼写建议**。"""
    config = TRAVEL_PROFILES.get(name)
    if config is None:
        close = get_close_matches(name, list(TRAVEL_PROFILES), n=1, cutoff=0.6)
        hint = f"（是否想写 {close[0]!r}？）" if close else ""
        raise ConfigurationError(
            f"未知的运载器画像 {name!r}{hint}。可用：{'、'.join(profile_names())}"
        )
    return config


def resolve_profile(value: "str | CostConfig") -> CostConfig:
    """画像名 **或** 一份已经调好的配置 → 配置。

    为什么两种都收：画像给的是**介质口径**（这片地形对地面车辆意味着什么），
    而"这辆车过不去山地、吃水 6 m"是**具体底盘**的属性，由组件参数给出。
    组件在 :meth:`~milsim.models.mover.Mover.initialize` 里把画像叠上自己的
    门槛、算出一份配置，之后每次寻路都用它——所以门面的入参必须能同时接
    这两样东西。

    只写字面量名字的调用方（想定工具、测试）行为完全不变。
    """
    if isinstance(value, CostConfig):
        return value
    return travel_profile(value)


def terrain_codes(names: Sequence[str]) -> frozenset[int]:
    """地貌**名字** → 编号集合。名字写错**当场报错并给拼写建议**。

    查表而不是猜：静默忽略一个拼错的 `"山底"`，表现是"这条门槛没生效"，
    而那要翻很久才能看出来。想定侧本来就有 ``choices`` 拦一道，这里再拦
    一道路由是给**代码里写死名字**的调用方（使用者扩展的组件、脚本）。
    """
    by_name = {terrain: code for code, terrain in TERRAIN_NAMES.items()}
    codes: set[int] = set()
    for name in names:
        code = by_name.get(name)
        if code is None:
            #: 阈值比画像名那边低（0.5 而不是 0.6）：地貌名只有两个字，
            #: difflib 是按字符算比值的，"山底"↔"山地" 恰好是 0.5，正好
            #: 卡在默认阈值下面——而那是最该给出建议的一类错法（错一个字）。
            close = get_close_matches(name, list(by_name), n=1, cutoff=0.5)
            hint = f"（是否想写 {close[0]!r}？）" if close else ""
            raise ConfigurationError(
                f"未知的地貌名 {name!r}{hint}。"
                f"可用：{'、'.join(sorted(by_name))}"
            )
        codes.add(code)
    return frozenset(codes)


class NavService:
    """按实体问"怎么走"的门面。

    只持有 :class:`~milsim.services.map.zone.MapService` 一个引用，**不缓存
    任何派生量**：战区可能在推演中途激活/释放，缓存下来的 ``HexGrid`` 会
    变成指向旧战区的悬空引用——那种错不报错，只让实体在错误的网格上寻路。
    """

    __slots__ = ("_maps",)

    def __init__(self, maps: Any) -> None:
        self._maps = maps

    # -- 画像 --------------------------------------------------------------

    def profile(self, name: str) -> CostConfig:
        return travel_profile(name)

    @staticmethod
    def profiles() -> list[str]:
        return profile_names()

    def capable_profile(
        self,
        name: str,
        *,
        blocked_terrain: Sequence[str] = (),
        min_water_depth: float = 0.0,
        max_slope: float | None = None,
    ) -> CostConfig:
        """画像 + **这一辆车**的通行门槛 → 一份可直接用来寻路的配置。

        地貌用**名字**给（``"山地"``），不是编号：组件与想定里写的是
        "轮式进不了山地"，编号只是本包的内部表示。名字写错会报错并给
        拼写建议——静默忽略一个拼错的地貌名，表现为"这条门槛没生效"。

        给出的是**新对象**，共享的画像（``GROUND_TRAVEL`` 等）不受影响；
        否则改一辆车的门槛会把整个战区同画像的车一起改掉，且不报错。
        """
        return travel_profile(name).with_capabilities(
            blocked_terrain=terrain_codes(blocked_terrain),
            min_water_depth=min_water_depth,
            max_slope=max_slope,
        )

    # -- 定位 --------------------------------------------------------------

    def zone_of(self, ref: Any) -> Any:
        """实体所在战区。全球层（``GlobalCellRef``）返回 ``None``。

        用 ``getattr`` 而不是 ``isinstance``：战区外的格引用是另一种类型，
        而本模块不该为了判断"这是不是局部格"去 import 存储层的类。
        """
        zone_id = getattr(ref, "zone_id", None)
        if zone_id is None:
            return None
        return self._maps.zone(zone_id)

    def grid_of(self, ref: Any) -> HexGrid | None:
        zone_id = getattr(ref, "zone_id", None)
        if zone_id is None:
            return None
        return self._maps.grid(zone_id, getattr(ref, "layer", 0))

    def ensure_loaded(self, ref: Any, *cells: Axial) -> int:
        """确保这些格所在的分块已生成。返回新生成的块数。

        寻路前必须对**起点与终点**做这一步：分块是踩到才生成的，而
        "没生成的块"读出来是平坦默认地形。在默认地形上规划出来的路
        会穿过实际存在的山，且**不会有任何报错**。
        """
        zone_id = getattr(ref, "zone_id", None)
        layer = getattr(ref, "layer", 0)
        if zone_id is None:
            return 0
        created = 0
        for cell in cells:
            if self._maps.load_chunk_at(zone_id, cell, layer, LEVEL_SURFACE):
                created += 1
        return created

    def ensure_corridor(self, ref: Any, start: Axial, goal: Axial) -> int:
        """确保 ``start`` → ``goal`` **沿途**的分块都真实生成。返回新生成的块数。

        只加载端点是不够的，这一点踩过：A* 的搜索空间远大于端点，而分块
        没生成时 ``terrain`` 读出的是 ``0``——那正是**水域**。地面画像按
        地形查表，于是整片"还没生成"的区域对地面件都是不可通行的。后果是
        地面单位的目标只要跨过两个分块，A* 就被一圈假水域围住、返回
        ``None``，机动件随即报"没有可行路线（起点或终点不可通行、或被完全
        包围）"——**那个理由是假的**，真实原因是中间那片地形还没生成。

        沿线加载之所以够用：加载的粒度是分块（``CHUNK_SIZE`` × ``CHUNK_SIZE``
        个单元），所以实际拿到的是沿线的若干整块，A* 绕行水域的幅度通常
        落在带内。真正绕出加载带的残余风险记在 §9。
        """
        cells = line(start, goal)
        return self.ensure_loaded(ref, *cells)

    def ensure_region(self, ref: Any, center: Axial, radius: int) -> int:
        """确保以 ``center`` 为圆心、``radius`` 格以内的分块都已生成。

        返回本次新生成的块数。

        这是给**规划**用的，不是给"走到哪生成哪"用的。:meth:`ensure_corridor`
        按"起点→终点"的沿线带加载，够日常机动用；但**绕行**会伸出带外，
        带外读到的是 ``DEFAULT_TERRAIN``——编号 0，**水域**。地面画像下
        那是一片假墙，于是"绕远一点就能到"的地方被判成不可达，而且不报错。
        代价是慢（A* 在大地形上多展开一圈），不是错。

        半径按**绕行幅度**取，不是按航程取：自动生成的绕行虽可能有几倍
        航程的代价，偏离直线的**横向**距离却远小于航程。两三个分块
        （``CHUNK_SIZE`` = 32）够用。
        """
        zone_id = getattr(ref, "zone_id", None)
        if zone_id is None:
            return 0
        cells = [
            Axial(center.q + dq, center.r + dr)
            for dq in range(-radius, radius + 1)
            for dr in range(-radius, radius + 1)
        ]
        return self.ensure_loaded(ref, *cells)

    def cell_center(self, ref: Any, cell: Axial) -> tuple[float, float] | None:
        """格 → 局部世界坐标 ``(x, y)``，米。战区外返回 ``None``。"""
        zone = self.zone_of(ref)
        if zone is None:
            return None
        return zone.frame.to_world(cell)

    def axial_at(self, ref: Any, x: float, y: float) -> Axial | None:
        """局部世界坐标 → 格。战区外返回 ``None``。"""
        zone = self.zone_of(ref)
        if zone is None:
            return None
        return zone.frame.from_world(x, y)

    def ground_at(self, ref: Any, x: float, y: float) -> float | None:
        """点位处的地面高程（米，海平面 = 0）。战区外返回 ``None``。

        地面机动件用它把 z 拉回地面：不给的话，一辆车翻过山后位置里的
        高度还是出发时的值——位置数据自相矛盾，而且**不报错**。
        """
        grid = self.grid_of(ref)
        axial = self.axial_at(ref, x, y)
        if grid is None or axial is None:
            return None
        return float(grid.elevation(axial))

    def surface_height(
        self, ref: Any, x: float, y: float, *, terrain_floor: float | None = 0.0
    ) -> float | None:
        """点位处的**地表高程**（米，海平面 = 0），水域按海平面算。

        与 :meth:`ground_at` 的差别只有一个：默认把高程**下限钉在 0**。
        陆地上两者相同；水面上 ``ground_at`` 给的是**海床**（某格实测
        −423.5 m），而感知/交战层要的是"目标站在哪儿"——海面上的舰船站在
        海平面上，不是站在海床上。传 ``terrain_floor=None`` 取回原始口径。

        战区外返回 ``None``——与"高度 0"是两件不同的事，调用方必须分开处理
        （0 是"海平面"，``None`` 是"这里没有逐格数据"）。
        """
        height = self.ground_at(ref, x, y)
        if height is None:
            return None
        if terrain_floor is None:
            return height
        return max(height, float(terrain_floor))

    def line_of_sight(
        self,
        ref: Any,
        observer_xy: tuple[float, float],
        target_xy: tuple[float, float],
        observer_z: float = 0.0,
        target_z: float = 0.0,
        *,
        observer_extra_height: float = 0.0,
        target_extra_height: float = 0.0,
        required_clearance: float = 0.0,
        terrain_floor: float | None = 0.0,
        ensure_loaded: bool = True,
    ) -> bool | None:
        """两点之间是否通视（含地球曲率与 4/3 折射）。**数据缺失返回 ``None``**。

        为什么在门面上而不是让组件自己调 :func:`~milsim.services.map.los.check_line_of_sight`：
        那个函数要一个 :class:`~milsim.services.map.grid.HexGrid`，而"组件不许
        认识地图服务的内部结构"是 §5 的硬约定。这里把网格、格归属、按需生成
        全部吞掉，组件只给两个坐标和两个高度。

        三个口径写在这里，因为每一个都曾经是"报出来的理由是假的"那类 bug：

        * ``observer_z`` / ``target_z`` 是**海拔**（米），与实体位置里的 z 同口径。
          内部会各自减去脚下的地表高程（按 ``terrain_floor`` 取过下限）换算成
          ``check_line_of_sight`` 要的"离地高度"——不减就会在陆地上把高度算两遍。
        * ``observer_extra_height`` 是**架高**（天线离地高度）。
        * ``target_extra_height`` 是**对面那一端的架高**，v0.13.33 加。默认
          ``0`` ⇒ 对面只有它自己的位置高度（探测链路的旧口径，逐字不变）。
          它存在是因为**干扰链路的另一端也可能架天线**：一台架高的干扰机
          压一台架高的雷达，两端都要算架高；而探测时"我架天线、目标飞着"，
          对面这一项本来就是 0。
        * ``ensure_loaded=True`` 会先把沿线分块真实生成。不这么做时读到的
          是"还没生成"的平坦默认地形（水域），于是判出来的通视**反映的是
          生成进度不是地形**——这一点与 :meth:`ensure_region` 的理由完全一样。

        返回 ``None`` 表示"这里没有逐格精度"（全球层 / 战区外），调用方应当
        **跳过**这条判据，而不是当作遮挡。
        """
        grid = self.grid_of(ref)
        if grid is None:
            return None
        observer = self.axial_at(ref, *observer_xy)
        target = self.axial_at(ref, *target_xy)
        if observer is None or target is None:
            return None
        if ensure_loaded:
            self.ensure_corridor(ref, observer, target)

        floor = None if terrain_floor is None else float(terrain_floor)

        def above_ground(cell: Axial, z: float) -> float:
            height = float(grid.elevation(cell))
            if floor is not None:
                height = max(height, floor)
            return z - height

        result = check_line_of_sight(
            grid,
            observer,
            target,
            above_ground(observer, observer_z) + observer_extra_height,
            above_ground(target, target_z) + target_extra_height,
            required_clearance=required_clearance,
            terrain_floor=terrain_floor,
        )
        return result.visible

    # -- 寻路 --------------------------------------------------------------

    def route(
        self,
        ref: Any,
        goal: Axial,
        *,
        profile: "str | CostConfig" = PROFILE_GROUND,
        forbidden: set[Axial] | None = None,
    ) -> PathResult | None:
        """从实体当前格到 ``goal`` 的一条路。走不通返回 ``None``。

        ``profile`` 可以是画像名，也可以是组件自己调好门槛的
        :class:`CostConfig`（见 :func:`resolve_profile`）。

        返回 ``None`` 的三种情形（目标不可通行、被完全包围、超出展开上限）
        对调用方是同一件事：**该走直线，而不是停在原地**。
        """
        grid = self.grid_of(ref)
        if grid is None:
            return None
        start = getattr(ref, "axial", None)
        if start is None:
            return None

        self.ensure_corridor(ref, start, goal)
        return find_path(
            grid,
            start,
            goal,
            forbidden=forbidden,
            config=resolve_profile(profile),
        )

    # -- 通行 --------------------------------------------------------------

    def terrain_name(self, ref: Any, cell: Axial) -> str:
        """这一格的地貌名（"水域"/"山地"/…）。查不到返回空串。

        报错与诊断用：'目的格走不了' 与 '目的格是水域' 对作者的价值差得很远，
        前者还要自己去查是哪一格、什么地貌。
        """
        grid = self.grid_of(ref)
        if grid is None:
            return ""
        return TERRAIN_NAMES.get(int(grid.terrain(cell)), "")

    def move_factor(
        self,
        ref: Any,
        cell: Axial | None = None,
        *,
        profile: "str | CostConfig" = PROFILE_GROUND,
    ) -> float:
        """这一格的**速度系数**：1.0 = 平坦好路，越小越难走，不可通行 0.0。

        为什么不另立一张"速度表"：代价表已经是"这片地形对这类运载器意味着
        什么"的**唯一**定义。再立一张速度表就是同一件事有两个真相，两份
        数字会各自演化，迟早不同步——而不报错。

        ``1/代价`` 是近似（真实底盘的速度还取决于坡度、路况、编队间距），
        但它与寻路用的是同一份数据，所以"绕远路但好走"和"直穿山地"的
        时间比较是自洽的。

        门槛也走这里：一辆被山地封住的轮式车，在山地格上的速度系数是 0.0
        ——与"寻路绕开了山地"是同一件事的两个说法，不该出现"路线绕开了、
        但速度还算得出 12 m/s"这种自相矛盾。
        """
        grid = self.grid_of(ref)
        if grid is None:
            return 1.0
        target = getattr(ref, "axial", None) if cell is None else cell
        if target is None:
            return 1.0
        cost = resolve_profile(profile).cost_at(grid, target)
        if cost >= IMPASSABLE:
            return 0.0
        return 1.0 / cost if cost > 0.0 else 1.0

    def passable(
        self,
        ref: Any,
        cell: Axial,
        *,
        profile: "str | CostConfig" = PROFILE_GROUND,
    ) -> bool:
        grid = self.grid_of(ref)
        if grid is None:
            return False
        return resolve_profile(profile).is_passable(grid, cell)

    def refusal(
        self,
        ref: Any,
        cell: Axial,
        *,
        profile: "str | CostConfig" = PROFILE_GROUND,
    ) -> str:
        """这一格走不了的**原因**（人话）；走得通或查不到地图时返回空串。

        报错与诊断用。机动件说"走不通"时要能说清是哪条门槛挡的，否则会把
        人引向错误的方向：潜艇被拦下来通常是因为**水深**，而那一格在地貌上
        仍然写着"水域"——只报地貌的话，作者会去查一张查不出问题的地图。

        与 :meth:`passable` 是同一批门槛的两个视图（一个给判断、一个给
        解释），所以两边不会各说各话。
        """
        grid = self.grid_of(ref)
        if grid is None:
            return ""
        return resolve_profile(profile).gate_reason(grid, cell)

    def __repr__(self) -> str:
        return f"<NavService 画像：{'、'.join(self.profiles())}>"


__all__ = [
    "NavService",
    "PROFILE_AMPHIBIOUS",
    "PROFILE_GROUND",
    "PROFILE_WATER",
    "TRAVEL_PROFILES",
    "profile_names",
    "resolve_profile",
    "terrain_codes",
    "travel_profile",
]
