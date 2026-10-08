"""机动件基类：把"想去哪"变成"每帧挪多少"（§5.7）。

本模块**只有基类**。四个参考实现各在自己的文件里（``ground`` / ``water`` /
``subsurface`` / ``air``），都直接继承 :class:`Mover`、彼此不互相依赖——
一条横着的继承链（水面继承地面、潜艇继承水面）会让改一个组件牵动另一个，
而它们之间本来没有任何共享行为。

.. code-block:: text

    使用者扩展    MyWheeledMover(GroundMover)          写自己的底盘模型
                        ↑ 继承
    参考实现      GroundMover / WaterMover /            可直接用、能跑通
                  SubsurfaceMover / AirMover           （四个文件，四份参数）
                        ↑ 继承
    抽象基类      Mover                                只定义契约（本文件）

基类负责三件所有运载器都要做的事，子类只补"自己那部分"：

| 谁 | 负责 |
|---|---|
| 本类 | 航路点与段推进、**三维质点运动学**、节拍、到没到、走不通 |
| 子类 | 路线怎么算（地面绕地形 / 空中直线）、速度随什么变、高度怎么定 |

**三自由度：三个通道，每个通道两层**
------------------------------------
本类实现的是一个**三维质点模型**（飞行力学里的 3-DOF，只建模三个平动
自由度，不建模滚转/俯仰/偏航姿态）。三个受控通道，**每个通道都有两层
参数**：一层说"这个通道最多多快"，一层说"这个通道最快多久能变到那么快"。

| 通道 | 速率上限 | 加速度上限 | 谁在管 |
|---|---|---|---|
| 前向 | ``max_speed`` | ``linear_accel`` | 本类（再乘当地通行系数，§5.7.6） |
| **转向** | ``turn_rate`` / ``radial_accel`` | ``angular_accel`` | 本类（§5.7.11） |
| 垂向 | ``climb_rate`` / ``descent_rate`` | ``vertical_accel`` | 子类的 :meth:`_approach_z` |

**只有速率上限是不够的**：``max_speed`` 说得出"最快多快"，说不出"多久能
到那么快"。少了第二层，速率就是**阶跃**——第一帧 0 → 25 m/s，那是一个
无穷大的加速度。三处一模一样：起步（速率阶跃）、爬升率建立（垂向速率
阶跃）、转向（角速度阶跃）。补上第二层之后三个通道的状态都变成**分段
线性**的：加速度取常数，速率是折线。这是 v0.13.15 补的。

转向通道的角速度上界由两层合起来给：

.. code-block:: text

    ω_max(v) = min( turn_rate , radial_accel / v )        （0 = 不限制）
    R(v)     = v / ω_max(v)

``radial_accel``（最大水平径向加速度）这一项是 v0.13.15 补的：**固定转弯率
在不同速度下不具备可比性**——同一条"3 °/s"的曲线在 40 m/s 隐含 0.21 g、
在 320 m/s 隐含 1.7 g，等于把"过载上限"这个真实约束藏进了速度里，而且
高速端越来越松。真实平台的机动能力由过载（结构 + 升力）定，与速度近似
无关。所以**速度会变**的件（空中件、车轮件）写 ``radial_accel``；
速度基本不变的件（水面舰、潜艇）固定 ``turn_rate`` 与它是等价的。

两层的语义不要混：``turn_rate 0`` 说的是角速度**上界**不存在，**不是**
"瞬转"——角速度**建立**得多快仍然归 ``angular_accel`` 管。三个通道一律
如此，所以"完全不约束某个通道"要**两层都写 0**。

地面件与水面的垂向保持**位形约束**（z = 当地地面高程 / 海平面），不由速率
控制：它们的第三个自由度是"随地形起伏"而不是"按速率升降"，``vertical_accel``
对它们**不接通**（不是"写了不生效"——是那个通道没有速率这一层）。空中件与
水下件的垂向才是速率控制。这是四个文件各自的 :meth:`_approach_z` 的事，共用
基类的 :meth:`_glide_z` 把"速率 + 加速度"两步走完。

**目的地不是组件参数**
----------------------
目的地是运行期下达的东西（一条任务、一条指令），属 M5a 的台账。组件参数
只说"这辆车最快多快、多久算一次"。把目的地的初值写成参数会立刻多出一个
问题："想定里写死的路线"与"任务下达的路线"哪个优先——这个问题没有正确
答案，只有"没人说得清"。

**走不通时停住，不穿山**
----------------------
:mod:`~milsim.services.map.path` 的建议是"找不到路就改走直线，别停在原地"。
对空中是对的，对地面**不能照抄**：直线会穿过湖泊与山脊。停住至少是物理上
可能做到的，而"看起来在动"的假象会让上层的任务台账以为进展顺利。所以
地面件走不通时置 ``blocked``、原地不动，并在**状态跳变那一下**发一条总线
信号（``mover_blocked``），让任务层据此重规划——每帧都发就等于没有信号。

**地面机动的 z 是贴地的，不是恒定的**
------------------------------------
"实体踩到哪就生成哪块地形"意味着海拔随位置变。若机动件不改 z，一辆车
翻过 800 m 的山后它的 AGL 会变成 -800 m——按 §3.11 的垂向分层它仍在地表带
（负高度也归地表），暂时不出错，但位置数据本身就是错的。所以地面件每帧
把 z 拉回当地地面高程；水面件恒取海平面 0 m（§9 第 13 条）。

**"可通行能力"是参数，"走得快慢"也是参数，但两者不是一回事**
------------------------------------------------------------
一辆轮式车与一辆履带车在平原上只是速度不同，到了山地是**能不能去**的
区别。所以底盘有两条互相独立的参数通道：

- ``max_speed`` —— 走得快慢；
- ``blocked_terrain`` / ``min_water_depth`` —— 过不过得去。

后者由 :meth:`Mover._build_travel` 叠到画像上，算出一份属于**这辆车**的
:class:`~milsim.services.map.path.CostConfig`，此后寻路、减速、判"下一格
能不能走"全用这一份。分成两份画像会漂移（轮式的山地表与履带的山地表迟早
不一致，而不报错），所以这里是从介质表里做**减法**，不是各写一张表。

为什么地貌用**名字**（``blocked_terrain [山地]``）而不是编号：想定作者写的是
"轮式进不了山地"，不是"terrain == 4"。名字与编号的对应只在
:mod:`milsim.services.map` 里存一份，组件不参与。

**水上的门槛是吃水，水下的门槛是"定深 + 离底余量"**
--------------------------------------------------
``min_water_depth`` 只看"这一格的水够不够深"。地形那边水深是**派生量**
（``max(0, -elevation)``，§9 第 13 条），所以陆地格恒为 0——把
``min_water_depth`` 设成正数的地面件会发现自己哪儿都去不了，这是对的：
参数语义是"至少这么深的水"，而不是"水深不影响我"。

对潜艇它**不是反的**：航渡时水越深越好，约束是单边的——``水深 ≥ 定深 +
离底余量``。同一个门槛，只是那个数随"今天潜多深"变，不是船体常数。
真正**双边**的是 **z** 的可行区间：

.. code-block:: text

    -(水深 - 离底余量)  ≤  z  ≤  0
          海底之上、余量之外        水面之下

门槛保证这个区间非空（否则那一格对这条艇不可通行），水下件的每帧夹取保证
它真的被守住——按坐标点下达目的地时根本没有门槛，中间跨过的格没人查过。
别把"潜艇怕水浅"读成"潜艇也怕水太深"，那会把一条本该畅通的深水航线封掉。
"""

from __future__ import annotations

from math import degrees, hypot, inf, radians, sin, sqrt
from typing import Any, Sequence

from ...engine import EventResult, PRIORITY_MOVER
from ...services.map import TERRAIN_NAMES
from ...services.map.hex import Axial, heading_to_offset, offset_to_heading
from ...services.map.nav import PROFILE_GROUND
from ...services.params import Params
from ..component import Component

#: 机动件的默认更新周期。与引擎的 2 s 栅栏错开，免得挤在同一时刻。
MOVER_PERIOD_US = 5_000_000

#: ``blocked_terrain`` 参数能写的地貌名。**排序**，让报错信息与参数库
#: 回写都可复现。取自地图服务公开的词汇表——组件不该自己维护第二份
#: 地貌名单，否则地图那边加一种地貌，这边会静默地漏掉它。
BLOCKABLE_TERRAIN = tuple(sorted(TERRAIN_NAMES.values()))

#: 航向误差大于它时**先掉头、不前进**。
#:
#: 为什么不是"边走边转"：转向速率给出来的是**转弯半径**（速率 ÷ 角速度），
#: 而半径与路点间距不是同一个量级。海军潜艇的半径约 690 m，而 200 m 网格
#: 的路点间距是 346 m——带着 60° 偏差边走边转，它会沿着那条圆弧**绕过**
#: 路点：每一帧都在动、到路点的距离却在增大，最后绕着路点打转。
#: 30° 是"横向偏离还不到半格"的那个量级（346 m × sin30° ≈ 173 m）。
STEER_BEFORE_MOVE_DEG = 30.0

#: 分步推进的步长 = 相邻格中心距 ÷ 这个数。见 :meth:`Mover._march`：
#: 路点是可通行的，但**弧线不是**，所以要按格切碎了查。
MARCH_STEPS_PER_CELL = 4


def signed_angle(deg: float) -> float:
    """把角度折到 ``(-180, 180]``：转向要走最短的那一侧。

    不折的话，从 350° 转到 10° 会被算成倒转 340°，实体当场掉头绕一大圈
    ——而且它就"看起来在动"，只有对着地图看航迹才会发现方向反了。
    """
    value = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 else value


class Mover(Component):
    """机动件基类。**不含任何地图知识**——那由装配层注入的导航门面提供。

    子类要实现的是四个钩子，而不是重写整个推进逻辑：

    - :meth:`_plan` —— 目的地变成一串航路点（地面走 A*，空中直接连直线）
    - :meth:`_waypoint_z` —— 这段路走多高
    - :meth:`_surface_z` —— 走完之后 z 拉回哪（贴地 / 海平面 / 不管）
    - :meth:`_approach_z` —— 这一帧怎么走到那个高度（一步到 / 按速率）

    覆盖粒度这么细是 §5.4 的要求：使用者要改的往往只是"高度怎么定"，
    不该为了那一件事把段推进、跨格、走不通判定全部复制一遍。
    """

    PARAMS = {
        "max_speed": Params.speed(20.0, minimum=0.0),
        #: **转向速率**（度/秒）——三自由度里的第二个，也是补得最晚的一个。
        #: 它给航向加一个角速度上界：转向要花时间，于是**转弯半径 = 速率 ÷
        #: 角速度**出现了，进弯之前得先减速。``0`` = **不加转向约束**
        #: （航向瞬时对准目标）——与 ``min_water_depth 0`` / ``blocked_terrain``
        #: 留空同一条约定：门槛参数的 0 / 空值表示"不限制"，不是"禁止"。
        #: 想定里想回到"瞬转"就显式写 ``turn_rate 0``。
        "turn_rate": Params.angle_rate(30.0, minimum=0.0),
        #: **最大水平径向加速度**（m/s²，0 = 不限制）——速度相关的角速度上界
        #: ``ω ≤ radial_accel / v``。它才是"过载"，而 ``turn_rate`` 只是机构
        #: 上限：同一枚弹在 40 m/s 与 320 m/s 下的可用角速度差 8 倍，写一个
        #: 固定的 ``turn_rate`` 等于把过载上限藏进速度里（§5.7.11）。
        #:
        #: ``0`` = 不限制。恒速平台（水面舰、潜艇）不写它——它们的"转弯半径
        #: 与航速无关"这条与固定 ``turn_rate`` 等价。
        "radial_accel": Params.accel(0.0, minimum=0.0),
        #: **直线加速度上限**（m/s²，0 = 不限制）——前向通道的第二层。
        #: 起步与刹车共用它（对称限幅）。少了它，速率是阶跃的：命令下达的
        #: 第一帧就从 0 跳到 ``max_speed``，那是无穷大的加速度。
        #: 它同时是"停得下来"那一条：剩余距离 < ``v²/(2·a)`` 时开始减速。
        "linear_accel": Params.accel(2.0, minimum=0.0),
        #: **角加速度上限**（deg/s²，0 = 不限制）——转向通道的第二层。
        #: 角速度从当前值朝上界按它建立，不是一帧跳到 ``turn_rate``。
        #: 少了它，角速度可以瞬间反向，那与"航向瞬转"是同一类缺项，只是
        #: 低了一阶：模型里有"转得多快"却没有"转向建立得多快"。
        "angular_accel": Params.angle_accel(5.0, minimum=0.0),
        #: **垂向加速度上限**（m/s²，0 = 不限制）——垂向通道的第二层。
        #: 只有空中件与水下件接通（它们的垂向由速率控制）；地面件与水面的
        #: z 是位形约束（贴地 / 海平面），这个参数在它们身上不参与计算。
        #: 所以基类默认 0（不限制），由子类给出本型号的量级。
        "vertical_accel": Params.accel(0.0, minimum=0.0),
        #: 多久算一次。给成参数是因为"雷达 0.5 s 扫、后勤 60 s 算"这种
        #: 节拍差异是模型的一部分，不是框架的常数。
        "period": Params.duration(MOVER_PERIOD_US, minimum=1_000),
        #: 离航路点多近算"到了"。防止在格中心附近来回抖（格中心距是
        #: 边长×√3，而一次推进的步长可能远小于它）。
        "arrive_radius": Params.distance(5.0, minimum=0.0),
        #: 一次机动最多重规划几次。地形是按需生成的，路线前方可能直到
        #: 快走到时才真实生成——走不通就得重算，但不能每帧都重算。
        "replan_limit": Params.integer(3, minimum=0),
        #: **过不去的地貌**（名字列表，空 = 什么地貌都能走）。这是"可通行
        #: 能力"，与 "max_speed" 是两回事：轮式与履带在平原上只是速度不同，
        #: 到了山地是能不能去的区别。想定里写 ``blocked_terrain [山地]``。
        "blocked_terrain": Params.strings(
            default=(), choices=BLOCKABLE_TERRAIN
        ),
        #: **最小水深（米，0 = 不限制）**。水面舰写吃水；水下件不写它，
        #: 它的水柱是"定深 + 离底余量"**算出来**的（见
        #: :meth:`required_water_depth`）。陆地格的水深恒为 0，所以把地面件
        #: 的这个参数设成正数等于封死自己——那是使用者写错了，不是框架的
        #: 问题。
        "min_water_depth": Params.distance(0.0, minimum=0.0),
    }

    SLOT_HINT = "mover"

    #: 运载器画像名（:mod:`milsim.services.map.nav` 里的键）。
    #: ``None`` = 不吃地形：对空中单位是对的，天空没有"通行代价"这回事，
    #: 硬给它安一个画像会让人以为天空也能分层寻路（§3.11 已说明不这么做）。
    PROFILE: str | None = PROFILE_GROUND

    __slots__ = (
        "view",
        "_nav",
        "_travel",
        "_goal_cell",
        "_goal_xyz",
        "_waypoints",
        "_leg",
        "_travelled_m",
        "_blocked",
        "_blocked_reason",
        "_replans",
        "_bus",
        "_last_speed",
        "_started_us",
        "_finished_us",
        "_last_tick_us",
        "_step_m",
        #: 三个通道的**速率状态**——第二层的落脚处。它们跨帧保留，所以
        #: "起步要时间""转向要建立"这两件事才真的发生过；每帧从参数表
        #: 重算的话，它们又会退化成阶跃。
        "_speed",
        "_climb",
        "_turn",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self.view = mount.mover_view()
        #: 导航门面。装配层给的是 :class:`~milsim.services.map.nav.NavService`；
        #: 没有它（全球层、或测试里没装地图）就只能按点机动。
        self._nav = mount.nav
        self._bus = mount.bus
        #: **这辆车**的代价配置 = 画像 + 自己的通行门槛。装配时算一次，
        #: 之后寻路/减速/判通行全用它——每帧现算的话，参数改动的生效时刻
        #: 会散在各处，而且多出无谓的对象分配。
        self._travel = self._build_travel()
        #: 分步推进的步长（米）。取格中心距的一小部分：弧线可能斜切过格角，
        #: 步长越小越查得细。**按装配层注入的格距算**，组件不必认识战区
        #: 投影；拿不到格距（战区外）就是 0，此时不分步——见 :meth:`_march`。
        span_m = mount.cell_span_m()
        self._step_m = span_m / MARCH_STEPS_PER_CELL if span_m > 0.0 else 0.0
        self._goal_cell: Axial | None = None
        self._goal_xyz: tuple[float, float, float] | None = None
        self._waypoints: tuple[tuple[float, float, float], ...] = ()
        self._leg = 0
        self._travelled_m = 0.0
        self._blocked = False
        self._blocked_reason = ""
        self._replans = 0
        self._last_speed = 0.0
        #: 三个通道的速率状态。开工时是 0——**不预置成额定值**：预置的话
        #: 实体在命令下达的那一瞬间就已经在满速了，那正是第二层要消掉的东西。
        self._speed = 0.0
        self._climb = 0.0
        self._turn = 0.0
        self._started_us: int | None = None
        self._finished_us: int | None = None
        self._last_tick_us: int | None = None
        mount.every(self.spec["period"], self._on_tick, PRIORITY_MOVER)

    # -- 下达目的地（运行期） ----------------------------------------------

    def move_to_cell(self, cell: Axial) -> bool:
        """按格下达目的地。返回**是否已有一条能走通的路**。

        返回 ``False`` 时实体不会动，也不会偷偷改走直线——见模块头。
        """
        self._goal_cell = cell
        self._goal_xyz = None
        self._restart_trip()
        return self._plan()

    def move_to_point(self, x: float, y: float, z: float) -> bool:
        """按世界坐标下达目的地。**不需要地图**——战区外只有这一种走法。

        坐标是局部平面坐标（米），与 :meth:`~milsim.models.mount.MountContext`
        拿到的是同一套。用经纬度下达目的地属于任务层的事（要先定位到战区）。
        """
        self._goal_cell = None
        self._goal_xyz = (float(x), float(y), float(z))
        self._restart_trip()
        return self._plan()

    def move_along_route(
        self, points: Sequence[tuple[float, float, float]]
    ) -> bool:
        """沿**一整条航线**走。空航线返回 ``False``（什么也不改），其余 ``True``。

        坐标与 :meth:`move_to_point` 同一套（局部平面米 + 高度）。与它的差别
        只有一处，而那一处决定了**能不能跟着一条航线走**：航路点表一次给全，
        而不是每次只给一个。

        为什么非要一次给全——"每个航路点都当最后一个航路点"有两处代价，而且
        两处都不报错：

        1. :meth:`_speed_goal` 里"停得下来"那一条 ``v ≤ √(2·a·dist)`` 只在
           ``self._leg == len(self._waypoints) - 1`` 时算。目的地一个一个下达，
           于是**每一腿都是最后一腿**：每过一个航路点都刹一次车。那是在模拟
           "每个路口都停车"，不是"沿这条路走"。整条航线给全之后，只有真的
           走到终点才减速——这正是 :meth:`_speed_goal` 文档里那句承诺。
        2. 更重的那一处：:meth:`move_to_point` 会走 :meth:`_restart_trip` →
           :meth:`_halt`，**把三个通道的速率状态一起清零**。换一次航路点就是
           一次"重新出发"：速率回 0，而且下一帧只记开工时刻、不挪窝。对一条
           只有几个航路点的机动无所谓；对一条会绕圈的航线就是**每个拐角停一下
           再起步**——绕一圈被切掉几百米，速率剖面上一串周期性凹坑，而它看起来
           只是"这个件有点慢"。

        所以本方法是"**把这条航线交给它**"，不是"把 N 个目的地依次交给它"。
        AFSIM 的 route mover 就是这个口径（``use_route`` + 整条 ``route`` 块），
        本项目里 :meth:`move_to_cell` 也已经是（``_nav.route`` 一次给出一串格
        中心）——缺的只是"按坐标点一次给一串"这条入口。

        调用方要注意**接力语义**：本方法与另两个下达口一样走
        ``_restart_trip``，速率状态归零。绕圈、接力这类"上一趟没走完就接下
        一条航线"的用法，需要调用方自己把速率状态接回去；``stop()` 之后的
        第一次下达不需要任何额外动作。``tools/demo_afsim_mover.py`` 里的
        ``Repro._hand_over`` 就是那个写法的现成例子。
        """
        route = tuple((float(x), float(y), float(z)) for x, y, z in points)
        if not route:
            return False
        self._goal_cell = None
        # 终点仍要写进 `_goal_xyz`：`arrived()` 与 `destination_point()` 都靠它，
        # 而"这一趟有没有走完"是航线层面的问题，不是最后一个航路点的问题。
        self._goal_xyz = route[-1]
        self._restart_trip()
        self._waypoints = route
        self._leg = 0
        self._clear_blocked()
        return True

    def stop(self) -> None:
        """清空目的地与路线。已经走过的里程不归零——那是履历，不是状态。"""
        self._goal_cell = None
        self._goal_xyz = None
        self._waypoints = ()
        self._leg = 0
        self._clear_blocked()
        self._halt()
        self._started_us = None
        self._finished_us = None
        self._last_tick_us = None

    # -- 查询 --------------------------------------------------------------

    def destination_cell(self) -> Axial | None:
        return self._goal_cell

    def destination_point(self) -> tuple[float, float, float] | None:
        return self._goal_xyz

    def waypoints(self) -> tuple[tuple[float, float, float], ...]:
        return self._waypoints

    def next_waypoint(self) -> tuple[float, float, float] | None:
        if self._leg < len(self._waypoints):
            return self._waypoints[self._leg]
        return None

    def heading_deg(self) -> float:
        """当前航向（度，0 = 北，90 = 东）。查询用。

        取的是**写进存储的那个值**，与 :meth:`speed_mps` 同一条理由：读
        存储才是"别人看到的那份"，自己再算一遍会多出一个真相。
        """
        pose = self.view.my_pose()
        return 0.0 if pose is None else float(pose[3])

    def turn_rate_dps(self) -> float:
        """当前速率下的**角速度上界**（度/秒）。0 = 不限制。

        由 ``turn_rate`` 与 ``radial_accel / v`` 取小得到（§5.7.11）。
        查询用：它比单看 ``turn_rate`` 更接近事实——高速时真正卡住转向的
        往往是过载那一项。
        """
        return self._turn_rate_ceiling(self._speed)

    def turn_radius_m(self) -> float:
        """当前速率下的转弯半径（米）。0 = 不限制（半径没有下界）。

        查询用。"这辆车为什么绕不进城"——看一眼这个数（它该与街道宽度
        比，而不是与地图比）比翻参数快。

        取的是**速率状态** ``_speed`` 而不是存储里那一帧的实际速率：转向
        只在"打算以多快走"这个量上有定义，而实际速率在被转向冻结的那一帧
        是 0，用它会把半径算成 0（"原地打转"），恰好把话说反。
        """
        if self._speed <= 0.0:
            return 0.0
        rate = radians(self.turn_rate_dps())
        return 0.0 if rate <= 0.0 else self._speed / rate

    def climb_rate_mps(self) -> float:
        """当前的**垂向速率状态**（m/s，正 = 上升）。查询用。

        地面件与水面件恒为 0：它们的垂向是位形约束，没有速率这一层。
        """
        return self._climb

    def commanded_speed_mps(self) -> float:
        """速率通道的**状态**（m/s）——它此刻"打算"跑多快。查询用。

        与 :meth:`speed_mps` 不是一个数：后者是**上一帧实际**走出来的速率。
        两者在两种情形下会分开，而两种都容易被当成 bug：

        - **掉头帧**：实际是 0（它没在走），状态仍是巡航速率——"先掉头、
          不前进"那一帧不该顺带把速度也清掉（§5.7.11）；
        - **起步与刹车**：状态每帧按 ``linear_accel`` 变，而实际速率是这一帧
          真正走出来的那个值（撞墙只走半步时会比状态小）。

        看状态回答"它以为自己多快"，看 :meth:`speed_mps` 回答"它上一帧
        实际多快"。诊断时两个都要看。
        """
        return self._speed

    def arrived(self) -> bool:
        """到了没有。**没有任何目的地时算"到了"**——否则调用方要写两个判断，
        而漏掉其中一个就会得到"停在原地却被当成在赶路"。

        判据是"航路点走完了" **且** 高度也到位（后者由 :meth:`_at_altitude`
        决定，地面件恒真）。少了高度这一半，飞机就会在**还差 3 km 高度**的
        时候宣布到达并停住——那不是模型简化，那是错的。
        """
        if self._goal_cell is None and self._goal_xyz is None:
            return True
        if self._leg < len(self._waypoints):
            return False
        return self._at_altitude(self._waypoints[-1] if self._waypoints else None)

    def blocked(self) -> bool:
        return self._blocked

    def blocked_reason(self) -> str:
        return self._blocked_reason

    def travelled_m(self) -> float:
        """累计里程（米）。只增不减——它是履历，用来算油耗/磨损一类的事。"""
        return self._travelled_m

    def replans(self) -> int:
        return self._replans

    def elapsed_s(self) -> float:
        """这一趟走了多久（秒）。

        用引擎给的时刻算，不从"里程 / 速度"倒推：倒推出来的平均速度会随着
        采样间隔而变（采到 5 s 一次就少算几秒），而那看起来像"模型跑得比
        额定还快"。
        """
        if self._started_us is None:
            return 0.0
        # 在途时用最近一帧的时刻，不要用开工时刻——那会让"用时"在整段路程里
        # 一直是 0，直到到达才跳到真值。
        end = self._finished_us
        if end is None:
            end = self._last_tick_us if self._last_tick_us is not None else self._started_us
        return max(0.0, (end - self._started_us) / 1_000_000.0)

    def _restart_trip(self) -> None:
        """新目的地 = 新一趟：里程累计保留（履历），本趟计时与重规划额度重开。

        **重规划额度的清零必须在这里，不能在 :meth:`_plan` 里。** 放在
        ``_plan`` 里的话，每次重规划都会先 ``+1`` 再被清零，于是
        ``replans >= replan_limit`` 永远不成立——机动件会**每帧重规划一次、
        永远不动**，而且不发任何信号。这是"参数存在但不起作用"的典型，
        只会表现为"单位原地待着"。
        """
        self._halt()
        self._replans = 0
        self._started_us = None
        self._finished_us = None
        self._last_tick_us = None

    def speed_mps(self) -> float:
        """最近一帧**实际**写进存储的速率（m/s）。0 表示没动。

        注意它现在是**实际值**而不是额定值：加了转向与分步推进之后，"这一帧
        花掉的速度预算"可能小于 ``max_speed``（转向还没对准、被转弯半径限
        速、前方一步走不了）。写额定值的话，存储里的速度与位置就对不上——
        而那两个数都在同一行里，谁也看不出来哪个是假的。

        取的是**写下去的那个值**，不是"按当前格再算一遍"：再算一遍会得出
        "已经停在原地、但按地形还能跑 12 m/s"这种与存储不一致的答案，
        而两个速度就是两个真相。
        """
        return self._last_speed

    # -- 节拍 --------------------------------------------------------------

    def _on_tick(self, engine: Any, event: Any) -> EventResult:
        if self.enabled:
            self.update(engine.now)
        return EventResult.RESCHEDULE

    def update(self, ctx: Any = None) -> None:
        """推进一步。子类一般覆盖 :meth:`_advance` 而不是本方法。

        ``ctx`` 是当前仿真时刻（微秒）。引擎传的就是它——组件不该自己去问
        时钟，那会让"这一帧"与"现在"变成两个时刻。

        **接到命令后的第一帧只记开工时刻，不挪窝。** 一帧代表的是区间
        ``[now, now + period]`` 上的位移，而命令刚下达时这个区间还没过去——
        照走的话，实体在**零时刻瞬移**一个周期的距离（5 s、180 m/s 就是
        900 m）。那点距离在地图上看不出来，但里程与用时会对不上：算出来的
        平均速度比额定速度高十几个百分点，而且**不报错**。
        """
        now = ctx if isinstance(ctx, int) else None
        if now is not None:
            self._last_tick_us = now
        if self.arrived():
            self._halt(now)
            return
        if self._started_us is None:
            self._started_us = now
            return
        pose = self.view.my_pose()
        if pose is None:
            return
        self._advance(float(self.spec["period"]) / 1_000_000.0, pose)
        if self.arrived():
            # 到的那一刻就把速度清零，不要留到下一帧。留一帧的话，在
            # [到达, 下一帧) 这段区间里位置写着"已到"而速度写着"在跑"，
            # 采样落在这一段的观察者会拿到一个不存在的事实；而推演如果
            # 恰好在这一帧收尾，那个错值就永远留在结果里了。
            self._finished_us = now
            self._halt(now)

    def _halt(self, now: int | None = None) -> None:
        """停下来之后把速度字段清零。

        不写这一步的话，存储里的速度会**停在最后一次推进时的值**——一件
        已经到达的装备会永远显示在 180 m/s 上飞。它不报错，只是让所有
        读速度的地方（视图、耗油、被探测判定）拿到一个不存在的事实。

        **三个通道的速率状态也一起归零**，而且在那个"速度已经是 0 就早退"
        的判断之前：留下 ``_speed = 25`` 的话，下一次接到命令时实体是**从
        25 m/s 起步**的（它明明停着），而存储里那个 0 会立刻变成假的——
        两个真相，其中一个还不报错。
        """
        self._speed = 0.0
        self._climb = 0.0
        self._turn = 0.0
        if self._finished_us is None:
            self._finished_us = now
        if self._last_speed == 0.0:
            return
        pose = self.view.my_pose()
        self._last_speed = 0.0
        if pose is None:
            return
        x, y, z, heading, _ = pose
        self.view.set_pose(x, y, z, heading, 0.0)

    # -- 推进 --------------------------------------------------------------

    def _advance(self, dt_s: float, pose: tuple[float, float, float, float, float]) -> None:
        """把这一帧的位移预算花在航路点上。

        一帧走不完一段是常态，一帧走完好几段也是（路点密、周期长）。所以
        用 while 而不是"每帧一个路点"：后者在格边长小于步长时会**永远追不上**
        第一个路点，实体看起来粘在原地。

        三维质点模型的三个通道都在这里，顺序固定：**先转向**（角速度有上界，
        且角速度本身按角加速度建立）、**再前进**（沿**新**航向走，速率按直线
        加速度朝本帧的目标逼近）、**最后定高**（交给子类的 :meth:`_approach_z`）。
        顺序不能换：先按旧航向走再转，等于这一帧白走；先定高再算距离，等于把
        爬升当成了水平位移。

        **每个通道都是"先定这一帧想要的速率，再按加速度逼近它"**。逼近是
        关键：目标可以一帧变很多，实际速率不行。
        """
        start_x, start_y = x, y = pose[0], pose[1]
        z, heading = pose[2], pose[3]
        ref = self.view.my_cell()

        if self._replan_if_blocked(ref) or self._blocked:
            # 走不通就**停住**。重规划额度用完、或上一次规划就已经判定走不通时
            # 继续推进，会让实体沿着一条不存在于地形的路线穿湖穿山——那是
            # 模块头说的"看起来在动的假象"，比停住坏得多。
            return

        accel = float(self.spec["linear_accel"])
        budget = 0.0
        first = True

        while self._leg < len(self._waypoints):
            wx, wy, wz = self._waypoints[self._leg]
            if self._waypoint_unwalkable(ref, (wx, wy, wz)):
                # 前方这一格不能走：本帧就停在这儿，下一帧交给重规划。
                # 不检查的话，一帧跨几个路点时实体会**落脚在水里或山上**，
                # 而下一帧的路点已经越过它了，再也查不到。
                break

            dx, dy = wx - x, wy - y
            dist = hypot(dx, dy)
            desired = offset_to_heading(dx, dy)
            error = signed_angle(desired - heading)

            # -- 转向通道：角速度有上界，而角速度本身按角加速度建立 --
            heading, error, _ = self._steer(heading, error, dt_s)

            if abs(error) > STEER_BEFORE_MOVE_DEG:
                # 目标在侧后方：沿当前航向推进是**在远离它**，那一段全是白走
                # 的。原地转向不是"模型偷懒"，转弯本来就要花时间——代价是这
                # 一帧里程为 0、用时照走，于是平均速度低于峰值。
                #
                # 这一帧**速率状态原样保留**（不朝目标推，也不朝 0 推）：
                # 它在掉头，不在走，也没有在减速。把它一起降到 0 的话，每次
                # 转向都会附带一次重新加速——对起步慢的件（潜艇 0.08 m/s²）
                # 那是灾难性的，而"先掉头"这条抽象本来是想省掉这个代价。
                break

            if first:
                first = False
                # -- 速率通道：目标取"三件事里最小的那个"，再按加速度逼近 --
                self._speed = self._ramp(
                    self._speed, self._speed_goal(ref, dist, error), accel, dt_s
                )
                budget = self._speed * dt_s

            aligned = abs(error) <= 1e-9
            capture = max(budget, float(self.spec["arrive_radius"]))
            if dist <= capture:
                # **航路点捕获**：这一帧的位移本来就够得着它，就吸附过去。
                #
                # 捕获半径必须取 ``max(一帧的位移, arrive_radius)``，而**不能**
                # 额外要求"已经对准航向"。要求对准的话，转弯半径大于路点间距的
                # 件（潜艇：690 m 的半径、346 m 的格距）会**绕着路点转圈**：
                # 每一帧都在动、到路点的距离却不减，永远到不了，而且不报错、
                # 不置 blocked。限制转向的代价不该是"它再也到不了目的地"。
                #
                # 代价是最后这一帧的位移方向与航向不一致（最多差一帧的量）。
                # 这一步本来也走不出格——捕获半径就是一帧的位移加上一个
                # 格内的容差，所以它不会把实体送过一格去。
                x, y = wx, wy
                budget = max(0.0, budget - dist)
                self._leg += 1
                continue

            if budget <= 0.0:
                # 这一步是给"速率状态正好是 0"留的出口：此时路点够不着，
                # 但**捕获判据刚刚查过**（上面那一句），所以停在 5 m 之外
                # 不会再往前走——否则它永远到不了。下一帧速率重新建立。
                break

            travel = min(budget, dist)
            x, y, used, wall = self._march(ref, x, y, heading, travel)
            budget -= used

            if wall is not None:
                if used <= 0.0 and aligned:
                    # 航向已经对准、却连一小步都迈不进去：不是转不过来，是
                    # 这一格真的走不了。说不出原因就成了"原地不动且不报错"。
                    self._set_blocked(self._why_unwalkable(ref, wall))
                # 弧线切进了不可通行地形：本帧到此为止，下一帧重新测距与
                # 转向（航向会更贴近目标，弧也就更贴）。
                break
            if not aligned:
                # 没对准时走的是弧，不能在同一帧里接着去追下一个路点——
                # 下一段的朝向得从新位置重新算。
                break
            if used <= 0.0:
                break

        # 高度最后定：水平那一段已经走完了，"这一步怎么到目标高度"要按走完
        # 之后的位置算（海床、地面高程都随位置变）。
        z = self._approach_z(ref, x, y, z, dt_s)
        # 写**实际**速率而不是额定速率：这一帧可能只转了向、可能被限速、
        # 可能撞墙只走了半段。存储里的速度与位置在同一行里，写额定值的话
        # 两个数对不上，而没有任何东西会报错。
        moved = hypot(x - start_x, y - start_y)
        self._commit(
            x, y, z, heading, moved / dt_s if dt_s > 0.0 else 0.0
        )

    # -- 三个通道：速率与加速度 --------------------------------------------

    @staticmethod
    def _ramp(current: float, goal: float, accel: float, dt_s: float) -> float:
        """把速率 ``current`` 朝目标 ``goal`` 推，一帧最多变 ``accel × dt``。

        ``accel <= 0`` = **不限制**（一帧到位）——与 ``turn_rate 0`` /
        ``min_water_depth 0`` 同一条约定：门槛参数的 0 表示"不限制"，不是
        "不能动"。读成"不能动"的话，想定里漏写一个 ``linear_accel`` 会让
        实体**永远停在 0 速**，而且不报错。

        加速与减速共用同一个上限（对称限幅）。真实的刹车往往比起步更狠，
        但那是下一步的事：现在多出来的 ``brake_accel`` 会在没有实测依据的
        情况下先变成一个人为选择。
        """
        if accel <= 0.0:
            return goal
        step = accel * dt_s
        if goal > current:
            return min(goal, current + step)
        return max(goal, current - step)

    def _turn_rate_ceiling(self, speed: float) -> float:
        """速率 ``speed`` 下的角速度上界（度/秒）。0 = 不限制。

        两项取小，两条约束互不相干：

        | 来源 | 表达式 | 什么时候它说了算 |
        |---|---|---|
        | 结构 / 气动过载 | ``radial_accel / v`` | 高速：速度越高允许的角速度越小 |
        | 转向机构 | ``turn_rate`` | 低速：机构本身有个绝对上限 |

        两项都留空（都是 0）就是"角速度无上界"——想定里显式这么写才回到
        旧行为，而且**仍受 ``angular_accel`` 约束**（角速度建立要时间）。

        ``speed ≈ 0`` 时过载那一项**不参与**：``radial_accel / v`` 在 v→0 时
        发散。它发散的方向无害（不限制），但那个数是虚的——v→0 时离心力为
        零、根本不需要径向加速度，所以这一项在该点本来就不该出现。
        """
        caps: list[float] = []
        mech = float(self.spec["turn_rate"])
        if mech > 0.0:
            caps.append(mech)
        radial = float(self.spec["radial_accel"])
        if radial > 0.0 and speed > 1e-9:
            caps.append(degrees(radial / speed))
        return min(caps) if caps else 0.0

    def _steer(
        self, heading: float, error: float, dt_s: float
    ) -> tuple[float, float, float]:
        """转向通道走一步。返回 ``(新航向, 剩余误差, 本帧角速度)``。

        角速度**本身也有惯性**：它从当前值朝目标按 ``angular_accel`` 建立，
        不是一帧跳到上界。目标取 ``min(|误差| ÷ dt, 上界)``——误差小就正好
        一步对准，误差大就顶到上界。

        两处**不许转过头**：① 角速度推上去之后夹回上界；② 本帧转角超过误差
        时直接取误差、并把角速度改写成"正好够"的那一个。少了 ②，转弯半径大
        的件会在路点两侧来回摆——每次过头同样多，看起来像在抖。
        """
        ceiling = self._turn_rate_ceiling(self._speed)
        accel = float(self.spec["angular_accel"])
        if ceiling <= 0.0 and accel <= 0.0:
            # 两层都不限制：一帧对准。这是想定里同时写 ``turn_rate 0`` 与
            # ``angular_accel 0`` 的情形，也是"航向瞬转"那个旧行为的开关。
            self._turn = error / dt_s if dt_s > 0.0 else 0.0
            return (heading + error) % 360.0, 0.0, self._turn
        want = error / dt_s if dt_s > 0.0 else 0.0
        if ceiling > 0.0:
            want = max(-ceiling, min(ceiling, want))
        self._turn = self._ramp(self._turn, want, accel, dt_s)
        if ceiling > 0.0:
            self._turn = max(-ceiling, min(ceiling, self._turn))
        turn = self._turn * dt_s
        if abs(turn) > abs(error):
            turn = error
            self._turn = turn / dt_s if dt_s > 0.0 else 0.0
        return (heading + turn) % 360.0, error - turn, self._turn

    def _speed_goal(self, ref: Any, dist: float, error: float) -> float:
        """前向通道这一帧的**目标**速率（m/s）。三件事取小：

        1. **当地通行系数**给的巡航速率（:meth:`_speed_at`）——地貌决定的
           "这条路上想跑多快"；
        2. **转弯几何**（:meth:`_turn_speed_limit`）：转弯半径不能大于
           "够得着目标"那个上界 ``dist ÷ (2·sin θ)``；
        3. **停得下来**：``v ≤ √(2·a_lin·dist)``。**只在最后一段算**——
           每过一个中间航路点都刹一次车，那是在模拟"每个路口都停车"，
           而不是"沿这条路走"。

        返回的是**目标**，不是一个立即生效的上限：实际速率按 ``linear_accel``
        逼近它。跟不上的时候实体冲过头、绕回去——那是真实超调，不是 bug。
        """
        goal = self._speed_at(ref)
        if goal <= 0.0:
            return goal
        goal = min(goal, self._turn_speed_limit(dist, error))
        if self._leg == len(self._waypoints) - 1:
            accel = float(self.spec["linear_accel"])
            if accel > 0.0:
                goal = min(goal, sqrt(2.0 * accel * max(0.0, dist)))
        return goal

    def _turn_speed_limit(self, dist: float, error: float) -> float:
        """转弯几何给出的速率上限（m/s）。``inf`` = 不限。

        **几何**：目标在航向误差 θ 的那一侧、距离 ``dist``。要**够得着**它，
        转弯半径 ρ 必须**不超过** ``dist ÷ (2·sin θ)``；而半径 = 速率 ÷ 角速度
        上界。角速度上界本身还可能是速率的函数，所以两个来源要**分别解出来
        再取小**：

        .. code-block:: text

            固定 turn_rate      ω 与 v 无关 →  v ≤ ω·dist / (2·sin θ)
            固定 radial_accel   ω = a / v   →  v ≤ √(a·dist / (2·sin θ))

        推导（``2·sin θ`` 而不是 ``2·sin(θ/2)``，这一点 v0.13.15 修正过）：
        沿半径 ρ 的圆弧转弯时，转过转角 φ 之后位移是 ``(ρ·sinφ, ρ(1−cosφ))``
        ——**弦与初始航向的夹角是 φ/2，不是 φ**。目标在 θ 方向，所以 φ = 2θ，
        弦长 ``dist = 2ρ·sin(φ/2) = 2ρ·sin θ``，于是 ``ρ = dist/(2·sin θ)``。

        走"先直线再圆弧"也一样：直线 a 之后以 ρ 转 φ，则
        ``ρ = dist·sinθ/(1−cosφ)`` 且 ``a = dist·[cosθ − sinθ·cot(φ/2)]``；
        ``a ≥ 0`` 要求 φ ≥ 2θ，而 ρ 在 φ = 2θ 处取最大——还是 ``dist/(2·sinθ)``。
        所以它是**能抵达**的充要上界：ρ 更大就一定能抵达，更小就够不着。
        目标是**点**时它成立；点后面的那一截不算在内，因为够不着就不是
        "走到附近"，而是"冲过头再绕回来"。

        **旧式 ``2·sin(θ/2)`` 是错的**（v0.13.14 到 v0.13.15 之间用的就是这个）：
        它把弦角当成了 φ 本身，于是半径上限被放大到 ``1/cos(θ/2)`` 倍——
        θ = 30° 时是 1.03 倍、θ = 90° 时是 1.41 倍，**最大 2 倍**。
        实测代价（潜艇，A* 路线 31 格 / 10.4 km）：θ/2 口径下每一帧都以
        12 m/s 冲出 ~60 m、误差立刻涨破 30° 的掉头门限，于是"走一帧、停两帧"，
        64% 的帧里程为 0，用时 3560 s、绕行比 1.235；改成 2·sin θ 之后
        1350 s、绕行比 1.171、巡航帧从 12.5% 升到 50.7%。

        （顺带一笔，免得下次又"修"回去：更旧的 ``ω·dist/|θ|`` 与
        ``2·sin(θ/2)`` 在 0~90° 上**数值上几乎相同**——``2·sin(θ/2) ≈ θ``，
        30° 处差 1%、15° 处差 0.1%。当年那一次"用弦不用弧"的修正其实什么
        都没改，真正错的是弦角取成了 φ 而不是 φ/2。）

        θ → 0 时退化成"不限"（直着走当然够得着）；θ → 180° 时也退化成
        "不限"——目标在正后方时，任何半径的圆弧都能绕到它，这一点与直觉
        相反但确实是对的，所以 ``twist → 0`` 两端都返回 ``inf``。

        少了它，航向还没转过来就先冲出去，轨迹会**大幅偏离**那条直的路点
        连线。有了它，接近目标时自动减速（半径随速度一起缩小），进得了弯。
        飞机最后几公里明显减速就是这个原因，那是物理，不是 bug。
        """
        if dist <= 0.0 or error == 0.0:
            return inf
        twist = 2.0 * sin(radians(abs(error)))
        if twist <= 0.0:
            return inf
        bounds: list[float] = []
        mech = float(self.spec["turn_rate"])
        if mech > 0.0:
            bounds.append(radians(mech) * dist / twist)
        radial = float(self.spec["radial_accel"])
        if radial > 0.0:
            bounds.append(sqrt(radial * dist / twist))
        return min(bounds) if bounds else inf

    def _glide_z(
        self, z: float, target: float, up_rate: float, down_rate: float, dt_s: float
    ) -> float:
        """垂向通道走一步：先定"这一帧想以多快升降"，再按 ``vertical_accel``
        逼近它。返回新的 z。

        上下行各有自己的速率上限（``climb_rate`` / ``descent_rate``、
        ``surface_rate`` / ``dive_rate``），加速度上限共用一个
        ``vertical_accel``。**加速度这一层是 v0.13.15 补的**：补之前爬升率
        是阶跃的——平飞转满爬升率发生在**一帧之内**，与起步端那个
        "0 → 25 m/s" 是同一类缺项（垂向加速度无穷大），只是它藏在子类的
        钩子里不显眼。

        到位那一下**不许越过目标**，而且不算"减速"：``_climb`` 直接改写成
        "正好够走到目标"的那个值。这是刻意的——高度保持本来就是一套独立
        回路，允许比水平方向更硬地收尾；让加速度去管这件事只会在目标高度
        附近来回超调。夹的是**速率**，不是位置。
        """
        if dt_s <= 0.0:
            return z
        accel = float(self.spec["vertical_accel"])
        dz = target - z
        if dz == 0.0:
            self._climb = self._ramp(self._climb, 0.0, accel, dt_s)
            return z
        want = max(-down_rate, min(up_rate, dz / dt_s))
        self._climb = self._ramp(self._climb, want, accel, dt_s)
        if abs(self._climb * dt_s) > abs(dz):
            self._climb = dz / dt_s
        return z + self._climb * dt_s

    def _march(
        self, ref: Any, x: float, y: float, heading: float, distance: float
    ) -> tuple[float, float, float, Any]:
        """沿航向走 ``distance``，遇到走不了的格就**停在那之前**。

        返回 ``(新 x, 新 y, 实际走了多少米, 挡住去路的那一格或 None)``。

        为什么要分步：**路点是可通行的，但弧线不是**。航向有了角速度上界
        之后，实体的轨迹不再是路点之间的折线，而是带圆角的弧——弧会斜切过
        格角，而那个角上可能正是水域或山脊。只查"下一个路点"查不到它
        （路点本身是好的），实体就站在里面了，而且没有任何信号。

        拿不到格距（战区外、或没有导航门面）时不分步、也不拦：没有逐格精度
        的地方谈不上"这一格能不能走"，凭空拦住反而会挡住按点机动的调用方。
        """
        if distance <= 0.0:
            return x, y, 0.0, None
        if self._travel is None or self._nav is None or self._step_m <= 0.0:
            dx, dy = heading_to_offset(heading, distance)
            return x + dx, y + dy, distance, None

        done = 0.0
        while done < distance:
            step = min(self._step_m, distance - done)
            dx, dy = heading_to_offset(heading, step)
            nx, ny = x + dx, y + dy
            cell = self._nav.axial_at(ref, nx, ny)
            if cell is not None and not self._nav.passable(
                ref, cell, profile=self._travel
            ):
                return x, y, done, cell
            x, y = nx, ny
            done += step
        return x, y, done, None

    def _waypoint_unwalkable(self, ref: Any, point: tuple[float, float, float]) -> bool:
        """这个航路点所在格，本人走不了吗。

        没有画像（空中）或没有导航门面时恒为 ``False``——天空没有"不可通行"
        这回事，没地图时也不该凭空拦住按点机动的调用方。

        ``self._travel is None`` 同时涵盖"不吃地形"（空中）与"没有地图"
        两种情形：它正是 :meth:`_build_travel` 在这两种情况下返回的东西。
        """
        if self._travel is None:
            return False
        cell = self._nav.axial_at(ref, point[0], point[1])
        if cell is None:
            return False
        return not self._nav.passable(ref, cell, profile=self._travel)

    def _commit(self, x: float, y: float, z: float, heading: float, speed: float) -> None:
        """写回位置与速度，并累计里程。

        所有位置变更都走这一个出口。散在各处直接 ``set_pose`` 会漏掉三件事，
        而漏掉每一件的后果都不报错：

        1. 空间索引的同步（在 ``MoverView.set_pose`` 里）；
        2. "当前速度"的记账——到达时要靠它决定该不该把速度清零，
           漏了就是"实体已经停下但速度还写着 180 m/s"；
        3. 里程。**里程按三维距离算**，不是水平投影：飞机爬升 4 km、车翻过
           一道梁，走的都比水平投影远。做成逐段的水平累加会得到"里程等于
           直线距离"这种看着没问题、实际少算的数字。

        不要用 ``MoverView.advance()``：它按**运动学表里**的速度推进，而表里的
        速度是装配时写进去的 0——想控制速度就得先把速度写进去，多一层隐式状态。
        """
        previous = self.view.my_position()
        if previous is not None:
            self._travelled_m += hypot(
                hypot(x - previous[0], y - previous[1]), z - previous[2]
            )
        self.view.set_pose(x, y, z, heading, speed)
        self._last_speed = speed

    def _speed_at(self, ref: Any) -> float:
        """这一格上的速度。地形越难走越慢——**用寻路那张代价表的倒数**，
        不另立一张速度表（见 :meth:`~milsim.services.map.nav.NavService.move_factor`）。"""
        base = float(self.spec["max_speed"])
        if base <= 0.0 or self._travel is None:
            return base
        return base * self._nav.move_factor(ref, profile=self._travel)

    # -- 通行能力 ----------------------------------------------------------

    def _build_travel(self) -> Any:
        """画像 + 这辆车的门槛 → 属于它自己的代价配置。

        门槛走 :meth:`~milsim.services.map.nav.NavService.capable_profile`，
        只传**地貌名字**——"地貌名 ↔ 编号"的对应关系只在地图服务里存一份，
        组件不该跟着维护第二份（地图加一种地貌时，这边会静默漏掉它）。
        """
        if self.PROFILE is None or self._nav is None:
            return None
        return self._nav.capable_profile(
            self.PROFILE,
            blocked_terrain=tuple(self.spec["blocked_terrain"]),
            min_water_depth=self.required_water_depth(),
        )

    def required_water_depth(self) -> float:
        """这辆运载器需要多深的水（米）。寻路用的门槛就是它。

        水面件就是吃水（``min_water_depth`` 参数本身）。水下件覆盖它：
        潜艇要的水柱 = **定深 + 离底余量**——那个数随"今天潜多深"变，
        不是船体常数。所以门槛必须从这里取，而不是各写一遍
        ``self.spec["min_water_depth"]``：两处取数迟早不同步，而症状是
        "寻路绕开了，但实际走的那一脚已经钻进海底了"。
        """
        return float(self.spec["min_water_depth"])

    def travel_config(self) -> Any:
        """这辆车的代价配置（画像 + 自己的门槛）。没有地图服务时为 ``None``。

        查询用：诊断"轮式为什么不去山地"时，看一眼
        ``travel_config().blocked_terrain`` 比翻想定快。
        """
        return self._travel

    # -- 规划 --------------------------------------------------------------

    def _plan(self) -> bool:
        """目的地 → 航路点表。走不通返回 ``False`` 并置 ``blocked``。"""
        self._waypoints = ()
        self._leg = 0

        if self._goal_xyz is not None:
            self._waypoints = (self._goal_xyz,)
            self._clear_blocked()
            return True

        if self._goal_cell is None:
            self._clear_blocked()
            return True

        if self._nav is None:
            self._set_blocked("没有地图服务：战区外只能按坐标点机动")
            return False

        ref = self.view.my_cell()
        if ref is None:
            self._set_blocked("实体还没有格归属")
            return False

        result = self._nav.route(ref, self._goal_cell, profile=self._travel)
        if result is None:
            self._set_blocked(self._no_route_reason(ref))
            return False

        points: list[tuple[float, float, float]] = []
        # 起点格跳过：实体的世界坐标未必正好落在格中心，先"走过去"会让
        # 第一步朝格中心偏一下，看起来像抖了一下
        for cell in result.cells[1:]:
            center = self._nav.cell_center(ref, cell)
            if center is None:
                continue
            points.append(
                (center[0], center[1], self._waypoint_z(ref, center[0], center[1]))
            )

        self._waypoints = tuple(points)
        if not points and self._goal_cell != getattr(ref, "axial", None):
            self._set_blocked("路线为空且没到目的地")
            return False
        self._clear_blocked()
        return True

    def _no_route_reason(self, ref: Any) -> str:
        """说不出"为什么走不通"时，至少要说**准**是哪一头的问题。

        原先这里是一句写死的"起点或终点不可通行、或被完全包围"。它曾经是
        **假的**：真正的原因常常是"走廊那片地形还没生成"，而没生成的地形
        读出的是水域（见 :mod:`milsim.services.map.nav` 的模块头）。既然
        现在走廊会先加载，写死的理由就只是在掩盖下一个同类问题——所以这里
        逐条查证，并把**具体原因**带出来（地貌名、水深之类的数字），
        比"不可通行"省掉一次翻地图的工夫。
        """
        goal = self._goal_cell
        assert goal is not None                      # 调用方保证
        where = f"({goal.q}, {goal.r})"
        if not self._nav.passable(ref, goal, profile=self._travel):
            return f"目的格 {where} {self._why_unwalkable(ref, goal)}"
        mine = getattr(ref, "axial", None)
        if mine is not None and not self._nav.passable(ref, mine, profile=self._travel):
            return (
                f"起点格 ({mine.q}, {mine.r}) {self._why_unwalkable(ref, mine)}"
            )
        return f"到 {where} 没有可行路线（被水域或山地隔开）"

    def _why_unwalkable(self, ref: Any, cell: Axial) -> str:
        """这一格走不了的**原因**。说不出具体的就退回"不可通行"。

        不能写死"是水域或山地"：真正挡住潜艇的常常是**水深**，而那一格在
        地貌上仍然写着"水域"——写死的一句会让作者去翻一张查不出问题的地图。
        原因由导航门面统一给出（它是"能不能走"的同一个视图的另一面），
        这里只负责在它说不出话时别留空。
        """
        why = self._nav.refusal(ref, cell, profile=self._travel)
        if why:
            return why
        terrain = self._nav.terrain_name(ref, cell)
        return f"是{terrain}，这类运载器走不了" if terrain else "不可通行"

    def _replan_if_blocked(self, ref: Any) -> bool:
        """下一段走不通就地重规划。返回是否已经重规划（调用方应放弃本帧推进）。

        地形是**按需生成**的：规划时前方的大片区域可能还没生成，等实体
        快走到时那块才真实生成，届时可能整段路都不可通行。不重规划的
        后果不是报错，而是实体沿一条不存在于地形的路线一直走下去。
        """
        if self._travel is None or self._goal_cell is None:
            return False
        if self._leg >= len(self._waypoints) or self._blocked:
            return False

        target = self._waypoints[self._leg]
        cell = self._nav.axial_at(ref, target[0], target[1])
        if cell is None or self._nav.passable(ref, cell, profile=self._travel):
            return False

        limit = int(self.spec["replan_limit"])
        if self._replans >= limit:
            # 额度用完还不能走：**置走不通并停住**，不要"退回去照原路走"。
            # 照原路走的后果是穿过那片不可通行的地形，而且不报错——比停在
            # 原地糟得多，因为上层的任务台账会以为进展顺利。
            self._set_blocked(
                f"连续重规划 {self._replans} 次后前方仍不可通行"
                f"（replan_limit={limit}）"
            )
            return True

        self._replans += 1
        self._plan()
        return True

    # -- 子类钩子 ----------------------------------------------------------

    def _waypoint_z(self, ref: Any, x: float, y: float) -> float:
        """航路点的高度。地面件用当地地面高程，空中件用巡航高度。

        默认由 :meth:`_surface_z` 给出——"我想待在哪个高度"对绝大多数件
        都不依赖位置，所以只需要实现一处。
        """
        return self._surface_z(ref, x, y, 0.0)

    def _surface_z(self, ref: Any, x: float, y: float, z: float) -> float:
        """本件**想要**待在哪个高度。默认不动 z（按点机动的调用方自己负责）。"""
        return z

    def _approach_z(
        self, ref: Any, x: float, y: float, z: float, dt_s: float
    ) -> float:
        """这一帧结束时 z 在哪。默认**一步到**（即 :meth:`_surface_z`）。

        地面件一步到是对的：它的 z 由位置唯一决定（"这一格的地面多高"），
        中途没有"还在爬"这个状态。空中件与水下件的 z 是**指令给定的目标**，
        得从当前高度按速率走过去——一步到等于瞬移：从海面跳到 -60 m、或从
        0 爬到 4000 m 会发生在**一帧之内**，里程与用时随即对不上，而且
        没有任何报错。所以它们覆盖本钩子，而不是覆盖整个 :meth:`_advance`：
        段推进、转向、跨格、落脚点检查那几段是完全共用的。
        """
        return self._surface_z(ref, x, y, z)

    def _at_altitude(self, target: tuple[float, float, float] | None) -> bool:
        """水平到位之后，高度是否也已经到位。默认 True。

        只有空中件与水下件需要覆盖它：地面件的 ``_surface_z`` 每帧把 z 拉回
        地面，高度天然是"到位"的。
        """
        return True

    # -- 走不通 ------------------------------------------------------------

    def _set_blocked(self, reason: str) -> None:
        """置"走不通"。

        **状态与原因都没变时不发信号。** 光看"现在是不是 blocked"不够：
        同一条走不通的路线被反复下达（每次 ``move_to_cell`` 都会重算）
        会一条条发出去，订阅方收到的全是同一件事——而 :meth:`_emit_blocked`
        的承诺是"只在状态跳变时发"。原因变了仍然要发：那对订阅方是新信息
        （"原先封路的是湖泊，现在是山"），不是重复。
        """
        if self._blocked and self._blocked_reason == reason:
            return
        self._blocked = True
        self._blocked_reason = reason
        self._emit_blocked(True, reason)

    def _clear_blocked(self) -> None:
        if not self._blocked:
            return
        self._blocked = False
        self._emit_blocked(False, "")

    def _emit_blocked(self, blocked: bool, reason: str) -> None:
        """只在**状态跳变**时发信号。每帧都发等于没有信号——订阅方会被迫
        自己去做去重，而那是发布方本该做的事。"""
        if self._bus is None:
            return
        self._bus.mover_blocked.emit(self.entity_id, blocked, reason)

    def describe(self) -> str:
        state = "走不通" if self._blocked else ("已到" if self.arrived() else "在途")
        return f"{self.type_name or type(self).__name__}({self.slot or '?'}·{state})"


__all__ = [
    "MARCH_STEPS_PER_CELL",
    "MOVER_PERIOD_US",
    "BLOCKABLE_TERRAIN",
    "STEER_BEFORE_MOVE_DEG",
    "Mover",
    "signed_angle",
]
