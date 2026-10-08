"""水下机动件：定深巡航，z 在水面之下，而且不许钻进海底。"""

from __future__ import annotations

from typing import Any

from ...services.map.nav import PROFILE_WATER
from ...services.params import Params
from ...services.type_registry import register_component
from .base import Mover


@register_component("SUBSURFACE_MOVER")
class SubsurfaceMover(Mover):
    """水下机动：定深巡航，z 在水面之下，而且不许钻进海底。

    与 ``WATER_MOVER`` 是**并列**关系而不是继承（两者各一个文件，都直接
    继承基类）。两者共用同一张水面画像（水域可走、陆地不可、不吃坡度），
    差别全在高度那一半——一个待在水面、一个待在水下，方向恰好相反。
    继承会让"潜艇是水面舰"变成一句写在代码里的断言，而那句话是错的。

    三个参数决定它在哪
    ------------------
    | 参数 | 是什么 | 默认 |
    |---|---|---|
    | ``cruise_depth`` | **定深**（米，海平面以下，z 取它的相反数） | 60 |
    | ``hull_clearance`` | **离底余量**（米）：龙骨离海底至少这么远 | 10 |
    | ``dive_rate`` / ``surface_rate`` | 下潜 / 上浮速率（m/s） | 2 / 4 |

    **定深是"按格下达目的地"时的默认深度。** 按坐标点下达
    （:meth:`~milsim.models.mover.Mover.move_to_point`）时，命令给的那个
    z 就是目标深度——与空中件"按格用巡航高度、按点用命令给的 z"是同一条
    规则。所以改潜深不需要（也没有）另一个入口：那是任务层下一条命令的事。

    **门槛是算出来的，不是另写一个数**：``水深 ≥ 定深 + 离底余量``。所以
    ``min_water_depth`` 在这个件上取
    ``max(声明值, 定深 + 离底余量)``——声明的那个仍然可以更严（"本艇不得进入
    水深不足 200 m 的海区"），但**不可能更松**。这样"忘了写吃水"这类漏配
    不会让潜艇钻进海底：它只是没有额外的下限，而不是没有下限。

    为什么要取严而不是各管各的：定深是**任务/型号**给的，吃水是**船体**给的，
    两者是不同的事；但"这一格能不能去"只有一个答案。分开判会得到"寻路觉得
    能走、实际走的时候在海底里"，而那是**两个真相**。

    z 的可行区间是双边的
    --------------------
    ``−(水深 − 离底余量) ≤ z ≤ 0``：水深门槛保证这个区间非空，每帧夹取保证
    它真的被守住。夹取不是多余的——**按坐标点下达目的地**
    （:meth:`~milsim.models.mover.Mover.move_to_point`）不经过寻路，中途跨过
    的格没有任何人查过，少了这一夹潜艇会无声地走在海底以下。

    转向速率覆盖成 1 °/s
    --------------------
    12 m/s 的航速下转弯半径约 690 m。潜艇的转向本来就是三件装备里最迟钝的
    一个（舵效随航速下降、艇体很长），而基类默认的 30 °/s 会让它以 23 m 的
    半径原地打转。这一条与水面件同理：把"这是潜艇"落到数字上。

    潜艇也**不需要** ``radial_accel`` ——它的航速基本不变，而"转弯半径与
    航速无关"（战术直径）这条与固定 ``turn_rate`` 是等价的。过载口径是给
    速度会变的件（空中件、车轮件）用的（§5.7.11）。

    三个通道各自的第二层
    --------------------
    ``linear_accel 0.08`` / ``vertical_accel 0.2`` / ``angular_accel 0.05``：
    一条 150 s 才加到全速的艇，下潜速率 10 s 建立、转满舵 20 s 建立。这三条
    不是装饰——把任一写成 0（= 不限制）都会让那**一个**通道退化成阶跃，
    于是"机动"在那条通道上又变成瞬变。

    没做的
    ------
    温跃层、声学传播、浮力与纵倾、定深控制回路的超调、海底对航速的影响
    （越浅越慢）都没有。现在深度只是一个**能待在哪**的约束，不是一套
    水动力学。另外"到点之后自动起浮"这类动作也没有：那要由任务层显式再下
    一条命令（按坐标点给一个 z），模型不替指挥员决定什么时候露头。
    """

    __slots__ = ("_depth_limited",)

    PROFILE = PROFILE_WATER

    PARAMS = {
        **Mover.PARAMS,
        #: 潜艇的水下航速：比水面舰慢。默认取核潜艇的巡航量级（约 23 kn）。
        "max_speed": Params.speed(12.0, minimum=0.0),
        "turn_rate": Params.angle_rate(1.0, minimum=0.0),
        #: 水下直线加速度 0.08 m/s²（约 0.008 g）——0→12 m/s 要 150 s。
        #: 核潜艇的加速就是这么慢：它靠的是排水量，不是推力。
        "linear_accel": Params.accel(0.08, minimum=0.0),
        #: 垂向加速度 0.2 m/s²：下潜速率 2 m/s 要 10 s 建立。
        #: 注水与吹除都是**有限流率**的过程，一帧跳到满速率是把这个流率
        #: 当成了无穷大（§5.7.11）。
        "vertical_accel": Params.accel(0.2, minimum=0.0),
        #: 转向建立 0.05 °/s²：满舵角速度 1 °/s 要 20 s。艇体长、舵效随航速
        #: 下降，这一条与"转弯半径 690 m"是同一件事的两面。
        "angular_accel": Params.angle_accel(0.05, minimum=0.0),
        #: **定深**（米，海平面以下）。默认值是**一个示例工作深度，不是某个
        #: 型号的实值**——想定与参数库里按型号改。给 0 会让这个件与
        #: WATER_MOVER 完全一样（贴着水面走），那才是更容易出事的默认。
        "cruise_depth": Params.distance(60.0, minimum=0.0),
        #: **离底余量**（米）：龙骨与海底之间最少留这么多。它既是每帧夹取的
        #: 余量，也参与门槛（定深 + 它 = 需要的水柱）。
        "hull_clearance": Params.distance(10.0, minimum=0.0),
        #: 下潜 / 上浮速率（m/s）。分开两个参数是因为两者本来就不对称
        #: （上浮可以吹除压载，下潜只能注水），而合成一个数会让想定作者
        #: 无从表达"这艇上浮快、下潜慢"。
        "dive_rate": Params.speed(2.0, minimum=0.0),
        "surface_rate": Params.speed(4.0, minimum=0.0),
        #: 深度到位容差。判"到达"时用：与空中件的 ``altitude_tolerance``
        #: 是同一件事的两个名字——不放到基类里，是因为地面/水面件的 z 由
        #: 位置决定、天然到位，那个参数在它们身上永远不会被读（"参数在
        #: 但不生效"）。
        "depth_tolerance": Params.distance(5.0, minimum=0.0),
    }

    def initialize(self, mount: Any) -> None:
        super().initialize(mount)
        self._depth_limited = False

    # -- 深度 --------------------------------------------------------------

    def required_water_depth(self) -> float:
        """水柱高度 = ``max(声明值, 定深 + 离底余量)``。见类文档。"""
        ordered = float(self.spec["cruise_depth"]) + float(
            self.spec["hull_clearance"]
        )
        return max(float(self.spec["min_water_depth"]), ordered)

    def ordered_z(self) -> float:
        """定深对应的 z（海平面 0 之下取负）。查询用。"""
        return -float(self.spec["cruise_depth"])

    def depth_limited(self) -> bool:
        """上一帧的 z 是不是**被海底顶上去的**（而不是到了定深）。

        查询用。"我订的 -60 m，为什么它在 -35 m"——答案在这里。被海底顶
        上去是物理上做得到的动作（总比钻进海底强），但它**与命令不符**，
        而"与命令不符却一声不响"正是这个项目最不能接受的那一类事实。
        """
        return self._depth_limited

    def _surface_z(self, ref: Any, x: float, y: float, z: float) -> float:
        """定深——"我想待在哪个深度"。不看地形：定深是命令，不是位置的函数。

        所以它同时是航路点的高度（基类的 :meth:`_waypoint_z` 默认走
        这里），而"怎么走过去、到了海底怎么办"留在 :meth:`_approach_z`。
        """
        return self.ordered_z()

    def _approach_z(
        self, ref: Any, x: float, y: float, z: float, dt_s: float
    ) -> float:
        """按速率朝**目标深度**走，再把结果夹到海底之上。

        目标深度的取法与空中件同构：取下一个航路点的高度，航路点全走完了
        就用最后一个。按格下达目的地时那个高度就是 :meth:`_surface_z`
        （= 定深），按坐标点下达时它是**命令给的那个 z**——于是"改潜深"是
        任务层再下一条命令的事（§5.7.2 对目的地说过同一件事），型号参数只
        负责"平时潜多深"。少了这一条，``surface_rate`` 就是个永不生效的
        参数：定深写在参数里、装配时解析一次，运行期再没有别的入口能改它。

        夹取的顺序不能反：先夹再走，速率那一步会重新把它推到海底以下；
        先走再夹，夹出来的就是本帧的最终值。速率是**舒服**的问题（不想
        瞬移），夹取是**物理**的问题（不能进海底），后者压过前者。

        被夹住时**不是一声不响**：置"走不通"并发那条总线信号。它意味着
        "这条艇按命令的深度过不去这里"，任务层据此改深度或改路线——而
        :meth:`depth_limited` 让人事后查得到"为什么它停在 -20 m 而不是
        -60 m"。解除靠新的命令（:meth:`Mover.move_to_cell` /
        :meth:`~milsim.models.mover.Mover.move_to_point`），与其它"走不通"
        一样：原地等着不是模型该替指挥员做的决定。
        """
        target = self.next_waypoint()
        if target is None and self._waypoints:
            target = self._waypoints[-1]
        if target is None:
            return z                    # 没有目的地：不动深度（与其它件一致）
        z = self._glide_z(
            z,
            target[2],
            float(self.spec["surface_rate"]),
            float(self.spec["dive_rate"]),
            dt_s,
        )
        ground = self._ground_here(ref, x, y)
        if ground is None:
            # 没有地图数据：既夹不了，也不该据此报"走不通"——
            # 没有地形不等于地形不允许。
            self._depth_limited = False
            return z
        limit = ground + float(self.spec["hull_clearance"])
        if z >= limit:
            self._depth_limited = False
            return z
        self._depth_limited = True
        self._set_blocked(self._depth_reason(ground))
        return limit

    def _ground_here(self, ref: Any, x: float, y: float) -> float | None:
        """点位处的地面高程（水下为负）。没有地图服务或没有数据时 ``None``。"""
        if self._nav is None:
            return None
        return self._nav.ground_at(ref, x, y)

    def _depth_reason(self, ground: float) -> str:
        """被海底顶住时的理由。带上三个数：实际水深、要多少、要的那份怎么来的。

        只说"水深不足"是不够的：作者得知道要改 ``cruise_depth`` 还是
        ``hull_clearance``，或者干脆换一条更深的路。
        """
        depth = max(0.0, -ground)
        return (
            f"水深不足，压不住定深：这里水深 {depth:.1f} m，本艇要 "
            f"{self.required_water_depth():.1f} m（定深 "
            f"{float(self.spec['cruise_depth']):.0f} m + 离底余量 "
            f"{float(self.spec['hull_clearance']):.0f} m）"
        )

    def _at_altitude(self, target: tuple[float, float, float] | None) -> bool:
        """水平到位之后，**深度**是不是也到了。

        少了这一半，潜艇会在刚潜下去一点点的时候就宣布"到达"——那与空中件
        在还差 3 km 高度时说到达是同一个错误（§5.7.5）。
        """
        if target is None:
            return True
        pose = self.view.my_pose()
        if pose is None:
            return True
        return abs(pose[2] - target[2]) <= float(self.spec["depth_tolerance"])


__all__ = ["SubsurfaceMover"]
