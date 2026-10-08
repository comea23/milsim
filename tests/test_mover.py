"""机动件测试：把"想去哪"变成"每帧挪多少"（§5.7）。

这一组盯的是**不报错的错**——机动件的失败方式几乎全是"数字对不上"而不是
抛异常，所以每个用例都落在"这个数必须是多少"上：

- 接到命令的第一帧就落位（零时刻瞬移一个周期的距离，里程与用时于是对不上）
- 到达之后速度字段还写着额定速度（一件停住的装备永远显示在 180 m/s）
- 里程按水平投影算，飞机爬升的那几千米被漏掉
- 走不通却报一句与实际原因无关的话（"起点或终点不可通行"是假的）
- 走不通之后照旧往前推，**穿过湖与山**
- 走不通的信号每帧都发（等于没有信号）

用**手写的导航桩**而不是真战区：这里要钉的是"机动件怎么用导航结果"，
不是"导航算得对不对"（那是 ``tests/test_nav.py`` 的事）。真战区里
"这一格是山还是水"由地形生成决定，拿它写断言等于把测试绑在种子上。

真接线（``NavService`` 注入、平台属性、事件节拍）另有一组集成用例，跑
真正的想定 ``scenarios/mover_demo.txt``。
"""

from __future__ import annotations

from math import cos, degrees, hypot, inf, radians, sin, sqrt
from pathlib import Path

import pytest

from milsim.models.mover import (
    AirMover,
    GroundMover,
    Mover,
    SubsurfaceMover,
    WaterMover,
)
from milsim.services.bus import EventBus
from milsim.services.map import Axial, NavService, terrain_codes
from milsim.services.map.path import CostConfig, PathResult
from milsim.services.type_registry import ComponentFactory
from milsim.simulation import Simulation

DEMO = Path(__file__).resolve().parents[1] / "scenarios" / "mover_demo.txt"

PERIOD_S = 5.0
PERIOD_US = int(PERIOD_S * 1_000_000)


# ---------------------------------------------------------------------------
# 桩
# ---------------------------------------------------------------------------

class FakeCell:
    """只要 ``axial``——机动件不碰格引用的其它字段。"""

    __slots__ = ("axial",)

    def __init__(self, axial: Axial) -> None:
        self.axial = axial


class FakeView:
    """``MoverView`` 的四个方法。顺手数一下写了几次，好验证"不写"这件事。"""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x, self.y, self.z = x, y, z
        self.heading = 0.0
        self.speed = 0.0
        self.cell: FakeCell | None = FakeCell(Axial(0, 0))
        self.writes = 0
        self.positions: list[tuple[float, float, float]] = []

    def my_pose(self):
        return (self.x, self.y, self.z, self.heading, self.speed)

    def my_position(self):
        return (self.x, self.y, self.z)

    def my_cell(self):
        return self.cell

    def set_pose(self, x, y, z, heading=0.0, speed=0.0) -> None:
        self.x, self.y, self.z = x, y, z
        self.heading, self.speed = heading, speed
        self.writes += 1
        self.positions.append((x, y, z))


class FakeMount:
    """``MountContext`` 里机动件真正用到的那几项。"""

    def __init__(
        self, view: FakeView, *, nav=None, bus=None, cell_span_m=0.0
    ) -> None:
        self._view = view
        self.nav = nav
        self.bus = bus
        self._span_m = cell_span_m
        self.schedules: list[tuple[int, int]] = []

    def mover_view(self):
        return self._view

    def cell_span_m(self) -> float:
        """相邻格中心距。桩里取自 ``StubNav`` 的 ``cell_span``——两者取不同
        的值，"按格分步推进"测的就不是同一张格子了。

        默认 0.0 表示"这里没有逐格精度"：机动件据此不分步、也不拦（战区外
        和纯点机动就是这个情形）。
        """
        return self._span_m

    def every(self, interval_us, fn, priority=0):
        self.schedules.append((interval_us, priority))
        return None


class StubNav:
    """手写导航桩。

    ``impassable`` 里的格一律"走不了"，``route_cells`` 是寻路的固定答案；
    ``ground`` 是地面高程（``None`` 表示"这里没有地面数据"）。
    """

    def __init__(
        self,
        *,
        route_cells=None,
        impassable=(),
        ground=None,
        factor=1.0,
        terrain="",
        cell_span=100.0,
    ) -> None:
        self._route_cells = route_cells
        self._impassable = set(impassable)
        self._ground = ground
        self._factor = factor
        self._terrain = terrain
        self._span = cell_span
        #: 公开一份：装配层会把同一个数交给组件的 ``MountContext.cell_span_m``
        #: （分步推进按它切），桩里两处必须一致。
        self.cell_span = cell_span
        self.route_calls = 0
        #: ``capable_profile`` 收到的实参。用来钉"通行门槛真的传下去了吗"
        #: ——只断言"实体没动"是分不出"门槛生效"和"门槛根本没传"的。
        self.profile_calls: list[tuple] = []

    def capable_profile(
        self, name, *, blocked_terrain=(), min_water_depth=0.0, max_slope=None
    ):
        """桩返回**真的** ``CostConfig``，只是不带代价表。

        返回假对象或 None 会让"门槛有没有传下去"变成测不出来的事，而
        ``Mover`` 正是靠读它的门槛字段判通行的。
        """
        self.profile_calls.append(
            (name, tuple(blocked_terrain), min_water_depth, max_slope)
        )
        return CostConfig(
            blocked_terrain=terrain_codes(blocked_terrain),
            min_water_depth=min_water_depth,
            max_slope=max_slope,
        )

    def route(self, ref, goal, *, profile=None, forbidden=None):
        self.route_calls += 1
        if self._route_cells is None:
            return None
        return PathResult(list(self._route_cells), 0.0, len(self._route_cells))

    def passable(self, ref, cell, *, profile=None):
        return cell not in self._impassable

    def refusal(self, ref, cell, *, profile=None):
        """桩也说得出原因——生产实现会带出水深一类的数字。

        这里只说地貌：桩没有水深，硬造一个数字会让测试断言一个假的理由。
        """
        if cell not in self._impassable:
            return ""
        return f"是{self._terrain}，这类运载器走不了" if self._terrain else ""

    def terrain_name(self, ref, cell):
        return self._terrain

    def move_factor(self, ref, cell=None, *, profile=None):
        return self._factor

    def ground_at(self, ref, x, y):
        return self._ground

    def axial_at(self, ref, x, y):
        """按 ``cell_span`` 反推格号，与 :meth:`cell_center` 互逆。

        桩必须能区分"这一格"和"那一格"：好几种失败方式（一帧跨两个路点时
        落脚在水里、重规划只查下一格）只有在坐标与格号一一对应时才暴露。
        """
        return Axial(round(x / self._span), round(y / self._span))

    def cell_center(self, ref, cell):
        return (float(cell.q) * self._span, float(cell.r) * self._span)


def make_mover(kind: str = "GROUND_MOVER", *, x=0.0, y=0.0, z=0.0, nav=None, bus=None,
               **params):
    """按类型名造一个装好线的机动件。参数走**同一套**解析，含默认值。"""
    mover = ComponentFactory().build("mover", kind, params)
    view = FakeView(x, y, z)
    # 格距从桩里取：装配层交给组件的 ``MountContext.cell_span_m`` 与寻路
    # 用的是同一个数，两处不一致的话"按格分步推进"测的就不是同一张格子。
    mount = FakeMount(
        view, nav=nav, bus=bus, cell_span_m=getattr(nav, "cell_span", 0.0)
    )
    mover.initialize(mount)
    return mover, view, mount


def advance(mover, *, ticks: int, start_us: int = 0) -> int:
    """按周期推进若干帧，返回最后一帧的时刻。"""
    now = start_us
    for _ in range(ticks):
        mover.update(now)
        now += PERIOD_US
    return now - PERIOD_US


# ---------------------------------------------------------------------------
# 第一帧：只记开工时刻
# ---------------------------------------------------------------------------

def test_first_tick_only_starts_the_clock() -> None:
    """命令刚下达的那一帧**不挪窝**。

    一帧代表区间 ``[now, now+period]`` 上的位移，而这个区间还没过去。照走
    的话实体在零时刻瞬移一个周期的距离（5 s × 20 m/s = 100 m），那点距离
    在地图上看不出来，但里程与用时会对不上：算出来的平均速度比额定高十几个
    百分点，而且不报错。
    """
    mover, view, _ = make_mover(max_speed=20.0, linear_accel=0.0)
    # linear_accel 0 = 关掉速率通道的第二层（§5.7.11），速率于是**阶跃**。
    # 本用例问的是"第一帧有没有挪窝"，加速过程另有测试；开着的话首帧只走
    # 50 m 而不是 100 m，断言会被加速曲线而不是被"瞬移"决定。
    assert mover.move_to_point(1000.0, 0.0, 0.0) is True

    mover.update(0)
    assert view.x == 0.0
    assert mover.travelled_m() == 0.0
    assert mover.elapsed_s() == 0.0
    assert view.writes == 0                 # 一帧都不该写位置

    mover.update(PERIOD_US)
    assert view.x == pytest.approx(100.0)
    assert mover.travelled_m() == pytest.approx(100.0)
    assert mover.elapsed_s() == pytest.approx(PERIOD_S)


def test_average_speed_matches_the_rating() -> None:
    """整段的"里程 ÷ 用时"必须等于额定速度。

    这是首帧瞬移那条的**判据**：瞬移一帧的话这个比值会偏高，而所有更细的
    断言（某个时刻的 x）都还是对的。
    """
    mover, view, _ = make_mover(max_speed=20.0, linear_accel=0.0)
    # linear_accel 0：本用例问的是"里程 ÷ 用时 = 额定速度"，加速过程会把
    # 这个比值压下去（那是真事实，但它是另一个用例的事）。
    mover.move_to_point(2000.0, 0.0, 0.0)
    advance(mover, ticks=5)                  # 第 1 帧只记时刻，之后 4 帧推进

    assert mover.travelled_m() == pytest.approx(400.0)
    assert view.x == pytest.approx(400.0)
    assert mover.travelled_m() / mover.elapsed_s() == pytest.approx(20.0)


def test_elapsed_is_not_derived_from_distance() -> None:
    """在途时"用时"也要是对的，不能等到达才跳到真值。"""
    mover, _, _ = make_mover(max_speed=10.0)
    mover.move_to_point(10_000.0, 0.0, 0.0)
    advance(mover, ticks=3)                  # 第 1 帧只记时刻，之后 2 帧推进
    assert mover.elapsed_s() == pytest.approx(2 * PERIOD_S)
    advance(mover, ticks=2, start_us=3 * PERIOD_US)
    assert mover.elapsed_s() == pytest.approx(4 * PERIOD_S)


# ---------------------------------------------------------------------------
# 里程：按三维距离算
# ---------------------------------------------------------------------------

def test_travel_uses_three_dimensional_distance() -> None:
    """飞机爬升的那一段必须计入里程。

    按水平投影累加会得出"里程 ≈ 直线距离"这种看着没问题、实际少算的数字。
    """
    mover, view, _ = make_mover(
        "AIR_MOVER",
        max_speed=100.0,
        climb_rate=50.0,
        cruise_altitude=1000.0,
        # vertical_accel 0：本用例要的是"一帧爬满爬升率"那个干净的高度剖面
        # （50 m/s × 5 s = 250 m），好把注意力留给三维里程。垂向速率的建立
        # 过程另有测试（§5.7.11）。
        vertical_accel=0.0,
    )
    mover.move_to_point(1000.0, 0.0, 1000.0)
    mover.update(0)
    mover.update(PERIOD_US)

    x, y, z = view.my_position()
    assert view.z == pytest.approx(250.0)        # 50 m/s × 5 s
    assert mover.travelled_m() == pytest.approx(hypot(hypot(x, y), z))
    assert mover.travelled_m() > hypot(x, y)     # 水平投影会漏掉那 250 m
    assert not mover.arrived()


def test_ground_travel_is_horizontal_when_the_ground_is_flat() -> None:
    """平地上三维里程就等于水平里程——别把"按三维算"做成"总是更大"。"""
    mover, _, _ = make_mover(nav=StubNav(ground=0.0), max_speed=10.0)
    mover.move_to_point(150.0, 0.0, 0.0)
    advance(mover, ticks=4)
    assert mover.travelled_m() == pytest.approx(150.0)


# ---------------------------------------------------------------------------
# 到达
# ---------------------------------------------------------------------------

def test_arrival_waits_for_altitude() -> None:
    """水平到位但高度还差几千米时**不算到达**。

    少了高度这一半，飞机会在还差 3 km 高度的时候宣布到达并停住——那不是
    模型简化，那是错的。
    """
    mover, view, _ = make_mover(
        "AIR_MOVER",
        max_speed=100.0,
        climb_rate=50.0,
        cruise_altitude=1000.0,
        altitude_tolerance=100.0,
        # 本测试只看"高度没到就不算到"，转向另有测试（§5.7.11）。不关掉的
        # 话，飞到 (500, 0) 的头一帧还在转舵，x 到不了 500——那是转向在
        # 起作用，不是高度判据坏了。
        turn_rate=0.0,
        # 同理关掉水平与垂向的第二层：本用例要的是"一帧走完/一帧爬满"这种
        # 干净的台阶，好让 x、z 的期望值是整数。加速过程另有测试。
        linear_accel=0.0,
        vertical_accel=0.0,
    )
    mover.move_to_point(500.0, 0.0, 1000.0)
    mover.update(0)

    mover.update(PERIOD_US)
    assert view.x == pytest.approx(500.0)        # 水平已到
    assert view.z == pytest.approx(250.0)
    assert not mover.arrived()

    advance(mover, ticks=2, start_us=2 * PERIOD_US)
    assert view.z == pytest.approx(750.0)
    assert not mover.arrived()

    mover.update(5 * PERIOD_US)
    assert view.z == pytest.approx(1000.0)
    assert mover.arrived()
    assert view.speed == 0.0
    assert mover.speed_mps() == 0.0


def test_speed_is_zeroed_on_the_arrival_tick() -> None:
    """到达的**那一帧**就把速度清零，不留到下一帧。

    留一帧的话，在 ``[到达, 下一帧)`` 这段区间里位置写着"已到"而速度写着
    "在跑"，采样落在这一段的观察者拿到的是一个不存在的事实；推演如果恰好
    在这一帧收尾，那个错值就永远留在结果里了。
    """
    mover, view, _ = make_mover(max_speed=10.0)
    mover.move_to_point(50.0, 0.0, 0.0)
    mover.update(0)
    mover.update(PERIOD_US)

    assert mover.arrived()
    assert mover.speed_mps() == 0.0
    assert view.speed == 0.0            # 存储里也不能留着 10 m/s


def test_arrived_without_a_destination() -> None:
    """没有目的地就算"到了"。

    否则调用方要写两个判断，而漏掉其中一个就会得到"停在原地却被当成在赶路"。
    """
    mover, _, _ = make_mover()
    assert mover.arrived()
    assert mover.destination_cell() is None
    assert mover.destination_point() is None


def test_stop_keeps_the_odometer() -> None:
    """停车**不**清里程：里程是履历，不是状态。"""
    mover, view, _ = make_mover(max_speed=10.0)
    mover.move_to_point(1000.0, 0.0, 0.0)
    advance(mover, ticks=3)
    moved = mover.travelled_m()
    assert moved > 0.0

    mover.stop()
    assert mover.travelled_m() == moved
    assert mover.arrived()
    assert mover.destination_point() is None
    assert mover.waypoints() == ()
    assert mover.elapsed_s() == 0.0          # 本趟计时归零
    assert view.speed == 0.0


def test_new_destination_keeps_the_odometer_and_restarts_the_clock() -> None:
    mover, _, _ = make_mover(max_speed=10.0)
    mover.move_to_point(1000.0, 0.0, 0.0)
    advance(mover, ticks=3)
    moved = mover.travelled_m()

    mover.move_to_point(-1000.0, 0.0, 0.0)   # 掉头，履历保留
    mover.update(3 * PERIOD_US)
    assert mover.travelled_m() == pytest.approx(moved)     # 新一趟第一帧不挪
    assert mover.elapsed_s() == 0.0
    mover.update(4 * PERIOD_US)
    assert mover.travelled_m() > moved


# ---------------------------------------------------------------------------
# 走不通：停住、报准、只发一次
# ---------------------------------------------------------------------------

def test_blocked_without_a_map_service() -> None:
    """没有导航门面时按格机动**走不通**，且给的是准确的理由。"""
    bus = EventBus()
    seen: list[tuple[bool, str]] = []
    bus.mover_blocked.subscribe(lambda eid, blocked, reason: seen.append((blocked, reason)))

    mover, view, _ = make_mover(nav=None, bus=bus)
    assert mover.move_to_cell(Axial(1, 0)) is False
    assert mover.blocked()
    assert "没有地图服务" in mover.blocked_reason()
    assert [b for b, _ in seen] == [True]

    mover.update(0)
    mover.update(PERIOD_US)
    assert view.x == 0.0 and view.y == 0.0      # 停住，不偷偷改走直线


def test_blocked_signal_fires_only_on_transitions() -> None:
    """同一条走不通的路线再下一次**不该再发**——状态与原因都没变。

    每帧都发等于没有信号：订阅方会被迫自己去做去重，而那是发布方的事。
    """
    bus = EventBus()
    seen: list[tuple[bool, str]] = []
    bus.mover_blocked.subscribe(lambda eid, blocked, reason: seen.append((blocked, reason)))

    mover, _, _ = make_mover(nav=None, bus=bus)
    mover.move_to_cell(Axial(1, 0))
    mover.move_to_cell(Axial(1, 0))
    mover.move_to_cell(Axial(1, 0))
    assert [b for b, _ in seen] == [True]

    # 换一条走法且这一次能走通 → 释放信号发一次
    mover.move_to_point(10.0, 0.0, 0.0)
    assert [b for b, _ in seen] == [True, False]
    assert not mover.blocked()


def test_blocked_reason_changing_is_new_information() -> None:
    """**原因**变了要发：那不是重复，是新消息（原先封路的是湖，现在是山）。"""
    bus = EventBus()
    seen: list[str] = []
    bus.mover_blocked.subscribe(lambda eid, blocked, reason: seen.append(reason))

    mover, _, _ = make_mover(
        nav=StubNav(route_cells=None, impassable={Axial(5, 0), Axial(6, 0)}, terrain="水域"),
        bus=bus,
        replan_limit=0,
    )
    mover.move_to_cell(Axial(5, 0))
    mover.move_to_cell(Axial(6, 0))
    assert len(seen) == 2
    assert "(5, 0)" in seen[0] and "(6, 0)" in seen[1]


@pytest.mark.parametrize(
    "expected, impassable",
    [
        ("目的格", {Axial(7, 0)}),      # 目的地走不了
        ("起点格", {Axial(0, 0)}),      # 起点走不了（桩里实体在原点格）
        ("被水域或山地隔开", set()),      # 两头都能走，中间不通
    ],
)
def test_no_route_reason_points_at_the_right_end(expected: str, impassable: set) -> None:
    """说不出原因时至少要说**准**是哪一头的问题。

    原先这里是一句写死的"起点或终点不可通行、或被完全包围"。它曾经是假的：
    真正的原因常常是"走廊那片地形还没生成"。写死的理由只是在掩盖下一个
    同类问题。
    """
    mover, _, _ = make_mover(
        nav=StubNav(route_cells=None, impassable=impassable, terrain="水域"),
        replan_limit=0,
    )
    assert mover.move_to_cell(Axial(7, 0)) is False
    assert expected in mover.blocked_reason()


def test_terrain_name_is_included_in_the_reason() -> None:
    """带上地貌名：'目的格是水域' 比 '目的格不可通行' 省掉一次翻地图。"""
    mover, _, _ = make_mover(
        nav=StubNav(route_cells=None, impassable={Axial(7, 0)}, terrain="水域")
    )
    assert mover.move_to_cell(Axial(7, 0)) is False
    assert "水域" in mover.blocked_reason()


# ---------------------------------------------------------------------------
# 重规划与"不穿山"
# ---------------------------------------------------------------------------

def test_next_waypoint_becoming_impassable_triggers_a_replan() -> None:
    """下一段走不通就就地重规划，而且**本帧不挪**。

    地形是按需生成的：规划时前方可能还没生成，等走到时那块才真实生成，
    届时可能整段路都不可通行。不重规划的后果不是报错，而是实体沿一条
    不存在于地形的路线一直走下去。
    """
    route = [Axial(0, 0), Axial(1, 0), Axial(2, 0)]
    # 桩的格中心距是 100 m，所以"下一个路点"落在 Axial(1, 0) 上——把它
    # 标成不可通行，模拟"规划时那片地形还没生成、走到时才发现是水"。
    nav = StubNav(route_cells=route, impassable={Axial(1, 0)}, ground=0.0)
    mover, view, _ = make_mover(nav=nav, max_speed=20.0, replan_limit=1)
    assert mover.move_to_cell(Axial(2, 0)) is True
    assert nav.route_calls == 1

    mover.update(0)
    mover.update(PERIOD_US)                  # 这一帧撞上不可通行的下一格
    assert nav.route_calls == 2
    assert mover.replans() == 1
    assert view.x == 0.0 and view.y == 0.0   # 停住，不硬穿


def test_replan_budget_exhausted_stops_the_mover() -> None:
    """重规划额度用完就**停住并置走不通**，不要"退回去照原路走"。

    照原路走会穿过那片不可通行的地形，而且不报错——比停在原地糟得多，
    因为上层任务台账会以为进展顺利。
    """
    route = [Axial(0, 0), Axial(1, 0), Axial(2, 0)]
    nav = StubNav(route_cells=route, impassable={Axial(1, 0)})
    mover, view, _ = make_mover(nav=nav, max_speed=20.0, replan_limit=1)

    mover.move_to_cell(Axial(2, 0))
    mover.update(0)
    mover.update(PERIOD_US)                  # 第一次重规划
    mover.update(2 * PERIOD_US)              # 额度用完 → 停住
    assert mover.blocked()
    assert "replan_limit" in mover.blocked_reason()
    assert view.x == 0.0 and view.y == 0.0

    mover.update(3 * PERIOD_US)
    assert view.x == 0.0 and view.y == 0.0   # 之后也不许往前走


def test_mover_never_lands_on_an_impassable_waypoint_within_one_frame() -> None:
    """一帧跨好几个路点时，也不许**落脚在**不能走的格上。

    只检查"下一个路点"是不够的：一帧走完之后路点已经越过了那一格，
    下一帧再也查不到它——实体就站在水里，而且没有任何信号。
    """
    # 格中心距 50 m、一帧位移 100 m：必然跨两格。第二格标为不可通行。
    nav = StubNav(
        route_cells=[Axial(0, 0), Axial(1, 0), Axial(2, 0)],
        impassable={Axial(2, 0)},
        ground=0.0,
        cell_span=50.0,
    )
    mover, view, _ = make_mover(nav=nav, max_speed=20.0, replan_limit=0)
    assert mover.move_to_cell(Axial(2, 0)) is True

    mover.update(0)
    mover.update(PERIOD_US)
    assert view.x == pytest.approx(50.0)      # 停在能走的最后一格，不越过
    assert view.y == pytest.approx(0.0)


def test_air_mover_ignores_passability() -> None:
    """空中件不吃通行代价：天空没有"不可通行"这回事。"""
    mover, view, _ = make_mover(
        "AIR_MOVER",
        nav=StubNav(impassable={Axial(0, 0)}),
        max_speed=100.0,
        cruise_altitude=1000.0,
        turn_rate=0.0,                          # 只看通行性，不看转向
        linear_accel=0.0,                       # 也不看加速（一帧就走 500 m）
    )
    assert mover.PROFILE is None
    mover.move_to_point(500.0, 0.0, 1000.0)
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.x == pytest.approx(500.0)     # 该飞就飞


# ---------------------------------------------------------------------------
# 高度：贴地 / 海平面
# ---------------------------------------------------------------------------

def test_ground_mover_sticks_to_the_ground() -> None:
    """地面件的 z 每帧拉回当地地面高程。

    不这么做的话，一辆车翻过 800 m 的山之后位置里的高度还是出发时的值——
    位置数据自相矛盾，而且不报错。
    """
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=250.0)
    mover, view, _ = make_mover(nav=nav, max_speed=50.0)

    assert mover.move_to_cell(Axial(1, 0)) is True
    assert mover.waypoints()[0][2] == pytest.approx(250.0)   # 航路点按地面高程
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.z == pytest.approx(250.0)


def test_ground_mover_keeps_z_when_there_is_no_ground_data() -> None:
    """查不到地面高程时保持原 z，别把高度抹成 0。"""
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=None)
    mover, view, _ = make_mover(nav=nav, z=120.0, max_speed=50.0)
    mover.move_to_cell(Axial(1, 0))
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.z == pytest.approx(120.0)


def test_water_mover_sits_at_sea_level() -> None:
    """水面件恒取海平面 0 m（§9 第 13 条：海平面定死 0）。"""
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=250.0)
    mover, view, _ = make_mover("WATER_MOVER", nav=nav, z=999.0, max_speed=50.0)
    mover.move_to_cell(Axial(1, 0))
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.z == 0.0


def test_water_mover_uses_the_water_profile() -> None:
    """参考实现只差画像与高度，寻路代码共用一套。

    **四个件都直接继承基类、彼此不互相继承**（§5.7.12）：共用的是"段推进 /
    转向 / 落脚点检查"，那些本来就在基类里；横着的继承只会把两个无关的组件
    绑在一起（改地面件要牵动水面件）。
    """
    assert GroundMover.PROFILE == "ground"
    assert WaterMover.PROFILE == "water"
    assert AirMover.PROFILE is None
    assert SubsurfaceMover.PROFILE == "water"
    for cls in (GroundMover, WaterMover, SubsurfaceMover, AirMover):
        assert cls.__mro__[1] is Mover, f"{cls.__name__} 不该继承别的参考实现"


# ---------------------------------------------------------------------------
# 水下：定深 / 离底余量 / 门槛是算出来的
# ---------------------------------------------------------------------------

def test_subsurface_is_a_peer_of_the_water_mover() -> None:
    """水下件与水面件**并列**，不是它的子类：同一张水面画像，相反的高度。

    继承 ``WaterMover`` 会把"潜艇是水面舰"写成一句代码里的断言，而那是错的
    ——一个待在水面、一个待在水下，z 的处理正好相反。
    """
    assert SubsurfaceMover.PROFILE == "water"
    assert issubclass(SubsurfaceMover, Mover)
    assert not issubclass(SubsurfaceMover, WaterMover)

    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=-500.0)
    # turn_rate 0：桩里一格只有 100 m，比这艘艇的转弯半径还短，按速率转就得
    # 绕半天——本测试问的是"并列关系与定深"，转向在 §5.7.11 单独测。
    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER", nav=nav, cruise_depth=60.0, turn_rate=0.0
    )

    assert mover.move_to_cell(Axial(1, 0)) is True
    assert mover.waypoints()[0][2] == pytest.approx(-60.0)   # 航路点在定深
    advance(mover, ticks=30)
    assert view.z == pytest.approx(-60.0)
    assert mover.arrived()
    assert mover.depth_limited() is False        # 500 m 水深，碰不到海底


def test_subsurface_required_water_depth_is_ordered_depth_plus_clearance() -> None:
    """门槛是**算出来的**：定深 + 离底余量。声明值只能更严，不能更松。

    定深是任务/型号给的，吃水是船体给的——两件事，但"这一格能不能去"只有
    一个答案。取严的那个，于是"忘了写吃水"这类漏配不会让潜艇钻进海底：
    它只是没有额外的下限，而不是没有下限。
    """
    mover, _, _ = make_mover(
        "SUBSURFACE_MOVER", nav=StubNav(), cruise_depth=60.0, hull_clearance=15.0
    )
    assert mover.required_water_depth() == pytest.approx(75.0)
    assert mover.travel_config().min_water_depth == pytest.approx(75.0)

    strict, _, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=StubNav(),
        cruise_depth=60.0,
        hull_clearance=15.0,
        min_water_depth=200.0,      # 更严：取它
    )
    assert strict.required_water_depth() == pytest.approx(200.0)

    loose, _, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=StubNav(),
        cruise_depth=60.0,
        hull_clearance=15.0,
        min_water_depth=8.0,        # 更松（比如从水面舰那里抄来的吃水）→ 压不过
    )
    assert loose.required_water_depth() == pytest.approx(75.0)
    assert loose.travel_config().min_water_depth == pytest.approx(75.0)


def test_subsurface_dives_at_the_rate_instead_of_jumping() -> None:
    """z 不是瞬移过去的：一帧只走 ``dive_rate`` × 周期。

    一步到的话，"从海面潜到 -60 m"发生在**一帧之内**——里程与用时随即对不上，
    而且不报错。
    """
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=-500.0)
    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=nav,
        cruise_depth=60.0,
        dive_rate=2.0,
        surface_rate=10.0,
        max_speed=0.0,              # 只留下潜，好数得清
        # vertical_accel 0：本用例数的是"一帧潜多少"，要的是满速率的台阶。
        # 下潜速率的建立过程另有测试（§5.7.11）。
        vertical_accel=0.0,
    )
    assert mover.move_to_cell(Axial(1, 0)) is True

    mover.update(0)                          # 第一帧只记开工时刻
    mover.update(PERIOD_US)
    assert view.z == pytest.approx(-10.0)    # 2 m/s × 5 s
    mover.update(PERIOD_US * 2)
    assert view.z == pytest.approx(-20.0)

    advance(mover, ticks=10, start_us=PERIOD_US * 3)
    assert view.z == pytest.approx(-60.0)    # 到定深就停住，不再往下


def test_subsurface_surfaces_on_a_command_and_faster_than_it_dives() -> None:
    """按坐标点给的 z 就是目标深度，上浮走 ``surface_rate``。

    两个速率分开不是装饰：上浮可以吹除压载，下潜只能注水，拿一个数去表达
    两件事必然有一头是错的。
    """
    nav = StubNav(route_cells=[Axial(0, 0)], ground=-500.0)
    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=nav,
        z=-60.0,
        cruise_depth=60.0,
        dive_rate=2.0,
        surface_rate=10.0,
        vertical_accel=0.0,          # 只看"上浮比下潜快"，不看速率怎么建立
    )
    assert mover.move_to_point(0.0, 0.0, 0.0) is True
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.z == pytest.approx(-10.0)    # 10 m/s × 5 s（若用下潜速率会是 -50）
    advance(mover, ticks=3, start_us=PERIOD_US)
    assert view.z == pytest.approx(0.0)
    assert mover.arrived()                   # 水平本来就在点上，深度也到了


def test_subsurface_waits_for_the_commanded_depth_before_arriving() -> None:
    """水平到位了但深度没到，不算到达——与空中件的高度判据同构。

    少了这一半，潜艇会在刚潜下去一点点的时候就宣布"到达"。
    容差收紧到 0.5 m，好把"差 55 m 也算到了"这种情况挡在外面。
    """
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=-500.0)
    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=nav,
        cruise_depth=60.0,
        dive_rate=1.0,
        depth_tolerance=0.5,
        max_speed=200.0,
        turn_rate=0.0,                          # 只看深度判据，不约束转向
        # 角速度上界不存在 ≠ 航向瞬转：**角速度的建立**是另一层，归
        # angular_accel 管（§5.7.11）。本用例要的是"一帧就对准"，所以两层
        # 都关。只关上层的话，艇会以 0.25 °/s 慢慢转（20 s 才转满舵），
        # 头几帧判"目标在侧后方"而不前进——那不是深度判据坏了。
        angular_accel=0.0,
        # 同理关掉水平与垂向的第二层：本用例要的是"一帧走完 100 m、只潜
        # 5 m"这两个干净的数，拿它们去分"水平到了没有"与"深度到了没有"。
        linear_accel=0.0,
        vertical_accel=0.0,
    )
    assert mover.move_to_cell(Axial(1, 0)) is True

    mover.update(0)
    mover.update(PERIOD_US)
    assert view.x == pytest.approx(100.0)     # 一帧就走完了 100 m
    assert view.z == pytest.approx(-5.0)     # 但只潜了 5 m
    assert not mover.arrived()

    advance(mover, ticks=12, start_us=PERIOD_US)
    assert view.z == pytest.approx(-60.0)
    assert mover.arrived()


def test_subsurface_stops_within_the_depth_tolerance() -> None:
    """到位判据带容差，所以它可能停在离定深几步的地方——那是**声明过的**容差。

    把它写下来是因为这类"和命令差一点"的事实最容易被当成 bug：容差是参数，
    差在容差之内就是到位，差在之外就是没到。
    """
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], ground=-500.0)
    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=nav,
        cruise_depth=60.0,
        dive_rate=1.0,
        depth_tolerance=5.0,
        max_speed=200.0,
        turn_rate=0.0,                          # 只看容差，不约束转向
        # linear_accel 0：让水平一帧走完、停在 100 m 处，从而"什么时候
        # 到达"完全由深度决定。开着的话水平要爬几十帧（这时深度已经潜到
        # 底），本用例就不再是在测容差了。
        linear_accel=0.0,
    )
    assert mover.move_to_cell(Axial(1, 0)) is True
    mover.update(0)
    advance(mover, ticks=30)

    assert mover.arrived()
    assert view.z <= -60.0 + float(mover.spec["depth_tolerance"])
    assert view.z > -60.0                     # 不会潜过头


def test_subsurface_is_clamped_above_the_seabed_and_says_so() -> None:
    """海底把 z 顶上去，而且这件事**查得到、也报得出来**。

    门槛只管规划出来的那串格；按坐标点下达目的地不经过寻路，中途跨过的格
    没人查过。少了这一夹，潜艇会无声地走在海底以下——位置数据自相矛盾，
    而且没有任何信号。
    """
    nav = StubNav(route_cells=[Axial(0, 0)], ground=-30.0)
    bus = EventBus()
    seen: list[tuple[bool, str]] = []
    bus.mover_blocked.subscribe(lambda eid, blocked, why: seen.append((blocked, why)))

    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER",
        nav=nav,
        bus=bus,
        cruise_depth=60.0,
        hull_clearance=10.0,
        dive_rate=20.0,             # 快一点，几帧就到底
        max_speed=0.0,
    )
    assert mover.move_to_point(0.0, 0.0, -60.0) is True

    advance(mover, ticks=6)
    # 海底 -30 m、余量 10 m → z 不得低于 -20 m（而不是命令的 -60 m）
    assert view.z == pytest.approx(-20.0)
    assert mover.depth_limited() is True
    assert mover.blocked() is True
    assert "水深不足" in mover.blocked_reason()
    assert "30.0 m" in mover.blocked_reason()          # 实际水深
    assert "70.0 m" in mover.blocked_reason()          # 定深 60 + 余量 10
    assert len(seen) == 1 and seen[0][0] is True       # 只跳变那一下发信号


def test_subsurface_keeps_the_depth_when_there_is_no_map_data() -> None:
    """查不到海底就**不夹**，也不据此报走不通：没有地形不等于地形不允许。"""
    nav = StubNav(route_cells=[Axial(0, 0)], ground=None)
    mover, view, _ = make_mover(
        "SUBSURFACE_MOVER", nav=nav, cruise_depth=60.0, dive_rate=20.0, max_speed=0.0
    )
    assert mover.move_to_point(0.0, 0.0, -60.0) is True
    advance(mover, ticks=6)
    assert view.z == pytest.approx(-60.0)
    assert mover.depth_limited() is False
    assert mover.blocked() is False


def test_speed_follows_the_terrain_factor() -> None:
    """速度 = 最高速度 × 当地通行系数，且写进存储的就是这个值。"""
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], factor=0.25, ground=0.0)
    mover, view, _ = make_mover(nav=nav, max_speed=40.0)
    mover.move_to_cell(Axial(1, 0))
    mover.update(0)
    mover.update(PERIOD_US)
    assert mover.speed_mps() == pytest.approx(10.0)
    assert view.speed == pytest.approx(10.0)
    assert view.x == pytest.approx(50.0)          # 10 m/s × 5 s


def test_speed_is_zero_for_an_impassable_current_cell() -> None:
    """站在不可通行的格上时速度是 0，不是"按地形还能跑 12 m/s"。"""
    nav = StubNav(route_cells=[Axial(0, 0), Axial(1, 0)], factor=0.0, ground=0.0)
    mover, _, _ = make_mover(nav=nav, max_speed=40.0)
    mover.move_to_cell(Axial(1, 0))
    mover.update(0)
    mover.update(PERIOD_US)
    assert mover.speed_mps() == 0.0


# ---------------------------------------------------------------------------
# 参数与自省
# ---------------------------------------------------------------------------

def test_defaults_are_resolved_into_the_spec() -> None:
    """组件拿到的是**含默认值**的完整参数，不该自己查表、判空。"""
    mover, _, mount = make_mover()
    assert float(mover.spec["max_speed"]) == pytest.approx(20.0)
    assert int(mover.spec["period"]) == int(PERIOD_US)
    assert mover.slot == "mover"
    assert mount.schedules[0][0] == int(PERIOD_US)        # 节拍真的注册了


def test_period_is_a_parameter_not_a_constant() -> None:
    """节拍是模型的一部分：后勤 60 s 算一次与雷达 0.5 s 扫一次都得能写。"""
    mover, view, mount = make_mover(
        period=1_000_000,
        max_speed=10.0,
        turn_rate=0.0,
        angular_accel=0.0,
        linear_accel=0.0,
    )
    # turn_rate 0 **且** angular_accel 0 = 整个航向通道两层都关（瞬转）；
    # linear_accel 0 关掉速率那一层。三个带时间常数的通道全关掉，x 才只由
    # 周期 × 速率 决定——本用例问的是"周期是参数"，不是"多久加起速"。
    assert mount.schedules[0][0] == 1_000_000
    mover.move_to_point(1000.0, 0.0, 0.0)
    mover.update(0)
    mover.update(1_000_000)
    assert view.x == pytest.approx(10.0)      # 1 s × 10 m/s


def test_describe_reports_the_state() -> None:
    mover, _, _ = make_mover()
    assert "已到" in mover.describe()
    mover.move_to_cell(Axial(1, 0))
    assert "走不通" in mover.describe()       # 无导航门面


def test_unknown_parameter_is_rejected() -> None:
    """拼错的参数名当场报错——参数表是契约。"""
    from milsim.services.params import UnknownParameterError

    with pytest.raises(UnknownParameterError):
        make_mover(max_speeed=10.0)


# ---------------------------------------------------------------------------
# 集成：真接线、真想定
# ---------------------------------------------------------------------------

def build_demo() -> Simulation:
    sim = Simulation.from_scenario_file(DEMO)
    sim.build()
    sim.initialize()
    return sim


def pick_goal(sim: Simulation, ref, profile: object = "ground", span: int = 16) -> Axial:
    """找一个**该底盘**走得到的远目的地。

    不写死格号：可通行与否由地形生成决定，写死等于把集成测试绑在种子上
    ——换一次种子就会得到一堆与被测行为无关的失败。

    ``profile`` 传的是底盘自己的代价配置（``mover.travel_config()``）。
    为什么不能只看 ``passable``：它只判**目标格自己**，而轮式底盘进不了
    山地——目标格是一片平原，中间横着一条山脉，照样走不到。用
    ``passable`` 挑出来的目的地在轮式上会直接 ``move_to_cell`` 失败，
    失败信息还写着"被水域或山地隔开"，看起来像地形的问题。

    开头的区域预热见 :meth:`NavService.ensure_region`：A* 绕行会伸出
    ``ensure_corridor`` 的沿线带，带外读到默认地貌（编号 0 = 水域），
    地面画像下是一片假墙——"绕远能到"会被静默判成"不可达"。
    """
    sim.nav.ensure_region(ref, ref.axial, span + 32)
    for dq in range(span, span + 24):
        for dr in (0, 1, -1, 2, -2):
            cell = Axial(ref.axial.q + dq, ref.axial.r + dr)
            sim.nav.ensure_loaded(ref, cell)
            if sim.nav.route(ref, cell, profile=profile) is not None:
                return cell
    raise AssertionError("找不到可通行的目标——换一个地形种子")


def test_scenario_injects_nav_and_platform_params() -> None:
    """装配层要把导航门面与平台属性真的接进来。

    这两样都是"没接上也照样跑"的东西：没接导航，地面件只是永远按最高速度
    直线走；没接平台属性，``side`` 会是空串。都不会抛异常。
    """
    sim = build_demo()
    assert isinstance(sim.nav, NavService)

    car = sim.registry.by_name("CAR_1").entity_id
    jet = sim.registry.by_name("JET_1").entity_id
    assert sim.platform_params[car]["side"] == "red"
    assert sim.platform_params[jet]["side"] == "blue"

    mount = sim.mount_for(car)
    assert mount.nav is sim.nav
    assert mount.platform_param("side") == "red"
    assert mount.platform_param("没有这个键") is None
    assert mount.platform_param("没有这个键", "兜底") == "兜底"


def test_demo_movers_actually_move_and_stop() -> None:
    """跑一段真想定：三件装备都要到达、停下，且里程与用时自洽。"""
    sim = build_demo()
    names = ("CAR_1", "CAR_2", "JET_1")
    movers = {
        name: sim.entities[sim.registry.by_name(name).entity_id].require("mover")
        for name in names
    }

    ref = sim.store.cell_of(sim.registry.by_name("CAR_1").entity_id)
    # 目的地按轮式的门槛挑：想定里 MOVER_WHEELED 封了山地，履带不封。
    # 履带门槛更宽，所以轮式能到的目的地履带一定也能到——两条车共用一个
    # 目的地仍然成立。
    goal = pick_goal(sim, ref, movers["CAR_1"].travel_config())
    assert movers["CAR_1"].move_to_cell(goal) is True, movers["CAR_1"].blocked_reason()
    assert movers["CAR_2"].move_to_cell(goal) is True, movers["CAR_2"].blocked_reason()

    gx, gy = sim.nav.cell_center(ref, goal)
    assert movers["JET_1"].move_to_point(
        gx, gy, float(movers["JET_1"].spec["cruise_altitude"])
    ) is True

    for _ in range(600):
        sim.run_for(5_000_000)
        if all(m.arrived() for m in movers.values()):
            break

    for name in names:
        mover = movers[name]
        assert mover.arrived(), f"{name} 没到达：{mover.blocked_reason() or '在途'}"
        assert mover.travelled_m() > 0.0
        pose = sim.store.pose_of(sim.registry.by_name(name).entity_id)
        assert pose is not None and pose[4] == 0.0        # 停下就是停下
        # 里程 ÷ 用时 不该超过额定速度：超过了就说明发生了瞬移
        rated = float(mover.spec["max_speed"])
        assert mover.travelled_m() / mover.elapsed_s() <= rated * 1.05


def test_ground_mover_height_tracks_the_terrain_in_the_real_zone() -> None:
    """真战区里地面件的 z 必须逐步等于它所在格的地面高程。"""
    sim = build_demo()
    eid = sim.registry.by_name("CAR_1").entity_id
    mover = sim.entities[eid].require("mover")

    ref = sim.store.cell_of(eid)
    goal = pick_goal(sim, ref, mover.travel_config())
    assert mover.move_to_cell(goal) is True, mover.blocked_reason()

    mismatches: list[tuple[float, float]] = []
    for _ in range(600):
        sim.run_for(5_000_000)
        pose = sim.store.pose_of(eid)
        here = sim.store.cell_of(eid)
        grid = sim.maps.grid(here.zone_id, here.layer)
        want = float(grid.elevation(here.axial))
        if abs(pose[2] - want) > 1.0:
            mismatches.append((pose[2], want))
        if mover.arrived():
            break

    assert not mismatches, (
        f"有 {len(mismatches)} 步的 z 与所在格地面高程不符：{mismatches[:3]}"
        "（地面件没在贴地——位置数据自相矛盾，而且不报错）"
    )


def test_air_mover_climbs_to_cruise_altitude() -> None:
    sim = build_demo()
    eid = sim.registry.by_name("JET_1").entity_id
    mover = sim.entities[eid].require("mover")
    cruise = float(mover.spec["cruise_altitude"])

    ref = sim.store.cell_of(eid)
    goal = pick_goal(sim, ref)
    gx, gy = sim.nav.cell_center(ref, goal)
    assert mover.move_to_point(gx, gy, cruise) is True

    for _ in range(600):
        sim.run_for(5_000_000)
        if mover.arrived():
            break

    assert mover.arrived()
    pose = sim.store.pose_of(eid)
    assert pose[2] == pytest.approx(cruise, abs=float(mover.spec["altitude_tolerance"]))


# ---------------------------------------------------------------------------
# 三自由度：三个通道，每个通道两层（速率 + 加速度）（§5.7.11）
# ---------------------------------------------------------------------------

def test_every_mover_has_a_turn_rate() -> None:
    """转向速率是**每一件都有**的，而且默认就生效。

    这是"三自由度"的形式判据：三个受控通道里，只丢掉转向，模型就退化成
    "一个标量速度 + 一个免费的朝向"——任何件都能零半径掉头，转弯半径恒为
    0，那不是精度差一点，那是**机动能力根本不存在**。
    """
    for cls in (GroundMover, WaterMover, SubsurfaceMover, AirMover):
        assert "turn_rate" in cls.PARAMS, f"{cls.__name__} 少了转向速率"
        assert float(cls.PARAMS["turn_rate"].default) > 0.0, (
            f"{cls.__name__} 的转向速率默认值是 0——那等于默认不约束转向"
        )


def test_heading_turns_at_the_rate_instead_of_snapping() -> None:
    """航向一帧只转 ``turn_rate`` × 周期，不是直接赋值。

    目标在正东、初始航向朝北（差 90°）：6 °/s 的件要走三帧才对准。
    直接赋值的话第一帧就是 90°——**方向是免费的**，而"机动"这个词也就
    没有内容了。
    """
    mover, view, _ = make_mover(max_speed=1.0, turn_rate=6.0)   # 一帧 30°
    mover.move_to_point(10_000.0, 0.0, 0.0)     # 正东 = 90°，初始航向 0

    mover.update(0)
    mover.update(PERIOD_US)
    assert view.heading == pytest.approx(30.0)  # 不是 90
    mover.update(2 * PERIOD_US)
    assert view.heading == pytest.approx(60.0)
    mover.update(3 * PERIOD_US)
    assert view.heading == pytest.approx(90.0, abs=0.5)   # 对准了
    # 对准之后仍然有零点几度的微调：实体在往前挪，而目标方向的方位角随
    # 位置微微变化，它在追。那不是"还在转"，所以给一个宽容差。
    mover.update(4 * PERIOD_US)
    assert view.heading == pytest.approx(90.0, abs=0.5)


def test_turn_rate_zero_means_no_constraint() -> None:
    """``turn_rate 0`` = **不加约束**（航向瞬时对准），不是"不能转"。

    与 ``min_water_depth 0`` / ``blocked_terrain`` 留空同一条约定：门槛参数
    的 0 / 空值表示"不限制"。要是把 0 读成"转不了"，想定里少写一个参数就
    会让实体**永远原地不动**——而且不报错。

    注意它管的是**速率**那一层：``turn_rate 0`` 说的是角速度上界不存在，
    角速度**怎么建立**仍然归 ``angular_accel`` 管（§5.7.11）。本用例看不出
    这一层的差别是有原因的——底盘默认 12 °/s² × 5 s = 60 °/s，远大于"一帧
    对准 90° 所需的 18 °/s"，所以角速度一帧就到位。角加速度那一层由
    :func:`test_a_small_angular_accel_delays_the_turn` 单独钉住。
    """
    mover, view, _ = make_mover(max_speed=20.0, turn_rate=0.0, linear_accel=0.0)
    mover.move_to_point(1000.0, 0.0, 0.0)
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.heading == pytest.approx(90.0)      # 一帧就对上了
    assert view.x == pytest.approx(100.0)           # 而且真的往前走了


def test_the_mover_turns_around_before_it_moves() -> None:
    """目标在侧后方（误差 > 90°）时**先掉头、不前进**。

    这时候沿当前航向推进是在**远离目标**，走的那一段全是白走的。代价是
    这一帧里程为 0、用时照走——于是平均速度低于峰值速度，那是转向花掉的
    时间，不是模型算错了。
    """
    mover, view, _ = make_mover(max_speed=20.0, turn_rate=9.0)   # 一帧 45°
    mover.move_to_point(0.0, 1000.0, 0.0)       # 正南：y 轴向南，所以是 180°

    mover.update(0)
    mover.update(PERIOD_US)
    assert view.heading == pytest.approx(45.0)
    assert mover.travelled_m() == 0.0           # 还差 135°，这一步没走

    mover.update(2 * PERIOD_US)
    assert view.heading == pytest.approx(90.0)
    assert mover.travelled_m() == 0.0           # 还差 90°，仍然不走

    mover.update(3 * PERIOD_US)
    assert view.heading == pytest.approx(135.0)
    assert mover.travelled_m() == 0.0           # 还差 45°，仍然不走

    mover.update(4 * PERIOD_US)
    assert view.heading == pytest.approx(180.0)
    assert mover.travelled_m() > 0.0            # 对准了才走


def test_a_slower_turn_rate_makes_a_wider_arc() -> None:
    """同样的起点与目的地，转得慢的**先向一侧偏出去**——转弯半径 = 速率 ÷ 角速度。

    判据取的是**横向偏离**而不是里程：圆弧对总里程的影响本来就小（实测
    2253 m 对 2236 m，差不到 1%），而"转弯半径"这件事恰恰只体现在偏离上。

    角度取 25°：小到不会触发"先掉头"（那是 30° 以上的事），于是两者都在
    "边走边转"，差别只在转得多快。
    """
    angle = radians(25.0)

    def lateral_offset(turn_rate: float) -> float:
        """五帧之后，位置离线起点→目的地那条直线有多远（米）。"""
        mover, view, _ = make_mover(max_speed=20.0, turn_rate=turn_rate)
        mover.move_to_point(2000.0 * sin(angle), -2000.0 * cos(angle), 0.0)
        mover.update(0)
        for tick in range(1, 6):
            mover.update(tick * PERIOD_US)
        x, y, _ = view.my_position()
        return abs(x * cos(angle) + y * sin(angle))

    assert lateral_offset(60.0) == pytest.approx(0.0, abs=1.0)   # 一帧就对准 → 走直线
    assert lateral_offset(0.2) > 50.0, "转得慢却没偏出去，转向没进运动学"


def test_a_target_it_cannot_turn_into_slows_it_down() -> None:
    """转弯半径大于到目标的距离时要**减速**，而不是绕着目标转圈。

    少了这条限速，实体会每一帧都"在动"、到目标的距离却不减——永远到不了，
    而且不报错、不置 blocked，从外面看就是"它一直在飞"。有了限速，接近
    目标时速度降下来、半径跟着缩，于是进得了弯。
    """
    mover, _, _ = make_mover(max_speed=100.0, turn_rate=1.0)   # 半径 5.7 km
    mover.move_to_point(300.0, 0.0, 0.0)        # 目标只有 300 m，远小于半径

    for tick in range(150):
        mover.update(tick * PERIOD_US)
        if mover.arrived():
            break

    assert mover.arrived(), "转不进去就绕圈——限速没生效"
    assert mover.travelled_m() < 300.0 * 4.0, "绕的圈太大了，限速形同虚设"


def test_march_stops_at_the_edge_of_an_impassable_cell() -> None:
    """分步推进：**斜插过去的格也要查**，不是只查路点。

    航向有了角速度上界之后，轨迹不再是路点之间的折线，而是带圆角的弧——
    弧会斜切过格角，而那个角上可能正是水域或山脊。路点本身没有任何问题，
    所以"只查下一个路点"查不到它；实体就站在里面了，而且没有任何信号。
    """
    nav = StubNav(impassable={Axial(1, 0)}, cell_span=100.0)
    mover, _, _ = make_mover(nav=nav, max_speed=20.0)

    x, y, used, wall = mover._march(None, 0.0, 0.0, 90.0, 300.0)
    assert wall == Axial(1, 0)                  # 挡住了，而且说得出是哪一格
    assert 0.0 < used < 300.0                   # 停在进入那一格之前
    assert x == pytest.approx(used)             # 沿航向走，没有横向漂移
    assert y == pytest.approx(0.0)


def test_the_real_zone_run_never_stands_on_an_impassable_cell() -> None:
    """真战区跑一段：**每一帧**都必须站在可通行格上。

    这是"分步校验真的在生效"的看门测试。它失效时的症状极隐蔽：实体在
    地图上看着在正常赶路，而它的格归属落在水域或山地里——没有任何信号，
    直到有人拿它的位置去查地形才会发现。
    """
    sim = build_demo()
    eid = sim.registry.by_name("CAR_1").entity_id
    mover = sim.entities[eid].require("mover")
    profile = mover.travel_config()

    ref = sim.store.cell_of(eid)
    goal = pick_goal(sim, ref, profile)
    assert mover.move_to_cell(goal) is True, mover.blocked_reason()

    offenders: list[tuple] = []
    for _ in range(400):
        sim.run_for(5_000_000)
        here = sim.store.cell_of(eid)
        if not sim.nav.passable(here, here.axial, profile=profile):
            offenders.append((here.axial, mover.blocked_reason()))
        if mover.arrived() or mover.blocked():
            break

    assert not offenders, (
        f"有 {len(offenders)} 帧站在不可通行的格上：{offenders[:3]}"
        "（弧线切进了障碍，而落脚点没人查）"
    )


def test_turn_radius_is_speed_over_turn_rate() -> None:
    """转弯半径 = 速率 ÷ 角速度。查询用——"它为什么绕不进这条巷子"。

    ``linear_accel 0`` 是为了让 `speed_mps()` 就是 30（否则它在加速中，
    半径会随速率一起变）；``radial_accel 0`` 是为了让角速度上界**只**由
    ``turn_rate`` 给——底盘默认的 5 m/s² 会算出 5/30 rad/s = 9.5 °/s，
    那时半径就不再是 30 ÷ ω(90 °/s) 了。
    """
    mover, _, _ = make_mover(
        max_speed=30.0,
        turn_rate=90.0,                 # 一帧 450°
        radial_accel=0.0,
        linear_accel=0.0,
    )
    mover.move_to_point(10_000.0, 0.0, 0.0)
    mover.update(0)
    mover.update(PERIOD_US)
    assert mover.speed_mps() == pytest.approx(30.0)
    assert mover.turn_radius_m() == pytest.approx(30.0 / radians(90.0))

    free, _, _ = make_mover(
        max_speed=30.0, turn_rate=0.0, radial_accel=0.0, linear_accel=0.0
    )
    free.move_to_point(10_000.0, 0.0, 0.0)
    free.update(0)
    free.update(PERIOD_US)
    assert free.turn_radius_m() == 0.0          # 不限制转向就没有半径这回事


# ---------------------------------------------------------------------------
# 三个通道的第二层：加速度（§5.7.11）
# ---------------------------------------------------------------------------

def test_every_mover_has_both_layers_on_every_channel() -> None:
    """三个通道**每个都有两层**，而且第二层默认就生效。

    只有速率上限是不够的：``max_speed`` 说得出"最快多快"，说不出"多久能到
    那么快"。少了第二层，速率、垂向速率、角速度三处全是**阶跃**——第一帧
    0 → 25 m/s 是一个无穷大的加速度。参数表里有这一项、默认值却是 0
    （= 不限制）的话，那句话就只是写在表里而已。
    """
    for cls in (GroundMover, WaterMover, SubsurfaceMover, AirMover):
        for name in ("linear_accel", "angular_accel"):
            assert name in cls.PARAMS, f"{cls.__name__} 少了 {name}"
            assert float(cls.PARAMS[name].default) > 0.0, (
                f"{cls.__name__} 的 {name} 默认是 0——那等于默认没有加速度"
            )

    # 垂向那一层只对"按速率升降"的件接通：地面件与水面的 z 是**位形约束**
    # （贴地 / 海平面），没有速率这一层，所以它们的默认值是 0。
    for cls in (SubsurfaceMover, AirMover):
        assert float(cls.PARAMS["vertical_accel"].default) > 0.0, (
            f"{cls.__name__} 的垂向由速率控制，却没有垂向加速度"
        )
    for cls in (GroundMover, WaterMover):
        assert float(cls.PARAMS["vertical_accel"].default) == 0.0


def test_speed_builds_up_instead_of_jumping() -> None:
    """速率是**建立**起来的，不是一帧跳到额定。

    ``linear_accel 2 m/s²``、周期 5 s ⇒ 每帧最多长 10 m/s。所以第一帧只到
    10 m/s（走 50 m），第二帧才到 20 m/s（再走 100 m）——而"阶跃"那一版
    两帧都是 100 m。里程、平均速度、到达时刻全都会因此不同。
    """
    mover, view, _ = make_mover(max_speed=20.0, linear_accel=2.0)
    mover.move_to_point(100_000.0, 0.0, 0.0)

    mover.update(0)
    mover.update(PERIOD_US)
    assert mover.speed_mps() == pytest.approx(10.0)
    assert mover.commanded_speed_mps() == pytest.approx(10.0)
    assert view.x == pytest.approx(50.0)

    mover.update(2 * PERIOD_US)
    assert mover.speed_mps() == pytest.approx(20.0)     # 到额定就不再涨
    assert view.x == pytest.approx(150.0)               # 50 + 100


def test_linear_accel_zero_is_a_step_not_a_stop() -> None:
    """``linear_accel 0`` = **不限制**（速率阶跃），不是"不能动"。

    与 ``turn_rate 0`` / ``min_water_depth 0`` 同一条约定：门槛参数的 0 表示
    "不限制"。读成"不能动"的话，想定里漏写一个 ``linear_accel`` 会让实体
    **永远停在 0 速**——而且不报错，从外面看就是"它就是不动"。
    """
    mover, view, _ = make_mover(max_speed=20.0, linear_accel=0.0)
    mover.move_to_point(1000.0, 0.0, 0.0)
    mover.update(0)
    mover.update(PERIOD_US)
    assert view.x == pytest.approx(100.0)               # 一帧到额定


def test_a_small_angular_accel_delays_the_turn() -> None:
    """角速度也是**建立**起来的：``angular_accel 1 °/s²`` ⇒ 每帧最多长
    5 °/s，所以第一帧只转到 25° 而不是 30°。

    这一层是"航向瞬转"那类缺项的**低一阶版本**：有 ``turn_rate`` 只能说出
    "最多转多快"，说不出"多久能转到那么快"。三处都一样，只是它最不显眼。
    """
    mover, view, _ = make_mover(
        max_speed=1.0, turn_rate=6.0, angular_accel=1.0
    )
    mover.move_to_point(10_000.0, 0.0, 0.0)     # 正东 = 90°，初始航向 0

    mover.update(0)
    for tick, heading in enumerate((25.0, 55.0, 85.0), start=1):
        mover.update(tick * PERIOD_US)
        assert view.heading == pytest.approx(heading, abs=0.01)

    # 同一个件把角加速度关掉，第一帧就是 30°——两层的差别落在数字上
    step, step_view, _ = make_mover(
        max_speed=1.0, turn_rate=6.0, angular_accel=0.0
    )
    step.move_to_point(10_000.0, 0.0, 0.0)
    step.update(0)
    step.update(PERIOD_US)
    assert step_view.heading == pytest.approx(30.0)


def test_a_small_vertical_accel_delays_the_climb() -> None:
    """垂向速率同样要建立：``vertical_accel 4 m/s²`` ⇒ 每帧最多长 20 m/s，
    所以第一帧只以 20 m/s 爬（50 s 才到满爬升率的一半）。

    补之前这里是**阶跃**的：平飞转满爬升率发生在一帧之内，与起步端那个
    "0 → 25 m/s" 是同一类缺项（垂向加速度无穷大），只是它藏在子类的钩子
    里，不像水平那一处那么显眼。
    """
    mover, view, _ = make_mover(
        "AIR_MOVER",
        max_speed=0.0,                     # 只留爬升，好数得清
        climb_rate=40.0,
        cruise_altitude=3000.0,
        vertical_accel=4.0,
    )
    mover.move_to_point(0.0, 0.0, 3000.0)

    mover.update(0)
    mover.update(PERIOD_US)
    assert mover.climb_rate_mps() == pytest.approx(20.0)
    assert view.z == pytest.approx(100.0)           # 20 m/s × 5 s

    mover.update(2 * PERIOD_US)
    assert mover.climb_rate_mps() == pytest.approx(40.0)   # 到满速率
    assert view.z == pytest.approx(300.0)                  # +40 m/s × 5 s


def test_the_last_leg_is_capped_by_the_stopping_distance() -> None:
    """最后一段的**目标速率**被"停得下来"压住：``v ≤ √(2·a·dist)``。

    400 m 的目标、``linear_accel 5 m/s²`` ⇒ 起步那一刻的目标就是
    ``√(2×5×400) = 63.2 m/s``，而额定是 100 m/s。少了这一条，实体一路
    100 m/s 冲到终点再一帧清零——**到达即瞬停**，正是起步端那个无穷大
    加速度的另一半。

    **中间航路点不算它**：每过一个路点都刹一次车，那是在模拟"每个路口都
    停车"，而不是"沿这条路走"。
    """
    mover, _, _ = make_mover(max_speed=100.0, linear_accel=5.0)
    mover.move_to_point(400.0, 0.0, 0.0)
    assert mover._speed_goal(None, 400.0, 0.0) == pytest.approx(63.246, abs=0.01)

    nav = StubNav(
        route_cells=[Axial(0, 0), Axial(1, 0), Axial(2, 0)], ground=0.0
    )
    multi, _, _ = make_mover(nav=nav, max_speed=100.0, linear_accel=5.0)
    assert multi.move_to_cell(Axial(2, 0)) is True
    assert len(multi.waypoints()) == 2                    # 还有下一段
    assert multi._speed_goal(None, 300.0, 0.0) == pytest.approx(100.0)


def test_the_turn_speed_limit_is_the_radius_that_can_actually_reach() -> None:
    """几何限速的判据是"**够得着**"：``v ≤ ω·dist / (2·sin θ)``。

    推导：沿半径 ρ 的弧转过转角 φ 之后的位移是 ``(ρ·sinφ, ρ(1−cosφ))``，
    弦与**初始航向**的夹角是 ``φ/2``——不是 φ。目标在航向误差 θ 的方向，
    所以 φ = 2θ，弦长 ``dist = 2ρ·sin(φ/2) = 2ρ·sin θ``，于是
    ``ρ = dist/(2·sin θ)``；半径再大就一定冲过头（够不着）。走"先直线再弧"
    得到同一个上界：``a ≥ 0`` 要求 φ ≥ 2θ，而 ρ 在 φ = 2θ 处最大。

    这一条 v0.13.15 修正过：v0.13.14 写的是 ``2·sin(θ/2)``，把**弦角**当成
    了转角本身。两者差 ``1/cos(θ/2)`` 倍（θ=30° 时 1.03、90° 时 1.41、
    180° 时 2），也就是限速整体偏松，最多松一倍。后果不是"稍微冲一点"：
    实体每一帧都跑到够不着的速度上，到路点的距离反而在涨，立刻撞上"先掉头"
    门限停一帧——实测（潜艇、31 格路线）64% 的帧里程为 0、用时 3560 s；
    改成 ``2·sin θ`` 之后是 24% / 1350 s。

    两个端点值得单独钉住：θ → 0（直着走）**永远够得着**；θ → 180°（目标在
    正后方）也永远够得着——任何半径的圆弧都绕得到它。所以 ``twist`` 在两端
    都退化成 0，速率都不受这一项约束，而 90° 是这条曲线的最低点。
    """
    mover, _, _ = make_mover(turn_rate=30.0, radial_accel=0.0)
    w = radians(30.0)

    for theta, dist in ((10.0, 400.0), (30.0, 346.4), (60.0, 200.0)):
        limit = mover._turn_speed_limit(dist, theta)
        assert limit == pytest.approx(w * dist / (2.0 * sin(radians(theta))))

        # 落在限速上时转弯半径**恰好**够得着目标（弦长 = dist）——这就是这个
        # 公式的物理含义，也是它比 ``2·sin(θ/2)`` 严的那一部分。
        mover._speed = limit
        radius = mover.turn_radius_m()
        assert 2.0 * radius * sin(radians(theta)) == pytest.approx(dist)

    assert mover._turn_speed_limit(1000.0, 0.0) == inf
    # 180° 这里是"极大"而不是 inf：``sin(180°)`` 在浮点上是 1.2e-16 而不是 0。
    # 物理含义一样——目标在正后方时任何半径的圆弧都绕得到，速率不受这一项约束。
    assert mover._turn_speed_limit(1000.0, 180.0) > 1e12
    assert mover._turn_speed_limit(1000.0, 90.0) == pytest.approx(w * 1000.0 / 2.0)

    # 过载那一支走同一条几何：ω = a/v ⇒ v ≤ √(a·dist / (2·sin θ))
    under, _, _ = make_mover(turn_rate=0.0, radial_accel=5.0)
    assert under._turn_speed_limit(200.0, 30.0) == pytest.approx(
        sqrt(5.0 * 200.0 / (2.0 * sin(radians(30.0))))
    )


def test_radial_accel_makes_the_turn_radius_grow_with_speed() -> None:
    """``radial_accel`` 是**跨速度可比**的能力口径：``R = v² / a``。

    同一个件在 25 m/s 下的半径是 10 m/s 下的 6.25 倍（= (25/10)²）。
    换成固定 ``turn_rate`` 就没有这回事——那时半径与速率成正比，v → 0 时
    半径也 → 0，也就是"任何件都能原地掉头"，而那正是它要修的病（§5.7.11）。

    目标放在正北（方位角 0 = 初始航向），好让第一帧不在掉头——掉头那一帧
    速率通道是冻结的，`turn_rate_dps` 会读到 0 速下的结果。
    """
    def probe(max_speed: float) -> tuple[float, float]:
        mover, _, _ = make_mover(
            max_speed=max_speed,
            turn_rate=0.0,          # 关掉机构上限，只看过载那一项
            radial_accel=5.0,
            linear_accel=0.0,       # 一帧到额定速率
        )
        mover.move_to_point(0.0, -100_000.0, 0.0)
        mover.update(0)
        mover.update(PERIOD_US)
        return mover.turn_rate_dps(), mover.turn_radius_m()

    slow_rate, slow_radius = probe(10.0)
    fast_rate, fast_radius = probe(25.0)
    assert slow_rate == pytest.approx(degrees(5.0 / 10.0))
    assert fast_rate == pytest.approx(degrees(5.0 / 25.0))
    assert fast_radius == pytest.approx(slow_radius * 6.25, rel=1e-9)


def test_the_mechanism_ceiling_still_applies_at_low_speed() -> None:
    """过载那一项在 v→0 时发散，所以低速端必须由 ``turn_rate`` 兜底。

    2 m/s 下 ``5 m/s²`` 给出的角速度是 143 °/s（半径 0.8 m）——那不是车，
    那是陀螺。真实机构有个绝对上限，两项取小之后 R(v) 才是 U 形：高速由
    过载压住、低速由机构压住。
    """
    mover, _, _ = make_mover(
        max_speed=2.0, turn_rate=12.0, radial_accel=5.0, linear_accel=0.0
    )
    mover.move_to_point(0.0, -100_000.0, 0.0)
    mover.update(0)
    mover.update(PERIOD_US)
    assert mover.turn_rate_dps() == pytest.approx(12.0)      # 机构那一项说了算
    assert mover.turn_radius_m() == pytest.approx(2.0 / radians(12.0))


def test_turning_in_place_keeps_the_commanded_speed() -> None:
    """掉头那一帧**实际速率为 0**，但速率通道的状态保留下来。

    "先掉头、不前进"是一条抽象（§5.7.11）：那一帧它在转舵，不在走，也没有
    在减速。把状态一起清零的话，每次转向都会附带一次重新加速——对起步慢的
    件（潜艇 0.08 m/s²，150 s 才到全速）那是灾难性的。
    """
    mover, view, _ = make_mover(
        max_speed=20.0, linear_accel=3.0, turn_rate=9.0, angular_accel=0.0
    )
    mover.move_to_point(0.0, 100_000.0, 0.0)    # 正南，与初始航向差 180°
    mover._speed = 20.0                         # 假装它已经在朝北巡航

    mover.update(0)
    mover.update(PERIOD_US)
    assert view.heading == pytest.approx(45.0)              # 转了一帧
    assert mover.travelled_m() == 0.0                       # 还没对准，没走
    assert mover.speed_mps() == 0.0                         # 实际速率是 0
    assert mover.commanded_speed_mps() == pytest.approx(20.0)   # 状态没丢

