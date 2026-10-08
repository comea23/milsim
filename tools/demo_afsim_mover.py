"""把 AFSIM 的机动 demo 在 milsim 里复现，并把两边摆在一起比。

用法::

    python tools/demo_afsim_mover.py \
        --afsim demos/afsim_mover/kinematic_mover_demo.txt \
        --ref   demos/afsim_mover/ref_kinematic_mover.csv \
        --repro demos/afsim_mover/kinematic_mover.txt \
        --out   demos/afsim_mover/repro_kinematic_mover.csv \
        --plot  docs/images/afsim_kinematic_compare.png \
        --seconds 600 --alt-programme kinematic

它做四件事：

1. **读 AFSIM 脚本**（原 demo 或那份重写的合法版），把每个平台的起点、
   航向与整条航路解出来——**航线只有一个出处**，不在这里再抄一份。
2. **把 milsim 平台摆到同一个起点**：位置在格内由想定负责，而**初始高度
   与初速想定里写不了**（§9-22：落格的 z 恒为 0；也没有初速这条），
   所以由这里写进存储与速率状态。
3. **一路推演**：把整条航线（含 `go_to` 那个回环）**一次**交给机动件。
   AFSIM 的 route mover 本来就是"整条 `route` 一次下达"（`use_route`），
   我们的 `Mover.move_along_route` 也是这个口径。**不能每个拐角抛一个
   `move_to_point`**：那会走 `_restart_trip` → `_halt`，三个通道的速率状态
   每过一个航路点清零一次，于是每个拐角都"停一下再重新起步"——绕一圈被切掉
   几百米，速率剖面上一串周期性凹坑，而它看起来只是"这个件有点慢"。
4. **对照**：两边都换算到同一个战区平面坐标系，比航迹（同弧长分数下的
   横向偏差）、高度剖面、速率剖面，并算最小转弯半径。

**为什么不逐时刻对齐**：两边的速率控制不一样（原 demo 每个航路点带速度，
我们只有一个 `max_speed`），跑着跑着就会差出一个航段。逐时刻比出来的
是"到哪了"，同弧长比出来的才是"这条线走得像不像"。
"""

from __future__ import annotations

import csv
import math
import re
import sys
import unicodedata
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.services.map.hex import LocalFrame, offset_to_heading  # noqa: E402
from milsim.simulation import Simulation  # noqa: E402

#: 驱动步长。原 demo 的 `update_interval` 是 0.1 sec，复现想定的 `max_step`
#: 也是 0.1 s——**采样、节拍、驱动三者用同一个数**，否则量出来的转弯快慢里
#: 混着采样粒度的差别。
DT_S = 0.1

_FT = 0.3048
_NM = 1852.0
_KTS = _NM / 3600.0
_MPH = 1609.344 / 3600.0
_G = 9.80665

#: ``position 33:54:44.51n 83:20:16.78w altitude 32808.40 ft``
#:
#: **经纬度有两种合法写法，AFSIM 的 demo 里混着用**：度分秒（``33:54:44.51n``）
#: 与十进制（``00.31532n``）。``demos/route_finder_demos/3d_route_finder.txt``
#: 里 SAM 全是度分秒、而 target 是十进制的 ``00.31532n 00.73940e``——只认一种
#: 会把那一行漏掉（漏掉的表现是坐标停在默认值 0，不报错）。
_POS = re.compile(
    r"position\s+"
    r"(?:(?P<lat_d>\d{1,3}):(?P<lat_m>\d{1,2}):(?P<lat_s>[\d.]+)|(?P<lat_dec>[\d.]+))"
    r"\s*(?P<lat_h>[ns])\s+"
    r"(?:(?P<lng_d>\d{1,3}):(?P<lng_m>\d{1,2}):(?P<lng_s>[\d.]+)|(?P<lng_dec>[\d.]+))"
    r"\s*(?P<lng_h>[we])"
    r"(?:\s+altitude\s+(?P<alt>-?[\d.]+)\s*(?P<alt_u>ft|m))?",
    re.I,
)
_SPEED = re.compile(r"speed\s+([\d.]+)\s*(kts|mph|m/s)\b", re.I)

#: ``label begin`` —— 挂在**紧随其后的那个 position** 上（见 AFSIM 的 route 文档：
#: "Associates a string label with the immediately following waypoint definition"）。
_LABEL = re.compile(r"(?mi)^\s*label\s+(\S+)")

#: ``go_to begin`` / ``goto begin`` —— **两种拼法都合法**，实测都生效：原版 demo
#: 的 flyer1 写 `go_to`、flyer2/3/4 写 `goto`，四条轨迹都绕着各自的航线循环。
_GOTO = re.compile(r"(?mi)^\s*go_?to\s+(\S+)")


# ---------------------------------------------------------------------------
# AFSIM 脚本 → 航线
# ---------------------------------------------------------------------------


def _dms(deg: str, minute: str, sec: str, hemi: str) -> float:
    value = float(deg) + float(minute) / 60.0 + float(sec) / 3600.0
    return -value if hemi.lower() in ("s", "w") else value


def _coord(match: "re.Match[str]", axis: str) -> float:
    """从 :data:`_POS` 的一个匹配里取纬度（``"lat"``）或经度（``"lng"``）。

    两种写法共用一条出口：度分秒走 :func:`_dms`（它自己处理南北东西），
    十进制直接取值。符号一律由 ``n/s`` / ``e/w`` 那个字母决定。
    """
    decimal = match.group(f"{axis}_dec")
    if decimal is None:
        return _dms(match.group(f"{axis}_d"), match.group(f"{axis}_m"),
                    match.group(f"{axis}_s"), match.group(f"{axis}_h"))
    value = float(decimal)
    hemi = match.group(f"{axis}_h").lower()
    return -value if hemi in ("s", "w") else value


def _altitude(value: str | None, unit: str | None, default: float) -> float:
    if value is None:
        return default
    return float(value) * (_FT if unit and unit.lower() == "ft" else 1.0)


def _speed_mps(value: str, unit: str) -> float:
    factor = {"kts": _KTS, "mph": _MPH, "m/s": 1.0}[unit.lower()]
    return float(value) * factor


class AfsimPlatform:
    """AFSIM 脚本里一个平台：起点、航向、航路。"""

    __slots__ = ("name", "lat", "lng", "alt", "heading_deg", "route",
                 "loop_index", "processors")

    def __init__(self, name: str) -> None:
        self.name = name
        self.lat = 0.0
        self.lng = 0.0
        self.alt = 0.0
        self.heading_deg = 0.0
        #: ``[(lat, lng, alt_m, speed_mps or None), ...]``
        self.route: list[tuple[float, float, float, float | None]] = []
        #: ``go_to <label>`` 跳回的那个航路点在 `route` 里的下标；**没有
        #: `go_to` 就是 ``None``**（航路走完就结束，不是绕圈）。这两件事必须
        #: 分开：绕圈是脚本显式写出来的，不是航路的默认行为。
        self.loop_index: int | None = None
        #: 这个平台挂了哪些处理器型号。**必须有**：原脚本里高度改写器
        #: `VARY_DIR_ALT` 只用 `add processor` 挂在 **flyer1** 上，flyer2/3/4
        #: 没有。照抄成"所有平台都做高度改写"会让另外三架的对照凭空多出一段
        #: 20 km 的爬升——那是驱动脚本的错，不是模型的差别。
        self.processors: set[str] = set()


def _scan_positions(text: str, default_alt: float):
    """按出现顺序扫出 ``position``，并把紧随其后的 ``speed`` 挂上去。

    航路点的高度可能写成 ``altitude 32808.40 ft``，也可能只写 ``speed``。
    speed 常被写到下一行（原 demo 的 flyer2/flyer3 就是这样），所以不能按行读。
    """
    marks: list[tuple[int, tuple[float, float, float, float | None]]] = []
    for match in _POS.finditer(text):
        lat = _coord(match, "lat")
        lng = _coord(match, "lng")
        alt = _altitude(match.group("alt"), match.group("alt_u"), default_alt)
        marks.append((match.end(), (lat, lng, alt, None)))

    for match in _SPEED.finditer(text):
        speed = _speed_mps(match.group(1), match.group(2))
        # 挂给**它前面最近的那个** position
        for index in range(len(marks) - 1, -1, -1):
            if marks[index][0] <= match.start():
                lat, lng, alt, _old = marks[index][1]
                marks[index] = (marks[index][0], (lat, lng, alt, speed))
                break

    return [item[1] for item in marks]


def parse_afsim(path: Path) -> dict[str, AfsimPlatform]:
    """解出脚本里所有顶层 ``platform`` 的起点与航路。

    只认 ``platform <名> <型号> ... end_platform`` 这种顶层块。型号里的
    mover 参数**刻意不读**——那些参数在本项目里写在复现想定中，
    一处一处地对（README 有映射表），在这里再解一遍就是第二个真相。
    """
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    platforms: dict[str, AfsimPlatform] = {}

    index = 0
    while index < len(lines):
        head = lines[index].strip()
        words = head.split()
        if len(words) >= 2 and words[0] == "platform" and not head.startswith("platform_type"):
            name = words[1]
            body: list[str] = []
            index += 1
            while index < len(lines) and lines[index].strip() != "end_platform":
                body.append(lines[index])
                index += 1
            platforms[name] = _build_platform(name, "\n".join(body))
        index += 1

    return platforms


def _route_loop(text: str) -> int | None:
    """``go_to <label>`` 跳回的那个航路点，在 route 里是第几个。

    ``label`` 挂在**紧随其后的那个 ``position``** 上，所以要按两者在文本里
    的**出现次序**配起来——不是"标签行后面的第一行"，因为原 demo 里
    ``position`` 与 ``speed`` 常常被拆在相邻两行（flyer2/flyer3 就是这样），
    而 ``label`` 与 ``position`` 之间还可能夹着注释。

    没有 ``go_to`` 时返回 ``None``：**航路走完就结束，不是绕圈**。原版 demo
    的四条航线都有 ``goto``，而 mover_demo 那份的七个平台也都有——所以这个
    分支不是"没实现"，是"脚本里真没有"。
    """
    events: list[tuple[int, str]] = []
    events.extend((match.start(), "label:" + match.group(1))
                  for match in _LABEL.finditer(text))
    events.extend((match.start(), "point") for match in _POS.finditer(text))
    events.sort(key=lambda item: item[0])

    labels: dict[str, int] = {}
    index = -1
    pending: str | None = None
    for _at, kind in events:
        if kind.startswith("label:"):
            pending = kind[len("label:"):]
            continue
        index += 1
        if pending is not None:
            labels[pending] = index
            pending = None

    jump = _GOTO.search(text)
    return None if jump is None else labels.get(jump.group(1))


def _build_platform(name: str, body: str) -> AfsimPlatform:
    plat = AfsimPlatform(name)

    # 航路块单独摘出来：起点是块**外**那个 position，航路点是块内那些。
    split = re.search(r"(?mi)^\s*route\b", body)
    if split is not None:
        outside, inside = body[: split.start()], body[split.start():]
        stop = re.search(r"(?mi)^\s*end_route\b", inside)
        inside = inside[: stop.start()] if stop else inside
    else:
        outside, inside = body, ""

    outer = _scan_positions(outside, 0.0)
    if outer:
        plat.lat, plat.lng, plat.alt, _ = outer[0]

    heading = re.search(r"(?mi)^\s*heading\s+(-?[\d.]+)\s*deg", outside)
    if heading:
        plat.heading_deg = float(heading.group(1)) % 360.0

    # `add processor <实例名> <型号> [end_processor]` —— 只取型号。
    for match in re.finditer(
        r"(?mi)^\s*add\s+processor\s+\S+\s+(\S+)", body
    ):
        plat.processors.add(match.group(1))

    plat.route = _scan_positions(inside, plat.alt)
    plat.loop_index = _route_loop(inside)
    return plat


# ---------------------------------------------------------------------------
# 复现：摆起点、按航路推进
# ---------------------------------------------------------------------------


#: 绕圈的航线要**先展开成一条更长的航线**再交下去，而不是每转完一圈重新
#: 下达一次。展开的理由是终点那一条"停得下来"（``_speed_goal`` 只在**最后
#: 一段**算减速）：每圈都走一遍终点就每圈刹一次车。展开几圈按**这趟要跑多久**
#: 估，宁可多绕也不能少——少展开了它就真的在最后一个点停住。
_LAP_SLACK = 1.6


class Repro:
    """一个 milsim 平台上挂一条 AFSIM 航路。"""

    __slots__ = ("name", "eid", "mover", "points", "initial_speed",
                 "alt_programme")

    def __init__(self, name, eid, mover, points, initial_speed):
        self.name = name
        self.eid = eid
        self.mover = mover
        #: 世界平面坐标 + 高度的**整条航线**（绕圈已按 :data:`_LAP_SLACK`
        #: 展开）。**一次交给机动件**，不是一条腿一条腿地交。
        self.points: list[tuple[float, float, float]] = points
        #: 原 demo 第一个航路点的速度。机动件没有"初速"这条参数，速率状态
        #: 一律从 0 起（见 :func:`_seed`）。
        self.initial_speed = initial_speed
        #: 这个平台要不要吃高度改写（原脚本只把它挂在 flyer1 上）。
        self.alt_programme = False

    def launch(self) -> None:
        """下达整条航线，**然后把初速补回去**。

        顺序不能反：``move_along_route`` 会走 ``_restart_trip`` → ``_halt``，
        把三个通道的速率状态清零。先写初速再下命令 = 初速被当场冲掉，飞机
        从 0 重新加速——前 8 秒的速率剖面整个是错的，而它看起来只是"起步慢"。
        """
        self._hand_over(self.points)
        if self.initial_speed > 0.0:
            self.mover._speed = self.initial_speed

    def remaining(self) -> list[tuple[float, float, float]]:
        """从**当前正飞的那一腿**起的剩余航线。

        高度改写要用它：改写的是"剩下的路飞多高"，不是"从头再飞一遍"。

        读 ``_leg`` 是**故意的**。"现在在第几腿"只有机动件知道，驱动脚本
        自己数一份就是同一件事的第二个真相——而改写高度那三次正好落在拐角
        附近，两处一旦不同步，改的就不是该改的那一腿。机动件没有公开的
        "第几腿"查询，为一个 demo 驱动脚本往模型的公开面上加一个访问器
        不值当（它只被这里用到一次），所以直接读。
        """
        leg = self.mover._leg
        return self.points[leg:] if leg < len(self.points) else []

    def issue(self, override_z: float | None) -> None:
        """重新下达**剩余的**航线，高度按 ``override_z`` 改写。

        ``None`` = 把高度交还给航线自己的高度剖面（原脚本里
        ``ReturnToRoute()`` 的等价物）。原脚本的 ``GoToAltitude(alt)`` 是
        **平台级**命令"保持航线、只改高度目标"，所以改写落在**剩下的所有**
        高度目标上——不是只改下一腿。这一点与原实现一致。
        """
        rest = self.remaining()
        if not rest:
            return
        if override_z is not None:
            rest = [(x, y, override_z) for x, y, _z in rest]
        self._hand_over(rest)

    def _hand_over(self, points) -> None:
        """下达航线 + **把速率状态接回去**。

        ``move_along_route`` 与另两个下达口一样走 ``_restart_trip``，三个
        通道的速率状态归零。而这里每一次重新下达（高度改写、绕圈接力）都是
        **同一趟在继续**，速率不该回零：回零的代价是停 0.9 s 再花 8 s 加速
        回来，速率剖面上一串周期凹坑，而它看起来只是"这个件有点慢"。

        ``_started_us`` 一起接回去，否则 `update()` 会把下一帧当成"接到命令的
        第一帧"——只记开工时刻、不挪窝，等于白丢一帧。
        """
        speed, started = self.mover._speed, self.mover._started_us
        self.mover.move_along_route(points)
        self.mover._speed = speed
        self.mover._started_us = started


#: 判定"这个航路点就是起点本身"的阈值（米）。原 demo 的 `route` 第一点都
#: 与平台的 `position` 逐字相同，所以这个阈值只会命中那一个点。
_COINCIDENT_M = 1.0


def _route_entry(frame, src: AfsimPlatform, x0: float, y0: float) -> int:
    """航路上**第一个不与起点重合**的点是第几个。

    原 demo 的 `route` 把起点本身写成第一个 `position`，真正要飞的是第二个。
    照抄的话复现会先"飞到自己脚下"再换腿——多一帧无用指令，而且让
    "第一腿"这个概念在两边错开一位。
    """
    for index, (lat, lng, _alt, _speed) in enumerate(src.route):
        x, y = frame.geo_to_world(lat, lng)
        if math.hypot(x - x0, y - y0) > _COINCIDENT_M:
            return index
    return 0


def _initial_heading_deg(frame, src: AfsimPlatform, x0: float, y0: float) -> float:
    """起点该朝哪：**指向航路上第一个不与起点重合的点**。

    这条不是我替 AFSIM 编的规则，是对着参照轨迹量出来的：把 7 个平台在 t=0
    的航向与它们各自的航线逐一对，**全部都等于"起点指向第一个航路点"的方位角**
    （kinematic 那份的 `heading 215 deg` 只是恰好与它差 0.3°，所以光看那一份
    看不出来；aircraft 那份差得最多——MAX_G 声明 180°、实际 1.8°）。
    两个件都是这样：**一旦开始走航线，机头就已经在航线的方向上了**。
    照抄想定里那句 `heading` 会让复现平白多出一段几十度的掉头，而这段掉头在
    参照侧根本不存在——那几十度会原样进入横向偏差，把真正的差别盖掉。

    用的是机动件自己的 :func:`offset_to_heading`，**不在这里再写一份方位角
    公式**：`set_pose` 写进去的航向必须与机动件算出来的那个同口径，否则第一帧
    就会多出一次"掉头"。
    """
    entry = _route_entry(frame, src, x0, y0)
    if 0 <= entry < len(src.route):
        lat, lng, _alt, _speed = src.route[entry]
        x, y = frame.geo_to_world(lat, lng)
        if math.hypot(x - x0, y - y0) > _COINCIDENT_M:
            return offset_to_heading(x - x0, y - y0)
    return src.heading_deg


def _geographic_heading(heading_deg: float) -> float:
    """机动件的航向 → **地理**航向（写 CSV 用）。

    绕开的是 `hex.py` 里一个**既有的、与本 demo 无关的缺陷**：``geo_to_world``
    把正北算在 **+y**，而 ``heading_to_offset`` / ``offset_to_heading``
    （有测试钉死：``heading_to_offset(0°, 100) == (0, -100)``）把正北算在
    **-y**。两套 y 轴相反，于是整条链上的航向数都相对地理**南北镜像**。

    一锤定音的证据（探针把平台摆在锚点正南、命令它去正北的点）：

        offset_to_heading(目标 - 起点) = 180.00°
        第 1100 帧  lat=34.000000  （纬度确实升高了 = 地理上真往北飞）
        heading = 180.00°          （而 180° 在项目自己的口径里指向正南）

    ⇒ **位移是对的，航向数是镜像的**。所以两边直接比绝对航向会差一个
    ``180° - h``，而按 ``|Δ航向|`` 算的一切（角速度、转弯半径）不受影响。
    这里把它换算成地理口径，只为让 CSV 与 AFSIM 的 ``Heading()`` 能并排看。

    修不修 ``hex.py`` 是另一件事——那会改变 `hex` 坐标在想定里的含义，
    不属于本 demo 的范围。见 ``docs/00-框架设计.md`` §9。
    """
    return (180.0 - heading_deg) % 360.0


def _route_points(frame, src: AfsimPlatform, seconds: float, top_speed: float):
    """航路 → 世界平面坐标的**整条航线**（绕圈已展开）。

    三件事：

    1. **回环从 ``go_to`` 的那个标签处起算**。AFSIM 的 ``go_to begin`` 是
       "走到最后一个航路点就跳回 ``begin``"，所以一圈 = ``route[begin:]``；
       没有 ``go_to`` 就没有回环（航路走完即结束），``laps`` 取 1。
    2. **第一圈从"第一个不与起点重合的点"开始**（见 :func:`_route_entry`）：
       回环里那个起点自己的航路点是**终点线**，它属于第二圈；第一圈先飞它
       只会多出一帧朝零向量的转向，而零向量的方位角是没有意义的。
    3. **展开成 ``laps`` 圈**，见 :data:`_LAP_SLACK`。
    """
    if not src.route:
        return [], 0

    loop_start = src.loop_index or 0
    cycle = [
        (*frame.geo_to_world(lat, lng), alt)
        for lat, lng, alt, _speed in src.route[loop_start:]
    ]
    if not cycle:
        return [], 0

    entry = max(0, _route_entry(frame, src, *frame.geo_to_world(src.lat, src.lng))
                - loop_start)

    span = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(cycle, cycle[1:])
    )
    span += math.hypot(cycle[0][0] - cycle[-1][0], cycle[0][1] - cycle[-1][1])
    if src.loop_index is None or span <= 0.0 or top_speed <= 0.0:
        laps = 1
    else:
        laps = 2 + int(_LAP_SLACK * seconds * top_speed / span)
    return cycle[entry:] + cycle * laps, entry


def _seed(sim, frame, name: str, src: AfsimPlatform, seconds: float) -> Repro:
    """把平台摆到 AFSIM 的起点：位置 + **初始高度** + 航向 + 初速，并把航线交下去。

    位置本来就在想定里（`position latlng ...`，与这里同一对坐标），但
    **高度、航向与初速想定写不了**：

    - 高度：落格的 z 恒为 0（§9-22）。原 demo 从 10000 m 起步，不写进去
      前几分钟就是一段"从海平面慢慢爬"，两边一比全是它。
    - 航向：见 :func:`_initial_heading_deg`。
    - 初速：机动件没有"初速"这条参数，速率状态一律从 0 起。
    """
    eid = sim.registry.by_name(name).entity_id
    mover = sim.entities[eid].require("mover")
    x, y = frame.geo_to_world(src.lat, src.lng)

    entry = _route_entry(frame, src, x, y)
    first = src.route[entry] if src.route else None
    speed = first[3] if first is not None and first[3] is not None else 0.0

    sim.store.set_pose(eid, x, y, src.alt, _initial_heading_deg(frame, src, x, y),
                       speed)

    points, _at = _route_points(frame, src, seconds,
                               float(mover.spec["max_speed"]))
    item = Repro(name, eid, mover, points, speed)
    item.alt_programme = _ALT_PROCESSOR in src.processors
    return item


#: 原脚本里那个"中途改高度"的处理器型号。**挂在哪个平台上由脚本说了算**，
#: 不在这里再写一份名单——原脚本只有 flyer1 挂了它。
_ALT_PROCESSOR = "VARY_DIR_ALT"

#: VARY_DIR_ALT 那个处理器的等价改写：``(起, 止, 做法, 数)``。
#: ``add`` = 在当前高度上加；``set`` = 直接给一个高度；``route`` = 取消改写。
ALT_PROGRAMMES = {
    "kinematic": (
        (25.0, 35.0, "add", 20000.0),
        (100.0, 120.0, "set", 4000.0),
        (200.0, 220.0, "route", 0.0),
    ),
}


def run_repro(
    scenario: Path,
    afsim: dict[str, AfsimPlatform],
    out_csv: Path,
    seconds: float,
    alt_programme: str | None,
) -> tuple[dict[str, list[tuple[float, float, float, float, float, float]]], LocalFrame]:
    """装想定、摆起点、按航路推演。

    返回 ``({平台名: [(t, lat, lng, alt, v, heading)]}, frame)``——**frame 交出去**
    是有意的：对照那边要把参照与复现都换算到同一个战区平面，用的必须是这里
    装出来的那一个（重复装配想定会让两边落在两套坐标上）。
    """
    sim = Simulation.from_scenario_file(scenario)
    sim.build()
    sim.initialize()

    zone_ids = [spec.zone_id for spec in sim.maps]
    frame = sim.maps.zone(zone_ids[0]).frame

    repros: list[Repro] = []
    for name in sorted(afsim):
        if sim.registry.by_name(name) is None:
            continue
        item = _seed(sim, frame, name, afsim[name], seconds)
        item.launch()                      # 下航线 → 再补初速，顺序不能反
        repros.append(item)

    if not repros:
        raise SystemExit("想定里没有一个平台名与 AFSIM 脚本对得上")

    history: dict[str, list[tuple[float, float, float, float, float, float]]] = {
        item.name: [] for item in repros
    }
    programme = ALT_PROGRAMMES.get(alt_programme or "", ())
    fired: set[tuple[str, int]] = set()
    #: 航路走完（含展开的那几圈）就再也接不上了。真出现说明展开圈数不够，
    #: 得报出来——不然它在图上只是"后半段不动了"。
    exhausted: set[str] = set()

    def record(item: Repro, when: float) -> None:
        pose = sim.store.pose_of(item.eid)
        if pose is None:
            return
        x, y, z, heading, speed = pose
        lat, lng = frame.world_to_geo(x, y)
        history[item.name].append((when, lat, lng, z, speed, heading))

    # t=0 也要采一帧：参照那边第一帧就在 t=0（`observer` 在装配期就回调过一次），
    # 少这一帧等于两条轨迹的弧长原点差一个 ``v·dt``——按弧长对齐时它就成了
    # 一个恒定的沿航向偏差，白送给"横向偏差"这一列。
    for item in repros:
        record(item, sim.engine.now / 1_000_000.0)

    steps = max(1, int(round(seconds / DT_S)))
    for _ in range(steps):
        sim.run_for(int(DT_S * 1_000_000))
        now = sim.engine.now / 1_000_000.0

        for item in repros:
            pose = sim.store.pose_of(item.eid)
            if pose is None:
                continue
            z = pose[2]

            # -- VARY_DIR_ALT 的等价改写：换高度目标但不换水平航路 --
            # **只对挂了那个处理器的平台做**（原脚本只有 flyer1 挂了）。
            for order, (lo, hi, mode, value) in enumerate(programme):
                if not item.alt_programme:
                    continue
                if not (lo <= now < hi) or (item.name, order) in fired:
                    continue
                fired.add((item.name, order))
                if mode == "add":
                    item.issue(z + value)
                elif mode == "set":
                    item.issue(value)
                else:
                    item.issue(None)       # ReturnToRoute：高度交还给航线

            if item.mover.arrived():
                exhausted.add(item.name)
            record(item, now)

    if exhausted:
        print(f"  · 注意：{', '.join(sorted(exhausted))} 的展开航路走完了"
              "（后半段没有航线，会停住）")

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["time_s", "platform", "lat_deg", "lon_deg", "alt_m", "speed_mps",
             "heading_deg"]
        )
        for name, rows in history.items():
            for now, lat, lng, z, speed, heading in rows:
                writer.writerow([f"{now:.1f}", name, f"{lat:.7f}", f"{lng:.7f}",
                                 f"{z:.3f}", f"{speed:.3f}",
                                 f"{_geographic_heading(heading):.4f}"])

    print(f"  · 复现 {len(repros)} 个平台、{steps} 步 → {out_csv}")
    return history, frame


# ---------------------------------------------------------------------------
# 对照
# ---------------------------------------------------------------------------


def load_csv(path: Path):
    """读参照/复现 CSV → ``{平台名: [(t, x, y, z, v, heading)]}``。"""
    table: dict[str, list[tuple[float, ...]]] = {}
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            name = row["platform"]
            table.setdefault(name, []).append(
                (
                    float(row["time_s"]),
                    float(row["lat_deg"]),
                    float(row["lon_deg"]),
                    float(row["alt_m"]),
                    float(row["speed_mps"]),
                    float(row.get("heading_deg") or "nan"),
                )
            )
    return table


def _flat(track, frame):
    """(t, lat, lng, z, v, heading) → (t, x, y, z, v, heading)，米制平面。"""
    out = []
    for t, lat, lng, z, v, heading in track:
        x, y = frame.geo_to_world(lat, lng)
        out.append((t, x, y, z, v, heading))
    return out


def _cumulative(track, *, three_d: bool = False):
    """累计航迹长度，返回等长的前缀和列表。

    **默认只算水平**：同弧长重采样要的"进度"是"沿这条航线的水平走向走了多远"，
    而且航线本身是水平的（`route` 的 waypoint 高度只改高度目标，不改水平走向）。

    ``three_d`` 为真时带上爬升。**与 AFSIM 比飞行距离时必须用三维**：AFSIM 的
    `Speed()` 是**三维速度矢量的大小**，它的运动学件"爬升时把水平分量让出去"
    （实测 flyer1 在爬升段的水平速度掉到 13 m/s），所以只看水平投影会把它的
    航迹算短——flyer1 水平 40.3 km / 三维 61.3 km，差的 21 km 全在爬升里。
    """
    total = 0.0
    acc = [0.0]
    for (_, x0, y0, z0, *_a), (_, x1, y1, z1, *_b) in zip(track, track[1:]):
        step = math.hypot(x1 - x0, y1 - y0)
        if three_d:
            step = math.hypot(step, z1 - z0)
        total += step
        acc.append(total)
    return acc


def _sample_at(track, acc, s: float):
    """按累计弧长取一个点（线性插值）。"""
    lo, hi = 0, len(acc) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if acc[mid] < s:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return track[0]
    span = acc[lo] - acc[lo - 1]
    ratio = 0.0 if span <= 0.0 else (s - acc[lo - 1]) / span
    a, b = track[lo - 1], track[lo]
    return tuple(a[i] + (b[i] - a[i]) * ratio for i in range(len(a)))


#: 转弯指标的重采样步长（秒）。**不能拿逐帧差分的最大值当角速度**：参照那边
#: 同一个时刻会出现两个样本（AFSIM 的 `GoToAltitude` 会让机动件额外来一次
#: `MOVER_UPDATED`），dt 趋近 0 时一点航向差也能除出上百 °/s。实测 flyer1
#: 这样量出 127 °/s，而它的 `maximum_body_turn_rate` 只有 65 °/s——那 2 倍不是
#: AFSIM 超了限，是除数里混进了零点几毫秒。等时重采样把这个坑填掉。
_TURN_STRIDE_S = 0.5

#: 认为"这一帧没在走"的速率阈值（m/s）。用来量停摆占比。
_STALL_MPS = 5.0

#: 算最小转弯半径时要求速率不低于最大速率的这个比例。见 :func:`_turn_stats`。
_CRUISE_FRACTION = 0.5


def _unwrap_headings(track):
    """航向解缠并补空值。跨 ±180° 要 ±360° 补偿，否则"转过正北"会算成 359°。"""
    out: list[float] = []
    previous = 0.0
    offset = 0.0
    for row in track:
        heading = row[5]
        if math.isnan(heading):
            out.append(previous)         # 缺值就沿用上一个，不制造假的跳变
            continue
        while heading + offset - previous > 180.0:
            offset -= 360.0
        while heading + offset - previous < -180.0:
            offset += 360.0
        previous = heading + offset
        out.append(previous)
    return out


def _on_grid(track, stride: float):
    """按等时间间隔重采样出 ``[(t, 航向, 三维速率, 水平速率), ...]``（线性插值）。

    速率是**位置差分**出来的，**不用 CSV 里的 `speed_mps` 列**——两边的这一列
    不是一回事：AFSIM 的 `Speed()` 是**三维速度矢量的大小**，而我们写进去的是
    机动件的**水平**速率状态（垂向是另一条通道）。拿它们直接比，会把我们
    "原地转向、同时在升降"的帧（实测那些帧垂向速率均 60 m/s、最大 100 m/s）
    算成"停住了"，而 AFSIM 那边同样在升降的帧算成"没停"。两边同口径只有一个
    办法：都从位置差分。

    等时网格同时解决 AFSIM 的重复采样——`GoToAltitude` 会让机动件在同一时刻
    多出一次 `MOVER_UPDATED`，逐帧差分会把 `dt` 逼近 0，除出假的角速度。
    """
    headings = _unwrap_headings(track)
    times = [row[0] for row in track]
    out: list[tuple[float, float, float, float]] = []
    index = 0
    when = times[0]
    previous: tuple[float, float, float, float] | None = None
    while when <= times[-1] + 1e-9:
        while index + 1 < len(times) and times[index + 1] < when:
            index += 1
        nxt = min(index + 1, len(times) - 1)
        span = times[nxt] - times[index]
        ratio = 0.0 if span <= 0.0 else (when - times[index]) / span
        heading = headings[index] + (headings[nxt] - headings[index]) * ratio
        here = tuple(track[index][k] + (track[nxt][k] - track[index][k]) * ratio
                     for k in (1, 2, 3))                   # x, y, z（米）
        v3 = vh = 0.0
        if previous is not None and when > previous[0]:
            dt = when - previous[0]
            flat = math.hypot(here[0] - previous[1], here[1] - previous[2])
            vh = flat / dt
            v3 = math.hypot(flat, here[2] - previous[3]) / dt
        out.append((when, heading, v3, vh))
        previous = (when, here[0], here[1], here[2])
        when += stride
    return out


def _turn_stats(track) -> tuple[float, float, float, float]:
    """返回 ``(峰值角速度 °/s, 最小转弯半径 m, 停摆占比, 零前进占比)``。

    角速度在**等时网格**上差分（见 ``_TURN_STRIDE_S``）。半径是
    ``速率 ÷ 角速度``（§5.7.11 的那个定义，不另算一个数），但**排除速率趋零
    的帧**——一个停在原地的"转弯半径 0"是除法的假象，不是机动能力。所以只取
    速率还在最大速率一半以上的帧；速率本身就小的那段由后两个数单独交代。

    **两个"停"要分开看**：

    * **停摆**：三维速率 < `_STALL_MPS` —— 真的没在动。
    * **零前进**：**水平**速率 < `_STALL_MPS` —— 水平没挪窝，但可能正在升降。
      这正是基类 `STEER_BEFORE_MOVE_DEG`（"先掉头、不前进"）在空中航线上的
      样子，与"停了"是两回事（§9-20 / §9-21）。
    """
    grid = _on_grid(track, _TURN_STRIDE_S)
    if len(grid) < 3:
        return 0.0, math.inf, 0.0, 0.0

    fastest = max(item[2] for item in grid)
    peak = 0.0
    radius = math.inf
    stalled = sum(1 for item in grid[1:] if item[2] < _STALL_MPS)
    no_advance = sum(1 for item in grid[1:] if item[3] < _STALL_MPS)
    for (t0, h0, v0, _u0), (t1, h1, _v1, _u1) in zip(grid, grid[1:]):
        omega = abs(h1 - h0) / (t1 - t0)     # 航向是度，除以秒就是 °/s
        peak = max(peak, omega)
        # 只在**巡航速率附近**取半径：速率掉到一半以下时算出来的小半径是
        # "它慢下来了"，不是"它转得动"。否则这个数会由停摆段主导，读出来的
        # 是速度剖面而不是转弯能力。
        if omega >= 1.0 and v0 >= _CRUISE_FRACTION * fastest:
            radius = min(radius, v0 / math.radians(omega))
    span = len(grid) - 1
    return peak, radius, stalled / span, no_advance / span


def _offset_to_polyline(point, polyline) -> float:
    """点到折线的**最近距离**（逐段算点到线段距离）。

    这是两边**都能算**的形状指标：AFSIM 的参照航迹与复现航迹都要贴着同一条
    航线飞，谁贴得住由这个数说。它与"同弧长偏差"互补——后者会把每圈被切掉的
    那几百米逐圈累加成公里级的相位差，看起来像"整条线都走错了"。
    """
    px, py = point[1], point[2]
    best = math.inf
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        if span <= 0.0:
            ratio = 0.0
        else:
            ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / span))
        best = min(best, math.hypot(px - (ax + dx * ratio), py - (ay + dy * ratio)))
    return best


def _route_offset(track, polyline) -> tuple[float, float]:
    if len(polyline) < 2:
        return math.inf, math.inf
    values = [_offset_to_polyline(point, polyline) for point in track]
    return sum(values) / len(values), max(values)


def compare(ref_track, rep_track, polyline, samples=400) -> dict[str, float]:
    """按**同弧长分数**把两条航迹摆在一起比，并各自与航线折线比。"""
    ref_acc = _cumulative(ref_track)
    rep_acc = _cumulative(rep_track)
    span = min(ref_acc[-1], rep_acc[-1])
    if span <= 0.0:
        return {}

    worst = 0.0
    total = 0.0
    dz_total = 0.0
    for index in range(samples):
        s = span * index / (samples - 1)
        a = _sample_at(ref_track, ref_acc, s)
        b = _sample_at(rep_track, rep_acc, s)
        d = math.hypot(a[1] - b[1], a[2] - b[2])
        total += d
        worst = max(worst, d)
        dz_total += abs(a[3] - b[3])

    ref_peak, ref_radius, ref_stall, ref_zero = _turn_stats(ref_track)
    rep_peak, rep_radius, rep_stall, rep_zero = _turn_stats(rep_track)
    ref_off_mean, ref_off_max = _route_offset(ref_track, polyline)
    rep_off_mean, rep_off_max = _route_offset(rep_track, polyline)
    return {
        "ref_len": ref_acc[-1],
        "rep_len": rep_acc[-1],
        "ref_len3": _cumulative(ref_track, three_d=True)[-1],
        "rep_len3": _cumulative(rep_track, three_d=True)[-1],
        "d_mean": total / samples,
        "d_max": worst,
        "dz_mean": dz_total / samples,
        "ref_speed": _mean_speed(ref_track, three_d=True),
        "rep_speed": _mean_speed(rep_track, three_d=True),
        "ref_turn": ref_peak,
        "rep_turn": rep_peak,
        "ref_radius": ref_radius,
        "rep_radius": rep_radius,
        "ref_stall": ref_stall * 100.0,
        "rep_stall": rep_stall * 100.0,
        "ref_zero": ref_zero * 100.0,
        "rep_zero": rep_zero * 100.0,
        "ref_off": ref_off_mean,
        "ref_off_max": ref_off_max,
        "rep_off": rep_off_mean,
        "rep_off_max": rep_off_max,
    }


def _mean_speed(track, *, three_d: bool = False) -> float:
    if len(track) < 2:
        return 0.0
    span = track[-1][0] - track[0][0]
    if span <= 0.0:
        return 0.0
    return _cumulative(track, three_d=three_d)[-1] / span


def _pad(text: str, width: int, *, right: bool = False) -> str:
    """按**显示宽度**补齐（东亚宽字符占两列）。"""
    fill = " " * max(0, width - sum(
        2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text
    ))
    return fill + text if right else text + fill


def _number(value: float, digits: int = 0) -> str:
    """``inf`` 写成 ``—``：没有解出来的数不能长得像一个数。"""
    return "—" if not math.isfinite(value) else f"{value:.{digits}f}"


def report(names, ref, rep, stats) -> None:
    widths = (18, 11, 11, 9, 10, 8, 8, 13, 13, 13, 13)
    print()
    print(_pad("平台", widths[0])
          + _pad("水平航迹 km", widths[1], right=True)
          + _pad("三维航迹 km", widths[2], right=True)
          + _pad("平均 m/s", widths[3], right=True)
          + _pad("峰值 ω °/s", widths[4], right=True)
          + _pad("停摆 %", widths[5], right=True)
          + _pad("零前进 %", widths[6], right=True)
          + _pad("到航线 参照", widths[7], right=True)
          + _pad("到航线 复现", widths[8], right=True)
          + _pad("巡航 R m", widths[9], right=True)
          + _pad("同弧长偏差", widths[10], right=True))
    print(_pad("", widths[0])
          + _pad("参照/复现", widths[1], right=True)
          + _pad("参照/复现", widths[2], right=True)
          + _pad("参照/复现", widths[3], right=True)
          + _pad("参照/复现", widths[4], right=True)
          + _pad("三维", widths[5], right=True)
          + _pad("水平", widths[6], right=True)
          + _pad("平均/最大", widths[7], right=True)
          + _pad("平均/最大", widths[8], right=True)
          + _pad("参照/复现", widths[9], right=True)
          + _pad("平均/最大", widths[10], right=True))
    for name in names:
        row = stats.get(name)
        if not row:
            continue
        cells = (
            f"{row['ref_len'] / 1000:.2f}/{row['rep_len'] / 1000:.2f}",
            f"{row['ref_len3'] / 1000:.2f}/{row['rep_len3'] / 1000:.2f}",
            f"{row['ref_speed']:.0f}/{row['rep_speed']:.0f}",
            f"{row['ref_turn']:.1f}/{row['rep_turn']:.1f}",
            f"{row['ref_stall']:.1f}/{row['rep_stall']:.1f}",
            f"{row['ref_zero']:.1f}/{row['rep_zero']:.1f}",
            f"{row['ref_off']:.0f}/{row['ref_off_max']:.0f}",
            f"{row['rep_off']:.0f}/{row['rep_off_max']:.0f}",
            f"{_number(row['ref_radius'])}/{_number(row['rep_radius'])}",
            f"{row['d_mean']:.0f} / {row['d_max']:.0f}",
        )
        print(_pad(name, widths[0])
              + "".join(_pad(text, width, right=True)
                        for text, width in zip(cells, widths[1:])))
    print()
    print("“到航线”是每条航迹到**航线折线**的最近距离（平均/最大）。两边都与同一条"
          "航线比，")
    print("  所以这一列**不受相位影响**：谁跟得住航线，看这一列。")
    print("“同弧长偏差”是把两条航迹按**累计弧长**重采样到 400 个点之后的距离"
          "（平均/最大）。")
    print("  它回答“这条线走得像不像”，**刻意不逐时刻比**——两边速率控制不同，"
          "逐时刻比出来的先是“到哪了”。")
    print("  代价是它会把**每圈被切掉的那几百米逐圈累加**成相位差，所以公里级读数"
          "要与“到航线”一起看，")
    print("  别当成“整条线都走错了”。")
    print("“平均 m/s”与“三维航迹”都按**三维**算，好与 AFSIM 的 `Speed()` 同口径"
          "（它是三维速率矢量的大小）。")
    print("  **比飞行距离必须看三维那一列**：AFSIM 的件在爬升时把水平分量让出去"
          "（flyer1 爬升段水平速率掉到 13 m/s），")
    print("  只看水平投影会把参照算短——flyer1 水平 40.3 km / 三维 61.3 km。")
    print("“停摆 %”与“零前进 %”的速率都**由位置差分得到**，两边同口径——"
          "两边的 CSV 那一列不是一回事。")
    print("  停摆 = **三维**速率 < 5 m/s（真的没在动）；零前进 = **水平**速率 < 5 m/s"
          "（水平没挪窝，但可能正在升降）。")
    print("“峰值 ω”在等时网格上差分（0.5 s）；“巡航 R”= 速率 ÷ 角速度，"
          "只取 ω ≥ 1 °/s 且速率在最大速率一半以上的帧。")


def draw(names, ref, rep, routes, stats, out: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    colors = ["#c0392b", "#d9822b", "#b7950b", "#2c6fbb", "#5b2c8d",
              "#1f7a6b", "#7f8c8d"]
    fig = plt.figure(figsize=(19.0, 5.4), dpi=110)

    ax1 = fig.add_subplot(1, 3, 1)
    for name in names:
        line = routes.get(name) or []
        if len(line) > 1:
            ax1.plot([p[0] for p in line], [p[1] for p in line],
                     "-", color="#9aa0a6", lw=0.8, alpha=0.7, zorder=1)
    ax1.plot([], [], "-", color="#9aa0a6", lw=0.8, label="航线折线")
    for index, name in enumerate(names):
        color = colors[index % len(colors)]
        if name in ref:
            ax1.plot([p[1] for p in ref[name]], [p[2] for p in ref[name]],
                     "-", color=color, lw=2.2, alpha=0.85, label=f"{name} 参照")
        if name in rep:
            ax1.plot([p[1] for p in rep[name]], [p[2] for p in rep[name]],
                     "--", color=color, lw=1.4, label=f"{name} 复现")
    # **不反转 y 轴**：``geo_to_world`` 的 +y 就是正北（探针实测：锚点正北
    # 0.1° 的点给 y = +11119.5），而 matplotlib 默认 y 向上 —— 正好"北在上"。
    # 顺手反转反而会把整张图南北颠倒。项目里其他绘图工具带着
    # `invert_yaxis()` 是同一个符号问题的另一面，见 README 的“既有缺陷”一节。
    ax1.set_aspect("equal")
    ax1.set_xlabel("东向 m（相对战区锚点）", fontsize=9)
    ax1.set_ylabel("北向 m", fontsize=9)
    ax1.set_title(f"{title}\n水平航迹：实线 = AFSIM 参照，虚线 = milsim 复现",
                  fontsize=10)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=7, ncol=2)
    ax1.tick_params(labelsize=8)

    ax2 = fig.add_subplot(1, 3, 2)
    for index, name in enumerate(names):
        color = colors[index % len(colors)]
        if name in ref:
            ax2.plot([p[0] for p in ref[name]], [p[3] for p in ref[name]],
                     "-", color=color, lw=1.6, alpha=0.85)
        if name in rep:
            ax2.plot([p[0] for p in rep[name]], [p[3] for p in rep[name]],
                     "--", color=color, lw=1.3)
    ax2.plot([], [], "-", color="#555555", lw=1.6, label="AFSIM 参照")
    ax2.plot([], [], "--", color="#555555", lw=1.3, label="milsim 复现")
    ax2.legend(fontsize=7, loc="best")
    ax2.set_xlabel("时间 s", fontsize=9)
    ax2.set_ylabel("高度 m（海拔）", fontsize=9)
    ax2.set_title("高度剖面：垂向那一层是两边差别最大的地方\n"
                  "（运动学件算垂向加速度，气动件不算）", fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.tick_params(labelsize=8)

    ax3 = fig.add_subplot(1, 3, 3)
    for index, name in enumerate(names):
        color = colors[index % len(colors)]
        if name in ref:
            grid = _on_grid(ref[name], _TURN_STRIDE_S)
            ax3.plot([g[0] for g in grid], [g[2] for g in grid],
                     "-", color=color, lw=1.6, alpha=0.85)
        if name in rep:
            grid = _on_grid(rep[name], _TURN_STRIDE_S)
            ax3.plot([g[0] for g in grid], [g[2] for g in grid],
                     "--", color=color, lw=1.3)
    ax3.plot([], [], "-", color="#555555", lw=1.6, label="AFSIM 参照")
    ax3.plot([], [], "--", color="#555555", lw=1.3, label="milsim 复现")
    ax3.set_xlabel("时间 s", fontsize=9)
    ax3.set_ylabel("三维速率 m/s", fontsize=9)
    ax3.set_title("三维速率剖面：原 demo 每段航路各带速度，\n"
                  "复现只有一个 max_speed（见 README 的能力缺口）", fontsize=10)
    ax3.grid(alpha=0.3)
    ax3.tick_params(labelsize=8)
    # 纵轴**不夹取**：复现那条会周期性砸到 0（拐角不前进，见 README 的
    # "已知差异"），那一片下探正是要看的东西，夹掉它图就好看了，也就没用了。
    ax3.legend(fontsize=7, loc="best")

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"  · 图已写入 {out}")


# ---------------------------------------------------------------------------


def main(argv: list[str]) -> int:
    options = {
        "--afsim": None, "--ref": None, "--repro": None, "--out": None,
        "--plot": None, "--seconds": 600.0, "--alt-programme": None,
        "--title": "AFSIM 机动 demo 复现",
    }
    index = 1
    while index < len(argv):
        item = argv[index]
        if item not in options or index + 1 >= len(argv):
            print(f"无法识别的参数：{item}", file=sys.stderr)
            print(__doc__)
            return 2
        options[item] = float(argv[index + 1]) if item == "--seconds" else argv[index + 1]
        index += 2

    for key in ("--afsim", "--ref", "--repro", "--out", "--plot"):
        if options[key] is None:
            print(f"缺少参数 {key}", file=sys.stderr)
            return 2

    afsim_path = Path(str(options["--afsim"]))
    print(f"[复现] AFSIM 脚本 {afsim_path.name}")
    parsed = parse_afsim(afsim_path)
    if not parsed:
        raise SystemExit(f"{afsim_path} 里没解析出任何平台")
    print(f"  · 解出 {len(parsed)} 个平台、"
          f"{sum(len(p.route) for p in parsed.values())} 个航路点")

    # 想定只装一次：跑复现的 frame 就是对照要用的 frame。再装一遍想定会得到
    # 另一套战区平面，参照与复现就会分别落在两个坐标系里——比出来的偏差是假的。
    _history, frame = run_repro(
        Path(str(options["--repro"])), parsed, Path(str(options["--out"])),
        float(options["--seconds"]), options["--alt-programme"],
    )

    ref = {k: _flat(v, frame) for k, v in load_csv(Path(str(options["--ref"]))).items()}
    rep = {k: _flat(v, frame) for k, v in load_csv(Path(str(options["--out"]))).items()}

    names = [n for n in sorted(ref) if n in rep]
    if not names:
        raise SystemExit("参照与复现没有一个平台名对得上")
    for name in sorted(set(ref) | set(rep)):
        if name not in ref:
            print(f"  · {name}：参照里没有，跳过")
        elif name not in rep:
            print(f"  · {name}：复现里没有，跳过")

    def polyline_of(name: str) -> list[tuple[float, float]]:
        route = parsed[name].route if name in parsed else []
        return [frame.geo_to_world(lat, lng) for lat, lng, _alt, _sp in route]

    stats = {
        name: compare(ref[name], rep[name], polyline_of(name)) for name in names
    }
    report(names, ref, rep, {k: v for k, v in stats.items() if v})
    draw(names, ref, rep, {name: polyline_of(name) for name in names}, stats,
         Path(str(options["--plot"])), str(options["--title"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
