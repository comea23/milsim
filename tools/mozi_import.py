"""把 Military-KG 导出的外部数据导成参数库（§5.5）。

用法::

    python tools/mozi_import.py --report          # 只统计，不写文件
    python tools/mozi_import.py --sample -o out.db --text out.txt
    python tools/mozi_import.py -o library/external.db --text library/external.txt

两个源，各归各的父类型
----------------------
.. code-block:: text

    源                              目标父类型        个数
    MoziWeaponData.json（制导武器）  MISSILE_MOVER    1336
    radarData.json                  HEX_SEARCH_RADAR  695 行 / 609 个型号

**只收能对上参数表的那几个字段**（见下面的映射表）。其余的量——毁伤概率、
圆概率误差、国别、服役年份……组件**现在没有消费者**，所以进 ``attr``
（``model_attr`` 侧表）而不是 ``model_param``：后者每一行都要能对上组件
``PARAMS`` 的一个键，塞进去只会让 ``lint`` 报一屏"未知参数"，而真正拼错的
``detec_range`` 就淹在里面了。

射程为什么不进库
----------------
库里**没有** ``range`` 这个参数——射程不是一个数，它由燃料、推力、比冲、
阻力积出来（§5.8.6）。所以射程进 ``attr``（``Range_Land_km`` 之类），
只用来反推 ``fuel_mass``（见下）。

``fuel_mass`` 的两条路（记在 ``fuel_source`` 属性里）
----------------------------------------------------
.. code-block:: text

    ① Weight − BurnoutWeight     直接量：关机重量是数据给的（**任何组都走**）
    ② 射程反推                   只在**有等速平飞段**的组里走（见下）

**① 优先。** 它不含任何假设。② 的每一步都挂着建模常数，而**默认比冲不能取
3000 s**——那是液体火箭 / 涡扇的真空口径，这批数据里绝大多数是固体火箭战术弹。
拿 51 条有直接量的弹反解等效比冲（``½ρV·cdA·R/(fuel·g)``，V 取 240 m/s）：
弹道组中位 **114 s**、巡航组中位 **293 s**。所以②**按动力形式分组**
（``_POWER_GROUPS``）：**只有"巡航"组给标定值**，无动力炸弹 / 弹道 / 反坦克 /
防空空空**一律不反推**——它们不满足"燃料全用来克服气动阻力"这个前提，
反推只会给出**看着合理的错数**。没有燃料的型号靠 ``动力`` 属性答得上"为什么没有"。

**射程取四个通道里最大的那个**（Air / Surface / Land / Subsurface），
因为"射程"对一型弹只有一个，而它打什么目标由制导件决定。

丢掉的东西与理由（``--report`` 会逐条报出计数）
----------------------------------------------
- ``Weight`` 是哨兵值 ``1`` / ``2999``  → 不写 ``mass``（2999 在制导武器里出现
  25 次，``1`` 出现在反卫星导弹上）
- ``BurnoutWeight`` 是哨兵值 ``1``，或 ``≥ Weight``（净重为负）→ 不走①
- ``instrumented range`` / ``antenna rotation`` 是区间、多值、"或"、
  "≥/≤"、或者单位不认识 → **不猜**，丢掉并计数。雷达那两列的原始文本长这样：
  ``'120 km - 250 km, and 480 km - 550 km'``、``'15, 20 and 25 min-1'``。
- 型号名里的 ASCII 双引号 ``"`` → 换成单引号 ``'``（库文本没有转义序列，
  装不下双引号；``_quote_body`` 会为此**拒绝回写**整份库）。
- ``weaponDescription``（长简介）→ 一行一个值的库文本装不下多行内容。
- 同一个雷达名出现两次（``type`` 非空 695 行、去重 609）→ 保留第一条有可用
  参数的，其余跳过并计数。

``source`` 必须如实写
---------------------
这两个源都是**公开资料汇总 + 第三方推演软件导出**，不是实测。库里的
``source`` 列存在的理由就是半年后有人问"这个 522 kg 哪来的"时能答上来
（见 ``services/library.py`` 对 ``METADATA_KEYS`` 的说明）。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from milsim.services.library import (  # noqa: E402
    KIND_COMPONENT,
    Literal,
    ModelRecord,
    ParamLibrary,
    literal_of,
)
from milsim.services.params import WithUnit  # noqa: E402

# ---------------------------------------------------------------------------
# 路径与来源
# ---------------------------------------------------------------------------

_REPO = Path(__file__).resolve().parents[1]
DEFAULT_DATA = _REPO / "Military-KG-main" / "ConstructDatabases" / "data"

WEAPON_FILE = Path("Mozi") / "MoziWeaponData.json"
RADAR_FILE = Path("radarData.json")

SOURCE_WEAPON = (
    "墨子·未来指挥官数据库（经 Military-KG 导出；第三方推演软件数据，非实测）"
)
SOURCE_RADAR = "radartutorial（经 Military-KG 导出；公开资料汇总，非实测）"

#: 想定侧的父类型。弹体是框架参考实现；雷达目前只有 ``tools/demo_components.py``
#: 里那个**演示件**（``models/`` 的感知件还没到 M4a）——它是 ``model_library.py``
#: 的 ``_COMPONENT_MODULES`` 之一，所以 ``lint`` 解析得到，但它不是参考实现。
MISSILE_PARENT = "MISSILE_MOVER"
RADAR_PARENT = "HEX_SEARCH_RADAR"

#: 弹体型号名前缀。名字必须是裸词（不含空白与 ``,:[]{}#``，且**不以数字开头**
#: ——词法层会把开头的数字读成 NUMBER）。墨子原名两样都占，所以型号名用
#: GUID 尾号，中文原名进 ``note`` 与 ``名称`` 属性。
WEAPON_PREFIX = "MOZI_W_"

#: 雷达型号名前缀。与 ``library/demo_models.txt`` 里的 ``RADAR_*`` 一致。
RADAR_PREFIX = "RADAR_"

# ---------------------------------------------------------------------------
# 反推用的建模常数（**不是数据**，``--report`` 会把它们打出来）
# ---------------------------------------------------------------------------

CD_ESTIMATE = 0.3          #: 零升阻力系数估计（弹体量级）
CRUISE_SPEED_MPS = 240.0   #: 亚音速巡航速率（m/s）
RHO0 = 1.225               #: 海平面大气密度（kg/m³）
G0 = 9.80665               #: 标准重力（m/s²）

#: 反推比冲**按动力形式分组标定**：``(组名, 关键词, 等效比冲秒数)``。
#: ``None`` = 这一组**不反推**（``fuel_mass`` 不写，见下）。
#:
#: **为什么不是 3000 s。** 3000 s 是液体火箭 / 涡扇的真空口径，而这批数据里
#: 绝大多数是固体火箭战术弹。拿 51 条有直接量（``Weight − BurnoutWeight``，逐型号
#: 实测的关机重量）的弹**反解**等效比冲（``½ρV·cdA·R/(fuel·g)``，V 取 240 m/s）：
#: 弹道组中位 **114 s**（97~560）、巡航组中位 **293 s**（164~642）——与 3000 差一个
#: 数量级，所以反推出来的燃料整体小一两个量级（鱼叉 522 kg 只得 8.8 kg，而公开
#: 推进剂是 45 kg 量级）。
#:
#: **为什么只有巡航组给值。** 这个公式算的是"**燃料全用来在速度 V 下克服气动
#: 阻力**"，前提是**长时间等速平飞**：
#:
#: - **无动力**（制导炸弹 / 滑翔弹）根本没有发动机——1336 条里这一层占了大头；
#: - **弹道**的燃料主要用来**垂直爬升克服重力**，出大气层后还没有阻力；
#: - **反坦克 / 短程**推重比远大于 1，没有等速平飞段。
#:
#: 三类都不满足前提，反推会给出**看着合理但错得离谱**的数（那正是"静默出错"），
#: 所以写成 ``None``："库里体现、代码只读可用参数"。
#:
#: **关键词是启发式的，判据是"宁可漏、不可错"**：落不进巡航组的就不反推。
_POWER_GROUPS: tuple[tuple[str, tuple[str, ...], float | None], ...] = (
    ("无动力", ("炸弹", "滑翔", "gbu", "jdam", "kab-", "paveway", "宝石路", "十字翼"), None),
    ("弹道", ("弹道", "地地", "icbm", "srbm", "mrbm", "scaleboard"), None),
    ("反坦克", ("反坦克", "atgm", "hellfire", "地狱火", "hot", "tow"), None),
    ("巡航", ("巡航", "反舰", "空舰", "空地", "反辐射", "cruise",
              "kingfish", "kitchen", "asm", "arm"), 293.0),
)

#: 落在所有组之外（防空 / 空空 / 火箭弹……）→ 不反推。理由同 ``_POWER_GROUPS``：
#: 这一族全是推重比 > 1 的固体火箭，而且**没有 burnout 样本可用来标定**。
_POWER_OTHER = "其余（防空/空空等）"


def power_group(name: str) -> tuple[str, float | None]:
    """按名字判动力形式，返回 ``(组名, 该组反推用的等效比冲或 None)``。

    ASCII 词按**词边界**匹配——``arm`` 不该命中 ``Army``，``hot`` 不该命中
    ``shot``；中文词直接找子串。
    """
    low = name.lower()
    for label, words, isp in _POWER_GROUPS:
        for word in words:
            if word.isascii():
                if re.search(rf"(?<![a-z0-9]){re.escape(word)}(?![a-z0-9])", low):
                    return label, isp
            elif word in low:
                return label, isp
    return _POWER_OTHER, None

#: ``mass`` 的哨兵值。2999 在制导武器里出现 25 次（含一枚 10 t 级的反导拦截弹
#: 与一枚 16 m 长的弹道弹），1 出现在反卫星导弹上——都不是真实重量。
MASS_SENTINELS = frozenset({1, 2999})
#: ``BurnoutWeight`` 的哨兵值。``1`` 出现在 12 行上（对应的 ``Weight`` 从 1 到
#: 18000 都有），显然不是"关机时还剩 1 kg"。
BURNOUT_SENTINEL = 1

#: 射程/转速的换算。``NM`` 与 ``nm`` 在这份表里都是海里（雷达规格书的写法）。
_LENGTH_FACTOR = {
    "km": 1.0,
    "nm": 1.852,
    "nmi": 1.852,
    "mi": 1.609344,
}

#: 数值的形状。**不能用 ``[\d.]+``**：它会把 ``'0...5 U/min'`` 里的 ``0...5``
#: 当成一个数（实测踩过，症状是 ``float()`` 报 ValueError）。小数点只能有一个，
#: 且两边都要有数字。
_NUM = r"\d+(?:\.\d+)?"

_KM_PAREN = re.compile(rf"[^(]*\(\s*≙\s*({_NUM})\s*km\s*\)\s*$")
_LENGTH = re.compile(
    rf"({_NUM})\s*(km|NM|nm|nmi|mi|nautical\s+miles?)\s*\.?$", re.IGNORECASE
)
_ROTATION = re.compile(
    rf"({_NUM})\s*(?:rpm|min-1|u/min|rev\.?\s*/\s*min|rev/min)\s*\.?$",
    re.IGNORECASE,
)

#: 组件实现类靠 import 触发注册。没有它们，``validate`` 会把**每一个**型号
#: 都报成"找不到对应的组件实现"——"校验通过"与"其实没校验"是两回事，
#: 所以导入失败要说出来（同 ``tools/model_library.py``）。
_COMPONENT_MODULES = ("milsim.models", "demo_components")


def load_components() -> list[str]:
    """导入组件模块，返回导入失败的模块名。"""
    failed: list[str] = []
    for name in _COMPONENT_MODULES:
        try:
            __import__(name)
        except ImportError:
            failed.append(name)
    return failed


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------


class Report:
    """逐条记录"多少个、因为什么被丢"。**不静默丢东西**是本文件的规矩。"""

    def __init__(self) -> None:
        self.rows = 0
        self.models = 0
        self.dropped: Counter[str] = Counter()
        self.notes: list[str] = []
        self.fuel_by_burnout = 0
        self.fuel_by_range = 0
        self.fuel_missing = 0
        self.quotes_replaced = 0
        #: 每个动力形式各多少个型号（含"不反推"的那些，见 ``_POWER_GROUPS``）。
        self.power_groups: Counter[str] = Counter()

    def drop(self, reason: str) -> None:
        self.dropped[reason] += 1

    def lines(self) -> list[str]:
        out = [f"  读数 {self.rows} 行 → 型号 {self.models} 个"]
        for reason, count in sorted(self.dropped.items(), key=lambda kv: -kv[1]):
            out.append(f"     丢掉 {count}：{reason}")
        if self.quotes_replaced:
            out.append(
                f"     型号名里的 ASCII 双引号换成单引号 {self.quotes_replaced} 条"
                "（库文本没有转义序列，装不下双引号）"
            )
        return out


def _power_summary(report: Report, isp: float | None) -> list[str]:
    """每个动力形式一行：``组名  型号数  条  反推比冲 / 不反推``。

    ``isp`` 是 ``--isp`` 的显式覆盖值；给了就所有可反推的组都用它（敏感性对照），
    没给就用 ``_POWER_GROUPS`` 里那一组标定的值。
    """
    out: list[str] = []
    entries = [(label, calibrated) for label, _, calibrated in _POWER_GROUPS]
    entries.append((_POWER_OTHER, None))
    for label, calibrated in entries:
        count = report.power_groups.get(label, 0)
        if not count:
            continue
        value = isp if isp is not None else calibrated
        tail = f"{value:g} s" if value is not None else "不反推"
        out.append(f"{label:<10}{count:>5} 条   {tail}")
    return out


# ---------------------------------------------------------------------------
# 读源
# ---------------------------------------------------------------------------


def read_lines(path: Path) -> list[dict[str, Any]]:
    """读 **JSON Lines**。这两个文件是"一行一个对象"，不是 JSON 数组
    ——``json.loads`` 整份读会报 ``Extra data: line 2 column 1``。"""
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def number(value: Any) -> float | None:
    """数值字段 → float。布尔、字符串、空值一律当"没有"。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def usable(value: Any, sentinels: Iterable[float] = ()) -> float | None:
    """取一个可用的正数：非零、非哨兵、非负。"""
    got = number(value)
    if got is None or got <= 0.0:
        return None
    if any(got == s for s in sentinels):
        return None
    return got


def safe_text(text: str) -> tuple[str, bool]:
    """库文本装得下的文本：ASCII 双引号换单引号、换行换空格。

    ``_quote_body`` 会为这两种字符**拒绝回写整份库**，所以不能留到那一刻
    才发现——但也不能悄悄换。返回"换过没有"，由 :class:`Report` 计数。
    """
    changed = '"' in text or "\n" in text or "\r" in text
    fixed = text.replace('"', "'").replace("\r", " ").replace("\n", " ")
    return " ".join(fixed.split()), changed


def attr_of(value: Any) -> Literal | None:
    """属性值 → 字面量。**不查参数表**（那是 ``attr`` 的定义）。

    **要舍入**：源里的数是浮点运算的残渣——射程 ``1349.9999999999998``
    （是 ``0.5399568034557235`` 的反面，海里换算留下的）。原样写进库文本会
    让人以为那是精度，其实是噪声。接近整数就取整，否则留 4 位小数。
    """
    got = number(value)
    if got is None:
        return None
    if abs(got - round(got)) < 1e-6:
        return literal_of(int(round(got)))
    return literal_of(round(got, 4))


# ---------------------------------------------------------------------------
# 弹：MoziWeaponData.json
# ---------------------------------------------------------------------------

#: 源字段 → 库属性名。**列出来的都是组件没有消费者的量**，所以走 ``attr``；
#: 真正进 ``param`` 的只有 ``mass`` / ``fuel_mass`` / ``drag_area`` 三个
#: （见 :func:`missile_record`）。
_WEAPON_ATTRS: tuple[tuple[str, str], ...] = (
    ("Length_m", "Length"),
    ("Span_m", "Span"),
    ("Diameter_m", "Diameter"),
    ("Weight_kg", "Weight"),
    ("BurnoutWeight_kg", "BurnoutWeight"),
    ("CEP_m", "CEP"),
    ("CEPSurface_m", "CEPSurface"),
    ("PoK_Air", "AirPoK"),
    ("PoK_Surface", "SurfacePoK"),
    ("PoK_Land", "LandPoK"),
    ("PoK_Subsurface", "SubsurfacePoK"),
    ("Range_Air_km", "AirRangeMax"),
    ("Range_Surface_km", "SurfaceRangeMax"),
    ("Range_Land_km", "LandRangeMax"),
    ("Range_Subsurface_km", "SubsurfaceRangeMax"),
    ("LaunchSpeedMax_mps", "LaunchSpeedMax"),
    ("TargetSpeedMax_mps", "TargetSpeedMax"),
    ("MaxFlightTime_s", "MaxFlightTime"),
    ("WaypointNumber", "WaypointNumber"),
    ("CruiseAltitude_m", "CruiseAltitude"),
    ("服役起年", "YearStart"),
    ("服役止年", "YearEnd"),
    ("世代", "Generation"),
)

_RANGE_FIELDS = (
    "AirRangeMax",
    "SurfaceRangeMax",
    "LandRangeMax",
    "SubsurfaceRangeMax",
)


def weapon_model_name(row: dict[str, Any]) -> str | None:
    """型号名：``MOZI_W_`` + 12 位 ``weaponID``。

    不能直接用墨子原名：``'芦洞-1型弹道导弹 [常规]'`` 含空白与方括号，
    ``AGM-84G型"鱼叉"反舰导弹`` 含双引号——三样都会破坏库文本。
    """
    weapon_id = row.get("weaponID")
    if not isinstance(weapon_id, int) or weapon_id <= 0:
        return None
    return f"{WEAPON_PREFIX}{weapon_id:012d}"


def max_range_km(row: dict[str, Any]) -> float | None:
    """四个通道里最大的那个射程。**"射程"对一型弹只有一个**——打什么目标
    由制导件决定，不由弹体决定。"""
    values = [usable(row.get(f)) for f in _RANGE_FIELDS]
    live = [v for v in values if v is not None]
    return max(live) if live else None


def fuel_by_range(
    mass_kg: float, diameter_m: float, range_km: float, *, cd: float,
    speed: float, isp: float,
) -> tuple[float, float] | None:
    """射程反推燃料：返回 ``(燃料 kg, Cd·A m²)``。

    稳态巡航（推力 = 阻力）下：::

        Cd·A   = cd · π · (D/2)²                  迎风圆面积 × 估计的 Cd
        D_drag = ½·ρ0·V²·(Cd·A)                   海平面动压 × Cd·A
        t      = R / V                            飞完全程要多久
        fuel   = D_drag · t / (Isp·g)             烧掉的质量

    **每一步都挂着常数**，所以这条路只在没有关机重量时才走，且结果要连同
    假设一起看（见模块头）。
    """
    if diameter_m <= 0.0 or range_km <= 0.0 or speed <= 0.0 or isp <= 0.0:
        return None
    cd_area = cd * 3.141592653589793 * (diameter_m / 2.0) ** 2
    drag = 0.5 * RHO0 * speed * speed * cd_area
    fuel = drag * (range_km * 1000.0 / speed) / (isp * G0)
    if not (0.0 < fuel < mass_kg):
        return None
    return fuel, cd_area


def missile_record(
    row: dict[str, Any], report: Report, *,
    cd: float, speed: float, isp: float | None,
) -> ModelRecord | None:
    """一条制导武器 → 一个弹体型号。返回 ``None`` 表示这条不要。"""
    name = weapon_model_name(row)
    if name is None:
        report.drop("没有 weaponID")
        return None

    original = str(row.get("Name") or "")
    power, group_isp = power_group(original)
    report.power_groups[power] += 1
    #: 显式给了 ``--isp`` 就全用它（做敏感性对照的逃生口）；否则用该组的标定值。
    isp_for_derivation = isp if isp is not None else group_isp

    mass = usable(row.get("Weight"), MASS_SENTINELS)
    if mass is None:
        report.drop("Weight 缺 / 是哨兵值（1、2999）")
        return None

    params: dict[str, Literal] = {"mass": literal_of(WithUnit(mass, "kg"))}

    attrs: dict[str, Literal] = {}
    for key, field in _WEAPON_ATTRS:
        literal = attr_of(row.get(field))
        if literal is not None:
            attrs[key] = literal
    for key, field in (("名称", "Name"), ("类别", "typeComment"),
                       ("国别", "countyrComment"), ("GUID", "GUID")):
        raw = row.get(field)
        if isinstance(raw, str) and raw.strip():
            text, changed = safe_text(raw)
            report.quotes_replaced += 1 if changed else 0
            attrs[key] = literal_of(text)
    #: 动力形式（``_POWER_GROUPS`` 的组名）。**每条都写**——没有 ``fuel_mass`` 的
    #: 型号，靠这一列才答得上"为什么它没有燃料"。
    attrs["动力"] = literal_of(power)

    # ---- 燃料：① 关机重量（直接量）----
    burnout = number(row.get("BurnoutWeight"))
    diameter = usable(row.get("Diameter"))
    fuel: float | None = None
    if (
        burnout is not None
        and burnout != BURNOUT_SENTINEL
        and 0.0 < mass - burnout < mass
    ):
        fuel = mass - burnout
        attrs["fuel_source"] = literal_of("burnout")
        report.fuel_by_burnout += 1
    elif burnout is not None and burnout >= mass:
        report.drop("BurnoutWeight ≥ Weight（净重为负）")

    # ---- 阻力面积：反推要用它，写进 drag_area 让它有用；没直径就不写 ----
    cd_area: float | None = None
    span = max_range_km(row)
    if diameter is not None:
        cd_area = cd * 3.141592653589793 * (diameter / 2.0) ** 2

    # ---- 燃料：② 射程反推（挂常数，见模块头）----
    if fuel is None:
        if isp_for_derivation is None:
            # 前提不成立（见 ``_POWER_GROUPS``）⇒ 宁可没有，不给一个看着合理的错数。
            report.drop(f"「{power}」没有等速平飞段，不走射程反推")
        elif span is None:
            report.drop("没有可用射程（四个通道全空）")
        elif diameter is None:
            report.drop("没有 Diameter，反推不了")
        else:
            derived = fuel_by_range(
                mass, diameter, span, cd=cd, speed=speed, isp=isp_for_derivation
            )
            if derived is None:
                report.drop("反推结果不合理（≥ 总质量）")
            else:
                fuel, _ = derived
                attrs["fuel_source"] = literal_of("range")
                attrs["反推_射程km"] = literal_of(round(span, 3))
                attrs["反推_CdA_m2"] = literal_of(round(cd_area or 0.0, 5))
                #: 反推用的等效比冲（秒）。**必须记**——换一组常数射程就变一倍，
                #: 半年后要能从库里答出"这个燃料是拿什么算的"。
                attrs["反推_Isp_s"] = literal_of(round(isp_for_derivation, 1))
                report.fuel_by_range += 1

    if fuel is not None:
        params["fuel_mass"] = literal_of(WithUnit(round(fuel, 1), "kg"))
    else:
        report.fuel_missing += 1

    if cd_area is not None:
        params["drag_area"] = literal_of(WithUnit(round(cd_area, 5), "m2"))

    note, changed = safe_text(original or name)
    report.quotes_replaced += 1 if changed else 0
    return ModelRecord(
        kind=KIND_COMPONENT,
        name=name,
        params=params,
        parent=MISSILE_PARENT,
        note=note,
        source=SOURCE_WEAPON,
        attrs=attrs,
    )


def missiles(
    data_dir: Path, report: Report, *,
    sample: bool, cd: float, speed: float, isp: float | None,
) -> list[ModelRecord]:
    rows = read_lines(data_dir / WEAPON_FILE)
    report.rows = len(rows)
    records: list[ModelRecord] = []
    for row in rows:
        if row.get("typeComment") != "制导武器":
            report.drop("不是制导武器（火炮/鱼雷/炸弹/油桶……）")
            continue
        if sample and not any(k in str(row.get("Name") or "")
                              for k in SAMPLE_MISSILES):
            report.drop("不在小样本名单里（鱼叉/战斧/飞毛腿）")
            continue
        record = missile_record(row, report, cd=cd, speed=speed, isp=isp)
        if record is not None:
            records.append(record)
    report.models = len(records)
    return records


#: 小样本先做这三型：一个有公开推进剂数据的反舰弹、一个巡航弹、一个弹道弹。
SAMPLE_MISSILES = ("鱼叉", "战斧", "飞毛腿")


# ---------------------------------------------------------------------------
# 雷达：radarData.json
# ---------------------------------------------------------------------------


def parse_range_km(text: str) -> float | None:
    """``instrumented range`` → 千米。不认识就返回 ``None``（**不猜**）。

    只认三种写法：括号里的换算结果、单个"数值 + 长度单位"、以及那两种后面
    拖一个多余的逗号。区间、多值、"或"、"≥/≤"一律返回 ``None``。
    """
    body = text.strip().rstrip(",").strip()
    if not body:
        return None
    found = _KM_PAREN.fullmatch(body)
    if found is not None:
        return float(found.group(1))
    found = _LENGTH.fullmatch(body)
    if found is None:
        return None
    unit = re.sub(r"\s+", " ", found.group(2).lower())
    if unit.startswith("nautical"):
        unit = "nm"
    return float(found.group(1)) * _LENGTH_FACTOR[unit]


def parse_rotation_s(text: str) -> float | None:
    """``antenna rotation`` → 扫描周期（秒）。不认识就返回 ``None``。

    "15, 20 and 25 min-1" 这种**算不出一个周期**——它是一组可选的转速，
    取哪个要看工作模式，而数据没说。丢掉并计数，不取平均、不取第一个。
    """
    body = text.strip().rstrip(",.").strip()
    if not body:
        return None
    found = _ROTATION.fullmatch(body)
    if found is None:
        return None
    rpm = float(found.group(1))
    if rpm <= 0.0:
        return None
    return 60.0 / rpm


def radar_model_name(type_name: str, index: int) -> str:
    """雷达型号名：``RADAR_`` + 原名做成的裸词。

    名字必须能以 NAME token 出现：不含空白与 ``,:[]{}#``、**不以数字开头**
    （``1S91 (Kub)`` 原名以数字开头，词法层会把它读成 NUMBER）。所以先做
    替换，再补前缀。
    """
    slug = type_name.replace("/", "_")
    slug = re.sub(r"[^\w.\-]", "_", slug, flags=re.UNICODE)
    slug = re.sub(r"_{2,}", "_", slug).strip("_.")
    name = f"{RADAR_PREFIX}{slug}" if slug else f"{RADAR_PREFIX}{index:04d}"
    return name


def radar_records(
    data_dir: Path, report: Report, *, sample: bool
) -> list[ModelRecord]:
    rows = read_lines(data_dir / RADAR_FILE)
    report.rows = len(rows)
    records: list[ModelRecord] = []
    seen: dict[str, int] = {}
    for index, row in enumerate(rows):
        raw_name = str(row.get("type") or "").strip()
        if not raw_name:
            report.drop("type 列是空的")
            continue
        if sample and raw_name not in SAMPLE_RADARS:
            report.drop("不在小样本名单里（HR-76/W-160/JY-14）")
            continue
        name = radar_model_name(raw_name, index)
        if name in seen:
            report.drop(f"型号名重复（{raw_name!r}）")
            continue

        params: dict[str, Literal] = {}
        raw_range = str(row.get("instrumented range:") or "")
        range_km = parse_range_km(raw_range)
        if range_km is not None and range_km > 0.0:
            params["detect_range"] = literal_of(WithUnit(range_km, "km"))
        elif not raw_range.strip():
            report.drop("instrumented range 源里是空的")
        else:
            report.drop("instrumented range 认不出（区间/多值/单位不认识）")

        raw_rotation = str(row.get("antenna rotation:") or "")
        period_s = parse_rotation_s(raw_rotation)
        if period_s is not None:
            params["scan_interval"] = literal_of(WithUnit(round(period_s, 3), "s"))
        elif not raw_rotation.strip():
            report.drop("antenna rotation 源里是空的")
        else:
            report.drop("antenna rotation 认不出（多值/是转速区间/单位不认识）")

        if not params:
            # 两列都没取到，这个型号在库里没有任何参数位——留着只会让 lint
            # 报一句"它没有参数"，而它本来就没有。
            report.drop("两列都没取到，整个型号不要")
            continue

        seen[name] = index
        attrs: dict[str, Literal] = {"名称": literal_of(safe_text(raw_name)[0])}
        for key, field in (
            ("instrumented_range_raw", "instrumented range:"),
            ("antenna_rotation_raw", "antenna rotation:"),
            ("frequency", "frequency:"),
            ("beamwidth_raw", "beamwidth:"),
            ("range_resolution_raw", "range resolution:"),
            ("peak_power_raw", "peak power:"),
            ("average_power_raw", "average power:"),
            ("prf_raw", "pulse repetition frequency (PRF):"),
            ("pulsewidth_raw", "pulsewidth (τ):"),
            ("accuracy_raw", "accuracy:"),
            ("hits_per_scan_raw", "hits per scan:"),
            ("dead_time_raw", "dead time:"),
            ("receive_time_raw", "receive time:"),
            ("MTBCF_raw", "MTBCF:"),
            ("MTTR_raw", "MTTR:"),
        ):
            raw = row.get(field)
            if isinstance(raw, str) and raw.strip():
                attrs[key] = literal_of(safe_text(raw)[0])

        records.append(
            ModelRecord(
                kind=KIND_COMPONENT,
                name=name,
                params=params,
                parent=RADAR_PARENT,
                note=raw_name,
                source=SOURCE_RADAR,
                attrs=attrs,
            )
        )
    report.models = len(records)
    return records


#: 小样本先做这三个雷达：源里既有探测距离又有转速、且格式干净。
SAMPLE_RADARS = ("HR-76", "W-160", "JY-14")

#: 基型声明块，各排在自己那一节的最前面。
#:
#: **必须有，且必须在前**：``:`` 后面只能写**已声明的类型**，而
#: ``@register_component`` 注册的那个类是"实现"、不是"类型"——库里要先写一个
#: 同名空块把它引进类型表（写法同 ``library/demo_models.txt``）。少了这两块，
#: ``lint`` 会给**每一个**型号报"继承自未定义的组件类型"，1699 个型号就是 1699 条。
_BASE_MISSILE = ModelRecord(
    kind=KIND_COMPONENT,
    name=MISSILE_PARENT,
    note="弹体的基型声明（对接框架参考实现；不写参数，只给派生用）",
)
_BASE_RADAR = ModelRecord(
    kind=KIND_COMPONENT,
    name=RADAR_PARENT,
    note="六边形搜索雷达的基型声明（演示件；不写参数，只给派生用）",
)


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------


def build_library(
    data_dir: Path, *, sample: bool, cd: float, speed: float, isp: float | None
) -> tuple[ParamLibrary, list[str]]:
    weapon_report = Report()
    missile_records = missiles(
        data_dir, weapon_report, sample=sample, cd=cd, speed=speed, isp=isp
    )
    radar_report = Report()
    radar_records_ = radar_records(data_dir, radar_report, sample=sample)

    log = [
        f"弹（{WEAPON_FILE}）—— 源：{SOURCE_WEAPON}",
        f"  基型声明 {_BASE_MISSILE.name} + 型号 {len(missile_records)} 个",
        *weapon_report.lines(),
        f"  燃料来源：关机重量 {weapon_report.fuel_by_burnout} 条 / "
        f"射程反推 {weapon_report.fuel_by_range} 条 / 都没有 "
        f"{weapon_report.fuel_missing} 条",
        "  动力形式（括号内是该组反推用的等效比冲，'不反推'= 前提不成立）：",
        *[f"     {line}" for line in _power_summary(weapon_report, isp)],
        "",
        f"雷达（{RADAR_FILE}）—— 源：{SOURCE_RADAR}",
        f"  基型声明 {_BASE_RADAR.name} + 型号 {len(radar_records_)} 个",
        *radar_report.lines(),
        "",
        f"反推常数：Cd={cd:g} 巡航速率={speed:g} m/s "
        f"海平面密度={RHO0:g} g={G0:g}",
    ]

    origin = f"{WEAPON_FILE} + {RADAR_FILE}（mozi_import.py）"
    library = ParamLibrary(
        [_BASE_MISSILE, *missile_records, _BASE_RADAR, *radar_records_],
        origin=origin,
    )
    return library, log


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="把 Military-KG 的外部数据导成参数库", add_help=True
    )
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA,
                        help=f"数据目录（默认 {DEFAULT_DATA}）")
    parser.add_argument("-o", "--out", type=Path, help="目标库文件（.db）")
    parser.add_argument("--text", type=Path, help="同时 dump 一份文本（进版本控制）")
    parser.add_argument("--sample", action="store_true",
                        help="只做小样本（鱼叉/战斧/飞毛腿 + HR-76/W-160/JY-14）")
    parser.add_argument("--report", action="store_true", help="只统计，不写文件")
    parser.add_argument("--cd", type=float, default=CD_ESTIMATE)
    parser.add_argument("--cruise-speed", type=float, default=CRUISE_SPEED_MPS)
    parser.add_argument("--isp", type=float, default=None,
                        help="覆盖分组标定的等效比冲（秒）；不给就用各组标定值")
    args = parser.parse_args(argv)

    if not args.data.is_dir():
        print(f"数据目录不存在：{args.data}", file=sys.stderr)
        return 1

    library, log = build_library(
        args.data, sample=args.sample, cd=args.cd,
        speed=args.cruise_speed, isp=args.isp,
    )
    for line in log:
        print(line)

    print()
    print(library.describe())

    if args.report:
        return 0

    if args.out is None:
        print("要写文件就得给 -o <目标.db>（或 --report 只看统计）", file=sys.stderr)
        return 2

    library.write_sqlite(args.out, overwrite=True)
    print(f"已写入 {args.out}")
    if args.text is not None:
        args.text.write_text(library.to_text(), encoding="utf-8")
        print(f"已 dump 到 {args.text}（{len(library.to_text().splitlines())} 行）")

    remaining = load_components()
    if remaining:
        print(
            f"警告：{'、'.join(remaining)} 没能导入——下面的结果**没有**校验"
            "这些模块里的组件",
            file=sys.stderr,
        )
    problems = library.validate()
    if problems:
        print(f"全量校验发现 {len(problems)} 个问题：", file=sys.stderr)
        for item in problems[:20]:
            print(f"  - {item}", file=sys.stderr)
        return 1
    print("全量校验通过")
    return 0

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
