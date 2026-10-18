"""机动组件 × AFSIM 批量对拍自动化。

把项目**全部实现的机动组件**（空中/地面/水面/水下四个平台件 + 导弹弹体
+ 巡航途径点航线）与 AFSIM 同构件在大步长（1 s 采样）条件下批量对照：
参数网格自动展开、两侧想定自动生成、AFSIM mission.exe 自动跑、轨迹自动
对齐并按 ``tools/cmp_metrics`` 的口径出指标，最后落一份汇总表 + 报告 +
最差配置的轨迹叠图。

用法::

    python tools/cmp_movers.py                # 全部族全部配置
    python tools/cmp_movers.py --quick        # 每族抽 2 个配置
    python tools/cmp_movers.py --probe        # 每族抽 1 个配置（探管线）
    python tools/cmp_movers.py --only air_turn,asm
    python tools/cmp_movers.py --skip-asm     # 弹族要 AFSIM 制导链，慢

需要 ``AFSIM_MISSION`` 指向 mission.exe（或默认 F:/afsim2.9cn/bin）。

族 → 两侧同构件的映射（映射依据见 demos/afsim_mover/kinematic_mover.txt
头注的参数映射表，全部是已验证过的对应关系）：

=================  ========================  ==========================
族                 milsim                    AFSIM
=================  ========================  ==========================
air_straight       AIR_MOVER 直线            WSF_KINEMATIC_MOVER
air_turn           AIR_MOVER 转弯            WSF_KINEMATIC_MOVER
air_route          AIR_MOVER 多路点          WSF_KINEMATIC_MOVER
air_vertical       AIR_MOVER 爬升/下降       WSF_AIR_MOVER（爬升率口径）
ground_straight    GROUND_MOVER              WSF_KINEMATIC_MOVER(land)
asm                MISSILE_MOVER+ASM_GUIDANCE  WSF_GUIDED_MOVER 链
=================  ========================  ==========================

转弯/直线/途径点族的 AFSIM 侧用 KINEMATIC_MOVER（带 maximum_body_turn_rate
/ maximum_radial_acceleration，与本项目 ω=max(turn_rate, radial_accel/v)
同一套限制口径）；垂向族用 AIR_MOVER（maximum_climb_rate ↔ climb_rate，
两边都是"垂向按速率"的口径）。

**未纳入的两族及理由（诚实口径，不是漏做）**：WATER_MOVER / SUBSURFACE_MOVER
依赖画像与水深门槛（PROFILE_WATER / 吃水），本项目要"有水的格子"才动
（flat 地形上 A* 无路，不报错只是不动——实测踩过）；AFSIM 侧没有同构的
"水面地图 + 吃水"概念，procedural 地形又会引入 A* 绕行差异。两件的推进
与地面件共用同一套基类（直线层已被 ground 覆盖到米级），硬对是自欺。

**已知差异**（报告里随行注出，不是本工具能修的）：

1. 转弯/途径点族：AFSIM 在路点前**提前转弯**（转弯半径预判、切内线），
   本项目在路点处**原地换向**（点转弯）——拐角附近的横向偏差是设计差异，
   不是参数错；报告按占比如实呈现。
2. 弹族横向：本项目弹没有预规划航路跟随（§9-31），AFSIM 参照弹先走航路
   再转向——纵向可比、横向结构性偏大（README §11、§9-36）。
3. 弹族阻力：本项目常数 drag_area，AFSIM 气动表跨音速抬升——只取亚音速档。
4. AFSIM 采样是自己的节奏（update_interval 0.1 s），本项目 1 s 步长——
   等长指标走 ``align`` 插值对齐（不外推），弗雷歇直接吃原始点列。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import afsim_ref  # noqa: E402
from afsim_ref import (  # noqa: E402
    _LAT_ANCHOR,
    _LNG_ANCHOR,
    _mission_exe,
    _observer_block,
    _run_once,
)

from tools.cmp_metrics import (  # noqa: E402
    align_pairs,
    frechet_distance,
    rmse,
    tic,
)

from milsim.services.map.hex import LocalFrame, offset_to_heading  # noqa: E402

#: 大步长口径：两边都换算到 1 s 等间隔栅格再算等长指标。
DT_S = 1.0
#: 弗雷歇吃的原始点列上限（O(nm) 动态规划，防长轨迹爆内存）。
FRECHET_MAX_PTS = 150
#: 判读线：水平 RMSE 与航程之比。超过只标"看一眼"，不是硬门槛。
PASS_RATIO = 0.02
SEED = 20261017
LAT_ANCHOR = 34.0      # 平台件族的战区锚点（佐治亚，与既有 demo 同一片平地）
LNG_ANCHOR = -83.17
ASM_ANCHOR = (39.05, -89.80)   # 弹族锚点（new_guidance 那片，terrain flat）

_OUT = ROOT / "demos" / "afsim_mover" / "cmp_movers"


# ---------------------------------------------------------------------------
# 几何小工具
# ---------------------------------------------------------------------------


def _fmt_latlng(lat: float, lng: float) -> str:
    """AFSIM 位置写法：``38.60000n 90.30000w``（十进制 + 半球字母）。"""
    ns = "n" if lat >= 0 else "s"
    ew = "e" if lng >= 0 else "w"
    return f"{abs(lat):.5f}{ns} {abs(lng):.5f}{ew}"


def _bearing_to_offset(brg_deg: float, dist: float) -> tuple[float, float]:
    """罗盘方位角（0=北顺时针）→ 平面偏移 (东, 北)。"""
    r = math.radians(brg_deg)
    return dist * math.sin(r), dist * math.cos(r)


def _geo_brg(dx: float, dy: float) -> float:
    """平面偏移 (东, 北) → 罗盘方位角。"""
    return math.degrees(math.atan2(dx, dy)) % 360.0


# ---------------------------------------------------------------------------
# 配置网格
# ---------------------------------------------------------------------------


def _base_air(**over) -> dict:
    p = {
        "max_speed": "250 m/s", "turn_rate": "120 deg/s",
        "radial_accel": "100 m/s2", "linear_accel": "20 m/s2",
        "angular_accel": "80 deg/s2", "vertical_accel": "100 m/s2",
        "climb_rate": "200 m/s", "descent_rate": "200 m/s",
        "altitude_tolerance": "1 m", "period": "0.1 s",
    }
    p.update(over)
    return p


def _kin_afsim(v: float, tr: float, ra: float, la: float = 20.0) -> str:
    """KINEMATIC_MOVER 的参数块——**只能写在平台实例的 mover 块里**
    （kinematic demo 的 flyer1 就是这么放的；写进 platform_type 报
    "Could not find mover initial_speed"，实测踩过）。"""
    return f"""initial_speed {v:g} m/s
      target_speed {v:g} m/s
      maximum_linear_acceleration {la:g} m/s2
      maximum_radial_acceleration {ra:g} m/s2
      maximum_body_turn_rate {tr:g} deg/sec
      maximum_body_roll_rate 500 deg/sec
      proportional_navigation_gain 10.0
      velocity_pursuit_gain 30.0
      update_interval 0.1 sec"""


def _air_afsim(v: float, climb: float) -> str:
    return f"""update_interval 0.1 sec
      update_time_tolerance 0.01 sec
      altitude_offset 0 ft
      at_end_of_path extrapolate
      on_turn_failure best_effort
      turn_failure_threshold 0.01
      maximum_speed {v:g} m/s
      minimum_speed 1 m/s
      maximum_altitude 50000 m
      minimum_altitude 0 m
      maximum_climb_rate {climb:g} m/s
      maximum_flight_path_angle 45 deg
      maximum_linear_acceleration 20 m/s2
      maximum_radial_acceleration 100 m/s2
      bank_angle_limit 60 deg
      body_g_limit 10 g
      roll_rate_limit 60 deg/sec
      heading_pursuit_gain 5
      path_variance_radius 0 ft
      speed_variance_percent 0
      turn_rate_limit 21600 deg/min
      path_compute_timestep 0.5 sec"""


def _cfg(family, cid, *, duration, start, wps, milsim_params, afsim_mover,
         domain, anchor=(LAT_ANCHOR, LNG_ANCHOR), mover_component="AIR_MOVER",
         zone_radius_km=60, guidance=None, trim_s=None):
    return {
        "family": family, "cid": cid, "duration": duration,
        "start": start, "wps": wps,
        "milsim_params": milsim_params,
        "afsim_mover": afsim_mover,
        "domain": domain, "anchor": anchor,
        "mover_component": mover_component,
        "zone_radius_km": zone_radius_km, "guidance": guidance,
        # 名义飞行时刻（航程/速度）：对比截断到它——AFSIM 的
        # at_end_of_path extrapolate 会在走完后继续直飞，尾巴不截
        # 会把"到达后停住 vs 继续飞"算成巨大偏差（垂直族实测 8.7 km）。
        "trim_s": trim_s,
    }


def _straight_cfgs() -> list[dict]:
    out = []
    for v in (60.0, 150.0, 250.0):
        for brg in (30.0, 90.0, 210.0):
            for alt in (1000.0, 8000.0):
                dist = max(5000.0, v * 45.0)
                seed = offset_to_heading(*_bearing_to_offset(brg, dist))
                cid = f"v{int(v)}_brg{int(brg)}_a{int(alt)}"
                out.append(_cfg(
                    "air_straight", cid,
                    duration=dist / v + 15.0,
                    trim_s=dist / v,
                    start={"brg": brg, "dist": dist, "alt": alt, "v": v,
                           "seed": seed},
                    wps=[], milsim_params=_base_air(max_speed=f"{v:g} m/s"),
                    afsim_mover=("kinematic", _kin_afsim(v, 120.0, 100.0)),
                    domain="air",
                ))
    return out


def _turn_cfgs() -> list[dict]:
    out = []
    for tr, g in ((15.0, 1.5), (45.0, 2.2), (90.0, 4.0)):
        for v in (80.0, 200.0):
            ra = g * 9.80665
            cid = f"tr{int(tr)}_g{str(g).replace('.', 'p')}_v{int(v)}"
            leg = 8000.0
            out.append(_cfg(
                "air_turn", cid,
                duration=2 * leg / v + 100.0,
                # 转弯弧比直线长一点：留 1.15 倍的名义时刻
                trim_s=2 * leg / v * 1.15,
                start={"brg": 0.0, "dist": leg, "alt": 3000.0, "v": v,
                       "seed": offset_to_heading(*_bearing_to_offset(0.0, leg))},
                wps=[{"brg": 90.0, "dist": leg, "alt": 3000.0, "v": v}],
                milsim_params=_base_air(
                    max_speed=f"{v:g} m/s", turn_rate=f"{tr:g} deg/s",
                    radial_accel=f"{ra:.2f} m/s2"),
                afsim_mover=("kinematic", _kin_afsim(v, tr, ra)),
                domain="air",
            ))
    return out


def _vertical_cfgs() -> list[dict]:
    out = []
    for rate, dalt in ((8.0, 3000.0), (25.0, 5000.0), (60.0, -4000.0),
                       (15.0, -2000.0)):
        for v in (100.0, 200.0):
            cid = f"r{int(rate)}_d{int(dalt):+d}_v{int(v)}"
            out.append(_cfg(
                "air_vertical", cid,
                duration=30000.0 / v + abs(dalt) / rate + 40.0,
                trim_s=30000.0 / v,
                start={"brg": 0.0, "dist": 15000.0, "alt": 2000.0, "v": v,
                       "seed": offset_to_heading(0.0, 15000.0)},
                # 两腿：第一腿爬/降到目标高度，第二腿平飞
                wps=[{"brg": 0.0, "dist": 15000.0, "alt": 2000.0 + dalt,
                      "v": v},
                     {"brg": 0.0, "dist": 15000.0, "alt": 2000.0 + dalt,
                      "v": v}],
                milsim_params=_base_air(
                    max_speed=f"{v:g} m/s", turn_rate="120 deg/s",
                    radial_accel="100 m/s2", linear_accel="20 m/s2",
                    vertical_accel="50 m/s2",
                    climb_rate=f"{rate:g} m/s",
                    descent_rate=f"{rate:g} m/s"),
                afsim_mover=("air", _air_afsim(v, rate)),
                domain="air",
            ))
    return out


def _route_cfgs() -> list[dict]:
    shapes = {
        "L3": [(0.0, 8000.0), (6000.0, 12000.0), (12000.0, 4000.0)],
        "S5": [(0.0, 8000.0), (6000.0, 12000.0), (12000.0, 4000.0),
               (16000.0, 9000.0), (20000.0, 1000.0)],
    }
    out = []
    for name, pts in shapes.items():
        for v in (100.0, 200.0):
            span = math.hypot(pts[0][0], pts[0][1]) + sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(pts, pts[1:]))
            cid = f"{name}_v{int(v)}"
            out.append(_cfg(
                "air_route", cid,
                duration=span / v + 60.0,
                trim_s=span / v * 1.15,
                start={"brg": _geo_brg(pts[0][0], pts[0][1]),
                       "dist": math.hypot(*pts[0]), "alt": 3000.0, "v": v,
                       "seed": offset_to_heading(*pts[0])},
                wps=[{"brg": _geo_brg(b[0] - a[0], b[1] - a[1]),
                      "dist": math.hypot(b[0] - a[0], b[1] - a[1]),
                      "alt": 3000.0, "v": v}
                     for a, b in zip(pts, pts[1:])],
                milsim_params=_base_air(
                    max_speed=f"{v:g} m/s", turn_rate="45 deg/s",
                    radial_accel="22 m/s2"),
                afsim_mover=("kinematic", _kin_afsim(v, 45.0, 22.0)),
                domain="air",
            ))
    return out


def _surface_cfgs() -> list[dict]:
    out = []
    for v, comp, dom, fam in ((8.0, "GROUND_MOVER", "land", "ground_straight"),
                              (25.0, "GROUND_MOVER", "land", "ground_straight")):
        brg = 0.0 if fam == "ground_straight" else 60.0
        seed = offset_to_heading(*_bearing_to_offset(brg, 3000.0))
        cid = f"{fam.split('_')[0]}_v{int(v)}"
        p = {"max_speed": f"{v:g} m/s", "turn_rate": "30 deg/s",
             "radial_accel": "5 m/s2", "linear_accel": "3 m/s2",
             "period": "0.1 s"}
        out.append(_cfg(
            fam, cid, duration=3000.0 / v + 30.0,
            trim_s=3000.0 / v,
            start={"brg": brg, "dist": 3000.0, "alt": 0.0, "v": v,
                   "seed": seed},
            wps=[], milsim_params=p,
            afsim_mover=("kinematic", _kin_afsim(v, 30.0, 5.0, 3.0)),
            domain=dom, mover_component=comp,
        ))
    return out


def _sub_cfgs() -> list[dict]:
    out = []
    for depth in (60.0, 200.0):
        for v in (4.0, 10.0):
            cid = f"d{int(depth)}_v{int(v)}"
            p = {"max_speed": f"{v:g} m/s", "turn_rate": "5 deg/s",
                 "linear_accel": "0.5 m/s2", "cruise_depth": f"{depth:g} m",
                 "dive_rate": "5 m/s", "surface_rate": "5 m/s",
                 "depth_tolerance": "2 m", "period": "0.1 s"}
            out.append(_cfg(
                "sub_straight", cid, duration=2000.0 / v + 60.0,
                trim_s=2000.0 / v,
                start={"brg": 45.0, "dist": 2000.0, "alt": -depth, "v": v,
                       "seed": offset_to_heading(*_bearing_to_offset(45.0,
                                                                    2000.0))},
                wps=[], milsim_params=p,
                afsim_mover=("kinematic", _kin_afsim(v, 5.0, 2.0, 0.5)),
                domain="subsurface", mover_component="SUBSURFACE_MOVER",
            ))
    return out


_ASM_BODY = {
    "mass": "519 kg", "fuel_mass": "200 kg", "thrust": "2900 N",
    "specific_impulse": "1478 s", "drag_area": "0.00923 m2",
    "radial_accel": "45 m/s2", "max_speed": "600 m/s",
    "launch_speed": "231.5 m/s", "launch_fpa": "0 deg", "period": "0.5 s",
}


#: 参照弹（demos/new_guidance/weapons/agm/blue_asm_missile.txt）mover 里
#: 写死的两个预规划航路点（绝对经纬度）。milsim 侧把同两点换算成局部平面
#: 坐标喂给制导件的 route 参数（ROUTE_ASM_GUIDANCE，v0.13.66）。
ASM_ROUTE_LL = ((39.0, -89.5), (38.8, -89.5))


def _asm_cfgs() -> list[dict]:
    out = []
    for dist_km in (50.0, 80.0):
        for lalt in (500.0, 8000.0):
            cid = f"d{int(dist_km)}km_a{int(lalt)}"
            out.append(_cfg(
                "asm", cid, duration=700.0,
                start={"brg": 30.0, "dist": dist_km * 1000.0, "alt": lalt,
                       "v": 231.5, "seed": None},
                wps=[], milsim_params=dict(_ASM_BODY),
                afsim_mover=("guided", ""),
                domain="air", mover_component="MISSILE_MOVER",
                anchor=ASM_ANCHOR, zone_radius_km=120,
                guidance="ROUTE_ASM_GUIDANCE",
            ))
            out[-1]["route_ll"] = ASM_ROUTE_LL
    return out


def build_configs(quick: bool, probe: bool, only: list[str],
                  skip_asm: bool) -> list[dict]:
    fams = {
        "air_straight": _straight_cfgs,
        "air_turn": _turn_cfgs,
        "air_vertical": _vertical_cfgs,
        "air_route": _route_cfgs,
        "ground_straight": _surface_cfgs,
        "asm": _asm_cfgs,
    }
    if only:
        fams = {k: v for k, v in fams.items() if k in only}
    if skip_asm:
        fams.pop("asm", None)
    out: list[dict] = []
    for name, build in fams.items():
        # ground/water 共用一个构造器：先按族过滤再切片
        cfgs = [c for c in build() if c["family"] == name]
        if probe:
            cfgs = cfgs[:1]
        elif quick:
            cfgs = cfgs[:2]
        out.extend(cfgs)
    return out


# ---------------------------------------------------------------------------
# AFSIM 侧：想定生成 + 运行 + 采样
# ---------------------------------------------------------------------------


def _wps_of(cfg: dict) -> list[tuple[float, float, float, float]]:
    """想定航路（含起点）：[(lat,lng,alt,v), ...]。

    平台起点恒在**锚点**（局部平面原点）；``start`` 描述第一个航路点
    （相对锚点的方位+距离），``wps`` 各项描述相对**上一航路点**的一腿。
    """
    frame = LocalFrame(*cfg["anchor"], 1000)
    lat0, lng0 = cfg["anchor"]
    s = cfg["start"]
    pts = [(lat0, lng0, s["alt"], s["v"])]
    cx, cy = _bearing_to_offset(s["brg"], s["dist"])   # 光标=当前航路点
    lat, lng = frame.world_to_geo(cx, cy)
    pts.append((lat, lng, s["alt"], s["v"]))
    for wp in cfg["wps"]:
        dx, dy = _bearing_to_offset(wp["brg"], wp["dist"])
        cx, cy = cx + dx, cy + dy                       # 逐腿累加
        lat, lng = frame.world_to_geo(cx, cy)
        pts.append((lat, lng, wp["alt"], wp["v"]))
    return pts


def afsim_script(cfg: dict) -> str:
    """平台件族的自包含 AFSIM 想定（弹族另有生成器）。

    KINEMATIC_MOVER 的参数块放**平台实例**（``("kinematic", …)``），
    WSF_AIR_MOVER 的参数块放 **platform_type**（``("air", …)``）——
    两边各按自己那份已验证的 demo 写法。
    """
    lat0, lng0 = cfg["anchor"]
    pts = _wps_of(cfg)
    s = cfg["start"]
    hdg_geo = (180.0 - s["seed"]) % 360.0 if s["seed"] is not None else 0.0
    kind, mover_block = cfg["afsim_mover"]
    if kind == "kinematic":
        type_mover = "   mover WSF_KINEMATIC_MOVER\n   end_mover"
        plat_mover = f"   mover\n      {mover_block}\n   end_mover"
    else:
        type_mover = f"   mover WSF_AIR_MOVER\n      {mover_block}\n   end_mover"
        plat_mover = ""
    lines = [
        f"# 生成：tools/cmp_movers.py · {cfg['family']} · {cfg['cid']}",
        f"end_time {cfg['duration'] + 10.0:.0f} sec",
        "",
        f"platform_type CMP_{cfg['cid'].upper()} WSF_PLATFORM",
        "   icon f-18",
        "   side blue",
        f"   spatial_domain {cfg['domain']}",
        type_mover,
        "end_platform_type",
        "",
        f"platform P1 CMP_{cfg['cid'].upper()}",
        f"   position {_fmt_latlng(pts[0][0], pts[0][1])} "
        f"altitude {pts[0][2]:g} m",
        f"   heading {hdg_geo:.2f} deg",
    ]
    if plat_mover:
        lines.append(plat_mover)
    lines.append("   route")
    for lat, lng, alt, v in pts:
        lines.append(f"      position {_fmt_latlng(lat, lng)} "
                     f"altitude {alt:g} m speed {v:g} m/s")
    lines += ["   end_route", "end_platform", "",
              _observer_block(afsim_ref.DEFAULT_TAG).strip(), ""]
    return "\n".join(lines)


def run_afsim_platform(mission: Path, cfg: dict, scripts_dir: Path):
    """自包含脚本直接跑；返回 [(t, lat, lng, alt, v, hdg_geo)]。"""
    text = afsim_script(cfg)
    (scripts_dir / "afsim").mkdir(parents=True, exist_ok=True)
    (scripts_dir / "afsim" / f"{cfg['cid']}.txt").write_text(
        text, encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="cmp_movers_") as work:
        proc = _run_once(mission, Path(work), text)
        if proc.returncode != 0:
            tail = "\n".join(proc.stdout.splitlines()[-15:])
            raise RuntimeError(f"AFSIM 跑失败（{cfg['cid']}）：\n{tail}")
        tag = afsim_ref.DEFAULT_TAG
        rows = []
        for line in proc.stdout.splitlines():
            if not line.startswith(f"{tag},"):
                continue
            f = line.split(",")[1:]
            # 观察器打的是 lat-33.0 / lng-(-84.0) 的偏移，加回去就是绝对值
            rows.append((float(f[0]), float(f[2]) + _LAT_ANCHOR,
                         float(f[3]) + _LNG_ANCHOR,
                         float(f[4]), float(f[5]), float(f[6])))
    if not rows:
        raise RuntimeError(f"AFSIM 跑通但没采到样本（{cfg['cid']}）")
    return rows


def run_afsim_asm(mission: Path, cfg: dict, scripts_dir: Path):
    """弹族：生成顶层脚本（内联想定），借 afsim_ref.capture 走 include 链。"""
    s = cfg["start"]
    lat0, lng0 = cfg["anchor"]
    frame = LocalFrame(lat0, lng0, 1000)
    dx, dy = _bearing_to_offset(s["brg"], s["dist"])
    llat, llng = frame.world_to_geo(dx, dy)
    hdg_geo = _geo_brg(-dx, -dy)  # 从发射点指向目标（目标在锚点=原点）
    tlat, tlng = lat0, lng0
    cid = cfg["cid"]
    text = f"""# 生成：tools/cmp_movers.py · asm · {cid}
define_path_variable CASE {cid}
log_file output/{cid}.log
include setup.txt
include_once weapons/agm/blue_asm_missile.txt

platform_type FIGHTER WSF_PLATFORM
   icon F-18E
   mover WSF_AIR_MOVER
      default_radial_acceleration 2 g
   end_mover
   weapon blue_asm_missile BLUE_ASM_MISSILE
      quantity 1
   end_weapon
end_platform_type

platform_type TARGET WSF_PLATFORM
   icon C4I
end_platform_type

platform target TARGET
   side red
   position {_fmt_latlng(tlat, tlng)} altitude 1 m agl
end_platform

platform fighter FIGHTER
   side blue
   route
      position {_fmt_latlng(llat, llng)} altitude {s['alt']:g} m speed 450 kts heading {hdg_geo:.2f} deg
   end_route
   track
      position {_fmt_latlng(tlat, tlng)} altitude 1 m agl
      platform target
   end_track
   execute at_time 0.1 sec relative
      WsfLocalTrackList trackList = PLATFORM.MasterTrackList();
      WsfLocalTrack track = trackList.Entry(0);
      PLATFORM.Weapon("blue_asm_missile").Fire(track);
   end_execute
end_platform

end_time {cfg['duration']:.0f} sec
"""
    # new_guidance 在 **AFSIM 根**（milsim 的上一级），include 链相对它解析
    gen_dir = ROOT.parent / "demos" / "new_guidance"
    gen_file = gen_dir / f"_cmp_gen_{cid}.txt"
    gen_file.write_text(text, encoding="utf-8")
    out_csv = scripts_dir / "afsim" / f"{cid}.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    try:
        afsim_ref.capture(gen_file, out_csv, None, set())
    finally:
        gen_file.unlink(missing_ok=True)
    rows = []
    for row in csv.DictReader(out_csv.read_text(encoding="utf-8").splitlines()):
        # 观察器对**所有**带机动件的平台回调：fighter/target 的行不要，
        # 只留弹本体（blue_asm_missile-1）。
        if "missile" not in row["platform"].lower():
            continue
        rows.append((float(row["time_s"]), float(row["lat_deg"]),
                     float(row["lon_deg"]), float(row["alt_m"]),
                     float(row["speed_mps"]), float(row["heading_deg"])))
    out_csv.unlink()
    if not rows:
        raise RuntimeError(f"AFSIM 弹族没采到样本（{cid}）")
    return rows


# ---------------------------------------------------------------------------
# milsim 侧：想定生成 + 运行 + 采样
# ---------------------------------------------------------------------------


def milsim_script(cfg: dict) -> str:
    lat0, lng0 = cfg["anchor"]
    frame = LocalFrame(lat0, lng0, 1000)
    pts = _wps_of(cfg)
    comp = cfg["mover_component"]
    lines = [
        f"# 生成：tools/cmp_movers.py · {cfg['family']} · {cfg['cid']}",
        "simulation",
        f"    name     \"cmp {cfg['cid']}\"",
        "    max_step 0.1 s",
        f"    seed     {SEED}",
        "end_simulation",
        "",
        "zone CMP_AREA",
        f"    anchor     {lat0} {lng0}",
        f"    radius     {cfg['zone_radius_km']} km",
        "    resolution 1 km",
        "    layers     1",
        "    terrain    flat",
        "end_zone",
        "",
        f"component_type {comp}",
        "end_component_type",
        f"component_type CMP_BODY : {comp}",
    ]
    for k, v in cfg["milsim_params"].items():
        lines.append(f"    {k:<20}{v}")
    lines += ["end_component_type", ""]
    if cfg.get("guidance"):
        guide_lines = [
            f"component_type {cfg['guidance']}",
            "end_component_type",
            "platform_type ASM_TARGET",
            "    side red",
            "end_platform_type",
            "platform_type CMP_PLAT",
            "    side blue",
            "    component mover CMP_BODY",
            "    end_component",
            f"    component guidance {cfg['guidance']}",
            "        target_name   TGT_1",
            "        lethal_radius 100 m",
        ]
        if cfg.get("route_ll"):
            # 参照弹的预规划航路（绝对经纬度）→ 局部平面坐标。
            # waypoint_radius 给 2 km：240 m/s 定速下转弯半径 ≈ v²/a =
            # 1280 m，切点半径给得比它小会画圆够不着（HORIZ_ROUTE 语义）。
            wxy = [frame.geo_to_world(la, ln) for la, ln in cfg["route_ll"]]
            flat = " ".join(f"{q:.1f}" for p in wxy for q in p)
            guide_lines += [
                f"        route         {flat}",
                "        waypoint_radius 2 km",
            ]
        guide_lines += ["    end_component", "end_platform_type", ""]
        guide_lines += [
            "platform TGT_1 ASM_TARGET",
            f"    position latlng {lat0:.5f} {lng0:.5f}",
            "end_platform",
            "platform P1 CMP_PLAT",
            # 弹的起点是**发射点**（锚点偏移后的那个点，pts[1]），不是锚点
            # ——锚点是目标位置。写错就成了"弹与目标同点"，起飞即触地。
            f"    position latlng {pts[1][0]:.5f} {pts[1][1]:.5f}",
            f"    heading  {cfg['seed_hdg']:.2f} deg",
            "end_platform",
            "",
        ]
        lines += guide_lines
    else:
        lines += [
            "platform_type CMP_PLAT",
            "    side blue",
            "    component mover CMP_BODY",
            "    end_component",
            "end_platform_type",
            "",
            "platform P1 CMP_PLAT",
            f"    position latlng {pts[0][0]:.5f} {pts[0][1]:.5f}",
            "end_platform",
            "",
        ]
    return "\n".join(lines)


def run_milsim(cfg: dict, scripts_dir: Path):
    """跑 milsim 侧；返回 [(t, x, y, z, v, hdg_geo)]（世界平面米）。"""
    from milsim.simulation import Simulation

    lat0, lng0 = cfg["anchor"]
    frame = LocalFrame(lat0, lng0, 1000)
    s = cfg["start"]
    is_asm = bool(cfg.get("guidance"))
    if is_asm:
        dx, dy = _bearing_to_offset(s["brg"], s["dist"])
        cfg["seed_hdg"] = (180.0 - _geo_brg(-dx, -dy)) % 360.0
    text = milsim_script(cfg)
    (scripts_dir / "milsim").mkdir(parents=True, exist_ok=True)
    path = scripts_dir / "milsim" / f"{cfg['cid']}.txt"
    path.write_text(text, encoding="utf-8")

    sim = Simulation.from_scenario_file(path)
    sim.build()
    sim.initialize()
    eid = sim.registry.by_name("P1").entity_id
    mover = sim.entities[eid].require("mover")

    x, y = frame.geo_to_world(*(_wps_of(cfg)[1][:2] if is_asm
                                else _wps_of(cfg)[0][:2]))
    sim.store.set_pose(eid, x, y, s["alt"], s["seed"] or 0.0, s["v"])

    if not is_asm:
        # 航路**不含起点自己**：起点当第一个航路点会造出零长度首腿，
        # 机动件在零向量上算不出方向、原地掉头卡死（demo 的 _route_entry
        # 跳过重合点就是这个道理，实测踩过）。
        pts = []
        for (lat, lng, alt, _v) in _wps_of(cfg)[1:]:
            wx, wy = frame.geo_to_world(lat, lng)
            pts.append((wx, wy, alt))
        speed, started = mover._speed, mover._started_us
        mover.move_along_route(pts)
        mover._speed = speed if speed > 0.0 else s["v"]
        mover._started_us = started

    rows = [(0.0, x, y, s["alt"], s["v"], (180.0 - (s["seed"] or 0.0)) % 360.0)]
    steps = int(cfg["duration"] / DT_S)
    for _ in range(steps):
        sim.run_for(int(DT_S * 1_000_000))
        pose = sim.store.pose_of(eid)
        if pose is None:
            break
        px, py, pz, heading, pv = pose
        rows.append((sim.engine.now / 1_000_000.0, px, py, pz, pv,
                     (180.0 - heading) % 360.0))
        if is_asm:
            guide = mover.guidance()
            if guide is not None and mover.terminate_reason():
                break
            if math.hypot(px, py) < 100.0:
                break
    return rows


# ---------------------------------------------------------------------------
# 对齐与指标
# ---------------------------------------------------------------------------


def _wrap180(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def compare(cfg: dict, ref_rows, sim_rows) -> dict:
    """ref=AFSIM（经纬度）→ 换算到 milsim 同一平面；两套口径出指标。"""
    lat0, lng0 = cfg["anchor"]
    frame = LocalFrame(lat0, lng0, 1000)
    ref = [(t, *frame.geo_to_world(la, ln), z, v, h)
           for t, la, ln, z, v, h in ref_rows]
    # 名义飞行时刻截断（去 AFSIM 走完后的外推尾巴）
    trim = cfg.get("trim_s")
    if trim is not None:
        ref = [r for r in ref if r[0] <= trim + 1e-9]
        sim_rows = [r for r in sim_rows if r[0] <= trim + 1e-9]
    # 同刻去重（观察器偶发同刻双回调；等长对齐要求严格递增）
    def _dedupe(rows):
        out, last = [], None
        for r in sorted(rows, key=lambda x: x[0]):
            if last is not None and r[0] <= last + 1e-9:
                continue
            out.append(r)
            last = r[0]
        return out
    ref = _dedupe(ref)
    sim_rows = _dedupe(sim_rows)
    # --- 等长口径：插值对齐到 1 s 栅格 ---
    ta = [r[0] for r in ref]
    tb = [r[0] for r in sim_rows]
    _, ax, bx = align_pairs(ta, [r[1] for r in ref], tb, [r[1] for r in sim_rows], DT_S)
    _, ay, by = align_pairs(ta, [r[2] for r in ref], tb, [r[2] for r in sim_rows], DT_S)
    _, az, bz = align_pairs(ta, [r[3] for r in ref], tb, [r[3] for r in sim_rows], DT_S)
    _, av, bv = align_pairs(ta, [r[4] for r in ref], tb, [r[4] for r in sim_rows], DT_S)
    n = len(ax)
    errs = [math.hypot(bx[i] - ax[i], by[i] - ay[i]) for i in range(n)]
    rmse_h = math.sqrt(sum(e * e for e in errs) / n)
    maxae_h = max(errs)
    rmse_alt = rmse(bz, az)
    rmse_v = rmse(bv, av)
    u_v = tic(bv, av)
    # 航向：两边都是地理口径，平均绝对差
    ha = [(r[5]) for r in ref]
    hb = [(r[5]) for r in sim_rows]
    _, ah, bh = align_pairs(ta, ha, tb, hb, DT_S)
    dh = sum(abs(_wrap180(bh[i] - ah[i])) for i in range(n)) / n
    # --- 形态口径：弗雷歇直接吃原始点列 ---
    def _stride(pts):
        if len(pts) <= FRECHET_MAX_PTS:
            return pts
        step = math.ceil(len(pts) / FRECHET_MAX_PTS)
        return pts[::step]
    fr = frechet_distance(
        _stride([(r[1], r[2], r[3]) for r in ref]),
        _stride([(r[1], r[2], r[3]) for r in sim_rows]))
    path_len = sum(
        math.hypot(sim_rows[i + 1][1] - sim_rows[i][1],
                   sim_rows[i + 1][2] - sim_rows[i][2])
        for i in range(len(sim_rows) - 1))
    return {
        "n": n, "dur": min(ta[-1], tb[-1]), "path_km": path_len / 1000.0,
        "rmse_h_m": rmse_h, "maxae_h_m": maxae_h, "rmse_alt_m": rmse_alt,
        "rmse_v_mps": rmse_v, "tic_v": u_v, "mean_dhdg_deg": dh,
        "frechet_m": fr,
        "ratio": rmse_h / max(path_len, 1.0),
        "verdict": "ok" if rmse_h / max(path_len, 1.0) <= PASS_RATIO else "看一眼",
    }


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------


def _plot(family: str, cfg, ref, sim_rows, out: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    lat0, lng0 = cfg["anchor"]
    frame = LocalFrame(lat0, lng0, 1000)
    refm = [(t, *frame.geo_to_world(la, ln), z)
            for t, la, ln, z, _v, _h in ref]
    fig, (p1, p2) = plt.subplots(1, 2, figsize=(11, 4.5))
    p1.plot([r[1] / 1000 for r in refm], [r[2] / 1000 for r in refm],
            label="AFSIM", lw=1.5)
    p1.plot([r[1] / 1000 for r in sim_rows], [r[2] / 1000 for r in sim_rows],
            "--", label="milsim", lw=1.5)
    p1.set_xlabel("East km"); p1.set_ylabel("North km")
    p1.set_title(f"{family} / {cfg['cid']} (worst)")
    p1.legend(); p1.set_aspect("equal")
    p2.plot([r[0] for r in refm], [r[3] for r in refm], label="AFSIM")
    p2.plot([r[0] for r in sim_rows], [r[3] for r in sim_rows], "--",
            label="milsim")
    p2.set_xlabel("t s"); p2.set_ylabel("Altitude m"); p2.legend()
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120)
    plt.close(fig)


def report(results: list[dict], out_dir: Path, skipped: list[tuple[str, str]]) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    cols = ["family", "cid", "n", "dur_s", "path_km", "rmse_h_m", "maxae_h_m",
            "rmse_alt_m", "rmse_v_mps", "tic_v", "mean_dhdg_deg",
            "frechet_m", "ratio_pct", "verdict", "params"]
    with (out_dir / "summary.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for r in results:
            w.writerow([
                r["cfg"]["family"], r["cfg"]["cid"], r["m"]["n"],
                f"{r['m']['dur']:.0f}", f"{r['m']['path_km']:.2f}",
                f"{r['m']['rmse_h_m']:.1f}", f"{r['m']['maxae_h_m']:.1f}",
                f"{r['m']['rmse_alt_m']:.1f}", f"{r['m']['rmse_v_mps']:.2f}",
                f"{r['m']['tic_v']:.4f}", f"{r['m']['mean_dhdg_deg']:.2f}",
                f"{r['m']['frechet_m']:.1f}",
                f"{r['m']['ratio'] * 100:.2f}", r["m"]["verdict"],
                r["params_str"],
            ])
        for fam, why in skipped:
            w.writerow([fam, "(AFSIM 侧跑失败)", "", "", "", "", "", "", "",
                        "", "", "", "", "失败", why])

    lines = [
        "# 机动组件 × AFSIM 批量对拍报告",
        "",
        f"- 采样口径：milsim 1 s 步长 vs AFSIM 0.1 s 更新 → 等长指标经 "
        f"`align` 插值对齐到 {DT_S:g} s 栅格（不外推）；弗雷歇直接吃原始点列。",
        f"- 判读线：rmse_h / 航程 ≤ {PASS_RATIO:.0%} 记 ok，否则『看一眼』。",
        "- 弹族（v0.13.66 起）：ROUTE_ASM_GUIDANCE + 俯仰限位已补（§9-31/§9-35），"
        "横向按同一判读线判；剩余差在下降段垂向律形状（线性增益 vs 参照 sqrt 剖面）、"
        "转角形状（切点半径 vs 参照飞到航路终点才转）与末段时间相位。"
        "阻力仍是常数口径（亚音速档）。",
        "",
    ]
    fams: dict[str, list[dict]] = {}
    for r in results:
        fams.setdefault(r["cfg"]["family"], []).append(r)
    for fam in sorted(fams):
        lines += [f"## {fam}", "",
                  "| 配置 | 航程 km | RMSE_h m | MaxAE_h m | RMSE_alt m | "
                  "RMSE_v m/s | TIC_v | Δhdg° | Fréchet m | 占比 | 判 |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in sorted(fams[fam], key=lambda x: x["m"]["rmse_h_m"]):
            m = r["m"]
            lines.append(
                f"| {r['cfg']['cid']} | {m['path_km']:.2f} | "
                f"{m['rmse_h_m']:.1f} | {m['maxae_h_m']:.1f} | "
                f"{m['rmse_alt_m']:.1f} | {m['rmse_v_mps']:.2f} | "
                f"{m['tic_v']:.4f} | {m['mean_dhdg_deg']:.1f} | "
                f"{m['frechet_m']:.1f} | {m['ratio'] * 100:.2f}% | "
                f"{m['verdict']} |")
        worst = max(fams[fam], key=lambda x: x["m"]["rmse_h_m"])
        if worst.get("plot"):
            lines += ["", f"最差配置叠图：`plots/{fam}_{worst['cfg']['cid']}.png`",
                      ""]
    if skipped:
        lines += ["## 跑失败的配置", ""]
        for fam, why in skipped:
            lines.append(f"- {fam}: {why}")
        lines.append("")
    rp = out_dir / "report.md"
    rp.write_text("\n".join(lines), encoding="utf-8")
    return rp


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="每族抽 2 个配置")
    ap.add_argument("--probe", action="store_true", help="每族抽 1 个配置")
    ap.add_argument("--only", default="", help="逗号分隔的族过滤")
    ap.add_argument("--skip-asm", action="store_true")
    ap.add_argument("--out", default=str(_OUT))
    args = ap.parse_args(argv)
    only = [s for s in args.only.split(",") if s]
    cfgs = build_configs(args.quick, args.probe, only, args.skip_asm)
    out_dir = Path(args.out)
    scripts_dir = out_dir / "scenarios"
    scripts_dir.mkdir(parents=True, exist_ok=True)
    mission = _mission_exe()
    print(f"mission.exe = {mission}；配置 {len(cfgs)} 个")

    results: list[dict] = []
    skipped: list[tuple[str, str]] = []
    for i, cfg in enumerate(cfgs, 1):
        cid, fam = cfg["cid"], cfg["family"]
        params_str = ";".join(f"{k}={v}" for k, v in cfg["milsim_params"].items()
                              if k != "period")
        print(f"[{i}/{len(cfgs)}] {fam}/{cid} ...", flush=True)
        try:
            if fam == "asm":
                ref = run_afsim_asm(mission, cfg, scripts_dir)
            else:
                ref = run_afsim_platform(mission, cfg, scripts_dir)
            sim_rows = run_milsim(cfg, scripts_dir)
            m = compare(cfg, ref, sim_rows)
            results.append({"cfg": cfg, "m": m, "params_str": params_str,
                            "ref": ref, "sim": sim_rows})
            print(f"    rmse_h={m['rmse_h_m']:.1f} m  frechet={m['frechet_m']:.1f} m"
                  f"  {m['verdict']}", flush=True)
        except Exception as exc:  # noqa: BLE001 —— 单配置失败不拖垮整批
            skipped.append((f"{fam}/{cid}", str(exc)[:300]))
            print(f"    失败：{exc}", flush=True)

    # 每族最差配置出叠图
    fams: dict[str, list[dict]] = {}
    for r in results:
        fams.setdefault(r["cfg"]["family"], []).append(r)
    for fam, rows in fams.items():
        worst = max(rows, key=lambda x: x["m"]["rmse_h_m"])
        png = out_dir / "plots" / f"{fam}_{worst['cfg']['cid']}.png"
        try:
            _plot(fam, worst["cfg"], worst["ref"], worst["sim"], png)
            worst["plot"] = True
        except Exception:  # noqa: BLE001
            worst["plot"] = False

    rp = report(results, out_dir, skipped)
    ok = sum(1 for r in results if r["m"]["verdict"] == "ok")
    print(f"\n完成：{len(results)} 成功 / {len(skipped)} 失败；"
          f"ok {ok}、看一眼 {len(results) - ok}")
    print(f"报告 → {rp}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
