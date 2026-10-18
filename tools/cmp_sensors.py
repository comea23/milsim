#!/usr/bin/env python3
"""域二 · 雷达探测时序对拍：milsim ↔ AFSIM（§5.12.10 静态对拍的动态版）。

静态矩阵（spy1d_static_cmp.txt）已经把 SNR 本身对到 ≤0.109 dB；本工具把
**同一部雷达**放到动目标场景里，对拍"探测"这一层的三样东西：

1. **探测掩码序列**——每次照射"SNR ≥ θ（Pd=0.5 线）"算不算探到，两侧
   逐照射对齐后算 Jaccard / SMC + 漏报/虚警计数；
2. **SNR(t) 序列**——逐照射 SNR 的 RMSE / MaxAE；
3. **首次探测**——掩码首次为真的时刻与斜距（绝对差）。

两侧口径（对齐 §5.12.10 的结论）：
- 雷达参数逐字抄 ``demos/afsim_mover/spy1d_static_cmp.txt``（已验证映射）：
  3.3 GHz、峰值 6 MW、主瓣 40 dB、B_r 2 MHz、N = −151.722 dBW、帧周期 2 s、
  方位扇面 ±45°、俯仰带 2~80°、swerling 0、required_pd 0.5；
- **判探是确定性的**：掩码 = SNR ≥ θ，θ = milsim 标定法的 Pd=0.5 线
  （``single_pulse_snr(detect_range, 目标RCS)``）。AFSIM 侧不用它内部的
  Pd 掷骰（``SENSOR_DETECTION_ATTEMPT`` 只读 ``SignalToNoise()``），两侧
  用同一条线切——掷骰口径不可跨引擎对齐，确定性边界才是可对拍的量；
- 目标：直线进场（正北 90 km 处、朝南、定高定速），两侧同一几何。

用法::

    python tools/cmp_sensors.py                 # 全部 12 配置
    python tools/cmp_sensors.py --probe         # 每维取 1 个（验管线）
    python tools/cmp_sensors.py --only a10000_v250_r0p1

存档：``demos/afsim_cmp/sensors/``（summary.csv + report.md + 两侧想定）。
"""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from afsim_ref import _mission_exe, _run_once  # noqa: E402
from milsim.services.map.hex import LocalFrame  # noqa: E402

ANCHOR = (30.0, 120.0)          # spy1d 静态对拍的同一片海
DT_S = 1.0                      # 采样栅格
SCAN_S = 2.0                    # 两侧同拍：milsim scan_interval = AFSIM frame_time
PAIR_TOL_S = 1.0                # 逐照射配对容差（半拍）
PASS_JACCARD = 0.95             # 判读线：掩码 Jaccard
PASS_FIRST_S = 4.0              # 判读线：首探时刻差 ≤ 2 拍

# 雷达参数（spy1d 静态对拍的同源映射；两侧逐字对齐）
# ★ milsim 走**标定法**（不给 peak_power/antenna_gain_db）：SNR 曲线由
#   detect_range 锚定（main 里按 AFSIM 曲线单点标定），噪声显式给
#   （-151.722 dBW 换算成瓦——dBW 刻意不收，params.py 明文）。
RADAR_PARAMS = {
    "scan_interval": "2 s",
    # ★ 固定指向（fov=0 + beam_width）：目标沿正北子午线直进，方位恒 0，
    #   固定扇区才能与 AFSIM 的 azimuth_scan_limits ±45°（每帧重扫）同拍——
    #   milsim 的扇扫是**旋转**的（90° 扇面 8 s 才重访），节奏不同构。
    "fov": "0 deg",
    "beam_width": "2 deg",
    # ★ face/scan_center 用**存储约定**（bearing = atan2(dx,−dy)，+y = 地理北
    #   对应存储方位 180）：固定指向朝北要写 180，写 0 是朝南。
    "scan_center": "180 deg",
    "fov_el_min": "2 deg",
    "fov_el_max": "80 deg",
    "max_range": "300 km",
    "detect_range": "120 km",
    "required_pd": "0.5",
    "reference_rcs": "0.1 m2",
    "frequency": "3.3 GHz",
    "bandwidth": "2 MHz",
    "noise_power": f"{10 ** (-151.722 / 10):.9e} W",
    "hits_to_establish": "1",
    "establish_window": "1",
    "swerling_case": "0",
    "probability_of_false_alarm": "1e-6",
    # AFSIM 两侧各乘 cos²（tx/rx 指数 2+2）⇔ 本项目单次 cos⁴
    "ebs_loss_exponent": "4",
}

START_N = 90_000.0              # 目标起点：正北 90 km
END_N = 2_500.0                 # 终点：距雷达 2.5 km（俯仰带内收尾）


def _fmt(lat: float, lng: float) -> str:
    """AFSIM 位置写法：``30.00000n 120.00000e``（十进制 + 半球字母）。"""
    ns = "n" if lat >= 0 else "s"
    ew = "e" if lng >= 0 else "w"
    return f"{abs(lat):.5f}{ns} {abs(lng):.5f}{ew}"


def _bearing_to_offset(brg_deg: float, dist_m: float) -> tuple[float, float]:
    r = math.radians(brg_deg)
    return dist_m * math.sin(r), dist_m * math.cos(r)


# ---------------------------------------------------------------------------
# 配置网格：高度 × 速度 × RCS
# ---------------------------------------------------------------------------

def build_configs(probe: bool, only: list[str]) -> list[dict]:
    out = []
    for alt in (5_000.0, 10_000.0, 20_000.0):
        for v in (150.0, 250.0):
            for rcs in (0.1, 1.0):
                cid = f"a{int(alt)}_v{int(v)}_r{str(rcs).replace('.', 'p')}"
                out.append({
                    "cid": cid, "alt": alt, "v": v, "rcs": rcs,
                    "duration": (START_N - END_N) / v + 20.0,
                })
    if only:
        out = [c for c in out if c["cid"] in only]
    if probe:
        out = out[:1]
    return out


# ---------------------------------------------------------------------------
# AFSIM 侧
# ---------------------------------------------------------------------------

def _kin_block(v: float) -> str:
    # 与 cmp_movers 同一份已验证写法（急弯增益带上，直线用不到但无害）
    return f"""   mover
      initial_speed {v:g} m/s
      target_speed {v:g} m/s
      maximum_linear_acceleration 20 m/s2
      maximum_radial_acceleration 100 m/s2
      maximum_body_turn_rate 120 deg/sec
      maximum_body_roll_rate 500 deg/sec
      proportional_navigation_gain 10
      velocity_pursuit_gain 30
      update_interval 0.1 sec
   end_mover"""


def afsim_script(cfg: dict, power_db: float = 0.0) -> str:
    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    slat, slng = frame.world_to_geo(0.0, START_N)
    elat, elng = frame.world_to_geo(0.0, END_N)
    rcs = cfg["rcs"]
    # 发射功率按 power_db 缩放（6 MW 基准）：探测边界要落在航路内，
    # 由 main 按首跑的 AFSIM 曲线自动定标——功率对两侧同乘，形状不变。
    power_w = 6e6 * 10 ** (power_db / 10.0)
    return f"""# 生成：tools/cmp_sensors.py · {cfg['cid']}（功率偏移 {power_db:+.1f} dB）
end_time {cfg['duration']:.0f} sec

radar_signature SIG_TGT
   constant {rcs:g} m2
end_radar_signature

antenna_pattern LR_ANT
   circular_pattern
   peak_gain 40 db
   beamwidth 2 deg
   minimum_gain -30 db
end_antenna_pattern

script void detAttempt(WsfPlatform aPlatform, WsfSensor aSensor,
                       WsfPlatform aTarget, WsfSensorInteraction aResult)
   WsfGeoPoint p = aTarget.Location();
   writeln("DET,", TIME_NOW, ",", aResult.SignalToNoise(), ",",
           p.Latitude() - {lat0}, ",", p.Longitude() - {lng0}, ",",
           p.Altitude());
end_script

observer
   enable SENSOR_DETECTION_ATTEMPT detAttempt
end_observer

platform_type RADAR_SHIP WSF_PLATFORM
   side blue
   sensor s WSF_RADAR_SENSOR
      frame_time                     2.0 sec
      scan_mode                      azimuth_and_elevation
      azimuth_scan_limits            -45 deg 45 deg
      elevation_scan_limits          2.0 deg 80.0 deg
      transmitter
         antenna_pattern             LR_ANT
         electronic_beam_steering both
         electronic_beam_steering_loss_exponent 2.0
         power                       {power_w:.6e} W
         frequency                   3.3 ghz
      end_transmitter
      receiver
         antenna_pattern             LR_ANT
         electronic_beam_steering both
         electronic_beam_steering_loss_exponent 2.0
         bandwidth                   2.0 mhz
         noise_power                 -151.722 dBw
      end_receiver
      hits_to_establish_track        1 1
      probability_of_false_alarm     1.0e-6
      required_pd                    0.5
      swerling_case                  0
      reports_range
      reports_bearing
      reports_elevation
      reports_location
      on
   end_sensor
end_platform_type

platform_type INTRUDER WSF_PLATFORM
   side red
   radar_signature SIG_TGT
   mover WSF_KINEMATIC_MOVER
   end_mover
end_platform_type

platform R RADAR_SHIP
   position {_fmt(lat0, lng0)}
   heading 0 deg
end_platform

platform T INTRUDER
   position {_fmt(slat, slng)} altitude {cfg['alt']:g} m msl
   heading 180 deg
   route
      position {_fmt(slat, slng)} altitude {cfg['alt']:g} m speed {cfg['v']:g} m/s
      position {_fmt(elat, elng)} altitude {cfg['alt']:g} m speed {cfg['v']:g} m/s
   end_route
   {_kin_block(cfg['v'])}
end_platform
"""


def run_afsim(mission: Path, cfg: dict, out_dir: Path,
              power_db: float = 0.0) -> list[tuple]:
    """跑一次；返回 [(t, snr_db, lat, lng, alt), ...]（仅 DET 行）。"""
    text = afsim_script(cfg, power_db)
    with tempfile.TemporaryDirectory(prefix="cmp_sensors_") as work:
        proc = _run_once(mission, Path(work), text)
        if proc.returncode != 0:
            tail = "\n".join(proc.stdout.splitlines()[-15:])
            raise RuntimeError(f"AFSIM 跑失败（{cfg['cid']}）：\n{tail}")
        rows = []
        for line in proc.stdout.splitlines():
            if not line.startswith("DET,"):
                continue
            f = line.split(",")[1:]
            rows.append((float(f[0]), float(f[1]), float(f[2]) + ANCHOR[0],
                         float(f[3]) + ANCHOR[1], float(f[4])))
    if not rows:
        raise RuntimeError(f"AFSIM 没采到照射事件（{cfg['cid']}）")
    (out_dir / "afsim").mkdir(parents=True, exist_ok=True)
    (out_dir / "afsim" / f"{cfg['cid']}.txt").write_text(text, encoding="utf-8")
    return rows


# ---------------------------------------------------------------------------
# milsim 侧
# ---------------------------------------------------------------------------

def milsim_script(cfg: dict, params: dict) -> str:
    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    slat, slng = frame.world_to_geo(0.0, START_N)
    radar_lines = "\n".join(f"        {k:<26} {v}" for k, v in params.items())
    return f"""# 生成：tools/cmp_sensors.py · {cfg['cid']}
simulation
    name     "cmp sensors {cfg['cid']}"
    max_step 0.5 s
    seed     20261017
end_simulation

zone CMP_AREA
    anchor     {lat0} {lng0}
    radius     150 km
    resolution 1 km
    layers     1
    terrain    flat
end_zone

component_type AIR_MOVER
end_component_type
component_type CMP_BODY : AIR_MOVER
    max_speed       {cfg['v']:g} m/s
    turn_rate       120 deg/s
    radial_accel    100 m/s2
    linear_accel    20 m/s2
    period          0.1 s
end_component_type

platform_type RADAR_NODE
    side blue
    component sensor HEX_SEARCH_RADAR
{radar_lines}
    end_component
end_platform_type

platform_type INTRUDER
    side red
    radar_cross_section {cfg['rcs']:g} m2
    component mover CMP_BODY
    end_component
end_platform_type

platform R_RADAR RADAR_NODE
    position latlng {lat0} {lng0}
    heading 0 deg
end_platform

platform TGT INTRUDER
    position latlng {slat:.6f} {slng:.6f}
    heading 180 deg
end_platform
"""


def run_milsim(cfg: dict, out_dir: Path, d_m: float):
    """装配并跑 milsim 侧；返回 (records, snr_at_reference)。

    ``d_m`` 是标定法的 ``detect_range``（米）——由 main 按 AFSIM 曲线单点
    标定后传入。``single_pulse_snr`` 被工具层包一层记录——**不改引擎**：
    每次照射本来就调它（第 5~6 步取无扰 SNR、检测概率内部再调一次），包装
    器把 (时刻, SNR, 斜距) 记下来，语义与 AFSIM 的 ``detAttempt`` 同位。
    """
    from milsim.simulation import Simulation

    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    params = dict(RADAR_PARAMS)
    params["detect_range"] = f"{d_m:.3f} m"
    text = milsim_script(cfg, params)
    (out_dir / "milsim").mkdir(parents=True, exist_ok=True)
    path = out_dir / "milsim" / f"{cfg['cid']}.txt"
    path.write_text(text, encoding="utf-8")

    sim = Simulation.from_scenario_file(path)
    sim.build()
    sim.initialize()
    rid = sim.registry.by_name("R_RADAR").entity_id
    tid = sim.registry.by_name("TGT").entity_id
    radar = sim.entities[rid].require("sensor")
    mover = sim.entities[tid].require("mover")
    # snr_at_reference 是**线性**值（13.3124 线性 = 11.243 dB，非起伏
    # Pfa=1e-6/N=1/Pd=0.5 的门限）；判探线两侧统一用它的 dB 形式。
    theta_db = 10.0 * math.log10(float(radar.snr_at_reference))

    # 航路下达（想定不写命令，与 cmp_movers 同一条规矩）。★ 两处对齐坑：
    # ① set_pose 的 heading 是**存储约定**（罗盘角 = 180 − h），朝南要给 0
    #   （offset_to_heading(0, −1)），按罗盘角给 180 会朝北；
    # ② move_along_route 走 _restart_trip（速率归零 + 首帧不挪窝），要把
    #   速率状态接回去，否则与 AFSIM 的匀速直进差一段起步斜坡。
    from milsim.services.map.hex import offset_to_heading

    ex, ey = frame.geo_to_world(*frame.world_to_geo(0.0, END_N))
    sx, sy = frame.geo_to_world(*frame.world_to_geo(0.0, START_N))
    sim.store.set_pose(tid, sx, sy, cfg["alt"],
                       offset_to_heading(0.0, -1.0), cfg["v"])
    started = mover._started_us
    mover.move_along_route([(ex, ey, cfg["alt"])])
    mover._speed = cfg["v"]
    mover._started_us = started

    records: list[tuple[float, float, float]] = []
    seen: set[tuple[int, int]] = set()
    orig = radar.single_pulse_snr

    def wrap(range_m: float, rcs_m2=None, *, jammer_noise_w=0.0, steer_gain=1.0):
        snr = orig(range_m, rcs_m2, jammer_noise_w=jammer_noise_w,
                   steer_gain=steer_gain)
        t_s = sim.engine.now / 1_000_000.0
        key = (round(t_s * 10), round(range_m))
        if jammer_noise_w == 0.0 and key not in seen:
            seen.add(key)
            # single_pulse_snr 返回**线性**；AFSIM 的 SignalToNoise() 是 dB，
            # 统一成 dB 记录
            records.append((t_s, 10.0 * math.log10(snr) if snr > 0.0 else -999.0,
                            range_m))
        return snr

    radar.single_pulse_snr = wrap
    try:
        steps = int(cfg["duration"] / DT_S)
        for _ in range(steps):
            sim.run_for(int(DT_S * 1_000_000))
            if sim.store.pose_of(tid) is None:
                break
    finally:
        radar.single_pulse_snr = orig
    if not records:
        raise RuntimeError(f"milsim 没采到照射事件（{cfg['cid']}）")
    return records, theta_db


# ---------------------------------------------------------------------------
# 对齐与指标
# ---------------------------------------------------------------------------

def _wrap180(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def compare(cfg: dict, sim_recs, theta_db: float, ref_rows) -> dict:
    """逐照射配对 → 掩码 Jaccard/SMC/漏报/虚警 + SNR(t) RMSE + 首探差。"""
    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    # 名义飞行窗：两侧走完航路后的尾巴裁掉（milsim 停在终点、AFSIM 外推
    # 继续飞——与 cmp_movers 的截断同一口径）
    t_end = (START_N - END_N) / cfg["v"] + 1.0
    sim_recs = [r for r in sim_recs if r[0] <= t_end]
    ref_rows = [r for r in ref_rows if r[0] <= t_end]
    # AFSIM 行 → (t, snr, slant)：经纬偏移还原平面距（高差勾股）
    ref = []
    for t, snr, lat, lng, alt in ref_rows:
        x, y = frame.geo_to_world(lat, lng)
        ref.append((t, snr, math.hypot(math.hypot(x, y), alt - 0.0)))

    # 逐照射最近邻配对（容差半拍）
    pairs = []
    ref_left = list(ref)
    for t, snr, slant in sim_recs:
        best = min(ref_left, key=lambda r: abs(r[0] - t), default=None)
        if best is not None and abs(best[0] - t) <= PAIR_TOL_S:
            pairs.append((t, snr, slant, best[1], best[2]))
            ref_left.remove(best)
    unpaired_m, unpaired_a = len(sim_recs) - len(pairs), len(ref_left)

    mask_m = [snr >= theta_db for _t, snr, _s, _sn, _sl in pairs]
    mask_a = [sn >= theta_db for _t, _s, _sl, sn, _sl2 in pairs]
    n = len(pairs)
    both = sum(1 for a, b in zip(mask_m, mask_a) if a and b)
    miss = sum(1 for a, b in zip(mask_m, mask_a) if b and not a)    # AFSIM 探到、milsim 漏
    false = sum(1 for a, b in zip(mask_m, mask_a) if a and not b)   # milsim 探到、AFSIM 没有
    union = both + miss + false
    jaccard = both / union if union else 1.0
    smc = (both + sum(1 for a, b in zip(mask_m, mask_a) if not a and not b)) / n if n else 1.0

    errs = [a - b for _t, a, _s, b, _sl in pairs]
    rmse = math.sqrt(sum(e * e for e in errs) / n) if n else float("nan")
    maxae = max(abs(e) for e in errs) if errs else float("nan")
    denom = sum(abs(a) + abs(b) for _t, a, _s, b, _sl in pairs)
    tic = math.sqrt(sum(e * e for e in errs) / n) / (denom / n) if denom else 0.0

    def _first(mask, seq):
        for flag, row in zip(mask, seq):
            if flag:
                return row
        return None

    fm = _first(mask_m, pairs)
    fa = _first(mask_a, pairs)
    if fm and fa:
        dt = fm[0] - fa[0]
        dd = fm[2] - fa[4]
    else:
        dt = float("nan")
        dd = float("nan")

    ok = (
        jaccard >= PASS_JACCARD
        and (math.isnan(dt) or abs(dt) <= PASS_FIRST_S)
        and rmse <= 0.5
    )
    return {
        "cid": cfg["cid"], "n_pairs": n, "unpaired": f"{unpaired_m}/{unpaired_a}",
        "jaccard": jaccard, "smc": smc, "miss": miss, "false": false,
        "snr_rmse_db": rmse, "snr_maxae_db": maxae, "tic": tic,
        "theta_db": theta_db,
        "first_t_m": fm[0] if fm else float("nan"),
        "first_t_a": fa[0] if fa else float("nan"),
        "first_dt_s": dt,
        "first_d_m": dd,
        "verdict": "ok" if ok else "看一眼",
    }


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--probe", action="store_true", help="每维取 1 个验管线")
    ap.add_argument("--only", nargs="*", default=[], help="只跑这些 cid")
    args = ap.parse_args(argv)

    cfgs = build_configs(args.probe, args.only)
    mission = _mission_exe()
    out_dir = ROOT / "demos" / "afsim_cmp" / "sensors"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"mission.exe = {mission}；配置 {len(cfgs)} 个")

    results = []
    frame = LocalFrame(*ANCHOR, 1000)
    for i, cfg in enumerate(cfgs, 1):
        print(f"[{i}/{len(cfgs)}] sensors/{cfg['cid']} ...")
        try:
            # 判探线 θ（Pd=0.5 的 SNR，dB，与 RCS 无关——两侧同一条线）；
            # detect_range 由 AFSIM 曲线单点标定：milsim SNR(r) = θ + 40·lg(D/r)。
            _, theta_db = run_milsim(cfg, out_dir, 120_000.0)
            ref_rows = run_afsim(mission, cfg, out_dir, 0.0)
            d_m = _d_from_afsim(ref_rows, theta_db, frame)
            power_db = 0.0
            # 探测边界要落在航路内（0.3~0.85×起点），否则掩码全程全真/全假，
            # Jaccard 没有信息量。功率对两侧同乘、形状不变，缩放只挪边界。
            if not (0.3 * START_N <= d_m <= 0.85 * START_N):
                power_db = -40.0 * math.log10(d_m / (0.7 * START_N))
                ref_rows = run_afsim(mission, cfg, out_dir, power_db)
                d_m = _d_from_afsim(ref_rows, theta_db, frame)
            # ★ d_m 是"目标 RCS 下"AFSIM 的 θ 边界；milsim 标定法的
            #   detect_range 锚定的是 reference_rcs 的 SNR=θ——目标 RCS
            #   ≠ 参考时差 ΔdB=10lg(rcs/ref)，边界差 10^(ΔdB/40) 倍，
            #   必须换算回参考等效距离，否则整条 milsim 曲线抬高 ΔdB
            #   （r1p0 组 9.8 dB 系统差 + 全程虚警的根因）。
            delta_db = 10.0 * math.log10(cfg["rcs"] / 0.1)
            d_ref = d_m * 10 ** (-delta_db / 40.0)
            records, theta = run_milsim(cfg, out_dir, d_ref)
            row = compare(cfg, records, theta, ref_rows)
            row["detect_range_m"] = d_m
            row["detect_range_ref_m"] = d_ref
            row["power_db"] = power_db
            results.append(row)
            print(f"    jac={row['jaccard']:.3f} 漏/虚={row['miss']}/{row['false']}"
                  f" rmse={row['snr_rmse_db']:.3f} dB 首探差={row['first_dt_s']:+.1f} s"
                  f" (D={d_m/1000:.1f} km, P{power_db:+.1f}dB) → {row['verdict']}")
        except Exception as exc:                                    # noqa: BLE001
            print(f"    失败：{exc}")
            results.append({"cid": cfg["cid"], "verdict": "失败",
                            "error": str(exc)})

    _write_archive(out_dir, results)
    ok = sum(1 for r in results if r.get("verdict") == "ok")
    look = sum(1 for r in results if r.get("verdict") == "看一眼")
    bad = sum(1 for r in results if r.get("verdict") == "失败")
    print(f"完成：{len(results) - bad} 成功 / {bad} 失败；ok {ok}、看一眼 {look}")
    print(f"报告 → {out_dir / 'report.md'}")
    return 0 if bad == 0 else 1


def _d_from_afsim(ref_rows, s_ref: float, frame: LocalFrame) -> float:
    """按 AFSIM 曲线上最接近 60 km 的样本反推标定距离 D（米）。

    两侧 SNR 曲线同为 R⁻⁴ 形状（静态对拍已验），单点标定即整条曲线对齐；
    D 在数值上就是"AFSIM 的 Pd=0.5 边界距离"。
    """
    best = None
    for t, snr, lat, lng, alt in ref_rows:
        x, y = frame.geo_to_world(lat, lng)
        slant = math.hypot(math.hypot(x, y), alt)
        gap = abs(slant - 60_000.0)
        if best is None or gap < best[0]:
            best = (gap, slant, snr)
    return best[1] * 10 ** ((best[2] - s_ref) / 40.0)


COLS = ["cid", "n_pairs", "unpaired", "jaccard", "smc", "miss", "false",
        "snr_rmse_db", "snr_maxae_db", "tic", "theta_db", "detect_range_m",
        "detect_range_ref_m", "power_db", "first_t_m", "first_t_a",
        "first_dt_s", "first_d_m", "verdict"]


def _write_archive(out_dir: Path, results: list[dict]) -> None:
    lines = ["\t".join(COLS)]
    for r in results:
        lines.append("\t".join(
            f"{r.get(c, ''):.6g}" if isinstance(r.get(c), float) else str(r.get(c, ""))
            for c in COLS))
    (out_dir / "summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

    rep = [
        "# 域二 · 雷达探测时序对拍报告",
        "",
        "- 口径：雷达参数逐字抄 spy1d 静态对拍（已验证映射）；判探线 θ = 标定法"
        " Pd=0.5 的 SNR（两侧同一条线，不用 AFSIM 内部 Pd 掷骰）。",
        f"- 逐照射最近邻配对（容差 {PAIR_TOL_S:g} s，两侧同拍 2 s）；unpaired 列是"
        " [milsim 侧未配上 / AFSIM 侧未配上] 的照射数。",
        f"- 判读线：Jaccard ≥ {PASS_JACCARD}、首探时刻差 ≤ {PASS_FIRST_S:g} s、"
        "SNR RMSE ≤ 0.5 dB 记 ok。",
        "",
        "| 配置 | 配对 | 未配 | Jaccard | SMC | 漏 | 虚 | RMSE dB | MaxAE | TIC | θ dB | 首探差 s | 首探距差 m | 判 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        if r.get("verdict") == "失败":
            rep.append(f"| {r['cid']} | - | - | - | - | - | - | - | - | - | - | - | - | 失败：{r.get('error', '')[:40]} |")
            continue
        rep.append(
            f"| {r['cid']} | {r['n_pairs']} | {r['unpaired']} | {r['jaccard']:.3f} |"
            f" {r['smc']:.3f} | {r['miss']} | {r['false']} | {r['snr_rmse_db']:.3f} |"
            f" {r['snr_maxae_db']:.2f} | {r['tic']:.4f} | {r['theta_db']:.2f} |"
            f" {r['first_dt_s']:+.1f} | {r['first_d_m']:+.0f} | {r['verdict']} |")
    rep += ["", "指标明细 → `summary.csv`；两侧想定存档在 `afsim/` 与 `milsim/`。"]
    rep += [
        "",
        "## 结论",
        "",
        "- r1p0 组曾因工具层标定换算缺失整组虚警（milsim 曲线被抬高 10 dB）；",
        "  修法 `d_ref = d_m × 10^(−ΔdB/40)` 后与 r0p1 逐位一致（详见设计文档 §5.12.10.1）。",
        "- a20000 组残余 RMSE≈1.0 dB 为**与斜距/仰角无关的定常偏置**（5/10/20 km 高度",
        "  → 0.15/0.35/1.08 dB），对照实验（AFSIM EBS 指数归零 → 残差变强斜距相关，",
        "  反推 milsim 账 = −10lg cos⁴(el) 精确成立）定位为 **AFSIM 动态扫描路径的",
        "  定常增益账**，非雷达方程模型差（静态对拍 ≤0.109 dB）；milsim 零改动。",
    ]
    (out_dir / "report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
