#!/usr/bin/env python3
"""域三 · 干扰下 SNR(t) 时序对拍：milsim ↔ AFSIM（§5.14.16 静态对拍的动态版）。

静态矩阵（hop_static_cmp.txt，13 配置 gap≤0.109 dB）已把 J 账本身对死；
本工具把**同一部雷达 + 同一台干扰机**放进动目标场景，对拍"压制"这一层：

1. **受扰 SNR(t)**——自卫干扰（干扰机与目标同址编队，永远在主瓣内，排除
   §5.14.16 踩过的"偏主瓣进副瓣 42 dB"三维天线模型差），逐照射配对后算
   RMSE / TIC；
2. **烧穿距离**——受扰掩码（S/(N+J) ≥ θ）从远到近**首次为真**的斜距，
   两侧差单列标量；
3. 掩码 Jaccard（顺带）。

两侧口径：
- 雷达逐字抄 ``cmp_sensors``（spy1d 静态对拍同源映射，标定法，噪声显式给）；
  干净探测边界先按域二同一条路标定到 0.7×起点（功率对两侧同乘）；
- 干扰机：全向 0 dB、P_j ∈ {100 W, 1000 W} × B_j ∈ {2 MHz（瞄准式，F_BW=1）,
  20 MHz（F_BW=0.1）}，频率 3.3 GHz 居中；milsim 走产品路径（JamService 名册
  → jamming_effect → single_pulse_snr 的 ``jammer_noise_w``），AFSIM 走
  ESM 探测雷达后 ``StartJamming(TRACK, …, "noise_jamming")``（§5.14.16 同一
  写法）；
- 干扰机平台 RCS 0.0001 m²（SIG_EMPTY），不污染目标侧记录；与目标同航路
  同速（高度 +10 m 避免重合点）。

用法::

    python tools/cmp_jam.py                 # 全部 4 配置
    python tools/cmp_jam.py --probe         # 取 1 个验管线

存档：``demos/afsim_cmp/jam/``（summary.csv + report.md + 两侧想定）。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from afsim_ref import _mission_exe, _run_once  # noqa: E402
from cmp_sensors import (  # noqa: E402
    ANCHOR, DT_S, END_N, PAIR_TOL_S, RADAR_PARAMS, START_N,
    _fmt, _kin_block, run_milsim as _run_milsim_clean,
)
from milsim.services.map.hex import LocalFrame  # noqa: E402

PASS_RMSE = 0.5                 # 受扰 SNR RMSE 判读线（dB）
PASS_BURN = 2000.0              # 烧穿距离差判读线（m，≈1 个采样栅格）

#: 干扰机相对目标的高度偏置（m）——避开重合点的除零/同点歧义。
JAM_ALT_OFFSET = 10.0

BASE = {"alt": 5000.0, "v": 250.0, "rcs": 0.1}   # 域二 ok 组的代表几何


def build_configs(probe: bool, only: list[str]) -> list[dict]:
    # ★ 功率刻度：自卫干扰 J∝1/r²、S∝1/r⁴，90 km 起步的航路里 S/J 最多抬
    #   20lg(90/5.6)≈24 dB——100 W 干扰机（J/S(90 km)=−64.8 dB）**全程压制**，
    #   烧穿要落进航路必须把 P_j 缩到 mW 级。对拍口径不受影响：功率线性
    #   已由 §5.14.16 静态对拍钉死（10 W/1 W 两档逐位对齐）。
    grid = [
        (100.0, 2.0e6),     # 全程压制（掩码恒假，纯比 J 账 RMSE）
        (0.003, 2.0e6),     # 烧穿 ≈8.6 km 斜距（航路内）
        (0.001, 2.0e6),     # 烧穿 ≈14 km 斜距（航路中段）
        (0.001, 20.0e6),    # F_BW=0.1：从不压制（掩码恒真）
    ]
    out = []
    for p_j, b_j in grid:
        cid = f"p{str(p_j).replace('.', 'p')}_b{int(b_j/1e6)}m"
        cfg = dict(BASE)
        cfg.update({"cid": cid, "p_j": p_j, "b_j": b_j,
                    "duration": (START_N - END_N) / BASE["v"] + 20.0})
        out.append(cfg)
    if only:
        out = [c for c in out if c["cid"] in only]
    if probe:
        out = out[:1]
    return out


# ---------------------------------------------------------------------------
# AFSIM 侧
# ---------------------------------------------------------------------------

def afsim_script(cfg: dict, power_w: float, d_cal_hint: str = "",
                 with_jammer: bool = True) -> str:
    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    slat, slng = frame.world_to_geo(0.0, START_N)
    elat, elng = frame.world_to_geo(0.0, END_N)
    alt, v = cfg["alt"], cfg["v"]
    # 干净标定（with_jammer=False）不含干扰机：WSF_RF_JAMMER 的 power 必须 >0，
    # 标定配置 p_j=0 会被解析器拒绝（"Expected value '0' to be > 0"）。
    if with_jammer:
        jam_types = f"""platform_type JAMMER_PLAT WSF_PLATFORM
   side red
   radar_signature SIG_EMPTY
   mover WSF_KINEMATIC_MOVER
   end_mover
   weapon j WSF_RF_JAMMER_TEST
      on
   end_weapon
   sensor esm WSF_ESM_SENSOR
      frame_time              1.0 sec
      slew_mode               fixed
      maximum_range           500 km
      receiver
         antenna_pattern      OMNI
         detection_threshold  5 db
         noise_power          -180 dbw
         internal_loss        0 db
      end_receiver
      frequency_band          0.1 ghz 20 ghz
      reports_location
      reports_type
      reports_frequency
      on
      internal_link data_mgr
      internal_link j
      ignore_same_side
   end_sensor
   processor data_mgr WSF_TRACK_PROCESSOR
      purge_interval 30.0 sec
   end_processor
   processor jam_ctrl jam_mgr
   end_processor
end_platform_type

weapon WSF_RF_JAMMER_TEST WSF_RF_JAMMER
   transmitter
      power           {cfg['p_j']:.6f} w
      frequency_band  3.2 ghz 3.4 ghz
      antenna_pattern OMNI
      electronic_attack
         technique noise_jamming
            mitigation_class_name NOISE_JAMMING
            effect power_effect WSF_POWER_EFFECT
            end_effect
         end_technique
      end_electronic_attack
   end_transmitter
end_weapon

"""
        jam_platform = f"""platform J JAMMER_PLAT
   position {_fmt(slat, slng)} altitude {alt + JAM_ALT_OFFSET:g} m msl
   heading 180 deg
   route
      position {_fmt(slat, slng)} altitude {alt + JAM_ALT_OFFSET:g} m speed {v:g} m/s
      position {_fmt(elat, elng)} altitude {alt + JAM_ALT_OFFSET:g} m speed {v:g} m/s
   end_route
   {_kin_block(v)}
end_platform
"""
    else:
        jam_types = ""
        jam_platform = ""
    return f"""# 生成：tools/cmp_jam.py · {cfg['cid']}（{d_cal_hint}）
end_time {cfg['duration']:.0f} sec

radar_signature SIG_TGT
   constant {cfg['rcs']:g} m2
end_radar_signature

radar_signature SIG_EMPTY
   constant 0.0001 m2
end_radar_signature

antenna_pattern LR_ANT
   circular_pattern
   peak_gain 40 db
   beamwidth 2 deg
   minimum_gain -30 db
end_antenna_pattern

antenna_pattern OMNI
   constant 0 db
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

processor jam_mgr WSF_TASK_PROCESSOR
   script bool AddSpot()
      bool started = false;
      for (int i = 0; i < PLATFORM.WeaponCount(); i = i + 1)
      {{
         WsfWeapon j = PLATFORM.WeaponEntry(i);
         if (j.CanJam(3300000000.000000))
         {{
            started = StartJamming(TRACK, "jam1", j, 3300000000.000000,
                                   {cfg['b_j']:.1f}, "noise_jamming");
         }}
      }}
      return started;
   end_script

   evaluation_interval DETECTED 2.0 sec
   state DETECTED
      next_state JAMMING
         return AddSpot();
      end_next_state
   end_state
   evaluation_interval JAMMING 2.0 sec
   state JAMMING
      next_state DETECTED
         return false;
      end_next_state
   end_state
end_processor

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

{jam_types}platform R RADAR_SHIP
   position {_fmt(lat0, lng0)}
   heading 0 deg
end_platform

platform T INTRUDER
   position {_fmt(slat, slng)} altitude {alt:g} m msl
   heading 180 deg
   route
      position {_fmt(slat, slng)} altitude {alt:g} m speed {v:g} m/s
      position {_fmt(elat, elng)} altitude {alt:g} m speed {v:g} m/s
   end_route
   {_kin_block(v)}
end_platform
{jam_platform}"""


def run_afsim(mission: Path, cfg: dict, out_dir: Path, power_w: float,
              tag: str = "", with_jammer: bool = True) -> list[tuple]:
    text = afsim_script(cfg, power_w, tag, with_jammer=with_jammer)
    # ★ 受扰 SNR 必须读 event_output：observer 的 SignalToNoise() 是**无扰**
    #   S/(N+C)（§5.14.16 踩过的坑）；evt 的 SENSOR_DETECTION_ATTEMPT 行
    #   才带 S/(N+C+J)，且自带 Range 字段（连几何重建都省了）。
    text = text.rstrip() + ("\nevent_output\n   enable SENSOR_DETECTION_ATTEMPT"
                            "\n   file jam.evt\nend_event_output\n")
    with tempfile.TemporaryDirectory(prefix="cmp_jam_") as work:
        proc = _run_once(mission, Path(work), text)
        if proc.returncode != 0:
            tail = "\n".join(proc.stdout.splitlines()[-15:])
            raise RuntimeError(f"AFSIM 跑失败（{cfg['cid']}）：\n{tail}")
        evt_path = Path(work) / "jam.evt"
        if not evt_path.exists():
            raise RuntimeError(f"AFSIM 没写出 jam.evt（{cfg['cid']}）")
        rows = _parse_evt(evt_path.read_text(errors="replace"))
    if not rows:
        raise RuntimeError(f"AFSIM 没采到照射事件（{cfg['cid']}）")
    (out_dir / "afsim").mkdir(parents=True, exist_ok=True)
    (out_dir / "afsim" / f"{cfg['cid']}.txt").write_text(text, encoding="utf-8")
    return rows


def _d_from_rows(rows: list[tuple], s_ref: float) -> float:
    """按 AFSIM 受扰/干净曲线上最接近 60 km 的样本反推标定距离 D（米）。

    与 cmp_sensors._d_from_afsim 同一条单点标定路，但 evt 行已带斜距，
    不必再做几何重建。
    """
    best = min(rows, key=lambda r: abs(r[2] - 60_000.0))
    return best[2] * 10 ** ((best[1] - s_ref) / 40.0)


def _parse_evt(text: str) -> list[tuple]:
    """解析 jam.evt：目标 T 的照射行 → [(t, S/(N+C+J) dB, 斜距 m), ...]。

    记录为反斜杠续行；一条记录的最后一行带 ``S/(N+C+J)``。只取雷达 ``R``
    对目标 ``T`` 的行（``R J`` 是干扰机平台自己的回波，SIG_EMPTY 不污染）。
    """
    rows = []
    cur_t = cur_range = None
    is_tgt = False
    for line in text.splitlines():
        m = re.match(r"^\s*([\d.]+) SENSOR_DETECTION_ATTEMPT R (\S+) ", line)
        if m:
            cur_t, tgt = float(m.group(1)), m.group(2)
            is_tgt = tgt == "T"
            cur_range = None
            continue
        if not is_tgt:
            continue
        m = re.search(r"Range: ([\d.]+) km", line)
        if m:
            cur_range = float(m.group(1)) * 1000.0
            continue
        m = re.search(r"S/\(N\+C\+J\):\s*(-?[\d.]+) dB", line)
        if m and cur_range is not None:
            rows.append((cur_t, float(m.group(1)), cur_range))
            is_tgt = False
    return rows


# ---------------------------------------------------------------------------
# milsim 侧
# ---------------------------------------------------------------------------

def milsim_script(cfg: dict, params: dict) -> str:
    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    slat, slng = frame.world_to_geo(0.0, START_N)
    radar_lines = "\n".join(f"        {k:<26} {v}" for k, v in params.items())
    v = cfg["v"]
    return f"""# 生成：tools/cmp_jam.py · {cfg['cid']}
simulation
    name     "cmp jam {cfg['cid']}"
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
    max_speed       {v:g} m/s
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

platform_type JAMMER_NODE
    side red
    radar_cross_section 0.0001 m2
    component mover CMP_BODY
    end_component
    component jam RF_JAMMER
        peak_power       {cfg['p_j']:g} W
        frequency        3.3 GHz
        bandwidth        {cfg['b_j']:g} Hz
        duty_cycle       1.0
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

platform JAM JAMMER_NODE
    position latlng {slat:.6f} {slng:.6f}
    heading 180 deg
end_platform
"""


def run_milsim(cfg: dict, out_dir: Path, d_ref_m: float):
    """装配并跑 milsim 侧；返回受扰照射记录 [(t, S/(N+J) dB, 斜距), ...]。

    只记录 ``jammer_noise_w > 0`` 且 ``rcs == 目标 RCS`` 的调用——自卫干扰下
    产品路径对每个候选各调一次（干净/受扰两支 + 干扰机自己的回波），按这两条
    筛出"目标的受扰账"，与 AFSIM 的 detAttempt 同位。
    """
    from milsim.simulation import Simulation

    lat0, lng0 = ANCHOR
    frame = LocalFrame(*ANCHOR, 1000)
    params = dict(RADAR_PARAMS)
    params["detect_range"] = f"{d_ref_m:.3f} m"
    # ★ 标定法算 J 账需要天线增益 = 4π/(θ_H·θ_V)——按 §5.14.16 的标定口径
    #   θ_H=θ_V=2.03129° ↔ 40 dB（与 AFSIM LR_ANT peak_gain 40 db 对齐）
    params["beamwidth_h"] = "2.03129 deg"
    params["beamwidth_v"] = "2.03129 deg"
    text = milsim_script(cfg, params)
    (out_dir / "milsim").mkdir(parents=True, exist_ok=True)
    path = out_dir / "milsim" / f"{cfg['cid']}.txt"
    path.write_text(text, encoding="utf-8")

    sim = Simulation.from_scenario_file(path)
    sim.build()
    sim.initialize()
    rid = sim.registry.by_name("R_RADAR").entity_id
    tid = sim.registry.by_name("TGT").entity_id
    jid = sim.registry.by_name("JAM").entity_id
    radar = sim.entities[rid].require("sensor")
    mover = sim.entities[tid].require("mover")
    mover2 = sim.entities[jid].require("mover")
    theta_db = 10.0 * math.log10(float(radar.snr_at_reference))

    from milsim.services.map.hex import offset_to_heading

    ex, ey = frame.geo_to_world(*frame.world_to_geo(0.0, END_N))
    sx, sy = frame.geo_to_world(*frame.world_to_geo(0.0, START_N))
    for ent_id, mv in ((tid, mover), (jid, mover2)):
        sim.store.set_pose(ent_id, sx, sy, cfg["alt"],
                           offset_to_heading(0.0, -1.0), cfg["v"])
        started = mv._started_us
        mv.move_along_route([(ex, ey, cfg["alt"])])
        mv._speed = cfg["v"]
        mv._started_us = started

    records = []
    seen = set()
    orig = radar.single_pulse_snr

    def wrap(range_m, rcs_m2=None, *, jammer_noise_w=0.0, steer_gain=1.0):
        snr = orig(range_m, rcs_m2, jammer_noise_w=jammer_noise_w,
                   steer_gain=steer_gain)
        if jammer_noise_w > 0.0 and rcs_m2 is not None \
                and abs(rcs_m2 - cfg["rcs"]) < 1e-12:
            t_s = sim.engine.now / 1_000_000.0
            key = (round(t_s * 10), round(range_m))
            if key not in seen:
                seen.add(key)
                records.append((t_s, 10.0 * math.log10(snr) if snr > 0.0
                                else -999.0, range_m))
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
        raise RuntimeError(f"milsim 没采到受扰照射（{cfg['cid']}）")
    return records, theta_db


# ---------------------------------------------------------------------------
# 对齐与指标（配对逻辑与 cmp_sensors.compare 同构）
# ---------------------------------------------------------------------------

def compare(cfg, sim_recs, theta_db, ref_rows) -> dict:
    t_end = (START_N - END_N) / cfg["v"] + 1.0
    sim_recs = [r for r in sim_recs if r[0] <= t_end]
    ref = [r for r in ref_rows if r[0] <= t_end]
    pairs = []
    ref_left = list(ref)
    for t, snr, slant in sim_recs:
        best = min(ref_left, key=lambda r: abs(r[0] - t), default=None)
        if best is not None and abs(best[0] - t) <= PAIR_TOL_S:
            pairs.append((t, snr, slant, best[1], best[2]))
            ref_left.remove(best)
    n = len(pairs)
    mask_m = [snr >= theta_db for _t, snr, _s, _sn, _sl in pairs]
    mask_a = [sn >= theta_db for _t, _s, _sl, sn, _sl2 in pairs]
    both = sum(1 for a, b in zip(mask_m, mask_a) if a and b)
    union = sum(1 for a, b in zip(mask_m, mask_a) if a or b)
    jaccard = both / union if union else 1.0
    errs = [a - b for _t, a, _s, b, _sl in pairs]
    rmse = math.sqrt(sum(e * e for e in errs) / n) if n else float("nan")
    denom = sum(abs(a) + abs(b) for _t, a, _s, b, _sl in pairs)
    tic = math.sqrt(sum(e * e for e in errs) / n) / (denom / n) if denom else 0.0

    # 烧穿距离：受扰掩码从远到近首次为真的斜距 = 该侧掩码为真的最大斜距
    burn_m = max((p[2] for m2, p in zip(mask_m, pairs) if m2),
                 default=float("nan"))
    burn_a = max((p[4] for ma, p in zip(mask_a, pairs) if ma),
                 default=float("nan"))

    ok = (math.isnan(rmse) is False and rmse <= PASS_RMSE
          and abs(burn_m - burn_a) <= PASS_BURN)
    return {
        "cid": cfg["cid"], "n_pairs": n, "jaccard": jaccard,
        "snr_rmse_db": rmse, "tic": tic, "theta_db": theta_db,
        "burnthrough_m": burn_m, "burnthrough_a_m": burn_a,
        "burn_diff_m": burn_m - burn_a,
        "verdict": "ok" if ok else "看一眼",
    }


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--only", nargs="*", default=[])
    args = ap.parse_args(argv)

    cfgs = build_configs(args.probe, args.only)
    mission = _mission_exe()
    out_dir = ROOT / "demos" / "afsim_cmp" / "jam"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"mission.exe = {mission}；配置 {len(cfgs)} 个")

    # 第一步：干净场景标定（与域二同一条路）——干净 AFSIM 曲线单点标定
    # detect_range，把干净边界放到 0.7×起点；功率与 D 对 4 个配置共用。
    clean = dict(BASE)
    clean.update({"cid": "calib_clean", "p_j": 0.0, "b_j": 2.0e6,
                  "duration": (START_N - END_N) / BASE["v"] + 20.0})
    _, theta_db = _run_milsim_clean(clean, out_dir, 120_000.0)
    ref_rows = run_afsim(mission, clean, out_dir, 6e6, "clean-calib",
                         with_jammer=False)
    d_m = _d_from_rows(ref_rows, theta_db)
    power_db = 0.0
    if not (0.3 * START_N <= d_m <= 0.85 * START_N):
        power_db = -40.0 * math.log10(d_m / (0.7 * START_N))
        ref_rows = run_afsim(mission, clean, out_dir,
                             6e6 * 10 ** (power_db / 10.0),
                             f"clean-calib P{power_db:+.1f}",
                             with_jammer=False)
        d_m = _d_from_rows(ref_rows, theta_db)
    delta_db = 10.0 * math.log10(BASE["rcs"] / 0.1)
    d_ref = d_m * 10 ** (-delta_db / 40.0)
    power_w = 6e6 * 10 ** (power_db / 10.0)
    print(f"标定：D={d_m/1000:.1f} km  P{power_db:+.1f} dB  θ={theta_db:.3f} dB")

    results = []
    for i, cfg in enumerate(cfgs, 1):
        print(f"[{i}/{len(cfgs)}] jam/{cfg['cid']} ...")
        try:
            records, theta = run_milsim(cfg, out_dir, d_ref)
            ref_rows = run_afsim(mission, cfg, out_dir, power_w)
            row = compare(cfg, records, theta, ref_rows)
            row["power_db"] = power_db
            row["d_ref_m"] = d_ref
            results.append(row)
            print(f"    jac={row['jaccard']:.3f} rmse={row['snr_rmse_db']:.3f} dB"
                  f" tic={row['tic']:.4f} 烧穿 {row['burnthrough_m']/1000:.1f} vs"
                  f" {row['burnthrough_a_m']/1000:.1f} km"
                  f" (差 {row['burn_diff_m']:+.0f} m) → {row['verdict']}")
        except Exception as exc:                                    # noqa: BLE001
            print(f"    失败：{exc}")
            results.append({"cid": cfg["cid"], "verdict": "失败",
                            "error": str(exc)})

    _write_archive(out_dir, results, power_db, d_m)
    ok = sum(1 for r in results if r.get("verdict") == "ok")
    bad = sum(1 for r in results if r.get("verdict") == "失败")
    print(f"完成：{len(results) - bad} 成功 / {bad} 失败；ok {ok}")
    print(f"报告 → {out_dir / 'report.md'}")
    return 0 if bad == 0 else 1


COLS = ["cid", "n_pairs", "jaccard", "snr_rmse_db", "tic", "theta_db",
        "power_db", "d_ref_m", "burnthrough_m", "burnthrough_a_m",
        "burn_diff_m", "verdict"]


def _write_archive(out_dir: Path, results: list[dict], power_db: float,
                   d_m: float) -> None:
    lines = ["\t".join(COLS)]
    for r in results:
        lines.append("\t".join(
            f"{r.get(c, ''):.6g}" if isinstance(r.get(c), float) else str(r.get(c, ""))
            for c in COLS))
    (out_dir / "summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

    rep = [
        "# 域三 · 干扰下 SNR(t) 时序对拍报告",
        "",
        "- 口径：自卫干扰（干扰机与目标同址编队，恒在主瓣内）；雷达/标定与域二同源",
        f"  （干净边界 D={d_m/1000:.1f} km，功率偏移 {power_db:+.1f} dB）；受扰判读线",
        "  = 同一条 θ（Pd=0.5）；烧穿 = 受扰掩码从远到近首次为真的斜距。",
        "- 判读线：受扰 SNR RMSE ≤ 0.5 dB、烧穿距离差 ≤ 2 km 记 ok。",
        "",
        "| 配置 | 配对 | Jaccard | RMSE dB | TIC | 烧穿 milsim km | 烧穿 AFSIM km | 差 m | 判 |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        if r.get("verdict") == "失败":
            rep.append(f"| {r['cid']} | - | - | - | - | - | - | - | 失败：{r.get('error', '')[:40]} |")
            continue
        rep.append(
            f"| {r['cid']} | {r['n_pairs']} | {r['jaccard']:.3f} |"
            f" {r['snr_rmse_db']:.3f} | {r['tic']:.4f} |"
            f" {r['burnthrough_m']/1000:.1f} | {r['burnthrough_a_m']/1000:.1f} |"
            f" {r['burn_diff_m']:+.0f} | {r['verdict']} |")
    rep += [
        "",
        "指标明细 → `summary.csv`；两侧想定存档在 `afsim/` 与 `milsim/`。",
        "",
        "## 口径注记",
        "",
        "- 自卫干扰下 J ∝ 1/r²、S ∝ 1/r⁴ ⇒ S/(N+J) 随接近单调抬升，受扰边界即烧穿点；",
        "- 跳频两端点（跟随=瞄准式 / 阻塞盖满=宽阻塞）的 J 账已由 §5.14.16 静态",
        "  对拍钉死（gap≤0.109 dB），动态时序不重复——干扰机逐位老口径只进 J 账。",
        "- ★ 受扰 SNR 必须读 event_output 的 SENSOR_DETECTION_ATTEMPT 行（带",
        "  S/(N+C+J) 与 Range）；observer 的 SignalToNoise() 是**无扰** S/(N+C)，",
        "  直接拿它对拍会得出 60 dB 级假差。",
        "- 功率刻度：自卫干扰 J∝1/r²、S∝1/r⁴，90 km 起步航路里 S/J 至多抬",
        "  24 dB——100 W 干扰机全程压制（烧穿恒 nan 是物理不是 bug）；烧穿落进",
        "  航路须把 P_j 缩到 mW 级（功率线性已由静态对拍钉死，对拍口径不变）。",
        "- ★ 残差结构：远场（≥25 km）J 账逐位一致（|err|≤0.25 dB）；近距偏差",
        "  随仰角增长（13 km/23° 处 −0.95 dB、7.5 km/45° 处 −2.66 dB），量级与",
        "  \"AFSIM 对干扰单程路径收端增益记 EBS 损失 cos²(el)、milsim 的",
        "  jamming_effect 未记\" 相符——真实模型口径差（10-16 静态对拍时干扰机",
        "  在主瓣视线上且仰角固定，差被几何吸收），**milsim 零改动**，是否给",
        "  jamming_effect 补收端 EBS 留拍板（§9 候选）。",
    ]
    (out_dir / "report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
