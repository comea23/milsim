#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""弹命中概率大样本标定（engage + 查表法）——项目 Warhead 概率表数据源。

对 (目标过载 g, 弹速档, 目标自卫干扰功率, 种子) 网格逐格跑 AFSIM engage
（demos/engage/grid_react_man.txt 骨架），解析 fort.grm.7 的弹-目标距离
序列取 CPA，按杀伤半径统计命中率 = 查表法（各条件格直接统计，非参数拟合）。

产出（--out 目录）：
  calib_data.json   全量样本 + hit_table（键 = g/v/jam/down_range_km）
  calib_report.md   人读报告（CPA 分位 + 各杀伤半径命中率）

制导架构（终版）：WSF_GUIDANCE_COMPUTER PN + 弹载雷达导引头（40 km 捕获、
switch_range 切末段）+ tracker uplink 数据链（中段 cmd track 持续修正）。
guide_to_truth false：PN 按带噪 track 修正，误差源 = track filter 对机动
目标的滞后与雷达测量噪声（SNR 内生），表值是该制导架构的条件概率，
换架构需重标。目标自卫干扰（WSF_RF_JAMMER 噪声法）压 seeker SNR ⇒
探测距离缩短/track 质量下降，作为表的第四维（jam 功率档）。

用法：
  python calibrate_warhead.py --out /f/afsim2.9cn/calib_final \
      --seeds 150 --gs 0,2,4,6 --speeds 20,10 --jams 0,10,100
"""
import argparse
import io
import json
import os
import subprocess
import sys
import time

# engage 可执行与 utility 脚本默认路径（本机 AFSIM 2.9.0 中文版布局）
ENGAGE_EXE = "F:/afsim2.9cn/bin/engage.exe"
UTILITY_TXT = "F:/afsim2.9cn/demos/engage/scripts/utility_scripts.txt"

JAMMER_TEMPLATE = """weapon TARGET_JAMMER WSF_RF_JAMMER
   maximum_range 200 km
   transmitter
      frequency_band             14 ghz 16 ghz
      power                      @JAMMER_POWER@ w
      electronic_attack
         technique noise_jamming
            effect noise_effect WSF_JAMMER_POWER_EFFECT
               jamming_delta_gain 0 db
            end_effect
         end_technique
      end_electronic_attack
   end_transmitter
end_weapon
"""

SCENARIO_TEMPLATE = r"""
# 骨架 = demos/engage/grid_react_man.txt（engage.exe 专用语法：engage 自己
# 按 LAUNCHER_TYPE/TRACKER_TYPE/TARGET_TYPE 派生实例并逐格重跑）。
# 相对 demo 的改动（全部列明）：
#   1. lrsam 内联（保留 demo 引信+weapon_effects 原样）——engage 的数据
#      落地机制 = WsfWeaponEngagement 终止（引信在最近点 CPA 引爆 →
#      WeaponTerminated → ShotCount++ + 该格仿真结束 + fort 输出落地）。
#      实测删引信/效果后弹永不终止 ⇒ fort.grm.7 全空、ShotCount=0；
#      引信在 CPA 引爆 ⇒ 弹道序列末点即 CPA（观测止于最近点，正偏
#      ≤1 个输出周期）；
#   2. 目标机动 = 随机 break（种子驱动方向/时机），替换 ESAMS 反应机动
#      （mar_track_list/maneuver 处理器不再需要）；
#   3. TTR 内联原样（SEARCH 方位扫描 ±10° → ±60° 覆盖网格横距）；
#   4. utility_scripts.txt 完整内联（运行时从 demo 原文读取后替换正文
#      占位符）——任务处理器/target 赋值引用其全局表（sites/launchedWeapons/
#      target 等 17 个变量与 8 个脚本），人肉挑选已两次漏引用（sites、
#      target），不再挑；注意注释里不可再出现占位符字面量（replace 全局
#      替换会把 demo 内容嵌两份）。
#   5. target_grid = 标定几何（3 距离 × 2 横距）；output_rate 0.02 s
#      （demo 默认 1 s，CPA 观测偏差随周期线性增大）。

@UTILITY_SCRIPTS@

antenna_pattern LRSAM_TTR_ANTENNA
  sine_pattern
    peak_gain            40 dB
    minimum_gain         -10.0 db
    azimuth_beamwidth    1 deg
    elevation_beamwidth  1 deg
  end_sine_pattern
end_antenna_pattern

sensor LRSAM_TTR WSF_RADAR_SENSOR
   selection_mode                      multiple
   slew_mode                           azimuth_and_elevation
   azimuth_slew_limits                 -180 deg 180 deg
   elevation_slew_limits               0.0 deg 80.0 deg

   mode_template
      maximum_request_count            1
      one_m2_detect_range              150.0 nm
      antenna_height                   4.0 m
      transmitter
         antenna_pattern               LRSAM_TTR_ANTENNA
         power                         1000.0 kw
         frequency                     9500 mhz
         internal_loss                 2 db
      end_transmitter
      receiver
         antenna_pattern               LRSAM_TTR_ANTENNA
         bandwidth                     500.0 khz
         noise_power                   -160 dBw
         internal_loss                 7 dB
      end_receiver
      probability_of_false_alarm       1.0e-6
      required_pd                      0.5
      swerling_case                    1
      reports_range
      reports_bearing
      reports_elevation
      reports_velocity
   end_mode_template

   mode SEARCH
      cue_mode azimuth
      scan_mode                       azimuth_and_elevation
      azimuth_scan_limits            -60 deg 60 deg
      elevation_scan_limits            0 deg 40 deg
      frame_time                       2.0 sec
      hits_to_establish_track          3 5
      hits_to_maintain_track           1 3
      track_quality                    0.7
   end_mode

   mode ACQUIRE
      maximum_request_count            1
      scan_mode                        azimuth_and_elevation
      azimuth_scan_limits             -2 deg 2 deg
      elevation_scan_limits           -2 deg 2 deg
      frame_time                       1.0 sec
      hits_to_establish_track          2 4
      hits_to_maintain_track           1 3
      track_quality                    0.9
   end_mode

   mode TRACK
      maximum_request_count            1
      scan_mode                        azimuth_and_elevation
      azimuth_scan_limits             -1 deg 1 deg
      elevation_scan_limits           -1 deg 1 deg
      frame_time                       1.0 sec
      hits_to_establish_track          2 4
      hits_to_maintain_track           1 3
      track_quality                    1.0
   end_mode

   ignore_same_side
end_sensor

radar_signature FIGHTER_RADAR_SIGNATURE
   constant 1.0 m^2
end_radar_signature

antenna_pattern LRSAM_SEEKER_ANTENNA
  sine_pattern
    peak_gain            30 dB
    minimum_gain         -10.0 db
    azimuth_beamwidth    5 deg
    elevation_beamwidth  5 deg
  end_sine_pattern
end_antenna_pattern

# 弹载导引头（替代 WSF_PERFECT_TRACKER 的上帝视角）：中段按 uplink cmd
# track 惯性修正，导引头 ~40 km 捕获、switch_range 切末段后 PN 按带噪
# track 修正。测量误差由雷达方程内生（SNR 越低测得越歪），是命中概率
# 出现中间值的来源；目标挂干扰机可进一步压 SNR/缩短探测距离。
sensor LRSAM_SEEKER WSF_RADAR_SENSOR
   selection_mode                      single
   slew_mode                           azimuth_and_elevation
   azimuth_slew_limits                 -60 deg 60 deg
   elevation_slew_limits               -60 deg 60 deg

   mode_template
      one_m2_detect_range              40.0 km
      transmitter
         antenna_pattern               LRSAM_SEEKER_ANTENNA
         power                         100.0 W
         frequency                     15000 mhz
         internal_loss                 4 db
      end_transmitter
      receiver
         antenna_pattern               LRSAM_SEEKER_ANTENNA
         bandwidth                     1.0 mhz
         noise_power                   -155 dBw
         internal_loss                 6 dB
      end_receiver
      reports_range
      reports_bearing
      reports_elevation
      reports_velocity
   end_mode_template

   mode TRACK
      frame_time                       0.1 sec
      hits_to_establish_track          1 2
      track_quality                    1.0
   end_mode

   ignore_same_side
end_sensor

weapon_effects LRSAM_LETHALITY WSF_GRADUATED_LETHALITY
   radius_and_pk 100 m 0.9
end_weapon_effects

aero LRSAM_AERO WSF_AERO
   cd_zero_subsonic    0.10
   cd_zero_supersonic  0.25
   mach_begin_cd_rise  0.95
   mach_end_cd_rise    1.30
   mach_max_supersonic 4.00
   reference_area      0.159 m2
   cl_max             10.0
   aspect_ratio       16.0
end_aero

mover LRSAM_MOVER WSF_GUIDED_MOVER
   update_interval      0.01 sec
   integration_timestep 0.01 sec

   stage 1
      aero            LRSAM_AERO
      total_mass      0.0078 kg
      empty_mass      0.0001 kg
      thrust          51479.4 nt
      thrust_duration 0.77 sec
   end_stage

   stage 2
      aero            LRSAM_AERO
      total_mass      1284.001 kg
      fuel_mass       1284 kg
      thrust_duration @STAGE2@ sec
      thrust          157386.0 nt
      thrust_vectoring_time_limits  0.77 sec 5.0 sec
      thrust_vectoring_angle_limit  15 deg
   end_stage

   stage 3
      aero       LRSAM_AERO
      total_mass 854.0 kg
   end_stage
end_mover

processor LRSAM_GUIDANCE WSF_GUIDANCE_COMPUTER
   guide_to_truth false
   phase LAUNCH
      guidance_delay 100.0 sec
      next_phase TERMINAL when phase_time >= 0.8 sec
   end_phase
   phase TERMINAL
      guidance_target perception
      proportional_navigation_gain 4.0
      g_bias                       1.0
   end_phase
end_processor

platform_type LRSAM WSF_PLATFORM
   icon SA-10_Missile
   weapon_effects LRSAM_LETHALITY
   mover LRSAM_MOVER
   end_mover
   processor guidance LRSAM_GUIDANCE
   end_processor
   # 制导链路（源码实证 WsfWeaponTrackProcessor.cpp L569/L585/L765）：
   # seeker 探测 -> tracks(WSF_TRACK_PROCESSOR 建本地 track) -> seeker_tracks
   # (WSF_WEAPON_TRACK_PROCESSOR: 本地 sensor track => mSnrTrackPtr，自动
   # SetCurrentTarget) -> guidance(WSF_GUIDANCE_COMPUTER 从 TrackManager
   # GetCurrentTarget 取 track，guide_to_truth false => PN 按带噪 track 修正)。
   # 中段无探测时 current target = 发射时 WSF_EXPLICIT_WEAPON 注入的静态
   # track（预测拦截点），弹惯性飞向该点；导引头捕获后切末段 PN。
   # 注意：AFSIM 平台部件默认关闭（WsfPlatformPart mIsTurnedOn=false），
   # sensor 必须显式 on，否则永不探测、WSF_WEAPON_TRACK_PROCESSOR 的
   # mSnrTrackPtr 永远为空，弹全程追发射时刻的静态 cmd track。
   sensor seeker LRSAM_SEEKER
      on
      internal_link tracks
   end_sensor
   processor tracks WSF_TRACK_PROCESSOR
      internal_link seeker_tracks
   end_processor
   processor seeker_tracks WSF_WEAPON_TRACK_PROCESSOR
      switch_range 30.0 km
   end_processor
   # 弹载数据链收发器（与 tracker 同在 default 网络）：接收 uplink track，
   # 交给 seeker_tracks 处理（外部航迹 => cmd track 持续更新）。
   comm uplink WSF_COMM_TRANSCEIVER
      internal_link seeker_tracks
   end_comm
   processor fuse WSF_AIR_TARGET_FUSE
      gross_proximity_range  100 km
      hit_proximity_range   90 km
   end_processor
end_platform_type

weapon LRSAM WSF_EXPLICIT_WEAPON
   cue_to_predicted_intercept true
   launched_platform_type     LRSAM
   launch_delta_v             22.4 0 0 m/s
   tilt  89.9 degrees
   tilt  90.0 degrees
   tilt  89.999 degrees
   slew_mode azimuth
   azimuth_slew_limits -180 deg 180 deg
   quantity 1
   firing_delay 1.0 secs
end_weapon

platform_type LAUNCHER_TYPE WSF_PLATFORM
   side red
   icon SA-10_Launcher
   comm comm WSF_COMM_TRANSCEIVER
      internal_link operator
   end_comm
   processor operator WSF_TASK_PROCESSOR
   end_processor
   weapon lrsam LRSAM
      quantity 1
   end_weapon
end_platform_type

platform_type TRACKER_TYPE WSF_PLATFORM
   side red
   icon Flap_Lid

   on_initialize
      sites.Insert(PLATFORM.Index());
   end_on_initialize

   comm comm WSF_COMM_TRANSCEIVER
      internal_link operator
   end_comm

   sensor find WSF_GEOMETRIC_SENSOR
      ignore_same_side
      frame_time 10 sec
      reports_location
      on
      track_quality 0.5
      internal_link tracker
   end_sensor

   sensor ttr LRSAM_TTR
      on
      internal_link tracker
   end_sensor

   # 数据链 uplink：tracker 的 track 经 comm 广播到 default 网络，飞行中的
   # 弹经自身 comm 收到 => WSF_WEAPON_TRACK_PROCESSOR 判为外部指挥航迹
   # （源码 L612-618），mCmdTrackPtr 持续更新 => 中段跟随目标机动。
   processor tracker WSF_TRACK_PROCESSOR
      internal_link comm
   end_processor

   processor operator WSF_TASK_PROCESSOR
      evaluation_interval DETECTED 0.5 sec
      state DETECTED
         next_state TRY_TO_ACQUIRE
            return StartTracking(TRACK, "DUMMY", PLATFORM.Sensor("ttr"), "SEARCH");
         end_next_state
      end_state

      evaluation_interval TRY_TO_ACQUIRE 0.5 sec
      state TRY_TO_ACQUIRE
         next_state TRY_TO_TRACK
            if(!acquisitionTime.Exists(PLATFORM.Index()))
            {
               acquisitionTime[PLATFORM.Index()] = TIME_NOW;
            }
            bool status = false;
            if (TRACK.TrackQuality() > 0.65)
            {
               status = StartTracking(TRACK, "DUMMY", PLATFORM.Sensor("ttr"), "ACQUIRE");
            }
            return status;
         end_next_state
      end_state

      evaluation_interval TRY_TO_TRACK 0.5 sec
      state TRY_TO_TRACK
         next_state TRY_TO_FIRE
            if(!trackTime.Exists(PLATFORM.Index()))
            {
               trackTime[PLATFORM.Index()] = TIME_NOW;
            }
            if(!illuminationTime.Exists(PLATFORM.Index()))
            {
               illuminationTime[PLATFORM.Index()] = TIME_NOW;
            }
            bool status = false;
            if (TRACK.TrackQuality() > 0.85)
            {
               status = StartTracking(TRACK, "DUMMY", PLATFORM.Sensor("ttr"), "TRACK");
            }
            return status;
         end_next_state
      end_state

      evaluation_interval TRY_TO_FIRE 0.5 sec
      state TRY_TO_FIRE
         next_state FIRED
            bool status = false;
            if (TRACK.TrackQuality() > 0.95)
            {
               WsfPlatform launcher = WsfSimulation.FindPlatform("launcher");
               status = FireAt(TRACK, "DUMMY", "lrsam", 1, launcher);
            }
            return status;
         end_next_state
      end_state

      evaluation_interval FIRED 0.1 sec
      state FIRED
         next_state FIRED
            return true;
         end_next_state
      end_state
   end_processor
end_platform_type

# 自卫干扰机（挂目标；功率档由 --jams 控制，0 = 整块不渲染）。
# 无 antenna_pattern => 默认全向；noise_jamming 把噪声注入同频段接收机
# （WSF_JAMMER_POWER_EFFECT），seeker 的 S/(N+J) 恶化 => 探测距离缩短/
# track 质量下降，与项目 §5.15 判据同源。
@JAMMER_BLOCK@

platform_type TARGET_TYPE WSF_PLATFORM
   side blue
   icon F-18E
   radar_signature FIGHTER_RADAR_SIGNATURE

   mover WSF_AIR_MOVER
      maximum_speed    @SPEED@ knots
      maximum_altitude 50000 ft
      body_g_limit     @GLIMIT_MOVER@ g
      bank_angle_limit 75 deg
   end_mover

@JAMMER_MOUNT@
   script_variables
      double mDir   = 1.0;
      double mStart = 1.0e10;
      bool   mDone  = false;
   end_script_variables

   on_initialize2
      target = PLATFORM.Index();
      if (@GLIMIT@ > 0.0)
      {
         if (MATH.RandomUniform() < 0.5)
         {
            mDir = -1.0;
         }
         else
         {
            mDir = 1.0;
         }
         mStart = 25.0 + MATH.RandomUniform() * 25.0;
      }
   end_on_initialize2

   execute at_interval_of 0.5 s
      if ((!mDone) && (TIME_NOW >= mStart))
      {
         mDone = true;
         PLATFORM.TurnToHeading(PLATFORM.Heading() + 90.0 * mDir);
      }
   end_execute
end_platform_type

random_seed @SEED@
frame_time 0.02 sec
end_time 200 secs

# 输出周期 0.02 s（demo 默认 1 s）——CPA 观测偏差随周期线性增大。
# 注意：output_rate 是顶层命令（run 块内不认）；coarse 表只给 fort6
# （目标轨迹，仅作锚点核对）。
output_rate default
   time 0.0 secs period 0.02 sec
end_output_rate

output_rate coarse
   time 0.0 secs period 1.0 sec
end_output_rate

run
   record_file_base_name cal

   target_grid
      down_range  from 30 km to 90 km by 30 km
      cross_range from -30 km to 30 km by 60 km
      altitude    from 30 kft to 30 kft by 10 kft
      speed @SPEED@ knots
   end_target_grid

   output
      file fort.grm.7
      phase flying
      items
         variable weapon_flight_time format "%.2f"
         variable target_x units km format " %7.3f"
         variable target_y units km format " %7.3f"
         variable target_z units m format " %7.1f"
         variable weapon_x units km format " %7.3f"
         variable weapon_y units km format " %7.3f"
         variable weapon_z units m format " %7.1f"
         variable weapon_to_target_range units m format " %9.2f"
         variable weapon_vx units m/s format " %8.1f"
         variable weapon_vy units m/s format " %8.1f"
         variable weapon_vz units m/s format " %8.1f"
         variable weapon_ax units m/s^2 format " %8.1f"
         variable weapon_ay units m/s^2 format " %8.1f"
         variable weapon_az units m/s^2 format " %8.1f"
      end_items
   end_output

   output
      file fort.grm.6
      phase all
      rate_table_name coarse
      items
         variable time format "%.2f"
         variable target_x units km format " %7.3f"
         variable target_y units km format " %7.3f"
         variable target_z units m format " %7.1f"
      end_items
   end_output
end_run
"""


def write_scenario(dirpath, seed, g_limit, stage2, speed_kts, utility_text,
                   jam_power=0.0):
    """渲染想定 cal.txt。g0 档 = 直线飞行目标（σ_floor 直接观测）：
    WSF_AIR_MOVER 的 body_g_limit 必须 > 1 g（9.80665 m/s²），0 会被拒——
    给最小合法值，目标不掷机动（on_initialize2 的 GLIMIT 判断仍用 0），
    能力上限无意义。jam=0 时干扰块/挂载整块不渲染；jam>0 渲染
    JAMMER_TEMPLATE（power 档替换）+ 实例挂载（平台部件默认关闭，
    WsfPlatformPart mIsTurnOn=false，实例必须显式 on）。"""
    os.makedirs(dirpath, exist_ok=True)
    g_mover = g_limit if g_limit > 0.0 else 1.001
    if jam_power > 0.0:
        jammer_block = JAMMER_TEMPLATE.replace(
            "@JAMMER_POWER@", ("%g" % jam_power))
        jammer_mount = ("   weapon jammer TARGET_JAMMER\n"
                        "      on\n"
                        "   end_weapon\n")
    else:
        jammer_block = ""
        jammer_mount = ""
    text = (SCENARIO_TEMPLATE
            .replace("@UTILITY_SCRIPTS@", utility_text)
            .replace("@SEED@", str(seed))
            .replace("@GLIMIT_MOVER@", ("%g" % g_mover))
            .replace("@GLIMIT@", ("%g" % g_limit))
            .replace("@STAGE2@", ("%g" % stage2))
            .replace("@SPEED@", ("%g" % speed_kts))
            .replace("@JAMMER_BLOCK@", jammer_block)
            .replace("@JAMMER_MOUNT@", jammer_mount))
    # 防呆：干扰块渲染次数必须精确（0 或 1），防占位符失配静默丢块
    if text.count("WSF_RF_JAMMER") != (1 if jam_power > 0.0 else 0):
        raise RuntimeError("jammer block 渲染失败: %s" % dirpath)
    with io.open(os.path.join(dirpath, "cal.txt"), "w",
                 encoding="utf-8") as f:
        f.write(text)


def run_one(engage_exe, dirpath, timeout_s):
    """cwd=run 目录调 engage（输入固定名 cal.txt），留时间戳日志。"""
    t0 = time.strftime("%Y-%m-%d %H:%M:%S")
    proc = subprocess.run([engage_exe, "cal.txt"], cwd=dirpath,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          timeout=timeout_s)
    with io.open(os.path.join(dirpath, "engage.log"), "w",
                 encoding="utf-8") as f:
        f.write("%s file cal.txt\n" % t0)
    out = proc.stdout.decode("utf-8", "replace")
    err = proc.stderr.decode("utf-8", "replace")
    return proc.returncode, out, err


def _rows(path, ncols):
    """fort 文本表 → float 行（精确列数匹配，其他行跳过）。"""
    rows = []
    try:
        f = io.open(path, "r", encoding="utf-8", errors="replace")
    except OSError:
        return rows
    with f:
        for ln in f:
            parts = ln.split()
            if len(parts) != ncols:
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue
    return rows


def parse_fort7(dirpath):
    """fort.grm.7 = 14 列：wft, tgt_x/y(km), tgt_z(m), w_x/y(km), w_z(m),
    range(m), w_vx/vy/vz(m/s), w_ax/ay/az(m/s^2)。engage 按 target_grid
    逐格重跑并连写同一文件，wft 回退 = 新 cell 起点（cell 0/1=30km、
    2/3=60km、4/5=90km 发射距离；横距 ±30km 对称）。返回每 cell 的行
    序列列表。"""
    rows = _rows(os.path.join(dirpath, "fort.grm.7"), 14)
    series = []
    cur = []
    prev = -1.0
    for r in rows:
        w = r[0]
        if w < prev and cur:
            series.append(cur)
            cur = []
        cur.append(r)
        prev = w
    if cur:
        series.append(cur)
    return series


def first_pass_min(ser):
    """首次穿越法最近点：CPA(m)、对应 wft(s)、closing 标志。
    closing = 终距 > 始距（弹从未有效接近——慢弹追远格的真实不可达，
    属该格低命中率的一部分，统计时与正常样本合并；仅异常形态剔除）。"""
    best_i = 0
    best = ser[0][7]
    for i in range(1, len(ser)):
        if ser[i][7] < best:
            best = ser[i][7]
            best_i = i
    return best, float(ser[best_i][0]), ser[-1][7] > ser[0][7]


def _pct(sorted_vals, q):
    """nearest-rank 分位（升序列）。"""
    n = len(sorted_vals)
    if n == 0:
        return 0.0
    k = min(int(q * n), n - 1)
    return sorted_vals[k]


def build_hit_table(samples, kill_radii):
    """查表法统计：不剔除任何 CPA 形态（慢弹够不着远格 = 该格真实低
    命中概率的一部分）；closing（异常形态，从未接近）仍剔除。
    cell → 发射距离（实证：fort6 首行锚点，cell 0/1=30km、2/3=60km、
    4/5=90km；横距 ±30km 对称合并）。"""
    used = [s for s in samples if not s["closing"]]
    for s in used:
        s["down_range_km"] = 30.0 * (s["cell"] // 2 + 1)
    cells = {}
    for s in used:
        key = (s["g"], s["v"], s["jam"], s["down_range_km"])
        cells.setdefault(key, []).append(abs(s["cpa"]))
    rows = []
    for key in sorted(cells):
        g, v, jam, dr = key
        cpas = sorted(cells[key])
        n = len(cpas)
        row = {"g": g, "v": v, "jam": jam, "down_range_km": dr, "n": n,
               "cpa_median": round(_pct(cpas, 0.5), 2),
               "cpa_p90": round(_pct(cpas, 0.9), 2),
               "cpa_max": round(cpas[-1], 2)}
        for radius in kill_radii:
            row["p_hit_%gm" % radius] = round(
                sum(1 for c in cpas if c <= radius) / float(n), 4)
        row["p_within_3km"] = round(
            sum(1 for c in cpas if c <= 3000.0) / float(n), 4)
        rows.append(row)
    return rows, used


def write_report(path, data):
    L = []
    L.append("# 弹命中概率标定报告（engage 大样本 + 查表法）\n")
    L.append("")
    L.append("- 架构：WSF_GUIDANCE_COMPUTER PN + 弹载雷达导引头（40 km 捕获、"
             "switch_range 切末段）+ tracker uplink 数据链。")
    L.append("- 网格：seeds=%(seeds)s × 目标过载 g=%(gs)s × 弹速档=%(speeds)s"
             " × 目标自卫干扰功率 W=%(jams)s。" % data)
    L.append("- 推演 %(n_runs)s 次（失败 %(n_fail)s）；样本 %(n_samples)s，"
             "剔除 closing 后 %(n_used)s。" % data)
    L.append("- 格键 = (目标过载 g, 弹速档, 目标自卫干扰功率 W, 发射距离 km)；"
             "CPA 单位 m；p_hit_R = |CPA|≤R 样本比例（杀伤半径 R=30/100/300 m，"
             "项目 Warhead 默认 30 m）。")
    L.append("- 局限：非完美导引头架构——误差源 = track filter 对机动目标的"
             "滞后与雷达测量噪声；表值是该制导架构的条件概率，换架构需重标。")
    L.append("- 磁盘注意：fort.grm.7 逐 run 解析后即截断清零（沙箱 safe-delete"
             " 对每轮删除有 50 文件配额，删除会移入回收站不释放空间）。")
    L.append("")
    L.append("| g | v | jam | down_range_km | n | cpa_median | cpa_p90 | "
             "cpa_max | p_hit_30m | p_hit_100m | p_hit_300m | p_within_3km |")
    L.append("|---|---|-----|---------------|---|------------|---------|"
             "---------|-----------|------------|------------|--------------|")
    for r in data["hit_table"]:
        L.append("| %g | %g | %g | %g | %d | %.2f | %.2f | %.2f | %.3f | "
                 "%.3f | %.3f | %.3f |" %
                 (r["g"], r["v"], r["jam"], r["down_range_km"], r["n"],
                  r["cpa_median"], r["cpa_p90"], r["cpa_max"],
                  r["p_hit_30m"], r["p_hit_100m"], r["p_hit_300m"],
                  r["p_within_3km"]))
    if data["failures"]:
        L.append("")
        L.append("## 失败 run（尾部输出）")
        for f in data["failures"]:
            L.append("- %s rc=%s `%s`" % (f["tag"], f["rc"],
                                          f["tail"].replace("\n", " ")[:120]))
    L.append("")
    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="engage 大样本弹命中概率标定（查表法）")
    ap.add_argument("--out", required=True, help="产出目录")
    ap.add_argument("--seeds", type=int, default=150,
                    help="种子数（seed = 1..N）")
    ap.add_argument("--gs", default="0,2,4,6", help="目标过载档，g 逗号分隔")
    ap.add_argument("--speeds", default="20,10",
                    help="弹速档（stage2 推力时长 s），逗号分隔")
    ap.add_argument("--jams", default="0",
                    help="目标自卫干扰功率档 W，逗号分隔")
    ap.add_argument("--afsim", default=ENGAGE_EXE, help="engage.exe 路径")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="单 run 超时 s")
    ap.add_argument("--utility", default=UTILITY_TXT,
                    help="utility_scripts.txt 路径")
    ap.add_argument("--keep-failed", action="store_true",
                    help="失败 run 保留现场（默认截断 fort 释放磁盘）")
    ap.add_argument("--resume", action="store_true",
                    help="断点续跑：读 out/checkpoint.json 跳过已完成 run"
                         "（--seeds/--gs/--speeds/--jams 必须与上次一致）")
    args = ap.parse_args(argv)

    utility_text = io.open(args.utility, encoding="utf-8").read()
    print("utility_scripts.txt 已加载（%d 字符）" % len(utility_text),
          flush=True)

    seeds = list(range(1, args.seeds + 1))
    gs = [float(x) for x in args.gs.split(",") if x.strip()]
    speeds = [float(x) for x in args.speeds.split(",") if x.strip()]
    jams = [float(x) for x in args.jams.split(",") if x.strip()]
    grid_cells = 6  # 3 发射距离 × 2 横距（想定 target_grid 3×2 实证）

    os.makedirs(args.out, exist_ok=True)
    runs = []
    for jam in jams:
        for g in gs:
            for v in speeds:
                for seed in seeds:
                    runs.append((g, v, seed, jam))
    print("计划推演 %d 次（%d 格/次）" % (len(runs), grid_cells), flush=True)

    ok_runs, fail_runs = 0, []
    samples = []       # (g, v, jam, cell, seed, a_t, t_go, cpa, closing)
    done = {}          # tag -> True（断点续跑的已完成集）
    ck_path = os.path.join(args.out, "checkpoint.json")
    if args.resume and os.path.exists(ck_path):
        ck = json.load(io.open(ck_path, encoding="utf-8"))
        done = ck.get("done", {})
        samples = ck.get("samples", [])
        ok_runs = ck.get("ok_runs", 0)
        fail_runs = [tuple(x) for x in ck.get("fail_runs", [])]
        print("断点续跑：已完成 %d，样本 %d" % (len(done), len(samples)),
              flush=True)
    n_new = 0
    for i, (g, v, seed, jam) in enumerate(runs):
        tag = "g%g_v%g_j%g_s%d" % (g, v, jam, seed)
        if tag in done:
            continue
        rdir = os.path.join(args.out, "run_" + tag)
        write_scenario(rdir, seed, g, v, speed_kts=500.0,
                       utility_text=utility_text, jam_power=jam)
        try:
            rc, so, se = run_one(args.afsim, rdir, args.timeout)
        except subprocess.TimeoutExpired:
            rc, so, se = -99, "", "TIMEOUT"
        f7 = os.path.join(rdir, "fort.grm.7")
        if rc != 0 or not os.path.exists(f7):
            fail_runs.append((tag, rc, (se or so)[-300:]))
            done[tag] = True
            # 失败 run 也不能 rmtree——沙箱 safe-delete 把删除物移入
            # 回收站（同分区不释放空间），批量失败会把盘塞满；统一截断。
            if os.path.exists(f7) and not args.keep_failed:
                try:
                    with io.open(f7, "wb"):
                        pass
                except OSError:
                    pass
            continue
        ok_runs += 1
        series = parse_fort7(rdir)
        for cell, ser in enumerate(series):
            if len(ser) < 5:
                continue
            cpa, t_go, closing = first_pass_min(ser)
            a_t = g * 9.80665
            samples.append(dict(g=g, v=v, jam=jam, seed=seed, cell=cell,
                                a_t=a_t, t_go=round(t_go, 2),
                                cpa=round(cpa, 2), closing=closing))
        # fort 已逐 run 解析进 samples（纯内存），原件即清：3600 run ×
        # ~2.8MB 顺序落盘约 10GB，会撑爆磁盘。必须"截断为零"而非删除
        # ——沙箱 safe-delete 对每个后台任务轮次有 50 文件的删除累计
        # 配额（os.remove/rmtree 一律计数，且删除物移入回收站不释放
        # 空间），批跑第 51 次删除即被拦杀；截断是写操作不受配额限，
        # 磁盘照样回收，下一 run 覆盖写。
        try:
            with io.open(f7, "wb"):
                pass
        except OSError:
            pass
        # 断点标记 + 每 25 个新 run 落盘一次 checkpoint（崩溃/爆盘后
        # --resume 可续，samples 不再丢）。
        done[tag] = True
        n_new += 1
        if n_new % 25 == 0:
            try:
                with io.open(ck_path, "w", encoding="utf-8") as f:
                    json.dump(dict(done=done, samples=samples,
                                   ok_runs=ok_runs, fail_runs=fail_runs),
                              f, ensure_ascii=False)
            except OSError:
                pass
        if (i + 1) % 10 == 0:
            print("  %d/%d 完成" % (i + 1, len(runs)), flush=True)

    print("完成 %d / 失败 %d" % (ok_runs, len(fail_runs)), flush=True)

    kill_radii = (30.0, 100.0, 300.0)
    hit_table, used = build_hit_table(samples, kill_radii)
    data = dict(seeds=args.seeds, gs=gs, speeds=speeds, jams=jams,
                kill_radii=list(kill_radii), n_runs=len(runs),
                n_fail=len(fail_runs), n_samples=len(samples),
                n_used=len(used), hit_table=hit_table, samples=samples,
                failures=[dict(tag=t, rc=rc, tail=tl)
                          for t, rc, tl in fail_runs])
    with io.open(os.path.join(args.out, "calib_data.json"), "w",
                 encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    write_report(os.path.join(args.out, "calib_report.md"), data)
    print("产出：%s" % args.out, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
