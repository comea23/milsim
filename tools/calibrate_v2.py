#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""弹命中概率标定 v2（engage + 查表法）——3km 锚点口径。

相对 calibrate_warhead.py（v1）的变化（2026-10-09 拍板）：
  1. 主表 4 维：弹过载 g_m=[0,1,2,3,4,5,6,10,20] × 弹末速 v_m=[1..10 km/s]
     × 目标过载 g_t=同弹侧 × 目标速度 v_t=[0.2,0.6,1..10 km/s]。
     发射距离是想定几何（按弹速/目标速推导，不进表），判定锚点 =
     弹目距离首破 3 km 时刻（本帧>3km 且下帧<3km 线性插值），p_hit =
     全部交战样本中 |CPA|<=R 的比例（够不着 = miss，不剔除）。
  2. 干扰与 RCS 不进主表：单独基准格出修正系数 K_jam / K_rcs（本文件
     --kcoef 模式），主表按标称目标（1 m^2、无干扰）标定。
  3. 末段速度：解析 fort.grm.7 的 w_vx/vy/vz，输出 3km 时刻弹速 v_3km、
     CPA 时刻弹速 v_impact、末段按 1s 区间的弹速中位数剖面。
  4. 弹速档由 stage2 推力校准实现（--calibrate）：g=0 直线目标，
     比例迭代 stage2 推力使 v_peak = 档位值；产出 speed_map.json。
  5. 并行：Pool(16) worker，每 worker 固定一个复用 run 目录（覆盖写，
     解析后截断 fort）——29 万 run 零目录增长，规避沙箱 safe-delete 配额。

2026-10-10 拍板改动（首轮 291,600 run 批跑后表形态核查发现的两个根因）：
  6. 主表制导口径改 guidance_target truth（原 perception）。实测教训：
     目标 weaving 后弹拿到的感知航迹冻结在发射时刻（弹 wy≡0/ay≡0，
     CPA≈目标横移量 1.76 km 且与 g_m 逐 0.1 m 无关；gt≥2 全格塌零），
     同场景 truth 模式 CPA=0.01 m——弹机动/PN/mover 全正常，坏在感知
     航迹链（switch_range 30→200 km 无改善，seeker 航迹也未进 guidance）。
     truth 口径下表 = 纯运动学能力包络，感知/干扰效应由 K_jam/K_rcs
     系数层表达（与第 2 条乘性结构对齐）。
  7. stage2 推进时长上限 60→24 s + 推力下限 a0≥A0_MIN=100 m/s²。
     实测教训：vm10 旧剖面 fuel=81.4 t / T=4e6 N，a0=49 m/s²，慢助推
     被重力转弯压进海面（wz 9144→0 于 27.5 s，残距 230 km）——vm10
     全格、vm9 78% 格 no-anchor。可行性条件 A0_MIN*dur < Isp*g0。
     10 km/s 档若仍不可达按 unreachable 标注（可达上限）。

想定相对 v1 的改动：
  - target_grid 单格（发射距离按 launch_range_km 推导），speed 用 m/s。
  - 目标速度进网格：target_grid speed 用 m/s；mover maximum_speed 固定
    22000 m/s 覆盖全部档。
  - 目标机动 = 持续 weaving（±60° 交替、周期 8 s），在 3km 锚点前
    25 s 启动（_maneuver_time，下限 14 s）——保证机动窗口覆盖末段判定
    窗口。转向过载绑定（WsfFollower.cpp L819）：横向过载 = min(tan(bank)·g,
    v·ω, body_g, radial)——bank_angle_limit 有 85° 硬上限（WsfPath-
    Constraints.cpp L86，tan85°≈11.4g 会把 g_t=20 塌缩），故不设 bank
    （0 = 退出 min 合成）、加 turn_rate_limit 2 rad/s（v_t=200 时折合
    40g 余量），使 body_g_limit（=g_t）成为唯一绑定。
  - 弹过载 g_m 进想定：guidance phase TERMINAL 的 maximum_commanded_g。
  - g=0/10/20：WSF_AIR_MOVER body_g_limit 直接给档位（0 -> 1.001 最小
    合法值，不掷机动）。

用法：
  python calibrate_v2.py --out F:/afsim2.9cn/calib_v2 --calibrate
  python calibrate_v2.py --out F:/afsim2.9cn/calib_v2 --validate
  python calibrate_v2.py --out F:/afsim2.9cn/calib_v2            # 全量
  python calibrate_v2.py --out F:/afsim2.9cn/calib_v2 --resume
"""
import argparse
import io
import json
import math
import os
import subprocess
import sys
import time
from multiprocessing import Pool

ENGAGE_EXE = "F:/afsim2.9cn/bin/engage.exe"
UTILITY_TXT = "F:/afsim2.9cn/demos/engage/scripts/utility_scripts.txt"
G_M = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 20.0]
V_M_KMS = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
V_T_KMS = [0.2, 0.6, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
ANCHOR_M = 3000.0          # 3 km 锚点
MANEUVER_TIME_S = 14.0     # weaving 启动时刻下限
WEAVE_S = 8.0              # weaving 换向周期 s（±60° 交替）
T_FIRE_S = 8.0             # TTR 链路（探测->起套->发射）经验时长
LEAD_MANEUVER_S = 25.0     # weaving 提前 3km 锚点的时长
SITE_ALT_M = 9144.0        # 阵地高度 = 目标高度（30 kft），水平碰撞航线
MID_ALT_M = 9144.0         # MIDCOURSE 巡航高度 = 同上（水平追踪）
HANDOVER_MIN_KM = 40.0     # PN 交接距离下限 km（再乘闭合速度 ×8 s）
KILL_RADII = (30.0, 100.0, 300.0)
ISP_S = 250.0              # stage2 比冲（燃料量按 T*t/(Isp*g0) 标定）
A_DESIGN = 200.0           # 设计纵加速度 m/s^2（决定推进时长）
DUR_MAX_S = 24.0           # stage2 推进时长上限 s（★2026-10-10：a0 下限
                           # 可行性条件 A0_MIN*dur < Isp*g0 ⇒ dur<24.5，
                           # 见 stage2_params）
A0_MIN = 100.0             # stage2 初始纵向加速度下限 m/s^2（★2026-10-10
                           # 实测教训：vm10 按旧协议 fuel=81.4t/T=4e6N，
                           # a0=49 m/s²，慢助推被重力转弯压进海面——wz
                           # 9144→0 于 27.5 s、残距 230 km，全格 no-anchor；
                           # vm9 亦 78% 格 no-anchor）


def _maneuver_time(launch_km, v_m, v_t):
    """weaving 启动时刻 = 3km 锚点时刻前 25 s（下限 14 s）。锚点时刻
    t_anchor ≈ t_fire + (L-3km)/(v_m+v_t)。实测教训：旧协议"14 s 或距
    发射器 10 km 先到"在远距交会里机动远早于末段（单次 90° 转完就直
    飞，g_t 语义丢失；v_m=8 时目标在弹加速段就转走 60 km，no-anchor
    帮凶）——持续 weaving + 锚点相对时刻才保证机动覆盖判定窗口。"""
    closure = v_m * 1000.0 + v_t * 1000.0
    if closure <= 0.0:
        return MANEUVER_TIME_S
    t_anchor = T_FIRE_S + (launch_km * 1000.0 - ANCHOR_M) / closure
    return round(max(MANEUVER_TIME_S, t_anchor - LEAD_MANEUVER_S), 2)


def launch_range_km(v_m, v_t=0.0):
    """发射距离（想定几何，不进表）：保证 3km 锚点事件发生在巡航段——
    L = max(20, 弹加速段位移 (v_m*1000)^2/2a + 目标同期接近量 v_t*t_boost
    + 15, v_t*50)。★实测教训：位移项曾多除一个 1000（v_m^2*1000/2a/
    1000），L 恒缩水到 20-40 km——高速弹在 39 km 几何里 t≈4 s 迎头相撞
    （加速初期 v≈140 m/s），vm8 全格 no-anchor 的真正根因。
    判定锚点统一 3 km 不变。"""
    t_boost = min(max(v_m * 1000.0 / A_DESIGN, 5.0), 60.0)
    boost_km = (v_m * 1000.0) ** 2 / (2.0 * A_DESIGN) / 1000.0
    return max(20.0, boost_km + v_t * t_boost + 15.0, v_t * 50.0)


def stage2_params(dur, thrust_n):
    """stage2 燃料与有效推力：燃料 = T*dur/(Isp*g0)（T 放大则燃料等比
    放大——实测教训二：推力放大 13 倍而燃料不变时，大推力档 v_peak 崩
    到 28 m/s，燃料/质量模型失配）。
    ★2026-10-10（拍板）：推力下限 a0≥A0_MIN——T ≥ A0_MIN*(m_dry+fuel)，
    解出 T_min = A0_MIN*m_dry/(1 - A0_MIN*dur/(Isp*g0))，要求
    A0_MIN*dur < Isp*g0（故 dur 上限 DUR_MAX_S=24 s）；返回有效推力供
    调用方回写（迭代基于有效值）。旧协议 dur=50 s 时该式无解（a0 恒
    <100），vm10 弹坠海根因。dur 也参与校准迭代：floor 推力仍超速时
    缩短 dur（a0 下限不可降，低速档唯一自由度）。"""
    t_floor = A0_MIN * 854.0 / (1.0 - A0_MIN * dur / (ISP_S * 9.80665))
    thrust_eff = max(thrust_n, t_floor)
    fuel = thrust_eff * dur / (ISP_S * 9.80665)
    return fuel, thrust_eff

# ---------------------------------------------------------------- 想定模板
# 与 v1 calibrate_warhead.py 的 SCENARIO_TEMPLATE 差异（其余逐字沿用）：
#   a) target_grid 单格 15km/0/30kft，speed 用 m/s；
#   b) TARGET_TYPE mover maximum_speed 固定 22000 m/s；
#   c) stage2 thrust 占位符化（弹速校准），thrust_duration 固定 20 s；
#   d) 目标机动触发 = 距发射器 10km 或 14 s 先到者；
#   e) v1 的 @SPEED@/@STAGE2@ 占位符由 @TGT_SPEED@/@STAGE2_THRUST@ 替代。
JAMMER_TEMPLATE = """weapon TARGET_JAMMER WSF_RF_JAMMER
   maximum_range 200 km
   transmitter
      frequency_band             8 ghz 16 ghz
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

_TAIL = r"""
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
      one_m2_detect_range              400.0 nm
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
      frame_time                       1.0 sec
      hits_to_establish_track          2 3
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
      total_mass      @STAGE2_FUEL2@ kg
      fuel_mass       @STAGE2_FUEL@ kg
      thrust_duration @STAGE2_DUR@ sec
      thrust          @STAGE2_THRUST@ nt
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
      next_phase MIDCOURSE when phase_time >= 0.8 sec
   end_phase
   phase MIDCOURSE
      commanded_altitude @MID_ALT@ m msl
      next_phase TERMINAL when target_slant_range < @HANDOVER_KM@ km
   end_phase
   phase TERMINAL
      guidance_target truth
      proportional_navigation_gain 4.0
      g_bias                       1.0
      maximum_commanded_g @GMAX@ g
   end_phase
end_processor

platform_type LRSAM WSF_PLATFORM
   icon SA-10_Missile
   weapon_effects LRSAM_LETHALITY
   mover LRSAM_MOVER
   end_mover
   processor guidance LRSAM_GUIDANCE
   end_processor
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
   comm uplink WSF_COMM_TRANSCEIVER
      internal_link seeker_tracks
   end_comm
@FUSE_BLOCK@
end_platform_type

weapon LRSAM WSF_EXPLICIT_WEAPON
   cue_to_predicted_intercept true
   launched_platform_type     LRSAM
   launch_delta_v             22.4 0 0 m/s
   tilt 2.0 degrees
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

   processor tracker WSF_TRACK_PROCESSOR
      internal_link comm
   end_processor

   processor operator WSF_TASK_PROCESSOR
      auto_weapon_uplink on
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

@JAMMER_BLOCK@

platform_type TARGET_TYPE WSF_PLATFORM
   side blue
   icon F-18E
   radar_signature FIGHTER_RADAR_SIGNATURE

   mover WSF_AIR_MOVER
      maximum_speed    22000 m/s
      maximum_altitude 50000 ft
      body_g_limit     @GLIMIT_MOVER@ g
      turn_rate_limit  2.0 rad/s
   end_mover

@JAMMER_MOUNT@
   script_variables
      double mBase = 0.0;
      double mSign = 1.0;
      double mNext = 1.0e10;
      bool   mInit = false;
   end_script_variables

   on_initialize2
      target = PLATFORM.Index();
      if (@GLIMIT@ > 0.0)
      {
         mNext = @MANEUVER_TIME@;
      }
   end_on_initialize2

   execute at_interval_of 0.5 s
      if ((!mInit) && (TIME_NOW >= 1.0))
      {
         mInit = true;
         mBase = PLATFORM.Heading();
         if (MATH.RandomUniform() < 0.5)
         {
            mSign = -1.0;
         }
      }
      if (mInit && (TIME_NOW >= mNext))
      {
         PLATFORM.TurnToHeading(mBase + 60.0 * mSign);
         mSign = -mSign;
         mNext = TIME_NOW + @WEAVE_S@;
      }
   end_execute
end_platform_type

random_seed @SEED@
frame_time 0.02 sec
end_time 200 secs

output_rate default
   time 0.0 secs period 0.02 sec
end_output_rate

output_rate coarse
   time 0.0 secs period 1.0 sec
end_output_rate

run
   record_file_base_name cal

   sites
      xyz 0 km 0 km @SITE_ALT@ m
   end_sites

   target_grid
      down_range  from @LAUNCH_KM@ km to @LAUNCH_KM@ km by 1 km
      cross_range from 0 km to 0 km by 1 km
      altitude    from 30 kft to 30 kft by 1 kft
      speed @TGT_SPEED@ m/s
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

UTILITY_CACHE = {}


def utility_text(path):
    if path not in UTILITY_CACHE:
        UTILITY_CACHE[path] = io.open(path, encoding="utf-8").read()
    return UTILITY_CACHE[path]


def write_scenario(dirpath, seed, tgt_glimit, tgt_speed_mps,
                   stage2, utility, jam_power=0.0, launch_km=20.0,
                   fuse_on=True, g_missile=6.0,
                   maneuver_time=MANEUVER_TIME_S,
                   handover_km=HANDOVER_MIN_KM):
    """渲染 cal.txt。tgt_glimit = 目标机动档（0 -> 1.001 最小合法值，
    不机动）；g_missile = 弹 maximum_commanded_g（0 -> 0.001g = 弹不机动，
    ★实测教训：@GMAX@ 必须吃 g_missile——曾误接 tgt_glimit，弹过载从未
    进过想定，g_m=0 与 6 逐位同结果）；stage2 = (thrust_n, fuel_kg, dur_s)；
    maneuver_time = 目标 weaving 启动时刻。
    fuse_on=False 用于弹速校准：去掉引信让弹飞满 end_time，v_peak = 推进
    结束真实末速（带引信时迎头 intercept 会在加速完成前引爆弹，v_peak
    被截断低估——实测 v=9/10 档 v_peak 卡在 ~1300 m/s 即此因）。
    发射几何协议（想定布景，不进表）：阵地高度 = 目标高度（engage 的
    run/sites/xyz 支持阵地 XYZ，SiteConfig.cpp L117 SetLocationXYZ），
    tilt 2°、delta_v 22.4 m/s——整个交会 = 水平碰撞航线。★实测教训链：
    WSF_GUIDED_MOVER 推力沿速度矢量，地面发射的任何倾斜角（15°/30°）
    在低速段（v≈100-500）被重力转弯压进海面（dγ/dt=-g·cosγ/v，z 实测
    15→-0 m 于 4 s，引信 MSL 引爆；关引信 z=-20 m 仍能飞——弹体不死，
    死于引信）；垂直发射（89.9°）能活但纯 PN 从 90° 拉平的 loft 过冲
    把弹甩到目标上方 10-20 km（vm8 CPA 21 km）；MIDCOURSE commanded_
    altitude 20 km 也拉不平（垂直动量太大，z 冲到 75 km，CPA 71 km）。
    水平发射下低速段下沉无触地风险（下方 9 km 空气），PN 秒拉回。
    MIDCOURSE commanded_altitude 9144 m（水平追踪目标），交接判据
    target_slant_range < max(40, (v_m+v_t)*8) km；短程格起手即交接 =
    退化回纯 PN 剖面。注：中段 aimpoint 追踪不受 maximum_commanded_g
    限制（gm0 行也会被中段修正——g_m 语义 = 末端机动能力）。"""
    if not os.path.isdir(dirpath):
        os.makedirs(dirpath, exist_ok=True)
    # body_g_limit 输入校验要求严格 > 1g（WsfPathConstraints.cpp L101
    # ValueGreater(limit, cACCEL_OF_GRAVITY)），g_t<=1 渲染为 1.001——
    # 物理上 1g 机体法向过载转向能力 = sqrt(a^2-g^2) = 0，与不机动等价。
    g_mover = tgt_glimit if tgt_glimit > 1.0 else 1.001
    if jam_power > 0.0:
        jammer_block = JAMMER_TEMPLATE.replace("@JAMMER_POWER@", "%g" % jam_power)
        jammer_mount = ("   weapon jammer TARGET_JAMMER\n"
                        "      on\n"
                        "   end_weapon\n")
    else:
        jammer_block = ""
        jammer_mount = ""
    fuse_block = ("   processor fuse WSF_AIR_TARGET_FUSE\n"
                  "      gross_proximity_range  100 km\n"
                  "      hit_proximity_range   90 km\n"
                  "   end_processor\n") if fuse_on else ""
    text = (_TAIL
            .replace("@UTILITY_SCRIPTS@", utility)
            .replace("@SEED@", str(seed))
            .replace("@GLIMIT_MOVER@", "%g" % g_mover)
            .replace("@GLIMIT@", "%g" % (tgt_glimit if tgt_glimit > 1.0 else 0.0))
            .replace("@GMAX@", "%g" % (g_missile if g_missile > 0.0 else 0.001))
            .replace("@STAGE2_THRUST@", "%g" % stage2[0])
            .replace("@STAGE2_FUEL@", "%.3f" % stage2[1])
            .replace("@STAGE2_FUEL2@", "%.3f" % (stage2[1] + 0.001))
            .replace("@STAGE2_DUR@", "%g" % stage2[2])
            .replace("@TGT_SPEED@", "%g" % tgt_speed_mps)
            .replace("@LAUNCH_KM@", "%g" % launch_km)
            .replace("@MANEUVER_TIME@", "%g" % maneuver_time)
            .replace("@SITE_ALT@", "%g" % SITE_ALT_M)
            .replace("@MID_ALT@", "%g" % MID_ALT_M)
            .replace("@HANDOVER_KM@", "%g" % handover_km)
            .replace("@WEAVE_S@", "%g" % WEAVE_S)
            .replace("@JAMMER_BLOCK@", jammer_block)
            .replace("@JAMMER_MOUNT@", jammer_mount)
            .replace("@FUSE_BLOCK@", fuse_block))
    if text.count("WSF_RF_JAMMER") != (1 if jam_power > 0.0 else 0):
        raise RuntimeError("jammer block render fail: %s" % dirpath)
    if "@STAGE2_THRUST@" in text or "@TGT_SPEED@" in text \
            or "@LAUNCH_KM@" in text \
            or "@STAGE2_FUEL@" in text or "@STAGE2_DUR@" in text \
            or "@STAGE2_FUEL2@" in text \
            or "@WEAVE_S@" in text or "@MANEUVER_TIME@" in text \
            or "@MID_ALT@" in text or "@HANDOVER_KM@" in text \
            or "@SITE_ALT@" in text:
        raise RuntimeError("placeholder residue: %s" % dirpath)
    with io.open(os.path.join(dirpath, "cal.txt"), "w", encoding="utf-8") as f:
        f.write(text)


def run_one(engage_exe, dirpath, timeout_s):
    t0 = time.strftime("%Y-%m-%d %H:%M:%S")
    proc = subprocess.run([engage_exe, "cal.txt"], cwd=dirpath,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          timeout=timeout_s)
    with io.open(os.path.join(dirpath, "engage.log"), "w", encoding="utf-8") as f:
        f.write("%s file cal.txt\n" % t0)
    return proc.returncode, proc.stdout.decode("utf-8", "replace"), \
        proc.stderr.decode("utf-8", "replace")


def _rows(path, ncols):
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


def parse_cell(dirpath):
    """单格 fort.grm.7 解析（v2 每 run 恰 1 个 cell；wft 回退只做保险）。
    返回 list[dict]（一般长 1）：CPA/t_go/closing + 3km 锚点 + 末段速度。"""
    rows = _rows(os.path.join(dirpath, "fort.grm.7"), 14)
    series, cur, prev = [], [], -1.0
    for r in rows:
        if r[0] < prev and cur:
            series.append(cur)
            cur = []
        cur.append(r)
        prev = r[0]
    if cur:
        series.append(cur)
    out = []
    for ser in series:
        if len(ser) < 5:
            continue
        best_i, best = 0, ser[0][7]
        for i in range(1, len(ser)):
            if ser[i][7] < best:
                best, best_i = ser[i][7], i
        cpa = best
        t_go = float(ser[best_i][0])
        closing = ser[-1][7] > ser[0][7]
        # 速度模序列（w_vx/vy/vz = 列 8/9/10）
        speeds = [math.sqrt(r[8] ** 2 + r[9] ** 2 + r[10] ** 2) for r in ser]
        v_peak = max(speeds)
        v_impact = speeds[best_i]
        # 3km 锚点：首破 ANCHOR_M，相邻两行线性插值时刻与弹速
        t_anchor, v_anchor = None, None
        for i in range(1, len(ser)):
            if ser[i][7] < ANCHOR_M:
                r0, r1 = ser[i - 1][7], ser[i][7]
                if r1 < r0:
                    frac = (r0 - ANCHOR_M) / (r0 - r1)
                    t_anchor = ser[i - 1][0] + frac * (ser[i][0] - ser[i - 1][0])
                    v0 = math.sqrt(ser[i - 1][8] ** 2 + ser[i - 1][9] ** 2 +
                                   ser[i - 1][10] ** 2)
                    v1 = math.sqrt(ser[i][8] ** 2 + ser[i][9] ** 2 +
                                   ser[i][10] ** 2)
                    v_anchor = v0 + frac * (v1 - v0)
                else:
                    t_anchor, v_anchor = float(ser[i][0]), v_impact
                break
        # 末段速度剖面：t>=锚点按 1s 分桶取中位数（最多 12 桶）
        profile = []
        if t_anchor is not None:
            buckets = {}
            for r in ser:
                if r[0] >= t_anchor:
                    v = math.sqrt(r[8] ** 2 + r[9] ** 2 + r[10] ** 2)
                    buckets.setdefault(int(r[0]), []).append(v)
            for k in sorted(buckets)[:12]:
                vv = sorted(buckets[k])
                profile.append(round(vv[len(vv) // 2], 1))
        out.append(dict(cpa=round(cpa, 2), t_go=round(t_go, 2),
                        closing=bool(closing),
                        v_impact=round(v_impact, 1),
                        v_peak=round(v_peak, 1),
                        t_3km=(round(t_anchor, 3) if t_anchor is not None
                               else None),
                        v_3km=(round(v_anchor, 1) if v_anchor is not None
                               else None),
                        profile=profile))
        break  # 单格想定：只取第一个 cell
    return out


def _truncate(path):
    try:
        with io.open(path, "wb"):
            pass
    except OSError:
        pass


# ------------------------------------------------------- 弹速校准
def calibrate_speeds(out_dir, engage_exe, utility, seeds=(11, 12, 13),
                     timeout=180.0):
    """每弹速档比例迭代校准 stage2，使弹道速度峰值 v_peak（推进结束
    时刻弹速 = 巡航末速，3 km 锚点落在巡航段故 v_3km≈v_peak）= 档位值。
    自由度两个：推力 T（比例迭代、步长 clamp ±40%、clamp [5e3,3e7] N）
    与推进时长 dur（初值 v/a_design clamp [5,24] s；floor 推力仍超速时
    缩短 dur——a0 下限不可降）。燃料=T*dur/(Isp*g0) 随推力联动
    （stage2_params）。产出 speed_map.json。"""
    rdir = os.path.join(out_dir, "calib_speed")
    os.makedirs(rdir, exist_ok=True)
    sm_path = os.path.join(out_dir, "speed_map.json")
    speed_map = {}
    if os.path.exists(sm_path):
        speed_map = json.load(io.open(sm_path, encoding="utf-8"))
    for v_des in V_M_KMS:
        key = "%g" % v_des
        if key in speed_map and speed_map[key].get("converged", False):
            print("  v=%g km/s 已校准 thrust=%g"
                  % (v_des, speed_map[key]["thrust_n"]), flush=True)
            continue
        target = v_des * 1000.0
        dur = min(max(v_des * 1000.0 / A_DESIGN, 5.0), DUR_MAX_S)
        thrust = 0.0   # 0 = 用 floor 推力
        v_peak = 0.0
        converged = False
        for it in range(12):
            fuel, thrust_eff = stage2_params(dur, thrust)
            thrust = thrust_eff   # 迭代基于有效值（a0 下限可能抬升）
            write_scenario(rdir, seeds[0], 0.0, 300.0,
                           (thrust, fuel, dur), utility,
                           launch_km=launch_range_km(v_des), fuse_on=False)
            try:
                rc, so, se = run_one(engage_exe, rdir, timeout)
            except subprocess.TimeoutExpired:
                rc, so, se = -99, "", "TIMEOUT"
            cells = parse_cell(rdir) if rc == 0 else []
            _truncate(os.path.join(rdir, "fort.grm.7"))
            v_peak = cells[0]["v_peak"] if cells else 0.0
            print("    v=%g it%d T=%g fuel=%.0f dur=%g -> v_peak=%.0f"
                  % (v_des, it, thrust, fuel, dur, v_peak), flush=True)
            if v_peak <= 0.0:
                thrust *= 0.5
                continue
            if abs(v_peak - target) <= 0.02 * target:
                converged = True
                break
            ratio = target / v_peak
            ratio = min(max(ratio, 0.6), 1.4)   # 步长阻尼 ±40%
            t_floor = A0_MIN * 854.0 / (1.0 - A0_MIN * dur / (ISP_S * 9.80665))
            if v_peak > target and thrust <= t_floor * 1.001:
                # floor 推力已超速且推力不可再降 ⇒ 缩短燃烧时长
                # （a0 下限不可降，低速档唯一自由度；★2026-10-10：
                #  旧协议 dur=v/a_design 卡死 vm5-8 于 v_peak=8792）
                dur = min(max(dur * ratio, 5.0), DUR_MAX_S)
                thrust = 0.0
            else:
                thrust = min(max(thrust * ratio, 5000.0), 30000000.0)
        fuel, thrust = stage2_params(dur, thrust)
        speed_map[key] = dict(
            thrust_n=round(thrust, 1), fuel_kg=round(fuel, 3),
            dur_s=round(dur, 3), v_peak_measured=round(v_peak, 1),
            converged=bool(converged),
            unreachable=bool(v_peak < 0.9 * target))
        with io.open(sm_path, "w", encoding="utf-8") as f:
            json.dump(speed_map, f, ensure_ascii=False, indent=1)
    return speed_map


# ------------------------------------------------------- 并行 worker
_W = {}


def _init_worker(engage_exe, utility, out_dir, timeout):
    _W["exe"] = engage_exe
    _W["utility"] = utility
    _W["out"] = out_dir
    _W["timeout"] = timeout
    _W["dir"] = os.path.join(out_dir, "w%05d" % os.getpid())
    os.makedirs(_W["dir"], exist_ok=True)


def _do_job(job):
    """job = (tag, seed, g_m, v_m_kms, g_t, v_t_kms, stage2_tuple)。返回
    sample dict（或失败 dict）。fort 解析后截断；run 目录复用覆盖写。"""
    tag, seed, g_m, v_m, g_t, v_t, s2 = job
    rdir = _W["dir"]
    lk = launch_range_km(v_m, v_t)
    write_scenario(rdir, seed, g_t, v_t * 1000.0, s2, _W["utility"],
                   launch_km=lk, g_missile=g_m,
                   maneuver_time=_maneuver_time(lk, v_m, v_t),
                   handover_km=max(HANDOVER_MIN_KM, (v_m + v_t) * 8.0))
    try:
        rc, so, se = run_one(_W["exe"], rdir, _W["timeout"])
    except subprocess.TimeoutExpired:
        rc, so, se = -99, "", "TIMEOUT"
    base = dict(tag=tag, g_m=g_m, v_m=v_m, g_t=g_t, v_t=v_t, seed=seed)
    if rc != 0:
        _truncate(os.path.join(rdir, "fort.grm.7"))
        base["fail"] = (se or so)[-200:]
        return base
    cells = parse_cell(rdir)
    _truncate(os.path.join(rdir, "fort.grm.7"))  # 解析后再截断（顺序勿倒）
    if not cells:
        base["fail"] = "no cell parsed"
        return base
    c = cells[0]
    base.update(a_t=round(g_m * 9.80665, 3), **c)
    return base


def _pct(sorted_vals, q):
    n = len(sorted_vals)
    if n == 0:
        return 0.0
    return sorted_vals[min(int(q * n), n - 1)]


def build_table(samples):
    """查表法：键 (g_m, v_m, g_t, v_t)；统计 |CPA| 分位 + p_hit_R +
    p_within_3km + v_3km/v_impact 分位 + anchor 率。
    v2 不再剔除 closing：迎头 flyby 后终距>始距是正常 miss 形态；
    不可交战（弹追不上/没进圈）由 anchor_rate 与大 CPA 如实体现在
    p_hit≈0——表语义是"该弹打该目标的命中概率"，够不着也算 miss。"""
    used = [s for s in samples if "fail" not in s]
    cells = {}
    for s in used:
        cells.setdefault((s["g_m"], s["v_m"], s["g_t"], s["v_t"]), []).append(s)
    rows = []
    for key in sorted(cells):
        g_m, v_m, g_t, v_t = key
        ss = cells[key]
        anchored = [s for s in ss if s["t_3km"] is not None]
        n, na = len(ss), len(anchored)
        row = dict(g_m=g_m, v_m=v_m, g_t=g_t, v_t=v_t, n=n,
                   n_anchored=na,
                   anchor_rate=round(na / float(n), 4) if n else 0.0)
        cpas = sorted(abs(s["cpa"]) for s in ss)
        row["cpa_median"] = round(_pct(cpas, 0.5), 2)
        row["cpa_p90"] = round(_pct(cpas, 0.9), 2)
        row["cpa_max"] = round(cpas[-1], 2)
        for radius in KILL_RADII:
            row["p_hit_%gm" % radius] = round(
                sum(1 for c in cpas if c <= radius) / float(n), 4)
        row["p_within_3km"] = round(
            sum(1 for c in cpas if c <= ANCHOR_M) / float(n), 4)
        if anchored:
            v3s = sorted(s["v_3km"] for s in anchored)
            vis = sorted(s["v_impact"] for s in anchored)
            row["v_3km_median"] = round(_pct(v3s, 0.5), 1)
            row["v_3km_p10"] = round(_pct(v3s, 0.1), 1)
            row["v_impact_median"] = round(_pct(vis, 0.5), 1)
            row["v_impact_p10"] = round(_pct(vis, 0.1), 1)
            row["t_go_median"] = round(
                _pct(sorted(s["t_go"] for s in anchored), 0.5), 2)
        rows.append(row)
    return rows, used


def write_report(path, data):
    L = ["# 弹命中概率标定 v2（3km 锚点口径）", ""]
    L.append("- 主表 4 维：弹过载 %(g_ms)s × 弹末速 km/s %(v_ms)s × "
             "目标过载 %(g_ts)s × 目标速度 km/s %(v_ts)s；seeds=%(seeds)s。"
             % data)
    L.append("- 判定锚点 = 弹目距离首破 3 km（线性插值）；p_hit_R = "
             "|CPA|≤R 的无条件频率（R=30/100/300 m；CPA 对全部 run 统计，"
             "未进圈自然计 miss）；anchor_rate = 进圈率，仅作诊断。")
    L.append("- 弹速档由 stage2 推力校准（speed_map.json）；目标机动 = "
             "持续 weaving（±60°、周期 %(weave_period_s)g s），锚点前 "
             "%(maneuver_lead_s)g s 启动。" % data)
    L.append("- 干扰/RCS 不在主表：K_jam/K_rcs 修正系数另批标定。")
    L.append("- 推演 %(n_runs)s 次（失败 %(n_fail)s）。" % data)
    L.append("")
    L.append("| g_m | v_m | g_t | v_t | n | anch | a_rate | cpa_med | cpa_p90 |"
             " p30 | p100 | p300 | v3km_med | vimp_med | t_go_med |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in data["hit_table"]:
        L.append("| %g | %g | %g | %g | %d | %d | %.3f | %.2f | %.2f | "
                 "%.3f | %.3f | %.3f | %s | %s | %s |" %
                 (r["g_m"], r["v_m"], r["g_t"], r["v_t"], r["n"],
                  r["n_anchored"], r["anchor_rate"], r["cpa_median"],
                  r["cpa_p90"], r["p_hit_30m"], r["p_hit_100m"],
                  r["p_hit_300m"],
                  r.get("v_3km_median", "-"), r.get("v_impact_median", "-"),
                  r.get("t_go_median", "-")))
    L.append("")
    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def main(argv=None):
    ap = argparse.ArgumentParser(description="engage 标定 v2（3km 锚点）")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--procs", type=int, default=16)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--afsim", default=ENGAGE_EXE)
    ap.add_argument("--utility", default=UTILITY_TXT)
    ap.add_argument("--calibrate", action="store_true",
                    help="只跑弹速校准（产 speed_map.json）")
    ap.add_argument("--validate", action="store_true",
                    help="单点验证：3 个格 × seeds=1..3，落 validate.json")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--limit-cells", default="",
                    help="调试：逗号分隔 g_m:v_m:g_t:v_t，只跑这些格")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    utility = utility_text(args.utility)

    sm_path = os.path.join(args.out, "speed_map.json")
    if args.calibrate:
        sm = calibrate_speeds(args.out, args.afsim, utility,
                              timeout=args.timeout)
        print("校准完成：%s" % json.dumps(sm, ensure_ascii=False), flush=True)
        return 0
    if not os.path.exists(sm_path):
        print("缺 speed_map.json，先 --calibrate", flush=True)
        return 1
    speed_map = json.load(io.open(sm_path, encoding="utf-8"))
    s2map = {}
    for k, rec in speed_map.items():
        s2map[float(k)] = (rec["thrust_n"], rec["fuel_kg"], rec["dur_s"])
        if rec.get("unreachable"):
            print("警告：v=%s km/s 不可达（实测 v_peak=%.0f m/s），按可达上限跑"
                  % (k, rec["v_peak_measured"]), flush=True)

    ck_path = os.path.join(args.out, "checkpoint.json")
    done, samples, ok_runs, fail_runs = {}, [], 0, []
    if args.resume and os.path.exists(ck_path):
        ck = json.load(io.open(ck_path, encoding="utf-8"))
        done = ck.get("done", {})
        samples = ck.get("samples", [])
        ok_runs = ck.get("ok_runs", 0)
        fail_runs = ck.get("fail_runs", [])
        print("续跑：已完成 %d" % len(done), flush=True)

    jobs = []
    for g_m in G_M:
        for v_m in V_M_KMS:
            for g_t in G_M:
                for v_t in V_T_KMS:
                    for seed in range(1, args.seeds + 1):
                        tag = "gm%g_vm%g_gt%g_vt%g_s%d" % (g_m, v_m, g_t, v_t, seed)
                        if tag in done:
                            continue
                        jobs.append((tag, seed, g_m, v_m, g_t, v_t,
                                     s2map[v_m]))
    if args.validate:
        jobs = [j for j in jobs
                if j[2] in (0.0, 6.0) and j[3] in (2.0, 8.0)
                and j[4] in (0.0, 6.0) and j[5] in (0.6, 2.0)
                and j[1] <= 3]
    if args.limit_cells:
        keys = set(tuple(float(x) for x in c.split(":"))
                   for c in args.limit_cells.split(","))
        jobs = [j for j in jobs
                if (j[2], j[3], j[4], j[5]) in keys]
    print("计划推演 %d 次（procs=%d）" % (len(jobs), args.procs), flush=True)
    if not jobs:
        print("无待跑 job", flush=True)

    n_new = 0
    t0 = time.time()
    if jobs:
        with Pool(args.procs, initializer=_init_worker,
                  initargs=(args.afsim, utility, args.out, args.timeout)) as pool:
            for res in pool.imap_unordered(_do_job, jobs, chunksize=4):
                tag = res["tag"]
                done[tag] = True
                if "fail" in res:
                    fail_runs.append(dict(tag=tag, tail=res["fail"]))
                else:
                    ok_runs += 1
                    samples.append(res)
                n_new += 1
                if n_new % 200 == 0:
                    dt = time.time() - t0
                    print("  %d/%d  %.2f s/run  free=%.1f GB" %
                          (n_new, len(jobs), dt / n_new,
                           __import__("shutil").disk_usage("F:/").free / 2**30),
                          flush=True)
                    try:
                        with io.open(ck_path, "w", encoding="utf-8") as f:
                            json.dump(dict(done=done, samples=samples,
                                           ok_runs=ok_runs,
                                           fail_runs=fail_runs), f,
                                      ensure_ascii=False)
                    except OSError:
                        pass

    print("完成 %d / 失败 %d  用时 %.1f min"
          % (ok_runs, len(fail_runs), (time.time() - t0) / 60.0), flush=True)
    hit_table, used = build_table(samples)
    data = dict(g_ms=G_M, v_ms=V_M_KMS, g_ts=G_M, v_ts=V_T_KMS,
                seeds=args.seeds, kill_radii=list(KILL_RADII),
                anchor_m=ANCHOR_M,
                launch_range_km_rule="想定几何（不进表）：max(20, "
                "v_m^2/2a + v_t*t_boost + 15, v_t*50) km",
                weave_period_s=WEAVE_S,
                maneuver_lead_s=LEAD_MANEUVER_S,
                n_runs=len(done), n_fail=len(fail_runs),
                n_samples=len(samples), n_used=len(used),
                hit_table=hit_table, samples=samples,
                failures=fail_runs)
    with io.open(os.path.join(args.out, "calib_data.json"), "w",
                 encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    write_report(os.path.join(args.out, "calib_report.md"), data)
    try:
        with io.open(ck_path, "w", encoding="utf-8") as f:
            json.dump(dict(done=done, samples=samples, ok_runs=ok_runs,
                           fail_runs=fail_runs), f, ensure_ascii=False)
    except OSError:
        pass
    print("产出：%s" % args.out, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
