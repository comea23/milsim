# -*- coding: utf-8 -*-
"""Jeff 框架第一轮推演驱动：电子战压制 vs 要地防空（金标准 Growler vs Growler）。

架构（与已批准方案逐条对应）：
  想定层    demos/jeff/growler_eq.txt（只给能力，不写命令）
  规则层    本脚本——从引擎真值算态势数值 → 译英文摘要 → 组选项（语义键，与
            aaw-defend 适配器训练词表一致）→ 调 Jeff → 置信度兜底 → 下发指令
  裁决层    引擎——包线/弹数/同方检查（command_rejected 事件即裁决记录）
  证据链    每次决策落一条记录：态势原文 + 概率分布 + 采纳项 + 兜底标记

已知缺口（见 first_round_gap.md；v0.13.67 起 G1/G2 已补）：
  ~~G1 雷达无运行期辐射开关~~ ⇒ 已补：sensor on/off 指令（§5.18），本驱动
     的 em 决策经 exec_em 执行（emcon⇒off，tws/illum⇒on，带迟滞）
  ~~G2 无地空导弹制导阶段表~~ ⇒ 已补：SAM_GUIDANCE（垂直发射→程序转弯→
     定高→末段双轴 PN），实测 15~60 km×≤8 km 命中；第一轮 hit=True
  G3 红方无专属适配器（暂用 aaw-defend 答红方意图题）
  G4 决策延迟按 wall-clock 不计入仿真时间（指挥周期未建模）
  G8 计分/终局裁判层未建（白方 L2 的"结束条件"一栏因此无法判定）
"""
from __future__ import annotations

import json
import os
import sys
import time
import urllib.request
from math import hypot
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import demo_components  # noqa: E402
from milsim.models import register_framework  # noqa: E402
from milsim.services.store import IFF_FOE  # noqa: E402
from milsim.services.type_registry import ComponentRegistry, PlatformRegistry  # noqa: E402
from milsim.simulation import Simulation  # noqa: E402

JEFF = "http://127.0.0.1:8765/v1/systemone"
MODEL = "aaw-defend"
CONF_MIN = 0.60
#: 推演时长（s）：700 是为了让**包线边沿**的第二发也落地——红方 skim
#: 500 m 时 33~35 km 发射属边缘拦截，弹要飞 60~290 s 才解决（600 s 时
#: 第二发还在飞，白方"链路闭环"判据看到的是未决而非结果）。
SIM_SECONDS = float(os.environ.get("FIRST_ROUND_SECONDS", "700"))
TICK_S = 5.0
#: 红方决策拍：每 60 s 一拍（第一轮只给 3 拍太稀——进场是动态过程）
RED_TICKS = tuple(float(t) for t in range(0, 601, 60))
#: 同一目标的开火冷却（s）：ROE 授权是"能不能打"，节拍是"什么时候打"——
#: 后者是规则层的事，不能每 5 s 问一次就打一次（第一轮实测：25 s 打光 6 弹）。
FIRE_COOLDOWN_S = 30.0

OUT = Path(__file__).resolve().parent / "output"
OUT.mkdir(exist_ok=True)

# ---------------------------------------------------------------- Jeff 客户端 ---


def ask(situation: str, questions: dict, timeout: float = 30.0) -> dict:
    body = {"model": MODEL, "state": {"situation": situation}, "questions": questions}
    req = urllib.request.Request(
        JEFF, data=json.dumps(body).encode("utf-8"),
        headers={"content-type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


def choose(ans: dict, qkey: str, conservative: str) -> tuple[str, float, bool, dict]:
    """取一项；低置信/异常 → 保守默认。返回 (键, 置信度, 是否兜底, 完整分布)。"""
    try:
        a = ans["answers"][qkey]
        if a["type"] == "noul":
            probs = {"true": a.get("p_true", 0.0), "false": 1 - a.get("p_true", 0.0)}
            pick = "true" if a.get("noul") else "false"
            conf = max(probs.values())
        else:
            probs = a.get("probabilities", {})
            pick = a.get("choice", "")
            conf = a.get("confidence", 0.0)
        if pick not in probs or conf < CONF_MIN:
            return conservative, conf, True, probs
        return pick, conf, False, probs
    except Exception:
        return conservative, 0.0, True, {}


# ---------------------------------------------------------------- 态势构建 ---


def contact_rows(sim: Simulation, watcher_id: int, foes_only: bool = True) -> list[dict]:
    """航迹表 → 决策用行。默认只取敌踪（iff == foe）。

    ★ 为什么必须过滤：航迹表里混着**己方刚发射的导弹**（出生即 blue 方，
    在飞期间被己方雷达照着）。不按敌我过滤时"最近的航迹"永远是己方弹，
    driver 会拿它当目标去开火——引擎侧同方拒发算是最后一道闸，但决策链
    已经在问"要不要打自己"了（第一轮实测踩到：190~220 s 连续 7 次拒发）。
    """
    rows = []
    for c in sim.store.contacts_of(watcher_id):
        if foes_only and c.iff != IFF_FOE:
            continue
        tgt = sim.registry.get(c.target_id)
        if tgt is None:
            continue
        rows.append(dict(name=tgt.name, id=c.target_id, range_km=c.range_m / 1000.0,
                         bearing_deg=c.bearing_deg, quality=c.quality,
                         iff=c.iff, phantom=c.phantom,
                         age_s=(sim.engine.now - c.detected_at) / 1e6))
    return rows


def blue_situation(sim: Simulation, sam_id: int, rows: list[dict], stock: int,
                   closing: dict[str, float], jam_w: float = 0.0,
                   radar_on: bool = True) -> str:
    em_txt = ("Own search radar radiating." if radar_on else
              "Own search radar held dark (emcon); tracks frozen at last update.")
    if not rows:
        return (f"Blue SAM site on routine air-defense watch. No hostile tracks. "
                f"{em_txt} {stock} missiles in stock.")
    tr = "; ".join(
        f"{r['name']} at {r['range_km']:.0f} km bearing {r['bearing_deg']:.0f} deg, "
        f"closing {closing.get(r['name'], 0.0):+.0f} m/s, "
        f"time to impact {r['range_km'] * 1000 / max(abs(closing.get(r['name'], 1.0)), 1.0):.0f} s, "
        f"track quality {r['quality']:.2f}, age {r['age_s']:.0f} s" for r in rows)
    # ★ 干扰状态必须按**实测值**说：jam_w > 0 才敢写 "jamming active"。
    # 硬编码一句"jamming active"是让模型对着不存在的信息决策（第一版犯过）。
    if jam_w > 0.0:
        ew = (f"Enemy jamming confirmed on the site's 4 GHz search band "
              f"(received {jam_w * 1000:.1f} mW in beam), track quality degraded.")
    else:
        ew = "No enemy jamming detected on the search band, tracks clean."
    return (f"Blue SAM site with {len(rows)} hostile EA-18G jammer tracks: {tr}. "
            f"{ew} {em_txt} Own missiles in stock {stock}. "
            f"Rules of engagement permit engaging identified hostiles inside "
            f"40 km with fresh tracks.")


def red_situation(sim: Simulation, rows: list[dict], alt_m: float) -> str:
    tr = "; ".join(f"{r['name']} at {r['range_km']:.0f} km" for r in rows) or "no site track yet"
    return (f"Red EW formation: two EA-18G jammer aircraft ingressing toward a blue "
            f"SAM site at {alt_m:.0f} m, closing 150 m/s, no fighter escort, no "
            f"anti-radiation weapons. Own jamming active against the site's 4 GHz "
            f"radar. Blue site track: {tr}.")


# ---------------------------------------------------------------- 主流程 ---


def main() -> int:
    source = (Path(__file__).resolve().parent / "growler_eq.txt").read_text(encoding="utf-8")
    components, platforms = ComponentRegistry(), PlatformRegistry()
    register_framework(components, platforms)
    demo_components.register_all(components)
    sim = Simulation.from_scenario(source, name="growler_eq",
                                   components=components, platforms=platforms)
    sim.build()
    sim.initialize()

    sam_id = sim.registry.by_name("SAM_1").entity_id
    jam_ids = [sim.registry.by_name(n).entity_id for n in ("JAM_1", "JAM_2")]
    zone = sim.maps.active_zones[0]
    frame = zone.frame

    def eid(name: str) -> int:
        return sim.registry.by_name(name).entity_id

    def eid_of(entity_id: int) -> str:
        e = sim.registry.get(entity_id)
        return e.name if e else str(entity_id)

    def xy(lat: float, lng: float) -> tuple[float, float]:
        return frame.geo_to_world(lat, lng)

    SAM = (39.9042, 116.4074)
    log: list[dict] = []
    events: list[dict] = []

    def on_fired(a, b, c, d):
        events.append(dict(t=sim.engine.now / 1e6, kind="weapon_fired",
                           src=sim.registry.get(a).name if sim.registry.get(a) else a,
                           missile=sim.registry.get(b).name if sim.registry.get(b) else b,
                           tgt=sim.registry.get(c).name if sim.registry.get(c) else c))
    def on_resolved(attacker, target, hit, fraction, destroyed):
        at = sim.registry.get(attacker)
        tg = sim.registry.get(target)
        events.append(dict(t=sim.engine.now / 1e6, kind="engagement_resolved",
                           attacker=at.name if at else attacker,
                           target=tg.name if tg else target,
                           hit=hit, fraction=round(fraction, 4), destroyed=destroyed))
    def on_rejected(eid, cmd, why):
        events.append(dict(t=sim.engine.now / 1e6, kind="command_rejected",
                           entity=sim.registry.get(eid).name if sim.registry.get(eid) else eid,
                           cmd=cmd, why=why))

    sim.bus.weapon_fired.subscribe(on_fired)
    sim.bus.engagement_resolved.subscribe(on_resolved)
    sim.bus.command_rejected.subscribe(on_rejected)

    def stock_of() -> int:
        w = sim.entities[sam_id].component("weapon")
        return int(w.stock()) if w is not None else 0

    def jam_pos(i: int) -> tuple[float, float, float]:
        p = sim.store.position_of(jam_ids[i])
        return (p[0], p[1], p[2]) if p else (0.0, 0.0, 0.0)

    def alt_of(i: int) -> float:
        p = sim.store.position_of(jam_ids[i])
        return p[2] if p else 0.0

    PROFILE_ALT = {"high": 8000.0, "dash": 3000.0, "skim": 500.0}

    def issue_red(i: int, intent: str, profile: str) -> None:
        """红方决策 → 航线指令。intent 决定目标点，profile 决定高度。"""
        lat0, lng0 = SAM
        alt = PROFILE_ALT.get(profile, 3000.0)
        if intent == "egress":
            lat_d = lat0 - 0.63          # 70 km 南
        elif intent == "transit":
            lat_d = lat0 - 0.36          # 40 km 南待机
        else:                             # ingress
            lat_d = lat0 - 0.108         # 12 km 南（SAM 包线内）
        lng_d = lng0 + (0.004 if i == 0 else -0.004)
        x, y = xy(lat_d, lng_d)
        red_dest[i] = (x, y, alt)
        ok, why = sim.command(jam_ids[i], "move", {"points": [[x, y, alt]]})
        log[-1].setdefault("cmd", []).append(
            f"move {eid_of(jam_ids[i])} -> ({lat_d:.4f},{lng_d:.4f}) @{alt:.0f}m ok={ok} {why}")

    def arrived_all(tol_m: float) -> bool:
        """双机都已到位（离各自目标点 tol_m 以内）。"""
        for i in (0, 1):
            dest = red_dest[i]
            if dest is None:
                return False
            p = sim.store.position_of(jam_ids[i])
            if p is None:
                return False
            if hypot(p[0] - dest[0], p[1] - dest[1]) > tol_m:
                return False
        return True

    def band_of(alt_m: float) -> str:
        """按当前高度反推高度带名（profile 题的选项键）。"""
        if alt_m > 6500.0:
            return "high"
        if alt_m > 1500.0:
            return "dash"
        return "skim"

    # 红机各自的目标点（世界坐标 x/y/alt）——"已到位"判据用：双机都到位时
    # profile 题没有答案（选哪个高度带是个伪问题），规则层直接跳过。
    red_dest: list[tuple[float, float, float] | None] = [None, None]

    # ---- t=0：红方进场 + 首次决策 ----
    t0 = time.perf_counter()
    for i in (0, 1):
        lat_d, lng_d = SAM[0] - 0.36, SAM[1] + (0.004 if i == 0 else -0.004)
        x, y = xy(lat_d, lng_d)
        red_dest[i] = (x, y, 3000.0)
        sim.command(jam_ids[i], "move", {"points": [[x, y, 3000.0]]})

    ticks = int(SIM_SECONDS / TICK_S)
    low_streak = 0
    range_hist: dict[str, tuple[float, int]] = {}
    last_shot: dict[str, float] = {}
    # 雷达辐射态（v0.13.67 G1）：em 决策经 sensor on/off 执行。None = 本拍
    # 从组件同步一次（想定 emitting 定的初始态）。
    radar_on: bool | None = None

    def exec_em(rec: dict, want_on: bool) -> None:
        """em 决策 → sensor on/off。带迟滞：只在**状态要变**时才下令。

        指令本身幂等（重复开关回执都是成功），但每拍都记一条回执是噪声；
        而"变天"那一刻必须留痕——em 是蓝方最贵的决策（开机即暴露）。
        状态没变时**不记录**：日志里出现 em_exec 就是一次真实的状态切换。
        """
        nonlocal radar_on
        if want_on != radar_on:
            cmd = "on" if want_on else "off"
            ok, why = sim.command(sam_id, cmd)
            rec["em_exec"] = dict(cmd=cmd, ok=ok, why=why)
            if ok:
                radar_on = want_on
        # 每拍都记**真实**辐射态（白方按它累计开机时长）；em_exec 只在
        # 上面真下令时出现——日志里见到 em_exec 就是一次状态切换。
        rec["radar_on"] = bool(radar_on)
    for k in range(ticks + 1):
        t_sim = k * TICK_S
        sim.run_for(int(TICK_S * 1_000_000))

        # ---- 红方决策拍 ----
        if any(abs(t_sim - rt) < 1e-6 for rt in RED_TICKS):
            rows = contact_rows(sim, jam_ids[0])
            sit = red_situation(sim, rows, alt_of(0))
            qs = {
                "intent": {"type": "choice", "instructions": "Which option applies?",
                           "criteria": {
                               "ingress": "strike_ingress: flying inbound toward the defended point, an attack run",
                               "egress": "egress: opening from the defended point, leaving the area",
                               "transit": "transit_patrol: parallel or orbiting flight, neither closing nor opening"}},
                "profile": {"type": "choice", "instructions": "Which option applies?",
                            # ★ 判据必须是**数值**的，且与规则层 band_of 同阈值：
                            #   态势文本只给当前高度（不给红方自己的距离），判据
                            #   却写"far from the threat ring"——模型只能猜，爬升/
                            #   俯冲过带时必然五五开（第一轮 t=120/300/360/420 四拍
                            #   兜底全在这：定性带没有边界）。
                            "criteria": {
                                "high": "high_alt_cruise: your current altitude is above 6500 m",
                                "dash": "medium_dash: your current altitude is between 1500 m and 6500 m",
                                "skim": "sea_skim: your current altitude is below 1500 m"}},
            }
            try:
                ans = ask(sit, qs)
            except Exception as exc:
                ans = {}
                log.append(dict(t=t_sim, side="red", fallback=True, err=str(exc)))
            rec = dict(t=t_sim, side="red", situation=sit, answers=ans.get("answers", {}))
            intent, c1, fb1, p1 = choose(ans, "intent", "transit")
            # ★ profile 题只在**还在机动**时才问：双机都已到位（离目标点
            #   <2 km）时"选哪个高度带"是个没有答案的问题——模型只能在两个
            #   区间之间猜，出来就是五五开的低置信（第一轮 7 拍兜底全在这）。
            #   规则层按当前高度给答案并记 skipped，不计兜底、不累计 escalate。
            if arrived_all(2_000.0):
                profile = band_of(alt_of(0))
                c2, fb2 = 1.0, False
                rec["profile_skipped"] = True
            else:
                profile, c2, fb2, p2 = choose(ans, "profile", "dash")
            rec.update(intent=intent, intent_conf=c1, intent_fb=fb1,
                       profile=profile, profile_conf=c2, profile_fb=fb2)
            log.append(rec)
            issue_red(0, intent, profile)
            issue_red(1, intent, profile)

        # ---- 蓝方决策拍（每拍）----
        rows = contact_rows(sim, sam_id)
        # 目标距离历史 → closing（态势文本必须给这个量：phase/em 的训练词表
        # 直接按 closing 与 TTI 分区，不给就是让模型在两个区间之间猜）
        closing = {}
        for r in rows:
            prev = range_hist.get(r["name"])
            if prev is not None:
                dt = max(t_sim - prev[1], 0.001)   # 两侧都用**秒**
                closing[r["name"]] = (prev[0] - r["range_km"]) * 1000.0 / dt
            range_hist[r["name"]] = (r["range_km"], t_sim)
        # 雷达当时收到的干扰功率（J/S 压制的直接读数；0 = 无干扰进波束）
        radar = sim.entities[sam_id].component("sensor")
        if radar_on is None:
            radar_on = bool(getattr(radar, "enabled", True))
        jam_w = 0.0
        if radar is not None and callable(getattr(radar, "received_jamming_w", None)):
            try:
                jam_w = radar.received_jamming_w(
                    radar.face_deg if hasattr(radar, "face_deg") else 0.0)
            except Exception:
                pass
        sit = blue_situation(sim, sam_id, rows, stock_of(), closing, jam_w, radar_on)
        rec = dict(t=t_sim, side="blue", situation=sit, answers={},
                   contacts=[dict(name=r["name"], range_km=r["range_km"],
                                  age_s=r["age_s"]) for r in rows])
        # ★ 与 engage_auth 同一条原则：问题必须适定。没有航迹时
        #   phase/em 都缺关键输入（TTI/量测距离），问出来只会得到五五开的
        #   低置信——那是"问题不成立"，不是"模型没把握"。规则层直接取
        #   默认档并记录，不计兜底、不累计 escalate。
        if not rows:
            rec.update(phase="watch", phase_conf=1.0, phase_fb=False, phase_skipped=True,
                       em="tws", em_conf=1.0, em_fb=False, em_skipped=True,
                       engage_auth="false", auth_conf=1.0, auth_fb=False, auth_skipped=True,
                       fire=None, jam_w=jam_w)
            # 无航迹 ⇒ 规则层默认"开机搜索"（emcon 是为了防反辐射弹，没有目标
            # 在头上时静默没有任何收益）
            exec_em(rec, True)
            log.append(rec)
            continue
        qs = {
            "phase": {"type": "choice", "instructions": "Which option applies?",
                      "criteria": {
                          "watch": "normal_watch: threat far out, time to impact over 300 s, routine monitoring",
                          "alert": "heightened_alert: time to impact 60 to 300 s, weapons crews alerted",
                          "engage": "active_engagement: time to impact under 60 s or range under 15 km, firing window",
                          "reset": "reset_and_assess: threat has flown past and is opening, stand down and assess"}},
            "em": {"type": "choice", "instructions": "Which option applies?",
                   "criteria": {
                       "emcon": "passive_emcon_hold: range over 40 km, keep radar silent, wait for cueing",
                       "tws": "radar_on_lowpower_tws: range 15 to 40 km, radar on in track-while-scan at low power",
                       "illum": "full_active_illuminate: range under 15 km, full active illumination for missiles"}},
        }
        # ★ 交战授权只在**有航迹可授权**时才问——没有 track 时"May this track be
        # engaged" 这个问题本身不成立（问出来只会得到无依据的 true/false）。
        # （走到这里必有航迹：无航迹拍已在上方 continue）
        qs["engage_auth"] = {
            "type": "noul",
            "instructions": ("May this track be engaged now under the rules of "
                             "engagement? Answer true if engagement is allowed."),
            "criteria": {
                "true": "engagement allowed: identified hostile, inside the 40 km envelope, track fresh, not beyond 20 km while opening",
                "false": "engagement not allowed: track stale, outside the envelope, or opening beyond 20 km"}}
        try:
            ans = ask(sit, qs)
        except Exception as exc:
            ans = {}
            rec["error"] = str(exc)
        rec["answers"] = ans.get("answers", {})
        phase, cp, fbp, pp = choose(ans, "phase", "alert")
        em, ce, fbe, pe = choose(ans, "em", "tws")
        auth, ca, fba, pa = choose(ans, "engage_auth", "false")
        rec.update(phase=phase, phase_conf=cp, phase_fb=fbp,
                   em=em, em_conf=ce, em_fb=fbe,
                   engage_auth=auth, auth_conf=ca, auth_fb=fba)
        log.append(rec)
        low_streak = low_streak + 1 if (fbp or fbe or fba) else 0
        if low_streak >= 3:
            events.append(dict(t=t_sim, kind="escalate", why="3 consecutive low-confidence decisions"))
            low_streak = 0
        # ---- 裁决链：授权 + 包线（引擎）+ 弹数 + 节拍（规则层）----
        fired = False
        best = min(rows, key=lambda r: r["range_km"]) if rows else None
        cool_ok = best is not None and (t_sim - last_shot.get(best["name"], -1e9)) >= FIRE_COOLDOWN_S
        if auth == "true" and not fba and best is not None and stock_of() > 0 and cool_ok:
            ok, why = sim.command(sam_id, "fire", {"target": best["name"]})
            rec["fire"] = dict(target=best["name"], range_km=best["range_km"], ok=ok, why=why)
            fired = ok
            if ok:
                last_shot[best["name"]] = t_sim
        elif best is not None and not cool_ok:
            rec["fire"] = dict(target=best["name"], ok=False,
                               why=f"cooldown {FIRE_COOLDOWN_S:.0f}s not elapsed")
        if not fired:
            rec["fire"] = rec.get("fire") or None
        # ---- em 决策 → sensor on/off（v0.13.67，G1 闭环）----
        # emcon ⇒ 停机静默；tws/illum ⇒ 辐射。框架只有一位开关、没有功率档——
        # "低功率 TWS"与"全照射"的差别等发射功率参数化（peak_power 已是参数，
        # 运行期可调）后再分，现在按"开/关"二值执行并如实记录。
        exec_em(rec, em in ("tws", "illum"))
        rec["jam_w"] = jam_w

    report = sim.shutdown()
    wall = time.perf_counter() - t0

    (OUT / "first_round_log.json").write_text(json.dumps(
        dict(scenario="growler_eq.txt", model=MODEL, sim_seconds=SIM_SECONDS,
             wall_seconds=round(wall, 1), decisions=log, events=events,
             final_state=report.summary()),
        ensure_ascii=False, indent=1), encoding="utf-8")

    # ---- 汇总 ----
    fb = sum(1 for r in log if r.get("intent_fb") or r.get("phase_fb")
             or r.get("em_fb") or r.get("auth_fb"))
    fires = [r for r in log if r.get("fire")]
    launches = [r for r in fires if r["fire"].get("ok")]
    blocked = [r for r in fires if not r["fire"].get("ok")]
    em_flips = [r for r in log if r.get("side") == "blue" and "em_exec" in r]
    print(f"推演 {SIM_SECONDS:.0f} s（wall {wall:.0f} s）｜决策 {len(log)} 拍｜兜底 {fb} 拍｜"
          f"发射 {len(launches)} 次（发射指令 {len(fires)} 次，冷却/拒发挡下 "
          f"{len(blocked)} 次）｜雷达开关指令 {len(em_flips)} 次")
    print(f"实体终态：{report.summary().splitlines()[-1] if report.summary() else ''}")
    print("\n== 决策序列（每 5 s 一拍，节选）==")
    for r in log:
        if r["side"] == "red":
            cmds = " | ".join(r.get("cmd", []))
            print(f"  t={r['t']:>5.0f} 红 intent={r.get('intent')}({r.get('intent_conf', 0):.2f}) "
                  f"profile={r.get('profile')}({r.get('profile_conf', 0):.2f}) "
                  f"{'FB' if r.get('intent_fb') or r.get('profile_fb') else ''} {cmds}")
        else:
            f = r.get("fire")
            if f and f.get("target"):
                fs = f"→ 开火 {f['target']}@{f.get('range_km', 0):.0f}km ok={f['ok']}"
            elif f:
                fs = f"→ 未发 {f.get('why', '')}"
            else:
                fs = ""
            ex = r.get("em_exec")
            if ex:
                fs += f"｜雷达 {ex.get('cmd')} ok={ex.get('ok')}"
            print(f"  t={r['t']:>5.0f} 蓝 phase={r.get('phase')} em={r.get('em')} "
                  f"auth={r.get('engage_auth')}({r.get('auth_conf', 0):.2f}) "
                  f"{'FB' if r.get('phase_fb') or r.get('em_fb') or r.get('auth_fb') else ''} {fs}")
    print("\n== 引擎事件 ==")
    for e in events:
        print(" ", e)
    print(f"\n证据链：{OUT / 'first_round_log.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
