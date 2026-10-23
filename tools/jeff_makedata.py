"""域七·Jeff 域内适配器训练数据生成（规则真值合成，无 LLM 教师标签）。

**与探针同源**：真值规则=``tools/jeff_probe.py`` 的 ``truth()``（intent/phase/em/
profile/terminal/engage_auth 六问，口径逐字一致），特征空间从真实弹道特征
（``demos/jeff/flight_features.json``：R 800~62 km、closing ±430 m/s、alt
30~12 000 m、vz −65~+35 m/s、track_age 1~30 s）扩成连续采样。

**防 shortcut 设计**（adapter-kit README 的教训）：
- 标签分层逆采样：先抽标签再抽特征区域，标签均衡；
- 态势模板 8 套 × 数字格式变体，模板选择与标签独立；
- IFF 友/敌 70/30 与全部 choice 标签独立（只影响 engage_auth 真值）；
- 选项描述每键 2 个措辞变体随机取、字典序随机洗牌（防位置/长度作弊）；
- 噪声上下文句（海况/编队/气象）与标签独立。

**行格式**（kit 校验口径）::

    {"id","suite","family","state","question","label","target","source"}
    choice: label/target=选项键；noul: label/target=bool。

运行（milsim venv，纯标准库）::

    python tools/jeff_makedata.py --rows 12000 --families 48 \
        --out demos/jeff/data/rows.jsonl
"""
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

SUITE = "aaw-defend"

# ---------------------------------------------------------------- 真值规则 ---
# 与 tools/jeff_probe.py truth() 逐字同口径（键=选项键）。

def truth(kind: str, f: dict) -> str | bool:
    r, alt, vz = f["range_m"], f["alt_m"], f["vz_mps"]
    c = f["closing_mps"]
    tti = r / c if c > 5 else float("inf")
    opening = c < -5
    if kind == "intent":
        if c > 5:
            return "ingress"
        if c < -5:
            return "egress"
        return "transit"
    if kind == "engage_auth":
        if not f["iff_hostile"]:
            return False
        return (f["track_age_s"] <= 6.0 and r <= 40_000
                and not (opening and r > 20_000))
    if kind == "phase":
        if opening:
            return "reset"
        if tti <= 60 or r <= 15_000:
            return "engage"
        if tti <= 300:
            return "alert"
        return "watch"
    if kind == "em":
        if r > 40_000:
            return "emcon"
        if r <= 15_000:
            return "illum"
        return "tws"
    if kind == "profile":
        if alt >= 2000 and r > 30_000:
            return "high"
        if alt <= 500 and r <= 30_000:
            return "skim"
        return "dash"
    if kind == "terminal":
        if opening:
            return "climbout"
        if vz < -10 and r <= 10_000:
            return "dive"
        return "continue"
    raise ValueError(kind)

KINDS = ["intent", "engage_auth", "phase", "em", "profile", "terminal"]
CHOICE_KINDS = ["intent", "phase", "em", "profile", "terminal"]

# 每问的选项键与 2 个措辞变体（键与探针一致；标签名与键的映射由此文档化）。
OPTS = {
    "intent": {
        "ingress": ["strike_ingress: flying inbound toward the defended point, an attack run",
                    "strike_ingress: inbound on an attack run toward the force"],
        "egress": ["egress: opening from the defended point, leaving the area",
                   "egress: outbound and opening, departing the area"],
        "transit": ["transit_patrol: parallel or orbiting flight, neither closing nor opening",
                    "transit_patrol: range steady, transiting or on patrol"],
    },
    "phase": {
        "watch": ["normal_watch: threat far out, time to impact over 300 s, routine monitoring",
                  "normal_watch: time to impact above 300 s, routine monitoring only"],
        "alert": ["heightened_alert: time to impact 60 to 300 s, weapons crews alerted",
                  "heightened_alert: one to five minutes to impact, crews to alert"],
        "engage": ["active_engagement: time to impact under 60 s or range under 15 km, firing window",
                   "active_engagement: under a minute to impact or inside 15 km, weapons window open"],
        "reset": ["reset_and_assess: threat has flown past and is opening, stand down and assess",
                  "reset_and_assess: contact is opening after passing, stand down and reassess"],
    },
    "em": {
        "emcon": ["passive_emcon_hold: range over 40 km, keep radar silent, wait for cueing",
                  "passive_emcon_hold: beyond 40 km, stay radar silent and wait for cueing"],
        "tws": ["radar_on_lowpower_tws: range 15 to 40 km, radar on in track-while-scan at low power",
                "radar_on_lowpower_tws: 15 to 40 km, radiate low power in track-while-scan"],
        "illum": ["full_active_illuminate: range under 15 km, full active illumination for missiles",
                  "full_active_illuminate: inside 15 km, full active illumination for the shooters"],
    },
    "profile": {
        "high": ["high_alt_cruise: above 2000 m and far from the threat ring, cruise high",
                 "high_alt_cruise: high and far out, cruise at altitude"],
        "skim": ["sea_skim: below 500 m and inside 30 km, hug the deck under the radar horizon",
                 "sea_skim: low and inside 30 km, hug the wave tops below the radar horizon"],
        "dash": ["medium_dash: transition band, dash at medium altitude",
                 "medium_dash: neither high-far nor low-near, dash at medium height"],
    },
    "terminal": {
        "continue": ["continue_ingress: keep flying the approach, not yet in the terminal gate",
                     "continue_ingress: not yet in the terminal gate, keep the approach"],
        "dive": ["terminal_dive: descending and within 10 km, commit to the terminal dive",
                 "terminal_dive: descending inside 10 km, commit to the dive"],
        "climbout": ["egress_climb: past the target and opening, climb out",
                     "egress_climb: past the point and opening, climb away"],
    },
}

AUTH_TRUE_TXT = [
    "engagement allowed: identified hostile, inside the 40 km envelope, track fresh, not beyond 20 km while opening",
    "engagement allowed: hostile ID, inside 40 km, fresh track, and not an opening contact beyond 20 km",
]
AUTH_FALSE_TXT = [
    "engagement not allowed: track stale, outside the envelope, or opening beyond 20 km",
    "engagement not allowed: stale track, outside the envelope, or an opening contact beyond 20 km",
]

# ------------------------------------------------------------ 特征区域采样 ---

def _u(rng: random.Random, a: float, b: float) -> float:
    return rng.uniform(a, b)

def sample_features(kind: str, label: str | bool,
                    fam: dict, rng: random.Random) -> dict:
    """按 (kind, label) 的真值区域直接采样（分层逆采样，保证标签均衡）。"""
    r = c = alt = vz = age = None
    iff_hostile = True
    if kind == "intent":
        r = _u(rng, 2_000, 62_000)
        if label == "ingress":
            c = _u(rng, 30, 430)
        elif label == "egress":
            c = _u(rng, -430, -30)
        else:  # transit
            c = _u(rng, -4.5, 4.5)
    elif kind == "em":
        c = _u(rng, -430, 430)
        if label == "emcon":
            r = _u(rng, 40_500, 62_000)
        elif label == "tws":
            r = _u(rng, 15_300, 39_800)
        else:  # illum
            r = _u(rng, 900, 14_900)
    elif kind == "profile":
        r = _u(rng, 2_000, 62_000)
        c = _u(rng, -430, 430)
        if label == "high":
            alt = _u(rng, 2_100, 12_000)
            r = _u(rng, 30_500, 62_000)
        elif label == "skim":
            alt = _u(rng, 40, 480)
            r = min(r, 29_500)
        else:  # dash：既有不高远、也不低近
            for _ in range(500):
                alt = _u(rng, 40, 12_000)
                if not (alt >= 2_000 and r > 30_000) and not (alt <= 500 and r <= 30_000):
                    break
    elif kind == "phase":
        if label == "reset":
            c = _u(rng, -430, -30)
            r = _u(rng, 2_000, 62_000)
        elif label == "engage":
            if rng.random() < 0.5:
                r = _u(rng, 900, 14_900)
                c = _u(rng, -4.9, 430)  # 非 opening（opening→reset 先判）
            else:
                r = _u(rng, 15_100, 60_000)
                c = r / _u(rng, 10, 60)
        elif label == "alert":
            r = _u(rng, 15_300, 62_000)
            c = r / _u(rng, 62, 298)
        else:  # watch
            r = _u(rng, 20_000, 62_000)
            c = r / _u(rng, 305, 1_500)
    elif kind == "terminal":
        c = _u(rng, -4.9, 430)  # dive/continue 都要求非 opening（opening→climbout）
        r = _u(rng, 900, 62_000)
        if label == "dive":
            vz = _u(rng, -65, -10.5)
            r = _u(rng, 900, 9_900)
        elif label == "climbout":
            c = _u(rng, -430, -30)
            vz = _u(rng, -65, 35)
        else:  # continue：非 opening、非（vz<-10 且 r<=10 km）
            for _ in range(500):
                vz = _u(rng, -65, 35)
                if not (vz < -10 and r <= 10_000):
                    break
    elif kind == "engage_auth":
        r = _u(rng, 900, 62_000)
        c = _u(rng, -430, 430)
        age = _u(rng, 1, 30)
        iff_hostile = True
        if label is True:
            age = _u(rng, 1, 5.8)
            r = _u(rng, 900, 39_800)
            c = _u(rng, -4.9, 430) if r > 20_000 else _u(rng, -430, 430)
        else:
            why = rng.randrange(4)
            if why == 0:
                iff_hostile = False  # IFF 应答，友方
            elif why == 1:
                age = _u(rng, 6.5, 30)
            elif why == 2:
                r = _u(rng, 40_500, 62_000)
            else:  # opening 且 r>20 km
                c = _u(rng, -430, -30)
                r = _u(rng, 20_500, 62_000)
    if alt is None:
        alt = _u(rng, 40, 12_000)
    if vz is None:
        vz = _u(rng, -65, 35)
    if age is None:
        age = _u(rng, 1, 30)
    return {"range_m": r, "closing_mps": c, "alt_m": alt, "vz_mps": vz,
            "track_age_s": age, "iff_hostile": iff_hostile}

# ---------------------------------------------------------------- 文本渲染 ---

TEMPLATES = [
    "Anti-air warfare. You defend a naval group. A radar track at {r} km, {motion}, altitude {alt} m, vertical speed {vz} m/s. {iff}. Track age {age} s. Source: own search radar.",
    "You are the air defence watch officer for a naval task group. Own search radar holds one contact: range {r} km, {motion}, altitude {alt} m, climb or descent rate {vz} m/s, {iff_lower}. Track is {age} s old.",
    "Air defence of a naval group under way. A single air contact is held on the search radar: {r} km out, {motion}, flying at {alt} m ({vz} m/s vertical), {iff_lower}; the track is {age} s old.",
    "Warship air defence. Radar picture: one air contact at {r} km, {motion}; altitude {alt} m; vertical rate {vz} m/s; {iff_lower}; age of track {age} s.",
    "You run the air defence of a convoy. One air contact: range to it {r} km, it is {motion}, altitude {alt} m, vertical speed {vz} m/s, {iff_lower}. The radar track is {age} s old.",
    "Fleet air defence watch. A new air track: {r} km, {motion}; {alt} m; {vz} m/s vertical; {iff_lower}; track age {age} s. Judged from your own search radar picture.",
    "Air picture: single contact, {r} km from the force, {motion}, at {alt} m and {vz} m/s vertically, {iff_lower}, {age} s since the track formed.",
    "Defending a naval group. One air track on the search radar: range {r} km, {motion}, height {alt} m, vertical speed {vz} m/s, {iff_lower}; the track is {age} s old.",
]

IFF_HOSTILE = [
    "IFF interrogation failed (hostile)",
    "IFF challenge returned no reply, treat as hostile",
    "No IFF response; presumed hostile",
    "Mode 4 IFF negative, hostile",
]
IFF_FRIENDLY = [
    "IFF responded, presumed friendly",
    "IFF Mode 4 correct, friendly",
    "IFF challenge answered, friendly contact",
]

NOISE = [
    "Sea state {n}.",
    "Own group is in formation {form}, course {crs} at {kn} knots.",
    "Weather: overcast, visibility {vis} km.",
    "No other contacts within 100 km.",
    "A helicopter is on station ahead of the screen.",
    "The force is conducting {op}.",
]

INSTR_CHOICE = [
    "Which option applies?",
    "Which category best fits this track right now?",
    "Select the option that matches the current situation.",
]

def _fill_noise(rng: random.Random) -> str:
    t = rng.choice(NOISE)
    return (t.replace("{n}", str(rng.randint(2, 6)))
             .replace("{form}", rng.choice(["alpha", "bravo", "charlie"]))
             .replace("{crs}", str(rng.randint(0, 359)))
             .replace("{kn}", str(rng.randint(12, 25)))
             .replace("{vis}", str(rng.randint(5, 15)))
             .replace("{op}", rng.choice(["transit", "an ASW screen", "an AEW patrol", "replenishment"])))

def render_state(f: dict, rng: random.Random) -> str:
    c = f["closing_mps"]
    if c > 5:
        motion = rng.choice([f"closing at {c:.0f} m/s", f"inbound, range shrinking {c:.0f} m/s"])
    elif c < -5:
        motion = rng.choice([f"opening at {-c:.0f} m/s", f"range opening {-c:.0f} m/s"])
    else:
        motion = rng.choice(["holding constant range", "range steady", "on a parallel course, range steady"])
    iff = rng.choice(IFF_HOSTILE if f["iff_hostile"] else IFF_FRIENDLY)
    body = rng.choice(TEMPLATES).format(
        r=f"{f['range_m'] / 1000:.1f}", alt=round(f["alt_m"]), vz=round(f["vz_mps"]),
        age=round(f["track_age_s"]), motion=motion, iff=iff,
        iff_lower=iff[0].lower() + iff[1:])
    n_noise = rng.randrange(3)  # 0~2 句
    for _ in range(n_noise):
        body += " " + _fill_noise(rng)
    return body

def build_question(kind: str, rng: random.Random) -> dict:
    if kind == "engage_auth":
        return {"type": "noul",
                "instructions": rng.choice([
                    "May this track be engaged now under the rules of engagement? Answer true if engagement is allowed.",
                    "Under the rules of engagement, is engagement of this track allowed now? True means yes."]),
                "criteria": {"true": rng.choice(AUTH_TRUE_TXT),
                             "false": rng.choice(AUTH_FALSE_TXT)}}
    opts = {k: rng.choice(v) for k, v in OPTS[kind].items()}
    items = list(opts.items())
    rng.shuffle(items)  # 防位置捷径
    return {"type": "choice", "instructions": rng.choice(INSTR_CHOICE),
            "criteria": dict(items)}

# ------------------------------------------------------------------ 主流程 ---

def make_rows(n_rows: int, n_families: int, seed: int) -> list[dict]:
    rng = random.Random(seed)
    # family 先验：各自的几何偏好带（标签与 family 独立：每 family 内标签均衡）
    fams = [{"alt_lo": rng.choice([40, 300, 2_000, 5_000]),
             "r_lo": rng.choice([900, 5_000, 15_000, 40_000])}
            for _ in range(n_families)]
    per_fam = n_rows // n_families
    rows: list[dict] = []
    seq = 0
    for fi in range(n_families):
        fam = fams[fi]
        fam_id = f"sc{fi:02d}"
        for j in range(per_fam):
            kind = KINDS[j % len(KINDS)]
            if kind == "engage_auth":
                label = (j // len(KINDS)) % 2 == 0
            else:
                keys = list(OPTS[kind])
                label = keys[(j // len(KINDS)) % len(keys)]
            # family 先验影响采样下界（保持各标签可达：仅压缩范围，不改真值区域）
            feats = sample_features(kind, label, fam, rng)
            if feats["range_m"] < fam["r_lo"]:
                cand = dict(feats, range_m=_u(rng, fam["r_lo"], min(62_000, fam["r_lo"] * 2)))
                if truth(kind, cand) == label:
                    feats = cand
            if feats["alt_m"] < fam["alt_lo"]:
                cand = dict(feats, alt_m=_u(rng, fam["alt_lo"], 12_000))
                if truth(kind, cand) == label:
                    feats = cand
            assert truth(kind, feats) == label, (kind, label, feats)
            seq += 1
            rows.append({
                "id": f"{SUITE}-{fam_id}-{seq:05d}",
                "suite": SUITE,
                "family": fam_id,
                "state": {"situation": render_state(feats, rng)},
                "question": build_question(kind, rng),
                "label": label,
                "target": label,
                "source": {"dataset": "aaw-synth-v1", "round": "round-1",
                           "generator": "milsim-rules", "kind": kind},
            })
    rng.shuffle(rows)
    return rows

def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rows", type=int, default=12_000)
    ap.add_argument("--families", type=int, default=48)
    ap.add_argument("--seed", type=int, default=20261020)
    ap.add_argument("--out", default="demos/jeff/data/rows.jsonl")
    args = ap.parse_args(argv)
    rows = make_rows(args.rows, args.families, args.seed)
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as fh:
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")
    # 标签分布速览
    from collections import Counter
    kinds: dict[str, Counter] = {}
    for row in rows:
        kinds.setdefault(row["source"]["kind"], Counter())[str(row["label"])] += 1
    for k, cnt in sorted(kinds.items()):
        print(f"{k:12s} {dict(cnt)}")
    print(f"{len(rows)} rows -> {out}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
