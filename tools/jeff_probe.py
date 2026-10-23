"""域七·Jeff 决策模型实测：真实弹道态势 → zero-shot 判断准确率/校准/时延。

**目的**（2026-10-20 定案）：在把 Jeff 接进红蓝双层决策前，先用 milsim 真实
推演出来的态势数据测它的 zero-shot 水平——数据说话，决定直接接还是先微调。

**两段式运行**（两个解释器，别混）::

    # ① 采特征（milsim venv：有 milsim 包）
    python tools/jeff_probe.py --extract --out demos/jeff/flight_features.json

    # ② 发问（jeff venv，server 已起在 :8765）
    E:/jeff-repo/.venv/Scripts/python.exe tools/jeff_probe.py --probe \
        --features demos/jeff/flight_features.json

**态势来源（全部真实推演，非编造）**
- ASM 绕行弹 4 个采样点（climb/cruise/term/past）：``cmp_movers._asm_cfgs()[0]``
  的 milsim 弹道（d50km_a500：起点 50 km/500 m/231.5 m/s，航路绕行，
  目标=锚点 TGT_1），取 (t, R_xy, alt, vz, closing)。
- 巡航平移几何 3 个采样点：``scenarios/comm_track_jam.txt`` 里 SCOUT_1 对
  RED_TGT_1 的航迹（平行同速 ⇒ closing≈0，"过航/巡逻"类对照；R≈5.9 km）。

**真值规则（写死在 truth()，全部可从特征复算；改规则=改标签口径）**
- intent    closing>5 → strike_ingress；closing<-5 → egress；
            |closing|≤5 → transit_patrol
- engage_auth（noul）IFF 失败 且 R≤40 km 且 track_age≤6 s 且
            非(opening 且 R>20 km) → true，否则 false
- phase     opening → reset_and_assess；TTI=R/closing≤60 或 R≤15 km →
            active_engagement；TTI≤300 → heightened_alert；否则 normal_watch
- em        R>40 km → passive_emcon_hold；R≤15 km → full_active_illuminate；
            其余 → radar_on_lowpower_tws
- profile   alt≥2000 且 R>30 km → high_alt_cruise；alt≤500 且 R≤30 km →
            sea_skim；其余 → medium_dash
- terminal  opening → egress_climb；vz<-10 且 R≤10 km → terminal_dive；
            其余 → continue_ingress

track_age：ASM 弹取 2 s（本机雷达 1 s 刷新）；巡航平移 3 点取 2/4/8 s
（人为拉开，让 engage_auth 出现假标签样本）。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVER = "http://127.0.0.1:8765/v1/systemone"

# ---------------------------------------------------------------- 特征采集 ---

def _extract(out: Path) -> None:  # pragma: no cover - 手工运行
    sys.path.insert(0, str(ROOT / "src"))
    sys.path.insert(0, str(ROOT / "tools"))
    import cmp_movers as cm

    feats: list[dict] = []

    # --- ASM 绕行弹：4 个采样点（真实弹道） ---
    cfg = cm._asm_cfgs()[0]
    frame = cm.LocalFrame(*cfg["anchor"], 1000)
    tx, ty = frame.geo_to_world(*cfg["anchor"])  # TGT_1 = 锚点（milsim_script 定案）
    rows = cm.run_milsim(cfg, ROOT / "demos" / "jeff")
    hist = [(t, math.hypot(x - tx, y - ty), z) for t, x, y, z, _v, _h in rows]
    if len(hist) < 20:
        raise SystemExit(f"ASM 弹道太短：{len(hist)} 行")

    def feat_at(i: int, tag: str) -> None:
        t, r, z = hist[i]
        i0, i1 = max(0, i - 5), min(len(hist) - 1, i + 5)
        dt = hist[i1][0] - hist[i0][0]
        vz = (hist[i1][2] - hist[i0][2]) / dt if dt > 0 else 0.0
        closing = -(hist[i1][1] - hist[i0][1]) / dt if dt > 0 else 0.0
        feats.append({
            "src": "asm", "tag": tag, "t_s": round(t, 1),
            "range_m": round(r, 1), "alt_m": round(z, 1),
            "vz_mps": round(vz, 2), "closing_mps": round(closing, 2),
            "track_age_s": 2.0,
        })

    n = len(hist)
    for frac, tag in [(0.15, "climb"), (0.40, "cruise"), (0.85, "term")]:
        feat_at(int(frac * (n - 1)), tag)
    feat_at(n - 1, "past")

    # --- 巡航平移几何（comm_track_jam：SCOUT_1↔RED_TGT_1 平行同速） ---
    for k, age in enumerate([2.0, 4.0, 8.0]):
        feats.append({
            "src": "transit", "tag": f"flat{k + 1}", "t_s": 20.0 + 40 * k,
            "range_m": 5900.0, "alt_m": 3000.0, "vz_mps": 0.0,
            "closing_mps": 0.0, "track_age_s": age,
        })

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(feats, indent=1), encoding="utf-8")
    print(f"extracted {len(feats)} feature rows -> {out}")
    for f in feats:
        print("  ", f)

# ---------------------------------------------------------------- 真值规则 ---

def truth(kind: str, f: dict) -> str | bool:
    """返回值=OPTS 的**选项键**（choice 判分直接比对键）；engage_auth 返回 bool。"""
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

# ---------------------------------------------------------------- 问题构造 ---

OPTS = {
    "intent": {
        "ingress": "strike_ingress: flying inbound toward the defended point, an attack run",
        "egress": "egress: opening from the defended point, leaving the area",
        "transit": "transit_patrol: parallel or orbiting flight, neither closing nor opening",
    },
    "phase": {
        "watch": "normal_watch: threat far out, time to impact over 300 s, routine monitoring",
        "alert": "heightened_alert: time to impact 60 to 300 s, weapons crews alerted",
        "engage": "active_engagement: time to impact under 60 s or range under 15 km, firing window",
        "reset": "reset_and_assess: threat has flown past and is opening, stand down and assess",
    },
    "em": {
        "emcon": "passive_emcon_hold: range over 40 km, keep radar silent, wait for cueing",
        "tws": "radar_on_lowpower_tws: range 15 to 40 km, radar on in track-while-scan at low power",
        "illum": "full_active_illuminate: range under 15 km, full active illumination for missiles",
    },
    "profile": {
        "high": "high_alt_cruise: above 2000 m and far from the threat ring, cruise high",
        "skim": "sea_skim: below 500 m and inside 30 km, hug the deck under the radar horizon",
        "dash": "medium_dash: transition band, dash at medium altitude",
    },
    "terminal": {
        "continue": "continue_ingress: keep flying the approach, not yet in the terminal gate",
        "dive": "terminal_dive: descending and within 10 km, commit to the terminal dive",
        "climbout": "egress_climb: past the target and opening, climb out",
    },
}

KINDS_ASM = ["intent", "engage_auth", "phase", "em", "profile", "terminal"]
KINDS_TRANSIT = ["intent", "engage_auth"]

def _state_txt(f: dict) -> str:
    if f["closing_mps"] > 5:
        motion = "closing"
    elif f["closing_mps"] < -5:
        motion = "opening"
    else:
        motion = "constant range"
    return (f"Anti-air warfare. You defend a naval group. A radar track at "
            f"{f['range_m'] / 1000:.1f} km, closing speed {f['closing_mps']:.0f} m/s ({motion}), "
            f"altitude {f['alt_m']:.0f} m, vertical speed {f['vz_mps']:+.0f} m/s, "
            f"IFF interrogation failed (hostile), track age {f['track_age_s']:.0f} s. "
            f"Source: own search radar.")

def build_plan(feats: list[dict], model: str = "jeff-latest") -> tuple[list[dict], dict]:
    """每特征一个请求（server 的 state 是请求级）；返回 (请求列表, 期望标签)。"""
    per_q: list[dict] = []
    expected: dict[str, str | bool] = {}
    for f in feats:
        kinds = KINDS_ASM if f["src"] == "asm" else KINDS_TRANSIT
        qs: dict[str, dict] = {}
        for kind in kinds:
            name = f"{f['src']}_{f['tag']}_{kind}"
            if kind == "engage_auth":
                qs[name] = {
                    "type": "noul",
                    "instructions": ("May this track be engaged now under the rules of "
                                     "engagement? Answer true if engagement is allowed."),
                    "criteria": {
                        "true": ("engagement allowed: identified hostile, inside the "
                                 "40 km envelope, track fresh, not beyond 20 km while opening"),
                        "false": ("engagement not allowed: track stale, outside the "
                                  "envelope, or opening beyond 20 km"),
                    },
                }
            else:
                qs[name] = {"type": "choice",
                            "instructions": "Which option applies?",
                            "criteria": OPTS[kind]}
            expected[name] = truth(kind, f)
        per_q.append({"model": model,
                      "state": {"situation": _state_txt(f)},
                      "questions": qs})
    return per_q, expected

# ---------------------------------------------------------------- 发问与评分 ---

def _post(body: dict, timeout: float = 120.0) -> dict:
    req = urllib.request.Request(
        SERVER, data=json.dumps(body).encode("utf-8"),
        headers={"content-type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())

def _probe(features: Path, model: str = "jeff-latest") -> None:
    feats = json.loads(features.read_text(encoding="utf-8"))
    per_q, expected = build_plan(feats, model)

    rows: list[dict] = []
    t_all = time.perf_counter()
    for body in per_q:
        t0 = time.perf_counter()
        try:
            resp = _post(body)
        except Exception as exc:  # noqa: BLE001 - 报告层兜底
            print(f"REQUEST FAILED: {exc}")
            return
        dt = time.perf_counter() - t0
        for name, ans in resp["answers"].items():
            exp = expected[name]
            if ans["type"] == "noul":
                p = float(ans["noul"])
                got = p >= 0.5
                correct = got == exp
                rows.append({"name": name, "expected": exp, "chosen": got,
                             "conf": p if exp else 1.0 - p, "probs": {"p_true": p},
                             "correct": correct, "req_s": round(dt, 3)})
            else:
                chosen = ans["choice"]
                rows.append({"name": name, "expected": exp, "chosen": chosen,
                             "conf": ans.get("confidence"),
                             "probs": ans.get("probabilities"),
                             "correct": chosen == exp, "req_s": round(dt, 3)})
    wall = time.perf_counter() - t_all

    n = len(rows)
    acc = sum(r["correct"] for r in rows) / n if n else float("nan")
    # ECE：choice 用模型 confidence（校准量），noul 用 |p_true - label| 的对侧
    # ——统一口径：conf = 模型给自己所选答案的概率质量。
    bins: dict[int, list[float]] = {}
    for r in rows:
        c = r["conf"]
        if c is None:
            continue
        b = min(9, int(c * 10))
        bins.setdefault(b, []).append(1.0 if r["correct"] else 0.0)
    ece = sum(len(v) / n * abs(sum(v) / len(v) - (b + 0.5) / 10)
              for b, v in sorted(bins.items()))

    print("\n=== Jeff zero-shot 实测 ===")
    print(f"请求数 {len(per_q)}，题目 {n}，墙钟 {wall:.1f} s"
          f"（平均 {wall / max(1, len(per_q)) * 1000:.0f} ms/请求）")
    print(f"准确率 {sum(r['correct'] for r in rows)}/{n} = {acc:.1%}")
    print(f"ECE(binned, 0.5中心) ≈ {ece:.3f}")
    kinds: dict[str, list[dict]] = {}
    for r in rows:
        kinds.setdefault(r["name"].rsplit("_", 1)[1], []).append(r)
    for k, rs in sorted(kinds.items()):
        a = sum(r["correct"] for r in rs) / len(rs)
        print(f"  {k:14s} {sum(r['correct'] for r in rs)}/{len(rs)} = {a:.0%}")
    print("\n明细:")
    for r in rows:
        mark = "OK " if r["correct"] else "MISS"
        print(f"  [{mark}] {r['name']}: 期望 {r['expected']}  实答 {r['chosen']}  "
              f"conf={r['conf']}")
    misses = [r for r in rows if not r["correct"]]
    if misses:
        print("\n错题概率分布:")
        for r in misses:
            print(f"  {r['name']}: {r['probs']}")

def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--extract", action="store_true")
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--out", default="demos/jeff/flight_features.json")
    ap.add_argument("--features", default="demos/jeff/flight_features.json")
    ap.add_argument("--model", default="jeff-latest",
                    help="base 用 jeff-latest；挂适配器后传适配器名（如 aaw-defend）")
    args = ap.parse_args(argv)
    if args.extract:
        _extract(ROOT / args.out)
    if args.probe:
        _probe(ROOT / args.features, args.model)
    return 0

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
