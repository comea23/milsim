"""白方评判（v0.13.67）：红蓝推演的证据链 → 机械标尺 + 语义评判 + 验收对照。

为什么需要"白方"
----------------
红蓝两个 Jeff 适配器各带一支军队的立场，**不能拿它们给自己的表现打分**——
让运动员当裁判是自欺。白方是第三方：只看证据链（决策序列 + 引擎事件 +
终态）与金标准，不共享红蓝任何一方的态势文本。它是"AI 分析推演过程、
找功能缺失"那一步（框架第②步）的常驻化身。

三层，各自独立可核对
--------------------
**L0 机械标尺**（不含模型，可复算）：命中率、雷达开机/静默时长、干扰可见
时长、兜底拍数与平均置信度、指令拒绝数。数错了就是代码错了，不涉观点。

**L1 语义评判**（底座 Jeff，**不挂适配器**）：对每个维度问一个 noul 二值
问题 + 置信度。白方服务是**另一个实例**（默认 ``http://127.0.0.1:8766``，
启动时不给 ``JEFF_ADAPTERS`` ⇒ 纯底座 zero-shot）。置信度低于
:data:`JUDGE_CONF_MIN` 记 ``uncertain``——**不替它猜**，那不是评判是和稀泥。

**L2 验收对照**：金标准文档 §5 的验收要点逐条过，输出"成立 / 部分 / 不
成立 / 无法判定"四态 + 实测值。差距清单从这里出，交给用户拍板。

用法::

    python demos/jeff/white_judge.py                 # 评第一轮日志
    python demos/jeff/white_judge.py --log <json>    # 指定证据链

白方服务（另起一个终端）::

    PYTHONPATH=/e/jeff-repo/src JEFF_CHECKPOINT=E:/Jeff-Qwen3.5-0.8B \\
      PORT=8766 .venv_p3d/Scripts/python.exe -m jeff.server
"""

from __future__ import annotations

import argparse
import json
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "output"

#: 白方服务地址（纯底座实例；红蓝决策服务在 8765，别混）。
JUDGE_URL = "http://127.0.0.1:8766"
#: 语义评判的置信度门槛。低于它记 uncertain——zero-shot 底座本来就不是干这个
#: 的，硬给个 true/false 是制造确定性假象。
JUDGE_CONF_MIN = 0.60

#: L1 的四个评判维度。问型全部 noul（二值 + 置信度）——Jeff 的 System-1
#: 问型里这是唯一适合"是/否"裁决的。criteria 的写法与红蓝 skill 同一条
#: 纪律：**语义键 + 后果描述**，禁裸数字键。
JUDGMENTS: dict[str, dict[str, str]] = {
    "roe_respected": {
        "instructions": ("Did the blue defender respect the rules of engagement "
                         "in this replay? Answer true if it did."),
        "true": ("blue fired only at identified hostiles inside the engagement "
                 "envelope with fresh tracks, and held fire otherwise"),
        "false": ("blue fired outside the envelope, at stale tracks, or ignored "
                  "clear engagement opportunities inside the envelope"),
    },
    "ew_effective": {
        "instructions": ("Was the red electronic attack effective in this replay? "
                         "Answer true if the jamming measurably degraded the "
                         "blue radar."),
        "true": ("blue track quality dropped or tracks were lost while red "
                 "aircraft were inside jamming range, with received jamming "
                 "power in beam"),
        "false": ("blue radar kept clean tracks throughout despite red jamming "
                  "aircraft in range"),
    },
    "chain_closed": {
        "instructions": ("Did the replay close the full kill chain at least once? "
                         "Answer true if detect, track, decide, fire and intercept "
                         "all happened."),
        "true": ("blue detected and tracked the red aircraft, decided to fire, "
                 "launched a missile, and the missile reached the target"),
        "false": ("the chain broke at some step: no track, no fire decision, or "
                  "no missile reached any target"),
    },
    "credible": {
        "instructions": ("Is this replay credible as a military exercise overall? "
                         "Answer true if the observed behavior matches sound "
                         "tactics for both sides."),
        "true": ("both sides' decisions, timings and outcomes are consistent with "
                 "the equipment and the geometry"),
        "false": ("some behavior is tactically nonsensical given the equipment "
                  "and the geometry"),
    },
}


def ask_judge(digest: str, *, url: str = JUDGE_URL,
              timeout: float = 60.0) -> dict[str, dict]:
    """把一份证据摘要交给底座 Jeff，返回四维 noul 裁决。"""
    questions = {
        name: {"type": "noul",
               "instructions": spec["instructions"],
               "criteria": {"true": spec["true"], "false": spec["false"]}}
        for name, spec in JUDGMENTS.items()
    }
    body = json.dumps({
        "model": "jeff-qwen3.5-0.8b",
        "state": {"situation": digest},
        "questions": questions,
    }).encode()
    req = urllib.request.Request(
        url.rstrip("/") + "/v1/systemone", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        payload = json.loads(resp.read().decode())
    out: dict[str, dict] = {}
    for name, ans in (payload.get("answers") or {}).items():
        if ans.get("type") == "noul":
            # noul 问型回的是 p_true（键名就叫 "noul"，不是 probability——
            # 与 tools/jeff_probe.py 同一口径）；裁决取 p>=0.5，置信度取
            # "所选那一侧的概率质量"（与探针的 ECE 口径一致）。
            p = float(ans.get("noul", 0.0))
            choice = p >= 0.5
            conf = p if choice else 1.0 - p
        else:
            choice = ans.get("choice")
            conf = float(ans.get("confidence") or 0.0)
        out[name] = dict(
            choice=choice, confidence=conf,
            verdict=("true" if choice is True else "false" if choice is False
                     else "uncertain"),
            confident=conf >= JUDGE_CONF_MIN and choice is not None,
        )
    return out


def mechanical(data: dict) -> dict:
    """L0：从证据链算标尺。只信 log 里有的数，不补拍。"""
    decisions = data["decisions"]
    events = data["events"]
    tick_s = 5.0
    blue = [r for r in decisions if r["side"] == "blue"]
    red = [r for r in decisions if r["side"] == "red"]

    def fallbacks(rows: list[dict], keys: tuple[str, ...]) -> int:
        return sum(1 for r in rows if any(r.get(k) for k in keys))

    def mean_conf(rows: list[dict], keys: tuple[str, ...]) -> float:
        vals = [r[k] for r in rows for k in keys if r.get(k) is not None]
        return sum(vals) / len(vals) if vals else 0.0

    fires = [r["fire"] for r in blue if r.get("fire") and r["fire"].get("target")]
    fire_ok = [f for f in fires if f.get("ok")]
    launched = [e for e in events if e.get("kind") == "weapon_fired"]
    hits = [e for e in events
            if e.get("kind") == "engagement_resolved" and e.get("hit")]
    destroyed = [e for e in events
                 if e.get("kind") == "engagement_resolved" and e.get("destroyed")]
    damaged = [e for e in events
               if e.get("kind") == "engagement_resolved"
               and not e.get("destroyed") and e.get("fraction", 0) > 0]
    rejected = [e for e in events if e.get("kind") == "command_rejected"]

    # 雷达开机时长：每拍的 radar_on 即该拍之后的真实辐射态（驱动层从组件
    # 同步初值、em 决策经 sensor on/off 执行后回写）。em_exec 只记状态切换
    # 那一刻，不能用来累计时长。只有蓝方拍有这一项。
    on_beats = sum(1 for r in blue if r.get("radar_on"))
    jam_beats = sum(1 for r in blue if (r.get("jam_w") or 0.0) > 0.0)
    # ★ 致盲时长 = 首次建航**之后**无航迹的累计拍数。不能用"航迹年龄"测：
    #   掉航迹时航迹表是**空的**，年龄恒为 0——那是仪器选错了，不是没致盲
    #   （v0.13.67 第一轮就这么误判过一次：干扰可见 335 s 而"陈旧 0 s"，
    #   实际 t=280 起掉航迹后再未建航）。
    first_track_t: float | None = None
    dark_beats = 0
    longest_dark = 0
    streak = 0
    for r in blue:
        has = bool(r.get("contacts"))
        if has and first_track_t is None:
            first_track_t = r["t"]
        if first_track_t is not None and not has:
            dark_beats += 1
            streak += 1
            longest_dark = max(longest_dark, streak)
        else:
            streak = 0

    return dict(
        sim_seconds=data.get("sim_seconds", 0.0),
        beats=dict(blue=len(blue), red=len(red), total=len(decisions)),
        fallback=dict(
            blue=fallbacks(blue, ("phase_fb", "em_fb", "auth_fb")),
            red=fallbacks(red, ("intent_fb", "profile_fb")),
        ),
        confidence=dict(
            phase=mean_conf(blue, ("phase_conf",)),
            em=mean_conf(blue, ("em_conf",)),
            auth=mean_conf(blue, ("auth_conf",)),
            intent=mean_conf(red, ("intent_conf",)),
            profile=mean_conf(red, ("profile_conf",)),
        ),
        fires=dict(issued=len(fires), ok=len(fire_ok),
                   in_flight=len(launched) - len([e for e in events
                                                  if e.get("kind") == "engagement_resolved"])),
        hits=dict(count=len(hits), damaged=len(damaged), destroyed=len(destroyed)),
        rejected=len(rejected),
        radar=dict(on_s=on_beats * tick_s, total_s=len(blue) * tick_s,
                   jam_visible_s=jam_beats * tick_s,
                   dark_s=dark_beats * tick_s,
                   longest_dark_s=longest_dark * tick_s,
                   first_track_t=first_track_t),
    )


def acceptance(m: dict, data: dict) -> list[dict]:
    """L2：金标准验收要点逐条对照（scenario_growler_vs_growler.md §5）。"""
    events = data["events"]
    rows: list[dict] = []

    det = "无法判定"
    if m["radar"]["first_track_t"] is None:
        det = "不成立"
        ev = "整场未建航"
    elif m["radar"]["dark_s"] >= 30.0:
        det = "成立"
        ev = (f"首次建航 t={m['radar']['first_track_t']:.0f} s；此后致盲 "
              f"{m['radar']['dark_s']:.0f} s（最长连续 "
              f"{m['radar']['longest_dark_s']:.0f} s）；干扰可见 "
              f"{m['radar']['jam_visible_s']:.0f} s")
    elif m["radar"]["dark_s"] > 0.0:
        det = "部分"
        ev = f"致盲 {m['radar']['dark_s']:.0f} s（<30 s 门槛）"
    else:
        det = "部分"
        ev = (f"干扰可见 {m['radar']['jam_visible_s']:.0f} s 但航迹未断"
              "（可能烧穿或干扰功率不足）")
    rows.append(dict(item="探测链：干扰致盲/退化（烧穿）", verdict=det, evidence=ev))

    if m["fires"]["ok"] == 0:
        verdict = "不成立"
    elif m["hits"]["count"] > 0:
        verdict = "成立"
    else:
        verdict = "部分"
    rows.append(dict(item="接战链：发射决策 + 末段拦截", verdict=verdict,
                     evidence=(f"开火 {m['fires']['issued']} 次（成功 "
                               f"{m['fires']['ok']}）；命中 {m['hits']['count']}；"
                               f"击伤 {m['hits']['damaged']}；击毁 "
                               f"{m['hits']['destroyed']}；截止时在飞 "
                               f"{m['fires']['in_flight']}")))

    total_fb = m["fallback"]["blue"] + m["fallback"]["red"]
    verdict = "成立" if total_fb == 0 else ("部分" if total_fb <= 6 else "不成立")
    rows.append(dict(item="决策链：Jeff 三问型全程在线", verdict=verdict,
                     evidence=(f"兜底 {total_fb} 拍（蓝 "
                               f"{m['fallback']['blue']} / 红 "
                               f"{m['fallback']['red']}）")))

    rows.append(dict(item="结束条件：计分/终局事件", verdict="无法判定",
                     evidence=(f"engagement_resolved {len(events)} 条；"
                               "计分终局层未建（gap G8）")))
    return rows


def digest_text(m: dict, data: dict) -> str:
    """L1 的输入：一页英文证据摘要（Jeff 的训练语言）。"""
    events = data.get("events", [])
    fire_lines = [f"t={e['t']:.0f}s {e.get('missile')} -> {e.get('tgt')}"
                  for e in events if e.get("kind") == "weapon_fired"]
    hit_lines = [f"t={e['t']:.0f}s {e.get('attacker')} hit {e.get('target')} "
                 f"(damage {e.get('fraction')}, destroyed={e.get('destroyed')})"
                 for e in events if e.get("kind") == "engagement_resolved" and e.get("hit")]
    return (
        "Wargame replay under review (milsim, 0.1 s physics steps). "
        "Golden scenario: a blue SAM site (search radar on 4 GHz plus SAM "
        "missiles, rules of engagement allow engaging identified hostiles "
        "inside 40 km with fresh tracks) defends a point; red sends two "
        "EA-18G-class jammer aircraft to suppress the site's search radar. "
        f"Replay ran {m['sim_seconds']:.0f} s: {m['beats']['blue']} blue "
        f"decision beats (every 5 s), {m['beats']['red']} red beats. "
        f"Blue fired {m['fires']['issued']} missiles ({m['fires']['ok']} launch "
        f"orders accepted, {m['fires']['in_flight']} still in flight at "
        f"cutoff); {m['hits']['count']} hits, {m['hits']['damaged']} "
        f"targets damaged, {m['hits']['destroyed']} destroyed. "
        f"Radar radiating {m['radar']['on_s']:.0f} s of {m['radar']['total_s']:.0f} s; "
        f"jamming visible in beam for {m['radar']['jam_visible_s']:.0f} s; "
        f"radar dark (no tracks after first detection) for "
        f"{m['radar']['dark_s']:.0f} s. "
        f"Low-confidence fallbacks: blue {m['fallback']['blue']}, red "
        f"{m['fallback']['red']}. "
        f"Fire events: {'; '.join(fire_lines) or 'none'}. "
        f"Hit events: {'; '.join(hit_lines) or 'none'}. "
        f"Final state: {data.get('final_state', 'n/a')}"
    )


def render(m: dict, verdicts: dict[str, dict], rows: list[dict],
           data: dict) -> str:
    lines = [
        "# 白方评判报告（white judge）",
        "",
        f"- 证据链：`{data.get('scenario', '?')}`，推演 {m['sim_seconds']:.0f} s",
        "- 评判模型：Jeff 底座 zero-shot（**不挂适配器**，独立于红蓝双方）",
        f"- 置信度门槛：{JUDGE_CONF_MIN:.2f}（低于它记 uncertain，不猜）",
        "",
        "## L0 机械标尺（可复算）",
        "",
        "| 指标 | 值 |",
        "|---|---|",
        f"| 决策拍 | 蓝 {m['beats']['blue']} / 红 {m['beats']['red']} |",
        f"| 兜底拍 | 蓝 {m['fallback']['blue']} / 红 {m['fallback']['red']} |",
        f"| 平均置信度 | phase {m['confidence']['phase']:.2f} / em "
        f"{m['confidence']['em']:.2f} / auth {m['confidence']['auth']:.2f} / "
        f"intent {m['confidence']['intent']:.2f} / profile "
        f"{m['confidence']['profile']:.2f} |",
        f"| 开火 | {m['fires']['issued']} 次（接令 {m['fires']['ok']}，"
        f"截止时在飞 {m['fires']['in_flight']}） |",
        f"| 交战结果 | 命中 {m['hits']['count']} / 击伤 "
        f"{m['hits']['damaged']} / 击毁 {m['hits']['destroyed']} |",
        f"| 指令拒绝 | {m['rejected']} 条 |",
        f"| 雷达开机 | {m['radar']['on_s']:.0f} s / {m['radar']['total_s']:.0f} s |",
        f"| 干扰可见 | {m['radar']['jam_visible_s']:.0f} s |",
        f"| 致盲（首次建航后无航迹） | {m['radar']['dark_s']:.0f} s"
        f"（最长连续 {m['radar']['longest_dark_s']:.0f} s） |",
        "",
        "## L1 语义评判（底座 noul 四问）",
        "",
        "| 维度 | 裁决 | 置信度 |",
        "|---|---|---|",
    ]
    for name, v in verdicts.items():
        mark = v["verdict"] if v["confident"] else f"{v['verdict']}（uncertain）"
        lines.append(f"| {name} | {mark} | {v['confidence']:.2f} |")
    if not verdicts:
        lines.append("| — | 白方服务不在线，L1 跳过 | — |")
    lines += [
        "",
        "## L2 金标准验收对照（scenario_growler_vs_growler.md §5）",
        "",
        "| 验收要点 | 判定 | 实测 |",
        "|---|---|---|",
    ]
    for r in rows:
        lines.append(f"| {r['item']} | {r['verdict']} | {r['evidence']} |")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="白方评判：红蓝推演证据链 → 评分")
    ap.add_argument("--log", default=str(OUT / "first_round_log.json"))
    ap.add_argument("--judge-url", default=JUDGE_URL)
    ap.add_argument("--no-l1", action="store_true",
                    help="只跑 L0/L2（白方服务不在线时用）")
    args = ap.parse_args()

    data = json.loads(Path(args.log).read_text(encoding="utf-8"))
    m = mechanical(data)
    rows = acceptance(m, data)

    verdicts: dict[str, dict] = {}
    if not args.no_l1:
        try:
            verdicts = ask_judge(digest_text(m, data), url=args.judge_url)
        except (urllib.error.URLError, OSError, ValueError) as exc:
            print(f"白方服务不可用（{args.judge_url}）：{exc}；L1 跳过")

    report = render(m, verdicts, rows, data)
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "white_judge.md"
    path.write_text(report, encoding="utf-8")
    print(report)
    print(f"\n报告：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
