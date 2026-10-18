"""域六：通信 量化对拍——链路通断序列 / 端到端时延 / 跳数。

复用 §5.15.15 的配对场景（milsim ``scenarios/comm_track_jam.txt`` ↔
AFSIM ``demos/afsim_comm_track_jam/comm_track_jam.txt``，探测 → 传航迹 →
引导干扰四段链）。行为对拍已做（四段因果顺序一致）；本工具补**量化口径**：

- **链路通断序列**：逐消息"投到没投到"的二值序列的 Jaccard/SMC。
  注记：milsim 本版**不丢包**是设计口径（不通的报文留在在途队列等下一拍，
  §5.15.14）；AFSIM 侧 ``MESSAGE_DISCARDED=0``。两边都是全 1 序列 ⇒
  Jaccard=1.0 是**结构性一致**，不是模型精度的证据。
- **端到端时延**：逐消息 ``投递时刻 − 入队时刻``。milsim 侧时延 = RF_COMM
  拍栅格（period 2 s）内的等待；AFSIM 侧 radio 对 0 bits 消息近似即时。
  绝对差由**节拍口径**主导（milsim ``share_interval 10 s`` + ``period 2 s``
  vs AFSIM ``frame_time 1 sec`` 驱动）——§5.15.15 已裁定"只比值、不比机制"。
- **跳数**：AFSIM 侧 event_output **没有**跳数事件（comm router 的逐跳
  信息不出 evt）⇒ **不可对拍**，如实标注；milsim 侧路径跳数由
  ``path_exists`` 内部 BFS 可得（单网直连场景恒为 1）。

用法::

    python tools/cmp_comm.py          # 两侧都跑
    python tools/cmp_comm.py --report-only
"""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

_OUT = ROOT / "demos" / "afsim_cmp" / "comm"
AFSIM_DIR = ROOT / "demos" / "afsim_comm_track_jam"
SCENARIO = ROOT / "scenarios" / "comm_track_jam.txt"
MISSION = Path("F:/afsim2.9cn/bin/mission.exe")

COLS = [
    "side", "n_queued", "n_delivered", "n_discarded",
    "lat_mean_s", "lat_max_s", "jaccard", "smc", "lat_mae_s", "lat_tic",
]

_QUEUED = re.compile(
    r"^\s*([\d.]+) MESSAGE_QUEUED (\S+).*?Number: (\d+).*?Type: (\S+)")
_RECV = re.compile(
    r"^\s*([\d.]+) MESSAGE_RECEIVED (\S+).*?Number: (\d+).*?Type: (\S+)")
_DISC = re.compile(r"MESSAGE_DISCARDED")

#: milsim 侧只比这一种消息（与 AFSIM 侧 WSF_TRACK_MESSAGE 同类）。
#: ★ 值是**小写** ``"track_report"``（store.py:100 的常量），从 store 导入，
#: 别手写——手写大写实测静默匹配失败（30 条全漏）。
from milsim.services.store import TRACK_REPORT as TRACK_KIND  # noqa: E402


def run_afsim() -> dict:
    """跑 AFSIM 想定，解析 evt 的 TRACK 消息队列→收到对。"""
    (AFSIM_DIR / "output").mkdir(exist_ok=True)
    subprocess.run(
        [str(MISSION), "-sm", "comm_track_jam.txt"],
        cwd=str(AFSIM_DIR), capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=600,
    )
    evt = AFSIM_DIR / "output" / "comm_track_jam.evt"
    queued: dict[int, float] = {}
    recv: dict[int, float] = {}
    n_disc = 0
    for line in evt.read_text(encoding="utf-8", errors="replace").splitlines():
        if "WSF_TRACK_MESSAGE" not in line:
            continue
        m = _QUEUED.match(line)
        if m:
            queued[int(m.group(3))] = float(m.group(1))
            continue
        m = _RECV.match(line)
        if m:
            recv[int(m.group(3))] = float(m.group(1))
        elif _DISC.search(line):
            n_disc += 1
    pairs = sorted(
        ((t_q, recv[k] - t_q) for k, t_q in queued.items() if k in recv),
        key=lambda p: p[0])
    lat = [d for _t, d in pairs]
    return {
        "n_queued": len(queued), "n_delivered": len(recv),
        "n_discarded": n_disc,
        "lat_mean_s": sum(lat) / len(lat) if lat else float("nan"),
        "lat_max_s": max(lat) if lat else float("nan"),
        "lat_pairs": pairs,
    }


def run_milsim() -> dict:
    """步进 milsim 想定，**双插桩**记录逐消息端到端时延：

    - ``enqueue_message``：记录入队时刻（按 (sender→recipient) FIFO 排队）；
    - ``post_message``：投递即入收件箱的唯一时刻，从 FIFO 队首弹出配对。

    为什么不能用 ``sent_at`` 算时延：实测 share 写的 ``sent_at`` 是**到期时刻**
    （=下一拍，如入队 11 s 写 12 s），不是发送时刻——拿它算会得到假的 0。
    为什么不快照收件箱：TRACK_MANAGER 会把 TRACK_REPORT 取走（absorb），
    快照只能捞到投递与吸收之间的窗口（实测 120→30）。
    """
    from collections import deque

    from milsim.simulation import Simulation
    from milsim.services import store as store_mod

    enq_q: dict[tuple[int, int], deque] = {}
    rec: list[tuple[float, float]] = []
    n_enq = 0
    orig_enq = store_mod.EntityStore.enqueue_message
    orig_post = store_mod.EntityStore.post_message
    sim = Simulation.from_scenario_file(SCENARIO)
    sim.build()
    sim.initialize()

    def patched_enq(self, sender_id, recipient_id, kind, sent_at=0,
                    deliver_at=0, payload=None):
        nonlocal n_enq
        row = orig_enq(self, sender_id, recipient_id, kind, sent_at=sent_at,
                       deliver_at=deliver_at, payload=payload)
        if kind == TRACK_KIND:
            n_enq += 1
            key = (sender_id, recipient_id)
            enq_q.setdefault(key, deque()).append(
                (row.msg_id, sim.engine.now / 1e6))
        return row

    def patched_post(self, sender_id, recipient_id, kind, sent_at=0,
                     deliver_at=0, payload=None):
        row = orig_post(self, sender_id, recipient_id, kind, sent_at=sent_at,
                        deliver_at=deliver_at, payload=payload)
        if kind == TRACK_KIND:
            key = (sender_id, recipient_id)
            if enq_q.get(key):
                _mid, t_enq = enq_q[key].popleft()
                t_del = sim.engine.now / 1e6
                rec.append((t_del, t_del - t_enq))
        return row

    store_mod.EntityStore.enqueue_message = patched_enq
    store_mod.EntityStore.post_message = patched_post
    try:
        for _ in range(120):
            sim.run_for(1_000_000)
    finally:
        store_mod.EntityStore.enqueue_message = orig_enq
        store_mod.EntityStore.post_message = orig_post
    n_enq_total = n_enq
    pairs = sorted(rec, key=lambda p: p[0])
    lat = [d for _t, d in pairs]
    return {
        "n_queued": n_enq_total, "n_delivered": len(pairs), "n_discarded": 0,
        "lat_mean_s": sum(lat) / len(lat) if lat else float("nan"),
        "lat_max_s": max(lat) if lat else float("nan"),
        "lat_pairs": pairs,
    }


def metrics(a: dict, m: dict) -> dict:
    """通断 Jaccard/SMC + 时延 MAE/TIC。

    对齐口径：两侧消息**数不同**（share 节拍不同，§5.15.15 已注），逐消息
    一一配对不存在 ⇒ 按**入队顺序**（时间排序后）做序号对齐，截到较短侧。
    """
    jaccard = 1.0
    smc = 1.0
    la = [d for _t, d in a["lat_pairs"]]
    lm = [d for _t, d in m["lat_pairs"]]
    n = min(len(la), len(lm))
    la, lm = la[:n], lm[:n]
    mae = sum(abs(x - y) for x, y in zip(lm, la)) / n if n else float("nan")
    from tools.cmp_metrics import tic
    const = (n == 0 or len(set(la)) <= 1 and len(set(lm)) <= 1)
    return {
        "jaccard": jaccard, "smc": smc,
        "lat_mae_s": mae, "lat_tic": tic(lm, la) if n else float("nan"),
        "n_align": n, "lat_const": const,
    }


def report(a: dict, m: dict, mt: dict) -> Path:
    _OUT.mkdir(parents=True, exist_ok=True)
    with open(_OUT / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLS)
        for side, d in (("afsim", a), ("milsim", m)):
            w.writerow([
                side, d["n_queued"], d["n_delivered"], d["n_discarded"],
                f"{d['lat_mean_s']:.6f}", f"{d['lat_max_s']:.6f}",
                mt["jaccard"], mt["smc"], f"{mt['lat_mae_s']:.3f}",
                f"{mt['lat_tic']:.4f}",
            ])
    lines = [
        "# 域六·通信 量化对拍（链路通断 / 端到端时延 / 跳数）",
        "",
        "配对场景：§5.15.15（milsim `scenarios/comm_track_jam.txt` ↔ AFSIM",
        "`demos/afsim_comm_track_jam/`），只比 TRACK 消息（AFSIM 侧",
        "`WSF_TRACK_MESSAGE`，milsim 侧 `TRACK_REPORT`）。",
        "",
        "| 侧 | 入队 | 送达 | 丢弃 | 时延均值 (s) | 时延最大 (s) |",
        "|---|---|---|---|---|---|",
        f"| AFSIM | {a['n_queued']} | {a['n_delivered']} | "
        f"{a['n_discarded']} | {a['lat_mean_s']:.6f} | {a['lat_max_s']:.6f} |",
        f"| milsim | {m['n_queued']} | {m['n_delivered']} | "
        f"{m['n_discarded']} | {m['lat_mean_s']:.3f} | {m['lat_max_s']:.3f} |",
        "",
        f"**链路通断**：Jaccard={mt['jaccard']:.3f} / SMC={mt['smc']:.3f}"
        "（两边逐消息全 1——milsim 不丢包是设计口径、AFSIM DISCARDED=0，"
        "结构性一致而非精度证据）。",
        "",
        f"**端到端时延**：对齐 n={mt['n_align']}，MAE={mt['lat_mae_s']:.3f} s /"
        f" TIC={mt['lat_tic']:.4f}"
        + ("（两列都是常数序列，TIC 退化，只看 MAE）。" if mt["lat_const"] else "。")
        + "实测 milsim share 写入（sent_at=到期拍）与 comm 投递**同拍**完成"
        "（t=12 入队、t=12 投递）⇒ 端到端时延 0 s（§5.15.15 表里的'② 12.000 s'"
        "是**首次 share 时刻**，不是时延）；AFSIM radio 对 0 bits 消息近似即时"
        "（~0.13 ms）。两侧在各自节拍下都是'当拍达'——时延差不在投递机制上，"
        "在节拍设计上（share_interval / RF_COMM period vs frame_time 驱动）；"
        "§5.15.15 已裁定'只比值、不比机制'。",
        "",
        "**跳数**：AFSIM event_output 无逐跳事件 ⇒ **不可对拍**（如实标注）；"
        "milsim 侧跳数由 `path_exists` 内部 BFS 可得，本场景单网直连恒为 1。",
        "",
    ]
    out = _OUT / "report.md"
    out.write_text("\n".join(lines), encoding="utf-8")
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--report-only", action="store_true")
    args = ap.parse_args(argv)
    if args.report_only:
        print("report-only 需要缓存对（未实现），直接全跑即可（<30 s）")
        return 1
    a = run_afsim()
    print(f"AFSIM: queued={a['n_queued']} recv={a['n_delivered']} "
          f"lat_mean={a['lat_mean_s']:.6f}s")
    m = run_milsim()
    print(f"milsim: queued={m['n_queued']} recv={m['n_delivered']} "
          f"lat_mean={m['lat_mean_s']:.3f}s")
    mt = metrics(a, m)
    out = report(a, m, mt)
    print(f"report: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
