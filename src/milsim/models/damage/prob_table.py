"""查表命中概率：AFSIM 大样本标定表的运行期接口（§5.17.10）。

它是什么
--------
§5.17.5 的瑞利骨架是 Phase 1 的解析占位——单调性对，系数是拍的。
Phase 2 的标定在 AFSIM 2.9 engage 上做完了：9720 格 × 30 seeds 的
truth 主表（291,573 run）加 108 格 × 6 干扰档的系数批（22,680 run），
产出 :mod:`prob_table.json`（同目录，316 KB）。本件就是那张表的
运行期接口：给四维能力参数与几何量，返回查表概率。

表的语义（与标定工具 calibrate_v2.py 逐字一致）
----------------------------------------------
- **主表 ``p_hit``**：键 (g_m, v_m, g_t, v_t) → P(|CPA| ≤ R)，
  R = 30 / 100 / 300 m 三档。**无条件口径**——弹没进 3 km 锚点
  （追不上、全程无交会）的 run 自然计 miss，不另设"进圈率"通道。
- **系数层**：主表是 truth 制导（弹看真实目标）的**运动学能力包络**；
  感知与电子战的代价全部走乘性系数——

  ::

      K_link        = p(perception clean) / p(truth)      数据链代价
      K_jam(j)      = p(perception + 干扰 j) / p(clean)   干扰压制
      K_rcs(x)      = p(perception, RCS x) / p(RCS 1 m²)  目标 RCS 通道
      p_联合        = p_hit × K_link × K_jam × K_rcs

  注意 K_rcs 的方向：RCS 在这里是**跟踪质量**通道——隐身（小 RCS）降
  命中、大 RCS 升命中（perception 链下目标更容易被稳定跟踪），不是
  "目标更难打"的弹道通道。

查询协议：最近档，不插值
------------------------
四条轴都是**仿真档位**（g: 0,1..6,10,20；v: 0.2..10 km/s），不是连续
量——概率面在档间本就非线性（低 g_m × 高 g_t 象限整片塌零，是"追不上"
的阶跃，不是光滑下降）。对档间值做线性插值会**伪造中间概率**，所以
查询一律映射**最近档**。要改分辨率，回标定工具加档重跑，不在运行期
脑补。

唯一做插值的地方是杀伤半径 R：30/100/300 三档之间在 log(R) 上线性
内插（三档是同一格的三次统计，插的是"同一物理格内引信阈值"的连续量，
语义成立）；区间外按端点斜率外推，clamp 到 [0, 1]。

K 表的粗网格与回退
------------------
系数批是 108 格的粗网格（g_m{1,3,6,20} × v_m{2,4,8} × g_t{0,2,4} ×
v_t{0.2,2,6}），查询同样最近档映射。truth p=0 的格 K **无定义**
（分母为零，json 里是 null）：运行期回退到**同 g_m 组**的非空 K
中位数——g_m 是 K 的最强解释变量（表散布随 g_m 分层），其余三维的
组内散布次要。整组全空（g_m=0 不在 K 网格里）按 1.0 = 无效应。

随机性
------
本模块是**纯查表**：不掷骰、不碰仿真状态。掷骰在 ``Warhead._judge``
（``Stream.ENGAGE`` + ``bernoulli``，§5.17.5）——概率的生产与消费
分家，表可以单独测试、单独替换。
"""

from __future__ import annotations

import json
import math
import statistics
from functools import lru_cache
from pathlib import Path
from typing import Any

#: 数据文件与本模块同目录（随包分发，标定工具重跑后由导出脚本再生）。
_DATA_PATH = Path(__file__).with_name("prob_table.json")

#: 主表杀伤半径三档（米）——log 插值的支撑点。
KILL_RADII = (30.0, 100.0, 300.0)

#: K 表的干扰功率档（瓦；0 W = 无干扰 = 恒 1.0，不存表）。
JAM_POWERS = (1.0, 10.0, 30.0, 100.0, 1000.0)

#: K 表的 RCS 档（m²；基档 1.0 = 恒 1.0，不存表）。log 十倍程最近档。
RCS_LEVELS = (0.01, 0.1, 10.0)


def _nearest(values: tuple[float, ...] | list[float], x: float) -> int:
    """最近档下标。并列取低档（保守：低档的命中概率不高于高档）。"""
    best, best_d = 0, float("inf")
    for i, v in enumerate(values):
        d = abs(x - v)
        if d < best_d - 1e-12:
            best, best_d = i, d
    return best


class ProbTable:
    """标定表的运行期接口。线程安全：加载后只读。"""

    def __init__(self, data: dict[str, Any]) -> None:
        meta = data["meta"]
        ax = meta["axes"]
        self._g_axis: tuple[float, ...] = tuple(ax["g_m"])
        self._vm_axis: tuple[float, ...] = tuple(ax["v_m_kms"])
        self._gt_axis: tuple[float, ...] = tuple(ax["g_t"])
        self._vt_axis: tuple[float, ...] = tuple(ax["v_t_kms"])
        self._radii: tuple[float, ...] = tuple(meta["kill_radii_m"])
        self._jams: tuple[float, ...] = tuple(
            p for p in meta["jam_powers_w"] if p > 0.0)
        # 键 (gi, vi, gi_t, vi_t) → [p30, p100, p300]
        self._p: dict[tuple[int, int, int, int], list[float]] = {}
        for row in data["p_hit"]:
            g_m, v_m, g_t, v_t = row[0], row[1], row[2], row[3]
            key = (self._g_axis.index(g_m), self._vm_axis.index(v_m),
                   self._gt_axis.index(g_t), self._vt_axis.index(v_t))
            self._p[key] = [float(row[4]), float(row[5]), float(row[6])]
        # K 层：粗网格键 → [K_link_p100, K_link_p300, K_jam 展平列,
        # K_rcs 展平列]；k_link / k_jam / k_rcs 三个 json 数组按键合并。
        # None 原样保留，回退在查询时按 g_m 组做（组内中位在 __init__
        # 预聚合，运行期零开销）。
        self._k: dict[tuple[int, int, int, int], list[Any]] = {}
        for row in data["k_link"]:
            key = (self._g_axis.index(row[0]), self._vm_axis.index(row[1]),
                   self._gt_axis.index(row[2]), self._vt_axis.index(row[3]))
            self._k[key] = [row[4], row[5], None, None]
        for row in data["k_jam"]:
            key = (self._g_axis.index(row[0]), self._vm_axis.index(row[1]),
                   self._gt_axis.index(row[2]), self._vt_axis.index(row[3]))
            entry = self._k.setdefault(key, [None, None, None, None])
            entry[2] = list(row[4:])
        for row in data["k_rcs"]:
            key = (self._g_axis.index(row[0]), self._vm_axis.index(row[1]),
                   self._gt_axis.index(row[2]), self._vt_axis.index(row[3]))
            entry = self._k.setdefault(key, [None, None, None, None])
            entry[3] = list(row[4:])
        self._k_link_med = self._group_medians('link')
        self._k_jam_med = self._group_medians('jam')
        self._k_rcs_med = self._group_medians('rcs')

    # -- 构造辅助 ----------------------------------------------------------

    def _group_medians(self, kind: str) -> dict[int, list[float | None]]:
        """按 g_m 组聚合非空中位。``kind='link'`` 聚 K_link 两列；
        ``'jam'`` / ``'rcs'`` 聚各自的展平列（半径优先，布局见各方法）。"""
        grouped: dict[int, list[list[Any]]] = {}
        for key, entry in self._k.items():
            grouped.setdefault(key[0], []).append(entry)
        if kind == "link":
            n_slots, slot = 2, 0
        elif kind == "jam":
            n_slots, slot = 2 * len(self._jams), 2
        else:
            n_slots, slot = 2 * len(RCS_LEVELS), 3
        out: dict[int, list[float | None]] = {}
        for g, entries in grouped.items():
            vals: list[float | None] = []
            for s in range(n_slots):
                if kind == "link":
                    col = [e[s] for e in entries if e[s] is not None]
                else:
                    col = [e[slot][s] for e in entries
                           if e[slot] is not None and e[slot][s] is not None]
                vals.append(statistics.median(col) if col else None)
            out[g] = vals
        return out

    # -- 主表 --------------------------------------------------------------

    def axes(self) -> dict[str, tuple[float, ...]]:
        """四条轴的档位（测试与诊断用）。"""
        return {"g_m": self._g_axis, "v_m_kms": self._vm_axis,
                "g_t": self._gt_axis, "v_t_kms": self._vt_axis}

    def p_hit(self, g_m: float, v_m_kms: float, g_t: float,
              v_t_kms: float, kill_radius_m: float) -> float:
        """主表查表：最近档 × log(R) 内插，clamp [0, 1]。

        g/v 轴语义见模块头；``kill_radius_m`` 在 30/100/300 之间 log
        线性内插，区间外沿端点斜率外推（概率单调性由 clamp 兜底）。
        """
        gi = _nearest(self._g_axis, g_m)
        vi = _nearest(self._vm_axis, v_m_kms)
        gi_t = _nearest(self._gt_axis, g_t)
        vi_t = _nearest(self._vt_axis, v_t_kms)
        key = (gi, vi, gi_t, vi_t)
        row = self._p.get(key)
        if row is None:
            # 主表是全网格（9720 = 9×10×9×12），最近档必命中；防御分支。
            return 0.0
        ps = row
        lr = math.log(kill_radius_m) if kill_radius_m > 0.0 else float("-inf")
        # 杀伤半径 ≤ 0：必不中（与解析模型同语义）。
        if kill_radius_m <= 0.0:
            return 0.0
        lrs = [math.log(r) for r in self._radii]
        if lr <= lrs[0]:
            # 低端外推：沿 30→100 的 log 斜率，clamp 防负。
            slope = ((ps[1] - ps[0]) / (lrs[1] - lrs[0])) if lrs[1] > lrs[0] else 0.0
            p = ps[0] + slope * (lr - lrs[0])
        elif lr >= lrs[-1]:
            slope = ((ps[2] - ps[1]) / (lrs[2] - lrs[1])) if lrs[2] > lrs[1] else 0.0
            p = ps[1] + slope * (lr - lrs[1]) if lr > lrs[1] else ps[1]
            if lr > lrs[2]:
                p = ps[2] + slope * (lr - lrs[2])
        else:
            for i in range(len(lrs) - 1):
                if lrs[i] <= lr <= lrs[i + 1]:
                    w = (lr - lrs[i]) / (lrs[i + 1] - lrs[i])
                    p = ps[i] + w * (ps[i + 1] - ps[i])
                    break
            else:  # pragma: no cover —— 区间划分完备，防御分支
                p = ps[0]
        return min(1.0, max(0.0, p))

    # -- 系数层 ------------------------------------------------------------

    def k_link(self, g_m: float, v_m_kms: float, g_t: float,
               v_t_kms: float, kill_radius_m: float) -> float:
        """数据链代价系数（perception clean / truth）。"""
        gi, vi, gi_t, vi_t, ri = self._k_index(
            g_m, v_m_kms, g_t, v_t_kms, kill_radius_m)
        entry = self._k.get((gi, vi, gi_t, vi_t))
        raw = entry[ri] if entry is not None else None
        if raw is None:
            med = self._k_link_med.get(gi, [None, None])[ri]
            raw = 1.0 if med is None else med
        return float(raw)

    def k_jam(self, g_m: float, v_m_kms: float, g_t: float, v_t_kms: float,
              jam_power_w: float, kill_radius_m: float) -> float:
        """干扰压制系数（perception(jam) / perception clean）。

        ``jam_power_w ≤ 0`` = 无干扰 → 1.0（表里不存恒等档）。
        """
        if jam_power_w <= 0.0:
            return 1.0
        ji = _nearest(self._jams, jam_power_w)
        gi, vi, gi_t, vi_t, ri = self._k_index(
            g_m, v_m_kms, g_t, v_t_kms, kill_radius_m)
        entry = self._k.get((gi, vi, gi_t, vi_t))
        raw = None
        if entry is not None:
            jam_cols = entry[2]
            # K_jam 行是**半径优先**展平列：[j1..j1k 的 p100 五档,
            #                                     j1..j1k 的 p300 五档]，
            # 下标 = ri * len(JAM_POWERS) + ji。
            if jam_cols is not None:
                idx = ri * len(self._jams) + ji
                if idx < len(jam_cols):
                    raw = jam_cols[idx]
        if raw is None:
            med = self._k_jam_med.get(gi, [None] * (2 * len(self._jams)))
            idx = ri * len(self._jams) + ji
            raw = 1.0 if idx >= len(med) or med[idx] is None else med[idx]
        return float(raw)

    def _k_index(self, g_m: float, v_m_kms: float, g_t: float, v_t_kms: float,
                 kill_radius_m: float) -> tuple[int, int, int, int, int]:
        gi = _nearest(self._g_axis, g_m)
        vi = _nearest(self._vm_axis, v_m_kms)
        gi_t = _nearest(self._gt_axis, g_t)
        vi_t = _nearest(self._vt_axis, v_t_kms)
        # 半径只有 p100/p300 两列（p30 的系数批没跑）：<100 用 p100 列。
        ri = 1 if kill_radius_m > 100.0 else 0
        return gi, vi, gi_t, vi_t, ri

    def k_rcs(self, g_m: float, v_m_kms: float, g_t: float, v_t_kms: float,
              rcs_m2: float, kill_radius_m: float) -> float:
        """目标 RCS 修正系数（perception(rcs) / perception(1 m²)）。

        ``rcs_m2 ≤ 0`` 或命中基档 1.0 → 1.0（表里不存恒等档）。
        档轴是 log 十倍程（0.01 / 0.1 / 10）——**隐身降命中、大 RCS 升
        命中**（perception 链下 RCS 大 = 跟踪更稳）：RCS 在这里是跟踪
        质量通道，不是"目标更难打"通道。
        """
        if rcs_m2 <= 0.0 or rcs_m2 == 1.0:
            return 1.0
        xi = _nearest([math.log10(x) for x in RCS_LEVELS], math.log10(rcs_m2))
        gi, vi, gi_t, vi_t, ri = self._k_index(
            g_m, v_m_kms, g_t, v_t_kms, kill_radius_m)
        entry = self._k.get((gi, vi, gi_t, vi_t))
        raw = None
        if entry is not None:
            rcs_cols = entry[3]
            # K_rcs 行是**半径优先**展平列：[0.01/0.1/10 的 p100 三档,
            #                                     0.01/0.1/10 的 p300 三档]。
            if rcs_cols is not None:
                idx = ri * len(RCS_LEVELS) + xi
                if idx < len(rcs_cols):
                    raw = rcs_cols[idx]
        if raw is None:
            med = self._k_rcs_med.get(gi, [None] * (2 * len(RCS_LEVELS)))
            idx = ri * len(RCS_LEVELS) + xi
            raw = 1.0 if idx >= len(med) or med[idx] is None else med[idx]
        return float(raw)

    def combined(self, g_m: float, v_m_kms: float, g_t: float, v_t_kms: float,
                 kill_radius_m: float, jam_power_w: float = 0.0,
                 rcs_m2: float = 1.0) -> float:
        """联合概率 = p_hit × K_link × K_jam × K_rcs，clamp [0, 1]。"""
        p = self.p_hit(g_m, v_m_kms, g_t, v_t_kms, kill_radius_m)
        p *= self.k_link(g_m, v_m_kms, g_t, v_t_kms, kill_radius_m)
        p *= self.k_jam(g_m, v_m_kms, g_t, v_t_kms, jam_power_w, kill_radius_m)
        p *= self.k_rcs(g_m, v_m_kms, g_t, v_t_kms, rcs_m2, kill_radius_m)
        return min(1.0, max(0.0, p))


@lru_cache(maxsize=1)
def load_default() -> ProbTable:
    """包内标定表的单例。文件随包分发，加载一次后只读。"""
    with _DATA_PATH.open(encoding="utf-8") as f:
        return ProbTable(json.load(f))


__all__ = ["JAM_POWERS", "KILL_RADII", "RCS_LEVELS", "ProbTable",
           "load_default"]
