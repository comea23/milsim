"""域四：武器/弹道 量化对拍——飞行时间 / 命中-失的 / 脱靶量。

弹道轨迹（RMSE + Fréchet）已由 ``tools/cmp_movers.py`` 的 asm 族覆盖
（v0.13.66：ROUTE_ASM_GUIDANCE + 俯仰限位，Fréchet ~2 km 量级）。本工具
复用同一条发射链（AFSIM 侧真武器 ``Fire(track)`` + 弹轨迹采样；milsim 侧
MISSILE_MOVER + ROUTE_ASM_GUIDANCE），在 4 个弹族配置上补齐清单其余三项：

- **飞行时间**：首末样本时刻差（两侧同口径：发射 → 终止）。
- **脱靶量**：弹轨迹逐段到目标点的最小距离（点到线段 3D 投影，即 CPA）。
- **命中/失的**：CPA < lethal_radius(100 m) 判命中，两侧各自判、比一致率。

口径注记（报告里随行注出）：

1. milsim 弹在 CPA 穿越 lethal_radius 时注销（§5.17：进 fuse_radius 掷骰
   +CPA 穿越插补一次性事件，命中=弹体注销），轨迹末点 ≈ 终止半径 ⇒ 其
   CPA 是"终止脱靶量"；AFSIM 弹命中即删平台，末点同理。因此**命中组**两
   侧的脱靶量都是终止半径量级（≈100 m），真失的组（无制导偏置射/机动目标）
   留二期——清单原文的"命中率检验（比例 Z）"也属二期。
2. AFSIM 采样 0.1 s 节奏、milsim 1 s 步长：飞行时间直接用首末样本，不做
   插值（终止时刻本来就不是栅格对齐的量）。

用法::

    python tools/cmp_weapon.py            # 4 个 asm 配置全跑
    python tools/cmp_weapon.py --probe    # 抽 1 个配置探管线
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cmp_movers as cm  # noqa: E402

#: 判命中口径：与两侧终止半径一致（milsim lethal_radius 100 m，§5.17）。
LETHAL_M = 100.0
_OUT = ROOT / "demos" / "afsim_cmp" / "weapon"

COLS = [
    "cid", "t_flight_a_s", "t_flight_m_s", "dt_flight_s",
    "cpa_a_m", "cpa_m_m", "d_cpa_m",
    "hit_a", "hit_m", "verdict",
]

#: 结论段（域四全量实测后固化；改动指标口径时同步改）。
CONCLUSION = """
**发现（全 4 配置一致，v0.13.66 当日实测）**：AFSIM 侧 4/4 命中，milsim 侧
4/4 失的——DIVE 末段从目标头顶 560~670 m 掠过（r_xy 最小 18 m 但 3D 距离
超 lethal_radius），落地自毁（terminate_reason='落地'），Δt = −19~−56 s。

根因即 **§5.11.7 已定位的那一条**（弹体缺诱导阻力 → 末段不掉速 → 同倾角下
快 ~97 m/s → 视线转速 1.6 倍而 ω=a/v 只有 1/1.6 → 追不上视线），本组数据是
它在 **绕行几何**（ROUTE_ASM_GUIDANCE，v0.13.66）下的量化再现。插桩补充证
据：POPUP 定高律全推到 45 m/s² 过载上限，切 DIVE 时带 +158 m/s 爬升动量；
VERT_PN 指令 2 s 内到 −74→−276 m/s² 但被弹体上限钳住（过目标时垂向距离仍
672 m）。对照：无航路直扑 40 km 命中 28.8 m（§5.11 验收）；AFSIM 同一绕行
几何（1876 m / 950 m 高度）照样命中。

**milsim 零改动**——修法二选一已在 §9-36 挂拍板：给弹体补诱导阻力
（`C_di = C_l²/(π·e·AR)`），或接受"末段靠过载硬转"把这类弹的 `radial_accel`
标到 7 g 以上。
"""


def _cpa_3d(pts: list[tuple[float, float, float]],
            q: tuple[float, float, float]) -> float:
    """点列（含首末段投影）到目标点 q 的最小 3D 距离。"""
    best = math.inf
    for i in range(len(pts) - 1):
        ax, ay, az = pts[i]
        bx, by, bz = pts[i + 1]
        dx, dy, dz = bx - ax, by - ay, bz - az
        qx, qy, qz = q[0] - ax, q[1] - ay, q[2] - az
        seg2 = dx * dx + dy * dy + dz * dz
        if seg2 <= 1e-12:
            d = math.sqrt(qx * qx + qy * qy + qz * qz)
        else:
            u = max(0.0, min(1.0, (qx * dx + qy * dy + qz * dz) / seg2))
            ex, ey, ez = qx - u * dx, qy - u * dy, qz - u * dz
            d = math.sqrt(ex * ex + ey * ey + ez * ez)
        best = min(best, d)
    return best


def run_one(cfg: dict, scripts_dir: Path) -> dict:
    """跑两侧，算三项指标。目标点 = 锚点原点（两份想定目标都在锚点）。"""
    lat0, lng0 = cfg["anchor"]
    frame = cm.LocalFrame(lat0, lng0, 1000)

    a_geo = cm.run_afsim_asm(None, cfg, scripts_dir)
    a_rows = [(t, *frame.geo_to_world(la, ln), z)
              for t, la, ln, z, _v, _h in a_geo]
    m_rows = [(t, x, y, z) for t, x, y, z, _v, _h in cm.run_milsim(cfg, scripts_dir)]

    q = (0.0, 0.0, 0.0)
    cpa_a = _cpa_3d([(r[1], r[2], r[3]) for r in a_rows], q)
    cpa_m = _cpa_3d([(r[1], r[2], r[3]) for r in m_rows], q)
    t_a = a_rows[-1][0] - a_rows[0][0]
    t_m = m_rows[-1][0] - m_rows[0][0]
    hit_a, hit_m = cpa_a < LETHAL_M, cpa_m < LETHAL_M
    return {
        "cid": cfg["cid"],
        "t_flight_a_s": round(t_a, 2), "t_flight_m_s": round(t_m, 2),
        "dt_flight_s": round(t_m - t_a, 2),
        "cpa_a_m": round(cpa_a, 1), "cpa_m_m": round(cpa_m, 1),
        "d_cpa_m": round(cpa_m - cpa_a, 1),
        "hit_a": hit_a, "hit_m": hit_m,
        "verdict": "ok" if hit_a == hit_m and abs(t_m - t_a) <= 5.0 else "看一眼",
    }


def report(results: list[dict]) -> Path:
    _OUT.mkdir(parents=True, exist_ok=True)
    with open(_OUT / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        for r in results:
            w.writerow({k: r.get(k, "") for k in COLS})

    n = len(results)
    dts = [r["dt_flight_s"] for r in results]
    dcps = [r["d_cpa_m"] for r in results]
    agree = sum(1 for r in results if r["hit_a"] == r["hit_m"])
    mae_t = sum(abs(d) for d in dts) / n
    max_t = max(abs(d) for d in dts)
    lines = [
        "# 域四·武器/弹道 量化对拍（飞行时间 / 命中-失的 / 脱靶量）",
        "",
        "工具 `tools/cmp_weapon.py`；轨迹口径见 `tools/cmp_movers.py` asm 族"
        "（RMSE+Fréchet 已覆盖，此处不重复）。",
        "",
        "| 配置 | 飞行时 a/m (s) | Δt (s) | CPA a/m (m) | ΔCPA (m) | 命中 a/m | 判读 |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in results:
        lines.append(
            f"| {r['cid']} | {r['t_flight_a_s']} / {r['t_flight_m_s']} "
            f"| {r['dt_flight_s']:+.2f} | {r['cpa_a_m']} / {r['cpa_m_m']} "
            f"| {r['d_cpa_m']:+.1f} | {r['hit_a']} / {r['hit_m']} "
            f"| {r['verdict']} |")
    lines += [
        "",
        f"**汇总**（n={n}）：飞行时间 MAE={mae_t:.2f} s / MaxAE={max_t:.2f} s；"
        f"CPA 差范围 [{min(dcps):+.1f}, {max(dcps):+.1f}] m；"
        f"命中判定一致 {agree}/{n}。",
        CONCLUSION,
        "**口径注记**：两侧弹都在终止半径（lethal_radius=100 m）量级上注销——"
        "milsim §5.17 CPA 穿越掷骰注销，AFSIM 命中即删平台；故命中组的 CPA 是"
        "**终止脱靶量**而非自由飞 CPA。真失的组（无制导偏置射/机动目标）与命中"
        "率比例 Z 检验按清单划二期。",
        "",
    ]
    out = _OUT / "report.md"
    out.write_text("\n".join(lines), encoding="utf-8")
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--probe", action="store_true", help="抽 1 个配置探管线")
    ap.add_argument("--report-only", action="store_true",
                    help="从 summary.csv 重生成报告（不重跑仿真）")
    args = ap.parse_args(argv)

    if args.report_only:
        with open(_OUT / "summary.csv", encoding="utf-8") as f:
            results = [{**r, "hit_a": r["hit_a"] == "True",
                        "hit_m": r["hit_m"] == "True"}
                       for r in csv.DictReader(f)]
        for r in results:
            for k in ("t_flight_a_s", "t_flight_m_s", "dt_flight_s"):
                r[k] = float(r[k])
            for k in ("cpa_a_m", "cpa_m_m", "d_cpa_m"):
                r[k] = float(r[k])
        print(f"report regenerated: {len(results)} rows -> {report(results)}")
        return 0

    cfgs = cm._asm_cfgs()
    if args.probe:
        cfgs = cfgs[:1]
    scripts_dir = _OUT
    results = []
    for cfg in cfgs:
        print(f"· {cfg['cid']} ...", flush=True)
        results.append(run_one(cfg, scripts_dir))
        r = results[-1]
        print(f"  Δt={r['dt_flight_s']:+.2f}s  ΔCPA={r['d_cpa_m']:+.1f}m  "
              f"hit {r['hit_a']}/{r['hit_m']}")
    out = report(results)
    print(f"report: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
