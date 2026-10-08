"""数值核对《大步长雷达探测与干扰 —— 行为组件规格书 v2》。

本脚本**只做算术**，不依赖 milsim 的任何模块。目的是在动手写代码之前，
把规格书里的每条公式与其参考数值逐条算一遍——公式对不对，不该靠读，
该靠算。

用法::

    python tools/verify_radar_spec.py
"""

from __future__ import annotations

import math

import numpy as np

C = 299792458.0
K_B = 1.380650424e-23

#: 规格书 A.3 FlushStub 里的常数。
FLUSH_K = 1.7570265424158548e-09


def db(x: float) -> float:
    return 10.0 * math.log10(x)


def lin(x: float) -> float:
    return 10.0 ** (x / 10.0)


def flush_stub(dist_km: float, freq_hz: float) -> float:
    """规格书 A.3：单程自由空间损耗（dB），距离用 km。"""
    return (
        10.0 * math.log10(FLUSH_K)
        + 20.0 * math.log10(dist_km)
        + 20.0 * math.log10(freq_hz)
    )


def list_stub(az_deg: float, peak_dbi: float, bw_deg: float) -> float:
    """规格书 A.1：方位角偏离 → 增益（dBi）。"""
    a = abs(az_deg)
    if a > 180.0:
        raise ValueError(f"azimuth out of range: {az_deg}")
    if a < 0.001:
        return peak_dbi
    x = 2.7831 * a / bw_deg
    return min(peak_dbi, peak_dbi + 20.0 * math.log10(abs(math.sin(x) / x)))


def title(text: str) -> None:
    print()
    print("=" * 78)
    print(text)
    print("=" * 78)


# ---------------------------------------------------------------------------
# 1. FlushStub 的常数对不对
# ---------------------------------------------------------------------------
title("1. FlushStub 的定义与常数")

FREQ = 9.4e9
DIST_KM = 100.0
L_stub = flush_stub(DIST_KM, FREQ)
R_M = DIST_KM * 1000.0
L_direct = 20.0 * math.log10(4.0 * math.pi * R_M * FREQ / C)

print(f"  FlushStub(100 km, 9.4 GHz)        = {L_stub:.4f} dB")
print(f"  20log10(4πRf/c) 直接算           = {L_direct:.4f} dB")
print(f"  偏差                              = {abs(L_stub - L_direct):.2e} dB")
print()
print(f"  常数 K = {FLUSH_K:.6e}，10log10(K) = {10 * math.log10(FLUSH_K):.4f} dB")
print(f"  理论 (4π·1000/c)² = {((4 * math.pi * 1000.0) / C) ** 2:.6e}")
print("  → 结论：FlushStub 确实是**单程**损耗，且距离单位是 km。定义正确。")


# ---------------------------------------------------------------------------
# 2. Step 4「版本二」与「版本一」是否等价
# ---------------------------------------------------------------------------
title("2. Step 4：版本一（标准式）vs 版本二（等价格式）")

Pt = 1.0e6
G_dBi = 33.0
G = lin(G_dBi)
RCS = 5.0
L_sys_dB = 4.0
L_sys = lin(L_sys_dB)
LAM = C / FREQ

# 版本一：Pr = Pt·G²·λ²·σ / ((4π)³·R⁴·L_sys²)
Pr_v1 = Pt * G * G * LAM * LAM * RCS / ((4.0 * math.pi) ** 3 * R_M ** 4 * L_sys ** 2)

# 版本二：Pr = Pt·G²·σ·λ² / (1984.4 · L_twoway_linear · L_sys²)
L_2way_dB = 2.0 * flush_stub(DIST_KM, FREQ)
L_2way = lin(L_2way_dB)
Pr_v2 = Pt * G * G * RCS * LAM * LAM / (1984.4 * L_2way * L_sys * L_sys)

print(f"  (4π)³ = {(4 * math.pi) ** 3:.4f}   ← 规格书写的 1984.4，一致")
print(f"  双程损耗 L_twoway = (4πR/λ)⁴ 线性值 = {L_2way:.6e}")
print()
print(f"  版本一 Pr = {Pr_v1:.6e} W")
print(f"  版本二 Pr = {Pr_v2:.6e} W")
print(f"  比值 Pr_v2/Pr_v1 = {Pr_v2 / Pr_v1:.6e}")
print()
ratio_theory = (LAM / (4.0 * math.pi)) ** 4
print(f"  (λ/4π)⁴ = {ratio_theory:.6e}  → {db(ratio_theory):+.2f} dB")
print("  → 结论：版本二把 (4π)³ 和已经含完整 (4π)⁴ 的 L_twoway **乘了两遍**。")
print(f"    低估 SNR {abs(db(ratio_theory)):.1f} dB，等效最大探测距离缩短 "
      f"{lin(abs(db(ratio_theory)) / 4):.0f} 倍。")
print()
# 要把 R⁴ 换成 L_twoway：R⁴ = L_twoway·λ⁴/(4π)⁴，代回标准式得
#   Pr = Pt·G²·σ·(4π) / (λ² · L_twoway · L_sys²)
# 也就是说分母里**不能再有 (4π)³**，而且要乘一个 4π、除一个 λ²。
Pr_fix = (4.0 * math.pi * Pt * G * G * RCS
          / (LAM * LAM * L_2way * L_sys * L_sys))
print("  修正：L_twoway 已含完整 (4πR/λ)⁴，代回标准式后分母不能再有 (4π)³：")
print("        Pr = 4π·Pt·G²·σ / ( λ² · L_twoway_linear · L_sys² )")
print(f"        算得 Pr = {Pr_fix:.6e} W，与版本一比值 = {Pr_fix / Pr_v1:.12f}")
print("        → 恒等于 1，两条式子自洽。但既然版本一本来就是对的，")
print("          最简做法是**删掉版本二**，只留版本一。")


# ---------------------------------------------------------------------------
# 3. 3f 欺骗干扰的单程方程
# ---------------------------------------------------------------------------
title("3. Step 3f：欺骗干扰路径的接收功率")

bw_hz = 1.0 / (1.0e-6)          # 脉宽 1 μs
L_1way = lin(flush_stub(DIST_KM, FREQ))

# 规格书写法
P_doc = (Pt * G * G * LAM * LAM / ((4.0 * math.pi) ** 2 * R_M ** 2 * L_1way))
# Friis 标准
P_ref = (Pt * G * G * LAM * LAM / ((4.0 * math.pi) ** 2 * R_M ** 2))

print(f"  规格书写法（除了一次 L_oneway）= {P_doc:.6e} W")
print(f"  Friis 标准（不除）            = {P_ref:.6e} W")
print(f"  比值 = {P_doc / P_ref:.6e} → {db(P_doc / P_ref):+.1f} dB")
print()
print(f"  理论多除的因子 (λ/4πR)² = {(LAM / (4 * math.pi * R_M)) ** 2:.6e}")
print()
print(f"  正确的干扰机 SNR = {db(P_ref / (K_B * 290.0 * bw_hz)):.1f} dB "
      f"（门限 13 dB → P_deception = 0.9）")
print(f"  规格书的干扰机 SNR = {db(P_doc / (K_B * 290.0 * bw_hz)):.1f} dB "
      f"（门限 13 dB → P_deception = 0.0）")
print("  → 结论：欺骗支路**永远返回 0**，RGPO/VGPO 完全不生效（功能缺陷，不只是精度问题）。")
print("    同时这里 `powerWatts` 指的是**雷达**功率还是**干扰机**功率，规格书没有写清。")


# ---------------------------------------------------------------------------
# 4. 波束形状损耗
# ---------------------------------------------------------------------------
title("4. Step 2：波束形状损耗 1.6 dB 的口径")


def integrate(func, a, b, n=200_000):
    """梯形积分。"""
    xs = np.linspace(a, b, n + 1)
    ys = np.array([func(float(x)) for x in xs])
    return float(np.trapezoid(ys, xs))


def one_way(u: float) -> float:
    """以波束宽度 BW 为单位的偏移 u 处的单程功率增益（峰值归一为 1）。"""
    x = 2.7831 * u
    if abs(x) < 1e-12:
        return 1.0
    return (math.sin(x) / x) ** 2


# 口径 A：在 ±BW/2（即 ±3 dB 宽度）内对**单程 dB 增益**取平均
mean_db = integrate(lambda u: db(one_way(u)), -0.5, 0.5) / 1.0
# 口径 B：在 ±BW/2 内对**单程线性增益**取平均
mean_lin = integrate(one_way, -0.5, 0.5)
print(f"  口径 A：±BW/2 内单程增益的 dB 均值   = {mean_db:+.3f} dB（相对峰值）")
print(f"  口径 B：±BW/2 内单程增益的线性均值   = {mean_lin:.4f} → {db(mean_lin):+.3f} dB")
print(f"  口径 C：把 A 的结果按双向折算（×2）   = {2 * mean_db:+.3f} dB")
print()
print("  Blake 的波束形状损耗 L_p 是**双向功率损耗**口径：")
print("    表值 1.6 dB（均匀照射 / sinc²）")
print("    按上面的口径 C 直接积分得 {:.2f} dB —— 同一量级，差异来自积分区间的约定".format(
    -2 * mean_db))
print()
print("  问题不在 1.6 这个数，而在**没标方向**。规格书写的是：")
print("        G_target = antennaGain_dBi - 1.6")
print("  而 G 在雷达方程里是平方进入的：")
print(f"    按「单向增益损耗」理解 → Pr 上实际扣 {2 * 1.6:.1f} dB")
print("    按「双向功率损耗」理解 → Pr 上只该扣 1.6 dB，即 G 上扣 0.8 dB")
print(f"    两种理解相差 1.6 dB → 探测距离相差 {lin(1.6 / 4) - 1:+.1%}")
print()
print("  → 结论：二选一，写明口径。若沿用 Blake 的 1.6 dB，应写成")
print("    Pr /= 10^(1.6/10)，而不是从 G_target 上扣。")
print("    另外 AFSIM 本身并不用这个经验系数——它按每个时刻的实际波束偏离角")
print("    查方向图。如果大步长实现里能拿到偏离角，用 ListStub 更准；")
print("    1.6 dB 只适合「几何冻结」这条近似路径。")


# ---------------------------------------------------------------------------
# 5. 波束宽度 → 峰值增益
# ---------------------------------------------------------------------------
title("5. 快查表 7.2：G_peak 由波束宽度推导")

print("  公式：G_peak_dBi = 10log10( 4π / (2·sin(HBW/2)·VBW) )")
print()
print(f"  {'HBW×VBW':<12}{'规格书表值':>10}{'按弧度假定':>12}{'按度数直算':>12}")
for hbw, vbw, table in ((1.0, 3.0, 40.0), (2.0, 10.0, 33.0), (5.0, 20.0, 27.0)):
    g_rad = db(4.0 * math.pi / (2.0 * math.sin(math.radians(hbw) / 2.0)
                                * math.radians(vbw)))
    g_deg = db(4.0 * math.pi / (2.0 * math.sin(hbw / 2.0) * vbw))
    print(f"  {hbw:>4.0f}°×{vbw:<5.0f}{table:>10.1f}{g_rad:>12.1f}{g_deg:>12.1f}")
print()
print("  → 结论：只有把两个角都换成**弧度**才对得上表值。")
print("    规格书写的是 2·sin(HBW/2)·VBW，没标单位，照字面实现会得到荒谬的负增益。")
print("    建议写成 4π/(θ_H·θ_V)，θ 为弧度；sin 修正只在波束很宽时才需要。")


# ---------------------------------------------------------------------------
# 6. ListStub 的方位角越界
# ---------------------------------------------------------------------------
title("6. A.1 ListStub：方位角超出 180° 会抛异常")

print("  规格书 3c：offset_jam = |jammer.beamAzimuth - jammerToRadarBearing|")
print("  这两个角各自都在 [0,360)，差值范围是 [0,360)，**不是** [0,180]。")
print()
for beam_az, to_radar in ((10.0, 350.0), (350.0, 10.0), (90.0, 270.0)):
    offset = abs(beam_az - to_radar)
    try:
        g = list_stub(offset, 20.0, 30.0)
        verdict = f"{g:.2f} dBi"
    except ValueError as exc:
        verdict = f"抛异常：{exc}"
    print(f"    beamAzimuth={beam_az:>6.1f}  目标方位={to_radar:>6.1f}  "
          f"offset={offset:>6.1f}  → {verdict}")
print()
print("  → 结论：只要干扰机天线大致背对雷达就会**崩溃**。")
print("    修正：offset = abs(((a - b) + 180) % 360 - 180)，先归一化到 [0,180]。")
print("    同一个问题在 offset_radar 上也存在。")


# ---------------------------------------------------------------------------
# 7. 退化输入
# ---------------------------------------------------------------------------
title("7. 退化输入：距离 0 / 脉宽 0")

print("  FlushStub 里有两处 log10 与一处除法，都没有保护：")
for d in (0.0, 1e-9):
    try:
        val = flush_stub(d, FREQ)
        print(f"    dist_km={d:<10} → {val}")
    except Exception as exc:
        print(f"    dist_km={d:<10} → Python 抛 {type(exc).__name__}；"
              f"C++ 的 std::log10(0) 返回 -inf（不抛）")

try:
    val = flush_stub(100.0, 0.0)
    print(f"    freq=0          → {val}")
except Exception as exc:
    print(f"    freq=0          → Python 抛 {type(exc).__name__}；C++ 返回 -inf")
print()
print("  **C++ 不抛异常**，这才是危险的地方：距离 0 → L = -inf → L_twoway = 0")
print("  → Pr = x/0 = inf（或 0，看公式写法）→ SNR = inf/-inf")
print("  → 再经 round(SNR_dB + 20) 转 int：**负无穷转 int 是未定义行为**，")
print("    实测常见的表现是得到一个 INT_MIN 级的负数，clamp 后查表得 idx=0，")
print("    于是「距离 0」这个明显该报错的输入，静默变成「Pd = 表头值」。")
print("  脉宽 0 则更早：bw_hz = 1/(0×1e-6) 直接除零。")
print("  → 结论：参数入口需要下限校验（距离下限、脉宽 > 0、频率 > 0），")
print("    或在 FlushStub 内改成 log10(max(x, eps))。不要在公式里留这条路。")


# ---------------------------------------------------------------------------
# 8. pdCurve 与步长的耦合
# ---------------------------------------------------------------------------
title("8. Step 7：pdCurve 只在模式初始化时生成 —— 但 N_pulses 依赖步长")

print("  规格书：scanRate=0（固定指向）时 pulsesPerDwell = stepDuration × prf")
print("         pdCurve 在模式初始化时生成一次，之后只按 SNR 查表")
print()
for prf in (1000.0,):
    print(f"  prf = {prf:.0f} Hz")
    for step in (1.0, 2.0, 10.0, 60.0):
        n = step * prf
        print(f"    stepDuration={step:>5.1f} s → N_pulses={n:>8.0f}  "
              f"积累增益 {db(n):>5.1f} dB")
    print()
print("  → 结论：固定指向模式下 N_pulses 随步长变化（1 s 与 60 s 差 "
      f"{db(60000) - db(1000):.1f} dB），")
print("    但表只生成一次 → 换了步长就用错了脉冲数。")
print("    修正：pdCurve 的缓存键必须包含 N_pulses，或干脆按 (mode, N) 分桶。")


# ---------------------------------------------------------------------------
# 9. N_encounters 的取整
# ---------------------------------------------------------------------------
title("9. 三、N_encounters：round 之后又 max(1, …)")

print("  N_scans = stepDuration × scanRate / 60")
print()
for step in (1.0, 2.0, 6.0):
    for rpm in (6.0, 15.0, 30.0):
        exact = step * rpm / 60.0
        doc = max(1, round(exact))
        print(f"    step={step:>4.1f}s  scanRate={rpm:>4.0f} rpm  "
              f"理论 {exact:>5.2f} 次 → 规格书取 {doc} 次"
              + ("   ← 放大 {:.0f} 倍".format(doc / exact) if exact > 0 and doc > exact else ""))
print()
print("  → 结论：只要一次步长内转不满一圈，理论值就 < 1，被 max(1,·) 抬到 1。")
print("    步长 1 s、15 rpm 的场景下探测机会被高估 4 倍。")
print("    修正：向下取整得 k 次，余下的小数部分按旋转初相做一次概率抽取，")
print("    或把旋转相位作为状态在步之间传递。")


# ---------------------------------------------------------------------------
# 10. Pd 表（第 7.4 节参考值）交叉验证
# ---------------------------------------------------------------------------
title("10. 快查表 7.4：Swerling 1 + Pfa=1e-6 的 Pd 参考值")

PFA = 1e-6
RNG = np.random.default_rng(20260918)


def pfa_threshold(n: int, pfa: float) -> float:
    """由虚警概率反解检测门限 V（噪声输出 ~ χ²(2N)）。

    注意 χ²(2N) 的尾概率是 ``e^{-V/2} Σ_{k<N} (V/2)^k / k!``——先按
    ``V/2`` 解出中间量再乘 2。漏掉这个 2 会让门限偏低 3 dB，
    N=1 时表现为 Pd 从 0.036 变成 0.19。
    """
    def tail(u: float) -> float:
        acc = 0.0
        term = 1.0
        for k in range(n):
            if k:
                term *= u / k
            acc += term
        return math.exp(-u) * acc

    # 先解出 u = V/2，再乘 2 得到 χ² 门限 V
    lo, hi = 0.0, 1.0
    while tail(hi) > pfa:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if tail(mid) > pfa:
            lo = mid
        else:
            hi = mid
    u = 0.5 * (lo + hi)
    return 2.0 * u


def pd_swerling1(snr_db: float, n: int, trials: int = 300_000) -> float:
    """蒙特卡洛：目标起伏服从指数分布且在整帧内恒定（Swerling 1）。"""
    v = pfa_threshold(n, PFA)
    s = RNG.exponential(lin(snr_db), size=trials)
    y = RNG.noncentral_chisquare(2 * n, 2 * n * s)
    return float(np.mean(y > v))


# 先用 N=1 的闭式解校验蒙特卡洛本身
print("  校验蒙特卡洛（N=1 有闭式解 Pd = Pfa^(1/(1+SNR))）：")
for snr_db in (5.0, 13.0):
    mc = pd_swerling1(snr_db, 1, trials=200_000)
    closed = PFA ** (1.0 / (1.0 + lin(snr_db)))
    print(f"    SNR={snr_db:>4.0f} dB   蒙特卡洛 {mc:.4f}   闭式 {closed:.4f}   "
          f"差 {abs(mc - closed):.4f}")
print()


# --- 独立的精确解：Poisson 混合表示，可替代 Kanter 近似 ---
def chi2_tail(m: int, v: float) -> float:
    """P(χ²(2m) > v)，即无信号时 m 个脉冲非相干积累超过门限的概率。"""
    u = v / 2.0
    acc = 0.0
    term = 1.0
    for k in range(m):
        if k:
            term *= u / k
        acc += term
    return math.exp(-u) * acc


def pd_swerling1_exact(snr_db: float, n: int, pfa: float) -> float:
    """Swerling 1、N 个脉冲非相干积累的**精确** Pd（级数，无近似）。

    非中心 χ² 用 Poisson 混合展开成中心 χ² 的加权和，再对 Swerling 1 的
    指数起伏做解析平均，得到

        Pd = e^{-u} Σ_j  [β^j / (1+NS)] · Σ_{k<n+j} u^k/k!,   u = V/2

    j 的衰减尺度是 (1+NS)，所以典型参数几百项就收敛。
    这个式子可以直接在初始化时生成 pdCurve，不必引入 Kanter 近似。

    **实现要点**：内层 χ² 尾和随 j 只增加一项，必须**增量**累加。
    每个 j 重算一遍的话复杂度是 O(j²)，实测跑几分钟都不出结果。
    """
    v = pfa_threshold(n, pfa)
    u = v / 2.0
    ns = n * lin(snr_db)
    beta = ns / (1.0 + ns)
    lead = math.exp(-u) / (1.0 + ns)

    # 内层和：acc = Σ_{k=0}^{m-1} u^k/k!，m 从 n 起，每步 +1
    acc = 0.0
    term = 1.0
    for k in range(n):
        if k:
            term *= u / k
        acc += term

    total = 0.0
    power = 1.0                      # β^j
    j = 0
    while True:
        total += lead * power * acc

        # 级数各项恒正，部分和单调上升；已经碰到 1 就不必再算
        if total >= 1.0 - 1e-12:
            return 1.0
        if power < 1e-15 and j > n:
            break

        j += 1
        term *= u / (n + j - 1)      # 增量补上 k = n+j-1 这一项
        acc += term
        power *= beta
        if j > 500_000:
            break

    return total


print("  自检：增量累加的内层 χ² 求和，必须与逐项直接计算一致")
_v = pfa_threshold(10, PFA)
_acc, _term = 0.0, 1.0
for _k in range(10):
    if _k:
        _term *= (_v / 2.0) / _k
    _acc += _term
print(f"    增量写法 {math.exp(-_v / 2.0) * _acc:.12f}   "
      f"直接写法 {chi2_tail(10, _v):.12f}")
print()

print("  独立对照：精确级数解 vs 蒙特卡洛")
print(f"    {'SNR':>5} {'N':>5} {'级数解':>9} {'蒙特卡洛':>9} {'差':>8}")
for snr_db, n in ((0.0, 1), (10.0, 1), (0.0, 10), (5.0, 10),
                  (10.0, 10), (5.0, 56), (10.0, 100)):
    exact = pd_swerling1_exact(snr_db, n, PFA)
    mc = pd_swerling1(snr_db, n, trials=300_000)
    print(f"    {snr_db:>5.0f} {n:>5} {exact:>9.4f} {mc:>9.4f} {abs(exact - mc):>8.4f}")
print("  → 两个独立方法互相吻合，可以认为 Swerling 1 的真实值就是下面这一列。")
print()

DOC_TABLE = {
    0.0: (0.001, 0.002, 0.01, 0.03),
    5.0: (0.04, 0.12, 0.35, 0.55),
    10.0: (0.29, 0.62, 0.87, 0.94),
    13.0: (0.52, 0.82, 0.95, 0.98),
    15.0: (0.66, 0.90, 0.98, 0.995),
    20.0: (0.87, 0.98, 1.0, 1.0),
}
PULSES = (1, 10, 56, 100)

print("  SNR(dB) |       N=1        |       N=10       |       N=56       |      N=100")
print("          |  表值    真值    |  表值    真值    |  表值    真值    |  表值    真值")
print("  " + "-" * 82)
worst = (0.0, "")
for snr_db in sorted(DOC_TABLE):
    cells = []
    for n, doc in zip(PULSES, DOC_TABLE[snr_db]):
        exact = pd_swerling1_exact(snr_db, n, PFA)
        flag = " " if abs(exact - doc) < 0.05 else "!"
        if abs(exact - doc) > worst[0]:
            worst = (abs(exact - doc), f"SNR={snr_db:.0f} dB, N={n}")
        cells.append(f"{doc:>5.3f} {exact:>6.3f}{flag}")
    print(f"  {snr_db:>6.0f}  | " + " | ".join(cells))
print()
print("  标 ! 的是与表值偏差 > 0.05 的格子。")
print(f"  最大偏差 {worst[0]:.3f}（{worst[1]}）。")
print("  → 结论：")
print("    · N=1 一列与 Swerling 1 单脉冲闭式解**完全吻合**，可信。")
print("    · N>1 各列在低 SNR 段系统性**偏低**：N=10、SNR=0 dB 处表值 0.002，")
print("      真值 0.120——差 60 倍。原因是低 SNR 下起伏目标偶发的强回波")
print("      贡献了大部分检测概率，而离散表把它平滑掉了。")
print("    · 高 SNR 段（≥13 dB）两者接近，说明表格是按某个高 SNR 锚点标定的。")
print("    · 建议：pdCurve 直接用上面的精确级数生成，不要用参考表；")
print("      参考表只能当量级校对，不能当验收基准。")
print("    · 另外 N_pulses=56 这种非整数来源没写清（dwell×prf 通常是小数），")
print("      规格书按 round 处理还是按插值处理，需要明确。")


# ---------------------------------------------------------------------------
# 11. 修正后的端到端算例
# ---------------------------------------------------------------------------
title("11. 用修正后的公式跑一条完整链路")

PRF = 1000.0
PULSE_US = 1.0
HBW = 2.0
SCAN_RPM = 15.0
STEP_S = 2.0
T_ANT = 290.0
T_RX = 500.0
bw = 1.0 / (PULSE_US * 1e-6)

dwell = HBW / (6.0 * SCAN_RPM)
n_pulses = max(1, round(dwell * PRF))
N_watts = K_B * (T_ANT + T_RX) * bw

print(f"  参数：Pt={Pt/1e6:.1f} MW，G={G_dBi:.0f} dBi，σ={RCS:.0f} m²，"
      f"f={FREQ/1e9:.1f} GHz，τ={PULSE_US:.0f} μs")
print(f"        dwell = HBW/(6·scanRate) = {dwell*1000:.2f} ms → N_pulses = {n_pulses}")
print(f"        噪声 N = kTB = {N_watts:.4e} W")
print()


def snr_at(dist_km: float, gain_dbi: float) -> float:
    r = dist_km * 1000.0
    g = lin(gain_dbi)
    pr = Pt * g * g * LAM * LAM * RCS / ((4 * math.pi) ** 3 * r ** 4 * L_sys ** 2)
    return db(pr / N_watts)


g_peak = G_dBi
g_scan = G_dBi - 1.6 * 0.5          # 双向 1.6 dB 折算到单向增益
print(f"  {'距离':>8}  {'SNR/脉冲':>10}  {'Pd(峰值增益)':>13}  {'Pd(扫描均值)':>13}")
for dist in (25.0, 50.0, 100.0, 150.0, 200.0):
    s1 = snr_at(dist, g_peak)
    s2 = snr_at(dist, g_scan)
    print(f"  {dist:>6.0f}km  {s1:>9.1f}dB  "
          f"{pd_swerling1_exact(s1, n_pulses, PFA):>13.3f}  "
          f"{pd_swerling1_exact(s2, n_pulses, PFA):>13.3f}")

# 反解 Pd = 0.5 的距离（用确定性级数解，不用抽样）
lo, hi = 1.0, 800.0
for _ in range(40):
    mid = 0.5 * (lo + hi)
    if pd_swerling1_exact(snr_at(mid, g_scan), n_pulses, PFA) > 0.5:
        lo = mid
    else:
        hi = mid
r50 = 0.5 * (lo + hi)
print()
print(f"  单次扫描 Pd=0.5 的探测距离 ≈ {r50:.0f} km")
print(f"  （{n_pulses} 脉冲积累，Swerling 1，Pfa=1e-6，含扫描损耗）")
print()
print("  → 这是**单次扫描**的严格口径。实际雷达用 M/N 建航逻辑，")
print("    等效作用距离会比这个数大（多次扫描累积）。")
print("    这个锚点的作用是验证公式链路自洽：如果实现出来偏离几十公里")
print("    以上，说明某处单位或系数错了，而不是「雷达性能就这样」。")

# ---------------------------------------------------------------------------
# 12. 剩余的一致性问题
# ---------------------------------------------------------------------------
title("12. 其余一致性问题（含几何冻结的量化代价）")

print("  (a) 步长内几何冻结的误差 —— 这是「大步长」这个前提本身的代价")
print()
print(f"  {'步长':>6} {'目标速度':>9} {'步内位移':>10} {'R=50km 时 R⁻⁴ 变化':>20}")
for step, speed in ((1.0, 300.0), (10.0, 300.0), (30.0, 300.0),
                    (60.0, 300.0), (60.0, 900.0)):
    travel = speed * step / 1000.0
    # 最坏情况：目标沿径向远离 / 靠近
    r0, r1 = 50.0, 50.0 + travel
    ratio = (r0 / r1) ** 4
    print(f"  {step:>4.0f} s {speed:>7.0f} m/s {travel:>8.1f} km "
          f"{db(ratio):>17.2f} dB")
print()
print("  → 结论：规格书 Step 1 写「步长内几何变化可忽略」，这个假设只在")
print("    短步长 + 慢目标时成立。60 s 步长下 R=50 km 处：")
print("    300 m/s 目标 → 5.3 dB；900 m/s 目标 → 12.7 dB。")
print("    这比「波束形状损耗 1.6 dB」的量级大得多，是真正需要处理的误差项。")
print("    建议：要么按实际步长动态限制（例如 60 s 步长只允许对慢目标用），")
print("    要么在步内做一次分段（把 60 s 拆成 2~3 段各自算几何）。")
print()

print("  (b) 脉冲干扰没有乘占空比")
print("    规格书 3b：jamType=1（SyncPulse）/ 2（ChaoticPulse）时 bwMatch = 1.0")
print("    但脉冲干扰只在占空比 d 的时间内有功率输出，平均功率应为 P×d。")
for duty in (1.0, 0.1, 0.01):
    print(f"      占空比 {duty:>5.1%} → 有效功率低 {abs(db(duty)):>5.1f} dB "
          f"（现在被完全忽略）")
print("    → 一个 1% 占空比的脉冲干扰机，压制效果被高估了 20 dB。")
print()

print("  (c) 主瓣干扰路径少了扫描损耗")
print("    目标回波用 G_target（旋转时已扣 1.6 dB），但干扰机方向用")
print("    ListStub(offset_radar, antennaGain_dBi, horzBeamWidth) —— 用的是峰值。")
print("    两者口径不一致，主瓣干扰会被系统性高估。")
print()

print("  (d) 干扰机路径损耗用了雷达频率")
print("    3d 写 FlushStub(dist_jammer_km, frequency)，这里 frequency 是雷达的。")
print("    干扰机在自身频率上发射，应使用 jammer.frequency。")
print("    同频段时影响很小（几 % 的频率差 → 零点几 dB），但要写明是有意为之。")
print()

print("  (e) 3f 里的 powerWatts 指谁？")
print("    该段物理含义是「干扰机收到雷达信号」，用的是雷达的发射功率；")
print("    但变量名 powerWatts 在 2.2 节是干扰机的功率。**同名字段两种含义**，")
print("    是最容易在实现时静默写错的一类问题。建议改名 radarPowerWatts。")
print()

print("  (f) 文档小问题")
print("    附录里有两个「A.3」（FlushStub 与 大步长入口），编号重复。")
print("    Step 7 说「初始化时生成一次」，但 A.2 BuildPdCurve 的参数里")
print("    没有区分模式的字段，多模式雷达需要说明何时重建。")
print()

print("=" * 78)
print("核对完毕。")
print("=" * 78)
