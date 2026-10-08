"""雷达探测链的**纯函数**：方程 → 噪声 → 信噪比 → 检测概率。

这一层刻意不依赖任何 milsim 模块（只 import ``math``），因为它是
**可以被独立核对**的那一半：单位、系数、级数的收敛性都能拿一张纸算出来，
而"雷达算得对不对"这件事不该和"网格怎么查、航迹怎么写"混在一起查。

对应设计文档 §5.12.7 的第 5~7 步：

.. code-block:: text

    第 5 步  回波功率      Pr = Pt·G²·λ²·σ / ((4π)³·R⁴·L)
    第 6 步  信噪比        SNR_pulse = Pr / (k·T_sys·B)
    第 7 步  检测概率      Pd = 非起伏(SNR_pulse, N, Pfa)  ← 默认（固定 RCS）
                           （写成 swerling_case 1 时改走 Swerling1(SNR_pulse, N, Pfa)）

★ **三个容易静默错的地方**，都在这里的函数签名上钉死：

1. **送进 Pd 的是「单脉冲」SNR，不是「积累后」的总 SNR。**
   Swerling 1 的闭式/级数里 ``N``（脉冲数）与 ``SNR`` 是**两个独立入口**
   （``q = N·S/(1 + N·S)`` 那种写法里 S 明确是单脉冲的）。把已经乘过 N 的
   总量再送进去，等于把积累增益算了两次 —— 症状是**作用距离虚高**，而航迹表
   看上去完全正常。实测锚点（N=22、Pfa=1e-6）：100 km 处单脉冲 1.483
   （1.7 dB）⇒ Pd 0.4033；若误送已乘过 N 的 32.6（15.1 dB）⇒ Pd 0.9580
   （差 0.55，不是"看着差不多"而是作用距离整体外推）。
   ⚠ AFSIM 的 ``signal_to_noise`` **正是**那个"积累后"的口径，直接拿来喂本
   函数就是把积累增益算两次——见 §5.12.8 的对照。
2. **``B = 1/τ`` 配 ``N = k·T·B`` 正好等于匹配滤波的 ``E/N₀``**，
   脉压增益没有漏掉，不要另乘一次。接收机若给了显式带宽，用它。
3. **门限解的是 ``χ²(2N)`` 的 V，不是 V/2。** 尾概率写成
   ``e^{-u}·Σ_{k<N} u^k/k!`` 时 ``u = V/2``；漏掉那个 2 会让门限偏低 3 dB。

所有 SNR 一律以**线性比值**传参、以 dB 查询，避免"这到底是 dB 还是线性"
这个最常见的接口错误。
"""

from __future__ import annotations

from functools import lru_cache
from math import cos, exp, lgamma, log, log10, pi, sqrt

__all__ = [
    "BOLTZMANN",
    "C_LIGHT",
    "FOUR_PI",
    "STANDARD_TEMPERATURE",
    "db",
    "linear",
    "peak_gain",
    "half_power_beamwidth",
    "wavelength",
    "received_power",
    "thermal_noise",
    "cascade_noise_temperature",
    "MONOPULSE_ANGLE_SLOPE",
    "angle_measurement_sigma",
    "range_gate",
    "range_measurement_sigma",
    "dwell_time",
    "pulses_in_dwell",
    "false_alarm_threshold",
    "swerling1_pd",
    "nonfluctuating_pd",
    "snr_for_pd",
    "integrated_snr",
    # -- 电子战（§5.14）----------------------------------------------------
    # ★ 这一节原先不在 ``__all__`` 里（是遗漏，不是有意）：清单一处不全会让
    # ``from ... import *`` 的使用方少拿到几个名字，而症状是"名字不存在"
    # 这类与公式无关的报错。补上，与模块实际导出的东西对齐。
    "POLARIZATION_KINDS",
    "polarization_effect",
    "bandwidth_overlap_ratio",
    "one_way_loss",
    "jamming_path_loss_db",
    "jamming_power",
    "jamming_to_signal",
    "snr_under_jamming",
    "antenna_pattern_factor",
]

#: 光速（m/s）。
C_LIGHT = 299792458.0

#: 玻尔兹曼常数（J/K）。用 CODATA 2018 的值——与规格书核对脚本一致。
BOLTZMANN = 1.380649e-23

#: 标准噪声温度（K）。接收机噪声系数按 ``T_sys = T0 · F`` 折算。
STANDARD_TEMPERATURE = 290.0

#: ``(4π)³``。雷达方程分母里的那个常数，写成具名量免得每次重算。
FOUR_PI = 4.0 * pi


def db(value: float) -> float:
    """线性比值 → dB。**小于等于 0 返回 −inf**，不抛异常。

    为什么不抛：雷达链上"信号弱到没有"是常态，不是错误。抛异常会让
    "目标太远"这条正常路径变成崩溃，而调用方本来就要处理它。
    """
    if value <= 0.0:
        return float("-inf")
    return 10.0 * log10(value)


def linear(decibels: float) -> float:
    """dB → 线性比值。``−inf`` 给 0，``+inf`` 给 ``inf``。"""
    if decibels == float("-inf"):
        return 0.0
    if decibels == float("inf"):
        return float("inf")
    return 10.0 ** (decibels / 10.0)


def wavelength(frequency_hz: float) -> float:
    """波长（米）。频率必须为正——0 会让 λ 发散，构造期就该拦住。"""
    if frequency_hz <= 0.0:
        raise ValueError(f"频率必须为正，收到 {frequency_hz}")
    return C_LIGHT / frequency_hz


def peak_gain(azimuth_beamwidth_deg: float, elevation_beamwidth_deg: float) -> float:
    """由波束宽度推**峰值增益**（线性）。

    ``G = 4π / (θ_H · θ_V)``，两个角都取**弧度**。

    规格书快查表 7.2 给出 1°×3° ⇒ 40 dBi、2°×10° ⇒ 33 dBi、5°×20° ⇒ 27 dBi。
    只有把两个角都换成弧度才对得上表值；照字面用度数会得到荒谬的负增益
    （已验证：1°×3° 按度数直算 = −15.6 dBi）。
    """
    if azimuth_beamwidth_deg <= 0.0 or elevation_beamwidth_deg <= 0.0:
        raise ValueError("波束宽度必须为正")
    theta_h = azimuth_beamwidth_deg * pi / 180.0
    theta_v = elevation_beamwidth_deg * pi / 180.0
    return FOUR_PI / (theta_h * theta_v)


def received_power(
    power_w: float,
    gain: float,
    wavelength_m: float,
    rcs_m2: float,
    range_m: float,
    loss: float = 1.0,
) -> float:
    """第 5 步：**版本一（标准式）**的回波功率。

    ``Pr = Pt · G² · λ² · σ / ((4π)³ · R⁴ · L)``

    ``gain`` 与 ``loss`` 都是**线性**值。``loss`` 是**双程总损耗**（发射内损 +
    接收内损 + 波束形状损耗……全在里面），因此式子分母里只有**一个** L ——
    写成 ``L²`` 会把两套口径混起来。

    规格书里的"版本二"（``Pr = Pt·G²·σ·λ² /(1984.4·L_twoway·L_sys²)``）
    把 ``(4π)³`` 和已经含完整 ``(4πR/λ)⁴`` 的 ``L_twoway`` **乘了两遍**，
    低估 SNR 约 60 dB。核对脚本 §2 已把这条钉死：版本一本来就对，
    版本二应当删除。这里只实现版本一。
    """
    if range_m <= 0.0:
        raise ValueError(f"距离必须为正，收到 {range_m}")
    if rcs_m2 <= 0.0:
        return 0.0
    return (
        power_w * gain * gain * wavelength_m * wavelength_m * rcs_m2
        / (FOUR_PI**3 * range_m**4 * loss)
    )


def thermal_noise(bandwidth_hz: float, temperature_k: float) -> float:
    """接收机噪声功率 ``N = k·T·B``（W）。

    ``T`` 是**系统噪声温度**（天线 + 接收机），不是室温。核对脚本里
    ``T_ant + T_Rx = 290 + 500 = 790 K``，反推时按 290 K 会差 2.7 倍 ——
    这个坑踩过一次，所以参数名写死是 ``temperature_k``。
    """
    if bandwidth_hz <= 0.0:
        raise ValueError(f"带宽必须为正，收到 {bandwidth_hz}")
    if temperature_k <= 0.0:
        raise ValueError(f"噪声温度必须为正，收到 {temperature_k}")
    return BOLTZMANN * temperature_k * bandwidth_hz


def cascade_noise_temperature(
    antenna_temperature_k: float,
    line_loss: float,
    noise_figure: float,
) -> float:
    """系统噪声温度（K）：Blake 级联式 ``T_s = T0·(L_r·F − 1) + T_a``。

    ``line_loss``（天线 → 接收机的馈线损耗）与 ``noise_figure`` 都是**线性**值。

    ★ 这个式子是 AFSIM 那个三步展开的**等价化简**（``docs/receiver.rst`` 的
    *Receiver Noise*，引 Blake 1986 *Radar Range Performance* 第 4 章）：

    .. code-block:: text

        T_a = T0 + (0.876·Tant − 254)/L_ohmic    天线（天空 + 欧姆两项）
        T_l = T0·(L_r − 1)                       馈线
        T_r = T0·(F − 1)                         接收机
        T_s = T_a + T_l + L_r·T_r

    展开：``T_l + L_r·T_r = T0·L_r − T0 + L_r·T0·F − L_r·T0 = T0·(L_r·F − 1)``
    —— 那一对 ``±T0·L_r`` **恰好抵消**，所以式子里只剩**一个** ``L_r``。
    我们这一步只做化简，不改语义：``T_a`` 由调用方整项给出（AFSIM 拆成
    ``Tant`` 与 ``antenna_ohmic_loss`` 两个参数，后者默认 0 dB ⇒ 两者等价）。

    ★ **默认值让"旧口径"成为它的一个特例**，不是另立一套：
    ``L_r = 1``（0 dB）且 ``T_a = T0`` ⇒ ``T_s = T0·F``，正是 AFSIM 文档里
    "给了 ``noise_figure``、而 ``antenna_ohmic_loss`` 与 ``receive_line_loss``
    **都没给**"的那条支路。两条支路因此共用同一个式子。
    """
    if antenna_temperature_k <= 0.0:
        raise ValueError(f"天线噪声温度必须为正，收到 {antenna_temperature_k}")
    if line_loss < 1.0:
        raise ValueError(f"馈线损耗（线性）不得小于 1（无损），收到 {line_loss}")
    if noise_figure < 1.0:
        raise ValueError(f"噪声系数（线性）不得小于 1（理想接收机），收到 {noise_figure}")
    return STANDARD_TEMPERATURE * (line_loss * noise_figure - 1.0) + antenna_temperature_k


#: 单脉冲测角的归一化斜率 ``k_m``。教科书（Barton, *Radar Equations for
#: Modern Radar*）给的典型区间是 1.5~1.8，取中值 1.6。
#:
#: ★ **两条独立证据都落在 1.6 附近**，所以它不是一个"调出来的数"：
#: ① 教科书区间；② ``SNR = 13 dB`` 时本式给出 ``σ_θ = θ_3dB / 10.05``，
#: 而 AFSIM 自带的 ``demos/air_to_air/sensors/radar/aesa.txt`` 里写的是
#: ``azimuth_error_sigma 0.3 deg // beamwidth / 10``——那条注释正是这条
#: 经验关系（而 13 dB 恰好是它的 ``detection_probability`` 表里 Pd≈0.95
#: 的那一档）。**仍未与 AFSIM 实跑标定**，见设计文档 §9。
MONOPULSE_ANGLE_SLOPE = 1.6


def angle_measurement_sigma(
    beamwidth_deg: float, snr: float, slope: float = MONOPULSE_ANGLE_SLOPE
) -> float:
    """角度量测误差的**标准差**（度）：``σ_θ = θ_3dB / (k_m·√(2·SNR))``。

    ``beamwidth_deg`` 是该维的**半功率波束宽度**，``snr`` 是**单脉冲**信噪比
    （线性）——与 :func:`swerling1_pd` 同一个入口。★ 送"积累后"的总 SNR
    进来等于把积累增益算两次，这正是本模块头第 1 条钉死的那个坑的同一家族。

    **它就是"Pd 越大越准"的物理来路**：不是 Pd 决定精度，而是 **Pd 与 σ_θ
    都由 SNR 决定**——两者是同源的兄弟，不是父子（见设计文档 §5.12.9）。
    所以"人为把 ``required_pd`` 调高"只动 Pd、动不了 σ：那才是判据。

    ``snr <= 0`` 返回 ``+inf``：没有回波就没有角度可言，真雷达此时根本不会
    起批。调用方应当先过 Pd 那一关，本函数不替它兜底。
    """
    if beamwidth_deg <= 0.0:
        raise ValueError(f"波束宽度必须为正，收到 {beamwidth_deg}")
    if slope <= 0.0:
        raise ValueError(f"测角斜率必须为正，收到 {slope}")
    if snr <= 0.0:
        return float("inf")
    return beamwidth_deg / (slope * sqrt(2.0 * snr))


def range_gate(time_s: float) -> float:
    """一个时间间隔在距离上对应多长：``c·t/2``（米）。

    同一个式子在探测链上有**两个**用途，而它们只能有一个出处：

    * ``t = τ``（脉宽）⇒ **距离分辨单元**——两个回波至少差这么远才分得开
      （所以假目标的间隔不能比它更密）；
    * ``t = PRI``（脉冲重复间隔）⇒ **最大不模糊距离**——再远的回波会折叠回
      下一个 PRI 的距离里，雷达分不清（所以假目标铺不了更深）。

    两处都走本函数：手写两遍 ``C_LIGHT * t / 2`` 的话，"距离单元"与"不模糊
    距离"迟早会有两个不同的系数，而症状是假目标的间距或条数看着合理但全错。
    """
    return C_LIGHT * time_s / 2.0


def range_measurement_sigma(pulse_width_s: float, snr: float) -> float:
    """距离量测误差的**标准差**（米）：``σ_R = c·τ / (2·√(2·SNR))``。

    ``c·τ/2`` 是未压缩脉冲的**距离分辨单元**（走 :func:`range_gate`），
    ``√(2·SNR)`` 是匹配滤波带来的精度改善——所以这里同样送**单脉冲** SNR。

    ★ 口径与 AFSIM 对得上：它的 ``error_model_parameters`` 里给的正是
    ``pulse_width`` / ``receiver_bandwidth`` 这几个量，即"**由波形参数**推
    距离误差"，而不是由 Pd 推。
    """
    if pulse_width_s <= 0.0:
        raise ValueError(f"脉冲宽度必须为正，收到 {pulse_width_s}")
    if snr <= 0.0:
        return float("inf")
    return range_gate(pulse_width_s) / sqrt(2.0 * snr)


def dwell_time(beamwidth_deg: float, scan_rate_deg_per_s: float) -> float:
    """目标被波束照住的时间（秒）``= 波束宽 ÷ 角速度``。

    ★ **这个量与引擎步长无关。** 旋转扫描下，目标在波束里待多久只取决于
    "波束多宽 ÷ 天线转多快"。

    **固定指向是唯一的例外**：天线不转，目标一直在波束里，驻留时间就等于
    **评估间隔**（``beam_step``），因此 ``dwell_time`` 在那里由调用方直接
    传步长进来，不经过本函数——规格书 P1-4 那个坑（pdCurve 缓存键不含
    ``N_pulses``）说的正是这件事。
    """
    if scan_rate_deg_per_s <= 0.0:
        raise ValueError("扫描角速度必须为正")
    return beamwidth_deg / scan_rate_deg_per_s


def pulses_in_dwell(dwell_s: float, prf_hz: float) -> int:
    """一个驻留期里打了多少脉冲。**至少 1**。

    ``N`` 直接进检测概率公式（积累增益 ≈ N 倍）。``max(1, ...)`` 不是
    修饰：``dwell × prf < 1`` 意味着连一个完整脉冲都不一定落在窗口里，
    但那仍然是**一次检测机会**，不是零次。
    """
    if prf_hz <= 0.0:
        return 1
    return max(1, int(round(dwell_s * prf_hz)))


@lru_cache(maxsize=256)
def false_alarm_threshold(pulses: int, probability_of_false_alarm: float) -> float:
    """由虚警概率反解**平方律检波**的检测门限 ``V``（噪声输出 ~ χ²(2N)）。

    尾概率 ``P(χ²(2N) > V) = e^{-u}·Σ_{k<N} u^k/k!``，其中 ``u = V/2``。

    ★ 实现上先按 ``u`` 解、最后乘 2 —— 直接把 ``V`` 代进那个和式会少一个
    2 倍因子，门限偏低 3 dB，N=1 时 Pd 从 0.036 变成 0.19（规格书核对 §10）。

    结果按 ``(N, Pfa)`` 缓存：一个雷达在整个推演里用同一组，而求解本身是
    200 次二分 × O(N) 的求和，不缓存会在每个目标每一拍上重复付费。
    """
    if pulses < 1:
        raise ValueError(f"脉冲数必须 >= 1，收到 {pulses}")
    if not 0.0 < probability_of_false_alarm < 1.0:
        raise ValueError(f"虚警概率必须在 (0,1)，收到 {probability_of_false_alarm}")

    def tail(u: float) -> float:
        total = 0.0
        term = 1.0
        for k in range(pulses):
            if k:
                term *= u / k
            total += term
        return exp(-u) * total

    low, high = 0.0, 1.0
    while tail(high) > probability_of_false_alarm:
        high *= 2.0
        if high > 1.0e6:                     # Pfa 小到不合理时的兜底
            break
    for _ in range(200):
        mid = 0.5 * (low + high)
        if tail(mid) > probability_of_false_alarm:
            low = mid
        else:
            high = mid
    return 2.0 * 0.5 * (low + high)


def swerling1_pd(
    snr_pulse: float, pulses: int, probability_of_false_alarm: float
) -> float:
    """Swerling 1（慢起伏）在 N 个脉冲**非相干积累**下的检测概率。

    ``snr_pulse`` 是**单脉冲**信噪比（线性）——见模块头第 1 条。

    精确解（不是 Kanter 近似）。非中心 χ² 用 Poisson 混合展开成中心 χ² 的
    加权和，再对 Swerling 1 的指数起伏做解析平均；**外层几何和可以解析收缩掉**：

    .. code-block:: text

        原始式  Pd = e^{-u} · Σ_j (1−β)·β^j · Σ_{k < N+j} u^k / k!
        收缩式  Pd = e^{-u} · Σ_{k < N} u^k/k!  +  e^{-u} · β · Σ_m u^{N+m}·β^m / (N+m)!
        记号    u = V/2,  β = N·S / (1 + N·S),  m = 0, 1, 2, …

    ★ 收缩的来历：把 ``Σ_{k<N+j}`` 拆成 ``Σ_{k<N} + Σ_{N≤k<N+j}``，外层对 ``j``
    求和时 ``Σ_{j ≥ k−N+1} (1−β)β^j = β^{k−N+1}``，几何和消失，只剩一个级数；
    代入 ``k = N+m`` 即得第二项。**第一项恰好就是 Pfa。**

    ★ **为什么必须收缩（v0.13.25 修）**：原始式里 ``j`` 的衰减尺度是
    ``1/(1−β) = 1 + N·S``，近程强信号下 ``N·S`` 上千（AFSIM 对照里 4 km 处
    ``N·S ≈ 4×10⁶``）要上千万项才收敛。上一版有个 ``j > 200_000`` 的兜底截断
    ⇒ **静默返回 ≈ 200000/(1+N·S)**：4 km 处给出 Pd = 0.0486，而真值是 1.0；
    因为数看着"像个概率"、航迹表也照常写，这个错一直没露头。
    收缩后 ``m`` 级数的收敛尺度只由 ``β·u`` 与 ``N`` 决定（≤ 数百项），
    **与 ``N·S`` 无关**。

    ★ 两项都恒正、无相减，因此不存在灾难性抵消。

    已与三路独立方法对齐（规格书核对 §10）：N=1 的闭式 ``Pfa^(1/(1+S))``、
    Swerling 1 "总 SNR 随 N 上升"的教科书表、以及 30 万次蒙特卡洛。
    """
    if pulses < 1:
        raise ValueError(f"脉冲数必须 >= 1，收到 {pulses}")
    if not 0.0 < probability_of_false_alarm < 1.0:
        raise ValueError(f"虚警概率必须在 (0,1)，收到 {probability_of_false_alarm}")
    if snr_pulse <= 0.0:
        return probability_of_false_alarm

    u = false_alarm_threshold(pulses, probability_of_false_alarm) / 2.0
    ns = pulses * snr_pulse
    beta = ns / (1.0 + ns)

    # 第一项 = Pfa = e^{-u}·Σ_{k<N} u^k/k!（递推，不出现大数幂与阶乘）
    term = exp(-u)
    first = term
    for k in range(1, pulses):
        term *= u / k
        first += term

    # 第二项 = e^{-u}·β·Σ_m u^{N+m}·β^m/(N+m)!，从 m=0 起步递推
    term = exp(-u)
    for k in range(1, pulses + 1):
        term *= u / k                        # e^{-u}·u^N/N!（= Poisson(109.45) 量级，不会溢出）
    term *= beta
    second = term
    # ★ u > N 时前若干项是**上升**的（峰值在 m ≈ β·u − N），不能"见小就停"
    stop_at = int(beta * u) + pulses + 512
    for m in range(1, stop_at):
        term *= beta * u / (pulses + m)
        second += term
        if pulses + m > beta * u and term <= second * 1e-17:
            break
    return min(1.0, first + second)


def nonfluctuating_pd(
    snr_pulse: float, pulses: int, probability_of_false_alarm: float
) -> float:
    """**固定 RCS**（非起伏目标，Swerling 0 / Swerling 5）的检测概率。

    ``snr_pulse`` 是**单脉冲**信噪比（线性），与 :func:`swerling1_pd` 同一个入口。

    ★ **为什么要有它（v0.13.40 改口径）**：本框架的 RCS 是**常数标量**
    （平台属性 ``radar_cross_section``，见 ``percep.base._rcs_of``），雷达方程
    里那个 ``σ/σ_ref`` 也是确定的。而 Swerling 1 的 Pd 公式是对"RCS 逐扫描
    服从**指数分布**"做了**解析平均**后的结果——拿它去算一个**确定**的 RCS，
    是"起伏模型的公式配非起伏的输入"，两边语义对不上。这条改动就是把默认
    口径换成与输入一致的那个。

    ★ **非起伏的统计模型**：回波幅度确定 ⇒ 平方律检波后，`N` 个脉冲的非相干
    积累服从**非中心 χ²**，自由度 `2N`、非中心参量 ``λ = 2·N·S``
    （`S` = 单脉冲 SNR，线性）。判据仍是"超过门限 `V`"，而 `V` 由
    :func:`false_alarm_threshold` 按 ``P(χ²(2N) > V) = Pfa`` 定——**门限的定义
    一个字节都没变**，变的只是"统计量服从哪个分布"。

    上尾写成 Marcum Q：``Pd = Q_N(√(2·N·S), √V)``。

    ★ **与 Swerling 1 的差不是单向的，两条曲线会交叉**（`Pfa=1e-6`、`N=1`，
    实测）：

    | 单脉冲 SNR | 非起伏 Pd | Swerling 1 Pd | 谁高 |
    |---|---|---|---|
    | 0 dB | 0.000122 | 0.001000 | **Swerling 1** |
    | 6 dB | 0.010564 | 0.062437 | **Swerling 1** |
    | 10 dB | 0.248049 | 0.284804 | **Swerling 1** |
    | 12.7721 dB | 0.837747 | 0.500018 | **非起伏** |
    | 15 dB | 0.997225 | 0.654756 | **非起伏** |
    | 20 dB | 1.000000 | 0.872156 | **非起伏** |

    交叉点约在 **11.5 dB**（`Pd ≈ 0.6` 附近）。原因不是"哪个更保守"，是
    **起伏对"Pd 是 SNR 的凸函数还是凹函数"这两段的权重不同**：

    * **低 SNR 段**：`Pd` 对 SNR 是**凸**的（强回波那一支上升极快）⇒ 对指数分布
      求平均（Swerling 1）反而**抬高** Pd，"偶发强回波"贡献了大部分检测概率。
    * **高 SNR 段**：`Pd` 已饱和、对 SNR 变**凹**⇒ 平均把"偶发弱回波"的漏检
      摊进来，Swerling 1 反而**压低** Pd。

    ⇒ 所以本框架"从 Swerling 1 口径换成固定 RCS 口径"在**工作点**
    （`Pd = 0.5`，落在高 SNR 侧）的方向是**放宽**——`Pd = 0.5` 需要的单脉冲
    SNR 从 **12.7719 dB 降到 11.2426 dB**（差 **1.5293 dB**，作用距离按 `R⁴`
    放大约 **+1.088** 倍）。但在极低 SNR 侧它会让 Pd **变小**——这两件事都必须
    记在文档里，不能只说"变宽松"。

    已与 30 万次蒙特卡洛对齐（N=1：0 dB→0.00012、1 dB→0.00023、
    10 dB→0.24811、12.7721 dB→0.83822）。
    """
    if pulses < 1:
        raise ValueError(f"脉冲数必须 >= 1，收到 {pulses}")
    if not 0.0 < probability_of_false_alarm < 1.0:
        raise ValueError(f"虚警概率必须在 (0,1)，收到 {probability_of_false_alarm}")
    if snr_pulse <= 0.0:
        return probability_of_false_alarm

    # V = 门限能量（平方律），与 Swerling 1 用的是**同一个** V
    v = false_alarm_threshold(pulses, probability_of_false_alarm)
    return _marcum_q(pulses, sqrt(2.0 * pulses * snr_pulse), sqrt(v))


def _marcum_q(order: int, a: float, b: float) -> float:
    """广义 Marcum Q：``Q_m(a,b) = ∫_b^∞ x·(x/a)^{m−1}·exp(−(x²+a²)/2)·I_{m−1}(a·x) dx``。

    物理上这正是**非中心 χ² 的上尾**（本项目用它算非起伏目标的 `Pd`）：

    .. code-block:: text

        Q_m(a,b) = P( χ'²_{2m}(λ = a²) > b² )

    实现在**混合表示**（Poisson 权重 × 中心 χ² 上尾）上做——这是唯一在整个
    量程（从 `a ≪ b` 到 `a ≫ b`）都数值稳定的形式：

    .. code-block:: text

        P(χ'²_{2m}(λ) > V)
            = Σ_{j≥0} e^{−λ/2}·(λ/2)^j/j! · P(χ²_{2(m+j)} > V)

    为什么不直接用"Marcum 级数"``e^{−(a²+b²)/2}·Σ_k (a/b)^k·I_k(ab)``（踩过
    的坑，留档）：它在 `a ≪ b`（弱信号）收敛很快，但**在高 SNR（`a ≫ b`，
    恰好是本项目的工作区）收敛极慢且数值崩坏**——`(a/b)^k` 指数发散、`I_k(ab)`
    在 `k ≳ ab` 后正向递推失真。实测 `SNR=30 dB`、`N=1`（`a=44.7, b=5.26`）
    时该式给 `e^{−438}`（应为 1）。

    ★★ 本实现的关键（第二版，第一版踩过两个坑，都留档）：

    1. **不能"截窗求和"**。第一版取 `j ∈ [λ/2 ± (10√(λ/2)+64)]` 求和，理由
       是"窗外 Poisson 权重 < 1e-20"。这对 `a ≪ b` 成立，但对**大 SNR**
       致命：那时 χ² 上尾 `P(χ²_{2(m+j)}>V)` 在窗口内几乎处处 = 1，于是
       `Σ = Σ_{j∈窗} w_j = 1 − (窗外质量)`，把本该是 1 的结果**从下面**砍掉
       一个窗外的质量（实测 20 dB、N=1 给 `0.999999`，真值 `1−3.8e-14`），
       并让 `Pd(R)` **非单调**（`Pd(1 km) < Pd(5 km)`）。**缺的那一截恰好
       就是答案里 1e-6 级别的修正量，不能丢。**

    2. **正确写法（本版）= 缺额分解，从 j=0 起步，末尾用 Poisson 余量补齐**：

       .. code-block:: text

           Pd = Σ_{j ≥ 0} w_j·t_j
              = Σ_{j < j_sat} w_j·t_j  +  Σ_{j ≥ j_sat} w_j      (那里 t_j ≡ 1)
              = Σ_{j < j_sat} w_j·t_j  +  (1 − Σ_{j < j_sat} w_j)
              = 1 − Σ_{j < j_sat} w_j·(1 − t_j)

       其中 `t_j = P(χ²_{2(m+j)}>V)`，`j_sat` 是 `t_j` **数值饱和到 1** 的
       第一处（`1 − t_j < _PD_MAX_SATURATION`）。饱和点只由 `V` 决定：
       `n_sat ≈ u + 9√u`（`u = V/2`），与 `λ` **无关**。

    ★★ 这个写法同时解决三件事：
       * **精度**：`1 − t_j` 只在 `j < j_sat`（`t_j` 很小、可以精确表示）时
         参与运算，不出现 `1 − 接近 1` 的抵消；`j ≥ j_sat` 整段用 Poisson
         余量的**闭式**补上，一分不少。
       * **速度**：迭代次数 ≈ `j_sat ≈ u + 9√u`（`Pfa=1e-6` 时 ≈ 52 次），
         **与 `λ` 无关**——第一版 O(λ) 的窗口（大 SNR 时几百万项）和
         O(j_lo²) 的重复求和都不会再出现。
       * **单调**：大 SNR 时 `w_j`（`j` 很小时）下溢为 0 ⇒ `Pd = 1 − 0 = 1`，
         严格单调不下降。

    ★ 保守性：`a` 或 `b` 为 0 的退化情形单独处理，不让它进级数。
    """
    if b <= 0.0:
        return 1.0
    if a <= 0.0:
        # Q_m(0,b) = P(χ²_{2m} > b²)（λ=0 的非中心分布就是中心分布）
        return _chi2_upper_tail_normalized(order, b * b / 2.0)

    v = b * b            # 门限 V
    u = 0.5 * v
    half_lam = 0.5 * a * a   # λ/2 = N·S（Poisson 分布的均值）

    # ---- χ² 上尾 t_j = P(χ²_{2(m+j)} > V) = e^{−u}·Σ_{i<m+j} u^i/i! ----
    # 从 n = order 起步，用 P(χ²_{2n+2}>V) = P(χ²_{2n}>V) + e^{−u}u^n/n! 递推。
    #
    # ★ `n_sat`：χ² 上尾**数值饱和**（`1 − t < 1e-16`）的自由度半指数。递推
    #   增量 `e^{−u}u^n/n!` 是**单峰**的（峰在 `n ≈ u`），峰后迅速衰减；取
    #   `u + 12√u + 64` 作解析上界（`P(χ²_{2n}>V) ≥ 1 − 1e-16` 的 `n` 远小于
    #   此）。★ 不能用"逐项看 `1 − tail < eps`"来判断——增量会先**下溢到 0**
    #   （`exp(log_chi_term)` 为 0），`tail` 就永远停在 `1 − 1e-17` 附近**到不了
    #   `1 − 1e-16`**，判据永不触发，循环会跑满 `_PD_MAX_TERMS` 白跑 2e6 次
    #   （实测每个调用 0.6 s、结果错成 1.0）。用解析上界一步到位。
    n_sat = order + max(64, int(u + 12.0 * sqrt(u) + 64.0))
    n = order
    tail = 1.0               # Σ_{i<order} u^i/i!（**不含** e^{−u} 因子）
    term = 1.0
    for i in range(1, order):
        term *= u / i
        tail += term
    tail *= exp(-u)          # ★ e^{−u} 只在这里乘**一次**（踩过：初值也写成
                             #   `exp(-u)` 又乘一次 ⇒ 多一个 e^{−u} 因子，
                             #   `tail` 永远停在 `1 − e^{−u}`，20 dB 时结果
                             #   偏低整整 1e-6）
    if tail > 1.0:
        tail = 1.0

    # e^{−u}·u^n/n!（自由度递推项；u = 0 时该式为 0）
    log_chi_term = (-u + n * log(u) - _log_factorial(n)) if u > 0.0 else float("-inf")

    # ---- Poisson 权重（对数域递推，避免 e^{−λ/2} 下溢） ----
    log_half_lam = log(half_lam) if half_lam > 0.0 else float("-inf")
    log_w = -half_lam        # j = 0：w_0 = e^{−λ/2}

    # ---- Pd = Σ_{j<j_sat} w_j·t_j + (1 − Σ_{j<j_sat} w_j) ----
    # `j_sat` = 使 `order + j ≥ n_sat` 的最小 j（其后的 `t_j` 一律按 1 计，
    # 那段 Poisson 质量用 `1 − Σ_{j<j_sat} w_j` 一次补齐）。
    j_sat = max(0, n_sat - order)
    mass_below = 0.0         # Σ_{j<j_sat} w_j
    partial = 0.0            # Σ_{j<j_sat} w_j·t_j
    for j in range(0, j_sat):
        wj = exp(log_w) if log_w > -745.0 else 0.0
        mass_below += wj
        partial += wj * tail
        # 推进 j → j+1
        log_w = log_w + log_half_lam - log(j + 1)
        if log_chi_term != float("-inf"):
            tail = tail + exp(log_chi_term)
            if tail > 1.0:
                tail = 1.0
        n += 1
        log_chi_term = (-u + n * log(u) - _log_factorial(n)) if u > 0.0 else float("-inf")

    pd = partial + (1.0 - mass_below)
    if pd <= 0.0:
        return 0.0
    if pd >= 1.0:
        return 1.0
    return pd


def _log_factorial(n: int) -> float:
    """``ln(n!)``（Lanczos 的 `lgamma` 由 `math` 提供，直接转调以统一接口）。"""
    return lgamma(n + 1.0)


def _chi2_upper_tail_normalized(k_half: int, u: float) -> float:
    """中心 χ²(2·k_half) 的上尾 ``P(χ²_{2n} > 2u) = e^{−u}·Σ_{k<n} u^k/k!``。

    ★ 只在 `λ = 0` 的退化分支用到（`a ≤ 0`）；正常路径由 `_marcum_q` 内部
    的递推形式覆盖。
    """
    total = 1.0
    term = 1.0
    for k in range(1, k_half):
        term *= u / k
        total += term
    val = exp(-u) * total
    return val if val < 1.0 else 1.0


def snr_for_pd(
    pd: float,
    pulses: int,
    probability_of_false_alarm: float,
    *,
    nonfluctuating: bool = True,
    tolerance: float = 1e-9,
) -> float:
    """``pd`` 的**反解**：要得到这个 ``pd``，单脉冲 SNR 得是多少。

    这是"标定法"（``§5.12`` 的 ``detect_range`` 语义）用的：一个只给了
    *探测距离*、没给发射机参数的真实型号，我们就认定

        「``detect_range`` 处、对 ``reference_rcs`` 的目标，单次扫描的
        Pd 恰好等于 ``required_pd``」

    ``Pd(SNR)`` 单调上升，所以二分即可，不必求导。

    ★ ``nonfluctuating``（**v0.13.40 起默认 True**）选反解哪条曲线：

    * ``True``  → :func:`nonfluctuating_pd`（固定 RCS，与常数 RCS 输入一致）；
    * ``False`` → :func:`swerling1_pd`（起伏，旧口径，显式要才用）。

    换口径会移动 ``snr_at_reference``：``Pd=0.5``、`N=1`、``Pfa=1e-6`` 下
    非起伏要 **11.2426 dB**、Swerling 1 要 **12.7719 dB**（差 **1.5293 dB**）。
    标定法自己"反解 ⇒ 正算"仍逐字自洽（两处用同一条曲线），变的是那个
    参考 SNR 的绝对值。
    """
    if not 0.0 < pd < 1.0:
        raise ValueError(f"要求的 Pd 必须在 (0,1)，收到 {pd}")
    curve = nonfluctuating_pd if nonfluctuating else swerling1_pd
    low, high = 1e-12, 1.0
    while curve(high, pulses, probability_of_false_alarm) < pd:
        high *= 2.0
        if high > 1e12:
            break
    for _ in range(200):
        mid = 0.5 * (low + high)
        if curve(mid, pulses, probability_of_false_alarm) < pd:
            low = mid
        else:
            high = mid
        if high - low < tolerance * max(1.0, high):
            break
    return 0.5 * (low + high)


def integrated_snr(snr_pulse: float, pulses: int) -> float:
    """积累后的**总**信噪比（线性）。

    ★ **它不进 Pd 公式**（那要单脉冲值），它的用处有两个：

    * 报给人看的诊断量（"这目标多强"）；
    * 一条反直觉的判据 —— **Swerling 1 的总 SNR 随 N 上升**（慢起伏目标
      几乎吃不到积累增益，所以要更高的总能量才达到同样的 Pd：Pd=0.5 时
      N=1 要 12.77 dB、N=100 要 18.96 dB）。**如果实现出来发现它随 N 下降，
      说明算成了 Swerling 2**（快起伏，能吃到 ``√N``）。
    """
    return max(0.0, snr_pulse) * max(1, pulses)


def apparent_range(scale: float, range_m: float) -> float:
    """把"距离因子的四次方"写成一处，免得每个调用点各推一遍。

    ``R⁻⁴`` 是雷达方程里唯一与几何有关的项，出现频率极高。写成函数是为了
    **同一个量只有一个出处**（弹道倾角那次两边各推一次，重力被抵了两次，
    弹朝反方向飞）。
    """
    if range_m <= 0.0:
        raise ValueError(f"距离必须为正，收到 {range_m}")
    return scale / (range_m**4)


def range_for_scale(scale: float, value: float) -> float:
    """``apparent_range`` 的反解：给定距离因子求出距离。"""
    if value <= 0.0:
        raise ValueError("距离因子必须为正")
    return (scale / value) ** 0.25


def half_power_beamwidth(gain: float) -> float:
    """峰值增益 → 等效的``θ_H·θ_V`` 乘积（弧度²）``= 4π/G``。

    给"只给了增益、没给波束宽度"的型号反推一个等效张角用。注意它给的
    是**乘积**，单个角还需要一个假定（见 ``percep.base`` 的
    ``elevation_beamwidth`` 默认值）。
    """
    if gain <= 0.0:
        raise ValueError("增益必须为正")
    return FOUR_PI / gain


def free_space_loss(range_m: float, frequency_hz: float) -> float:
    """单程自由空间损耗（线性）``= (4πR/λ)²``。

    不参与 Pr 的计算（标准式里 λ 与 R 已经显式在式子上了），但它是对照
    AFSIM 报告里 ``signal_to_noise`` 时的常用中间量，也是规格书
    ``FlushStub`` 那个常数的定义。放这里是为了"两处各推一次"不再发生。
    """
    if range_m <= 0.0 or frequency_hz <= 0.0:
        raise ValueError("距离与频率都必须为正")
    return (FOUR_PI * range_m * frequency_hz / C_LIGHT) ** 2


def dbm(watts: float) -> float:
    """瓦 → dBm。诊断量用，别拿它进公式。"""
    if watts <= 0.0:
        return float("-inf")
    return 10.0 * log10(watts * 1000.0)


def rms(*values: float) -> float:
    """若干量的均方根。信号处理里报"平均功率"时用它。"""
    if not values:
        return 0.0
    return sqrt(sum(v * v for v in values) / len(values))


# ---------------------------------------------------------------------------
# 干扰（AFSIM 的 Jam.1 + RF.5）
#
# 这一段只放**纯公式**，不含任何"谁在压谁"的策略——名册与几何在
# ``services/ew``、受害侧的接入在探测链第 6 步。出处与口径见 §5.14。
#
# ★ 为什么公式放在 ``models`` 而不是 ``services``：光速、4π 这些常数只有一份，
#   它们已经住在本模块；把公式搬进 ``services/ew`` 就会要么重复常数、
#   要么让常数搬到一个叫"电子战"的地方。代价是 ``services`` 不能 import 本
#   模块，所以 ``services/ew`` 只回答**几何与名册**（"谁在射程内、在哪个方位、
#   进没进主瓣"），**"压掉多少"由受害侧自己算**——那个式子要用 l 的调谐带，
#   本来就是受害方的属性（§5.14）。
# ---------------------------------------------------------------------------

def bandwidth_overlap_ratio(
    jammer_frequency_hz: float,
    jammer_bandwidth_hz: float,
    receiver_frequency_hz: float,
    receiver_bandwidth_hz: float,
) -> float:
    """``F_BW``：干扰机频谱落在**接收机调谐带**里的比例（AFSIM 的 RF.5）。

    ``F_BW = min( (min(上限) − max(下限)) / (干扰机谱宽), 1 )``，完全错开取 0。
    **分母是干扰机的谱宽**——这是"阻塞式吃亏、瞄准式不吃亏"的全部来源：

    - **瞄准式**（``B_j`` 小、落在带内）⇒ ``F_BW = 1``：功率全进得去。
    - **阻塞式**（``B_j`` ≫ ``B_r``）⇒ ``F_BW = B_r / B_j``：
      功率摊在整个宽带上，只有一小份落在接收机带宽里。
    - **完全错开**（``Δf ≥ (B_j + B_r)/2``）⇒ ``F_BW = 0``：一点都进不去。
      跳频干扰能生效，靠的就是把这一项压到 0。

    ★ **两个中心频率都要显式给**。AFSIM 文档的符号表把接收调谐带写成以
    ``F_t``（发射机频率）为中心，那只在"雷达自己的收发同频"时才对；干扰机
    是**另一台发射机**，它的谱心与受害接收机的心是两个数。照那样写的话
    "干扰机把频率调到别处"这件事**根本表达不出来**，而瞄准/阻塞的区别正是它。

    ★ ``jammer_bandwidth_hz <= 0`` 按**连续波（CW）**处理：单频点落在
    接收带内就是 1、落在带外就是 0。这不是补丁——CW 瞄准干扰本来就是这个
    形状，而"带宽取 0 ⇒ 分母为零"直接报错会把一个真实存在的干扰样式挡在门外。
    """
    if receiver_bandwidth_hz <= 0.0:
        raise ValueError(f"接收机带宽必须为正，收到 {receiver_bandwidth_hz}")
    f_r, b_r = receiver_frequency_hz, receiver_bandwidth_hz

    if jammer_bandwidth_hz <= 0.0:            # CW：单频点
        return 1.0 if abs(jammer_frequency_hz - f_r) <= 0.5 * b_r else 0.0

    f_j, b_j = jammer_frequency_hz, jammer_bandwidth_hz
    low_j, high_j = f_j - 0.5 * b_j, f_j + 0.5 * b_j
    low_r, high_r = f_r - 0.5 * b_r, f_r + 0.5 * b_r
    if high_j <= low_r or low_j >= high_r:
        return 0.0                            # 完全错开
    overlap = min(high_j, high_r) - max(low_j, low_r)
    return min(overlap / (high_j - low_j), 1.0)


def one_way_loss(two_way_loss_db: float) -> float:
    """（已作废，见 :func:`jamming_path_loss_db`）**双程总损耗 → 单程等效损耗**。

    ★ **v0.13.36 起本函数不再出现在干扰链路上**。它不是"错的公式"，而是
    **问错了问题**：它假设雷达的 ``system_loss_db`` 可以沿两程对称地劈成
    ``L_x·L_r`` 各一半，于是干扰只走一程 ⇒ 拿 ``√L``。AFSIM 的实测口径
    不是这样——它给干扰只扣**受害方接收支路**那一份内损（``receive_loss``），
    发射支路的内损**完全不进**干扰链路（干扰不是雷达自己发出去的，凭什么
    扣它的发射内损）。

    ⇒ 保留本函数只为**旧调用点/旧测试**不留悬空引用，**新代码一律用**
    :func:`jamming_path_loss_db`。实测偏差：用 ``√L`` 比 AFSIM 高
    **+2.4999 dB**（两个独立场景同值，见 §5.14.13）。
    """
    return sqrt(linear(two_way_loss_db))


def jamming_path_loss_db(system_loss_db: float, receive_loss_db: float) -> float:
    """**干扰单程路径损耗**（dB）：只扣受害方的**接收支路**内损（AFSIM 口径）。

    ``L_jam = receive_loss_db``

    ★ **为什么不是 ``½·system_loss_db``**（本项 v0.13.36 的口径修正，§9-54
    结案）：``system_loss_db`` 是**雷达自己**收发两程的总内损，而干扰信号
    **不是这部雷达发出去的**——它从干扰机飞过来，只经过受害方的**接收**
    支路（天线到接收机那一小段）。拿整份双程损耗劈一半再扣给干扰，等于
    "替别人的发射机也扣了一笔"。

    ★ AFSIM 实测（`demos/electronic_warfare/barrage_jamming.txt`，两个独立
    场景都做了同一组对照）：
    ``√L``(``system=4.5 dB`` 时取 2.25) ⇒ ``J`` 偏高 **+2.4999 dB**；
    只扣 ``L_rx``(7 dB) ⇒ ``J`` 差 **−0.0001 dB**。两者的差
    ``7 − 4.5 = 2.5 dB`` 精确解释那个偏差 ⇒ 后者是 AFSIM 的真实口径。

    ★ ``receive_loss_db`` 从哪来：AFSIM 想定里雷达 ``transmitter`` 块的
    ``receive_loss``。本项目的 ``system_loss_db`` 是**发射内损 + 接收内损**
    混在一起的一个数（见那条参数的说明），所以新加一个显式的
    ``receive_loss_db`` 参数把"接收那一份"单拎出来；**没给**（0）⇒
    退化成"不记损耗"（旧想定里 ``system_loss_db=0`` 的行为逐字不变，
    而 ``system_loss_db>0`` 的想定会看到 J 抬高 —— 那是**修正**，不是回归）。
    """
    return max(0.0, float(receive_loss_db))


#: 极化类型。**七个取值、顺序照抄 AFSIM**（``WsfEM_Types.hpp`` 的
#: ``cPOL_DEFAULT``…``cPOL_RIGHT_CIRCULAR``；字符串拼写取 ``WsfEM_Util.cpp``）。
#: ``"default"`` 是 AFSIM 的默认值，含义是"没声明极化"⇒ 不产生失配。
POLARIZATION_KINDS: tuple[str, ...] = (
    "horizontal",
    "vertical",
    "slant_45",
    "slant_135",
    "left_circular",
    "right_circular",
    "default",
)

#: ``F_POL`` 表：**接收机主极化 × 来波极化** ⇒ 收到的那一份（线性比例）。
#:
#: 出处是 AFSIM 的 ``WsfEM_Rcvr::UpdatePolarizationEffects()``（源码）与
#: ``doc/receiver.rst`` 里那张同名表（文档），两处**逐格一致**——本表照抄，
#: **不化简、不参数化**：取值只有 **1.0（匹配）/ 0.5（−3 dB）/ 0.0（全挡）**
#: 三种，而它们分别对应"同极化" / "线极化与线极化差 45°、或线极化对圆极化"
#: / "正交"三类几何关系。写成一个 ``cos²Δθ`` 公式会更短，但 AFSIM 的表里
#: **圆极化对线极化恒为 0.5**、与自己恒为 1.0（不是 ``cos²``），把这两族塞进
#: 一个公式反而要加例外分支——照抄表是最不容易出错的那条路。
#:
#: 行序 = 列序 = :data:`POLARIZATION_KINDS`。
_POLARIZATION_EFFECTS: tuple[tuple[float, ...], ...] = (
    (1.0, 0.0, 0.5, 0.5, 0.5, 0.5, 1.0),   # 接收 horizontal
    (0.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0),   # 接收 vertical
    (0.5, 0.5, 1.0, 0.0, 0.5, 0.5, 1.0),   # 接收 slant_45
    (0.5, 0.5, 0.0, 1.0, 0.5, 0.5, 1.0),   # 接收 slant_135
    (0.5, 0.5, 0.5, 0.5, 1.0, 0.0, 1.0),   # 接收 left_circular
    (0.5, 0.5, 0.5, 0.5, 0.0, 1.0, 1.0),   # 接收 right_circular
    (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),   # 接收 default（没声明 ⇒ 不判）
)

_POLARIZATION_INDEX = {name: i for i, name in enumerate(POLARIZATION_KINDS)}


def polarization_effect(
    receiver_polarization: str,
    transmitter_polarization: str,
    override: float = -1.0,
) -> float:
    """``F_POL``：来波被接收机**收下**的那一份（线性，AFSIM 的 RF.4a / RF.5）。

    ★ **这是 AFSIM 的真实机制，不是"手填一个 dB"**：两端各声明一个极化
    **类型**，损失由那张 7×7 表查出来。所以"交叉极化要扣多少"**不需要人来算**，
    也就不存在"填了一个没人管的数"。

    ★ **只用在单向链路上**：干扰、（以后）无源传感器与通信。雷达**双程不乘**
    它——AFSIM 的干扰方程文档写明 "This is not incorporated for radar
    interactions..."（收发共用一部天线，视为匹配）。我们这条链只算干扰，
    所以 ``received_power`` 里没有它，只有 :func:`jamming_power` 有。

    ★ ``override`` 对应 AFSIM 接收机的 ``polarization_effect <pol> <fraction>``
    命令：**显式给了就用显式值**（源码 ``UpdatePolarizationEffects()`` 里
    `若该项被显式声明则覆盖默认表`）。约定 **负数 = 未给** ⇒ 走查表；这也
    让"没声明这个参数"与"声明了恰好等于 1.0"区分得开——两者物理上不同：
    前者随两端类型变，后者永远不扣。
    """
    if override >= 0.0:
        return override
    try:
        row = _POLARIZATION_INDEX[receiver_polarization]
        column = _POLARIZATION_INDEX[transmitter_polarization]
    except KeyError as exc:
        raise ValueError(
            f"未知的极化类型 {exc.args[0]!r}，合法取值见 POLARIZATION_KINDS："
            f"{'、'.join(POLARIZATION_KINDS)}"
        ) from None
    return _POLARIZATION_EFFECTS[row][column]


def jamming_power(
    power_w: float,
    jammer_gain: float,
    victim_gain: float,
    wavelength_m: float,
    range_m: float,
    loss: float = 1.0,
    bandwidth_overlap: float = 1.0,
    polarization: float = 1.0,
) -> float:
    """**Jam.1**：干扰机在受害接收机输入端造成的功率（W）。

    ``P_j = P_peak · G_j · G_r · λ² · F_BW · F_POL / ((4π)² · R² · L)``

    = AFSIM 的 RF.1（发射）→ RF.2b（**单程**传播）→ RF.4a（接收）三步相乘。

    ★ **与 :func:`received_power` 差一个 ``(4πR/λ)²``，也就是 R⁻² 而不是 R⁻⁴**
    ——回波走两程、干扰走一程。这是干扰最要紧的一条：接收到的干扰功率按
    **距离平方**衰减，而回波按**距离四次方**衰减。于是存在一个**烧穿距离**——
    目标越近，回波涨得越快，最终压过干扰。

    ★ 所有增益/损耗都是**线性**值，``loss`` 在这里是**单程路径损耗**
    （**只含受害方接收支路那一份**，见
    :func:`~milsim.models.percep.equation.jamming_path_loss_db` 与 §9-54），
    因此分母里只有**一个** ``L``，与 :func:`received_power` 同形。

    ★ ``polarization`` 仍然只作为**输入因子**（默认 1.0 = 匹配），本函数**不**
    自己去查极化类型——那是 :func:`polarization_effect` 的活，由**受害方**
    在调用这里之前算好并传进来。分工与 ``F_BW`` 完全一样：这一层是纯公式，
    "两端各是什么极化"属于调用方（受害方知道自己、也知道名册上那台是什么）。
    ★ 传进来的是**线性比例**（1.0 / 0.5 / 0.0），不是 dB。
    """
    if range_m <= 0.0:
        raise ValueError(f"距离必须为正，收到 {range_m}")
    if power_w <= 0.0 or loss <= 0.0:
        return 0.0
    return (
        power_w * jammer_gain * victim_gain * wavelength_m * wavelength_m
        * bandwidth_overlap * polarization
        / (FOUR_PI**2 * range_m**2 * loss)
    )


def jamming_to_signal(
    jammer_power_w: float, noise_w: float, signal_to_noise: float
) -> float:
    """``J/S``：由**绝对的**干扰功率、噪声功率与信号信噪比算出。

    ``S = SNR · N``（信号功率的定义），所以 ``J/S = J / (SNR · N)``。

    ★ 这一步的存在说明了**标定法为什么接不上干扰**：标定法把 ``N`` 折进了
    标定好的 SNR，手上**没有绝对的噪声功率**（``noise_w = 0``）。而
    ``J/S`` 需要 ``S`` 绝对、``J/N`` 需要 ``N`` 绝对——两者都要一个绝对
    基准。所以标定法要接干扰，必须显式给 ``noise_power``（AFSIM 的 RF.6
    第 1 条"给了 ``noise_power`` 就用它"正是这个口子）。
    """
    if noise_w <= 0.0:
        raise ValueError("噪声功率必须为正——标定法没有绝对的 N，接不了干扰")
    if signal_to_noise <= 0.0:
        return float("inf")
    return jammer_power_w / (signal_to_noise * noise_w)


def snr_under_jamming(signal_to_noise: float, jam_to_signal: float) -> float:
    """**干扰下的信噪比**：``SN = SNR / (1 + (J/S)·SNR)``。

    由 ``S/(N+J)`` 上下同除 ``N`` 得到：``(S/N) / (1 + (J/N))``，
    再代入 ``J/N = (J/S)·(S/N)``。

    ★ 这个写法与直接算 ``S/(N+J)`` **代数恒等**（实测 21 组、``J/S`` 从
    −20 到 +40 dB、三个距离，最大相对差 **1.795e-16**），但它有两个好处：

    1. **只用到 ``J/S`` 与已经算好的 SNR**，不需要再走一遍雷达方程；
    2. **对零干扰退化成恒等式**：``J/S = 0`` ⇒ 原样返回 SNR。
       这就是"没挂干扰机时输出逐字不变"的数学依据，不是靠分支挡住的。
    """
    if jam_to_signal <= 0.0:
        return signal_to_noise
    return signal_to_noise / (1.0 + jam_to_signal * signal_to_noise)


def antenna_pattern_factor(
    off_axis_deg: float, beam_width_deg: float, sidelobe_linear: float
) -> float:
    """天线方向图在偏轴 ``off_axis_deg`` 处的**相对增益因子**（线性，≤ 1）。

    ⟨最省⟩口径（§5.14.11 ④）——**两段拼起来，没有第三种花活**：

    * **主瓣内**（``off <= θ/2``）：``cos²(π/4 · off/(θ/2))``。取余弦平方是因为
      它在轴心处为 1、在**主瓣边缘恰好降到 0.5**（−3.0103 dB，正是一个半功率
      波束宽度该有的定义），且**连续**——换个更陡的函数会让"波束边缘"这个
      判据与"边缘 3 dB"那句话对不上。
      ★ 角里那个 ``π/4`` 不能写成 ``π/2``：``π/2`` 在边缘处 ``cos(π/2)=0``
      ⇒ 因子归零（实测还留下 ``3.7e-33`` 的浮点尾巴），而边缘该是 0.5。
    * **主瓣外**：常数 ``sidelobe_linear``（= 10^(−13/10) ≈ 0.05012）。**不做
      副瓣起伏、不做扇贝损失**——那需要一张天线实测表，而我们现在没有；
      编一个起伏出来就是把不确定当已知（⟨最省⟩）。
      ★ 于是边缘（0.5）到主瓣外（0.0501）是一个**跳变**（约 10 dB）。这是
      ⟨最省⟩口径的固有代价，如实记录：不编过渡段，因为任何过渡段的形状都
      是猜的。

    ★ ``beam_width_deg <= 0`` ⇒ 恒返 ``1.0``（**各向同性**）。这是"不写波束
    宽度 = 旧行为逐字不变"的实现处：不是让角度算出来恰好是 1，而是**根本不
    走角度那条路**，乘上去也只多一次 ``× 1.0``（IEEE754 下逐位不变）。

    ★ ``sidelobe_linear`` 由**调用方**给（组件从 ``DEFAULT_SIDELOBE_LEVEL_DB``
    折出来），这样"副瓣取多少 dB"这件事仍然只有 ``services.ew`` 那一个出处。
    """
    if beam_width_deg <= 0.0:
        return 1.0
    half = beam_width_deg * 0.5
    off = abs(off_axis_deg)
    if off > half:
        return sidelobe_linear
    return cos(pi * 0.25 * off / half) ** 2
