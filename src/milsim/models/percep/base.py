"""雷达传感器基类：探测链 8 步的落点。

第 1~4 步与第 5~8 步的分界
--------------------------

前四步（对表 / 转向 / 粗筛 / 精筛）是**扫描**：天线此刻在哪、这一拍照到谁。
后四步（回波功率 / 信噪比 / 检测概率 / 建航）是**探测**：照到了不等于看得见。

v0.13.24 之前只有前四步，后四步的位置上站着两行占位——``if distance > range:
continue`` 与 ``quality = 1 - d/range``。本模块把后四步补齐：

.. code-block:: text

    第 4 步（补完）  几何：地平线 + 地形通视（同一个调用，见下）
    第 5 步         回波功率   Pr = Pt·G²·λ²·σ / ((4π)³·R⁴·L)
    第 6 步         信噪比     S = Pr / (k·T_sys·B)
    第 7 步         检测概率   Pd = 非起伏(S, N, Pfa)   ← 默认；swerling_case 1 则 Swerling1
    第 8 步         建航       M/N：hits_to_establish / hits_to_maintain

.. note::

    **地平线不是第 4 步里另加的一条判据。** 它就是通视判定里
    ``earth_curvature=True`` 那部分的几何结果——越过地平线后曲率下沉量超过
    视线高度，余隙转负。单独再判一次 ``radio_horizon_m`` 属于"同一个量两处
    各判一次"，迟早出现"过了地平线却报通视"的不一致。所以这里只有一个调用。

★★ 送进 Pd 的是**单脉冲**信噪比（见 :mod:`~milsim.models.percep.equation`
的模块头第 1 条）。AFSIM 的 ``signal_to_noise`` 恰好是"积累后"的口径，
直接拿来喂 Swerling1 就是把积累增益算两次——症状是作用距离整体外推，而
航迹表看着完全正常。对照数字见设计文档 §5.12.8。

两条路算 SNR：标定法 / 逐参法
-----------------------------

真实数据表通常只给一个"探测距离"。已落库的 6 型雷达**只有**
``detect_range`` 与 ``scan_interval`` 两个真参数（频率、转速、波束宽度全躺在
``attr`` 里当文本），所以不能要求想定作者填满一整张发射机参数表。

* **标定法**（默认）：认定"``detect_range`` 处、对 ``reference_rcs`` 的目标，
  单次扫描的 Pd 恰好等于 ``required_pd``"。于是
  ``S_ref = snr_for_pd(required_pd, N, Pfa)``（★ v0.13.40 起默认按**非起伏**
  反解，与默认的 ``swerling_case 0`` 同口径），再由 ``Pr ∝ R⁻⁴`` 外推到别的
  距离：``S(R) = S_ref · (σ/σ_ref) · (detect_range/R)⁴``。
  两个数据表数字就能给出一条**有物理形状**的 Pd-距离曲线。
* **逐参法**：给全 ``peak_power`` / ``frequency`` / 增益（或两个波束宽度）等，
  直接从雷达方程算。

**哪条路由参数本身决定，且半给不行**：任何一个发射机参数被写下来（非默认
值）就必须把该路所需的一整套给全，缺一项在装配期报
:class:`~milsim.errors.ConfigurationError`。静默退回标定法的后果是"我明明
填了功率"却一点没生效，而数字看着仍然合理——这类静默退化是本项目明确要
消灭的（§9 第 15 条、弹的"推重比 < 1 报错不跑假航迹"同理）。

第 8 步：M/N 建航
-----------------

``hits_to_establish`` / ``establish_window``：最近 ``establish_window`` 次
**被照射**里命中 ``hits_to_establish`` 次 ⇒ 建航。
``hits_to_maintain`` / ``maintain_window``：已建航的，最近 ``maintain_window``
次被照射里命中不足 ``hits_to_maintain`` 次 ⇒ 撤航（并 ``drop_contact``）。

**★ 默认档是 `3/5` 建、`2/5` 维持**（v0.13.39 起；旧默认是全 `1`，那等于
"一次命中即建航、一次漏检即撤航"，M/N 这道工序**形同不存在**）。要"一次命中
即建航"（比如只测几何 / 扫描、不测 M/N）就**显式**写 `1/1/1/1`。

**分母是"被照射"，不是"每一拍"**——波束这一拍不在目标那个方位时，目标
既没被照射也不构成漏检，把它算成未命中会让高转速雷达的建航判据随扫描
周期漂移。另一头**几何遮挡算漏检**（波束照到了它，只是什么也没收到），
这两件事必须分开。

默认 ``1 / 1`` 表示"每次探测即建航"——等价于没有建航逻辑，与 v0.13.24
的行为逐字相同。要真做 M/N 就显式写，例如 AFSIM 的 ``3 5`` / ``1 3``。

第 7c 步：假目标（v0.13.30，§5.14）
----------------------------------

**在候选循环之前、每拍一次**。它不改已有的八步，只是往航迹表里添一批
**不对应实体**的航迹（``Contact.phantom``），并在本拍的真实检测上叠一道
"被淹没就阻断"：

* 条数按 AFSIM 的 ``NumberOfFalseTargets`` 算（要用**本雷达自己的** PW / PRI /
  扫描时间），"想造多少条"与"屏幕显示几条"由两个不同的数表达；
* 幽灵铺在**干扰机之外**（``c·τ/2`` 一格，最多铺到不模糊距离 / 探测半径），
  所以"比最近的幽灵还近的真目标"必然放行——这条几何优先权是白送的；
* 淹没判据（``N_ft > capacity``）走 AFSIM 的随机抽签式。

★ **没挂假目标干扰机时这条路一步都不走**（``false_targets_seen`` 恒为 0 ⇒
不抽签、不查名册、不动航迹表），所以"没挂干扰机的想定逐字不变"这条判据在
这里也成立。

诊断量
------

``ticks`` / ``swept_deg`` / ``sweeps`` / ``contacts_seen`` 沿用；新增
``detections``（命中次数，含未建航的）、``geometry_blocked``（几何判掉的
候选数）、``last_pd``（最近一次被照射目标的检测概率）——后者是"雷达到底
看得见多远"的最直接读数，比航迹条数灵敏得多。

``contacts_seen`` 的口径是**写进航迹表的次数**：命中**且**航迹成立/维持才算，
所以它等于"真正被看到"的次数，而不是"被照射"的次数。四个量合起来才分得清
四种情况——没扫到（谁都不动）、几何遮挡（``geometry_blocked``）、照射了但
没测到（``detections`` 不动、``contacts_seen`` 不动）、测到了（都动）。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import cos, hypot, isfinite, radians, sin, sqrt

from ...engine import PRIORITY_SENSOR, EventResult
from ...errors import ConfigurationError
from ...services.ew import DEFAULT_SIDELOBE_LEVEL_DB, FalseTargetSpec
from ...services.params import Params
from ...services.store import IFF_FOE, IFF_FRIEND, IFF_UNKNOWN, phantom_id
from ..component import Component
from . import equation as eq

#: 第 5~8 步用的默认值。**都写成具名常量**，因为它们是"没写参数时雷达是
#: 什么行为"的唯一出处，散在函数体里会让"这型雷达为什么看得见"无从回答。
DEFAULT_REQUIRED_PD = 0.5
"""标定法的 Pd 目标：``detect_range`` 处恰好有一半的机会被发现。

0.5 是刻意的：它就是"检测概率曲线的半功率点"，与"探测距离"这个词在
数据表里的通常含义一致（而不是 0.9 那种"保证发现"的口径）。改它等于
改整条曲线的标定锚点，属于想定层的决定。
"""

DEFAULT_REFERENCE_RCS = 1.0
"""标定法与 RCS 缺失时的参考目标 RCS（m²）。"""

DEFAULT_ANTENNA_NOISE_TEMPERATURE = eq.STANDARD_TEMPERATURE
"""天线噪声温度 ``T_a`` 的默认值（K）——**取 T0**。

这个默认值不是"天线就在 290 K 下"的物理断言，而是一个**兼容性**选择：
``T_a = T0`` 且馈线 0 dB 时，Blake 级联式退化成 ``T_sys = T0·F``，与
v0.13.25 的噪声口径逐字相同（AFSIM 文档里"给了 ``noise_figure``、两个
损耗项都没给"那条支路也正是它）。

要按 AFSIM 那型冷空天线对齐就得**显式**给：从它的 ``noise_power``
反推是 ≈ 42.8 K（见 §5.12.8 ⑦）。
"""


@dataclass(frozen=True, slots=True)
class FalseTargetSource:
    """一台干扰机**本拍要造的那一串幽灵**从哪儿起、造多少条。

    纯数据，受害方算完之后放在 :attr:`JammingEffect.false_targets` 里。

    ★ 为什么"从哪儿起"必须带上：假目标的距离**不可能比干扰机更近**
    （AFSIM 的 ``false_target`` 原文：``range_constrained`` 默认 ``false``，
    因为"把假目标放到干扰机**前面**需要预知雷达的 PRF 序列"——要抢在真回波
    之前把假脉冲发出去）。所以整串幽灵都铺在**干扰机之外**，而这正是
    "比最近的幽灵还近的真目标一定放行"那条几何优先权的由来。
    """

    #: 造这批幽灵的干扰机。
    entity_id: int
    #: 幽灵铺在哪个方位（度）——**干扰机自己的方位**：距离 / 方位欺骗都是沿
    #: 视线拉的，假回波和干扰机从同一个方向进来。
    bearing_deg: float
    #: 最近那条幽灵的斜距（米）= 干扰机斜距 + 一个距离分辨单元。
    nearest_m: float
    #: 这批幽灵的条数（AFSIM 的 ``NumberOfFalseTargets``，**未**被槽位截断）。
    count: int


@dataclass(frozen=True, slots=True)
class JammingEffect:
    """一次求值里，干扰对**某一部受害接收机**的全部后果。

    AFSIM 把求值点定在"检测发生的那一刻"，而那一刻有三件事同时发生：功率
    并进噪声（压制）、量测被挪走（欺骗）、屏幕被幽灵填满（假目标）。它们
    **同源**（同一批干扰机、同一次几何查询），所以必须一起算出来——分几次
    算就会有几条"谁在压我"的答案，而它们迟早不同步。

    ==================  ================================================
    ``power_w``         并进噪声的干扰功率（W）。0 ⇒ ``S/(N+J)`` 原样
    ``peak_j_to_s``     进得来带的那几台里**最强**的 ``J/S``（诊断 + 门限）
    ``in_band``         进得来带的台数（诊断）
    ``pullers``         骗成了的台数（诊断）
    ``range_bias_m``    距离量测偏置（米），正 = 报得更远
    ``azimuth_bias_deg``方位量测偏置（度），正 = 顺时针
    ``false_targets``   放假目标的那些台（逐台一条 :class:`FalseTargetSource`）
    ==================  ================================================

    ★ 假目标**不在**这个求值点被"铺出去"：它的条数公式里没有目标、方位取的是
    干扰机自己的方位，铺的动作是**每拍一次**（``_update_false_targets``）。
    这里只是把"谁要造、造多少"算出来。
    """

    power_w: float = 0.0
    peak_j_to_s: float = 0.0
    in_band: int = 0
    pullers: int = 0
    range_bias_m: float = 0.0
    azimuth_bias_deg: float = 0.0
    false_targets: tuple[FalseTargetSource, ...] = ()

    @property
    def any(self) -> bool:
        """有没有任何后果（功率 / 偏置 / 幽灵）。"""
        return (
            self.power_w > 0.0
            or self.range_bias_m != 0.0
            or self.azimuth_bias_deg != 0.0
            or bool(self.false_targets)
        )

    @property
    def false_target_count(self) -> int:
        """本拍这批干扰机一共想造多少条幽灵（**未**截断）。"""
        return sum(source.count for source in self.false_targets)


class RadarSensor(Component):
    """雷达探测链的公共部分：扫描律 + 几何 + 方程 + 建航。

    它**不是**可直接注册的实现（族约定：基类不注册，见 ``models/mover/base.py``）。
    具体型号继承它并只给默认值/注册名。
    """

    SLOT_HINT = "sensor"

    PARAMS = {
        # -- 第 1~3 步：扫描 --------------------------------------------------
        "detect_range": Params.distance(30_000.0, minimum=0.0),
        "scan_interval": Params.duration(2_000_000, minimum=1_000),
        "beam_step": Params.duration(1_000_000, minimum=1_000),
        #: **扫描视场**（度）：天线摆动的范围。360 = 圆周；0 < fov < 360 =
        #: 扇区扫描；0 = 固定指向（跟踪雷达 / 相控阵）。它**不是**波束宽度——
        #: 一次照射有多宽看 ``beam_width``（§9-37 已结）。
        "fov": Params.angle(360.0, minimum=0.0, maximum=360.0),
        #: 基准指向（度，正北 0°、顺时针）：扇区的**中心**方位，或固定指向时
        #: 天线朝的那个方位。圆周扫描下它只决定相位的零点。
        "scan_center": Params.angle(0.0, minimum=0.0, maximum=360.0),
        #: 波束自身的宽度（度）——**瞬时**张角。默认 0 是"波束宽度远小于每拍
        #: 转角"的隐含前提。固定指向（``fov == 0``）时**必须给非零值**。
        #: 它同时是第 6 步"驻留时间"里那个张角（``dwell = 波束宽 ÷ 角速度``）。
        "beam_width": Params.angle(0.0, minimum=0.0, maximum=360.0),
        # -- 第 3 步的上界 ----------------------------------------------------
        #: 搜索半径的上界（米）。0 = 用 ``detect_range``。
        #: 为什么要有它：标定法下 Pd 在 ``detect_range`` 处正好等于
        #: ``required_pd``，若搜索上界也钉在那里，雷达在标称距离**以外**
        #: 就永远发现不了任何东西——真实雷达只是概率低，不是不可能。
        "max_range": Params.distance(0.0, minimum=0.0),
        # -- 第 4 步：几何 ----------------------------------------------------
        #: 天线离地/离海面的架高（米）。进通视判定的视线起点高度。
        #: **0 = 没有架高这一项 ⇒ 不判几何**（**不是**"天线贴在地面上"，
        #: 那个解释会让雷达一条航迹都建不起来）——见
        #: :meth:`_has_line_of_sight` 与 §9-42。要真的判几何就写一个正数。
        "antenna_height": Params.distance(0.0, minimum=0.0),
        # -- 第 5~7 步：方程 --------------------------------------------------
        #: 标定法的 Pd 目标（见模块头）。
        "required_pd": Params.number(DEFAULT_REQUIRED_PD, minimum=0.0, maximum=1.0),
        #: 参考目标 RCS（m²）：``detect_range`` 是对着多大的目标标的。
        #: 目标实际 RCS 读平台属性 ``radar_cross_section``（默认同值）。
        "reference_rcs": Params.area(DEFAULT_REFERENCE_RCS, minimum=0.0),
        #: 虚警概率。1e-6 是雷达界的常见取值，也是 AFSIM 例子的取值。
        "probability_of_false_alarm": Params.number(1.0e-6, minimum=0.0, maximum=1.0),
        #: 一次照射里非相干积累的脉冲数。0 = 由驻留时间与 PRF 算（见
        #: ``pulse_repetition_frequency``）；算不出来时取 1。
        "pulses_integrated": Params.integer(0, minimum=0),
        #: 目标的起伏模型（Swerling 情形）。**v0.13.40 新增**。
        #:
        #: * ``0``（**默认**）= **非起伏**（Swerling 0/5）：目标的 RCS 是常数，
        #:   检测统计量服从**非中心 χ²**（自由度 ``2N``、非中心参量 ``λ = 2NS``）。
        #: * ``1`` = **Swerling 1**：RCS 逐扫描服从指数分布，Pd 是解析平均。
        #:
        #: ★ 为什么默认 ``0``：本模型喂进雷达方程的 RCS 来自平台属性
        #: ``radar_cross_section``（见 :meth:`_rcs_of`）——那是个**常数标量**，
        #: 方程里 ``σ/σ_ref`` 也是确定的。给一个确定的目标配一个"逐扫描起伏"
        #: 的检测模型，是语义错配（§9-73）。旧口径（一律 Swerling 1）会让
        #: Pd **系统性偏低**（起伏模型"偶尔回波特别弱"⇒ 那些次漏检），
        #: `Pd=0.5` 处偏保守约 **1.53 dB**（`Pfa=1e-6`、`N=1`：
        #: 非起伏 11.2426 dB vs Swerling 1 12.7719 dB）。
        #:
        #: ★ 想复现"逐扫描起伏"的旧行为（或与 AFSIM 的 ``swerling_case 1``
        #: 对齐）就在想定里写 ``swerling_case 1``。
        "swerling_case": Params.integer(0, minimum=0, maximum=5),
        #: 脉冲重复频率（Hz）。给了它、且 ``beam_width > 0``，
        #: ``pulses_integrated = 0`` 时按 ``驻留时间 × PRF`` 算 N。
        "pulse_repetition_frequency": Params.number(0.0, minimum=0.0),
        #: 发射峰值功率。0 = 未给 ⇒ 走标定法。想定里可写 ``250 kW``；
        #: 不带单位时按 **W**（与 v0.13.29 及以前逐字相同——裸数因子为 1）。
        "peak_power": Params.power(0.0, minimum=0.0),
        #: 工作频率（Hz）。★ **两条路都要它**：逐参法用它取波长，
        #: 标定法用它算干扰的带内匹配 ``F_BW`` 与波长。所以**它不决定走哪条路**
        #: ——"走不走雷达方程"只看有没有写 ``peak_power``（v0.13.30 起）。
        "frequency": Params.frequency(0.0, minimum=0.0),
        #: 峰值增益（dB）。0 = 未给；与两个波束宽度任给其一即可（逐参法）。
        "antenna_gain_db": Params.number(0.0),
        #: 方位/俯仰波束宽度（度）。两个都给时按 ``G = 4π/(θ_H·θ_V)`` 反推增益。
        "beamwidth_h": Params.angle(0.0, minimum=0.0),
        "beamwidth_v": Params.angle(0.0, minimum=0.0),
        #: **双程总损耗**（dB）：发射内损 + 接收内损 + 波束形状……全在里面。
        #: 写进方程分母时只有**一个** L，不能写 ``L²``。
        #: ★ **不含馈线**——``receive_line_loss_db`` 那一项只进噪声支（AFSIM
        #: 口径，§5.12.8 ⑧），把它同时算进这里会让同一个损耗生效两次。
        #: ★ **它不用于干扰链路**（v0.13.36 口径修正，§9-54）：干扰只扣接收
        #: 支路那一份，见 ``receive_loss_db``。
        "system_loss_db": Params.number(0.0, minimum=0.0),
        #: **发射支路内损**（dB）。v0.13.36 加：给"想分开记收发内损"的想定用。
        #: ★ **它不进干扰链路**——干扰信号不是这部雷达发出去的（见 §9-54）。
        #: 只写它、不写 ``system_loss_db`` ⇒ 干扰看不到它，回波也看不到它
        #: （回波走的是 ``system_loss_db``）⇒ 等于**写了个不生效的数**。
        #: 所以它在"只写了它、没写 system_loss_db"时会**报错**，不静默吞掉。
        "transmit_loss_db": Params.number(0.0, minimum=0.0),
        #: **接收支路内损**（dB）。v0.13.36 加：这是 AFSIM
        #: ``transmitter.receive_loss`` 的对应物，也是**干扰链路唯一该扣的那一份**。
        #: ``L_jam = receive_loss_db``（单程、只这一份）——见
        #: :func:`~milsim.models.percep.equation.jamming_path_loss_db` 与 §9-54。
        #: ★ 与 ``receive_line_loss_db`` **不是一回事**：那个是"天线到接收机的
        #: 馈线损耗"，AFSIM 只把它折进**噪声温度**；这个进**信号/干扰的功率**。
        #: 默认 0 ⇒ 干扰不记损耗（旧想定里 ``system_loss_db=0`` 的行为逐字不变）。
        "receive_loss_db": Params.number(0.0, minimum=0.0),
        #: 接收机噪声系数（dB）。与下面两项一起进 Blake 级联式
        #: :func:`~milsim.models.percep.equation.cascade_noise_temperature`。
        "noise_figure_db": Params.number(0.0, minimum=0.0),
        #: **天线到接收机的馈线损耗**（dB）。★ 它**不衰减信号**，只抬高系统
        #: 噪声温度——AFSIM 的 ``receive_line_loss`` 就只进噪声那一支。
        #: 所以填 ``system_loss_db`` 时**不要**再把它算进去。默认 0 dB。
        "receive_line_loss_db": Params.number(0.0, minimum=0.0),
        #: **天线噪声温度**（K）：Blake 级联式里的 ``T_a``——**已经含**天空噪声
        #: 与天线欧姆损耗两项（AFSIM 拆成 ``Tant`` 与 ``antenna_ohmic_loss``
        #: 两个参数给，后者默认 0 dB ⇒ 两者等价）。默认见
        #: :data:`DEFAULT_ANTENNA_NOISE_TEMPERATURE`。
        "antenna_noise_temperature": Params.number(
            DEFAULT_ANTENNA_NOISE_TEMPERATURE, minimum=0.0
        ),
        #: 接收机带宽（Hz）。0 = 由 ``pulse_width`` 取 ``1/τ``。
        #: ★ **标定法也要它**：干扰是按"带内功率"算的（AFSIM 的 ``F_BW``），
        #: 而 ``F_BW`` 的分母要接收机的调谐带宽。标定法**不**用它算噪声
        #: （标定法的 N 折进标定好的 SNR 里了，见 ``noise_power``）。
        "bandwidth": Params.frequency(0.0, minimum=0.0),
        #: **接收机噪声功率**（W）。★ 只有**标定法**需要它，理由是一条算术：
        #: ``J/S = J/(SNR·N)`` 要有**绝对的 N**，而标定法把 N 折进了那条
        #: "距离 ↔ Pd"的曲线里，手上没有绝对值（``noise_w = 0``）。
        #: AFSIM 的 RF.6 第 1 条就是"给了 ``noise_power`` 就用它"，同一个口子。
        #: 标定法要参与干扰求值就必须给；**没被干扰时给不给都不影响任何数字**。
        #: ★ 逐参法**不许**给：那时 N 由"天线温度 + 馈线 + 噪声系数"这条链
        #: 算出来（§5.12.8 ⑧），再给一个就是同一个量的第二个出处。
        "noise_power": Params.power(0.0, minimum=0.0),
        #: 脉冲宽度（微秒）。``bandwidth`` 为 0 时用它定 ``B = 1/τ``。
        #: ★ 它**同时**是距离量测误差的波形输入（``σ_R = c·τ/(2·√(2·SNR))``）。
        "pulse_width": Params.duration(0, minimum=0),
        # -- 第 7b 步：量测误差（AFSIM 的两条路）-------------------------------
        #: 方位量测误差 σ（度）——**直接给**的那一支。对应 AFSIM 的"通用传感器
        #: 误差模型"（``compute_measurement_errors false``，也是它的默认值）
        #: 与 ``standard_sensor_error`` 里的 ``azimuth_error_sigma``。
        #: **默认 0 = 不注入**，于是航迹位置逐字等于目标真实位置。
        "azimuth_error_sigma": Params.angle(0.0, minimum=0.0),
        #: 距离量测误差 σ（米）——同上，对应 ``range_error_sigma``。默认 0。
        #: ★ **高度不注入误差**（真雷达测不到目标高度，靠的是别的渠道），
        #: 与 AFSIM 的 ``2d_position_error_sigma``（"目标高度将无误差地报告"）
        #: 同口径。所以这里**没有** elevation 那一项。
        "range_error_sigma": Params.distance(0.0, minimum=0.0),
        #: 改为**由 SNR 反算**误差（对应 AFSIM 的"标准雷达误差模型"，
        #: ``compute_measurement_errors true``）：σ 由波束宽度与脉冲宽度按
        #: ``1/√(2·SNR)`` 算出 ⇒ **Pd 大 ⇒ 误差小**。
        #: 与上面两个 sigma 是**互斥**的两条路；同时给会当场报错，见
        #: :meth:`RadarSensor._resolve_errors`。
        "compute_measurement_errors": Params.boolean(False),
        # -- 第 7c 步：假目标（§5.14）------------------------------------------
        #: 航迹表的**槽位容量**：假目标干扰把屏幕淹掉时"最多能显示多少条"。
        #: 默认 1000 —— AFSIM 的 ``false_target_screener.track_capacity`` 默认
        #: 1000，``WSF_SIMPLE_FT_EFFECT.maximum_false_target_capacity`` 也是
        #: 1000（有 screener 时用它的 ``track_capacity``）。**照抄默认值是
        #: 有意的**：它是"雷达能处理多少条航迹"的工程数量级，不是我们拍的。
        #: ★ 它只管**假目标那一支**：真实航迹本版**不**受它限制（§9-53）。
        "false_target_capacity": Params.integer(1000, minimum=1),
        #: 本雷达的**接收主极化**（AFSIM 接收机的同名参数，同一个七取值集合）。
        #: 默认 ``"default"`` = 没声明 ⇒ 干扰那一支的 ``F_POL`` 恒为 1（不扣）。
        #: ★ 它只管**单向链路**（干扰，以及以后的通信）：**雷达自己的回波不乘
        #: 它**——AFSIM 的干扰方程文档写明 "This is not incorporated for radar
        #: interactions..."（收发共用一部天线，视为匹配）。所以这一项**只影响
        #: 干扰，不影响探测距离**，`detect_range` 标定出来的数一个不动。
        "polarization": Params.string("default", choices=eq.POLARIZATION_KINDS),
        #: **假目标的最低到达功率**（W）：到不了这个值的干扰机，一个幽灵都显不出来。
        #: **0 = 不判**（默认值刻意取 0，好让"没写这一项"的想定数字逐字不变）。
        #: ★ 为什么加这一项：AFSIM 的 ``WSF_SIMPLE_FT_EFFECT`` 只有"密度 / 容量 /
        #: 随机抽签"三样，**没有功率门限**——于是只要参数表上给了密度，一台功率
        #: 小得可怜（甚至几何上远得没用）的干扰机照样能把屏幕撒满。这一项是
        #: **我们自己加的**（§9-51 结案），语义是"检测门限折算到接收机输入端的
        #: 等效功率"，所以它是**受害方**的属性，不是干扰机的（干扰机不知道对方的
        #: 灵敏度，写在这边就不存在"两台互相猜"的问题）。
        "false_target_min_power": Params.power(0.0, minimum=0.0),
        # -- 第 8 步：M/N 建航 -------------------------------------------------
        # ★ **v0.13.39 起默认 3/5 建、2/5 维持**（旧默认是全 1）。
        # 旧默认的语义是"最近 1 次照射里至少命中 1 次"⇒ 退化成**单次抽签**，
        # 没有任何虚警抑制——参数都在、逻辑也对、就是不生效，而这一步不报错。
        # 雷达的常规做法是"建航要严（压住虚警）、维持要松（别把跟上的丢掉）"，
        # 这也是 :meth:`_update_track` docstring 与 ``scenarios/patrol.txt`` 早就
        # 写下的口径。本次只是把**已经写好的口径**落成默认值，不是新造数。
        # 显式写 1/1/1/1 仍能逐字回到旧行为（要"一次命中即建航"的场合——比如
        # 只测几何/扫描、不测 M/N 的用例——必须显式写出来，不再靠默认值兜底）。
        "hits_to_establish": Params.integer(3, minimum=1),
        "establish_window": Params.integer(5, minimum=1),
        "hits_to_maintain": Params.integer(2, minimum=1),
        "maintain_window": Params.integer(5, minimum=1),
    }

    __slots__ = (
        "view",
        "_nav",
        "_target_attr",
        "_side_of",
        "_rng",
        "span_m",
        "period_us",
        "fov_deg",
        "face_deg",
        "beam_deg",
        "phase_deg",
        "spin",
        "swept_deg",
        "ticks",
        "contacts_seen",
        "_last_us",
        "range_model",
        "detect_range_m",
        "max_range_m",
        "antenna_height_m",
        "required_pd",
        "pfa",
        "pulses",
        "reference_rcs",
        "snr_at_reference",
        "power_w",
        "gain",
        "wavelength_m",
        "noise_w",
        "loss",
        "jamming_path_loss",
        "_rx_loss_given",
        "_tx_loss_given",
        "frequency_hz",
        "bandwidth_hz",
        "polarization_type",
        "_jam",
        "jamming_w",
        "jamming_range_m",
        "jamming_azimuth_deg",
        "jammed_evals",
        "jamming_masked",
        "_pull_off",
        "_now_us",
        "false_targets_seen",
        "false_target_min_power_w",
        "phantoms_seen",
        "jamming_blocked",
        "_phantoms",
        "_nearest_phantom_m",
        "error_azimuth_deg",
        "error_range_m",
        "errors_from_snr",
        "_hits",
        "_tracks",
        "detections",
        "geometry_blocked",
        "last_pd",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount) -> None:
        self.view = mount.sensor_view()
        self._nav = mount.nav
        #: 电子战门面（§5.14）。**为 None = 这份装配没有电子战**，此时
        #: 干扰功率恒为 0、``single_pulse_snr`` 逐字不变。
        self._jam = mount.jam_service()
        #: 读**别人**的平台属性（目标 RCS）。这是感知层唯一需要看别人属性的
        #: 地方——位置一律走 ``SensorView``，不从这里拿。
        self._target_attr = getattr(mount, "target_platform_param", None)
        #: 读某实体**属于哪一方**（v0.13.33）。与 ``_target_attr`` 同一条窄
        #: 口子（只读属性，不开位置的口子），用来判这条航迹的敌我。
        self._side_of = getattr(mount, "side_of", None)
        # ``streams().sensor`` 是**属性**（EntityStreams 把 (实体, 用途) 折成一个
        # 命名空间），不是工厂方法——写 ``.sensor()`` 报的是
        # "'Random' object is not callable"。
        self._rng = mount.streams().sensor

        self.span_m = mount.cell_span_m()
        self.period_us = float(self.spec["scan_interval"])
        self.fov_deg = float(self.spec["fov"])
        self.face_deg = float(self.spec["scan_center"]) % 360.0
        self.beam_deg = float(self.spec["beam_width"])
        if self.fov_deg <= 0.0 and self.beam_deg <= 0.0:
            self._require_beam_width()

        self.phase_deg = 0.0
        self.spin = 1
        self.swept_deg = 0.0
        self.ticks = 0
        self.contacts_seen: list[int] = []
        self._last_us = float(mount.now)

        self.detect_range_m = float(self.spec["detect_range"])
        self.antenna_height_m = float(self.spec["antenna_height"])
        upper = float(self.spec["max_range"])
        self.max_range_m = upper if upper > 0.0 else self.detect_range_m
        self.required_pd = float(self.spec["required_pd"])
        self.pfa = float(self.spec["probability_of_false_alarm"])
        #: 目标的起伏模型（v0.13.40）。``0`` = 非起伏（默认，与常数 RCS 一致）；
        #: ``1`` = Swerling 1（旧行为、也是 AFSIM ``swerling_case 1`` 的口径）。
        #: 见参数表 ``swerling_case`` 与 §9-73。
        self.swerling_case = int(self.spec["swerling_case"])
        self.reference_rcs = float(self.spec["reference_rcs"])
        self.pulses = self._resolve_pulses()
        self._resolve_budget()
        self._resolve_errors()
        #: 本雷达的**接收主极化**（AFSIM 接收机同名参数）。它只进干扰那一支的
        #: ``F_POL``：与在册干扰机的**类型**一起查表（§5.14）。
        #: 默认 ``"default"`` ⇒ 恒 1.0 ⇒ "没写这一项"的想定数字一个不动。
        self.polarization_type = str(self.spec["polarization"])
        #: 假目标的**最低到达功率**（W）。0 = 不判（默认）。
        self.false_target_min_power_w = float(self.spec["false_target_min_power"])

        #: 每个目标的"最近若干次被照射"命中记录（True = 命中）。
        #: 只在**被照射**时入队，所以窗口计数的分母是"被照射次数"。
        self._hits: dict[int, deque[bool]] = {}
        #: 已建航的目标。
        self._tracks: set[int] = set()
        self.detections = 0
        self.geometry_blocked = 0
        self.last_pd = 0.0
        #: 最近一次求值算出的干扰功率（W）。诊断用——"干扰到底进来多少"
        #: 在事后没有第二个出口（名册只说谁在压、不说压掉多少）。
        #: ★ 这三个字段与 ``last_pd`` 一样是**在"被照射的那一刻"**写的：目标
        #: 不在本拍照到的弧里就没有求值，读到的会停在上一拍——转速慢的雷达
        #: （``scan_interval`` 远大于拍照间隔）上，这可能是半个周期前的数。
        #: 要逐拍采它们，就得让每拍都照到目标；否则量出来的是"采样密度"，
        #: 不是干扰本身（与"粗节拍量的是采样密度不是飞行状态"同一类）。
        self.jamming_w = 0.0
        #: 最近一次求值算出的**欺骗偏置**（米 / 度）。同上，诊断用。
        self.jamming_range_m = 0.0
        self.jamming_azimuth_deg = 0.0
        #: 有多少次求值的 J > 0（诊断用：它应当只在"有干扰机在册"时非零）。
        self.jammed_evals = 0
        #: 累计有多少台次干扰机**被地形 / 地平线挡在外面**（诊断用）。
        #: 与 :attr:`geometry_blocked` 是一对：那个数目标被挡，这个数干扰机被挡。
        #: 两个都要有——合成一个数就再也分不清"是真的没有干扰，还是干扰被山挡住了"。
        self.jamming_masked = 0
        #: 距离拖引（RGPO）的**行程表**：干扰机实体 ID → 这一轮从哪一刻开始拉。
        #: 这是**真状态**，不是派生量缓存——它记的是"被跟踪了多久"，没有任何
        #: 参数或几何能反算出来。停机 / 换频 / 不再被照射都会把它清掉
        #: （见 :meth:`_pull_off_forget`）。
        self._pull_off: dict[int, int] = {}
        #: 本帧的仿真时刻（微秒）。拖引行程要用它，而 ``_sweep`` 拿到的
        #: ``engine.now`` 才是**真实经过的时间**（允许事件被推迟）。
        self._now_us = float(mount.now)
        #: 本拍算出的假目标**原始条数**（没被槽位 / 距离截断的那个数）。
        #: 洪泛判据用它，所以它必须是"干扰机想造多少条"，不是"屏幕上真有几条"。
        self.false_targets_seen = 0
        #: 本拍**实际铺上屏幕**的幽灵条数（= ``len(_phantoms)``，诊断用）。
        self.phantoms_seen = 0
        #: 累计有多少条真实检测被假目标**淹没**（诊断用：它只在有假目标干扰
        #: 且洪泛时增长）。◆ 与"Pd 低所以没命中"是两回事，所以分开计数。
        self.jamming_blocked = 0
        #: 本拍铺出去的幽灵 ID 集合。**真状态**：它们不对应任何实体，所以
        #: 既不在 ``_hits`` 里、也不会被 :meth:`_prune` 碰——下一拍要自己清掉，
        #: 否则航迹表里的幽灵只会越积越多。
        self._phantoms: set[int] = set()
        #: 最近那条幽灵的斜距（米）。几何优先权要用它：比它还近的真目标
        #: **无条件放行**（AFSIM ``false_target_screener`` 原文）。没有幽灵时
        #: 是 ``inf`` ⇒ 那条判据恒真、等于不判。
        self._nearest_phantom_m = float("inf")

        mount.every(self.spec["beam_step"], self._sweep, PRIORITY_SENSOR)

    def _require_beam_width(self) -> None:
        """固定指向（``fov == 0``）不许配零宽波束：那台雷达什么都照不到。

        零宽波束下"照到"退化成一条没有面积的射线，命中要靠目标中心恰好
        落在相位上。症状是**一条航迹都没有**，而参数表看起来一个都没错
        （跟弹的"推重比 < 1 跑出假航迹"是同一类：与其跑出一个看着像
        "目标不在那儿"的结果，不如当场报错）。
        """
        raise ConfigurationError(
            f"{self.type_name or type(self).__name__}: fov = 0 表示固定指向，"
            "但 beam_width 也是 0——零宽波束什么都照不到（命中要目标中心恰好"
            "落在相位那一条射线上）。固定指向请给出 beam_width：一次照射的"
            "瞬时张角，例如 2 deg"
        )

    # -- 第 6 步的前置：脉冲数 ---------------------------------------------

    def _resolve_pulses(self) -> int:
        """非相干积累的脉冲数 N。

        规则只有一条：**显式给的优先，否则按时长算，算不出来就是 1**。

        * ``pulses_integrated >= 1`` ⇒ 用它；
        * 否则若给了 ``pulse_repetition_frequency`` 且 ``beam_width > 0``：
          ``N = max(1, round(dwell × PRF))``，``dwell = 波束宽 ÷ 角速度``；
        * 否则 1。

        ★ 固定指向是唯一的例外：天线不转，目标一直在波束里，驻留时间等于
        **评估间隔**（``beam_step``），不经过"波束宽 ÷ 角速度"。
        """
        explicit = int(self.spec["pulses_integrated"])
        if explicit >= 1:
            return explicit
        prf = float(self.spec["pulse_repetition_frequency"])
        if prf <= 0.0 or self.beam_deg <= 0.0:
            return 1
        if self.fov_deg <= 0.0:
            dwell = float(self.spec["beam_step"]) / 1.0e6
        else:
            scan_rate = 360.0 / (self.period_us / 1.0e6)
            dwell = eq.dwell_time(self.beam_deg, scan_rate)
        return eq.pulses_in_dwell(dwell, prf)

    # -- 第 5~7 步的预算 ---------------------------------------------------

    def _resolve_budget(self) -> None:
        """把参数折成"算一次单脉冲 SNR 需要的那几个数"。

        半给不行：任何一个发射机参数被写下来就必须把该路给全，缺一项当场
        报错（见模块头）。

        ★ **走哪条路由 ``peak_power`` 决定**（v0.13.30 起重划）：写了功率
        就是"我要走雷达方程"（逐参法），没写就是"我手上只有一条距离 ↔ Pd
        的标定曲线"（标定法）。判据只看**只有在方程里才有意义的那些量**：

        ==================  ============================================
        决定走哪条路的       ``peak_power``（发射机）、``system_loss_db`` /
        参数                ``noise_figure_db`` / ``receive_line_loss_db`` /
                            ``antenna_noise_temperature``（噪声温度支）
        两条路都要用的      ``frequency``（波长 / ``F_BW``）、``bandwidth``、
                            ``antenna_gain_db`` / ``beamwidth_h·v``
        ==================  ============================================

        右边那一栏**不再**参与判断，因为它们在标定法里也有意义：标定法要算
        干扰就得知道自己的调谐带（``F_BW``）、波长、天线增益——这些是**天线与
        频段**的属性，不是"你写没写雷达方程"的表态。在此之前"给一部标定法
        雷达补上频率"会被判成"写了发射机参数却缺 ``peak_power``"而报错，
        于是标定法雷达在结构上**根本接不了干扰**（§5.14）。
        """
        power = float(self.spec["peak_power"])
        frequency = float(self.spec["frequency"])
        gain_db = float(self.spec["antenna_gain_db"])
        bw_h = float(self.spec["beamwidth_h"])
        bw_v = float(self.spec["beamwidth_v"])
        loss_db = float(self.spec["system_loss_db"])
        nf_db = float(self.spec["noise_figure_db"])
        line_db = float(self.spec["receive_line_loss_db"])
        tx_loss_db = float(self.spec["transmit_loss_db"])
        rx_loss_db = float(self.spec["receive_loss_db"])
        ant_temp = float(self.spec["antenna_noise_temperature"])
        # ★ 接收机那三个参数也算"写了发射机参数"：只写 ``noise_figure_db`` 而
        # 不写功率时，标定法**根本不用噪声**，于是"我明明填了噪声系数"被静默
        # 忽略而数字看着仍然合理——正是模块头要消灭的那类静默退化。
        requested = (
            power > 0.0
            or loss_db > 0.0
            or nf_db > 0.0
            or line_db > 0.0
            or ant_temp != DEFAULT_ANTENNA_NOISE_TEMPERATURE
        )

        #: 工作频段（Hz）。**两条路都要**：逐参法用它取波长与噪声带宽，
        #: 标定法用它算干扰的带内匹配与波长。0 = 未给定 ⇒ 接不了干扰
        #: （见 :meth:`_require_jam_budget`），**不影响**任何无干扰的数字。
        self.frequency_hz = frequency
        #: 接收机调谐带宽（Hz）。逐参法由 ``bandwidth`` 或 ``1/τ`` 定，
        #: 标定法直接取参数（0 = 未给定）。
        self.bandwidth_hz = float(self.spec["bandwidth"])

        if not requested:
            self.range_model = "标定法"
            self.power_w = 0.0
            # 天线增益在标定法里**只有干扰用得上**（回波功率已经折进标定曲线
            # 了）。没有波束宽度就取 0——0 不是"各向同性"，是"不知道"，而
            # 真需要它的那条路会当场报错（_require_jam_budget），不会拿一个
            # 假增益算出一个看着合理的 J。这个口径与 v0.13.29 的 1.0（一个
            # 谁都没读过的占位值）只差"谁读它"：那时没有读者。
            self.gain = (
                eq.peak_gain(bw_h, bw_v) if (bw_h > 0.0 and bw_v > 0.0) else 0.0
            )
            self.wavelength_m = eq.wavelength(frequency) if frequency > 0.0 else 0.0
            #: ★ 标定法的 N **没有第二个来源**：要么显式给 ``noise_power``
            #: （AFSIM 的 RF.6 第 1 条"给了它就用它"），要么就是 0——0 的意思
            #: 不是"噪声为零"，是"这个数不知道"。
            self.noise_w = float(self.spec["noise_power"])
            self.loss = 1.0
            #: 标定法**不另记**干扰的路径损耗：标定曲线里已经有一份总损耗，
            #: 再给一个 ``system_loss_db`` 就会走逐参法（那才是它的归宿）。
            #: ★ 但 **v0.13.36 起** ``receive_loss_db`` 是**独立**可给的：它是
            #: "接收支路内损"，与标定曲线里那份总损耗**不是同一个量**（那个
            #: 是双程、含发射），所以标定法雷达也能（且只能）用它接干扰的损耗。
            self.jamming_path_loss = eq.linear(rx_loss_db)
            self._rx_loss_given = rx_loss_db > 0.0
            self._tx_loss_given = tx_loss_db > 0.0
            self.snr_at_reference = eq.snr_for_pd(
                self.required_pd,
                self.pulses,
                self.pfa,
                nonfluctuating=(self.swerling_case == 0),
            )
            return

        name = self.type_name or type(self).__name__
        if float(self.spec["noise_power"]) > 0.0:
            raise ConfigurationError(
                f"{name}: 逐参法的噪声功率由『天线噪声温度 + 馈线损耗 + 噪声"
                "系数』这条链算出来（Blake 级联式），不能再给一个 noise_power"
                "——同一个量有两个出处，改哪一个都只生效一半。标定法才需要它"
            )
        missing = []
        if power <= 0.0:
            missing.append("peak_power")
        if frequency <= 0.0:
            missing.append("frequency")
        if gain_db <= 0.0 and not (bw_h > 0.0 and bw_v > 0.0):
            missing.append("antenna_gain_db（或同时给 beamwidth_h 与 beamwidth_v）")
        if missing:
            raise ConfigurationError(
                f"{name}: 写了发射机参数就是要走逐参法（雷达方程），"
                f"但缺 {'、'.join(missing)}——补全，或者把已写的那些删掉改用标定法。"
                "半给会静默退回标定法，于是'我明明填了功率'却一点没生效"
            )
        # ★ v0.13.36（§9-54）：``transmit_loss_db`` **不进干扰链路**，只写它
        # 而没写 ``system_loss_db`` ⇒ 它压根不生效（回波走 system_loss_db）。
        # 报错而不是静默：症状是"我写了 2 dB 发射内损，J 一点没变"。
        if tx_loss_db > 0.0 and loss_db <= 0.0:
            raise ConfigurationError(
                f"{name}: 只写了 transmit_loss_db 而没写 system_loss_db。"
                "干扰链路**只扣接收支路内损**（receive_loss_db）——发射内损不进"
                "干扰（信号不是这部雷达发出去的，§9-54）；而回波走的又是"
                "system_loss_db。所以单独一个 transmit_loss_db 谁都影响不到。"
                "要分开记收发内损，请同时写 system_loss_db（= 发射 + 接收，供回波）"
                "与 receive_loss_db（供干扰）"
            )

        self.range_model = "逐参法"
        self.power_w = power
        self.gain = (
            eq.linear(gain_db) if gain_db > 0.0 else eq.peak_gain(bw_h, bw_v)
        )
        self.wavelength_m = eq.wavelength(frequency)

        bandwidth = self.bandwidth_hz
        if bandwidth <= 0.0:
            pulse_width_us = float(self.spec["pulse_width"])
            if pulse_width_us <= 0.0:
                raise ConfigurationError(
                    f"{name}: 逐参法需要带宽：给 bandwidth，"
                    "或给 pulse_width 由 B = 1/τ 定（两者都没有就无法算噪声）"
                )
            bandwidth = 1.0 / (pulse_width_us / 1.0e6)
        self.bandwidth_hz = bandwidth
        # 系统噪声温度：Blake 级联式（天线 + 馈线 + 接收机三段）。
        # ★ 馈线损耗**只在这里**出现这一次——它不进信号路径（AFSIM 口径）。
        temperature_k = eq.cascade_noise_temperature(
            ant_temp, eq.linear(line_db), eq.linear(nf_db)
        )
        self.noise_w = eq.thermal_noise(bandwidth, temperature_k)
        #: 信号路径的双程损耗（**不含** ``receive_line_loss_db``）。
        self.loss = eq.linear(loss_db)
        #: ★ 干扰的单程路径损耗（线性），**只扣受害方接收支路内损**
        #: （v0.13.36，§9-54；AFSIM 的 ``receive_loss``）。旧口径是
        #: ``one_way_loss(loss_db) = √L``，比 AFSIM 高 +2.4999 dB。
        self.jamming_path_loss = eq.linear(
            eq.jamming_path_loss_db(loss_db, rx_loss_db)
        )
        #: ``receive_loss_db`` 显式给了（>0）就**不再**用 ``system_loss_db``
        #: 的劈半口径——两者同时给会当场报错（同一个量两个出处）。见下面校验。
        self._rx_loss_given = rx_loss_db > 0.0
        self._tx_loss_given = tx_loss_db > 0.0
        self.snr_at_reference = 0.0

    # -- 第 7b 步：量测误差的两条路 ---------------------------------------

    def _resolve_errors(self) -> None:
        """把量测误差折成"每次命中加多大的 σ"。

        **两条路互斥**，对应 AFSIM 的两个开关：

        ==============  ==================================================
        路              参数 / σ 从哪来
        ==============  ==================================================
        直接给          ``azimuth_error_sigma`` / ``range_error_sigma``，
                        常数，**与距离无关**
        由 SNR 反算      ``compute_measurement_errors true`` ⇒
                        ``θ_3dB/(k·√(2·SNR))``、``c·τ/(2·√(2·SNR))``
        ==============  ==================================================

        ★ **同时给会当场报错**，不静默取一个。AFSIM 里 ``compute_measurement_
        errors`` 打开时是它优先、sigma 被忽略，但那会让"我明明填了 sigma"
        变成一句废话——正是本模块头要消灭的那类静默退化。

        ★ 由 SNR 反算**需要** ``beamwidth_h``（角度）与 ``pulse_width``
        （距离）。缺一个就报错，**不退回零误差**：退回的症状是"开了开关却
        一点误差都没有"，而参数表看起来完全正常。
        """
        self.error_azimuth_deg = float(self.spec["azimuth_error_sigma"])
        self.error_range_m = float(self.spec["range_error_sigma"])
        self.errors_from_snr = bool(self.spec["compute_measurement_errors"])

        if not self.errors_from_snr:
            return

        name = self.type_name or type(self).__name__
        if self.error_azimuth_deg > 0.0 or self.error_range_m > 0.0:
            raise ConfigurationError(
                f"{name}: compute_measurement_errors 打开时 σ 由 SNR 反算，"
                "azimuth_error_sigma / range_error_sigma 必须留空——两条路同时"
                "给，无从判断该信哪一个（sigma 会被静默忽略）"
            )
        missing = []
        if float(self.spec["beamwidth_h"]) <= 0.0:
            missing.append("beamwidth_h（角度误差 ∝ 波束宽度）")
        if float(self.spec["pulse_width"]) <= 0.0:
            missing.append("pulse_width（距离误差 ∝ c·τ/2）")
        if missing:
            raise ConfigurationError(
                f"{name}: compute_measurement_errors 需要波形参数，缺 "
                f"{'、'.join(missing)}——缺了就退化成「零误差」，症状是"
                "（开了开关却一点误差都没有）而参数表看着正常"
            )

    def _measurement_sigmas(
        self, slant_m: float, rcs_m2: float, *, jammer_noise_w: float = 0.0
    ) -> tuple[float, float]:
        """本次量测的两个 σ：``(方位误差 度, 距离误差 米)``。

        两条路（见 :meth:`_resolve_errors`）在这里合流成**同一对返回值**，
        所以下游不必知道误差是从哪一条来的。

        ★ 送进公式的是**单脉冲** SNR —— 与 Pd **同一个入口**。若改用积累后的
        总 SNR，σ 与 Pd 就会各自吃一次积累增益，两者不再是同源的，
        "Pd 大 ⇒ 航迹准"这句话也就失去了物理依据。

        ★ 干扰**也进这里**（``jammer_noise_w``）：它压的是同一个 SNR。
        于是"被压制 ⇒ 航迹更糊"不是一句额外的话，而是同一个数自然地传到
        了 σ 上——与"Pd 低 ⇒ 不准"走的是同一条路（§5.12.9）。
        """
        if not self.errors_from_snr:
            return self.error_azimuth_deg, self.error_range_m
        snr = self.single_pulse_snr(
            slant_m, rcs_m2, jammer_noise_w=jammer_noise_w
        )
        return (
            eq.angle_measurement_sigma(float(self.spec["beamwidth_h"]), snr),
            eq.range_measurement_sigma(float(self.spec["pulse_width"]) / 1.0e6, snr),
        )

    def _measure(
        self,
        bearing_deg: float,
        horizontal_m: float,
        slant_m: float,
        dz: float,
        rcs_m2: float,
        *,
        jammer_noise_w: float = 0.0,
        range_bias_m: float = 0.0,
        azimuth_bias_deg: float = 0.0,
    ) -> tuple[float, float]:
        """在**极坐标**上加量测误差，返回 ``(带误差的方位, 带误差的水平距离)``。

        ★ 误差加在**极坐标**上，不是在 x/y 上，因为真雷达就是这么测的：方位
        靠天线、距离靠时延，两套误差机制完全不同（一个 ∝ 波束宽度、一个 ∝
        脉冲宽度），落到位置上是一片"**切向大、径向小**"的椭圆。若在 x/y 上
        各加一个同方差噪声，这个各向异性会被直接抹掉。

        ★ 距离误差加在**斜距**上（雷达测的是斜距），再由真实高差反算水平
        距离。高度本身不注入误差——见参数表里那条说明。

        ★ **偏置（欺骗）先加、σ（噪声）后加**，顺序不能反：偏置改变的是量测
        的**中心**，σ 是围绕那个中心的散布。反过来（先撒再挪）会让"偏置为零
        时"的两次抽样不是同一个分布——那个差别很小，但它是错的。

        ★ 偏置加在**未经噪声**的真实量测上，且**不受 σ 是否有限影响**：
        欺骗生效与否只看 ``J/S``（第 6 步已判过），与本次命中的采样无关。
        （σ 发散时下面会提前返回——那时位置量测没有精度可言，但**中心还是
        被挪走的**，所以偏置要在那个返回之前加。）
        """
        # ★ 偏置先落在**斜距**上（雷达测的就是斜距），再由真实高差反算水平
        # 距离；σ 随后在这个**已经被挪走的中心**周围撒。两步必须按这个顺序
        # 作用在同一个量上——分别在两个变量上各做一次，偏置就会被 σ 那一支
        # 悄悄覆盖掉（σ > 0 时偏置消失、σ = 0 时偏置还在，正是最难查的那种）。
        biased_slant = max(0.0, slant_m + range_bias_m)
        bearing_deg = (bearing_deg + azimuth_bias_deg) % 360.0
        horizontal_m = sqrt(max(0.0, biased_slant * biased_slant - dz * dz))

        sigma_bearing, sigma_range = self._measurement_sigmas(
            slant_m, rcs_m2, jammer_noise_w=jammer_noise_w
        )
        # 非有限的 σ 意味着"这次量测根本没有精度可言"（SNR 为 0 或发散）。
        # 此时**不加噪**：``gauss(0, inf)`` 会给 nan，而一个 nan 位置会沿着
        # 航迹表一路带下去，比"没加误差"危险得多。**但偏置照旧生效**——它是
        # 中心被挪走，与这次采样准不准是两件事。
        if not (isfinite(sigma_bearing) and isfinite(sigma_range)):
            return bearing_deg, horizontal_m

        measured_bearing = bearing_deg
        if sigma_bearing > 0.0:
            measured_bearing = (
                bearing_deg + self._rng.gauss(0.0, sigma_bearing)
            ) % 360.0

        measured_horizontal = horizontal_m
        if sigma_range > 0.0:
            measured_slant = biased_slant + self._rng.gauss(0.0, sigma_range)
            # 噪声可能把斜距推到小于高差 ⇒ 水平分量取 0（不可能为负）
            measured_horizontal = sqrt(
                max(0.0, measured_slant * measured_slant - dz * dz)
            )
        return measured_bearing, measured_horizontal

    # -- 第 6 步的干扰项：S/(N+J)（§5.14）----------------------------------

    def _require_jam_budget(self) -> None:
        """标定法要参与干扰求值时，把缺的那几项**一次报全**。

        这笔账与逐参法的"半给不行"是同一条规矩，但**报错时机不同**：干扰是
        **运行期**才知道有没有的（干扰机可以中途开机 / 被击毁），装配期无从
        判断。所以不在 ``initialize`` 里查，而是在**第一次真有干扰要算**的
        时候查——那时缺项**必然**算出一个假数（0 或者无穷），与其静默地让它
        "压不住 / 压死了"，不如当场停下来说清楚要补什么。

        逐参法不走这里：它的 N、λ、B、G 在 ``_resolve_budget`` 里已经算好了。
        """
        if self.range_model != "标定法":
            return
        name = self.type_name or type(self).__name__
        missing = []
        if self.noise_w <= 0.0:
            missing.append(
                "noise_power（标定法把噪声折进了标定曲线，手上没有绝对的 N，"
                "而 J/S = J/(SNR·N) 要它）"
            )
        if self.gain <= 0.0:
            missing.append(
                "beamwidth_h 与 beamwidth_v（干扰要按**天线增益**算，而标定法"
                "没有发射机参数可反推，只能给波束宽度由 4π/(θ_H·θ_V) 换）"
            )
        if self.frequency_hz <= 0.0:
            missing.append("frequency（决定波长与 F_BW）")
        if self.bandwidth_hz <= 0.0:
            missing.append("bandwidth（F_BW 的分母）")
        if missing:
            raise ConfigurationError(
                f"{name}: 标定法雷达被干扰，但缺 {'、'.join(missing)}——"
                "补全它们才能把干扰并进噪声。在此之前它算不出 J，而『算不出』"
                "若被当成 0，症状是『挂了干扰机却一点不受影响』"
            )

    def _require_false_target_budget(self) -> None:
        """假目标那一支要用的**波形量**，缺哪一项就一次报全。

        与 :meth:`_require_jam_budget` 同一条规矩（有没有干扰是**运行期**才
        知道的，所以不在 ``initialize`` 里查），但**仍然分成两个函数**：这边
        要的是"波形量 + 天线增益"，那边要的是"噪声 + 天线增益"。两边在
        λ / B / G 上**确实有交集**（v0.13.31 给假目标加了到达功率门限之后），
        但**交集不是合并的理由**：合成一个的话，"只挂了假目标卡"的想定会被
        要求补 ``noise_power``——而那台雷达的压制链根本不需要它。

        ★ 为什么必须报错、不许取默认值：条数公式
        ``(PRI/PW)·(ScanTime/PRI)·N_int·density`` 在 ``PW`` 或 ``PRI`` 为 0 时
        没有意义（除零 / 无穷），而它算出来的东西会**直接变成航迹表里的行**。
        取一个"看着合理"的默认脉宽，症状就是屏幕上凭空多出一片幽灵，而参数表
        上每一项都正常。
        """
        name = self.type_name or type(self).__name__
        missing = []
        if float(self.spec["pulse_width"]) <= 0.0:
            missing.append(
                "pulse_width（距离分辨单元 ΔR = c·τ/2 要用它，幽灵的间距就是它）"
            )
        if float(self.spec["pulse_repetition_frequency"]) <= 0.0:
            missing.append(
                "pulse_repetition_frequency（条数公式里的 PRI 与最大不模糊距离 "
                "c·PRI/2 都要它）"
            )
        # ★ v0.13.31 起这三项也归这一支：假目标加了**到达功率门限**，而算
        # "到达多少瓦"要天线增益、波长与带宽（F_BW）。它们是**压制那一支本来
        # 就要的同一批数**，所以清单在这里只是补齐。
        if self.frequency_hz <= 0.0:
            missing.append("frequency（决定波长与 F_BW）")
        if self.bandwidth_hz <= 0.0:
            missing.append("bandwidth（F_BW 的分母）")
        if self.gain <= 0.0:
            missing.append(
                "beamwidth_h 与 beamwidth_v（假目标的**到达功率**要按天线增益算，"
                "而标定法没有发射机参数可反推，只能给波束宽度由 4π/(θ_H·θ_V) 换）"
            )
        if missing:
            raise ConfigurationError(
                f"{name}: 有干扰机在放假目标，但缺 {'、'.join(missing)}——"
                "补全它们才算得出条数与幽灵的距离；在补齐之前那两个量没有意义"
                "（除零 / 无穷），而它们会**直接变成航迹表里的行**"
            )

    def received_jamming_w(self, beam_bearing_deg: float) -> float:
        """**只要功率**时的简写：``jamming_effect(...).power_w``。

        欺骗那一支不参与（没给单脉冲 SNR，``J/S`` 就无从谈起）。要完整的
        第 6 步后果——功率**与**量测偏置——请调 :meth:`jamming_effect`。
        """
        return self.jamming_effect(beam_bearing_deg).power_w

    def jamming_effect(
        self,
        beam_bearing_deg: float,
        signal_to_noise: float = 0.0,
        *,
        me: tuple[float, float, float] | None = None,
    ) -> JammingEffect:
        """波束正指着 ``beam_bearing_deg`` 时，干扰造成的**全部**后果。

        ``me`` 是本机（受害方）的三维位置，判"干扰机那边山挡不挡得住"要用。
        ``None`` 时自己从视图取——留这个默认值是为了让"只想拿个功率"的调用方
        不必先自己查一次位置；视图本来就是"我在哪"的唯一出处，这里不会引入
        第二个来源。

        求值时机就是 AFSIM 的原话——"at the time a radar detection ...
        occurs"：天线正指着某个目标，别处的干扰机从主瓣或副瓣漏进来。
        所以调用处传的是**目标的方位**（那一刻天线对着它）与**未受干扰的
        单脉冲 SNR**（判"骗得成骗不成"要用它）。

        三道工序，顺序就是 AFSIM 那一句 "sum the power for every possible
        jammer if there is **in-band** power"：

        1. **谁在压我** —— 门面向名册要候选（谁在册、在哪、进没进主瓣）；
        2. **进不进得来** —— 逐台过**四道**：``F_BW``（带内比例，``= 0`` 的那台
           **跳过**——跳频干扰就是靠它生效的）、**地形遮蔽 / 地平线**（v0.13.31，
           山那面的进不来）、接收增益（主瓣还是副瓣，差
           :data:`~milsim.services.ew.DEFAULT_SIDELOBE_LEVEL_DB`）、
           ``F_POL``（**两端极化类型查表**，v0.13.31）；
        3. **求和** —— 功率相加；带欺骗卡且过了 ``J/S`` 门限的那些台，把
           偏置相加；带假目标卡**且到达功率过了门限**的那些台，把"要造多少条
           幽灵"列出来（v0.13.31）。

        ★ **第三个后果在这里只是"列出来"**：假目标的条数公式里没有目标
        （全是波形量）、方位取的是干扰机自己的方位，所以它不随"这一刻波束
        指着谁"变。真正铺幽灵的动作是**每拍一次**的
        :meth:`_update_false_targets`——那个函数才判"本拍扫到它没有"。
        这里之所以还是要过一道：**带内**（``F_BW > 0``）是"接收机收不收得到"
        的前提，跳频抗干扰因此在挡掉压制的同时也挡掉了假目标。

        ★ **求和顺序按实体 ID**（门面 ``specs()`` 已排好）：浮点加法不满足
        结合律，换个顺序末位就可能变，而"压制效果可复现"比末位更值钱。

        ★ 多台同时欺骗时**偏置相加**。AFSIM 只规定了功率求和，对偏置没说。
        取"相加"而不是"取最强"的理由：相加至少保证"两台同向拉得更多"是单调
        的，而"取最强"会让两台反向的拉锯变成随机胜负（末位差一点就换赢家）。

        ★ 没挂干扰机时**连位置都不查**：``is_empty`` 直接就返回空效果。于是
        "没挂干扰机的想定逐字不变"不是靠一个 ``if`` 挡出来的，而是"没有这个
        东西"的自然结果。

        ★ **判干扰机那一侧的遮蔽 / 地平线**（v0.13.31 起，§9-48 结案）：山那面
        的干扰机**进不来**，被挡掉的台次累加进 ``jamming_masked``。用的是**受害方
        自己那条第 4 步的同一个调用**（:meth:`_has_line_of_sight`），**不另写一份
        几何**——两份几何迟早分叉，而症状是"探测看得见的目标，干扰却过得来"，
        这种不一致在参数表上完全看不出来。判据的边界也照抄那条：没有地形数据 /
        没给 ``antenna_height`` ⇒ **不判**（缺数据 ≠ 被遮挡）。
        ★ 干扰机**自己的天线架高**从 v0.13.33 起建模（``antenna_height`` ⇒
        ``Threat.antenna_height_m``，见 §5.14.11 ①）：判遮蔽时把那个值作为对面
        的架高喂进去，而不是拿受害方自己的架高顶替。没给（0）⇒ 两端都用本雷达
        架高，旧口径逐字不变。
        ★ 干扰机**天线朝向**从 v0.13.34 起建模（``aim_at`` / ``beam_width_deg``，
        见 §5.14.11 ④）：方向图因子折在**它的发射增益**上，不在这一段的遮蔽判据里。

        ★ **自己的身份取 ``self.view.entity_id``**，不是 ``self.entity_id``：
        "我是谁"在这条链上只有一个出处——视图（``position_of`` 与
        ``add_contact`` 用的都是它）。组件属性是从宿主实体推出来的，装配之外
        （比如单元测试里手工 ``initialize``）可能是 ``-1``，那时门面查不到
        "我在哪"，于是**安静地返回空效果**——症状是"干扰机挂上了却一点不受
        影响"，正是最像"干扰没生效"的那一类静默错（实测踩过一次）。
        """
        jam = self._jam
        if jam is None:
            return JammingEffect()
        if jam.is_empty:
            # ★ 名册空了**也要走一次清表**：干扰机"关机"正是"这一拍不再出现"
            # 的极端情形。漏掉它，行程表会**穿过停机期继续计时**——再开机的
            # 第一帧就报出一个已经拉满的偏置（实测：静态偏置 100 m、速率
            # 50 m/s 的卡，停机 1 s 后再开机直接给 400 m，而这 400 在参数表
            # 与日志里都找不到出处）。没有干扰机时这一句就是"取个长度再
            # 返回"，代价可以忽略，而它换来的是"清表只有一条路"。
            self._pull_off_forget(set())
            return JammingEffect()
        threats = jam.threats(
            self.view.entity_id,
            beam_bearing_deg=beam_bearing_deg,
            main_lobe_deg=self.beam_deg,
        )
        if not threats:
            self._pull_off_forget(set())
            return JammingEffect()
        self._require_jam_budget()

        if me is None:
            me = self.view.my_position()
        sidelobe = eq.linear(DEFAULT_SIDELOBE_LEVEL_DB)
        total = 0.0
        peak_js = 0.0
        in_band = 0
        pullers = 0
        bias_range = 0.0
        bias_azimuth = 0.0
        phantoms: list[FalseTargetSource] = []
        for threat in threats:
            f_bw = eq.bandwidth_overlap_ratio(
                threat.frequency_hz,
                threat.bandwidth_hz,
                self.frequency_hz,
                self.bandwidth_hz,
            )
            if f_bw <= 0.0:
                continue                    # 这台在带外：一分都进不来
            # ---- ② 山那面挡不挡得住（v0.13.31）----
            # ★ 与受害方自己那条第 4 步**共用同一个调用**。判据的边界也一致：
            # 没有地形数据 / 没给架高 ⇒ 不判。挡掉的那台**连假目标也一起挡掉**
            # （它根本没照到对方），所以这一道必须在假目标之前。
            if me is not None and not self._has_line_of_sight(
                me, threat.position, threat.antenna_height_m
            ):
                self.jamming_masked += 1
                continue
            rx_gain = self.gain if threat.in_main_lobe else self.gain * sidelobe
            # ---- ⑤ 干扰机**自己的**天线方向图（v0.13.34，§5.14.11 ④）----
            # ★ 它与 ``rx_gain`` 是**两台设备上两个不同的天线**：``rx_gain``
            # 是"我的接收天线在它那个方位上取主瓣还是副瓣"，这里折的是"**它的**
            # 发射天线对**我**这个方向给多少增益"。写在发射那个参数上而不是并进
            # ``rx_gain``：两者来源不同，合并后"到底谁的方向图在起作用"就查不出来。
            # ★ ``threat.antenna_off_deg is None`` ⇒ 这台没启用置向 ⇒ 因子 1.0
            # （旧想定/没写 ``aim_at`` 的想定逐位不变）。
            # ★ ``beam_width_deg == 0`` ⇒ 因子 1.0（各向同性档），同样逐位不变。
            tx_gain = threat.gain
            if threat.antenna_off_deg is not None:
                tx_gain *= eq.antenna_pattern_factor(
                    threat.antenna_off_deg, threat.beam_width_deg, sidelobe
                )
            # 两边的损耗**相乘**：干扰机内损是它自己的，路径损耗是受害方的
            # **接收支路**那一份（v0.13.36 口径修正，§9-54 —— AFSIM 的
            # ``receive_loss``，**不是**旧口径的 ``√L``）。
            loss = threat.internal_loss * self.jamming_path_loss
            # ---- ③ 极化：两端**类型**查表（v0.13.31）----
            # ★ 受害方自己算：要用"我的主极化"与"它的类型"两个数。显式覆盖
            # （``polarization`` ≥ 0）优先，语义同 AFSIM 的 ``polarization_effect``。
            f_pol = eq.polarization_effect(
                self.polarization_type,
                threat.polarization_type,
                threat.polarization,
            )
            power = eq.jamming_power(
                threat.radiated_power_w,
                tx_gain,
                rx_gain,
                self.wavelength_m,
                threat.slant_m,
                loss=loss,
                bandwidth_overlap=f_bw,
                polarization=f_pol,
            )
            # ---- ④ 假目标（第三个后果，v0.13.31 起带功率门限）----
            # ★ 它的条数要用**我自己的**波形算，所以在这里算而不是门面里算
            # （§5.14 的分工：门面给几何，受害方算物理）。
            # ★ ``power`` 用**主瓣 / 副瓣**那个接收增益，与压制那一支同口径：
            # 假目标是在"天线正照到干扰机"的那一刻注入的，而这个函数正是
            # "波束指着某处"时求的值。★ 门限是**到达功率**，不是 J/S——每拍
            # 一次的铺幽灵那条路（``_update_false_targets``）没有单脉冲 SNR，
            # 用 J/S 的话两条路会各判各的（详见那条的说明）。
            card_ft = threat.false_targets
            if card_ft is not None and power >= self.false_target_min_power_w:
                count = self._false_target_count(card_ft)
                if count > 0:
                    phantoms.append(
                        FalseTargetSource(
                            entity_id=threat.entity_id,
                            bearing_deg=threat.bearing_deg,
                            nearest_m=threat.slant_m + self._range_cell_m(),
                            count=count,
                        )
                    )
            if power <= 0.0:
                continue
            in_band += 1
            total += power

            if signal_to_noise <= 0.0:
                continue                # 没给 SNR：骗得成骗不成判不了
            j_to_s = eq.jamming_to_signal(power, self.noise_w, signal_to_noise)
            if j_to_s > peak_js:
                peak_js = j_to_s
            card = threat.deception
            if card is None:
                continue
            # ---- ① 欺骗走**自己那条链**的功率（v0.13.31）----
            # ★ ``card.power_w <= 0`` ⇒ 两链共用压制那一份 ⇒ 下面两个变量
            # **逐位等于**上面那两个，于是旧想定的判定与数字分毫不差。
            # 这不是"近似保持兼容"，是"同一个数走同一条路"。
            # ★ 欺骗的功率**不并进 ``total``**：它是一条信号路的功率，不是噪声。
            # 这正是 v0.13.31 之前做不到的那件事（§9-50 结案）：那时"骗得成"
            # 的功率按定义已经把 Pd 压下来了。
            if card.power_w <= 0.0:
                d_js = j_to_s
            else:
                d_js = eq.jamming_to_signal(
                    eq.jamming_power(
                        card.power_w,
                        tx_gain,
                        rx_gain,
                        self.wavelength_m,
                        threat.slant_m,
                        loss=loss,
                        bandwidth_overlap=f_bw,
                        polarization=f_pol,
                    ),
                    self.noise_w,
                    signal_to_noise,
                )
            if d_js < eq.linear(card.required_j_to_s_db):
                continue
            pullers += 1
            bias_range += self._pull_off_m(card, threat.entity_id)
            bias_azimuth += card.azimuth_bias_deg

        self._pull_off_forget({t.entity_id for t in threats})
        return JammingEffect(
            power_w=total,
            peak_j_to_s=peak_js,
            in_band=in_band,
            pullers=pullers,
            range_bias_m=bias_range,
            azimuth_bias_deg=bias_azimuth,
            false_targets=tuple(phantoms),
        )

    # -- 拖引（RGPO）的行程表 ---------------------------------------------

    def _pull_off_m(self, card, jammer_id: int) -> float:
        """这台干扰机此刻把距离拉走了多少米（静态偏置 + 拖引行程）。

        ``offset(t) = 静态偏置 + sign(速率) · min(|速率| · t, 封顶)``，
        ``t`` 是从**它开始有效地压我**那一刻算起的秒数。

        ★ 行程表按**干扰机**记（``_pull_off``），不按目标记：距离波门被拉走
        是"跟踪环路被带偏"的过程，而我们这一版没有环路，只有"这台机器盯着
        我拉了多久"。同一个干扰机换一个目标来照，行程是接着走的。

        ★ ``recycle`` = **拉满就松手重来**：行程归零，从这一刻重新开始拉。
        这是"先把波门拖走、再突然松手让环路扑空"的那个松手动作。
        """
        rate = card.walkoff_rate_mps
        if rate == 0.0:
            return card.range_bias_m
        started = self._pull_off.get(jammer_id)
        if started is None:
            started = self._now_us
            self._pull_off[jammer_id] = started
        # ``max(0, ...)``：时钟不允许倒退，但 ``_sweep`` 明确容忍"同刻重复触发"
        # （``delta_deg <= 0`` 那一支）。真出现倒退时行程**不能变负**——那会让
        # "被拖走多远"缩回去，是一个没有任何物理含义的动作。
        elapsed = max(0.0, (self._now_us - started) / 1.0e6)
        walked = abs(rate) * elapsed
        cap = card.holdout_m
        if cap > 0.0 and walked >= cap:
            if card.recycle:
                self._pull_off[jammer_id] = self._now_us      # 松手，从头再拉
                walked = 0.0
            else:
                walked = cap                                  # 拉满就停在那儿
        return card.range_bias_m + (rate / abs(rate)) * walked

    def _pull_off_forget(self, still_present: set[int]) -> None:
        """把**这一帧不再出现**的干扰机的行程表清掉。

        不清的后果是"干扰机停了一会儿又开机，行程从上次的地方接着走"——
        它会在"刚开机"的第一帧就报出一个已经拉了几百米的偏置，而参数表
        与日志都看不出这是从哪儿来的。停机就该从头开始。
        """
        if len(self._pull_off) <= len(still_present):
            return
        for jammer_id in [k for k in self._pull_off if k not in still_present]:
            del self._pull_off[jammer_id]

    # -- 第 7c 步：假目标（§5.14）------------------------------------------

    def _range_cell_m(self) -> float:
        """距离**分辨单元** ``ΔR = c·τ/2``（米）——幽灵之间的最小间距。

        出处就在条数公式里：AFSIM 那一项 ``(PRI/PW)`` 的意思正是"一个 PRI 里
        有多少个分辨单元"。所以幽灵**一个单元一条**、从干扰机之外一个单元起铺，
        而"一个不模糊距离里最多几条"就自动等于 ``PRI/PW``（见
        :meth:`_update_false_targets` 的那张上界表）。
        """
        return eq.range_gate(float(self.spec["pulse_width"]) / 1.0e6)

    def _false_target_count(self, card: FalseTargetSpec) -> int:
        """AFSIM 的条数公式（``WSF_FT_EFFECT`` 原文）：

        ``N_ft = (PRI/PW) · (ScanTime/PRI) · N_int · jamming_pulse_density``

        ``quantity`` 给了就**直接用它**（原文："If the number of false targets
        is hard set it is used instead of the dynamically calculated number"），
        所以两个都给会被 ``RfJammer`` 在**装配期**拦下。

        ★ 公式**照抄原文的形状**（``(PRI/PW)·(ScanTime/PRI)``，不先化简成
        ``ScanTime/PW``）：两项各说各的事（每个 PRI 有多少个分辨单元 / 一次扫描
        有多少个 PRI），化简掉之后"PRI 在这个式子里出现两次"就看不见了——而
        紧接着的代码还要用同一个 PRI 算不模糊距离。

        ★ 结果**可以很大**（2 s 扫描、1 µs 脉宽、10 个积累脉冲、密度 0.1 ⇒
        ``2×10⁶`` 条），**不在这里封顶**：那是"干扰机想造多少"，而"雷达能显示
        多少"是另一件事，由 ``false_target_capacity`` 与距离几何管。
        """
        if card.quantity > 0:
            return card.quantity
        pw_s = float(self.spec["pulse_width"]) / 1.0e6
        prf = float(self.spec["pulse_repetition_frequency"])
        pri_s = 1.0 / prf
        scan_s = self.period_us / 1.0e6
        return int(
            (pri_s / pw_s)
            * (scan_s / pri_s)
            * self.pulses
            * card.pulse_density
        )

    def _lit_by(self, bearing_deg: float, arcs, full_circle: bool) -> bool:
        """这一拍照到 ``bearing_deg`` 没有。

        ★ 这一个判据有**两个**使用者（"扫到目标没有"与"扫到干扰机没有"），
        所以只能有一份实现：抄第二份的话，某天改了弧的半开闭性质，两个用户的
        行为会分叉，而症状是**只有假目标**在帧边界上凭空多一批或少一批。
        """
        if full_circle:
            return True
        return any(
            (bearing_deg - (self.face_deg + lo)) % 360.0 < hi - lo
            for lo, hi in arcs
        )

    def _update_false_targets(self, me, arcs, full_circle: bool) -> None:
        """第 7c 步：本拍扫过的那些假目标干扰机，各自铺出一串幽灵。

        **每拍一次**（在候选循环之前），不是逐目标一次。条数公式里没有目标
        （全是波形量），方位取的是**干扰机自己的**方位——挂在逐目标的求值点上，
        症状是"扫到 3 个真目标 ⇒ 幽灵也变成 3 倍"，而那个倍数在参数表上完全
        看不出来。

        三道工序：

        1. **先撤上一拍的**（:meth:`_clear_false_targets`）：幽灵是"这一拍的
           屏幕"，不是跟踪记忆。本版不做 AFSIM 的 ``persistence``（§9-53）。
        2. **这一拍扫到谁**：名册里方位落在本拍弧内的假目标干扰机。判据与
           "扫到目标没有"是**同一个**（:meth:`_lit_by`）——假回波只有天线指着
           那个方向时才进得来，所以慢转速雷达上幽灵与真航迹一样是**每圈一次**。
           ★ 方位之外还要过**带内**（``F_BW > 0``）、**遮蔽**（山那面照不到）
           与**到达功率门限**（``false_target_min_power``）三道（v0.13.31）。
           与压制那一支走**同一套判据**：同一台干扰机在两个视图里必须得到
           同一个"进不进得来"的答案。
        3. **铺**：逐台按距离分辨单元往外铺，总数受三个上界同时管。

        ★ 三个上界各管一件事，而且都是**算出来的**（不是拍的）：

        ==================  ==================================================
        ``false_target_capacity``  雷达的槽位（参数，默认 1000）
        ``PRI/PW``          一个不模糊距离里能放下多少个分辨单元
        ``max_range_m``     探测半径——更远的幽灵这部雷达根本看不见
        ==================  ==================================================

        所以"条数算出来两百万"不会变成两百万行航迹：真正上屏幕的是三者的最小
        值。★ ``false_targets_seen`` 记的是**没截断**的原始条数（洪泛判据要
        它），``phantoms_seen`` 记的是实际铺了几条——两个数都会出现在诊断里，
        因为"算出来多少"与"显示了多少"是**两件事**，混成一个数就再也分不清
        "干扰机没劲"与"雷达屏幕满了"。（三个"本拍"量由
        :meth:`_clear_false_targets` **一起**归零——早退那两条路上它们也不会
        留着上一拍的旧值。）

        ★ 没挂干扰机时**连名册都不查**（``is_empty`` 一步早退）：于是"没挂
        干扰机的想定逐字不变"不是靠某个 ``if`` 挡出来的（连随机流都不动）。
        """
        self._clear_false_targets()
        jam = self._jam
        if jam is None or jam.is_empty:
            return
        rows = [
            row
            for row in jam.on_roster(self.view.entity_id)
            if row.false_targets is not None
            and self._lit_by(row.bearing_deg, arcs, full_circle)
        ]
        if not rows:
            return
        self._require_false_target_budget()

        cell = self._range_cell_m()
        # 最大不模糊距离：比它更远的回波会折叠回下一个 PRI 里，雷达分不清。
        span = eq.range_gate(1.0 / float(self.spec["pulse_repetition_frequency"]))
        capacity = int(self.spec["false_target_capacity"])

        index = 0
        for row in rows:
            # ---- 三道"进不进得来"（v0.13.31）----
            # ★ 这里原先**只判了方位**（``_lit_by``）：名册不筛频段（那是受害方
            # 的事）、功率也没算。于是"跳频把干扰挡在带外"这条在**压制**那一支
            # 生效、在**假目标**这一支不生效——同一个干扰机在两个视图里得到
            # 两个答案。现在两支走同一套判据。
            f_bw = eq.bandwidth_overlap_ratio(
                row.frequency_hz,
                row.bandwidth_hz,
                self.frequency_hz,
                self.bandwidth_hz,
            )
            if f_bw <= 0.0:
                continue                        # 带外：假回波也造不出来
            if not self._has_line_of_sight(me, row.position, row.antenna_height_m):
                self.jamming_masked += 1
                continue                        # 山那面：照都照不到
            # 天线正扫到它 ⇒ 收的是**主瓣**增益。这不是"取个乐观值"：
            # 这条路径的前提就是"本拍照到它了"（上面 ``_lit_by`` 刚判过），
            # 而假回波正是趁天线指着干扰机的那一刻注进去的。
            # ★ 但干扰机**自己**的天线方向图仍要折（v0.13.34，§5.14.11 ④）：
            # 它可能正指着别处，那我这个方向少收增益，注入功率跟着降。
            # 判据与 ``jamming_effect`` 那一支**同一个函数**——两条路的门限
            # 若各算各的，"同一个门限"那句话就不再成立。
            tx_gain = row.gain
            if row.antenna_off_deg is not None:
                tx_gain *= eq.antenna_pattern_factor(
                    row.antenna_off_deg,
                    row.beam_width_deg,
                    eq.linear(DEFAULT_SIDELOBE_LEVEL_DB),
                )
            power = eq.jamming_power(
                row.radiated_power_w,
                tx_gain,
                self.gain,
                self.wavelength_m,
                row.slant_m,
                loss=row.internal_loss * self.jamming_path_loss,
                bandwidth_overlap=f_bw,
                polarization=eq.polarization_effect(
                    self.polarization_type,
                    row.polarization_type,
                    row.polarization,
                ),
            )
            if power < self.false_target_min_power_w:
                continue                        # ④ 到不了这台雷达的门限
            count = self._false_target_count(row.false_targets)
            # ★ 条数**先记满**再判槽位：槽位满了的机器照样在"想造多少条"里
            # 计数，否则洪泛判据会被"屏幕已经满了"这件事反过来稀释掉。
            # ★ 但**进不来**的那些（带外 / 被山挡 / 到不了门限）不记：它们的
            # 幽灵一条都不存在，记进去等于把"没进来"算成"屏幕被占了"。
            self.false_targets_seen += count
            if index >= capacity:
                continue
            # ★ 深度写成 ``min(max_range − R, span)``，**不要**写成
            # ``min(max_range, R + span) − R``：后者是两个大数相减，在
            # ``span`` 那一支上会掉一个 ulp——``PRI/PW`` 明明等于 100、
            # ``span/cell`` 也正好是 100.0，却因为
            # ``(R+span)−R = 14989.622899999998`` 而只铺出 **99** 条。
            # 一个幽灵的差别不会报错，只会在"条数对不上 AFSIM 公式"时
            # 变成一个查不出来的末位问题。
            depth = min(self.max_range_m - row.slant_m, span)
            fits = int(depth / cell)
            n = max(0, min(fits, count, capacity - index))
            for k in range(1, n + 1):
                slant = row.slant_m + k * cell
                self._emit_phantom(index, row.bearing_deg, slant, me)
                if slant < self._nearest_phantom_m:
                    self._nearest_phantom_m = slant
                index += 1
        self.phantoms_seen = index

    def _clear_false_targets(self) -> None:
        """把上一拍铺的幽灵从航迹表里撤掉。

        ★ 只撤**自己铺过的那些 ID**（``_phantoms``），不按"``target_id`` 是负数"
        去扫全表：负号只是编号约定、不是判据，而"按约定扫"会把别人写的负号
        ID 一起撤掉。漏撤的症状是幽灵**永远挂在屏幕上**——它们不在 ``_hits``
        里，:meth:`_prune` 根本不会碰它们。

        ★ **本拍画面的三个诊断量一起在这里归零**（``false_targets_seen`` /
        ``phantoms_seen`` / ``_nearest_phantom_m``）。把 ``false_targets_seen``
        留在铺幽灵那一段里归零的话，它会在**每一次早退**（名册空了、本拍没
        扫到任何假目标干扰机）之后留着上一拍的数——于是"干扰机已经关机"与
        "干扰机还在造 40 万条"在诊断里长得一模一样，而两个数的差别正是用来
        分辨这件事的。
        """
        for pid in self._phantoms:
            self.view.drop_contact(pid)
        self._phantoms.clear()
        self.false_targets_seen = 0
        self.phantoms_seen = 0
        self._nearest_phantom_m = float("inf")

    def _emit_phantom(self, index: int, bearing_deg: float, slant_m: float, me) -> None:
        """把一条幽灵写进**自己的**航迹表。

        ★ 高度填**量测者自己的**：二维搜索雷达给不出目标高度（真目标那一支也是
        直接抄真实高度，见 :meth:`_measure`），而幽灵连"真实高度"都不存在。
        填量测者高度至少是一个**已知**的编造，不是从别处借来的数（§9-53）。
        """
        pid = phantom_id(index)
        angle = radians(bearing_deg)
        self.view.add_contact(
            pid,
            quality=1.0,
            detected_at=int(self._now_us),
            bearing_deg=bearing_deg,
            range_m=slant_m,
            # 与 `_measure` 同一套极坐标 → 直角坐标（dx = d·sinθ、dy = −d·cosθ）
            x=me[0] + slant_m * sin(angle),
            y=me[1] - slant_m * cos(angle),
            z=me[2],
            phantom=True,
        )
        self._phantoms.add(pid)

    def _flood_passes(self, slant_m: float) -> bool:
        """屏幕被幽灵淹没时，这条**真实**检测还能不能过。

        两条规则，各有一份出处：

        1. **几何优先权** —— ``false_target_screener`` 原文："A real target that
           has a range from the sensor that is less than that of the closest
           false target will be allowed as a track." 比最近的幽灵**还近**的真
           目标**无条件放行**：它比所有假回波都先回来，雷达先看到的就是它。
        2. **洪泛抽签** —— ``WSF_SIMPLE_FT_EFFECT.use_random_calculation_draw``
           的式子 ``Blocked = UniformRandomDraw(0,1) > TrackCapacity /
           NumberFalseTargets``。反过来说就是"通过概率 = 容量 ÷ 条数"，装得下
           （``N_ft ≤ capacity``）就**不抽签**、全放行。

        ★ 抽签用**本雷达自己的随机流**（``self._rng``）：与 Pd 那一次抽签同源，
        所以"同一颗种子 ⇒ 同一场推演"这条可复现性在这里也成立。
        """
        if slant_m < self._nearest_phantom_m:
            return True
        capacity = int(self.spec["false_target_capacity"])
        if self.false_targets_seen <= capacity:
            return True
        return self._rng.random() <= capacity / self.false_targets_seen

    # -- 第 5~6 步：回波功率 → 单脉冲信噪比 --------------------------------

    def single_pulse_snr(
        self,
        range_m: float,
        rcs_m2: float | None = None,
        *,
        jammer_noise_w: float = 0.0,
    ) -> float:
        """目标在 ``range_m``（**斜距**）处的**单脉冲**信噪比（线性）。

        标定法下 RCS 的相对关系仍然生效（``σ/σ_ref`` 乘在标定好的 SNR 上）——
        这正好是雷达方程的线性部分，不必重推一遍。

        ``jammer_noise_w`` 是**要并入噪声的干扰功率**（W，默认 0）。干扰进来
        走的是 ``SN = SNR/(1 + (J/S)·SNR)``，而 ``J/S = J/(SNR·N)``
        （:func:`~milsim.models.percep.equation.snr_under_jamming`）。

        ★ 默认 0 时那个式子是**恒等映射**（逐位相同，不是"相差极小"），
        所以"没挂干扰机 ⇒ 数字一个不动"有数学依据，不靠分支挡住。
        """
        sigma = self.reference_rcs if rcs_m2 is None else rcs_m2
        if range_m <= 0.0:
            return float("inf")
        if self.range_model == "标定法":
            if self.reference_rcs <= 0.0:
                return 0.0
            sn = (
                self.snr_at_reference
                * (sigma / self.reference_rcs)
                * (self.detect_range_m / range_m) ** 4
            )
        else:
            sn = eq.received_power(
                self.power_w, self.gain, self.wavelength_m, sigma, range_m, self.loss
            ) / self.noise_w
        # ``sn <= 0`` 时不必（也不能）折算：``J/S`` 里那个 "∞ × 0" 会给 nan，
        # 而一个 nan 信噪比会顺着一整条链路传下去。没有信号就没有 J/S 可言。
        if jammer_noise_w <= 0.0 or sn <= 0.0:
            return sn
        return eq.snr_under_jamming(
            sn, eq.jamming_to_signal(jammer_noise_w, self.noise_w, sn)
        )

    # -- 第 7 步：检测概率 -------------------------------------------------

    def detection_probability(
        self,
        range_m: float,
        rcs_m2: float | None = None,
        *,
        jammer_noise_w: float = 0.0,
    ) -> float:
        """单次扫描的检测概率。**入口是单脉冲 SNR**（见模块头）。

        ★ v0.13.40：按 ``swerling_case`` 分派口径——
        ``0``（默认）走**非起伏** ``nonfluctuating_pd``，``1`` 走 ``swerling1_pd``，
        ``2..5`` 目前**没有**实现（Swerling 2/3/4 要"逐脉冲起伏"与"扫描间起伏"
        的区别，属于缺口，见 §9-73）。
        """
        snr = self.single_pulse_snr(range_m, rcs_m2, jammer_noise_w=jammer_noise_w)
        if self.swerling_case == 0:
            return eq.nonfluctuating_pd(snr, self.pulses, self.pfa)
        if self.swerling_case == 1:
            return eq.swerling1_pd(snr, self.pulses, self.pfa)
        raise ConfigurationError(
            f"swerling_case={self.swerling_case} 尚未实现：本模型只有 0（非起伏）"
            "与 1（Swerling 1）两个口径。Swerling 2/3/4 要区分'逐脉冲起伏'与"
            "'扫描间起伏'，属于已知缺口（§9-73）"
        )

    # -- 扫描律（第 1~3 步，自 v0.13.24 起行为不变） -----------------------

    @property
    def sweeps(self) -> float:
        """累计转角折算的**等效圈数**（``swept_deg / 360``）。"""
        return self.swept_deg / 360.0

    @property
    def mode(self) -> str:
        """扫描模式的一句话描述，给想定输出用。"""
        if self.fov_deg >= 360.0:
            return "圆周"
        if self.fov_deg <= 0.0:
            return f"固定指向 {self.face_deg:g}°"
        return f"扇扫 {self.fov_deg:g}° 朝 {self.face_deg:g}°"

    def _steer(self, delta_deg: float) -> tuple[list[tuple[float, float]], float]:
        """推进相位 ``delta_deg``，返回 ``(本拍扫过的弧, 实际转过的角度)``。

        弧是**相对 ``face_deg`` 的偏移**、半开区间 ``[lo, hi)``。

        三种扫描模式的差别**全在这个函数里**：

        * **圆周**：相位单调前进、``mod 360``。本拍一段弧。
        * **扇区**：相位在 ``±fov/2`` 之间往复，到界**反向**（不是取模）。
          一拍里折返几次就有几段弧；走过整整一个视场宽就说明两端都到过，
          本拍覆盖的是**整片视场**（一趟就够，剩下的按 ``2·fov`` 取模——
          一次完整往返后相位与方向完全复原，这是能取模的依据）。
        * **固定指向**：相位恒为 0、不推进，本拍只有一段**零宽**弧，宽度由
          ``beam_width`` 在调用处撑开。**转过的角度是 0**——天线没动。

        第二个返回值与 ``delta_deg`` 只在固定指向上不同：它决定 ``swept_deg``
        要不要累加，而那是"这部雷达转过多少"的唯一出处。
        """
        span = self.fov_deg
        if span >= 360.0:
            start = self.phase_deg
            self.phase_deg = (start + delta_deg) % 360.0
            return [(start, start + delta_deg)], delta_deg

        if span <= 0.0:
            self.phase_deg = 0.0
            return [(0.0, 0.0)], 0.0

        half = span / 2.0
        off = self.phase_deg
        direction = self.spin
        left = delta_deg
        whole = left >= span              # 走过一整个视场宽 ⇒ 两端都到过
        if whole:
            left = left % (2.0 * span)
        arcs: list[tuple[float, float]] = []
        for _ in range(8):                # 取模后至多两次折返，多出的只是保险
            if left <= 0.0:
                break
            room = (half - off) if direction > 0 else (off + half)
            if room <= 0.0:               # 正压在边界上：先掉头，还没有位移
                direction = -direction
                continue
            step = left if left < room else room
            arcs.append((off, off + step) if direction > 0 else (off - step, off))
            off += direction * step
            left -= step
        self.phase_deg = off
        self.spin = direction
        if whole:
            return [(-half, half)], delta_deg   # 两端都到过 = 整片视场
        return arcs, delta_deg

    # -- 主循环 ------------------------------------------------------------

    def _sweep(self, engine, event) -> EventResult:
        # 本帧转过的角度**按真实经过的时间**算，不用名义步长。引擎允许事件
        # 被推迟（栅栏截断、同刻排队），按名义步长算会把这部分时间丢掉；
        # 相位一旦落后，慢转速雷达再也补不回来（每拍都少一点，永不闭合）。
        now_us = float(engine.now)
        delta_deg = 360.0 * (now_us - self._last_us) / self.period_us
        self._last_us = now_us
        self._now_us = now_us

        if delta_deg <= 0.0:
            # 装配时刻那一拍：只对齐时钟，不推进相位、也不照任何东西。
            # 与机动件"第一帧只走时钟不挪窝"是同一条离散约定。同刻重复
            # 触发（dt ≤ 0）也在这里一并挡掉。
            return EventResult.RESCHEDULE

        self.ticks += 1
        swept, turned_deg = self._steer(delta_deg)
        self.swept_deg += turned_deg
        # 粗筛与精筛用**同一组**"被照到的弧"（见 _lit_arcs 的警告）。
        arcs = self._lit_arcs(swept)

        if self.span_m <= 0.0:
            return EventResult.RESCHEDULE        # 全球层没有逐格精度

        me = self.view.my_position()
        if me is None:
            return EventResult.RESCHEDULE

        candidates = self._candidates(arcs)
        if candidates is None:
            return EventResult.RESCHEDULE

        full_circle = candidates.pop()           # 哨兵：整圈时不判方位
        # ---- 第 7c 步：假目标 ----
        # ★ 在候选循环**之前**、且**每拍一次**：幽灵要先把槽位占上，本拍的真实
        # 检测才谈得上"被淹没"（§5.14）。反过来放会在第一拍漏掉这一层。
        self._update_false_targets(me, arcs, full_circle)
        for target_id in candidates:
            self._evaluate(engine, me, target_id, full_circle, arcs)
        self._prune()
        return EventResult.RESCHEDULE

    def _lit_arcs(
        self, arcs: list[tuple[float, float]]
    ) -> list[tuple[float, float]]:
        """把"天线扫过的弧"外扩成"**被照到的弧**"。

        波束有宽度 ⇒ 天线相位走到 ``p`` 时，真正被照亮的是 ``p ± beam_width/2``。
        所以本拍照到的是每一段弧两端各外扩半个波束；零宽波束（默认）下这是恒等。

        ★ 这个外扩**只能有一处**：粗筛（索引按格心判）与精筛（模型按真实位置判）
        必须用同一组弧。只外扩一处的话，索引交出来的候选会在精筛那里被同一段
        没外扩的弧挡回去，于是"波束宽度"只在**恰好压线**的目标上生效——
        表现成"改 beam_width 没反应"，而参数表看着完全正常。
        """
        half = self.beam_deg / 2.0
        if half <= 0.0:
            return arcs
        return [(lo - half, hi + half) for lo, hi in arcs]

    def _candidates(self, arcs: list[tuple[float, float]]):
        """粗筛：把"本拍照到的弧"交给空间索引，返回候选（末尾塞一个哨兵）。

        粗筛只保证"格心在扇区里"，方位要用真实位置复判一次（索引按格心判，
        有格心半张角量级的量化误差）。哨兵放在列表末尾而不是单独返回，
        是为了让调用处只有一处循环。

        传进来的 ``arcs`` 是**已外扩**的（见 :meth:`_lit_arcs`）。
        """
        radius_cells = int(self.max_range_m / self.span_m) + 1

        full_circle = any(hi - lo >= 360.0 for lo, hi in arcs)
        if full_circle:
            # 一拍就转过整圈（或整片视场）：不必按扇区算
            return list(self.view.query_disk(radius_cells)) + [True]

        candidates: list[int] = []
        seen: set[int] = set()
        for lo, hi in arcs:
            for target_id in self.view.query_sector(
                self.face_deg + lo, self.face_deg + hi, radius_cells
            ):
                if target_id not in seen:
                    seen.add(target_id)
                    candidates.append(target_id)
        candidates.append(False)
        return candidates

    def _evaluate(
        self,
        engine,
        me: tuple[float, float, float],
        target_id: int,
        full_circle: bool,
        arcs: list[tuple[float, float]],
    ) -> None:
        """一个候选目标走完第 3 步的精筛 + 第 4~8 步。"""
        if target_id == self.view.entity_id:      # 别把自己当目标
            return
        target = self.view.position_of(target_id)
        if target is None:
            return
        horizontal = hypot(target[0] - me[0], target[1] - me[1])
        if horizontal > self.max_range_m:
            return                                  # 第 3 步：距离门

        bearing = self.view.bearing_to(target_id)
        if bearing is None:
            return
        # 索引的扇区判定用的是**格心**，这里按真实位置复判一次方位。
        # 弧取**半开区间**：闭区间会让恰好落在帧边界上的方位被相邻两帧
        # 各算一次（天线在那一刻只经过一次）。
        if not self._lit_by(bearing, arcs, full_circle):
            return                                  # 第 3 步：方位门

        # ---- 第 4 步：几何（地平线 + 地形通视，同一个调用）----
        if not self._has_line_of_sight(me, target):
            self.geometry_blocked += 1
            # 波束确实照到了它，只是什么也没收到 —— 这是**漏检**，不是"没扫到"。
            self._update_track(target_id, hit=False)
            return

        # ---- 第 5~7 步：斜距 → 单脉冲 SNR → Pd ----
        # 雷达方程要的是**斜距**；航迹里的 range 沿用既有的水平距离口径。
        slant = sqrt(horizontal * horizontal + (target[2] - me[2]) ** 2)
        rcs = self._rcs_of(target_id)
        # ---- 第 6 步的干扰项 ----
        # ★ 这一刻天线正指着**这个目标**（它落在本拍照到的弧里），所以干扰按
        # "谁在这个方向上漏进来"算——这正是 AFSIM 的求值时机（"at the time a
        # radar detection ... occurs"）。目标自己的方位就是波束指向，不必
        # 再去问天线相位：判"照到没照到"用的已经是同一段弧。
        # ★ 判欺骗要先知道**未受干扰**的 SNR（``J/S`` 要它），所以这里取一次
        # 无干扰的 SNR 交给第 6 步；被干扰后的 SNR 由下面那一步自己算。
        effect = self.jamming_effect(
            bearing, self.single_pulse_snr(slant, rcs)
        )
        self.jamming_w = effect.power_w
        self.jamming_range_m = effect.range_bias_m
        self.jamming_azimuth_deg = effect.azimuth_bias_deg
        if effect.power_w > 0.0:
            self.jammed_evals += 1
        pd = self.detection_probability(slant, rcs, jammer_noise_w=effect.power_w)
        self.last_pd = pd
        hit = self._rng.random() < pd
        # ---- 第 7c 步的另一半：屏幕被幽灵淹了没有 ----
        # ★ 放在 **hit 之后、_update_track 之前**：AFSIM 的原话是 "If it is
        # flooded the detection is **blocked**" ——被淹掉的那一次**等于没检测
        # 到**，所以它要进 M/N 窗口当一次漏检，而不是"检测到了但不建航"。
        # ★ 第一道守卫 ``false_targets_seen > 0`` 很关键：没有假目标时**一次
        # 随机数都不多抽**，否则整条随机流会移位，"没挂干扰机的想定逐字不变"
        # 这条验收判据会以一个极隐蔽的方式失效（每一拍的数字都变一点点）。
        if hit and self.false_targets_seen > 0 and not self._flood_passes(slant):
            hit = False
            self.jamming_blocked += 1
        if hit:
            self.detections += 1

        # ---- 第 8 步：M/N 建航 ----
        # ``_update_track`` 返回的是"**这条航迹现在还算不算数**"，不是"这一拍命中
        # 了"。所以"M/N 维持住了"（``True``）里面有两种情况，要分开处理：
        # 命中 ⇒ 写一条新量测；没命中但航迹还在 ⇒ **什么都不写**。
        #
        # ★ 后者不能拿真位置去刷新：那是"没观测也写一条量测"，等于把目标的真实
        # 位置直接灌进航迹表（上帝视角泄漏）。真实雷达靠**预测**把航迹推下去，
        # 我们这里没有滤波器，所以正确的做法是**让航迹停在最后一次量测上**——
        # 位置过时与航迹撤除是两件事。
        if not self._update_track(target_id, hit=hit):
            return
        if not hit:
            return

        # ---- 第 7b 步：量测误差 ----
        # ★ 放在**命中之后**：误差不参与 hit 判定。让它参与的话，Pd 就等于在
        # "命中判一次、量测再判一次"里被算了两次，实测命中率与设计值对不上。
        measured_bearing, measured_horizontal = self._measure(
            bearing,
            horizontal,
            slant,
            target[2] - me[2],
            rcs,
            jammer_noise_w=effect.power_w,
            range_bias_m=effect.range_bias_m,
            azimuth_bias_deg=effect.azimuth_bias_deg,
        )
        angle = radians(measured_bearing)
        self.view.add_contact(
            target_id,
            quality=pd,
            detected_at=engine.now,
            bearing_deg=measured_bearing,
            range_m=measured_horizontal,
            # 极坐标 → 直角坐标。★ 与 KinematicsTable.advance_all 同一套几何：
            # dx = d·sinθ、dy = −d·cosθ（0° 指北，y 轴指南）。
            x=me[0] + measured_horizontal * sin(angle),
            y=me[1] - measured_horizontal * cos(angle),
            z=target[2],
            # 敌我状态在**量测这一刻**判出来写进航迹（v0.13.33）：航迹一旦上路，
            # 接收方手里没有阵营信息，只能靠这个字段。
            iff=self._iff_of(target_id),
        )
        self.contacts_seen.append(target_id)

    # -- 第 4 步：几何 -----------------------------------------------------

    def _has_line_of_sight(self, me, target, peer_height_m: float = 0.0) -> bool:
        """地平线 + 地形通视。

        ``peer_height_m`` 是**对面那一端自己的架高**（v0.13.33 起）。
        它存在的唯一理由是**干扰链路的不对称**：探测是"我架天线、他飞着"，
        而干扰是"**他架天线**压我"——一台架高 10 m 的干扰机能照到我，
        而我拿自己 10 m 的架高是算不出来的。

        * ``peer_height_m <= 0``（默认）⇒ 用**本雷达自己的架高**给两端
          （旧口径）。旧想定因此逐字不变。
        * ``peer_height_m > 0`` ⇒ **本端仍用自己的架高**（只有这一端真的是
          我的天线），对面那一端换成它的架高。判据的其余边界（没有地形数据 /
          战区外 / 本雷达没给架高 ⇒ 不判）一律照旧。

        **没有逐格精度时返回 True**（不判）——战区外 / 纯单元测试里
        ``NavService`` 可能是 None。这与 ``cell_span_m() == 0`` 那条约定
        同源：没有地形数据就没有地形遮蔽可言，而传一个假的高度差出去
        只会把"数据缺失"伪装成"被遮挡"。

        ★ **``antenna_height = 0`` 表示"没有架高这一项"，不是"天线贴在地面上"**
        ——同样走"没有数据就不判"这条口子。这个区分是**必须**的：

        贴地天线对同样贴地的目标，含地球曲率的余隙在任意距离上**恒为负**
        （视线在两端之间整段落到地面以下），于是**一部没写架高的雷达会一条
        航迹都建不起来**，而进程退出码 0、输出看着"正常"——正是本项目最要
        消灭的那类静默错（实测：`scenarios/library_demo.txt` 接上第 4 步后
        航迹从有变 0）。

        而"贴地"这个解释本身也是编出来的：已落库的雷达里 3 型**真实型号**
        （`library/external.txt`）的数据表**根本没有天线架高这一项**，凭空
        给个默认值就是造数据。所以定成"没给就不判"，要判就写架高
        （`scenarios/patrol.txt` 写 10 m）。这条缺口记在 §9-42。
        """
        nav = self._nav
        if nav is None:
            return True
        if self.antenna_height_m <= 0.0:
            return True                                    # 没给架高 ⇒ 不判几何
        ref = self.view.my_cell()
        if ref is None or not hasattr(ref, "zone_id"):     # 全球层
            return True
        # ★ 对面那一端自己的架高（干扰机架高）。<=0 ⇒ 用本雷达的架高给两端
        #   （旧口径，逐字不变）。判据**同一个调用**，只是把"对面也有架高"
        #   这件事喂进去——干扰链路两端不对称，探测链路两端都取本机。
        peer = peer_height_m if peer_height_m > 0.0 else self.antenna_height_m
        visible = nav.line_of_sight(
            ref,
            (me[0], me[1]),
            (target[0], target[1]),
            me[2],
            target[2],
            observer_extra_height=self.antenna_height_m,
            target_extra_height=peer,
            terrain_floor=0.0,
        )
        return True if visible is None else bool(visible)

    def _iff_of(self, target_id: int) -> str:
        """这条航迹的**敌我状态**（v0.13.33）。

        口径（照 AFSIM ``WsfIFF_Manager`` 的**简化版**，见 §9-57）：

        * 目标方 == 本平台方（且都非空）⇒ ``IFF_FRIEND``
        * 两边都有方、但不同 ⇒ ``IFF_FOE``
        * 任一方查不到（空串）⇒ ``IFF_UNKNOWN``

        ★ 这不是"雷达探测出的敌我"：真实的 IFF 是独立的询问/应答系统。
        AFSIM 也把它简化成**想定给定的态势表**（``iff_mapping`` 块，查
        (方,方)/(方,类别)/(方,默认) 三张表，都没查到就"同方为友、否则为敌"）。
        我们取同一口径的简版——**敌方阵营是态势库里就写着的**，不是从这里
        推出来的上帝视角。判据放在**量测那一刻**而不是事后查，是因为航迹
        一旦上路，接收方手里没有阵营信息，只能靠这个字段。

        ★ 判不出来就是 ``UNKNOWN``，**不猜"友"**：把未知当友会让"照敌方
        航迹干扰"漏掉没写阵营的那一半，而症状是"干扰机不响应"，参数表正常。
        """
        reader = self._side_of
        if reader is None:
            return IFF_UNKNOWN
        mine = reader(self.view.entity_id)
        theirs = reader(target_id)
        if not mine or not theirs:
            return IFF_UNKNOWN
        return IFF_FRIEND if mine == theirs else IFF_FOE

    def _rcs_of(self, target_id: int) -> float:
        """目标的雷达截面积（m²）。

        读平台的 ``radar_cross_section``；读不到就用本雷达的 ``reference_rcs``
        （数据表的"探测距离"本来就是对着某个参考 RCS 标的，拿它当缺省
        目标比拿 1 m² 更贴近"这张表在说什么"）。
        """
        reader = self._target_attr
        if reader is None:
            return self.reference_rcs
        value = reader(target_id, "radar_cross_section", None)
        if value is None:
            return self.reference_rcs
        return max(0.0, float(value))

    # -- 第 8 步：M/N 建航 -------------------------------------------------

    def _update_track(self, target_id: int, *, hit: bool) -> bool:
        """记一次照射结果，返回"现在该不该有一条航迹"。

        窗口用 ``deque(maxlen=...)``：只保留最近若干次**被照射**的记录，
        所以 ``sum(...) >= hits`` 就是"最近 window 次被照射里至少命中 hits 次"。

        建航与撤航用的是**不同**的窗口和门限（默认 ``3/5`` 建、``2/5`` 维持，
        见参数表；``scenarios/patrol.txt`` 也是这一档）。这是雷达的常规做法：
        建航要严（压住虚警），维持要松（别把已经跟上的目标轻易丢掉）。
        """
        if not self.view.is_alive(target_id):
            self._forget(target_id)
            return False

        window = int(
            self.spec["establish_window"]
            if target_id not in self._tracks
            else self.spec["maintain_window"]
        )
        needed = int(
            self.spec["hits_to_establish"]
            if target_id not in self._tracks
            else self.spec["hits_to_maintain"]
        )
        record = self._hits.get(target_id)
        if record is None or record.maxlen != window:
            record = deque(record or (), maxlen=window)
            self._hits[target_id] = record
        record.append(hit)

        enough = sum(record) >= needed
        if target_id in self._tracks:
            if enough:
                return True
            self._tracks.discard(target_id)
            self.view.drop_contact(target_id)
            return False
        if enough:
            self._tracks.add(target_id)
            return True
        return False

    def _forget(self, target_id: int) -> None:
        """忘掉一个目标：建航状态与航迹一起清掉。

        ★ **判"在不在集合里"要用 ``in``，不能拿 ``discard`` 的返回值当条件**——
        ``set.discard`` 的返回值是 ``None``（它存在就是为了"没有也不报错"），
        ``if s.discard(x):`` 恒为假 ⇒ 状态清了、**航迹却留在航迹表里**。
        这是"幽灵航迹"：目标已经不在世，雷达表上还挂着它，而这一步不报任何错。
        """
        self._hits.pop(target_id, None)
        if target_id in self._tracks:
            self._tracks.discard(target_id)
            self.view.drop_contact(target_id)

    def _prune(self) -> None:
        """把已经不在世的目标的建航状态清掉。

        不清的后果不是内存（几乎不增长）而是**ID 复用**：实体注销后 ID 可能
        被后来者拿走，于是新目标一出生就"已经建航"，而这一步不产生任何日志。
        """
        stale = [t for t in self._hits if not self.view.is_alive(t)]
        for target_id in stale:
            self._forget(target_id)

    # -- 展示 --------------------------------------------------------------

    def describe(self) -> str:
        """``描述 [路由 N=脉冲数]``。**未 ``initialize`` 时不给后半段。**

        ``range_model`` 是装配期算出来的，而 ``repr`` 在装配**之前**就会被
        调用——工厂构建失败时的报错信息里就带着它。所以这里必须容忍
        "还没初始化"，否则一个参数写错会变成一个看不懂的 AttributeError。
        """
        base = super().describe()
        model = getattr(self, "range_model", "")
        if not model:
            return base
        return f"{base} [{model} N={self.pulses}]"


__all__ = [
    "DEFAULT_REFERENCE_RCS",
    "DEFAULT_REQUIRED_PD",
    "FalseTargetSource",
    "JammingEffect",
    "RadarSensor",
]
