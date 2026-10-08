"""电子战门面：干扰源名册 + 受害接收机在"检测那一刻"的几何查询。

为什么要有这一层
================

"谁在压我"是一个**全局汇总**问题。AFSIM 的原话是 "WSF will **sum the power
for every possible jammer** that can affect the output (i.e.: if there is
**in-band** power that would affect the receiver)"。受害的接收机自己不可能知道
战场上还有哪些干扰机——那是施扰方的东西，而且分布在别的平台上。

而组件之间不许互相 import，所以这个汇总必须由装配层注入的一个门面来回答。
本模块属于 ``services`` 层，**不许 import ``models``**：干扰源在这里是一张
纯数据（:class:`JammerSpec`），由干扰机组件在 ``initialize()`` 里登记进来，
受害接收机在"要检测 / 要收信"的那一刻来查。

★ 分工：**门面给几何，受害方算物理**
------------------------------------

AFSIM 的那句话里有两个动作："**in-band** 筛"与"**sum the power**"。
它们被刻意分在两侧：

====================  ============================================
门面（本模块）          受害接收机（雷达 / 通信）
====================  ============================================
谁在册、在哪、多远      按自己的 ``B_r`` / 调谐频率算 ``F_BW``
在哪个方位             按自己的主瓣 / 副瓣取接收增益
进没进我的主瓣         **按自己的** ``N`` 归一化成 ``J/S``
（**不碰**频率交叠）   最后求和
====================  ============================================

分界线的依据是**谁拥有那个数**：``F_BW`` 要用**两台设备**的频段，但"我的调谐带"
是受害方的属性；``J/S`` 要用受害方的噪声温度。把它们放在门面里，门面就得反过来
认识每一个受害方的内部结构——那正是窄接口要避免的。

附带的好处是**没有反向依赖**：门面不需要 ``C_LIGHT``、``FOUR_PI`` 这些住在
``models.percep.equation`` 里的常数，于是它真的只 import ``services``。

★ 两种卡在这里**都只是原样带过去**（``JammerSpec.deception`` →
``Threat.deception``、``JammerSpec.false_targets`` → ``Threat.false_targets``），
门面不算它们。因为"骗得成骗不成"要看 ``J/S``、而"能造出多少条假目标"要看
受害方自己的脉宽与脉冲重复周期——那都是受害方的账（见 :class:`DeceptionSpec`
与 :class:`FalseTargetSpec`）。

★ "没挂干扰机 ⇒ 零开销"是正确性的一部分
--------------------------------------

``_jammers`` 为空时 :meth:`threats` / :meth:`on_roster` **连位置都不查**、直接
返回空元组。这不只是省时间：它保证"没挂干扰机的想定逐字不变"这条验收判据不是
靠一个 ``if`` 挡出来的，而是"没有这个东西"的自然结果。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Any

from ..errors import ConfigurationError
from .spatial import bearing_of_offset
from .store import IFF_FOE, Contact

#: 受害接收机在**副瓣方向**上的相对增益（dB）。
#:
#: ``-13 dB`` 是"第一副瓣"的经典值，也是 AFSIM 那类天线模型的常用默认。
#: 放这里而不是各组件各写一份：雷达与通信接收机是**同一个物理量**的两个
#: 使用者，"各写一份迟早不同步，而且不同步时不报错"。
DEFAULT_SIDELOBE_LEVEL_DB = -13.0

#: 判定"干扰机是否落在受害方的主瓣里"时，方位差要拿波束张角的多少来比。
#: 取一半：波束张角是**全宽**，方位差是相对**轴心**的偏移。
MAIN_LOBE_HALF = 0.5


def angular_offset_deg(a_deg: float, b_deg: float) -> float:
    """两个方位角的**最小夹角**（度，0~180）。

    直接相减会让 359° 与 1° 差出 358°，而它们在物理上只差 2°——这类"跨零度"
    的错误不会在任何地方显形，只会让主/副瓣判反、副瓣增益偶尔用成主瓣增益。
    """
    return abs(((a_deg - b_deg + 180.0) % 360.0) - 180.0)


def radiated_power(peak_w: float, duty_cycle: float, use_peak_power: bool) -> float:
    """**发射链真正进 Jam.1 的那个功率**（W）。

    ★ AFSIM 的默认走平均功率，不是峰值。依据是源码而不是文档：
    ``WsfEM_Xmtr::GetPower()`` 是 ``mUsePeakPower ? GetPeakPower() :
    GetAveragePower()``，而 ``mUsePeakPower`` 的初值是 ``false``、
    ``GetAveragePower() = GetPeakPower() * GetDutyCycle()``、
    ``mDutyCycle`` 的初值是 ``1.0``。

    ⇒ **脉冲干扰机的占空比进 J**（占空比 1% ⇒ −20 dB），而
    ``use_peak_power true`` 的那一台（峰值功率受限）则不进。占空比默认 1
    时两者相等，所以连续波干扰机看不出这个区别。

    （Jam.1 的文档把符号写成 ``P_peak``，与源码默认行为不一致。以源码为准，
    理由与偏差量见设计文档 §5.14。）

    ★ **写成模块级函数而不是某一个类的方法**：压制链与欺骗链（``DeceptionSpec``
    的那一份功率）是两条独立的发射链，但"峰值还是平均"这条规则**只有一条**。
    各写一份的话，症状是"改了 ``use_peak_power`` 只有一条链变了"，而参数表上
    只有一项、看起来完全正常。
    """
    if use_peak_power:
        return peak_w
    return peak_w * duty_cycle


@dataclass(frozen=True, slots=True)
class DeceptionSpec:
    """一台干扰机的**欺骗卡**：它能把量测骗成什么样。

    ★ 欺骗与压制是**两条独立的发射链**，后果也完全不同（v0.13.31 起功率可分开）：

    ==============  ================================================
    压制（噪声）     把功率并进噪声 ⇒ ``S/(N+J)`` 变小 ⇒ Pd 变小
    欺骗（假信号）   把量测中心**挪走** ⇒ 航迹建得起来，但**位置是错的**
    ==============  ================================================

    ★ **两条链各有各的功率**（:attr:`power_w`）：AFSIM 的 ``WSF_RF_JAMMER``
    就是一个部件里挂**多个 ``transmitter``**，每个 ``effect`` 用自己的那台。
    "只骗不压"（欺骗链功率足、压制链功率小）因此是一句能表达的话。

    不给（``power_w = 0``）⇒ **两链共用压制那一份功率**，也就是"只有一条链"
    的旧行为。这个默认值是刻意的：它让"没写欺骗功率"的想定数字**一个不动**。

    所以欺骗卡不是"另一台设备"，而是同一台干扰机的另一种效果；一张
    :class:`JammerSpec` 上最多挂一张（``deception``），没挂就是纯压制。

    ★ **不是有功率就一定骗得成**：假信号要盖过回波才像真的，所以有一道
    ``required_j_to_s_db`` 门限（AFSIM 的 ``WSF_SLB_EFFECT`` 里就有这一项）。
    门限以下这台干扰机只是"压噪声的"，不产生偏置——这正是"压制弱、欺骗强"
    这句话在代码里的落点。
    """

    #: 生效门限：``J/S ≥ 10^(required/10)``。AFSIM 的默认是 3 dB（J 为 S 的两倍）。
    required_j_to_s_db: float = 3.0
    #: 距离偏置（米）。正 = 把目标**报得更远**。这是静态的那一份。
    range_bias_m: float = 0.0
    #: 方位偏置（度）。正 = 顺时针偏。
    azimuth_bias_deg: float = 0.0
    #: **距离拖引速率**（米/秒，RGPO）：被跟踪期间偏置随时间线性增长。
    #: 0 = 不拖引（只有静态偏置）。符号决定往哪边拉。
    walkoff_rate_mps: float = 0.0
    #: 拖引的**封顶**（米）。**0 = 不封顶**（同本项目的"0 = 不限制"约定）。
    holdout_m: float = 0.0
    #: 到顶之后**重新开始拉**（`range gate pull-off` 的 recycle）。
    #: ``False`` = 拉满就停在那儿。只有效于"封了顶"的情形。
    recycle: bool = False
    #: 欺骗链的发射功率（W，**已经过了"峰值还是平均"那条规则**，见
    #: :func:`radiated_power`）。**0 = 与压制共用同一份功率**（旧行为）。
    #: ★ 新字段一律加在末尾：已有的位置含义一个不动，"按位置构造"的调用方
    #: 不会因为一次加字段而静默拿到错的值。
    power_w: float = 0.0
    #: 诊断用的型号名。
    tag: str = ""


@dataclass(frozen=True, slots=True)
class FalseTargetSpec:
    """一台干扰机的**假目标卡**：它能在受害方的屏幕上造出多少个不存在的目标。

    ★ 这是同一台干扰机的**第三种效果**，与前两种**同时生效**：

    ==============  ================================  ================================
    压制（噪声）     把功率并进噪声 ⇒ Pd 变小            真目标**更难建航**
    欺骗（假信号）   把量测中心搬走                      航迹**建得起来，但是错的**
    假目标           航迹表里添**幽灵**、占满槽位        真目标**被挤出去 / 被淹没**
    ==============  ================================  ================================

    ⇒ 真假目标是**两类**后果（"没建起来" vs "建起来了但不是它"），而**一台机器
    可以同时具备**——所以三张卡都挂在同一张 :class:`JammerSpec` 上，
    而不是三种不能混用的组件。AFSIM 也是这个口径：一个 ``electronic_attack
    technique`` 里可以**同时**列出多个 ``effect``。

    ★ **条数不在这张卡上**
    ----------------------

    AFSIM 的条数公式（``WSF_FT_EFFECT`` 原文）要用**受害雷达自己的波形参数**：

    ::

        NumberOfFalseTargets = (PRI/PW) * (ScanTime/PRI) * NumberPulsesIntegrated
                               * jamming_pulse_density

    所以卡上只声明**干扰机自己的两个旋钮**，条数由**受害侧**算——与
    "门面给几何、受害方算物理"是同一条分工。这也解释了为什么这张卡没有
    功率门限：AFSIM 的 ``WSF_SIMPLE_FT_EFFECT`` 里只有
    "密度 / 容量 / 随机抽签" 三样（§9-51 记下了这个缺口）。

    ``pulse_density`` 与 ``quantity`` **互斥**（AFSIM 原文 "mutually
    exclusive"）：给了固定条数就不再用密度反推条数。
    """

    #: 脉冲密度，``[0, 1]``。**0 = 没有这张卡**。
    #: ★ 为什么不是 AFSIM 的默认 ``0.1``：本项目的默认值必须让"没写它"的
    #: 行为逐字不变，而"一挂上干扰机就自动放幽灵"会让所有既有想定变样。
    pulse_density: float = 0.0
    #: 固定条数。0 = 按 ``pulse_density`` 算（与它互斥，两个都给是配置错误）。
    quantity: int = 0
    #: 诊断用的型号名。
    tag: str = ""


@dataclass(frozen=True, slots=True)
class JammerSpec:
    """一台干扰机的**能力声明**（纯数据，不含行为）。

    干扰机组件在 ``initialize()`` 里造一个登记进来，受害接收机读它。字段全是
    "这台机器是什么"，**没有一个字是"它现在压谁"**——后者由几何与频段决定。
    """

    entity_id: int
    #: **总**功率（W）。口径同 AFSIM 的 ``transmitter.power``。
    #: ★ v0.13.33 起它是**整台机器所有天线加起来的**那一份：只有一条天线时
    #: 就是这条天线的功率（旧想定逐字不变）；``antenna_count`` 条时由装配层
    #: 分摊好再登记进来（见 :attr:`antenna_count`）——所以这里**不叫"每台的"
    #: 功率**，那会在多天线时静默地把系统功率乘以条数。
    power_w: float
    #: 峰值增益（线性），取朝**被压对象**的那一个。
    gain: float
    #: 干扰机谱中心频率（Hz）。
    frequency_hz: float
    #: **天线条数**（v0.13.33）。1（默认）= 单天线 ⇒ 与旧想定逐字相同。
    #: ★ 名册是"**每条天线**占一行"，所以同一台机器的多条天线是**几条独立条目**
    #: （各有自己的位置，因此各有自己的斜距 / 方位 / 遮蔽），名字靠
    #: ``tag`` 里那个 ``#序号`` 串起来；``entity_id`` 相同**不代表**是同一条天线。
    #: ⇒ 受害方侧完全不必知道"多天线"这件事（见 §5.14.4）。
    antenna_count: int = 1
    #: 干扰机谱宽（Hz）。**0 = 连续波**（单频点），见 ``bandwidth_overlap_ratio``。
    bandwidth_hz: float = 0.0
    #: 占空比，``(0, 1]``。1 = 连续波。**默认档下它进 J**（见下）。
    duty_cycle: float = 1.0
    #: 见 :meth:`radiated_power_w`。默认 ``False``（= AFSIM 的 ``mUsePeakPower``
    #: 初值），也就是**按平均功率算**。
    use_peak_power: bool = False
    #: 干扰机内部损耗（线性，**单程**）。0 dB 写 1.0。
    internal_loss: float = 1.0
    #: 干扰机**天线架高**（米），v0.13.33 加。0 = 没给 ⇒ 判地形遮蔽时用受害方
    #: 自己的架高给两端（旧口径，逐字不变）。
    #: ★ 它只影响**遮蔽判据**（山那面 / 地平线），不影响链路预算——干扰机在
    #: 三维位置里的 z 已经进了斜距。这一项说的是"天线架多高"，不是"飞多高"。
    antenna_height_m: float = 0.0
    #: **极化失配的显式覆盖**（线性比例）。对应 AFSIM 接收机那条
    #: ``polarization_effect <pol> <fraction>`` 命令——它也**只做覆盖**。
    #: **负数 = 未给** ⇒ 由受害方按两端**类型**去查
    #: :func:`~milsim.models.percep.equation.polarization_effect` 那张 7×7 表。
    #: ★ 为什么"未给"不用 1.0 表示：1.0 是一个**合法且语义不同**的取值
    #: （"永远不扣"，与两端类型无关），拿它兼作"没写"就把两件事混成一件。
    polarization: float = -1.0
    #: 干扰机自己的**极化类型**（AFSIM 同名参数；七个取值，默认 ``"default"``
    #: = 没声明 ⇒ 不产生失配）。它只说明"这台机器朝哪个方向极化"，
    #: **折算成损耗是受害方的事**——要用两端类型查表，施扰侧算不了（§5.14）。
    polarization_type: str = "default"
    #: 同阵营不打（同 AFSIM 的 ``ignore_same_side``）。默认开。
    ignore_same_side: bool = True
    #: **欺骗卡**（可选）。``None`` = 这台不搬量测中心。
    deception: DeceptionSpec | None = None
    #: **假目标卡**（可选）。``None`` = 这台不放幽灵。
    false_targets: FalseTargetSpec | None = None
    #: **天线主瓣全宽**（度，v0.13.34 加）。**0（默认）= 各向同性** ⇒ 方向图
    #: 因子恒 1.0，与加这一项之前逐位相同（⟨最省⟩口径：不写就不启用）。
    #: >0 才启用连续方向图：主瓣内平滑降到 −3 dB，主瓣外平坦 −13 dB。
    #: ★ 只认显式写的值，**不从 ``gain`` 反推**（§5.14.11 ④ ⟨丙⟩）。
    beam_width_deg: float = 0.0
    #: **照哪条航迹置向**（v0.13.34 加）。空串（默认）= 不启用照航迹 ⇒ 逐字走
    #: 旧逻辑（不看朝向、不查航迹表），老想定零回归。
    #: 非空 ⇒ 门面在**本机航迹表**里挑 ``iff == foe`` 的条目置向：
    #: ``"nearest"`` 挑斜距最近的，也可以写实体名 / 实体 ID（都认）。
    #: ⇒ 挑不到 / 挑中的不是敌 ⇒ 这台**不辐射**（不静默退化）。
    aim_at: str = ""
    #: 诊断用的型号名。
    tag: str = ""

    def radiated_power_w(self) -> float:
        """真正进 Jam.1 的那个功率（= 压制链那一份）。

        口径与出处全在 :func:`radiated_power`（**规则只有一处实现**）——
        这里只是把本台的三个字段喂进去，免得调用方每次都要记得那三个名字。
        """
        return radiated_power(self.power_w, self.duty_cycle, self.use_peak_power)

    def total_power_w(self, machine_total_w: float | None = None) -> float:
        """**整台机器**的峰值功率（W）。

        ★ 为什么必须由调用方（或装配层）把那个数带进来：名册是**按天线**登记
        的，一条天线只知道自己那一份，**推不出**机器总量——``power_w × 条数``
        是错的（那条天线的份额本来就是总量除出来的，再乘回去等于把总量平方再
        除）。默认返回本条天线自己的 ``power_w``，那是**单天线**时唯一正确的
        答案，也是"没告诉我就别猜"的默认。
        """
        if machine_total_w is not None:
            return float(machine_total_w)
        return self.power_w

    def power_share(self) -> float:
        """这条天线分到的份额（``(0, 1]``）。单天线恒为 ``1.0``。

        诊断用：``0.25`` 的意思是"四条天线均分，这条占四分之一"。
        """
        if self.antenna_count <= 0:
            return 1.0
        return 1.0 / self.antenna_count


@dataclass(frozen=True, slots=True)
class Threat:
    """一台干扰机**相对某个受害方**的一份威胁明细（纯数据）。

    受害方拿到它就能算物理，而**不需要知道门面内部**：所有量都是"这台干扰机
    怎么样"与"它相对我在哪"。

    ★ 两个出口共用它：:meth:`JamService.threats`（逐目标：这一刻波束指着某个
    目标时谁从主/副瓣漏进来）与 :meth:`JamService.on_roster`（每拍一次：这一
    拍谁在我视野里）。后者**没有**波束指向可比，于是 ``off_beam_deg`` 为
    ``None``——用 0.0 代替是在说"它就在波束中心"，那是一句假话。
    """

    entity_id: int
    tag: str
    #: 真正参与计算的发射功率（W，见 :meth:`JammerSpec.radiated_power_w`）。
    radiated_power_w: float
    #: 干扰机峰值增益（线性）。
    gain: float
    #: 干扰机内部损耗（线性，**单程**）。与受害方的单程损耗**相乘**。
    internal_loss: float
    frequency_hz: float
    #: 0 = 连续波。
    bandwidth_hz: float
    #: 极化失配的**显式覆盖**（线性）。**负数 = 未给** ⇒ 受害方按两端**类型**
    #: 查表（见 :attr:`JammerSpec.polarization`）。
    polarization: float
    #: 受害方到干扰机的**斜距**（米）。
    slant_m: float
    #: 受害方看干扰机的真实方位（度）。
    bearing_deg: float
    #: 干扰机的**三维位置**（米）。★ 判"山那面挡不挡得住"必须用它——斜距与
    #: 方位描述不了遮挡（同一斜距上，山那头与山这头看起来一模一样）。
    position: tuple[float, float, float]
    #: 与受害方**当前波束指向**的最小夹角（度）。0 = 就在波束中心。
    #: ``None`` = 这次查询没有波束指向可比（``on_roster``）。
    off_beam_deg: float | None
    #: 是否落在主瓣里（由 ``off_beam_deg`` 与主瓣张角判出）。``on_roster``
    #: 那一支没有波束指向 ⇒ 恒 ``False``，且**不要读它**。
    in_main_lobe: bool
    #: 干扰机**天线架高**（米）。0 = 没给 ⇒ 判遮蔽时两端都用受害方的架高。
    #: 见 :attr:`JammerSpec.antenna_height_m`。
    antenna_height_m: float = 0.0
    #: 干扰机的**极化类型**（见 :attr:`JammerSpec.polarization_type`）。原样
    #: 带过来：折算成损耗要用**受害方自己**的主极化，施扰侧算不了（§5.14）。
    polarization_type: str = "default"
    #: 这台干扰机的**欺骗卡**（``None`` = 不搬量测中心）。原样带过来，因为
    #: "骗得成骗不成"要用受害方自己的 ``J/S`` 判，门面判不了（§5.14）。
    deception: DeceptionSpec | None = None
    #: 这台干扰机的**假目标卡**（``None`` = 不放幽灵）。同样原样带过来：
    #: 条数要用受害方自己的 ``PW`` / ``PRI`` / 扫描时间算（§5.14）。
    false_targets: FalseTargetSpec | None = None
    #: 干扰机**天线主瓣全宽**（度，v0.13.34）。**0 = 各向同性** ⇒ 受害方不
    #: 打折（方向图因子恒 1.0），与加这一项之前逐位相同。
    beam_width_deg: float = 0.0
    #: **受害方偏离干扰机天线指向的夹角**（度，v0.13.34）。``None`` = 这台干扰机
    #: 没启用置向（``aim_at`` 为空）⇒ 无方向图可言，受害方按各向同性算。
    #: ★ 这是**几何**（门面给），折算成增益是受害方的物理（§5.14 分工）。
    antenna_off_deg: float | None = None


#: ``_aim_offset_deg`` 的哨兵：**这台对这个受害方不辐射**（置向取不到 / 主瓣
#: 盖不住）。用一个**与任何合法角度都不同**的对象而不是 ``-1.0`` 之类：角度是
#: 实数，拿一个数兼作"没有角度"会在某次边界计算里被当成真角度用下去，而症状
#: 是"天线明明指着别处，J 却算出来了"。
_AIM_BLOCKED = object()

#: ``_eligible`` 的一行：（能力卡, 干扰机位置, 方位, 斜距, 天线偏角）。
#: 末项 ``None`` = 这台没启用置向（无方向图）；非 ``None`` = 受害方偏离干扰机
#: 天线指向的夹角（度），受害方据此折算方向图增益（v0.13.34）。
_Eligible = tuple[
    JammerSpec, tuple[float, float, float], float, float, "float | None"
]



class JamService:
    """干扰源名册 + 逐受害方的威胁明细。

    ``__slots__`` **只放句柄与计数器**，不缓存任何派生量——干扰机中途可以
    被击毁 / 关机 / 换频，缓存下来的"谁能压我"会立刻变成假数据。
    """

    __slots__ = (
        "_store",
        "_registry",
        "_jammers",
        "_tracks_of",
        "queries",
        "threats_seen",
        "aimed",
        "aim_failed",
    )

    def __init__(self, store: Any, registry: Any, tracks_of: Any = None) -> None:
        self._store = store
        self._registry = registry
        #: **"按实体 ID 取它自己那张航迹表"的函数**（v0.13.34 加）。默认
        #: ``None`` ⇒ 照航迹置向**未启用**（与加这一项之前逐字相同，§5.14.11 ④
        #: 的 ⟨可选启用⟩ 口径）。
        #:
        #: ★ 它是**注入**进来的、不是门面直接翻底账：门面属于 ``services``，
        #: 但它拿到的应当只是"某实体的航迹表快照"这一个只读切片，而不是
        #: ``EntityStore`` 的全部能力（真值位置、他人损耗都在那里面）。注入
        #: 让"门面能看什么"由装配层一句话决定，将来换"按战区隔离"也只改这里。
        #:
        #: ★ **正式装配注入 ``store.contacts_of``**（用户裁定 ⟨乙⟩，v0.13.34，
        #: 见 §9-59 与 ``simulation.py``）。原先"只在测试里注入、正式装配不传"
        #: 的口径**已作废**。⇒ 写了 ``aim_at`` 的干扰机在正式想定下**能真辐射**，
        #: 但**前提是它自己那张航迹表里有敌航迹**——现在的表由"谁探测到谁写"
        #: 填：干扰机自己没探测组件、也没人把航迹传给它 ⇒ 表是空的 ⇒ 仍**不辐射**。
        #: 要让"看到才打"真的发生，得先解决"航迹怎么到它手上"（自带探测 /
        #: 己方通信传 / 同方共享，三条候选见 §9-59）。
        #: 表为空时取不到航迹 ⇒ 不辐射——这是"没有航迹就不该照着打"的正确表现。
        self._tracks_of = tracks_of
        #: ``实体 ID →（机器名, 那天线的条目清单）``，即**那一刻这台机器身上还
        #: 在册的天线**。★ 为什么按实体再分一层：一台机器可以挂 N 条天线，而
        #: 它们的**注销是逐条**发生的（组件把自己的 ``JammerSpec`` 交给
        #: ``unregister``）。按实体整批增删的话，先关的那条会把后关的那条一起
        #: 抹掉，症状是"我只关了一条天线，另一条也不压了"（§5.14.4 ④）。
        self._jammers: dict[int, tuple[str, list[JammerSpec]]] = {}
        #: 名册被查过多少次（诊断用）。★ 两个出口都算在里面：逐目标的
        #: :meth:`threats` 与每拍一次的 :meth:`on_roster`——它们共用
        #: ``_eligible``，所以这个计数是"名册被问了几次"，不是"检测了几次"。
        self.queries = 0
        #: 历次查询一共吐出过多少条威胁明细（含进不了带的）。
        self.threats_seen = 0
        #: **照航迹置向**成功过多少次（诊断用，v0.13.34）。每选中一次敌方航迹
        #: 记 1；一个"想瞄却没瞄上"的想定靠它和 :attr:`aim_failed` 就能分辨。
        self.aimed = 0
        #: **照航迹置向失败**过多少次（诊断用，v0.13.34）。三种原因都算：
        #: 取不到航迹表、表里没有敌方条目、``aim_at`` 指定的那条找不到 / 不是敌。
        self.aim_failed = 0

    # -- 名册 --------------------------------------------------------------

    def register(self, spec: JammerSpec) -> None:
        """登记**一条天线**。同一个实体上**属于两台不同机器** ⇒ 当场报错。

        报错那一支的理由（"『这台压不压得到我』有两个答案"）只在"一台机器一个
        固定方向"的前提下成立；一旦拆成多天线，**同一条天线的**答案仍是唯一的
        ——所以判据从"实体"收到"**机器**"这一层：

        * 同一实体、同一台机器（机器名相同）⇒ 这是**多天线**，照收；
        * 同一实体、**机器名不同** ⇒ 那是两台机器挤在一个实体上，报错
          （分不出谁是谁，而两台的参数都是用户自己写的、看起来都对）。

        ★ 机器名是**显式**的一层身份，不从实体属性推：想定可以在一个平台上换装 /
        增装干扰机，那时"实体 ID 相同"是**预期的**，拿它当"同一台机器"会静默地
        把两台拼成一台（§5.14.4 ④）。
        """
        old = self._jammers.get(spec.entity_id)
        if old is not None and old[0] != spec.tag:
            raise ConfigurationError(
                f"实体 {spec.entity_id} 上已有干扰机（{old[0] or '未命名'}），"
                f"又登记了另一台（{spec.tag or '未命名'}）：一个实体上只能有**一台"
                "机器**，否则『这台压不压得到我』有两个答案。"
                "（**多天线**请用同一个型号名登记多条——名册按天线记，同名即同一台）"
            )
        if old is None:
            self._jammers[spec.entity_id] = (spec.tag, [spec])
        else:
            old[1].append(spec)

    def unregister(self, spec_or_id: JammerSpec | int) -> bool:
        """注销**一条天线**。返回"名册里真的有它、并已移除"。

        两种调用形态，**都必须给得出天线身份**：

        * ``unregister(spec)`` —— 组件关机走的这条路。交出的是**登记时那张卡**
          （它同时说明"这是哪台机器、哪条天线"）；
        * ``unregister(entity_id)`` —— **整台机器**下架（该实体上当前在册的
          全部天线）。这是"我把它整台拆了"的动作，不是"删掉某一条"。

        ★ **按位置删必须是安全的那一个**：名册条目上没有"第几条"这种身份，
        用位置删会随插入顺序漂移（先装的被后装的顶掉）。想删**一条**就交回它的
        卡；只给实体 ID 时语义是"这台机器我整个不要了"——这条差别是有意的，
        多天线的注销是逐条发生的（先关的那条不该带走后关的那条）。
        """
        if isinstance(spec_or_id, JammerSpec):
            entry = self._jammers.get(spec_or_id.entity_id)
            if entry is None:
                return False
            name, rows = entry
            if name != spec_or_id.tag:
                return False            # 换了机器 ⇒ 这张卡不属于现在在册的那台
            for i, row in enumerate(rows):
                if row is spec_or_id:
                    del rows[i]
                    break
            else:
                return False
            if not rows:
                del self._jammers[spec_or_id.entity_id]
            return True
        return self._jammers.pop(spec_or_id, None) is not None

    @property
    def is_empty(self) -> bool:
        """名册为空 ⇒ 全仿真不存在干扰。受害侧据此**一步早退**。"""
        return not self._jammers

    def specs(self) -> list[JammerSpec]:
        """全部天线条目，按 ``(实体 ID, 登记顺序)`` 展平。

        **排序是为了可复现**：字典的插入顺序取决于装配顺序，而浮点求和
        **不满足结合律**——换一个顺序，末位就可能变。压制的可复现性靠它。
        天线条目之间保持各自的**登记顺序**（同一台机器的多条天线按装配顺序
        排列），于是"先装的那条先求和"也是确定的。
        """
        return [row for k in sorted(self._jammers) for row in self._jammers[k][1]]

    @property
    def machine_names(self) -> list[str]:
        """在册机器的型号名，按实体 ID 排序（诊断：一实体一行）。"""
        return [self._jammers[k][0] for k in sorted(self._jammers)]

    def antennas_of(self, entity_id: int) -> int:
        """某个实体上**当前**在册的天线条数（0 = 没有它）。"""
        entry = self._jammers.get(entity_id)
        return len(entry[1]) if entry else 0

    def __len__(self) -> int:
        """在册的**天线条数**（不是实体数）。

        ★ 定义成天线条数是因为名册的每一行就是一条天线：加天线 ⇒ 名册变长。
        要数机器用 :meth:`machine_names` / :attr:`_jammers`。
        """
        return sum(len(rows) for _, rows in self._jammers.values())

    # -- 查询 --------------------------------------------------------------

    def threats(
        self,
        victim_id: int,
        *,
        beam_bearing_deg: float,
        main_lobe_deg: float,
    ) -> tuple[Threat, ...]:
        """列出**能影响到 ``victim_id``** 的干扰机，各带一份几何明细。

        ``beam_bearing_deg`` / ``main_lobe_deg`` 是**受害方当前的波束指向与
        主瓣张角**（度、全宽）。方位差 ≤ 一半张角 ⇒ ``in_main_lobe = True``。
        """
        rows = self._eligible(victim_id)
        if not rows:
            return ()
        out: list[Threat] = []
        for spec, other, bearing, slant, antenna_off in rows:
            off = angular_offset_deg(bearing, beam_bearing_deg)
            out.append(
                self._threat(
                    spec,
                    position=other,
                    bearing=bearing,
                    slant=slant,
                    off_beam_deg=off,
                    in_main_lobe=off <= main_lobe_deg * MAIN_LOBE_HALF,
                    antenna_off_deg=antenna_off,
                )
            )
        self.threats_seen += len(out)
        return tuple(out)

    def on_roster(self, victim_id: int) -> tuple[Threat, ...]:
        """**这一拍谁在我视野里**——每拍问一次的那个视图。

        与 :meth:`threats` 是两个不同的问题，所以是两个方法：

        ==================  ==============================================
        ``threats``         逐目标：波束**正指着这个目标**时谁漏进来（算 J、算欺骗）
        ``on_roster``       每拍一次：这拍波束扫过了谁（算假目标）
        ==================  ==============================================

        假目标为什么不能走 ``threats``：它的条数公式里**没有目标**（
        ``(PRI/PW)·(ScanTime/PRI)·N_int·density`` 全是波形量），而它的方位是
        **干扰机自己的方位**、不是某个目标的方位。硬套 ``threats`` 就得给
        ``beam_bearing_deg`` 编一个指向出来，而 ``off_beam_deg`` 会跟着变成
        一个没有含义的数——所以这里明确地回 ``off_beam_deg = None``。

        **"我不问方位"不等于"假目标不看方位"**：受害方仍要判"本拍扫到它没有"
        （见 ``RadarSensor._update_false_targets``），那是它的弧、它的判据。
        """
        rows = self._eligible(victim_id)
        if not rows:
            return ()
        out = tuple(
            self._threat(spec, position=other, bearing=bearing, slant=slant,
                         off_beam_deg=None, in_main_lobe=False,
                         antenna_off_deg=antenna_off)
            for spec, other, bearing, slant, antenna_off in rows
        )
        self.threats_seen += len(out)
        return out

    def _aim_bearing_deg(self, spec: JammerSpec) -> float | None:
        """这台干扰机的**天线指向**（度，真北顺时针）；``None`` = 不辐射。

        ★ 三条判据，全在"**本机航迹表**"里做（§5.14.11 ④）：

        1. **先有航迹才谈置向**。取不到本机航迹表（``tracks_of`` 没注入 /
           实体查不到）⇒ ``None``。这就是"平台中有了敌方航迹，才能对着航迹
           干扰"——不是"看不见也照样压"。
        2. **只瞄 ``iff == foe`` 的航迹**。己方航迹（``friend``）与"还没判"
           （``unknown`` / ``ambiguous`` / ``neutral``）**都不瞄**：拿一条不知道
           是不是敌人的航迹去置向，等于蒙。幽灵航迹（``phantom``）**也不算**
           ——那是干扰自己造/别人造的假目标，照着它打是自欺。
        3. **挑哪一条**：``aim_at`` 为空（但本参数整体启用时）由调用方决定不
           走这里；``"nearest"`` = 斜距最近的那条；否则按**实体名或实体 ID**
           匹配（⟨容错⟩口径，两种都认）。匹配不到 ⇒ ``None``，**不退化成一个
           方向图默认值**——"想瞄 A 结果瞄了 B"比"不辐射"难查得多。

        ★ 位置一律取**航迹里那个** ``(x, y, z)``，**不是** ``position_of`` 的
        真值：航迹有量测误差，天线就会指偏，方向图自然少收增益——这正是用户
        要的"位置有偏差，干扰效果就差一点"。取真值就把这个效应抹掉了。
        """
        if self._tracks_of is None:
            return None
        try:
            tracks = self._tracks_of(spec.entity_id)
        except (KeyError, TypeError):
            return None
        if not tracks:
            return None

        me = self._store.position_of(spec.entity_id)
        if me is None:
            return None

        wanted = spec.aim_at.strip()
        foes = [
            c
            for c in tracks
            if c.iff == IFF_FOE and not c.phantom and c.target_id != spec.entity_id
        ]
        if not foes:
            return None

        chosen = self._pick_aim_target(foes, wanted, me)
        if chosen is None:
            return None

        east, south = chosen.x - me[0], chosen.y - me[1]
        if east == 0.0 and south == 0.0:
            #: 目标航迹与本机同点 ⇒ 方位无定义（与 ``_eligible`` 第 3 条同一
            #: 个理由）。置向拿不到角度，按"没瞄上"处理。
            return None
        return bearing_of_offset(east, south)

    def _pick_aim_target(
        self,
        foes: list[Contact],
        wanted: str,
        me: tuple[float, float, float],
    ) -> Contact | None:
        """从**敌方航迹**里挑一条置向对象；挑不到返 ``None``。

        ``wanted`` 的三种形态（⟨容错⟩口径：实体名与 ID 都认）：

        * ``""`` 或 ``"nearest"`` ⇒ 斜距**最近**的那条（用户原话："如果没有
          指定，则瞄准最近的"）；
        * 其它 ⇒ 当作**实体名**，过 :meth:`EntityRegistry.by_name` 解析成实体再
          取它的 ID 比对。
        """
        if not wanted or wanted.lower() == "nearest":
            return min(foes, key=lambda c: self._track_slant(c, me))

        target_id: int | None = None
        if wanted.lstrip("-").isdigit():
            target_id = int(wanted)
        else:
            resolved = self._registry.by_name(wanted)
            if resolved is not None:
                target_id = resolved.entity_id
        if target_id is None:
            return None
        for contact in foes:
            if contact.target_id == target_id:
                return contact
        return None

    @staticmethod
    def _track_slant(contact: Contact, me: tuple[float, float, float]) -> float:
        """航迹到本机的三维距离（米）——用来挑"最近的"。"""
        dx = contact.x - me[0]
        dy = contact.y - me[1]
        dz = contact.z - me[2]
        return sqrt(dx * dx + dy * dy + dz * dz)

    def _eligible(self, victim_id: int) -> tuple[_Eligible, ...]:
        """名册里**够得着 ``victim_id``** 的干扰机，各带（卡、位置、方位、斜距）。

        三道筛（对应 AFSIM 那句 "if there is **in-band** power"）：

        1. **不是自己**——一台干扰机不压自己（真实的自卫干扰机确有自扰，
           那是"收发隔离"问题，属另一个模型）；
        2. **同阵营不打**（``ignore_same_side``）；
        3. **两台机不可以同处一点**——斜距为零时 :func:`~milsim.models.percep.
           equation.jamming_power` 会拒绝（距离必须为正），而这里**跳过**它
           比让它抛异常更合适：一个位置还没初始化的实体不该把整场推演带崩。

        ★ 第四道筛（v0.13.34，仅当这台写了 ``aim_at`` 且 ``beam_width_deg > 0``
        时生效）：**天线主瓣盖不盖得住受害方**。写了 ``aim_at`` 的干扰机不再
        各向同性——它的天线指着**本机航迹表里挑出的那条敌航迹**（
        :meth:`_aim_bearing_deg`），那么"盖不盖得住我这个受害方"就是一道真实
        的判据：盖不住 ⇒ 这门对**我**没贡献。**两个出口共用**它，否则假目标
        与压制的可见性会分叉。未写 ``aim_at``（或 ``beam_width_deg == 0``）⇒
        这道筛**逐字不参与**，与旧想定完全一致。

        ★ 这四行筛**只写一遍**：``threats`` 与 ``on_roster`` 共用它。抄第二份
        的话，"同阵营不打"这种判据迟早只改一处，而症状是"两个视图对不上"。

        ★ **不在这里筛频段**（虽然 AFSIM 的 "in-band" 说的是频段）：``F_BW``
        要两份频段，而"我的调谐带"是受害方的参数。筛它就是把受害方的调谐带
        抄一份进门面——两份迟早不同步。受害方拿到 :class:`Threat` 之后自己算，
        算出来 0 就是进不了带。
        """
        if self.is_empty:
            return ()

        me = self._store.position_of(victim_id)
        if me is None:
            return ()
        self.queries += 1

        my_side = self._registry.side_of(victim_id)
        rows: list[_Eligible] = []
        for spec in self.specs():
            if spec.entity_id == victim_id:
                continue
            if (
                spec.ignore_same_side
                and my_side
                and self._registry.side_of(spec.entity_id) == my_side
            ):
                continue
            other = self._store.position_of(spec.entity_id)
            if other is None:
                continue
            east, south = other[0] - me[0], other[1] - me[1]
            if east == 0.0 and south == 0.0:
                continue
            off = self._aim_offset_deg(spec, me, other)
            if off is _AIM_BLOCKED:
                continue
            rows.append(
                (
                    spec,
                    other,
                    bearing_of_offset(east, south),
                    sqrt(east * east + south * south + (other[2] - me[2]) ** 2),
                    off,
                )
            )
        return tuple(rows)

    def _aim_offset_deg(
        self,
        spec: JammerSpec,
        victim_pos: tuple[float, float, float],
        jammer_pos: tuple[float, float, float],
    ) -> float | None:
        """受害方偏离干扰机**天线指向**的夹角（度）；不参与置向时返 ``None``。

        返回 :data:`_AIM_BLOCKED` 表示**这台对当前受害方不辐射**（第四道筛）。

        ★ 三种情形，按"启用到什么程度"分：

        * ``aim_at`` 为空 ⇒ **旧行为**：不查航迹、不看朝向，返 ``None``
          （受害方据此按各向同性算，想定零回归）。
        * 置向取不到（没航迹表 / 没敌航迹 / 挑不中）⇒ :data:`_AIM_BLOCKED`：
          **不辐射**。这是用户要的口径——"看见才打"，而不是"看不见就按各向
          同性兜底"。
        * 置向取到 ⇒ 返夹角。**宽度为 0** 时这一支仍返偏角（置向生效、方向图
          不生效），由受害方决定折不折——但为了让"盖不住就不参与"这条能落地，
          这里仍要判一次主瓣（见下）。

        ★ 主瓣是**全宽**，与受害方判"是否落在我主瓣里"同一个约定
        （:data:`MAIN_LOBE_HALF`）。**盖不住 ⇒ 这门对当前受害方没有贡献**：
        这一条放在门面而不是受害方，是因为它要用"干扰机天线指向"这个只有门面
        拿得到的量（受害方那边拿到的已经是 :class:`Threat` 了）。
        """
        if not spec.aim_at:
            return None
        bearing = self._aim_bearing_deg(spec)
        if bearing is None:
            self.aim_failed += 1
            return _AIM_BLOCKED
        self.aimed += 1
        to_victim = bearing_of_offset(
            victim_pos[0] - jammer_pos[0], victim_pos[1] - jammer_pos[1]
        )
        off = angular_offset_deg(to_victim, bearing)
        if spec.beam_width_deg > 0.0 and off > spec.beam_width_deg * MAIN_LOBE_HALF:
            return _AIM_BLOCKED
        return off

    @staticmethod
    def _threat(
        spec: JammerSpec,
        *,
        position: tuple[float, float, float],
        bearing: float,
        slant: float,
        off_beam_deg: float | None,
        in_main_lobe: bool,
        antenna_off_deg: float | None,
    ) -> Threat:
        """把一条名册条目折成 :class:`Threat`（两个出口共用一份字段映射）。"""
        return Threat(
            entity_id=spec.entity_id,
            tag=spec.tag,
            radiated_power_w=spec.radiated_power_w(),
            gain=spec.gain,
            internal_loss=spec.internal_loss,
            antenna_height_m=spec.antenna_height_m,
            frequency_hz=spec.frequency_hz,
            bandwidth_hz=spec.bandwidth_hz,
            polarization=spec.polarization,
            polarization_type=spec.polarization_type,
            slant_m=slant,
            bearing_deg=bearing,
            position=position,
            off_beam_deg=off_beam_deg,
            in_main_lobe=in_main_lobe,
            beam_width_deg=spec.beam_width_deg,
            antenna_off_deg=antenna_off_deg,
            deception=spec.deception,
            false_targets=spec.false_targets,
        )

    # -- 诊断 --------------------------------------------------------------

    def statistics(self) -> dict[str, Any]:
        return {
            #: ★ 两个数都报：``jammers`` 是**天线条数**（名册的行数，多天线时
            #: 大于机器数），``machines`` 是**实体数**。只报一个的话，"我加了
            #: 一条天线"与"我加了一台机器"在诊断里长得一样。
            "jammers": len(self),
            "machines": len(self._jammers),
            "queries": self.queries,
            "threats": self.threats_seen,
        }

    def __repr__(self) -> str:
        return f"<JamService {len(self._jammers)} 台 / {len(self)} 条天线>"


__all__ = [
    "DEFAULT_SIDELOBE_LEVEL_DB",
    "MAIN_LOBE_HALF",
    "DeceptionSpec",
    "FalseTargetSpec",
    "JamService",
    "JammerSpec",
    "Threat",
    "angular_offset_deg",
    "radiated_power",
]
