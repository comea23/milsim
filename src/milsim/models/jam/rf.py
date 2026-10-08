"""射频干扰机：**唯一**的干扰机实现，注册名 ``RF_JAMMER``。

对应 AFSIM 的 ``WSF_RF_JAMMER``（一个 ``weapon`` 部件、自带 ``transmitter``）。
这个文件**一行算法都没有**——三种效果的全部物理都在别处：方程在
``models/percep/equation``，名册与几何在 ``services/ew``，后果在受害方的
探测链上。它存在的意义是**族约定**（§5.7）：一个实现一个文件、清单只在子包里
定义一处。

★ 一个组件，三种效果（v0.13.30）
--------------------------------

======================  ==============================  ==========================
参数组                   改了哪一个量                    受害方看到什么
======================  ==============================  ==========================
**核心**（必给）          ``N`` ⇒ ``S/(N+J)``             真目标**更难建航**
``deception_*``          量测的**中心**                   航迹**建得起来，但是错的**
``false_target_*``       航迹表里添**幽灵**、占满槽位     真目标**被挤出去 / 被淹没**
======================  ==============================  ==========================

三者**同时生效**。**频率与谱宽共用一份**（``frequency`` / ``bandwidth``：三条效果
都得落在同一个带里，才进得去对方的接收机），而**功率可以分成两份**（v0.13.31 起：
``deception_peak_power`` / ``deception_duty_cycle`` 不给就与压制共用）——AFSIM 的
``WSF_RF_JAMMER`` 正是一个部件挂**多个 ``transmitter``**，每个 ``effect`` 用自己那台。

"不按效果拆成三个组件类型"仍然成立，而且理由更硬：拆开之后每一型都得把核心那张
参数表再抄一份（同一份频率两个出处 ⇒ 迟早不同步），而门面又规定"一个实体只能有
一台干扰机"，于是"同时压制 + 欺骗"在结构上就写不出来了。

★ 用户问的两个维度都压在核心表里，**不是二选一**
------------------------------------------------

**频域（宽窄）** —— ``bandwidth`` 与 ``frequency``，成果是 ``F_BW``：

* **阻塞式**（``bandwidth`` 远大于受害方带宽）⇒ ``F_BW = B_r/B_j``，
  功率摊在整个宽带上，只有一份进得去（窄 10 倍 = −10 dB）；
* **瞄准式**（``bandwidth`` 落在受害方带内）⇒ ``F_BW = 1``，一点不吃亏；
* **完全错开**（``Δf`` 超过两者半宽之和）⇒ ``F_BW = 0``，**一分钱都不进**。
  跳频抗干扰能生效，靠的就是把这一项压到 0。

**时域（连续 / 脉冲）** —— ``duty_cycle``：

* ``1`` ⇒ 连续波，平均功率 = 峰值功率；
* ``0.01`` ⇒ 脉冲干扰，**默认档下直接进 J**（−20 dB）。依据是 AFSIM 源码而不是
  文档，见 :meth:`~milsim.services.ew.JammerSpec.radiated_power_w` 的推导；
* 若这型干扰机是**峰值功率受限**的，写 ``use_peak_power true``，那时占空比
  **不进** J。

两个维度是正交的：`宽频阻塞 + 连续波`、`窄带瞄准 + 脉冲` 都是合法组合，而它们
在探测链上的表现完全不同（前者处处 −10 dB，后者是"能烧穿就烧穿"）。

★ 两张卡都是**可选**的，判据是"这组量有没有非零值"
--------------------------------------------------

* 欺骗卡：``deception_range_bias_m`` / ``deception_azimuth_bias_deg`` /
  ``deception_walkoff_rate_mps`` **三个全 0 ⇒ 没有这张卡**。理由：一张什么都
  不搬的卡没有任何可观察后果，让它存在只会让"这台到底骗不骗"多一个出口。
  （``deception_recycle`` 单独给不算数——它是拖引的修饰，没有拖引就没有它。）
* 假目标卡：``false_target_pulse_density`` 与 ``false_target_quantity``
  **两个都给 ⇒ 装配期报错**（AFSIM 原文 "mutually exclusive"）；
  **两个都是 0 ⇒ 没有这张卡**。

★ 假目标**看数量、也看到达功率**（§9-51 结案）
---------------------------------------------

AFSIM 的 ``WSF_SIMPLE_FT_EFFECT`` 只有三样输入：``jamming_pulse_density`` /
``maximum_false_target_capacity`` / 一个随机抽签开关——**没有功率门限**。于是只要
参数表上给了密度，一台功率小得可怜（或者远得没用）的干扰机也能把屏幕撒满。

★ 门限加在**受害方**那一边（``RadarSensor.false_target_min_power``，默认 0 =
不判）：它的语义是"检测门限折算到接收机输入端的等效功率"，属于**雷达**的属性。
写在受害方而不是干扰机上，就不存在"两台互相猜对方灵敏度"的问题。
★ 门限判的是**到达功率**而不是 ``J/S``：铺幽灵那条路（每拍一次）**没有单脉冲
``SNR``**，用 ``J/S`` 会让同一个干扰机在两个视图里得到两个答案。

★ 一张卡上**没有**的东西（§9-52）
--------------------------------

* 方位拖引（AGPO）只做静态偏置，没有"随时间拉走"的那一项；
* 假目标不做 ``persistence`` / ``scan_rate`` / ``random_scan_to_scan`` 的随机化、
  不做 ``speeds``（多普勒）与 ``distribution`` 的分布形状；
* 极化**没有**"逐类型覆盖表"（AFSIM 的 ``polarization_effect`` 可以一条条覆盖）——
  我们只给两个类型参数 + 一个全局的显式覆盖值。

参数表上不出现它们，也就不会有人写了它却没生效。
"""

from __future__ import annotations

from ...errors import ConfigurationError
from ...services.ew import DeceptionSpec, FalseTargetSpec, radiated_power
from ...services.params import Params
from ...services.type_registry import register_component
from .base import Jammer


@register_component("RF_JAMMER")
class RfJammer(Jammer):
    """功率 + 三张效果卡的干扰机。

    核心（压制）那一张表在基类里，这里只加**两组可选卡**——
    "换一种干扰机"在本项目里就是"换一张表"，而这三组量属于同一台机器。
    """

    PARAMS = {
        #: 压制的**核心**那张表（功率 / 频率 / 谱宽 / 占空比…）住在基类里，
        #: 这里用 ``**`` 合并进来——类继承给的合并（§5.2 ①）。**不是**把核心
        #: 抄一份：抄一份就会出现"同一份功率两个出处"，而改了一处不生效时
        #: 参数表上两处都看着对。
        **Jammer.PARAMS,
        # -- 欺骗链的**功率**（可选，v0.13.31）--------------------------------
        #: 欺骗链的**峰值**功率（W）。**0 = 与压制共用核心表的 ``peak_power``**
        #: （默认 ⇒ 旧想定数字一个不动）。
        #: ★ 为什么要有它：AFSIM 的 ``WSF_RF_JAMMER`` 是一个部件挂**多个
        #: ``transmitter``**，每个 ``effect`` 用自己那台。不给这一项时两条链
        #: 就是同一台发射机；给了就分开了，于是"**只骗不压**"（欺骗链功率足、
        #: 压制链功率小）才是一句能表达的话。在此之前它表达不出来（§9-50）。
        "deception_peak_power": Params.power(0.0, minimum=0.0),
        #: 欺骗链的**占空比**，``(0, 1]``。**0 = 与压制共用核心表的 ``duty_cycle``**。
        #: ★ "峰值还是平均"那条规则两链共用（``ew.radiated_power`` 只有一份实现），
        #: 所以这里给的是**峰值**，不是已经乘过占空比的数。
        "deception_duty_cycle": Params.number(0.0, minimum=0.0, maximum=1.0),
        # -- 欺骗卡（可选）--------------------------------------------------
        #: 生效门限（dB）：``J/S`` 到这个值以上才产生偏置。默认 3 dB（J = 2·S）。
        "deception_required_j_to_s_db": Params.number(3.0, minimum=0.0),
        #: 距离偏置（米）。正 = 把目标**报得更远**。静态的那一份。
        "deception_range_bias_m": Params.distance(0.0),
        #: 方位偏置（度）。正 = 顺时针偏。
        "deception_azimuth_bias_deg": Params.angle(0.0),
        #: 距离拖引速率（米/秒，RGPO）。0 = 不拖引。符号决定往哪边拉。
        "deception_walkoff_rate_mps": Params.speed(0.0),
        #: 拖引封顶（米）。**0 = 不封顶**（同本项目的"0 = 不限制"约定）。
        #: ★ 取 0 而 ``deception_walkoff_rate_mps`` 不为 0 时偏置会一直涨下去
        #: ——这是**合法配置**（真雷达的跟踪环路自己会跟丢），不是"没给"。
        "deception_holdout_m": Params.distance(0.0, minimum=0.0),
        #: 到顶之后重新开始拉。只有效于"封了顶"的情形。
        "deception_recycle": Params.boolean(False),
        # -- 假目标卡（可选）------------------------------------------------
        #: 脉冲密度，``[0, 1]``。**0 = 没有这张卡**（AFSIM 的默认是 0.1，但本项目
        #: 的默认值必须让"没写它"的行为逐字不变）。与 ``quantity`` 互斥。
        "false_target_pulse_density": Params.number(
            0.0, minimum=0.0, maximum=1.0
        ),
        #: 固定条数（AFSIM 的 ``number_of_false_targets``）。0 = 按密度算。
        #: 上限由受害方的 ``false_target_capacity`` 兜底，这里不设——容量是
        #: **受害方**的能力，不是干扰机的。
        "false_target_quantity": Params.integer(0, minimum=0),
    }

    __slots__ = ("deception", "false_targets")

    def deception_card(self) -> DeceptionSpec | None:
        """造欺骗卡；三个位移量全 0 ⇒ 没有这张卡（返回 ``None``）。

        ★ **"给了欺骗链的功率、却没给任何位移量"当场报错**：那张卡不存在，
        于是那两个功率参数**永远不会被任何一处读到**——正是本项目最想消灭的
        "参数表上有这一项却没人读"。静默忽略的症状是"我调了欺骗功率，
        一点反应都没有"，而参数表上两个值都写着。
        """
        bias = float(self.spec["deception_range_bias_m"])
        azimuth = float(self.spec["deception_azimuth_bias_deg"])
        rate = float(self.spec["deception_walkoff_rate_mps"])
        name = self.type_name or type(self).__name__
        peak = float(self.spec["deception_peak_power"])
        duty = float(self.spec["deception_duty_cycle"])
        if bias == 0.0 and azimuth == 0.0 and rate == 0.0:
            orphans = [
                param
                for param, value in (
                    ("deception_peak_power", peak),
                    ("deception_duty_cycle", duty),
                )
                if value > 0.0
            ]
            if orphans:
                raise ConfigurationError(
                    f"{name}: 给了 {'、'.join(orphans)}，但三个位移量"
                    "（deception_range_bias_m / deception_azimuth_bias_deg / "
                    "deception_walkoff_rate_mps）全是 0 ⇒ **没有欺骗卡**，"
                    "这两个功率参数不会被任何地方读到。一条「只改功率、不改偏置」"
                    "的欺骗链没有意义——「骗成什么样」是位移量说的"
                )
            return None
        # ★ 两条链的**功率**在这一步定死：都不给 ⇒ ``power_w = 0`` ⇒ 受害侧
        # 用压制那一份（旧行为）；给了就用自己的那一份。"峰值还是平均"那条
        # 规则走 ``radiated_power``（与压制链同一份实现）。
        power = 0.0
        if peak > 0.0 or duty > 0.0:
            power = radiated_power(
                peak if peak > 0.0 else float(self.spec["peak_power"]),
                duty if duty > 0.0 else float(self.spec["duty_cycle"]),
                bool(self.spec["use_peak_power"]),
            )
        self.deception = DeceptionSpec(
            required_j_to_s_db=float(self.spec["deception_required_j_to_s_db"]),
            range_bias_m=bias,
            azimuth_bias_deg=azimuth,
            walkoff_rate_mps=rate,
            holdout_m=float(self.spec["deception_holdout_m"]),
            recycle=bool(self.spec["deception_recycle"]),
            power_w=power,
            tag=name,
        )
        return self.deception

    def false_target_card(self) -> FalseTargetSpec | None:
        """造假目标卡。

        ★ **互斥要在这里拦**（AFSIM 原文 "mutually exclusive"）：两个都给时
        "听谁的"没有第二个判据，静默取其一的话，症状是"我改的那个参数没反应"
        ——而参数表上两个值都写着，看起来都对。
        """
        density = float(self.spec["false_target_pulse_density"])
        quantity = int(self.spec["false_target_quantity"])
        name = self.type_name or type(self).__name__
        if density > 0.0 and quantity > 0:
            raise ConfigurationError(
                f"{name}: false_target_pulse_density（{density:g}）与 "
                f"false_target_quantity（{quantity}）只能给一个——AFSIM 的原文是 "
                "'mutually exclusive'。两个都给时条数有两个出处，静默取其一"
                "会让另一个参数改了没反应"
            )
        if density <= 0.0 and quantity <= 0:
            return None
        self.false_targets = FalseTargetSpec(
            pulse_density=density,
            quantity=quantity,
            tag=name,
        )
        return self.false_targets

    def describe(self) -> str:
        """``型号(槽 功率 @ 频率)`` + 挂了哪几张卡。**未 ``initialize`` 时只给前半段。**

        ★ 两个卡属性用 ``getattr`` 取：**初始化失败的部件也会被 ``describe``**
        （工厂构建失败时的报错信息里就带着它），而那时两个属性根本没写过。
        直接取会变成一个与"参数写错"毫无关系的 ``AttributeError``。
        """
        base = super().describe()
        if not hasattr(self, "power_w"):
            return base
        card = getattr(self, "deception", None)
        if card is not None:
            base += (
                f" 欺骗 {card.range_bias_m / 1e3:+g} km / "
                f"{card.azimuth_bias_deg:+g}°"
                + (
                    f" 拖引 {card.walkoff_rate_mps:+g} m/s"
                    if card.walkoff_rate_mps
                    else ""
                )
            )
        ft = getattr(self, "false_targets", None)
        if ft is not None:
            how = (
                f"{ft.quantity} 条"
                if ft.quantity > 0
                else f"密度 {ft.pulse_density:g}"
            )
            base += f" 假目标 {how}"
        return base


__all__ = ["RfJammer"]
