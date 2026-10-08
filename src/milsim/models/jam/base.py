"""干扰机族：**施扰侧**。

★ 干扰机不是一个流程，是一台发射机
-----------------------------------

AFSIM 把它做成 ``WSF_RF_JAMMER`` —— 一个 ``weapon`` 部件，**自带 ``transmitter``**。
它声明"我有多大的功率、多宽的谱、朝哪打"，然后**什么都不做**。

求值发生在**受害接收机**身上：AFSIM 原文是 "Jamming calculations take place
**at the time a radar detection or communication attempt occurs**"，而且 EW 流程图
的第一跳就是 ``Check for EA data on Txer? → NO → Return``（没挂 EA 的发射机
**零开销**返回）。

所以本族组件：

* **没有 ``update``** —— 不求值、不推进、不需要节拍；
* 在 ``initialize()`` 里**登记一次**（造一个 :class:`~milsim.services.ew.JammerSpec`
  交给电子战门面），这也是它唯一做的事；
* ``shutdown()`` 里注销 —— 关机 / 被击毁靠"登记与注销"表达，不靠时间窗。

这一条决定了"干扰不在探测流程里"：探测链上**不新增步骤**，只是第 6 步那个
求值点从"只有热噪声 ``N``"变成"热噪声 ``N`` + 干扰 ``J``"，而 ``J`` 是别人送来的。

★ 一台干扰机 = 一个组件，三种效果（v0.13.30）
--------------------------------------------

压制、欺骗、假目标是**同一台机器能同时具备**的三种效果，所以它们不是三种互斥
的组件类型，而是**同一张参数表里的三组**（见 ``models/jam/rf.py``）：

* **压制**是核心（功率 / 频率 / 谱宽 / 占空比）——不给就装配期报错；
* **欺骗**与**假目标**各是一组**可选卡**，不写就是不产生那个效果。

为什么不拆成三个组件类型：三张卡**共用同一张核心表**（频率 / 谱宽 / 极化类型…
以及默认情况下的功率）。按类型拆开，每一型都得把核心那张表再抄一份（"同一个
频率有两个出处"，实测踩过），而门面又规定**一个实体只能有一台干扰机**——于是
"一台机器同时压制 + 欺骗"在结构上就**表达不出来**了。AFSIM 的口径也是并列的：
一个 ``electronic_attack technique`` 里可以同时列出多个 ``effect``，而它们各自
可以指向同一台机器上不同的 ``transmitter``。

★ **频率/谱宽是共用的一份，功率可以分成两份**（v0.13.31）：三条效果都得落在同一个
带里才进得去对方的接收机，所以谱不可能各写一份；而功率是两条发射链各自的事，
``deception_peak_power`` / ``deception_duty_cycle`` 给了就把欺骗链分开。
不给 ⇒ 与压制共用（旧行为）。

导入约定的一处例外
------------------

族约定是"子包内实现只 import ``base``、不 import 彼此"（§5.7）。本文件
破了半条：它 import 了 ``..percep.equation`` 的 ``linear``。理由是这个模块
**不是实现**，是一份纯数学工具（只 import ``math``），而 dB 换算在
"同一族各写一份"与"重复一个常数"之间没有第三条路——本项目的规矩是
**"能算的数别另写一个数"**。换成在这里手写 ``10 ** (db / 10)`` 的话，
dB 的定义就有了第二个出处。
"""

from __future__ import annotations

from ...errors import ConfigurationError
from ...services.ew import DeceptionSpec, FalseTargetSpec, JammerSpec
from ...services.params import Params
from ..component import Component
from ..percep import equation as eq


class Jammer(Component):
    """干扰机族的公共部分：**发射机侧**的参数与登记。

    它**不是**可直接注册的实现（族约定：基类不注册，见 ``models/mover/base.py``）。
    """

    SLOT_HINT = "jammer"

    PARAMS = {
        #: **峰值**功率（W）。想定里可写 ``100 kW``。0 = 没给 ⇒ 装配期报错：
        #: 一台没写功率的干扰机不是"不干扰"，是"参数没写全"。
        "peak_power": Params.power(0.0, minimum=0.0),
        #: 干扰机谱**中心**频率（Hz）。0 = 没给 ⇒ 装配期报错。
        #: ★ 它与受害方的调谐频率是两个数，两者的关系决定 ``F_BW``：
        #: 对上⇒进得去、错开⇒一分不进（``bandwidth_overlap_ratio``）。
        "frequency": Params.frequency(0.0, minimum=0.0),
        #: 干扰机谱宽（Hz）。**0 = 连续波**（单频点），这是默认档。
        #: ★ 宽窄是**频域**维度：``B_j`` 远大于受害方带宽 = 阻塞式，
        #: ``F_BW`` 会被打成 ``B_r/B_j``（窄 10 倍就 −10 dB）。
        "bandwidth": Params.frequency(0.0, minimum=0.0),
        #: 峰值增益（dB）。**默认 0 dBi = 各向同性**——这是一个**合法取值**，
        #: 不是"没给"（同 §9-42 那条判据的反面：0 落在指数上才是缺数据，
        #: 落在"就是这个值"的位置上就是真的 0）。
        #: 要"对着目标打"就写型号的实际峰值增益，例如 ``30``。
        "antenna_gain_db": Params.number(0.0),
        #: 干扰机天线**主瓣全宽**（度，v0.13.34 加）。
        #: ★ **0（默认）= 各向同性 = 旧行为逐字不变**——写 0 时本参数一个数都
        #: 不参与计算（方向图因子恒为 1.0），与加这一项之前逐位相同。
        #: ★ **只认显式写的值，不从 ``antenna_gain_db`` 反推**（§5.14.11 ④
        #: 的 ⟨丙⟩ 口径）：写了 30 dBi 不等于有方向图——增益是"峰值多高"，
        #: 主瓣多宽是另一个独立的物理量，反推出来的宽度是拿一个假定当已知。
        #: >0 才启用连续方向图：主瓣内按偏向角平滑衰减（边缘降到 −3 dB），
        #: 主瓣外**平坦取** ``DEFAULT_SIDELOBE_LEVEL_DB``（−13 dB）。
        "beam_width_deg": Params.angle(0.0, minimum=0.0),
        #: **照哪条航迹置向**（v0.13.34 加）。空串（默认）= 不启用照航迹。
        #: ★ 写了它才走"照敌航迹置向"这套新逻辑（⟨可选启用⟩口径，§5.14.11 ④）：
        #: 本机航迹表里挑 ``iff == foe`` 的条目，``"nearest"`` = 挑斜距最近的，
        #: 也可以写**实体名**或**实体 ID**（两种都认）。挑不到 / 挑中的不是敌
        #: ⇒ **这台不辐射**（不静默退化成一个假的方向图）。
        #: ★ 不写 ⇒ **逐字走旧逻辑**（不看朝向、不查航迹），老想定零回归。
        "aim_at": Params.string(""),
        #: 占空比，``(0, 1]``。1 = 连续波。
        #: ★ 宽窄是**频域**、它才是**时域**：占空比 < 1 就是脉冲干扰。
        #: 默认档下它**进 J**（AFSIM 源码 ``mUsePeakPower = false`` ⇒ 用平均
        #: 功率），1%（−20 dB）的脉冲干扰就是这个量写出来的。
        "duty_cycle": Params.number(1.0, minimum=0.0, maximum=1.0),
        #: 改按**峰值**功率算（= AFSIM 的 ``transmitter.use_peak_power``）。
        #: 默认 ``false`` ⇒ 用 ``峰值 × 占空比`` = 平均功率。
        #: 只有"峰值功率受限"的干扰机才写 ``true``，那时占空比不进 J。
        "use_peak_power": Params.boolean(False),
        #: 干扰机内部损耗（dB，**单程**）。0 dB = 不记损耗。
        #: ★ 它和受害方的损耗是**相乘**关系，不是二选一：两台设备各有各的。
        "internal_loss_db": Params.number(0.0, minimum=0.0),
        #: 干扰机**天线架高**（米），v0.13.33 加。
        #: ★ 它只影响**地形遮蔽判据**（山那面 / 地平线），**不进链路预算**
        #: ——干扰机在三维位置里的 z 已经进了斜距，这一项说的是"天线架多高"。
        #: ``0``（默认）= 没给 ⇒ 判遮蔽时两端都用**受害方**的架高（旧口径，
        #: 逐字不变）。要表达"我架在山上/塔上压你"就写一个正数。
        "antenna_height": Params.distance(0.0, minimum=0.0),
        #: 本机的**极化类型**（AFSIM 的同名参数）。七个取值：``horizontal`` /
        #: ``vertical`` / ``slant_45`` / ``slant_135`` / ``left_circular`` /
        #: ``right_circular`` / ``default``。默认 ``default`` = **没声明极化**
        #: （这也是 AFSIM 的默认值）⇒ 不产生失配。
        #: ★ 损耗 = 两端的类型一起查那张 7×7 表（``equation.polarization_effect``），
        #: 表在方程层**只有一份**；这里只写"我这台朝哪个方向极化"。
        "polarization": Params.string("default", choices=eq.POLARIZATION_KINDS),
        #: 极化失配的**显式覆盖**（线性比例）。这就是 AFSIM 接收机那条
        #: ``polarization_effect <pol> <fraction>`` 的对应物——它**也只做覆盖**
        #: （源码：被显式声明的项压过默认表）。
        #: **-1 = 未给**（默认）⇒ 按两端类型查表。
        #: ★ 为什么"没给"不用 1.0 表示：1.0 的语义是"永远不扣"，与"按类型查表"
        #: 是两件不同的事；用同一个数兼作两义就再也分不开了。
        "polarization_mismatch": Params.number(-1.0, minimum=-1.0, maximum=1.0),
        #: 同阵营不打（同 AFSIM 的 ``ignore_same_side``）。
        "ignore_same_side": Params.boolean(True),
    }

    __slots__ = (
        "service",
        "jammer_spec",
        "power_w",
        "total_power_w",
        "gain",
        "frequency_hz",
    )

    def initialize(self, mount) -> None:
        """登记进电子战门面。**这是干扰机做的唯一一件事。**

        ★ **天线：一个部件实例 = 一条天线**（v0.13.33，§5.14.11 ③）。
        同一台平台上挂 N 个同名干扰机部件，就是同一台机器的 N 条天线：它们
        **同名**（⇒ 门面认作同一台机器，不报错），各交回自己的那张卡
        （⇒ 注销逐条发生，先关的那条带不走后关的那条）。

        天线条数写在**槽名**尾部的裸数字上（``jammer2`` ⇒ 2，想定里
        ``component jammer2 RF_JAMMER``）。它只用来算**每条的默认功率份额**
        （总功率 ÷ 条数）；每条天线自己写 ``peak_power`` 就压过这个默认。

        ★ 半给不行：写了功率没写频率、或者反过来，**装配期直接报错**。
        一台"半给"的干扰机在求值时会算出 0 或无穷，而症状是"干扰好像不生效"、
        参数表看起来却处处有值。
        """
        self.service = mount.jam_service()
        if self.service is None:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 本次装配没有提供电子战"
                "门面，干扰机无处登记——这是装配层的缺陷，不是参数问题"
            )

        name = self.type_name or type(self).__name__
        count = self._antenna_count()
        total_power = float(self.spec["peak_power"])
        #: 这台机器（= 本平台上的同名部件）的**总额定功率**：份额是由它除出来的，
        #: 所以这里存的是**除之前**那个数。名册给不了它（按天线登记 ⇒ 单条推不
        #: 出总量），诊断与验收才拿得回来。
        self.total_power_w = total_power
        self.power_w = self._share_power(total_power, count)
        self.frequency_hz = float(self.spec["frequency"])
        self.gain = eq.linear(float(self.spec["antenna_gain_db"]))
        duty = float(self.spec["duty_cycle"])

        missing = []
        if self.power_w <= 0.0:
            missing.append("peak_power")
        if self.frequency_hz <= 0.0:
            missing.append("frequency")
        if missing:
            raise ConfigurationError(
                f"{name}: 干扰机缺 {'、'.join(missing)}——"
                "一台没有发射功率或没有工作频率的干扰机不是『不干扰』，"
                "是参数没写全（半给会在求值时算出一个看着合理的假数）"
            )
        if duty <= 0.0:
            raise ConfigurationError(
                f"{name}: duty_cycle = 0 表示这台干扰机不发射。"
                "连续波写 1.0；要表达『关机』请注销它，不要用占空比"
            )

        self.jammer_spec = JammerSpec(
            entity_id=mount.entity_id,
            power_w=self.power_w,
            gain=self.gain,
            frequency_hz=self.frequency_hz,
            antenna_count=count,
            bandwidth_hz=float(self.spec["bandwidth"]),
            duty_cycle=duty,
            use_peak_power=bool(self.spec["use_peak_power"]),
            internal_loss=eq.linear(float(self.spec["internal_loss_db"])),
            antenna_height_m=float(self.spec["antenna_height"]),
            polarization=float(self.spec["polarization_mismatch"]),
            polarization_type=str(self.spec["polarization"]),
            ignore_same_side=bool(self.spec["ignore_same_side"]),
            deception=self.deception_card(),
            false_targets=self.false_target_card(),
            beam_width_deg=float(self.spec["beam_width_deg"]),
            aim_at=str(self.spec["aim_at"]).strip(),
            tag=name,
        )
        self.service.register(self.jammer_spec)

    def _antenna_count(self) -> int:
        """这台机器几条天线 = 槽名尾部的裸数字（没有数字 ⇒ 1）。

        ``"jammer2"`` ⇒ 2、``"jammer"`` ⇒ 1、``"jammer_a"`` ⇒ 1。纯字符串解析，
        不去认数字开头那类命名（如 ``"2nd_jammer"``）——取不到尾数就是 1。
        """
        digits = ""
        for char in reversed(self.slot):
            if not char.isdigit():
                break
            digits = char + digits
        return int(digits) if digits else 1

    def _share_power(self, total_w: float, count: int) -> float:
        """这条天线的**默认**功率 = 总功率 ÷ 条数。

        ★ 单天线（``count == 1``）走的正是恒等：``total / 1 == total``，
        所以旧想定那个数**逐位不变**——它不是"特判出来的"，是除法本身给的。
        ``count <= 0`` 按 1 处理（与 :meth:`_antenna_count` 的下界一致）。
        """
        return total_w / (count if count > 0 else 1)

    def deception_card(self) -> DeceptionSpec | None:
        """这台干扰机的**欺骗卡**；基类返回 ``None`` = 它不搬受害方的量测中心。

        做成方法而不是基类参数表里的一堆字段：欺骗的参数（静态偏置、拖引
        速率、封顶…）只对**这一种效果**有意义，摆在基类会让每一台干扰机的
        参数表里都挂着几个永远用不到的空位——而"参数表上有这一项却没人读"
        正是本项目最要消灭的那类静默退化。
        """
        return None

    def false_target_card(self) -> FalseTargetSpec | None:
        """这台干扰机的**假目标卡**；基类返回 ``None`` = 它不放幽灵。

        与 :meth:`deception_card` 同一条理由、同一个位置。两张卡可以**同时**
        给：压制的后果是"建不起来"、欺骗是"建起来是错的"、假目标是"建起来的
        不是它"——三件事互不排斥，AFSIM 的一个 ``technique`` 里也是并列的。
        """
        return None

    def shutdown(self) -> None:
        """关机 / 被击毁时**注销**——名册干净是"谁在压我"能答对的前提。

        ★ 注销交回的是**登记时那张卡**（``jammer_spec``），不是现算的
        ``self.entity_id``。一台机器可以挂多条天线（v0.13.33 起），而名册是
        **按天线**记的：只给实体 ID 是**整台机器**级的批量动作——那时先关的
        那条会把后关的那条一起抹掉，症状是"我只关了一条天线，另一条也不压
        了"。登记的那张卡是"这条天线是谁"唯一的出处。

        它同时也是**机器身份**（``tag``）：实体上换了另一台机器时，这张卡不
        属于现在在册的那台，``unregister`` 会拒绝它——这正是我们要的
        （"旧机器关机"不该关掉新机器）。

        ``getattr`` 是必要的：**登记失败过的部件也会被 shutdown**
        （``initialize`` 在参数校验那一关抛了，装配层照旧走销毁流程），
        那时这两个字段根本没写过——直接取会变成一个与"参数写错"毫无关系的
        AttributeError。
        """
        service = getattr(self, "service", None)
        spec = getattr(self, "jammer_spec", None)
        if service is not None and spec is not None:
            service.unregister(spec)

    def describe(self) -> str:
        """``型号(槽 功率 @ 频率)``，多天线时多一个 ``×N``。**未 ``initialize``
        时只给前半段。**

        理由同 :meth:`~milsim.models.percep.base.RadarSensor.describe`：
        ``repr`` 在装配**之前**就会被调用（工厂构建失败时的报错信息里就带着
        它），所以这里必须容忍"还没初始化"。

        ★ ``×N`` 是**多天线唯一能被看见的地方**（§5.14.11 ③）：想定里挂两个
        ``jammer2`` 与挂一个 ``jammer2`` 加一个 ``jammer3``，参数表上一模一样
        （两个部件、同样的参数），只有这个标记能让"怎么装的"和"名义上几条"露
        出来——后者两条其实是**两台**（尾数不同 ⇒ 各算 1 条）。
        """
        base = super().describe()
        if not hasattr(self, "power_w"):
            return base
        shown = (
            f" ×{self._antenna_count()}"
            if self._antenna_count() > 1
            else ""
        )
        return (
            f"{base} {self.power_w / 1e3:g} kW @ {self.frequency_hz / 1e9:g} GHz"
            f"{shown}"
        )


__all__ = ["Jammer"]
