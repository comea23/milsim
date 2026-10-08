"""通信件族：**通信节点侧**的组件（§5.15）。

★ 通信件与干扰机是**同一类东西**，但节拍相反
--------------------------------------------

干扰机（``models/jam``）**没有 ``update``**：它只在装配期登记一次，此后
"谁在压我"由受害方在检测那一刻去问。通信节点**两样都要**：

* 装配期登记（"我在哪个网"）——与干扰机一样，一个 :class:`CommNodeSpec`；
* **运行期有节拍**——按拍提交拓扑、按拍投递在途报文。因为"一个时间步内
  完成通信"是一句**关于时间**的话，而干扰"压不压得住"是一句**关于此刻**的话。

.. code-block:: text

    ======================  ===========================  =====================
    做的事                    谁做                         什么时候
    ======================  ===========================  =====================
    登记"我在哪个网"          通信件 initialize()          装配期，一次
    提交拓扑变化（掉线/恢复）  通信件每拍              ★ 在投递之前
    投递到期报文（在途→收件箱） 通信件每拍（门面在服务层）    同上
    注销（关机 / 被毁）        通信件 shutdown()            运行期，一次
    ======================  ===========================  =====================

★ 三阶段节拍，顺序不能反（§5.15，用户裁定）
------------------------------------------

.. code-block:: text

    ① 标记   把这一拍"谁被压制"的结论写进意图（mark_offline / mark_online）
    ② 提交   commit() —— 没有任何标记变过 ⇒ 直接跳过，图一个字节不动
    ③ 传信   deliver() —— 把到期的在途报文搬进收件箱

顺序反了的后果是**静默的**：这一拍发的信会用上一拍的拓扑做路由——被压制的
节点这一拍还在旧图里，信就还从它走。而"干扰明明生效了、报文还是穿过去了"
没有任何断言能自然地抓住它。

★ 标记这一步**由谁做**（v0.13.38 第二步，用户裁定）
--------------------------------------------------

**门面做，组件不重复做。** 一句"谁被压住"要求的是**全网**的答案：一个通信节点
只知道自己在哪、自己那台收信机是什么，不知道战场上还有哪些干扰机——那是施扰方
的东西、分布在别的平台上（与"谁能通到谁"是同一类全局问题）。

所以节拍是：

.. code-block:: text

    ① service.jammed_now(engine.now)   ← 门面逐节点求值 S/(N+J)，标掉那么几个
    ② service.commit()                 ← 变了才重建图
    ③ service.deliver(engine.now)      ← 按（新）拓扑投递

★ 为什么这三步**必须连在一起调**：``jammed_now`` 如果被漏掉，症状是"干扰机
挂着、通信照旧"；如果被放在 ``commit`` **之后**，症状是"干扰晚一拍生效"。
两者在参数表上都看不出来。

★ **每拍只调一次，不是每个节点调一次**（``jammed_now`` 遍历全网）：每个节点
各自跑一遍就是 N² 次求值，且**同一拍里不同节点看到的拓扑会分叉**（先跑的那个
``commit`` 过了、后跑的还没）——那是一个只有诊断计数能发现的静默不一致。

★ 通信件**不做**寻路，也不判"谁能通"
-----------------------------------

"从 A 到 B 现在通不通"是一个**全局拓扑**问题（§5.15），单节点答不了。所以
通信件的每拍只做三件事：求值并标记、提交、投递。真判"通不通"在发信那一刻由
:meth:`CommService.path_exists` 回答——由**门面**回答，不是由通信件。

★ 边界：本版接上了干扰，但**不接 AFSIM 的 ``EW_Effect``**
----------------------------------------------------------

AFSIM 的通信通断判据是 ``(S/N >= GetDetectionThreshold()) &&
(mInterferenceFactor < 0.5)``。第二个因子只在 AFSIM 的 ``EW_Effect`` 模型里
动，而**我们没有那个模型**（§9-44）⇒ 本版只用 SNR 门限，**判定变宽**（更
容易通）。这是自主裁定并被显式记录的缺口（§9-62），补它要一整层基于效应的
电子战模型。

另外本版**不建路径预算**（用户裁定①"一个时间步内均可完成通信"）：``S`` 不
按距离衰减，取"发信方额定功率 × 收发增益"作名义信号电平 ⇒ 名义 ``S/N`` 是
常数（见 :meth:`_nominal_snr`）。真建 ``R⁻²`` 衰减需要一个与"一拍必达"
冲突的距离模型。
"""

from __future__ import annotations

from ...engine.engine import PRIORITY_COMMS, EventResult
from ...errors import ConfigurationError
from ...services.comm import CommNodeSpec
from ...services.params import Params
from ..component import Component


class CommNode(Component):
    """通信件族的公共部分：**节点侧**的参数与登记。

    它**不是**可直接注册的实现（族约定：基类不注册，见 ``models/mover/base.py``）。
    """

    SLOT_HINT = "comm"

    PARAMS = {
        # -- 网归属 ------------------------------------------------------------
        #: **通信网名**（字符串）。空串 = 没写 ⇒ 由门面按**成员表**反查
        #: （见 :meth:`~milsim.services.comm.CommService.register`）。
        #: ★ 想定里的 ``platform`` 只声明"这个实体有通信能力"，**归属哪个网由
        #: ``network`` 块的 ``member`` 说清**（§5.15）——这里是那条显式路径的
        #: 出口：想定不写就留空，由成员表决定。写死一个名字的意义在于让
        #: "同一型号将来用于两个网"有地方表达（本版会与成员表校验）。
        #: ★ 一个实体**只能属于一个网**（§9-55 那条"多网归属不可复现"的判断）：
        #: 门面按实体建索引，一个实体两个网会让"它算哪个网的"没有唯一答案。
        "network": Params.string(""),
        # -- 收信机：算 S/(N+J) 的那几个数（v0.13.38 第二步，用户裁定③"一张表"）--
        #: **发射功率**（W）。第二步起**真的被读**：它就是受害方那一侧的 ``S``
        #: 来源（名义电平，不按距离衰减，见模块头）。
        #: ★ 默认 0 ⇒ :meth:`_nominal_snr` 算出 0 ⇒ 这一节点**不判干扰**
        #: （缺数据不等于被压住）⇒ "没写这一项"的想定数字逐字不变。
        "power": Params.power(0.0, minimum=0.0),
        #: 发射增益（dB）。与 ``power`` 一起给出名义 ``S``。默认 0 dBi
        #: （各向同性）——这是一个**合法取值**，不是"没给"。
        "antenna_gain_db": Params.number(0.0),
        #: 调谐频率（Hz）。``F_BW`` 的受害者侧那一半——**干扰机要落在这个带上
        #: 才进得来**。0 = 未给定 ⇒ 判干扰时 ``F_BW`` 为 0（一分不进）。
        "frequency": Params.frequency(0.0, minimum=0.0),
        #: 接收带宽（Hz）。``F_BW`` 的另一个输入。0 = 未给定 ⇒ ``F_BW`` 为 0。
        "bandwidth": Params.frequency(0.0, minimum=0.0),
        #: **接收支路内损**（dB，单程）。口径同 §9-54：干扰只扣受害方**接收
        #: 支路**内损（AFSIM 的 ``receive_loss``），发射支路内损不进干扰链路。
        #: 默认 0 dB ⇒ 不记损耗。
        "receive_loss_db": Params.number(0.0, minimum=0.0),
        #: **接收机噪声功率**（W）。``J/S = J/(SNR·N)`` 要有**绝对的 N**。
        #: 0 = 未给定 ⇒ 不判干扰（缺数据 ≠ 被压住）。想定里可写 ``1e-13 W``。
        "noise_power": Params.power(0.0, minimum=0.0),
        #: **收信机的 SNR 门限**（dB）。★ 用户裁定①B 的落点：它**不叫**
        #: ``detection_threshold_db``——AFSIM 的 ``GetDetectionThreshold()``
        #: 语义就是**信噪比门限**（不是 Pd），名字要照它说的事，不能照雷达那边
        #: 的 ``required_pd`` 借（那个是**概率**，两个量不同纲）。
        #: ``S/(N+J)`` 低于它 ⇒ 这一拍这个节点**被压住**（从拓扑上摘掉）。
        "required_snr_db": Params.number(0.0),
        #: 本节点的**接收主极化**（七取值之一）。默认 ``"default"`` = 没声明
        #: ⇒ ``F_POL`` 恒 1（不扣）。与雷达接收机**同一个取值集合**。
        "polarization": Params.string("default"),
        #: **天线架高**（米）。0 = 不判几何（同雷达口径，§9-42：0 是"没有这一项"，
        #: **不是**"天线贴地"）。本版通信**不判遮蔽**（见下），所以它现在只被
        #: 记录下来，供将来"山那面的干扰机进不来"用。
        "antenna_height": Params.distance(0.0, minimum=0.0),
        # -- 节拍 --------------------------------------------------------------
        #: 投递节拍（微秒）。默认 **2 s** —— 用户裁定"一个时间步内（2s）均可
        #: 完成通信"。这个值必须 ≥ 想定的 ``max_step``，否则一份报文会在
        #: 一个时间步内被搬运多次（"2 s 内完成"变成"2 s 内完成 N 次"）。
        #: 想定不写就用默认；这就是"只要在网里就一拍必达"的落点。
        "period": Params.duration(2_000_000, minimum=1),
    }

    __slots__ = (
        "service",
        "node_spec",
        "network",
        "_jam",
        "nominal_snr",
        "jam_screened",
        "jam_suppressed",
    )

    def initialize(self, mount) -> None:
        """登记进通信门面 + 注册每拍节拍。**这是通信件做的两件事。**

        ★ **半给不行**：想定里这个实体没有出现在任何 ``network`` 块的成员表里
        ⇒ ``register`` 当场报错。一个"自称在某网、而想定没把它列进去"的通信件
        会让寻路的成员表与宣称不一致——而症状是"它收不到信"，看起来像被干扰了。

        ★ 收信机那几个数**折算成线性**之后交给门面（``CommNodeSpec``）：门面
        不 import ``models``，也就拿不到 dB 换算——**这是分工，不是缺漏**。
        """
        self.service = mount.comm_service()
        if self.service is None:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 本次装配没有提供通信"
                "门面，通信件无处登记——这是装配层的缺陷，不是参数问题"
            )
        #: 电子战门面（§5.14）。第二步起要它才判得了"谁在压我"。
        #: ``None`` = 这份装配没有电子战 ⇒ 不判（与"名册为空"同一条路）。
        self._jam = mount.jam_service()

        name = self.type_name or type(self).__name__
        #: 本节点声称属于哪个网。想定通常在 ``network`` 块里点名，这里为空；
        #: 门面按**成员表**反查（见 :meth:`CommService.register`），所以空串
        #: 时取门面算出来的那个网名。
        declared = str(self.spec["network"]).strip()
        #: 名义信号电平（W）：``power × 10^(G/10) × 10^(G_rx/10)``，其中
        #: ``G_rx`` 复用发射增益那一项（本版收发共用一副天线，与雷达"收发共用
        #: 一部天线"同口径）。**不按距离衰减**（见模块头）。
        self.nominal_snr = self._nominal_snr()
        self.node_spec = CommNodeSpec(
            entity_id=mount.entity_id,
            network=declared,
            tag=name,
            power_w=float(self.spec["power"]),
            tx_gain=10.0 ** (float(self.spec["antenna_gain_db"]) / 10.0),
            rx_gain=10.0 ** (float(self.spec["antenna_gain_db"]) / 10.0),
            frequency_hz=float(self.spec["frequency"]),
            bandwidth_hz=float(self.spec["bandwidth"]),
            rx_loss=10.0 ** (float(self.spec["receive_loss_db"]) / 10.0),
            noise_w=float(self.spec["noise_power"]),
            snr=self.nominal_snr,
            required_snr_db=float(self.spec["required_snr_db"]),
            polarization_type=str(self.spec["polarization"]),
            antenna_height_m=float(self.spec["antenna_height"]),
        )
        self.service.register(self.node_spec)
        #: 生效的网名（门面判定的那个）。诊断用。
        self.network = self.service.network_of(mount.entity_id) or declared
        #: 本节点被求值过多少次（诊断用）。与门面那个 ``screened`` 是**同一件事
        #: 的两个视图**：门面记全网合计，这里记本节点自己。
        self.jam_screened = 0
        #: 本节点被判成"被压住"过多少次（诊断用）。
        self.jam_suppressed = 0

        mount.every(int(self.spec["period"]), self._tick, PRIORITY_COMMS)

    def _nominal_snr(self) -> float:
        """名义 ``S/N``（线性）；**缺数据就返 0** = 不判干扰。

        ``S = power × G_tx × G_rx``（不按距离衰减），``N = noise_power``。
        ★ 返 0 的意思**不是**"信噪比为零"，而是"这个数不知道"——与 §9-42 那条
        "0 落在指数上才是缺数据"同源。真拿 0 去算 ``J/S`` 会得到无穷大的
        ``J/S``、进而"任何干扰都压得住"——那正是"缺数据伪装成被干扰"。

        ★ **写死在一个地方**（组件这里算好、交给门面），不是让门面再算一遍：
        门面拿到的是折算好的线性值（它不认识 dB），而"名义 S/N 怎么来的"
        只有这一处出处。
        """
        power = float(self.spec["power"])
        noise = float(self.spec["noise_power"])
        if power <= 0.0 or noise <= 0.0:
            return 0.0
        gain = 10.0 ** (float(self.spec["antenna_gain_db"]) / 10.0)
        return power * gain * gain / noise

    def _tick(self, engine, event):
        """每拍：**求值并标记** → 提交 → 投递。返回 ``RESCHEDULE``。

        ★ 三步的顺序是**语义**，不是风格（见模块头）：反了的话，这一拍发的信
        会用上一拍的拓扑路由——"干扰明明生效了、报文还是穿过去了"。

        ★ ``jammed_now`` **每拍每组件调一次**（虽然它遍历全网）：调用点放在
        这里是为了让"这一拍"由一个时钟定义，而不是由"某个节点恰好跑到了"定义。
        真要优化，正确做法是让**装配层**每拍调一次、组件只读结论——那会引入
        第二个节拍源，属后续（本版节点数不大，N² 与 N 的差别看不出来）。
        """
        service = self.service
        if service is None:
            return EventResult.RESCHEDULE
        jammed = service.jammed_now(engine.now)
        if jammed:
            self.jam_suppressed += jammed
        if self._jam is not None and not self._jam.is_empty:
            self.jam_screened += 1
        service.commit()
        service.deliver(engine.now)
        return EventResult.RESCHEDULE

    def shutdown(self) -> None:
        """关机 / 被击毁时注销节点——名册干净是"这条链路通不通"能答对的前提。

        ★ 注销交回的是**登记时那张卡**（``node_spec``），与干扰机同一条理由：
        ``getattr`` 是必要的（登记失败过的部件也会被 shutdown，那时字段没写过）。
        """
        service = getattr(self, "service", None)
        spec = getattr(self, "node_spec", None)
        if service is not None and spec is not None:
            service.unregister(spec.entity_id)

    def describe(self) -> str:
        """``型号(槽 @ 网名)``，**未 ``initialize`` 时只给前半段**。"""
        base = super().describe()
        network = getattr(self, "network", "")
        if not network:
            return base
        return f"{base} @ {network}"


__all__ = ["CommNode"]
