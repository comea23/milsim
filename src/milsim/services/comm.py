"""通信门面：通信网名册 + "从 A 到 B 现在通不通"的**图寻路**。

为什么要有这一层
================

"能不能通"是一个**全局拓扑**问题：A 想发信给 B，走不走得到，取决于**整个网里
有哪些节点、它们之间连了哪些边、以及此刻谁被干扰掉线了**。任何单个通信节点都
不知道这些——它只知道自己在哪个网、连了谁。

而组件之间不许互相 import，所以这个汇总必须由装配层注入的一个门面来回答。
本模块属于 ``services`` 层，**不许 import ``models``**：通信节点在这里是一张
纯数据（:class:`CommNodeSpec`），由通信组件在 ``initialize()`` 里登记进来，
发送方在"要发信"的那一刻来查。

★ 抄 AFSIM 的骨架，不抄它的七层
--------------------------------

AFSIM 的通信是一条完整的 OSI 七层栈（``WsfCommPhysicalLayer`` → Datalink →
Network → Transport → Session → Presentation → Application，200+ 文件）。
对**任务级**仿真一层都用不上。真正支撑"能不能通"的只有三件事：

1. **网络 = 成员集合 + 地址**（``network_name`` / ``Address``）；
2. **发信前查图**：``NetworkManager::PathExists`` → ``Graph::FindAnyPath``
   （图上最短路）；
3. **拓扑模型是图**，不是规则：``WSF_COMM_NETWORK_GENERIC`` 收
   ``comm_list``（成员）+ ``comm_link_list``（有向边）。星型/环型只是便利封装。

本模块把这三件事落成 1（成员）+ 2（寻路）+ 3（图）。**不建传输延时模型**：
发送入队、下一拍必达（节拍由通信组件给出，见 §5.15）。

★ 三条口径（用户裁定，§5.15）
-----------------------------

1. **被干扰 ⇒ 从拓扑上消失、报文绕路**。不是"断网"，是**重路由**：原来直达的
   那条路，因为路上有个节点被压制而不可用，就换别的路走。⇒ :meth:`mark_offline`
   把节点标为待摘除，:meth:`commit` 提交后它从**可通行集合**里消失，此后
   :meth:`path_exists` 在**新图**上重新寻路。
2. **入网即全连通，但"通不通"按拓扑算**。所有成员两两之间**默认有边**
   （AFSIM 的默认行为："all comm devices on the same network are assumed to
   have connectivity to every other object in the same network ... mesh
   network topology"）；想定里的 ``link`` 用来**收窄**成真实拓扑。可达性
   每次在（已提交的）图上重算。
3. **按拍**：这一拍被压制 ⇒ 这一拍离线；下一拍没压住 ⇒ 自动回来
   （因此 :meth:`mark_offline` / :meth:`mark_online` 是**幂等的开关**，
   不引入"恢复"这个新状态）。

★ 两阶段：先提交状态，再传信（§5.15，用户裁定）
-----------------------------------------------

每一拍通信**必须**分两步，顺序不能反：

.. code-block:: text

    ① 标记   mark_offline(eid) / mark_online(eid) / register / unregister
       —— 只记"有谁想变"，拓扑**不动**
    ② 提交   commit()
       —— 比较新旧状态：变了 ⇒ 重建图、epoch += 1、返回 True
                        没变 ⇒ 立刻返回 False，**一个字节都不动**
    ③ 传信   path_exists() / route() —— 用提交后的图寻路

为什么顺序不能反：**反了的话，这一拍发的信会用上一拍的拓扑做路由**——被干扰的
节点这一拍还在旧图里，信就还从它走。那是一个"干扰明明生效了、可报文还是穿过去了"
的静默错误，没有任何断言能自然地抓住它。

为什么"没变化就跳过"是**语义**而不只是性能：它让"拓扑这一拍改没改"变成一个
**可观察、可计数**的事实（:attr:`epoch`）。诊断"信为什么从那儿走了"的第一个问题
就是"这一拍拓扑动过没有"，而 :attr:`epoch` / :attr:`commits` 直接回答它。

★ "没建通信网 ⇒ 零开销"是正确性的一部分
----------------------------------------

``_nets`` 为空时 :meth:`path_exists` **直接返回"不限制"**——不建网就是想定
里根本没提通信，那么旧想定的所有消息投递**逐字不变**（这是本项目的验收判据，
不是优化）。真"没网就断"会让每一份既有想定突然发不出信，而症状是"什么都没动
却全哑了"。

★ 第二步：接干扰（v0.13.38，§5.15）
-----------------------------------

干扰的**施扰端复用** ``RF_JAMMER``（用户裁定②A）：通信受害机调的是**雷达那同一个
门面**。这在 AFSIM 源码上是逐项成立的——``WsfEW_CommComponent.cpp:178`` 调的就是
``WsfRF_Jammer::ComputeTotalJammerEffects(...)``，通信接收机与雷达接收机用的是
**同一个函数**。所以这里不需要"通信专用干扰机"这套语法。

受害端只多一件事：**标记阶段逐节点问一次"谁在压我"**，算

.. code-block:: text

    S/(N+J) = snr_under_jamming(S/N, J/S)     （与雷达同一个式子）

不过收信机自己的 SNR 门限（``required_snr_db``）⇒ :meth:`mark_offline` 自己。
拓扑下一拍自动绕路——**不需要任何新的后果模型**：被压住就是"这一拍照它不通"，
与"它关机了"共用一条通路。

★ 本版**不实现** AFSIM 判据的第二个条件
----------------------------------------

AFSIM 的通信通断判据是 ``(S/N >= GetDetectionThreshold()) &&
(mInterferenceFactor < 0.5)``（``WsfCommComponentHW.cpp:303``）。第二个因子只在
AFSIM 的 ``EW_Effect`` 模型里被写成 1.0（``cEB_DROP_MESSAGE`` /
``cEB_...``，见 ``WsfEW_CommComponent.cpp:183``），而**我们没有 ``EW_Effect``
模型**（§9-44）。所以本版只用**第一个条件**（SNR 门限）。

⇒ 这是一个**判定变宽**（更容易通）：AFSIM 在"干扰够强到触发效应"时会**直接丢包**，
我们只会"这一拍不通、下一拍再看"。**它是自主裁定并被显式记录的缺口**，不是疏忽
（§9-62）。补它需要一整层"基于效应的电子战"模型，那是另一件事。

★ 名册为空 ⇒ 零开销，也是正确性的一部分
----------------------------------------

``jam is None``（这份装配没有电子战）或 ``jam.is_empty``（没有干扰机在册）时，
:meth:`_jam_now` **在遍历节点之前就返回**——连位置都不查。于是"没挂干扰机的
想定逐字不变"不是靠某个 ``if`` 挡出来的，而是"没有这个东西"的自然结果。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import cos, log10, pi
from typing import Any

from ..errors import ConfigurationError

# ★ 从电子战门面借副瓣水平（-13 dB），**不重写一个数**：它是**同一个物理量**
# 的两个使用者（雷达接收机与通信接收机）共用的常数，各写一份迟早不同步，
# 而且不同步时不报错。
from .ew import DEFAULT_SIDELOBE_LEVEL_DB

#: ``F_POL`` 表的**只读镜像**（唯一主人在 ``models.percep.equation``，出处是
#: AFSIM 的 ``WsfEM_Rcvr::UpdatePolarizationEffects()``）。门面不能 import
#: ``models``，所以这一份是**抄过来的**——它能被允许的唯一理由是
#: ``tests/test_comm.py`` 里有一条把两边同时算一遍比数值的守门人用例。
_POLARIZATION_KINDS: tuple[str, ...] = (
    "horizontal",
    "vertical",
    "slant_45",
    "slant_135",
    "left_circular",
    "right_circular",
    "default",
)

_POLARIZATION_EFFECTS: tuple[tuple[float, ...], ...] = (
    (1.0, 0.0, 0.5, 0.5, 0.5, 0.5, 1.0),   # 接收 horizontal
    (0.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0),   # 接收 vertical
    (0.5, 0.5, 1.0, 0.0, 0.5, 0.5, 1.0),   # 接收 slant_45
    (0.5, 0.5, 0.0, 1.0, 0.5, 0.5, 1.0),   # 接收 slant_135
    (0.5, 0.5, 0.5, 0.5, 1.0, 0.0, 1.0),   # 接收 left_circular
    (0.5, 0.5, 0.5, 0.5, 0.0, 1.0, 1.0),   # 接收 right_circular
    (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),   # 接收 default（没声明 ⇒ 不判）
)

_POLARIZATION_INDEX = {name: i for i, name in enumerate(_POLARIZATION_KINDS)}


@dataclass(frozen=True, slots=True)
class CommNodeSpec:
    """一个通信节点的**能力声明**（纯数据，不含行为）。

    通信组件在 ``initialize()`` 里造一个登记进来，发送方读它。字段全是
    "这个节点是什么"，**没有一个字是"它现在能不能通"**——后者由拓扑决定。

    ★ 第二段（``rxs_*``）是**收信机的接收链口径**（v0.13.38，用户裁定③"一张表"）：
    干扰要算 ``S/(N+J)``，"我的接收增益 / 调谐带 / 噪声 / 门限"全是**收信方自己**
    的属性（AFSIM 的 ``Rcvr`` 块），所以它们落在**收信方这条卡上**，不从平台的
    别的部件借。

    ★ 为什么**不给默认值**（全部关键字必填或带显式默认）：这些字段每一个都有
    "合理"的默认值可编（增益 0 dBi、带宽 25 kHz、噪声 290 K…），而编出来的
    默认值会让一份没写干扰相关参数的想定**算出一个看着合理的 J**——正是本项目
    最想消灭的那类静默退化。⇒ 由组件在 ``initialize()`` 里**显式喂全**，缺哪项
    当场报错（见 :meth:`CommNode._require_jam_budget`）。
    """

    entity_id: int
    #: 它属于哪个通信网（网络名）。同名即同网。
    network: str
    #: 诊断用的型号名。
    tag: str = ""

    # -- 收信机（算 S/(N+J) 要的那几项）-------------------------------------
    #: 发射功率（W）。第二步起**真的被读**：它就是受害方那一侧的 ``S`` 来源。
    #: 但本版**仍然不建路径预算**（用户裁定①"一个时间步内均可完成通信"）——
    #: 于是 ``S`` 不按距离衰减，取"发信方额定功率 × 增益"作为名义信号电平。
    #: ★ 这是**记录的简化**，不是疏忽：真建 R⁻² 衰减就需要一个与"一拍必达"
    #: 冲突的距离模型（§5.15）。
    power_w: float = 0.0
    #: 发射增益（线性）。与 ``power_w`` 一起给出名义 ``S``。
    tx_gain: float = 1.0
    #: 收信机**接收增益**（线性）——主瓣方向取它。干扰从副瓣进来时按
    #: :data:`~milsim.services.ew.DEFAULT_SIDELOBE_LEVEL_DB` 打折（与雷达
    #: **同一个常数**）。
    rx_gain: float = 1.0
    #: 调谐频率（Hz）。``F_BW`` 的受害者侧那一半。
    frequency_hz: float = 0.0
    #: 接收带宽（Hz）。``F_BW`` 的分母之一。
    bandwidth_hz: float = 0.0
    #: **接收支路内损**（线性，单程）。口径同 §9-54：干扰只扣受害方接收支路
    #: 内损（AFSIM 的 ``receive_loss``），发射支路内损不进干扰链路。
    rx_loss: float = 1.0
    #: **接收机噪声功率**（W）。``J/S = J/(SNR·N)`` 要有**绝对的 N**。
    noise_w: float = 0.0
    #: 名义 ``S/N``（线性）。本版不建路径预算 ⇒ ``S`` 与 ``N`` 都是常数，
    #: 于是这个比值也是常数。★ 它不是"标定出来的"：由 ``power_w·tx_gain``
    #: 与 ``noise_w`` 直接算出（见 :meth:`CommNode._nominal_snr`），所以
    #: **同一个量只有一个出处**。
    snr: float = 0.0
    #: **收信机的 SNR 门限**（dB，用户裁定①B：叫 ``required_snr_db``，因为
    #: AFSIM 的 ``GetDetectionThreshold()`` 语义就是**信噪比门限**，不是 Pd）。
    #: 门限以上才谈"收得到"。
    required_snr_db: float = 0.0
    #: 本节点的**接收主极化**（七取值之一，默认 ``"default"`` = 不判）。
    polarization_type: str = "default"
    #: **天线架高**（米）。0 = 不判几何（同雷达口径，§9-42）。
    antenna_height_m: float = 0.0
    #: 主瓣全宽（度）。本版通信**不做扫描**（`fov=0` 的固定指向），所以
    #: 判"干扰进没进主瓣"时用**全向**那一档：``0`` ⇒ 任何方位都算主瓣内。
    #: ★ 这与"通信天线通常是全向鞭状"一致，也是"通信比雷达更怕干扰"的
    #: 物理来源——雷达能靠天线指向把干扰甩到副瓣，通信甩不掉。
    beam_width_deg: float = 0.0


@dataclass(frozen=True, slots=True)
class CommNet:
    """一个通信网的**声明**（纯数据）：成员 + 显式边。

    ``links`` 为空 ⇒ 用"入网即全连通"的默认行为；
    非空 ⇒ **只用这些显式有向边**（想定说了算，"真实节点分布"就是这个意思）。

    ``links`` 是有向的：``(a, b)`` 表示"a 能发给 b"，不蕴含 b 能发给 a。
    AFSIM 的 ``comm_link_list`` 原文："The order specified matters, as a link
    is only created from the first comm specified, to the second."
    """

    name: str
    members: tuple[int, ...] = ()
    #: 显式有向边：``(源实体 ID, 目的实体 ID)``。空 ⇒ 全连通。
    links: tuple[tuple[int, int], ...] = ()
    #: 诊断：因**单边通信**而自动并入的成员（§5.15 裁定③——想定里 A 向网内 B
    #: 发信 ⇒ A 自动入网）。与 ``members`` 分开存，是为了让"谁是想定点名进来的、
    #: 谁是发信蹭进来的"在复盘时看得见。
    auto_members: tuple[int, ...] = ()


class CommService:
    """通信网名册 + 逐次查询的"通不通"。

    ``__slots__`` **只放句柄与计数器**，不缓存任何派生量——节点中途可以被
    干扰掉线 / 恢复 / 注销，缓存下来的"谁能通到谁"会立刻变成假数据。
    （唯一缓存的是**图的邻接表** :attr:`_links`，而它**只在 commit 批准**的
    时候重建——那正是两阶段要保证的东西。）
    """

    __slots__ = (
        "_store",
        "_registry",
        "_jam",
        "_nets",
        "_membership",
        "_specs",
        "_links",
        "_live",
        "_pending",
        "epoch",
        "commits",
        "queries",
        "routed",
        "blocked",
        "delivered",
        "jammed",
        "screened",
    )

    def __init__(self, store: Any = None, registry: Any = None, jam: Any = None) -> None:
        #: 实体存储。**只为 :meth:`deliver` 搬报文**——"通不通"的判据一个字都
        #: 不靠它（那是纯拓扑）。``None``（纯单元测试）时 :meth:`deliver` 直接
        #: 返回 0，寻路那几条入口**逐字不变**。
        self._store = store
        self._registry = registry
        #: 电子战门面（:class:`~milsim.services.ew.JamService`）。**第二步加的**。
        #: ``None`` = 这份装配没有电子战 ⇒ :meth:`jammed_now` 一步早退，
        #: 与"名册为空"同一条路（都是"这场没有干扰"）。这就是"没挂干扰机的
        #: 想定逐字不变"在**通信侧**的落点。
        self._jam = jam
        #: ``网名 → CommNet``。想定里声明的网在这里。
        self._nets: dict[str, CommNet] = {}
        #: ``实体 ID → 网名``。**已经从实体到网**的直接索引——寻路每一步都要
        #: 问"这个节点在哪个网"，靠遍历 ``_nets`` 会退化成 O(网 × 节点)。
        self._membership: dict[int, str] = {}
        #: ``实体 ID → 它登记的那张能力卡``。★ 第二步加的：算 ``S/(N+J)`` 要
        #: 收信机的接收链参数，而它们**就在节点登记时交回来的那张卡上**
        #: （:meth:`register`）——门面不必（也不许）回头去问 ``models`` 层。
        #: 注销时**当场**删掉，否则一份"已经关机"的卡还会参与干扰求值。
        self._specs: dict[int, CommNodeSpec] = {}
        #: ``网名 → 有向邻接表``（``dict[源, set[目的]]``）。**只在 commit 批准
        #: 拓扑变化时重建**——它是"当前生效的图"，不是"想要生效的图"。
        self._links: dict[str, dict[int, set[int]]] = {}
        #: **当前生效的掉线节点**集合（实体 ID）。被干扰 ⇒ 提交后进来；
        #: 恢复 ⇒ 提交后出去。★ 它是**已提交**的真相，:attr:`_pending` 是意图。
        self._live: set[int] = set()
        #: **待提交**的掉线节点集合（标记阶段写它）。commit 时与 :attr:`_live`
        #: 比对，不同才重建。
        self._pending: set[int] = set()
        #: **拓扑纪元**。每一次 commit **真的改了**拓扑就 +1。诊断的抓手：
        #: "这一拍信为什么走了那条路" ⇒ 先看 epoch 动没动。
        self.epoch = 0
        #: commit 被调用过多少次（含"没变化"的那些）。
        self.commits = 0
        #: 名册被问过多少次（诊断用）。
        self.queries = 0
        #: 寻路"找到路"过多少次（诊断用）。
        self.routed = 0
        #: 寻路"没找到路"过多少次（诊断用）——含"节点掉线导致断开"那一类。
        self.blocked = 0
        #: 历次 :meth:`deliver` 一共投递了多少条报文（诊断用）。
        self.delivered = 0
        #: :meth:`jammed_now` 一共把多少**节点次**判成"被压住"（诊断用）。
        #: 与 :attr:`screened` 是一对：那个数"问过几个节点"，这个数"压住了几个"
        #: ——合成一个数就再也分不清"没有干扰机在册"与"干扰机功率不够"。
        self.jammed = 0
        #: :meth:`jammed_now` 一共对多少**节点次**做过求值（诊断用）。
        #: 名册为空 / 没建网时恒为 0 ⇒ "零开销"是一个可观察的事实。
        self.screened = 0

    # -- 建网（装配期，一次性） --------------------------------------------

    def declare(self, net: CommNet) -> None:
        """声明一个通信网（想定解析期调用）。同名重复声明 ⇒ 报错。

        报错的理由同"实体重名"：静默合并两张网会让"我明明分了两个网"变成一个，
        而症状是"不该通的两个节点通了"。

        ★ 装配期的建网**不推 epoch**：拓扑在装配完就定了，:attr:`epoch` 从 0
        开始记的是**运行期**的改动。这样"epoch 变了"永远等于"这一拍拓扑动过"，
        不会因为想定里多写一个网而误报。
        """
        if net.name in self._nets:
            raise ConfigurationError(f"通信网 {net.name!r} 已经声明过了")
        self._nets[net.name] = net
        self._membership_put(net.name, net.members)
        self._membership_put(net.name, net.auto_members)
        self._rebuild_links(net.name)

    def _membership_put(self, network: str, ids: tuple[int, ...]) -> None:
        """把一批实体登记进某个网。**同一个实体只属于一个网**。

        ★ 为什么不允许多网归属：``path_exists`` 的第一步是"这两个节点在不在
        同一个网"——一个实体属于两个网时，"它算哪个网的"没有唯一答案，
        而寻路会随遍历顺序给出不同的结果（不可复现）。真要跨网通信，那是
        **网关**（AFSIM 的 ``gateway``），属后续，不是"多挂几个网"。
        """
        for entity_id in ids:
            old = self._membership.get(entity_id)
            if old is not None and old != network:
                raise ConfigurationError(
                    f"实体 {entity_id} 已在通信网 {old!r}，不能又加入 {network!r}"
                    "（一个节点一个网；跨网通信要走网关，那是另一件事）"
                )
            self._membership[entity_id] = network

    def _rebuild_links(self, network: str) -> None:
        """重建某个网的邻接表。**全连通网与显式边网是两种，不是叠加**。"""
        net = self._nets[network]
        members = list(net.members + net.auto_members)
        if net.links:
            table: dict[int, set[int]] = {e: set() for e in members}
            for src, dst in net.links:
                table.setdefault(src, set()).add(dst)
                table.setdefault(dst, set())
        else:
            #: 入网即全连通：每个成员能直发到**其它每一个**成员。
            table = {e: {o for o in members if o != e} for e in members}
        self._links[network] = table

    # -- 登记 / 注销单个节点（组件生命周期） --------------------------------

    def register(self, spec: CommNodeSpec) -> None:
        """登记一个通信节点。

        ★ 网的**成员表在解析期就定好了**（想定写了谁在网里），这里做的事是
        确认"这个实体确实属于它声称的那个网"。**不在这里扩建成员表**——
        一个网有哪些成员必须由想定说清，否则"谁该收到信"是运行期才知道的，
        拓扑就不确定了。

        ★ 空串 = **没声明**（想定整句都没写 ``network``），不是"声明了一个空网
        名"。两种都要收：想定的常规写法是"``network`` 块点名，平台块不写网名"，
        那时组件交回来的就是默认空串。**只有显式写了一个对不上的名字**才是
        配置错误——那种情况下"它到底属于哪个网"有两个答案，而症状是寻路
        随装配顺序给不同结果。
        """
        owner = self._membership.get(spec.entity_id)
        if owner is None:
            raise ConfigurationError(
                f"通信节点 {spec.tag or spec.entity_id} 想登记，"
                "但想定里没有把该实体列进任何通信网——装配层的缺陷"
            )
        declared = spec.network.strip()
        if declared and declared != owner:
            raise ConfigurationError(
                f"通信节点 {spec.tag or spec.entity_id} 声明属于网 {declared!r}，"
                f"但想定把它列进了网 {owner!r}。一个节点一个网"
                "（两种写法冲突时没有唯一答案，而寻路会随装配顺序变）"
            )
        #: 校验过了才收下这张卡：算 ``S/(N+J)`` 时它是**这个节点**的收信机口径。
        self._specs[spec.entity_id] = spec

    def unregister(self, entity_id: int) -> bool:
        """注销一个节点（关机 / 被摧毁）。返回"名册里真的有它"。

        ★ 注销与**掉线**是两件事，**不能合并**：

        ==============  ================================  ==========================
        动作            语义                              对拓扑的影响
        ==============  ================================  ==========================
        ``mark_offline`` 被干扰，**暂时**从图里摘掉         这一拍不可通行，下一拍可能回来
        ``unregister``   **永久**消失（关机 / 被打掉）       从成员表里彻底移除
        ==============  ================================  ==========================

        合并的话，"这一拍被压制"会永久毁掉这个节点，而症状是"干扰早停了，
        它却再也收不到信"。

        ★ 注销**立即生效**（不像掉线那样要等 commit）：关机 / 被毁是**结构
        删除**，不是"这一拍的通行状态"。想定不会在**同一拍**里既注销又发信，
        所以这里不必排进待提交队列。
        """
        self._live.discard(entity_id)
        self._pending.discard(entity_id)
        #: 卡也一起撤：一份"已经关机"的收信机口径不该继续参与干扰求值
        #: （它会让"击毁干扰机压制源"这类动作被一个幻影节点抵消）。
        self._specs.pop(entity_id, None)
        network = self._membership.pop(entity_id, None)
        if network is None:
            return False
        net = self._nets.get(network)
        if net is not None:
            self._nets[network] = CommNet(
                name=net.name,
                members=tuple(m for m in net.members if m != entity_id),
                links=tuple(
                    (a, b) for a, b in net.links if a != entity_id and b != entity_id
                ),
                auto_members=tuple(m for m in net.auto_members if m != entity_id),
            )
            self._rebuild_links(network)
        self.epoch += 1
        return True

    # -- 阶段①：标记（只改意图，不碰拓扑） ----------------------------------

    def mark_offline(self, entity_id: int) -> None:
        """标记"这一拍该节点不可通行"（被干扰）。**幂等**，只写意图。"""
        self._pending.discard(entity_id)
        self._pending.add(entity_id)

    def mark_online(self, entity_id: int) -> None:
        """标记"这一拍该节点可通行"（没被压住 / 恢复）。**幂等**，只写意图。"""
        self._pending.discard(entity_id)

    @property
    def pending_nodes(self) -> frozenset[int]:
        """待提交的掉线意图（诊断用）。**不等于**当前生效的掉线集合。"""
        return frozenset(self._pending)

    # -- 阶段①的**求值**：谁被压制（v0.13.38，§5.15 第二步）-------------------

    def jammed_now(self, now: int = 0) -> int:
        """**标记阶段的全网求值**：这一拍谁被压制 ⇒ :meth:`mark_offline` 它自己。

        返回"这一拍把几个节点判成被压住"。``now`` 目前不参与判据（压制是个
        **几何瞬时量**，不是过程），保留它是因为**调用点的位置**才是这一步的
        要点：它必须在 :meth:`commit` **之前**，否则这一拍的信会用上一拍的拓扑
        路由（§5.15 三条口径，顺序反了是静默错误）。

        逐节点做的三件事（与雷达那条链**共用同一套公式与同一个常数**）：

        .. code-block:: text

            ① 向名册要候选      jam.threats(eid, beam_bearing=?, main_lobe_deg=?)
            ② 逐台算进不来的   F_BW → 遮蔽 → 接收增益（主瓣/副瓣）→ F_POL → J
            ③ S/(N+J) 与门限比  snr_under_jamming(S/N, J/S) < required ⇒ 压住

        ★ **波束指向取哪儿**（通信没有扫描，见 ``CommNodeSpec.beam_width_deg``）：
        本版把通信天线当**全向**——``main_lobe_deg = 360`` ⇒ 任何方位都算主瓣内，
        于是"干扰从主瓣进来"。这不是偷懒：通信天线通常是全向鞭状，甩不掉干扰，
        而"通信比雷达更怕干扰"正是这个物理差别的后果。要改成有向，只需给
        ``beam_width_deg`` 一个正数并把 ``beam_bearing_deg`` 指向收件方——
        **接口已经摆好**（门面的 ``threats`` 就是那个形状）。

        ★ ``S/(N+J)`` 用**同一个** :func:`~milsim.models.percep.equation.
        snr_under_jamming`。它是"干扰下信噪比"的唯一出处；各写一份的话，两个
        受害侧的判据迟早分叉，而症状是"同一个干扰机对雷达有效、对通信无效"
        ——那在参数表上完全看不出来。

        ★ 这个方法的**唯一后果**是 ``mark_offline``：被压住 ⇒ 这一拍从拓扑上
        摘掉 ⇒ :meth:`path_exists` 与 :meth:`deliver` 自然绕路/不投。**没有新的
        后果模型**，也不需要改寻路一个字。

        ★ 名册为空 / 没建网 ⇒ **在遍历节点之前就返回**（连位置都不查）：
        "没挂干扰机的想定逐字不变"是"没有这个东西"的自然结果。
        """
        jam = self._jam
        if jam is None or jam.is_empty or self.is_empty:
            return 0

        by_id = self._specs
        if not by_id:
            return 0

        count = 0
        for entity_id in sorted(by_id):
            spec = by_id[entity_id]
            if entity_id in self._pending:
                continue                    # 已经被标掉线（上游别的理由）⇒ 不重复求值
            self.screened += 1
            if self._suppressed(entity_id, spec):
                self.mark_offline(entity_id)
                self.jammed += 1
                count += 1
        return count

    def _suppressed(self, victim_id: int, spec: CommNodeSpec) -> bool:
        """这一个收信机**此刻**被压住没有（``S/(N+J) < required``）。

        与雷达 :meth:`~milsim.models.percep.base.RadarSensor.jamming_effect` 的
        求值**逐项同源**（同一个 ``Jam.1``、同一个 ``F_BW``、同一个副瓣常数、
        同一个 ``snr_under_jamming``），但**不 import models**：公式在这里用
        ``services/ew`` 与一个本层的纯函数副本复现。

        ★ 为什么允许"公式有一份副本"：门面不能 import ``models.percep.equation``
        （那会让 ``services`` 依赖 ``models``，撞破本项目最硬的一条分层约定）。
        代价是这里要手写三个式子——**所以它们必须与方程模块逐字一致**，而
        ``tests/test_comm.py`` 里有一条守门人用例**把两边同时算一遍比数值**
        （:func:`test_the_comm_link_budget_matches_the_radar_equation`）。这是
        "两处实现"能被允许的唯一条件：有一处会**报错**的对照。
        """
        threats = self._jam.threats(
            victim_id,
            # 通信不做扫描：主瓣全宽取满一圈 ⇒ 任何方位都算"从主瓣进来"。
            beam_bearing_deg=0.0,
            main_lobe_deg=360.0,
        )
        if not threats:
            return False
        signal = spec.power_w * spec.tx_gain * spec.rx_gain
        noise = spec.noise_w
        if signal <= 0.0 or noise <= 0.0:
            # 没有绝对的 S 或 N ⇒ J/S、J/N 都无从谈起。这**不是**"被压住"：
            # 缺数据（这个节点没写功率/噪声）不该伪装成"它被干扰了"。
            return False
        snr = signal / noise
        sidelobe = 10.0 ** (DEFAULT_SIDELOBE_LEVEL_DB / 10.0)
        total = 0.0
        for threat in threats:
            f_bw = self._bandwidth_overlap(
                threat.frequency_hz,
                threat.bandwidth_hz,
                spec.frequency_hz,
                spec.bandwidth_hz,
            )
            if f_bw <= 0.0:
                continue                    # 带外：一分都进不来
            # 接收增益：通信天线全向 ⇒ 恒取主瓣那一份（见 ``_suppressed`` 头）。
            rx_gain = spec.rx_gain
            tx_gain = threat.gain
            if threat.antenna_off_deg is not None:
                tx_gain *= self._pattern_factor(
                    threat.antenna_off_deg, threat.beam_width_deg, sidelobe
                )
            loss = threat.internal_loss * spec.rx_loss
            f_pol = self._polarization(
                spec.polarization_type,
                threat.polarization_type,
                threat.polarization,
            )
            total += self._jam_power(
                threat.radiated_power_w,
                tx_gain,
                rx_gain,
                spec.frequency_hz,
                threat.slant_m,
                loss=loss,
                bandwidth_overlap=f_bw,
                polarization=f_pol,
            )
        if total <= 0.0:
            return False
        # S/(N+J) = SNR / (1 + (J/S)·SNR)，而 J/S = J/(SNR·N)。
        j_to_s = total / (snr * noise)
        sn = snr / (1.0 + j_to_s * snr) if j_to_s > 0.0 else snr
        return log10(sn) * 10.0 < spec.required_snr_db

    # -- 与 equation 模块逐字一致的三个纯式子（见 :meth:`_suppressed` 的说明）--

    @staticmethod
    def _bandwidth_overlap(
        jammer_frequency_hz: float,
        jammer_bandwidth_hz: float,
        receiver_frequency_hz: float,
        receiver_bandwidth_hz: float,
    ) -> float:
        """``F_BW`` —— **逐字**等于 ``equation.bandwidth_overlap_ratio``。

        照抄而不是"大概一样"：这两份的数值必须一致到能被测试逐位比对
        （见 :func:`test_the_comm_link_budget_matches_the_radar_equation`）。
        """
        if receiver_bandwidth_hz <= 0.0:
            return 0.0
        f_r, b_r = receiver_frequency_hz, receiver_bandwidth_hz
        if jammer_bandwidth_hz <= 0.0:            # CW：单频点
            return 1.0 if abs(jammer_frequency_hz - f_r) <= 0.5 * b_r else 0.0
        f_j, b_j = jammer_frequency_hz, jammer_bandwidth_hz
        low_j, high_j = f_j - 0.5 * b_j, f_j + 0.5 * b_j
        low_r, high_r = f_r - 0.5 * b_r, f_r + 0.5 * b_r
        if high_j <= low_r or low_j >= high_r:
            return 0.0
        overlap = min(high_j, high_r) - max(low_j, low_r)
        return min(overlap / (high_j - low_j), 1.0)

    @staticmethod
    def _pattern_factor(
        off_axis_deg: float, beam_width_deg: float, sidelobe_linear: float
    ) -> float:
        """天线方向图因子 —— **逐字**等于 ``equation.antenna_pattern_factor``。"""
        if beam_width_deg <= 0.0:
            return 1.0
        half = beam_width_deg * 0.5
        off = abs(off_axis_deg)
        if off > half:
            return sidelobe_linear
        return cos(pi * 0.25 * off / half) ** 2

    @staticmethod
    def _polarization(
        receiver_polarization: str, transmitter_polarization: str, override: float
    ) -> float:
        """``F_POL`` —— **逐字**等于 ``equation.polarization_effect`` 的那张 7×7 表。

        ★ 这里**抄的是表，不是公式**：表本身是本项目从 AFSIM 源码照抄下来的
        事实（在 ``equation.py`` 里有完整出处），抄一份表比"另写一个近似公式"
        安全得多——但也正是"抄一份"最容易过期的东西，所以那张表的**唯一主人**
        仍然是 ``equation``，这里只是通信侧的一个只读镜像。
        """
        if override >= 0.0:
            return override
        if receiver_polarization == "default" or transmitter_polarization == "default":
            return 1.0
        try:
            row = _POLARIZATION_INDEX[receiver_polarization]
            column = _POLARIZATION_INDEX[transmitter_polarization]
        except KeyError:
            return 1.0               # 未知类型 ⇒ 不扣（与"没声明"同待遇）
        return _POLARIZATION_EFFECTS[row][column]

    @staticmethod
    def _jam_power(
        power_w: float,
        jammer_gain: float,
        victim_gain: float,
        frequency_hz: float,
        range_m: float,
        loss: float = 1.0,
        bandwidth_overlap: float = 1.0,
        polarization: float = 1.0,
    ) -> float:
        """``Jam.1`` —— **逐字**等于 ``equation.jamming_power``（只是把
        ``wavelength_m`` 换成由 ``frequency_hz`` 现取，免得门面持有光速常数）。
        """
        if range_m <= 0.0 or power_w <= 0.0 or loss <= 0.0 or frequency_hz <= 0.0:
            return 0.0
        wavelength = 299_792_458.0 / frequency_hz
        four_pi = 12.566370614359172
        return (
            power_w * jammer_gain * victim_gain * wavelength * wavelength
            * bandwidth_overlap * polarization
            / (four_pi * four_pi * range_m * range_m * loss)
        )

    # -- 阶段②：提交（比较新旧，变了才重建） --------------------------------

    def commit(self) -> bool:
        """把标记阶段的意图**提交**成生效拓扑。返回"拓扑真的变了没有"。

        ★ **没变化 ⇒ 立刻返回 ``False``，一个字节都不动**（用户裁定）：
        不重建邻接表、不推 epoch。这样"这一拍拓扑动过没有"这个问题，
        答案永远是 :attr:`epoch` 的上一次变化时刻。

        ★ 运行期**每拍调一次**（在传信之前）：通信组件的节拍里，先更新
        状态（谁在网里、谁被干扰），再 commit，最后才投递在途报文。
        """
        self.commits += 1
        if self._pending == self._live:
            return False
        self._live = set(self._pending)
        #: 图要重建——掉线集合变了，邻接表的**可达性**就变了。★ 注意：
        #: 邻接表本身（谁连谁）没变，变的是"能不能走"。而本项目把"谁被摘掉"
        #: 放在**寻路**里判（:meth:`_reachable` 跳过掉线节点），不在这里
        #: 删边——删边会真的改掉图结构，恢复时就得**重建回来**，而"重建回来"
        #: 是两份状态（显式边 / 全连通）各自的事，抄错一处就是静默的通断错误。
        #: 放在寻路里判，图的**结构**始终只有一份真相。
        self.epoch += 1
        return True

    def is_offline(self, entity_id: int) -> bool:
        """**已提交**的通行状态：这个节点现在是不是被摘掉了。

        ★ 读的是 :attr:`_live`（生效值），不是 :attr:`_pending`（意图值）：
        传信阶段看到的必须是**提交后**的拓扑，否则"先状态后传信"就白排了。
        """
        return entity_id in self._live

    @property
    def offline_nodes(self) -> frozenset[int]:
        """当前**生效**的掉线节点（诊断 / 复盘用）。"""
        return frozenset(self._live)

    # -- 阶段③：传信（用提交后的图寻路） ------------------------------------

    def deliver(self, now: int) -> int:
        """把**到期的在途报文**投进收件箱，返回投了几条。

        ★ 只在"路径真的通"的前提下投递：一条在途报文如果此刻从发送方到收件方
        **不通**（被压制 / 跨网 / 有一端掉线），它就**这一拍不投**，留在在途
        队列里等下一拍——这正是"通信可以被干扰"的最小落点（§5.15 裁定③），
        而且**不需要在发信那一刻判**：发的时候先收下（发信方看不出对方被压了，
        这本来就是真实的），投的时候按**此刻**的拓扑决定。

        ★ 直接调 :meth:`~milsim.services.store.EntityStore.deliver_due` 不行：
        那个方法只看 ``deliver_at <= now``，不看拓扑。于是"被干扰的节点照样
        收得到信"，而症状是"干扰对通信完全没用"。

        ★ **不丢包**（本版）：不通的报文留在在途队列里，等路通了再投。等第二步
        接上干扰门限后，"压制时长超过某阈值 ⇒ 丢弃"才是另一个决定（那需要
        超时参数，而本版没有）。

        ★ **只移走投出去的那些**：不通的报文**一个字节都不碰**。曾经写成
        "先把到期的全清掉、再挨个投"，于是不通的那些被静默删了——症状是
        "被干扰压制的信不是迟到，而是永远消失"，与"等下一拍"完全相反。
        """
        store = self._store
        if store is None:
            return 0
        due = store.due_messages(now)
        if not due:
            return 0
        delivered = 0
        for message in due:
            if not self.path_exists(message.sender_id, message.recipient_id):
                continue           # 不通 ⇒ 留在在途队列，下一拍再看
            store.post_message(
                message.sender_id,
                message.recipient_id,
                message.kind,
                sent_at=message.sent_at,
                deliver_at=message.deliver_at,
                payload=message.payload,
            )
            store.forget_due([message])
            delivered += 1
        self.delivered += delivered
        return delivered

    @property
    def is_empty(self) -> bool:
        """一个网都没建 ⇒ 通信**不做任何限制**（旧想定逐字不变的落点）。"""
        return not self._nets

    def network_of(self, entity_id: int) -> str | None:
        """这个实体属于哪个网；不在任何网里返 ``None``。"""
        return self._membership.get(entity_id)

    def path_exists(self, source_id: int, target_id: int) -> bool:
        """**此刻**从 ``source_id`` 到 ``target_id`` 通不通。

        ★ 用**提交后**的图（:attr:`_live`），并且**每次都重算**，不缓存路径
        （§5.15 裁定①）：被干扰的节点从可通行集合里摘掉之后，原来那条路没了，
        得看**还有没有别的路**——有就绕过去，没有才真断。缓存路径会让"绕路"
        变成"断网"，而两者的症状完全一样（信没到），只有诊断计数能分辨。

        判据（顺序即短路顺序）：

        1. **没建任何网** ⇒ ``True``（不限制，见 :attr:`is_empty`）；
        2. **同一个实体** ⇒ ``True``（自己发给自己恒通）；
        3. **有一个不在任何网里** ⇒ ``False``（没入网 = 没通信能力；
           ★ 这条与 AFSIM 一致：不在网里、又没显式 ``link`` 的 comm 通不到）；
        4. **不在同一个网** ⇒ ``False``（跨网要网关，本版没有）；
        5. **两端任一掉线** ⇒ ``False``（被干扰的节点发不出、也收不到）；
        6. 否则在（跳过掉线节点的）**有向图**上 BFS：找到 ⇒ ``True``。

        ``source == target`` 放在第 2 条，而不是让寻路自己处理：零长路径在
        "源已被摘掉"时会给出 ``False``，而"自己给自己"不该受掉线影响——那是
        一个恒真的事实，不是一条可断的链路。
        """
        self.queries += 1
        if self.is_empty:
            return True
        if source_id == target_id:
            return True

        net_src = self._membership.get(source_id)
        net_dst = self._membership.get(target_id)
        if net_src is None or net_dst is None or net_src != net_dst:
            self.blocked += 1
            return False
        if source_id in self._live or target_id in self._live:
            self.blocked += 1
            return False

        ok = target_id in self._reachable(net_src, source_id)
        if ok:
            self.routed += 1
        else:
            self.blocked += 1
        return ok

    def _reachable(self, network: str, source_id: int) -> set[int]:
        """从 ``source_id`` 出发、**绕开掉线节点**能到达的全部节点。

        BFS（不是 A*）：本层只问"通不通"，不问"哪条路最短"——路由选哪条路
        对任务级仿真没有可观察后果，而引入代价函数就得先定义"边权是什么"
        （距离？带宽？拥塞？），那是一个我们现在答不了、也不该假装能答的问题。
        """
        table = self._links.get(network, {})
        seen = {source_id}
        queue = deque([source_id])
        while queue:
            node = queue.popleft()
            for nxt in table.get(node, ()):
                if nxt in seen or nxt in self._live:
                    continue
                seen.add(nxt)
                queue.append(nxt)
        return seen

    def route(self, source_id: int, target_id: int) -> list[int] | None:
        """从 ``source_id`` 到 ``target_id`` 的**一条可行路径**（诊断用）。

        与 :meth:`path_exists` 共用同一张（跳过掉线节点的）图，所以这里给出的
        路一定可通行。返回 ``None`` = 不通。用途：复盘"这一拍信是绕哪走的"。

        ★ 邻居按 **ID 升序**遍历：BFS 的分支顺序确定 ⇒ 同一份想定两次跑出
        同一条路。用 ``set`` 的天然顺序会让路径随哈希值变。
        """
        if source_id == target_id:
            return [source_id]
        network = self._membership.get(source_id)
        if network is None or self._membership.get(target_id) != network:
            return None
        if source_id in self._live or target_id in self._live:
            return None

        table = self._links.get(network, {})
        prev: dict[int, int] = {source_id: source_id}
        queue = deque([source_id])
        while queue:
            node = queue.popleft()
            if node == target_id:
                break
            for nxt in sorted(table.get(node, ())):
                if nxt in prev or nxt in self._live:
                    continue
                prev[nxt] = node
                queue.append(nxt)
        if target_id not in prev:
            return None
        path = [target_id]
        while path[-1] != source_id:
            path.append(prev[path[-1]])
        path.reverse()
        return path

    # -- 诊断 --------------------------------------------------------------

    def networks(self) -> list[str]:
        """已声明的网名，**排序**（可复现）。"""
        return sorted(self._nets)

    def members_of(self, network: str) -> tuple[int, ...]:
        net = self._nets.get(network)
        if net is None:
            return ()
        return net.members + net.auto_members

    def statistics(self) -> dict[str, Any]:
        return {
            "networks": len(self._nets),
            "nodes": len(self._membership),
            "offline": len(self._live),
            "epoch": self.epoch,
            "commits": self.commits,
            "queries": self.queries,
            "routed": self.routed,
            "blocked": self.blocked,
            "delivered": self.delivered,
            #: 第二步（干扰）的两个计数。``screened`` = 求值过的节点次；
            #: ``jammed`` = 其中判成被压住的。两个都要报——只报"压住了几个"
            #: 的话，"没有干扰机在册"与"干扰机功率不够"长得一模一样。
            "screened": self.screened,
            "jammed": self.jammed,
        }

    def __repr__(self) -> str:
        return (
            f"<CommService {len(self._nets)} 个网 / {len(self._membership)} 个节点"
            f" / {len(self._live)} 个掉线 / epoch {self.epoch}>"
        )


__all__ = [
    "CommNet",
    "CommNodeSpec",
    "CommService",
]
