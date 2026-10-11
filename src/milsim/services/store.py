"""实体状态存储：唯一的真相源，按视图收窄访问。

为什么存储要集中
----------------
态势展示与模型读的是**同一份数据**，天然不会出现"显示的和算的不一致"。
调试推演时这条价值极大——看到的位置就是算出来的位置，没有第二份缓存要同步。

但**存储集中 ≠ 接口集中**
-------------------------
如果所有模型都拿到同一个 store 并直接 ``store.x[eid] = ...``，这个类会长成
几百个方法、几十个字段的上帝对象：改任何一处数据结构所有模型都要检查，
雷达模型理论上能改弹药、改战损——写权限失控。

所以 store 是私有的，模型拿到的是**视图对象**（:class:`SensorView` 等）。
以 :class:`EngagementView` 为例，它上面**没有** ``set_damage`` 这个方法——
不是"约定不能调"，是根本不存在。这比写一条"禁止修改他人状态"的文档约定
可靠得多。

可以给对方送东西，不能替对方改状态
----------------------------------
| 交互 | 能否写对方的域 | 做法 |
|---|---|---|
| 下达命令、投递消息 | ✅ | 写进对方的**收件箱**，由对方自己处理 |
| 扣血、改位置、改弹药 | ❌ | 写"待裁决效果"，由裁决器统一结算 |

第二条若允许直接写，同一时间步里两个模型都打 C 时，"谁先执行"决定了 C 是被
打死还是挨两下——这个顺序在事件乱序或并行时不确定，推演结果就不可复现。

收件箱是数据字段，不是第二个队列
--------------------------------
**全局只有引擎一个调度队列。** 队友的"收件箱"不参与调度，它是实体状态的
一部分，由实体在自己的节拍里读取。队列管时间，数据字段管内容，两者不重叠。

位置为什么用结构数组
--------------------
实体 ID 直接当数组下标，一个实体的位置是 5 次数组索引而不是 5 次字典查找加
对象属性访问。更重要的是能直接喂给 numpy：::

    table.x[:n] += table.speed_mps[:n] * np.cos(np.radians(table.heading_deg[:n])) * dt

这是数据并行的落点，用 ``dict`` 的话这块优化无从谈起。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, sqrt
from typing import Any, Callable, Iterator, Sequence

import numpy as np

from .map.hex import Axial, heading_to_offset
from .spatial import LayeredSpatialIndex, bearing_of_offset, cells_in_disk

#: 相邻格中心距与边长的比值（pointy-top 六边形）。粗筛步数换算用。
_SQRT3 = sqrt(3.0)

#: 初始容量。太小会频繁倍增，太大浪费内存——256 个实体的数组只有 10 KB。
DEFAULT_CAPACITY = 256

#: 完好度上下限。裁决器据此判存活。
DAMAGE_INTACT = 1.0
DAMAGE_DESTROYED = 0.0

#: :attr:`Contact.origin_id` 的哨兵值：这条航迹由**本平台自己的传感器**量测。
#: 用 ``-1`` 而不是 ``0`` —— 实体 ID 从 0 开始，用 0 当哨兵会与"实体 0 测到的"
#: 撞在一起，而那种撞法不报错，只让防回环判据偶尔失效。
ORIGIN_LOCAL = -1

#: 幽灵（假目标）航迹的 **ID 空间起点**：从 ``-1`` 往下（``-1, -2, -3 …``）。
#:
#: 假目标**不对应任何实体**——AFSIM 的原话是 "generate and maintain false
#: target representations **without creating a platform instance for each
#: false target**"。所以它们需要一段自己的 ID：实体 ID 一律非负，负号就是
#: "这不是谁"。
#:
#: ★ 与 :data:`ORIGIN_LOCAL` 同值（也是 ``-1``）**不是冲突**：那是 ``origin_id``
#: 字段上的哨兵，这是 ``target_id`` 上的空间。但正因为两个哨兵长得一样，读代码
#: 时**不要**拿 ``target_id == ORIGIN_LOCAL`` 去判幽灵——判幽灵只看
#: :attr:`Contact.phantom`，那是唯一有语义的出处。
PHANTOM_ID_BASE = -1


def phantom_id(index: int) -> int:
    """第 ``index`` 条幽灵的 ID（``index`` 从 0 起）。**只此一处算它。**"""
    return PHANTOM_ID_BASE - index

#: 一条航迹最多允许经过多少次通信转发，超了不再同化。
#:
#: ★ 这是**防护常量，不是物理量**。AFSIM 的同名子系统里没有这一项，它靠
#: ``circular_report_rejection`` 断环；我们两道都留：``origin_id`` 断环
#: （见 :meth:`EntityStore.relay_contact`），跳数上限兜"环外的雪崩"——
#: 报文每转发一次就多一个收信人，没有上限时一次三角转发就能让航迹表按
#: 指数涨，而**表涨了不会报错**，只是内存慢慢爬。
#: 4 跳对任务级编队（旅-营-连-平台）够用：正常的航迹共享只有"上报 + 下发"两跳。
MAX_RELAY_HOPS = 4

#: 航迹报告的报文种类。**这条绳子只有一处系**——发送端（``share_tracks``）
#: 与接收端（``TrackManager``）都引用这个常量，各写一份字面量时改一边就静默失配。
TRACK_REPORT = "track_report"

#: **指挥指令**的报文种类（§5.15，用户裁定"Message 加指挥指令"）。
#: ★ 与 ``TRACK_REPORT`` 一样只有一处系。指挥件发它、受令方按种类取它——
#: 同一实体上的航迹管理器与指挥控制器**共用一个收件箱**，各按种类取才不会
#: 互相吃掉（这就是 :meth:`EntityStore.take_messages_of_kind` 存在的原因）。
COMMAND_ORDER = "command_order"


# ---------------------------------------------------------------------------
# 格归属
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class LocalCellRef:
    """局部层（战区）的格归属。

    实体进入战区后落在某个战区的某一层网格上。``zone_id`` 与 ``layer``
    一起构成 :class:`~milsim.services.spatial.LayeredSpatialIndex` 的层键。

    ``level`` 是**垂向层**（§3.11），0 = 地表带。它与 ``layer`` 无关：

    - ``layer`` 是**分辨率层**（LOD），边长逐层翻倍
    - ``level`` 是**垂向层**，按高度带切（低空 / 中空 / 高空）

    两个概念都叫"层"，所以字段名必须分开——混用不会报错，只会让实体落在
    错误的网格上。
    """

    zone_id: int
    layer: int
    axial: Axial
    #: 垂向层。地面、海面、水下都算 0；空中按高度带往上。
    level: int = 0

    def __post_init__(self) -> None:
        if self.zone_id < 0:
            raise ValueError(f"局部层的 zone_id 必须非负，收到 {self.zone_id}")
        if self.layer < 0:
            raise ValueError(f"局部层的 layer 必须非负，收到 {self.layer}")
        if self.level < 0:
            raise ValueError(f"垂向层 level 必须非负，收到 {self.level}")

    def __str__(self) -> str:
        tail = f"/V{self.level}" if self.level else ""
        return f"zone{self.zone_id}/L{self.layer}{tail}({self.axial.q},{self.axial.r})"


@dataclass(frozen=True, slots=True)
class GlobalCellRef:
    """全球层（H3 球面单元）的格归属。跨洲航渡中的实体属于这一层。

    特意**不**与局部层混在一个索引里：两层的格子粒度差几个数量级，
    悄悄合并会让调用方拿到语义不明的结果（"距离 40 格"是 40 km 还是 400 km）。
    """

    h3_cell: int

    def __str__(self) -> str:
        return f"H3({self.h3_cell:x})"


CellRef = LocalCellRef | GlobalCellRef


def _grow(array: np.ndarray, needed: int) -> np.ndarray:
    """把数组扩容到至少 ``needed``，内容保留，新槽位清零。

    倍增而不是"刚好够"：实体是逐个加入的，每次只加一个就重新分配会让
    装配阶段变成 O(n²)。
    """
    if needed <= array.size:
        return array
    capacity = array.size or DEFAULT_CAPACITY
    while capacity < needed:
        capacity *= 2
    grown = np.zeros(capacity, dtype=array.dtype)
    grown[: array.size] = array
    return grown


# ---------------------------------------------------------------------------
# 纯数据载体
# ---------------------------------------------------------------------------

#: 航迹的**敌我识别**状态（v0.13.33）。取值照抄 AFSIM 的
#: ``WsfTrack::IFF_Status`` 五个（``WsfTrack.hpp``），**含"未知"与"模糊"**——
#: 少了这两个就只能二选一，而"我根本没判出来"与"我判出来是敌人"是两件事。
IFF_UNKNOWN = "unknown"          # 未报告 / 判不出（= AFSIM cIFF_UNKNOWN）
IFF_AMBIGUOUS = "ambiguous"      # 判了但拿不准（= cIFF_AMBIGUOUS）
IFF_FOE = "foe"                  # 敌（= cIFF_FOE）
IFF_FRIEND = "friend"            # 友（= cIFF_FRIEND）
IFF_NEUTRAL = "neutral"          # 中立（= cIFF_NEUTRAL）

#: 五个取值的全集。**加一个值就要在这里加一处**：这是序列化校验的唯一出处。
IFF_KINDS = (IFF_UNKNOWN, IFF_AMBIGUOUS, IFF_FOE, IFF_FRIEND, IFF_NEUTRAL)


@dataclass(slots=True)
class Contact:
    """一条航迹。感知模型写给自己的，不是"真实目标"。

    含 ``quality`` 与位置估计，就是为了让误判、延迟、误差有地方存在——
    真实位置在 :class:`EntityStore` 的运动学表里，两者刻意分开。

    **航迹是"状态"不是"历史"**（AFSIM 的 ``track`` 块同理：``position`` /
    ``bearing`` / ``speed`` 全是当前值）。同一个目标重复探测是**覆盖更新**，
    所以一个平台的航迹表大小 = 它看到的**目标数**，与推演跑了多久无关。
    要历史得显式打开（AFSIM 的 ``retain_track_history``，默认不保留；我们
    连那个开关都还没做）。⇒ 用户担心的"一直探测就要一直保留、存储压力太大"
    在**状态**口径下不成立；真正的压力来自**不做清理**，见
    ``drop_after_inactive`` 那条（§5.13）。

    四个来源字段让"多渠道汇聚"这件事在数据里看得见：

    - ``origin_id`` —— 最初量测者（``ORIGIN_LOCAL`` = 本平台自己）
    - ``hops`` —— 经过几次通信转发
    - ``range_m`` / ``bearing_deg`` —— **相对最初量测者**的量测极坐标，
      不是相对当前持有者。转发链上每一跳都不重算，因为重算需要转发方的
      位置与时钟，而那不是"量测"、是**用别人的位置造一次新量测**。
    """

    #: 线格式的字段全集。``as_dict`` / ``from_dict`` 与它一起构成唯一格式定义。
    FIELDS = (
        "target_id",
        "quality",
        "detected_at",
        "bearing_deg",
        "range_m",
        "x",
        "y",
        "z",
        "origin_id",
        "hops",
        "phantom",
        "iff",
        "sigma_pos_m",
        "sigma_vel_mps",
        "vx",
        "vy",
        "vz",
    )

    target_id: int
    quality: float = 1.0
    detected_at: int = 0
    bearing_deg: float = 0.0
    range_m: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    #: 最初量测者。``ORIGIN_LOCAL``(-1) = 本平台传感器；其他值为转发的来源。
    origin_id: int = ORIGIN_LOCAL
    #: 通信转发跳数。``0`` = 本机量测。
    hops: int = 0
    #: **这条航迹是不是幽灵**（假目标）。``False`` = 真实量测。
    #:
    #: ★ 它是**唯一**判"幽灵"的出处：``target_id`` 落在负数段（
    #: :data:`PHANTOM_ID_BASE`）只是"给它们一段不撞实体的号"的实现细节，
    #: 而"这是假的"是一个**必须能上路**的性质——转给友邻之后，接收端拿到的
    #: 是一条 ``target_id`` 查不到实体的航迹，它凭这个字段才知道那是干扰造的，
    #: 而不是"报文坏了"或"目标已经不存在了"。
    #:
    #: ★ 本版**不拦幽灵的转发**（``TrackView.share_tracks`` 照发）：真雷达把
    #: 假目标当成航迹上报，正是假目标干扰想达到的效果——让**整个网络**都看到
    #: 不存在的目标。拦掉它等于把干扰的效果限制在单平台（§9-53）。
    phantom: bool = False
    #: **敌我识别状态**（v0.13.33）。取值见 :data:`IFF_KINDS`，照抄 AFSIM 的
    #: ``WsfTrack::IFF_Status``。默认 ``IFF_UNKNOWN`` —— "没判"是缺省，
    #: 不是"友"。
    #:
    #: ★ 它的**唯一**来源是量测那一刻的判据（传感器取本平台与目标的阵营，
    #: 同方为友、异方为敌），**不是从真值查出来的**：真实的 IFF 是独立的
    #: 询问/应答系统，AFSIM 也把它简化成想定给定的态势表（§9-57）。
    #: ★ 与 ``phantom`` 一样**要跟着上路**：转发链上每一跳原样保留。洗掉它的
    #: 症状是"友邻把你的敌人当成了自己人"，而位置、时间戳全都正常。
    iff: str = IFF_UNKNOWN
    #: **位置不确定度**（米，1σ 标量，``hypot(σ_R, R·σ_θ)``——距离与切向两支
    #: 合成一个各向同性等效值）。``0`` = **未知**：传感器没开误差模型（两条
    #: σ 参数/``compute_measurement_errors`` 都没给），这条航迹**不参与融合**
    #: ——没有"精度"就没有权重，融合退化成覆盖更新。标量而不是 (σ_θ, σ_R)
    #: 两支的理由：融合要跨来源加权，各支误差椭圆随各自量测者的朝向转，
    #: 迭代融合之后"椭圆相对谁的朝向"就没有意义了；迹（矩阵）不变量
    #: （位置的总均方误差）才是跨来源可加的量。
    sigma_pos_m: float = 0.0
    #: **速度估计的不确定度**（m/s，1σ 标量）。``0`` = 未知 ⇒ 该航迹**不敢
    #: 当对齐的基准**（外推误差没有界），融合对它退化为覆盖更新。
    sigma_vel_mps: float = 0.0
    #: **速度估计**（m/s）——传感器用**相邻两次带噪量测**差分得出，不是真值。
    #: 零向量 = 未知（首次量测没有差分的基准）。融合用它把旧估计**时间对齐**
    #: 到来者时刻，再按精度加权平均；没有它，跨时刻的两个位置直接平均会把
    #: 动目标抹糊（300 m/s 的目标隔 2 s 就是 600 m 的系统性错位）。
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    def as_dict(self, *, sender_id: int) -> dict[str, Any]:
        """线格式：**扁平 dict，键名与字段同名**。

        ★ 这是航迹"上路"的**唯一**形状定义处——发送端打包与接收端解包
        都走这里，写得下一份就会有两份格式定义，而格式一旦分叉不会报错：
        缺的字段在接收端变成默认值 0，位置静静跑到原点去。

        ★ ``sender_id`` 是必须的，因为 ``ORIGIN_LOCAL`` 是**本地编码**，
        上了线就没有意义：线上必须写"这是**哪个实体**测的"。原样把
        ``-1`` 发出去的话，接收端会把它读成"发送方的传感器测的"——
        于是转发方的名字顶替了真正的原始观察者，"这情报谁先看到的"
        从此查不出来，防回环也跟着失效（下一跳认不出这是自己的报告）。
        这条是本轮**测试先抓出来**的：报告从 A 发出、B 收到之后
        ``origin_id`` 是 ``-1`` 而不是 ``A``。
        """
        return {
            "target_id": self.target_id,
            "quality": self.quality,
            "detected_at": self.detected_at,
            "bearing_deg": self.bearing_deg,
            "range_m": self.range_m,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "origin_id": (
                sender_id if self.origin_id == ORIGIN_LOCAL else self.origin_id
            ),
            "hops": self.hops,
            "phantom": self.phantom,
            "iff": self.iff,
            "sigma_pos_m": self.sigma_pos_m,
            "sigma_vel_mps": self.sigma_vel_mps,
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "Contact":
        """从线格式还原。**缺字段报错，不补默认值；线上有哨兵也报错。**

        补默认值的后果是"报文少了一个 ``x``，航迹就落到原点"——报告出来的
        位置错得离谱，而参数、代码、日志全都正常。宁可当场炸。

        线上的 ``origin_id`` 必须是具体实体 ID：出现 ``ORIGIN_LOCAL``
        说明打包那一侧没做解析（见 :meth:`as_dict`），而它的症状是
        **来源被静默顶替**，不是报错。所以这里也要拦一道。
        """
        missing = [name for name in cls.FIELDS if name not in payload]
        if missing:
            raise KeyError(
                f"航迹报文缺字段 {'、'.join(sorted(missing))}——不补默认值，"
                "否则缺的那个会变 0 并静默产生一条位置错误的航迹"
            )
        unknown = [name for name in payload if name not in cls.FIELDS]
        if unknown:
            raise KeyError(
                f"航迹报文含未知字段 {'、'.join(sorted(unknown))}——"
                "格式分叉了，接收端不猜"
            )
        if int(payload["origin_id"]) == ORIGIN_LOCAL:
            raise ValueError(
                "航迹报文的 origin_id 是 ORIGIN_LOCAL——线上必须写具体实体 ID。"
                "原样发哨兵会让接收端把转发方当成原始观察者（来源被静默顶替）"
            )
        if payload["iff"] not in IFF_KINDS:
            raise ValueError(
                f"航迹报文的 iff 是 {payload['iff']!r}，不在 {IFF_KINDS} 里——"
                "线上不猜未知取值，否则接收端会把一个没定义的状态当成'友'"
            )
        return cls(**{name: payload[name] for name in cls.FIELDS})

    @property
    def is_local(self) -> bool:
        """是否本平台传感器直接量测（没经过通信）。"""
        return self.origin_id == ORIGIN_LOCAL


@dataclass(slots=True)
class Message:
    """投递给某个实体的消息。

    ``sent_at`` 与 ``deliver_at`` 分开，是通信延迟能建模的前提：指挥模型只负责
    写 ``sent_at``，通信模型决定何时改写 ``deliver_at`` 并投进收件箱。
    """

    msg_id: int
    sender_id: int
    recipient_id: int
    kind: str
    sent_at: int
    deliver_at: int
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class Effect:
    """待裁决的效果意图。

    **这是意图不是结果**——交战模型提交"我打中了、预期毁伤 0.3"，扣不扣、
    扣多少由裁决器结合目标抗性统一算。多个模型同刻提交的效果因此与提交
    顺序无关。
    """

    source_id: int
    target_id: int
    kind: str
    magnitude: float
    at: int = 0
    note: str = ""


# ---------------------------------------------------------------------------
# 运动学表：结构数组
# ---------------------------------------------------------------------------

class KinematicsTable:
    """位置 / 姿态 / 速度的 SoA 存储。实体 ID 直接当数组下标。

    注销时把槽位标记为"未使用"，**不回收下标**——回收会让 ID → 下标不再
    一一对应，所有按 ID 索引的数组都要跟着重排。
    """

    __slots__ = ("x", "y", "z", "heading_deg", "speed_mps")

    def __init__(self, capacity: int = DEFAULT_CAPACITY) -> None:
        capacity = max(1, int(capacity))
        self.x = np.zeros(capacity, dtype=np.float64)
        self.y = np.zeros(capacity, dtype=np.float64)
        self.z = np.zeros(capacity, dtype=np.float64)
        self.heading_deg = np.zeros(capacity, dtype=np.float64)
        self.speed_mps = np.zeros(capacity, dtype=np.float64)

    @property
    def capacity(self) -> int:
        return int(self.x.size)

    def reserve(self, needed: int) -> bool:
        """确保能容纳下标 ``needed``，返回是否发生了扩容。"""
        if needed < self.capacity:
            return False
        self.x = _grow(self.x, needed + 1)
        self.y = _grow(self.y, needed + 1)
        self.z = _grow(self.z, needed + 1)
        self.heading_deg = _grow(self.heading_deg, needed + 1)
        self.speed_mps = _grow(self.speed_mps, needed + 1)
        return True

    def position(self, entity_id: int) -> tuple[float, float, float]:
        """返回**元组的副本**，不是数组视图。

        返回视图的话，调用方拿到 ``arr`` 后 ``arr[0] = 999`` 就能改掉别人的
        位置，视图权限就白设了。
        """
        return (
            float(self.x[entity_id]),
            float(self.y[entity_id]),
            float(self.z[entity_id]),
        )

    def advance(self, entity_id: int, dt_s: float) -> None:
        """按当前航向与速度推进一个实体的位置。

        **用 ``math`` 而不是 ``numpy`` 标量**：numpy 的 ufunc 对单个标量
        有固定的包装开销，实测比 ``math`` 慢 1.5 倍。而本方法是每帧每实体
        各调一次的路径，这个开销是实打实的。

        批量推进用 :meth:`advance_all`——那边向量化能到 90 倍以上，
        与这里不是同一个量级的优化。
        """
        speed = float(self.speed_mps[entity_id])
        dx, dy = heading_to_offset(float(self.heading_deg[entity_id]), speed * dt_s)
        self.x[entity_id] += dx
        self.y[entity_id] += dy

    def advance_all(self, count: int, dt_s: float) -> None:
        """批量推进前 ``count`` 个实体。数据并行的入口。

        5000 实体推进 200 帧实测比逐个调用快 **93.6x**——省下的是几千次
        Python 函数调用与属性访问，不是并行收益。
        """
        if count <= 0:
            return
        n = min(count, self.capacity)
        # 与 advance() 用同一套几何：dx = d·sinθ，dy = -d·cosθ。
        # 两处各推一遍公式迟早会推歪一个，所以这里把推导写成一行向量式
        distance = self.speed_mps[:n] * dt_s
        rad = np.radians(self.heading_deg[:n])
        self.x[:n] += distance * np.sin(rad)
        self.y[:n] -= distance * np.cos(rad)

    def __repr__(self) -> str:
        return f"<KinematicsTable 容量 {self.capacity}>"


# ---------------------------------------------------------------------------
# 主存储
# ---------------------------------------------------------------------------

class EntityStore:
    """实体状态存储。线程不安全——只在引擎主线程使用。

    与 :class:`~milsim.services.registry.EntityRegistry` 的分工按**变更频率**
    切：注册表管身份（低频），本类管状态（高频）。

    用法::

        store = EntityStore()
        store.add(entity_id, LocalCellRef(1, 0, Axial(10, 0)))
        store.set_pose(entity_id, 1732.0, 0.0, 500.0, 90.0, 250.0)

        view = store.sensor_view(entity_id)        # 交给感知模型
        for tid in view.query_cone(direction=0, max_steps=20):
            ...
    """

    __slots__ = (
        "_kinematics",
        "_damage",
        "_damage_capacity",
        "_active",
        "_cell",
        "_local",
        "_global",
        "_contacts",
        "_inbox",
        "_inflight",
        "_loadout",
        "_effects",
        "_msg_seq",
        "_relayed",
        "_relay_rejected",
    )

    def __init__(self, capacity: int = DEFAULT_CAPACITY) -> None:
        self._kinematics = KinematicsTable(capacity)
        self._damage = np.ones(max(1, capacity), dtype=np.float64)
        self._damage_capacity = int(self._damage.size)
        #: 哪些下标是"在用"的。销毁后置 False，但下标不回收。
        self._active: set[int] = set()
        self._cell: dict[int, CellRef] = {}
        self._local = LayeredSpatialIndex()
        self._global: dict[int, list[int]] = {}
        #: entity_id → 自己探测到的航迹。**私有域**，别人的视图看不到。
        self._contacts: dict[int, list[Contact]] = {}
        #: entity_id → 待收消息。**投递域**，任何发起方都能写。
        self._inbox: dict[int, list[Message]] = {}
        #: 在途消息。有通信建模时消息先到这里，由通信模型按时搬进收件箱。
        #: 直接写收件箱等于通信瞬时且无损，那就不叫任务级仿真了。
        self._inflight: list[Message] = []
        #: entity_id → {物资名: 数量}。弹药、油量这类。
        self._loadout: dict[int, dict[str, float]] = {}
        #: 全局待裁决效果。只有裁决器消费。
        self._effects: list[Effect] = []
        self._msg_seq = 0
        #: 同化成功的转发报告数（诊断用）。
        self._relayed = 0
        #: 被拒的转发报告数，按原因分类。**"没反应"要能查出来是哪一道挡的**，
        #: 否则"我发了报告它没收到"只能靠猜。
        self._relay_rejected: dict[str, int] = {
            "circular": 0,
            "hops": 0,
            "stale": 0,
        }

    # -- 登记与注销 --------------------------------------------------------

    def add(self, entity_id: int, cell: CellRef | None = None) -> None:
        """登记实体，初始化状态槽位。

        重复登记是幂等的（不重复扣初始化），这样装配阶段不必小心翼翼地
        去重——重复 ``add`` 不会把已经受损的实体又还原成完好。
        """
        if entity_id in self._active:
            if cell is not None:
                self.set_cell(entity_id, cell)
            return

        self._reserve(entity_id)
        self._active.add(entity_id)
        self._damage[entity_id] = DAMAGE_INTACT
        self._kinematics.x[entity_id] = 0.0
        self._kinematics.y[entity_id] = 0.0
        self._kinematics.z[entity_id] = 0.0
        self._kinematics.heading_deg[entity_id] = 0.0
        self._kinematics.speed_mps[entity_id] = 0.0

        if cell is not None:
            self.set_cell(entity_id, cell)

    def remove(self, entity_id: int) -> bool:
        """注销实体，从**所有**索引与数据域里摘干净。

        漏摘任何一个都会留下"幽灵实体"——它已经不在名册上，却仍然出现在
        ``entities_at(cell)`` 的结果里，雷达于是能探测到一个不存在的目标。
        这是本模块最容易出的 bug，``validate()`` 专门守它。
        """
        if entity_id not in self._active:
            return False

        cell = self._cell.pop(entity_id, None)
        if isinstance(cell, LocalCellRef):
            self._local.layer(cell.zone_id, cell.layer).remove(entity_id)
        elif isinstance(cell, GlobalCellRef):
            self._detach_global(entity_id, cell.h3_cell)

        self._active.discard(entity_id)
        self._contacts.pop(entity_id, None)
        self._inbox.pop(entity_id, None)
        self._loadout.pop(entity_id, None)
        self._damage[entity_id] = DAMAGE_INTACT        # 槽位还原，等 ID 复用时是干净的
        self._kinematics.speed_mps[entity_id] = 0.0

        # 收发双方已不存在的消息与效果一并丢弃。留着会让下一帧的裁决器去访问
        # 一个不存在的实体，也让 validate() 报出"效果目标已注销"这种无从处理的错。
        if self._inflight:
            self._inflight = [
                m
                for m in self._inflight
                if m.sender_id != entity_id and m.recipient_id != entity_id
            ]
        if self._effects:
            self._effects = [
                e
                for e in self._effects
                if e.source_id != entity_id and e.target_id != entity_id
            ]
        return True

    def _reserve(self, entity_id: int) -> None:
        if entity_id >= self._kinematics.capacity:
            self._kinematics.reserve(entity_id)
        if entity_id >= self._damage_capacity:
            self._damage = _grow(self._damage, entity_id + 1)
            self._damage_capacity = int(self._damage.size)

    def _detach_global(self, entity_id: int, h3_cell: int) -> None:
        bucket = self._global.get(h3_cell)
        if bucket is None:
            return
        try:
            bucket.remove(entity_id)
        except ValueError:
            return
        if not bucket:
            del self._global[h3_cell]

    def clear(self) -> None:
        capacity = self._kinematics.capacity
        self._kinematics = KinematicsTable(capacity)
        self._damage = np.ones(max(1, capacity), dtype=np.float64)
        self._damage_capacity = int(self._damage.size)
        self._active.clear()
        self._cell.clear()
        self._local.clear()
        self._global.clear()
        self._contacts.clear()
        self._inbox.clear()
        self._inflight.clear()
        self._loadout.clear()
        self._effects.clear()
        self._relayed = 0
        for key in self._relay_rejected:
            self._relay_rejected[key] = 0

    # -- 位置与姿态 --------------------------------------------------------

    def set_pose(
        self,
        entity_id: int,
        x: float,
        y: float,
        z: float,
        heading_deg: float = 0.0,
        speed_mps: float = 0.0,
    ) -> None:
        """写位置与姿态。**不**碰格归属——那是 :meth:`set_cell` 的事。

        分开是为了让调用方决定何时付"跨格检测"的代价：高速移动的实体可以
        每帧写位置、每几帧才更新一次格归属。
        """
        self._require(entity_id)
        self._kinematics.x[entity_id] = x
        self._kinematics.y[entity_id] = y
        self._kinematics.z[entity_id] = z
        self._kinematics.heading_deg[entity_id] = heading_deg
        self._kinematics.speed_mps[entity_id] = speed_mps

    def set_cell(self, entity_id: int, cell: CellRef) -> bool:
        """更新格归属，同步空间索引。返回**是否发生了跨格移动**。

        返回值让调用方只在真正跨格时才做后续处理（刷新雷达快照等），
        避免每帧重算。
        """
        self._require(entity_id)
        previous = self._cell.get(entity_id)
        if previous == cell:
            return False

        if isinstance(previous, LocalCellRef):
            self._local.layer(previous.zone_id, previous.layer).remove(entity_id)
        elif isinstance(previous, GlobalCellRef):
            self._detach_global(entity_id, previous.h3_cell)

        if isinstance(cell, LocalCellRef):
            self._local.layer(cell.zone_id, cell.layer).add(entity_id, cell.axial)
        else:
            bucket = self._global.get(cell.h3_cell)
            if bucket is None:
                self._global[cell.h3_cell] = [entity_id]
            else:
                bucket.append(entity_id)

        self._cell[entity_id] = cell
        return previous is not None

    def position_of(self, entity_id: int) -> tuple[float, float, float] | None:
        """位置副本。实体不存在时返回 ``None``——不抛异常。

        调对方位置时"目标已销毁"是常态而非异常，抛异常会逼每个调用点
        写 try。返回 ``None`` 让调用方自然地写 ``if pos is None: continue``。
        """
        if entity_id not in self._active:
            return None
        return self._kinematics.position(entity_id)

    def pose_of(
        self, entity_id: int
    ) -> tuple[float, float, float, float, float] | None:
        """(x, y, z, heading_deg, speed_mps) 副本。"""
        if entity_id not in self._active:
            return None
        return (
            *self._kinematics.position(entity_id),
            float(self._kinematics.heading_deg[entity_id]),
            float(self._kinematics.speed_mps[entity_id]),
        )

    def cell_of(self, entity_id: int) -> CellRef | None:
        return self._cell.get(entity_id)

    def has(self, entity_id: int) -> bool:
        return entity_id in self._active

    def _require(self, entity_id: int) -> None:
        if entity_id not in self._active:
            raise KeyError(f"实体 {entity_id} 不在存储中（未 add 或已 remove）")

    # -- 空间查询（局部层） ------------------------------------------------

    def entities_at(self, cell: CellRef) -> list[int]:
        """某个格子里的实体，**升序**。"""
        if isinstance(cell, LocalCellRef):
            return self._local.layer(cell.zone_id, cell.layer).entities_at(cell.axial)
        bucket = self._global.get(cell.h3_cell)
        return list(bucket) if bucket else []

    def entities_in_disk(self, center: LocalCellRef, radius: int) -> list[int]:
        """圆形区域内的实体，由近及远。"""
        index = self._local.layer(center.zone_id, center.layer)
        return index.entities_in_disk(center.axial, radius)

    def entities_in_cone(
        self,
        center: LocalCellRef,
        direction: int,
        max_steps: int,
        half_width: int = 0,
    ) -> list[int]:
        """锥形搜索。雷达扫描的粗筛入口，结果由近及远。"""
        index = self._local.layer(center.zone_id, center.layer)
        return index.entities_in_cone(
            center.axial, direction, max_steps, half_width
        )

    def entities_in_sector(
        self,
        center: LocalCellRef,
        bearing_start: float,
        bearing_end: float,
        max_steps: int,
    ) -> list[int]:
        """方位角扇区内的实体，由近及远。旋转天线的"这一帧照到哪一片"用它。"""
        index = self._local.layer(center.zone_id, center.layer)
        return index.entities_in_sector(
            center.axial, bearing_start, bearing_end, max_steps
        )

    def occupied_cells(self, zone_id: int, layer: int) -> int:
        """某一层里有实体的格子数。层不存在返回 0，**不**创建空层。"""
        key = (zone_id, layer)
        if key not in self._local:
            return 0
        return len(self._local.layer(zone_id, layer))

    # -- 损耗：裁决器的专属域 ----------------------------------------------

    def damage_of(self, entity_id: int) -> float:
        """完好度，1.0 = 完好，0.0 = 摧毁。"""
        if entity_id not in self._active:
            return DAMAGE_DESTROYED
        return float(self._damage[entity_id])

    def is_alive(self, entity_id: int) -> bool:
        if entity_id not in self._active:
            return False
        return bool(self._damage[entity_id] > DAMAGE_DESTROYED)

    def apply_damage(self, entity_id: int, amount: float) -> float:
        """**只有裁决器该调这个方法。** 返回施加后的完好度。

        裁决器不通过视图访问——它是引擎侧的系统，直接持有 store。
        视图里没有对应方法，所以模型层的任何部件都改不了别人的损耗。
        """
        self._require(entity_id)
        current = max(DAMAGE_DESTROYED, float(self._damage[entity_id]) - abs(amount))
        self._damage[entity_id] = current
        return current

    def set_damage(self, entity_id: int, value: float) -> None:
        """直接设完好度。裁决器与想定初始化用。"""
        self._require(entity_id)
        self._damage[entity_id] = min(DAMAGE_INTACT, max(DAMAGE_DESTROYED, value))

    def alive_ids(self) -> list[int]:
        """所有存活实体，升序。"""
        return [eid for eid in sorted(self._active) if self.is_alive(eid)]

    # -- 三维范围查询 ------------------------------------------------------

    def entities_in_ball(
        self,
        center: tuple[float, float, float],
        radius_m: float,
        *,
        ref: LocalCellRef,
        cell_size_m: float,
        alive_only: bool = True,
    ) -> list[tuple[int, float]]:
        """三维球形范围内的实体，返回 ``[(entity_id, 距离米)]`` 按距离升序。

        ``ref`` 是**球心所在格的归属**（由装配层注入的 locator 算出来），
        ``cell_size_m`` 是该战区的网格边长。

        做法是**先按二维格取候选、再按精确三维距离精筛**，这样是完备的：

            三维距离 ≥ 水平距离  ⇒  三维半径 R 的球必落在水平半径 R 的圆内

        所以用水平圆取候选**不会漏掉任何目标**（§3.11）。反过来做
        （用三维格筛候选）不会更准，只会更慢。

        **不要拿"同一格"当命中判据。** 格边长 100 m ~ 2 km，而 155 mm 榴弹
        的破片半径只有 10~30 m、250 kg 航弹 50~100 m——"同格"会把 100 m 外
        的目标算成被命中，同时漏掉邻格内的；而且它不含高度，3 楼爆炸与
        地下室同格。网格只负责**筛候选**，判定必须用这里的精确距离。

        同距离时按 ID 升序，保证结果顺序可复现。
        """
        if radius_m <= 0.0:
            return []
        if cell_size_m <= 0.0:
            raise ValueError(f"网格边长必须为正，实际 {cell_size_m}")

        # 步数换算：相邻格中心距 = 边长 × √3（pointy-top，见 path.step_cost）。
        # 多取一圈是刻意的——候选里少一个，后面的精筛再准也救不回来。
        radius_cells = int(ceil(radius_m / (cell_size_m * _SQRT3))) + 1
        index = self._local.layer(ref.zone_id, ref.layer)
        candidates = index.entities_in(cells_in_disk(ref.axial, radius_cells))

        cx, cy, cz = center
        out: list[tuple[int, float]] = []
        for entity_id in candidates:
            if alive_only and not self.is_alive(entity_id):
                continue
            position = self.position_of(entity_id)
            if position is None:
                continue
            dx = position[0] - cx
            dy = position[1] - cy
            dz = position[2] - cz
            distance = sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= radius_m:
                out.append((entity_id, distance))

        out.sort(key=lambda pair: (pair[1], pair[0]))
        return out

    def horizontal_distance_m(
        self, entity_id: int, center: tuple[float, float]
    ) -> float | None:
        """到某点的**水平**距离。用于快速粗判，不必构造球查询。"""
        position = self.position_of(entity_id)
        if position is None:
            return None
        dx = position[0] - center[0]
        dy = position[1] - center[1]
        return sqrt(dx * dx + dy * dy)

    # -- 待裁决效果 --------------------------------------------------------

    def request_effect(self, effect: Effect) -> None:
        """提交一个待裁决效果。任何模型都能提交，但只有裁决器会消费。"""
        self._effects.append(effect)

    def pending_effects(self) -> list[Effect]:
        """待裁决效果，**提交顺序**。裁决器结算后要调 :meth:`clear_effects`。"""
        return list(self._effects)

    def clear_effects(self) -> int:
        """清空待裁决队列，返回清掉多少条。"""
        count = len(self._effects)
        self._effects.clear()
        return count

    # -- 航迹（各模型私有域） ----------------------------------------------

    def add_contact(
        self,
        owner_id: int,
        target_id: int,
        quality: float = 1.0,
        detected_at: int = 0,
        bearing_deg: float = 0.0,
        range_m: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        origin_id: int = ORIGIN_LOCAL,
        hops: int = 0,
        phantom: bool = False,
        iff: str = IFF_UNKNOWN,
        sigma_pos_m: float = 0.0,
        sigma_vel_mps: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
    ) -> Contact:
        """把一条航迹写进**自己**的域。

        同一目标的重复探测会更新已有航迹而不是追加——否则每帧探测一次，
        航迹表几秒内就能涨到几千条，而其中九成是同一个目标。

        ``origin_id`` / ``hops`` 默认就是"本平台传感器直接量测"。**从通信
        收到的报告不走这里**，走 :meth:`relay_contact`——那里才有防回环。

        ``phantom`` 见 :attr:`Contact.phantom`。★ **覆盖更新时它也要跟着写**：
        这一支是逐字段赋值的，漏掉一个字段的症状是"同一个目标上一拍是幽灵、
        这一拍变成真的（或反过来）"，而位置与时间戳全都正常。

        σ 与速度（v0.13.56）同一条纪律：**给了多少写多少，不给就写 0**。
        覆盖更新把 σ 一并覆盖——上一拍的精度不能冒充这一拍的（量测没了，
        精度也就没了）；要不要**用**这些 σ 做加权是 :meth:`merge_contact`
        的事，本方法只负责忠实记账。
        """
        self._require(owner_id)
        contacts = self._contacts.setdefault(owner_id, [])
        for existing in contacts:
            if existing.target_id == target_id:
                existing.quality = quality
                existing.detected_at = detected_at
                existing.bearing_deg = bearing_deg
                existing.range_m = range_m
                existing.x = x
                existing.y = y
                existing.z = z
                existing.origin_id = origin_id
                existing.hops = hops
                existing.phantom = phantom
                existing.iff = iff
                existing.sigma_pos_m = sigma_pos_m
                existing.sigma_vel_mps = sigma_vel_mps
                existing.vx = vx
                existing.vy = vy
                existing.vz = vz
                return existing

        contact = Contact(
            target_id=target_id,
            quality=quality,
            detected_at=detected_at,
            bearing_deg=bearing_deg,
            range_m=range_m,
            x=x,
            y=y,
            z=z,
            origin_id=origin_id,
            hops=hops,
            phantom=phantom,
            iff=iff,
            sigma_pos_m=sigma_pos_m,
            sigma_vel_mps=sigma_vel_mps,
            vx=vx,
            vy=vy,
            vz=vz,
        )
        contacts.append(contact)
        return contact

    def merge_contact(
        self,
        owner_id: int,
        target_id: int,
        *,
        quality: float,
        detected_at: int,
        bearing_deg: float,
        range_m: float,
        x: float,
        y: float,
        z: float,
        origin_id: int = ORIGIN_LOCAL,
        hops: int = 0,
        phantom: bool = False,
        iff: str = IFF_UNKNOWN,
        sigma_pos_m: float = 0.0,
        sigma_vel_mps: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
    ) -> tuple[Contact, str]:
        """按精度融合地写航迹：能融合就融合，不能就退化为覆盖更新。

        返回 ``(航迹, 处理方式)``，处理方式 ``"fused"``（加权融合）或
        ``"replaced"``（覆盖更新，与 :meth:`add_contact` 逐字同一行为）。

        **融合的门槛（缺一即退化，退化是常态而非异常）**：

        1. **双方都有位置 σ**——``sigma_pos_m`` 为 0 = 传感器没开误差模型，
           没有"精度"可加权。这是**兼容闸门**：没给 σ 参数的想定，行为与
           v0.13.55 逐字相同；
        2. **来者严格更新**（``detected_at`` 更大）——同刻报告不值得平均
           （权重 game 无信息增益时只添乱），旧的更不会被新报告倒灌；
        3. **旧航迹有速度估计**——融合先把旧估计**时间对齐**到来者时刻，
           没有速度就没有对齐手段，跨时刻的两个位置直接平均会把动目标
           抹糊（300 m/s 的目标隔 2 s 就是 600 m 的系统性错位）；
        4. **旧航迹的速度 σ 有限**——``sigma_vel_mps`` 为 0 = "不知道自己
           的速度估计有多准"，拿它外推是拿无界的误差当有界的用。

        融合本身是**逆方差加权**（§5.13.5）：旧估计按自身速度推到
        ``detected_at`` 时刻（对齐把速度误差也吃进 σ²），两个估计各按
        ``1/σ²`` 加权平均；融合后的 σ² 收缩为 ``1/(1/σ₁²+1/σ₂²)``——
        两个来源都看一眼，比只看其中一个准，这就是融合的全部收益来源。
        """
        self._require(owner_id)
        existing = self._find_contact(owner_id, target_id)
        if existing is not None:
            fused = self._try_fuse(
                existing,
                quality=quality,
                detected_at=detected_at,
                bearing_deg=bearing_deg,
                range_m=range_m,
                x=x,
                y=y,
                z=z,
                origin_id=origin_id,
                hops=hops,
                phantom=phantom,
                iff=iff,
                sigma_pos_m=sigma_pos_m,
                sigma_vel_mps=sigma_vel_mps,
                vx=vx,
                vy=vy,
                vz=vz,
            )
            if fused is not None:
                return fused, "fused"
        return (
            self.add_contact(
                owner_id,
                target_id,
                quality=quality,
                detected_at=detected_at,
                bearing_deg=bearing_deg,
                range_m=range_m,
                x=x,
                y=y,
                z=z,
                origin_id=origin_id,
                hops=hops,
                phantom=phantom,
                iff=iff,
                sigma_pos_m=sigma_pos_m,
                sigma_vel_mps=sigma_vel_mps,
                vx=vx,
                vy=vy,
                vz=vz,
            ),
            "replaced",
        )

    @staticmethod
    def _try_fuse(
        existing: Contact,
        *,
        quality: float,
        detected_at: int,
        bearing_deg: float,
        range_m: float,
        x: float,
        y: float,
        z: float,
        origin_id: int,
        hops: int,
        phantom: bool,
        iff: str,
        sigma_pos_m: float,
        sigma_vel_mps: float,
        vx: float,
        vy: float,
        vz: float,
    ) -> Contact | None:
        """对同目标的两条估计做一次融合，不可融合返回 ``None``。

        ★ 返回 ``None`` 时**一个字段都没动**——调用方随即走覆盖更新，
        两条路在"动没动数据"上不会出现第三种状态。
        """
        if sigma_pos_m <= 0.0 or existing.sigma_pos_m <= 0.0:
            return None
        if detected_at <= existing.detected_at:
            return None
        if existing.vx == 0.0 and existing.vy == 0.0 and existing.vz == 0.0:
            return None
        if existing.sigma_vel_mps <= 0.0:
            return None

        dt_s = (detected_at - existing.detected_at) / 1_000_000
        # 时间对齐：旧估计按它自己的速度估计推到来者时刻。对齐不是白拿的——
        # 速度估计有自己的误差，外推越远它吃进去越多（σ² += σv²·Δt²），
        # 于是"对不齐的两条"自动地几乎只剩来者的权重，融合温和地退化为
        # "新者胜"，不需要任何额外的窗口参数去硬卡。
        aligned_x = existing.x + existing.vx * dt_s
        aligned_y = existing.y + existing.vy * dt_s
        aligned_z = existing.z + existing.vz * dt_s
        var_a = existing.sigma_pos_m**2 + (existing.sigma_vel_mps * dt_s) ** 2
        var_b = sigma_pos_m**2
        w_a = 1.0 / var_a
        w_b = 1.0 / var_b
        w = w_a + w_b

        # 极坐标（range/bearing）与来源链（origin/hops）描述的是"这条量测
        # 是谁在什么几何关系下测的"——融合后的位置不再对应任何一条量测线，
        # 所以这两个字段跟**主贡献方**（权重大者）走，并在类文档里注明
        # "融合航迹的极坐标是主贡献量测的极坐标，不是融合位置的极坐标"。
        if w_a >= w_b:
            prim_bearing, prim_range = existing.bearing_deg, existing.range_m
            prim_origin, prim_hops = existing.origin_id, existing.hops
        else:
            prim_bearing, prim_range = bearing_deg, range_m
            prim_origin, prim_hops = origin_id, hops

        existing.quality = (w_a * existing.quality + w_b * quality) / w
        existing.detected_at = detected_at
        existing.bearing_deg = prim_bearing
        existing.range_m = prim_range
        existing.x = (w_a * aligned_x + w_b * x) / w
        existing.y = (w_a * aligned_y + w_b * y) / w
        existing.z = (w_a * aligned_z + w_b * z) / w
        existing.origin_id = prim_origin
        existing.hops = prim_hops
        # phantom / iff 是"这一刻这条航迹是什么"的判断，跟**来者**走：
        # 它是在更新时刻判的，比旧的判断新鲜。
        existing.phantom = phantom
        existing.iff = iff

        # 速度：同样逆方差加权；来者没给 σv（它的首次量测）时权重为 0，
        # 速度与 σv 原样保留——那是手里唯一的速度信息。
        if sigma_vel_mps > 0.0:
            iv_a = 1.0 / existing.sigma_vel_mps**2
            iv_b = 1.0 / sigma_vel_mps**2
            iv = iv_a + iv_b
            existing.vx = (iv_a * existing.vx + iv_b * vx) / iv
            existing.vy = (iv_a * existing.vy + iv_b * vy) / iv
            existing.vz = (iv_a * existing.vz + iv_b * vz) / iv
            existing.sigma_vel_mps = 1.0 / sqrt(iv)
        existing.sigma_pos_m = 1.0 / sqrt(w)
        return existing

    def relay_contact(
        self,
        owner_id: int,
        payload: dict[str, Any],
        *,
        sender_id: int,
        received_at: int,
        max_hops: int = MAX_RELAY_HOPS,
    ) -> tuple[Contact | None, str]:
        """把**通信收到的航迹报告**同化进自己的航迹表。

        返回 ``(航迹或 None, 处理结论)``。结论只用于诊断与测试——被拒是
        正常运行结果，不是异常，所以不抛。

        三道过滤，顺序固定（先便宜的后贵的，且**先判死的不再往下走**）：

        1. ``环形报告`` —— ``origin_id == owner_id`` 或 ``sender_id == owner_id``：
           这条航迹本来就是**我自己**量测的，绕了一圈又回来。收下会让"我自
           己的量测"变成"别人转述的报告"，来源标记被洗掉。对应 AFSIM 的
           ``circular_report_rejection``（**它默认 off，我们默认 on** ——
           理由见下）。
        2. ``跳数超限`` —— ``hops + 1 > max_hops``。
        3. ``不更新`` —— 已经有一条同样新或更新的同目标航迹（``detected_at``
           大于等于来者）。这一条不是"拒绝"，是**融合语义**：AFSIM 的
           ``fusion_method`` 默认 ``replacement``，"报告到达即替换"在实现上
           就是"新者胜"（旧报告替换掉新报告等于把情报往回倒，那不是融合）。

        ★ 为什么我们的环形报告拒收默认 on、AFSIM 默认 off：AFSIM 的默认值
        是在**有人工配置的作战网络**里定的，链路上跑的是"谁该收到什么"。
        我们的通信件还没有路由与订阅（§9-44），一条 ``track_report`` 会投给
        全部同方单位，自己发出去的那份**必然**绕回来。默认 off 的话，每个
        平台的航迹表都会被自己的报告刷一遍，而**来源标记全变成"转发"**——
        看起来一切正常，只有"本机量测"这个口径不见了。
        """
        origin = int(payload.get("origin_id", ORIGIN_LOCAL))
        if origin == owner_id or sender_id == owner_id:
            self._relay_rejected["circular"] += 1
            return None, "circular"

        incoming = Contact.from_dict(payload)
        if incoming.hops + 1 > max_hops:
            self._relay_rejected["hops"] += 1
            return None, "hops"

        previous = self._find_contact(owner_id, incoming.target_id)
        if previous is not None and previous.detected_at >= incoming.detected_at:
            self._relay_rejected["stale"] += 1
            return previous, "stale"

        self._relayed += 1
        contact, how = self.merge_contact(
            owner_id,
            incoming.target_id,
            quality=incoming.quality,
            detected_at=incoming.detected_at,
            bearing_deg=incoming.bearing_deg,
            range_m=incoming.range_m,
            x=incoming.x,
            y=incoming.y,
            z=incoming.z,
            # ★ 原始量测者逐跳原样保留：洗成"转发方"就没有防回环了，
            #   而且"这情报是谁测的"这个最要紧的元数据会丢。
            origin_id=incoming.origin_id,
            hops=incoming.hops + 1,
            # ★ 「这条航迹是假目标」也要跟着走上路：假目标的任务就是污染
            #   对方的情报链，拦在本平台等于把干扰的效果限制在单平台
            #   （§9-53）。这里漏一个字段的症状极隐蔽——**转发之后它就被
            #   洗成真目标**，而位置、时间戳、来源标记全都正常。
            phantom=incoming.phantom,
            # ★ 敌我状态逐跳原样保留（同 ``phantom`` 的理由）：洗掉它
            #   的症状是"友邻把你的敌人当成了自己人"。
            iff=incoming.iff,
            # 精度与速度跟着报文走（v0.13.56）：有 σ 才有得融合，
            # 没有的报文在这道闸门外就退化成覆盖更新了。
            sigma_pos_m=incoming.sigma_pos_m,
            sigma_vel_mps=incoming.sigma_vel_mps,
            vx=incoming.vx,
            vy=incoming.vy,
            vz=incoming.vz,
        )
        # ★ hops 在 merge 里已经 +1：融合时它跟主贡献方走（取的是
        #   existing.hops 与 incoming.hops+1 里的主贡献方），覆盖更新时
        #   直接就是 +1 后的值——两路都不会把转发链的长度记丢。
        return contact, ("fused" if how == "fused" else "accepted")

    def _find_contact(self, owner_id: int, target_id: int) -> Contact | None:
        for contact in self._contacts.get(owner_id, ()):
            if contact.target_id == target_id:
                return contact
        return None

    def contacts_of(self, owner_id: int) -> list[Contact]:
        """返回**副本列表**——调用方可以随便改，不会动到内部状态。

        返回内部列表的话，模型清空一下就等于改了别人的数据域。
        """
        return list(self._contacts.get(owner_id, ()))

    def drop_contact(self, owner_id: int, target_id: int) -> bool:
        contacts = self._contacts.get(owner_id)
        if not contacts:
            return False
        for i, contact in enumerate(contacts):
            if contact.target_id == target_id:
                del contacts[i]
                return True
        return False

    def contact_count(self, owner_id: int) -> int:
        """自己名下的航迹条数。"""
        return len(self._contacts.get(owner_id, ()))

    # -- 消息：在途队列与收件箱 --------------------------------------------

    def post_message(
        self,
        sender_id: int,
        recipient_id: int,
        kind: str,
        sent_at: int = 0,
        deliver_at: int = 0,
        payload: dict[str, Any] | None = None,
    ) -> Message:
        """**直接**投进收件箱，跳过通信建模。

        适用场景：同一载具内部部件之间的交互，军事上确实是瞬时无损的。
        需要建模延迟与丢包时用 :meth:`enqueue_message`。
        """
        message = self._make_message(
            sender_id, recipient_id, kind, sent_at, deliver_at, payload
        )
        self._inbox.setdefault(recipient_id, []).append(message)
        return message

    def enqueue_message(
        self,
        sender_id: int,
        recipient_id: int,
        kind: str,
        sent_at: int = 0,
        deliver_at: int = 0,
        payload: dict[str, Any] | None = None,
    ) -> Message:
        """投进**在途**队列，等通信模型搬运。

        ``deliver_at`` 是发送方给出的**预期**送达时刻；通信模型可以改写它
        （延迟、绕路），也可以直接丢弃（丢包、链路中断）。
        """
        message = self._make_message(
            sender_id, recipient_id, kind, sent_at, deliver_at, payload
        )
        self._inflight.append(message)
        return message

    def _make_message(
        self,
        sender_id: int,
        recipient_id: int,
        kind: str,
        sent_at: int,
        deliver_at: int,
        payload: dict[str, Any] | None,
    ) -> Message:
        self._msg_seq += 1
        return Message(
            msg_id=self._msg_seq,
            sender_id=sender_id,
            recipient_id=recipient_id,
            kind=kind,
            sent_at=sent_at,
            deliver_at=deliver_at if deliver_at else sent_at,
            payload=dict(payload) if payload else {},
        )

    def inflight_messages(self) -> list[Message]:
        """在途消息，**发送顺序**。通信模型按需筛选。"""
        return list(self._inflight)

    def deliver_due(self, now: int) -> list[Message]:
        """把 ``deliver_at <= now`` 的在途消息搬进收件箱，返回搬了哪些。

        由通信模型调用。**保持原顺序**——同一发送方连发的两条消息不能因为
        排序而颠倒，那会让"先侦察后开火"变成"先开火后侦察"。
        """
        if not self._inflight:
            return []

        delivered: list[Message] = []
        remaining: list[Message] = []
        for message in self._inflight:
            if message.deliver_at <= now:
                self._inbox.setdefault(message.recipient_id, []).append(message)
                delivered.append(message)
            else:
                remaining.append(message)
        self._inflight = remaining
        return delivered

    def take_messages(self, entity_id: int) -> list[Message]:
        """取走收件箱里的全部消息并清空。FIFO 顺序。

        "取走"而不是"读"——消息处理一次就该消失，留着会让实体每帧重复
        执行同一条命令。
        """
        return self._inbox.pop(entity_id, [])

    # -- 通信模型专用：按拓扑择时投递 --------------------------------------

    def due_messages(self, now: int) -> list[Message]:
        """**到期但尚未投递**的在途消息，发送顺序。

        与 :meth:`deliver_due` 的区别：那个**顺手就投了**，这个只**挑出来**
        给调用方看，投不投由调用方决定。通信模型要的正是后者——"到点"不等于
        "通"，一条报文到点了、但收件方此刻被压制（或跨网、或掉了线），就该
        留着等下一拍（§5.15）。

        ★ 不在这里判"通不通"：那是拓扑问题，只有
        :class:`~milsim.services.comm.CommService` 答得了。存储层只回答
        "哪些到点了"。
        """
        if not self._inflight:
            return []
        return [m for m in self._inflight if m.deliver_at <= now]

    def forget_due(self, messages: Sequence[Message]) -> int:
        """把一批在途消息**移出**在途队列（投递成功或放弃等下一拍之前先取走）。

        按**对象身份**移除，不是按字段比较——两条内容完全一样的报文（同一
        发送方连发两条同样的命令）是两条消息，按值删会把它们一起删掉，
        而症状是"同一条命令发两次，只到了一半"。

        返回实际移除的条数（诊断 / 断言用）。
        """
        if not messages or not self._inflight:
            return 0
        drop = {id(m) for m in messages}
        before = len(self._inflight)
        self._inflight = [m for m in self._inflight if id(m) not in drop]
        return before - len(self._inflight)

    def reinsert_message(self, message: Message) -> None:
        """把一条消息**放回**在途队列尾（等下一拍再看它通不通）。

        ★ 调一次 :meth:`forget_due` 之后再 :meth:`reinsert_message` 是"这一拍
        投不了、下一拍再试"的完整写法。放在这里而不是让通信模型直接碰
        ``_inflight``——那个私有队列只有本类能动，"谁能改在途"才是一句能答的
        话。
        """
        self._inflight.append(message)

    def peek_messages(self, entity_id: int) -> list[Message]:
        """看一眼但不取走。调试与测试用。"""
        return list(self._inbox.get(entity_id, ()))

    def take_messages_of_kind(self, entity_id: int, kind: str) -> list[Message]:
        """取走收件箱里**指定种类**的消息，别种的原样留下、顺序不变。

        ★ 为什么不能一律用 :meth:`take_messages`：一个实体上可以同时挂着航迹
        管理器与指挥控制器，两者**共用一个收件箱**。``take_messages`` 是
        "取走即清空"，先跑的那个把第二个的信也吃了——症状是"指令偶尔不生效"，
        而发作频率取决于同刻事件的先后，基本复现不出来。

        所以：**收件箱的消费者一律按种类取**，只有"这个实体上只有一个消费者"
        这种明确情形才用全取。这条对本项目是硬约束，不是风格偏好。
        """
        inbox = self._inbox.get(entity_id)
        if not inbox:
            return []
        mine = [m for m in inbox if m.kind == kind]
        if not mine:
            return []
        self._inbox[entity_id] = [m for m in inbox if m.kind != kind]
        return mine

    def next_delivery(self) -> int | None:
        """最近一条在途消息的投递时刻。通信模型据此注册自己的周期事件。"""
        best: int | None = None
        for message in self._inflight:
            if best is None or message.deliver_at < best:
                best = message.deliver_at
        return best

    # -- 物资（自己的域） --------------------------------------------------

    def set_loadout(self, entity_id: int, name: str, value: float) -> None:
        """写自己的弹药 / 油量。"""
        self._require(entity_id)
        self._loadout.setdefault(entity_id, {})[name] = float(value)

    def loadout_of(self, entity_id: int) -> dict[str, float]:
        """读**任意**实体的物资。读是公开的——知道对手还剩几发弹是合理的
        战场认知（虽然实际要靠侦察），写才是受限的。"""
        return dict(self._loadout.get(entity_id, {}))

    # -- 视图工厂 ----------------------------------------------------------

    def sensor_view(self, entity_id: int) -> "SensorView":
        """感知模型能看到的全部。"""
        self._require(entity_id)
        return SensorView(self, entity_id)

    def mover_view(
        self, entity_id: int, locator: Callable[[float, float, float], CellRef] | None = None
    ) -> "MoverView":
        """机动模型能看到的全部。

        ``locator`` 由装配层注入（把局部坐标转成格归属），这样机动模型
        不必知道地图服务的存在，也不必自己算格子。
        """
        self._require(entity_id)
        return MoverView(self, entity_id, locator)

    def engagement_view(self, entity_id: int) -> "EngagementView":
        """交战模型能看到的全部。"""
        self._require(entity_id)
        return EngagementView(self, entity_id)

    def comm_view(self, entity_id: int) -> "CommView":
        """通信模型能看到的全部。"""
        self._require(entity_id)
        return CommView(self, entity_id)

    def track_view(self, entity_id: int) -> "TrackView":
        """平台级航迹管理器能看到的全部。"""
        self._require(entity_id)
        return TrackView(self, entity_id)

    # -- 诊断 --------------------------------------------------------------

    def statistics(self) -> dict[str, int]:
        return {
            "entities": len(self._active),
            "alive": len(self.alive_ids()),
            "layers": len(self._local),
            "global_cells": len(self._global),
            "contacts": sum(len(c) for c in self._contacts.values()),
            "contacts_local": sum(
                1
                for bucket in self._contacts.values()
                for contact in bucket
                if contact.is_local
            ),
            "relayed": self._relayed,
            "relay_rejected_circular": self._relay_rejected["circular"],
            "relay_rejected_hops": self._relay_rejected["hops"],
            "relay_rejected_stale": self._relay_rejected["stale"],
            "queued_messages": sum(len(q) for q in self._inbox.values()),
            "inflight_messages": len(self._inflight),
            "pending_effects": len(self._effects),
            "capacity": self._kinematics.capacity,
        }

    def validate(self) -> list[str]:
        """自检：索引与数据域之间必须完全一致。测试和调试用。"""
        problems: list[str] = []

        # 1. 每个活跃实体都必须有格归属
        for entity_id in sorted(self._active):
            if entity_id not in self._cell:
                problems.append(f"活跃实体无格归属：id={entity_id}")

        # 2. 格归属与索引必须双向一致（幽灵实体守在这里）
        for entity_id, cell in self._cell.items():
            if entity_id not in self._active:
                problems.append(f"格归属残留（幽灵实体）：id={entity_id} cell={cell}")
                continue
            if isinstance(cell, LocalCellRef):
                index = self._local.layer(cell.zone_id, cell.layer)
                if entity_id not in index.entities_at(cell.axial):
                    problems.append(f"局部索引缺实体：id={entity_id} {cell}")
            else:
                if entity_id not in self._global.get(cell.h3_cell, ()):
                    problems.append(f"全球索引缺实体：id={entity_id} {cell}")

        # 3. 空间索引里不能有已注销的实体
        for zone_id, layer in self._local.keys():
            index = self._local.layer(zone_id, layer)
            for entity_id in index:
                if entity_id not in self._active:
                    problems.append(
                        f"局部索引有幽灵 ID：id={entity_id} zone={zone_id} L{layer}"
                    )
        for h3_cell, bucket in self._global.items():
            for entity_id in bucket:
                if entity_id not in self._active:
                    problems.append(f"全球索引有幽灵 ID：id={entity_id} cell={h3_cell:x}")
            if not bucket:
                problems.append(f"全球索引有空桶：{h3_cell:x}")

        # 4. 数据域不能有已注销实体的残留
        for domain_name, domain in (
            ("航迹", self._contacts),
            ("收件箱", self._inbox),
            ("物资", self._loadout),
        ):
            for entity_id in domain:
                if entity_id not in self._active:
                    problems.append(f"{domain_name}域有注销实体残留：id={entity_id}")

        # 5. 完好度必须在值域内
        for entity_id in sorted(self._active):
            value = float(self._damage[entity_id])
            if not (DAMAGE_DESTROYED <= value <= DAMAGE_INTACT):
                problems.append(f"完好度越界：id={entity_id} value={value}")

        # 6. 消息与效果的收发方必须存在
        for message in self._inflight:
            if message.recipient_id not in self._active:
                problems.append(f"在途消息收方已注销：msg={message.msg_id}")
        for effect in self._effects:
            if effect.source_id not in self._active:
                problems.append(f"效果来源已注销：id={effect.source_id}")
            if effect.target_id not in self._active:
                problems.append(f"效果目标已注销：id={effect.target_id}")

        return problems

    # -- 协议 --------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._active)

    def __contains__(self, entity_id: int) -> bool:
        return entity_id in self._active

    def __iter__(self) -> Iterator[int]:
        """遍历**活跃**实体 ID，升序。"""
        return iter(sorted(self._active))

    def __repr__(self) -> str:
        return (
            f"<EntityStore {len(self._active)} 实体"
            f"（存活 {len(self.alive_ids())}）/ 容量 {self._kinematics.capacity}>"
        )


# ---------------------------------------------------------------------------
# 视图：模型能看到的全部
#
# 这些类刻意不做成"接口 + 实现"的继承体系，而是四个互不相干的 frozen
# dataclass。它们之间没有共同基类——有共同基类就会有人往基类上加方法，
# 于是四个视图又一起变大，收窄的意义就没了。
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class SensorView:
    """感知模型的能力边界。

    能做的：看自己的位置、查某方向有哪些实体、写**自己**的航迹。
    不能做的：改任何人的位置、改任何人的损耗、看别人的航迹。
    """

    _store: EntityStore
    _entity_id: int

    @property
    def entity_id(self) -> int:
        return self._entity_id

    def my_position(self) -> tuple[float, float, float] | None:
        return self._store.position_of(self._entity_id)

    def my_cell(self) -> CellRef | None:
        return self._store.cell_of(self._entity_id)

    def position_of(self, target_id: int) -> tuple[float, float, float] | None:
        """目标的**真实**位置。

        感知模型可以用它配合探测概率决定"能不能发现"，但**不该**直接把它
        当成航迹写下去——那等于上帝视角。真实位置与航迹之间的差异（误差、
        延迟、误判）正是任务级仿真要产出的东西。
        """
        return self._store.position_of(target_id)

    def bearing_to(self, target_id: int) -> float | None:
        """自己到目标的**真实**方位角（度，正北 0°，顺时针）。

        与 :meth:`position_of` 一样是上帝视角的原始量，不是航迹。写进航迹时
        要不要加误差、加多少，是感知模型的事。

        方位角是**世界坐标**上的量：服务层替模型把"y 轴指向南"这条约定吃掉，
        模型层就不必自己写 ``atan2`` 再猜一次符号。目标不存在、或就在脚下
        （偏移为零）时返回 ``None``——后者没有方位可言。
        """
        me = self._store.position_of(self._entity_id)
        target = self._store.position_of(target_id)
        if me is None or target is None:
            return None
        east = target[0] - me[0]
        south = target[1] - me[1]
        if east == 0.0 and south == 0.0:
            return None
        return bearing_of_offset(east, south)

    def is_alive(self, target_id: int) -> bool:
        return self._store.is_alive(target_id)

    def query_cone(
        self, direction: int, max_steps: int, half_width: int = 0
    ) -> list[int]:
        """向某方向搜索候选实体，由近及远。

        粗筛——只保证"在锥形覆盖的格子里"，不含通视与探测概率判定。
        精筛由模型自己配合地图服务做。
        """
        cell = self._store.cell_of(self._entity_id)
        if not isinstance(cell, LocalCellRef):
            return []
        return self._store.entities_in_cone(cell, direction, max_steps, half_width)

    def query_sector(
        self, bearing_start: float, bearing_end: float, max_steps: int
    ) -> list[int]:
        """**按天线方向**搜索候选实体：方位角落在给定扇区内，由近及远。

        与 :meth:`query_cone` 的区别：锥形吸附到六个网格方向、宽度固定；
        这里用的是真实方位角，张角随距离自然展开——旋转天线"这一帧从
        ``bearing_start`` 扫到 ``bearing_end``"就该用这个。

        方位角跨 0° 时自动环绕。**两个边界一样时不代表空扇区**，按整圈
        解释（方位角取模的习惯），调用方要判"这一帧没转"请自己先判。

        粗筛——只保证格心方位角在扇区内，不含通视与探测概率判定。
        """
        cell = self._store.cell_of(self._entity_id)
        if not isinstance(cell, LocalCellRef):
            return []
        return self._store.entities_in_sector(
            cell, bearing_start, bearing_end, max_steps
        )

    def query_disk(self, radius: int) -> list[int]:
        """全向搜索候选实体，由近及远。"""
        cell = self._store.cell_of(self._entity_id)
        if not isinstance(cell, LocalCellRef):
            return []
        return self._store.entities_in_disk(cell, radius)

    def add_contact(
        self,
        target_id: int,
        quality: float = 1.0,
        detected_at: int = 0,
        bearing_deg: float = 0.0,
        range_m: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        phantom: bool = False,
        iff: str = IFF_UNKNOWN,
    ) -> Contact:
        """写自己的航迹（``phantom=True`` = 假目标造的幽灵）。"""
        return self._store.add_contact(
            self._entity_id,
            target_id,
            quality=quality,
            detected_at=detected_at,
            bearing_deg=bearing_deg,
            range_m=range_m,
            x=x,
            y=y,
            z=z,
            phantom=phantom,
            iff=iff,
        )

    def my_contacts(self) -> list[Contact]:
        return self._store.contacts_of(self._entity_id)

    def merge_contact(
        self,
        target_id: int,
        *,
        quality: float,
        detected_at: int,
        bearing_deg: float,
        range_m: float,
        x: float,
        y: float,
        z: float,
        phantom: bool = False,
        iff: str = IFF_UNKNOWN,
        sigma_pos_m: float = 0.0,
        sigma_vel_mps: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
    ) -> tuple[Contact, str]:
        """按精度融合地写自己的航迹（``"fused"`` / ``"replaced"``）。

        本机量测的融合入口：同一平台的多台传感器、同一传感器的相邻两拍，
        只要双方都带 σ，就按精度加权融合，而不是谁后写谁赢。
        """
        return self._store.merge_contact(
            self._entity_id,
            target_id,
            quality=quality,
            detected_at=detected_at,
            bearing_deg=bearing_deg,
            range_m=range_m,
            x=x,
            y=y,
            z=z,
            origin_id=ORIGIN_LOCAL,
            hops=0,
            phantom=phantom,
            iff=iff,
            sigma_pos_m=sigma_pos_m,
            sigma_vel_mps=sigma_vel_mps,
            vx=vx,
            vy=vy,
            vz=vz,
        )

    def drop_contact(self, target_id: int) -> bool:
        return self._store.drop_contact(self._entity_id, target_id)


@dataclass(frozen=True, slots=True)
class MoverView:
    """机动模型的能力边界。

    只写得动自己的位置。``locator`` 由装配层注入，把局部坐标转成格归属，
    所以机动模型不必知道地图服务的存在。
    """

    _store: EntityStore
    _entity_id: int
    _locator: Callable[[float, float, float], CellRef] | None = None

    @property
    def entity_id(self) -> int:
        return self._entity_id

    def my_position(self) -> tuple[float, float, float] | None:
        return self._store.position_of(self._entity_id)

    def my_pose(
        self,
    ) -> tuple[float, float, float, float, float] | None:
        """(x, y, z, heading_deg, speed_mps)。"""
        return self._store.pose_of(self._entity_id)

    def my_cell(self) -> CellRef | None:
        return self._store.cell_of(self._entity_id)

    def my_alive(self) -> bool:
        """自己的存活状态（完好度 > 0）。

        机动件每帧用它早退：**实体被毁后残骸停在最后一帧的位置**，不再
        推进。引擎不知道"死亡"、事件照常到期，不挡的话被打死的车还会
        继续跑（位置照旧更新，只有 ``is_alive`` 变了）。
        """
        return self._store.is_alive(self._entity_id)

    def set_pose(
        self,
        x: float,
        y: float,
        z: float,
        heading_deg: float = 0.0,
        speed_mps: float = 0.0,
    ) -> None:
        """写自己的位置与姿态，并按 ``locator`` 同步格归属。

        没有配 ``locator`` 时不更新格归属——此时位置与索引会不一致，
        只适合"不动"的实体或单元测试。装配层必须注入 ``locator``。
        """
        self._store.set_pose(
            self._entity_id, x, y, z, heading_deg, speed_mps
        )
        if self._locator is not None:
            self._store.set_cell(self._entity_id, self._locator(x, y, z))

    def set_cell(self, cell: CellRef) -> bool:
        """显式更新格归属。返回是否跨格。没有 ``locator`` 时用它。"""
        return self._store.set_cell(self._entity_id, cell)

    def advance(self, dt_s: float) -> None:
        """按当前航向与速度前进 ``dt_s`` 秒，并同步格归属。

        **没有 ``locator`` 时直接报错**，不静默跳过——位置变了而空间索引
        没变，实体在新位置上就"消失"了，雷达再也搜不到它。这种 bug 表现
        为"单位刚启动就不见了"，极难往索引失配上想。
        """
        if self._locator is None:
            raise RuntimeError(
                f"MoverView({self._entity_id}) 未配置 locator，无法推进位置——"
                "movement 必须同步空间索引，否则实体会从索引里消失"
            )
        kinematics = self._store._kinematics
        kinematics.advance(self._entity_id, dt_s)
        self._store.set_cell(self._entity_id, self._locator(*kinematics.position(self._entity_id)))


@dataclass(frozen=True, slots=True)
class EngagementView:
    """交战模型的能力边界。**没有** ``set_damage``——那是裁决器的专属域。"""

    _store: EntityStore
    _entity_id: int

    @property
    def entity_id(self) -> int:
        return self._entity_id

    def my_position(self) -> tuple[float, float, float] | None:
        return self._store.position_of(self._entity_id)

    def position_of(self, target_id: int) -> tuple[float, float, float] | None:
        return self._store.position_of(target_id)

    def damage_of(self, target_id: int) -> float:
        return self._store.damage_of(target_id)

    def is_alive(self, target_id: int) -> bool:
        return self._store.is_alive(target_id)

    def my_loadout(self) -> dict[str, float]:
        return self._store.loadout_of(self._entity_id)

    def loadout_of(self, target_id: int) -> dict[str, float]:
        return self._store.loadout_of(target_id)

    def set_my_loadout(self, name: str, value: float) -> None:
        self._store.set_loadout(self._entity_id, name, value)

    def request_effect(
        self, target_id: int, kind: str, magnitude: float, at: int = 0, note: str = ""
    ) -> Effect:
        """提交毁伤**意图**，由裁决器结算。

        这里写的是"我打中了、预期毁伤 0.3"，不是"扣它 0.3 血"。差别在于
        目标的抗性由目标自己算——打击方不需要知道对方是坦克还是飞机，
        这正是"交互密集但依赖稀疏"的落点。
        """
        effect = Effect(
            source_id=self._entity_id,
            target_id=target_id,
            kind=kind,
            magnitude=magnitude,
            at=at,
            note=note,
        )
        self._store.request_effect(effect)
        return effect


@dataclass(frozen=True, slots=True)
class CommView:
    """通信模型的能力边界。

    发消息是唯一允许写别人域的通道——"把信放进邮箱"不影响收信人的状态，
    什么时候读、怎么反应是他自己的事。
    """

    _store: EntityStore
    _entity_id: int

    @property
    def entity_id(self) -> int:
        return self._entity_id

    def my_position(self) -> tuple[float, float, float] | None:
        return self._store.position_of(self._entity_id)

    def send(
        self,
        recipient_id: int,
        kind: str,
        sent_at: int = 0,
        deliver_at: int = 0,
        payload: dict[str, Any] | None = None,
    ) -> Message:
        """投进**在途**队列，由通信模型决定何时送达、是否丢弃。

        不直接塞进收件箱是刻意的：直接塞等于宣告通信瞬时且无损，
        指挥延迟、链路拥塞、丢包就全都没地方发生。
        """
        return self._store.enqueue_message(
            self._entity_id,
            recipient_id,
            kind,
            sent_at=sent_at,
            deliver_at=deliver_at,
            payload=payload,
        )

    def take_messages(self) -> list[Message]:
        """取走发给自己的全部消息并清空。"""
        return self._store.take_messages(self._entity_id)

    def take_messages_of_kind(self, kind: str) -> list[Message]:
        """只取走某一类报文，别种原样留下。

        见 :meth:`EntityStore.take_messages_of_kind`：同一实体上多个部件共用
        一个收件箱时，全取会把别人的信一起吃掉。
        """
        return self._store.take_messages_of_kind(self._entity_id, kind)

    def peek_messages(self) -> list[Message]:
        return self._store.peek_messages(self._entity_id)

    def loadout_of(self, target_id: int) -> dict[str, float]:
        return self._store.loadout_of(target_id)


@dataclass(frozen=True, slots=True)
class TrackView:
    """平台级航迹管理器（``models/percep/track_manager.py``）的能力边界。

    它比 :class:`SensorView` **多**两样、**少**一样：

    - **多** ``take_messages`` —— 外部报告从收件箱进来。这是"多渠道汇聚"
      的**唯一**入口，所以它必须能取信；
    - **多** ``share_tracks`` —— 把自己的航迹发给相关方（AFSIM 的 track
      processor "向相关方发送更新后的航迹"）。发往在途队列而不是直塞收件箱，
      所以链路延迟与丢包仍然有地方发生；
    - **少** 一切真实位置 —— 航迹管理器**不判"能不能发现"**，那件事只能由
      传感器做。给它 ``position_of``，"拿真值刷航迹"就变成一个顺手能写下的
      操作，而那正是本项目最想堵死的上帝视角泄漏。

    ``add_contact`` 也**不在**这里：本机量测由传感器写（它才知道量测），
    外部报告由 :meth:`relay_contact` 写（它带防回环）。两条来路各有各的门，
    合起来就没有一道是"什么都收"的。
    """

    _store: EntityStore
    _entity_id: int

    @property
    def entity_id(self) -> int:
        return self._entity_id

    def my_contacts(self) -> list[Contact]:
        """本平台当前的全部航迹（副本）。"""
        return self._store.contacts_of(self._entity_id)

    def drop_contact(self, target_id: int) -> bool:
        """撤掉一条航迹。过时清理与"目标已确认不存在"都走这里。"""
        return self._store.drop_contact(self._entity_id, target_id)

    def relay_contact(
        self,
        payload: dict[str, Any],
        *,
        sender_id: int,
        received_at: int,
        max_hops: int = MAX_RELAY_HOPS,
    ) -> tuple[Contact | None, str]:
        """同化一条通信收到的航迹报告。见 :meth:`EntityStore.relay_contact`。"""
        return self._store.relay_contact(
            self._entity_id,
            payload,
            sender_id=sender_id,
            received_at=received_at,
            max_hops=max_hops,
        )

    def share_tracks(
        self,
        recipient_id: int,
        *,
        sent_at: int = 0,
        deliver_at: int = 0,
    ) -> list[Message]:
        """把本平台航迹**逐条**打包发给 ``recipient_id``，返回发出的报文。

        逐条而不是打包成一个列表：真实的航迹报告就是一条一条发的，
        而合在一起之后"只丢了一条"这种情形就表达不出来——要么全到、
        要么全不到。``payload`` 的形状由 :meth:`Contact.as_dict` 一家定义。
        """
        return [
            self._store.enqueue_message(
                self._entity_id,
                recipient_id,
                TRACK_REPORT,
                sent_at=sent_at,
                deliver_at=deliver_at,
                payload=contact.as_dict(sender_id=self._entity_id),
            )
            for contact in self._store.contacts_of(self._entity_id)
        ]

    def take_messages(self) -> list[Message]:
        """取走发给自己的全部消息并清空。"""
        return self._store.take_messages(self._entity_id)

    def take_messages_of_kind(self, kind: str) -> list[Message]:
        """只取走某一类报文，别种原样留下。

        ★ 航迹管理器**必须**用这个：平台上还挂着指挥控制器的时候，两者共用一个
        收件箱，全取会把"前进"这类指令一起吃掉。
        """
        return self._store.take_messages_of_kind(self._entity_id, kind)

    def peek_messages(self) -> list[Message]:
        return self._store.peek_messages(self._entity_id)


__all__ = [
    "CellRef",
    "COMMAND_ORDER",
    "Contact",
    "DAMAGE_DESTROYED",
    "DAMAGE_INTACT",
    "DEFAULT_CAPACITY",
    "Effect",
    "EngagementView",
    "EntityStore",
    "GlobalCellRef",
    "IFF_AMBIGUOUS",
    "IFF_FOE",
    "IFF_FRIEND",
    "IFF_KINDS",
    "IFF_NEUTRAL",
    "IFF_UNKNOWN",
    "KinematicsTable",
    "LocalCellRef",
    "MAX_RELAY_HOPS",
    "Message",
    "MoverView",
    "ORIGIN_LOCAL",
    "SensorView",
    "CommView",
    "TRACK_REPORT",
    "TrackView",
]
