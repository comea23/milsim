"""平台级航迹管理器：把多渠道的航迹报告汇成**一份**本平台的航迹表。

对应 AFSIM 的两个东西
---------------------

* ``track_manager`` 是 **``platform`` 的子命令**，维护平台的 **master track
  list** —— 也就是说"航迹挂在平台上"这件事 AFSIM 就是这么定的，不是我们
  发明的；
* ``WSF_TRACK_PROCESSOR`` —— "**接收来自本地和外部来源的报告**，交给 track
  manager 做**关联 + 融合**，并**向相关方发送**更新后的航迹"。

本组件就是那句话的后半段。本机传感器仍然直接写航迹（它才知道量测），
本组件负责三件传感器管不了的事：

1. **汇聚** —— 把通信收到的航迹报告同化进本平台的航迹表（§5.13.2）；
2. **清理** —— 超过 ``drop_after_inactive`` 没有任何更新的航迹撤掉（§5.13.3）；
3. **转发**（可选，默认关）—— 把本平台航迹发给想定点名的相关方（§5.13.4）。

.. note::

    **我们比 AFSIM 少一整层：关联。**

    AFSIM 之所以需要 ``correlation_method``（``perfect`` / ``nearest_neighbor``
    / ``truth`` / ``mtt``）外加一整套 gating，是因为它的 track processor 手里
    **只有量测**（位置、方位、距离），一条报告对应哪个目标要靠几何去猜。

    我们**有真值实体 ID**：传感器量的是实体，报文里带的也是实体 ID，所以
    "两条报告是不是同一个目标"是**比较两个整数**。这层不是被简化了，是
    **在我们这套接口下不存在**。★ 代价要写在明处：我们因此也**做不到**
    "把同一批实体 ID 的两条航迹判成不同目标"——真正的关联错误（把一架
    飞机当成两架、或把两架当成一架）出不来。要那种能力得走"量测级接口"，
    那是另一条路（§9-45）。

    **融合落在 ``replacement`` + 精度加权两级。** AFSIM 的 ``fusion_method``
    有 ``replacement`` 与 ``weighted_average``。``replacement``（新者胜）是
    基础档：报文没带精度（σ=0）时就是它，与 v0.13.55 逐字相同。
    ``weighted_average``（v0.13.56 落地）：报文带 σ 的同目标报告**时间对齐**
    后按 ``1/σ²`` 加权平均（§5.13.5）——两个来源都看一眼，比只看一个准。
    当年不落它的理由（``Contact`` 没有 σ、``quality`` 是 Pd 不是精度）已经
    解决：σ 由传感器第 7b 步算出后**随量测上路**（``sigma_pos_m``），
    融合的权重用的是 σ 而不是 Pd。**关联层依旧不存在**（真值 ID 接口下
    退化为按 ``target_id`` 归并），真关联错误仍出不来（§9-45）。

.. warning::

    **转发把报文放进"在途队列"，搬进收件箱的是通信件的活。**

    ``TrackView.share_tracks`` 写的是 ``EntityStore._inflight``，不直接塞
    收件箱——直接塞等于宣告通信瞬时无损（见 ``store.py`` 模块头）。
    把在途报文搬进收件箱的 ``deliver_due`` 由**通信件**（``RF_COMM``）调用；
    **v0.13.37 起通信件已落地**（§5.15，`CommNode._tick` 里的
    ``deliver(now)``），所以"发了就有人搬"这条链路是通的。
    只打开 ``share_interval`` 而**没建网**时，``comm.path_exists`` 走
    "没建网不限制"那条口子 ⇒ 报文照投（旧想定逐字不变）。
"""

from __future__ import annotations

from ...engine import PRIORITY_SENSOR, EventResult
from ...errors import ConfigurationError
from ...services.params import Params
from ...services.store import MAX_RELAY_HOPS, TRACK_REPORT
from ...services.type_registry import register_component
from ..component import Component

#: 默认汇聚节拍（微秒）。3 s：与引擎的 2 s 栅栏、机动件的 5 s 都错开，
#: 免得三件事挤在同一刻——同刻事件按优先级排队是对的，但挤在一起会让
#: "某一帧特别慢"变得无法归因。
TRACK_PERIOD_US = 3_000_000

#: ``relay_contact`` 的五种处理结论 → 计数器字段名。
#: **一张表，两处不再各写一遍**：加一种结论时漏改一处会让统计悄悄少一项。
_VERDICT_COUNTERS = {
    "accepted": "absorbed",
    "circular": "rejected_circular",
    "hops": "rejected_hops",
    "stale": "rejected_stale",
    "fused": "fused",
}


@register_component("TRACK_MANAGER")
class TrackManager(Component):
    """一个平台上的航迹汇聚点。**不装它，行为与以前逐字相同。**

    "不装它"是刻意的默认：航迹仍由传感器直接写平台域，没有汇聚、没有清理、
    没有转发。这样 v0.13.28 及以前的所有想定与对照数字一个不动——新行为的
    唯一正当判据是"旧行为可逐字复现"。

    ``drop_after_inactive`` 默认 **0 = 不清理**，与 AFSIM 一致（它的
    ``drop_after_inactive`` 不写就是无限）。这一条值得单说：

    * 用户担心的"一直探测就要一直保留、存储压力太大"**在状态口径下不成立**
      —— 同一个目标重复探测是覆盖更新，航迹表大小 = 看到的目标数，与推演
      跑了多久无关（AFSIM 的 ``track`` 块也全是当前值，历史要
      ``retain_track_history`` 显式打开，默认不保留）；
    * 真正的压力来自**不做清理**：一个目标被摧毁、或飞出了所有雷达的探测
      范围之后，那条航迹会永远挂在表上。``drop_after_inactive`` 治的是这个，
      与 M/N 撤航（传感器侧，"还跟不跟得住"）是**两件事**：M/N 管量测的
      连续性，这里管"多久没有任何更新"。

    ``share_interval`` / ``share_recipients`` 是**半给不行**的一对：开了转发
    却一个人都不发、或者写了收件人却没开转发，都在装配期报错。静默的后果
    是"我明明配了共享，对端什么都没收到"，而参数表看着完全正常。
    """

    PARAMS = {
        #: 汇聚 / 清理节拍。AFSIM 把这两件事分成 ``update_interval`` 与
        #: ``purge_interval`` 两条，这里合成一条——两个节拍只会让"什么时候
        #: 撤的航"这个可测的量变得依赖两者的最小公倍数。
        "period": Params.duration(TRACK_PERIOD_US, minimum=1_000),
        #: 超过这么久**没有任何更新**就撤航（微秒）。``0`` = 不清理（默认）。
        #: ★ 判据是**严格大于**：相等不算超时。
        "drop_after_inactive": Params.duration(0, minimum=0),
        #: 转发报告的跳数上限，见 :data:`~milsim.services.store.MAX_RELAY_HOPS`。
        "max_relay_hops": Params.integer(MAX_RELAY_HOPS, minimum=1),
        #: 多长时间把本平台航迹转发一轮（微秒）。``0`` = 不转发（默认）。
        "share_interval": Params.duration(0, minimum=0),
        #: 转发给谁 —— **同方实体的名字列表**，想定里写
        #: ``share_recipients [RED_SCOUT_1]``。用名字而不是编号：想定作者写的
        #: 是"共享给谁"，不是"共享给实体 7"。
        "share_recipients": Params.strings(default=()),
    }

    SLOT_HINT = "track"

    __slots__ = (
        "view",
        "_recipients",
        "_period_us",
        "_inactive_us",
        "_max_hops",
        "_share_us",
        "_next_share_us",
        "ticks",
        "absorbed",
        "fused",
        "rejected_circular",
        "rejected_hops",
        "rejected_stale",
        "expired",
        "shared",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount) -> None:
        self.view = mount.track_view()
        self._period_us = int(self.spec["period"])
        self._inactive_us = int(self.spec["drop_after_inactive"])
        self._max_hops = int(self.spec["max_relay_hops"])
        self._share_us = int(self.spec["share_interval"])
        self._recipients = self._resolve_recipients(mount)

        #: 第一次 tick 就发一轮。装配时刻本平台还没有航迹，那一轮发出 0 条
        #: —— 把初值设成 0 而不是 ``share_interval``，是为了让"节拍从 t=0
        #: 起算"这件事与其它周期事件一致，不必额外记一条相位规则。
        self._next_share_us = 0
        self.ticks = 0
        self.absorbed = 0
        self.fused = 0
        self.rejected_circular = 0
        self.rejected_hops = 0
        self.rejected_stale = 0
        self.expired = 0
        self.shared = 0

        mount.every(self._period_us, self._tick, PRIORITY_SENSOR)

    def _resolve_recipients(self, mount) -> tuple[int, ...]:
        """把 ``share_recipients`` 的名字解析成实体 ID。**装配期就解析。**

        运行期再解析的代价是"收件人还不存在时静默少发一个"——而航迹共享
        本来就没有确认机制，少发一个谁都不会报错。
        """
        names = tuple(self.spec["share_recipients"])
        name = self.type_name or type(self).__name__

        if self._share_us > 0 and not names:
            raise ConfigurationError(
                f"{name}: share_interval 开了却没写 share_recipients —— "
                "一条也发不出去，而参数表看着正常（半给不行）"
            )
        if self._share_us <= 0 and names:
            raise ConfigurationError(
                f"{name}: 写了 share_recipients 却没开 share_interval —— "
                "收件人永远不会收到任何东西（半给不行）"
            )

        out: list[int] = []
        for item in names:
            entity = mount.registry.by_name(item)
            if entity is None:
                raise ConfigurationError(
                    f"{name}: share_recipients 里的 {item!r} 不在名册上"
                )
            if entity.entity_id == mount.entity_id:
                raise ConfigurationError(
                    f"{name}: share_recipients 里有自己（{item!r}）—— "
                    "自己发给自己的报告会被环形报告过滤挡掉，白配一条"
                )
            out.append(entity.entity_id)
        return tuple(out)

    # -- 节拍 --------------------------------------------------------------

    def _tick(self, engine, event):
        self.ticks += 1
        self.absorb(engine.now)
        if self._inactive_us > 0:
            self.expire(engine.now)
        if self._share_us > 0 and engine.now >= self._next_share_us:
            self.broadcast(engine.now)
            self._next_share_us = engine.now + self._share_us
        return EventResult.RESCHEDULE

    # -- 三件事，各自可单独调用（诊断与测试不必跑引擎）-----------------------

    def absorb(self, now: int) -> int:
        """把收件箱里的航迹报告同化进本平台航迹表，返回收下几条。

        ★ 用 :meth:`take_messages_of_kind` 而不是全取：平台上同时挂着指挥
        控制器时，两者共用一个收件箱，全取会把"前进"这类指令一起吃掉。

        报文格式错了（缺字段 / 带本地哨兵）会**抛异常**，不吞——见
        :meth:`~milsim.services.store.Contact.from_dict`。
        """
        accepted = 0
        for message in self.view.take_messages_of_kind(TRACK_REPORT):
            _, verdict = self.view.relay_contact(
                message.payload,
                sender_id=message.sender_id,
                received_at=now,
                max_hops=self._max_hops,
            )
            counter = _VERDICT_COUNTERS.get(verdict)
            if counter is not None:
                setattr(self, counter, getattr(self, counter) + 1)
            if verdict == "accepted":
                accepted += 1
        return accepted

    def expire(self, now: int) -> list[int]:
        """撤掉超时未更新的航迹，返回被撤的目标 ID（升序）。

        ``now - detected_at > drop_after_inactive``。**没有滤波器**，所以
        一条航迹的位置**停在最后一次量测上**（不拿真位置外推，那是上帝视角
        泄漏）；"位置过时"与"航迹撤除"因此是两件事——位置从最后一刻就不再
        动了，撤除要等这条超时判据。
        """
        dropped: list[int] = []
        for contact in self.view.my_contacts():
            if now - contact.detected_at > self._inactive_us:
                if self.view.drop_contact(contact.target_id):
                    dropped.append(contact.target_id)
        self.expired += len(dropped)
        return sorted(dropped)

    def broadcast(self, now: int) -> int:
        """把本平台航迹逐个发给 ``share_recipients``，返回发出的报文数。

        报文进的是**在途队列**，见模块头的 warning：把它搬进收件箱的是
        通信件（``RF_COMM``，**v0.13.37 已落地**，§5.15）。
        """
        sent = 0
        for recipient in self._recipients:
            sent += len(self.view.share_tracks(recipient, sent_at=now, deliver_at=now))
        self.shared += sent
        return sent

    # -- 展示 --------------------------------------------------------------

    def statistics(self) -> dict[str, int]:
        """诊断快照。"我发了报告它说没收到"要靠这几个数归因。"""
        return {
            "ticks": self.ticks,
            "absorbed": self.absorbed,
            "fused": self.fused,
            "rejected_circular": self.rejected_circular,
            "rejected_hops": self.rejected_hops,
            "rejected_stale": self.rejected_stale,
            "expired": self.expired,
            "shared": self.shared,
            "contacts": len(self.view.my_contacts()),
        }


__all__ = ["TrackManager", "TRACK_PERIOD_US"]
