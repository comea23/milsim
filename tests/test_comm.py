"""通信件与通信网（§5.15）的测试。

分三层，每层都能单独核对：

1. :class:`~milsim.services.comm.CommService` —— **纯拓扑**，不碰引擎、不碰
   地图。"从 A 到 B 现在通不通"是这一层唯一的问题，所以它用最少的桩就能测；
2. 三阶段节拍 —— **标记 → 提交 → 传信**。顺序与"没变化就跳过"是这一层的
   核心断言（用户裁定）；
3. ``network`` 想定块 + 装配接线 —— 名字 → 实体 ID、单边通信自动入网、
   一个实体一个网。

★ 本文件里有两条**守门人**用例，各盯住一个曾经会静默通过的错：

* :func:`test_a_message_to_an_offline_node_stays_inflight` —— 到点不等于通。
  直接调 ``deliver_due`` 的话，被压制节点照样收得到信，而症状是"干扰对通信
  完全没用"；
* :func:`test_a_to_b_goes_around_a_down_node` —— 绕路不是断网。摘掉中间节点
  之后有旁路就必须走旁路，缓存路径会让"绕路"退化成"断网"（两者症状一样）。
* :func:`test_the_comm_link_budget_matches_the_radar_equation` —— **公式有一份
  副本**。门面不能 import ``models``（分层约定），所以 ``CommService`` 里手写了
  ``F_BW`` / 方向图 / ``F_POL`` / ``Jam.1`` 四个式子的**只读镜像**。这条用例把
  两边同时算一遍比数值——**它是"两处实现"能被允许的唯一条件**：有一处会报错。
"""

from __future__ import annotations

import math

import pytest

from milsim.errors import ConfigurationError
from milsim.services.comm import CommNet, CommNodeSpec, CommService
from milsim.services.scenario import load_scenario

# ---------------------------------------------------------------------------
# 第一层：纯拓扑
# ---------------------------------------------------------------------------


def _mesh(*ids: int, name: str = "N") -> CommService:
    """建一张**全连通**网（不写 ``link``，走 AFSIM 的默认口径）。"""
    service = CommService()
    service.declare(CommNet(name, members=tuple(ids)))
    return service


def test_an_empty_service_never_restricts_anything() -> None:
    """**一个网都没建 ⇒ 通通放行**（旧想定逐字不变的落点）。

    这条是验收判据而不是优化：真"没网就断"会让每一份既有想定突然发不出信，
    而症状是"什么都没动却全哑了"。
    """
    service = CommService()
    assert service.is_empty
    # 谁都不在名册上，却照样判"通"。
    assert service.path_exists(0, 1)
    assert service.path_exists(999, -5)


def test_two_members_of_the_same_net_can_reach_each_other() -> None:
    """全连通网：任何两个成员恒通（不写 ``link`` 就是 mesh）。"""
    service = _mesh(1, 2, 3)
    assert service.path_exists(1, 2)
    assert service.path_exists(1, 3)
    assert service.path_exists(3, 2)
    assert service.route(1, 3) == [1, 3]      # 直达，不绕
    assert not service.is_empty


def test_a_node_outside_every_net_has_no_comms_at_all() -> None:
    """**没入网 = 没通信能力**（同 AFSIM：不在网里又没显式 link 的通不到）。

    两类都算：外来实体（99）与跨网实体。前者是"根本没这个节点"，后者是
    "节点在，但不同网"——分开的判据在 ``path_exists`` 里是第 3、4 条。
    """
    service = CommService()
    service.declare(CommNet("A", members=(1, 2)))
    service.declare(CommNet("B", members=(3, 4)))
    assert not service.path_exists(1, 3)      # 跨网
    assert not service.path_exists(1, 99)     # 不在任何网
    assert not service.path_exists(99, 1)     # 反向也要判


def test_a_node_reaches_itself_even_when_it_is_down() -> None:
    """**自己给自己恒通**，且不受掉线影响（零长路径不是一个可断的链路）。"""
    service = _mesh(1, 2)
    service.mark_offline(1)
    service.commit()
    assert service.is_offline(1)
    assert service.path_exists(1, 1)


# ---------------------------------------------------------------------------
# 第二层：三阶段节拍（标记 → 提交 → 传信）
# ---------------------------------------------------------------------------


def test_marking_offline_alone_does_not_change_the_topology() -> None:
    """**标记阶段不碰拓扑**：没 ``commit`` 之前，寻路看到的还是旧图。

    这条是"顺序不能反"的**前半段**——标记只写意图。它同时也是"上一拍拓扑"
    这个错误之所以危险的原因：不 commit 就传信，用的是旧图。
    """
    service = _mesh(1, 2)
    service.mark_offline(2)
    assert service.path_exists(1, 2)          # 还没提交 ⇒ 照旧通
    assert service.is_offline(2) is False     # ``is_offline`` 读的是生效值
    assert service.pending_nodes == frozenset({2})
    assert service.epoch == 0


def test_commit_is_the_only_thing_that_moves_the_epoch() -> None:
    """**拓扑变了 epoch +1；没变一个字节不动**（用户裁定）。

    "没变化就跳过"是语义不只是性能：它让"这一拍拓扑改没改"变成一个可观察、
    可计数的事实（``epoch``）。诊断"信为什么从那儿走了"的第一个问题就是它。
    """
    service = _mesh(1, 2, 3)
    assert service.commit() is False          # 从来没标记过 ⇒ 没变化
    assert service.epoch == 0

    service.mark_offline(2)
    assert service.commit() is True
    assert service.epoch == 1

    # 重复标记同一个节点 ⇒ 集合没变 ⇒ 一次 commit 就够，再提也不动。
    service.mark_offline(2)
    assert service.commit() is False
    assert service.epoch == 1

    service.mark_online(2)
    assert service.commit() is True
    assert service.epoch == 2


def test_mark_offline_and_mark_online_are_idempotent_switches() -> None:
    """两个标记**幂等**，不引入"恢复"这个新状态（§5.15 裁定③）。

    按拍语义：这一拍被压 ⇒ 离线；下一拍没压住 ⇒ 自动回来。若两者不对称
    （比如 ``mark_online`` 会多记一笔"曾经离线"），"按拍"就变成了"状态机"。
    """
    service = _mesh(1, 2)
    service.mark_offline(2)
    service.mark_offline(2)
    service.mark_offline(2)
    service.commit()
    assert service.offline_nodes == frozenset({2})

    service.mark_online(2)
    service.mark_online(2)
    assert service.pending_nodes == frozenset()
    service.commit()
    assert service.offline_nodes == frozenset()


def test_an_offline_node_cannot_send_or_receive() -> None:
    """被标记掉线的节点**发不出、也收不到**（判据第 5 条）。"""
    service = _mesh(1, 2, 3)
    service.mark_offline(2)
    service.commit()
    assert not service.path_exists(2, 1)      # 它发不出
    assert not service.path_exists(1, 2)      # 也收不到
    assert service.path_exists(1, 3)          # 别人之间照旧
    assert service.route(1, 2) is None


def test_a_to_b_goes_around_a_down_node() -> None:
    """★ 守门人：**绕路不是断网**（§5.15 裁定①）。

    显式链 ``1-2-3`` 加旁路 ``1-3``：摘掉中间节点 2 之后，1→3 必须改走旁路。
    缓存路径（或"摘掉节点就删掉所有经过它的边"）会让它退化成"断网"，而
    "绕路"与"断网"的症状完全一样（信没到），只有诊断计数能分辨。
    """
    service = CommService()
    service.declare(CommNet(
        "N",
        members=(1, 2, 3),
        links=((1, 2), (2, 3), (1, 3)),
    ))
    assert service.route(1, 3) in ([1, 2, 3], [1, 3])
    # 摘掉 2 之后：直达那条还在，所以仍然通，而且路径里不含 2。
    service.mark_offline(2)
    service.commit()
    path = service.route(1, 3)
    assert path is not None and 2 not in path

    # 把旁路也摘掉的一个情形：只剩 1-2-3 链 + 摘掉 3 ⇒ 1→3 必须断。
    only_chain = CommService()
    only_chain.declare(CommNet("N", members=(1, 2, 3), links=((1, 2), (2, 3))))
    only_chain.mark_offline(3)
    only_chain.commit()
    assert not only_chain.path_exists(1, 3)


def test_a_chain_breaks_when_its_only_bridge_goes_down() -> None:
    """链式拓扑：摘掉必经节点 ⇒ 两端不再可达（这不是绕路，是真断）。"""
    service = CommService()
    service.declare(CommNet(
        "N", members=(1, 2, 3, 4), links=((1, 2), (2, 3), (3, 4))
    ))
    assert service.path_exists(1, 4)
    service.mark_offline(2)
    service.commit()
    assert not service.path_exists(1, 4)      # 2 是唯一桥
    assert service.path_exists(3, 4)          # 3-4 这一截还连着


def test_unregister_removes_a_node_permanently_and_at_once() -> None:
    """**注销 ≠ 掉线**：关机是**结构删除**，立即生效、不走待提交队列。

    合并这两件事会让"这一拍被压制"永久毁掉这个节点，而症状是"干扰早停了，
    它却再也收不到信"。
    """
    service = _mesh(1, 2, 3)
    assert service.unregister(2) is True
    assert service.unregister(2) is False     # 已经没了
    assert service.members_of("N") == (1, 3)
    assert not service.path_exists(1, 2)
    assert service.network_of(2) is None
    assert service.epoch == 1                 # 注销推 epoch


def test_route_is_reproducible_across_runs() -> None:
    """邻居按 **ID 升序** 遍历 ⇒ 同一份拓扑两次跑出同一条路。

    用 ``set`` 的天然顺序会让路径随哈希值变——而"两次跑结果不一样"是本项目
    最不能接受的那类不确定性。
    """
    first = _mesh(1, 2, 3, 4, 5)
    second = _mesh(1, 2, 3, 4, 5)
    assert first.route(1, 5) == second.route(1, 5)
    assert first.route(1, 5) == [1, 5]


def test_declaring_the_same_net_twice_is_rejected() -> None:
    """同名两张网**报错**：静默合并会让"我分了两个网"变成一个。"""
    service = _mesh(1, 2)
    with pytest.raises(ConfigurationError, match="已经声明过"):
        service.declare(CommNet("N", members=(3, 4)))


def test_a_node_cannot_belong_to_two_nets() -> None:
    """一个实体只能属于一个网：两个网会让"它算哪个"没有唯一答案。"""
    service = _mesh(1, 2)
    with pytest.raises(ConfigurationError, match="不能又加入"):
        service.declare(CommNet("M", members=(2, 3)))


def test_registering_an_unlisted_node_is_rejected() -> None:
    """想定没把它列进任何网 ⇒ 登记当场报错（装配缺陷，不是参数问题）。"""
    service = _mesh(1, 2)
    with pytest.raises(ConfigurationError, match="没有把该实体列进任何通信网"):
        service.register(CommNodeSpec(entity_id=99, network="", tag="RF_COMM"))


def test_a_node_may_register_with_an_empty_declared_name() -> None:
    """**空串 = 没声明**，不是"声明了一个空网名"（想定的常规写法）。

    想定的常规写法是"``network`` 块点名，平台块不写网名"，那时组件交回来的
    就是参数默认值空串。只有**显式写了一个对不上的名字**才是配置错误。
    """
    service = _mesh(1, 2)
    service.register(CommNodeSpec(entity_id=1, network="", tag="RF_COMM"))
    service.register(CommNodeSpec(entity_id=2, network="N", tag="RF_COMM"))

    with pytest.raises(ConfigurationError, match="一个节点一个网"):
        service.register(CommNodeSpec(entity_id=1, network="M", tag="RF_COMM"))


# ---------------------------------------------------------------------------
# 第三层：按拍投递（到点 ≠ 通）
# ---------------------------------------------------------------------------


class _FakeStore:
    """只实现 ``deliver`` 用得到的那几个方法的最小存储桩。"""

    def __init__(self) -> None:
        from milsim.services.store import Message

        self._Message = Message
        self._seq = 0
        self.inflight: list = []
        self.inbox: dict[int, list] = {}

    def enqueue(self, sender: int, recipient: int, kind: str, deliver_at: int = 0):
        self._seq += 1
        message = self._Message(
            msg_id=self._seq,
            sender_id=sender,
            recipient_id=recipient,
            kind=kind,
            sent_at=0,
            deliver_at=deliver_at,
            payload={},
        )
        self.inflight.append(message)
        return message

    def due_messages(self, now: int):
        return [m for m in self.inflight if m.deliver_at <= now]

    def forget_due(self, messages):
        drop = {id(m) for m in messages}
        before = len(self.inflight)
        self.inflight = [m for m in self.inflight if id(m) not in drop]
        return before - len(self.inflight)

    def post_message(self, sender_id, recipient_id, kind, sent_at=0,
                     deliver_at=0, payload=None):
        self.inbox.setdefault(recipient_id, []).append(kind)


def test_a_message_to_an_offline_node_stays_inflight() -> None:
    """★ 守门人：**到点不等于通**。

    一条报文到点了、但收件方此刻被压制 ⇒ 它**这一拍不投**，留在在途队列里。
    直接调 ``deliver_due`` 的话（它只看 ``deliver_at <= now``），被压制的节点
    照样收得到信，而症状是"干扰对通信完全没用"。
    """
    store = _FakeStore()
    service = CommService(store)
    service.declare(CommNet("N", members=(1, 2)))
    store.enqueue(1, 2, "track_report", deliver_at=0)

    service.mark_offline(2)
    service.commit()
    assert service.deliver(1_000_000) == 0
    assert len(store.inflight) == 1           # 还在途，等下一拍
    assert 2 not in store.inbox

    # 下一拍恢复 ⇒ 同一份报文这次投出去了。
    service.mark_online(2)
    service.commit()
    assert service.deliver(2_000_000) == 1
    assert store.inflight == []
    assert store.inbox[2] == ["track_report"]


def test_deliver_needs_no_store() -> None:
    """没有存储时 ``deliver`` 安静地返回 0（纯拓扑单元测试）。"""
    assert CommService().deliver(0) == 0


def test_deliver_skips_cross_net_messages() -> None:
    """跨网报文投不出去：它留在在途队列里，不会被硬塞进收件箱。"""
    store = _FakeStore()
    service = CommService(store)
    service.declare(CommNet("A", members=(1,)))
    service.declare(CommNet("B", members=(2,)))
    store.enqueue(1, 2, "track_report", deliver_at=0)
    assert service.deliver(1_000_000) == 0
    assert store.inflight and 2 not in store.inbox


# ---------------------------------------------------------------------------
# 第四层：network 想定块
# ---------------------------------------------------------------------------

_SCENARIO_HEAD = """
zone THEATER
    anchor latlng 39.90 116.40
    radius 60 km
    resolution 200 m
    layers 3
end_zone

simulation
    max_step 2 s
end_simulation

platform_type COMMS_NODE
    component comm0 RF_COMM
        period 2 s
    end_component
end_platform_type
"""


def _scenario(body: str) -> str:
    return _SCENARIO_HEAD + body


def test_network_block_reads_members_and_links() -> None:
    """``member`` 一行多值、``link`` 有向、``to`` 连接词可有可无。"""
    spec = load_scenario(_scenario(
        """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform
platform B COMMS_NODE
    position latlng 39.99 116.40
end_platform
platform C COMMS_NODE
    position latlng 39.91 116.45
end_platform

network BLUE_NET
    member A B
    member C
    link A to B
    link B C
end_network
"""
    ))
    assert spec.network_names() == ["BLUE_NET"]
    net = spec.networks[0]
    assert net.members == ("A", "B", "C")
    assert net.links == (("A", "B"), ("B", "C"))
    assert spec.network_of("B") == "BLUE_NET"
    assert spec.network_of("Z") == ""


def test_a_duplicate_member_is_harmless() -> None:
    """同一个成员写两遍 ⇒ 只留一份（不是报错，也不是两条记录）。"""
    spec = load_scenario(_scenario(
        """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform

network N
    member A A
end_network
"""
    ))
    assert spec.networks[0].members == ("A",)


def test_an_unknown_directive_in_a_network_block_is_rejected() -> None:
    """写错的指令词报错，并列出可用的两个。"""
    with pytest.raises(Exception, match="member / link"):
        load_scenario(_scenario(
            """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform

network N
    join A
end_network
"""
        ))


def test_two_network_blocks_with_the_same_name_are_rejected() -> None:
    """同名两张网报错（与 ``declare`` 同一条理由，但报在更早的地方）。"""
    with pytest.raises(Exception, match="重复定义"):
        load_scenario(_scenario(
            """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform

network N
    member A
end_network

network N
    member A
end_network
"""
        ))


# ---------------------------------------------------------------------------
# 第五层：装配接线与端到端
# ---------------------------------------------------------------------------


def _assemble(body: str):
    from milsim import models
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    spec = load_scenario(_scenario(body))
    registry = ComponentRegistry()
    models.register_framework(components=registry)
    sim = Simulation(spec, components=registry)
    sim.build()
    return sim


_PLATFORMS = """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform
platform B COMMS_NODE
    position latlng 39.99 116.40
end_platform
platform C COMMS_NODE
    position latlng 39.91 116.45
end_platform
platform D COMMS_NODE
    position latlng 39.93 116.30
end_platform
"""

#: 只含 A/B/C 的变体。★ 挂了 ``RF_COMM`` 的平台**必须在某个网里**（§5.15）：
#: 一台"在网外"的通信件登记不上去，而那正是"它为什么收不到信"最容易的答案。
#: 用到 D 的用例必须把它也列进某个网（见 :data:`_FOUR_IN_TWO_NETS`）。
_THREE = """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform
platform B COMMS_NODE
    position latlng 39.99 116.40
end_platform
platform C COMMS_NODE
    position latlng 39.91 116.45
end_platform
"""


def test_assembly_resolves_names_to_entity_ids() -> None:
    """装配把**平台实例名**解析成实体 ID，门面拿到的是 ID。"""
    sim = _assemble(_PLATFORMS + """
network BLUE_NET
    member A B C
end_network

network RED_NET
    member D
end_network
""")
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABCD"}
    assert sim.comm.members_of("BLUE_NET") == (
        ids["A"], ids["B"], ids["C"]
    )
    assert sim.comm.members_of("RED_NET") == (ids["D"],)
    assert sim.comm.path_exists(ids["A"], ids["B"])
    assert not sim.comm.path_exists(ids["A"], ids["D"])   # 跨网


def test_a_comms_component_outside_every_net_cannot_register() -> None:
    """★ 挂了 ``RF_COMM`` 却不在任何网里 ⇒ **装配期报错**。

    一个"自称在某网、而想定没把它列进去"的通信件会让寻路的成员表与宣称
    不一致——而症状是"它收不到信"，看起来像被干扰了。这种歧义必须在
    装配期就消掉（与干扰机的"登记不上去等于不存在"同一条理由）。
    """
    from milsim import models
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    spec = load_scenario(_scenario(_PLATFORMS + """
network BLUE_NET
    member A B C
end_network
"""))
    registry = ComponentRegistry()
    models.register_framework(components=registry)
    sim = Simulation(spec, components=registry)
    sim.build()
    with pytest.raises(ConfigurationError, match="没有把该实体列进任何通信网"):
        sim.initialize()


def test_an_empty_network_list_leaves_comms_unrestricted() -> None:
    """没写 ``network`` 块 ⇒ 门面为空 ⇒ 一切照旧放行（零回归）。

    ★ 这种想定里**不能**挂通信件（没有网就登记不上去，见上一条），所以这里
    只测门面本身：``is_empty`` 与"不限制"的语义。
    """
    sim = _assemble(_PLATFORMS)
    assert sim.comm.is_empty
    assert sim.comm.path_exists(0, 1)
    assert sim.comm.path_exists(0, 999)


def test_a_link_only_net_narrows_the_topology() -> None:
    """写了 ``link`` ⇒ 只用那些边，不再全连通。"""
    sim = _assemble(_THREE + """
network CHAIN
    member A B C
    link A B
    link B C
end_network
""")
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABC"}
    assert sim.comm.path_exists(ids["A"], ids["C"])          # 经 B 可达
    assert not sim.comm.path_exists(ids["C"], ids["A"])      # 边有向，反向不通


def test_a_link_endpoint_outside_the_members_is_rejected() -> None:
    """``link`` 端点不是本网成员 ⇒ 装配期报错（link 只在网内连边）。"""
    with pytest.raises(Exception, match="不是本网成员"):
        _assemble(_PLATFORMS + """
network N
    member A B
    link A D
end_network
""")


def test_a_platform_listed_in_two_nets_is_rejected() -> None:
    """想定把同一个平台列进两个网 ⇒ 装配期报错。"""
    with pytest.raises(Exception, match="一个节点只能属于一个网"):
        _assemble(_PLATFORMS + """
network ONE
    member A B
end_network

network TWO
    member A C
end_network
""")


def test_a_network_member_that_is_not_in_the_scenario_is_rejected() -> None:
    """``member`` 写了想定里没有的名字 ⇒ 报错（不是静默少一个节点）。"""
    with pytest.raises(Exception, match="不在想定里"):
        _assemble(_THREE + """
network N
    member A GHOST
end_network
""")


def test_messages_are_delivered_on_the_tick() -> None:
    """端到端：报文进在途队列 → 按拍搬到收件箱；拓扑没变时 ``epoch`` 不动。"""
    sim = _assemble(_THREE + """
network BLUE_NET
    member A B C
end_network
""")
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABC"}
    sim.store.enqueue_message(
        ids["A"], ids["B"], "track_report", sent_at=0, deliver_at=0
    )
    assert len(sim.store.inflight_messages()) == 1

    sim.initialize()
    sim.run_for(4_000_000)

    assert sim.store.inflight_messages() == []
    assert [m.kind for m in sim.store.peek_messages(ids["B"])] == ["track_report"]
    assert sim.comm.epoch == 0            # 拓扑一次都没动过
    assert sim.comm.commits > 0           # 但每拍都提了（没变化就跳过）
    assert sim.comm.statistics()["delivered"] == 1


def test_a_message_across_nets_never_arrives() -> None:
    """端到端：跨网报文永远投不出去（留在在途队列里）。"""
    sim = _assemble(_PLATFORMS + """
network ONE
    member A B
end_network

network TWO
    member C D
end_network
""")
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABC"}
    sim.store.enqueue_message(
        ids["A"], ids["C"], "track_report", sent_at=0, deliver_at=0
    )
    sim.initialize()
    sim.run_for(4_000_000)
    assert len(sim.store.inflight_messages()) == 1
    assert sim.store.peek_messages(ids["C"]) == []


def test_shutdown_unregisters_the_node() -> None:
    """关机 ⇒ 注销：拓扑上少一个节点，epoch 推一格。"""
    sim = _assemble(_THREE + """
network BLUE_NET
    member A B C
end_network
""")
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABC"}
    sim.initialize()
    assert sim.comm.path_exists(ids["A"], ids["B"])
    sim.entities[ids["B"]].shutdown()
    assert not sim.comm.path_exists(ids["A"], ids["B"])


# ---------------------------------------------------------------------------
# 第六层：接干扰（v0.13.38，§5.15 第二步）
#
# 这一层的四件事，各自盯住一个"会静默通过"的错：
#
# 1. **对照测试**（第一条）——门面手写的那几个式子与方程模块逐字一致。
#    它是"公式有一份副本"能被允许的**唯一条件**：有一处会报错。
# 2. **压制 ⇒ 从拓扑上消失** —— 干扰生效的唯一后果就是 mark_offline。
# 3. **没挂干扰机 ⇒ 逐字不变** —— 零开销是正确性，不是优化。
# 4. **门限边界与幂等** —— 判据恰好落在门限哪一侧、同拍求两次会不会重复计数。
# ---------------------------------------------------------------------------


def _jammer_spec(**overrides):
    """一份最小的干扰机卡（对照测试与门面单测用）。

    ★ 与 ``tests/test_percep.py`` 的那个桩**同口径**（默认值逐项一致）：同一
    份物理量在两处测试里必须给同一个数，否则"通信被压住"与"雷达被压住"会
    在测试层面就先分叉了。
    """
    from milsim.services.ew import JammerSpec

    fields = dict(
        entity_id=1,
        power_w=100.0,
        gain=1000.0,
        frequency_hz=4.0e9,
        bandwidth_hz=0.0,
        duty_cycle=1.0,
        use_peak_power=False,
        internal_loss=1.0,
        polarization=-1.0,
        polarization_type="default",
        ignore_same_side=False,
        tag="JAMMER",
    )
    fields.update(overrides)
    return JammerSpec(**fields)


class _Sides:
    """``registry`` 的最小桩：``JamService`` 只问 ``side_of`` 与 ``by_name``。"""

    def __init__(self) -> None:
        self.sides: dict[int, str] = {}

    def side_of(self, entity_id: int) -> str:
        return self.sides.get(entity_id, "")


def _spec(entity_id: int, **overrides) -> CommNodeSpec:
    """一张收信机能力卡（门面单测用）。默认**门限 −10 dB、名义 S/N 100**。"""
    fields = dict(
        entity_id=entity_id,
        network="N",
        tag="RF_COMM",
        power_w=1.0,
        tx_gain=1.0,
        rx_gain=1.0,
        frequency_hz=4.0e9,
        bandwidth_hz=2.0e6,
        rx_loss=1.0,
        noise_w=1.0e-6,
        snr=100.0,
        required_snr_db=-10.0,
    )
    fields.update(overrides)
    return CommNodeSpec(**fields)


# -- 1. 对照测试：门面手写的镜像式子 == 方程模块 ---------------------------


def test_the_comm_link_budget_matches_the_radar_equation() -> None:
    """★ 守门人：**门面手写的四个式子与方程模块逐字一致**。

    门面（``services``）不能 import ``models.percep.equation``——那会撞破本项目
    最硬的一条分层约定。代价是 ``CommService`` 里手写了 ``F_BW`` / 天线方向图 /
    ``F_POL`` / ``Jam.1`` 四个式子的**只读镜像**。

    ⇒ **这条用例就是那份副本能被允许的唯一理由**：两处实现各算一遍、逐位比对。
    没有它的话，方程模块改一次（比如把 ``F_BW`` 的分母换成接收带宽），通信侧
    会**静默地**继续用旧式子——症状是"同一个干扰机对雷达有效、对通信无效"，
    而在参数表上一个字都看不出来。

    ★ 两个**有意保留**的差异（不是"没比出来"）：

    * ``F_BW`` 在接收带宽 ≤ 0 时：方程模块**报错**（那是一个无效的接收机），
      镜像**返 0**（门面在求值链上，遇到坏数据该"压不住"而不是把整个推演炸掉）。
      两者在这一条上都对，但方向相反——所以下面单独判，**不放进"逐位相等"**。
    * ``Jam.1``：镜像把 ``wavelength_m`` 换成"由频率现取"（``C/f``），于是
      多一次乘除往返 ⇒ 末位可能有 **1 ULP** 的差（实测最大相对差 1.6e-16）。
      这是**浮点结合律**，不是公式不同，所以给一个明确的容差并**说明它为什么
      存在**；给成 0 的话这条用例会随平台/编译器抖，而被删掉。
    """
    from milsim.models.percep import equation as eq

    # -- F_BW：逐位相等（含带内 / 阻塞 / 错开 / 边界 / CW 几种形状）--------
    bw_cases = (
        (4.0e9, 0.0, 4.0e9, 2.0e6),       # CW 正好落在带心
        (4.0e9, 1.0e6, 4.0e9, 2.0e6),     # 瞄准式（窄，全进）
        (4.0e9, 20.0e6, 4.0e9, 2.0e6),    # 阻塞式 ⇒ B_r/B_j = 0.1
        (4.0e9, 2.0e6, 4.0e9, 2.0e6),     # 恰好等宽
        (1.0e9, 1.0e6, 4.0e9, 2.0e6),     # 完全错开
        (4.001e9, 1.0e6, 4.0e9, 2.0e6),   # 半边重叠 ⇒ 0.5
        (4.099e9, 1.0e6, 4.0e9, 2.0e6),   # 刚好擦边（外）
    )
    for args in bw_cases:
        assert CommService._bandwidth_overlap(*args) == eq.bandwidth_overlap_ratio(*args)

    # -- 有意差异：接收带宽 0 时一边报错、一边返 0 -------------------------
    with pytest.raises(ValueError):
        eq.bandwidth_overlap_ratio(4.0e9, 1.0e6, 4.0e9, 0.0)
    assert CommService._bandwidth_overlap(4.0e9, 1.0e6, 4.0e9, 0.0) == 0.0

    # -- 天线方向图：逐位相等（轴心 / 边缘 / 刚出主瓣 / 各向同性）----------
    sidelobe = 10.0 ** (-13.0 / 10.0)
    for off, width in ((0.0, 10.0), (5.0, 10.0), (5.0000001, 10.0), (3.0, 0.0)):
        assert CommService._pattern_factor(
            off, width, sidelobe
        ) == eq.antenna_pattern_factor(off, width, sidelobe)

    # -- F_POL：整张 7×7 表逐格相等（这一条最重要：抄表最容易过期）--------
    for receiver in eq.POLARIZATION_KINDS:
        for transmitter in eq.POLARIZATION_KINDS:
            assert CommService._polarization(
                receiver, transmitter, -1.0
            ) == eq.polarization_effect(receiver, transmitter, -1.0)
    # 显式覆盖：两边都原样返回（负数 = 未给才查表）
    assert CommService._polarization("horizontal", "vertical", 0.25) == 0.25

    # -- Jam.1：相对差 ≤ 1 ULP（见 docstring 那条说明）---------------------
    wavelength = eq.C_LIGHT / 4.0e9
    reference = eq.jamming_power(
        1000.0, 10.0, 2.0, wavelength, 5_000.0,
        loss=5.0, bandwidth_overlap=0.5, polarization=0.5,
    )
    mirror = CommService._jam_power(
        1000.0, 10.0, 2.0, 4.0e9, 5_000.0,
        loss=5.0, bandwidth_overlap=0.5, polarization=0.5,
    )
    assert reference > 0.0
    assert math.isclose(mirror, reference, rel_tol=1e-15)


def test_the_mirror_uses_the_same_sidelobe_constant() -> None:
    """副瓣水平是**同一个物理量**的两个使用者 ⇒ 从一处取，不各写一份。

    雷达接收机与通信接收机用的是同一个 ``DEFAULT_SIDELOBE_LEVEL_DB``：各写一份
    迟早不同步，而且**不同步时不报错**——症状是"同一台干扰机从副瓣压雷达扣 13 dB、
    压通信扣 20 dB"，参数表上完全看不出来。
    """
    from milsim.services.comm import DEFAULT_SIDELOBE_LEVEL_DB
    from milsim.services.ew import DEFAULT_SIDELOBE_LEVEL_DB as EW_SIDELOBE

    assert DEFAULT_SIDELOBE_LEVEL_DB == EW_SIDELOBE


# -- 2. 压制 ⇒ 从拓扑上消失（门面单测） -----------------------------------


def _jammed_service(*, required_snr_db: float, jammer_id: int = 3, **jam_overrides):
    """建"通信节点 1/2 + 一台干扰机（实体 3）"的门面，返回 ``(门面, 名册)``。

    ★ **干扰机必须是另一个实体**：名册的三道筛里第一道就是"一台干扰机不压
    自己"（``spec.entity_id == victim_id`` ⇒ 跳过）。把干扰机挂在受害节点自己
    的 ID 上，``threats()`` 恒返空 ⇒ ``jammed_now`` 恒 0，而**症状是"干扰完全
    没用"**——正是本层要抓的那类静默错误，所以夹具用独立的实体 3。

    ★ **还必须给真的位置**：另两道筛是"受害方要有位置"（``position_of`` 返
    ``None`` ⇒ 直接返回空元组）与"两台机不可以同处一点"（斜距为零时
    ``jamming_power`` 会拒绝）。

    几何：节点 1 在原点、节点 2 在它以东 1 km；干扰机默认放在**原点以东 1 km**
    处（靠功率与距离把``S/(N+J)`` 压到门限以下）。
    """
    from milsim.services.ew import JamService
    from milsim.services.store import EntityStore

    store = EntityStore()
    for entity_id in (1, 2, jammer_id):
        if store.has(entity_id):
            continue
        store.add(entity_id)
        store.set_pose(
            entity_id, 0.0 if entity_id != jammer_id else 1000.0, 0.0, 0.0
        )

    service = CommService(store, None)
    service.declare(CommNet("N", members=(1, 2)))
    service.register(_spec(1, required_snr_db=required_snr_db))
    service.register(_spec(2, required_snr_db=required_snr_db))

    jam = JamService(store, _Sides())
    jam.register(_jammer_spec(entity_id=jammer_id, **jam_overrides))
    service._jam = jam
    return service, jam


def test_a_jammed_node_is_marked_offline_and_leaves_the_topology() -> None:
    """★ 干扰生效的**唯一后果**就是 ``mark_offline``：这一拍它从拓扑上消失。

    （§5.15 裁定①：被干扰 ⇒ **从拓扑上消失、报文绕路**，不是"断网"。）
    于是"干扰压住通信"不需要任何新的后果模型——它与"它关机了"共用一条通路。

    ★ 夹具的几何：受害节点 1/2 都在原点，干扰机在东边 1 km，瞄准式
    （``B_j = 0`` ⇒ 连续波）。实测 ``S/(N+J) = 53.41307054040149 dB``
    （见 :data:`_NARROW_SNR_DB`）⇒ 门限取 60 dB 一定压得住。
    """
    service, _ = _jammed_service(required_snr_db=60.0)   # 门限高 ⇒ 必定被压
    assert service.path_exists(1, 2)

    marked = service.jammed_now(0)
    assert marked == 2                     # 两个节点都被压
    assert service.is_offline(1) is False  # 还没 commit ⇒ 拓扑没动（阶段①的语义）
    service.commit()
    assert service.is_offline(1) is True
    assert not service.path_exists(1, 2)   # 从拓扑上摘掉 ⇒ 通不到
    assert service.statistics()["jammed"] == 2


#: 上面那套几何下实测的 ``10·log10(S/(N+J))``（dB）。瞄准式（``B_j ≤ B_r``）
#: 是一个数，阻塞式（``B_j ≫ B_r``）是另一个数——两者的差就是 ``F_BW`` 的账。
#: ★ 这两个**不是**手算出来的：由探针脚本按门面那四个镜像式子复算得到，
#: 而它们与方程模块一致由
#: :func:`test_the_comm_link_budget_matches_the_radar_equation` 保证。
_NARROW_SNR_DB = 53.41307054040149      # B_j = 0（CW）或 1 MHz（≤ B_r）
_WIDE_SNR_DB = 59.28907321434249        # B_j = 40 MHz（阻塞式，F_BW = 0.05）


def test_a_long_hop_can_go_around_a_jammed_node() -> None:
    """★ 被压住的节点被摘掉之后，**有旁路就走旁路**（与掉线同一条通路）。

    这条把"干扰"与"绕路"接起来：干扰的后果不是"信全丢了"，而是"这一拍这条路
    不通了"。若摘点之后不再重算路径（缓存），"绕路"会静默退化成"断网"。

    几何：链 ``1-2-3`` 加旁路 ``1-3``；干扰机**只够得着中间节点 2**（架在 2 号
    附近 1 km 处），于是只有 2 号被摘掉，1→3 必须改走旁路。

    ★ 三个距离（1 号 0 m、2 号 5 km、3 号 50 km，干扰机 6 km）配门限 59 dB
    是**算出来的**，不是凑的：``J ∝ 1/R²``，所以 2 号看到的 ``S/(N+J) = 53.41``
    （< 59 ⇒ 压住）、1 号 59.59、3 号 59.99（> 59 ⇒ 压不住）。把 3 号放到
    20 km 是不够的（那里是 59.96 dB，离门限只差 0.04 dB，随几何一抖就翻），
    所以这里**取远 10 倍**把余量做厚。
    """
    from milsim.services.ew import JamService
    from milsim.services.store import EntityStore

    store = EntityStore()
    for entity_id, x in ((1, 0.0), (2, 5000.0), (3, 50_000.0), (4, 6000.0)):
        store.add(entity_id)
        store.set_pose(entity_id, x, 0.0, 0.0)
    service = CommService(store, None)
    service.declare(CommNet(
        "N", members=(1, 2, 3), links=((1, 2), (2, 3), (1, 3))
    ))
    for entity_id in (1, 2, 3):
        service.register(_spec(entity_id, required_snr_db=59.0))

    jam = JamService(store, _Sides())
    jam.register(_jammer_spec(entity_id=4))
    service._jam = jam

    assert service.route(1, 3) in ([1, 2, 3], [1, 3])
    service.jammed_now(0)
    service.commit()
    #: **只有中间那个节点被摘掉**（干扰机就架在它旁边）。
    assert service.is_offline(2)
    assert not service.is_offline(1)
    assert not service.is_offline(3)
    path = service.route(1, 3)
    assert path is not None and 2 not in path     # 改走旁路，不是断网
    assert path == [1, 3]


def test_a_jammed_bridge_breaks_the_chain() -> None:
    """★ 只有一条路、而那条路被干扰掐断 ⇒ **真断**（不是绕路）。

    与上一条是一对：同样的"摘掉一个节点"，有旁路就绕、没旁路就断。少了这一条，
    "有旁路就走旁路"可能被实现成"永远绕得开"，而症状是"信还是到了"。
    """
    from milsim.services.ew import JamService
    from milsim.services.store import EntityStore

    store = EntityStore()
    for entity_id, x in ((1, 0.0), (2, 5000.0), (3, 50_000.0), (4, 6000.0)):
        store.add(entity_id)
        store.set_pose(entity_id, x, 0.0, 0.0)
    service = CommService(store, None)
    service.declare(CommNet("N", members=(1, 2, 3), links=((1, 2), (2, 3))))
    for entity_id in (1, 2, 3):
        service.register(_spec(entity_id, required_snr_db=59.0))

    jam = JamService(store, _Sides())
    jam.register(_jammer_spec(entity_id=4))        # 与 2 号相近（1 km）
    service._jam = jam

    assert service.path_exists(1, 3)
    service.jammed_now(0)
    service.commit()
    assert service.is_offline(2)
    assert not service.path_exists(1, 3)           # 2 是唯一桥 ⇒ 真断
    assert service.route(1, 3) is None


def test_a_wider_bandwidth_jammer_needs_more_power() -> None:
    """阻塞式（``B_j`` 宽）比瞄准式（``B_j`` 窄）**难压住** ⇒ ``F_BW`` 真的进了账。

    同一个功率、同一个门限（56 dB，卡在实测的两个 ``S/(N+J)`` 之间：瞄准式
    ``53.41``、阻塞式 ``59.29``），只把干扰机谱宽从 1 MHz 加到 40 MHz ⇒
    ``F_BW`` 从 1 掉到 0.05 ⇒ ``J`` 少 13 dB ⇒ 压不住。若 ``F_BW`` 被漏掉
    （照抄时最常漏的一项），这条会**静默通过**——两种带宽给出同一个结论。
    """
    assert _NARROW_SNR_DB < 56.0 < _WIDE_SNR_DB     # 门限的选择有依据
    narrow, _ = _jammed_service(required_snr_db=56.0, bandwidth_hz=1.0e6)
    wide, _ = _jammed_service(required_snr_db=56.0, bandwidth_hz=40.0e6)

    assert narrow.jammed_now(0) == 2         # 瞄准式：压得住
    assert wide.jammed_now(0) == 0           # 阻塞式：压不住（F_BW = 0.05）


def test_a_jammed_node_comes_back_when_the_jamming_stops() -> None:
    """**掉线是按拍可逆的**：下一拍没压住 ⇒ 自动回来（它不是注销）。

    这条与 :meth:`test_unregister_removes_a_node_permanently_and_at_once`
    是一对：合并这两件事会让"这一拍被压制"永久毁掉这个节点，而症状是
    "干扰早停了，它却再也收不到信"。
    """
    service, jam = _jammed_service(required_snr_db=60.0)
    service.jammed_now(0)
    service.commit()
    assert service.is_offline(1)

    #: 干扰机被击毁（注销）⇒ 这一拍没人被求值成被压住。「恢复」这条通路是
    #: **调用方**把没压住的标回在线（门面只回答"谁被压住"，不回答"谁恢复"）。
    jam.unregister(3)
    service.mark_online(1)
    service.mark_online(2)
    service.commit()
    assert not service.is_offline(1)
    assert service.path_exists(1, 2)


def test_jammed_now_is_idempotent_within_one_tick() -> None:
    """同一拍重复求值**不重复计数**：已经在待提交集合里的节点跳过。

    ★ 这条守的是"每拍只调一次"这条约定被破坏时的可观察后果：如果重复调会把
    同一个节点数两遍（``jammed`` 涨成 2 倍），"这一拍压住了几个"这个诊断量
    就再也对不上"名册里有几个节点"。
    """
    service, _ = _jammed_service(required_snr_db=60.0)
    first = service.jammed_now(0)
    second = service.jammed_now(0)
    assert first == 2
    assert second == 0                    # 两个都已在 pending ⇒ 一个都不再求值
    assert service.statistics()["jammed"] == 2
    assert service.statistics()["screened"] == 2


def test_an_off_band_jammer_cannot_suppress_anything() -> None:
    """带外的干扰机**一分都进不来**（``F_BW = 0`` ⇒ 不判压制）。

    这条与"缺数据不伪装成被压住"是同一类判据：干扰机挂在名册上、功率很大、
    距离很近，但**频率差得远**（40 GHz 对 4 GHz 调谐带）⇒ 它压不住。若不判
    ``F_BW``，任何一台在册的干扰机都会压住任何通信（症状：换个频段就失效了，
    而参数表上处处有值）。
    """
    service, _ = _jammed_service(required_snr_db=60.0, frequency_hz=40.0e9)
    assert service.jammed_now(0) == 0
    assert service.is_offline(1) is False


def test_a_node_without_noise_or_power_is_not_suppressed() -> None:
    """缺 ``noise_power`` / ``power`` ⇒ **不判干扰**（缺数据 ≠ 被压住）。

    真拿 ``N = 0`` 去算 ``J/S`` 会得到无穷大、进而"任何干扰都压得住"——那正是
    "少写一个参数，通信就悄悄哑了"。返 0 的意思是"这个数不知道"，不是"信噪比
    为零"（同 §9-42 那条"0 落在指数上才是缺数据"）。

    ★ 两个变体一起测，因为它们是**两条不同的早退**：``power = 0`` 让
    ``signal <= 0``、``noise = 0`` 让 ``noise <= 0``，但两者都落在
    :meth:`CommService._suppressed` 里那一句 ``if signal <= 0 or noise <= 0``
    上——将来若把这一句拆成两句、只改其中一句，这条会红。
    """
    from milsim.services.ew import JamService
    from milsim.services.store import EntityStore

    store = EntityStore()
    for entity_id, x in ((1, 0.0), (2, 0.0), (3, 1000.0)):
        store.add(entity_id)
        store.set_pose(entity_id, x, 0.0, 0.0)

    service = CommService(store, None)
    service.declare(CommNet("N", members=(1, 2)))
    #: 门限 60 dB —— 正常节点在这个几何下**一定被压**（实测 53.41 dB），
    #: 所以"没被压"只能来自"缺数据 ⇒ 不判"这一条，不会是功率不够。
    service.register(_spec(1, noise_w=0.0, required_snr_db=60.0))     # 缺 N
    service.register(_spec(2, power_w=0.0, required_snr_db=60.0))     # 缺 S

    jam = JamService(store, _Sides())
    jam.register(_jammer_spec(entity_id=3))
    service._jam = jam

    assert service.jammed_now(0) == 0      # 两个都缺数据 ⇒ 一个都不判
    assert service.statistics()["screened"] == 2   # 但**求值过了**（不是早退）
    assert service.statistics()["jammed"] == 0
    assert service.is_offline(1) is False

    #: 对照组：把 1 号的噪声补回来 ⇒ 同一套几何下它立刻被压住。
    #: 这一步证明"上面那个 0 是缺数据挡下来的"，而不是"这套几何压不住"。
    service.register(_spec(1, required_snr_db=60.0))
    assert service.jammed_now(0) == 1


# -- 3. 门限边界：判据恰好落在门限哪一侧 ----------------------------------


def test_the_required_snr_threshold_is_compared_in_decibels() -> None:
    """门限比较在 **dB** 上做：``10·log10(S/(N+J)) < required_snr_db``。

    约定是 **``<``**（严格小于）⇒ **恰好等于门限不算被压住**。这一条必须写死：
    改成 ``<=`` 的话边界两侧各有一条用例会红，而"刚好卡在门限上"是最常被
    讨论的那个点。

    实测这条几何下 ``S/(N+J) = :data:`_NARROW_SNR_DB```（dB），所以直接拿它
    当门限：相等 ⇒ 通；再加一个极小量 ⇒ 压。**不二分、不算**——数从探针来，
    而那个数与方程模块一致由对照用例保证。
    """
    at = _jammed_service(required_snr_db=_NARROW_SNR_DB)[0]
    assert at._suppressed(1, _spec(1, required_snr_db=_NARROW_SNR_DB)) is False

    above = _jammed_service(required_snr_db=_NARROW_SNR_DB + 1e-9)[0]
    assert above._suppressed(1, _spec(1, required_snr_db=_NARROW_SNR_DB + 1e-9)) is True

    #: 逐层下沉：门限降 1 dB ⇒ 也通（判据是单调的）。
    below = _jammed_service(required_snr_db=_NARROW_SNR_DB - 1.0)[0]
    assert below._suppressed(1, _spec(1, required_snr_db=_NARROW_SNR_DB - 1.0)) is False


# -- 4. 没挂干扰机 ⇒ 逐字不变（零回归） -----------------------------------


def test_no_jammer_means_zero_overhead() -> None:
    """**名册为空 / 没有门面 ⇒ 一步早退**（连位置都不查）。

    这是"没挂干扰机的想定逐字不变"在通信侧的落点，而且它**不是靠某个 ``if``
    挡出来的**，是"没有这个东西"的自然结果：``jam is None`` 与 ``jam.is_empty``
    走同一条返回 0 的路。两个计数器保持 0 ⇒ "零开销"是一个**可观察的事实**。
    """
    service = _mesh(1, 2)
    service.register(_spec(1))
    service.register(_spec(2))
    assert service._jam is None
    assert service.jammed_now(0) == 0
    assert service.statistics()["screened"] == 0
    assert service.statistics()["jammed"] == 0

    # 门面在、但名册是空的：结果完全一样（"这场没有干扰"）。
    from milsim.services.ew import JamService
    from milsim.services.store import EntityStore

    empty = JamService(EntityStore(), _Sides())
    service._jam = empty
    assert service.jammed_now(0) == 0
    assert service.statistics()["screened"] == 0


def test_a_scenario_without_a_jammer_keeps_the_digits_unchanged() -> None:
    """端到端：**没挂干扰机**的想定里，``jammed_now`` 恒 0、epoch 不动。

    与 :func:`test_messages_are_delivered_on_the_tick` 合起来就是"第二步
    接进来之后，第一步那些数字一个都没变"的验收。
    """
    sim = _assemble(_THREE + """
network BLUE_NET
    member A B C
end_network
""")
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABC"}
    sim.store.enqueue_message(
        ids["A"], ids["B"], "track_report", sent_at=0, deliver_at=0
    )
    sim.initialize()
    sim.run_for(8_000_000)

    assert [m.kind for m in sim.store.peek_messages(ids["B"])] == ["track_report"]
    assert sim.comm.epoch == 0                 # 拓扑一次都没动
    assert sim.comm.statistics()["screened"] == 0   # 没有干扰机 ⇒ 一个节点都没求值
    assert sim.comm.statistics()["jammed"] == 0


# -- 5. 想定侧：干扰机 + 通信件端到端 --------------------------------------


_JAM_HEAD = """
zone THEATER
    anchor latlng 39.90 116.40
    radius 60 km
    resolution 200 m
    layers 3
end_zone

simulation
    max_step 2 s
end_simulation

platform_type COMMS_NODE
    component comm0 RF_COMM
        period 2 s
        power 10 W
        antenna_gain_db 6
        frequency 4 GHz
        bandwidth 2 MHz
        noise_power 1e-11 W
        required_snr_db 40
    end_component
end_platform_type

platform_type JAMMER_PLATFORM
    component jammer1 RF_JAMMER
        peak_power 100 kW
        frequency 4 GHz
        antenna_gain_db 30
    end_component
end_platform_type
"""


def _jam_scenario(body: str, *, jammer_frequency: str = "4 GHz") -> str:
    """把干扰机的频率换成 ``jammer_frequency``（默认与通信件同频 ⇒ 进带）。

    ★ 替换必须发生在 ``_JAM_HEAD`` 上（型号块在那里），不是在平台文本上——
    平台块里没有 ``frequency``，改它等于什么都没改，而症状是"这条用例永远
    给出同一个结论"。
    """
    return _JAM_HEAD.replace(
        "        peak_power 100 kW\n        frequency 4 GHz",
        f"        peak_power 100 kW\n        frequency {jammer_frequency}",
    ) + body


def _assemble_jam(body: str, *, jammer_frequency: str = "4 GHz"):
    from milsim import models
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    spec = load_scenario(_jam_scenario(body, jammer_frequency=jammer_frequency))
    registry = ComponentRegistry()
    models.register_framework(components=registry)
    sim = Simulation(spec, components=registry)
    sim.build()
    return sim


#: ★ 干扰机**不能与受害方同一个 ``latlng``**：``_eligible`` 有一道"两台机不
#: 可以同处一点"的筛，同点会被跳过（``J ∝ 1/R²`` 在 R=0 处发散，跳过比抛异常
#: 合适）。所以这里把 J 放在 A 的东南约 900 m 处。
#:
#: ★ 另一个坑：``latlng`` 会被**吸附到 h3 格心**（200 m 分辨率的格），
#: ``39.9505 116.3505`` 与 ``39.95 116.35`` 会吸到**同一个点**。要错开就得
#: 差到千米量级，不能只差小数点后四位。
#:
#: ★ 槽名写 ``jammer1`` **不是** ``jammer0``：天线条数读的是槽名尾部的裸数字
#: （``_antenna_count``），而 ``jammer0`` 会给出 **0 条天线**（一个真实的
#: 边界缺陷，已记 §9-63，本版不在这里顺手改）。
_BOTH_PLATFORMS = """
platform A COMMS_NODE
    position latlng 39.95 116.35
end_platform
platform B COMMS_NODE
    position latlng 39.99 116.40
end_platform
platform J JAMMER_PLATFORM
    position latlng 39.95 116.36
    side RED
end_platform

network BLUE_NET
    member A B
end_network
"""


def test_a_jammer_silences_a_comms_net_in_a_scenario() -> None:
    """端到端：想定里挂一台干扰机、**压在 A 附近** ⇒ 通信节点被压住、
    报文投不出去。

    ★ 门限 ``required_snr_db 40`` 是"我这部收信机要求多高的信噪比"：
    **值越高越容易掉线**（要求高 = 稍微被压就不达标）。这个方向的直觉容易反，
    所以夹具用真实量级（10 W 发射 / 1e-11 W 噪声 ⇒ 名义 S/N 约 132 dB）配
    约 900 m 处的干扰机（100 kW / 30 dBi）把判据钉住。

    ★ 压制**逐节点**发生：A 被压不意味着 B 也被压（B 离干扰机 5 km 以上）。
    这条顺便确认了"不是一刀切"——``jammed == 1`` 而不是 2。
    """
    sim = _assemble_jam(_BOTH_PLATFORMS)
    ids = {name: sim.registry.by_name(name).entity_id for name in "ABJ"}
    sim.store.enqueue_message(
        ids["A"], ids["B"], "track_report", sent_at=0, deliver_at=0
    )
    sim.initialize()
    assert sim.jam.is_empty is False
    sim.run_for(4_000_000)

    assert sim.comm.statistics()["jammed"] == 1        # 只有 A 被压
    assert len(sim.store.inflight_messages()) == 1     # 不通 ⇒ 留在在途
    assert sim.store.peek_messages(ids["B"]) == []


def test_an_off_band_jammer_platform_changes_nothing() -> None:
    """端到端：干扰机挂在**另一个频段** ⇒ 通信一切照旧（``F_BW = 0``）。

    这条与上一条是一对：同样是"想定里挂了一台干扰机"，只有频率不同，结论必须
    相反。若 ``F_BW`` 没有真的进账（比如照抄时漏乘），两条会给出同一个结论。
    """
    sim = _assemble_jam(_BOTH_PLATFORMS, jammer_frequency="40 GHz")
    ids = {name: sim.registry.by_name(name).entity_id for name in "AB"}
    sim.store.enqueue_message(
        ids["A"], ids["B"], "track_report", sent_at=0, deliver_at=0
    )
    sim.initialize()
    sim.run_for(4_000_000)

    assert sim.comm.statistics()["screened"] > 0       # 求值过了
    assert sim.comm.statistics()["jammed"] == 0        # 但一次都没压住
    assert [m.kind for m in sim.store.peek_messages(ids["B"])] == ["track_report"]


# -- 6. 端到端四段链路：探测 → 通信传航迹 → 引导干扰 ------------------------


#: ★ 这条链**跨三个部件**（感知 / 通信 / 电子战），此前从没被端到端跑过：
#: ``share_recipients`` 与 ``network`` 的交集为零，转发用例全部手写
#: ``store.deliver_due()`` 绕过通信件。下面这两条用例就是把它串起来。
#:
#: 四段链路各自的可观察量：
#:
#: .. code-block:: text
#:
#:     ① 探测    SCOUT 的雷达建航迹     ⇒ store.contacts_of(SCOUT) 非空
#:     ② 传航迹  TRACK_MANAGER 走 share_recipients 写**在途队列**，
#:               RF_COMM 按拍搬进 JAMMER 的收件箱  ⇒ comm.delivered > 0
#:     ③ 置向    JAMMER 的 TRACK_MANAGER absorb ⇒ 它表里有敌航迹
#:               ⇒ RF_JAMMER 的 aim_at nearest 挑得到  ⇒ jam.aimed > 0
#:     ④ 压制    受害方算 S/(N+J) 不过门限 ⇒ 从拓扑摘掉
#:
#: ★ 干扰机**自己不带雷达** —— 它那张航迹表里的敌航迹**只能**从通信件来。
#:   这正是本用例要演的事：把"航迹怎么到它手上"从"自带探测"里摘出来。
#:   若哪天有人给干扰机顺手加了雷达，这条用例仍会过，但**证明力没了**；
#:   所以下面显式断言"干扰机自己没有传感器件"。
_TRACK_CHAIN = """
zone THEATER
    anchor latlng 39.90 116.40
    radius 60 km
    resolution 200 m
    layers 3
end_zone

simulation
    max_step 2 s
    seed 20260929
end_simulation

platform_type SCOUT_NODE
    side blue
    component sensor HEX_SEARCH_RADAR
    end_component
    component track TRACK_MANAGER
        share_interval   10 s
        share_recipients [JAMMER_1]
    end_component
    component comm RF_COMM
    end_component
end_platform_type

platform_type JAMMER_NODE
    side blue
    component track TRACK_MANAGER
    end_component
    component jammer RF_JAMMER
        peak_power       100 kW
        frequency        4 GHz
        antenna_gain_db  30
        aim_at           nearest
    end_component
    component comm RF_COMM
    end_component
end_platform_type

platform_type RED_TARGET_NODE
    side red
end_platform_type

platform_type RED_RADAR_NODE
    side red
    component sensor HEX_SEARCH_RADAR
        peak_power      50 MW
        antenna_gain_db 6
        frequency       4 GHz
        bandwidth       2 MHz
        required_pd     0.5
    end_component
end_platform_type

network LINK_NET
    member SCOUT_1 JAMMER_1
end_network

platform SCOUT_1 SCOUT_NODE
    position latlng 39.8600 116.3600
    heading  90 deg
end_platform
platform JAMMER_1 JAMMER_NODE
    position latlng 39.8700 116.4600
    heading  90 deg
end_platform
platform RED_TGT_1 RED_TARGET_NODE
    position latlng 39.9000 116.4100
    heading  90 deg
end_platform
platform RED_RADAR_1 RED_RADAR_NODE
    position latlng 39.9000 116.5100
    heading  90 deg
end_platform
"""


def _assemble_track_chain():
    """装配四段链路的想定（与 ``scenarios/comm_track_jam.txt`` 同构）。"""
    from milsim import models
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    spec = load_scenario(_TRACK_CHAIN)
    registry = ComponentRegistry()
    models.register_framework(components=registry)
    sim = Simulation(spec, components=registry)
    sim.build()
    return sim


def test_a_track_travels_by_radio_and_aims_the_jammer() -> None:
    """**正向**：探测 → 通信传航迹 → 干扰机置向 → 压制，四段全通。

    每一段都单独断言，这样"链在哪一段断的"能从失败信息里直接读出来：
    只断言最后 ``aimed > 0`` 的话，② 和 ③ 同时坏掉也只报一个症状。
    """
    sim = _assemble_track_chain()
    ids = {name: sim.registry.by_name(name).entity_id for name in
           ("SCOUT_1", "JAMMER_1", "RED_TGT_1", "RED_RADAR_1")}
    sim.initialize()

    # ★ 前提：干扰机自己没有传感器 ⇒ 它的航迹表只能靠通信填。这条不成立
    #   的话，"航迹从哪来"这件事就没被演到。
    jammer_entity = sim.entities[ids["JAMMER_1"]]
    assert not [p for p in jammer_entity.parts() if hasattr(p, "last_pd")]

    sim.run_for(60_000_000)

    # ① 探测：侦察机的表里有敌航迹。
    assert sim.store.contacts_of(ids["SCOUT_1"])

    # ② 传航迹：通信件真投递过（不是留在在途队列里干等）。
    assert sim.comm.delivered > 0

    # ③ 置向：干扰机挑中过敌航迹 ⇒ 它拿到了本机探测不到的东西。
    assert sim.jam.aimed > 0
    # 而它自己那张表确实非空（③ 的机制来源）。
    assert sim.store.contacts_of(ids["JAMMER_1"])

    # ④ 压制：受害雷达被压过（J 进得来 ⇒ jamming_w > 0）。
    radar = next(
        p for p in sim.entities[ids["RED_RADAR_1"]].parts() if hasattr(p, "jamming_w")
    )
    assert radar.jamming_w > 0.0


def test_cutting_the_comms_link_stops_everything_downstream() -> None:
    """**反向**：中途把侦察机摘下线 ⇒ ``commit`` 之后 ②③④ 全不再推进。

    ★ 这条是正向用例的守门人。它盯住的错是"链路断了后果却照旧" ——
    例如 ``deliver`` 不看 ``commit`` 的结果就直接投递，或 ``aimed`` 用的是
    缓存的航迹表。两者的症状都是"干扰照打"，只有把链路拉断才能看出来。

    ★ 反向用例**只在 milsim 侧做**（用户裁定）：AFSIM 的
    ``jamming_perception_threshold`` 是"感知到干扰"而不是"链路断开"，
    没有等价机制可对拍。

    ★ 顺序要点：``mark_offline`` 只写意图 → ``jammed_now`` （本用例不调，
    我们手工摘）→ ``commit`` 才改拓扑。所以直接摘点是合法的**外部**操作，
    下一拍生效（§5.15「掉线 ≠ 注销」）。

    ★ 断言的是"**没有新航迹进来**"，而**不是**"干扰机不再挑目标"——
    链路断了不代表手里那条旧航迹会消失（真实系统里"通信中断了，我手里
    那张图还能打一会儿"）。第一版就是这么写错的：``aimed`` 继续涨到 165，
    而那是**正确行为**。所以判据落在 ② ``delivered`` 停住、外加干扰机
    表**条数**不涨。
    """
    sim = _assemble_track_chain()
    ids = {name: sim.registry.by_name(name).entity_id for name in
           ("SCOUT_1", "JAMMER_1", "RED_RADAR_1")}
    sim.initialize()
    sim.run_for(40_000_000)

    # 先确认链本来是通的 —— 不然下面的断言可能"因为本来就没通"而假过。
    assert sim.comm.delivered > 0
    assert sim.jam.aimed > 0

    # 摘掉侦察机（发信方）⇒ 它是唯一的航迹来源。
    sim.comm.mark_offline(ids["SCOUT_1"])
    # 走完三阶段：标记 → 提交 → 传信。手工调 commit 是为了让"这一拍生效"
    # 这条口径显式出现在用例里，而不是依赖引擎内部的调用时机。
    sim.comm.commit()
    assert sim.comm.epoch > 0                    # 拓扑真的变了

    delivered_before = sim.comm.delivered
    tracks_before = len(sim.store.contacts_of(ids["JAMMER_1"]))
    sim.run_for(40_000_000)

    # ② 不再有新报文送达（链路断在通信件这一层）。
    assert sim.comm.delivered == delivered_before
    # ③ 干扰机的表里也没有**新**航迹进来（旧的那条仍在，故用"不涨"）。
    assert len(sim.store.contacts_of(ids["JAMMER_1"])) == tracks_before
    # ④ 但旧的照打 —— 这不是 bug，是"手里的图还能用"。
    assert sim.jam.aimed > 0

