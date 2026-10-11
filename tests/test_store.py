"""EntityStore 的测试。

重点在三处容易出错的地方：

1. **幽灵实体**——注销时漏摘某个索引。表现为"雷达能探测到不存在的目标"，
   是本模块最容易出的 bug，密集生灭测试专门守它。
2. **视图权限**——不是"约定不能调"，而是方法根本不存在。用 ``hasattr`` 锁死。
3. **位置副本**——返回数组视图的话，视图权限就白设了（拿到句柄就能改）。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from milsim.services.map import Axial
from milsim.services.store import (
    DAMAGE_DESTROYED,
    DAMAGE_INTACT,
    MAX_RELAY_HOPS,
    ORIGIN_LOCAL,
    TRACK_REPORT,
    Contact,
    Effect,
    EngagementView,
    EntityStore,
    GlobalCellRef,
    KinematicsTable,
    LocalCellRef,
    Message,
)


def ref(col: int, row: int, zone_id: int = 1, layer: int = 0) -> LocalCellRef:
    return LocalCellRef(zone_id, layer, Axial(col, row))


@pytest.fixture()
def store() -> EntityStore:
    s = EntityStore(capacity=4)
    for eid in range(6):
        s.add(eid, ref(eid, 0))
    return s


# ---------------------------------------------------------------------------
# 航迹的来源标记与通信同化（B 项）
#
# 实体 0 = 观察者 A、1 = 观察者 B、2 = 目标 T。航迹按所有者私有，
# 所以"多渠道汇聚"这件事只有走通信才发生。
# ---------------------------------------------------------------------------

_A, _B, _T = 0, 1, 2


def _local_track(store: EntityStore, owner: int, *, at: int = 1_000) -> Contact:
    """给 ``owner`` 造一条**本机量测**的航迹。"""
    return store.sensor_view(owner).add_contact(
        _T, quality=0.8, detected_at=at, bearing_deg=90.0, range_m=5_000.0,
        x=5_000.0, y=0.0, z=0.0,
    )


def test_local_contacts_are_marked_as_local(store: EntityStore) -> None:
    """传感器写的航迹自带来源标记：``ORIGIN_LOCAL`` + 0 跳。

    没有这条标记，"这条情报是**我测的**还是别人转述的"就分不开——
    而这两种情报的可信度、新鲜度含义完全不同。
    """
    contact = _local_track(store, _A)
    assert contact.origin_id == ORIGIN_LOCAL
    assert contact.hops == 0
    assert contact.is_local


def test_relayed_report_keeps_the_original_origin_and_counts_a_hop(
    store: EntityStore,
) -> None:
    """A 测到 → 发给 B → B 同化：**原始量测者仍是 A**，跳数 +1。

    若转发方把自己写成来源，情报链就断了——"这目标是谁先看到的"从此查
    不出来，而防回环也一并失效（下一跳就认不出这是自己的报告）。
    """
    _local_track(store, _A, at=2_000)
    sent = store.track_view(_A).share_tracks(_B, sent_at=2_000, deliver_at=3_000)
    assert len(sent) == 1
    assert store.peek_messages(_B) == []          # 进的是在途队列，不是收件箱

    store.deliver_due(3_000)
    delivery = store.take_messages(_B)
    assert [m.kind for m in delivery] == [TRACK_REPORT]
    assert delivery[0].sender_id == _A

    contact, verdict = store.track_view(_B).relay_contact(
        delivery[0].payload, sender_id=_A, received_at=3_000
    )
    assert verdict == "accepted"
    assert contact is not None
    assert contact.origin_id == _A                    # ★ 不是 _B
    assert contact.hops == 1
    assert not contact.is_local
    assert [c.target_id for c in store.contacts_of(_B)] == [_T]
    assert store.contacts_of(_A)[0].is_local          # A 自己那条没被碰


def test_circular_report_is_rejected(store: EntityStore) -> None:
    """★ 守门人：绕一圈回到自己手里的报告必须拒收。

    ``circular_report_rejection``。AFSIM 默认 **off**，我们默认 **on**——
    因为我们的通信件还没有路由与订阅，一条报告会投给全部同方单位，
    "自己发出去的那份必然绕回来"。默认 off 的后果不是多一条航迹（同一个
    目标本来就覆盖更新），而是**来源标记被洗成"转发"**：本机量测这个口径
    静静消失，而表里看着完全正常。

    判据要盯住"标记有没有被洗掉"，不能只盯"航迹条数没变"。
    """
    _local_track(store, _A, at=2_000)
    store.track_view(_A).share_tracks(_B, sent_at=2_000, deliver_at=3_000)
    store.deliver_due(3_000)
    relayed, _ = store.track_view(_B).relay_contact(
        store.take_messages(_B)[0].payload, sender_id=_A, received_at=3_000
    )
    assert relayed is not None

    # B 把它转回给 A —— 这条航迹本来就是 A 测的
    store.track_view(_B).share_tracks(_A, sent_at=4_000, deliver_at=4_000)
    store.deliver_due(4_000)
    back = store.take_messages(_A)[0]
    contact, verdict = store.track_view(_A).relay_contact(
        back.payload, sender_id=_B, received_at=4_000
    )
    assert verdict == "circular"
    assert contact is None

    mine = store.contacts_of(_A)[0]
    assert mine.is_local and mine.hops == 0           # ★ 标记没被洗掉
    assert store.statistics()["relay_rejected_circular"] == 1


def test_relay_hops_are_capped(store: EntityStore) -> None:
    """跳数到顶就不再往下一站走。

    上限是**防护常量不是物理量**。没有它，一次三角转发就能让航迹表按指数
    涨——而表涨了不会报错，只是内存慢慢爬。
    """
    far = Contact(target_id=_T, detected_at=9_000, x=1.0, y=2.0, z=0.0,
                  origin_id=5, hops=3)
    contact, verdict = store.track_view(3).relay_contact(
        far.as_dict(sender_id=4), sender_id=4, received_at=9_000,
        max_hops=MAX_RELAY_HOPS,
    )
    assert verdict == "accepted"
    assert contact is not None and contact.hops == MAX_RELAY_HOPS

    # 拿着满跳的那条再转一手 ⇒ hops + 1 > 上限
    again, verdict = store.track_view(4).relay_contact(
        contact.as_dict(sender_id=3), sender_id=3, received_at=9_500
    )
    assert verdict == "hops"
    assert again is None
    assert store.statistics()["relay_rejected_hops"] == 1


def test_stale_report_does_not_overwrite_a_fresher_track(store: EntityStore) -> None:
    """旧报告不能盖掉更新的航迹——那是把情报往回倒。

    AFSIM 的 ``fusion_method`` 默认 ``replacement``，"报告到达即替换"在
    实现上就是"**新者胜**"：只有 ``detected_at`` 更新的才接手。
    """
    _local_track(store, _A, at=5_000)
    stale = Contact(target_id=_T, detected_at=4_000, x=-9.0, y=-9.0, z=0.0,
                    origin_id=_B)
    contact, verdict = store.track_view(_A).relay_contact(
        stale.as_dict(sender_id=_B), sender_id=_B, received_at=5_100
    )
    assert verdict == "stale"
    assert contact is not None
    assert (contact.x, contact.y) == (5_000.0, 0.0)      # 还是本机那条

    fresher = Contact(target_id=_T, detected_at=6_000, x=-9.0, y=-9.0, z=0.0,
                      origin_id=_B, hops=1)
    contact, verdict = store.track_view(_A).relay_contact(
        fresher.as_dict(sender_id=_B), sender_id=_B, received_at=6_100
    )
    assert verdict == "accepted"
    assert contact is not None
    assert (contact.x, contact.y) == (-9.0, -9.0)
    assert contact.origin_id == _B
    assert contact.hops == 2                              # 在来者的 1 跳上再 +1


def test_report_format_round_trips_and_refuses_a_broken_payload() -> None:
    """线格式只有一处定义，且**缺字段 / 多字段 / 带哨兵都报错**。

    "缺字段补默认值"的后果是"报文少了一个 ``x``，航迹就落到原点"——
    错得离谱而日志全绿。格式分叉同理：接收端猜不出来就该炸。
    """
    contact = Contact(target_id=7, quality=0.25, detected_at=1_234, bearing_deg=45.0,
                      range_m=3_210.0, x=1.0, y=2.0, z=3.0, origin_id=4, hops=2)
    assert Contact.from_dict(contact.as_dict(sender_id=9)) == contact

    short = contact.as_dict(sender_id=9)
    del short["x"]
    with pytest.raises(KeyError, match="缺字段"):
        Contact.from_dict(short)

    fat = contact.as_dict(sender_id=9)
    fat["altitude"] = 1.0
    with pytest.raises(KeyError, match="未知字段"):
        Contact.from_dict(fat)


def test_local_origin_is_resolved_before_the_track_goes_on_the_wire() -> None:
    """★ 守门人：``ORIGIN_LOCAL`` 是**本地编码**，上线前必须落成具体实体。

    原样发 ``-1`` 出去，接收端会读成"发送方的传感器测的"——转发方的名字
    顶替了真正的原始观察者。**症状是来源被静默顶替**：航迹条数、位置、
    质量全对，只有"这情报谁先看到的"错了。这条是本轮测试先抓出来的
    （报告从 A 发出，B 收到后 ``origin_id`` 是 ``-1`` 而不是 ``A``）。
    """
    local = Contact(target_id=7, detected_at=1_000, x=1.0, y=2.0, z=0.0)
    assert local.origin_id == ORIGIN_LOCAL
    assert local.as_dict(sender_id=42)["origin_id"] == 42

    on_wire = local.as_dict(sender_id=42)
    on_wire["origin_id"] = ORIGIN_LOCAL
    with pytest.raises(ValueError, match="ORIGIN_LOCAL"):
        Contact.from_dict(on_wire)


def test_track_view_cannot_see_the_truth(store: EntityStore) -> None:
    """航迹管理器**没有**真实位置可用——权限是"方法不存在"，不是"约定别调"。

    给它 ``position_of``，"拿真值刷航迹"就变成一个顺手能写下的操作，
    而那正是本项目最想堵死的上帝视角泄漏。
    """
    track = store.track_view(_A)
    for forbidden in ("position_of", "bearing_to", "add_contact"):
        assert not hasattr(track, forbidden), forbidden


# ---------------------------------------------------------------------------
# 登记与注销
# ---------------------------------------------------------------------------

def test_add_initializes_state(store: EntityStore) -> None:
    assert len(store) == 6
    assert store.position_of(0) == (0.0, 0.0, 0.0)
    assert store.damage_of(0) == DAMAGE_INTACT
    assert store.is_alive(0)
    assert store.cell_of(3) == ref(3, 0)


def test_add_grows_capacity_beyond_initial(store: EntityStore) -> None:
    """容量 4 的 store 装了 6 个实体，数组必须已经扩容。"""
    assert store._kinematics.capacity >= 6
    for eid in range(6):
        assert store.has(eid)


def test_add_is_idempotent_and_keeps_damage(store: EntityStore) -> None:
    """重复登记不能把已受损的实体还原成完好。

    装配阶段为了少写去重逻辑会重复 add，如果每次 add 都重置状态，
    想定里预置的损伤就会被无声抹掉。
    """
    store.apply_damage(2, 0.4)
    store.add(2, ref(2, 0))                       # 重复登记
    assert store.damage_of(2) == pytest.approx(0.6)
    assert len(store) == 6


def test_remove_clears_every_domain(store: EntityStore) -> None:
    store.add_contact(3, target_id=4)
    store.post_message(3, 4, "ATTACK")
    store.set_loadout(3, "missile", 4.0)

    assert store.remove(3)
    assert not store.has(3)
    assert store.position_of(3) is None
    assert store.cell_of(3) is None
    assert store.contacts_of(3) == []
    assert store.peek_messages(3) == []
    assert store.loadout_of(3) == {}
    assert store.validate() == []


def test_remove_unknown_returns_false(store: EntityStore) -> None:
    assert store.remove(999) is False


def test_removed_entity_disappears_from_spatial_query(store: EntityStore) -> None:
    """注销必须同步空间索引，否则雷达还能搜到它。"""
    assert 4 in store.entities_at(ref(4, 0))
    store.remove(4)
    assert store.entities_at(ref(4, 0)) == []


def test_mass_create_destroy_keeps_store_consistent() -> None:
    """密集生灭后自检仍干净。幽灵实体就是这个场景下漏出来的。"""
    store = EntityStore(capacity=8)
    for eid in range(100):
        store.add(eid, ref(eid % 7, eid % 5))
        store.add_contact(eid, target_id=(eid + 1) % 100)
        if eid % 2 == 0:
            store.remove(eid)
    assert store.validate() == []
    assert len(store) == 50


def test_validate_detects_ghost_entity(store: EntityStore) -> None:
    """自检必须能抓到幽灵实体——测试自检本身是否有效。"""
    store._cell[99] = ref(0, 0)                  # 手工制造一个幽灵
    problems = store.validate()
    assert any("幽灵" in p for p in problems)


def test_validate_detects_index_mismatch(store: EntityStore) -> None:
    """索引与格归属不一致也要能抓出来。"""
    # 把实体的格归属改了，但不动索引——模拟 set_cell 被绕过的场景
    store._cell[1] = ref(50, 50)
    problems = store.validate()
    assert any("缺实体" in p for p in problems)


# ---------------------------------------------------------------------------
# 位置与格归属
# ---------------------------------------------------------------------------

def test_position_returns_copy_not_view(store: EntityStore) -> None:
    """位置必须是副本。返回数组视图的话，视图权限形同虚设。"""
    for arr in (
        store._kinematics.x,
        store._kinematics.y,
        store._kinematics.z,
    ):
        assert isinstance(arr, np.ndarray)

    pos = store.position_of(1)
    assert pos == (0.0, 0.0, 0.0)

    # 内部数组改了，先前取出的元组不受影响
    store._kinematics.x[1] = 12345.0
    assert pos == (0.0, 0.0, 0.0)
    assert store.position_of(1)[0] == 12345.0


def test_pose_returns_all_five_fields(store: EntityStore) -> None:
    store.set_pose(2, 100.0, 200.0, 300.0, 45.0, 10.0)
    assert store.pose_of(2) == (100.0, 200.0, 300.0, 45.0, 10.0)


def test_set_cell_reports_cross_cell_move(store: EntityStore) -> None:
    """只有真正跨格才返回 True——调用方据此决定要不要刷新快照。"""
    assert store.set_cell(0, ref(0, 0)) is False      # 没动
    assert store.set_cell(0, ref(1, 0)) is True       # 跨格
    assert store.set_cell(0, ref(1, 0)) is False      # 又没动


def test_set_cell_moves_between_index_buckets(store: EntityStore) -> None:
    store.set_cell(0, ref(20, 20))
    assert store.entities_at(ref(0, 0)) == []
    assert store.entities_at(ref(20, 20)) == [0]
    assert store.validate() == []


def test_global_cell_is_indexed_separately(store: EntityStore) -> None:
    """全球层与局部层分开索引：两套编码的粒度差几个数量级。"""
    global_ref = GlobalCellRef(0x8928308280FFFFF)
    store.set_cell(5, global_ref)
    assert store.entities_at(global_ref) == [5]

    # 局部索引里不该再有它
    assert store.entities_at(ref(5, 0)) == []
    assert store.cell_of(5) == global_ref
    assert store.validate() == []


def test_local_ref_rejects_negative_zone() -> None:
    with pytest.raises(ValueError):
        LocalCellRef(-1, 0, Axial(0, 0))


def test_remove_global_cell_entity(store: EntityStore) -> None:
    store.set_cell(5, GlobalCellRef(0x1234))
    store.remove(5)
    assert store.entities_at(GlobalCellRef(0x1234)) == []
    assert store.validate() == []


# ---------------------------------------------------------------------------
# 空间查询
# ---------------------------------------------------------------------------

def test_entities_at_returns_ascending_ids() -> None:
    store = EntityStore()
    for eid in (7, 3, 5):
        store.add(eid, ref(0, 0))
    assert store.entities_at(ref(0, 0)) == [3, 5, 7]


def test_entities_in_disk_is_nearest_first() -> None:
    store = EntityStore()
    store.add(0, ref(0, 0))
    store.add(1, ref(3, 0))
    store.add(2, ref(1, 0))
    result = store.entities_in_disk(ref(0, 0), radius=5)
    assert result[0] == 0
    assert result.index(2) < result.index(1)      # 近的先出现


def test_entities_in_cone_only_returns_that_direction() -> None:
    """锥形搜索只返回指定方向扇区内的实体。"""
    store = EntityStore()
    store.add(0, ref(0, 0))                       # 原点
    store.add(1, ref(5, 0))                       # 正东
    store.add(2, ref(0, 5))                       # 另一个方向

    east = store.entities_in_cone(ref(0, 0), direction=0, max_steps=6)
    assert 1 in east
    assert 2 not in east


def test_query_through_sensor_view(store: EntityStore) -> None:
    view = store.sensor_view(0)
    assert view.my_cell() == ref(0, 0)
    assert 1 in view.query_cone(direction=0, max_steps=3)


def test_disk_query_on_global_layer_raises(store: EntityStore) -> None:
    """全球层不提供邻域查询——那需要 H3 的单元运算，不该由存储层猜。"""
    store.set_cell(0, GlobalCellRef(0x99))
    view = store.sensor_view(0)
    assert view.query_disk(radius=3) == []        # 不报错，但也没有候选


def test_query_sector_only_returns_that_arc() -> None:
    """按天线方向的扇区查询：正西的目标不该出现在"往东扫"的结果里。"""
    store = EntityStore()
    store.add(0, ref(0, 0))
    store.add(1, ref(6, 0))                       # 正东（90°）
    store.add(2, ref(-6, 0))                      # 正西（270°）

    view = store.sensor_view(0)
    # 自己那一格**无条件**收进结果（目标贴在脚下时方位角没有定义），
    # 排在"由近及远"的最前面；其余按弧筛。
    assert view.query_sector(80.0, 100.0, max_steps=8) == [0, 1]
    assert view.query_sector(260.0, 280.0, max_steps=8) == [0, 2]


def test_bearing_to_is_measured_from_true_positions() -> None:
    """方位角按**真实世界坐标**算，北 0°、顺时针；y 轴增大是往南。"""
    store = EntityStore()
    store.add(0, ref(0, 0))
    store.add(1, ref(0, 0))
    store.set_pose(1, 1_000.0, 0.0, 0.0)          # 正东
    assert store.sensor_view(0).bearing_to(1) == pytest.approx(90.0)

    store.set_pose(1, 0.0, -1_000.0, 0.0)         # y 减小 = 往北
    assert store.sensor_view(0).bearing_to(1) == pytest.approx(0.0)

    store.set_pose(1, 0.0, 1_000.0, 0.0)          # y 增大 = 往南
    assert store.sensor_view(0).bearing_to(1) == pytest.approx(180.0)


def test_bearing_to_has_no_answer_for_missing_or_coincident_targets() -> None:
    """目标不存在、或就在脚下，方位角没有意义——返回 ``None``，不抛异常。"""
    store = EntityStore()
    store.add(0, ref(0, 0))
    view = store.sensor_view(0)
    assert view.bearing_to(999) is None
    store.add(1, ref(0, 0))
    assert view.bearing_to(1) is None              # 两个都在原点


# ---------------------------------------------------------------------------
# 损耗：裁决器专属域
# ---------------------------------------------------------------------------

def test_apply_damage_accumulates(store: EntityStore) -> None:
    assert store.apply_damage(0, 0.3) == pytest.approx(0.7)
    assert store.apply_damage(0, 0.2) == pytest.approx(0.5)
    assert store.is_alive(0)


def test_damage_clamps_at_zero(store: EntityStore) -> None:
    assert store.apply_damage(0, 5.0) == DAMAGE_DESTROYED
    assert not store.is_alive(0)
    assert store.apply_damage(0, 0.5) == DAMAGE_DESTROYED   # 已死不会复活


def test_set_damage_clamps_both_ends(store: EntityStore) -> None:
    store.set_damage(0, 3.0)
    assert store.damage_of(0) == DAMAGE_INTACT
    store.set_damage(0, -1.0)
    assert store.damage_of(0) == DAMAGE_DESTROYED


def test_destroyed_entity_stays_in_store_but_not_alive(store: EntityStore) -> None:
    """摧毁 ≠ 注销。残骸还在场上（可供 BDA 侦察），只是不再存活。"""
    store.apply_damage(1, 1.0)
    assert store.has(1)
    assert not store.is_alive(1)
    assert 1 not in store.alive_ids()
    assert store.position_of(1) is not None


def test_damage_of_unknown_entity_is_destroyed(store: EntityStore) -> None:
    assert store.damage_of(999) == DAMAGE_DESTROYED
    assert not store.is_alive(999)


# ---------------------------------------------------------------------------
# 待裁决效果
# ---------------------------------------------------------------------------

def test_effect_is_intent_not_result(store: EntityStore) -> None:
    """提交效果**不**改变目标状态——扣血是裁决器的事。"""
    view = store.engagement_view(0)
    before = store.damage_of(1)
    view.request_effect(target_id=1, kind="MISSILE", magnitude=0.3)
    assert store.damage_of(1) == before
    assert len(store.pending_effects()) == 1


def test_effects_keep_submission_order(store: EntityStore) -> None:
    store.engagement_view(0).request_effect(1, "GUN", 0.1)
    store.engagement_view(2).request_effect(1, "MISSILE", 0.5)
    kinds = [e.kind for e in store.pending_effects()]
    assert kinds == ["GUN", "MISSILE"]


def test_clear_effects_returns_count(store: EntityStore) -> None:
    store.engagement_view(0).request_effect(1, "GUN", 0.1)
    assert store.clear_effects() == 1
    assert store.pending_effects() == []
    assert store.clear_effects() == 0


def test_removing_entity_drops_its_effects(store: EntityStore) -> None:
    """实体注销后，与它相关的效果一并丢弃——否则裁决器会去访问不存在的实体。"""
    store.engagement_view(0).request_effect(1, "GUN", 0.1)
    store.remove(1)
    assert store.pending_effects() == []
    assert store.validate() == []


# ---------------------------------------------------------------------------
# 视图权限
# ---------------------------------------------------------------------------

def test_engagement_view_has_no_way_to_change_damage() -> None:
    """不是"约定不能调"，是方法根本不存在。这条锁死视图权限的设计。"""
    view = EngagementView(EntityStore(), 0)
    for forbidden in ("set_damage", "apply_damage", "set_position", "set_pose"):
        assert not hasattr(view, forbidden)      # type: ignore[arg-type]


def test_sensor_view_can_only_write_its_own_contacts(store: EntityStore) -> None:
    store.sensor_view(0).add_contact(target_id=1, quality=0.9)
    assert [c.target_id for c in store.contacts_of(0)] == [1]
    assert store.contacts_of(1) == []            # 别人的航迹域没被碰


def test_sensor_view_cannot_move_anything(store: EntityStore) -> None:
    view = store.sensor_view(0)
    assert not hasattr(view, "set_pose")
    assert not hasattr(view, "advance")


def test_mover_view_writes_only_its_own_pose(store: EntityStore) -> None:
    before = store.position_of(1)
    store.mover_view(0).set_pose(10.0, 20.0, 0.0, 0.0, 5.0)
    assert store.position_of(0) == (10.0, 20.0, 0.0)
    assert store.position_of(1) == before


def test_mover_view_advance_without_locator_raises(store: EntityStore) -> None:
    """缺 locator 时推进必须报错。

    静默跳过会让位置变了、索引没变，实体在新位置"消失"，雷达再也搜不到。
    """
    with pytest.raises(RuntimeError, match="locator"):
        store.mover_view(0).advance(1.0)


def test_mover_view_advance_with_locator_syncs_index() -> None:
    store = EntityStore()
    store.add(0, ref(0, 0))
    # 航向 90° 是**正东**（0° 是正北），x 才该增大
    store.set_pose(0, 0.0, 0.0, 0.0, 90.0, 100.0)

    # locator：把世界坐标按 1000 m 一格折算成格归属。
    #
    # **不能用 ``int(x // 1000)``**：``//`` 是向下取整，而 cos(90°) 的浮点
    # 残差让 y 变成 -6e-15，`floor(-6e-18)` 是 **-1** 而不是 0——单位会凭空
    # 偏一格。框架里的 LocalFrame.from_world 用的是 cube_round，没有这个问题。
    def locator(x: float, y: float, z: float) -> LocalCellRef:
        return ref(math.floor(x / 1000.0 + 0.5), math.floor(y / 1000.0 + 0.5))

    view = store.mover_view(0, locator)
    view.set_pose(0.0, 0.0, 0.0, 90.0, 100.0)

    for _ in range(12):
        view.advance(1.0)                          # 每秒 100 m，12 秒到 1200 m

    assert store.position_of(0)[0] == pytest.approx(1200.0)
    assert store.position_of(0)[1] == pytest.approx(0.0, abs=1e-9)
    assert store.cell_of(0) == ref(1, 0)
    assert store.entities_at(ref(1, 0)) == [0]
    assert store.validate() == []


def test_comm_view_send_goes_to_inflight_not_inbox(store: EntityStore) -> None:
    """通信模型存在时，消息先到在途队列——直接进收件箱等于通信瞬时无损。"""
    store.comm_view(0).send(1, "ATTACK", sent_at=0, deliver_at=5000)
    assert store.peek_messages(1) == []
    assert len(store.inflight_messages()) == 1


# ---------------------------------------------------------------------------
# 航迹
# ---------------------------------------------------------------------------

def test_add_contact_updates_instead_of_appending(store: EntityStore) -> None:
    """同一目标的重复探测必须更新已有航迹。

    追加的话，每帧探测一次，航迹表几秒内涨到几千条，九成是同一个目标。
    """
    store.add_contact(0, 1, quality=0.5)
    store.add_contact(0, 1, quality=0.9)
    contacts = store.contacts_of(0)
    assert len(contacts) == 1
    assert contacts[0].quality == pytest.approx(0.9)


def test_contacts_of_returns_isolated_list(store: EntityStore) -> None:
    """返回副本——调用方 clear 一下不该清空别人的数据域。"""
    store.add_contact(0, 1)
    snapshot = store.contacts_of(0)
    snapshot.clear()
    assert len(store.contacts_of(0)) == 1


def test_drop_contact(store: EntityStore) -> None:
    store.add_contact(0, 1)
    store.add_contact(0, 2)
    assert store.drop_contact(0, 1)
    assert [c.target_id for c in store.contacts_of(0)] == [2]
    assert not store.drop_contact(0, 99)


def test_add_contact_requires_owner_in_store(store: EntityStore) -> None:
    with pytest.raises(KeyError):
        store.add_contact(999, 1)


# ---------------------------------------------------------------------------
# 消息
# ---------------------------------------------------------------------------

def test_direct_message_lands_in_inbox(store: EntityStore) -> None:
    """无通信建模的交互（载具内部）直接投收件箱。"""
    store.post_message(0, 1, "RELOAD")
    assert len(store.peek_messages(1)) == 1


def test_deliver_due_moves_inflight_to_inbox(store: EntityStore) -> None:
    store.enqueue_message(0, 1, "ATTACK", sent_at=0, deliver_at=5_000_000)
    assert store.deliver_due(4_999_999) == []
    delivered = store.deliver_due(5_000_000)
    assert [m.kind for m in delivered] == ["ATTACK"]
    assert len(store.peek_messages(1)) == 1
    assert store.inflight_messages() == []


def test_deliver_due_preserves_order(store: EntityStore) -> None:
    """同一发送方连发的两条不能因为排序而颠倒。

    颠倒会让"先侦察后开火"变成"先开火后侦察"——命令语义直接反了。
    """
    for kind in ("SCOUT", "FIRE"):
        store.enqueue_message(0, 1, kind, deliver_at=1000)
    store.deliver_due(1000)
    assert [m.kind for m in store.peek_messages(1)] == ["SCOUT", "FIRE"]


def test_take_messages_empties_inbox(store: EntityStore) -> None:
    """取走而不是读——留着会让实体每帧重复执行同一条命令。"""
    store.post_message(0, 1, "ATTACK")
    assert len(store.take_messages(1)) == 1
    assert store.take_messages(1) == []


def test_next_delivery_is_earliest(store: EntityStore) -> None:
    store.enqueue_message(0, 1, "A", deliver_at=9000)
    store.enqueue_message(0, 1, "B", deliver_at=3000)
    assert store.next_delivery() == 3000


def test_next_delivery_none_when_empty(store: EntityStore) -> None:
    assert store.next_delivery() is None


def test_message_ids_are_unique_and_increasing(store: EntityStore) -> None:
    ids = [store.post_message(0, 1, "X").msg_id for _ in range(5)]
    assert ids == sorted(set(ids))


def test_removing_recipient_drops_inflight_messages(store: EntityStore) -> None:
    store.enqueue_message(0, 1, "ATTACK", deliver_at=9999)
    store.remove(1)
    assert store.inflight_messages() == []
    assert store.validate() == []


def test_payload_is_copied(store: EntityStore) -> None:
    """payload 必须拷贝一份，否则发送方事后改 dict 会改到自己发的消息。"""
    payload = {"target": 7}
    store.post_message(0, 1, "ATTACK", payload=payload)
    payload["target"] = 99
    assert store.peek_messages(1)[0].payload["target"] == 7


# ---------------------------------------------------------------------------
# 物资
# ---------------------------------------------------------------------------

def test_loadout_readable_by_others_writable_only_by_self(store: EntityStore) -> None:
    store.engagement_view(0).set_my_loadout("missile", 4.0)
    assert store.loadout_of(0) == {"missile": 4.0}
    assert store.loadout_of(1) == {}

    # 交战模型能读别人的物资，但视图里没有写别人物资的方法
    view = store.engagement_view(1)
    assert not hasattr(view, "set_loadout")


# ---------------------------------------------------------------------------
# 批量运算：数据并行的入口
# ---------------------------------------------------------------------------

def test_advance_all_matches_per_entity_advance() -> None:
    """批量推进的结果必须与逐个推进完全一致，否则并行化就没意义。"""
    batch = KinematicsTable(capacity=8)
    single = KinematicsTable(capacity=8)

    for eid in range(5):
        for table in (batch, single):
            table.heading_deg[eid] = eid * 37.0
            table.speed_mps[eid] = 10.0 + eid

    batch.advance_all(5, 2.5)
    for eid in range(5):
        single.advance(eid, 2.5)

    assert np.allclose(batch.x[:5], single.x[:5])
    assert np.allclose(batch.y[:5], single.y[:5])


def test_advance_all_ignores_entities_beyond_count() -> None:
    table = KinematicsTable(capacity=8)
    table.speed_mps[7] = 100.0
    table.advance_all(3, 1.0)
    assert table.x[7] == 0.0


def test_advance_direction_matches_bearing_convention() -> None:
    """推进方向必须与方位角约定一致。

    **这条测试是为一个真实 bug 写的**：世界坐标的 y 轴指向**南**（图像
    坐标习惯），位移公式应该是 ``(d·sinθ, -d·cosθ)``。最初写成
    ``(d·cosθ, d·sinθ)``，结果所有单位整体偏了 90°——航向正东的单位在
    往南走。而 45° 与 225° 的位移恰好对称，"看起来在动"，很难一眼看出。

    与其逐个方位角写死期望值，不如**用反向换算交叉验证**：从位移反推
    方位角，必须与输入的航向吻合。这样两处几何只要有一处推歪就会被抓住。
    """
    from math import atan2, degrees

    for heading in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
        table = KinematicsTable(capacity=2)
        table.heading_deg[0] = heading
        table.speed_mps[0] = 1_000.0
        table.advance(0, 1.0)

        x, y = float(table.x[0]), float(table.y[0])
        # 反向：dx 是东向分量，-dy 是北向分量
        bearing = (degrees(atan2(x, -y)) + 360.0) % 360.0
        assert bearing == pytest.approx(heading, abs=1e-6), (
            f"航向 {heading}° 推进后反向算出 {bearing:.3f}°"
        )


def test_advance_cardinal_directions() -> None:
    """四个正方向的具体位移。留一组肉眼可核对的基准。"""
    cases = {
        0.0: (0.0, -100.0),      # 正北：y 减小（y 轴朝南）
        90.0: (100.0, 0.0),      # 正东
        180.0: (0.0, 100.0),     # 正南
        270.0: (-100.0, 0.0),    # 正西
    }
    for heading, (expected_x, expected_y) in cases.items():
        table = KinematicsTable(capacity=2)
        table.heading_deg[0] = heading
        table.speed_mps[0] = 100.0
        table.advance(0, 1.0)
        assert float(table.x[0]) == pytest.approx(expected_x, abs=1e-9), heading
        assert float(table.y[0]) == pytest.approx(expected_y, abs=1e-9), heading


def test_advance_all_uses_same_geometry_as_advance() -> None:
    """批量推进与逐个推进必须完全一致——公式只该有一处推导。"""
    batch = KinematicsTable(capacity=4)
    single = KinematicsTable(capacity=4)
    for eid, heading in enumerate((0.0, 90.0, 180.0, 270.0)):
        for table in (batch, single):
            table.heading_deg[eid] = heading
            table.speed_mps[eid] = 50.0

    batch.advance_all(4, 2.0)
    for eid in range(4):
        single.advance(eid, 2.0)

    assert np.allclose(batch.x[:4], single.x[:4])
    assert np.allclose(batch.y[:4], single.y[:4])
    assert float(batch.y[0]) < 0.0          # 航向 0° 是向北，y 减小


# ---------------------------------------------------------------------------
# 诊断与协议
# ---------------------------------------------------------------------------

def test_statistics_counts_each_domain(store: EntityStore) -> None:
    store.add_contact(0, 1)
    store.post_message(0, 1, "X")
    store.enqueue_message(0, 1, "Y")
    store.engagement_view(0).request_effect(1, "GUN", 0.1)
    store.apply_damage(1, 1.0)

    stats = store.statistics()
    assert stats["entities"] == 6
    assert stats["alive"] == 5
    assert stats["contacts"] == 1
    assert stats["queued_messages"] == 1
    assert stats["inflight_messages"] == 1
    assert stats["pending_effects"] == 1


def test_iteration_is_ascending(store: EntityStore) -> None:
    store.remove(3)
    assert list(store) == [0, 1, 2, 4, 5]


def test_contains_uses_active_set(store: EntityStore) -> None:
    assert 0 in store
    store.remove(0)
    assert 0 not in store


def test_clear_resets_everything(store: EntityStore) -> None:
    store.add_contact(0, 1)
    store.post_message(0, 1, "X")
    store.clear()
    assert len(store) == 0
    assert store.statistics()["contacts"] == 0
    assert store.validate() == []


def test_capacity_only_grows(store: EntityStore) -> None:
    """容量只增不减——退回让 ID → 下标不再一致，所有按 ID 索引的数组都要重排。"""
    before = store._kinematics.capacity
    store.remove(0)
    store.remove(1)
    assert store._kinematics.capacity == before


def test_dataclasses_are_plain_carriers() -> None:
    """Contact / Message / Effect 是纯数据，不带行为。"""
    contact = Contact(target_id=3, quality=0.8)
    message = Message(1, 2, 3, "X", 0, 0)
    effect = Effect(1, 2, "GUN", 0.5)
    assert (contact.target_id, message.kind, effect.magnitude) == (3, "X", 0.5)


# ---------------------------------------------------------------------------
# 集成：身份与状态的分工
# ---------------------------------------------------------------------------

class _Unit:
    """最小实体 stub——只满足注册表的协议。

    services 不 import models，实体是什么类型由模型层决定。这里用一个
    三行的类就能替代真实实体，正是依赖倒置想要的效果。
    """

    __slots__ = ("entity_id", "name")

    def __init__(self) -> None:
        self.entity_id = -1
        self.name = ""


def test_identity_and_state_are_stitched_by_id_only() -> None:
    """Registry 管身份、Store 管状态，**两者互不 import**，装配层用 ID 缝合。

    这条测试的价值在于它证明了分层是通的：注册表返回一个 int，
    存储接受一个 int，中间没有任何对象引用穿过层边界。
    """
    from milsim.services.registry import EntityRegistry

    registry = EntityRegistry()
    store = EntityStore()

    # -- 装配：注册身份，建立状态槽位 --
    def enlist(name: str, side: str, col: int, row: int) -> int:
        entity_id = registry.register(_Unit(), name=name, side=side)
        store.add(entity_id, ref(col, row))
        return entity_id

    radar = enlist("RADAR_1", "red", 0, 0)
    scout = enlist("SCOUT_1", "red", 2, 0)
    intruder = enlist("BANDIT_1", "blue", 6, 0)

    assert registry.by_side("red") == [radar, scout]
    assert registry.by_type("") == [radar, scout, intruder]

    # -- 感知：雷达向正东搜索，粗筛 → 精筛 --
    sensor = store.sensor_view(radar)
    candidates = sensor.query_cone(direction=0, max_steps=8)
    assert intruder in candidates

    # 精筛（真实实现里这里接通视与探测概率）
    sensor.add_contact(intruder, quality=0.7, detected_at=1000, bearing_deg=90.0)

    # 航迹只写进自己的域
    assert [c.target_id for c in store.contacts_of(radar)] == [intruder]
    assert store.contacts_of(scout) == []

    # -- 交战：写意图，不写结果 --
    store.engagement_view(radar).request_effect(intruder, "SAM", 0.6, at=2000)
    assert store.damage_of(intruder) == DAMAGE_INTACT     # 还没结算

    # -- 裁决器（M4 实现，这里先手工演示语义）--
    for effect in store.pending_effects():
        store.apply_damage(effect.target_id, effect.magnitude)
    assert store.clear_effects() == 1
    assert store.damage_of(intruder) == pytest.approx(0.4)

    # -- 通信：指挥所的指令先进在途，到点才进收件箱 --
    command = enlist("HQ_1", "red", -3, 0)
    store.comm_view(command).send(scout, "ADVANCE", sent_at=2000, deliver_at=8000)

    assert store.peek_messages(scout) == []                # 还没送到
    store.deliver_due(7999)
    assert store.peek_messages(scout) == []
    store.deliver_due(8000)
    assert [m.kind for m in store.take_messages(scout)] == ["ADVANCE"]
    assert store.take_messages(scout) == []                # 取走即清空

    # -- 全链路自检 --
    assert store.validate() == []

    # -- 摧毁 ≠ 注销：残骸留在场上供 BDA 侦察 --
    store.apply_damage(intruder, 1.0)
    assert not store.is_alive(intruder)
    assert store.position_of(intruder) is not None
    assert registry.has(intruder)

    # -- 注销：两边都要摘干净 --
    registry.unregister(intruder)
    store.remove(intruder)
    assert registry.by_side("blue") == []
    assert store.entities_at(ref(6, 0)) == []
    assert store.validate() == []
    assert registry.validate() == []


# ---------------------------------------------------------------------------
# 按精度融合（v0.13.56，§5.13.5）
#
# 前提都在量测里：σ（位置/速度）与速度估计随量测上路。双方都带 σ 才有
# "精度"可加权；旧航迹还要有速度才能把旧估计时间对齐到来者时刻——
# 缺任何一样就退化为覆盖更新（"replaced"，与 v0.13.55 逐字同一行为）。
# ---------------------------------------------------------------------------


def _sigma_track(
    store: EntityStore, owner: int, *, at: int, x: float,
    sigma: float = 100.0, sv: float = 50.0, vx: float = 50.0,
) -> tuple[Contact, str]:
    """给 owner 造一条**带精度与速度**的本机量测航迹（正东 x 米）。"""
    return store.sensor_view(owner).merge_contact(
        _T, quality=0.8, detected_at=at, bearing_deg=90.0, range_m=x,
        x=x, y=0.0, z=0.0,
        sigma_pos_m=sigma, sigma_vel_mps=sv, vx=vx, vy=0.0, vz=0.0,
    )


def test_fusion_weights_two_estimates_by_precision(store: EntityStore) -> None:
    """两个都带 σ 的估计 ⇒ 时间对齐后按 1/σ² 加权，σ 收缩。

    手算（Δt=2 s，旧速度 50 m/s，两支位置 σ 都是 100 m、σv 都是 50 m/s）：
    旧估计对齐到 1000+50·2 = 1100 m；σa² = 100²+(50·2)² = 20000，
    w_a = 1/20000、w_b = 1/10000 ⇒
    x = (w_a·1100 + w_b·1200)/(w_a+w_b) = 1166.67 m，
    σ = 1/√(w_a+w_b) = 81.65 m——**比任何一支都小**，这就是融合的收益。
    """
    _sigma_track(store, _A, at=1_000_000, x=1000.0)
    contact, verdict = store.sensor_view(_A).merge_contact(
        _T, quality=0.6, detected_at=3_000_000, bearing_deg=90.0,
        range_m=1200.0, x=1200.0, y=0.0, z=0.0,
        sigma_pos_m=100.0, sigma_vel_mps=50.0, vx=100.0, vy=0.0, vz=0.0,
    )
    assert verdict == "fused"
    assert contact.x == pytest.approx(1166.667, abs=0.01)
    assert contact.sigma_pos_m == pytest.approx(81.6497, abs=0.01)
    # 速度同样逆方差加权：(50+100)/2 = 75，σv = 1/√(2/50²) = 35.36
    assert contact.vx == pytest.approx(75.0)
    assert contact.sigma_vel_mps == pytest.approx(35.3553, abs=0.01)
    # quality 按同权重平均：(0.8·w_a + 0.6·w_b)/(w_a+w_b)
    assert contact.quality == pytest.approx(0.6667, abs=0.001)
    assert contact.detected_at == 3_000_000


def test_fusion_degrades_to_replacement_without_sigma(store: EntityStore) -> None:
    """任一侧 σ=0 ⇒ 覆盖更新。

    这是**兼容闸门**：没开误差模型的传感器写的航迹没有"精度"可言，
    没有精度就没有权重——整条融合链在第一道门槛就让位，行为与
    v0.13.55 逐字相同。
    """
    _sigma_track(store, _A, at=1_000_000, x=1000.0)
    contact, verdict = store.sensor_view(_A).merge_contact(
        _T, quality=0.6, detected_at=3_000_000, bearing_deg=90.0,
        range_m=1200.0, x=1200.0, y=0.0, z=0.0,      # 来者没给 σ
    )
    assert verdict == "replaced"
    assert contact.x == 1200.0
    assert contact.sigma_pos_m == 0.0


def test_fusion_needs_a_velocity_to_align(store: EntityStore) -> None:
    """旧航迹没有速度 ⇒ 不敢跨时刻平均，覆盖更新。

    动目标隔 2 s 就是 600 m 的系统性错位：没有速度估计就没有时间对齐
    的手段，硬平均不是融合是把航迹抹糊。这是刻意从严——速度缺失时
    "新者胜"永远不比"硬平均"差。
    """
    store.sensor_view(_A).merge_contact(
        _T, quality=0.8, detected_at=1_000_000, bearing_deg=90.0,
        range_m=1000.0, x=1000.0, y=0.0, z=0.0,
        sigma_pos_m=100.0, sigma_vel_mps=50.0, vx=0.0, vy=0.0, vz=0.0,
    )
    contact, verdict = store.sensor_view(_A).merge_contact(
        _T, quality=0.6, detected_at=3_000_000, bearing_deg=90.0,
        range_m=1200.0, x=1200.0, y=0.0, z=0.0,
        sigma_pos_m=100.0, sigma_vel_mps=50.0, vx=100.0, vy=0.0, vz=0.0,
    )
    assert verdict == "replaced"
    assert contact.x == 1200.0


def test_fusion_yields_when_alignment_is_poor(store: EntityStore) -> None:
    """速度不可信（σv 大）⇒ 旧估计权重趋于 0，融合温和地退化为"新者胜"。

    σv=1000 m/s、Δt=2 s ⇒ 对齐项 σv²Δt² = 4e6 压倒 σ² = 1e4，
    w_a ≈ 2.49e-7 对 w_b = 1e-4——融合位置离来者不足 1 m。没有这条
    自限，"融合"会把快目标的航迹拖回旧估计，比不融合更糟；有了它，
    融合与覆盖更新之间不需要任何硬窗口参数。
    """
    _sigma_track(store, _A, at=1_000_000, x=1000.0, sv=1000.0)
    contact, verdict = store.sensor_view(_A).merge_contact(
        _T, quality=0.6, detected_at=3_000_000, bearing_deg=90.0,
        range_m=1200.0, x=1200.0, y=0.0, z=0.0,
        sigma_pos_m=100.0, sigma_vel_mps=50.0, vx=100.0, vy=0.0, vz=0.0,
    )
    assert verdict == "fused"
    assert contact.x == pytest.approx(1200.0, abs=1.0)


def test_relay_fuses_a_precision_report(store: EntityStore) -> None:
    """A 的带 σ 报告到 B，与 B 自己的同目标航迹融合，结论是 "fused"。

    通信路径的融合与本地路径走同一个 merge_contact：报文带 σ 就加权，
    不带就覆盖——两条来路一个语义。跳数跟主贡献方：来者权重更大，
    融合后的 hops 是来者那一支的（0+1）。
    """
    _sigma_track(store, _B, at=1_000_000, x=1000.0)
    payload = Contact(
        target_id=_T, quality=0.6, detected_at=3_000_000,
        bearing_deg=90.0, range_m=1200.0, x=1200.0, y=0.0, z=0.0,
        origin_id=_A, hops=0,
        sigma_pos_m=100.0, sigma_vel_mps=50.0, vx=100.0,
    ).as_dict(sender_id=_A)
    contact, verdict = store.track_view(_B).relay_contact(
        payload, sender_id=_A, received_at=3_000_000
    )
    assert verdict == "fused"
    assert contact is not None
    assert contact.sigma_pos_m == pytest.approx(81.6497, abs=0.01)
    assert contact.hops == 1
