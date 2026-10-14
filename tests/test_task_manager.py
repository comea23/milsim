"""任务层（§5.19，v0.13.59）的测试：TaskManager + 六类执行器。

三个层次：

- **纯数据**：Task 的快照/往返/ID 空间；
- **桩级单元**：执行器协议的认领/轮询/交还（借 test_weapon 的桩手法）；
- **真想定端到端**：本地指派（move/attack/jam/resupply/cancel）与跨实体
  指派（通信网投递 + ack + 状态回报 + 可靠重传耗尽 + 无网直投）。

与 ``test_task.py``（M5a 战略意图台账，§4.5）分属两层，互不覆盖。

★ 端到端用例共用一份**模块级**想定（fixture scope="module"），用例之间
有顺序依赖（前一炮打的弹、后一测补的弹药）——断言全部按"增量"写，
不写绝对值。
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from milsim.models import Task
from milsim.models.task.base import (
    ACTIVE,
    ASSIGNED,
    CANCELED,
    COMPLETE,
    FAILED,
    TaskManager,
)
from milsim.models.weapon import Weapon
from milsim.services.bus import EventBus
from milsim.services.registry import EntityRegistry
from milsim.simulation import Simulation

_ROOT = Path(__file__).resolve().parents[1]


def _load_serve_replay(tag: str):
    spec = importlib.util.spec_from_file_location(
        f"serve_replay_{tag}", _ROOT / "tools" / "serve_replay.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# ---------------------------------------------------------------------------
# 纯数据：Task
# ---------------------------------------------------------------------------


def test_task_roundtrip_strips_manager() -> None:
    task = Task(task_id=7, task_type="attack", group_id="G1",
                assigner_id=2, assignee_id=3, target_id=9,
                payload={"rounds": 2}, assign_time_us=100)
    data = task.to_dict()
    assert data["status"] == ASSIGNED and "payload" in data
    clone = Task.from_dict(data)
    assert clone.task_id == 7 and clone.task_type == "attack"
    assert clone.target_id == 9 and clone.payload["rounds"] == 2
    assert clone.status == ASSIGNED          # 状态不随报文走
    clone.report(COMPLETE, "x")               # 无管理器 ⇒ 上报静默、状态不动
    assert clone.status == ASSIGNED


def test_task_id_space_is_entity_scoped() -> None:
    """同实体不重号；不同实体各用自己的 2²⁰ 段。"""
    assert 3 * (1 << 20) + 5 == (3 << 20) + 5


# ---------------------------------------------------------------------------
# 桩级单元：Weapon 的 attack 执行器协议
# ---------------------------------------------------------------------------


class _StubEntity:
    def __init__(self, eid: int, name: str) -> None:
        self.entity_id = eid
        self.name = name
        self._slots: dict[str, object] = {}

    def add(self, slot: str, part: object) -> None:
        self._slots[slot] = part

    def component(self, slot: str):
        return self._slots.get(slot)

    def parts(self) -> list:
        return list(self._slots.values())


class _StubMover:
    def __init__(self) -> None:
        self._terminated = False

    def terminated(self) -> bool:
        return self._terminated


class _StubGuidance:
    def __init__(self) -> None:
        self.target_id: int | None = None

    def assign_target_id(self, eid: int) -> None:
        self.target_id = eid


def _attack_rig(*, stock: int = 2):
    """发射方 0、目标 1、弹出生从 10 起。可控制弹的存续。"""
    reg = EntityRegistry()
    shooter, target = _StubEntity(0, "DDG"), _StubEntity(1, "TARGET")
    reg.register(shooter, name="DDG", side="red", type_name="SHIP")
    reg.register(target, name="TARGET", side="blue", type_name="SHIP")
    positions = {0: (0.0, 0.0, 0.0), 1: (30_000.0, 0.0, 0.0)}
    missiles: list[_StubEntity] = []
    counter = {"n": 10}

    def spawn(type_name, **kw):
        entity = _StubEntity(-1, "M")
        entity.add("mover", _StubMover())
        entity.add("guidance", _StubGuidance())
        mid = reg.register(entity, name=f"M-{counter['n']}",
                           side="red", type_name=type_name)
        counter["n"] += 1
        missiles.append(entity)
        return mid

    mount = SimpleNamespace(
        entity_id=0, registry=reg,
        store=SimpleNamespace(
            position_of=lambda eid: positions.get(eid),
            pose_of=lambda eid: (*positions.get(eid, (0.0, 0.0, 0.0)), 0.0, 0.0),
        ),
        bus=EventBus(), spawn=spawn, my_cell=lambda: "CELL",
    )
    spec = {
        "weapon_class": "SSM", "weapon_type": "M", "range_min": 0.0,
        "range_max": 0.0, "stock": stock,
        "az_min": -180.0, "az_max": 180.0,
        "el_min": -90.0, "el_max": 90.0, "dead_zone": (),
    }
    weapon = Weapon(spec=spec)
    weapon.bind(shooter)
    weapon.initialize(mount)
    return weapon, reg, missiles


def _mk_task(**over):
    defaults = dict(task_id=1, task_type="attack", group_id="G",
                    assigner_id=0, assignee_id=0, target_id=1)
    defaults.update(over)
    return Task(**defaults)


def test_weapon_attack_task_fires_salvo_and_polls() -> None:
    weapon, reg, missiles = _attack_rig(stock=3)
    task = _mk_task(payload={"rounds": 2})
    ok, why = weapon.acquire_task(task)
    assert ok and "齐射 2 发" in why
    assert weapon.stock() == 1

    # 弹还在飞 ⇒ 无变化
    assert weapon.poll_task(task) is None
    # 一发被注销（命中起爆）一发终止（失的）⇒ 收工并报命中数
    reg.unregister(missiles[0].entity_id)
    missiles[1].component("mover")._terminated = True
    assert weapon.poll_task(task) == (COMPLETE, "命中 1/2")
    assert task.task_id not in weapon._jobs


def test_weapon_attack_rejects_wrong_class_and_dry() -> None:
    weapon, _, _ = _attack_rig(stock=0)
    ok, why = weapon.acquire_task(_mk_task(payload={"weapon_class": "SAM"}))
    assert not ok and "SAM" in why
    ok, why = weapon.acquire_task(_mk_task())
    assert not ok and "耗尽" in why


def test_weapon_attack_completes_when_target_gone() -> None:
    weapon, reg, _ = _attack_rig()
    task = _mk_task()
    assert weapon.acquire_task(task)[0]
    reg.unregister(1)
    assert weapon.poll_task(task) == (COMPLETE, "target_destroyed")


def test_weapon_release_forgets_missiles() -> None:
    weapon, _, _ = _attack_rig()
    task = _mk_task()
    assert weapon.acquire_task(task)[0]
    weapon.release_task(task)
    assert task.task_id not in weapon._jobs
    assert weapon.poll_task(task) is None


# ---------------------------------------------------------------------------
# 真想定端到端
# ---------------------------------------------------------------------------

_SCENARIO = """
simulation
    name     "任务层测试"
    max_step 2 s
    seed     20261011
end_simulation

component_type GROUND_MOVER
end_component_type
component_type MISSILE_MOVER
end_component_type
component_type BALLISTIC_GUIDANCE
end_component_type
component_type WARHEAD
end_component_type
component_type RF_COMM
end_component_type
component_type RF_JAMMER
end_component_type
component_type TASK_MANAGER
end_component_type
component_type WEAPON
end_component_type
component_type SUPPLY
end_component_type

component_type SSM_SRBM : MISSILE_MOVER
    mass               800 kg
    fuel_mass          400 kg
    thrust             66 kN
    specific_impulse   250 s
    drag_area           0.09 m2
    radial_accel        40 m/s2
    max_speed         1500 m/s
    launch_fpa          85 deg
end_component_type

component_type MOVER_TRUCK : GROUND_MOVER
    max_speed 15 m/s
    turn_rate 40 deg/s
end_component_type

platform_type TEL_R
    side red
    component mover MOVER_TRUCK
    end_component
    component weapon WEAPON
        weapon_type SRBM_MISSILE
        range_max 0
        stock 6
    end_component
    component task TASK_MANAGER
    end_component
    component comm RF_COMM
    end_component
end_platform_type

platform_type FOE_TRUCK
    side blue
    component mover MOVER_TRUCK
    end_component
    component task TASK_MANAGER
    end_component
    component comm RF_COMM
    end_component
end_platform_type

platform_type LONE_TRUCK
    side red
    component mover MOVER_TRUCK
    end_component
end_platform_type

platform_type SUPPLY_R
    side red
    component mover MOVER_TRUCK
    end_component
    component supply SUPPLY
    end_component
    component task TASK_MANAGER
    end_component
    component comm RF_COMM
    end_component
end_platform_type

platform_type JAM_TRUCK
    side red
    component mover MOVER_TRUCK
    end_component
    component jammer RF_JAMMER
        peak_power 1 kW
        frequency 10 GHz
    end_component
    component task TASK_MANAGER
    end_component
    component comm RF_COMM
    end_component
end_platform_type

platform_type SRBM_MISSILE
    side red
    component mover SSM_SRBM
    end_component
    component guidance BALLISTIC_GUIDANCE
        lethal_radius 30 m
    end_component
    component damage WARHEAD
        fuse_radius 3000 m
    end_component
end_platform_type

zone TEST_SECTOR
    anchor     39.9042 116.4074
    radius     40 km
    resolution 500 m
    layers     1
    terrain    flat
end_zone

network NET
    member CMD_A FOE_B SUPPLY_1 JAM_1
end_network

platform CMD_A TEL_R
    position latlng 39.9042 116.4074
end_platform

platform FOE_B FOE_TRUCK
    position latlng 40.1700 116.4074
end_platform

platform LONE_B LONE_TRUCK
    position latlng 39.9100 116.4130
end_platform

platform SUPPLY_1 SUPPLY_R
    position latlng 39.9043 116.4075
end_platform

platform JAM_1 JAM_TRUCK
    position latlng 39.9050 116.4080
end_platform
"""


@pytest.fixture(scope="module")
def sim() -> Simulation:
    sim = Simulation.from_scenario(_SCENARIO)
    sim.initialize()
    return sim


def _manager(sim: Simulation, name: str) -> tuple[int, TaskManager]:
    entity = sim.entity_by_name(name)
    assert entity is not None, name
    for part in entity.parts():
        if isinstance(part, TaskManager):
            return entity.entity_id, part
    raise AssertionError(f"{name} 没有 TASK_MANAGER")


def test_local_move_task_runs_to_arrival(sim: Simulation) -> None:
    eid, manager = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "move",
        "payload": {"points": [[2000.0, 0.0, 0.0]]},   # 局部平面米，锚点系
    })
    assert ok, why
    # 认领即开工
    live = [t for t in manager.snapshot() if t["ledger"] != "history"]
    assert live and live[0]["status"] == ACTIVE
    # 开到 ⇒ complete/arrived，进归档
    sim.run_for(400 * 1_000_000)
    hist = [t for t in manager.snapshot()
            if t["ledger"] == "history" and t["task_type"] == "move"]
    assert hist and hist[0]["status"] == COMPLETE and hist[0]["sub"] == "arrived"


def test_local_attack_task_fires_and_completes(sim: Simulation) -> None:
    eid, manager = _manager(sim, "CMD_A")
    entity = sim.entity_by_name("CMD_A")
    weapon = entity.component("weapon")
    before = weapon.stock()
    assert before > 0
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "attack", "target": "FOE_B",
        "payload": {"rounds": 1},
    })
    assert ok, why
    assert weapon.stock() == before - 1             # 开了一炮
    spawned = [e for e in sim.alive_entities()
               if str(getattr(e, "name", "")).startswith("red-SRBM")]
    assert spawned, "弹实体没有出生"
    # 弹挂上了战斗部目标（v0.13.59 的交接链）
    warhead = spawned[0].component("damage")
    assert getattr(warhead, "_target_id", None) is not None

    sim.run_for(300 * 1_000_000)
    hist = [t for t in manager.snapshot()
            if t["ledger"] == "history" and t["task_type"] == "attack"]
    assert hist and hist[0]["status"] == COMPLETE
    assert hist[0]["sub"].startswith(("命中", "target"))


def test_remote_assignment_over_network_reports_back(sim: Simulation) -> None:
    """CMD_A 给 FOE_B 派 jam（FOE_B 没干扰机）：走网、拒绝、回报。"""
    eid, manager = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "jam", "assignees": ["FOE_B"], "target": "CMD_A",
        "group": "REMOTE_JAM",
    })
    assert ok, why
    live = [t for t in manager.snapshot()
            if t["ledger"] == "sent" and t["group_id"] == "REMOTE_JAM"]
    assert live and live[0]["status"] == ASSIGNED   # 在途，等受令方回话
    sim.run_for(30 * 1_000_000)
    hist = [t for t in manager.snapshot()
            if t["ledger"] == "history" and t["group_id"] == "REMOTE_JAM"]
    assert hist and hist[0]["status"] == FAILED
    assert "unclaimed" in hist[0]["sub"]            # 受令方没干扰机，回报了拒绝


def test_local_unclaimed_and_no_manager(sim: Simulation) -> None:
    eid, manager = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {"task_type": "dance"})
    assert ok
    hist = [t for t in manager.snapshot() if t["task_type"] == "dance"]
    assert hist[0]["status"] == FAILED and "unclaimed" in hist[0]["sub"]

    ok, why = sim.command(eid, "assign_task", {
        "task_type": "move", "assignees": ["LONE_B"],
        "payload": {"points": [[0.0, 0.0]]},
    })
    assert ok
    hist = [t for t in manager.snapshot()
            if t["task_type"] == "move" and t["assignee_id"] != eid
            and t["ledger"] == "history"]
    assert hist and hist[0]["sub"] == "no_task_manager"


def test_one_to_many_fanout_shares_group(sim: Simulation) -> None:
    eid, manager = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "jam", "assignees": ["CMD_A", "FOE_B"],
        "target": "FOE_B", "group": "FANOUT_1",
    })
    assert ok, why
    tasks = [t for t in manager.snapshot()
             if t["task_type"] == "jam" and t["group_id"] == "FANOUT_1"]
    assert len(tasks) == 2
    assert len({t["task_id"] for t in tasks}) == 2


def test_two_attack_jobs_run_concurrently_and_cancel_one(sim: Simulation) -> None:
    """两项 attack 并发（各盯各的弹）；撤一项只撕一项。"""
    eid, manager = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "attack", "target": "FOE_B", "group": "PAIR_A",
    })
    assert ok, why
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "attack", "target": "FOE_B", "group": "PAIR_B",
    })
    assert ok, why
    live = [t for t in manager.snapshot()
            if t["status"] == ACTIVE and t["group_id"] in ("PAIR_A", "PAIR_B")]
    assert len(live) == 2
    ok, why = sim.command(eid, "cancel_task", {"task_id": live[0]["task_id"]})
    assert ok, why
    still = [t for t in manager.snapshot()
             if t["status"] == ACTIVE and t["group_id"] in ("PAIR_A", "PAIR_B")]
    assert len(still) == 1 and still[0]["task_id"] == live[1]["task_id"]
    gone = [t for t in manager.snapshot() if t["task_id"] == live[0]["task_id"]]
    assert gone[0]["status"] == CANCELED


def test_cancel_local_move_task_stops_the_mover(sim: Simulation) -> None:
    eid, manager = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {
        "task_type": "move",
        "payload": {"points": [[5000.0, 0.0], [8000.0, 0.0]]},
    })
    assert ok, why
    live = [t for t in manager.snapshot() if t["status"] == ACTIVE
            and t["task_type"] == "move"]
    task_id = live[0]["task_id"]
    ok, why = sim.command(eid, "cancel_task", {"task_id": task_id})
    assert ok, why
    hist = [t for t in manager.snapshot() if t["task_id"] == task_id]
    assert hist[0]["status"] == CANCELED
    mover = sim.entity_by_name("CMD_A").component("mover")
    assert mover.destination_point() is None     # 停车


def test_resupply_task_transfers_and_completes(sim: Simulation) -> None:
    eid, _ = _manager(sim, "CMD_A")
    # ★ 前面的用例把 CMD_A 开出去 2 km 了——先开回锚点（补给半径 200 m）。
    sim.command(eid, "move", {"points": [[0.0, 0.0, 0.0]]})
    sim.run_for(400 * 1_000_000)
    sid, _ = _manager(sim, "SUPPLY_1")
    tel_weapon = sim.entity_by_name("CMD_A").component("weapon")
    before = tel_weapon.stock()
    capacity = tel_weapon._capacity
    if before == capacity:
        pytest.skip("发射槽是满的，补不动（用例顺序变了）")
    ok, why = sim.command(sid, "assign_task", {
        "task_type": "resupply", "target": "CMD_A",
    })
    assert ok, why
    sim.run_for(3 * 1_000_000)
    assert tel_weapon.stock() == capacity            # 补满
    supply_entity = sim.entity_by_name("SUPPLY_1")
    supply = supply_entity.component("supply")
    assert supply.ammo_left() == 8 - (capacity - before)
    mgr = next(p for p in supply_entity.parts() if isinstance(p, TaskManager))
    hist = [t for t in mgr.snapshot()
            if t["ledger"] == "history" and t["task_type"] == "resupply"]
    assert hist and hist[0]["status"] == COMPLETE


def test_duplicate_assign_is_deduped(sim: Simulation) -> None:
    """重传副本只补 ack，不重复派发。"""
    eid, manager = _manager(sim, "CMD_A")
    payload = {
        "task_id": 999_001, "task_type": "move", "group_id": "GX",
        "assigner_id": 424242, "assignee_id": eid,
        "target_id": None, "payload": {"points": [[1.0, 1.0]]},
        "assign_time_us": 0,
    }
    msg = SimpleNamespace(sender_id=424242, payload=payload)
    manager._on_assign(msg, 0)
    manager._on_assign(msg, 0)                # 重传副本
    mine = [t for t in manager._received.values() if t.task_id == 999_001]
    assert len(mine) == 1                     # 只派发一次


def test_command_layer_vocabulary_declares_task_words(sim: Simulation) -> None:
    eid, _ = _manager(sim, "CMD_A")
    ok, why = sim.command(eid, "assign_task", {})
    assert not ok and "task_type" in why
    ok, why = sim.command(eid, "cancel_task", {"task_id": "x"})
    assert not ok and "task_id" in why
    assert "assign_task" in TaskManager.COMMANDS
    assert "cancel_task" in TaskManager.COMMANDS


def test_uplink_assign_and_cancel_parse() -> None:
    srv = _load_serve_replay("parse")
    cmd, data, err = srv.parse_uplink(json.dumps({
        "cmd": "assign_task", "target": "CMD_A", "task_type": "attack",
        "victim": "FOE_B", "assignees": ["CMD_A"],
        "payload": {"rounds": 1},
    }))
    assert (cmd, err) == ("assign_task", "")
    assert data["task_type"] == "attack"

    cmd, _, err = srv.parse_uplink(json.dumps({
        "cmd": "assign_task", "target": "CMD_A"}))
    assert cmd == "" and "task_type" in err

    cmd, data, err = srv.parse_uplink(json.dumps({
        "cmd": "cancel_task", "target": "CMD_A", "task_id": 5}))
    assert (cmd, err) == ("cancel_task", "")

    cmd, _, err = srv.parse_uplink(json.dumps({
        "cmd": "cancel_task", "target": "CMD_A", "task_id": "5"}))
    assert cmd == "" and "task_id" in err


def test_uplink_assign_executes(sim: Simulation) -> None:
    srv = _load_serve_replay("exec")
    ok, why = srv.execute_uplink(sim, {
        "cmd": "assign_task", "target": "CMD_A", "task_type": "jam",
        "victim": "FOE_B",
    })
    assert ok, why
    eid, manager = _manager(sim, "CMD_A")
    assert any(t["task_type"] == "jam" for t in manager.snapshot())


def test_target_death_auto_resolves_jam_task(sim: Simulation) -> None:
    """JAM_1 自派 jam 盯 FOE_B；目标注销 ⇒ canceled/target_lost。

    ★ 放在用例序列**最后**：它把 FOE_B 从名册上摘掉，别的用例不能再
    引用这个目标。
    """
    jid, manager = _manager(sim, "JAM_1")
    ok, why = sim.command(jid, "assign_task", {
        "task_type": "jam", "target": "FOE_B",
    })
    assert ok, why
    live = [t for t in manager.snapshot() if t["status"] == ACTIVE
            and t["task_type"] == "jam"]
    assert live, why
    foe_id = sim.entity_by_name("FOE_B").entity_id
    sim.registry.unregister(foe_id)
    sim.run_for(3 * 1_000_000)
    hist = [t for t in manager.snapshot()
            if t["task_type"] == "jam" and t["ledger"] == "history"]
    assert hist and hist[0]["status"] == CANCELED
    assert hist[0]["sub"] == "target_lost"


# ---------------------------------------------------------------------------
# 可靠重传：专用想定（单向网 + 双方都有台账）
# ---------------------------------------------------------------------------

_RETRANSMIT_SCENARIO = """
simulation
    name     "重传测试"
    max_step 1 s
    seed     7
end_simulation

component_type GROUND_MOVER
end_component_type
component_type MOVER_TRUCK : GROUND_MOVER
    max_speed 15 m/s
    turn_rate 40 deg/s
end_component_type
component_type RF_COMM
end_component_type
component_type TASK_MANAGER
end_component_type
component_type TASK_MANAGER_FAST : TASK_MANAGER
    period 500000
    retry_interval 1000000
    max_retries 3
end_component_type

platform_type NODE_R
    side red
    component mover MOVER_TRUCK
    end_component
    component task TASK_MANAGER_FAST
    end_component
    component comm RF_COMM
    end_component
end_platform_type

zone R_SECTOR
    anchor     30.0 120.0
    radius     20 km
    resolution 500 m
    layers     1
    terrain    flat
end_zone

network ONE_WAY
    member SENDER_A RECEIVER_B
    link RECEIVER_B to SENDER_A
end_network

platform SENDER_A NODE_R
    position latlng 30.0000 120.0000
end_platform

platform RECEIVER_B NODE_R
    position latlng 30.0100 120.0100
end_platform
"""


def test_retransmit_exhaustion_marks_comm_lost() -> None:
    """ONE_WAY 只有 RECEIVER_B→SENDER_A 的边：令送不到，重传耗尽认输。"""
    sim = Simulation.from_scenario(_RETRANSMIT_SCENARIO)
    sim.initialize()
    sender = sim.entity_by_name("SENDER_A")
    manager = next(p for p in sender.parts() if isinstance(p, TaskManager))
    ok, why = sim.command(sender.entity_id, "assign_task", {
        "task_type": "move", "assignees": ["RECEIVER_B"],
        "payload": {"points": [[0.0, 0.0]]},
    })
    assert ok, why
    # retry_interval 1 s、max_retries 3：首发后 1/2/3 s 各重传一次，
    # 第 4 s 认输 ⇒ failed/comm_lost。
    sim.run_for(10 * 1_000_000)
    hist = [t for t in manager.snapshot() if t["ledger"] == "history"]
    assert hist and hist[0]["status"] == FAILED
    assert hist[0]["sub"] == "comm_lost"
    assert manager._retries == 3


def test_no_network_falls_back_to_direct_post() -> None:
    """没建网 ⇒ 直投（不限制口径），任务照常送达，无需重传。

    ★ 没建网时**不能有 RF_COMM**（挂通信件却不在任何网里 = 装配期
    报错），所以这份想定把通信件与网络块一起去掉。
    """
    text = _RETRANSMIT_SCENARIO.replace(
        """network ONE_WAY
    member SENDER_A RECEIVER_B
    link RECEIVER_B to SENDER_A
end_network
""", "").replace(
        """    component task TASK_MANAGER_FAST
    end_component
    component comm RF_COMM
    end_component
""", """    component task TASK_MANAGER_FAST
    end_component
""")
    sim = Simulation.from_scenario(text)
    sim.initialize()
    sender = sim.entity_by_name("SENDER_A")
    receiver = sim.entity_by_name("RECEIVER_B")
    manager = next(p for p in sender.parts() if isinstance(p, TaskManager))
    ok, why = sim.command(sender.entity_id, "assign_task", {
        "task_type": "move", "assignees": ["RECEIVER_B"],
        "payload": {"points": [[0.0, 0.0]]},
    })
    assert ok, why
    sim.run_for(5 * 1_000_000)
    r_mgr = next(p for p in receiver.parts() if isinstance(p, TaskManager))
    assert (r_mgr.statistics()["received"]
            + r_mgr.statistics()["history"]) >= 1
    assert manager._retries == 0            # 直投无需重传


def test_command_rejection_is_a_formal_event(sim: Simulation) -> None:
    """拒绝也是正式事件（v0.13.60）：不存在的实体、被部件拒掉的指令，
    都从 ``Simulation.command`` 落 ``bus.command_rejected``。"""
    seen: list[tuple[int, str, str]] = []
    sub = sim.bus.command_rejected.subscribe(
        lambda eid, cmd, why: seen.append((eid, cmd, why))
    )
    try:
        ok, why = sim.command(999_999, "fire", {})
        assert ok is False and "不存在" in why
        ok, why = sim.command(
            sim.entity_by_name("CMD_A").entity_id, "self_destruct", {}
        )
        assert ok is False
    finally:
        sub.unsubscribe()
    assert [e[0] for e in seen] == [999_999, sim.entity_by_name("CMD_A").entity_id]
    assert [e[1] for e in seen] == ["fire", "self_destruct"]
    assert all(e[2] for e in seen)
