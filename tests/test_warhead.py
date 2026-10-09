"""战斗部组件测试：命中概率裁决 + 毁伤结算（§5.17）。

这一组钉六类东西：

- **概率骨架的数值锚**：``hit_probability`` 是纯函数，手算值直接对拍——
  不动目标必中、机动目标按 σ 折损、弹强则回升、远离则趋零。系数将来
  被 AFSIM 大样本替换（Phase 2），锚点也跟着换，骨架错了这里先炸。
- **标称毁伤**：类型因子 × 质量效应 × 基数，逐项核对。
- **插补事件**：3 km/s 的弹一个 1 s 节拍跑 3 km，触发圈可能整段落在
  两次采样之间——不插补就漏判，漏判与"打歪了"长得一样（模块头）。
  穿越时刻是二次方程的最小正根，测试里手算对拍。
- **一弹一判**：判定后再怎么巡回都不再掷骰——逐帧掷骰会把 P=0.2
  累积成必然命中，这条是"帧率不变性"的根。
- **毁伤换算链**：标称毁伤 × armor ÷ max_health → 完好度。三个默认值
  （1 / 1 / 1）保证"没写这些参数的想定"逐字走旧行为。
- **放弃语义**：弹停飞（起爆/坠地）、目标先死、目标消失——都不判定、
  有结论可查，不静默挂着。
"""

from __future__ import annotations

import random
from typing import Any

import pytest

from milsim.errors import ConfigurationError
from milsim.models.damage import WARHEAD_FACTORS, Warhead, hit_probability
from milsim.services.bus import EventBus
from milsim.services.type_registry import ComponentFactory

TARGET_ID = 7
TARGET_NAME = "TGT"
MY_ID = 1


# ---------------------------------------------------------------------------
# 纯函数：概率骨架的数值锚
# ---------------------------------------------------------------------------

def test_stationary_target_is_certain_hit() -> None:
    """目标不机动（max_accel 0）→ σ 退化为地板 → 必中。"""
    p = hit_probability(
        3000.0, 3000.0, 0.0, 150.0,
        kill_radius=30.0, maneuver_factor=0.35, sigma_floor=3.0,
    )
    assert p >= 1.0 - 1e-9


def test_maneuvering_target_drops_probability() -> None:
    """a_t=100、a_m=150、t_go=1 s：σ = 0.35·100·1·(100/250)+3 = 17。

    P = 1 − exp(−30²/(2·17²)) = 1 − exp(−900/578) ≈ 0.7890。
    """
    p = hit_probability(
        3000.0, 3000.0, 100.0, 150.0,
        kill_radius=30.0, maneuver_factor=0.35, sigma_floor=3.0,
    )
    assert p == pytest.approx(1.0 - pow(2.718281828459045, -900.0 / 578.0), abs=1e-3)
    assert 0.5 < p < 0.95


def test_agile_missile_recovers_probability() -> None:
    """弹机动 400：份额 100/550 → σ = 0.35·100·(100/550)+3 ≈ 9.36 → P≈0.994。"""
    p = hit_probability(
        3000.0, 3000.0, 100.0, 400.0,
        kill_radius=30.0, maneuver_factor=0.35, sigma_floor=3.0,
    )
    assert p > 0.98


def test_not_closing_means_no_hit() -> None:
    """没在接近（closing=0）而目标有机动 → t_go 巨大 → σ 巨大 → P≈0。"""
    p = hit_probability(
        3000.0, 0.0, 100.0, 150.0,
        kill_radius=30.0, maneuver_factor=0.35, sigma_floor=3.0,
    )
    assert p < 1e-6


def test_no_history_judges_on_floor() -> None:
    """closing 不可知（无差分历史）→ 只按固有散布判（偏向命中）。"""
    p = hit_probability(
        3000.0, None, 100.0, 150.0,
        kill_radius=30.0, maneuver_factor=0.35, sigma_floor=3.0,
    )
    assert p >= 1.0 - 1e-9


def test_zero_kill_radius_never_hits() -> None:
    assert hit_probability(
        100.0, 1000.0, 0.0, 150.0,
        kill_radius=0.0, maneuver_factor=0.35, sigma_floor=3.0,
    ) == 0.0


# ---------------------------------------------------------------------------
# 标称毁伤
# ---------------------------------------------------------------------------

def build_warhead(params: dict[str, Any] | None = None) -> Warhead:
    """自建注册表构造——不依赖全局表的状态（别的测试会清它）。"""
    from milsim import models
    from milsim.services.type_registry import ComponentRegistry

    components = ComponentRegistry()
    models.register_framework(components=components)
    factory = ComponentFactory(components=components)
    return factory.build("damage", "WARHEAD", dict(params or {}))


def test_raw_damage_default_he_250kg() -> None:
    """默认档：0.5 × HE(1.0) × (250/250)^1 = 0.5 点（质量比默认中性）。"""
    assert build_warhead().raw_damage() == pytest.approx(0.5)


def test_raw_damage_type_factor() -> None:
    w = build_warhead({"warhead_type": "FRAG"})
    assert w.raw_damage() == pytest.approx(0.5 * WARHEAD_FACTORS["FRAG"])


def test_raw_damage_zero_mass() -> None:
    assert build_warhead({"warhead_mass": 0.0}).raw_damage() == 0.0


def test_raw_damage_exponent() -> None:
    """α=2：1000 kg → (1000/250)² = 16 → 0.5×1×16 = 8.0 点。"""
    w = build_warhead({"warhead_mass": 1000.0, "damage_exponent": 2.0})
    assert w.raw_damage() == pytest.approx(8.0)


def test_raw_damage_quality_independent_of_mass() -> None:
    w = build_warhead({"warhead_mass": 100.0, "damage_exponent": 0.0})
    assert w.raw_damage() == pytest.approx(0.5)


def test_raw_damage_mass_scaling_via_reference() -> None:
    """只改装药不改参考档：250 → 500 kg 翻倍 = 1.0 点（质量效应通道仍在）。"""
    w = build_warhead({"warhead_mass": 500.0})
    assert w.raw_damage() == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# 桩：store / 视图 / 注册表 / 引擎
# ---------------------------------------------------------------------------

class FakeStore:
    """裁决域的最小真语义：apply_damage 与 store 同式（完好度封顶在 0）。"""

    def __init__(self) -> None:
        self._damage: dict[int, float] = {}
        self.effects: list[Any] = []
        self.apply_calls: list[tuple[int, float]] = []
        self.removed: list[int] = []

    def apply_damage(self, entity_id: int, amount: float) -> float:
        current = max(0.0, float(self._damage.get(entity_id, 1.0)) - abs(amount))
        self._damage[entity_id] = current
        self.apply_calls.append((entity_id, amount))
        return current

    def set_damage(self, entity_id: int, value: float) -> None:
        self._damage[entity_id] = min(1.0, max(0.0, float(value)))

    def remove(self, entity_id: int) -> bool:
        """注销：位置查不到、is_alive False——战斗部命中后弹体走这条。"""
        self.removed.append(entity_id)
        self._damage.pop(entity_id, None)
        return True

    def clear_effects(self) -> int:
        count = len(self.effects)
        self.effects.clear()
        return count


class FakeEngagement:
    """位置 + request_effect（意图进 store 队列——真通道的桩）。"""

    def __init__(self, store: FakeStore,
                 positions: dict[int, tuple[float, float, float]]) -> None:
        self._store = store
        # 持引用不拷贝：测试挪 rig.positions 必须被组件看见（拷贝快照
        # 模拟不了"组件读实时位置"，这是 test_moved_target 的老坑）。
        self._positions = positions

    def my_position(self) -> tuple[float, float, float] | None:
        return self._positions.get(MY_ID)

    def position_of(self, target_id: int) -> tuple[float, float, float] | None:
        return self._positions.get(target_id)

    def is_alive(self, target_id: int) -> bool:
        return self._store._damage.get(target_id, 1.0) > 0.0

    def request_effect(self, target_id: int, kind: str, magnitude: float,
                       at: int = 0, note: str = "") -> Any:
        effect = type("Effect", (), {})()
        effect.source_id = MY_ID
        effect.target_id = target_id
        effect.kind = kind
        effect.magnitude = magnitude
        effect.at = at
        effect.note = note
        self._store.effects.append(effect)
        return effect


class FakeEntity:
    def __init__(self, entity_id: int,
                 platform_params: dict[str, Any] | None = None,
                 components: dict[str, Any] | None = None) -> None:
        self.entity_id = entity_id
        self.platform_params = dict(platform_params or {})
        self._components = dict(components or {})

    def component(self, slot: str) -> Any:
        return self._components.get(slot)


class FakeMover:
    """弹停飞检测读的 ``terminated()``（解析弹道弹 / 弹体的公共接口）。"""

    def __init__(self) -> None:
        self._done = False

    def terminated(self) -> bool:
        return self._done


class FakeRegistry:
    def __init__(self, by_name: dict[str, FakeEntity]) -> None:
        self._by_name = dict(by_name)
        self.unregistered: list[int] = []

    def by_name(self, name: str) -> FakeEntity | None:
        return self._by_name.get(name)

    def get(self, entity_id: int) -> FakeEntity | None:
        for entity in self._by_name.values():
            if entity.entity_id == entity_id:
                return entity
        return None

    def unregister(self, entity_id: int) -> bool:
        self.unregistered.append(entity_id)
        return True


class FakeEngine:
    def __init__(self) -> None:
        self.now = 0
        self.events: list[tuple[int, Any, int]] = []

    def schedule(self, delay_us: int, fn: Any, priority: int = 0) -> int:
        self.events.append((self.now + int(delay_us), fn, priority))
        return len(self.events)


class WarMount:
    """战斗部 initialize 真正用到的那几项。"""

    def __init__(self, store: FakeStore, engagement: FakeEngagement,
                 registry: FakeRegistry, bus: EventBus | None, seed: int = 42,
                 entity: FakeEntity | None = None) -> None:
        self.store = store
        self.engagement = engagement
        self.registry = registry
        self.bus = bus
        self.entity = entity
        self._seed = seed
        self.every_calls: list[tuple[int, Any, int]] = []

    def engagement_view(self) -> FakeEngagement:
        return self.engagement

    def streams(self) -> Any:
        mount = self

        class _Streams:
            engage = random.Random(mount._seed)

        return _Streams()

    def every(self, interval_us: int, fn: Any, priority: int = 0) -> None:
        self.every_calls.append((interval_us, fn, priority))


class Rig:
    """一份装好线的战斗部 + 可手推的时钟。"""

    def __init__(self, warhead: Warhead, mount: WarMount, engine: FakeEngine,
                 positions: dict[int, tuple[float, float, float]],
                 mover: FakeMover | None = None) -> None:
        self.warhead = warhead
        self.mount = mount
        self.engine = engine
        self.positions = positions
        self.mover = mover

    def sweep(self) -> None:
        self.warhead._sweep(self.engine, None)

    def run_events(self) -> int:
        """执行所有已到期的一次性事件，返回执行数。"""
        due = [item for item in self.engine.events if item[0] <= self.engine.now]
        self.engine.events = [item for item in self.engine.events if item[0] > self.engine.now]
        for _, fn, _ in due:
            fn(self.engine)
        return len(due)


def make_rig(
    params: dict[str, Any] | None = None,
    *,
    target: tuple[float, float, float] = (0.0, -7000.0, 0.0),
    missile: tuple[float, float, float] = (0.0, 0.0, 0.0),
    target_params: dict[str, Any] | None = None,
    bus: EventBus | None = None,
    seed: int = 42,
    with_mover: bool = True,
) -> Rig:
    store = FakeStore()
    positions = {MY_ID: missile, TARGET_ID: target}
    engagement = FakeEngagement(store, positions)
    target_entity = FakeEntity(TARGET_ID, target_params)
    registry = FakeRegistry({TARGET_NAME: target_entity})
    mover = FakeMover() if with_mover else None
    my_entity = FakeEntity(MY_ID, components={"mover": mover} if mover else None)
    mount = WarMount(store, engagement, registry, bus, seed=seed, entity=my_entity)
    warhead = build_warhead({"target_name": TARGET_NAME, **(params or {})})
    warhead.bind(my_entity)
    engine = FakeEngine()
    warhead.initialize(mount)
    return Rig(warhead, mount, engine, positions, mover)


# ---------------------------------------------------------------------------
# 防呆
# ---------------------------------------------------------------------------

def test_missing_target_name_is_rejected() -> None:
    store = FakeStore()
    mount = WarMount(store, FakeEngagement(store, {}), FakeRegistry({}), None)
    warhead = build_warhead({})
    with pytest.raises(ConfigurationError, match="没有目标"):
        warhead.initialize(mount)


def test_unknown_target_name_is_rejected() -> None:
    store = FakeStore()
    mount = WarMount(store, FakeEngagement(store, {}), FakeRegistry({}), None)
    warhead = build_warhead({"target_name": "GHOST"})
    with pytest.raises(ConfigurationError, match="不在"):
        warhead.initialize(mount)


# ---------------------------------------------------------------------------
# 巡回、插补与一弹一判
# ---------------------------------------------------------------------------

def test_first_sweep_without_history_does_not_interpolate() -> None:
    """第一拍圈外：没有差分历史，不插补也不判定。"""
    rig = make_rig()
    rig.sweep()
    assert not rig.warhead.judged()
    assert not rig.engine.events


def test_crossing_event_is_scheduled_with_exact_time() -> None:
    """弹 3 km/s、目标 7 km 外：第二拍 d=4 km，t_cross=1/3 s → 插 333 333 µs。

    手算：|v|²t² + 2(r·v)t + (|r|²−R²) = 9e6t² − 24e6t + 7e6 = 0，
    小根 (24−18)/18 = 1/3。
    """
    rig = make_rig()
    rig.positions[MY_ID] = (0.0, -3000.0, 0.0)     # 第二拍时弹的位置
    rig.engine.now = 1_000_000
    rig.warhead._prev_my = (0.0, 0.0, 0.0)         # 上一拍历史
    rig.warhead._prev_tgt = (0.0, -7000.0, 0.0)
    rig.warhead._prev_t = 0
    rig.sweep()
    assert len(rig.engine.events) == 1
    due, _, _ = rig.engine.events[0]
    assert due == 1_000_000 + 333_333


def test_crossing_event_judges_with_true_geometry() -> None:
    """插补事件触发：此刻弹恰在圈沿（d=3 km），判定发生并扣 0.25。"""
    rig = make_rig()
    rig.engine.now = 1_000_000
    rig.positions[MY_ID] = (0.0, -3000.0, 0.0)     # 第二拍：3 km/s 飞了 3 km
    rig.warhead._prev_my = (0.0, 0.0, 0.0)         # 上一拍历史
    rig.warhead._prev_tgt = (0.0, -7000.0, 0.0)
    rig.warhead._prev_t = 0
    rig.sweep()
    assert len(rig.engine.events) == 1

    rig.engine.now = 1_333_333
    rig.positions[MY_ID] = (0.0, -4000.0, 0.0)     # 3 km/s × 1.333 s
    assert rig.run_events() == 1
    assert rig.warhead.judged()
    assert "命中" in rig.warhead.verdict()
    damage = rig.mount.store._damage[TARGET_ID]
    assert damage == pytest.approx(0.5)            # 1 − 0.5（默认伤害 0.5 点 / 血条 1.0）


def test_judged_exactly_once() -> None:
    """判定之后：再巡回、再插补都不再掷骰——rng 状态纹丝不动。"""
    rig = make_rig({"fuse_radius": 9000.0})
    rig.sweep()                                     # 第一拍：d=7 km < 9 km 圈内
    assert rig.warhead.judged()
    rng_before = rig.warhead._rng.getstate()
    rig.sweep()
    rig.sweep()
    rig.engine.now += 1_000_000
    rig.sweep()
    assert rig.warhead._rng.getstate() == rng_before
    assert len(rig.mount.store.apply_calls) == 1


def test_terminated_missile_outside_ring_is_abandoned() -> None:
    """机动件报告终止而弹没进圈（起爆在圈外 / 坠地）→ 放弃，结论可查。

    ★ 弹停必须读机动件的终止状态，不能用"位置连续几拍没变"：机动件
    一帧才置位一次（解析弹道弹 5 s 一帧），帧间采样读到的都是旧位置
    ——速度差分判停会在弹正常飞行途中误放弃（端到端实测踩过）。
    """
    rig = make_rig(target=(0.0, -8000.0, 0.0))
    rig.positions[MY_ID] = (0.0, -1000.0, 0.0)
    rig.engine.now = 1_000_000
    rig.warhead._prev_my = (0.0, 0.0, 0.0)
    rig.warhead._prev_tgt = (0.0, -8000.0, 0.0)
    rig.warhead._prev_t = 0
    rig.sweep()
    assert not rig.warhead.abandoned()

    rig.mover._done = True                 # 机动件终止（弹已坠地/起爆）
    rig.sweep()
    assert rig.warhead.abandoned()
    assert "放弃" in rig.warhead.verdict()


def test_terminated_missile_inside_ring_still_judges() -> None:
    """终止弹落在圈内（解析弹落地 = 目标点，d=0）→ 判定优先于放弃。"""
    rig = make_rig({"fuse_radius": 3000.0}, target=(0.0, -500.0, 0.0))
    rig.mover._done = True
    rig.sweep()
    assert rig.warhead.judged()
    assert not rig.warhead.abandoned()
    assert rig.mount.store._damage[TARGET_ID] == pytest.approx(0.5)


def test_no_mover_slot_disables_stop_detection() -> None:
    """mover_slot 指向空槽：停飞检测关闭，巡回继续（不误弃）。"""
    rig = make_rig({"mover_slot": "no_such_slot"}, target=(0.0, -8000.0, 0.0))
    rig.positions[MY_ID] = (0.0, -1000.0, 0.0)
    rig.engine.now = 1_000_000
    rig.warhead._prev_my = (0.0, 0.0, 0.0)
    rig.warhead._prev_tgt = (0.0, -8000.0, 0.0)
    rig.warhead._prev_t = 0
    rig.sweep()
    assert not rig.warhead.abandoned()
    assert not rig.warhead.judged()


def test_dead_target_is_abandoned() -> None:
    rig = make_rig({"fuse_radius": 9000.0})
    rig.mount.store._damage[TARGET_ID] = 0.0
    rig.sweep()
    assert rig.warhead.abandoned()
    assert "目标" in rig.warhead.verdict()


def test_vanished_target_is_abandoned() -> None:
    rig = make_rig({"fuse_radius": 9000.0})
    del rig.positions[TARGET_ID]
    rig.sweep()
    assert rig.warhead.abandoned()


# ---------------------------------------------------------------------------
# 毁伤换算链
# ---------------------------------------------------------------------------

def test_default_chain_subtracts_half() -> None:
    """默认链：伤害 0.5 点、血条 1.0 → 扣一半 → 完好度 0.5。"""
    rig = make_rig({"fuse_radius": 9000.0})
    rig.sweep()
    assert rig.mount.store._damage[TARGET_ID] == pytest.approx(0.5)


def test_damage_is_points_subtracted_from_health() -> None:
    """用户口径：血条 1000 挨 200 伤害 → 剩 800 → 对外显示 80% 血量。"""
    rig = make_rig(
        {"fuse_radius": 9000.0, "damage_scale": 200.0},
        target_params={"max_health": 1000.0},
    )
    rig.sweep()
    assert rig.mount.store._damage[TARGET_ID] == pytest.approx(0.8)


def test_armor_scales_damage() -> None:
    """armor 0.9：实际伤害 0.5×0.9 = 0.45。"""
    rig = make_rig({"fuse_radius": 9000.0}, target_params={"armor": 0.9})
    rig.sweep()
    assert rig.mount.store._damage[TARGET_ID] == pytest.approx(1.0 - 0.45)


def test_max_health_dilutes_damage() -> None:
    """血条 1000 挨 0.5 点：完好度 1 − 0.0005。"""
    rig = make_rig({"fuse_radius": 9000.0}, target_params={"max_health": 1000.0})
    rig.sweep()
    assert rig.mount.store._damage[TARGET_ID] == pytest.approx(1.0 - 0.0005)


def test_armor_and_max_health_compose() -> None:
    """合成：伤害 200、防护 0.9、血条 1000 → 实际伤害 180 → 剩 820 → 82%。"""
    rig = make_rig(
        {"fuse_radius": 9000.0, "damage_scale": 200.0},
        target_params={"max_health": 1000.0, "armor": 0.9},
    )
    rig.sweep()
    assert rig.mount.store._damage[TARGET_ID] == pytest.approx(0.82)


def test_lethal_hit_destroys_and_broadcasts() -> None:
    """raw=1.0 一发入魂：完好度 0、is_alive False、事件 (…, True, 1.0, True)。"""
    bus = EventBus()
    seen: list[tuple[Any, ...]] = []
    bus.engagement_resolved.subscribe(lambda *args: seen.append(args))
    rig = make_rig(
        {"fuse_radius": 9000.0, "damage_scale": 1.0, "warhead_mass": 250.0},
        bus=bus,
    )
    rig.sweep()
    assert rig.mount.store._damage[TARGET_ID] == 0.0
    assert not rig.mount.engagement.is_alive(TARGET_ID)
    assert len(seen) == 1
    attacker, victim, hit, fraction, destroyed = seen[0]
    assert attacker == MY_ID and victim == TARGET_ID
    assert hit is True and destroyed is True
    assert fraction == pytest.approx(1.0)


def test_miss_broadcasts_no_damage() -> None:
    """kill_radius=0 → P=0 → 必失的：不扣血、effects 清空、事件 hit=False。"""
    bus = EventBus()
    seen: list[tuple[Any, ...]] = []
    bus.engagement_resolved.subscribe(lambda *args: seen.append(args))
    rig = make_rig({"fuse_radius": 9000.0, "kill_radius": 0.0}, bus=bus)
    rig.sweep()
    assert TARGET_ID not in rig.mount.store._damage
    assert not rig.mount.store.effects
    assert len(seen) == 1
    _, _, hit, fraction, destroyed = seen[0]
    assert hit is False and destroyed is False and fraction == 0.0


def test_hit_deregisters_the_missile_itself() -> None:
    """命中 = 起爆：弹体**注销**——store.remove + registry.unregister，
    什么都没有了（位置查不到、名册摘净），不是留个完好度 0 的残骸。"""
    rig = make_rig({"fuse_radius": 9000.0})
    rig.sweep()
    assert MY_ID in rig.mount.store.removed
    assert MY_ID in rig.mount.registry.unregistered


def test_miss_keeps_the_missile_flying() -> None:
    """失的 = 没起爆：弹不被注销，保持飞行直到引信/弹道自然终止。"""
    rig = make_rig({"fuse_radius": 9000.0, "kill_radius": 0.0})
    rig.sweep()
    assert MY_ID not in rig.mount.store.removed
    assert MY_ID not in rig.mount.registry.unregistered


# ---------------------------------------------------------------------------
# 节拍不变性
# ---------------------------------------------------------------------------

def test_verdict_identical_across_check_intervals() -> None:
    """同一匀速几何、不同巡回周期：判定都发生、概率输入同值 →
    同种子下完好度逐字一致。

    节拍 0.5 s 走插补事件（在圈沿判定）、2 s 直接采样进圈（在圈内
    判定）——判定时刻不同，但目标不机动时概率只由固有散布决定，
    与 d 无关。毁伤值本身与判定几何无关，两档必然一致。
    """
    results: list[float] = []
    for interval_us in (500_000, 2_000_000):
        rig = make_rig(
            {"fuse_radius": 3000.0, "check_interval": interval_us}, seed=7
        )
        # 弹从原点以 3 km/s 正北匀速飞向 7 km 外的目标。
        for _ in range(12):
            rig.engine.now += interval_us
            flown = 3000.0 * rig.engine.now / 1e6
            rig.positions[MY_ID] = (0.0, -flown, 0.0)
            rig.sweep()
            rig.run_events()
            if rig.warhead.judged():
                break
        assert rig.warhead.judged(), f"interval={interval_us} 未判定"
        results.append(rig.mount.store._damage[TARGET_ID])
    assert results[0] == pytest.approx(results[1])


# ---------------------------------------------------------------------------
# 端到端：真装配、真弹道、真裁决（demo 想定就是被测对象）
# ---------------------------------------------------------------------------

def _demo_path():
    from pathlib import Path

    return Path(__file__).resolve().parent.parent / "demos" / "afsim_mover" / "warhead.txt"


def test_scenario_end_to_end_engagement() -> None:
    """analytic 弹全程 421.5 s（600 km / 高点 200 km 的闭式值），末段
    进圈判定 → 目标完好度 0.5（伤害 0.5 点 / 血条 1.0）。

    这一条同时验证：想定装配（damage 槽）、真 store 裁决域、真随机流、
    机动件终止检测不误触发（解析弹道弹 5 s 一帧，1 s 采样的帧间静止
    不能当停飞）。目标不机动（max_accel 默认 0）→ 必中。命中 = 起爆：
    弹体注销——名册摘净、位置查不到。
    """
    from milsim import models
    from milsim.services.scenario import load_scenario
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    registry = ComponentRegistry()
    models.register_framework(components=registry)
    spec = load_scenario(_demo_path().read_text(encoding="utf-8"))
    sim = Simulation(spec, components=registry)
    sim.initialize()                               # 装配是懒执行的：先建实体再抓组件引用

    launcher = sim.registry.by_name("SRBM_1").entity_id
    warhead = sim.registry.get(launcher).component("damage")
    sim.run_for(500_000_000)                       # 500 s > 421.5 s 全程

    target_id = sim.registry.by_name("TGT_WH").entity_id
    assert sim.store.damage_of(target_id) == pytest.approx(0.5)

    assert warhead is not None and warhead.judged()
    assert "命中" in warhead.verdict()
    # 命中 = 起爆：弹体注销，什么都没有了。
    assert sim.registry.by_name("SRBM_1") is None
    assert sim.store.position_of(launcher) is None
    assert not sim.store.is_alive(launcher)
