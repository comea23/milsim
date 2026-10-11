"""查表命中概率测试：标定表运行期接口（§5.17.10）。

这一组钉五类东西：

- **最近档映射**：轴值落在档间映射到最近档，并列取低档（保守）。
- **表值忠实**：查询结果与 json 原始行逐字一致（已知的零命中格、
  满命中格、0.9333 格各一）——查表层的唯一职责是把数据原样递出来，
  任何"聪明"的修正都是错。
- **log 半径内插**：R=30/100/300 精确落档值；中间值夹在两档之间；
  区间外外推 clamp [0, 1]。
- **K 层回退**：truth p=0 的格（json 里 null）回退同 g_m 组中位；
  g_m=0 不在 K 网格 → 1.0（无效应）；jam=0 → 恒 1.0。
- **Warhead 集成**：``p_model="table"`` 的判定概率 = 查表值（固定
  随机流对拍 verdict）；verdict 带通道注记；默认 analytic 行为不被
  波及（test_warhead.py 的 33 项守护旧行为）。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from milsim.models.damage import ProbTable, Warhead, load_default
from milsim.models.damage.prob_table import _nearest

_TABLE_PATH = Path(__file__).parents[1] / "src" / "milsim" / "models" / "damage" / "prob_table.json"


@pytest.fixture(scope="module")
def table() -> ProbTable:
    return load_default()


# ---------------------------------------------------------------------------
# 最近档映射
# ---------------------------------------------------------------------------

def test_nearest_axis() -> None:
    axis = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 20.0]
    assert _nearest(axis, 5.4) == 5          # 5.4 → 5 档（|0.4| < |0.6|）
    assert _nearest(axis, 5.6) == 6
    assert _nearest(axis, 7.9) == 6          # 高档间靠 6（|1.9|<|2.1|）
    assert _nearest(axis, 15.0) == 7         # 15 → 10 与 20 并列，取低档
    assert _nearest(axis, -3.0) == 0         # 越界 → 边缘档


def test_axes_from_data(table: ProbTable) -> None:
    ax = table.axes()
    assert ax["g_m"] == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 20.0)
    assert ax["v_t_kms"][0] == 0.2 and ax["v_t_kms"][-1] == 10.0


# ---------------------------------------------------------------------------
# 表值忠实：与 json 原始行对拍
# ---------------------------------------------------------------------------

def _raw_row(key: tuple[float, float, float, float]) -> list[float]:
    data = json.loads(_TABLE_PATH.read_text(encoding="utf-8"))
    for row in data["p_hit"]:
        if tuple(row[:4]) == key:
            return row[4:7]
    raise AssertionError(f"cell {key} not in table")


def test_table_value_matches_json(table: ProbTable) -> None:
    """三格对拍：满命中、零命中、已知 0.9333 格（vm4/gt3/gm20/seed 系
    综计的 CPA 锚点值）。"""
    for key in ((6.0, 8.0, 0.0, 0.2), (0.0, 1.0, 0.0, 0.2),
                (20.0, 4.0, 3.0, 2.0), (6.0, 4.0, 2.0, 2.0)):
        expected = _raw_row(key)
        for r, want in zip((30.0, 100.0, 300.0), expected):
            assert table.p_hit(*key, r) == pytest.approx(want), (key, r)


def test_probability_within_bounds(table: ProbTable) -> None:
    for g_m in (0.0, 3.0, 6.0, 20.0):
        for v_m in (1.0, 5.0, 10.0):
            for g_t in (0.0, 6.0, 20.0):
                for v_t in (0.2, 5.0, 10.0):
                    for r in (10.0, 30.0, 65.0, 300.0, 1000.0):
                        p = table.p_hit(g_m, v_m, g_t, v_t, r)
                        assert 0.0 <= p <= 1.0


def test_maneuver_monotonicity(table: ProbTable) -> None:
    """目标机动越强概率不升（表的单调性抽查：g_t=0 vs g_t=6，
    逐格对拍——0-anchor 阶跃面允许相等，不允许反向）。"""
    for g_m in (1.0, 6.0, 20.0):
        for v_m in (1.0, 4.0, 8.0):
            for v_t in (0.2, 2.0, 6.0):
                assert (table.p_hit(g_m, v_m, 6.0, v_t, 100.0)
                        <= table.p_hit(g_m, v_m, 0.0, v_t, 100.0) + 1e-12)


# ---------------------------------------------------------------------------
# log 半径内插
# ---------------------------------------------------------------------------

def test_radius_interpolation_brackets(table: ProbTable) -> None:
    key = (6.0, 4.0, 2.0, 2.0)
    p30 = table.p_hit(*key, 30.0)
    p100 = table.p_hit(*key, 100.0)
    p65 = table.p_hit(*key, 65.0)
    assert p65 == pytest.approx(_raw_row(key)[0]
                                + (_raw_row(key)[1] - _raw_row(key)[0])
                                * (__import__("math").log(65.0 / 30.0)
                                   / __import__("math").log(100.0 / 30.0)))
    assert min(p30, p100) - 1e-12 <= p65 <= max(p30, p100) + 1e-12


def test_radius_extrapolation_clamped(table: ProbTable) -> None:
    key = (3.0, 2.0, 2.0, 2.0)
    assert 0.0 <= table.p_hit(*key, 5.0) <= 1.0     # 低端外推
    assert 0.0 <= table.p_hit(*key, 2000.0) <= 1.0  # 高端外推
    assert table.p_hit(*key, 0.0) == 0.0            # 半径 0 = 必不中


# ---------------------------------------------------------------------------
# K 层：回退与联合
# ---------------------------------------------------------------------------

def test_k_jam_zero_power_is_identity(table: ProbTable) -> None:
    assert table.k_jam(6.0, 4.0, 2.0, 2.0, 0.0, 100.0) == 1.0
    assert table.k_jam(0.0, 1.0, 0.0, 0.2, 0.0, 30.0) == 1.0


def test_k_jam_matches_raw_row(table: ProbTable) -> None:
    """与原始 k_jam 行逐字对拍（半径优先展平：p100 五档 + p300 五档）。"""
    data = json.loads(_TABLE_PATH.read_text(encoding="utf-8"))
    row = next(e for e in data["k_jam"]
               if tuple(e[:4]) == (6.0, 4.0, 2.0, 2.0))
    flat = row[4:]
    for ji, jam in enumerate((1.0, 10.0, 30.0, 100.0, 1000.0)):
        assert table.k_jam(6.0, 4.0, 2.0, 2.0, jam, 100.0) == pytest.approx(flat[ji])
        assert table.k_jam(6.0, 4.0, 2.0, 2.0, jam, 300.0) == pytest.approx(flat[5 + ji])


def test_k_null_falls_back_to_group_median(table: ProbTable) -> None:
    """truth p=0 的格 K 为 null → 同 g_m 组中位；g_m=0 组不存在 → 1.0。"""
    # g_m=0 不在 K 网格（{1,3,6,20}）——回退 1.0。
    assert table.k_link(0.0, 4.0, 2.0, 2.0, 100.0) == 1.0
    # K 值全部在 (0, +inf)：回退后仍是合法概率乘子。
    for g_m in (1.0, 3.0, 6.0, 20.0):
        k = table.k_link(g_m, 4.0, 2.0, 2.0, 100.0)
        assert 0.0 < k <= 2.0


def test_k_rcs_base_and_direction(table: ProbTable) -> None:
    """基档 1.0 m² = 恒等；隐身（0.01）降、大 RCS 升（跟踪质量通道）。"""
    key = (6.0, 4.0, 2.0, 2.0)
    assert table.k_rcs(*key, 1.0, 100.0) == 1.0
    assert table.k_rcs(*key, 0.0, 100.0) == 1.0     # ≤0 → 恒等（防呆）
    k_stealth = table.k_rcs(*key, 0.01, 100.0)
    k_big = table.k_rcs(*key, 10.0, 100.0)
    assert 0.0 < k_stealth <= 1.5
    assert 0.0 < k_big <= 2.0


def test_k_rcs_matches_raw_row(table: ProbTable) -> None:
    """与原始 k_rcs 行逐字对拍（半径优先展平：p100 三档 + p300 三档）。"""
    data = json.loads(_TABLE_PATH.read_text(encoding="utf-8"))
    row = next(e for e in data["k_rcs"]
               if tuple(e[:4]) == (6.0, 4.0, 2.0, 2.0))
    flat = row[4:]
    for xi, x in enumerate((0.01, 0.1, 10.0)):
        assert table.k_rcs(6.0, 4.0, 2.0, 2.0, x, 100.0) == pytest.approx(flat[xi])
        assert table.k_rcs(6.0, 4.0, 2.0, 2.0, x, 300.0) == pytest.approx(flat[3 + xi])


def test_k_rcs_log_nearest(table: ProbTable) -> None:
    """log 十倍程最近档：0.05 → 0.01 档（log|0.05/0.01|=0.7 <
    log|0.1/0.05|=0.3? 否——0.7 vs 0.3 取 0.1 档）；0.3 → 0.1 档。"""
    key = (6.0, 4.0, 2.0, 2.0)
    k01 = table.k_rcs(*key, 0.01, 100.0)
    k1 = table.k_rcs(*key, 0.1, 100.0)
    # 0.05 离 0.1 更近（log 距离 0.3 vs 0.7）
    assert table.k_rcs(*key, 0.05, 100.0) == pytest.approx(k1)
    # 0.02 离 0.01 更近（log 距离 0.3 vs 0.7）
    assert table.k_rcs(*key, 0.02, 100.0) == pytest.approx(k01)


def test_combined_multiplies_and_clamps(table: ProbTable) -> None:
    p = table.p_hit(6.0, 4.0, 2.0, 2.0, 100.0)
    kl = table.k_link(6.0, 4.0, 2.0, 2.0, 100.0)
    kj = table.k_jam(6.0, 4.0, 2.0, 2.0, 100.0, 100.0)
    assert table.combined(6.0, 4.0, 2.0, 2.0, 100.0, 100.0) == pytest.approx(
        min(1.0, p * kl * kj))
    # 无干扰 = 只乘 K_link。
    assert table.combined(6.0, 4.0, 2.0, 2.0, 100.0, 0.0) == pytest.approx(
        min(1.0, p * kl))


# ---------------------------------------------------------------------------
# Warhead 集成（p_model="table"）
# ---------------------------------------------------------------------------

def _rig(params: dict[str, Any]) -> Any:
    """最小判定环境：进圈、有差分历史、目标平台参数可读。
    与 test_warhead.py 的桩同构，但只搭本文件需要的部分。"""
    class FakeStore:
        def __init__(self) -> None:
            self.applied: list[tuple[int, float]] = []
            self.effects: list[Any] = []
            self.removed: list[int] = []

        def apply_damage(self, eid: int, amount: float) -> float:
            self.applied.append((eid, amount))
            return amount

        def clear_effects(self) -> int:
            n = len(self.effects)
            self.effects.clear()
            return n

        def remove(self, eid: int) -> bool:
            self.removed.append(eid)
            return True

    class FakeView:
        def __init__(self, store: FakeStore) -> None:
            self.store = store
            self.my_pos = (0.0, 0.0, 9144.0)
            self.tgt_pos = (2900.0, 0.0, 9144.0)  # 距离 2900 < fuse 3000

        def my_position(self):
            return self.my_pos

        def position_of(self, eid: int):
            return self.tgt_pos

        def is_alive(self, eid: int) -> bool:
            return True

        def request_effect(self, tid: int, kind: str, magnitude: float,
                           at: int = 0, note: str = "") -> Any:
            e = type("Effect", (), {})()
            e.target_id, e.magnitude = tid, magnitude
            self.store.effects.append(e)
            return e

    class FakeRegistry:
        def __init__(self) -> None:
            self.names = {"TGT": 7}
            self.entities = {}

        def by_name(self, name: str):
            return type("Ref", (), {"entity_id": self.names[name]})() \
                if name in self.names else None

        def get(self, eid: int):
            return self.entities.get(eid)

        def unregister(self, eid: int) -> None:
            pass

    class FakeEntity:
        def __init__(self, eid: int) -> None:
            self.entity_id = eid
            self.platform_params = {"max_accel": 19.6133}  # ≈ 2 g

        def component(self, slot: str):
            return None

    class FakeMount:
        pass

    from milsim.services.bus import EventBus
    from milsim.services.type_registry import ComponentFactory, ComponentRegistry
    from milsim import models

    store = FakeStore()
    view = FakeView(store)
    reg = FakeRegistry()
    tgt = FakeEntity(7)
    mine = FakeEntity(1)
    reg.entities = {1: mine, 7: tgt}

    components = ComponentRegistry()
    models.register_framework(components=components)
    factory = ComponentFactory(components=components)
    spec = dict(params)
    w: Warhead = factory.build("damage", "WARHEAD", spec)

    import random as _random

    mount = FakeMount()
    mount.engagement_view = lambda: view
    mount.store = store
    mount.bus = EventBus()
    mount.registry = reg
    mount.entity = mine
    mount._seed = 20261010
    mount.streams = lambda: type("S", (), {"engage": _random.Random(mount._seed)})()
    mount.every_calls: list[tuple[int, Any, int]] = []
    mount.every = lambda interval_us, fn, priority=0: mount.every_calls.append(
        (interval_us, fn, priority))

    class FakeEngine:
        now = 0

        def schedule(self, delay_us: int, cb: Any, priority: int) -> None:
            pass

    w.initialize(mount)
    # 首拍推进：填差分历史（上一拍位置）。
    w._prev_t = -1_000_000
    w._prev_my = (-1000.0, 0.0, 9144.0)   # 弹速 1000 m/s 沿 +x
    w._prev_tgt = (2900.0, 0.0, 9144.0)   # 目标静止
    return w, store, view


def test_warhead_table_model_probability(table: ProbTable) -> None:
    """p_model=table：判定概率 = combined(15g, 1km/s, 2g, 0, 30m, 0W)。"""
    w, store, view = _rig({
        "target_name": "TGT", "p_model": "table",
        "missile_accel": 147.09975,           # 恰 15 g
        "maneuver_factor": 0.35, "sigma_floor": 3.0,
    })
    w._last_my_speed = 1000.0   # 1 km/s
    w._last_tgt_speed = 0.0     # 静止 → 最低速档 0.2
    w._last_closing = 1000.0
    w._judge(type("E", (), {"now": 0})(), 2900.0)
    expected = table.combined(15.0999, 1.0, 2.0, 0.2, 30.0, 0.0)
    assert "table" in w.verdict()
    assert f"P={expected:.3f}" in w.verdict()


def test_warhead_table_fallback_no_history(table: ProbTable) -> None:
    """首拍无任何差分历史 → 解析骨架回退，verdict 注明。"""
    w, store, view = _rig({
        "target_name": "TGT", "p_model": "table",
    })
    w._judge(type("E", (), {"now": 0})(), 2900.0)
    assert "table→analytic" in w.verdict()


def test_warhead_default_model_unchanged() -> None:
    """默认 analytic：verdict 无 table 注记（旧行为逐字不变）。"""
    w, store, view = _rig({"target_name": "TGT"})
    w._last_closing = 1000.0
    w._judge(type("E", (), {"now": 0})(), 2900.0)
    assert "table" not in w.verdict()
