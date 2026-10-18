"""能力降级（三杀模型，§5.17.11）：纯函数 / 三通道注入 / 装配校验。

分层与 test_freq_hop 同构：

1. **纯函数**——``degrade.resolve`` / 档位边界 / ``capability_of``；
2. **机动通道**——GROUND/AIR mover 的速度出口按完好度查档；
3. **火力通道**——weapon 瘫痪拒发、重损一发耗 2 发；
4. **探测通道**——single_pulse_snr 折扣、关机档扫掠早退；
5. **弹类禁配**——MISSILE/ANALYTIC 弹类误配装配期报错。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.models.damage import degrade
from milsim.services.type_registry import ComponentFactory

from test_mover import FakeMount, FakeView
from test_percep import _EW_RADAR, _build
from test_weapon import _rig


# ===========================================================================
# 第一层：纯函数
# ===========================================================================


def test_builtin_armored_profile_matches_the_three_kill_doctrine() -> None:
    """armored 档 = 三杀经典口径：0.5 重损 / 0.2 瘫痪，三通道各就位。"""
    p = degrade.resolve({"degradation": "armored"}, origin="t")
    assert p.thresholds == (0.5, 0.2)
    assert p.mobility == (1.0, 0.6, 0.25)
    assert p.firepower == ("ok", "halved", "down")
    assert p.sensor_db == (0.0, 6.0, None)
    # 边界逐位钉死：r 恰好落在阈值上算**下一档**（> 才算上一档）。
    assert p.tier(0.9) == 0
    assert p.tier(0.5) == 1
    assert p.tier(0.5000001) == 0
    assert p.tier(0.2) == 2
    assert p.tier(0.0) == 2          # 摧毁档号上与瘫痪同，语义调用方先挡


def test_builtin_naval_profile_survives_progressive_kills() -> None:
    """naval 档：血厚、逐档掉，火力从不全瘫（舰炮打坏一半还能打）。"""
    p = degrade.resolve({"degradation": "naval"}, origin="t")
    assert p.thresholds == (0.6, 0.3)
    assert p.mobility == (1.0, 0.75, 0.5)
    assert p.firepower == ("ok", "halved", "halved")
    assert p.sensor_db == (0.0, 4.0, 12.0)
    assert p.sensor_factor(0.4) == pytest.approx(10.0 ** (-0.4))


def test_unconfigured_platform_resolves_to_none() -> None:
    """什么都不写 = 不降级（现状）；off = 显式关，可覆盖类上的档。"""
    assert degrade.resolve({}, origin="t") is None
    assert degrade.resolve({"degradation": "off"}, origin="t") is None


def test_custom_profile_needs_mobility_and_thresholds() -> None:
    got = degrade.resolve(
        {
            "degradation": "custom",
            "mobility_scale": (1.0, 0.5),
            "degradation_thresholds": (0.4,),
        },
        origin="t",
    )
    assert got.mobility == (1.0, 0.5)
    # 没陪写的通道按档数补"全额"
    assert got.firepower == ("ok", "ok")
    assert got.sensor_db == (0.0, 0.0)


@pytest.mark.parametrize(
    "params, why",
    [
        ({"mobility_scale": (1.0, 0.5)}, "没写 degradation"),
        (
            {
                "degradation": "custom",
                "mobility_scale": (1.0, 0.5),
                "degradation_thresholds": (0.7, 0.2),
            },
            "档数不咬合",
        ),
        ({"degradation": "armored", "mobility_scale": (1.0, 0.0, 0.25)}, "机动比例 0"),
        ({"degradation": "armored", "mobility_scale": (1.2, 0.5, 0.25)}, "机动比例 1.2"),
        ({"degradation": "armored", "sensor_penalty_db": (0.0, -6.0, "off")}, "必须非负"),
        ({"degradation": "armored", "firepower_tiers": ("ok", "bad", "down")}, "火力样式"),
        ({"degradation": "armored", "degradation_thresholds": (0.2, 0.5)}, "严格降序"),
        ({"degradation": "nope"}, "未知的降级档"),
        ({"degradation": "off", "mobility_scale": (1.0, 0.5)}, "互相矛盾"),
    ],
)
def test_bad_configs_are_rejected_at_assembly(params: dict, why: str) -> None:
    with pytest.raises(ConfigurationError, match=why):
        degrade.resolve(params, origin="t")


def test_capability_of_is_the_observer_port() -> None:
    """观察口：未配置=全额；配置了按完好度查档；未知通道报错。"""
    params = {"degradation": "armored"}
    assert degrade.capability_of(params, 0.9, "mobility") == 1.0
    assert degrade.capability_of(params, 0.4, "mobility") == 0.6
    assert degrade.capability_of(params, 0.4, "firepower") == "halved"
    assert degrade.capability_of({}, 0.1, "firepower") == "ok"
    assert degrade.capability_of(params, 0.1, "sensor") is None
    with pytest.raises(ConfigurationError, match="未知通道"):
        degrade.capability_of(params, 0.9, "crew")


# ===========================================================================
# 第二层：机动通道
# ===========================================================================


def _mover(kind: str, platform_params: dict, dmg: dict):
    """造一个带平台属性的机动件（复用 test_mover 的桩件）。"""
    mover = ComponentFactory().build("mover", kind, {})
    view = FakeView()
    view.my_damage = lambda: dmg["v"]
    mount = FakeMount(view)
    mount.platform_params = platform_params
    mover.initialize(mount)
    return mover, view


def test_ground_mover_speed_follows_the_mobility_tier() -> None:
    dmg = {"v": 1.0}
    mover, view = _mover("GROUND_MOVER", {"degradation": "armored"}, dmg)
    base = mover._speed_at(view.my_cell())
    assert base == pytest.approx(20.0)                 # 完好：全额

    dmg["v"] = 0.4                                     # 重损档
    assert mover._speed_at(view.my_cell()) == pytest.approx(20.0 * 0.6)
    dmg["v"] = 0.1                                     # 瘫痪档
    assert mover._speed_at(view.my_cell()) == pytest.approx(20.0 * 0.25)
    dmg["v"] = 1.0                                     # 修回来（纯函数无缓存）
    assert mover._speed_at(view.my_cell()) == pytest.approx(20.0)


def test_unconfigured_mover_is_bitwise_unchanged() -> None:
    dmg = {"v": 0.1}
    mover, view = _mover("GROUND_MOVER", {}, dmg)      # 没配档：挨打照跑
    assert mover._speed_at(view.my_cell()) == pytest.approx(20.0)


def test_air_mover_scales_level_speed_not_climb_rate() -> None:
    dmg = {"v": 0.4}
    mover, view = _mover("AIR_MOVER", {"degradation": "armored"}, dmg)
    assert mover._speed_at(view.my_cell()) == pytest.approx(200.0 * 0.6)
    assert float(mover.spec["climb_rate"]) == 40.0     # 垂直速率不折


# ===========================================================================
# 第三层：火力通道
# ===========================================================================


def _weapon(stock: int, dmg: dict):
    """发射方 0、目标 1（正东 30 km）。平台属性挂好后再重接线。"""
    weapon, spawned, _ = _rig(stock=stock)
    weapon._mount.platform_params = {"degradation": "armored"}
    weapon._mount.store.damage_of = lambda eid: dmg["v"]
    weapon.initialize(weapon._mount)
    return weapon, spawned


def test_crippled_weapon_rejects_fire() -> None:
    dmg = {"v": 0.1}
    weapon, spawned = _weapon(2, dmg)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert not ok and "火力瘫痪" in why
    assert spawned == [] and weapon.stock() == 2


def test_heavy_damage_doubles_ammo_per_shot() -> None:
    dmg = {"v": 0.4}
    weapon, spawned = _weapon(3, dmg)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok, why
    assert weapon.stock() == 1                         # 一发耗 2 发
    assert len(spawned) == 1

    # 剩 1 发不够一次发射（要 2 发）——拒发且不生成弹
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert not ok and "火力受损" in why and weapon.stock() == 1
    assert len(spawned) == 1


def test_intact_weapon_consumes_one_round_as_before() -> None:
    dmg = {"v": 0.9}
    weapon, spawned = _weapon(2, dmg)
    ok, why = weapon.execute("fire", {"target": "TARGET"})
    assert ok and weapon.stock() == 1 and len(spawned) == 1


# ===========================================================================
# 第四层：探测通道
# ===========================================================================


def test_sensor_penalty_scales_only_the_signal() -> None:
    radar, engine, store = _build(**_EW_RADAR)
    radar._degrade = degrade.BUILTIN_PROFILES["armored"]
    sn_full = radar.single_pulse_snr(20_000.0)
    assert sn_full > 0.0

    store.set_damage(0, 0.4)                           # 重损：−6 dB
    sn_heavy = radar.single_pulse_snr(20_000.0)
    assert sn_heavy == pytest.approx(sn_full * 10.0 ** (-0.6))

    store.set_damage(0, 0.1)                           # 瘫痪：关机
    assert radar.single_pulse_snr(20_000.0) == 0.0

    store.set_damage(0, 1.0)                           # 无缓存，立即回满
    assert radar.single_pulse_snr(20_000.0) == pytest.approx(sn_full)


def test_unconfigured_radar_is_bitwise_unchanged() -> None:
    radar, _, store = _build(**_EW_RADAR)
    sn0 = radar.single_pulse_snr(20_000.0)
    store.set_damage(0, 0.1)                           # 挨打了但没配档
    assert radar.single_pulse_snr(20_000.0) == pytest.approx(sn0)


def test_crippled_radar_sweep_does_not_tick() -> None:
    radar, engine, store = _build(**_EW_RADAR)
    radar._degrade = degrade.BUILTIN_PROFILES["armored"]
    store.set_damage(0, 0.1)
    ticks = radar.ticks
    assert radar._sweep(engine, None).__class__.__name__ == "EventResult"
    assert radar.ticks == ticks                        # 不推进相位、不出航迹


def test_radar_default_sensor_factor_is_one() -> None:
    radar, _, _ = _build(**_EW_RADAR)
    assert radar._sensor_state() == 1.0


# ===========================================================================
# 第五层：弹类禁配
# ===========================================================================


@pytest.mark.parametrize("kind", ["MISSILE_MOVER", "ANALYTIC_BALLISTIC_MOVER"])
def test_missile_movers_reject_degradation_at_assembly(kind: str) -> None:
    mover = ComponentFactory().build("mover", kind, {})
    view = FakeView()
    mount = FakeMount(view)
    mount.platform_params = {"degradation": "armored"}
    with pytest.raises(ConfigurationError, match="弹类机动件不支持"):
        mover.initialize(mount)


# ===========================================================================
# 第六层：端到端——demos/afsim_mover/degrade.txt
# ===========================================================================


def _demo_path():
    from pathlib import Path

    return Path(__file__).resolve().parent.parent / "demos" / "afsim_mover" / "degrade.txt"


def test_demo_scenario_two_hits_land_in_the_heavy_tier() -> None:
    """两发 0.5 点 / 血条 1.6：第一发后完好档、第二发后重损档，目标**存活**。

    完好度锚：1.0 → 0.6875 → 0.375（0.2 < 0.375 ≤ 0.5）。档位是纯函数：
    第二发落账后立即按重损档读数——机动 ×0.6、火力 halved、探测 −6 dB。
    """
    from milsim import models
    from milsim.services.scenario import load_scenario
    from milsim.services.type_registry import ComponentRegistry
    from milsim.simulation import Simulation

    registry = ComponentRegistry()
    models.register_framework(components=registry)
    spec = load_scenario(_demo_path().read_text(encoding="utf-8"))
    assert spec.validate() == []
    sim = Simulation(spec, components=registry)
    sim.initialize()

    sim.run_for(500_000_000)                       # 500 s > 421.5 s 全程
    target_id = sim.registry.by_name("TGT_DEGRADE").entity_id
    intact = sim.store.damage_of(target_id)
    assert intact == pytest.approx(0.375)
    assert sim.store.is_alive(target_id)           # 重损 ≠ 摧毁

    params = sim.registry.get(target_id).platform_params
    assert degrade.capability_of(params, intact, "mobility") == pytest.approx(0.6)
    assert degrade.capability_of(params, intact, "firepower") == "halved"
    assert degrade.capability_of(params, intact, "sensor") == pytest.approx(
        10.0 ** (-0.6)
    )
    # 两发都起爆了：两个发射平台都注销。
    assert sim.registry.by_name("SRBM_1") is None
    assert sim.registry.by_name("SRBM_2") is None


def test_hex_radar_without_platform_params_still_initializes() -> None:
    """简装桩（无 platform_params）= 无平台属性，装配照常。"""
    radar, _, _ = _build(**_EW_RADAR)
    assert radar._degrade is None
