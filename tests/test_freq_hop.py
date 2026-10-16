"""跳频（捷变频）的账：``F_BW`` 的跳频平均口径（v0.13.64，§5.14.15）。

三层：

1. :func:`~milsim.models.percep.equation.hop_jamming_overlap` —— 纯函数。
   守四条**自洽性**：不跳频/跟随 ⇒ 逐位老口径；跳频带=瞬时带 ⇒ 退化；
   盖满跳频带的阻塞式 ⇒ 与"阻塞式对单频点"同一个数；谱在跳频带外 ⇒ 0；
2. 装配校验 —— 跳频带装不下瞬时带就报错，不静默；
3. 组件级 —— CW 瞄准干扰的 J 按 ``B_r/B_hop`` 缩、跟随干扰机逐位不变、
   波长与 SNR 不受跳频影响（捷变频只进 J 账）。

★ 偏中心 30 MHz 的 CW 干扰机是本文件最有味道的一条：老口径下它**一分不进**
（接收带 ±0.5 MHz），跳频开着它却**进得来**（跳频带 ±50 MHz）——这正是
"跳频挡不住知道带段的干扰机、只挡得住赌单点的干扰机"的反面：跳频**扩大**
了"能撞上的频段"，代价是每拍只吃到 ``B_r/B_hop`` 份功率。两头都要有人守。
"""

from __future__ import annotations

import pytest

from milsim.errors import ConfigurationError
from milsim.models.percep import equation as eq
from test_percep import _EW_RADAR, _build, _ew_build, _jammer_spec, _range_between

BR = 1.0e6          # _EW_RADAR 的接收带宽（Hz）
BHOP = 100.0e6      # 跳频带（Hz）：±50 MHz


# ===========================================================================
# 第一层：equation（纯函数）
# ===========================================================================


def test_hop_off_is_bitwise_the_old_ratio() -> None:
    """不跳频（hop=0）⇒ 与 :func:`eq.bandwidth_overlap_ratio` **逐位相同**。"""
    cases = [
        (4.0e9, 1.0e6),      # 瞄准式居中
        (4.0e9, 400.0e6),    # 阻塞式盖满
        (4.5e9, 0.0),        # CW 完全错开
        (3.999e9, 0.0),      # CW 带内
        (3.5e9, 50.0e6),     # 阻塞式错开
    ]
    for f_j, b_j in cases:
        assert (
            eq.hop_jamming_overlap(f_j, b_j, 4.0e9, 0.0, BR, False)
            == eq.bandwidth_overlap_ratio(f_j, b_j, 4.0e9, BR)
        )


def test_follow_degenerates_to_the_single_frequency_case() -> None:
    """``hop_follow=True`` ⇒ 跟得上每一拍，跳频对它**无效**（逐位老口径）。"""
    for f_j, b_j in [(4.0e9, 1.0e6), (4.03e9, 0.0), (3.8e9, 400.0e6)]:
        assert (
            eq.hop_jamming_overlap(f_j, b_j, 4.0e9, BHOP, BR, True)
            == eq.bandwidth_overlap_ratio(f_j, b_j, 4.0e9, BR)
        )


def test_spot_cw_at_the_center_averages_to_br_over_bhop() -> None:
    """居中 CW：随机的驻留只有 ``B_r/B_hop`` 的概率撞上 ⇒ E[F_BW] 就是它。"""
    assert eq.hop_jamming_overlap(4.0e9, 0.0, 4.0e9, BHOP, BR, False) == (
        pytest.approx(BR / BHOP)
    )


def test_blocker_covering_the_hop_band_keeps_the_old_ratio() -> None:
    """盖满跳频带的阻塞式 = "不挑频点的干扰"，跳频奈何不了它 ⇒ 数不变。"""
    old = eq.bandwidth_overlap_ratio(4.0e9, 400.0e6, 4.0e9, BR)
    assert old == pytest.approx(BR / 400.0e6)          # 先钉住它是阻塞式
    assert eq.hop_jamming_overlap(
        4.0e9, 400.0e6, 4.0e9, BHOP, BR, False
    ) == pytest.approx(old, rel=1e-12)


def test_partial_overlap_scales_by_the_covered_fraction() -> None:
    """谱搭住跳频带**边缘**：线性升起一段 + 带内平台一段，解析可积、逐位对照。

    干扰谱 [4.045, 4.065] GHz，跳频带 [3.95, 4.05] GHz：滑窗右缘在
    f ∈ (4.0445, 4.0455] GHz 线性扫入谱（三角形 ``(1 MHz)²/2``），此后到带
    上沿滑窗整个落在谱里（平台 ``1 MHz × 4.5 MHz``）。
    """
    got = eq.hop_jamming_overlap(4.055e9, 20.0e6, 4.0e9, BHOP, BR, False)
    exact = ((BR ** 2) / 2.0 + BR * (4.05e9 - (4.045e9 + BR / 2))) / (
        BHOP * 20.0e6
    )
    assert got == pytest.approx(exact)


def test_interior_spectrum_pays_br_over_bhop_whatever_its_width() -> None:
    """谱整体在跳频带内部（离边 ≥ B_r/2）⇒ E = B_r/B_hop，**与 B_j 无关**。"""
    for b_j in (0.0, 1.0e6, 20.0e6, 60.0e6):
        assert eq.hop_jamming_overlap(
            4.0e9, b_j, 4.0e9, BHOP, BR, False
        ) == pytest.approx(BR / BHOP)


def test_spectrum_outside_the_hop_band_is_zero() -> None:
    """谱完全在跳频带外 ⇒ 一拍都撞不上 ⇒ 0（老口径在带外也该是 0）。"""
    assert eq.hop_jamming_overlap(4.5e9, 0.0, 4.0e9, BHOP, BR, False) == 0.0
    assert eq.hop_jamming_overlap(4.5e9, 20.0e6, 4.0e9, BHOP, BR, False) == 0.0


def test_hop_band_equal_to_the_receiver_band_degenerates() -> None:
    """``B_hop == B_r`` 合法但退化：逐位回到老口径（装配校验放行它）。"""
    assert eq.hop_jamming_overlap(
        4.0e9, 0.0, 4.0e9, BR, BR, False
    ) == eq.bandwidth_overlap_ratio(4.0e9, 0.0, 4.0e9, BR)


# ===========================================================================
# 第二层：装配校验
# ===========================================================================


def test_hop_band_must_contain_the_receiver_bandwidth() -> None:
    """跳频带装不下瞬时带 ⇒ 装配报错（"跳出来的每一拍比接收机还窄"）。"""
    with pytest.raises(ConfigurationError, match="不得小于接收机带宽"):
        _build(frequency=4.0e9, bandwidth=10.0e6, hop_bandwidth=1.0e6)


def test_hop_band_needs_a_receiver_bandwidth() -> None:
    """写了跳频、没写带宽（bandwidth=0 也没法由 1/τ 推）⇒ 报错，不静默。"""
    with pytest.raises(ConfigurationError, match="先补 bandwidth"):
        _build(hop_bandwidth=1.0e6)


def test_hop_defaults_to_off_and_old_scenarios_pass() -> None:
    """默认 0 = 不跳频：老想定照常装配，``hop_bandwidth_hz`` 就是 0。"""
    radar, _, _, _ = _ew_build(**_EW_RADAR)
    assert radar.hop_bandwidth_hz == 0.0


# ===========================================================================
# 第三层：组件级（J 账与链路不变性）
# ===========================================================================


def test_spot_jamming_power_scales_by_br_over_bhop() -> None:
    """居中 CW 干扰机：跳频开 ⇒ J 恰好缩成 ``B_r/B_hop`` 倍（其余因子全消）。"""
    radar0, _, store, jam = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {}),), **_EW_RADAR
    )
    _, bearing = _range_between(store, 2)
    j0 = radar0.received_jamming_w(bearing)
    assert j0 > 0.0

    jam.register(_jammer_spec(2))     # 同一台干扰机、同一份名册
    radar1, engine, store1, jam1 = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    _, bearing1 = _range_between(store1, 2)
    j1 = radar1.received_jamming_w(bearing1)
    assert j1 == pytest.approx(j0 * BR / BHOP, rel=1e-9)


def test_follow_jammer_is_bitwise_unchanged_by_hopping() -> None:
    """``hop_follow=True`` 的干扰机：跳频开着 J 也**逐位**等于不跳频。"""
    radar0, _, store, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {}),), **_EW_RADAR
    )
    _, bearing = _range_between(store, 2)
    j0 = radar0.received_jamming_w(bearing)

    radar1, _, store1, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"hop_follow": True}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    _, bearing1 = _range_between(store1, 2)
    assert radar1.received_jamming_w(bearing1) == j0


def test_off_center_cw_only_bites_when_hopping() -> None:
    """偏 30 MHz 的 CW：老口径**一分不进**（±0.5 MHz），跳频开着进得来。"""
    radar0, _, store, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"frequency_hz": 4.03e9}),),
        **_EW_RADAR,
    )
    _, bearing = _range_between(store, 2)
    assert radar0.received_jamming_w(bearing) == 0.0

    radar1, _, store1, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"frequency_hz": 4.03e9}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    _, bearing1 = _range_between(store1, 2)
    assert radar1.received_jamming_w(bearing1) > 0.0


def test_wavelength_and_snr_are_unaffected_by_hopping() -> None:
    """捷变频只进 J 账：波长、标定 SNR、噪声逐位不变——不是"换了一部雷达"。"""
    radar0, _, _, _ = _ew_build(**_EW_RADAR)
    radar1, _, _, _ = _ew_build(hop_bandwidth=BHOP, **_EW_RADAR)
    assert radar1.wavelength_m == radar0.wavelength_m
    assert radar1.snr_at_reference == radar0.snr_at_reference
    assert radar1.noise_w == radar0.noise_w
    assert radar1.bandwidth_hz == radar0.bandwidth_hz


def test_the_shared_judge_serves_both_views() -> None:
    """``_jam_band_overlap`` 对名册条目与威胁行给**同一个数**——两支共用由此钉住。"""
    radar, _, store, jam = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"frequency_hz": 4.03e9}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    spec = _jammer_spec(2, frequency_hz=4.03e9)
    by_helper = radar._jam_band_overlap(spec)
    assert by_helper == pytest.approx(BR / BHOP)

    # 威胁行（JamService._threat 的产物）与名册条目同值：
    row = next(
        t for t in jam.on_roster(0)
        if t.entity_id == 2
    )
    assert radar._jam_band_overlap(row) == by_helper

    # 跟随 ⇒ 退化为老口径 = 0（30 MHz 在 ±0.5 MHz 接收带外）。
    # 这一行来自 on_roster（off_beam_deg=None）⇒ 截获闸门不判 ⇒ 跟随照算。
    assert radar._jam_band_overlap(
        _jammer_spec(2, frequency_hz=4.03e9, hop_follow=True)
    ) == 0.0


# ===========================================================================
# 第四层：实时路线的截获闸门——"主瓣照到才跟"（两条知识路线）
# ===========================================================================


def test_follow_is_gated_by_mainlobe_intercept() -> None:
    """实时路线的闸门：雷达波束没照到干扰机 ⇒ 退回档案路线（按不跟随算）。

    同一台干扰机（hop_follow=True、CW 居中）两种指向：
    波束正对它（主瓣截获）⇒ 跟随生效，逐位 = 不跳频；
    波束偏开 10°（2° 波束盖不住）⇒ 退回档案路线——此时接收增益也换副瓣，
    于是与"同指向的不跟随干扰机"**逐位同一个数**（闸门只是关掉跟随，
    不另外扣账）。
    """
    radar0, _, store, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {}),), **_EW_RADAR
    )
    _, bearing = _range_between(store, 2)
    j_full = radar0.received_jamming_w(bearing)
    assert j_full > 0.0

    # 主瓣截获 ⇒ 跟随生效
    radar_ok, _, store_ok, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"hop_follow": True}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    _, bearing_ok = _range_between(store_ok, 2)
    assert radar_ok.received_jamming_w(bearing_ok) == j_full

    # 波束偏开 10°：跟随与不跟随逐位同一个数（都走档案口径 + 副瓣接收）
    away = (bearing + 10.0) % 360.0
    radar_follow, _, store_f, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"hop_follow": True}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    radar_nofollow, _, store_n, _ = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    _, _ = _range_between(store_n, 2)
    assert radar_follow.received_jamming_w(away) == radar_nofollow.received_jamming_w(away)
    # 且闸门真的关了：偏开方向若跟随生效，会多出 B_hop/B_r 倍的功率
    assert radar_follow.received_jamming_w(away) < j_full * BR / BHOP


def test_false_target_path_keeps_follow_when_no_beam_context() -> None:
    """名册出口（off_beam_deg=None）⇒ 截获闸门不判 ⇒ 跟随照算（同一条边界）。"""
    radar, _, store, jam = _ew_build(
        entities=((2, 6, 0),), jammers=((2, {"hop_follow": True}),),
        hop_bandwidth=BHOP, **_EW_RADAR,
    )
    row = next(t for t in jam.on_roster(0) if t.entity_id == 2)
    assert row.off_beam_deg is None
    # 跟随生效 ⇒ 老口径；CW 居中在接收带内 ⇒ 全额 1
    assert radar._jam_band_overlap(row) == 1.0
