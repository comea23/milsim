"""对比算法库（tools/cmp_metrics）测试——算例数值与《算法库方法梳理_四类》
文档的示例逐一交叉核对；等价类与边界另补。"""

import math

import pytest

from tools.cmp_metrics import (
    align_pairs,
    binary_match,
    directed_hausdorff,
    dtw_distance,
    frechet_distance,
    frechet_similarity,
    hausdorff,
    jaccard_score,
    mae,
    max_ae,
    mape,
    mod_hausdorff,
    mse,
    proportion_ztest,
    resample,
    rmae,
    rmse,
    rrmse,
    smc_score,
    spearman_rho,
    tic,
)

# 文档 2.1 算例：基准 A、预测 B（误差 e = B - A）
A = [100, 102, 98, 105, 110, 108, 112, 115]
B = [101, 99, 97, 106, 112, 107, 110, 116]


class TestTimeDomain:
    def test_doc_2_1(self):
        # 文档：MAE=1.5、RMSE=1.6583、MAPE=1.4142%、ρ=0.9524
        assert mae(B, A) == pytest.approx(1.5)
        # 文档示例表 MSE=2.125 是笔误：22/8=2.75，RMSE=sqrt(2.75)=1.6583
        assert mse(B, A) == pytest.approx(2.75)
        assert rmse(B, A) == pytest.approx(math.sqrt(2.75))
        assert max_ae(B, A) == 3
        assert mape(B, A) == pytest.approx(1.4142, abs=1e-3)
        assert spearman_rho(A, B) == pytest.approx(1 - 24 / 504)  # 0.95238...

    def test_spearman_reversed(self):
        assert spearman_rho([1, 2, 3, 4], [4, 3, 2, 1]) == pytest.approx(-1.0)
        assert spearman_rho([1, 2, 3, 4], [1, 2, 3, 4]) == pytest.approx(1.0)

    def test_spearman_ties_average_rank(self):
        # 秩 a=[1.5,1.5,3]、b=[1,2,3] 的 Pearson：
        # num=1.5, da=sqrt(1.5), db=sqrt(2) → 1.5/sqrt(3)=0.8660
        rho = spearman_rho([1, 1, 2], [1, 2, 3])
        assert rho == pytest.approx(0.8660254037844387)

    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError):
            mae([1, 2], [1])

    def test_mape_zero_ref_raises(self):
        with pytest.raises(ValueError):
            mape([1, 2], [0, 2])

    def test_cosine(self):
        from tools.cmp_metrics import cosine_similarity

        assert cosine_similarity([1, 0], [2, 0]) == pytest.approx(1.0)
        assert cosine_similarity([1, 0], [0, 1]) == pytest.approx(0.0)


class TestDTW:
    def test_doc_2_2(self):
        X = [0, 1, 2, 3, 4]
        Y = [0, 1, 1, 2, 3, 4]  # 形状相同, 中段多耗一拍
        Z = [0, 1, 2.5, 3, 4]  # 节奏相同, 第 3 点偏高 0.5
        total, avg = dtw_distance(X, Y)
        assert total == pytest.approx(0.0)
        assert avg == pytest.approx(0.0)
        total, _ = dtw_distance(X, Z)
        assert total == pytest.approx(0.5)

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            dtw_distance([], [1])

    def test_self_distance_zero(self):
        total, _ = dtw_distance([3, 1, 4], [3, 1, 4])
        assert total == pytest.approx(0.0)


class TestFrechet:
    def test_doc_2_3(self):
        # 平行偏移 1 的两条 5 点平行线
        A5 = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        B5 = [(0, 1), (1, 1), (2, 1), (3, 1), (4, 1)]
        f = frechet_distance(A5, B5)
        assert f == pytest.approx(1.0)
        assert frechet_similarity(f) == pytest.approx(1 / (1 + math.exp(5)))
        # 偏移 0.5 → sim=0.0759（文档对照）
        A_half = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        B_half = [(0, 0.5), (1, 0.5), (2, 0.5), (3, 0.5), (4, 0.5)]
        assert frechet_similarity(frechet_distance(A_half, B_half)) == pytest.approx(
            0.0759, abs=1e-4
        )

    def test_identical(self):
        pts = [(0, 0), (1, 1), (2, 0)]
        assert frechet_distance(pts, pts) == pytest.approx(0.0)
        # 库口径：完全重合 sim 也只有 0.5
        assert frechet_similarity(0.0) == pytest.approx(0.5)

    def test_reversed_sawtooth(self):
        # 文档 2.10 情形 2：点集全同、时序倒序 → 保序方法给 4.0
        A5 = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]
        assert frechet_distance(A5, A5[::-1]) == pytest.approx(4.0)


class TestRelative:
    def test_tic_zero_on_identical(self):
        assert tic([1, 2, 3], [1, 2, 3]) == pytest.approx(0.0)

    def test_tic_bounds_and_scale(self):
        ref = [10, 20, 30, 40]
        pred = [12, 18, 33, 41]
        u = tic(pred, ref)
        assert 0.0 < u < 1.0
        # 幅值整体放大 10 倍：U 只与 k 有关, 不随量级跑
        assert tic([10 * x for x in pred], [10 * x for x in ref]) == pytest.approx(u)

    def test_tic_all_zero_raises(self):
        with pytest.raises(ValueError):
            tic([0, 0], [0, 0])

    def test_rmae(self):
        # MAE=1.5, mean|ref|=106.25 → 0.014118
        assert rmae(B, A) == pytest.approx(1.5 / (sum(map(abs, A)) / 8))

    def test_rmae_zero_ref_raises(self):
        with pytest.raises(ValueError):
            rmae([1, 1], [0, 0])

    def test_rrmse(self):
        v = rrmse(B, A, denom="mean")
        assert v == pytest.approx(math.sqrt(2.75) / (sum(A) / 8))
        v2 = rrmse(B, A, denom="rms")
        assert v2 == pytest.approx(math.sqrt(2.75) / math.sqrt(sum(x * x for x in A) / 8))
        with pytest.raises(ValueError):
            rrmse([1], [1], denom="bad")


class TestHausdorff:
    A5 = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]

    def test_doc_2_10_case1(self):
        base = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        off = [(x, 1) for x in range(5)]
        assert directed_hausdorff(base, off) == pytest.approx(1.0)
        assert hausdorff(base, off) == pytest.approx(1.0)

    def test_doc_2_10_case2_not_order_preserving(self):
        # 点集全同、时序倒序 → H=0（不保序的实证）
        assert hausdorff(self.A5, self.A5[::-1]) == pytest.approx(0.0)

    def test_mod(self):
        base = [(i * 0.1, 0.0) for i in range(10)] + [(2.0, 5.0)]
        off = [(i * 0.1, 0.5) for i in range(11)]
        h = hausdorff(base, off)
        m = mod_hausdorff(base, off, q=0.95)
        assert m < h  # 缓解单点支配
        assert mod_hausdorff(base, off, q=1.0) == pytest.approx(h)  # 100% 分位=max


class TestBinary:
    def test_doc_2_13(self):
        base = [1, 0, 1, 1, 0, 1, 0, 0, 1, 1]
        pred = [1, 0, 1, 1, 0, 1, 0, 0, 1, 0]
        m = binary_match(base, pred)
        assert m == {"a": 5, "b": 1, "c": 0, "d": 4}
        assert jaccard_score(base, pred) == pytest.approx(5 / 6)
        assert smc_score(base, pred) == pytest.approx(0.9)

    def test_no_event_both(self):
        assert jaccard_score([0, 0], [0, 0]) == 1.0
        assert smc_score([0, 0], [0, 0]) == 1.0

    def test_reversed_gives_low_jaccard(self):
        # 率相同但模式完全错开: a=0
        base = [1, 1, 0, 0]
        pred = [0, 0, 1, 1]
        assert jaccard_score(base, pred) == 0.0
        assert smc_score(base, pred) == 0.0

    def test_nonbinary_raises(self):
        with pytest.raises(ValueError):
            binary_match([1, 2], [1, 0])


class TestProportionZ:
    def test_doc_2_14(self):
        z, p, ok = proportion_ztest(70, 100, 0.6)
        assert z == pytest.approx(2.0412, abs=1e-3)
        assert p == pytest.approx(0.0412, abs=1e-3)
        assert ok is True

    def test_small_sample_flag(self):
        _, _, ok = proportion_ztest(3, 10, 0.5)
        assert ok is False  # 10*0.25=2.5 < 5

    def test_perfect_match_zero_z(self):
        z, p, _ = proportion_ztest(60, 100, 0.6)
        assert z == pytest.approx(0.0)
        assert p == pytest.approx(1.0)


class TestAlign:
    def test_resample_linear(self):
        t, v = resample([0.0, 2.0], [0.0, 10.0], dt=1.0)
        assert t == [0.0, 1.0, 2.0]
        assert v == pytest.approx([0.0, 5.0, 10.0])

    def test_resample_no_extrapolation(self):
        with pytest.raises(ValueError):
            resample([1.0, 2.0], [0.0, 1.0], dt=0.5, t0=0.5)

    def test_align_pairs_common_window(self):
        # A: 0..4s, B: 1..3.5s → 共同窗 [1, 3.5], dt=0.5
        ta = [0.0, 1.0, 2.0, 3.0, 4.0]
        va = [0.0, 1.0, 2.0, 3.0, 4.0]
        tb = [1.0, 2.0, 3.0, 3.5]
        vb = [10.0, 20.0, 30.0, 35.0]
        t, a, b = align_pairs(ta, va, tb, vb, dt=0.5)
        assert t[0] == pytest.approx(1.0)
        assert t[-1] == pytest.approx(3.5)
        assert len(t) == len(a) == len(b) == 6
        assert a == pytest.approx([1.0, 1.5, 2.0, 2.5, 3.0, 3.5])
        assert b == pytest.approx([10.0, 15.0, 20.0, 25.0, 30.0, 35.0])

    def test_align_pairs_disjoint_raises(self):
        with pytest.raises(ValueError):
            align_pairs([0, 1], [0, 1], [5, 6], [0, 1], dt=0.5)

    def test_non_increasing_time_raises(self):
        with pytest.raises(ValueError):
            resample([0.0, 1.0, 1.0], [0, 1, 2], dt=0.5)
