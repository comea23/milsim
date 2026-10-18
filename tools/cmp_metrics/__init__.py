"""对比算法库（milsim ↔ AFSIM 输出相似性）。

编号对齐《算法库方法梳理_四类》文档（资料库 2.A/2.B 分类），一算法一文件：

=====  ============================  ==============================
编号   文件                          一句话用途
=====  ============================  ==============================
2.1    ``time_domain``               等长对齐序列逐点误差（MAE/MSE/RMSE/MaxAE/MAPE/余弦/斯皮尔曼）
2.2    ``dtw``                       动态时间规整（长度不等、节奏不同）
2.3    ``frechet``                   离散弗雷歇距离（顺序约束下的最坏偏差）+ sigmoid 相似度
2.7    ``tic``                       Theil 不等式系数（RMSE 归一化到 [0,1]）
2.8    ``rmae``                      相对平均绝对误差
2.9    ``rrmse``                     相对均方根误差
2.10   ``hausdorff``                 Hausdorff 距离（含 MOD 分位数变体）
2.13   ``jaccard``                   二值 0/1 向量匹配（Jaccard/SMC + 漏报/虚警）
2.14   ``proportion_z``              比例 Z 检验（事件率 vs 基准率）
—      ``align``                     时间轴插值对齐（RMSE 类等长指标的前置）
=====  ============================  ==============================

全部标准库实现（math 仅此一项依赖），零第三方包。详细说明见同目录 README.md。
"""

from .time_domain import (
    mae,
    mse,
    rmse,
    max_ae,
    mape,
    cosine_similarity,
    spearman_rho,
)
from .dtw import dtw_distance
from .frechet import frechet_distance, frechet_similarity
from .tic import tic
from .rmae import rmae
from .rrmse import rrmse
from .hausdorff import (
    directed_hausdorff,
    hausdorff,
    mod_hausdorff,
)
from .jaccard import binary_match, jaccard_score, smc_score
from .proportion_z import proportion_ztest
from .align import resample, align_pairs

__all__ = [
    "mae", "mse", "rmse", "max_ae", "mape", "cosine_similarity", "spearman_rho",
    "dtw_distance",
    "frechet_distance", "frechet_similarity",
    "tic", "rmae", "rrmse",
    "directed_hausdorff", "hausdorff", "mod_hausdorff",
    "binary_match", "jaccard_score", "smc_score",
    "proportion_ztest",
    "resample", "align_pairs",
]
