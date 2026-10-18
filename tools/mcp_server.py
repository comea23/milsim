"""milsim MCP server —— 把推演链路（想定校验 → 运行 → 指标计算）封装成 MCP 工具。

三个工具：
- ``milsim_validate_scenario``  加载 + 装配 + 初始化校验，返回实体清单与初始位姿。
- ``milsim_run_scenario``       跑想定，按采样间隔回各实体轨迹与末态。
- ``milsim_compute_metrics``    ``tools/cmp_metrics`` 算法库封装（RMSE/Fréchet/Jaccard/TIC/…）。

运行方式（stdio，由 MCP 客户端拉起，不手动运行）::

    <venv-python> tools/mcp_server.py

已在 ``~/.workbuddy/mcp.json`` 注册为 ``milsim``。所有工具返回
``{"ok": True, ...}`` 或 ``{"ok": False, "error": "..."}``，不抛异常到协议层。
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path

#: milsim 仓库根（tools/ 的上一级）。
ROOT = Path(__file__).resolve().parents[1]
for _p in (str(ROOT), str(ROOT / "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mcp.server.mcpserver import MCPServer

from tools.cmp_metrics import (  # noqa: E402
    align_pairs,
    binary_match,
    dtw_distance,
    frechet_distance,
    hausdorff,
    jaccard_score,
    mae,
    max_ae,
    mod_hausdorff,
    rmae,
    rmse,
    rrmse,
    smc_score,
    tic,
)

server: MCPServer = MCPServer("milsim")

#: 运行参数防呆上限。
MAX_DURATION_S = 7200.0
MAX_POINTS = 5000

_ALIGN_METRICS = {"rmse", "mae", "max_ae", "tic", "rrmse", "rmae"}
_POINT_METRICS = {"frechet", "dtw", "hausdorff", "mod_hausdorff"}
_FLAG_METRICS = {"jaccard", "smc", "binary_match"}


def _ok(**kw) -> dict:
    return {"ok": True, **kw}


def _err(exc: Exception) -> dict:
    return {"ok": False, "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(limit=4)}


def _load_sim(path: str):
    """加载 + 装配 + 初始化，返回 (sim, {name: entity_id})。"""
    from milsim.simulation import Simulation

    sim = Simulation.from_scenario_file(path)
    sim.build()
    sim.initialize()
    named = {}
    for eid in sim.registry.all_ids():
        re_ = sim.registry.get(eid)
        if re_ is not None:
            named[re_.name] = eid
    return sim, named


@server.tool(
    name="milsim_validate_scenario",
    description="校验 milsim 想定：加载、装配、初始化；返回实体清单（名称/阵营/类型）与初始位姿。不推进时间。",
)
def validate_scenario(path: str) -> dict:
    try:
        sim, named = _load_sim(path)
    except Exception as exc:  # noqa: BLE001 —— 校验工具的本职就是把异常带回去
        return _err(exc)
    ents = []
    for name, eid in named.items():
        pose = sim.store.pose_of(eid)
        ents.append({
            "name": name,
            "side": sim.registry.side_of(eid),
            "type": sim.registry.type_of(eid),
            "pose": None if pose is None else [round(v, 3) for v in pose],
        })
    return _ok(n_entities=len(ents), entities=ents)


@server.tool(
    name="milsim_run_scenario",
    description="运行 milsim 想定 duration_s 秒，每 sample_s 秒采样一次各实体 (t,x,y,z,heading,speed)。"
                "返回逐实体轨迹（超 max_points 自动等距降采样）与末态。实体中途消亡后停止采样该实体。",
)
def run_scenario(path: str, duration_s: float = 120.0, sample_s: float = 1.0,
                 max_points: int = 2000) -> dict:
    try:
        if not (0.01 <= sample_s and duration_s <= MAX_DURATION_S):
            return {"ok": False, "error": "约束：0.01≤sample_s；duration_s≤7200"}
        max_points = max(2, min(int(max_points), MAX_POINTS))
        sim, named = _load_sim(path)
        n_steps = max(1, int(round(duration_s / sample_s)))
        # 降采样步长：步数超上限时按等距抽点
        keep = max(1, n_steps // max_points)
        tracks: dict[str, list] = {name: [] for name in named}
        dead: set[str] = set()
        for i in range(1, n_steps + 1):
            sim.run_for(int(sample_s * 1e6))
            if (i - 1) % keep:
                continue
            t = sim.engine.now / 1e6
            for name, eid in named.items():
                if name in dead:
                    continue
                pose = sim.store.pose_of(eid)
                if pose is None:
                    dead.add(name)
                    continue
                tracks[name].append([round(t, 3)] + [round(v, 3) for v in pose])
        ends = {}
        for name, eid in named.items():
            pose = sim.store.pose_of(eid)
            ends[name] = None if pose is None else [round(v, 3) for v in pose]
        return _ok(duration_s=sim.engine.now / 1e6,
                   sample_s=sample_s, keep_every=keep,
                   dead=sorted(dead), tracks=tracks, end_poses=ends)
    except Exception as exc:  # noqa: BLE001
        return _err(exc)


@server.tool(
    name="milsim_compute_metrics",
    description="cmp_metrics 相似性指标。标量时间序列（先 align 插值对齐再算）："
                "rmse/mae/max_ae/tic/rrmse/rmae，传 a_times,a_values,b_times,b_values；"
                "二维点列（原始点列，不插值）：frechet/dtw/hausdorff/mod_hausdorff，传 a_points,b_points；"
                "0/1 事件序列：jaccard/smc/binary_match，传 a_values,b_values。",
)
def compute_metrics(metric: str,
                    a_times: list[float] | None = None,
                    a_values: list[float] | None = None,
                    b_times: list[float] | None = None,
                    b_values: list[float] | None = None,
                    a_points: list[list[float]] | None = None,
                    b_points: list[list[float]] | None = None,
                    step_s: float = 1.0) -> dict:
    try:
        metric = metric.lower()
        if metric in _ALIGN_METRICS:
            if not (a_times and a_values and b_times and b_values):
                return {"ok": False, "error": f"{metric} 需要 a_times/a_values/b_times/b_values"}
            _t, la, lb = align_pairs(a_times, a_values, b_times, b_values, step_s)
            n = len(la)
            if not n:
                return {"ok": False, "error": "对齐后无公共样本"}
            fn = {"rmse": rmse, "mae": mae, "max_ae": max_ae, "tic": tic,
                  "rrmse": rrmse, "rmae": rmae}[metric]
            return _ok(metric=metric, n=n, value=fn(lb, la))
        if metric in _POINT_METRICS:
            if not (a_points and b_points):
                return {"ok": False, "error": f"{metric} 需要 a_points/b_points"}
            A = [tuple(p) for p in a_points]
            B = [tuple(p) for p in b_points]
            if metric == "frechet":
                val = frechet_distance(A, B)
            elif metric == "dtw":
                val = dtw_distance(A, B)
            elif metric == "hausdorff":
                val = hausdorff(A, B)
            else:
                val = mod_hausdorff(A, B)
            return _ok(metric=metric, n_a=len(A), n_b=len(B), value=val)
        if metric in _FLAG_METRICS:
            if a_values is None or b_values is None:
                return {"ok": False, "error": f"{metric} 需要 a_values/b_values（0/1 序列）"}
            if metric == "jaccard":
                return _ok(metric="jaccard", value=jaccard_score(a_values, b_values))
            if metric == "smc":
                return _ok(metric="smc", value=smc_score(a_values, b_values))
            m = binary_match(a_values, b_values)
            n = len(a_values)
            return _ok(metric="binary_match", n=n, **m)
        return {"ok": False, "error": f"未知 metric={metric!r}；可选："
                + ", ".join(sorted(_ALIGN_METRICS | _POINT_METRICS | _FLAG_METRICS))}
    except Exception as exc:  # noqa: BLE001
        return _err(exc)


if __name__ == "__main__":
    server.run()  # stdio
