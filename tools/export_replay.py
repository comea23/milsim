"""跑一份想定，把**逐拍态势**导出成前端可读的回放 JSON（离线那一半）。

用法::

    python tools/export_replay.py scenarios/comm_track_jam.txt --seconds 60
    python tools/export_replay.py scenarios/comm_track_jam.txt --seconds 480 \
        --out app/replay.json
    python tools/export_replay.py scenarios/patrol.txt --seconds 120 --library library/demo_models.db
    # 会动的想定（机动命令运行期下达，见 --demo）
    python tools/export_replay.py scenarios/mover_demo.txt --demo mover \
        --seconds 600 --sample 5

产物是一份 ``replay.json``：**每帧 = 该时刻的一份全量态势**，含四条图层所需
的全部数据——实体（含经纬度）、航迹、通信拓扑（节点 + 边）、干扰关系（谁压谁）。
前端拿到它就能离线回放，不需要跑 Python。

**另一条出口**：想边跑边看用 ``tools/serve_replay.py``（实时推流）。两条路
**共用同一份帧定义**（``milsim.services.replay``），所以"离线看到的"与
"实时看到的"不会各自演化成两回事。本文件只管"跑完写文件"。

★ 两种想定，两套采样节拍
------------------------
想定分两类，取样步长不能一刀切：

* **有事件的想定**（侦察/通信/干扰：``comm_track_jam``、``patrol``）
  每拍都可能改变态势，步长按想定的 ``max_step``（2 s）走，宁可多出帧。
* **运动学想定**（``mover_demo``：几辆车几百秒才跑完十几公里）
  想定的 ``max_step`` 是 2 s，若照它采样，600 s 就是 300 帧、每帧几 KB，
  一份回放上兆——而位置在这 2 s 里只挪了几十米，**看不出差别**。
  所以加 ``--sample``（秒）单独定采样粒度：帧间隔 = ``--sample``，
  引擎内部照旧按 ``max_step`` 推（两者解耦，见 ``demo_mover.SAMPLE_US``
  的同一条理由：采样粒度与模型节拍不是一回事）。

★ 想定里没有机动命令怎么办
--------------------------
``mover_demo.txt`` 刻意**不写**初始机动命令（目的地是运行期下达的东西，
否则"想定里的初始条件"与"推演中的命令"会有两处真相）。要让它动起来，
用 ``--demo mover``：本工具照 ``tools/demo_mover.py`` 的做法，在装配完成后
选一个"最需要绕行"的目的地并逐件下达命令，然后才开始采样。**这段逻辑只跑
在导出侧，不落进想定文件**——与 demo 那份保持同一套选择规则（同一份想定
两次跑出的图完全一样）。

帧里的东西、坐标怎么换算、两条口径坑
------------------------------------
见 :mod:`milsim.services.replay` 的模块 docstring——那里写了一次，
两条出口都从那取。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import demo_components  # noqa: E402
from milsim.errors import MilsimError  # noqa: E402
from milsim.models import register_framework  # noqa: E402
from milsim.services import replay as replay_mod  # noqa: E402
from milsim.services.input.lexer import LexError, format_error  # noqa: E402
from milsim.services.situation import format_sim_time  # noqa: E402
from milsim.services.type_registry import (  # noqa: E402
    ComponentRegistry,
    PlatformRegistry,
)
from milsim.simulation import Simulation  # noqa: E402

#: 帧定义在 ``milsim.services.replay``——**离线导出与实时推流共用同一份**。
#: 本文件只负责"跑完写文件"这一半。
FORMAT = replay_mod.FORMAT

#: ``--demo mover`` 挑目的地用的等角候选数与半径（格）。照 ``demo_mover.py``
#: 的取值——两者都是"起点周围一圈 12 个方向、约 9 km（格中心距 = 边长 × √3）"。
#: 取值一致才能保证"回放里的路线"与"demo 表里的路线"是同一条。
_CANDIDATE_DIRECTIONS = 12
_CANDIDATE_RADIUS_CELLS = 26


def parse_args(argv: list[str]) -> tuple[str, float, str, list[str], float, str]:
    """解析命令行，返回 ``(想定, 秒数, 输出, 库, 采样秒, demo 名)``。

    ``采样秒`` 为 0 ⇒ 用想定的 ``max_step``；``demo 名`` 空 ⇒ 不下达任何
    运行期命令（想定自带初始机动的话照跑）。
    """
    if len(argv) < 2:
        print(__doc__.strip(), file=sys.stderr)
        raise SystemExit(2)

    path = argv[1]
    seconds = 60.0
    out = ""
    libraries: list[str] = []
    sample = 0.0
    demo = ""

    rest = argv[2:]
    index = 0
    while index < len(rest):
        flag = rest[index]
        if flag == "--seconds" and index + 1 < len(rest):
            seconds = float(rest[index + 1])
            index += 2
        elif flag == "--out" and index + 1 < len(rest):
            out = rest[index + 1]
            index += 2
        elif flag == "--library" and index + 1 < len(rest):
            libraries.append(rest[index + 1])
            index += 2
        elif flag == "--sample" and index + 1 < len(rest):
            sample = float(rest[index + 1])
            index += 2
        elif flag == "--demo" and index + 1 < len(rest):
            demo = rest[index + 1]
            index += 2
        else:
            print(f"无法识别的参数：{flag}", file=sys.stderr)
            raise SystemExit(2)

    if len(argv) == 1 or not path:
        print(__doc__.strip(), file=sys.stderr)
        raise SystemExit(2)
    if demo not in ("", "mover"):
        print(f"未知的 --demo 值：{demo}（目前只有 mover）", file=sys.stderr)
        raise SystemExit(2)

    if not out:
        # 默认落在想定同名文件旁边，省得每次都要写 --out
        out = str(Path("app") / (Path(path).stem + ".replay.json"))
    return path, seconds, out, libraries, sample, demo


def main(argv: list[str]) -> int:
    path, seconds, out_path, libraries, sample, demo = parse_args(argv)
    file = Path(path)
    source = file.read_text(encoding="utf-8")

    components = ComponentRegistry()
    platforms = PlatformRegistry()
    register_framework(components, platforms)
    demo_components.register_all(components)

    sim = Simulation.from_scenario(
        source,
        name=file.name,
        components=components,
        platforms=platforms,
        library=libraries or None,
    )

    try:
        sim.build()
    except MilsimError as exc:
        if isinstance(exc, (LexError,)) or hasattr(exc, "line"):
            print(format_error(exc, path=file.name, source=source), file=sys.stderr)
        else:
            print(f"{file.name}: {exc}", file=sys.stderr)
        return 1

    sim.initialize()

    # --- 运行期下达的机动命令（可选） ---------------------------------------
    # 想定刻意不写初始机动（见模块 docstring），所以想让它动起来必须在这里下。
    # 这段发生在第一帧快照之前 ⇒ 回放的 t=0 就是"已受令、尚未动"的布势。
    issued: list[str] = []
    if demo == "mover":
        issued = _issue_mover_commands(sim)
        for line in issued:
            print(line)

    # --- 静态部分：帧帧不变的量只导出一次 -----------------------------------
    zones = [replay_mod.zone_dict(z) for z in sim.maps.active_zones]
    zone_frames = replay_mod.zone_frames_of(sim)
    entity_static = {
        sid: replay_mod.entity_static(sim, sid) for sid in sorted(sim.entities)
    }

    # --- 逐拍采样 -----------------------------------------------------------
    # ★ 不用 engine.on_fence：它只在"这拍被栅栏截断（无事发生）"时才回调，
    #   而有事件的想定几乎不踩栅栏 ⇒ 帧数会塌成 1。改用定步长循环：
    #   每步推进 step_us、步末取一份快照，时间轴严格等距。
    #
    # step_us 不是想定的 max_step：那两者是解耦的（见模块 docstring）。
    # 引擎内部仍按 max_step 推（run_for 自己会细分），这里只管取帧粒度。
    frames: list[dict[str, Any]] = []
    # 起点也取一帧：第 0 拍各实体还没动，但它给出"初始布势"，是回放的第一格
    frames.append(replay_mod.frame(sim, zone_frames))

    step_us = replay_mod.default_step_us(sim, sample)
    total_us = int(seconds * 1_000_000)
    while sim.engine.now < total_us:
        remaining = total_us - sim.engine.now
        sim.run_for(min(step_us, remaining))
        frames.append(replay_mod.frame(sim, zone_frames))

    report = sim.shutdown()

    payload = {
        "format": FORMAT,
        "scenario": sim.spec.settings.name or file.stem,
        "seed": sim.spec.settings.seed,
        "max_step_us": sim.spec.settings.max_step_us,
        "sample_us": step_us,
        "duration_us": sim.engine.now,
        "zones": zones,
        "entities": entity_static,
        "frames": frames,
        "legend": replay_mod.legend(),
        "report": {
            "sim_time_us": report.sim_time_us,
            "steps": report.steps,
            "events_dispatched": report.events_dispatched,
            "fences": report.fences,
            "entities": report.entities,
            "alive": report.alive,
        },
    }

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )

    size_kb = out.stat().st_size / 1024.0
    print(f"想定：{payload['scenario']}  时长 {format_sim_time(payload['duration_us'])}")
    print(f"战区 {len(zones)} 个 / 实体 {len(entity_static)} 个 / 帧 {len(frames)} 个"
          f"（采样 {step_us / 1_000_000.0:.3g} s）")
    print(f"已写出：{out}（{size_kb:.1f} KB）")
    return 0


# ---------------------------------------------------------------------------
# 组装
# ---------------------------------------------------------------------------

def _issue_mover_commands(sim: Simulation) -> list[str]:
    """给 ``mover_demo`` 那样的想定逐件下达目的地（运行期命令）。

    照 ``tools/demo_mover.py`` 的同一套规则挑目的地——**判据是
    「代价 ÷ 直线距离」最大**（不是格数最大：代价里含地形权重，绕远但好走的
    路在格数上反而更长；也不是"最远的那个"，那常常是一片山地，轮式根本进不去）。
    规则照抄的好处是"回放里看到的路线"与"demo 表格里量出来的路线"是同一条，
    两处不会各说各话。

    ★ 这里只挑一个目的地（陆上件共用一个），不下发潜艇/船各自的专属目的地方案
    ——本工具的目的是"让回放看得出在动"，不是复刻 demo 的全部测量节。所以
    潜艇那份单列（它在另一个域里，用陆上的目的地会一步都动不了）。

    返回的是给人看的说明行（由 ``main`` 打印），本函数不产生 JSON。
    """
    names = ("CAR_1", "CAR_2", "JET_1", "SUB_1")
    movers = {}
    for name in names:
        entry = sim.registry.by_name(name)
        if entry is None:
            continue
        mover = sim.entities[entry.entity_id].component("mover")
        if mover is not None:
            movers[name] = (entry.entity_id, mover)
    if not movers:
        # 不是 mover_demo 那套名字（v0.13.49 起也服务 latlng 想定）：
        # 走通用档——不挑地形目的地（这类想定多半没有地形画像，格寻路
        # 会直接失败），按**当前航向**给每台带运动件的平台下达两段航线。
        return _issue_generic_mover_commands(sim)

    lines: list[str] = []
    # 地面件用第一个（轮式）的代价配置挑目的地——它封了山地，挑出来的是
    # "绕远也到得了"的那一类。没有轮式则退回任意一件。
    anchor_name = "CAR_1" if "CAR_1" in movers else sorted(movers)[0]
    anchor_id, anchor_mover = movers[anchor_name]
    ref = sim.store.cell_of(anchor_id)
    start = ref.axial
    goal, route = _choose_goal(sim, ref, start, anchor_mover.travel_config())
    goal_x, goal_y = sim.maps.zone(ref.zone_id).frame.to_world(goal)
    lines.append(
        f"--demo mover：{anchor_name} 起点 {start} → 目的地 {goal}"
        f"（A* {route.length_cells} 格），逐件下达"
    )

    # 地面件（车）：走到同一个目的地格 → 同一格 → 空中件飞到该点的巡航高度。
    for name in ("CAR_1", "CAR_2", "SUB_1"):
        if name not in movers:
            continue
        entity_id, mover = movers[name]
        if name == "SUB_1":
            # 潜艇在另一个域里：用**它自己**的起点挑一个水下目的地，否则
            # 陆上那格它一步都动不了（门槛按格判）。
            sub_ref = sim.store.cell_of(entity_id)
            sub_goal, sub_route = _choose_goal(
                sim, sub_ref, sub_ref.axial, mover.travel_config()
            )
            ok = mover.move_to_cell(sub_goal)
            lines.append(
                f"    SUB_1 → {sub_goal}（A* {sub_route.length_cells} 格）："
                f"{'已受令' if ok else '走不通——' + mover.blocked_reason()}"
            )
            continue
        ok = mover.move_to_cell(goal)
        lines.append(
            f"    {name} → {goal}："
            f"{'已受令' if ok else '走不通——' + mover.blocked_reason()}"
        )

    if "JET_1" in movers:
        _, jet = movers["JET_1"]
        altitude = float(jet.spec["cruise_altitude"])
        jet.move_to_point(goal_x, goal_y, altitude)
        lines.append(f"    JET_1 → ({goal_x:.0f}, {goal_y:.0f}) @ {altitude:.0f} m：已受令")

    return lines


def _issue_generic_mover_commands(sim: Simulation) -> list[str]:
    """通用档（v0.13.49）：给**任意**带运动件的平台下达两段航线。

    背景：``_issue_mover_commands`` 原本只认 ``mover_demo`` 的四个名字，
    且挑目的地要走**格寻路**（``_choose_goal``）——那要求战区有地形画像。
    ``comm_track_jam`` 这类 latlng 想定没有 terrain 块，寻路必然失败，
    平台又确实要动 ⇒ 另立一条不依赖地形的规则：

      * 对每台带 ``mover`` 件的平台（弹除外——弹由制导件管），沿**当前
        航向**飞 10 km，再左转 60° 飞 6 km。一段直线看不出"机动"，
        两段折线拐个弯才看得出；距离取小值是让全场待在战区里。
      * 空中件（spec 里有 ``cruise_altitude``）飞巡航高度；其余保持
        当前高度。坐标是**局部平面米**（x 东、y 南），与
        :meth:`milsim.models.mover.Mover.move_along_route` 同一套。
      * 航向换算：dx = d·sinθ、dy = −d·cosθ（θ 为罗盘航向、y 轴指南），
        与 ``EntityStore.advance_all`` 的向量式逐字一致。

    返回给人看的说明行；同一想定两次跑结果完全一样（航向与位置都是
    装配期的确定值）。
    """
    import math

    lines: list[str] = [
        "--demo mover：通用档（latlng 想定，无格寻路）——逐台下达两段航线"
    ]
    issued = 0
    for entity_id in sorted(sim.entities):
        entity = sim.entities[entity_id]
        mover = entity.component("mover")
        if mover is None:
            continue
        spec = getattr(mover, "spec", None)
        # ★ spec 是 ResolvedComponent 视图：支持 ``spec.get(name, default)``、
        #   **不支持** ``in``（没有 __contains__，``in`` 会走 __getitem__ 撞
        #   KeyError: 0 —— 实测踩过）。
        if spec is not None and spec.get("launch_speed") is not None:
            continue        # 弹由制导件管，不接机动命令（§5.8）
        pose = sim.store.pose_of(entity_id)
        if pose is None:
            continue
        x, y, z, heading, _speed = pose
        alt = spec.get("cruise_altitude") if spec is not None else None
        alt = float(alt) if alt is not None else z
        h1 = math.radians(float(heading))
        leg1, leg2 = 10000.0, 6000.0
        p1 = (x + leg1 * math.sin(h1), y - leg1 * math.cos(h1), alt)
        h2 = math.radians(float(heading) + 60.0)
        p2 = (p1[0] + leg2 * math.sin(h2), p1[1] - leg2 * math.cos(h2), alt)
        ok = mover.move_along_route([p1, p2])
        issued += 1
        lines.append(
            f"    {entity.name} → 两段航线（10 km 航向 {float(heading):.0f}°"
            f" + 6 km 左转 60°）@ {alt:.0f} m："
            f"{'已受令' if ok else '走不通——' + mover.blocked_reason()}"
        )
    if not issued:
        return ["--demo mover：这份想定里没有可下达命令的机动件"]
    return lines


def _choose_goal(sim: Simulation, ref: Any, start: Any, profile: Any) -> tuple[Any, Any]:
    """挑一个"最需要绕行"的目的地，返回 ``(格, 路线)``。

    与 ``tools/demo_mover.py`` 的 ``choose_goal`` 同规则：在起点周围一圈等角
    候选里，取代价 ÷ 直线距离最大者。候选顺序固定 ⇒ 同一份想定两次跑出的
    结果完全一样。这里自己实现一份（而不是 import demo_mover）是为了让
    ``tools/`` 里的两个工具各自独立——本文件 import 的是 ``demo_components``
    这类**注册用**的东西，不是另一个工具的测量逻辑。
    """
    from math import cos, hypot, radians, sin

    from milsim.services.map import Axial

    zone = sim.maps.zone(ref.zone_id)
    sx, sy = zone.frame.to_world(start)
    # 先把起点周围一大片地形生成出来：不预热的话，判据选的其实是
    # "被默认地貌（水域）堵得最死的方向"，而不是"最需要绕行的方向"。
    sim.nav.ensure_region(ref, start, _CANDIDATE_RADIUS_CELLS + 24)

    best: tuple[Any, Any, float] | None = None
    for index in range(_CANDIDATE_DIRECTIONS):
        angle = index * (360.0 / _CANDIDATE_DIRECTIONS)
        dq = round(_CANDIDATE_RADIUS_CELLS * sin(radians(angle)))
        dr = round(_CANDIDATE_RADIUS_CELLS * cos(radians(angle)))
        cell = Axial(start.q + dq, start.r + dr)
        route = sim.nav.route(ref, cell, profile=profile)
        if route is None or len(route.cells) < 3:
            continue
        gx, gy = zone.frame.to_world(cell)
        straight = hypot(gx - sx, gy - sy)
        if straight <= 0.0:
            continue
        ratio = route.cost / straight
        if best is None or ratio > best[2]:
            best = (cell, route, ratio)

    if best is None:
        raise SystemExit(
            "这个战区里找不到任何可到达的目的地——换一个地形种子，"
            "或这份想定不适合 --demo mover（例如没有地形画像）"
        )
    return best[0], best[1]


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
