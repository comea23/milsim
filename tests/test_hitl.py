"""人在回路上行指令（v0.13.57，§5.16.22）的解析与执行测试。

``tools/serve_replay.py`` 不在包搜索路径上，按文件加载——与
``tests/test_library.py`` 加载 ``tools/model_library.py`` 同一手法。

覆盖两半：

- ``parse_uplink``：语法与数值校验（握手线程侧，不碰仿真状态）；
- ``execute_uplink``：真实想定（mover_demo）上的语义执行（主线程侧）。
  线程编排（inbox 投递、WS 帧循环）不在单测里——那是 IO 编排，
  由 ``python tools/serve_replay.py`` + 页面人工验证。
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

import pytest

from milsim.models import register_framework
from milsim.services.type_registry import ComponentRegistry, PlatformRegistry
from milsim.simulation import Simulation

_ROOT = Path(__file__).resolve().parents[1]
_TOOL = _ROOT / "tools" / "serve_replay.py"
_SPEC = importlib.util.spec_from_file_location("serve_replay_for_hitl", _TOOL)
serve_replay = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(serve_replay)
# demo_components 也在 tools/ 下，按文件加载（serve_replay 自己也是这么导的）
_DEMO_SPEC = importlib.util.spec_from_file_location(
    "demo_components_for_hitl", _ROOT / "tools" / "demo_components.py")
demo_components = importlib.util.module_from_spec(_DEMO_SPEC)
_DEMO_SPEC.loader.exec_module(demo_components)

_SCENARIO = _ROOT / "scenarios" / "mover_demo.txt"


# ---------------------------------------------------------------------------
# parse_uplink：语法与数值校验
# ---------------------------------------------------------------------------


def test_parse_accepts_a_route() -> None:
    cmd, data, err = serve_replay.parse_uplink(json.dumps({
        "cmd": "route", "target": "CAR_1",
        "points": [[39.9, 116.4], [39.91, 116.41, 500]],
    }))
    assert err == ""
    assert cmd == "route"
    assert data["points"] == [[39.9, 116.4], [39.91, 116.41, 500.0]]


def test_parse_accepts_goto_and_stop_and_pace() -> None:
    cmd, data, err = serve_replay.parse_uplink(
        '{"cmd":"goto","target":3,"lat":39.9,"lng":116.4,"alt":800}')
    assert (cmd, err) == ("goto", "")
    assert data["points"] == [[39.9, 116.4, 800.0]]

    cmd, data, err = serve_replay.parse_uplink('{"cmd":"stop","target":"CAR_1"}')
    assert (cmd, err) == ("stop", "")

    cmd, data, err = serve_replay.parse_uplink('{"cmd":"pace","value":0.5}')
    assert (cmd, err) == ("pace", "")
    assert data["value"] == 0.5


def test_parse_rejects_bad_json_and_unknown_cmd() -> None:
    cmd, _, err = serve_replay.parse_uplink("not json")
    assert cmd == "" and "JSON" in err

    cmd, _, err = serve_replay.parse_uplink('{"cmd":"teleport","target":1}')
    assert cmd == "" and "未知指令" in err

    cmd, _, err = serve_replay.parse_uplink('["route"]')
    assert cmd == "" and "JSON 对象" in err


def test_parse_rejects_bad_target() -> None:
    for target in (None, True, [1], 1.5):
        cmd, _, err = serve_replay.parse_uplink(
            json.dumps({"cmd": "stop", "target": target}))
        assert cmd == "" and "target" in err, target


def test_parse_rejects_bad_points() -> None:
    cases: list[dict[str, Any]] = [
        {"cmd": "goto", "target": 1},                         # 缺 lat/lng
        {"cmd": "goto", "target": 1, "lat": "x", "lng": 1},   # 非数字
        {"cmd": "route", "target": 1, "points": []},          # 空航线
        {"cmd": "route", "target": 1, "points": [[1, 2, 3, 4]]},  # 形状不对
        {"cmd": "route", "target": 1, "points": [[91, 0]]},   # 纬度越界
        {"cmd": "route", "target": 1, "points": [[0, 181]]},  # 经度越界
        {"cmd": "route", "target": 1, "points": "north"},     # 不是列表
        {"cmd": "route", "target": 1,
         "points": [[0, 0]] * (serve_replay.MAX_UPLINK_POINTS + 1)},  # 超上限
    ]
    for case in cases:
        cmd, _, err = serve_replay.parse_uplink(json.dumps(case))
        assert cmd == "" and err, case


def test_parse_rejects_bad_pace() -> None:
    for value in (None, "fast", -0.1, 10.1):
        cmd, _, err = serve_replay.parse_uplink(
            json.dumps({"cmd": "pace", "value": value}))
        assert cmd == "" and err, value


# ---------------------------------------------------------------------------
# execute_uplink：真实想定上的语义执行（主线程侧）
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def sim() -> Simulation:
    components = ComponentRegistry()
    platforms = PlatformRegistry()
    register_framework(components, platforms)
    demo_components.register_all(components)
    s = Simulation.from_scenario(
        _SCENARIO.read_text(encoding="utf-8"),
        name=_SCENARIO.name, components=components, platforms=platforms,
    )
    s.build()
    s.initialize()
    return s


def test_execute_stop_clears_the_destination(sim: Simulation) -> None:
    ok, detail = serve_replay.execute_uplink(
        sim, {"cmd": "stop", "target": "CAR_1"})
    assert ok and "CAR_1" in detail
    mover = sim.entities[sim.registry.by_name("CAR_1").entity_id] \
        .component("mover")
    assert mover.destination_point() is None


def test_execute_goto_by_name_converts_latlng(sim: Simulation) -> None:
    # 战区锚点 (39.9042, 116.4074) 必在 EAST_SECTOR 内
    ok, detail = serve_replay.execute_uplink(
        sim, {"cmd": "goto", "target": "CAR_1",
              "lat": 39.9042, "lng": 116.4074})
    assert ok, detail
    assert "单点机动" in detail
    mover = sim.entities[sim.registry.by_name("CAR_1").entity_id] \
        .component("mover")
    assert mover.destination_point() is not None


def test_execute_route_by_entity_id(sim: Simulation) -> None:
    jet_id = sim.registry.by_name("JET_1").entity_id
    ok, detail = serve_replay.execute_uplink(
        sim, {"cmd": "route", "target": jet_id,
              "points": [[39.9042, 116.4074], [39.92, 116.42]]})
    assert ok, detail
    assert "2 点航线" in detail


def test_execute_rejects_unknown_target(sim: Simulation) -> None:
    ok, detail = serve_replay.execute_uplink(
        sim, {"cmd": "stop", "target": "NOPE_9"})
    assert not ok and "找不到目标" in detail

    ok, _ = serve_replay.execute_uplink(sim, {"cmd": "stop", "target": 99999})
    assert not ok


def test_execute_out_of_zone_point_is_tolerated_in_single_zone(
    sim: Simulation,
) -> None:
    """唯一战区时战区外的点容错收下（投影拉回战区边缘附近）。"""
    ok, detail = serve_replay.execute_uplink(
        sim, {"cmd": "goto", "target": "CAR_1", "lat": 0.0, "lng": 0.0})
    assert ok, detail
