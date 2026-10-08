"""逐拍态势快照：把仿真世界导成前端可读的帧（JSON-ready）。

这个模块是**回放/实时推流两条路共用的那一半**：

    离线导出  tools/export_replay.py   → 写文件
    实时推流  tools/serve_replay.py    → 推 WebSocket
                    ↘         ↙
                 本模块（同一份帧定义）

★ 为什么要有这一层
------------------
一开始帧导出只有离线一条路，代码写在 ``tools/export_replay.py`` 里。加实时
推流时若照抄一遍，两处就会各自演化——"离线看到的"和"实时看到的"慢慢不是
一回事，而它们本来是同一份数据。**一份实现，两个出口**，离线与实时才可比。

★ 帧里有什么（五层）
--------------------
    entities    实体位姿 + **经纬度** + 航向 + 速度 + 存活
    contacts    每个平台的航迹表（谁发现了谁；含 `same_side` / `hops`）
    comm        网络成员 / 显式边 / 掉线节点 / 可达性
    jammers     在册干扰天线（→ 谁有干扰能力）
    sensors     每部雷达这一拍的读数（被压制的证据：`jamming_w` / `last_pd`）

全部来自**已有的只读接口**，本模块不碰仿真内部状态、不写回。

★ 两条口径坑（前端必须知道，这里只说一次）
----------------------------------------
1. **航迹表含友军**。``TRACK_MANAGER.broadcast`` 不区分敌我（引擎真实行为），
   而 ``SituationProjector._fuse_contacts`` 是"按方过滤后再融合"的。本模块给的是
   **未过滤的原料**（给复盘用），所以每条航迹带 ``same_side`` 标记，
   **画不画由前端定**。
2. **干扰关系要自己拼**。``jammers`` 只回答"谁有干扰能力"，**没有**"在压谁"。
   受害方从 ``sensors`` 那层的 ``jamming_w > 0`` 读出来——那才是"被压制"的直接证据。

★ 坐标：一行投影公式都不写
--------------------------
实体 ``(x, y)`` 是战区局部米制平面坐标。换成经纬度**不用新写**：战区挂着
``LocalFrame``，它的 ``world_to_geo`` 就是方位等距投影（AED）的闭式反解。
实测与独立实现的精确解差 0.4 mm(北) / 3 mm(东)。★ 反例：用**小角近似**
自算会差 4.54 m——那全是近似公式的错。**"很像对"的公式错起来不报错，
只会让整张图悄悄偏掉。**

★ y 轴指向南
------------
``LocalFrame.to_world`` 的 y 轴指向南（图像坐标习惯）。``world_to_geo``
内部用的是**真实东/北分量**，所以导出的经纬度是对的；调用方不必再翻。
"""

from __future__ import annotations

from typing import Any

from milsim.services.situation import format_sim_time
from milsim.services.store import LocalCellRef

#: 回放文件格式版本。前端可以据此判断字段是否有变。
#: 实时推流用的是**同一份帧**，所以共用这个常量。
FORMAT = "milsim-replay/1"


# ---------------------------------------------------------------------------
# 静态部分（帧帧不变，整场只算一次）
# ---------------------------------------------------------------------------

def zone_dict(zone: Any) -> dict[str, Any]:
    """战区的静态几何。前端拿它画边界圆 + 定底图视野。"""
    spec = zone.spec
    return {
        "zone_id": spec.zone_id,
        "name": spec.name,
        # 圆心 = 局部坐标原点 = LocalFrame 的锚点
        "lat": spec.lat,
        "lng": spec.lng,
        "radius_m": spec.radius_m,
        "resolution_m": spec.resolution_m,
        "layers": spec.layer_count,
    }


def entity_static(sim: Any, entity_id: int) -> dict[str, Any]:
    """实体在整场推演里不变的属性（型号、阵营、挂载件）。"""
    entity = sim.entities[entity_id]
    return {
        "name": entity.name,
        "side": sim.registry.side_of(entity_id),
        "type_name": sim.registry.type_of(entity_id),
        "slots": list(entity.slots()),
        # 干扰能力：在册天线条数。0 ⇒ 没有干扰件
        "jammer_antennas": sim.jam.antennas_of(entity_id),
    }


def zone_frames_of(sim: Any) -> dict[int, Any]:
    """``zone_id → zone``。取 frame 用。"""
    return {z.spec.zone_id: z for z in sim.maps.active_zones}


def legend() -> dict[str, Any]:
    """给前端的一组"该怎么画"的提示。纯展示，不影响计算。"""
    return {
        "sides": {
            "red": {"label": "红方", "color": "#E24B4A"},
            "blue": {"label": "蓝方", "color": "#378ADD"},
        },
        "layers": [
            {"key": "entities", "label": "实体", "hint": "圆点 + 航向短线"},
            {"key": "contacts", "label": "航迹", "hint": "观察者→目标虚线"},
            {"key": "comm", "label": "通信网", "hint": "实线 = 通，虚线 = 断"},
            {"key": "jammers", "label": "干扰源", "hint": "施扰端加一圈辐射标记"},
        ],
    }


# ---------------------------------------------------------------------------
# 一帧
# ---------------------------------------------------------------------------

def to_latlng(
    sim: Any, zone_frames: dict[int, Any], entity_id: int, x: float, y: float
) -> tuple[float | None, float | None]:
    """局部米制 ``(x, y)`` → 经纬度。

    实体的 zone_id 从 ``store.cell_of()`` 的 :class:`LocalCellRef` 取。
    **不自己推投影公式**——用战区自带的 ``LocalFrame.world_to_geo``。
    """
    ref = sim.store.cell_of(entity_id)
    if not isinstance(ref, LocalCellRef):
        return None, None
    zone = zone_frames.get(ref.zone_id)
    if zone is None:
        return None, None
    lat, lng = zone.frame.world_to_geo(x, y)
    return lat, lng


def frame(sim: Any, zone_frames: dict[int, Any]) -> dict[str, Any]:
    """一份逐拍快照。含实体、航迹、通信、干扰、感知五层。"""
    at = sim.engine.now

    # --- 实体层：位置 + 经纬度 + 航向 --------------------------------------
    entities: list[dict[str, Any]] = []
    for entity_id in sorted(sim.entities):
        pose = sim.store.pose_of(entity_id)
        if pose is None:
            continue
        x, y, z, heading, speed = pose
        lat, lng = to_latlng(sim, zone_frames, entity_id, x, y)
        entities.append({
            "entity_id": entity_id,
            "name": sim.entities[entity_id].name,
            "side": sim.registry.side_of(entity_id),
            "x": round(x, 3),
            "y": round(y, 3),
            "z": round(z, 3),
            "lat": round(lat, 7) if lat is not None else None,
            "lng": round(lng, 7) if lng is not None else None,
            "heading_deg": round(heading, 2),
            "speed_mps": round(speed, 2),
            "alive": sim.store.is_alive(entity_id),
        })

    # --- 探测层：谁发现了谁（航迹连线） ------------------------------------
    # ★ 导出的是**上帝视角的原始航迹表**，含友军航迹（`TRACK_MANAGER` 转发时
    #   并不区分敌我，这是引擎的真实行为）。所以这里**不能直接当"敌情"画**：
    #   每条航迹带上 `same_side` 标记，前端自行决定要不要显示。
    contacts: list[dict[str, Any]] = []
    for observer_id in sorted(sim.entities):
        observer_side = sim.registry.side_of(observer_id)
        for c in sim.store.contacts_of(observer_id):
            contact_side = sim.registry.side_of(c.target_id)
            contacts.append({
                "from": observer_id,
                "to": c.target_id,
                "quality": round(c.quality, 4),
                "range_m": round(c.range_m, 1),
                "bearing_deg": round(c.bearing_deg, 2),
                "detected_at": c.detected_at,
                #: 0 = 本平台亲自量测；≥1 = 经通信转发来的（有几跳）
                "hops": c.hops,
                #: 是否友军航迹（不是敌情，画不画由前端定）
                "same_side": contact_side == observer_side,
                "phantom": bool(getattr(c, "phantom", False)),
            })

    # --- 通信层：节点 + 边 + 掉线 ------------------------------------------
    nodes = sorted(sim.comm._membership) if sim.comm.networks() else []
    edges: list[dict[str, Any]] = []
    for net_name in sim.comm.networks():
        members = sim.comm.members_of(net_name)
        for src, dst in _net_links(sim, net_name, members):
            edges.append({
                "from": src,
                "to": dst,
                "network": net_name,
                # 此刻这条边通不通（掉线节点参与的边 = 不通）
                "up": sim.comm.path_exists(src, dst),
            })

    # --- 干扰层：在册的干扰天线（施扰端） ----------------------------------
    # ★ ``specs()`` 的每一行是**一条天线**，不是一台机器；``tag`` 里带 ``#序号``
    #   把同机的多条天线串起来。前端按 ``entity_id`` 去重画辐射标记。
    jammers = [
        {
            "entity_id": j.entity_id,
            "tag": j.tag,
            "power_w": round(j.power_w, 3),
            "frequency_hz": j.frequency_hz,
        }
        for j in sim.jam.specs()
    ]

    # --- 感知层：每部雷达这一拍的读数（被压制的证据在这里） ----------------
    # 「哪部雷达在挨打」这个问题的最直接答案：``jamming_w`` > 0 就是有干扰进来，
    # ``last_pd`` 是这一拍的检测概率（被压后会掉）。逐拍拉出来就是一条曲线。
    sensors: list[dict[str, Any]] = []
    for entity_id in sorted(sim.entities):
        radar = sim.entities[entity_id].component("sensor")
        if radar is None:
            continue
        sensors.append({
            "entity_id": entity_id,
            "ticks": radar.ticks,
            "sweeps": round(radar.sweeps, 3),
            "detections": radar.detections,
            "geometry_blocked": radar.geometry_blocked,
            "last_pd": round(radar.last_pd, 6),
            "jamming_w": float(radar.jamming_w),
            "jamming_range_m": round(radar.jamming_range_m, 1),
        })

    return {
        "at": at,
        "at_text": format_sim_time(at),
        "entities": entities,
        "contacts": contacts,
        "comm": {
            "networks": sim.comm.networks(),
            "nodes": nodes,
            "offline": sorted(sim.comm.offline_nodes),
            "edges": edges,
        },
        "jammers": jammers,
        "sensors": sensors,
    }


def _net_links(
    sim: Any, net_name: str, members: tuple[int, ...]
) -> list[tuple[int, int]]:
    """一个网的**有向边集合**。

    ``links`` 非空 ⇒ 用显式边；为空 ⇒ 该网是"入网即全连通"的 mesh，
    此时按完备图展开（每对成员双向），否则前端画不出默认拓扑。
    """
    net = sim.comm._nets.get(net_name)
    if net is None:
        return []
    if net.links:
        return list(net.links)
    ordered = sorted(members)
    return [(a, b) for a in ordered for b in ordered if a != b]


# ---------------------------------------------------------------------------
# 采样节拍
# ---------------------------------------------------------------------------

def default_step_us(sim: Any, sample_seconds: float) -> int:
    """取帧粒度（微秒）。

    ``sample_seconds`` 为 0 ⇒ 用想定的 ``max_step``。两者**解耦**：
    引擎内部仍按 ``max_step`` 推（``run_for`` 自己会细分），这里只管取帧。
    理由见 ``tools/export_replay.py`` 的模块 docstring——采样粒度与模型节拍
    不是一回事；运动学想定（600 s 跑十几公里）照 2 s 采会出上兆且看不出动。
    """
    if sample_seconds > 0.0:
        return max(1, int(sample_seconds * 1_000_000))
    return int(sim.spec.settings.max_step_us)


__all__ = [
    "FORMAT",
    "default_step_us",
    "entity_static",
    "frame",
    "legend",
    "to_latlng",
    "zone_dict",
    "zone_frames_of",
]
