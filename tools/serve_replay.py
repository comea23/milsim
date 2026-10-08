"""实时推流：边跑仿真边把态势推给浏览器（**标准库，零新依赖**）。

用法::

    python tools/serve_replay.py                    # 默认 comm_track_jam，端口 8765
    python tools/serve_replay.py scenarios/patrol.txt --port 9000
    python tools/serve_replay.py scenarios/mover_demo.txt --demo mover \
        --seconds 600 --sample 5 --pace 0.5
    python tools/serve_replay.py --list             # 列出可推的想定

然后打开 ``http://127.0.0.1:<port>/``，页面左栏「数据源」有两个实时档：
**一键跑完**（``mode=fast``，服务端 pace 收敛到 0，全速跑完后拖时间轴回看）
与 **边跑边看**（按想定时间轴逐拍长出来）。两者**共用同一份前端渲染路径与
同一套帧定义**，但服务端**各持一份会话**（缓存键 = 想定 + 模式）——
差别只有"每拍之间等多久"这一个数，而会话必须分开：一个会话是**一次性**的。

它做两件事
----------
1. 静态文件服务：把 ``app/`` 目录原样发给浏览器（同一个 ``index.html``）。
2. **WebSocket 推送**：仿真在服务端逐拍推进，每拍把一份帧推给所有连着的页面。

★ 为什么用标准库
----------------
``pyproject.toml`` 现在只有 ``numpy`` 一个必需依赖，``h3`` 是可选，**web 依赖
一个都没有**。为了一个内网回放页去装 FastAPI + uvicorn + websockets 三个包，
与"引擎层刻意保持零依赖"的设计取向不符。WebSocket 的服务端一侧比想象中薄：
握手是一次 SHA-1 + base64，之后就是**帧格式**（长度前缀 + 掩码）——百来行就够。

★ 与离线导出的关系
------------------
帧定义在 :mod:`milsim.services.replay`，**与 ``tools/export_replay.py`` 共用**。
所以实时看到的与离线看到的**不是两套数据**。协议上：
``{"type":"hello", ...}`` 先给静态部分（战区 / 实体表 / 图例），
之后每拍一条 ``{"type":"frame", "frame":{...}}``，
跑完一条 ``{"type":"done", ...}`` 并带上汇总。
★ 前端因此可以**离线文件与实时推流走同一条渲染路径**（见 ``app/index.html``）。

★ 节奏控制
----------
``--pace`` 是"每拍之间实时等多久"（秒）。纯加速跑（``0``）会把 60 s 的想定
在 1 s 内推完，人眼只看得到一个残影——那等于没推。默认按想定的采样间隔真实
推进（``--pace`` 留空 = 用 ``sample``）。``--pace 0`` 是"尽快推完"。

★ ``--pace`` = **启动默认值**；页面上的**一键跑完**会连 ``mode=fast``，
服务端按 ``(想定, mode)`` 开一份**独立的**会话并把它的 pace 定成 0。
"边跑边看"则是 ``(想定, "")`` 那份，按时间轴走。两档**各有一份会话**，
互不污染——这是"两种用法一套实现"的落点：差别只有 pace 这个数，
但会话必须分开（一个会话是**一次性**的，见 ``_handle_ws``）。

★ 线程模型
----------
仿真跑在一个**单独的线程**里（不是每连接一个线程跑仿真）——一份想定推给
多个页面时只有一份计算。页面连接/断开只动订阅名单。
``EventBus`` 是服务层的东西，但这里**不用它**：视图要的是"每拍一帧"，
而不是 11 个信号各自的通知；定步长取帧才是前端要的那条等距轴（与离线导出
同一条理由，见 ``export_replay.py``）。
"""

from __future__ import annotations

import base64
import hashlib
import io
import json
import math
import re
import sqlite3
import struct
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import demo_components  # noqa: E402
from milsim.errors import MilsimError  # noqa: E402
from milsim.models import register_framework  # noqa: E402
from milsim.services import replay as replay_mod  # noqa: E402
from milsim.services.input.lexer import LexError, format_error  # noqa: E402
from milsim.services.type_registry import (  # noqa: E402
    ComponentRegistry,
    PlatformRegistry,
)
from milsim.simulation import Simulation  # noqa: E402

#: WebSocket 握手用的固定 GUID（RFC 6455 §4.2.2）。
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

#: 默认想定与端口。
DEFAULT_SCENARIO = "scenarios/comm_track_jam.txt"
DEFAULT_PORT = 8765

#: 一帧的默认时长（没给 --sample 时用想定的 max_step）。
DEFAULT_SECONDS = 60.0


# ===========================================================================
# 1. WebSocket 帧编解码（RFC 6455 §5）——服务端一侧
# ===========================================================================

def ws_accept_key(client_key: str) -> str:
    """客户端 ``Sec-WebSocket-Key`` → 服务端 ``Sec-WebSocket-Accept``。

    ``base64(sha1(key + GUID))``。RFC 6455 §4.2.2 给了一个测试向量：
    ``dGhlIHNhbXBsZSBub25jZQ==`` ⇒ ``s3pPLMBiTxaQ9kYGzzhZRbK+xOo=``。
    """
    digest = hashlib.sha1((client_key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def ws_encode(payload: bytes) -> bytes:
    """把一段数据封成一个**未掩码**的服务端文本帧。

    服务端发的帧**不得掩码**（RFC 6455 §5.1）。长度有三种编码：
    ``<126`` 直接写；``126..65535`` 用 2 字节；再大用 8 字节。
    """
    header = bytearray()
    header.append(0x81)  # FIN=1 + opcode=1（text）
    n = len(payload)
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header.extend(struct.pack("!H", n))
    else:
        header.append(127)
        header.extend(struct.pack("!Q", n))
    return bytes(header) + payload


def ws_send_text(sock: Any, text: str) -> None:
    sock.sendall(ws_encode(text.encode("utf-8")))


def ws_send_close(sock: Any, code: int = 1000) -> None:
    """发一个关闭帧（尽力而为，失败就算了——对端可能已经走了）。"""
    try:
        sock.sendall(bytes([0x88, 2]) + struct.pack("!H", code))
    except OSError:
        pass


def ws_read_frame(sock: Any) -> tuple[int, bytes] | None:
    """读一个客户端帧，返回 ``(opcode, payload)``；连接结束返回 ``None``。

    客户端帧**一定掩码**（RFC 6455 §5.1），所以要解掩码。这里只处理本工具
    用得到的：``close`` / ``ping`` / ``pong`` / ``text``；控制帧的 payload
    一律 ≤125 字节。不做分片重组——本工具的服务端**只发不收**，收只是为了
    知道"对端关了"和"还活着"。
    """
    head = _recv_exact(sock, 2)
    if head is None:
        return None
    b0, b1 = head[0], head[1]
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    length = b1 & 0x7F
    if length == 126:
        ext = _recv_exact(sock, 2)
        if ext is None:
            return None
        length = struct.unpack("!H", ext)[0]
    elif length == 127:
        ext = _recv_exact(sock, 8)
        if ext is None:
            return None
        length = struct.unpack("!Q", ext)[0]
    mask = b""
    if masked:
        mask = _recv_exact(sock, 4) or b""
    payload = _recv_exact(sock, length) if length else b""
    if payload is None:
        return None
    if masked and mask:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return opcode, payload


def _recv_exact(sock: Any, n: int) -> bytes | None:
    """读满 ``n`` 字节；对端先关就返回 ``None``。"""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


# ===========================================================================
# 2. 推流会话：一份想定、一个线程、一组订阅者
# ===========================================================================

class StreamSession:
    """跑一份想定并把每帧广播给订阅者。

    ★ 仿真在**一个**线程里跑（``_run``），订阅者只是被追加/移除的 socket。
    这样 N 个页面连着也只有一份计算。
    """

    def __init__(
        self,
        scenario: str,
        *,
        seconds: float,
        sample: float,
        demo: str,
        pace: float,
        libraries: list[str] | None = None,
    ) -> None:
        self.scenario = scenario
        self.seconds = seconds
        self.sample = sample
        self.demo = demo
        #: ★ ``pace`` 是**可变的**（``set_pace``）：同一个会话既可能被
        #:   "边跑边看"（按时间轴）连，也可能被"一键跑完"（尽快）连。
        #:   两种模式**不是两套代码**——差别只有这一个数。
        self._pace = pace
        self.libraries = libraries or []

        self._lock = threading.Lock()
        self._subs: list[Any] = []
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        #: 有第一个订阅者才开跑。★ 不这么做的话，服务一起跑就把整场推完了，
        #: 而那时一个页面都没连上——人打开浏览器看到的是"已结束"。
        #: ★ 放行点必须在**装配之前**：装配要 1~2 s，而"一键跑完"整场仅
        #:   0.07 s——若等 hello 备好再放行，模拟器会在任何人连上之前跑完。
        self._gate = threading.Event()
        #: hello 已备好（供握手线程等——否则会撞上"attach 时 hello 还是 None"）。
        self._hello_ready = threading.Event()

        self.hello: dict[str, Any] | None = None
        self.frame_count = 0
        self.done = False
        self.error: str | None = None
        self.log: list[str] = []

    # -- 订阅管理 ----------------------------------------------------------
    def attach(self, sock: Any) -> None:
        """接纳一个订阅者：放行仿真线程，等 hello 备好并**发给本人**。

        ★ hello 只在这里发，不在仿真线程里 broadcast。原因：
          ① 握手线程调 ``attach`` 时，hello 可能还没生成（装配要 1~2 s）——
             由 ``attach`` 负责"等它、然后发"最省事，调用方不必知道时序；
          ② 若改成 hello 一备好就 broadcast，同时握手的人会**收两次** hello
             （补发一次 + 广播一次）。一条路径只管一件事。

        ★ 顺序不能反：**先** ``_gate.set()`` 放行仿真，**再**等 hello。
          反了会死锁——没人放行 ⇒ 仿真线程不动 ⇒ hello 永不出来 ⇒ 干等。
        """
        with self._lock:
            self._subs.append(sock)
        self._gate.set()                    # 放行仿真线程（幂等）
        # 装配 ~1~2 s：等 hello 备好（超时给足，装配再慢也不该漏掉静态部分）
        self._hello_ready.wait(timeout=60.0)
        hello = self.hello
        if hello is None:
            return
        try:
            ws_send_text(sock, json.dumps(hello, ensure_ascii=False,
                                          separators=(",", ":")))
        except OSError:
            with self._lock:
                if sock in self._subs:
                    self._subs.remove(sock)

    def detach(self, sock: Any) -> None:
        with self._lock:
            if sock in self._subs:
                self._subs.remove(sock)

    def set_pace(self, pace: float) -> None:
        """改每拍之间的等待秒数（0 = 尽快）。线程安全。

        典型用法：ws 握手时看到 ``mode=fast`` ⇒ 收敛到 0，不为"人眼看"而等。
        """
        with self._lock:
            self._pace = pace

    def pace_of(self) -> float:
        with self._lock:
            return self._pace

    def _broadcast(self, text: str) -> int:
        """发给所有订阅者，剔除发不动的。返回成功数。"""
        with self._lock:
            targets = list(self._subs)
        dead = []
        for sock in targets:
            try:
                ws_send_text(sock, text)
            except OSError:
                dead.append(sock)
        if dead:
            with self._lock:
                for sock in dead:
                    if sock in self._subs:
                        self._subs.remove(sock)
        return len(targets) - len(dead)

    def subscriber_count(self) -> int:
        with self._lock:
            return len(self._subs)

    # -- 生命周期 ----------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="milsim-stream",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    # -- 仿真线程 ----------------------------------------------------------
    def _run(self) -> None:
        path = Path(self.scenario)
        try:
            source = path.read_text(encoding="utf-8")
        except OSError as exc:
            self._fail(f"读不到想定：{exc}")
            return

        # ★ 装配/初始化要花 1~2 s，而"一键跑完"整场只要 0.07 s。若一上来
        #   就进主循环，模拟器自己会在任何页面连上之前跑完 ⇒ 后面连进来的人
        #   只拿到 hello、一帧没有。**仿真必须在第一个订阅者到达后才开始**，
        #   而不是等静态部分（hello）备好就开跑。
        if not self._gate.wait(timeout=120.0):
            self.log.append("等 120 s 无人连接，照常开跑")

        components = ComponentRegistry()
        platforms = PlatformRegistry()
        register_framework(components, platforms)
        demo_components.register_all(components)

        sim = Simulation.from_scenario(
            source, name=path.name, components=components,
            platforms=platforms, library=self.libraries or None,
        )
        try:
            sim.build()
        except MilsimError as exc:
            if isinstance(exc, LexError) or hasattr(exc, "line"):
                self._fail(format_error(exc, path=path.name, source=source))
            else:
                self._fail(f"{path.name}: {exc}")
            return
        sim.initialize()

        if self.demo == "mover":
            from export_replay import _issue_mover_commands  # 同目录工具
            for line in _issue_mover_commands(sim):
                self.log.append(line)

        zone_frames = replay_mod.zone_frames_of(sim)
        step_us = replay_mod.default_step_us(sim, self.sample)

        # 先发静态部分（hello）——前端拿它建底图与实体表，之后只收帧
        self.hello = {
            "type": "hello",
            "format": replay_mod.FORMAT,
            "scenario": sim.spec.settings.name or path.stem,
            "seed": sim.spec.settings.seed,
            "max_step_us": sim.spec.settings.max_step_us,
            "sample_us": step_us,
            "duration_us": int(self.seconds * 1_000_000),
            "zones": [replay_mod.zone_dict(z) for z in sim.maps.active_zones],
            "entities": {
                sid: replay_mod.entity_static(sim, sid)
                for sid in sorted(sim.entities)
            },
            "legend": replay_mod.legend(),
            "live": True,
            "log": list(self.log),
        }
        # ★ hello 不在这里广播：由 ``attach()`` 发给每个新订阅者（见那里的注释）。
        #   这样"晚连进来的人"也一定拿到静态部分，且不会重复收两次。
        self._hello_ready.set()

        # 每拍之间等多久 = pace（0 ⇒ 尽快）。★ 每圈**重读**，这样
        #   ``set_pace`` 中途改也生效（"边跑边看"切"一键跑完"不必重开会话）。
        total_us = int(self.seconds * 1_000_000)

        # 起点也发一帧（初始布势），与离线导出对齐
        self._emit(replay_mod.frame(sim, zone_frames))
        while sim.engine.now < total_us and not self._stop.is_set():
            per_frame = self.pace_of()
            if per_frame > 0.0:
                time.sleep(per_frame)
            remaining = total_us - sim.engine.now
            sim.run_for(min(step_us, remaining))
            self._emit(replay_mod.frame(sim, zone_frames))

        report = sim.shutdown() if not self._stop.is_set() else None
        self.done = True
        summary: dict[str, Any] = {
            "type": "done",
            "frames": self.frame_count,
            "stopped": self._stop.is_set(),
        }
        if report is not None:
            summary["report"] = {
                "sim_time_us": report.sim_time_us,
                "steps": report.steps,
                "events_dispatched": report.events_dispatched,
                "entities": report.entities,
            }
        self._broadcast(json.dumps(summary, ensure_ascii=False,
                                   separators=(",", ":")))

    def _emit(self, frame_dict: dict[str, Any]) -> None:
        self.frame_count += 1
        self._broadcast(json.dumps({"type": "frame", "frame": frame_dict},
                                   ensure_ascii=False, separators=(",", ":")))

    def _fail(self, message: str) -> None:
        self.error = message
        self.done = True
        self._broadcast(json.dumps({"type": "error", "message": message},
                                   ensure_ascii=False, separators=(",", ":")))


# ===========================================================================
# 2.5 卫星影像瓦片：AFSIM 自带的 NASA Blue Marble NG（本地 mbtiles）
# ===========================================================================
#
# ★ 为什么要走本地瓦片，而不是 ``baseMap:{type:'satellite'}``
#   腾讯 GL JS 确实有官方卫星底图（``baseMap.type='satellite'``），但那走的
#   是腾讯自己的在线瓦片服务，① 断网就没了，② 与我们"先离线后实时"的用法
#   不同步（想定时间轴在动，底图却在走别人的 CDN）。
#   ★ 更关键的是：**AFSIM 自己就带着卫星影像**——
#     ``resources/maps/bluemarble_db/bmng.mbtiles``（303 MB，jpg，zoom 0~6），
#     内容是 NASA Blue Marble Next Generation（公有领域），
#     与 CMO 用的 ``BMNGv2_webp.mbtiles`` 是**同一套东西**。
#   所以底图直接用这份，既离线、又合规（NASA 公有领域）、又和 CMO 选型一致。
#
# ★ 谱系：这批 mbtiles 是 **等经纬度（plate carrée）2:1 金字塔**，
#   而且**行号是 TMS 口径（南起）** ★★
#
#   ① 版面 2:1 —— 由库自身的 metadata 一锤定音（``SELECT * FROM metadata``）：
#        profile = {"num_tiles_high_at_lod_0":"1","num_tiles_wide_at_lod_0":"2",
#                   "srs":"+proj=longlat +datum=WGS84 +no_defs",
#                   "xmin":"-180","xmax":"180","ymin":"-90","ymax":"90"}
#      ⇒ LOD0 = 2 宽 1 高、投影是 longlat ⇒ **列数 = 2*2^z、行数 = 2^z**。
#      实测各级：(z0) 2x1、(z1) 4x2、(z2) 8x4、(z3) 16x8、(z4) 32x16、
#      (z5) 64x32、(z6) 128x64 —— 每级恰好 2:1，且**每行的列数恒定**。
#      AFSIM 的 ``about.txt`` 也记着源图是 ``-a_ullr -180 90 -90 0`` 的
#      等经纬度世界图（``osgearth_package --tms``）。
#
#   ② ★★ 行号是 TMS（南起）—— 这才是"地图一片黑"的**唯一真因** ★★
#      mbtiles 规范里 ``tile_row`` 是 **TMS** 口径：**row 0 在南极**。
#      而腾讯 ``ImageTileLayer.getTileUrl(x, y, z)`` 给的 y 是 **XYZ** 口径
#      （**行 0 在北极**）。旧代码把 SDK 的 y 直接当库里的 row ⇒ 整幅图
#      **上下镜像** ⇒ 北京（北纬 40°）被画到南半球的海里 = 一片深蓝。
#      ⇒ 修法就一行：``row_db = (2^z - 1) - row_xyz``。
#
#      实测判据（**像素级**，别再翻案）：
#        · 按 2:1 + **不翻** 拼 z4 得 8192x4096，缩略看是"阶梯状错位世界"；
#        · 按 2:1 + **翻行** 拼 z4 得**干净的等经纬度世界图**
#          （南极在下、北极在上、各大洲形状正确）；
#        · 北京 lon116.4/lat39.9 在 z5 整幅上的像素 (13489,2280)：
#          不翻时 RGB=(4,14,39) 深蓝海；**翻行后 RGB=(111,102,69) 土黄陆地**，
#          裁出的 1400x1400 邻域里能认出渤海/山东半岛/朝鲜半岛/贝加尔湖。
#
#   ★ ``/tiles`` 与 ``/satmap`` **都在服务端翻行**，前端只做"经纬度 → 等经纬
#     格号"这一件纯几何的事（不含 TMS 翻转），行列号语义在这里一次钉死。
#
# ★ 数据来源目录可用环境变量 ``MILSIM_TILE_MBTILES`` 覆盖，缺省找：
#   ``<仓库根>/../resources/maps/bluemarble_db/bmng.mbtiles``（AFSIM 2.9 布局）。

#: 默认瓦片库路径（相对 serve_replay.py 的仓库根）。
DEFAULT_TILE_MBTILES = (
    "resources/maps/bluemarble_db/bmng.mbtiles"
)

#: 瓦片扩展名 → mbtiles 里实际的格式（只用于回 Content-Type）。
_TILE_MIME = {
    "jpg": "image/jpeg",
    "jpeg": "image/jpeg",
    "png": "image/png",
    "webp": "image/webp",
}

#: 等经纬度金字塔在每级的**网格边长**：列 = 2 * 2^z、行 = 2^z。
EQ_COLS = lambda z: 2 * (1 << z)      # noqa: E731
EQ_ROWS = lambda z: 1 << z            # noqa: E731


def tms_to_xyz_row(z: int, row: int) -> int:
    """TMS 行号（南起）→ XYZ 行号（北起）。mbtiles 存的是 TMS。

    ★ 这是"地图一片黑"的**唯一真因**，只有一行算式，但非翻不可。
    """
    return (1 << z) - 1 - row


class TileProvider:
    """从 mbtiles 里取瓦片，并把 TMS 行号翻成 XYZ。

    ★ 每个请求开一个只读连接。sqlite 的连接不能跨线程共用
      （``ThreadingTCPServer`` 每连接一线程），而瓦片请求是低频短事务，
      新建连接（几微秒）比维护连接池省事且不会踩线程安全的坑。
    ★ 取不到就返回 ``None``，由调用方回 404——**不报错、不抛**：
      超出 z6 的级别本来就该静默交给矢量底图兜底。
    ★ **行号口径**：对外（路由/前端）一律用 **XYZ 行号（北起 0）**，
      只有 ``_query`` 落库那一刻才翻成 TMS。这样"翻转"只有一处，
      不会在这里翻了、那里又翻一次。
    """

    def __init__(self, mbtiles: Path, ext: str = "jpg") -> None:
        self.path = mbtiles
        self.ext = ext
        self.mime = _TILE_MIME.get(ext.lower(), "application/octet-stream")
        self.ok = mbtiles.is_file()
        self._wh = (0, 0)   # 每级 (min_zoom, max_zoom)，懒查

    def _connect(self) -> sqlite3.Connection:
        # 只读打开（``mode=ro``）：这个库归 AFSIM 所有，我们只借来读。
        uri = "file:%s?mode=ro" % self.path.as_posix()
        return sqlite3.connect(uri, uri=True, timeout=5.0)

    def zoom_range(self) -> tuple[int, int]:
        """返回库里的 ``(min_zoom, max_zoom)``；查不到给 ``(0, 0)``。"""
        if self._wh != (0, 0) or not self.ok:
            return self._wh
        try:
            con = self._connect()
            try:
                row = con.execute(
                    "SELECT MIN(zoom_level), MAX(zoom_level) FROM tiles"
                ).fetchone()
            finally:
                con.close()
            if row and row[0] is not None:
                self._wh = (int(row[0]), int(row[1]))
        except sqlite3.Error:
            pass
        return self._wh

    def get_raw(self, z: int, col: int, row: int) -> bytes | None:
        """按**等经纬度格号**（行号 XYZ、北起）取瓦片字节。

        ``row`` 是 **XYZ 口径（北起 0）**；落库时才翻成 TMS。
        """
        if not self.ok:
            return None
        if col < 0 or row < 0 or col >= EQ_COLS(z) or row >= EQ_ROWS(z):
            return None
        return self._query(z, col, row)

    def present_set(self, z: int) -> set:
        """第 ``z`` 级里**存在**的 ``(col, row)`` 集合（**XYZ 行号**）。"""
        if not self.ok:
            return set()
        try:
            con = self._connect()
            try:
                rows = con.execute(
                    "SELECT tile_column, tile_row FROM tiles WHERE zoom_level=?",
                    (z,)).fetchall()
            finally:
                con.close()
        except sqlite3.Error:
            return set()
        # ★ 翻成 XYZ 再对外：清单里的行列号要和拼图/前端同一口径。
        return {(int(a), tms_to_xyz_row(z, int(b))) for a, b in rows}

    def _query(self, z: int, col: int, row: int) -> bytes | None:
        """真正的查库动作（``row`` 是 **XYZ** 口径，这里翻成 TMS）。

        不做范围校验，调用方负责。
        """
        try:
            con = self._connect()
            try:
                q = con.execute(
                    "SELECT tile_data FROM tiles WHERE zoom_level=? AND "
                    "tile_column=? AND tile_row=?",
                    (z, col, tms_to_xyz_row(z, row))
                ).fetchone()
            finally:
                con.close()
        except (sqlite3.Error, OverflowError, ValueError):
            return None
        if not q:
            return None
        return bytes(q[0])

    def get(self, z: int, x: int, y: int) -> bytes | None:
        """瓦片字节。``(x, y)`` 是**等经纬度格号（行号 XYZ、北起）**。

        ★ 前端只负责"经纬度 → 等经纬度格号"这一件纯几何的事；
          **TMS 行翻转在服务端**（``_query`` 里做），见 §2.5 的实测判据。
        """
        if not self.ok:
            return None
        # ★ 先卡 zoom 上限再算 ``1 << z``：z 是 URL 里来的任意整数，
        #   不设防的话 z=99 会算出一个 2**99 的行号，
        #   sqlite 绑参时抛 OverflowError（不是 sqlite3.Error，兜不住）。
        lo, hi = self.zoom_range()
        if z < lo or z > hi:
            return None
        return self.get_raw(z, x, y)


#: 路径形态 ``/tiles/<z>/<x>/<y>.<ext>``
_TILE_RE = re.compile(r"^/tiles/(\d+)/(\d+)/(\d+)\.([A-Za-z0-9]+)$")

#: ★★ 底图投影口径 —— 这条是"地图画不出来"的**总根因**，别再翻案 ★★
#:
#: 库（``bmng.mbtiles``）是 **plate carrée（等经纬度）2:1** 版面：
#: 列 ``2*2^z``、行 ``2^z``，整幅 = lon -180~180 / lat 90~-90。
#: 这一点由 AFSIM **自己的**地图定义文件一锤定音 ——
#: ``resources/maps/bluemarble_db/bmng_flat.earth``：
#:     ``<map type="projected"><options><profile>plate-carre</profile>``
#: 也就是同一个 mbtiles 被 AFSIM 当 plate-carrée 用，我的推导与它一致。
#:
#: 但**腾讯 GL JS 要的是 Web 墨卡托（EPSG:3857）XYZ 瓦片**。
#: 实测坐实（探针 g19）：
#:   ``ImageTileLayer.getTileUrl`` 被调 5 次，参数是 ``z=8, x=210, y=97``
#:   —— 反算 ``lon=115.31, lat=21.80``，正是 Web 墨卡托格号；
#:   更早还调过一次 ``z=0, x=0, y=0``。而我们库里 **z0 有 2 列 1 行**，
#:   根本没有"第 0 级只有 1 块"的版式。
#:   ⇒ SDK 拿不到它要的瓦片，**静默丢弃**：``getTileUrl`` 被调、
#:     网络里 **0 条** ``/tiles/`` 请求、画布上一个像素都不画。
#:
#: 结论（踩过的坑，别再回头）：
#:   * ``ImageTileLayer``：调 getTileUrl 但不取 ⇒ 不出图
#:   * ``ImageGroundLayer``：构造成功、``_image`` 已解码、登记为
#:     ``IMAGE_Ground``、``visible=true``，但同样不出图
#:   * SDK 也没有 ``TMap.TileLayer``（只有 ``ImageTileLayer`` /
#:     ``WMTSLayer`` / ``ImageGroundLayer``）
#:   * 给地图挂官方 ``setBaseMap({type:'satellite'})`` 也救不回来
#:
#: ⇒ **任何 SDK 栅格图层都不要再用**。要让 SDK 能取到瓦片，唯一的路是
#:   **服务端把 plate-carrée 源重投影成 Web 墨卡托瓦片**再吐给它
#:   （见 ``_merc_tile``）。前端那边用 ``ImageTileLayer`` 指到
#:   ``/tiles/<z>/<x>/<y>.jpg`` 即可。


def _merc_xy_to_lonlat(z: int, x: int, y: int) -> tuple[float, float]:
    """Web 墨卡托格号 ``(z, x, y)`` 的瓦片**左上角**经纬度。"""
    import math
    n = float(1 << z)
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lon, lat


def _lonlat_to_eq_xy(z: int, lon: float, lat: float) -> tuple[float, float]:
    """经纬度 → **等经纬度**连续格号（``(col, row)``，行号北起）。"""
    ncol = float(2 * (1 << z))
    nrow = float(1 << z)
    return ((lon + 180.0) / 360.0 * ncol, (90.0 - lat) / 180.0 * nrow)


def _merc_tile(tiles: "TileProvider", z: int, x: int, y: int,
               out: int = 256) -> bytes | None:
    """把请求的**墨卡托**瓦片重投影成一张 ``out x out`` 的 JPEG。

    ★★ v0.13.48 起改为**条带式**取样，不再依赖整幅拼接（``_stitch_level``）。

      旧实现要先把整幅等经纬度源图拼出来（z5 约 2.8 s / z6 约 9 s，
      还要占几百 MB 内存），首块瓦片必须等整幅拼完 —— 这正是用户报的
      "加载太慢"的主因。新实现只取**这一块瓦片覆盖到的源库小块**：

      ① 反解该墨卡托瓦片的经纬度矩形（左上/右下角）；
      ② 换算成源库（等经纬度）在第 ``lvl = min(z, 库内最高级)`` 级的
         连续格号，圈出覆盖到的源瓦片（通常 1~10 块）；
      ③ 只从 sqlite 取这几块，贴成一张小条带图；
      ④ 在条带内做与旧版相同的 MESH 重投影。

    ⇒ 单块瓦片几十毫秒、内存可忽略；``ThreadingTCPServer`` 每连接一线程，
      浏览器本来就是**并发**取瓦片的 ⇒ 服务端重投影天然并行。

    ★★ **越级放大（overzoom）顺带解决**：``z > 库内最高级`` 时直接从最高级
      源图连续坐标采样（源分辨率撑得住，只是放得更糊），服务端照样出图。
      前端把 ``maxDataZoom`` 放开即可 —— 修"放大后地形没了"。

    ★ MESH 口径（踩过的坑，别改）：
      * ``box`` 必须是**整数**像素边界（``i*out//N``）；
      * ``[(box, quad)]`` 是"目标矩形 ← 源四边形"，方向别反；
      * 必须分 16x16 块：墨卡托纬度是非线性的，整块一个 quad 高纬会"分层"。
    """
    lo, hi = tiles.zoom_range()
    if not tiles.ok or z < lo:
        return None
    lvl = z if z <= hi else hi      # 源采样级：越级放大时用库内最高级

    # 结果缓存：拖动/缩放往返会反复要同一块，重投影再快也不必重算
    key = (z, x, y)
    with _MERC_LOCK:
        hit = _MERC_CACHE.get(key)
        if hit is not None:
            _MERC_CACHE.move_to_end(key)
            return hit

    lon0, lat0 = _merc_xy_to_lonlat(z, x, y)          # 左上（北西角）
    lon1, lat1 = _merc_xy_to_lonlat(z, x + 1, y + 1)  # 右下（南东角）

    # 源库在该级的版面（等经纬度）：列 2*2^lvl、行 2^lvl，每格 256 px
    ncol, nrow = 2 * (1 << lvl), 1 << lvl
    # 覆盖到的源**连续**格号（浮点；行号北起，与 get_raw 的 XYZ 口径一致）
    cf0 = (lon0 + 180.0) / 360.0 * ncol
    cf1 = (lon1 + 180.0) / 360.0 * ncol
    rf0 = (90.0 - lat0) / 180.0 * nrow
    rf1 = (90.0 - lat1) / 180.0 * nrow
    c0 = max(0, min(ncol - 1, int(math.floor(cf0))))
    c1 = max(0, min(ncol - 1, int(math.floor(cf1 - 1e-9))))
    r0 = max(0, min(nrow - 1, int(math.floor(rf0))))
    r1 = max(0, min(nrow - 1, int(math.floor(rf1 - 1e-9))))

    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    strip = Image.new("RGB", ((c1 - c0 + 1) * 256, (r1 - r0 + 1) * 256),
                      (8, 10, 16))
    for row in range(r0, r1 + 1):
        for col in range(c0, c1 + 1):
            blob = tiles.get_raw(lvl, col, row)
            if blob is None:
                continue
            try:
                im = Image.open(io.BytesIO(blob)).convert("RGB")
            except Exception:
                continue
            if im.size != (256, 256):
                im = im.resize((256, 256))
            strip.paste(im, ((col - c0) * 256, (row - r0) * 256))

    n = float(1 << z)

    def src_pt(u: float, v: float) -> tuple[float, float]:
        """目标归一化坐标 (u,v) ∈ [0,1] → 条带内的源像素坐标。"""
        lon = lon0 + (lon1 - lon0) * u
        my = (y + v) / n
        lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * my))))
        sx = ((lon + 180.0) / 360.0 * ncol - c0) * 256.0
        sy = ((90.0 - lat) / 180.0 * nrow - r0) * 256.0
        return (sx, sy)

    N = 16
    mesh = []
    for j in range(N):
        for i in range(N):
            # 目标这一小块的矩形（★ 必须是**整数**像素边界）
            dx0 = i * out // N
            dy0 = j * out // N
            dx1 = (i + 1) * out // N
            dy1 = (j + 1) * out // N
            u0, u1 = i / N, (i + 1) / N
            v0, v1 = j / N, (j + 1) / N
            p00 = src_pt(u0, v0)
            p10 = src_pt(u1, v0)
            p11 = src_pt(u1, v1)
            p01 = src_pt(u0, v1)
            mesh.append(((dx0, dy0, dx1, dy1),
                         (p00[0], p00[1], p10[0], p10[1],
                          p11[0], p11[1], p01[0], p01[1])))
    out_im = strip.transform((out, out), Image.MESH, mesh,
                             resample=Image.BILINEAR)
    buf = io.BytesIO()
    out_im.save(buf, format="JPEG", quality=85, optimize=True)
    blob = buf.getvalue()
    with _MERC_LOCK:
        _MERC_CACHE[key] = blob
        while len(_MERC_CACHE) > _MERC_CACHE_MAX:
            _MERC_CACHE.popitem(last=False)
    return blob


#: 单块墨卡托瓦片的结果缓存：键 ``(z, x, y)``，LRU 上限防长会话内存无界。
#: ★ 浏览器侧另有 ``Cache-Control`` 缓存（见 ``_handle_tile``），两层配合：
#:   浏览器缓存管"刷新/回看不重发"，这里管"发了也秒回"。
_MERC_CACHE: "OrderedDict[tuple[int, int, int], bytes]" = OrderedDict()
_MERC_CACHE_MAX = 4096
_MERC_LOCK = threading.Lock()


#: 拼好的整幅底图缓存：``{z: jpeg_bytes}``。整幅 z5 约 4~5 MB，
#: 拼一次 2~3 s，但每次页面刷新都重拼很浪费，缓存住。
_SATMAP_CACHE: dict[int, bytes] = {}

#: 正在后台拼的级（防止同一个 z 被并发拼两次）。
_SATMAP_BUSY: set[int] = set()

#: 拼图锁：``_SATMAP_CACHE`` / ``_SATMAP_BUSY`` 会被多线程碰（每连接一线程）。
_SATMAP_LOCK = threading.Lock()

#: 默认拼到哪一级。★ 用 z5 不用 z6：见 ``_stitch_level`` 的坑②。
#: z5 = 4096x2048（2048 块，约 4.6 MB，~2.4 s），
#: z6 = 16384x8192（8192 块，约 45 MB，~9 s）。
#: BMNG 源图是 21600x10800，z5 放大到 z6 只差 1.33 倍采样，
#: 肉眼几乎无差别，体积差 10 倍 —— 选 z5。
DEFAULT_SATMAP_Z = 5


def _stitch_level(tiles: "TileProvider", z: int) -> bytes | None:
    """把第 ``z`` 级的所有瓦片拼成**一张整图**（JPEG 字节）。

    ★ 为什么要"服务端拼成一张"而不是让前端逐块取瓦片：
      腾讯 GL JS 的 ``ImageTileLayer`` 在本项目**代理模式**下不会主动回调
      ``getTileUrl``（实测：图层已 ``setMap``、``zIndex`` 正常、手动调
      ``getTileUrl`` 返回正确 URL，但 SDK 一次都不自己调，网络里 0 条
      ``/tiles/`` 请求）。既然 SDK 不肯取瓦片，就让服务端**拼好一整幅**，
      前端当普通 ``<img>`` 垫在地图 canvas 下面 —— 这条路不碰 SDK 内部机制，
      行为完全可预测。

    ★ 依赖：Pillow（``pyproject.toml`` 里是 optional 的 ``satmap`` 组）。
      没装就走 ``_tiles_manifest`` 那条前端拼图的后备路径。

    ★ 等经纬度网格：列 2*2^z、行 2^z，逐格贴；整幅 = lon -180~180、
      lat 90~-90，正是等经纬度图的自然展开（与 AFSIM 的 ``about.txt``
      里 ``-a_ullr -180 90 -90 0`` 一致）。

    ★★ 三个实测数值坑（都踩过，别再改回去）★★
      ① **反炸弹上限**：z5 整幅 = 16384x8192 = **134,217,728 像素**，
         z6 整幅 = 32768x16384 = **536,870,912 像素** —— 后者远超 Pillow
         默认的 ``MAX_IMAGE_PIXELS``（178,956,970）⇒ ``Image.new`` 本身
         不报，但任何 *后续读取* 这张图都会抛 ``DecompressionBombError``。
         建图前必须把上限关掉。
      ② **体积**：z5 = 16384x8192（2048 块）存 JPEG **约 12.6 MB / 2.8 s**；
         z6 = 32768x16384（8192 块）**约 45 MB / 9 s**。
         BMNG 源图本身只有 21600x10800 宽，z5 已经覆盖到 16384 宽
         （源分辨率的 76%），再往上级只是把同一张源图放大 ⇒
         **默认拼 z5**（见 ``DEFAULT_SATMAP_Z``），``?z=6`` 才要 z6。
      ③ **耗时**：2048 次 sqlite 查询 + 2048 次 JPEG 解码约 2.8 s，
         属于"拼一次就缓存住"的一次性成本；但首屏等待仍偏久，所以：
         **拼图放到后台线程**，HTTP 请求先回 202，前端隔一会儿重来。
         （见 ``_handle_satmap``。）
    """
    if not tiles.ok:
        return None
    lo, hi = tiles.zoom_range()
    if z < lo or z > hi:
        return None
    with _SATMAP_LOCK:
        if z in _SATMAP_CACHE:
            return _SATMAP_CACHE[z]
        if z in _SATMAP_BUSY:
            return None          # 别的人正在拼，让调用方去等/重试
        _SATMAP_BUSY.add(z)
    try:
        return _stitch_level_inner(tiles, z)
    finally:
        with _SATMAP_LOCK:
            _SATMAP_BUSY.discard(z)


def _stitch_level_inner(tiles: "TileProvider", z: int) -> bytes | None:
    """真正干活的那半（锁已由 ``_stitch_level`` 管住）。

    ★ 版面 = 等经纬度 2:1：列 ``2*2^z``、行 ``2^z``，逐格贴。
      **行号一律走 ``tiles.get_raw``（内部统一做 TMS 翻转）**，
      所以这里的 ``row`` 是 XYZ 口径、北起 0 —— 贴到画布上也按北起。
    """
    try:
        from PIL import Image
    except ImportError:
        return None

    # ★ 见坑①：z5/z6 整幅分别是 1.3 亿 / 5.4 亿像素，开着上限连
    #   "读回自己"都会抛。
    Image.MAX_IMAGE_PIXELS = None

    n = 1 << z
    ncol, nrow, size = 2 * n, n, 256
    sheet = Image.new("RGB", (ncol * size, nrow * size), (8, 10, 16))
    got = 0
    for row in range(nrow):
        for col in range(ncol):
            blob = tiles.get_raw(z, col, row)   # row = XYZ（北起）
            if blob is None:
                continue
            try:
                im = Image.open(io.BytesIO(blob)).convert("RGB")
            except Exception:
                continue
            if im.size != (size, size):
                im = im.resize((size, size))
            sheet.paste(im, (col * size, row * size))
            got += 1
    if not got:
        return None
    buf = io.BytesIO()
    # ★ 见坑②：z5 约 12.6 MB、z6 约 45 MB —— 所以默认级用 z5。
    sheet.save(buf, format="JPEG", quality=80, optimize=True,
               progressive=True)
    data = buf.getvalue()
    with _SATMAP_LOCK:
        _SATMAP_CACHE[z] = data
    return data


def _tiles_manifest(tiles: "TileProvider", z: int) -> dict[str, Any] | None:
    """后备路径：给前端一张"该取哪些瓦片"的清单，由浏览器 canvas 自己拼。

    ★ 只在没装 Pillow 时用。见 ``_stitch_level`` 的说明。
    清单里的 ``(col, row)`` 是**等经纬度格号、行号 XYZ（北起 0）**
    （``present_set`` 已把 TMS 翻过了），前端按
    ``lon = col/(2n)*360-180``、``lat = 90 - row/n*180`` 直接算即可。
    """
    if not tiles.ok:
        return None
    lo, hi = tiles.zoom_range()
    if z < lo or z > hi:
        return None
    n = 1 << z
    return {
        "z": z, "cols": 2 * n, "rows": n, "tile_size": 256,
        "ext": tiles.ext,
        "lon0": -180.0, "lon1": 180.0, "lat0": 90.0, "lat1": -90.0,
        "tiles": sorted(tiles.present_set(z)),
    }


def find_tile_mbtiles(repo_root: Path) -> Path | None:
    """找 BMNG 瓦片库。环境变量优先，其次 AFSIM 2.9 标准布局。"""
    import os
    env = os.environ.get("MILSIM_TILE_MBTILES")
    if env:
        p = Path(env)
        return p if p.is_file() else None
    for cand in [
        repo_root.parent / DEFAULT_TILE_MBTILES,   # F:/afsim2.9cn/resources/...
        repo_root / DEFAULT_TILE_MBTILES,
    ]:
        if cand.is_file():
            return cand
    return None


# ===========================================================================
# 3. HTTP + WebSocket 服务
# ===========================================================================

def make_handler(root: Path, session_factory: Any,
                 tiles: TileProvider | None = None) -> Any:
    """造一个 handler 类。``root`` = 静态文件根（app/）；``tiles`` 可空。"""
    import http.server

    class Handler(http.server.SimpleHTTPRequestHandler):
        #: 关闭每请求日志（会刷屏，而回放页每秒可能连好几次）
        protocol_version = "HTTP/1.1"

        # ★ 每连接一线程（ThreadingTCPServer 的默认 _threads=None 会
        #   给每个连接记一个 Thread 对象并 join，长连接 + 大响应的组合
        #   下反而添乱）。
        daemon_threads = True

        # ★ /satmap 的响应是 3~13 MB 的整幅 JPEG。若走 HTTP/1.1
        #   keep-alive，同一个 socket 上前面若有一个**没读完**的响应
        #   （例如浏览器中途取消的请求），下一个响应就会被夹在里面 ⇒
        #   浏览器只看到半张图 ⇒ ``net::ERR_FAILED`` / ``Failed to
        #   fetch``（服务端日志却记着 200）。所以只对这条路径回
        #   ``Connection: close``，让它一个请求一条新连接。

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, directory=str(root), **kwargs)

        def log_message(self, fmt: str, *args: Any) -> None:
            msg = fmt % args
            # 瓦片请求量大且无信息量，只记非 200 的
            if "GET /ws" in msg:
                return
            if "GET /tiles/" in msg and " 200 " in msg:
                return
            sys.stderr.write("[http] " + msg + "\n")

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            if path == "/ws":
                self._handle_ws()
                return
            if path == "/satmap":
                self._handle_satmap()
                return
            if path.startswith("/tiles/"):
                if self._handle_tile(path):
                    return
            super().do_GET()

        # -- 整幅卫星底图（自绘） ------------------------------------------
        def _handle_satmap(self) -> None:
            """把 mbtiles 的某一级**拼成一张整图**发给前端当静态底图。

            ★ 为什么要有这条路（而不是让前端逐块取瓦片）：
              腾讯 GL JS 的 ``ImageTileLayer`` 在本项目**代理模式**下不会主动
              回调 ``getTileUrl`` 去取瓦片（实测：图层已 ``setMap``、``zIndex``
              正常、手动调 ``getTileUrl`` 返回正确 URL，但 SDK 一次都不自己调，
              网络里 0 条 ``/tiles/`` 请求）。既然如此，就别把"取瓦片"这件
              事交给 SDK —— **服务端拼好一整幅，前端当普通 ``<img>`` 垫底**。
              这条路不碰 SDK 内部机制，行为完全可预测。

            ``?z=6`` 指定级（缺省取 ``DEFAULT_SATMAP_Z``）；
            ``?format=json`` 要清单（没装 Pillow 时的前端拼图后备路径）。

            ★ 异步：首次请求某一级时，若缓存里没有，就**开后台线程去拼**，
              立刻回 ``202`` + 一个 ``{"state":"stitching"}`` 的 JSON，
              前端隔一会儿重来（见 ``app/index.html`` 的 ``loadSatMap``）。
              拼图要 2~9 s，同步等会让首屏一直白 —— 这个 202 就是为它设计的。
              拼完再请求就是 200 + JPEG。
            """
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            if tiles is None or not tiles.ok:
                self.send_error(404, "tile source unavailable")
                return
            lo, hi = tiles.zoom_range()
            try:
                z = int((q.get("z") or [DEFAULT_SATMAP_Z])[0])
            except (TypeError, ValueError):
                z = DEFAULT_SATMAP_Z
            # ★ 缺省级可能超出库里范围（比如换成别的库）⇒ 夹回来
            z = max(lo, min(hi, z))
            fmt = ((q.get("format") or ["jpeg"])[0] or "jpeg").lower()

            if fmt == "json":
                man = _tiles_manifest(tiles, z)
                if man is None:
                    self.send_error(500, "manifest failed")
                    return
                body = json.dumps(man, ensure_ascii=False,
                                  separators=(",", ":")).encode("utf-8")
                self._send_bytes(body, "application/json; charset=utf-8")
                return

            with _SATMAP_LOCK:
                cached = _SATMAP_CACHE.get(z)
                busy = z in _SATMAP_BUSY
            if cached is None:
                if not busy:
                    # 开后台线程拼（拼完自己塞进缓存）
                    threading.Thread(target=_stitch_level,
                                     args=(tiles, z), daemon=True).start()
                # 202 = "接着呢"，前端稍后重试
                body = json.dumps(
                    {"state": "stitching", "z": z},
                    separators=(",", ":")).encode("utf-8")
                self.send_response(202)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except OSError:
                    pass
                return

            self._send_bytes(cached, "image/jpeg", close=True)

        def _send_bytes(self, blob: bytes, ctype: str,
                        close: bool = False) -> None:
            """回一段字节（带 no-store）。``close=True`` 则用完就关连接。"""
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(blob)))
            self.send_header("Cache-Control", "no-store")
            if close:
                self.send_header("Connection", "close")
                self.close_connection = True
            self.end_headers()
            try:
                self.wfile.write(blob)
            except OSError:
                pass

        # -- 卫星影像瓦片 --------------------------------------------------
        def _handle_tile(self, path: str) -> bool:
            """``/tiles/z/x/y.ext`` → **Web 墨卡托**瓦片（服务端重投影）。

            ★★ 别再往"直接从库里抠 XYZ 格"改回去 ★★
              SDK 要的是墨卡托格号；库里是 plate-carrée。
              直接抠出来的格对不上，SDK 会当作取不到而**静默丢弃**
              （实测 ``getTileUrl`` 被调、网络 0 请求、画布全空）。
              见 ``_merc_tile`` 上面的长注释。

            ★ v0.13.48：``z`` 允许**超过库内最高级**（overzoom，服务端
              从最高级源图连续坐标采样出图）；只挡 ``z > 30`` 这种明显
              乱来的 URL。

            ``?raw=1`` 保留原来的"直取等经纬度格"行为，给离线调试用。

            ★ 缓存头：底图数据是**静态**的（库文件运行期不变）⇒ 回
              ``max-age=86400`` 让浏览器缓存住 —— 刷新/回看不重发，
              这比任何服务端优化都更能改善"加载慢"的体感。
              （静态资源 ``index.html`` 等不走这里，仍按原有策略。）
            """
            m = _TILE_RE.match(path)
            if not m:
                return False
            if tiles is None or not tiles.ok:
                self.send_error(404, "tile source unavailable")
                return True
            z, x, y, ext = (int(m.group(1)), int(m.group(2)),
                            int(m.group(3)), m.group(4).lower())
            # ★ URL 里的 z 可能是任意位数（/tiles/999999.../0/0.jpg）。
            #   先卡一个宽松上限，避免把天文数字送进 sqlite 绑参。
            if z > 30 or x > (1 << 30) or y > (1 << 30):
                self.send_error(404, "tile out of range")
                return True

            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            raw = (q.get("raw") or ["0"])[0] not in ("0", "", "false")

            if raw:
                blob = tiles.get(z, x, y)
                ctype = _TILE_MIME.get(ext, tiles.mime)
            else:
                # ★ 墨卡托格号必须在 [0, 2^z) 内
                n = 1 << z
                if x < 0 or y < 0 or x >= n or y >= n:
                    self.send_error(404, "tile out of mercator range")
                    return True
                blob = _merc_tile(tiles, z, x, y)
                ctype = "image/jpeg"

            if blob is None:
                # 库里没有这一格（超范围/空洞）⇒ 404 让前端保持透明。
                self.send_error(404, "no such tile")
                return True
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(blob)))
            # ★ 底图是静态数据 ⇒ 允许浏览器缓存（刷新/回看零请求）。
            #   别改回 no-store——那是"加载慢"的一大半原因。
            self.send_header("Cache-Control", "max-age=86400")
            self.end_headers()
            try:
                self.wfile.write(blob)
            except OSError:
                pass
            return True

        # -- WebSocket 升级 ------------------------------------------------
        def _handle_ws(self) -> None:
            key = self.headers.get("Sec-WebSocket-Key")
            if not key or "websocket" not in (
                self.headers.get("Upgrade", "").lower()
            ):
                self.send_error(400, "not a websocket handshake")
                return

            # 按 URL 上的 query 决定放哪份想定、按什么节拍推
            #   mode=fast ⇒ 一键跑完（pace 0）；缺省/其它 ⇒ 边跑边看（按时间轴）
            from urllib.parse import parse_qs, urlparse
            query = parse_qs(urlparse(self.path).query)
            scenario = (query.get("scenario") or [None])[0]
            mode = ((query.get("mode") or [""])[0] or "").lower()

            self.send_response(101, "Switching Protocols")
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", ws_accept_key(key))
            self.end_headers()

            sock = self.connection
            try:
                # ★ 会话键 =（想定, 模式）。**不能只用想定**：
                #   一个会话是**一次性**的（`_gate` 只放行一次、跑完就 done），
                #   而且 `mode=fast` 会把它的 `pace` 永久改成 0。
                #   若两个档共用一份会话，"后点的那个"连上的是**已经跑完的**
                #   会话 ⇒ 只收得到 hello、一帧没有，且 `pace` 已被前一个污染。
                #   实测判据（tools/_gui 探针）：同一想定先 fast 后 live，
                #   前者收 11 帧+done，后者收 0 帧。见设计文档 §5.16.14。
                session = session_factory(scenario, mode)
                # ★ 先放行仿真（若这是第一个订阅者），再等 hello 备好。
                #   顺序反了会死锁：没人 attach ⇒ gate 不放行 ⇒ 永不出 hello。
                session.attach(sock)
                # 节拍已在**建会话时**按 mode 定死（fast ⇒ pace 0），
                # 这里不再改——改共享会话的 pace 会波及同会话的其它订阅者。
                # ★ 已推过的帧**不重放**：这是"实时"，从头补放是"离线"的事
                #   （那样两件事的语义又混了）。要完整回放请用离线文件
                #   或再连一次（会新开一份会话从头推）。
                while True:
                    got = ws_read_frame(sock)
                    if got is None:
                        break
                    opcode, _payload = got
                    if opcode == 0x8:  # close
                        break
                    if opcode == 0x9:  # ping → pong（保持连接）
                        try:
                            sock.sendall(bytes([0x8A, 0]))
                        except OSError:
                            break
            except (OSError, ValueError):
                pass
            finally:
                try:
                    session.detach(sock)  # type: ignore[possibly-undefined]
                except UnboundLocalError:
                    pass
                self.close_connection = True

    return Handler


def parse_args(argv: list[str]) -> dict[str, Any]:
    cfg: dict[str, Any] = {
        "scenario": DEFAULT_SCENARIO,
        "port": DEFAULT_PORT,
        "seconds": DEFAULT_SECONDS,
        "sample": 0.0,
        "demo": "",
        "pace": None,          # None ⇒ 用 sample（真实时）
        "libraries": [],
        "list": False,
    }
    rest = argv[1:]
    i = 0
    while i < len(rest):
        a = rest[i]
        if a == "--list":
            cfg["list"] = True
            i += 1
        elif a in ("--port", "--seconds", "--sample", "--demo", "--pace",
                   "--library") and i + 1 < len(rest):
            v = rest[i + 1]
            if a == "--port":
                cfg["port"] = int(v)
            elif a == "--seconds":
                cfg["seconds"] = float(v)
            elif a == "--sample":
                cfg["sample"] = float(v)
            elif a == "--pace":
                cfg["pace"] = float(v)
            elif a == "--demo":
                cfg["demo"] = v
            else:
                cfg["libraries"].append(v)
            i += 2
        elif a.startswith("-"):
            print(f"无法识别的参数：{a}", file=sys.stderr)
            raise SystemExit(2)
        else:
            cfg["scenario"] = a
            i += 1
    return cfg


def list_scenarios(root: Path) -> list[str]:
    d = root / "scenarios"
    if not d.is_dir():
        return []
    return sorted(p.name for p in d.glob("*.txt"))


def main(argv: list[str]) -> int:
    import socketserver

    cfg = parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    app_dir = root / "app"

    if cfg["list"]:
        print("可推的想定：")
        for name in list_scenarios(root):
            print(f"  scenarios/{name}")
        return 0

    if not app_dir.is_dir():
        print(f"找不到 {app_dir}——先跑一次 tools/export_replay.py 生成 app/",
              file=sys.stderr)
        return 1

    scenario = cfg["scenario"]
    if not (root / scenario).exists():
        print(f"找不到想定：{scenario}", file=sys.stderr)
        return 1

    # ★ pace 决定"看起来是不是实时的"。
    #   默认（没写 --pace）= 按采样间隔真实推进；--pace 0 = 尽快推完。
    #   为什么不让默认是 0：那样 60 s 的想定 1 s 内推完，人眼只看到残影，
    #   等于没推。"实时"本来就该按时间轴走。想尽快跑完请显式写 --pace 0。
    pace = cfg["pace"]
    if pace is None:
        pace = cfg["sample"] if cfg["sample"] > 0.0 else 2.0
        # sample 没给就退 2 s（多数想定的 max_step 就是这个量级，
        # 顶多多等/少等一点，不影响正确性）。

    #: ★ 缓存键 = ``(想定, 模式)``。理由见 ``_handle_ws`` 里的注释：
    #:   一个会话跑完就结束了，"一键跑完"与"边跑边看"必须是**两份**会话，
    #:   否则后点的那个只能收到 hello。
    _cache: dict[tuple[str, str], StreamSession] = {}
    #: ★ 会话缓存并发保护：多个页面同时连进来时，"判 done → 重开"这步
    #:   必须原子（否则两人各开一份，后一份把前一份顶掉）。
    _cache_lock = threading.Lock()

    def _make(scenario_path: str, mode: str) -> StreamSession:
        """按 (想定, 模式) 造一份会话。``fast`` ⇒ pace 0（尽快推完）。"""
        # ★ pace 在建会话时就定死，不再事后 ``set_pace`` 改共享对象
        #   （那会波及同一会话的其它订阅者）。
        this_pace = 0.0 if mode == "fast" else pace
        return StreamSession(
            scenario_path, seconds=cfg["seconds"], sample=cfg["sample"],
            demo=cfg["demo"] if scenario_path == scenario else "",
            pace=this_pace, libraries=cfg["libraries"])

    def session_factory(requested: str | None,
                        mode: str = "") -> StreamSession:
        """按 (想定, 模式) 取会话；没有就新开一份并起线程。

        ★ 全部**懒开**：不预先建任何会话。这样"没人连就不跑仿真"
          （`_gate` 那条设计）与"两个档各自一份"同时成立——
          谁第一个连进来，就用谁的模式开第一份会话。

        ★★ 会话是**一次性**的（跑完 ``done=True`` 就永远不再产帧）。
          跑完的会话若还留在缓存里，后连进来的人**只收到 hello、
          一帧没有、done 也不会补发**——前端就停在 ``0/0``，点播放
          因 ``frames.length < 2`` 直接 return，看上去"整个页面死了"。
          （实测踩过：用户点过一次"一键跑完"后，之后无论怎么切档、
          重开页面、换浏览器都收不到帧。）⇒ 命中"已结束"的缓存键时
          **必须重开一份新的从头推**——这与 ``_handle_ws`` 里"要完整
          回放就再连一次"的语义正好对齐。线程死了但 ``done`` 没置上的
          情况（``running=False``）一并回收，不留僵尸会话。
        """
        key = (requested or scenario, mode or "")
        with _cache_lock:
            old = _cache.get(key)
            if old is not None and not old.done and old.running:
                return old
            _cache[key] = _make(key[0], key[1])
            _cache[key].start()
            return _cache[key]

    # 卫星影像瓦片：AFSIM 自带的 NASA Blue Marble NG（本地 mbtiles）。
    # 找到就挂 /tiles 路由；找不到只是底图没有卫星影像，不影响推流。
    tile_path = find_tile_mbtiles(root)
    tiles = TileProvider(tile_path) if tile_path else None

    handler = make_handler(app_dir, session_factory, tiles)
    socketserver.TCPServer.allow_reuse_address = True
    try:
        with socketserver.ThreadingTCPServer(("127.0.0.1", cfg["port"]),
                                             handler) as httpd:
            url = f"http://127.0.0.1:{cfg['port']}/index.html"
            print(f"想定：{scenario}")
            print(f"时长 {cfg['seconds']:.0f} s"
                  f"（采样 {cfg['sample'] or '想定 max_step'} s，"
                  f"启动节拍 {pace if pace else '尽快'} ）")
            if tiles and tiles.ok:
                lo, hi = tiles.zoom_range()
                print(f"卫星底图：{tile_path}（zoom {lo}~{hi}，"
                      f"{tile_path.stat().st_size / 1048576.0:.0f} MB）")
            else:
                print("卫星底图：未找到 bmng.mbtiles，底图退回矢量图"
                      "（可用 MILSIM_TILE_MBTILES 指定路径）")
            print(f"打开：{url}")
            print("左栏「数据源」：一键跑完 / 边跑边看（都不需要重启服务）")
            print("（Ctrl-C 停止）")
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n停止。")
    finally:
        for s in _cache.values():
            s.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
