# milsim · 任务级军事仿真器 / Task-level Military Simulation Framework

[中文](#中文) | [English](#english)

---

<a id="中文"></a>
## 中文

**milsim** 是一个纯 Python 的任务级（实体级）作战仿真框架：自研离散事件引擎之上有完整的模型栈——五类运载器机动、雷达探测（雷达方程 / SNR / Pd / 波束扫描）、通信网（航迹分发）、电子干扰（压制）、规则与 LLM 决策、地图网格与寻路——外加一套文本想定格式和 Web 态势回放页。核心仿真层零第三方依赖（仅 `numpy` 用于全场实体并行推进）。

### 功能特性

- **离散事件引擎**：固定步长 + 事件队列 + 消息收件箱，支持批量并行推演（`services/orchestrator`）
- **运载器**：空中 / 地面（轮式、履带）/ 水面 / 水下 / 导弹（弹道、巡航、反舰，带制导件）；巡航弹支持途径点航线（`ROUTE_CRUISE_GUIDANCE` + `route` 参数，按序过点后自动转末段导引）；反舰弹同款途径点航线 + 定高段俯仰角上限（`ROUTE_ASM_GUIDANCE` / `Phase.max_pitch_angle`，v0.13.66）
- **雷达探测**：雷达方程 → SNR → Pd（Swerling 起伏 + Marcum Q）、波束扫描相位制、地平线与通视判决、航迹管理（M/N 建航）、按精度逆方差加权融合（σ 收缩）、俯仰视场与电子扫描损失（相控阵斜视衰减，内置 AN/SPY-1D / SPY-6(V)1 型号）、TWS 航迹容量与专用跟踪波束（电扫回访 + 波束时间账本，跟踪帧率与搜索解耦）
- **通信网**：多网络组网、链路判通（SNR 判据）、航迹报文分发、掉线与延迟
- **电子战**：压制干扰（S/(N+J) 判据、极化失配）、干扰守门与被压标注
- **决策**：规则决策器 + LLM 决策器（可接任意 OpenAI 兼容接口），任务编排（Task / Intent / Ledger）
- **毁伤与武器**：战斗部命中判定（引信圈 + 最近点穿越插补、标称点数 / 装甲折扣）；**能力降级（三杀模型）**——存活目标按完好度阶梯降级机动 / 火力 / 探测三通道（M-kill / F-kill / S-kill），按类统一设置（platform_type 继承下发）、不配置则不降级，导弹类装配期禁配；武器组件（发射 / 装填、包线与阵营检查、运行期生成弹实体并交接目标）；补给组件（补弹补油）与机动油料模型
- **实体命令层**：`sim.command(实体id, 指令, 参数)` 统一入口——指令下到实体、按挂载顺序分发到组件（`move / stop / fire / reload / resupply`）
- **自主任务层**：任务管理器组件（双账本 + 四消息族走通信网 + 可靠重传 + 一对多指派），六类任务（attack / move / track / jam / comm / resupply）由挂载组件按声明认领执行，目标注销自动收尾
- **人在回路**：WebSocket 双向通道——上行指令（`route / goto / stop / pace / fire / resupply / assign_task / cancel_task`）+ 逐条 ack 回执 + 可控实体清单，浏览器里点选实体实时操控
- **地图**：经纬度与 H3 六边形双坐标系、战区网格、通视（LOS）、A* 寻路
- **想定文本格式**：可读可写的声明式想定（`scenarios/*.txt`），配套 lint 校验与参数库（想定只写型号、参数进库）
- **回放与可视化**：一键导出回放 JSON；`serve_replay.py` 起本地服务，浏览器里看实时推流 / 一键跑完 / 拖时间轴回放 / 下发操控指令，卫星底图走本地瓦片（可选）

### 安装

```bash
# Python ≥ 3.11
pip install -e .            # 核心依赖仅 numpy
pip install -e .[dev]       # + pytest / pyflakes
pip install -e .[satmap]    # + Pillow（卫星底图重投影，可选）
pip install -e .[map]       # + h3（全球六边形图层，可选）
```

### 快速开始

```bash
# 1) 跑一份想定（文本 → 仿真 → 控制台报告）
python tools/run_scenario.py scenarios/patrol.txt --seconds 120

# 2) 校验想定（语法 / 型号 / 参数，报错定位到行）
python tools/lint_scenario.py scenarios/comm_track_jam.txt

# 3) 导出离线回放 JSON（--demo mover 给带运动件的平台下达演示航线）
python tools/export_replay.py scenarios/comm_track_jam.txt \
    --seconds 60 --demo mover --out app/comm_track_jam.replay.json

# 4) 起推流服务，浏览器打开 http://127.0.0.1:8791/index.html
#    左栏可切"一键跑完 / 边跑边看"，也可直接回看第 3 步导出的离线文件
python tools/serve_replay.py scenarios/comm_track_jam.txt \
    --port 8791 --seconds 480 --sample 20 --pace 2 --demo mover

# 5) 跑测试（约 1400 条用例）
python -m pytest
```

### 想定格式一瞥

```text
simulation
    name     "参数库调用"
    max_step 2 s
    seed     20260917
end_simulation

platform_type RED_RECON : RECON_VEHICLE      # 型号参数来自参数库
    side red
    component mover  MOVER_WHEELED
    end_component
    component sensor RADAR_SEARCH_S          # 中程搜索雷达（60 km / 2 s）
    end_component
end_platform_type

zone EAST_SECTOR
    anchor     39.9042 116.4074
    radius     60 km
    resolution 200 m
end_zone

platform RED_1 RED_RECON
    position latlng 39.9042 116.3100
    heading  90 deg
end_platform
```

更多示例见 [`scenarios/`](scenarios/)（巡逻遭遇、通信传航迹 + 引导干扰、五类运载器机动等）。

### 目录结构

```
milsim/
├── src/milsim/          # 框架源码
│   ├── engine/          #   离散事件引擎（时间 / 事件 / 队列 / 收件箱）
│   ├── models/          #   运载器 / 雷达 / 通信 / 干扰 / 制导 / 武器 / 毁伤 / 任务 模型
│   ├── services/        #   想定解析 / 参数库 / 存储 / 通信 / 电子战 / 地图 / 编排
│   ├── decision/        #   规则与 LLM 决策
│   └── simulation.py    #   装配与仿真入口
├── tools/               # 命令行工具（跑想定 / 导出回放 / 推流服务 / 参数库 / lint）
├── app/                 # Web 态势页（单文件 index.html）+ 示例回放 JSON
├── scenarios/           # 示例想定（文本格式）
├── library/             # 参数库（文本 + .db）
├── tests/               # pytest 测试套件
└── docs/                # 设计文档（框架设计、雷达规格核对）
```

### 卫星底图（可选）

态势页的卫星底图默认读取本地 mbtiles 瓦片（NASA Blue Marble NG，公有领域）：

```bash
# 指向任一 plate-carrée 布局的 .mbtiles（z0 为 2 列 × 1 行）
set MILSIM_TILE_MBTILES=F:/afsim2.9cn/resources/maps/bluemarble_db/bmng.mbtiles
python tools/serve_replay.py ...
```

未提供时态势页照常工作（覆盖物 / 网格 / 关联线全正常），只是没有底图影像。

### 数据来源与致谢

- 卫星底图：NASA Blue Marble Next Generation（公有领域）
- 部分设计口径参照 AFSIM 2.9 开源架构研读笔记，代码为独立实现

## License

以 [MIT](LICENSE) 发布——可自由使用、修改、分发（保留版权声明即可）。

> 欢迎大家下载、使用本项目，并以本项目为基础，共同扩展开源仿真生态。

---

<a id="english"></a>
## English

**milsim** is a pure-Python task-level (entity-level) military simulation framework. On top of a home-grown discrete-event engine it ships a full model stack — five classes of movers, radar detection (radar equation / SNR / Pd / beam sweep), communication networks (track reporting), electronic jamming (stand-in suppression), rule-based and LLM decision-making, map grids and pathfinding — plus a declarative text scenario format and a web-based replay/situation page. The simulation core has zero third-party dependencies except `numpy` (used for vectorized per-step advancement of all entities).

### Features

- **Discrete-event engine**: fixed time step + event queue + message inboxes; batch parallel runs via the orchestrator
- **Movers**: air, ground (wheeled / tracked), surface, subsurface, and missiles (ballistic, cruise, anti-ship, with guidance components); cruise missiles fly waypoint routes (`ROUTE_CRUISE_GUIDANCE` + `route` param, auto hand-over to terminal guidance after the last waypoint); anti-ship missiles get the same waypoint route plus a pitch-angle cap on the altitude-hold segment (`ROUTE_ASM_GUIDANCE` / `Phase.max_pitch_angle`, v0.13.66)
- **Radar detection**: radar equation → SNR → Pd (Swerling fluctuations + Marcum Q), phase-based beam sweep, horizon and line-of-sight tests, track management (M/N criteria), accuracy-weighted track fusion (inverse-variance, σ shrinkage), elevation field of view and electronic scan loss (phased-array off-boresight falloff, with AN/SPY-1D / SPY-6(V)1 model entries), TWS track capacity and dedicated track beam (electronically-steered revisit with a beam-time ledger, decoupling track rate from search)
- **Communication networks**: multi-net composition, link budget (SNR criterion), track-message dissemination, dropouts and latency
- **Electronic warfare**: stand-in jamming with the S/(N+J) criterion and polarization mismatch, jam gating and suppressed-radar annotation
- **Decision**: rule-based and LLM planners (any OpenAI-compatible endpoint), task orchestration (Task / Intent / Ledger)
- **Damage & weapons**: warhead hit adjudication (fuse radius + closest-point crossing interpolation, nominal damage points / armor factor); **capability degradation (three-kill model)** — surviving targets step down mobility / firepower / sensor channels by intactness (M-kill / F-kill / S-kill), configurable per class via platform-type inheritance, off by default, missiles rejected at assembly; weapon component (fire / reload, envelope and side checks, runtime missile spawning with target hand-off); supply component (ammo & fuel) and mover fuel model
- **Entity command layer**: one entry point `sim.command(entity_id, cmd, args)` — commands go to the entity and dispatch to components by mount order (`move / stop / fire / reload / resupply`)
- **Autonomous task layer**: task-manager component (dual ledgers + four message families over the comm network + reliable retransmission + one-to-many assignment); six task types (attack / move / track / jam / comm / resupply) claimed by mounted components via a declaration protocol, with auto-resolution when targets die
- **Human-in-the-loop**: bidirectional WebSocket — uplink commands (`route / goto / stop / pace / fire / resupply / assign_task / cancel_task`), per-command acks, and a controllables list for live control from the browser
- **Map**: lat/lng and H3 hex coordinates, zone grids, line-of-sight, A* pathfinding
- **Scenario text format**: readable declarative scenarios (`scenarios/*.txt`) with a linter and a parameter library (scenarios reference model types; values live in the library)
- **Replay & visualization**: one-command replay export; `serve_replay.py` serves a local web page supporting live streaming, run-to-completion, timeline scrubbing, and interactive uplink control, with an optional local satellite-tile basemap

### Installation

```bash
# Python ≥ 3.11
pip install -e .            # core (numpy only)
pip install -e .[dev]       # + pytest / pyflakes
pip install -e .[satmap]    # + Pillow (satellite basemap reprojection, optional)
pip install -e .[map]       # + h3 (global hex layers, optional)
```

### Quick Start

```bash
# 1) Run a scenario (text → simulation → console report)
python tools/run_scenario.py scenarios/patrol.txt --seconds 120

# 2) Lint a scenario (syntax / model types / params, errors located by line)
python tools/lint_scenario.py scenarios/comm_track_jam.txt

# 3) Export an offline replay JSON (--demo mover issues demo routes to platforms with movers)
python tools/export_replay.py scenarios/comm_track_jam.txt \
    --seconds 60 --demo mover --out app/comm_track_jam.replay.json

# 4) Start the streaming server, then open http://127.0.0.1:8791/index.html
#    The sidebar switches between "run to completion" and "watch live";
#    offline JSON files from step 3 can also be replayed.
python tools/serve_replay.py scenarios/comm_track_jam.txt \
    --port 8791 --seconds 480 --sample 20 --pace 2 --demo mover

# 5) Run the test suite (~1400 cases)
python -m pytest
```

### Scenario Format at a Glance

```text
simulation
    name     "library demo"
    max_step 2 s
    seed     20260917
end_simulation

platform_type RED_RECON : RECON_VEHICLE      # type params come from the library
    side red
    component mover  MOVER_WHEELED
    end_component
    component sensor RADAR_SEARCH_S          # medium-range search radar (60 km / 2 s)
    end_component
end_platform_type

zone EAST_SECTOR
    anchor     39.9042 116.4074
    radius     60 km
    resolution 200 m
end_zone

platform RED_1 RED_RECON
    position latlng 39.9042 116.3100
    heading  90 deg
end_platform
```

See [`scenarios/`](scenarios/) for more (patrol encounter, sensor→track→jamming kill chain, five-mover demo, …).

### Layout

```
milsim/
├── src/milsim/          # framework source
│   ├── engine/          #   discrete-event engine (time / events / queue / inboxes)
│   ├── models/          #   mover / radar / comm / jamming / guidance / weapon / damage / task models
│   ├── services/        #   scenario parsing / param library / store / comm / EW / maps / orchestrator
│   ├── decision/        #   rule-based and LLM decision-making
│   └── simulation.py    #   assembly and simulation entry point
├── tools/               # CLI tools (run / export replay / stream server / param library / lint)
├── app/                 # web situation page (single-file index.html) + sample replays
├── scenarios/           # example scenarios (text format)
├── library/             # parameter library (text + .db)
├── tests/               # pytest suite
└── docs/                # design documents (framework design, radar spec cross-check)
```

### Satellite Basemap (optional)

The situation page reads satellite tiles from a local mbtiles file (NASA Blue Marble NG, public domain):

```bash
# Point to any plate-carrée .mbtiles (LOD 0 = 2 columns × 1 row)
set MILSIM_TILE_MBTILES=F:/afsim2.9cn/resources/maps/bluemarble_db/bmng.mbtiles
python tools/serve_replay.py ...
```

Without it the page still works (overlays / graticule / contact lines render fine) — only the imagery background is missing.

### Data Sources & Acknowledgements

- Satellite basemap: NASA Blue Marble Next Generation (public domain)
- Some design viewpoints were informed by studying the open AFSIM 2.9 architecture; all code here is an independent implementation

## License

Released under the [MIT](LICENSE) — free to use, modify, and distribute (keep the copyright notice).

> You are welcome to download and use this project, and to build upon it to help grow an open-source simulation ecosystem.
