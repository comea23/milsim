# 探测 → 通信传航迹 → 引导干扰：AFSIM 侧基线

本目录只有一份想定 `comm_track_jam.txt`。它的用途是给 milsim 侧
`scenarios/comm_track_jam.txt` 当**对照基线**，两边跑同一个场景、比**中间输出**。

跑法（`cwd` 必须是本目录，AFSIM 的 `include` 相对它解析）：

```
set AFSIM_MISSION=F:\afsim2.9cn\bin\mission.exe
cd F:\afsim2.9cn\milsim\demos\afsim_comm_track_jam
%AFSIM_MISSION% -sm comm_track_jam.txt
```

产物在 `output/comm_track_jam.evt`（事件流）与 `output/comm_track_jam.aer`
（轨迹）。文件名由想定里的 `define_path_variable CASE` 定，**别写死**。

---

## 为什么这份基线是"拼的"

AFSIM 自带的 demo 里，**没有任何一份同时具备这两个环节**：

| 环节 | 出处 demo | 那里的做法 |
|---|---|---|
| ① 传航迹 | `demos/electronic_warfare/comm_jamming.txt` | `WSF_GEOMETRIC_SENSOR` → `WSF_TRACK_PROCESSOR report_to subordinates via radio` → 下属 `WSF_RADIO_TRANSCEIVER` 收 `WSF_TRACK_MESSAGE` |
| ② 引导干扰 | `demos/electronic_warfare/agile_jamming.txt` | `WSF_TASK_PROCESSOR` 的 `AddSpot()`：`TRACK.SignalFrequency()` + `PLATFORM.WithinFieldOfView(TRACK, jammer)` + `StartJamming(TRACK, ...)` |

所以这份想定把两个环节**各抄一半**拼在一起（页首 43 行注释记了每段抄谁）。
拼接处必然踩坑，全部记在下面「踩过的坑」一节，供后来者少走弯路。

---

## 三处语义差异（**不是 bug**，用户裁定"只比值、不比机制"）

### 1. 干扰机怎么"置向"

| | AFSIM | milsim |
|---|---|---|
| 接口 | `StartJamming(TRACK, ...)` —— 按**目标对象**打 | `aim_at nearest` —— 读**本机航迹表**算方位角 |
| 天线指向 | 内部 `WithinFieldOfView(TRACK, jammer)` 判 | 独立的一道筛（`beam_width_deg > 0` 才启用） |

语义等价（都是"看见才打"），机制不同。**比值时只看"有没有打中"**，不看怎么打中的。

### 2. "链路断开"有没有对应机制

AFSIM 的 `jamming_perception_threshold` 讲的是**"感知到干扰"**，不是"链路断开"。
它没有等价于 milsim `mark_offline` 的机制。

⇒ **反向用例（中途切断通信链）只在 milsim 侧做**（用户裁定）。milsim 侧的
反向用例见 `tests/test_comm.py::test_cutting_the_comms_link_stops_everything_downstream`。

AFSIM 里链路真断的可见后果是 `MESSAGE_DISCARDED`（实测 `comm_jamming.txt` 里
600 条收到 vs 3720 条丢弃）。本 demo 跑到 `end_time 480 sec` 时
`MESSAGE_DISCARDED = 0`，因为拓扑从没断过。

### 3. 干扰的"后果模型"深度不同

AFSIM 走完整 EA/EP 框架（`EW_Effect` 那一套）。milsim 本版只做
**"通 / 不通"** 与 **"压 / 不压"** 二值后果，没有 BER / drop / distort。
`mInterferenceFactor < 0.5` 那一半后果未实现（§9-62）。

⇒ milsim 的判定**比 AFSIM 宽**（更容易判"通"）。这条差异显式记在
`docs/00-框架设计.md` §9-62。

---

## 四个中间量（"看中间输出是否一致"的落点）

两边**只要中间量在同一个时刻以同一种含义出现**，就算对上了。下面是四个
中间量在两边的**可观察量**：

| 段 | milsim 侧观察量 | AFSIM 侧观察量 |
|---|---|---|
| ① 探测 | `store.contacts_of(SCOUT_1)` 条数 | `SENSOR_DETECTION_ATTEMPT` 的 `Detected: 1` / `Detected: 0` |
| ② 传航迹 | `comm.delivered` / `comm.routed` | `MESSAGE_QUEUED` / `MESSAGE_TRANSMITTED` / `MESSAGE_DELIVERY_ATTEMPT` / `MESSAGE_RECEIVED` |
| ③ 置向 | `jam.aimed` / `jam.aim_failed` | `TASK_ASSIGNED` / `JAMMING_REQUEST_INITIATED` / `JAMMING_REQUEST_COMPLETED` |
| ④ 压制 | 受害雷达 `jamming_w` / `last_pd` | `JAMMING_ATTEMPT` + `S/(N+C+J)` 对比 `S/N` |

对拍工具：

```
cd F:\afsim2.9cn\milsim
python tools/demo_comm_track_jam.py            # 两边一起跑并排输出
python tools/demo_comm_track_jam.py --ours-only
```

### 实测数字（`seed 20260929`，480 s）

```
【① 探测】
    milsim  SCOUT_1 / JAMMER_1 各 3 条航迹（RED_RADAR_1 / RED_TGT_1 各 0 条）
    AFSIM   SENSOR_DETECTION_ATTEMPT 1922 次 → Detected: 1 966 次 / Detected: 0 956 次
            SENSOR_TRACK_INITIATED 13 条 / SENSOR_TRACK_UPDATED 901 次
【② 传航迹】
    milsim  delivered=120  routed=120  screened=964  epoch=0
    AFSIM   MESSAGE_QUEUED / TRANSMITTED / DELIVERY_ATTEMPT / RECEIVED 各 69
            MESSAGE_DISCARDED=0
【③ 置向】
    milsim  aimed=1165  aim_failed=35
    AFSIM   TASK_ASSIGNED=2  JAMMING_REQUEST_INITIATED=2  （脚本侧 started jamming 2 次）
【④ 压制】
    milsim  RED_RADAR_1  jamming_w 2.558e-05 W（恒定）  Pd 0.0000
            SCOUT_1 全程未被压（jamming_w 恒 0）
    AFSIM   JAMMING_ATTEMPT=1888 次
            S/N 区间 [25.3, 68.09] dB    S/(N+C+J) 区间 [-27.44, 54.72] dB
            最深一次压制：S/(N+C+J) 比 S/N 低 52.74 dB
```

★ **v0.13.40 复测（检测概率默认改非起伏后，§9-73）**：上表基本不变 —— 只有
② 的 `delivered / routed` 由 **119 → 120**（+1 次投递，是 Pd 曲线换了之后
某几拍的探测结果变了一遍所致）；③ `aimed=1165` / `aim_failed=35`、
④ `jamming_w 2.558e-05 W` / `Pd 0.0000`、以及**时序四段顺序**全部逐字不变。
⇒ 换代口径**没有**动摇这条链的任何结论（被压到底的 `Pd` 本来就在 0 附近，
Swerling 1 → 非起伏那 1.5 dB 移动不了它）。

### 时序因果（第一次出现的时刻，s）

```
段           milsim      AFSIM
① 探测       t = 2.000   t = 0.000
② 传航迹      t = 12.000  t = 8.443
③ 置向       t = 16.000  t = 8.463
④ 压制       t = 16.000  t = 8.500
```

**两边顺序一致**（探测 → 传航迹 → 置向 → 压制）—— 这就是"这条链真的串起来了"
的证据。绝对时刻不同是因为两边的转发/搬运节拍不同（milsim `share_interval 10 s`
+ `RF_COMM period 2 s`；AFSIM 是 `frame_time 1 sec` 的雷达节拍驱动）。

---

## ★ 哪里一致、哪里不一致（**必读，先看这一节**）

第一版报告把"定性一致"说成"结果一致"，是**过度解读**。真实情况分四类：

### 一致（可当证据用）

| 项 | 两边 | 说明 |
|---|---|---|
| **四段链路的因果顺序** | 都是 探测 → 传航迹 → 置向 → 压制 | 四段各自的"第一次发生"时刻单调递增，两边同序 |
| **"干扰机不自带探测"这条前提** | milsim `JAMMER_1` 无 sensor 件；AFSIM TRACK 只来自 `WSF_TRACK_MESSAGE` | 它表里的敌航迹**只能**从通信来 —— 链的通断才有意义 |
| **② 传航迹这一段的量级** | `delivered 120` vs `MESSAGE_RECEIVED 69` | 同一量级（1.7×），且两端都是"每拍一批、全程不断"（v0.13.40 换代检测口径后由 119 → 120，量级不变） |
| **③ 置向一旦发生就持续** | milsim 摘链路前每拍照涨；AFSIM `TASK_CANCELED = 0` | 都不抖（AFSIM 的 `HadToStopJamming` 判视场而非频点，正是为消抖） |
| **④ 压制方向** | 两边都是"受害雷达探测被压掉" | AFSIM `S/(N+C+J)` 显著低于 `S/N`；milsim `Pd` 掉到 `1e-6` |

### 不一致（**口径不同，不是同一件事**）

| 项 | milsim | AFSIM | 差在哪 |
|---|---|---|---|
| **① 探测次数** | `SCOUT_1` 480 次 / 480 s | `SENSOR_DETECTION_ATTEMPT` 1922 次 | **扫描律不同**：AFSIM 每秒转一整圈（480 圈）、milsim 1 圈 = 20 拍 × 6 目标（4 圈）⇒ 次数比是节拍比，**不是覆盖率或质量的比**（§9-70） |
| **④ 压制的"判决通过率"** | `42.2% → 0%` | `58.3% → 42.2%` | **两边建航门限差 8.95 dB**：AFSIM 给 `N = 16` ⇒ `Pd=0.5` 门限 **3.821 dB**（Swerling 1，AFSIM 侧口径）；milsim 没给 ⇒ `N = 1` ⇒ 门限 **12.772 dB**（Swerling 1 同等口径）。★ 不能用 `−10·log10(Pfa)` 当门限（那是功率门限、与 SNR 门限差 3 dB 且 N 依赖不同）。★ **milsim 侧默认口径 v0.13.40 起已是非起伏**（§9-73）⇒ 按默认跑时 milsim 那侧 N=1 门限是 **11.2426 dB**（两口径差同为 8.95 dB，结论不变）。`Detected` **不可比** |
| **④ 压制的绝对深度** | `jamming_w = 2.558e-05 W` | `S/(N+C+J)` 最深比 `S/N` 低 52.74 dB | 机制不同（AFSIM `StartJamming(TRACK,...)` + EA/EP 框架；milsim `aim_at` + 二值后果），**只比值不比机制**（用户裁定） |
| **③ 置向次数** | `aimed = 1165` | `TASK_ASSIGNED = 2` | milsim `aimed` 是**每拍**计数；AFSIM 是**事件**计数（只在起停时产生）。同名不同义 |

### 结论

> **"这条链在两边都能串起来、且因果顺序相同"是成立的**；
> **"两边中间输出的数值一致"是不成立的** —— 两边比的是不同粒度、不同门限、
> 不同节拍的量。真正逐位可对的是 **`S/I`**（§5.14.13 已有 `Δ(S/I) ≤ 8.6e-04 dB`、
> 37/37 的既成结论），**不是 `Detected`**（§9-67）。

### 效率

**本 demo 的用途是"数值对照"，不是"效率对比"。**

| 指标 | 读数 | 说明 |
|---|---|---|
| milsim 跑 480 s 推演 | **< 1 s**（墙钟） | 纯 Python、事件驱动、无外部依赖 |
| AFSIM 跑 480 s | **≈ 8~12 s**（墙钟，含进程启动） | `mission.exe` 启动本身占大头 |
| 加速比 | **≈ 10×** | ★ **但这个数不可外推**：两边**建模深度不同**（AFSIM 是完整 EA/EP + 三维运动学 + 地形；milsim 本版是简化模型）。**比的是"简化模型 vs 全模型"，不是"同一模型的两种实现"** |

⇒ 要谈"效率提升多少"，必须先固定"建模深度"再比；**当前这个数只能说明"简化模型快"，不能说明"算法更优"**。

---

## 踩过的坑（AFSIM 静默失败清单）

这些都是**不报错但结果为零**的坑，排查代价很高，逐条记下来。

### 1. `report_to subordinates via radio` 需要下属有真航迹

侦察机的雷达探不到目标 ⇒ 没有航迹 ⇒ 一条报文都不发。而"探不到"的原因
可能只是**俯仰视场**。

`esm_detect_jammer.txt` 的 `EW_RADAR` 写 `elevation_field_of_view -2.5 deg 42.5 deg`
—— 那边雷达在地面、目标是飞机（仰角）。本 demo 侦察机在 30000 ft **俯视**
地面目标，俯角约 −9°，下限 −2.5° 挡得死死的。

**症状**：480 次探测全部（当时连 `Detected` 字段都没有）。
**修法**：改成 `-60 deg 60 deg`。修后 `MESSAGE_*` 从 0 变 96。

### 2. `WSF_TRACK_MESSAGE` 不带"信号"字段

`agile_jamming.txt` 的 `AddSpot()` 用 `TRACK.SignalFrequency(signalIndex)`
拿被干扰的频点 —— 那是因为它的 TRACK 来自 `WSF_ESM_SENSOR`（有
`reports_frequency`）。**通信转发来的 `WSF_TRACK_MESSAGE` 不带信号字段**，
实测 `TRACK.SignalCount() == 0`，照抄的 `AddSpot()` 一次都进不去。

**症状**：`started jamming: 0`（收到 57 条航迹却一次没打）。
**修法**：改成"位置来自航迹、频点由干扰机自己定"：

```c
double freq = 900000000.0;   // 红雷达频点
if (StartJamming(TRACK, "Jam_surv", jammer, freq, 2.e6, "noise_jamming"))
```

这才是"用航迹引导置向"的正确写法 —— 航迹给的是**目标在哪**，
不是**目标辐射什么频率**。

### 3. `HadToStopJamming()` 不能比频点

同上：航迹不带频点 ⇒ 比不到 ⇒ 每拍都判"该停" ⇒ 起停抖动。

**症状**：`TASK_ASSIGNED` / `TASK_CANCELED` 各 11908，`JAMMING_ATTEMPT` 消失。
**修法**：只判 `WithinFieldOfView`。

### 4. 干扰机频点必须落在受害机带内

`WSF_RF_JAMMER` 的频点不在受害机带内 ⇒ **静默零辐射**，`JAMMING_ATTEMPT`
一条都不产生。

**症状**：`JAMMING_ATTEMPT = 0`。
**修法**：把频点设成受害雷达的 900 MHz（原来写 2.4 GHz）。修后 1888 次。

### 5. `SENSOR_DETECTION_ATTEMPT` 默认**不带结算字段**

这是最隐蔽的一条。sensor 块只写 `reports_signal_to_noise` **不够** ——
必须同时给出**检测判决三件套**：

```c
   swerling_case                 1
   probability_of_false_alarm    1.0e-6
   required_pd                   0.5
   number_of_pulses_integrated   16
```

★ **`swerling_case 1` 是 AFSIM 侧的口径，本 demo 保持不动**。milsim 侧自
v0.13.40 起检测概率**默认换成非起伏**（固定 RCS，`swerling_case 0`，见
`docs/00-框架设计.md` §9-73）——理由是框架喂进雷达方程的 RCS 是**常数标量**，
而 Swerling 1 的公式是对"RCS 逐扫描指数起伏"做了平均的结果，两边语义对不上。
⇒ 本 demo 的两侧 **`Pd` 数值不再可比**（用哪条曲线都不同：`N=16`、`Pd=0.5`
门槛非起伏 2.2850 dB vs Swerling 1 3.8205 dB，差 1.54 dB）；**可比的是 `S/I`**
——它是唯一不经过检测判决曲线的量，§9-67 已钉死"只有 `S/I` 是尺子"，`Pd` /
`Detected` 在两侧都只当**过程计数**看。若要让 milsim 侧对齐本 demo 的 `Pd`
口径，在想定里显式写 `swerling_case 1` 即可。

缺了它们，AFSIM **不做概率判决**，事件块输出到 `RcvrBeam` 那行就结束，
`S/I` / `Threshold` / `S/N` / `Pd` / `Detected` 这几行**根本不出现**。

**症状**：`.evt` 里连 `Detected` 这个词都搜不到（不是"0 次"，是"没有这个字段"）；
对拍工具会报 `0 次`，很容易误读成"没探到"。
**修法**：补齐三件套（取值抄 `barrage_jamming.txt`）。修后：

```
Xmtd_Power: 92.3318 dBw Rcvd_Power: -77.1826 dBw Rcvr_Noise: -131.656 dBw
S/I: 54.473 dB Threshold: 12.8251 dB S/N: 54.473 dB S/(N+C) ... S/(N+C+J): 54.473 dB
Pd: 0.999953 RequiredPd: 0.976576 Detected: 1
```

★ **`Detected` 的真实字段名是 `Detected: 1` / `Detected: 0`，不是
`Detected: true`。** 写 `true` 会恒得 0（`parse_evt` 第一版就这么错）。

### 6. `weapon jammer on` ⇒ t=0 自发辐射

写了 `on` 会在 t=0 直接产生 `JAMMING_REQUEST_INITIATED`，
**与 `task_mgr` 无关**（摘掉 `task_mgr` 结果一模一样 ⇒ 对照实验定位）。
必须写 `off`，让辐射完全由 `AddSpot()` 驱动。

### 7. `on_message` 里没有 `end_type` / `forward`

正确写法：

```c
   on_message
      type WSF_TRACK_MESSAGE
         script
            ...
         end_script
      end_on_message
```

写成 `type X ... end_type` 会报 `Unknown command: end_type`。

### 8. `.evt` 是多行**续行**格式

一条事件的头行以时间戳开头，后续行以空格开头、除最后一行外都以 `\` 结尾。
**`S/N:` / `Detected:` 这些字段只出现在续行里。** 解析时必须先按 `\` 拼回
逻辑行，且正则要带 `re.S`（否则 `.*$` 匹配不到跨行内容）。

**症状**：`parse_evt` 把 5000 个事件块只认出 40 个（只有不带续行的那几类），
`Detected` 计数恒 0、`SENSOR_DETECTION_ATTEMPT` 计数为 0。
**修法**：先拼块，再对整块匹配，正则加 `re.S`。

---

## 两边的参数对照

| 概念 | AFSIM | milsim |
|---|---|---|
| 雷达发射功率 | `transmitter / power` | `component sensor ... peak_power` |
| 判决门限 | `required_pd` | `required_pd`（同） |
| 通信件门限 | — | `required_snr_db`（**注意与雷达侧不同名**） |
| 噪声功率 | `receiver / noise_power` | **不给**（走天线噪声温度链） |

★ milsim 侧**不能同时给** `noise_power` 与逐参法噪声链（同一个量两个出处，
装配期报错）。写了 `peak_power` ⇒ 自动走逐参法。

★ milsim 侧雷达参数名是 `peak_power` / `required_pd`，写成 `power` /
`required_snr_db` 会在装配期报未知参数。

---

## 已知缺陷（本版不改，已记文档）

- **§9-62**：`mInterferenceFactor < 0.5` 那一半后果未实现 ⇒ 判定比 AFSIM 宽。
- **§9-63**：槽名尾部的裸数字 = 天线条数，所以 `jammer0` 解析出 **0 条天线**。
  本版缓解 = 想定注释写"不要写 `jammer0`"。
- **§9-55**：想定里一个 `zone` 都没有 ⇒ 所有 `latlng` 静默变 `(0,0,0)`，
  实体全叠原点 ⇒ 干扰筛会跳过整台名册。**这份想定**必须带 `zone`。

---

## 相关文件

| 文件 | 作用 |
|---|---|
| `comm_track_jam.txt` | 本目录的 AFSIM 想定 |
| `../../scenarios/comm_track_jam.txt` | milsim 侧的对应想定 |
| `../../tools/demo_comm_track_jam.py` | 两边跑 + 并排对拍 |
| `../../tests/test_comm.py` | 端到端用例（正向 + 反向） |
| `../../docs/00-框架设计.md` | §5.15 通信件、§9-62/§9-63 |
| `../../../demos/electronic_warfare/comm_jamming.txt` | 环节① 出处 |
| `../../../demos/electronic_warfare/agile_jamming.txt` | 环节② 出处 |
