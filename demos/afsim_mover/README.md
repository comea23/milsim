# AFSIM 机动 demo：复现与对照

本项目里"AFSIM 的机动件长什么样"这件事的**唯一出处**。四部分：复现想定、
参照数据、对照工装、弹那一份的例外说明。原脚本一行没改，改了什么全部打印在下面。

```
demos/afsim_mover/
  kinematic_mover.txt          ← milsim 复现想定（原版 kinematic_mover demo，4 个平台）
  aircraft_mover.txt           ← milsim 复现想定（mover_demo 的 7 个科目）
  afsim_aircraft_mover.txt     ← AFSIM 侧的合法参照脚本（原稿跑不起来，见 §5）
  route_finder.txt             ← milsim 复现想定（3d_route_finder：选路，无 component_type）
  missile_guidance.txt         ← milsim 复现想定（new_guidance3 的弹体 + 巡航制导，见 §11）
  ref_*.csv                    ← AFSIM 采出来的参照轨迹
  ref_3d_route_finder_route.csv        ← AFSIM 实际选中的那条航路（脚本里 shortestRoute 的航路点）
  repro_*.csv                  ← milsim 跑出来的复现轨迹 / 选路结果
docs/images/afsim_*_compare.png  ← 三格对照图
tools/afsim_ref.py             ← 跑 AFSIM、采参照
tools/demo_afsim_mover.py      ← 跑复现 + 对照 + 出图（跟航线那两份）
tools/demo_route_finder.py     ← 逐层选路 + 对照 + 出图（选路那一份，见 §8）
tools/demo_afsim_missile.py    ← 弹体 + 巡航制导 + 对照 + 出图（弹那一份，见 §11）
```

## 0. 怎么跑

三条链路，顺序不能反（参照先采，复现后跑）：

```bash
# 参照：跑 AFSIM 原版 demo，采成 CSV（约 1 分钟）
python tools/afsim_ref.py ../demos/kinematic_mover/kinematic_mover_demo.txt \
    demos/afsim_mover/ref_kinematic_mover.csv --seconds 600

# 复现 + 对照 + 出图
python tools/demo_afsim_mover.py \
    --afsim ../demos/kinematic_mover/kinematic_mover_demo.txt \
    --ref   demos/afsim_mover/ref_kinematic_mover.csv \
    --repro demos/afsim_mover/kinematic_mover.txt \
    --out   demos/afsim_mover/repro_kinematic_mover.csv \
    --plot  docs/images/afsim_kinematic_compare.png \
    --seconds 600 --alt-programme kinematic

# 选路那一份（§8）：一条命令搞定，它自己跑三次 AFSIM
python tools/demo_route_finder.py \
    --afsim ../demos/route_finder_demos/3d_route_finder.txt \
    --repro demos/afsim_mover/route_finder.txt \
    --ref   demos/afsim_mover/ref_3d_route_finder.csv \
    --route demos/afsim_mover/ref_3d_route_finder_route.csv \
    --out   demos/afsim_mover/repro_3d_route_finder.csv \
    --plot  docs/images/afsim_route_finder_compare.png \
    --seconds 1500

# 弹那一份（§11）：同样是"一条命令跑完"，参照由它自己采
python tools/demo_afsim_missile.py \
    --afsim ../demos/new_guidance/new_guidance3.txt \
    --repro demos/afsim_mover/missile_guidance.txt \
    --ref   demos/afsim_mover/ref_new_guidance3.csv \
    --out   demos/afsim_mover/repro_new_guidance3.csv \
    --plot  docs/images/afsim_missile_compare.png \
    --seconds 700
```

`afsim_ref.py` 需要 AFSIM 的 `mission.exe`：设环境变量 `AFSIM_MISSION`
指向它，或把它放在 AFSIM 根的 `bin/` 下（本项目就寄居在 AFSIM 根里）。
后两条其实**自带采参照**：`--afsim` 与 `--ref` 都给就先采，只给 `--ref` 就直接
读现成的 CSV——**参照数据是怎么产生的和参照数据一样重要**，所以采它的那条
命令要能重跑。

> 三条链路里，**弹那一份与前两条不是一类东西**：前两条比的是"谁跟得住航线"，
> 它比的是"给定同一枚弹体 + 同一张阶段表，两边飞出来什么形状"。它**不是原 demo 的
> 等价复现**——先读 §11.4 的缺口清单（**横向那条仍缺**）与 §11.6 的末段根因
> （**末段打不中，原因已定位在弹体**）再看数字。

`aircraft` 那一份把两条命令里的 `kinematic_mover_demo.txt` 换成
`demos/afsim_mover/afsim_aircraft_mover.txt`、`--alt-programme` 去掉即可。

---

## 1. 主线：`kinematic_mover_demo.txt`（AFSIM 原生 demo）

### 1.1 原 demo 自己在说什么

`demos/kinematic_mover/doc/kinematic_mover.rst` 只有三句，但三句都要紧：

> This demo has three platforms using the same route (just offset from each
> other) but have different limits using the kinematic mover. **Flyer4 is the
> same as Flyer3 but uses the WSF_AIR_MOVER** to illustrate the difference
> between the AIR MOVER vs the KINEMATIC MOVER.
>
> Keep in mind that the **KINEMATIC MOVER calculates acceleration in the
> vertical plane whereas the AIR MOVER does not** (only in the horizontal plane).

所以这条 demo 讲的是两件事，**不是一条**：

- **甲**：同一条航线、同一个件，三组限制（过载 4.0 / 2.2 / 2.2 g，转向
  65 / 50 / 120 °/s）分别把轨迹拉成什么样；
- **乙**：**垂向那一半**——AFSIM 把"平滑垂向过渡"与"垂向瞬变"分给了两个
  件。我们没有两个空中件，只有一个 `AIR_MOVER`，这件事就落在**参数**上：
  `vertical_accel`（有 = 运动学式，垂向按加速度建立）与 `climb_rate` /
  `descent_rate`。这是复现里**唯一一处"用参数代两个件"**的地方。

参照能采到 24064 个样本 / 4 个平台 / 600 s，**AFSIM 一条命令都没报错**
（`afsim_ref.py` 的"逐行摘掉认不出的命令"这一步零命中）。

### 1.2 结果

```
平台     水平航迹 km  三维航迹 km  平均 m/s  峰值 ω  停摆 %  零前进 %  到航线（米）      巡航 R m   同弧长偏差
         参照/复现    参照/复现    参照/复现 °/s     三维  水平      参照均/最大        参照/复现  平均/最大
          ─────────────────────────────────────────── 复现均/最大 ──────────────────────────────
flyer1   40.29/55.78  61.33/71.82  102/120  97.0/20.8  0.0/0.2  0.0/12.3 179/983   64/891   55/211   1894/4853
flyer2   52.40/52.64  63.59/63.77  106/106  72.8/11.5  0.0/1.2  0.0/17.5 493/2506  92/903   85/370   2848/6130
flyer3   52.45/52.80  63.59/63.88  106/106  72.8/11.5  0.0/1.1  0.0/17.3 485/2482  93/903   85/370   2873/6190
flyer4   60.38/52.64  64.95/65.77  108/110  16.4/11.5  0.0/2.8  0.0/17.5 284/1081  92/903  371/384   2258/5704
```

**读法**（每一列各回答一个问题，别混着看）：

- **三维航迹**：600 s 里两边各飞了多远（带上爬升）。**这是"像不像"最硬的一列**：
  flyer2 / flyer3 / flyer4 两边差 **0.3% / 0.5% / 1.3%**（63.6 / 63.6 / 65.0 km vs
  63.8 / 63.9 / 65.8 km）。只有 flyer1 差得多（61.3 vs 71.8 km，+17%）——它是唯一
  挂 `VARY_DIR_ALT` 高度脚本的那一架，见"已知差异 4"。
- **水平航迹**：同一趟路只算水平投影。flyer1 的参照只有 40.3 km 而三维是 61.3 km，
  **差的那 21 km 全在爬升里**——AFSIM 的件爬升时把水平分量让出去（它的水平速率
  掉到 13 m/s），所以**不能拿这一列当"飞了多远"**。
- **平均 m/s** 也按三维算，好与 AFSIM 的 `Speed()` 同口径（它是三维速率矢量的
  大小）。
- **停摆 %（三维）**：三维速率 < 5 m/s 的帧占比——**真的没在动**。参照 0.0%，
  复现 0.2 / 1.2 / 1.1 / 2.8%：**两边都几乎没有停住的帧**。
- **零前进 %（水平）**：**水平**速率 < 5 m/s 的帧占比——水平没挪窝，但可能正在
  升降。参照 **0.0%**（它的水平速率最低也有 13 m/s），复现 **12~18%**。
  **这才是"先掉头、不前进"在空中航线上的样子**，见"已知差异 1"。
  （这两个数的速率都是**从位置差分算的**，不是 CSV 里那一列——两边的
  `speed_mps` 列不是一回事，见 §7.3。）
- **到航线**：每条航迹到**航线折线**的最近距离（点到线段，逐点算）。两边与
  **同一条**航线比，所以**不受相位影响**——"谁跟得住航线"看这一列。复现平均
  64~93 m，参照 179~493 m；最大偏差复现 891~903 m，参照 983~2506 m。
  **复现比 AFSIM 更贴航线**，代价就是上面那 12~18% 的水平零前进帧。
- **同弧长偏差**：把两条航迹按**累计弧长**重采样到 400 个点之后的距离。它回答
  "这条线走得像不像"，**刻意不逐时刻比**——两边速率控制不同（原 demo 每个航路点
  带自己的速度，我们只有一个 `max_speed`），逐时刻比出来的先是"到哪了"。代价是
  它会把**每圈被切掉的那几百米逐圈累加**成相位差，所以公里级读数要与"到航线"
  一起看，别当成"整条线都走错了"。
- **巡航 R** = 速率 ÷ 角速度，只取 `ω ≥ 1 °/s` 且速率在最大速率一半以上的帧。
  速率掉一半以下算出的小半径是"它慢下来了"，不是"它转得动"。

**峰值 ω 那一列不能当判据。** 参照的 flyer1 量出 97.0 °/s，而它自己声明的
`maximum_body_turn_rate` 只有 65 °/s——2 倍。两种解释都还在：AFSIM 的
`maximum_body_turn_rate` 约束的是**三维速度矢量**的转动（垂向也在转），
而我只量了水平航向；或者它确实越了限。所以这一列**只作观察**。

### 1.3 参数映射

`kinematic_mover.txt` 里逐条对应，来源写在想定注释里：

| AFSIM（原 demo 的 mover 块） | milsim（复现想定的 `component_type`） | 口径 |
|---|---|---|
| `maximum_body_turn_rate` | `turn_rate` | 同为角速度上限，`deg/s` 直抄 |
| `maximum_radial_acceleration` | `radial_accel` | 同为横向加速度上限；`ω = a/v`，是同一个量 |
| `maximum_linear_acceleration` | `linear_accel` | 同为前向加速度上限 |
| `maximum_body_roll_rate` | `angular_accel` | **只作量级代理**：AFSIM 的转向建立由滚转带出来，我们的第二层是直接的角加速度上限，两者不是同一个量，所以按同数量级取，不假装等价 |
| 运动学件"垂向算加速度" / 气动件"不算" | `vertical_accel`（有 / 给到不成为瓶颈） | 见 §1.1 的"乙" |
| `initial_speed` / `target_speed` | `max_speed` + 驱动脚本写速率状态 | 想定里写不了初速（§9-22） |
| `velocity_pursuit_gain` / `proportional_navigation_gain` | **无对应** | 我们的转向是"朝目标对准"，不是 PN/VP 导引律 |
| `maximum_flight_path_angle` | **无对应** | 见 §3 |
| route 里每个航路点的 `speed` | **无对应** | 只有一个 `max_speed`，见 §3 |
| `go_to <label>` | 驱动脚本展开成更长的航线 | 见 §2 |
| `GoToAltitude` / `ReturnToRoute` | 驱动脚本改写剩余航线的高度 | 见 §2 |
| `update_interval` | `period` | 两边都用 0.1 s |
| `position` / `heading` / `altitude`(MSL) | 一样 | 经纬度逐字照抄 |

原 demo 的 `maximum_radial_acceleration` 与 §5.7.11 的 `ω_max(v) =
min(turn_rate, radial_accel / v)` 是**同一件事**：两项取小，正是 AFSIM
文档里那句 "This constraint is imposed simultaneously with
`maximum_body_turn_rate`, the most restrictive is used"。

---

## 2. 驱动脚本做的那几件事（以及为什么必须做）

复现侧的想定只写**参数与起点**。航路是运行期下达的东西（§5.7），而
AFSIM 的 route mover 一口气拿整条 `route`。这中间的桥就是
`tools/demo_afsim_mover.py`，它做四件事：

1. **把 `go_to` 的回环展开成一条更长的航线**，而不是每转完一圈重新下达。
   展开的理由是 `_speed_goal` 里"停得下来"那一条只在**最后一段**算（在
   `leg == len(waypoints) - 1` 时）：每圈都走一遍终点就每圈刹一次车。
   展开几圈按这趟要跑多久估，故意给足余量。
2. **整条航线一次交下去**（`Mover.move_along_route`，本轮新加的入口）。
   **不能每个拐角抛一个 `move_to_point`**：那会走 `_restart_trip` →
   `_halt`，三个通道的速率状态每过一个航路点清零一次，于是每个拐角都
   "停一下再重新起步"。AFSIM 的 route mover 与 `move_to_cell` 都是
   "整条一次下达"的口径，缺的只是"按坐标点一次给一串"这条入口。
3. **初速、初始高度、初始航向由脚本写进去**。想定里没有这三条：
   落格的 z 恒为 0（§9-22）；机动件的速率状态一律从 0 起；而原 demo 是
   从 32808.40 ft（10000 m）起步的——不写进去，前 250 s 就是一段"从海平面
   爬到 10000 m"，两边一比全是它。
   - 顺序不能反：`move_along_route` 会走 `_restart_trip` → `_halt`，
     **先写初速再下命令 = 初速被当场冲掉**。
   - **初始航向不是想定里那句 `heading`**，是"起点指向航路上第一个不与起点
     重合的点"的方位角。这条是对着参照轨迹量出来的：7 个平台在 t=0 的航向
     与各自航线逐一对，**全部**等于那个方位角（kinematic 那份的
     `heading 215 deg` 只是恰好差 0.3°；aircraft 那份的 `MAX_G` 声明 180°、
     实际 1.8°）。两个件都是这样：**一旦开始走航线，机头就已经在航线的
     方向上了。**照抄想定里那句会让复现平白多出一段几十度的掉头，而这段
     掉头在参照侧根本不存在。
4. **`VARY_DIR_ALT` 处理器的等价改写**。原脚本把它挂在 **flyer1** 上（
   flyer2/3/4 没有，这一点照抄），在 t≈25 / 100 / 200 s 上分别下
   "在当前高度上加 20000 m / 降到 4000 m / `ReturnToRoute()`"。我们没有
   `GoToAltitude`，用**改写剩余航线的高度目标**来等效（同一条水平航路、
   换一个高度目标）。这是**等价改写不是新增能力**。

---

## 3. 已知差异（照图的时候先把这三条读掉）

### 3.1 拐角**原地掉头**：零前进 0% vs 12~18%（最大的一条）

参照 4 个平台的**水平**速率**一帧都没有低到 5 m/s 以下**（最低 13 m/s）；
复现每圈在每个拐角处**停下来转向**，600 s 里 12%（flyer1）到 18%（flyer2~4）的帧
水平速率为 0。

根因是**量出来的**，不是猜的。把那些段一段段摘出来看：

```
flyer1   8.00 s  水平速率 0.00  转 162.5°  ⇒ 20.32 °/s
flyer2  11.90 s  水平速率 0.00  转 135.2°  ⇒ 11.36 °/s
flyer3  11.80 s  水平速率 0.00  转 134.1°  ⇒ 11.36 °/s
flyer4  11.80 s  水平速率 0.00  转 134.1°  ⇒ 11.36 °/s
```

段里**水平**速率恒为 0.00，航向以**恒定角速度**转够一个超过 30° 的角才走。
那个角速度正是 `radial_accel / max_speed`：

| | `radial_accel` | `/ max_speed` | 量出来 |
|---|---|---|---|
| flyer1 | 39.2 m/s² | 20.796 °/s | 20.32~20.50 °/s |
| flyer2~4 | 21.6 m/s² | 11.459 °/s | 11.35~11.36 °/s |

（略低是因为 `angular_accel` 要花一点时间把角速度建立起来。）

⇒ **我们的模型"先掉头、不前进"（基类的 `STEER_BEFORE_MOVE_DEG = 30°`），
AFSIM 的 route mover 是边走边转的连续弧。**同一条角速度上界，我们把它花在
"停着转"上，AFSIM 花在"转着走"上。于是复现比参照**更贴航线**
（到航线平均 64~93 m vs 179~493 m），代价就是那 12~18% 的水平零前进帧。

#### 这不是"停了"，而且以前那个"14~19% 停摆"是**口径不对等**量出来的

两条都要说清，否则数字会被读反：

1. **它没停。** 那些帧里平台的**垂向**在动——实测那些段的垂向速率**均 60 m/s、
   最大 100 m/s**，99% 以上的帧垂向速率大于 1 m/s。按**三维**速率数，
   < 5 m/s 的帧只有 **0.2 / 1.2 / 1.1 / 2.8%**（参照 0.0%），也就是
   **两边都几乎没有真正停住的帧**。所以这一节说的是"水平不前进"，
   不是"停下来"。基类的顺序是**转向 → 前进 → 定高**，"掉头"那一帧只是
   跳过了"前进"，"定高"照走。
2. **两边的速率列不是一回事，所以"停摆 %"以前是假的。** AFSIM 的
   `Speed()` 是**三维**速度矢量的大小，而我们写进 CSV 的是机动件的**水平**
   速率状态。上一版直接拿这两列比，等于用"三维速率 < 5 m/s"去量参照、
   用"水平速率 < 5 m/s"去量复现——数出来的 0% vs 14~19% 里，**那 14~19% 有
   九成以上是"在升降、只是没前进"**。现在两边的速率一律**由位置差分算**，
   并分成"停摆（三维）"与"零前进（水平）"两列，见 §1.2 与 §7.3。

**这一条以前也不是这个数。**本轮把换腿改成"整条航线一次下达"之前，
水平零前进是 7~9%——**那个数更小，但同样是假的**：`move_to_point` 每次都把
`_speed` 清零，而 `_turn_rate_ceiling(0)` 里过载那一项不参与（`a/v` 在 v→0
发散），于是掉头的角速度上界退化成了机构上限 `turn_rate`（50~120 °/s），转得快。
那不是模型更快，是**速率状态被抹掉了**。把状态露出来之后，掉头按过载那一项
走，代价才第一次显形。

**没改基类那条规则**：它当初是按**格尺度**的论证定的（"346 m 的格中心距
× sin30° ≈ 173 m，横向偏离还不到半格"），而空中航线的腿是**公里级**，
两者不是同一个尺度。怎么处理是设计决策，不是本 demo 能定的——已记进
`docs/00-框架设计.md` §9。

### 3.2 航路点速度缺（GoToSpeed）

原 demo 每段航路各带一个速度（180 / 200 / 210 / 300 kts），我们的空中件只有
一个 `max_speed`。这里取 210 kts（原件的 `target_speed`）当全程速率。
**后果是可预期的**：参照的速率剖面在 180~300 kts 之间变，复现是一条直线。
想定注释里写了这一条，图上速率那一格也标了。

### 3.3 参照自己越限（只作观察）

flyer1 声明 `maximum_body_turn_rate 65 °/s`，参照轨迹量出 **97.0 °/s**。
见 §1.2 末尾——这一列只作观察，不当判据。

### 3.4 速率的口径：AFSIM 封顶**总速度**，我们封顶**水平速度**（flyer1 差 17% 的原因）

这一条是上面"三维航迹"那一列唯一一处对不上的地方的根因。

AFSIM 的运动学件把**速度当三维矢量的大小**来管（`Speed()`，只认 route 里
每个航路点的 `speed`）。要爬升，它就把水平分量让出去——实测参照的水平速率
**最低掉到 13.4 m/s**，而它的三维速率**全程封在 154~156 m/s**（= 300 kts，
航路上最快那一段的 `speed`）。

我们的空中件是**两条独立通道**：水平走 `max_speed`，垂向走
`climb_rate` / `descent_rate`。于是：

```
三维速率峰值 = √(108² + 100²) = 147.2 m/s   ← 实测复现正好 147.2
声明的 max_speed 只有            108 m/s
```

**我们爬升不占用水平速率，总速率可以超过声明的 `max_speed`。**这等价于
**"航迹角上限"这一条缺席**（§4 能力缺口表里的 `maximum_flight_path_angle`
那一行）——AFSIM 用"总速度封顶"把航迹角间接管住了，我们没有。

后果就是 flyer1：它是四架里唯一挂 `VARY_DIR_ALT` 高度脚本的，全程有公里级
的升降，于是那段时间**我们"边爬边全速平飞"、它"把水平速度换成爬升"**——
600 s 下来三维航迹 71.8 vs 61.3 km，差 17%。flyer2/3/4 没有高度脚本（或
高度变化小），两边就回到 0.3~1.3% 以内。

想定注释里写了这一条，图上第三格也从速率剖面改成了**三维速率剖面**
（以前画的是 CSV 那一列，两边口径还不同，见 §7.3）。

### 3.5 高度剖面：flyer1 没有降到脚本要求的高度

`VARY_DIR_ALT` 在 t≈100 s 上要求降到 4000 m。参照降到 **4111 m**（到位了）；
复现只降到 **5360 m**，随后 t≈200 s 的 `ReturnToRoute()` 等价改写把高度目标
改回航线高度，下降就中断在那里。

根因是**等价改写本身的性质**：我们用"改写剩余航线的高度目标"代替
`GoToAltitude`，于是**下一次改写会覆盖上一次还没走完的下降**；AFSIM 的
`GoToAltitude` 是独立的命令通道，不会被 `ReturnToRoute` 冲掉。这是
§2.4 那条等价改写的已知边界，不是 bug。
（flyer2/3/4 没有这个脚本，高度剖面除了相位以外基本重合——flyer4 两边的最低
高度都是 **6914.4 m**，正好是航线最低那个航路点 22685.04 ft。）

---

## 4. 能力缺口清单（复现不了的，逐个列出来）

| 缺什么 | 原 demo 里的样子 | 本 demo 怎么办的 |
|---|---|---|
| 航路点速度 / `GoToSpeed` | `position ... speed 210 kts` | 取 `target_speed` 当常数（§3.2） |
| `GoToAltitude`（保持航线改高度） | 脚本处理器的 `PLATFORM.GoToAltitude()` | 驱动脚本改写剩余航线的高度（§2.4） |
| `maximum_flight_path_angle` | Mover 命令 | 无对应。我们的垂向是"速率 + 加速度"两层，没有"航迹角上限"这一条 |
| `velocity_pursuit_gain` / `proportional_navigation_gain` | 运动学件的导引增益 | 无对应（我们的转向是朝目标对准） |
| `at_end_of_path` 的 `extrapolate` / `remove` | Route 命令 | 无对应：航路走完就停 |
| 旋翼机 `WSF_ROTORCRAFT_MOVER` | `mover_demo` 里的 8 号科目 | **没有旋翼机件**，整个科目不复现 |
| 垂向加速度分层 | 运动学件在三维里算加速度、气动件只在水平面算 | 用 `vertical_accel` 参数代（§1.1 的"乙"） |
| `bank_angle_limit` / `body_g_limit` | Mover 命令 | 无独立参数，由 `radial_accel` 表达 |
| 整条航线一次下达 | `use_route` + 整个 `route` 块 | **本轮补上了** `Mover.move_along_route` |

---

## 5. 另一份：`demos/mover_demo/aircraft_mover_demo.txt`（**不是 AFSIM 的 demo**）

这一节是把话说清，免得后面对着一堆对不上的数字找原因。

**它不是 AFSIM 自带的 demo。**判据是三条硬的：

- 目录里**没有 `README.md`、没有 `doc/`**，而每一个 AFSIM 原生 demo 目录都有
  （`kinematic_mover/README.md` 是 2022 年的）；
- 文件时间戳是 **2026-04-19 19:28**，而同树里其它 demo 是 2025-11 或 2022；
- 页首写"本文件完全自包含，无需外部 include 文件"——原生 demo 不这么写。

**原样跑不起来。**`afsim_ref.py` 会把 AFSIM 认不出的命令逐行摘掉才能跑，
这份实测要摘掉四种：

```
simulation / duration / time_step / end_simulation   → Unknown command: simulation
platform_type 里的 name "..."                        → Unknown command: name
platform <名> + 块内 platform_type <型号> 两步写法    → Could not find platform_type
旋翼机的 mode HOVER / CRUISE / DASH                   → 'mode' cannot be used in this context
```

所以 AFSIM 侧的参照不是原稿，而是**按原稿的航线与参数重写的合法脚本**
`afsim_aircraft_mover.txt`（页首列了同样四条）。航线坐标、高度、每段速度、
航向、初始高度**逐字照抄，一个数字没改**——改的是写法。

**重写之后它自己也不跟航线。**7 个科目的对照数字：

```
平台                   到航线 参照(均/最大)  到航线 复现(均/最大)  停摆% 参照/复现  零前进% 参照/复现
                                                               （三维）        （水平）
AIR_MOVER_DIVE           309 / 3702 m           84 / 233 m         43.6 / 0.0     43.6 / 12.4
AIR_MOVER_MAX_CLIMB     6916 / 19191 m         121 / 367 m          0.0 / 0.0      0.0 / 20.0
AIR_MOVER_MAX_G         9580 / 30318 m         177 / 1385 m        41.8 / 21.1    41.8 / 21.1
AIR_MOVER_MAX_SPEED    24283 / 56147 m         276 / 1209 m         0.0 / 34.0     0.0 / 34.0
AIR_MOVER_MAX_TURN     10545 / 25400 m         604 / 6271 m         0.0 / 15.6     0.0 / 15.6
KINEMATIC_DIVE         26999 / 59471 m          21 / 74 m           0.0 / 0.0      0.0 / 4.7
KINEMATIC_SMOOTH       41190 / 98395 m         337 / 4292 m         0.0 / 0.0      0.0 / 5.2
```

单位都是**米**：AFSIM 自己跑这条航线时基本**不在航线上**（平均离开航线 0.3~41 km、
最远 98 km）。（`KINEMATIC_SMOOTH` 以 ~400 mph 恒定航向直线飞出 109 km、高度冲到
52757 m。）

注意这一边**参照自己也有"停摆"**（`AIR_MOVER_DIVE` 43.6%、`AIR_MOVER_MAX_G`
41.8%）——那是 `WSF_AIR_MOVER` 的水平机动件配上这条航线时自己出的行为，
不是复现复现错了。而 `KINEMATIC_*` 两个科目的复现反而**比参照贴航线得多**
（21 m / 337 m vs 27 km / 41 km）。

⇒ **这一份的定位是"参数罗列"，不是量化对照。**它的真正价值是把
`WSF_AIR_MOVER` / `WSF_KINEMATIC_MOVER` / `WSF_ROTORCRAFT_MOVER`
的**每一条命令**列了一遍——§1.3 那张映射表的另一半就是从这里来的。它的
对照数字照列在 `repro_aircraft_mover.csv` 与
`docs/images/afsim_aircraft_compare.png` 里，但**不要拿它当基线**：那一边
参照自己就没有跟航线，两边比的不是同一件事。

---

## 6. 一个既有的缺陷（与本 demo 无关，但这里会用上）

`services/map/hex.py` 里**两套 y 轴符号相反**：

- `LocalFrame.geo_to_world()` 把**正北放在 +y**（探针实测：锚点正北 0.1° 的
  点给出 `y = +11119.5`）；
- 而 `heading_to_offset()` / `offset_to_heading()`（**有测试钉死**：
  `test_heading_zero_points_north` 断言 `heading_to_offset(0°, 100) == (0, -100)`）
  规定 **−y = 北**。

端到端判据（平台摆在锚点正南、命令它去正北的点）：纬度确实升到 34.0
（**位移是对的**），但 `heading` 一路报 **180°**。

⇒ **位移对，航向数相对地理南北镜像。**影响面：

- 两边直接比**绝对航向**会差一个 `180° − h`；按 `|Δ航向|` 算的一切（角速度、
  转弯半径）**不受影响**；
- 项目里所有带 `invert_yaxis()` 的绘图工具是同一个符号问题的另一面。

本 demo 里绕开它的地方只有两处，都写了理由：工具的 `_geographic_heading()`
（把航向换算成地理口径，好让 CSV 与 AFSIM 的 `Heading()` 并排看），以及
绘图里**不加** `invert_yaxis()`（`geo_to_world` 的 +y 就是正北，而 matplotlib
默认 y 向上，正好"北在上"）。

**没改 `hex.py`**：那会改变 `hex` 坐标在想定里的含义，不属于本 demo 的范围。
已记进 `docs/00-框架设计.md` §9。

---

## 7. 对照时的三个坑（改这个工具之前先看）

1. **AFSIM 的 `writeln` 打浮点只有 `%g`（6 位有效数字）**。绝对经纬度落盘
   就是 **11 m** 分辨率，而机动件每帧只走约 10 m——**位移与量化噪声同量级**，
   按坐标差算弧长算出来的是噪声（实测把 600 s 的航迹算成 43.5 km，真值约
   60 km）。所以观察器打的是**相对锚点的偏移**（`_LAT_ANCHOR` / `_LNG_ANCHOR`，
   数值落在 0.1~1 之间，同一个 `%g` 下分辨率进到 **0.1 m**），落盘时再加回去，
   CSV 里仍是绝对经纬度。
2. **角速度不能拿逐帧差分的最大值**。参照那边 `GoToAltitude` 会让机动件
   额外来一次 `MOVER_UPDATED`，于是同一时刻有两个样本、`dt` 趋近 0，一点
   航向差就能除出上百 °/s（实测 flyer1 这样量出 127 °/s）。所以角速度在
   **等时网格（0.5 s）**上差分。
3. **★ 两边的 `speed_mps` 列不是一回事，不能直接比。** 参照的是 AFSIM 的
   `Speed()`——**三维**速度矢量的大小；复现写进去的是机动件的**水平**速率状态
   （垂向是另一条通道）。直接比会得出一个假的结论：按这两列数"速率 < 5 m/s"，
   得到 0% vs 14~19%，看起来"参照从不停、复现老在停"；**实际上那 14~19% 有
   九成以上是"在原地转向、同时以 100 m/s 升降"**（实测那些帧的三维速率均
   60 m/s）。同口径只有一个办法：**两边都从位置差分算**。所以工具里
   `_on_grid()` 返回的是差分出来的三维/水平速率，`speed_mps` 列**只用来给人看**，
   不参与任何统计。
   - 由此得到的两列：**停摆 %（三维 < 5 m/s）** = 真没在动；**零前进 %
     （水平 < 5 m/s）** = 水平没挪窝。
   - 同理，**比"飞了多远"要用三维航迹**：AFSIM 爬升时把水平分量让出去
     （flyer1 水平 40.3 km / 三维 61.3 km，差 21 km 全在爬升里）。

---

## 8. 第三份：`demos/route_finder_demos/3d_route_finder.txt`（选路）

### 8.1 这一份比的东西与前两份不是一类

前两份（kinematic / aircraft）比的是"**给定航线谁跟得住**"——跑动力学。
这一份比的是"**给定分层禁飞圆柱谁选得短**"——跑**地图服务**，不跑动力学。

原 demo 的 `3D_ROUTER` 处理器干的事：对每一层高度 `alt`（40000 ft 往下到
10000 ft，步长 1000 ft），把每个 SAM 当一根圆柱、半径 = `alt × 7.0`，调
`WsfRouteFinder` 从起点找一条到目标的航路；在 31 条里挑**三维长度最短**的那条
`SetRoute` 给 striker1。所以复现想定 `route_finder.txt` 里**没有
`component_type`**——选路与机动件无关。

`tools/demo_route_finder.py` 把同一件事在 milsim 里重做：逐层建"禁飞格集合"、
逐层调一次 `NavService.route`、按水平航程挑最短。它还会自己跑三次 AFSIM
（航迹 / **选中的航路本身** / 端点漂移诊断）——`afsim_ref.py` 的命令行放不下
多行注入，所以调的是它的 `capture()`。

### 8.2 结果

```
格 1000 m，格心距 1732.1 m；起终点直线 347.67 km
AFSIM 的层梯：40000 → 10000 ft，步长 1000 ft，共 31 层；半径 = 层高 × 7

  层 ft   半径 km   禁区格   航路格   水平 km   比直线   比底数   余量 km   展开
                                      ── 上面三个是小节 8.5 要读的口径 ──
  40000     85.3    27753     331     573.31    1.649    1.607     0.01   8632 ＊
  35000     74.7    23217     317     549.06    1.579    1.539     0.02   8133 ＊
  30000     64.0    18858     303     524.81    1.510    1.471     0.05   7681 ＊
  20000     42.7    10109     283     490.17    1.410    1.374     0.01   7516 ＊
  16000     34.1     6988     277     479.78    1.380    1.345     0.01   7714 ＊
  15000     32.0     6197     229     396.64    1.141    1.112     0.03   1155 ＊   ← 台阶在这里
  14000     29.9     5391     227     393.18    1.131    1.102     0.07    966 ＊
  13000     27.7     4645     224     387.98    1.116    1.087     0.01    738 ＊
  12000     25.6     3967     221     382.78    1.101    1.073     0.10   2999
  11000     23.5     3335     219     379.32    1.091    1.063     0.09   2986
  10000     21.3     2757     215     372.39    1.071    1.044     0.23   2704   ← milsim 选这层
   底数       —         0     206     356.80    1.026    1.000       —    2217   ← 无禁区

AFSIM 选中的航路：8 个航路点（7 段），水平 349.76 km（比直线 1.006），
                  三维 350.26 km；最低层 10000 ft
    R0  0.17470  3.86220  10668.0 m  (35000 ft)   ← 起点
    R1  0.29840  2.84360   3048.0 m  (10000 ft)   ┐
    R2  0.29980  2.82070   3048.0 m  (10000 ft)   ├ sam3 北侧的弧（3 点，每段 ≈2.6 km）
    R3  0.29750  2.79790   3048.0 m  (10000 ft)   ┘
    R4  0.18080  1.81410   3048.0 m  (10000 ft)   ┐
    R5  0.17940  1.79040   3048.0 m  (10000 ft)   ├ sam5 南侧的弧（3 点）
    R6  0.18110  1.76680   3048.0 m  (10000 ft)   ┘
    R7  0.31530  0.74010  10668.0 m  (35000 ft)   ← 目标

绕行幅度（横向偏移 min→max km，正 = 左 = 南）：
    AFSIM     -8.68 →  +9.85
    milsim    -8.95 → +14.94      （选中的 10000 ft 层）
      各 SAM 的位置与要求（该层半径 21.3 km）：
        sam1 沿线 107.7 横向 -115.83      far
        sam2 沿线 118.2 横向  -51.74      far
        sam3 沿线 115.4 横向  +12.96      ⇒ 必须 ≤ -8.34（北）或 ≥ +34.26（南）
        sam4 沿线 102.0 横向  +76.43      far
        sam5 沿线 231.1 横向  -11.70      ⇒ 必须 ≤ -33.00（北）或 ≥ +9.60（南）
```

`＊` = 该层的搜索**降级**了（禁区把战区切碎，`degraded` 置位）——16000 ft 及以上
全部降级，那几行的航程只能当参考。15000 ft 那个台阶（277 格 → 229 格）就是
"降级的大绕行"与"正常绕行"的分界。

### 8.3 结论：选路结论一致

| | AFSIM | milsim |
|---|---|---|
| 选哪一层 | **10000 ft**（最底） | **10000 ft**（最底） |
| 绕哪几个 SAM | 只有 sam3、sam5（另三个在半径外 20 km 以上） | 同 |
| sam3 绕哪一侧 | **北**（横向 −8.68） | **北**（横向 −8.95） |
| sam5 绕哪一侧 | **南**（横向 +9.85） | **南**（横向 +14.94） |
| 贴外沿留多少 | **+0.18 km** | **+0.23 km** |
| 航路点数 | 8 点 / 7 段 | 215 格 / 214 步 |
| 水平航程 | 349.76 km（**1.006×** 直线） | 372.39 km（**1.071×** 直线） |

**逐层趋势也一致**：层越低半径越小、航程越短，31 层单调（中间那个台阶除外），
所以"往低层钻"这个结论两边是同一条。这正是原脚本 `mRadiusPerAltitude = 7.0`
的用意——半径正比于层高。

### 8.4 航程那一列**不能直接比**（这是本节最容易被读错的地方）

上表的 349.76 vs 372.39 km 看起来差 6.5%，但它**不是绕行差**：

- **AFSIM 输出的是切线与圆弧拼成的连续折线**，航路点可以落在任意位置，
  所以它能贴到"直线 + 0.6%"（+2.1 km）；
- **milsim 的 A* 走格心**，路径长度 = 步数 × 格心距，而步数的下界由六边距离
  给出 —— **这本身就是一把不准的尺**：同一段直线，方向贴着格轴时 = 1.000×，
  偏离格轴 30° 时 = 1.155×（最坏）。实测**无禁区**的 A* 也要
  **206 步 = 356.80 km（1.0263×）**。

⇒ 两个 `1.000` 不在同一条基线上。工具因此额外跑一趟"无禁区"当底数，并同时报
两个比值：`比直线`（总共绕了多少）与 **`比底数`（绕行本身贵不贵）**。
按 `比底数` 读：**AFSIM 1.000×（它没有底数，直线就是它的底数）、milsim 1.044×**
——**绕行本身的代价是 4.4%，不是 7.1%**。已记进 `docs/00-框架设计.md` §9-30。

### 8.5 原 demo 自己的一个缺陷：端点被逐层拖进 6.3 km

`on_initialize2` 的循环里写着：

```c
beg = path.Front().Location();   beg.SetAltitudeAGL(src.Altitude());
end = path.Back().Location();    end.SetAltitudeAGL(tgt.Altitude());
```

`beg` / `end` 是**循环外的局部量**，下一层却拿上一层的 `path.Front()` 去覆盖
它们 —— 于是 31 层下来端点被一路往里拖。实测：

```
原样          起点缩进  6.297 km  终点缩进  6.307 km  航路点 8 个
重建 beg/end  起点缩进  0.079 km  终点缩进  0.078 km  航路点 8 个
```

**每层从 `src`/`tgt` 重建之后，选出来的航路点与"只跑最底一层"逐字相同**，
所以这不是"换了个答案"，是同一个答案被端点拖动了。上面表里的 349.76 km
用的是**重建那一趟**：原样那趟因为没有端点缩进、航路比"起终点直线"还短
（337.35 km，0.970×），拿它比长度是错的。工具每次跑三次 AFSIM，把两个数
都打出来——**"参照数据是怎么产生的"和参照数据一样重要**。

顺带排除了两个别的解释（都做了对照实验）：

- **不是 `SHIFT` 响应触发的。** 文档说 `SetImpossibleRouteResponse` 的三种取值
  只在"起点或终点落在规避区内 / 被围住"时才动手。实测把 `"SHIFT"` 换成
  `"IGNORE"`，**航路与航迹逐字相同**；而且两个端点离最近的 SAM 有 116 / 117 km，
  而最大半径只有 85.3 km —— 端点根本不在规避区里。
- **不是转弯半径。** 把 `mFinder.Route(..., 250)` 的航路速度换成 125 / 500，
  端点缩进**逐字不变**（6.297 / 6.307 km）。250²/g = 6.373 km 只是巧合。

还有一条**值得知道的行为**：把 `mRadiusPerAltitude` 改成 0.5（禁飞半径缩到
1.5~6.1 km，远离所有 SAM）之后，**一个航路点都 dump 不出来** —— 因为 finder
在不需要规避时只返回 **2 点**航路，被脚本那句 `if (path.Size() > 2)` 过滤掉
（注释写着"忽略那些飞行高度过低、无需规避的路径"），`shortestRoute` 始终
invalid ⇒ **不调 `SetRoute`**，平台退回**脚本自带的那条两段直线航路**、
速度也退回 450 kt（实测 231.5 m/s）。所以那个 `Size() > 2` 的判断是**真的会
改行为**的，不是纯粹的过滤。

---

## 9. `demos/` 下到底有几个"机动"（清点与判据）

"机动"这件事在 AFSIM `demos/` 底下有**两层**，判据不一样：**平台机动件**（跟航线
的那几个件，§5.10）与**弹**（`WSF_GUIDED_MOVER` + `WSF_GUIDANCE_COMPUTER`，
§5.11）。上一版只清了第一层，把第二层用一句"属武器"划出去了——**划得太宽**，
补在 §9.2。

### 9.1 平台机动件：共 **7 个候选**（其中 1 个不是 AFSIM 原生）

逐个的判定（**同一份判据也写进了 `docs/00-框架设计.md` §5.10.6**）：

| # | demo | 机动相关点 | 参照能采 | 本项目的处置 |
|---|---|---|---|---|
| 1 | `kinematic_mover/` | `WSF_KINEMATIC_MOVER` vs `WSF_AIR_MOVER`；同一航线三组过载/角速度限制 | ✅ | **已对照**（§1~§4） |
| 2 | `mover_demo/`（**不是 AFSIM 原生**） | `WSF_AIR_MOVER` / `KINEMATIC` / `ROTORCRAFT` 的参数罗列 | 需重写脚本（§5） | **列参数、不当基线**（§5） |
| 3 | `route_finder_demos/3d_route_finder.txt` | `WsfRouteFinder` 分层避障选路 | ✅ | **已对照**（§8），落成 `route_finder.txt` + `tools/demo_route_finder.py` |
| 4 | `example_scripts/proc_mover_demo.txt` | **脚本处理器接管运动**（`SetLocation` 直写航线） | ✅ | 未对照，见下 |
| 5 | `alv_routing/alv_demo.txt` | 全局 `route` + `goto` 回环 + 任务管理器重规划 | ✅ | 未对照，见下 |
| 6 | `multiresolution_demos/air_mover_demo.txt` | `WSF_MULTIRESOLUTION_MOVER` **按信度切节拍**（30 s / 0.1 s） | ✅ | 节拍那条线**做成负结果**，见下 |
| 7 | `terrain_following/terrain_demo.txt` | `GoToAltitude` 地形跟随 | ✅（但退化） | **判定不做实质对照**，见下 |

（`hellfire/`、`parachute/`、`six_dof/` 这些**不在这张表里**，但也不是"不查"——
它们归到 §9.2 的**弹**那一层去了。）

#### 4. `proc_mover_demo`：脚本直写位置，不是机动件

它用 `SetLocation` 每帧把平台**搬到**航线上的下一个点，`WSF_KINEMATIC_MOVER`
只是挂在平台上而**不参与运动**。所以它不是"另一个机动件"，是"**不用机动件**"。
要对照就得先回答"我们这边允许不允许脚本直写位置"——那是决策层的事
（§4 决策抽象层），不是机动件的事。**等决策层落地再说**。

#### 5. `alv_routing`：一半是选路、一半是任务管理

它的运动部分是"一条全局 `route` + `goto` 回环"，**与已做的 kinematic 那一份
是同一个东西**（我们的 `move_along_route` 已能覆盖）；真正没有对应物的是
`alv_task_mgr` 那个**任务管理器**——它按态势重新下达任务、进而改航路。
要对照得先把决策层做出来。**它的选路部分会作为 §8 那个工具的一个附加用例**，
不单独立一份。

#### 6. `multiresolution_demos`：变保真度节拍 —— 这条线是**负结果**

它展示 `WSF_MULTIRESOLUTION_MOVER` 按信度在 30 s 与 0.1 s 两种节拍之间切。
我们这边对应的是组件参数 `period`（与 AFSIM 的 `update_interval` 同一件事），
所以可以两边各扫一遍节拍。**扫完的结论是"这条线不做交付对照"**：

- **AFSIM 侧有一个隐蔽的封顶**：`event_pipe` 的
  `maximum_mover_update_interval` **默认 5 秒**，它把机动件的更新节拍盖在
  5 s 以内。这就是 `update_interval 30 sec` 跑出来与 5 s **逐字相同**的原因
  ——不是参数无效，是被上面那一层挡住了。原 demo 特意写
  `maximum_mover_update_interval 0 sec` 就是为了绕开它；用
  `tools/afsim_ref.py --replace` 补上这一行之后，5 / 15 / 30 s 分别采到
  **484 / 204 / 124** 个样本，节拍才真的生效。
- **粗节拍下位置成了阶梯**（5 s 里只有 1 帧在动），于是三项指标**同时失真**：
  零前进占比 12~17% → **90.1~90.6%** → **98.3%**，峰值 ω 20.8/11.5 → **208/114.6**
  → **346.3/348.5 °/s**。它们量的是**采样密度，不是飞行状态**。
- AFSIM 侧同样塌：30 s 节拍下运动学件的高度发散到 **−15668 / −34324 m**。

⇒ 一是节拍必须与采样同步（否则量出来的东西没有意义），二是"降低保真度"这件事
在 AFSIM 那边**本身也没被控制住**，拿它当基线是拿一个发散的结果当参照。
它的价值是把"**节拍是一个会改变结论的参数**"这件事量出来了，记在这里。

#### 7. `terrain_following`：缺数据，退化成定高平飞

它靠 `GoToAltitude` 跟地形（AGL 定高）。判据是"**有没有那片地形**"：
AFSIM 自带的 dted 只有 `coverage_demos/dted/w107/n39.dt2`（107°W / 39°N），
而这个 demo 的世界坐标在 **115°W / 36°N** ⇒ 地形读不到，高度**恒 305 m**
（实测 361 个样本里高度一个值都没变）。所以它跑出来的不是"地形跟随"，是
"定高平飞"——**两边比的是同一个常数，没有信息量**。

**要做得先有地形数据。** 我们的 `terrain` 是 `flat / procedural / none`
（§3.1），没有 DTED 读入。等有真实地形之后再补这一份，那时要对照的
"跟地高度"才第一次成为一个可比的量。

### 9.2 弹：`WSF_GUIDED_MOVER` + `WSF_GUIDANCE_COMPUTER`

上一版拿一句"属武器 / 六自由度 / 伞降"把整类划出去，**划得太宽**：§5.8 花了一整节
做的正是"**弹体 + 制导件**"两个部件，而 `WSF_GUIDED_MOVER` 在 `demos/` 下出现在
**34 个目录、101 个文件**里。所以另列一张表，判据换成弹这一层的四问：

1. **弹是不是这个 demo 的题目？**（README / `.rst` 头一句在讲弹，还是在讲感知、
   数据链、交战流程、指挥、签名？）
2. 弹用的是不是 `WSF_GUIDED_MOVER` + `WSF_GUIDANCE_COMPUTER`——**也就是**我们的
   `MISSILE_MOVER` + `ASM_GUIDANCE` / `CRUISE_GUIDANCE` / `BALLISTIC_GUIDANCE`
   的对应物？
3. 弹道形状由**阶段表**决定，还是由**射击几何 / 导引头 / 数据链**决定？
4. 这份 demo 的脚本**是 AFSIM 原版带的**吗？

| # | demo | 弹相关点 | 参照能采 | 处置 |
|---|---|---|---|---|
| 1 | `new_guidance/` | `WSF_GUIDED_MOVER` + `WSF_GUIDANCE_COMPUTER`；3 份想定，官方自己标"最复杂 → 最简" | ✅ exit 0 | **已对照**（§11），落成 `missile_guidance.txt` + `tools/demo_afsim_missile.py` |
| 2 | `ballistic/` | 5 型弹道弹（`red_srbm_1/2/3/4`、`red_mrbm_2`，弹体在 `../base_types/weapons/ssm`）从地面打出去 | 待试 | 未做，见下 |
| 3 | `tbm_demos/` | **`WSF_TBM_MOVER`**——不是 `GUIDED_MOVER`，是弹道专用件 | — | **不做**，见下 |
| 4 | `base_types/`（+`base_types_nx/`） | 武器**类型库**：巡航弹 / 弹道弹 / SAM / AAM / 制导炸弹各一 | 可采 | **当参数取源，不当独立对照** |
| 5 | `hellfire/`（3 份）、`laser_designator/` | 弹是半主动激光 / 主动雷达导引头的载体，题目是**照射链** | 可采 | **不做**（没有照射件与导引头） |
| 6 | `air_to_air/`、`brawler/`、`shooter/`、`behavior_tree/` | AAM 交战：发射条件、导引头、脱离 | 可采 | **不做**（属决策 / 感知层） |
| 7 | `ballistic_missile_shootdown/` | 题目是**什么时候发射拦截弹**（processor 算拦截点） | 可采 | **不做**（属决策层） |
| 8 | `orwaca_iads/`、`iads/`、`iads_c2_demos/`、`ship_ad/`、`suppressor/`、`engage/`、`launcher/`、`space_operations/`、`satellite_demos/`、`cyber/`、`swarm/`、`signature_demos/`、`script_demos/`、`draw/`、`alternate_locations/`、`electronic_warfare/`、`wargame/` | 弹是**夹具**（指挥 / 签名 / 脚本 / 干扰 / 装填的载体） | — | **不做** |
| 9 | `yj18a/`、`yj20/`、`yj21/`、`yj21a/` | **本项目自己搭的**反舰弹想定（`YJ-xx参数参考.txt`、`.bak`、`analyze_diag.py`） | ✅ | **不是 AFSIM 原生基线**，但**是现成的用例** |
| 10 | `combined/` | 目录里混进了 `yj18a.txt` / `yj20.txt` / `yj21a.txt` / `df17h*.txt` / `df26d.txt` / `df27.txt` | — | 该目录**已被改过**，引用时先分清哪几行是原版 |
| 11 | `parachute/`、`six_dof/`、`p6dof/`、`six_dof_with_brawler/` | 伞降 / 六自由度 | — | 不在这一层（我们是三自由度） |

34 个目录里，真正"**弹当主体 + 阶段表决定弹道**"的只有 `new_guidance/` 一份——
**它是这一层的唯一原生基线，已经对照完了（§11）**。剩下几条逐个交代：

#### 2. `ballistic/`：唯一还值得补的一份（但先要拍板一条）

`setup.txt` 里连着两行 `file_path`（`file_path .`，然后 `file_path ../base_types`），
**后一行赢** ⇒ 弹体解析到 `base_types/weapons/ssm/red_*bm_*.txt`——又踩一次 §10.2
那条。可采，而且这一份**正好落在 `BALLISTIC_GUIDANCE` 能表达的范围内**：
`red_srbm_1.txt` 的制导件也是三段，
`LIFTOFF → PITCH_OVER → BALLISTIC`，与 `ballistic.py` 的阶段表**逐段对得上**
（模块头就写着"AFSIM 的 `red_icbm_1.txt` 是同一形状"）。

差在两处，一处已知一处**新**：

- **已知**：`PITCH_OVER` 的倾角它写 `commanded_flight_path_angle from_launch_computer`
  （由 `WSF_BALLISTIC_MISSILE_LAUNCH_COMPUTER` + `*_launch_data.txt` 按射程解算），
  我们写**固定 50°**。这就是 `ballistic.py` 模块头"不做的：没有瞄准计算"那一条。
- **新**：它的推进条件是 **`when on_commanded_flight_path_angle`**（"倾角到位就切走"），
  我们用 `until=Until.event("burnout")`（"转到熄火"）。后果是**主动段后半段的形状
  不一样**：它转到位就交给无制导（注释写"重力转弯"），此时**还有推力**；我们一直
  用 `VERT_FPA` 把倾角钉在 50° 直到熄火。`on_commanded_flight_path_angle` 这个
  标志**我们没有**（`UNTIL_FLAGS` 只有 `burnout` / `descending`），要对照就得先
  决定加不加——**与 §11.4 的 `target_elevation` 是同一类缺口**。

所以它**能做，但先要拍板那一条**。留作下一步。

#### 3. `tbm_demos/`：`WSF_TBM_MOVER` 是另一个件

它写 `mover WSF_TBM_MOVER`（`weapons/ssm/tbm1/2/3.txt`），README 讲的是
**多级 / 分导**（`scenarios/launch1multi_stage_tbm.txt`）。这个件把"级间分离"写在
**件里面**，而我们的 `MISSILE_MOVER` 是**单级弹体**（`fuel_mass` / `thrust` 一套
参数走到底，`WSF_GUIDED_MOVER` 那边叫 `stage 1`，也只有一级）。要对照得先决定
"多级要不要做"，那是 §5.8 的功能决策，不是复现的事。**不做。**

#### 4. `base_types/`：参数取源，不是对照对象

它是**类型库**（`weapons/ssm|sam|aam|agm` 各一型），题目是"一个类型该怎么写"，
`demos/ballistic/` 的弹体就从这里取（`red_srbm_1.txt` 里 `specific_impulse 220 sec`
是**真给**的，没有像 §11.3 那样被注释掉）。**用它取参数时仍要按 §11.3 换算**
（燃料流率、阻力口径），别照抄。

#### 9. `yj18a` / `yj20` / `yj21` / `yj21a`：我们自己搭的，别当基线

这四个目录里有 `YJ-18A参数参考.txt`、`weapons/yj20.txt.bak*`、
`tools/analyze_diag.py`、`output/*.aer`——是**在 AFSIM 里自己搭的反舰弹想定**，
不是 AFSIM 原版 demo（AFSIM 不会带这些文件）。但它们恰好是**最好的现成用例**：
哪天真要让本项目的弹去复现"某型反舰弹的射程 / 末段"，直接拿这里的 `.aer` 当
参照就行（在 AFSIM 侧重跑一遍即可）。**只是不要当"原生基线"引用**——它没有
AFSIM 的 doc 背书。

---

## 10. 一条给改这个工具的人的清单

改 `tools/afsim_ref.py` / `tools/demo_afsim_mover.py` / `tools/demo_route_finder.py` /
`tools/demo_afsim_missile.py` 之前，先读 §7 的三条 + 下面四条（都在本轮踩过）：

1. **素材路径要"镜像"，不是"改写"。** 原 demo 的 `include_once movers/x.txt`、
   `dted 1 dted/w107`、`log_file output/$(CASE).log` 都是**相对原想定目录**的，
   而工具把脚本写进临时目录 ⇒ `FATAL: Cannot open file`。`_stage_assets()` 按
   同样的相对结构把素材复制过去、再建出输出路径的父目录，**脚本文本一个字不改**
   （读脚本的人看到的仍是原 demo 的写法）。
2. **`include` 的基准是 `file_path`，不是被包含文件所在目录。**
   `alv_routing/setup.txt` 写着 `file_path .`，于是 `platforms/alv.txt` 里的
   `include processors/alv_task_mgr.txt` 解成 `alv_routing/processors/...`，
   **不是** `alv_routing/platforms/processors/...`。按后者递归会解析失败。
3. **解析 AFSIM 脚本时，参数可能被注释掉的旧值压着。**
   `3d_route_finder.txt` 里 `mRadiusPerAltitude` 的真值（7.0）上面就压着一行
   `#double mRadiusPerAltitude = 2.5;`。正则不锚行首就会读到注释，半径系数
   悄悄变成 2.5 —— 而 2.5 也是"一个合理的数"，**图和数据都不会报错**。
   同理 `position` 的经纬度有两种合法写法（度分秒 / 十进制），只认一种会
   **静默漏掉整行**（坐标停在默认 0）。
4. **弹那一份特有的两条（§11.5）**：参照里的弹叫 `fighter_blue_asm_missile_1`
   ——**载机名是前缀**，用"名字里含 fighter"当"这是载机"会把弹一起滤掉（第一版
   据此报"一个弹的样本都没有"）；AFSIM `writeln` 的 `%g` 只有 **6 位有效数字**，
   "保持巡航高度"不能按逐字相等判（否则 533 s 的巡航段被切碎，报出爬升前的
   7620 m 当巡航高度）。

---

## 11. 第四份：`demos/new_guidance/new_guidance3.txt`（弹：弹体 + 制导件）

### 11.1 这一份比的不是"谁跟得住航线"

前三份跑的是**平台机动件**。这一份比的是 `docs/00-框架设计.md` §5.8 的**两个部件**：

```text
AFSIM    WSF_GUIDED_MOVER            +  WSF_GUIDANCE_COMPUTER
本项目   MISSILE_MOVER               +  ASM_GUIDANCE     ← v0.13.22 起用这个
         （能飞多快、烧多少油）          CRUISE_GUIDANCE / BALLISTIC_GUIDANCE
                                         （要往哪飞——阶段表）
```

**v0.13.22 之前**这一份跑的是 `CRUISE_GUIDANCE`（`LAUNCH → CRUISE 50 m →
TERMINAL`），因为参照那套 `CRUISE → POPUP → DIVE` 当时写不出来。补上
`target_elevation` 之后，参照那三段现在就是 `ASM_GUIDANCE` 的默认表
（`models/guidance/asm.py`，照抄原脚本），这一份也跟着换了过去——**下表是换过之后的数**。

原 demo 自己有三份想定（`README.md` + `doc/new_guidance.rst`），官方自己标了
"一份比一份简"：

- `new_guidance1.txt`（**最复杂**）——舰射、远程传感器给航迹、飞行中收目标位置更新、
  末段由弹上雷达导引头接管；
- `new_guidance2.txt`——把 1 的弹挂到飞机上，打一条**预先装订**的航迹（无传感器）；
- `new_guidance3.txt`（**最简**）——"a weapon fly-out that **has to go to a lat and
  lon en-route to the target**"（先去一个中途经纬度、再到目标），而且
  **它用的不是前两份那枚弹**。

选 3 是因为 1 / 2 多出来的东西是雷达、航迹、中途更新——那是感知与数据链，不是弹体
这一层的事。**★ 这一份不是原 demo 的等价复现**，读数字前先读 §11.4。

### 11.2 结果

| | AFSIM | 本项目 |
|---|---|---|
| 飞行时间 s | 625.5 | 554.0 |
| 巡航高度 m | **75**（71~605 s） | **75**（114~538 s） |
| 巡航速率 m/s | **240.0**（218~604 s） | **240.0**（0~538 s） |
| 水平平移 km | 132.2 | 132.1 |
| 水平航迹 km | 155.3 | 132.1 |
| 峰值速率 m/s | 365.6 | 322.4 |
| 终止质量 kg | 499.16（604.1 s 时） | 503.3 |
| 峰值过载 g | 无显式上限 | 4.59 / 上限 4.6 |
| 终止 | 命中（脱靶 **0.0 m**） | **落地**（没进起爆区） |

**一句话：巡航段对得上，中段三段都飞得出来，末段打不中——原因查清了（§11.6）。**

逐条读：

- **巡航高度与速率两边都平了**：75 m / 240.0 m/s，逐字一致。这是最硬的一条——
  同一组质量 / 推力 / 阻力，加同一条"定高 75 m、定速 240 m/s"的指令。
  v0.13.22 之前我们的巡航高度是 50 m（那是 `CRUISE_PHASES` 的默认值），
  换到 `ASM_GUIDANCE` 之后它来自**照抄的原脚本**，于是自动对齐了。
- **水平平移 132.2 vs 132.1**：发射点与目标点是同一组经纬度，这一列量的不是
  "飞得好不好"，是"起点到终点有多远"——两边当然接近。
- **水平航迹 155.3 vs 132.1（差 23.2 km）**：参照先走完**预规划航路**
  （`39.0N/89.5W` → `38.8N/89.5W`，`End of route encountered T=309.6`）再转向目标，
  我们**直扑**（§11.4 第 1 条）。那 23 km 全在航路上。
- **峰值速率 365.6 vs 322.4**：两边的来处完全不同。参照那一峰在**投放后的下降段**
  （弹从 **7620 m** 载机高度降到 75 m，t≈46 s）；我们那一峰在**末段俯冲**。
  见下面"发射高度"那条。
- **峰值过载 4.59 / 上限 4.6**：又顶着自己的上限飞。这个 45 m/s² 不是随手给的，
  是从参照 POPUP 段那 6 s 抬头的角速度倒推的（§11.4 末）。
- **"落地"vs"脱靶 0.0 m"**：这一列才是这一份真正的结论——**末段没打中**。
  不是参数错、也不是表错，是弹体的一处已知限制，根因与数字全在 §11.6。
- **★ 发射高度：参照 7620 m（25000 ft，载机高度），复现 z ≈ 0 m。** 复现想定里
  `position latlng 39.5 -89.3` **给不出高度**——`Simulation._locate` 对 `hex` 与
  `latlng` **两种形式都返回 z = 0**（§9-22 那条只写了 `hex`，其实两条路一样）。
  定高巡航段因此不受影响（两边都稳在 75 m），但**投放后的下降加速段只有参照有**
  ——参照那 365.6 m/s 的峰就是它。想定里能给弹的只有 `launch_speed 231.5 m/s`
  与 `launch_fpa`，**高度给不了**。见 §9-34。

阶段（参照侧只能从 AFSIM 自己的输出里读，见 §11.5 第 3 条）：

```text
AFSIM    CRUISE → POPUP（T=604.1   Alt 75 m   Downrange 129423 m
                                 Mass 499.16 kg   Speed 240 m/s）
                → DIVE （T=610.1   Alt 738 m  Pitch 67.57°  Speed 178 m/s）
                → 命中  T=625.5    Alt −0.13 m  Speed 249.99 m/s  脱靶 0.0 m
本项目   CRUISE(0~536 s) → POPUP(536~542 s) → DIVE(542~554 s) → 落地
```

**三段的名字与次序现在两边一样了**（v0.13.22 之前我们只有
`LAUNCH → CRUISE → TERMINAL`）。切段判据也是同一句：
`target_slant_range < 3000 m` ↔ `range < 3000`，
`target_elevation < −20 deg` ↔ `target_elevation < −20`。

**耗油：两个"全程平均"不可比，别拿它们相减。**

```text
参照    19.84 kg / 604.1 s = 0.0328 kg/s（这段含**起飞与爬升加速**、不含跃升/俯冲）
本项目  15.72 kg / 554.0 s = 0.0284 kg/s（这段含**末段俯冲**）
        ├ 单看巡航平台段（114~538 s）  9.776 kg / 424 s = 0.0231 kg/s
        └ 满推力 0.2000 kg/s（= 参照的 fuel_mass ÷ thrust_duration）
          ⇒ 全程平均油门 14.2%
```

**巡航段那个 0.0231 kg/s 可以交叉验算**：240 m/s、75 m 处 ρ ≈ 1.214 kg/m³、
`Cd·A = 0.00923 m²` ⇒ 阻力 `½ρV²·Cd·A = 323 N` ⇒ 平飞时推力 = 阻力 ⇒
耗油率 `323 ÷ (1478 × 9.80665) = 0.0223 kg/s`。实测 0.0231（含进出平台的过渡），
**对得上**。

★ **但"0.0328 vs 0.0231"仍然不可比**：参照那个数是**发射到 604.1 s 的平均**，
含起飞、爬升与加速；我们的 0.0231 是**纯巡航平台**。而参照侧的**纯巡航段**
耗油**没有采**——`afsim_ref.py` 的观测器只写
`time/platform/lat/lon/alt/speed/heading` 七列，**没有质量**；要算它得给脚本加一列
`aPlatform.Mass()` 再重采一次（那会改到另外三份 CSV 的列，所以没顺手改）。
**这是本节的未闭环项。**

### 11.3 两处必须换算的常量（不换算就静默跑出另一枚弹）

参照那份弹体是 `weapons/agm/blue_asm_missile.txt` 的 `stage 1`：

```text
initial_mass 519 kg     fuel_mass 200 kg     thrust 2900 Nt
thrust_duration 1000 sec                     #specific_impulse 300 sec   ← 被注释掉
cd_zero_subsonic 0.10   cd_zero_supersonic 0.25   mach_begin_cd_rise 0.95
reference_area 0.092347 m2                   cl_max 10.0   aspect_ratio 16.0
```

1. **燃料模型口径不同。** AFSIM 的 `WSF_GUIDED_MOVER` 用
   `fuel_mass ÷ thrust_duration` 定**满推力质量流率**（200 ÷ 1000 = **0.2000 kg/s**），
   `specific_impulse` 那一行**是注释掉的**；我们的 `MissileMover._step` 按
   `thrust ÷ (Isp·g)` 烧 ⇒ 想定里必须写
   **`specific_impulse 1478 s`** = 2900 ÷ (0.2000 × 9.80665)。
   **照抄注释里那个 300 s 会让燃烧率差 4.9 倍**，而两者都是"合理的数"、图也照画。
2. **阻力口径不同。** AFSIM 用气动表（有马赫数、有跨音速跳升），我们只有**常数**
   `drag_area`（§5.8 明记的已知限制）⇒ 取 `0.10 × 0.092347 = 0.00923 m²`，
   **只在亚音速段可比**。240 m/s = M0.71，还在 `mach_begin_cd_rise 0.95` **以下**，
   所以这一段两边口径一致；再快就不一致了。

### 11.4 缺口清单：两条仍缺、两条 v0.13.22 补上了

**仍缺（结构性，不是"还没做"）**

1. **预规划航路**（§9-31）。原脚本在 **mover 里**写了 `route`，CRUISE 段写
   `allow_route_following true`，实测弹**先走完那两个航路点再转向目标**
   （t=217.1 到航路点 1、t=309.6 走完航路）。本项目的
   `MissileMover.move_to_point()` / `move_to_cell()` **显式抛 `ConfigurationError`**
   （"弹不接受按点下达目的地——它的轨迹由制导件按阶段决定"）⇒ 复现侧**只能直扑**。
   **这是横向对不上的唯一原因**（水平航迹 132.1 vs 155.3 km）。
2. **`maximum_pitch_angle 50 deg` 无对应物**（§9-35）。我们不夹俯仰角；它与
   机动件那条"航迹角上限"（§9-28）**是同一个量**，所以两条应该一起定。

**已补（v0.13.22）**

3. ~~`target_elevation` 这类推进条件我们没有~~ ⇒ **补上了**（§9-33）。
   原脚本的注释里**自己列了一遍**可用变量（15 个：`target_slant_range` /
   `target_elevation` / `los_target_azimuth` / `los_target_elevation` /
   `time_to_intercept` …），我们原本只有 9 个。这一份真正用到的只有**一个**：
   `POPUP` 段那句 `next_phase DIVE when target_elevation < −20 deg`。现在
   `UNTIL_VARIABLES` 多了 **`target_elevation`**（度，正 = 目标在弹**上方**），
   由 `GuidanceComputer._geometry()` 从**同一支视线**上取出（与 `λ̇_v` 同源：
   `atan2(dz, r_xy)`）。**它顶不了 `descending`**——"目标掉到视线下方"与
   "自己在下降"不是同一个问题，跃升段里后者一帧都不成立。
4. ~~阶段表写不进想定~~ ⇒ **一半补上了**（§9-32 **仍然开着**）。
   想定作者**还是改不了表里的值**（`Params.phases` 是类默认值），但参照那套
   `CRUISE → POPUP → DIVE` 已经作为**一张现成的表**落进框架
   （`ASM_GUIDANCE`，`models/guidance/asm.py`，逐句照抄原脚本）⇒ 这一份现在
   飞得出来了，"换一种弹 = 换一个**组件名**"。**要定的是"想定作者该不该碰表"
   这件事本身。**

**弹体那一侧的配套：`radial_accel 45 m/s²` 是倒推出来的，不是随手给的**

参照那枚弹在 POPUP 段用 6 s 把弹道倾角从 0 抬到 67.6°（t=604.1 → 610.1，速率
240 → 178 m/s）⇒ `a = v·ω ≈ 240 × (67.6° = 1.18 rad) ÷ 6 s ≈ 47 m/s²`。
实测扫过**同一张表**：`radial_accel` 取 12 / 30 m/s² 时弹只抬到 15°、在目标
**头顶 750 m** 掠过（最近通过 754 / 804 m，不命中）；**45 起命中**（另一个
40 km 算例上最近通过 28.8 m）。⇒ **表与弹体是配套的**，`ASM_GUIDANCE` 配一具
拉不动 4.6 g 的弹体等于白给。

### 11.5 参照侧的三个读法坑

1. **`writeln` 的 `%g` 只有 6 位有效数字。** "巡航高度 75 m"实际在
   74.98 / 75.00 / 75.01 之间跳 ⇒ 按**逐字相等**判"保持"会把 533 s 的巡航段切成
   2 s 一小段，于是**报出爬升前的 7620 m 当巡航高度**——数看着完全合理，第一版
   就这么错的。现在两边用**同一套算法**（高度量化 5 m、速率量化 1 m/s）认平台段。
2. **弹的名字带载机前缀。** 弹叫 `fighter_blue_asm_missile_1` ⇒ 用"名字里含
   fighter"当"这是载机"**会把弹一起滤掉**（第一版据此报"一个弹的样本都没有"）。
   `read_ref()` 改成只认含 `missile` 的那一路，并把这条写进源码注释。
3. **CSV 里没有"阶段"这一列。** 参照侧的阶段边界只能去 AFSIM 自己的输出里读
   （`New Phase: POPUP T=604.1`、`New Phase: DIVE T=610.1`）⇒
   `demo_afsim_missile.py` 报的阶段那行**只有复现侧是自动的**，参照侧那两个边界的
   时刻是人工抄的（v0.13.22 起工具会把这句提示连同两个时刻一起打出来，免得读者以为
   两边都是自动对齐的）。数字对不上时先确认是不是抄错了行。

### 11.6 末段为什么打不中（v0.13.22 实测，根因已定位）

**现象**：三段都按同一张表飞出来了，最后一列却是"落地"——弹从目标**上方掠过**。

| 末段某一点 | 参照（t=619.9 s） | 本项目（t=550 s） |
|---|---|---|
| 弹道倾角 γ | −29.7° | −29.5° |
| 水平距离 | 694 m | **171 m** |
| 高度 | 968 m | 772 m |
| 总速率 | **178.8 m/s** | **275.5 m/s** |

**同样的弹道倾角下，我们快 96.7 m/s。** 后果是两条：水平速度 240 vs 155 m/s
⇒ **视线的旋转速度是参照的 1.6 倍**；而转弯角速度 `ω = a/v` 又只有它的 1/1.6
⇒ **追不上视线**。于是弹在水平只剩 171 m 时还有 772 m 高，从目标头顶掠过，
最后撞海面。参照在同一位置已经压到 −57° 的俯冲角上。

**根因在弹体这一侧，而且是 §5.8 里早就写明的已知限制**：`mover/missile.py` 的
"已知限制"第 4 条——**"不转弯消耗动力：诱导阻力不在 `D` 里，所以持续盘旋不掉速。"**
参照那枚用 `WSF_AERO`（有 `cl_max 10.0` / `aspect_ratio 16.0`），转弯产生诱导阻力、
末段掉速；我们只算 `½ρV²·(Cd·A)` 这个常数项，**转弯不掉速**，而 2900 N 推力对
323 N 阻力 ⇒ 末段一路加速（实测 244 → 275 → 322 m/s，峰值就是它）。

**扫过参数，确认这不是"参数没调好"**（132 km 算例，`lethal_radius 30 m`）：

```text
改动                            结果
弹体 radial_accel 45 m/s²       不命中（最近三维 567.3 m）
弹体 radial_accel 70 m/s²       命中（最近 29.8 m）   ← 要 7 g 才转得回来
只给 POPUP 段速度指令 190 m/s    不命中，但最近距离 567 → 107.4 m
只给 POPUP 段速度指令 130 m/s    同上（107.4 m，再降也饱和）
```

两条路都指向同一件事：**要么给更大的过载把这 96.7 m/s 硬转回来，要么让弹在
末段掉速。** 而参照**两样都没特别设置**——按 2.3~2.5 s 的窗口从它的轨迹反推，
末段过载只有 **2~3.6 g**，速度也是自己掉下去的。所以这一条不是参数问题。

**要定的**：给弹体补诱导阻力（`C_di = C_l²/(π·e·AR)`，需要升力与动压，属气动表
那一档），还是接受"末段靠过载硬转"（把 `radial_accel` 提到 7 g 以上）？已记进
§9-36。

**没做的**：预规划航路（§9-31，横向仍然对不上）、参照侧的纯巡航段耗油
（§11.2 末，观测器没有质量列）。

## 12. 火箭弹：`rocket.txt`（无控弹包线，v0.13.51）

### 12.1 "无控弹"不是一个制导件，是"没有制导件"

`MissileMover` 在 `guidance_slot ""` 时只受推力、阻力与重力（`_command`
返回 `phase="无控"`）——`test_empty_slot_is_an_uncontrolled_missile_not_a_bug`
钉的就是这条语义。想定写法只有一条要领：**弹体显式写 `guidance_slot ""`、
平台不装 guidance 组件**；不写槽名的话装配期报"制导槽里没有部件"
（静默退化成无控弹的情形不存在，§5.8 的老约定）。

### 12.2 射程是发射角 × 燃料买来的（探针扫 35 例，v0.13.51）

无控弹离轨即做重力转向（`γ̇ = −g·cosγ / V`），所以射程**不随发射角单调**、
也没有"射程"参数。基准弹体（mass 800 kg / thrust 66 kN / Isp 250 s /
drag_area 0.09 m²）的实测包线——`射程 km / 弹道高点 km / 飞行时间 s`，
终止原因全部"落地"：

| 燃料 | 30° | 45° | 60° | 75° | 89° |
|---|---|---|---|---|---|
| 100 | 0.1/0.0/2 | 3.7/0.2/16 | 6.6/1.3/36 | 5.7/2.9/52 | 0.5/3.8/58 |
| 200 | 0.1/0.0/2 | 8.7/0.4/22 | 16.5/3.4/58 | 16.4/8.7/90 | 1.5/12.0/104 |
| 300 | 0.1/0.0/2 | 12.5/0.5/26 | 25.0/5.4/72 | 30.2/15.8/122 | 3.2/23.8/148 |
| 400 | 0.1/0.0/2 | 15.2/0.5/26 | 31.9/7.3/84 | 51.0/25.1/156 | 6.3/42.9/200 |
| 500 | 0.1/0.0/2 | 17.1/0.5/26 | 37.5/8.9/94 | 89.3/39.6/200 | 13.0/81.0/276 |
| 650 | 0.1/0.0/2 | 18.1/0.5/24 | 41.2/11.1/112 | 245.2/90.4/316 | 47.0/263.7/502 |

mass 1000 kg（净重 200 kg）再探一档：650 kg @75° = 121.3 km；
**800 kg @80° = 482.7 km / 高点 183 km**。⇒ **0~300 km 档由发射角 + 燃料
覆盖**：30° 全部 2 s 内撞地（离轨速度小、被重力转向按进地里）；45°/60°
是中低射程；**75°~80° 是远射程最优角**（真空里"45° 最优"在有阻力与
重力转向时不成立——`missile.py` 模块头的老结论，这张表是它的全景版）；
近垂直（89°）用射程换弹道高。

### 12.3 想定里的一枚坐标坑

第一版把近程弹放在 anchor **北边** 16.5 km 处、heading 0（北）——按
§5.7 "航向 0 = 北 = **y 减小**"，那是**朝着战区中心飞回来**，实测落点
0.9 km、先 16.4 km 一路递减。修正后两枚弹都从 anchor 出发向北：
RKT_1（60°/200 kg）16.5 km、RKT_2（75°/650 kg）245.2 km，与探针逐位
一致。**弹的出发点必须在"前进方向的那一侧"上。**

### 12.4 与弹道导弹的分工

火箭弹没有末段导引，落点散布就是发射角给的；要"打得准"上制导
（`BALLISTIC_GUIDANCE`），要"打得远"加燃料，在这张表里都是明码标价。
射程档位全景见 §13.4 的覆盖表。

## 13. 解析弹道弹：`analytic.txt`（ANALYTIC_BALLISTIC_MOVER，v0.13.51）

### 13.1 为什么需要与 §11 无关的另一条路

`MissileMover` 是平面 3-DOF：直角坐标、常数 g、无地球曲率。打 85 km
它够准，打洲际（10000 km 级、弹道高点 1200 km 量级）**物理上不成立**
——那一档 g 要变三成，平面几何把四分之一个地球当成一张平纸。逐子步
积分的路要球面化，动的是 `KinematicsTable` 与全部实体坐标；解析弹道走
另一条：**弹道不在仿真里"飞"出来，而是在第一次推进前一次解算出来**
——发射点 + 目标点 + 弹道高点 → 球面椭圆闭式根数，之后每帧按开普勒
时间律置位（牛顿解收敛判据 1e-12）。积分的物理错误从根上就不参与。

**物理锚点**（tests/test_analytic.py 钉住）：Φ=90°（≈10008 km）+
高点 1200 km ⇒ 发射速度 7.2 km/s、全程 31.2 min——真实洲际弹道的量级。
平面模型给不出这两个数（它要 9.9 km/s 才飞得了 10000 km），这正是
本件存在的理由。测试 15 例：闭式独立重算对拍（公式与实现分开写，防
同源错）、vis-viva 能量对拍（与时间律无关的另一条通道）、落点/高点/
飞行时间/任意方位、五个防呆校验。

### 13.2 想定写法与端到端实测

```
component_type MRBM : ANALYTIC_BALLISTIC_MOVER
    target_name     TGT_950      # 解算时锁定该平台位置
    apogee_altitude 250 km       # 唯一的形状参数
end_component_type
```

没有制导件、没有 guidance_slot——弹道没有"想怎么飞"的自由度。目标
平台用 `AIR_MOVER`（PROFILE=None，位置即打击点）。想定端到端探针
（run_for 1000 s）：SRBM_1 → TGT_600 与 MRBM_1 → TGT_950 的 miss 都是
**0.000 m**——落点精确等于目标的平面坐标（投影在两边抵消）。

实测出的两条引擎边界，写想定前先知道：

1. **所有平台必须在战区内**：latlng → 米制坐标的投影以战区为基准，
   战区外装配期直接报错（不是静默）。而 `zone.radius` 上限 1000 km、
   `resolution` 上限 2000 m → **洲际档（10008 km）进不了想定**，它的
   锚点由测试用平面坐标直摆钉住；想定演示贴着上限放 600 / 950 km
   两档（全程 421 / 482 s，闭式解算值）。
2. **平台位置吸附战区格网**（格距 2 km，实测 TGT_950 吸附到 y=948 km）：
   弹打的正是吸附后的那个点，两者仍严格重合。

### 13.3 三个已知简化（都写在明处，`analytic.py` 模块头有全文）

1. **主动段不建模**：本件从发射点起就以轨道速度运动（Φ=90° 档首帧
   γ≈21°、7.2 km/s）；真实弹用 100~200 s 加速到关机速度，早期预警段
   的"哪一秒爬到什么高度"是失真的。
2. **目标位置解算时锁定**：洲际弹打的是发射前装订的坐标。目标在飞行
   中被移走 = 弹落在旧位置上，按 `lethal_radius`（默认 30 m）判
   "落地"而非"命中"——test_moved_target_lands_on_the_locked_point 钉住。
3. **发射/落点取同一半径**：球面弹道的落点高度等于发射点高度；平原
   就是海平面，高原上两头的 z 与当地地面一致。

### 13.4 全射程档位覆盖表（§12.4 指的那张）

| 档位 | 射程 | 组件 | 怎么打 | 验证出处 |
|---|---|---|---|---|
| 火箭弹 | 0~300 km | `MISSILE_MOVER` + `guidance_slot ""` | 发射角 × 燃料，无控 | §12.2 探针 35 例（16.5 / 245.2 / 482.7 km） |
| 近程弹道 | 25~85 km | `MISSILE_MOVER` + `BALLISTIC_GUIDANCE` | 阶段表导引 | §11 实测 |
| 中程 | 300~1000 km | `ANALYTIC_BALLISTIC_MOVER` | 解算弹道 | §13.2 想定 600 / 950 km（miss 0.000 m） |
| 巡航 | 20~2200 km | `MISSILE_MOVER` + `CRUISE_GUIDANCE` | 燃料定射程 | §11 实测 |
| 洲际 | 10000 km 级 | `ANALYTIC_BALLISTIC_MOVER` | 解算弹道（球面） | tests/test_analytic.py：7.2 km/s / 31.2 min / 落点=目标 |

ASM（空对面）走 `ASM_GUIDANCE`，仅单算例验证，不进这张表。两档
1000~3000 km 与 3000~10000 km 之间的空档由解析弹道连续覆盖（解算上限
地心角 170° ≈ 19000 km），只是想定演示摆不到（边界 1），测试已钉。

## 14. 战斗部：`warhead.txt`（WARHEAD 毁伤组件，v0.13.52 – v0.13.54）

§13 的解析弹道弹加挂一枚战斗部：弹飞它的球面弹道，战斗部独立巡回弹目
距离——进触发圈（`fuse_radius`）后按瑞利脱靶量骨架掷**一次**骰（不是逐帧
掷：P=0.2 的弹每帧掷一次，十帧滚成 1−0.8¹⁰≈89%，节拍越密越必然命中），
中了按 `标称伤害 = damage_scale × 类型因子 × (质量/参考质量)^指数` 算标称
**点数**、按**目标**的 `armor` 折扣后扣在目标血条（`max_health`）上——
伤害与血量同量纲：血条 1000 挨 200 剩 800，对外显示 80% 血量；
`engagement_resolved` 总线广播结果。判定时刻由 CPA 穿越插补的一次性事件
钉住——巡回只是采样（3 km/s 的弹一个节拍跑 6 km，触发圈可能整段落在
两拍之间），外推只定"何时判"，"判得对不对"由事件时刻的真实几何兜底。

想定：SRBM_1（600 km / 高点 200 km，全程约 421 s）打 TGT_WH，全默认
参数链 = 0.5 × 1.0 × (250/250)¹ = 0.5 点 → 目标完好度 **1.0 → 0.5**，
战斗部 verdict 含"命中"。命中 = 起爆：宿主弹体**注销**——从注册表与
store 所有索引摘干净，什么都没有了（不是残骸；死亡实体停推见 mover
基类），失的弹不起爆、继续飞。
要给目标"挣扎"能力，在它的 platform_type 写 `max_accel`
（越大越难被打中，概率模型的目标过载项）。

想定写法要领写在 `warhead.txt` 文件头；参数表、概率骨架与 Phase 2 标定
协议见设计文档 §5.17。tests/test_warhead.py 34 例（含本想定端到端）。

