# 演习场景金标准 #1：Growler vs Growler（电子战压制 vs S-400 防空）

> 来源：CMO 想定 `Growler Vs. Growler, 2014.scen`（`tools/cmo_scen_decode.py` 解出）。
> 用途：Jeff 决策框架第一轮"演习场景文本→推演→对照→gap 分析"的金标准载体。

## 1. 想定背景

世界上最强的电子战飞机（EA-18G Growler）与同名的最强防空系统（SA-21 S-400 "Growler"）在演习中相遇。蓝方（BLUFOR）出动电子战编队执行防空压制/防区外干扰，红方（REDFOR）以 S-400 营 + 远程警戒雷达构成要地防空。

- 时间：2014，白昼；地域：北大西洋沿岸（新斯科舍 ↔ 圣劳伦斯湾）
- 复杂度 3 / 难度 4；双方阵营：BLUFOR vs REDFOR

## 2. 初始兵力

### 蓝方 BLUFOR（进攻方）

| 单位 | 型号 | 位置 | 状态 |
|---|---|---|---|
| Single-Unit Airfield（1×2601-3200m 跑道） | 机场 | (-62.933, 45.237) | 地面 |
| Growler #1~#5（×5） | EA-18G 电子战攻击机 (DBID 343) | 机场驻场（起飞后机动） | 待命 |

### 红方 REDFOR（防御方）

| 单位 | 型号 | 位置 | 状态 |
|---|---|---|---|
| SAM Bn (SA-21a/b [S-400 Triumph]) | S-400 防空营 (DBID 2029) | (-56.693, 48.460) | 静默待命 |
| Radar (Bar Lock A [P-37]) | P-37 远程警戒雷达 (DBID 188) | (-56.837, 48.554) | 巡逻任务"Radar Activates"控制开关机 |

红方防御纵深：S-400 与警戒雷达相距约 18km；两防区中心与蓝方机场相距约 **550km**（蓝方防区外进入）。

## 3. 任务与交战规则

- **蓝方任务**：编队 5 机从机场起飞，向东进入红方防区；利用电子干扰压制红方探测与火控，伺机摧毁红方防空节点。
- **红方任务**：`Patrol "Radar Activates"`（巡逻/警戒任务，含 ATO 计划）——P-37 按节奏控制开关机，S-400 依据雷达引导接战；单元接战授权遵循 CMO 默认 ROE（ weapons free on detected hostile）。
- **电磁对抗核心**：EA-18G 的 AN/ALQ-99 干扰 vs S-400/P-37 探测链——SNR 压制、烧穿距离、单脉冲/捷变抗干扰是本场推演的主要变量。

## 4. 胜利条件（事件触发器，硬裁决）

| 触发 | 动作 |
|---|---|
| EventTrigger_UnitDestroyed（红方单位被毁 ×3 条） | EventAction_Points 计分 |
| EventTrigger_UnitDamaged | EventAction_Points 计分 |
| EventTrigger_Points（累计分数门槛） | EventAction_ChangeMissionStatus / **EventAction_EndScenario 结束推演** |

计分口径：击毁/击伤红方单位得分；推演在分数门槛触发时结束。

## 5. 金标准验收要点（对照 milsim 推演结果）

1. **探测链**：P-37 在未受干扰时对 EA-18G 的探测距离量级；受 ALQ-99 压制后 SNR 下降→探测/跟踪中断或精度退化（烧穿距离）。
2. **接战链**：S-400 在雷达受干扰条件下的发射决策、导弹进入末段后自导引头与干扰的对抗。
3. **决策链（Jeff 决策点对应）**：
   - 红方高层：`em`（雷达开关机/静默）×`engage_auth`（何时开火）；蓝方高层：`ingress` 方向/高度剖面、`em` 干扰时机
   - 低层：拦截/规避机动、末段机动
4. **结束条件**：红方 SAM 营被毁（蓝方胜）/ 蓝方编队损失过半（红方胜）/ 时间耗尽按分数判定。

## 6. 映射到 milsim 的缺口（预判，待第一轮推演后修正）

- EA-18G 干扰机、S-400 导弹/雷达、P-37 均需在 milsim 侧配等效装备（性能数值按 DB3K_491.db3 量级折算）
- 雷达开关机控制（`Radar Activates` 任务逻辑）→ 需要红方决策层指令或脚本
- 计分/结束事件 → 演习裁判层（引擎侧已有 engagement_resolved 事件可挂钩）
