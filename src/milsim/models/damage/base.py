"""战斗部组件：命中概率裁决 + 毁伤结算（§5.17）。

它在流水线里的位置
------------------
弹的制导件（§5.8）只管飞行与引信起爆——"到目标的距离 ≤ ``lethal_radius``
就终止"，终止后**什么都不发生**：谁的血都没动。这是一个刻意留了三个
版本的口子（``guidance/base.py`` 的 M4b 注释）：精确的末端弹目博弈
（导引闭环、目标机动、脱靶量）算起来太贵，而战场上的问题从来是
"这一发打上去，目标还剩几成"。

本件就是这个口子的第一个 occupant。它把"命中"从**几何事件**改写成
**概率事件**：弹进入触发圈（``fuse_radius``，默认 3 km——按 3 km/s 的
末速那是 1 s 的导引时间，正好是"目标还来得及挣扎一下"的尺度）之后，
按瑞利脱靶量骨架掷一次骰子，中了就把毁伤扣到目标头上。

为什么是"进入触发圈一次性判定"而不是逐帧掷骰
--------------------------------------------
逐帧掷骰在数学上是错的：一枚 P=0.2 的弹每帧掷一次，十帧之后命中概率
是 1−0.8¹⁰ ≈ 89%——**节拍越密越必然命中**，推演结果随帧率漂移。
一次性判定天然与节拍无关：概率只依赖判定那一刻的几何（距离、接近
速率、双方机动能力），帧率改了，判定的时刻几乎不变，概率就不变。

为什么高速弹需要"插补事件"而不是只靠巡回节拍
--------------------------------------------
巡回节拍（``check_interval``）是**采样**，采样有盲区：3 km/s 的弹一个
2 s 的节拍跑 6 km，触发圈 3 km 完全可能整段落在两次采样之间——上一拍
在圈外 4 km，这一拍已经掠过目标 2 km，逐帧查"在不在圈内"就会**漏判**，
而漏判的表现是"这发弹无声无息地飞过去了"，与"打歪了"长得一模一样。

解法是用户拍板的口径：**每次巡回都外推"若保持当前相对速度，何时穿进
触发圈"，预测的时刻近（小于一个节拍间隔）就当场插一个一次性事件**。
事件时刻到了重读真实距离——进圈了就判定，没进（外推误差）就等下一拍
重算。判定本身用的是**那一刻的真实几何**，所以外推只影响"什么时候判"，
不影响"判得对不对"。

裁决通道：提交走视图，结算代行 M4b
----------------------------------
扣血这件事的分工是三层（``store.py`` 裁决域的注释）：

1. **提交意图**走 :meth:`EngagementView.request_effect`——"我打中了、
   预期毁伤 X"。这是视图上的正式口子，将来独立的裁决器照单结算；
2. **结算**调 :meth:`EntityStore.apply_damage`——那是裁决器的专属域，
   视图上没有。本件在 M4b 裁决器独立成系统之前**代行这个职责**：目标
   的防护（``armor``）与血量上限（``max_health``）是**目标**的平台属性，
   在结算时才读（打击方只提交点数口径的标称伤害，不需要知道打的是
   坦克还是飞机）——这正是 request_effect 设计意图里"抗性由目标自己算"
   的分工；
3. **结果**发 :class:`EventBus.engagement_resolved`——视图/回放订阅它，
   模型层不反向持有任何人。

弹的消亡与裁决绑定
------------------
命中 = 战斗部起爆。起爆了的弹**注销**——结算与广播一完成，宿主从
注册表与 store 的所有索引里摘掉，什么都没有了：位置查不到、雷达看不
见、机动件周期回调里 ``my_alive()`` 为 False 停推。失的弹没有起爆，
保持飞行直到引信/弹道自然终止。（被摧毁的**目标**完好度归 0、残骸
留注册表——那是 store 的既有语义。）

毁伤值的口径：伤害是点数，扣在血条上
------------------------------------
伤害是**点数**（与目标 ``max_health`` 同量纲），不是比例::

    标称伤害 = damage_scale × 类型因子 × (战斗部质量 / 参考质量)^指数
    实际伤害 = 标称伤害 × armor            （armor = 对这一发的折扣，0 = 免疫）
    剩余血量 = max_health − 实际伤害       （封底 0）
    对外显示 = 剩余血量 / max_health       （store 完好度 = 血量百分比）

用户口径的例子：血条 1000 的平台挨了一发 200 的伤害，剩 800，
``damage_of`` 变 0.8——对外显示 **80% 血量**。``armor`` 缺省 1.0 =
无防护（伤害全额生效）；``max_health`` 缺省 1.0 = 血条与完好度同
量纲，伤害直接写比例值（0.5 = 打掉一半）就能用；``mass_reference``
缺省与默认装药相同 = 质量效应默认不修正（写 200 扣 200）。

概率模型：骨架已定，系数留给大样本
----------------------------------
``hit_probability`` 是瑞利脱靶量骨架：脱靶量散布 σ 越小、杀伤半径越大，
命中概率越高。σ 拆成两块——目标在剩余导引时间里能横移出去的距离
（弹修正不掉的份额与弹/目标机动能力之比有关）加一个固有散布地板。
**系数（``maneuver_factor``、``sigma_floor``）目前是给物理合理默认值的
占位**，Phase 2 用 AFSIM 大样本统计标定（§5.17 的标定协议）。骨架的
单调性现在就是对的：目标机动越强概率越低、弹越快（剩余时间越短）
概率越高、弹机动能力越强概率越高。
"""

from __future__ import annotations

from math import exp, sqrt
from typing import Any

from ...engine import PRIORITY_ENGAGE, EventResult
from ...errors import ConfigurationError
from ...services.params import Params
from ...services.random import bernoulli
from ...services.type_registry import register_component
from ..component import Component

#: 战斗部类型。Phase 1 只影响类型因子；Phase 2 里各类型会有自己的
#: 毁伤分布（破片的空间衰减、侵彻的深度效应……）。
WARHEAD_KINDS = ("BLAST", "FRAG", "PEN", "HE")

#: 类型因子（标称毁伤的乘数）。Phase 1 的占位表：爆破/高爆按全额计，
#: 破片与侵彻对"整平台完好度"的换算效率低一截。Phase 2 标定后替换。
WARHEAD_FACTORS = {"BLAST": 1.0, "FRAG": 0.6, "PEN": 0.8, "HE": 1.0}

#: 接近速率的下限（m/s）。``t_go = d / V_c`` 的分母保护：V_c ≤ 0 表示
#: 没在接近（已掠过 / 相对静止），按 1 m/s 算出的巨大 t_go 会把散布
#: 撑大、把概率压到 0——这正是"打不着"的正确读数。
CLOSING_EPS = 1.0


def hit_probability(
    distance_m: float,
    closing_mps: float | None,
    target_accel: float,
    missile_accel: float,
    *,
    kill_radius: float,
    maneuver_factor: float,
    sigma_floor: float,
) -> float:
    """瑞利脱靶量骨架下的命中概率（§5.17 的 Phase 1 模型）。

    ``closing_mps`` 传 ``None`` 表示"没有差分历史、接近速率不可知"——
    此时按固有散布地板判（目标机动观测不到，偏向命中）。

    独立成模块级纯函数：它只吃几何与能力参数、不碰任何仿真状态，
    帧率不变性与系数标定（Phase 2）都在这一个函数上做。
    """
    if sigma_floor <= 0.0 or kill_radius <= 0.0:
        return 1.0 if kill_radius > 0.0 else 0.0
    if closing_mps is None:
        sigma = sigma_floor
    else:
        t_go = distance_m / max(closing_mps, CLOSING_EPS)
        denom = missile_accel + target_accel
        # 弹机动能力无穷大 → 修正不掉的份额趋于 0；弹完全不会机动 →
        # 目标横移全额计入。target_accel = 0（不机动）时份额为 0，
        # σ 退化为地板——不动的目标必中。
        share = target_accel / denom if denom > 0.0 else 1.0
        sigma = maneuver_factor * target_accel * t_go * t_go * share + sigma_floor
    if sigma <= 0.0:
        return 1.0
    p = 1.0 - exp(-(kill_radius * kill_radius) / (2.0 * sigma * sigma))
    return min(1.0, max(0.0, p))


@register_component("WARHEAD")
class Warhead(Component):
    """战斗部：进入触发圈后一次性判定命中并结算毁伤。

    挂在**弹**上（槽 ``damage``）。一弹一件；目标用 ``target_name``
    声明——与制导件各查各的表，互不依赖（制导件被摘掉时本件照常
    工作，反之亦然）。
    """

    SLOT_HINT = "damage"

    PARAMS = {
        #: 目标平台名。**空 = 装配期报错**：没有目标的战斗部什么也裁决
        #: 不了，静默挂着只会让人以为"装了毁伤"。
        "target_name": Params.string(""),
        #: 触发圈半径（米）。弹与目标的距离小于它就开始裁决流程。
        #: 默认 3 km 是"3 km/s 末速下约 1 s 导引时间"的尺度——太小会把
        #: 概率裁决退化为必中，太大则目标还有整段航路可以跑出圈。
        "fuse_radius": Params.distance(3000.0, minimum=0.0),
        #: 杀伤半径（米）——瑞利模型里的 R：最终脱靶量小于它判"命中"。
        #: 与制导件的 ``lethal_radius``（引信起爆距离）是两个数：那个管
        #: "弹在哪儿停"，这个管"这一发算不算打中"。
        "kill_radius": Params.distance(30.0, minimum=0.0),
        #: 战斗部类型（见 :data:`WARHEAD_KINDS`）。
        "warhead_type": Params.string("HE", choices=WARHEAD_KINDS),
        #: 战斗部质量（kg）。
        "warhead_mass": Params.mass(250.0, minimum=0.0),
        #: 参考质量（kg）：``damage_scale`` 标定的那档装药。质量效应按
        #: ``(质量/参考质量)^指数`` 折算。默认 250 kg 与默认装药相同——
        #: **默认不修正**（写 200 扣 200）；想定只改 ``warhead_mass``
        #: 时伤害按质量比缩放。
        "mass_reference": Params.mass(250.0, minimum=1e-06),
        #: 标称伤害（**点数**，与目标的 max_health 同量纲）：参考质量、
        #: 类型因子 1.0 时这一发造成的伤害。写 200 就是 200 点——结算时
        #: 从目标血条里扣（先乘 armor 打折）。默认 0.5 点，配默认血条
        #: 1.0 = 一发扣掉一半血条。
        "damage_scale": Params.number(0.5, minimum=0.0),
        #: 质量效应指数 α。0 = 毁伤与质量无关；1 = 线性。AFSIM 侧的
        #: 经验区间在 0.7~1.3，Phase 2 标定。
        "damage_exponent": Params.number(1.0, minimum=0.0),
        #: 弹的末端机动能力（m/s²）——概率模型里"修正掉目标横移"的那
        #: 一方。它是**弹的属性**，所以放在本件参数里，而不是去翻制导
        #: 件或弹体的参数表（组件之间不互相读参数）。
        "missile_accel": Params.number(150.0, minimum=0.0),
        #: 目标机动散布系数（Phase 2 标定，见模块头）。
        "maneuver_factor": Params.number(0.35, minimum=0.0),
        #: 固有散布地板（米）：导引噪声、风、弹体离散——与目标机动无关
        #: 的那部分脱靶量尺度。默认 3 m，对 30 m 杀伤半径几乎是必中。
        "sigma_floor": Params.distance(3.0, minimum=0.0),
        #: 巡回周期（微秒）。它只是**采样**节拍，判定时刻的精度由插补
        #: 事件保证（见模块头），所以默认 1 s 足够。
        "check_interval": Params.duration(1_000_000, minimum=1),
        #: 本平台机动件的**槽名**——弹停飞检测读它的 ``terminated()``。
        #: ★ 不能用"位置连续几拍没变"来判停：机动件的置位是**一帧一次**
        #: 的（解析弹道弹 5 s 一帧），巡回采样在帧间读到的当然是旧位置
        #: ——拿它当"停飞"会在弹正常飞行途中误放弃。终止状态是机动件
        #: 自己的一手信息，读它才不会把"帧间静止"错当"任务结束"。
        #: 槽里没有部件（非弹平台装了战斗部）时为 ``None``：停飞检测
        #: 关闭，巡回继续到进圈或目标消失为止。
        "mover_slot": Params.string("mover"),
    }

    __slots__ = (
        "_view",
        "_store",
        "_bus",
        "_registry",
        "_rng",
        "_mover",
        "_target_id",
        "_fuse",
        "_kill",
        "_missile_accel",
        "_maneuver_factor",
        "_sigma_floor",
        "_interval_s",
        "_judged",
        "_dead",
        "_verdict",
        "_pending",
        "_prev_my",
        "_prev_tgt",
        "_prev_t",
        "_last_closing",
    )

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self._view = mount.engagement_view()
        # 裁决器的专属域：apply_damage / clear_effects 只在 _adjudicate 里调。
        # M4b 裁决器独立成系统后，这里换成对它的提交（见模块头）。
        self._store = mount.store
        self._bus = mount.bus
        self._registry = mount.registry
        self._rng = mount.streams().engage

        name = str(self.spec["target_name"]).strip()
        if not name:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 战斗部没有目标——"
                "写 target_name。没有目标的战斗部什么也裁决不了，"
                "静默挂着会让人误以为已经装了毁伤"
            )
        found = mount.registry.by_name(name)
        if found is None:
            raise ConfigurationError(
                f"{self.type_name or type(self).__name__}: 目标 {name!r} 不在"
                "这份想定里——检查 platform 名字是否写错"
            )
        self._target_id = found.entity_id

        self._fuse = float(self.spec["fuse_radius"])
        self._kill = float(self.spec["kill_radius"])
        self._missile_accel = float(self.spec["missile_accel"])
        self._maneuver_factor = float(self.spec["maneuver_factor"])
        self._sigma_floor = float(self.spec["sigma_floor"])
        self._interval_s = float(self.spec["check_interval"]) / 1_000_000.0

        self._judged = False
        self._dead = False
        self._verdict = ""
        self._pending = False
        self._prev_my: tuple[float, float, float] | None = None
        self._prev_tgt: tuple[float, float, float] | None = None
        self._prev_t: int | None = None
        self._last_closing: float | None = None

        # 弹停飞检测的依据：同平台机动件的终止状态。terminated() 与
        # arrived() 两个名字都认（机动件族里两种叫法都在用），都没有就
        # 不做这项检测——判停错误的代价是"这发弹静默地不再裁决"。
        self._mover: Any = None
        slot = str(self.spec["mover_slot"]).strip()
        if mount.entity is not None and slot:
            mover = mount.entity.component(slot)
            if mover is not None:
                self._mover = (
                    getattr(mover, "terminated", None)
                    or getattr(mover, "arrived", None)
                )

        mount.every(self.spec["check_interval"], self._sweep, PRIORITY_ENGAGE)

    # -- 巡回：采样 + 外推插补 --------------------------------------------

    def _sweep(self, engine: Any, event: Any = None) -> EventResult:
        """每 ``check_interval`` 一次：差分速度、查圈、按需插补。"""
        if self._judged or self._dead:
            return EventResult.RESCHEDULE
        now = int(engine.now)
        my = self._view.my_position()
        tgt = self._view.position_of(self._target_id)
        if my is None:
            return self._give_up("弹体已不存在")
        if tgt is None or not self._view.is_alive(self._target_id):
            return self._give_up("目标已不存在或已被摧毁")

        v_my = v_tgt = None
        if self._prev_t is not None and self._prev_my is not None and self._prev_tgt is not None:
            dt = (now - self._prev_t) / 1e6
            if dt > 0.0:
                v_my = _velocity_of(my, self._prev_my, dt)
                v_tgt = _velocity_of(tgt, self._prev_tgt, dt)
                r = (tgt[0] - my[0], tgt[1] - my[1], tgt[2] - my[2])
                rel = _sub(v_tgt, v_my)
                self._last_closing = _closing_rate(r, rel)
        self._prev_my, self._prev_tgt, self._prev_t = my, tgt, now

        d = _distance(my, tgt)
        if d <= self._fuse:
            self._judge(engine, d)
            return EventResult.RESCHEDULE

        # 弹停飞检测：机动件自己报告终止（起爆 / 坠地 / 完成）而弹又没进
        # 圈——这发弹永远不会有裁决机会了，放弃并留下结论。判定优先：
        # 落在圈内的终止弹（落地=目标点的解析弹）先判再说。
        if self._mover is not None and self._mover():
            return self._give_up("弹已终止且未进入触发圈")

        # 穿越外推：保持当前相对速度，何时穿进触发圈。预测时刻近于一个
        # 节拍间隔就插一次性事件（回调里重读真实距离再定）。已有待触发
        # 的插补事件时不重复插——事件触发或下一拍自然重算。
        if v_my is not None and v_tgt is not None and not self._pending:
            t_cross = _crossing_time(_sub(tgt, my), _sub(v_tgt, v_my), self._fuse)
            if t_cross is not None and t_cross <= self._interval_s:
                self._pending = True
                engine.schedule(max(1, int(t_cross * 1e6)), self._on_crossing, PRIORITY_ENGAGE)
        return EventResult.RESCHEDULE

    def _on_crossing(self, engine: Any) -> None:
        """插补事件：预测的穿圈时刻到了。判定用**此刻的真实几何**。"""
        self._pending = False
        if self._judged or self._dead:
            return
        my = self._view.my_position()
        tgt = self._view.position_of(self._target_id)
        if my is None or tgt is None or not self._view.is_alive(self._target_id):
            return
        d = _distance(my, tgt)
        if d <= self._fuse:
            self._judge(engine, d)
        # 没进圈（外推偏早）：什么都不做，下一拍重算。不在这里反复
        # 插补——外推误差由真实几何兜底，多插只会浪费事件。

    # -- 判定与结算 --------------------------------------------------------

    def _judge(self, engine: Any, distance_m: float) -> None:
        """一次性判定：掷骰 → 提交意图 → 结算 → 广播。"""
        self._judged = True
        a_t = float(self._target_param(self._target_id, "max_accel", 0.0))
        p = hit_probability(
            distance_m,
            self._last_closing,
            a_t,
            self._missile_accel,
            kill_radius=self._kill,
            maneuver_factor=self._maneuver_factor,
            sigma_floor=self._sigma_floor,
        )
        hit = bernoulli(self._rng, p)
        fraction = 0.0
        destroyed = False
        if hit:
            raw = self.raw_damage()
            effect = self._view.request_effect(
                self._target_id, "warhead", raw, at=int(engine.now),
                note=f"{self.type_name or 'WARHEAD'} P={p:.3f} d={distance_m:.0f}m",
            )
            fraction = self._adjudicate(effect)
            destroyed = not self._view.is_alive(self._target_id)
        self._verdict = (
            f"命中 P={p:.3f} 毁伤={fraction:.4f} 摧毁={destroyed}"
            if hit else f"失的 P={p:.3f} d={distance_m:.0f}m"
        )
        if self._bus is not None:
            # engagement_resolved 协议：(攻击方, 目标, 命中?, 扣减比例, 摧毁?)
            self._bus.engagement_resolved.emit(
                self.entity_id, self._target_id, hit, fraction, destroyed
            )
        if hit:
            # 战斗部随裁决消亡：命中 = 起爆。弹体**注销**——从注册表与
            # store 的所有索引里摘干净，什么都没有了：位置查不到、雷达
            # 看不见、机动件周期回调里 my_alive() 为 False 停推。失的弹
            # 没有起爆，保持飞行直到引信/弹道自然终止。
            self._store.remove(self.entity_id)
            self._registry.unregister(self.entity_id)

    def _adjudicate(self, effect: Any) -> float:
        """结算一条毁伤意图（M4b 代行）：防护打折、从血条里扣、清队列。

        伤害是**点数**（与 max_health 同量纲）：实际伤害 = 标称 × armor
        （armor 是对这一发的折扣，0.9 = 只起九成作用，0 = 免疫）；
        剩余血量 = max_health − 实际伤害（封底 0）。store 的完好度域吃
        "剩余/上限"——那正好是对外显示的**血量百分比**：血条 1000 挨了
        200，``damage_of`` 变 0.8 = 显示 80% 血量。返回实际扣掉的完好度
        比例（广播协议里的那个数）。
        """
        armor = float(self._target_param(effect.target_id, "armor", 1.0))
        max_health = float(self._target_param(effect.target_id, "max_health", 1.0))
        fraction = effect.magnitude * armor / max(max_health, 1e-9)
        self._store.apply_damage(effect.target_id, fraction)
        self._store.clear_effects()
        return fraction

    def raw_damage(self) -> float:
        """标称伤害（点数，与目标 max_health 同量纲）：基数 × 类型因子 × 质量效应。"""
        factor = WARHEAD_FACTORS[str(self.spec["warhead_type"])]
        mass = float(self.spec["warhead_mass"])
        reference = max(float(self.spec["mass_reference"]), 1e-9)
        exponent = float(self.spec["damage_exponent"])
        scaled = (mass / reference) ** exponent if mass > 0.0 else 0.0
        return float(self.spec["damage_scale"]) * factor * scaled

    def _target_param(self, entity_id: int, name: str, default: Any) -> Any:
        """读目标的平台属性。与 ``mount.target_platform_param`` 同一套
        逻辑——运行期不再回头找 mount，所以在这里照抄那三条线：只读
        属性、读不到给默认、不开位置的口子。"""
        entity = self._registry.get(entity_id)
        params = getattr(entity, "platform_params", None)
        if not params:
            return default
        return params.get(name, default)

    def _give_up(self, reason: str) -> EventResult:
        self._dead = True
        self._verdict = f"放弃：{reason}"
        return EventResult.RESCHEDULE

    # -- 诊断 --------------------------------------------------------------

    def judged(self) -> bool:
        return self._judged

    def abandoned(self) -> bool:
        return self._dead

    def verdict(self) -> str:
        """判定结论（未判定时空串）。测试与回放用它对拍。"""
        return self._verdict

    def describe(self) -> str:
        base = super().describe()
        if not hasattr(self, "_target_id"):
            return base
        return f"{base} → eid={self._target_id}"


# ---------------------------------------------------------------------------
# 几何小工具（模块级：不依赖组件状态，测试可以直接对拍）
# ---------------------------------------------------------------------------

def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    d = _sub(a, b)
    return sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])


def _velocity_of(
    now: tuple[float, float, float], prev: tuple[float, float, float], dt: float
) -> tuple[float, float, float]:
    return ((now[0] - prev[0]) / dt, (now[1] - prev[1]) / dt, (now[2] - prev[2]) / dt)


def _closing_rate(
    r: tuple[float, float, float], v_rel: tuple[float, float, float]
) -> float:
    """接近速率：正 = 在靠近。``V_c = −d|r|/dt = −(r·v)/|r|``。

    与制导件 ``_geometry`` 的 closing 同一口径（那边的注释钉过符号）。
    """
    norm = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    if norm <= 1e-9:
        return 0.0
    return -(r[0] * v_rel[0] + r[1] * v_rel[1] + r[2] * v_rel[2]) / norm


def _crossing_time(
    r: tuple[float, float, float],
    v_rel: tuple[float, float, float],
    radius: float,
) -> float | None:
    """保持相对速度不变，何时 ``|r + v·t| = radius``。返回最小正根。

    解 ``|v|²t² + 2(r·v)t + (|r|²−R²) = 0``：无实根（轨迹不达圈）或
    只在过去穿圈时返回 ``None``——调用方按"这一拍不插补"处理。
    """
    a = v_rel[0] * v_rel[0] + v_rel[1] * v_rel[1] + v_rel[2] * v_rel[2]
    if a <= 1e-12:
        return None
    b = 2.0 * (r[0] * v_rel[0] + r[1] * v_rel[1] + r[2] * v_rel[2])
    c = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = sqrt(disc)
    for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
        if t > 0.0:
            return t
    return None


__all__ = ["WARHEAD_FACTORS", "WARHEAD_KINDS", "Warhead", "hit_probability"]
