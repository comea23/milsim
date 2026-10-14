"""发射组件（§5.18，v0.13.58）——AFSIM ``WSF_WEAPON`` 的对应物。

职责
----
想定里挂 ``component weapon WEAPON`` 的平台持有弹药与发射能力；``fire``
指令生成**弹实体**（运行期 spawn），把目标交给弹的制导管，然后剩下的
事全归 §5.8 的弹体+制导件。

与 AFSIM 的对应
---------------
====================  ==========================================
AFSIM                 milsim
====================  ==========================================
``WSF_WEAPON``        :class:`Weapon`（弹药、发射、包线检查）
``WSF_LAUNCH_COMPUTER`` **简版**：出生位=发射平台当前位置，初速/姿态
                      用弹型号自己的 ``launch_speed`` / ``launch_fpa``；
                      按目标距离做**包线检查** + **三维射界**（v0.13.60：
                      水平扇区/高低扇区/结构死区，全部体轴）。抬升角
                      解算留作增强。
``WeaponFired`` evt   ``bus.weapon_fired`` 信号
====================  ==========================================

多挂载
------
一个实体可以挂多个发射组件（不同槽名），``weapon_class`` 区分用途：
``SAM``（地对空）/``SSM``（地或海对地或海）/``ASM``（空对地或海）/
``AAM``（空对空）。``fire`` 时按**目标的阵营与距离**选弹是调用方的事
（决策层/人在回路），本组件只验"这一发打得打不得"：目标在包线内、
还有弹。

**射程不是写死的**：``range_min/range_max`` 是**这一档弹的实测包线**
（同一张阶段表换燃料分档 = 不同弹型号，各自实测——§5.8 的 25~85 km
就是 SSM_SRBM 那一档的实测边界）。``range_max 0`` = 不检查。
"""

from __future__ import annotations

from math import atan2, degrees, hypot
from typing import Any

from ...errors import ConfigurationError
from ...services.params import Params
from ...services.type_registry import register_component
from ..component import Component
from ..guidance.base import wrap180

#: weapon_class 的合法取值（想定里写错装配期就报，带候选清单）。
WEAPON_CLASSES: tuple[str, ...] = ("SAM", "SSM", "ASM", "AAM")


def _in_arc(rel_deg: float, lo: float, hi: float) -> bool:
    """方位 ``rel_deg``（已折到 [-180, 180]）是否落在体轴扇区 [lo, hi] 内。

    ``lo > hi`` = 跨 ±180° 的环形扇区（如 ``170..-170`` 是正后方 20° 宽）。
    """
    if lo <= hi:
        return lo <= rel_deg <= hi
    return rel_deg >= lo or rel_deg <= hi


@register_component("WEAPON")
class Weapon(Component):
    """发射组件：弹药、包线检查、生成弹实体。"""

    SLOT_HINT = "weapon"

    #: 运行期指令（§5.18）。
    COMMANDS = {
        "fire": "发射：args = {'target': 实体名或编号}",
        "reload": "装填：args = {'rounds': 弹数}，装满到弹匣上限为止",
    }

    #: 认领的任务类型（§5.19）：attack 任务 = 按任务参数齐射并盯到弹终结。
    #: 多挂载时多个发射槽都认领 ``attack``——任务管理器按挂载顺序问，
    #: 类别不符/没弹的槽拒绝，下一个槽接手（§5.19 认领链）。
    TASKS = ("attack",)

    PARAMS = {
        #: 用途类别。``fire`` 时不看它（选弹是调用方的事），它给的是
        #: **想定作者与决策层的标签**——"这个槽是打谁的"。
        "weapon_class": Params.string("SSM"),
        #: 弹的**平台类型名**（想定里的 ``platform_type``）。装配期查不到
        #: 这个类型直接报错——发射时才发现等于白挂了一门炮。
        "weapon_type": Params.string(""),
        #: 打击包线（米）——**实测值**，不是愿望。目标在包线外拒发；
        #: ``range_max 0`` = 不检查（演示弹、直射弹）。
        "range_min": Params.distance(0.0, minimum=0.0),
        "range_max": Params.distance(0.0, minimum=0.0),
        #: 弹匣容量（发）。也是 ``reload`` 的上限。
        "stock": Params.integer(4, minimum=0),
        #: 水平射界（度，**体轴**方位，v0.13.60）：目标方位换算到发射平台
        #: 体轴后必须落在 [az_min, az_max] 内——射界随平台朝向转。
        #: ``az_min > az_max`` = 跨 ±180° 的环形扇区。默认 ±180 = 全向。
        "az_min": Params.angle(-180.0),
        "az_max": Params.angle(180.0),
        #: 高低射界（度）：目标仰角 = atan2(高度差, 水平距)。默认 ±90 = 不限。
        #: 仰角相对水平面（项目没有俯仰姿态模型，平台永远水平——§5.18 记账）。
        "el_min": Params.angle(-90.0),
        "el_max": Params.angle(90.0),
        #: 结构死区（体轴方位区间列表，每项 ``"lo..hi"`` 度）：平台自身
        #: 结构（舰桥、肩部）遮挡的方向，落在里面拒发。与水平射界独立
        #: 叠加——先判射界，再判死区。
        "dead_zone": Params.strings(default=()),
    }

    __slots__ = ("_mount", "_stock", "_capacity", "_jobs", "_dead_zones")

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self._mount = mount
        self._capacity = int(self.spec["stock"])
        self._stock = self._capacity            # 开局满弹
        #: 在身的 attack 任务：task_id → {'target': id, 'missiles': [弹id]}。
        #: 多个 attack 任务可并发（每项各盯各的弹），弹全终结即收工。
        self._jobs: dict[int, dict[str, Any]] = {}
        wtype = str(self.spec["weapon_type"])
        if not wtype:
            raise ConfigurationError(
                f"{self.describe()}：weapon_type 是空的——不写弹型的发射组件"
                "等于白挂，想定里补上（弹的平台类型名）"
            )
        if str(self.spec["weapon_class"]) not in WEAPON_CLASSES:
            raise ConfigurationError(
                f"{self.describe()}：weapon_class {self.spec['weapon_class']!r} "
                f"不认识（可用：{'、'.join(WEAPON_CLASSES)}）"
            )
        el_lo = float(self.spec["el_min"])
        el_hi = float(self.spec["el_max"])
        if el_lo > el_hi:
            raise ConfigurationError(
                f"{self.describe()}：高低射界写反了（el_min {el_lo} > "
                f"el_max {el_hi}）。高低射界不是环形——要打天顶就写 "
                "el_min -89、el_max 89"
            )
        #: 解析好的结构死区 [(lo, hi), ...]（度，体轴）。写法错误在
        #: **装配期**报——发射时才发现等于白挂了遮挡。
        self._dead_zones: list[tuple[float, float]] = []
        for raw in self.spec["dead_zone"]:
            text = str(raw).strip()
            if ".." not in text:
                raise ConfigurationError(
                    f"{self.describe()}：死区 {text!r} 看不懂——每项写 "
                    "'lo..hi'（度，体轴方位），如 '170..-170' 是正后方"
                )
            lo_s, hi_s = text.split("..", 1)
            try:
                self._dead_zones.append((float(lo_s), float(hi_s)))
            except ValueError as exc:
                raise ConfigurationError(
                    f"{self.describe()}：死区 {text!r} 的端点不是数——{exc}"
                ) from exc

    # -- 查询 --------------------------------------------------------------

    def stock(self) -> int:
        return self._stock

    def weapon_class(self) -> str:
        return str(self.spec["weapon_class"])

    def add_stock(self, rounds: int) -> int:
        """装填 n 发，返回实际装进去的量（弹匣满了不收）。"""
        before = self._stock
        self._stock = min(self._capacity, self._stock + max(0, int(rounds)))
        return self._stock - before

    # -- 指令（§5.18） ------------------------------------------------------

    def execute(self, cmd: str, args: dict) -> tuple[bool, str]:
        ok, why = super().execute(cmd, args)
        if not ok and cmd not in self.COMMANDS:
            return ok, why
        if cmd == "reload":
            rounds = args.get("rounds", self._capacity)
            try:
                n = int(rounds)
            except (TypeError, ValueError):
                return False, "reload 的 rounds 必须是整数"
            added = self.add_stock(n)
            return True, f"装填 {added} 发（现 {self._stock}/{self._capacity}）"
        if cmd == "fire":
            return self._fire(args)
        return False, f"指令 {cmd!r} 没有实现"

    def _resolve_target(self, args: dict) -> tuple[int | None, str]:
        raw = args.get("target")
        if isinstance(raw, bool) or not isinstance(raw, (str, int)):
            return None, "fire 需要 target（目标实体名或编号）"
        if isinstance(raw, int):
            found = self._mount.registry.get(raw)
            if found is None:
                return None, f"目标实体 {raw} 不存在"
            return raw, ""
        found = self._mount.registry.by_name(str(raw))
        if found is None:
            return None, f"目标 {raw!r} 不在这份想定里"
        return found.entity_id, ""

    def _fire(self, args: dict) -> tuple[bool, str]:
        me = self._mount.entity_id

        if self._stock <= 0:
            return False, f"弹药耗尽（{self._stock}/{self._capacity}），先 reload"

        tid, why = self._resolve_target(args)
        if tid is None:
            return False, why
        if tid == me:
            return False, "不能打自己"

        mid, why = self._launch(tid)
        if mid is None:
            return False, why
        return True, f"发射（弹={mid}，目标={tid}，余弹 {self._stock}）"

    def _launch(self, tid: int) -> tuple[int | None, str]:
        """发射一发：检查链（同方/包线/库存）→ 生成弹实体 → 交接目标。

        ``fire`` 指令与 attack 任务共用这一条链。返回 ``(弹实体id, "")``
        或 ``(None, 拒发原因)``——**不抛异常**，失败是回执。
        """
        me = self._mount.entity_id
        store = self._mount.store

        if self._stock <= 0:
            return None, f"弹药耗尽（{self._stock}/{self._capacity}），先 reload"

        # ★ 敌我由想定态势判（§5.18 同 IFF 口径）：同方拒发——打自己人
        # 应该是想定写错了，响着失败比打出去强。
        my_side = self._mount.registry.side_of(me)
        target_side = self._mount.registry.side_of(tid)
        if my_side and target_side and my_side == target_side:
            return None, f"目标 {tid} 与我同属 {my_side} 方，拒发"

        # 包线检查：水平距离。位置走 store 真值——与制导件 target_point()
        # 同一条"发射计算机看得见真值"的口径（§5.8 已记账）。
        mine = store.position_of(me)
        theirs = store.position_of(tid)
        if mine is None or theirs is None:
            return None, "发射方或目标没有位置（不在任何战区里）"
        dist = hypot(theirs[0] - mine[0], theirs[1] - mine[1])
        rmin = float(self.spec["range_min"])
        rmax = float(self.spec["range_max"])
        if rmax > 0.0 and not (rmin <= dist <= rmax):
            return None, (
                f"目标距离 {dist * 0.001:.1f} km 在包线外"
                f"（{rmin * 0.001:.1f}~{rmax * 0.001:.1f} km），拒发"
            )

        # 三维射界（v0.13.60，TF 三维裁决篇）：方位与仰角都换算到**体轴**
        # 再判——射界随平台朝向转（"随姿态变化"），结构死区是平台自身
        # 的遮挡、同样按体轴。仰角相对水平面：项目没有俯仰姿态模型，
        # 平台恒水平（§5.18 记账）。位姿走 store 真值，与上面的口径同源。
        my_pose = store.pose_of(me)
        if my_pose is None:
            return None, "发射方没有位姿（不在任何战区里）"
        dx = theirs[0] - mine[0]
        dy = theirs[1] - mine[1]
        bearing = degrees(atan2(dx, -dy))       # 地理方位：0=北(+y)，顺时针
        rel = wrap180(bearing - my_pose[3])     # 体轴方位
        elev = degrees(atan2(theirs[2] - mine[2], dist))
        az_lo = float(self.spec["az_min"])
        az_hi = float(self.spec["az_max"])
        if not _in_arc(rel, az_lo, az_hi):
            return None, (
                f"目标体轴方位 {rel:.0f}° 在水平射界"
                f" [{az_lo:.0f}..{az_hi:.0f}°] 外，拒发"
            )
        el_lo = float(self.spec["el_min"])
        el_hi = float(self.spec["el_max"])
        if not (el_lo <= elev <= el_hi):
            return None, (
                f"目标仰角 {elev:.1f}° 在高低射界"
                f" [{el_lo:.1f}..{el_hi:.1f}°] 外，拒发"
            )
        for lo, hi in self._dead_zones:
            if _in_arc(rel, lo, hi):
                return None, (
                    f"目标体轴方位 {rel:.0f}° 落在结构死区"
                    f" [{lo:.0f}..{hi:.0f}°] 内（平台自遮挡），拒发"
                )

        # 简版发射计算机：出生位=发射平台当前位置，弹的姿态朝目标；
        # 初速/发射姿态由弹型号自己的 launch 参数给（§5.8）。
        spawn = self._mount.spawn
        if spawn is None:
            return None, "装配缺陷：mount 没有 spawn 回调，发不出去"
        heading = degrees(atan2(theirs[0] - mine[0], -(theirs[1] - mine[1])))
        try:
            missile_id = int(
                spawn(
                    str(self.spec["weapon_type"]),
                    side=my_side,
                    cell=self._mount.my_cell(),
                    x=mine[0], y=mine[1], z=mine[2],
                    heading_deg=heading,
                )
            )
        except Exception as exc:                    # noqa: BLE001——回执化
            return None, f"生成弹实体失败：{exc}"

        # 目标交给弹的制导管（§5.8）——弹从此自己盯它；战斗部同一条
        # 交接（v0.13.59）：运行期出生的弹没有想定 target_name，两个
        # 部件都在发射这一刻拿到目标。
        missile = self._mount.registry.get(missile_id)
        guide = getattr(missile, "component", lambda s: None)("guidance")
        assign = getattr(guide, "assign_target_id", None)
        if assign is None:
            return None, (
                f"弹 {missile_id} 的平台类型没有 guidance 槽或制导件不支持"
                "运行期指定目标"
            )
        assign(tid)
        warhead = getattr(missile, "component", lambda s: None)("damage")
        wassign = getattr(warhead, "assign_target_id", None)
        if callable(wassign):
            wassign(tid)

        self._stock -= 1
        bus = self._mount.bus
        emit = getattr(bus, "weapon_fired", None)
        if emit is not None:
            emit.emit(me, missile_id, tid, self.weapon_class())
        return missile_id, ""

    # -- 任务执行器协议（§5.19）：attack 任务 --------------------------------

    def acquire_task(self, task) -> tuple[bool, str]:
        """attack 任务：按 ``payload`` 齐射，然后盯到弹全部终结。

        - ``payload['rounds']``：发数（默认 1）；实际发出 min(发数, 余弹)。
        - ``payload['weapon_class']``：给多挂载用的类别筛选——与本槽不符
          就**拒绝**，让任务管理器去问下一个槽（§5.19 认领链）。
        - 一项任务打一个目标；一发射失败（没弹/包线外）整项拒绝。
        """
        want = str((task.payload or {}).get("weapon_class", "")).strip()
        if want and want != self.weapon_class():
            return False, f"任务要 {want} 槽，本槽是 {self.weapon_class()}"
        if task.target_id is None:
            return False, "attack 任务需要 target（目标实体）"
        rounds = max(1, int((task.payload or {}).get("rounds", 1)))
        fired: list[int] = []
        why = ""
        for _ in range(rounds):
            mid, why = self._launch(int(task.target_id))
            if mid is None:
                break
            fired.append(mid)
        if not fired:
            return False, why
        self._jobs[task.task_id] = {
            "target": int(task.target_id),
            "missiles": fired,
        }
        return True, f"齐射 {len(fired)} 发（弹 {fired}）"

    def poll_task(self, task):
        """盯弹：全部终结 ⇒ 收工。弹被注销 = 命中起爆；弹机动终止 = 失的。"""
        job = self._jobs.get(task.task_id)
        if job is None:
            return None
        registry = self._mount.registry
        # 目标先没了：打完了就是打完了，不等剩余在飞弹慢慢落。
        if registry.get(job["target"]) is None:
            self._jobs.pop(task.task_id, None)
            return ("complete", "target_destroyed")
        alive = []
        hits = 0
        for mid in job["missiles"]:
            entity = registry.get(mid)
            if entity is None:
                hits += 1          # 弹体注销只发生在命中起爆（§5.17）
                continue
            mover = getattr(entity, "component", lambda s: None)("mover")
            term = getattr(mover, "terminated", None)
            if callable(term) and term():
                continue           # 失的弹自然终止
            alive.append(mid)
        if alive:
            return None
        self._jobs.pop(task.task_id, None)
        total = len(job["missiles"])
        return ("complete", f"命中 {hits}/{total}")

    def release_task(self, task) -> None:
        """撤单 = 不再盯这批弹（AbortSalvo 语义：只影响未发射的，
        在飞的弹继续飞）。"""
        self._jobs.pop(task.task_id, None)


__all__ = ["WEAPON_CLASSES", "Weapon"]
