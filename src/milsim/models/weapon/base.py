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
                      按目标距离做**包线检查**。抬升角解算留作增强。
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

#: weapon_class 的合法取值（想定里写错装配期就报，带候选清单）。
WEAPON_CLASSES: tuple[str, ...] = ("SAM", "SSM", "ASM", "AAM")


@register_component("WEAPON")
class Weapon(Component):
    """发射组件：弹药、包线检查、生成弹实体。"""

    SLOT_HINT = "weapon"

    #: 运行期指令（§5.18）。
    COMMANDS = {
        "fire": "发射：args = {'target': 实体名或编号}",
        "reload": "装填：args = {'rounds': 弹数}，装满到弹匣上限为止",
    }

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
    }

    __slots__ = ("_mount", "_stock", "_capacity")

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self._mount = mount
        self._capacity = int(self.spec["stock"])
        self._stock = self._capacity            # 开局满弹
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
        store = self._mount.store

        if self._stock <= 0:
            return False, f"弹药耗尽（{self._stock}/{self._capacity}），先 reload"

        tid, why = self._resolve_target(args)
        if tid is None:
            return False, why
        if tid == me:
            return False, "不能打自己"

        # ★ 敌我由想定态势判（§5.18 同 IFF 口径）：同方拒发——打自己人
        # 应该是想定写错了，响着失败比打出去强。
        my_side = self._mount.registry.side_of(me)
        target_side = self._mount.registry.side_of(tid)
        if my_side and target_side and my_side == target_side:
            return False, f"目标 {tid} 与我同属 {my_side} 方，拒发"

        # 包线检查：水平距离。位置走 store 真值——与制导件 target_point()
        # 同一条"发射计算机看得见真值"的口径（§5.8 已记账）。
        mine = store.position_of(me)
        theirs = store.position_of(tid)
        if mine is None or theirs is None:
            return False, "发射方或目标没有位置（不在任何战区里）"
        dist = hypot(theirs[0] - mine[0], theirs[1] - mine[1])
        rmin = float(self.spec["range_min"])
        rmax = float(self.spec["range_max"])
        if rmax > 0.0 and not (rmin <= dist <= rmax):
            return False, (
                f"目标距离 {dist * 0.001:.1f} km 在包线外"
                f"（{rmin * 0.001:.1f}~{rmax * 0.001:.1f} km），拒发"
            )

        # 简版发射计算机：出生位=发射平台当前位置，弹的姿态朝目标；
        # 初速/发射姿态由弹型号自己的 launch 参数给（§5.8）。
        spawn = self._mount.spawn
        if spawn is None:
            return False, "装配缺陷：mount 没有 spawn 回调，发不出去"
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
            return False, f"生成弹实体失败：{exc}"

        # 目标交给弹的制导管（§5.8）——弹从此自己盯它。
        missile = self._mount.registry.get(missile_id)
        guide = getattr(missile, "component", lambda s: None)("guidance")
        assign = getattr(guide, "assign_target_id", None)
        if assign is None:
            return False, (
                f"弹 {missile_id} 的平台类型没有 guidance 槽或制导件不支持"
                "运行期指定目标"
            )
        assign(tid)

        self._stock -= 1
        bus = self._mount.bus
        emit = getattr(bus, "weapon_fired", None)
        if emit is not None:
            emit.emit(me, missile_id, tid, self.weapon_class())
        return True, f"发射（弹={missile_id}，目标={tid}，余弹 {self._stock}）"


__all__ = ["WEAPON_CLASSES", "Weapon"]
