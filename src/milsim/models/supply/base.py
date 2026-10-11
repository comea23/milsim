"""补给组件（§5.18，v0.13.58）——弹药与油料的陆地/海上来源。

``component supply SUPPLY`` 挂在补给车/补给舰上。``resupply`` 指令把
自带的弹药与油料转给**身边的**单位：

- **补弹**：目标实体上所有 :class:`~milsim.models.weapon.base.Weapon` 槽
  逐个装填（缺多少补多少，补给车自己的库存同步扣减）；
- **补油**：目标实体的机动件
  :meth:`~milsim.models.mover.base.Mover.add_fuel`（缺多少补多少）。

**补给半径**（``resupply_radius``）内才算"到了身边"——隔着 40 km 下
resupply 指令等于没有物流，拒收。补给车自己的库存是**有限**的：给光了
再去发射平台 reload？不——它没有 reload，它回到补给点再领属于任务层
的事，v1 只做"转手"。
"""

from __future__ import annotations

from typing import Any

from ...services.params import Params
from ...services.type_registry import register_component
from ..component import Component


@register_component("SUPPLY")
class Supply(Component):
    """补给组件：转手弹药与油料。"""

    SLOT_HINT = "supply"

    #: 运行期指令（§5.18）。
    COMMANDS = {
        "resupply": "补给：args = {'supply_to': 实体名或编号}，须在补给半径内",
    }

    #: 认领的任务类型（§5.19）：resupply 任务 = 给目标实体转手弹药与油料。
    #: ★ 只管"转手"不管"开过去"——受补单位不在补给半径内任务失败
    #: （``out_of_range``），要送货上门请先派 move 任务把车开过去。
    TASKS = ("resupply",)

    PARAMS = {
        #: 自带弹药（发）。补给车上每个发射槽共用的"弹药堆"。
        "ammo": Params.integer(8, minimum=0),
        #: 自带油料（kg）。
        "fuel_kg": Params.mass(2000.0, minimum=0.0),
        #: 补给半径（米）：目标不进圈拒收。默认 200 m——车队/靠帮的距离。
        "resupply_radius": Params.distance(200.0, minimum=0.0),
    }

    __slots__ = ("_mount", "_ammo", "_fuel", "_results")

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self, mount: Any) -> None:
        self._mount = mount
        self._ammo = int(self.spec["ammo"])
        self._fuel = float(self.spec["fuel_kg"])
        #: 在身 resupply 任务的一次性结论：task_id → (状态, 子状态)。
        #: 补给在认领那一刻就发生完了，结论攒着等台账下一拍来取。
        self._results: dict[int, tuple[str, str]] = {}

    # -- 查询 --------------------------------------------------------------

    def ammo_left(self) -> int:
        return self._ammo

    def fuel_left_kg(self) -> float:
        return self._fuel

    # -- 指令（§5.18） ------------------------------------------------------

    def execute(self, cmd: str, args: dict) -> tuple[bool, str]:
        ok, why = super().execute(cmd, args)
        if not ok and cmd not in self.COMMANDS:
            return ok, why

        raw = args.get("supply_to")
        if isinstance(raw, bool) or not isinstance(raw, (str, int)):
            return False, "resupply 需要 supply_to（受补实体名或编号）"
        registry = self._mount.registry
        if isinstance(raw, int):
            found = registry.get(raw)
            tid = raw if found is not None else None
        else:
            hit = registry.by_name(str(raw))
            tid = hit.entity_id if hit is not None else None
        if tid is None:
            return False, f"受补单位 {raw!r} 不在这份想定里"
        if tid == self._mount.entity_id:
            return False, "不能给自己补给"

        return self._transfer(tid)

    def _transfer(self, tid: int) -> tuple[bool, str]:
        """给受补单位 ``tid`` 转手弹药与油料。``resupply`` 指令与补给任务
        共用这一条链（§5.19）。"""
        store = self._mount.store
        registry = self._mount.registry
        mine = store.position_of(self._mount.entity_id)
        theirs = store.position_of(tid)
        if mine is None or theirs is None:
            return False, "补给方或受补方没有位置（不在任何战区里）"
        dist = ((theirs[0] - mine[0]) ** 2 + (theirs[1] - mine[1]) ** 2) ** 0.5
        radius = float(self.spec["resupply_radius"])
        if dist > radius:
            return False, (
                f"受补单位距离 {dist:.0f} m 超出补给半径 {radius:.0f} m，"
                "开到身边再补"
            )

        target = registry.get(tid)
        parts = getattr(target, "parts", lambda: [])()
        ammo_used = 0
        fuel_used = 0.0
        for part in parts:
            add_stock = getattr(part, "add_stock", None)
            if callable(add_stock) and self._ammo > 0:
                need = max(0, int(getattr(part, "_capacity", 0)) - part.stock())
                give = min(need, self._ammo)
                if give > 0:
                    ammo_used += add_stock(give)
                    self._ammo -= give
            add_fuel = getattr(part, "add_fuel", None)
            if callable(add_fuel) and self._fuel > 0.0:
                cap = getattr(part, "fuel_capacity_kg", lambda: 0.0)()
                need = max(0.0, cap - part.fuel_kg())
                give = min(need, self._fuel)
                if give > 0.0:
                    fuel_used += add_fuel(give)
                    self._fuel -= give

        if ammo_used == 0 and fuel_used == 0.0:
            return False, "受补单位不缺弹药也不缺油（或没有可补的槽）"
        return True, (
            f"补给完成：弹药 {ammo_used} 发、油料 {fuel_used:.0f} kg"
            f"（车上余：弹药 {self._ammo} 发、油 {self._fuel:.0f} kg）"
        )

    # -- 任务执行器协议（§5.19）：resupply 任务 ------------------------------

    def acquire_task(self, task) -> tuple[bool, str]:
        """resupply 任务：认领那一刻就转手，结论等台账下一拍来取。"""
        if task.target_id is None:
            return False, "resupply 任务需要 target（受补实体）"
        if int(task.target_id) == self._mount.entity_id:
            return False, "不能给自己补给"
        ok, why = self._transfer(int(task.target_id))
        self._results[task.task_id] = (
            ("complete", why) if ok else ("failed", why)
        )
        return True, why

    def poll_task(self, task):
        verdict = self._results.pop(task.task_id, None)
        return verdict

    def release_task(self, task) -> None:
        self._results.pop(task.task_id, None)


__all__ = ["Supply"]
