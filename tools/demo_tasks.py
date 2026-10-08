"""任务骨架演示：意图 → 细化 → 原子登记 → 巡检失效（M5a）。

    python tools/demo_tasks.py [输出路径]

它跑的是**真实想定装配出来的世界**（编队树 / 注册表 / 存储都齐），
只有"细化器"是照着计划表摊的最简形态——真实实现要么查条令表、
要么问大模型，那是 M5b。

三张面板：

1. **任务时间轴** —— 每个执行者一行，颜色是任务**最终**状态；
   两条竖线分别是统一生效时刻与巡检时刻
2. **原子性对照** —— 逐条登记（旧做法）与整批登记在"第 5 条出错"时
   留下的台账存量，数字是跑出来的不是画的
3. **一条任务的状态链** —— 谁在什么时候、凭什么判定的
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milsim.simulation import Simulation  # noqa: E402
from milsim.services import (  # noqa: E402
    ASSIGNEE_LABELS,
    NOBODY,
    KIND_ENGAGE,
    KIND_MOVE,
    KIND_OCCUPY,
    KIND_RECON,
    LEVEL_ENTITY,
    LEVEL_UNIT,
    PRIORITY_URGENT,
    STATE_ABORTED,
    STATE_ACTIVE,
    STATE_COMPLETED,
    STATE_EXPIRED,
    STATE_FAILED,
    STATE_LABELS,
    STATE_PENDING,
    CommandLedger,
    Objective,
    TaskError,
    world_checker,
)

MIN = 60_000_000
H = 3_600_000_000

#: 演示想定：红方合成旅 vs 蓝方一个连。故意不写任何 component——
#: 组件层属 M4a，本次演示的是任务骨架，不是感知机动。
SCENARIO = """
simulation
    name     "夺取 116 高地"
    max_step 2 s
    seed     20260918
end_simulation

zone EAST_SECTOR
    anchor     39.9042 116.4074
    radius     60 km
    resolution 200 m
end_zone

# ---- 红方编队：旅 → 营 → 连 ----
formation RED_BDE
    side    red
    echelon 旅
end_formation
formation RED_1BN : RED_BDE
    side    red
    echelon 营
end_formation
formation RED_1_1 : RED_1BN
    side    red
    echelon 连
end_formation
formation RED_1_2 : RED_1BN
    side    red
    echelon 连
end_formation
formation RED_RECON : RED_BDE
    side    red
    echelon 侦察排
end_formation

# ---- 蓝方 ----
formation BLUE_1BN
    side    blue
    echelon 营
end_formation
formation BLUE_1_1 : BLUE_1BN
    side    blue
    echelon 连
end_formation

platform_type RED_IFV
    side red
end_platform_type
platform_type BLUE_HOLD
    side blue
end_platform_type

platform RED_1_1_A RED_IFV
    position  latlng 39.9300 116.3600
    formation RED_1_1
end_platform
platform RED_1_1_B RED_IFV
    position  latlng 39.9320 116.3640
    formation RED_1_1
end_platform
platform RED_1_2_A RED_IFV
    position  latlng 39.9400 116.3900
    formation RED_1_2
end_platform
platform RED_RECON_A RED_IFV
    position  latlng 39.9200 116.3400
    formation RED_RECON
end_platform
platform RED_2_1_A RED_IFV
    position  latlng 39.9350 116.3700
    formation RED_1_2
end_platform
platform BLUE_1_1_A BLUE_HOLD
    position  latlng 39.9600 116.4300
    formation BLUE_1_1
end_platform
platform BLUE_1_1_B BLUE_HOLD
    position  latlng 39.9620 116.4340
    formation BLUE_1_1
end_platform
"""

STATE_COLOR = {
    STATE_PENDING: "#5D6D7E",
    STATE_ACTIVE: "#B9770E",
    STATE_COMPLETED: "#1E8449",
    STATE_FAILED: "#B03A2E",
    STATE_ABORTED: "#6C3483",
    STATE_EXPIRED: "#95A5A6",
}


def rule(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def stamp(micros: int) -> str:
    return f"T+{micros / 60_000_000.0:6.1f} min"


# ---------------------------------------------------------------------------
# 最简形态的"细化器"
# ---------------------------------------------------------------------------

#: 意图细化出来的任务计划。真实实现里这张表要么来自条令，要么来自大模型。
#: ``(类型, 层级, 执行者名, 目标要素, 生效, 截止, 备注)``
#:
#: 目标要素取 ``None`` 表示"细化时按当时态势确定"——打击任务的目标实体
#: 不是上级给的，是细化器从态势里选出来的。
PLAN = (
    (KIND_OCCUPY, LEVEL_UNIT, "RED_1_1",
     Objective(area="HILL_116", criterion="连续 10 分钟无敌方活动"),
     30 * MIN, 8 * H, "主攻：占领 116 高地"),
    (KIND_RECON, LEVEL_UNIT, "RED_1_2",
     Objective(area="EAST_SECTOR", criterion="报告通行量"),
     30 * MIN, 6 * H, "翼侧：监视东侧公路"),
    (KIND_RECON, LEVEL_UNIT, "RED_RECON",
     Objective(area="EAST_SECTOR", criterion="建立观察哨"),
     30 * MIN, 4 * H, "侦察排前出"),
    (KIND_MOVE, LEVEL_ENTITY, "RED_1_1_A",
     Objective(point=(12000.0, -4000.0), radius_m=300.0),
     30 * MIN, 2 * H, "主攻连的突击排"),
    (KIND_MOVE, LEVEL_ENTITY, "RED_1_1_B",
     Objective(point=(11500.0, -2500.0), radius_m=300.0),
     30 * MIN, 2 * H, "主攻连的掩护排"),
    (KIND_ENGAGE, LEVEL_ENTITY, "RED_1_2_A", None,
     40 * MIN, 3 * H, "压制当面之敌"),
    (KIND_RECON, LEVEL_ENTITY, "RED_RECON_A",
     Objective(point=(9000.0, -6000.0)),
     30 * MIN, 3 * H, "侦察排前车"),
)

#: 演示原子性时故意写错第几条（0 起）。改成一条打击任务，但目标是不存在的实体。
BROKEN_INDEX = 4
BROKEN_ASSUMES = "RED_2_1_A"   # 预备排：没别的任务，便于把错误孤立成一条


def refine(ledger: CommandLedger, *, sim: Simulation, at: int, intent_id: int,
           break_at: int | None = None) -> list:
    """把意图摊成任务。``break_at`` 指定第几条故意写错。"""
    enemy = sim.registry.by_name("BLUE_1_1_A").entity_id
    tasks = []
    for index, (kind, level, name, objective, start, deadline, note) in enumerate(PLAN):
        if index == break_at:
            # 典型的大模型输出错误：番号幻觉，引用了一个不存在的目标
            kind, level, name = KIND_ENGAGE, LEVEL_ENTITY, BROKEN_ASSUMES
            objective = Objective(target=9999)
            start, deadline = 30 * MIN, 3 * H
            note = "（故意写错：目标实体不存在）"
        elif objective is None:
            objective = Objective(target=enemy, criterion="摧毁或使其失去战斗力")

        assignee = (
            sim.formations.resolve(name) if level == LEVEL_UNIT
            else sim.registry.by_name(name).entity_id
        )
        tasks.append(ledger.draft(
            kind=kind, assignee=assignee, assignee_level=level, objective=objective,
            issuer=sim.formations.resolve("RED_BDE"), at=at,
            effective_at=start, deadline_at=deadline,
            intent_id=intent_id, note=note,
        ))
    return tasks


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main() -> None:
    sim = Simulation.from_scenario(SCENARIO, name="夺取 116 高地")
    sim.build()
    ledger = CommandLedger()
    check = world_checker(sim.registry, sim.store, sim.formations)

    def name_of(ident: int, level: str) -> str:
        """层级一起用上。少了它，编队 #2 会被解析成恰好同号的那个平台。"""
        if level == LEVEL_UNIT:
            node = sim.formations.get(ident)
            return node.name if node is not None else f"编队#{ident}"
        entity = sim.registry.get(ident)
        return entity.name if entity is not None else f"实体#{ident}"

    def actor(change) -> str:
        """判定者是谁。**层级取自记录本身**（``by_level``），不靠推断。

        早先写的是"``by`` 等于执行者就按执行者的层级查"，结果"旅部下达"
        被显示成恰好同号的平台——两个 ID 空间都从 0 开始，同号是常态。
        判定者的空间是记录的一部分，不该由读的人猜。

        注意 ``NOBODY`` 要单独判，不能写成 ``if by:``：**0 是合法编号**，
        会被判成"没有人"。
        """
        if change.by == NOBODY:
            return "系统"
        return name_of(change.by, change.by_level)

    # -- 1 ------------------------------------------------------------------
    rule("① 世界就绪：编队树与实体")
    for decl in sim.spec.formations:
        indent = "  " if decl.parent is not None else ""
        total = len(sim.formations.all_entities_below(
            sim.formations.resolve(decl.name)))
        print(f"  {indent}{decl.name:<12} {decl.side:<5} {decl.echelon:<6}"
              f"下辖平台（含下级）{total}")
    print(f"\n  实体 {len(sim.registry)} 个，其中存活 {len(sim.store.alive_ids())} 个")
    print("  注意：实体 ID 与编队 ID **都从 0 开始**，是两个独立的整数空间")

    # -- 2 ------------------------------------------------------------------
    rule("② 宏观：旅部下达意图")
    intent = ledger.issue_intent(ledger.draft_intent(
        statement="夺取 116 高地并在其上建立观察哨，重点压制东侧公路",
        issuer=sim.formations.resolve("RED_BDE"), at=0, area="HILL_116",
        priority=PRIORITY_URGENT, expires_at=48 * H,
        constraints=("不得越过东部边界", "保持无线电静默至 T+30min"),
    ))
    print(f"  {intent.describe()}")

    # -- 3 ------------------------------------------------------------------
    rule("③ 战役：细化成任务（这里用最简形态，真实实现查条令或问大模型）")
    tasks = refine(ledger, sim=sim, at=0, intent_id=intent.intent_id)
    for task in tasks:
        print(f"  {task.describe(names_of=name_of)}")

    # -- 4 ------------------------------------------------------------------
    rule("④ 原子登记：先演示整批被拒")
    # 同一份计划、同一个错误，两种登记做法。旧做法是逐条 post_message。
    naive = CommandLedger()
    naive.issue_intent(naive.draft_intent(
        statement=intent.statement, issuer=intent.issuer, at=0, area=intent.area,
        priority=intent.priority, expires_at=intent.expires_at,
        constraints=intent.constraints,
    ))
    broken = refine(naive, sim=sim, at=0, intent_id=1, break_at=BROKEN_INDEX)

    registered_one_by_one = 0
    for task in broken:
        try:
            naive.issue([task], issuer=intent.issuer, at=0, check=check)
        except TaskError as exc:
            print(f"  ✗ 逐条登记在第 {registered_one_by_one + 1} 条失败：")
            print(f"    {str(exc).splitlines()[-1].strip()}")
            break
        registered_one_by_one += 1
    print(f"\n  ⚠ 逐条登记（旧做法）留下 {registered_one_by_one} 条已生效的任务"
          f"——半个营在推进，另半个还在原地")
    print("  注：这里**必须**传 check。台账不认识世界，"
          "只校验任务自身的结构；\n      \"目标实体存在吗\"要靠 world_checker 查注册表。"
          "忘了传，那条错误会一路通过。")

    # 整批登记：同一条错误，一条都不进台账。把好任务和那条坏任务放在同一批里。
    try:
        ledger.issue(
            tasks + [refine(ledger, sim=sim, at=0, intent_id=intent.intent_id,
                            break_at=BROKEN_INDEX)[BROKEN_INDEX]],
            issuer=intent.issuer, at=0, check=check,
        )
    except TaskError as exc:
        print("\n  ✓ 整批登记被拒，错误一次报全：")
        for line in str(exc).splitlines():
            print(f"    {line}" if line.startswith("  -") else f"  {line}")
    print(f"\n  此时台账：{ledger.describe()}")
    print(f"  逐条登记那份台账：{naive.describe()}")

    # 修正后一次性登记
    batch = ledger.issue(tasks, issuer=intent.issuer, at=0, check=check)
    print(f"\n  修正后：{batch.describe()}")
    print(f"  台账：{ledger.describe()}")

    # -- 5 ------------------------------------------------------------------
    rule("⑤ 执行：连队按统一生效时刻开工、上报")
    occupy = next(t for t in tasks if t.kind == KIND_OCCUPY)
    recon = next(t for t in tasks if t.note.startswith("侦察排前出"))
    engage = next(t for t in tasks if t.kind == KIND_ENGAGE)

    occupy.activate(at=32 * MIN, by=occupy.assignee)
    recon.activate(at=31 * MIN, by=recon.assignee)
    engage.activate(at=45 * MIN, by=engage.assignee)
    occupy.set_progress(0.6, at=70 * MIN)
    recon.complete(at=95 * MIN, by=recon.assignee, reason="观察哨已建立")
    print(f"  {occupy.describe(names_of=name_of)}")
    print(f"  {recon.describe(names_of=name_of)}")
    print(f"  {engage.describe(names_of=name_of)}")

    # -- 6 ------------------------------------------------------------------
    rule("⑥ 巡检：世界变了，台账要判失效")
    blue_a = sim.registry.by_name("BLUE_1_1_A").entity_id
    red_a = sim.registry.by_name("RED_1_1_A").entity_id
    sim.store.apply_damage(blue_a, 1.0)   # 目标被打掉
    sim.store.apply_damage(red_a, 1.0)    # 突击排也损失了
    print(f"  蓝方 {name_of(blue_a, LEVEL_ENTITY)} 被摧毁，"
          f"红方 {name_of(red_a, LEVEL_ENTITY)} 也被摧毁")

    report = ledger.sweep(120 * MIN, is_alive=sim.store.is_alive)
    print(f"\n  {report.describe()}")
    for task in report.tasks:
        print(f"    {task.describe(names_of=name_of)}")
        print(f"      判定：{task.history[-1].describe()}")

    idle = ledger.sweep(180 * MIN, is_alive=sim.store.is_alive)
    print(f"\n  再来一次：{idle.describe()}（巡检是幂等的，不会重复报同一件事）")

    # -- 7 ------------------------------------------------------------------
    rule("⑦ 台账终态")
    print(f"  {ledger.describe()}")
    for task in ledger.tasks():
        print(f"  {task.describe(names_of=name_of)}")
    print()
    for task in ledger.tasks():
        print(f"  任务 #{task.task_id} 的状态链：")
        for change in task.history:
            print(f"    {stamp(change.at)}  {change.from_state or '（新）':<10}"
                  f" → {change.state:<10} 由 {actor(change)} 判定："
                  f"{change.reason}")

    print("\n  注：objective.area 的名称目前**不做存在性校验**——台账不认识地图，"
          "\n      而且想定里只有战区声明，没有「目标区域」这一类。已记入待决策。")

    draw(tasks, ledger, naive, registered_one_by_one, occupy, report)


# ---------------------------------------------------------------------------
# 画图
# ---------------------------------------------------------------------------

def draw(tasks, ledger, naive, one_by_one: int, sample, report) -> None:
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False

    fig = plt.figure(figsize=(14, 11.5))
    grid = fig.add_gridspec(3, 1, height_ratios=(2.3, 1.1, 1.1), hspace=0.42)
    uniform = min(t.effective_at for t in tasks)

    # -- ① 时间轴 ----------------------------------------------------------
    ax = fig.add_subplot(grid[0])
    order = sorted(
        range(len(tasks)),
        key=lambda i: (tasks[i].assignee_level != LEVEL_UNIT, tasks[i].task_id),
    )
    labels = []
    for row, index in enumerate(order):
        task = tasks[index]
        start_min = task.effective_at / 60_000_000.0
        end_min = (task.deadline_at or task.effective_at) / 60_000_000.0
        ax.barh(row, end_min - start_min, left=start_min, height=0.52,
                color=STATE_COLOR.get(task.state, "#333"), alpha=0.85,
                edgecolor="white", linewidth=1.2)
        ax.text(start_min + 1.5, row, f"#{task.task_id}", va="center",
                fontsize=8, color="white", fontweight="bold")
        labels.append(
            f"{ASSIGNEE_LABELS.get(task.assignee_level, task.assignee_level)}"
            f"  {task.note}"
        )

    ax.set_yticks(range(len(order)))
    ax.set_yticklabels(labels, fontsize=9)
    ax.invert_yaxis()
    ax.set_xlabel("推演时间（分钟）", fontsize=10)
    ax.set_title(
        "① 任务时间轴：条 = [生效, 截止)，颜色 = 最终状态\n"
        "两条竖虚线是统一生效时刻（T+30min，通信何时送到都不许提前动）"
        "与巡检时刻（T+120min）",
        fontsize=11.5, pad=10,
    )
    ax.grid(axis="x", alpha=0.25, linestyle=":")

    ax.axvline(uniform / 60_000_000.0, color="#1F618D", linewidth=1.6,
               linestyle="--")
    ax.axvline(120, color="#B03A2E", linewidth=1.6, linestyle="--")

    handles = [
        plt.Line2D([], [], marker="s", linestyle="", markersize=10,
                   color=STATE_COLOR[s], label=STATE_LABELS[s])
        for s in (STATE_PENDING, STATE_ACTIVE, STATE_COMPLETED, STATE_FAILED,
                  STATE_ABORTED, STATE_EXPIRED)
    ]
    handles.append(plt.Line2D([], [], color="#1F618D", linestyle="--",
                              label="统一生效 T+30min"))
    handles.append(plt.Line2D([], [], color="#B03A2E", linestyle="--",
                              label=f"巡检 T+120min（{len(report.tasks)} 条失效）"))
    ax.legend(handles=handles, loc="lower right", fontsize=8.5, ncol=2,
              framealpha=0.92)

    # -- ② 原子性对照 ------------------------------------------------------
    ax = fig.add_subplot(grid[1])
    total = len(tasks)
    groups = ["逐条登记（旧做法）", "整批登记（现在）"]
    done = [one_by_one, 0]
    rejected = [total - one_by_one, total]

    ax.barh(groups, done, height=0.42, color="#B03A2E", alpha=0.85,
            label="已生效")
    ax.barh(groups, rejected, left=done, height=0.42, color="#D5D8DC",
            edgecolor="#839192", label="未下达")
    for row, (d, r) in enumerate(zip(done, rejected)):
        if d:
            ax.text(d / 2, row, f"{d} 条已生效", va="center", ha="center",
                    fontsize=9, color="white", fontweight="bold")
        if r:
            ax.text(d + r / 2, row, f"{r} 条未下达", va="center", ha="center",
                    fontsize=9, color="#566573")
    ax.set_xlim(0, total)
    ax.set_xlabel("任务条数", fontsize=10)
    ax.set_title(
        f"② 一批 {total + 1} 条任务里第 {BROKEN_INDEX + 1} 条校验失败时，"
        "两种做法留下的台账存量（跑出来的真实数字）\n"
        f"逐条登记：前 {one_by_one} 条已生效——半个营在推进、另半个还在原地，"
        "这个态势物理上不可能出现，却会被后续决策当成真实初始条件\n"
        "整批登记：一条都没进台账，错误一次报全，改完再整批下达",
        fontsize=11, pad=10,
    )
    ax.legend(fontsize=9, loc="lower right", framealpha=0.92)

    # -- ③ 一条任务的状态链 ------------------------------------------------
    ax = fig.add_subplot(grid[2])
    # 挑状态链最长的那条：链越长，"谁判定"越能显出层次
    failed = max(report.tasks, key=lambda t: len(t.history)) if report.tasks else sample
    ax.set_xlim(-0.5, 4.5)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_title("③ 一条任务的完整状态链：谁在什么时候、凭什么判定的",
                 fontsize=12, pad=6)

    marks = failed.history
    for index, change in enumerate(marks):
        x = index * 1.5
        color = STATE_COLOR.get(change.state, "#333")
        ax.plot([x], [0.55], marker="o", markersize=15, color=color,
                zorder=3)
        ax.text(x, 0.55, str(index + 1), ha="center", va="center",
                fontsize=9, color="white", fontweight="bold", zorder=4)
        ax.text(x, 0.78, f"{change.from_state or '（新）'} → {change.state}",
                ha="center", fontsize=9.5, color=color, fontweight="bold")
        ax.text(x, 0.30, f"{stamp(change.at)}\n{change.reason or '下达'}",
                ha="center", va="top", fontsize=8.5, color="#566573")
        if index + 1 < len(marks):
            ax.annotate("", xy=(x + 1.42, 0.55), xytext=(x + 0.08, 0.55),
                        arrowprops=dict(arrowstyle="->", color="#ABB2B9",
                                        lw=1.6))

    ax.text(4.5, 0.55, f"任务 #{failed.task_id}\n{failed.note}", ha="right",
            va="center", fontsize=9, color="#1C2833")

    fig.suptitle("分级控制骨架：意图 → 任务 → 原子登记 → 巡检失效（M5a）",
                 fontsize=15, y=0.985)

    out = (
        Path(sys.argv[1]) if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1] / "docs" / "images" / "tasks.png"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120, facecolor="white", bbox_inches="tight")
    print(f"\n  图已输出：{out}")


if __name__ == "__main__":
    main()
