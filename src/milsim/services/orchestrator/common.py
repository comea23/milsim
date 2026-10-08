"""分级控制骨架的公共部分：状态变更记录、状态转移校验、公共错误。

为什么状态转移要单独一个函数来管
--------------------------------
`Task` 和 `Intent` 各有一套状态。如果各写各的 ``if`` 判断，很快就会变成
两套风格不同的规则，而且新增状态时容易漏掉某个调用点。

这里把"能不能转"抽成一张**表**，转移动作只做一件事：查表、报错、记录。
好处有三个：非法转移当场报错；错误消息里带上"从哪到哪"；新状态必须显式
写进表里——而写表的时候，人自然会想一遍它和现有状态的组合。

为什么用白名单而不是黑名单
--------------------------
黑名单只拦得住想得到的错误组合。``PENDING`` 直接跳到 ``COMPLETED``
（没开工就完成）很像笔误，但只要没人把它写进黑名单，它就悄悄过去了。
白名单拦得住没想到的那些，代价是加新状态时要动表——这正是我们想要的。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from ...engine.time import SimTime
from ...errors import MilsimError


class TaskError(MilsimError, ValueError):
    """任务或意图的结构、状态不合法。

    落在 :class:`~milsim.errors.MilsimError` 下面是有意的：这类错误和
    "想定写错了"对调用方是同一件事——打一句人话给用户看就行，不需要
    栈回溯。同时继承 ``ValueError``，与框架其余部分保持一致。

    特别注意它会被**两个来源**触发：想定/程序写错了，或者大模型返回的
    细化结果不合规。后者是常态而非缺陷，所以校验必须给出**全部**问题而
    不是第一个——大模型改一次只修一个错，来回几轮就没人愿意用了。
    """


#: "没有具体的人"：系统、导演部、或无法归属的判定。
#:
#: **取 -1 而不是 0。** 实体 ID 与编队 ID 两个空间都从 **0** 开始
#: （``EntityRegistry(id_start=0)`` / ``FormationTree._next_id = 0``），
#: 所以 0 是一个完全合法的实体编号。用 0 当哨兵值，等于让"导演部"和
#: "0 号实体"变成同一个东西——而它**不报错**，只是把某个团体的判定
#: 记成了"系统判定"，或者反过来把系统事件归到 0 号实体头上。
#:
#: 取负数还有个附带好处：它能和 ``entity_id`` / ``unit_id`` 直接比较，
#: 不需要 ``None`` 分支，也不会踩到 ``None < 0`` 那个 TypeError。
NOBODY = -1

#: ID 空间标签：编队与实体是**两个各自从 0 开始的整数空间**。
#:
#: 放在公共模块里，是因为它不只用来标注"执行者是哪一级"，还要标注
#: "**判定者是哪一级**"——:class:`StateChange` 记的 ``by`` 是个裸整数，
#: 不给空间标签的话，"旅部（编队 #0）下达"会被解析成恰好同号的
#: "实体 #0 下达"。两个空间都是从 0 开始，"同号"是常态而不是巧合。
LEVEL_UNIT = "unit"
LEVEL_ENTITY = "entity"

ID_SPACES = frozenset({LEVEL_UNIT, LEVEL_ENTITY})

ID_SPACE_LABELS = {LEVEL_UNIT: "编队", LEVEL_ENTITY: "实体"}


@dataclass(slots=True)
class StateChange:
    """一次状态变更。**谁、在什么时候、凭什么判定的**（§4.5.2c）。

    §4.5.2c 列的三个问题里，"没有状态"是最容易被忽略的一个。补状态机很
    直观，但只有状态机、没有变更记录，复盘时"这条任务为什么变成失败"
    还是要靠猜。

    所以三样都必须留下：

    - ``at``：时刻。可复现，也是排序依据。
    - ``by``：判定者。可追责——是执行方报的，还是台账判的，差别很大。
    - ``reason``：理由。可解释，且要考虑它会被打给用户看。

    ``from_state`` 也留下：否则读日志的人得把整条时间线自己接起来，
    而日志常常是**跳着**看的。

    ``by_level`` 标出判定者的 ID 属于哪个空间。少了它会出这种错：
    "旅部下达"（编队 #0）被判成"实体 #0 下达"——两个空间都从 0 开始，
    同号是常态。默认取编队空间，因为上级与系统判定是常态；执行方自己
    上报时由 :meth:`Task.activate` / :meth:`Task.complete` 覆盖成执行者的
    层级。
    """

    at: SimTime
    from_state: str
    state: str
    by: int = NOBODY
    by_level: str = LEVEL_UNIT
    reason: str = ""

    def describe(self) -> str:
        who = f"#{self.by}" if self.by != NOBODY else "系统"
        if self.by != NOBODY:
            who = f"{ID_SPACE_LABELS.get(self.by_level, self.by_level)}{who}"
        tail = f"（{self.reason}）" if self.reason else ""
        return f"{self.from_state} → {self.state}，由{who}判定{tail}"


def label(name: str, labels: Mapping[str, str] | None) -> str:
    """``(name, 中文名)`` 的形式。输出给人看时用。"""
    if labels and name in labels:
        return f"{name}（{labels[name]}）"
    return repr(name)


def names(values: "frozenset[str] | set[str] | tuple[str, ...]",
          labels: Mapping[str, str] | None = None) -> str:
    """枚举一组取值，顺序稳定。错误消息里列"可用值"时用。"""
    return " / ".join(label(v, labels) for v in sorted(values))


def transition(
    *,
    current: str,
    target: str,
    table: Mapping[str, frozenset[str]],
    labels: Mapping[str, str] | None = None,
    what: str = "状态",
) -> None:
    """校验一次状态转移。不合法就抛 :class:`TaskError`。

    只做校验，不改数据——真正赋值由调用方在通过之后做。这样"改状态"和
    "记录这次改动"能挨在一起，不会因为校验掺在中间而写漏记录。
    """
    if current == target:
        raise TaskError(
            f"{what} {target!r} 重复设置：它已经是这个值了。"
            "重复设置通常意味着调用方在重放一段已经处理过的输入"
        )

    allowed = table.get(current)
    if allowed is None:
        raise TaskError(
            f"未知的{what} {current!r}，可用取值：{names(set(table), labels)}"
        )
    if target in allowed:
        return

    message = f"非法的{what}转移：{current!r} → {target!r}"
    if labels:
        message += f"（{labels.get(current, current)} → {labels.get(target, target)}）"
    if not allowed:
        message += f"。{current!r} 已是终态，不能再变"
    else:
        message += f"。{current!r} 只能转到 {names(allowed, labels)}"
    raise TaskError(message)


__all__ = [
    "ID_SPACE_LABELS",
    "ID_SPACES",
    "LEVEL_ENTITY",
    "LEVEL_UNIT",
    "NOBODY",
    "StateChange",
    "TaskError",
    "label",
    "names",
    "transition",
]
