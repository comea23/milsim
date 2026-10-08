"""线程安全的事件投递箱。

用途
----
引擎本身**不是线程安全的**，也刻意不打算做成线程安全的——那会拖慢每一步。
但有三个场景必须从外部线程把东西送进引擎：

  1. 大模型决策返回（HTTP 回调线程）
  2. UI / 网络 / 文件监控等外部输入
  3. 并行计算阶段产出的结果（numpy 释放 GIL 的那种）

做法参考 AFSIM 的挂钟事件队列（``WsfSimulation::mWallEventManager`` +
``AddWallEvent`` / ``DispatchWallEvents``）：开一条独立的、只用于"往里塞东西"
的通道，主循环在每个 step 开头把它清空。

约定
----
外部线程只能调 ``post()``，**绝不能**直接碰 Engine 或 EventQueue。
投递进去的是无参可调用对象，由主线程在 ``engine.now`` 时刻执行。

这条规则很硬但很好守：只要外部线程的代码里只出现 ``inbox.post(...)``，
就一定是安全的。

    # 在 HTTP 回调线程里（不要在回调里直接改实体状态！）
    engine.inbox.post(lambda: commander.apply(response))

    # 更好：连状态解析也推迟到主线程，回调线程只搬运原始数据
    engine.inbox.post(lambda: commander.apply_parsed(raw_text))
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Iterable

#: 投递进去的动作签名。无参数——需要的一切都靠闭包捕获。
Action = Callable[[], None]


class EventInbox:
    """多生产者、单消费者的线程安全动作队列。

    只有 ``post()`` 是线程安全的；``drain()`` 只应由主线程调用。
    """

    __slots__ = ("_lock", "_pending", "_post_count")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending: list[Action] = []
        self._post_count = 0

    def post(self, action: Action) -> None:
        """从任意线程投递一个动作。O(1)，持锁时间极短。"""
        with self._lock:
            self._pending.append(action)
            self._post_count += 1

    def drain(self) -> list[Action]:
        """取出并清空当前所有待处理动作。只应由主线程调用。

        采用"整体换出"而非逐个 pop：持锁时间与待处理数量无关，
        且换出后主线程可以无锁地遍历执行。
        """
        with self._lock:
            items = self._pending
            self._pending = []
            return items

    def __len__(self) -> int:
        with self._lock:
            return len(self._pending)

    @property
    def posted_total(self) -> int:
        """累计投递总数。用于诊断——长期只增不减说明有人在狂投。"""
        with self._lock:
            return self._post_count

    def __repr__(self) -> str:
        return f"<EventInbox pending={len(self)} total={self.posted_total}>"


class SyncInbox(EventInbox):
    """同步执行的投递箱。单线程测试用，行为等价但省掉线程与锁。

    只应在明确知道没有并发时使用（单元测试、单线程回放）。
    """

    __slots__ = ()

    def post(self, action: Action) -> None:  # type: ignore[override]
        action()
        self._post_count += 1

    def drain(self) -> list[Action]:  # type: ignore[override]
        return []


def post_all(inbox: EventInbox, actions: Iterable[Action]) -> Any:
    """批量投递的便利函数。"""
    for action in actions:
        inbox.post(action)
