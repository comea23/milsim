"""引擎：混合步进时间推进器。

推进规则
--------
这是整个仿真器的心脏，只有一行：

    Δt = min( 下一事件时刻, 当前时刻 + MAX_STEP )

"能跳多远跳多远，但一次最多跳 2 秒。" 任何一个推进步只会被下面两者之一选中：

    有事件等着  → 跳到该事件时刻，派发它（可能连续派发若干个同时刻事件）
    没事件      → 跳到栅栏 now + MAX_STEP，只跑兜底回调

这个规则同时给出三个保证：

* **事件不被延迟** —— 离散动作在自己的精确时刻发生，误差为零。
* **时间轴不被拉长** —— 无事发生的时段以 2 秒为粒度跳进，不会因为
  "下一个事件在半小时之后"而把一步拉成半小时。
* **状态刷新有上界** —— 外部观察者（UI、快照、统计）最多等 2 秒
  就能看到一次新状态。

三条驱动源必须分清
------------------
1. **事件** —— 主力。探测、消息、毁伤、任务完成等离散动作。
2. **时间栅栏（2s）** —— 兜底。只保证时间单调推进和刷新节拍。
   **不要**把周期性模型逻辑写进栅栏回调。
3. **模型自定周期** —— 每个部件注册自己的 update_interval，
   引擎在部件启动时自动派生一个 RecurringEvent。
   这与栅栏是两回事，混用会导致模型更新节奏失控。

需要比 2 秒更高时间精度的模型（弹道交会、近炸引信），在自己的 update()
内部做**帧内子步**积分，**不改变全局时间轴**。这是全局 2 秒步长能成立的
前提，否则各模型的精度需求会把步长一路拖到毫秒级。
"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable

from .event import Event, EventResult, OneShotEvent, RecurringEvent
from .event_queue import EventQueue
from .inbox import EventInbox
from .time import MAX_STEP, Duration, SimTime

# ---------------------------------------------------------------------------
# 优先级分段
#
# 约束的是**同一时刻**的次序——不同时刻的事件永远按时间先后执行。
# **数值小者先执行**，所以 0 是最高优先级。
#
# 间隔取 100 是为了后续插入新类别时不必重排已有常量。
#
# 外部投递排最前，因为它是"刚刚抵达的新输入"：大模型可能已经算了几十秒，
# 让它再排在感知、决策之后毫无道理；而且要保证它在本刻一开始就进入状态，
# 否则同刻的后续动作会基于过期决策展开。
# ---------------------------------------------------------------------------
PRIORITY_EXTERNAL = 0      # 外部线程投递：LLM 决策返回、UI 指令、上级命令
PRIORITY_SENSOR = 100      # 感知与航迹更新
PRIORITY_COMMAND = 200     # 指挥决策与任务分配
PRIORITY_MOVER = 300       # 机动推进
PRIORITY_ENGAGE = 400      # 交战与毁伤裁决
PRIORITY_COMMS = 500       # 通信投递
PRIORITY_INTERNAL = 900    # 引擎内部：实体增删、快照、心跳（也是调度默认值）

#: 单个 step 内派发事件数的保护阈值。
#: 正常情况下同一时刻不会有这么多事件；一旦超过，几乎必然是事件在执行中
#: 反复创建同刻事件造成的死循环。与其让进程卡死，不如快速失败。
DEFAULT_MAX_DISPATCH_PER_STEP = 1_000_000


class EngineState(Enum):
    CREATED = auto()
    INITIALIZED = auto()
    RUNNING = auto()
    PAUSED = auto()
    COMPLETED = auto()


class DispatchOverflow(RuntimeError):
    """单个 step 内事件派发数超限，通常意味着事件循环自激。"""


class Engine:
    """仿真引擎门面：持有时间轴、事件队列和运行状态。"""

    __slots__ = (
        "now",
        "queue",
        "inbox",
        "state",
        "_max_step",
        "_max_dispatch",
        "_fence_hooks",
        "dispatch_count",
        "fence_count",
        "step_count",
    )

    def __init__(
        self,
        max_step: Duration = MAX_STEP,
        max_dispatch_per_step: int = DEFAULT_MAX_DISPATCH_PER_STEP,
    ) -> None:
        self.now: SimTime = 0
        self.queue = EventQueue()
        #: 外部线程（LLM 回调、UI、并行计算）向引擎投递动作的唯一入口。
        #: 见 inbox.py 的约定：外部线程只许调 post()，绝不直接改引擎状态。
        self.inbox = EventInbox()
        self.state = EngineState.CREATED
        self._max_step = max_step
        self._max_dispatch = max_dispatch_per_step
        self._fence_hooks: list[Callable[[SimTime], None]] = []

        self.dispatch_count = 0   # 累计派发事件数
        self.fence_count = 0      # 累计触发栅栏次数
        self.step_count = 0       # 累计推进步数

    # -- 生命周期 ----------------------------------------------------------

    def initialize(self) -> None:
        if self.state is not EngineState.CREATED:
            raise RuntimeError(f"重复初始化：当前状态 {self.state.name}")
        self.state = EngineState.INITIALIZED

    def start(self) -> None:
        if self.state is EngineState.CREATED:
            self.initialize()
        if self.state is EngineState.COMPLETED:
            raise RuntimeError("仿真已结束，请先 reset()")
        self.state = EngineState.RUNNING

    def pause(self) -> None:
        if self.state is not EngineState.RUNNING:
            raise RuntimeError(f"无法暂停：当前状态 {self.state.name}")
        self.state = EngineState.PAUSED

    def resume(self) -> None:
        if self.state is not EngineState.PAUSED:
            raise RuntimeError(f"无法恢复：当前状态 {self.state.name}")
        self.state = EngineState.RUNNING

    def complete(self) -> None:
        self.state = EngineState.COMPLETED

    def reset(self) -> None:
        self.queue.clear()
        self.now = 0
        self.state = EngineState.CREATED
        self.dispatch_count = 0
        self.fence_count = 0
        self.step_count = 0

    # -- 事件调度 ----------------------------------------------------------

    def add_event(self, event: Event) -> Event:
        """把已构造好的事件入队。事件时间可以早于 now，但不会被回溯执行。"""
        return self.queue.push(event)

    def schedule_at(
        self,
        time: SimTime,
        fn: Callable[[Engine], None],
        priority: int = PRIORITY_INTERNAL,
    ) -> Event:
        """在绝对时刻 time 触发一次。

        ``priority`` 默认是最低优先级——**未指定就排在最后**。
        这样忘记传参时不会意外抢占感知、决策的位置；常规事件请显式传
        对应的 ``PRIORITY_*`` 常量。
        """
        return self.queue.push(OneShotEvent(time, fn, priority))

    def schedule(
        self,
        delay: Duration,
        fn: Callable[[Engine], None],
        priority: int = PRIORITY_INTERNAL,
    ) -> Event:
        """在 now + delay 触发一次。delay 单位为微秒。默认优先级说明同 schedule_at。"""
        return self.queue.push(OneShotEvent(self.now + delay, fn, priority))

    def schedule_recurring(
        self,
        delay: Duration,
        interval: Duration,
        fn: Callable[[Engine, RecurringEvent], EventResult],
        priority: int = PRIORITY_INTERNAL,
    ) -> RecurringEvent:
        """在 now + delay 首次触发，之后每 interval 一次。

        这就是"模型自定周期"的落地点：部件在 initialize() 里调用它注册
        自己的 update_interval，引擎负责后续重排。

        回调**只需返回 RESCHEDULE**，不要自己改 time——interval 由事件自己
        推进（见 event.py 的说明）。想中途换周期，直接改 ``ev.interval``。
        """
        return self.queue.push(
            RecurringEvent(self.now + delay, fn, priority, interval=interval)
        )

    def on_fence(self, hook: Callable[[SimTime], None]) -> None:
        """注册时间栅栏回调。

        只用于兜底逻辑：心跳、快照、UI 刷新。**不要把模型更新放这里**，
        那会让所有模型被迫跟随 2 秒节拍，失去各自周期的意义。
        """
        self._fence_hooks.append(hook)

    # -- 时间推进 ----------------------------------------------------------

    def step_once(self) -> SimTime:
        """推进一步，返回推进后的时间。这是混合步进的完整实现。"""
        if self.state is not EngineState.RUNNING:
            raise RuntimeError(f"引擎状态为 {self.state.name}，无法推进")

        # 0) 把外部线程投递进来的动作转成本刻事件。
        #    必须在计算 t_target 之前做：这些动作的时间戳就是"现在"，
        #    晚一步处理会让 LLM 返回的决策凭空多延迟一个 step。
        #    用默认参数 a=action 固化闭包变量，避免经典的循环捕获陷阱。
        for action in self.inbox.drain():
            self.queue.push(
                OneShotEvent(self.now, lambda _, a=action: a(), PRIORITY_EXTERNAL)
            )

        t_fence = self.now + self._max_step
        t_target = min(self.queue.peek_time(), t_fence)

        # 1) 派发所有时刻 <= t_target 的事件。
        #    循环内重新 peek 是刻意的：事件执行时可能插入新的、时刻仍
        #    <= t_target 的事件，它们必须在同一步内被处理掉，否则就会
        #    违反"事件不被延迟"的保证。
        dispatched = 0
        while True:
            event = self.queue.peek()
            if event is None or event.time > t_target:
                break
            self.queue.pop()
            if event.time > self.now:
                self.now = event.time
            if event.execute(self) is EventResult.RESCHEDULE:
                self.queue.push(event)
            dispatched += 1
            if dispatched > self._max_dispatch:
                raise DispatchOverflow(
                    f"单步派发事件数超过 {self._max_dispatch}，"
                    f"请检查是否有事件在执行中反复创建同刻事件"
                )

        # 2) 推进到目标时刻
        self.now = t_target
        self.dispatch_count += dispatched
        self.step_count += 1

        # 3) 被栅栏截断说明这段时间内无事发生，跑兜底回调
        if t_target == t_fence:
            self.fence_count += 1
            for hook in self._fence_hooks:
                hook(self.now)

        return self.now

    def run_to(self, end_time: SimTime) -> SimTime:
        """一直推进到不少于 end_time。

        注意会产生超调：由于单步最多 2 秒，实际停止时刻可能略晚于
        end_time。若需要精确停点，请用 step_until()。
        """
        if self.state is not EngineState.RUNNING:
            self.start()
        # 注意这里**不能**加 "and self.queue"：队列为空时时间轴仍需继续走，
        # 只是每一步都由栅栏截断（空转推进）。否则"推演两小时但后一小时
        # 无事发生"的场景会提前停在最后一个事件处。
        while self.now < end_time:
            self.step_once()
        return self.now

    def step_until(self, end_time: SimTime) -> SimTime:
        """逐事件推进直到 end_time，不产生超调（用 MAX_STEP 切分剩余区间）。

        适合需要精确时间点的场景，如按固定节拍导出快照。
        """
        if self.state is not EngineState.RUNNING:
            self.start()
        while self.now < end_time:
            remaining = end_time - self.now
            if remaining >= self._max_step:
                self.step_once()
                continue
            # 剩余不足一步：临时把栅栏收窄到 end_time
            saved = self._max_step
            self._max_step = remaining
            try:
                self.step_once()
            finally:
                self._max_step = saved
        return self.now

    # -- 调试辅助 ----------------------------------------------------------

    def drain(self, limit: int = 100_000) -> int:
        """把队列里的事件全部跑完，返回派发数量。仅用于测试与收尾清理。"""
        count = 0
        while self.queue and count < limit:
            self.step_once()
            count += 1
        return count

    def pending_times(self, n: int = 10) -> list[SimTime]:
        """返回最近的 n 个事件时刻（不改动队列）。用于调试。"""
        return sorted(e.time for e in self.queue)[:n]

    def __repr__(self) -> str:
        return (
            f"<Engine {self.state.name} now={self.now} "
            f"queued={len(self.queue)} dispatched={self.dispatch_count}>"
        )
