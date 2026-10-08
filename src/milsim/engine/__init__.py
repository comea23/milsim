"""引擎层：时间、事件、事件队列、混合步进推进器。

这一层不依赖任何上层模块，可以独立测试和复用。
"""

from .engine import (
    DEFAULT_MAX_DISPATCH_PER_STEP,
    PRIORITY_COMMAND,
    PRIORITY_COMMS,
    PRIORITY_ENGAGE,
    PRIORITY_EXTERNAL,
    PRIORITY_INTERNAL,
    PRIORITY_MOVER,
    PRIORITY_SENSOR,
    DispatchOverflow,
    Engine,
    EngineState,
)
from .event import Event, EventResult, OneShotEvent, RecurringEvent
from .event_queue import EventQueue
from .inbox import EventInbox, SyncInbox
from .time import (
    HOUR,
    INF_TIME,
    MAX_STEP,
    MILLISECOND,
    MINUTE,
    SECOND,
    Duration,
    SimTime,
    fmt,
    to_micros,
    to_seconds,
)

__all__ = [
    "Engine",
    "EngineState",
    "Event",
    "EventQueue",
    "EventResult",
    "OneShotEvent",
    "RecurringEvent",
    "DispatchOverflow",
    "EventInbox",
    "SyncInbox",
    "SimTime",
    "Duration",
    "MAX_STEP",
    "INF_TIME",
    "SECOND",
    "MILLISECOND",
    "MINUTE",
    "HOUR",
    "to_micros",
    "to_seconds",
    "fmt",
    "PRIORITY_SENSOR",
    "PRIORITY_COMMAND",
    "PRIORITY_MOVER",
    "PRIORITY_ENGAGE",
    "PRIORITY_COMMS",
    "PRIORITY_INTERNAL",
    "PRIORITY_EXTERNAL",
    "DEFAULT_MAX_DISPATCH_PER_STEP",
]
