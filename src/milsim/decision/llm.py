"""大模型决策提供者。

为什么必须异步
--------------
大模型一次调用要 1~30 秒，而仿真步长是 2 秒。如果同步等待，仿真时间会被
真实世界的推理耗时绑架——推演 10 分钟的想定要等几个小时。

做法：把请求丢进线程池立刻返回，模型返回后通过回调交还结果。
主循环在此期间照常推进，实体按"决策未定"时的默认行为运行。

这不是妥协，而是**更真实的建模**。军事上的指挥周期本来就有几秒到几十秒，
大模型的响应延迟恰好对应这个量级——所以延迟不该被消除，该被当作仿真量
记录在 ``DecisionResponse.latency_us`` 里。

三个必须处理好的失败路径
------------------------
1. **超时** —— 模型迟迟不回。用 ``threading.Timer`` 在 deadline 处触发降级。
2. **错误** —— 网络断、限流、返回格式不符。捕获后降级。
3. **竞态** —— 超时降级已发出，真实响应随后才到。必须丢弃后者。

第 3 点是本模块最容易写错的地方（也是写错后最难查的）。
用一个带锁的 "完成门闩" 保证每个请求恰好 resolve 一次。

线程契约
--------
``resolve`` 会在**工作线程**里被调用。调用方在回调里只能往
``engine.inbox`` 投递，绝不能直接碰仿真状态。
"""

from __future__ import annotations

import logging
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Callable, Protocol, runtime_checkable

from ..engine.time import SECOND, to_seconds
from .base import (
    DecisionRequest,
    DecisionResponse,
    ResolveFn,
    SOURCE_FALLBACK,
    SOURCE_LLM,
)

log = logging.getLogger(__name__)

#: 默认决策时限。30 秒是任务级仿真的合理上限——
#: 真实的营连级指挥周期通常在这个量级以内。
DEFAULT_TIMEOUT_US: int = 30 * SECOND


# ---------------------------------------------------------------------------
# 模型客户端
# ---------------------------------------------------------------------------

@runtime_checkable
class LLMClient(Protocol):
    """大模型客户端。实现只要提供一个同步的 ``complete``。

    换成任意厂商只改这一层：OpenAI 兼容接口、本地推理服务、公司内网网关。
    """

    def complete(self, prompt: str, timeout_s: float) -> str:
        ...


class MockLLMClient:
    """测试与离线推演用的假客户端。

    可以配置固定延迟来模拟真实推理耗时——这对验证"决策延迟"对推演结果的
    影响非常有用（同一个想定跑 1 秒延迟和 20 秒延迟，结果应当有明显差异）。
    """

    __slots__ = ("_response", "_delay_s", "_fail_times", "_calls")

    def __init__(
        self,
        response: str = "ACTION: hold\nCONFIDENCE: 0.5\nREASON: mock",
        delay_s: float = 0.0,
        fail_times: int = 0,
    ) -> None:
        self._response = response
        self._delay_s = delay_s
        self._fail_times = fail_times
        self._calls = 0

    @property
    def calls(self) -> int:
        return self._calls

    def complete(self, prompt: str, timeout_s: float) -> str:
        self._calls += 1
        if self._delay_s > 0:
            # 刻意**不**按 timeout_s 截断。
            # 真实世界里模型服务不会因为客户端设了短超时就提前返回——它还在算，
            # 只是结果来得比调用方愿意等的时间晚。这个"晚到的响应"正是
            # 竞态测试要覆盖的场景，截断掉就没法测了。
            time.sleep(self._delay_s)
        if self._calls <= self._fail_times:
            raise ConnectionError(f"模拟的第 {self._calls} 次调用失败")
        return self._response


# ---------------------------------------------------------------------------
# 提示词与解析
# ---------------------------------------------------------------------------

def default_prompt_builder(request: DecisionRequest) -> str:
    """默认提示词模板。

    刻意做得简短、结构化、可解析。任务级决策不需要长篇推理——
    选项是预定义的，模型只需选一个。
    """
    options = "\n".join(
        f"{i + 1}. {opt}" for i, opt in enumerate(request.options)
    )
    context_lines = "\n".join(
        f"- {k}: {v}" for k, v in request.context.items()
    )
    return (
        "你是军事指挥决策辅助系统。请从给定选项中选出最合适的一个动作。\n\n"
        f"【决策类型】{request.kind}\n\n"
        f"【当前态势】\n{request.situation}\n\n"
        f"【结构化信息】\n{context_lines or '（无）'}\n\n"
        f"【可选动作】\n{options}\n\n"
        "【输出格式】严格按下面三行输出，不要添加其他内容：\n"
        "ACTION: <动作名称，必须与可选动作完全一致>\n"
        "CONFIDENCE: <0 到 1 之间的小数>\n"
        "REASON: <一句话理由>"
    )


_ACTION_RE = re.compile(r"ACTION\s*[:：]\s*(.+)", re.IGNORECASE)
_CONFIDENCE_RE = re.compile(r"CONFIDENCE\s*[:：]\s*([0-9]*\.?[0-9]+)", re.IGNORECASE)
_REASON_RE = re.compile(r"REASON\s*[:：]\s*(.+)", re.IGNORECASE)


def default_parser(text: str, request: DecisionRequest) -> DecisionResponse:
    """解析模型输出。

    **最关键的一步是校验动作合法性**。模型可能返回选项之外的词、返回编号、
    返回带标点的原文。非法动作必须抛异常走降级，绝不能让引擎拿到一个
    它不认识的动作名——那会变成运行时才暴露的诡异 bug。
    """
    match = _ACTION_RE.search(text)
    if not match:
        raise ValueError(f"模型输出中找不到 ACTION 字段: {text[:120]!r}")

    action = match.group(1).strip().strip("\"'。，,.、")

    # 容忍模型返回编号而非动作名
    if action.isdigit():
        index = int(action) - 1
        if 0 <= index < len(request.options):
            action = request.options[index]

    # 容忍大小写与空白差异
    if action not in request.options:
        normalized = {opt.strip().lower(): opt for opt in request.options}
        action = normalized.get(action.lower(), action)

    if action not in request.options:
        raise ValueError(
            f"模型返回了非法动作 {action!r}，合法选项为 {list(request.options)}"
        )

    conf_match = _CONFIDENCE_RE.search(text)
    confidence = float(conf_match.group(1)) if conf_match else 0.5
    confidence = min(max(confidence, 0.0), 1.0)

    params: dict[str, str] = {}
    reason_match = _REASON_RE.search(text)
    if reason_match:
        params["reason"] = reason_match.group(1).strip()

    return DecisionResponse(
        request_id=request.request_id,
        action=action,
        params=params,
        confidence=confidence,
        source=SOURCE_LLM,
    )


# ---------------------------------------------------------------------------
# 提供者
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class LLMStats:
    """运行统计。想定跑完后看一眼就知道模型这条链路是否健康。"""

    requests: int = 0
    succeeded: int = 0
    timeouts: int = 0
    errors: int = 0
    total_latency_us: int = 0

    @property
    def fallback_count(self) -> int:
        return self.timeouts + self.errors

    @property
    def fallback_rate(self) -> float:
        return self.fallback_count / self.requests if self.requests else 0.0

    @property
    def avg_latency_ms(self) -> float:
        return (
            self.total_latency_us / self.succeeded / 1000
            if self.succeeded
            else 0.0
        )

    def __str__(self) -> str:
        return (
            f"请求 {self.requests} / 成功 {self.succeeded} / "
            f"超时 {self.timeouts} / 错误 {self.errors} "
            f"（降级率 {self.fallback_rate:.1%}，平均延迟 {self.avg_latency_ms:.0f} ms）"
        )


class LLMDecisionProvider:
    """基于大模型的异步决策提供者。

    参数
    ----
    client
        模型客户端。
    fallback
        降级用的决策提供者（通常是 :class:`RuleDecisionProvider`）。
        超时、报错、解析失败时都由它接管。
        **必须提供**——没有降级的大模型决策在推演里等于定时炸弹。
    """

    __slots__ = (
        "_client",
        "_fallback",
        "_prompt_builder",
        "_parser",
        "_pool",
        "_timeout_us",
        "_name",
        "_stats",
        "_stats_lock",
    )

    def __init__(
        self,
        client: LLMClient,
        fallback: "object",
        *,
        prompt_builder: Callable[[DecisionRequest], str] = default_prompt_builder,
        parser: Callable[[str, DecisionRequest], DecisionResponse] = default_parser,
        max_workers: int = 4,
        default_timeout_us: int = DEFAULT_TIMEOUT_US,
        name: str = "llm",
    ) -> None:
        if fallback is None:
            raise ValueError("必须提供 fallback 决策提供者，否则模型不可用时仿真会停摆")

        self._client = client
        self._fallback = fallback
        self._prompt_builder = prompt_builder
        self._parser = parser
        #: 线程池大小 = 同时在飞的模型请求数上限。
        #: 设太小会让排队请求白白超时，设太大可能触发服务端限流。
        self._pool = ThreadPoolExecutor(
            max_workers=max_workers, thread_name_prefix="llm-decision"
        )
        self._timeout_us = default_timeout_us
        self._name = name
        self._stats = LLMStats()
        self._stats_lock = threading.Lock()

    # -- 对外 --------------------------------------------------------------

    @property
    def stats(self) -> LLMStats:
        with self._stats_lock:
            return LLMStats(
                requests=self._stats.requests,
                succeeded=self._stats.succeeded,
                timeouts=self._stats.timeouts,
                errors=self._stats.errors,
                total_latency_us=self._stats.total_latency_us,
            )

    def request(self, request: DecisionRequest, resolve: ResolveFn) -> None:
        """立刻返回，模型在工作线程里调用。"""
        with self._stats_lock:
            self._stats.requests += 1

        # ---- 完成门闩 ----
        # 超时降级和真实响应可能竞争同一个请求。只有第一个能通过。
        gate = threading.Lock()
        finished = False
        timer_holder: list[threading.Timer] = []

        def finish(response: DecisionResponse) -> None:
            nonlocal finished
            with gate:
                if finished:
                    return
                finished = True

            for timer in timer_holder:
                timer.cancel()

            with self._stats_lock:
                self._stats.total_latency_us += response.latency_us
                if response.is_fallback:
                    reason = response.params.get("_reason", "")
                    if reason == "timeout":
                        self._stats.timeouts += 1
                    else:
                        self._stats.errors += 1
                else:
                    self._stats.succeeded += 1

            resolve(response)

        # ---- 超时兜底 ----
        timeout_us = (
            request.deadline_us if request.deadline_us is not None
            else self._timeout_us
        )
        if timeout_us and timeout_us > 0:
            timer = threading.Timer(
                to_seconds(timeout_us),
                lambda: finish(
                    self._make_fallback(request, "timeout", timeout_us)
                ),
            )
            timer.daemon = True
            timer_holder.append(timer)
            timer.start()

        self._pool.submit(self._work, request, finish, timeout_us)

    def shutdown(self, wait: bool = True) -> None:
        """想定跑完后调用，回收线程池。"""
        self._pool.shutdown(wait=wait)

    # -- 内部 --------------------------------------------------------------

    def _work(
        self,
        request: DecisionRequest,
        finish: Callable[[DecisionResponse], None],
        timeout_us: int,
    ) -> None:
        started = time.perf_counter()
        try:
            prompt = self._prompt_builder(request)
            text = self._client.complete(prompt, to_seconds(timeout_us))
            response = self._parser(text, request)
            response.latency_us = int((time.perf_counter() - started) * 1_000_000)
            finish(response)
        except Exception as exc:  # noqa: BLE001 — 任何异常都必须降级，不能让决策悬空
            elapsed = int((time.perf_counter() - started) * 1_000_000)
            log.warning(
                "LLM 决策失败 req=%d kind=%s: %s: %s",
                request.request_id, request.kind, type(exc).__name__, exc,
            )
            finish(self._make_fallback(request, f"error:{type(exc).__name__}", elapsed))

    def _make_fallback(
        self,
        request: DecisionRequest,
        reason: str,
        latency_us: int,
    ) -> DecisionResponse:
        """向 fallback 提供者要一个决策。

        fallback 是同步的，直接在当前线程取结果。它自身也可能失败——
        那种情况下返回一个保底动作，绝不让请求悬空。
        """
        try:
            captured: list[DecisionResponse] = []
            self._fallback.request(request, captured.append)
            if captured:
                response = captured[0]
                response.source = SOURCE_FALLBACK
                response.params.setdefault("_reason", reason)
                response.latency_us = latency_us
                return response
        except Exception:  # noqa: BLE001
            log.exception("fallback 决策本身也失败了 req=%d", request.request_id)

        return DecisionResponse(
            request_id=request.request_id,
            action=request.options[0] if request.options else "hold",
            params={"_reason": reason, "note": "fallback 失效，取首个候选动作"},
            confidence=0.0,
            source=SOURCE_FALLBACK,
            latency_us=latency_us,
        )

    def __repr__(self) -> str:
        return f"<LLMDecisionProvider {self._name} {self.stats}>"
