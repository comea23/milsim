"""同步规则决策。

行为树、状态机、脚本三种决策模型在这里统一适配。它们本质上是同一回事：
**给定态势立即产出动作** 的纯映射，区别只在内部怎么组织规则。

本模块不实现规则引擎本身（那是 ``models/behave`` 的事），只提供适配契约
和几个常用策略。

同步不等于简单
--------------
规则决策一样可以很复杂——行为树可以有几百个节点、状态机可以有几十个状态。
"同步"只表示它在微秒级给出答案，不需要等待外部系统。
"""

from __future__ import annotations

import time
from typing import Any, Callable, Iterable

from .base import (
    DecisionRequest,
    DecisionResponse,
    ResolveFn,
    SOURCE_RULE,
)

#: 策略函数签名：态势 → 动作名，或直接给出完整响应
PolicyFn = Callable[[DecisionRequest], "str | DecisionResponse"]


class RuleDecisionProvider:
    """把任意策略函数适配成 DecisionProvider。

    回调是立即发生的（在同一调用栈内），这符合 ``base.py`` 的契约——
    调用方本来就必须按"回调可能立即发生"来写。
    """

    __slots__ = ("_policy", "_name")

    def __init__(self, policy: PolicyFn, name: str = "rule") -> None:
        self._policy = policy
        self._name = name

    def request(self, request: DecisionRequest, resolve: ResolveFn) -> None:
        t0 = time.perf_counter()
        outcome = self._policy(request)
        latency = int((time.perf_counter() - t0) * 1_000_000)

        if isinstance(outcome, DecisionResponse):
            response = outcome
            response.request_id = request.request_id
        else:
            response = DecisionResponse(
                request_id=request.request_id,
                action=outcome,
                source=self._name,
            )
        response.latency_us = latency
        resolve(response)

    def __repr__(self) -> str:
        return f"<RuleDecisionProvider {self._name}>"


class Rule:
    """一条规则：条件 + 动作。

    ``when`` 返回真值时采纳 ``action``。用 ``weight`` 可以做优先级排序，
    权重高的先匹配。
    """

    __slots__ = ("name", "when", "action", "weight")

    def __init__(
        self,
        name: str,
        when: Callable[[DecisionRequest], bool],
        action: str,
        weight: int = 0,
    ) -> None:
        self.name = name
        self.when = when
        self.action = action
        self.weight = weight


class RuleTable:
    """规则表：按权重降序、同权重按加入顺序匹配，命中即返回。

    这是行为树和状态机之外最朴素的决策形式，但在任务级仿真里覆盖了
    大多数场景——"弹药耗尽则返航"这类硬规则本来就不需要树或状态机。
    """

    __slots__ = ("_rules", "_default")

    def __init__(
        self,
        rules: Iterable[Rule] = (),
        default: str | None = None,
    ) -> None:
        self._rules = sorted(rules, key=lambda r: -r.weight)
        self._default = default

    def add(self, rule: Rule) -> "RuleTable":
        self._rules.append(rule)
        self._rules.sort(key=lambda r: -r.weight)
        return self

    def __call__(self, request: DecisionRequest) -> str:
        for rule in self._rules:
            try:
                if rule.when(request):
                    return rule.action
            except (KeyError, TypeError, IndexError):
                # 单条规则的条件写错不应该让整个决策挂掉，
                # 跳过它继续匹配下一条。
                continue
        if self._default is not None:
            return self._default
        raise LookupError(f"无规则命中且未设默认动作: kind={request.kind}")


class ScoreBasedPolicy:
    """打分选择：对每个候选动作打分，取最高分。

    军事决策里比"硬规则表"更常见的形式——"哪个方案综合评分最高"。
    评分项可以包括距离、威胁、弹药、时间窗等。
    """

    __slots__ = ("_scorer", "_tie_breaker")

    def __init__(
        self,
        scorer: Callable[[DecisionRequest, str], float],
        tie_breaker: Callable[[DecisionRequest, list[str]], str] | None = None,
    ) -> None:
        self._scorer = scorer
        #: 平分时的裁决函数。默认取选项列表中靠前的——
        #: **不要**用 set 或 dict 迭代顺序，那会破坏可复现性。
        self._tie_breaker = tie_breaker or (lambda req, tied: tied[0])

    def __call__(self, request: DecisionRequest) -> DecisionResponse:
        if not request.options:
            raise ValueError("ScoreBasedPolicy 需要至少一个候选动作")

        best_score = float("-inf")
        best: list[str] = []
        for option in request.options:
            score = self._scorer(request, option)
            if score > best_score:
                best_score = score
                best = [option]
            elif score == best_score:
                best.append(option)

        action = best[0] if len(best) == 1 else self._tie_breaker(request, best)
        return DecisionResponse(
            request_id=request.request_id,
            action=action,
            params={"score": best_score},
            confidence=1.0,
            source=SOURCE_RULE,
        )


class ScriptPolicy:
    """脚本决策：把决策逻辑写成 Python 可调用对象或代码字符串。

    代码字符串会被编译一次后缓存——**不要**在每次决策时重新编译，
    那比决策本身还慢。

    注意安全边界：脚本在本进程内执行，能访问一切。只加载可信来源的脚本。
    若将来要加载外部想定里的脚本，需要换成受限执行环境（独立解释器 + 白名单）。
    """

    __slots__ = ("_fn", "_source")

    def __init__(self, source: str, entry: str = "decide") -> None:
        self._source = source
        namespace: dict[str, Any] = {}
        exec(compile(source, "<script>", "exec"), namespace)  # noqa: S102
        fn = namespace.get(entry)
        if not callable(fn):
            raise ValueError(f"脚本中未找到可调用入口 {entry!r}")
        self._fn = fn

    def __call__(self, request: DecisionRequest) -> "str | DecisionResponse":
        return self._fn(request)
