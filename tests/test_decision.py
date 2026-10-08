"""决策接口测试。

重点覆盖三条失败路径（超时、报错、非法输出）和一条竞态
（超时降级与真实响应同时到达）。这四条是异步决策最容易写错、
写错后最难查的地方。
"""

from __future__ import annotations

import threading
import time

import pytest

from milsim.decision import (
    KIND_MANEUVER,
    SOURCE_FALLBACK,
    SOURCE_LLM,
    SOURCE_RULE,
    LLMDecisionProvider,
    MockLLMClient,
    Rule,
    RuleDecisionProvider,
    RuleTable,
    ScoreBasedPolicy,
    ScriptPolicy,
    default_parser,
)
from milsim.decision.base import DecisionRequest
from milsim.engine import SECOND, Engine


def make_request(**overrides) -> DecisionRequest:
    fields = {
        "request_id": 1,
        "sim_time": 0,
        "entity_id": 7,
        "kind": KIND_MANEUVER,
        "situation": "红方一个机械化连正沿公路向东推进，距离我阵地 12 公里。",
        "options": ("advance", "hold", "retreat"),
    }
    fields.update(overrides)
    return DecisionRequest(**fields)


def hold_provider() -> RuleDecisionProvider:
    return RuleDecisionProvider(lambda req: "hold")


def collect(provider, request, timeout: float = 3.0):
    """发起请求并等待回调，返回收到的响应列表。"""
    results: list = []
    done = threading.Event()

    def resolve(response) -> None:
        results.append(response)
        done.set()

    provider.request(request, resolve)
    done.wait(timeout)
    return results


# ---------------------------------------------------------------------------
# 同步规则
# ---------------------------------------------------------------------------

def test_rule_provider_resolves_immediately() -> None:
    provider = RuleDecisionProvider(lambda req: "advance")
    results = collect(provider, make_request())
    assert len(results) == 1
    assert results[0].action == "advance"
    assert results[0].source == SOURCE_RULE


def test_rule_table_picks_highest_weight_match() -> None:
    table = RuleTable(default="hold")
    table.add(Rule("low_ammo", lambda r: r.context.get("ammo", 1) < 0.3,
                   "retreat", weight=10))
    table.add(Rule("near", lambda r: r.context.get("range_km", 99) < 15,
                   "engage", weight=5))

    both = make_request(context={"ammo": 0.1, "range_km": 5})
    assert table(both) == "retreat"       # 权重高的先命中

    only_near = make_request(context={"ammo": 0.9, "range_km": 5})
    assert table(only_near) == "engage"

    neither = make_request(context={"ammo": 0.9, "range_km": 50})
    assert table(neither) == "hold"       # 默认动作


def test_rule_table_survives_broken_condition() -> None:
    """单条规则的条件写错不该拖垮整个决策。"""
    table = RuleTable(default="hold")
    table.add(Rule("broken", lambda r: r.context["missing_key"] == 1, "x",
                   weight=10))
    table.add(Rule("good", lambda r: True, "engage", weight=1))
    assert table(make_request()) == "engage"


def test_score_based_policy_is_deterministic_on_tie() -> None:
    policy = ScoreBasedPolicy(lambda req, opt: 1.0)   # 全部同分
    response = policy(make_request())
    assert response.action == make_request().options[0]   # 取列表首个，不依赖容器顺序


def test_script_policy() -> None:
    policy = ScriptPolicy(
        """
def decide(request):
    return "retreat" if "机械化" in request.situation else "hold"
"""
    )
    assert policy(make_request()) == "retreat"
    assert policy(make_request(situation="无异常")) == "hold"


# ---------------------------------------------------------------------------
# 大模型：正常路径
# ---------------------------------------------------------------------------

def test_llm_success_path() -> None:
    client = MockLLMClient(
        response="ACTION: retreat\nCONFIDENCE: 0.82\nREASON: 对方兵力占优"
    )
    provider = LLMDecisionProvider(client, hold_provider())
    try:
        results = collect(provider, make_request())
        assert len(results) == 1
        assert results[0].action == "retreat"
        assert results[0].source == SOURCE_LLM
        assert 0.8 < results[0].confidence < 0.85
        assert results[0].params["reason"] == "对方兵力占优"
    finally:
        provider.shutdown()


def test_request_returns_immediately() -> None:
    """request() 必须不阻塞——这是整个异步设计的立足点。"""
    client = MockLLMClient(delay_s=0.5)
    provider = LLMDecisionProvider(client, hold_provider())
    try:
        started = time.perf_counter()
        provider.request(make_request(), lambda r: None)
        elapsed = time.perf_counter() - started
        assert elapsed < 0.1, f"request() 阻塞了 {elapsed:.3f}s"
    finally:
        provider.shutdown()


# ---------------------------------------------------------------------------
# 大模型：失败路径
# ---------------------------------------------------------------------------

def test_timeout_falls_back_to_rules() -> None:
    client = MockLLMClient(delay_s=0.3)     # 模型要 300 ms
    provider = LLMDecisionProvider(client, hold_provider())
    try:
        results = collect(
            provider, make_request(deadline_us=50_000)   # 但只给 50 ms
        )
        assert len(results) == 1
        assert results[0].action == "hold"
        assert results[0].source == SOURCE_FALLBACK
        assert provider.stats.timeouts == 1
    finally:
        provider.shutdown()


def test_error_falls_back_to_rules() -> None:
    client = MockLLMClient(fail_times=1)    # 第一次调用抛 ConnectionError
    provider = LLMDecisionProvider(client, hold_provider())
    try:
        results = collect(provider, make_request())
        assert len(results) == 1
        assert results[0].action == "hold"
        assert results[0].source == SOURCE_FALLBACK
        assert provider.stats.errors == 1
    finally:
        provider.shutdown()


def test_invalid_model_output_falls_back() -> None:
    """模型瞎编了一个选项之外的动作，必须被拦截并降级。"""
    client = MockLLMClient(response="ACTION: 核打击\nCONFIDENCE: 0.99")
    provider = LLMDecisionProvider(client, hold_provider())
    try:
        results = collect(provider, make_request())
        assert len(results) == 1
        assert results[0].action == "hold"
        assert results[0].source == SOURCE_FALLBACK
    finally:
        provider.shutdown()


def test_broken_fallback_still_produces_a_response() -> None:
    """连降级都挂了，也必须给出一个响应——请求绝不能悬空。"""
    def exploding(req):
        raise RuntimeError("fallback 内部错误")

    client = MockLLMClient(fail_times=1)
    provider = LLMDecisionProvider(client, RuleDecisionProvider(exploding))
    try:
        results = collect(provider, make_request())
        assert len(results) == 1
        assert results[0].action == "advance"     # 取首个候选
        assert results[0].confidence == 0.0
    finally:
        provider.shutdown()


# ---------------------------------------------------------------------------
# 大模型：竞态
# ---------------------------------------------------------------------------

def test_resolve_called_exactly_once_when_timeout_races_with_late_reply() -> None:
    """超时降级已发出、真实响应随后才到——后者必须被丢弃。

    这是本模块最关键的测试。写错的话，一个决策会被应用两次，
    表现为实体的诡异行为（比如先撤退然后又执行推进）。
    """
    client = MockLLMClient(
        delay_s=0.3, response="ACTION: retreat\nCONFIDENCE: 0.9"
    )
    provider = LLMDecisionProvider(client, hold_provider())
    try:
        calls: list = []
        done = threading.Event()

        def resolve(response) -> None:
            calls.append(response)
            done.set()

        provider.request(make_request(deadline_us=50_000), resolve)
        assert done.wait(2.0), "超时降级没有触发"

        time.sleep(0.5)      # 等晚到的真实响应也走完

        assert len(calls) == 1, f"回调了 {len(calls)} 次，应当恰好 1 次"
        assert calls[0].action == "hold"
        assert calls[0].source == SOURCE_FALLBACK
    finally:
        provider.shutdown()


def test_many_concurrent_requests_each_resolve_once() -> None:
    client = MockLLMClient(delay_s=0.02, response="ACTION: advance\nCONFIDENCE: 0.7")
    provider = LLMDecisionProvider(client, hold_provider(), max_workers=8)
    try:
        lock = threading.Lock()
        seen: dict[int, int] = {}
        done = threading.Event()

        def make_resolver(rid: int):
            def resolve(response) -> None:
                with lock:
                    seen[rid] = seen.get(rid, 0) + 1
                    if len(seen) == 50:
                        done.set()
            return resolve

        for rid in range(50):
            provider.request(
                make_request(request_id=rid, options=("advance", "hold")),
                make_resolver(rid),
            )

        assert done.wait(5.0), f"只完成了 {len(seen)}/50"
        assert set(seen) == set(range(50))
        assert all(count == 1 for count in seen.values())
        assert provider.stats.succeeded == 50
    finally:
        provider.shutdown()


# ---------------------------------------------------------------------------
# 解析器
# ---------------------------------------------------------------------------

def test_parser_accepts_option_index() -> None:
    assert default_parser("ACTION: 2\nCONFIDENCE: 0.6", make_request()).action == "hold"


def test_parser_is_case_insensitive_and_trims_punctuation() -> None:
    assert default_parser('ACTION: "Retreat"。', make_request()).action == "retreat"


def test_parser_rejects_unknown_action() -> None:
    with pytest.raises(ValueError, match="非法动作"):
        default_parser("ACTION: nuclear_strike", make_request())


def test_parser_rejects_missing_action_field() -> None:
    with pytest.raises(ValueError, match="ACTION"):
        default_parser("我觉得应该撤退", make_request())


# ---------------------------------------------------------------------------
# 端到端：引擎 ← 大模型
# ---------------------------------------------------------------------------

def test_decision_flows_back_into_engine_through_inbox() -> None:
    """端到端串联：引擎推进 → 大模型异步返回 → 经 inbox 回投 → 引擎应用。

    这正是真实 Commander 的接线方式，验证三块能不能咬合。
    """
    engine = Engine()
    engine.start()
    applied: list[tuple[int, str]] = []

    client = MockLLMClient(
        delay_s=0.05, response="ACTION: advance\nCONFIDENCE: 0.9"
    )
    provider = LLMDecisionProvider(client, hold_provider())

    try:
        def on_decision(response) -> None:
            # 此刻在 LLM 工作线程里！只能投递，绝不能直接改仿真状态。
            engine.inbox.post(
                lambda: applied.append((engine.now, response.action))
            )

        provider.request(make_request(), on_decision)

        engine.run_to(2 * SECOND)      # 主循环照常推进，不等模型
        time.sleep(0.2)                # 模型在这段真实时间里返回了
        engine.run_to(4 * SECOND)      # inbox 里的动作在这一步开头生效

        # 生效时刻是 2 s 而不是 4 s——外部投递的时间戳是"处理的当下"，
        # 而 run_to 返回后引擎停在 2 s，本次 step 的 drain 就发生在 2 s。
        # 决策不会凭空推迟到下一个仿真时刻，这正是我们要的语义。
        assert applied == [(2 * SECOND, "advance")]
    finally:
        provider.shutdown()
