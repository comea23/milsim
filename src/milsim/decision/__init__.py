"""决策抽象层。

四种决策模型（行为树 / 状态机 / 脚本 / 大模型）在这里统一到同一个
请求-回调契约下。详见 ``base.py`` 的模块文档。

分层位置：本包只依赖 ``engine``，不依赖 ``services`` 和 ``models``，
因此可以被任何一层引用而不产生循环依赖。
"""

from .base import (
    KIND_ENGAGE,
    KIND_ESCALATE,
    KIND_MANEUVER,
    KIND_PRIORITY,
    KIND_TASK_ASSIGN,
    SOURCE_FALLBACK,
    SOURCE_LLM,
    SOURCE_RULE,
    DecisionError,
    DecisionProvider,
    DecisionRequest,
    DecisionResponse,
    ResolveFn,
)
from .llm import (
    DEFAULT_TIMEOUT_US,
    LLMClient,
    LLMDecisionProvider,
    LLMStats,
    MockLLMClient,
    default_parser,
    default_prompt_builder,
)
from .rule import (
    PolicyFn,
    Rule,
    RuleDecisionProvider,
    RuleTable,
    ScoreBasedPolicy,
    ScriptPolicy,
)

__all__ = [
    "DecisionRequest",
    "DecisionResponse",
    "DecisionProvider",
    "DecisionError",
    "ResolveFn",
    "SOURCE_RULE",
    "SOURCE_LLM",
    "SOURCE_FALLBACK",
    "KIND_TASK_ASSIGN",
    "KIND_MANEUVER",
    "KIND_ENGAGE",
    "KIND_PRIORITY",
    "KIND_ESCALATE",
    "RuleDecisionProvider",
    "RuleTable",
    "Rule",
    "ScoreBasedPolicy",
    "ScriptPolicy",
    "PolicyFn",
    "LLMDecisionProvider",
    "LLMClient",
    "LLMStats",
    "MockLLMClient",
    "default_prompt_builder",
    "default_parser",
    "DEFAULT_TIMEOUT_US",
]
