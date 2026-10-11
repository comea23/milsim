"""补给件包（§5.18）。"""

from .base import Supply

FRAMEWORK_SUPPLY: tuple[type, ...] = (Supply,)

__all__ = ["FRAMEWORK_SUPPLY", "Supply"]
