"""发射件包（§5.18）。"""

from .base import WEAPON_CLASSES, Weapon

FRAMEWORK_WEAPONS: tuple[type, ...] = (Weapon,)

__all__ = ["FRAMEWORK_WEAPONS", "WEAPON_CLASSES", "Weapon"]
