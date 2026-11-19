"""Teleop leaders: anything that yields follower joint targets (``base.Leader``).

``StarLeader`` reads a Star Arm 102 (FashionStar servos); ``ArmLeader`` floats a second arm of the
same kind so it can be moved by hand. Both import their hardware libraries only when connected.
"""

from .arm import ArmLeader
from .base import Leader, LeaderError
from .star import StarLeader

__all__ = ["ArmLeader", "Leader", "LeaderError", "StarLeader"]
