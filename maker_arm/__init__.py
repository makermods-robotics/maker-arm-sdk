"""maker-arm: SDK for a 6x RobStride RS00 robot arm."""

from .arm import Arm, ArmState
from .config import ArmConfig, JointConfig

__version__ = "0.1.0"
__all__ = ["Arm", "ArmState", "ArmConfig", "JointConfig"]
