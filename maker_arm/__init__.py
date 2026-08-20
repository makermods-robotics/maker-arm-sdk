"""maker-arm: Linux/macOS SDK for a RobStride arm and gripper."""

from importlib.metadata import PackageNotFoundError, version

from .arm import Arm, ArmState
from .config import ArmConfig, JointConfig

try:
    __version__ = version("maker-arm")
except PackageNotFoundError:
    __version__ = "0.1.0"
__all__ = ["Arm", "ArmState", "ArmConfig", "JointConfig"]
