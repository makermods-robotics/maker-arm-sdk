"""Linux/macOS SDK for a 6-DOF CAN arm plus gripper. Product names live in _product.py.

Optional hardware libraries (OpenCV for cameras, pyusb for gs_usb adapters, the Star leader's
servo library) are imported only when a camera, adapter or leader is opened.
"""

from importlib.metadata import PackageNotFoundError, version

from ._product import DIST_NAME
from .arm import Arm, ArmState
from .cameras import Camera, list_cameras
from .config import ArmConfig, JointConfig
from .errors import ArmError, ConnectError, FaultLatched, ParamTimeout, StateError
from .kinematics import Chain
from .teleop import Teleop

try:
    __version__ = version(DIST_NAME)
except PackageNotFoundError:
    __version__ = "0.1.0"
__all__ = ["Arm", "ArmState", "ArmConfig", "JointConfig", "Chain", "Teleop", "Camera", "list_cameras",
           "ArmError", "ConnectError", "FaultLatched", "ParamTimeout", "StateError"]
