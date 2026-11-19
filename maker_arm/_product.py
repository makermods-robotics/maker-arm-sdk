"""Product identity: names, shipped profile, Star mapping and URDF of this arm. The rest of the
package reads them from here and never names the arm itself."""

from pathlib import Path
from typing import Optional

PACKAGE = "maker_arm"
DIST_NAME = "maker-arm"
CLI_NAME = "maker-arm"
DISPLAY_NAME = "Maker Arm"
MOTOR_VENDOR = "RobStride"

PROFILE_DIR = Path(__file__).resolve().parent / "profiles"
DEFAULT_PROFILE = PROFILE_DIR / "maker_arm_v1.yaml"
DEFAULT_STAR_MAPPING = PROFILE_DIR / "star_to_maker_v1.json"
URDF: Optional[Path] = Path(__file__).resolve().parent / "urdf" / "robot.urdf"

ZERO_POSE = "folded against the base, gripper fully closed"
