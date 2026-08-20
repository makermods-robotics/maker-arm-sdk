"""Read-only environment and profile checks for Linux and macOS."""

from __future__ import annotations

import argparse
import glob
import platform
import shutil

from maker_arm.config import ArmConfig
from maker_arm.mapping import JointMapper
from maker_arm.profiles import DEFAULT_ARM_CONFIG, DEFAULT_STAR_MAPPING


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--map", dest="map_path", default=str(DEFAULT_STAR_MAPPING))
    a = ap.parse_args()

    system = platform.system()
    print(f"platform: {system} {platform.release()}")
    if system not in {"Linux", "Darwin"}:
        print("unsupported platform: maker-arm releases support Linux and macOS only")

    config = ArmConfig.from_yaml(a.config)
    JointMapper.from_json(a.map_path)
    print(f"arm profile: {a.config} ({config.n_joints} joints, valid)")
    print(f"Star map:   {a.map_path} (valid)")

    if system == "Darwin":
        follower = sorted(glob.glob("/dev/cu.usbmodem*"))
        serial = sorted(glob.glob("/dev/cu.usbserial*"))
        print("SLCAN candidates:", ", ".join(follower) if follower else "none")
        print("serial candidates:", ", ".join(serial) if serial else "none")
    elif system == "Linux":
        print("ip command:", shutil.which("ip") or "missing")
        print("slcand:    ", shutil.which("slcand") or "missing (only needed for serial SLCAN)")
        print("candump:   ", shutil.which("candump") or "missing (install can-utils)")

    try:
        import motorbridge_smart_servo  # noqa: F401
    except ImportError:
        print("Star dependency: missing motorbridge-smart-servo")
    else:
        print("Star dependency: available")
