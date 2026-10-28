"""Read-only environment and profile checks for Linux and macOS.

doctor never talks to the motors; run `maker-arm scan` next to find them.
"""

from __future__ import annotations

import argparse
import platform
import shutil
import subprocess

from maker_arm.cli.common import default_backend, slcan_candidates
from maker_arm.config import ArmConfig
from maker_arm.mapping import JointMapper
from maker_arm.profiles import DEFAULT_ARM_CONFIG, DEFAULT_STAR_MAPPING


def port_holders(port: str) -> list[str]:
    """Processes holding a serial device open (another program blocks the SDK)."""
    if not shutil.which("lsof"):
        return []
    try:
        out = subprocess.run(["lsof", "-Fc", port], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return sorted({line[1:] for line in out.splitlines() if line.startswith("c")})


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
    print(f"arm profile: {a.config} ({config.n_joints} joints, MIT protocol, valid)")
    print(f"Star map:   {a.map_path} (valid)")

    candidates = slcan_candidates()
    print("USB CAN adapter candidates:", ", ".join(candidates) if candidates else "none")
    for port in candidates:
        holders = port_holders(port)
        if holders:
            print(f"  {port} is open in: {', '.join(holders)} -- close it before using the SDK")
    if system == "Linux":
        print("ip command:", shutil.which("ip") or "missing")
        print("slcand:    ", shutil.which("slcand") or "missing (only needed for serial SLCAN)")
        print("candump:   ", shutil.which("candump") or "missing (install can-utils)")

    try:
        import motorbridge_smart_servo  # noqa: F401
    except ImportError:
        print("Star dependency: missing motorbridge-smart-servo")
    else:
        print("Star dependency: available")

    backend = default_backend()
    hint = "maker-arm scan" if backend == "socketcan" or len(candidates) == 1 else "maker-arm scan --port <adapter>"
    print(f"\ndoctor does not contact the motors. Next: {hint}")
