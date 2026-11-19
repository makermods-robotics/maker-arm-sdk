"""Read-only checks of the computer, the profiles and the rig file, for Linux and macOS.

doctor never talks to the motors; run `scan` next to find them.
"""

from __future__ import annotations

import os
import platform
import shutil
import subprocess

from .._product import CLI_NAME, DEFAULT_PROFILE, DEFAULT_STAR_MAPPING
from ..config import ArmConfig
from ..mapping import JointMapper
from ..rig import Rig, RigError, default_backend, default_path
from .common import add_rig_arg, make_parser, run
from .ports import discover, print_discovery


def port_holders(port: str) -> list[str]:
    """Processes holding a serial device open (another program blocks the SDK)."""
    if not shutil.which("lsof"):
        return []
    try:
        out = subprocess.run(["lsof", "-Fc", port], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return sorted({line[1:] for line in out.splitlines() if line.startswith("c")})


def check_rig(path, found: dict) -> dict:
    """Is the rig file valid, and is everything it names plugged in?"""
    try:
        rig = Rig.load(path)
    except RigError as e:
        return {"path": str(path or default_path()), "exists": True, "valid": False, "problems": [str(e)]}
    problems = []
    adapters = {ad["port"] for ad in found["gs_usb"]}
    interfaces = {itf["name"] for itf in found["socketcan"]}
    camera_ids = {cam["id"] for cam in found["cameras"]}
    for name, spec in rig.arms.items():
        try:
            spec.config(rig.base)
        except (OSError, ValueError) as e:
            problems.append(f"arm {name}: profile does not load ({e})")
        if spec.backend == "gs_usb" and spec.port and not found["gs_usb_error"] and spec.port not in adapters:
            problems.append(f"arm {name}: adapter {spec.port} is not plugged in")
        if spec.backend == "socketcan" and spec.address not in interfaces:
            problems.append(f"arm {name}: SocketCAN interface {spec.address} does not exist")
        if spec.backend == "slcan" and spec.port and not os.path.exists(spec.port):
            problems.append(f"arm {name}: serial device {spec.port} does not exist")
    for name, cam in rig.cameras.items():
        if cam.startswith("usb:") and not found["cameras_error"] and cam not in camera_ids:
            problems.append(f"camera {name}: {cam} is not connected")
    for name, leader in rig.leaders.items():
        if leader.type == "star" and not os.path.exists(leader.port):
            problems.append(f"leader for {name}: serial port {leader.port} does not exist")
    return {"path": str(rig.path), "exists": rig.path.exists(), "valid": True, "arms": rig.arm_names,
            "cameras": list(rig.cameras), "leaders": list(rig.leaders), "problems": problems}


def body(a, out) -> dict:
    say = out.say
    system = platform.system()
    say(f"platform: {system} {platform.release()}")
    if system not in {"Linux", "Darwin", "Windows"}:
        say(f"unsupported platform: {CLI_NAME} supports Linux, macOS and Windows")

    config = ArmConfig.from_yaml(a.config)
    JointMapper.from_json(a.map_path)
    say(f"arm profile: {a.config} ({config.n_joints} joints, MIT protocol, valid)")
    say(f"Star map:    {a.map_path} (valid)")

    found = discover()
    print_discovery(found, say)
    for sp in found["serial"]:
        holders = port_holders(sp["device"])
        sp["held_by"] = holders
        if holders:
            say(f"  {sp['device']} is open in: {', '.join(holders)} -- close it before using the SDK")
    tools = {}
    if system == "Linux":
        tools = {name: shutil.which(name) for name in ("ip", "slcand", "candump")}
        say("tools:      " + ", ".join(f"{k} {'ok' if v else 'missing'}" for k, v in tools.items()))
    try:
        import motorbridge_smart_servo  # noqa: F401
        star = True
    except ImportError:
        star = False
    say(f"Star dependency: {'available' if star else 'missing motorbridge-smart-servo'}")

    rig = check_rig(a.rig, found)
    if not rig["valid"]:
        say(f"rig file:    {rig['path']}: INVALID -- {rig['problems'][0]}")
    elif not rig["exists"]:
        say(f"rig file:    {rig['path']} (none yet; '{CLI_NAME} rig init' creates one)")
    else:
        say(f"rig file:    {rig['path']} (arms: {', '.join(rig['arms']) or 'none'}; "
            f"cameras: {', '.join(rig['cameras']) or 'none'})")
        for problem in rig["problems"]:
            say(f"  {problem}")

    if rig.get("arms"):
        hint = f"{CLI_NAME} scan --arm {rig['arms'][0]}"
    elif found["gs_usb"]:
        hint = f"{CLI_NAME} scan --port {found['gs_usb'][0]['port']}"
    elif default_backend() == "socketcan" or len(found["serial"]) == 1:
        hint = f"{CLI_NAME} scan"
    else:
        hint = f"{CLI_NAME} scan --port <adapter>"
    say(f"\ndoctor does not contact the motors. Next: {hint}")
    return {"platform": f"{system} {platform.release()}", "profile": a.config, "star_map": a.map_path,
            "star_dependency": star, "tools": tools, "rig": rig, **found}


def main() -> None:
    ap = make_parser(__doc__, arms=None)
    ap.add_argument("--config", default=str(DEFAULT_PROFILE))
    ap.add_argument("--map", dest="map_path", default=str(DEFAULT_STAR_MAPPING))
    add_rig_arg(ap)
    run("doctor", ap.parse_args(), body)


if __name__ == "__main__":
    main()
