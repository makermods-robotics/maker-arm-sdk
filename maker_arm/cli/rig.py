"""Show or edit the rig file: the arms on this bench, its cameras and the teleop leaders.

  rig show
  rig init [--detect] [--force]        start a file; --detect adds every gs_usb adapter and camera
  rig set-arm left --port gs_usb:<serial>  [--profile P]     (or --backend socketcan --channel can1)
  rig set-camera top usb:<path>
  rig set-leader left --type star --star-port /dev/ttyUSB0 [--map M]
  rig set-leader right --type arm --leader-arm spare
Each set-* takes --remove. No device is opened.
"""

from __future__ import annotations

from ..rig import ArmSpec, LeaderSpec, Rig, RigError
from .common import CliError, add_rig_arg, explicit_spec, make_parser, run
from . import ports


def describe(rig: Rig, say) -> None:
    say(f"rig file: {rig.path}" + ("" if rig.path and rig.path.exists() else " (not created yet)"))
    say("arms:" + ("" if rig.arms else " none"))
    for name, spec in rig.arms.items():
        say(f"  {name:<12} {spec.backend:<9} {spec.address:<40} profile: {spec.profile or 'shipped'}")
    say("cameras:" + ("" if rig.cameras else " none"))
    for name, cam in rig.cameras.items():
        say(f"  {name:<12} {cam}")
    say("leaders:" + ("" if rig.leaders else " none"))
    for name, leader in rig.leaders.items():
        source = leader.port if leader.type == "star" else f"arm {leader.arm}"
        say(f"  {name:<12} {leader.type:<5} {source}" + (f" map {leader.map}" if leader.map else ""))


def init(rig: Rig, a, out) -> None:
    if rig.path.exists() and not a.force:
        raise CliError(f"{rig.path} exists; pass --force to start over")
    rig.arms, rig.cameras, rig.leaders = {}, {}, {}
    if a.detect:
        adapters, error = ports.gs_usb_adapters()
        if error:
            out.warn(f"gs_usb adapters not listed: {error}")
        for i, ad in enumerate(adapters, 1):
            rig.arms[f"arm{i}"] = ArmSpec("gs_usb", port=ad["port"])
        for i, cam in enumerate(ports.camera_list()[0], 1):
            rig.cameras[f"camera{i}"] = cam["id"]
        if rig.arms:
            out.say("arms are named arm1, arm2, ...: run 'identify --assign' to name them by position")


def set_arm(rig: Rig, a) -> None:
    if a.remove:
        rig.spec(a.name)
        del rig.arms[a.name]
        rig.leaders = {n: s for n, s in rig.leaders.items() if n != a.name and s.arm != a.name}
        return
    spec = explicit_spec(a)
    if spec is None:
        raise CliError("set-arm needs --backend/--channel/--port")
    spec.profile = a.profile or (rig.arms[a.name].profile if a.name in rig.arms else None)
    rig.arms[a.name] = spec


def set_camera(rig: Rig, a) -> None:
    if a.remove:
        rig.camera_id(a.name)
        rig.cameras.pop(a.name)
        return
    if not a.id or a.id.split(":", 1)[0] not in ("usb", "path", "index"):
        raise CliError("set-camera needs a camera id: usb:<path>, path:<path> or index:N (see the ports command)")
    rig.cameras[a.name] = a.id


def set_leader(rig: Rig, a) -> None:
    if a.remove:
        if rig.leaders.pop(a.name, None) is None:
            raise CliError(f"no leader for {a.name!r}")
        return
    if a.type is None:
        raise CliError("set-leader needs --type star|arm")
    rig.leaders[a.name] = LeaderSpec(a.type, port=a.star_port, map=a.map, arm=a.leader_arm)


def body(a, out) -> dict:
    rig = Rig.load(a.rig)
    if a.action != "show":
        try:
            {"init": lambda: init(rig, a, out), "set-arm": lambda: set_arm(rig, a),
             "set-camera": lambda: set_camera(rig, a), "set-leader": lambda: set_leader(rig, a)}[a.action]()
            rig.save()
        except RigError as e:
            raise CliError(f"not saved: {e}") from e
        out.say(f"saved {rig.path}")
    describe(rig, out.say)
    return {"path": str(rig.path), "rig": rig.to_dict()}


def main() -> None:
    ap = make_parser(__doc__, arms=None)
    add_rig_arg(ap)
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("show")
    p = sub.add_parser("init")
    p.add_argument("--detect", action="store_true", help="add every gs_usb adapter and camera plugged in now")
    p.add_argument("--force", action="store_true", help="replace an existing file")
    p = sub.add_parser("set-arm")
    p.add_argument("name")
    p.add_argument("--backend", choices=("socketcan", "slcan", "gs_usb"))
    p.add_argument("--channel")
    p.add_argument("--port")
    p.add_argument("--serial-baudrate", type=int)
    p.add_argument("--slcan-startup-delay", type=float)
    p.add_argument("--profile", help="arm profile YAML (default: the shipped one)")
    p.add_argument("--remove", action="store_true")
    p = sub.add_parser("set-camera")
    p.add_argument("name")
    p.add_argument("id", nargs="?", help="usb:<path>, path:<path> or index:N")
    p.add_argument("--remove", action="store_true")
    p = sub.add_parser("set-leader")
    p.add_argument("name", help="the follower arm this leader drives")
    p.add_argument("--type", choices=("star", "arm"))
    p.add_argument("--star-port")
    p.add_argument("--map", help="Star mapping JSON (default: the shipped one)")
    p.add_argument("--leader-arm", help="another arm in the rig, floated by hand")
    p.add_argument("--remove", action="store_true")
    for p in sub.choices.values():   # so --json/--rig also work after the action
        p.add_argument("--json", action="store_true", default=None)
        p.add_argument("--rig", default=None)
    a = ap.parse_args()
    run("rig", a, body)


if __name__ == "__main__":
    main()
