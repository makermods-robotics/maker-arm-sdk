#!/usr/bin/env python3
"""Which physical arm is on which adapter: wiggle one gripper at a time and watch the cameras.

For each arm (the rig's arms, --arm names, or every gs_usb adapter plugged in) only the gripper is
energized, at low gain, and opened a few degrees and back a few times. Camera frames taken at every
closed and open position say which wrist camera moved with it and on which side of the top camera
the motion was. Only change that follows every cycle counts, so people moving in view do not.
--assign then names the arms left/right by that half (--mirror when the top camera faces the
operator) and records the wrist cameras as <name>_wrist in the rig file. Headless use needs --yes.
"""

from __future__ import annotations

import math
import sys
import time
from contextlib import ExitStack

import numpy as np

from ..rig import ArmSpec
from .check import choose_test_delta
from .common import (CliError, Interrupt, Target, add_yes_arg, confirm, joint_position, load_rig, make_parser,
                     run, targets)
from . import ports
from ..bus import release_motor

MAX_KP = 20.0       # the gripper's gain during the wiggle (lower if the profile's is lower)
STEPS = 40          # ramp steps each way
STEP_S = 0.025
CYCLES = 3          # open/close repeats
PIXEL = 12.0        # grey-level change of one pixel that counts as moved
BLOCK = 16          # pixels per side of the blocks the change is averaged over
MOTION = 10.0       # block score that counts as the gripper having moved in view


def candidates(a) -> list[Target]:
    if a.arm or any(getattr(a, k) for k in ("backend", "channel", "port")):
        return targets(a, many=True)
    rig = load_rig(a)
    if rig.arms:
        return [Target(n, s, rig.base, a.config) for n, s in rig.arms.items()]
    adapters, error = ports.gs_usb_adapters()
    if not adapters:
        raise CliError(f"no arms in the rig file and no gs_usb adapter found{': ' + error if error else ''}")
    return [Target(ad["port"], ArmSpec("gs_usb", port=ad["port"]), profile=a.config) for ad in adapters]


# ── the wiggle: one motor, by hand, bounded ──
def hold_for(m, target: float, kp: float, kd: float, seconds: float, stop=None) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if stop is not None and stop.is_set():
            raise CliError("interrupted")
        m.send_command(target, 0.0, kp, kd, 0.0)
        time.sleep(STEP_S)


def wiggle(arm, index: int, degrees: float, shot, stop=None) -> dict:
    """Energize only motor ``index`` (its profile watchdog armed), open it ``degrees`` and back
    CYCLES times, torque off (confirmed). ``shot("open"|"closed")`` is called at each end.
    ``stop`` (an Event) cuts the wiggle short; the release always runs."""
    j, m = arm.config.joints[index], arm.motors[index]
    status = m.check()
    if status.has_fault:
        raise CliError(f"{j.label} reports a fault ({status.text}); not energizing it")
    fb = m.read_state()
    limit = j.enable_temperature_c if j.enable_temperature_c is not None else j.max_temperature_c
    if limit is not None and fb.temperature >= limit:
        raise CliError(f"{j.label} is {fb.temperature:.1f} C; needs below {limit:.0f} C")
    if fb.enabled:
        raise CliError(f"{j.label} already has torque on; release the arm first")
    start = fb.position
    delta = choose_test_delta(joint_position(j, start), j.lo, j.hi, math.radians(degrees)) * j.direction
    kp, kd = min(j.kp, MAX_KP), j.kd
    if stop is not None and stop.is_set():
        raise CliError("interrupted")
    token, peak = None, math.nan
    try:
        if j.can_timeout_ms is not None:
            token = m.arm_watchdog(j.can_timeout_ms, arm.PARAM_TIMEOUT)
        m.send_command(start, 0.0, 0.0, 0.0, 0.0)   # neutral preload: never resume an old setpoint
        time.sleep(0.01)
        m.enable()
        for _ in range(CYCLES):
            for k in range(1, STEPS + 1):
                hold_for(m, start + delta * k / STEPS, kp, kd, STEP_S, stop)
            hold_for(m, start + delta, kp, kd, 0.3, stop)
            peak = m.feedback.position if m.feedback else math.nan
            shot("open")
            for k in range(STEPS - 1, -1, -1):
                hold_for(m, start + delta * k / STEPS, kp, kd, STEP_S, stop)
            hold_for(m, start, kp, kd, 0.3, stop)
            shot("closed")
    finally:
        released = release_motor(m)
        if released and token is not None:   # only on a motor confirmed off
            m.restore_watchdog(token, arm.PARAM_TIMEOUT)
    return {"joint": j.label, "commanded": abs(delta), "moved": abs(peak - start), "released": released}


# ── cameras ──
def grey(frame) -> np.ndarray:
    f = np.asarray(frame, dtype=float)
    return f.mean(axis=2) if f.ndim == 3 else f


def change(closed: list, opened: list) -> tuple[float, float]:
    """Strongest local change that follows the wiggle: (score, x position as a fraction of the
    width). ``closed`` has one more frame than ``opened`` (before, then after each cycle). A pixel
    scores only as much as it changed on its weakest cycle, less what it failed to return, and
    the score is averaged over BLOCK-pixel blocks so the gripper stands out over sensor noise."""
    closed, opened = [grey(f) for f in closed], [grey(f) for f in opened]
    score = None
    for i, o in enumerate(opened):
        cycle = np.clip(np.abs(o - closed[i]) - np.abs(closed[i + 1] - closed[i]) - PIXEL, 0.0, None)
        score = cycle if score is None else np.minimum(score, cycle)
    rows, cols = score.shape[0] // BLOCK, score.shape[1] // BLOCK
    blocks = score[:rows * BLOCK, :cols * BLOCK].reshape(rows, BLOCK, cols, BLOCK).mean(axis=(1, 3))
    row, col = np.unravel_index(np.argmax(blocks), blocks.shape)
    return float(blocks[row, col]), (col + 0.5) / cols


def grab(cams: dict) -> dict:
    return {name: cam.latest() for name, cam in cams.items()}


def judge(closed: list[dict], opened: list[dict], top: str | None, mirror: bool) -> dict:
    """Motion per camera, the wrist camera that moved, the side of the top camera it moved on."""
    seen = [n for n in closed[0] if all(f.get(n) is not None for f in closed + opened)]
    found = {n: change([f[n] for f in closed], [f[n] for f in opened]) for n in seen} if opened else {}
    motion = {n: score for n, (score, _) in found.items()}
    wrist = sorted(((v, n) for n, v in motion.items() if n != top), reverse=True)
    wrist_camera = None
    if wrist and wrist[0][0] > MOTION and (len(wrist) == 1 or wrist[0][0] > 2 * wrist[1][0]):
        wrist_camera = wrist[0][1]
    side = None
    if top in found and motion[top] > MOTION:
        side = "left" if (found[top][1] < 0.5) != mirror else "right"
    return {"motion": motion, "wrist_camera": wrist_camera, "side": side}


def assign(rig, found: list[dict], out) -> dict:
    names: dict[str, str] = {}
    for f in found:
        name = f.get("side")
        if name is None and sys.stdin.isatty():
            sys.stderr.write(f"name for the arm on {f['address']} (Enter skips): ")
            name = input().strip() or None
        if name:
            names[f["candidate"]] = name
    clashes = {n for n in names.values() if list(names.values()).count(n) > 1}
    if clashes:
        raise CliError(f"several arms came out as {sorted(clashes)}; nothing written. Check the top camera "
                       "and that only the wiggled gripper moved")
    renamed = {f["candidate"]: f for f in found if f["candidate"] in names}
    keep = {n: s for n, s in rig.arms.items() if n not in renamed and n not in names.values()}
    for old, f in renamed.items():
        keep[names[old]] = f["spec"]
        if f.get("wrist_camera_id"):
            rig.cameras[f"{names[old]}_wrist"] = f["wrist_camera_id"]
    rig.arms = keep
    rig.leaders = {n: s for n, s in rig.leaders.items() if n in rig.arms and (s.arm is None or s.arm in rig.arms)}
    rig.save()
    out.say(f"saved {rig.path}: " + ", ".join(f"{names[o]} = {f['address']}" for o, f in renamed.items()))
    return {renamed[o]["address"]: n for o, n in names.items()}


def body(a, out) -> dict:
    if not 2 <= a.degrees <= 30:
        raise CliError("--degrees must be in [2, 30]")
    rig = load_rig(a)
    chosen = candidates(a)
    camera_names = [] if a.no_cameras else (a.camera or list(rig.cameras))
    top = a.top or ("top" if "top" in camera_names else None)
    out.say(f"will energize ONLY the gripper of {', '.join(t.label for t in chosen)}, one at a time, "
            f"and open it {a.degrees:g} deg and back (low gain)")
    confirm(a, out, "energize the grippers?")
    found = []
    with ExitStack() as stack:
        stop = stack.enter_context(Interrupt())
        cams = {n: stack.enter_context(rig.camera(n)) for n in camera_names}
        for t in chosen:
            if stop.event.is_set():
                break
            entry = {"candidate": t.name, "address": t.spec.address, "spec": t.spec}
            arm = t.build()
            try:
                arm.connect()
                index = arm.joint_names.index("gripper") if "gripper" in arm.joint_names else len(arm.motors) - 1
                shots = {"closed": [grab(cams)], "open": []}
                entry.update(wiggle(arm, index, a.degrees, lambda pose: shots[pose].append(grab(cams)),
                                    stop.event))
                entry.update(judge(shots["closed"], shots["open"], top, a.mirror))
                if entry["wrist_camera"]:
                    entry["wrist_camera_id"] = rig.camera_id(entry["wrist_camera"])
            except Exception as e:   # one adapter without an arm must not stop the others
                entry["error"] = str(e)
            finally:
                arm.disconnect()
            out.say(f"{t.name} ({t.spec.address}): " + (f"ERROR {entry['error']}" if "error" in entry else
                    f"gripper moved {math.degrees(entry['moved']):.1f} deg; wrist camera: "
                    f"{entry['wrist_camera'] or '?'}; top camera half: {entry['side'] or '?'}"))
            found.append(entry)
    result = {"arms": [{k: v for k, v in f.items() if k != "spec"} for f in found]}
    if a.assign:
        result["assigned"] = assign(rig, [f for f in found if "error" not in f], out)
    result["ok"] = all("error" not in f and f.get("released") for f in found)
    return result


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    ap.add_argument("--degrees", type=float, default=15.0, help="gripper opening, degrees (default 15)")
    ap.add_argument("--camera", action="append", metavar="NAME|ID", help="cameras to watch (default: the rig's)")
    ap.add_argument("--top", metavar="NAME", help="the overhead camera (default: the rig camera named top)")
    ap.add_argument("--no-cameras", action="store_true", help="only wiggle; watch the arms yourself")
    ap.add_argument("--mirror", action="store_true", help="the top camera faces the operator: swap left/right")
    ap.add_argument("--assign", action="store_true", help="write the names (and wrist cameras) into the rig file")
    add_yes_arg(ap)
    run("identify", ap.parse_args(), body)


if __name__ == "__main__":
    main()
