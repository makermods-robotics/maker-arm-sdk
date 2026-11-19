#!/usr/bin/env python3
"""Cameras: list them, save a snapshot from each, or measure frame rate and latency.

  cameras list
  cameras snapshot [--camera NAME|ID ...] [--out DIR]     default: every rig camera, else every camera
  cameras stream [--camera ...] [--seconds 5]             fps and frame age per camera; no window
"""

from __future__ import annotations

import time
from contextlib import ExitStack
from pathlib import Path

from .common import CliError, add_rig_arg, load_rig, make_parser, run
from . import ports


def chosen(a, rig) -> list[tuple[str, str]]:
    """(label, id) for --camera values, else the rig's cameras, else every camera found."""
    if a.camera:
        return [(c, rig.camera_id(c)) for c in a.camera]
    if rig.cameras:
        return list(rig.cameras.items())
    found, error = ports.camera_list()
    if not found:
        raise CliError(f"no cameras found{': ' + error if error else ''}")
    return [(c["id"], c["id"]) for c in found]


def safe_name(label: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in label)


def snapshot(a, out, rig) -> dict:
    folder = Path(a.out)
    folder.mkdir(parents=True, exist_ok=True)
    saved = {}
    for label, cam_id in chosen(a, rig):
        with rig.camera(cam_id) as cam:
            path = cam.snapshot(folder / f"{safe_name(label)}.jpg")
        out.say(f"{label}: {path}")
        saved[label] = str(path)
    return {"snapshots": saved}


def stream(a, out, rig) -> dict:
    cams = chosen(a, rig)
    stats = {label: {"frames": 0, "age_sum": 0.0, "max_age": 0.0} for label, _ in cams}
    with ExitStack() as stack:
        opened = {label: stack.enter_context(rig.camera(cam_id)) for label, cam_id in cams}
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds:
            for label, cam in opened.items():
                try:
                    _, stamp = cam.read(timeout=0.2)
                except TimeoutError:
                    continue
                age = max(0.0, time.time() - stamp)
                s = stats[label]
                s["frames"] += 1
                s["age_sum"] += age
                s["max_age"] = max(s["max_age"], age)
        elapsed = time.monotonic() - t0
    result = {}
    for label, s in stats.items():
        fps = s["frames"] / elapsed
        mean_age = s["age_sum"] / s["frames"] if s["frames"] else None
        result[label] = {"fps": fps, "frames": s["frames"], "mean_age_s": mean_age, "max_age_s": s["max_age"]}
        out.say(f"{label:<24} {fps:5.1f} fps  frame age mean "
                f"{(mean_age or 0) * 1000:5.1f} ms, max {s['max_age'] * 1000:5.1f} ms")
    return {"ok": all(r["frames"] for r in result.values()), "seconds": elapsed, "cameras": result}


def body(a, out) -> dict:
    rig = load_rig(a)
    if a.action == "list":
        found, error = ports.camera_list()
        names = {v: k for k, v in rig.cameras.items()}
        for c in found:
            out.say(f"{c['id']:<48} {c['path'] or '':<12} {c['name']}" +
                    (f"  (rig: {names[c['id']]})" if c["id"] in names else ""))
        if not found:
            out.say(f"no cameras found{': ' + error if error else ''}")
        return {"cameras": [dict(c, rig_name=names.get(c["id"])) for c in found]}
    return snapshot(a, out, rig) if a.action == "snapshot" else stream(a, out, rig)


def main() -> None:
    ap = make_parser(__doc__, arms=None)
    ap.add_argument("action", choices=("list", "snapshot", "stream"))
    ap.add_argument("--camera", action="append", metavar="NAME|ID", help="rig camera name or camera id (repeatable)")
    ap.add_argument("--out", default=".", help="snapshot: directory for the JPEGs (default: here)")
    ap.add_argument("--seconds", type=float, default=5.0, help="stream: how long to measure")
    add_rig_arg(ap)
    run("cameras", ap.parse_args(), body)


if __name__ == "__main__":
    main()
