#!/usr/bin/env python3
"""Read-only monitor + limit capture: 10Hz polling display in CONNECTED state (not enabled, safe to move by hand).

After manually pushing each joint to both end limits, press Enter: automatically writes the
backed-off lo/hi into the YAML (comments preserved, .bak backup made before writing, auto
rollback if post-write validation fails), and stores the raw record in
configs/limits_capture.json.
Ctrl-C exits without writing anything.
"""

import argparse
import json
import math
import re
import select
import shutil
import sys
import time
from datetime import datetime

from maker_arm.arm import Arm
from maker_arm.config import ArmConfig
from maker_arm.errors import fault_text

MIN_TRAVEL = 0.3  # rad: travel below this is treated as "joint never moved", skip writing it


def compute_limits(mins, maxs, backoff, min_travel=MIN_TRAVEL):
    """Per joint, return (lo, hi) or None (no data / insufficient travel / range collapsed after backoff)."""
    out = []
    for lo_raw, hi_raw in zip(mins, maxs):
        if not (math.isfinite(lo_raw) and math.isfinite(hi_raw)):
            out.append(None)
            continue
        if hi_raw - lo_raw < min_travel:
            out.append(None)
            continue
        lo, hi = round(lo_raw + backoff, 3), round(hi_raw - backoff, 3)
        out.append((lo, hi) if lo < hi else None)
    return out


def update_yaml_limits(text: str, per_motor: dict) -> str:
    """Line by line, replace the lo:/hi: values on the line matching motor_id; comments and formatting are left as-is."""
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        m = re.search(r"motor_id:\s*(\d+)", line)
        if not m or int(m.group(1)) not in per_motor:
            continue
        lo, hi = per_motor[int(m.group(1))]
        line = re.sub(r"(lo:\s*)-?\d+(?:\.\d+)?", lambda g: f"{g.group(1)}{lo}", line, count=1)
        line = re.sub(r"(hi:\s*)-?\d+(?:\.\d+)?", lambda g: f"{g.group(1)}{hi}", line, count=1)
        lines[i] = line
    return "".join(lines)


def write_limits(config_path, json_path, mins, maxs, backoff, joints):
    limits = compute_limits(mins, maxs, backoff)
    per_motor, record_joints = {}, []
    for i, j in enumerate(joints):
        entry = {"joint": i + 1, "motor_id": j.motor_id,
                 "min": round(mins[i], 4) if math.isfinite(mins[i]) else None,
                 "max": round(maxs[i], 4) if math.isfinite(maxs[i]) else None}
        if limits[i] is None:
            entry.update(written=False, reason="no data or insufficient travel (never pushed to both ends?)")
            print(f"⚠️ J{i + 1} (motor {j.motor_id}) skipped: {entry['reason']}")
        else:
            per_motor[j.motor_id] = limits[i]
            entry.update(written=True, lo=limits[i][0], hi=limits[i][1])
            print(f"J{i + 1} (motor {j.motor_id}): lo={limits[i][0]} hi={limits[i][1]}")
        record_joints.append(entry)

    if per_motor:
        bak = config_path + ".bak"
        shutil.copy2(config_path, bak)
        with open(config_path) as f:
            text = f.read()
        with open(config_path, "w") as f:
            f.write(update_yaml_limits(text, per_motor))
        try:
            ArmConfig.from_yaml(config_path)
        except Exception as e:
            shutil.copy2(bak, config_path)
            raise SystemExit(f"post-write validation failed, rolled back from {bak}: {e}")
        print(f"written to {config_path} (backup at {bak})")
    else:
        print("no joint met the write condition, YAML unchanged")

    with open(json_path, "w") as f:
        json.dump({"timestamp": datetime.now().isoformat(timespec="seconds"),
                   "backoff": backoff, "min_travel": MIN_TRAVEL,
                   "joints": record_joints}, f, ensure_ascii=False, indent=2)
    print(f"raw record stored at {json_path}")


def _fmt(x):
    return f"{x:+7.3f}" if math.isfinite(x) else "  ——   "


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--backoff", type=float, default=0.05, help="limit backoff amount, rad")
    ap.add_argument("--json", dest="json_path", default="configs/limits_capture.json")
    a = ap.parse_args()

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    n = arm.config.n_joints
    mins, maxs = [math.inf] * n, [-math.inf] * n
    stdin_ok = sys.stdin.isatty()
    if stdin_ok:
        try:
            import termios
            termios.tcflush(sys.stdin, termios.TCIFLUSH)   # flush any leftover Enter keypresses
        except Exception:
            pass
    else:
        print("⚠️ stdin is not a terminal (e.g. launched via conda run): Enter-to-write is unavailable, monitoring only. "
              "To use limit capture, run directly: ~/miniconda3/envs/maker-arm/bin/python tools/monitor.py ...")
    print("connected (not enabled, safe to move by hand).")
    first = True
    try:
        while True:
            arm.refresh()
            time.sleep(0.1)
            pos, vel = arm.get_joint_positions(), arm.get_joint_velocities()
            tmp, flt = arm.get_temperatures(), arm.get_faults()
            for i, p in enumerate(pos):
                if math.isfinite(p):
                    mins[i] = min(mins[i], p)
                    maxs[i] = max(maxs[i], p)
            rows = [f"motor {arm.config.joints[i].motor_id}: {pos[i]:+7.3f} rad {vel[i]:+6.2f} rad/s "
                    f"{tmp[i]:5.1f}°C min {_fmt(mins[i])} max {_fmt(maxs[i])} "
                    f"travel {maxs[i] - mins[i]:5.3f} {fault_text(flt[i])}"
                    if math.isfinite(mins[i]) else
                    f"motor {arm.config.joints[i].motor_id}: waiting for data..."
                    for i in range(n)]
            rows.append("-- push to both ends (travel >=0.3 required to write). Enter to write and exit; Ctrl-C to exit without writing.")
            if first:
                print("\n".join(rows), flush=True)
                first = False
            else:
                # In-place refresh: move cursor back len(rows) lines, erase and rewrite each line (no clear-screen, doesn't flood the scrollback)
                print(f"\x1b[{len(rows)}F" + "\n".join("\x1b[2K" + r for r in rows), flush=True)
            if stdin_ok and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line == "":
                    # stdin is EOF (e.g. launched via conda run): select always reports readable but reads empty --
                    # this isn't a real keypress, so disable Enter-to-write and keep monitoring, never write by mistake.
                    stdin_ok = False
                    continue
                print()
                write_limits(a.config, a.json_path, mins, maxs, a.backoff, arm.config.joints)
                break
    except KeyboardInterrupt:
        print("\nno files written")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
