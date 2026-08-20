#!/usr/bin/env python3
"""Torque-free Star leader range capture. Move every joint to both usable endpoints;
press Enter to save raw per-servo min/max angles to JSON, or Ctrl-C to discard.
"""

import argparse
import json
import math
import select
import sys
import time
from datetime import datetime


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    ap.add_argument("--out", required=True, help="raw engineering capture output path")
    a = ap.parse_args()
    ids = [int(x) for x in a.star_ids.split(",")]

    from motorbridge_smart_servo import FashionStarServo
    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    mins = {i: math.inf for i in ids}
    maxs = {i: -math.inf for i in ids}
    stdin_ok = sys.stdin.isatty()
    print("Star range capture active. Move every joint gently to both endpoints.")
    try:
        next_print = 0.0
        while True:
            data = bus.sync_monitor(ids)
            for i in ids:
                state = data.get(i)
                if state and state.reliable and math.isfinite(state.angle_deg):
                    mins[i] = min(mins[i], state.angle_deg)
                    maxs[i] = max(maxs[i], state.angle_deg)
            now = time.monotonic()
            if now >= next_print:
                rows = []
                for i in ids:
                    if math.isfinite(mins[i]):
                        rows.append(f"servo {i}: min {mins[i]:+7.1f}° max {maxs[i]:+7.1f}° "
                                    f"travel {maxs[i] - mins[i]:6.1f}°")
                    else:
                        rows.append(f"servo {i}: waiting for reliable data")
                print(" | ".join(rows), flush=True)
                next_print = now + 1.0
            if stdin_ok and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline() == "":
                    stdin_ok = False
                    continue
                break
            time.sleep(0.02)

        joints = []
        for i in ids:
            travel = maxs[i] - mins[i]
            joints.append({"servo": i, "min_deg": round(mins[i], 3),
                           "max_deg": round(maxs[i], 3), "travel_deg": round(travel, 3)})
        with open(a.out, "w") as f:
            json.dump({"timestamp": datetime.now().isoformat(timespec="seconds"),
                       "joints": joints}, f, indent=2)
        print(f"raw Star ranges written to {a.out}")
    except KeyboardInterrupt:
        print("\nno files written")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
