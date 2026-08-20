#!/usr/bin/env python3
"""Powered response check for every configured motor, with all joints held together.

Each motor receives one small, soft-limit-safe joint-coordinate jog while the other motors
hold their positions. The tool measures actual encoder displacement, returns the tested
joint to its starting pose, prints a pass/fail summary, then waits before releasing torque.
"""

from __future__ import annotations

import argparse
import math
import time

from maker_arm.arm import Arm, ArmState
from maker_arm.cli.safety import require_interactive_terminal, wait_for_release
from maker_arm.profiles import DEFAULT_ARM_CONFIG


def choose_test_delta(position: float, lo: float, hi: float, magnitude: float) -> float:
    """Choose a signed delta that points into the configured range and stays within it."""
    if not all(math.isfinite(x) for x in (position, lo, hi, magnitude)) or magnitude <= 0:
        raise ValueError("position, limits, and positive magnitude must be finite")
    plus_room = hi - position
    minus_room = position - lo
    if plus_room >= magnitude and plus_room >= minus_room:
        return magnitude
    if minus_room >= magnitude:
        return -magnitude
    if plus_room >= magnitude:
        return magnitude
    raise ValueError(f"position {position:+.3f} has less than {magnitude} rad room inside [{lo}, {hi}]")


def wait_and_measure(arm: Arm, index: int, start: float, signed_delta: float,
                     seconds: float) -> tuple[float, float]:
    direction = 1.0 if signed_delta > 0 else -1.0
    peak = -math.inf
    final = math.nan
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and arm.state is ArmState.ENABLED:
        final = arm.get_joint_positions()[index]
        if math.isfinite(final):
            peak = max(peak, (final - start) * direction)
        time.sleep(0.02)
    return max(0.0, peak) if math.isfinite(peak) else math.nan, final


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--serial-baudrate", type=int,
                    help="SLCAN serial rate (default 115200)")
    ap.add_argument("--slcan-startup-delay", type=float, default=2.5)
    ap.add_argument("--delta", type=float, default=0.05, help="commanded test motion, rad")
    ap.add_argument("--minimum-response", type=float, default=0.008,
                    help="minimum measured displacement to pass, rad")
    ap.add_argument("--move-seconds", type=float, default=1.2)
    ap.add_argument("--return-seconds", type=float, default=0.9)
    ap.add_argument("--motor", type=int, help="check only this configured CAN motor")
    a = ap.parse_args()
    if not 0 < a.delta <= 0.1:
        raise SystemExit("--delta must be in (0, 0.1] rad")
    if not 0 < a.minimum_response < a.delta:
        raise SystemExit("--minimum-response must be positive and smaller than --delta")
    if a.move_seconds < 0.5 or a.return_seconds < 0.5:
        raise SystemExit("move/return durations must each be at least 0.5 seconds")
    require_interactive_terminal("powered checks")

    if a.backend == "socketcan":
        kw = {"channel": a.channel}
    elif a.backend == "slcan":
        kw = {"port": a.port, "baudrate": a.serial_baudrate or 115200,
              "rtscts": False, "startup_delay": a.slcan_startup_delay}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.config.control_rate_hz = 25.0 if a.backend == "slcan" else 50.0
    arm.config.max_velocity = 0.12
    results = []
    try:
        arm.connect()
        arm.enable()
        time.sleep(0.5)
        print("all motors enabled and holding; starting bounded response checks")
        selected = [
            (index, joint)
            for index, joint in enumerate(arm.config.joints)
            if a.motor is None or joint.motor_id == a.motor
        ]
        if not selected:
            raise RuntimeError(f"motor {a.motor} is not present in {a.config}")
        for index, joint in selected:
            if arm.state is not ArmState.ENABLED:
                raise RuntimeError(f"arm left ENABLED state: {arm.fault_reason}")
            baseline = arm.get_joint_positions()
            start = baseline[index]
            signed_delta = choose_test_delta(start, joint.lo, joint.hi, a.delta)
            targets = list(baseline)
            targets[index] = start + signed_delta
            arm.set_joint_targets(targets)
            peak, final = wait_and_measure(arm, index, start, signed_delta, a.move_seconds)
            passed = math.isfinite(peak) and peak >= a.minimum_response
            print(f"motor {joint.motor_id}: command {signed_delta:+.3f} rad, "
                  f"peak response {peak:+.4f} rad, final {final:+.4f} "
                  f"{'PASS' if passed else 'FAIL'}")
            results.append((joint.motor_id, signed_delta, peak, passed))

            return_targets = arm.get_joint_positions()
            return_targets[index] = start
            arm.set_joint_targets(return_targets)
            wait_and_measure(arm, index, return_targets[index], -signed_delta, a.return_seconds)
            time.sleep(0.1)

        passed_count = sum(passed for _, _, _, passed in results)
        print(f"\nresponse summary: {passed_count}/{len(results)} motors passed")
        for motor_id, signed_delta, peak, passed in results:
            print(f"  CAN {motor_id}: {peak:.4f} rad measured for {signed_delta:+.3f} rad command "
                  f"-- {'RESPONDING' if passed else 'NO ADEQUATE MOTION'}")
    finally:
        if arm.state in (ArmState.ENABLED, ArmState.FAULT):
            wait_for_release()
            arm.disable()
        arm.disconnect()
        print("torque released, test exited")


if __name__ == "__main__":
    main()
