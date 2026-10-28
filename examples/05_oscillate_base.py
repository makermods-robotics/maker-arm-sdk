#!/usr/bin/env python3
"""Oscillate the lone Maker Arm base motor around its startup position.

Place the unpowered base at the desired center before starting. The sequence is
center -> center + amplitude -> center - amplitude -> center + amplitude, repeating.
With the default 90-degree amplitude, the repeated endpoint moves are -180 and +180
degrees. Ctrl-C stops the sweep and holds until the operator types RELEASE.
"""

from __future__ import annotations

import math
import time
from dataclasses import replace

from maker_arm.arm import Arm, ArmState
from maker_arm.cli.common import backend_from_args, make_parser
from maker_arm.cli.safety import release_if_holding, require_interactive_terminal
from maker_arm.config import ArmConfig


BASE_MOTOR_ID = 1
MAX_AMPLITUDE_DEG = 120.0
MAX_SPEED_DEG_S = 30.0
POSITION_TOLERANCE_DEG = 2.0


def _safe(fn) -> None:
    try:
        fn()
    except Exception as exc:
        print(f"cleanup step failed (continuing): {exc}")


def _backend_from_args(args):
    return backend_from_args(args)


def _move_and_wait(arm: Arm, target: float, timeout: float) -> None:
    if not arm.set_joint_targets([target]):
        raise RuntimeError("the base target was rejected")

    tolerance = math.radians(POSITION_TOLERANCE_DEG)
    deadline = time.monotonic() + timeout
    settled_since = None
    while time.monotonic() < deadline:
        if arm.state is ArmState.FAULT:
            raise RuntimeError(f"base motor fault: {arm.fault_reason}")
        position = arm.get_joint_positions()[0]
        commanded = arm.get_commanded_positions()[0]
        if abs(position - target) <= tolerance and abs(commanded - target) <= tolerance:
            settled_since = settled_since or time.monotonic()
            if time.monotonic() - settled_since >= 0.3:
                return
        else:
            settled_since = None
        time.sleep(0.02)

    raise RuntimeError(
        f"base did not reach {math.degrees(target):+.1f} degrees before the timeout"
    )


def main() -> None:
    ap = make_parser(__doc__)
    ap.add_argument(
        "--amplitude-deg",
        type=float,
        default=90.0,
        help="angle on each side of the startup center (default: 90)",
    )
    ap.add_argument(
        "--speed-deg-s",
        type=float,
        default=20.0,
        help="motion speed in degrees/second (default: 20; maximum: 30)",
    )
    ap.add_argument(
        "--dwell-seconds",
        type=float,
        default=0.5,
        help="pause at each endpoint (default: 0.5)",
    )
    ap.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="complete -/+ sweeps; 0 repeats until Ctrl-C (default: 0)",
    )
    args = ap.parse_args()

    if not math.isfinite(args.amplitude_deg) or not 0 < args.amplitude_deg <= MAX_AMPLITUDE_DEG:
        raise SystemExit(f"--amplitude-deg must be in (0, {MAX_AMPLITUDE_DEG:g}]")
    if not math.isfinite(args.speed_deg_s) or not 0 < args.speed_deg_s <= MAX_SPEED_DEG_S:
        raise SystemExit(f"--speed-deg-s must be in (0, {MAX_SPEED_DEG_S:g}]")
    if not math.isfinite(args.dwell_seconds) or not 0 <= args.dwell_seconds <= 10:
        raise SystemExit("--dwell-seconds must be in [0, 10]")
    if args.cycles < 0:
        raise SystemExit("--cycles must be 0 or greater")
    require_interactive_terminal("powered base oscillation")

    # Use the production model for the base but do not require motors 2-7 to be online.
    full_config = ArmConfig.from_yaml(args.config)
    base_joints = [joint for joint in full_config.joints if joint.motor_id == BASE_MOTOR_ID]
    if len(base_joints) != 1:
        raise SystemExit(
            f"{args.config} must contain exactly one base entry with motor_id {BASE_MOTOR_ID}"
        )
    config = replace(
        full_config,
        control_rate_hz=25.0 if args.backend == "slcan" else full_config.control_rate_hz,
        max_velocity=math.radians(args.speed_deg_s),
        required_motor_ids=None,
        joints=base_joints,
    )
    config._validate()
    arm = Arm(config, _backend_from_args(args))

    try:
        arm.connect()
        center = arm.get_joint_positions()[0]
        amplitude = math.radians(args.amplitude_deg)
        positive_target = center + amplitude
        negative_target = center - amplitude
        joint = config.joints[0]
        safe_lo = joint.lo + config.limit_margin
        safe_hi = joint.hi - config.limit_margin
        if negative_target < safe_lo or positive_target > safe_hi:
            raise RuntimeError(
                f"startup center {math.degrees(center):+.1f} degrees cannot fit +/-"
                f"{args.amplitude_deg:.1f} degrees inside the base soft limits "
                f"[{math.degrees(safe_lo):+.1f}, {math.degrees(safe_hi):+.1f}] degrees; "
                "the motor was not enabled. Reposition the unpowered base closer to the "
                f"calibrated midpoint {math.degrees((safe_lo + safe_hi) / 2):+.1f} degrees."
            )

        cycle_text = "until Ctrl-C" if args.cycles == 0 else f"for {args.cycles} cycle(s)"
        print(
            f"base is CAN motor {BASE_MOTOR_ID} (J1 / Python index 0)\n"
            f"startup center: {math.degrees(center):+.1f} degrees\n"
            f"positive end:   {math.degrees(positive_target):+.1f} degrees\n"
            f"negative end:   {math.degrees(negative_target):+.1f} degrees\n"
            f"speed:          {args.speed_deg_s:.1f} degrees/second\n"
            f"run:            {cycle_text}"
        )
        if input("Type OSCILLATE to enable the base motor and begin > ").strip().upper() != "OSCILLATE":
            print("cancelled; motor was not enabled")
            return

        arm.enable()
        first_timeout = args.amplitude_deg / args.speed_deg_s + 5.0
        sweep_timeout = (2 * args.amplitude_deg) / args.speed_deg_s + 5.0

        # Requested sequence: +90, -180, +180, then repeat the full sweeps.
        _move_and_wait(arm, positive_target, first_timeout)
        completed_cycles = 0
        while args.cycles == 0 or completed_cycles < args.cycles:
            if args.dwell_seconds:
                time.sleep(args.dwell_seconds)
            _move_and_wait(arm, negative_target, sweep_timeout)
            if args.dwell_seconds:
                time.sleep(args.dwell_seconds)
            _move_and_wait(arm, positive_target, sweep_timeout)
            completed_cycles += 1
            print(
                f"completed cycle {completed_cycles}; position "
                f"{math.degrees(arm.get_joint_positions()[0]):+.1f} degrees",
                flush=True,
            )

        print("requested cycles complete; base is holding at the positive endpoint")
    except KeyboardInterrupt:
        print("\noscillation interrupted; holding current position")
        if arm.state is ArmState.ENABLED:
            arm.hold_current_position()
    finally:
        torque_was_enabled = arm.state in (ArmState.ENABLED, ArmState.FAULT)
        _safe(lambda: release_if_holding(arm))
        _safe(arm.disconnect)
        if torque_was_enabled:
            print("torque released, exiting")
        else:
            print("torque was not enabled; transport closed")


if __name__ == "__main__":
    main()
