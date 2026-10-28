#!/usr/bin/env python3
"""Move only the Maker Arm base by a relative angle, with the other motors unplugged.

The base is J1 / CAN motor ID 1. Clockwise and counterclockwise are defined as viewed
from above the arm, looking down the base axis. The motor holds the destination until
the operator types RELEASE.
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
    direction = ap.add_mutually_exclusive_group(required=True)
    direction.add_argument(
        "--clockwise",
        action="store_true",
        help="move clockwise as viewed from above, looking down the base axis",
    )
    direction.add_argument(
        "--counterclockwise",
        action="store_true",
        help="move counterclockwise as viewed from above, looking down the base axis",
    )
    ap.add_argument("--degrees", type=float, default=90.0, help="relative move angle (default: 90)")
    ap.add_argument(
        "--speed-deg-s",
        type=float,
        default=15.0,
        help="motion speed in degrees/second (default: 15; maximum: 30)",
    )
    ap.add_argument(
        "--return-to-start",
        action="store_true",
        help="return after reaching the destination; useful for a small direction test",
    )
    args = ap.parse_args()

    if not math.isfinite(args.degrees) or not 0 < args.degrees <= 180:
        raise SystemExit("--degrees must be in (0, 180]")
    if not math.isfinite(args.speed_deg_s) or not 0 < args.speed_deg_s <= MAX_SPEED_DEG_S:
        raise SystemExit(f"--speed-deg-s must be in (0, {MAX_SPEED_DEG_S:g}]")
    require_interactive_terminal("powered base movement")

    # The standard profile contains all seven motors and therefore expects all seven online.
    # Retain its calibrated base model while connecting and controlling only CAN motor 1.
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
        start = arm.get_joint_positions()[0]
        # Standard positive rotation is counterclockwise when viewed down the joint axis.
        signed_degrees = -args.degrees if args.clockwise else args.degrees
        target = start + math.radians(signed_degrees)
        joint = config.joints[0]
        safe_lo = joint.lo + config.limit_margin
        safe_hi = joint.hi - config.limit_margin
        if not safe_lo <= target <= safe_hi:
            raise RuntimeError(
                f"requested target {math.degrees(target):+.1f} degrees is outside the base "
                f"soft limits [{math.degrees(safe_lo):+.1f}, {math.degrees(safe_hi):+.1f}] degrees; "
                "the motor was not enabled"
            )

        direction_name = "clockwise" if args.clockwise else "counterclockwise"
        print(
            f"base is CAN motor {BASE_MOTOR_ID} (J1 / Python index 0)\n"
            f"current: {math.degrees(start):+.1f} degrees\n"
            f"move:    {args.degrees:.1f} degrees {direction_name}\n"
            f"target:  {math.degrees(target):+.1f} degrees"
        )
        if input("Type MOVE to enable the base motor and begin > ").strip().upper() != "MOVE":
            print("cancelled; motor was not enabled")
            return

        arm.enable()
        timeout = args.degrees / args.speed_deg_s + 5.0
        _move_and_wait(arm, target, timeout)
        print(f"destination reached at {math.degrees(arm.get_joint_positions()[0]):+.1f} degrees")
        if args.return_to_start:
            _move_and_wait(arm, start, timeout)
            print(f"returned to start at {math.degrees(arm.get_joint_positions()[0]):+.1f} degrees")
        print("base is holding its current position")
    except KeyboardInterrupt:
        print("\nmovement interrupted; holding current position")
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
