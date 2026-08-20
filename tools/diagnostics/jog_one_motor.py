#!/usr/bin/env python3
"""Safely jog one configured motor a small distance, return, and release torque.

All other motors remain disabled. The jog is limited to 0.1 rad, must remain inside the
configured soft limits, is ramped at a bounded speed, and is followed by a return ramp.
"""

import argparse
import math
import time

from maker_arm import protocol
from maker_arm.arm import Arm
from maker_arm.errors import fault_text
from maker_arm.profiles import DEFAULT_ARM_CONFIG


MAX_DELTA = 0.1


def ramp(motor, start, end, kp, kd, seconds):
    ticks = max(2, math.ceil(seconds / 0.02))
    for step in range(1, ticks + 1):
        target = start + (end - start) * step / ticks
        motor.send_mit(target, 0.0, kp, kd, 0.0)
        time.sleep(seconds / ticks)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--serial-baudrate", type=int,
                    help="serial rate (SLCAN default 115200; AT default 921600)")
    ap.add_argument("--slcan-startup-delay", type=float, default=2.5)
    ap.add_argument("--motor-id", type=int, required=True)
    ap.add_argument("--delta", type=float, required=True, help="joint-coordinate jog in radians")
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=1.2, help="duration of each one-way ramp")
    a = ap.parse_args()

    if not math.isfinite(a.delta) or abs(a.delta) > MAX_DELTA or a.delta == 0:
        raise SystemExit(f"--delta must be nonzero and no greater than {MAX_DELTA} rad in magnitude")
    if a.seconds < 0.5:
        raise SystemExit("--seconds must be at least 0.5")
    if not (0 <= a.kp <= protocol.KP_MAX and 0 <= a.kd <= protocol.KD_MAX):
        raise SystemExit("kp/kd are outside protocol limits")

    if a.backend == "socketcan":
        kw = {"channel": a.channel}
    elif a.backend == "slcan":
        kw = {"port": a.port, "baudrate": a.serial_baudrate or 115200,
              "rtscts": False, "startup_delay": a.slcan_startup_delay}
    else:
        kw = {"port": a.port, "baudrate": a.serial_baudrate or 921600}

    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    ids = [j.motor_id for j in arm.config.joints]
    if a.motor_id not in ids:
        arm.disconnect()
        raise SystemExit(f"motor {a.motor_id} is not configured; configured IDs: {ids}")
    index = ids.index(a.motor_id)
    joint = arm.config.joints[index]
    motor = arm.motors[index]
    start_joint = arm.get_joint_positions()[index]
    end_joint = start_joint + a.delta
    if not (joint.lo <= start_joint <= joint.hi and joint.lo <= end_joint <= joint.hi):
        arm.disconnect()
        raise SystemExit(
            f"jog [{start_joint:+.4f}, {end_joint:+.4f}] would leave motor {a.motor_id} "
            f"soft limits [{joint.lo:+.4f}, {joint.hi:+.4f}]"
        )
    start_motor = arm._to_motor(index, start_joint)
    end_motor = arm._to_motor(index, end_joint)
    print(f"motor {a.motor_id}: {start_joint:+.4f} -> {end_joint:+.4f} rad, then return")

    try:
        # Explicitly leave every non-target motor disabled, then configure only the target.
        for other in arm.motors:
            other.disable()
        time.sleep(0.05)
        motor.write_param(protocol.ParamIndex.RUN_MODE, 0, "u8")
        motor.write_param(protocol.ParamIndex.CAN_TIMEOUT,
                          arm.config.motor_can_timeout_ms * protocol.CAN_TIMEOUT_PER_MS, "u32")
        time.sleep(0.05)
        motor.enable()
        time.sleep(0.05)
        ramp(motor, start_motor, end_motor, a.kp, a.kd, a.seconds)
        reached = arm._to_joint(index, motor.feedback.position)
        print(f"outbound feedback: {reached:+.4f} rad")
        ramp(motor, end_motor, start_motor, a.kp, a.kd, a.seconds)
        returned = arm._to_joint(index, motor.feedback.position)
        faults = motor.feedback.fault_bits
        print(f"return feedback:   {returned:+.4f} rad; {fault_text(faults)}")
    finally:
        motor.disable()
        time.sleep(0.05)
        arm.disconnect()
        print("target torque released")


if __name__ == "__main__":
    main()
