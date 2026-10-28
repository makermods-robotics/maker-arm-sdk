#!/usr/bin/env python3
"""Safely jog one configured motor a small distance, return, and release torque (MIT protocol).

All other motors remain disabled. The jog is limited to 0.1 rad, must remain inside the
configured soft limits, is ramped at a bounded speed, and is followed by a return ramp. The
motor-side CAN watchdog is only armed where the profile asks for it (MakerModsLab arms a
500 ms watchdog on the gripper only, via ``can_timeout_ms``); it is restored afterwards.
"""

import argparse
import math
import time

from maker_arm import protocol
from maker_arm.cli.common import add_backend_args, arm_from_args
from maker_arm.profiles import DEFAULT_ARM_CONFIG


MAX_DELTA = 0.1
REPLY_TIMEOUT = 0.05


def wait_reply(motor, sequence, timeout=REPLY_TIMEOUT):
    deadline = time.monotonic() + timeout
    while motor.feedback_sequence == sequence and time.monotonic() < deadline:
        time.sleep(0.0005)
    return motor.feedback_sequence != sequence


def ramp(motor, start, end, kp, kd, seconds, feedback_timeout):
    ticks = max(2, math.ceil(seconds / 0.02))
    for step in range(1, ticks + 1):
        target = start + (end - start) * step / ticks
        motor.send_command(target, 0.0, kp, kd, 0.0)
        time.sleep(seconds / ticks)
        if motor.fault_latched:
            raise RuntimeError(f"motor {motor.motor_id} faulted during the jog: {motor.fault_latched}")
        if motor.feedback_age > feedback_timeout:
            raise RuntimeError(f"motor {motor.motor_id} feedback is stale "
                               f"({motor.feedback_age * 1000:.0f} ms); aborting")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    add_backend_args(ap)
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
    if not (protocol.KP_MIN <= a.kp <= protocol.KP_MAX and protocol.KD_MIN <= a.kd <= protocol.KD_MAX):
        raise SystemExit("kp/kd are outside protocol limits")

    arm = arm_from_args(a)
    ids = [j.motor_id for j in arm.config.joints]
    if a.motor_id not in ids:
        raise SystemExit(f"motor {a.motor_id} is not configured; configured IDs: {ids}")
    # Fault evidence is kept: a faulted motor makes connect() fail and is never cleared here.
    arm.connect()
    index = ids.index(a.motor_id)
    joint = arm.config.joints[index]
    motor = arm.motors[index]
    start_joint = arm.get_joint_positions()[index]
    end_joint = start_joint + a.delta
    if not (joint.lo <= start_joint <= joint.hi and joint.lo <= end_joint <= joint.hi):
        arm.disconnect()
        raise SystemExit(
            f"jog [{start_joint:+.4f}, {end_joint:+.4f}] would leave {joint.label} "
            f"soft limits [{joint.lo:+.4f}, {joint.hi:+.4f}]"
        )
    temp = motor.feedback.temperature
    temp_limit = joint.enable_temperature_c if joint.enable_temperature_c is not None else joint.max_temperature_c
    if temp_limit is not None and temp >= temp_limit:
        arm.disconnect()
        raise SystemExit(f"{joint.label} is {temp:.1f} °C; enabling needs it below {temp_limit:.0f} °C")
    start_motor = arm._to_motor(index, start_joint)
    end_motor = arm._to_motor(index, end_joint)
    print(f"{joint.label} (motor {a.motor_id}): {start_joint:+.4f} -> {end_joint:+.4f} rad, then return")

    watchdog_original = None
    try:
        # Explicitly leave every non-target motor disabled.
        for other in arm.motors:
            if other is not motor:
                other.disable()
        time.sleep(0.05)
        if joint.can_timeout_ms is not None:
            if motor.params_supported:
                ticks = joint.can_timeout_ms * protocol.CAN_TIMEOUT_TICKS_PER_MS
                watchdog_original = motor.read_param(protocol.ParamIndex.CAN_TIMEOUT, "u32")
                readback = motor.write_param(protocol.ParamIndex.CAN_TIMEOUT, ticks, "u32")
                if readback != ticks:
                    raise RuntimeError(f"CAN timeout read back {readback}, expected {ticks}")
                print(f"motor watchdog armed: {joint.can_timeout_ms} ms")
            else:
                print(f"warning: firmware has no parameter access; jogging without the "
                      f"{joint.can_timeout_ms} ms motor watchdog")
        # Neutral preload (kp=kd=0 at the present pose) so enabling cannot resume a stale setpoint.
        sequence = motor.feedback_sequence
        motor.send_command(start_motor, 0.0, 0.0, 0.0, 0.0)
        if not wait_reply(motor, sequence):
            raise RuntimeError(f"motor {a.motor_id} did not acknowledge the neutral preload; not enabling")
        sequence = motor.feedback_sequence
        motor.enable()
        if not wait_reply(motor, sequence):
            raise RuntimeError(f"motor {a.motor_id} did not reply to enable")
        if motor.fault_latched:
            raise RuntimeError(f"motor {a.motor_id} reported a fault on enable: {motor.fault_latched}")
        if motor.params_supported and motor.feedback.mode != protocol.MODE_RUN:
            raise RuntimeError(f"motor {a.motor_id} enable rejected: mode={motor.feedback.mode}")
        timeout = arm.config.feedback_timeout
        ramp(motor, start_motor, end_motor, a.kp, a.kd, a.seconds, timeout)
        reached = arm._to_joint(index, motor.feedback.position)
        print(f"outbound feedback: {reached:+.4f} rad")
        ramp(motor, end_motor, start_motor, a.kp, a.kd, a.seconds, timeout)
        returned = arm._to_joint(index, motor.feedback.position)
        status = motor.query_fault()
        print(f"return feedback:   {returned:+.4f} rad; "
              f"fault word 0x{status.word:08X} ({protocol.fault_text(status.word)})")
    finally:
        released = motor.disable_confirmed()
        if not released:
            print("could not confirm torque release; leaving the motor watchdog armed -- cut motor power if it holds")
        if watchdog_original is not None and released:
            try:
                motor.write_param(protocol.ParamIndex.CAN_TIMEOUT, watchdog_original, "u32")
            except Exception as e:
                print(f"could not restore the CAN timeout ({watchdog_original} ticks): {e}")
        arm.disconnect()
        print("target torque released")


if __name__ == "__main__":
    main()
