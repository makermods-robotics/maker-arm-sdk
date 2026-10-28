"""Open an isolated MIT-mode gripper slowly and capture both encoder endpoints.

Only the gripper motor is enabled. The open target is the profile's gripper open limit (or
--open-deg inside the profile limits), so the capture can never drive past the configured
bound. The gripper's motor watchdog (``can_timeout_ms``) is armed for the motion and restored
afterwards, as MakerModsLab does. A gripper that reports a fault is refused, never cleared.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from maker_arm import protocol as p
from maker_arm.cli.common import add_backend_args, backend_from_args
from maker_arm.config import ArmConfig
from maker_arm.errors import ParamTimeout
from maker_arm.motor import Motor
from maker_arm.profiles import DEFAULT_ARM_CONFIG

KP, KD = 10.0, 1.0
DURATION = 6.0
TICKS = 180
REPLY_TIMEOUT = 0.06


def gripper_joint(config: ArmConfig):
    for j in config.joints:
        if j.name == "gripper":
            return j
    return config.joints[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    add_backend_args(ap)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--open-deg", type=float,
                    help="open target in joint degrees (default: the profile's gripper open limit)")
    args = ap.parse_args()
    if args.output.exists():
        ap.error("Output already exists")
    config = ArmConfig.from_yaml(args.config)
    joint = gripper_joint(config)
    open_joint = joint.lo if args.open_deg is None else math.radians(args.open_deg)
    if not (math.isfinite(open_joint) and joint.lo <= open_joint <= joint.hi):
        ap.error(f"--open-deg must be inside the profile limits "
                 f"[{math.degrees(joint.lo):.1f}, {math.degrees(joint.hi):.1f}] deg")
    safe_open = open_joint * joint.direction + joint.offset   # motor coordinates

    be = backend_from_args(args)
    motor = Motor(joint.motor_id, be, p.MOTOR_PARAMS[joint.model])
    samples = []

    def receive(cid, data):
        if p.reply_motor_id(cid, data) != joint.motor_id:
            return
        msg = p.parse_frame(cid, data, motor.params, config.host_id)
        motor.handle_frame(msg)
        if isinstance(msg, p.MotorFeedback):
            samples.append({"position_rad": msg.position, "velocity_rad_s": msg.velocity,
                            "torque_nm": msg.torque, "temperature_c": msg.temperature,
                            "mode": msg.mode, "fault_flag": msg.fault_flag,
                            "monotonic_s": time.monotonic()})

    def command(position, timeout=REPLY_TIMEOUT):
        sequence = motor.feedback_sequence
        motor.send_command(position, 0.0, KP, KD, 0.0)
        deadline = time.monotonic() + timeout
        while motor.feedback_sequence == sequence and time.monotonic() < deadline:
            time.sleep(0.0005)
        if motor.fault_latched:
            raise RuntimeError(f"gripper faulted: {motor.fault_latched}")
        return motor.feedback if motor.feedback_sequence != sequence else None

    be.set_recv_callback(receive)
    be.open()
    enabled = False
    watchdog_original = None
    try:
        try:
            status = motor.query_fault(timeout=0.1, attempts=3)
        except ParamTimeout:
            raise RuntimeError(f"gripper motor {joint.motor_id} did not answer the fault query")
        if status.has_fault:
            raise RuntimeError(f"gripper reports fault 0x{status.word:08X} ({status.text}); "
                               "fault kept for inspection")

        initial_states = []
        for _ in range(10):
            try:
                initial_states.append(motor.read_state(timeout=0.08, attempts=1))
            except ParamTimeout:
                pass
            time.sleep(0.03)
        if len(initial_states) < 8:
            raise RuntimeError("Gripper feedback became unreliable before motion")
        closed = statistics.median(x.position for x in initial_states)
        if not -0.15 <= closed <= 0.15:
            raise RuntimeError(f"Expected a closed position near zero; measured {closed:+.4f} rad")
        temp = initial_states[-1].temperature
        temp_limit = joint.enable_temperature_c if joint.enable_temperature_c is not None else joint.max_temperature_c
        if temp_limit is not None and temp >= temp_limit:
            raise RuntimeError(f"gripper is {temp:.1f} °C; enabling needs it below {temp_limit:.0f} °C")

        if joint.can_timeout_ms is not None and motor.probe_params():
            ticks = joint.can_timeout_ms * p.CAN_TIMEOUT_TICKS_PER_MS
            watchdog_original = motor.read_param(p.ParamIndex.CAN_TIMEOUT, "u32")
            readback = motor.write_param(p.ParamIndex.CAN_TIMEOUT, ticks, "u32")
            if readback != ticks:
                raise RuntimeError(f"CAN timeout read back {readback}, expected {ticks}")

        # Neutral preload (kp=kd=0) at the present pose, then enable only the gripper.
        sequence = motor.feedback_sequence
        motor.send_command(closed, 0.0, 0.0, 0.0, 0.0)
        deadline = time.monotonic() + REPLY_TIMEOUT
        while motor.feedback_sequence == sequence and time.monotonic() < deadline:
            time.sleep(0.0005)
        if motor.feedback_sequence == sequence:
            raise RuntimeError("gripper did not acknowledge the neutral preload; not enabling")
        sequence = motor.feedback_sequence
        enabled = True   # disable in cleanup even if the enable reply is lost
        motor.enable()
        deadline = time.monotonic() + REPLY_TIMEOUT
        while motor.feedback_sequence == sequence and time.monotonic() < deadline:
            time.sleep(0.0005)
        if motor.feedback_sequence == sequence:
            raise RuntimeError("gripper did not reply to enable")
        if motor.fault_latched:
            raise RuntimeError(f"gripper reported a fault on enable: {motor.fault_latched}")

        misses = 0
        for step in range(1, TICKS + 1):
            target = closed + (safe_open - closed) * step / TICKS
            state = command(target)
            if state is None:
                misses += 1
                if misses >= 3:
                    raise RuntimeError("Lost gripper feedback during opening")
            else:
                misses = 0
                # Stop early if the motor loads against an unexpected obstruction.
                if step > 20 and abs(state.torque) > 3.0 and abs(state.velocity) < 0.15:
                    raise RuntimeError("Stopped on unexpected gripper resistance")
                if joint.max_temperature_c is not None and state.temperature >= joint.max_temperature_c:
                    raise RuntimeError(f"gripper reached {state.temperature:.1f} °C")
            time.sleep(DURATION / TICKS)

        final_states = []
        for _ in range(15):
            state = command(safe_open)
            if state:
                final_states.append(state)
            time.sleep(0.03)
        if len(final_states) < 10:
            raise RuntimeError("Insufficient open-position feedback")
        opened = statistics.median(x.position for x in final_states)
        if abs(opened - safe_open) > 0.12:
            raise RuntimeError(f"Gripper did not reach the open limit: {opened:+.4f} rad")

        result = {
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "motor_id": joint.motor_id,
            "protocol": "MIT, standard 11-bit CAN, 1 Mbps",
            "closed_encoder_rad": closed,
            "closed_encoder_deg": math.degrees(closed),
            "open_encoder_rad": opened,
            "open_encoder_deg": math.degrees(opened),
            "encoder_span_rad": opened - closed,
            "user_measured_gap_mm": {"closed": 0.0, "open": 104.825},
            "per_jaw_travel_mm": 52.4125,
            "commanded_safe_open_limit_rad": safe_open,
            "motion": f"Motor {joint.motor_id} only; {DURATION:g}-second ramp; kp={KP:g}, kd={KD:g}, "
                      "zero feed-forward torque",
            "final_torque_released": True,
            "samples": samples,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({key: value for key, value in result.items() if key != "samples"}, indent=2))
    finally:
        try:
            released = motor.disable_confirmed() if enabled else True
            if not released:
                print("could not confirm gripper torque release; leaving its watchdog armed -- cut motor power if it holds")
            if watchdog_original is not None and released:
                try:
                    motor.write_param(p.ParamIndex.CAN_TIMEOUT, watchdog_original, "u32")
                except Exception as e:
                    print(f"could not restore the gripper CAN timeout ({watchdog_original} ticks): {e}")
        finally:
            be.close()


if __name__ == "__main__":
    main()
