#!/usr/bin/env python3
"""Absolute-only teleoperation of Maker Arm v1 from a Star 102 leader."""

import math
import time
from pathlib import Path

from maker_arm.cli.common import arm_from_args, make_parser
from maker_arm.mapping import JointMapper
from maker_arm.profiles import DEFAULT_STAR_MAPPING
from maker_arm.cli.safety import release_if_holding, require_interactive_terminal
from maker_arm.stall import MotorStateRecorder, StallThresholds, default_stall_log_path


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"cleanup step failed (continuing): {e}")


def _read_leader_snapshot(bus, servo_ids: list[int], timeout: float = 2.0) -> dict[int, float]:
    """Accumulate a complete reliable snapshot across intermittent serial replies."""
    raw: dict[int, float] = {}
    deadline = time.monotonic() + timeout
    while len(raw) != len(servo_ids) and time.monotonic() < deadline:
        data = bus.sync_monitor(servo_ids)
        raw.update({i: data[i].angle_deg for i in servo_ids
                    if data.get(i) and data[i].reliable})
    return raw


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    ap.add_argument("--map", dest="map_path", default=str(DEFAULT_STAR_MAPPING))
    ap.add_argument("--rate", type=float,
                    help="leader update rate in Hz (default: 100 SocketCAN, 25 serial SLCAN)")
    ap.add_argument("--control-rate", type=float,
                    help="override follower control-loop rate in Hz (25 is verified for serial SLCAN)")
    ap.add_argument("--max-velocity", type=float,
                    help="override follower velocity limit in rad/s (use a low value for first-motion checks)")
    ap.add_argument("--sync-threshold", type=float, default=0.8, help="startup pose-difference confirmation threshold, rad")
    ap.add_argument(
        "--record-motor-states", nargs="?", const="", metavar="CSV",
        help="record all motor states and detect stalls; optionally provide the CSV path",
    )
    ap.add_argument("--payload-kg", type=float, default=0.0,
                    help="payload mass stored with telemetry (default: 0)")
    ap.add_argument("--telemetry-dir", default="logs",
                    help="directory for automatically named telemetry files")
    ap.add_argument("--stall-error", type=float, default=0.12,
                    help="minimum |command-position| error for a stall candidate, rad")
    ap.add_argument("--stall-velocity", type=float, default=0.04,
                    help="maximum |velocity| for a stall candidate, rad/s")
    ap.add_argument("--stall-torque", type=float, default=0.5,
                    help="minimum |measured torque| for a stall candidate, Nm")
    ap.add_argument("--stall-seconds", type=float, default=0.5,
                    help="candidate persistence before stall confirmation")
    ap.add_argument("--max-temperature", type=float, default=70.0,
                    help="freeze and stop teleop if any motor reaches this temperature, °C")
    a = ap.parse_args()
    star_ids = [int(x) for x in a.star_ids.split(",")]
    if not math.isfinite(a.payload_kg) or a.payload_kg < 0:
        raise SystemExit("--payload-kg must be finite and non-negative")
    thresholds_raw = (a.stall_error, a.stall_velocity, a.stall_torque, a.stall_seconds)
    if not all(math.isfinite(value) and value > 0 for value in thresholds_raw):
        raise SystemExit("stall thresholds must be finite and greater than zero")
    if not math.isfinite(a.max_temperature) or a.max_temperature <= 0:
        raise SystemExit("--max-temperature must be finite and greater than zero")
    require_interactive_terminal("teleoperation")
    leader_rate = a.rate if a.rate is not None else (25.0 if a.backend == "slcan" else 100.0)
    if not math.isfinite(leader_rate) or leader_rate <= 0:
        raise SystemExit("--rate must be finite and greater than zero")

    try:
        from motorbridge_smart_servo import FashionStarServo
    except ImportError:
        raise SystemExit("missing motorbridge_smart_servo: conda run -n maker-arm pip install motorbridge-smart-servo (or see the plan's Task 18 environment notes)")

    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    mapper = JointMapper.from_json(a.map_path)
    arm = arm_from_args(a)
    recorder = None
    stop_reason = "normal_exit"
    if a.max_velocity is not None:
        if a.max_velocity <= 0:
            raise SystemExit("--max-velocity must be greater than zero")
        arm.config.max_velocity = a.max_velocity
        print(f"follower velocity limit: {a.max_velocity:.3f} rad/s")
    if a.control_rate is None and a.backend == "slcan":
        arm.config.control_rate_hz = 25.0
        print("follower control rate: 25.0 Hz (serial SLCAN safe default)")
    elif a.control_rate is not None:
        if a.control_rate <= 0:
            raise SystemExit("--control-rate must be greater than zero")
        arm.config.control_rate_hz = a.control_rate
        print(f"follower control rate: {a.control_rate:.1f} Hz")
    try:
        arm.connect()
        if a.record_motor_states is not None:
            telemetry_path = (
                Path(a.record_motor_states) if a.record_motor_states
                else default_stall_log_path(a.payload_kg, a.telemetry_dir)
            )
            recorder = MotorStateRecorder(
                telemetry_path,
                a.payload_kg,
                arm,
                StallThresholds(
                    error_rad=a.stall_error,
                    velocity_rad_s=a.stall_velocity,
                    torque_nm=a.stall_torque,
                    persistence_s=a.stall_seconds,
                    max_feedback_age_s=min(arm.config.feedback_timeout, 0.15),
                ),
            )
            print(f"recording all motor states to {telemetry_path}")
        raw = _read_leader_snapshot(bus, star_ids)
        if len(raw) != len(star_ids):
            raise SystemExit(f"leader only read {sorted(raw)} -- check the star serial port/IDs")
        targets = mapper.map(raw)
        diff = max(abs(t - c) for t, c in zip(targets, arm.get_joint_positions()))
        print(f"startup pose difference max={diff:.2f} rad (the follower will ramp up to it at the rate limit)")
        if diff > a.sync_threshold:
            input(f"⚠️ pose difference exceeds {a.sync_threshold} rad, confirm the area is clear and press ENTER to start > ")
        arm.enable()
        dt = 1.0 / leader_rate
        teleop_t0 = time.monotonic()
        print("teleoperating, Ctrl-C to exit." +
              (" Stall detection is active." if recorder else ""))
        misses = {i: 0 for i in star_ids}
        while arm.state.name == "ENABLED":
            t0 = time.perf_counter()
            try:
                data = bus.sync_monitor(star_ids)
            except Exception as e:
                print(f"leader read error, stopping follow: {e}")
                stop_reason = f"leader_read_error: {e}"
                break
            raw = {i: data[i].angle_deg for i in star_ids if data.get(i) and data[i].reliable}
            for i in star_ids:
                misses[i] = 0 if i in raw else misses[i] + 1
            lost = [i for i, count in misses.items() if count >= 50]
            if lost:
                print(f"leader readings lost for servos {lost}, stopping follow")
                stop_reason = f"leader_readings_lost: {lost}"
                break
            targets = mapper.map(raw)
            arm.set_joint_targets(targets)
            if recorder:
                elapsed = time.monotonic() - teleop_t0
                new_events = recorder.record(elapsed, targets)
                if new_events:
                    first = min(
                        new_events,
                        key=lambda event: (
                            event.candidate_since_s,
                            event.confirmed_at_s,
                            event.motor_id,
                        ),
                    )
                    arm.hold_current_position()
                    stop_reason = f"stall_CAN{first.motor_id}"
                    print(
                        f"\nSTALL DETECTED: CAN {first.motor_id} stalled first at "
                        f"{first.confirmed_at_s:.3f}s "
                        f"(error={first.error_rad:+.3f} rad, "
                        f"velocity={first.velocity_rad_s:+.3f} rad/s, "
                        f"torque={first.torque_nm:+.2f} Nm)."
                    )
                    print("All commands frozen at the measured pose; teleop stopped.")
                    break
                temperatures = arm.get_temperatures()
                hot = [
                    arm.config.joints[i].motor_id
                    for i, temperature in enumerate(temperatures)
                    if math.isfinite(temperature) and temperature >= a.max_temperature
                ]
                if hot:
                    arm.hold_current_position()
                    stop_reason = f"over_temperature_CAN{','.join(map(str, hot))}"
                    print(f"\nTEMPERATURE ABORT: CAN motors {hot}; commands frozen and teleop stopped.")
                    break
            remain = dt - (time.perf_counter() - t0)
            if remain > 0:
                time.sleep(remain)
        if arm.state.name == "FAULT":
            stop_reason = f"sdk_fault: {arm.fault_reason}"
            print("FAULT:", arm.fault_reason)
    except KeyboardInterrupt:
        stop_reason = "operator_interrupt"
        if arm.state.name == "ENABLED":
            arm.hold_current_position()
    finally:
        if recorder:
            summary_path = recorder.close(stop_reason)
            events = recorder.detector.events
            print(f"\nmotor-state CSV: {recorder.path}")
            print(f"stall summary:    {summary_path}")
            if events:
                ordered = sorted(
                    events,
                    key=lambda event: (
                        event.candidate_since_s,
                        event.confirmed_at_s,
                        event.motor_id,
                    ),
                )
                print("stall order: " + " -> ".join(f"CAN{event.motor_id}" for event in ordered))
            else:
                print("no motor met all configured stall conditions")
        torque_was_enabled = arm.state.name in ("ENABLED", "FAULT")
        _safe(lambda: release_if_holding(arm))
        _safe(arm.disconnect)
        _safe(bus.close)
        if torque_was_enabled:
            print("torque released, exiting")
        else:
            print("torque was not enabled; transport closed without sending disable")


if __name__ == "__main__":
    main()
