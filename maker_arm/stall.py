"""Motor-state CSV recording and persistent stall detection for powered control loops."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from .errors import fault_text


@dataclass(frozen=True)
class StallThresholds:
    error_rad: float = 0.12
    velocity_rad_s: float = 0.04
    torque_nm: float = 0.5
    persistence_s: float = 0.5
    max_feedback_age_s: float = 0.15


@dataclass(frozen=True)
class StallEvent:
    motor_id: int
    candidate_since_s: float
    confirmed_at_s: float
    error_rad: float
    velocity_rad_s: float
    torque_nm: float


class StallDetector:
    """Track independent stall candidates and preserve their confirmation order."""

    def __init__(self, motor_ids: list[int], thresholds: StallThresholds):
        self.thresholds = thresholds
        self.candidate_since: dict[int, float | None] = {motor_id: None for motor_id in motor_ids}
        self.events: list[StallEvent] = []
        self._confirmed: set[int] = set()

    def is_candidate(self, *, error: float, velocity: float, torque: float,
                     feedback_age: float, mode: int, fault_bits: int) -> bool:
        values = (error, velocity, torque, feedback_age)
        return (
            all(math.isfinite(value) for value in values)
            and abs(error) >= self.thresholds.error_rad
            and abs(velocity) <= self.thresholds.velocity_rad_s
            and abs(torque) >= self.thresholds.torque_nm
            and feedback_age <= self.thresholds.max_feedback_age_s
            and mode == 2
            and fault_bits == 0
        )

    def update(self, now_s: float, motor_id: int, *, error: float, velocity: float,
               torque: float, feedback_age: float, mode: int, fault_bits: int) -> bool:
        candidate = self.is_candidate(
            error=error,
            velocity=velocity,
            torque=torque,
            feedback_age=feedback_age,
            mode=mode,
            fault_bits=fault_bits,
        )
        if motor_id in self._confirmed:
            return candidate
        if not candidate:
            self.candidate_since[motor_id] = None
            return False
        if self.candidate_since[motor_id] is None:
            self.candidate_since[motor_id] = now_s
            return True
        since = self.candidate_since[motor_id]
        if now_s - since >= self.thresholds.persistence_s - 1e-9:
            self._confirmed.add(motor_id)
            self.events.append(
                StallEvent(motor_id, since, now_s, error, velocity, torque)
            )
        return True


def default_stall_log_path(payload_kg: float, directory: str = "logs") -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(directory) / f"teleop_motor_states_{timestamp}_{payload_kg:g}kg.csv"


class MotorStateRecorder:
    """Record one row per motor per teleop tick and produce a stall summary JSON."""

    FIELDS = [
        "elapsed_s", "wall_time", "payload_kg", "joint", "motor_id", "model",
        "leader_target_rad", "command_rad", "position_rad", "error_rad",
        "velocity_rad_s", "torque_nm", "torque_fraction", "temperature_c",
        "mode", "fault_bits", "fault_text", "feedback_age_s",
        "stall_candidate", "stall_confirmed",
    ]

    def __init__(self, path: Path, payload_kg: float, arm, thresholds: StallThresholds):
        self.path = path
        self.payload_kg = payload_kg
        self.arm = arm
        self.thresholds = thresholds
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self.detector = StallDetector([joint.motor_id for joint in arm.config.joints], thresholds)
        self.started_at = datetime.now().astimezone()
        self._last_flush_s = -math.inf
        self._closed = False
        self.peaks = {
            joint.motor_id: {
                "max_abs_error_rad": 0.0,
                "max_abs_torque_nm": 0.0,
                "max_temperature_c": -math.inf,
            }
            for joint in arm.config.joints
        }

    def record(self, elapsed_s: float, leader_targets: list[float]) -> list[StallEvent]:
        commanded = self.arm.get_commanded_positions()
        positions = self.arm.get_joint_positions()
        velocities = self.arm.get_joint_velocities()
        torques = self.arm.get_joint_torques()
        temperatures = self.arm.get_temperatures()
        event_count = len(self.detector.events)
        samples = []

        for index, (joint, motor) in enumerate(zip(self.arm.config.joints, self.arm.motors)):
            feedback = motor.feedback
            mode = feedback.mode if feedback else -1
            fault_bits = feedback.fault_bits if feedback else 0
            error = commanded[index] - positions[index]
            candidate = self.detector.update(
                elapsed_s,
                joint.motor_id,
                error=error,
                velocity=velocities[index],
                torque=torques[index],
                feedback_age=motor.feedback_age,
                mode=mode,
                fault_bits=fault_bits,
            )
            torque_limit = max(abs(motor.params.t_min), abs(motor.params.t_max))
            peak = self.peaks[joint.motor_id]
            peak["max_abs_error_rad"] = max(peak["max_abs_error_rad"], abs(error))
            peak["max_abs_torque_nm"] = max(peak["max_abs_torque_nm"], abs(torques[index]))
            peak["max_temperature_c"] = max(peak["max_temperature_c"], temperatures[index])
            samples.append((index, joint, motor, mode, fault_bits, error, torque_limit, candidate))

        confirmed = {event.motor_id for event in self.detector.events}
        wall_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
        for index, joint, motor, mode, fault_bits, error, torque_limit, candidate in samples:
            self._writer.writerow({
                "elapsed_s": f"{elapsed_s:.6f}",
                "wall_time": wall_time,
                "payload_kg": self.payload_kg,
                "joint": index + 1,
                "motor_id": joint.motor_id,
                "model": joint.model,
                "leader_target_rad": f"{leader_targets[index]:.7f}",
                "command_rad": f"{commanded[index]:.7f}",
                "position_rad": f"{positions[index]:.7f}",
                "error_rad": f"{error:.7f}",
                "velocity_rad_s": f"{velocities[index]:.7f}",
                "torque_nm": f"{torques[index]:.7f}",
                "torque_fraction": f"{abs(torques[index]) / torque_limit:.6f}",
                "temperature_c": f"{temperatures[index]:.2f}",
                "mode": mode,
                "fault_bits": fault_bits,
                "fault_text": fault_text(fault_bits),
                "feedback_age_s": f"{motor.feedback_age:.6f}",
                "stall_candidate": int(candidate),
                "stall_confirmed": int(joint.motor_id in confirmed),
            })
        if elapsed_s - self._last_flush_s >= 0.25:
            self._stream.flush()
            self._last_flush_s = elapsed_s
        return self.detector.events[event_count:]

    def close(self, stop_reason: str) -> Path:
        if self._closed:
            return self.path.with_suffix(".summary.json")
        self._stream.flush()
        self._stream.close()
        self._closed = True
        events = sorted(
            self.detector.events,
            key=lambda event: (event.candidate_since_s, event.confirmed_at_s, event.motor_id),
        )
        clean_peaks = {
            str(motor_id): {
                name: value if math.isfinite(value) else None
                for name, value in values.items()
            }
            for motor_id, values in self.peaks.items()
        }
        summary = {
            "started_at": self.started_at.isoformat(timespec="seconds"),
            "payload_kg": self.payload_kg,
            "stop_reason": stop_reason,
            "thresholds": asdict(self.thresholds),
            "first_stall": asdict(events[0]) if events else None,
            "stall_events": [asdict(event) for event in events],
            "motor_peaks": clean_peaks,
            "csv": str(self.path),
        }
        summary_path = self.path.with_suffix(".summary.json")
        with summary_path.open("w") as stream:
            json.dump(summary, stream, indent=2)
        return summary_path
