import math
import json
from types import SimpleNamespace

from maker_arm.stall import MotorStateRecorder, StallDetector, StallThresholds


def sample(detector, now, motor_id=1, *, error=0.2, velocity=0.01, torque=2.0,
           feedback_age=0.01, mode=2, fault_bits=0):
    return detector.update(
        now,
        motor_id,
        error=error,
        velocity=velocity,
        torque=torque,
        feedback_age=feedback_age,
        mode=mode,
        fault_bits=fault_bits,
    )


def test_persistent_candidate_confirms_once():
    detector = StallDetector([1], StallThresholds(persistence_s=0.5))
    assert sample(detector, 1.0)
    assert sample(detector, 1.49)
    assert detector.events == []
    assert sample(detector, 1.5)
    assert len(detector.events) == 1
    assert detector.events[0].motor_id == 1
    assert detector.events[0].candidate_since_s == 1.0
    sample(detector, 2.0)
    assert len(detector.events) == 1


def test_candidate_timer_resets_when_motor_moves_again():
    detector = StallDetector([1], StallThresholds(persistence_s=0.5))
    sample(detector, 0.0)
    assert not sample(detector, 0.3, velocity=0.2)
    sample(detector, 0.4)
    sample(detector, 0.8)
    assert detector.events == []
    sample(detector, 0.9)
    assert len(detector.events) == 1
    assert detector.events[0].candidate_since_s == 0.4


def test_candidate_requires_error_low_speed_torque_and_fresh_healthy_feedback():
    detector = StallDetector([1], StallThresholds())
    cases = [
        {"error": 0.01},
        {"velocity": 0.2},
        {"torque": 0.1},
        {"feedback_age": 0.3},
        {"mode": 0},
        {"fault_bits": 1},
        {"error": math.nan},
    ]
    for index, overrides in enumerate(cases):
        assert not sample(detector, float(index), **overrides)


def test_independent_motors_preserve_confirmation_order():
    detector = StallDetector([1, 2], StallThresholds(persistence_s=0.5))
    sample(detector, 0.0, motor_id=2)
    sample(detector, 0.2, motor_id=1)
    sample(detector, 0.5, motor_id=2)
    sample(detector, 0.7, motor_id=1)
    assert [event.motor_id for event in detector.events] == [2, 1]


def test_recorder_writes_all_motors_and_summary(tmp_path):
    feedback = SimpleNamespace(mode=2, fault_bits=0)
    motors = [
        SimpleNamespace(feedback=feedback, feedback_age=0.01,
                        params=SimpleNamespace(t_min=-14.0, t_max=14.0)),
        SimpleNamespace(feedback=feedback, feedback_age=0.01,
                        params=SimpleNamespace(t_min=-17.0, t_max=17.0)),
    ]
    arm = SimpleNamespace(
        config=SimpleNamespace(joints=[
            SimpleNamespace(motor_id=1, model="RS00"),
            SimpleNamespace(motor_id=2, model="RS02"),
        ]),
        motors=motors,
        get_commanded_positions=lambda: [1.0, 0.0],
        get_joint_positions=lambda: [0.7, 0.0],
        get_joint_velocities=lambda: [0.01, 0.2],
        get_joint_torques=lambda: [2.0, 0.1],
        get_temperatures=lambda: [35.0, 36.0],
    )
    path = tmp_path / "states.csv"
    recorder = MotorStateRecorder(
        path, 1.5, arm, StallThresholds(persistence_s=0.5)
    )
    assert recorder.record(0.0, [1.2, 0.0]) == []
    events = recorder.record(0.5, [1.2, 0.0])
    assert [event.motor_id for event in events] == [1]
    summary_path = recorder.close("stall_CAN1")

    assert len(path.read_text().splitlines()) == 5  # header + 2 motors × 2 ticks
    summary = json.loads(summary_path.read_text())
    assert summary["payload_kg"] == 1.5
    assert summary["first_stall"]["motor_id"] == 1
    assert summary["motor_peaks"]["1"]["max_abs_torque_nm"] == 2.0
