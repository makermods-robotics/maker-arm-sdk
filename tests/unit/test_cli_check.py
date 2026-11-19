"""check: the per-joint jog and the sine tracking test, on simulated motors."""

import csv
import math
import threading
import time
from types import SimpleNamespace

import pytest

from conftest import sdk
from fakes import FakeMotor, classify, make_arm
from test_cli_bench import Bench

check = sdk.cli.check


@pytest.fixture
def bench(monkeypatch, capsys, tmp_path):
    return Bench(monkeypatch, capsys, tmp_path)


class LaggingMotor(FakeMotor):
    """An enabled motor that reaches each commanded position ``lag`` seconds later."""

    def __init__(self, motor_id, model, position, lag):
        super().__init__(motor_id, model, position=position)
        self.lag = lag
        self.history = [(-math.inf, position)]

    def handle(self, can_id, data):
        before = self.position
        replies = super().handle(can_id, data)
        if classify(can_id, data)[0] != "command" or not self.enabled or self.last_kp <= 0:
            return replies
        now = time.monotonic()
        self.history.append((now, self.position))
        self.position = before
        for stamp, value in self.history:
            if stamp <= now - self.lag:
                self.position = value
        self.history = [h for h in self.history if h[0] > now - 1.0] or self.history[-1:]
        return [self.feedback()]


def test_jog_one_motor_passes(bench):
    motors = bench.add_arm()
    doc = bench.json("check", "--motor", "4", "--move-seconds", "0.5", "--return-seconds", "0.5", "--yes")
    assert doc["mode"] == "jog" and doc["joints"]["wrist_flex"]["passed"] and doc["then"] == "rest"
    assert not any(m.enabled for m in motors.values())


def test_jog_fails_on_a_motor_that_does_not_move(bench):
    bench.add_arm(special={"wrist_flex": lambda i, model, p: FakeMotor(i, model, position=p, track=False)})
    doc = bench.json("check", "--motor", "4", "--move-seconds", "0.5", "--return-seconds", "0.5", "--yes", code=1)
    assert doc["ok"] is False and doc["joints"]["wrist_flex"]["passed"] is False


def test_sine_measures_a_known_lag(bench, tmp_path):
    lag = 0.05
    bench.add_arm(special={"elbow_flex": lambda i, model, p: LaggingMotor(i, model, p, lag)})
    path = tmp_path / "sine.csv"
    doc = bench.json("check", "--sine", "5", "--period", "0.5", "--cycles", "3", "--joints", "elbow_flex",
                     "--csv", path, "--yes")
    assert doc["mode"] == "sine" and doc["then"] == "rest" and list(doc["joints"]) == ["elbow_flex"]
    elbow = doc["joints"]["elbow_flex"]
    # the control loop adds a few ticks on top of the motor's own lag, more on a loaded machine
    assert lag - 0.005 <= elbow["lag_s"] <= lag + 0.08
    omega, amplitude = 2 * math.pi / 0.5, math.radians(5)
    expected_rms = amplitude * 2 * math.sin(omega * elbow["lag_s"] / 2) / math.sqrt(2)
    assert elbow["rms_error"] == pytest.approx(expected_rms, rel=0.35)
    assert elbow["max_error"] >= elbow["rms_error"] and elbow["temperature_rise"] == 0.0
    with open(path) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == doc["samples"] > 100
    assert set(rows[0]) == {"t", "elbow_flex_cmd", "elbow_flex_pos", "elbow_flex_torque", "elbow_flex_temp"}


def test_sine_refuses_limits_and_speed(bench):
    motors = bench.add_arm()
    doc = bench.json("check", "--sine", "30", "--joints", "gripper", "--period", "4", "--cycles", "1", "--yes", code=1)
    assert "leaves its limits" in doc["error"]
    doc = bench.json("check", "--sine", "40", "--period", "0.1", "--yes", code=1)
    assert "peak speed" in doc["error"]
    assert not any(m.enabled for m in motors.values())


def test_sine_targets_ease_in():
    q = check.sine_targets([0.0, 1.0], [0], 1.0, 2.0, 0.25)
    assert q[1] == 1.0 and q[0] == pytest.approx(0.25 * math.sin(math.pi / 4))


def test_lag_estimate_recovers_a_shift():
    import numpy as np
    t = np.arange(0, 2, 0.005)
    cmd = np.sin(2 * math.pi * t)
    pos = np.sin(2 * math.pi * (t - 0.04))
    assert check.lag_estimate(t, cmd, pos, 1.0, 0.5) == pytest.approx(0.04, abs=0.0015)


def test_jog_stops_between_steps_when_interrupted(monkeypatch):
    monkeypatch.setattr(sdk.arm.Arm, "PARAM_TIMEOUT", 0.005)
    arm, _, _ = make_arm()
    arm.connect()
    arm.enable(start_loop=False)
    stop = threading.Event()
    stop.set()
    a = SimpleNamespace(motor=None, delta=0.05, minimum_response=0.008, move_seconds=1.0, return_seconds=1.0)
    targets = list(arm._user_targets)
    with pytest.raises(sdk.cli.common.CliError, match="interrupted"):
        check.jog(arm, a, print, stop)
    assert arm._user_targets == pytest.approx(targets, abs=1e-3)   # no jog target went out
    arm.disable()
