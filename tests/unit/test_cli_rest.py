"""The rest command on simulated motors: an arm left energized or one with torque off."""

import signal

import pytest

from conftest import sdk
from test_cli_bench import Bench  # noqa: F401  (used by the fixture)

CLI = sdk._product.CLI_NAME


@pytest.fixture
def bench(monkeypatch, capsys, tmp_path):
    return Bench(monkeypatch, capsys, tmp_path)


def stiff(motors):
    return [m for name, m in motors.items() if name != "gripper"]


def test_rest_takes_over_an_energized_arm_brings_it_down_and_releases(bench):
    motors = bench.add_arm(enabled=True, params_supported=True)
    doc = bench.json("rest", "--yes", "--speed", "200")
    assert doc["ok"] and doc["attached"] is True and doc["then"] == "rest" and doc["reached"]
    for m in stiff(motors):
        assert "enable" not in m.log and m.frames[0][1] > 0   # never re-enabled, no zero-gain first frame
    assert all(abs(v - g) < 0.05 for v, g in zip(doc["final"].values(), doc["goal"].values()))
    assert not any(m.enabled for m in motors.values())


def test_rest_enables_an_arm_with_torque_off(bench):
    motors = bench.add_arm(params_supported=True)
    doc = bench.json("rest", "--yes", "--speed", "200", "--pose", "zero")
    assert doc["ok"] and doc["attached"] is False and doc["then"] == "rest"
    assert all("enable" in m.log for m in motors.values()) and not any(m.enabled for m in motors.values())


def test_rest_refuses_without_yes_and_sends_nothing(bench):
    motors = bench.add_arm(enabled=True, params_supported=True)
    doc = bench.json("rest", code=1)
    assert "refusing without --yes" in doc["error"]
    assert all(m.enabled and not m.frames and "disable" not in m.log for m in motors.values())


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_a_signal_during_rest_keeps_the_arm_holding(bench, monkeypatch, sig):
    motors = bench.add_arm(enabled=True, params_supported=True)
    move_to = sdk.arm.Arm.move_to

    def interrupted(self, *args, **kwargs):
        signal.raise_signal(sig)
        return move_to(self, *args, **kwargs)

    monkeypatch.setattr(sdk.arm.Arm, "move_to", interrupted)
    code, out, err = bench.run("rest", "--yes", "--json")
    assert code == 1 and '"then": "holding"' in out
    assert all(m.enabled and "disable" not in m.log for m in stiff(motors))
    assert f"{CLI} rest" in err + out


def test_unverified_arrival_keeps_holding_and_names_the_rest_command(bench, monkeypatch):
    motors = bench.add_arm(enabled=True, params_supported=True)
    monkeypatch.setattr(sdk.arm.Arm, "stop", lambda self: (self.hold_current_position(), False)[1])
    code, out, err = bench.run("rest", "--yes", "--speed", "200", "--json")
    assert code == 1 and '"then": "holding"' in out
    assert all(m.enabled and "disable" not in m.log for m in stiff(motors))
    assert f"'{CLI} rest" in err and "drops" in err
