import sys
from types import SimpleNamespace

from conftest import sdk

safety = sdk.cli.safety
release_if_holding = safety.release_if_holding


class FakeArm:
    def __init__(self, state_name: str, rest_ok: bool = True, interrupt: bool = False, unsafe=(),
                 watchdogs=None):
        self.state = SimpleNamespace(name=state_name)
        self.rest_ok = rest_ok
        self.interrupt = interrupt
        self.unsafe = list(unsafe)
        self.watchdogs = watchdogs or {}
        self.joint_names = ["j1", "gripper"]
        self.unconfirmed_disable = []
        self.calls = []

    def stop(self):
        self.calls.append("stop")
        if self.interrupt:
            raise KeyboardInterrupt
        return self.rest_ok

    def disable(self):
        self.calls.append("disable")

    def hold_current_position(self):
        self.calls.append("hold")
        return True

    def detach_unsafe(self):
        return self.unsafe

    def detach(self):
        self.calls.append("detach")
        return dict(self.watchdogs)


def no_prompt(monkeypatch):
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: (_ for _ in ()).throw(AssertionError("must not prompt")),
    )


def terminal(monkeypatch, answer="RELEASE"):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    monkeypatch.setattr("builtins.input", lambda _prompt: answer)


def test_pre_enable_failure_never_broadcasts_disable(monkeypatch):
    arm = FakeArm("CONNECTED")
    no_prompt(monkeypatch)
    assert release_if_holding(arm) == "not_enabled"
    assert arm.calls == []


def test_verified_return_to_rest_releases_without_prompt(monkeypatch):
    arm = FakeArm("ENABLED", rest_ok=True)
    no_prompt(monkeypatch)
    assert release_if_holding(arm) == "rest"
    assert arm.calls == ["stop"]


def test_unverified_return_requires_release_confirmation(monkeypatch):
    arm = FakeArm("FAULT", rest_ok=False)
    terminal(monkeypatch)
    assert release_if_holding(arm) == "released"
    assert arm.calls == ["stop", "disable"]


def test_unconfirmed_release_is_reported_as_such(monkeypatch):
    arm = FakeArm("FAULT", rest_ok=False)
    arm.unconfirmed_disable = [2]
    terminal(monkeypatch)
    assert release_if_holding(arm) == "unconfirmed"


def test_unverified_return_without_a_terminal_leaves_the_arm_holding(monkeypatch, capsys):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False, raising=False)
    no_prompt(monkeypatch)
    arm = FakeArm("ENABLED", rest_ok=False, watchdogs={"gripper": 0.5})
    messages = []
    assert release_if_holding(arm, say=messages.append) == "holding"
    assert "disable" not in arm.calls and arm.calls[-2:] == ["hold", "detach"]
    assert "gripper releases after 0.5 s" in messages[-1] and "j1 keep holding" in messages[-1]


def test_second_interrupt_during_return_never_releases(monkeypatch):
    no_prompt(monkeypatch)
    arm = FakeArm("ENABLED", interrupt=True)
    assert release_if_holding(arm, say=lambda *_: None) == "holding"
    assert "disable" not in arm.calls and "detach" in arm.calls


def test_holding_process_stays_when_detach_would_drop_a_stiff_joint(monkeypatch):
    # an armed watchdog on a stiff joint: the process keeps holding until RELEASE is typed
    terminal(monkeypatch)
    arm = FakeArm("ENABLED", interrupt=True, unsafe=["j1"])
    assert release_if_holding(arm, say=lambda *_: None) == "released"
    assert "detach" not in arm.calls and arm.calls[-1] == "disable"
