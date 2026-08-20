from types import SimpleNamespace

from maker_arm.cli.safety import release_if_holding


class FakeArm:
    def __init__(self, state_name: str):
        self.state = SimpleNamespace(name=state_name)
        self.disable_calls = 0

    def disable(self):
        self.disable_calls += 1


def test_pre_enable_failure_never_broadcasts_disable(monkeypatch):
    arm = FakeArm("CONNECTED")
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: (_ for _ in ()).throw(AssertionError("must not prompt")),
    )

    assert release_if_holding(arm) is False
    assert arm.disable_calls == 0


def test_enabled_arm_requires_release_confirmation(monkeypatch):
    arm = FakeArm("ENABLED")
    monkeypatch.setattr("builtins.input", lambda _prompt: "RELEASE")

    assert release_if_holding(arm) is True
    assert arm.disable_calls == 1
