from types import SimpleNamespace

from maker_arm.cli.safety import release_if_holding


class FakeArm:
    def __init__(self, state_name: str, rest_ok: bool = True):
        self.state = SimpleNamespace(name=state_name)
        self.rest_ok = rest_ok
        self.disable_calls = 0
        self.stop_calls = 0

    def stop(self):
        self.stop_calls += 1
        return self.rest_ok

    def disable(self):
        self.disable_calls += 1


def no_prompt(monkeypatch):
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: (_ for _ in ()).throw(AssertionError("must not prompt")),
    )


def test_pre_enable_failure_never_broadcasts_disable(monkeypatch):
    arm = FakeArm("CONNECTED")
    no_prompt(monkeypatch)
    assert release_if_holding(arm) is False
    assert arm.disable_calls == 0 and arm.stop_calls == 0


def test_verified_return_to_rest_releases_without_prompt(monkeypatch):
    arm = FakeArm("ENABLED", rest_ok=True)
    no_prompt(monkeypatch)
    assert release_if_holding(arm) is True
    assert arm.stop_calls == 1


def test_unverified_return_requires_release_confirmation(monkeypatch):
    arm = FakeArm("FAULT", rest_ok=False)
    monkeypatch.setattr("builtins.input", lambda _prompt: "RELEASE")
    assert release_if_holding(arm) is True
    assert arm.disable_calls == 1
