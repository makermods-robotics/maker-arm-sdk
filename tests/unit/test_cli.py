import sys

from maker_arm.cli import main as cli


def test_top_level_help_lists_supported_commands(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["maker-arm", "--help"])
    cli.main()
    output = capsys.readouterr().out
    for command in ("doctor", "scan", "assign-id", "zero", "check", "teleop"):
        assert command in output
    assert "windows" not in output.lower()
