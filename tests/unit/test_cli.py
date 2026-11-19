import importlib
import sys

import pytest

from conftest import sdk

cli = sdk.cli.main


def test_top_level_help_lists_supported_commands(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["cli", "--help"])
    cli.main()
    output = capsys.readouterr().out
    for command in cli.COMMANDS:
        assert command in output
    assert "windows" not in output.lower()
    assert "assign-id" not in output


def test_unknown_command_exits_with_usage(capsys):
    with pytest.raises(SystemExit, match="unknown command 'fly'"):
        cli.main(["fly"])


@pytest.mark.parametrize("command", list(cli.COMMANDS))
def test_every_command_has_help_and_json(command, capsys):
    with pytest.raises(SystemExit) as e:
        cli.main([command, "--help"])
    assert e.value.code == 0
    text = capsys.readouterr().out
    assert "--json" in text
    module = importlib.import_module(f".{cli.COMMANDS[command][0]}", sdk.cli.__name__)
    assert module.__doc__ and callable(module.main)


def test_json_failure_document_and_exit_code(monkeypatch, tmp_path, capsys):
    import json
    monkeypatch.setenv(sdk.rig.ENV_VAR, str(tmp_path / "rig.yaml"))
    with pytest.raises(SystemExit) as e:
        cli.main(["scan", "--arm", "nowhere", "--json"])
    assert e.value.code == 1
    doc = json.loads(capsys.readouterr().out)
    assert doc == {"command": "scan", "ok": False, "error": doc["error"]} and "nowhere" in doc["error"]
    with pytest.raises(SystemExit) as e:
        cli.main(["scan", "--arm", "nowhere"])
    captured = capsys.readouterr()
    assert e.value.code == 1 and captured.out == "" and "no arm named 'nowhere'" in captured.err


def test_clean_makes_json_safe():
    clean = sdk.cli.common.clean
    path = sdk.rig.Path("/x")
    assert clean({"a": float("nan"), "b": (1.0, float("inf")), 3: [path]}) == \
        {"a": None, "b": [1.0, None], "3": [str(path)]}
