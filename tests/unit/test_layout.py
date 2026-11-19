"""Shared files stay product-neutral so they can be byte-identical across the arm SDKs."""

import ast
import importlib
import importlib.util
from pathlib import Path

import pytest

from conftest import sdk

PKG = Path(sdk.__file__).resolve().parent
ROOT = PKG.parent
product = sdk._product
_spec = importlib.util.spec_from_file_location("sync_shared", ROOT / "tools" / "sync_shared.py")
_sync = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_sync)
SHARED = sorted(p.partition("/")[2] for p in _sync.shared_files(ROOT) if p.startswith(f"{_sync.PKG}/"))
TESTS = sorted(p for p in _sync.shared_files(ROOT) if p.startswith("tests/"))
NAMES = {product.PACKAGE, product.DIST_NAME, product.CLI_NAME, product.DISPLAY_NAME,
         product.DISPLAY_NAME.split()[0].lower(), product.MOTOR_VENDOR.lower()}


@pytest.mark.parametrize("name", SHARED)
def test_shared_file_is_product_neutral(name):
    text = (PKG / name).read_text()
    lowered = text.lower()
    assert not [n for n in NAMES if n.lower() in lowered]
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            assert node.level > 0 or not node.module.startswith(product.PACKAGE)
            assert node.module.split(".")[-1] != "protocol"
        if isinstance(node, ast.Import):
            assert not any(a.name.split(".")[0] == product.PACKAGE for a in node.names)
        if isinstance(node, ast.ImportFrom) and node.level and node.names:
            assert not any(a.name == "protocol" for a in node.names)


@pytest.mark.parametrize("name", TESTS)
def test_shared_test_does_not_name_the_package(name):
    text = (ROOT / name).read_text()
    assert product.PACKAGE not in text and product.CLI_NAME not in text


def test_every_cli_command_resolves_relative_to_the_package():
    for module, _ in sdk.cli.main.COMMANDS.values():
        assert callable(importlib.import_module(f".{module}", sdk.cli.__name__).main)


def test_backends_declare_their_pacing():
    base = sdk.transport.base
    assert base.CanBackend.feedback_paced_control is False
    assert base.CanBackend.feedback_wait_timeout > 0
    assert sdk.transport.mock.MockBackend().feedback_paced_control is False
    assert sdk.transport.slcan_serial.SlcanSerialBackend(port="unused").feedback_paced_control is True


def test_unknown_backend_lists_the_choices():
    with pytest.raises(ValueError, match="gs_usb"):
        sdk.transport.base.create_backend("nope")
