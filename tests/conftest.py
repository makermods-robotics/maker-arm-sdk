"""Exposes the package under test as ``sdk`` (``from conftest import sdk``) so shared tests never
name it, with every submodule imported; tests/ is on sys.path for fakes.py."""

import importlib
import pkgutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

_PACKAGE = next(p.parent.name for p in HERE.parent.glob("*/_product.py"))
sdk = importlib.import_module(_PACKAGE)
for _info in pkgutil.walk_packages(sdk.__path__, f"{_PACKAGE}."):
    try:
        importlib.import_module(_info.name)
    except ImportError:   # a backend whose optional dependency is missing
        pass

# The control loop spaces frames on a real bus by spinning the CPU; with simulated motors that
# only starves the other threads, so the tests switch it off.
sdk.arm.Arm._busy_wait_us = staticmethod(lambda us: None)
