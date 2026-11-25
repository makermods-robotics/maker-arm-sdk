#!/usr/bin/env python3
"""Files shared byte-for-byte between the sibling arm SDK repos, and a check / copy between them.

Each repo holds one package (the directory with ``_product.py``); paths below write it as ``pkg/``.
Every ``.py`` file in the package must be listed here as shared or specific (a test enforces it).

    python tools/sync_shared.py --check ../other-arm-sdk      # exit 1 listing differing / missing files
    python tools/sync_shared.py --copy-to ../other-arm-sdk    # mirror this repo's shared files there
    ... --exclude 'pkg/cli/*'                                  # leave matching paths out (repeatable)
"""

import argparse
import filecmp
import fnmatch
import shutil
import sys
from pathlib import Path

PKG = "pkg"

# Byte-identical in both repos (globs; a path may exist in neither repo yet).
SHARED = [
    "pkg/__init__.py", "pkg/arm.py", "pkg/bus.py", "pkg/cameras.py", "pkg/config.py", "pkg/errors.py",
    "pkg/gravity.py", "pkg/gripper.py", "pkg/kinematics.py", "pkg/mapping.py", "pkg/realtime.py", "pkg/rig.py",
    "pkg/_planner.py",
    "pkg/stall.py", "pkg/teleop.py", "pkg/testing.py", "pkg/trajectory_ik.py", "pkg/trajectory_ik_jax.py",
    "pkg/cli/*.py", "pkg/leaders/*.py", "pkg/transport/*.py", "pkg/profiles/__init__.py",
    "tests/conftest.py",
    "tests/unit/chain_fixture.py", "tests/unit/mjpeg_server.py", "tests/unit/teleop_fakes.py",
    "tests/unit/test_arm.py", "tests/unit/test_arm_api.py", "tests/unit/test_arm_model.py",
    "tests/unit/test_attach.py",
    "tests/unit/test_bus.py",
    "tests/unit/test_cameras.py", "tests/unit/test_check_all_motors.py", "tests/unit/test_cli*.py",
    "tests/unit/test_config.py", "tests/unit/test_errors.py", "tests/unit/test_gravity.py",
    "tests/unit/test_gripper.py", "tests/unit/test_import.py", "tests/unit/test_kinematics.py",
    "tests/unit/test_layout.py", "tests/unit/test_leaders.py", "tests/unit/test_mapping.py", "tests/unit/test_planner.py", "tests/unit/test_realtime.py",
    "tests/unit/test_rig.py", "tests/unit/test_slcan_codec.py", "tests/unit/test_socketcan_frames.py",
    "tests/unit/test_stall.py", "tests/unit/test_sync_shared.py", "tests/unit/test_teleop.py",
    "tests/unit/test_testing.py",
    "tests/unit/test_trajectory_ik.py", "tests/unit/test_trajectory_ik_jax.py",
    "tests/unit/test_transport_mock.py",
    "tools/sync_shared.py", "scripts/linux/setup_slcan.sh",
]

# Package files that differ per repo (protocol, identity, motor simulation); profiles/* data and urdf/* are specific too.
SPECIFIC = ["pkg/_product.py", "pkg/_sim_motor.py", "pkg/motor.py", "pkg/protocol.py"]


def package_dir(root: Path) -> str:
    found = sorted(p.parent.name for p in root.glob("*/_product.py"))
    if len(found) != 1:
        raise SystemExit(f"{root}: expected one package with _product.py, found {found or 'none'}")
    return found[0]


def _real(root: Path, neutral: str) -> Path:
    head, _, rest = neutral.partition("/")
    return root / (package_dir(root) if head == PKG else head) / rest


def shared_files(root: Path, exclude=()) -> set[str]:
    """Neutral paths (``pkg/...``) of the shared files present in ``root``."""
    out = set()
    pkg = package_dir(root)
    for pattern in SHARED:
        head, _, rest = pattern.partition("/")
        base = pkg if head == PKG else head
        for path in root.glob(f"{base}/{rest}"):
            if path.is_file() and "__pycache__" not in path.parts:
                neutral = path.relative_to(root).as_posix()
                if head == PKG:
                    neutral = f"{PKG}/{neutral.partition('/')[2]}"
                out.add(neutral)
    return {p for p in out if not any(fnmatch.fnmatch(p, x) for x in exclude)}


def unclassified(root: Path) -> list[str]:
    """Package .py files listed neither as shared nor as specific."""
    pkg = package_dir(root)
    listed = shared_files(root) | set(SPECIFIC)
    out = []
    for path in sorted((root / pkg).rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        neutral = f"{PKG}/{path.relative_to(root / pkg).as_posix()}"
        if neutral not in listed:
            out.append(neutral)
    return out


def check(here: Path, other: Path, exclude=()) -> list[str]:
    mine, theirs = shared_files(here, exclude), shared_files(other, exclude)
    problems = []
    for neutral in sorted(mine | theirs):
        if neutral not in theirs:
            problems.append(f"missing in {other}: {neutral}")
        elif neutral not in mine:
            problems.append(f"missing in {here}: {neutral}")
        elif not filecmp.cmp(_real(here, neutral), _real(other, neutral), shallow=False):
            problems.append(f"differs: {neutral}")
    return problems


def copy_to(here: Path, other: Path, exclude=()) -> list[str]:
    copied = []
    for neutral in sorted(shared_files(here, exclude)):
        src, dst = _real(here, neutral), _real(other, neutral)
        if dst.is_file() and filecmp.cmp(src, dst, shallow=False):
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        copied.append(neutral)
    return copied


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    action = ap.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", metavar="OTHER_REPO", type=Path)
    action.add_argument("--copy-to", metavar="OTHER_REPO", type=Path)
    action.add_argument("--list", action="store_true", help="print this repo's shared files")
    ap.add_argument("--exclude", action="append", default=[], metavar="GLOB",
                    help="leave out neutral paths matching GLOB, e.g. 'pkg/cli/*'")
    ap.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1],
                    help="this repo (default: the one holding this script)")
    a = ap.parse_args(argv)
    here = a.repo.resolve()
    if a.exclude:
        print(f"excluded: {', '.join(a.exclude)}")
    if a.list:
        print("\n".join(sorted(shared_files(here, a.exclude))))
        return 0
    if a.copy_to:
        copied = copy_to(here, a.copy_to.resolve(), a.exclude)
        print("\n".join(f"copied: {p}" for p in copied) or "nothing to copy")
        stale = sorted(shared_files(a.copy_to.resolve(), a.exclude) - shared_files(here, a.exclude))
        for p in stale:
            print(f"only in {a.copy_to}: {p} (not removed)")
        return 0
    problems = check(here, a.check.resolve(), a.exclude)
    for p in problems:
        print(p)
    if not problems:
        print(f"shared files identical: {here} == {a.check.resolve()}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
