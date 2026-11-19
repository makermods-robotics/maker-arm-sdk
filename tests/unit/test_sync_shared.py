"""tools/sync_shared.py classifies every package file and checks / copies shared files between repos."""

import importlib.util
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("sync_shared", ROOT / "tools" / "sync_shared.py")
sync = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sync)


def test_every_package_file_is_shared_or_specific():
    assert sync.unclassified(ROOT) == [], "add new package files to SHARED or SPECIFIC in tools/sync_shared.py"


def test_specific_files_exist_and_are_not_shared():
    shared = sync.shared_files(ROOT)
    for neutral in sync.SPECIFIC:
        assert sync._real(ROOT, neutral).is_file()
        assert neutral not in shared


def test_the_script_and_its_test_are_shared():
    shared = sync.shared_files(ROOT)
    assert {"tools/sync_shared.py", "tests/unit/test_sync_shared.py", "pkg/arm.py"} <= shared


@pytest.fixture
def two_repos(tmp_path):
    def repo(name, pkg):
        root = tmp_path / name
        (root / pkg / "cli").mkdir(parents=True)
        (root / pkg / "_product.py").write_text(f"PACKAGE = {pkg!r}\n")
        (root / pkg / "arm.py").write_text("shared\n")
        (root / pkg / "cli" / "main.py").write_text("cli\n")
        (root / pkg / "motor.py").write_text(f"{pkg} motor\n")
        return root
    return repo("a", "alpha"), repo("b", "beta")


def test_check_and_copy_between_two_repos(two_repos):
    a, b = two_repos
    assert sync.check(a, b) == []
    (a / "alpha" / "arm.py").write_text("changed\n")
    (a / "alpha" / "gravity.py").write_text("new\n")
    (b / "beta" / "cli" / "main.py").write_text("other cli\n")
    assert sync.check(a, b) == ["differs: pkg/arm.py", "differs: pkg/cli/main.py",
                                f"missing in {b}: pkg/gravity.py"]
    assert sync.check(a, b, exclude=["pkg/cli/*"]) == ["differs: pkg/arm.py", f"missing in {b}: pkg/gravity.py"]
    assert sync.copy_to(a, b, exclude=["pkg/cli/*"]) == ["pkg/arm.py", "pkg/gravity.py"]
    assert sync.check(a, b, exclude=["pkg/cli/*"]) == []
    assert (b / "beta" / "motor.py").read_text() == "beta motor\n"   # specific files untouched
    assert sync.main(["--repo", str(a), "--check", str(b)]) == 1
    sync.main(["--repo", str(a), "--copy-to", str(b)])
    assert sync.main(["--repo", str(a), "--check", str(b)]) == 0


def test_a_repo_needs_exactly_one_package(tmp_path):
    with pytest.raises(SystemExit):
        sync.package_dir(tmp_path)
    shutil.rmtree(tmp_path)
