import subprocess
import sys

from conftest import sdk


def test_import():
    assert sdk.__version__ == "0.1.0"
    assert set(sdk.__all__) == {"Arm", "ArmState", "ArmConfig", "JointConfig", "Chain", "Teleop", "Camera",
                                "list_cameras", "ArmError", "ConnectError", "FaultLatched", "ParamTimeout",
                                "StateError"}
    assert all(hasattr(sdk, name) for name in sdk.__all__)


def test_import_loads_no_optional_dependency():
    code = (f"import sys, {sdk.__name__}; "
            "loaded = [m for m in ('cv2', 'usb', 'motorbridge') if any(k == m or k.startswith(m + '.') for k in sys.modules)]; "
            "assert not loaded, loaded")
    subprocess.run([sys.executable, "-c", code], check=True)
