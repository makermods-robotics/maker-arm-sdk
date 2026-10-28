"""Shared CLI arguments: CAN backend selection with platform-aware defaults."""

import argparse
import glob
import platform

from maker_arm.arm import Arm
from maker_arm.profiles import DEFAULT_ARM_CONFIG
from maker_arm.transport.base import CanBackend, create_backend


def default_backend() -> str:
    """SocketCAN on Linux; serial SLCAN elsewhere (macOS has no SocketCAN)."""
    return "socketcan" if platform.system() == "Linux" else "slcan"


def slcan_candidates() -> list[str]:
    if platform.system() == "Darwin":
        return sorted(glob.glob("/dev/cu.usbmodem*"))
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def add_backend_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default=default_backend(),
                    help="CAN backend (default: socketcan on Linux, slcan on macOS)")
    ap.add_argument("--channel", default="can0", help="SocketCAN interface (default: can0)")
    ap.add_argument("--port", help="SLCAN serial device (default: the only USB CAN adapter found)")
    ap.add_argument("--serial-baudrate", type=int, default=115200,
                    help="SLCAN serial baud rate (default: 115200; ignored by USB CDC adapters)")
    ap.add_argument("--slcan-startup-delay", type=float, default=2.5,
                    help="seconds to wait after opening an SLCAN serial device (default: 2.5)")


def make_parser(desc: str) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    add_backend_args(ap)
    return ap


def resolve_port(a) -> str:
    if a.port:
        return a.port
    found = slcan_candidates()
    if len(found) == 1:
        print(f"using CAN adapter {found[0]}")
        return found[0]
    if not found:
        raise SystemExit("no USB CAN adapter found; plug it in or pass --port "
                         "(on Linux with SocketCAN use --backend socketcan)")
    raise SystemExit(f"several serial devices found ({', '.join(found)}); pass --port")


def backend_kwargs(a) -> dict:
    if a.backend == "socketcan":
        return {"channel": a.channel}
    return {"port": resolve_port(a), "baudrate": a.serial_baudrate, "rtscts": False,
            "startup_delay": a.slcan_startup_delay}


def backend_from_args(a) -> CanBackend:
    return create_backend(a.backend, **backend_kwargs(a))


def arm_from_args(a) -> Arm:
    return Arm.from_yaml(a.config, backend=a.backend, **backend_kwargs(a))
