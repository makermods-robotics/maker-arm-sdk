import argparse

from maker_arm.arm import Arm
from maker_arm.profiles import DEFAULT_ARM_CONFIG


def make_parser(desc: str) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--serial-baudrate", type=int,
                    help="SLCAN serial transport baud rate (default: 115200)")
    ap.add_argument("--slcan-startup-delay", type=float, default=2.5,
                    help="seconds to wait after opening an SLCAN serial device (default: 2.5)")
    return ap


def arm_from_args(a) -> Arm:
    if a.backend == "socketcan":
        kw = {"channel": a.channel}
    elif a.backend == "slcan":
        kw = {
            "port": a.port,
            "baudrate": a.serial_baudrate or 115200,
            "rtscts": False,
            "startup_delay": a.slcan_startup_delay,
        }
    return Arm.from_yaml(a.config, backend=a.backend, **kw)
