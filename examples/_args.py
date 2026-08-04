import argparse

from maker_arm.arm import Arm


def make_parser(desc: str) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--config", default="configs/maker_arm_6dof.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    return ap


def arm_from_args(a) -> Arm:
    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    return Arm.from_yaml(a.config, backend=a.backend, **kw)
