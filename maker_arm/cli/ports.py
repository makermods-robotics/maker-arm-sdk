"""List what is plugged in: CAN adapters, cameras and serial ports. No bus traffic, no device opened.

Use the ids printed here in the rig file: gs_usb adapters as gs_usb:<serial>, cameras as usb:<path>.
"""

from __future__ import annotations

import os
from pathlib import Path

from .. import cameras
from ..rig import slcan_candidates
from .common import make_parser, run

NET = Path("/sys/class/net")
SERIAL_BY_ID = Path("/dev/serial/by-id")
ARPHRD_CAN = "280"


def gs_usb_adapters() -> tuple[list[dict], str | None]:
    """(adapters, why the list is unavailable)."""
    try:
        from ..transport.gs_usb import list_adapters
        return list_adapters(), None
    except Exception as e:   # pyusb or libusb missing, USB permission
        return [], str(e)


def socketcan_interfaces() -> list[dict]:
    out = []
    try:
        entries = sorted(NET.iterdir())
    except OSError:
        return out
    for entry in entries:
        try:
            if (entry / "type").read_text().strip() != ARPHRD_CAN:
                continue
            state = (entry / "operstate").read_text().strip()
        except OSError:
            continue
        driver = None
        try:
            driver = os.path.basename(os.readlink(entry / "device" / "driver"))
        except OSError:
            pass
        out.append({"name": entry.name, "state": state, "driver": driver})
    return out


def serial_ports() -> list[dict]:
    """Serial devices (USB-SLCAN adapters, Star leaders) with their stable /dev/serial/by-id link."""
    by_id = {}
    try:
        for link in SERIAL_BY_ID.iterdir():
            by_id[os.path.realpath(link)] = str(link)
    except OSError:
        pass
    return [{"device": d, "by_id": by_id.get(os.path.realpath(d))} for d in slcan_candidates()]


def camera_list() -> tuple[list[dict], str | None]:
    try:
        return [{"id": c.id, "index": c.index, "name": c.name, "path": c.path} for c in cameras.list_cameras()], None
    except Exception as e:
        return [], str(e)


def discover() -> dict:
    adapters, adapter_error = gs_usb_adapters()
    cams, camera_error = camera_list()
    return {"gs_usb": adapters, "gs_usb_error": adapter_error, "socketcan": socketcan_interfaces(),
            "serial": serial_ports(), "cameras": cams, "cameras_error": camera_error}


def print_discovery(found: dict, say) -> None:
    say("gs_usb CAN adapters:" + ("" if found["gs_usb"] else
                                  f" none{' (' + found['gs_usb_error'] + ')' if found['gs_usb_error'] else ''}"))
    for ad in found["gs_usb"]:
        say(f"  {ad['port']:<40} usb {ad['usb_path']:<10} {ad['product']}")
    say("SocketCAN interfaces:" + ("" if found["socketcan"] else " none"))
    for itf in found["socketcan"]:
        say(f"  {itf['name']:<8} {itf['state']:<8} driver {itf['driver'] or '?'}")
    say("serial ports (USB-SLCAN adapters, Star leaders):" + ("" if found["serial"] else " none"))
    for sp in found["serial"]:
        say(f"  {sp['device']:<16} {sp['by_id'] or ''}")
    say("cameras:" + ("" if found["cameras"] else
                      f" none{' (' + found['cameras_error'] + ')' if found['cameras_error'] else ''}"))
    for cam in found["cameras"]:
        say(f"  {cam['id']:<48} {cam['path'] or ''} {cam['name']}")


def body(a, out) -> dict:
    found = discover()
    print_discovery(found, out.say)
    return found


def main() -> None:
    a = make_parser(__doc__, arms=None).parse_args()
    run("ports", a, body)


if __name__ == "__main__":
    main()
