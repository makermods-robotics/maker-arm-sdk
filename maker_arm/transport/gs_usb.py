"""candleLight / gs_usb CAN adapters in userspace over PyUSB (no kernel gs_usb module, no python-can).

Ports are ``gs_usb:<USB serial>``; the serial survives re-plugging and USB renumbering. The adapter
firmware wedges when the host USB-resets it or keeps writing to a bus where nothing ACKs, so this
backend never issues a USB reset, bounds every USB transfer with a timeout, caps the frames awaiting
their TX echo (unacknowledged frames are never echoed) and raises a clear error instead of writing
more. On Linux the kernel gs_usb driver, when present, is detached from the selected adapter only,
and left detached on close so a quick reopen cannot race a rebind.

Wire format (Linux drivers/net/can/usb/gs_usb.c; candleLight_fw): vendor control requests on
interface 0 (1 BITTIMING, 2 MODE, 4 BT_CONST), bulk OUT 0x02 / IN 0x81 carrying 20-byte host frames
``<echo_id u32, can_id u32, dlc u8, channel u8, flags u8, reserved u8, data[8]>``; received frames
have echo_id 0xFFFFFFFF, a transmitted frame comes back with the echo_id it was sent with once it
was acknowledged on the bus. Sources: Linux gs_usb driver; gs-usb 0.3.1 / python-can 4.6.1 gs_usb
backend (bit timing at 87.5 % sample point, prop_seg 1).
"""

import logging
import math
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from ..errors import ConnectError
from .base import CanBackend, CanId

log = logging.getLogger(__package__)

# Vendor/product ids the Linux gs_usb driver binds (candleLight and compatibles).
GS_USB_IDS = frozenset({(0x1D50, 0x606F), (0x1209, 0x2323), (0x1CD2, 0x606F), (0x16D0, 0x10B8),
                        (0x1209, 0xCA01)})

BREQ_BITTIMING = 1
BREQ_MODE = 2
BREQ_BT_CONST = 4
MODE_RESET = 0
MODE_START = 1
EP_OUT = 0x02
EP_IN = 0x81
INTERFACE = 0
ECHO_RX = 0xFFFFFFFF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
FLAG_OVERFLOW = 0x01      # the adapter dropped received frames before this one
SAMPLE_POINT = 0.875
MAX_IN_FLIGHT = 10        # frames sent but not yet echoed (the Linux driver's GS_MAX_TX_URBS)

_FRAME = struct.Struct("<IIBBBB8s")   # 20 bytes, no hardware timestamp
_BT_CONST = struct.Struct("<10I")

INSTALL_HINT = ("Install pyusb and libusb (Linux: apt install libusb-1.0-0; macOS: brew install libusb; Windows: pip install libusb-package). "
                "On Linux without root, allow the adapter with a udev rule such as "
                'SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666".')


def _usb():
    try:
        import usb.core
        import usb.util
    except ImportError as e:
        raise ConnectError(f"gs_usb needs pyusb: {e}. {INSTALL_HINT}") from e
    return usb


def _libusb_backend():
    _usb()
    from usb.backend import libusb1
    backend = libusb1.get_backend()
    if backend is None and sys.platform == "darwin":
        for path in ("/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib",
                     "/usr/local/opt/libusb/lib/libusb-1.0.dylib"):
            if Path(path).is_file():
                backend = libusb1.get_backend(find_library=lambda _, path=path: path)
                if backend is not None:
                    break
    if backend is None:
        try:   # the libusb-package wheel bundles the library (installed with this package on Windows)
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except ImportError:
            pass
    if backend is None:
        raise ConnectError(f"libusb not found. {INSTALL_HINT}")
    return backend


def _find_devices() -> list:
    usb = _usb()
    return list(usb.core.find(find_all=True, backend=_libusb_backend(),
                              custom_match=lambda d: (d.idVendor, d.idProduct) in GS_USB_IDS))


def _dispose(dev) -> None:
    try:
        _usb().util.dispose_resources(dev)
    except Exception:
        log.debug("USB resource cleanup failed", exc_info=True)


def _describe(dev) -> dict:
    """Descriptor-only facts: no interface claim, no CAN channel start."""
    def text(attr):
        try:
            return getattr(dev, attr) or ""
        except Exception:
            return ""
    ports = getattr(dev, "port_numbers", None) or ()
    serial = text("serial_number")
    return {
        "serial": serial,
        "port": f"gs_usb:{serial}" if serial else "",
        "usb_path": f"{dev.bus}-{'.'.join(map(str, ports))}" if ports else f"{dev.bus}:{dev.address}",
        "product": text("product"),
        "manufacturer": text("manufacturer"),
        "vid_pid": f"{dev.idVendor:04x}:{dev.idProduct:04x}",
    }


def list_adapters(devices: Optional[list] = None) -> list[dict]:
    """Every gs_usb adapter: serial, port token, USB path, product. Never opens the CAN bus."""
    devices = _find_devices() if devices is None else devices
    out = []
    for dev in devices:
        try:
            out.append(_describe(dev))
        finally:
            _dispose(dev)
    return sorted(out, key=lambda a: (a["serial"], a["usb_path"]))


def parse_port(port: Optional[str]) -> Optional[str]:
    """The USB serial a port names ("gs_usb:<serial>" or a bare serial), or None for "the only one"."""
    if port is None or port in ("", "gs_usb", "gs_usb:"):
        return None
    serial = port[len("gs_usb:"):] if port.startswith("gs_usb:") else port
    if not serial or any(c.isspace() for c in serial):
        raise ValueError(f"expected gs_usb:<USB serial>, got {port!r}")
    return serial


def bit_timing(fclk: int, bitrate: int, tseg1_min: int, tseg1_max: int, tseg2_min: int,
               tseg2_max: int, sjw_max: int, brp_min: int, brp_max: int, brp_inc: int) -> tuple:
    """(prop_seg, phase_seg1, phase_seg2, sjw, brp) for `bitrate` at an 87.5 % sample point, using
    the most time quanta the adapter allows."""
    brp_inc = max(1, brp_inc)
    for brp in range(max(1, brp_min), brp_max + 1, brp_inc):
        quanta, rest = divmod(fclk, brp * bitrate)
        if rest:
            continue
        tseg2 = max(tseg2_min, round(quanta * (1 - SAMPLE_POINT)))
        tseg1 = quanta - 1 - tseg2
        if tseg2 <= tseg2_max and tseg1_min <= tseg1 <= tseg1_max and tseg1 >= 2:
            return 1, tseg1 - 1, tseg2, 1, brp
    raise ConnectError(f"adapter clock {fclk} Hz cannot produce {bitrate} bit/s")


class GsUsbBackend(CanBackend):
    def __init__(self, port: Optional[str] = None, bitrate: int = 1_000_000, io_timeout: float = 0.1,
                 device=None):
        self.serial = parse_port(port)
        self.bitrate = bitrate
        self.io_timeout = io_timeout
        self._device = device          # injected device (tests); otherwise found by serial at open()
        self._dev = None
        self._cb: Optional[Callable[[int, bytes], None]] = None
        self._tx_lock = threading.Lock()
        self._in_flight = threading.Condition()
        self._pending: set[int] = set()
        self._next_echo = 0
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._claimed = False
        self._started = False
        self.error: Optional[str] = None   # why the RX thread stopped, if it did
        self.error_frames = 0
        self.rx_overflows = 0

    @property
    def port(self) -> str:
        return f"gs_usb:{self.serial}" if self.serial else "gs_usb"

    def _ms(self, seconds: float) -> int:
        return max(1, math.ceil(seconds * 1000))

    def _select(self):
        if self._device is not None:
            return self._device
        devices = _find_devices()
        matches = []
        for dev in devices:
            try:
                serial = dev.serial_number
            except Exception:
                serial = None
            if self.serial is None or serial == self.serial:
                matches.append(dev)
        for dev in devices:
            if len(matches) != 1 or dev is not matches[0]:
                _dispose(dev)
        if len(matches) == 1:
            return matches[0]
        if self.serial is None:
            raise ConnectError(f"expected exactly one gs_usb adapter, found {len(matches)}; "
                               "name one as gs_usb:<serial> (see the ports listing)")
        raise ConnectError(f"no gs_usb adapter with serial {self.serial!r} (found {len(devices)} adapter(s)); "
                           f"check the cable and USB permissions. {INSTALL_HINT}")

    def open(self) -> None:
        usb = _usb()
        dev = self._select()
        self._dev = dev
        timeout = self._ms(self.io_timeout)
        try:
            if sys.platform.startswith("linux"):
                try:
                    if dev.is_kernel_driver_active(INTERFACE):
                        dev.detach_kernel_driver(INTERFACE)
                except NotImplementedError:
                    pass
            try:
                dev.get_active_configuration()
            except usb.core.USBError:
                dev.set_configuration()
            usb.util.claim_interface(dev, INTERFACE)
            self._claimed = True
            raw = bytes(dev.ctrl_transfer(0xC1, BREQ_BT_CONST, 0, INTERFACE, _BT_CONST.size, timeout))
            feature, fclk, *limits = _BT_CONST.unpack(raw[:_BT_CONST.size])
            timing = bit_timing(fclk, self.bitrate, *limits)
            dev.ctrl_transfer(0x41, BREQ_BITTIMING, 0, INTERFACE, struct.pack("<5I", *timing), timeout)
            self._started = True   # stop on close even if the start transfer fails partway
            # Channel start only; never a USB reset (it wedges these adapters).
            dev.ctrl_transfer(0x41, BREQ_MODE, 0, INTERFACE, struct.pack("<II", MODE_START, 0), timeout)
        except Exception as e:
            self._shutdown_usb()
            if isinstance(e, ConnectError):
                raise
            raise ConnectError(f"cannot open {self.port}: {e}. Close other CAN software; "
                               f"unplug and replug the adapter if it stopped answering. {INSTALL_HINT}") from e
        self.error = None
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name=f"gs_usb-rx-{self.serial}")
        self._rx_thread.start()

    def close(self) -> None:
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0 + self.io_timeout)
            self._rx_thread = None
        self._shutdown_usb()
        with self._in_flight:
            self._pending.clear()
            self._in_flight.notify_all()

    def _shutdown_usb(self) -> None:
        dev, self._dev = self._dev, None
        if dev is None:
            return
        try:
            if self._started:
                dev.ctrl_transfer(0x41, BREQ_MODE, 0, INTERFACE, struct.pack("<II", MODE_RESET, 0),
                                  self._ms(self.io_timeout))
        except Exception:
            log.warning("gs_usb channel stop failed on %s", self.port, exc_info=True)
        finally:
            self._started = False
            if self._claimed:
                try:
                    _usb().util.release_interface(dev, INTERFACE)
                except Exception:
                    log.debug("gs_usb interface release failed", exc_info=True)
                self._claimed = False
            _dispose(dev)

    def send(self, can_id: int, data: bytes, extended: bool = True) -> None:
        data = bytes(data)
        if len(data) > 8:
            raise ValueError("classic CAN carries at most 8 data bytes")
        dev = self._dev
        if dev is None:
            raise ConnectError(f"{self.port} is not open" + (f" ({self.error})" if self.error else ""))
        ident = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG if extended else can_id & 0x7FF
        deadline = time.monotonic() + self.io_timeout
        with self._tx_lock:
            with self._in_flight:
                while len(self._pending) >= MAX_IN_FLIGHT:
                    left = deadline - time.monotonic()
                    if left <= 0 or self._dev is None:
                        raise TimeoutError(
                            f"{self.port}: {len(self._pending)} frames were never acknowledged on the CAN bus; "
                            "stopped writing to protect the adapter. Check that the motors are powered and "
                            "the CAN wiring and bitrate are right, then reconnect")
                    self._in_flight.wait(left)
                echo = self._next_echo
                self._next_echo = (self._next_echo + 1) % 0x10000
                self._pending.add(echo)
            packet = _FRAME.pack(echo, ident, len(data), 0, 0, 0, data.ljust(8, b"\x00"))
            try:
                written = dev.write(EP_OUT, packet, self._ms(max(0.0, deadline - time.monotonic())))
            except Exception as e:
                self._forget(echo)
                raise TimeoutError(f"{self.port}: USB write failed ({e}); the adapter may need "
                                   "unplugging and replugging") from e
            if written != len(packet):
                self._forget(echo)
                raise TimeoutError(f"{self.port}: incomplete USB write ({written}/{len(packet)} bytes)")

    def _forget(self, echo: int) -> None:
        with self._in_flight:
            self._pending.discard(echo)
            self._in_flight.notify_all()

    def _rx_loop(self) -> None:
        usb = _usb()
        timeout = self._ms(self.io_timeout)
        while self._running:
            dev = self._dev
            if dev is None:
                break
            try:
                raw = bytes(dev.read(EP_IN, 64, timeout))
            except usb.core.USBTimeoutError:
                continue
            except Exception as e:
                if self._running:
                    self.error = f"USB read failed: {e}"
                    log.error("%s: %s; receive stopped (unplug and replug the adapter)", self.port, self.error)
                break
            try:
                self._handle(raw)
            except Exception:
                log.exception("gs_usb RX frame-processing exception (continuing to receive frames)")
        self._running = False
        with self._in_flight:
            self._in_flight.notify_all()

    def _handle(self, raw: bytes) -> None:
        if len(raw) < _FRAME.size:
            return
        echo, ident, dlc, channel, flags, _, payload = _FRAME.unpack(raw[:_FRAME.size])
        if echo != ECHO_RX:
            self._forget(echo)   # our frame was acknowledged on the bus
            return
        if flags & FLAG_OVERFLOW:
            self.rx_overflows += 1
        if channel != 0 or flags & ~FLAG_OVERFLOW or ident & CAN_RTR_FLAG:
            return
        if ident & CAN_ERR_FLAG:
            self.error_frames += 1
            return
        cb = self._cb
        if cb is not None:
            extended = bool(ident & CAN_EFF_FLAG)
            cb(CanId(ident & (CAN_EFF_MASK if extended else 0x7FF), extended), payload[:min(dlc, 8)])
