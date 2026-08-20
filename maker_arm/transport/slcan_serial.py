"""Cross-platform Lawicel SLCAN serial backend.

This is the userspace equivalent of Linux ``slcand``. It is the supported macOS
transport and talks directly to a CANable-style adapter using ASCII SLCAN frames.
"""

import logging
import threading
import time
from typing import Callable, Optional

from .base import CanBackend

log = logging.getLogger("maker_arm.transport")


def slcan_encode(can_id: int, data: bytes, extended: bool = True) -> bytes:
    """Encode one classic-CAN data frame in Lawicel SLCAN format."""
    if len(data) > 8:
        raise ValueError(f"classic CAN payload is limited to 8 bytes, got {len(data)}")
    if extended:
        if not 0 <= can_id <= 0x1FFFFFFF:
            raise ValueError(f"extended CAN ID out of range: {can_id:#x}")
        head = f"T{can_id:08X}{len(data):X}"
    else:
        if not 0 <= can_id <= 0x7FF:
            raise ValueError(f"standard CAN ID out of range: {can_id:#x}")
        head = f"t{can_id:03X}{len(data):X}"
    return (head + data.hex().upper() + "\r").encode("ascii")


class SlcanFrameParser:
    """Incremental SLCAN stream parser; command acknowledgements are ignored."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(chunk)
        out: list[tuple[int, bytes]] = []
        while True:
            try:
                end = self._buf.index(0x0D)  # SLCAN records end with CR
            except ValueError:
                # A valid classic-CAN record is at most 30 bytes including a
                # timestamp.  Bound garbage if the stream loses framing.
                if len(self._buf) > 256:
                    del self._buf[:-32]
                break
            raw = bytes(self._buf[:end])
            del self._buf[:end + 1]
            if not raw or raw[:1] not in (b"t", b"T"):
                continue
            try:
                id_len = 8 if raw[:1] == b"T" else 3
                if len(raw) < 2 + id_len:
                    continue
                can_id = int(raw[1:1 + id_len], 16)
                dlc = int(raw[1 + id_len:2 + id_len], 16)
                if dlc > 8:
                    continue
                payload_end = 2 + id_len + dlc * 2
                if len(raw) < payload_end:
                    continue
                data = bytes.fromhex(raw[2 + id_len:payload_end].decode("ascii"))
            except (UnicodeDecodeError, ValueError):
                continue
            out.append((can_id, data))
        return out


class SlcanSerialBackend(CanBackend):
    """CANable/Lawicel adapter accessed directly through a serial port."""

    BITRATE_COMMANDS = {
        10_000: "S0", 20_000: "S1", 50_000: "S2", 100_000: "S3",
        125_000: "S4", 250_000: "S5", 500_000: "S6", 800_000: "S7",
        1_000_000: "S8",
    }

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200,
                 bitrate: int = 1_000_000, rtscts: bool = False,
                 startup_delay: float = 2.5,
                 auto_retransmit: bool = False,
                 tx_interval: float = 0.0015,
                 write_timeout: float = 1.0):
        if bitrate not in self.BITRATE_COMMANDS:
            raise ValueError(f"unsupported SLCAN bitrate {bitrate}; choices: {sorted(self.BITRATE_COMMANDS)}")
        self._port = port
        self._baud = baudrate
        self._bitrate = bitrate
        self._rtscts = rtscts
        self._startup_delay = startup_delay
        self._auto_retransmit = auto_retransmit
        if tx_interval < 0:
            raise ValueError("SLCAN tx_interval must be non-negative")
        if write_timeout <= 0:
            raise ValueError("SLCAN write_timeout must be positive")
        self._tx_interval = tx_interval
        self._write_timeout = write_timeout
        self._last_tx = 0.0
        # normaldotcom/canable-fw has no USB-TX buffering and its hardware CAN
        # RX FIFO holds only three frames.  Arm._tick uses these hints to wait
        # for each motor's reply before issuing the next MIT command, preventing
        # a seven-reply burst from consistently hiding motors 4..7.
        self.feedback_paced_control = True
        self.feedback_wait_timeout = 0.015
        self._ser = None
        self._cb: Optional[Callable[[int, bytes], None]] = None
        self._tx_lock = threading.Lock()
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None

    def open(self) -> None:
        import serial

        try:
            self._ser = serial.Serial(
                self._port,
                self._baud,
                timeout=0.02,
                write_timeout=self._write_timeout,
                rtscts=self._rtscts,
            )
            # USB serial bridges may reset when the port opens. Commands sent
            # during that reset are silently ignored on real adapters.
            if self._startup_delay:
                time.sleep(self._startup_delay)
            self._ser.reset_input_buffer()
            speed = self.BITRATE_COMMANDS[self._bitrate]
            # The tested normaldotcom CANable firmware's A0 extension prevents an unacknowledged frame from
            # being retried indefinitely. Besides avoiding stale commands,
            # this keeps its small TX queues responsive if motor power is off.
            retry = "A1" if self._auto_retransmit else "A0"
            with self._tx_lock:
                self._ser.write(f"C\r{speed}\r{retry}\rO\r".encode("ascii"))
                self._ser.flush()
                self._last_tx = time.monotonic()
        except Exception:
            if self._ser is not None:
                self._ser.close()
                self._ser = None
            raise
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True,
                                           name=f"slcan-rx-{self._port}")
        self._rx_thread.start()

    def close(self) -> None:
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._ser:
            try:
                with self._tx_lock:
                    self._ser.write(b"C\r")
                    self._ser.flush()
            except Exception:
                log.debug("could not close SLCAN channel cleanly", exc_info=True)
            self._ser.close()
            self._ser = None

    def send(self, can_id: int, data: bytes, extended: bool = True) -> None:
        with self._tx_lock:
            if self._ser is None:
                raise RuntimeError("SLCAN backend is not open")
            # Old normaldotcom CANable SLCAN firmware has a 28-frame CAN TX
            # queue and processes only one queued CAN frame per firmware main-loop
            # iteration.  Unpaced USB writes can fill that queue even though the
            # CAN bus itself has ample bandwidth, after which feedback disappears
            # and the adapter requires a physical USB reset.  Pace every CAN frame,
            # including enable/disable and parameter traffic, at the transport layer
            # so callers cannot accidentally burst-fill the firmware queue.
            remain = self._tx_interval - (time.monotonic() - self._last_tx)
            if remain > 0:
                time.sleep(remain)
            self._ser.write(slcan_encode(can_id, data, extended))
            self._last_tx = time.monotonic()

    def set_recv_callback(self, cb) -> None:
        self._cb = cb

    def _rx_loop(self) -> None:
        parser = SlcanFrameParser()
        while self._running:
            try:
                chunk = self._ser.read(max(1, self._ser.in_waiting))
            except Exception:
                if self._running:
                    log.exception("SLCAN serial read failed")
                break
            if not chunk:
                continue
            try:
                cb = self._cb
                for cid, data in parser.feed(chunk):
                    if cb:
                        cb(cid, data)
            except Exception:
                log.exception("SLCAN RX frame-processing exception (continuing to receive frames)")
