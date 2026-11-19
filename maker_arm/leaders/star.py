"""Star Arm 102 leader: FashionStar UART smart servos read through ``motorbridge-smart-servo``.

Connect pings every servo, unlocks it (back-drivable) and resets its multi-turn counter. Each read
is one ``sync_monitor`` of all servos; a servo that misses a reply keeps its last target
(``JointMapper``), and one that misses ``miss_limit`` reads in a row ends the session with
``LeaderError``.
"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Callable, Optional, Union

from ..mapping import JointMapper
from .base import LeaderError

log = logging.getLogger(__name__)

SETTLE_S = 0.01      # between per-servo unlocks
BAUDRATE = 1_000_000


def _fashionstar(port: str, baudrate: int):
    try:
        from motorbridge_smart_servo import FashionStarServo
    except ImportError as e:
        raise LeaderError("the Star leader needs motorbridge-smart-servo: pip install motorbridge-smart-servo") from e
    return FashionStarServo(port, baudrate=baudrate)


class StarLeader:
    name = "star"

    def __init__(self, port: str, mapping: Union[JointMapper, str, Path, None] = None,
                 baudrate: int = BAUDRATE, miss_limit: int = 50,
                 bus_factory: Optional[Callable] = None, sleep: Callable[[float], None] = time.sleep):
        if mapping is None:
            from .._product import DEFAULT_STAR_MAPPING
            mapping = DEFAULT_STAR_MAPPING
        self.mapper = mapping if isinstance(mapping, JointMapper) else JointMapper.from_json(mapping)
        self.port = port
        self.baudrate = baudrate
        if miss_limit < 1:
            raise ValueError("miss_limit must be >= 1")
        self.miss_limit = miss_limit
        self._factory = bus_factory or _fashionstar
        self._sleep = sleep
        self.bus = None
        self.misses = {sid: 0 for sid in self.mapper.servo_ids}
        self.last_error: Optional[str] = None

    @property
    def servo_ids(self) -> list[int]:
        return self.mapper.servo_ids

    # ── lifecycle ──
    def connect(self, cancel=None) -> None:
        """Open the port and unlock the servos (``cancel`` is accepted for the Leader interface;
        nothing here energizes anything)."""
        if self.bus is not None:
            return
        bus = self._factory(self.port, self.baudrate)
        try:
            silent = [sid for sid in self.servo_ids if not bus.ping(sid)]
            if silent:
                raise LeaderError(f"Star leader servos {silent} did not answer on {self.port} -- check the port and ids")
            for sid in self.servo_ids:
                bus.unlock(sid)
                self._sleep(SETTLE_S)
            for sid in self.servo_ids:
                bus.reset_multi_turn(sid)
        except BaseException:
            bus.close()
            raise
        self.bus = bus
        self.mapper.reset()
        self.misses = {sid: 0 for sid in self.servo_ids}

    def close(self) -> None:
        bus, self.bus = self.bus, None
        if bus is not None:
            try:
                bus.close()
            except Exception as e:  # a USB dropout makes close() raise; the port is gone either way
                log.warning("closing the Star leader port failed: %s", e)

    # ── reading ──
    def read_raw(self) -> dict[int, float]:
        """One sync read: servo id -> multi-turn degrees, for the servos that replied."""
        if self.bus is None:
            raise LeaderError("Star leader is not connected")
        data = self.bus.sync_monitor(self.servo_ids)
        return {sid: float(data[sid].angle_deg) for sid in self.servo_ids
                if data.get(sid) is not None and math.isfinite(data[sid].angle_deg)}

    def read(self) -> Optional[list[float]]:
        try:
            raw = self.read_raw()
        except LeaderError:
            raise
        except Exception as e:
            self.last_error = str(e)
            raw = {}
        for sid in self.servo_ids:
            self.misses[sid] = 0 if sid in raw else self.misses[sid] + 1
        lost = [sid for sid, count in self.misses.items() if count >= self.miss_limit]
        if lost:
            raise LeaderError(f"Star leader servos {lost} missed {self.miss_limit} reads in a row"
                              + (f" (last error: {self.last_error})" if self.last_error else ""))
        if not raw:
            return None
        targets = self.mapper.map(raw)
        return targets if all(math.isfinite(t) for t in targets) else None

    # ── calibration ──
    def zero(self) -> None:
        """Make the present pose every servo's origin (the leader must be at the zero pose)."""
        if self.bus is None:
            raise LeaderError("Star leader is not connected")
        for sid in self.servo_ids:
            self.bus.unlock(sid)
            self._sleep(SETTLE_S)
            self.bus.set_origin_point(sid)
        for sid in self.servo_ids:
            self.bus.reset_multi_turn(sid)
        self.mapper.reset()
