"""Keeping an enabled arm's own threads responsive on small computers.

The control loop and the CAN receive thread are Python threads, so they share the interpreter lock (GIL) with
everything else in the process. Two things starved them on a big.LITTLE board (4 Cortex-A53 + 4 Cortex-A72):

- **Slow cores.** Left to the scheduler, the threads can land on the small cores, where one tick costs about
  three times as much. ``pin_to_fast_cpus()`` moves the calling thread to the fastest cores it is allowed on, only
  when the cores it may use differ in capacity; on a computer whose cores are all alike it does nothing.
- **The GIL switch interval.** A thread that gives up the GIL for I/O (each USB write, each received frame) waits
  up to ``sys.getswitchinterval()`` (5 ms by default) to get it back while another thread computes in Python.
  With IK solving in the same process the loop fell from 150 Hz to 20 Hz. ``SwitchInterval`` lowers the interval
  to ``FAST_SWITCH_S`` while at least one control loop runs and puts the previous value back afterwards.

Set the environment variable ``ARM_SDK_THREAD_TUNING=off`` to leave thread placement and the switch interval alone.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
from typing import Optional

log = logging.getLogger(__package__)

FAST_SWITCH_S = 0.0005    # GIL switch interval while a control loop runs (CPython default: 0.005)
TUNING_ENV = "ARM_SDK_THREAD_TUNING"
_SYSFS_CPU = "/sys/devices/system/cpu"


def tuning_enabled() -> bool:
    return os.environ.get(TUNING_ENV, "").strip().lower() not in ("0", "off", "no", "false")


def _read_number(path: str) -> Optional[int]:
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def cpu_capacity(cpu: int, sysfs: str = _SYSFS_CPU) -> Optional[int]:
    """The kernel's relative capacity of ``cpu`` (``cpu_capacity``), else its maximum clock; None if unknown."""
    for name in ("cpu_capacity", "cpufreq/cpuinfo_max_freq"):
        value = _read_number(f"{sysfs}/cpu{cpu}/{name}")
        if value is not None:
            return value
    return None


def fast_cpus(allowed, sysfs: str = _SYSFS_CPU) -> Optional[set[int]]:
    """The fastest of the ``allowed`` CPUs when they differ in capacity; None when they are all alike or unknown."""
    allowed = set(allowed)
    capacity = {cpu: cpu_capacity(cpu, sysfs) for cpu in allowed}
    if len(allowed) < 2 or any(v is None for v in capacity.values()):
        return None
    best = max(capacity.values())
    fast = {cpu for cpu, v in capacity.items() if v == best}
    return fast if fast != allowed else None


def pin_to_fast_cpus() -> Optional[set[int]]:
    """Restrict the calling thread to the fastest CPUs it may run on (Linux, big.LITTLE only). Returns the CPUs it
    was pinned to, or None when nothing changed. Never raises."""
    if not tuning_enabled() or not hasattr(os, "sched_setaffinity"):
        return None
    try:
        allowed = os.sched_getaffinity(0)   # pid 0 is the calling thread on Linux
        fast = fast_cpus(allowed)
        if fast:
            os.sched_setaffinity(0, fast)
            log.debug("%s pinned to CPUs %s", threading.current_thread().name, sorted(fast))
        return fast
    except OSError as e:
        log.debug("could not pin %s to the fast CPUs: %s", threading.current_thread().name, e)
        return None


class SwitchInterval:
    """Reference-counted lowering of the GIL switch interval: the first ``acquire()`` lowers it (never raises it),
    the last ``release()`` restores the previous value unless someone else changed it meanwhile."""

    def __init__(self, interval: float = FAST_SWITCH_S):
        self.interval = interval
        self._lock = threading.Lock()
        self._holders = 0
        self._saved: Optional[float] = None
        self._set: Optional[float] = None

    def acquire(self) -> None:
        with self._lock:
            self._holders += 1
            if self._holders == 1 and tuning_enabled():
                current = sys.getswitchinterval()
                if current > self.interval:
                    self._saved = current
                    sys.setswitchinterval(self.interval)
                    self._set = sys.getswitchinterval()

    def release(self) -> None:
        with self._lock:
            if self._holders == 0:
                return
            self._holders -= 1
            if self._holders == 0 and self._saved is not None:
                if sys.getswitchinterval() == self._set:
                    sys.setswitchinterval(self._saved)
                self._saved = self._set = None


switch_interval = SwitchInterval()
