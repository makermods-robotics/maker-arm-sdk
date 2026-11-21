"""USB cameras: list them without opening them, and capture on a background thread.

Ids are stable across replugs into the same port where the OS allows it: ``usb:<by-path name>``
on Linux (``/dev/v4l/by-path``, ``-video-index0`` dropped), ``index:N`` elsewhere (OpenCV's
capture index, which can change on replug). Capture asks for MJPG 640x480 @ 30 fps before the
first read: that format sizes the USB bandwidth reservation, and left at its default a camera
may negotiate 1080p and starve the next camera on the same hub.

OpenCV is imported only when a camera opens (``pip install <package>[cameras]``).
"""

from __future__ import annotations

import json
import logging
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Union

log = logging.getLogger(__name__)

V4L_SYSFS = Path("/sys/class/video4linux")
V4L_BY_PATH = Path("/dev/v4l/by-path")
READ_FAILURE_S = 1.0   # a failed read is not a dead camera; this long without a frame is
REOPEN_S = 1.0         # wait before reopening a dead camera
_OPEN_LOCK = threading.Lock()  # AVFoundation can fail cameras opened at the same moment


def _cv2():
    try:
        import cv2
    except ImportError as e:
        from ._product import DIST_NAME
        raise ImportError(f"cameras need OpenCV: pip install '{DIST_NAME}[cameras]'") from e
    return cv2


@dataclass(frozen=True)
class CameraInfo:
    id: str                 # "usb:<by-path>" or "index:N"
    index: int              # OpenCV capture index
    name: str
    path: Optional[str]     # /dev/videoN on Linux


# ── discovery ──
def _read(path: Path) -> Optional[str]:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def _linux_stable_ids(by_path: Path) -> dict[str, str]:
    """{"videoN": "usb:<by-path name>"} from the /dev/v4l/by-path links."""
    ids = {}
    try:
        links = sorted(by_path.iterdir())
    except OSError:
        return ids
    for link in links:
        try:
            node = os.path.basename(os.readlink(link))
        except OSError:
            continue
        name = re.sub(r"-video-index\d+$", "", link.name)
        if link.name.endswith("-video-index0"):
            ids[node] = ("usb:" if "-usb-" in name else "path:") + name
    return ids


def _linux_cameras() -> list[CameraInfo]:
    """Capture nodes from sysfs: a UVC camera's capture node has index 0 (its metadata node 1)."""
    stable = _linux_stable_ids(V4L_BY_PATH)
    try:
        entries = [e for e in V4L_SYSFS.iterdir() if re.fullmatch(r"video\d+", e.name)]
    except OSError:
        return []
    cameras = []
    for entry in sorted(entries, key=lambda e: int(e.name[5:])):
        if _read(entry / "index") not in (None, "0"):
            continue
        index = int(entry.name[5:])
        name = _read(entry / "name") or entry.name
        product, _, rest = name.partition(": ")
        if rest == product:
            name = product
        cameras.append(CameraInfo(stable.get(entry.name, f"index:{index}"), index, name, f"/dev/{entry.name}"))
    return cameras


def _macos_cameras() -> list[CameraInfo]:
    """system_profiler's cameras, ordered as OpenCV's AVFoundation backend numbers them (by unique id)."""
    try:
        out = subprocess.run(["system_profiler", "SPCameraDataType", "-json"], capture_output=True,
                             text=True, timeout=10, check=True).stdout
        found = json.loads(out).get("SPCameraDataType", [])
    except (OSError, subprocess.SubprocessError, ValueError):
        return []
    found.sort(key=lambda c: c.get("spcamera_unique-id", ""))
    return [CameraInfo(f"index:{i}", i, c.get("_name", f"camera {i}"), None) for i, c in enumerate(found)]


def list_cameras() -> list[CameraInfo]:
    """Cameras present now. Never opens a device."""
    if sys.platform.startswith("linux"):
        return _linux_cameras()
    if sys.platform == "darwin":
        return _macos_cameras()
    return []


def resolve(camera_id: Union[str, int]) -> Union[int, str]:
    """An id as OpenCV opens it: a capture index, or a /dev/videoN path for a ``usb:``/``path:`` id."""
    if isinstance(camera_id, int):
        return camera_id
    if camera_id.startswith("index:") and camera_id[6:].isdigit():
        return int(camera_id[6:])
    if camera_id.startswith(("usb:", "path:")):
        link = V4L_BY_PATH / f"{camera_id.split(':', 1)[1]}-video-index0"
        if not link.exists():
            raise LookupError(f"camera {camera_id} is not connected")
        return os.path.realpath(link)
    raise ValueError(f"unknown camera id {camera_id!r}: expected usb:<by-path>, path:<by-path> or index:N")


# ── capture ──
class Frame(tuple):
    """A captured frame. Unpacks as ``(image, stamp)``, ``stamp`` the wall-clock capture time (s);
    ``captured`` is the time.monotonic() capture time and ``age_s`` the seconds since, which a
    wall-clock step cannot change."""

    def __new__(cls, image, stamp: float, captured: float) -> "Frame":
        frame = super().__new__(cls, (image, stamp))
        frame.captured = captured
        return frame

    @property
    def image(self):
        return self[0]

    @property
    def stamp(self) -> float:
        return self[1]

    @property
    def age_s(self) -> float:
        return max(0.0, time.monotonic() - self.captured)


class Camera:
    """One camera captured on a background thread; the newest frame is kept, a dead camera reopened."""

    def __init__(self, id: Union[str, int], width: int = 640, height: int = 480, fps: int = 30,
                 fourcc: str = "MJPG"):
        self.id = id
        self.width, self.height, self.fps, self.fourcc = width, height, fps, fourcc
        self.error: Optional[str] = None
        self.reopens = 0
        self._frame = None
        self._stamp = 0.0
        self._captured = 0.0
        self._count = 0
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._cap = None

    def __enter__(self) -> "Camera":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _open_capture(self):
        cv2 = _cv2()
        backend = {"linux": cv2.CAP_V4L2, "darwin": cv2.CAP_AVFOUNDATION}.get(
            "linux" if sys.platform.startswith("linux") else sys.platform, cv2.CAP_ANY)
        with _OPEN_LOCK:
            cap = cv2.VideoCapture(resolve(self.id), backend)
            if not cap.isOpened():
                cap.release()
                raise OSError(f"cannot open camera {self.id}")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap

    def open(self, timeout: float = 5.0) -> None:
        """Open the device and start capturing; returns once the first frame arrived."""
        if self.is_open:
            return
        self._cap = self._open_capture()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"camera-{self.id}")
        self._thread.start()
        try:
            self.read(timeout)
        except TimeoutError:
            self.close()
            raise TimeoutError(f"camera {self.id} opened but sent no frame within {timeout} s") from None

    def _run(self) -> None:
        last_frame = time.monotonic()
        while not self._stop.is_set():
            if self._cap is None:
                try:
                    self._cap = self._open_capture()
                    self.reopens += 1
                    last_frame = time.monotonic()
                    log.info("camera %s reopened", self.id)
                except Exception as e:
                    self.error = str(e)
                    self._stop.wait(REOPEN_S)
                    continue
            try:
                ok, frame = self._cap.read()
            except Exception as e:
                ok, frame = False, None
                self.error = str(e)
            now = time.monotonic()
            if ok and frame is not None:
                last_frame = now
                self.error = None
                with self._cond:
                    self._frame, self._stamp, self._captured = frame, time.time(), now
                    self._count += 1
                    self._cond.notify_all()
            elif now - last_frame > READ_FAILURE_S:
                self.error = self.error or f"camera {self.id} stopped returning frames"
                log.warning("%s; reopening", self.error)
                self._release()
                self._stop.wait(REOPEN_S)
            else:
                self._stop.wait(0.01)
        self._release()

    def _release(self) -> None:
        cap, self._cap = self._cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

    def read(self, timeout: float = 1.0) -> Frame:
        """The next new frame as a Frame, ``(image, wall-clock stamp)``; TimeoutError if none comes."""
        with self._cond:
            seen = self._count
            if not self._cond.wait_for(lambda: self._count > seen or not self.is_open, timeout):
                raise TimeoutError(f"no frame from camera {self.id} within {timeout} s")
            if self._count == seen:
                raise RuntimeError(f"camera {self.id} is closed")
            return Frame(self._frame, self._stamp, self._captured)

    def newest(self, max_age_s: Optional[float] = None) -> Optional[Frame]:
        """The newest frame as read() gives it (no wait), or None when there is none or it is older
        than ``max_age_s`` (judged on the monotonic clock)."""
        with self._cond:
            if self._frame is None or (max_age_s is not None and time.monotonic() - self._captured > max_age_s):
                return None
            return Frame(self._frame, self._stamp, self._captured)

    def latest(self, max_age_s: Optional[float] = None):
        """The newest frame (no wait), or None when there is none or it is older than ``max_age_s``."""
        newest = self.newest(max_age_s)
        return None if newest is None else newest[0]

    def snapshot(self, path, max_age_s: float = 0.5) -> Path:
        """Save the current picture (the newest frame, or the next one if that is stale) to ``path``."""
        frame = self.latest(max_age_s)
        if frame is None:
            frame, _ = self.read()
        path = Path(path)
        if not _cv2().imwrite(str(path), frame):
            raise OSError(f"could not write {path}")
        return path

    def close(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._release()
        with self._cond:
            self._cond.notify_all()
