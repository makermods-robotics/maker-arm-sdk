import os
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

from chain_fixture import load

cameras = load("cameras")


# ── discovery (a fake sysfs and by-path tree; no device is opened) ──
@pytest.fixture
def v4l(tmp_path, monkeypatch):
    sysfs, by_path, dev = tmp_path / "sys", tmp_path / "by-path", tmp_path / "dev"
    for d in (sysfs, by_path, dev):
        d.mkdir()

    def add(n, name, index, link=None):
        entry = sysfs / f"video{n}"
        entry.mkdir()
        (entry / "name").write_text(name + "\n")
        (entry / "index").write_text(f"{index}\n")
        (dev / f"video{n}").touch()
        if link:
            os.symlink(f"../dev/video{n}", by_path / f"{link}-video-index{index}")

    add(0, "Wrist Cam: Wrist Cam", 0, "platform-3610000.usb-usb-0:2.2.2:1.0")
    add(1, "Wrist Cam: Wrist Cam", 1, "platform-3610000.usb-usb-0:2.2.2:1.0")  # metadata node
    add(2, "Top Cam: UVC", 0, "platform-3610000.usb-usb-0:2.2.1:1.0")
    add(3, "vivid", 0)
    monkeypatch.setattr(cameras, "V4L_BY_PATH", by_path)
    monkeypatch.setattr(cameras, "V4L_SYSFS", sysfs)
    monkeypatch.setattr(sys, "platform", "linux")
    return SimpleNamespace(sysfs=sysfs, by_path=by_path, dev=dev)


@pytest.mark.skipif(sys.platform == "win32", reason="a Linux /dev tree of symlinks")
def test_linux_lists_capture_nodes_with_stable_ids(v4l):
    found = cameras.list_cameras()
    assert [(c.id, c.index, c.name) for c in found] == [
        ("usb:platform-3610000.usb-usb-0:2.2.2:1.0", 0, "Wrist Cam"),
        ("usb:platform-3610000.usb-usb-0:2.2.1:1.0", 2, "Top Cam: UVC"),
        ("index:3", 3, "vivid"),
    ]
    assert found[0].path == "/dev/video0"


@pytest.mark.skipif(sys.platform == "win32", reason="a Linux /dev tree of symlinks")
def test_resolve_turns_ids_into_what_opencv_opens(v4l):
    assert cameras.resolve("index:4") == 4 and cameras.resolve(2) == 2
    assert cameras.resolve("usb:platform-3610000.usb-usb-0:2.2.1:1.0") == os.path.realpath(v4l.dev / "video2")
    with pytest.raises(LookupError):
        cameras.resolve("usb:platform-3610000.usb-usb-0:9.9:1.0")
    with pytest.raises(ValueError):
        cameras.resolve("webcam")


def test_no_cameras_without_sysfs(tmp_path, monkeypatch):
    monkeypatch.setattr(cameras, "V4L_SYSFS", tmp_path / "missing")
    monkeypatch.setattr(sys, "platform", "linux")
    assert cameras.list_cameras() == []


def test_macos_numbers_cameras_as_avfoundation_does(monkeypatch):
    out = ('{"SPCameraDataType": [{"_name": "USB Cam", "spcamera_unique-id": "0x2"},'
           ' {"_name": "FaceTime HD", "spcamera_unique-id": "0x1"}]}')
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(cameras.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout=out))
    assert [(c.id, c.name) for c in cameras.list_cameras()] == [("index:0", "FaceTime HD"), ("index:1", "USB Cam")]


# ── capture (a fake cv2) ──
class FakeCapture:
    def __init__(self, source, backend, script):
        self.source, self.backend, self.script = source, backend, script
        self.props, self.released, self.reads = {}, False, 0

    def isOpened(self):
        return self.script.get("open", True)

    def set(self, prop, value):
        self.props[prop] = value

    def read(self):
        self.reads += 1
        time.sleep(0.002)
        if self.script.get("fail_after") is not None and self.reads > self.script["fail_after"]:
            return False, None
        return True, np.full((480, 640, 3), self.reads % 256, np.uint8)

    def release(self):
        self.released = True


@pytest.fixture
def fake_cv2(monkeypatch):
    opened, written, script = [], [], {}

    def capture(source, backend):
        cap = FakeCapture(source, backend, dict(script))
        opened.append(cap)
        return cap

    module = SimpleNamespace(
        CAP_V4L2=200, CAP_AVFOUNDATION=1200, CAP_ANY=0, CAP_PROP_FOURCC=6, CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5, VideoCapture=capture,
        VideoWriter_fourcc=lambda *c: "".join(c),
        imwrite=lambda path, frame: written.append((path, frame.shape)) or True)
    monkeypatch.setattr(cameras, "_cv2", lambda: module)
    monkeypatch.setattr(sys, "platform", "linux")
    return SimpleNamespace(opened=opened, written=written, script=script)


def test_open_asks_for_the_format_before_reading(fake_cv2):
    with cameras.Camera("index:1") as cam:
        frame, stamp = cam.read()
        cap = fake_cv2.opened[0]
        assert cap.source == 1 and cap.backend == 200
        assert cap.props == {6: "MJPG", 3: 640, 4: 480, 5: 30}
        assert frame.shape == (480, 640, 3) and abs(stamp - time.time()) < 1.0
    assert cap.released and not cam.is_open


def test_read_waits_for_a_new_frame(fake_cv2):
    with cameras.Camera(0) as cam:
        first, t1 = cam.read()
        second, t2 = cam.read()
        assert t2 >= t1 and not np.array_equal(first, second)


def test_latest_respects_max_age(fake_cv2):
    with cameras.Camera(0) as cam:
        assert cam.latest(max_age_s=1.0) is not None
        cam._stamp -= 10
        assert cam.latest(max_age_s=1.0) is None and cam.latest() is not None


def test_a_camera_that_stops_sending_is_reopened(fake_cv2, monkeypatch):
    monkeypatch.setattr(cameras, "READ_FAILURE_S", 0.05)
    monkeypatch.setattr(cameras, "REOPEN_S", 0.01)
    fake_cv2.script["fail_after"] = 3
    with cameras.Camera(0) as cam:
        deadline = time.monotonic() + 3
        while cam.reopens < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert cam.reopens >= 1 and fake_cv2.opened[0].released
        assert len(fake_cv2.opened) >= 2


def test_a_camera_that_will_not_open_raises(fake_cv2):
    fake_cv2.script["open"] = False
    with pytest.raises(OSError, match="cannot open"):
        cameras.Camera(0).open()


def test_a_camera_with_no_frames_fails_open_and_releases(fake_cv2):
    fake_cv2.script["fail_after"] = 0
    cam = cameras.Camera(0)
    with pytest.raises(TimeoutError):
        cam.open(timeout=0.1)
    assert not cam.is_open and fake_cv2.opened[0].released


def test_snapshot_writes_the_newest_frame(fake_cv2, tmp_path):
    with cameras.Camera(0) as cam:
        path = cam.snapshot(tmp_path / "shot.jpg")
    assert fake_cv2.written == [(str(path), (480, 640, 3))]


def test_read_on_a_closed_camera_fails_fast(fake_cv2):
    cam = cameras.Camera(0)
    cam.open()
    cam.close()
    with pytest.raises((RuntimeError, TimeoutError)):
        cam.read(timeout=0.2)


def test_missing_opencv_names_the_extra(monkeypatch):
    monkeypatch.setitem(sys.modules, "cv2", None)
    with pytest.raises(ImportError, match=r"\[cameras\]"):
        cameras._cv2()
