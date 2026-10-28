"""Read Star sensors and serve the designer ZIP unchanged on localhost.

There are deliberately no motor writes, SDK Arm objects, CAN connections, or
calibration-write endpoints. The serial adapter issues angle queries only.
"""
import argparse
import hashlib
import json
import math
import mimetypes
import signal
import threading
import time
import zipfile
import xml.etree.ElementTree as ET
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[3]
STATIC = Path(__file__).parent / "static"
IDS = list(range(7))
JOINT_NAMES = [f"link_{i:03d}_joint" for i in range(2, 8)]
LABELS = ["Base", "Shoulder", "Elbow", "Forearm", "Wrist", "Wrist roll"]
MAX_AGE = 0.5


def describe_model(path):
    """Audit this designer export without changing its XML or meshes."""
    with zipfile.ZipFile(path) as archive:
        names = set(archive.namelist())
        if "robot.urdf" not in names:
            raise ValueError("Expected robot.urdf at the root of the designer ZIP")
        if sum(i.file_size for i in archive.infolist()) > 256 * 1024 * 1024:
            raise ValueError("ZIP exceeds the viewer's 256 MB expanded-size limit")
        xml = archive.read("robot.urdf")
        robot = ET.fromstring(xml)
        links = {l.get("name"): l for l in robot.findall("link")}
        if len(links) != len(robot.findall("link")):
            raise ValueError("Duplicate link names")
        child_links = set()
        joints = []
        for joint in robot.findall("joint"):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in links or child not in links or child in child_links:
                raise ValueError("Invalid parent/child link tree")
            child_links.add(child)
            lim = joint.find("limit")
            origin = joint.find("origin")
            axis = joint.find("axis")
            mimic = joint.find("mimic")
            joints.append({"name": joint.get("name"), "type": joint.get("type"),
                           "parent": parent, "child": child,
                           "limits": {k: float(v) for k, v in lim.attrib.items()} if lim is not None else {},
                           "origin": dict(origin.attrib) if origin is not None else {},
                           "axis": axis.get("xyz") if axis is not None else "1 0 0",
                           "mimic": dict(mimic.attrib) if mimic is not None else None})
        roots = set(links) - child_links
        if len(roots) != 1:
            raise ValueError("URDF must have one root")
        reached = set(roots)
        for _ in links:
            reached.update(j["child"] for j in joints if j["parent"] in reached)
        if reached != set(links):
            raise ValueError("URDF contains a disconnected or cyclic link graph")
        # This bridge has an explicit mapping to the designer's six-joint export.
        # Reject other topologies rather than silently pairing the wrong motors.
        arm_joints = [j for j in joints if j["name"] in JOINT_NAMES]
        if [j["name"] for j in arm_joints] != JOINT_NAMES:
            raise ValueError("This viewer's Star mapping requires the designer's link_002_joint through link_007_joint export")
        if any(j["type"] != "revolute" for j in arm_joints):
            raise ValueError("Expected six revolute arm joints")
        expected_parent = next(iter(roots))
        for j in arm_joints:
            if j["parent"] != expected_parent:
                raise ValueError("Expected the designer's serial six-joint chain")
            expected_parent = j["child"]
        extra = {j["name"]: j for j in joints if j["name"] not in JOINT_NAMES}
        if len({j["name"] for j in joints}) != len(joints):
            raise ValueError("Duplicate joint names")
        if extra:
            if set(extra) != {"gripper_left_joint", "gripper_right_joint", "grasp_center_joint"}:
                raise ValueError("Unsupported extra joints in Star mapping")
            left, right = extra["gripper_left_joint"], extra["gripper_right_joint"]
            for j in (left, right):
                if j["type"] != "prismatic" or j["parent"] != "link_007" or j["limits"].get("lower") != 0 or not math.isclose(j["limits"].get("upper", 0), .0524125):
                    raise ValueError("Invalid symmetric gripper joint")
            if right["mimic"] != {"joint": "gripper_left_joint", "multiplier": "1", "offset": "0"} or left["mimic"]:
                raise ValueError("Invalid gripper mimic relationship")
            if extra["grasp_center_joint"]["type"] != "fixed":
                raise ValueError("Expected fixed grasp center")
        for mesh in robot.findall(".//mesh"):
            filename = mesh.get("filename", "")
            if PurePosixPath(filename).is_absolute() or ".." in PurePosixPath(filename).parts or ":" in filename or filename not in names:
                raise ValueError(f"Missing or non-local mesh: {filename}")
        report = json.loads(archive.read("export_report.json")) if "export_report.json" in names else {}
        model = json.loads(archive.read("user_model.json")) if "user_model.json" in names else {}
        mass = sum(float(x.get("value")) for x in robot.findall("./link/inertial/mass"))
        revision = json.loads(archive.read("revision_report.json")) if "revision_report.json" in names else {}
        metadata = {"filename": Path(path).name, "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                    "urdf_sha256": hashlib.sha256(xml).hexdigest(), "root": next(iter(roots)),
                    "links": len(links), "joints": arm_joints, "all_joints": joints, "mass_kg": mass,
                    "visual_meshes": len(robot.findall("./link/visual/geometry/mesh")),
                    "collision_meshes": len(robot.findall("./link/collision/geometry/mesh")),
                    "export_warnings": report.get("warnings", []),
                    "gripper_joint_present": bool(extra), "revision": revision,
                    "mass_sources": [l.get("mass_properties", {}).get("source", "unknown") for l in model.get("links", [])]}
        return metadata


class StarReader:
    def __init__(self, port, baud=1_000_000):
        self.port, self.baud = port, baud
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.raw = {}
        self.seen = {}
        self.sequence = 0
        self.connected = False
        self.error = "Waiting for Star servo replies. Check servo power and USB."
        self.hz = 0.0
        self.thread = None
        self.initial_reference = None

    def ingest(self, data, now):
        with self.lock:
            good = 0
            for i in IDS:
                sample = data.get(i)
                if sample is not None and sample.reliable and math.isfinite(sample.angle_deg):
                    self.raw[i] = float(sample.angle_deg)
                    self.seen[i] = now
                    good += 1
            if good:
                self.sequence += 1
                self.error = ""
            if self.initial_reference is None and all(i in self.seen and now - self.seen[i] <= MAX_AGE for i in IDS):
                self.initial_reference = {"raw_deg": {str(i): self.raw[i] for i in IDS},
                                          "timestamp": time.time(), "sequence": self.sequence}

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            readings = [{"servo": i, "angle_deg": self.raw.get(i),
                         "age_ms": round(1000 * (now - self.seen[i]), 1) if i in self.seen else None,
                         "fresh": i in self.seen and now - self.seen[i] <= MAX_AGE} for i in IDS]
            count = sum(r["fresh"] for r in readings)
            return {"readings": readings, "fresh_count": count,
                    "all_fresh": count == 7, "arm_fresh": all(r["fresh"] for r in readings[:6]),
                    "port": self.port, "serial_open": self.connected, "sequence": self.sequence,
                    "hz": round(self.hz, 1), "error": self.error,
                    "timestamp": time.time(), "sensor_only": True,
                    "initial_reference": self.initial_reference}

    def run(self):
        from .star_serial import StarAngleBus
        while not self.stop.is_set():
            bus = None
            try:
                bus = StarAngleBus(self.port, baudrate=self.baud)
                with self.lock:
                    self.connected = True
                    self.seen.clear()
                start, cycles, last_good = time.monotonic(), 0, time.monotonic()
                while not self.stop.is_set():
                    tick = time.monotonic()
                    data = {}
                    for servo_id in IDS:
                        if self.stop.is_set():
                            break
                        data[servo_id] = bus.read_angle(servo_id)
                        self.ingest({servo_id: data[servo_id]}, time.monotonic())
                    if any(s is not None and s.reliable for s in data.values()):
                        last_good = time.monotonic()
                    elif time.monotonic() - last_good > 2:
                        raise RuntimeError("No servo replies. Check the Star's servo power and cable.")
                    cycles += 1
                    if time.monotonic() - start >= 1:
                        with self.lock:
                            self.hz = cycles / (time.monotonic() - start)
                        start, cycles = time.monotonic(), 0
                    self.stop.wait(max(0, 0.04 - (time.monotonic() - tick)))
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                    self.hz = 0
            finally:
                if bus is not None:
                    try:
                        bus.close()
                    except Exception:
                        pass
                with self.lock:
                    self.connected = False
            self.stop.wait(1)

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True, name="star-sensors")
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=3)


class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def __init__(self, *args, reader, model, archive_path, mapping, **kwargs):
        self.reader, self.model = reader, model
        self.archive_path, self.mapping = archive_path, mapping
        super().__init__(*args, directory=str(STATIC), **kwargs)

    def send_bytes(self, body, kind):
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = unquote(urlsplit(self.path).path)
        if path == "/api/state":
            self.send_bytes(json.dumps(self.reader.snapshot(), allow_nan=False).encode(), "application/json")
        elif path == "/api/model":
            info = {**self.model, "star_mapping": self.mapping, "joint_labels": LABELS, "max_age_ms": MAX_AGE * 1000}
            self.send_bytes(json.dumps(info, allow_nan=False).encode(), "application/json")
        elif path == "/download/model.zip":
            self.send_bytes(self.archive_path.read_bytes(), "application/zip")
        elif path == "/joints":
            # An old temporary viewer may still be open in another browser tab.
            self.send_bytes(json.dumps({"ok": False, "err": "Viewer upgraded. Reload this page.",
                                        "star": {}, "motor": {}, "hz": 0}).encode(), "application/json")
        elif path.startswith("/model/"):
            name = path.removeprefix("/model/")
            if ".." in PurePosixPath(name).parts or name.startswith("/"):
                self.send_error(404)
                return
            try:
                with zipfile.ZipFile(self.archive_path) as archive:
                    data = archive.read(name)
                self.send_bytes(data, "application/xml" if name.endswith(".urdf") else mimetypes.guess_type(name)[0] or "application/octet-stream")
            except KeyError:
                self.send_error(404)
        else:
            if ".." in PurePosixPath(path).parts:
                self.send_error(404)
                return
            super().do_GET()

    def do_POST(self):
        self.send_error(405, "Sensor-only viewer: no write endpoints")

    def list_directory(self, path):
        self.send_error(404)

    def log_message(self, fmt, *args):
        if args and str(args[1] if len(args) > 1 else "") not in ("200", "304"):
            super().log_message(fmt, *args)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--zip", type=Path, default=Path.home() / "Downloads/Maker Arm_urdf_260902.zip")
    ap.add_argument("--star-port", default="/dev/cu.usbserial-1130")
    ap.add_argument("--http-port", type=int, default=8420)
    ap.add_argument("--no-serial", action="store_true", help="Inspect CAD without connecting hardware")
    ap.add_argument("--reference-file", type=Path, help="Use a previously captured, user-confirmed Star rest reference")
    a = ap.parse_args()
    model = describe_model(a.zip)
    mapping = json.loads((ROOT / "maker_arm/profiles/star_to_maker_v1.json").read_text())["joints"]
    if not (STATIC / "vendor/three/build/three.module.js").exists():
        raise SystemExit("Build viewer assets first: npm ci && npm run build (in tools/diagnostics/urdf_viewer)")
    reader = StarReader(a.star_port)
    if a.reference_file:
        reference = json.loads(a.reference_file.read_text())
        if set(reference.get("raw_deg", {})) != {str(i) for i in IDS} or not all(math.isfinite(v) for v in reference["raw_deg"].values()):
            raise SystemExit("Invalid saved Star reference")
        reader.initial_reference = reference
    # Prevent competing diagnostics from interleaving requests on the same bus.
    import fcntl
    import subprocess
    import tempfile
    lockfile = open(Path(tempfile.gettempdir()) / ("maker-star-" + hashlib.sha256(a.star_port.encode()).hexdigest()[:12] + ".lock"), "w")
    if not a.no_serial:
        try:
            fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Another Star viewer is already reading this port")
        if Path(a.star_port).exists():
            try:
                owners = subprocess.run(["lsof", "-t", a.star_port], capture_output=True, text=True, timeout=3)
                if owners.stdout.strip():
                    raise SystemExit("Star serial port is already in use by PID(s): " + owners.stdout.strip().replace("\n", ", "))
            except FileNotFoundError:
                pass
    handler = partial(Handler, reader=reader, model=model, archive_path=a.zip, mapping=mapping)
    server = ThreadingHTTPServer(("127.0.0.1", a.http_port), handler)
    server.daemon_threads = True
    if not a.no_serial:
        reader.start()
    def stop_server(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_server)
    print(f"Viewer: http://127.0.0.1:{a.http_port}/", flush=True)
    print(f"Designer ZIP: {a.zip}\nStar sensors only: {a.star_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        server.server_close()
        lockfile.close()


if __name__ == "__main__":
    main()
