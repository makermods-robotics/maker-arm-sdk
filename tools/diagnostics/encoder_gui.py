#!/usr/bin/env python3
"""Read-only MakerArm encoder GUI for inspecting motion and calibration ranges.

The arm is connected but never enabled: torque stays released. Every CAN motor shows its
raw encoder position and SDK joint-coordinate position in radians, plus capture min/max,
travel, velocity, temperature, feedback age, and fault state. Select a row to inspect its
high-resolution history. Reset starts a fresh range capture; Export writes diagnostics only
and never changes the arm YAML or motor parameters.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import ttk

from maker_arm.arm import Arm
from maker_arm.errors import fault_text
from maker_arm.profiles import DEFAULT_ARM_CONFIG


@dataclass
class RangeStats:
    start: float = math.nan
    previous: float = math.nan
    minimum: float = math.inf
    maximum: float = -math.inf
    peak_step: float = 0.0
    samples: int = 0

    def update(self, value: float) -> None:
        if not math.isfinite(value):
            return
        if not math.isfinite(self.start):
            self.start = value
        if math.isfinite(self.previous):
            self.peak_step = max(self.peak_step, abs(value - self.previous))
        self.previous = value
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)
        self.samples += 1

    @property
    def travel(self) -> float:
        if not (math.isfinite(self.minimum) and math.isfinite(self.maximum)):
            return math.nan
        return self.maximum - self.minimum

    @property
    def delta(self) -> float:
        if not (math.isfinite(self.start) and math.isfinite(self.previous)):
            return math.nan
        return self.previous - self.start


def _fmt(value: float, digits: int = 6) -> str:
    return f"{value:+.{digits}f}" if math.isfinite(value) else "--"


class EncoderInspector:
    def __init__(self, root: tk.Tk, arm: Arm, poll_hz: float, history_seconds: float,
                 movement_threshold: float, export_dir: str, selected_motor: int):
        self.root = root
        self.arm = arm
        self.poll_hz = poll_hz
        self.movement_threshold = movement_threshold
        self.export_dir = Path(export_dir)
        self.motor_ids = [j.motor_id for j in arm.config.joints]
        self.models = [j.model for j in arm.config.joints]
        self.selected_motor = selected_motor if selected_motor in self.motor_ids else self.motor_ids[0]
        history_size = max(20, math.ceil(poll_hz * history_seconds))
        self.history = {mid: deque(maxlen=history_size) for mid in self.motor_ids}
        self.stats = {mid: RangeStats() for mid in self.motor_ids}
        self.last_snapshot = None
        self.started_at = time.monotonic()
        self.capture_started_at = datetime.now()
        self.messages: queue.Queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._reader, daemon=True, name="encoder-reader")
        self._closing = False

        self._build_ui()
        self.worker.start()
        self.root.after(50, self._drain_messages)

    def _build_ui(self) -> None:
        self.root.title("MakerArm Encoder Inspector — READ ONLY")
        self.root.geometry("1380x720")
        self.root.minsize(1080, 600)
        self.root.configure(bg="#111827")

        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Treeview", background="#111827", fieldbackground="#111827",
                        foreground="#e5e7eb", rowheight=31, borderwidth=0,
                        font=("Menlo", 11))
        style.configure("Treeview.Heading", background="#1f2937", foreground="#f9fafb",
                        font=("Helvetica Neue", 11, "bold"), relief="flat")
        style.map("Treeview", background=[("selected", "#1d4ed8")],
                  foreground=[("selected", "#ffffff")])
        style.configure("TButton", font=("Helvetica Neue", 11, "bold"), padding=(12, 7))

        header = tk.Frame(self.root, bg="#111827")
        header.pack(fill="x", padx=18, pady=(14, 8))
        tk.Label(header, text="MakerArm Encoder Inspector", bg="#111827", fg="#f9fafb",
                 font=("Helvetica Neue", 22, "bold")).pack(side="left")
        self.status_var = tk.StringVar(value="Connecting…")
        tk.Label(header, textvariable=self.status_var, bg="#111827", fg="#34d399",
                 font=("Menlo", 11, "bold")).pack(side="right")

        controls = tk.Frame(self.root, bg="#111827")
        controls.pack(fill="x", padx=18, pady=(0, 10))
        tk.Label(controls, text="READ ONLY • torque released • raw encoder and adjusted joint radians",
                 bg="#111827", fg="#fbbf24", font=("Helvetica Neue", 12, "bold")).pack(side="left")
        ttk.Button(controls, text="Reset ranges", command=self.reset_ranges).pack(side="right", padx=(8, 0))
        ttk.Button(controls, text="Export JSON", command=self.export_json).pack(side="right")

        body = tk.PanedWindow(self.root, orient="horizontal", bg="#111827", sashwidth=5,
                              sashrelief="flat", borderwidth=0)
        body.pack(fill="both", expand=True, padx=18, pady=(0, 12))

        table_frame = tk.Frame(body, bg="#111827")
        chart_frame = tk.Frame(body, bg="#0b1220")
        body.add(table_frame, minsize=760, width=900)
        body.add(chart_frame, minsize=300)

        columns = ("id", "model", "raw", "joint", "delta", "velocity", "minimum",
                   "maximum", "travel", "temp", "age", "state")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="browse")
        headings = {
            "id": "CAN", "model": "Model", "raw": "Raw encoder rad",
            "joint": "Joint rad", "delta": "Δ start", "velocity": "rad/s",
            "minimum": "Min", "maximum": "Max", "travel": "Travel",
            "temp": "°C", "age": "Age ms", "state": "State",
        }
        widths = {"id": 44, "model": 52, "raw": 112, "joint": 102, "delta": 82,
                  "velocity": 72, "minimum": 82, "maximum": 82, "travel": 74,
                  "temp": 48, "age": 58, "state": 70}
        for col in columns:
            self.tree.heading(col, text=headings[col])
            self.tree.column(col, width=widths[col], minwidth=45, anchor="center", stretch=False)
        self.tree.column("state", stretch=True)
        self.tree.tag_configure("static", background="#111827", foreground="#d1d5db")
        self.tree.tag_configure("moved", background="#133529", foreground="#a7f3d0")
        self.tree.tag_configure("moving", background="#075985", foreground="#e0f2fe")
        self.tree.tag_configure("fault", background="#7f1d1d", foreground="#fee2e2")
        for mid, model in zip(self.motor_ids, self.models):
            self.tree.insert("", "end", iid=f"motor-{mid}",
                             values=(mid, model, "--", "--", "--", "--", "--", "--",
                                     "--", "--", "--", "waiting"), tags=("static",))
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self._select_motor)
        self.tree.selection_set(f"motor-{self.selected_motor}")
        self.tree.focus(f"motor-{self.selected_motor}")

        tk.Label(chart_frame, text="Selected motor history", bg="#0b1220", fg="#f9fafb",
                 font=("Helvetica Neue", 16, "bold")).pack(anchor="w", padx=14, pady=(14, 4))
        self.detail_var = tk.StringVar(value=f"CAN motor {self.selected_motor}: waiting for data")
        tk.Label(chart_frame, textvariable=self.detail_var, bg="#0b1220", fg="#cbd5e1",
                 justify="left", anchor="w", wraplength=390,
                 font=("Menlo", 11)).pack(fill="x", padx=14, pady=(0, 8))
        self.canvas = tk.Canvas(chart_frame, bg="#07101f", highlightthickness=1,
                                highlightbackground="#334155")
        self.canvas.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        self.canvas.bind("<Configure>", lambda _event: self._draw_chart())

        self.footer_var = tk.StringVar(value="Range capture starts with the first sample.")
        tk.Label(self.root, textvariable=self.footer_var, bg="#111827", fg="#94a3b8",
                 anchor="w", font=("Helvetica Neue", 11)).pack(fill="x", padx=18, pady=(0, 12))

    def _reader(self) -> None:
        interval = 1.0 / self.poll_hz
        try:
            while not self.stop_event.is_set():
                cycle_start = time.monotonic()
                fresh = self.arm.refresh(wait=True, timeout=0.05)
                snapshot = {
                    "timestamp": time.monotonic(),
                    "raw": [m.feedback.position if m.feedback else math.nan for m in self.arm.motors],
                    "joint": self.arm.get_joint_positions(),
                    "velocity": self.arm.get_joint_velocities(),
                    "temperature": self.arm.get_temperatures(),
                    "faults": self.arm.get_faults(),
                    "age": [m.feedback_age for m in self.arm.motors],
                    "fresh": fresh,
                }
                try:
                    self.messages.put_nowait(("snapshot", snapshot))
                except queue.Full:
                    try:
                        self.messages.get_nowait()
                    except queue.Empty:
                        pass
                    self.messages.put_nowait(("snapshot", snapshot))
                remaining = interval - (time.monotonic() - cycle_start)
                if remaining > 0:
                    self.stop_event.wait(remaining)
        except Exception as exc:
            try:
                self.messages.put_nowait(("error", str(exc)))
            except queue.Full:
                pass

    def _drain_messages(self) -> None:
        if self._closing:
            return
        latest = None
        error = None
        while True:
            try:
                kind, payload = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == "snapshot":
                latest = payload
            else:
                error = payload
        if latest is not None:
            self._apply_snapshot(latest)
        if error:
            self.status_var.set("READER ERROR")
            self.footer_var.set(error)
        self.root.after(50, self._drain_messages)

    def _apply_snapshot(self, snapshot) -> None:
        self.last_snapshot = snapshot
        elapsed = snapshot["timestamp"] - self.started_at
        self.status_var.set(f"CONNECTED • {self.poll_hz:g} Hz • {elapsed:6.1f} s")
        for i, mid in enumerate(self.motor_ids):
            value = snapshot["joint"][i]
            stat = self.stats[mid]
            previous = stat.previous
            if snapshot["fresh"][i]:
                stat.update(value)
                self.history[mid].append((snapshot["timestamp"], value))
            step = abs(value - previous) if math.isfinite(previous) and math.isfinite(value) else 0.0
            fault = snapshot["faults"][i]
            moving_now = step >= self.movement_threshold or abs(snapshot["velocity"][i]) >= 0.02
            if not snapshot["fresh"][i]:
                state, tag = "STALE", "fault"
            elif fault:
                state, tag = "FAULT", "fault"
            elif moving_now:
                state, tag = "MOVING", "moving"
            elif math.isfinite(stat.travel) and stat.travel >= self.movement_threshold:
                state, tag = "MOVED", "moved"
            else:
                state, tag = "static", "static"
            self.tree.item(
                f"motor-{mid}",
                values=(mid, self.models[i], _fmt(snapshot["raw"][i]), _fmt(value),
                        _fmt(stat.delta, 4), _fmt(snapshot["velocity"][i], 3),
                        _fmt(stat.minimum, 4), _fmt(stat.maximum, 4), _fmt(stat.travel, 5),
                        f"{snapshot['temperature'][i]:.1f}" if math.isfinite(snapshot["temperature"][i]) else "--",
                        f"{snapshot['age'][i] * 1000:.0f}" if math.isfinite(snapshot["age"][i]) else "--",
                        state),
                tags=(tag,),
            )
        self._update_detail()
        self._draw_chart()

    def _select_motor(self, _event=None) -> None:
        selected = self.tree.selection()
        if selected:
            self.selected_motor = int(selected[0].split("-")[1])
            self._update_detail()
            self._draw_chart()

    def _update_detail(self) -> None:
        if self.last_snapshot is None:
            return
        i = self.motor_ids.index(self.selected_motor)
        stat = self.stats[self.selected_motor]
        joint = self.arm.config.joints[i]
        raw = self.last_snapshot["raw"][i]
        adjusted = self.last_snapshot["joint"][i]
        wrap_delta = adjusted - raw if math.isfinite(raw) and math.isfinite(adjusted) else math.nan
        moved = math.isfinite(stat.travel) and stat.travel >= self.movement_threshold
        self.detail_var.set(
            f"CAN motor {self.selected_motor} ({self.models[i]})\n"
            f"raw encoder   {_fmt(raw)} rad\n"
            f"joint value   {_fmt(adjusted)} rad\n"
            f"SDK adjustment {_fmt(wrap_delta)} rad\n"
            f"capture min   {_fmt(stat.minimum)} rad\n"
            f"capture max   {_fmt(stat.maximum)} rad\n"
            f"travel        {_fmt(stat.travel)} rad\n"
            f"peak step     {_fmt(stat.peak_step)} rad\n"
            f"soft limits   [{joint.lo:+.3f}, {joint.hi:+.3f}] rad\n"
            f"fault         {fault_text(self.last_snapshot['faults'][i])}\n"
            f"result        {'MOTION DETECTED' if moved else 'NO MOTION ABOVE THRESHOLD'}"
        )

    def _draw_chart(self) -> None:
        if not hasattr(self, "canvas"):
            return
        self.canvas.delete("all")
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        if width < 40 or height < 40:
            return
        pad = 38
        points = [(t, y) for t, y in self.history[self.selected_motor] if math.isfinite(y)]
        self.canvas.create_text(pad, 16, text="joint encoder rad (auto-scaled)", anchor="w",
                                fill="#94a3b8", font=("Helvetica Neue", 10))
        if len(points) < 2:
            self.canvas.create_text(width / 2, height / 2, text="Waiting for samples…",
                                    fill="#64748b", font=("Helvetica Neue", 14))
            return
        t0, t1 = points[0][0], points[-1][0]
        values = [y for _, y in points]
        lo, hi = min(values), max(values)
        center = (lo + hi) / 2
        half_span = max((hi - lo) * 0.6, self.movement_threshold * 1.5, 0.0005)
        lo, hi = center - half_span, center + half_span
        plot_w, plot_h = width - 2 * pad, height - 2 * pad
        self.canvas.create_rectangle(pad, pad, pad + plot_w, pad + plot_h,
                                     outline="#334155", fill="#07101f")
        for n in range(1, 4):
            y = pad + plot_h * n / 4
            self.canvas.create_line(pad, y, pad + plot_w, y, fill="#1e293b")
        duration = max(t1 - t0, 1e-6)
        coords = []
        for timestamp, value in points:
            x = pad + (timestamp - t0) / duration * plot_w
            y = pad + (hi - value) / (hi - lo) * plot_h
            coords.extend((x, y))
        self.canvas.create_line(*coords, fill="#38bdf8", width=2, smooth=False)
        self.canvas.create_text(4, pad, text=f"{hi:+.6f}", anchor="w", fill="#94a3b8",
                                font=("Menlo", 9))
        self.canvas.create_text(4, pad + plot_h, text=f"{lo:+.6f}", anchor="w", fill="#94a3b8",
                                font=("Menlo", 9))
        self.canvas.create_text(pad, height - 14, text=f"{duration:.1f} s history",
                                anchor="w", fill="#64748b", font=("Helvetica Neue", 9))
        self.canvas.create_text(width - pad, height - 14,
                                text=f"span {max(values) - min(values):.6f} rad",
                                anchor="e", fill="#38bdf8", font=("Menlo", 9, "bold"))

    def reset_ranges(self) -> None:
        self.capture_started_at = datetime.now()
        self.stats = {mid: RangeStats() for mid in self.motor_ids}
        for values in self.history.values():
            values.clear()
        self.footer_var.set("Ranges and charts reset. Move joints by hand; torque remains released.")

    def export_json(self) -> None:
        if self.last_snapshot is None:
            self.footer_var.set("No samples available to export yet.")
            return
        self.export_dir.mkdir(parents=True, exist_ok=True)
        path = self.export_dir / f"encoder_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        joints = []
        for i, mid in enumerate(self.motor_ids):
            stat = self.stats[mid]
            joint = self.arm.config.joints[i]
            joints.append({
                "motor_id": mid,
                "model": self.models[i],
                "raw_encoder_rad": self.last_snapshot["raw"][i],
                "joint_rad": self.last_snapshot["joint"][i],
                "capture_start_rad": stat.start,
                "capture_min_rad": stat.minimum if math.isfinite(stat.minimum) else None,
                "capture_max_rad": stat.maximum if math.isfinite(stat.maximum) else None,
                "capture_travel_rad": stat.travel,
                "peak_sample_step_rad": stat.peak_step,
                "configured_lo_rad": joint.lo,
                "configured_hi_rad": joint.hi,
                "fault_bits": self.last_snapshot["faults"][i],
            })
        with path.open("w") as handle:
            json.dump({"timestamp": datetime.now().isoformat(timespec="seconds"),
                       "capture_started": self.capture_started_at.isoformat(timespec="seconds"),
                       "poll_hz": self.poll_hz,
                       "movement_threshold_rad": self.movement_threshold,
                       "read_only": True, "joints": joints}, handle, indent=2)
        self.footer_var.set(f"Exported diagnostic capture to {path}")

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1.5)
        self.arm.disconnect()
        self.root.destroy()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--serial-baudrate", type=int,
                    help="serial rate (SLCAN default 115200; AT default 921600)")
    ap.add_argument("--slcan-startup-delay", type=float, default=2.5)
    ap.add_argument("--poll-hz", type=float, default=10.0)
    ap.add_argument("--history-seconds", type=float, default=30.0)
    ap.add_argument("--movement-threshold", type=float, default=0.002,
                    help="cumulative/instant movement threshold in rad")
    ap.add_argument("--selected-motor", type=int, default=4)
    ap.add_argument("--export-dir", default="configs")
    a = ap.parse_args()
    if a.poll_hz <= 0 or a.poll_hz > 25:
        raise SystemExit("--poll-hz must be in (0, 25]")
    if a.history_seconds <= 0:
        raise SystemExit("--history-seconds must be positive")
    if a.movement_threshold <= 0:
        raise SystemExit("--movement-threshold must be positive")

    if a.backend == "socketcan":
        kw = {"channel": a.channel}
    elif a.backend == "slcan":
        kw = {"port": a.port, "baudrate": a.serial_baudrate or 115200,
              "rtscts": False, "startup_delay": a.slcan_startup_delay}
    else:
        kw = {"port": a.port, "baudrate": a.serial_baudrate or 921600}

    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    root = tk.Tk()
    app = EncoderInspector(root, arm, a.poll_hz, a.history_seconds,
                           a.movement_threshold, a.export_dir, a.selected_motor)
    root.protocol("WM_DELETE_WINDOW", app.close)
    try:
        root.mainloop()
    finally:
        if not app._closing:
            app.close()


if __name__ == "__main__":
    main()
