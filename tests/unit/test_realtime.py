"""Keeping an enabled arm's own threads responsive: fast-core placement, the GIL switch interval, real-time
stepping of a late control loop, the measured loop rate, frame spacing, and silence judged from the request."""

import sys
import threading
import time

import pytest

from conftest import sdk
from fakes import make_arm

realtime = sdk.realtime
Arm, ArmState = sdk.arm.Arm, sdk.arm.ArmState


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


def fake_sysfs(tmp_path, capacities, name="cpu_capacity"):
    for cpu, value in capacities.items():
        d = tmp_path / f"cpu{cpu}" / name
        d.parent.mkdir(parents=True, exist_ok=True)
        d.write_text(f"{value}\n")
    return str(tmp_path)


def test_fast_cpus_on_big_little(tmp_path):
    sysfs = fake_sysfs(tmp_path, {0: 442, 1: 442, 2: 1024, 3: 1024})
    assert realtime.fast_cpus({0, 1, 2, 3}, sysfs) == {2, 3}
    assert realtime.fast_cpus({0, 1}, sysfs) is None          # only small cores allowed: nothing to choose
    assert realtime.fast_cpus({2, 3}, sysfs) is None          # already on the fast ones


def test_fast_cpus_on_alike_or_unknown_cores(tmp_path):
    assert realtime.fast_cpus({0, 1}, fake_sysfs(tmp_path / "a", {0: 1024, 1: 1024})) is None
    assert realtime.fast_cpus({0, 1}, str(tmp_path / "missing")) is None


def test_fast_cpus_fall_back_to_the_maximum_clock(tmp_path):
    sysfs = fake_sysfs(tmp_path, {0: 2016000, 1: 2208000}, name="cpufreq/cpuinfo_max_freq")
    assert realtime.fast_cpus({0, 1}, sysfs) == {1}


def test_pinning_can_be_switched_off(monkeypatch):
    monkeypatch.setenv(realtime.TUNING_ENV, "off")
    assert realtime.pin_to_fast_cpus() is None


def test_switch_interval_is_lowered_while_held_and_restored_after():
    before = sys.getswitchinterval()
    s = realtime.SwitchInterval(0.0005)
    s.acquire()
    s.acquire()
    assert sys.getswitchinterval() == pytest.approx(0.0005, rel=0.01)
    s.release()
    assert sys.getswitchinterval() == pytest.approx(0.0005, rel=0.01)
    s.release()
    assert sys.getswitchinterval() == before
    s.release()   # extra release: harmless
    assert sys.getswitchinterval() == before


def test_switch_interval_never_raised_or_overwritten(monkeypatch):
    before = sys.getswitchinterval()
    try:
        sys.setswitchinterval(0.0001)
        s = realtime.SwitchInterval(0.0005)
        s.acquire()
        assert sys.getswitchinterval() == pytest.approx(0.0001, rel=0.01)   # already lower: kept
        s.release()
        sys.setswitchinterval(0.005)
        s.acquire()
        sys.setswitchinterval(0.002)                                       # the caller changed it meanwhile
        s.release()
        assert sys.getswitchinterval() == pytest.approx(0.002, rel=0.01)
        monkeypatch.setenv(realtime.TUNING_ENV, "0")
        s.acquire()
        assert sys.getswitchinterval() == pytest.approx(0.002, rel=0.01)
        s.release()
    finally:
        sys.setswitchinterval(before)


def enabled_arm(**overrides):
    arm, bus, fakes = make_arm(**overrides)
    arm.connect()
    arm.enable(start_loop=False)
    return arm, bus, fakes


def run_loop(arm, seconds):
    arm._running = True
    thread = threading.Thread(target=arm._control_loop, args=(arm._session,), daemon=True)
    thread.start()
    time.sleep(seconds)
    arm._running = False
    thread.join(2.0)
    assert not thread.is_alive()


def test_a_late_tick_advances_by_the_time_that_passed_up_to_a_cap(monkeypatch):
    arm, _, _ = enabled_arm(control_rate_hz=100.0)
    seen = []
    tick = arm._tick

    def slow_tick(dt, session=None):
        seen.append(dt)
        tick(dt, session)
        if len(seen) == 3:
            time.sleep(0.015)   # 1.5 periods late
        if len(seen) == 6:
            time.sleep(0.2)     # far behind: capped

    monkeypatch.setattr(arm, "_tick", slow_tick)
    run_loop(arm, 0.4)
    period = 0.01
    assert all(period <= dt <= sdk.arm.MAX_TICK_PERIODS * period + 1e-9 for dt in seen)
    assert seen[3] >= 0.015
    assert seen[6] == pytest.approx(sdk.arm.MAX_TICK_PERIODS * period)
    arm.disable()


def test_loop_rate_is_measured_and_reported(monkeypatch):
    monkeypatch.setattr(sdk.arm, "LOOP_RATE_WINDOW_S", 0.1)
    arm, _, _ = enabled_arm(control_rate_hz=100.0)
    assert arm.status()["loop_rate_hz"] is None
    run_loop(arm, 0.45)
    assert 20 < arm.loop_rate_hz <= 130   # a late tick's successor can come early: one extra tick per window
    assert arm.status()["loop_rate_hz"] == pytest.approx(arm.loop_rate_hz, abs=0.1)
    arm.disable()


def test_slow_loop_is_reported_once(monkeypatch, caplog):
    monkeypatch.setattr(sdk.arm, "LOOP_RATE_WINDOW_S", 0.05)
    arm, _, _ = enabled_arm(control_rate_hz=100.0)
    tick = arm._tick

    def slow_tick(dt, session=None):
        tick(dt, session)
        time.sleep(0.03)

    monkeypatch.setattr(arm, "_tick", slow_tick)
    with caplog.at_level("WARNING"):
        run_loop(arm, 0.5)
    assert arm.loop_rate_hz < 50
    assert sum("cannot keep up" in r.getMessage() for r in caplog.records) == 1
    arm.disable()


def test_frames_are_spaced_only_when_a_send_is_quicker_than_the_spacing(monkeypatch):
    spacing = 0.005
    monkeypatch.setattr(sdk.arm, "FRAME_SPACING_S", spacing)
    arm, bus, _ = enabled_arm()
    waits = []
    arm._busy_wait_us = lambda us: waits.append(us)
    arm._tick(0.005)
    assert waits and all(0 < us <= spacing * 1e6 for us in waits)   # quick sends: spaced
    waits.clear()
    send = bus.send

    def slow_send(*a, **k):
        time.sleep(1.5 * spacing)
        return send(*a, **k)

    monkeypatch.setattr(bus, "send", slow_send)
    arm._tick(0.005)
    assert waits == []   # each send already took longer than the spacing
    arm.disable()


def test_a_stalled_loop_is_not_taken_for_a_silent_motor():
    arm, _, _ = enabled_arm(feedback_timeout=0.05)
    arm._tick(0.005)
    time.sleep(0.15)          # the loop could not run, so nothing asked the motors
    arm._tick(0.005)
    assert arm.state is ArmState.ENABLED


def test_a_silent_motor_still_faults_after_a_stall():
    arm, _, fakes = enabled_arm(feedback_timeout=0.05)
    arm._tick(0.005)
    time.sleep(0.15)
    fakes[1].online = False
    arm._tick(0.005)          # asked now; no answer
    assert arm.state is ArmState.ENABLED
    time.sleep(0.06)
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "feedback timeout" in arm.fault_reason
    arm.disable()
