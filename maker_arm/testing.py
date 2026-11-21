"""Simulated arms for application tests, no hardware: a real Arm on a MockBackend whose simulated
motors track MIT commands, report status and temperatures, and take injected faults.

    arm, sim = simulated_arm()             # shipped profile, each joint at zero clipped into its limits
    arm.connect()
    arm.enable()
    arm.move_to(q)
    sim.positions                          # where the simulated joints are (rad)
    sim.fault("gripper"); sim.set_temperature("elbow_flex", 80.0); sim.offline("wrist_roll")

    group, sims = simulated_group({"arms": {"left": {"backend": "gs_usb"}, "right": {"backend": "gs_usb"}}})
    with group:                            # a whole robot from a rig description (Rig, dict or path)
        group.enable()

The control loop runs on the wall clock, and a loaded test machine can delay its wake-ups by
hundreds of milliseconds. Simulated arms therefore use simulation_timing(config): the same profile
with a longer host-side feedback deadline (which also sets the status-reply deadline). Shipped
profiles and arms on real buses keep their own values.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Iterable, Optional, Sequence, Union

from . import _sim_motor
from ._product import DEFAULT_PROFILE
from .arm import Arm
from .config import ArmConfig
from .rig import ArmGroup, Rig

ConfigLike = Union[ArmConfig, str, Path, None]

SIM_FEEDBACK_TIMEOUT = 1.0   # s, host-side feedback deadline for simulated motors (at least)


class SimulatedMotors:
    """The simulated motors behind one arm, addressed by joint name or index."""

    def __init__(self, config: ArmConfig, motors: list):
        self.config = config
        self.motors = {j.label: m for j, m in zip(config.joints, motors)}   # joint name -> simulated motor

    def motor(self, joint: Union[str, int]):
        if isinstance(joint, int) and not isinstance(joint, bool):
            return list(self.motors.values())[joint]
        if joint not in self.motors:
            raise KeyError(f"no joint {joint!r} (joints: {', '.join(self.motors)})")
        return self.motors[joint]

    @property
    def positions(self) -> list[float]:
        """Joint positions of the simulated motors, rad."""
        return [(m.position - j.offset) * j.direction for j, m in zip(self.config.joints, self.motors.values())]

    def fault(self, joint: Union[str, int], faulted: bool = True) -> None:
        """Make the motor report a fault (False: clear it)."""
        self.motor(joint).faulted = faulted

    def set_temperature(self, joint: Union[str, int], celsius: float) -> None:
        self.motor(joint).temperature = celsius

    def offline(self, joint: Union[str, int], offline: bool = True) -> None:
        """Stop the motor answering (False: answer again)."""
        self.motor(joint).online = not offline

    def watchdog_timeout(self, joint: Union[str, int]) -> None:
        """Do what the motor's watchdog does once commands stop (torque off, and the state the
        protocol reports for it)."""
        _sim_motor.watchdog_timeout(self.motor(joint))


def simulation_timing(config: ArmConfig) -> ArmConfig:
    """A copy of ``config`` whose feedback deadline is at least SIM_FEEDBACK_TIMEOUT, for simulated
    motors on a machine that may starve the control loop."""
    return dataclasses.replace(config, feedback_timeout=max(config.feedback_timeout, SIM_FEEDBACK_TIMEOUT))


def _config(config: ConfigLike) -> ArmConfig:
    if isinstance(config, ArmConfig):
        return config
    return ArmConfig.from_yaml(str(config or DEFAULT_PROFILE))


def simulated_backend(config: ConfigLike = None, positions: Optional[Sequence[float]] = None,
                      **options) -> tuple:
    """(backend, SimulatedMotors): a MockBackend with one simulated motor per joint of ``config``
    (an ArmConfig, a profile path, or None for the shipped profile) at ``positions`` (rad, default
    zero clipped into each joint's limits), its timing relaxed by simulation_timing(). ``options``
    go to every simulated motor (e.g. ``temperature=40.0``)."""
    config = simulation_timing(_config(config))
    joints = config.joints
    if positions is None:
        positions = [min(j.hi, max(j.lo, 0.0)) for j in joints]
    if len(positions) != len(joints):
        raise ValueError(f"positions needs one value per joint ({len(joints)}), got {len(positions)}")
    motors = [_sim_motor.for_joint(j, config.host_id, q * j.direction + j.offset, **options)
              for j, q in zip(joints, positions)]
    return _sim_motor.SimBus(motors), SimulatedMotors(config, motors)


def simulated_arm(config: ConfigLike = None, positions: Optional[Sequence[float]] = None,
                  **options) -> tuple[Arm, SimulatedMotors]:
    """(arm, sim): a real Arm, not connected, on simulated_backend(config, positions, **options)."""
    backend, sim = simulated_backend(config, positions, **options)
    arm = Arm(sim.config, backend)
    arm._busy_wait_us = lambda us: None   # frame spacing is for real buses
    return arm, sim


def simulated_group(rig: Union[Rig, dict, str, Path], names: Iterable[str] = ("all",),
                    positions: Optional[dict[str, Sequence[float]]] = None,
                    **options) -> tuple[ArmGroup, dict[str, SimulatedMotors]]:
    """(group, {name: sim}): the rig's arms (``rig.group(names)``) each with its profile on its own
    simulated bus; the rig's backends and addresses are ignored. ``rig`` is a Rig, the contents of
    a rig file, or its path; ``positions`` maps arm names to starting positions."""
    if isinstance(rig, dict):
        rig = Rig.from_dict(rig)
    elif not isinstance(rig, Rig):
        rig = Rig.load(rig, missing_ok=False)
    arms, sims = {}, {}
    for name in rig.names(names):
        arms[name], sims[name] = simulated_arm(rig.spec(name).config(rig.base), (positions or {}).get(name),
                                               **options)
    return ArmGroup(arms, parallel=True), sims
