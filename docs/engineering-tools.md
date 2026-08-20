# Engineering tools

`tools/engineering/calibration/` contains factory characterization utilities that can rewrite
limits or Star mapping. They are retained for developing a new arm revision, not installed as
public commands, and not part of replacement-motor zero calibration.

`tools/diagnostics/` contains inspection and bounded bench tools. Read each module's warning
before connecting hardware.

`tools/experimental/lerobot/` contains unsupported protocol-switching work. Switching protocol
is persistent, requires power cycles, and removes motors from the normal Maker Arm SDK workflow.
