# Engineering tools

`tools/engineering/calibration/` contains factory characterization utilities that can rewrite
limits or Star mapping. They are retained for developing a new arm revision, not installed as
public commands, and not part of replacement-motor zero calibration.

`tools/diagnostics/` contains inspection and bounded bench tools. Read each module's warning
before connecting hardware.

`tools/experimental/lerobot/switch_protocol.py` is the one-time factory switch of a new motor
from RobStride's private protocol to MIT. The switch is persistent and takes effect after a power
cycle; the SDK only speaks MIT afterwards.
