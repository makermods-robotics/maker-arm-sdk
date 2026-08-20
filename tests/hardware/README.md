# Hardware acceptance

Hardware acceptance is intentionally operator-run and is not part of automated CI. Before a
release, verify seven-motor scan, zero reference, bounded response, 60-second hold, absolute
teleoperation, SLCAN reconnect, watchdog behavior, and explicit torque release on both supported
platform paths.
