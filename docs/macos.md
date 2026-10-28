# macOS

macOS has no SocketCAN interface, so Maker Arm uses the CANable's serial SLCAN protocol directly.
That is the default backend on macOS; with exactly one `/dev/cu.usbmodem*` adapter plugged in
it is found automatically, otherwise pass `--port`.

```bash
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
maker-arm doctor
maker-arm scan
```

Only one program can use the adapter. `maker-arm doctor` names any process holding it open
(for example MakerModsLab); close it first.

The supported defaults are 1 Mbps CAN, 115200 serial line coding, RTS/CTS disabled, automatic
CAN retransmit disabled, 2.5 seconds startup delay, and 25 Hz sequential-feedback control.
The tested dongle identifies as original `normaldotcom/canable-fw` commit `9fddea4`.
candleLight / gs_usb adapters (supported by MakerModsLab) are not supported by the SDK.

If even the firmware `V` command stops responding, completely depower the dongle. Some wiring
can back-power it from the CAN side, so removing USB alone may not reset the MCU.
