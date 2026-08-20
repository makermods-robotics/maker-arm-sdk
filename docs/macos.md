# macOS

macOS has no SocketCAN interface, so Maker Arm uses the CANable's serial SLCAN protocol directly.
Prefer `/dev/cu.*` callout devices.

```bash
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
maker-arm doctor
maker-arm scan --backend slcan --port /dev/cu.usbmodem-FOLLOWER --max-id 7
```

The supported defaults are 1 Mbps CAN, 115200 serial line coding, RTS/CTS disabled, automatic
CAN retransmit disabled, 2.5 seconds startup delay, and 25 Hz sequential-feedback control.
The tested dongle identifies as original `normaldotcom/canable-fw` commit `9fddea4`.

If even the firmware `V` command stops responding, completely depower the dongle. Some wiring
can back-power it from the CAN side, so removing USB alone may not reset the MCU.
