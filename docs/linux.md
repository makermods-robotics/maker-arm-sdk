# Linux

Linux uses the kernel SocketCAN API without a `python-can` dependency.

For a native SocketCAN adapter, configure `can0` for classic CAN at 1 Mbps using the adapter's
normal system setup. For a serial SLCAN device:

```bash
sudo apt install can-utils
sudo bash scripts/linux/setup_slcan.sh /dev/ttyACM0 can0
maker-arm scan --channel can0 --max-id 7
```

Rerun the setup script after reconnecting a serial SLCAN dongle.
