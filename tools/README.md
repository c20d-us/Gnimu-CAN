# Tools

Standalone diagnostic sketches for the XIAO ESP32C6. Each one is a folder with a single `.ino` of the same name. None of them are part of the firmware.

Build, flash and open the console with [`scripts/tool.sh`](../scripts/tool.sh):

```bash
scripts/tool.sh board_check --flash --monitor
```

- `scripts/tool.sh --list` lists the tools.
- `--monitor` on its own opens the console without building or flashing.
- With one XIAO plugged in, the port is found automatically. With several, pass `--port /dev/cu.usbmodemXXXX`.

If an upload can't connect, put the XIAO into its bootloader by hand: hold **BOOT**, tap **RESET**, release **BOOT**, then flash again. Press **RESET** afterwards to start the new sketch.

| Tool | What it does |
|---|---|
| [`board_check`](board_check/) | Confirms the toolchain and a bare XIAO: prints chip and flash details, blinks the LED, advertises over BLE, reads A0 once a second, and echoes console input. |
| [`fake_ecu`](fake_ecu/) | A bench stand-in for a car's engine ECU on a second XIAO and CAN Pal: answers OBD-II Mode 01 requests with sweeping RPM and throttle. Console commands switch it between broadcast chatter and a silent gatewayed port, change the bit rate and reply delay, and turn it off and on like the car's ignition. |
