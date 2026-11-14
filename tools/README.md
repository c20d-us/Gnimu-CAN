# Tools

Standalone diagnostic sketches for the XIAO ESP32C6. Each one is a folder with a single `.ino` of the same name. None of them are part of the firmware.

Build, flash and open the console with [`scripts/tool.sh`](../scripts/tool.sh):

```bash
scripts/tool.sh board_check --flash --monitor
```

- `scripts/tool.sh --list` lists the tools.
- `--monitor` on its own opens the console without building or flashing.
- `--dump <file>` saves a tool's log to a file, for tools that keep one (`voltage_log`).
- With one XIAO plugged in, the port is found automatically. With several, pass `--port /dev/cu.usbmodemXXXX`.

If an upload can't connect, put the XIAO into its bootloader by hand: hold **BOOT**, tap **RESET**, release **BOOT**, then flash again. Press **RESET** afterwards to start the new sketch.

| Tool | What it does |
|---|---|
| [`board_check`](board_check/) | Confirms the toolchain and a bare XIAO: prints chip and flash details, blinks the LED, advertises over BLE, reads A0 once a second, and echoes console input. |
| [`fake_ecu`](fake_ecu/) | A bench stand-in for a car's engine ECU on a second XIAO and CAN Pal: answers OBD-II Mode 01 requests with sweeping RPM and throttle. Console commands switch it between broadcast chatter and a silent gatewayed port, change the bit rate and reply delay, and turn it off and on like the car's ignition. |
| [`voltage_log`](voltage_log/) | A car-side logger for a complete unit: once a second it appends the supply voltage (mean, min and max over the second) and the OBD port's CAN frame and bus-error counts to a log in flash, listening only. Holds about 16 hours, survives power cycles, and starts a new section each boot. |

### Logging a drive with `voltage_log`

1. Flash it with the unit on USB: `scripts/tool.sh voltage_log --flash --monitor`. Type `CLEAR LOG` to start from an empty log, and `STATUS` to check it's logging.
2. In the car, with the CAN Pal's termination switch **off**, plug the unit into the OBD-II port. It logs from power-up; the LED blinks once a second while it does.
3. Do the drive. Note roughly when each step happened (engine started, pulled away, parked, switched off), as minutes after plugging in: the log's `t_s` column counts seconds from power-up.
4. Leave it plugged in for 30–60 minutes after switching off, then unplug it.
5. Back at the computer, with the unit on USB only: `scripts/tool.sh voltage_log --dump drive.csv`.

A brown-out while cranking restarts the unit; the log then has a new `# boot` section, with `t_s` counting from that restart.
