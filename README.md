# Gnimu-CAN

Firmware for a Seeed Studio XIAO ESP32C6 that reads vehicle data from CAN and publishes it over Bluetooth LE for a receiver to subscribe to.

This is the OBD-II version: it polls standard Mode 01 PIDs (RPM, speed, temperatures, and so on) using 11-bit ISO 15765-4 requests, and sends each response as a BLE notification. The frame format is in [docs/ble-protocol.md](docs/ble-protocol.md).

## Hardware

The firmware targets the Gnimu-CAN carrier boards in [Gnimu-PCB](https://github.com/c20d-us/Gnimu-PCB/tree/main/boards/gnimu-can): a Seeed Studio XIAO ESP32C6 with an Adafruit CAN Pal transceiver, connected to the car through the OBD-II helper board. The first target is the 12V carrier.

| XIAO pin | GPIO | Net |
|---|---|---|
| D0/A0 | 0 | `V12_SENSE`, supply voltage ÷11 |
| D8 | 19 | `CAN_SILENT`, CAN Pal S (high = listen-only) |
| D9 | 20 | `CAN_TX` |
| D10 | 18 | `CAN_RX` |

> [!NOTE]
> The firmware design is in [docs/architecture.md](docs/architecture.md). The sketch in `code/GnimuCAN/` is an early skeleton that predates the design. It still uses D6/D7 for CAN and will be replaced.

## Toolchain

- [`arduino-cli`](https://arduino.github.io/arduino-cli/) (`brew install arduino-cli`)
- ESP32 Arduino core **3.3.12**. Install it with `make setup`. CAN (TWAI) and BLE (NimBLE) support both come with the core, so no extra libraries are needed.

## Build and flash

```bash
make build
```

```bash
make flash
```

```bash
make monitor
```

`flash` and `monitor` find the first `/dev/cu.usbmodem*` port. If you have more than one board plugged in, pass `PORT=/dev/cu.usbmodemXXXX`.

## Layout

```
code/GnimuCAN/       The firmware sketch
  GnimuCAN.ino       Open this in the Arduino IDE
  Config.h           The settings you might change
  src/               Everything else
docs/
  architecture.md    firmware design
  ble-protocol.md    BLE packet format for receivers
scripts/             build and flash helpers
tools/               standalone diagnostic sketches, built with scripts/tool.sh
```

To check a new board and the command-line toolchain, see [tools/](tools/README.md).

## License

GPL-3.0. See [LICENSE](LICENSE).
