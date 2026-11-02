<p>
  <img src="https://raw.githubusercontent.com/c20d-us/Gnimu-branding/main/svg/gnimu-icon-default-rounded.svg" alt="Gnimu icon" height="72">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/c20d-us/Gnimu-branding/main/wordmark/gnimu-wordmark-transparent-white.svg">
    <img src="https://raw.githubusercontent.com/c20d-us/Gnimu-branding/main/wordmark/gnimu-wordmark-transparent-navy.svg" alt="gnimu" height="72">
  </picture>
</p>

# Gnimu-CAN: CAN bus motorsports telemetry

[![License: GPL v3+](https://img.shields.io/badge/License-GPLv3%2B-blue.svg)](LICENSE)
[![Platform: ESP32-C6](https://img.shields.io/badge/platform-ESP32--C6-000000.svg)](#hardware)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

Gnimu-CAN is firmware for a Seeed Studio XIAO ESP32C6 that reads vehicle data from a car's CAN bus and streams it over BLE (Bluetooth Low Energy) at a fixed rate, for telemetry apps to record alongside GNSS and IMU data. It's a companion to [Gnimu](https://github.com/c20d-us/Gnimu), and its first target app is [AutoX Data Logger](https://autoxdrivermod.com), which reads it as a custom BLE sensor.

The first version is vehicle-agnostic: it polls standard OBD-II PIDs (RPM and throttle position to start) over the OBD-II port and works on most cars from 2008 on. Vehicle-specific variants come later. It streams the shared [Gnimu frame](https://github.com/c20d-us/Gnimu-protocol); what it fills and how to set up AutoXDL are in [docs/ble-protocol.md](docs/ble-protocol.md).

> [!NOTE]
> **Work in progress.** The design is written up and a breadboard prototype is being built, but the firmware itself isn't written yet. Anything here may change.

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
  ble-protocol.md    BLE identity, frame fields and AutoXDL setup
scripts/             build and flash helpers
tools/               standalone diagnostic sketches, built with scripts/tool.sh
```

To check a new board and the command-line toolchain, see [tools/](tools/README.md).

## License

GPL-3.0. See [LICENSE](LICENSE).
