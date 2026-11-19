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

The first version is vehicle-agnostic: it polls standard OBD-II PIDs (RPM and throttle position to start) over the OBD-II port and works on most cars from 2008 on. Vehicle-specific variants will come later. It streams the shared [Gnimu frame](https://github.com/c20d-us/Gnimu-protocol); what it fills and how to set up AutoXDL are in [docs/ble-protocol.md](docs/ble-protocol.md).

> [!NOTE]
> **Work in progress.** The firmware finds the car's CAN bit rate (or reaches a gatewayed port with a probe), discovers the engine ECU, polls RPM and throttle position, and streams them with the supply voltage in Gnimu frames over BLE. It sleeps when the car is parked or its battery is low, waking on a timer to check if it should resume operation. It has bench-tested against a fake ECU and with AutoXDL; the first car test is pending arrival of a custom PCB that will allow the hardware to be installed into a compact project box with a J1962 OBD-II plug ([docs/car-test.md](docs/car-test.md)). Nothing has been tagged or released, and anything here may change at any time.

> [!WARNING]
> **This device connects to your car's diagnostic port and transmits on its CAN bus.** It's a personal project, not a product: it hasn't been tested on any car other than the author's, and it comes with no warranty. If you build or use it, you do so entirely at your own risk. Read [Use at your own risk](#use-at-your-own-risk) first.

## Hardware

The firmware targets the Gnimu-CAN carrier boards in [Gnimu-PCB](https://github.com/c20d-us/Gnimu-PCB/tree/main/boards/gnimu-can): a Seeed Studio XIAO ESP32C6 with an Adafruit CAN Pal transceiver, connected to the car through the OBD-II helper board. The first target is the 12V carrier.

| XIAO pin | GPIO | Net |
|---|---|---|
| D0/A0 | 0 | `V12_SENSE`, supply voltage ÷11 |
| D8 | 19 | `CAN_SILENT`, CAN Pal S (high = listen-only) |
| D9 | 20 | `CAN_TX` |
| D10 | 18 | `CAN_RX` |

> [!NOTE]
> The firmware design is in [docs/architecture.md](docs/architecture.md).

## Toolchain

- [`arduino-cli`](https://arduino.github.io/arduino-cli/) (`brew install arduino-cli`)
- ESP32 Arduino core **3.3.12**. Install it with `scripts/setup.sh`. CAN (TWAI) and BLE (NimBLE) support both come with the core, so no extra libraries are needed.

## Build and flash

```bash
scripts/build.sh
```

```bash
scripts/flash.sh --monitor
```

`build.sh` stamps the version, git revision and build time into the firmware, and fails on any compiler warning in the project's own code. `flash.sh` uploads what was last built (add `--build` to build first) and finds the XIAO by its USB ID; with more than one plugged in, pass `--port /dev/cu.usbmodemXXXX`. With a USB-C host, connect USB before powering the board from 12V, or it won't show up (see [Bench rig](docs/architecture.md#bench-rig)).

The host tests cover the hardware-independent logic in `code/GnimuCAN/src/core`:

```bash
tests/run.sh
```

## Layout

```
code/GnimuCAN/          The firmware sketch
  GnimuCAN.ino          Open this in the Arduino IDE
  Config.h              The settings you might change
  src/
    core/               Logic with no hardware: scheduler, states, supply monitor, OBD-II codec and poller, report text (host-tested)
    platform/           The board: clock, watchdog, reset reason, CAN controller, LED, supply ADC, console port
    peripherals/        BLE (the Gnimu service on NimBLE)
    protocol/           The Gnimu frame encoder, copied from Gnimu-protocol
    app/                The job table, states, boot log, console and diagnostics
docs/
  architecture.md       Firmware design
  ble-protocol.md       BLE identity, frame fields and AutoXDL setup
  car-test.md           Checklist for the first car test
scripts/                setup.sh, build.sh, flash.sh, and tool.sh for the diagnostic sketches
tests/                  Host tests for src/core
tools/                  Standalone diagnostic sketches
```

To check a new board and the command-line toolchain, see [tools/](tools/README.md). The plan for the first test in a car is [docs/car-test.md](docs/car-test.md).

## Use at your own risk

Gnimu-CAN is open source, and anyone is welcome to build, use or fork it. That said, it plugs into your car and interacts with the vehicle's electronics. **Make sure you understand what that means before you experiment with it.**

- **It transmits on your car's bus.** It listens first, and it's designed to stay quiet unless it has a reason to speak, but to read data it sends OBD-II requests through the connector, and on a car whose diagnostic port is behind a gateway it sends probe frames to find out whether the car is on. Any device on a car's bus can, in the wrong circumstances, cause warning lights, stored fault codes, modules that wake up or misbehave, or worse.
- **It draws power from your car's battery.** The OBD-II port is powered even when the car is off. The firmware sleeps when the car is parked, but it isn't designed to look after a battery for weeks. Switch it off or unplug it when you park for long periods.
- **The hardware is yours to get right.** Wiring mistakes, a short, the wrong termination setting or a poor connection can damage the device, the car's wiring or its electronics. Check your build carefully before plugging it in.
- **Don't let it distract you while driving.** Set it up, read the console and change settings with the car parked, or with a passenger doing it. Follow your local laws, and the rules of any event or track you take it to.
- **It's not affiliated with any car maker, and nobody supports it for you.** Vehicle and product names used here are for identification only.

**You are solely responsible for how you use this software and hardware, and for any damage to your vehicle, other property or anyone else that results.** As the [license](LICENSE) states, it's provided "as is", without warranty of any kind, and the author accepts no liability for any loss or damage arising from its use. If you aren't comfortable with that, *do not* connect it to a car.

## License

GPL-3.0. See [LICENSE](LICENSE).
