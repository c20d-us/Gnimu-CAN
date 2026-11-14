# Gnimu-CAN over BLE

Gnimu-CAN streams the **[Gnimu frame](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md)**, the one telemetry layout shared by every Gnimu-family device. It never sends anything else. This page covers only what's specific to Gnimu-CAN: its identity, which frame fields it fills, and how to set up AutoX Data Logger (AutoXDL). The layout itself, the conventions, and the encoding rules are in the [Gnimu protocol](https://github.com/c20d-us/Gnimu-protocol) repo.

> [!NOTE]
> Gnimu frame v1 is still in progress, so details here may change before it's published. Everything on this page is implemented except the Control characteristic.

## Identity

The device advertises as **`Gnimu-CAN <device ID>`**, with the service UUID in its advertising data. AutoXDL's scan only lists devices that advertise the service UUID entered in its settings.

These are the Gnimu protocol's UUIDs, the same on every Gnimu device. The Control characteristic isn't implemented yet.

| Item | UUID | Properties |
|---|---|---|
| Service | `9074beb4-2f1a-4b5f-9818-fe2d2277ef05` | |
| Frame characteristic | `0752d3c3-ceaf-462f-a944-bcc06c57284b` | Notify, Read |
| Schema characteristic | `1c9b3945-10eb-4fd0-a5a4-b9d80842380d` | Read |
| Device info characteristic | `6d0adfef-8218-488a-a5f9-1ba05f422c22` | Read |

It also provides the standard Device Information Service (`0x180A`):

| Characteristic | Value |
|---|---|
| Manufacturer Name | `Gnimu` |
| Model Number | `Gnimu-CAN` |
| Serial Number | The 10-digit device ID |
| Firmware Revision | The firmware version, such as `0.1.0` |
| Hardware Revision | `XIAO ESP32C6, 12V carrier v1.0` |

## What Gnimu-CAN fills

Capabilities **12** (vehicle and power). Every field not listed here is sent as 0 with its valid bit clear; the GNSS and IMU blocks' masks are 0.

| Frame field | Block, bit | Offset | Source | Valid when |
|---|---|---|---|---|
| Header | — | 0–7 | Device time, sequence, capabilities, state | Always |
| Engine speed | vehicle, 0 | 75 | OBD-II PID `0x0C` | Polling, and the car answered within the last second |
| Throttle position | vehicle, 1 | 77 | OBD-II PID `0x11` | Polling, and the car answered within the last second |
| Supply voltage | power, 0 | 134 | The supply ADC, through the carrier's ÷11 divider | Always, once the first reading is taken |

More vehicle fields come as the firmware polls more PIDs. Each new one fills its existing frame field, so the frame and any app setup already made don't change.

**State** in the header follows the device's [run states](architecture.md#run-states): 1 starting, 2 running (Gnimu-CAN's Polling state), 3 waiting for the vehicle, 4 fault. The protocol names code 2 "running" for every Gnimu device.

## Rate

Gnimu-CAN has no GNSS, so under the protocol's [rate rule](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md#rate) it sends at a fixed rate. While a client is subscribed, that's 20 Hz to start, with a target in the 30–50 Hz range once testing shows what's worthwhile (see [Emit rate](architecture.md#emit-rate)). Every frame carries the latest value of each field.

AutoXDL accepts frames at whatever rate the device sends them and aligns the values to its GNSS source's cadence, with each recorded row carrying the latest value.

## Setting up AutoX Data Logger

In **Settings → Custom Sensor**:

- **Service UUID:** `9074beb4-2f1a-4b5f-9818-fe2d2277ef05`
- **Notify UUID:** `0752d3c3-ceaf-462f-a944-bcc06c57284b`
- **Packet length:** **140**. Any other value and the app drops every frame.

Then add channels. Up to four are allowed today; the AutoXDL developer has said that limit can be raised on request.

| Name | Type | Byte offset | Scale |
|---|---|---|---|
| RPM | int16 | 75 | 1 |
| Throttle (%) | int16 | 77 | 10 |
| Supply (V) | uint16 | 134 | 100 |
| Device time (ms) | int32 | 0 | 1 |

Device time is optional, but worth recording: it lets a log be re-timed afterwards from the device's own clock.

The console's `P` command prints the same setup, laid out like the app's form:

```
Gnimu CAN Protocol
  Service         9074beb4-2f1a-4b5f-9818-fe2d2277ef05
  Notify          0752d3c3-ceaf-462f-a944-bcc06c57284b
  Packet length   140

  Name                Type    Offset  Scale
  Device time (ms)    int32        0      1
  RPM                 int16       75      1
  Throttle (%)        int16       77     10
  Supply voltage (V)  uint16     134    100
```

## Schema

The schema characteristic returns what the device fills, as defined by the frame spec:

```json
{"v":1,"d":"Gnimu-CAN","r":20,"fill":{"vehicle":3,"power":1}}
```

## Device info

The device info characteristic returns the build and current settings, in the [format the protocol defines](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md#device-info). Gnimu-CAN's `cfg` keys:

| Key | Example | Meaning |
|---|---|---|
| `hw` | `"12v"` | Hardware profile |
| `vehicle` | `"obd2"` | Vehicle profile |
| `can` | `"auto 500/250"` | CAN bit rates tried, in order |
| `probe_kbps` | `500` | Gateway probe bit rate |
| `emit_hz` | `20` | Frame rate |
| `run_v` | `13.20` | Engine-running threshold, V |
| `cutoff_v` | `11.80` | Low-voltage cutoff, V |
| `supply_cal` | `1.010` | Supply calibration factor |
| `pids` | `"0C,11"` | Mode 01 PIDs polled |

```json
{"v":1,"build":{"rev":"7f7efe5f4d6a","utc":"2026-10-30T17:27:43Z","target":"esp32c6"},
 "cfg":{"hw":"12v","vehicle":"obd2","can":"auto 500/250","probe_kbps":500,"emit_hz":20,"run_v":13.20,"cutoff_v":11.80,"supply_cal":1.010,"pids":"0C,11"}}
```

The `V` console command prints the same settings, as text.

## Limits

AutoXDL connects to one custom sensor at a time, so Gnimu-CAN has one service, and the app reads at most four of its fields at once. Gnimu-CAN itself accepts one connection at a time, as the protocol requires for v1, and stops advertising while connected.
