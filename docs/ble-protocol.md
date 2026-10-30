# Gnimu-CAN BLE protocol (draft v1)

> [!NOTE]
> **Draft.** The layout follows the AutoX Data Logger (AutoXDL) custom sensor model. One detail of that model is still unconfirmed; see [Open questions](#open-questions).

## What AutoXDL expects

AutoXDL's **Custom Sensor** setting describes a BLE device as:

- a **service UUID** and a **notify characteristic UUID**. The app's device scan only shows devices that advertise the service UUID entered there.
- a **packet length**, up to 244 bytes.
- **up to four channels**, each with a name, a type, a byte offset, and a scale.
  - Types: `uint8`, `int16`, `uint16`, `int32`, `float64`.
  - Scale: 1, 10, 100 or 1000. The value the app records is the raw value divided by the scale.
- **little-endian** byte order.

Confirmed with the AutoXDL developer:

- **The packet length must match exactly.** A notification of any other length is dropped. So the firmware's packet length is fixed, and changing it later means every user has to update their app setting.
- **Only the bytes the channel definitions point at are used.** Anything else in the packet is ignored.
- **The four-channel limit is arbitrary.** The developer will add more if asked.

The firmware is built around these points:

- The packet has **eight channel slots**, more than the app takes today, so the length never has to change when the channel limit goes up or channels are added.
- The slots are followed by a short status block that AutoXDL ignores but Gnimu's own tools can read.

## Service

The device advertises as **`Gnimu-CAN <device ID>`** and includes the service UUID in its advertising data, so the app's scan finds it.

| Item | UUID |
|---|---|
| Service | `9074beb4-2f1a-4b5f-9818-fe2d2277ef05` |
| Notify characteristic | `0752d3c3-ceaf-462f-a944-bcc06c57284b` |

While a client is subscribed, the device sends packets at a fixed rate. That rate starts at 20 Hz (one packet every 50 ms) and may be raised or lowered after testing. It's a firmware setting, and the packet layout doesn't depend on it. Every packet carries the latest value of each channel, whether or not the car has answered since the last packet.

## Packet

20 bytes, little-endian. Set the packet length in AutoXDL to **20**. Any other value and the app drops every packet. At 20 bytes it exactly fits the default BLE packet size, so it doesn't depend on the connection negotiating a larger one.

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | int16 | Slot 1 | Channel |
| 2 | int16 | Slot 2 | Channel |
| 4 | int16 | Slot 3 | Channel |
| 6 | int16 | Slot 4 | Channel |
| 8 | int16 | Slot 5 | Channel |
| 10 | int16 | Slot 6 | Channel |
| 12 | int16 | Slot 7 | Channel |
| 14 | int16 | Slot 8 | Channel |
| 16 | uint8 | `sequence` | Counts up by one per packet and wraps at 255. A gap means a packet was lost. |
| 17 | uint8 | `valid` | Bit *n* set: slot *n*+1 holds a value from the car (or the ADC). |
| 18 | uint8 | `fresh` | Bit *n* set: slot *n*+1 was updated since the previous packet. |
| 19 | uint8 | `state` | 1 = Running, 2 = WaitingForVehicle. |

v1 fills two slots:

| Slot | Offset | Channel | Type | AutoXDL scale |
|---|---|---|---|---|
| 1 | 0 | RPM | int16 | 1 |
| 2 | 2 | Throttle position (%) | int16 | 10 |
| 3–8 | 4–14 | Unused, always 0 | — | — |

All eight slots are always sent, even while some are unused. That way, adding a channel later doesn't change the packet length or move anything already configured in the app. Only the used slots need to be added as channels in AutoXDL. Until the app takes more than four channels, only slots 1–4 can be used with it.

Which channel goes in each slot is set in the firmware's `Config.h`. The boot log and the `V` command print the slot table with the type, offset and scale to enter in the app, so setting up AutoXDL is a matter of copying a few lines.

All eight slots are `int16`, even where `uint8` would fit, so every slot is configured the same way in the app and the status block's offsets never move.

### Channel encodings

| Channel | Source | Raw value | AutoXDL scale | Range |
|---|---|---|---|---|
| RPM | PID `0x0C` | rpm | 1 | 0–16,384 |
| Throttle position | PID `0x11` | 0.1 % | 10 | 0–1,000 |
| Engine load | PID `0x04` | 0.1 % | 10 | 0–1,000 |
| Vehicle speed | PID `0x0D` | km/h | 1 | 0–255 |
| Coolant temperature | PID `0x05` | 0.1 °C | 10 | −400–2,150 |
| Intake air temperature | PID `0x0F` | 0.1 °C | 10 | −400–2,150 |
| Supply voltage | ADC | 10 mV | 100 | 0–3,600 |

Supply voltage is in 10 mV steps, not millivolts, because 36,000 mV (the regulator's upper limit) doesn't fit in an `int16`.

### No value yet

Until the car has answered for a channel, and while the device is waiting for the vehicle, the slot holds **0** and its `valid` bit is clear. AutoXDL only sees the 0. A Gnimu tool can use the `valid` bit to tell a real zero from a missing value.

## Limits

AutoXDL connects to one custom sensor at a time, so the device has one service. The app currently takes four channels per sensor, but the developer has said that limit is arbitrary and can be raised on request, so more channels don't need a second service.

## Open questions

1. **Recording rate.** Working assumption: the app takes packets at whatever rate the device sends them and blends the values into its telemetry stream, as it does with OBD-II dongles. To be confirmed from a recorded VBO file. See [architecture.md](architecture.md#emit-rate).
