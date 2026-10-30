# Architecture (draft)

This page describes how the Gnimu-CAN firmware will be put together. It is a design draft: nothing below is implemented yet, and the [open questions](#open-questions) at the end still need answers. The skeleton currently in `code/GnimuCAN/` predates this design and will be replaced, not extended.

Gnimu-CAN reads vehicle data from CAN and streams it over BLE to a third-party app, the first being [AutoX Data Logger](https://autoxdrivermod.com), as a custom BLE sensor. It follows the shape of the [Gnimu](https://github.com/c20d-us/Gnimu) firmware: the same layers, fixed-rate scheduler, run-state machine, boot log, console and diagnostics.

The design goal is the same as Gnimu's: **consistency**. The app gets a packet at a fixed rate (e.g., 20 Hz) no matter how fast or slow the car answers, and the firmware makes it visible how fresh each value really is.

---

## Vehicle scope

The first version is **vehicle-agnostic**. It uses only standard OBD-II over CAN (ISO 15765-4) and Mode 01 PIDs, so it should work on most cars without per-vehicle setup. Vehicle-specific variants come later as [vehicle profiles](#vehicle-profiles).

What "most cars" means here:

- **Covered:** cars that speak OBD-II over CAN. In the US that's every car from model year 2008, and many from 2003–2007.
- **Not covered:** older cars that use the pre-CAN OBD-II protocols (ISO 9141, KWP2000 on K-line, SAE J1850). This hardware has no transceiver for them.
- **v1 limits:** 11-bit IDs only, at 500 or 250 kbit/s. The standard also allows 29-bit IDs, which some vehicles use. That's a later addition if a target car needs it.
- **Per-car differences:** not every car supports every PID. The boot-time support check drops what the car doesn't have, and says so in the boot log. RPM (`0x0C`) and throttle position (`0x11`) are among the most widely supported.

---

## Hardware

The first target is the **12V CAN carrier (XIAO) v1** from [Gnimu-PCB](https://github.com/c20d-us/Gnimu-PCB/tree/main/boards/gnimu-can), connected to the car through the OBD-II helper board.

| Part | Role |
|---|---|
| Seeed XIAO ESP32C6 | MCU. Its TWAI controller is the CAN controller. |
| Adafruit CAN Pal | CAN transceiver, powered from the XIAO's 3V3. |
| Pololu D24V7F5 | 12V to 5V buck regulator, feeding the XIAO's 5V pin through a Schottky diode. |
| OBD-II helper | Breaks out CAN_H, CAN_L, GND and +12V (OBD pin 16) from the plug. |

| XIAO pin | GPIO | Net | Use |
|---|---|---|---|
| D0/A0 | 0 | `V12_SENSE` | Switched 12V through a ÷11 divider (100kΩ + 10kΩ). 14.8V reads as 1.35V. |
| D8 | 19 | `CAN_SILENT` | CAN Pal S pin. R1 pulls it high, so the transceiver is **listen-only until the firmware drives it low**. |
| D9 | 20 | `CAN_TX` | TWAI TX |
| D10 | 18 | `CAN_RX` | TWAI RX |
| — | 15 | user LED | The XIAO's single yellow LED, active low. |

What follows from the hardware:

- **The device is powered whenever the car's battery is connected.** OBD pin 16 is unswitched. The carrier's switch (J3) can cut power by hand, but the firmware has no way to cut its own. Keeping the parked current low is a firmware job (see [Power](#power-and-sleep)).
- **Listen-only is the hardware default.** A blank or crashed MCU can't put anything on the car's bus. The firmware drives `CAN_SILENT` low only after it has confirmed the bit rate.
- **The firmware can see the vehicle's supply voltage.** That tells it whether the engine is running (alternator charging, about 13.5–14.8V) or the car is parked (about 12.0–12.7V), and warns it before it flattens the battery.

The LiPo carrier has the same CAN section, with `VBAT_SENSE` (÷2) on A0 instead. It would be a second hardware profile, as in Gnimu, with a battery power policy instead of the vehicle one. It's out of scope for v1.

There's also a **bench** hardware profile for the breadboard prototype. It's wired like the 12V carrier but powered from a 9V battery where the car's 12V would be, so the voltage thresholds don't fit. 9V is below the car's low-voltage cutoff, and a vehicle build would go straight to sleep. The bench profile changes only the power settings:

| Setting | 12V carrier | Bench |
|---|---|---|
| Engine-running threshold | 13.2V | 13.2V, so the 9V battery reads as parked |
| Low-voltage cutoff | 11.8V | 6.5V, about where a 9V battery is spent |
| Sleep when parked | After about 5 minutes | Off |
| `S` console command | Not available | Overrides the measured supply voltage |

`S 14.2` makes the firmware act as if it measured 14.2V, which is how the bench gets to "engine running" for the gateway probe, and how it tests the cutoff (`S 6`). `S` on its own goes back to the real reading. While an override is active, the status line and `D` report mark the voltage as simulated. The command exists only in bench builds, so a vehicle build can never be talked into probing a parked car.

---

## Layers

The same four layers as Gnimu, under `code/GnimuCAN/src/`, with the sketch and `Config.h` in `code/GnimuCAN/` as in Gnimu:

| Layer | What's in it |
|---|---|
| `core/` | Plain C++, host-tested: the scheduler, the run-state machine, the power policy, the OBD request/response codec, the poll planner, the channel cache, the BLE packet encoder, the report formatters. |
| `platform/` | The board: clock, watchdog, reset reason, hardware ID, the TWAI controller and silent pin, the supply ADC, the LED, deep sleep. ESP32-C6 only for now, but kept behind headers so `core/` stays testable. |
| `peripherals/` | The CAN bus driver on top of TWAI, and the BLE transport (NimBLE through the Arduino BLE library). |
| `app/` | The job table, the state handling, the console, the diagnostics. |

These Gnimu pieces carry over nearly unchanged and will be copied, not shared: `Scheduler`, `ConsoleEntry`, `Report`, `TextOut`, `StatusLine`, `DeviceId`, `BuildInfo`, `ResetReason`, `RateTracker`, `StatusLed`, and the build script's warnings-as-errors check.

---

## Data flow

```
CAN bus ──► can job ──► OBD poller ──► channel cache ──► emit job (fixed) ──► BLE notify
            (drain RX)   (requests,      (latest value,     (snapshot packet)
                          replies)        time, count)
supply ADC ─────────────────────────────►┘
```

Collecting and emitting are independent:

- The **poller** runs whenever the vehicle is awake, whether or not an app is connected, so the cache is already warm when one subscribes.
- The **emitter** runs only while a client is subscribed. At a fixed rate it reads the cache and sends one packet. It never waits on the bus.
- AutoXDL reads only the channel values. The packet's trailing status block carries which slots are valid and which were updated since the last packet, for Gnimu's own tools. The cache also tracks each channel's age and update rate for the console diagnostics.

### Emit rate

**20 Hz is a starting point, not a design limit.** The rate is one setting, `kEmitRateHz` in `Config.h`, and the `emit` job's period follows from it. The plan is to raise it as far as testing shows is worthwhile, or lower it if something can't keep up.

What sets the useful ceiling:

- **How fast the car answers.** Emitting faster than a channel updates only repeats values. The `fresh` bits and the per-channel rates in the diagnostics show where that point is. With two channels sharing a typical car's 20–60 replies per second, it's roughly 10–30 Hz per channel. Getting more real data means raising the poll rate: fewer channels, multi-PID requests, or a vehicle profile that reads broadcast frames instead of polling.
- **The BLE connection.** The phone sets the connection interval. iOS typically allows down to 15 ms, which is about 66 connection events per second. Going above roughly 50 Hz needs testing with the real phone and app.
- **What AutoXDL records.** The working assumption is that the app takes data at whatever rate the device sends it and blends it into its telemetry stream. With a Veepeak OBD-II dongle, which updates more slowly than the GNSS source, the app's VBO files show the OBD values repeating between updates. That suggests the recorded rows follow the GNSS source's rate (20–25 Hz with Gnimu), with each row carrying the latest OBD value. If so, emitting faster than the GNSS rate would make the values fresher in each row but wouldn't add rows. A VBO recorded with Gnimu-CAN will confirm it ([open question 1](#open-questions)).

The emitter keeps its rate fixed whatever happens upstream. That's the consistency goal: a higher rate is only worth it if it holds steady.

---

## CAN and OBD-II

### Bring-up

1. **Controller start, listen-only.** TWAI starts in listen-only mode while the transceiver is still held silent by R1. A failure here is a hardware fault.
2. **Bit-rate check.** The firmware listens for traffic at 500 kbit/s, then 250 kbit/s. Clean frames with no bus errors confirm the rate. This step only listens and never transmits.
3. **Go active.** It restarts TWAI in normal mode at the confirmed rate and drives `CAN_SILENT` low.
4. **ECU discovery.** It sends Mode 01 PID `0x00` to the functional address `0x7DF`. The first responder in `0x7E8`–`0x7EF` becomes the target, normally the engine ECU at `0x7E8`. From then on, requests go to that ECU's physical address (response ID − 8, usually `0x7E0`), so only one ECU answers.
5. **Supported PIDs.** It reads the support bitmaps (PIDs `0x00`, `0x20`, `0x40`, ...) and drops any configured channel the car doesn't support, saying so in the boot log.

If step 2 hears nothing, there are two possible reasons: the car is off, or the OBD port is behind a gateway that stays silent until a scan tool asks for something. Gatewayed ports are common from roughly the late 2010s on, the BMW F-series included. The supply voltage tells the two apart.

- **Supply below the engine-running threshold** (about 13.2V): the car is treated as off. The device goes to [WaitingForVehicle](#run-states) and keeps listening.
- **Supply above the threshold** for a few seconds, with a silent bus: the engine is running, so a gateway is the likely reason. The device makes a **gateway probe**:
  1. It drives `CAN_SILENT` low and starts TWAI in normal mode at the probe rate, 500 kbit/s (set in `Config.h`). Nearly every gatewayed car uses that rate.
  2. It sends one Mode 01 PID `0x00` request to `0x7DF` as a **single-shot** frame. The controller doesn't retransmit it, so a wrong rate produces at most one bad frame.
  3. A valid reply within the 50 ms timeout confirms the rate, and bring-up continues at step 4.
  4. If there's no reply, a missing acknowledgement, or any rise in the error counters, it stops TWAI, releases `CAN_SILENT`, logs the result, and goes to WaitingForVehicle.
  5. It doesn't probe again for 30 seconds, and stops probing after 5 failed tries until the next boot. A car that never answers sees at most 5 single frames per power cycle.

A gatewayed car with the ignition on but the engine off won't be found, because its voltage looks parked. Starting the engine brings it up. That's an acceptable limit for v1.

### Polling

- **One request in flight.** The poller sends a request and waits for its reply or for the 50 ms timeout (the ISO 15765-4 P2 limit) before sending the next. It never blocks: the `obd` job checks once a millisecond and returns.
- **Throughput is shared.** A car typically answers 20–60 single-PID requests per second. Every polled PID takes a share of that, so 20 Hz of real data for every channel isn't possible on most cars.
- **Tiered schedule.** Each channel has a target rate. Fast channels such as RPM, throttle and speed get most of the budget; slow ones such as temperatures get about 1 Hz. The planner picks whichever channel is most overdue relative to its target. The diagnostics report shows the rate each one actually got.
- **Single-frame replies only, for v1.** Every Mode 01 PID fits in one frame. Asking for several PIDs per request raises throughput a lot, but the replies span several frames and need ISO-TP flow control. That's a later step, and the poller's interface leaves room for it.

### Channels

The first build sends **RPM and throttle position**, replacing what an ELM327-style OBD-II dongle (such as the Veepeak OBDCheck BLE+) gives AutoXDL today. More channels come later. The packet has eight slots; AutoXDL reads four today, and its developer has stated that he could raise that on request.

| Channel | PID | Decode | Slot | In v1 |
|---|---|---|---|---|
| RPM | `0x0C` | (256A + B) / 4 | 1 | Yes |
| Throttle position | `0x11` | A × 100 / 255 % | 2 | Yes |
| Coolant temperature | `0x05` | A − 40 °C | — | Later |
| Engine load | `0x04` | A × 100 / 255 % | — | Later |
| Intake air temperature | `0x0F` | A − 40 °C | — | Later |
| Supply voltage | ADC, divider × 11 | — | — | Later in the packet. Measured from v1 for the power policy. |

`Config.h` has a channel table and a slot table. The poller only polls channels that are in a slot, so every request goes to data the app will actually receive. With two channels, each gets about half of the car's reply budget: roughly 10–30 Hz per channel on a typical car, which is the number the diagnostics will confirm.

### Vehicle profiles

Everything vehicle-specific sits behind a **vehicle profile**, chosen at build time the way Gnimu chooses its hardware and protocol profiles. v1 has one profile, `obd2`. A profile supplies:

- **How to reach the car:** bit rate, 11- or 29-bit IDs, request and response IDs, and the addressing scheme.
- **How to ask for each channel:** the service and identifier (Mode 01 PID, or a manufacturer-specific identifier), and how to decode the reply.
- **How to tell the car is awake,** for the bring-up and power logic.

The scheduler, cache, emitter, BLE packet, power policy, console and states don't change between profiles. A channel is a channel, wherever it came from.

What a manufacturer-specific profile is likely to need, and v1 leaves room for:

- **UDS `0x22` ReadDataByIdentifier** requests to a specific ECU, instead of Mode 01. This is how most manufacturers expose data beyond the standard PIDs.
- **ISO-TP multi-frame replies with flow control,** because manufacturer identifiers often return more than one frame of data.
- **Extended addressing,** where the target ECU's address goes in the first data byte. BMW's diagnostic CAN works this way, so a BMW F-series profile (such as the F87 M2) would need it.
- **Diagnostic sessions and tester-present keep-alives,** for identifiers that are only readable in an extended session.

The `obd2` profile also stays useful on those cars: Mode 01 PIDs answer on BMWs too, so the generic build works on the M2 before its own profile exists.

---

## BLE

The device advertises as `Gnimu-CAN` followed by the 10-digit device ID, the same scheme as Gnimu, with the service UUID in the advertising data so AutoXDL's scan finds it. It exposes one service with one notify characteristic, and sends a 20-byte packet at the [emit rate](#emit-rate) (20 Hz to start) while a client is subscribed: eight little-endian `int16` channel slots, followed by a status block (sequence, valid and fresh bits, state) that AutoXDL ignores. The schema, and the scale to enter in AutoXDL for each channel, are in [ble-protocol.md](ble-protocol.md).

The boot log and the `V` command print the slot table (name, type, offset, scale), so setting up the app means copying it from the console.

---

## Run states

| State | Meaning | Jobs running |
|---|---|---|
| **Starting** | Boot: subsystem checks, CAN bring-up, BLE start. | None yet |
| **WaitingForVehicle** | Hardware is fine, but the bus is silent or no ECU answers. Retries discovery every few seconds. | `can` (listen), `power`, `ble`, `emit`, `led`, `console` |
| **Running** | ECU found, polling. Streams when a client is subscribed. | All |
| **Fault** | A hardware or stack failure: TWAI won't start, BLE won't start, the ADC won't read. Doesn't retry. | `led`, `console` |
| **Sleep** | Car parked, so the device goes into deep sleep with a timer wake. Waking is a fresh boot. | None |

| From | Event | To |
|---|---|---|
| Starting | Hardware failed | Fault |
| Starting | Hardware fine, no vehicle | WaitingForVehicle |
| Starting | ECU found | Running |
| Running | No replies for 2 s | WaitingForVehicle |
| WaitingForVehicle | ECU found | Running |
| WaitingForVehicle | Parked conditions held (see below) | Sleep |
| Sleep | Timer wake | Starting (fresh boot) |

Unlike Gnimu, Running and WaitingForVehicle move back and forth without a restart. A car switching off and on again is normal operation, not a failure. Sleep, though, always comes back through a fresh boot, so the device still starts up only one way.

While waiting, a subscribed client still gets packets at the emit rate, so the app's connection stays up. CAN channels read 0, and the supply voltage, if it's in a slot, stays live.

---

## Power and sleep

Because the device runs from OBD pin 16, it has to stop drawing current on its own when the car is parked.

- **Parked** means the supply has been below the engine-running threshold and the bus has been silent, both for a set time (about 5 minutes to start with). The device then goes into ESP32-C6 deep sleep with a timer wake.
- **On a timer wake**, which is a fresh boot, it reads the supply voltage and listens to the bus briefly. If either shows the car is awake (voltage above the running threshold, or CAN traffic), it boots fully. Otherwise it sleeps again straight away.
- **Low-voltage cutoff.** Below a cutoff (about 11.8V, configurable) the device sleeps no matter what, with a longer wake interval, to protect the car's battery.
- CAN RX (GPIO18) isn't one of the C6's low-power GPIOs, so CAN traffic can't wake the chip directly. The timer wake is the only way back.

The parked current (regulator quiescent current plus XIAO deep sleep) has to be measured on the real board. It decides the wake interval and whether the hardware switch is necessary for long-term parking.

---

## Scheduler jobs

| Job | Every | Work |
|---|---|---|
| `can` | 1 ms | Drains TWAI RX into the poller, tracks bus state and error counters, recovers from bus-off. |
| `obd` | 1 ms | Sends the next due request, matches replies, handles timeouts, updates the cache. |
| `emit` | 1 / emit rate (50 ms at 20 Hz) | Encodes the snapshot and notifies, if a client is subscribed. |
| `power` | 100 ms | Reads the supply ADC, filters it, applies the power policy. |
| `ble` | 10 ms | BLE housekeeping. |
| `led` | 25 ms | Status LED. |
| `console` | 10 ms | Commands and the status line. |

Table order is priority, as in Gnimu: draining the bus comes first.

---

## Console and diagnostics

Same port, baud and commands as Gnimu: `D` report, `C` clear counters, `V` version and config, `L` status line, a number for N status lines, `R` restart, plus `S` in bench builds (see [the bench profile](#hardware)). Boot log in the same style:

```
BUILD,version=0.1.0,revision=...,dirty=0,utc=...,target=esp32c6
Device ID: 0301821570
Config: hardware 12v, CAN auto 500/250 kbit/s, OBD 2 channels
Config: BLE name "Gnimu-CAN 0301821570", 20 Hz
Gnimu-CAN starting
Reset: power-on
Watchdog: 5000 ms ✅
Supply: 14.2V ✅
CAN: controller started, listen-only ✅
CAN: traffic at 500 kbit/s ✅
CAN: active ✅
OBD: ECU 0x7E8 answered, polling 0x7E0 ✅
OBD: PID 0C RPM supported ✅
OBD: PID 11 throttle supported ✅
...
BLE: advertising as "Gnimu-CAN 0301821570" ✅
Job can: every 1000 us, budget 500 us
...
Running.
```

The `D` report adds a CAN and OBD section to Gnimu's job table: bus state and error counters, requests/s, replies/s, timeouts, and for each channel its target rate, the rate it actually got, and the age of its last value. The status line shows the state, the supply voltage, the reply rate, and the subscribed client's packet rate.

The status LED is the XIAO's single yellow LED, so it shows patterns only, like Gnimu's ESP32 build: slow blink while waiting for a client, solid while streaming, a different pattern for WaitingForVehicle, and fast blink in Fault.

---

## Testing

- **Host tests** for everything in `core/`: OBD request encoding and reply parsing (including the malformed and wrong-ECU cases), support-bitmap decoding, the poll planner's rate shares, the packet encoder's byte layout, the state machine and the power policy.
- **Build checks** with warnings as errors in the project's own code, as in Gnimu.
- **On hardware**: the bit-rate check against a real car, poll throughput, sleep current, and timing under load from the diagnostics job table.

### Bench rig

Two breadboards, joined by a CAN_H/CAN_L/GND pair:

- **Device:** a XIAO ESP32C6 and a CAN Pal wired as on the 12V carrier, with R1 on `CAN_SILENT` and the ÷11 divider on A0. A 9V battery on the 12V input powers it through the buck regulator, and the [bench profile](#hardware) adjusts the voltage thresholds to suit. Keep the Schottky diode (D2) between the regulator and the XIAO's 5V pin: the USB cable for the console will be plugged in at the same time.
- **Fake ECU:** a second XIAO ESP32C6 and CAN Pal running a small sketch (`tools/fake_ecu/`) that answers Mode 01 requests on `0x7E8` with changing RPM and throttle values. It has settings for a reply delay, so it can stand in for slow or fast ECUs, and for staying silent until asked, like a gatewayed port. It runs from a 5V plug-in supply on the XIAO's 5V pin through a Schottky diode (a 1N5819, like D2), so it's safe to have its USB cable plugged in too. It has no voltage divider or buck regulator.
- **Termination:** both CAN Pals' termination switches **on** for the bench, so the bus has a 120Ω end on each side as a car does. Both must be **off** before either is used in a car.

Without the fake ECU, the device alone still exercises boot, states, voltage sense, BLE and, with `S 14.2`, the gateway probe's failure path (no node to acknowledge it). With it, the whole OBD path can be tested: discovery, the gateway probe's success path, polling rates and the diagnostics.

---

## Open questions

1. **AutoXDL recording rate.** Working assumption: the app accepts any rate and blends the values into its stream, as it does with a Veepeak dongle. To confirm from a VBO file recorded with Gnimu-CAN: whether rows follow the GNSS rate, and whether emitting faster than it changes anything. This decides how high the [emit rate](#emit-rate) is worth going.
2. **Sleep thresholds and wake interval.** Set once the parked current is measured.
