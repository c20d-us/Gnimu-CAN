# Architecture

This page describes how the Gnimu-CAN firmware is put together. Everything here is built and bench-tested except [sleep and the low-voltage cutoff's action](#sleep-and-the-cutoff) and [vehicle profiles](#vehicle-profiles), which are still design; those sections say so. The [open questions](#open-questions) at the end still need answers.

Gnimu-CAN reads vehicle data from CAN and streams it over BLE to a third-party app, the first being [AutoX Data Logger](https://autoxdrivermod.com), as a custom BLE sensor. It follows the shape of the [Gnimu](https://github.com/c20d-us/Gnimu) firmware: the same layers, fixed-rate scheduler, run-state machine, boot log, console and diagnostics.

The design goal is the same as Gnimu's: **consistency**. The app gets a packet at a fixed rate (e.g., 20 Hz) no matter how fast or slow the car answers, and the firmware makes it visible how fresh each value really is.

---

## Vehicle scope

The first version is **vehicle-agnostic**. It uses only standard OBD-II over CAN (ISO 15765-4) and Mode 01 PIDs, so it should work on most cars without per-vehicle setup. Vehicle-specific variants come later as [vehicle profiles](#vehicle-profiles).

What "most cars" means here:

- **Covered:** cars that speak OBD-II over CAN. In the US that's every car from model year 2008, and many from 2003–2007.
- **Not covered:** older cars that use the pre-CAN OBD-II protocols (ISO 9141, KWP2000 on K-line, SAE J1850). This hardware has no transceiver for them.
- **v1 limits:** 11-bit IDs only, at 500 or 250 kbit/s. The standard also allows 29-bit IDs, which some vehicles use. That's a later addition if a target car needs it.
- **Per-car differences:** not every car supports every PID. The support check at ECU discovery drops what the car doesn't have, and says so in the log. RPM (`0x0C`) and throttle position (`0x11`) are among the most widely supported.

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
- **Listen-only is the hardware default.** A blank or crashed MCU can't put anything on the car's bus. The firmware drives `CAN_SILENT` low only once it has confirmed the bit rate, or for the single frame of a [gateway probe](#bring-up).
- **The firmware can see the vehicle's supply voltage.** That tells it whether the engine is running (alternator charging, about 13.5–14.8V) or the car is parked (about 12.0–12.7V), and warns it before it flattens the battery.
- **A USB console in the car needs the right order.** The XIAO's 5V pin is the USB connector's VBUS line, so a running device puts about 4.8V on it, and a USB-C host (such as a MacBook) won't attach to a VBUS that already has voltage on it. To connect a laptop to a device in the car:
  1. Switch the device off with the carrier's power switch (J3).
  2. Plug the USB cable into the laptop and the device.
  3. Switch the device back on.

  Plugging the laptop in before connecting the device to the OBD-II port works the same way. A USB-A port or hub on the laptop avoids the issue entirely. While both are connected, D2 keeps the laptop's 5V and the car's regulator apart.

The LiPo carrier has the same CAN section, with `VBAT_SENSE` (÷2) on A0 instead. It would be a second hardware profile, as in Gnimu, with a battery power policy instead of the vehicle one. It's out of scope for v1.

The breadboard prototype runs the 12V carrier build unchanged. An adjustable bench supply stands in for the car's voltage, so the power logic sees real readings through the real divider (see [Bench rig](#bench-rig)).

---

## Layers

The same layers as Gnimu, plus `protocol/`, under `code/GnimuCAN/src/`, with the sketch and `Config.h` in `code/GnimuCAN/` as in Gnimu:

| Layer | What's in it |
|---|---|
| `core/` | Plain C++, host-tested: the scheduler, the run-state machine, the supply monitor, the gateway probe policy, the OBD-II request/reply codec, the poller, vehicle activity, the LED patterns, the report and status-line formatters, and the BLE JSON documents. |
| `platform/` | The board: clock, watchdog, reset reason, hardware ID, the TWAI controller and silent pin (`CanPort`), the supply ADC, the LED, the console port. ESP32-C6 only for now, but kept behind headers so `core/` stays testable. Deep sleep joins it with the power work. |
| `peripherals/` | The BLE transport: the Gnimu service on NimBLE, through the Arduino BLE library. |
| `protocol/` | The Gnimu frame encoder and layout, the protocol repo's C++ reference, copied in. |
| `app/` | The job table, the state handling, CAN bring-up and polling, the console, the diagnostics. |

These Gnimu pieces carried over nearly unchanged, copied, not shared: `Scheduler`, `ConsoleEntry`, `Report`, `TextOut`, `StatusLine`, `DeviceId`, `BuildInfo`, `ResetReason`, `RateTracker`, `StatusLed`, and the build script's warnings-as-errors check.

---

## Data flow

```
CAN bus ──► can job ──────► OBD poller ──────────► emit job (fixed rate) ──► BLE notify
            (drain RX,       (one request in         (Gnimu frame)
             send requests)   flight; latest reply,
                              its time and counts)
supply ADC ──► power job ──────────────────────────►┘
```

Collecting and emitting are independent:

- The **poller** runs whenever an ECU answers, whether or not an app is connected, so the latest values are already there when one subscribes.
- The **emitter** runs only while a client is subscribed. At a fixed rate it takes the latest values and sends one packet. It never waits on the bus.
- The frame's valid mask says which fields hold a value. AutoXDL ignores it; Gnimu tools use it. The poller also keeps each channel's request, reply and timeout counts for the console diagnostics.

### Emit rate

**20 Hz is a starting point, not a design limit.** The rate is one setting, `kEmitRateHz` in `Config.h`, and the `emit` job's period follows from it. The plan is to raise it as far as testing shows is worthwhile, or lower it if something can't keep up. About 30 Hz is the likely upper end, a little above the GNSS rate for the reasons below.

What sets the useful ceiling:

- **How fast the car answers.** Emitting faster than a channel updates only repeats values. The per-channel rates in the diagnostics show where that point is. With two channels sharing a typical car's 20–60 replies per second, it's roughly 10–30 Hz per channel. Getting more real data means raising the poll rate: fewer channels, multi-PID requests, or a vehicle profile that reads broadcast frames instead of polling.
- **The BLE connection.** The phone sets the connection interval. iOS typically allows down to 15 ms, which is about 66 connection events per second. Going above roughly 50 Hz needs testing with the real phone and app.
- **What AutoXDL records.** Confirmed: the app accepts custom sensor data at whatever rate the device sends it, then aligns it to the GNSS source's cadence (20–25 Hz with Gnimu). Each recorded row carries the latest value received. Emitting faster than the GNSS rate doesn't add rows.

How old the value in a row is has three parts:

- **Poll age:** how long ago the car answered. With a channel updating at *P* Hz, this averages about 1/(2*P*): 17–50 ms at 10–30 Hz. **This is the big one.**
- **Emit age:** how long ago the latest packet left the device. At emit rate *E*, it averages 1/(2*E*): 25 ms at 20 Hz, 20 ms at 25 Hz, 10 ms at 50 Hz.
- **BLE delivery:** packets arrive at the phone in the connection's events (every 15–30 ms or so), which adds a similar amount whatever *E* is.

So raising the emit rate shaves only a few milliseconds off the average age. The real reason to run **a little above** the GNSS rate is different. The device's clock and the GNSS receiver's clock aren't synchronised. At exactly the same nominal rate, the two drift past each other, and now and then a GNSS interval gets no new packet, so that row repeats the previous value. A margin of a few Hz makes that rare. The real gains in freshness come from polling the car faster.

The emitter keeps its rate fixed whatever happens upstream. That's the consistency goal: a higher rate is only worth it if it holds steady.

---

## CAN and OBD-II

### Bring-up

1. **Controller start, listen-only.** TWAI starts in listen-only mode while the transceiver is still held silent by R1. A failure here is a hardware fault.
2. **Bit-rate check.** The firmware listens for traffic at 500 kbit/s, then 250 kbit/s: 300 ms each at boot, then 2 s windows alternating between the two while it waits. Clean frames with no bus errors confirm the rate. This step only listens and never transmits.
3. **Go active.** It restarts TWAI in normal mode at the confirmed rate and drives `CAN_SILENT` low.
4. **ECU discovery.** It sends Mode 01 PID `0x00` to the functional address `0x7DF`. The first responder in `0x7E8`–`0x7EF` becomes the target, normally the engine ECU at `0x7E8`. From then on, requests go to that ECU's physical address (response ID − 8, usually `0x7E0`), so only one ECU answers.
5. **Supported PIDs.** The PID `0x00` reply is the support bitmap for PIDs `0x01`–`0x20`. Any polled channel the car doesn't list is dropped, and the log says so. v1's channels are all in that range; reading the later bitmaps (`0x20`, `0x40`, ...) comes with channels that need them.

If no ECU answers discovery, the device goes back to listening and asks again every 5 seconds. After 3 unanswered tries it starts over from the bit-rate check.

If step 2 hears nothing, there are two possible reasons: the car is off, or the OBD port is behind a gateway that stays silent until a scan tool asks for something. Gatewayed ports are common from roughly the late 2010s on, the BMW F-series included. The supply voltage is the only hint, and a weak one:

- **Supply below the probe threshold** (12.4V, set in `Config.h`): the car is treated as off. The device goes to [WaitingForVehicle](#run-states) and keeps listening.
- **Supply at or above it** for a few seconds, with a silent bus: the car may be on behind a gateway, so the device makes a **gateway probe**:
  1. It drives `CAN_SILENT` low and starts TWAI in normal mode at the probe rate, 500 kbit/s (set in `Config.h`). Nearly every gatewayed car uses that rate.
  2. It sends one Mode 01 PID `0x00` request to `0x7DF` as a **single-shot** frame. The controller doesn't retransmit it, so a wrong rate produces at most one bad frame.
  3. A valid reply within the 50 ms timeout confirms the rate, and bring-up continues at step 3.
  4. Otherwise it stops TWAI, releases `CAN_SILENT`, logs the result ("no acknowledgement", "acknowledged, no reply" or "controller error"), and keeps listening in WaitingForVehicle.
  5. It doesn't probe again for 30 seconds, and stops probing after 5 failed tries until the next boot. A car that never answers sees at most 5 single frames per power cycle.

Neither the probe nor discovery waits inside a job. Each sends its request and returns. Later `can` runs, every 10 ms, check whether the controller has reported the frame sent, take the reply when it arrives, or give up at the timeout. Until the step finishes, the rate check and probe policy stay paused. The first bench test of the blocking version showed why this matters: an unanswered probe held the loop for 50 ms, and BLE, the LED and the console all waited behind it.

**Why 12.4V and not the 13.2V engine-running threshold.** Many modern cars, the BMW F-series among them, use intelligent alternator control: the alternator often stops charging while the engine runs, and the supply sits around 12.4–12.8V. A 13.2V trigger would miss those cars. The cost is that a healthy parked battery (about 12.6–12.8V) also passes, so a parked gatewayed car can see a probe. The limit of 5 single frames per boot is what bounds that. The first car test logs the supply with the engine running, so the threshold can be adjusted to suit.

A gatewayed car with the ignition on but the engine off won't be found, because its voltage looks parked. Starting the engine brings it up. That's an acceptable limit for v1.

### Polling

- **One request in flight.** The poller sends a request and waits for its reply or for the 50 ms timeout (the ISO 15765-4 P2 limit) before sending the next. It never blocks: the `can` job runs every 10 ms, takes any reply, and sends the next request. That caps polling at about 100 requests per second, above what most cars answer; a faster job period is the change if a car ever outruns it.
- **Vehicle lost.** No reply on any channel for 2 seconds: the device logs it, goes back to listening, and returns to WaitingForVehicle. The gateway probe budget starts over, so a gatewayed car that comes back later in the same boot is found again.
- **Throughput is shared.** A car typically answers 20–60 single-PID requests per second. Every polled PID takes a share of that, so 20 Hz of real data for every channel isn't possible on most cars.
- **Tiered schedule.** Each channel has a target rate, and the poller picks whichever channel is most overdue relative to its own target. v1's two channels both aim for 25 Hz. As channels are added, fast ones such as RPM, throttle and speed get most of the budget, and slow ones such as temperatures about 1 Hz. The status line shows the rate each one actually gets.
- **Bus-off.** Not handled on its own: a controller that goes bus-off stops getting replies, so after 2 s the vehicle-lost path restarts it in listen-only mode.
- **Single-frame replies only, for v1.** Every Mode 01 PID fits in one frame. Asking for several PIDs per request raises throughput a lot, but the replies span several frames and need ISO-TP flow control. That's a later step, and the poller's interface leaves room for it.

### Channels

The first build sends **RPM and throttle position**, replacing what an ELM327-style OBD-II dongle (such as the Veepeak OBDCheck BLE+) gives AutoXDL today. More channels come later. Each one already has its field in the [Gnimu frame](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md), so adding one never changes the frame.

| Channel | PID | Decode | Frame field | In v1 |
|---|---|---|---|---|
| RPM | `0x0C` | (256A + B) / 4 | Engine speed (vehicle bit 0) | Yes |
| Throttle position | `0x11` | A × 100 / 255 % | Throttle position (vehicle bit 1) | Yes |
| Supply voltage | ADC, divider × 11 | — | Supply voltage (power bit 0) | Yes, from the ADC, not the bus |
| Coolant temperature | `0x05` | A − 40 °C | Coolant temperature (vehicle bit 4) | Later |
| Intake air temperature | `0x0F` | A − 40 °C | Intake air temperature (vehicle bit 5) | Later |
| Ambient air temperature | `0x46` | A − 40 °C | Ambient air temperature (vehicle bit 6) | Later |

Each channel's target period is in `Config.h` (40 ms, 25 Hz, for both to start). The channel list itself, PID, decode and frame field, is in `App.cpp` for v1; it becomes a table when vehicle profiles arrive. A value older than 1 second is left out of the frame (its valid bit clear) rather than sent stale. With two channels, each gets about half of the car's reply budget: roughly 10–30 Hz per channel on a typical car, which is the number the diagnostics will confirm.

### Vehicle profiles

*Design, not built yet.* Everything vehicle-specific sits behind a **vehicle profile**, chosen at build time the way Gnimu chooses its hardware and protocol profiles. v1 has one profile, `obd2`. A profile supplies:

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

The device advertises as `Gnimu-CAN` followed by the 10-digit device ID, the same scheme as Gnimu, with the service UUID in the advertising data so AutoXDL's scan finds it. It sends only the 140-byte [Gnimu frame](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md), the layout shared by the whole Gnimu family, at the [emit rate](#emit-rate) (20 Hz to start) while a client is subscribed, to one client at a time. It fills the header, the vehicle fields it polls and the supply voltage, and sends the GNSS and IMU blocks as zeros. The encoder is the protocol repo's C++ reference, copied into `src/`. The UUIDs, the fields it fills and the AutoXDL setup are in [ble-protocol.md](ble-protocol.md).

The console's `P` command prints the protocol setup (service, notify characteristic, packet length, and each filled field's type, offset and scale), so setting up an app means copying it from the console.

---

## Run states

| State | Meaning | Jobs running |
|---|---|---|
| **Starting** | Boot: supply reading, CAN bring-up, BLE start. | None yet |
| **WaitingForVehicle** | Hardware is fine, but no ECU is answering yet: the bus is silent, or discovery is under way or being retried. | All |
| **Polling** | An ECU answers, and the vehicle fields are being polled. Streams when a client is subscribed. | All |
| **Fault** | The CAN controller or BLE failed to start. Doesn't retry; `R` restarts. | `power`, `led`, `console` |
| **Sleep** | *Not built yet.* Car parked, so the device goes into deep sleep with a timer wake. Waking is a fresh boot. | None |

| From | Event | To |
|---|---|---|
| Starting | Hardware failed | Fault |
| Starting | Hardware fine | WaitingForVehicle |
| WaitingForVehicle | ECU found | Polling |
| Polling | No replies for 2 s | WaitingForVehicle |
| WaitingForVehicle | Parked conditions held (see below; not built yet) | Sleep |
| Sleep | Timer wake | Starting (fresh boot) |

Starting always goes through WaitingForVehicle, even when traffic is heard at boot: discovery runs in the `can` job, so the device is waiting until the ECU answers, a few milliseconds later.

The state is called Polling, not Running, because it's about whether the device can talk to the car, not whether the engine runs: an ECU answers with the ignition on and the engine off too. Whether the engine runs is the separate [vehicle activity](#vehicle-activity). The frame header's state code for it stays "running" (2), since that code describes the device.

Unlike Gnimu, Polling and WaitingForVehicle move back and forth without a restart. A car switching off and on again is normal operation, not a failure. Sleep, though, always comes back through a fresh boot, so the device still starts up only one way.

While waiting, a subscribed client still gets packets at the emit rate, so the app's connection stays up. The vehicle fields read 0 with their valid bits clear, the header's state says waiting, and the supply voltage stays live.

---

## Power and sleep

Because the device runs from OBD pin 16, it has to stop drawing current on its own when the car is parked.

### Vehicle activity

Separate from the run state (can the device talk to the car?), the device keeps track of what the car is doing, from the best signal it has:

| Signal available | Activity |
|---|---|
| **RPM, fresh** (answered within the last second) | **Engine running** at 400 RPM or more, past cranking speed. **Ignition on** below that: an ECU answers, but the engine isn't turning. |
| **No fresh RPM** (bus silent, or no ECU) | **Engine running** with the supply at 13.2V or more for 3 s. **Parked** below that, or below the cutoff. |
| Neither (no supply reading yet) | **Unknown** |

The console logs each change as `Vehicle: …`, and the status line and `D` report show it. RPM overrides the supply voltage whenever it's fresh. That matters on cars with intelligent alternator control, whose supply can sit at 12.4–12.8V with the engine running: voltage alone would call that parked. Voltage is the fallback for when the device can't reach an ECU, which includes every parked car.

### Sleep and the cutoff

*Design, not built yet.* For now the cutoff is only reported: the log says `Power: below cutoff - sleep not implemented yet`, and the device carries on.

- **Parked** means no fresh RPM, the supply below the engine-running threshold, the bus silent, and no client connected, all for a set time (about 5 minutes to start with). The device then goes into ESP32-C6 deep sleep with a timer wake.
- **On a timer wake**, which is a fresh boot, it reads the supply voltage and listens to the bus briefly. If either shows the car is awake (voltage above the running threshold, or CAN traffic), it boots fully. Otherwise it sleeps again straight away.
- **Low-voltage cutoff.** Below a cutoff (about 11.8V, configurable) the device sleeps no matter what, with a longer wake interval, to protect the car's battery. It has **hysteresis**: once below the cutoff, the supply must rise back above about 12.1V to count as recovered, so a supply hovering at the cutoff, with ADC noise of about ±0.08V, can't flip the device in and out of sleep.
- CAN RX (GPIO18) isn't one of the C6's low-power GPIOs, so CAN traffic can't wake the chip directly. The timer wake is the only way back.

The parked current (regulator quiescent current plus XIAO deep sleep) has to be measured on the real board. It decides the wake interval and whether the hardware switch is necessary for long-term parking.

---

## Scheduler jobs

| Job | Every | Work |
|---|---|---|
| `can` | 10 ms | Drains TWAI RX. Waiting: checks the bit rate, makes gateway probes, runs ECU discovery. Polling: matches replies, times out the request in flight, sends the next, and notices the car going quiet. |
| `emit` | 1 / emit rate (50 ms at 20 Hz) | Encodes a Gnimu frame from the latest values and notifies, if a client is subscribed. |
| `power` | 100 ms | Reads the supply ADC, filters it, feeds the probe policy, and logs supply and vehicle-activity changes. |
| `ble` | 10 ms | BLE housekeeping. |
| `led` | 25 ms | Status LED. |
| `console` | 10 ms | Commands and the status line. |

Table order is priority, as in Gnimu: draining the bus comes first.

---

## Console and diagnostics

Same port, baud and commands as Gnimu: `D` report, `C` clear counters, `V` version and config, `L` status line, a number for N status lines, `R` restart. Plus `P`, which prints the BLE protocol setup.

**Console output never blocks.** The console is the C6's built-in USB serial port. With a cable attached but no terminal reading, its transmit buffer fills, and by default every write then waits 100 ms for room, which stalls the whole loop: polling, BLE frames, everything. The firmware sets the port's transmit timeout to 0 (`Serial.setTxTimeoutMs(0)`), so output nobody is reading is dropped instead. Found on the bench with `board_check`, whose LED froze whenever USB was connected without a monitor open.

Boot log in the same style:

```
BUILD,version=0.1.0,revision=...,utc=...,target=esp32c6
Device ID: 0301821570
Config: hardware 12v, vehicle obd2, CAN auto 500/250 kbit/s, gateway probe at 12.40V
Config: BLE "Gnimu-CAN 0301821570", Gnimu frame v1, 140 bytes at 20 Hz
Config: supply calibration 1.010, engine running 13.20V, cutoff 11.80V
Gnimu-CAN starting
Reset: power-on
Watchdog: 5000 ms ✅
Supply: 14.20V, engine running
CAN: controller started, listen-only ✅
CAN: traffic at 500 kbit/s ✅
BLE: advertising as "Gnimu-CAN 0301821570" ✅
Job can: every 10000 us, budget 2000 us
...
Waiting for vehicle.
Commands end with Enter: ...
OBD: ECU 0x7E8 answered, polling via 0x7E0 ✅
OBD: PID 0C RPM supported ✅
OBD: PID 11 Throttle supported ✅
Vehicle: engine running
```

The `D` report adds CAN and OBD lines to Gnimu's job table: bus state and error counters, the ECU being polled, and for each channel its latest value with its request, reply and timeout counts, then the [vehicle activity](#vehicle-activity). The status line shows the state, the supply voltage, each channel's value and reply rate (replies per second since the previous line), the vehicle activity, and the subscribed client's packet rate.

The status LED is the XIAO's single yellow LED, so it shows patterns only, like Gnimu's ESP32 build:

| Pattern | Meaning |
|---|---|
| A short flash every 2 s | WaitingForVehicle |
| Slow blink (1 s on, 1 s off) | Polling, no client |
| Solid | Polling, client connected |
| Fast blink | Fault |

---

## Testing

- **Host tests** (`tests/run.sh`) for `core/`: the scheduler, rate tracking, device ID, console input, the supply monitor, the state machine, the LED patterns, the report, status-line and JSON text, OBD request encoding and reply parsing (including the malformed and wrong-ECU cases), the probe policy, the poller (one in flight, timeouts, support bitmap, rate shares) and vehicle activity. Each runs plain and under AddressSanitizer and UndefinedBehaviorSanitizer.
- **Build checks** with warnings as errors in the project's own code, as in Gnimu.
- **On hardware**: the bit-rate check against a real car, poll throughput, sleep current, and timing under load from the diagnostics job table.

### Bench rig

Two breadboards, joined by a CAN_H/CAN_L/GND pair:

- **Device:** a XIAO ESP32C6 and a CAN Pal wired as on the 12V carrier, with R1 on `CAN_SILENT` and the ÷11 divider on A0. An adjustable bench supply (0–30V) on the 12V input powers it through the buck regulator and plays the car's voltage: about 12.4V parked, about 14.2V engine running, below 11.8V for the low-voltage cutoff, and anything in between to test the transitions. Set its current limit to about 150–200 mA. Keep below about 16V if the TVS diode (D1) is fitted, since it starts conducting around 17V. Keep the Schottky diode (D2) between the regulator and the XIAO's 5V pin: the USB cable for the console will be plugged in at the same time.

  **Connect USB before turning the supply on.** The XIAO's 5V pin is the USB connector's VBUS line, so with the supply on, about 4.8V sits on VBUS before a cable is attached. A USB-C host (such as a MacBook) checks that VBUS is at 0V before it attaches, sees the voltage, and doesn't enumerate the board. In the other order it attaches normally and stays attached. A USB-A port or hub doesn't make that check. Found on the bench: the board appeared the moment the supply was switched off. The same applies to the 12V carrier if a USB-C cable is plugged into a board already running from the car.
- **Fake ECU:** a second XIAO ESP32C6 and CAN Pal running a small sketch (`tools/fake_ecu/`) that answers Mode 01 requests on `0x7E8` with changing RPM and throttle values. It has settings for a reply delay, so it can stand in for slow or fast ECUs, and for staying silent until asked, like a gatewayed port. It runs from a 5V plug-in supply on the XIAO's 5V pin through a Schottky diode (a 1N5819, like D2), so it's safe to have its USB cable plugged in too. It has no voltage divider or buck regulator.
- **Termination:** both CAN Pals' termination switches **on** for the bench, so the bus has a 120Ω end on each side as a car does. Both must be **off** before either is used in a car.

Without the fake ECU, the device alone still exercises boot, states, voltage sense, BLE and, with the supply at or above the 12.4V probe threshold, the gateway probe's failure path (no node to acknowledge it). With it, the whole OBD path can be tested: discovery, the gateway probe's success path, polling rates and the diagnostics.

---

## Open questions

1. **Sleep thresholds and wake interval.** Set once the parked current is measured.
